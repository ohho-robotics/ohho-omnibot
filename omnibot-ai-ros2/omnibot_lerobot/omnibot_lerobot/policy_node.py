#!/usr/bin/env python3
"""Model-agnostic visuomotor policy inference node for OmniBot.

Loads any registered policy backend (SmolVLA, ACT, Diffusion, OpenVLA, …)
and runs it at the configured rate. Swap models by changing the model_type
parameter — no code changes required.

Unified 9-DOF action space: [arm(6), base_vx, base_vy, base_vz]
  arm[:6]   → /arm/joint_commands  (JointState, routes via arm_cmd_mux)
  base[6:9] → /cmd_vel/vla         (Twist, routed by cmd_vel_mux in "vla" mode)

Parameters
----------
model_type           : str   — registry name: "smolvla" | "act" | "diffusion" | "openvla" | "serve"
checkpoint_path      : str   — HuggingFace hub ID or local path (``hf://…`` for Serve)
serve_url            : str   — OhhO Serve base URL when model_type is "serve"
serve_timeout_sec    : float — HTTP timeout for Serve /predict (default 0.5)
serve_api_key        : str   — optional X-API-Key for Serve
device               : str   — "cuda" | "cpu"
policy_hz            : float — inference rate (Hz)
state_dim            : int   — proprioceptive state dimension (default 9)
action_dim           : int   — action dimension (default 9)
image_width          : int   — resize target width (default 320)
image_height         : int   — resize target height (default 240)
task_description     : str   — default task for language-conditioned models
base_vel_scale       : float — scale factor applied to base velocity outputs
use_trt              : bool  — enable TRT vision encoder patch
trt_engine_path      : str   — path to .trt engine built by vla_engine.trt.build_engine
publish_diagnostics  : bool  — publish timing to /diagnostics at 1 Hz

Topics subscribed
-----------------
/camera/wrist/image_raw     (sensor_msgs/Image)
/camera/base/bev/image_raw  (sensor_msgs/Image)
/arm/joint_states           (sensor_msgs/JointState)
/odom                       (nav_msgs/Odometry)
/policy/task                (std_msgs/String)   — update task description
/policy/enable              (std_msgs/Bool)     — enable/disable inference
/policy/load                (std_msgs/String)   — checkpoint path; does not enable inference

Topics published
----------------
/arm/joint_commands  (sensor_msgs/JointState)
/cmd_vel             (geometry_msgs/Twist)     — remapped to /cmd_vel/vla by launch file
"""

import collections
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool

try:
    from omnibot_lerobot.policy_guard import (
        MAX_JOINT_DELTA_RAD,
        PolicyGuard,
        is_black_frame,
    )
except ImportError:
    from policy_guard import (
        MAX_JOINT_DELTA_RAD,
        PolicyGuard,
        is_black_frame,
    )

try:
    from omnibot_lerobot.policy_load import requested_checkpoint
except ImportError:
    from policy_load import requested_checkpoint

try:
    from cv_bridge import CvBridge

    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

try:
    import cv2

    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    import torch

    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    from lerobot_engine.models import make_policy

    REGISTRY_AVAILABLE = True
except ImportError:
    REGISTRY_AVAILABLE = False

try:
    from vla_engine.trt import patch_policy_vision_encoder

    TRT_PATCH_AVAILABLE = True
except ImportError:
    TRT_PATCH_AVAILABLE = False

ARM_JOINT_NAMES = [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
]


# ---------------------------------------------------------------------------
# Fallback when lerobot_engine.models is not on PYTHONPATH
# ---------------------------------------------------------------------------


class _DummyAdapter:
    """Zero-action fallback when no model backend is available."""

    image_keys = ["observation.images.wrist", "observation.images.bev"]
    state_key = "observation.state"
    task_key = "task"
    action_dim = 9
    image_size = (320, 240)

    def reset(self):
        pass

    def select_action(self, obs):
        return np.zeros(9, dtype=np.float32)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------


class PolicyNode(Node):
    """Model-agnostic VLA inference node."""

    def __init__(self):
        super().__init__("policy_node")

        # Parameters
        self.declare_parameter("model_type", "smolvla")
        self.declare_parameter("checkpoint_path", "lerobot/smolvla_base")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("policy_hz", 10.0)
        self.declare_parameter("state_dim", 9)
        self.declare_parameter("action_dim", 9)
        self.declare_parameter("image_width", 320)
        self.declare_parameter("image_height", 240)
        self.declare_parameter("task_description", "pick up the object and place it")
        self.declare_parameter("base_vel_scale", 0.3)
        self.declare_parameter("publish_diagnostics", False)
        self.declare_parameter("use_trt", False)
        self.declare_parameter("trt_engine_path", "")
        self.declare_parameter("use_depth", False)
        # Max joint-target change per cycle (same 0.15 rad limit as arm driver)
        self.declare_parameter("max_joint_delta_rad", MAX_JOINT_DELTA_RAD)
        self.declare_parameter(
            "joint_min", [-3.14, -1.57, -1.57, -1.57, -3.14, -0.1]
        )
        self.declare_parameter(
            "joint_max", [3.14, 1.57, 1.57, 1.57, 3.14, 0.8]
        )
        self.declare_parameter("serve_url", "http://127.0.0.1:8000")
        self.declare_parameter("serve_timeout_sec", 0.5)
        self.declare_parameter("serve_api_key", "")

        model_type = self.get_parameter("model_type").value
        checkpoint = self.get_parameter("checkpoint_path").value
        device_str = self.get_parameter("device").value
        self.policy_hz = self.get_parameter("policy_hz").value
        self.state_dim = self.get_parameter("state_dim").value
        self.action_dim = self.get_parameter("action_dim").value
        self.image_width = self.get_parameter("image_width").value
        self.image_height = self.get_parameter("image_height").value
        self.task_description = self.get_parameter("task_description").value
        self.base_vel_scale = self.get_parameter("base_vel_scale").value
        self._diag_enabled = self.get_parameter("publish_diagnostics").value
        self.use_trt = self.get_parameter("use_trt").value
        self.trt_engine_path = self.get_parameter("trt_engine_path").value
        self.use_depth = self.get_parameter("use_depth").value
        self.max_joint_delta_rad = float(
            self.get_parameter("max_joint_delta_rad").value
        )
        self.joint_min = [
            float(x) for x in self.get_parameter("joint_min").value
        ]
        self.joint_max = [
            float(x) for x in self.get_parameter("joint_max").value
        ]
        self.policy_period = (
            1.0 / self.policy_hz if self.policy_hz > 0 else 0.10
        )

        # Timing accumulators
        self._t_preprocess = collections.deque(maxlen=100)
        self._t_inference = collections.deque(maxlen=100)
        self._t_total = collections.deque(maxlen=100)

        # Sensor cache (protected by _cam_lock for thread safety)
        self.enabled = False
        self._cam_lock = threading.Lock()
        self.camera_images: dict[str, np.ndarray | None] = {}
        self.arm_positions = np.zeros(6, dtype=np.float32)
        self.base_vel = np.zeros(3, dtype=np.float32)

        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn(
                "cv_bridge not available — camera images will be missing."
            )

        # Device
        if TORCH_AVAILABLE:
            if device_str == "cuda" and torch.cuda.is_available():
                self.device = torch.device("cuda")
            else:
                self.device = torch.device("cpu")
                if device_str == "cuda":
                    self.get_logger().warn("CUDA unavailable, falling back to CPU.")
        else:
            self.device = None

        # Load policy. /policy/load may replace the checkpoint later.
        # A failed reload keeps this adapter. Load never enables inference.
        self._model_type = model_type
        self._checkpoint = checkpoint
        self._device_str = device_str
        self._serve_url = self.get_parameter("serve_url").value
        self._serve_timeout_sec = float(self.get_parameter("serve_timeout_sec").value)
        self._serve_api_key = self.get_parameter("serve_api_key").value
        self.adapter = self._load_adapter(
            self._model_type,
            self._checkpoint,
            self._device_str,
            serve_url=self._serve_url,
            serve_timeout_sec=self._serve_timeout_sec,
            serve_api_key=self._serve_api_key,
        )

        # Safety guard for camera frame validation, inference latency, and arm clamping
        self.guard = PolicyGuard(
            policy_period=self.policy_period,
            max_joint_delta_rad=self.max_joint_delta_rad,
            required_keys=self.adapter.image_keys,
            joint_min=self.joint_min,
            joint_max=self.joint_max,
            base_vel_scale=self.base_vel_scale,
        )

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            JointState, "/arm/joint_commands", 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Camera subscribers — driven by adapter.image_keys so any model works
        for key in self.adapter.image_keys:
            # Map key → ROS topic
            topic = self._key_to_topic(key)
            self.camera_images[key] = None
            self.create_subscription(
                Image,
                topic,
                lambda msg, k=key: self._image_cb(msg, k),
                10,
            )

        if self.use_depth:
            self.create_subscription(
                Image, "/camera/depth/image_raw", self._depth_cb, 10
            )

        self.create_subscription(
            JointState, "/arm/joint_states", self._arm_state_cb, 10
        )
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(String, "/policy/task", self._task_cb, 10)
        self.create_subscription(Bool, "/policy/enable", self._enable_cb, 10)
        self.create_subscription(String, "/policy/load", self._policy_load_cb, 10)

        # Inference timer
        self.create_timer(1.0 / self.policy_hz, self._inference_loop)

        if self._diag_enabled:
            from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

            self._DiagArray = DiagnosticArray
            self._DiagStatus = DiagnosticStatus
            self._KeyValue = KeyValue
            self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f"PolicyNode started | model={model_type} | device={self.device} "
            f"| hz={self.policy_hz} | image_keys={self.adapter.image_keys}"
        )

    # ------------------------------------------------------------------
    # Policy loading
    # ------------------------------------------------------------------

    def _load_adapter(
        self,
        model_type: str,
        checkpoint: str,
        device_str: str,
        *,
        serve_url: str = "http://127.0.0.1:8000",
        serve_timeout_sec: float = 0.5,
        serve_api_key: str = "",
    ):
        if not REGISTRY_AVAILABLE:
            self.get_logger().warn(
                "lerobot_engine not on PYTHONPATH — using zero-action DummyAdapter. "
                "Add /path/to/OmniBot to PYTHONPATH to enable real inference."
            )
            return _DummyAdapter()

        try:
            self.get_logger().info(f'Loading "{model_type}" from "{checkpoint}"...')
            extra = {}
            if model_type == "serve":
                extra = {
                    "serve_url": serve_url,
                    "timeout_sec": serve_timeout_sec,
                    "api_key": serve_api_key,
                }
            adapter = make_policy(
                model_type,
                checkpoint=checkpoint,
                device=device_str,
                **extra,
            )
            self.get_logger().info(f"Policy loaded. image_keys={adapter.image_keys}")

            if self.use_trt and self.trt_engine_path:
                self._apply_trt(adapter)

            return adapter
        except Exception as exc:
            self.get_logger().error(f"Policy load failed: {exc} — using DummyAdapter.")
            return _DummyAdapter()

    def _apply_trt(self, adapter) -> None:
        if not TRT_PATCH_AVAILABLE:
            self.get_logger().error(
                "use_trt=True but vla_engine.trt not importable. "
                "Install tensorrt and add vla_engine to PYTHONPATH."
            )
            return
        if adapter.model is None:
            self.get_logger().error("use_trt=True but adapter.model is None.")
            return
        try:
            patch_policy_vision_encoder(adapter.model, self.trt_engine_path)
            self.get_logger().info(f"TRT encoder patch applied: {self.trt_engine_path}")
        except Exception as exc:
            self.get_logger().error(f"TRT patch failed: {exc}")

    # ------------------------------------------------------------------
    # Topic key → ROS topic mapping
    # ------------------------------------------------------------------

    def _key_to_topic(self, key: str) -> str:
        _MAP = {
            "observation.images.wrist": "/camera/wrist/image_raw",
            "observation.images.bev": "/camera/base/bev/image_raw",
            "observation.images.depth": "/camera/depth/image_raw",
        }
        return _MAP.get(key, f"/camera/{key.split('.')[-1]}/image_raw")

    # ------------------------------------------------------------------
    # Image preprocessing
    # ------------------------------------------------------------------

    def _ros_image_to_numpy(self, msg: Image) -> np.ndarray | None:
        if self.bridge is not None:
            try:
                return np.array(
                    self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"),
                    dtype=np.uint8,
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"cv_bridge error: {exc}", throttle_duration_sec=5.0
                )
        return None

    def _numpy_to_tensor(self, img_np: np.ndarray):
        w, h = self.adapter.image_size
        if CV2_AVAILABLE:
            resized = cv2.resize(img_np, (w, h), interpolation=cv2.INTER_LINEAR)
        else:
            resized = img_np
        arr = resized.astype(np.float32) / 255.0
        arr = arr.transpose(2, 0, 1)  # HWC → CHW
        if TORCH_AVAILABLE:
            t = torch.from_numpy(arr).unsqueeze(0)
            if self.device is not None:
                t = t.to(self.device)
            return t
        return arr[np.newaxis]

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image, key: str) -> None:
        np_img = self._ros_image_to_numpy(msg)
        with self._cam_lock:
            self.camera_images[key] = np_img

    def _depth_cb(self, msg: Image) -> None:
        pass  # depth reserved for future use

    def _arm_state_cb(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        self.arm_positions = np.array(
            [name_to_pos.get(n, 0.0) for n in ARM_JOINT_NAMES], dtype=np.float32
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self.base_vel = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.angular.z,
            ],
            dtype=np.float32,
        )

    def _task_cb(self, msg: String) -> None:
        self.task_description = msg.data
        self.get_logger().info(f'Task: "{self.task_description}"')
        if hasattr(self.adapter, "reset"):
            self.adapter.reset()
        if hasattr(self, "guard"):
            self.guard.reset()

    def _enable_cb(self, msg: Bool) -> None:
        self.enabled = msg.data
        self.get_logger().info(f"Policy {'ENABLED' if msg.data else 'DISABLED'}.")
        if msg.data:
            self.adapter.reset()
            if hasattr(self, "guard"):
                self.guard.reset()

    def _policy_load_cb(self, msg: String) -> None:
        """Reload a checkpoint from the headset. Does not enable inference."""
        requested = requested_checkpoint(getattr(self, "_checkpoint", ""), msg.data)
        if requested is None:
            return
        previous = self.adapter
        previous_ckpt = self._checkpoint
        self._checkpoint = requested
        loaded = self._load_adapter(
            self._model_type,
            requested,
            self._device_str,
            serve_url=self._serve_url,
            serve_timeout_sec=self._serve_timeout_sec,
            serve_api_key=self._serve_api_key,
        )
        if isinstance(loaded, _DummyAdapter) and not isinstance(previous, _DummyAdapter):
            self.adapter = previous
            self._checkpoint = previous_ckpt
            self.get_logger().error(
                f'policy/load "{requested}" failed; keeping the previous adapter.'
            )
            return
        self.adapter = loaded
        if hasattr(self, "guard"):
            self.guard.required_keys = list(self.adapter.image_keys)
            self.guard.reset()
        self.get_logger().info(
            f'policy/load applied checkpoint "{requested}". '
            f"Inference stays {'on' if self.enabled else 'off'}."
        )

    # ------------------------------------------------------------------
    # Inference loop
    # ------------------------------------------------------------------

    def _inference_loop(self) -> None:
        if not self.enabled:
            return

        with self._cam_lock:
            camera_snapshot = dict(self.camera_images)

        required_keys = self.adapter.image_keys
        valid, invalid = self.guard.check_images(camera_snapshot, required_keys)
        if not valid:
            self.get_logger().warn(
                f"Waiting for valid images (missing or black: {invalid})",
                throttle_duration_sec=2.0,
            )
            self._publish_zero_base()
            return

        t0 = time.perf_counter() if self._diag_enabled else None

        try:
            obs = {}
            for key in required_keys:
                img = camera_snapshot[key]
                obs[key] = self._numpy_to_tensor(img)

            state = np.concatenate([self.arm_positions, self.base_vel])
            if TORCH_AVAILABLE:
                state_t = torch.from_numpy(state).unsqueeze(0)
                if self.device is not None:
                    state_t = state_t.to(self.device)
            else:
                state_t = state[np.newaxis]
            obs[self.adapter.state_key] = state_t

            if self.adapter.task_key:
                obs[self.adapter.task_key] = self.task_description

            t1 = time.perf_counter()

            action = self.adapter.select_action(obs)

            t2 = time.perf_counter()
            inference_dur = t2 - t1

            if self._diag_enabled and t0 is not None:

                def ms(a, b):
                    return (b - a) * 1000.0

                self._t_preprocess.append(ms(t0, t1))
                self._t_inference.append(ms(t1, t2))
                self._t_total.append(ms(t0, t2))

            if self.guard.is_latency_exceeded(inference_dur, self.policy_period):
                self.get_logger().warn(
                    f"Inference latency ({inference_dur * 1000.0:.1f}ms) exceeded "
                    f"policy period ({self.policy_period * 1000.0:.1f}ms) — action dropped.",
                    throttle_duration_sec=1.0,
                )
                self._publish_zero_base()
                return

            action_arr = np.asarray(action, dtype=np.float32).ravel()
            clamped_arm = self.guard.process_arm_action(
                action_arr[:6], reference=self.arm_positions
            )
            self._publish_arm(clamped_arm)
            self._publish_base(action_arr[6:9])

        except Exception as exc:
            self.get_logger().error(
                f"Inference error: {exc}", throttle_duration_sec=5.0
            )
            self._publish_zero_base()

    def _publish_zero_base(self) -> None:
        """Publish zero base velocity when policy tick is skipped."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def _publish_arm(self, arm_action: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINT_NAMES
        msg.position = [float(x) for x in arm_action]
        self.joint_cmd_pub.publish(msg)

    def _publish_base(self, base_action: np.ndarray) -> None:
        s = self.base_vel_scale
        # Clip to match the yahboom_controller_node hardware limits
        # (0.2 m/s linear, 0.5 rad/s angular). The scale parameter
        # (default 0.3) determines output range within these limits.
        max_lin = 0.20
        max_ang = 0.50
        msg = Twist()
        msg.linear.x = float(np.clip(base_action[0] * s, -max_lin, max_lin))
        msg.linear.y = float(np.clip(base_action[1] * s, -max_lin, max_lin))
        msg.angular.z = float(np.clip(base_action[2] * s, -max_ang, max_ang))
        self.cmd_vel_pub.publish(msg)

    def _publish_diagnostics(self) -> None:
        msg = self._DiagArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        def _make(name, dq):
            st = self._DiagStatus()
            st.name = name
            if not dq:
                st.message = "no data"
                return st
            s = sorted(dq)
            p95 = s[max(0, int(0.95 * len(s)) - 1)]
            budget = 1000.0 / self.policy_hz
            st.level = (
                self._DiagStatus.ERROR
                if p95 > budget
                else self._DiagStatus.WARN
                if p95 > budget * 0.8
                else self._DiagStatus.OK
            )
            st.message = f"p95={p95:.1f}ms (budget={budget:.0f}ms)"
            for k, v in [
                ("mean_ms", sum(s) / len(s)),
                ("p95_ms", p95),
                ("max_ms", s[-1]),
            ]:
                kv = self._KeyValue()
                kv.key = k
                kv.value = f"{v:.3f}"
                st.values.append(kv)
            return st

        msg.status = [
            _make("policy/total_ms", self._t_total),
            _make("policy/preprocess_ms", self._t_preprocess),
            _make("policy/inference_ms", self._t_inference),
        ]
        self._diag_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
