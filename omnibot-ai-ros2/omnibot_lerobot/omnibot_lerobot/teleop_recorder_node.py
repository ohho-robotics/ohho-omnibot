#!/usr/bin/env python3
"""
Teleoperation + dataset recording node for OmniBot mobile manipulation.

Records synchronized arm + base demonstrations for SmolVLA fine-tuning.

Leader arm (SO-101) provides arm action commands.
Xbox controller drives the base and triggers recording.

State machine:
  IDLE      → waiting for RB button press
  RECORDING → accumulating frames
  SAVING    → writing episode to disk

Topics subscribed:
  /arm/leader_states           (sensor_msgs/JointState) - leader arm (arm actions, leader mode)
  /arm/joint_states            (sensor_msgs/JointState) - follower arm (arm observations)
  /joy                         (sensor_msgs/Joy)         - Xbox controller (leader base action)
  /camera/wrist/image_raw      (sensor_msgs/Image)  → observation.images.wrist
  /camera/base/bev/image_raw   (sensor_msgs/Image)  → observation.images.bev (BEV mosaic from 4 base cameras)
  /odom                        (nav_msgs/Odometry)       - base velocity (state)

action_source:=vr (default is leader; leader behaviour is unchanged):
  Arm action is the last JointState on /arm/joint_commands/teleop (OHH-197).
  If that topic is silent, /arm/joint_commands/out is used only while
  /control_mode is teleop. Base action is /cmd_vel/teleop, with /cmd_vel/out
  as the same teleop-only fallback. State stays follower joints + /odom.
  Episode start/stop/discard are the /ohho/record services. /joy is not
  required. This node never publishes a joint command or a base twist.
  Leader episodes still go through the ohho-os Recorder when it imports.

Unified 9D action:  [arm(6), base_vx, base_vy, base_vz]
Unified 9D state:   [arm(6), base_vx, base_vy, base_vz]

Services (std_srvs/Trigger), plus /ohho/record/task and /ohho/record/episode_index:
  /ohho/record/start     begin one episode (refused if already recording)
  /ohho/record/stop      save the in-progress episode
  /ohho/record/discard   drop the in-progress episode; nothing is written

Topic:
  /ohho/record/state     std_msgs/String JSON, transient-local, 2 Hz and on each phase change
"""

import json
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from enum import Enum, auto

try:
    from cv_bridge import CvBridge

    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

try:
    import cv2  # noqa: F401

    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# ---------------------------------------------------------------------------
# Optional LeRobot import
# ---------------------------------------------------------------------------
try:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False

try:
    from omnibot_lerobot.ohho_teleop_recorder import (
        OHHO_AVAILABLE,
        WRITER_AVAILABLE,
        OhhoTeleopRecorder,
    )
except ImportError:
    OHHO_AVAILABLE = False
    WRITER_AVAILABLE = False
    OhhoTeleopRecorder = None  # type: ignore[misc, assignment]

OHHO_RECORDING = OHHO_AVAILABLE and WRITER_AVAILABLE

try:
    from omnibot_lerobot.record_session import RecordPhase, RecordSession, commit_stop
    from omnibot_lerobot.lerobot_v2 import append_episode
    from omnibot_lerobot.vr_action import (
        ACTION_SOURCE_VR,
        DEFAULT_ARM_COMMAND_TOPIC,
        DEFAULT_ARM_MUX_TOPIC,
        DEFAULT_BASE_COMMAND_TOPIC,
        DEFAULT_BASE_MUX_TOPIC,
        DEFAULT_CONTROL_MODE_TOPIC,
        JOINT_NAMES,
        assemble_vr_sample,
        episode_metadata,
        normalize_action_source,
        normalize_control_mode,
        normalize_prefer,
        normalize_robot_kind,
        positions_for_joints,
        twist_vector,
    )
except ImportError:
    from record_session import RecordPhase, RecordSession, commit_stop
    from lerobot_v2 import append_episode
    from vr_action import (
        ACTION_SOURCE_VR,
        DEFAULT_ARM_COMMAND_TOPIC,
        DEFAULT_ARM_MUX_TOPIC,
        DEFAULT_BASE_COMMAND_TOPIC,
        DEFAULT_BASE_MUX_TOPIC,
        DEFAULT_CONTROL_MODE_TOPIC,
        JOINT_NAMES,
        assemble_vr_sample,
        episode_metadata,
        normalize_action_source,
        normalize_control_mode,
        normalize_prefer,
        normalize_robot_kind,
        positions_for_joints,
        twist_vector,
    )

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MAX_LINEAR = 0.2  # m/s safe recording speed
MAX_ANGULAR = 1.0  # rad/s
STATE_TOPIC = "/ohho/record/state"
STATE_HZ = 2.0


class RecordState(Enum):
    IDLE = auto()
    RECORDING = auto()
    SAVING = auto()


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class TeleopRecorderNode(Node):
    """Records teleoperation episodes in LeRobot HF dataset format."""

    def __init__(self, **node_kwargs):
        super().__init__("teleop_recorder_node", **node_kwargs)

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("output_dir", "~/datasets/mobile_manipulation")
        self.declare_parameter("repo_id", "local/mobile_manipulation")
        self.declare_parameter("record_hz", 30.0)
        self.declare_parameter("episode_timeout_s", 60.0)
        self.declare_parameter("joy_record_button", 7)  # Xbox RB
        self.declare_parameter("joy_discard_button", 6)  # Xbox LB
        self.declare_parameter("task_name", "mobile manipulation task")
        # -1 means "use the recorder's next index". >= 0 pins that index on every start.
        self.declare_parameter("episode_index", -1)
        # leader keeps the joystick + leader-arm path. vr records teleop topics.
        self.declare_parameter("action_source", "leader")
        self.declare_parameter("operator", "")
        self.declare_parameter("robot_kind", "sim")
        self.declare_parameter("sim_world", "")
        self.declare_parameter("vr_action_prefer", "teleop")
        self.declare_parameter("vr_arm_command_topic", DEFAULT_ARM_COMMAND_TOPIC)
        self.declare_parameter("vr_arm_mux_topic", DEFAULT_ARM_MUX_TOPIC)
        self.declare_parameter("vr_base_command_topic", DEFAULT_BASE_COMMAND_TOPIC)
        self.declare_parameter("vr_base_mux_topic", DEFAULT_BASE_MUX_TOPIC)
        self.declare_parameter("control_mode_topic", DEFAULT_CONTROL_MODE_TOPIC)

        self.output_dir = os.path.expanduser(self.get_parameter("output_dir").value)
        self.repo_id = self.get_parameter("repo_id").value
        self.record_hz = self.get_parameter("record_hz").value
        self.episode_timeout_s = self.get_parameter("episode_timeout_s").value
        self.joy_record_button = self.get_parameter("joy_record_button").value
        self.joy_discard_button = self.get_parameter("joy_discard_button").value
        self.action_source = self._read_action_source()
        self.robot_kind, robot_kind_ok = normalize_robot_kind(
            self.get_parameter("robot_kind").value
        )
        self.vr_action_prefer, prefer_ok = normalize_prefer(
            self.get_parameter("vr_action_prefer").value
        )
        sim_world = self.get_parameter("sim_world").value
        self.sim_world = sim_world.strip() if isinstance(sim_world, str) else ""
        if not robot_kind_ok:
            self.get_logger().warn(
                f'Unknown robot_kind "{self.get_parameter("robot_kind").value}" — using "sim".'
            )
        if not prefer_ok:
            self.get_logger().warn(
                "Unknown vr_action_prefer "
                f'"{self.get_parameter("vr_action_prefer").value}" — using "teleop".'
            )
        operator = self.get_parameter("operator").value
        if isinstance(operator, str) and operator.strip():
            self.operator = operator.strip()
        elif self.action_source == ACTION_SOURCE_VR:
            self.operator = "vr"
        else:
            self.operator = ""
        self.vr_arm_command_topic = self._topic_param(
            "vr_arm_command_topic", DEFAULT_ARM_COMMAND_TOPIC
        )
        self.vr_arm_mux_topic = self._topic_param("vr_arm_mux_topic", DEFAULT_ARM_MUX_TOPIC)
        self.vr_base_command_topic = self._topic_param(
            "vr_base_command_topic", DEFAULT_BASE_COMMAND_TOPIC
        )
        self.vr_base_mux_topic = self._topic_param(
            "vr_base_mux_topic", DEFAULT_BASE_MUX_TOPIC
        )
        self.control_mode_topic = self._topic_param(
            "control_mode_topic", DEFAULT_CONTROL_MODE_TOPIC
        )

        os.makedirs(self.output_dir, exist_ok=True)

        # ------------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------------
        task_name = self.get_parameter("task_name").value
        self.session = RecordSession(
            task=task_name if isinstance(task_name, str) else "mobile manipulation task"
        )
        self._topic_task: str | None = None
        # One-shot index from /ohho/record/episode_index. Cleared after a successful start.
        self._pending_episode_index: int | None = None
        self._mirror_session()

        # Sensor cache
        self.leader_positions = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.follower_positions = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.base_vel = np.zeros(3, dtype=np.float32)
        self.joy_axes = [0.0] * 8
        self.joy_buttons = [0] * 16
        self.prev_joy_buttons = [0] * 16
        self.wrist_image = None  # numpy HxWx3 uint8 RGB
        self.bev_image = None  # numpy HxWx3 uint8 RGB — BEV mosaic from 4 base cameras
        self.wrist_msg_time = None
        self.bev_msg_time = None

        self._ohho_rec = (
            OhhoTeleopRecorder(self.repo_id, self.record_hz)
            if OHHO_RECORDING and OhhoTeleopRecorder is not None
            else None
        )
        # Last VR samples. None means that topic has not published yet.
        # Mux samples are ignored unless /control_mode is teleop.
        self._arm_teleop = None
        self._arm_mux = None
        self._base_teleop = None
        self._base_mux = None
        self._control_mode = "teleop"

        # ------------------------------------------------------------------
        # cv_bridge
        # ------------------------------------------------------------------
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn("cv_bridge not available — images will be zeros.")

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(
            JointState, "/arm/leader_states", self.leader_state_cb, 10
        )
        self.create_subscription(
            JointState, "/arm/joint_states", self.follower_state_cb, 10
        )
        self.create_subscription(Joy, "/joy", self.joy_cb, 10)
        self.create_subscription(
            Image, "/camera/wrist/image_raw", self.wrist_image_cb, 10
        )
        self.create_subscription(
            Image, "/camera/base/bev/image_raw", self.bev_image_cb, 10
        )
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        if self.action_source == ACTION_SOURCE_VR:
            self._subscribe_vr_topics()

        # ------------------------------------------------------------------
        # Record timer
        # ------------------------------------------------------------------
        timer_period = 1.0 / self.record_hz
        self.create_timer(timer_period, self.record_timer_cb)

        if self.action_source == ACTION_SOURCE_VR:
            writer = "lerobot v2"
        elif self._ohho_rec is not None:
            writer = "ohho-os+data_engine"
        elif LEROBOT_AVAILABLE:
            writer = "lerobot"
        else:
            writer = "numpy fallback"
        self.get_logger().info(
            f"TeleopRecorderNode started | hz={self.record_hz} | "
            f"source={self.action_source} | output={self.output_dir} | writer={writer}"
        )
        self.get_logger().info(
            f"Press button {self.joy_record_button} (RB) to start/stop recording. "
            f"Button {self.joy_discard_button} (LB) discards the current episode."
        )
        if self.action_source == ACTION_SOURCE_VR:
            self.get_logger().info(
                "VR action source. Episode control is /ohho/record/start, "
                "/ohho/record/stop, and /ohho/record/discard (joystick not required). "
                f"Arm {self.vr_arm_command_topic} (mux fallback {self.vr_arm_mux_topic} "
                f"only in teleop), base {self.vr_base_command_topic} "
                f"(mux fallback {self.vr_base_mux_topic}). "
                f"prefer={self.vr_action_prefer} operator={self.operator} "
                f"robot_kind={self.robot_kind} sim_world={self.sim_world or '(none)'}."
            )

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(String, STATE_TOPIC, state_qos)
        self.create_subscription(String, "/ohho/record/task", self._task_cb, 10)
        self.create_subscription(
            Int32, "/ohho/record/episode_index", self._episode_index_cb, 10
        )
        self.create_service(Trigger, "/ohho/record/start", self._on_start_service)
        self.create_service(Trigger, "/ohho/record/stop", self._on_stop_service)
        self.create_service(Trigger, "/ohho/record/discard", self._on_discard_service)
        self.create_timer(1.0 / STATE_HZ, self._publish_state)
        self._publish_state()
        self.get_logger().info(
            "Recording services: /ohho/record/start /ohho/record/stop "
            "/ohho/record/discard ; state on /ohho/record/state"
        )

    # ------------------------------------------------------------------
    # Image helpers
    # ------------------------------------------------------------------

    def _ros_image_to_numpy_rgb(self, msg: Image):
        """Convert ROS Image to numpy HxWx3 uint8 RGB."""
        if self.bridge is not None:
            try:
                return np.array(
                    self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"),
                    dtype=np.uint8,
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Image conversion error: {exc}", throttle_duration_sec=5.0
                )
        return np.zeros((240, 320, 3), dtype=np.uint8)

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def leader_state_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        self.leader_positions = np.array(
            [name_to_pos.get(n, 0.0) for n in JOINT_NAMES], dtype=np.float32
        )

    def follower_state_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        self.follower_positions = np.array(
            [name_to_pos.get(n, 0.0) for n in JOINT_NAMES], dtype=np.float32
        )

    def odom_cb(self, msg: Odometry):
        self.base_vel = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.angular.z,
            ],
            dtype=np.float32,
        )

    def _subscribe_vr_topics(self) -> None:
        """Subscribe to teleop commands. This node does not publish them."""
        from geometry_msgs.msg import Twist

        self.create_subscription(
            JointState, self.vr_arm_command_topic, self._vr_arm_cb, 10
        )
        self.create_subscription(JointState, self.vr_arm_mux_topic, self._vr_arm_mux_cb, 10)
        self.create_subscription(Twist, self.vr_base_command_topic, self._vr_base_cb, 10)
        self.create_subscription(Twist, self.vr_base_mux_topic, self._vr_base_mux_cb, 10)
        self.create_subscription(String, self.control_mode_topic, self._control_mode_cb, 10)

    def _vr_arm_cb(self, msg: JointState) -> None:
        positions = positions_for_joints(msg.name, msg.position)
        if positions is not None:
            self._arm_teleop = positions

    def _vr_arm_mux_cb(self, msg: JointState) -> None:
        positions = positions_for_joints(msg.name, msg.position)
        if positions is not None:
            self._arm_mux = positions

    def _vr_base_cb(self, msg) -> None:
        sample = twist_vector(msg.linear.x, msg.linear.y, msg.angular.z)
        if sample is not None:
            self._base_teleop = sample

    def _vr_base_mux_cb(self, msg) -> None:
        sample = twist_vector(msg.linear.x, msg.linear.y, msg.angular.z)
        if sample is not None:
            self._base_mux = sample

    def _control_mode_cb(self, msg: String) -> None:
        mode = normalize_control_mode(msg.data)
        if mode is None:
            shown = msg.data if isinstance(msg.data, str) else msg.data
            self.get_logger().warn(
                f'Unknown control mode "{shown}" ignored. Keeping "{self._control_mode}".'
            )
            return
        self._control_mode = mode

    def _read_action_source(self) -> str:
        try:
            return normalize_action_source(self.get_parameter("action_source").value)
        except ValueError:
            shown = self.get_parameter("action_source").value
            self.get_logger().warn(
                f'Unknown action_source "{shown}" — using "leader".'
            )
            return "leader"

    def _topic_param(self, name: str, default: str) -> str:
        value = self.get_parameter(name).value
        if isinstance(value, str) and value.strip():
            return value.strip()
        self.get_logger().warn(f'Empty {name} — using "{default}".')
        return default

    def joy_cb(self, msg: Joy):
        self.prev_joy_buttons = list(self.joy_buttons)
        self.joy_axes = list(msg.axes)
        self.joy_buttons = list(msg.buttons)
        self._handle_button_events()

    def _image_stamp_s(self, msg: Image) -> float:
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return time.time()

    def wrist_image_cb(self, msg: Image):
        self.wrist_image = self._ros_image_to_numpy_rgb(msg)
        self.wrist_msg_time = self._image_stamp_s(msg)

    def bev_image_cb(self, msg: Image):
        self.bev_image = self._ros_image_to_numpy_rgb(msg)
        self.bev_msg_time = self._image_stamp_s(msg)

    # ------------------------------------------------------------------
    # Button handling
    # ------------------------------------------------------------------

    def _button_just_pressed(self, idx: int) -> bool:
        """True if button transitioned 0→1 this tick."""
        if idx >= len(self.joy_buttons) or idx >= len(self.prev_joy_buttons):
            return False
        return self.joy_buttons[idx] == 1 and self.prev_joy_buttons[idx] == 0

    def _handle_button_events(self):
        if self._button_just_pressed(self.joy_record_button):
            self._on_record_button()

        if self._button_just_pressed(self.joy_discard_button):
            self._on_discard_button()

    def _on_record_button(self):
        if self.session.state is RecordPhase.IDLE:
            result = self._begin_recording()
            if result.success:
                self.get_logger().info(
                    f"Recording episode {result.episode_index} ... "
                    f"Press RB again to save, LB to discard."
                )
            else:
                self.get_logger().warn(result.message)
        elif self.session.state is RecordPhase.RECORDING:
            self.get_logger().info(
                f"Stopping recording. Saving {self.session.frame_count} frames..."
            )
            self._stop_and_save()

    def _on_discard_button(self):
        if self.session.state is not RecordPhase.RECORDING:
            return
        frames_recorded = self.session.frame_count
        index = self.session.episode_index
        result = self.session.discard()
        if result.success:
            self._discard_ohho_episode()
        self._mirror_session()
        self._publish_state()
        self.get_logger().warn(
            f"Discarding episode {index} ({frames_recorded} frames recorded)."
        )
        if not result.success:
            self.get_logger().warn(result.message)

    # ------------------------------------------------------------------
    # Record timer
    # ------------------------------------------------------------------

    def record_timer_cb(self):
        # Check timeout
        if self.session.state is RecordPhase.RECORDING:
            elapsed = time.time() - self.session.started_at
            if elapsed > self.episode_timeout_s:
                self.get_logger().warn(
                    f"Episode timeout ({self.episode_timeout_s}s). Auto-saving..."
                )
                self._stop_and_save()
                return

            self._record_frame()

    def _leader_action_state(self):
        """Leader arm positions plus Xbox base axes. Unchanged leader path."""
        base_vx = (
            float(self.joy_axes[1]) * MAX_LINEAR if len(self.joy_axes) > 1 else 0.0
        )
        base_vy = (
            float(self.joy_axes[0]) * MAX_LINEAR if len(self.joy_axes) > 0 else 0.0
        )
        base_vz = (
            float(self.joy_axes[3]) * MAX_ANGULAR if len(self.joy_axes) > 3 else 0.0
        )
        arm_action = self.leader_positions.copy()
        base_action = np.array([base_vx, base_vy, base_vz], dtype=np.float32)
        full_action = np.concatenate([arm_action, base_action])
        full_state = np.concatenate(
            [self.follower_positions.copy(), self.base_vel.copy()]
        )
        return full_action, full_state

    def _vr_action_state(self):
        """Last teleop command as the action. Follower joints and odom as the state."""
        sample = assemble_vr_sample(
            arm_teleop=self._arm_teleop,
            arm_mux=self._arm_mux,
            base_teleop=self._base_teleop,
            base_mux=self._base_mux,
            control_mode=self._control_mode,
            follower=self.follower_positions,
            odom_twist=self.base_vel,
            prefer=self.vr_action_prefer,
        )
        return sample

    def _record_frame(self):
        """Capture and append one frame to the episode buffer."""
        vr_sample = None
        if self.action_source == ACTION_SOURCE_VR:
            vr_sample = self._vr_action_state()
            full_action = vr_sample.action
            full_state = vr_sample.state
        else:
            full_action, full_state = self._leader_action_state()

        # Images (copy to avoid mutation)
        wrist_img = (
            self.wrist_image.copy()
            if self.wrist_image is not None
            else np.zeros((240, 320, 3), dtype=np.uint8)
        )
        bev_img = (
            self.bev_image.copy()
            if self.bev_image is not None
            else np.zeros((240, 320, 3), dtype=np.uint8)
        )

        captured_at = time.time()
        frame = {
            "state": full_state.astype(np.float32),
            "action": full_action.astype(np.float32),
            "wrist_image": wrist_img,
            "bev_image": bev_img,  # BEV mosaic from 4 base cameras
            "timestamp": captured_at,
            "wrist_timestamp": captured_at,
            "bev_timestamp": captured_at,
        }
        if vr_sample is not None:
            frame["arm_source"] = vr_sample.arm_source
            frame["base_source"] = vr_sample.base_source
        if self.session.append_frame(frame) and self._leader_uses_ohho():
            self._ohho_rec.capture_frame(
                full_state,
                full_action,
                wrist_img,
                bev_img,
                wall_timestamp=captured_at,
                wrist_timestamp=self.wrist_msg_time,
                bev_timestamp=self.bev_msg_time,
            )
            self.session.note_frame_count(self._ohho_rec.frame_count)
        self.frame_count = self.session.frame_count
        self.episode_buffer = self.session.live_frames

    # ------------------------------------------------------------------
    # Services and state topic
    # ------------------------------------------------------------------

    def _mirror_session(self) -> None:
        """Copy session fields onto the attributes the episode writer reads."""
        self.record_state = {
            RecordPhase.IDLE: RecordState.IDLE,
            RecordPhase.RECORDING: RecordState.RECORDING,
            RecordPhase.SAVING: RecordState.SAVING,
        }[self.session.state]
        self.episode_index = self.session.episode_index
        self.frame_count = self.session.frame_count
        self.task_description = self.session.task
        self.recording_start_time = self.session.started_at or None
        self.episode_buffer = self.session.live_frames

    def _task_for_start(self) -> str | None:
        if self._topic_task is not None and self._topic_task.strip():
            return self._topic_task.strip()
        value = self.get_parameter("task_name").value
        if isinstance(value, str) and value.strip():
            return value.strip()
        return None

    def _episode_index_for_start(self) -> int | None:
        if self._pending_episode_index is not None:
            if self._pending_episode_index < 0:
                return None
            return self._pending_episode_index
        value = self.get_parameter("episode_index").value
        try:
            index = int(value)
        except (TypeError, ValueError):
            return None
        if isinstance(value, bool) or index < 0:
            return None
        return index

    def _begin_recording(self):
        result = self.session.start(
            task=self._task_for_start(),
            episode_index=self._episode_index_for_start(),
        )
        if result.success:
            self._pending_episode_index = None
        self._mirror_session()
        if result.success:
            self._start_ohho_episode()
        self._publish_state()
        return result

    def _leader_uses_ohho(self) -> bool:
        """Leader episodes use ohho-os. VR episodes use the LeRobot v2 writer."""
        return self._ohho_rec is not None and self.action_source != ACTION_SOURCE_VR

    def _start_ohho_episode(self) -> None:
        if not self._leader_uses_ohho():
            return
        # discard_episode is a no-op when nothing is open. It clears a leftover
        # episode so start_episode cannot raise after an empty stop.
        self._ohho_rec.discard_episode()
        self._ohho_rec.start_episode(self.task_description)

    def _discard_ohho_episode(self) -> None:
        if not self._leader_uses_ohho():
            return
        self._ohho_rec.discard_episode()

    def _stop_and_save(self):
        def write(frames):
            self.episode_buffer = frames
            self.frame_count = len(frames)
            self.episode_index = self.session.episode_index
            self.task_description = self.session.task
            self.record_state = RecordState.SAVING
            self.recording_start_time = self.session.started_at or None
            self._publish_state()
            self._save_episode()

        result = commit_stop(self.session, write)
        if not result.success and "no frames to save" in result.message:
            self._discard_ohho_episode()
        self._mirror_session()
        self._publish_state()
        if not result.success:
            self.get_logger().warn(result.message)
        return result

    def _publish_state(self) -> None:
        msg = String()
        msg.data = json.dumps(self.session.snapshot(), separators=(",", ":"))
        self._state_pub.publish(msg)

    def _task_cb(self, msg: String) -> None:
        self._topic_task = msg.data

    def _episode_index_cb(self, msg: Int32) -> None:
        self._pending_episode_index = int(msg.data)

    def _fill_trigger(self, response, result):
        response.success = result.success
        response.message = result.message
        return response

    def _on_start_service(self, _request, response):
        result = self._begin_recording()
        if result.success:
            self.get_logger().info(result.message)
        else:
            self.get_logger().warn(result.message)
        return self._fill_trigger(response, result)

    def _on_stop_service(self, _request, response):
        result = self._stop_and_save()
        return self._fill_trigger(response, result)

    def _on_discard_service(self, _request, response):
        result = self.session.discard()
        if result.success:
            self._discard_ohho_episode()
        self._mirror_session()
        self._publish_state()
        self.get_logger().warn(result.message)
        return self._fill_trigger(response, result)

    # ------------------------------------------------------------------
    # Episode saving
    # ------------------------------------------------------------------

    def _save_episode(self):
        """Write the current episode buffer. Does not change recording state.

        The episode index advances in ``RecordSession.complete_save`` only
        after this returns. Discard never calls this method.
        """
        if not self.episode_buffer:
            raise RuntimeError("no frames to save")

        if self.action_source == ACTION_SOURCE_VR:
            self._save_vr_dataset()
        elif self._ohho_rec is not None:
            self._ohho_rec.save_episode(
                self.output_dir,
                self.task_description,
                self.record_hz,
                self.repo_id,
            )
        elif LEROBOT_AVAILABLE:
            self._save_lerobot_format()
        else:
            self._save_numpy_format()

        self.get_logger().info(
            f"Episode {self.episode_index} saved "
            f"({self.frame_count} frames @ {self.record_hz:.0f} Hz, "
            f"{self.frame_count / self.record_hz:.1f}s)."
        )

    def _save_vr_dataset(self):
        """Write a LeRobot v2 episode. Does not publish motion commands."""
        metadata = episode_metadata(
            task=self.task_description,
            operator=self.operator or "vr",
            robot_kind=self.robot_kind,
            sim_world=self.sim_world,
        )
        append_episode(
            self.output_dir,
            self.episode_buffer,
            metadata,
            fps=max(int(round(float(self.record_hz))), 1),
            episode_index=int(self.episode_index),
            repo_id=self.repo_id,
        )
        self.get_logger().info(
            "VR episode metadata "
            f"operator={metadata['operator']} robot_kind={metadata['robot_kind']} "
            f"sim_world={metadata['sim_world'] or '(none)'} task={metadata['task']}"
        )

    def _save_lerobot_format(self):
        """Save episode using LeRobot HuggingFace dataset format."""
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (9,),
                "names": {"joints": JOINT_NAMES + ["base_vx", "base_vy", "base_vz"]},
            },
            "action": {
                "dtype": "float32",
                "shape": (9,),
                "names": {"joints": JOINT_NAMES + ["base_vx", "base_vy", "base_vz"]},
            },
            "observation.images.wrist": {
                "dtype": "video",
                "shape": (3, 240, 320),
                "names": ["channels", "height", "width"],
            },
            "observation.images.bev": {
                "dtype": "video",
                "shape": (3, 240, 320),
                "names": ["channels", "height", "width"],
            },
        }

        dataset = LeRobotDataset.create(
            repo_id=self.repo_id,
            fps=int(self.record_hz),
            root=self.output_dir,
            features=features,
            image_writer_threads=4,
        )

        for frame in self.episode_buffer:
            dataset.add_frame(
                {
                    "observation.state": frame["state"],
                    "action": frame["action"],
                    "observation.images.wrist": frame["wrist_image"],
                    "observation.images.bev": frame["bev_image"],
                }
            )

        dataset.save_episode(task=self.task_description)

    def _save_numpy_format(self):
        """Fallback: save episode as .npz file."""
        states = np.stack([f["state"] for f in self.episode_buffer])
        actions = np.stack([f["action"] for f in self.episode_buffer])
        wrist_imgs = np.stack([f["wrist_image"] for f in self.episode_buffer])
        bev_imgs = np.stack([f["bev_image"] for f in self.episode_buffer])
        timestamps = np.array([f["timestamp"] for f in self.episode_buffer])

        ep_dir = os.path.join(self.output_dir, f"episode_{self.episode_index:05d}")
        os.makedirs(ep_dir, exist_ok=True)

        np.savez_compressed(
            os.path.join(ep_dir, "data.npz"),
            states=states,
            actions=actions,
            wrist_images=wrist_imgs,
            bev_images=bev_imgs,
            timestamps=timestamps,
            joint_names=np.array(JOINT_NAMES),
            task=np.array([self.task_description]),
            fps=np.array([self.record_hz]),
        )
        self.get_logger().info(
            f"Saved numpy episode to {ep_dir}/data.npz "
            "(pip install ohho-os and set PYTHONPATH to omnibot-ai-engines for LeRobot v2, "
            "or install lerobot for the HF writer)"
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = TeleopRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
