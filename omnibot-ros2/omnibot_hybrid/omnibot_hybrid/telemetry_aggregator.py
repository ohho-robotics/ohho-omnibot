#!/usr/bin/env python3
"""Publish /ohho/telemetry (ohho.telemetry.v1 JSON) at 20 Hz.

Read-only. This node does not publish base or arm commands and it does not
touch teleop_safety, the muxes, or their watchdogs.

Subscriptions
    /odom                         nav_msgs/Odometry
    /cmd_vel/out                  geometry_msgs/Twist
    /arm/joint_states             sensor_msgs/JointState
    /arm/joint_commands/out       sensor_msgs/JointState
    /imu/data                     sensor_msgs/Imu
    /control_mode/active          std_msgs/String
    /arm/cmd_mode/active          std_msgs/String
    /battery                      sensor_msgs/BatteryState
    /voltage                      std_msgs/Float32   (Yahboom pack voltage)
    /ohho/safety/state            std_msgs/String
    /ohho/heartbeat               std_msgs/String    (ohho.telemetry.v1 heartbeat)
    navigate_to_pose/_action/status
    navigate_to_pose/_action/feedback
    /diagnostics                  diagnostic_msgs/DiagnosticArray  (policy_node)
    /policy/enable                std_msgs/Bool
    /policy/task                  std_msgs/String
    /rosbag_recorder/status       std_msgs/String

Publications
    /ohho/telemetry               std_msgs/String JSON, transient-local, 20 Hz
    /ohho/heartbeat_echo          std_msgs/String JSON echo for client RTT

teleop_safety still listens for std_msgs/Empty on /ohho/heartbeat. A String
heartbeat for RTT does not replace that Empty deadman. Joint limits come
from arm_params.yaml. sim:=true sets kind to sim and battery.simulated.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque

import rclpy
import yaml
from action_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import Bool, Float32, String

from omnibot_hybrid.telemetry_snapshot import (
    SCHEMA_VERSION,
    dumps_json,
    duration_seconds,
    echo_heartbeat,
    joint_limits_from_arm_params,
    joint_limits_from_sequences,
    parse_recorder_status,
    policy_from_parts,
    rate_hz,
    resolve_robot_id,
    rpy_from_quaternion,
    select_goal_status,
    assemble,
)

_TELEMETRY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_DEFAULT_JOINTS = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)
_DEFAULT_MIN = (-3.14, -1.57, -1.57, -1.57, -3.14, -0.1)
_DEFAULT_MAX = (3.14, 1.57, 1.57, 1.57, 3.14, 0.8)


def _as_bool(value, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("1", "true", "yes", "on"):
            return True
        if text in ("0", "false", "no", "off", ""):
            return False
    return default


def _as_float(value, default: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(number):
        return default
    return number


class TelemetryAggregator(Node):
    """Cache the latest messages and publish one schema sample at a fixed rate."""

    def __init__(self):
        super().__init__("telemetry_aggregator")
        self.declare_parameter("sim", False)
        self.declare_parameter("robot_id", "")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("arm_present", True)
        self.declare_parameter("arm_params_file", "")
        self.declare_parameter("joint_names", list(_DEFAULT_JOINTS))
        self.declare_parameter("joint_min", list(_DEFAULT_MIN))
        self.declare_parameter("joint_max", list(_DEFAULT_MAX))
        self.declare_parameter("sim_battery_voltage", 12.6)
        self.declare_parameter("sim_battery_percent", 100.0)
        self.declare_parameter("battery_empty_voltage", 9.9)
        self.declare_parameter("battery_full_voltage", 12.6)
        self.declare_parameter("battery_topic", "/battery")
        self.declare_parameter("voltage_topic", "/voltage")
        self.declare_parameter("heartbeat_window_s", 2.0)

        self._sim = _as_bool(self.get_parameter("sim").value, False)
        self._robot_id = resolve_robot_id(
            str(self.get_parameter("robot_id").value or ""),
            sim=self._sim,
        )
        self._arm_present = _as_bool(self.get_parameter("arm_present").value, True)
        hz = _as_float(self.get_parameter("publish_hz").value, 20.0)
        if hz <= 0.0 or hz > 100.0:
            hz = 20.0
        self._publish_hz = hz
        self._heartbeat_window_s = _as_float(
            self.get_parameter("heartbeat_window_s").value, 2.0
        )
        self._sim_battery_v = _as_float(
            self.get_parameter("sim_battery_voltage").value, 12.6
        )
        self._sim_battery_pct = _as_float(
            self.get_parameter("sim_battery_percent").value, 100.0
        )
        self._empty_v = _as_float(self.get_parameter("battery_empty_voltage").value, 9.9)
        self._full_v = _as_float(self.get_parameter("battery_full_voltage").value, 12.6)
        self._warns: dict[str, float] = {}
        self._limits = self._load_limits()

        self._lock = threading.Lock()
        self._sources: dict = {}
        self._heartbeats: deque = deque(maxlen=128)
        self._cpu_prev = None
        self._policy_enabled = None
        self._policy_task = None
        self._policy_latency = None
        self._policy_level = None

        self._telemetry_pub = self.create_publisher(
            String, "/ohho/telemetry", _TELEMETRY_QOS
        )
        self._echo_pub = self.create_publisher(String, "/ohho/heartbeat_echo", 10)

        sensor = qos_profile_sensor_data
        self.create_subscription(Odometry, "/odom", self._on_odom, sensor)
        self.create_subscription(Twist, "/cmd_vel/out", self._on_cmd_vel, sensor)
        self.create_subscription(
            JointState, "/arm/joint_states", self._on_joint_state, sensor
        )
        self.create_subscription(
            JointState, "/arm/joint_commands/out", self._on_joint_cmd, sensor
        )
        self.create_subscription(Imu, "/imu/data", self._on_imu, sensor)
        self.create_subscription(String, "/control_mode/active", self._on_control_mode, 10)
        self.create_subscription(String, "/arm/cmd_mode/active", self._on_arm_mode, 10)
        self.create_subscription(
            BatteryState,
            str(self.get_parameter("battery_topic").value),
            self._on_battery,
            sensor,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("voltage_topic").value),
            self._on_voltage,
            sensor,
        )
        self.create_subscription(
            String, "/ohho/safety/state", self._on_safety, _TELEMETRY_QOS
        )
        self.create_subscription(String, "/ohho/heartbeat", self._on_heartbeat, 10)
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._on_nav_status,
            10,
        )
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            "/navigate_to_pose/_action/feedback",
            self._on_nav_feedback,
            10,
        )
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)
        self.create_subscription(Bool, "/policy/enable", self._on_policy_enable, 10)
        self.create_subscription(String, "/policy/task", self._on_policy_task, 10)
        self.create_subscription(
            String, "/rosbag_recorder/status", self._on_recorder, 10
        )

        period = 1.0 / self._publish_hz
        self.create_timer(period, self._on_tick)
        self.create_timer(1.0, self._on_host)
        self._on_tick()
        self.get_logger().info(
            "telemetry_aggregator "
            f"v={SCHEMA_VERSION} hz={self._publish_hz} "
            f"kind={'sim' if self._sim else 'real'} robot_id={self._robot_id} "
            f"joints={len(self._limits)}. "
            "Publishing /ohho/telemetry (transient-local) and /ohho/heartbeat_echo."
        )

    def _load_limits(self) -> dict:
        path = str(self.get_parameter("arm_params_file").value or "").strip()
        if path:
            try:
                with open(path, encoding="utf-8") as handle:
                    limits = joint_limits_from_arm_params(handle.read())
            except (OSError, ValueError, yaml.YAMLError) as exc:
                self._warn(
                    "arm_params",
                    f"arm_params.yaml ignored ({exc}); using joint_* parameters",
                )
            else:
                self.get_logger().info(f"joint limits from {path}")
                return limits
        try:
            return joint_limits_from_sequences(
                list(self.get_parameter("joint_names").value),
                list(self.get_parameter("joint_min").value),
                list(self.get_parameter("joint_max").value),
            )
        except ValueError as exc:
            self._warn("joint_limits", str(exc))
            return {}

    def _warn(self, key: str, text: str) -> None:
        now = time.monotonic()
        if now - self._warns.get(key, 0.0) < 5.0:
            return
        self._warns[key] = now
        self.get_logger().warn(text)

    def _put(self, key: str, data) -> None:
        with self._lock:
            self._sources[key] = {"t": time.time(), "data": data}

    def _drop(self, key: str) -> None:
        with self._lock:
            self._sources.pop(key, None)

    def _on_odom(self, msg: Odometry) -> None:
        try:
            if _covariance_invalid(msg.pose.covariance):
                return
            rpy = rpy_from_quaternion(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            if rpy is None:
                return
            self._put(
                "odom",
                {
                    "frame": msg.header.frame_id,
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "yaw": rpy[2],
                    "vx": msg.twist.twist.linear.x,
                    "vy": msg.twist.twist.linear.y,
                    "wz": msg.twist.twist.angular.z,
                },
            )
        except Exception as exc:
            self._warn("odom", f"odom ignored: {exc}")

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._put(
            "cmd_vel",
            {"vx": msg.linear.x, "vy": msg.linear.y, "wz": msg.angular.z},
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._put(
            "joint_state",
            {
                "names": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort),
            },
        )

    def _on_joint_cmd(self, msg: JointState) -> None:
        self._put(
            "joint_cmd",
            {"names": list(msg.name), "position": list(msg.position)},
        )

    def _on_imu(self, msg: Imu) -> None:
        try:
            if _covariance_invalid(msg.orientation_covariance) or _covariance_invalid(
                msg.linear_acceleration_covariance
            ):
                return
            rpy = rpy_from_quaternion(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            )
            if rpy is None:
                return
            self._put(
                "imu",
                {
                    "roll": rpy[0],
                    "pitch": rpy[1],
                    "yaw": rpy[2],
                    "ax": msg.linear_acceleration.x,
                    "ay": msg.linear_acceleration.y,
                    "az": msg.linear_acceleration.z,
                },
            )
        except Exception as exc:
            self._warn("imu", f"imu ignored: {exc}")

    def _on_control_mode(self, msg: String) -> None:
        self._put("control_mode", msg.data)

    def _on_arm_mode(self, msg: String) -> None:
        self._put("arm_mode", msg.data)

    def _on_battery(self, msg: BatteryState) -> None:
        try:
            volts = float(msg.voltage)
            if not math.isfinite(volts) or volts < 0.0:
                return
            pct = _battery_percent(msg.percentage)
            self._put("battery", {"v": volts, "pct": pct})
        except Exception as exc:
            self._warn("battery", f"battery ignored: {exc}")

    def _on_voltage(self, msg: Float32) -> None:
        try:
            volts = float(msg.data)
        except (TypeError, ValueError):
            return
        if not math.isfinite(volts) or volts < 0.0:
            return
        self._put("voltage", {"v": volts, "pct": None})

    def _on_safety(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self._warn("safety", "safety state is not JSON")
            return
        if isinstance(data, dict):
            self._put("safety", data)

    def _on_heartbeat(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self._warn("heartbeat", "heartbeat is not JSON")
            return
        received = time.time()
        echo = echo_heartbeat(payload, received)
        if echo is None:
            self._warn("heartbeat", "heartbeat does not match ohho.telemetry.v1")
            return
        out = String()
        out.data = dumps_json(echo)
        self._echo_pub.publish(out)
        with self._lock:
            self._heartbeats.append(received)

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        try:
            code = select_goal_status([item.status for item in msg.status_list])
        except Exception as exc:
            self._warn("nav_status", f"nav status ignored: {exc}")
            return
        if code is None:
            self._drop("nav_status")
        else:
            self._put("nav_status", code)

    def _on_nav_feedback(self, msg) -> None:
        try:
            feedback = msg.feedback
            eta = duration_seconds(
                feedback.estimated_time_remaining.sec,
                feedback.estimated_time_remaining.nanosec,
            )
            goal = _goal_from_feedback(feedback)
            if eta is None and goal is None:
                return
            self._put("nav_feedback", {"eta_s": eta, "goal": goal})
        except Exception as exc:
            self._warn("nav_feedback", f"nav feedback ignored: {exc}")

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        try:
            latency = None
            fault = False
            seen = False
            for status in msg.status:
                name = str(status.name)
                if not name.startswith("policy/"):
                    continue
                seen = True
                if int(status.level) >= 2:
                    fault = True
                if "inference" in name:
                    for item in status.values:
                        if item.key in ("p95_ms", "mean_ms"):
                            try:
                                latency = float(item.value)
                            except (TypeError, ValueError):
                                continue
            if not seen:
                return
            if latency is not None:
                self._policy_latency = latency
            self._policy_level = "error" if fault else None
            self._note_policy()
        except Exception as exc:
            self._warn("policy", f"policy diagnostics ignored: {exc}")

    def _on_policy_enable(self, msg: Bool) -> None:
        self._policy_enabled = bool(msg.data)
        self._note_policy()

    def _on_policy_task(self, msg: String) -> None:
        self._policy_task = msg.data
        self._note_policy()

    def _note_policy(self) -> None:
        block = policy_from_parts(
            enabled=self._policy_enabled,
            task=self._policy_task,
            latency_ms=self._policy_latency,
            level=self._policy_level,
        )
        if block is not None:
            self._put("policy", block)

    def _on_recorder(self, msg: String) -> None:
        parsed = parse_recorder_status(msg.data)
        if parsed is None:
            self._drop("recorder")
        else:
            self._put("recorder", parsed)

    def _on_host(self) -> None:
        cpu, self._cpu_prev = _cpu_percent(self._cpu_prev)
        temp = _cpu_temp()
        if cpu is None or temp is None:
            return
        self._put("sys", {"cpu": cpu, "temp": temp})

    def _on_tick(self) -> None:
        now = time.time()
        with self._lock:
            sources = {
                key: {"t": sample["t"], "data": sample["data"]}
                for key, sample in self._sources.items()
            }
            stamps = list(self._heartbeats)
        try:
            document = assemble(
                now,
                sources,
                robot_id=self._robot_id,
                sim=self._sim,
                joint_limits=self._limits,
                rtt_ms=0.0,
                heartbeat_hz=rate_hz(stamps, now, self._heartbeat_window_s),
                arm_present=self._arm_present,
                sim_battery_v=self._sim_battery_v if self._sim else None,
                sim_battery_pct=self._sim_battery_pct if self._sim else None,
                battery_empty_v=self._empty_v,
                battery_full_v=self._full_v,
            )
            payload = dumps_json(document)
        except (TypeError, ValueError) as exc:
            self._warn("assemble", f"telemetry sample skipped: {exc}")
            return
        msg = String()
        msg.data = payload
        self._telemetry_pub.publish(msg)


def _covariance_invalid(covariance) -> bool:
    try:
        return len(covariance) > 0 and float(covariance[0]) < 0.0
    except (TypeError, ValueError):
        return False


def _battery_percent(percentage) -> float | None:
    try:
        if percentage is None or math.isnan(float(percentage)):
            return None
    except (TypeError, ValueError):
        return None
    value = float(percentage)
    if not math.isfinite(value):
        return None
    if 0.0 <= value <= 1.0:
        return value * 100.0
    if 1.0 < value <= 100.0:
        return value
    return None


def _goal_from_feedback(feedback):
    for name in ("goal", "goal_pose"):
        pose = getattr(feedback, name, None)
        if pose is None or not hasattr(pose, "pose"):
            continue
        rpy = rpy_from_quaternion(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        if rpy is None:
            return None
        return {
            "x": pose.pose.position.x,
            "y": pose.pose.position.y,
            "yaw": rpy[2],
        }
    return None


def _read_cpu() -> tuple[int, int] | None:
    try:
        with open("/proc/stat", encoding="utf-8") as handle:
            parts = handle.readline().split()
    except OSError:
        return None
    if not parts or parts[0] != "cpu":
        return None
    try:
        nums = [int(part) for part in parts[1:]]
    except ValueError:
        return None
    if len(nums) < 4:
        return None
    idle = nums[3] + (nums[4] if len(nums) > 4 else 0)
    return sum(nums), idle


def _cpu_percent(previous):
    sample = _read_cpu()
    if sample is None or previous is None:
        return None, sample
    total = sample[0] - previous[0]
    idle = sample[1] - previous[1]
    if total <= 0:
        return None, sample
    percent = 100.0 * (total - idle) / total
    if percent < 0.0:
        percent = 0.0
    if percent > 100.0:
        percent = 100.0
    return percent, sample


def _cpu_temp() -> float | None:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", encoding="utf-8") as handle:
            raw = handle.read().strip()
    except OSError:
        return None
    try:
        temp = float(raw) / 1000.0
    except ValueError:
        return None
    if not math.isfinite(temp):
        return None
    return temp


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
