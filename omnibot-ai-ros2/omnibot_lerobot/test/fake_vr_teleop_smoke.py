#!/usr/bin/env python3
"""Publish synthetic VR teleop topics and record one short episode.

This is the ROS 2 launch check for OHH-226. It is not a robot node. It
publishes joint targets and a base twist, so it refuses to start unless
ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID is set to something other than 30
(the robot domain). Run it inside the navdev image, not on the robot.

Example (from the repo root, inside the image):

    export ROS_DOMAIN_ID=226 ROS_LOCALHOST_ONLY=1
    PYTHONPATH=omnibot-ai-ros2/omnibot_lerobot \\
      python3 omnibot-ai-ros2/omnibot_lerobot/test/fake_vr_teleop_smoke.py
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import time


_PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

ARM = [0.10, -0.20, 0.30, -0.15, 0.40, 0.20]
JOINT_NAMES = [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
]
BASE = (0.05, -0.02, 0.10)
MOTION_FRAGMENTS = ("cmd_vel", "joint_commands")


def _refuse_unless_isolated() -> None:
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    localhost = os.environ.get("ROS_LOCALHOST_ONLY", "")
    if domain in ("", "30") or localhost != "1":
        raise SystemExit(
            "Refusing to publish teleop topics. Set ROS_LOCALHOST_ONLY=1 and "
            "ROS_DOMAIN_ID to a value other than 30 before running this smoke."
        )


def main() -> int:
    _refuse_unless_isolated()
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    from omnibot_lerobot.teleop_recorder_node import TeleopRecorderNode

    output_dir = tempfile.mkdtemp(prefix="ohh226-vr-")
    rclpy.init()
    recorder = TeleopRecorderNode(
        parameter_overrides=[
            Parameter("action_source", Parameter.Type.STRING, "vr"),
            Parameter("output_dir", Parameter.Type.STRING, output_dir),
            Parameter("record_hz", Parameter.Type.DOUBLE, 10.0),
            Parameter("episode_timeout_s", Parameter.Type.DOUBLE, 30.0),
            Parameter("robot_kind", Parameter.Type.STRING, "sim"),
            Parameter("sim_world", Parameter.Type.STRING, "synthetic"),
            Parameter("task_name", Parameter.Type.STRING, "synthetic vr pick"),
            Parameter("operator", Parameter.Type.STRING, "vr"),
        ]
    )
    for publisher in recorder.publishers:
        for fragment in MOTION_FRAGMENTS:
            if fragment in publisher.topic_name:
                raise SystemExit(
                    f"recorder published a motion topic: {publisher.topic_name}"
                )

    class _FakeTeleop(Node):
        def __init__(self):
            super().__init__("fake_vr_teleop")
            self.arm = self.create_publisher(JointState, "/arm/joint_commands/teleop", 10)
            self.arm_mux = self.create_publisher(JointState, "/arm/joint_commands/out", 10)
            self.base = self.create_publisher(Twist, "/cmd_vel/teleop", 10)
            self.base_mux = self.create_publisher(Twist, "/cmd_vel/out", 10)
            self.follower = self.create_publisher(JointState, "/arm/joint_states", 10)
            self.odom = self.create_publisher(Odometry, "/odom", 10)
            self.mode = self.create_publisher(String, "/control_mode", 10)

        def publish_once(self):
            arm = JointState()
            arm.name = list(JOINT_NAMES)
            arm.position = list(ARM)
            self.arm.publish(arm)
            mux = JointState()
            mux.name = list(JOINT_NAMES)
            mux.position = [1.0] * 6
            self.arm_mux.publish(mux)
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.angular.z = BASE
            self.base.publish(twist)
            other = Twist()
            other.linear.x = 0.9
            self.base_mux.publish(other)
            follower = JointState()
            follower.name = list(JOINT_NAMES)
            follower.position = [value + 0.01 for value in ARM]
            self.follower.publish(follower)
            odom = Odometry()
            odom.twist.twist.linear.x = 0.04
            odom.twist.twist.linear.y = -0.01
            odom.twist.twist.angular.z = 0.08
            self.odom.publish(odom)
            mode = String()
            mode.data = "vla"
            self.mode.publish(mode)

    fake = _FakeTeleop()
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    executor.add_node(fake)
    start = fake.create_client(Trigger, "/ohho/record/start")
    stop = fake.create_client(Trigger, "/ohho/record/stop")
    try:
        deadline = time.time() + 5.0
        while not start.service_is_ready() or not stop.service_is_ready():
            fake.publish_once()
            executor.spin_once(timeout_sec=0.05)
            if time.time() > deadline:
                raise SystemExit("recorder services were not ready")
        for _ in range(10):
            fake.publish_once()
            executor.spin_once(timeout_sec=0.05)
        started = start.call_async(Trigger.Request())
        while not started.done():
            fake.publish_once()
            executor.spin_once(timeout_sec=0.05)
        if not started.result().success:
            raise SystemExit(f"start failed: {started.result().message}")
        end = time.time() + 1.0
        while time.time() < end:
            fake.publish_once()
            executor.spin_once(timeout_sec=0.05)
        stopped = stop.call_async(Trigger.Request())
        while not stopped.done():
            executor.spin_once(timeout_sec=0.05)
        if not stopped.result().success:
            raise SystemExit(f"stop failed: {stopped.result().message}")
    finally:
        executor.shutdown()
        recorder.destroy_node()
        fake.destroy_node()
        rclpy.shutdown()

    info_path = os.path.join(output_dir, "meta", "info.json")
    episode_path = os.path.join(output_dir, "meta", "episodes.jsonl")
    parquet_path = os.path.join(output_dir, "data", "chunk-000", "episode_000000.parquet")
    if not os.path.isfile(parquet_path):
        raise SystemExit(f"no episode written under {output_dir}")
    import pyarrow.parquet as pq

    table = pq.read_table(parquet_path)
    actions = table["action"].to_pylist()
    expected = ARM + list(BASE)

    def _matches(row):
        return len(row) == 9 and all(
            abs(float(got) - want) <= 1e-5 for got, want in zip(row, expected)
        )

    if not actions or not _matches(actions[-1]):
        raise SystemExit(f"last action {actions[-1] if actions else None} != {expected}")
    if any(abs(float(value) - 1.0) < 1e-6 for value in actions[-1][:6]):
        raise SystemExit("recorded the mux joint target while control mode was vla")
    action = [float(value) for value in actions[-1]]
    info = json.loads(open(info_path, encoding="utf-8").read())
    episode = json.loads(open(episode_path, encoding="utf-8").read().strip())
    print(
        f"fake VR topics wrote {info['total_frames']} frames to {output_dir} "
        f"operator={episode['operator']} robot_kind={episode['robot_kind']} "
        f"sim_world={episode['sim_world']} task={episode['task']}"
    )
    print(f"action={action}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except Exception as exc:
        print(f"fake VR smoke failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
