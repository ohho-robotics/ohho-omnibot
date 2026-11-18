#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Scripted virtual headset for the apartment pick.

Publishes the VR client contract and asks teleop_recorder_node to store
episodes with action_source:=vr. Joint targets step at most 0.08 rad from
the measured arm pose. The deadman is /ohho/heartbeat at 10 Hz.
teleop_safety starts latched, so this client calls /ohho/estop_reset
before the first command.

This process does not publish plant commands or an e-stop. teleop_safety
and the muxes have to forward the client topics, and the measured arm has
to move. If the gated topics stay silent or the joints do not leave the
pose measured at the start, the episode is discarded and the process exits 1.

Refuses to start unless ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID is a
canonical integer from 0 to 232 other than 30.
"""

from __future__ import annotations

import sys
import time

from omnibot_lerobot.scripted_vr_pick import (
    CONTROL_MODE_TOPIC,
    GATED_ARM_TOPIC,
    GATED_BASE_TOPIC,
    HEARTBEAT_TOPIC,
    JOINT_NAMES,
    MIN_JOINT_MOTION_RAD,
    OPERATOR,
    RATE_HZ,
    RAW_ARM_TOPIC,
    RAW_BASE_TOPIC,
    isolation_failure,
    joint_motion_rad,
    step_toward,
    write_collection_record,
)
from omnibot_lerobot.scripted_vr_pick import PHASES
from omnibot_lerobot.vr_action import positions_for_joints


def main(args=None) -> int:
    failure = isolation_failure()
    if failure:
        print(failure, file=sys.stderr)
        return 2

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Empty, String
    from std_srvs.srv import Trigger

    class ScriptedHeadset(Node):
        def __init__(self):
            super().__init__("scripted_vr_headset")
            # Launch writes numeric arguments as integers. Declare the same type.
            self.declare_parameter("episodes", 3)
            self.declare_parameter("dataset_dir", "")
            self.declare_parameter("pick_place_seed", 0)
            self.episodes = int(self.get_parameter("episodes").value)
            dataset = self.get_parameter("dataset_dir").value
            self.dataset_dir = dataset if isinstance(dataset, str) else ""
            self.seed = int(self.get_parameter("pick_place_seed").value)
            if self.episodes < 1:
                raise ValueError("episodes must be >= 1")

            self.arm_pub = self.create_publisher(JointState, RAW_ARM_TOPIC, 10)
            self.base_pub = self.create_publisher(Twist, RAW_BASE_TOPIC, 10)
            self.heart_pub = self.create_publisher(Empty, HEARTBEAT_TOPIC, 10)
            self.mode_pub = self.create_publisher(String, CONTROL_MODE_TOPIC, 10)
            self.create_subscription(JointState, "/arm/joint_states", self._on_joints, 10)
            self.create_subscription(JointState, GATED_ARM_TOPIC, self._on_gated_arm, 10)
            self.create_subscription(Twist, GATED_BASE_TOPIC, self._on_gated_base, 10)
            self.start = self.create_client(Trigger, "/ohho/record/start")
            self.stop = self.create_client(Trigger, "/ohho/record/stop")
            self.discard = self.create_client(Trigger, "/ohho/record/discard")
            self.estop_reset = self.create_client(Trigger, "/ohho/estop_reset")
            self.measured = None
            self.gated_arm = 0
            self.gated_base = 0
            self.saved = 0

        def _on_joints(self, msg):
            positions = positions_for_joints(list(msg.name), list(msg.position), JOINT_NAMES)
            if positions is None:
                return
            self.measured = [float(value) for value in positions]

        def _on_gated_arm(self, _msg):
            self.gated_arm += 1

        def _on_gated_base(self, _msg):
            self.gated_base += 1

        def _publish_mode_and_heartbeat(self):
            mode = String()
            mode.data = "teleop"
            self.mode_pub.publish(mode)
            self.heart_pub.publish(Empty())

        def _publish_command(self, joints, twist):
            self._publish_mode_and_heartbeat()
            arm = JointState()
            arm.name = list(JOINT_NAMES)
            arm.position = [float(value) for value in joints]
            self.arm_pub.publish(arm)
            cmd = Twist()
            cmd.linear.x = float(twist[0])
            cmd.linear.y = float(twist[1])
            cmd.angular.z = float(twist[2])
            self.base_pub.publish(cmd)

        def _spin_once(self, timeout):
            rclpy.spin_once(self, timeout_sec=timeout)

        def _wait_until(self, seconds, ready):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                self._publish_mode_and_heartbeat()
                self._spin_once(0.05)
                if ready():
                    return True
            return False

        def _call(self, client):
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                self._publish_mode_and_heartbeat()
                self._spin_once(0.05)
            if not future.done() or future.result() is None:
                return None
            return future.result()

        def _run_episode(self):
            self.gated_arm = 0
            self.gated_base = 0
            origin = list(self.measured)
            peak = 0.0
            commanded = list(self.measured)
            period = 1.0 / RATE_HZ
            for phase in PHASES:
                for _ in range(phase.ticks):
                    tick = time.monotonic()
                    commanded = list(step_toward(self.measured or commanded, phase.joints))
                    self._publish_command(commanded, phase.twist)
                    while time.monotonic() - tick < period:
                        self._spin_once(0.02)
                    peak = max(peak, joint_motion_rad(origin, self.measured))
            forwarded = self.gated_arm > 0 and self.gated_base > 0
            return forwarded, peak > MIN_JOINT_MOTION_RAD

        def run(self) -> int:
            services = (self.start, self.stop, self.discard, self.estop_reset)
            if not self._wait_until(60.0, lambda: all(client.service_is_ready() for client in services)):
                self.get_logger().error("recorder services were not ready")
                return 1
            if not self._wait_until(90.0, lambda: self.measured is not None):
                self.get_logger().error("/arm/joint_states was not ready")
                return 1
            cleared = self._call(self.estop_reset)
            if cleared is None or not cleared.success:
                message = cleared.message if cleared is not None else "no response"
                self.get_logger().error(f"/ohho/estop_reset failed: {message}")
                return 1
            for index in range(self.episodes):
                started = self._call(self.start)
                if started is None or not started.success:
                    message = started.message if started is not None else "no response"
                    self.get_logger().error(f"record start failed: {message}")
                    return 1
                forwarded, moved = self._run_episode()
                if not forwarded or not moved:
                    self._call(self.discard)
                    if not forwarded:
                        self.get_logger().error(
                            "teleop_safety did not forward headset commands on "
                            f"{GATED_ARM_TOPIC} and {GATED_BASE_TOPIC}"
                        )
                    else:
                        self.get_logger().error(
                            "/arm/joint_states did not move during the episode"
                        )
                    return 1
                stopped = self._call(self.stop)
                if stopped is None or not stopped.success:
                    message = stopped.message if stopped is not None else "no response"
                    self.get_logger().error(f"record stop failed: {message}")
                    return 1
                self.saved += 1
                self.get_logger().info(
                    f"saved scripted-sim episode {index} operator={OPERATOR}"
                )
            if self.dataset_dir:
                path = write_collection_record(self.dataset_dir, self.seed, self.saved)
                self.get_logger().info(f"wrote {path}")
            print(
                "SCRIPTED_VR_PICK "
                f"episodes={self.saved} operator={OPERATOR} "
                f"human_vr=false seed={self.seed}",
                flush=True,
            )
            return 0

    rclpy.init(args=args)
    node = ScriptedHeadset()
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except Exception as exc:
        print(f"scripted VR headset failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
