#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Reflect /arm/joint_commands/out onto /arm/joint_states.

sim_vr.launch.py arm:=true includes this so a rosbridge client can see the
arm mux output. Measured seeds are the zero pose, which gives teleop_safety
a joint reference. The node does not read the raw headset command, does not command
hardware, and does not change SafetyGate clamps. OHH-199's Gazebo
arm_sim_driver is a different publisher of /arm/joint_states.
"""

import math

# Same names as teleop_gate.JOINT_LIMITS. The contract test moves the first three.
JOINT_NAMES = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)


def apply_joint_command(measured, names, positions):
    """Copy finite known joints from a mux command. Leave the rest alone."""
    updated = dict(measured)
    if len(names) != len(positions):
        return updated
    for name, position in zip(names, positions):
        if name not in updated:
            continue
        try:
            value = float(position)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(value):
            continue
        updated[name] = value
    return updated


def _spin():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    class ArmJointStateMirror(Node):
        """Publish the last mux output as the measured arm pose."""

        def __init__(self):
            super().__init__("arm_joint_state_mirror")
            self._positions = {name: 0.0 for name in JOINT_NAMES}
            self._pub = self.create_publisher(JointState, "/arm/joint_states", 10)
            self.create_subscription(
                JointState, "/arm/joint_commands/out", self._on_command, 10
            )
            self.create_timer(0.05, self._publish)
            self.get_logger().info(
                "arm_joint_state_mirror publishing /arm/joint_states from "
                "/arm/joint_commands/out"
            )

        def _on_command(self, msg):
            self._positions = apply_joint_command(
                self._positions, list(msg.name), list(msg.position)
            )

        def _publish(self):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(JOINT_NAMES)
            msg.position = [self._positions[name] for name in JOINT_NAMES]
            self._pub.publish(msg)

    rclpy.init()
    node = ArmJointStateMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    del args
    _spin()


if __name__ == "__main__":
    main()
