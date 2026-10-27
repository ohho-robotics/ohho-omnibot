#!/usr/bin/env python3
"""Gazebo sim driver for the SO-101 arm.

Topics published:
  /arm/joint_states                         sensor_msgs/JointState
  /arm/sim/<joint>/cmd_pos                  std_msgs/Float64  (one per joint)

Topics subscribed:
  /arm/joint_commands/out                   sensor_msgs/JointState
  /arm/sim/joint_states                     sensor_msgs/JointState  (from gz)
  /arm/enable                               std_msgs/Bool
  /emergency_stop                           std_msgs/Bool

Safety lives in arm_safety.py via arm_sim_logic.ArmSimCommander. This node
reads arm_params.yaml, calls that helper, and writes position targets.
Position commands only. An e-stop freezes the sim joints at the measured
pose and rejects later commands. Untested on hardware.
"""

import os
import time

from arm_sim_logic import FEEDBACK_TOPIC
from arm_sim_logic import bundled_arm_params_path
from arm_sim_logic import command_topic
from arm_sim_logic import fallback_arm_params
from arm_sim_logic import load_arm_driver_params
from arm_sim_logic import ArmSimCommander

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64


class ArmSimDriverNode(Node):
    """ROS wrapper that applies ArmSafety and commands Gazebo joint plugins."""

    def __init__(self):
        super().__init__("arm_sim_driver_node")

        self.declare_parameter("arm_params_file", "")
        path = self.get_parameter("arm_params_file").value or ""
        if not path:
            candidate = bundled_arm_params_path()
            if os.path.isfile(candidate):
                path = candidate
        if path and os.path.isfile(path):
            params = load_arm_driver_params(path)
            self.params_path = path
        else:
            params = fallback_arm_params()
            self.params_path = ""
            self.get_logger().error(
                "arm_params.yaml was not found. Using fallback limits that "
                "match the checked-in arm_params.yaml. Pass arm_params_file."
            )

        self.joint_names = list(params["joint_names"])
        self.joint_min = [float(value) for value in params["joint_min"]]
        self.joint_max = [float(value) for value in params["joint_max"]]
        self.publish_rate = float(params["publish_rate"])
        self._commander = ArmSimCommander.from_driver_params(params)

        self.joint_state_pub = self.create_publisher(
            JointState, "/arm/joint_states", 10
        )
        self._cmd_pubs = []
        for name in self.joint_names:
            self._cmd_pubs.append(
                self.create_publisher(Float64, command_topic(name), 10)
            )

        self.create_subscription(
            JointState, "/arm/joint_commands/out", self.joint_command_cb, 10
        )
        self.create_subscription(
            JointState, FEEDBACK_TOPIC, self.feedback_cb, 10
        )
        self.create_subscription(Bool, "/arm/enable", self.enable_cb, 10)
        self.create_subscription(
            Bool, "/emergency_stop", self.emergency_stop_cb, 10
        )

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.01
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            "ArmSimDriverNode started | "
            f"params={self.params_path or 'fallback'} | "
            f"joints={self.joint_names} | "
            f"max_joint_delta_rad={self._commander.safety.max_joint_delta_rad} | "
            f"command_timeout_sec={self._commander.safety.command_timeout_sec}"
        )

    def _publish_positions(self, positions):
        for publisher, position in zip(self._cmd_pubs, positions):
            msg = Float64()
            msg.data = float(position)
            publisher.publish(msg)

    def _publish_joint_state(self):
        if not self._commander.have_feedback:
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self._commander.joint_names)
        js.position = list(self._commander.measured)
        self.joint_state_pub.publish(js)

    def _log_action(self, action):
        if not action.message:
            return
        if action.level == "warn":
            self.get_logger().warn(action.message)
        else:
            self.get_logger().info(action.message)

    def joint_command_cb(self, msg):
        written = self._commander.on_command(
            list(msg.name), list(msg.position), time.monotonic()
        )
        if written is None:
            return
        self._publish_positions(written)

    def feedback_cb(self, msg):
        if self._commander.on_feedback(list(msg.name), list(msg.position)):
            self._publish_joint_state()

    def enable_cb(self, msg):
        action, written = self._commander.on_arm_enable(
            bool(msg.data), time.monotonic()
        )
        self._log_action(action)
        if written is not None:
            self._publish_positions(written)

    def emergency_stop_cb(self, msg):
        """Reject further targets. Freeze Gazebo at the measured pose."""
        action, hold = self._commander.on_emergency_stop(bool(msg.data))
        self._log_action(action)
        if hold is not None:
            self._publish_positions(hold)

    def _on_timer(self):
        action = self._commander.hold_if_stale(time.monotonic())
        if action.message:
            self.get_logger().warn(action.message)
        if action.rewrite and action.positions is not None:
            self._publish_positions(action.positions)
        self._publish_joint_state()


def main(args=None):
    rclpy.init(args=args)
    node = ArmSimDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Launch delivers SIGINT, and rclpy's handler already shut the context down.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
