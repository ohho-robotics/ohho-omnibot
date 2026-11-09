#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Keep the existing command topics working on top of ros2_control.

Sim (command_source:=cmd_pos) listens to the arm_sim_driver Float64 topics
and republishes the arm slice of /joint_states on /arm/sim/joint_states.
The robot (command_source:=joint_commands) applies arm_safety to
/arm/joint_commands/out and republishes the arm slice on /arm/joint_states.

Both modes turn a Twist into TwistStamped on the mecanum controller's
reference topic, and drop that reference when it goes stale or e-stop is set.
"""

import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from ros2_control_adapter_logic import ALL_ARM_JOINTS
from ros2_control_adapter_logic import ARM_JOINTS
from ros2_control_adapter_logic import GRIPPER_JOINT
from ros2_control_adapter_logic import TRAJECTORY_SEC
from ros2_control_adapter_logic import CmdPosTracker
from ros2_control_adapter_logic import TwistRelay
from ros2_control_adapter_logic import arm_subset


def _load_arm_safety():
    """Import arm_safety from the installed omnibot_arm lib directory."""
    from ament_index_python.packages import get_package_prefix

    lib = os.path.join(get_package_prefix("omnibot_arm"), "lib", "omnibot_arm")
    if lib not in sys.path:
        sys.path.insert(0, lib)
    import arm_safety

    return arm_safety


def _load_limits(path):
    import yaml

    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    params = data["arm_driver_node"]["ros__parameters"]
    return (
        [float(value) for value in params["joint_min"]],
        [float(value) for value in params["joint_max"]],
        float(params["max_joint_delta_rad"]),
        float(params["command_timeout_sec"]),
    )


class Ros2ControlAdapter(Node):
    def __init__(self):
        super().__init__("ros2_control_adapter")
        self.declare_parameter("command_source", "cmd_pos")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("reference_topic", "/mecanum_drive_controller/reference")
        self.declare_parameter("odom_in_topic", "/mecanum_drive_controller/odometry")
        self.declare_parameter("relay_odom", False)
        self.declare_parameter("publish_odom_tf", False)
        self.declare_parameter("publish_arm_feedback_topic", "")
        self.declare_parameter("publish_arm_joint_states", False)
        self.declare_parameter("arm_params_file", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("trajectory_topic", "/arm_joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("gripper_action", "/gripper_controller/gripper_cmd")
        self.declare_parameter("twist_timeout_sec", 0.25)
        self.declare_parameter("publish_hz", 20.0)

        self._source = str(self.get_parameter("command_source").value)
        self._relay_odom = bool(self.get_parameter("relay_odom").value)
        self._publish_tf = bool(self.get_parameter("publish_odom_tf").value)
        self._publish_arm_joint_states = bool(self.get_parameter("publish_arm_joint_states").value)
        feedback = str(self.get_parameter("publish_arm_feedback_topic").value)
        self._estop = False
        self._measured = [0.0] * len(ALL_ARM_JOINTS)
        self._have_measured = False
        self._safety = None
        self._tracker = CmdPosTracker()
        self._twists = TwistRelay(float(self.get_parameter("twist_timeout_sec").value))
        self._arm_goal = None
        self._grip_goal = None
        self._arm_dirty = False
        self._grip_dirty = False
        self._pending_gripper = None

        self._reference = self.create_publisher(
            TwistStamped, str(self.get_parameter("reference_topic").value), 10
        )
        self._trajectory = self.create_publisher(
            JointTrajectory, str(self.get_parameter("trajectory_topic").value), 10
        )
        self._feedback_pub = None
        if feedback:
            self._feedback_pub = self.create_publisher(JointState, feedback, 10)
        self._arm_state_pub = None
        if self._publish_arm_joint_states:
            self._arm_state_pub = self.create_publisher(JointState, "/arm/joint_states", 10)
        self._odom_pub = None
        self._tf = None
        if self._relay_odom or self._publish_tf:
            if self._relay_odom:
                self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
            if self._publish_tf:
                from tf2_ros import TransformBroadcaster

                self._tf = TransformBroadcaster(self)
            self.create_subscription(
                Odometry,
                str(self.get_parameter("odom_in_topic").value),
                self._on_odom,
                10,
            )

        from control_msgs.action import ParallelGripperCommand

        self._gripper_type = ParallelGripperCommand
        self._gripper = ActionClient(
            self, ParallelGripperCommand, str(self.get_parameter("gripper_action").value)
        )

        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_topic").value), self._on_twist, 10)
        self.create_subscription(Bool, "/emergency_stop", self._on_estop, 10)
        self.create_subscription(
            JointState, str(self.get_parameter("joint_states_topic").value), self._on_joint_states, 20
        )

        if self._source == "cmd_pos":
            for name in ALL_ARM_JOINTS:
                self.create_subscription(
                    Float64,
                    f"/arm/sim/{name}/cmd_pos",
                    lambda msg, joint=name: self._on_cmd_pos(joint, msg),
                    10,
                )
        elif self._source == "joint_commands":
            self._bind_arm_safety()
            self.create_subscription(JointState, "/arm/joint_commands/out", self._on_joint_command, 10)
            self.create_subscription(Bool, "/arm/enable", self._on_enable, 10)
        else:
            self.get_logger().error(
                f"Unknown command_source {self._source!r}; not forwarding arm commands"
            )

        period = 1.0 / float(self.get_parameter("publish_hz").value)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            "ros2_control adapter"
            f" source={self._source}"
            f" cmd_vel={self.get_parameter('cmd_vel_topic').value}"
            f" relay_odom={self._relay_odom}"
        )

    def _bind_arm_safety(self):
        path = str(self.get_parameter("arm_params_file").value)
        try:
            arm_safety = _load_arm_safety()
            joint_min, joint_max, max_delta, timeout = _load_limits(path)
        except Exception as exc:
            self.get_logger().error(
                "arm_safety did not load, so /arm/joint_commands/out is not forwarded: "
                f"{exc}"
            )
            self._safety = None
            return
        self._safety = arm_safety.ArmSafety(
            joint_min,
            joint_max,
            max_joint_delta_rad=max_delta,
            command_timeout_sec=timeout,
        )

    def _note_arm(self, positions):
        arm = tuple(float(value) for value in positions[:5])
        grip = float(positions[5])
        if self._arm_goal is None or any(abs(a - b) > 1e-4 for a, b in zip(arm, self._arm_goal)):
            self._arm_goal = arm
            self._arm_dirty = True
        if self._grip_goal is None or abs(grip - self._grip_goal) > 1e-4:
            self._grip_goal = grip
            self._grip_dirty = True

    def _on_cmd_pos(self, joint, msg):
        self._tracker.update(joint, msg.data)

    def _on_joint_command(self, msg):
        if self._safety is None:
            return
        index = {name: i for i, name in enumerate(msg.name)}
        commanded = [
            float(msg.position[index[name]]) if name in index else 0.0 for name in ALL_ARM_JOINTS
        ]
        reference = list(self._measured) if self._have_measured else None
        action = self._safety.accept_command(commanded, time.monotonic(), reference)
        if action.write:
            self._note_arm(action.positions)

    def _on_enable(self, msg):
        if self._safety is None:
            return
        action = self._safety.on_arm_enable(bool(msg.data))
        if action.value == 1 and self._have_measured:
            seeded = self._safety.seed_target(list(self._measured), time.monotonic())
            self._note_arm(seeded)

    def _on_estop(self, msg):
        self._estop = bool(msg.data)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._estop:
            self._twists.force_zero(now)
        if self._safety is None:
            return
        self._safety.on_emergency_stop(self._estop)
        if self._estop and self._have_measured:
            self._note_arm(self._measured)

    def _on_twist(self, msg):
        if self._estop:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self._twists.update(
            (msg.linear.x, msg.linear.y, msg.linear.z),
            (msg.angular.x, msg.angular.y, msg.angular.z),
            now,
        )

    def _on_joint_states(self, msg):
        subset = arm_subset(list(msg.name), list(msg.position), list(msg.velocity))
        if subset is None:
            return
        names, positions, velocities = subset
        self._measured = positions
        self._have_measured = True
        self._publish_arm(names, positions, velocities)

    def _publish_arm(self, names, positions, velocities):
        if self._feedback_pub is None and self._arm_state_pub is None:
            return
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(names)
        state.position = list(positions)
        if velocities is not None:
            state.velocity = list(velocities)
        if self._feedback_pub is not None:
            self._feedback_pub.publish(state)
        if self._arm_state_pub is not None:
            self._arm_state_pub.publish(state)

    def _on_odom(self, msg):
        if self._odom_pub is not None:
            self._odom_pub.publish(msg)
        if self._tf is None:
            return
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(transform)

    def _publish_trajectory(self, positions):
        message = JointTrajectory()
        message.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = Duration(
            sec=0, nanosec=int(TRAJECTORY_SEC * 1e9)
        )
        message.points = [point]
        self._trajectory.publish(message)

    def _send_gripper(self, position):
        self._pending_gripper = float(position)
        self._flush_gripper()

    def _flush_gripper(self):
        if self._pending_gripper is None or not self._gripper.server_is_ready():
            return
        goal = self._gripper_type.Goal()
        goal.command.name = [GRIPPER_JOINT]
        goal.command.position = [self._pending_gripper]
        self._gripper.send_goal_async(goal)
        self._pending_gripper = None

    def _on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        sample = self._twists.sample(now)
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = "base_footprint"
        stamped.twist.linear.x = sample[0]
        stamped.twist.linear.y = sample[1]
        stamped.twist.linear.z = sample[2]
        stamped.twist.angular.x = sample[3]
        stamped.twist.angular.y = sample[4]
        stamped.twist.angular.z = sample[5]
        self._reference.publish(stamped)

        if self._source == "cmd_pos":
            arm = self._tracker.trajectory_if_changed()
            if arm is not None:
                self._publish_trajectory(arm)
            grip = self._tracker.gripper_if_changed()
            if grip is not None:
                self._send_gripper(grip)
        elif self._safety is not None:
            hold = self._safety.hold_if_stale(time.monotonic())
            if hold.rewrite and hold.positions is not None:
                self._note_arm(hold.positions)
            if self._arm_dirty and self._arm_goal is not None:
                self._publish_trajectory(self._arm_goal)
                self._arm_dirty = False
            if self._grip_dirty and self._grip_goal is not None:
                self._send_gripper(self._grip_goal)
                self._grip_dirty = False
        self._flush_gripper()


def main(args=None):
    rclpy.init(args=args)
    node = Ros2ControlAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
