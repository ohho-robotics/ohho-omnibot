# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Headless mock hardware: one OMPL plan, Pilz PTP, Pilz LIN, and the gripper.

Colcon runs this. It is not in the CPU pytest list. Without ROS the module skips.
"""

import math
import os
import subprocess
import time
import unittest
import xml.etree.ElementTree as ET

import pytest

pytest.importorskip("launch_testing")
pytest.importorskip("rclpy")

import launch_testing
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Quaternion
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

ARM = [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
]
GRIPPER = "arm_gripper"
TOOL = "arm_gripper_frame_link"
CONTROLLERS = (
    "joint_state_broadcaster",
    "mecanum_drive_controller",
    "arm_joint_trajectory_controller",
    "gripper_controller",
)
ARM_TOLERANCE = 0.05
GRIPPER_TOLERANCE = 0.08
LIN_TOLERANCE_M = 0.015
BUDGET_SEC = 340.0

ERROR_NAMES = {
    getattr(MoveItErrorCodes, name): name
    for name in (
        "SUCCESS",
        "PLANNING_FAILED",
        "INVALID_MOTION_PLAN",
        "CONTROL_FAILED",
        "TIMED_OUT",
        "START_STATE_IN_COLLISION",
        "GOAL_IN_COLLISION",
        "NO_IK_SOLUTION",
    )
    if hasattr(MoveItErrorCodes, name)
}


def generate_test_description():
    share = FindPackageShare("omnibot_moveit_config").find("omnibot_moveit_config")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "demo.launch.py")
                ),
                launch_arguments={
                    "rviz": "false",
                    "gz": "false",
                    "gui": "false",
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {}


def _group_state(root, group, name):
    node = root.find(f"group_state[@name='{name}'][@group='{group}']")
    assert node is not None, name
    return {
        joint.get("name"): float(joint.get("value")) for joint in node.findall("joint")
    }


def _sample_indexes(count, limit):
    if count <= limit:
        return list(range(count))
    step = (count - 1) / (limit - 1)
    return sorted({int(round(index * step)) for index in range(limit)})


class TestMoveItPlanExecute(unittest.TestCase):
    def setUp(self):
        self._own_rclpy = False
        if not rclpy.ok():
            rclpy.init()
            self._own_rclpy = True
        self.node = rclpy.create_node("moveit_plan_execute_test")
        self.joints = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.action = ActionClient(self.node, MoveGroup, "move_action")
        self.validity = self.node.create_client(GetStateValidity, "check_state_validity")
        share = FindPackageShare("omnibot_moveit_config").find("omnibot_moveit_config")
        self.srdf = ET.parse(os.path.join(share, "config", "mecanum_bot.srdf")).getroot()
        self.deadline = time.monotonic() + BUDGET_SEC

    def tearDown(self):
        self.node.destroy_node()
        if self._own_rclpy and rclpy.ok():
            rclpy.shutdown()

    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joints[name] = position

    def _remaining(self):
        return self.deadline - time.monotonic()

    def _spin_until(self, predicate, label):
        while self._remaining() > 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail(label + f" (joints={self.joints})")

    def _wait_future(self, future, timeout, label):
        limit = min(timeout, max(0.0, self._remaining()))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=limit)
        self.assertTrue(future.done(), label + " timed out")

    def test_plans_execute_without_self_collision(self):
        self._wait_controllers()
        self._spin_until(
            lambda: self.action.server_is_ready() and self.validity.service_is_ready(),
            "move_group action or check_state_validity did not come up",
        )
        self._spin_until(
            lambda: all(name in self.joints for name in ARM + [GRIPPER]),
            "arm joint states missing",
        )
        self._tool()
        disabled = {
            frozenset((tag.get("link1"), tag.get("link2")))
            for tag in self.srdf.findall("disable_collisions")
        }
        self.assertIn(frozenset(("arm_wrist_link", "wrist_camera_link")), disabled)
        self.assertNotIn(frozenset(("arm_lower_arm_link", "top_plate_link")), disabled)
        self._assert_valid(list(self.joints), list(self.joints.values()), "start")

        ready = _group_state(self.srdf, "arm", "ready")
        home = _group_state(self.srdf, "arm", "home")
        opened = _group_state(self.srdf, "gripper", "gripper_open")
        closed = _group_state(self.srdf, "gripper", "gripper_closed")

        print("OMPL RRTConnect -> ready", flush=True)
        self._execute("ompl ready", "arm", "ompl", "RRTConnect", joints=ready)
        self._wait_joints(ready, ARM_TOLERANCE)

        print("Pilz PTP -> home", flush=True)
        self._execute(
            "pilz ptp home",
            "arm",
            "pilz_industrial_motion_planner",
            "PTP",
            joints=home,
        )
        self._wait_joints(home, ARM_TOLERANCE)

        print("Pilz LIN tool position", flush=True)
        x_m, y_m, z_m, quat = self._tool()
        target = (x_m - 0.03, y_m, z_m - 0.02, quat)
        self._execute(
            "pilz lin",
            "arm",
            "pilz_industrial_motion_planner",
            "LIN",
            pose=target,
        )
        got = self._tool()
        error = math.dist((got[0], got[1], got[2]), target[:3])
        self.assertLess(error, LIN_TOLERANCE_M, f"LIN tool error {error}")
        arm_after_lin = {name: self.joints[name] for name in ARM}

        print("gripper open then closed", flush=True)
        self._execute(
            "gripper open",
            "gripper",
            "pilz_industrial_motion_planner",
            "PTP",
            joints=opened,
        )
        self._wait_joints(opened, GRIPPER_TOLERANCE)
        self._execute(
            "gripper closed",
            "gripper",
            "pilz_industrial_motion_planner",
            "PTP",
            joints=closed,
        )
        self._wait_joints(closed, GRIPPER_TOLERANCE)
        self._wait_joints(arm_after_lin, ARM_TOLERANCE)
        self._assert_valid(list(self.joints), list(self.joints.values()), "final")

    def _wait_controllers(self):
        last = ""
        while self._remaining() > 0:
            completed = subprocess.run(
                ["ros2", "control", "list_controllers"],
                capture_output=True,
                text=True,
                timeout=max(1.0, min(30.0, self._remaining())),
                check=False,
            )
            last = completed.stdout + completed.stderr
            if _all_active(last):
                print(last, flush=True)
                return
            time.sleep(2.0)
        self.fail("mock controllers did not become active:\n" + last)

    def _tool(self):
        pose = {}

        def _lookup():
            try:
                transform = self.buffer.lookup_transform("world", TOOL, Time())
            except TransformException:
                return False
            translation = transform.transform.translation
            pose["value"] = (
                translation.x,
                translation.y,
                translation.z,
                transform.transform.rotation,
            )
            return True

        self._spin_until(_lookup, "no TF world -> " + TOOL)
        return pose["value"]

    def _execute(self, label, group, pipeline, planner, joints=None, pose=None):
        request = MotionPlanRequest()
        request.group_name = group
        request.pipeline_id = pipeline
        request.planner_id = planner
        request.num_planning_attempts = 3
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        request.start_state = self._robot_state(list(self.joints), list(self.joints.values()))
        request.workspace_parameters.header.frame_id = "world"
        request.workspace_parameters.min_corner.x = -2.0
        request.workspace_parameters.min_corner.y = -2.0
        request.workspace_parameters.min_corner.z = -2.0
        request.workspace_parameters.max_corner.x = 2.0
        request.workspace_parameters.max_corner.y = 2.0
        request.workspace_parameters.max_corner.z = 2.0
        constraints = Constraints()
        if joints is not None:
            for name, value in joints.items():
                constraint = JointConstraint()
                constraint.joint_name = name
                constraint.position = float(value)
                constraint.tolerance_above = 0.01
                constraint.tolerance_below = 0.01
                constraint.weight = 1.0
                constraints.joint_constraints.append(constraint)
        if pose is not None:
            x_m, y_m, z_m, quat = pose
            position = PositionConstraint()
            position.header.frame_id = "world"
            position.link_name = TOOL
            position.weight = 1.0
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [0.01]
            position.constraint_region.primitives.append(sphere)
            region = Pose()
            region.position.x = float(x_m)
            region.position.y = float(y_m)
            region.position.z = float(z_m)
            region.orientation.w = 1.0
            position.constraint_region.primitive_poses.append(region)
            orientation = OrientationConstraint()
            orientation.header.frame_id = "world"
            orientation.link_name = TOOL
            orientation.orientation = Quaternion(
                x=quat.x, y=quat.y, z=quat.z, w=quat.w
            )
            orientation.absolute_x_axis_tolerance = math.tau
            orientation.absolute_y_axis_tolerance = math.tau
            orientation.absolute_z_axis_tolerance = math.tau
            orientation.weight = 1.0
            constraints.position_constraints.append(position)
            constraints.orientation_constraints.append(orientation)
        request.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0
        send_future = self.action.send_goal_async(goal)
        self._wait_future(send_future, 20.0, label + " accept")
        handle = send_future.result()
        self.assertTrue(handle.accepted, label + " was rejected")
        result_future = handle.get_result_async()
        self._wait_future(result_future, 70.0, label + " result")
        wrapped = result_future.result()
        code = wrapped.result.error_code.val
        name = ERROR_NAMES.get(code, "")
        self.assertEqual(
            wrapped.status,
            GoalStatus.STATUS_SUCCEEDED,
            f"{label} status {wrapped.status} error {code} {name}",
        )
        self.assertEqual(
            code,
            MoveItErrorCodes.SUCCESS,
            f"{label} error {code} {name}",
        )
        trajectory = wrapped.result.planned_trajectory.joint_trajectory
        self.assertGreater(len(trajectory.points), 0, label + " empty trajectory")
        for index in _sample_indexes(len(trajectory.points), 12):
            point = trajectory.points[index]
            self._assert_valid(
                list(trajectory.joint_names),
                list(point.positions),
                f"{label} waypoint {index}",
            )
        return wrapped.result

    def _robot_state(self, names, positions):
        state = RobotState()
        state.is_diff = True
        state.joint_state.name = list(names)
        state.joint_state.position = [float(value) for value in positions]
        return state

    def _assert_valid(self, names, positions, label):
        request = GetStateValidity.Request()
        request.robot_state = self._robot_state(names, positions)
        request.group_name = ""
        future = self.validity.call_async(request)
        self._wait_future(future, 15.0, label + " validity")
        response = future.result()
        contacts = [
            f"{contact.contact_body_1}/{contact.contact_body_2}"
            for contact in response.contacts
        ]
        self.assertTrue(
            response.valid,
            f"{label} self-collision {contacts} error {response.error_code.val}",
        )

    def _wait_joints(self, targets, tolerance):
        def _done():
            return all(
                name in self.joints and abs(self.joints[name] - value) <= tolerance
                for name, value in targets.items()
            )

        self._spin_until(_done, f"joints did not reach {targets}")


def _all_active(text):
    for name in CONTROLLERS:
        matched = False
        for line in text.splitlines():
            if name in line and "active" in line:
                matched = True
        if not matched:
            return False
    return True
