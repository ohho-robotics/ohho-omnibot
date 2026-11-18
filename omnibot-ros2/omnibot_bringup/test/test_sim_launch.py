# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Headless launch test for sim.launch.py world:=flat.

Asserts /odom and /scan publish within 30 s and that /cmd_vel moves the
robot at least 0.2 m. Also waits for /imu and /camera/front/image_raw, the
other topics the bridge is required to expose. A second test runs
arm_sim_scripted_move.py: each SO-101 joint reaches its target and an
e-stop holds that pose.

Set OMNIBOT_SIM_FRAME_DIR to a directory to save front-camera PPM frames
while the robot moves (used to build the README GIF). The assertion does
not depend on that directory.
"""

import math
import os
import subprocess
import sys
import time
import unittest

import launch_testing
import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
)
GRIPPER_JOINT = "arm_gripper"
ACTIVE_CONTROLLERS = (
    "joint_state_broadcaster",
    "mecanum_drive_controller",
    "arm_joint_trajectory_controller",
    "gripper_controller",
)


def generate_test_description():
    bringup = FindPackageShare("omnibot_bringup").find("omnibot_bringup")
    launch_file = os.path.join(bringup, "launch", "sim.launch.py")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "world": "flat",
                    "gui": "false",
                    "drive": "planar",
                    "rosbridge": "false",
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {}


def _log(text):
    print(text, flush=True)


def _save_ppm(msg, path):
    # sensor_msgs/Image rgb8, tightly packed or with step padding.
    channels = 3
    with open(path, "wb") as handle:
        handle.write(f"P6\n{msg.width} {msg.height}\n255\n".encode())
        raw = bytes(msg.data)
        row = msg.width * channels
        if msg.step == row:
            handle.write(raw)
            return
        for y in range(msg.height):
            start = y * msg.step
            handle.write(raw[start : start + row])


class TestSimSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("omnibot_sim_smoke")
        cls.odom = []
        cls.scan = []
        cls.imu = []
        cls.images = []
        cls.joints = []
        cls.node.create_subscription(Odometry, "/odom", cls.odom.append, 10)
        cls.node.create_subscription(LaserScan, "/scan", cls.scan.append, 10)
        cls.node.create_subscription(Imu, "/imu", cls.imu.append, 10)
        cls.node.create_subscription(Image, "/camera/front/image_raw", cls.images.append, 10)
        cls.node.create_subscription(JointState, "/joint_states", cls.joints.append, 10)
        cls.cmd = cls.node.create_publisher(Twist, "/cmd_vel", 10)
        cls.arm = ActionClient(
            cls.node,
            FollowJointTrajectory,
            "/arm_joint_trajectory_controller/follow_joint_trajectory",
        )
        cls.gripper = ActionClient(
            cls.node,
            ParallelGripperCommand,
            "/gripper_controller/gripper_cmd",
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, deadline, ready):
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if ready():
                return True
        return False

    def test_odom_scan_and_motion(self):
        deadline = time.monotonic() + 30.0
        topics_ok = self._spin_until(
            deadline,
            lambda: self.odom and self.scan and self.imu and self.images,
        )
        _log(
            "EVIDENCE topic_counts "
            f"odom={len(self.odom)} scan={len(self.scan)} "
            f"imu={len(self.imu)} camera={len(self.images)}"
        )
        self.assertTrue(self.odom, "/odom did not publish within 30 s")
        self.assertTrue(self.scan, "/scan did not publish within 30 s")
        self.assertTrue(self.imu, "/imu did not publish within 30 s")
        self.assertTrue(
            self.images, "/camera/front/image_raw did not publish within 30 s"
        )
        self.assertTrue(topics_ok, "sensor topics were late")
        self.assertGreater(len(self.scan[-1].ranges), 0)

        start = self.odom[-1].pose.pose.position
        _log(
            "EVIDENCE /odom start "
            f"frame_id={self.odom[-1].header.frame_id} "
            f"x={start.x:.4f} y={start.y:.4f}"
        )
        _log(
            "EVIDENCE /scan "
            f"frame_id={self.scan[-1].header.frame_id} "
            f"ranges={len(self.scan[-1].ranges)}"
        )

        twist = Twist()
        twist.linear.x = 0.5
        frame_dir = os.environ.get("OMNIBOT_SIM_FRAME_DIR", "")
        if frame_dir:
            os.makedirs(frame_dir, exist_ok=True)
        saved = 0
        dist = 0.0
        move_deadline = time.monotonic() + 20.0
        while time.monotonic() < move_deadline:
            self.cmd.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            pos = self.odom[-1].pose.pose.position
            dist = math.hypot(pos.x - start.x, pos.y - start.y)
            if frame_dir and self.images and saved < 8:
                path = os.path.join(frame_dir, f"frame_{saved:02d}.ppm")
                _save_ppm(self.images[-1], path)
                saved += 1
            if dist >= 0.2:
                break
        self.cmd.publish(Twist())
        pos = self.odom[-1].pose.pose.position
        _log(
            "EVIDENCE /cmd_vel motion "
            f"x={pos.x:.4f} y={pos.y:.4f} displacement_m={dist:.4f}"
        )
        self.assertGreaterEqual(dist, 0.2)

    def _joint_position(self, name):
        if not self.joints or name not in self.joints[-1].name:
            return None
        message = self.joints[-1]
        return message.position[list(message.name).index(name)]

    def _wait_for_positions(self, targets, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all(
                (pos := self._joint_position(name)) is not None and abs(pos - goal) <= 0.08
                for name, goal in targets.items()
            ):
                return True
        return False

    def _wait_controllers(self):
        deadline = time.monotonic() + 60.0
        last = ""
        while time.monotonic() < deadline:
            completed = subprocess.run(
                ["ros2", "control", "list_controllers"],
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
            last = completed.stdout + completed.stderr
            if all(
                any(name in line and "active" in line for line in last.splitlines())
                for name in ACTIVE_CONTROLLERS
            ):
                _log("EVIDENCE controllers\n" + last)
                return
            time.sleep(2.0)
        self.fail("controllers did not become active:\n" + last)

    def _move_arm(self, positions):
        self.assertTrue(self.arm.wait_for_server(timeout_sec=15.0))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        goal.trajectory.points = [point]
        future = self.arm.send_goal_async(goal)
        accepted_deadline = time.monotonic() + 10.0
        while time.monotonic() < accepted_deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(future.done() and future.result() is not None and future.result().accepted)
        targets = dict(zip(ARM_JOINTS, positions))
        self.assertTrue(
            self._wait_for_positions(targets, 10.0),
            f"arm did not reach {positions}",
        )

    def _move_gripper(self, position):
        self.assertTrue(self.gripper.wait_for_server(timeout_sec=15.0))
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [GRIPPER_JOINT]
        goal.command.position = [position]
        future = self.gripper.send_goal_async(goal)
        accepted_deadline = time.monotonic() + 10.0
        while time.monotonic() < accepted_deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(future.done() and future.result() is not None and future.result().accepted)
        self.assertTrue(
            self._wait_for_positions({GRIPPER_JOINT: position}, 10.0),
            f"gripper did not reach {position}",
        )

    def test_arm_scripted_move_and_estop_hold(self):
        """Controllers, a trajectory, a gripper command, then the e-stop hold."""
        self._wait_controllers()
        for index in range(len(ARM_JOINTS)):
            positions = [0.0] * len(ARM_JOINTS)
            positions[index] = 0.30
            self._move_arm(positions)
        self._move_arm([0.0] * len(ARM_JOINTS))
        self._move_gripper(0.40)
        self._move_gripper(0.05)

        from ament_index_python.packages import get_package_prefix

        script = os.path.join(
            get_package_prefix("omnibot_arm"),
            "lib",
            "omnibot_arm",
            "arm_sim_scripted_move.py",
        )
        self.assertTrue(os.path.isfile(script), script)
        completed = subprocess.run(
            [sys.executable, script],
            capture_output=True,
            text=True,
            timeout=160,
            check=False,
        )
        _log(completed.stdout)
        if completed.stderr:
            _log(completed.stderr)
        self.assertEqual(
            completed.returncode,
            0,
            completed.stdout + "\n" + completed.stderr,
        )
