# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""CI hardware path: ros2_control_node, GenericSystem, no robot and no Gazebo.

Jazzy controller_manager ignores a robot_description parameter and waits for
the /robot_description topic. robot_state_publisher is that publisher.

The four controllers in controllers.yaml must reach active. This file is a
launch test, not part of the CPU pytest list.
"""

import os
import subprocess
import time
import unittest

import launch_testing
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CONTROLLERS = (
    "joint_state_broadcaster",
    "mecanum_drive_controller",
    "arm_joint_trajectory_controller",
    "gripper_controller",
)


def generate_test_description():
    bringup = FindPackageShare("omnibot_bringup").find("omnibot_bringup")
    description = FindPackageShare("omnibot_description").find("omnibot_description")
    xacro_file = os.path.join(description, "urdf", "omnibot.urdf.xacro")
    controllers = os.path.join(bringup, "config", "controllers.yaml")
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " use_sim:=false",
                " use_mock:=true",
            ]
        ),
        value_type=str,
    )
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": False},
                ],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                output="screen",
                parameters=[
                    controllers,
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    *CONTROLLERS,
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "90",
                    "--param-file",
                    controllers,
                ],
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {}


class TestMockControllers(unittest.TestCase):
    def test_controllers_become_active(self):
        deadline = time.monotonic() + 75.0
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
            if _all_active(last):
                print(last, flush=True)
                return
            time.sleep(2.0)
        self.fail("mock controllers did not become active:\n" + last)


def _all_active(text):
    for name in CONTROLLERS:
        matched = False
        for line in text.splitlines():
            if name in line and "active" in line:
                matched = True
        if not matched:
            return False
    return True
