# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""move_group for the robot, on the hardware URDF.

Start omnibot_bringup robot.launch.py first. That launch already runs the
controller manager. This file publishes the fixed virtual joint
(world -> base_footprint) and move_group. ``rsp:=true`` also starts
robot_state_publisher when nothing else is publishing TF.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from moveit_config import build_moveit_config  # noqa: E402


def _enabled(value: str) -> bool:
    return value.lower() in ("1", "true", "yes")


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    start_rsp = _enabled(LaunchConfiguration("rsp").perform(context))
    use_sim_time = _enabled(LaunchConfiguration("use_sim_time").perform(context))
    # Hardware plugin string only. xacro does not open a serial port.
    # FindPackageShare("omnibot_description") keeps the URDF package explicit.
    FindPackageShare("omnibot_description").perform(context)
    moveit_config = build_moveit_config(
        {
            "use_mock": "false",
            "use_sim": "false",
            "drive": "mecanum",
            "lidar": "false",
            "minimal_sensors": "false",
            "vr_cameras": "false",
            "controllers_file": "",
        }
    )
    actions = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="fixed_base_tf",
            output="log",
            arguments=["--frame-id", "world", "--child-frame-id", "base_footprint"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": use_sim_time},
            ],
        ),
    ]
    if start_rsp:
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    {"use_sim_time": use_sim_time},
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rsp",
                default_value="false",
                description="Also start robot_state_publisher",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
