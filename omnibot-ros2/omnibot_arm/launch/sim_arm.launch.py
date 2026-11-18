# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Arm include point for sim_vr.launch.py arm:=true.

Mirrors /arm/joint_commands/out onto /arm/joint_states. This is not
OHH-199's Gazebo arm_sim_driver and it does not command hardware.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="omnibot_arm",
                executable="arm_joint_state_mirror.py",
                name="arm_joint_state_mirror",
                output="screen",
                parameters=[{"use_sim_time": False}],
            )
        ]
    )
