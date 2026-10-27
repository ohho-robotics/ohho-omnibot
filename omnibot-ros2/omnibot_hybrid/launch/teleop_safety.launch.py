#!/usr/bin/env python3
"""Start teleop_safety in front of the cmd_vel and arm muxes.

Clients publish /cmd_vel/teleop_raw and /arm/joint_commands/teleop_raw.
The node publishes /cmd_vel/teleop and /arm/joint_commands/teleop.
Speed ceilings live in teleop_gate.py. This file sets no speed parameter.
use_sim_time is pinned false so the 300 ms watchdogs stay on wall time.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="omnibot_hybrid",
                executable="teleop_safety",
                name="teleop_safety",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )
