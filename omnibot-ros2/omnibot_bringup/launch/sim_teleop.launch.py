# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Gazebo teleop profile: safety gate in front of the muxes.

    ros2 launch omnibot_bringup sim_teleop.launch.py world:=flat

Includes sim.launch.py and forces bridge_config:=sim_bridge_vr.yaml so
Gazebo follows /cmd_vel/out. Clients publish /cmd_vel/teleop_raw and
/arm/joint_commands/teleop_raw. teleop_safety publishes /cmd_vel/teleop
and /arm/joint_commands/teleop. cmd_vel_mux and arm_cmd_mux start in teleop.

vr:=true is still the Quest camera profile from sim.launch.py. It does
not, by itself, start this gate. The default here leaves vr false so the
flat path keeps the minimal front camera. sim_vr.launch.py does not
include this file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("omnibot_bringup")
    hybrid = get_package_share_directory("omnibot_hybrid")
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "drive": LaunchConfiguration("drive"),
            "gui": LaunchConfiguration("gui"),
            "arm": LaunchConfiguration("arm"),
            "rosbridge": LaunchConfiguration("rosbridge"),
            "descriptor": LaunchConfiguration("descriptor"),
            "rosbridge_address": LaunchConfiguration("rosbridge_address"),
            "vr": LaunchConfiguration("vr"),
            "video": LaunchConfiguration("video"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "bridge_config": "sim_bridge_vr.yaml",
        }.items(),
    )
    teleop_safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hybrid, "launch", "teleop_safety.launch.py")
        )
    )
    cmd_vel_mux = Node(
        package="omnibot_hybrid",
        executable="cmd_vel_mux",
        name="cmd_vel_mux",
        output="screen",
        parameters=[{"default_mode": "teleop", "use_sim_time": False}],
    )
    arm_cmd_mux = Node(
        package="omnibot_rl",
        executable="arm_cmd_mux",
        name="arm_cmd_mux",
        output="screen",
        parameters=[{"default_mode": "teleop", "use_sim_time": False}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="flat",
                description="flat, apartment, or a path to an SDF world",
            ),
            DeclareLaunchArgument(
                "drive",
                default_value="planar",
                description="planar (velocity control) or mecanum (MecanumDrive)",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Open the Gazebo GUI. Default is headless.",
            ),
            DeclareLaunchArgument(
                "arm",
                default_value="true",
                description="Forwarded to sim.launch.py. Default starts arm_sim_driver.",
            ),
            DeclareLaunchArgument(
                "rosbridge",
                default_value="true",
                description="Forwarded to sim.launch.py. The watchdog test sets this false.",
            ),
            DeclareLaunchArgument(
                "descriptor",
                default_value="true",
                description="Publish the latched RobotDescriptor JSON on /ohho/descriptor",
            ),
            DeclareLaunchArgument(
                "rosbridge_address",
                default_value="127.0.0.1",
                description=(
                    "Interface rosbridge_websocket binds. "
                    "127.0.0.1 is loopback only. "
                    "0.0.0.0 exposes it on the network without authentication."
                ),
            ),
            DeclareLaunchArgument(
                "vr",
                default_value="false",
                description=(
                    "Forwarded to sim.launch.py. True enables the Quest cameras. "
                    "This profile starts the safety gate either way."
                ),
            ),
            DeclareLaunchArgument(
                "video",
                default_value="true",
                description="Forwarded to sim.launch.py. Used only when vr:=true.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0"),
            DeclareLaunchArgument("spawn_y", default_value="0"),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="0",
                description="Spawn heading about +Z, radians",
            ),
            sim,
            teleop_safety,
            cmd_vel_mux,
            arm_cmd_mux,
        ]
    )
