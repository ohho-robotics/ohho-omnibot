# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""OmniBot sim with windows: Gazebo Harmonic client + RViz.

    ros2 launch omnibot_bringup sim_desktop.launch.py
    ros2 launch omnibot_bringup sim_desktop.launch.py world:=apartment
    ros2 launch omnibot_bringup sim_desktop.launch.py gui:=false rviz:=false

This is the launch the browser desktop container runs (Dockerfile.sim,
target "desktop", see docker/README.md). It needs an X display.

It includes sim.launch.py (world, drive, arm, rosbridge pass through) with
gui:=true by default, and adds:

  * joint_state_publisher: publishes /joint_states for every non-fixed URDF
    joint (the wheels at zero) and merges the SO-101 positions from
    /arm/joint_states, so robot_state_publisher emits the full TF tree and
    the RViz robot model is complete.
  * rviz2 with config/sim_desktop.rviz (fixed frame odom: robot model, TF,
    /odom, /scan and the front camera), on sim time.
  * GZ_SIM_RESOURCE_PATH += the share/ folder of omnibot_description, so
    Gazebo resolves model://omnibot_description/... meshes (AprilTag plates)
    in its window.
"""

import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("omnibot_bringup")

    # model://omnibot_description/<path> resolves against <prefix>/share.
    model_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.dirname(get_package_share_directory("omnibot_description")),
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, "launch", "sim.launch.py"])
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "drive": LaunchConfiguration("drive"),
            "arm": LaunchConfiguration("arm"),
            "rosbridge": LaunchConfiguration("rosbridge"),
        }.items(),
    )

    joint_states = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("joint_states")),
        parameters=[
            {
                "source_list": ["/arm/joint_states"],
                "rate": 20,
                "use_sim_time": True,
            }
        ],
    )

    # Opened a few seconds after Gazebo so the window stacks on top of it.
    rviz = TimerAction(
        period=float(os.environ.get("OMNIBOT_RVIZ_DELAY", "5.0")),
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                condition=IfCondition(LaunchConfiguration("rviz")),
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="flat",
                description="flat, apartment, or a path to an SDF world",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Open the Gazebo client window",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [bringup, "config", "sim_desktop.rviz"]
                ),
                description="RViz config file",
            ),
            DeclareLaunchArgument(
                "joint_states",
                default_value="true",
                description=(
                    "Run joint_state_publisher (wheels at zero, arm from "
                    "/arm/joint_states) so the full TF tree is published"
                ),
            ),
            DeclareLaunchArgument(
                "drive",
                default_value="planar",
                description="planar (velocity control) or mecanum (MecanumDrive)",
            ),
            DeclareLaunchArgument(
                "arm",
                default_value="true",
                description="Start arm_sim_driver (see sim.launch.py)",
            ),
            DeclareLaunchArgument(
                "rosbridge",
                default_value="true",
                description="Start rosbridge_websocket on 127.0.0.1:9090",
            ),
            model_path,
            sim,
            joint_states,
            rviz,
        ]
    )
