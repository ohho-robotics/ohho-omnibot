# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""MoveIt demo for the SO-101 arm and gripper.

Default: mock GenericSystem, move_group, and RViz. The mecanum base is a
fixed virtual joint. ``gz:=true`` uses Gazebo and gz_ros2_control instead
of ros2_control_node. ``rviz:=false`` is the headless CI path.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from moveit_config import build_moveit_config  # noqa: E402

CONTROLLERS = [
    "joint_state_broadcaster",
    "mecanum_drive_controller",
    "arm_joint_trajectory_controller",
    "gripper_controller",
]


def _enabled(value: str) -> bool:
    return value.lower() in ("1", "true", "yes")


def _mappings(use_mock: str, use_sim: str, controllers_file: str) -> dict[str, str]:
    return {
        "use_mock": use_mock,
        "use_sim": use_sim,
        "drive": "mecanum",
        "lidar": "false",
        "minimal_sensors": "false",
        "vr_cameras": "false",
        "controllers_file": controllers_file,
    }


def _static_tf(use_sim_time: bool) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fixed_base_tf",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_footprint"],
        parameters=[{"use_sim_time": use_sim_time}],
    )


def _robot_state_publisher(moveit_config, use_sim_time: bool) -> Node:
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )


def _move_group(moveit_config, use_sim_time: bool) -> Node:
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )


def _rviz(moveit_config, rviz_config: str, use_sim_time: bool) -> Node:
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )


def _spawner(controllers_file: str) -> Node:
    return Node(
        package="controller_manager",
        executable="spawner",
        name="controller_spawner",
        output="screen",
        arguments=[
            *CONTROLLERS,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--param-file",
            controllers_file,
        ],
        parameters=[{"use_sim_time": False}],
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    use_gz = _enabled(LaunchConfiguration("gz").perform(context))
    use_rviz = _enabled(LaunchConfiguration("rviz").perform(context))
    gui = _enabled(LaunchConfiguration("gui").perform(context))

    bringup = FindPackageShare("omnibot_bringup").perform(context)
    moveit_share = FindPackageShare("omnibot_moveit_config").perform(context)
    controllers_file = os.path.join(bringup, "config", "controllers.yaml")
    rviz_config = os.path.join(moveit_share, "config", "moveit.rviz")
    use_sim_time = use_gz
    moveit_config = build_moveit_config(
        _mappings(
            "false" if use_gz else "true",
            "true" if use_gz else "false",
            controllers_file,
        )
    )

    actions = [
        _static_tf(use_sim_time),
        _robot_state_publisher(moveit_config, use_sim_time),
        _move_group(moveit_config, use_sim_time),
    ]
    if use_rviz:
        actions.append(_rviz(moveit_config, rviz_config, use_sim_time))

    if use_gz:
        ros_gz_sim = FindPackageShare("ros_gz_sim").perform(context)
        world = os.path.join(bringup, "worlds", "flat.sdf")
        if gui:
            gz_args = f"-r {world}"
        else:
            gz_args = f"-r -s --headless-rendering {world}"
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
                ),
                launch_arguments={"gz_args": gz_args}.items(),
            )
        )
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="clock_bridge",
                output="screen",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                parameters=[{"use_sim_time": False}],
            )
        )
        actions.append(
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        name="spawn_omnibot",
                        output="screen",
                        arguments=[
                            "-name",
                            "omnibot",
                            "-topic",
                            "robot_description",
                            "-x",
                            "0",
                            "-y",
                            "0",
                            "-z",
                            "0.05",
                            "-Y",
                            "0",
                        ],
                    )
                ],
            )
        )
        # Wall time, so a late /clock cannot freeze the controller wait.
        actions.append(TimerAction(period=8.0, actions=[_spawner(controllers_file)]))
        return actions

    actions.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            # Jazzy controller_manager ignores a robot_description parameter and
            # subscribes to /robot_description. robot_state_publisher publishes it.
            parameters=[
                controllers_file,
                {"use_sim_time": False},
            ],
        )
    )
    actions.append(_spawner(controllers_file))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz",
                default_value="false",
                description="true: Gazebo and gz_ros2_control. false: mock GenericSystem",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the MotionPlanning display",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="With gz:=true, open the Gazebo window",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
