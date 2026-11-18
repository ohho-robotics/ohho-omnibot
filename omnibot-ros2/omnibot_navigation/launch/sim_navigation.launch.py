#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Nav2 on the saved apartment map in headless Gazebo Harmonic.

    ros2 launch omnibot_navigation sim_navigation.launch.py
    ros2 run omnibot_navigation go_to_waypoint.py kitchen

Starts, all on the Gazebo clock:
  - omnibot_bringup sim.launch.py (world:=apartment, planar holonomic drive)
  - nav2_map_server on maps/apartment.yaml and AMCL, started at the spawn pose
  - the Nav2 stack from autonomous_navigation.launch.py (DWB, NavFn, BT)

It reuses config/nav2_params.yaml. Two keys are rewritten for the sim:
`yaml_filename` (the map) and `observation_sources` (lidar only: the sim has no
depth point cloud). Named goals live in config/waypoints.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

# Spawn pose used by omnibot_bringup sim.launch.py (-x 0 -y 0, yaw 0).
DEFAULT_INITIAL_POSE = {"x": 0.0, "y": 0.0, "yaw": 0.0}


def _launch(context, *args, **kwargs):
    del args, kwargs
    nav = get_package_share_directory("omnibot_navigation")
    bringup = get_package_share_directory("omnibot_bringup")

    map_yaml = LaunchConfiguration("map").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)

    sim_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "use_sim_time": "true",
            "yaml_filename": map_yaml,
            "observation_sources": "scan",
        },
        convert_types=True,
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "sim.launch.py")),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            # Holonomic planar velocity control. drive:=mecanum (wheel physics) was tried
            # once with this stack and the map frame never appeared before Nav2's
            # startup timeout, so it is not offered here.
            "drive": "planar",
            "rosbridge": "false",
            "spawn_x": LaunchConfiguration("initial_x"),
            "spawn_y": LaunchConfiguration("initial_y"),
            "spawn_yaw": LaunchConfiguration("initial_yaw"),
        }.items(),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[sim_params],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            sim_params,
            {
                "set_initial_pose": True,
                "initial_pose": {
                    "x": float(LaunchConfiguration("initial_x").perform(context)),
                    "y": float(LaunchConfiguration("initial_y").perform(context)),
                    "z": 0.0,
                    "yaw": float(LaunchConfiguration("initial_yaw").perform(context)),
                },
            },
        ],
    )

    lifecycle_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": ["map_server", "amcl"],
            }
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav, "launch", "autonomous_navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": sim_params,
            "autostart": "true",
            "bond_timeout": "0.0",
        }.items(),
    )

    nodes = [sim, map_server, amcl, lifecycle_localization, navigation]

    autonavigate = LaunchConfiguration("autonavigate").perform(context).lower() in ("1", "true", "yes")
    foxglove = LaunchConfiguration("foxglove").perform(context).lower() in ("1", "true", "yes")
    route = LaunchConfiguration("route").perform(context)
    waypoint_file = LaunchConfiguration("waypoint_file").perform(context)
    loop_waypoints = LaunchConfiguration("loop_waypoints").perform(context).lower() in ("1", "true", "yes")

    if foxglove:
        foxglove_node = Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[
                {
                    "port": 8765,
                    "address": "0.0.0.0",
                    "use_sim_time": True,
                }
            ],
        )
        nodes.append(foxglove_node)

    if autonavigate:
        waypoint_navigator = Node(
            package="omnibot_navigation",
            executable="waypoint_navigator.py",
            name="waypoint_navigator",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "waypoint_file": waypoint_file,
                    "loop_waypoints": loop_waypoints,
                    "route": ParameterValue(LaunchConfiguration("route"), value_type=str),
                }
            ],
        )
        nodes.append(waypoint_navigator)

    return nodes


def generate_launch_description():
    nav = get_package_share_directory("omnibot_navigation")
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="apartment"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(nav, "maps", "apartment.yaml"),
                description="Saved slam_toolbox map (yaml)",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(nav, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument("initial_x", default_value=str(DEFAULT_INITIAL_POSE["x"])),
            DeclareLaunchArgument("initial_y", default_value=str(DEFAULT_INITIAL_POSE["y"])),
            DeclareLaunchArgument("initial_yaw", default_value=str(DEFAULT_INITIAL_POSE["yaw"])),
            DeclareLaunchArgument(
                "autonavigate",
                default_value="false",
                description="Automatically navigate to waypoints after bringup",
            ),
            DeclareLaunchArgument(
                "foxglove",
                default_value="false",
                description="Launch Foxglove bridge on port 8765",
            ),
            DeclareLaunchArgument(
                "route",
                default_value="kitchen",
                description="Comma-separated waypoint route, e.g. 'kitchen' or 'kitchen,desk,door'",
            ),
            DeclareLaunchArgument(
                "waypoint_file",
                default_value="waypoints.yaml",
                description="Waypoint YAML filename",
            ),
            DeclareLaunchArgument(
                "loop_waypoints",
                default_value="false",
                description="Continuously loop through waypoints",
            ),
            OpaqueFunction(function=_launch),
        ]
    )
