# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""VR Gazebo profile for the GPU sim container, plus headset include points.

    ros2 launch omnibot_bringup sim_vr.launch.py
    ros2 launch omnibot_bringup sim_vr.launch.py nav:=true

The topic list is config/vr_topic_contract.yaml, shared with
robot_vr.launch.py. robot_vr.launch.py starts teleop_safety on the robot.
This file does not.

`docker compose -f docker-compose.gpu.yml up sim-vr` runs this file.
It includes sim.launch.py once, with:

    world:=apartment
    vr:=true
    gui:=false
    arm:=true
    video:=true
    rosbridge:=true
    rosbridge_address:=0.0.0.0

gui:=false keeps `-s --headless-rendering` in sim.launch.py. On the GPU
compose file that path is Ogre2 EGL (the NVIDIA vendor library), not a
window and not llvmpipe. vr:=true is the wrist, BEV, and chase cameras
at 960x540 and 30 Hz, plus web_video_server on port 8080.
arm:=true starts arm_sim_driver inside sim.launch.py.

rosbridge and web_video_server bind 0.0.0.0 inside the container so
Docker can forward 9090 and 8080. Those sockets are not authenticated.
The compose file publishes them on 127.0.0.1 only. sim.launch.py itself
still defaults rosbridge_address to 127.0.0.1.

This file does not start teleop_safety, the cmd_vel mux, or the Feetech
driver. The gate and the /cmd_vel/out bridge live in sim_teleop.launch.py.
Extra camera, arm, Nav2, and telemetry include points stay off unless
those arguments are set. gateway:=true does not bind 0.0.0.0.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_support():
    import importlib.util

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_vr_support.py")
    spec = importlib.util.spec_from_file_location("omnibot_sim_vr_support", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_support = _load_support()


def _include_if(enabled, path, label):
    decision = _support.optional_launch(enabled, os.path.isfile(path))
    if decision == "include":
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path))]
    if decision == "missing":
        return [
            LogInfo(
                msg=(
                    f"{label} include point is not installed yet: {path}. "
                    "The sim VR profile is still starting."
                )
            )
        ]
    return []


def _navigation():
    """Nav2 on the apartment map. Does not start Gazebo."""
    from ament_index_python.packages import PackageNotFoundError
    from ament_index_python.packages import get_package_share_directory
    from nav2_common.launch import RewrittenYaml

    try:
        nav = get_package_share_directory("omnibot_navigation")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "nav:=true requires omnibot_navigation on the workspace path"
        ) from exc

    map_yaml = os.path.join(nav, "maps", "apartment.yaml")
    params_file = os.path.join(nav, "config", "nav2_params.yaml")
    sim_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "use_sim_time": "true",
            "yaml_filename": map_yaml,
            "observation_sources": "scan",
        },
        convert_types=True,
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
                "initial_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
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
    return [map_server, amcl, lifecycle_localization, navigation]


def _launch_vr(context, *args, **kwargs):
    del args, kwargs
    bringup = FindPackageShare("omnibot_bringup").perform(context)

    gateway = LaunchConfiguration("gateway").perform(context)
    video = LaunchConfiguration("video").perform(context)
    requested_address = LaunchConfiguration("rosbridge_address").perform(context)
    gateway_path = os.path.join(bringup, "launch", _support.GATEWAY_LAUNCH_NAME)
    gateway_exists = os.path.isfile(gateway_path)
    plan = _support.connectivity(gateway, gateway_exists, "true", video)
    address = _support.upstream_address(gateway, gateway_exists, requested_address)

    actions = [
        LogInfo(
            msg=(
                "VR topic contract video streams: "
                + ", ".join(_support.video_stream_topics())
            )
        )
    ]
    if plan["use_gateway_launch"]:
        actions.append(
            LogInfo(
                msg=(
                    "gateway:=true includes the gateway launch. "
                    "rosbridge and web_video_server stay on 127.0.0.1. "
                    "This does not bind 0.0.0.0."
                )
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gateway_path),
                launch_arguments={
                    "gateway": "true",
                    "start_rosbridge": "true",
                    "start_web_video": "true" if _support.truthy(video) else "false",
                    "start_foxglove": "true",
                }.items(),
            )
        )
    elif plan["gateway_missing"]:
        actions.append(
            LogInfo(
                msg=(
                    f"gateway:=true but {_support.GATEWAY_LAUNCH_NAME} is not "
                    f"installed. Upstreams stay on {address}. "
                    "This does not bind 0.0.0.0."
                )
            )
        )
    elif address != _support.DEFAULT_BIND:
        actions.append(
            LogInfo(
                msg=(
                    f"rosbridge_address {address} is not {_support.DEFAULT_BIND}. "
                    f"web_video_server port {_support.VIDEO_PORT}. "
                    "That exposes rosbridge without authentication."
                )
            )
        )

    forced = {
        "vr": "true",
        "gui": "false",
        "arm": "true",
        "rosbridge": "true",
    }
    if not (
        _support.truthy(_support.DEFAULT_ROSBRIDGE) and plan["sim_starts_rosbridge"]
    ):
        forced["rosbridge"] = "false"
    video_value = video if plan["sim_starts_video"] else "false"
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, "launch", "sim.launch.py")
            ),
            launch_arguments={
                "world": LaunchConfiguration("world").perform(context),
                "video": video_value,
                "rosbridge_address": address,
                **forced,
            }.items(),
        )
    )

    if _support.truthy(LaunchConfiguration("arm").perform(context)):
        arm_share = FindPackageShare("omnibot_arm").perform(context)
        actions.extend(
            _include_if(
                True,
                os.path.join(arm_share, "launch", _support.ARM_LAUNCH_NAME),
                "OHH-199 arm",
            )
        )
    cameras = LaunchConfiguration("cameras").perform(context)
    mode = _support.camera_mode(cameras)
    if mode == "extra":
        actions.extend(
            _include_if(
                True,
                os.path.join(bringup, "launch", _support.CAMERA_LAUNCH_NAME),
                "OHH-200 cameras",
            )
        )
    elif mode == "unknown":
        actions.append(
            LogInfo(
                msg=(
                    f"cameras:={cameras} is unknown. "
                    "Keeping the front camera from sim.launch.py."
                )
            )
        )
    actions.extend(
        _include_if(
            LaunchConfiguration("telemetry").perform(context),
            os.path.join(bringup, "launch", _support.TELEMETRY_LAUNCH_NAME),
            "telemetry aggregator",
        )
    )
    if _support.truthy(LaunchConfiguration("nav").perform(context)):
        actions.extend(_navigation())
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="apartment"
                if _support.DEFAULT_WORLD == "apartment"
                else _support.DEFAULT_WORLD,
                description="World passed to sim.launch.py. Default is apartment.",
            ),
            DeclareLaunchArgument(
                "video",
                default_value="true"
                if _support.DEFAULT_VIDEO == "true"
                else _support.DEFAULT_VIDEO,
                description="Start web_video_server on port 8080.",
            ),
            DeclareLaunchArgument(
                "rosbridge_address",
                default_value="0.0.0.0",
                description=(
                    "Bind address for rosbridge and web_video_server inside "
                    "the container. 0.0.0.0 lets Docker publish 9090 and 8080. "
                    "The compose file publishes those ports on 127.0.0.1 only. "
                    "There is no authentication on this socket. "
                    f"The support-module default is {_support.DEFAULT_BIND}. "
                    "gateway:=true forces loopback and does not bind 0.0.0.0."
                ),
            ),
            DeclareLaunchArgument(
                "gateway",
                default_value=_support.DEFAULT_GATEWAY,
                description=(
                    "Include the gateway launch when that file is installed. "
                    "Default false. True does not bind rosbridge to 0.0.0.0."
                ),
            ),
            DeclareLaunchArgument(
                "nav",
                default_value=_support.DEFAULT_NAV,
                description=(
                    "Start Nav2 on the apartment map. Default false. "
                    "Does not start a second Gazebo."
                ),
            ),
            DeclareLaunchArgument(
                "arm",
                default_value=_support.DEFAULT_ARM,
                description=(
                    "OHH-199 include point. When true, include the arm launch "
                    "named by the support module if it is installed. "
                    "sim.launch.py still starts arm_sim_driver."
                ),
            ),
            DeclareLaunchArgument(
                "cameras",
                default_value=_support.DEFAULT_CAMERAS,
                description=(
                    "OHH-200 include point. front keeps the camera from "
                    "sim.launch.py. extra includes the camera launch named "
                    "by the support module when that file is installed."
                ),
            ),
            DeclareLaunchArgument(
                "telemetry",
                default_value=_support.DEFAULT_TELEMETRY,
                description=(
                    "Include the telemetry launch named by the support module "
                    "when that file is installed. Default false."
                ),
            ),
            OpaqueFunction(function=_launch_vr),
        ]
    )
