# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Real-robot VR bringup for the Raspberry Pi.

One command, wheels on blocks:

    ros2 launch omnibot_bringup robot_vr.launch.py

Starts the base driver, arm driver, front/rear/left/right/wrist cameras,
BEV stitcher, EKF, teleop_safety, cmd_vel_mux, and arm_cmd_mux. Startup
mode is teleop. Clients publish /cmd_vel/teleop_raw and
/arm/joint_commands/teleop_raw. The driver reads /cmd_vel/out.

Also starts telemetry_aggregator (sim:=false) and, when OHH-214 is
installed, the latched descriptor publisher. Video for front, wrist, and
BEV is web_video_server. video_backend:=webrtc includes
webrtc_streamer.launch.py when that file is installed.

rosbridge, foxglove, and web_video_server bind 127.0.0.1. gateway:=true
(the default) includes ohho_gateway.launch.py when that file is installed
and does not start a second rosbridge. A missing gateway file keeps the
upstreams on loopback. This profile does not publish them on every
interface. SLAM and Nav2 stay off.

The topic list is config/vr_topic_contract.yaml, shared with
sim_vr.launch.py. Depth is off unless depth_camera:=true.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_support():
    import importlib.util

    path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "robot_vr_support.py",
    )
    spec = importlib.util.spec_from_file_location("omnibot_robot_vr_support", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_support = _load_support()


def _try_share(context, package):
    try:
        return FindPackageShare(package).perform(context)
    except Exception:
        return ""


def _robot_state(xacro):
    robot_desc = ParameterValue(Command(["xacro ", xacro]), value_type=str)
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
    )


def _node(spec):
    kwargs = {
        "package": spec["package"],
        "executable": spec["executable"],
        "name": spec["name"],
        "output": "screen",
        "parameters": spec["parameters"],
    }
    if spec.get("namespace"):
        kwargs["namespace"] = spec["namespace"]
    if spec.get("remappings"):
        kwargs["remappings"] = [tuple(item) for item in spec["remappings"]]
    return Node(**kwargs)


def _include(spec):
    if spec.get("loader") == "any":
        source = AnyLaunchDescriptionSource(spec["path"])
    else:
        source = PythonLaunchDescriptionSource(spec["path"])
    if spec.get("arguments"):
        return IncludeLaunchDescription(
            source,
            launch_arguments=[tuple(item) for item in spec["arguments"]],
        )
    return IncludeLaunchDescription(source)


def actions_from_plan(result):
    """Turn a support plan into launch actions. No nodes are started here."""
    actions = []
    for spec in result["actions"]:
        kind = spec["kind"]
        if kind == "log":
            actions.append(LogInfo(msg=spec["msg"]))
        elif kind == "robot_state":
            xacro = spec.get("xacro") or ""
            if xacro and os.path.isfile(xacro):
                actions.append(_robot_state(xacro))
            else:
                actions.append(
                    LogInfo(msg="omnibot_description xacro was not found.")
                )
        elif kind == "include":
            actions.append(_include(spec))
        elif kind == "node":
            actions.append(_node(spec))
        else:
            raise RuntimeError(f"unknown VR plan action {kind}")
    return actions


def _launch_robot(context, *args, **kwargs):
    del args, kwargs
    devices = {
        name: LaunchConfiguration(f"cam_{name}").perform(context)
        for name in ("front", "rear", "left", "right", "wrist")
    }
    shares = {
        "omnibot_bringup": FindPackageShare("omnibot_bringup").perform(context),
        "omnibot_hybrid": FindPackageShare("omnibot_hybrid").perform(context),
        "omnibot_description": FindPackageShare("omnibot_description").perform(
            context
        ),
        "omnibot_arm": _try_share(context, "omnibot_arm"),
        "ohho_webrtc": _try_share(context, "ohho_webrtc"),
        "astra_camera": _try_share(context, "astra_camera"),
    }
    result = _support.plan(
        shares,
        gateway=LaunchConfiguration("gateway").perform(context),
        rosbridge=LaunchConfiguration("rosbridge").perform(context),
        video=LaunchConfiguration("video").perform(context),
        video_backend=LaunchConfiguration("video_backend").perform(context),
        foxglove=LaunchConfiguration("foxglove").perform(context),
        arm=LaunchConfiguration("arm").perform(context),
        telemetry=LaunchConfiguration("telemetry").perform(context),
        descriptor=LaunchConfiguration("descriptor").perform(context),
        ekf=LaunchConfiguration("ekf").perform(context),
        bev=LaunchConfiguration("bev").perform(context),
        depth_camera=LaunchConfiguration("depth_camera").perform(context),
        robot_id=LaunchConfiguration("robot_id").perform(context),
        requested_bind=LaunchConfiguration("rosbridge_address").perform(context),
        camera_devices=devices,
    )
    return actions_from_plan(result)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gateway",
                default_value=_support.DEFAULT_GATEWAY,
                description=(
                    "Include ohho_gateway.launch.py when that file is installed. "
                    "Default true. Upstreams stay on loopback either way."
                ),
            ),
            DeclareLaunchArgument(
                "rosbridge",
                default_value=_support.DEFAULT_ROSBRIDGE,
                description=(
                    "Start rosbridge_websocket on port 9090, bound to loopback, "
                    "unless the gateway snippet owns that port."
                ),
            ),
            DeclareLaunchArgument(
                "rosbridge_address",
                default_value=_support.DEFAULT_BIND,
                description=(
                    "Requested bind for rosbridge, foxglove, and web_video_server. "
                    "Anything other than loopback is clamped to loopback."
                ),
            ),
            DeclareLaunchArgument(
                "video",
                default_value=_support.DEFAULT_VIDEO,
                description=(
                    "Serve the front, wrist, and BEV streams from the shared "
                    "VR topic contract."
                ),
            ),
            DeclareLaunchArgument(
                "video_backend",
                default_value=_support.DEFAULT_VIDEO_BACKEND,
                description=(
                    "web starts web_video_server. webrtc includes "
                    "webrtc_streamer.launch.py when it is installed, and "
                    "otherwise falls back to web_video_server."
                ),
            ),
            DeclareLaunchArgument(
                "foxglove",
                default_value=_support.DEFAULT_FOXGLOVE,
                description="Start foxglove_bridge on loopback, or under the gateway.",
            ),
            DeclareLaunchArgument(
                "arm",
                default_value=_support.DEFAULT_ARM,
                description="Start the SO-101 arm driver. The arm mux always starts.",
            ),
            DeclareLaunchArgument(
                "telemetry",
                default_value=_support.DEFAULT_TELEMETRY,
                description="Include omnibot_hybrid telemetry.launch.py with sim:=false.",
            ),
            DeclareLaunchArgument(
                "descriptor",
                default_value=_support.DEFAULT_DESCRIPTOR,
                description=(
                    "Start the latched /ohho/descriptor publisher when "
                    "descriptor_node.py and robot_descriptor.yaml are installed."
                ),
            ),
            DeclareLaunchArgument(
                "ekf",
                default_value=_support.DEFAULT_EKF,
                description="Start robot_localization. The driver publishes TF only when this is false.",
            ),
            DeclareLaunchArgument(
                "bev",
                default_value=_support.DEFAULT_BEV,
                description="Start the four base cameras and the BEV stitcher.",
            ),
            DeclareLaunchArgument(
                "depth_camera",
                default_value=_support.DEFAULT_DEPTH,
                description="Start the Astra Pro depth camera. Default false.",
            ),
            DeclareLaunchArgument(
                "robot_id",
                default_value=_support.DEFAULT_ROBOT_ID,
                description="Telemetry robot id. Empty uses omnibot.",
            ),
            DeclareLaunchArgument("cam_front", default_value="/dev/video2"),
            DeclareLaunchArgument("cam_rear", default_value="/dev/video10"),
            DeclareLaunchArgument("cam_left", default_value="/dev/video6"),
            DeclareLaunchArgument("cam_right", default_value="/dev/video4"),
            DeclareLaunchArgument("cam_wrist", default_value="/dev/video0"),
            OpaqueFunction(function=_launch_robot),
        ]
    )
