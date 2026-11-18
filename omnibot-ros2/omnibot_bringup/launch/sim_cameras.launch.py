# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Bridge the VR sim cameras and serve them with web_video_server.

Included by sim.launch.py when vr:=true. That launch also passes
vr_cameras:=true into the robot xacro, which is what creates the
wrist, BEV, and chase sensors. This file does not start Gazebo and
does not spawn a second robot.

    ros2 launch omnibot_bringup sim.launch.py world:=apartment vr:=true

Streams, once web_video_server is listening (video:=true, port 8080):

    http://<host>:8080/stream?topic=/sim/chase_cam/image_raw
    http://<host>:8080/stream?topic=/camera/front/image_raw
    http://<host>:8080/stream?topic=/camera/wrist/image_raw
    http://<host>:8080/stream?topic=/camera/base/bev/image_raw

The bind address is `address` (sim.launch.py passes rosbridge_address,
which defaults to 127.0.0.1). A non-loopback address exposes the
streams without authentication.

The sim VR profile on ohh-198 includes this file again when
cameras:=extra. Include it once. A second include starts another
parameter_bridge and, when video is true, another web_video_server
on port 8080.

GPU: sensors are always_on, so Gazebo renders them even with no
client. web_video_server encodes a topic only while a client is
connected. gui:=false keeps --headless-rendering. The GPU delta was
not measured in this workspace (no Gazebo install). See the note in
sim.launch.py.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_support():
    import importlib.util

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_cameras_support.py")
    spec = importlib.util.spec_from_file_location("omnibot_sim_cameras_support", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_cameras = _load_support()


def _launch_cameras(context, *args, **kwargs):
    del args, kwargs
    bringup = FindPackageShare("omnibot_bringup").perform(context)
    bridge_config = os.path.join(bringup, "config", "sim_bridge_cameras.yaml")
    address = _cameras.bind_address(LaunchConfiguration("address").perform(context))
    video = _cameras.truthy(LaunchConfiguration("video").perform(context))

    actions = [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_camera_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config, "use_sim_time": False}],
        )
    ]
    if not video:
        return actions
    warning = _cameras.public_bind_warning(address)
    if warning:
        actions.append(LogInfo(msg=warning))
    actions.append(
        Node(
            package="web_video_server",
            executable="web_video_server",
            name="web_video_server",
            output="screen",
            parameters=[
                {
                    "port": _cameras.VIDEO_PORT,
                    "address": address,
                    "use_sim_time": True,
                }
            ],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video",
                default_value="true",
                description=(
                    "Start web_video_server on port 8080. "
                    "Chase stream: /stream?topic=/sim/chase_cam/image_raw."
                ),
            ),
            DeclareLaunchArgument(
                "address",
                default_value=_cameras.LOOPBACK_ADDRESS,
                description=(
                    "Interface web_video_server binds. "
                    "127.0.0.1 is loopback only. "
                    "Any other address exposes the streams without authentication."
                ),
            ),
            OpaqueFunction(function=_launch_cameras),
        ]
    )
