# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""VR Gazebo profile for the GPU sim container.

    ros2 launch omnibot_bringup sim_vr.launch.py

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

rosbridge and web_video_server bind 0.0.0.0 inside the container so
Docker can forward 9090 and 8080. Those sockets are not authenticated.
The compose file publishes them on 127.0.0.1 only. sim.launch.py itself
still defaults rosbridge_address to 127.0.0.1.

This file does not start teleop_safety, the cmd_vel mux, or the Feetech
driver. It includes sim_cameras.launch.py only through sim.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory("omnibot_bringup")
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "vr": "true",
            "gui": "false",
            "arm": "true",
            "video": LaunchConfiguration("video"),
            "rosbridge": "true",
            "rosbridge_address": LaunchConfiguration("rosbridge_address"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="apartment",
                description="World passed to sim.launch.py. Default is apartment.",
            ),
            DeclareLaunchArgument(
                "video",
                default_value="true",
                description="Start web_video_server on port 8080.",
            ),
            DeclareLaunchArgument(
                "rosbridge_address",
                default_value="0.0.0.0",
                description=(
                    "Bind address for rosbridge and web_video_server inside "
                    "the container. 0.0.0.0 lets Docker publish 9090 and 8080. "
                    "The compose file publishes those ports on 127.0.0.1 only. "
                    "There is no authentication on this socket."
                ),
            ),
            sim,
        ]
    )
