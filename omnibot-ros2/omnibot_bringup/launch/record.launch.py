# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Record OmniBot topics to MCAP for Twin replay & Foxglove replay.

Writes /tf, /odom, /scan, /camera, /agent/trace and related topics to MCAP.

Usage:
    ros2 launch omnibot_bringup record.launch.py
    ros2 launch omnibot_bringup record.launch.py bag_path:=my_run
    ros2 launch omnibot_bringup record.launch.py use_sim_time:=true
"""

import sys
from datetime import datetime

# Default topic list capturing base navigation, perception, and agent trace.
DEFAULT_TOPICS = [
    "/tf",
    "/tf_static",
    "/odom",
    "/scan",
    "/camera",
    "/camera/front/image_raw",
    "/camera/front/camera_info",
    "/camera/wrist/image_raw",
    "/camera/wrist/camera_info",
    "/camera/base/bev/image_raw",
    "/camera/depth/image_raw",
    "/camera/depth/camera_info",
    "/agent/trace",
    "/agent/loop_trace",
    "/imu",
    "/imu/data",
    "/rosout",
]


def _build_record_process(context, *args, **kwargs):
    del args, kwargs
    from launch.actions import ExecuteProcess
    from launch.substitutions import LaunchConfiguration

    bag_path = LaunchConfiguration("bag_path").perform(context).strip()
    output = LaunchConfiguration("output").perform(context).strip()
    target_bag = bag_path or output
    if not target_bag:
        stamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        target_bag = f"rosbag2_{stamp}"

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )
    preset_profile = LaunchConfiguration("preset_profile").perform(context).strip()
    include_unpublished = LaunchConfiguration("include_unpublished").perform(
        context
    ).lower() in ("true", "1", "yes")

    topics_arg = LaunchConfiguration("topics").perform(context).strip()
    regex_arg = LaunchConfiguration("regex").perform(context).strip()

    cmd = [
        "ros2",
        "bag",
        "record",
        "-s",
        "mcap",
        "-o",
        target_bag,
    ]

    if preset_profile and preset_profile != "none":
        cmd += ["--storage-preset-profile", preset_profile]

    if use_sim_time:
        cmd.append("--use-sim-time")

    if include_unpublished:
        cmd.append("--include-unpublished-topics")

    if regex_arg:
        cmd += ["-e", regex_arg]
    else:
        if topics_arg:
            topics = [t for t in topics_arg.split() if t]
        else:
            topics = list(DEFAULT_TOPICS)
        cmd += ["--topics"] + topics

    return [ExecuteProcess(cmd=cmd, output="screen", log_cmd=True)]


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="Output path for the recorded MCAP bag (defaults to timestamped rosbag2_*)",
            ),
            DeclareLaunchArgument(
                "output",
                default_value="",
                description="Alias for bag_path",
            ),
            DeclareLaunchArgument(
                "topics",
                default_value="",
                description="Space-separated list of topics to record (empty for default list)",
            ),
            DeclareLaunchArgument(
                "regex",
                default_value="",
                description="Regex pattern of topics to record (overrides topics if set)",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (/clock) time",
            ),
            DeclareLaunchArgument(
                "preset_profile",
                default_value="zstd_fast",
                description="MCAP storage preset profile (e.g. zstd_fast, zstd_small, none)",
            ),
            DeclareLaunchArgument(
                "include_unpublished",
                default_value="true",
                description="Subscribe to unpublished topics so they are recorded as soon as they appear",
            ),
            OpaqueFunction(function=_build_record_process),
        ]
    )
