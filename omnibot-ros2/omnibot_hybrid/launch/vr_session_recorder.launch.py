#!/usr/bin/env python3
"""Record VR session topics to rotating MCAP files.

    ros2 launch omnibot_hybrid vr_session_recorder.launch.py
    ros2 launch omnibot_bringup sim.launch.py vr:=true record_vr_session:=true

The node starts a bag when /ohho/vr_session/start arrives, or on the first
/ohho/heartbeat when auto_session is true (the default). It stops on
/ohho/vr_session/stop, on heartbeat loss for an auto session, or on shutdown.

Topics are /cmd_vel/teleop*, /arm/joint_commands/teleop*, /ohho/heartbeat,
/ohho/telemetry, /ohho/safety/state, and /control_mode*. Parts rotate at
max_bag_size_bytes. Oldest parts are deleted past max_total_bytes.

OHH-198 sim_vr.launch.py includes sim.launch.py with vr:=true. Forward the
flag and do not include this file a second time:

    DeclareLaunchArgument("record_vr_session", default_value="false")
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(... "sim.launch.py"),
        launch_arguments={
            "vr": "true",
            "record_vr_session": LaunchConfiguration("record_vr_session"),
        }.items(),
    )

A standalone include, only when sim.launch.py is not already starting it:

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("omnibot_hybrid"),
                "launch",
                "vr_session_recorder.launch.py",
            ])
        )
    )
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from omnibot_hybrid.vr_session_record import (
    DEFAULT_HEARTBEAT_GAP_SEC,
    DEFAULT_MAX_BAG_SIZE_BYTES,
    DEFAULT_MAX_TOTAL_BYTES,
    DEFAULT_METRICS_PORT,
    DEFAULT_OUTPUT_DIR,
    DEFAULT_SESSION_IDLE_SEC,
)


def _as_bool(text: str) -> bool:
    return text.strip().lower() in ("1", "true", "yes", "on")


def _as_int(text: str, default: int) -> int:
    try:
        return int(str(text).strip())
    except (TypeError, ValueError):
        return default


def _as_float(text: str, default: float) -> float:
    try:
        return float(str(text).strip())
    except (TypeError, ValueError):
        return default


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    output_dir = LaunchConfiguration("output_dir").perform(context).strip()
    if not output_dir:
        output_dir = DEFAULT_OUTPUT_DIR
    output_dir = os.path.expanduser(output_dir)
    return [
        Node(
            package="omnibot_hybrid",
            executable="vr_session_recorder",
            name="vr_session_recorder",
            output="screen",
            parameters=[
                {
                    "output_dir": output_dir,
                    "max_bag_size_bytes": _as_int(
                        LaunchConfiguration("max_bag_size_bytes").perform(context),
                        DEFAULT_MAX_BAG_SIZE_BYTES,
                    ),
                    "max_total_bytes": _as_int(
                        LaunchConfiguration("max_total_bytes").perform(context),
                        DEFAULT_MAX_TOTAL_BYTES,
                    ),
                    "auto_session": _as_bool(
                        LaunchConfiguration("auto_session").perform(context)
                    ),
                    "session_idle_sec": _as_float(
                        LaunchConfiguration("session_idle_sec").perform(context),
                        DEFAULT_SESSION_IDLE_SEC,
                    ),
                    "heartbeat_gap_sec": _as_float(
                        LaunchConfiguration("heartbeat_gap_sec").perform(context),
                        DEFAULT_HEARTBEAT_GAP_SEC,
                    ),
                    "bag_use_sim_time": _as_bool(
                        LaunchConfiguration("bag_use_sim_time").perform(context)
                    ),
                    "metrics_port": _as_int(
                        LaunchConfiguration("metrics_port").perform(context),
                        DEFAULT_METRICS_PORT,
                    ),
                    "use_sim_time": False,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value=DEFAULT_OUTPUT_DIR,
                description="Directory for session-* MCAP parts.",
            ),
            DeclareLaunchArgument(
                "max_bag_size_bytes",
                default_value=str(DEFAULT_MAX_BAG_SIZE_BYTES),
                description="Rotate to a new part at this many bytes.",
            ),
            DeclareLaunchArgument(
                "max_total_bytes",
                default_value=str(DEFAULT_MAX_TOTAL_BYTES),
                description="Delete oldest parts when the output directory exceeds this.",
            ),
            DeclareLaunchArgument(
                "auto_session",
                default_value="true",
                description="Start on the first heartbeat and stop after session_idle_sec of silence.",
            ),
            DeclareLaunchArgument(
                "session_idle_sec",
                default_value=str(DEFAULT_SESSION_IDLE_SEC),
                description="Auto-session silence that ends the recording. Not the deadman.",
            ),
            DeclareLaunchArgument(
                "heartbeat_gap_sec",
                default_value=str(DEFAULT_HEARTBEAT_GAP_SEC),
                description="Silence that counts one heartbeat-gap event for Grafana.",
            ),
            DeclareLaunchArgument(
                "bag_use_sim_time",
                default_value="false",
                description="Pass --use-sim-time to ros2 bag record. The node timer stays on wall time.",
            ),
            DeclareLaunchArgument(
                "metrics_port",
                default_value=str(DEFAULT_METRICS_PORT),
                description="Loopback Prometheus port. 0 disables it.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
