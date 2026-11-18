#!/usr/bin/env python3
"""Publish /ohho/telemetry at 20 Hz.

OHH-198 (sim VR profile) and OHH-220 should include this file.
Do not copy the node into sim.launch.py or the teleop_safety launch.
Pass sim:=true on the simulator and leave it false on the robot.
use_sim_time stays false so the 20 Hz tick and age_ms stay on wall time.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(text: str) -> bool:
    return text.strip().lower() in ("1", "true", "yes", "on")


def _arm_params_file(context) -> str:
    try:
        share = FindPackageShare("omnibot_arm").perform(context)
    except Exception:
        return ""
    path = os.path.join(share, "config", "arm_params.yaml")
    if os.path.isfile(path):
        return path
    return ""


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    sim = _as_bool(LaunchConfiguration("sim").perform(context))
    robot_id = LaunchConfiguration("robot_id").perform(context).strip()
    try:
        publish_hz = float(LaunchConfiguration("publish_hz").perform(context))
    except ValueError:
        publish_hz = 20.0
    return [
        Node(
            package="omnibot_hybrid",
            executable="telemetry_aggregator",
            name="telemetry_aggregator",
            output="screen",
            parameters=[
                {
                    "sim": sim,
                    "robot_id": robot_id,
                    "publish_hz": publish_hz,
                    "use_sim_time": False,
                    "arm_params_file": _arm_params_file(context),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim",
                default_value="false",
                description=(
                    "true in simulation: kind=sim and battery.simulated=true. "
                    "false on the real robot."
                ),
            ),
            DeclareLaunchArgument(
                "robot_id",
                default_value="",
                description="Stable robot id. Empty uses omnibot-sim or omnibot.",
            ),
            DeclareLaunchArgument(
                "publish_hz",
                default_value="20.0",
                description="Telemetry publish rate in hertz.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
