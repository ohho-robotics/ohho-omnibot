#!/usr/bin/env python3
"""Serve the ACT VR sim-pick checkpoint through policy_node.

Leaves /control_mode at teleop. The headset policy panel publishes vla and
arm cmd_mode policy after the operator confirms. This file does not switch
modes and does not change SafetyGate, teleop_safety, or e-stop.

Requires a Serve process (or the stdlib mock) at serve_url before the node
can load weights. The default checkpoint is the local smoke/full output:

    checkpoints/act_vr_sim_pick/best

Usage
-----
ros2 launch omnibot_lerobot act_vr_sim_pick.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_lerobot = get_package_share_directory("omnibot_lerobot")
    policy_launch = os.path.join(pkg_lerobot, "launch", "policy_inference.launch.py")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(policy_launch),
                launch_arguments={
                    "model_type": "serve",
                    "checkpoint": "checkpoints/act_vr_sim_pick/best",
                    "device": "cpu",
                    "serve_url": "http://127.0.0.1:8000",
                    "include_arm_mux": "true",
                }.items(),
            )
        ]
    )
