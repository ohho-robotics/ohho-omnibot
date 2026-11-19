# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Build MoveItConfigs for the mecanum_bot SRDF.

The installed MoveItConfigsBuilder reads ``.setup_assistant`` from the
package share directory. String mappings replace the xacro args in that
file (an empty dict would fall back to them).
"""

from __future__ import annotations


def build_moveit_config(mappings: dict[str, str]):
    if not mappings:
        raise ValueError(
            "xacro mappings must be set; an empty dict falls back to .setup_assistant"
        )
    from moveit_configs_utils import MoveItConfigsBuilder

    return (
        MoveItConfigsBuilder("mecanum_bot", package_name="omnibot_moveit_config")
        .robot_description(mappings=mappings)
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            load_all=False,
        )
        .trajectory_execution(moveit_manage_controllers=False)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .pilz_cartesian_limits()
        .to_moveit_configs()
    )
