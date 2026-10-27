# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Spawn randomiser and launch-argument checks. No ROS and no Gazebo."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import pytest

BRINGUP = Path(__file__).resolve().parents[1]
SCRIPTS = BRINGUP / "scripts"
sys.path.insert(0, str(SCRIPTS))

import pick_place_spawn as spawn  # noqa: E402


def _ros_topics(text: str) -> list[str]:
    topics = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("- ros_topic_name:"):
            topics.append(stripped.split(":", 1)[1].strip())
    return topics


def test_same_seed_is_identical_and_other_seeds_move_the_cube():
    first = spawn.layout_for_seed(0)
    assert first == spawn.layout_for_seed("0")
    assert first == spawn.layout_for_seed(0)
    other = spawn.layout_for_seed(1)
    assert (first.cube.x, first.cube.y, first.cube.yaw) != (
        other.cube.x,
        other.cube.y,
        other.cube.yaw,
    )


def test_seed_must_be_an_integer():
    with pytest.raises(ValueError):
        spawn.parse_seed("nope")
    with pytest.raises(ValueError):
        spawn.parse_seed(True)
    with pytest.raises(ValueError):
        spawn.parse_seed("")


def test_cube_and_bin_stay_in_range_and_apart():
    for seed in range(50):
        layout = spawn.layout_for_seed(seed)
        assert layout.table.x == pytest.approx(spawn.TABLE_CENTER_X)
        assert layout.table.y == pytest.approx(spawn.TABLE_CENTER_Y)
        assert layout.table.yaw == pytest.approx(0.0)
        assert spawn.CUBE_X[0] <= layout.cube.x <= spawn.CUBE_X[1]
        assert spawn.CUBE_Y[0] <= layout.cube.y <= spawn.CUBE_Y[1]
        assert spawn.BIN_X[0] <= layout.bin.x <= spawn.BIN_X[1]
        assert spawn.BIN_Y[0] <= layout.bin.y <= spawn.BIN_Y[1]
        assert spawn.center_gap(layout) >= spawn.MIN_CENTER_GAP_M
        assert layout.cube.z == pytest.approx(
            spawn.TABLE_TOP_Z + spawn.CUBE_SIZE / 2.0 + spawn.CUBE_CLEARANCE_Z
        )
        assert layout.bin.z == pytest.approx(spawn.TABLE_TOP_Z)


def test_yaw_keeps_the_objects_on_the_table():
    half_x = spawn.TABLE_SIZE_X / 2.0
    half_y = spawn.TABLE_SIZE_Y / 2.0
    for seed in range(50):
        layout = spawn.layout_for_seed(seed)
        for pose, size_x, size_y in (
            (layout.cube, spawn.CUBE_SIZE, spawn.CUBE_SIZE),
            (layout.bin, spawn.BIN_SIZE_X, spawn.BIN_SIZE_Y),
        ):
            hx = size_x / 2.0
            hy = size_y / 2.0
            extent_x = hx * abs(math.cos(pose.yaw)) + hy * abs(math.sin(pose.yaw))
            extent_y = hx * abs(math.sin(pose.yaw)) + hy * abs(math.cos(pose.yaw))
            assert abs(pose.x - spawn.TABLE_CENTER_X) + extent_x <= half_x + 1e-9
            assert abs(pose.y - spawn.TABLE_CENTER_Y) + extent_y <= half_y + 1e-9


def test_disabled_spawn_plan_is_empty_and_enabled_plan_is_deterministic():
    assert spawn.spawn_plan(False, 0, SCRIPTS, "apartment") == []
    plan = spawn.spawn_plan(True, 4, BRINGUP / "models", "apartment")
    again = spawn.spawn_plan(True, 4, BRINGUP / "models", "apartment")
    assert [item["name"] for item in plan] == [
        spawn.TABLE_NAME,
        spawn.CUBE_NAME,
        spawn.BIN_NAME,
    ]
    assert plan == again
    assert plan[0]["arguments"][:4] == [
        "-world",
        "apartment",
        "-file",
        str(BRINGUP / "models" / "pick_place_table.sdf"),
    ]
    assert "-name" in plan[1]["arguments"]


def test_default_world_file_has_no_task_props_and_launch_defaults_off():
    apartment = (BRINGUP / "worlds" / "apartment.sdf").read_text(encoding="utf-8")
    assert "pick_place_table" not in apartment
    assert "pick_place_cube" not in apartment
    assert "pick_place_bin" not in apartment
    launch = (BRINGUP / "launch" / "sim.launch.py").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument(\n                "pick_place",\n                default_value="false",' in launch
    assert 'DeclareLaunchArgument(\n                "cmd_vel_from_mux",\n                default_value="false",' in launch
    assert 'if _enabled(LaunchConfiguration("pick_place").perform(context)):' in launch
    assert "table_node, *objects = spawners" in launch
    assert "OnProcessExit(target_action=table_node, on_exit=objects)" in launch
    assert 'bridge_name = "sim_bridge_mux.yaml" if from_mux else "sim_bridge.yaml"' in launch


def test_mux_bridge_only_redirects_the_base_command():
    direct = _ros_topics((BRINGUP / "config" / "sim_bridge.yaml").read_text(encoding="utf-8"))
    mux = _ros_topics((BRINGUP / "config" / "sim_bridge_mux.yaml").read_text(encoding="utf-8"))
    assert direct[1] == '"/cmd_vel"'
    assert mux[1] == '"/cmd_vel/out"'
    assert direct[:1] + direct[2:] == mux[:1] + mux[2:]
    mux_text = (BRINGUP / "config" / "sim_bridge_mux.yaml").read_text(encoding="utf-8")
    assert 'gz_topic_name: "/cmd_vel"' in mux_text


def test_model_files_match_the_spawn_sizes():
    table = (BRINGUP / "models" / "pick_place_table.sdf").read_text(encoding="utf-8")
    cube = (BRINGUP / "models" / "pick_place_cube.sdf").read_text(encoding="utf-8")
    assert "<size>0.90 0.55 0.72</size>" in table
    assert "<size>0.04 0.04 0.04</size>" in cube
    assert "<static>true</static>" in table
    assert "<static>false</static>" in cube


def test_named_worlds_and_custom_sdf():
    assert spawn.world_name_for("apartment", "ignored") == "apartment"
    assert spawn.world_name_for("flat", "ignored") == "flat"
    path = BRINGUP / "worlds" / "apartment.sdf"
    assert spawn.world_name_for(str(path), path) == "apartment"
