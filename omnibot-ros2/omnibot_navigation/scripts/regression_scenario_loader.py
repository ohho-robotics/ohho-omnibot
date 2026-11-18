#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Load Proof v0 scenario regression suite YAML (start / goal / obstacle).

No ROS dependency — safe for CPU CI.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path

import yaml

from waypoint_loader import (
    PACKAGE_NAME,
    Waypoint,
    angle_diff,
    quaternion_from_yaw,
)

DEFAULT_FILENAME = "nav_regression_scenarios.yaml"
YAW_TOLERANCE_RAD = math.radians(1.0)


@dataclass(frozen=True)
class ObstacleBox:
    name: str
    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float
    size_z: float


@dataclass(frozen=True)
class RegressionScenario:
    id: str
    start_x: float
    start_y: float
    start_yaw: float
    goal: Waypoint
    obstacle: ObstacleBox
    world: str = "apartment"
    frame_id: str = "map"


def default_scenarios_path(filename: str = DEFAULT_FILENAME) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory(PACKAGE_NAME)) / "config" / filename
        if installed.is_file():
            return installed
    except Exception:
        pass
    return Path(__file__).resolve().parents[1] / "config" / filename


def resolve_scenarios_path(path: str) -> Path:
    candidate = Path(path)
    if candidate.is_file():
        return candidate
    return default_scenarios_path(os.path.basename(path))


def _pose_from_mapping(mapping: dict, label: str) -> tuple[float, float, float]:
    try:
        x = float(mapping["x"])
        y = float(mapping["y"])
        yaw = math.radians(float(mapping["yaw_deg"]))
    except KeyError as missing:
        raise ValueError(f"{label} is missing {missing}") from missing
    return x, y, yaw


def _goal_waypoint(frame_id: str, scenario_id: str, mapping: dict) -> Waypoint:
    x, y, yaw = _pose_from_mapping(mapping, f"scenario {scenario_id!r} goal")
    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    waypoint = Waypoint(
        name=scenario_id,
        x=x,
        y=y,
        z=float(mapping.get("z", 0.0)),
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
        frame_id=frame_id,
    )
    if "yaw_deg" in mapping:
        declared = math.radians(float(mapping["yaw_deg"]))
        if abs(angle_diff(declared, waypoint.yaw)) > YAW_TOLERANCE_RAD:
            raise ValueError(
                f"scenario {scenario_id!r}: goal yaw_deg disagrees with implied heading"
            )
    return waypoint


def parse_regression_scenarios(data: dict) -> dict[str, RegressionScenario]:
    if not isinstance(data, dict) or "scenarios" not in data:
        raise ValueError("scenario file must be a mapping with a 'scenarios' list")
    frame_id = str(data.get("frame_id", "map"))
    world = str(data.get("world", "apartment"))
    result: dict[str, RegressionScenario] = {}
    for index, entry in enumerate(data["scenarios"] or []):
        if not isinstance(entry, dict):
            raise ValueError(f"scenario #{index} must be a mapping")
        scenario_id = entry.get("id")
        if not scenario_id or not isinstance(scenario_id, str):
            raise ValueError(f"scenario #{index} has no id")
        if scenario_id in result:
            raise ValueError(f"duplicate scenario id {scenario_id!r}")
        start_x, start_y, start_yaw = _pose_from_mapping(
            entry.get("start") or {}, f"scenario {scenario_id!r} start"
        )
        goal = _goal_waypoint(frame_id, scenario_id, entry.get("goal") or {})
        obst = entry.get("obstacle") or {}
        name = obst.get("name")
        if not name:
            raise ValueError(f"scenario {scenario_id!r} obstacle has no name")
        pose = obst.get("pose") or {}
        size = obst.get("size") or {}
        try:
            obstacle = ObstacleBox(
                name=str(name),
                x=float(pose["x"]),
                y=float(pose["y"]),
                yaw=math.radians(float(pose.get("yaw_deg", 0.0))),
                size_x=float(size["x"]),
                size_y=float(size["y"]),
                size_z=float(size["z"]),
            )
        except KeyError as missing:
            raise ValueError(f"scenario {scenario_id!r} obstacle missing {missing}") from missing
        for dim in (obstacle.size_x, obstacle.size_y, obstacle.size_z):
            if dim <= 0.0 or dim > 2.0:
                raise ValueError(f"scenario {scenario_id!r}: obstacle size out of range")
        result[scenario_id] = RegressionScenario(
            id=scenario_id,
            start_x=start_x,
            start_y=start_y,
            start_yaw=start_yaw,
            goal=goal,
            obstacle=obstacle,
            world=world,
            frame_id=frame_id,
        )
    return result


def load_regression_scenarios(path: str | os.PathLike | None = None) -> dict[str, RegressionScenario]:
    resolved = Path(path) if path else default_scenarios_path()
    with open(resolved, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return parse_regression_scenarios(data)


def scenario_ids(path: str | os.PathLike | None = None) -> list[str]:
    return list(load_regression_scenarios(path).keys())
