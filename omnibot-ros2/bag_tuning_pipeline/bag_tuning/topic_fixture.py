"""Load synthetic recorded-topic JSON fixtures (no bag binary parser)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

from bag_tuning.poses import ImuSample, Pose2D


def load_topic_fixture(path: Path) -> Dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if "topics" not in data:
        raise ValueError(f"fixture missing 'topics': {path}")
    return data


def fixture_odom(data: Dict[str, Any], topic: str = "/odom") -> List[Pose2D]:
    rows = data["topics"].get(topic, [])
    return [
        Pose2D(int(r["t_ns"]), float(r["x"]), float(r["y"]), float(r["yaw"]))
        for r in rows
    ]


def fixture_imu(data: Dict[str, Any], topic: str = "/imu/data") -> List[ImuSample]:
    rows = data["topics"].get(topic, [])
    return [
        ImuSample(int(r["t_ns"]), float(r["yaw"]), float(r.get("wz", 0.0)))
        for r in rows
    ]


def fixture_goal(data: Dict[str, Any], topic: str = "/goal_pose") -> Pose2D | None:
    rows = data["topics"].get(topic, [])
    if not rows:
        return None
    r = rows[-1]
    return Pose2D(int(r["t_ns"]), float(r["x"]), float(r["y"]), float(r["yaw"]))
