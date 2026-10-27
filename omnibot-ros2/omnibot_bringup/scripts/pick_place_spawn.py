# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Seeded table, cube, and bin poses for the apartment pick-and-place scene.

The apartment SDF is unchanged. sim.launch.py spawns these models only when
``pick_place:=true``. The table pose is fixed. The cube and the bin are drawn
from ``random.Random(seed)`` in this order: cube x, cube y, cube yaw, bin x,
bin y, bin yaw. The same seed always returns the same poses.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import random
import re


TABLE_NAME = "pick_place_table"
CUBE_NAME = "pick_place_cube"
BIN_NAME = "pick_place_bin"

TABLE_CENTER_X = 0.20
TABLE_CENTER_Y = 1.15
TABLE_SIZE_X = 0.90
TABLE_SIZE_Y = 0.55
TABLE_HEIGHT = 0.72
# Bottom sits 1 cm above the ground plane.
TABLE_CENTER_Z = 0.37
TABLE_TOP_Z = TABLE_CENTER_Z + TABLE_HEIGHT / 2.0

CUBE_SIZE = 0.04
CUBE_CLEARANCE_Z = 0.002
BIN_SIZE_X = 0.18
BIN_SIZE_Y = 0.16
BIN_SIZE_Z = 0.08

# World-frame ranges. They sit on the tabletop and leave a gap between the
# cube (west) and the bin (east) that yaw cannot close.
CUBE_X = (-0.18, 0.12)
CUBE_Y = (0.95, 1.35)
CUBE_YAW = (-0.5, 0.5)
BIN_X = (0.34, 0.52)
BIN_Y = (1.00, 1.30)
BIN_YAW = (-0.3, 0.3)

MIN_CENTER_GAP_M = 0.16


@dataclass(frozen=True)
class SpawnPose:
    """One model spawned by ros_gz_sim create."""

    name: str
    file_name: str
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class PickPlaceLayout:
    """Table plus the seeded cube and bin."""

    seed: int
    table: SpawnPose
    cube: SpawnPose
    bin: SpawnPose

    @property
    def models(self) -> tuple[SpawnPose, SpawnPose, SpawnPose]:
        return (self.table, self.cube, self.bin)


def parse_seed(value) -> int:
    """Parse a launch-argument seed. Booleans and blank text are rejected."""
    if isinstance(value, bool) or value is None:
        raise ValueError(f"pick_place_seed must be an integer, got {value!r}")
    text = str(value).strip()
    if not text:
        raise ValueError("pick_place_seed is empty")
    try:
        return int(text)
    except ValueError as exc:
        raise ValueError(f"pick_place_seed must be an integer, got {value!r}") from exc


def flag_enabled(value) -> bool:
    """True for the launch spellings 1, true, and yes."""
    return str(value).strip().lower() in ("1", "true", "yes")


def world_name_for(world_key, world_path) -> str:
    """Gazebo world name for ``ros_gz_sim create -world``."""
    key = str(world_key).strip()
    if key in ("flat", "apartment"):
        return key
    text = Path(world_path).read_text(encoding="utf-8")
    match = re.search(r'<world\s+name="([^"]+)"', text)
    if match is None:
        raise ValueError(f"SDF {world_path} has no world name")
    return match.group(1)


def layout_for_seed(seed) -> PickPlaceLayout:
    """Return the table and the cube/bin poses for ``seed``."""
    parsed = parse_seed(seed)
    rng = random.Random(parsed)
    cube_x = rng.uniform(*CUBE_X)
    cube_y = rng.uniform(*CUBE_Y)
    cube_yaw = rng.uniform(*CUBE_YAW)
    bin_x = rng.uniform(*BIN_X)
    bin_y = rng.uniform(*BIN_Y)
    bin_yaw = rng.uniform(*BIN_YAW)
    return PickPlaceLayout(
        seed=parsed,
        table=SpawnPose(
            name=TABLE_NAME,
            file_name=f"{TABLE_NAME}.sdf",
            x=TABLE_CENTER_X,
            y=TABLE_CENTER_Y,
            z=TABLE_CENTER_Z,
            yaw=0.0,
        ),
        cube=SpawnPose(
            name=CUBE_NAME,
            file_name=f"{CUBE_NAME}.sdf",
            x=cube_x,
            y=cube_y,
            z=TABLE_TOP_Z + CUBE_SIZE / 2.0 + CUBE_CLEARANCE_Z,
            yaw=cube_yaw,
        ),
        bin=SpawnPose(
            name=BIN_NAME,
            file_name=f"{BIN_NAME}.sdf",
            x=bin_x,
            y=bin_y,
            z=TABLE_TOP_Z,
            yaw=bin_yaw,
        ),
    )


def center_gap(layout: PickPlaceLayout) -> float:
    """Horizontal distance between the cube centre and the bin centre."""
    return math.hypot(layout.cube.x - layout.bin.x, layout.cube.y - layout.bin.y)


def spawn_plan(enabled, seed, models_dir, world_name) -> list[dict]:
    """``ros_gz_sim create`` argument lists. Empty when the scene is off.

    ``enabled`` false is the sim.launch.py default, so the apartment world
    is not given a table, cube, or bin.
    """
    if not enabled:
        return []
    layout = layout_for_seed(seed)
    models = Path(models_dir)
    plan = []
    for pose in layout.models:
        plan.append(
            {
                "name": pose.name,
                "arguments": [
                    "-world",
                    str(world_name),
                    "-file",
                    str(models / pose.file_name),
                    "-name",
                    pose.name,
                    "-x",
                    f"{pose.x:.6f}",
                    "-y",
                    f"{pose.y:.6f}",
                    "-z",
                    f"{pose.z:.6f}",
                    "-Y",
                    f"{pose.yaw:.6f}",
                ],
            }
        )
    return plan
