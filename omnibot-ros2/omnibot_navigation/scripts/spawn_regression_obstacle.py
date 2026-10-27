#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Spawn the static obstacle box for one scenario regression run in Gazebo.

    ros2 run omnibot_navigation spawn_regression_obstacle.py origin_to_kitchen

Simulation only. Uses ros_gz_sim create with a generated SDF box.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

from regression_scenario_loader import load_regression_scenarios, resolve_scenarios_path


def _sdf_for_box(name: str, sx: float, sy: float, sz: float) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>0.9 0.45 0.1 1</ambient>
          <diffuse>0.9 0.45 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def spawn(scenario_id: str, scenarios_file: str, world: str) -> int:
    scenarios = load_regression_scenarios(resolve_scenarios_path(scenarios_file))
    if scenario_id not in scenarios:
        print(
            f"unknown scenario {scenario_id!r}; known: {sorted(scenarios)}",
            file=sys.stderr,
        )
        return 2
    scenario = scenarios[scenario_id]
    obst = scenario.obstacle
    model_name = f"regression_{scenario_id}_{obst.name}".replace(" ", "_")
    z_center = obst.size_z / 2.0
    sdf = _sdf_for_box(model_name, obst.size_x, obst.size_y, obst.size_z)
    with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as handle:
        handle.write(sdf)
        sdf_path = handle.name
    cmd = [
        "ros2",
        "run",
        "ros_gz_sim",
        "create",
        "--world",
        world or scenario.world,
        "-name",
        model_name,
        "-file",
        sdf_path,
        "-x",
        str(obst.x),
        "-y",
        str(obst.y),
        "-z",
        str(z_center),
        "-Y",
        str(obst.yaw),
    ]
    print(" ".join(cmd), flush=True)
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as err:
        return err.returncode
    finally:
        Path(sdf_path).unlink(missing_ok=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("scenario_id", help="id from nav_regression_scenarios.yaml")
    parser.add_argument(
        "--scenarios-file",
        default="nav_regression_scenarios.yaml",
        help="scenario YAML (name under config/ or path)",
    )
    parser.add_argument("--world", default="", help="override Gazebo world name")
    args = parser.parse_args(argv)
    return spawn(args.scenario_id, args.scenarios_file, args.world)


if __name__ == "__main__":
    sys.exit(main())
