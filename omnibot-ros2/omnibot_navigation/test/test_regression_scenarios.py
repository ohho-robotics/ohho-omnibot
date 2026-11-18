# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""ROS-free checks of config/nav_regression_scenarios.yaml."""

import sys
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))

import regression_scenario_loader as rsl  # noqa: E402
import test_nav_assets as nav_assets  # noqa: E402
import waypoint_loader as wl  # noqa: E402

SCENARIOS_YAML = PACKAGE / "config" / "nav_regression_scenarios.yaml"
MIN_CLEARANCE_M = 0.35


@pytest.fixture(scope="module")
def sim_map():
    return nav_assets.load_sim_map_grid()


def test_exactly_ten_scenarios():
    scenarios = rsl.load_regression_scenarios(SCENARIOS_YAML)
    assert len(scenarios) == 10


def test_scenario_ids_are_unique_and_stable():
    ids = rsl.scenario_ids(SCENARIOS_YAML)
    assert len(ids) == len(set(ids))
    assert "origin_to_kitchen" in ids


def test_starts_and_goals_are_in_free_space(sim_map):
    for scenario in rsl.load_regression_scenarios(SCENARIOS_YAML).values():
        for label, x, y in (
            ("start", scenario.start_x, scenario.start_y),
            ("goal", scenario.goal.x, scenario.goal.y),
        ):
            row, col = sim_map.cell(x, y)
            assert sim_map.free[row, col], f"{scenario.id} {label} ({x}, {y}) not free"
            assert not nav_assets._occupied_within(
                sim_map, x, y, MIN_CLEARANCE_M
            ), f"{scenario.id} {label} too close to map obstacles"


def test_obstacle_centers_are_in_free_space(sim_map):
    for scenario in rsl.load_regression_scenarios(SCENARIOS_YAML).values():
        obst = scenario.obstacle
        row, col = sim_map.cell(obst.x, obst.y)
        assert sim_map.free[row, col], f"{scenario.id} obstacle center not free"


def test_goals_match_waypoint_targets_where_named():
    waypoints = wl.load_waypoints(PACKAGE / "config" / "waypoints.yaml")
    scenarios = rsl.load_regression_scenarios(SCENARIOS_YAML)
    for name in ("kitchen", "desk", "door"):
        match = [s for s in scenarios.values() if s.goal.x == waypoints[name].x]
        assert match, f"no regression scenario targets waypoint {name}"
