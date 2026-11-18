# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Headless Gazebo integration test for one Proof v0 regression scenario.

Pick the scenario with OMNIBOT_NAV_REGRESSION_SCENARIO=<id> (see
config/nav_regression_scenarios.yaml). Each run spawns the robot at the
scenario start pose, inserts the static obstacle box, and navigates to the goal.

Needs ROS 2 Jazzy + Gazebo Harmonic; registered in CMake only when
OMNIBOT_NAV_SIM=1 at configure time.
"""

import math
import os
import subprocess
import sys
import unittest
from pathlib import Path

import launch_testing
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from go_to_waypoint import (  # noqa: E402
    DEFAULT_TIMEOUT_S,
    GOAL_XY_TOLERANCE_M,
    GOAL_YAW_TOLERANCE_RAD,
    START_POSE_TOLERANCE_M,
    WaypointGoalClient,
    emit,
)
from regression_scenario_loader import load_regression_scenarios  # noqa: E402

SCENARIO_ID = os.environ.get("OMNIBOT_NAV_REGRESSION_SCENARIO", "origin_to_kitchen")
RESULTS_FILE = os.environ.get("OMNIBOT_NAV_RESULTS_FILE", "")
TIMEOUT_S = float(os.environ.get("OMNIBOT_NAV_TIMEOUT_S", DEFAULT_TIMEOUT_S))


def generate_test_description():
    scenarios = load_regression_scenarios()
    if SCENARIO_ID not in scenarios:
        raise RuntimeError(
            f"OMNIBOT_NAV_REGRESSION_SCENARIO={SCENARIO_ID!r} not in "
            f"{sorted(scenarios)}"
        )
    scenario = scenarios[SCENARIO_ID]
    nav = FindPackageShare("omnibot_navigation").find("omnibot_navigation")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav, "launch", "sim_navigation.launch.py")
                ),
                launch_arguments={
                    "gui": "false",
                    "world": scenario.world,
                    "initial_x": str(scenario.start_x),
                    "initial_y": str(scenario.start_y),
                    "initial_yaw": str(scenario.start_yaw),
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {}


class TestNavRegressionScenario(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = WaypointGoalClient(use_sim_time=True)
        cls.scenario = load_regression_scenarios()[SCENARIO_ID]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_reaches_goal_from_scenario_start(self):
        self.assertTrue(
            self.node.wait_until_ready(360.0),
            "sim, TF map->base_link, or Nav2 lifecycle did not become ready in 360 s",
        )
        spawn_script = SCRIPTS / "spawn_regression_obstacle.py"
        status = subprocess.run(
            [sys.executable, str(spawn_script), SCENARIO_ID],
            check=False,
        ).returncode
        self.assertEqual(status, 0, "failed to spawn regression obstacle in Gazebo")
        self.node.spin_for(2.0)

        truth = self.node.ground_truth()
        self.assertIsNotNone(truth, "no /odom before the goal was sent")
        self.assertLessEqual(
            math.hypot(
                truth[0] - self.scenario.start_x,
                truth[1] - self.scenario.start_y,
            ),
            START_POSE_TOLERANCE_M,
            f"robot was not at the scenario start pose when the goal was sent: {truth}",
        )

        result = self.node.navigate_to(self.scenario.goal, TIMEOUT_S)
        result["scenario"] = SCENARIO_ID
        emit(result, RESULTS_FILE or None)
        print(
            f"EVIDENCE scenario={SCENARIO_ID} success={result['success']} "
            f"time_to_goal_s={result.get('time_to_goal_s')} "
            f"collision_proximity_hits={result.get('collision_proximity_hits')} "
            f"final_error_xy_m={result.get('final_error_xy_m')} "
            f"(limit {TIMEOUT_S:.0f} s, {GOAL_XY_TOLERANCE_M} m, "
            f"{GOAL_YAW_TOLERANCE_RAD} rad)",
            flush=True,
        )
        self.assertTrue(result["success"], f"{SCENARIO_ID}: {result['reason']}")
