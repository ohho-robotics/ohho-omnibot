# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""ROS-free checks that nav-sim CI can build and separate Gazebo runs."""

import re
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
WORKFLOW = REPO / ".github" / "workflows" / "nav-sim.yml"
RUNNER = PACKAGE / "scripts" / "run_nav_scenarios.sh"


def test_nav_sim_builds_bringup_workspace_dependencies():
    """colcon --packages-select does not build packages it does not name.

    omnibot_bringup exec_depends omnibot_hybrid and omnibot_rl. Without their
    install prefixes, the bringup configure step fails looking for package.sh.
    """
    text = WORKFLOW.read_text(encoding="utf-8")
    selected = set()
    for match in re.finditer(r"--packages-select\s+([^\n\\]+)", text):
        selected.update(match.group(1).split())
    for name in (
        "omnibot_description",
        "omnibot_arm",
        "omnibot_hybrid",
        "omnibot_rl",
        "omnibot_bringup",
        "omnibot_navigation",
    ):
        assert name in selected
    assert "src/omnibot_hybrid" in text
    assert "src/omnibot_rl" in text


def test_nav_scenarios_isolate_gz_transport_and_ros_domain():
    text = RUNNER.read_text(encoding="utf-8")
    assert "GZ_PARTITION=" in text
    assert "ROS_DOMAIN_ID=" in text
    assert "cleanup_sim" in text
