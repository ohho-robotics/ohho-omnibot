#!/usr/bin/env python3
"""Run CPU test suites under coverage (same scope as .github/workflows/ci.yml)."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
OUT_DIR = REPO / "coverage"


def _run(cmd: list[str], **kwargs) -> int:
    print("+", " ".join(cmd), flush=True)
    return subprocess.run(cmd, cwd=REPO, **kwargs).returncode


def main() -> int:
    OUT_DIR.mkdir(exist_ok=True)
    json_path = OUT_DIR / "cpu-coverage.json"
    summary_path = OUT_DIR / "coverage-summary.txt"
    env = os.environ.copy()
    py = sys.executable

    if _run([py, "-m", "coverage", "erase"]) != 0:
        return 1

    pytest_tests = [
        "mecanum-kinematics/tests",
        "yahboom-python-driver/tests",
        "ros2-bev-stitcher/tests",
        "ohho-webrtc/tests",
        "omnibot-ros2/omnibot_arm/test",
        "omnibot-ai-ros2/omnibot_lerobot/test",
        "omnibot-ros2/omnibot_bringup/test/test_unbuilt_launch.py",
        "omnibot-ros2/omnibot_bringup/test/test_teleop_default.py",
        "omnibot-ros2/omnibot_bringup/test/test_teleop_safety.py",
        "omnibot-ros2/omnibot_bringup/test/test_pick_place_spawn.py",
        "omnibot-ros2/omnibot_bringup/test/test_descriptor.py",
        "omnibot-ros2/omnibot_bringup/test/test_sim_cameras.py",
        "omnibot-ros2/omnibot_bringup/test/test_sim_vr_launch.py",
        "omnibot-ros2/omnibot_bringup/test/test_docker_sim.py",
        "omnibot-ros2/omnibot_bringup/test/test_frame_launch_docs.py",
        "omnibot-ros2/omnibot_navigation/test",
        "omnibot-ros2/omnibot_description/test",
        "infra/observability/tests",
        "scripts/test_architecture_launch_table.py",
        "scripts/tests",
    ]
    if (
        _run(
            [py, "-m", "coverage", "run", "-m", "pytest", "-q", *pytest_tests],
            env=env,
        )
        != 0
    ):
        return 1

    if (
        _run(
            [
                py,
                "-m",
                "coverage",
                "run",
                "-a",
                "-m",
                "unittest",
                "discover",
                "-s",
                "omnibot-ai-engines/learning_engine/tests",
                "-t",
                "omnibot-ai-engines",
            ],
            env=env,
        )
        != 0
    ):
        return 1

    if (
        _run(
            [
                py,
                "-m",
                "coverage",
                "run",
                "-a",
                "-m",
                "unittest",
                "discover",
                "-s",
                "omnibot-ai-engines/agent_engine/tests",
                "-t",
                "omnibot-ai-engines",
            ],
            env=env,
        )
        != 0
    ):
        return 1

    env["PYTHONPATH"] = str(REPO / "omnibot-ai-engines")
    if (
        _run(
            [
                py,
                "-m",
                "coverage",
                "run",
                "-a",
                "-m",
                "pytest",
                "omnibot-ai-engines/data_engine/tests",
                "-q",
            ],
            env=env,
        )
        != 0
    ):
        return 1

    if _run([py, "-m", "coverage", "report"]) != 0:
        return 1
    if _run([py, "-m", "coverage", "json", "-o", str(json_path)]) != 0:
        return 1

    data = json.loads(json_path.read_text(encoding="utf-8"))
    total = data.get("totals", {})
    pct = total.get("percent_covered")
    lines = (
        f"CPU line coverage: {pct:.1f}%\n"
        f"  covered lines: {total.get('covered_lines')}/{total.get('num_statements')}\n"
    )
    summary_path.write_text(lines, encoding="utf-8")
    print(lines)
    step_summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if step_summary:
        with open(step_summary, "a", encoding="utf-8") as fh:
            fh.write("## CPU line coverage\n\n")
            fh.write(f"**{pct:.1f}%** line coverage ")
            fh.write(f"({total.get('covered_lines')}/{total.get('num_statements')} statements).\n\n")
            fh.write("Artifact: `cpu-coverage` (`coverage-summary.txt`).\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
