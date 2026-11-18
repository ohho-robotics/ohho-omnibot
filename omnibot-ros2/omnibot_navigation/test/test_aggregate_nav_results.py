# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""ROS-free tests for aggregate_nav_results.py."""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import aggregate_nav_results as aggregate  # noqa: E402


def test_aggregate_marks_all_passed(tmp_path):
    jsonl = tmp_path / "runs.jsonl"
    jsonl.write_text(
        json.dumps({"scenario": "a", "success": True, "collision_proximity_hits": 0}) + "\n",
        encoding="utf-8",
    )
    out = tmp_path / "out.json"
    assert aggregate.main([str(jsonl), "-o", str(out)]) == 0
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["suite"] == "scenario regression suite"
    assert data["all_passed"] is True
    assert data["scenario_count"] == 1
