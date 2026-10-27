# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Parity trial log and the docs/results.md table."""

from __future__ import annotations

import json

import pytest

import demo_drive
import run_parity_trials as parity


def test_protocol_is_ten_trials_of_each_typed_goal(monkeypatch):
    def boom(*_args, **_kwargs):
        raise AssertionError("serial port opened from the parity runner")

    monkeypatch.setattr(demo_drive, "open_serial_port", boom)
    monkeypatch.setattr(demo_drive, "publish_agent_goal", boom)
    rows = parity.run_protocol(blockers=["ros2 is not on PATH"])
    assert parity.DEFAULT_TRIALS == 10
    assert len(rows) == 30
    by_goal: dict[str, list[dict]] = {}
    for row in rows:
        by_goal.setdefault(row["goal"], []).append(row)
    assert set(by_goal) == set(parity.DEFAULT_GOALS)
    for goal, group in by_goal.items():
        assert len(group) == 10
        assert [r["trial"] for r in group] == list(range(1, 11))
        assert {r["launch"] for r in group} == {demo_drive.PARITY_LAUNCH}
        assert {r["transport"] for r in group} == {"sim"}
        assert {r["real"] for r in group} == {False}
        location = goal.removeprefix("go to the ")
        assert {r["mission_command"] for r in group} == {f"navigate:{location}"}
        for row in group:
            assert row["sim_status"] == "not_run"
            assert row["sim_time_s"] is None
            assert row["sim_interventions"] is None
            assert row["sim_failure"] is None
            assert row["sim_bag"] is None
            assert "placeholder" in row["sim_note"]
            assert row["real_status"] is None
            assert row["real_time_s"] is None
            assert row["real_interventions"] is None
            assert row["real_failure"] is None
            assert row["real_bag"] is None


def test_table_leaves_real_blank_and_records_a_measured_failure():
    rows = parity.run_protocol(trials=1, goals=("go to the kitchen",), blockers=["gz is not on PATH"])
    failed = {
        "trial": 2,
        "goal": "go to the kitchen",
        "location": "kitchen",
        "mission_command": "navigate:kitchen",
        "launch": demo_drive.PARITY_LAUNCH,
        "transport": "sim",
        "real": False,
        "sim_status": "fail",
        "sim_time_s": None,
        "sim_interventions": 1,
        "sim_failure": "not reached within 60 s",
        "sim_bag": parity.planned_bag_path("sim", "kitchen", 2),
        "sim_note": None,
        "real_status": None,
        "real_time_s": None,
        "real_interventions": None,
        "real_failure": None,
        "real_bag": None,
    }
    text = parity.render(rows + [failed], blockers=["gz is not on PATH"])
    assert "not reached within 60 s" in text
    assert "Sim failure" in text
    assert "~/rosbags/parity_sim_kitchen_trial02" in text
    assert "Real status" in text
    # Placeholder row: real cells empty. Split keeps the leading pipe.
    placeholder = next(line for line in text.splitlines() if line.startswith("| go to the kitchen | 1 |"))
    cells = [cell.strip() for cell in placeholder.strip("|").split("|")]
    assert cells[3] == "not_run"
    assert cells[4] == ""  # time
    assert cells[5] == ""  # interventions
    assert cells[6] == ""  # failure
    assert cells[7] == ""  # bag
    assert cells[8:] == ["", "", "", "", ""]
    assert "17.0" not in text
    assert "PASS" not in text


def test_summary_does_not_count_placeholders_as_passes_or_failures():
    rows = parity.run_protocol(blockers=["ros2 is not on PATH"])
    summary = parity.summarize(rows)
    assert {item["goal"] for item in summary} == set(parity.DEFAULT_GOALS)
    for item in summary:
        assert item["planned"] == 10
        assert item["sim_completed"] == 0
        assert item["sim_passed"] is None
        assert item["sim_failed"] is None
        assert item["sim_time"] is None
        assert item["real_completed"] is None


def test_main_writes_log_and_rejects_real_mode_without_the_flag(tmp_path, monkeypatch):
    def boom(*_args, **_kwargs):
        raise AssertionError("hardware opener called")

    monkeypatch.setattr(demo_drive, "open_serial_port", boom)
    results = tmp_path / "results.md"
    log = tmp_path / "trials.jsonl"
    assert parity.main(
        ["--results", str(results), "--log", str(log), "--trials", "1"]
    ) == 0
    text = results.read_text(encoding="utf-8")
    assert "OHH-37" in text
    assert "not_run" in text
    assert "go to the door" in text
    saved = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
    assert len(saved) == 3
    assert all(row["sim_bag"] is None and row["real_bag"] is None for row in saved)

    untouched = tmp_path / "nope.md"
    code = parity.main(
        ["--mode", "real", "--results", str(untouched), "--log", str(tmp_path / "nope.jsonl")]
    )
    assert code == 2
    assert not untouched.exists()


def test_real_flag_does_not_fill_real_column_or_open_serial(tmp_path, monkeypatch, capsys):
    def boom(*_args, **_kwargs):
        raise AssertionError("hardware opener called")

    monkeypatch.setattr(demo_drive, "open_serial_port", boom)
    results = tmp_path / "results.md"
    log = tmp_path / "trials.jsonl"
    assert parity.main(
        [
            "--mode",
            "real",
            "--real",
            "--trials",
            "1",
            "--results",
            str(results),
            "--log",
            str(log),
        ]
    ) == 0
    err = capsys.readouterr().err
    assert "--real" in err
    assert "does not open serial" in err
    rows = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
    assert rows
    assert all(row["real_status"] is None and row["real"] is False for row in rows)


def test_same_launch_on_every_row():
    rows = parity.run_protocol(trials=2, blockers=["gz is not on PATH"])
    assert len({row["launch"] for row in rows}) == 1
    assert rows[0]["launch"] == "ros2 launch omnibot_orchestration agent_nav2.launch.py"
