#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Plan sim/real parity trials and render docs/results.md.

    python scripts/run_parity_trials.py
    python scripts/run_parity_trials.py --trials 10 --results docs/results.md

Each trial sends its typed goal through :func:`demo_drive.drive` (the same
path as ``scripts/demo_drive.py``). The default session is sim and does not
pass ``--real``. Real mode (``--mode real``) is rejected unless ``--real``
is set, and even then this process does not open a serial port and does not
fill the real column.

Sim cells are filled only from a completed trial. This environment cannot
start headless Gazebo, so the sim column is a placeholder (``not_run``) and
bag paths stay empty. No success, time, or intervention count is invented.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from collections import defaultdict
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import demo_drive

REPO = Path(__file__).resolve().parents[1]

DEFAULT_GOALS = (
    "go to the kitchen",
    "go to the desk",
    "go to the door",
)
DEFAULT_TRIALS = 10

ROW_FIELDS = (
    "trial",
    "goal",
    "location",
    "mission_command",
    "launch",
    "transport",
    "real",
    "sim_status",
    "sim_time_s",
    "sim_interventions",
    "sim_failure",
    "sim_bag",
    "sim_note",
    "real_status",
    "real_time_s",
    "real_interventions",
    "real_failure",
    "real_bag",
)


def sim_blockers() -> list[str]:
    """Reasons headless sim cannot be executed in this process."""
    missing = []
    if shutil.which("ros2") is None:
        missing.append("ros2 is not on PATH (ROS 2 Jazzy is required)")
    if shutil.which("gz") is None:
        missing.append("gz is not on PATH (Gazebo Harmonic is required)")
    return missing


def placeholder_note(blockers: list[str]) -> str:
    if blockers:
        detail = "; ".join(blockers)
    else:
        detail = (
            "ros2 and gz are on PATH, but this process does not start Gazebo "
            "or Nav2"
        )
    return (
        f"Sim was not run ({detail}). Sim results are placeholders. No bag was recorded."
    )


def planned_bag_path(mode: str, location: str, trial: int) -> str:
    """Path a completed trial would record. Not written until a trial runs."""
    return f"~/rosbags/parity_{mode}_{location}_trial{trial:02d}"


def run_protocol(
    *,
    goals: tuple[str, ...] = DEFAULT_GOALS,
    trials: int = DEFAULT_TRIALS,
    blockers: list[str] | None = None,
) -> list[dict]:
    """Dispatch every typed goal ``trials`` times on the sim demo_drive path.

    Real columns stay null. ``execute`` is false, so ROS is not required and
    no serial port is opened.
    """
    if trials < 1:
        raise ValueError("trials must be >= 1")
    found = sim_blockers() if blockers is None else list(blockers)
    note = placeholder_note(found)
    rows: list[dict] = []
    for goal in goals:
        for trial in range(1, trials + 1):
            link = demo_drive.SimTransport()
            dispatch = demo_drive.drive(
                goal,
                transport="sim",
                real=False,
                execute=False,
                link=link,
            )
            if link.payloads != [dispatch.agent_goal_payload]:
                raise RuntimeError(
                    "typed goal was not delivered on the demo_drive sim path"
                )
            row = {key: None for key in ROW_FIELDS}
            row.update(
                {
                    "trial": trial,
                    "goal": dispatch.agent_goal_payload,
                    "location": dispatch.location,
                    "mission_command": dispatch.mission_command,
                    "launch": dispatch.launch,
                    "transport": dispatch.transport,
                    "real": False,
                    "sim_status": "not_run",
                    "sim_note": note,
                }
            )
            rows.append(row)
    return rows


def _cell(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.1f}"
    return str(value).replace("|", "/").replace("\n", " ")


def _spread(times: list[float]) -> str | None:
    if not times:
        return None
    import statistics

    return (
        f"{min(times):.1f} / {statistics.median(times):.1f} / {max(times):.1f}"
    )


def summarize(rows: list[dict]) -> list[dict]:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in rows:
        grouped[str(row["goal"])].append(row)
    summary = []
    for goal, group in grouped.items():
        completed = [
            r for r in group if r.get("sim_status") in ("pass", "fail")
        ]
        passed_times = [
            float(r["sim_time_s"])
            for r in completed
            if r.get("sim_status") == "pass" and r.get("sim_time_s") is not None
        ]
        real_done = sum(
            1 for r in group if r.get("real_status") in ("pass", "fail")
        )
        summary.append(
            {
                "goal": goal,
                "planned": len(group),
                "sim_completed": len(completed),
                "sim_passed": (
                    sum(1 for r in completed if r.get("sim_status") == "pass")
                    if completed
                    else None
                ),
                "sim_failed": (
                    sum(1 for r in completed if r.get("sim_status") == "fail")
                    if completed
                    else None
                ),
                "sim_time": _spread(passed_times),
                "real_completed": real_done if real_done else None,
            }
        )
    return summary


def render(rows: list[dict], blockers: list[str] | None = None) -> str:
    """Markdown results table. Real cells are whatever the row stores (empty here)."""
    found = sim_blockers() if blockers is None else list(blockers)
    note = placeholder_note(found)
    goals = list(dict.fromkeys(str(r["goal"]) for r in rows))
    launch = rows[0]["launch"] if rows else demo_drive.PARITY_LAUNCH
    lines = [
        "# Sim / real parity results (OHH-37)",
        "",
        "Same launch, same typed goal, 10 trials per goal by default. "
        "This file is written by `scripts/run_parity_trials.py`. "
        "It does not invent a pass, a time, an intervention count, or a bag path.",
        "",
        "## Protocol",
        "",
        "Each trial calls `demo_drive.drive` (the `scripts/demo_drive.py` path). "
        "The typed string is what `/agent/goal` receives. ScriptedBrain turns it "
        "into the mission command shown below (`go to the kitchen` → `navigate:kitchen`).",
        "",
        f"- Launch (sim and real): `{launch}`",
        f"- Sim stack that must already be up before a sim trial: `{demo_drive.SIM_STACK_LAUNCH}`",
        "- Sim clock: `use_sim_time:=true`. Real clock: `use_sim_time:=false`.",
        "- Real mapping (`slam_toolbox` on the robot) and the real trials are not in this file.",
        "- Serial transport and `--mode real` require an explicit `--real`. "
        "This render used transport `sim` and `real=false`, and did not open a serial port.",
        "- A completed trial records a bag at "
        "`~/rosbags/parity_<sim|real>_<location>_trial<NN>`. "
        "No trial completed, so those paths were not created.",
        "",
        "Typed goals:",
        "",
    ]
    for goal in goals:
        lines.append(f"- `{goal}`")
    lines += [
        "",
        "## Why the sim column is a placeholder",
        "",
        note,
        "",
        "Status `not_run` means the goal was dispatched on the in-memory sim "
        "transport inside this process. The robot did not move. Time, "
        "interventions, failure, and bag are blank because they were not measured.",
        "",
        "## Why the real column is empty",
        "",
        "Mapping with slam_toolbox and the real trials are Varun's. "
        "Real cells are blank on purpose. They are not zeros and they are not a pass.",
        "",
        "## Summary",
        "",
        "| Goal | Trials planned | Sim completed | Sim passed | Sim failed | "
        "Sim time min / median / max (s) | Real completed |",
        "|---|---|---|---|---|---|---|",
    ]
    for item in summarize(rows):
        lines.append(
            "| {goal} | {planned} | {done} | {passed} | {failed} | {time} | {real} |".format(
                goal=_cell(item["goal"]),
                planned=_cell(item["planned"]),
                done=_cell(item["sim_completed"]),
                passed=_cell(item["sim_passed"]),
                failed=_cell(item["sim_failed"]),
                time=_cell(item["sim_time"]),
                real=_cell(item["real_completed"]),
            )
        )
    lines += [
        "",
        "## Trials",
        "",
        "| Goal | Trial | Mission command | Sim status | Sim time (s) | "
        "Sim interventions | Sim failure | Sim bag | Real status | Real time (s) | "
        "Real interventions | Real failure | Real bag |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for row in rows:
        lines.append(
            "| {goal} | {trial} | {mission} | {sim_status} | {sim_time} | "
            "{sim_interventions} | {sim_failure} | {sim_bag} | {real_status} | "
            "{real_time} | {real_interventions} | {real_failure} | {real_bag} |".format(
                goal=_cell(row.get("goal")),
                trial=_cell(row.get("trial")),
                mission=_cell(row.get("mission_command")),
                sim_status=_cell(row.get("sim_status")),
                sim_time=_cell(row.get("sim_time_s")),
                sim_interventions=_cell(row.get("sim_interventions")),
                sim_failure=_cell(row.get("sim_failure")),
                sim_bag=_cell(row.get("sim_bag")),
                real_status=_cell(row.get("real_status")),
                real_time=_cell(row.get("real_time_s")),
                real_interventions=_cell(row.get("real_interventions")),
                real_failure=_cell(row.get("real_failure")),
                real_bag=_cell(row.get("real_bag")),
            )
        )
    failures = [
        row
        for row in rows
        if row.get("sim_status") == "fail" or row.get("real_status") == "fail"
    ]
    lines += ["", "## Failures", ""]
    if not failures:
        lines.append(
            "No completed sim or real trial. No robot failure was observed, "
            "and none was invented. A measured failure uses status `fail` and "
            "puts the reason in the Sim failure or Real failure cell "
            "(timeout, pose tolerance, e-stop, or an abort after an intervention)."
        )
    else:
        lines.append("| Goal | Trial | Where | Reason | Bag |")
        lines.append("|---|---|---|---|---|")
        for row in failures:
            if row.get("sim_status") == "fail":
                lines.append(
                    f"| {_cell(row.get('goal'))} | {_cell(row.get('trial'))} | "
                    f"sim | {_cell(row.get('sim_failure'))} | {_cell(row.get('sim_bag'))} |"
                )
            if row.get("real_status") == "fail":
                lines.append(
                    f"| {_cell(row.get('goal'))} | {_cell(row.get('trial'))} | "
                    f"real | {_cell(row.get('real_failure'))} | {_cell(row.get('real_bag'))} |"
                )
    lines += [
        "",
        "## Bags",
        "",
        "No raw bags were recorded. Sim bag and Real bag are empty. "
        f"A completed sim kitchen trial 1 would be `{planned_bag_path('sim', 'kitchen', 1)}`.",
        "",
    ]
    return "\n".join(lines)


def write_outputs(
    rows: list[dict],
    blockers: list[str],
    results_path: Path,
    log_path: Path,
) -> None:
    results_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    results_path.write_text(render(rows, blockers), encoding="utf-8", newline="\n")
    body = "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows)
    log_path.write_text(body, encoding="utf-8", newline="\n")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS)
    parser.add_argument(
        "--goal",
        action="append",
        dest="goals",
        help="typed goal (repeatable). Default: kitchen, desk, door",
    )
    parser.add_argument(
        "--mode",
        choices=("sim", "real"),
        default="sim",
        help="real is rejected unless --real is also set",
    )
    parser.add_argument(
        "--real",
        action="store_true",
        help="required for --mode real. Does not open a serial port",
    )
    parser.add_argument(
        "--results",
        type=Path,
        default=REPO / "docs" / "results.md",
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=REPO / "docs" / "parity_trials.jsonl",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.mode == "real":
            demo_drive.select_transport("real", real=args.real)
        else:
            demo_drive.select_transport("sim", real=False)
    except demo_drive.RealModeRequired as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if args.trials < 1:
        print("trials must be >= 1", file=sys.stderr)
        return 2
    goals = tuple(args.goals) if args.goals else DEFAULT_GOALS
    blockers = sim_blockers()
    try:
        rows = run_protocol(goals=goals, trials=args.trials, blockers=blockers)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if args.mode == "real":
        print(
            "real mode acknowledged (--real). This process does not open serial, "
            "USB, or CAN, and does not fill the real column.",
            file=sys.stderr,
        )
    write_outputs(rows, blockers, args.results, args.log)
    print(f"wrote {args.results}")
    print(f"wrote {args.log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
