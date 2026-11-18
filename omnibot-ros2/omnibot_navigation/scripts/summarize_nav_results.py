#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Summarise NAV_RESULT JSON lines (go_to_waypoint.py / the scenario test).

    summarize_nav_results.py nav-results.jsonl [more.jsonl ...]

Prints a Markdown table of time-to-goal per scenario (one row per run, then a
per-scenario summary) and exits 1 if any run failed. No ROS dependency.
"""

from __future__ import annotations

import json
import statistics
import sys
from collections import defaultdict


def load(paths):
    rows = []
    for path in paths:
        with open(path, encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if line:
                    rows.append(json.loads(line))
    return rows


def _name(row):
    return row.get("scenario") or row.get("waypoint") or "?"


def _fmt(value, spec="{:.1f}"):
    return "-" if value is None else spec.format(value)


def render(rows) -> str:
    out = [
        "| Run | Scenario | Result | Time to goal (sim s) | Wall s | Collisions (scan) "
        "| Final xy error (m) | Final yaw error (rad) |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for index, row in enumerate(rows, 1):
        collisions = row.get("collision_proximity_hits")
        out.append(
            f"| {index} | {_name(row)} | "
            f"{'PASS' if row.get('success') else 'FAIL: ' + str(row.get('reason'))} | "
            f"{_fmt(row.get('time_to_goal_s'))} | {_fmt(row.get('wall_time_s'))} | "
            f"{collisions if collisions is not None else '-'} | "
            f"{_fmt(row.get('final_error_xy_m'), '{:.2f}')} | "
            f"{_fmt(row.get('final_error_yaw_rad'), '{:.2f}')} |"
        )
    by_name = defaultdict(list)
    for row in rows:
        by_name[_name(row)].append(row)
    out += [
        "",
        "| Scenario | Runs | Passed | Time to goal min / median / max (sim s) |",
        "|---|---|---|---|",
    ]
    for name, group in by_name.items():
        times = [r["time_to_goal_s"] for r in group if r.get("success")]
        spread = (
            f"{min(times):.1f} / {statistics.median(times):.1f} / {max(times):.1f}"
            if times
            else "-"
        )
        passed = sum(1 for r in group if r.get("success"))
        out.append(f"| {name} | {len(group)} | {passed} | {spread} |")
    return "\n".join(out)


def main(argv):
    if not argv:
        print(__doc__, file=sys.stderr)
        return 2
    rows = load(argv)
    if not rows:
        print("no results found", file=sys.stderr)
        return 1
    print(render(rows))
    return 0 if all(r.get("success") for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
