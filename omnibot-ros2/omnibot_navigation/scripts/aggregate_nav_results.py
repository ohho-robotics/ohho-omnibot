#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Merge NAV_RESULT JSONL lines into one Proof v0 evidence JSON file.

    aggregate_nav_results.py nav-results.jsonl -o nav-regression-results.json

No ROS dependency.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone


def load_jsonl(path: str) -> list[dict]:
    rows = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def aggregate(rows: list[dict], suite: str) -> dict:
    passed = sum(1 for r in rows if r.get("success"))
    return {
        "suite": suite,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "scenario_count": len(rows),
        "passed": passed,
        "failed": len(rows) - passed,
        "all_passed": passed == len(rows) and len(rows) > 0,
        "runs": rows,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("jsonl", help="nav-results.jsonl from run_nav_regression_suite.sh")
    parser.add_argument("-o", "--output", required=True, help="output JSON path")
    parser.add_argument(
        "--suite",
        default="scenario regression suite",
        help="label stored in the JSON (Proof v0)",
    )
    args = parser.parse_args(argv)
    rows = load_jsonl(args.jsonl)
    if not rows:
        print("no results in jsonl", file=sys.stderr)
        return 1
    payload = aggregate(rows, args.suite)
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")
    return 0 if payload["all_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
