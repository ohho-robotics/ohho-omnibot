#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Write the nightly Markdown report for the scenario regression suite.

Reads JSONL result lines and optional aggregate JSON metadata. No ROS dependency.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import summarize_nav_results as summary  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("jsonl", help="nav-results.jsonl")
    parser.add_argument("-o", "--output", required=True, help="Markdown report path")
    parser.add_argument(
        "--badge-url",
        default="",
        help="shields.io or workflow badge URL to embed",
    )
    parser.add_argument(
        "--workflow-url",
        default="",
        help="link to the GitHub Actions run (Proof v0 evidence)",
    )
    args = parser.parse_args(argv)

    rows = summary.load([args.jsonl])
    passed = sum(1 for r in rows if r.get("success"))
    total = len(rows)
    status = "passing" if passed == total and total else "failing"

    lines = [
        "# Proof v0 — scenario regression suite",
        "",
    ]
    if args.badge_url:
        lines.append(f"![scenario regression suite]({args.badge_url})")
        lines.append("")
    lines.append(
        f"**Status:** {status} — {passed}/{total} scenarios reached the goal in time "
        "with acceptable pose error."
    )
    if args.workflow_url:
        lines.append(f"**CI run:** {args.workflow_url}")
    lines.append("")
    lines.append("## Results")
    lines.append("")
    lines.append(summary.render(rows))
    lines.append("")
    lines.append(
        "_Collision column counts lidar proximity hits (scan range < 0.20 m) "
        "during navigation — a conservative proxy, not physics contact events._"
    )
    Path(args.output).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0 if passed == total and total else 1


if __name__ == "__main__":
    sys.exit(main())
