# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Ensure docs/architecture.md launch tables match the generator script."""

import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
ARCH_DOC = REPO_ROOT / "docs" / "architecture.md"
GENERATOR = SCRIPTS / "generate_architecture_launch_table.py"

sys.path.insert(0, str(SCRIPTS))
from generate_architecture_launch_table import BEGIN_MARKER, END_MARKER  # noqa: E402

BEGIN = BEGIN_MARKER
END = END_MARKER


def _extract_generated_block(text: str) -> str:
    pattern = re.compile(
        rf"{re.escape(BEGIN)}\s*\n(.*?)\n\s*{re.escape(END)}",
        flags=re.DOTALL,
    )
    match = pattern.search(text)
    assert match, f"Missing {BEGIN} ... {END} markers in {ARCH_DOC}"
    return match.group(1).rstrip() + "\n"


def _run_generator() -> str:
    completed = subprocess.run(
        [sys.executable, str(GENERATOR)],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )
    assert completed.returncode == 0, completed.stderr or completed.stdout
    return completed.stdout


def test_launch_table_matches_generator():
    doc = ARCH_DOC.read_text(encoding="utf-8")
    embedded = _extract_generated_block(doc)
    generated = _run_generator()
    assert embedded == generated


def test_generator_lists_sim_launch_nodes():
    generated = _run_generator()
    assert "omnibot_bringup/launch/sim.launch.py" in generated
    assert "ros_gz_bridge" in generated
    assert "/cmd_vel" in generated


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
