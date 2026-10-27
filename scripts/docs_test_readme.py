#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Extract bash fences from the root README and run the CPU-only ones.

Every ```bash / ```sh fence in README.md must be preceded by an annotation:

    <!-- docs-test: cpu -->
    <!-- docs-test: skip ros -->
    <!-- docs-test: skip hardware -->
    <!-- docs-test: skip gpu -->

cpu fences run in a fresh virtualenv. skip fences are printed and not
executed. A fence with no annotation fails the script.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
README = ROOT / "README.md"

FENCE_OPEN = re.compile(r"^```(bash|sh)\s*$")
ANNOTATION = re.compile(
    r"^<!--\s*docs-test:\s*(cpu|skip\s+(hardware|ros|gpu))\s*-->\s*$"
)
CPU_COMMANDS = (
    "pip install ohho-os",
    "ohho doctor",
    "ohho sim --robot omnibot --seconds 2",
)
REQUIRED_SKIPS = ("hardware", "ros", "gpu")
FORBIDDEN = "not on pypi"
ENGINE_LICENSE_HREF = "https://github.com/ohho-robotics/ohho-os/blob/main/LICENSE"
STALE_ENGINE_LICENSE_HREF = "https://github.com/ohho-robotics/ohho-sdk/blob/main/LICENSE"
MARKDOWN_SKIP_DIRS = {".git", ".venv", "Packages", "build", "install", "node_modules"}


class Fence:
    def __init__(self, kind: str, skip: str | None, start: int, end: int, body: str):
        self.kind = kind
        self.skip = skip
        self.start = start
        self.end = end
        self.body = body


def _fail(message: str) -> None:
    raise SystemExit(f"docs-test: {message}")


def parse_readme(text: str) -> list[Fence]:
    lines = text.splitlines()
    fences: list[Fence] = []
    index = 0
    while index < len(lines):
        if not FENCE_OPEN.match(lines[index]):
            index += 1
            continue
        start = index + 1  # 1-based line of the opening fence
        annotation_line = _annotation_before(lines, index)
        if annotation_line is None:
            _fail(f"README.md:{start}: bash fence has no docs-test annotation")
        match = ANNOTATION.match(lines[annotation_line])
        if match is None:
            _fail(
                f"README.md:{annotation_line + 1}: annotation must be "
                "'docs-test: cpu' or 'docs-test: skip hardware|ros|gpu'"
            )
        raw = match.group(1)
        if raw == "cpu":
            kind, skip = "cpu", None
        else:
            kind, skip = "skip", match.group(2)
        body_lines: list[str] = []
        index += 1
        closed = False
        while index < len(lines):
            if lines[index].startswith("```"):
                closed = True
                break
            body_lines.append(lines[index])
            index += 1
        if not closed:
            _fail(f"README.md:{start}: unclosed bash fence")
        end = index + 1
        fences.append(Fence(kind, skip, start, end, "\n".join(body_lines)))
        index += 1
    return fences


def _annotation_before(lines: list[str], fence_index: int) -> int | None:
    cursor = fence_index - 1
    while cursor >= 0 and lines[cursor].strip() == "":
        cursor -= 1
    if cursor < 0:
        return None
    if lines[cursor].lstrip().startswith("<!--"):
        return cursor
    return None


def check_fences(fences: list[Fence]) -> None:
    cpu = [fence for fence in fences if fence.kind == "cpu"]
    if len(cpu) != 1:
        _fail(f"expected exactly one cpu fence, found {len(cpu)}")
    commands = tuple(line.strip() for line in cpu[0].body.splitlines() if line.strip())
    if commands != CPU_COMMANDS:
        _fail(
            "cpu fence must be exactly: " + " / ".join(CPU_COMMANDS)
        )
    present = {fence.skip for fence in fences if fence.kind == "skip"}
    missing = [name for name in REQUIRED_SKIPS if name not in present]
    if missing:
        _fail("missing skip annotation(s): " + ", ".join(missing))


def check_engine_license_link() -> None:
    text = README.read_text(encoding="utf-8")
    if STALE_ENGINE_LICENSE_HREF in text:
        _fail(
            "README.md must not link engine licence to ohho-sdk; use "
            + ENGINE_LICENSE_HREF
        )
    if ENGINE_LICENSE_HREF not in text:
        _fail("README.md must link ohho-os licence at " + ENGINE_LICENSE_HREF)


def check_forbidden_phrase() -> None:
    offenders: list[str] = []
    for path in ROOT.rglob("*.md"):
        if any(part in MARKDOWN_SKIP_DIRS for part in path.parts):
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        if FORBIDDEN in text.lower():
            offenders.append(str(path.relative_to(ROOT)))
    if offenders:
        _fail("phrase 'not on PyPI' is present in " + ", ".join(offenders))


def print_plan(fences: list[Fence]) -> None:
    print(f"README.md: {len(fences)} bash fence(s)")
    for fence in fences:
        if fence.kind == "cpu":
            print(f"RUN cpu — README.md:{fence.start}-{fence.end}")
        else:
            print(
                f"SKIP {fence.skip} — README.md:{fence.start}-{fence.end} not executed"
            )


def run_cpu(fence: Fence) -> int:
    with tempfile.TemporaryDirectory(prefix="omnibot-docs-test-") as tmp:
        venv = Path(tmp) / "venv"
        print(f"harness: python -m venv {venv}", flush=True)
        created = subprocess.run([sys.executable, "-m", "venv", str(venv)], check=False)
        if created.returncode != 0:
            return created.returncode
        python = venv / "bin" / "python"
        env = os.environ.copy()
        env["VIRTUAL_ENV"] = str(venv)
        env["PATH"] = str(venv / "bin") + os.pathsep + env.get("PATH", "")
        env.pop("PYTHONHOME", None)
        print("harness: upgrade pip (not a README command)", flush=True)
        upgraded = subprocess.run(
            [str(python), "-m", "pip", "install", "--upgrade", "pip"],
            cwd=ROOT,
            env=env,
            check=False,
        )
        if upgraded.returncode != 0:
            return upgraded.returncode
        script = "set -euo pipefail\n" + fence.body + "\n"
        print("== README cpu commands ==", flush=True)
        completed = subprocess.run(
            ["bash", "-c", script],
            cwd=ROOT,
            env=env,
            check=False,
        )
        return completed.returncode


def main() -> int:
    check_engine_license_link()
    check_forbidden_phrase()
    fences = parse_readme(README.read_text(encoding="utf-8"))
    check_fences(fences)
    print_plan(fences)
    if "--list" in sys.argv[1:]:
        return 0
    cpu = next(fence for fence in fences if fence.kind == "cpu")
    code = run_cpu(cpu)
    if code != 0:
        print(f"docs-test: cpu fence failed ({code})", flush=True)
        return code
    skipped = [fence.skip for fence in fences if fence.kind == "skip"]
    print("docs-test: cpu ok; skipped " + ", ".join(skipped), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
