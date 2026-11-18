#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The VR profile notes must stay unpublished and name the real pins."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHANGELOG = ROOT / "CHANGELOG.md"
COMMANDS = ROOT / "docs" / "releases" / "vr-profile-tag-commands.md"
README = ROOT / "README.md"

REQUIRED = (
    "not yet tagged",
    "not released yet",
    "vr-profile-v1.0.0",
    "sim_vr.launch.py",
    "robot_vr.launch.py",
    "ghcr.io/ohho-robotics/omnibot-sim:vr-1.0.0",
    "placeholder",
    "d296c9be6cc898e11e8f671c2457292b30cec905",
    "b010b84c2bab7ec4f11f928af1b20be1fc31cc95",
    "ddb4beefd3e81f9c6ae0d21f75a9b0820023de05",
    "git tag -a vr-profile-v1.0.0",
    "gh release create vr-profile-v1.0.0",
    "Do not run",
)

FORBIDDEN = (
    "v1.0.0 is released",
    "release is published",
    "image is on GHCR",
    "has been tagged",
    "not on pypi",
)


def fail(message: str) -> None:
    print(f"vr-profile-notes: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> None:
    for path in (CHANGELOG, COMMANDS, README):
        if not path.is_file():
            fail(f"missing {path.relative_to(ROOT)}")
    text = "\n".join(path.read_text(encoding="utf-8") for path in (CHANGELOG, COMMANDS, README))
    folded = text.lower()
    for needle in REQUIRED:
        if needle not in text and needle not in folded:
            fail(f"missing {needle!r}")
    for needle in FORBIDDEN:
        if needle.lower() in folded:
            fail(f"forbidden phrase {needle!r}")
    script = (ROOT / "scripts" / "print-release-commands.sh").read_text(encoding="utf-8")
    for line in script.splitlines():
        stripped = line.strip()
        if stripped.startswith(("git tag", "git push", "gh release create")):
            fail("print-release-commands.sh would execute a tag or release command")
    print("vr-profile-notes: ok")


if __name__ == "__main__":
    main()
