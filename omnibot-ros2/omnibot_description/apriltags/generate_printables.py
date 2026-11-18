#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Write the AprilTag PDF, jig STL/SCAD, Gazebo meshes, and nominal xacro."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from apriltag_mounts import documentation_rows, generate_all, mounts


def main() -> int:
    description = Path(__file__).resolve().parents[1]
    generate_all(description)
    for mount in mounts():
        xyz = " ".join(f"{value:.9f}" for value in mount.origin_base)
        print(f"id {mount.tag_id} {mount.link} {xyz}")
    for row in documentation_rows():
        print(row)
    return 0


if __name__ == "__main__":
    sys.exit(main())
