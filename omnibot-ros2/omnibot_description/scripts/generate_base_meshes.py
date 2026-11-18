# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Write simplified chassis and mecanum-wheel STLs from the xacro primitives.

These files are not CAD. There is no chassis or wheel STEP model in this
repo. Each mesh is metres, Z-up, and its origin is the link frame origin.
Regenerate after changing the wheel or plate properties in
``urdf/omnibot.urdf.xacro``:

    python omnibot-ros2/omnibot_description/scripts/generate_base_meshes.py

The wheel rollers follow the poses that used to be cylinder visuals in the
xacro: eight rollers, centre radius ``wheel_radius - roller_radius``,
``rpy = (cs * -pi/4, pi/2 + phi, 0)``. ``phi = 0`` is +Z. ``cs = +1`` is
the front-left / rear-right chirality.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_XACRO = PACKAGE_ROOT / "urdf" / "omnibot.urdf.xacro"
DEFAULT_OUTPUT = PACKAGE_ROOT / "meshes" / "base"
SEGMENTS = 24
ROLLER_COUNT = 8

_PROPERTY = re.compile(
    r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"',
)


def xacro_numbers(xacro_path: Path) -> dict[str, float]:
    """Read numeric ``xacro:property`` values. Expressions are skipped."""
    text = xacro_path.read_text(encoding="utf-8")
    numbers: dict[str, float] = {}
    for name, raw in _PROPERTY.findall(text):
        if raw.startswith("${"):
            continue
        try:
            numbers[name] = float(raw)
        except ValueError:
            continue
    return numbers


def _require(numbers: dict[str, float], name: str) -> float:
    if name not in numbers:
        raise SystemExit(f"xacro is missing numeric property {name}")
    return numbers[name]


def _rot_x(angle: float) -> tuple[tuple[float, float, float], ...]:
    c, s = math.cos(angle), math.sin(angle)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(angle: float) -> tuple[tuple[float, float, float], ...]:
    c, s = math.cos(angle), math.sin(angle)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(angle: float) -> tuple[tuple[float, float, float], ...]:
    c, s = math.cos(angle), math.sin(angle)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _matmul(
    left: tuple[tuple[float, float, float], ...],
    right: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(sum(left[i][k] * right[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _rpy(roll: float, pitch: float, yaw: float) -> tuple[tuple[float, float, float], ...]:
    """URDF fixed-axis rpy: Rz(yaw) * Ry(pitch) * Rx(roll)."""
    return _matmul(_rot_z(yaw), _matmul(_rot_y(pitch), _rot_x(roll)))


def _apply(
    rotation: tuple[tuple[float, float, float], ...],
    point: tuple[float, float, float],
    translation: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(
        sum(rotation[i][k] * point[k] for k in range(3)) + translation[i]
        for i in range(3)
    )


def _normal(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    c: tuple[float, float, float],
) -> tuple[float, float, float]:
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return (nx / length, ny / length, nz / length)


def _cylinder(
    radius: float,
    length: float,
    *,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
    segments: int = SEGMENTS,
) -> list[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]]:
    """Closed cylinder along local Z, then placed with a URDF origin."""
    rotation = _rpy(*rpy)
    half = length / 2.0
    rings = []
    for z in (half, -half):
        ring = []
        for index in range(segments):
            theta = 2.0 * math.pi * index / segments
            local = (radius * math.cos(theta), radius * math.sin(theta), z)
            ring.append(_apply(rotation, local, xyz))
        rings.append(ring)
    top, bottom = rings
    center_top = _apply(rotation, (0.0, 0.0, half), xyz)
    center_bottom = _apply(rotation, (0.0, 0.0, -half), xyz)
    triangles = []
    for index in range(segments):
        nxt = (index + 1) % segments
        triangles.append((bottom[index], bottom[nxt], top[nxt]))
        triangles.append((bottom[index], top[nxt], top[index]))
        triangles.append((center_top, top[index], top[nxt]))
        triangles.append((center_bottom, bottom[nxt], bottom[index]))
    return triangles


def _box(
    size: tuple[float, float, float],
) -> list[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]]:
    """Axis-aligned box centred on the origin. Size is full x, y, z extent."""
    hx, hy, hz = (value / 2.0 for value in size)
    corners = [
        (sx * hx, sy * hy, sz * hz)
        for sx in (-1.0, 1.0)
        for sy in (-1.0, 1.0)
        for sz in (-1.0, 1.0)
    ]
    # corners index = sz + 2*sy + 4*sx, each of sx,sy,sz in {0,1} = {-,+}.
    # Quad order is CCW when viewed from outside.
    faces = (
        (0, 1, 3, 2),  # -X
        (4, 6, 7, 5),  # +X
        (0, 4, 5, 1),  # -Y
        (2, 3, 7, 6),  # +Y
        (0, 2, 6, 4),  # -Z
        (1, 5, 7, 3),  # +Z
    )
    triangles = []
    for a, b, c, d in faces:
        triangles.append((corners[a], corners[b], corners[c]))
        triangles.append((corners[a], corners[c], corners[d]))
    return triangles


def chassis_plate_triangles(numbers: dict[str, float]):
    return _box(
        (
            _require(numbers, "chassis_length"),
            _require(numbers, "chassis_width"),
            _require(numbers, "plate_thickness"),
        )
    )


def wheel_triangles(numbers: dict[str, float], *, chirality: float):
    radius = _require(numbers, "wheel_radius")
    width = _require(numbers, "wheel_width")
    hub_scale = _require(numbers, "hub_radius_scale")
    roller_radius = _require(numbers, "roller_radius")
    roller_length = _require(numbers, "roller_length")
    centre = radius - roller_radius
    triangles = _cylinder(
        radius * hub_scale,
        width,
        xyz=(0.0, 0.0, 0.0),
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    for index in range(ROLLER_COUNT):
        phi = index * (2.0 * math.pi / ROLLER_COUNT)
        triangles.extend(
            _cylinder(
                roller_radius,
                roller_length,
                xyz=(centre * math.sin(phi), 0.0, centre * math.cos(phi)),
                rpy=(chirality * (-math.pi / 4.0), math.pi / 2.0 + phi, 0.0),
            )
        )
    return triangles


def _write_stl(path: Path, solid: str, triangles) -> None:
    lines = [f"solid {solid}"]
    for tri in triangles:
        nx, ny, nz = _normal(*tri)
        lines.append(f"  facet normal {nx:.8f} {ny:.8f} {nz:.8f}")
        lines.append("    outer loop")
        for vertex in tri:
            lines.append(
                f"      vertex {vertex[0]:.8f} {vertex[1]:.8f} {vertex[2]:.8f}"
            )
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {solid}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def write_base_meshes(xacro_path: Path, output_dir: Path) -> list[Path]:
    """Write the three simplified base STLs. Returns the paths."""
    numbers = xacro_numbers(xacro_path)
    written = []
    plate = output_dir / "chassis_plate.stl"
    _write_stl(plate, "simplified_chassis_plate", chassis_plate_triangles(numbers))
    written.append(plate)
    for name, chirality in (
        ("mecanum_wheel_cs_pos.stl", 1.0),
        ("mecanum_wheel_cs_neg.stl", -1.0),
    ):
        path = output_dir / name
        solid = "simplified_" + path.stem
        _write_stl(path, solid, wheel_triangles(numbers, chirality=chirality))
        written.append(path)
    return written


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xacro", type=Path, default=DEFAULT_XACRO)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    paths = write_base_meshes(args.xacro, args.output)
    for path in paths:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
