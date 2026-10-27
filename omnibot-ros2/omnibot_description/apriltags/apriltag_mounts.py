# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Nominal OmniBot AprilTag mounts, printables, and Gazebo textures.

The tag36h11 codewords and bit coordinates are taken from AprilRobotics
apriltag ``tag36h11.c`` (Copyright (C) 2013-2016, The Regents of the
University of Michigan). That project is BSD-2-Clause:

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Poses are nominal CAD. They are not a measured calibration.
"""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

FAMILY = "tag36h11"
TAG_SIZE_M = 0.08
CARD_M = TAG_SIZE_M * 10.0 / 8.0
TILT_RAD = math.pi / 4.0
JIG_BASE_M = 0.004
FRONT_ARM_GAP_M = 0.010
REAR_EDGE_MARGIN_M = 0.005
ARM_SIDE_GAP_M = 0.012
REAR_CENTER_Y_M = -0.070
ARM_CENTER_X_M = -0.078
MESH_THICKNESS_M = 0.0005

# AprilRobotics tag36h11.c codedata[0:3] and tag36h11_create bit_x/bit_y.
TAG36H11_CODES = (
    0x0000000D7E00984B,
    0x0000000DDA664CA7,
    0x0000000DC4A1C821,
)
BIT_X = (
    1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4,
    6, 5, 4, 3, 2, 5, 4, 3, 4, 1, 1, 1, 1, 1, 2, 2, 2, 3,
)
BIT_Y = (
    1, 1, 1, 1, 1, 2, 2, 2, 3, 1, 2, 3, 4, 5, 2, 3, 4, 3,
    6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4,
)

A4_W_PT = 595.28
A4_H_PT = 841.89
MM_TO_PT = 72.0 / 25.4


@dataclass(frozen=True)
class Chassis:
    """Literals copied from omnibot.urdf.xacro. Tests refuse a drift."""

    length: float = 0.265
    width: float = 0.250
    plate_thickness: float = 0.006
    standoff_height: float = 0.160
    arm_base_x: float = 0.055
    arm_body: tuple = (0.075, 0.065, 0.055)
    waveshare_origin_x: float = -0.031
    waveshare_size_x: float = 0.080

    @property
    def top_plate_z(self) -> float:
        return self.plate_thickness + self.standoff_height

    @property
    def top_surface_z(self) -> float:
        return self.top_plate_z + self.plate_thickness / 2.0

    @property
    def arm_origin(self) -> tuple:
        return (self.arm_base_x, 0.0, self.top_surface_z)

    @property
    def arm_front_x(self) -> float:
        return self.arm_base_x + self.arm_body[0] / 2.0

    @property
    def arm_rear_x(self) -> float:
        return self.arm_base_x + self.waveshare_origin_x - self.waveshare_size_x / 2.0

    @property
    def arm_side_y(self) -> float:
        return self.arm_body[1] / 2.0


@dataclass(frozen=True)
class TagMount:
    tag_id: int
    name: str
    parent: str
    parent_origin: tuple
    location: str
    origin_base: tuple
    x_axis: tuple
    y_axis: tuple
    z_axis: tuple

    @property
    def link(self) -> str:
        return self.name + "_link"

    @property
    def joint(self) -> str:
        return self.name + "_joint"

    def origin_parent(self) -> tuple:
        return tuple(
            self.origin_base[i] - self.parent_origin[i] for i in range(3)
        )

    def matrix(self) -> list:
        """Row-major T such that p_base = T * p_tag."""
        axes = (self.x_axis, self.y_axis, self.z_axis)
        rows = []
        for row in range(3):
            rows.extend(axes[col][row] for col in range(3))
            rows.append(self.origin_base[row])
        rows.extend((0.0, 0.0, 0.0, 1.0))
        return rows

    def rpy(self) -> tuple:
        return rotation_to_rpy(self.x_axis, self.y_axis, self.z_axis)

    def card_corners(self) -> tuple:
        half = CARD_M / 2.0
        corners = []
        for sx in (-half, half):
            for sy in (-half, half):
                corners.append(
                    tuple(
                        self.origin_base[i]
                        + sx * self.x_axis[i]
                        + sy * self.y_axis[i]
                        for i in range(3)
                    )
                )
        return tuple(corners)


def rotation_to_rpy(x_axis, y_axis, z_axis) -> tuple:
    """URDF fixed-axis roll, pitch, yaw. R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r00, r10, r20 = x_axis
    r01, r11, r21 = y_axis
    r02, r12, r22 = z_axis
    sy = math.hypot(r00, r10)
    if sy > 1e-9:
        roll = math.atan2(r21, r22)
        pitch = math.atan2(-r20, sy)
        yaw = math.atan2(r10, r00)
    else:
        roll = math.atan2(-r12, r11)
        pitch = math.atan2(-r20, sy)
        yaw = 0.0
    return (roll, pitch, yaw)


def rpy_to_axes(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Columns of Rz Ry Rx.
    x_axis = (cy * cp, sy * cp, -sp)
    y_axis = (
        cy * sp * sr - sy * cr,
        sy * sp * sr + cy * cr,
        cp * sr,
    )
    z_axis = (
        cy * sp * cr + sy * sr,
        sy * sp * cr - cy * sr,
        cp * cr,
    )
    return x_axis, y_axis, z_axis


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _tag_frame(normal, viewer_right):
    z_axis = tuple(-v for v in normal)
    x_axis = viewer_right
    y_axis = _cross(z_axis, x_axis)
    got = _cross(x_axis, y_axis)
    for a, b in zip(got, z_axis):
        if abs(a - b) > 1e-9:
            raise RuntimeError("tag frame is not right-handed")
    if y_axis[2] >= 0.0:
        raise RuntimeError("printed tag up-direction is not above the bottom edge")
    if normal[2] <= 0.0:
        raise RuntimeError("tag normal does not face upward")
    return x_axis, y_axis, z_axis


def _slope_center(low, up_dir):
    half = CARD_M / 2.0
    return tuple(low[i] + half * up_dir[i] for i in range(3))


def mounts(chassis: Chassis | None = None) -> tuple:
    """Three nominal mounts. Parents are pure translations of base_link."""
    body = chassis or Chassis()
    cos_t = math.cos(TILT_RAD)
    sin_t = math.sin(TILT_RAD)
    depth = CARD_M * sin_t
    z_low = body.top_surface_z + JIG_BASE_M
    plate_x = body.length / 2.0

    front_inner_x = body.arm_front_x + FRONT_ARM_GAP_M
    front_outer_x = front_inner_x + depth
    front_low = (front_outer_x, 0.0, z_low)
    front_up = (-sin_t, 0.0, cos_t)
    front_axes = _tag_frame((cos_t, 0.0, sin_t), (0.0, 1.0, 0.0))

    rear_outer_x = -plate_x + REAR_EDGE_MARGIN_M
    rear_low = (rear_outer_x, REAR_CENTER_Y_M, z_low)
    rear_up = (sin_t, 0.0, cos_t)
    rear_axes = _tag_frame((-cos_t, 0.0, sin_t), (0.0, -1.0, 0.0))

    arm_inner_y = body.arm_side_y + ARM_SIDE_GAP_M
    arm_outer_y = arm_inner_y + depth
    arm_low = (ARM_CENTER_X_M, arm_outer_y, z_low)
    arm_up = (0.0, -sin_t, cos_t)
    arm_axes = _tag_frame((0.0, cos_t, sin_t), (-1.0, 0.0, 0.0))

    top_origin = (0.0, 0.0, body.top_plate_z)
    return (
        TagMount(
            0,
            "apriltag_front",
            "top_plate_link",
            top_origin,
            "top plate front, centred, overhanging the front lip",
            _slope_center(front_low, front_up),
            *front_axes,
        ),
        TagMount(
            1,
            "apriltag_rear",
            "top_plate_link",
            top_origin,
            "top plate back-right, low edge 5 mm in from the rear lip",
            _slope_center(rear_low, rear_up),
            *rear_axes,
        ),
        TagMount(
            2,
            "apriltag_arm_base",
            "arm_base_link",
            body.arm_origin,
            "beside the arm base, back-left, facing left and up",
            _slope_center(arm_low, arm_up),
            *arm_axes,
        ),
    )


def clearance_problems(chassis: Chassis | None = None) -> tuple:
    """Separating-plane gaps between each card and the arm body."""
    body = chassis or Chassis()
    front, rear, arm = mounts(body)
    problems = []
    front_limit = body.arm_front_x + 0.008
    for corner in front.card_corners():
        if corner[0] < front_limit:
            problems.append("front card enters the arm's forward gap")
            break
    rear_limit = body.arm_rear_x - 0.008
    for corner in rear.card_corners():
        if corner[0] > rear_limit:
            problems.append("rear card enters the arm footprint")
            break
    side_limit = body.arm_side_y + 0.008
    rear_arm_limit = body.arm_rear_x - 0.008
    for corner in arm.card_corners():
        if corner[1] < side_limit or corner[0] > rear_arm_limit:
            problems.append("arm-base card intersects the pedestal")
            break
    half_y = body.width / 2.0
    half_x = body.length / 2.0
    for mount in (rear, arm):
        for corner in mount.card_corners():
            if abs(corner[1]) > half_y - 0.002:
                problems.append(mount.name + " card leaves the top plate in Y")
                break
    for corner in rear.card_corners():
        if corner[0] < -half_x + 0.002:
            problems.append("rear card leaves the rear edge")
            break
    return tuple(problems)


def render_tag(code: int) -> tuple:
    """10x10 grid. '#' is black, '.' is white. Row 0 is the top of the tag."""
    image = [["."] * 10 for _ in range(10)]
    for y in range(1, 9):
        for x in range(1, 9):
            image[y][x] = "#"
    border = 1
    for index in range(36):
        bit = (code >> (35 - index)) & 1
        x = BIT_X[index] + border
        y = BIT_Y[index] + border
        if bit:
            image[y][x] = "."
    return tuple("".join(row) for row in image)


def write_png(path: Path, grid: tuple, scale: int = 40) -> None:
    width = 10 * scale
    height = 10 * scale
    raw = bytearray()
    for row in grid:
        expanded = bytearray()
        for cell in row:
            value = 0 if cell == "#" else 255
            expanded.extend((value, value, value) * scale)
        for _ in range(scale):
            raw.append(0)
            raw.extend(expanded)

    def chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(bytes(raw), 9))
        + chunk(b"IEND", b"")
    )
    path.write_bytes(png)


def write_tag_mesh(directory: Path, tag_id: int, grid: tuple) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    stem = f"tag36h11_id{tag_id}"
    write_png(directory / f"{stem}.png", grid, scale=40)
    mtl = (
        f"newmtl {stem}\n"
        "Ka 1.000 1.000 1.000\n"
        "Kd 1.000 1.000 1.000\n"
        "Ks 0.000 0.000 0.000\n"
        "illum 1\n"
        f"map_Kd {stem}.png\n"
        "\n"
        "newmtl apriltag_backing\n"
        "Ka 0.050 0.050 0.050\n"
        "Kd 0.050 0.050 0.050\n"
        "Ks 0.000 0.000 0.000\n"
        "illum 1\n"
    )
    (directory / f"{stem}.mtl").write_text(mtl, encoding="utf-8", newline="\n")
    half = CARD_M / 2.0
    thick = MESH_THICKNESS_M
    verts = [
        (-half, -half, 0.0),
        (-half, half, 0.0),
        (half, half, 0.0),
        (half, -half, 0.0),
        (-half, -half, thick),
        (-half, half, thick),
        (half, half, thick),
        (half, -half, thick),
    ]
    uvs = ((0.0, 1.0), (0.0, 0.0), (1.0, 0.0), (1.0, 1.0))
    # Printed face at z=0, normal -Z, image up toward -Y. Backing is matte.
    printed = ((1, 2, 3), (1, 3, 4))
    backing = (
        (5, 8, 7),
        (5, 7, 6),
        (4, 3, 7),
        (4, 7, 8),
        (1, 5, 6),
        (1, 6, 2),
        (1, 4, 8),
        (1, 8, 5),
        (2, 6, 7),
        (2, 7, 3),
    )
    lines = [f"mtllib {stem}.mtl", f"usemtl {stem}"]
    for vert in verts:
        lines.append(f"v {vert[0]:.6f} {vert[1]:.6f} {vert[2]:.6f}")
    for uv in uvs:
        lines.append(f"vt {uv[0]:.6f} {uv[1]:.6f}")
    for face in printed:
        lines.append(
            "f {0}/{0} {1}/{1} {2}/{2}".format(face[0], face[1], face[2])
        )
    lines.append("usemtl apriltag_backing")
    for face in backing:
        lines.append("f {0} {1} {2}".format(face[0], face[1], face[2]))
    lines.append("")
    (directory / f"{stem}.obj").write_text(
        "\n".join(lines), encoding="utf-8", newline="\n"
    )


def jig_profile_mm() -> tuple:
    depth = CARD_M * math.sin(TILT_RAD) * 1000.0
    rise = CARD_M * math.cos(TILT_RAD) * 1000.0
    base = JIG_BASE_M * 1000.0
    return (
        (0.0, 0.0),
        (depth, 0.0),
        (depth, base + rise),
        (0.0, base),
    )


def jig_size_mm() -> tuple:
    profile = jig_profile_mm()
    depth = max(point[0] for point in profile)
    height = max(point[1] for point in profile)
    return (depth, CARD_M * 1000.0, height)


def _stl_triangles():
    profile = jig_profile_mm()
    width = CARD_M * 1000.0

    def vert(index, y):
        x, z = profile[index]
        return (x, y, z)

    tris = [
        (vert(0, width), vert(1, width), vert(2, width)),
        (vert(0, width), vert(2, width), vert(3, width)),
        (vert(0, 0.0), vert(2, 0.0), vert(1, 0.0)),
        (vert(0, 0.0), vert(3, 0.0), vert(2, 0.0)),
    ]
    count = len(profile)
    for index in range(count):
        nxt = (index + 1) % count
        a = vert(index, 0.0)
        b = vert(nxt, 0.0)
        c = vert(nxt, width)
        d = vert(index, width)
        tris.append((a, b, c))
        tris.append((a, c, d))
    if _mesh_volume(tris) < 0.0:
        tris = tuple((a, c, b) for a, b, c in tris)
    return tuple(tris)


def _mesh_volume(tris) -> float:
    total = 0.0
    for a, b, c in tris:
        total += (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        )
    return total / 6.0


def _edge_key(p, q):
    a = tuple(round(v, 6) for v in p)
    b = tuple(round(v, 6) for v in q)
    return (a, b) if a <= b else (b, a)


def mesh_is_closed(tris) -> bool:
    counts = {}
    for a, b, c in tris:
        for edge in ((a, b), (b, c), (c, a)):
            key = _edge_key(*edge)
            counts[key] = counts.get(key, 0) + 1
    return all(count == 2 for count in counts.values())


def write_stl(path: Path) -> None:
    tris = _stl_triangles()
    if not mesh_is_closed(tris) or _mesh_volume(tris) <= 0.0:
        raise RuntimeError("jig mesh is not a closed positive volume")
    lines = ["solid apriltag_mount_jig_mm"]
    for a, b, c in tris:
        ux, uy, uz = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        vx, vy, vz = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        norm = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        lines.append(f"  facet normal {nx / norm:.6e} {ny / norm:.6e} {nz / norm:.6e}")
        lines.append("    outer loop")
        for point in (a, b, c):
            lines.append(
                f"      vertex {point[0]:.6f} {point[1]:.6f} {point[2]:.6f}"
            )
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append("endsolid apriltag_mount_jig_mm")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def write_scad(path: Path) -> None:
    depth, width, _height = jig_size_mm()
    base = JIG_BASE_M * 1000.0
    rise = CARD_M * math.cos(TILT_RAD) * 1000.0
    text = f"""// Nominal OmniBot AprilTag jig. Units: millimeters.
// Generated by generate_printables.py. Low end (x = 0) points outward.
// Tall end points inward. Underside (z = 0) tapes to the top plate.
// The 100 mm card covers the slope; its top edge is the tall end.
base_t = {base:.6f};
depth = {depth:.6f};
rise = {rise:.6f};
width = {width:.6f};
rotate([90, 0, 0])
translate([0, 0, -width])
linear_extrude(height = width)
  polygon(points = [
    [0, 0],
    [depth, 0],
    [depth, base_t + rise],
    [0, base_t]
  ]);
"""
    path.write_text(text, encoding="utf-8", newline="\n")


def _pdf_escape(text: str) -> str:
    return text.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")


def _pdf_bytes(page_streams: list) -> bytes:
    objects = []
    objects.append(b"<< /Type /Catalog /Pages 2 0 R >>")
    objects.append(b"")  # pages, filled after ids are known
    objects.append(b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>")
    page_ids = []
    next_id = 4
    for stream in page_streams:
        data = stream.encode("ascii")
        content_id = next_id
        page_id = next_id + 1
        next_id += 2
        objects.append(
            b"<< /Length "
            + str(len(data)).encode("ascii")
            + b" >>\nstream\n"
            + data
            + b"\nendstream"
        )
        objects.append(
            (
                f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {A4_W_PT:.2f} {A4_H_PT:.2f}] "
                f"/Contents {content_id} 0 R /Resources << /Font << /F1 3 0 R >> >> >>"
            ).encode("ascii")
        )
        page_ids.append(page_id)
    kids = " ".join(f"{page_id} 0 R" for page_id in page_ids)
    objects[1] = (
        f"<< /Type /Pages /Count {len(page_ids)} /Kids [{kids}] >>".encode("ascii")
    )
    out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
    offsets = [0]
    for index, body in enumerate(objects, start=1):
        offsets.append(len(out))
        out.extend(f"{index} 0 obj\n".encode("ascii"))
        out.extend(body)
        out.extend(b"\nendobj\n")
    xref = len(out)
    size = len(objects) + 1
    out.extend(f"xref\n0 {size}\n".encode("ascii"))
    out.extend(b"0000000000 65535 f \n")
    for offset in offsets[1:]:
        out.extend(f"{offset:010d} 00000 n \n".encode("ascii"))
    out.extend(
        f"trailer\n<< /Size {size} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n".encode(
            "ascii"
        )
    )
    return bytes(out)


def _mm(value_mm: float) -> float:
    return value_mm * MM_TO_PT


def _tag_page(mount: TagMount, grid: tuple) -> str:
    module = _mm(10.0)
    card = 10.0 * module
    left = (A4_W_PT - card) / 2.0
    top = _mm(250.0)
    parts = [
        "0.8 w\n",
        _pdf_text(
            _mm(15),
            _mm(275),
            16,
            f"OmniBot {FAMILY} id {mount.tag_id} - {mount.location}",
        ),
        _pdf_text(
            _mm(15),
            top + _mm(6),
            11,
            "TOP of this page is the top of the tag. Mount that edge at the tall end of the jig.",
        ),
    ]
    for row_index, row in enumerate(grid):
        for col_index, cell in enumerate(row):
            if cell != "#":
                continue
            x = left + col_index * module
            y = top - (row_index + 1) * module
            parts.append(f"{x:.2f} {y:.2f} {module:.2f} {module:.2f} re f\n")
    bar_w = _mm(100.0)
    bar_h = _mm(4.0)
    bar_x = (A4_W_PT - bar_w) / 2.0
    bar_y = _mm(28.0)
    parts.append(f"{bar_x:.2f} {bar_y:.2f} {bar_w:.2f} {bar_h:.2f} re f\n")
    parts.append(
        _pdf_text(
            bar_x,
            bar_y + bar_h + _mm(3),
            11,
            "100 mm scale bar. Print at 100% (actual size) on matte paper. Do not scale to fit.",
        )
    )
    parts.append(
        _pdf_text(
            _mm(15),
            _mm(18),
            10,
            "Black square must measure 80 mm. Descriptor size is that length in metres. nominal, to be measured.",
        )
    )
    return "".join(parts)


def _pdf_text(x, y, size, text) -> str:
    return (
        f"BT /F1 {size:.1f} Tf {x:.2f} {y:.2f} Td ({_pdf_escape(text)}) Tj ET\n"
    )


def _instruction_page() -> str:
    depth, width, height = jig_size_mm()
    parts = [
        "0.9 w\n",
        _pdf_text(_mm(15), _mm(275), 18, "OmniBot AprilTag sheet - tag36h11"),
        _pdf_text(
            _mm(15),
            _mm(262),
            11,
            "IDs 0 front, 1 back-right, 2 arm base. One tag per following page. Matte paper, actual size.",
        ),
        _pdf_text(
            _mm(15),
            _mm(248),
            11,
            "OHH-213 detects this family on the Quest passthrough camera. Do not use a monitor.",
        ),
    ]
    bar_w = _mm(100.0)
    bar_y = _mm(228.0)
    parts.append(f"{_mm(15):.2f} {bar_y:.2f} {bar_w:.2f} {_mm(4):.2f} re f\n")
    parts.append(
        _pdf_text(
            _mm(15),
            bar_y + _mm(7),
            11,
            "100 mm scale bar. If this bar is not 100 mm on paper, do not mount the tags.",
        )
    )
    # 1:1 side view. x=0 at the left of the drawing, z up. Low end on the left.
    origin_x = _mm(20.0)
    origin_y = _mm(40.0)
    profile = jig_profile_mm()
    parts.append("1.2 w\n")
    for index, point in enumerate(profile):
        nxt = profile[(index + 1) % len(profile)]
        x1 = origin_x + _mm(point[0])
        y1 = origin_y + _mm(point[1])
        x2 = origin_x + _mm(nxt[0])
        y2 = origin_y + _mm(nxt[1])
        parts.append(f"{x1:.2f} {y1:.2f} m {x2:.2f} {y2:.2f} l S\n")
    parts.append(
        _pdf_text(
            origin_x,
            origin_y + _mm(height + 8),
            11,
            f"Jig side view, 1:1. {width:.0f} mm wide, {depth:.1f} mm deep, {height:.1f} mm tall.",
        )
    )
    parts.append(
        _pdf_text(
            origin_x,
            origin_y - _mm(8),
            11,
            "Left is the low end (outward). Right is the tall end (inward). Bottom tapes to the plate.",
        )
    )
    return "".join(parts)


def build_pdf() -> bytes:
    pages = [_instruction_page()]
    for mount in mounts():
        pages.append(_tag_page(mount, render_tag(TAG36H11_CODES[mount.tag_id])))
    return _pdf_bytes(pages)


def _fmt(value: float) -> str:
    if abs(value) < 5e-12:
        value = 0.0
    return f"{value:.9f}"


def render_xacro() -> str:
    lines = [
        '<?xml version="1.0"?>',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="omnibot_apriltags">',
        "  <!-- Nominal tag36h11 planes for the chase cam. CAD poses, not measured.",
        "       docs/apriltags.md. No cameras and no sensors in this file. -->",
    ]
    for mount in mounts():
        parent_xyz = " ".join(_fmt(v) for v in mount.origin_parent())
        rpy = " ".join(_fmt(v) for v in mount.rpy())
        mesh = (
            "package://omnibot_description/meshes/apriltags/"
            f"tag36h11_id{mount.tag_id}.obj"
        )
        lines.extend(
            [
                f'  <link name="{mount.link}">',
                "    <visual>",
                '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                "      <geometry>",
                f'        <mesh filename="{mesh}"/>',
                "      </geometry>",
                f'      <material name="{mount.name}_matte">',
                '        <color rgba="1 1 1 1"/>',
                "      </material>",
                "    </visual>",
                "    <collision>",
                f'      <origin xyz="0 0 {MESH_THICKNESS_M / 2.0:.6f}" rpy="0 0 0"/>',
                "      <geometry>",
                f'        <box size="{CARD_M:.3f} {CARD_M:.3f} {MESH_THICKNESS_M:.6f}"/>',
                "      </geometry>",
                "    </collision>",
                "    <inertial>",
                '      <mass value="0.004"/>',
                '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                '      <inertia ixx="3.4e-6" ixy="0" ixz="0" iyy="3.4e-6" iyz="0" izz="6.7e-6"/>',
                "    </inertial>",
                "  </link>",
                f'  <joint name="{mount.joint}" type="fixed">',
                f'    <parent link="{mount.parent}"/>',
                f'    <child link="{mount.link}"/>',
                f'    <origin xyz="{parent_xyz}" rpy="{rpy}"/>',
                "  </joint>",
            ]
        )
    lines.append("</robot>")
    lines.append("")
    return "\n".join(lines)


def documentation_rows() -> tuple:
    """Markdown table rows. docs/apriltags.md must contain each one."""
    rows = []
    for mount in mounts():
        xyz = " ".join(f"{v:.6f}" for v in mount.origin_base)
        rows.append(
            f"| {mount.tag_id} | {mount.link} | {mount.location} | {TAG_SIZE_M:.3f} | {xyz} |"
        )
    return tuple(rows)


def generate_all(description_dir: Path) -> None:
    problems = clearance_problems()
    if problems:
        raise RuntimeError("; ".join(problems))
    for mount in mounts():
        recovered = rpy_to_axes(*mount.rpy())
        for got, want in zip(recovered, (mount.x_axis, mount.y_axis, mount.z_axis)):
            for a, b in zip(got, want):
                if abs(a - b) > 1e-9:
                    raise RuntimeError("rpy round-trip failed for " + mount.name)
    print_dir = description_dir / "apriltags" / "print"
    print_dir.mkdir(parents=True, exist_ok=True)
    (print_dir / "omnibot_tag36h11.pdf").write_bytes(build_pdf())
    write_stl(print_dir / "apriltag_mount_jig.stl")
    write_scad(print_dir / "apriltag_mount_jig.scad")
    mesh_dir = description_dir / "meshes" / "apriltags"
    for mount in mounts():
        write_tag_mesh(mesh_dir, mount.tag_id, render_tag(TAG36H11_CODES[mount.tag_id]))
    xacro_path = description_dir / "urdf" / "apriltags.urdf.xacro"
    xacro_path.write_text(render_xacro(), encoding="utf-8", newline="\n")
