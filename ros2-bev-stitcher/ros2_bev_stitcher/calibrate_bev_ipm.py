#!/usr/bin/env python3
"""
Headless IPM calibration for OhhO View (no ROS required).

Writes a ``.npz`` homography file for ``bev_stitcher`` from default or
CLI-supplied camera poses. Use on a workstation before deploying to the Pi,
or on-robot after fine-tuning poses with ``bev_cal_ui``.

Example::

    python -m ros2_bev_stitcher.calibrate_bev_ipm \\
        --output /tmp/bev_calibration.npz

See ``docs/CALIBRATION.md`` for the full real-robot workflow.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List

from ros2_bev_stitcher.bev_ipm import (
    compute_ipm_homographies_for_cameras,
    save_homography_npz,
)


def _parse_name_value_triplets(
    entries: List[str] | None,
) -> Dict[str, List[float]]:
    """Parse ``name:a:b:c`` CLI tokens into a dict of float triples."""
    out: Dict[str, List[float]] = {}
    if not entries:
        return out
    for entry in entries:
        parts = entry.split(":")
        if len(parts) != 4:
            raise ValueError(
                f"Expected name:x:y:z or name:r:p:y, got '{entry}'"
            )
        name, a, b, c = parts
        out[name] = [float(a), float(b), float(c)]
    return out


def run_calibration(
    camera_names: List[str],
    output: str,
    canvas_size: int,
    pixels_per_meter: float,
    ground_z: float,
    positions: Dict[str, List[float]],
    orientations: Dict[str, List[float]],
    focal_length: float,
    principal_point: List[float],
) -> str:
    per_camera: Dict[str, Dict[str, List[float]]] = {}
    for name in camera_names:
        cfg: Dict[str, List[float]] = {}
        if name in positions:
            cfg["position"] = positions[name]
        if name in orientations:
            cfg["orientation_rpy"] = orientations[name]
        cfg["fx"] = focal_length
        cfg["fy"] = focal_length
        cfg["cx"] = principal_point[0]
        cfg["cy"] = principal_point[1]
        if cfg:
            per_camera[name] = cfg

    homographies = compute_ipm_homographies_for_cameras(
        camera_names,
        canvas_size=canvas_size,
        pixels_per_meter=pixels_per_meter,
        ground_z=ground_z,
        per_camera=per_camera,
    )
    return save_homography_npz(output, homographies)


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Write BEV IPM homographies to a .npz file (no ROS)"
    )
    parser.add_argument(
        "--camera-names",
        nargs="+",
        default=["front", "rear", "left", "right"],
        help="Camera names (default: front rear left right)",
    )
    parser.add_argument("--canvas-size", type=int, default=800)
    parser.add_argument("--pixels-per-meter", type=float, default=80.0)
    parser.add_argument("--ground-z", type=float, default=0.0)
    parser.add_argument(
        "--output",
        default=os.path.expanduser("~/bev_calibration.npz"),
        help="Output .npz path",
    )
    parser.add_argument(
        "--positions",
        nargs="*",
        default=None,
        help="Per-camera position overrides: name:x:y:z ...",
    )
    parser.add_argument(
        "--orientations",
        nargs="*",
        default=None,
        help="Per-camera RPY overrides (yaw pitch roll rad): name:y:p:r ...",
    )
    parser.add_argument("--focal-length", type=float, default=320.0)
    parser.add_argument(
        "--principal-point",
        nargs=2,
        type=float,
        default=[320.0, 240.0],
        metavar=("CX", "CY"),
    )
    parser.add_argument(
        "--summary-json",
        action="store_true",
        help="Print a one-line JSON summary on stdout (for CI logs)",
    )
    args = parser.parse_args(argv)

    try:
        positions = _parse_name_value_triplets(args.positions)
        orientations = _parse_name_value_triplets(args.orientations)
    except ValueError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    out_path = run_calibration(
        camera_names=list(args.camera_names),
        output=args.output,
        canvas_size=args.canvas_size,
        pixels_per_meter=args.pixels_per_meter,
        ground_z=args.ground_z,
        positions=positions,
        orientations=orientations,
        focal_length=args.focal_length,
        principal_point=list(args.principal_point),
    )

    if args.summary_json:
        summary = {
            "cameras": args.camera_names,
            "output": out_path,
            "canvas_size": args.canvas_size,
            "pixels_per_meter": args.pixels_per_meter,
        }
        print(json.dumps(summary))
    else:
        print(f"Wrote {len(args.camera_names)} homographies → {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
