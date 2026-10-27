# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Assemble an ohho.descriptor.v1 document from the OmniBot YAML. No ROS imports.

Hard caps must stay at or below the OmniBot ceilings. This module refuses a
document that raises them. It does not change the Yahboom 0.12 m/s clip.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

MAX_VX_MPS = 0.2
MAX_VY_MPS = 0.2
MAX_WZ_RAD_S = 1.0
MAX_ACCEL_X_MPS2 = 0.5
MAX_ACCEL_Y_MPS2 = 0.5
MAX_ACCEL_WZ_RAD_S2 = 2.0
MAX_JOINT_STEP_RAD = 0.15

_HARD_CEILINGS = {
    "max_vx_mps": MAX_VX_MPS,
    "max_vy_mps": MAX_VY_MPS,
    "max_wz_rad_s": MAX_WZ_RAD_S,
    "max_accel_x_mps2": MAX_ACCEL_X_MPS2,
    "max_accel_y_mps2": MAX_ACCEL_Y_MPS2,
    "max_accel_wz_rad_s2": MAX_ACCEL_WZ_RAD_S2,
    "max_joint_step_rad": MAX_JOINT_STEP_RAD,
}
_DRIVE_CEILINGS = {
    "vx_mps": MAX_VX_MPS,
    "vy_mps": MAX_VY_MPS,
    "wz_rad_s": MAX_WZ_RAD_S,
}
_REQUIRED = (
    "v",
    "id",
    "model",
    "drive",
    "arms",
    "topics",
    "cameras",
    "urdf_url",
    "apriltags",
    "telemetry",
    "safety",
)


def load_descriptor_yaml(path, kind=None):
    """Read YAML and return a validated descriptor dict."""
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - CI installs pyyaml
        raise RuntimeError("robot_descriptor.yaml requires PyYAML") from exc
    text = Path(path).read_text(encoding="utf-8")
    data = yaml.safe_load(text)
    return assemble_descriptor(data, kind=kind)


def assemble_descriptor(data, kind=None):
    """Validate ``data`` and optionally override telemetry.kind (sim or real)."""
    if not isinstance(data, dict):
        raise ValueError("descriptor must be an object")
    missing = [key for key in _REQUIRED if key not in data]
    if missing:
        raise ValueError(f"descriptor missing {missing}")
    document = json.loads(json.dumps(data))
    if kind is not None:
        if kind not in ("sim", "real"):
            raise ValueError("kind must be sim or real")
        telemetry = document.get("telemetry")
        if not isinstance(telemetry, dict):
            raise ValueError("telemetry must be an object")
        telemetry["kind"] = kind
    _validate(document)
    return document


def dumps_descriptor(document):
    """Stable JSON for the latched /ohho/descriptor message."""
    return json.dumps(document, separators=(",", ":"), sort_keys=True)


def _validate_tags(tags):
    """Same apriltags contract as ohho-sdk RobotDescriptor v1."""
    if not isinstance(tags, list):
        raise ValueError("apriltags must be a list")
    allowed = {"id", "size", "T_tag_base"}
    for index, tag in enumerate(tags):
        where = f"apriltags[{index}]"
        if not isinstance(tag, dict):
            raise ValueError(f"{where} must be an object")
        missing = sorted(allowed.difference(tag))
        extra = sorted(set(tag).difference(allowed))
        if missing:
            raise ValueError(f"{where} missing {missing}")
        if extra:
            raise ValueError(f"{where} unknown keys {extra}")
        tag_id = tag["id"]
        if isinstance(tag_id, bool) or not isinstance(tag_id, int) or tag_id < 0:
            raise ValueError(f"{where}.id must be an integer >= 0")
        _tag_size(f"{where}.size", tag["size"])
        matrix = tag["T_tag_base"]
        if not isinstance(matrix, list) or len(matrix) != 16:
            raise ValueError(f"{where}.T_tag_base must have 16 numbers")
        for mndex, value in enumerate(matrix):
            _finite_number(f"{where}.T_tag_base[{mndex}]", value)


def _finite_number(name, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _tag_size(name, value):
    number = _finite_number(name, value)
    if number <= 0.0:
        raise ValueError(f"{name} must be greater than 0.0")
    if number > 1.0:
        raise ValueError(f"{name} exceeds the hard ceiling 1.0")
    return number


def _validate(document):
    hard = document["safety"]["hard"]
    for key, ceiling in _HARD_CEILINGS.items():
        _cap(f"safety.hard.{key}", hard[key], ceiling)
    caps = document["drive"]["caps"]
    for key, ceiling in _DRIVE_CEILINGS.items():
        _cap(f"drive.caps.{key}", caps[key], ceiling)
    if caps["vx_mps"] > hard["max_vx_mps"]:
        raise ValueError("drive.caps.vx_mps exceeds safety.hard.max_vx_mps")
    if caps["vy_mps"] > hard["max_vy_mps"]:
        raise ValueError("drive.caps.vy_mps exceeds safety.hard.max_vy_mps")
    if caps["wz_rad_s"] > hard["max_wz_rad_s"]:
        raise ValueError("drive.caps.wz_rad_s exceeds safety.hard.max_wz_rad_s")
    if document["telemetry"]["robot_id"] != document["id"]:
        raise ValueError("telemetry.robot_id must match id")
    if document["telemetry"]["kind"] not in ("sim", "real"):
        raise ValueError("telemetry.kind is invalid")
    _validate_tags(document["apriltags"])
    image_topics = set(document["topics"]["images"])
    for camera in document["cameras"]:
        if camera["topic"] not in image_topics:
            raise ValueError(f"camera topic {camera['topic']} is not in topics.images")
    for arm in document["arms"]:
        for joint in arm["joints"]:
            lo = float(joint["min"])
            hi = float(joint["max"])
            home = float(joint["home"])
            if not (math.isfinite(lo) and math.isfinite(hi) and math.isfinite(home)):
                raise ValueError(f"joint {joint['name']} limits must be finite")
            if lo > hi:
                raise ValueError(f"joint {joint['name']} min is above max")
            if home < lo or home > hi:
                raise ValueError(f"joint {joint['name']} home is outside min/max")


def _cap(name, value, ceiling):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise ValueError(f"{name} must be a finite number >= 0")
    if number > ceiling:
        raise ValueError(f"{name} exceeds the hard ceiling {ceiling}")
    return number
