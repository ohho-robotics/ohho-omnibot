# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Load the shared VR topic contract. No ROS imports.

sim_vr.launch.py and robot_vr.launch.py both load this module by path.
The loader rejects a contract that would move a sim topic, widen the
default bind, or document a looser safety ceiling.
"""

from __future__ import annotations

import os

try:
    import yaml
except ImportError as exc:  # pragma: no cover - CI installs PyYAML
    raise RuntimeError("vr_topic_contract.yaml requires PyYAML") from exc


CONTRACT_FILENAME = "vr_topic_contract.yaml"

REQUIRED_PORTS = {
    "rosbridge": 9090,
    "web_video": 8080,
    "foxglove": 8765,
    "bind": "127.0.0.1",
}

REQUIRED_VIDEO = {
    "front": "/camera/front/image_raw",
    "wrist": "/camera/wrist/image_raw",
    "bev": "/camera/base/bev/image_raw",
}

REQUIRED_TELEOP_PATHS = (
    {
        "raw": "/cmd_vel/teleop_raw",
        "gated": "/cmd_vel/teleop",
        "mux": "cmd_vel_mux",
        "driver_output": "/cmd_vel/out",
        "driver_remap_from": "cmd_vel",
    },
    {
        "raw": "/arm/joint_commands/teleop_raw",
        "gated": "/arm/joint_commands/teleop",
        "mux": "arm_cmd_mux",
        "driver_output": "/arm/joint_commands/out",
        "driver_remap_from": "joint_commands",
    },
)

REQUIRED_TOPIC_NAMES = (
    "/cmd_vel/teleop_raw",
    "/arm/joint_commands/teleop_raw",
    "/ohho/heartbeat",
    "/emergency_stop",
    "/ohho/estop_reset",
    "/ohho/safety/state",
    "/cmd_vel/teleop",
    "/arm/joint_commands/teleop",
    "/cmd_vel",
    "/cmd_vel/vla",
    "/cmd_vel/rl",
    "/cmd_vel/out",
    "/control_mode",
    "/control_mode/active",
    "/arm/joint_commands/policy",
    "/arm/joint_commands",
    "/arm/joint_commands/rl",
    "/arm/joint_commands/out",
    "/arm/cmd_mode",
    "/arm/cmd_mode/active",
    "/odom",
    "/scan",
    "/imu",
    "/imu/data",
    "/camera/front/image_raw",
    "/camera/front/camera_info",
    "/camera/wrist/image_raw",
    "/camera/wrist/camera_info",
    "/camera/rear/image_raw",
    "/camera/left/image_raw",
    "/camera/right/image_raw",
    "/camera/base/bev/image_raw",
    "/tf",
    "/ohho/telemetry",
    "/ohho/descriptor",
)

ALLOWED_SAFETY_KEYS = {
    "default_control_mode",
    "use_sim_time",
    "heartbeat_timeout_ms",
}

# A contract edit must not grow a knob that raises a ceiling.
FORBIDDEN_SAFETY_KEYS = {
    "max_linear",
    "max_linear_vel",
    "max_angular",
    "max_angular_vel",
    "max_speed",
    "accel",
    "acceleration",
    "max_joint_delta_rad",
    "source_timeout_s",
    "watchdog_timeout",
    "command_timeout_sec",
    "heartbeat_timeout_ms_override",
}


def default_contract_path(anchor_file):
    """config/vr_topic_contract.yaml next to the launch directory."""
    launch_dir = os.path.dirname(os.path.abspath(anchor_file))
    return os.path.join(os.path.dirname(launch_dir), "config", CONTRACT_FILENAME)


def _require_mapping(value, label):
    if not isinstance(value, dict):
        raise RuntimeError(f"VR contract {label} must be a mapping")
    return value


def _teleop_path(raw):
    if not isinstance(raw, dict):
        raise RuntimeError("VR contract teleop_paths entries must be mappings")
    keys = (
        "raw",
        "gated",
        "mux",
        "driver_output",
        "driver_remap_from",
    )
    return {key: raw[key] for key in keys}


def load_vr_contract(path):
    """Return the normalized contract or raise if it would change safety."""
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    data = _require_mapping(data, "root")

    safety = _require_mapping(data.get("safety"), "safety")
    unknown = set(safety) - ALLOWED_SAFETY_KEYS
    forbidden = set(safety) & FORBIDDEN_SAFETY_KEYS
    if unknown or forbidden:
        raise RuntimeError(
            "VR contract safety keys cannot add a cap or timeout: "
            + ", ".join(sorted(unknown | forbidden))
        )
    if safety.get("default_control_mode") != "teleop":
        raise RuntimeError("VR contract default_control_mode must stay teleop")
    if safety.get("use_sim_time") is not False:
        raise RuntimeError("VR contract use_sim_time must stay false")
    if safety.get("heartbeat_timeout_ms") != 300:
        raise RuntimeError("VR contract heartbeat timeout must stay 300 ms")

    ports = _require_mapping(data.get("ports"), "ports")
    for key, expected in REQUIRED_PORTS.items():
        if ports.get(key) != expected:
            raise RuntimeError(
                f"VR contract ports.{key} must stay {expected}, got {ports.get(key)!r}"
            )

    if data.get("telemetry_hz") != 20:
        raise RuntimeError("VR contract telemetry_hz must stay 20")

    streams = data.get("video_streams")
    if not isinstance(streams, list) or len(streams) != len(REQUIRED_VIDEO):
        raise RuntimeError("VR contract video_streams must list front, wrist, and bev")
    seen = {}
    for item in streams:
        item = _require_mapping(item, "video stream")
        seen[item.get("id")] = item.get("topic")
    if seen != REQUIRED_VIDEO:
        raise RuntimeError(
            "VR contract video streams must stay front, wrist, and bev"
        )

    raw_paths = data.get("teleop_paths")
    if not isinstance(raw_paths, list):
        raise RuntimeError("VR contract teleop_paths must be a list")
    paths = [_teleop_path(item) for item in raw_paths]
    if paths != [dict(item) for item in REQUIRED_TELEOP_PATHS]:
        raise RuntimeError("VR contract teleop paths must stay on teleop_safety")

    raw_topics = data.get("topics")
    if not isinstance(raw_topics, list):
        raise RuntimeError("VR contract topics must be a list")
    topics = {}
    for item in raw_topics:
        item = _require_mapping(item, "topic")
        name = item.get("name")
        if not isinstance(name, str) or not name.startswith("/"):
            raise RuntimeError(f"VR contract topic name is invalid: {name!r}")
        if name in topics:
            raise RuntimeError(f"VR contract topic repeated: {name}")
        if not item.get("type"):
            raise RuntimeError(f"VR contract topic {name} is missing a type")
        topics[name] = {
            "name": name,
            "type": item["type"],
            "publisher": item.get("publisher", ""),
            "consumer": item.get("consumer", ""),
        }
    missing = [name for name in REQUIRED_TOPIC_NAMES if name not in topics]
    if missing:
        raise RuntimeError("VR contract is missing topics: " + ", ".join(missing))

    return {
        "version": data.get("version"),
        "safety": {
            "default_control_mode": "teleop",
            "use_sim_time": False,
            "heartbeat_timeout_ms": 300,
        },
        "ports": dict(REQUIRED_PORTS),
        "telemetry_hz": 20,
        "video_streams": [
            {"id": item["id"], "topic": item["topic"]}
            for item in streams
        ],
        "teleop_paths": paths,
        "topics": topics,
        "path": path,
    }


def video_topics(contract):
    return tuple(item["topic"] for item in contract["video_streams"])


def teleop_path(contract, mux):
    for item in contract["teleop_paths"]:
        if item["mux"] == mux:
            return item
    raise RuntimeError(f"VR contract has no teleop path for {mux}")
