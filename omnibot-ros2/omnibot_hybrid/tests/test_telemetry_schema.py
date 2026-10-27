"""Golden vectors from ohho-sdk ohh-201-telemetry-schema, plus assembled samples."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from jsonschema.exceptions import ValidationError

from omnibot_hybrid.telemetry_snapshot import (
    assemble,
    echo_heartbeat,
    parse_heartbeat,
    schema_body,
)
from schema_tools import FIXTURE_DIR, assert_valid_telemetry, load_validators

VECTORS = FIXTURE_DIR / "vectors"
SOURCE = FIXTURE_DIR / "SOURCE.txt"
COMMIT = "ae1909439092bceac667553ea7f5cfeaf506c4f6"


def _vectors(kind: str, bucket: str) -> list[Path]:
    paths = sorted((VECTORS / kind / bucket).glob("*.json"))
    assert paths, f"no {bucket} vectors for {kind}"
    return paths


def test_fixture_source_commit():
    text = SOURCE.read_text(encoding="utf-8")
    assert COMMIT in text
    assert "ohh-201-telemetry-schema" in text
    assert "ohho-sdk" in text


@pytest.mark.parametrize(
    "kind,schema_name",
    [
        ("telemetry", "telemetry.json"),
        ("safety_state", "safety_state.json"),
        ("heartbeat", "heartbeat.json"),
        ("heartbeat_echo", "heartbeat_echo.json"),
    ],
)
def test_golden_vectors(kind, schema_name):
    validator = load_validators()[schema_name]
    for path in _vectors(kind, "valid"):
        validator.validate(json.loads(path.read_text(encoding="utf-8")))
    for path in _vectors(kind, "invalid"):
        instance = json.loads(path.read_text(encoding="utf-8"))
        with pytest.raises(ValidationError):
            validator.validate(instance)


def test_echo_matches_golden_typical():
    heartbeat = json.loads(
        (VECTORS / "heartbeat" / "valid" / "typical.json").read_text(encoding="utf-8")
    )
    echo = echo_heartbeat(heartbeat, 1735689601.5)
    expected = json.loads(
        (VECTORS / "heartbeat_echo" / "valid" / "typical.json").read_text(encoding="utf-8")
    )
    assert echo == expected
    load_validators()["heartbeat_echo.json"].validate(echo)


def test_invalid_heartbeats_are_not_echoed():
    for path in _vectors("heartbeat", "invalid"):
        assert parse_heartbeat(json.loads(path.read_text(encoding="utf-8"))) is None


def test_assembled_full_sample_matches_v1_after_stripping_age():
    now = 1735689600.5
    limits = {"arm_joint_1": (-1.57, 1.57)}
    sources = {
        "odom": {
            "t": now - 0.1,
            "data": {
                "frame": "odom",
                "x": 1.5,
                "y": -0.25,
                "yaw": 0.785398163397,
                "vx": 0.18,
                "vy": 0.0,
                "wz": 0.09,
            },
        },
        "cmd_vel": {"t": now - 0.05, "data": {"vx": 0.2, "vy": 0.0, "wz": 0.1}},
        "joint_state": {
            "t": now - 0.02,
            "data": {
                "names": ["arm_joint_1"],
                "position": [0.1],
                "velocity": [0.0],
                "effort": [0.2],
            },
        },
        "joint_cmd": {
            "t": now - 0.02,
            "data": {"names": ["arm_joint_1"], "position": [0.1]},
        },
        "imu": {
            "t": now - 0.01,
            "data": {
                "roll": 0.01,
                "pitch": -0.02,
                "yaw": 0.78,
                "ax": 0.1,
                "ay": -0.05,
                "az": 9.81,
            },
        },
        "battery": {"t": now, "data": {"v": 24.2, "pct": 80}},
        "control_mode": {"t": now, "data": "teleop"},
        "arm_mode": {"t": now, "data": "hold"},
        "safety": {
            "t": now,
            "data": {"armed": True, "latched": False, "reason": ""},
        },
        "nav_status": {"t": now, "data": 2},
        "nav_feedback": {
            "t": now,
            "data": {"eta_s": 12.5, "goal": {"x": 2.0, "y": 1.0, "yaw": 0.0}},
        },
        "policy": {
            "t": now,
            "data": {"name": "pick-place", "state": "running", "latency_ms": 8},
        },
        "recorder": {
            "t": now,
            "data": {"active": True, "episode": "ep-12", "frames": 340},
        },
        "sys": {"t": now, "data": {"cpu": 37.5, "temp": 54}},
        "gripper": {"t": now, "data": {"pos": 0.04, "cmd": 0.04}},
    }
    document = assemble(
        now,
        sources,
        robot_id="omnibot-1",
        sim=False,
        joint_limits=limits,
        rtt_ms=18.5,
        heartbeat_hz=50,
        arm_present=True,
    )
    assert document["age_ms"]["base.cmd"] == pytest.approx(50.0)
    assert document["age_ms"]["base"] == pytest.approx(100.0)
    assert document["battery"]["simulated"] is False
    assert document["joints"][0]["min"] == pytest.approx(-1.57)
    assert document["joints"][0]["max"] == pytest.approx(1.57)
    assert_valid_telemetry(document)
    body = schema_body(document)
    assert "age_ms" not in body
    assert body["mode"] == {"base": "teleop", "arm": "teleop"}
    assert body["nav"]["state"] == "navigating"
    assert body["deadman"] is True
