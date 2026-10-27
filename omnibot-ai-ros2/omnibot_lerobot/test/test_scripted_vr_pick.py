# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Scripted virtual headset: labels, safety gate, and the reject report."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys

import numpy as np
import pytest

_TEST_DIR = Path(__file__).resolve().parent
_PKG_DIR = _TEST_DIR.parent
_REPO = _PKG_DIR.parents[1]
_HYBRID = _REPO / "omnibot-ros2" / "omnibot_hybrid"
_RL = _REPO / "omnibot-ai-ros2" / "omnibot_rl"
for path in (_PKG_DIR, _HYBRID, _RL):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from omnibot_hybrid.stream_gate import StreamGate  # noqa: E402
from omnibot_hybrid.teleop_gate import (  # noqa: E402
    HARD_MAX_VX,
    HARD_MAX_VY,
    HARD_MAX_WZ,
    JOINT_LIMITS,
    MAX_JOINT_DELTA_RAD,
    TeleopGate,
)
from omnibot_lerobot.lerobot_v2 import append_episode  # noqa: E402
from omnibot_lerobot.scripted_vr_pick import (  # noqa: E402
    EPISODE_SECONDS,
    HEADSET_TOPICS,
    JOINT_NAMES,
    MAX_JOINT_STEP_RAD,
    MIN_JOINT_MOTION_RAD,
    OPERATOR,
    PHASES,
    PLANT_TOPICS,
    RATE_HZ,
    collection_record,
    isolation_failure,
    iter_open_loop,
    joint_motion_rad,
    scripted_episode_metadata,
    write_reject_report,
)
from omnibot_rl.arm_stream_gate import ArmStreamGate  # noqa: E402


HEADSET_SOURCE = _PKG_DIR / "omnibot_lerobot" / "scripted_vr_headset.py"
LAUNCH_SOURCE = _PKG_DIR / "launch" / "scripted_vr_pick.launch.py"


def test_episode_is_labelled_scripted_sim_not_human_vr():
    meta = scripted_episode_metadata()
    assert meta["operator"] == "scripted-sim"
    assert meta["operator"] != "vr"
    assert meta["robot_kind"] == "sim"
    assert meta["sim_world"] == "apartment"
    assert meta["task"].startswith("scripted-sim")
    record = collection_record(237, 3)
    assert record["human_vr"] is False
    assert record["collector"] == OPERATOR
    assert record["action_source"] == "vr"
    assert set(record["headset_topics"]).isdisjoint(PLANT_TOPICS)


def test_headset_publishers_are_only_the_client_contract():
    text = HEADSET_SOURCE.read_text(encoding="utf-8")
    publisher_lines = [line for line in text.splitlines() if "create_publisher" in line]
    assert len(publisher_lines) == 4
    allowed = ("RAW_ARM_TOPIC", "RAW_BASE_TOPIC", "HEARTBEAT_TOPIC", "CONTROL_MODE_TOPIC")
    for line in publisher_lines:
        assert any(name in line for name in allowed)
    assert set(HEADSET_TOPICS).isdisjoint(PLANT_TOPICS)
    assert 'declare_parameter("episodes", 3)' in text
    assert 'declare_parameter("pick_place_seed", 0)' in text
    assert "joint_motion_rad" in text
    assert "MIN_JOINT_MOTION_RAD" in text
    assert "/ohho/estop_reset" in text


def test_launch_uses_safety_muxes_and_vr_recorder():
    text = LAUNCH_SOURCE.read_text(encoding="utf-8")
    assert 'executable="teleop_safety"' in text
    assert 'executable="cmd_vel_mux"' in text
    assert 'executable="arm_cmd_mux"' in text
    assert 'executable="teleop_recorder_node"' in text
    assert '"action_source": "vr"' in text
    assert '"operator": "scripted-sim"' in text
    assert '"cmd_vel_from_mux": "true"' in text
    assert "from omnibot_lerobot.scripted_vr_pick import isolation_failure" in text
    assert "def _isolation_failure" not in text
    assert "isolation_failure()" in text
    assert '"vr": "true"' in text
    assert '"spawn_x": "0.20"' in text
    assert '"spawn_y": "1.6575"' in text
    assert '"spawn_yaw": "-1.5707963267948966"' in text
    assert '"sim_world": LaunchConfiguration("world")' in text


def test_isolation_refuses_the_robot_domain():
    assert isolation_failure({"ROS_DOMAIN_ID": "30", "ROS_LOCALHOST_ONLY": "1"})
    assert isolation_failure({"ROS_DOMAIN_ID": "88", "ROS_LOCALHOST_ONLY": "0"})
    assert isolation_failure({})
    assert isolation_failure({"ROS_DOMAIN_ID": "237", "ROS_LOCALHOST_ONLY": "1"})
    assert isolation_failure({"ROS_DOMAIN_ID": "88", "ROS_LOCALHOST_ONLY": "1"}) is None
    assert isolation_failure({"ROS_DOMAIN_ID": "030", "ROS_LOCALHOST_ONLY": "1"})
    assert isolation_failure({"ROS_DOMAIN_ID": "036", "ROS_LOCALHOST_ONLY": "1"})
    assert isolation_failure({"ROS_DOMAIN_ID": " 30", "ROS_LOCALHOST_ONLY": "1"})
    assert isolation_failure({"ROS_DOMAIN_ID": "+30", "ROS_LOCALHOST_ONLY": "1"})
    assert isolation_failure({"ROS_DOMAIN_ID": "00", "ROS_LOCALHOST_ONLY": "1"})
    assert isolation_failure({"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "1"}) is None
    assert isolation_failure({"ROS_DOMAIN_ID": "232", "ROS_LOCALHOST_ONLY": "1"}) is None


def test_joint_motion_rad_is_the_largest_absolute_delta():
    assert joint_motion_rad(None, [0.0]) == 0.0
    assert joint_motion_rad([0.0, 0.0], [0.0]) == 0.0
    assert joint_motion_rad([0.0, 0.0], [0.0, 0.01]) == pytest.approx(0.01)
    assert joint_motion_rad([0.0, 0.0], [0.0, MIN_JOINT_MOTION_RAD]) == pytest.approx(
        MIN_JOINT_MOTION_RAD
    )


def test_script_stays_inside_joint_limits_and_lasts_long_enough():
    assert EPISODE_SECONDS >= 8.0
    assert MAX_JOINT_STEP_RAD <= MAX_JOINT_DELTA_RAD
    for phase in PHASES:
        for name, value in zip(JOINT_NAMES, phase.joints):
            low, high = JOINT_LIMITS[name]
            assert low <= value <= high
        assert abs(phase.twist[0]) <= HARD_MAX_VX
        assert abs(phase.twist[1]) <= HARD_MAX_VY
        assert abs(phase.twist[2]) <= HARD_MAX_WZ
    samples = list(iter_open_loop([0.0] * 6))
    assert len(samples) / RATE_HZ == pytest.approx(EPISODE_SECONDS)
    previous = (0.0,) * 6
    for sample in samples:
        for left, right in zip(previous, sample.joints):
            assert abs(right - left) <= MAX_JOINT_STEP_RAD + 1e-9
        previous = sample.joints


def test_commands_pass_through_teleop_gate_and_muxes(tmp_path):
    now = {"t": 0.0}

    def clock():
        return now["t"]

    gate = TeleopGate(clock=clock, audit_dir=tmp_path)
    assert gate.reset(0.0).ok
    gate.note_joint_state(JOINT_NAMES, [0.0] * 6)
    now["t"] = 0.1
    dropped = gate.command_joints(JOINT_NAMES, [0.2] * 6, now["t"])
    zeroed = gate.command_velocity(0.2, 0.0, 0.0, now["t"])
    assert not dropped.joints
    assert zeroed.twist == (0.0, 0.0, 0.0)

    base = StreamGate()
    arm = ArmStreamGate.from_default_mode("policy")
    assert base.base_allows("teleop") is True
    assert arm.allows("teleop") is True
    assert arm.allows("policy") is False

    measured = [0.0] * 6
    saw_motion = False
    previous = list(measured)
    for sample in iter_open_loop(measured):
        now["t"] = sample.t
        gate.heartbeat(sample.t)
        joints = gate.command_joints(JOINT_NAMES, sample.joints, sample.t)
        twist = gate.command_velocity(*sample.twist, sample.t)
        assert joints.joints
        assert twist.twist is not None
        assert abs(twist.twist[0]) <= HARD_MAX_VX + 1e-9
        assert abs(twist.twist[1]) <= HARD_MAX_VY + 1e-9
        assert abs(twist.twist[2]) <= HARD_MAX_WZ + 1e-9
        applied = [position for _name, position in joints.joints]
        for left, right in zip(previous, applied):
            assert abs(right - left) <= MAX_JOINT_DELTA_RAD + 1e-6
        if any(abs(value) > 0.05 for value in applied):
            saw_motion = True
        previous = applied
        gate.note_joint_state(JOINT_NAMES, applied)
        assert arm.allows("teleop") is True
        assert base.base_allows("teleop") is True
    assert saw_motion

    gate.set_emergency_stop(True, now["t"])
    arm.set_emergency_stop(True)
    base.set_emergency_stop(True)
    stopped = gate.command_velocity(0.05, 0.0, 0.0, now["t"])
    held = gate.command_joints(JOINT_NAMES, previous, now["t"])
    assert stopped.twist == (0.0, 0.0, 0.0)
    assert not held.joints
    assert arm.allows("teleop") is False
    assert arm.allows("policy") is False
    assert base.base_allows("teleop") is False


def _audit_text(directory):
    files = list(directory.glob("*.jsonl"))
    assert len(files) == 1
    return files[0].read_text(encoding="utf-8")


def test_repeated_clamp_audits_collapse_until_the_reason_changes(tmp_path):
    now = {"t": 5.0}

    def clock():
        return now["t"]

    gate = TeleopGate(clock=clock, audit_dir=tmp_path)
    assert gate.reset(5.0).ok
    gate.heartbeat(5.0)
    gate.command_velocity(1.0, 0.0, 0.0, 5.0)
    gate.command_velocity(1.0, 0.0, 0.0, 5.0)
    assert _audit_text(tmp_path).count('"event": "clamp"') == 1
    gate.command_velocity(0.05, 0.0, 0.0, 5.0)
    assert _audit_text(tmp_path).count('"event": "clamp"') == 2

    gate.note_joint_state(JOINT_NAMES, [0.0] * 6)
    gate.command_joints(JOINT_NAMES, [0.4] * 6, 5.0)
    gate.command_joints(JOINT_NAMES, [0.4] * 6, 5.0)
    assert _audit_text(tmp_path).count('"command": "joints"') == 1

    gate.set_emergency_stop(True, 5.0)
    gate.command_velocity(0.1, 0.0, 0.0, 5.0)
    gate.command_velocity(0.2, 0.0, 0.0, 5.0)
    assert _audit_text(tmp_path).count('"reason": "estop_latched"') == 1


def test_watchdog_trip_is_recorded_again_after_recovery(tmp_path):
    gate = TeleopGate(clock=lambda: 0.0, audit_dir=tmp_path)
    assert gate.reset(0.0).ok
    gate.heartbeat(0.0)
    first = gate.tick(1.0)
    assert first.twist == (0.0, 0.0, 0.0)
    text = _audit_text(tmp_path)
    assert text.count('"event": "watchdog_trip"') == 1
    assert text.count('"command": "tick"') == 1
    held = gate.tick(1.1)
    assert held.twist is None
    assert _audit_text(tmp_path).count('"event": "watchdog_trip"') == 1
    gate.heartbeat(1.1)
    second = gate.tick(2.0)
    assert second.twist == (0.0, 0.0, 0.0)
    text = _audit_text(tmp_path)
    assert text.count('"event": "watchdog_trip"') == 2
    assert text.count('"command": "tick"') == 2


def test_audit_failure_zeros_velocity_once_on_the_next_tick(tmp_path):
    blocked = tmp_path / "not-a-directory"
    blocked.write_text("x", encoding="utf-8")
    gate = TeleopGate(clock=lambda: 0.0, audit_dir=blocked)
    first = gate.tick(0.0)
    assert first.twist == (0.0, 0.0, 0.0)
    assert first.log == "audit log failed; velocity zeroed"
    second = gate.tick(0.01)
    assert second.twist is None


def test_reject_report_uses_the_quality_filter(tmp_path):
    frames = []
    for index in range(5):
        image = np.zeros((8, 8, 3), dtype=np.uint8)
        frames.append(
            {
                "state": np.zeros(9, dtype=np.float32),
                "action": np.zeros(9, dtype=np.float32),
                "wrist_image": image,
                "bev_image": image.copy(),
                "timestamp": index * 0.05,
                "wrist_timestamp": index * 0.05,
                "bev_timestamp": index * 0.05,
            }
        )
    append_episode(
        tmp_path,
        frames,
        scripted_episode_metadata(),
        fps=10,
        episode_index=0,
        repo_id="local/omnibot-sim-vr-pick",
    )
    report = tmp_path / "reject_report.md"
    result = write_reject_report(tmp_path, report)
    text = report.read_text(encoding="utf-8")
    assert result.rejected == [0]
    assert result.kept == []
    assert "REJECT" in text
    assert "episode_too_short" in text
    payload = json.loads((tmp_path / "meta" / "filter_results.json").read_text(encoding="utf-8"))
    assert payload["rejected"] == [0]
    assert os.path.isfile(tmp_path / "meta" / "filter_results.json")
