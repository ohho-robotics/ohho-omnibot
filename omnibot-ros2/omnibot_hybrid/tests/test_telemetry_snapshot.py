"""Pure assembly tests for ohho.telemetry.v1. No ROS."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from omnibot_hybrid.telemetry_snapshot import (
    assemble,
    dumps_json,
    echo_heartbeat,
    fields_to_grey,
    joint_limits_from_arm_params,
    map_arm_mode,
    map_base_mode,
    map_estop_deadman,
    nav_phase,
    parse_recorder_status,
    percent_from_voltage,
    policy_from_parts,
    rate_hz,
    resolve_robot_id,
    rpy_from_quaternion,
    select_goal_status,
)
from schema_tools import assert_valid_telemetry

ARM_PARAMS = (
    Path(__file__).resolve().parents[2] / "omnibot_arm" / "config" / "arm_params.yaml"
)
NOW = 1_735_689_600.5


def _at(age_s: float, data):
    return {"t": NOW - age_s, "data": data}


def _limits():
    return joint_limits_from_arm_params(ARM_PARAMS.read_text(encoding="utf-8"))


def test_arm_params_yaml_limits():
    limits = _limits()
    assert list(limits) == [
        "arm_shoulder_pan",
        "arm_shoulder_lift",
        "arm_elbow_flex",
        "arm_wrist_flex",
        "arm_wrist_roll",
        "arm_gripper",
    ]
    assert limits["arm_shoulder_pan"] == pytest.approx((-3.14, 3.14))
    assert limits["arm_gripper"] == pytest.approx((-0.1, 0.8))


def test_sim_without_sources_is_valid_and_simulated():
    document = assemble(
        NOW,
        {},
        robot_id="omnibot-sim",
        sim=True,
        joint_limits=_limits(),
        arm_present=False,
        rtt_ms=5,
        heartbeat_hz=20,
    )
    assert document["kind"] == "sim"
    assert document["mode"] == {"base": "idle", "arm": "not_present"}
    assert document["deadman"] is False
    assert document["estop"] == {"active": False, "latched": False, "source": "none"}
    assert document["battery"] == {"v": 12.6, "pct": 100.0, "simulated": True}
    assert document["age_ms"]["battery"] == 0.0
    assert "joints" not in document
    assert "base" not in document
    assert_valid_telemetry(document)


def test_real_robot_omits_battery_until_a_pack_sample_arrives():
    document = assemble(
        NOW,
        {},
        robot_id="omnibot",
        sim=False,
        joint_limits={},
        arm_present=True,
    )
    assert document["kind"] == "real"
    assert "battery" not in document
    assert document["mode"]["arm"] == "idle"
    assert_valid_telemetry(document)


def test_sim_flag_forces_simulated_even_when_voltage_is_measured():
    document = assemble(
        NOW,
        {"voltage": _at(1.0, {"v": 11.25, "pct": None})},
        robot_id="omnibot-sim",
        sim=True,
        joint_limits={},
        arm_present=False,
    )
    assert document["battery"]["v"] == pytest.approx(11.25)
    assert document["battery"]["pct"] == pytest.approx(50.0)
    assert document["battery"]["simulated"] is True
    assert document["age_ms"]["battery"] == pytest.approx(1000.0)


def test_voltage_percent_curve_does_not_clamp_the_voltage():
    assert percent_from_voltage(24.0, 9.9, 12.6) == 100.0
    assert percent_from_voltage(9.0, 9.9, 12.6) == 0.0
    document = assemble(
        NOW,
        {"voltage": _at(0.0, {"v": 24.0, "pct": None})},
        robot_id="omnibot",
        sim=False,
        joint_limits={},
    )
    assert document["battery"]["v"] == pytest.approx(24.0)
    assert document["battery"]["pct"] == 100.0
    assert document["battery"]["simulated"] is False


def test_zero_command_is_kept_and_missing_base_part_omits_the_block():
    odom = _at(
        0.1,
        {"frame": "odom", "x": 1.0, "y": 2.0, "yaw": 0.0, "vx": 0.0, "vy": 0.0, "wz": 0.0},
    )
    cmd = _at(0.05, {"vx": 0.0, "vy": 0.0, "wz": 0.0})
    document = assemble(
        NOW,
        {"odom": odom, "cmd_vel": cmd},
        robot_id="omnibot",
        sim=False,
        joint_limits={},
        sim_battery_v=None,
        sim_battery_pct=None,
    )
    assert document["base"]["cmd"] == {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    assert document["base"]["meas"]["vx"] == 0.0
    assert document["age_ms"]["base.cmd"] == pytest.approx(50.0)
    assert document["age_ms"]["base.pose"] == pytest.approx(100.0)
    missing = assemble(
        NOW,
        {"cmd_vel": cmd},
        robot_id="omnibot",
        sim=False,
        joint_limits={},
        sim_battery_v=None,
    )
    assert "base" not in missing


def test_position_only_joint_state_is_omitted_until_velocity_and_effort_exist():
    limits = _limits()
    names = list(limits)
    state = _at(0.0, {"names": names, "position": [0.1] * 6, "velocity": [], "effort": []})
    cmd = _at(0.0, {"names": names, "position": [0.2] * 6})
    document = assemble(
        NOW,
        {"joint_state": state, "joint_cmd": cmd},
        robot_id="omnibot",
        sim=False,
        joint_limits=limits,
        sim_battery_v=None,
    )
    assert "joints" not in document

    state["data"]["velocity"] = [0.0] * 6
    state["data"]["effort"] = [0.0] * 6
    full = assemble(
        NOW,
        {"joint_state": state, "joint_cmd": cmd},
        robot_id="omnibot",
        sim=False,
        joint_limits=limits,
        sim_battery_v=None,
    )
    assert len(full["joints"]) == 6
    gripper = full["joints"][-1]
    assert gripper["name"] == "arm_gripper"
    assert gripper["min"] == pytest.approx(-0.1)
    assert gripper["max"] == pytest.approx(0.8)
    assert gripper["vel"] == 0.0
    assert gripper["cmd"] == pytest.approx(0.2)
    assert "gripper" not in full
    assert_valid_telemetry(full)


def test_command_without_names_follows_arm_params_order():
    limits = _limits()
    state = _at(
        0.0,
        {
            "names": list(limits),
            "position": [0.0] * 6,
            "velocity": [0.0] * 6,
            "effort": [0.0] * 6,
        },
    )
    cmd = _at(0.0, {"names": [], "position": [0.3, 0.0, 0.0, 0.0, 0.0, 0.1]})
    document = assemble(
        NOW,
        {"joint_state": state, "joint_cmd": cmd},
        robot_id="omnibot",
        sim=False,
        joint_limits=limits,
        sim_battery_v=None,
    )
    assert document["joints"][0]["cmd"] == pytest.approx(0.3)
    assert document["joints"][-1]["name"] == "arm_gripper"
    assert document["joints"][-1]["cmd"] == pytest.approx(0.1)


def test_stale_values_stay_in_the_sample_and_are_listed_for_grey_out():
    document = assemble(
        NOW,
        {
            "odom": _at(
                1.0,
                {
                    "frame": "odom",
                    "x": 0.0,
                    "y": 0.0,
                    "yaw": 0.0,
                    "vx": 0.1,
                    "vy": 0.0,
                    "wz": 0.0,
                },
            ),
            "cmd_vel": _at(0.01, {"vx": 0.1, "vy": 0.0, "wz": 0.0}),
        },
        robot_id="omnibot",
        sim=False,
        joint_limits={},
        sim_battery_v=None,
    )
    assert document["base"]["meas"]["vx"] == pytest.approx(0.1)
    assert document["age_ms"]["base.meas"] == pytest.approx(1000.0)
    stale = fields_to_grey(document["age_ms"])
    assert "base.meas" in stale
    assert "base.pose" in stale
    assert "base.cmd" not in stale


def test_modes_and_latched_estop():
    assert map_base_mode("nav2") == "auto"
    assert map_base_mode("vla") == "auto"
    assert map_base_mode("rl_nav") == "auto"
    assert map_base_mode("teleop") == "teleop"
    assert map_base_mode("nope") == "idle"
    assert map_arm_mode("teleop", "policy", arm_present=True) == "teleop"
    assert map_arm_mode("nav2", "policy", arm_present=True) == "auto"
    assert map_arm_mode("rl_nav", "rl_arm", arm_present=True) == "auto"
    assert map_arm_mode("teleop", "policy", arm_present=False) == "not_present"

    estop, deadman, measured = map_estop_deadman(
        {"armed": False, "latched": True, "reason": "estop"}
    )
    assert measured is True
    assert deadman is False
    assert estop == {"active": False, "latched": True, "source": "software"}

    watchdog, _, _ = map_estop_deadman(
        {"armed": False, "latched": True, "reason": "watchdog"}
    )
    assert watchdog["source"] == "watchdog"

    clear, armed, _ = map_estop_deadman(
        {"armed": True, "latched": False, "reason": "heartbeat_watchdog"}
    )
    assert clear["source"] == "none"
    assert armed is True

    missing, dead, measured_missing = map_estop_deadman(None)
    assert measured_missing is False
    assert dead is False
    assert missing["source"] == "none"

    document = assemble(
        NOW,
        {
            "control_mode": _at(0.2, "nav2"),
            "arm_mode": _at(0.4, "policy"),
            "safety": _at(0.2, {"armed": False, "latched": True, "reason": "estop"}),
        },
        robot_id="omnibot",
        sim=False,
        joint_limits={},
        sim_battery_v=None,
    )
    assert document["mode"] == {"base": "auto", "arm": "auto"}
    assert document["estop"]["latched"] is True
    assert document["deadman"] is False
    assert document["age_ms"]["estop"] == pytest.approx(200.0)
    assert document["age_ms"]["mode.arm"] == pytest.approx(400.0)
    assert_valid_telemetry(document)


def test_schema_estop_passthrough_repairs_illegal_source():
    estop, deadman, measured = map_estop_deadman(
        {
            "deadman": False,
            "estop": {"active": True, "latched": True, "source": "none"},
        }
    )
    assert measured is True
    assert deadman is False
    assert estop["source"] == "software"
    assert estop["active"] is True


def test_nav_policy_and_recorder():
    assert nav_phase(2) == "navigating"
    assert nav_phase(4) == "succeeded"
    assert nav_phase(6) == "aborted"
    assert select_goal_status([4, 2, 5]) == 2
    assert parse_recorder_status("idle") is None
    assert parse_recorder_status("recording:/tmp/ep") is None
    parsed = parse_recorder_status("recording:ep-12:340")
    assert parsed == {"active": True, "episode": "ep-12", "frames": 340}
    assert policy_from_parts(enabled=True, task="pick", latency_ms=None) is None
    block = policy_from_parts(enabled=True, task="pick", latency_ms=8, level="error")
    assert block["state"] == "fault"
    assert block["latency_ms"] == 8

    document = assemble(
        NOW,
        {
            "nav_status": _at(0.2, 4),
            "nav_feedback": _at(
                0.1, {"eta_s": 0.0, "goal": {"x": 1.0, "y": -1.0, "yaw": 0.5}}
            ),
            "policy": _at(0.0, block),
            "recorder": _at(0.0, "recording:ep-12:340"),
        },
        robot_id="omnibot",
        sim=False,
        joint_limits={},
        sim_battery_v=None,
    )
    assert document["nav"]["state"] == "succeeded"
    assert document["nav"]["eta"] == 0.0
    assert document["policy"]["state"] == "fault"
    assert document["rec"]["frames"] == 340
    assert document["age_ms"]["nav"] == pytest.approx(200.0)
    assert_valid_telemetry(document)

    no_goal = assemble(
        NOW,
        {
            "nav_status": _at(0.0, 2),
            "nav_feedback": _at(0.0, {"eta_s": 3.0, "goal": None}),
        },
        robot_id="omnibot",
        sim=False,
        joint_limits={},
        sim_battery_v=None,
    )
    assert "nav" not in no_goal


def test_heartbeat_echo_copies_seq_and_rejects_bad_clocks():
    echo = echo_heartbeat({"v": "1.0.0", "seq": 4, "t_client": 10}, 11.5)
    assert echo == {"v": "1.0.0", "seq": 4, "t_client": 10, "t": 11.5}
    assert echo_heartbeat({"v": "1.0.0", "seq": 1, "t_client": 1, "extra": 1}, 2) is None
    assert echo_heartbeat({"v": "1.0.0", "seq": True, "t_client": 1}, 2) is None
    assert echo_heartbeat({"v": "1.0.0", "seq": 1, "t_client": 1}, -1) is None
    assert echo_heartbeat({"v": "2.0.0", "seq": 1, "t_client": 1}, 2) is None


def test_rate_over_a_window():
    stamps = [0.0, 0.1, 0.2]
    assert rate_hz(stamps, 0.2, window_s=2.0) == pytest.approx(10.0)
    assert rate_hz([0.2], 0.2, window_s=2.0) == 0.0
    assert rate_hz([], 1.0) == 0.0


def test_quaternion_yaw_and_identity():
    roll, pitch, yaw = rpy_from_quaternion(0.0, 0.0, 0.0, 1.0)
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)
    half = math.sqrt(0.5)
    _, _, yaw90 = rpy_from_quaternion(0.0, 0.0, half, half)
    assert yaw90 == pytest.approx(math.pi / 2)
    assert rpy_from_quaternion(0.0, 0.0, 0.0, 0.0) is None


def test_robot_id_and_clock_guards():
    assert resolve_robot_id("", sim=True) == "omnibot-sim"
    assert resolve_robot_id("bad id", sim=False) == "omnibot"
    assert resolve_robot_id("omnibot-1", sim=False) == "omnibot-1"
    with pytest.raises(ValueError):
        assemble(-1, {}, robot_id="omnibot", sim=False, joint_limits={})
    with pytest.raises(ValueError):
        assemble(NOW, {}, robot_id="", sim=False, joint_limits={})
    with pytest.raises(TypeError):
        assemble(NOW, {}, robot_id="omnibot", sim="true", joint_limits={})


def test_dumps_rejects_nan_and_launch_wiring_is_separate():
    with pytest.raises(ValueError):
        dumps_json({"t": float("nan")})
    launch = Path(__file__).resolve().parents[1] / "launch" / "telemetry.launch.py"
    text = launch.read_text(encoding="utf-8")
    assert "telemetry_aggregator" in text
    assert 'DeclareLaunchArgument(\n                "sim"' in text or '"sim"' in text
    assert "use_sim_time" in text
    setup = (Path(__file__).resolve().parents[1] / "setup.py").read_text(encoding="utf-8")
    assert "telemetry_aggregator = omnibot_hybrid.telemetry_aggregator:main" in setup
    sim = (
        Path(__file__).resolve().parents[2] / "omnibot_bringup" / "launch" / "sim.launch.py"
    ).read_text(encoding="utf-8")
    assert "telemetry.launch.py" not in sim
    assert "telemetry_aggregator" not in sim
