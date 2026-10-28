# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Teleop safety gate. No ROS 2 install required.

The rclpy node is imported in a subprocess with stubbed ROS so the topic
wiring can be checked on machines that do not have ROS.
"""

import json
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
HYBRID_ROOT = REPO / "omnibot-ros2" / "omnibot_hybrid"
BRINGUP = REPO / "omnibot-ros2" / "omnibot_bringup"
GATE_SRC = HYBRID_ROOT / "omnibot_hybrid" / "teleop_gate.py"
NODE_SRC = HYBRID_ROOT / "omnibot_hybrid" / "teleop_safety.py"
SAFETY_LAUNCH = HYBRID_ROOT / "launch" / "teleop_safety.launch.py"

sys.path.insert(0, str(HYBRID_ROOT))

from omnibot_hybrid.teleop_gate import (  # noqa: E402
    AUDIT_DIR_ENV,
    AUDIT_REPEAT_S,
    CMD_TIMEOUT_S,
    HARD_MAX_ACCEL_X,
    HARD_MAX_VX,
    HARD_MAX_WZ,
    HEARTBEAT_TIMEOUT_S,
    MAX_JOINT_DELTA_RAD,
    TeleopGate,
    default_audit_dir,
    parameter_change_allowed,
)


class Clock:
    def __init__(self, stamp=0.0):
        self.t = stamp

    def __call__(self):
        return self.t


def make_gate(tmp_path, clock=None, *, release_startup=True, **kwargs):
    clock = clock or Clock()
    gate = TeleopGate(clock=clock, audit_dir=tmp_path, **kwargs)
    if release_startup:
        released = gate.reset(clock())
        assert released.ok is True, released.message
        assert gate.latched is False
    return gate, clock


def _audit_text(tmp_path):
    return "\n".join(
        path.read_text(encoding="utf-8") for path in tmp_path.glob("*.jsonl")
    )


def _audit_events(tmp_path):
    events = []
    for path in sorted(tmp_path.glob("*.jsonl")):
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                events.append(json.loads(line))
    return events


def test_ceilings_match_omnibot_profile_and_stop_budget():
    assert HARD_MAX_VX == 0.2
    assert HARD_MAX_WZ == 1.0
    assert HARD_MAX_ACCEL_X == 0.5
    assert CMD_TIMEOUT_S == 0.3
    assert HEARTBEAT_TIMEOUT_S == 0.3
    assert MAX_JOINT_DELTA_RAD <= 0.15
    assert CMD_TIMEOUT_S + 0.01 <= 0.350
    source = NODE_SRC.read_text(encoding="utf-8")
    assert "TICK_PERIOD_S = 0.01" in source
    assert "declare_parameter" not in source
    # Jazzy's rclpy.parameter does not export SetParametersResult. The callback
    # result is the rcl_interfaces message.
    assert "from rcl_interfaces.msg import SetParametersResult" in source
    assert "from rclpy.parameter import SetParametersResult" not in source


def test_callers_cannot_raise_caps_or_timeouts(tmp_path):
    gate, _clock = make_gate(
        tmp_path,
        max_vx=9.0,
        max_vy=9.0,
        max_wz=9.0,
        max_accel_x=99.0,
        max_accel_y=99.0,
        max_accel_wz=99.0,
        cmd_timeout_s=5.0,
        heartbeat_timeout_s=5.0,
        max_joint_delta_rad=2.0,
    )
    assert gate.max_vx == 0.2
    assert gate.max_vy == 0.2
    assert gate.max_wz == 1.0
    assert gate.max_accel_x == 0.5
    assert gate.max_accel_y == 0.5
    assert gate.max_accel_wz == 2.0
    assert gate.cmd_timeout_s == 0.3
    assert gate.heartbeat_timeout_s == 0.3
    assert gate.max_joint_delta_rad <= 0.15

    loose, _clock = make_gate(
        tmp_path,
        max_vx=float("nan"),
        cmd_timeout_s=0.0,
    )
    assert loose.max_vx == 0.0
    assert loose.cmd_timeout_s == 0.3

    tight, _clock = make_gate(
        tmp_path,
        max_vx=0.05,
        cmd_timeout_s=0.1,
        max_joint_delta_rad=0.05,
    )
    assert tight.max_vx == 0.05
    assert tight.cmd_timeout_s == 0.1
    assert tight.max_joint_delta_rad == 0.05


def test_parameter_changes_cannot_raise_limits_or_use_sim_time():
    assert parameter_change_allowed("max_vx", 1.0) is False
    assert parameter_change_allowed("max_vx", 0.05) is False
    assert parameter_change_allowed("cmd_timeout_s", 1.0) is False
    assert parameter_change_allowed("disable_watchdog", True) is False
    assert parameter_change_allowed("use_sim_time", True) is False
    assert parameter_change_allowed("use_sim_time", False) is True


def test_velocity_cap_clamp(tmp_path):
    gate, _clock = make_gate(tmp_path)
    gate.heartbeat(0.0)
    twist = (0.0, 0.0, 0.0)
    stamp = 0.0
    for _ in range(8):
        stamp += 0.3
        gate.heartbeat(stamp)
        twist = gate.command_velocity(5.0, -5.0, 9.0, now=stamp).twist
    assert twist[0] == pytest.approx(0.2)
    assert twist[1] == pytest.approx(-0.2)
    assert twist[2] == pytest.approx(1.0)
    assert abs(twist[0]) <= 0.2 + 1e-9
    assert abs(twist[1]) <= 0.2 + 1e-9
    assert abs(twist[2]) <= 1.0 + 1e-9
    assert "velocity_cap" in _audit_text(tmp_path)


def test_acceleration_limit_and_free_braking(tmp_path):
    gate, _clock = make_gate(tmp_path)
    gate.heartbeat(0.0)
    first = gate.command_velocity(0.2, 0.0, 0.0, now=0.1).twist
    assert first[0] == pytest.approx(0.05)
    gate.heartbeat(0.2)
    second = gate.command_velocity(0.2, 0.0, 0.0, now=0.2).twist
    assert second[0] == pytest.approx(0.10)
    gate.heartbeat(0.25)
    stopped = gate.command_velocity(0.0, 0.0, 0.0, now=0.25).twist
    assert stopped == (0.0, 0.0, 0.0)

    gate.heartbeat(1.3)
    gate.command_velocity(0.2, 0.0, 0.0, now=1.3)
    gate.heartbeat(1.6)
    held = gate.command_velocity(0.2, 0.0, 0.0, now=1.6).twist
    assert held[0] == pytest.approx(0.2)
    gate.heartbeat(1.7)
    reversal = gate.command_velocity(-0.2, 0.0, 0.0, now=1.7).twist
    assert reversal[0] == pytest.approx(-0.05)


def test_command_watchdog_trips_at_300ms_and_does_not_latch(tmp_path):
    gate, _clock = make_gate(tmp_path)
    gate.heartbeat(0.0)
    moving = gate.command_velocity(0.2, 0.0, 0.0, now=0.2)
    assert moving.twist[0] > 0.0
    cmd_t = 0.2
    under = cmd_t + gate.cmd_timeout_s - 0.001
    gate.heartbeat(under)
    assert gate.tick(under).twist is None
    assert gate.latched is False

    at = cmd_t + gate.cmd_timeout_s
    gate.heartbeat(at)
    zero = gate.tick(at)
    assert zero.twist == (0.0, 0.0, 0.0)
    assert gate.latched is False
    assert gate.armed is True
    assert gate.reason == "command_watchdog"
    assert gate.tick(at).twist is None

    gate.heartbeat(at + 0.3)
    again = gate.command_velocity(0.2, 0.0, 0.0, now=at + 0.3)
    assert again.twist[0] > 0.0
    assert "watchdog_trip" in _audit_text(tmp_path)


def test_heartbeat_watchdog_zeros_without_latching(tmp_path):
    gate, _clock = make_gate(tmp_path)
    gate.heartbeat(0.0)
    gate.command_velocity(0.2, 0.0, 0.0, now=0.05)
    assert gate.tick(0.299).twist is None
    zero = gate.tick(0.300)
    assert zero.twist == (0.0, 0.0, 0.0)
    assert gate.latched is False
    assert gate.armed is False
    assert gate.reason == "heartbeat_watchdog"
    refused = gate.command_velocity(0.2, 0.0, 0.0, now=0.301)
    assert refused.twist == (0.0, 0.0, 0.0)

    gate.heartbeat(0.301)
    assert gate.armed is True
    resumed = gate.command_velocity(0.2, 0.0, 0.0, now=0.401)
    assert resumed.twist[0] > 0.0


def test_latch_survives_false_and_reset_requires_service(tmp_path):
    gate, _clock = make_gate(tmp_path)
    gate.heartbeat(0.0)
    gate.command_velocity(0.2, 0.0, 0.0, now=0.2)
    gate.heartbeat(0.2)
    latched = gate.set_emergency_stop(True, 0.2)
    assert latched.twist == (0.0, 0.0, 0.0)
    assert gate.latched is True
    assert gate.armed is False
    state = json.loads(gate.state_json(0.3))
    assert state == {"armed": False, "latched": True, "reason": "estop"}

    cleared = gate.set_emergency_stop(False, 0.4)
    assert cleared.twist is None
    assert gate.latched is True
    assert gate.armed is False
    held = gate.command_velocity(0.2, 0.0, 0.0, now=0.4)
    assert held.twist == (0.0, 0.0, 0.0)
    assert json.loads(gate.state_json(0.4))["latched"] is True

    refused = gate.reset(0.4)
    assert refused.ok is True
    assert gate.latched is False

    gate.set_emergency_stop(True, 0.5)
    blocked = gate.reset(0.5)
    assert blocked.ok is False
    assert gate.latched is True
    gate.set_emergency_stop(False, 0.6)
    assert gate.latched is True
    released = gate.reset(0.6)
    assert released.ok is True
    assert released.message == "e-stop reset"
    assert gate.latched is False
    gate.heartbeat(0.6)
    assert gate.armed is True
    resumed = gate.command_velocity(0.2, 0.0, 0.0, now=0.8)
    assert resumed.twist[0] > 0.0
    assert '"event": "estop"' in _audit_text(tmp_path)
    assert '"event": "estop_release"' in _audit_text(tmp_path)


def test_joint_limits_and_policy_guard_step(tmp_path):
    gate, _clock = make_gate(tmp_path)
    gate.heartbeat(0.0)
    missing = gate.command_joints(["arm_elbow_flex"], [0.2], now=0.0)
    assert missing.joints is None

    gate.note_joint_state(["arm_elbow_flex", "arm_gripper"], [0.0, 0.7])
    stepped = gate.command_joints(["arm_elbow_flex"], [1.0], now=0.0)
    assert stepped.joints[0][0] == "arm_elbow_flex"
    assert stepped.joints[0][1] == pytest.approx(MAX_JOINT_DELTA_RAD)

    limited = gate.command_joints(["arm_gripper"], [3.0], now=0.0)
    assert limited.joints[0][0] == "arm_gripper"
    assert limited.joints[0][1] == pytest.approx(0.8)

    unknown = gate.command_joints(["tail"], [0.1], now=0.0)
    assert unknown.joints is None

    gate.set_emergency_stop(True, 0.1)
    dropped = gate.command_joints(["arm_elbow_flex"], [0.1], now=0.1)
    assert dropped.joints is None
    gate.set_emergency_stop(False, 0.2)
    still = gate.command_joints(["arm_elbow_flex"], [0.1], now=0.2)
    assert still.joints is None
    assert "joint_step" in _audit_text(tmp_path)


def _velocity_cap_clamps(tmp_path):
    return [
        event
        for event in _audit_events(tmp_path)
        if event["event"] == "clamp" and event.get("reasons") == ["velocity_cap"]
    ]


def test_repeated_clamp_and_latched_reject_audits_are_throttled(tmp_path):
    gate, _clock = make_gate(tmp_path)
    gate.heartbeat(0.0)
    gate.heartbeat(0.5)
    gate.command_velocity(0.2, 0.0, 0.0, now=0.5)
    gate.heartbeat(0.8)
    at_cap = gate.command_velocity(0.2, 0.0, 0.0, now=0.8)
    assert at_cap.twist[0] == pytest.approx(0.2)

    first_stamp = 1.2
    gate.heartbeat(first_stamp)
    first = gate.command_velocity(5.0, 0.0, 0.0, now=first_stamp)
    assert first.twist[0] == pytest.approx(0.2)
    stamp = first_stamp
    while stamp < first_stamp + AUDIT_REPEAT_S - 0.05:
        stamp += 0.05
        gate.heartbeat(stamp)
        out = gate.command_velocity(5.0, 0.0, 0.0, now=stamp)
        assert out.twist[0] == pytest.approx(0.2)
    assert len(_velocity_cap_clamps(tmp_path)) == 1

    later = first_stamp + AUDIT_REPEAT_S + 0.05
    gate.heartbeat(later)
    again = gate.command_velocity(5.0, 0.0, 0.0, now=later)
    assert again.twist[0] == pytest.approx(0.2)
    assert len(_velocity_cap_clamps(tmp_path)) == 2

    gate.set_emergency_stop(True, later)
    for step in range(15):
        held = gate.command_velocity(0.2, 0.0, 0.0, now=later + step * 0.01)
        assert held.twist == (0.0, 0.0, 0.0)
    rejects = [
        event
        for event in _audit_events(tmp_path)
        if event["event"] == "reject" and event.get("reason") == "estop_latched"
    ]
    assert len(rejects) == 1
    assert any(event["event"] == "estop" for event in _audit_events(tmp_path))

    gate.set_emergency_stop(False, later + 0.2)
    released = gate.reset(later + 0.2)
    assert released.ok is True
    assert any(event["event"] == "estop_release" for event in _audit_events(tmp_path))


def _reject_count(tmp_path, reason, command):
    return sum(
        1
        for event in _audit_events(tmp_path)
        if event["event"] == "reject"
        and event.get("reason") == reason
        and event.get("command") == command
    )


def test_streaming_reject_audits_are_throttled(tmp_path):
    gate, clock = make_gate(tmp_path)

    def burst(at):
        clock.t = at
        gate.heartbeat(at)
        gate.command_velocity(float("nan"), 0.0, 0.0, now=at)
        gate.command_velocity(float("inf"), 0.0, 0.0, now=at)
        gate.command_joints([], [], now=at)
        gate.command_joints(["arm_elbow_flex"], [float("nan")], now=at)
        gate.command_joints(["not_a_joint"], [0.1], now=at)
        dropped = gate.command_joints(["arm_elbow_flex"], [0.2], now=at)
        assert dropped.joints is None

    for step in range(6):
        burst(1.0 + step * 0.04)
    assert _reject_count(tmp_path, "non_finite", "velocity") == 1
    assert _reject_count(tmp_path, "bad_joints", "joints") == 1
    assert _reject_count(tmp_path, "non_finite", "joints") == 1
    assert _reject_count(tmp_path, "unknown_joint", "joints") == 1
    assert _reject_count(tmp_path, "no_joint_reference", "joints") == 1

    burst(1.0 + AUDIT_REPEAT_S + 0.05)
    assert _reject_count(tmp_path, "non_finite", "velocity") == 2
    assert _reject_count(tmp_path, "bad_joints", "joints") == 2
    assert _reject_count(tmp_path, "non_finite", "joints") == 2
    assert _reject_count(tmp_path, "unknown_joint", "joints") == 2
    assert _reject_count(tmp_path, "no_joint_reference", "joints") == 2


def test_startup_latch_blocks_until_reset_and_returns_on_restart(tmp_path):
    gate, _clock = make_gate(tmp_path, release_startup=False)
    assert gate.latched is True
    assert gate.armed is False
    assert gate.reason == "startup"
    assert json.loads(gate.state_json(0.0)) == {
        "armed": False,
        "latched": True,
        "reason": "startup",
    }
    gate.heartbeat(0.0)
    assert gate.armed is False
    held = gate.command_velocity(0.2, 0.0, 0.0, now=0.1)
    assert held.twist == (0.0, 0.0, 0.0)
    gate.note_joint_state(["arm_elbow_flex"], [0.0])
    dropped = gate.command_joints(["arm_elbow_flex"], [0.1], now=0.1)
    assert dropped.joints is None
    assert gate.latched is True

    false_stop = gate.set_emergency_stop(False, 0.2)
    assert false_stop.twist is None
    assert gate.latched is True
    gate.heartbeat(0.2)
    assert gate.armed is False

    true_stop = gate.set_emergency_stop(True, 0.3)
    assert true_stop.twist == (0.0, 0.0, 0.0)
    assert gate.reason == "estop"
    refused = gate.reset(0.3)
    assert refused.ok is False
    assert gate.latched is True

    gate.set_emergency_stop(False, 0.4)
    released = gate.reset(0.4)
    assert released.ok is True
    assert gate.latched is False
    gate.heartbeat(0.4)
    assert gate.armed is True
    moving = gate.command_velocity(0.2, 0.0, 0.0, now=0.5)
    assert moving.twist[0] > 0.0

    restarted, _restart_clock = make_gate(tmp_path, release_startup=False)
    assert restarted.latched is True
    assert restarted.reason == "startup"
    restarted.heartbeat(1.0)
    assert restarted.armed is False
    assert restarted.command_velocity(0.2, 0.0, 0.0, now=1.1).twist == (
        0.0,
        0.0,
        0.0,
    )


def test_default_audit_dir_honors_nonempty_override(tmp_path):
    home = Path.home() / ".ohho" / "safety"
    assert default_audit_dir({}) == home
    assert default_audit_dir({AUDIT_DIR_ENV: ""}) == home
    assert default_audit_dir({AUDIT_DIR_ENV: str(tmp_path)}) == Path(tmp_path)


def test_audit_write_failure_stops_motion(tmp_path):
    blocked = tmp_path / "blocked"
    blocked.write_text("not a directory", encoding="utf-8")
    gate = TeleopGate(clock=Clock(), audit_dir=blocked)
    assert gate.armed is False
    assert gate.latched is True
    assert gate.reason == "audit_failure"
    gate.heartbeat(0.0)
    out = gate.command_velocity(0.2, 0.0, 0.0, now=0.2)
    assert out.twist == (0.0, 0.0, 0.0)
    refused = gate.reset(0.2)
    assert refused.ok is False
    assert gate.latched is True


def test_bringup_docs_state_heartbeat_and_raw_ceilings():
    reference = (REPO / "docs" / "reference.md").read_text(encoding="utf-8")
    safety = (REPO / "docs" / "safety.md").read_text(encoding="utf-8")
    assert "/ohho/heartbeat" in reference
    assert "std_msgs/Empty" in reference
    assert "/ohho/estop_reset" in reference
    assert "safety.md" in reference
    assert "pass through `teleop_safety`" in safety
    assert "/cmd_vel/teleop_raw" in safety
    assert "starts latched" in safety
    assert "OHHO_SAFETY_DIR" in safety
    assert "~/.ohho/safety/" in safety
    assert "heartbeat alone does not arm it" in reference


def test_launch_files_wire_teleop_safety():
    pi = (
        BRINGUP / "launch" / "master" / "omnibot_pi.launch.py"
    ).read_text(encoding="utf-8")
    mobile = (BRINGUP / "launch" / "mobile_manipulation.launch.py").read_text(
        encoding="utf-8"
    )
    hybrid = (
        REPO / "omnibot-ros2" / "omnibot_hybrid" / "launch" / "hybrid_robot.launch.py"
    ).read_text(encoding="utf-8")
    sim = (BRINGUP / "launch" / "sim.launch.py").read_text(encoding="utf-8")
    sim_teleop = (BRINGUP / "launch" / "sim_teleop.launch.py").read_text(
        encoding="utf-8"
    )
    safety_launch = SAFETY_LAUNCH.read_text(encoding="utf-8")
    vr_bridge = (BRINGUP / "config" / "sim_bridge_vr.yaml").read_text(encoding="utf-8")
    flat_bridge = (BRINGUP / "config" / "sim_bridge.yaml").read_text(encoding="utf-8")

    for text in (pi, mobile, hybrid):
        assert "teleop_safety.launch.py" in text
    # sim.launch.py vr:=true is the camera profile. The gate lives in
    # sim_teleop.launch.py so that file can keep the bridge name and the
    # camera tests can keep forbidding the gate token in sim.launch.py.
    assert 'default_value="false"' in sim
    assert "sim_bridge_vr.yaml" in sim
    assert "teleop_safety.launch.py" not in sim
    assert "teleop_safety.launch.py" in sim_teleop
    assert "sim_bridge_vr.yaml" in sim_teleop
    assert "sim.launch.py" in sim_teleop
    assert '"use_sim_time": False' in safety_launch
    assert "max_vx" not in safety_launch
    assert 'ros_topic_name: "/cmd_vel/out"' in vr_bridge
    assert 'ros_topic_name: "/cmd_vel"' not in vr_bridge
    assert 'ros_topic_name: "/cmd_vel"' in flat_bridge
    sim_vr = (BRINGUP / "launch" / "sim_vr.launch.py").read_text(encoding="utf-8")
    assert "sim.launch.py" in sim_vr
    assert '"vr": "true"' in sim_vr
    robot_vr = (BRINGUP / "launch" / "robot_vr_support.py").read_text(encoding="utf-8")
    assert "teleop_safety.launch.py" in robot_vr
    assert 'parameters=[{"default_mode": "teleop", "use_sim_time": False}]' in robot_vr
    assert "declare_parameter" not in GATE_SRC.read_text(encoding="utf-8")


def test_sim_launch_nodes_install_into_package_libexec():
    """Launch resolves executables in lib/<package>, which symlink installs miss."""
    hybrid_cfg = (HYBRID_ROOT / "setup.cfg").read_text(encoding="utf-8")
    rl_root = REPO / "omnibot-ai-ros2" / "omnibot_rl"
    rl_cfg = (rl_root / "setup.cfg").read_text(encoding="utf-8")
    assert "script_dir=$base/lib/omnibot_hybrid" in hybrid_cfg
    assert "install_scripts=$base/lib/omnibot_hybrid" in hybrid_cfg
    assert "script_dir=$base/lib/omnibot_rl" in rl_cfg
    assert "install_scripts=$base/lib/omnibot_rl" in rl_cfg
    hybrid_setup = (HYBRID_ROOT / "setup.py").read_text(encoding="utf-8")
    rl_setup = (rl_root / "setup.py").read_text(encoding="utf-8")
    assert "teleop_safety = omnibot_hybrid.teleop_safety:main" in hybrid_setup
    assert "cmd_vel_mux = omnibot_hybrid.cmd_vel_mux:main" in hybrid_setup
    assert "arm_cmd_mux        = omnibot_rl.arm_cmd_mux:main" in rl_setup


def test_teleop_safety_imports_set_parameters_result_from_rcl_interfaces():
    """Jazzy rclpy.parameter does not export SetParametersResult."""
    source = NODE_SRC.read_text(encoding="utf-8")
    assert "from rcl_interfaces.msg import SetParametersResult" in source
    assert "from rclpy.parameter import SetParametersResult" not in source


def _run_stubbed(script):
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(
        [str(HYBRID_ROOT), env.get("PYTHONPATH", "")]
    )
    completed = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(script)],
        capture_output=True,
        text=True,
        check=False,
        env=env,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr


def test_node_wiring_latch_and_service_without_ros(tmp_path):
    script = """
        import sys
        import tempfile
        from types import ModuleType, SimpleNamespace

        def _pkg(name):
            module = ModuleType(name)
            sys.modules[name] = module
            return module

        rclpy = _pkg("rclpy")
        rclpy.init = lambda *a, **k: None
        rclpy.shutdown = lambda *a, **k: None
        rclpy.spin = lambda *a, **k: None
        node_mod = _pkg("rclpy.node")
        clock_mod = _pkg("rclpy.clock")
        clock_mod.Clock = lambda *a, **k: None
        clock_mod.ClockType = SimpleNamespace(STEADY_TIME=0)
        class SetParametersResult:
            def __init__(self, successful=True, reason=""):
                self.successful = successful
                self.reason = reason

        interfaces = _pkg("rcl_interfaces")
        interfaces_msg = _pkg("rcl_interfaces.msg")
        interfaces.msg = interfaces_msg
        interfaces_msg.SetParametersResult = SetParametersResult
        qos = _pkg("rclpy.qos")
        qos.QoSProfile = lambda *a, **k: None
        qos.DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL=1)
        qos.ReliabilityPolicy = SimpleNamespace(RELIABLE=1)

        class Pub:
            def __init__(self):
                self.msgs = []
            def publish(self, msg):
                self.msgs.append(msg)

        class Node:
            def __init__(self, name):
                self.subscriptions = []
                self.services = []
            def add_on_set_parameters_callback(self, fn):
                self._param_cb = fn
            def create_subscription(self, msg_type, topic, callback, qos):
                self.subscriptions.append(topic)
                return None
            def create_publisher(self, *a, **k):
                return Pub()
            def create_service(self, srv_type, name, callback):
                self.services.append(name)
                return None
            def create_timer(self, *a, **k):
                return None
            def get_logger(self):
                return SimpleNamespace(
                    info=lambda *a, **k: None,
                    warn=lambda *a, **k: None,
                )

        node_mod.Node = Node
        geom = _pkg("geometry_msgs")
        geom_msg = _pkg("geometry_msgs.msg")
        geom.msg = geom_msg

        class Twist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

        geom_msg.Twist = Twist
        sensor = _pkg("sensor_msgs")
        sensor_msg = _pkg("sensor_msgs.msg")
        sensor.msg = sensor_msg

        class JointState:
            def __init__(self):
                self.name = []
                self.position = []

        sensor_msg.JointState = JointState
        std = _pkg("std_msgs")
        std_msg = _pkg("std_msgs.msg")
        std.msg = std_msg

        class StringMsg:
            def __init__(self):
                self.data = ""

        class BoolMsg:
            def __init__(self):
                self.data = False

        class Empty:
            pass

        std_msg.String = StringMsg
        std_msg.Bool = BoolMsg
        std_msg.Empty = Empty
        srvs = _pkg("std_srvs")
        srv = _pkg("std_srvs.srv")
        srvs.srv = srv

        class Trigger:
            pass

        srv.Trigger = Trigger

        from omnibot_hybrid.teleop_safety import TeleopSafety

        t = {"n": 0.0}
        node = TeleopSafety(
            clock=lambda: t["n"],
            audit_dir=tempfile.mkdtemp(prefix="teleop-safety-"),
        )
        assert "/cmd_vel/teleop_raw" in node.subscriptions
        assert "/arm/joint_commands/teleop_raw" in node.subscriptions
        assert "/ohho/heartbeat" in node.subscriptions
        assert "/emergency_stop" in node.subscriptions
        assert "/ohho/estop_reset" in node.services
        assert node._state_pub.msgs[-1].data
        assert node._gate.latched is True
        assert node._gate.armed is False

        t["n"] = 0.0
        node._heartbeat_cb(Empty())
        assert node._gate.armed is False
        held = Twist()
        held.linear.x = 0.2
        node._twist_cb(held)
        assert node._twist_pub.msgs[-1].linear.x == 0.0
        startup = SimpleNamespace(success=False, message="")
        node._reset_cb(None, startup)
        assert startup.success is True
        assert node._gate.latched is False
        node._twist_pub.msgs.clear()

        t["n"] = 0.0
        node._heartbeat_cb(Empty())
        t["n"] = 1.0
        node._heartbeat_cb(Empty())
        twist = Twist()
        twist.linear.x = 5.0
        twist.angular.z = 8.0
        node._twist_cb(twist)
        sent = node._twist_pub.msgs[-1]
        assert abs(sent.linear.x - 0.15) < 1e-9
        assert abs(sent.angular.z - 0.6) < 1e-9
        t["n"] = 1.3
        node._heartbeat_cb(Empty())
        node._twist_cb(twist)
        sent = node._twist_pub.msgs[-1]
        assert abs(sent.linear.x - 0.2) < 1e-9
        assert abs(sent.angular.z - 1.0) < 1e-9

        stop = BoolMsg()
        stop.data = True
        node._estop_cb(stop)
        assert node._gate.latched is True
        assert node._twist_pub.msgs[-1].linear.x == 0.0
        stop.data = False
        node._estop_cb(stop)
        assert node._gate.latched is True
        node._twist_pub.msgs.clear()
        node._twist_cb(twist)
        assert node._twist_pub.msgs[-1].linear.x == 0.0

        class Response:
            def __init__(self):
                self.success = False
                self.message = ""

        blocked = Response()
        node._estop_cb(BoolMsg())
        # BoolMsg defaults to false above; re-assert true then try reset.
        stop.data = True
        node._estop_cb(stop)
        node._reset_cb(None, blocked)
        assert blocked.success is False
        assert node._gate.latched is True

        stop.data = False
        node._estop_cb(stop)
        released = Response()
        node._reset_cb(None, released)
        assert released.success is True
        assert node._gate.latched is False

        class Param:
            def __init__(self, name, value):
                self.name = name
                self.value = value

        assert node._on_parameters([Param("max_vx", 2.0)]).successful is False
        assert node._on_parameters([Param("use_sim_time", True)]).successful is False
        """
    _run_stubbed(script)
