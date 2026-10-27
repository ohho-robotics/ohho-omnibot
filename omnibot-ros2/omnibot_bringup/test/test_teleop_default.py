# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Teleop-default gate tests. No ROS 2 install required.

The mux nodes are also imported in a subprocess with stubbed rclpy so the
wrapper wiring is checked on machines that do not have ROS.
"""

import math
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
HYBRID_ROOT = REPO / "omnibot-ros2" / "omnibot_hybrid"
RL_ROOT = REPO / "omnibot-ai-ros2" / "omnibot_rl"
LAUNCH = (
    REPO
    / "omnibot-ros2"
    / "omnibot_bringup"
    / "launch"
    / "mobile_manipulation.launch.py"
)
CMD_VEL_MUX = HYBRID_ROOT / "omnibot_hybrid" / "cmd_vel_mux.py"

sys.path.insert(0, str(HYBRID_ROOT))
sys.path.insert(0, str(RL_ROOT))

from omnibot_hybrid.stream_gate import (  # noqa: E402
    DEFAULT_SOURCE_TIMEOUT_S,
    StreamGate,
    resolve_source_timeout_s,
)
from omnibot_rl.arm_stream_gate import ArmStreamGate  # noqa: E402


def test_launch_starts_in_teleop():
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'parameters=[{"default_mode": "teleop"}]' in text
    assert 'parameters=[rl_arm_params, {"default_mode": "teleop"}]' in text
    assert '"default_mode": "vla"' not in text
    assert '"default_mode": "nav2"' not in text


def test_cmd_vel_mux_parameter_default_is_teleop():
    text = CMD_VEL_MUX.read_text(encoding="utf-8")
    assert 'declare_parameter("default_mode", "teleop")' in text
    assert 'declare_parameter("source_timeout_s", DEFAULT_SOURCE_TIMEOUT_S)' in text
    assert "SOURCE_STALE_TICK_S = 0.05" in text


def test_default_holds_autonomous_sources_and_passes_teleop():
    base = StreamGate()
    arm = ArmStreamGate.from_default_mode("policy")
    assert base.mode == "teleop"
    assert arm.control_mode == "teleop"
    assert base.base_allows("teleop") is True
    for source in ("nav2", "vla", "rl_nav"):
        assert base.base_allows(source) is False
    assert arm.allows("policy") is False
    assert arm.allows("rl_arm") is False
    assert arm.allows("teleop") is True


@pytest.mark.parametrize("mode", ["vla", "nav2", "rl_nav"])
def test_autonomous_mode_releases_matching_streams(mode):
    base = StreamGate()
    arm = ArmStreamGate.from_default_mode("teleop")
    assert base.set_mode(mode) is True
    assert arm.set_control_mode(mode) is True
    assert base.base_allows(mode) is True
    assert base.base_allows("teleop") is False
    assert arm.allows("teleop") is False
    if mode == "rl_nav":
        assert arm.allows("rl_arm") is True
        assert arm.allows("policy") is False
    else:
        assert arm.allows("policy") is True
        assert arm.allows("rl_arm") is False


def test_return_to_teleop_stops_both_streams_within_one_call():
    base = StreamGate()
    arm = ArmStreamGate()
    base.set_mode("vla")
    arm.set_control_mode("vla")
    assert base.base_allows("vla") is True
    assert arm.allows("policy") is True

    assert base.set_mode("teleop") is True
    assert arm.set_control_mode("teleop") is True
    assert base.take_base_stop() is True
    assert base.base_allows("vla") is False
    assert base.base_allows("nav2") is False
    assert base.base_allows("rl_nav") is False
    assert base.base_allows("teleop") is True
    assert arm.allows("policy") is False
    assert arm.allows("rl_arm") is False
    assert arm.allows("teleop") is True
    assert base.take_base_stop() is False


def test_emergency_stop_stops_both_streams_within_one_call():
    base = StreamGate()
    arm = ArmStreamGate()
    base.set_mode("nav2")
    arm.set_control_mode("rl_nav")
    base.set_emergency_stop(True)
    arm.set_emergency_stop(True)
    assert base.take_base_stop() is True
    assert base.base_allows("nav2") is False
    assert base.base_allows("teleop") is False
    assert arm.allows("rl_arm") is False
    assert arm.allows("policy") is False
    assert arm.allows("teleop") is False

    base.set_emergency_stop(False)
    arm.set_emergency_stop(False)
    assert base.base_allows("nav2") is True
    assert arm.allows("rl_arm") is True


def test_unknown_mode_is_ignored():
    base = StreamGate()
    arm = ArmStreamGate()
    assert base.set_mode("VLA") is True
    assert arm.set_control_mode("  NAV2  ") is True
    assert base.set_mode("nope") is False
    assert arm.set_control_mode("smolvla") is False
    assert base.mode == "vla"
    assert arm.control_mode == "nav2"
    assert arm.set_arm_mode("not-a-mode") is False


def test_teleop_arm_forwarded_only_in_teleop_when_estop_clear():
    arm = ArmStreamGate()
    assert arm.allows("teleop") is True
    assert arm.allows("policy") is False

    arm.set_emergency_stop(True)
    assert arm.allows("teleop") is False
    arm.set_emergency_stop(False)
    assert arm.allows("teleop") is True

    for mode in ("vla", "nav2"):
        assert arm.set_control_mode(mode) is True
        assert arm.allows("teleop") is False
        assert arm.allows("policy") is True

    assert arm.set_control_mode("teleop") is True
    assert arm.allows("teleop") is True
    assert arm.allows("policy") is False
    assert arm.allows("rl_arm") is False


def test_arm_cmd_mode_cannot_select_teleop_or_release_policy():
    arm = ArmStreamGate()
    assert arm.set_arm_mode("teleop") is False
    assert arm.arm_mode == "policy"
    assert arm.set_arm_mode("rl_arm") is True
    assert arm.allows("teleop") is True
    assert arm.allows("rl_arm") is False
    assert arm.allows("policy") is False
    assert arm.allows("not-a-source") is False


def test_invalid_default_falls_back_to_teleop():
    assert StreamGate("nav2-now").mode == "teleop"
    held = ArmStreamGate.from_default_mode("policy")
    assert held.control_mode == "teleop"
    assert held.allows("policy") is False


def test_repeated_teleop_does_not_zero_again():
    base = StreamGate("vla")
    base.set_mode("teleop")
    assert base.take_base_stop() is True
    base.set_mode("teleop")
    assert base.take_base_stop() is False


ALL_MODE_TRANSITIONS = [
    (m1, m2)
    for m1 in ("nav2", "vla", "teleop", "rl_nav")
    for m2 in ("nav2", "vla", "teleop", "rl_nav")
    if m1 != m2
]


@pytest.mark.parametrize("from_mode,to_mode", ALL_MODE_TRANSITIONS)
def test_stream_gate_stops_base_on_every_transition_pair(from_mode, to_mode):
    gate = StreamGate(from_mode)
    assert gate.mode == from_mode
    assert gate.take_base_stop() is False

    assert gate.set_mode(to_mode) is True
    assert gate.mode == to_mode
    assert gate.take_base_stop() is True
    assert gate.take_base_stop() is False


@pytest.mark.parametrize("mode", ("nav2", "vla", "teleop", "rl_nav"))
def test_stream_gate_repeated_mode_does_not_zero_again(mode):
    gate = StreamGate(mode)
    assert gate.set_mode(mode) is True
    assert gate.take_base_stop() is False


@pytest.mark.parametrize("mode", ("nav2", "vla", "teleop", "rl_nav"))
@pytest.mark.parametrize("invalid", ("unknown", "invalid", "", "   ", None))
def test_stream_gate_unknown_mode_ignored_and_does_not_zero(mode, invalid):
    gate = StreamGate(mode)
    assert gate.set_mode(invalid) is False
    assert gate.mode == mode
    assert gate.take_base_stop() is False


def _run_stubbed(script):
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(
        [str(HYBRID_ROOT), str(RL_ROOT), env.get("PYTHONPATH", "")]
    )
    completed = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(script)],
        capture_output=True,
        text=True,
        check=False,
        env=env,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr


def test_cmd_vel_mux_wrapper_without_ros():
    _run_stubbed(
        """
        import sys
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

        class Pub:
            def __init__(self):
                self.msgs = []
            def publish(self, msg):
                self.msgs.append(msg)

        class Node:
            def __init__(self, name):
                self._params = {}
            def declare_parameter(self, name, default):
                self._params[name] = default
            def get_parameter(self, name):
                return SimpleNamespace(value=self._params[name])
            def create_subscription(self, *a, **k):
                return None
            def create_publisher(self, *a, **k):
                return Pub()
            def create_timer(self, *a, **k):
                return None
            def get_logger(self):
                return SimpleNamespace(
                    info=lambda *a, **k: None,
                    warn=lambda *a, **k: None,
                )
            def destroy_node(self):
                return None

        node_mod.Node = Node
        geom = _pkg("geometry_msgs")
        geom_msg = _pkg("geometry_msgs.msg")
        geom.msg = geom_msg

        class Twist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

        geom_msg.Twist = Twist
        std = _pkg("std_msgs")
        std_msg = _pkg("std_msgs.msg")
        std.msg = std_msg

        class StringMsg:
            def __init__(self):
                self.data = ""

        class BoolMsg:
            def __init__(self):
                self.data = False

        std_msg.String = StringMsg
        std_msg.Bool = BoolMsg

        from omnibot_hybrid.cmd_vel_mux import CmdVelMux

        node = CmdVelMux()
        assert node._active_mode == "teleop"
        twist = Twist()
        twist.linear.x = 0.5
        node._nav2_cb(twist)
        node._vla_cb(twist)
        node._rl_nav_cb(twist)
        assert node._out_pub.msgs == []
        teleop = Twist()
        teleop.linear.x = 0.2
        node._teleop_cb(teleop)
        assert node._out_pub.msgs[-1].linear.x == 0.2

        mode = StringMsg()
        mode.data = "vla"
        node._mode_cb(mode)
        node._out_pub.msgs.clear()
        node._vla_cb(twist)
        assert node._out_pub.msgs[-1].linear.x == 0.5

        back = StringMsg()
        back.data = "teleop"
        node._mode_cb(back)
        assert node._out_pub.msgs[-1].linear.x == 0.0
        node._out_pub.msgs.clear()
        node._vla_cb(twist)
        assert node._out_pub.msgs == []
        node._teleop_cb(teleop)
        assert len(node._out_pub.msgs) == 1

        stop = BoolMsg()
        stop.data = True
        node._estop_cb(stop)
        assert node._out_pub.msgs[-1].linear.x == 0.0
        node._out_pub.msgs.clear()
        node._teleop_cb(teleop)
        assert node._out_pub.msgs == []
        """
    )


def test_arm_cmd_mux_wrapper_without_ros():
    _run_stubbed(
        """
        import sys
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

        class Pub:
            def __init__(self):
                self.msgs = []
            def publish(self, msg):
                self.msgs.append(msg)

        class Node:
            def __init__(self, name):
                self._params = {}
                self.subscriptions = []
                self.warnings = []
            def declare_parameter(self, name, default):
                self._params[name] = default
            def get_parameter(self, name):
                return SimpleNamespace(value=self._params[name])
            def create_subscription(self, msg_type, topic, callback, qos):
                self.subscriptions.append(topic)
                return None
            def create_publisher(self, *a, **k):
                return Pub()
            def create_timer(self, *a, **k):
                return None
            def get_logger(self):
                warnings = self.warnings
                return SimpleNamespace(
                    info=lambda *a, **k: None,
                    warn=lambda message, *a, **k: warnings.append(message),
                )
            def destroy_node(self):
                return None

        node_mod.Node = Node
        sensor = _pkg("sensor_msgs")
        sensor_msg = _pkg("sensor_msgs.msg")
        sensor.msg = sensor_msg

        class JointState:
            def __init__(self):
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

        std_msg.String = StringMsg
        std_msg.Bool = BoolMsg

        from omnibot_rl.arm_cmd_mux import ArmCmdMux
        from omnibot_rl.arm_stream_gate import (
            TOPIC_JOINT_COMMANDS_POLICY,
            TOPIC_JOINT_COMMANDS_POLICY_ALIAS,
            TOPIC_JOINT_COMMANDS_RL,
            TOPIC_JOINT_COMMANDS_TELEOP,
        )

        node = ArmCmdMux()
        assert node._gate.control_mode == "teleop"
        assert TOPIC_JOINT_COMMANDS_TELEOP in node.subscriptions
        assert TOPIC_JOINT_COMMANDS_POLICY in node.subscriptions
        assert TOPIC_JOINT_COMMANDS_POLICY_ALIAS in node.subscriptions
        assert TOPIC_JOINT_COMMANDS_RL in node.subscriptions
        cmd = JointState()
        cmd.position = [0.1]
        node._teleop_cb(cmd)
        assert node._out_pub.msgs[-1].position == [0.1]
        node._out_pub.msgs.clear()
        node._policy_cb(cmd)
        node._policy_alias_cb(cmd)
        node._rl_arm_cb(cmd)
        assert node._out_pub.msgs == []
        assert len(node.warnings) == 1

        mode = StringMsg()
        mode.data = "vla"
        node._control_mode_cb(mode)
        node._teleop_cb(cmd)
        assert node._out_pub.msgs == []
        node._policy_cb(cmd)
        assert len(node._out_pub.msgs) == 1
        node._policy_alias_cb(cmd)
        assert len(node._out_pub.msgs) == 2
        assert len(node.warnings) == 1
        node._rl_arm_cb(cmd)
        assert len(node._out_pub.msgs) == 2

        nav = StringMsg()
        nav.data = "nav2"
        node._control_mode_cb(nav)
        node._out_pub.msgs.clear()
        node._teleop_cb(cmd)
        assert node._out_pub.msgs == []
        node._policy_cb(cmd)
        assert len(node._out_pub.msgs) == 1

        back = StringMsg()
        back.data = "teleop"
        node._control_mode_cb(back)
        node._out_pub.msgs.clear()
        node._policy_cb(cmd)
        node._policy_alias_cb(cmd)
        node._rl_arm_cb(cmd)
        assert node._out_pub.msgs == []
        node._teleop_cb(cmd)
        assert len(node._out_pub.msgs) == 1

        mode.data = "rl_nav"
        node._control_mode_cb(mode)
        node._out_pub.msgs.clear()
        node._teleop_cb(cmd)
        assert node._out_pub.msgs == []
        node._rl_arm_cb(cmd)
        assert len(node._out_pub.msgs) == 1
        stop = BoolMsg()
        stop.data = True
        node._estop_cb(stop)
        node._out_pub.msgs.clear()
        node._rl_arm_cb(cmd)
        node._policy_cb(cmd)
        node._teleop_cb(cmd)
        assert node._out_pub.msgs == []

        clear = BoolMsg()
        clear.data = False
        node._estop_cb(clear)
        node._control_mode_cb(back)
        node._out_pub.msgs.clear()
        node._teleop_cb(cmd)
        assert len(node._out_pub.msgs) == 1
        stop.data = True
        node._estop_cb(stop)
        node._out_pub.msgs.clear()
        node._teleop_cb(cmd)
        assert node._out_pub.msgs == []
        """
    )


def test_cmd_vel_mux_mode_switch_zeros_output_without_ros():
    _run_stubbed(
        """
        import sys
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

        class Pub:
            def __init__(self):
                self.msgs = []
            def publish(self, msg):
                self.msgs.append(msg)

        class Node:
            def __init__(self, name):
                self._params = {}
            def declare_parameter(self, name, default):
                self._params[name] = default
            def get_parameter(self, name):
                return SimpleNamespace(value=self._params[name])
            def create_subscription(self, *a, **k):
                return None
            def create_publisher(self, *a, **k):
                return Pub()
            def create_timer(self, *a, **k):
                return None
            def get_logger(self):
                return SimpleNamespace(
                    info=lambda *a, **k: None,
                    warn=lambda *a, **k: None,
                )
            def destroy_node(self):
                return None

        node_mod.Node = Node
        geom = _pkg("geometry_msgs")
        geom_msg = _pkg("geometry_msgs.msg")
        geom.msg = geom_msg

        class Twist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

        geom_msg.Twist = Twist
        std = _pkg("std_msgs")
        std_msg = _pkg("std_msgs.msg")
        std.msg = std_msg

        class StringMsg:
            def __init__(self):
                self.data = ""

        std_msg.String = StringMsg
        std_msg.Bool = SimpleNamespace

        from omnibot_hybrid.cmd_vel_mux import CmdVelMux

        node = CmdVelMux()
        modes = ["teleop", "vla", "nav2", "rl_nav"]

        # 1. Every transition pair (m1 -> m2) publishes exactly one zero Twist
        for m1 in modes:
            for m2 in modes:
                if m1 == m2:
                    continue
                node._gate.mode = m1
                node._gate._base_stop = False
                node._out_pub.msgs.clear()

                msg = StringMsg()
                msg.data = m2
                node._mode_cb(msg)

                assert len(node._out_pub.msgs) == 1, (
                    f"Expected 1 msg on {m1} -> {m2}, got {len(node._out_pub.msgs)}"
                )
                zero = node._out_pub.msgs[0]
                assert zero.linear.x == 0.0
                assert zero.linear.y == 0.0
                assert zero.angular.z == 0.0

        # 2. Repeated same mode must NOT publish zero Twist
        for m in modes:
            node._gate.mode = m
            node._gate._base_stop = False
            node._out_pub.msgs.clear()

            msg = StringMsg()
            msg.data = m
            node._mode_cb(msg)

            assert len(node._out_pub.msgs) == 0, (
                f"Repeated mode {m} should not publish zero"
            )

        # 3. Unknown mode must NOT publish zero Twist and must not change mode
        node._gate.mode = "teleop"
        node._gate._base_stop = False
        node._out_pub.msgs.clear()
        bad = StringMsg()
        bad.data = "nonexistent"
        node._mode_cb(bad)
        assert len(node._out_pub.msgs) == 0
        assert node._active_mode == "teleop"
        """
    )


def test_source_timeout_defaults_to_0_3_and_cannot_be_raised_or_disabled():
    assert DEFAULT_SOURCE_TIMEOUT_S == pytest.approx(0.3)
    kept, warning = resolve_source_timeout_s(0.3)
    assert kept == pytest.approx(0.3)
    assert warning is None

    short = StreamGate("teleop", source_timeout_s=0.1)
    assert short.source_timeout_s == pytest.approx(0.1)
    assert short.source_timeout_warning is None
    short.note_forward("teleop", 0.0)
    assert short.poll_staleness(math.nextafter(0.1, 0.0)) is False
    assert short.poll_staleness(0.1) is True
    assert short.poll_staleness(1.0) is False

    clamped, high_warning = resolve_source_timeout_s(2.0)
    assert clamped == pytest.approx(0.3)
    assert high_warning is not None and "clamped" in high_warning
    high = StreamGate("vla", source_timeout_s=2.0)
    assert high.source_timeout_s == pytest.approx(0.3)
    high.note_forward("vla", 0.0)
    assert high.poll_staleness(math.nextafter(0.3, 0.0)) is False
    assert high.poll_staleness(0.3) is True


@pytest.mark.parametrize(
    "raw",
    [0, 0.0, -0.5, -1, None, "0.1", True, False, float("nan"), float("inf")],
)
def test_nonpositive_source_timeout_stays_armed(raw):
    gate = StreamGate("nav2", source_timeout_s=raw)
    assert gate.source_timeout_s == pytest.approx(0.3)
    assert gate.source_timeout_warning
    gate.note_forward("nav2", 0.0)
    assert gate.poll_staleness(math.nextafter(0.3, 0.0)) is False
    assert gate.poll_staleness(0.3) is True
    assert gate.poll_staleness(5.0) is False


@pytest.mark.parametrize("mode", ["teleop", "nav2", "vla", "rl_nav"])
def test_stream_gate_zeros_once_within_timeout_for_active_source(mode):
    gate = StreamGate("teleop")
    assert gate.set_mode(mode) is True
    gate.note_forward(mode, 0.0)
    timeout = gate.source_timeout_s
    assert gate.poll_staleness(math.nextafter(timeout, 0.0)) is False
    assert gate.poll_staleness(timeout) is True
    assert gate.poll_staleness(timeout + 5.0) is False
    gate.note_forward(mode, 1.0)
    assert gate.poll_staleness(1.0 + math.nextafter(timeout, 0.0)) is False
    assert gate.poll_staleness(1.0 + timeout) is True


def test_inactive_source_does_not_refresh_stream_gate_timeout():
    gate = StreamGate("teleop")
    gate.note_forward("teleop", 0.0)
    gate.note_forward("nav2", 0.2)
    gate.note_forward("vla", 0.2)
    assert gate.poll_staleness(math.nextafter(0.3, 0.0)) is False
    assert gate.poll_staleness(0.3) is True


def test_cmd_vel_mux_source_timeout_zeros_once_without_ros():
    """Mux wiring without rclpy: teleop, nav2, and vla zero within one tick."""
    _run_stubbed(
        """
        import math
        import sys
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

        class Pub:
            def __init__(self):
                self.msgs = []
            def publish(self, msg):
                self.msgs.append(msg)

        class Logger:
            def __init__(self):
                self.warnings = []
            def info(self, *a, **k):
                return None
            def warn(self, msg, *a, **k):
                self.warnings.append(msg)

        class Node:
            timeout_override = None
            def __init__(self, name):
                self._params = {}
                self.logger = Logger()
                self.timers = []
            def declare_parameter(self, name, default):
                self._params[name] = default
            def get_parameter(self, name):
                if name == "source_timeout_s" and Node.timeout_override is not None:
                    return SimpleNamespace(value=Node.timeout_override)
                return SimpleNamespace(value=self._params[name])
            def create_subscription(self, *a, **k):
                return None
            def create_publisher(self, *a, **k):
                return Pub()
            def create_timer(self, period, callback):
                self.timers.append((period, callback))
                return None
            def get_logger(self):
                return self.logger
            def destroy_node(self):
                return None

        node_mod.Node = Node
        geom = _pkg("geometry_msgs")
        geom_msg = _pkg("geometry_msgs.msg")
        geom.msg = geom_msg

        class Twist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

        geom_msg.Twist = Twist
        std = _pkg("std_msgs")
        std_msg = _pkg("std_msgs.msg")
        std.msg = std_msg

        class StringMsg:
            def __init__(self):
                self.data = ""

        std_msg.String = StringMsg
        std_msg.Bool = SimpleNamespace

        from omnibot_hybrid.cmd_vel_mux import CmdVelMux

        def zero(msg):
            assert msg.linear.x == 0.0
            assert msg.linear.y == 0.0
            assert msg.linear.z == 0.0
            assert msg.angular.x == 0.0
            assert msg.angular.y == 0.0
            assert msg.angular.z == 0.0

        def drive(node, mode, send):
            clock = {"t": 0.0}
            node._now = lambda: clock["t"]
            msg = StringMsg()
            msg.data = mode
            node._mode_cb(msg)
            node._out_pub.msgs.clear()
            twist = Twist()
            twist.linear.x = 0.55
            send(twist)
            assert node._out_pub.msgs[-1].linear.x == 0.55
            node._out_pub.msgs.clear()
            timeout = node._gate.source_timeout_s
            tick = node.SOURCE_STALE_TICK_S
            just_before = math.nextafter(timeout, 0.0)
            clock["t"] = just_before
            node._check_source_staleness()
            assert node._out_pub.msgs == [], mode
            clock["t"] = just_before + tick
            node._check_source_staleness()
            assert len(node._out_pub.msgs) == 1, mode
            zero(node._out_pub.msgs[0])
            assert clock["t"] <= timeout + tick
            node._out_pub.msgs.clear()
            clock["t"] = timeout + 5.0
            node._check_source_staleness()
            assert node._out_pub.msgs == [], mode
            clock["t"] = 1.0
            twist.linear.x = 0.25
            send(twist)
            assert node._out_pub.msgs[-1].linear.x == 0.25
            node._out_pub.msgs.clear()
            clock["t"] = 1.0 + math.nextafter(timeout, 0.0)
            node._check_source_staleness()
            assert node._out_pub.msgs == [], mode
            clock["t"] = 1.0 + timeout
            node._check_source_staleness()
            assert len(node._out_pub.msgs) == 1, mode
            zero(node._out_pub.msgs[0])
            assert (clock["t"] - 1.0) <= timeout + tick

        node = CmdVelMux()
        assert node._gate.source_timeout_s == 0.3
        assert node.logger.warnings == []
        assert any(period == node.SOURCE_STALE_TICK_S for period, _cb in node.timers)
        drive(node, "teleop", node._teleop_cb)
        drive(node, "nav2", node._nav2_cb)
        drive(node, "vla", node._vla_cb)
        drive(node, "rl_nav", node._rl_nav_cb)

        quiet = CmdVelMux()
        clock = {"t": 0.0}
        quiet._now = lambda: clock["t"]
        quiet._out_pub.msgs.clear()
        clock["t"] = 10.0
        quiet._check_source_staleness()
        assert quiet._out_pub.msgs == []

        fresh = CmdVelMux()
        clock = {"t": 0.0}
        fresh._now = lambda: clock["t"]
        twist = Twist()
        twist.linear.x = 0.4
        fresh._teleop_cb(twist)
        fresh._out_pub.msgs.clear()
        clock["t"] = math.nextafter(0.3, 0.0)
        other = Twist()
        other.linear.x = 0.9
        fresh._nav2_cb(other)
        fresh._vla_cb(other)
        fresh._check_source_staleness()
        assert fresh._out_pub.msgs == []
        clock["t"] = 0.3
        fresh._check_source_staleness()
        assert len(fresh._out_pub.msgs) == 1
        zero(fresh._out_pub.msgs[0])

        Node.timeout_override = 5.0
        high = CmdVelMux()
        assert high._gate.source_timeout_s == 0.3
        assert any("clamped" in item for item in high.logger.warnings)

        Node.timeout_override = 0.0
        disabled = CmdVelMux()
        assert disabled._gate.source_timeout_s == 0.3
        assert any("cannot be disabled" in item for item in disabled.logger.warnings)

        Node.timeout_override = -1
        negative = CmdVelMux()
        assert negative._gate.source_timeout_s == 0.3
        assert any("cannot be disabled" in item for item in negative.logger.warnings)

        Node.timeout_override = 0.1
        lowered = CmdVelMux()
        assert lowered._gate.source_timeout_s == 0.1
        assert lowered.logger.warnings == []
        clock = {"t": 0.0}
        lowered._now = lambda: clock["t"]
        twist = Twist()
        twist.linear.x = 0.2
        lowered._teleop_cb(twist)
        lowered._out_pub.msgs.clear()
        clock["t"] = math.nextafter(0.1, 0.0)
        lowered._check_source_staleness()
        assert lowered._out_pub.msgs == []
        clock["t"] = math.nextafter(0.1, 0.0) + lowered.SOURCE_STALE_TICK_S
        lowered._check_source_staleness()
        assert len(lowered._out_pub.msgs) == 1
        zero(lowered._out_pub.msgs[0])
        assert clock["t"] <= 0.1 + lowered.SOURCE_STALE_TICK_S
        """
    )
