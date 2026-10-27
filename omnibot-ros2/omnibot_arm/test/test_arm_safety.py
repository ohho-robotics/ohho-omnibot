"""
Safety decisions for the SO-101 arm: e-stop torque, command timeout, clamps.

No hardware and no ROS graph. ArmSafety is pure Python. A second group
stubs rclpy only long enough to import arm_driver_node and check that the
node writes Torque_Enable, holds a stale goal, and logs the hold.
"""

import os
import sys
import types
from unittest.mock import MagicMock

import pytest

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, _SCRIPTS)

import arm_safety  # noqa: E402 — scripts path must come first
import servo_offsets  # noqa: E402

@pytest.fixture(autouse=True)
def _no_calibration_env(monkeypatch):
    """Driver tests must not pick up a calibration path from the machine."""
    monkeypatch.delenv("OMNIBOT_ARM_CALIBRATION", raising=False)
    monkeypatch.delenv("OMNIBOT_ARM_ROBOT_ID", raising=False)
    monkeypatch.delenv("OMNIBOT_ARM_CALIBRATION_DIR", raising=False)


JOINT_MIN = [-3.14, -1.57, -1.57, -1.57, -3.14, -0.1]
JOINT_MAX = [3.14, 1.57, 1.57, 1.57, 3.14, 0.8]
ZEROS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def _safety(**kwargs):
    return arm_safety.ArmSafety(JOINT_MIN, JOINT_MAX, **kwargs)


def _install_ros_stubs():
    """Bind arm_driver_node to fakes. Restore sys.modules before return."""
    saved = {}

    def _module(name):
        saved[name] = sys.modules.get(name)
        module = types.ModuleType(name)
        sys.modules[name] = module
        return module

    class _Param:
        def __init__(self, value):
            self.value = value

    class _Logger:
        def __init__(self):
            self.warnings = []
            self.infos = []

        def warn(self, message, **_kwargs):
            self.warnings.append(message)

        def info(self, message, **_kwargs):
            self.infos.append(message)

        def error(self, message, **_kwargs):
            self.warnings.append(message)

    class _Node:
        def __init__(self, name):
            self._name = name
            self._params = {}
            self._logger = _Logger()
            self._subs = []

        def declare_parameter(self, name, default=None):
            self._params[name] = default

        def get_parameter(self, name):
            return _Param(self._params[name])

        def get_logger(self):
            return self._logger

        def get_clock(self):
            clock = MagicMock()
            clock.now.return_value.to_msg.return_value = object()
            return clock

        def create_publisher(self, *_args, **_kwargs):
            return MagicMock()

        def create_subscription(self, _msg_type, topic, callback, _qos):
            self._subs.append((topic, callback))
            return MagicMock()

        def create_timer(self, *_args, **_kwargs):
            return MagicMock()

        def set_parameters(self, params):
            for param in params:
                self._params[param.name] = param.value
            return []

        def destroy_node(self):
            return None

    class JointState:
        def __init__(self):
            self.name = []
            self.position = []
            self.header = types.SimpleNamespace(stamp=None)

    class Bool:
        def __init__(self):
            self.data = False

    rclpy = _module("rclpy")
    rclpy_node = _module("rclpy.node")
    rclpy.node = rclpy_node
    rclpy_node.Node = _Node
    rclpy.init = lambda *_a, **_k: None
    rclpy.shutdown = lambda *_a, **_k: None
    rclpy.spin = lambda *_a, **_k: None

    sensor_msgs = _module("sensor_msgs")
    sensor_msgs_msg = _module("sensor_msgs.msg")
    sensor_msgs.msg = sensor_msgs_msg
    sensor_msgs_msg.JointState = JointState

    std_msgs = _module("std_msgs")
    std_msgs_msg = _module("std_msgs.msg")
    std_msgs.msg = std_msgs_msg
    std_msgs_msg.Bool = Bool

    parameter_mod = _module("rclpy.parameter")
    rclpy.parameter = parameter_mod

    class Parameter:
        class Type:
            INTEGER_ARRAY = 7

        def __init__(self, name, type_, value):
            self.name = name
            self.type_ = type_
            self.value = value

    parameter_mod.Parameter = Parameter

    try:
        import arm_driver_node  # noqa: E402
    finally:
        for name, previous in saved.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous
    return arm_driver_node


# Installed once. sys.modules is restored so later tests still see a missing
# rclpy (or the real one, if this file is collected under ROS).
arm_driver_node = _install_ros_stubs()


def _driver():
    return arm_driver_node.ArmDriverNode()


def _bool(value):
    msg = arm_driver_node.Bool()
    msg.data = value
    return msg


def _joints(node, positions):
    msg = arm_driver_node.JointState()
    msg.name = list(node.joint_names)
    msg.position = list(positions)
    return msg


# ---------------------------------------------------------------------------
# Named limit (OHH-100 reuses this constant)
# ---------------------------------------------------------------------------


class TestNamedLimit:
    def test_cycle_limit_constant(self):
        assert arm_safety.MAX_JOINT_DELTA_RAD == 0.15

    def test_timeout_constant(self):
        assert arm_safety.COMMAND_TIMEOUT_SEC == 0.2

    def test_default_matches_constant(self):
        safety = _safety()
        assert safety.max_joint_delta_rad == arm_safety.MAX_JOINT_DELTA_RAD
        assert safety.command_timeout_sec == arm_safety.COMMAND_TIMEOUT_SEC

    def test_yaml_exposes_the_same_names(self):
        path = os.path.join(_SCRIPTS, "..", "config", "arm_params.yaml")
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
        assert "max_joint_delta_rad: 0.15" in text
        assert "command_timeout_sec: 0.2" in text


# ---------------------------------------------------------------------------
# Clamp: joint limits and max delta per cycle
# ---------------------------------------------------------------------------


class TestClamp:
    def test_within_limits_and_delta_unchanged(self):
        safety = _safety()
        action = safety.accept_command(
            [0.1, -0.1, 0.0, 0.0, 0.0, 0.0], 0.0, ZEROS
        )
        assert action.write is True
        assert action.positions[0] == pytest.approx(0.1)
        assert action.positions[1] == pytest.approx(-0.1)

    def test_delta_limited_to_0_15_per_cycle(self):
        safety = _safety()
        first = safety.accept_command([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, ZEROS)
        assert first.positions[0] == pytest.approx(0.15)
        second = safety.accept_command(
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.01, ZEROS
        )
        assert second.positions[0] == pytest.approx(0.30)
        assert second.positions[0] - first.positions[0] == pytest.approx(0.15)

    def test_negative_delta_limited(self):
        safety = _safety()
        action = safety.accept_command(
            [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, ZEROS
        )
        assert action.positions[0] == pytest.approx(-0.15)

    def test_exact_delta_not_reduced(self):
        safety = _safety()
        action = safety.accept_command(
            [0.15, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, ZEROS
        )
        assert action.positions[0] == pytest.approx(0.15)

    def test_above_max_does_not_jump_past_the_cycle_limit(self):
        safety = _safety()
        action = safety.accept_command([99.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, ZEROS)
        assert action.positions[0] == pytest.approx(0.15)
        assert action.positions[0] <= JOINT_MAX[0]

    def test_result_near_limit_stays_inside(self):
        safety = _safety()
        reference = [3.10, 0.0, 0.0, 0.0, 0.0, 0.0]
        action = safety.accept_command(
            [99.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, reference
        )
        assert action.positions[0] == pytest.approx(3.14)
        assert action.positions[0] <= JOINT_MAX[0]

    def test_below_min_steps_toward_min(self):
        safety = _safety()
        action = safety.accept_command(
            [-99.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, ZEROS
        )
        assert action.positions[0] == pytest.approx(-0.15)
        assert action.positions[0] >= JOINT_MIN[0]

    def test_gripper_limit_and_delta(self):
        safety = _safety()
        action = safety.accept_command(
            [0.0, 0.0, 0.0, 0.0, 0.0, 99.0], 0.0, ZEROS
        )
        assert action.positions[5] == pytest.approx(0.15)
        assert action.positions[5] <= JOINT_MAX[5]

    def test_joints_are_independent(self):
        safety = _safety()
        action = safety.accept_command(
            [1.0, -1.0, 0.05, 0.0, 0.0, 0.0], 0.0, ZEROS
        )
        assert action.positions[0] == pytest.approx(0.15)
        assert action.positions[1] == pytest.approx(-0.15)
        assert action.positions[2] == pytest.approx(0.05)

    def test_custom_delta_parameter(self):
        safety = _safety(max_joint_delta_rad=0.5)
        action = safety.accept_command([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, ZEROS)
        assert action.positions[0] == pytest.approx(0.5)

    def test_limits_only_when_reference_is_missing(self):
        clamped = arm_safety.clamp_joint_command(
            [99.0, 0.0, 0.0, 0.0, 0.0, -99.0],
            JOINT_MIN,
            JOINT_MAX,
            None,
            arm_safety.MAX_JOINT_DELTA_RAD,
        )
        assert clamped[0] == pytest.approx(JOINT_MAX[0])
        assert clamped[5] == pytest.approx(JOINT_MIN[5])


# ---------------------------------------------------------------------------
# Command timeout hold
# ---------------------------------------------------------------------------


class TestTimeoutHold:
    def test_silence_under_timeout_does_not_hold(self):
        safety = _safety()
        safety.accept_command(ZEROS, 0.0, ZEROS)
        hold = safety.hold_if_stale(0.199)
        assert hold.rewrite is False
        assert hold.message is None

    def test_timeout_holds_last_command_and_logs_once(self):
        safety = _safety()
        accepted = safety.accept_command(
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, ZEROS
        )
        hold = safety.hold_if_stale(0.2)
        assert hold.rewrite is True
        assert hold.positions == pytest.approx(accepted.positions)
        assert "holding last command" in hold.message
        assert "0.200s" in hold.message

        again = safety.hold_if_stale(1.0)
        assert again.rewrite is False
        assert again.message is None
        assert again.positions == pytest.approx(accepted.positions)

    def test_new_command_then_another_silence_logs_again(self):
        safety = _safety()
        safety.accept_command(ZEROS, 0.0, ZEROS)
        assert safety.hold_if_stale(0.2).message is not None
        safety.accept_command([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], 0.3, ZEROS)
        quiet = safety.hold_if_stale(0.5)
        assert quiet.rewrite is True
        assert "holding last command" in quiet.message
        assert quiet.positions[0] == pytest.approx(0.1)

    def test_no_command_yet_does_not_hold(self):
        safety = _safety()
        hold = safety.hold_if_stale(10.0)
        assert hold.rewrite is False
        assert hold.message is None

    def test_custom_timeout_parameter(self):
        safety = _safety(command_timeout_sec=0.05)
        safety.accept_command(ZEROS, 1.0, ZEROS)
        assert safety.hold_if_stale(1.049).rewrite is False
        assert safety.hold_if_stale(1.05).rewrite is True

    def test_stale_goal_snap_regression_torque_off_drops_target(self):
        safety = _safety()
        safety.accept_command(ZEROS, 0.0, ZEROS)
        assert safety.last_command is not None

        # Torque off drops target and timestamp
        safety.on_emergency_stop(True)
        assert safety.last_command is None
        assert safety.last_command_time is None

        # Re-enabling torque without new command does not hold stale pre-drop target
        safety.on_emergency_stop(False)
        safety.on_arm_enable(True)
        hold = safety.hold_if_stale(1.0)
        assert hold.rewrite is False
        assert hold.positions is None

        # Disabling torque via arm_enable(False) also drops target
        safety.accept_command(ZEROS, 1.1, ZEROS)
        assert safety.last_command is not None
        safety.on_arm_enable(False)
        assert safety.last_command is None
        assert safety.last_command_time is None


# ---------------------------------------------------------------------------
# Emergency stop torque latch
# ---------------------------------------------------------------------------


class TestEmergencyStopTorque:
    def test_estop_disables_torque(self):
        safety = _safety()
        action = safety.on_emergency_stop(True)
        assert action.value == 0
        assert safety.torque_enabled is False
        assert "Torque_Enable 0" in action.message

    def test_repeated_estop_stays_disabled(self):
        safety = _safety()
        safety.on_emergency_stop(True)
        action = safety.on_emergency_stop(True)
        assert action.value == 0
        assert safety.torque_enabled is False

    def test_clear_does_not_reenable_until_arm_enable(self):
        safety = _safety()
        safety.on_emergency_stop(True)
        cleared = safety.on_emergency_stop(False)
        assert cleared.value == 0
        assert safety.torque_enabled is False
        assert safety.emergency_stop is False

        still = safety.on_arm_enable(False)
        assert still.value == 0
        assert safety.torque_enabled is False

        enabled = safety.on_arm_enable(True)
        assert enabled.value == 1
        assert safety.torque_enabled is True

    def test_enable_during_estop_is_not_remembered(self):
        safety = _safety()
        safety.on_arm_enable(True)
        safety.on_emergency_stop(True)
        during = safety.on_arm_enable(True)
        assert during.value == 0
        assert safety.torque_enabled is False
        safety.on_emergency_stop(False)
        assert safety.torque_enabled is False
        assert safety.on_arm_enable(True).value == 1

    def test_command_during_estop_does_not_replace_last_target(self):
        safety = _safety()
        safety.accept_command(ZEROS, 0.0, ZEROS)
        safety.on_emergency_stop(True)
        assert safety.last_command is None
        rejected = safety.accept_command(
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.1, ZEROS
        )
        assert rejected.write is False
        assert safety.last_command is None
        safety.on_emergency_stop(False)
        safety.on_arm_enable(True)
        stepped = safety.accept_command(
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.2, ZEROS
        )
        assert stepped.positions[0] == pytest.approx(0.15)


# ---------------------------------------------------------------------------
# Driver node wiring (stubbed rclpy, no hardware)
# ---------------------------------------------------------------------------


class TestDriverWiring:
    def test_parameters_and_subscriptions(self):
        node = _driver()
        assert node._params["max_joint_delta_rad"] == arm_safety.MAX_JOINT_DELTA_RAD
        assert node._params["command_timeout_sec"] == arm_safety.COMMAND_TIMEOUT_SEC
        topics = [topic for topic, _callback in node._subs]
        assert "/emergency_stop" in topics
        assert "/arm/enable" in topics
        assert "/arm/joint_commands/out" in topics

    def test_estop_writes_torque_enable_zero(self):
        node = _driver()
        bus = MagicMock()
        node.follower_bus = bus
        node.emergency_stop_cb(_bool(True))
        register, values = bus.write.call_args[0]
        assert register == "Torque_Enable"
        assert values == {name: 0 for name in node.joint_names}
        assert node.torque_enabled is False

    def test_clear_does_not_write_torque_enable_one(self):
        node = _driver()
        bus = MagicMock()
        node.follower_bus = bus
        node.emergency_stop_cb(_bool(True))
        node.enable_cb(_bool(True))
        assert node.torque_enabled is False
        register, values = bus.write.call_args[0]
        assert register == "Torque_Enable"
        assert all(value == 0 for value in values.values())

        bus.write.reset_mock()
        node.emergency_stop_cb(_bool(False))
        assert node.torque_enabled is False
        register, values = bus.write.call_args[0]
        assert register == "Torque_Enable"
        assert all(value == 0 for value in values.values())

        node.enable_cb(_bool(True))
        torque_call = [c for c in bus.write.call_args_list if c[0][0] == "Torque_Enable"][-1]
        register, values = torque_call[0]
        assert register == "Torque_Enable"
        assert values == {name: 1 for name in node.joint_names}
        assert node.torque_enabled is True

    def test_node_clamps_delta_without_hardware(self):
        node = _driver()
        node.joint_command_cb(
            _joints(node, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        )
        assert node.sim_positions[0] == pytest.approx(0.15)
        node.joint_command_cb(
            _joints(node, [99.0, 0.0, 0.0, 0.0, 0.0, -99.0])
        )
        assert node.sim_positions[0] == pytest.approx(0.30)
        assert node.sim_positions[5] == pytest.approx(-0.1)

    def test_node_holds_and_logs_on_timeout(self, monkeypatch):
        clock = {"now": 10.0}
        monkeypatch.setattr(
            arm_driver_node.time, "monotonic", lambda: clock["now"]
        )
        node = _driver()
        bus = MagicMock()
        bus.read.side_effect = RuntimeError("no hardware")
        node.follower_bus = bus
        node.joint_command_cb(_joints(node, [0.0] * 6))
        bus.write.reset_mock()
        node.get_logger().warnings.clear()

        # 10.25 is safely past the 0.2 s timeout. 10.2 - 10.0 is a hair under
        # 0.2 in binary floats, so it must not be used as the boundary.
        clock["now"] = 10.25
        node.publish_states()
        assert any(
            "holding last command" in message
            for message in node.get_logger().warnings
        )
        register, _values = bus.write.call_args[0]
        assert register == "Goal_Position"

        writes = bus.write.call_count
        node.publish_states()
        assert bus.write.call_count == writes
        holds = [
            message
            for message in node.get_logger().warnings
            if "holding last command" in message
        ]
        assert len(holds) == 1

    def test_command_rejected_while_estop(self):
        node = _driver()
        node.emergency_stop_cb(_bool(True))
        node.joint_command_cb(_joints(node, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        assert node.sim_positions == pytest.approx([0.0] * 6)

    def test_torque_on_reseeds_from_measured_pose(self):
        node = _driver()
        bus = MagicMock()
        node.follower_bus = bus

        # Measured positions from bus indicate arm sagged while limp
        sagged_ticks = [2048 + 512, 2048 - 512, 2048 + 256, 2048, 2048, 2048]
        sagged_rads = node.ticks_to_radians(sagged_ticks)
        bus.read.return_value = dict(zip(node.joint_names, sagged_ticks))

        # Re-enable torque
        node.enable_cb(_bool(True))

        # Writes Torque_Enable 1 then Goal_Position equals measured pose
        assert bus.write.call_count == 2
        call1, call2 = bus.write.call_args_list
        assert call1[0][0] == "Torque_Enable"
        assert call1[0][1] == {name: 1 for name in node.joint_names}
        assert call2[0][0] == "Goal_Position"
        assert call2[0][1] == dict(zip(node.joint_names, sagged_ticks))
        assert node._safety.last_command == pytest.approx(sagged_rads)

        # Next joint command steps from sagged pose, not 0.0 or pre-drop target
        target = list(sagged_rads)
        target[0] += 1.0  # Big jump
        bus.write.reset_mock()
        node.joint_command_cb(_joints(node, target))
        assert bus.write.call_count == 1
        goal_call = bus.write.call_args[0]
        assert goal_call[0] == "Goal_Position"
        # Only stepped by 0.15 rad from sagged_rads[0] (allowing tick quantization)
        written_ticks = [goal_call[1][name] for name in node.joint_names]
        written_rads = node.ticks_to_radians(written_ticks)
        assert written_rads[0] == pytest.approx(sagged_rads[0] + 0.15, abs=0.005)

    def test_torque_on_read_failure_skips_goal_and_logs(self):
        node = _driver()
        bus = MagicMock()
        bus.read.side_effect = RuntimeError("Bus read error")
        node.follower_bus = bus

        node.enable_cb(_bool(True))

        # Wrote Torque_Enable but skipped Goal_Position
        assert bus.write.call_count == 1
        assert bus.write.call_args[0][0] == "Torque_Enable"
        assert node._safety.last_command is None
        assert any("Cannot read present position" in msg for msg in node.get_logger().warnings)


_FIXTURE_YAML = os.path.join(
    os.path.dirname(__file__), "fixtures", "so101_ci_fixture.yaml"
)
_FIXTURE_TICKS = [100, 200, 300, 400, 500, 600]


class TestHomeTickLoading:
    def test_factory_default_warns(self):
        node = _driver()
        assert node.home_ticks == [2048, 2048, 2048, 2048, 2048, 2048]
        assert node.home_ticks_source == "parameter"
        assert any("NOT this robot" in message for message in node.get_logger().warnings)

    def test_env_file_is_loaded(self, monkeypatch):
        monkeypatch.setenv("OMNIBOT_ARM_CALIBRATION", _FIXTURE_YAML)
        node = _driver()
        assert node.home_ticks == _FIXTURE_TICKS
        assert node.home_ticks_source == "OMNIBOT_ARM_CALIBRATION"
        assert node.get_parameter("home_ticks").value == _FIXTURE_TICKS
        assert not any(
            "NOT this robot" in message for message in node.get_logger().warnings
        )

    def test_missing_robot_id_raises_before_the_node_returns(self, monkeypatch):
        monkeypatch.setenv("OMNIBOT_ARM_ROBOT_ID", "ci_missing_robot")
        with pytest.raises(servo_offsets.CalibrationError, match="not found"):
            _driver()
