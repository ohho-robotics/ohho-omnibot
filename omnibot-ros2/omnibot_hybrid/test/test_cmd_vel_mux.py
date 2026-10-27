"""
Unit tests for CmdVelMux — pure logic, no hardware required.

Tests exercise mode-switching and message-forwarding logic by calling
callbacks directly, with no DDS delivery or threading.  This makes them
fast and completely deterministic.
"""

import math
from unittest.mock import MagicMock

import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from omnibot_hybrid.cmd_vel_mux import CmdVelMux
from omnibot_hybrid.stream_gate import (
    DEFAULT_SOURCE_TIMEOUT_S,
    StreamGate,
    resolve_source_timeout_s,
)


@pytest.fixture()
def mux_node():
    node = CmdVelMux()
    # Swap real publishers for mocks so tests can assert on calls
    # without requiring DDS discovery or message delivery.
    node._out_pub = MagicMock()
    node._active_mode_pub = MagicMock()
    yield node
    node.destroy_node()


# ── Helpers ─────────────────────────────────────────────────────────────────


def _set_mode(node: CmdVelMux, mode: str) -> None:
    """Drive the mode-switch callback directly (no DDS publish needed)."""
    msg = String()
    msg.data = mode
    node._mode_cb(msg)


def _make_twist(x: float = 1.0) -> Twist:
    t = Twist()
    t.linear.x = x
    return t


# ── Tests ────────────────────────────────────────────────────────────────────


class TestDefaultMode:
    def test_default_is_teleop(self, mux_node):
        assert mux_node._active_mode == "teleop"

    def test_autonomous_sources_idle_until_mode_changes(self, mux_node):
        mux_node._nav2_cb(_make_twist(0.5))
        mux_node._vla_cb(_make_twist(0.6))
        mux_node._rl_nav_cb(_make_twist(0.7))
        mux_node._out_pub.publish.assert_not_called()

    def test_teleop_forwarded_by_default(self, mux_node):
        mux_node._teleop_cb(_make_twist(0.2))
        mux_node._out_pub.publish.assert_called_once()
        msg = mux_node._out_pub.publish.call_args[0][0]
        assert msg.linear.x == pytest.approx(0.2)


class TestModeSwitching:
    def test_switch_to_vla(self, mux_node):
        _set_mode(mux_node, "vla")
        assert mux_node._active_mode == "vla"

    def test_switch_to_teleop(self, mux_node):
        _set_mode(mux_node, "teleop")
        assert mux_node._active_mode == "teleop"

    def test_switch_back_to_nav2(self, mux_node):
        _set_mode(mux_node, "vla")
        _set_mode(mux_node, "nav2")
        assert mux_node._active_mode == "nav2"

    def test_unknown_mode_ignored(self, mux_node):
        _set_mode(mux_node, "nav2")
        _set_mode(mux_node, "unknown_mode")
        assert mux_node._active_mode == "nav2"

    def test_mode_is_case_insensitive(self, mux_node):
        _set_mode(mux_node, "VLA")
        assert mux_node._active_mode == "vla"

    def test_mode_strips_whitespace(self, mux_node):
        _set_mode(mux_node, "  teleop  ")
        assert mux_node._active_mode == "teleop"


class TestForwarding:
    """Verify that Twist messages are forwarded only when mode matches."""

    def test_nav2_forwarded_in_nav2_mode(self, mux_node):
        mux_node._active_mode = "nav2"
        mux_node._nav2_cb(_make_twist(0.5))
        mux_node._out_pub.publish.assert_called_once()
        msg = mux_node._out_pub.publish.call_args[0][0]
        assert msg.linear.x == pytest.approx(0.5)

    def test_vla_not_forwarded_in_nav2_mode(self, mux_node):
        mux_node._active_mode = "nav2"
        mux_node._vla_cb(_make_twist(0.7))
        mux_node._out_pub.publish.assert_not_called()

    def test_vla_forwarded_in_vla_mode(self, mux_node):
        mux_node._active_mode = "vla"
        mux_node._vla_cb(_make_twist(0.3))
        mux_node._out_pub.publish.assert_called_once()
        msg = mux_node._out_pub.publish.call_args[0][0]
        assert msg.linear.x == pytest.approx(0.3)

    def test_teleop_forwarded_in_teleop_mode(self, mux_node):
        mux_node._active_mode = "teleop"
        mux_node._teleop_cb(_make_twist(0.1))
        mux_node._out_pub.publish.assert_called_once()


class TestStopWithinOneCycle:
    def test_return_to_teleop_zeros_and_blocks_vla(self, mux_node):
        _set_mode(mux_node, "vla")
        mux_node._out_pub.reset_mock()
        _set_mode(mux_node, "teleop")
        zero = mux_node._out_pub.publish.call_args[0][0]
        assert zero.linear.x == pytest.approx(0.0)
        assert zero.angular.z == pytest.approx(0.0)
        mux_node._out_pub.reset_mock()
        mux_node._vla_cb(_make_twist(0.9))
        mux_node._out_pub.publish.assert_not_called()
        mux_node._teleop_cb(_make_twist(0.2))
        mux_node._out_pub.publish.assert_called_once()

    def test_switch_teleop_to_autonomous_zeros_output(self, mux_node):
        assert mux_node._active_mode == "teleop"
        mux_node._out_pub.reset_mock()
        _set_mode(mux_node, "vla")
        assert mux_node._out_pub.publish.call_count == 1
        zero = mux_node._out_pub.publish.call_args[0][0]
        assert zero.linear.x == pytest.approx(0.0)
        assert zero.angular.z == pytest.approx(0.0)

    def test_switch_between_autonomous_modes_zeros_output(self, mux_node):
        _set_mode(mux_node, "vla")
        mux_node._out_pub.reset_mock()
        _set_mode(mux_node, "nav2")
        assert mux_node._out_pub.publish.call_count == 1
        zero = mux_node._out_pub.publish.call_args[0][0]
        assert zero.linear.x == pytest.approx(0.0)
        assert zero.angular.z == pytest.approx(0.0)

    def test_repeated_mode_does_not_publish_zero(self, mux_node):
        _set_mode(mux_node, "vla")
        mux_node._out_pub.reset_mock()
        _set_mode(mux_node, "vla")
        mux_node._out_pub.publish.assert_not_called()

    def test_unknown_mode_does_not_publish_zero(self, mux_node):
        mux_node._out_pub.reset_mock()
        _set_mode(mux_node, "unknown_mode")
        mux_node._out_pub.publish.assert_not_called()

    def test_emergency_stop_zeros_and_blocks_teleop(self, mux_node):
        mux_node._out_pub.reset_mock()
        stop = Bool()
        stop.data = True
        mux_node._estop_cb(stop)
        zero = mux_node._out_pub.publish.call_args[0][0]
        assert zero.linear.x == pytest.approx(0.0)
        mux_node._out_pub.reset_mock()
        mux_node._teleop_cb(_make_twist(0.4))
        mux_node._out_pub.publish.assert_not_called()
        clear = Bool()
        clear.data = False
        mux_node._estop_cb(clear)
        mux_node._teleop_cb(_make_twist(0.3))
        mux_node._out_pub.publish.assert_called_once()


class TestTeleopPreemptsAutonomy:
    """Teleop via the mux must win over Nav2 / VLA while in teleop mode."""

    def test_nav2_ignored_while_teleop_active(self, mux_node):
        mux_node._active_mode = "teleop"
        mux_node._nav2_cb(_make_twist(0.8))
        mux_node._out_pub.publish.assert_not_called()
        mux_node._teleop_cb(_make_twist(0.15))
        mux_node._out_pub.publish.assert_called_once()

    def test_switching_to_teleop_stops_autonomous_feed(self, mux_node):
        _set_mode(mux_node, "nav2")
        mux_node._out_pub.publish.assert_called_once()
        zero = mux_node._out_pub.publish.call_args[0][0]
        assert zero.linear.x == pytest.approx(0.0)
        assert zero.angular.z == pytest.approx(0.0)
        mux_node._out_pub.reset_mock()
        mux_node._nav2_cb(_make_twist(0.5))
        mux_node._out_pub.publish.assert_called_once()
        msg = mux_node._out_pub.publish.call_args[0][0]
        assert msg.linear.x == pytest.approx(0.5)
        mux_node._out_pub.reset_mock()
        _set_mode(mux_node, "teleop")
        mux_node._nav2_cb(_make_twist(0.5))
        mux_node._out_pub.publish.assert_called_once()  # zero cmd only
        zero = mux_node._out_pub.publish.call_args[0][0]
        assert zero.linear.x == pytest.approx(0.0)


class TestActiveModePublisher:
    def test_active_mode_published(self, mux_node):
        mux_node._active_mode = "vla"
        mux_node._publish_active_mode()
        mux_node._active_mode_pub.publish.assert_called_once()
        msg = mux_node._active_mode_pub.publish.call_args[0][0]
        assert msg.data == "vla"


def _freeze_clock(node, start=0.0):
    clock = {"t": start}
    node._now = lambda: clock["t"]
    return clock


def _assert_zero_twist(msg) -> None:
    assert msg.linear.x == pytest.approx(0.0)
    assert msg.linear.y == pytest.approx(0.0)
    assert msg.linear.z == pytest.approx(0.0)
    assert msg.angular.x == pytest.approx(0.0)
    assert msg.angular.y == pytest.approx(0.0)
    assert msg.angular.z == pytest.approx(0.0)


# ── Source staleness ─────────────────────────────────────────────────────────


class TestSourceTimeout:
    def test_default_timeout_and_tick_are_armed(self, mux_node):
        assert mux_node.get_parameter("source_timeout_s").value == pytest.approx(
            DEFAULT_SOURCE_TIMEOUT_S
        )
        assert mux_node._gate.source_timeout_s == pytest.approx(0.3)
        assert mux_node._gate.source_timeout_warning is None
        tick_ns = int(CmdVelMux.SOURCE_STALE_TICK_S * 1_000_000_000)
        periods = [timer.timer_period_ns for timer in mux_node.timers]
        assert tick_ns in periods

    def test_silence_before_any_command_does_not_publish(self, mux_node):
        clock = _freeze_clock(mux_node)
        mux_node._out_pub.reset_mock()
        clock["t"] = 10.0
        mux_node._check_source_staleness()
        mux_node._out_pub.publish.assert_not_called()

    @pytest.mark.parametrize(
        "mode,callback_name",
        [
            ("teleop", "_teleop_cb"),
            ("nav2", "_nav2_cb"),
            ("vla", "_vla_cb"),
            ("rl_nav", "_rl_nav_cb"),
        ],
    )
    def test_active_source_zeros_once_within_timeout_plus_one_tick(
        self, mux_node, mode, callback_name
    ):
        clock = _freeze_clock(mux_node)
        _set_mode(mux_node, mode)
        mux_node._out_pub.reset_mock()

        send = getattr(mux_node, callback_name)
        send(_make_twist(0.55))
        forwarded = mux_node._out_pub.publish.call_args[0][0]
        assert forwarded.linear.x == pytest.approx(0.55)
        mux_node._out_pub.reset_mock()

        timeout = mux_node._gate.source_timeout_s
        tick = mux_node.SOURCE_STALE_TICK_S
        assert timeout == pytest.approx(0.3)
        just_before = math.nextafter(timeout, 0.0)

        clock["t"] = just_before
        mux_node._check_source_staleness()
        mux_node._out_pub.publish.assert_not_called()

        clock["t"] = just_before + tick
        mux_node._check_source_staleness()
        assert mux_node._out_pub.publish.call_count == 1
        _assert_zero_twist(mux_node._out_pub.publish.call_args[0][0])
        assert clock["t"] <= timeout + tick

        mux_node._out_pub.reset_mock()
        clock["t"] = timeout + 5.0
        mux_node._check_source_staleness()
        mux_node._out_pub.publish.assert_not_called()

        clock["t"] = 1.0
        send(_make_twist(0.25))
        restarted = mux_node._out_pub.publish.call_args[0][0]
        assert restarted.linear.x == pytest.approx(0.25)
        mux_node._out_pub.reset_mock()
        clock["t"] = 1.0 + math.nextafter(timeout, 0.0)
        mux_node._check_source_staleness()
        mux_node._out_pub.publish.assert_not_called()
        clock["t"] = 1.0 + timeout
        mux_node._check_source_staleness()
        assert mux_node._out_pub.publish.call_count == 1
        _assert_zero_twist(mux_node._out_pub.publish.call_args[0][0])
        assert (clock["t"] - 1.0) <= timeout + tick

    def test_inactive_source_does_not_refresh_active_timeout(self, mux_node):
        clock = _freeze_clock(mux_node)
        mux_node._teleop_cb(_make_twist(0.4))
        mux_node._out_pub.reset_mock()
        timeout = mux_node._gate.source_timeout_s
        clock["t"] = math.nextafter(timeout, 0.0)
        mux_node._nav2_cb(_make_twist(0.9))
        mux_node._vla_cb(_make_twist(0.8))
        mux_node._check_source_staleness()
        mux_node._out_pub.publish.assert_not_called()
        clock["t"] = timeout
        mux_node._check_source_staleness()
        assert mux_node._out_pub.publish.call_count == 1
        _assert_zero_twist(mux_node._out_pub.publish.call_args[0][0])

    def test_shorter_timeout_is_kept_and_longer_is_clamped(self):
        short = StreamGate("teleop", source_timeout_s=0.1)
        assert short.source_timeout_warning is None
        assert short.source_timeout_s == pytest.approx(0.1)
        short.note_forward("teleop", 0.0)
        assert short.poll_staleness(math.nextafter(0.1, 0.0)) is False
        assert short.poll_staleness(0.1) is True
        assert short.poll_staleness(1.0) is False

        clamped, warning = resolve_source_timeout_s(2.0)
        assert clamped == pytest.approx(DEFAULT_SOURCE_TIMEOUT_S)
        assert warning is not None and "clamped" in warning
        gate = StreamGate("vla", source_timeout_s=2.0)
        assert gate.source_timeout_s == pytest.approx(0.3)
        gate.note_forward("vla", 0.0)
        assert gate.poll_staleness(math.nextafter(0.3, 0.0)) is False
        assert gate.poll_staleness(0.3) is True

    @pytest.mark.parametrize(
        "raw",
        [0, 0.0, -0.5, -1, None, "0.1", True, False, float("nan"), float("inf")],
    )
    def test_nonpositive_or_invalid_timeout_stays_armed(self, raw):
        gate = StreamGate("nav2", source_timeout_s=raw)
        assert gate.source_timeout_s == pytest.approx(DEFAULT_SOURCE_TIMEOUT_S)
        assert gate.source_timeout_warning
        gate.note_forward("nav2", 0.0)
        assert gate.poll_staleness(math.nextafter(0.3, 0.0)) is False
        assert gate.poll_staleness(0.3) is True
