"""Unit tests for policy guards and policy node decisions.

Covers:
1. Black-frame and missing-image rejection (no publish that tick, throttled warning).
2. Arm delta clamping (0.15 rad/cycle limit matching arm driver).
3. Inference latency check (dropping action when select_action exceeds policy period).
4. Fake adapter test harness without Torch, LeRobot, or ROS graph.
5. PolicyNode ROS stub tests for end-to-end node integration.
"""

from __future__ import annotations

import os
import sys
import types
from unittest.mock import MagicMock
import numpy as np
import pytest

# Ensure package is on sys.path
_PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from omnibot_lerobot.policy_guard import (
    MAX_JOINT_DELTA_RAD,
    PolicyGuard,
    check_camera_images,
    clamp_arm_action,
    clamp_base_action,
    clamp_joint_deltas,
    is_black_frame,
    is_latency_exceeded,
)


class FakePolicyAdapter:
    """Fake adapter for testing without Torch or LeRobot dependencies."""

    def __init__(
        self,
        action: np.ndarray | None = None,
        image_keys: list[str] | None = None,
        duration: float = 0.01,
    ):
        self.image_keys = (
            list(image_keys)
            if image_keys is not None
            else ["observation.images.wrist", "observation.images.bev"]
        )
        self.state_key = "observation.state"
        self.task_key = "task"
        self.action_dim = 9
        self.image_size = (320, 240)
        self.duration = duration
        self.action = (
            np.asarray(action, dtype=np.float32)
            if action is not None
            else np.zeros(9, dtype=np.float32)
        )
        self.select_action_calls = 0
        self.last_obs: dict | None = None
        self.reset_calls = 0

    def reset(self) -> None:
        self.reset_calls += 1

    def select_action(self, obs: dict) -> np.ndarray:
        self.select_action_calls += 1
        self.last_obs = obs
        return self.action


def _make_valid_image(h: int = 240, w: int = 320) -> np.ndarray:
    """Generate a non-black uint8 RGB test image."""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[10:20, 10:20] = 200
    return img


def _make_black_image(h: int = 240, w: int = 320) -> np.ndarray:
    """Generate an all-zero uint8 RGB black frame."""
    return np.zeros((h, w, 3), dtype=np.uint8)


# ---------------------------------------------------------------------------
# 1. Black frame and missing image tests
# ---------------------------------------------------------------------------


class TestBlackFrameAndMissingImages:
    """Acceptance criterion: Missing wrist or bev means no publish that tick, and warning."""

    def test_is_black_frame(self):
        assert is_black_frame(None) is True
        assert is_black_frame("not an array") is True
        assert is_black_frame(np.array([], dtype=np.uint8)) is True
        assert is_black_frame(_make_black_image()) is True
        assert is_black_frame(np.zeros((10, 10), dtype=np.float32)) is True

        valid_img = _make_valid_image()
        assert is_black_frame(valid_img) is False

        # Single non-zero pixel in an otherwise black image is not a pure black frame
        single_px = np.zeros((100, 100, 3), dtype=np.uint8)
        single_px[50, 50, 0] = 1
        assert is_black_frame(single_px) is False

    def test_check_camera_images_missing_wrist(self):
        images = {
            "observation.images.wrist": None,
            "observation.images.bev": _make_valid_image(),
        }
        valid, invalid = check_camera_images(images)
        assert valid is False
        assert "observation.images.wrist" in invalid
        assert len(invalid) == 1

    def test_check_camera_images_missing_bev(self):
        images = {
            "observation.images.wrist": _make_valid_image(),
            "observation.images.bev": None,
        }
        valid, invalid = check_camera_images(images)
        assert valid is False
        assert "observation.images.bev" in invalid
        assert len(invalid) == 1

    def test_check_camera_images_black_wrist(self):
        images = {
            "observation.images.wrist": _make_black_image(),
            "observation.images.bev": _make_valid_image(),
        }
        valid, invalid = check_camera_images(images)
        assert valid is False
        assert "observation.images.wrist" in invalid

    def test_check_camera_images_black_bev(self):
        images = {
            "observation.images.wrist": _make_valid_image(),
            "observation.images.bev": _make_black_image(),
        }
        valid, invalid = check_camera_images(images)
        assert valid is False
        assert "observation.images.bev" in invalid

    def test_check_camera_images_both_black(self):
        images = {
            "observation.images.wrist": _make_black_image(),
            "observation.images.bev": _make_black_image(),
        }
        valid, invalid = check_camera_images(images)
        assert valid is False
        assert len(invalid) == 2

    def test_check_camera_images_all_valid(self):
        images = {
            "observation.images.wrist": _make_valid_image(),
            "observation.images.bev": _make_valid_image(),
        }
        valid, invalid = check_camera_images(images)
        assert valid is True
        assert len(invalid) == 0

    def test_check_camera_images_short_keys(self):
        images = {
            "wrist": _make_black_image(),
            "bev": _make_valid_image(),
        }
        valid, invalid = check_camera_images(images)
        assert valid is False
        assert "wrist" in invalid


# ---------------------------------------------------------------------------
# 2. Arm delta clamping tests
# ---------------------------------------------------------------------------


class TestArmDeltaClamping:
    """Acceptance criterion: Arm deltas are clamped (0.15 rad/cycle limit)."""

    def test_max_joint_delta_rad_constant(self):
        assert MAX_JOINT_DELTA_RAD == 0.15

    def test_clamp_positive_large_delta(self):
        commanded = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        reference = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        clamped = clamp_arm_action(commanded, reference, max_delta=0.15)
        assert pytest.approx(clamped[0]) == 0.15
        assert pytest.approx(clamped[1]) == 0.0

    def test_clamp_negative_large_delta(self):
        commanded = [-0.8, 0.0, 0.0, 0.0, 0.0, 0.0]
        reference = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        clamped = clamp_arm_action(commanded, reference, max_delta=0.15)
        assert pytest.approx(clamped[0]) == -0.15

    def test_clamp_small_delta_preserved(self):
        commanded = [0.05, -0.10, 0.14, 0.0, 0.0, 0.0]
        reference = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        clamped = clamp_arm_action(commanded, reference, max_delta=0.15)
        assert pytest.approx(clamped[0]) == 0.05
        assert pytest.approx(clamped[1]) == -0.10
        assert pytest.approx(clamped[2]) == 0.14

    def test_clamp_reference_none_preserves_targets(self):
        commanded = [1.5, -1.0, 0.5, 0.0, 0.0, 0.0]
        clamped = clamp_arm_action(commanded, reference=None, max_delta=0.15)
        assert np.allclose(clamped, commanded)

    def test_multi_step_trajectory(self):
        guard = PolicyGuard(max_joint_delta_rad=0.15)
        ref = [0.0] * 6
        target = [0.40] + [0.0] * 5

        # Step 1: 0.0 -> 0.15
        c1 = guard.process_arm_action(target, reference=ref)
        assert pytest.approx(c1[0]) == 0.15
        assert np.allclose(guard.last_arm_cmd, c1)

        # Step 2: 0.15 -> 0.30
        c2 = guard.process_arm_action(target, reference=ref)
        assert pytest.approx(c2[0]) == 0.30

        # Step 3: 0.30 -> 0.40 (reaches target in 0.10 rad step)
        c3 = guard.process_arm_action(target, reference=ref)
        assert pytest.approx(c3[0]) == 0.40

        # Step 4: already at target
        c4 = guard.process_arm_action(target, reference=ref)
        assert pytest.approx(c4[0]) == 0.40

    def test_guard_reset_clears_baseline(self):
        guard = PolicyGuard(max_joint_delta_rad=0.15)
        guard.process_arm_action([0.15] * 6, reference=[0.0] * 6)
        assert guard.last_arm_cmd is not None

        guard.reset()
        assert guard.last_arm_cmd is None

        # Next action references new arm position, not the old command
        new_ref = [0.8] * 6
        c = guard.process_arm_action([0.0] * 6, reference=new_ref)
        assert pytest.approx(c[0]) == 0.8 - 0.15

    def test_clamp_with_joint_limits(self):
        joint_min = [-1.0] * 6
        joint_max = [1.0] * 6
        # Target exceeds joint limits
        commanded = [2.5] * 6
        reference = [0.9] * 6
        clamped = clamp_arm_action(
            commanded,
            reference,
            max_delta=0.15,
            joint_min=joint_min,
            joint_max=joint_max,
        )
        # Even though ref + delta = 1.05, limited to 1.0
        assert float(clamped[0]) <= 1.0
        assert pytest.approx(clamped[0]) == 1.0

    def test_baseline_does_not_wind_past_joint_limits_and_reverses_immediately(self):
        joint_min = [-3.14, -1.57, -1.57, -1.57, -3.14, -0.1]
        joint_max = [3.14, 1.57, 1.57, 1.57, 3.14, 0.8]
        guard = PolicyGuard(
            max_joint_delta_rad=0.15,
            joint_min=joint_min,
            joint_max=joint_max,
        )
        ref = [3.0] + [0.0] * 5

        # Command saturated value (10.0 rad) for multiple cycles
        for _ in range(10):
            cmd = guard.process_arm_action([10.0] + [0.0] * 5, reference=ref)
            # Clamped to joint_max (3.14)
            assert pytest.approx(cmd[0]) == 3.14
            assert float(cmd[0]) <= 3.14 + 1e-5
            assert pytest.approx(guard.last_arm_cmd[0]) == 3.14
            assert float(guard.last_arm_cmd[0]) <= 3.14 + 1e-5

        # Verify internal baseline did not wind past 3.14
        assert pytest.approx(guard.last_arm_cmd[0]) == 3.14

        # Reversing command must step immediately below 3.14 on cycle 1 (no unwinding lag)
        rev = guard.process_arm_action([0.0] + [0.0] * 5, reference=ref)
        assert pytest.approx(rev[0]) == 3.14 - 0.15
        assert pytest.approx(guard.last_arm_cmd[0]) == 3.14 - 0.15

        # Verify negative joint limit clamping and reversal
        guard_neg = PolicyGuard(
            max_joint_delta_rad=0.15,
            joint_min=joint_min,
            joint_max=joint_max,
        )
        ref_neg = [-3.0] + [0.0] * 5
        for _ in range(10):
            cmd_neg = guard_neg.process_arm_action([-10.0] + [0.0] * 5, reference=ref_neg)
            assert pytest.approx(cmd_neg[0]) == -3.14
            assert float(cmd_neg[0]) >= -3.14 - 1e-5
            assert pytest.approx(guard_neg.last_arm_cmd[0]) == -3.14
            assert float(guard_neg.last_arm_cmd[0]) >= -3.14 - 1e-5

        assert pytest.approx(guard_neg.last_arm_cmd[0]) == -3.14
        rev_neg = guard_neg.process_arm_action([0.0] + [0.0] * 5, reference=ref_neg)
        assert pytest.approx(rev_neg[0]) == -3.14 + 0.15
        assert pytest.approx(guard_neg.last_arm_cmd[0]) == -3.14 + 0.15


# ---------------------------------------------------------------------------
# 3. Latency check tests
# ---------------------------------------------------------------------------


class TestInferenceLatency:
    """Acceptance criterion: If select_action exceeds policy period, action is dropped."""

    def test_latency_within_period(self):
        assert is_latency_exceeded(duration_sec=0.08, policy_period_sec=0.10) is False

    def test_latency_equal_period(self):
        assert is_latency_exceeded(duration_sec=0.10, policy_period_sec=0.10) is False

    def test_latency_exceeds_period(self):
        assert is_latency_exceeded(duration_sec=0.101, policy_period_sec=0.10) is True
        assert is_latency_exceeded(duration_sec=0.25, policy_period_sec=0.10) is True


# ---------------------------------------------------------------------------
# 4. Tick decision with fake adapter
# ---------------------------------------------------------------------------


class TestTickDecisionWithFakeAdapter:
    """Acceptance criteria: Pure-Python tick tests with FakeAdapter."""

    def test_tick_black_frame_refused_no_publish(self):
        guard = PolicyGuard(policy_period=0.10, max_joint_delta_rad=0.15)
        adapter = FakePolicyAdapter()
        camera_images = {
            "observation.images.wrist": _make_black_image(),
            "observation.images.bev": _make_valid_image(),
        }

        res = guard.tick(
            camera_images=camera_images,
            adapter=adapter,
            arm_positions=np.zeros(6),
        )

        assert res.should_publish is False
        assert res.dropped_reason == "missing_or_black_images"
        assert "Waiting for valid images" in res.warning
        assert "observation.images.wrist" in res.warning
        # select_action must NEVER be called when an image is black
        assert adapter.select_action_calls == 0
        assert guard.last_arm_cmd is None

    def test_tick_missing_frame_refused_no_publish(self):
        guard = PolicyGuard(policy_period=0.10, max_joint_delta_rad=0.15)
        adapter = FakePolicyAdapter()
        camera_images = {
            "observation.images.wrist": _make_valid_image(),
            "observation.images.bev": None,
        }

        res = guard.tick(
            camera_images=camera_images,
            adapter=adapter,
            arm_positions=np.zeros(6),
        )

        assert res.should_publish is False
        assert res.dropped_reason == "missing_or_black_images"
        assert "observation.images.bev" in res.warning
        assert adapter.select_action_calls == 0

    def test_tick_late_action_dropped(self):
        guard = PolicyGuard(policy_period=0.10, max_joint_delta_rad=0.15)
        raw_action = np.array([0.5] * 6 + [0.1, 0.0, 0.0], dtype=np.float32)
        adapter = FakePolicyAdapter(action=raw_action)
        camera_images = {
            "observation.images.wrist": _make_valid_image(),
            "observation.images.bev": _make_valid_image(),
        }

        # Simulated clock: inference takes 0.13 s (> 0.10 s period)
        clock_ticks = [0.0, 0.001, 0.001, 0.131]
        t_iter = iter(clock_ticks)

        res = guard.tick(
            camera_images=camera_images,
            adapter=adapter,
            arm_positions=np.zeros(6),
            now_fn=lambda: next(t_iter),
        )

        assert res.should_publish is False
        assert res.dropped_reason == "stale_inference"
        assert "action dropped" in res.warning
        assert adapter.select_action_calls == 1
        # Action was dropped: last_arm_cmd must NOT be updated
        assert guard.last_arm_cmd is None

    def test_skipped_tick_publishes_zero_base_decision(self):
        guard = PolicyGuard(policy_period=0.10, max_joint_delta_rad=0.15)
        adapter = FakePolicyAdapter()
        # Black frame
        res_black = guard.tick(
            camera_images={
                "observation.images.wrist": _make_black_image(),
                "observation.images.bev": _make_valid_image(),
            },
            adapter=adapter,
            arm_positions=np.zeros(6),
        )
        assert res_black.should_publish is False
        assert res_black.publish_zero_base is True
        assert res_black.arm_command is None
        assert np.allclose(res_black.base_command, [0.0, 0.0, 0.0])

        # Missing frame
        res_missing = guard.tick(
            camera_images={
                "observation.images.wrist": _make_valid_image(),
                "observation.images.bev": None,
            },
            adapter=adapter,
            arm_positions=np.zeros(6),
        )
        assert res_missing.should_publish is False
        assert res_missing.publish_zero_base is True
        assert res_missing.arm_command is None
        assert np.allclose(res_missing.base_command, [0.0, 0.0, 0.0])

        # Latency exceeded
        clock_ticks = [0.0, 0.001, 0.001, 0.150]
        t_iter = iter(clock_ticks)
        res_late = guard.tick(
            camera_images={
                "observation.images.wrist": _make_valid_image(),
                "observation.images.bev": _make_valid_image(),
            },
            adapter=adapter,
            arm_positions=np.zeros(6),
            now_fn=lambda: next(t_iter),
        )
        assert res_late.should_publish is False
        assert res_late.publish_zero_base is True
        assert res_late.arm_command is None
        assert np.allclose(res_late.base_command, [0.0, 0.0, 0.0])

    def test_tick_valid_action_published_and_clamped(self):
        guard = PolicyGuard(policy_period=0.10, max_joint_delta_rad=0.15)
        raw_action = np.array([1.0, -1.0, 0.05, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0], dtype=np.float32)
        adapter = FakePolicyAdapter(action=raw_action)
        camera_images = {
            "observation.images.wrist": _make_valid_image(),
            "observation.images.bev": _make_valid_image(),
        }

        # Simulated clock: inference takes 0.04 s (<= 0.10 s period)
        clock_ticks = [0.0, 0.001, 0.001, 0.041]
        t_iter = iter(clock_ticks)

        res = guard.tick(
            camera_images=camera_images,
            adapter=adapter,
            arm_positions=np.zeros(6),
            now_fn=lambda: next(t_iter),
        )

        assert res.should_publish is True
        assert res.arm_command is not None
        assert res.base_command is not None
        # Joint 0 jumped by 1.0 rad -> clamped to 0.15 rad
        assert pytest.approx(res.arm_command[0]) == 0.15
        # Joint 1 jumped by -1.0 rad -> clamped to -0.15 rad
        assert pytest.approx(res.arm_command[1]) == -0.15
        # Joint 2 jumped by 0.05 rad -> preserved
        assert pytest.approx(res.arm_command[2]) == 0.05
        # Base velocity clipped
        assert pytest.approx(res.base_command[0]) == min(0.20, 0.5 * 0.3)
        # last_arm_cmd updated
        assert guard.last_arm_cmd is not None
        assert pytest.approx(guard.last_arm_cmd[0]) == 0.15


# ---------------------------------------------------------------------------
# 5. PolicyNode integration tests with stubbed ROS 2
# ---------------------------------------------------------------------------


def _install_ros_stubs():
    """Install minimal ROS 2 module stubs for testing PolicyNode without rclpy."""
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
            self.errors = []

        def warn(self, message, **_kwargs):
            self.warnings.append(message)

        def info(self, message, **_kwargs):
            self.infos.append(message)

        def error(self, message, **_kwargs):
            self.errors.append(message)

    class _Node:
        def __init__(self, name):
            self._name = name
            self._params = {}
            self._logger = _Logger()
            self._publishers = {}

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

        def create_publisher(self, msg_type, topic, _qos):
            pub = MagicMock()
            self._publishers[topic] = pub
            return pub

        def create_subscription(self, _msg_type, topic, callback, _qos):
            return MagicMock()

        def create_timer(self, *_args, **_kwargs):
            return MagicMock()

        def destroy_node(self):
            return None

    class JointState:
        def __init__(self):
            self.name = []
            self.position = []
            self.header = types.SimpleNamespace(stamp=None)

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class Odometry:
        def __init__(self):
            self.twist = types.SimpleNamespace(
                twist=types.SimpleNamespace(
                    linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                )
            )

    class String:
        def __init__(self):
            self.data = ""

    class Bool:
        def __init__(self):
            self.data = False

    class Image:
        pass

    # Stub modules
    m_rclpy = _module("rclpy")
    m_rclpy.node = types.ModuleType("rclpy.node")
    m_rclpy.node.Node = _Node
    sys.modules["rclpy.node"] = m_rclpy.node

    m_sensor = _module("sensor_msgs")
    m_sensor_msg = _module("sensor_msgs.msg")
    m_sensor_msg.Image = Image
    m_sensor_msg.JointState = JointState
    m_sensor.msg = m_sensor_msg

    m_geom = _module("geometry_msgs")
    m_geom_msg = _module("geometry_msgs.msg")
    m_geom_msg.Twist = Twist
    m_geom.msg = m_geom_msg

    m_nav = _module("nav_msgs")
    m_nav_msg = _module("nav_msgs.msg")
    m_nav_msg.Odometry = Odometry
    m_nav.msg = m_nav_msg

    m_std = _module("std_msgs")
    m_std_msg = _module("std_msgs.msg")
    m_std_msg.String = String
    m_std_msg.Bool = Bool
    m_std.msg = m_std_msg

    return saved


def _restore_ros_stubs(saved):
    for name, module in saved.items():
        if module is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = module


class TestPolicyNodeWithROSStubs:
    """Test PolicyNode ROS callback and publication logic using stubs."""

    @pytest.fixture(autouse=True)
    def setup_and_teardown(self):
        saved = _install_ros_stubs()
        yield
        _restore_ros_stubs(saved)

    def test_node_declares_max_joint_delta_rad(self):
        from omnibot_lerobot import policy_node

        node = policy_node.PolicyNode()
        assert "max_joint_delta_rad" in node._params
        assert node._params["max_joint_delta_rad"] == 0.15
        assert node.guard.max_joint_delta_rad == 0.15
        assert "/arm/joint_commands/policy" in node._publishers
        assert "/arm/joint_commands" not in node._publishers

    def test_node_passes_joint_limits_to_guard(self):
        from omnibot_lerobot import policy_node

        node = policy_node.PolicyNode()
        assert "joint_min" in node._params
        assert "joint_max" in node._params
        expected_min = [-3.14, -1.57, -1.57, -1.57, -3.14, -0.1]
        expected_max = [3.14, 1.57, 1.57, 1.57, 3.14, 0.8]
        assert node._params["joint_min"] == expected_min
        assert node._params["joint_max"] == expected_max
        assert node.guard.joint_min == expected_min
        assert node.guard.joint_max == expected_max

    def test_node_refuses_black_frame_and_drops_publish(self):
        from omnibot_lerobot import policy_node

        node = policy_node.PolicyNode()
        node.enabled = True
        node.adapter = FakePolicyAdapter()

        # Wrist is black frame
        node.camera_images["observation.images.wrist"] = _make_black_image()
        node.camera_images["observation.images.bev"] = _make_valid_image()

        node._inference_loop()

        # Arm command not published; zero base twist published
        node.joint_cmd_pub.publish.assert_not_called()
        node.cmd_vel_pub.publish.assert_called_once()
        base_msg = node.cmd_vel_pub.publish.call_args[0][0]
        assert base_msg.linear.x == 0.0
        assert base_msg.linear.y == 0.0
        assert base_msg.angular.z == 0.0

        # Warning logged
        warnings = node.get_logger().warnings
        assert any("missing or black" in w for w in warnings)

    def test_node_refuses_missing_frame_and_drops_publish(self):
        from omnibot_lerobot import policy_node

        node = policy_node.PolicyNode()
        node.enabled = True
        node.adapter = FakePolicyAdapter()

        node.camera_images["observation.images.wrist"] = _make_valid_image()
        node.camera_images["observation.images.bev"] = None

        node._inference_loop()

        # Arm command not published; zero base twist published
        node.joint_cmd_pub.publish.assert_not_called()
        node.cmd_vel_pub.publish.assert_called_once()
        base_msg = node.cmd_vel_pub.publish.call_args[0][0]
        assert base_msg.linear.x == 0.0
        assert base_msg.linear.y == 0.0
        assert base_msg.angular.z == 0.0

        warnings = node.get_logger().warnings
        assert any("missing or black" in w for w in warnings)

    def test_node_drops_late_action(self, monkeypatch):
        import time
        from omnibot_lerobot import policy_node

        node = policy_node.PolicyNode()
        node.enabled = True
        node.policy_period = 0.10
        node.guard.policy_period = 0.10
        node.adapter = FakePolicyAdapter(action=np.zeros(9))

        node.camera_images["observation.images.wrist"] = _make_valid_image()
        node.camera_images["observation.images.bev"] = _make_valid_image()

        # Mock time.perf_counter so select_action takes 0.15s (> 0.10s period)
        times = [100.0, 100.15]
        monkeypatch.setattr(time, "perf_counter", lambda: times.pop(0) if times else 200.0)

        node._inference_loop()

        # Arm command not published; zero base twist published
        node.joint_cmd_pub.publish.assert_not_called()
        node.cmd_vel_pub.publish.assert_called_once()
        base_msg = node.cmd_vel_pub.publish.call_args[0][0]
        assert base_msg.linear.x == 0.0
        assert base_msg.linear.y == 0.0
        assert base_msg.angular.z == 0.0

        warnings = node.get_logger().warnings
        assert any("action dropped" in w for w in warnings)

    def test_node_publishes_clamped_action_on_valid_tick(self):
        from omnibot_lerobot import policy_node

        node = policy_node.PolicyNode()
        node.enabled = True
        # Large jump in joint 0
        raw = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.0, 0.0])
        node.adapter = FakePolicyAdapter(action=raw)

        node.camera_images["observation.images.wrist"] = _make_valid_image()
        node.camera_images["observation.images.bev"] = _make_valid_image()

        node._inference_loop()

        # Both arm and base must have published
        node.joint_cmd_pub.publish.assert_called_once()
        node.cmd_vel_pub.publish.assert_called_once()

        # Verify published arm positions are clamped to 0.15 rad
        arm_msg = node.joint_cmd_pub.publish.call_args[0][0]
        assert pytest.approx(arm_msg.position[0]) == 0.15
