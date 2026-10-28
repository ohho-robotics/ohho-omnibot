"""Two reference adapters through the headless sim harness and policy_node.

OHH-447. Weights are numpy float32 drawn from normal(0, 0.05). Nothing is
downloaded. Gazebo is not started. rclpy is not started. No robot device is
opened.
"""

from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path

import numpy as np
import pytest

_REPO = Path(__file__).resolve().parents[3]
_ENGINES = _REPO / "omnibot-ai-engines"
_PKG = Path(__file__).resolve().parents[1]
_SCRIPTS = _REPO / "scripts"
for _path in (_ENGINES, _PKG, _SCRIPTS):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from lerobot_engine.models import RUNTIME_SAFETY, make_policy  # noqa: E402
from lerobot_engine.models.reference import (  # noqa: E402
    LINEAR_SEED,
    MLP_SEED,
    ReferenceLinearAdapter,
    ReferenceMlpAdapter,
)

import eval_pick_place as harness  # noqa: E402

_SIM_SPEC = importlib.util.spec_from_file_location(
    "ohh447_eval_pick_place_tests",
    _SCRIPTS / "tests" / "test_eval_pick_place.py",
)
_SIM_TESTS = importlib.util.module_from_spec(_SIM_SPEC)
assert _SIM_SPEC.loader is not None
_SIM_SPEC.loader.exec_module(_SIM_TESTS)
MemoryTransport = _SIM_TESTS.MemoryTransport

_GUARD_SPEC = importlib.util.spec_from_file_location(
    "ohh447_policy_guard_tests",
    Path(__file__).with_name("test_policy_guard.py"),
)
_GUARD_TESTS = importlib.util.module_from_spec(_GUARD_SPEC)
assert _GUARD_SPEC.loader is not None
_GUARD_SPEC.loader.exec_module(_GUARD_TESTS)
_install_ros_stubs = _GUARD_TESTS._install_ros_stubs
_restore_ros_stubs = _GUARD_TESTS._restore_ros_stubs


_DOC = _REPO / "docs" / "model-adapter.md"
_RUNS = (
    ("reference_linear", LINEAR_SEED, ReferenceLinearAdapter, {"W": [9, 11], "b": [9]}),
    (
        "reference_mlp",
        MLP_SEED,
        ReferenceMlpAdapter,
        {"W1": [16, 15], "b1": [16], "W2": [9, 16], "b2": [9]},
    ),
)


def _frames():
    return {
        harness.WRIST_KEY: np.full((240, 320, 3), 128, dtype=np.uint8),
        harness.BEV_KEY: np.full((240, 320, 3), 64, dtype=np.uint8),
    }


def _sim_images():
    """Plain lists. The harness treats an all-zero walk as a blank frame.

    Numpy uint8 scalars are not Python ints, so a uint8 array flattens to
    nothing and is refused. The existing harness tests pass nested lists.
    """

    def frame(value: int):
        return [
            [[value, value, value], [value, 1, 2]],
            [[3, value, value], [value, 4, value]],
        ]

    return {
        harness.WRIST_KEY: frame(128),
        harness.BEV_KEY: frame(64),
    }


def _load(model_type: str, seed: int):
    adapter = make_policy(model_type, checkpoint="random", device="cpu", seed=seed)
    assert adapter.manifest["downloaded_checkpoint"] is False
    assert adapter.manifest["checkpoint_file"] is None
    assert adapter.manifest["executed_on"] == "cpu"
    assert adapter.manifest["seed"] == seed
    assert adapter.manifest["real_robot"] is False
    assert adapter.contract.safety == RUNTIME_SAFETY
    assert adapter.contract.latency.control_hz == 10.0
    assert adapter.contract.latency.max_tick_s == pytest.approx(0.1)
    assert adapter.contract.action.dim == 9
    assert adapter.contract.action.units[0] == "rad"
    assert adapter.contract.action.units[6:] == ("m/s", "m/s", "rad/s")
    assert [spec.key for spec in adapter.contract.images] == [
        "observation.images.wrist",
        "observation.images.bev",
    ]
    assert adapter.contract.images[0].height == 240
    assert adapter.contract.images[0].width == 320
    return adapter


def test_reference_models_share_one_contract_and_differ():
    linear = _load("reference_linear", LINEAR_SEED)
    mlp = _load("reference_mlp", MLP_SEED)
    assert isinstance(linear, ReferenceLinearAdapter)
    assert isinstance(mlp, ReferenceMlpAdapter)
    assert linear.contract == mlp.contract
    assert linear.manifest["weight_shapes"] == {"W": [9, 11], "b": [9]}
    assert mlp.manifest["weight_shapes"] == {
        "W1": [16, 15],
        "b1": [16],
        "W2": [9, 16],
        "b2": [9],
    }
    obs = harness.build_policy_obs(
        harness.SceneSense(images=_frames(), arm_positions=(0, 0, 0, 0, 0, 0))
    )
    left = linear.select_action(obs)
    right = mlp.select_action(obs)
    assert left.shape == right.shape == (9,)
    assert not np.allclose(left, right)

    text = _DOC.read_text(encoding="utf-8")
    assert "seed 447" in text
    assert "seed 448" in text
    assert "reference_linear" in text
    assert "reference_mlp" in text
    assert "Gazebo was not started" in text
    assert "rclpy was not started" in text
    assert "No robot device was opened" in text
    assert "downloaded_checkpoint" in text


@pytest.fixture
def ros_stubs():
    saved = _install_ros_stubs()
    yield
    _restore_ros_stubs(saved)


@pytest.mark.parametrize("model_type,seed,cls,shapes", _RUNS)
def test_adapter_runs_in_sim_harness_and_policy_node(
    model_type, seed, cls, shapes, tmp_path, ros_stubs
):
    del ros_stubs
    adapter = _load(model_type, seed)
    assert isinstance(adapter, cls)
    assert adapter.manifest["weight_shapes"] == shapes

    images = _sim_images()
    preview = harness.build_policy_obs(
        harness.SceneSense(images=images, arm_positions=(0, 0, 0, 0, 0, 0))
    )
    raw = adapter.select_action(preview)
    expected_arm, expected_base = harness.clamp_policy_action(raw, (0, 0, 0, 0, 0, 0))

    transport = MemoryTransport(
        senses=[harness.SceneSense(images=images)],
        record=False,
    )
    backend = harness.GazeboSimBackend(
        transport,
        world="apartment",
        policy=adapter,
        max_steps=1,
        hz=10.0,
    )
    results = harness.run_eval(backend, trials=1, output_dir=tmp_path, seed_start=seed)
    assert results[0].backend == "sim"
    assert results[0].scripted is False
    assert results[0].result == "timeout"
    assert "cube pose was not observed" in results[0].detail
    assert len(transport.sent) == 1
    arm, base = transport.sent[0]
    assert arm == pytest.approx(expected_arm)
    assert base == pytest.approx(expected_base)
    for value, ref in zip(arm, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
        assert abs(value - ref) <= 0.15 + 1e-9
    assert max(abs(value) for value in base[:2]) <= harness.MAX_BASE_LIN_VEL
    assert abs(base[2]) <= harness.MAX_BASE_ANG_VEL

    from omnibot_lerobot import policy_node

    node_images = _frames()
    node = policy_node.PolicyNode()
    node.enabled = True
    node.policy_period = 30.0
    node.guard.policy_period = 30.0
    node.adapter = adapter
    node.guard.required_keys = list(adapter.image_keys)
    node.guard.reset()
    node.camera_images[harness.WRIST_KEY] = node_images[harness.WRIST_KEY]
    node.camera_images[harness.BEV_KEY] = node_images[harness.BEV_KEY]
    seen = {}
    original = adapter.select_action

    def _record(obs):
        action = original(obs)
        seen["action"] = np.asarray(action, dtype=np.float32).reshape(-1)
        return action

    adapter.select_action = _record
    node._inference_loop()
    adapter.select_action = original

    node.joint_cmd_pub.publish.assert_called_once()
    node.cmd_vel_pub.publish.assert_called_once()
    assert "/arm/joint_commands/policy" in node._publishers
    published = node.joint_cmd_pub.publish.call_args[0][0]
    twist = node.cmd_vel_pub.publish.call_args[0][0]
    from omnibot_lerobot.policy_guard import clamp_arm_action, clamp_base_action

    clamped_arm = clamp_arm_action(
        seen["action"][:6],
        np.zeros(6),
        max_delta=node.max_joint_delta_rad,
        joint_min=node.joint_min,
        joint_max=node.joint_max,
    )
    clamped_base = clamp_base_action(seen["action"][6:9], scale=node.base_vel_scale)
    assert list(published.position) == pytest.approx(list(clamped_arm))
    assert twist.linear.x == pytest.approx(float(clamped_base[0]))
    assert twist.linear.y == pytest.approx(float(clamped_base[1]))
    assert twist.angular.z == pytest.approx(float(clamped_base[2]))

    blocked = policy_node.PolicyNode()
    blocked.enabled = True
    blocked.adapter = adapter

    def _forbid(_obs):
        raise AssertionError("select_action ran on a black frame")

    adapter.select_action = _forbid
    blocked.camera_images[harness.WRIST_KEY] = np.zeros((240, 320, 3), dtype=np.uint8)
    blocked.camera_images[harness.BEV_KEY] = node_images[harness.BEV_KEY]
    blocked._inference_loop()
    adapter.select_action = original
    blocked.joint_cmd_pub.publish.assert_not_called()
    blocked.cmd_vel_pub.publish.assert_called_once()
    zero = blocked.cmd_vel_pub.publish.call_args[0][0]
    assert (zero.linear.x, zero.linear.y, zero.angular.z) == (0.0, 0.0, 0.0)
    assert any("missing or black" in warning for warning in blocked.get_logger().warnings)

    late = policy_node.PolicyNode()
    late.enabled = True
    late.adapter = adapter
    late.policy_period = 0.10
    late.guard.policy_period = 0.10
    late.camera_images[harness.WRIST_KEY] = node_images[harness.WRIST_KEY]
    late.camera_images[harness.BEV_KEY] = node_images[harness.BEV_KEY]
    calls = {"n": 0}

    def _count(obs):
        calls["n"] += 1
        return original(obs)

    adapter.select_action = _count
    times = [100.0, 100.05, 100.15]
    monkeypatch = pytest.MonkeyPatch()
    monkeypatch.setattr(time, "perf_counter", lambda: times.pop(0) if times else 200.0)
    try:
        late._inference_loop()
    finally:
        monkeypatch.undo()
        adapter.select_action = original
    assert calls["n"] == 1
    late.joint_cmd_pub.publish.assert_not_called()
    late.cmd_vel_pub.publish.assert_called_once()
    dropped = late.cmd_vel_pub.publish.call_args[0][0]
    assert (dropped.linear.x, dropped.linear.y, dropped.angular.z) == (0.0, 0.0, 0.0)
    assert any("action dropped" in warning for warning in late.get_logger().warnings)


def test_doc_names_both_seeds():
    text = _DOC.read_text(encoding="utf-8")
    assert f"seed {LINEAR_SEED}" in text or "Seed | 447" in text
    assert "448" in text
