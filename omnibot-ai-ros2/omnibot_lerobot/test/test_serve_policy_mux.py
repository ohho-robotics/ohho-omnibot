"""Mock OhhO Serve + policy guard/mux tests for OHH-53 (headless, no GPU)."""

from __future__ import annotations

import os
import sys
import time

import numpy as np
import pytest

_ENGINES = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "omnibot-ai-engines")
)
_PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_RL_PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "omnibot_rl"))

for path in (_ENGINES, _PKG_DIR, _RL_PKG):
    if path not in sys.path:
        sys.path.insert(0, path)

from lerobot_engine.models import make_policy  # noqa: E402
from lerobot_engine.serve import MockServeServer, ServeClient  # noqa: E402
from lerobot_engine.serve.client import ServeError  # noqa: E402
from omnibot_lerobot.policy_guard import PolicyGuard  # noqa: E402
from omnibot_rl.arm_stream_gate import ArmStreamGate  # noqa: E402


def _valid_images():
    img = np.full((240, 320, 3), 128, dtype=np.uint8)
    return {
        "observation.images.wrist": img,
        "observation.images.bev": img,
    }


def test_mock_serve_endpoints_and_act_load():
    server = MockServeServer()
    server.start()
    try:
        client = ServeClient(server.base_url, timeout_sec=2.0)
        health = client.health()
        assert health["status"] == "ok"
        assert health["model_loaded"] is False

        loaded = client.load_model("hf://mock/act-checkpoint")
        assert loaded["status"] == "loaded"
        assert client.health()["model_loaded"] is True

        metrics = client.metrics_text()
        assert "ohho_serve_predict_requests_total" in metrics

        img = np.full((240, 320, 3), 64, dtype=np.uint8)
        action, latency_ms = client.predict("pick", img)
        assert action.shape == (9,)
        assert latency_ms >= 0.0
        assert "ohho_serve_predict_requests_total 1" in client.metrics_text()
    finally:
        server.stop()


def test_serve_adapter_select_action_via_mock():
    server = MockServeServer(action_vector=[0.2] * 6 + [0.1, 0.0, 0.0])
    server.start()
    try:
        adapter = make_policy(
            "serve",
            checkpoint="hf://lerobot/mock-act",
            device="cpu",
            serve_url=server.base_url,
            timeout_sec=2.0,
        )
        obs = _valid_images()
        obs["observation.state"] = np.zeros((1, 9), dtype=np.float32)
        obs["task"] = "place the cup"
        action = adapter.select_action(obs)
        assert pytest.approx(action[0]) == 0.2
        assert pytest.approx(action[6]) == 0.1
    finally:
        server.stop()


def test_serve_slow_predict_dropped_by_policy_guard():
    server = MockServeServer(predict_delay_sec=0.15, action_vector=[1.0] * 9)
    server.start()
    try:
        adapter = make_policy(
            "serve",
            checkpoint="hf://lerobot/mock-act",
            device="cpu",
            serve_url=server.base_url,
            timeout_sec=2.0,
        )
        guard = PolicyGuard(policy_period=0.10, max_joint_delta_rad=0.15)
        res = guard.tick(
            camera_images=_valid_images(),
            adapter=adapter,
            arm_positions=np.zeros(6),
            task_description="test",
        )
        assert res.should_publish is False
        assert res.dropped_reason == "stale_inference"
        assert res.publish_zero_base is True
    finally:
        server.stop()


def test_serve_unreachable_raises_on_load():
    with pytest.raises(ServeError):
        make_policy(
            "serve",
            checkpoint="hf://lerobot/mock-act",
            device="cpu",
            serve_url="http://127.0.0.1:1",
            timeout_sec=0.05,
        )


def test_mux_policy_mode_requires_autonomous_control():
    gate = ArmStreamGate.from_default_mode("policy")
    assert gate.control_mode == "teleop"
    assert gate.allows("policy") is False

    gate.set_control_mode("vla")
    assert gate.allows("policy") is True

    gate.set_control_mode("teleop")
    assert gate.allows("policy") is False


def test_mux_teleop_overrides_policy_stream():
    gate = ArmStreamGate.from_default_mode("policy")
    gate.set_control_mode("vla")
    assert gate.allows("policy") is True

    gate.set_control_mode("teleop")
    assert gate.allows("policy") is False


def test_policy_guard_clamps_serve_action():
    server = MockServeServer(action_vector=[2.0, -2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
    server.start()
    try:
        adapter = make_policy(
            "serve",
            checkpoint="hf://lerobot/mock-act",
            device="cpu",
            serve_url=server.base_url,
            timeout_sec=2.0,
        )
        guard = PolicyGuard(policy_period=0.10, max_joint_delta_rad=0.15)
        res = guard.tick(
            camera_images=_valid_images(),
            adapter=adapter,
            arm_positions=np.zeros(6),
            task_description="clamp",
        )
        assert res.should_publish is True
        assert res.arm_command is not None
        assert pytest.approx(res.arm_command[0]) == 0.15
        assert pytest.approx(res.arm_command[1]) == -0.15
    finally:
        server.stop()


def test_mock_serve_latency_benchmark_cpu():
    """Smoke benchmark for docs table (mock backend, CPU-only)."""
    server = MockServeServer(predict_delay_sec=0.0)
    server.start()
    try:
        client = ServeClient(server.base_url, timeout_sec=2.0)
        client.load_model("hf://mock/act")
        img = np.full((240, 320, 3), 32, dtype=np.uint8)
        samples = []
        for _ in range(20):
            t0 = time.perf_counter()
            client.predict("bench", img)
            samples.append((time.perf_counter() - t0) * 1000.0)
        p50 = float(np.percentile(samples, 50))
        assert p50 < 50.0, f"mock serve p50 too high: {p50:.2f} ms"
    finally:
        server.stop()
