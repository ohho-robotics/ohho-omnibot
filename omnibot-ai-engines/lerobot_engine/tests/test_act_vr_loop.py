"""OHH-243: ACT VR config, CPU smoke, serve mux path, and blank eval table."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

_ENGINES = Path(__file__).resolve().parents[2]
_LEROBOT_PKG = _ENGINES.parent / "omnibot-ai-ros2" / "omnibot_lerobot"
_RL_PKG = _ENGINES.parent / "omnibot-ai-ros2" / "omnibot_rl"
for _path in (_ENGINES, _LEROBOT_PKG, _RL_PKG):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from lerobot_engine.act_vr_sim import (  # noqa: E402
    CHECKPOINT_REL,
    HF_DATASET_ID,
    POLICY_ID,
    TRIAL_COUNT,
    hf_dataset_push_steps,
    load_train_config,
    plan_full_train,
    render_trial_markdown,
    resolve_dataset,
    run_lerobot_act_cpu_smoke,
    run_numpy_smoke,
    success_rate,
    trial_rows,
)
from lerobot_engine.models import make_policy  # noqa: E402
from lerobot_engine.serve import MockServeServer, ServeClient  # noqa: E402
from omnibot_lerobot.policy_guard import PolicyGuard  # noqa: E402
from omnibot_lerobot.policy_load import requested_checkpoint  # noqa: E402
from omnibot_rl.arm_stream_gate import ArmStreamGate  # noqa: E402

_LAUNCH = (
    _ENGINES.parent
    / "omnibot-ai-ros2"
    / "omnibot_lerobot"
    / "launch"
    / "act_vr_sim_pick.launch.py"
)
_POLICY_NODE = (
    _ENGINES.parent
    / "omnibot-ai-ros2"
    / "omnibot_lerobot"
    / "omnibot_lerobot"
    / "policy_node.py"
)


def test_config_points_at_ohh237_dataset():
    cfg = load_train_config()
    dataset = cfg["dataset"]
    assert dataset["repo_id"] == "local/omnibot-sim-vr-pick"
    assert dataset["local_episode_root"] == "/tmp/omnibot-sim-vr-pick"
    assert dataset["card_path"] == "datasets/omnibot-sim-vr-pick"
    assert dataset["hf_id"] == HF_DATASET_ID
    assert dataset["hf_published"] is False
    assert cfg["model"]["type"] == "act"
    assert cfg["model"]["policy_id"] == POLICY_ID
    assert cfg["serve"]["checkpoint"] == CHECKPOINT_REL
    assert cfg["serve"]["model_type"] == "serve"
    assert cfg["serve"]["hf_checkpoint_published"] is False
    assert cfg["training"]["wandb_project"] == ""
    assert cfg["eval"]["trials"] == 20


def test_missing_dataset_is_not_treated_as_published():
    cfg = load_train_config()
    choice = resolve_dataset(cfg, dataset_path=None, smoke=False)
    assert choice["kind"] == "missing"
    assert choice["trainable"] is False
    assert choice["hf_published"] is False
    steps = hf_dataset_push_steps()
    assert HF_DATASET_ID in steps
    assert "huggingface-cli upload" in steps


def test_smoke_writes_manifest_without_claiming_act_or_wandb(tmp_path):
    manifest = run_numpy_smoke(tmp_path, steps=2, seed=243)
    assert manifest["backend"] == "numpy_chunk_linear_smoke"
    assert manifest["weights_are_lerobot_act"] is False
    assert manifest["wandb_run"] is None
    assert manifest["dataset_source"] == "synthetic_smoke"
    assert manifest["losses"][1] < manifest["losses"][0]
    saved = json.loads((tmp_path / "best" / "train_manifest.json").read_text(encoding="utf-8"))
    assert saved["policy_id"] == POLICY_ID
    assert (tmp_path / "best" / "policy_smoke.npz").is_file()


def test_lerobot_act_cpu_smoke_when_installed(tmp_path):
    try:
        manifest = run_lerobot_act_cpu_smoke(tmp_path, steps=2, seed=243)
    except ImportError:
        pytest.skip("lerobot ACT is not installed")
    assert manifest["backend"] == "lerobot_act_cpu_smoke"
    assert manifest["weights_are_lerobot_act"] is True
    assert manifest["checkpoint_persisted"] is False
    assert manifest["wandb_run"] is None
    assert manifest["parameters_changed"] is True
    assert manifest["device"] == "cpu"
    assert manifest["pretrained_backbone_weights"] is None
    assert len(manifest["losses"]) == 2
    assert all(math.isfinite(loss) for loss in manifest["losses"])
    assert isinstance(manifest["loss_decreased"], bool)
    saved = json.loads((tmp_path / "act_cpu_smoke.json").read_text(encoding="utf-8"))
    assert saved["parameter_count"] == manifest["parameter_count"]


def test_smoke_cli_is_one_command(tmp_path):
    import subprocess

    script = _ENGINES / "lerobot_engine" / "train_act_vr.py"
    completed = subprocess.run(
        [
            sys.executable,
            str(script),
            "--smoke",
            "--output-dir",
            str(tmp_path),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    assert "weights_are_lerobot_act: False" in completed.stdout
    assert "wandb_run: None" in completed.stdout


def test_full_plan_targets_act_and_does_not_run():
    cfg = load_train_config()
    command = plan_full_train(cfg, "/tmp/omnibot-sim-vr-pick")
    assert "--model" in command
    assert command[command.index("--model") + 1] == "act"
    assert "/tmp/omnibot-sim-vr-pick" in command
    assert "--wandb-project" not in command


def test_eval_scaffold_has_twenty_blank_trials():
    rows = trial_rows()
    assert len(rows) == TRIAL_COUNT == 20
    assert [row["seed"] for row in rows] == list(range(237, 257))
    assert all(row["success"] is None for row in rows)
    assert success_rate(rows) is None
    text = render_trial_markdown(rows)
    assert "not run" in text
    assert "| 20 | 256 |" in text
    scored = [dict(row, success=True) for row in rows]
    scored[0]["success"] = None
    assert success_rate(scored) is None
    filled = [dict(row, success=(row["trial"] % 2 == 0)) for row in rows]
    assert success_rate(filled) == 0.5


def test_serve_loads_act_vr_checkpoint_and_mux_stays_gated():
    server = MockServeServer(action_vector=[2.0, -2.0, 0, 0, 0, 0, 1, 0, 0])
    server.start()
    try:
        client = ServeClient(server.base_url, timeout_sec=2.0)
        listed = client.models()
        assert listed["models"][0]["id"] == POLICY_ID
        assert listed["models"][0]["checkpoint"] == CHECKPOINT_REL

        loaded = client.load_model(CHECKPOINT_REL)
        assert loaded["status"] == "loaded"
        assert server.model_path == CHECKPOINT_REL

        adapter = make_policy(
            "serve",
            checkpoint=CHECKPOINT_REL,
            device="cpu",
            serve_url=server.base_url,
            timeout_sec=2.0,
        )
        wrist = np.full((240, 320, 3), 128, dtype=np.uint8)
        obs = {
            "observation.images.wrist": wrist,
            "observation.images.bev": wrist,
            "task": "pick cube into bin",
        }
        guard = PolicyGuard(policy_period=0.10, max_joint_delta_rad=0.15)
        result = guard.tick(
            camera_images={
                "observation.images.wrist": wrist,
                "observation.images.bev": wrist,
            },
            adapter=adapter,
            arm_positions=np.zeros(6),
            task_description="pick cube into bin",
        )
        assert result.should_publish is True
        assert pytest.approx(result.arm_command[0]) == 0.15
        assert pytest.approx(result.arm_command[1]) == -0.15

        gate = ArmStreamGate.from_default_mode("policy")
        assert gate.allows("policy") is False
        gate.set_control_mode("vla")
        gate.set_arm_mode("policy")
        assert gate.allows("policy") is True
        gate.set_emergency_stop(True)
        assert gate.allows("policy") is False
    finally:
        server.stop()


def test_policy_load_message_does_not_enable_by_itself():
    assert requested_checkpoint("checkpoints/act_vr_sim_pick/best", "  ") is None
    assert (
        requested_checkpoint(CHECKPOINT_REL, CHECKPOINT_REL) is None
    )
    assert requested_checkpoint("", CHECKPOINT_REL) == CHECKPOINT_REL
    source = _POLICY_NODE.read_text(encoding="utf-8")
    assert "/policy/load" in source
    assert "requested_checkpoint" in source
    assert "self.enabled = True" not in source.split("def _policy_load_cb", 1)[1].split(
        "def ", 1
    )[0]
    launch = _LAUNCH.read_text(encoding="utf-8")
    assert "model_type" in launch and "serve" in launch
    assert CHECKPOINT_REL in launch
