"""ACT train/serve/eval ids for the OHH-237 VR sim pick dataset.

The episode files are not in this branch. OHH-237 recorded a 3-episode
headless smoke at ``/tmp/omnibot-sim-vr-pick`` inside the nav-dev container
(``repo_id`` ``local/omnibot-sim-vr-pick``) and committed only the dataset
card at ``datasets/omnibot-sim-vr-pick``. The Hugging Face dataset
``ohho-robotics/omnibot-sim-vr-pick`` has not been created.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

POLICY_ID = "act-vr-sim-pick"
POLICY_LABEL = "ACT VR sim pick"
# Local checkpoint directory the headset and ``ohho serve --checkpoint`` share.
CHECKPOINT_REL = "checkpoints/act_vr_sim_pick/best"
DATASET_REPO_ID = "local/omnibot-sim-vr-pick"
# Container path from the OHH-237 smoke. Not created by this repo.
LOCAL_EPISODE_ROOT = "/tmp/omnibot-sim-vr-pick"
# Card committed on branch ohh-237-sim-vr-pick. Not present on this branch.
DATASET_CARD = "datasets/omnibot-sim-vr-pick"
HF_DATASET_ID = "ohho-robotics/omnibot-sim-vr-pick"
HF_DATASET_PUBLISHED = False
HF_CHECKPOINT_ID = "ohho-robotics/omnibot-act-vr-sim-pick"
HF_CHECKPOINT_PUBLISHED = False
TRIAL_COUNT = 20
TRIAL_SEED_START = 237
CONFIG_PATH = Path(__file__).resolve().parent / "configs" / "act_vr_sim_pick.yaml"


def models_payload(checkpoint: str = CHECKPOINT_REL) -> dict[str, Any]:
    """JSON body for ``GET /models`` and the headset param fallback."""
    return {
        "models": [
            {
                "id": POLICY_ID,
                "label": POLICY_LABEL,
                "checkpoint": checkpoint,
            }
        ]
    }


def hf_dataset_push_steps(local_root: str = LOCAL_EPISODE_ROOT) -> str:
    """Human steps. This function does not upload anything."""
    return (
        "Hugging Face dataset "
        f"{HF_DATASET_ID} is not published. After at least 50 validated\n"
        "LeRobot v2 episodes exist on disk, a person with an HF token runs:\n"
        "  huggingface-cli login\n"
        f"  huggingface-cli upload {HF_DATASET_ID} {local_root} --repo-type dataset\n"
        "Then set dataset.hf_published: true in "
        "lerobot_engine/configs/act_vr_sim_pick.yaml.\n"
        "Checkpoint push (separate, also not done here):\n"
        "  huggingface-cli upload "
        f"{HF_CHECKPOINT_ID} {CHECKPOINT_REL} --repo-type model\n"
    )


def load_train_config(path: str | Path | None = None) -> dict[str, Any]:
    import yaml

    config_path = Path(path) if path else CONFIG_PATH
    with config_path.open(encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle)
    if not isinstance(loaded, dict):
        raise ValueError(f"ACT VR config must be a mapping: {config_path}")
    return loaded


def looks_like_lerobot_v2(root: str | Path) -> bool:
    """True when a directory has a LeRobot v2 meta or parquet tree."""
    path = Path(root)
    if (path / "meta" / "info.json").is_file():
        return True
    data = path / "data"
    return data.is_dir() and any(data.rglob("*.parquet"))


def resolve_dataset(
    config: dict[str, Any],
    *,
    dataset_path: str | None = None,
    smoke: bool = False,
) -> dict[str, Any]:
    """Pick a dataset source. Smoke never pretends a missing root is real data."""
    dataset = config.get("dataset") or {}
    configured = str(dataset.get("local_episode_root") or LOCAL_EPISODE_ROOT)
    hf_id = str(dataset.get("hf_id") or HF_DATASET_ID)
    published = bool(dataset.get("hf_published", HF_DATASET_PUBLISHED))

    if dataset_path:
        present = looks_like_lerobot_v2(dataset_path)
        return {
            "kind": "cli" if present else "cli_missing",
            "path": dataset_path,
            "hf_id": hf_id,
            "hf_published": published,
            "trainable": present,
        }

    if looks_like_lerobot_v2(configured):
        return {
            "kind": "local",
            "path": configured,
            "hf_id": hf_id,
            "hf_published": published,
            "trainable": True,
        }

    if published:
        return {
            "kind": "huggingface",
            "path": hf_id,
            "hf_id": hf_id,
            "hf_published": True,
            "trainable": True,
        }

    if smoke:
        return {
            "kind": "synthetic_smoke",
            "path": None,
            "hf_id": hf_id,
            "hf_published": False,
            "trainable": False,
            "configured_local_episode_root": configured,
        }

    return {
        "kind": "missing",
        "path": configured,
        "hf_id": hf_id,
        "hf_published": False,
        "trainable": False,
        "configured_local_episode_root": configured,
    }


def run_numpy_smoke(
    output_dir: str | Path,
    *,
    steps: int = 2,
    seed: int = 243,
) -> dict[str, Any]:
    """Two CPU steps of a chunked linear map.

    This is a CI smoke so the train command runs without torch or lerobot.
    It is not a LeRobot ACT update and it does not start W&B.
    """
    if steps < 1:
        raise ValueError("smoke steps must be >= 1")

    rng = np.random.RandomState(seed)
    state_dim = 9
    action_dim = 9
    chunk = 4
    n_frames = 16
    states = rng.uniform(-0.5, 0.5, size=(n_frames, state_dim)).astype(np.float32)
    true_w = rng.normal(scale=0.05, size=(chunk * action_dim, state_dim)).astype(
        np.float32
    )
    targets = states @ true_w.T
    weight = np.zeros_like(true_w)
    lr = 0.2
    losses: list[float] = []
    for _ in range(steps):
        error = states @ weight.T - targets
        losses.append(float(np.mean(error**2)))
        grad = (error.T @ states) / n_frames
        weight = weight - lr * grad

    out = Path(output_dir)
    best = out / "best"
    best.mkdir(parents=True, exist_ok=True)
    np.savez(best / "policy_smoke.npz", weight=weight, losses=np.asarray(losses))

    try:
        import torch  # noqa: F401

        torch_available = True
    except ImportError:
        torch_available = False

    manifest: dict[str, Any] = {
        "policy_id": POLICY_ID,
        "checkpoint": CHECKPOINT_REL,
        "backend": "numpy_chunk_linear_smoke",
        "weights_are_lerobot_act": False,
        "wandb_run": None,
        "torch_available": torch_available,
        "steps": steps,
        "seed": seed,
        "losses": losses,
        "dataset_source": "synthetic_smoke",
        "dataset_repo_id": DATASET_REPO_ID,
        "hf_dataset_id": HF_DATASET_ID,
        "hf_dataset_published": False,
        "hf_checkpoint_id": HF_CHECKPOINT_ID,
        "hf_checkpoint_published": False,
        "note": (
            "CPU smoke on synthetic 9-DoF chunks. Not LeRobot ACT weights "
            "and not a W&B run."
        ),
    }
    text = json.dumps(manifest, indent=2) + "\n"
    (best / "train_manifest.json").write_text(text, encoding="utf-8")
    (out / "train_manifest.json").write_text(text, encoding="utf-8")
    return manifest


def plan_full_train(
    config: dict[str, Any],
    dataset_path: str,
    *,
    train_py: str | Path | None = None,
) -> list[str]:
    """Argv for the existing ACT trainer. Does not start it."""
    import sys

    model = (config.get("model") or {})
    training = (config.get("training") or {})
    script = Path(train_py) if train_py else Path(__file__).resolve().parent / "train.py"
    output = str(training.get("output_dir") or "checkpoints/act_vr_sim_pick")
    return [
        sys.executable,
        str(script),
        "--model",
        "act",
        "--dataset-path",
        dataset_path,
        "--output-dir",
        output,
        "--device",
        str(model.get("device") or "cpu"),
        "--num-epochs",
        str(int(training.get("num_epochs") or 100)),
        "--batch-size",
        str(int(training.get("batch_size") or 8)),
        "--lr",
        str(float(training.get("lr") or 1e-4)),
        "--chunk-size",
        str(int(model.get("chunk_size") or 50)),
    ]


def full_train_blockers() -> list[str]:
    missing: list[str] = []
    try:
        import torch  # noqa: F401
    except ImportError:
        missing.append("torch")
    try:
        import lerobot  # noqa: F401
    except ImportError:
        missing.append("lerobot")
    return missing


def run_lerobot_act_cpu_smoke(
    output_dir: str | Path,
    *,
    steps: int = 2,
    seed: int = 243,
) -> dict[str, Any]:
    """Two CPU steps of a tiny LeRobot ACTPolicy.

    No pretrained backbone download, no W&B, no persisted weight file.
    Raises ImportError when lerobot's ACT package is not installed.
    """
    if steps < 1:
        raise ValueError("smoke steps must be >= 1")

    try:
        import torch
        from lerobot.configs.types import FeatureType, PolicyFeature
        from lerobot.constants import ACTION
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
    except ImportError as exc:
        raise ImportError(
            "lerobot ACT is not installed; the numpy smoke does not need it."
        ) from exc

    torch.manual_seed(seed)
    config = ACTConfig(
        chunk_size=4,
        n_action_steps=4,
        dim_model=64,
        n_heads=4,
        dim_feedforward=128,
        n_encoder_layers=1,
        n_decoder_layers=1,
        use_vae=False,
        pretrained_backbone_weights=None,
        dropout=0.0,
    )
    config.input_features = {
        "observation.images.wrist": PolicyFeature(
            type=FeatureType.VISUAL, shape=(3, 64, 64)
        ),
        "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(9,)),
    }
    config.output_features = {
        "action": PolicyFeature(type=FeatureType.ACTION, shape=(9,)),
    }

    def _stat(shape: tuple[int, ...]) -> dict[str, Any]:
        return {"mean": torch.zeros(*shape), "std": torch.ones(*shape)}

    policy = ACTPolicy(
        config,
        dataset_stats={
            "observation.images.wrist": _stat((3, 1, 1)),
            "observation.state": _stat((9,)),
            "action": _stat((9,)),
        },
    )
    policy.train()
    before = torch.nn.utils.parameters_to_vector(policy.parameters()).detach().clone()
    optimizer = torch.optim.AdamW(policy.parameters(), lr=1e-3)
    generator = torch.Generator()
    generator.manual_seed(seed)
    batch = {
        "observation.images.wrist": torch.rand(
            2, 3, 64, 64, generator=generator
        ),
        "observation.state": torch.randn(2, 9, generator=generator),
        ACTION: torch.randn(2, 4, 9, generator=generator),
        "action_is_pad": torch.zeros(2, 4, dtype=torch.bool),
    }
    losses: list[float] = []
    for _ in range(steps):
        optimizer.zero_grad()
        loss, _info = policy.forward(batch)
        loss.backward()
        optimizer.step()
        losses.append(float(loss.detach()))

    after = torch.nn.utils.parameters_to_vector(policy.parameters()).detach()
    changed = not torch.equal(before, after)
    n_params = sum(p.numel() for p in policy.parameters())
    manifest: dict[str, Any] = {
        "policy_id": POLICY_ID,
        "checkpoint": CHECKPOINT_REL,
        "backend": "lerobot_act_cpu_smoke",
        "weights_are_lerobot_act": True,
        "checkpoint_persisted": False,
        "parameters_changed": bool(changed),
        "parameter_count": int(n_params),
        "wandb_run": None,
        "device": "cpu",
        "cuda": bool(torch.cuda.is_available()),
        "pretrained_backbone_weights": None,
        "steps": steps,
        "seed": seed,
        "losses": losses,
        "dataset_source": "synthetic_smoke",
        "image_shape": [3, 64, 64],
        "chunk_size": 4,
        "action_dim": 9,
        "loss_decreased": bool(losses[-1] < losses[0]),
        "note": (
            "Two CPU steps of a randomly initialised LeRobot ACTPolicy on one "
            "synthetic batch. Not a dataset fine-tune and not a W&B run. "
            "Weights were not written to disk."
        ),
    }
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "act_cpu_smoke.json").write_text(
        json.dumps(manifest, indent=2) + "\n",
        encoding="utf-8",
    )
    return manifest


def trial_rows(
    *,
    count: int = TRIAL_COUNT,
    seed_start: int = TRIAL_SEED_START,
) -> list[dict[str, Any]]:
    """Blank 20-trial table. Success stays null until a person scores a run."""
    return [
        {
            "trial": index + 1,
            "seed": seed_start + index,
            "success": None,
            "notes": "",
        }
        for index in range(count)
    ]


def success_rate(trials: list[dict[str, Any]]) -> float | None:
    """Rate only when every trial has a real boolean. Blanks are not failures."""
    if not trials:
        return None
    decided: list[bool] = []
    for trial in trials:
        value = trial.get("success")
        if value is None:
            return None
        if not isinstance(value, bool):
            return None
        decided.append(value)
    return sum(1 for item in decided if item) / len(decided)


def render_trial_markdown(trials: list[dict[str, Any]] | None = None) -> str:
    rows = trials if trials is not None else trial_rows()
    rate = success_rate(rows)
    rate_cell = "not run" if rate is None else f"{rate:.2f}"
    lines = [
        "# ACT VR sim pick — 20-trial scaffold",
        "",
        "Success is blank until someone watches the cube end in the bin.",
        "This file does not record a rate from a run that did not happen.",
        "",
        f"Scored success rate: {rate_cell}",
        "",
        "| trial | seed | success | notes |",
        "| ---: | ---: | --- | --- |",
    ]
    for row in rows:
        success = "" if row.get("success") is None else str(bool(row["success"])).lower()
        lines.append(
            f"| {row['trial']} | {row['seed']} | {success} | {row.get('notes') or ''} |"
        )
    lines.append("")
    return "\n".join(lines)
