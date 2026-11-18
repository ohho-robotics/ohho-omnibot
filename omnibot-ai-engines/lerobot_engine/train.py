#!/usr/bin/env python3
"""Fine-tune any registered visuomotor policy on OmniBot mobile manipulation data.

Usage
-----
# ACT baseline via config (recommended for OHH-51)
python lerobot_engine/train.py --config lerobot_engine/configs/models/act.yaml

# ACT with explicit flags
python lerobot_engine/train.py \
    --model act \
    --dataset-path /data/lerobot/pick_and_place_cube \
    --output-dir ~/checkpoints/act_run1 \
    --num-epochs 100 --batch-size 16 --lr 1e-4 --chunk-size 50 \
    --wandb-project omnibot-act \
    --wandb-run-name "act-baseline-v1" \
    --export-onnx

# SmolVLA fine-tune
python lerobot_engine/train.py \
    --model smolvla \
    --checkpoint lerobot/smolvla_base \
    --dataset-path ~/datasets/mobile_manipulation \
    --output-dir ~/checkpoints/smolvla_run1 \
    --wandb-project omnibot_smolvla \
    --wandb-run-name "smolvla-lr1e-4-bs8"

# Fast CPU smoke test (1-2 steps for CI verification)
python lerobot_engine/train.py \
    --model act \
    --dataset-path /tmp/fixture_dataset \
    --output-dir /tmp/smoke_out \
    --num-epochs 1 \
    --max-steps 2 \
    --device cpu \
    --smoke-test

Available models
----------------
    python lerobot_engine/train.py --list-models
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import Any, Optional

try:
    import yaml

    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

try:
    import torch
    import torch.nn as nn
    from torch.optim import AdamW
    from torch.optim.lr_scheduler import CosineAnnealingLR
    from torch.utils.data import DataLoader, Subset

    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

# Try both LeRobot >= 0.3 and older layouts, falling back to LeRobotDatasetLite
LEROBOT_AVAILABLE = False
LeRobotDataset = None

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    LEROBOT_AVAILABLE = True
except ImportError:
    try:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

        LEROBOT_AVAILABLE = True
    except ImportError:
        try:
            from data_engine.loader.dataset import LeRobotDatasetLite as LeRobotDataset
        except ImportError:
            LeRobotDataset = None

try:
    import wandb

    WANDB_AVAILABLE = True
except ImportError:
    WANDB_AVAILABLE = False

from lerobot_engine.models import list_models, make_policy

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Defaults per model type
# ---------------------------------------------------------------------------

_MODEL_DEFAULTS = {
    "smolvla": {"checkpoint": "lerobot/smolvla_base", "lr": 1e-4, "batch_size": 8, "chunk_size": 50},
    "act": {"checkpoint": "lerobot/act_base", "lr": 1e-4, "batch_size": 16, "chunk_size": 50},
    "diffusion": {
        "checkpoint": "lerobot/diffusion_pusht",
        "lr": 1e-4,
        "batch_size": 64,
        "chunk_size": 16,
    },
    "openvla": {"checkpoint": "openvla/openvla-7b", "lr": 2e-5, "batch_size": 4, "chunk_size": 1},
}


# ---------------------------------------------------------------------------
# Config loader
# ---------------------------------------------------------------------------


def load_yaml_config(config_path: str | Path) -> dict[str, Any]:
    """Load configuration from a YAML file."""
    path = Path(config_path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"Configuration file not found: {path}")
    if not YAML_AVAILABLE:
        raise ImportError("PyYAML is required to parse configuration: pip install pyyaml")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data


# ---------------------------------------------------------------------------
# Args
# ---------------------------------------------------------------------------


def parse_args():
    parser = argparse.ArgumentParser(
        description="Fine-tune a visuomotor policy on OmniBot mobile manipulation data.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to YAML configuration file (e.g. lerobot_engine/configs/models/act.yaml).",
    )
    parser.add_argument(
        "--model",
        type=str,
        default="smolvla",
        help="Model type from the registry. Run --list-models to see all options.",
    )
    parser.add_argument(
        "--list-models",
        action="store_true",
        help="Print registered model names and exit.",
    )
    parser.add_argument(
        "--checkpoint",
        type=str,
        default=None,
        help="HuggingFace hub ID or local path for the pretrained checkpoint.",
    )
    parser.add_argument(
        "--dataset-path",
        type=str,
        required=False,
        default=None,
        help="Path to the LeRobot dataset directory.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="~/checkpoints/policy_run",
        help="Directory to save checkpoints.",
    )
    parser.add_argument("--num-epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--device", type=str, default="cuda")
    parser.add_argument("--chunk-size", type=int, default=50)
    parser.add_argument("--save-every", type=int, default=10)
    parser.add_argument("--grad-clip-norm", type=float, default=10.0)
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="Maximum number of training batches per epoch (useful for smoke tests).",
    )
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="Enable CPU smoke-test mode (runs 1-2 steps, skips network calls).",
    )

    # W&B experiment tracking
    parser.add_argument(
        "--wandb-project",
        type=str,
        default="",
        help="W&B project name. Leave empty to disable W&B logging.",
    )
    parser.add_argument(
        "--wandb-run-name",
        type=str,
        default=None,
        help="W&B run name (auto-generated when omitted).",
    )

    # Hugging Face Hub checkpoint export
    parser.add_argument(
        "--push-to-hub",
        action="store_true",
        help="Push final checkpoint to Hugging Face Hub (requires HF credentials).",
    )
    parser.add_argument(
        "--hf-repo-id",
        type=str,
        default=None,
        help="Hugging Face repo id (e.g. 'ohho-robotics/omnibot-act-mobile-manip').",
    )

    # ONNX export
    parser.add_argument(
        "--export-onnx",
        action="store_true",
        help="Export policy to ONNX format upon completion (if supported).",
    )
    parser.add_argument(
        "--onnx-output-path",
        type=str,
        default=None,
        help="Destination path for exported ONNX file.",
    )

    args = parser.parse_args()

    # Apply config overrides if --config is provided
    if args.config:
        cfg = load_yaml_config(args.config)
        model_sec = cfg.get("model", {})
        training_sec = cfg.get("training", {})
        dataset_sec = cfg.get("dataset", {})
        logging_sec = cfg.get("logging", {})
        export_sec = cfg.get("export", {})

        import sys

        def in_cli(*flags):
            return any(f in sys.argv for f in flags)

        if not in_cli("--model") and "type" in model_sec:
            args.model = model_sec["type"]
        if not in_cli("--checkpoint") and "pretrained" in model_sec:
            args.checkpoint = model_sec["pretrained"]
        if not in_cli("--chunk-size") and "chunk_size" in model_sec:
            args.chunk_size = int(model_sec["chunk_size"])
        if not in_cli("--device") and "device" in model_sec:
            args.device = model_sec["device"]

        if not in_cli("--dataset-path") and "path" in dataset_sec:
            args.dataset_path = dataset_sec["path"]

        if not in_cli("--num-epochs") and "num_epochs" in training_sec:
            args.num_epochs = int(training_sec["num_epochs"])
        if not in_cli("--batch-size") and "batch_size" in training_sec:
            args.batch_size = int(training_sec["batch_size"])
        if not in_cli("--lr") and "lr" in training_sec:
            args.lr = float(training_sec["lr"])
        if not in_cli("--grad-clip-norm") and "grad_clip_norm" in training_sec:
            args.grad_clip_norm = float(training_sec["grad_clip_norm"])
        if not in_cli("--save-every") and "save_every" in training_sec:
            args.save_every = int(training_sec["save_every"])
        if not in_cli("--num-workers") and "num_workers" in training_sec:
            args.num_workers = int(training_sec["num_workers"])
        if not in_cli("--output-dir") and "output_dir" in training_sec:
            args.output_dir = training_sec["output_dir"]

        if not in_cli("--wandb-project") and "wandb_project" in logging_sec:
            args.wandb_project = logging_sec["wandb_project"] or ""
        if not in_cli("--wandb-run-name") and "wandb_run_name" in logging_sec:
            args.wandb_run_name = logging_sec["wandb_run_name"]

        if not in_cli("--push-to-hub") and export_sec.get("push_to_hub"):
            args.push_to_hub = True
        if not in_cli("--hf-repo-id") and "hf_repo_id" in export_sec:
            args.hf_repo_id = export_sec["hf_repo_id"]
        if not in_cli("--export-onnx") and export_sec.get("export_onnx"):
            args.export_onnx = True
        if not in_cli("--onnx-output-path") and "onnx_output_path" in export_sec:
            args.onnx_output_path = export_sec["onnx_output_path"]

    return args


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def get_device(device_str: str):
    if not TORCH_AVAILABLE:
        raise RuntimeError("PyTorch is required for training.")
    if device_str == "cuda" and torch.cuda.is_available():
        return torch.device("cuda")
    if device_str == "cuda":
        print("[WARNING] CUDA not available, falling back to CPU.")
    return torch.device("cpu")


def save_checkpoint(policy_model, optimizer, scheduler, epoch, loss, output_dir):
    ckpt_dir = output_dir / f"checkpoint_epoch_{epoch:04d}"
    ckpt_dir.mkdir(parents=True, exist_ok=True)

    if hasattr(policy_model, "save_pretrained"):
        policy_model.save_pretrained(str(ckpt_dir))
    else:
        torch.save(policy_model.state_dict(), ckpt_dir / "policy.pt")

    torch.save(
        {
            "epoch": epoch,
            "loss": loss,
            "optimizer_state": optimizer.state_dict() if optimizer else None,
            "scheduler_state": scheduler.state_dict() if scheduler else None,
        },
        ckpt_dir / "train_state.pt",
    )
    print(f"  Checkpoint saved -> {ckpt_dir}")
    return ckpt_dir


def _value_for_device(value, device):
    """Move tensors to device. Leave text as-is. Convert other numeric values."""
    if hasattr(value, "to"):
        return value.to(device)
    if isinstance(value, (str, bytes)):
        return value
    if isinstance(value, (list, tuple)) and value and isinstance(value[0], (str, bytes)):
        return value
    try:
        return torch.as_tensor(value, device=device)
    except (TypeError, ValueError, RuntimeError):
        return value


def prepare_batch(batch: dict[str, Any], device, adapter) -> dict[str, Any]:
    """Prepare batch tensors: move to device, pad actions, synthesize missing images."""
    prepared = {k: _value_for_device(v, device) for k, v in batch.items()}

    # ACT and other transformers require 'action_is_pad'
    if "action_is_pad" not in prepared and "action" in prepared:
        prepared["action_is_pad"] = torch.zeros(
            prepared["action"].shape[:2], dtype=torch.bool, device=device
        )

    # When training on Parquet-only lightweight datasets, synthesize expected camera tensors
    if adapter is not None and hasattr(adapter, "image_keys"):
        state_tensor = prepared.get("observation.state")
        batch_size = state_tensor.shape[0] if state_tensor is not None else 1
        img_w, img_h = getattr(adapter, "image_size", (320, 240))
        for key in adapter.image_keys:
            if key not in prepared:
                prepared[key] = torch.zeros(
                    (batch_size, 3, img_h, img_w), dtype=torch.float32, device=device
                )

    return prepared


def compute_eval_loss(policy_model, dataloader, device, adapter, max_steps: Optional[int] = None) -> float:
    was_training = policy_model.training
    policy_model.eval()
    total, n = 0.0, 0
    with torch.no_grad():
        for i, batch in enumerate(dataloader):
            if max_steps is not None and i >= max_steps:
                break
            batch = prepare_batch(batch, device, adapter)
            try:
                out = policy_model.forward(batch)
            except TypeError:
                # Some policies (like LeRobot ACT VAE) require self.training=True to compute latent loss
                policy_model.train()
                out = policy_model.forward(batch)

            if isinstance(out, tuple):
                loss = out[0]
            elif isinstance(out, dict):
                loss = out.get("loss", next(iter(out.values())))
            else:
                loss = out
            total += loss.item()
            n += 1
    if was_training:
        policy_model.train()
    else:
        policy_model.eval()
    return total / max(n, 1)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    args = parse_args()

    if args.list_models:
        print("Registered models:", list_models())
        return

    if not TORCH_AVAILABLE:
        print("[ERROR] PyTorch is required: pip install torch torchvision")
        return

    if args.smoke_test:
        args.device = "cpu"
        args.num_workers = 0
        if args.max_steps is None:
            args.max_steps = 2
        args.num_epochs = min(args.num_epochs, 2)
        args.wandb_project = ""

    if args.dataset_path is None:
        print("[ERROR] --dataset-path or --config is required.")
        return

    # Apply per-model defaults for unset args
    defaults = _MODEL_DEFAULTS.get(args.model, {})
    checkpoint = args.checkpoint or defaults.get("checkpoint")
    if checkpoint is None and not args.smoke_test:
        print(f"[ERROR] --checkpoint is required for model '{args.model}'.")
        return

    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    dataset_path = Path(args.dataset_path).expanduser()
    device = get_device(args.device)

    print("=" * 60)
    print("  OmniBot — Policy Training")
    print("=" * 60)
    print(f"  Model:       {args.model}")
    print(f"  Checkpoint:  {checkpoint}")
    print(f"  Dataset:     {dataset_path}")
    print(f"  Output:      {output_dir}")
    print(f"  Epochs:      {args.num_epochs}")
    print(f"  Batch size:  {args.batch_size}")
    print(f"  LR:          {args.lr}")
    print(f"  Chunk size:  {args.chunk_size}")
    print(f"  Device:      {device}")
    if args.max_steps:
        print(f"  Max steps:   {args.max_steps} batches/epoch")
    print("=" * 60)

    # ── W&B initialisation ─────────────────────────────────────────────────
    wandb_run = None
    if args.wandb_project and WANDB_AVAILABLE and not args.smoke_test:
        wandb_run = wandb.init(
            project=args.wandb_project,
            name=args.wandb_run_name,
            config={
                "model": args.model,
                "checkpoint": checkpoint,
                "num_epochs": args.num_epochs,
                "batch_size": args.batch_size,
                "lr": args.lr,
                "chunk_size": args.chunk_size,
                "grad_clip_norm": args.grad_clip_norm,
                "device": str(device),
            },
        )
        print(f"  W&B run: {wandb_run.url}")
    elif args.wandb_project and not WANDB_AVAILABLE:
        print("[WARNING] --wandb-project set but wandb is not installed. Skipping.")

    # Load dataset
    print("\nLoading dataset...")
    dataset = None
    if dataset_path.is_dir():
        # Check if full LeRobot metadata exists locally
        if (dataset_path / "meta" / "info.json").exists() and LEROBOT_AVAILABLE:
            try:
                dataset = LeRobotDataset(repo_id=dataset_path.name, root=dataset_path)
            except Exception as e:
                logger.debug("LeRobotDataset on local root failed: %s", e)

        # Fall back to LeRobotDatasetLite for Parquet-only datasets
        if dataset is None:
            try:
                from data_engine.loader.dataset import LeRobotDatasetLite

                dataset = LeRobotDatasetLite(dataset_path, chunk_size=args.chunk_size)
            except Exception as e:
                logger.debug("LeRobotDatasetLite failed: %s", e)

    # Remote or Hub dataset
    if dataset is None and LEROBOT_AVAILABLE:
        try:
            dataset = LeRobotDataset(str(args.dataset_path))
        except Exception as e:
            logger.debug("LeRobotDataset from repo id failed: %s", e)

    if dataset is None:
        print(f"[ERROR] Could not load dataset from '{dataset_path}'.")
        print("  Ensure dataset exists and is in LeRobot v2 format (parquet/mp4).")
        return

    dataset_len = len(dataset)
    split = max(1, int(dataset_len * 0.9))
    if split >= dataset_len:
        split = max(1, dataset_len - 1) if dataset_len > 1 else dataset_len

    train_indices = list(range(split))
    eval_indices = list(range(split, dataset_len)) if dataset_len > 1 else list(range(1))

    train_loader = DataLoader(
        Subset(dataset, train_indices),
        batch_size=min(args.batch_size, len(train_indices)),
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=(device.type == "cuda"),
        drop_last=False,
    )
    eval_loader = (
        DataLoader(
            Subset(dataset, eval_indices),
            batch_size=min(args.batch_size, len(eval_indices)),
            shuffle=False,
            num_workers=args.num_workers,
            pin_memory=(device.type == "cuda"),
        )
        if eval_indices
        else None
    )
    print(
        f"  Train: {len(train_indices)} | Eval: {len(eval_indices)} | Batches/epoch: {len(train_loader)}"
    )

    # Load policy via registry
    print(f'\nLoading {args.model} (checkpoint="{checkpoint}")...')
    adapter = make_policy(
        args.model,
        checkpoint=checkpoint,
        device=str(device),
        chunk_size=args.chunk_size,
        scratch=args.smoke_test or (checkpoint in (None, "scratch")),
    )
    policy_model = adapter.model
    if policy_model is None:
        print("[ERROR] This adapter does not expose a trainable model.")
        return

    policy_model.train()
    n_params = sum(p.numel() for p in policy_model.parameters() if p.requires_grad)
    print(f"  Trainable params: {n_params:,}")

    optimizer = AdamW(policy_model.parameters(), lr=args.lr, weight_decay=1e-4)
    scheduler = CosineAnnealingLR(
        optimizer, T_max=max(1, args.num_epochs), eta_min=args.lr * 0.01
    )

    print("\nStarting training...\n")
    best_eval_loss = float("inf")
    global_step = 0
    epoch = 0

    try:
        for epoch in range(1, args.num_epochs + 1):
            policy_model.train()
            epoch_loss, n_batches = 0.0, 0

            for batch_idx, batch in enumerate(train_loader):
                if args.max_steps is not None and batch_idx >= args.max_steps:
                    break

                batch = prepare_batch(batch, device, adapter)
                optimizer.zero_grad()
                out = policy_model.forward(batch)

                if isinstance(out, tuple):
                    loss = out[0]
                elif isinstance(out, dict):
                    loss = out["loss"] if "loss" in out else next(iter(out.values()))
                else:
                    loss = out

                loss.backward()
                grad_norm = nn.utils.clip_grad_norm_(
                    policy_model.parameters(), args.grad_clip_norm
                )
                optimizer.step()
                epoch_loss += loss.item()
                n_batches += 1
                global_step += 1

                log_every = max(1, len(train_loader) // 5)
                if batch_idx % 50 == 0 or batch_idx % log_every == 0:
                    print(
                        f"  Epoch {epoch:4d}/{args.num_epochs} | "
                        f"Batch {batch_idx:4d}/{len(train_loader)} | "
                        f"Loss: {loss.item():.6f}"
                    )
                    if wandb_run is not None:
                        wandb.log(
                            {
                                "train/step_loss": loss.item(),
                                "train/grad_norm": float(grad_norm),
                                "train/step": global_step,
                            },
                            step=global_step,
                        )

            scheduler.step()
            avg_loss = epoch_loss / max(n_batches, 1)
            # Full runs with no eval split stay NaN. Step-capped smoke runs fall
            # back to the training loss so a best checkpoint is still written.
            if eval_loader and len(eval_loader.dataset) > 0:
                eval_loss = compute_eval_loss(
                    policy_model, eval_loader, device, adapter, max_steps=args.max_steps
                )
            elif args.max_steps is not None:
                eval_loss = avg_loss
            else:
                eval_loss = float("nan")
            is_best = eval_loss < best_eval_loss

            if is_best:
                best_eval_loss = eval_loss

            print(
                f"Epoch {epoch:4d}/{args.num_epochs} | "
                f"Train: {avg_loss:.6f} | Eval: {eval_loss:.6f} | "
                f"LR: {optimizer.param_groups[0]['lr']:.2e}"
                + (" [BEST]" if is_best else "")
            )

            if wandb_run is not None:
                wandb.log(
                    {
                        "train/epoch_loss": avg_loss,
                        "eval/loss": eval_loss,
                        "train/lr": optimizer.param_groups[0]["lr"],
                        "eval/is_best": int(is_best),
                        "epoch": epoch,
                    },
                    step=global_step,
                )

            if epoch % args.save_every == 0 or epoch == args.num_epochs:
                ckpt_dir = save_checkpoint(
                    policy_model, optimizer, scheduler, epoch, avg_loss, output_dir
                )
                if wandb_run is not None:
                    artifact = wandb.Artifact(
                        f"omnibot-{args.model}-ckpt",
                        type="model",
                        metadata={"epoch": epoch, "eval_loss": eval_loss},
                    )
                    artifact.add_dir(str(ckpt_dir))
                    wandb_run.log_artifact(artifact, aliases=[f"epoch-{epoch:04d}"])

            if is_best:
                best_dir = output_dir / "best"
                best_dir.mkdir(parents=True, exist_ok=True)
                if hasattr(policy_model, "save_pretrained"):
                    policy_model.save_pretrained(str(best_dir))
                else:
                    torch.save(policy_model.state_dict(), best_dir / "policy.pt")

                if wandb_run is not None:
                    best_artifact = wandb.Artifact(
                        f"omnibot-{args.model}-best",
                        type="model",
                        metadata={"epoch": epoch, "eval_loss": eval_loss},
                    )
                    best_artifact.add_dir(str(best_dir))
                    wandb_run.log_artifact(
                        best_artifact, aliases=["best", f"epoch-{epoch:04d}"]
                    )

    except Exception:
        print(f"\nTraining interrupted at epoch {epoch}")
        raise
    finally:
        if wandb_run is not None:
            wandb.summary["best_eval_loss"] = best_eval_loss
            wandb_run.finish()

    print(f"\nTraining complete. Best eval loss: {best_eval_loss:.6f}")
    print(f"  Best checkpoint: {output_dir / 'best'}")

    # ── Export ONNX if requested ──────────────────────────────────────────
    if args.export_onnx:
        onnx_dest = (
            Path(args.onnx_output_path).expanduser()
            if args.onnx_output_path
            else (output_dir / f"{args.model}_policy.onnx")
        )
        print(f"\nExporting policy to ONNX: {onnx_dest}...")
        if hasattr(adapter, "export_onnx"):
            ok = adapter.export_onnx(onnx_dest)
            if ok:
                print(f"  ONNX export complete: {onnx_dest}")
            else:
                print("  [INFO] ONNX export skipped or unsupported in current environment.")
        else:
            print(f"  [INFO] Model adapter '{args.model}' does not implement export_onnx().")

    # ── Push to Hugging Face Hub if requested ─────────────────────────────
    if args.push_to_hub:
        repo_id = args.hf_repo_id
        if not repo_id:
            print("  [ERROR] --push-to-hub requested but --hf-repo-id was not provided.")
        else:
            print(f"\nPushing best checkpoint to Hugging Face Hub: {repo_id}...")
            best_dir = output_dir / "best"
            try:
                from huggingface_hub import HfApi

                api = HfApi()
                api.upload_folder(
                    folder_path=str(best_dir),
                    repo_id=repo_id,
                    repo_type="model",
                )
                print(f"  Successfully uploaded to https://huggingface.co/{repo_id}")
            except Exception as e:
                print(f"  [WARNING] HF Hub push failed: {e}")
                print(
                    "  TODO (human/Varun): Run `huggingface-cli login` and ensure HF_TOKEN is set."
                )


if __name__ == "__main__":
    main()
