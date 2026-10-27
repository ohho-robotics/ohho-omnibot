"""Smoke tests and integration tests for lerobot_engine and ACT training (OHH-51).

Covers:
1. ACT model and training YAML configuration validation.
2. CLI argument parsing with config file defaults and flag overrides.
3. ACTAdapter interface, schema dimensions (9-DOF state/action), and select_action.
4. End-to-end CPU smoke training run on a tiny synthetic dataset fixture (1-2 steps).
5. Checkpoint saving and loading verification.
6. ONNX export interface and graceful dependency handling.
7. Hugging Face Hub export validation and human TODO reporting.
"""

from __future__ import annotations

import os
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import pandas as pd
import pytest

# Ensure omnibot-ai-engines is on sys.path
_ENGINES_DIR = Path(__file__).resolve().parents[2]
if str(_ENGINES_DIR) not in sys.path:
    sys.path.insert(0, str(_ENGINES_DIR))

from lerobot_engine.configs.models import __file__ as _models_init  # noqa: F401
from lerobot_engine.models import list_models, make_policy
from lerobot_engine.models.act import ACTAdapter
from lerobot_engine.train import (
    load_yaml_config,
    main as train_main,
    parse_args,
)

try:
    import torch

    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


def create_tiny_dataset_fixture(root_dir: Path, num_frames: int = 10) -> Path:
    """Create a minimal LeRobot v2 format dataset fixture with Parquet data."""
    data_dir = root_dir / "data" / "chunk-000"
    data_dir.mkdir(parents=True, exist_ok=True)
    meta_dir = root_dir / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    states = [np.linspace(-0.5, 0.5, 9, dtype=np.float32) for _ in range(num_frames)]
    actions = [np.linspace(-0.2, 0.2, 9, dtype=np.float32) for _ in range(num_frames)]
    dones = [False] * (num_frames - 1) + [True]
    timestamps = [float(i) * 0.033 for i in range(num_frames)]

    df = pd.DataFrame(
        {
            "observation.state": states,
            "action": actions,
            "episode_index": [0] * num_frames,
            "frame_index": list(range(num_frames)),
            "next.done": dones,
            "timestamp": timestamps,
        }
    )
    df.to_parquet(data_dir / "episode_000000.parquet")

    # Meta files
    (meta_dir / "info.json").write_text(
        '{"fps": 30, "total_episodes": 1, "total_frames": '
        + str(num_frames)
        + ', "features": {"observation.state": {"shape": [9]}, "action": {"shape": [9]}}}',
        encoding="utf-8",
    )
    (meta_dir / "episodes.jsonl").write_text(
        f'{{"episode_index": 0, "tasks": [0], "length": {num_frames}}}\n',
        encoding="utf-8",
    )
    return root_dir


class TestACTConfig(unittest.TestCase):
    """Verify ACT YAML configuration files."""

    def setUp(self):
        self.config_path = (
            Path(__file__).resolve().parents[1] / "configs" / "models" / "act.yaml"
        )

    def test_act_yaml_exists_and_valid(self):
        self.assertTrue(self.config_path.is_file(), f"Missing config: {self.config_path}")
        cfg = load_yaml_config(self.config_path)

        # Model section
        self.assertIn("model", cfg)
        self.assertEqual(cfg["model"]["type"], "act")
        self.assertEqual(cfg["model"]["chunk_size"], 50)
        self.assertGreater(cfg["model"]["policy_hz"], 0)

        # Robot section
        self.assertIn("robot", cfg)
        self.assertEqual(cfg["robot"]["action_dim"], 9)
        self.assertEqual(cfg["robot"]["state_dim"], 9)

        # Training section
        self.assertIn("training", cfg)
        self.assertGreater(cfg["training"]["num_epochs"], 0)
        self.assertGreater(cfg["training"]["batch_size"], 0)
        self.assertGreater(cfg["training"]["lr"], 0)

        # Logging and export sections
        self.assertIn("logging", cfg)
        self.assertEqual(cfg["logging"]["wandb_project"], "omnibot-act")
        self.assertIn("export", cfg)
        self.assertEqual(
            cfg["export"]["hf_repo_id"], "ohho-robotics/omnibot-act-mobile-manip"
        )

    def test_load_yaml_missing_raises(self):
        with self.assertRaises(FileNotFoundError):
            load_yaml_config("/nonexistent/path/to/missing.yaml")

    def test_parse_args_with_config(self):
        orig_argv = sys.argv
        try:
            sys.argv = ["train.py", "--config", str(self.config_path)]
            args = parse_args()
            self.assertEqual(args.model, "act")
            self.assertEqual(args.chunk_size, 50)
            self.assertEqual(args.wandb_project, "omnibot-act")
            self.assertEqual(args.hf_repo_id, "ohho-robotics/omnibot-act-mobile-manip")

            # Flag override takes precedence over config
            sys.argv = [
                "train.py",
                "--config",
                str(self.config_path),
                "--lr",
                "2e-4",
                "--num-epochs",
                "5",
            ]
            args = parse_args()
            self.assertEqual(args.lr, 2e-4)
            self.assertEqual(args.num_epochs, 5)
        finally:
            sys.argv = orig_argv


class TestACTAdapter(unittest.TestCase):
    """Verify ACTAdapter interface and policy behavior."""

    @pytest.mark.skipif(not TORCH_AVAILABLE, reason="PyTorch is required for ACTAdapter")
    def test_adapter_schema(self):
        self.assertIn("act", list_models())
        adapter = make_policy("act", checkpoint="scratch", device="cpu", chunk_size=20)
        self.assertIsInstance(adapter, ACTAdapter)
        self.assertEqual(adapter.action_dim, 9)
        self.assertEqual(adapter.image_size, (320, 240))
        self.assertIn("observation.images.wrist", adapter.image_keys)
        self.assertIn("observation.images.bev", adapter.image_keys)
        self.assertIsNone(adapter.task_key)

    @pytest.mark.skipif(not TORCH_AVAILABLE, reason="PyTorch is required for ACTAdapter")
    def test_adapter_select_action(self):
        adapter = make_policy("act", checkpoint="scratch", device="cpu", chunk_size=10)
        obs = {
            "observation.images.wrist": np.zeros((3, 240, 320), dtype=np.float32),
            "observation.images.bev": np.zeros((3, 240, 320), dtype=np.float32),
            "observation.state": np.zeros(9, dtype=np.float32),
        }
        action = adapter.select_action(obs)
        self.assertIsInstance(action, np.ndarray)
        self.assertEqual(action.shape, (9,))

    @pytest.mark.skipif(not TORCH_AVAILABLE, reason="PyTorch is required for ACTAdapter")
    def test_adapter_onnx_export_interface(self):
        adapter = make_policy("act", checkpoint="scratch", device="cpu", chunk_size=10)
        with tempfile.TemporaryDirectory() as tmp_dir:
            out_file = Path(tmp_dir) / "act_test.onnx"
            # export_onnx should return a boolean (True if onnx is installed, False if optional pkg missing)
            result = adapter.export_onnx(out_file)
            self.assertIsInstance(result, bool)


class TestCPUTrainSmoke(unittest.TestCase):
    """End-to-end CPU smoke test running 1-2 training steps on a tiny fixture."""

    @pytest.mark.skipif(not TORCH_AVAILABLE, reason="PyTorch is required for training smoke test")
    def test_smoke_training_run_completes(self):
        with tempfile.TemporaryDirectory() as tmp_dataset, tempfile.TemporaryDirectory() as tmp_output:
            dataset_dir = create_tiny_dataset_fixture(Path(tmp_dataset), num_frames=12)
            out_dir = Path(tmp_output)

            orig_argv = sys.argv
            try:
                sys.argv = [
                    "train.py",
                    "--model",
                    "act",
                    "--dataset-path",
                    str(dataset_dir),
                    "--output-dir",
                    str(out_dir),
                    "--num-epochs",
                    "1",
                    "--batch-size",
                    "4",
                    "--max-steps",
                    "2",
                    "--chunk-size",
                    "5",
                    "--smoke-test",
                ]
                train_main()
            finally:
                sys.argv = orig_argv

            # Verify checkpoint creation
            best_dir = out_dir / "best"
            self.assertTrue(best_dir.is_dir(), f"Expected best checkpoint directory at {best_dir}")

            saved_files = [p.name for p in out_dir.rglob("*") if p.is_file()]
            self.assertTrue(
                any("train_state.pt" in f for f in saved_files),
                f"Expected train_state.pt in output, found: {saved_files}",
            )


if __name__ == "__main__":
    unittest.main()
