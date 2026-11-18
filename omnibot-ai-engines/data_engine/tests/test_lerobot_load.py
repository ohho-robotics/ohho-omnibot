"""LeRobotDataset load on tiny fixture when lerobot is installed."""

from __future__ import annotations

from pathlib import Path

import pytest

from data_engine.scripts.validate_dataset import validate
from data_engine.scripts.validate_lerobot_load import try_load_lerobot_dataset
import numpy as np

from data_engine.recording.live_episode import append_live_episode


def _minimal_frames(n: int = 3) -> list[dict]:
    return [
        {
            "observation.state": [0.0] * 9,
            "action": [0.0] * 9,
            "timestamp": float(i) / 30.0,
            "wrist_image": np.zeros((8, 8, 3), dtype=np.uint8) + i,
            "bev_image": np.zeros((8, 8, 3), dtype=np.uint8) + i,
            "wrist_timestamp": float(i) / 30.0,
            "bev_timestamp": float(i) / 30.0,
        }
        for i in range(n)
    ]


def test_structural_validator_always_runs(tmp_path: Path):
    root = tmp_path / "tiny"
    append_live_episode(root, _minimal_frames(), "task", fps=30, repo_id="local/ci")
    assert validate(root, verbose=False)


def test_lerobot_dataset_load_when_available(tmp_path: Path):
    root = tmp_path / "tiny"
    append_live_episode(
        root,
        _minimal_frames(),
        "task",
        fps=30,
        repo_id="local/ci_fixture",
    )

    ok, msg = try_load_lerobot_dataset(root)
    if not ok and "lerobot not installed" in msg:
        pytest.skip(msg)
    assert ok, msg
