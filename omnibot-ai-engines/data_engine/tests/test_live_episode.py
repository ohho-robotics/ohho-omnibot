"""Live episode writer + sync jitter (headless, no ROS)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

from data_engine.recording.live_episode import append_live_episode
from data_engine.recording.sync_jitter import measure_dataset_skew
from data_engine.schema.camera_keys import LEROBOT_IMAGE_KEYS
from data_engine.scripts.validate_dataset import validate


def _frame(i: int, wrist_t: float, bev_t: float) -> dict:
    return {
        "observation.state": [float(i) * 0.01] * 9,
        "action": [0.0] * 9,
        "timestamp": float(i) / 30.0,
        "wrist_image": np.full((8, 8, 3), i % 255, dtype=np.uint8),
        "bev_image": np.full((8, 8, 3), (i * 2) % 255, dtype=np.uint8),
        "wrist_timestamp": wrist_t,
        "bev_timestamp": bev_t,
    }


def test_append_live_episode_writes_parquet_mp4_and_validates(tmp_path: Path):
    root = tmp_path / "ds"
    frames = [
        _frame(0, 0.0, 0.0),
        _frame(1, 1.0 / 30.0, 1.0 / 30.0 + 0.010),
        _frame(2, 2.0 / 30.0, 2.0 / 30.0),
    ]
    ep_idx = append_live_episode(
        root,
        frames,
        "pick cube",
        fps=30,
        repo_id="local/test",
    )
    assert ep_idx == 0
    assert validate(root, verbose=False)

    pq_path = root / "data" / "chunk-000" / "episode_000000.parquet"
    table = pq.read_table(pq_path)
    assert "observation.images.wrist.timestamp" in table.schema.names

    for key in LEROBOT_IMAGE_KEYS:
        mp4 = root / "videos" / "chunk-000" / key / "episode_000000.mp4"
        assert mp4.is_file() and mp4.stat().st_size > 0

    info = json.loads((root / "meta" / "info.json").read_text())
    assert info.get("repo_id") == "local/test"


def test_measure_sync_jitter_on_fixture(tmp_path: Path):
    root = tmp_path / "ds"
    frames = [
        _frame(0, 0.0, 0.0),
        _frame(1, 1.0 / 30.0, 1.0 / 30.0 + 0.020),
    ]
    append_live_episode(root, frames, "task", fps=30)
    reports = measure_dataset_skew(root)
    assert len(reports) == 1
    assert reports[0].wrist_bev_skew_s["max"] >= 0.019
