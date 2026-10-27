"""Headless tests for ohho-os Recorder bridge (no ROS spin)."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[3]
ENGINES = REPO / "omnibot-ai-engines"
sys.path.insert(0, str(REPO / "omnibot-ai-ros2" / "omnibot_lerobot"))
if str(ENGINES) not in sys.path:
    sys.path.insert(0, str(ENGINES))

try:
    import ohho  # noqa: F401
except ImportError:
    ohho = None

# A module-level importorskip raises during collection. On pytest 7.4 that
# aborts the rest of this package, so the VR and session tests never run.
pytestmark = pytest.mark.skipif(ohho is None, reason="ohho is not installed")


def test_ohho_recorder_writes_valid_dataset(tmp_path: Path):
    from omnibot_lerobot.ohho_teleop_recorder import OhhoTeleopRecorder
    from data_engine.scripts.validate_dataset import validate

    rec = OhhoTeleopRecorder("local/unit_test", fps=10.0)
    rec.start_episode("test task")
    for i in range(3):
        state = np.linspace(0, 0.1, 9, dtype=np.float32)
        action = state.copy()
        wrist = np.zeros((8, 8, 3), dtype=np.uint8) + i
        bev = np.zeros((8, 8, 3), dtype=np.uint8) + i * 2
        rec.capture_frame(
            state,
            action,
            wrist,
            bev,
            wall_timestamp=float(i) / 10.0,
            wrist_timestamp=float(i) / 10.0,
            bev_timestamp=float(i) / 10.0 + 0.005,
        )
    out = tmp_path / "dataset"
    os.makedirs(out, exist_ok=True)
    rec.save_episode(str(out), "test task", 10.0, "local/unit_test")
    assert validate(out, verbose=False)
