#!/usr/bin/env python3
"""Convenience training runner for ACT policy on OmniBot mobile manipulation dataset.

Usage:
------
# Basic run using act.yaml configuration:
python lerobot_engine/train_act.py

# Custom dataset and output paths:
python lerobot_engine/train_act.py \
    --dataset-path /data/lerobot/pick_and_place_cube \
    --output-dir ~/checkpoints/act_run1

# CPU smoke test (1-2 steps):
python lerobot_engine/train_act.py \
    --dataset-path /tmp/fixture_dataset \
    --output-dir /tmp/smoke_out \
    --smoke-test
"""

from __future__ import annotations

import sys
from pathlib import Path

# Ensure repo engines are in sys.path
_ENGINES_DIR = Path(__file__).resolve().parents[1]
if str(_ENGINES_DIR) not in sys.path:
    sys.path.insert(0, str(_ENGINES_DIR))

from lerobot_engine.train import main  # noqa: E402


def run_act_training():
    default_config = (
        Path(__file__).resolve().parent / "configs" / "models" / "act.yaml"
    )

    # Insert default config if neither --config nor --model is specified
    args = sys.argv[1:]
    if "--config" not in args and default_config.is_file():
        sys.argv.extend(["--config", str(default_config)])
    elif "--model" not in args:
        sys.argv.extend(["--model", "act"])

    main()


if __name__ == "__main__":
    run_act_training()
