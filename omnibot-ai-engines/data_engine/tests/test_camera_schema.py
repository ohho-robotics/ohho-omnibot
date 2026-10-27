"""Wrist + bird's-eye view is the only LeRobot camera schema.

Generates a few-frame fixture (no committed video binaries) and runs
``validate_dataset``. Also locks recorders, the policy topic map, and the
lerobot robot config to ``data_engine.schema.camera_keys``.
"""

from __future__ import annotations

import ast
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq

from data_engine.schema.camera_keys import LEROBOT_IMAGE_KEYS
from data_engine.schema.constants import LEROBOT_CAMERA_KEYS, lerobot_image_features
from data_engine.scripts.validate_dataset import validate

ENGINES = Path(__file__).resolve().parents[2]
REPO = ENGINES.parent
N_FRAMES = 3


def _write_tiny_dataset(root: Path, cam_keys: tuple[str, ...]) -> None:
    meta = root / "meta"
    meta.mkdir(parents=True)
    info = {
        "codebase_version": "v2.0",
        "fps": 30,
        "total_episodes": 1,
        "total_frames": N_FRAMES,
        "features": {key: {"dtype": "video"} for key in cam_keys},
    }
    (meta / "info.json").write_text(json.dumps(info))
    (meta / "tasks.jsonl").write_text(
        json.dumps({"task_index": 0, "task": "pick up the cube"}) + "\n"
    )
    (meta / "episodes.jsonl").write_text(
        json.dumps({"episode_index": 0, "tasks": [0], "length": N_FRAMES}) + "\n"
    )
    (meta / "stats.json").write_text("{}\n")

    table = pa.table(
        {
            "observation.state": [[0.0] * 9 for _ in range(N_FRAMES)],
            "action": [[0.0] * 9 for _ in range(N_FRAMES)],
            "timestamp": [0.0, 1.0 / 30.0, 2.0 / 30.0],
            "frame_index": [0, 1, 2],
            "episode_index": [0, 0, 0],
            "index": [0, 1, 2],
            "task_index": [0, 0, 0],
            "next.done": [False, False, True],
        }
    )
    data_dir = root / "data" / "chunk-000"
    data_dir.mkdir(parents=True)
    pq.write_table(table, data_dir / "episode_000000.parquet")

    for key in cam_keys:
        video_dir = root / "videos" / "chunk-000" / key
        video_dir.mkdir(parents=True)
        # Validator checks that the file exists. Three placeholder frames.
        (video_dir / "episode_000000.mp4").write_bytes(b"frame-0\nframe-1\nframe-2\n")


def test_shared_keys_are_wrist_and_bev():
    assert LEROBOT_IMAGE_KEYS == (
        "observation.images.wrist",
        "observation.images.bev",
    )
    assert tuple(LEROBOT_CAMERA_KEYS) == LEROBOT_IMAGE_KEYS
    features = lerobot_image_features(30)
    assert tuple(features) == LEROBOT_IMAGE_KEYS
    assert features["observation.images.wrist"]["shape"] == [480, 640, 3]
    assert features["observation.images.bev"]["shape"] == [800, 800, 3]
    assert "observation.images.front" not in features


def test_wrist_bev_fixture_passes_validate_dataset(tmp_path: Path):
    root = tmp_path / "wrist_bev"
    _write_tiny_dataset(root, LEROBOT_IMAGE_KEYS)

    env = os.environ.copy()
    prev = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = str(ENGINES) + (os.pathsep + prev if prev else "")
    proc = subprocess.run(
        [
            sys.executable,
            "-m",
            "data_engine.scripts.validate_dataset",
            "--dataset",
            str(root),
        ],
        cwd=str(ENGINES),
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "All checks passed" in proc.stdout
    assert validate(root, verbose=False) is True


def test_front_wrist_fixture_is_rejected(tmp_path: Path):
    root = tmp_path / "front_wrist"
    _write_tiny_dataset(
        root,
        ("observation.images.front", "observation.images.wrist"),
    )
    assert validate(root, verbose=False) is False


def test_recorders_configs_and_policy_match_shared_keys():
    yaml_text = (
        ENGINES / "lerobot_engine" / "configs" / "robot" / "omnibot_mobile_manip.yaml"
    ).read_text(encoding="utf-8")
    yaml_keys = tuple(re.findall(r"^\s*- key:\s*(\S+)\s*$", yaml_text, re.M))
    assert yaml_keys == LEROBOT_IMAGE_KEYS

    policy_src = (
        REPO
        / "omnibot-ai-ros2"
        / "omnibot_lerobot"
        / "omnibot_lerobot"
        / "policy_node.py"
    ).read_text(encoding="utf-8")
    maps = []
    for node in ast.walk(ast.parse(policy_src)):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id == "_MAP":
                maps.append(ast.literal_eval(node.value))
    assert len(maps) == 1
    assert "observation.images.front" not in maps[0]
    for key in LEROBOT_IMAGE_KEYS:
        assert key in maps[0]
    assert maps[0]["observation.images.wrist"] == "/camera/wrist/image_raw"
    assert maps[0]["observation.images.bev"] == "/camera/base/bev/image_raw"

    recorders = [
        ENGINES / "lerobot_engine" / "record.py",
        REPO
        / "omnibot-ai-ros2"
        / "omnibot_lerobot"
        / "omnibot_lerobot"
        / "teleop_recorder_node.py",
        REPO
        / "omnibot-ai-ros2"
        / "omnibot_lerobot"
        / "omnibot_lerobot"
        / "ohho_teleop_recorder.py",
    ]
    for path in recorders:
        text = path.read_text(encoding="utf-8")
        assert "observation.images.front" not in text
        for key in LEROBOT_IMAGE_KEYS:
            assert key in text

    for rel in (
        "lerobot_engine/models/base.py",
        "lerobot_engine/models/act.py",
        "lerobot_engine/models/diffusion.py",
        "lerobot_engine/models/smolvla.py",
        "data_engine/visualization/visualize_episode.py",
        "data_engine/ingestion/bag_to_omnibot.py",
        "data_engine/README.md",
        "data_engine/TRAINING_GUIDE.md",
    ):
        text = (ENGINES / rel).read_text(encoding="utf-8")
        assert "observation.images.front" not in text, rel

    from learning_engine.data.schema import (
        IMAGE_KEYS,
        LEROBOT_KEY_MAP,
        OBS_IMAGE_BEV,
        OBS_IMAGE_WRIST,
    )

    assert IMAGE_KEYS == (OBS_IMAGE_WRIST, OBS_IMAGE_BEV)
    assert LEROBOT_KEY_MAP[OBS_IMAGE_WRIST] == "observation.images.wrist"
    assert LEROBOT_KEY_MAP[OBS_IMAGE_BEV] == "observation.images.bev"
    assert "observation.images.front" not in LEROBOT_KEY_MAP.values()
