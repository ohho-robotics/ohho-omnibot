"""LeRobot v2 dataset metadata helpers (no rosbags dependency)."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pyarrow as pa

from data_engine.schema.constants import (
    MOBILE_MANIP_ACTION_SPEC,
    MOBILE_MANIP_STATE_SPEC,
    lerobot_image_features,
)

CHUNK_SIZE_EPISODES = 1000
CODEBASE_VERSION = "v2.0"

PARQUET_SCHEMA = pa.schema(
    [
        pa.field("observation.state", pa.list_(pa.float32())),
        pa.field("action", pa.list_(pa.float32())),
        pa.field("timestamp", pa.float64()),
        pa.field("frame_index", pa.int64()),
        pa.field("episode_index", pa.int64()),
        pa.field("index", pa.int64()),
        pa.field("task_index", pa.int64()),
        pa.field("next.done", pa.bool_()),
    ]
)


def load_meta(dataset_root: Path, fps: int = 30) -> dict[str, Any]:
    info_path = dataset_root / "meta" / "info.json"
    if info_path.exists():
        return json.loads(info_path.read_text(encoding="utf-8"))
    return {
        "codebase_version": CODEBASE_VERSION,
        "robot_type": "omnibot",
        "fps": fps,
        "total_episodes": 0,
        "total_frames": 0,
        "chunks_size": CHUNK_SIZE_EPISODES,
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": {
            "observation.state": {
                "dtype": "float32",
                "shape": [MOBILE_MANIP_STATE_SPEC.dim],
                "names": MOBILE_MANIP_STATE_SPEC.names,
            },
            "action": {
                "dtype": "float32",
                "shape": [MOBILE_MANIP_ACTION_SPEC.dim],
                "names": MOBILE_MANIP_ACTION_SPEC.names,
            },
            **lerobot_image_features(fps),
        },
    }


def save_meta(dataset_root: Path, info: dict) -> None:
    meta_dir = dataset_root / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)
    (meta_dir / "info.json").write_text(json.dumps(info, indent=2))


def append_jsonl(path: Path, record: dict) -> None:
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(record) + "\n")


def ensure_task(dataset_root: Path, task_name: str) -> int:
    tasks_path = dataset_root / "meta" / "tasks.jsonl"
    tasks_path.parent.mkdir(parents=True, exist_ok=True)
    if tasks_path.exists():
        for line in tasks_path.read_text(encoding="utf-8").splitlines():
            rec = json.loads(line)
            if rec["task"] == task_name:
                return rec["task_index"]
    count = sum(1 for _ in tasks_path.open(encoding="utf-8")) if tasks_path.exists() else 0
    append_jsonl(tasks_path, {"task_index": count, "task": task_name})
    return count


def write_video(frames: list[np.ndarray], path: Path, fps: int) -> None:
    if not frames:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    h, w = frames[0].shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(path), fourcc, fps, (w, h))
    for frame in frames:
        if frame.shape[:2] != (h, w):
            frame = cv2.resize(frame, (w, h))
        writer.write(frame)
    writer.release()


def compute_stats(states: list[list[float]], actions: list[list[float]]) -> dict:
    s = np.array(states, dtype=np.float32)
    a = np.array(actions, dtype=np.float32)
    return {
        "observation.state": {
            "mean": s.mean(axis=0).tolist(),
            "std": s.std(axis=0).tolist(),
            "min": s.min(axis=0).tolist(),
            "max": s.max(axis=0).tolist(),
        },
        "action": {
            "mean": a.mean(axis=0).tolist(),
            "std": a.std(axis=0).tolist(),
            "min": a.min(axis=0).tolist(),
            "max": a.max(axis=0).tolist(),
        },
    }


def update_stats(dataset_root: Path, rows: list[dict]) -> None:
    stats_path = dataset_root / "meta" / "stats.json"
    new_states = [r["observation.state"] for r in rows]
    new_actions = [r["action"] for r in rows]
    new_stats = compute_stats(new_states, new_actions)

    if not stats_path.exists():
        stats_path.write_text(json.dumps(new_stats, indent=2))
        return

    old = json.loads(stats_path.read_text(encoding="utf-8"))
    old["_num_episodes"] = old.get("_num_episodes", 1)

    for key in ("observation.state", "action"):
        for stat in ("min", "max"):
            fn = min if stat == "min" else max
            old[key][stat] = [
                fn(a, b) for a, b in zip(old[key][stat], new_stats[key][stat])
            ]
        old_mean = old[key]["mean"]
        old_std = old[key]["std"]
        new_mean = new_stats[key]["mean"]
        new_std = new_stats[key]["std"]
        n_old = old["_num_episodes"]
        n_new = 1
        merged_mean = []
        merged_std = []
        for i in range(len(old_mean)):
            m_a, m_b = old_mean[i], new_mean[i]
            s_a, s_b = old_std[i], new_std[i]
            mc = (n_old * m_a + n_new * m_b) / (n_old + n_new)
            var_a, var_b = s_a**2, s_b**2
            vc = (
                n_old * (var_a + (m_a - mc) ** 2) + n_new * (var_b + (m_b - mc) ** 2)
            ) / (n_old + n_new)
            merged_mean.append(mc)
            merged_std.append(math.sqrt(vc))
        old[key]["mean"] = merged_mean
        old[key]["std"] = merged_std

    old["_num_episodes"] += 1
    stats_path.write_text(json.dumps(old, indent=2))
