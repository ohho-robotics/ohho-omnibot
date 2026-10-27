"""Append one live-recorded episode to a LeRobot v2.0 dataset on disk.

Used by the ROS teleop recorder (via ohho-os :class:`~ohho.data.recorder.Recorder`)
and by headless tests. Writes Parquet tabular data, per-camera MP4 files, and
updates ``meta/*`` including optional per-camera timestamp columns for sync
jitter measurement.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from data_engine.recording.dataset_meta import (
    CHUNK_SIZE_EPISODES,
    CODEBASE_VERSION,
    PARQUET_SCHEMA,
    append_jsonl,
    ensure_task,
    load_meta,
    save_meta,
    update_stats,
    write_video,
)
from data_engine.schema.camera_keys import LEROBOT_IMAGE_KEYS
from data_engine.schema.constants import (
    CAMERA_BEV,
    CAMERA_WRIST,
    LEROBOT_CAMERA_KEYS,
    MOBILE_MANIP_ACTION_SPEC,
    MOBILE_MANIP_STATE_SPEC,
    lerobot_image_features,
)

_CAM_KEY_TO_ATTR = {
    "observation.images.wrist": ("wrist_image", "wrist_timestamp", CAMERA_WRIST),
    "observation.images.bev": ("bev_image", "bev_timestamp", CAMERA_BEV),
}

def _rgb_to_bgr(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 3 and frame.shape[2] == 3:
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame


def append_live_episode(
    dataset_root: str | Path,
    frames: list[dict[str, Any]],
    task_name: str,
    fps: int = 30,
    *,
    repo_id: str | None = None,
) -> int:
    """Write one episode from in-memory frames. Returns ``episode_index``.

    Each frame dict must include ``observation.state``, ``action``, and
    ``wrist_image`` / ``bev_image`` (HxWx3 uint8 RGB). Optional keys:
    ``wrist_timestamp``, ``bev_timestamp`` (seconds, same clock).
    """
    if not frames:
        raise ValueError("append_live_episode: empty frame list")

    dataset_root = Path(dataset_root)
    info = load_meta(dataset_root, fps)
    if repo_id:
        info["repo_id"] = repo_id

    episode_idx = info["total_episodes"]
    task_idx = ensure_task(dataset_root, task_name)
    chunk = episode_idx // CHUNK_SIZE_EPISODES
    ep_str = f"episode_{episode_idx:06d}"
    global_offset = info["total_frames"]
    t0 = float(frames[0].get("timestamp", 0.0))

    rows: list[dict[str, Any]] = []
    cam_frames: dict[str, list[np.ndarray]] = {key: [] for key in LEROBOT_CAMERA_KEYS}
    wrist_ts_col: list[float] = []
    bev_ts_col: list[float] = []

    for local_idx, frame in enumerate(frames):
        state = list(frame.get("observation.state", frame.get("state", [])))
        action = list(frame.get("action", []))
        if len(state) != MOBILE_MANIP_STATE_SPEC.dim:
            state = (state + [0.0] * MOBILE_MANIP_STATE_SPEC.dim)[: MOBILE_MANIP_STATE_SPEC.dim]
        if len(action) != MOBILE_MANIP_ACTION_SPEC.dim:
            action = (action + [0.0] * MOBILE_MANIP_ACTION_SPEC.dim)[: MOBILE_MANIP_ACTION_SPEC.dim]

        ts_raw = float(frame.get("timestamp", local_idx / fps))
        rel_ts = ts_raw - t0 if local_idx == 0 else local_idx / fps

        row = {
            "observation.state": state,
            "action": action,
            "timestamp": rel_ts,
            "frame_index": local_idx,
            "episode_index": episode_idx,
            "index": global_offset + local_idx,
            "task_index": task_idx,
            "next.done": local_idx == len(frames) - 1,
        }
        rows.append(row)

        for key, (img_attr, ts_attr, cam_cfg) in _CAM_KEY_TO_ATTR.items():
            img = frame.get(img_attr)
            if img is None:
                img = np.zeros((cam_cfg.height, cam_cfg.width, 3), dtype=np.uint8)
            cam_frames[key].append(_rgb_to_bgr(np.asarray(img, dtype=np.uint8)))
            if ts_attr == "wrist_timestamp":
                wrist_ts_col.append(float(frame.get(ts_attr, ts_raw)))
            else:
                bev_ts_col.append(float(frame.get(ts_attr, ts_raw)))

    data_dir = dataset_root / "data" / f"chunk-{chunk:03d}"
    data_dir.mkdir(parents=True, exist_ok=True)

    table_cols: dict[str, Any] = {col: [r[col] for r in rows] for col in PARQUET_SCHEMA.names}
    table_cols["observation.images.wrist.timestamp"] = wrist_ts_col
    table_cols["observation.images.bev.timestamp"] = bev_ts_col
    schema = pa.schema(
        list(PARQUET_SCHEMA)
        + [
            pa.field("observation.images.wrist.timestamp", pa.float64()),
            pa.field("observation.images.bev.timestamp", pa.float64()),
        ]
    )
    table = pa.Table.from_pydict(table_cols, schema=schema)
    pq.write_table(table, data_dir / f"{ep_str}.parquet", compression="snappy")

    vid_base = dataset_root / "videos" / f"chunk-{chunk:03d}"
    for key, imgs in cam_frames.items():
        write_video(imgs, vid_base / key / f"{ep_str}.mp4", fps)

    ep_length = len(rows)
    info["total_episodes"] += 1
    info["total_frames"] += ep_length
    info["fps"] = fps
    info.setdefault("features", {})
    info["features"].update(
        {
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
        }
    )
    info["codebase_version"] = CODEBASE_VERSION
    save_meta(dataset_root, info)

    append_jsonl(
        dataset_root / "meta" / "episodes.jsonl",
        {
            "episode_index": episode_idx,
            "tasks": [task_idx],
            "length": ep_length,
        },
    )
    update_stats(dataset_root, rows)

    return episode_idx
