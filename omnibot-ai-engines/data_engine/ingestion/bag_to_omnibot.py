"""
bag_to_omnibot.py
-----------------
Convert a single ROS 2 bag into a LeRobot v2.0 dataset episode.

Output layout (appended to an existing or new dataset root):
    <dataset_root>/
        meta/
            info.json
            tasks.jsonl
            episodes.jsonl
            stats.json         (written/updated after all episodes are added)
        data/
            chunk-000/
                episode_XXXXXX.parquet
        videos/
            chunk-000/
                observation.images.wrist/
                    episode_XXXXXX.mp4
                observation.images.bev/
                    episode_XXXXXX.mp4

Usage (CLI):
    python -m data_engine.ingestion.bag_to_omnibot \
        --bag /data/bags/ep000 \
        --dataset /data/lerobot/pick_place \
        --task "pick up the red cube" \
        --fps 30
"""

from __future__ import annotations

from pathlib import Path

import click
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from data_engine.ingestion.ros_parser import ROSBagParser
from data_engine.ingestion.sync_topics import TopicSynchronizer
from data_engine.recording.dataset_meta import (
    CHUNK_SIZE_EPISODES,
    PARQUET_SCHEMA,
    append_jsonl,
    ensure_task,
    load_meta,
    save_meta,
    update_stats,
    write_video,
)
from data_engine.schema.constants import (
    LEROBOT_CAMERA_KEYS,
    MOBILE_MANIP_ACTION_SPEC,
    MOBILE_MANIP_STATE_SPEC,
)


# ── Core conversion ──────────────────────────────────────────────────────────


def bag_to_omnibot(
    bag_path: str | Path,
    dataset_root: str | Path,
    task_name: str,
    fps: int = 30,
) -> int:
    """
    Convert one ROS 2 bag to a LeRobot episode and append it to dataset_root.
    Returns the episode_index assigned.
    """
    bag_path = Path(bag_path)
    dataset_root = Path(dataset_root)

    # ── Load existing metadata ────────────────────────────────────────────
    info = load_meta(dataset_root, fps)
    episode_idx = info["total_episodes"]
    task_idx = ensure_task(dataset_root, task_name)
    chunk = episode_idx // CHUNK_SIZE_EPISODES
    ep_str = f"episode_{episode_idx:06d}"

    # ── Parse bag ─────────────────────────────────────────────────────────
    parser = ROSBagParser(str(bag_path))
    raw = parser.extract_all()  # returns dict of topic→list[(ts, data)]

    syncer = TopicSynchronizer(fps=fps)
    frames_sync = syncer.synchronize(raw)  # list of dicts, one per target frame

    if not frames_sync:
        raise RuntimeError(f"No synchronized frames found in {bag_path}")

    global_offset = info["total_frames"]
    t0 = frames_sync[0]["timestamp"]

    rows: list[dict] = []
    cam_frames: dict[str, list[np.ndarray]] = {key: [] for key in LEROBOT_CAMERA_KEYS}

    for local_idx, frame in enumerate(frames_sync):
        state = frame.get("state", [0.0] * MOBILE_MANIP_STATE_SPEC.dim)
        action = frame.get("action", [0.0] * MOBILE_MANIP_ACTION_SPEC.dim)

        rows.append(
            {
                "observation.state": state,
                "action": action,
                "timestamp": frame["timestamp"] - t0,
                "frame_index": local_idx,
                "episode_index": episode_idx,
                "index": global_offset + local_idx,
                "task_index": task_idx,
                "next.done": local_idx == len(frames_sync) - 1,
            }
        )

        # Images: syncer returns np arrays (BGR) or None.
        # Stream names follow CameraConfig.name: camera_wrist, camera_bev.
        for key, cam in LEROBOT_CAMERA_KEYS.items():
            image = frame.get(f"camera_{cam.name}")
            cam_frames[key].append(
                image
                if image is not None
                else np.zeros((cam.height, cam.width, 3), np.uint8)
            )

    # ── Write Parquet ─────────────────────────────────────────────────────
    data_dir = dataset_root / "data" / f"chunk-{chunk:03d}"
    data_dir.mkdir(parents=True, exist_ok=True)

    table = pa.Table.from_pydict(
        {col: [r[col] for r in rows] for col in PARQUET_SCHEMA.names},
        schema=PARQUET_SCHEMA,
    )
    pq.write_table(table, data_dir / f"{ep_str}.parquet", compression="snappy")

    # ── Write videos ──────────────────────────────────────────────────────
    vid_base = dataset_root / "videos" / f"chunk-{chunk:03d}"
    for key, frames in cam_frames.items():
        write_video(frames, vid_base / key / f"{ep_str}.mp4", fps)

    # ── Update metadata ───────────────────────────────────────────────────
    ep_length = len(rows)
    info["total_episodes"] += 1
    info["total_frames"] += ep_length
    info["fps"] = fps
    save_meta(dataset_root, info)

    episodes_path = dataset_root / "meta" / "episodes.jsonl"
    append_jsonl(
        episodes_path,
        {
            "episode_index": episode_idx,
            "tasks": [task_idx],
            "length": ep_length,
        },
    )

    # ── Update running stats ──────────────────────────────────────────────
    update_stats(dataset_root, rows)

    print(
        f"[bag_to_omnibot] episode {episode_idx} written ({ep_length} frames) → {ep_str}"
    )
    return episode_idx


# ── CLI ──────────────────────────────────────────────────────────────────────


@click.command()
@click.option("--bag", required=True, help="Path to ROS 2 bag directory")
@click.option("--dataset", required=True, help="LeRobot dataset root to append to")
@click.option("--task", required=True, help="Natural language task description")
@click.option("--fps", default=30, show_default=True, help="Target frame rate")
def main(bag: str, dataset: str, task: str, fps: int) -> None:
    bag_to_omnibot(bag, dataset, task, fps)


if __name__ == "__main__":
    main()
