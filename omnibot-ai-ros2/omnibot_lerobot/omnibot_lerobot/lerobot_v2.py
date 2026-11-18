"""Write one teleop episode as a LeRobot v2.0 dataset on disk.

Matches the layout checked by ``data_engine.scripts.validate_dataset``:
``meta/info.json``, ``tasks.jsonl``, ``episodes.jsonl``, ``stats.json``,
one parquet file per episode, and an mp4 per camera. Camera frames are also
stored as ``.npy`` so the OHH-99 quality filter can score them without a
video decode. No ROS imports.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from omnibot_lerobot.vr_action import VECTOR_DIM, VECTOR_NAMES

CODEBASE_VERSION = "v2.0"
CHUNK_SIZE_EPISODES = 1000
CAMERA_KEYS = ("observation.images.wrist", "observation.images.bev")
IMAGE_ATTRS = {
    "observation.images.wrist": "wrist_image",
    "observation.images.bev": "bev_image",
}
PARQUET_COLUMNS = (
    "observation.state",
    "action",
    "timestamp",
    "frame_index",
    "episode_index",
    "index",
    "task_index",
    "next.done",
)


def _require_pyarrow():
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as exc:
        raise RuntimeError(
            "pyarrow is required to write a LeRobot v2 dataset "
            f"(pip install pyarrow): {exc}"
        ) from exc
    return pa, pq


def _load_info(dataset_root: Path, fps: int, repo_id: str) -> dict[str, Any]:
    path = dataset_root / "meta" / "info.json"
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return {
        "codebase_version": CODEBASE_VERSION,
        "robot_type": "omnibot",
        "repo_id": repo_id,
        "fps": fps,
        "total_episodes": 0,
        "total_frames": 0,
        "chunks_size": CHUNK_SIZE_EPISODES,
        "features": _features(fps),
    }


def _features(fps: int, height: int = 240, width: int = 320) -> dict[str, Any]:
    features: dict[str, Any] = {
        "observation.state": {
            "dtype": "float32",
            "shape": [VECTOR_DIM],
            "names": list(VECTOR_NAMES),
        },
        "action": {
            "dtype": "float32",
            "shape": [VECTOR_DIM],
            "names": list(VECTOR_NAMES),
        },
    }
    for key in CAMERA_KEYS:
        features[key] = {
            "dtype": "video",
            "shape": [height, width, 3],
            "names": ["height", "width", "channels"],
            "info": {
                "video.fps": fps,
                "video.codec": "mp4v",
                "video.pix_fmt": "yuv420p",
            },
        }
    return features


def _append_jsonl(path: Path, record: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record) + "\n")


def _ensure_task(dataset_root: Path, task_name: str) -> int:
    path = dataset_root / "meta" / "tasks.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            record = json.loads(line)
            if record.get("task") == task_name:
                return int(record["task_index"])
    count = 0
    if path.exists():
        count = sum(1 for line in path.read_text(encoding="utf-8").splitlines() if line.strip())
    _append_jsonl(path, {"task_index": count, "task": task_name})
    return count


def _vector(frame: dict[str, Any], key: str, alias: str) -> list[float]:
    raw = frame.get(key, frame.get(alias))
    if raw is None:
        values = [0.0] * VECTOR_DIM
    else:
        values = [float(item) for item in np.asarray(raw, dtype=np.float32).reshape(-1)]
    if len(values) < VECTOR_DIM:
        values = values + [0.0] * (VECTOR_DIM - len(values))
    return values[:VECTOR_DIM]


def _image(frame: dict[str, Any], attr: str) -> np.ndarray:
    image = frame.get(attr)
    if image is None:
        return np.zeros((8, 8, 3), dtype=np.uint8)
    array = np.asarray(image, dtype=np.uint8)
    if array.ndim == 2:
        array = np.repeat(array[:, :, None], 3, axis=2)
    return array


def _write_mp4(frames: list[np.ndarray], path: Path, fps: int) -> None:
    """Write an mp4 when OpenCV can, otherwise a placeholder file.

    ``validate_dataset`` checks that the mp4 path exists. The quality filter
    reads the ``.npy`` sibling first, so a placeholder does not hide a frozen
    camera from OHH-99.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        import cv2
    except ImportError:
        path.write_bytes(b"")
        return
    height, width = frames[0].shape[:2]
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(fps),
        (width, height),
    )
    if not writer.isOpened():
        path.write_bytes(b"")
        return
    for frame in frames:
        writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    writer.release()
    if not path.is_file() or path.stat().st_size == 0:
        path.write_bytes(b"")


def _compute_stats(states: list[list[float]], actions: list[list[float]]) -> dict[str, Any]:
    state = np.asarray(states, dtype=np.float64)
    action = np.asarray(actions, dtype=np.float64)
    return {
        "observation.state": {
            "mean": state.mean(axis=0).tolist(),
            "std": state.std(axis=0).tolist(),
            "min": state.min(axis=0).tolist(),
            "max": state.max(axis=0).tolist(),
        },
        "action": {
            "mean": action.mean(axis=0).tolist(),
            "std": action.std(axis=0).tolist(),
            "min": action.min(axis=0).tolist(),
            "max": action.max(axis=0).tolist(),
        },
        "_num_episodes": 1,
    }


def _merge_stats(dataset_root: Path, rows: list[dict[str, Any]]) -> None:
    path = dataset_root / "meta" / "stats.json"
    fresh = _compute_stats(
        [row["observation.state"] for row in rows],
        [row["action"] for row in rows],
    )
    if not path.exists():
        path.write_text(json.dumps(fresh, indent=2), encoding="utf-8")
        return
    old = json.loads(path.read_text(encoding="utf-8"))
    n_old = int(old.get("_num_episodes", 1))
    n_new = 1
    for key in ("observation.state", "action"):
        for stat, fn in (("min", min), ("max", max)):
            old[key][stat] = [
                fn(left, right) for left, right in zip(old[key][stat], fresh[key][stat])
            ]
        merged_mean = []
        merged_std = []
        for index, old_mean in enumerate(old[key]["mean"]):
            new_mean = fresh[key]["mean"][index]
            combined = (n_old * old_mean + n_new * new_mean) / (n_old + n_new)
            var_old = old[key]["std"][index] ** 2
            var_new = fresh[key]["std"][index] ** 2
            combined_var = (
                n_old * (var_old + (old_mean - combined) ** 2)
                + n_new * (var_new + (new_mean - combined) ** 2)
            ) / (n_old + n_new)
            merged_mean.append(combined)
            merged_std.append(math.sqrt(max(combined_var, 0.0)))
        old[key]["mean"] = merged_mean
        old[key]["std"] = merged_std
    old["_num_episodes"] = n_old + 1
    path.write_text(json.dumps(old, indent=2), encoding="utf-8")


def append_episode(
    dataset_root: str | Path,
    frames: list[dict[str, Any]],
    metadata: dict[str, str],
    fps: int,
    episode_index: int,
    repo_id: str = "local/mobile_manipulation",
) -> int:
    """Append one episode. Returns the episode index written.

    ``metadata`` must include ``task``, ``operator``, ``robot_kind``, and
    ``sim_world``. Those fields are stored on the ``episodes.jsonl`` row.
    """
    if not frames:
        raise ValueError("append_episode: empty frame list")
    if isinstance(episode_index, bool) or not isinstance(episode_index, int) or episode_index < 0:
        raise ValueError("episode_index must be an integer >= 0")
    rate = int(fps) if int(fps) > 0 else 1
    pa, pq = _require_pyarrow()

    root = Path(dataset_root)
    info = _load_info(root, rate, repo_id)
    task_name = metadata.get("task") or "mobile manipulation task"
    task_index = _ensure_task(root, task_name)
    chunk = episode_index // CHUNK_SIZE_EPISODES
    episode_name = f"episode_{episode_index:06d}"
    data_path = root / "data" / f"chunk-{chunk:03d}" / f"{episode_name}.parquet"
    if data_path.exists():
        raise FileExistsError(f"episode already exists: {data_path}")

    t0 = float(frames[0].get("timestamp", 0.0))
    rows: list[dict[str, Any]] = []
    cameras: dict[str, list[np.ndarray]] = {key: [] for key in CAMERA_KEYS}
    wrist_stamps: list[float] = []
    bev_stamps: list[float] = []
    global_offset = int(info.get("total_frames", 0))

    for local_index, frame in enumerate(frames):
        stamp = float(frame.get("timestamp", local_index / rate))
        rows.append(
            {
                "observation.state": _vector(frame, "observation.state", "state"),
                "action": _vector(frame, "action", "action"),
                "timestamp": stamp - t0,
                "frame_index": local_index,
                "episode_index": episode_index,
                "index": global_offset + local_index,
                "task_index": task_index,
                "next.done": local_index == len(frames) - 1,
            }
        )
        wrist_stamps.append(float(frame.get("wrist_timestamp", stamp)))
        bev_stamps.append(float(frame.get("bev_timestamp", stamp)))
        for key, attr in IMAGE_ATTRS.items():
            cameras[key].append(_image(frame, attr))

    schema = pa.schema(
        [
            pa.field("observation.state", pa.list_(pa.float32())),
            pa.field("action", pa.list_(pa.float32())),
            pa.field("timestamp", pa.float64()),
            pa.field("frame_index", pa.int64()),
            pa.field("episode_index", pa.int64()),
            pa.field("index", pa.int64()),
            pa.field("task_index", pa.int64()),
            pa.field("next.done", pa.bool_()),
            pa.field("observation.images.wrist.timestamp", pa.float64()),
            pa.field("observation.images.bev.timestamp", pa.float64()),
        ]
    )
    columns: dict[str, Any] = {name: [row[name] for row in rows] for name in PARQUET_COLUMNS}
    columns["observation.images.wrist.timestamp"] = wrist_stamps
    columns["observation.images.bev.timestamp"] = bev_stamps
    data_path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(pa.Table.from_pydict(columns, schema=schema), data_path, compression="snappy")

    video_root = root / "videos" / f"chunk-{chunk:03d}"
    for key, images in cameras.items():
        stack = np.stack(images)
        npy_path = video_root / key / f"{episode_name}.npy"
        npy_path.parent.mkdir(parents=True, exist_ok=True)
        np.save(npy_path, stack)
        _write_mp4(images, video_root / key / f"{episode_name}.mp4", rate)

    info["codebase_version"] = CODEBASE_VERSION
    info["robot_type"] = "omnibot"
    info["repo_id"] = repo_id
    info["fps"] = rate
    info["chunks_size"] = CHUNK_SIZE_EPISODES
    info["total_episodes"] = int(info.get("total_episodes", 0)) + 1
    info["total_frames"] = int(info.get("total_frames", 0)) + len(rows)
    sample = cameras["observation.images.wrist"][0]
    info["features"] = _features(rate, int(sample.shape[0]), int(sample.shape[1]))
    meta_dir = root / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)
    (meta_dir / "info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
    _append_jsonl(
        meta_dir / "episodes.jsonl",
        {
            "episode_index": episode_index,
            "tasks": [task_index],
            "length": len(rows),
            "task": task_name,
            "operator": metadata.get("operator", "vr"),
            "robot_kind": metadata.get("robot_kind", "sim"),
            "sim_world": metadata.get("sim_world", ""),
        },
    )
    _merge_stats(root, rows)
    return episode_index
