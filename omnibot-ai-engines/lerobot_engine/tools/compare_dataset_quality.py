#!/usr/bin/env python3
"""Compare LeRobot v2 VR teleop episodes with leader-arm episodes.

The on-disk layout is the one the OHH-226 recorder writes on
``origin/ohh-226-recorder-vr-source``: ``meta/info.json``,
``meta/episodes.jsonl``, and ``data/chunk-*/episode_*.parquet``, with optional
``observation.images.wrist.timestamp`` and ``observation.images.bev.timestamp``
columns. This file does not import that branch. Source names ``vr`` and
``leader`` match ``ACTION_SOURCE_VR`` and ``ACTION_SOURCE_LEADER`` in
``omnibot_lerobot.vr_action``. A directory is accepted when it has
``meta/info.json`` or parquet under ``data/``, the same check as
``looks_like_lerobot_v2`` on ``origin/ohh-243-act-vr-loop``. Train and serve
code on that branch is not imported.

Per episode, and again aggregated by source:

- action jerk: mean and max L2 of the third forward difference of ``action``
  divided by ``dt**3`` (action units per second cubed). At least four frames.
- action smoothness: ``1 / (1 + mean ||a[t]-a[t-1]||_2)``. 1 is a constant
  action. This is a per-frame step, so compare sets recorded at the same
  ``record_hz`` (the recorder default is 30).
- timestamp skew: mean and max absolute wrist-minus-BEV timestamp, in seconds
  internally. The markdown table shows milliseconds.
- frame drops: inter-frame gaps longer than ``drop_factor / fps`` (default
  1.5). ``dropped_frames`` is ``floor(gap * fps) - 1`` summed over those gaps.
- time per episode: ``score_episode`` duration (last timestamp minus first).
- quality filter: ``data_engine.quality.episode_filter.score_episode``
  (frozen video, wrist/BEV skew, joint step, short episode).

``--vr`` and ``--leader`` label every episode in that directory. ``--dataset``
reads ``source`` or ``action_source`` on the episode row, then the same keys
on ``info.json``, then ``operator`` only when it is exactly ``vr`` or
``leader``. OHH-226 stores ``operator: vr`` for a VR episode when operator is
left empty, and stores an empty operator for the leader path. It does not
write ``action_source`` into the episode row, so leader versus VR in one
shared directory is not reliable. Pass two roots.

The process does not record, does not open a device, and does not upload.

Example::

    python omnibot-ai-engines/lerobot_engine/tools/compare_dataset_quality.py \\
        --vr ~/datasets/mobile_manipulation_vr \\
        --leader ~/datasets/mobile_manipulation_leader \\
        --json-out docs/evidence/ohh-244/vr-vs-leader-quality.json \\
        --markdown-out docs/evidence/ohh-244/vr-vs-leader-quality.md
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import re
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

_ENGINES = Path(__file__).resolve().parents[2]
if str(_ENGINES) not in sys.path:
    sys.path.insert(0, str(_ENGINES))

from data_engine.quality.episode_filter import (  # noqa: E402
    DiskFrameSource,
    FilterThresholds,
    score_episode,
)
from data_engine.schema.camera_keys import (  # noqa: E402
    OBSERVATION_IMAGE_BEV,
    OBSERVATION_IMAGE_WRIST,
)

SCHEMA_VERSION = 1
TOOL_PATH = "omnibot-ai-engines/lerobot_engine/tools/compare_dataset_quality.py"
SOURCE_VR = "vr"
SOURCE_LEADER = "leader"
SOURCE_UNLABELED = "unlabeled"
SOURCE_COLUMNS = (SOURCE_VR, SOURCE_LEADER)
DEFAULT_FPS = 30.0
DEFAULT_DROP_FACTOR = 1.5
EPISODE_FILE = re.compile(r"episode_(\d+)\.parquet$")

# Same timestamp column names as data_engine.quality.episode_filter.filter_dataset.
WRIST_TS_COLUMNS = (
    "observation.images.wrist.timestamp",
    "observation.images.wrist_timestamp",
    "wrist_timestamp",
    "camera_wrist_timestamp",
    "timestamp_wrist",
    "wrist_ts",
)
BEV_TS_COLUMNS = (
    "observation.images.bev.timestamp",
    "observation.images.bev_timestamp",
    "bev_timestamp",
    "camera_bev_timestamp",
    "timestamp_bev",
    "bev_ts",
)
NUMERIC_COLUMNS = (
    "observation.state",
    "state",
    "action",
    "timestamp",
    "frame_index",
    "episode_index",
    *WRIST_TS_COLUMNS,
    *BEV_TS_COLUMNS,
)
SOURCE_ALIASES = {
    "vr": SOURCE_VR,
    "leader": SOURCE_LEADER,
    "leader-arm": SOURCE_LEADER,
    "leader_arm": SOURCE_LEADER,
}

# id, markdown label, aggregate field
COMPARISON_ROWS = (
    ("episode_count", "Episodes", "episode_count"),
    ("kept_episodes", "Kept by quality filter", "kept_episodes"),
    ("rejected_episodes", "Rejected by quality filter", "rejected_episodes"),
    ("reject_reasons", "Reject reasons", "reject_reasons"),
    ("mean_duration_s", "Mean time per episode (s)", "mean_duration_s"),
    ("median_duration_s", "Median time per episode (s)", "median_duration_s"),
    ("min_duration_s", "Min time per episode (s)", "min_duration_s"),
    ("max_duration_s", "Max time per episode (s)", "max_duration_s"),
    ("mean_jerk", "Mean action jerk", "mean_jerk"),
    ("max_jerk", "Max action jerk", "max_jerk"),
    ("mean_action_l2_step", "Mean action step (L2)", "mean_action_l2_step"),
    (
        "mean_action_smoothness",
        "Action smoothness (0-1, higher is smoother)",
        "mean_action_smoothness",
    ),
    (
        "mean_camera_skew_ms",
        "Mean wrist-BEV timestamp skew (ms)",
        "mean_camera_skew_ms",
    ),
    (
        "max_camera_skew_ms",
        "Max wrist-BEV timestamp skew (ms)",
        "max_camera_skew_ms",
    ),
    ("frame_drop_gaps", "Frame-drop gaps", "frame_drop_gaps"),
    ("dropped_frames", "Dropped frames", "dropped_frames"),
    ("frame_drop_rate", "Frame-drop rate", "frame_drop_rate"),
)


class DatasetLayoutError(Exception):
    """The path exists but is not a LeRobot v2 root this tool can score."""


class TolerantDiskFrames(DiskFrameSource):
    """Disk frames that skip mp4 when OpenCV is not installed.

    ``.npy`` and ``.npz`` are still returned. OHH-226 writes those sidecars
    so the frozen-video rule can run without a video decode.
    """

    def __init__(self, dataset_root: str | Path, chunks_size: int = 1000) -> None:
        super().__init__(dataset_root, chunks_size=chunks_size)
        self.mp4_skipped = 0

    def get_frames(self, ep_idx: int, cam_key: str):
        chunk = self._chunk(ep_idx)
        ep_str = f"episode_{ep_idx:06d}"
        cam_dir = self.dataset_root / "videos" / chunk / cam_key
        npy_path = cam_dir / f"{ep_str}.npy"
        if npy_path.exists():
            array = np.load(npy_path)
            return (array[i] for i in range(len(array)))
        npz_path = cam_dir / f"{ep_str}.npz"
        if npz_path.exists():
            data = np.load(npz_path)
            key = list(data.keys())[0]
            array = data[key]
            return (array[i] for i in range(len(array)))
        mp4_path = cam_dir / f"{ep_str}.mp4"
        if not mp4_path.exists():
            return []
        if importlib.util.find_spec("cv2") is None:
            self.mp4_skipped += 1
            return []
        return self._iter_mp4(mp4_path)


def normalize_source_label(raw: object) -> str | None:
    """Return ``vr`` or ``leader`` when ``raw`` is one of those names."""
    if not isinstance(raw, str):
        return None
    return SOURCE_ALIASES.get(raw.strip().lower())


def dataset_kind(root: Path) -> str:
    """Classify a directory as ``v2``, ``numpy``, or ``missing``.

    ``v2`` matches ``looks_like_lerobot_v2`` (info.json or any parquet under
    ``data/``) and also a root that already has ``meta/episodes.jsonl``.
    ``numpy`` is the leader recorder fallback ``episode_*/data.npz``.
    """
    if (root / "meta" / "info.json").is_file() or (root / "meta" / "episodes.jsonl").is_file():
        return "v2"
    data = root / "data"
    if data.is_dir() and any(data.rglob("*.parquet")):
        return "v2"
    if any(root.glob("episode_*/data.npz")):
        return "numpy"
    return "missing"


def timestamps_to_seconds(values: np.ndarray) -> np.ndarray:
    """Convert nanosecond stamps the way ``check_duration`` does."""
    stamps = np.asarray(values, dtype=np.float64).reshape(-1)
    if stamps.size == 0:
        return stamps
    finite = stamps[np.isfinite(stamps)]
    if finite.size and (float(np.median(finite)) > 1e12 or float(np.max(finite)) > 1e12):
        return stamps * 1e-9
    return stamps


def motion_metrics(
    actions: np.ndarray,
    timestamps: np.ndarray,
    *,
    fps: float,
    drop_factor: float,
    timestamps_synthetic: bool,
) -> dict[str, Any]:
    """Jerk, per-frame smoothness, and frame-drop counts for one episode."""
    signal = np.asarray(actions, dtype=np.float64)
    if signal.ndim == 1:
        signal = signal.reshape(-1, 1)
    stamps = np.asarray(timestamps, dtype=np.float64).reshape(-1)
    empty = {
        "mean_jerk": None,
        "max_jerk": None,
        "mean_action_l2_step": None,
        "mean_action_rate": None,
        "action_smoothness": None,
        "frame_drop_gaps": None,
        "dropped_frames": None,
        "max_interframe_gap_s": None,
    }
    if signal.shape[0] < 2 or stamps.shape[0] != signal.shape[0]:
        return empty

    delta = np.diff(signal, axis=0)
    dt = np.diff(stamps)
    valid = np.isfinite(dt) & (dt > 1e-9)
    steps = np.linalg.norm(delta, axis=1)
    finite_steps = steps[np.isfinite(steps)]
    mean_step = float(np.mean(finite_steps)) if finite_steps.size else None
    smoothness = None if mean_step is None else float(1.0 / (1.0 + mean_step))
    rates = [
        float(steps[index] / dt[index])
        for index in range(steps.shape[0])
        if valid[index] and np.isfinite(steps[index])
    ]
    mean_rate = float(np.mean(rates)) if rates else None
    mean_jerk, max_jerk = _jerk_l2(signal, dt, valid)

    gaps: int | None
    dropped: int | None
    max_gap: float | None
    if timestamps_synthetic:
        gaps, dropped, max_gap = None, None, None
    else:
        gaps, dropped, max_gap = _frame_drops(dt, fps=fps, drop_factor=drop_factor)

    return {
        "mean_jerk": mean_jerk,
        "max_jerk": max_jerk,
        "mean_action_l2_step": mean_step,
        "mean_action_rate": mean_rate,
        "action_smoothness": smoothness,
        "frame_drop_gaps": gaps,
        "dropped_frames": dropped,
        "max_interframe_gap_s": max_gap,
    }


def _jerk_l2(
    signal: np.ndarray, dt: np.ndarray, valid: np.ndarray
) -> tuple[float | None, float | None]:
    """Mean and max L2 of the third forward difference, or None if too short."""
    count = signal.shape[0]
    if count < 4 or not np.any(valid):
        return None, None
    delta = np.diff(signal, axis=0)
    velocity = np.full_like(delta, np.nan)
    velocity[valid] = delta[valid] / dt[valid, None]
    jerk_ok = valid[:-2] & valid[1:-1] & valid[2:]
    if not np.any(jerk_ok):
        return None, None
    acceleration = np.full((count - 2, signal.shape[1]), np.nan)
    acc_ok = valid[:-1] & valid[1:]
    acceleration[acc_ok] = (velocity[1:][acc_ok] - velocity[:-1][acc_ok]) / dt[:-1][acc_ok, None]
    jerk = np.full((count - 3, signal.shape[1]), np.nan)
    jerk[jerk_ok] = (acceleration[1:][jerk_ok] - acceleration[:-1][jerk_ok]) / dt[:-2][
        jerk_ok, None
    ]
    norms = np.linalg.norm(jerk[jerk_ok], axis=1)
    norms = norms[np.isfinite(norms)]
    if norms.size == 0:
        return None, None
    return float(np.mean(norms)), float(np.max(norms))


def _frame_drops(dt: np.ndarray, *, fps: float, drop_factor: float) -> tuple[int, int, float | None]:
    limit = drop_factor / fps
    gaps = 0
    dropped = 0
    measured: list[float] = []
    for gap in dt:
        if not math.isfinite(float(gap)) or float(gap) <= 0.0:
            continue
        measured.append(float(gap))
        if float(gap) > limit:
            gaps += 1
            missing = int(math.floor(float(gap) * fps + 1e-9)) - 1
            dropped += max(missing, 1)
    max_gap = max(measured) if measured else None
    return gaps, dropped, max_gap


def _read_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    loaded = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise DatasetLayoutError(f"{path} is not a JSON object")
    return loaded


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    records = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        record = json.loads(line)
        if not isinstance(record, dict):
            raise DatasetLayoutError(f"{path}:{line_number} is not a JSON object")
        records.append(record)
    return records


def _first_present(columns: Mapping[str, Sequence[Any]], names: Sequence[str]) -> str | None:
    for name in names:
        if name in columns:
            return name
    return None


def _matrix(values: Sequence[Any] | None) -> np.ndarray | None:
    if values is None:
        return None
    rows = list(values)
    if not rows or any(item is None for item in rows):
        return None
    try:
        matrix = np.asarray(rows, dtype=np.float64)
    except (TypeError, ValueError):
        matrix = np.stack([np.asarray(row, dtype=np.float64).reshape(-1) for row in rows])
    if matrix.ndim == 1:
        matrix = matrix.reshape(-1, 1)
    if matrix.ndim != 2 or not np.isfinite(matrix).all():
        finite = np.isfinite(matrix)
        if matrix.ndim != 2 or not finite.any():
            return None
    return matrix


def _vector(values: Sequence[Any] | None) -> np.ndarray | None:
    if values is None:
        return None
    converted = []
    for item in values:
        if item is None:
            converted.append(np.nan)
        else:
            converted.append(float(item))
    return np.asarray(converted, dtype=np.float64)


def _sidecar_timestamps(root: Path, chunks_size: int, episode_index: int, camera_key: str) -> np.ndarray | None:
    chunk = f"chunk-{episode_index // chunks_size:03d}"
    path = root / "videos" / chunk / camera_key / f"episode_{episode_index:06d}_timestamps.npy"
    if not path.is_file():
        return None
    return np.asarray(np.load(path), dtype=np.float64).reshape(-1)


def _episode_index_from_name(path: Path) -> int | None:
    match = EPISODE_FILE.search(path.name)
    if match is None:
        return None
    return int(match.group(1))


def _piece_from_table(path: Path) -> list[dict[str, Any]]:
    schema_names = set(pq_schema_names(path))
    columns = [name for name in NUMERIC_COLUMNS if name in schema_names]
    if not columns:
        return []
    table = _read_parquet(path, columns)
    rows = table.to_pydict()
    count = table.num_rows
    name_index = _episode_index_from_name(path)
    dedicated = name_index is not None
    episode_column = rows.get("episode_index")
    if episode_column is not None:
        indices = [int(item) for item in episode_column]
    elif name_index is not None:
        indices = [name_index] * count
    else:
        raise DatasetLayoutError(
            f"{path} has no episode_index column and is not named episode_NNNNNN.parquet"
        )
    frame_column = rows.get("frame_index")
    order = list(range(count))
    if frame_column is not None and all(item is not None for item in frame_column):
        order = sorted(order, key=lambda index: (int(frame_column[index]), index))
    grouped: dict[int, dict[str, list[Any]]] = {}
    for index in order:
        episode = indices[index]
        bucket = grouped.setdefault(episode, {name: [] for name in columns})
        for name in columns:
            bucket[name].append(rows[name][index])
    pieces = []
    for episode, payload in grouped.items():
        pieces.append(
            {
                "episode_index": episode,
                "dedicated": dedicated,
                "path": str(path),
                "n": len(next(iter(payload.values()))) if payload else 0,
                "columns": payload,
            }
        )
    return pieces


def pq_schema_names(path: Path) -> list[str]:
    import pyarrow.parquet as pq

    return list(pq.read_schema(path).names)


def _read_parquet(path: Path, columns: Sequence[str]):
    import pyarrow.parquet as pq

    return pq.read_table(path, columns=list(columns))


def load_episode_columns(root: Path) -> tuple[dict[int, dict[str, list[Any]]], list[str]]:
    """Read numeric parquet columns keyed by episode index.

    Rows from ``episode_NNNNNN.parquet`` win over rows for the same index in
    a combined chunk file. That is the OHH-226 layout. A LeRobot chunk file
    ``file-*.parquet`` with an ``episode_index`` column is used when no
    per-episode file exists.
    """
    data = root / "data"
    files = sorted(data.rglob("*.parquet")) if data.is_dir() else []
    incoming: dict[int, list[dict[str, Any]]] = {}
    for path in files:
        for piece in _piece_from_table(path):
            incoming.setdefault(int(piece["episode_index"]), []).append(piece)
    warnings: list[str] = []
    columns_by_episode: dict[int, dict[str, list[Any]]] = {}
    for episode, pieces in incoming.items():
        dedicated = [piece for piece in pieces if piece["dedicated"]]
        used = dedicated or pieces
        if dedicated and len(dedicated) != len(pieces):
            ignored = sum(piece["n"] for piece in pieces if not piece["dedicated"])
            warnings.append(
                f"episode {episode}: ignored {ignored} parquet rows because "
                f"episode_{episode:06d}.parquet is present"
            )
        merged: dict[str, list[Any]] = {}
        for piece in used:
            for name, values in piece["columns"].items():
                merged.setdefault(name, []).extend(values)
        columns_by_episode[episode] = merged
    return columns_by_episode, warnings


def _resolve_source(record: Mapping[str, Any], info: Mapping[str, Any], forced: str | None) -> str:
    if forced in SOURCE_COLUMNS:
        return forced
    for key in ("source", "action_source"):
        label = normalize_source_label(record.get(key))
        if label:
            return label
    for key in ("source", "action_source"):
        label = normalize_source_label(info.get(key))
        if label:
            return label
    label = normalize_source_label(record.get("operator"))
    if label:
        return label
    return SOURCE_UNLABELED


def _fps_of(info: Mapping[str, Any], override: float | None) -> tuple[float, str | None]:
    if override is not None:
        if not math.isfinite(override) or override <= 0:
            raise DatasetLayoutError(f"fps override must be positive, got {override}")
        return float(override), None
    raw = info.get("fps", DEFAULT_FPS)
    try:
        fps = float(raw)
    except (TypeError, ValueError):
        return DEFAULT_FPS, f"info.json fps {raw!r} is not a number; used {DEFAULT_FPS}"
    if not math.isfinite(fps) or fps <= 0:
        return DEFAULT_FPS, f"info.json fps {raw!r} is not positive; used {DEFAULT_FPS}"
    return fps, None


def load_dataset(
    root: Path,
    *,
    forced_source: str | None,
    thresholds: FilterThresholds,
    drop_factor: float,
    fps_override: float | None = None,
) -> dict[str, Any]:
    """Score every episode in one LeRobot v2 root."""
    kind = dataset_kind(root)
    if kind == "numpy":
        raise DatasetLayoutError(
            f"{root} is the recorder numpy fallback (episode_*/data.npz), not a "
            "LeRobot v2 root. This comparison reads meta/info.json and "
            "data/chunk-*/episode_*.parquet (or a chunk parquet with episode_index). "
            "Point --leader at a v2 dataset."
        )
    if kind != "v2":
        raise DatasetLayoutError(
            f"{root} is not a LeRobot v2 dataset (no meta/info.json, no "
            "meta/episodes.jsonl, and no parquet under data/)."
        )

    info = _read_json(root / "meta" / "info.json")
    episode_rows = _read_jsonl(root / "meta" / "episodes.jsonl")
    fps, fps_warning = _fps_of(info, fps_override)
    chunks_size = int(info.get("chunks_size", 1000) or 1000)
    columns_by_episode, load_warnings = load_episode_columns(root)
    meta_by_index: dict[int, dict[str, Any]] = {}
    order: list[int] = []
    for record in episode_rows:
        if "episode_index" not in record:
            load_warnings.append("skipped an episodes.jsonl row with no episode_index")
            continue
        index = int(record["episode_index"])
        meta_by_index[index] = record
        if index not in order:
            order.append(index)
    for index in sorted(columns_by_episode):
        if index not in meta_by_index:
            load_warnings.append(
                f"episode {index} is in parquet and not in meta/episodes.jsonl"
            )
            order.append(index)

    frames = TolerantDiskFrames(root, chunks_size=chunks_size)
    episodes: list[dict[str, Any]] = []
    for index in order:
        record = meta_by_index.get(index, {})
        episodes.append(
            _score_one(
                root=root,
                episode_index=index,
                record=record,
                info=info,
                columns=columns_by_episode.get(index),
                forced_source=forced_source,
                thresholds=thresholds,
                drop_factor=drop_factor,
                fps=fps,
                chunks_size=chunks_size,
                frames=frames,
            )
        )
    warnings = list(load_warnings)
    if fps_warning:
        warnings.append(fps_warning)
    if frames.mp4_skipped:
        warnings.append(
            f"frozen_video unchecked for {frames.mp4_skipped} camera stream(s): "
            "mp4 is present and OpenCV is not installed. .npy sidecars are scored "
            "when they exist."
        )
    return {
        "path": str(root),
        "fps": fps,
        "chunks_size": chunks_size,
        "codebase_version": info.get("codebase_version"),
        "forced_source": forced_source,
        "warnings": warnings,
        "episodes": episodes,
    }


def _score_one(
    *,
    root: Path,
    episode_index: int,
    record: Mapping[str, Any],
    info: Mapping[str, Any],
    columns: Mapping[str, Sequence[Any]] | None,
    forced_source: str | None,
    thresholds: FilterThresholds,
    drop_factor: float,
    fps: float,
    chunks_size: int,
    frames: TolerantDiskFrames,
) -> dict[str, Any]:
    source = _resolve_source(record, info, forced_source)
    row_count = len(next(iter(columns.values()))) if columns else 0
    if row_count == 0:
        columns = None
    base = {
        "source": source,
        "dataset_path": str(root),
        "episode_index": episode_index,
        "task": record.get("task"),
        "operator": record.get("operator", ""),
        "robot_kind": record.get("robot_kind"),
        "sim_world": record.get("sim_world", ""),
        "declared_length": record.get("length"),
    }
    if not columns:
        base.update(
            {
                "frame_count": 0,
                "duration_s": None,
                "timestamps_synthetic": True,
                "motion_signal": None,
                "mean_jerk": None,
                "max_jerk": None,
                "mean_action_l2_step": None,
                "mean_action_rate": None,
                "action_smoothness": None,
                "max_camera_skew_s": None,
                "mean_camera_skew_s": None,
                "camera_skew_status": "unchecked",
                "frame_drop_gaps": None,
                "dropped_frames": None,
                "max_interframe_gap_s": None,
                "quality_status": "reject",
                "quality_passed": False,
                "quality_reasons": [
                    f"missing_parquet: no parquet rows for episode {episode_index}"
                ],
                "quality_warnings": [],
                "warnings": [],
            }
        )
        return base

    count = len(next(iter(columns.values())))
    state_name = _first_present(columns, ("observation.state", "state"))
    states = _matrix(columns.get(state_name)) if state_name else None
    if states is None or states.shape[0] != count:
        states = np.zeros((count, 6), dtype=np.float64)
    actions = _matrix(columns.get("action")) if "action" in columns else None
    if actions is None or actions.shape[0] != count:
        motion = states
        motion_signal = "observation.state"
    else:
        motion = actions
        motion_signal = "action"

    raw_time = _vector(columns.get("timestamp")) if "timestamp" in columns else None
    synthetic = raw_time is None or raw_time.shape[0] != count
    if synthetic:
        stamps = np.arange(count, dtype=np.float64) / fps
    else:
        stamps = timestamps_to_seconds(raw_time)

    wrist = _camera_times(
        columns, WRIST_TS_COLUMNS, root, chunks_size, episode_index, OBSERVATION_IMAGE_WRIST
    )
    bev = _camera_times(
        columns, BEV_TS_COLUMNS, root, chunks_size, episode_index, OBSERVATION_IMAGE_BEV
    )
    score = score_episode(
        episode_index=episode_index,
        states=np.asarray(states, dtype=np.float32),
        timestamps=stamps,
        wrist_timestamps=wrist,
        bev_timestamps=bev,
        frame_source=frames,
        thresholds=thresholds,
        fps=fps,
    )
    motion_stats = motion_metrics(
        motion,
        stamps,
        fps=fps,
        drop_factor=drop_factor,
        timestamps_synthetic=synthetic,
    )
    mean_skew = _mean_abs_skew(wrist, bev)
    warnings: list[str] = []
    declared = record.get("length")
    if isinstance(declared, int) and not isinstance(declared, bool) and declared != count:
        warnings.append(
            f"episodes.jsonl length {declared} does not match parquet rows {count}"
        )
    max_skew = score.metrics.max_camera_skew_s
    base.update(
        {
            "frame_count": int(score.metrics.frame_count),
            "duration_s": float(score.metrics.duration_s),
            "timestamps_synthetic": synthetic,
            "motion_signal": motion_signal,
            "max_camera_skew_s": None if max_skew is None else float(max_skew),
            "mean_camera_skew_s": mean_skew,
            "camera_skew_status": "unchecked" if max_skew is None else "measured",
            "quality_status": score.status,
            "quality_passed": bool(score.passed),
            "quality_reasons": list(score.reasons),
            "quality_warnings": list(score.warnings),
            "warnings": warnings,
        }
    )
    base.update(motion_stats)
    return base


def _camera_times(
    columns: Mapping[str, Sequence[Any]],
    names: Sequence[str],
    root: Path,
    chunks_size: int,
    episode_index: int,
    camera_key: str,
) -> np.ndarray | None:
    name = _first_present(columns, names)
    if name is not None:
        vector = _vector(columns.get(name))
        if vector is not None:
            return timestamps_to_seconds(vector)
    sidecar = _sidecar_timestamps(root, chunks_size, episode_index, camera_key)
    if sidecar is None:
        return None
    return timestamps_to_seconds(sidecar)


def _mean_abs_skew(wrist: np.ndarray | None, bev: np.ndarray | None) -> float | None:
    if wrist is None or bev is None or wrist.size == 0 or bev.size == 0:
        return None
    width = min(wrist.shape[0], bev.shape[0])
    delta = np.abs(wrist[:width] - bev[:width])
    finite = delta[np.isfinite(delta)]
    if finite.size == 0:
        return None
    return float(np.mean(finite))


def reason_codes(reasons: Sequence[str]) -> list[str]:
    """Stable unique reason family, the text before the first colon."""
    codes: list[str] = []
    for reason in reasons:
        code = reason.split(":", 1)[0].strip() or "unknown"
        if code not in codes:
            codes.append(code)
    return codes


def format_reason_counts(counts: Mapping[str, int]) -> str:
    if not counts:
        return "none"
    return "; ".join(f"{key}: {counts[key]}" for key in sorted(counts))


def aggregate_episodes(episodes: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Aggregate one source. Each episode has equal weight."""
    rows = list(episodes)
    durations = _numbers(rows, "duration_s")
    jerk = _numbers(rows, "mean_jerk")
    max_jerk = _numbers(rows, "max_jerk")
    steps = _numbers(rows, "mean_action_l2_step")
    smooth = _numbers(rows, "action_smoothness")
    mean_skew = _numbers(rows, "mean_camera_skew_s")
    max_skew = _numbers(rows, "max_camera_skew_s")
    kept = sum(1 for row in rows if row.get("quality_passed") is True)
    rejected = sum(1 for row in rows if row.get("quality_passed") is False)
    counts: dict[str, int] = {}
    examples: dict[str, str] = {}
    for row in rows:
        if row.get("quality_passed") is True:
            continue
        seen = reason_codes(list(row.get("quality_reasons") or []))
        full = list(row.get("quality_reasons") or [])
        for code in seen:
            counts[code] = counts.get(code, 0) + 1
            if code not in examples:
                examples[code] = next(
                    (item for item in full if item.split(":", 1)[0].strip() == code),
                    code,
                )
    measured = [row for row in rows if row.get("timestamps_synthetic") is False]
    total_frames = int(sum(int(row.get("frame_count") or 0) for row in rows))
    if measured and all(row.get("dropped_frames") is not None for row in measured):
        drop_gaps = int(sum(int(row["frame_drop_gaps"]) for row in measured))
        dropped = int(sum(int(row["dropped_frames"]) for row in measured))
        measured_frames = int(sum(int(row.get("frame_count") or 0) for row in measured))
        rate = dropped / (measured_frames + dropped) if (measured_frames + dropped) else 0.0
    else:
        drop_gaps = dropped = None
        rate = None
    return {
        "episode_count": len(rows),
        "kept_episodes": kept,
        "rejected_episodes": rejected,
        "reject_reason_counts": counts,
        "reject_reason_examples": examples,
        "reject_reasons": format_reason_counts(counts),
        "total_frames": total_frames,
        "mean_duration_s": _mean(durations),
        "median_duration_s": _median(durations),
        "min_duration_s": _min(durations),
        "max_duration_s": _max(durations),
        "mean_jerk": _mean(jerk),
        "max_jerk": _max(max_jerk),
        "mean_action_l2_step": _mean(steps),
        "mean_action_smoothness": _mean(smooth),
        "mean_camera_skew_ms": None if _mean(mean_skew) is None else _mean(mean_skew) * 1000.0,
        "max_camera_skew_ms": None if _max(max_skew) is None else _max(max_skew) * 1000.0,
        "frame_drop_gaps": drop_gaps,
        "dropped_frames": dropped,
        "frame_drop_rate": rate,
        "timestamps_measured_episodes": len(measured),
    }


def _numbers(rows: Sequence[Mapping[str, Any]], key: str) -> list[float]:
    values = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value)):
            values.append(float(value))
    return values


def _mean(values: Sequence[float]) -> float | None:
    if not values:
        return None
    return float(np.mean(np.asarray(values, dtype=np.float64)))


def _median(values: Sequence[float]) -> float | None:
    if not values:
        return None
    return float(np.median(np.asarray(values, dtype=np.float64)))


def _min(values: Sequence[float]) -> float | None:
    if not values:
        return None
    return float(np.min(np.asarray(values, dtype=np.float64)))


def _max(values: Sequence[float]) -> float | None:
    if not values:
        return None
    return float(np.max(np.asarray(values, dtype=np.float64)))


def build_report(
    groups: Sequence[tuple[str, Path]],
    *,
    thresholds: FilterThresholds | None = None,
    drop_factor: float = DEFAULT_DROP_FACTOR,
    fps_override: float | None = None,
) -> dict[str, Any]:
    """Score ``(argument, dataset_root)`` pairs.

    ``argument`` is ``vr``, ``leader``, or ``dataset``. ``vr`` and ``leader``
    force that source. ``dataset`` uses episode metadata.
    """
    if drop_factor < 1.0 or not math.isfinite(drop_factor):
        raise ValueError("drop_factor must be a finite number >= 1")
    thresh = thresholds or FilterThresholds()
    loaded = []
    for argument, path in groups:
        forced = argument if argument in SOURCE_COLUMNS else None
        loaded.append(
            load_dataset(
                Path(path),
                forced_source=forced,
                thresholds=thresh,
                drop_factor=drop_factor,
                fps_override=fps_override,
            )
        )
    episodes: list[dict[str, Any]] = []
    for dataset in loaded:
        episodes.extend(dataset["episodes"])
    requested = {SOURCE_VR: False, SOURCE_LEADER: False}
    for argument, _path in groups:
        if argument in requested:
            requested[argument] = True
    by_source: dict[str, list[dict[str, Any]]] = {SOURCE_VR: [], SOURCE_LEADER: [], SOURCE_UNLABELED: []}
    for episode in episodes:
        source = str(episode.get("source") or SOURCE_UNLABELED)
        by_source.setdefault(source, []).append(episode)
    sources: dict[str, Any] = {}
    for name in (SOURCE_VR, SOURCE_LEADER, SOURCE_UNLABELED):
        rows = by_source.get(name, [])
        if not rows and not requested.get(name, False):
            continue
        sources[name] = aggregate_episodes(rows)
    comparison = []
    for row_id, label, field in COMPARISON_ROWS:
        vr_value = _column(sources, SOURCE_VR, field)
        leader_value = _column(sources, SOURCE_LEADER, field)
        comparison.append(
            {
                "id": row_id,
                "label": label,
                "vr": vr_value,
                "leader": leader_value,
                "delta_vr_minus_leader": _delta(vr_value, leader_value),
            }
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "tool": TOOL_PATH,
        "quality_filter": "data_engine.quality.episode_filter.score_episode",
        "drop_factor": drop_factor,
        "thresholds": _threshold_dict(thresh),
        "datasets": [
            {
                "path": dataset["path"],
                "fps": dataset["fps"],
                "codebase_version": dataset["codebase_version"],
                "forced_source": dataset["forced_source"],
                "episode_count": len(dataset["episodes"]),
                "warnings": dataset["warnings"],
            }
            for dataset in loaded
        ],
        "episodes": episodes,
        "sources": sources,
        "comparison": comparison,
    }


def _column(sources: Mapping[str, Mapping[str, Any]], source: str, field: str) -> Any:
    if source not in sources:
        return None
    return sources[source].get(field)


def _delta(left: Any, right: Any) -> Any:
    if isinstance(left, str) or isinstance(right, str):
        return None
    if left is None or right is None:
        return None
    if isinstance(left, bool) or isinstance(right, bool):
        return None
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        difference = left - right
        if isinstance(left, int) and isinstance(right, int):
            return int(difference)
        return float(difference)
    return None


def _threshold_dict(thresholds: FilterThresholds) -> dict[str, Any]:
    return {
        "max_frozen_s": thresholds.max_frozen_s,
        "max_camera_skew_s": thresholds.max_camera_skew_s,
        "max_joint_step_rad": thresholds.max_joint_step_rad,
        "min_duration_s": thresholds.min_duration_s,
        "frame_diff_threshold": thresholds.frame_diff_threshold,
        "missing_camera_timestamps": thresholds.missing_camera_timestamps,
        "continuous_joints": sorted(thresholds.continuous_joints),
    }


def round_tree(value: Any) -> Any:
    """JSON-safe copy. Floats are rounded to 6 decimal places."""
    if isinstance(value, dict):
        return {str(key): round_tree(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [round_tree(item) for item in value]
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)) and not isinstance(value, bool):
        return int(value)
    if isinstance(value, (float, np.floating)):
        number = float(value)
        if not math.isfinite(number):
            return None
        return round(number, 6)
    return value


def render_markdown(report: Mapping[str, Any]) -> str:
    """Markdown comparison table plus one row per episode."""
    payload = round_tree(report)
    lines = [
        "# VR vs leader-arm dataset quality",
        "",
        "Generated by `omnibot-ai-engines/lerobot_engine/tools/compare_dataset_quality.py`.",
        "Jerk and smoothness use the `action` column when it is present.",
        "Keep/reject uses the OHH-99 quality filter "
        "(`data_engine.quality.episode_filter.score_episode`).",
        "Delta is VR minus leader. `n/a` means that source was not in this run.",
        "",
        "## Datasets",
        "",
    ]
    datasets = payload.get("datasets") or []
    if not datasets:
        lines.append("No datasets.")
    for dataset in datasets:
        label = dataset.get("forced_source") or "from episode metadata"
        lines.append(
            f"- `{dataset.get('path')}` ({label}), "
            f"{dataset.get('episode_count')} episodes, fps {dataset.get('fps')}"
        )
        for warning in dataset.get("warnings") or []:
            lines.append(f"  - warning: {warning}")
    lines.extend(["", "## Comparison", ""])
    lines.append("| Metric | VR | Leader-arm | VR - leader |")
    lines.append("| --- | --- | --- | --- |")
    for row in payload.get("comparison") or []:
        lines.append(
            "| {label} | {vr} | {leader} | {delta} |".format(
                label=row.get("label", ""),
                vr=_cell(row.get("vr")),
                leader=_cell(row.get("leader")),
                delta=_cell(row.get("delta_vr_minus_leader")),
            )
        )
    unlabeled = (payload.get("sources") or {}).get(SOURCE_UNLABELED)
    if unlabeled:
        lines.extend(
            [
                "",
                f"Unlabeled episodes: {unlabeled.get('episode_count', 0)}. "
                "They are omitted from the VR and leader columns. "
                "OHH-226 does not store `action_source` on the episode row; "
                "pass the directory with `--vr` or `--leader`.",
            ]
        )
    lines.extend(["", "## Episodes", ""])
    lines.append(
        "| source | episode | frames | time_s | mean_jerk | smoothness | "
        "max_skew_ms | frame_drop_gaps | dropped_frames | quality | reasons |"
    )
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for episode in payload.get("episodes") or []:
        skew_s = episode.get("max_camera_skew_s")
        skew_ms = None if skew_s is None else skew_s * 1000.0
        reasons = "; ".join(episode.get("quality_reasons") or [])
        lines.append(
            "| {source} | {index} | {frames} | {time_s} | {jerk} | {smooth} | "
            "{skew} | {gaps} | {dropped} | {quality} | {reasons} |".format(
                source=_cell(episode.get("source")),
                index=_cell(episode.get("episode_index")),
                frames=_cell(episode.get("frame_count")),
                time_s=_cell(episode.get("duration_s")),
                jerk=_cell(episode.get("mean_jerk")),
                smooth=_cell(episode.get("action_smoothness")),
                skew=_cell(skew_ms),
                gaps=_cell(episode.get("frame_drop_gaps")),
                dropped=_cell(episode.get("dropped_frames")),
                quality=_cell(episode.get("quality_status")),
                reasons=_cell(reasons or "none"),
            )
        )
    lines.append("")
    return "\n".join(lines)


def _cell(value: Any) -> str:
    if value is None or value == "":
        return "n/a"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int) and not isinstance(value, bool):
        return str(value)
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value).replace("|", "/").replace("\n", " ")


def write_outputs(
    report: Mapping[str, Any],
    *,
    json_out: Path | None,
    markdown_out: Path | None,
) -> tuple[str, str]:
    """Return ``(markdown, json_text)`` and write the paths that were passed."""
    payload = round_tree(report)
    markdown = render_markdown(payload)
    json_text = json.dumps(payload, indent=2) + "\n"
    if markdown_out is not None:
        markdown_out.parent.mkdir(parents=True, exist_ok=True)
        markdown_out.write_text(markdown, encoding="utf-8")
    if json_out is not None:
        json_out.parent.mkdir(parents=True, exist_ok=True)
        json_out.write_text(json_text, encoding="utf-8")
    return markdown, json_text


def _thresholds_from_args(args: argparse.Namespace) -> FilterThresholds:
    joints = {name.strip() for name in args.continuous_joints.split(",") if name.strip()}
    return FilterThresholds(
        max_frozen_s=args.max_frozen_s,
        max_camera_skew_s=args.max_camera_skew_s,
        max_joint_step_rad=args.max_joint_step_rad,
        min_duration_s=args.min_duration_s,
        frame_diff_threshold=args.frame_diff_threshold,
        missing_camera_timestamps=args.missing_camera_timestamps,
        continuous_joints=joints or {"wrist_roll"},
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compare LeRobot v2 VR and leader-arm datasets. "
            "Writes a markdown table and JSON. Does not record or upload."
        )
    )
    parser.add_argument(
        "--vr",
        type=Path,
        action="append",
        default=[],
        help="LeRobot v2 root recorded with action_source vr. Repeatable.",
    )
    parser.add_argument(
        "--leader",
        type=Path,
        action="append",
        default=[],
        help="LeRobot v2 root recorded with the leader arm. Repeatable.",
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        action="append",
        default=[],
        help=(
            "LeRobot v2 root whose episode rows carry source or action_source. "
            "operator is used only when it is exactly vr or leader."
        ),
    )
    parser.add_argument("--json-out", type=Path, default=None, help="Write the JSON report here.")
    parser.add_argument(
        "--markdown-out",
        type=Path,
        default=None,
        help="Write the markdown comparison here.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=None,
        help="Override meta/info.json fps for synthetic timestamps and frame drops.",
    )
    parser.add_argument(
        "--drop-factor",
        type=float,
        default=DEFAULT_DROP_FACTOR,
        help="A gap longer than this many frame periods is a frame drop (default 1.5).",
    )
    parser.add_argument("--min-duration-s", type=float, default=2.0)
    parser.add_argument("--max-frozen-s", type=float, default=0.5)
    parser.add_argument("--max-camera-skew-s", type=float, default=0.05)
    parser.add_argument("--max-joint-step-rad", type=float, default=0.5)
    parser.add_argument("--frame-diff-threshold", type=float, default=2.0)
    parser.add_argument(
        "--missing-camera-timestamps",
        choices=("unchecked", "reject"),
        default="unchecked",
    )
    parser.add_argument("--continuous-joints", default="wrist_roll")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    groups: list[tuple[str, Path]] = []
    for label, paths in (
        (SOURCE_VR, args.vr),
        (SOURCE_LEADER, args.leader),
        ("dataset", args.dataset),
    ):
        for path in paths:
            groups.append((label, Path(path)))
    if not groups:
        parser.error("pass --vr, --leader, and/or --dataset")
    if args.drop_factor < 1.0 or not math.isfinite(args.drop_factor):
        parser.error("--drop-factor must be a finite number >= 1")
    resolved: list[Path] = []
    for _label, path in groups:
        if not path.exists():
            print(f"dataset path does not exist: {path}", file=sys.stderr)
            return 2
        if not path.is_dir():
            print(f"dataset path is not a directory: {path}", file=sys.stderr)
            return 2
        resolved.append(path.resolve())
    if len(resolved) != len(set(resolved)):
        print("the same dataset path was passed more than once", file=sys.stderr)
        return 2
    try:
        report = build_report(
            [(label, path.resolve()) for label, path in groups],
            thresholds=_thresholds_from_args(args),
            drop_factor=args.drop_factor,
            fps_override=args.fps,
        )
    except (DatasetLayoutError, ValueError, json.JSONDecodeError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if not report["episodes"]:
        print("no episodes to compare", file=sys.stderr)
        return 2
    markdown, json_text = write_outputs(
        report, json_out=args.json_out, markdown_out=args.markdown_out
    )
    _print_summary(report, args.json_out, args.markdown_out)
    if args.markdown_out is None and args.json_out is None:
        sys.stdout.write(markdown)
        sys.stdout.write("\n")
        sys.stdout.write(json_text)
    elif args.markdown_out is None:
        sys.stdout.write(markdown)
    elif args.json_out is None:
        sys.stdout.write(json_text)
    return 0


def _print_summary(report: Mapping[str, Any], json_out: Path | None, markdown_out: Path | None) -> None:
    sources = report.get("sources") or {}
    parts = []
    for name in (SOURCE_VR, SOURCE_LEADER, SOURCE_UNLABELED):
        block = sources.get(name)
        if not block:
            continue
        parts.append(
            f"{name} episodes={block['episode_count']} "
            f"kept={block['kept_episodes']} rejected={block['rejected_episodes']}"
        )
    print("; ".join(parts) if parts else "no labeled episodes", file=sys.stderr)
    if json_out is not None:
        print(f"wrote {json_out}", file=sys.stderr)
    if markdown_out is not None:
        print(f"wrote {markdown_out}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
