"""Camera / state sync jitter metrics from LeRobot parquet episodes."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import pyarrow.parquet as pq


@dataclass(frozen=True)
class SyncJitterReport:
    episode_index: int
    frame_count: int
    wrist_bev_skew_s: dict[str, float]
    state_wrist_skew_s: dict[str, float] | None

    def as_dict(self) -> dict:
        return {
            "episode_index": self.episode_index,
            "frame_count": self.frame_count,
            "wrist_bev_skew_s": self.wrist_bev_skew_s,
            "state_wrist_skew_s": self.state_wrist_skew_s,
        }


def _percentile(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    xs = sorted(values)
    k = (len(xs) - 1) * p
    f = int(k)
    c = min(f + 1, len(xs) - 1)
    if f == c:
        return xs[f]
    return xs[f] + (xs[c] - xs[f]) * (k - f)


def _skew_stats(deltas: Iterable[float]) -> dict[str, float]:
    vals = [abs(float(x)) for x in deltas]
    if not vals:
        return {"max": 0.0, "mean": 0.0, "p95": 0.0}
    return {
        "max": max(vals),
        "mean": sum(vals) / len(vals),
        "p95": _percentile(vals, 0.95),
    }


def measure_episode_skew(parquet_path: Path, episode_index: int) -> SyncJitterReport:
    table = pq.read_table(parquet_path)
    names = set(table.schema.names)
    n = len(table)

    wrist_bev: dict[str, float] = {"max": 0.0, "mean": 0.0, "p95": 0.0}
    if (
        "observation.images.wrist.timestamp" in names
        and "observation.images.bev.timestamp" in names
    ):
        tw = table.column("observation.images.wrist.timestamp").to_pylist()
        tb = table.column("observation.images.bev.timestamp").to_pylist()
        wrist_bev = _skew_stats(t - b for t, b in zip(tw, tb))

    state_wrist = None
    if "observation.images.wrist.timestamp" in names and "timestamp" in names:
        ts = table.column("timestamp").to_pylist()
        tw = table.column("observation.images.wrist.timestamp").to_pylist()
        state_wrist = _skew_stats(w - t for w, t in zip(tw, ts))

    return SyncJitterReport(
        episode_index=episode_index,
        frame_count=n,
        wrist_bev_skew_s=wrist_bev,
        state_wrist_skew_s=state_wrist,
    )


def measure_dataset_skew(dataset_root: str | Path) -> list[SyncJitterReport]:
    root = Path(dataset_root)
    reports: list[SyncJitterReport] = []
    for pq_path in sorted(root.glob("data/chunk-*/episode_*.parquet")):
        ep_idx = int(pq_path.stem.split("_")[-1])
        reports.append(measure_episode_skew(pq_path, ep_idx))
    return reports
