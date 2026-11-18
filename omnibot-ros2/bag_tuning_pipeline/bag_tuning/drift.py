"""Odom vs IMU drift metrics from bag trajectories."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence

from bag_tuning.poses import ImuSample, Pose2D, align_yaw_series_by_time


@dataclass
class DriftReport:
    odom_path_length_m: float
    yaw_rms_rad: float
    yaw_max_rad: float
    imu_integrated_yaw_delta_rad: float
    odom_yaw_delta_rad: float
    sample_count: int

    def summary_lines(self) -> List[str]:
        return [
            f"odom path length: {self.odom_path_length_m:.3f} m",
            f"odom yaw delta: {self.odom_yaw_delta_rad:.4f} rad",
            f"imu integrated yaw delta: {self.imu_integrated_yaw_delta_rad:.4f} rad",
            f"yaw pairing RMS (odom vs imu orientation): {self.yaw_rms_rad:.4f} rad",
            f"yaw pairing max |error|: {self.yaw_max_rad:.4f} rad",
            f"paired samples: {self.sample_count}",
        ]


def _path_length(poses: Sequence[Pose2D]) -> float:
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        total += math.hypot(b.x - a.x, b.y - a.y)
    return total


def analyze_odom_imu_drift(odom: Sequence[Pose2D], imu: Sequence[ImuSample]) -> DriftReport:
    if not odom or not imu:
        return DriftReport(0.0, float("nan"), float("nan"), float("nan"), float("nan"), 0)

    pairs = align_yaw_series_by_time(
        [(p.t_ns, p.yaw) for p in odom],
        [(s.t_ns, s.yaw) for s in imu],
    )
    errs = [
        math.atan2(math.sin(y_o - y_i), math.cos(y_o - y_i)) for y_o, y_i in pairs
    ]
    rms = math.sqrt(sum(e * e for e in errs) / len(errs)) if errs else float("nan")
    max_e = max(abs(e) for e in errs) if errs else float("nan")

    imu_delta = imu[-1].yaw - imu[0].yaw
    odom_delta = odom[-1].yaw - odom[0].yaw

    return DriftReport(
        odom_path_length_m=_path_length(odom),
        yaw_rms_rad=rms,
        yaw_max_rad=max_e,
        imu_integrated_yaw_delta_rad=imu_delta,
        odom_yaw_delta_rad=odom_delta,
        sample_count=len(pairs),
    )
