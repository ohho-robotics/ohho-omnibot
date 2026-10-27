"""Pose and IMU helpers (no ROS runtime required)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


@dataclass(frozen=True)
class Pose2D:
    t_ns: int
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ImuSample:
    t_ns: int
    yaw: float
    wz: float


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def integrate_yaw_from_gyro(samples: Sequence[ImuSample]) -> List[Pose2D]:
    """Integrate angular velocity into yaw (planar), holding x/y at zero."""
    if not samples:
        return []
    out: List[Pose2D] = []
    yaw = samples[0].yaw
    prev_t = samples[0].t_ns
    out.append(Pose2D(samples[0].t_ns, 0.0, 0.0, yaw))
    for s in samples[1:]:
        dt = (s.t_ns - prev_t) * 1e-9
        prev_t = s.t_ns
        yaw += s.wz * dt
        out.append(Pose2D(s.t_ns, 0.0, 0.0, yaw))
    return out


def path_length(poses: Sequence[Pose2D]) -> float:
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        total += math.hypot(b.x - a.x, b.y - a.y)
    return total


def final_goal_error(poses: Sequence[Pose2D], goal: Pose2D) -> Tuple[float, float]:
    if not poses:
        return (float("nan"), float("nan"))
    last = poses[-1]
    xy_err = math.hypot(goal.x - last.x, goal.y - last.y)
    yaw_err = abs(math.atan2(math.sin(goal.yaw - last.yaw), math.cos(goal.yaw - last.yaw)))
    return xy_err, yaw_err


def align_yaw_series_by_time(
    odom_yaws: Iterable[Tuple[int, float]],
    imu_yaws: Iterable[Tuple[int, float]],
    max_delta_ns: int = 50_000_000,
) -> List[Tuple[float, float]]:
    """Pair odom yaw with nearest IMU yaw sample (seconds-scale drift metrics)."""
    imu_list = list(imu_yaws)
    pairs: List[Tuple[float, float]] = []
    for t_o, yaw_o in odom_yaws:
        if not imu_list:
            break
        best = min(imu_list, key=lambda p: abs(p[0] - t_o))
        if abs(best[0] - t_o) <= max_delta_ns:
            pairs.append((yaw_o, best[1]))
    return pairs
