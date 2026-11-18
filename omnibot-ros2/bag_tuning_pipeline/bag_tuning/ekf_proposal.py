"""Propose robot_localization process noise tweaks from drift analysis."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List

import yaml

from bag_tuning.drift import DriftReport

# Indices for 15-state EKF diagonal (x,y,z, roll,pitch,yaw, vx,vy,vz, vroll,vpitch,vyaw, ax,ay,az)
# Matches robot_localization.yaml layout used on OmniBot.
_VX_IDX = 6
_VY_IDX = 7
_VYAW_IDX = 11
_YAW_IDX = 5


def _baseline_process_noise() -> List[float]:
    """Diagonal from omnibot_navigation/config/robot_localization.yaml."""
    diag = [0.0] * 15
    diag[0] = 0.05
    diag[1] = 0.05
    diag[2] = 0.06
    diag[3] = 0.03
    diag[4] = 0.03
    diag[5] = 0.06
    diag[6] = 0.025
    diag[7] = 0.025
    diag[8] = 0.04
    diag[9] = 0.01
    diag[10] = 0.01
    diag[11] = 0.5
    diag[12] = 0.5
    diag[13] = 0.5
    diag[14] = 0.5
    return diag


def propose_process_noise(report: DriftReport) -> Dict[str, float]:
    """
    Heuristic scaling from fixture/synthetic drift only — not validated on hardware.
    Higher yaw disagreement → slightly raise vyaw process noise; longer path with
    residual yaw error → raise vx/vy modestly.
    """
    diag = _baseline_process_noise()
    yaw_scale = 1.0
    vel_scale = 1.0
    if report.sample_count > 0 and report.yaw_rms_rad == report.yaw_rms_rad:
        yaw_scale = min(2.5, 1.0 + report.yaw_rms_rad * 5.0)
        vel_scale = min(2.0, 1.0 + report.odom_path_length_m * 0.05)
    proposed = {
        "vx_process_noise": diag[_VX_IDX] * vel_scale,
        "vy_process_noise": diag[_VY_IDX] * vel_scale,
        "vyaw_process_noise": diag[_VYAW_IDX] * yaw_scale,
        "yaw_process_noise": diag[_YAW_IDX] * yaw_scale,
        "yaw_rms_input_rad": report.yaw_rms_rad,
        "odom_path_length_m": report.odom_path_length_m,
    }
    return proposed


def flatten_process_noise(diag: List[float]) -> List[float]:
    """Expand diagonal to 15x15 row-major flat list for robot_localization."""
    out = [0.0] * (15 * 15)
    for i, v in enumerate(diag):
        out[i * 15 + i] = v
    return out


def write_ekf_proposal_yaml(path: Path, proposal: Dict[str, float]) -> None:
    payload: Dict[str, Any] = {
        "ohh_36_ekf_proposal": {
            "status": "UNVALIDATED_ON_HARDWARE",
            "source": "offline bag fixture analysis only",
            "notes": [
                "Copy suggested scalars into robot_localization.yaml after on-robot confirmation.",
                "Do not deploy these values without a stationary + drive test on the real base.",
            ],
            "suggested_diagonal_scalars": proposal,
            "process_noise_covariance_flat": flatten_process_noise(_scaled_diag(proposal)),
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def _scaled_diag(proposal: Dict[str, float]) -> List[float]:
    diag = _baseline_process_noise()
    diag[_VX_IDX] = proposal["vx_process_noise"]
    diag[_VY_IDX] = proposal["vy_process_noise"]
    diag[_VYAW_IDX] = proposal["vyaw_process_noise"]
    diag[_YAW_IDX] = proposal["yaw_process_noise"]
    return diag


def write_ekf_proposal_markdown(path: Path, report: DriftReport, proposal: Dict[str, float]) -> None:
    lines = [
        "# OHH-36 EKF process noise proposal",
        "",
        "**Status: UNVALIDATED ON HARDWARE** — generated from offline bag/fixture analysis only.",
        "",
        "## Drift summary",
        "",
    ]
    lines.extend(f"- {line}" for line in report.summary_lines())
    lines.extend(
        [
            "",
            "## Suggested diagonal adjustments (robot_localization)",
            "",
            f"- `vx` process noise: {proposal['vx_process_noise']:.4f}",
            f"- `vy` process noise: {proposal['vy_process_noise']:.4f}",
            f"- `yaw` process noise: {proposal['yaw_process_noise']:.4f}",
            f"- `vyaw` process noise: {proposal['vyaw_process_noise']:.4f}",
            "",
            "## Human TODO",
            "",
            "- Record a slow straight-line bag and a in-place rotate bag on the real robot.",
            "- Re-run `analyze_bag.py` and compare yaw RMS before merging into `robot_localization.yaml`.",
            "- Confirm `/odometry/filtered` tracks `/odom` without divergence over 60 s idle.",
            "",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")
