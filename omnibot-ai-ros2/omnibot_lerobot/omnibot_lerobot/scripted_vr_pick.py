# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Scripted virtual-headset pick, and the OHH-99 reject report.

This module does not publish ROS topics and does not import rclpy. The
headset node publishes ``HEADSET_TOPICS`` only. ``teleop_safety`` is what
republishes the gated arm and base commands. Episodes are labelled
``scripted-sim``. They are not human VR demonstrations.

The deadman is ``/ohho/heartbeat`` (std_msgs/Empty at 10 Hz). The VR profile
keeps teleop disarmed until that heartbeat is fresh.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import sys
from typing import Iterator

from omnibot_lerobot.vr_action import JOINT_NAMES, episode_metadata


OPERATOR = "scripted-sim"
ROBOT_KIND = "sim"
SIM_WORLD = "apartment"
TASK_NAME = "scripted-sim pick cube into bin"
REPO_ID = "local/omnibot-sim-vr-pick"
RATE_HZ = 10.0
# Under arm_safety / teleop_gate MAX_JOINT_DELTA_RAD (0.15).
MAX_JOINT_STEP_RAD = 0.08
# Below one scripted joint step. An episode that never leaves the pose
# measured at the start is discarded.
MIN_JOINT_MOTION_RAD = 0.02

RAW_ARM_TOPIC = "/arm/joint_commands/teleop_raw"
RAW_BASE_TOPIC = "/cmd_vel/teleop_raw"
HEARTBEAT_TOPIC = "/ohho/heartbeat"
CONTROL_MODE_TOPIC = "/control_mode"
GATED_ARM_TOPIC = "/arm/joint_commands/teleop"
GATED_BASE_TOPIC = "/cmd_vel/teleop"

HEADSET_TOPICS = (
    RAW_ARM_TOPIC,
    RAW_BASE_TOPIC,
    HEARTBEAT_TOPIC,
    CONTROL_MODE_TOPIC,
)

# Plant and post-gate topics. The headset must not publish these.
PLANT_TOPICS = (
    "/cmd_vel",
    "/cmd_vel/out",
    "/cmd_vel/teleop",
    "/arm/joint_commands",
    "/arm/joint_commands/out",
    "/arm/joint_commands/teleop",
    "/emergency_stop",
)


@dataclass(frozen=True)
class Phase:
    """One joint target held for ``ticks`` cycles, with a base twist."""

    name: str
    ticks: int
    joints: tuple[float, float, float, float, float, float]
    twist: tuple[float, float, float]


# Joint order matches JOINT_NAMES. Targets stay inside arm_params.yaml.
PHASES = (
    Phase("approach", 15, (0.35, 0.55, 0.70, -0.40, 0.00, 0.55), (0.05, 0.0, 0.0)),
    Phase("reach", 20, (0.45, 0.95, 1.05, -0.70, 0.10, 0.55), (0.0, 0.0, 0.0)),
    Phase("grasp", 12, (0.45, 0.95, 1.05, -0.70, 0.10, 0.05), (0.0, 0.0, 0.0)),
    Phase("lift", 15, (0.45, 0.40, 0.45, -0.25, 0.10, 0.05), (0.0, 0.0, 0.0)),
    Phase("carry", 20, (-0.35, 0.40, 0.45, -0.25, -0.10, 0.05), (0.0, 0.0, 0.0)),
    Phase("place", 18, (-0.45, 0.90, 1.00, -0.65, -0.10, 0.05), (0.0, 0.0, 0.0)),
    Phase("release", 12, (-0.45, 0.90, 1.00, -0.65, -0.10, 0.55), (0.0, 0.0, 0.0)),
    Phase("retreat", 15, (0.00, 0.20, 0.15, 0.00, 0.00, 0.55), (0.0, 0.0, 0.0)),
)

EPISODE_TICKS = sum(phase.ticks for phase in PHASES)
EPISODE_SECONDS = EPISODE_TICKS / RATE_HZ


@dataclass(frozen=True)
class HeadsetSample:
    """One 10 Hz client command, before teleop_safety."""

    t: float
    joints: tuple[float, ...]
    twist: tuple[float, float, float]
    phase: str


def isolation_failure(environ=None) -> str | None:
    """Refuse unless this process cannot see the robot DDS domain.

    ``ROS_DOMAIN_ID`` must be the canonical decimal spelling. ``"036"`` is
    decimal 36 here and octal domain 30 in ROS, so it is refused.
    """
    env = os.environ if environ is None else environ
    domain = env.get("ROS_DOMAIN_ID", "")
    localhost = env.get("ROS_LOCALHOST_ONLY", "")
    try:
        domain_id = int(domain, 10)
    except (TypeError, ValueError):
        domain_id = -1
    if domain != str(domain_id) or domain_id == 30 or localhost != "1":
        return (
            "Refusing to publish teleop topics. Set ROS_LOCALHOST_ONLY=1 and "
            "ROS_DOMAIN_ID to a value other than 30 before running this demo."
        )
    # FastDDS multicast port is domain_id * 250 + 7400. Above 232 it overflows.
    if domain_id < 0 or domain_id > 232:
        return (
            "Refusing to publish teleop topics. ROS_DOMAIN_ID must be an integer "
            "from 0 to 232, other than 30. FastDDS cannot compute a multicast "
            "port above 232."
        )
    return None


def scripted_episode_metadata() -> dict[str, str]:
    """Labels stored by the recorder. ``operator`` is scripted-sim."""
    return episode_metadata(TASK_NAME, OPERATOR, ROBOT_KIND, SIM_WORLD)


def collection_record(seed, episodes: int) -> dict:
    """Sidecar that marks the dataset as a scripted sim, not a Quest take."""
    meta = scripted_episode_metadata()
    return {
        "collector": OPERATOR,
        "human_vr": False,
        "action_source": "vr",
        "operator": meta["operator"],
        "robot_kind": meta["robot_kind"],
        "sim_world": meta["sim_world"],
        "task": meta["task"],
        "pick_place_seed": int(seed),
        "episodes_requested": int(episodes),
        "headset_topics": list(HEADSET_TOPICS),
        "gated_topics": [GATED_ARM_TOPIC, GATED_BASE_TOPIC],
        "heartbeat_hz": RATE_HZ,
        "note": (
            "Scripted virtual headset in simulation. Not a human Quest "
            "demonstration. Commands are published on the VR client topics "
            "and must pass through teleop_safety and the muxes."
        ),
    }


def write_collection_record(dataset_root, seed, episodes: int) -> Path:
    path = Path(dataset_root) / "meta" / "collection.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(collection_record(seed, episodes), indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def joint_motion_rad(start, current) -> float:
    """Largest absolute joint delta from ``start`` to ``current``."""
    if not start or not current or len(start) != len(current):
        return 0.0
    return max(abs(float(left) - float(right)) for left, right in zip(start, current))


def step_toward(current, target, max_step: float = MAX_JOINT_STEP_RAD) -> tuple[float, ...]:
    """Move each joint at most ``max_step`` radians toward ``target``."""
    stepped = []
    limit = abs(float(max_step))
    for cur, tgt in zip(current, target):
        delta = float(tgt) - float(cur)
        if delta > limit:
            delta = limit
        elif delta < -limit:
            delta = -limit
        stepped.append(float(cur) + delta)
    return tuple(stepped)


def iter_open_loop(initial_joints, rate_hz: float = RATE_HZ) -> Iterator[HeadsetSample]:
    """Joint-space pick starting from a measured pose. One sample per tick."""
    if len(initial_joints) != len(JOINT_NAMES):
        raise ValueError(f"expected {len(JOINT_NAMES)} joints, got {len(initial_joints)}")
    joints = tuple(float(value) for value in initial_joints)
    dt = 1.0 / float(rate_hz)
    t = 0.0
    for phase in PHASES:
        for _ in range(phase.ticks):
            t += dt
            joints = step_toward(joints, phase.joints, MAX_JOINT_STEP_RAD)
            yield HeadsetSample(t=t, joints=joints, twist=phase.twist, phase=phase.name)


class OpenLoopScript:
    """Cursor over ``iter_open_loop`` for the headset node."""

    def __init__(self, initial_joints):
        self._samples = list(iter_open_loop(initial_joints))
        self._index = 0

    def reset(self, initial_joints) -> None:
        self._samples = list(iter_open_loop(initial_joints))
        self._index = 0

    @property
    def done(self) -> bool:
        return self._index >= len(self._samples)

    def next_sample(self) -> HeadsetSample | None:
        if self.done:
            return None
        sample = self._samples[self._index]
        self._index += 1
        return sample


def _engines_path() -> Path:
    return Path(__file__).resolve().parents[3] / "omnibot-ai-engines"


def write_reject_report(dataset_root, markdown_path, json_path=None):
    """Run the OHH-99 filter and write the keep/reject report.

    Counts and reasons come from ``filter_dataset``. This function does not
    invent episodes.
    """
    engines = str(_engines_path())
    if engines not in sys.path:
        sys.path.insert(0, engines)
    from data_engine.quality.episode_filter import filter_dataset

    root = Path(dataset_root)
    result = filter_dataset(root)
    json_out = Path(json_path) if json_path else root / "meta" / "filter_results.json"
    result.save_json(json_out)

    lines = [
        "# OHH-99 reject report",
        "",
        f"Dataset: `{root}`",
        "",
        "Filter: `data_engine.quality.episode_filter` (OHH-99). "
        "Thresholds are that filter's defaults "
        "(frozen video 0.5 s, camera skew 50 ms, joint step 0.5 rad, "
        "minimum duration 2.0 s).",
        "",
        "## Summary",
        "",
        f"- total episodes: {result.summary['total_episodes']}",
        f"- kept episodes: {result.summary['kept_episodes']}",
        f"- rejected episodes: {result.summary['rejected_episodes']}",
        f"- keep ratio: {result.summary['keep_ratio']}",
        "",
        "## Episodes",
        "",
    ]
    if not result.episodes:
        lines.append("No episodes were in the dataset. Nothing was scored.")
    for index in sorted(result.episodes):
        score = result.episodes[index]
        if score.passed:
            lines.append(
                f"- {index:06d} KEEP "
                f"(duration {score.metrics.duration_s:.2f} s, "
                f"joint step {score.metrics.max_joint_step_rad:.3f} rad, "
                f"frozen {score.metrics.max_frozen_duration_s:.2f} s)"
            )
        else:
            lines.append(f"- {index:06d} REJECT: {'; '.join(score.reasons)}")
    lines.append("")
    lines.append(f"JSON: `{json_out}`")
    lines.append("")
    markdown = Path(markdown_path)
    markdown.parent.mkdir(parents=True, exist_ok=True)
    markdown.write_text("\n".join(lines), encoding="utf-8")
    return result


def main(argv=None) -> int:
    """Write a reject report. Does not record and does not publish."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Write an OHH-99 reject report for a recorded dataset."
    )
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--json", default=None)
    args = parser.parse_args(argv)
    result = write_reject_report(args.dataset, args.report, args.json)
    summary = result.summary
    print(
        "OHH-99 "
        f"total={summary['total_episodes']} "
        f"kept={summary['kept_episodes']} "
        f"rejected={summary['rejected_episodes']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
