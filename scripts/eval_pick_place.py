#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""20-trial pick-and-place eval for the frozen OHH-98 task.

The task is: pick the rigid cube and place it in the bin. This script resets
the scene, runs N trials (default 20), and writes one result per trial plus a
markdown summary. It does not train a policy and it does not edit the OHH-243
ACT server.

Backends
--------
fake
    Scripted outcomes for the trial loop and the summary. No ROS and no
    Gazebo. The summary says the outcomes are scripted.
sim
    Gazebo world reset and set_pose via ``gz service``, then policy actions
    on ``/arm/joint_commands`` and ``/cmd_vel/vla`` (the mux inputs). Refuses
    to start unless ``ROS_LOCALHOST_ONLY=1`` and ``ROS_DOMAIN_ID`` is set to
    a value other than 30, and unless ``gz``, ``ros2``, and ``rclpy`` import.
real
    Stub. Refuses unless ``--real`` and ``--confirm VARUN-CONFIRM-REAL-EVAL``.
    After that phrase the process still does not open a robot device.

Policy
------
``--policy none`` uses a zero action (default).
``--policy serve`` POSTs to an already-running OhhO Serve (OHH-243) at
``/predict``. This file does not start or modify that server.
``--policy adapter`` calls ``lerobot_engine.models.make_policy`` (the
existing PolicyAdapter). Import failures abort the run.

Results are only ``success``, ``collision``, ``timeout``, or ``intervention``.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import random
import re
import shutil
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence


TASK_ID = "OHH-98"
TASK_NAME = "pick rigid object, place in bin"
DEFAULT_TRIALS = 20
RESULT_LABELS = ("success", "collision", "timeout", "intervention")
DEFAULT_SCRIPTED_OUTCOMES = RESULT_LABELS
REAL_CONFIRMATION = "VARUN-CONFIRM-REAL-EVAL"
DEFAULT_SERVE_URL = "http://127.0.0.1:8000"
ACTION_DIM = 9
POLICY_HZ = 10.0

# Scene contract shared with the apartment pick scene (table, rigid cube, bin).
# Draw order is the seed: cube x, cube y, cube yaw, bin x, bin y, bin yaw.
TABLE_NAME = "pick_place_table"
CUBE_NAME = "pick_place_cube"
BIN_NAME = "pick_place_bin"
TABLE_CENTER_X = 0.20
TABLE_CENTER_Y = 1.15
TABLE_HEIGHT = 0.72
TABLE_CENTER_Z = 0.37
TABLE_TOP_Z = TABLE_CENTER_Z + TABLE_HEIGHT / 2.0
CUBE_SIZE = 0.04
CUBE_CLEARANCE_Z = 0.002
BIN_SIZE_X = 0.18
BIN_SIZE_Y = 0.16
BIN_SIZE_Z = 0.08
BIN_WALL_M = 0.008
CUBE_X = (-0.18, 0.12)
CUBE_Y = (0.95, 1.35)
CUBE_YAW = (-0.5, 0.5)
BIN_X = (0.34, 0.52)
BIN_Y = (1.00, 1.30)
BIN_YAW = (-0.3, 0.3)

# arm_params.yaml. arm_safety.clamp_joint_command applies these after the delta cap.
JOINT_MIN = (-3.14, -1.57, -1.57, -1.57, -3.14, -0.1)
JOINT_MAX = (3.14, 1.57, 1.57, 1.57, 3.14, 0.8)
ARM_JOINT_NAMES = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)
# policy_guard.MAX_BASE_LIN_VEL / MAX_BASE_ANG_VEL and DEFAULT_BASE_VEL_SCALE.
MAX_BASE_LIN_VEL = 0.20
MAX_BASE_ANG_VEL = 0.50
BASE_VEL_SCALE = 0.3

ARM_COMMAND_TOPIC = "/arm/joint_commands"
BASE_COMMAND_TOPIC = "/cmd_vel/vla"
CONTROL_MODE_TOPIC = "/control_mode"
ARM_MODE_TOPIC = "/arm/cmd_mode"
ESTOP_TOPIC = "/emergency_stop"
WRIST_IMAGE_TOPIC = "/camera/wrist/image_raw"
BEV_IMAGE_TOPIC = "/camera/base/bev/image_raw"
JOINT_STATE_TOPIC = "/arm/joint_states"

BAG_TOPICS = (
    "/clock",
    "/odom",
    "/tf",
    JOINT_STATE_TOPIC,
    ARM_COMMAND_TOPIC,
    "/arm/joint_commands/out",
    BASE_COMMAND_TOPIC,
    "/cmd_vel/out",
    CONTROL_MODE_TOPIC,
    ESTOP_TOPIC,
    WRIST_IMAGE_TOPIC,
    BEV_IMAGE_TOPIC,
)

WRIST_KEY = "observation.images.wrist"
BEV_KEY = "observation.images.bev"
STATE_KEY = "observation.state"
TASK_KEY = "task"


class EvalError(RuntimeError):
    """The harness could not finish a trial."""


class RealEvalRefused(EvalError):
    """The real backend will not run."""


class SimUnavailable(EvalError):
    """Gazebo/ROS sim eval cannot start on this machine."""


class SimCommandError(EvalError):
    """A Gazebo or rosbag command failed."""


@dataclass(frozen=True)
class SpawnPose:
    """One model pose in the world frame."""

    name: str
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class PickPlaceLayout:
    """Fixed table plus the seeded cube and bin."""

    seed: int
    table: SpawnPose
    cube: SpawnPose
    bin: SpawnPose


@dataclass(frozen=True)
class ServiceCall:
    """One Gazebo world service invocation."""

    service: str
    request_type: str
    reply_type: str
    request: str
    argv: tuple[str, ...]


@dataclass
class SceneSense:
    """What the backend can see after a reset or a control step."""

    cube: tuple[float, float, float] | None = None
    bin_pose: tuple[float, float, float] | None = None
    collision: bool = False
    intervention: bool = False
    arm_positions: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    images: dict[str, Any] | None = None
    detail: str = ""


@dataclass
class TrialObservation:
    """Backend report for one trial, before the summary row is written."""

    result: str
    duration_s: float | None = None
    bag_recorded: bool = False
    detail: str = ""
    scripted: bool = False


@dataclass(frozen=True)
class TrialResult:
    """One persisted trial row."""

    trial: int
    seed: int
    result: str
    duration_s: float
    bag_path: str
    bag_recorded: bool
    backend: str
    task: str
    task_id: str
    detail: str
    scripted: bool

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class ZeroPolicy:
    """Nine zeros. Used when no checkpoint and no serve URL are requested."""

    def reset(self) -> None:
        return None

    def select_action(self, obs: Mapping[str, Any]) -> tuple[float, ...]:
        del obs
        return tuple(0.0 for _ in range(ACTION_DIM))


class HttpServePolicy:
    """Client for an already-running OhhO Serve ``POST /predict``.

    Matches the OHH-243 request fields (``instruction``, ``image_base64``,
    ``config``) and reads ``action.vector``. Does not load weights and does
    not start the server.
    """

    def __init__(
        self,
        base_url: str = DEFAULT_SERVE_URL,
        timeout_sec: float = 2.0,
        api_key: str = "",
    ):
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = float(timeout_sec)
        self.api_key = api_key

    def reset(self) -> None:
        return None

    def select_action(self, obs: Mapping[str, Any]) -> tuple[float, ...]:
        instruction = str(obs.get(TASK_KEY) or TASK_NAME)
        image = obs.get(WRIST_KEY)
        payload: dict[str, Any] = {
            "instruction": instruction,
            "image_base64": encode_image_b64(image),
            "config": {},
        }
        bev = obs.get(BEV_KEY)
        if bev is not None:
            payload["config"] = {
                "bev_image_b64": encode_image_b64(bev),
                "bev_shape": list(image_hwc_shape(bev)),
            }
        body = json.dumps(payload).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["X-API-Key"] = self.api_key
        request = urllib.request.Request(
            f"{self.base_url}/predict",
            data=body,
            method="POST",
            headers=headers,
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                raw = response.read()
        except urllib.error.URLError as exc:
            raise EvalError(f"OhhO Serve unreachable at {self.base_url}: {exc}") from exc
        document = json.loads(raw.decode("utf-8"))
        action = document.get("action", {})
        if isinstance(action, Mapping):
            vector = action.get("vector", action.get("action"))
        else:
            vector = action
        if vector is None:
            raise EvalError(f"Serve /predict returned no action: {document!r}")
        return _as_action(vector)


def image_hwc_uint8(image: Any) -> bytes:
    """OHH-243 Serve body: HWC uint8 bytes, CHW float batches transposed."""
    if image is None:
        return bytes((0, 0, 0))
    if isinstance(image, (bytes, bytearray)):
        return bytes(image)
    try:
        import numpy as np
    except ImportError:
        np = None  # type: ignore[assignment]
    if np is not None:
        array = np.asarray(image)
        if array.ndim >= 2:
            if array.ndim == 4:
                array = array[0]
            if (
                array.ndim == 3
                and array.shape[0] in (1, 3)
                and array.shape[-1] not in (1, 3)
            ):
                array = np.transpose(array, (1, 2, 0))
            if array.dtype != np.uint8:
                peak = float(np.max(array)) if array.size else 0.0
                if peak <= 1.0:
                    array = np.clip(array, 0.0, 1.0) * 255.0
                array = array.astype(np.uint8)
            return np.ascontiguousarray(array).tobytes()
    flat = _flatten_numbers(image)
    if not flat:
        return bytes((0, 0, 0))
    peak = max(flat)
    if peak <= 1.0:
        return bytes(max(0, min(255, int(round(value * 255.0)))) for value in flat)
    return bytes(max(0, min(255, int(round(value)))) for value in flat)


def image_hwc_shape(image: Any) -> tuple[int, ...]:
    try:
        import numpy as np
    except ImportError:
        return ()
    array = np.asarray(image)
    if array.ndim == 4:
        array = array[0]
    if array.ndim == 3 and array.shape[0] in (1, 3) and array.shape[-1] not in (1, 3):
        array = np.transpose(array, (1, 2, 0))
    if array.ndim < 2:
        return ()
    return tuple(int(dim) for dim in array.shape)


def encode_image_b64(image: Any) -> str:
    """Base64 of :func:`image_hwc_uint8`."""
    if isinstance(image, str):
        return image
    return base64.b64encode(image_hwc_uint8(image)).decode("ascii")


def _flatten_numbers(value: Any) -> list[float]:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return [float(value)]
    if isinstance(value, (str, bytes, bytearray)):
        return []
    items: list[float] = []
    try:
        iterator = iter(value)
    except TypeError:
        return []
    for item in iterator:
        items.extend(_flatten_numbers(item))
    return items


def _as_action(values: Sequence[Any]) -> tuple[float, ...]:
    nums = [float(value) for value in values]
    if len(nums) < ACTION_DIM:
        nums.extend([0.0] * (ACTION_DIM - len(nums)))
    return tuple(nums[:ACTION_DIM])


def load_arm_safety():
    """Import arm_safety without adding a dependency on rclpy."""
    try:
        import arm_safety
    except ImportError:
        scripts = (
            Path(__file__).resolve().parents[1]
            / "omnibot-ros2"
            / "omnibot_arm"
            / "scripts"
        )
        folder = str(scripts)
        if folder not in sys.path:
            sys.path.insert(0, folder)
        import arm_safety
    return arm_safety


def clamp_policy_action(
    action: Sequence[float],
    reference: Sequence[float] | None,
) -> tuple[tuple[float, ...], tuple[float, float, float]]:
    """Clamp with arm_safety's delta cap and the policy_node base limits.

    A missing reference skips the per-cycle delta inside arm_safety and only
    applies joint limits. Callers pass the previous command once one exists.
    """
    safety = load_arm_safety()
    padded = _as_action(action)
    ref = None if reference is None else [float(value) for value in reference]
    arm = safety.clamp_joint_command(
        list(padded[:6]),
        list(JOINT_MIN),
        list(JOINT_MAX),
        ref,
        safety.MAX_JOINT_DELTA_RAD,
    )
    base_limits = (MAX_BASE_LIN_VEL, MAX_BASE_LIN_VEL, MAX_BASE_ANG_VEL)
    base = []
    for value, limit in zip(padded[6:9], base_limits):
        scaled = float(value) * BASE_VEL_SCALE
        base.append(max(-limit, min(limit, scaled)))
    return tuple(float(value) for value in arm), (base[0], base[1], base[2])


def image_is_blank(image: Any) -> bool:
    """True when a frame is missing or every sample is zero."""
    if image is None:
        return True
    flat = _flatten_numbers(image)
    if not flat:
        return True
    return all(value == 0.0 for value in flat)


def _as_policy_image(image: Any) -> Any:
    """HWC uint8 list or array -> (1, C, H, W) float32 when numpy is present.

    Values already in 0..1 are left on that scale. Stand-in sequences that
    are not HxWxC are returned unchanged so a fake camera can stay a list.
    """
    if image is None:
        return None
    try:
        import numpy as np
    except ImportError:
        return image
    array = np.asarray(image)
    if array.ndim == 4:
        return array
    if array.ndim == 3 and array.shape[0] in (1, 3) and array.shape[-1] not in (1, 3):
        chw = array
    elif array.ndim == 3 and array.shape[-1] in (1, 3):
        chw = np.transpose(array, (2, 0, 1))
    else:
        return image
    if np.issubdtype(array.dtype, np.floating) and float(np.max(chw)) <= 1.0:
        chw = chw.astype(np.float32)
    else:
        chw = chw.astype(np.float32) / 255.0
    return chw[None, ...]


def build_policy_obs(sense: SceneSense) -> dict[str, Any]:
    """Observation dict for PolicyAdapter.select_action and OhhO Serve."""
    images = sense.images or {}
    arm = tuple(float(value) for value in list(sense.arm_positions)[:6])
    if len(arm) < 6:
        arm = arm + (0.0,) * (6 - len(arm))
    state = arm + (0.0, 0.0, 0.0)
    try:
        import numpy as np

        state_out: Any = np.asarray(state, dtype=np.float32)[None, :]
    except ImportError:
        state_out = state
    return {
        WRIST_KEY: _as_policy_image(images.get(WRIST_KEY)),
        BEV_KEY: _as_policy_image(images.get(BEV_KEY)),
        STATE_KEY: state_out,
        TASK_KEY: TASK_NAME,
    }


def observation_ready(images: Mapping[str, Any] | None) -> bool:
    if not images:
        return False
    return not image_is_blank(images.get(WRIST_KEY)) and not image_is_blank(
        images.get(BEV_KEY)
    )


def parse_seed(value: Any) -> int:
    if isinstance(value, bool) or value is None:
        raise ValueError(f"seed must be an integer, got {value!r}")
    try:
        return int(str(value).strip())
    except (TypeError, ValueError) as exc:
        raise ValueError(f"seed must be an integer, got {value!r}") from exc


def layout_for_seed(seed: Any) -> PickPlaceLayout:
    """Table is fixed. Cube and bin follow ``random.Random(seed)``."""
    parsed = parse_seed(seed)
    rng = random.Random(parsed)
    cube_x = rng.uniform(*CUBE_X)
    cube_y = rng.uniform(*CUBE_Y)
    cube_yaw = rng.uniform(*CUBE_YAW)
    bin_x = rng.uniform(*BIN_X)
    bin_y = rng.uniform(*BIN_Y)
    bin_yaw = rng.uniform(*BIN_YAW)
    return PickPlaceLayout(
        seed=parsed,
        table=SpawnPose(
            TABLE_NAME, TABLE_CENTER_X, TABLE_CENTER_Y, TABLE_CENTER_Z, 0.0
        ),
        cube=SpawnPose(
            CUBE_NAME,
            cube_x,
            cube_y,
            TABLE_TOP_Z + CUBE_SIZE / 2.0 + CUBE_CLEARANCE_Z,
            cube_yaw,
        ),
        bin=SpawnPose(BIN_NAME, bin_x, bin_y, TABLE_TOP_Z, bin_yaw),
    )


def validate_world(world: str) -> str:
    text = str(world).strip()
    if not text or any(not (ch.isalnum() or ch == "_") for ch in text):
        raise ValueError(f"world name must be alphanumeric, got {world!r}")
    return text


def _yaw_quaternion(yaw: float) -> tuple[float, float]:
    half = float(yaw) / 2.0
    return math.sin(half), math.cos(half)


def world_reset_call(world: str) -> ServiceCall:
    """Gazebo ``/world/<name>/control`` with ``reset {all: true}``."""
    name = validate_world(world)
    service = f"/world/{name}/control"
    request = "reset: {all: true}"
    argv = (
        "gz",
        "service",
        "-s",
        service,
        "--reqtype",
        "gz.msgs.WorldControl",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        request,
    )
    return ServiceCall(service, "gz.msgs.WorldControl", "gz.msgs.Boolean", request, argv)


def set_pose_call(world: str, pose: SpawnPose) -> ServiceCall:
    """Gazebo ``/world/<name>/set_pose`` for one model."""
    name = validate_world(world)
    qz, qw = _yaw_quaternion(pose.yaw)
    service = f"/world/{name}/set_pose"
    request = (
        f'name: "{pose.name}", '
        f"position: {{x: {pose.x:.6f}, y: {pose.y:.6f}, z: {pose.z:.6f}}}, "
        f"orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}"
    )
    argv = (
        "gz",
        "service",
        "-s",
        service,
        "--reqtype",
        "gz.msgs.Pose",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        request,
    )
    return ServiceCall(service, "gz.msgs.Pose", "gz.msgs.Boolean", request, argv)


def reset_plan(world: str, seed: Any) -> list[ServiceCall]:
    """World reset, then table, cube, and bin poses for this trial seed."""
    layout = layout_for_seed(seed)
    return [
        world_reset_call(world),
        set_pose_call(world, layout.table),
        set_pose_call(world, layout.cube),
        set_pose_call(world, layout.bin),
    ]


def cube_in_bin(
    cube: Sequence[float],
    bin_pose: Sequence[float],
) -> bool:
    """True when the cube centre sits inside the open bin.

    The bin origin is the bottom centre. Walls are 8 mm. The cube is 4 cm,
    so the centre has to clear both the walls and the cube half-extent.
    """
    half_x = (BIN_SIZE_X / 2.0) - BIN_WALL_M - (CUBE_SIZE / 2.0)
    half_y = (BIN_SIZE_Y / 2.0) - BIN_WALL_M - (CUBE_SIZE / 2.0)
    dx = abs(float(cube[0]) - float(bin_pose[0]))
    dy = abs(float(cube[1]) - float(bin_pose[1]))
    z_low = float(bin_pose[2]) + CUBE_SIZE / 2.0 - 0.005
    z_high = float(bin_pose[2]) + BIN_SIZE_Z - CUBE_SIZE / 2.0 + 0.01
    return dx <= half_x and dy <= half_y and z_low <= float(cube[2]) <= z_high


def cube_fell(cube_z: float) -> bool:
    """True when the cube is below the table top (it left the workspace)."""
    return float(cube_z) < TABLE_TOP_Z - 0.02


def _sense_detail(sense: SceneSense, label: str) -> str:
    if label == "collision" and sense.cube is not None and cube_fell(sense.cube[2]):
        return "cube fell below the table"
    if label == "collision" and sense.collision:
        return "collision flag"
    if label == "success":
        return "cube in bin"
    return ""


def sense_label(sense: SceneSense) -> str | None:
    """Map a scene reading to a terminal result, or None to keep stepping."""
    if sense.intervention:
        return "intervention"
    fell = sense.cube is not None and cube_fell(sense.cube[2])
    if sense.collision or fell:
        return "collision"
    if (
        sense.cube is not None
        and sense.bin_pose is not None
        and cube_in_bin(sense.cube, sense.bin_pose)
    ):
        return "success"
    return None


def _xyz_block(block: str) -> tuple[float, float, float] | None:
    values = {}
    for key in ("x", "y", "z"):
        match = re.search(rf"\b{key}\s*:\s*([-+0-9.eE]+)", block)
        if match is None:
            return None
        values[key] = float(match.group(1))
    return (values["x"], values["y"], values["z"])


def parse_pose_text(text: str) -> dict[str, tuple[float, float, float]]:
    """Pull ``name`` and ``position { x y z }`` pairs out of gz topic text."""
    poses: dict[str, tuple[float, float, float]] = {}
    names = list(re.finditer(r'name\s*:\s*"([^"]+)"', text))
    for index, match in enumerate(names):
        stop = names[index + 1].start() if index + 1 < len(names) else len(text)
        window = text[match.end() : stop]
        marker = window.find("position")
        if marker < 0:
            continue
        brace = window.find("{", marker)
        end = window.find("}", brace)
        if brace < 0 or end < 0:
            continue
        xyz = _xyz_block(window[brace + 1 : end])
        if xyz is not None:
            poses[match.group(1)] = xyz
    return poses


def sim_isolation_failure(environ: Mapping[str, str] | None = None) -> str | None:
    """Refuse when this process can see the robot DDS domain.

    Same rule as the scripted VR pick: localhost-only, and a domain id that
    is not 30 and not empty. Domain 0 is allowed only with localhost-only,
    matching that check. Values above 232 break the FastDDS port formula.
    """
    env = os.environ if environ is None else environ
    domain = env.get("ROS_DOMAIN_ID", "")
    localhost = env.get("ROS_LOCALHOST_ONLY", "")
    if domain in ("", "30") or localhost != "1":
        return (
            "Refusing sim eval. Set ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID to "
            "an integer other than 30 so this process cannot see the robot."
        )
    try:
        domain_id = int(domain)
    except ValueError:
        domain_id = -1
    if domain_id < 0 or domain_id > 232:
        return (
            "Refusing sim eval. ROS_DOMAIN_ID must be an integer from 0 to 232, "
            "other than 30."
        )
    return None


def sim_runtime_problems(which: Callable[[str], str | None] | None = None) -> list[str]:
    """Missing tools. Empty means a sim eval is allowed to try."""
    finder = shutil.which if which is None else which
    missing = []
    if finder("gz") is None:
        missing.append("gz")
    if finder("ros2") is None:
        missing.append("ros2")
    try:
        import rclpy  # noqa: F401
    except ImportError:
        missing.append("rclpy")
    return missing


def normalize_result(result: str) -> str:
    if result not in RESULT_LABELS:
        raise EvalError(
            f"result must be one of {RESULT_LABELS}, got {result!r}"
        )
    return result


def trial_bag_path(output_dir: Path, trial: int) -> str:
    return str(output_dir / f"trial_{trial:03d}")


def render_summary(results: Sequence[TrialResult]) -> str:
    """Markdown table of every trial plus a count per result label."""
    if not results:
        raise EvalError("no trials to summarise")
    backend = results[0].backend
    scripted = all(row.scripted for row in results)
    lines = [
        "# Pick-and-place eval",
        "",
        f"Task: {results[0].task} ({results[0].task_id})",
        f"Backend: {backend}",
        f"Trials: {len(results)}",
    ]
    if scripted:
        lines.append(
            "Outcomes are scripted by the fake backend. "
            "They are not a measured policy success rate."
        )
    lines.extend(
        [
            "",
            "| Trial | Seed | Result | Duration (s) | Bag path | Recorded |",
            "| --- | --- | --- | --- | --- | --- |",
        ]
    )
    for row in results:
        recorded = "yes" if row.bag_recorded else "no"
        lines.append(
            f"| {row.trial} | {row.seed} | {row.result} | {row.duration_s:.3f} | "
            f"{row.bag_path} | {recorded} |"
        )
    lines.extend(["", "| Result | Count |", "| --- | --- |"])
    for label in RESULT_LABELS:
        count = sum(1 for row in results if row.result == label)
        lines.append(f"| {label} | {count} |")
    lines.append("")
    return "\n".join(lines)


def run_eval(
    backend: Any,
    trials: int,
    output_dir: Path | str,
    seed_start: int = 0,
    task: str = TASK_NAME,
) -> list[TrialResult]:
    """Reset, run ``trials``, append JSONL, and write ``summary.md``."""
    count = int(trials)
    if count < 1:
        raise ValueError("trials must be >= 1")
    start = parse_seed(seed_start)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    jsonl_path = out / "trials.jsonl"
    jsonl_path.write_text("", encoding="utf-8")
    results: list[TrialResult] = []
    for trial in range(1, count + 1):
        seed = start + trial - 1
        bag_path = trial_bag_path(out, trial)
        started = time.perf_counter()
        backend.reset_scene(trial, seed)
        observation = backend.run_trial(trial, seed, bag_path)
        elapsed = time.perf_counter() - started
        duration = (
            float(observation.duration_s)
            if observation.duration_s is not None
            else elapsed
        )
        row = TrialResult(
            trial=trial,
            seed=seed,
            result=normalize_result(observation.result),
            duration_s=duration,
            bag_path=bag_path,
            bag_recorded=bool(observation.bag_recorded),
            backend=str(getattr(backend, "name", "unknown")),
            task=task,
            task_id=TASK_ID,
            detail=str(observation.detail or ""),
            scripted=bool(observation.scripted),
        )
        results.append(row)
        with jsonl_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(row.as_dict()) + "\n")
    summary = render_summary(results)
    (out / "summary.md").write_text(summary, encoding="utf-8")
    return results


class FakeBackend:
    """Scripted trial loop. Does not open ROS, Gazebo, or a device."""

    name = "fake"

    def __init__(
        self,
        outcomes: Sequence[str] = DEFAULT_SCRIPTED_OUTCOMES,
        policy: Any | None = None,
        duration_for: Callable[[int], float] | None = None,
    ):
        if not outcomes:
            raise ValueError("fake backend needs at least one outcome")
        self.outcomes = tuple(normalize_result(item) for item in outcomes)
        self.policy = policy if policy is not None else ZeroPolicy()
        self.duration_for = duration_for or (lambda trial: 1.0 + 0.1 * (trial - 1))
        self.resets: list[tuple[int, int]] = []

    def reset_scene(self, trial: int, seed: int) -> None:
        self.resets.append((int(trial), int(seed)))
        self.policy.reset()

    def run_trial(self, trial: int, seed: int, bag_path: str) -> TrialObservation:
        del seed, bag_path
        self.policy.select_action({TASK_KEY: TASK_NAME, "trial": int(trial)})
        label = self.outcomes[(int(trial) - 1) % len(self.outcomes)]
        return TrialObservation(
            result=label,
            duration_s=float(self.duration_for(int(trial))),
            bag_recorded=False,
            detail="scripted fake backend",
            scripted=True,
        )


class RealBackend:
    """Gate for a hardware eval. This object never opens a device.

    ``--real`` plus the confirmation phrase only changes the refusal text.
    ``reset_scene`` still raises, so the trial loop cannot move a robot.
    """

    name = "real"

    def __init__(self, *, real_flag: bool, confirmation: str):
        if not real_flag:
            raise RealEvalRefused(
                "Real backend refuses to run without --real. "
                "This process does not connect to the robot."
            )
        if confirmation != REAL_CONFIRMATION:
            raise RealEvalRefused(
                "Real backend refuses to run without "
                f"--confirm {REAL_CONFIRMATION} (Varun only)."
            )
        self._authorized = True

    def reset_scene(self, trial: int, seed: int) -> None:
        del trial, seed
        raise RealEvalRefused(
            "Real backend accepted --real and the confirmation phrase, "
            "but this process does not open serial, USB, CAN, or a robot "
            "DDS session. Varun runs hardware eval on the robot."
        )

    def run_trial(self, trial: int, seed: int, bag_path: str) -> TrialObservation:
        del trial, seed, bag_path
        raise RealEvalRefused(
            "Real backend has no hardware runner in this checkout."
        )


class GazeboSimBackend:
    """Reset Gazebo, step a policy, and classify the scene.

    ``transport`` performs service calls, sensing, and already-clamped
    publishes. The fake transport used in tests never talks to a device.
    """

    name = "sim"

    def __init__(
        self,
        transport: Any,
        world: str = "apartment",
        policy: Any | None = None,
        max_steps: int = 20,
        hz: float = POLICY_HZ,
    ):
        self.transport = transport
        self.world = validate_world(world)
        self.policy = policy if policy is not None else ZeroPolicy()
        self.max_steps = int(max_steps)
        if self.max_steps < 1:
            raise ValueError("max_steps must be >= 1")
        self.hz = float(hz) if hz > 0 else POLICY_HZ
        self._last_arm: tuple[float, ...] | None = None
        self.saw_cube = False

    def reset_scene(self, trial: int, seed: int) -> None:
        del trial
        for call in reset_plan(self.world, seed):
            self.transport.exec_service(call)
        self._last_arm = None
        self.saw_cube = False
        self.policy.reset()

    def run_trial(self, trial: int, seed: int, bag_path: str) -> TrialObservation:
        del trial, seed
        recorded = bool(self.transport.start_bag(bag_path))
        detail = ""
        label = "timeout"
        steps = 0
        try:
            if self.transport.estop_latched():
                label = "intervention"
                detail = "emergency stop"
            else:
                self.transport.request_vla_mode()
                for steps in range(1, self.max_steps + 1):
                    if self.transport.estop_latched():
                        label = "intervention"
                        detail = "emergency stop"
                        break
                    sense = self.transport.sense()
                    if sense.cube is not None:
                        self.saw_cube = True
                    decided = sense_label(sense)
                    if decided is not None:
                        label = decided
                        detail = sense.detail or _sense_detail(sense, decided)
                        break
                    self._step_policy(sense)
                else:
                    label = "timeout"
                    if not self.saw_cube:
                        detail = "timeout; cube pose was not observed"
                    else:
                        detail = "timeout before the cube was in the bin"
        finally:
            self.transport.stop_bag()
        return TrialObservation(
            result=label,
            duration_s=steps / self.hz,
            bag_recorded=recorded,
            detail=detail,
            scripted=False,
        )

    def _step_policy(self, sense: SceneSense) -> None:
        if self.transport.estop_latched():
            return
        if not observation_ready(sense.images):
            self.transport.send_clamped(None, (0.0, 0.0, 0.0))
            return
        obs = build_policy_obs(sense)
        action = self.policy.select_action(obs)
        reference = sense.arm_positions if self._last_arm is None else self._last_arm
        arm, base = clamp_policy_action(action, reference)
        self._last_arm = arm
        self.transport.send_clamped(arm, base)


def rosbag_record_argv(bag_path: str, topics: Sequence[str] = BAG_TOPICS) -> list[str]:
    return ["ros2", "bag", "record", "-s", "mcap", "-o", bag_path, *topics]


def pose_topic_argv(world: str) -> tuple[str, ...]:
    name = validate_world(world)
    return (
        "gz",
        "topic",
        "-e",
        "-t",
        f"/world/{name}/pose/info",
        "-n",
        "1",
    )


class GzRosTransport:
    """Gazebo CLI reset plus ROS publishes on the policy mux inputs.

    Constructed only after isolation and runtime checks pass. Publishes
    ``/arm/joint_commands`` and ``/cmd_vel/vla``. Does not publish
    ``/emergency_stop``, ``/cmd_vel``, or ``/arm/joint_commands/out``.
    """

    def __init__(self, world: str, runner: Callable[..., Any] | None = None):
        self.world = validate_world(world)
        self._runner = runner if runner is not None else _run_command
        self._bag: subprocess.Popen[Any] | None = None
        self._node = None
        self._estop = False
        self._images: dict[str, Any] = {}
        self._arm = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._pubs: dict[str, Any] = {}

    def start(self) -> None:
        failure = sim_isolation_failure()
        if failure:
            raise SimUnavailable(failure)
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Bool, String

        if not rclpy.ok():
            rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
        node = Node("ohh98_eval_pick_place")
        self._node = node
        self._Twist = Twist
        self._JointState = JointState
        self._String = String
        self._pubs["arm"] = node.create_publisher(JointState, ARM_COMMAND_TOPIC, 10)
        self._pubs["base"] = node.create_publisher(Twist, BASE_COMMAND_TOPIC, 10)
        self._pubs["mode"] = node.create_publisher(String, CONTROL_MODE_TOPIC, 10)
        self._pubs["arm_mode"] = node.create_publisher(String, ARM_MODE_TOPIC, 10)
        node.create_subscription(Bool, ESTOP_TOPIC, self._on_estop, 10)
        node.create_subscription(Image, WRIST_IMAGE_TOPIC, self._on_wrist, 10)
        node.create_subscription(Image, BEV_IMAGE_TOPIC, self._on_bev, 10)
        node.create_subscription(JointState, JOINT_STATE_TOPIC, self._on_joints, 10)
        rclpy.spin_once(node, timeout_sec=0.0)

    def close(self) -> None:
        self.stop_bag()
        node = self._node
        self._node = None
        if node is not None:
            node.destroy_node()

    def exec_service(self, call: ServiceCall) -> None:
        self._runner(list(call.argv), timeout=10)

    def start_bag(self, bag_path: str) -> bool:
        argv = rosbag_record_argv(bag_path)
        self._bag = subprocess.Popen(
            argv,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=(os.name != "nt"),
        )
        time.sleep(0.5)
        if self._bag.poll() is not None:
            code = self._bag.returncode
            self._bag = None
            raise SimCommandError(f"ros2 bag record exited {code}: {' '.join(argv)}")
        return True

    def stop_bag(self) -> None:
        proc = self._bag
        self._bag = None
        if proc is None or proc.poll() is not None:
            return
        if os.name == "nt":
            proc.terminate()
            return
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()

    def request_vla_mode(self) -> None:
        self._spin()
        if self._estop:
            return
        mode = self._String()
        mode.data = "vla"
        arm_mode = self._String()
        arm_mode.data = "policy"
        self._pubs["mode"].publish(mode)
        self._pubs["arm_mode"].publish(arm_mode)

    def estop_latched(self) -> bool:
        self._spin()
        return bool(self._estop)

    def sense(self) -> SceneSense:
        self._spin()
        try:
            text = self._runner(list(pose_topic_argv(self.world)), timeout=5)
        except SimCommandError:
            text = ""
        poses = parse_pose_text(text or "")
        cube = poses.get(CUBE_NAME)
        bin_pose = poses.get(BIN_NAME)
        return SceneSense(
            cube=cube,
            bin_pose=bin_pose,
            collision=False,
            intervention=self._estop,
            arm_positions=self._arm,
            images=dict(self._images),
        )

    def send_clamped(
        self,
        arm: Sequence[float] | None,
        base: Sequence[float],
    ) -> None:
        if self._estop:
            return
        self._spin()
        if self._estop:
            return
        if arm is not None:
            message = self._JointState()
            message.name = list(ARM_JOINT_NAMES)
            message.position = [float(value) for value in arm]
            self._pubs["arm"].publish(message)
        twist = self._Twist()
        twist.linear.x = float(base[0])
        twist.linear.y = float(base[1])
        twist.angular.z = float(base[2])
        self._pubs["base"].publish(twist)

    def _spin(self) -> None:
        if self._node is None:
            return
        import rclpy

        rclpy.spin_once(self._node, timeout_sec=0.0)

    def _on_estop(self, msg: Any) -> None:
        self._estop = bool(msg.data)

    def _on_wrist(self, msg: Any) -> None:
        converted = _image_msg_to_samples(msg)
        if converted is not None:
            self._images[WRIST_KEY] = converted

    def _on_bev(self, msg: Any) -> None:
        converted = _image_msg_to_samples(msg)
        if converted is not None:
            self._images[BEV_KEY] = converted

    def _on_joints(self, msg: Any) -> None:
        positions = list(msg.position)
        if len(positions) >= 6:
            self._arm = tuple(float(value) for value in positions[:6])


def _image_msg_to_samples(msg: Any) -> Any:
    """rgb8/bgr8 bytes as a nested HWC list. Other encodings are ignored."""
    encoding = str(getattr(msg, "encoding", ""))
    if encoding not in ("rgb8", "bgr8"):
        return None
    width = int(msg.width)
    height = int(msg.height)
    raw = bytes(msg.data)
    if width < 1 or height < 1 or len(raw) < width * height * 3:
        return None
    rows = []
    for y in range(height):
        row = []
        for x in range(width):
            i = (y * width + x) * 3
            pixel = [raw[i], raw[i + 1], raw[i + 2]]
            if encoding == "bgr8":
                pixel = [pixel[2], pixel[1], pixel[0]]
            row.append(pixel)
        rows.append(row)
    return rows


def _run_command(argv: list[str], timeout: float = 10) -> str:
    if shutil.which(argv[0]) is None:
        raise SimUnavailable(f"{argv[0]} is not on PATH")
    try:
        completed = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise SimCommandError(f"{' '.join(argv)} timed out") from exc
    if completed.returncode != 0:
        raise SimCommandError(
            f"{' '.join(argv)} exited {completed.returncode}: {completed.stderr.strip()}"
        )
    return completed.stdout


def build_backend(
    name: str,
    *,
    real_flag: bool = False,
    confirmation: str = "",
    world: str = "apartment",
    policy: Any | None = None,
    max_steps: int = 200,
    which: Callable[[str], str | None] | None = None,
    environ: Mapping[str, str] | None = None,
) -> Any:
    """Select a backend. Sim and real raise instead of touching hardware."""
    if name == "fake":
        return FakeBackend(policy=policy)
    if name == "real":
        return RealBackend(real_flag=real_flag, confirmation=confirmation)
    if name == "sim":
        failure = sim_isolation_failure(environ)
        if failure:
            raise SimUnavailable(failure)
        missing = sim_runtime_problems(which)
        if missing:
            raise SimUnavailable(
                "Sim eval needs "
                + ", ".join(missing)
                + ". Refusing to invent Gazebo results."
            )
        transport = GzRosTransport(world)
        transport.start()
        return GazeboSimBackend(
            transport,
            world=world,
            policy=policy,
            max_steps=max_steps,
        )
    raise EvalError(f"unknown backend {name!r}")


def load_policy(kind: str, serve_url: str, model: str, checkpoint: str, device: str) -> Any:
    if kind == "none":
        return ZeroPolicy()
    if kind == "serve":
        return HttpServePolicy(
            serve_url,
            api_key=os.environ.get("OHHO_SERVE_API_KEY", ""),
        )
    if kind == "adapter":
        engines = Path(__file__).resolve().parents[1] / "omnibot-ai-engines"
        folder = str(engines)
        if folder not in sys.path:
            sys.path.insert(0, folder)
        try:
            from lerobot_engine.models import make_policy
        except ImportError as exc:
            raise EvalError(
                "PolicyAdapter is not importable. OHH-243 serve is the "
                f"other way to run ACT ({exc})."
            ) from exc
        if not checkpoint:
            raise EvalError("--checkpoint is required for --policy adapter")
        return make_policy(model, checkpoint, device)
    raise EvalError(f"unknown policy {kind!r}")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OHH-98 pick-and-place eval (fake, Gazebo sim, or a real stub)."
    )
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS)
    parser.add_argument("--backend", choices=("fake", "sim", "real"), default="fake")
    parser.add_argument("--output", type=Path, default=Path("eval_runs/ohh-98-pick-place"))
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--world", default="apartment")
    parser.add_argument("--max-steps", type=int, default=200)
    parser.add_argument("--policy", choices=("none", "serve", "adapter"), default="none")
    parser.add_argument("--serve-url", default=DEFAULT_SERVE_URL)
    parser.add_argument("--model", default="act")
    parser.add_argument("--checkpoint", default="")
    parser.add_argument("--device", default="cpu")
    parser.add_argument(
        "--real",
        action="store_true",
        help="Required with --backend real. Does not open a device by itself.",
    )
    parser.add_argument(
        "--confirm",
        default="",
        help=f"Must be {REAL_CONFIRMATION} together with --real (Varun only).",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    backend = None
    try:
        policy = load_policy(
            args.policy,
            args.serve_url,
            args.model,
            args.checkpoint,
            args.device,
        )
        backend = build_backend(
            args.backend,
            real_flag=args.real,
            confirmation=args.confirm,
            world=args.world,
            policy=policy,
            max_steps=args.max_steps,
        )
        if isinstance(backend, RealBackend):
            # Confirmed flag only. reset_scene raises before any trial files.
            backend.reset_scene(1, args.seed_start)
            print("real backend must not return from reset_scene", file=sys.stderr)
            return 2
        results = run_eval(
            backend,
            trials=args.trials,
            output_dir=args.output,
            seed_start=args.seed_start,
        )
    except (EvalError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    finally:
        transport = getattr(backend, "transport", None)
        close = getattr(transport, "close", None)
        if close is not None:
            close()
    summary_path = Path(args.output) / "summary.md"
    print(summary_path.read_text(encoding="utf-8"))
    print(f"wrote {summary_path} ({len(results)} trials)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
