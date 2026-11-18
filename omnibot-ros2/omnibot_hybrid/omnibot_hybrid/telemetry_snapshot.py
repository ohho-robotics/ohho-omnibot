"""Assemble one ohho.telemetry.v1 sample. No ROS imports.

The VR HUD reads ``/ohho/telemetry``. This module turns the latest cached
messages plus a clock into that JSON. Optional blocks are omitted until every
required field is a real number: zero is kept when a source sent zero, and it
is not invented for a missing array.

``age_ms`` is per published field, in milliseconds on the same clock as the
cache stamps. ohho.telemetry.v1 sets ``additionalProperties`` false, so
``schema_body`` drops ``age_ms`` before jsonschema. The HUD uses ``age_ms``
to grey a value out. Suggested thresholds live in ``SUGGESTED_GREY_MS``.
They are not safety watchdogs and they do not drop a sample.
"""

from __future__ import annotations

import json
import math
import re
from typing import Any, Mapping, Optional, Sequence

import yaml

SCHEMA_ID = "ohho.telemetry.v1"
SCHEMA_VERSION = "1.0.0"

_VERSION = re.compile(r"^1\.0\.(0|[1-9]\d*)$")
_ROBOT_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")
_FRAME_ID = re.compile(r"^[A-Za-z][A-Za-z0-9_./:-]{0,63}$")
_JOINT_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_./:-]{0,63}$")
_DISPLAY_NAME = re.compile(r"^\S(?:.*\S)?$")

# HUD grey-out hints for a 20 Hz snapshot. Mode, safety, and the recorder
# publish at about 1 Hz, so their hints sit above that period.
SUGGESTED_GREY_MS = {
    "base": 250.0,
    "joints": 250.0,
    "imu": 250.0,
    "gripper": 250.0,
    "battery": 5000.0,
    "nav": 1000.0,
    "policy": 1000.0,
    "rec": 2000.0,
    "mode": 2500.0,
    "estop": 2500.0,
    "deadman": 2500.0,
    "sys": 5000.0,
    "link": 1000.0,
}

_BASE_MODES = frozenset({"idle", "teleop", "auto", "hold", "fault", "not_present"})
_AUTO_CONTROL = frozenset({"nav2", "vla", "rl_nav", "auto"})
_ARM_SCHEMA = frozenset({"idle", "teleop", "auto", "hold", "fault", "not_present"})
_POLICY_STATES = frozenset({"idle", "running", "paused", "fault"})
_ESTOP_SOURCES = frozenset({"none", "hardware", "software", "remote", "watchdog"})

# action_msgs/GoalStatus
_NAV_FROM_STATUS = {
    0: "idle",
    1: "navigating",
    2: "navigating",
    3: "navigating",
    4: "succeeded",
    5: "aborted",
    6: "aborted",
    "unknown": "idle",
    "accepted": "navigating",
    "executing": "navigating",
    "canceling": "navigating",
    "succeeded": "succeeded",
    "canceled": "aborted",
    "cancelled": "aborted",
    "aborted": "aborted",
    "failed": "failed",
    "idle": "idle",
    "navigating": "navigating",
}


def dumps_json(document: Mapping[str, Any]) -> str:
    """Stable JSON. NaN and Infinity raise ValueError."""
    return json.dumps(
        document,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )


def schema_body(document: Mapping[str, Any]) -> dict[str, Any]:
    """v1 document without the HUD ``age_ms`` object."""
    return {key: value for key, value in document.items() if key != "age_ms"}


def resolve_robot_id(requested: str, *, sim: bool) -> str:
    """Use ``requested`` when it matches the schema, otherwise a stable default."""
    text = requested.strip() if isinstance(requested, str) else ""
    if _ROBOT_ID.match(text):
        return text
    return "omnibot-sim" if sim else "omnibot"


def joint_limits_from_sequences(
    names: Sequence[Any],
    mins: Sequence[Any],
    maxs: Sequence[Any],
) -> dict[str, tuple[float, float]]:
    """Pair joint names with radian limits. Order is kept."""
    if not (len(names) == len(mins) == len(maxs)) or not names:
        raise ValueError("joint_names, joint_min, and joint_max must be the same non-empty length")
    limits: dict[str, tuple[float, float]] = {}
    for name, lo, hi in zip(names, mins, maxs):
        if not isinstance(name, str) or not _JOINT_NAME.match(name):
            raise ValueError(f"joint name {name!r} is not a schema joint name")
        lo_n = _finite(lo)
        hi_n = _finite(hi)
        if lo_n is None or hi_n is None:
            raise ValueError(f"joint limits for {name} are not finite numbers")
        limits[name] = (lo_n, hi_n)
    return limits


def joint_limits_from_arm_params(text: str) -> dict[str, tuple[float, float]]:
    """Read ``joint_names`` / ``joint_min`` / ``joint_max`` from arm_params.yaml."""
    loaded = yaml.safe_load(text)
    if not isinstance(loaded, dict):
        raise ValueError("arm_params.yaml is empty")
    params = loaded
    node = loaded.get("arm_driver_node")
    if isinstance(node, dict) and isinstance(node.get("ros__parameters"), dict):
        params = node["ros__parameters"]
    elif isinstance(loaded.get("ros__parameters"), dict):
        params = loaded["ros__parameters"]
    names = params.get("joint_names")
    mins = params.get("joint_min")
    maxs = params.get("joint_max")
    if not isinstance(names, list) or not isinstance(mins, list) or not isinstance(maxs, list):
        raise ValueError("arm_params.yaml is missing joint_names, joint_min, or joint_max")
    return joint_limits_from_sequences(names, mins, maxs)


def percent_from_voltage(voltage: Any, empty_v: float, full_v: float) -> Optional[float]:
    """Map pack voltage to percent 0..100. Voltage itself is not clamped."""
    volts = _finite(voltage)
    if volts is None or volts < 0:
        return None
    span = full_v - empty_v
    if span <= 0:
        return None
    pct = (volts - empty_v) / span * 100.0
    if pct < 0.0:
        return 0.0
    if pct > 100.0:
        return 100.0
    return pct


def rpy_from_quaternion(
    x: Any, y: Any, z: Any, w: Any
) -> Optional[tuple[float, float, float]]:
    """Roll, pitch, yaw in radians. ``None`` when the quaternion is unusable."""
    parts = [_finite(x), _finite(y), _finite(z), _finite(w)]
    if any(part is None for part in parts):
        return None
    qx, qy, qz, qw = parts
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        return None
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (qw * qy - qz * qx)
    if sinp >= 1.0:
        pitch = math.pi / 2.0
    elif sinp <= -1.0:
        pitch = -math.pi / 2.0
    else:
        pitch = math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def duration_seconds(sec: Any, nanosec: Any) -> Optional[float]:
    """ROS duration to seconds. Negative durations are rejected."""
    whole = _finite(sec)
    frac = _finite(nanosec)
    if whole is None or frac is None:
        return None
    out = whole + frac * 1e-9
    if out < 0.0 or not math.isfinite(out):
        return None
    return out


def nav_phase(status: Any) -> Optional[str]:
    """Map a Nav2 goal status code or name onto the schema nav phase."""
    if isinstance(status, bool):
        return None
    if isinstance(status, int):
        return _NAV_FROM_STATUS.get(status)
    if isinstance(status, str):
        return _NAV_FROM_STATUS.get(status.strip().lower())
    return None


def select_goal_status(codes: Sequence[Any]) -> Optional[int]:
    """Prefer an active NavigateToPose goal over a finished one."""
    cleaned: list[int] = []
    for code in codes:
        if isinstance(code, bool) or not isinstance(code, int):
            continue
        cleaned.append(code)
    if not cleaned:
        return None
    for preferred in (2, 1, 3):
        if preferred in cleaned:
            return preferred
    return cleaned[-1]


def map_base_mode(control_mode: Optional[str]) -> str:
    """``/control_mode/active`` string to a schema base mode."""
    if not isinstance(control_mode, str):
        return "idle"
    mode = control_mode.strip().lower()
    if mode in _AUTO_CONTROL:
        return "auto"
    if mode in _BASE_MODES:
        return mode
    return "idle"


def map_arm_mode(
    control_mode: Optional[str],
    arm_mode: Optional[str],
    *,
    arm_present: bool,
) -> str:
    """Arm mux mode to a schema arm mode.

    Teleop forwards ``/arm/joint_commands/teleop`` even when the arm-source
    preference is ``policy``. Autonomous control modes report ``auto``.
    """
    if not arm_present:
        return "not_present"
    control = control_mode.strip().lower() if isinstance(control_mode, str) else ""
    arm = arm_mode.strip().lower() if isinstance(arm_mode, str) else ""
    if control == "teleop":
        return "teleop"
    if control in _AUTO_CONTROL:
        return "auto"
    if arm in _ARM_SCHEMA:
        return arm
    return "idle"


def map_estop_deadman(raw: Optional[Mapping[str, Any]]) -> tuple[dict[str, Any], bool, bool]:
    """Map ``/ohho/safety/state`` onto ``estop`` and ``deadman``.

    Returns ``(estop, deadman, measured)``. ``measured`` is false when no
    safety sample has arrived: deadman stays false and the e-stop is not
    invented. A latched gate is reported latched. This does not clear, latch,
    or command anything.

    The live gate publishes ``{"armed", "latched", "reason"}``. A document
    that already has schema ``estop`` and ``deadman`` is passed through.
    """
    clear = {"active": False, "latched": False, "source": "none"}
    if not isinstance(raw, Mapping):
        return clear, False, False
    if isinstance(raw.get("estop"), Mapping) and "deadman" in raw:
        estop = _normalize_estop(raw["estop"])
        return estop, raw.get("deadman") is True, True
    latched = raw.get("latched") is True
    armed = raw.get("armed") is True
    reason = raw.get("reason")
    reason_text = reason.strip().lower() if isinstance(reason, str) else ""
    if not latched:
        return clear, armed, True
    source = "software"
    for name in ("hardware", "remote", "watchdog", "software"):
        if name in reason_text:
            source = name
            break
    return {"active": False, "latched": True, "source": source}, armed, True


def parse_heartbeat(message: Any) -> Optional[dict[str, Any]]:
    """Accept one ohho.telemetry.v1 heartbeat object. Extra fields are rejected."""
    if not isinstance(message, Mapping):
        return None
    if set(message) != {"v", "seq", "t_client"}:
        return None
    version = message.get("v")
    if not isinstance(version, str) or not _VERSION.match(version):
        return None
    seq = message.get("seq")
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
        return None
    t_client = _nonneg_number(message.get("t_client"))
    if t_client is None:
        return None
    return {"v": version, "seq": seq, "t_client": t_client}


def echo_heartbeat(message: Any, server_time_s: Any) -> Optional[dict[str, Any]]:
    """Copy ``seq`` and ``t_client`` and stamp server time. Invalid input is None."""
    parsed = parse_heartbeat(message)
    server = _nonneg_number(server_time_s)
    if parsed is None or server is None:
        return None
    return {
        "v": parsed["v"],
        "seq": parsed["seq"],
        "t_client": parsed["t_client"],
        "t": server,
    }


def parse_recorder_status(text: str) -> Optional[dict[str, Any]]:
    """Recorder status to a schema ``rec`` object.

    Accepts a JSON object ``{active, episode, frames}`` or
    ``recording:<episode>:<frames>``. ``idle`` and ``recording:<episode>``
    without a frame count are omitted: the schema requires ``frames``, and
    zero is not substituted for an unknown count.
    """
    if not isinstance(text, str):
        return None
    raw = text.strip()
    if not raw or raw == "idle":
        return None
    if raw.startswith("{"):
        try:
            obj = json.loads(raw)
        except json.JSONDecodeError:
            return None
        if isinstance(obj, dict):
            return _recording_dict(obj)
        return None
    if raw.startswith("recording:"):
        rest = raw[len("recording:") :]
        head, sep, tail = rest.rpartition(":")
        if sep and tail.isdigit() and head.strip():
            return _recording_dict(
                {"active": True, "episode": head.strip(), "frames": int(tail)}
            )
    return None


def policy_from_parts(
    *,
    enabled: Optional[bool],
    task: Optional[str],
    latency_ms: Any,
    level: Optional[str] = None,
) -> Optional[dict[str, Any]]:
    """Build a policy block. Latency is required; it is not filled with zero."""
    latency = _finite(latency_ms)
    if latency is None or latency < 0.0:
        return None
    name = task.strip() if isinstance(task, str) else ""
    if not name:
        name = "policy"
    if not _DISPLAY_NAME.match(name) or len(name) > 128:
        return None
    if level in ("error", "stale"):
        state = "fault"
    elif enabled is True:
        state = "running"
    elif enabled is False:
        state = "idle"
    else:
        state = "idle"
    if state not in _POLICY_STATES:
        return None
    return {"name": name, "state": state, "latency_ms": latency}


def rate_hz(stamps: Sequence[float], now: float, window_s: float = 2.0) -> float:
    """Arrival rate over ``window_s``. Fewer than two stamps reports 0."""
    if window_s <= 0:
        return 0.0
    recent = []
    for stamp in stamps:
        value = _finite(stamp)
        if value is None or value > now or now - value > window_s:
            continue
        recent.append(value)
    if len(recent) < 2:
        return 0.0
    recent.sort()
    span = recent[-1] - recent[0]
    if span <= 0.0:
        return 0.0
    return (len(recent) - 1) / span


def fields_to_grey(
    age_ms: Mapping[str, Any],
    thresholds: Optional[Mapping[str, float]] = None,
) -> list[str]:
    """Field names whose age is at or above the suggested grey-out hint."""
    limits = SUGGESTED_GREY_MS if thresholds is None else thresholds
    stale: list[str] = []
    for key, age in age_ms.items():
        age_n = _finite(age)
        if age_n is None:
            continue
        block = str(key).split(".", 1)[0]
        limit = limits.get(key, limits.get(block))
        if limit is None:
            continue
        if age_n >= float(limit):
            stale.append(str(key))
    return stale


def assemble(
    now: float,
    sources: Mapping[str, Any],
    *,
    robot_id: str,
    sim: bool,
    joint_limits: Mapping[str, tuple[float, float]],
    rtt_ms: float = 0.0,
    heartbeat_hz: float = 0.0,
    arm_present: bool = True,
    sim_battery_v: Optional[float] = 12.6,
    sim_battery_pct: Optional[float] = 100.0,
    battery_empty_v: float = 9.9,
    battery_full_v: float = 12.6,
) -> dict[str, Any]:
    """Build one telemetry object, including ``age_ms``.

    ``sources`` values are ``{"t": receipt_time, "data": ...}`` or missing.
    ``now`` and those stamps share one clock. ``now`` is unix seconds and is
    copied into ``t``.

    ``sim`` sets ``kind`` and forces ``battery.simulated``. With no pack
    sample, sim emits ``sim_battery_v`` / ``sim_battery_pct``. Pass ``None``
    to omit that synthetic pack.
    """
    if not isinstance(sim, bool):
        raise TypeError("sim must be a bool")
    if not isinstance(arm_present, bool):
        raise TypeError("arm_present must be a bool")
    stamp = _finite(now)
    if stamp is None or stamp < 0.0:
        raise ValueError("now must be a finite unix time >= 0")
    if not isinstance(robot_id, str) or not _ROBOT_ID.match(robot_id):
        raise ValueError("robot_id does not match ohho.telemetry.v1")

    ages: dict[str, float] = {"link": 0.0}
    mode, mode_ages = _mode_state(sources, stamp, arm_present=arm_present)
    ages.update(mode_ages)
    estop, deadman, safety_ages = _safety_fields(sources, stamp)
    ages.update(safety_ages)

    document: dict[str, Any] = {
        "v": SCHEMA_VERSION,
        "t": stamp,
        "robot_id": robot_id,
        "kind": "sim" if sim else "real",
        "mode": mode,
        "estop": estop,
        "deadman": deadman,
        "link": {"rtt_ms": _nonneg(rtt_ms), "hz": _nonneg(heartbeat_hz)},
    }

    base, base_ages = _base_block(sources, stamp)
    if base is not None:
        document["base"] = base
        ages.update(base_ages)

    if arm_present:
        joints, joint_ages = _joints_block(sources, stamp, joint_limits)
        if joints:
            document["joints"] = joints
            ages.update(joint_ages)
        gripper, gripper_age = _gripper_block(sources.get("gripper"), stamp)
        if gripper is not None and gripper_age is not None:
            document["gripper"] = gripper
            ages["gripper"] = gripper_age

    imu, imu_age = _imu_block(sources.get("imu"), stamp)
    if imu is not None and imu_age is not None:
        document["imu"] = imu
        ages["imu"] = imu_age

    battery, battery_age = _battery_block(
        sources,
        stamp,
        sim=sim,
        sim_battery_v=sim_battery_v,
        sim_battery_pct=sim_battery_pct,
        empty_v=battery_empty_v,
        full_v=battery_full_v,
    )
    if battery is not None and battery_age is not None:
        document["battery"] = battery
        ages["battery"] = battery_age

    nav, nav_age = _nav_block(sources, stamp)
    if nav is not None and nav_age is not None:
        document["nav"] = nav
        ages["nav"] = nav_age

    policy, policy_age = _policy_block(sources.get("policy"), stamp)
    if policy is not None and policy_age is not None:
        document["policy"] = policy
        ages["policy"] = policy_age

    rec, rec_age = _rec_block(sources.get("recorder"), stamp)
    if rec is not None and rec_age is not None:
        document["rec"] = rec
        ages["rec"] = rec_age

    host, host_age = _sys_block(sources.get("sys"), stamp)
    if host is not None and host_age is not None:
        document["sys"] = host
        ages["sys"] = host_age

    document["age_ms"] = ages
    return document


def _mode_state(
    sources: Mapping[str, Any], now: float, *, arm_present: bool
) -> tuple[dict[str, str], dict[str, float]]:
    control = _sample(sources, "control_mode")
    arm = _sample(sources, "arm_mode")
    control_text = control["data"] if control and isinstance(control.get("data"), str) else None
    arm_text = arm["data"] if arm and isinstance(arm.get("data"), str) else None
    mode = {
        "base": map_base_mode(control_text),
        "arm": map_arm_mode(control_text, arm_text, arm_present=arm_present),
    }
    ages: dict[str, float] = {}
    if control is not None and control_text is not None:
        ages["mode.base"] = _age(now, control["t"])
    arm_age = _arm_age(now, control, control_text, arm, arm_text, arm_present)
    if arm_age is not None:
        ages["mode.arm"] = arm_age
    if ages:
        ages["mode"] = max(ages.values())
    return mode, ages


def _arm_age(now, control, control_text, arm, arm_text, arm_present) -> Optional[float]:
    if not arm_present:
        return None
    control_mode = control_text.strip().lower() if isinstance(control_text, str) else ""
    if control_mode == "teleop" and control is not None:
        return _age(now, control["t"])
    if control_mode in _AUTO_CONTROL and control is not None:
        ages = [_age(now, control["t"])]
        if arm is not None and isinstance(arm_text, str):
            ages.append(_age(now, arm["t"]))
        return max(ages)
    if arm is not None and isinstance(arm_text, str) and arm_text.strip().lower() in _ARM_SCHEMA:
        return _age(now, arm["t"])
    return None


def _safety_fields(
    sources: Mapping[str, Any], now: float
) -> tuple[dict[str, Any], bool, dict[str, float]]:
    sample = _sample(sources, "safety")
    raw = sample["data"] if sample and isinstance(sample.get("data"), Mapping) else None
    estop, deadman, measured = map_estop_deadman(raw)
    ages: dict[str, float] = {}
    if measured and sample is not None:
        age = _age(now, sample["t"])
        ages["estop"] = age
        ages["deadman"] = age
    return estop, deadman, ages


def _base_block(
    sources: Mapping[str, Any], now: float
) -> tuple[Optional[dict[str, Any]], dict[str, float]]:
    odom = _sample(sources, "odom")
    cmd = _sample(sources, "cmd_vel")
    if odom is None or cmd is None:
        return None, {}
    frame = odom["data"].get("frame") if isinstance(odom.get("data"), Mapping) else None
    if not isinstance(frame, str) or not _FRAME_ID.match(frame):
        return None, {}
    measured = _velocity(odom["data"])
    command = _velocity(cmd["data"])
    data = odom["data"]
    pose = (_finite(data.get("x")), _finite(data.get("y")), _finite(data.get("yaw")))
    if measured is None or command is None or any(part is None for part in pose):
        return None, {}
    cmd_age = _age(now, cmd["t"])
    pose_age = _age(now, odom["t"])
    block = {
        "cmd": command,
        "meas": measured,
        "pose": {"x": pose[0], "y": pose[1], "yaw": pose[2]},
        "frame": frame,
    }
    return block, {
        "base.cmd": cmd_age,
        "base.meas": pose_age,
        "base.pose": pose_age,
        "base": max(cmd_age, pose_age),
    }


def _joints_block(
    sources: Mapping[str, Any],
    now: float,
    limits: Mapping[str, tuple[float, float]],
) -> tuple[list[dict[str, Any]], dict[str, float]]:
    state = _sample(sources, "joint_state")
    cmd = _sample(sources, "joint_cmd")
    if state is None or cmd is None or not isinstance(state.get("data"), Mapping):
        return [], {}
    if not isinstance(cmd.get("data"), Mapping):
        return [], {}
    names = state["data"].get("names") or []
    positions = state["data"].get("position") or []
    velocities = state["data"].get("velocity") or []
    efforts = state["data"].get("effort") or []
    commanded = _command_map(cmd["data"], limits)
    state_age = _age(now, state["t"])
    cmd_age = _age(now, cmd["t"])
    age = max(state_age, cmd_age)
    samples: list[dict[str, Any]] = []
    ages: dict[str, float] = {}
    for index, name in enumerate(names):
        if not isinstance(name, str) or name not in limits or not _JOINT_NAME.match(name):
            continue
        pos = _at(positions, index)
        vel = _at(velocities, index)
        effort = _at(efforts, index)
        target = commanded.get(name)
        if pos is None or vel is None or effort is None or target is None:
            continue
        lo, hi = limits[name]
        samples.append(
            {
                "name": name,
                "pos": pos,
                "vel": vel,
                "effort": effort,
                "cmd": target,
                "min": lo,
                "max": hi,
            }
        )
        ages[f"joints.{name}"] = age
    if samples:
        ages["joints"] = age
    return samples, ages


def _command_map(
    cmd: Mapping[str, Any], limits: Mapping[str, tuple[float, float]]
) -> dict[str, float]:
    raw_names = cmd.get("names") or []
    names = list(raw_names) if isinstance(raw_names, (list, tuple)) else []
    positions = cmd.get("position") or []
    if isinstance(positions, (str, bytes)) or not isinstance(positions, (list, tuple)):
        positions = []
    mapped: dict[str, float] = {}
    if names:
        for index, name in enumerate(names):
            number = _at(positions, index)
            if isinstance(name, str) and number is not None:
                mapped[name] = number
        return mapped
    for index, name in enumerate(limits):
        number = _at(positions, index)
        if number is not None:
            mapped[name] = number
    return mapped


def _imu_block(sample: Optional[Mapping[str, Any]], now: float):
    if not sample or not isinstance(sample.get("data"), Mapping):
        return None, None
    data = sample["data"]
    keys = ("roll", "pitch", "yaw", "ax", "ay", "az")
    nums = [_finite(data.get(key)) for key in keys]
    if any(num is None for num in nums):
        return None, None
    return dict(zip(keys, nums)), _age(now, sample["t"])


def _gripper_block(sample: Optional[Mapping[str, Any]], now: float):
    if not sample or not isinstance(sample.get("data"), Mapping):
        return None, None
    pos = _finite(sample["data"].get("pos"))
    cmd = _finite(sample["data"].get("cmd"))
    if pos is None or cmd is None or pos < 0.0 or cmd < 0.0:
        return None, None
    return {"pos": pos, "cmd": cmd}, _age(now, sample["t"])


def _battery_block(
    sources: Mapping[str, Any],
    now: float,
    *,
    sim: bool,
    sim_battery_v: Optional[float],
    sim_battery_pct: Optional[float],
    empty_v: float,
    full_v: float,
):
    chosen: Optional[tuple[float, dict[str, Any]]] = None
    for key in ("battery", "voltage"):
        sample = _sample(sources, key)
        if sample is None or not isinstance(sample.get("data"), Mapping):
            continue
        block = _coerce_battery(
            sample["data"], sim=sim, empty_v=empty_v, full_v=full_v
        )
        if block is None:
            continue
        stamp = _finite(sample.get("t"))
        if stamp is None:
            continue
        if chosen is None or stamp >= chosen[0]:
            chosen = (stamp, block)
    if chosen is not None:
        return chosen[1], _age(now, chosen[0])
    if not sim or sim_battery_v is None or sim_battery_pct is None:
        return None, None
    synthetic = _coerce_battery(
        {"v": sim_battery_v, "pct": sim_battery_pct},
        sim=True,
        empty_v=empty_v,
        full_v=full_v,
    )
    if synthetic is None:
        return None, None
    return synthetic, 0.0


def _coerce_battery(
    data: Mapping[str, Any], *, sim: bool, empty_v: float, full_v: float
) -> Optional[dict[str, Any]]:
    volts = _finite(data.get("v"))
    if volts is None or volts < 0.0:
        return None
    if data.get("pct") is None:
        pct = percent_from_voltage(volts, empty_v, full_v)
    else:
        pct = _finite(data.get("pct"))
    if pct is None or pct < 0.0 or pct > 100.0:
        return None
    return {"v": volts, "pct": pct, "simulated": sim}


def _nav_block(sources: Mapping[str, Any], now: float):
    status = _sample(sources, "nav_status")
    feedback = _sample(sources, "nav_feedback")
    if status is None or feedback is None or not isinstance(feedback.get("data"), Mapping):
        return None, None
    phase = nav_phase(status.get("data"))
    goal = feedback["data"].get("goal")
    eta = _finite(feedback["data"].get("eta_s"))
    if phase is None or eta is None or eta < 0.0 or not isinstance(goal, Mapping):
        return None, None
    gx = _finite(goal.get("x"))
    gy = _finite(goal.get("y"))
    gyaw = _finite(goal.get("yaw"))
    if gx is None or gy is None or gyaw is None:
        return None, None
    block = {
        "state": phase,
        "goal": {"x": gx, "y": gy, "yaw": gyaw},
        "eta": eta,
    }
    return block, max(_age(now, status["t"]), _age(now, feedback["t"]))


def _policy_block(sample: Optional[Mapping[str, Any]], now: float):
    if not sample or not isinstance(sample.get("data"), Mapping):
        return None, None
    data = sample["data"]
    name = data.get("name")
    state = data.get("state")
    latency = _finite(data.get("latency_ms"))
    if (
        not isinstance(name, str)
        or not _DISPLAY_NAME.match(name)
        or len(name) > 128
        or state not in _POLICY_STATES
        or latency is None
        or latency < 0.0
    ):
        return None, None
    return {"name": name, "state": state, "latency_ms": latency}, _age(now, sample["t"])


def _rec_block(sample: Optional[Mapping[str, Any]], now: float):
    if not sample:
        return None, None
    data = sample.get("data")
    if isinstance(data, str):
        parsed = parse_recorder_status(data)
    elif isinstance(data, Mapping):
        parsed = _recording_dict(data)
    else:
        parsed = None
    if parsed is None:
        return None, None
    return parsed, _age(now, sample["t"])


def _recording_dict(data: Mapping[str, Any]) -> Optional[dict[str, Any]]:
    active = data.get("active")
    episode = data.get("episode")
    frames = data.get("frames")
    if not isinstance(active, bool):
        return None
    if not isinstance(episode, str) or not _DISPLAY_NAME.match(episode) or len(episode) > 128:
        return None
    if isinstance(frames, bool) or not isinstance(frames, int) or frames < 0:
        return None
    return {"active": active, "episode": episode, "frames": frames}


def _sys_block(sample: Optional[Mapping[str, Any]], now: float):
    if not sample or not isinstance(sample.get("data"), Mapping):
        return None, None
    cpu = _finite(sample["data"].get("cpu"))
    temp = _finite(sample["data"].get("temp"))
    if cpu is None or temp is None or cpu < 0.0 or cpu > 100.0:
        return None, None
    return {"cpu": cpu, "temp": temp}, _age(now, sample["t"])


def _normalize_estop(raw: Mapping[str, Any]) -> dict[str, Any]:
    active = raw.get("active") is True
    latched = raw.get("latched") is True
    source = raw.get("source")
    if not isinstance(source, str) or source not in _ESTOP_SOURCES:
        source = "software" if (active or latched) else "none"
    if (active or latched) and source == "none":
        source = "software"
    if not active and not latched:
        source = "none"
    return {"active": active, "latched": latched, "source": source}


def _velocity(data: Mapping[str, Any]) -> Optional[dict[str, float]]:
    vx = _finite(data.get("vx"))
    vy = _finite(data.get("vy"))
    wz = _finite(data.get("wz"))
    if vx is None or vy is None or wz is None:
        return None
    return {"vx": vx, "vy": vy, "wz": wz}


def _sample(sources: Mapping[str, Any], key: str) -> Optional[Mapping[str, Any]]:
    sample = sources.get(key)
    if not isinstance(sample, Mapping) or "t" not in sample:
        return None
    return sample


def _at(values: Sequence[Any], index: int) -> Optional[float]:
    if index < 0 or index >= len(values):
        return None
    return _finite(values[index])


def _age(now: float, stamp: Any) -> float:
    value = _finite(stamp)
    if value is None:
        return 0.0
    delta_ms = (now - value) * 1000.0
    if delta_ms < 0.0:
        delta_ms = 0.0
    return round(delta_ms, 3)


def _nonneg(value: Any) -> float:
    number = _finite(value)
    if number is None or number < 0.0:
        return 0.0
    return number


def _nonneg_number(value: Any) -> Any:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if not math.isfinite(float(value)) or value < 0:
        return None
    return value


def _finite(value: Any) -> Optional[float]:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
    elif isinstance(value, str):
        try:
            number = float(value)
        except ValueError:
            return None
    else:
        return None
    if not math.isfinite(number):
        return None
    return number
