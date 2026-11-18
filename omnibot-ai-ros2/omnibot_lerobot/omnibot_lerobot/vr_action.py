"""VR action and state assembly for the teleop recorder.

Pure Python: no rclpy. The recorder subscribes to teleop topics and calls
these functions. It does not publish joint commands or base twists.

Arm action is the last joint target on ``/arm/joint_commands/teleop``
(OHH-197). Point that parameter at the post-safety stream when one is
published. If that topic is silent, the mux output
``/arm/joint_commands/out`` is used only while ``/control_mode`` is
``teleop``, so a policy command on the mux is not stored as the action.

Base action is the last twist on ``/cmd_vel/teleop``, with the same
teleop-only mux fallback (``/cmd_vel/out``).

State is the follower arm (``/arm/joint_states``) plus base odometry.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

ACTION_SOURCE_LEADER = "leader"
ACTION_SOURCE_VR = "vr"
ACTION_SOURCES = (ACTION_SOURCE_LEADER, ACTION_SOURCE_VR)

PREFER_TELEOP = "teleop"
PREFER_MUX = "mux"
VR_PREFERS = (PREFER_TELEOP, PREFER_MUX)

ROBOT_KIND_SIM = "sim"
ROBOT_KIND_REAL = "real"
ROBOT_KINDS = (ROBOT_KIND_SIM, ROBOT_KIND_REAL)

# Same six names the leader-arm recorder already writes.
JOINT_NAMES = [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
]
BASE_NAMES = ["base_vx", "base_vy", "base_vz"]
VECTOR_NAMES = JOINT_NAMES + BASE_NAMES
ARM_DIM = len(JOINT_NAMES)
BASE_DIM = len(BASE_NAMES)
VECTOR_DIM = ARM_DIM + BASE_DIM

# OHH-197 arm mux, and the base mux output used only in teleop mode.
DEFAULT_ARM_COMMAND_TOPIC = "/arm/joint_commands/teleop"
DEFAULT_ARM_MUX_TOPIC = "/arm/joint_commands/out"
DEFAULT_BASE_COMMAND_TOPIC = "/cmd_vel/teleop"
DEFAULT_BASE_MUX_TOPIC = "/cmd_vel/out"
DEFAULT_CONTROL_MODE_TOPIC = "/control_mode"
CONTROL_MODES = ("teleop", "vla", "nav2", "rl_nav")

SOURCE_TELEOP = "teleop"
SOURCE_MUX = "mux"
SOURCE_NONE = "none"


@dataclass(frozen=True)
class VrSample:
    """One recorded VR frame. ``action`` is the command; ``state`` is the robot."""

    action: np.ndarray
    state: np.ndarray
    arm_source: str
    base_source: str


def normalize_action_source(raw: object) -> str:
    """Return ``leader`` or ``vr``. Anything else is an error."""
    if not isinstance(raw, str):
        raise ValueError("action_source must be 'leader' or 'vr'")
    value = raw.strip().lower()
    if value not in ACTION_SOURCES:
        raise ValueError("action_source must be 'leader' or 'vr'")
    return value


def normalize_prefer(raw: object) -> tuple[str, bool]:
    """Return ``(teleop|mux, recognized)``. Unrecognized values are teleop."""
    if not isinstance(raw, str):
        return PREFER_TELEOP, False
    value = raw.strip().lower()
    if value in VR_PREFERS:
        return value, True
    return PREFER_TELEOP, False


def normalize_robot_kind(raw: object) -> tuple[str, bool]:
    """Return ``(sim|real, recognized)``. Unrecognized values are sim."""
    if not isinstance(raw, str):
        return ROBOT_KIND_SIM, False
    value = raw.strip().lower()
    if value in ROBOT_KINDS:
        return value, True
    return ROBOT_KIND_SIM, False


def normalize_control_mode(raw: object) -> str | None:
    """Return a mux control mode, or None when the value should be ignored."""
    if not isinstance(raw, str):
        return None
    mode = raw.strip().lower()
    if mode not in CONTROL_MODES:
        return None
    return mode


def _finite_vector(values: object, width: int) -> np.ndarray | None:
    if values is None:
        return None
    try:
        array = np.asarray(values, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError):
        return None
    if array.size == 0 or not np.all(np.isfinite(array)):
        return None
    if array.size < width:
        array = np.pad(array, (0, width - array.size))
    return array[:width].astype(np.float32)


def positions_for_joints(
    names: object,
    positions: object,
    joint_names: tuple[str, ...] | list[str] = JOINT_NAMES,
) -> np.ndarray | None:
    """Map a joint message onto ``joint_names``.

    Empty names with a full position vector are taken in ``joint_names`` order.
    Short names (``shoulder_pan``) match the ``arm_`` prefix used in datasets.
    """
    if positions is None:
        return None
    try:
        values = [float(item) for item in list(positions)]
    except (TypeError, ValueError):
        return None
    if not values or not all(np.isfinite(values)):
        return None
    ordered = list(joint_names)
    label_list: list[str] = []
    if names is not None:
        try:
            label_list = [str(item) for item in list(names)]
        except TypeError:
            label_list = []
    if not label_list:
        return _finite_vector(values[: len(ordered)], len(ordered))
    mapping: dict[str, float] = {}
    for label, value in zip(label_list, values):
        mapping[label] = value
    resolved = []
    for joint in ordered:
        if joint in mapping:
            resolved.append(mapping[joint])
        elif joint.startswith("arm_") and joint[4:] in mapping:
            resolved.append(mapping[joint[4:]])
        else:
            resolved.append(0.0)
    return _finite_vector(resolved, len(ordered))


def twist_vector(linear_x: object, linear_y: object, angular_z: object) -> np.ndarray | None:
    """Return ``[vx, vy, wz]`` or None when a component is not a finite number."""
    try:
        values = [float(linear_x), float(linear_y), float(angular_z)]
    except (TypeError, ValueError):
        return None
    return _finite_vector(values, BASE_DIM)


def select_stream(
    teleop: np.ndarray | None,
    mux: np.ndarray | None,
    control_mode: str,
    prefer: str,
) -> tuple[np.ndarray | None, str]:
    """Pick the teleop sample, or the mux sample only in teleop mode.

    ``prefer='teleop'`` uses the dedicated topic whenever it has a sample.
    ``prefer='mux'`` uses the mux output when control mode is teleop, which
    is the forwarded command after the mux gate. A non-teleop mode never
    selects the mux, so policy and navigation commands are not recorded.
    """
    teleop_sample = teleop
    mux_allowed = mux is not None and control_mode == "teleop"
    if prefer == PREFER_MUX:
        if mux_allowed:
            return mux, SOURCE_MUX
        if teleop_sample is not None:
            return teleop_sample, SOURCE_TELEOP
        return None, SOURCE_NONE
    if teleop_sample is not None:
        return teleop_sample, SOURCE_TELEOP
    if mux_allowed:
        return mux, SOURCE_MUX
    return None, SOURCE_NONE


def assemble_vr_sample(
    arm_teleop: np.ndarray | None,
    arm_mux: np.ndarray | None,
    base_teleop: np.ndarray | None,
    base_mux: np.ndarray | None,
    control_mode: str,
    follower: np.ndarray | None,
    odom_twist: np.ndarray | None,
    prefer: str = PREFER_TELEOP,
) -> VrSample:
    """Build the 9-D action (command) and 9-D state (follower + odom)."""
    mode = control_mode if control_mode in CONTROL_MODES else "teleop"
    choice = prefer if prefer in VR_PREFERS else PREFER_TELEOP
    arm, arm_source = select_stream(arm_teleop, arm_mux, mode, choice)
    base, base_source = select_stream(base_teleop, base_mux, mode, choice)
    if arm is None:
        arm = np.zeros(ARM_DIM, dtype=np.float32)
        arm_source = SOURCE_NONE
    if base is None:
        base = np.zeros(BASE_DIM, dtype=np.float32)
        base_source = SOURCE_NONE
    follower_arm = follower if follower is not None else np.zeros(ARM_DIM, dtype=np.float32)
    odom = odom_twist if odom_twist is not None else np.zeros(BASE_DIM, dtype=np.float32)
    action = np.concatenate([np.asarray(arm, dtype=np.float32), np.asarray(base, dtype=np.float32)])
    state = np.concatenate(
        [np.asarray(follower_arm, dtype=np.float32), np.asarray(odom, dtype=np.float32)]
    )
    return VrSample(
        action=action.astype(np.float32),
        state=state.astype(np.float32),
        arm_source=arm_source,
        base_source=base_source,
    )


def episode_metadata(
    task: object,
    operator: object,
    robot_kind: object,
    sim_world: object,
) -> dict[str, str]:
    """Episode fields stored beside the LeRobot v2 tables.

    ``operator`` defaults to ``vr``. ``robot_kind`` is ``sim`` or ``real``.
    ``sim_world`` is the simulator world name (empty when unused).
    """
    kind, _recognized = normalize_robot_kind(robot_kind)
    if isinstance(operator, str) and operator.strip():
        who = operator.strip()
    else:
        who = "vr"
    world = sim_world.strip() if isinstance(sim_world, str) else ""
    if isinstance(task, str) and task.strip():
        task_name = task.strip()
    else:
        task_name = "mobile manipulation task"
    return {
        "operator": who,
        "robot_kind": kind,
        "sim_world": world,
        "task": task_name,
    }
