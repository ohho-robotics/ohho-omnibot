"""Gazebo arm command path. No ROS imports.

arm_sim_driver_node is a thin wrapper around ArmSimCommander. Limits, the
per-command delta, the command timeout, and the e-stop latch all come from
arm_safety.ArmSafety. Do not reimplement those rules here.
"""

import os

import yaml

from arm_safety import COMMAND_TIMEOUT_SEC
from arm_safety import MAX_JOINT_DELTA_RAD
from arm_safety import ArmSafety

# Gazebo JointPositionController subscribes to each command topic.
# JointStatePublisher publishes measured arm joints on FEEDBACK_TOPIC.
SIM_PREFIX = "/arm/sim"
FEEDBACK_TOPIC = "/arm/sim/joint_states"

# Inside arm_params.yaml joint limits and inside the tighter SO-101 URDF
# stops, so a scripted publisher can reach every value in Gazebo.
# Order matches arm_driver_node joint_names.
SCRIPTED_TARGETS = (0.45, -0.45, 0.45, -0.45, 0.60, 0.45)
POSITION_TOLERANCE_RAD = 0.08
HOLD_TOLERANCE_RAD = 0.08

# Used only when arm_params.yaml cannot be read. test_arm_sim.py fails if
# these drift from that file.
FALLBACK_JOINT_NAMES = [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
]
FALLBACK_JOINT_MIN = [-3.14, -1.57, -1.57, -1.57, -3.14, -0.1]
FALLBACK_JOINT_MAX = [3.14, 1.57, 1.57, 1.57, 3.14, 0.8]


def command_topic(joint_name):
    """ROS and gz topic for one joint position command."""
    return f"{SIM_PREFIX}/{joint_name}/cmd_pos"


def bundled_arm_params_path():
    """Source-tree arm_params.yaml next to this package's config/."""
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "arm_params.yaml")
    )


def fallback_arm_params():
    """Same numbers as arm_params.yaml. Not a second, looser limit set."""
    return {
        "joint_names": list(FALLBACK_JOINT_NAMES),
        "joint_min": list(FALLBACK_JOINT_MIN),
        "joint_max": list(FALLBACK_JOINT_MAX),
        "max_joint_delta_rad": MAX_JOINT_DELTA_RAD,
        "command_timeout_sec": COMMAND_TIMEOUT_SEC,
        "publish_rate": 100.0,
    }


def load_arm_driver_params(path):
    """Read the arm_driver_node parameter block from arm_params.yaml."""
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    params = data["arm_driver_node"]["ros__parameters"]
    required = (
        "joint_names",
        "joint_min",
        "joint_max",
        "max_joint_delta_rad",
        "command_timeout_sec",
        "publish_rate",
    )
    missing = [key for key in required if key not in params]
    if missing:
        raise KeyError(f"{path} missing {missing}")
    return params


def positions_within(actual, desired, tolerance):
    """True when every joint is within tolerance radians of desired."""
    if len(actual) != len(desired):
        return False
    return all(abs(a - b) <= tolerance for a, b in zip(actual, desired))


class ArmSimCommander:
    """Safety-filtered position targets for the Gazebo arm.

    on_command returns the positions to publish, or None when ArmSafety
    rejects the command (e-stop latched, torque off). Measured feedback is
    what /arm/joint_states echoes. An e-stop returns the measured pose so
    the position controllers hold it instead of tracking a new target.
    """

    def __init__(
        self,
        joint_names,
        joint_min,
        joint_max,
        max_joint_delta_rad=MAX_JOINT_DELTA_RAD,
        command_timeout_sec=COMMAND_TIMEOUT_SEC,
    ):
        self.joint_names = list(joint_names)
        self.safety = ArmSafety(
            joint_min,
            joint_max,
            max_joint_delta_rad=max_joint_delta_rad,
            command_timeout_sec=command_timeout_sec,
        )
        self.measured = [0.0] * len(self.joint_names)
        self.have_feedback = False
        self.last_written = None

    @classmethod
    def from_driver_params(cls, params):
        return cls(
            joint_names=list(params["joint_names"]),
            joint_min=[float(value) for value in params["joint_min"]],
            joint_max=[float(value) for value in params["joint_max"]],
            max_joint_delta_rad=float(params["max_joint_delta_rad"]),
            command_timeout_sec=float(params["command_timeout_sec"]),
        )

    def on_feedback(self, names, positions):
        """Store a full arm sample. Incomplete messages are ignored."""
        name_to_pos = {
            name: float(position) for name, position in zip(names, positions)
        }
        if not all(name in name_to_pos for name in self.joint_names):
            return False
        self.measured = [name_to_pos[name] for name in self.joint_names]
        self.have_feedback = True
        return True

    def on_command(self, names, positions, now):
        """Clamp one /arm/joint_commands/out sample. None means do not write."""
        name_to_pos = {
            name: float(position) for name, position in zip(names, positions)
        }
        # Missing names become 0, matching arm_driver_node.joint_command_cb.
        commanded = [float(name_to_pos.get(name, 0.0)) for name in self.joint_names]
        action = self.safety.accept_command(commanded, now, list(self.measured))
        if not action.write:
            return None
        self.last_written = list(action.positions)
        return list(action.positions)

    def on_emergency_stop(self, active):
        """Latch e-stop. When active, also return the pose to freeze in sim."""
        action = self.safety.on_emergency_stop(bool(active))
        if not active:
            return action, None
        hold = list(self.measured)
        self.last_written = list(hold)
        return action, hold

    def on_arm_enable(self, enable, now):
        """Honor /arm/enable only while the stop is clear. May reseed."""
        action = self.safety.on_arm_enable(bool(enable))
        if action.value != 1:
            return action, None
        seeded = self.safety.seed_target(self.measured, now)
        self.last_written = list(seeded)
        return action, list(seeded)

    def hold_if_stale(self, now):
        """Delegate the timeout hold. rewrite is True once per silence."""
        return self.safety.hold_if_stale(now)


def run_scripted_sequence(commander, step_dt=0.05, max_steps_per_joint=40):
    """Step each joint to SCRIPTED_TARGETS with ideal tracking.

    Ideal tracking copies each accepted command into feedback. Gazebo is
    the real plant; this helper is the unit-test plant. Returns
    (reached, measured, target).
    """
    names = list(commander.joint_names)
    target = [0.0] * len(names)
    now = 0.0
    for index, goal in enumerate(SCRIPTED_TARGETS):
        target[index] = float(goal)
        reached = False
        for _ in range(max_steps_per_joint):
            written = commander.on_command(names, list(target), now)
            if written is not None:
                commander.on_feedback(names, written)
            now += step_dt
            if positions_within(commander.measured, target, POSITION_TOLERANCE_RAD):
                reached = True
                break
        if not reached:
            return False, list(commander.measured), list(target)
    return True, list(commander.measured), list(target)
