# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Pure helpers for the ros2_control topic adapter. No rclpy."""

import re

ARM_JOINTS = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
)
GRIPPER_JOINT = "arm_gripper"
ALL_ARM_JOINTS = ARM_JOINTS + (GRIPPER_JOINT,)

CHANGE_EPSILON = 1e-4
TRAJECTORY_SEC = 0.2
TWIST_TIMEOUT_SEC = 0.25

MUX_BRIDGE_NAMES = ("sim_bridge_mux.yaml", "sim_bridge_vr.yaml")


_ANSI = re.compile(r"\x1b\[[0-9;]*m")
_CONTROLLER_STATES = {"unconfigured", "inactive", "active", "finalized", "configured"}


def controller_listing_active(text, names):
    """True when each controller's state token is exactly ``active``.

    ``ros2 control list_controllers`` prints either ``name[type] state`` or
    ``name type state``, sometimes with ANSI color. ``inactive`` contains
    the substring ``active``, so a line search is wrong.
    """
    found = {}
    for line in _ANSI.sub("", text).splitlines():
        stripped = line.strip()
        parts = stripped.split()
        if len(parts) < 2 or parts[-1] not in _CONTROLLER_STATES:
            continue
        name = stripped.split("[", 1)[0].strip() if "[" in stripped else parts[0]
        found[name] = parts[-1]
    return all(found.get(name) == "active" for name in names)


def cmd_vel_topic_for_bridge(bridge_name):
    """ROS twist the adapter should follow for a ros_gz_bridge file name."""
    name = bridge_name.replace("\\", "/").split("/")[-1]
    if name in MUX_BRIDGE_NAMES:
        return "/cmd_vel/out"
    return "/cmd_vel"


def _close(left, right, epsilon=CHANGE_EPSILON):
    return all(abs(a - b) <= epsilon for a, b in zip(left, right))


def arm_subset(names, positions, velocities=None):
    """Six arm joints in URDF order, or None when any name is missing.

    Wheel joints are dropped. Missing arm joints are not filled with zero.
    """
    index = {name: i for i, name in enumerate(names)}
    if any(name not in index for name in ALL_ARM_JOINTS):
        return None
    ordered = [float(positions[index[name]]) for name in ALL_ARM_JOINTS]
    velocity = None
    if velocities is not None and len(velocities) == len(names):
        velocity = [float(velocities[index[name]]) for name in ALL_ARM_JOINTS]
    return list(ALL_ARM_JOINTS), ordered, velocity


class CmdPosTracker:
    """Arm trajectories and gripper goals from per-joint cmd_pos samples.

    Nothing is emitted until every arm joint has been seen once. A repeat of
    the same command does not emit again, so a 100 Hz hold does not restart
    the trajectory controller.
    """

    def __init__(self):
        self.values = {}
        self._last_arm = None
        self._last_gripper = None

    def update(self, joint, value):
        self.values[joint] = float(value)

    def arm_ready(self):
        return all(name in self.values for name in ARM_JOINTS)

    def trajectory_if_changed(self):
        if not self.arm_ready():
            return None
        current = tuple(self.values[name] for name in ARM_JOINTS)
        if self._last_arm is not None and _close(current, self._last_arm):
            return None
        self._last_arm = current
        return current

    def gripper_if_changed(self):
        if GRIPPER_JOINT not in self.values:
            return None
        value = self.values[GRIPPER_JOINT]
        if self._last_gripper is not None and abs(value - self._last_gripper) <= CHANGE_EPSILON:
            return None
        self._last_gripper = value
        return value


class TwistRelay:
    """Latest Twist, or zeros once it is older than the timeout."""

    def __init__(self, timeout_sec=TWIST_TIMEOUT_SEC):
        self.timeout_sec = float(timeout_sec)
        self._stamp = None
        self._sample = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def update(self, linear, angular, now):
        self._sample = (
            float(linear[0]),
            float(linear[1]),
            float(linear[2]),
            float(angular[0]),
            float(angular[1]),
            float(angular[2]),
        )
        self._stamp = float(now)

    def force_zero(self, now):
        self._sample = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._stamp = float(now)

    def stopped(self):
        """True when the latest sample is a full stop.

        The adapter publishes that sample in the twist callback. Waiting for
        the refresh timer leaves the mecanum controller holding the last
        wheel speed.
        """
        return all(abs(value) <= 1e-6 for value in self._sample)

    def sample(self, now):
        if self._stamp is None or (float(now) - self._stamp) > self.timeout_sec:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        return self._sample
