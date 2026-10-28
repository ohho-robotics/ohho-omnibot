"""Arm torque and joint-command safety. No ROS imports.

arm_driver_node is a thin wrapper around ArmSafety. Position servos only:
there is no force control.

OHH-100 (policy node clamps) must reuse MAX_JOINT_DELTA_RAD — the same
0.15 rad per cycle limit. Do not rename that constant.
"""

from arm_math import clamp_radians

# Maximum joint-target change per accepted command, in radians.
# OHH-100 imports this name. Keep the value and the spelling stable.
MAX_JOINT_DELTA_RAD = 0.15

# Hold the last /arm/joint_commands/out target after this much silence.
COMMAND_TIMEOUT_SEC = 0.2


def open_hardware_bus(simulate, lerobot_available):
    """True only when this process should open the Feetech bus.

    A simulation launch passes simulate=True. Lerobot being installed on
    the same machine must not override that and touch a real arm.
    """
    if simulate:
        return False
    return bool(lerobot_available)


class TorqueAction:
    """Bus write for Torque_Enable. value is 0, 1, or None (do not write)."""

    def __init__(self, value, message, level="info"):
        self.value = value
        self.message = message
        self.level = level


class CommandAction:
    """One accepted joint target. write is False when torque is off."""

    def __init__(self, write, positions):
        self.write = write
        self.positions = positions


class HoldAction:
    """Timeout hold. rewrite is True only on the transition into hold."""

    def __init__(self, rewrite, positions, message):
        self.rewrite = rewrite
        self.positions = positions
        self.message = message


def clamp_joint_command(commanded, joint_min, joint_max, reference, max_delta):
    """Clamp to joint limits, then to max_delta from reference.

    reference None skips the per-cycle delta limit (limits only).
    When reference is set, the result steps at most max_delta radians
    toward the limit-clamped target. A reference already inside the
    limits stays inside them.
    """
    limited = clamp_radians(commanded, joint_min, joint_max)
    if reference is None:
        return limited
    step = abs(float(max_delta))
    stepped = []
    for target, ref in zip(limited, reference):
        delta = target - ref
        if delta > step:
            delta = step
        elif delta < -step:
            delta = -step
        stepped.append(ref + delta)
    return stepped


class ArmSafety:
    """E-stop torque latch, command timeout hold, and joint clamps.

    Torque starts enabled so an arm with no /arm/enable publisher still
    accepts commands, matching the previous driver. /emergency_stop true
    forces Torque_Enable 0. Clearing the stop does not turn torque back
    on. A later /arm/enable true, received while the stop is false, does.
    An enable received while the stop is still true is ignored and is not
    remembered.

    This object does not command the base. yahboom_controller_node already
    zeros base velocity while /emergency_stop is true.
    """

    def __init__(
        self,
        joint_min,
        joint_max,
        max_joint_delta_rad=MAX_JOINT_DELTA_RAD,
        command_timeout_sec=COMMAND_TIMEOUT_SEC,
    ):
        self.joint_min = list(joint_min)
        self.joint_max = list(joint_max)
        self.max_joint_delta_rad = float(max_joint_delta_rad)
        self.command_timeout_sec = float(command_timeout_sec)
        self.emergency_stop = False
        self.torque_enabled = True
        self.last_command = None
        self.last_command_time = None
        self._holding = False

    def drop_target(self):
        """Drop the stored target and timestamp so nothing is held while limp."""
        self.last_command = None
        self.last_command_time = None
        self._holding = False

    def seed_target(self, positions, now):
        """Re-seed target and clamp reference from measured joint positions."""
        clamped = clamp_radians(positions, self.joint_min, self.joint_max)
        self.last_command = list(clamped)
        self.last_command_time = float(now) if now is not None else None
        self._holding = False
        return self.last_command

    def on_emergency_stop(self, active):
        """Disable torque while active. Clearing the stop leaves torque off."""
        active = bool(active)
        if active:
            rising = not self.emergency_stop
            self.emergency_stop = True
            self.torque_enabled = False
            self.drop_target()
            message = None
            if rising:
                message = (
                    "EMERGENCY STOP — disabling follower torque (Torque_Enable 0)."
                )
            return TorqueAction(0, message, "warn")
        if self.emergency_stop:
            self.emergency_stop = False
            self.torque_enabled = False
            self.drop_target()
            return TorqueAction(
                0,
                "Emergency stop cleared — follower torque stays off until "
                "/arm/enable is true.",
                "info",
            )
        return TorqueAction(None, None, "info")

    def on_arm_enable(self, enable):
        """Honor /arm/enable only while the emergency stop is clear."""
        enable = bool(enable)
        if self.emergency_stop:
            self.torque_enabled = False
            self.drop_target()
            if enable:
                return TorqueAction(
                    0,
                    "Ignoring /arm/enable while emergency stop is active — "
                    "torque stays off.",
                    "warn",
                )
            return TorqueAction(0, "Arm torque disabled", "info")
        if enable:
            self.torque_enabled = True
            return TorqueAction(1, "Arm torque enabled", "info")
        self.torque_enabled = False
        self.drop_target()
        return TorqueAction(0, "Arm torque disabled", "info")

    def accept_command(self, commanded, now, reference):
        """Clamp one target. Rejected while the stop is active or torque is off.

        Rejected commands do not replace last_command, so a stream that
        arrives during the stop cannot jump the arm when torque returns.
        """
        if self.emergency_stop or not self.torque_enabled:
            return CommandAction(False, None)
        baseline = self.last_command if self.last_command is not None else reference
        positions = clamp_joint_command(
            commanded,
            self.joint_min,
            self.joint_max,
            baseline,
            self.max_joint_delta_rad,
        )
        self.last_command = list(positions)
        self.last_command_time = now
        self._holding = False
        return CommandAction(True, list(positions))

    def hold_if_stale(self, now):
        """Hold the last accepted target after command_timeout_sec of silence.

        message is set only on the transition into hold, so the driver logs
        once per silence. No hold before the first command, or while torque
        is off.
        """
        if (
            self.last_command is None
            or self.last_command_time is None
            or self.emergency_stop
            or not self.torque_enabled
        ):
            return HoldAction(False, None, None)
        silent = now - self.last_command_time
        if silent < self.command_timeout_sec:
            return HoldAction(False, None, None)
        message = None
        rewrite = False
        if not self._holding:
            self._holding = True
            rewrite = True
            message = (
                "Joint command timeout: /arm/joint_commands/out silent for "
                f"{silent:.3f}s (limit {self.command_timeout_sec:.3f}s) — "
                "holding last command"
            )
        return HoldAction(rewrite, list(self.last_command), message)
