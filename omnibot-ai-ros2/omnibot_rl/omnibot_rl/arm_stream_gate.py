"""Pure arm-command gate. No ROS imports.

``arm_cmd_mux`` is a thin wrapper. ``/arm/joint_commands/teleop`` (Quest VR
hand IK and Android joint teleop) is forwarded only while ``/control_mode``
is teleop and ``/emergency_stop`` is clear. Policy (``/arm/joint_commands/policy``,
plus the deprecated alias ``/arm/joint_commands``) and RL arm commands stay
idle until ``/control_mode`` is vla, nav2, or rl_nav. ``/emergency_stop``
blocks every arm source on the same call. Base teleop is a separate topic
and is not decided here.
"""

DEFAULT_CONTROL_MODE = "teleop"
VALID_CONTROL_MODES = ("nav2", "vla", "teleop", "rl_nav")
AUTONOMOUS_CONTROL_MODES = ("nav2", "vla", "rl_nav")
# Selectable with /arm/cmd_mode. Teleop is selected by /control_mode only.
ARM_SOURCES = ("policy", "rl_arm")
TELEOP_SOURCE = "teleop"
TOPIC_JOINT_COMMANDS_TELEOP = "/arm/joint_commands/teleop"
TOPIC_JOINT_COMMANDS_POLICY = "/arm/joint_commands/policy"
# Deprecated alias for TOPIC_JOINT_COMMANDS_POLICY. Same gate. One release.
TOPIC_JOINT_COMMANDS_POLICY_ALIAS = "/arm/joint_commands"
TOPIC_JOINT_COMMANDS_RL = "/arm/joint_commands/rl"
TOPIC_JOINT_COMMANDS_OUT = "/arm/joint_commands/out"
_VALID_CONTROL = frozenset(VALID_CONTROL_MODES)
_ARM_SOURCES = frozenset(ARM_SOURCES)


def normalize_control_mode(raw):
    """Return a known ``/control_mode`` value, or None."""
    if not isinstance(raw, str):
        return None
    mode = raw.strip().lower()
    if mode not in _VALID_CONTROL:
        return None
    return mode


def normalize_arm_mode(raw):
    """Return policy or rl_arm, or None."""
    if not isinstance(raw, str):
        return None
    mode = raw.strip().lower()
    if mode not in _ARM_SOURCES:
        return None
    return mode


class ArmStreamGate:
    """Decides which arm source may be forwarded."""

    VALID_CONTROL_MODES = VALID_CONTROL_MODES

    def __init__(self, arm_mode="policy", control_mode=DEFAULT_CONTROL_MODE):
        self.arm_mode = (
            arm_mode if arm_mode in _ARM_SOURCES else "policy"
        )
        normalized = normalize_control_mode(control_mode)
        self.control_mode = (
            normalized if normalized is not None else DEFAULT_CONTROL_MODE
        )
        self.emergency_stop = False

    @classmethod
    def from_default_mode(cls, default_mode):
        """Start in teleop.

        ``default_mode`` selects the arm source used once a human sets an
        autonomous control mode. ``teleop`` (and any unknown value) keeps
        the policy source as that later preference. The teleop joint source
        is forwarded immediately; policy and RL stay held.
        """
        arm_mode = normalize_arm_mode(default_mode)
        if arm_mode is None:
            arm_mode = "policy"
        return cls(arm_mode=arm_mode, control_mode=DEFAULT_CONTROL_MODE)

    def set_control_mode(self, raw):
        """Apply ``/control_mode``. Unknown values are ignored.

        A change to vla or nav2 selects the policy arm source. A change to
        rl_nav selects the RL arm source. Repeating the same control mode
        does not clobber a later ``/arm/cmd_mode`` choice.
        """
        mode = normalize_control_mode(raw)
        if mode is None:
            return False
        changed = mode != self.control_mode
        self.control_mode = mode
        if changed and mode == "rl_nav":
            self.arm_mode = "rl_arm"
        elif changed and mode in ("vla", "nav2"):
            self.arm_mode = "policy"
        return True

    def set_arm_mode(self, raw):
        """Apply ``/arm/cmd_mode``. Does not release the teleop hold by itself."""
        mode = normalize_arm_mode(raw)
        if mode is None:
            return False
        self.arm_mode = mode
        return True

    def set_emergency_stop(self, active):
        self.emergency_stop = bool(active)

    def allows(self, source):
        """True when ``source`` may be forwarded on this cycle.

        ``teleop`` is allowed only in teleop mode. Policy and ``rl_arm`` are
        allowed only in an autonomous control mode, and only when they are
        the selected arm source. Emergency stop blocks every source.
        """
        if self.emergency_stop:
            return False
        if self.control_mode not in _VALID_CONTROL:
            return False
        if source == TELEOP_SOURCE:
            return self.control_mode == "teleop"
        if source not in _ARM_SOURCES:
            return False
        if self.control_mode not in AUTONOMOUS_CONTROL_MODES:
            return False
        return source == self.arm_mode
