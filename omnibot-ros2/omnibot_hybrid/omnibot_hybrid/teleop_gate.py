"""SafetyGate rules for human teleop. No ROS imports.

``teleop_safety`` is a thin rclpy wrapper. Clients publish
``/cmd_vel/teleop_raw`` and ``/arm/joint_commands/teleop_raw``. This gate
publishes the clamped results on the topics the muxes already consume.

Ceilings match ``ohho.safety.OMNIBOT_PROFILE`` and ``CMD_TIMEOUT_S``
(OHH-117). Linear 0.2 m/s and angular 1.0 rad/s cannot be raised by a
parameter, topic, or service. Acceleration ceilings match that profile
(0.5 m/s^2, 0.5 m/s^2, 2.0 rad/s^2). Joint steps reuse
``policy_guard.clamp_joint_deltas`` and never exceed
``arm_safety.MAX_JOINT_DELTA_RAD`` (0.15 rad). A longer watchdog than
300 ms is rejected.

Command loss and ``/ohho/heartbeat`` loss zero base velocity. They do not
latch. ``/emergency_stop`` true latches until ``reset()``, which only the
``/ohho/estop_reset`` service calls. Publishing false does not clear it.
Joint targets are never zeroed; a refused arm command is dropped.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Optional, Sequence, Union

from omnibot_lerobot.policy_guard import (
    MAX_JOINT_DELTA_RAD,
    clamp_joint_deltas,
)

# ohho.safety.OMNIBOT_PROFILE / CMD_TIMEOUT_S. Do not raise these.
HARD_MAX_VX = 0.2  # m/s
HARD_MAX_VY = 0.2  # m/s
HARD_MAX_WZ = 1.0  # rad/s
HARD_MAX_ACCEL_X = 0.5  # m/s^2
HARD_MAX_ACCEL_Y = 0.5  # m/s^2
HARD_MAX_ACCEL_WZ = 2.0  # rad/s^2
CMD_TIMEOUT_S = 0.3
HEARTBEAT_TIMEOUT_S = CMD_TIMEOUT_S
# arm_safety.MAX_JOINT_DELTA_RAD. Stricter than SafetyProfile.max_joint_step.
HARD_MAX_JOINT_DELTA_RAD = 0.15
AUDIT_DIR_ENV = "OHHO_SAFETY_DIR"

# Same pairs as ohho.safety._OMNIBOT_JOINT_LIMITS and arm_params.yaml.
JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "arm_shoulder_pan": (-3.14, 3.14),
    "arm_shoulder_lift": (-1.57, 1.57),
    "arm_elbow_flex": (-1.57, 1.57),
    "arm_wrist_flex": (-1.57, 1.57),
    "arm_wrist_roll": (-3.14, 3.14),
    "arm_gripper": (-0.1, 0.8),
}

_FIXED_PARAMETERS = frozenset(
    {
        "max_vx",
        "max_vy",
        "max_wz",
        "max_accel_x",
        "max_accel_y",
        "max_accel_wz",
        "cmd_timeout_s",
        "heartbeat_timeout_s",
        "max_joint_delta_rad",
        "enable",
        "enabled",
        "bypass",
        "disable_estop",
        "disable_watchdog",
    }
)
_ZERO = (0.0, 0.0, 0.0)


_POLICY_JOINT_DELTA_RAD = float(MAX_JOINT_DELTA_RAD)
# Never step farther than the arm driver, even if the import drifts upward.
MAX_JOINT_DELTA_RAD = min(_POLICY_JOINT_DELTA_RAD, HARD_MAX_JOINT_DELTA_RAD)


def parameter_change_allowed(name: str, value: Any) -> bool:
    """False when a ROS parameter would loosen or retarget this gate."""
    if name in _FIXED_PARAMETERS:
        return False
    if name == "use_sim_time":
        return value is False or value == 0
    return True


def default_audit_dir(environ: Optional[Mapping[str, str]] = None) -> Path:
    """``$OHHO_SAFETY_DIR`` or ``~/.ohho/safety``, matching SafetyGate."""
    env = os.environ if environ is None else environ
    custom = env.get(AUDIT_DIR_ENV)
    return Path(custom) if custom else Path.home() / ".ohho" / "safety"


def _jsonable(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if value is None or isinstance(value, (str, int, bool)):
        return value
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    return str(value)


class AuditLog:
    """Append-only JSON Lines audit, one file per UTC day."""

    def __init__(
        self,
        directory: Optional[Union[str, Path]] = None,
        *,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        self.directory = (
            Path(directory) if directory is not None else default_audit_dir()
        )
        self._wall = wall_clock
        self._lock = threading.Lock()

    def path_for(self, stamp: float) -> Path:
        day = datetime.fromtimestamp(stamp, timezone.utc).strftime("%Y-%m-%d")
        return self.directory / f"{day}.jsonl"

    def write(self, event: str, **fields: Any) -> bool:
        """Append one record. Returns False (never raises) if it cannot."""
        stamp = self._wall()
        record: dict[str, Any] = {
            "ts": datetime.fromtimestamp(stamp, timezone.utc).isoformat(
                timespec="milliseconds"
            ),
            "t": stamp,
            "event": event,
        }
        record.update({key: _jsonable(value) for key, value in fields.items()})
        line = json.dumps(record, sort_keys=True, allow_nan=False) + "\n"
        try:
            with self._lock:
                self.directory.mkdir(parents=True, exist_ok=True)
                with self.path_for(stamp).open("a", encoding="utf-8") as handle:
                    handle.write(line)
        except (OSError, ValueError, TypeError):
            return False
        return True


class Outcome:
    """One gate step. ``twist`` and ``joints`` are published only when set."""

    def __init__(
        self,
        twist: Optional[tuple] = None,
        joints: Optional[list] = None,
        log: Optional[str] = None,
        ok: bool = True,
        message: str = "",
    ) -> None:
        self.twist = twist
        self.joints = joints
        self.log = log
        self.ok = ok
        self.message = message


def _ceiling(value: Any, ceiling: float) -> float:
    """A finite cap in ``[0, ceiling]``. Invalid values stop that axis."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return 0.0
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        return 0.0
    if number > ceiling:
        return ceiling
    return number


def _timeout(value: Any, ceiling: float) -> float:
    """A timeout in ``(0, ceiling]``. Anything else stays at ``ceiling``."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return ceiling
    number = float(value)
    if not math.isfinite(number) or number <= 0.0 or number > ceiling:
        return ceiling
    return number


def _clamp(value: float, lo: float, hi: float) -> float:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def _accel_step(prev: float, target: float, dmax: float) -> float:
    """Braking toward zero is free; speeding up or reversing moves at most dmax.

    Ported from ``ohho.safety._accel_step``.
    """
    base = prev if prev * target > 0.0 else 0.0
    if abs(target) <= abs(base):
        return target
    return base + math.copysign(min(abs(target) - abs(base), dmax), target)


def _finite_number(value: Any) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    if not math.isfinite(number):
        return None
    return number


class TeleopGate:
    """Caps, acceleration, joint steps, watchdogs, and a latched e-stop."""

    def __init__(
        self,
        *,
        clock: Callable[[], float] = time.monotonic,
        audit_dir: Optional[Union[str, Path]] = None,
        wall_clock: Callable[[], float] = time.time,
        max_vx: float = HARD_MAX_VX,
        max_vy: float = HARD_MAX_VY,
        max_wz: float = HARD_MAX_WZ,
        max_accel_x: float = HARD_MAX_ACCEL_X,
        max_accel_y: float = HARD_MAX_ACCEL_Y,
        max_accel_wz: float = HARD_MAX_ACCEL_WZ,
        cmd_timeout_s: float = CMD_TIMEOUT_S,
        heartbeat_timeout_s: float = HEARTBEAT_TIMEOUT_S,
        max_joint_delta_rad: float = MAX_JOINT_DELTA_RAD,
    ) -> None:
        self._clock = clock
        self.max_vx = _ceiling(max_vx, HARD_MAX_VX)
        self.max_vy = _ceiling(max_vy, HARD_MAX_VY)
        self.max_wz = _ceiling(max_wz, HARD_MAX_WZ)
        self.max_accel_x = _ceiling(max_accel_x, HARD_MAX_ACCEL_X)
        self.max_accel_y = _ceiling(max_accel_y, HARD_MAX_ACCEL_Y)
        self.max_accel_wz = _ceiling(max_accel_wz, HARD_MAX_ACCEL_WZ)
        self.cmd_timeout_s = _timeout(cmd_timeout_s, CMD_TIMEOUT_S)
        self.heartbeat_timeout_s = _timeout(heartbeat_timeout_s, HEARTBEAT_TIMEOUT_S)
        self.max_joint_delta_rad = _ceiling(max_joint_delta_rad, MAX_JOINT_DELTA_RAD)
        self._audit_log = AuditLog(audit_dir, wall_clock=wall_clock)
        self._lock = threading.RLock()
        self._latched = False
        self._reason = ""
        self._estop_asserted = False
        self._audit_blocked = False
        self._block_zero_pending = False
        self._last_audit_signature = None
        self._last_heartbeat: Optional[float] = None
        self._heartbeat_tripped = False
        self._last_cmd_t = float(self._clock())
        self._last_sent = _ZERO
        self._moving = False
        self._measured: dict[str, float] = {}
        self._joint_cmd: dict[str, float] = {}
        self._armed_flag = False
        self._audit("connect")

    @property
    def armed(self) -> bool:
        return self._armed_flag

    @property
    def latched(self) -> bool:
        return self._latched

    @property
    def reason(self) -> str:
        return self._reason

    def state_json(self, now: Optional[float] = None) -> str:
        """HUD payload: armed, latched, and the last trip reason."""
        with self._lock:
            self._sync(self._now(now))
            payload = {
                "armed": self._armed_flag,
                "latched": self._latched,
                "reason": self._reason,
            }
        return json.dumps(payload, sort_keys=True)

    def heartbeat(self, now: Optional[float] = None) -> Outcome:
        """A ``/ohho/heartbeat`` message. Motion stays disarmed without one."""
        with self._lock:
            stamp = self._now(now)
            self._last_heartbeat = stamp
            self._heartbeat_tripped = False
            self._sync(stamp)
        return Outcome()

    def set_emergency_stop(self, active: bool, now: Optional[float] = None) -> Outcome:
        """Latch on true. False does not clear the latch."""
        with self._lock:
            stamp = self._now(now)
            active = bool(active)
            self._estop_asserted = active
            if not active or self._latched:
                self._sync(stamp)
                return Outcome()
            self._latched = True
            self._reason = "estop"
            self._stop_motion()
            self._sync(stamp)
            self._audit("estop", reason="estop")
            return Outcome(
                twist=_ZERO,
                log="e-stop latched; reset only via /ohho/estop_reset",
            )

    def reset(self, now: Optional[float] = None) -> Outcome:
        """Clear a latched e-stop. Refused while ``/emergency_stop`` is true."""
        with self._lock:
            stamp = self._now(now)
            if self._estop_asserted:
                self._audit("release_refused", reason="estop_asserted")
                self._sync(stamp)
                return Outcome(
                    ok=False,
                    message="emergency_stop is still asserted",
                )
            if not self._latched:
                self._sync(stamp)
                return Outcome(ok=True, message="not latched")
            wrote = self._audit("estop_release")
            if not wrote:
                self._sync(stamp)
                return Outcome(ok=False, message="audit log unavailable")
            self._latched = False
            self._sync(stamp)
            return Outcome(ok=True, message="e-stop reset")

    def command_velocity(
        self,
        vx: Any,
        vy: Any,
        wz: Any,
        now: Optional[float] = None,
    ) -> Outcome:
        """Clamp one base command. Always returns a twist to publish."""
        with self._lock:
            stamp = self._now(now)
            self._sync(stamp)
            parsed = (
                _finite_number(vx),
                _finite_number(vy),
                _finite_number(wz),
            )
            if any(item is None for item in parsed):
                self._stop_motion()
                self._audit(
                    "reject",
                    command="velocity",
                    reason="non_finite",
                    requested=[vx, vy, wz],
                )
                return Outcome(twist=_ZERO)
            req = parsed  # type: tuple[float, float, float]
            if req == _ZERO:
                self._last_cmd_t = stamp
                self._stop_motion()
                return Outcome(twist=_ZERO)
            if self._latched or self._audit_blocked:
                self._stop_motion()
                self._audit(
                    "reject",
                    command="velocity",
                    reason="estop_latched" if self._latched else "audit_failure",
                    requested=list(req),
                )
                return Outcome(twist=_ZERO)
            if not self._heartbeat_fresh(stamp):
                first = not self._heartbeat_tripped
                self._trip_heartbeat(stamp, command="velocity")
                return Outcome(
                    twist=_ZERO,
                    log="heartbeat missing; velocity zeroed" if first else None,
                )
            capped = (
                _clamp(req[0], -self.max_vx, self.max_vx),
                _clamp(req[1], -self.max_vy, self.max_vy),
                _clamp(req[2], -self.max_wz, self.max_wz),
            )
            dt = min(max(stamp - self._last_cmd_t, 0.0), self.cmd_timeout_s)
            accels = (self.max_accel_x, self.max_accel_y, self.max_accel_wz)
            applied = tuple(
                _accel_step(prev, target, accel * dt)
                for prev, target, accel in zip(self._last_sent, capped, accels)
            )
            reasons = []
            if capped != req:
                reasons.append("velocity_cap")
            if applied != capped:
                reasons.append("accel_cap")
            if reasons and not self._audit(
                "clamp",
                command="velocity",
                requested=list(req),
                applied=list(applied),
                reasons=reasons,
            ):
                return Outcome(twist=_ZERO, log="audit log failed; velocity zeroed")
            self._last_sent = applied
            self._last_cmd_t = stamp
            self._moving = applied != _ZERO
            self._sync(stamp)
            return Outcome(twist=applied)

    def note_joint_state(self, names: Sequence[Any], positions: Sequence[Any]) -> None:
        """Remember measured joint positions for the per-step reference."""
        with self._lock:
            if len(names) != len(positions):
                return
            for name, position in zip(names, positions):
                number = _finite_number(position)
                if isinstance(name, str) and number is not None:
                    self._measured[name] = number

    def command_joints(
        self,
        names: Sequence[Any],
        positions: Sequence[Any],
        now: Optional[float] = None,
    ) -> Outcome:
        """Limit joints, then step them with ``clamp_joint_deltas``.

        Returns no joints when the command is dropped. Positions are not zeroed.
        """
        with self._lock:
            stamp = self._now(now)
            self._sync(stamp)
            if len(names) != len(positions) or not names:
                self._audit("reject", command="joints", reason="bad_joints")
                return Outcome()
            if self._latched or self._audit_blocked:
                self._audit(
                    "reject",
                    command="joints",
                    reason="estop_latched" if self._latched else "audit_failure",
                )
                return Outcome()
            if not self._heartbeat_fresh(stamp):
                first = not self._heartbeat_tripped
                self._trip_heartbeat(stamp, command="joints")
                return Outcome(
                    log="heartbeat missing; arm command dropped" if first else None
                )
            limited: list[float] = []
            reference: list[float] = []
            cleaned: list[str] = []
            requested: list[float] = []
            limit_hit = False
            for name, position in zip(names, positions):
                number = _finite_number(position)
                if not isinstance(name, str) or number is None:
                    self._audit("reject", command="joints", reason="non_finite")
                    return Outcome()
                bounds = JOINT_LIMITS.get(name)
                if bounds is None:
                    self._audit(
                        "reject",
                        command="joints",
                        reason="unknown_joint",
                        joint=name,
                    )
                    return Outcome()
                ref = self._joint_reference(name)
                if ref is None:
                    self._audit(
                        "reject",
                        command="joints",
                        reason="no_joint_reference",
                        joint=name,
                    )
                    return Outcome()
                bounded = _clamp(number, bounds[0], bounds[1])
                if bounded != number:
                    limit_hit = True
                cleaned.append(name)
                requested.append(number)
                limited.append(bounded)
                reference.append(ref)
            applied = clamp_joint_deltas(
                limited, reference, self.max_joint_delta_rad
            )
            if len(applied) != len(limited):
                self._audit("reject", command="joints", reason="bad_joints")
                return Outcome()
            applied_list = [float(item) for item in applied]
            reasons = []
            if limit_hit:
                reasons.append("joint_limit")
            if applied_list != limited:
                reasons.append("joint_step")
            if reasons and not self._audit(
                "clamp",
                command="joints",
                requested=requested,
                applied=applied_list,
                reasons=reasons,
            ):
                return Outcome(log="audit log failed; arm command dropped")
            for name, position in zip(cleaned, applied_list):
                self._joint_cmd[name] = position
            self._sync(stamp)
            return Outcome(joints=list(zip(cleaned, applied_list)))

    def tick(self, now: Optional[float] = None) -> Outcome:
        """Watchdogs. A trip zeros the base once and does not latch."""
        with self._lock:
            stamp = self._now(now)
            self._sync(stamp)
            if self._latched or self._audit_blocked:
                if self._block_zero_pending:
                    self._block_zero_pending = False
                    return Outcome(twist=_ZERO, log="audit log failed; velocity zeroed")
                return Outcome()
            if not self._heartbeat_fresh(stamp):
                if self._heartbeat_tripped:
                    return Outcome()
                self._trip_heartbeat(stamp, command="tick")
                return Outcome(
                    twist=_ZERO,
                    log="heartbeat watchdog: zeroing teleop velocity",
                )
            idle = stamp - self._last_cmd_t
            if self._moving and idle >= self.cmd_timeout_s:
                self._reason = "command_watchdog"
                self._stop_motion()
                self._sync(stamp)
                self._audit(
                    "watchdog_trip",
                    kind="command",
                    idle_s=idle,
                )
                return Outcome(
                    twist=_ZERO,
                    log="command watchdog: zeroing teleop velocity",
                )
            return Outcome()

    def _now(self, now: Optional[float]) -> float:
        if now is None:
            return float(self._clock())
        return float(now)

    def _heartbeat_fresh(self, now: float) -> bool:
        last = self._last_heartbeat
        if last is None:
            return False
        return (now - last) < self.heartbeat_timeout_s

    def _sync(self, now: float) -> None:
        self._armed_flag = (
            not self._latched
            and not self._audit_blocked
            and self._heartbeat_fresh(now)
        )

    def _stop_motion(self) -> None:
        self._last_sent = _ZERO
        self._moving = False

    def _trip_heartbeat(self, now: float, *, command: str) -> None:
        already = self._heartbeat_tripped
        self._heartbeat_tripped = True
        self._reason = "heartbeat_watchdog"
        self._stop_motion()
        self._sync(now)
        if already:
            return
        self._audit(
            "watchdog_trip",
            kind="heartbeat",
            command=command,
        )

    def _joint_reference(self, name: str) -> Optional[float]:
        if name in self._measured:
            return self._measured[name]
        return self._joint_cmd.get(name)

    def _audit_signature(self, event: str, fields: Mapping[str, Any]) -> tuple:
        """Identity of a repeated clamp or reject, ignoring sample values."""
        command = fields.get("command")
        if "reasons" in fields:
            raw = fields["reasons"]
            reasons = tuple(raw) if isinstance(raw, (list, tuple)) else (raw,)
        elif "reason" in fields:
            reasons = (fields["reason"],)
        else:
            reasons = ()
        return (event, command, reasons)

    def _audit(self, event: str, **fields: Any) -> bool:
        signature = self._audit_signature(event, fields)
        if signature == self._last_audit_signature and not self._audit_blocked:
            return True
        ok = self._audit_log.write(
            event, robot="omnibot", protocol="ros2", **fields
        )
        if ok:
            self._last_audit_signature = signature
            self._audit_blocked = False
            self._block_zero_pending = False
            return True
        self._audit_blocked = True
        self._block_zero_pending = True
        self._reason = "audit_failure"
        self._stop_motion()
        self._armed_flag = False
        return False
