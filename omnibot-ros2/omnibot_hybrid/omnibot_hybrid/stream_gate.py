"""Pure base-velocity gate. No ROS imports.

``cmd_vel_mux`` is a thin wrapper. Autonomous sources (nav2, vla, rl_nav)
are not forwarded while the mode is teleop. The teleop source is forwarded
in teleop mode. Switching between any two different modes marks one zero
Twist to publish in that same call. Raising emergency stop blocks every
source and marks one zero Twist to publish in that same call.

After a source has been forwarded, ``poll_staleness`` reports once when that
active source has been silent for ``source_timeout_s``. The wrapper publishes
one zero Twist for that report. The timeout defaults to 0.3 s. A caller may
shorten it. A longer value is clamped to 0.3 s. Zero, negative, and
non-numeric values stay at 0.3 s so the watchdog cannot be turned off.
"""

import math

DEFAULT_MODE = "teleop"
VALID_MODES = ("nav2", "vla", "teleop", "rl_nav")
AUTONOMOUS_MODES = ("nav2", "vla", "rl_nav")
_VALID = frozenset(VALID_MODES)

# Ceiling is the default on purpose. A longer timeout leaves a dropped
# source's last twist on the base for longer.
DEFAULT_SOURCE_TIMEOUT_S = 0.3


def normalize_mode(raw):
    """Return a known mode, or None when ``raw`` is not one."""
    if not isinstance(raw, str):
        return None
    mode = raw.strip().lower()
    if mode not in _VALID:
        return None
    return mode


def resolve_source_timeout_s(raw):
    """Return ``(seconds, warning)``.

    ``warning`` is None when ``raw`` is already a finite number in
    ``(0, DEFAULT_SOURCE_TIMEOUT_S]``. Every other value resolves to the
    default so the watchdog stays armed, and ``warning`` explains that.
    """
    default = DEFAULT_SOURCE_TIMEOUT_S
    if (
        isinstance(raw, bool)
        or not isinstance(raw, (int, float))
        or not math.isfinite(raw)
    ):
        return default, (
            f'Invalid source_timeout_s "{raw}" — using {default:.3f}s. '
            "The staleness timeout cannot be disabled."
        )
    value = float(raw)
    if value <= 0.0:
        return default, (
            f"source_timeout_s {value:.3f}s is not positive — using {default:.3f}s. "
            "The staleness timeout cannot be disabled."
        )
    if value > default:
        return default, (
            f"source_timeout_s {value:.3f}s exceeds {default:.3f}s — "
            f"clamped to {default:.3f}s."
        )
    return value, None


class StreamGate:
    """Decides which base source may be forwarded."""

    def __init__(self, mode=DEFAULT_MODE, source_timeout_s=DEFAULT_SOURCE_TIMEOUT_S):
        normalized = normalize_mode(mode)
        self.mode = normalized if normalized is not None else DEFAULT_MODE
        self.emergency_stop = False
        self._base_stop = False
        self.source_timeout_s, self.source_timeout_warning = resolve_source_timeout_s(
            source_timeout_s
        )
        self._last_forward_s = {}
        self._stale_zeroed = set()

    def set_mode(self, raw):
        """Apply a ``/control_mode`` string. Unknown values are ignored."""
        mode = normalize_mode(raw)
        if mode is None:
            return False
        previous = self.mode
        self.mode = mode
        # Switching between any two different modes must zero the latched base
        # command in this same call (one cycle), not on a later timer.
        if mode != previous:
            self._base_stop = True
        return True

    def set_emergency_stop(self, active):
        """Latch or clear ``/emergency_stop``. A rising edge zeros the base."""
        active = bool(active)
        if active and not self.emergency_stop:
            self._base_stop = True
        self.emergency_stop = active

    def take_base_stop(self):
        """True once when this call must publish a zero Twist, then clears."""
        stop = self._base_stop
        self._base_stop = False
        return stop

    def base_allows(self, source):
        """True when ``source`` may be forwarded on this cycle."""
        if self.emergency_stop:
            return False
        return source == self.mode

    def note_forward(self, source, now):
        """Record a command forwarded from the active ``source`` at ``now``."""
        if source != self.mode:
            return
        self._last_forward_s[source] = float(now)
        self._stale_zeroed.discard(source)

    def poll_staleness(self, now):
        """True once when the active source has been silent for the timeout.

        False before the first forwarded command, while the silence is still
        inside the window, and on later polls in the same silence. A new
        ``note_forward`` for that source arms the next silence.
        """
        source = self.mode
        last = self._last_forward_s.get(source)
        if last is None or source in self._stale_zeroed:
            return False
        if (float(now) - last) < self.source_timeout_s:
            return False
        self._stale_zeroed.add(source)
        return True
