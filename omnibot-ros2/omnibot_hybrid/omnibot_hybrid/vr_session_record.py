"""VR session MCAP recording: topic list, rotation, and size cap.

No ROS imports. The node in ``vr_session_recorder`` is a read-only subscriber
plus a ``ros2 bag record`` child. It does not publish base or arm commands
and it does not change ``teleop_safety``.

A session starts on ``/ohho/vr_session/start`` or, when auto session is on,
on the first ``/ohho/heartbeat``. It stops on ``/ohho/vr_session/stop``, on
shutdown, or when an auto session loses heartbeats for ``session_idle_sec``.
That idle timer is observability only. It is not the 300 ms deadman.

Each session directory holds ``part-NNNN`` bags. A part is rotated onto a new
directory when it reaches ``max_bag_size_bytes``. Oldest parts under the
output directory are deleted once the sum of part trees exceeds
``max_total_bytes``. The active part is kept. ``max_bag_size_bytes`` must be
less than or equal to ``max_total_bytes``.
"""

from __future__ import annotations

import json
import math
import os
import re
import shutil
import signal
import subprocess
import threading
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable, Optional, Sequence

DEFAULT_OUTPUT_DIR = "~/.ohho/vr_sessions"
DEFAULT_MAX_BAG_SIZE_BYTES = 64 * 1024 * 1024
DEFAULT_MAX_TOTAL_BYTES = 512 * 1024 * 1024
DEFAULT_SESSION_IDLE_SEC = 2.0
DEFAULT_HEARTBEAT_GAP_SEC = 0.5
DEFAULT_METRICS_PORT = 9188

# Issue OHH-239. Shell globs: a trailing * is the only wildcard.
TOPIC_GLOBS: tuple[str, ...] = (
    "/cmd_vel/teleop*",
    "/arm/joint_commands/teleop*",
    "/ohho/heartbeat",
    "/ohho/telemetry",
    "/ohho/safety/state",
    "/control_mode*",
)

TRIP_REASONS = frozenset({"estop", "heartbeat_watchdog", "command_watchdog"})

ZERO_TWIST = (
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
)
EMPTY_JOINTS = "{name: [], position: [], velocity: [], effort: []}"
_STRING_IDLE = "{data: idle}"

# Local proof publishes these and nothing with a non-zero command.
PROOF_TOPICS: tuple[tuple[str, str, str], ...] = (
    ("/cmd_vel/teleop", "geometry_msgs/msg/Twist", ZERO_TWIST),
    ("/cmd_vel/teleop_raw", "geometry_msgs/msg/Twist", ZERO_TWIST),
    ("/arm/joint_commands/teleop", "sensor_msgs/msg/JointState", EMPTY_JOINTS),
    (
        "/arm/joint_commands/teleop_raw",
        "sensor_msgs/msg/JointState",
        EMPTY_JOINTS,
    ),
    ("/ohho/heartbeat", "std_msgs/msg/Empty", "{}"),
    ("/ohho/telemetry", "std_msgs/msg/String", _STRING_IDLE),
    ("/ohho/safety/state", "std_msgs/msg/String", _STRING_IDLE),
    ("/control_mode", "std_msgs/msg/String", _STRING_IDLE),
    ("/control_mode/active", "std_msgs/msg/String", _STRING_IDLE),
)
EXCLUDED_PROOF_TOPIC = (
    "/ohho/heartbeat_echo",
    "std_msgs/msg/String",
    "{data: excluded}",
)

_PART_NAME = re.compile(r"^part-\d+$")
_SESSION_FILES = frozenset({"session.json", "record.log"})


def _glob_to_regex(pattern: str) -> str:
    """Turn one OHH-239 glob into a fully anchored regex body."""
    if pattern.count("*") > 1 or ("*" in pattern and not pattern.endswith("*")):
        raise ValueError(f"topic glob {pattern!r} must be literal or end with *")
    if pattern.endswith("*"):
        return re.escape(pattern[:-1]) + ".*"
    return re.escape(pattern)


def topic_regex() -> str:
    """One regex for ``ros2 bag record -e``. Anchored so search equals full match."""
    parts = [_glob_to_regex(pattern) for pattern in TOPIC_GLOBS]
    return "^(?:" + "|".join(parts) + ")$"


def topic_selected(name: str) -> bool:
    """True when ``name`` is one of the VR session topics."""
    return re.fullmatch(topic_regex(), name) is not None


def record_argv(output_path: str, *, use_sim_time: bool = False) -> list[str]:
    """``ros2 bag record`` argv for one MCAP part. No ``-a`` and no extra topics."""
    cmd = [
        "ros2",
        "bag",
        "record",
        "-s",
        "mcap",
        "-o",
        output_path,
        "--include-unpublished-topics",
        "-e",
        topic_regex(),
    ]
    if use_sim_time:
        cmd.append("--use-sim-time")
    return cmd


def sanitize_session_id(text: str) -> str:
    cleaned = "".join(ch for ch in text.strip() if ch.isalnum() or ch in "-_")
    return cleaned[:48]


def session_directory_name(session_id: str, wall: datetime) -> str:
    stamp = wall.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    suffix = sanitize_session_id(session_id)
    if suffix:
        return f"{stamp}-{suffix}"
    return stamp


def _as_utc(wall: Optional[datetime]) -> datetime:
    if wall is None:
        return datetime.now(timezone.utc)
    if wall.tzinfo is None:
        return wall.replace(tzinfo=timezone.utc)
    return wall.astimezone(timezone.utc)


def directory_size(path: Path) -> int:
    """Bytes in ``path``. Missing paths are 0. Directory entries themselves are not counted."""
    if not path.exists():
        return 0
    if path.is_file():
        return path.stat().st_size
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += (Path(root) / name).stat().st_size
            except OSError:
                continue
    return total


def _part_index(path: Path) -> int:
    return int(path.name.split("-", 1)[1])


def iter_parts(output_dir: Path) -> list[Path]:
    """Part directories, oldest session and lowest index first."""
    if not output_dir.is_dir():
        return []
    found: list[Path] = []
    for session in output_dir.iterdir():
        if not session.is_dir():
            continue
        for part in session.iterdir():
            if part.is_dir() and _PART_NAME.match(part.name):
                found.append(part)
    found.sort(key=lambda path: (path.parent.name, _part_index(path)))
    return found


def _same_path(left: Optional[Path], right: Path) -> bool:
    if left is None:
        return False
    return os.path.normcase(str(left.resolve())) == os.path.normcase(str(right.resolve()))


def _contained(root: Path, child: Path) -> bool:
    root_r = root.resolve()
    child_r = child.resolve()
    return root_r == child_r or root_r in child_r.parents


def _drop_session(session: Path) -> None:
    """Remove a session dir that has no parts left and only our bookkeeping files."""
    if not session.is_dir():
        return
    for child in list(session.iterdir()):
        if child.is_dir():
            return
        if not (child.is_file() and child.name in _SESSION_FILES):
            return
    for child in list(session.iterdir()):
        child.unlink()
    session.rmdir()


def prune_to_cap(
    output_dir: Path,
    max_total_bytes: int,
    active: Optional[Path],
    *,
    size_of: Callable[[Path], int] = directory_size,
    keep_session: Optional[Path] = None,
) -> list[Path]:
    """Delete oldest part directories until the sum of part sizes fits.

    ``active`` is never deleted. ``keep_session`` is not removed even when its
    parts are gone, so a rotate can open the next part in the same session.
    """
    if max_total_bytes <= 0:
        raise ValueError("max_total_bytes must be positive")
    parts = iter_parts(output_dir)
    deleted: list[Path] = []

    def total() -> int:
        return sum(size_of(path) for path in parts)

    while parts and total() > max_total_bytes:
        victims = [path for path in parts if not _same_path(active, path)]
        if not victims:
            break
        oldest = victims[0]
        if not _contained(output_dir, oldest):
            break
        try:
            shutil.rmtree(oldest)
        except OSError:
            break
        parts.remove(oldest)
        deleted.append(oldest)
        session = oldest.parent
        if keep_session is not None and _same_path(session, keep_session):
            continue
        _drop_session(session)
    return deleted


def parse_rtt_ms(text: str) -> Optional[float]:
    """``link.rtt_ms`` from an ohho.telemetry.v1 sample, or None."""
    try:
        document = json.loads(text)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(document, dict):
        return None
    link = document.get("link")
    if not isinstance(link, dict):
        return None
    value = link.get("rtt_ms")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        return None
    return number


def parse_safety_state(text: str) -> Optional[dict]:
    try:
        document = json.loads(text)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(document, dict):
        return None
    if "latched" not in document and "estop" not in document and "reason" not in document:
        return None
    return document


def _is_latched(document: dict) -> bool:
    if document.get("latched") is True:
        return True
    estop = document.get("estop")
    return isinstance(estop, dict) and estop.get("latched") is True


def _reason(document: dict) -> str:
    reason = document.get("reason")
    if isinstance(reason, str):
        return reason.strip().lower()
    return ""


def safety_trip(previous: Optional[dict], current: dict) -> bool:
    """True on a new e-stop latch or a new trip reason.

    Repeating the same ``/ohho/safety/state`` payload is not another trip.
    This does not latch, clear, or command the gate.
    """
    latched = _is_latched(current)
    reason = _reason(current)
    prev_latched = False
    prev_reason = ""
    if isinstance(previous, dict):
        prev_latched = _is_latched(previous)
        prev_reason = _reason(previous)
    if latched and not prev_latched:
        return True
    if reason in TRIP_REASONS and reason != prev_reason:
        return True
    return False


class HeartbeatTracker:
    """Gap gauge and a counter of silences that cross ``gap_event_sec``."""

    def __init__(self, gap_event_sec: float):
        if gap_event_sec <= 0:
            raise ValueError("gap_event_sec must be positive")
        self.gap_event_sec = gap_event_sec
        self.last: Optional[float] = None
        self.gap_sec = 0.0
        self.gaps_total = 0
        self._open = False

    def beat(self, now: float) -> None:
        self.last = now
        self.gap_sec = 0.0
        self._open = False

    def tick(self, now: float) -> None:
        if self.last is None:
            self.gap_sec = 0.0
            return
        self.gap_sec = max(0.0, now - self.last)
        if self.gap_sec >= self.gap_event_sec and not self._open:
            self.gaps_total += 1
            self._open = True


class SafetyTripCounter:
    def __init__(self) -> None:
        self.total = 0
        self.previous: Optional[dict] = None

    def observe(self, text: str) -> bool:
        current = parse_safety_state(text)
        if current is None:
            return False
        tripped = safety_trip(self.previous, current)
        self.previous = current
        if tripped:
            self.total += 1
        return tripped


def render_prometheus(
    *,
    rtt_ms: Optional[float],
    heartbeat_gap_sec: float,
    heartbeat_gaps_total: int,
    safety_trips_total: int,
) -> str:
    """Prometheus text for the VR session Grafana panels."""
    lines: list[str] = []
    if rtt_ms is not None:
        lines.append("# HELP ohho_vr_rtt_ms Last link.rtt_ms from /ohho/telemetry.")
        lines.append("# TYPE ohho_vr_rtt_ms gauge")
        lines.append(f"ohho_vr_rtt_ms {rtt_ms:.6f}")
    lines.append(
        "# HELP ohho_vr_heartbeat_gap_seconds Seconds since the last /ohho/heartbeat."
    )
    lines.append("# TYPE ohho_vr_heartbeat_gap_seconds gauge")
    lines.append(f"ohho_vr_heartbeat_gap_seconds {max(0.0, heartbeat_gap_sec):.6f}")
    lines.append(
        "# HELP ohho_vr_heartbeat_gaps_total "
        "Heartbeat silences that crossed the gap threshold."
    )
    lines.append("# TYPE ohho_vr_heartbeat_gaps_total counter")
    lines.append(f"ohho_vr_heartbeat_gaps_total {int(heartbeat_gaps_total)}")
    lines.append(
        "# HELP ohho_vr_safety_trips_total "
        "New e-stop latches and new safety trip reasons observed on /ohho/safety/state."
    )
    lines.append("# TYPE ohho_vr_safety_trips_total counter")
    lines.append(f"ohho_vr_safety_trips_total {int(safety_trips_total)}")
    return "\n".join(lines) + "\n"


def topics_in_bag_info(text: str) -> set[str]:
    """Topic names from ``ros2 bag info`` text."""
    found: set[str] = set()
    for line in text.splitlines():
        if "Topic:" not in line:
            continue
        part = line.split("Topic:", 1)[1].split("|", 1)[0].strip()
        if part.startswith("/"):
            found.add(part)
    return found


class RecorderConfig:
    """Validated recorder settings. The output path is expanded."""

    def __init__(
        self,
        output_dir: Path | str,
        max_bag_size_bytes: int = DEFAULT_MAX_BAG_SIZE_BYTES,
        max_total_bytes: int = DEFAULT_MAX_TOTAL_BYTES,
        auto_session: bool = True,
        session_idle_sec: float = DEFAULT_SESSION_IDLE_SEC,
        heartbeat_gap_sec: float = DEFAULT_HEARTBEAT_GAP_SEC,
        use_sim_time: bool = False,
        metrics_port: int = DEFAULT_METRICS_PORT,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.max_bag_size_bytes = int(max_bag_size_bytes)
        self.max_total_bytes = int(max_total_bytes)
        self.auto_session = bool(auto_session)
        self.session_idle_sec = float(session_idle_sec)
        self.heartbeat_gap_sec = float(heartbeat_gap_sec)
        self.use_sim_time = bool(use_sim_time)
        self.metrics_port = int(metrics_port)

    def validated(self) -> "RecorderConfig":
        if self.max_bag_size_bytes <= 0 or self.max_total_bytes <= 0:
            raise ValueError("bag size and total size must be positive")
        if self.max_bag_size_bytes > self.max_total_bytes:
            raise ValueError("max_bag_size_bytes cannot exceed max_total_bytes")
        if self.session_idle_sec <= 0 or self.heartbeat_gap_sec <= 0:
            raise ValueError("session_idle_sec and heartbeat_gap_sec must be positive")
        if self.metrics_port < 0 or self.metrics_port > 65535:
            raise ValueError("metrics_port out of range")
        config = RecorderConfig(
            output_dir=Path(self.output_dir).expanduser(),
            max_bag_size_bytes=self.max_bag_size_bytes,
            max_total_bytes=self.max_total_bytes,
            auto_session=self.auto_session,
            session_idle_sec=self.session_idle_sec,
            heartbeat_gap_sec=self.heartbeat_gap_sec,
            use_sim_time=self.use_sim_time,
            metrics_port=self.metrics_port,
        )
        return config


class SubprocessBagRunner:
    """One ``ros2 bag record`` process. SIGINT on POSIX so the MCAP is flushed."""

    def __init__(self) -> None:
        self._proc: Optional[subprocess.Popen] = None
        self._log = None

    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self, cmd: Sequence[str], log_path: Optional[Path] = None) -> None:
        if self.running():
            raise RuntimeError("bag record already running")
        self._close_log()
        stderr = subprocess.DEVNULL
        if log_path is not None:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log = open(log_path, "ab")
            stderr = self._log
        kwargs = {
            "stdout": subprocess.DEVNULL,
            "stderr": stderr,
        }
        if os.name == "posix":
            kwargs["start_new_session"] = True
        else:
            flags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
            kwargs["creationflags"] = flags
        self._proc = subprocess.Popen(list(cmd), **kwargs)

    def stop(self, timeout: float = 10.0) -> None:
        proc = self._proc
        if proc is None or proc.poll() is not None:
            self._proc = None
            self._close_log()
            return
        try:
            if os.name == "posix":
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            else:
                proc.terminate()
            proc.wait(timeout=timeout)
        except Exception:
            proc.kill()
            try:
                proc.wait(timeout=5)
            except Exception:
                pass
        finally:
            self._proc = None
            self._close_log()

    def _close_log(self) -> None:
        if self._log is not None:
            try:
                self._log.close()
            except OSError:
                pass
            self._log = None


class MetricsEndpoint:
    """Loopback Prometheus text. ``port`` 0 asks the OS for a free port."""

    def __init__(self, render: Callable[[], str], port: int) -> None:
        self._render = render
        endpoint = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802 - http.server API
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                if path not in ("/metrics",):
                    self.send_response(404)
                    self.end_headers()
                    return
                try:
                    body = endpoint._render().encode("utf-8")
                except Exception:
                    self.send_response(500)
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; version=0.0.4")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, fmt: str, *args) -> None:
                del fmt, args

        self._server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="vr-session-metrics",
            daemon=True,
        )
        self._thread.start()

    @property
    def port(self) -> int:
        return int(self._server.server_address[1])

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2)


class SessionRecorder:
    """Start and stop MCAP parts. Inject ``runner`` and ``size_of`` in tests."""

    def __init__(
        self,
        config: RecorderConfig,
        runner: Optional[SubprocessBagRunner] = None,
        *,
        size_of: Callable[[Path], int] = directory_size,
    ) -> None:
        self._config = config.validated()
        self._runner = runner if runner is not None else SubprocessBagRunner()
        self._size_of = size_of
        self._lock = threading.RLock()
        self._recording = False
        self._end_on_gap = False
        self._suppress_auto = False
        self._last_heartbeat: Optional[float] = None
        self._session_dir: Optional[Path] = None
        self._active: Optional[Path] = None
        self._part_index = 0
        self._gaps = HeartbeatTracker(self._config.heartbeat_gap_sec)
        self._trips = SafetyTripCounter()
        self._rtt_ms: Optional[float] = None

    @property
    def active_part(self) -> Optional[Path]:
        with self._lock:
            return self._active

    def status_text(self) -> str:
        with self._lock:
            if self._recording and self._session_dir is not None:
                return f"recording:{self._session_dir}"
            return "idle"

    def render_metrics(self) -> str:
        with self._lock:
            return render_prometheus(
                rtt_ms=self._rtt_ms,
                heartbeat_gap_sec=self._gaps.gap_sec,
                heartbeat_gaps_total=self._gaps.gaps_total,
                safety_trips_total=self._trips.total,
            )

    def telemetry(self, text: str) -> None:
        rtt = parse_rtt_ms(text)
        if rtt is None:
            return
        with self._lock:
            self._rtt_ms = rtt

    def safety(self, text: str) -> None:
        with self._lock:
            self._trips.observe(text)

    def start(
        self,
        now: float,
        session_id: str = "",
        wall: Optional[datetime] = None,
        *,
        end_on_gap: bool = False,
    ) -> list[str]:
        del now
        with self._lock:
            if self._recording:
                return ["already recording"]
            session_dir, part = self._allocate(session_id, wall)
            cmd = record_argv(str(part), use_sim_time=self._config.use_sim_time)
            try:
                self._runner.start(cmd, log_path=session_dir / "record.log")
            except Exception as exc:
                self._remove_empty_session(session_dir)
                return [f"error: failed to start recording: {exc}"]
            self._recording = True
            self._end_on_gap = end_on_gap
            self._suppress_auto = False
            self._session_dir = session_dir
            self._active = part
            self._part_index = 0
            self._write_manifest()
            return [f"recording {session_dir}"]

    def stop(self, now: float) -> list[str]:
        del now
        notes = self._halt(ended_by_gap=False)
        with self._lock:
            self._suppress_auto = True
        if not notes:
            return ["idle"]
        return notes

    def shutdown(self, now: float) -> list[str]:
        del now
        return self._halt(ended_by_gap=False)

    def heartbeat(self, now: float) -> list[str]:
        with self._lock:
            self._gaps.beat(now)
            self._last_heartbeat = now
            should_start = (
                self._config.auto_session
                and not self._recording
                and not self._suppress_auto
            )
        if not should_start:
            return []
        return self.start(now, end_on_gap=True)

    def poll(self, now: float) -> list[str]:
        with self._lock:
            self._gaps.tick(now)
            idle = self._config.session_idle_sec
            if self._suppress_auto and (
                self._last_heartbeat is None or now - self._last_heartbeat >= idle
            ):
                self._suppress_auto = False
            gap_stop = (
                self._recording
                and self._end_on_gap
                and self._last_heartbeat is not None
                and (now - self._last_heartbeat) >= idle
            )
            rotate = False
            if self._recording and not gap_stop and self._active is not None:
                rotate = self._size_of(self._active) >= self._config.max_bag_size_bytes
            if self._recording and not gap_stop and not rotate:
                self._prune(active=self._active, keep_session=self._session_dir)
        if gap_stop:
            return self._halt(ended_by_gap=True)
        if rotate:
            return self._rotate()
        return []

    def _allocate(self, session_id: str, wall: Optional[datetime]) -> tuple[Path, Path]:
        self._config.output_dir.mkdir(parents=True, exist_ok=True)
        base = session_directory_name(session_id, _as_utc(wall))
        session_dir = self._config.output_dir / base
        suffix = 2
        while session_dir.exists():
            session_dir = self._config.output_dir / f"{base}-{suffix}"
            suffix += 1
        session_dir.mkdir()
        return session_dir, session_dir / "part-0000"

    def _remove_empty_session(self, session_dir: Path) -> None:
        if not session_dir.is_dir():
            return
        try:
            if any(session_dir.iterdir()):
                return
            session_dir.rmdir()
        except OSError:
            return

    def _write_manifest(self) -> None:
        session = self._session_dir
        if session is None or not session.is_dir():
            return
        names = [
            path.name
            for path in iter_parts(self._config.output_dir)
            if _same_path(path.parent, session)
        ]
        active = ""
        if self._active is not None and _same_path(self._active.parent, session):
            active = self._active.name
            if active not in names:
                names.append(active)
        payload = {
            "active": active,
            "max_bag_size_bytes": self._config.max_bag_size_bytes,
            "max_total_bytes": self._config.max_total_bytes,
            "parts": names,
            "regex": topic_regex(),
            "storage": "mcap",
            "topics": list(TOPIC_GLOBS),
        }
        path = session / "session.json"
        tmp = session / "session.json.tmp"
        tmp.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        tmp.replace(path)

    def _prune(
        self,
        *,
        active: Optional[Path],
        keep_session: Optional[Path],
    ) -> list[Path]:
        return prune_to_cap(
            self._config.output_dir,
            self._config.max_total_bytes,
            active,
            size_of=self._size_of,
            keep_session=keep_session,
        )

    def _halt(self, *, ended_by_gap: bool) -> list[str]:
        with self._lock:
            if not self._recording:
                return []
            session = self._session_dir
            self._recording = False
            self._end_on_gap = False
            self._active = None
            if ended_by_gap:
                self._suppress_auto = False
            notes = [f"stopped {session}"]
        try:
            self._runner.stop()
        except Exception as exc:
            notes.append(f"error: stop failed: {exc}")
        with self._lock:
            deleted = self._prune(active=None, keep_session=None)
            if session is not None and session.exists():
                self._write_manifest()
            if deleted:
                notes.append("pruned " + ", ".join(path.name for path in deleted))
        return notes

    def _rotate(self) -> list[str]:
        with self._lock:
            if not self._recording or self._session_dir is None:
                return []
            session = self._session_dir
            self._part_index += 1
            nxt = session / f"part-{self._part_index:04d}"
            finished = self._active
            self._active = None
        try:
            self._runner.stop()
        except Exception as exc:
            with self._lock:
                self._recording = False
                self._active = None
            return [f"error: rotate stop failed: {exc}"]
        with self._lock:
            self._prune(active=None, keep_session=session)
            cmd = record_argv(str(nxt), use_sim_time=self._config.use_sim_time)
        try:
            self._runner.start(cmd, log_path=session / "record.log")
        except Exception as exc:
            with self._lock:
                self._recording = False
                self._active = None
            return [f"error: rotate start failed: {exc}"]
        with self._lock:
            self._active = nxt
            self._recording = True
            self._write_manifest()
            self._prune(active=nxt, keep_session=session)
            self._write_manifest()
        return [f"rotated {nxt} after {finished}"]
