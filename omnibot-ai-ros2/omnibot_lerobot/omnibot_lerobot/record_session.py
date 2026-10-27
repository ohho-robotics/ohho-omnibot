"""Episode recording state machine for the teleop recorder.

Pure Python: no rclpy. The node applies these results and is the only
component that writes an episode. Discard never returns frames to a writer.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import time
from typing import Any, Callable


class RecordPhase(str, Enum):
    IDLE = "idle"
    RECORDING = "recording"
    SAVING = "saving"


@dataclass(frozen=True)
class RecordResult:
    success: bool
    message: str
    episode_index: int


def _message(text: str, episode_index: int) -> str:
    return f"{text} (episode_index={episode_index})"


class RecordSession:
    """Idle → recording → saving, with an episode index that advances only on a successful save."""

    def __init__(
        self,
        task: str = "mobile manipulation task",
        episode_index: int = 0,
        clock: Callable[[], float] | None = None,
    ) -> None:
        if isinstance(episode_index, bool) or not isinstance(episode_index, int):
            raise ValueError("episode_index must be an integer >= 0")
        if episode_index < 0:
            raise ValueError("episode_index must be an integer >= 0")
        self.task = task.strip() if isinstance(task, str) and task.strip() else "mobile manipulation task"
        self.episode_index = episode_index
        self._clock = clock or time.time
        self.state = RecordPhase.IDLE
        self.frame_count = 0
        self.started_at = 0.0
        self._frames: list[Any] = []

    def start(
        self,
        task: str | None = None,
        episode_index: int | None = None,
    ) -> RecordResult:
        """Begin one episode. A second start while busy is refused and changes nothing."""
        if self.state is not RecordPhase.IDLE:
            return RecordResult(
                False,
                _message(f"cannot start while {self.state.value}", self.episode_index),
                self.episode_index,
            )
        if episode_index is not None:
            if isinstance(episode_index, bool) or not isinstance(episode_index, int):
                return RecordResult(
                    False,
                    _message("episode_index must be an integer >= 0", self.episode_index),
                    self.episode_index,
                )
            if episode_index < 0:
                return RecordResult(
                    False,
                    _message("episode_index must be an integer >= 0", self.episode_index),
                    self.episode_index,
                )
        if episode_index is not None:
            self.episode_index = episode_index
        if isinstance(task, str) and task.strip():
            self.task = task.strip()
        self._frames = []
        self.frame_count = 0
        self.started_at = float(self._clock())
        self.state = RecordPhase.RECORDING
        return RecordResult(
            True,
            _message("recording", self.episode_index),
            self.episode_index,
        )

    def request_stop(self) -> tuple[RecordResult, list[Any] | None]:
        """Detach frames for a full write, or return no frames when there is nothing to save.

        The caller must pass the returned list to exactly one writer, then
        ``complete_save``. The session buffer is empty after a successful
        request, so a later discard cannot write those frames.
        """
        if self.state is not RecordPhase.RECORDING:
            return (
                RecordResult(
                    False,
                    _message(f"cannot stop while {self.state.value}", self.episode_index),
                    self.episode_index,
                ),
                None,
            )
        index = self.episode_index
        if not self._frames:
            self._clear_to_idle()
            return (
                RecordResult(False, _message("no frames to save", index), index),
                None,
            )
        frames = self._frames
        self._frames = []
        self.state = RecordPhase.SAVING
        return (
            RecordResult(True, _message("saving", index), index),
            frames,
        )

    def complete_save(self, wrote: bool) -> RecordResult:
        """Leave SAVING. The episode index advances only when the writer finished."""
        if self.state is not RecordPhase.SAVING:
            return RecordResult(
                False,
                _message(f"cannot finish save while {self.state.value}", self.episode_index),
                self.episode_index,
            )
        index = self.episode_index
        self._clear_to_idle()
        if wrote:
            self.episode_index = index + 1
            return RecordResult(True, _message("saved", index), index)
        return RecordResult(False, _message("failed to save", index), index)

    def discard(self) -> RecordResult:
        """Drop the in-progress episode. Does not return frames and does not advance the index."""
        if self.state is not RecordPhase.RECORDING:
            return RecordResult(
                False,
                _message(f"cannot discard while {self.state.value}", self.episode_index),
                self.episode_index,
            )
        index = self.episode_index
        self._frames = []
        self._clear_to_idle()
        return RecordResult(
            True,
            _message("discarded; nothing written", index),
            index,
        )

    def append_frame(self, frame: Any) -> bool:
        if self.state is not RecordPhase.RECORDING:
            return False
        self._frames.append(frame)
        self.frame_count += 1
        return True

    def note_frame_count(self, count: int) -> None:
        """Record a frame count produced by an external buffer (for example the OHH-48 writer)."""
        if self.state is not RecordPhase.RECORDING:
            return
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            return
        self.frame_count = count

    def pending_frames(self) -> list[Any]:
        return list(self._frames)

    @property
    def live_frames(self) -> list[Any]:
        """The live buffer. Writers must use the list returned by ``request_stop`` instead."""
        return self._frames

    def snapshot(self) -> dict[str, Any]:
        return {
            "state": self.state.value,
            "task": self.task,
            "episode_index": self.episode_index,
            "frame_count": self.frame_count,
            "started_at": self.started_at,
        }

    def _clear_to_idle(self) -> None:
        self._frames = []
        self.frame_count = 0
        self.started_at = 0.0
        self.state = RecordPhase.IDLE


def commit_stop(
    session: RecordSession,
    write_frames: Callable[[list[Any]], None],
) -> RecordResult:
    """Stop an episode and call ``write_frames`` only when a full frame list was detached.

    Discard does not use this function. ``write_frames`` is not called for an
    idle session or an empty recording, so those paths create no file.
    """
    result, frames = session.request_stop()
    if frames is None:
        return result
    try:
        write_frames(frames)
    except Exception as exc:
        failed = session.complete_save(False)
        return RecordResult(
            False,
            _message(f"failed to save: {exc}", failed.episode_index),
            failed.episode_index,
        )
    return session.complete_save(True)
