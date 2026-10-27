"""
Bridge ROS teleop buffers to ohho-os :class:`~ohho.data.recorder.Recorder`.

State/action vectors follow the 9-D mobile-manipulation schema; images are
written as ``observation.images.wrist`` and ``observation.images.bev`` via
``data_engine.recording.live_episode.append_live_episode``.
"""

from __future__ import annotations

from typing import Any, List, Optional

import numpy as np

try:
    from ohho import capabilities as caps
    from ohho.data.recorder import Recorder
    from ohho.registry import get_spec
    from ohho.schema import JointReading, Odometry as OhhoOdom, Telemetry

    OHHO_AVAILABLE = True
except ImportError:
    OHHO_AVAILABLE = False
    Recorder = None  # type: ignore[misc, assignment]

try:
    from data_engine.recording.live_episode import append_live_episode

    WRITER_AVAILABLE = True
except ImportError:
    WRITER_AVAILABLE = False


class _RosStateShim:
    """Minimal stand-in for :class:`~ohho.robot.Robot` used only by ``Recorder``."""

    def __init__(self) -> None:
        if not OHHO_AVAILABLE:
            raise RuntimeError("ohho-os is required for LeRobot recording (pip install ohho-os)")
        self.spec = get_spec("omnibot")
        self._positions = [0.0] * len(self.spec.joint_names)
        self._base_vel = [0.0, 0.0, 0.0]

    def has(self, capability: str) -> bool:
        return capability == caps.MANIPULATION

    def set_observation(self, state_9d: np.ndarray) -> None:
        s = np.asarray(state_9d, dtype=np.float32).reshape(-1)
        n_arm = len(self._positions)
        self._positions = s[:n_arm].tolist()
        self._base_vel = s[n_arm : n_arm + 3].tolist()

    def telemetry(self) -> Telemetry:
        joints = [
            JointReading(name=n, position=float(p))
            for n, p in zip(self.spec.joint_names, self._positions)
        ]
        odom = OhhoOdom(vx=self._base_vel[0], vy=self._base_vel[1], omega=self._base_vel[2])
        return Telemetry(joints=joints, odom=odom)

    def drive(self, vx: float = 0.0, vy: float = 0.0, w: float = 0.0):
        return None

    def move_joints(self, positions: List[float]):
        return None


class OhhoTeleopRecorder:
    """Timer-driven episode buffer backed by ohho-os ``Recorder``."""

    def __init__(self, repo_id: str, fps: float) -> None:
        if not OHHO_AVAILABLE:
            raise RuntimeError("ohho-os is not installed")
        self._shim = _RosStateShim()
        self._rec = Recorder(self._shim, repo_id=repo_id, fps=fps)
        self._image_frames: list[dict[str, Any]] = []

    def start_episode(self, task: str) -> None:
        self._image_frames = []
        self._rec.start_episode(task=task)

    def discard_episode(self) -> None:
        self._rec.discard_episode()
        self._image_frames = []

    def capture_frame(
        self,
        state: np.ndarray,
        action: np.ndarray,
        wrist_image: np.ndarray,
        bev_image: np.ndarray,
        *,
        wall_timestamp: float,
        wrist_timestamp: Optional[float] = None,
        bev_timestamp: Optional[float] = None,
    ) -> None:
        self._shim.set_observation(state)
        self._rec._last_action = np.asarray(action, dtype=np.float32).reshape(-1).tolist()
        self._rec.capture_frame()
        self._image_frames.append(
            {
                "wrist_image": wrist_image,
                "bev_image": bev_image,
                "timestamp": wall_timestamp,
                "wrist_timestamp": wrist_timestamp if wrist_timestamp is not None else wall_timestamp,
                "bev_timestamp": bev_timestamp if bev_timestamp is not None else wall_timestamp,
            }
        )

    @property
    def frame_count(self) -> int:
        return len(self._image_frames)

    def save_episode(
        self,
        output_dir: str,
        task: str,
        fps: float,
        repo_id: str,
    ) -> int:
        if not WRITER_AVAILABLE:
            raise RuntimeError(
                "data_engine is not on PYTHONPATH — cannot write LeRobot v2 dataset"
            )
        ep = self._rec.stop_episode()
        if not ep.frames:
            raise RuntimeError("no frames captured")

        merged: list[dict[str, Any]] = []
        for ohho_frame, img in zip(ep.frames, self._image_frames):
            merged.append(
                {
                    "observation.state": ohho_frame.observation_state,
                    "action": ohho_frame.action,
                    "timestamp": ohho_frame.timestamp,
                    "wrist_image": img["wrist_image"],
                    "bev_image": img["bev_image"],
                    "wrist_timestamp": img["wrist_timestamp"],
                    "bev_timestamp": img["bev_timestamp"],
                }
            )
        return append_live_episode(
            output_dir,
            merged,
            task,
            fps=int(fps),
            repo_id=repo_id,
        )
