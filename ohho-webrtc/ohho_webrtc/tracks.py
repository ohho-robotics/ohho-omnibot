# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Register named camera tracks and keep the latest RGB frame for each one."""

from __future__ import annotations

import threading
import time

import numpy as np

from ohho_webrtc.config import TrackSpec
from ohho_webrtc.frames import prepare_frame, waiting_frame


class TrackError(ValueError):
    """Track list is inconsistent."""


class TrackRegistry:
    """Latest-frame store. ROS callbacks and the encoder do not share a thread."""

    def __init__(self, tracks: tuple[TrackSpec, ...]):
        if not tracks:
            raise TrackError("at least one track is required")
        names = [spec.name for spec in tracks]
        topics = [spec.topic for spec in tracks]
        if len(set(names)) != len(names):
            raise TrackError("duplicate track name")
        if len(set(topics)) != len(topics):
            raise TrackError("duplicate track topic")
        self._tracks = tracks
        self._by_topic = {spec.topic: spec for spec in tracks}
        self._frames: dict[str, tuple[np.ndarray, int]] = {}
        self._lock = threading.Lock()

    @property
    def tracks(self) -> tuple[TrackSpec, ...]:
        return self._tracks

    def names(self) -> tuple[str, ...]:
        return tuple(spec.name for spec in self._tracks)

    def topic_for(self, name: str) -> str:
        for spec in self._tracks:
            if spec.name == name:
                return spec.topic
        raise TrackError(f"unknown track {name}")

    def push(self, topic: str, image: np.ndarray, stamp_ns: int) -> bool:
        """Store ``image`` for ``topic``. Unknown topics are ignored."""
        spec = self._by_topic.get(topic)
        if spec is None:
            return False
        if image.ndim != 3 or image.shape[2] != 3:
            raise TrackError("image must be HxWx3")
        frame = np.ascontiguousarray(image, dtype=np.uint8)
        with self._lock:
            self._frames[spec.name] = (frame, int(stamp_ns))
        return True

    def has_frame(self, name: str) -> bool:
        with self._lock:
            return name in self._frames

    def frame_for_encode(self, name: str, width: int, height: int) -> np.ndarray:
        """Scaled RGB frame with the capture timestamp burned in.

        Before the first image, the frame is black and labelled ``waiting``.
        """
        self.topic_for(name)
        with self._lock:
            stored = self._frames.get(name)
        if stored is None:
            return waiting_frame(name, width, height, time.time_ns())
        image, stamp_ns = stored
        return prepare_frame(image, name, stamp_ns, width, height)
