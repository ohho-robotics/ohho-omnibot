"""Remote policy via OhhO Serve (``ohho serve --checkpoint …``)."""

from __future__ import annotations

from typing import Any

import numpy as np

from ..serve.client import ServeClient, ServeError, normalize_checkpoint_uri
from .base import PolicyAdapter
from .registry import register


@register("serve")
class ServeAdapter(PolicyAdapter):
    """Calls a running OhhO Serve instance at ``POST /predict``.

    ``checkpoint`` is the HF hub id or local path passed to ``/load_model``.
    Configure with ``serve_url``, ``timeout_sec``, and optional ``api_key``.
    """

    def __init__(self) -> None:
        self._client: ServeClient | None = None
        self._checkpoint = ""
        self.serve_url = "http://127.0.0.1:8000"
        self.timeout_sec = 0.5
        self.api_key = ""

    def load(self, checkpoint: str, device: str = "cuda") -> None:
        del device  # inference runs on the Serve host
        self._checkpoint = normalize_checkpoint_uri(checkpoint)
        self._client = ServeClient(
            self.serve_url,
            timeout_sec=self.timeout_sec,
            api_key=self.api_key,
        )
        health = self._client.health()
        if not health.get("model_loaded"):
            if not self._checkpoint:
                raise ServeError(
                    "Serve model not loaded and no checkpoint_path configured"
                )
            self._client.load_model(self._checkpoint)

    def reset(self) -> None:
        return

    def select_action(self, obs: dict) -> np.ndarray:
        if self._client is None:
            return np.zeros(self.action_dim, dtype=np.float32)
        wrist_key = self.image_keys[0]
        bev_key = self.image_keys[1] if len(self.image_keys) > 1 else None
        wrist = obs.get(wrist_key)
        bev = obs.get(bev_key) if bev_key else None
        task = ""
        if self.task_key:
            task = str(obs.get(self.task_key, "") or "")
        vector, _latency = self._client.predict(
            task,
            wrist,
            bev_image=bev,
        )
        return vector.astype(np.float32, copy=False)

    @property
    def task_key(self):
        return "task"
