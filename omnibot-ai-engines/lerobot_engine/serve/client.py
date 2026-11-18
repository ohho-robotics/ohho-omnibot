"""HTTP client for OhhO Serve (/health, /load_model, /predict, /metrics)."""

from __future__ import annotations

import base64
import json
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Mapping, Sequence

import numpy as np


class ServeError(RuntimeError):
    """Serve request failed or returned an unexpected payload."""


def normalize_checkpoint_uri(checkpoint: str) -> str:
    """Strip ``hf://`` so checkpoints match ``ohho serve --checkpoint``."""
    path = (checkpoint or "").strip()
    if path.lower().startswith("hf://"):
        return path[5:]
    return path


def tensor_to_uint8_hwc(value: Any) -> np.ndarray:
    """Convert policy obs image (CHW float batch or HWC uint8) to HWC uint8."""
    arr = np.asarray(value)
    if arr.ndim == 4:
        arr = arr[0]
    if arr.ndim == 3 and arr.shape[0] in (1, 3) and arr.shape[-1] not in (1, 3):
        arr = np.transpose(arr, (1, 2, 0))
    if arr.dtype != np.uint8:
        if arr.max() <= 1.0:
            arr = (np.clip(arr, 0.0, 1.0) * 255.0).astype(np.uint8)
        else:
            arr = arr.astype(np.uint8)
    if arr.ndim == 2:
        arr = np.stack([arr, arr, arr], axis=-1)
    return arr


def encode_image_b64(image_hwc: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(image_hwc).tobytes()).decode("ascii")


def parse_action_vector(action_field: Any, action_dim: int = 9) -> np.ndarray:
    """Parse OhhO Serve ``action`` field into a flat float32 vector."""
    if isinstance(action_field, Mapping):
        if "vector" in action_field:
            vec = action_field["vector"]
        elif "action" in action_field:
            vec = action_field["action"]
        else:
            raise ServeError(f"action dict missing 'vector': {action_field!r}")
    elif isinstance(action_field, Sequence):
        vec = action_field
    else:
        raise ServeError(f"unsupported action payload: {type(action_field)!r}")
    out = np.asarray(vec, dtype=np.float32).ravel()
    if out.size < action_dim:
        padded = np.zeros(action_dim, dtype=np.float32)
        padded[: out.size] = out
        return padded
    return out[:action_dim]


class ServeClient:
    """Minimal OhhO Serve REST client (stdlib only)."""

    def __init__(
        self,
        base_url: str = "http://127.0.0.1:8000",
        *,
        timeout_sec: float = 0.5,
        api_key: str = "",
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = float(timeout_sec)
        self.api_key = api_key or ""

    def _headers(self, json_body: bool = False) -> dict[str, str]:
        headers: dict[str, str] = {}
        if self.api_key:
            headers["X-API-Key"] = self.api_key
        if json_body:
            headers["Content-Type"] = "application/json"
        return headers

    def _request(
        self,
        method: str,
        path: str,
        *,
        query: dict[str, str] | None = None,
        body: bytes | None = None,
        json_body: bool = False,
    ) -> Any:
        url = f"{self.base_url}{path}"
        if query:
            url = f"{url}?{urllib.parse.urlencode(query)}"
        req = urllib.request.Request(
            url,
            data=body,
            method=method,
            headers=self._headers(json_body=json_body),
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
                raw = resp.read()
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise ServeError(f"HTTP {exc.code} {path}: {detail}") from exc
        except urllib.error.URLError as exc:
            raise ServeError(f"Serve unreachable at {url}: {exc}") from exc
        if not raw:
            return {}
        return json.loads(raw.decode("utf-8"))

    def health(self) -> dict[str, Any]:
        return self._request("GET", "/health")

    def models(self) -> dict[str, Any]:
        return self._request("GET", "/models")

    def metrics_text(self) -> str:
        url = f"{self.base_url}/metrics"
        req = urllib.request.Request(url, method="GET", headers=self._headers())
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
                return resp.read().decode("utf-8")
        except urllib.error.URLError as exc:
            raise ServeError(f"GET /metrics failed: {exc}") from exc

    def load_model(self, model_path: str) -> dict[str, Any]:
        path = normalize_checkpoint_uri(model_path)
        return self._request("POST", "/load_model", query={"model_path": path})

    def predict(
        self,
        instruction: str,
        wrist_image: np.ndarray,
        *,
        bev_image: np.ndarray | None = None,
        extra_config: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, float]:
        wrist_hwc = tensor_to_uint8_hwc(wrist_image)
        config: dict[str, Any] = dict(extra_config or {})
        if bev_image is not None:
            config["bev_image_b64"] = encode_image_b64(tensor_to_uint8_hwc(bev_image))
            config["bev_shape"] = list(tensor_to_uint8_hwc(bev_image).shape)
        payload = {
            "instruction": instruction or "",
            "image_base64": encode_image_b64(wrist_hwc),
            "config": config,
        }
        doc = self._request(
            "POST",
            "/predict",
            body=json.dumps(payload).encode("utf-8"),
            json_body=True,
        )
        latency_ms = float(doc.get("latency_ms", 0.0))
        return parse_action_vector(doc.get("action", {})), latency_ms
