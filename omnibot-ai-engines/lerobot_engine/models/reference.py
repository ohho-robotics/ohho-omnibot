"""CPU reference adapters with randomly initialised numpy weights.

These two models exist so the policy runtime can be exercised without a
checkpoint download, a GPU, or torch. They are not trained policies. Both
implement ``PolicyAdapter`` and nothing else in the runtime special-cases them.

``reference_linear``
    seed 447. Features are the 9-D state, the mean of the wrist image, and the
    mean of the bird's-eye image (11,). ``action = W @ features + b`` with
    ``W`` shape ``(9, 11)`` drawn from ``normal(0, 0.05)`` and ``b`` zeros.

``reference_mlp``
    seed 448. Features are the 9-D state plus the per-channel mean of each
    image (15,). One hidden layer of 16 units, ``tanh``, then a linear map to
    9 actions. Weights use the same distribution. Biases are zeros.

Both stay on CPU in float32. ``device`` is recorded and ignored.
"""

from __future__ import annotations

from typing import Any, Optional

import numpy as np

from .base import PolicyAdapter
from .registry import register

LINEAR_SEED = 447
MLP_SEED = 448
WEIGHT_SCALE = 0.05
MLP_HIDDEN = 16


def _as_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy") and not isinstance(value, np.ndarray):
        value = value.numpy()
    return np.asarray(value, dtype=np.float32)


def _state_vector(obs: dict, key: str, dim: int) -> np.ndarray:
    raw = obs.get(key)
    if raw is None:
        return np.zeros(dim, dtype=np.float32)
    arr = _as_numpy(raw).reshape(-1)
    if arr.size < dim:
        arr = np.pad(arr, (0, dim - arr.size))
    return arr[:dim].astype(np.float32, copy=False)


def _image_stat(img: Any, per_channel: bool) -> np.ndarray:
    width = 3 if per_channel else 1
    if img is None:
        return np.zeros(width, dtype=np.float32)
    arr = _as_numpy(img)
    while arr.ndim > 3 and arr.shape[0] == 1:
        arr = arr[0]
    if arr.size == 0:
        return np.zeros(width, dtype=np.float32)
    if arr.ndim == 3 and arr.shape[0] in (1, 3) and arr.shape[-1] not in (1, 3):
        if per_channel:
            return arr.reshape(arr.shape[0], -1).mean(axis=1).astype(np.float32)
        return np.array([float(arr.mean())], dtype=np.float32)
    if arr.ndim == 3 and arr.shape[-1] in (1, 3):
        if per_channel:
            return arr.reshape(-1, arr.shape[-1]).mean(axis=0).astype(np.float32)
        return np.array([float(arr.mean())], dtype=np.float32)
    fill = float(arr.mean())
    return np.full(width, fill, dtype=np.float32)


class _NumpyReference(PolicyAdapter):
    """Shared load path for the two numpy reference models."""

    model_type = ""
    seed_default = 0
    per_channel_images = False
    hidden_units = 0

    def __init__(self) -> None:
        self.seed = self.seed_default
        self.manifest: dict[str, Any] = {}
        self._w: Optional[np.ndarray] = None
        self._b: Optional[np.ndarray] = None
        self._w1: Optional[np.ndarray] = None
        self._b1: Optional[np.ndarray] = None
        self._w2: Optional[np.ndarray] = None
        self._b2: Optional[np.ndarray] = None

    def load(self, checkpoint: str | None = None, device: str = "cpu", seed: int | None = None, **kwargs: object) -> None:
        del kwargs
        self.seed = self.seed_default if seed is None else int(seed)
        feature_dim = self._feature_dim()
        rng = np.random.default_rng(self.seed)
        self._init_weights(rng, feature_dim)
        self.manifest = {
            "model_type": self.model_type,
            "checkpoint_argument": None if checkpoint is None else str(checkpoint),
            "weights": "randomly initialised numpy float32",
            "downloaded_checkpoint": False,
            "checkpoint_file": None,
            "requested_device": str(device),
            "executed_on": "cpu",
            "seed": self.seed,
            "distribution": f"normal(loc=0, scale={WEIGHT_SCALE})",
            "bias": "zeros",
            "dtype": "float32",
            "feature_dim": feature_dim,
            "weight_shapes": self._weight_shapes(),
            "features": self._feature_note(),
            "real_robot": False,
        }

    def reset(self) -> None:
        return None

    def select_action(self, obs: dict) -> np.ndarray:
        if self._w is None and self._w1 is None:
            raise RuntimeError(f"{self.model_type} is not loaded")
        parts = [_state_vector(obs, self.state_key, self.state_dim)]
        for key in self.image_keys:
            parts.append(_image_stat(obs.get(key), self.per_channel_images))
        features = np.concatenate(parts).astype(np.float32, copy=False)
        return self.validate_action(self._forward(features))

    def _feature_dim(self) -> int:
        image_dim = 3 if self.per_channel_images else 1
        return self.state_dim + image_dim * len(self.image_keys)

    def _feature_note(self) -> str:
        if self.per_channel_images:
            return "state[9] + per-channel mean of each image"
        return "state[9] + mean of each image"

    def _init_weights(self, rng: np.random.Generator, feature_dim: int) -> None:
        raise NotImplementedError

    def _forward(self, features: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def _weight_shapes(self) -> dict[str, list[int]]:
        raise NotImplementedError

    def _normal(self, rng: np.random.Generator, shape: tuple[int, ...]) -> np.ndarray:
        return rng.normal(0.0, WEIGHT_SCALE, size=shape).astype(np.float32)


@register("reference_linear")
class ReferenceLinearAdapter(_NumpyReference):
    """Single linear map from state and image means to the 9-D action."""

    model_type = "reference_linear"
    seed_default = LINEAR_SEED
    per_channel_images = False

    def _init_weights(self, rng: np.random.Generator, feature_dim: int) -> None:
        self._w = self._normal(rng, (self.action_dim, feature_dim))
        self._b = np.zeros(self.action_dim, dtype=np.float32)

    def _forward(self, features: np.ndarray) -> np.ndarray:
        assert self._w is not None and self._b is not None
        return self._w @ features + self._b

    def _weight_shapes(self) -> dict[str, list[int]]:
        assert self._w is not None and self._b is not None
        return {"W": list(self._w.shape), "b": list(self._b.shape)}


@register("reference_mlp")
class ReferenceMlpAdapter(_NumpyReference):
    """One tanh hidden layer. Different features and weights from the linear model."""

    model_type = "reference_mlp"
    seed_default = MLP_SEED
    per_channel_images = True
    hidden_units = MLP_HIDDEN

    def _init_weights(self, rng: np.random.Generator, feature_dim: int) -> None:
        self._w1 = self._normal(rng, (self.hidden_units, feature_dim))
        self._b1 = np.zeros(self.hidden_units, dtype=np.float32)
        self._w2 = self._normal(rng, (self.action_dim, self.hidden_units))
        self._b2 = np.zeros(self.action_dim, dtype=np.float32)

    def _forward(self, features: np.ndarray) -> np.ndarray:
        assert self._w1 is not None and self._b1 is not None
        assert self._w2 is not None and self._b2 is not None
        hidden = np.tanh(self._w1 @ features + self._b1)
        return self._w2 @ hidden + self._b2

    def _weight_shapes(self) -> dict[str, list[int]]:
        assert self._w1 is not None and self._b1 is not None
        assert self._w2 is not None and self._b2 is not None
        return {
            "W1": list(self._w1.shape),
            "b1": list(self._b1.shape),
            "W2": list(self._w2.shape),
            "b2": list(self._b2.shape),
        }

    def _feature_note(self) -> str:
        return f"state[9] + per-channel mean of each image, tanh hidden {self.hidden_units}"
