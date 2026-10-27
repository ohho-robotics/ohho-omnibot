"""Model adapter interface for the policy runtime.

Every backend implements ``PolicyAdapter``. The policy node loads one by
registry name and calls ``select_action`` once per control cycle. The runtime
applies the safety hooks. A model cannot turn those hooks off.

The contract is robot-agnostic: an adapter declares observation keys, image
shape, state and action units, and the control rate it was built for. It does
not name a robot vendor.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np


DEFAULT_IMAGE_KEYS = ("observation.images.wrist", "observation.images.bev")
DEFAULT_STATE_KEY = "observation.state"
DEFAULT_VECTOR_NAMES = (
    "joint_0",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "base_vx",
    "base_vy",
    "base_yaw_rate",
)
DEFAULT_VECTOR_UNITS = (
    "rad",
    "rad",
    "rad",
    "rad",
    "rad",
    "rad",
    "m/s",
    "m/s",
    "rad/s",
)
DEFAULT_CONTROL_HZ = 10.0


@dataclass(frozen=True)
class ImageSpec:
    """One image input after the runtime has decoded the camera frame.

    ``layout`` ``1CHW`` is a batch of one: ``(1, channels, height, width)``
    float32, scaled to ``value_range``. Cameras arrive as uint8 HWC. The
    runtime resizes to ``(width, height)`` and converts before ``select_action``.
    """

    key: str
    channels: int
    height: int
    width: int
    layout: str = "1CHW"
    dtype: str = "float32"
    value_range: tuple[float, float] = (0.0, 1.0)


@dataclass(frozen=True)
class VectorSpec:
    """Named vector with one unit per component.

    State and action use the same default order: joint position targets in
    radians, then planar base ``vx`` and ``vy`` in metres per second, then yaw
    rate in radians per second. The current policy runtime slices action
    ``[:6]`` as joint targets and ``[6:9]`` as base velocity.
    """

    key: str
    names: tuple[str, ...]
    units: tuple[str, ...]

    def __post_init__(self) -> None:
        if len(self.names) != len(self.units):
            raise ValueError("vector names and units must have the same length")

    @property
    def dim(self) -> int:
        return len(self.names)


@dataclass(frozen=True)
class LatencyBudget:
    """Control rate the adapter was built for.

    The runtime drops a tick whose preprocess plus ``select_action`` exceeds
    ``max_tick_s`` (one control period). The policy node reads this period
    from its ``policy_hz`` parameter. The default, 10 Hz, matches this budget.
    """

    control_hz: float

    def __post_init__(self) -> None:
        if self.control_hz <= 0:
            raise ValueError("control_hz must be positive")

    @property
    def max_tick_s(self) -> float:
        return 1.0 / float(self.control_hz)


@dataclass(frozen=True)
class SafetyHooks:
    """Hooks the policy runtime applies on every tick.

    These are not methods a model implements, and they are not optional.
    ``PolicyGuard`` and ``policy_node`` refuse a missing or all-zero frame,
    drop a tick slower than the latency budget, and limit the action before
    anything is published.
    """

    refuse_missing_or_black_frames: bool = True
    drop_stale_inference: bool = True
    limit_actions: bool = True


RUNTIME_SAFETY = SafetyHooks()


@dataclass(frozen=True)
class ModelAdapterContract:
    """What a model must accept and return to run in the policy runtime."""

    images: tuple[ImageSpec, ...]
    state: VectorSpec
    task_key: Optional[str]
    action: VectorSpec
    latency: LatencyBudget
    safety: SafetyHooks = RUNTIME_SAFETY


def _fit_labels(labels: tuple[str, ...], size: int, prefix: str) -> tuple[str, ...]:
    if size <= 0:
        raise ValueError(f"{prefix} size must be positive")
    if len(labels) == size:
        return labels
    if len(labels) > size:
        return labels[:size]
    extra = tuple(f"{prefix}_{index}" for index in range(len(labels), size))
    return labels + extra


class PolicyAdapter(ABC):
    """Uniform interface every policy backend must satisfy.

    Lifecycle::

        adapter = make_policy("reference_linear", checkpoint="random", device="cpu")
        adapter.load("random", device="cpu")   # make_policy already calls load
        adapter.reset()                         # between episodes
        action = adapter.select_action(obs)    # once per control cycle

    ``obs`` contains every ``contract.images`` key, ``contract.state.key``,
    and ``contract.task_key`` when that key is not None. ``select_action``
    returns a flat float32 vector of length ``contract.action.dim``.
    """

    @abstractmethod
    def load(self, checkpoint: str, device: str = "cuda") -> None:
        """Load weights from a hub id, a local path, or an explicit random init.

        ``checkpoint`` values ``None``, ``""``, ``"random"``, ``"scratch"``,
        and ``"none"`` mean "do not download". Adapters that only have a
        remote checkpoint raise if that download is impossible.
        """

    @abstractmethod
    def reset(self) -> None:
        """Clear recurrent state or an action-chunk queue between episodes."""

    @abstractmethod
    def select_action(self, obs: dict) -> np.ndarray:
        """Return a flat float32 vector of shape ``(action_dim,)``.

        Image values are ``(1, C, H, W)`` float32 in ``[0, 1]``. State is
        ``(1, state_dim)`` float32. A language-conditioned model reads
        ``task_key`` as a string. Other keys may be ignored.
        """

    @property
    def model(self):
        """Underlying module for a training loop, or None when there is none."""
        return getattr(self, "_policy", None) or getattr(self, "_model", None)

    @property
    def image_keys(self) -> list[str]:
        """Ordered image keys this model reads from the observation dict."""
        return list(DEFAULT_IMAGE_KEYS)

    @property
    def state_key(self) -> str:
        return DEFAULT_STATE_KEY

    @property
    def task_key(self) -> Optional[str]:
        """Language task key, or None when the model is not language-conditioned."""
        return None

    @property
    def action_dim(self) -> int:
        """Length of the vector returned by ``select_action``."""
        return len(DEFAULT_VECTOR_NAMES)

    @property
    def state_dim(self) -> int:
        return len(DEFAULT_VECTOR_NAMES)

    @property
    def image_size(self) -> tuple[int, int]:
        """``(width, height)`` the runtime resizes each image to."""
        return (320, 240)

    @property
    def image_channels(self) -> int:
        return 3

    @property
    def action_names(self) -> tuple[str, ...]:
        return DEFAULT_VECTOR_NAMES

    @property
    def action_units(self) -> tuple[str, ...]:
        return DEFAULT_VECTOR_UNITS

    @property
    def state_names(self) -> tuple[str, ...]:
        return DEFAULT_VECTOR_NAMES

    @property
    def state_units(self) -> tuple[str, ...]:
        return DEFAULT_VECTOR_UNITS

    @property
    def control_hz(self) -> float:
        """Control rate this adapter was built for. The runtime may run slower."""
        return DEFAULT_CONTROL_HZ

    @property
    def contract(self) -> ModelAdapterContract:
        """Inputs, outputs, latency budget, and the runtime safety hooks."""
        width, height = self.image_size
        images = tuple(
            ImageSpec(
                key=key,
                channels=self.image_channels,
                height=int(height),
                width=int(width),
            )
            for key in self.image_keys
        )
        state = VectorSpec(
            key=self.state_key,
            names=_fit_labels(tuple(self.state_names), self.state_dim, "state"),
            units=_fit_labels(tuple(self.state_units), self.state_dim, "unit"),
        )
        action = VectorSpec(
            key="action",
            names=_fit_labels(tuple(self.action_names), self.action_dim, "action"),
            units=_fit_labels(tuple(self.action_units), self.action_dim, "unit"),
        )
        return ModelAdapterContract(
            images=images,
            state=state,
            task_key=self.task_key,
            action=action,
            latency=LatencyBudget(float(self.control_hz)),
            safety=RUNTIME_SAFETY,
        )

    def validate_action(self, action: np.ndarray) -> np.ndarray:
        """Return ``action`` as a finite float32 vector of length ``action_dim``."""
        arr = np.asarray(action, dtype=np.float32).reshape(-1)
        if arr.shape != (self.action_dim,):
            raise ValueError(
                f"select_action must return shape {(self.action_dim,)}, got {arr.shape}"
            )
        if not bool(np.all(np.isfinite(arr))):
            raise ValueError("select_action returned a non-finite value")
        return arr
