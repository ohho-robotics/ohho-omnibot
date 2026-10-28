"""Policy model registry for OmniBot visuomotor training and inference.

Quick start
-----------
    from lerobot_engine.models import make_policy, list_models

    print(list_models())

    adapter = make_policy("reference_linear", checkpoint="random", device="cpu")
    adapter.reset()
    action = adapter.select_action(obs_dict)   # np.ndarray (9,)

The interface is ``PolicyAdapter.contract``. See ``docs/model-adapter.md``.
"""

from .base import (
    ModelAdapterContract,
    PolicyAdapter,
    RUNTIME_SAFETY,
)
from .registry import register, make_policy, list_models

# Import adapters so they self-register via @register(...)
from . import smolvla, act, diffusion, openvla, serve, reference  # noqa: F401

__all__ = [
    "ModelAdapterContract",
    "PolicyAdapter",
    "RUNTIME_SAFETY",
    "register",
    "make_policy",
    "list_models",
]
