"""Model registry: string name → PolicyAdapter subclass.

Usage
-----
Register (done automatically by each adapter module):

    from lerobot_engine.models.registry import register

    @register("my_model")
    class MyAdapter(PolicyAdapter): ...

Instantiate:

    from lerobot_engine.models import make_policy
    adapter = make_policy("smolvla", checkpoint="lerobot/smolvla_base", device="cuda")
"""

from __future__ import annotations

from typing import Optional, Type

from .base import PolicyAdapter

_REGISTRY: dict[str, Type[PolicyAdapter]] = {}


def register(name: str):
    """Class decorator that adds a PolicyAdapter to the global registry."""

    def decorator(cls: Type[PolicyAdapter]) -> Type[PolicyAdapter]:
        _REGISTRY[name] = cls
        return cls

    return decorator


def list_models() -> list[str]:
    """Return all registered model names sorted alphabetically."""
    return sorted(_REGISTRY.keys())


def make_policy(
    model_type: str,
    checkpoint: Optional[str] = None,
    device: str = "cuda",
    **kwargs: object,
) -> PolicyAdapter:
    """Instantiate and load a policy adapter by registry name.

    Args:
        model_type: Registry name, e.g. "smolvla", "act", "diffusion", "openvla".
        checkpoint:  HuggingFace hub ID, local path, or None to initialize from config.
        device:      "cuda" or "cpu".
        **kwargs:    ``serve_url``, ``timeout_sec``, and ``api_key`` are set on the
                     adapter before load. Remaining arguments are passed to
                     adapter.load().

    Returns:
        A fully loaded PolicyAdapter ready for select_action().

    Raises:
        ValueError: Unknown model_type.
        ImportError: Required backend library not installed.
    """
    if model_type not in _REGISTRY:
        raise ValueError(
            f"Unknown model type '{model_type}'. Registered: {list_models()}"
        )
    adapter = _REGISTRY[model_type]()
    load_kwargs = dict(kwargs)
    for key in ("serve_url", "timeout_sec", "api_key"):
        if key in load_kwargs:
            setattr(adapter, key, load_kwargs.pop(key))
    adapter.load(checkpoint, device, **load_kwargs)
    return adapter
