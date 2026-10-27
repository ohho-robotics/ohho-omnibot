"""Decide whether a headset ``/policy/load`` message names a new checkpoint."""

from __future__ import annotations


def requested_checkpoint(current: str, message_data: str) -> str | None:
    """Return the checkpoint to load, or None when the message is empty or unchanged.

    Does not enable inference and does not touch the safety gate.
    """
    text = (message_data or "").strip()
    if not text or text == (current or ""):
        return None
    return text
