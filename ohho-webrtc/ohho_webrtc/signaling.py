# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""JSON shapes exchanged on the signaling WebSocket.

Inbound messages are ``join``, ``offer``, ``answer``, ``ice``, and ``list``.
The server answers with ``welcome``, ``answer`` or a forwarded ``offer``,
``ice``, ``tracks``, and ``error``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass

from ohho_webrtc import GATEWAY_SIGNALING_PATH, UPSTREAM_SIGNALING_PATH
from ohho_webrtc.config import StreamerConfig, ice_servers

# Required keys for the messages this package emits and accepts.
MESSAGE_SHAPES: dict[str, frozenset[str]] = {
    "join": frozenset({"type", "role", "tracks"}),
    "offer": frozenset({"type", "sdp", "track"}),
    "answer": frozenset({"type", "sdp", "track"}),
    "ice": frozenset({"type", "track", "candidate", "sdpMid", "sdpMLineIndex"}),
    "list": frozenset({"type"}),
    "welcome": frozenset(
        {"type", "role", "viewer", "tracks", "iceServers", "signalingPath", "gatewayPath"}
    ),
    "error": frozenset({"type", "error"}),
    "tracks": frozenset({"type", "tracks"}),
}


class SignalingParseError(ValueError):
    def __init__(self, code: str):
        super().__init__(code)
        self.code = code


@dataclass(frozen=True)
class SignalingMessage:
    type: str
    role: str | None = None
    tracks: tuple[str, ...] = ()
    track: str | None = None
    sdp: str | None = None
    candidate: str | None = None
    sdp_mid: str | None = None
    sdp_mline_index: int | None = None
    viewer: str | None = None


def parse_signaling_message(payload: str) -> SignalingMessage:
    if not isinstance(payload, str):
        raise SignalingParseError("invalid_json")
    try:
        data = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise SignalingParseError("invalid_json") from exc
    if not isinstance(data, dict):
        raise SignalingParseError("invalid_message")
    msg_type = data.get("type")
    if not isinstance(msg_type, str) or msg_type not in ("join", "offer", "answer", "ice", "list"):
        raise SignalingParseError("unknown_type")
    viewer = _optional_str(data.get("viewer"), "viewer")
    if msg_type == "list":
        return SignalingMessage(type="list", viewer=viewer)
    if msg_type == "join":
        role = data.get("role")
        if role not in ("viewer", "publisher"):
            raise SignalingParseError("invalid_role")
        tracks = _track_list(data.get("tracks"), required=True)
        return SignalingMessage(type="join", role=role, tracks=tracks, viewer=viewer)
    if msg_type in ("offer", "answer"):
        sdp = data.get("sdp")
        if not isinstance(sdp, str) or not sdp.startswith("v="):
            raise SignalingParseError("invalid_sdp")
        tracks = _offer_tracks(data)
        return SignalingMessage(
            type=msg_type,
            sdp=sdp,
            tracks=tracks,
            track=tracks[0],
            viewer=viewer,
        )
    track = data.get("track")
    if not isinstance(track, str) or not track:
        raise SignalingParseError("invalid_track")
    if "candidate" not in data:
        raise SignalingParseError("invalid_candidate")
    candidate = data.get("candidate")
    if candidate is not None and not isinstance(candidate, str):
        raise SignalingParseError("invalid_candidate")
    sdp_mid = data.get("sdpMid")
    sdp_mline = data.get("sdpMLineIndex")
    if candidate:
        if not isinstance(sdp_mid, str):
            raise SignalingParseError("invalid_candidate")
        if isinstance(sdp_mline, bool) or not isinstance(sdp_mline, int):
            raise SignalingParseError("invalid_candidate")
    elif sdp_mid is not None and not isinstance(sdp_mid, str):
        raise SignalingParseError("invalid_candidate")
    elif sdp_mline is not None and (isinstance(sdp_mline, bool) or not isinstance(sdp_mline, int)):
        raise SignalingParseError("invalid_candidate")
    return SignalingMessage(
        type="ice",
        track=track,
        tracks=(track,),
        candidate=candidate,
        sdp_mid=sdp_mid if isinstance(sdp_mid, str) else None,
        sdp_mline_index=sdp_mline if isinstance(sdp_mline, int) and not isinstance(sdp_mline, bool) else None,
        viewer=viewer,
    )


def dump_message(payload: dict) -> str:
    missing = MESSAGE_SHAPES[payload["type"]] - payload.keys()
    if missing:
        raise SignalingParseError("incomplete_" + payload["type"])
    return json.dumps(payload, separators=(",", ":"))


def build_welcome(config: StreamerConfig, role: str, viewer: str) -> dict:
    return {
        "type": "welcome",
        "role": role,
        "viewer": viewer,
        "tracks": [{"name": spec.name, "topic": spec.topic} for spec in config.tracks],
        "iceServers": ice_servers(config),
        "signalingPath": UPSTREAM_SIGNALING_PATH,
        "gatewayPath": GATEWAY_SIGNALING_PATH,
    }


def build_error(code: str, **extra: object) -> dict:
    payload: dict[str, object] = {"type": "error", "error": code}
    payload.update(extra)
    return payload


def build_tracks(config: StreamerConfig) -> dict:
    return {
        "type": "tracks",
        "tracks": [{"name": spec.name, "topic": spec.topic} for spec in config.tracks],
    }


def build_session_description(
    msg_type: str,
    sdp: str,
    tracks: tuple[str, ...],
    viewer: str,
) -> dict:
    return {
        "type": msg_type,
        "sdp": sdp,
        "track": tracks[0],
        "tracks": list(tracks),
        "viewer": viewer,
    }


def build_ice(
    *,
    track: str,
    candidate: str | None,
    sdp_mid: str | None,
    sdp_mline_index: int | None,
    viewer: str,
) -> dict:
    return {
        "type": "ice",
        "track": track,
        "candidate": candidate,
        "sdpMid": sdp_mid,
        "sdpMLineIndex": sdp_mline_index,
        "viewer": viewer,
    }


def _optional_str(value: object, label: str) -> str | None:
    if value is None:
        return None
    if isinstance(value, str) and value:
        return value
    raise SignalingParseError("invalid_" + label)


def _track_list(value: object, required: bool) -> tuple[str, ...]:
    if value is None:
        if required:
            raise SignalingParseError("invalid_tracks")
        return ()
    if not isinstance(value, list) or (required and not value):
        raise SignalingParseError("invalid_tracks")
    names = []
    for item in value:
        if not isinstance(item, str) or not item:
            raise SignalingParseError("invalid_tracks")
        names.append(item)
    return tuple(names)


def _offer_tracks(data: dict) -> tuple[str, ...]:
    tracks = _track_list(data.get("tracks"), required=False)
    track = data.get("track")
    if track is not None and not isinstance(track, str):
        raise SignalingParseError("invalid_track")
    if tracks and track and track not in tracks:
        raise SignalingParseError("invalid_track")
    if tracks:
        return tracks
    if isinstance(track, str) and track:
        return (track,)
    raise SignalingParseError("invalid_track")
