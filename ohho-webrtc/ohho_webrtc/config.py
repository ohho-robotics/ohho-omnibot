# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Streamer configuration.

Default image topics:

* ``chase`` — ``/camera/chase/image_raw``. OHH-200 has not landed on main, so
  nothing in this tree publishes that name yet. Override ``track_topics`` when
  the chase camera uses a different topic.
* ``front`` — ``/camera/front/image_raw`` (Gazebo bridge and the hybrid launch).
* ``wrist`` — ``/camera/wrist/image_raw`` (``perception.launch.py``).

Low-latency video is fixed at no B-frames. Bitrate stays inside 2–4 Mbps.
Defaults are 720p30, a 15-frame GOP, and 3 Mbps.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from ohho_webrtc import GATEWAY_SIGNALING_PATH, UPSTREAM_SIGNALING_PATH

_NAME = re.compile(r"^[a-z][a-z0-9_]{0,31}$")
_ENCODERS = ("auto", "nvenc", "x264", "v4l2", "openh264")

DEFAULT_TRACKS = (
    ("chase", "/camera/chase/image_raw"),
    ("front", "/camera/front/image_raw"),
    ("wrist", "/camera/wrist/image_raw"),
)

MIN_BITRATE_BPS = 2_000_000
MAX_BITRATE_BPS = 4_000_000
DEFAULT_BITRATE_BPS = 3_000_000
DEFAULT_GOP = 15
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30
DEFAULT_STUN = "stun:stun.l.google.com:19302"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8766


class ConfigError(ValueError):
    """The parameter mapping is not a valid streamer config."""


@dataclass(frozen=True)
class TrackSpec:
    name: str
    topic: str


@dataclass(frozen=True)
class StreamerConfig:
    tracks: tuple[TrackSpec, ...]
    host: str
    port: int
    stun_urls: tuple[str, ...]
    turn_urls: tuple[str, ...]
    turn_username: str
    turn_credential: str
    width: int
    height: int
    fps: int
    bitrate_bps: int
    gop: int
    encoder: str
    test_pattern: bool

    @property
    def bframes(self) -> int:
        """B-frames are always off. There is no parameter that turns them on."""
        return 0

    def track_map(self) -> dict[str, str]:
        return {spec.name: spec.topic for spec in self.tracks}


def default_config() -> StreamerConfig:
    return parse_streamer_config({})


def ice_servers(config: StreamerConfig) -> list[dict[str, object]]:
    """Browser ``RTCPeerConnection`` iceServers list."""
    servers: list[dict[str, object]] = [{"urls": url} for url in config.stun_urls]
    if config.turn_urls:
        servers.append(
            {
                "urls": list(config.turn_urls),
                "username": config.turn_username,
                "credential": config.turn_credential,
            }
        )
    return servers


def load_config(path: str) -> StreamerConfig:
    import yaml

    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ConfigError("config file must be a mapping")
    return parse_streamer_config(data)


def parse_streamer_config(data: dict) -> StreamerConfig:
    """Accept a flat dict, a ``ros__parameters`` dict, or a node-keyed YAML file."""
    params = _unwrap_params(data)
    if "bframes" in params and _as_int(params["bframes"], "bframes") != 0:
        raise ConfigError("bframes must be 0")

    names, topics = _tracks(params)
    stun_urls = tuple(_urls(params.get("stun_urls", [DEFAULT_STUN]), "stun"))
    turn_urls = tuple(_urls(params.get("turn_urls", []), "turn"))
    username = _as_str(params.get("turn_username", ""))
    credential = _as_str(params.get("turn_credential", ""))
    if turn_urls and (not username or not credential):
        raise ConfigError("turn_urls require turn_username and turn_credential")

    width = _as_int(params.get("width", DEFAULT_WIDTH), "width")
    height = _as_int(params.get("height", DEFAULT_HEIGHT), "height")
    if width < 2 or height < 2 or width > 1920 or height > 1080:
        raise ConfigError("width and height must be between 2 and 1920x1080")
    if width % 2 or height % 2:
        raise ConfigError("width and height must be even")

    fps = _as_int(params.get("fps", DEFAULT_FPS), "fps")
    if fps < 1 or fps > 60:
        raise ConfigError("fps must be between 1 and 60")

    bitrate = _as_int(params.get("bitrate_bps", DEFAULT_BITRATE_BPS), "bitrate_bps")
    if bitrate < MIN_BITRATE_BPS or bitrate > MAX_BITRATE_BPS:
        raise ConfigError("bitrate_bps must be between 2000000 and 4000000")

    gop = _as_int(params.get("gop", DEFAULT_GOP), "gop")
    if gop < 1 or gop > 30:
        raise ConfigError("gop must be between 1 and 30")

    encoder = _as_str(params.get("encoder", "auto"))
    if encoder not in _ENCODERS:
        raise ConfigError("encoder must be auto, nvenc, x264, v4l2, or openh264")

    host = _as_str(params.get("signaling_host", DEFAULT_HOST))
    if not host:
        raise ConfigError("signaling_host is empty")
    port = _as_int(params.get("signaling_port", DEFAULT_PORT), "signaling_port")
    if port < 0 or port > 65535:
        raise ConfigError("signaling_port must be between 0 and 65535")

    test_pattern = _as_bool(params.get("test_pattern", False))
    tracks = tuple(TrackSpec(name, topic) for name, topic in zip(names, topics))
    return StreamerConfig(
        tracks=tracks,
        host=host,
        port=port,
        stun_urls=stun_urls,
        turn_urls=turn_urls,
        turn_username=username,
        turn_credential=credential,
        width=width,
        height=height,
        fps=fps,
        bitrate_bps=bitrate,
        gop=gop,
        encoder=encoder,
        test_pattern=test_pattern,
    )


def signaling_paths() -> dict[str, str]:
    return {
        "signalingPath": UPSTREAM_SIGNALING_PATH,
        "gatewayPath": GATEWAY_SIGNALING_PATH,
    }


def _unwrap_params(data: dict) -> dict:
    if not isinstance(data, dict):
        raise ConfigError("config must be a mapping")
    if "ros__parameters" in data and isinstance(data["ros__parameters"], dict):
        return data["ros__parameters"]
    if len(data) == 1:
        only = next(iter(data.values()))
        if isinstance(only, dict) and "ros__parameters" in only:
            return _unwrap_params(only)
    return data


def _tracks(params: dict) -> tuple[list[str], list[str]]:
    if "track_names" in params or "track_topics" in params:
        names = [_as_str(item) for item in _as_list(params.get("track_names"), "track_names")]
        topics = [_as_str(item) for item in _as_list(params.get("track_topics"), "track_topics")]
    else:
        names = [name for name, _topic in DEFAULT_TRACKS]
        topics = [topic for _name, topic in DEFAULT_TRACKS]
    if not names or len(names) != len(topics):
        raise ConfigError("track_names and track_topics must be the same non-empty length")
    if len(set(names)) != len(names):
        raise ConfigError("track names must be unique")
    if len(set(topics)) != len(topics):
        raise ConfigError("track topics must be unique")
    for name, topic in zip(names, topics):
        if not _NAME.match(name):
            raise ConfigError(f"invalid track name {name!r}")
        if not topic.startswith("/"):
            raise ConfigError(f"topic {topic!r} must be absolute")
    return names, topics


def _urls(value: object, kind: str) -> list[str]:
    urls = [_as_str(item) for item in _as_list(value, f"{kind}_urls")]
    prefix = ("stun:", "stuns:") if kind == "stun" else ("turn:", "turns:")
    for url in urls:
        if not url.startswith(prefix):
            raise ConfigError(f"{url!r} is not a {kind} URL")
    return urls


def _as_list(value: object, label: str) -> list:
    if value is None:
        raise ConfigError(f"{label} is missing")
    if isinstance(value, (str, bytes)):
        raise ConfigError(f"{label} must be a list")
    if not isinstance(value, (list, tuple)):
        raise ConfigError(f"{label} must be a list")
    return list(value)


def _as_str(value: object) -> str:
    if isinstance(value, str):
        return value
    raise ConfigError(f"expected a string, got {type(value).__name__}")


def _as_int(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{label} must be an integer")
    return value


def _as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    raise ConfigError("test_pattern must be a boolean")
