# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
from pathlib import Path

import pytest

from ohho_webrtc.config import (
    DEFAULT_TRACKS,
    ConfigError,
    default_config,
    ice_servers,
    load_config,
    parse_streamer_config,
)

_YAML = Path(__file__).resolve().parents[1] / "config" / "webrtc_params.yaml"


def test_defaults_are_chase_front_wrist_720p30():
    config = default_config()
    assert [(spec.name, spec.topic) for spec in config.tracks] == list(DEFAULT_TRACKS)
    assert config.tracks[0].topic == "/camera/chase/image_raw"
    assert config.tracks[1].topic == "/camera/front/image_raw"
    assert config.tracks[2].topic == "/camera/wrist/image_raw"
    assert (config.width, config.height, config.fps) == (1280, 720, 30)
    assert config.bitrate_bps == 3_000_000
    assert config.gop == 15
    assert config.bframes == 0
    assert config.stun_urls == ("stun:stun.l.google.com:19302",)
    assert config.turn_urls == ()
    assert config.host == "127.0.0.1"
    assert config.port == 8766
    assert config.encoder == "auto"
    assert config.test_pattern is False


def test_checked_in_yaml_matches_defaults():
    assert load_config(str(_YAML)) == default_config()


def test_ros_parameter_overrides():
    config = parse_streamer_config(
        {
            "ohho_webrtc": {
                "ros__parameters": {
                    "track_names": ["chase"],
                    "track_topics": ["/camera/custom/image_raw"],
                    "bitrate_bps": 4_000_000,
                    "gop": 8,
                    "width": 640,
                    "height": 480,
                    "fps": 15,
                    "encoder": "x264",
                    "signaling_port": 9000,
                    "test_pattern": True,
                }
            }
        }
    )
    assert config.tracks[0].topic == "/camera/custom/image_raw"
    assert config.bitrate_bps == 4_000_000
    assert config.gop == 8
    assert config.width == 640
    assert config.test_pattern is True
    assert config.bframes == 0


def test_bframes_cannot_be_enabled():
    with pytest.raises(ConfigError, match="bframes"):
        parse_streamer_config({"bframes": 2})


def test_bitrate_stays_inside_two_to_four_megabits():
    with pytest.raises(ConfigError, match="bitrate"):
        parse_streamer_config({"bitrate_bps": 1_999_999})
    with pytest.raises(ConfigError, match="bitrate"):
        parse_streamer_config({"bitrate_bps": 4_000_001})
    assert parse_streamer_config({"bitrate_bps": 2_000_000}).bitrate_bps == 2_000_000


def test_track_lists_must_match():
    with pytest.raises(ConfigError, match="same non-empty"):
        parse_streamer_config(
            {"track_names": ["chase", "front"], "track_topics": ["/camera/chase/image_raw"]}
        )


def test_topic_must_be_absolute():
    with pytest.raises(ConfigError, match="absolute"):
        parse_streamer_config(
            {"track_names": ["chase"], "track_topics": ["camera/chase/image_raw"]}
        )


def test_turn_requires_credentials_and_ice_servers_shape():
    with pytest.raises(ConfigError, match="turn_username"):
        parse_streamer_config({"turn_urls": ["turn:turn.example.com:3478"]})
    config = parse_streamer_config(
        {
            "turn_urls": ["turn:turn.example.com:3478"],
            "turn_username": "ohho",
            "turn_credential": "secret",
        }
    )
    servers = ice_servers(config)
    assert servers[0] == {"urls": "stun:stun.l.google.com:19302"}
    assert servers[1]["urls"] == ["turn:turn.example.com:3478"]
    assert servers[1]["username"] == "ohho"
    assert servers[1]["credential"] == "secret"


def test_gop_and_size_limits():
    with pytest.raises(ConfigError, match="gop"):
        parse_streamer_config({"gop": 31})
    with pytest.raises(ConfigError, match="even"):
        parse_streamer_config({"width": 641, "height": 480})
