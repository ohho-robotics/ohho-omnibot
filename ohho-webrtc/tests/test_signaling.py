# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
import asyncio
import json

import pytest

from ohho_webrtc import GATEWAY_SIGNALING_PATH, UPSTREAM_SIGNALING_PATH
from ohho_webrtc.config import default_config, parse_streamer_config
from ohho_webrtc.signaling import (
    MESSAGE_SHAPES,
    SignalingParseError,
    build_error,
    build_ice,
    build_session_description,
    build_tracks,
    build_welcome,
    dump_message,
    parse_signaling_message,
)
from ohho_webrtc.server import SignalingHub


class MemConn:
    def __init__(self, conn_id):
        self.id = conn_id
        self.role = None
        self.tracks = ()
        self.sent = []

    async def send_json(self, payload):
        self.sent.append(payload)


def test_message_shapes_cover_the_handshake():
    assert MESSAGE_SHAPES["join"] == frozenset({"type", "role", "tracks"})
    assert MESSAGE_SHAPES["offer"] == frozenset({"type", "sdp", "track"})
    assert MESSAGE_SHAPES["answer"] == frozenset({"type", "sdp", "track"})
    assert MESSAGE_SHAPES["ice"] == frozenset(
        {"type", "track", "candidate", "sdpMid", "sdpMLineIndex"}
    )
    assert "gatewayPath" in MESSAGE_SHAPES["welcome"]
    assert "iceServers" in MESSAGE_SHAPES["welcome"]
    assert MESSAGE_SHAPES["error"] == frozenset({"type", "error"})


def test_parse_join_offer_answer_ice_and_list():
    joined = parse_signaling_message(
        json.dumps({"type": "join", "role": "viewer", "tracks": ["chase", "front"]})
    )
    assert joined.role == "viewer"
    assert joined.tracks == ("chase", "front")

    offer = parse_signaling_message(
        json.dumps({"type": "offer", "sdp": "v=0\r\n", "track": "chase"})
    )
    assert offer.tracks == ("chase",)
    assert offer.sdp.startswith("v=")

    answer = parse_signaling_message(
        json.dumps(
            {
                "type": "answer",
                "sdp": "v=0\r\n",
                "tracks": ["chase", "wrist"],
                "viewer": "v1",
            }
        )
    )
    assert answer.track == "chase"
    assert answer.viewer == "v1"

    ice = parse_signaling_message(
        json.dumps(
            {
                "type": "ice",
                "track": "chase",
                "candidate": "candidate:1 1 UDP 1 127.0.0.1 50000 typ host",
                "sdpMid": "0",
                "sdpMLineIndex": 0,
            }
        )
    )
    assert ice.candidate.startswith("candidate:")
    assert ice.sdp_mline_index == 0
    assert parse_signaling_message(json.dumps({"type": "list"})).type == "list"


def test_parse_rejects_bad_shapes():
    with pytest.raises(SignalingParseError, match="invalid_json"):
        parse_signaling_message("{")
    with pytest.raises(SignalingParseError, match="unknown_type"):
        parse_signaling_message(json.dumps({"type": "trickle"}))
    with pytest.raises(SignalingParseError, match="invalid_sdp"):
        parse_signaling_message(json.dumps({"type": "offer", "sdp": "not-sdp", "track": "chase"}))
    with pytest.raises(SignalingParseError, match="invalid_role"):
        parse_signaling_message(json.dumps({"type": "join", "role": "robot", "tracks": ["chase"]}))
    with pytest.raises(SignalingParseError, match="invalid_candidate"):
        parse_signaling_message(json.dumps({"type": "ice", "track": "chase", "candidate": 5}))
    with pytest.raises(SignalingParseError, match="incomplete_welcome"):
        dump_message({"type": "welcome", "role": "viewer"})


def test_welcome_and_error_shapes():
    welcome = build_welcome(default_config(), "viewer", "v1")
    assert MESSAGE_SHAPES["welcome"] <= welcome.keys()
    assert welcome["gatewayPath"] == GATEWAY_SIGNALING_PATH
    assert welcome["signalingPath"] == UPSTREAM_SIGNALING_PATH
    assert welcome["tracks"][0] == {"name": "chase", "topic": "/camera/chase/image_raw"}
    assert welcome["iceServers"][0]["urls"].startswith("stun:")
    error = build_error("join_required")
    assert MESSAGE_SHAPES["error"] <= error.keys()
    assert error["error"] == "join_required"
    tracks = build_tracks(default_config())
    assert tracks["type"] == "tracks"
    ice = build_ice(
        track="front",
        candidate="candidate:1 1 UDP 1 127.0.0.1 9 typ host",
        sdp_mid="0",
        sdp_mline_index=0,
        viewer="v2",
    )
    assert MESSAGE_SHAPES["ice"] <= ice.keys()
    description = build_session_description("answer", "v=0\r\n", ("wrist",), "v2")
    assert description["type"] == "answer"
    assert description["track"] == "wrist"
    assert description["viewer"] == "v2"


def test_hub_requires_join_and_routes_offer_answer_and_ice():
    asyncio.run(_route())


def test_hub_rejects_unknown_track_and_answers_in_process():
    asyncio.run(_in_process())


async def _route():
    hub = SignalingHub(default_config(), answerer=None)
    viewer = MemConn("v1")
    publisher = MemConn("p1")
    await hub.handle(viewer, json.dumps({"type": "offer", "sdp": "v=0\r\n", "track": "chase"}))
    assert viewer.sent[-1]["error"] == "join_required"

    await hub.handle(
        publisher,
        json.dumps({"type": "join", "role": "publisher", "tracks": ["chase", "front", "wrist"]}),
    )
    assert publisher.sent[-1]["type"] == "welcome"
    assert publisher.sent[-1]["role"] == "publisher"

    await hub.handle(
        viewer,
        json.dumps({"type": "join", "role": "viewer", "tracks": ["chase"]}),
    )
    welcome = viewer.sent[-1]
    assert welcome["gatewayPath"] == "/webrtc/signaling"
    assert {item["name"] for item in welcome["tracks"]} == {"chase", "front", "wrist"}

    await hub.handle(viewer, json.dumps({"type": "list"}))
    assert viewer.sent[-1]["type"] == "tracks"

    await hub.handle(
        viewer,
        json.dumps({"type": "offer", "sdp": "v=0\r\noffer\r\n", "track": "chase"}),
    )
    forwarded = publisher.sent[-1]
    assert forwarded["type"] == "offer"
    assert forwarded["track"] == "chase"
    assert forwarded["viewer"] == viewer.id
    assert forwarded["sdp"].startswith("v=")

    await hub.handle(
        publisher,
        json.dumps(
            {
                "type": "answer",
                "sdp": "v=0\r\nanswer\r\n",
                "track": "chase",
                "viewer": viewer.id,
            }
        ),
    )
    assert viewer.sent[-1]["type"] == "answer"
    assert viewer.sent[-1]["sdp"].startswith("v=0")

    await hub.handle(
        viewer,
        json.dumps(
            {
                "type": "ice",
                "track": "chase",
                "candidate": "candidate:1 1 UDP 1 127.0.0.1 9 typ host",
                "sdpMid": "0",
                "sdpMLineIndex": 0,
            }
        ),
    )
    ice = publisher.sent[-1]
    assert ice["type"] == "ice"
    assert ice["candidate"].startswith("candidate:")
    assert ice["sdpMid"] == "0"
    assert ice["sdpMLineIndex"] == 0
    assert ice["viewer"] == viewer.id


async def _in_process():
    class Answerer:
        def __init__(self):
            self.offers = []
            self.removed = []

        async def handle_offer(self, viewer_id, tracks, sdp):
            self.offers.append((viewer_id, tracks, sdp))
            return "v=0\r\nlocal-answer\r\n"

        async def handle_ice(self, viewer_id, track, candidate, sdp_mid, sdp_mline_index):
            del viewer_id, track, candidate, sdp_mid, sdp_mline_index

        async def remove_viewer(self, viewer_id):
            self.removed.append(viewer_id)

    config = parse_streamer_config({"stun_urls": []})
    answerer = Answerer()
    hub = SignalingHub(config, answerer)
    viewer = MemConn("v9")
    await hub.handle(
        viewer,
        json.dumps({"type": "join", "role": "viewer", "tracks": ["nope"]}),
    )
    assert viewer.sent[-1]["error"] == "unknown_track"

    await hub.handle(
        viewer,
        json.dumps({"type": "join", "role": "viewer", "tracks": ["front"]}),
    )
    assert viewer.sent[-1]["iceServers"] == []
    await hub.handle(
        viewer,
        json.dumps({"type": "offer", "sdp": "v=0\r\n", "track": "front"}),
    )
    assert viewer.sent[-1]["type"] == "answer"
    assert answerer.offers[0][1] == ("front",)
    await hub.disconnect(viewer)
    assert answerer.removed == [viewer.id]
