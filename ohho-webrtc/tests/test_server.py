# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
import asyncio
import json
import os
import sys
import time
import urllib.request

from ohho_webrtc.config import parse_streamer_config
from ohho_webrtc.encoder import EncoderSupport, select_encoder
from ohho_webrtc.node import PatternPump
from ohho_webrtc.server import ServerThread, start_signaling_server
from ohho_webrtc.tracks import TrackRegistry
from ohho_webrtc.wsproto import (
    SAMPLE_ACCEPT,
    SAMPLE_KEY,
    accept_key,
    client_send_text,
    encode_frame,
    open_signaling,
    read_frame,
    read_text_message,
)


def test_websocket_accept_matches_the_rfc_sample():
    assert accept_key(SAMPLE_KEY) == SAMPLE_ACCEPT


def test_node_module_does_not_import_rclpy():
    sys.modules.pop("rclpy", None)
    import ohho_webrtc.node as node

    assert "rclpy" not in sys.modules
    assert callable(node.main)


def test_pattern_pump_fills_every_track():
    config = parse_streamer_config(
        {"width": 16, "height": 16, "fps": 20, "bitrate_bps": 2_000_000, "test_pattern": True}
    )
    registry = TrackRegistry(config.tracks)
    pump = PatternPump(registry, config)
    pump.start()
    deadline = time.time() + 2
    while time.time() < deadline and not registry.has_frame("wrist"):
        time.sleep(0.05)
    pump.stop()
    assert registry.has_frame("chase")
    assert registry.has_frame("front")
    assert registry.has_frame("wrist")


def test_http_page_health_and_signaling_socket():
    asyncio.run(_http_and_socket())


def test_background_thread_serves_health():
    config = parse_streamer_config(
        {"signaling_port": 0, "width": 64, "height": 64, "bitrate_bps": 2_000_000}
    )
    registry = TrackRegistry(config.tracks)
    encoder = select_encoder(config, EncoderSupport())
    thread = ServerThread(config, registry, encoder, answerer_factory=lambda *_args: None)
    thread.start()
    assert thread.ready.wait(5)
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{thread.port}/health", timeout=2) as resp:
            body = json.loads(resp.read().decode("utf-8"))
        assert body["service"] == "ohho_webrtc"
        assert body["bframes"] == 0
        assert body["gatewayPath"] == "/webrtc/signaling"
        assert body["signalingPath"] == "/signaling"
        assert body["gop"] == 15
    finally:
        thread.stop()


class _Answerer:
    async def handle_offer(self, viewer_id, tracks, sdp):
        del viewer_id, tracks
        assert sdp.startswith("v=")
        return "v=0\r\nanswer\r\n"

    async def handle_ice(self, viewer_id, track, candidate, sdp_mid, sdp_mline_index):
        del viewer_id, track, candidate, sdp_mid, sdp_mline_index

    async def remove_viewer(self, viewer_id):
        del viewer_id


async def _http_and_socket():
    config = parse_streamer_config({"stun_urls": []})
    registry = TrackRegistry(config.tracks)
    encoder = select_encoder(config, EncoderSupport())
    server = await start_signaling_server(
        config,
        registry,
        encoder,
        answerer=_Answerer(),
        host="127.0.0.1",
        port=0,
    )
    try:
        page = await _get(server.port, "/?token=super-secret-token")
        assert b"<video" in page
        assert b"/webrtc/signaling" in page
        assert b"super-secret-token" not in page
        assert b"120 ms" in page
        health = await _get(server.port, "/health")
        payload = json.loads(health.split(b"\r\n\r\n", 1)[1])
        assert payload["bframes"] == 0
        assert "bframes=0" in payload["pipeline"] or "profile=baseline" in payload["pipeline"]

        reader, writer = await open_signaling(f"ws://127.0.0.1:{server.port}/signaling")
        try:
            writer.write(encode_frame(0x9, b"ohho", os.urandom(4)))
            await writer.drain()
            _fin, opcode, payload = await read_frame(reader)
            assert opcode == 0xA
            assert payload == b"ohho"
            await client_send_text(
                writer,
                json.dumps({"type": "join", "role": "viewer", "tracks": ["chase"]}),
            )
            welcome = json.loads(await read_text_message(reader, writer))
            assert welcome["type"] == "welcome"
            assert welcome["gatewayPath"] == "/webrtc/signaling"
            assert welcome["signalingPath"] == "/signaling"
            assert welcome["iceServers"] == []
            await client_send_text(
                writer,
                json.dumps({"type": "offer", "sdp": "v=0\r\n", "track": "chase"}),
            )
            answer = json.loads(await read_text_message(reader, writer))
            assert answer["type"] == "answer"
            assert answer["track"] == "chase"
            assert answer["sdp"].startswith("v=")
        finally:
            writer.close()
            await writer.wait_closed()

        direct, direct_writer = await open_signaling(
            f"ws://127.0.0.1:{server.port}/webrtc/signaling"
        )
        direct_writer.close()
        await direct_writer.wait_closed()
        del direct
    finally:
        await server.close()


async def _get(port: int, path: str) -> bytes:
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    writer.write(
        f"GET {path} HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n".encode("ascii")
    )
    await writer.drain()
    body = await reader.read()
    writer.close()
    return body
