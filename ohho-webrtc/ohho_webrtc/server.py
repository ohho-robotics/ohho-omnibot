# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Loopback HTTP and WebSocket server.

OHH-196 forwards ``wss://<host>:8443/webrtc/signaling`` to this process with
the ``/webrtc`` prefix removed, so the socket here is ``/signaling``. The same
port serves the browser test page at ``/``. Point the gateway at it with
``ohho-gateway --webrtc http://127.0.0.1:<port>``.
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
from pathlib import Path

from ohho_webrtc import GATEWAY_SIGNALING_PATH, UPSTREAM_SIGNALING_PATH
from ohho_webrtc.config import StreamerConfig
from ohho_webrtc.encoder import EncoderChoice
from ohho_webrtc.signaling import (
    SignalingParseError,
    build_error,
    build_ice,
    build_session_description,
    build_tracks,
    build_welcome,
    parse_signaling_message,
)
from ohho_webrtc.tracks import TrackRegistry
from ohho_webrtc.wsproto import read_text_message, write_frame

LOG = logging.getLogger("ohho_webrtc.server")
_STATIC = Path(__file__).resolve().parent / "static" / "index.html"
_SIGNALING_PATHS = {UPSTREAM_SIGNALING_PATH, GATEWAY_SIGNALING_PATH}
_PAGE_PATHS = {"/", "/index.html", "/webrtc", "/webrtc/", "/webrtc/index.html"}
_HEALTH_PATHS = {"/health", "/webrtc/health"}


class Connection:
    def __init__(self, writer: asyncio.StreamWriter, conn_id: str):
        self.writer = writer
        self.id = conn_id
        self.role: str | None = None
        self.tracks: tuple[str, ...] = ()
        self._lock = asyncio.Lock()

    async def send_json(self, payload: dict) -> None:
        text = json.dumps(payload, separators=(",", ":"))
        async with self._lock:
            await write_frame(self.writer, 0x1, text.encode("utf-8"))


class SignalingHub:
    """Route viewer offers to an in-process answerer or to a publisher socket."""

    def __init__(self, config: StreamerConfig, answerer=None):
        self.config = config
        self.answerer = answerer
        self.known = {spec.name for spec in config.tracks}
        self.viewers: dict[str, Connection] = {}
        self.publisher: Connection | None = None
        self._ids = 0
        self._lock = asyncio.Lock()

    def next_id(self) -> str:
        self._ids += 1
        return f"v{self._ids}"

    async def handle(self, conn: Connection, raw: str) -> None:
        try:
            message = parse_signaling_message(raw)
        except SignalingParseError as exc:
            await conn.send_json(build_error(exc.code))
            return
        if conn.role is None:
            await self._join(conn, message)
            return
        if message.type == "list":
            await conn.send_json(build_tracks(self.config))
            return
        if message.type == "offer":
            await self._offer(conn, message)
            return
        if message.type == "answer":
            await self._answer(conn, message)
            return
        if message.type == "ice":
            await self._ice(conn, message)
            return
        await conn.send_json(build_error("unexpected_message"))

    async def disconnect(self, conn: Connection) -> None:
        async with self._lock:
            if conn.role == "viewer":
                self.viewers.pop(conn.id, None)
            elif conn.role == "publisher" and self.publisher is conn:
                self.publisher = None
        if conn.role == "viewer" and self.answerer is not None:
            await self.answerer.remove_viewer(conn.id)

    async def _join(self, conn: Connection, message) -> None:
        if message.type != "join":
            await conn.send_json(build_error("join_required"))
            return
        unknown = [name for name in message.tracks if name not in self.known]
        if unknown:
            await conn.send_json(build_error("unknown_track", track=unknown[0]))
            return
        if message.role == "publisher":
            if self.answerer is not None:
                await conn.send_json(build_error("publisher_internal"))
                return
            async with self._lock:
                if self.publisher is not None:
                    taken = True
                else:
                    taken = False
                    conn.role = "publisher"
                    conn.tracks = message.tracks
                    self.publisher = conn
            if taken:
                await conn.send_json(build_error("publisher_exists"))
                return
            await conn.send_json(build_welcome(self.config, "publisher", conn.id))
            return
        conn.role = "viewer"
        conn.tracks = message.tracks
        async with self._lock:
            self.viewers[conn.id] = conn
        await conn.send_json(build_welcome(self.config, "viewer", conn.id))

    async def _offer(self, conn: Connection, message) -> None:
        if conn.role != "viewer":
            await conn.send_json(build_error("offer_from_viewer_only"))
            return
        if any(name not in conn.tracks for name in message.tracks):
            await conn.send_json(build_error("unknown_track"))
            return
        if self.answerer is not None:
            try:
                answer = await self.answerer.handle_offer(conn.id, message.tracks, message.sdp)
            except Exception:
                LOG.warning("in-process answer failed for viewer %s", conn.id)
                await conn.send_json(build_error("media_unavailable"))
                return
            await conn.send_json(
                build_session_description("answer", answer, message.tracks, conn.id)
            )
            return
        publisher = self.publisher
        if publisher is None:
            await conn.send_json(build_error("publisher_unavailable"))
            return
        await publisher.send_json(
            build_session_description("offer", message.sdp, message.tracks, conn.id)
        )

    async def _answer(self, conn: Connection, message) -> None:
        if conn.role != "publisher":
            await conn.send_json(build_error("answer_from_publisher_only"))
            return
        if not message.viewer:
            await conn.send_json(build_error("unknown_viewer"))
            return
        target = self.viewers.get(message.viewer)
        if target is None:
            await conn.send_json(build_error("unknown_viewer"))
            return
        await target.send_json(
            build_session_description("answer", message.sdp, message.tracks, message.viewer)
        )

    async def _ice(self, conn: Connection, message) -> None:
        if message.track not in self.known:
            await conn.send_json(build_error("unknown_track"))
            return
        if conn.role == "viewer":
            if self.answerer is not None:
                await self.answerer.handle_ice(
                    conn.id,
                    message.track,
                    message.candidate,
                    message.sdp_mid,
                    message.sdp_mline_index,
                )
                return
            publisher = self.publisher
            if publisher is None:
                await conn.send_json(build_error("publisher_unavailable"))
                return
            await publisher.send_json(
                build_ice(
                    track=message.track,
                    candidate=message.candidate,
                    sdp_mid=message.sdp_mid,
                    sdp_mline_index=message.sdp_mline_index,
                    viewer=conn.id,
                )
            )
            return
        if not message.viewer:
            await conn.send_json(build_error("unknown_viewer"))
            return
        target = self.viewers.get(message.viewer)
        if target is None:
            await conn.send_json(build_error("unknown_viewer"))
            return
        await target.send_json(
            build_ice(
                track=message.track,
                candidate=message.candidate,
                sdp_mid=message.sdp_mid,
                sdp_mline_index=message.sdp_mline_index,
                viewer=message.viewer,
            )
        )


class SignalingServer:
    def __init__(self, server: asyncio.Server, port: int, hub: SignalingHub):
        self.server = server
        self.port = port
        self.hub = hub

    async def close(self) -> None:
        self.server.close()
        await self.server.wait_closed()


def health_payload(config: StreamerConfig, encoder: EncoderChoice) -> dict:
    from ohho_webrtc.encoder import gstreamer_pipeline

    return {
        "service": "ohho_webrtc",
        "signalingPath": UPSTREAM_SIGNALING_PATH,
        "gatewayPath": GATEWAY_SIGNALING_PATH,
        "tracks": [{"name": spec.name, "topic": spec.topic} for spec in config.tracks],
        "pipelineEncoder": encoder.pipeline_encoder,
        "aiortcCodec": encoder.aiortc_codec,
        "bframes": encoder.bframes,
        "gop": encoder.gop,
        "bitrateBps": encoder.bitrate_bps,
        "width": encoder.width,
        "height": encoder.height,
        "fps": encoder.fps,
        "pipeline": gstreamer_pipeline(encoder, config),
    }


def load_test_page() -> bytes:
    return _STATIC.read_bytes()


async def start_signaling_server(
    config: StreamerConfig,
    registry: TrackRegistry,
    encoder: EncoderChoice,
    answerer=None,
    host: str | None = None,
    port: int | None = None,
) -> SignalingServer:
    """Bind the loopback server. ``port`` 0 asks the OS for a free port."""
    del registry  # the answerer closes over the registry; the hub only routes
    hub = SignalingHub(config, answerer)
    page = load_test_page()
    health = json.dumps(health_payload(config, encoder)).encode("utf-8")
    bind_host = config.host if host is None else host
    bind_port = config.port if port is None else port

    async def on_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        await _handle_client(reader, writer, hub, page, health)

    server = await asyncio.start_server(on_client, bind_host, bind_port)
    sockets = server.sockets or []
    bound = sockets[0].getsockname()[1] if sockets else bind_port
    return SignalingServer(server, int(bound), hub)


async def _handle_client(reader, writer, hub: SignalingHub, page: bytes, health: bytes) -> None:
    try:
        blob = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), timeout=10)
    except (asyncio.TimeoutError, asyncio.IncompleteReadError, ConnectionError):
        writer.close()
        return
    try:
        head = blob.decode("iso-8859-1")
        lines = head.split("\r\n")
        method, target, _version = lines[0].split(" ", 2)
        headers = {}
        for line in lines[1:]:
            if ":" in line:
                key, value = line.split(":", 1)
                headers[key.strip().lower()] = value.strip()
        path = _normalize_path(target)
        if method != "GET":
            await _http(writer, 405, b"method not allowed\n", "text/plain")
            return
        if path in _PAGE_PATHS:
            await _http(writer, 200, page, "text/html; charset=utf-8")
            return
        if path in _HEALTH_PATHS:
            await _http(writer, 200, health, "application/json")
            return
        if path not in _SIGNALING_PATHS:
            await _http(writer, 404, b"not found\n", "text/plain")
            return
        upgrade = headers.get("upgrade", "")
        if upgrade.lower() != "websocket":
            await _http(writer, 426, b"websocket required\n", "text/plain")
            return
        key = headers.get("sec-websocket-key", "")
        if not key:
            await _http(writer, 400, b"missing websocket key\n", "text/plain")
            return
        from ohho_webrtc.wsproto import accept_key

        accept = accept_key(key)
        writer.write(
            (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept}\r\n"
                "\r\n"
            ).encode("ascii")
        )
        await writer.drain()
        conn = Connection(writer, hub.next_id())
        try:
            while True:
                text = await read_text_message(reader, writer)
                if text is None:
                    break
                await hub.handle(conn, text)
        finally:
            await hub.disconnect(conn)
    except Exception:
        LOG.debug("signaling client closed", exc_info=True)
    finally:
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            return


def _normalize_path(target: str) -> str:
    path = target.split("?", 1)[0]
    if path != "/" and path.endswith("/"):
        path = path[:-1]
    if path == "":
        return "/"
    return path


async def _http(writer, status: int, body: bytes, content_type: str) -> None:
    reason = {200: "OK", 400: "Bad Request", 404: "Not Found", 405: "Method Not Allowed", 426: "Upgrade Required"}[
        status
    ]
    writer.write(
        (
            f"HTTP/1.1 {status} {reason}\r\n"
            f"Content-Type: {content_type}\r\n"
            f"Content-Length: {len(body)}\r\n"
            "Cache-Control: no-store\r\n"
            "Connection: close\r\n"
            "\r\n"
        ).encode("ascii")
        + body
    )
    await writer.drain()


class ServerThread:
    """Run the signaling server on a background thread (ROS spin stays on main)."""

    def __init__(
        self,
        config: StreamerConfig,
        registry: TrackRegistry,
        encoder: EncoderChoice,
        answerer_factory=None,
    ):
        self.config = config
        self.registry = registry
        self.encoder = encoder
        self.answerer_factory = answerer_factory
        self.port = 0
        self.ready = threading.Event()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop: asyncio.Event | None = None
        self._thread = threading.Thread(target=self._run, name="ohho-webrtc", daemon=True)
        self._error: BaseException | None = None

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        if self._loop is not None and self._stop is not None:
            self._loop.call_soon_threadsafe(self._stop.set)
        self._thread.join(timeout=5)

    def _run(self) -> None:
        try:
            asyncio.run(self._amain())
        except BaseException as exc:
            self._error = exc
            self.ready.set()

    async def _amain(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._stop = asyncio.Event()
        answerer = None
        if self.answerer_factory is not None:
            answerer = self.answerer_factory(self.config, self.registry, self.encoder)
        server = await start_signaling_server(
            self.config,
            self.registry,
            self.encoder,
            answerer=answerer,
        )
        self.port = server.port
        self.ready.set()
        await self._stop.wait()
        await server.close()
