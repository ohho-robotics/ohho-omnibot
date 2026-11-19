# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0

import asyncio
import logging
import socket
from contextlib import asynccontextmanager

import aiohttp
import pytest
from aiohttp import web
from multidict import CIMultiDict

from ohho_gateway.cli import open_sites, prepare
from ohho_gateway.server import CLIENT_KEY
from ohho_gateway.proxy import (
    build_upstream_url,
    forward_request_headers,
    forward_response_headers,
    safe_upstream_path,
    strip_prefix,
    upstream_query,
)
from ohho_gateway.tls import client_ssl_context

TOKEN = "ohho-test-token-0123456789"
WRONG = "ohho-test-token-wrong-value"
WRONG_SAME_LENGTH = "ohho-test-token-0123456788"


def test_upstream_query_drops_the_token():
    query = upstream_query([("topic", "/camera/front/image_raw"), ("token", TOKEN)])
    assert query == [("topic", "/camera/front/image_raw")]
    url = build_upstream_url("http://127.0.0.1:8080", "/stream", query)
    assert TOKEN not in url
    assert "topic=/camera/front/image_raw" in url


def test_build_upstream_url_reencodes_a_decoded_path():
    assert (
        build_upstream_url("http://127.0.0.1:8080", "/a?b", [])
        == "http://127.0.0.1:8080/a%3Fb"
    )
    assert (
        build_upstream_url("http://127.0.0.1:8080", "/a#b", [])
        == "http://127.0.0.1:8080/a%23b"
    )
    assert (
        build_upstream_url("http://127.0.0.1:8080", "/a%b", [])
        == "http://127.0.0.1:8080/a%25b"
    )
    assert (
        build_upstream_url("http://127.0.0.1:8080", "/a b", [("topic", "x")])
        == "http://127.0.0.1:8080/a%20b?topic=x"
    )


def test_connection_nominated_headers_are_dropped_and_duplicates_kept():
    request_headers = CIMultiDict()
    request_headers.add("Connection", "close, X-Foo")
    request_headers.add("X-Foo", "secret")
    request_headers.add("X-Keep", "one")
    request_headers.add("X-Keep", "two")
    forwarded = forward_request_headers(request_headers)
    assert "x-foo" not in {key.lower() for key in forwarded}
    assert forwarded.getall("X-Keep") == ["one", "two"]

    response_headers = CIMultiDict()
    response_headers.add("Connection", "X-Bar")
    response_headers.add("X-Bar", "drop-me")
    response_headers.add("Set-Cookie", "a=1")
    response_headers.add("Set-Cookie", "b=2")
    response_headers.add("Proxy-Authenticate", "Basic realm=upstream")
    forwarded_response = forward_response_headers(response_headers)
    lowered = {key.lower() for key in forwarded_response}
    assert "x-bar" not in lowered
    assert "proxy-authenticate" not in lowered
    assert forwarded_response.getall("Set-Cookie") == ["a=1", "b=2"]
    assert forwarded_response["Cache-Control"] == "no-store"


def test_safe_path_rejects_parent_segments():
    assert strip_prefix("/video/stream", "/video") == "/stream"
    assert safe_upstream_path("/stream") == "/stream"
    with pytest.raises(ValueError):
        safe_upstream_path("/video/../../etc/passwd")


def _ws_app(name: str, seen: list):
    app = web.Application()

    async def handler(request):
        seen.append(
            {
                "which": name,
                "path": request.path,
                "query": dict(request.query),
                "authorization": request.headers.get("Authorization"),
            }
        )
        socket_ = web.WebSocketResponse()
        await socket_.prepare(request)
        async for message in socket_:
            if message.type == aiohttp.WSMsgType.TEXT:
                await socket_.send_str(f"{name}:{message.data}")
            elif message.type == aiohttp.WSMsgType.BINARY:
                await socket_.send_bytes(name.encode() + b":" + message.data)
            else:
                break
        return socket_

    app.router.add_get("/", handler)
    app.router.add_get("/{tail:.*}", handler)
    return app


def _http_app(name: str, seen: list):
    app = web.Application()

    async def handler(request):
        seen.append(
            {
                "which": name,
                "path": request.path,
                "query": dict(request.query),
                "authorization": request.headers.get("Authorization"),
            }
        )
        if request.path == "/chunks":
            response = web.StreamResponse(
                status=200,
                headers={"Content-Type": "text/plain"},
            )
            await response.prepare(request)
            await response.write(b"frame-a")
            await response.write(b"frame-b")
            await response.write_eof()
            return response
        if request.path == "/cut":
            response = web.StreamResponse(
                status=200,
                headers={"Content-Type": "application/octet-stream"},
            )
            await response.prepare(request)
            await response.write(b"partial-body")
            transport = request.transport
            if transport is not None:
                transport.close()
            return response
        if request.path == "/gzip-raw":
            return web.Response(
                body=b"not-gzip-body",
                headers={
                    "Content-Encoding": "gzip",
                    "Content-Type": "application/octet-stream",
                },
            )
        return web.Response(text=f"{name}-ok")

    app.router.add_route("*", "/", handler)
    app.router.add_route("*", "/{tail:.*}", handler)
    return app


async def _listen(app: web.Application, runners: list) -> int:
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", 0)
    await site.start()
    runners.append(runner)
    return site._server.sockets[0].getsockname()[1]


@asynccontextmanager
async def _stack(tmp_path, *, token=TOKEN, webrtc=False, tls_only=False, plain_host="127.0.0.1"):
    seen: list = []
    runners: list = []
    listening = None
    session = None
    try:
        video_port = await _listen(_http_app("video", seen), runners)
        ros_port = await _listen(_ws_app("ros", seen), runners)
        fox_port = await _listen(_ws_app("fox", seen), runners)
        webrtc_port = await _listen(_http_app("webrtc", seen), runners)
        token_file = tmp_path / "token"
        token_file.write_text(token + "\n", encoding="utf-8")
        argv = [
            "--state-dir",
            str(tmp_path / "state"),
            "--bind",
            "127.0.0.1",
            "--port",
            "0",
            "--plain-host",
            plain_host,
            "--plain-port",
            "0",
            "--token-file",
            str(token_file),
            "--rosbridge",
            f"ws://127.0.0.1:{ros_port}",
            "--video",
            f"http://127.0.0.1:{video_port}",
            "--foxglove",
            f"ws://127.0.0.1:{fox_port}",
        ]
        if tls_only:
            argv.append("--tls-only")
        if webrtc:
            argv.extend(["--webrtc", f"http://127.0.0.1:{webrtc_port}"])
        prepared = prepare(argv, env={})
        assert token not in prepared.banner
        listening = await open_sites(prepared)
        session = aiohttp.ClientSession()
        tls_port = listening.tls_address[1]
        yield {
            "seen": seen,
            "prepared": prepared,
            "listening": listening,
            "session": session,
            "tls": f"https://127.0.0.1:{tls_port}",
            "ssl": client_ssl_context(prepared.material),
            "plain": (
                None
                if listening.plain_address is None
                else f"http://127.0.0.1:{listening.plain_address[1]}"
            ),
        }
    finally:
        if session is not None:
            await session.close()
        if listening is not None:
            await listening.runner.cleanup()
        for runner in runners:
            await runner.cleanup()


def _run(coro):
    return asyncio.run(coro)


def test_missing_and_wrong_tokens_are_401_and_do_not_reach_upstream(tmp_path):
    async def body():
        async with _stack(tmp_path) as gateway:
            session = gateway["session"]
            ssl = gateway["ssl"]
            base = gateway["tls"]
            before = len(gateway["seen"])

            missing = await session.get(f"{base}/video/stream?topic=/cam", ssl=ssl)
            missing_text = await missing.text()
            assert missing.status == 401
            assert missing_text == "unauthorized\n"
            assert missing.headers["WWW-Authenticate"].startswith("Bearer ")
            assert missing.headers["Cache-Control"] == "no-store"
            assert TOKEN not in missing_text

            wrong = await session.get(
                f"{base}/video/stream",
                headers={"Authorization": f"Bearer {WRONG}"},
                ssl=ssl,
            )
            assert wrong.status == 401
            await wrong.release()

            same_length = await session.get(
                f"{base}/",
                headers={"Authorization": f"Bearer {WRONG_SAME_LENGTH}"},
                ssl=ssl,
            )
            assert same_length.status == 401
            await same_length.release()

            mismatch = await session.get(
                f"{base}/video/stream?token={TOKEN}",
                headers={"Authorization": f"Bearer {WRONG_SAME_LENGTH}"},
                ssl=ssl,
            )
            assert mismatch.status == 401
            await mismatch.release()

            try:
                await session.ws_connect(f"{base}/rosbridge", ssl=ssl)
                raise AssertionError("websocket without a token was accepted")
            except aiohttp.WSServerHandshakeError as exc:
                assert exc.status == 401

            try:
                await session.ws_connect(
                    f"{base}/rosbridge",
                    headers={"Authorization": f"Bearer {WRONG}"},
                    ssl=ssl,
                )
                raise AssertionError("websocket with a wrong token was accepted")
            except aiohttp.WSServerHandshakeError as exc:
                assert exc.status == 401

            assert gateway["seen"] == []
            assert len(gateway["seen"]) == before

            plain = await session.get(f"{gateway['plain']}/video/stream", ssl=False)
            assert plain.status == 401
            await plain.release()
            assert gateway["seen"] == []

    _run(body())


def test_valid_token_proxies_http_and_websocket_without_forwarding_the_token(tmp_path, caplog):
    caplog.set_level(logging.INFO)

    async def body():
        async with _stack(tmp_path, webrtc=True) as gateway:
            session = gateway["session"]
            ssl = gateway["ssl"]
            base = gateway["tls"]
            header = {"Authorization": f"Bearer {TOKEN}"}

            video = await session.get(
                f"{base}/video/stream?topic=/camera/front/image_raw&type=mjpeg&token={TOKEN}",
                ssl=ssl,
            )
            video_text = await video.text()
            assert video.status == 200
            assert video_text == "video-ok"
            assert video.headers["Cache-Control"] == "no-store"
            assert video.headers["Referrer-Policy"] == "no-referrer"
            assert TOKEN not in video_text
            video_hit = gateway["seen"][-1]
            assert video_hit["which"] == "video"
            assert video_hit["path"] == "/stream"
            assert video_hit["query"].get("topic") == "/camera/front/image_raw"
            assert "token" not in video_hit["query"]
            assert video_hit["authorization"] is None

            bearer_only = await session.get(
                f"{base}/video/stream?topic=/camera/wrist/image_raw",
                headers=header,
                ssl=ssl,
            )
            assert bearer_only.status == 200
            await bearer_only.release()
            assert gateway["seen"][-1]["authorization"] is None

            chunks = await session.get(f"{base}/video/chunks?token={TOKEN}", ssl=ssl)
            assert await chunks.read() == b"frame-aframe-b"

            async with session.ws_connect(
                f"{base}/rosbridge?token={TOKEN}",
                ssl=ssl,
            ) as ros:
                await ros.send_str("ping")
                assert await ros.receive_str() == "ros:ping"
                await ros.send_bytes(b"abc")
                binary = await ros.receive()
                assert binary.data == b"ros:abc"
            ros_hit = next(item for item in gateway["seen"] if item["which"] == "ros")
            assert ros_hit["path"] == "/"
            assert "token" not in ros_hit["query"]
            assert ros_hit["authorization"] is None

            async with session.ws_connect(
                f"{base}/foxglove",
                headers=header,
                ssl=ssl,
            ) as fox:
                await fox.send_str("ping")
                assert await fox.receive_str() == "fox:ping"

            reserved = await session.get(
                f"{base}/webrtc/signaling",
                headers=header,
                ssl=ssl,
            )
            # This stack enables the upstream, so the reserved path proxies.
            assert reserved.status == 200
            assert await reserved.text() == "webrtc-ok"
            webrtc_hit = gateway["seen"][-1]
            assert webrtc_hit["which"] == "webrtc"
            assert webrtc_hit["path"] == "/signaling"
            assert "token" not in webrtc_hit["query"]

            root = await session.get(f"{base}/?token={TOKEN}", ssl=ssl)
            root_body = await root.json()
            assert root.status == 200
            assert root_body["rosbridge"] == "/rosbridge"
            assert root_body["webrtc"] == "/webrtc/signaling"
            assert TOKEN not in str(root_body)

            missing_route = await session.get(f"{base}/no-such-path", headers=header, ssl=ssl)
            assert missing_route.status == 404
            await missing_route.release()

            not_ws = await session.get(f"{base}/rosbridge", headers=header, ssl=ssl)
            assert not_ws.status == 426
            await not_ws.release()

            assert gateway["listening"].runner.app[CLIENT_KEY]._trust_env is False
            assert gateway["listening"].runner.app[CLIENT_KEY].auto_decompress is False
            assert gateway["listening"].plain_address[0] == "127.0.0.1"

    _run(body())
    assert TOKEN not in caplog.text
    assert "GET /video/stream -> 200" in caplog.text


def test_webrtc_path_is_503_until_an_upstream_is_configured(tmp_path):
    async def body():
        async with _stack(tmp_path, webrtc=False) as gateway:
            response = await gateway["session"].get(
                f"{gateway['tls']}/webrtc/signaling?token={TOKEN}",
                ssl=gateway["ssl"],
            )
            body_json = await response.json()
            assert response.status == 503
            assert body_json["issue"] == "OHH-210"
            assert TOKEN not in str(body_json)

            hidden = await gateway["session"].get(
                f"{gateway['tls']}/webrtc/signaling",
                ssl=gateway["ssl"],
            )
            assert hidden.status == 401
            await hidden.release()
            assert gateway["seen"] == []

    _run(body())


def test_tls_only_has_no_plain_listener_and_default_trust_is_rejected(tmp_path):
    async def body():
        async with _stack(tmp_path, tls_only=True) as gateway:
            assert gateway["listening"].plain_address is None
            bare = aiohttp.ClientSession()
            try:
                with pytest.raises(aiohttp.ClientConnectorCertificateError):
                    await bare.get(f"{gateway['tls']}/")
            finally:
                await bare.close()
            authed = await gateway["session"].get(
                f"{gateway['tls']}/?token={TOKEN}",
                ssl=gateway["ssl"],
            )
            assert authed.status == 200
            await authed.release()

    _run(body())


def test_encoded_question_mark_stays_in_the_upstream_path(tmp_path):
    async def body():
        async with _stack(tmp_path) as gateway:
            session = gateway["session"]
            question = await session.get(
                f"{gateway['tls']}/video/a%3Fb?token={TOKEN}",
                ssl=gateway["ssl"],
            )
            assert question.status == 200
            await question.release()
            question_hit = gateway["seen"][-1]
            assert question_hit["path"] == "/a?b"
            assert question_hit["query"] == {}

            fragment = await session.get(
                f"{gateway['tls']}/video/a%23b?token={TOKEN}",
                ssl=gateway["ssl"],
            )
            assert fragment.status == 200
            await fragment.release()
            assert gateway["seen"][-1]["path"] == "/a#b"
            assert gateway["seen"][-1]["query"] == {}

    _run(body())


def test_truncated_upstream_is_not_a_completed_body(tmp_path, caplog):
    caplog.set_level(logging.DEBUG)

    async def body():
        async with _stack(tmp_path) as gateway:
            session = gateway["session"]
            response = await session.get(
                f"{gateway['tls']}/video/cut?token={TOKEN}",
                ssl=gateway["ssl"],
            )
            assert response.status == 200
            with pytest.raises(aiohttp.ClientError):
                await response.read()

    _run(body())
    assert "upstream HTTP payload failed" in caplog.text
    assert "upstream unavailable" not in caplog.text


def test_compressed_upstream_bytes_are_forwarded_unchanged(tmp_path):
    async def body():
        async with _stack(tmp_path) as gateway:
            response = await gateway["session"].get(
                f"{gateway['tls']}/video/gzip-raw?token={TOKEN}",
                ssl=gateway["ssl"],
                auto_decompress=False,
            )
            payload = await response.read()
            assert response.status == 200
            assert response.headers["Content-Encoding"] == "gzip"
            assert payload == b"not-gzip-body"

    _run(body())


def test_upstream_outage_is_502(tmp_path):
    async def body():
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        closed_port = probe.getsockname()[1]
        probe.close()
        token_file = tmp_path / "token"
        token_file.write_text(TOKEN + "\n", encoding="utf-8")
        prepared = prepare(
            [
                "--state-dir",
                str(tmp_path / "state"),
                "--bind",
                "127.0.0.1",
                "--port",
                "0",
                "--tls-only",
                "--token-file",
                str(token_file),
                "--video",
                f"http://127.0.0.1:{closed_port}",
                "--rosbridge",
                f"ws://127.0.0.1:{closed_port}",
                "--foxglove",
                f"ws://127.0.0.1:{closed_port}",
            ],
            env={},
        )
        listening = await open_sites(prepared)
        session = aiohttp.ClientSession()
        try:
            response = await session.get(
                f"https://127.0.0.1:{listening.tls_address[1]}/video/stream?token={TOKEN}",
                ssl=client_ssl_context(prepared.material),
            )
            text = await response.text()
            assert response.status == 502
            assert text == "upstream unavailable\n"
            assert TOKEN not in text
        finally:
            await session.close()
            await listening.runner.cleanup()

    _run(body())


def test_request_log_redacts_a_pairing_code(tmp_path, caplog):
    caplog.set_level(logging.INFO)

    async def body():
        prepared = prepare(
            ["--state-dir", str(tmp_path / "state"), "--tls-only", "--port", "0"],
            env={},
        )
        assert prepared.source == "generated"
        assert prepared.token in prepared.banner
        listening = await open_sites(prepared)
        session = aiohttp.ClientSession()
        try:
            logging.getLogger("ohho_gateway").info("operator banner %s", prepared.banner)
            logging.getLogger("ohho_gateway.access").info(
                "GET /video/stream?token=%s -> 200",
                prepared.token,
            )
            response = await session.get(
                f"https://127.0.0.1:{listening.tls_address[1]}/?token={prepared.token}",
                ssl=client_ssl_context(prepared.material),
            )
            assert response.status == 200
            await response.release()
            return prepared.token
        finally:
            await session.close()
            await listening.runner.cleanup()

    token = _run(body())
    assert token not in caplog.text
    assert "token=[redacted]" in caplog.text or "[redacted]" in caplog.text
