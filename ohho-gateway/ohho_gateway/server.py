# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""aiohttp application: authenticate, then proxy HTTP and WebSocket."""

from __future__ import annotations

import asyncio
import logging

import aiohttp
from aiohttp import WSMsgType, web

from ohho_gateway.auth import presented_token, tokens_equal
from ohho_gateway.proxy import (
    TOKEN_QUERY_PARAM,
    build_upstream_url,
    forward_request_headers,
    forward_response_headers,
    http_base_to_ws,
    safe_upstream_path,
    strip_prefix,
    upstream_query,
    ws_base_to_http,
)
from ohho_gateway.redact import install_redaction, redact_text, register_secret

ACCESS_LOG = logging.getLogger("ohho_gateway.access")
ERROR_LOG = logging.getLogger("ohho_gateway")
MAX_MESSAGE_BYTES = 8 * 1024 * 1024

TOKEN_KEY: web.AppKey[str] = web.AppKey("token", str)
ROSBRIDGE_KEY: web.AppKey[str] = web.AppKey("rosbridge_url", str)
VIDEO_KEY: web.AppKey[str] = web.AppKey("video_url", str)
FOXGLOVE_KEY: web.AppKey[str] = web.AppKey("foxglove_url", str)
WEBRTC_KEY: web.AppKey[str] = web.AppKey("webrtc_url", str)
CLIENT_KEY: web.AppKey[aiohttp.ClientSession] = web.AppKey("client", aiohttp.ClientSession)

_UNAUTHORIZED_HEADERS = {
    "WWW-Authenticate": 'Bearer realm="ohho-gateway"',
    "Cache-Control": "no-store",
    "Referrer-Policy": "no-referrer",
}

_ROOT = {
    "service": "ohho-gateway",
    "rosbridge": "/rosbridge",
    "video": "/video/",
    "foxglove": "/foxglove",
    "webrtc": "/webrtc/signaling",
}

_WEBRTC_RESERVED = {
    "error": "webrtc_signaling_unavailable",
    "issue": "OHH-210",
}


def _unauthorized() -> web.Response:
    return web.Response(status=401, text="unauthorized\n", headers=_UNAUTHORIZED_HEADERS)


def _query_items(request: web.Request) -> list[tuple[str, str]]:
    return [(key, value) for key, value in request.query.items()]


@web.middleware
async def auth_middleware(request: web.Request, handler):
    expected = request.app[TOKEN_KEY]
    query_token = request.query.get(TOKEN_QUERY_PARAM)
    provided = presented_token(request.headers.get("Authorization"), query_token)
    if provided is None or not tokens_equal(provided, expected):
        return _unauthorized()
    return await handler(request)


@web.middleware
async def access_middleware(request: web.Request, handler):
    response = await handler(request)
    status = getattr(response, "status", 0)
    path = redact_text(request.path)
    ACCESS_LOG.info("%s %s -> %s", request.method, path, status)
    return response


async def _startup(app: web.Application) -> None:
    timeout = aiohttp.ClientTimeout(total=None, sock_connect=10, sock_read=None)
    # Leave Content-Encoding and the body bytes untouched. Decompressing here
    # would forward a plain body with the upstream Content-Encoding still set.
    app[CLIENT_KEY] = aiohttp.ClientSession(
        timeout=timeout,
        trust_env=False,
        auto_decompress=False,
    )


async def _cleanup(app: web.Application) -> None:
    client = app.get(CLIENT_KEY)
    if client is not None:
        await client.close()


def _client(request: web.Request) -> aiohttp.ClientSession:
    return request.app[CLIENT_KEY]


def _abort_started_response(request: web.Request) -> None:
    """Drop the client socket so a partial body is not ended with a chunk terminator."""
    transport = request.transport
    if transport is not None and not transport.is_closing():
        transport.close()


async def _write_upstream_body(
    request: web.Request,
    response: web.StreamResponse,
    upstream: aiohttp.ClientResponse,
) -> None:
    """Copy the upstream body. Failures here must not send a second status line.

    aiohttp finishes a returned ``StreamResponse`` with ``write_eof()``, which
    emits a chunked terminator. Closing the transport first makes that write
    fail, so a truncated body does not look complete. A client disconnect
    during ``write`` is ``ConnectionError`` (``ClientConnectionResetError`` on
    aiohttp 3.10+, plain ``ConnectionResetError`` on 3.9).
    """
    try:
        async for chunk in upstream.content.iter_chunked(16384):
            try:
                await response.write(chunk)
            except asyncio.CancelledError:
                raise
            except ConnectionError as exc:
                ERROR_LOG.debug(
                    "client disconnected during HTTP proxy (%s)",
                    type(exc).__name__,
                )
                _abort_started_response(request)
                return
        await response.write_eof()
    except asyncio.CancelledError:
        raise
    except ConnectionError as exc:
        ERROR_LOG.debug(
            "client disconnected during HTTP proxy (%s)",
            type(exc).__name__,
        )
        _abort_started_response(request)
    except aiohttp.ClientError as exc:
        ERROR_LOG.warning(
            "upstream HTTP payload failed (%s)",
            type(exc).__name__,
        )
        _abort_started_response(request)


async def _proxy_http(request: web.Request, base: str, prefix: str) -> web.StreamResponse:
    try:
        relative = safe_upstream_path(strip_prefix(request.rel_url.path, prefix))
    except ValueError:
        return web.Response(status=400, text="bad path\n", headers=_UNAUTHORIZED_HEADERS)
    url = build_upstream_url(base, relative, upstream_query(_query_items(request)))
    body = await request.read()
    session = _client(request)
    try:
        async with session.request(
            request.method,
            url,
            data=body if body else None,
            headers=forward_request_headers(request.headers),
            allow_redirects=False,
        ) as upstream:
            response = web.StreamResponse(
                status=upstream.status,
                headers=forward_response_headers(upstream.headers),
            )
            try:
                await response.prepare(request)
            except asyncio.CancelledError:
                raise
            except (aiohttp.ClientError, ConnectionError) as exc:
                if response.prepared:
                    ERROR_LOG.debug(
                        "client disconnected before HTTP proxy body (%s)",
                        type(exc).__name__,
                    )
                    _abort_started_response(request)
                    return response
                raise
            await _write_upstream_body(request, response, upstream)
            return response
    except aiohttp.ClientError as exc:
        ERROR_LOG.warning("upstream HTTP request failed (%s)", type(exc).__name__)
        return web.Response(status=502, text="upstream unavailable\n")


def _ws_protocols(request: web.Request) -> list[str]:
    raw = request.headers.get("Sec-WebSocket-Protocol", "")
    return [part.strip() for part in raw.split(",") if part.strip()]


async def _pump(source: web.WebSocketResponse, dest: web.WebSocketResponse) -> None:
    async for message in source:
        if message.type == WSMsgType.TEXT:
            await dest.send_str(message.data)
        elif message.type == WSMsgType.BINARY:
            await dest.send_bytes(message.data)
        elif message.type == WSMsgType.PING:
            await dest.ping(message.data)
        elif message.type == WSMsgType.PONG:
            await dest.pong(message.data)
        else:
            break


async def _proxy_ws(request: web.Request, base: str) -> web.StreamResponse:
    if request.headers.get("Upgrade", "").lower() != "websocket":
        return web.Response(status=426, text="websocket required\n", headers=_UNAUTHORIZED_HEADERS)
    try:
        relative = safe_upstream_path(strip_prefix(request.rel_url.path, _ws_prefix(request)))
    except ValueError:
        return web.Response(status=400, text="bad path\n", headers=_UNAUTHORIZED_HEADERS)
    client_ws = web.WebSocketResponse(heartbeat=30.0, max_msg_size=MAX_MESSAGE_BYTES)
    await client_ws.prepare(request)
    url = build_upstream_url(
        http_base_to_ws(base),
        relative,
        upstream_query(_query_items(request)),
    )
    session = _client(request)
    try:
        upstream_ws = await session.ws_connect(
            url,
            protocols=_ws_protocols(request),
            heartbeat=30.0,
            max_msg_size=MAX_MESSAGE_BYTES,
        )
    except aiohttp.ClientError:
        ERROR_LOG.warning("upstream WebSocket connection failed")
        await client_ws.close(code=aiohttp.WSCloseCode.INTERNAL_ERROR)
        return client_ws
    client_task = asyncio.create_task(_pump(client_ws, upstream_ws))
    upstream_task = asyncio.create_task(_pump(upstream_ws, client_ws))
    try:
        done, pending = await asyncio.wait(
            {client_task, upstream_task},
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in pending:
            task.cancel()
        await asyncio.gather(*pending, return_exceptions=True)
        await asyncio.gather(*done, return_exceptions=True)
    finally:
        await upstream_ws.close()
        await client_ws.close()
    return client_ws


def _ws_prefix(request: web.Request) -> str:
    path = request.rel_url.path
    for prefix in ("/rosbridge", "/foxglove", "/webrtc"):
        if path == prefix or path.startswith(prefix + "/"):
            return prefix
    return ""


async def handle_root(request: web.Request) -> web.Response:
    if request.method != "GET":
        return web.Response(status=405, text="method not allowed\n")
    return web.json_response(_ROOT, headers={"Cache-Control": "no-store"})


async def handle_video(request: web.Request) -> web.StreamResponse:
    return await _proxy_http(request, request.app[VIDEO_KEY], "/video")


async def handle_rosbridge(request: web.Request) -> web.StreamResponse:
    return await _proxy_ws(request, request.app[ROSBRIDGE_KEY])


async def handle_foxglove(request: web.Request) -> web.StreamResponse:
    return await _proxy_ws(request, request.app[FOXGLOVE_KEY])


async def handle_webrtc(request: web.Request) -> web.StreamResponse:
    upstream = request.app[WEBRTC_KEY]
    if not upstream:
        return web.json_response(
            _WEBRTC_RESERVED,
            status=503,
            headers={"Cache-Control": "no-store"},
        )
    if request.headers.get("Upgrade", "").lower() == "websocket":
        ws_base = upstream if upstream.startswith(("ws://", "wss://")) else http_base_to_ws(upstream)
        return await _proxy_ws(request, ws_base)
    return await _proxy_http(request, ws_base_to_http(upstream), "/webrtc")


async def handle_fallback(request: web.Request) -> web.Response:
    del request
    return web.Response(status=404, text="not found\n", headers={"Cache-Control": "no-store"})


def create_app(
    *,
    token: str,
    rosbridge_url: str,
    video_url: str,
    foxglove_url: str,
    webrtc_url: str | None = None,
) -> web.Application:
    """Build the gateway app. Every matched route requires a valid token."""
    register_secret(token)
    install_redaction()
    if ACCESS_LOG.level == logging.NOTSET:
        ACCESS_LOG.setLevel(logging.INFO)
    app = web.Application(middlewares=[access_middleware, auth_middleware])
    app[TOKEN_KEY] = token
    app[ROSBRIDGE_KEY] = rosbridge_url
    app[VIDEO_KEY] = video_url
    app[FOXGLOVE_KEY] = foxglove_url
    app[WEBRTC_KEY] = webrtc_url or ""
    app.on_startup.append(_startup)
    app.on_cleanup.append(_cleanup)
    app.router.add_route("*", "/", handle_root)
    app.router.add_route("*", "/video", handle_video)
    app.router.add_route("*", "/video/{tail:.*}", handle_video)
    app.router.add_get("/rosbridge", handle_rosbridge)
    app.router.add_get("/rosbridge/{tail:.*}", handle_rosbridge)
    app.router.add_get("/foxglove", handle_foxglove)
    app.router.add_get("/foxglove/{tail:.*}", handle_foxglove)
    app.router.add_route("*", "/webrtc", handle_webrtc)
    app.router.add_route("*", "/webrtc/{tail:.*}", handle_webrtc)
    app.router.add_route("*", "/{tail:.*}", handle_fallback)
    return app
