# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""A small RFC 6455 codec so tests and the browser share one signaling socket.

The server speaks the subset aiohttp and Chrome use: text frames, masked
client frames, ping/pong, and close. Messages are capped at 4 MiB, under the
gateway's 8 MiB WebSocket limit.
"""

from __future__ import annotations

import asyncio
import base64
import hashlib
import os
from urllib.parse import urlsplit

_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MAX_MESSAGE_BYTES = 4 * 1024 * 1024

# RFC 6455 sample in section 1.3 / 4.2.2.
SAMPLE_KEY = "dGhlIHNhbXBsZSBub25jZQ=="
SAMPLE_ACCEPT = "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="


def accept_key(sec_websocket_key: str) -> str:
    digest = hashlib.sha1((sec_websocket_key + _GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def encode_frame(opcode: int, payload: bytes, mask: bytes | None = None) -> bytes:
    if len(payload) > MAX_MESSAGE_BYTES:
        raise ValueError("websocket message is too large")
    header = bytearray([0x80 | (opcode & 0x0F)])
    length = len(payload)
    mask_bit = 0x80 if mask is not None else 0
    if length < 126:
        header.append(mask_bit | length)
    elif length < 65536:
        header.append(mask_bit | 126)
        header.extend(length.to_bytes(2, "big"))
    else:
        header.append(mask_bit | 127)
        header.extend(length.to_bytes(8, "big"))
    if mask is not None:
        if len(mask) != 4:
            raise ValueError("websocket mask must be 4 bytes")
        header.extend(mask)
        payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
    return bytes(header) + payload


async def read_frame(reader: asyncio.StreamReader) -> tuple[bool, int, bytes]:
    """Return ``(fin, opcode, payload)``. Client frames must be masked."""
    try:
        first, second = await reader.readexactly(2)
    except asyncio.IncompleteReadError as exc:
        raise ConnectionError("websocket closed") from exc
    fin = bool(first & 0x80)
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = int.from_bytes(await reader.readexactly(2), "big")
    elif length == 127:
        length = int.from_bytes(await reader.readexactly(8), "big")
    if length > MAX_MESSAGE_BYTES:
        raise ConnectionError("websocket message is too large")
    mask = await reader.readexactly(4) if masked else b""
    payload = await reader.readexactly(length) if length else b""
    if masked:
        payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
    return fin, opcode, payload


async def write_frame(writer: asyncio.StreamWriter, opcode: int, payload: bytes = b"") -> None:
    writer.write(encode_frame(opcode, payload))
    await writer.drain()


async def read_text_message(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
) -> str | None:
    """Read one text message. Answer ping with pong. None means the peer closed."""
    parts: list[bytes] = []
    data_opcode = None
    while True:
        fin, opcode, payload = await read_frame(reader)
        if opcode == 0x8:
            await write_frame(writer, 0x8, payload[:125])
            return None
        if opcode == 0x9:
            await write_frame(writer, 0xA, payload)
            continue
        if opcode == 0xA:
            continue
        if opcode in (0x1, 0x2):
            data_opcode = opcode
            parts = [payload]
        elif opcode == 0x0 and data_opcode is not None:
            parts.append(payload)
        else:
            raise ConnectionError("unsupported websocket opcode")
        if fin:
            if data_opcode != 0x1:
                raise ConnectionError("binary websocket frames are not used")
            return b"".join(parts).decode("utf-8")


async def open_signaling(url: str) -> tuple[asyncio.StreamReader, asyncio.StreamWriter]:
    """Open a masked client WebSocket to ``ws://host:port/path``."""
    parts = urlsplit(url)
    if parts.scheme != "ws":
        raise ValueError("signaling client only opens ws:// URLs")
    host = parts.hostname or "127.0.0.1"
    port = parts.port or 80
    path = parts.path or "/"
    if parts.query:
        path = path + "?" + parts.query
    reader, writer = await asyncio.open_connection(host, port)
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    request = (
        f"GET {path} HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n"
    )
    writer.write(request.encode("ascii"))
    await writer.drain()
    raw = await reader.readuntil(b"\r\n\r\n")
    head = raw.decode("iso-8859-1")
    status_line = head.split("\r\n", 1)[0]
    if " 101 " not in status_line:
        writer.close()
        raise ConnectionError(status_line)
    expected = accept_key(key)
    if expected not in head:
        writer.close()
        raise ConnectionError("missing websocket accept")
    return reader, writer


async def client_send_text(writer: asyncio.StreamWriter, text: str) -> None:
    mask = os.urandom(4)
    writer.write(encode_frame(0x1, text.encode("utf-8"), mask))
    await writer.drain()
