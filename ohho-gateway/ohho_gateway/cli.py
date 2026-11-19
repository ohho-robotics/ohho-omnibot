# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Process entry point. The pairing code is printed only for a generated code."""

from __future__ import annotations

import argparse
import asyncio
import ipaddress
import os
import sys
from dataclasses import dataclass
from pathlib import Path

from aiohttp import web

from ohho_gateway.auth import resolve_token
from ohho_gateway.errors import ConfigError
from ohho_gateway.server import create_app
from ohho_gateway.tls import TlsMaterial, ensure_certificate, server_ssl_context

ENV_TOKEN = "OHHO_GATEWAY_TOKEN"
DEFAULT_BIND = "0.0.0.0"
DEFAULT_PORT = 8443
DEFAULT_PLAIN_HOST = "127.0.0.1"
DEFAULT_PLAIN_PORT = 9080


@dataclass(frozen=True)
class Settings:
    bind: str
    port: int
    plain_host: str
    plain_port: int
    tls_only: bool
    state_dir: Path
    token_file: Path | None
    rosbridge_url: str
    video_url: str
    foxglove_url: str
    webrtc_url: str | None


@dataclass(frozen=True)
class Prepared:
    settings: Settings
    token: str
    source: str
    material: TlsMaterial
    banner: str


@dataclass(frozen=True)
class Listening:
    runner: web.AppRunner
    tls_address: tuple
    plain_address: tuple | None


def is_loopback(host: str) -> bool:
    if host.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def assert_plain_bind(host: str, tls_only: bool) -> None:
    """Plain ws/http is refused on any non-loopback address."""
    if tls_only:
        return
    if not is_loopback(host):
        raise ConfigError(
            "plain ws/http is only allowed on a loopback address "
            f"(refusing {host!r})"
        )


def operator_banner(*, source: str, token: str, material: TlsMaterial) -> str:
    """Banner for stdout. Env and explicit token files are not printed."""
    lines = []
    if source in ("generated", "pairing"):
        lines.append(f"ohho-gateway pairing code: {token}")
    lines.append(material.fingerprint_line)
    return "\n".join(lines)


def default_state_dir() -> Path:
    return Path.home() / ".ohho" / "gateway"


def _parse(argv: list[str] | None) -> Settings:
    parser = argparse.ArgumentParser(prog="ohho-gateway")
    parser.add_argument("--bind", default=DEFAULT_BIND, help="TLS listen address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="TLS listen port")
    parser.add_argument(
        "--plain-host",
        default=DEFAULT_PLAIN_HOST,
        help="Plain ws/http listen address. Must be loopback.",
    )
    parser.add_argument("--plain-port", type=int, default=DEFAULT_PLAIN_PORT)
    parser.add_argument(
        "--tls-only",
        action="store_true",
        help="Do not open the plain loopback listener (cloud profile).",
    )
    parser.add_argument(
        "--state-dir",
        default="",
        help="Directory for the cert, key, and generated pairing code. Default: ~/.ohho/gateway",
    )
    parser.add_argument(
        "--token-file",
        default="",
        help="File containing the bearer token. Overrides the generated pairing code.",
    )
    parser.add_argument("--rosbridge", default="ws://127.0.0.1:9090")
    parser.add_argument("--video", default="http://127.0.0.1:8080")
    parser.add_argument("--foxglove", default="ws://127.0.0.1:8765")
    parser.add_argument(
        "--webrtc",
        default="",
        help="Upstream base URL for /webrtc/. Empty reserves the path with 503.",
    )
    args = parser.parse_args(argv)
    for name, value in (("port", args.port), ("plain-port", args.plain_port)):
        # 0 lets the kernel pick an ephemeral port (tests). Operators use 8443/9080.
        if value < 0 or value > 65535:
            raise ConfigError(f"{name} must be between 0 and 65535")
    state_dir = Path(args.state_dir).expanduser() if args.state_dir else default_state_dir()
    token_file = Path(args.token_file).expanduser() if args.token_file else None
    return Settings(
        bind=args.bind,
        port=args.port,
        plain_host=args.plain_host,
        plain_port=args.plain_port,
        tls_only=args.tls_only,
        state_dir=state_dir,
        token_file=token_file,
        rosbridge_url=args.rosbridge,
        video_url=args.video,
        foxglove_url=args.foxglove,
        webrtc_url=args.webrtc or None,
    )


def prepare(argv: list[str] | None = None, *, env: dict[str, str] | None = None) -> Prepared:
    settings = _parse(argv)
    assert_plain_bind(settings.plain_host, settings.tls_only)
    material = ensure_certificate(settings.state_dir)
    source_env = env if env is not None else os.environ
    token, source = resolve_token(
        env_token=source_env.get(ENV_TOKEN),
        token_file=settings.token_file,
        pairing_file=settings.state_dir / "pairing-code",
    )
    banner = operator_banner(source=source, token=token, material=material)
    return Prepared(
        settings=settings,
        token=token,
        source=source,
        material=material,
        banner=banner,
    )


def _socket_name(site) -> tuple:
    sockets = getattr(site, "_server").sockets
    if not sockets:
        raise ConfigError("listener did not bind a socket")
    return sockets[0].getsockname()


async def open_sites(prepared: Prepared) -> Listening:
    """Bind TLS, and plain ws/http only when the plain host is loopback."""
    settings = prepared.settings
    assert_plain_bind(settings.plain_host, settings.tls_only)
    app = create_app(
        token=prepared.token,
        rosbridge_url=settings.rosbridge_url,
        video_url=settings.video_url,
        foxglove_url=settings.foxglove_url,
        webrtc_url=settings.webrtc_url,
    )
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    try:
        tls_site = web.TCPSite(
            runner,
            settings.bind,
            settings.port,
            ssl_context=server_ssl_context(prepared.material),
        )
        await tls_site.start()
        tls_address = _socket_name(tls_site)
        plain_address = None
        if not settings.tls_only:
            plain_site = web.TCPSite(runner, settings.plain_host, settings.plain_port)
            await plain_site.start()
            plain_address = _socket_name(plain_site)
    except Exception:
        await runner.cleanup()
        raise
    return Listening(runner=runner, tls_address=tls_address, plain_address=plain_address)


async def serve(prepared: Prepared) -> None:
    listening = await open_sites(prepared)
    try:
        await asyncio.Event().wait()
    finally:
        await listening.runner.cleanup()


def main(argv: list[str] | None = None) -> int:
    try:
        prepared = prepare(argv)
    except ConfigError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(prepared.banner, flush=True)
    try:
        asyncio.run(serve(prepared))
    except KeyboardInterrupt:
        return 0
    return 0
