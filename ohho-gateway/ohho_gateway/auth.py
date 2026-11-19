# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Bearer and query-token checks.

Presented tokens are hashed before ``hmac.compare_digest`` so the compare
itself does not walk the configured token byte by byte.
"""

from __future__ import annotations

import hashlib
import hmac
import os
import secrets
from pathlib import Path

from ohho_gateway.errors import ConfigError

TOKEN_QUERY_PARAM = "token"
MIN_TOKEN_LENGTH = 16
MAX_TOKEN_LENGTH = 512
_BEARER = "bearer"


def tokens_equal(presented: str, expected: str) -> bool:
    """Return True when the two strings are equal, in constant time."""
    presented_digest = hashlib.sha256(presented.encode("utf-8")).digest()
    expected_digest = hashlib.sha256(expected.encode("utf-8")).digest()
    return hmac.compare_digest(presented_digest, expected_digest)


def validate_configured_token(token: str) -> str:
    """Accept a configured token. The error text never includes the token."""
    cleaned = token.strip()
    if len(cleaned) < MIN_TOKEN_LENGTH or len(cleaned) > MAX_TOKEN_LENGTH:
        raise ConfigError(
            f"token must be {MIN_TOKEN_LENGTH} to {MAX_TOKEN_LENGTH} characters"
        )
    if any(character.isspace() for character in cleaned):
        raise ConfigError("token must not contain whitespace")
    return cleaned


def bearer_token(authorization: str | None) -> str | None:
    """Return the credential from ``Authorization: Bearer <token>``."""
    if not authorization:
        return None
    scheme, separator, credential = authorization.partition(" ")
    if separator != " " or scheme.lower() != _BEARER:
        return None
    credential = credential.strip()
    if not credential or any(character.isspace() for character in credential):
        return None
    return credential


def presented_token(authorization: str | None, query_token: str | None) -> str | None:
    """Header, query, or both when they match. A mismatch is unauthenticated."""
    header = bearer_token(authorization)
    query = query_token if query_token else None
    if header is not None and query is not None:
        if not tokens_equal(header, query):
            return None
        return header
    if header is not None:
        return header
    return query


def generate_pairing_code() -> str:
    return secrets.token_urlsafe(18)


def _read_token_file(path: Path) -> str:
    try:
        raw = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"cannot read token file: {path}") from exc
    return validate_configured_token(raw)


def _write_token_file(path: Path, token: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(token + "\n", encoding="utf-8")
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass


def resolve_token(
    *,
    env_token: str | None,
    token_file: Path | None,
    pairing_file: Path,
) -> tuple[str, str]:
    """Return ``(token, source)``.

    ``source`` is ``env``, ``file``, ``pairing``, or ``generated``.
    Env and an explicit token file are not echoed on the operator banner.
    A generated or remembered pairing code is.
    """
    if env_token is not None and env_token.strip() != "":
        return validate_configured_token(env_token), "env"
    if token_file is not None:
        if not token_file.is_file():
            raise ConfigError(f"token file not found: {token_file}")
        return _read_token_file(token_file), "file"
    if pairing_file.is_file():
        return _read_token_file(pairing_file), "pairing"
    token = generate_pairing_code()
    _write_token_file(pairing_file, token)
    return token, "generated"
