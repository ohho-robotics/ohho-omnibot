# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

import pytest

from ohho_gateway.auth import (
    bearer_token,
    generate_pairing_code,
    presented_token,
    resolve_token,
    tokens_equal,
    validate_configured_token,
)
from ohho_gateway.errors import ConfigError

TOKEN = "ohho-test-token-0123456789"


def test_tokens_equal_rejects_wrong_and_different_lengths():
    assert tokens_equal(TOKEN, TOKEN) is True
    assert tokens_equal("ohho-test-token-0123456788", TOKEN) is False
    assert tokens_equal("short", TOKEN) is False
    assert tokens_equal("", TOKEN) is False


def test_bearer_and_query_extraction():
    assert bearer_token("Bearer " + TOKEN) == TOKEN
    assert bearer_token("bearer " + TOKEN) == TOKEN
    assert bearer_token("Basic " + TOKEN) is None
    assert bearer_token("Bearer") is None
    assert presented_token(None, TOKEN) == TOKEN
    assert presented_token("Bearer " + TOKEN, None) == TOKEN
    assert presented_token("Bearer " + TOKEN, TOKEN) == TOKEN
    assert presented_token("Bearer " + TOKEN, "ohho-test-token-0123456788") is None
    assert presented_token(None, None) is None


def test_configured_token_rejects_short_and_whitespace():
    with pytest.raises(ConfigError):
        validate_configured_token("too-short")
    with pytest.raises(ConfigError):
        validate_configured_token(TOKEN + " tail")
    assert validate_configured_token("  " + TOKEN + "\n") == TOKEN


def test_resolve_token_prefers_env_then_file_then_pairing(tmp_path: Path):
    pairing = tmp_path / "pairing-code"
    explicit = tmp_path / "explicit"
    explicit.write_text(TOKEN + "\n", encoding="utf-8")
    env_token = "env-token-value-0123456789"
    resolved, source = resolve_token(
        env_token=env_token,
        token_file=explicit,
        pairing_file=pairing,
    )
    assert (resolved, source) == (env_token, "env")

    resolved, source = resolve_token(env_token=None, token_file=explicit, pairing_file=pairing)
    assert (resolved, source) == (TOKEN, "file")

    resolved, source = resolve_token(env_token="", token_file=None, pairing_file=pairing)
    assert source == "generated"
    assert len(resolved) >= 16
    again, again_source = resolve_token(env_token=None, token_file=None, pairing_file=pairing)
    assert again_source == "pairing"
    assert again == resolved
    if __import__("sys").platform != "win32":
        assert pairing.stat().st_mode & 0o777 == 0o600


def test_generate_pairing_code_has_no_whitespace():
    code = generate_pairing_code()
    validate_configured_token(code)


def test_missing_token_file_is_an_error(tmp_path: Path):
    with pytest.raises(ConfigError) as caught:
        resolve_token(
            env_token=None,
            token_file=tmp_path / "missing",
            pairing_file=tmp_path / "pairing-code",
        )
    assert TOKEN not in str(caught.value)
