# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0

import logging

from ohho_gateway.redact import install_redaction, redact_text, register_secret

TOKEN = "ohho-test-token-0123456789"


def test_redact_text_hides_query_bearer_and_registered_secret():
    register_secret(TOKEN)
    text = redact_text(
        f"GET /video/stream?topic=/cam&token={TOKEN} Authorization: Bearer {TOKEN} {TOKEN}"
    )
    assert TOKEN not in text
    assert "token=[redacted]" in text
    assert "Bearer [redacted]" in text


def test_log_filter_hides_the_token(caplog):
    register_secret(TOKEN)
    install_redaction()
    caplog.set_level(logging.INFO, logger="ohho_gateway.access")
    logger = logging.getLogger("ohho_gateway.access")
    logger.info("GET /rosbridge?token=%s", TOKEN)
    logger.info("Authorization: Bearer %s", TOKEN)
    assert TOKEN not in caplog.text
    assert "token=[redacted]" in caplog.text
