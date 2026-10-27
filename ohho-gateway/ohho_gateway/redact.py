# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Keep configured tokens and bearer/query credentials out of log records."""

from __future__ import annotations

import logging
import re
import threading

_TOKEN_QUERY = re.compile(r"(?i)(token=)[^&\s\"']+")
_BEARER_HEADER = re.compile(r"(?i)(authorization:\s*bearer\s+)\S+")
_secrets: set[str] = set()
_lock = threading.Lock()


def register_secret(secret: str) -> None:
    if len(secret) < 8:
        return
    with _lock:
        _secrets.add(secret)


def redact_text(text: str) -> str:
    text = _BEARER_HEADER.sub(r"\1[redacted]", text)
    text = _TOKEN_QUERY.sub(r"\1[redacted]", text)
    with _lock:
        secrets = tuple(_secrets)
    for secret in secrets:
        if secret and secret in text:
            text = text.replace(secret, "[redacted]")
    return text


class SecretRedactFilter(logging.Filter):
    """Rewrite log records so a token cannot land in a handler."""

    def filter(self, record: logging.LogRecord) -> bool:
        # Interpolate first. Redacting the template can eat a ``%s`` that
        # sits after ``token=`` and then formatting raises.
        if isinstance(record.msg, str) and record.args:
            try:
                rendered = record.msg % record.args
            except (TypeError, ValueError):
                rendered = record.msg
            record.msg = redact_text(rendered)
            record.args = None
        elif isinstance(record.msg, str):
            record.msg = redact_text(record.msg)
        rendered = getattr(record, "message", None)
        if isinstance(rendered, str):
            record.message = redact_text(rendered)
        return True


def install_redaction() -> None:
    """Attach the filter to the root logger and every logger created so far."""
    filt = SecretRedactFilter()
    loggers: list[logging.Logger] = [logging.getLogger()]
    for name in list(logging.root.manager.loggerDict):
        logger = logging.getLogger(name)
        if isinstance(logger, logging.Logger):
            loggers.append(logger)
    for logger in loggers:
        if not any(isinstance(existing, SecretRedactFilter) for existing in logger.filters):
            logger.addFilter(filt)
