# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0

import datetime as dt
import hashlib
import logging

import pytest
from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509.oid import NameOID

from ohho_gateway.cli import prepare
from ohho_gateway.errors import ConfigError
from ohho_gateway.tls import _CERT_EXPIRY_WARNING, ensure_certificate

TOKEN = "file-token-value-0123456789"


def test_certificate_is_reused_and_fingerprint_matches_the_der(tmp_path):
    first = ensure_certificate(tmp_path)
    second = ensure_certificate(tmp_path)
    assert first.cert_sha256 == second.cert_sha256
    assert first.spki_pin == second.spki_pin
    assert first.spki_pin.startswith("sha256/")
    cert = x509.load_pem_x509_certificate(first.cert_path.read_bytes())
    der = cert.public_bytes(serialization.Encoding.DER)
    digest = hashlib.sha256(der).hexdigest().upper()
    assert digest == first.cert_sha256.replace(":", "")
    saved = (tmp_path / "fingerprint.txt").read_text(encoding="utf-8")
    assert first.cert_sha256 in saved
    assert first.spki_pin in saved
    if __import__("sys").platform != "win32":
        assert first.key_path.stat().st_mode & 0o777 == 0o600


def _write_dated_certificate(directory, not_after: dt.datetime) -> bytes:
    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "ohho-gateway")])
    not_before = not_after - dt.timedelta(days=2)
    builder = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
    )
    if hasattr(builder, "not_valid_before_utc"):
        builder = builder.not_valid_before_utc(not_before).not_valid_after_utc(not_after)
    else:
        builder = builder.not_valid_before(not_before).not_valid_after(not_after)
    cert = builder.sign(key, hashes.SHA256())
    (directory / "key.pem").write_bytes(
        key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.PKCS8,
            encryption_algorithm=serialization.NoEncryption(),
        )
    )
    cert_path = directory / "cert.pem"
    cert_path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    return cert_path.read_bytes()


def test_reused_certificate_warns_when_expired_or_close_and_is_not_replaced(tmp_path, caplog):
    now = dt.datetime.now(dt.timezone.utc)
    expired = _write_dated_certificate(tmp_path, now - dt.timedelta(days=1))
    with caplog.at_level(logging.WARNING, logger="ohho_gateway"):
        ensure_certificate(tmp_path)
    assert (tmp_path / "cert.pem").read_bytes() == expired
    assert "expired" in caplog.text
    assert "Delete the state directory" in caplog.text
    assert "re-pair clients" in caplog.text

    caplog.clear()
    soon = tmp_path / "soon"
    soon.mkdir()
    soon_bytes = _write_dated_certificate(soon, now + _CERT_EXPIRY_WARNING - dt.timedelta(days=1))
    with caplog.at_level(logging.WARNING, logger="ohho_gateway"):
        ensure_certificate(soon)
    assert (soon / "cert.pem").read_bytes() == soon_bytes
    assert "within" in caplog.text
    assert "Delete the state directory" in caplog.text

    caplog.clear()
    later = tmp_path / "later"
    later.mkdir()
    later_bytes = _write_dated_certificate(later, now + dt.timedelta(days=400))
    with caplog.at_level(logging.WARNING, logger="ohho_gateway"):
        ensure_certificate(later)
    assert (later / "cert.pem").read_bytes() == later_bytes
    assert caplog.text == ""


def test_incomplete_or_corrupt_material_is_not_replaced(tmp_path):
    (tmp_path / "cert.pem").write_text("not a cert", encoding="utf-8")
    with pytest.raises(ConfigError):
        ensure_certificate(tmp_path)
    (tmp_path / "key.pem").write_text("not a key", encoding="utf-8")
    with pytest.raises(ConfigError) as caught:
        ensure_certificate(tmp_path)
    assert "delete the state directory" in str(caught.value)


def test_plain_listener_refuses_non_loopback_before_writing_a_cert(tmp_path):
    with pytest.raises(ConfigError) as caught:
        prepare(
            ["--state-dir", str(tmp_path), "--plain-host", "0.0.0.0"],
            env={},
        )
    assert "loopback" in str(caught.value)
    assert not (tmp_path / "cert.pem").exists()
    assert TOKEN not in str(caught.value)


def test_env_token_is_absent_from_the_operator_banner(tmp_path):
    prepared = prepare(
        ["--state-dir", str(tmp_path), "--tls-only"],
        env={"OHHO_GATEWAY_TOKEN": TOKEN},
    )
    assert prepared.source == "env"
    assert TOKEN not in prepared.banner
    assert prepared.material.cert_sha256 in prepared.banner
    assert prepared.material.spki_pin in prepared.banner


def test_explicit_token_file_is_not_printed(tmp_path):
    token_file = tmp_path / "secret"
    token_file.write_text(TOKEN + "\n", encoding="utf-8")
    prepared = prepare(
        ["--state-dir", str(tmp_path / "state"), "--tls-only", "--token-file", str(token_file)],
        env={},
    )
    assert prepared.source == "file"
    assert TOKEN not in prepared.banner
