# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Self-signed TLS material for LAN pinning. Generated on first run and reused."""

from __future__ import annotations

import base64
import datetime as dt
import hashlib
import ipaddress
import logging
import os
import ssl
from dataclasses import dataclass
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509.oid import NameOID

from ohho_gateway.errors import ConfigError

CERT_NAME = "cert.pem"
KEY_NAME = "key.pem"
FINGERPRINT_NAME = "fingerprint.txt"
_CERT_LIFETIME = dt.timedelta(days=825)
# Warn before expiry so an operator can re-pair. Do not regenerate: a new
# cert would change the pins clients already stored.
_CERT_EXPIRY_WARNING = dt.timedelta(days=30)
_LOG = logging.getLogger("ohho_gateway")
_REPAIR_HINT = (
    "Delete the state directory and re-pair clients. "
    "The certificate was not regenerated, so existing pins stay unchanged."
)


@dataclass(frozen=True)
class TlsMaterial:
    cert_path: Path
    key_path: Path
    cert_sha256: str
    spki_pin: str

    @property
    def fingerprint_line(self) -> str:
        return (
            f"ohho-gateway TLS certificate SHA-256: {self.cert_sha256}\n"
            f"ohho-gateway TLS SPKI pin: {self.spki_pin}"
        )


def _chmod(path: Path, mode: int) -> None:
    try:
        os.chmod(path, mode)
    except OSError:
        pass


def certificate_sha256(cert: x509.Certificate) -> str:
    digest = cert.fingerprint(hashes.SHA256())
    return ":".join(f"{byte:02X}" for byte in digest)


def spki_pin(cert: x509.Certificate) -> str:
    """OkHttp CertificatePinner form: ``sha256/<base64 of SPKI SHA-256>``."""
    spki = cert.public_key().public_bytes(
        serialization.Encoding.DER,
        serialization.PublicFormat.SubjectPublicKeyInfo,
    )
    encoded = base64.b64encode(hashlib.sha256(spki).digest()).decode("ascii")
    return "sha256/" + encoded


def _not_valid_after(cert: x509.Certificate) -> dt.datetime:
    expiry = getattr(cert, "not_valid_after_utc", None)
    if expiry is not None:
        return expiry
    expiry = cert.not_valid_after
    if expiry.tzinfo is None:
        return expiry.replace(tzinfo=dt.timezone.utc)
    return expiry


def _warn_if_certificate_expiring(cert: x509.Certificate, state_dir: Path) -> None:
    expiry = _not_valid_after(cert)
    remaining = expiry - dt.datetime.now(dt.timezone.utc)
    if remaining <= dt.timedelta(0):
        _LOG.warning(
            "ohho-gateway TLS certificate in %s expired at %s. %s",
            state_dir,
            expiry.isoformat(),
            _REPAIR_HINT,
        )
        return
    if remaining <= _CERT_EXPIRY_WARNING:
        _LOG.warning(
            "ohho-gateway TLS certificate in %s expires at %s, within %s days. %s",
            state_dir,
            expiry.isoformat(),
            _CERT_EXPIRY_WARNING.days,
            _REPAIR_HINT,
        )


def _load_cert(path: Path) -> x509.Certificate:
    try:
        data = path.read_bytes()
        return x509.load_pem_x509_certificate(data)
    except (OSError, ValueError) as exc:
        raise ConfigError(
            f"cannot read TLS certificate {path}; delete the state directory to regenerate"
        ) from exc


def _generate(cert_path: Path, key_path: Path) -> x509.Certificate:
    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "ohho-gateway")])
    now = dt.datetime.now(dt.timezone.utc)
    builder = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
    )
    # cryptography 50 keeps not_valid_before and requires an aware datetime.
    # Older releases expose not_valid_before_utc for the same thing.
    if hasattr(builder, "not_valid_before_utc"):
        builder = builder.not_valid_before_utc(now).not_valid_after_utc(now + _CERT_LIFETIME)
    else:
        builder = builder.not_valid_before(now).not_valid_after(now + _CERT_LIFETIME)
    builder = (
        builder
        .add_extension(
            x509.SubjectAlternativeName(
                [
                    x509.DNSName("localhost"),
                    x509.DNSName("ohho-gateway"),
                    x509.IPAddress(ipaddress.IPv4Address("127.0.0.1")),
                ]
            ),
            critical=False,
        )
        .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
    )
    cert = builder.sign(key, hashes.SHA256())
    key_path.write_bytes(
        key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.PKCS8,
            encryption_algorithm=serialization.NoEncryption(),
        )
    )
    _chmod(key_path, 0o600)
    cert_path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    _chmod(cert_path, 0o644)
    return cert


def ensure_certificate(state_dir: Path) -> TlsMaterial:
    """Create a self-signed cert on first run. Later runs reuse it so pins stay valid."""
    state_dir.mkdir(parents=True, exist_ok=True)
    _chmod(state_dir, 0o700)
    cert_path = state_dir / CERT_NAME
    key_path = state_dir / KEY_NAME
    if cert_path.exists() != key_path.exists():
        raise ConfigError(
            f"incomplete TLS material in {state_dir}; delete the state directory to regenerate"
        )
    if cert_path.exists():
        cert = _load_cert(cert_path)
        _warn_if_certificate_expiring(cert, state_dir)
    else:
        cert = _generate(cert_path, key_path)
    material = TlsMaterial(
        cert_path=cert_path,
        key_path=key_path,
        cert_sha256=certificate_sha256(cert),
        spki_pin=spki_pin(cert),
    )
    fingerprint_path = state_dir / FINGERPRINT_NAME
    fingerprint_path.write_text(
        f"cert-sha256={material.cert_sha256}\nspki-sha256={material.spki_pin}\n",
        encoding="utf-8",
    )
    return material


def server_ssl_context(material: TlsMaterial) -> ssl.SSLContext:
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(material.cert_path, material.key_path)
    return context


def client_ssl_context(material: TlsMaterial) -> ssl.SSLContext:
    """Trust this gateway's cert. Used by tests and same-host tools."""
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_verify_locations(material.cert_path)
    context.check_hostname = True
    context.verify_mode = ssl.CERT_REQUIRED
    return context
