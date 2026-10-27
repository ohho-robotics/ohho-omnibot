"""jsonschema helpers for the copied ohho.telemetry.v1 fixtures.

Ubuntu Noble's python3-jsonschema 4.10.3 has Draft202012Validator and
RefResolver, and it does not ship the ``referencing`` package (that arrived
with jsonschema 4.18). CI keeps the Debian package. Use ``referencing`` when
it imports, and a local RefResolver store otherwise.
"""

from __future__ import annotations

import json
from pathlib import Path

from jsonschema import Draft202012Validator
from jsonschema.exceptions import ValidationError

from omnibot_hybrid.telemetry_snapshot import schema_body

FIXTURE_DIR = (
    Path(__file__).resolve().parents[1] / "test" / "fixtures" / "ohho.telemetry.v1"
)
SCHEMA_FILES = (
    "defs.json",
    "heartbeat.json",
    "heartbeat_echo.json",
    "safety_state.json",
    "telemetry.json",
)


def _load_documents() -> dict[str, dict]:
    documents = {}
    for name in SCHEMA_FILES:
        documents[name] = json.loads((FIXTURE_DIR / name).read_text(encoding="utf-8"))
    return documents


def _validators_with_referencing(documents: dict[str, dict]) -> dict[str, Draft202012Validator]:
    from referencing import Registry, Resource

    resources = [
        (document["$id"], Resource.from_contents(document))
        for document in documents.values()
    ]
    registry = Registry().with_resources(resources)
    return {
        name: Draft202012Validator(documents[name], registry=registry)
        for name in SCHEMA_FILES
        if name != "defs.json"
    }


def _validators_with_ref_resolver(documents: dict[str, dict]) -> dict[str, Draft202012Validator]:
    from jsonschema import RefResolver

    store = {document["$id"]: document for document in documents.values()}
    validators = {}
    for name in SCHEMA_FILES:
        if name == "defs.json":
            continue
        schema = documents[name]
        resolver = RefResolver(base_uri=schema["$id"], referrer=schema, store=store)
        validators[name] = Draft202012Validator(schema, resolver=resolver)
    return validators


def load_validators() -> dict[str, Draft202012Validator]:
    documents = _load_documents()
    try:
        import referencing  # noqa: F401
    except ImportError:
        return _validators_with_ref_resolver(documents)
    return _validators_with_referencing(documents)


def assert_valid_telemetry(document: dict) -> None:
    """The v1 body (age_ms removed) must validate. age_ms itself must not."""
    validator = load_validators()["telemetry.json"]
    body = schema_body(document)
    validator.validate(body)
    if "age_ms" in document:
        try:
            validator.validate(document)
        except ValidationError:
            return
        raise AssertionError("age_ms was accepted by ohho.telemetry.v1")
