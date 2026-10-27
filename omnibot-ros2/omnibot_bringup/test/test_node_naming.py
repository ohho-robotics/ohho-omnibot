"""Guardrails for OHH-54 policy_node / arm mux naming."""

from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[3]

# Paths that must not reintroduce legacy ROS node or mux names.
FORBIDDEN_SNIPPETS = (
    'name="smolvla_node"',
    "smolvla_node:",
    "use_smolvla",
    "SendArmMode(\"smolvla\")",
    "/smolvla/task",
    "/smolvla/enable",
    "routes smolvla/rl_arm",
    '"smolvla" | "rl_arm"',
)


def _iter_text_files():
    skip_dirs = {
        ".git",
        ".venv",
        "node_modules",
        "omnibot-vr/Packages",
        "omnibot-vr/Library",
    }
    for path in REPO.rglob("*"):
        if not path.is_file():
            continue
        rel = path.relative_to(REPO).as_posix()
        if any(part in rel for part in skip_dirs):
            continue
        if path.suffix not in {".py", ".yaml", ".yml", ".md", ".cs", ".launch.py"}:
            continue
        if path.name in {"test_node_naming.py", "CLAUDE.md", "smolvla_params.yaml"}:
            continue
        yield path


def test_no_legacy_smolvla_node_or_mux_names():
    violations: list[str] = []
    for path in _iter_text_files():
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for snippet in FORBIDDEN_SNIPPETS:
            if snippet in text:
                violations.append(f"{path.relative_to(REPO)}: contains `{snippet}`")
    assert not violations, "Legacy naming found:\n" + "\n".join(violations)
