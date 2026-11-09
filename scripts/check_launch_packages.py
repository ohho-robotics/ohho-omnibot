#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Fail when a launch file names a ROS package this repo cannot run.

A reference is a string literal passed as ``package=``, to
``get_package_share_directory``, or to ``FindPackageShare``. It is allowed
when any of these is true:

* a ``package.xml`` in the repo names that package (it is in the workspace)
* a ``package.xml`` lists it as ``<depend>`` or ``<exec_depend>`` (upstream
  ROS packages; ``<depend>`` already implies an exec dependency)
* the reference is guarded by ``PackageAvailable`` / ``optional_include_launch``
  (see ``omnibot_hybrid.optional_package.GUARD_CALLS``)

``<build_depend>`` and ``<test_depend>`` are not launch allowlist entries.
Commented-out Python is ignored because the scan uses the AST.
"""

from __future__ import annotations

import ast
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
_HYBRID_SRC = REPO_ROOT / "omnibot-ros2" / "omnibot_hybrid"
if str(_HYBRID_SRC) not in sys.path:
    sys.path.insert(0, str(_HYBRID_SRC))

from omnibot_hybrid.optional_package import GUARD_CALLS  # noqa: E402

SKIP_DIRS = frozenset(
    {
        ".git",
        ".venv",
        "build",
        "install",
        "log",
        "node_modules",
        "__pycache__",
        ".pytest_cache",
    }
)
ALLOWLIST_TAGS = frozenset({"depend", "exec_depend"})
SHARE_LOOKUP_CALLS = frozenset(
    {"get_package_share_directory", "FindPackageShare"}
)


@dataclass(frozen=True)
class Violation:
    path: Path
    line: int
    package: str

    def format(self, root: Path) -> str:
        try:
            rel = self.path.resolve().relative_to(root.resolve())
        except ValueError:
            rel = self.path
        return (
            f"{rel.as_posix()}:{self.line}: unguarded package '{self.package}' "
            "is not in the workspace and is not a package.xml depend/exec_depend"
        )


def _skip_dir(name: str) -> bool:
    return name in SKIP_DIRS or name.startswith(".")


def iter_launch_files(root: Path) -> list[Path]:
    found: list[Path] = []
    for dirpath, dirnames, filenames in os_walk(root):
        dirnames[:] = [name for name in dirnames if not _skip_dir(name)]
        for name in filenames:
            if name.endswith(".launch.py"):
                found.append(Path(dirpath) / name)
    return sorted(found)


def iter_package_xml(root: Path) -> list[Path]:
    found: list[Path] = []
    for dirpath, dirnames, filenames in os_walk(root):
        dirnames[:] = [name for name in dirnames if not _skip_dir(name)]
        if "package.xml" in filenames:
            found.append(Path(dirpath) / "package.xml")
    return sorted(found)


def os_walk(root: Path):
    import os

    return os.walk(root)


def workspace_packages(root: Path) -> set[str]:
    names: set[str] = set()
    for path in iter_package_xml(root):
        name = _xml_text(ET.parse(path).getroot().find("name"))
        if name:
            names.add(name)
    return names


def exec_depend_allowlist(root: Path) -> set[str]:
    """Upstream packages declared as ``depend`` or ``exec_depend``."""
    allowed: set[str] = set()
    for path in iter_package_xml(root):
        root_el = ET.parse(path).getroot()
        for tag in ALLOWLIST_TAGS:
            for element in root_el.findall(tag):
                name = _xml_text(element)
                if name:
                    allowed.add(name)
    return allowed


def _xml_text(element) -> str:
    if element is None or element.text is None:
        return ""
    return element.text.strip()


def _call_name(node: ast.Call) -> str | None:
    func = node.func
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return None


def _const_str(node: ast.AST | None) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _guard_package_names(call: ast.Call) -> set[str]:
    """Package strings passed into a guard call."""
    found: set[str] = set()
    for keyword in call.keywords:
        if keyword.arg in {"package", "package_name"}:
            value = _const_str(keyword.value)
            if value:
                found.add(value)
    for arg in call.args:
        value = _const_str(arg)
        if value:
            found.add(value)
            break
    return found


def _guarded_in(tree: ast.AST) -> set[str]:
    found: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and _call_name(node) in GUARD_CALLS:
            found.update(_guard_package_names(node))
    return found


def references_in_source(source: str, path: Path) -> list[tuple[int, str, bool]]:
    """Return ``(line, package, guarded)`` for each package reference."""
    tree = ast.parse(source, filename=str(path))
    refs: list[tuple[int, str, bool]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = _call_name(node)
        keywords = {kw.arg: kw.value for kw in node.keywords if kw.arg}
        packages: list[str] = []
        if "package" in keywords:
            value = _const_str(keywords["package"])
            if value:
                packages.append(value)
        if name in SHARE_LOOKUP_CALLS:
            if node.args:
                value = _const_str(node.args[0])
                if value:
                    packages.append(value)
            if "package_name" in keywords:
                value = _const_str(keywords["package_name"])
                if value:
                    packages.append(value)
        if not packages:
            continue
        guarded_names: set[str] = set()
        if name in GUARD_CALLS:
            guarded_names.update(_guard_package_names(node))
        if "condition" in keywords:
            guarded_names.update(_guarded_in(keywords["condition"]))
        seen: set[str] = set()
        line = getattr(node, "lineno", 1)
        for package in packages:
            if package in seen:
                continue
            seen.add(package)
            refs.append((line, package, package in guarded_names))
    return refs


def find_violations(root: Path) -> list[Violation]:
    workspace = workspace_packages(root)
    allowlist = exec_depend_allowlist(root)
    violations: list[Violation] = []
    for path in iter_launch_files(root):
        source = path.read_text(encoding="utf-8")
        try:
            refs = references_in_source(source, path)
        except SyntaxError as exc:
            violations.append(
                Violation(
                    path=path,
                    line=exc.lineno or 1,
                    package=f"<syntax error: {exc.msg}>",
                )
            )
            continue
        for line, package, guarded in refs:
            if guarded or package in workspace or package in allowlist:
                continue
            violations.append(Violation(path=path, line=line, package=package))
    return violations


def main(argv: list[str] | None = None) -> int:
    del argv
    root = REPO_ROOT
    launch_files = iter_launch_files(root)
    violations = find_violations(root)
    if violations:
        for item in violations:
            print(item.format(root), file=sys.stderr)
        print(
            f"{len(violations)} unguarded launch package reference(s) "
            f"in {len(launch_files)} launch file(s).",
            file=sys.stderr,
        )
        return 1
    print(
        f"Checked {len(launch_files)} launch files; every package reference "
        "is in the workspace, declared as depend/exec_depend, or guarded."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
