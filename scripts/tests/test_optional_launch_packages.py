# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Helper and launch-package scanner. No ROS install required."""

from __future__ import annotations

import ast
import sys
import textwrap
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
HYBRID_SRC = REPO / "omnibot-ros2" / "omnibot_hybrid"
SCRIPTS = REPO / "scripts"
for _path in (HYBRID_SRC, SCRIPTS):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from omnibot_hybrid.optional_package import (  # noqa: E402
    PackageAvailable,
    WarnOnce,
    coerce_enabled,
    missing_package_message,
    package_is_available,
    should_start_optional_node,
)
import check_launch_packages as scanner  # noqa: E402


class _Missing(Exception):
    def __init__(self, package_name: str) -> None:
        self.package_name = package_name
        super().__init__(package_name)


_Missing.__name__ = "PackageNotFoundError"


def _lookup(installed: set[str]):
    def resolve(package_name: str) -> str:
        if package_name not in installed:
            raise _Missing(package_name)
        return f"/opt/ros/{package_name}"

    return resolve


def test_missing_package_message_names_the_package_once():
    message = missing_package_message("omnibot_perception")
    assert message.count("omnibot_perception") == 1
    assert "not installed" in message


def test_coerce_enabled_matches_ros_launch_strings():
    assert coerce_enabled("true")
    assert coerce_enabled("True")
    assert coerce_enabled("1")
    assert not coerce_enabled("false")
    assert not coerce_enabled("False")
    assert not coerce_enabled("")
    assert not coerce_enabled("0")


def test_should_start_when_installed_and_enabled():
    start, warning = should_start_optional_node(
        "omnibot_ota",
        True,
        is_available=lambda _name: True,
    )
    assert start is True
    assert warning is None


def test_installed_but_disabled_skips_without_warning():
    start, warning = should_start_optional_node(
        "omnibot_metrics",
        False,
        is_available=lambda _name: True,
    )
    assert start is False
    assert warning is None


def test_missing_package_skips_with_one_warning_even_if_disabled():
    start, warning = should_start_optional_node(
        "omnibot_perception",
        False,
        is_available=lambda _name: False,
    )
    assert start is False
    assert warning == missing_package_message("omnibot_perception")


def test_missing_package_can_stay_silent_when_disabled():
    start, warning = should_start_optional_node(
        "astra_camera",
        False,
        is_available=lambda _name: False,
        warn_when_disabled=False,
    )
    assert start is False
    assert warning is None


def test_missing_package_warns_when_the_argument_asked_to_start():
    start, warning = should_start_optional_node(
        "astra_camera",
        True,
        is_available=lambda _name: False,
        warn_when_disabled=False,
    )
    assert start is False
    assert warning is not None
    assert "astra_camera" in warning


def test_warn_once_emits_a_single_line():
    seen: list[str] = []
    gate = WarnOnce()
    message = missing_package_message("omnibot_ota")
    gate(message, seen.append)
    gate(message, seen.append)
    gate(None, seen.append)
    assert seen == [message]


def test_package_is_available_uses_injected_lookup():
    assert package_is_available("rviz2", lookup=_lookup({"rviz2"})) is True
    assert package_is_available("omnibot_metrics", lookup=_lookup(set())) is False


def test_package_is_available_propagates_unexpected_errors():
    def explode(_package_name: str) -> str:
        raise RuntimeError("index broken")

    with pytest.raises(RuntimeError, match="index broken"):
        package_is_available("rviz2", lookup=explode)


def test_package_is_available_mocks_ament_index(monkeypatch):
    import types

    packages = types.ModuleType("ament_index_python.packages")

    class PackageNotFoundError(Exception):
        def __init__(self, package_name: str) -> None:
            self.package_name = package_name
            super().__init__(package_name)

    def get_package_share_directory(package_name: str) -> str:
        if package_name == "missing_pkg":
            raise PackageNotFoundError(package_name)
        return f"/opt/share/{package_name}"

    packages.PackageNotFoundError = PackageNotFoundError
    packages.get_package_share_directory = get_package_share_directory
    parent = types.ModuleType("ament_index_python")
    parent.packages = packages
    monkeypatch.setitem(sys.modules, "ament_index_python", parent)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", packages)

    assert package_is_available("missing_pkg") is False
    assert package_is_available("omnibot_bringup") is True


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(text).lstrip("\n"), encoding="utf-8")


def _fixture_repo(tmp_path: Path) -> Path:
    _write(
        tmp_path / "demo_pkg" / "package.xml",
        """
        <?xml version="1.0"?>
        <package format="3">
          <name>demo_pkg</name>
          <depend>rclpy</depend>
          <exec_depend>rviz2</exec_depend>
          <build_depend>build_only_pkg</build_depend>
          <test_depend>test_only_pkg</test_depend>
          <!-- <exec_depend>commented_pkg</exec_depend> -->
        </package>
        """,
    )
    _write(
        tmp_path / "demo_pkg" / "launch" / "demo.launch.py",
        """
        def generate():
            Node(package="demo_pkg", executable="demo")
            Node(package="rviz2", executable="rviz2")
            Node(package="rclpy", executable="nope")
            get_package_share_directory("demo_pkg")
            FindPackageShare("rviz2")
            FindPackageShare(package="rclpy")
            Node(
                package="omnibot_perception",
                executable="object_perception_node",
                condition=PackageAvailable("omnibot_perception", "false"),
            )
            optional_include_launch(
                package="astra_camera",
                relative_path=("launch", "astra.launch.py"),
                when="true",
            )
            Node(package="omnibot_metrics", executable="metrics_bridge")
            Node(package="build_only_pkg", executable="x")
            get_package_share_directory("test_only_pkg")
            # Node(package="ghost_pkg", executable="nope")
            '''package="ghost_doc"'''
        """,
    )
    return tmp_path


def test_scanner_allowlist_and_guards(tmp_path: Path):
    root = _fixture_repo(tmp_path)
    violations = scanner.find_violations(root)
    found = {item.package for item in violations}
    assert found == {"omnibot_metrics", "build_only_pkg", "test_only_pkg"}
    assert scanner.workspace_packages(root) == {"demo_pkg"}
    assert scanner.exec_depend_allowlist(root) == {"rclpy", "rviz2"}


def test_scanner_reports_syntax_errors(tmp_path: Path):
    _write(
        tmp_path / "pkg" / "package.xml",
        """
        <package format="3"><name>pkg</name></package>
        """,
    )
    _write(
        tmp_path / "pkg" / "broken.launch.py",
        "def generate(\n",
    )
    violations = scanner.find_violations(tmp_path)
    assert len(violations) == 1
    assert "syntax error" in violations[0].package


def test_commented_node_is_not_a_reference():
    source = textwrap.dedent(
        """
        # Node(package="ghost_pkg", executable="nope")
        Node(package="demo_pkg", executable="demo")
        """
    )
    refs = scanner.references_in_source(source, Path("demo.launch.py"))
    assert [package for _line, package, _guarded in refs] == ["demo_pkg"]


def test_public_repo_launch_files_are_guarded_or_declared():
    violations = scanner.find_violations(REPO)
    assert violations == []


def test_real_robot_launches_default_optional_packages_off():
    perception = (
        REPO / "omnibot-ros2" / "omnibot_bringup" / "launch" / "perception.launch.py"
    ).read_text(encoding="utf-8")
    pi = (
        REPO
        / "omnibot-ros2"
        / "omnibot_bringup"
        / "launch"
        / "master"
        / "omnibot_pi.launch.py"
    ).read_text(encoding="utf-8")
    robot = (
        REPO / "omnibot-ros2" / "omnibot_bringup" / "launch" / "robot.launch.py"
    ).read_text(encoding="utf-8")
    hybrid = (
        REPO / "omnibot-ros2" / "omnibot_hybrid" / "launch" / "hybrid_robot.launch.py"
    ).read_text(encoding="utf-8")

    assert _launch_arg_default(perception, "ai_perception") == "false"
    assert _launch_arg_default(pi, "ai_perception") == "false"
    assert "ai_perception:=false" in pi
    assert "ai_perception:=true" in pi
    assert _launch_arg_default(robot, "use_metrics") == "false"
    assert _launch_arg_default(robot, "use_ota") == "false"
    assert _launch_arg_default(hybrid, "use_ota") == "false"

    for source, package in (
        (perception, "omnibot_perception"),
        (pi, "omnibot_perception"),
        (robot, "omnibot_metrics"),
        (robot, "omnibot_ota"),
        (hybrid, "omnibot_ota"),
    ):
        assert package in _guarded_packages(source)


def _launch_arg_default(source: str, name: str) -> str | None:
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        func_name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", "")
        if func_name != "DeclareLaunchArgument":
            continue
        if not node.args:
            continue
        arg_name = node.args[0]
        if not isinstance(arg_name, ast.Constant) or arg_name.value != name:
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(keyword.value, ast.Constant):
                return str(keyword.value.value)
    return None


def _guarded_packages(source: str) -> set[str]:
    refs = scanner.references_in_source(source, Path("launch.py"))
    return {package for _line, package, guarded in refs if guarded}


def test_package_available_symbol_is_the_guard_helper():
    assert "PackageAvailable" in scanner.GUARD_CALLS
    assert callable(PackageAvailable)
