# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Tests for record.launch.py topics and Foxglove layout validation.

Runs without ROS 2 installed (pure Python/pytest), and also passes in ROS 2 / colcon test.
"""

import ast
import importlib.util
import json
from pathlib import Path

import pytest

THIS_DIR = Path(__file__).resolve().parent
BRINGUP_DIR = THIS_DIR.parent
BRINGUP_LAUNCH = BRINGUP_DIR / "launch" / "record.launch.py"


def find_repo_root() -> Path | None:
    for parent in [THIS_DIR] + list(THIS_DIR.parents):
        if (parent / ".git").exists() or (
            (parent / "foxglove" / "omnibot_layout.json").is_file()
            and (parent / "README.md").is_file()
        ):
            return parent
    return None


REPO_ROOT = find_repo_root()
ROOT_LAUNCH = REPO_ROOT / "record.launch.py" if REPO_ROOT else None
FOXGLOVE_LAYOUT = REPO_ROOT / "foxglove" / "omnibot_layout.json" if REPO_ROOT else None
TWIN_FOXGLOVE_LAYOUT = (
    REPO_ROOT / "omnibot-digital-twin" / "configs" / "foxglove" / "omnibot_layout.json"
    if REPO_ROOT
    else None
)
README_PATH = REPO_ROOT / "README.md" if REPO_ROOT else None


def _load_module(path: Path):
    spec = importlib.util.spec_from_file_location("test_record_mod", str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_record_launch_files_exist():
    assert BRINGUP_LAUNCH.is_file(), f"Missing {BRINGUP_LAUNCH}"
    if ROOT_LAUNCH:
        assert ROOT_LAUNCH.is_file(), f"Missing {ROOT_LAUNCH}"


def test_record_launch_default_topics():
    """Verify record.launch.py defines all required topics from OHH-34 criteria."""
    mod = _load_module(BRINGUP_LAUNCH)
    topics = mod.DEFAULT_TOPICS

    # Core required topics from issue description
    required = ["/tf", "/odom", "/scan", "/camera", "/agent/trace"]
    for req in required:
        assert req in topics, f"Expected {req} in DEFAULT_TOPICS: {topics}"

    # Related topics for completeness in Foxglove replay
    assert "/tf_static" in topics
    assert "/camera/front/image_raw" in topics
    assert "/camera/wrist/image_raw" in topics
    assert "/camera/base/bev/image_raw" in topics
    assert "/agent/loop_trace" in topics


def test_root_launch_wrapper_reexports_topics():
    if not ROOT_LAUNCH or not ROOT_LAUNCH.is_file():
        pytest.skip("Root launch wrapper not found in standalone package test")
    mod = _load_module(ROOT_LAUNCH)
    bringup_mod = _load_module(BRINGUP_LAUNCH)
    assert mod.DEFAULT_TOPICS == bringup_mod.DEFAULT_TOPICS


def test_record_launch_mcap_storage_flag():
    """Verify record.launch.py configures storage identifier as mcap."""
    text = BRINGUP_LAUNCH.read_text(encoding="utf-8")
    assert '"-s"' in text or "'-s'" in text
    assert '"mcap"' in text or "'mcap'" in text
    assert "ros2" in text
    assert "record" in text


def test_record_launch_arguments_declared():
    """Verify launch argument names via AST parsing."""
    tree = ast.parse(BRINGUP_LAUNCH.read_text(encoding="utf-8"))
    args_declared = []
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and getattr(node.func, "id", None) == "DeclareLaunchArgument"
        ):
            if node.args and isinstance(node.args[0], ast.Constant):
                args_declared.append(node.args[0].value)

    assert "bag_path" in args_declared
    assert "topics" in args_declared
    assert "use_sim_time" in args_declared
    assert "preset_profile" in args_declared


def test_foxglove_json_validity():
    """Verify foxglove/omnibot_layout.json is valid Foxglove Studio layout JSON."""
    if not FOXGLOVE_LAYOUT or not FOXGLOVE_LAYOUT.is_file():
        pytest.skip("Foxglove layout not found in isolated package test")
    data = json.loads(FOXGLOVE_LAYOUT.read_text(encoding="utf-8"))

    # Required top-level keys for Foxglove Studio
    for key in ("configById", "layout", "version"):
        assert key in data, f"Missing key '{key}' in foxglove layout"

    assert data["version"].startswith("foxglove.studio")

    config = data["configById"]
    assert "3D!main" in config
    assert "Plot!odom" in config
    assert "RawMessages!agent_trace" in config

    # Check 3D panel topics
    main_3d_topics = config["3D!main"].get("topics", {})
    assert "/tf" in main_3d_topics
    assert "/odom" in main_3d_topics
    assert "/scan" in main_3d_topics

    # Check agent trace topic
    assert config["RawMessages!agent_trace"].get("topicPath") == "/agent/trace"

    # Check camera topics
    image_panels = [k for k in config if k.startswith("Image!")]
    assert len(image_panels) >= 3
    camera_topics = [config[k].get("cameraTopic") for k in image_panels]
    assert any("/camera/front" in t for t in camera_topics)

    # Check layout references
    def _collect_panels(node):
        if isinstance(node, str):
            return [node]
        if isinstance(node, dict):
            panels = []
            if "first" in node:
                panels.extend(_collect_panels(node["first"]))
            if "second" in node:
                panels.extend(_collect_panels(node["second"]))
            return panels
        return []

    layout_panels = _collect_panels(data["layout"])
    for p in layout_panels:
        assert p in config, f"Panel {p} in layout tree is not defined in configById"


def test_twin_foxglove_json_in_sync():
    """Verify omnibot-digital-twin layout stays in sync with foxglove/omnibot_layout.json."""
    if not FOXGLOVE_LAYOUT or not TWIN_FOXGLOVE_LAYOUT:
        pytest.skip("Foxglove layouts not found in isolated package test")
    if not FOXGLOVE_LAYOUT.is_file() or not TWIN_FOXGLOVE_LAYOUT.is_file():
        pytest.skip("Foxglove layouts not found in isolated package test")
    root_data = json.loads(FOXGLOVE_LAYOUT.read_text(encoding="utf-8"))
    twin_data = json.loads(TWIN_FOXGLOVE_LAYOUT.read_text(encoding="utf-8"))
    assert root_data == twin_data


def test_readme_has_replay_section():
    """Verify README has the Replay a run section mentioning MCAP and Foxglove."""
    if not README_PATH or not README_PATH.is_file():
        pytest.skip("README not found in isolated package test")
    readme_text = README_PATH.read_text(encoding="utf-8")
    assert "## Replay a run (MCAP + Foxglove)" in readme_text
    assert "record.launch.py" in readme_text
    assert "foxglove/omnibot_layout.json" in readme_text
    assert "/tf" in readme_text
    assert "/odom" in readme_text
    assert "/scan" in readme_text
    assert "/camera" in readme_text
    assert "/agent/trace" in readme_text
