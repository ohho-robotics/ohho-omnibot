"""Static checks for bev_real_cameras.launch.py (no ROS runtime)."""

import ast
from pathlib import Path

import yaml

PKG_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PKG_ROOT / "launch" / "bev_real_cameras.launch.py"
REAL_PARAMS = PKG_ROOT / "config" / "bev_params_real.yaml"


def test_launch_file_parses():
    source = LAUNCH.read_text(encoding="utf-8")
    ast.parse(source, filename=str(LAUNCH))


def test_launch_wires_real_robot_topics():
    text = LAUNCH.read_text(encoding="utf-8")
    assert "bev_real_cameras" in text or "bev_params_real.yaml" in text
    assert "bev_stitcher" in text
    assert "ros2_bev_stitcher" in text
    assert "generate_launch_description" in text


def test_real_params_yaml_matches_omnibot_layout():
    data = yaml.safe_load(REAL_PARAMS.read_text(encoding="utf-8"))
    params = data["bev_stitcher"]["ros__parameters"]
    assert params["output_topic"] == "/camera/base/bev/image_raw"
    assert params["input_topic_pattern"] == "/camera/{name}/image_raw"
    assert params["camera_names"] == ["front", "rear", "left", "right"]
    assert params["use_tf"] is True
    assert params["use_camera_info"] is True
