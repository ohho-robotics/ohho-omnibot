# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""CPU checks for the sim VR profile. No ROS 2 install required."""

import importlib.util
from pathlib import Path

BRINGUP = Path(__file__).resolve().parents[1]
LAUNCH = BRINGUP / "launch"
REPO = Path(__file__).resolve().parents[3]
SUPPORT_PATH = LAUNCH / "sim_vr_support.py"
SIM_VR = LAUNCH / "sim_vr.launch.py"
SIM = LAUNCH / "sim.launch.py"
VR_BRIDGE = BRINGUP / "config" / "sim_bridge_vr.yaml"
DOCS = REPO / "docs" / "vr.md"


def _load_support():
    spec = importlib.util.spec_from_file_location("omnibot_sim_vr_support", SUPPORT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_defaults_stay_on_the_real_robot_contract():
    support = _load_support()
    text = SIM_VR.read_text(encoding="utf-8")
    sim = SIM.read_text(encoding="utf-8")
    bridge = VR_BRIDGE.read_text(encoding="utf-8")

    assert support.DEFAULT_WORLD == "apartment"
    assert support.DEFAULT_BIND == "127.0.0.1"
    assert support.DEFAULT_GATEWAY == "false"
    assert support.DEFAULT_NAV == "false"
    assert support.DEFAULT_ARM == "false"
    assert support.DEFAULT_CAMERAS == "front"
    assert support.DEFAULT_TELEMETRY == "false"
    assert support.DEFAULT_VIDEO == "true"
    assert support.DEFAULT_ROSBRIDGE == "true"
    assert support.VIDEO_PORT == 8080
    assert support.ROSBRIDGE_PORT == 9090
    assert support.LOOPBACK_ADDRESS == "127.0.0.1"

    for name in (
        "DEFAULT_WORLD",
        "DEFAULT_BIND",
        "DEFAULT_GATEWAY",
        "DEFAULT_NAV",
        "DEFAULT_ARM",
        "DEFAULT_CAMERAS",
        "DEFAULT_TELEMETRY",
        "DEFAULT_VIDEO",
        "DEFAULT_ROSBRIDGE",
        "VIDEO_PORT",
        "GATEWAY_LAUNCH_NAME",
        "ARM_LAUNCH_NAME",
        "CAMERA_LAUNCH_NAME",
        "TELEMETRY_LAUNCH_NAME",
    ):
        assert f"_support.{name}" in text

    assert support.GATEWAY_LAUNCH_NAME == "ohho_gateway.launch.py"
    assert support.ARM_LAUNCH_NAME == "sim_arm.launch.py"
    assert support.CAMERA_LAUNCH_NAME == "sim_cameras.launch.py"
    assert support.TELEMETRY_LAUNCH_NAME == "telemetry_aggregator.launch.py"
    assert '"vr": "true"' in text
    assert "sim.launch.py" in text
    assert 'executable="cmd_vel_mux"' not in text
    assert 'executable="arm_cmd_mux"' not in text
    assert "teleop_safety.launch.py" not in text
    # GPU container bind. Docker publishes 127.0.0.1. The support module
    # default stays loopback, and gateway:=true does not widen it.
    assert 'default_value="0.0.0.0"' in text
    assert "web_video_server" in text
    assert "arm_sim_driver" in text
    assert "does not bind 0.0.0.0" in text

    # The gate moved out of sim.launch.py into sim_teleop.launch.py so the
    # camera tests can keep forbidding that token in sim.launch.py.
    assert "teleop_safety.launch.py" not in sim
    assert "sim_bridge_vr.yaml" in sim
    sim_teleop = (LAUNCH / "sim_teleop.launch.py").read_text(encoding="utf-8")
    assert "teleop_safety.launch.py" in sim_teleop
    assert "cmd_vel_mux" in sim_teleop
    assert "arm_cmd_mux" in sim_teleop
    assert "sim_bridge_vr.yaml" in sim_teleop
    assert 'ros_topic_name: "/cmd_vel/out"' in bridge
    assert 'gz_topic_name: "/cmd_vel"' in bridge
    assert 'ros_topic_name: "/cmd_vel"' not in bridge


def test_gateway_on_stays_on_loopback_and_does_not_double_start():
    support = _load_support()
    assert support.upstream_address("true", True, "0.0.0.0") == "127.0.0.1"
    assert support.upstream_address("true", False, "0.0.0.0") == "127.0.0.1"
    assert support.upstream_address("false", False, "0.0.0.0") == "0.0.0.0"
    assert support.upstream_address("false", False, "") == "127.0.0.1"
    assert support.upstream_address("false", False, None) == "127.0.0.1"
    assert support.bind_address(" 10.1.2.3 ") == "10.1.2.3"

    owned = support.connectivity("true", True, "true", "true")
    assert owned["use_gateway_launch"] is True
    assert owned["gateway_missing"] is False
    assert owned["sim_starts_rosbridge"] is False
    assert owned["sim_starts_video"] is False

    missing = support.connectivity("true", False, "true", "true")
    assert missing["use_gateway_launch"] is False
    assert missing["gateway_missing"] is True
    assert missing["sim_starts_rosbridge"] is True
    assert missing["sim_starts_video"] is True

    default = support.connectivity("false", False, "true", "true")
    assert default["use_gateway_launch"] is False
    assert default["sim_starts_rosbridge"] is True
    assert default["sim_starts_video"] is True

    quiet = support.connectivity("false", True, "false", "false")
    assert quiet["use_gateway_launch"] is False
    assert quiet["sim_starts_rosbridge"] is False
    assert quiet["sim_starts_video"] is False


def test_include_points_skip_until_the_file_exists(tmp_path):
    support = _load_support()
    present = tmp_path / "sim_arm.launch.py"
    present.write_text("placeholder\n", encoding="utf-8")
    assert support.optional_launch("false", True) == "skip"
    assert support.optional_launch(False, True) == "skip"
    assert support.optional_launch("true", False) == "missing"
    assert support.optional_launch("yes", present.is_file()) == "include"
    assert support.camera_mode("front") == "front"
    assert support.camera_mode("") == "front"
    assert support.camera_mode("extra") == "extra"
    assert support.camera_mode("wrist") == "unknown"


def test_docs_list_the_shared_topic_contract():
    text = DOCS.read_text(encoding="utf-8")
    for topic in (
        "/cmd_vel/teleop_raw",
        "/cmd_vel/teleop",
        "/cmd_vel/out",
        "/cmd_vel/vla",
        "/cmd_vel/rl",
        "/control_mode",
        "/emergency_stop",
        "/ohho/heartbeat",
        "/ohho/estop_reset",
        "/ohho/safety/state",
        "/arm/joint_commands/teleop_raw",
        "/arm/joint_commands/teleop",
        "/arm/joint_commands/out",
        "/camera/front/image_raw",
        "/odom",
        "/scan",
    ):
        assert topic in text
    assert "127.0.0.1" in text
    assert "9090" in text
    assert "8080" in text
    assert "sim_vr.launch.py" in text
    assert "nav:=true" in text
    assert "ohho_gateway.launch.py" in text
    assert "sim_arm.launch.py" in text
    assert "sim_cameras.launch.py" in text
    assert "telemetry_aggregator.launch.py" in text
