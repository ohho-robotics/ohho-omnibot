# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""CPU smoke check for the VR sim cameras. No ROS and no Gazebo.

vr:=true must include the wrist, BEV, and chase cameras and web_video_server.
vr:=false must leave the minimal front camera and must not bind port 8080.
"""

import importlib.util
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

BRINGUP = Path(__file__).resolve().parents[1]
LAUNCH = BRINGUP / "launch" / "sim.launch.py"
CAMERAS_LAUNCH = BRINGUP / "launch" / "sim_cameras.launch.py"
SUPPORT_PATH = BRINGUP / "launch" / "sim_cameras_support.py"
XACRO = BRINGUP.parent / "omnibot_description" / "urdf" / "omnibot.urdf.xacro"
BRIDGE = BRINGUP / "config" / "sim_bridge_cameras.yaml"
BASE_BRIDGE = BRINGUP / "config" / "sim_bridge.yaml"

_VR_IF = re.compile(
    r'<xacro:if value="\$\(arg vr_cameras\)">(.*?)</xacro:if>',
    re.S,
)


def _load_support():
    spec = importlib.util.spec_from_file_location(
        "omnibot_sim_cameras_support", SUPPORT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


support = _load_support()


def _vr_bodies(text):
    bodies = _VR_IF.findall(text)
    assert len(bodies) >= 2
    return "\n".join(bodies)


def _without_vr_bodies(text):
    return _VR_IF.sub("", text)


class TestCameraPlan:
    def test_vr_false_stays_minimal(self):
        assert support.vr_enabled("false") is False
        assert support.vr_enabled("no") is False
        assert support.xacro_flag(False) == "false"
        assert support.start_web_video("false", "true") is False
        assert support.start_web_video(False, True) is False

    def test_vr_true_includes_cameras_and_web_video(self):
        assert support.vr_enabled("true") is True
        assert support.vr_enabled("yes") is True
        assert support.vr_enabled("1") is True
        assert support.xacro_flag(True) == "true"
        assert support.start_web_video("true", "true") is True
        assert support.start_web_video("true", "false") is False
        assert support.VIDEO_PORT == 8080
        assert support.QUEST_WIDTH == 960
        assert support.QUEST_HEIGHT == 540
        assert support.QUEST_FPS == 30
        assert support.stream_path() == "/stream?topic=/sim/chase_cam/image_raw"
        assert support.IMAGE_TOPICS == (
            "/camera/front/image_raw",
            "/camera/wrist/image_raw",
            "/camera/base/bev/image_raw",
            "/sim/chase_cam/image_raw",
        )

    def test_public_bind_stays_quiet_on_loopback(self):
        assert support.bind_address("") == "127.0.0.1"
        assert support.bind_address(None) == "127.0.0.1"
        assert support.public_bind_warning("127.0.0.1") == ""
        warning = support.public_bind_warning("0.0.0.0")
        assert "0.0.0.0" in warning
        assert "without authentication" in warning

    def test_camera_info_topics_follow_the_image_topics(self):
        assert support.extra_bridge_topics() == (
            "/camera/wrist/image_raw",
            "/camera/wrist/camera_info",
            "/camera/base/bev/image_raw",
            "/camera/base/bev/camera_info",
            "/sim/chase_cam/image_raw",
            "/sim/chase_cam/camera_info",
        )
        with pytest.raises(ValueError):
            support.camera_info_topic("/camera/wrist")


class TestLaunchWiring:
    def test_vr_branch_includes_cameras_and_video_server(self):
        text = LAUNCH.read_text(encoding="utf-8")
        cameras = CAMERAS_LAUNCH.read_text(encoding="utf-8")
        assert "_cameras.vr_enabled(" in text
        assert "_cameras.xacro_flag(vr_on)" in text
        assert "minimal_sensors:=true" in text
        vr_branch = text.split("if vr_on:", 1)[1].split("return actions", 1)[0]
        assert "sim_cameras.launch.py" in vr_branch
        assert "web_video_server" not in vr_branch
        assert 'package="web_video_server"' in cameras
        assert 'executable="web_video_server"' in cameras
        assert '"port": _cameras.VIDEO_PORT' in cameras
        assert "sim_bridge_cameras.yaml" in cameras
        assert "if not video:" in cameras
        video_arg = text.split('"video",', 1)[1].split("OpaqueFunction", 1)[0]
        assert 'default_value="true"' in video_arg
        vr_arg = text.split('"vr",', 1)[1].split('"video",', 1)[0]
        assert 'default_value="false"' in vr_arg

    def test_headless_rendering_flag_stays_on_without_a_measurement(self):
        text = LAUNCH.read_text(encoding="utf-8")
        assert 'gz_args = f"-r -s --headless-rendering {world_path}"' in text
        assert "not measured" in text
        assert "960x540" in text
        assert "/stream?topic=/sim/chase_cam/image_raw" in text

    def test_default_path_does_not_rewrite_teleop_safety(self):
        text = LAUNCH.read_text(encoding="utf-8")
        assert 'executable="arm_sim_driver_node.py"' in text
        assert "teleop_safety.launch.py" not in text
        assert "cmd_vel_mux" not in text
        assert "SafetyGate" not in text

    def test_camera_bridge_lists_the_extra_topics_only(self):
        bridge = BRIDGE.read_text(encoding="utf-8")
        base = BASE_BRIDGE.read_text(encoding="utf-8")
        for topic in support.extra_bridge_topics():
            assert f'ros_topic_name: "{topic}"' in bridge
            assert f'gz_topic_name: "{topic}"' in bridge
            assert topic not in base
        assert "GZ_TO_ROS" in bridge
        assert "sensor_msgs/msg/Image" in bridge
        assert "sensor_msgs/msg/CameraInfo" in bridge
        assert "gz.msgs.Image" in bridge
        assert "gz.msgs.CameraInfo" in bridge
        assert "/camera/front/image_raw" in base
        assert "/sim/chase_cam/image_raw" not in base


class TestXacroSensors:
    def test_quest_sensors_are_behind_vr_cameras(self):
        text = XACRO.read_text(encoding="utf-8")
        ET.parse(XACRO)
        assert f'name="vr_cam_width" value="{support.QUEST_WIDTH}"' in text
        assert f'name="vr_cam_height" value="{support.QUEST_HEIGHT}"' in text
        assert f'name="vr_cam_rate" value="{support.QUEST_FPS}"' in text
        assert 'name="vr_cameras" default="false"' in text
        minimal = text.index('name="front_cam_width" value="160"')
        override = text.index('name="front_cam_width" value="${vr_cam_width}"')
        assert minimal < override
        body = _vr_bodies(text)
        outside = _without_vr_bodies(text)
        for topic in support.EXTRA_IMAGE_TOPICS:
            assert f'topic="{topic}"' in body
            assert f'topic="{topic}"' not in outside
        assert 'link_name="wrist_camera_link"' in body
        assert "<parent link=\"base_link\"/>" in body
        assert 'name="chase_camera_joint"' in body
        assert 'rpy="0 ${PI/2} 0"' in body
        assert "<width>${vr_cam_width}</width>" in text
        assert "<height>${vr_cam_height}</height>" in text
        assert "<update_rate>${vr_cam_rate}</update_rate>" in text
        wrist = re.search(
            r'<joint name="wrist_camera_joint" type="fixed">(.*?)</joint>',
            text,
            re.S,
        )
        assert wrist is not None
        assert '<parent link="arm_gripper_link"/>' in wrist.group(1)
        assert 'with_sensor="${enable_extra_cameras}"' in text
