# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Launch-file contracts that do not need ROS 2 installed."""

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
LAUNCH = REPO / "omnibot-ros2" / "omnibot_bringup" / "launch"
GATEWAY = LAUNCH / "ohho_gateway.launch.py"
ROBOT = LAUNCH / "robot.launch.py"
PI = LAUNCH / "master" / "omnibot_pi.launch.py"
SIM = LAUNCH / "sim.launch.py"

WARNING = (
    "OHHO-GATEWAY WARNING: gateway:=false exposes rosbridge (9090), "
    "foxglove (8765), and web_video_server (8080) on 0.0.0.0 WITHOUT "
    "authentication. Anyone on the network can drive the robot. "
    "Prefer the default gateway (bearer token + TLS)."
)


def test_gateway_snippet_defaults_to_loopback_upstreams():
    text = GATEWAY.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument(\n                "gateway",\n                default_value="true",' in text
    assert 'address = "127.0.0.1" if gateway_on else "0.0.0.0"' in text
    assert WARNING in text
    assert '"--plain-host",\n            "127.0.0.1",' in text
    assert '"--token"' not in text
    assert "os.environ" not in text


def test_robot_and_pi_include_the_gateway_by_default():
    for path in (ROBOT, PI):
        text = path.read_text(encoding="utf-8")
        assert "ohho_gateway.launch.py" in text
        assert (
            'DeclareLaunchArgument(\n        "gateway",\n        default_value="true",' in text
            or 'DeclareLaunchArgument(\n            "gateway",\n            default_value="true",' in text
        )
        assert 'executable="rosbridge_websocket"' not in text
        assert 'executable="foxglove_bridge"' not in text
        assert 'executable="web_video_server"' not in text
        assert "0.0.0.0" not in text


def test_unknown_gateway_value_is_rejected():
    source = GATEWAY.read_text(encoding="utf-8")
    start = source.index("def _parse_enabled")
    end = source.index("def _enabled")
    namespace: dict = {}
    exec(source[start:end], namespace)
    parse = namespace["_parse_enabled"]
    assert parse("true", "gateway") is True
    assert parse("1", "gateway") is True
    assert parse(" TRUE ", "gateway") is True
    assert parse("false", "gateway") is False
    assert parse("0", "gateway") is False
    assert parse(" False ", "gateway") is False
    for value in ("flase", "on", "enabled", "yes", "no", ""):
        with pytest.raises(RuntimeError, match="gateway"):
            parse(value, "gateway")


def test_sim_launch_stays_on_loopback():
    text = SIM.read_text(encoding="utf-8")
    assert '"rosbridge_address"' in text
    assert 'default_value="127.0.0.1"' in text
    assert "0.0.0.0 exposes it on the network without authentication" in text
    assert "ohho_gateway.launch.py" not in text
