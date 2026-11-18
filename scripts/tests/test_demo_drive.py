# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""demo_drive serial transport with a mock stream, and the typed-goal path."""

from __future__ import annotations

import ast
import struct
from pathlib import Path

import pytest
from yahboom_ros2.protocol import packet_motion

import demo_drive


class MockSerial:
    """In-memory stream. This is not a device and must not call pyserial."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.writes = 0

    def write(self, data: bytes) -> int:
        self.buffer.extend(data)
        self.writes += 1
        return len(data)


def _motion_fields(packet: bytes) -> tuple[int, int, int, int]:
    payload = packet[4:-1]
    return struct.unpack("<bhhh", payload)


def test_module_does_not_import_serial_at_import_time():
    source = Path(demo_drive.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    for node in tree.body:
        if isinstance(node, ast.Import):
            assert all(alias.name != "serial" for alias in node.names)
        if isinstance(node, ast.ImportFrom):
            assert node.module != "serial"


def test_typed_goals_use_the_agent_goal_path():
    cases = {
        "go to the kitchen": ("kitchen", "navigate:kitchen"),
        "go to the desk": ("desk", "navigate:desk"),
        "go to the door": ("door", "navigate:door"),
    }
    for text, (location, mission) in cases.items():
        link = demo_drive.SimTransport()
        dispatch = demo_drive.drive(text, transport="sim", real=False, link=link)
        assert dispatch.location == location
        assert dispatch.mission_command == mission
        assert dispatch.agent_goal_payload == text
        assert dispatch.launch == demo_drive.PARITY_LAUNCH
        assert dispatch.real is False
        assert link.payloads == [text]


def test_serial_without_real_does_not_open(monkeypatch):
    def boom(*_args, **_kwargs):
        raise AssertionError("opener must not run")

    monkeypatch.setattr(demo_drive, "open_serial_port", boom)
    with pytest.raises(demo_drive.RealModeRequired, match="explicit --real"):
        demo_drive.build_dispatch("go to the kitchen", transport="serial", real=False)
    with pytest.raises(demo_drive.RealModeRequired):
        demo_drive.SerialTransport(real=False)
    with pytest.raises(demo_drive.RealModeRequired):
        demo_drive.select_transport("real", real=False)
    assert demo_drive.main(["go to the kitchen", "--transport", "serial"]) == 2


def test_plan_serial_with_real_does_not_open(monkeypatch):
    def boom(*_args, **_kwargs):
        raise AssertionError("opener must not run without --execute")

    monkeypatch.setattr(demo_drive, "open_serial_port", boom)
    dispatch = demo_drive.drive(
        "go to the kitchen",
        transport="serial",
        real=True,
        execute=False,
        opener=boom,
    )
    assert dispatch.mission_command == "navigate:kitchen"
    assert dispatch.real is True
    assert demo_drive.main(
        ["go to the door", "--transport", "serial", "--real"],
        opener=boom,
    ) == 0


def test_mock_transport_clips_deadman_and_estop():
    mock = MockSerial()
    link = demo_drive.SerialTransport(real=False, stream=mock)
    clipped = link.write_motion(1.0, -1.0, 3.0, deadman=True, estop=False)
    assert clipped == packet_motion(0.12, -0.12, 0.5)
    car, vx, vy, wz = _motion_fields(clipped)
    assert car == 1
    assert vx == 120
    assert vy == -120
    assert wz == 500
    assert abs(vx) <= int(demo_drive.SERIAL_MAX_LINEAR_MPS * 1000)
    assert abs(wz) <= int(demo_drive.SERIAL_MAX_ANGULAR_RAD_S * 1000)

    mock.buffer.clear()
    stopped = link.write_motion(0.1, 0.1, 0.1, deadman=False, estop=False)
    assert stopped == packet_motion(0.0, 0.0, 0.0)
    assert bytes(mock.buffer) == stopped

    held = link.write_motion(0.1, 0.0, 0.0, deadman=True, estop=True)
    assert held == packet_motion(0.0, 0.0, 0.0)
    assert link.opened_port is None


def test_caps_match_the_controller_ceilings():
    assert demo_drive.SERIAL_MAX_LINEAR_MPS == 0.12
    assert demo_drive.SERIAL_MAX_ANGULAR_RAD_S == 0.5


def test_serial_execute_opens_only_through_the_opener_and_writes_zeros():
    opened: list[tuple] = []
    mock = MockSerial()

    def opener(port, baud, *, real):
        assert real is True
        opened.append((port, baud, real))
        return mock

    dispatch = demo_drive.drive(
        "go to the desk",
        transport="serial",
        real=True,
        execute=True,
        opener=opener,
    )
    assert dispatch.location == "desk"
    assert opened == [(demo_drive.DEFAULT_SERIAL_PORT, demo_drive.SERIAL_BAUD, True)]
    assert bytes(mock.buffer) == packet_motion(0.0, 0.0, 0.0)
    assert mock.writes == 1


def test_mock_stream_execute_does_not_call_opener():
    mock = MockSerial()
    link = demo_drive.SerialTransport(real=False, stream=mock)

    def boom(*_args, **_kwargs):
        raise AssertionError("opener must not run when a mock stream is injected")

    demo_drive.drive(
        "go to the kitchen",
        transport="serial",
        real=True,
        execute=True,
        link=link,
        opener=boom,
    )
    assert bytes(mock.buffer) == packet_motion(0.0, 0.0, 0.0)
    assert link.goals == ["go to the kitchen"]


def test_open_serial_port_refuses_without_real(monkeypatch):
    def boom(*_args, **_kwargs):
        raise AssertionError("serial.Serial was called")

    monkeypatch.setitem(
        __import__("sys").modules,
        "serial",
        type("S", (), {"Serial": staticmethod(boom)})(),
    )
    with pytest.raises(demo_drive.RealModeRequired):
        demo_drive.open_serial_port("/dev/ttyUSB0", real=False)


def test_unparsed_goal_is_rejected():
    with pytest.raises(ValueError, match="cannot parse"):
        demo_drive.drive("pick up the cup", transport="sim", real=False)


def test_sim_execute_publishes_the_typed_goal_and_does_not_open_serial(monkeypatch):
    published: list[str] = []

    def boom(*_args, **_kwargs):
        raise AssertionError("serial opener ran on the sim path")

    monkeypatch.setattr(demo_drive, "open_serial_port", boom)
    link = demo_drive.SimTransport()
    demo_drive.drive(
        "go to the kitchen",
        transport="sim",
        real=False,
        execute=True,
        link=link,
        ros_publish=published.append,
    )
    assert published == ["go to the kitchen"]
    assert link.payloads == ["go to the kitchen"]


def test_publish_without_rclpy_is_an_error():
    with pytest.raises(RuntimeError, match="rclpy"):
        demo_drive.publish_agent_goal("go to the kitchen")


def test_main_prints_the_sim_dispatch(capsys):
    assert demo_drive.main(["go to the kitchen"]) == 0
    out = capsys.readouterr().out
    assert '"mission_command": "navigate:kitchen"' in out
    assert '"agent_goal_payload": "go to the kitchen"' in out
