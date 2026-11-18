#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Send one typed navigation goal on the sim path or the serial path.

    python scripts/demo_drive.py "go to the kitchen"
    python scripts/demo_drive.py "go to the desk" --execute
    python scripts/demo_drive.py "go to the door" --transport serial --real

The goal text is the same string the agent consumes on ``/agent/goal``
(``go to the kitchen`` → mission command ``navigate:kitchen``). Default
transport is ``sim``. ``--transport serial`` is real-robot mode and refuses
to run unless ``--real`` is set. Nothing in this module opens a serial port
unless that flag is set and ``--execute`` is set. Pass a stream object (the
unit tests use a mock) to exercise the packet path without a device.

Speed caps match ``yahboom_controller_node.send_motion_command`` (0.12 m/s,
0.5 rad/s) and are not raised here. A non-zero packet is written only when
the caller holds deadman and e-stop is clear. A typed goal never produces a
non-zero packet: Nav2 owns the motion, and the only execute-time serial write
is a zero-velocity packet.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Protocol

_REPO = Path(__file__).resolve().parents[1]
for _rel in ("omnibot-ai-engines", "yahboom-python-driver"):
    _entry = str(_REPO / _rel)
    if _entry not in sys.path:
        sys.path.insert(0, _entry)

from agent_engine.reasoners.scripted_brain import parse_navigation_location
from yahboom_ros2.protocol import packet_motion

# yahboom_controller_node.send_motion_command clips to these before FUNC_MOTION.
# Do not raise them. Joy turbo scales in that node are the same ceilings.
SERIAL_MAX_LINEAR_MPS = 0.12
SERIAL_MAX_ANGULAR_RAD_S = 0.5

# Same launch file for a sim session and a real session. The clock argument
# differs (use_sim_time); the goal still enters on /agent/goal.
PARITY_LAUNCH = "ros2 launch omnibot_orchestration agent_nav2.launch.py"
SIM_STACK_LAUNCH = "ros2 launch omnibot_navigation sim_navigation.launch.py"

DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

SERIAL_REQUIRES_REAL = (
    "serial transport is real-robot mode and requires explicit --real"
)
REAL_MODE_REQUIRES_FLAG = "real mode requires explicit --real"


class RealModeRequired(RuntimeError):
    """Real-robot mode or a serial-port open was requested without --real."""


class ByteStream(Protocol):
    def write(self, data: bytes) -> int: ...


@dataclass(frozen=True)
class GoalDispatch:
    """One typed goal after it has been accepted on a transport."""

    goal_text: str
    location: str
    mission_command: str
    agent_goal_payload: str
    transport: str
    real: bool
    launch: str

    def to_dict(self) -> dict:
        return asdict(self)


def select_transport(mode: str, *, real: bool) -> str:
    """Map a session mode to a transport. Real mode requires ``real=True``."""
    if mode == "sim":
        return "sim"
    if mode == "real":
        if not real:
            raise RealModeRequired(REAL_MODE_REQUIRES_FLAG)
        return "serial"
    raise ValueError(f"unknown mode {mode!r}")


def build_dispatch(goal: str, *, transport: str, real: bool) -> GoalDispatch:
    """Parse a typed goal. Serial transport raises if ``real`` is false.

    Does not publish, and does not open a port.
    """
    if transport not in ("sim", "serial"):
        raise ValueError(f"unknown transport {transport!r}")
    if transport == "serial" and not real:
        raise RealModeRequired(SERIAL_REQUIRES_REAL)
    text = goal.strip()
    location = parse_navigation_location(text)
    if location is None:
        raise ValueError(f"cannot parse navigation goal from {goal!r}")
    return GoalDispatch(
        goal_text=text,
        location=location,
        mission_command=f"navigate:{location}",
        agent_goal_payload=text,
        transport=transport,
        real=real,
        launch=PARITY_LAUNCH,
    )


def _clip(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return float(value)


def motion_packet(
    vx: float,
    vy: float,
    wz: float,
    *,
    deadman: bool = False,
    estop: bool = False,
) -> bytes:
    """Yahboom FUNC_MOTION bytes, clipped, deadman-gated, and e-stop zeroed.

    E-stop wins over deadman. Deadman defaults to released, so a caller who
    forgets it gets a zero packet rather than motion.
    """
    if estop or not deadman:
        vx, vy, wz = 0.0, 0.0, 0.0
    else:
        vx = _clip(vx, SERIAL_MAX_LINEAR_MPS)
        vy = _clip(vy, SERIAL_MAX_LINEAR_MPS)
        wz = _clip(wz, SERIAL_MAX_ANGULAR_RAD_S)
    return packet_motion(vx, vy, wz)


class SimTransport:
    """In-memory stand-in for publishing ``/agent/goal``. No ROS, no serial."""

    def __init__(self) -> None:
        self.payloads: list[str] = []

    def deliver(self, dispatch: GoalDispatch) -> None:
        if dispatch.transport != "sim":
            raise ValueError(
                f"sim transport cannot deliver {dispatch.transport!r}"
            )
        self.payloads.append(dispatch.agent_goal_payload)


class SerialTransport:
    """Serial FUNC_MOTION writer.

    A device is opened only by :meth:`open` when ``real`` is true and no
    stream was injected. Tests inject a mock stream and leave ``real`` false;
    that path never calls the opener.
    """

    def __init__(
        self,
        *,
        real: bool,
        stream: ByteStream | None = None,
        port: str | None = None,
        opener: Callable[..., ByteStream] | None = None,
    ) -> None:
        if stream is None and not real:
            raise RealModeRequired(SERIAL_REQUIRES_REAL)
        self.real = real
        self.goals: list[str] = []
        self._stream = stream
        self._port = port or DEFAULT_SERIAL_PORT
        self._opener = opener or open_serial_port
        self.opened_port: str | None = None

    def deliver(self, dispatch: GoalDispatch) -> None:
        if dispatch.transport != "serial":
            raise ValueError(
                f"serial transport cannot deliver {dispatch.transport!r}"
            )
        # A navigation goal is not a velocity. Record it; do not command motion.
        self.goals.append(dispatch.agent_goal_payload)

    def open(self) -> ByteStream:
        if self._stream is not None:
            return self._stream
        if not self.real:
            raise RealModeRequired(SERIAL_REQUIRES_REAL)
        self._stream = self._opener(self._port, baud=SERIAL_BAUD, real=True)
        self.opened_port = self._port
        return self._stream

    def write_motion(
        self,
        vx: float,
        vy: float,
        wz: float,
        *,
        deadman: bool = False,
        estop: bool = False,
    ) -> bytes:
        packet = motion_packet(vx, vy, wz, deadman=deadman, estop=estop)
        self.open().write(packet)
        return packet


def open_serial_port(port: str, baud: int = SERIAL_BAUD, *, real: bool = False):
    """Open a Yahboom serial port. Refuses unless ``real`` is true.

    ``serial`` is imported here so importing this module does not touch pyserial.
    """
    if not real:
        raise RealModeRequired(SERIAL_REQUIRES_REAL)
    import serial

    return serial.Serial(port=port, baudrate=baud, timeout=1.0)


def publish_agent_goal(payload: str) -> None:
    """Publish one typed goal on ``/agent/goal``. Requires rclpy."""
    try:
        import rclpy
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError(
            "cannot publish /agent/goal: rclpy is not installed"
        ) from exc

    rclpy.init(args=None)
    node = None
    try:
        node = rclpy.create_node("demo_drive")
        publisher = node.create_publisher(String, "/agent/goal", 10)
        message = String()
        message.data = payload
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def drive(
    goal: str,
    *,
    transport: str = "sim",
    real: bool = False,
    execute: bool = False,
    port: str | None = None,
    link: SimTransport | SerialTransport | None = None,
    opener: Callable[..., ByteStream] = open_serial_port,
    ros_publish: Callable[[str], None] = publish_agent_goal,
) -> GoalDispatch:
    """Deliver one typed goal on the selected transport.

    ``execute`` publishes ``/agent/goal`` for sim. For serial it opens the
    port (``--real`` required) and writes a zero-velocity packet only.
    """
    dispatch = build_dispatch(goal, transport=transport, real=real)
    if transport == "sim":
        sink = link if link is not None else SimTransport()
        if not isinstance(sink, SimTransport):
            raise TypeError("sim transport requires SimTransport")
        sink.deliver(dispatch)
        if execute:
            ros_publish(dispatch.agent_goal_payload)
        return dispatch

    sink = link if link is not None else SerialTransport(
        real=True, port=port, opener=opener
    )
    if not isinstance(sink, SerialTransport):
        raise TypeError("serial transport requires SerialTransport")
    sink.deliver(dispatch)
    if execute:
        sink.open()
        sink.write_motion(0.0, 0.0, 0.0, deadman=False, estop=True)
    return dispatch


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Send a typed goal (for example 'go to the kitchen') on the "
            "sim path or the serial path. Serial requires --real. "
            "Without --execute this only prints the dispatch."
        )
    )
    parser.add_argument("goal", help="typed goal, e.g. 'go to the kitchen'")
    parser.add_argument(
        "--transport",
        choices=("sim", "serial"),
        default="sim",
        help="sim publishes /agent/goal; serial is the Yahboom packet path",
    )
    parser.add_argument(
        "--real",
        action="store_true",
        help="required for --transport serial and for any serial port open",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="sim: publish /agent/goal. serial: open the port and send a zero packet",
    )
    parser.add_argument(
        "--port",
        default=None,
        help=f"serial device (default {DEFAULT_SERIAL_PORT}), used only with --execute",
    )
    return parser


def main(
    argv: list[str] | None = None,
    *,
    opener: Callable[..., ByteStream] = open_serial_port,
    ros_publish: Callable[[str], None] = publish_agent_goal,
    link: SimTransport | SerialTransport | None = None,
) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.transport == "serial":
            select_transport("real", real=args.real)
        dispatch = drive(
            args.goal,
            transport=args.transport,
            real=args.real,
            execute=args.execute,
            port=args.port,
            link=link,
            opener=opener,
            ros_publish=ros_publish,
        )
    except RealModeRequired as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except (ValueError, TypeError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print(json.dumps(dispatch.to_dict(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
