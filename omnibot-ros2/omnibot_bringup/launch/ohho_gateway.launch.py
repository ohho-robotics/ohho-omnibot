# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Reusable ohho-gateway launch snippet (OHH-196).

Include this from robot.launch.py, omnibot_pi.launch.py, and later from a
vr:=true sim profile (OHH-198). This file owns rosbridge, foxglove, and
web_video_server. A parent that also starts them will fight for the port.

Default (gateway:=true):
  Those three servers bind 127.0.0.1. ohho-gateway listens on
  gateway_bind:gateway_port with TLS, and on 127.0.0.1:gateway_plain_port
  with plain ws/http. LAN and cloud clients use https/wss on the TLS port.

Opt out (gateway:=false):
  Prints OHHO-GATEWAY WARNING and binds the three servers to 0.0.0.0
  with no authentication.

The token is OHHO_GATEWAY_TOKEN, or gateway_token_file, or a pairing code
under gateway_state_dir (default ~/.ohho/gateway). The pairing code is
printed by the gateway process. It is not a ROS parameter.

Cloud: gateway_tls_only:=true does not open the plain loopback port.
"""

from __future__ import annotations

import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Kept in lockstep with ohho-gateway/tests/test_launch_snippet.py.
OPT_OUT_WARNING = "OHHO-GATEWAY WARNING: gateway:=false exposes rosbridge (9090), foxglove (8765), and web_video_server (8080) on 0.0.0.0 WITHOUT authentication. Anyone on the network can drive the robot. Prefer the default gateway (bearer token + TLS)."


def _parse_enabled(value: str, name: str) -> bool:
    """Accept the same strings as Jazzy ``IfCondition``.

    ``evaluate_condition_expression`` allows only ``true``/``1`` and
    ``false``/``0``. Anything else is an error so a typo such as
    ``gateway:=flase`` cannot publish rosbridge with no authentication.
    """
    normalized = value.strip().lower()
    if normalized in ("true", "1"):
        return True
    if normalized in ("false", "0"):
        return False
    raise RuntimeError(
        f"Launch argument '{name}' must be true or false, got {value!r}."
    )


def _enabled(context, name: str) -> bool:
    return _parse_enabled(LaunchConfiguration(name).perform(context), name)


def _port(context, name: str) -> int:
    return int(LaunchConfiguration(name).perform(context).strip())


def _connectivity(context, *args, **kwargs):
    del args, kwargs
    gateway_on = _enabled(context, "gateway")
    actions = []
    if not gateway_on:
        banner = (
            "****************************************************************\n"
            + OPT_OUT_WARNING
            + "\n****************************************************************"
        )
        sys.stderr.write(banner + "\n")
        sys.stdout.write(banner + "\n")
        sys.stderr.flush()
        sys.stdout.flush()

    address = "127.0.0.1" if gateway_on else "0.0.0.0"

    if _enabled(context, "start_rosbridge"):
        actions.append(
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                parameters=[
                    {
                        "port": _port(context, "rosbridge_port"),
                        "address": address,
                    }
                ],
            )
        )
    if _enabled(context, "start_foxglove"):
        actions.append(
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    {
                        "port": _port(context, "foxglove_port"),
                        "address": address,
                    }
                ],
            )
        )
    if _enabled(context, "start_web_video"):
        actions.append(
            Node(
                package="web_video_server",
                executable="web_video_server",
                name="web_video_server",
                output="screen",
                parameters=[
                    {
                        "port": _port(context, "web_video_port"),
                        "address": address,
                    }
                ],
            )
        )

    if gateway_on:
        cmd = [
            "python3",
            "-m",
            "ohho_gateway",
            "--bind",
            LaunchConfiguration("gateway_bind").perform(context),
            "--port",
            str(_port(context, "gateway_port")),
            "--plain-host",
            "127.0.0.1",
            "--plain-port",
            str(_port(context, "gateway_plain_port")),
            "--rosbridge",
            "ws://127.0.0.1:" + str(_port(context, "rosbridge_port")),
            "--video",
            "http://127.0.0.1:" + str(_port(context, "web_video_port")),
            "--foxglove",
            "ws://127.0.0.1:" + str(_port(context, "foxglove_port")),
        ]
        state_dir = LaunchConfiguration("gateway_state_dir").perform(context).strip()
        if state_dir:
            cmd.extend(["--state-dir", state_dir])
        token_file = LaunchConfiguration("gateway_token_file").perform(context).strip()
        if token_file:
            cmd.extend(["--token-file", token_file])
        if _enabled(context, "gateway_tls_only"):
            cmd.append("--tls-only")
        actions.append(
            ExecuteProcess(
                cmd=cmd,
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gateway",
                default_value="true",
                description=(
                    "Bind rosbridge, foxglove, and web_video_server to 127.0.0.1 "
                    "and publish them through ohho-gateway (TLS + bearer token). "
                    "gateway:=false exposes those servers on 0.0.0.0 with no "
                    "authentication and prints a warning."
                ),
            ),
            DeclareLaunchArgument(
                "start_rosbridge",
                default_value="true",
                description="Start rosbridge_websocket on rosbridge_port.",
            ),
            DeclareLaunchArgument(
                "start_foxglove",
                default_value="true",
                description="Start foxglove_bridge on foxglove_port.",
            ),
            DeclareLaunchArgument(
                "start_web_video",
                default_value="true",
                description="Start web_video_server on web_video_port.",
            ),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            DeclareLaunchArgument("web_video_port", default_value="8080"),
            DeclareLaunchArgument(
                "gateway_bind",
                default_value="0.0.0.0",
                description="TLS listen address. Plain ws/http stays on 127.0.0.1.",
            ),
            DeclareLaunchArgument("gateway_port", default_value="8443"),
            DeclareLaunchArgument("gateway_plain_port", default_value="9080"),
            DeclareLaunchArgument(
                "gateway_tls_only",
                default_value="false",
                description="Cloud profile: do not open the plain loopback listener.",
            ),
            DeclareLaunchArgument(
                "gateway_state_dir",
                default_value="",
                description="Cert and pairing-code directory. Empty uses ~/.ohho/gateway.",
            ),
            DeclareLaunchArgument(
                "gateway_token_file",
                default_value="",
                description=(
                    "File containing the bearer token. Empty uses "
                    "OHHO_GATEWAY_TOKEN or a generated pairing code."
                ),
            ),
            OpaqueFunction(function=_connectivity),
        ]
    )
