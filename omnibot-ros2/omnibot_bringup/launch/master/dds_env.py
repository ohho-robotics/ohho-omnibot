# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Cell DDS environment for the Pi and workstation launches.

The checked-in profile is config/dds/cyclonedds.xml. Child processes must
see RMW_IMPLEMENTATION, CYCLONEDDS_URI, ROS_DOMAIN_ID, and the peer
addresses. Values already in the environment win, so sourcing
config/ros_domain.env before launch is preserved. OMNIBOT_WORKSTATION_IP
and OMNIBOT_WS_IP are one address; the documented name wins when set.
"""

from __future__ import annotations

import os
from pathlib import Path

RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
DEFAULT_DOMAIN_ID = "30"
# RFC 5737 TEST-NET-1. Not RFC 1918, not routable.
DEFAULT_PI_IP = "192.0.2.10"
DEFAULT_WS_IP = "192.0.2.20"


def cyclonedds_xml_path() -> Path:
    """Share or source path of the Cyclone profile.

    This file lives in launch/master. The profile lives in config/dds,
    in the source tree and in the installed package share.
    """
    return (
        Path(__file__).resolve().parents[2] / "config" / "dds" / "cyclonedds.xml"
    )


def _file_uri(path: Path) -> str:
    posix = path.resolve().as_posix()
    if posix.startswith("/"):
        return "file://" + posix
    return "file:///" + posix


def _set(name: str) -> str:
    return os.environ.get(name, "").strip()


def dds_environment() -> dict[str, str]:
    """Environment the Pi and workstation nodes should inherit."""
    pi = _set("OMNIBOT_PI_IP") or DEFAULT_PI_IP
    # Same order as ros_domain.env: documented name, XML alias, TEST-NET.
    ws = (
        _set("OMNIBOT_WORKSTATION_IP")
        or _set("OMNIBOT_WS_IP")
        or DEFAULT_WS_IP
    )
    uri = _set("CYCLONEDDS_URI") or _file_uri(cyclonedds_xml_path())
    peers = _set("ROS_STATIC_PEERS") or f"{pi};{ws}"
    return {
        "RMW_IMPLEMENTATION": _set("RMW_IMPLEMENTATION") or RMW_IMPLEMENTATION,
        "ROS_DOMAIN_ID": _set("ROS_DOMAIN_ID") or DEFAULT_DOMAIN_ID,
        "CYCLONEDDS_URI": uri,
        "OMNIBOT_PI_IP": pi,
        "OMNIBOT_WS_IP": ws,
        "OMNIBOT_WORKSTATION_IP": ws,
        "ROS_STATIC_PEERS": peers,
    }


def environment_actions(action_factory):
    """Wrap dds_environment() in launch SetEnvironmentVariable actions."""
    return [action_factory(key, value) for key, value in dds_environment().items()]
