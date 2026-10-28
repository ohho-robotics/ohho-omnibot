# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Pure decisions for sim_vr.launch.py. No ROS imports.

sim_vr.launch.py loads this file by path. The CPU test loads it the same way.
The shared topic list is config/vr_topic_contract.yaml. Loading it checks
that the ports and the front camera still match this profile.
"""

import os

LOOPBACK_ADDRESS = "127.0.0.1"

DEFAULT_WORLD = "apartment"
DEFAULT_DRIVE = "planar"
DEFAULT_GUI = "false"
DEFAULT_ROSBRIDGE = "true"
DEFAULT_BIND = LOOPBACK_ADDRESS
DEFAULT_GATEWAY = "false"
DEFAULT_VIDEO = "true"
DEFAULT_NAV = "false"
DEFAULT_ARM = "false"
DEFAULT_CAMERAS = "front"
DEFAULT_TELEMETRY = "false"

VIDEO_PORT = 8080
ROSBRIDGE_PORT = 9090

GATEWAY_LAUNCH_NAME = "ohho_gateway.launch.py"
ARM_LAUNCH_NAME = "sim_arm.launch.py"
CAMERA_LAUNCH_NAME = "sim_cameras.launch.py"
TELEMETRY_LAUNCH_NAME = "telemetry_aggregator.launch.py"


def truthy(raw):
    """True for 1 / true / yes. Booleans stringify to True and False."""
    return str(raw).strip().lower() in ("1", "true", "yes")


def bind_address(raw):
    """Return a bind address. Empty input stays on loopback.

    An explicit address is returned unchanged. This function does not
    widen a missing value to every interface.
    """
    if raw is None:
        return LOOPBACK_ADDRESS
    text = str(raw).strip()
    if not text:
        return LOOPBACK_ADDRESS
    return text


def camera_mode(raw):
    """front (the camera sim.launch.py already starts), extra, or unknown."""
    text = str(raw).strip().lower()
    if text in ("front", ""):
        return "front"
    if text == "extra":
        return "extra"
    return "unknown"


def optional_launch(enabled, path_exists):
    """skip, include, or missing.

    missing means the operator asked for a piece whose launch file is not
    installed yet. The caller logs that and keeps starting the profile.
    """
    if not truthy(enabled):
        return "skip"
    if path_exists:
        return "include"
    return "missing"


def connectivity(gateway, gateway_launch_exists, rosbridge, video):
    """Who starts rosbridge and web_video_server.

    The gateway snippet, when it is installed and gateway is true, owns
    both servers so port 9090 is claimed once. A missing snippet does not
    turn the bind into 0.0.0.0; the sim profile still starts the servers.
    """
    gateway_on = truthy(gateway)
    use_gateway = gateway_on and bool(gateway_launch_exists)
    return {
        "use_gateway_launch": use_gateway,
        "gateway_missing": gateway_on and not bool(gateway_launch_exists),
        "sim_starts_rosbridge": (not use_gateway) and truthy(rosbridge),
        "sim_starts_video": (not use_gateway) and truthy(video),
    }


def upstream_address(gateway, gateway_launch_exists, rosbridge_address):
    """Bind address for rosbridge and web_video_server.

    gateway:=true stays on loopback even if rosbridge_address is wider and
    even if ohho_gateway.launch.py is not installed yet. A public bind in
    that mode would skip the proxy. gateway_launch_exists is accepted so
    callers can keep one signature with connectivity(); it does not widen
    the bind. With the gateway off, an explicit rosbridge_address is used
    and an empty value stays on loopback.
    """
    del gateway_launch_exists
    if truthy(gateway):
        return LOOPBACK_ADDRESS
    return bind_address(rosbridge_address)


def _load_contract_module():
    import importlib.util

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vr_contract.py")
    spec = importlib.util.spec_from_file_location("omnibot_vr_contract_sim", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_contract_mod = _load_contract_module()
CONTRACT_PATH = _contract_mod.default_contract_path(__file__)
_CONTRACT = _contract_mod.load_vr_contract(CONTRACT_PATH)
if _CONTRACT["ports"]["bind"] != LOOPBACK_ADDRESS:
    raise RuntimeError("vr_topic_contract.yaml bind drifted from the sim VR profile")
if _CONTRACT["ports"]["web_video"] != VIDEO_PORT:
    raise RuntimeError("vr_topic_contract.yaml web_video port drifted")
if _CONTRACT["ports"]["rosbridge"] != ROSBRIDGE_PORT:
    raise RuntimeError("vr_topic_contract.yaml rosbridge port drifted")
if "/camera/front/image_raw" not in _contract_mod.video_topics(_CONTRACT):
    raise RuntimeError("vr_topic_contract.yaml dropped the sim front camera")


def video_stream_topics():
    """Topics from the shared contract. sim_vr does not retarget its bridge."""
    return _contract_mod.video_topics(_CONTRACT)
