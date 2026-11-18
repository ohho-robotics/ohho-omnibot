# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Pure decisions for robot_vr.launch.py. No ROS imports.

The real-robot VR profile reads vr_topic_contract.yaml, the same file
sim_vr.launch.py checks. teleop_safety stays in front of both muxes.
rosbridge, foxglove, and web_video_server stay on loopback. The gateway
launch, when it is installed, is the authenticated listener. This module
never selects a wildcard bind.
"""

from __future__ import annotations

import importlib.util
import os


def _load_contract_module():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vr_contract.py")
    spec = importlib.util.spec_from_file_location("omnibot_vr_contract_robot", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_contract_mod = _load_contract_module()

CONTRACT_PATH = _contract_mod.default_contract_path(__file__)
CONTRACT = _contract_mod.load_vr_contract(CONTRACT_PATH)

LOOPBACK_ADDRESS = CONTRACT["ports"]["bind"]
DEFAULT_BIND = LOOPBACK_ADDRESS
DEFAULT_GATEWAY = "true"
DEFAULT_ROSBRIDGE = "true"
DEFAULT_VIDEO = "true"
DEFAULT_VIDEO_BACKEND = "web"
DEFAULT_FOXGLOVE = "true"
DEFAULT_ARM = "true"
DEFAULT_TELEMETRY = "true"
DEFAULT_DESCRIPTOR = "true"
DEFAULT_EKF = "true"
DEFAULT_BEV = "true"
DEFAULT_DEPTH = "false"
DEFAULT_ROBOT_ID = ""

VIDEO_PORT = CONTRACT["ports"]["web_video"]
ROSBRIDGE_PORT = CONTRACT["ports"]["rosbridge"]
FOXGLOVE_PORT = CONTRACT["ports"]["foxglove"]

GATEWAY_LAUNCH_NAME = "ohho_gateway.launch.py"
TELEMETRY_LAUNCH_NAME = "telemetry.launch.py"
TELEOP_SAFETY_LAUNCH_NAME = "teleop_safety.launch.py"
WEBRTC_LAUNCH_NAME = "webrtc_streamer.launch.py"
ASTRA_LAUNCH_NAME = "astra_pro.launch.xml"
DESCRIPTOR_SCRIPT_NAME = "descriptor_node.py"
DESCRIPTOR_YAML_NAME = "robot_descriptor.yaml"

PUBLIC_BINDS = frozenset({"0.0.0.0", "::", "[::]", "*"})
LOOPBACK_NAMES = frozenset({"127.0.0.1", "localhost"})

FORBIDDEN_EXECUTABLES = frozenset(
    {
        "teleop_twist_joy",
        "joy_node",
        "teleop_twist_keyboard",
        "keyboard_teleop",
    }
)

FORBIDDEN_PARAM_KEYS = frozenset(
    {
        "max_linear_vel",
        "max_angular_vel",
        "linear_limit",
        "angular_limit",
        "max_speed",
        "source_timeout_s",
        "watchdog_timeout",
        "command_timeout_sec",
        "max_joint_delta_rad",
    }
)

CAMERA_ORDER = ("front", "rear", "left", "right", "wrist")
BEV_CAMERAS = ("front", "rear", "left", "right")
VR_CAMERAS = ("front", "wrist")

CAMERA_FRAMES = {
    "front": "camera_front_optical_frame",
    "rear": "camera_rear_optical_frame",
    "left": "camera_left_optical_frame",
    "right": "camera_right_optical_frame",
    "wrist": "camera_wrist_optical_frame",
}

CAMERA_PARAMS = {
    "framerate": 30.0,
    "image_width": 640,
    "image_height": 480,
    "pixel_format": "mjpeg2rgb",
    "io_method": "mmap",
}


def truthy(raw):
    return str(raw).strip().lower() in ("1", "true", "yes")


def video_stream_topics():
    return _contract_mod.video_topics(CONTRACT)


def loopback_bind(requested):
    """Return ``(address, clamped)``.

    Any address other than loopback is clamped. The gateway snippet is
    the process that may listen for clients. rosbridge itself stays here.
    """
    text = "" if requested is None else str(requested).strip()
    if text in ("", "localhost"):
        return LOOPBACK_ADDRESS, text in PUBLIC_BINDS
    if text == LOOPBACK_ADDRESS:
        return LOOPBACK_ADDRESS, False
    return LOOPBACK_ADDRESS, True


def _existing_file(path):
    return bool(path) and os.path.isfile(path)


def descriptor_ready(bringup_share):
    """True when OHH-214's publisher and YAML are both on disk."""
    share = os.path.abspath(bringup_share)
    prefix = os.path.dirname(os.path.dirname(share))
    scripts = [
        os.path.join(share, "scripts", DESCRIPTOR_SCRIPT_NAME),
        os.path.join(prefix, "lib", "omnibot_bringup", DESCRIPTOR_SCRIPT_NAME),
    ]
    yaml_path = os.path.join(share, "config", DESCRIPTOR_YAML_NAME)
    return _existing_file(yaml_path) and any(_existing_file(path) for path in scripts)


def _node(package, executable, name, parameters=None, remappings=None, namespace=""):
    return {
        "kind": "node",
        "package": package,
        "executable": executable,
        "name": name,
        "namespace": namespace,
        "parameters": list(parameters or []),
        "remappings": list(remappings or []),
    }


def _include(label, path, arguments=None, loader="python"):
    return {
        "kind": "include",
        "label": label,
        "path": path,
        "loader": loader,
        "arguments": list(arguments or []),
    }


def _log(msg):
    return {"kind": "log", "msg": msg}


def _camera_names(bev):
    if bev:
        names = list(BEV_CAMERAS) + ["wrist"]
    else:
        names = list(VR_CAMERAS)
    return [name for name in CAMERA_ORDER if name in names]


def _camera_node(name, device):
    params = dict(CAMERA_PARAMS)
    params["video_device"] = device
    params["camera_name"] = name
    params["camera_frame_id"] = CAMERA_FRAMES[name]
    params["use_sim_time"] = False
    return _node(
        "usb_cam",
        "usb_cam_node_exe",
        f"usb_cam_{name}",
        parameters=[params],
        namespace=f"camera/{name}",
    )


def _stream_urls(port, bind):
    return [
        f"http://{bind}:{port}/stream?topic={topic}"
        for topic in video_stream_topics()
    ]


def _validate(actions):
    names = []
    includes = []
    for action in actions:
        if action["kind"] == "node":
            names.append(action["name"])
            if action["executable"] in FORBIDDEN_EXECUTABLES:
                raise RuntimeError(
                    f"VR profile must not start {action['executable']}"
                )
            for params in action["parameters"]:
                if isinstance(params, dict):
                    overlap = FORBIDDEN_PARAM_KEYS & set(params)
                    if overlap:
                        raise RuntimeError(
                            "VR profile must not set "
                            + ", ".join(sorted(overlap))
                        )
                    if params.get("address") in PUBLIC_BINDS:
                        raise RuntimeError(
                            f"{action['name']} must not bind {params.get('address')}"
                        )
            for source, target in action["remappings"]:
                if target in ("/cmd_vel/teleop", "/arm/joint_commands/teleop"):
                    raise RuntimeError(
                        "a launch remap must not feed a mux teleop input"
                    )
                if source in ("/cmd_vel/teleop_raw", "/arm/joint_commands/teleop_raw"):
                    raise RuntimeError(
                        "a launch remap must not move a raw teleop topic"
                    )
        elif action["kind"] == "include":
            includes.append(action)
            if action["label"] == "teleop_safety" and action["arguments"]:
                raise RuntimeError("teleop_safety include takes no arguments")
            if action["label"] == "gateway":
                args = dict(action["arguments"])
                if args.get("gateway") != "true":
                    raise RuntimeError("gateway include must stay gateway:=true")

    def _pos(label_or_name, kind):
        for index, action in enumerate(actions):
            if kind == "include" and action["kind"] == "include":
                if action["label"] == label_or_name:
                    return index
            if kind == "node" and action["kind"] == "node":
                if action["name"] == label_or_name:
                    return index
        return None

    safety_at = _pos("teleop_safety", "include")
    base_at = _pos("cmd_vel_mux", "node")
    arm_at = _pos("arm_cmd_mux", "node")
    if safety_at is None or base_at is None or arm_at is None:
        raise RuntimeError("teleop_safety and both muxes are required")
    if not (safety_at < base_at and safety_at < arm_at):
        raise RuntimeError("teleop_safety must start before both muxes")

    gateway = [item for item in includes if item["label"] == "gateway"]
    rosbridge = [action for action in actions if action["kind"] == "node" and action["name"] == "rosbridge_websocket"]
    if gateway and rosbridge:
        raise RuntimeError("gateway and the parent must not both start rosbridge")
    if len(rosbridge) > 1 or names.count("cmd_vel_mux") != 1:
        raise RuntimeError("VR profile started a control node more than once")
    if names.count("teleop_safety") != 0:
        raise RuntimeError("teleop_safety is an include, not a second node")
    return actions


def plan(
    shares,
    *,
    gateway=DEFAULT_GATEWAY,
    rosbridge=DEFAULT_ROSBRIDGE,
    video=DEFAULT_VIDEO,
    video_backend=DEFAULT_VIDEO_BACKEND,
    foxglove=DEFAULT_FOXGLOVE,
    arm=DEFAULT_ARM,
    telemetry=DEFAULT_TELEMETRY,
    descriptor=DEFAULT_DESCRIPTOR,
    ekf=DEFAULT_EKF,
    bev=DEFAULT_BEV,
    depth_camera=DEFAULT_DEPTH,
    robot_id=DEFAULT_ROBOT_ID,
    requested_bind=DEFAULT_BIND,
    camera_devices=None,
    gateway_launch_exists=None,
    descriptor_installed=None,
    webrtc_launch_exists=None,
    astra_launch_exists=None,
    contract=None,
):
    """Return the ordered action list for the real-robot VR profile."""
    contract = CONTRACT if contract is None else contract
    bringup = shares["omnibot_bringup"]
    hybrid = shares["omnibot_hybrid"]
    arm_share = shares.get("omnibot_arm", "")
    description = shares.get("omnibot_description", "")
    devices = dict(camera_devices or {})

    bind, clamped = loopback_bind(requested_bind)
    gateway_on = truthy(gateway)
    gateway_path = os.path.join(bringup, "launch", GATEWAY_LAUNCH_NAME)
    if gateway_launch_exists is None:
        gateway_launch_exists = _existing_file(gateway_path)
    use_gateway = gateway_on and bool(gateway_launch_exists)

    if descriptor_installed is None:
        descriptor_on = truthy(descriptor) and descriptor_ready(bringup)
        descriptor_yaml = os.path.join(bringup, "config", DESCRIPTOR_YAML_NAME)
    else:
        descriptor_on = truthy(descriptor) and bool(descriptor_installed)
        descriptor_yaml = os.path.join(bringup, "config", DESCRIPTOR_YAML_NAME)

    webrtc_path = os.path.join(bringup, "launch", WEBRTC_LAUNCH_NAME)
    # OHH-210 installs the streamer in its own package. The bringup copy
    # is only a local include point. Callers pass an explicit path via
    # shares["ohho_webrtc"] when that package is on disk.
    webrtc_share = shares.get("ohho_webrtc", "")
    if webrtc_share:
        webrtc_path = os.path.join(webrtc_share, "launch", WEBRTC_LAUNCH_NAME)
    if webrtc_launch_exists is None:
        webrtc_launch_exists = _existing_file(webrtc_path)
    want_webrtc = str(video_backend).strip().lower() == "webrtc"
    use_webrtc = truthy(video) and want_webrtc and bool(webrtc_launch_exists)
    use_web_video = truthy(video) and not use_webrtc

    astra_share = shares.get("astra_camera", "")
    astra_path = ""
    if astra_share:
        astra_path = os.path.join(astra_share, "launch", ASTRA_LAUNCH_NAME)
    if astra_launch_exists is None:
        astra_launch_exists = _existing_file(astra_path) if astra_path else False

    base_path = _contract_mod.teleop_path(contract, "cmd_vel_mux")
    arm_path = _contract_mod.teleop_path(contract, "arm_cmd_mux")
    actions = []

    xacro = ""
    if description:
        xacro = os.path.join(description, "urdf", "omnibot.urdf.xacro")
    actions.append({"kind": "robot_state", "xacro": xacro})

    publish_tf = not truthy(ekf)
    actions.append(
        _node(
            "omnibot_driver",
            "yahboom_controller_node.py",
            "yahboom_controller_node",
            parameters=[
                {
                    "serial_port": "/dev/ttyUSB0",
                    "baud_rate": 115200,
                    "wheel_radius": 0.04,
                    "wheel_separation_width": 0.215,
                    "wheel_separation_length": 0.165,
                    "publish_tf": publish_tf,
                    "publish_diagnostics": True,
                    "use_sim_time": False,
                }
            ],
            remappings=[("cmd_vel", base_path["driver_output"])],
        )
    )

    for name in _camera_names(truthy(bev)):
        actions.append(_camera_node(name, devices.get(name, "")))

    if truthy(bev):
        actions.append(
            _node(
                "omnibot_lerobot",
                "bev_stitcher_node.py",
                "bev_stitcher_node",
                parameters=[{"use_sim_time": False}],
            )
        )

    if truthy(ekf):
        actions.append(
            _include(
                "ekf",
                os.path.join(bringup, "launch", "state_estimation.launch.py"),
                arguments=[("use_sim_time", "false")],
            )
        )

    if truthy(depth_camera):
        if astra_launch_exists and astra_path:
            actions.append(
                _include(
                    "astra",
                    astra_path,
                    loader="any",
                    arguments=[
                        ("camera_name", "camera"),
                        ("depth_registration", "true"),
                        ("enable_point_cloud", "true"),
                        ("enable_colored_point_cloud", "false"),
                        ("color_depth_synchronization", "false"),
                        ("oni_log_level", "none"),
                    ],
                )
            )
        else:
            actions.append(
                _log(
                    "depth_camera:=true but astra_pro.launch.xml is not "
                    "installed. The VR profile is still starting."
                )
            )

    if truthy(arm):
        params = [{"publish_diagnostics": True, "use_sim_time": False}]
        arm_params = ""
        if arm_share:
            arm_params = os.path.join(arm_share, "config", "arm_params.yaml")
        if _existing_file(arm_params):
            params = [arm_params, params[0]]
        actions.append(
            _node(
                "omnibot_arm",
                "arm_driver_node.py",
                "arm_driver_node",
                parameters=params,
                remappings=[("joint_commands", arm_path["driver_output"])],
            )
        )

    actions.append(
        _include(
            "teleop_safety",
            os.path.join(hybrid, "launch", TELEOP_SAFETY_LAUNCH_NAME),
        )
    )
    actions.append(
        _node(
            "omnibot_hybrid",
            "cmd_vel_mux",
            "cmd_vel_mux",
            parameters=[{"default_mode": "teleop", "use_sim_time": False}],
        )
    )
    actions.append(
        _node(
            "omnibot_rl",
            "arm_cmd_mux",
            "arm_cmd_mux",
            parameters=[{"default_mode": "teleop", "use_sim_time": False}],
        )
    )

    if truthy(telemetry):
        telemetry_path = os.path.join(hybrid, "launch", TELEMETRY_LAUNCH_NAME)
        if _existing_file(telemetry_path):
            actions.append(
                _include(
                    "telemetry",
                    telemetry_path,
                    arguments=[
                        ("sim", "false"),
                        ("robot_id", str(robot_id).strip()),
                        ("publish_hz", f"{contract['telemetry_hz']:.1f}"),
                    ],
                )
            )
        else:
            actions.append(
                _log(
                    "telemetry:=true but telemetry.launch.py is not installed. "
                    "The VR profile is still starting."
                )
            )

    if truthy(descriptor):
        # Autodetect requires the YAML on disk. An explicit installed flag
        # lets a unit test describe the node without copying OHH-214's tree.
        explicit = descriptor_installed is not None
        if descriptor_on and (_existing_file(descriptor_yaml) or explicit):
            yaml_path = descriptor_yaml
            actions.append(
                _node(
                    "omnibot_bringup",
                    DESCRIPTOR_SCRIPT_NAME,
                    "ohho_descriptor",
                    parameters=[
                        {
                            "descriptor_yaml": yaml_path,
                            "kind": "real",
                            "use_sim_time": False,
                        }
                    ],
                )
            )
        else:
            actions.append(
                _log(
                    "descriptor publisher is not installed yet "
                    "(OHH-214 descriptor_node.py and robot_descriptor.yaml). "
                    "The VR profile is still starting."
                )
            )

    urls = ", ".join(_stream_urls(contract["ports"]["web_video"], bind))
    actions.append(_log(f"VR topic contract video streams: {urls}"))

    if clamped:
        actions.append(
            _log(
                "Requested rosbridge bind is not loopback. "
                f"Upstreams stay on {bind}. This profile does not publish "
                "rosbridge on every interface without the gateway."
            )
        )

    if use_gateway:
        actions.append(
            _log(
                "gateway:=true includes ohho_gateway.launch.py. "
                f"rosbridge and web_video_server stay on {bind}."
            )
        )
        actions.append(
            _include(
                "gateway",
                gateway_path,
                arguments=[
                    ("gateway", "true"),
                    ("start_rosbridge", "true" if truthy(rosbridge) else "false"),
                    (
                        "start_web_video",
                        "true" if use_web_video else "false",
                    ),
                    ("start_foxglove", "true" if truthy(foxglove) else "false"),
                ],
            )
        )
    else:
        if gateway_on:
            actions.append(
                _log(
                    "gateway:=true but ohho_gateway.launch.py is not installed. "
                    f"rosbridge stays on {bind}."
                )
            )
        if truthy(rosbridge):
            actions.append(
                _node(
                    "rosbridge_server",
                    "rosbridge_websocket",
                    "rosbridge_websocket",
                    parameters=[
                        {
                            "port": contract["ports"]["rosbridge"],
                            "address": bind,
                        }
                    ],
                )
            )
        if use_web_video:
            actions.append(
                _node(
                    "web_video_server",
                    "web_video_server",
                    "web_video_server",
                    parameters=[
                        {
                            "port": contract["ports"]["web_video"],
                            "address": bind,
                        }
                    ],
                )
            )
        if truthy(foxglove):
            actions.append(
                _node(
                    "foxglove_bridge",
                    "foxglove_bridge",
                    "foxglove_bridge",
                    parameters=[
                        {
                            "port": contract["ports"]["foxglove"],
                            "address": bind,
                        }
                    ],
                )
            )

    if truthy(video) and want_webrtc:
        if use_webrtc:
            # webrtc_streamer.launch.py declares no launch arguments.
            # The streamer binds loopback itself.
            actions.append(_include("webrtc", webrtc_path))
        else:
            actions.append(
                _log(
                    "video_backend:=webrtc but webrtc_streamer.launch.py is "
                    "not installed. web_video_server is serving the contract "
                    "streams instead."
                )
            )

    _validate(actions)
    return {
        "actions": actions,
        "bind": bind,
        "clamped_public_bind": clamped,
        "use_gateway": use_gateway,
        "contract_path": contract["path"],
    }
