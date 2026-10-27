# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""CPU checks for the real-robot VR profile. No ROS 2 install required."""

import importlib.util
import subprocess
import sys
import tempfile
from pathlib import Path

BRINGUP = Path(__file__).resolve().parents[1]
LAUNCH = BRINGUP / "launch"
ROS = BRINGUP.parent
REPO = BRINGUP.parents[1]
SUPPORT_PATH = LAUNCH / "robot_vr_support.py"
LAUNCH_PATH = LAUNCH / "robot_vr.launch.py"
CONTRACT_PATH = BRINGUP / "config" / "vr_topic_contract.yaml"
DOCS = REPO / "docs" / "vr.md"
UNIT = BRINGUP / "config" / "systemd" / "omnibot-robot-vr.service"
SHARES = {
    "omnibot_bringup": str(BRINGUP),
    "omnibot_hybrid": str(ROS / "omnibot_hybrid"),
    "omnibot_arm": str(ROS / "omnibot_arm"),
    "omnibot_description": str(ROS / "omnibot_description"),
}
DEVICES = {
    "front": "/dev/video2",
    "rear": "/dev/video10",
    "left": "/dev/video6",
    "right": "/dev/video4",
    "wrist": "/dev/video0",
}


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _support():
    return _load(SUPPORT_PATH, "omnibot_robot_vr_support_test")


def _plan(support, **kwargs):
    return support.plan(SHARES, camera_devices=DEVICES, **kwargs)


def _by_name(actions, name):
    found = [item for item in actions if item.get("kind") == "node" and item["name"] == name]
    assert len(found) == 1, name
    return found[0]


def _includes(actions, label):
    return [
        item
        for item in actions
        if item.get("kind") == "include" and item["label"] == label
    ]


def _index(actions, kind, key):
    for index, item in enumerate(actions):
        if item.get("kind") != kind:
            continue
        if kind == "node" and item["name"] == key:
            return index
        if kind == "include" and item["label"] == key:
            return index
    raise AssertionError(f"missing {kind} {key}")


def test_contract_locks_the_sim_topics_and_the_loopback_bind():
    contract_mod = _load(LAUNCH / "vr_contract.py", "omnibot_vr_contract_test")
    contract = contract_mod.load_vr_contract(CONTRACT_PATH)
    assert contract["ports"]["bind"] == "127.0.0.1"
    assert contract["ports"]["rosbridge"] == 9090
    assert contract["ports"]["web_video"] == 8080
    assert contract["safety"]["default_control_mode"] == "teleop"
    assert contract["safety"]["use_sim_time"] is False
    assert contract["safety"]["heartbeat_timeout_ms"] == 300
    assert contract_mod.video_topics(contract) == (
        "/camera/front/image_raw",
        "/camera/wrist/image_raw",
        "/camera/base/bev/image_raw",
    )
    docs = DOCS.read_text(encoding="utf-8")
    for name in contract["topics"]:
        assert name in docs
    assert "vr_topic_contract.yaml" in docs
    assert "robot_vr.launch.py" in docs
    sim = _load(LAUNCH / "sim_vr_support.py", "omnibot_sim_vr_support_test")
    assert sim.video_stream_topics() == contract_mod.video_topics(contract)
    assert "video_stream_topics" in (LAUNCH / "sim_vr.launch.py").read_text(encoding="utf-8")


def test_default_plan_puts_safety_before_the_muxes_and_binds_loopback():
    support = _support()
    result = _plan(support)
    actions = result["actions"]
    assert result["bind"] == "127.0.0.1"
    assert result["clamped_public_bind"] is False
    assert result["use_gateway"] is False

    safety_at = _index(actions, "include", "teleop_safety")
    assert safety_at < _index(actions, "node", "cmd_vel_mux")
    assert safety_at < _index(actions, "node", "arm_cmd_mux")
    safety = _includes(actions, "teleop_safety")
    assert len(safety) == 1
    assert safety[0]["arguments"] == []
    assert safety[0]["path"].endswith("teleop_safety.launch.py")

    base = _by_name(actions, "yahboom_controller_node")
    assert base["remappings"] == [("cmd_vel", "/cmd_vel/out")]
    assert base["parameters"][0]["use_sim_time"] is False
    arm = _by_name(actions, "arm_driver_node")
    assert arm["remappings"] == [("joint_commands", "/arm/joint_commands/out")]

    for mux_name in ("cmd_vel_mux", "arm_cmd_mux"):
        mux = _by_name(actions, mux_name)
        assert mux["remappings"] == []
        assert mux["parameters"][0]["default_mode"] == "teleop"
        assert mux["parameters"][0]["use_sim_time"] is False

    rosbridge = _by_name(actions, "rosbridge_websocket")
    assert rosbridge["parameters"][0]["address"] == "127.0.0.1"
    assert rosbridge["parameters"][0]["port"] == 9090
    video = _by_name(actions, "web_video_server")
    assert video["parameters"][0]["address"] == "127.0.0.1"
    assert video["parameters"][0]["port"] == 8080
    fox = _by_name(actions, "foxglove_bridge")
    assert fox["parameters"][0]["address"] == "127.0.0.1"

    cameras = [
        item["name"]
        for item in actions
        if item.get("kind") == "node" and item["name"].startswith("usb_cam_")
    ]
    assert cameras == [
        "usb_cam_front",
        "usb_cam_rear",
        "usb_cam_left",
        "usb_cam_right",
        "usb_cam_wrist",
    ]
    assert _by_name(actions, "bev_stitcher_node")
    telemetry = _includes(actions, "telemetry")
    assert len(telemetry) == 1
    assert ("sim", "false") in telemetry[0]["arguments"]
    assert ("publish_hz", "20.0") in telemetry[0]["arguments"]
    assert _includes(actions, "gateway") == []
    assert _includes(actions, "astra") == []
    descriptor = _by_name(actions, "ohho_descriptor")
    assert descriptor["executable"] == "descriptor_node.py"
    assert descriptor["parameters"][0]["kind"] == "real"
    logs = " ".join(item["msg"] for item in actions if item.get("kind") == "log")
    assert "/camera/front/image_raw" in logs
    assert "/camera/wrist/image_raw" in logs
    assert "/camera/base/bev/image_raw" in logs
    assert "robot_descriptor.yaml" in descriptor["parameters"][0]["descriptor_yaml"]
    executables = [
        item["executable"] for item in actions if item.get("kind") == "node"
    ]
    for banned in ("joy_node", "teleop_twist_joy", "teleop_twist_keyboard"):
        assert banned not in executables


def test_public_bind_is_clamped_even_when_the_gateway_file_exists():
    support = _support()
    assert support.loopback_bind("0.0.0.0") == ("127.0.0.1", True)
    assert support.loopback_bind("192.168.1.5") == ("127.0.0.1", True)
    assert support.loopback_bind("127.0.0.1") == ("127.0.0.1", False)

    refused = _plan(support, requested_bind="0.0.0.0", gateway="false")
    assert refused["bind"] == "127.0.0.1"
    assert refused["clamped_public_bind"] is True
    assert refused["use_gateway"] is False
    assert _includes(refused["actions"], "gateway") == []
    assert _by_name(refused["actions"], "rosbridge_websocket")["parameters"][0][
        "address"
    ] == "127.0.0.1"

    owned = _plan(
        support,
        requested_bind="0.0.0.0",
        gateway="true",
        gateway_launch_exists=True,
    )
    assert owned["use_gateway"] is True
    gateway = _includes(owned["actions"], "gateway")
    assert len(gateway) == 1
    args = dict(gateway[0]["arguments"])
    assert args["gateway"] == "true"
    assert args["start_rosbridge"] == "true"
    assert args["start_web_video"] == "true"
    assert not any(
        item.get("kind") == "node" and item["name"] == "rosbridge_websocket"
        for item in owned["actions"]
    )

    present_but_off = _plan(
        support,
        gateway="false",
        gateway_launch_exists=True,
        requested_bind="0.0.0.0",
    )
    assert present_but_off["use_gateway"] is False
    assert _includes(present_but_off["actions"], "gateway") == []
    assert _by_name(present_but_off["actions"], "rosbridge_websocket")["parameters"][0][
        "address"
    ] == "127.0.0.1"


def test_optional_descriptor_webrtc_and_camera_flags():
    support = _support()
    described = _plan(support, descriptor_installed=True)
    node = _by_name(described["actions"], "ohho_descriptor")
    assert node["executable"] == "descriptor_node.py"
    assert node["parameters"][0]["kind"] == "real"
    assert node["parameters"][0]["use_sim_time"] is False

    fallback = _plan(support, video_backend="webrtc", webrtc_launch_exists=False)
    assert _by_name(fallback["actions"], "web_video_server")
    assert _includes(fallback["actions"], "webrtc") == []
    logs = " ".join(item["msg"] for item in fallback["actions"] if item["kind"] == "log")
    assert "webrtc_streamer.launch.py" in logs

    webrtc = _plan(
        support,
        video_backend="webrtc",
        webrtc_launch_exists=True,
        gateway_launch_exists=True,
    )
    assert _includes(webrtc["actions"], "webrtc")
    assert not any(
        item.get("kind") == "node" and item["name"] == "web_video_server"
        for item in webrtc["actions"]
    )
    assert dict(_includes(webrtc["actions"], "gateway")[0]["arguments"])[
        "start_web_video"
    ] == "false"

    cameras = _plan(support, bev="false")
    names = [
        item["name"]
        for item in cameras["actions"]
        if item.get("kind") == "node" and item["name"].startswith("usb_cam_")
    ]
    assert names == ["usb_cam_front", "usb_cam_wrist"]
    assert not any(
        item.get("kind") == "node" and item["name"] == "bev_stitcher_node"
        for item in cameras["actions"]
    )

    no_arm = _plan(support, arm="false")
    assert not any(
        item.get("kind") == "node" and item["name"] == "arm_driver_node"
        for item in no_arm["actions"]
    )
    assert _index(no_arm["actions"], "include", "teleop_safety") < _index(
        no_arm["actions"], "node", "arm_cmd_mux"
    )


def test_docs_cpu_table_is_empty_and_the_unit_is_not_enabled_here():
    text = DOCS.read_text(encoding="utf-8")
    assert "top" in text
    assert "htop" in text
    assert "omnibot-robot-vr.service" in text
    assert "does not enable it" in text
    rows = []
    started = False
    for line in text.splitlines():
        if line.startswith("| Stream |"):
            started = True
            continue
        if started and line.startswith("|---"):
            continue
        if started and line.startswith("|"):
            cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
            rows.append(cells)
            continue
        if started:
            break
    assert [row[0] for row in rows] == [
        "front",
        "wrist",
        "BEV",
        "rosbridge_websocket",
        "web_video_server",
        "telemetry_aggregator",
        "teleop_safety",
        "whole profile",
    ]
    for row in rows:
        assert row[2] == ""
        assert row[3] == ""

    unit = UNIT.read_text(encoding="utf-8")
    assert "ros2 launch omnibot_bringup robot_vr.launch.py" in unit
    assert "systemctl enable" not in unit
    assert "OHHO_GATEWAY_TOKEN=" not in unit
    assert "ExecStart=" in unit
    assert "WantedBy=multi-user.target" in unit
    launch_text = LAUNCH_PATH.read_text(encoding="utf-8")
    assert "0.0.0.0" not in launch_text
    assert "systemctl" not in launch_text
    assert _support().DEFAULT_GATEWAY == "true"
    assert _support().DEFAULT_DEPTH == "false"


def test_launch_description_builds():
    completed = subprocess.run(
        [sys.executable, str(Path(__file__).resolve()), "--fake-launch"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr + completed.stdout
    assert "FAKE_LAUNCH_OK" in completed.stdout


def _install_fakes():
    import types

    def add(name, **attrs):
        module = types.ModuleType(name)
        module.__path__ = []
        module.__package__ = name
        for key, value in attrs.items():
            setattr(module, key, value)
        sys.modules[name] = module
        return module

    class LaunchDescription:
        def __init__(self, entities=None):
            self.entities = list(entities or [])

    class DeclareLaunchArgument:
        def __init__(self, name, default_value=None, description=""):
            self.name = name
            self.default_value = default_value
            self.description = description

    class LogInfo:
        def __init__(self, msg=""):
            self.msg = msg

    class OpaqueFunction:
        def __init__(self, function):
            self.function = function

    class PythonLaunchDescriptionSource:
        def __init__(self, path):
            self.path = path

    class AnyLaunchDescriptionSource:
        def __init__(self, path):
            self.path = path

    class IncludeLaunchDescription:
        def __init__(self, source, launch_arguments=None):
            self.source = source
            self.launch_arguments = list(launch_arguments or [])

    class Node:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class LaunchConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context.values[self.name]

    class Command:
        def __init__(self, command):
            self.command = command

    class ParameterValue:
        def __init__(self, value, value_type=None):
            self.value = value
            self.value_type = value_type

    class FindPackageShare:
        def __init__(self, package):
            self.package = package

        def perform(self, context):
            if self.package not in context.shares:
                raise KeyError(self.package)
            return context.shares[self.package]

    launch = add("launch", LaunchDescription=LaunchDescription)
    actions = add(
        "launch.actions",
        DeclareLaunchArgument=DeclareLaunchArgument,
        IncludeLaunchDescription=IncludeLaunchDescription,
        LogInfo=LogInfo,
        OpaqueFunction=OpaqueFunction,
    )
    sources = add(
        "launch.launch_description_sources",
        PythonLaunchDescriptionSource=PythonLaunchDescriptionSource,
        AnyLaunchDescriptionSource=AnyLaunchDescriptionSource,
    )
    subs = add(
        "launch.substitutions",
        Command=Command,
        LaunchConfiguration=LaunchConfiguration,
    )
    launch.actions = actions
    launch.launch_description_sources = sources
    launch.substitutions = subs
    ros = add("launch_ros")
    ros_actions = add("launch_ros.actions", Node=Node)
    ros_desc = add("launch_ros.descriptions", ParameterValue=ParameterValue)
    ros_subs = add("launch_ros.substitutions", FindPackageShare=FindPackageShare)
    ros.actions = ros_actions
    ros.descriptions = ros_desc
    ros.substitutions = ros_subs


def _fake_main():
    _install_fakes()
    module = _load(LAUNCH_PATH, "omnibot_robot_vr_launch_fake")
    description = module.generate_launch_description()
    declares = [
        item
        for item in description.entities
        if item.__class__.__name__ == "DeclareLaunchArgument"
    ]
    defaults = {item.name: item.default_value for item in declares}
    assert defaults["gateway"] == "true"
    assert defaults["rosbridge_address"] == "127.0.0.1"
    assert defaults["telemetry"] == "true"
    assert defaults["descriptor"] == "true"
    assert defaults["depth_camera"] == "false"
    assert "0.0.0.0" not in set(defaults.values())
    assert "teleop_safety" not in defaults

    class Context:
        def __init__(self, values, shares):
            self.values = values
            self.shares = shares

    opaque = next(
        item
        for item in description.entities
        if item.__class__.__name__ == "OpaqueFunction"
    )
    shares = dict(SHARES)
    actions = opaque.function(Context(defaults, shares))
    names = [getattr(item, "name", "") for item in actions]
    assert "robot_state_publisher" in names
    assert "yahboom_controller_node" in names
    assert "cmd_vel_mux" in names
    assert "arm_cmd_mux" in names
    assert "rosbridge_websocket" in names
    assert names.count("cmd_vel_mux") == 1
    assert names.count("rosbridge_websocket") == 1

    def _path(item):
        source = getattr(item, "source", None)
        return getattr(source, "path", "")

    order = []
    for item in actions:
        if getattr(item, "name", ""):
            order.append(item.name)
        if "teleop_safety.launch.py" in _path(item):
            order.append("teleop_safety")
    assert order.index("teleop_safety") < order.index("cmd_vel_mux")
    assert order.index("teleop_safety") < order.index("arm_cmd_mux")

    yahboom = next(item for item in actions if getattr(item, "name", "") == "yahboom_controller_node")
    assert ("cmd_vel", "/cmd_vel/out") in [tuple(pair) for pair in yahboom.remappings]
    arm = next(item for item in actions if getattr(item, "name", "") == "arm_driver_node")
    assert ("joint_commands", "/arm/joint_commands/out") in [
        tuple(pair) for pair in arm.remappings
    ]
    rosbridge = next(item for item in actions if getattr(item, "name", "") == "rosbridge_websocket")
    assert rosbridge.parameters[0]["address"] == "127.0.0.1"
    for item in actions:
        for params in getattr(item, "parameters", []) or []:
            if isinstance(params, dict):
                assert params.get("address") != "0.0.0.0"

    values = dict(defaults)
    values["rosbridge_address"] = "0.0.0.0"
    values["gateway"] = "false"
    clamped = opaque.function(Context(values, shares))
    clamped_bridge = next(
        item for item in clamped if getattr(item, "name", "") == "rosbridge_websocket"
    )
    assert clamped_bridge.parameters[0]["address"] == "127.0.0.1"

    with tempfile.TemporaryDirectory() as tmp:
        launch_dir = Path(tmp) / "launch"
        launch_dir.mkdir()
        (launch_dir / "ohho_gateway.launch.py").write_text("# fixture\n", encoding="utf-8")
        (launch_dir / "state_estimation.launch.py").write_text(
            "# fixture\n", encoding="utf-8"
        )
        owned_shares = dict(shares)
        owned_shares["omnibot_bringup"] = tmp
        owned_values = dict(defaults)
        owned_values["rosbridge_address"] = "0.0.0.0"
        owned = opaque.function(Context(owned_values, owned_shares))
    assert not any(getattr(item, "name", "") == "rosbridge_websocket" for item in owned)
    gateway = next(item for item in owned if _path(item).endswith("ohho_gateway.launch.py"))
    assert ("gateway", "true") in [tuple(pair) for pair in gateway.launch_arguments]
    assert ("start_rosbridge", "true") in [tuple(pair) for pair in gateway.launch_arguments]
    print("FAKE_LAUNCH_OK")


if __name__ == "__main__":
    if "--fake-launch" in sys.argv:
        _fake_main()
