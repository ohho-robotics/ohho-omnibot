# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""CPU checks for the scripted headset. Gazebo is the vr-contract job."""

import asyncio
import importlib.util
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import virtual_headset as headset


ROOT = Path(__file__).resolve().parents[2]


class World:
    """Tiny stand-in for teleop_safety, the muxes, and the Gazebo bridge."""

    def __init__(self, gz_listens_to="/cmd_vel/out", arm_follows_mux=True, clear_latch_on_false=False):
        self.gz_listens_to = gz_listens_to
        self.arm_follows_mux = arm_follows_mux
        self.clear_latch_on_false = clear_latch_on_false
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.cmd_out = (0.0, 0.0)
        self.joints = {
            "arm_shoulder_pan": 0.0,
            "arm_shoulder_lift": 0.0,
            "arm_elbow_flex": 0.0,
            "arm_wrist_flex": 0.0,
            "arm_wrist_roll": 0.0,
            "arm_gripper": 0.0,
        }
        self.latched = False
        self.estop = False
        self.last_hb = None
        self.mode = "teleop"
        self.now = 0.0
        self.events = []
        self.service_calls = []

    def armed(self):
        if self.last_hb is None or self.latched:
            return False
        return (self.now - self.last_hb) < 0.3

    def safety(self):
        reason = ""
        if self.latched:
            reason = "estop"
        elif self.last_hb is not None and (self.now - self.last_hb) >= 0.3:
            reason = "heartbeat_watchdog"
        return {"armed": self.armed(), "latched": self.latched, "reason": reason}

    def on_publish(self, topic, msg):
        self.events.append((topic, msg, self.now))
        if topic == headset.HEARTBEAT:
            self.last_hb = self.now
            return
        if topic == headset.MODE:
            self.mode = str(msg.get("data") or "")
            return
        if topic == headset.ESTOP:
            self.estop = bool(msg.get("data"))
            if self.estop:
                self.latched = True
                self.vx = self.vy = 0.0
                self.cmd_out = (0.0, 0.0)
            elif self.clear_latch_on_false:
                self.latched = False
            return
        if topic == headset.TELEOP_RAW:
            linear = msg.get("linear") or {}
            vx = float(linear.get("x") or 0.0)
            vy = float(linear.get("y") or 0.0)
            if not (self.armed() and self.mode == "teleop"):
                self.cmd_out = (0.0, 0.0)
                self.vx = self.vy = 0.0
                return
            self.cmd_out = (vx, vy)
            if self.gz_listens_to == "/cmd_vel/out":
                self.vx, self.vy = vx, vy
            else:
                self.vx = self.vy = 0.0
            return
        if topic == headset.JOINT_RAW:
            if not (self.armed() and self.mode == "teleop" and self.arm_follows_mux):
                return
            names = msg.get("name") or []
            positions = msg.get("position") or []
            for name, position in zip(names, positions):
                if name not in self.joints:
                    continue
                target = float(position)
                previous = self.joints[name]
                delta = max(-0.15, min(0.15, target - previous))
                self.joints[name] = previous + delta

    def step(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.now += dt
        if not self.armed() or self.mode != "teleop":
            self.vx = 0.0
            self.vy = 0.0
            self.cmd_out = (0.0, 0.0)


class FakeLink:
    def __init__(self, world):
        self.world = world
        self.subscriptions = []

    def clock(self):
        return self.world.now

    async def sleep(self, seconds):
        self.world.step(seconds)

    def latest(self, topic):
        world = self.world
        if topic == headset.ODOM:
            return {
                "pose": {"pose": {"position": {"x": world.x, "y": world.y, "z": 0.0}}},
                "twist": {
                    "twist": {"linear": {"x": world.vx, "y": world.vy, "z": 0.0}}
                },
            }
        if topic == headset.JOINT_STATES:
            names = list(world.joints)
            return {"name": names, "position": [world.joints[name] for name in names]}
        if topic == headset.SAFETY:
            return {"data": json.dumps(world.safety(), sort_keys=True)}
        if topic == headset.CMD_OUT:
            return {
                "linear": {"x": world.cmd_out[0], "y": world.cmd_out[1], "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        if topic == headset.MODE_ACTIVE:
            return {"data": world.mode}
        return None

    async def advertise(self, topic, type_name):
        del topic, type_name

    async def subscribe(self, topic, type_name):
        del type_name
        self.subscriptions.append(topic)

    async def publish(self, topic, msg):
        self.world.on_publish(topic, msg)

    async def call_service(self, service, type_name, args=None, timeout=5.0):
        del type_name, args, timeout
        self.world.service_calls.append(service)
        self.world.events.append((service, {}, self.world.now))
        if service != headset.RESET:
            return {"values": {"success": False, "message": "unknown"}, "result": False}
        if self.world.estop:
            return {
                "values": {"success": False, "message": "emergency_stop is still asserted"},
                "result": False,
            }
        self.world.latched = False
        return {"values": {"success": True, "message": "e-stop reset"}, "result": True}


def _topics(world):
    return [topic for topic, _msg, _stamp in world.events]


def test_happy_path_drives_through_the_headset_topics():
    world = World()
    link = FakeLink(world)
    evidence = asyncio.run(headset.execute(link))

    assert evidence["ok"] is True
    assert evidence["forward_m"] >= headset.FORWARD_MIN_M
    assert abs(evidence["strafe_m"]) >= headset.STRAFE_MIN_M
    for name in headset.JOINTS:
        assert evidence["joint_delta_rad"][name] >= headset.JOINT_MIN_DELTA
    assert evidence["estop"]["latched"] is True
    assert evidence["estop"]["reason"] == "estop"
    assert evidence["reset"]["latched"] is False
    assert evidence["reset"]["armed"] is True
    assert evidence["link_loss"]["reason"] == "heartbeat_watchdog"
    assert evidence["link_loss"]["armed"] is False

    topics = _topics(world)
    assert headset.TELEOP_RAW in topics
    assert headset.JOINT_RAW in topics
    assert headset.HEARTBEAT in topics
    assert "/cmd_vel" not in topics
    assert "/cmd_vel/out" not in topics
    assert "/cmd_vel/teleop" not in topics
    assert world.service_calls == [headset.RESET]
    assert {headset.ODOM, headset.JOINT_STATES, headset.SAFETY, headset.CMD_OUT} <= set(
        link.subscriptions
    )

    estop_values = [
        msg.get("data") for topic, msg, _stamp in world.events if topic == headset.ESTOP
    ]
    assert True in estop_values
    assert False in estop_values
    assert estop_values.index(True) < estop_values.index(False)
    service_at = topics.index(headset.RESET)
    assert headset.ESTOP in topics[:service_at]
    assert any(
        topic == headset.ESTOP and msg.get("data") is False
        for topic, msg, _stamp in world.events[:service_at]
    )
    last_heartbeat = max(
        index for index, topic in enumerate(topics) if topic == headset.HEARTBEAT
    )
    assert headset.TELEOP_RAW in topics[last_heartbeat:]


def test_broken_mux_remap_does_not_move_the_base():
    world = World(gz_listens_to="/cmd_vel")
    with pytest.raises(headset.ContractError, match="mux output"):
        asyncio.run(headset.execute(FakeLink(world)))


def test_arm_commands_must_come_through_the_mux():
    world = World(arm_follows_mux=False)
    with pytest.raises(headset.ContractError, match="joint_commands/out"):
        asyncio.run(headset.execute(FakeLink(world)))


def test_estop_false_must_not_clear_the_latch():
    world = World(clear_latch_on_false=True)
    with pytest.raises(headset.ContractError, match="cleared the latch"):
        asyncio.run(headset.execute(FakeLink(world)))


def test_message_helpers_match_the_headset_shapes():
    twist = headset.twist_msg(0.2, -0.1, 0.0)
    assert twist["linear"]["x"] == pytest.approx(0.2)
    assert twist["linear"]["y"] == pytest.approx(-0.1)
    joints = headset.joint_msg(headset.JOINTS, (0.15, 0.15, 0.15))
    assert joints["name"] == list(headset.JOINTS)
    pose = headset.pose_of(
        {"pose": {"pose": {"position": {"x": 1.0, "y": 0.3}}}}
    )
    assert pose == pytest.approx((1.0, 0.3))
    safety = headset.safety_of(
        {"data": json.dumps({"armed": True, "latched": False, "reason": ""})}
    )
    assert safety == {"armed": True, "latched": False, "reason": ""}
    assert headset.service_ok(
        {"values": {"success": True, "message": "e-stop reset"}}
    ) == (True, "e-stop reset")


def test_bridge_yaml_feeds_gazebo_from_the_mux_output():
    text = """
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  direction: ROS_TO_GZ
"""
    assert headset.ros_topics_to_gz_cmd_vel(text) == ["/cmd_vel"]

    bridge = (
        ROOT / "omnibot-ros2/omnibot_bringup/config/sim_bridge_vr.yaml"
    ).read_text(encoding="utf-8")
    assert headset.ros_topics_to_gz_cmd_vel(bridge) == ["/cmd_vel/out"]

    launch = (
        ROOT / "omnibot-ros2/omnibot_bringup/launch/sim.launch.py"
    ).read_text(encoding="utf-8")
    teleop = (
        ROOT / "omnibot-ros2/omnibot_bringup/launch/sim_teleop.launch.py"
    ).read_text(encoding="utf-8")
    # sim.launch.py no longer switches the bridge on vr:=true. sim_teleop
    # passes bridge_config:=sim_bridge_vr.yaml, and that file maps
    # /cmd_vel/out onto gz /cmd_vel.
    assert 'default_value="sim_bridge.yaml"' in launch
    assert "sim_bridge_vr.yaml" in launch
    assert '"bridge_config": "sim_bridge_vr.yaml"' in teleop


def test_runner_and_mirror_stay_on_the_mux_path():
    runner = (ROOT / "tests/vr_contract/run_vr_contract.sh").read_text(encoding="utf-8")
    assert "sim_teleop.launch.py" in runner
    assert "arm_joint_state_mirror.py" in runner
    # set -u makes Jazzy setup.bash fail on unset AMENT_TRACE_SETUP_FILES.
    nounset_off = runner.index("set +u")
    ros_setup = runner.index("source /opt/ros/jazzy/setup.bash")
    nounset_on = runner.index("set -u", nounset_off)
    assert nounset_off < ros_setup < nounset_on
    assert "world:=flat" in runner
    assert "arm:=true" in runner
    assert "video:=false" in runner
    assert "-s mcap" in runner
    assert "virtual_headset.py" in runner

    mirror = (
        ROOT / "omnibot-ros2/omnibot_arm/scripts/arm_joint_state_mirror.py"
    ).read_text(encoding="utf-8")
    assert '"/arm/joint_commands/out"' in mirror
    assert '"/arm/joint_states"' in mirror
    assert "teleop_raw" not in mirror
    for name in headset.JOINTS:
        assert name in mirror

    launch = (
        ROOT / "omnibot-ros2/omnibot_arm/launch/sim_arm.launch.py"
    ).read_text(encoding="utf-8")
    assert "arm_joint_state_mirror.py" in launch

    workflow = (ROOT / ".github/workflows/vr-contract.yml").read_text(encoding="utf-8")
    assert "pull_request:" in workflow
    assert "schedule:" in workflow
    assert "ros:jazzy-ros-base-noble" in workflow
    assert "upload-artifact" in workflow
    assert "vr-contract-out/" in workflow


def test_apply_joint_command_ignores_unknown_names():
    path = ROOT / "omnibot-ros2/omnibot_arm/scripts/arm_joint_state_mirror.py"
    spec = importlib.util.spec_from_file_location("arm_joint_state_mirror", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    measured = {name: 0.0 for name in module.JOINT_NAMES}
    updated = module.apply_joint_command(
        measured,
        ["arm_shoulder_pan", "not_a_joint", "arm_elbow_flex"],
        [0.15, 9.0, float("nan")],
    )
    assert updated["arm_shoulder_pan"] == pytest.approx(0.15)
    assert updated["arm_elbow_flex"] == pytest.approx(0.0)
    assert "not_a_joint" not in updated
