"""VR action assembly and a synthetic 30 s LeRobot v2 episode.

The node tests stub rclpy. The dataset test writes parquet from plain arrays
and runs the OHH-48 layout check plus the OHH-99 quality filter.
"""

from __future__ import annotations

from contextlib import contextmanager
import json
import os
import subprocess
import sys
import types

import numpy as np
import pyarrow.parquet as pq
import pytest

_TEST_DIR = os.path.abspath(os.path.dirname(__file__))
_PKG_DIR = os.path.abspath(os.path.join(_TEST_DIR, ".."))
_REPO_ROOT = os.path.abspath(os.path.join(_PKG_DIR, "..", ".."))
_ENGINES = os.path.join(_REPO_ROOT, "omnibot-ai-engines")
if _TEST_DIR not in sys.path:
    sys.path.insert(0, _TEST_DIR)
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from omnibot_lerobot.lerobot_v2 import append_episode
from omnibot_lerobot.vr_action import (
    DEFAULT_ARM_COMMAND_TOPIC,
    DEFAULT_ARM_MUX_TOPIC,
    DEFAULT_BASE_COMMAND_TOPIC,
    DEFAULT_BASE_MUX_TOPIC,
    JOINT_NAMES,
    assemble_vr_sample,
    episode_metadata,
    normalize_action_source,
    positions_for_joints,
    twist_vector,
)
from test_record_session import _call, _install_ros_stubs, _restore_ros_stubs

MOTION_TOPICS = {
    DEFAULT_ARM_COMMAND_TOPIC,
    DEFAULT_ARM_MUX_TOPIC,
    DEFAULT_BASE_COMMAND_TOPIC,
    DEFAULT_BASE_MUX_TOPIC,
}
ARM = np.array([0.10, -0.20, 0.30, -0.15, 0.40, 0.20], dtype=np.float32)
MUX_ARM = np.ones(6, dtype=np.float32)
FOLLOWER = np.array([0.11, -0.19, 0.28, -0.14, 0.41, 0.18], dtype=np.float32)
BASE = np.array([0.05, -0.02, 0.10], dtype=np.float32)
MUX_BASE = np.array([0.9, 0.9, 0.9], dtype=np.float32)
ODOM = np.array([0.04, -0.01, 0.08], dtype=np.float32)


def _install_twist_stub(saved):
    saved["geometry_msgs"] = sys.modules.get("geometry_msgs")
    saved["geometry_msgs.msg"] = sys.modules.get("geometry_msgs.msg")
    package = types.ModuleType("geometry_msgs")
    messages = types.ModuleType("geometry_msgs.msg")

    class Twist:
        pass

    messages.Twist = Twist
    package.msg = messages
    sys.modules["geometry_msgs"] = package
    sys.modules["geometry_msgs.msg"] = messages


@contextmanager
def _vr_node(tmp_path, **extra):
    saved, _string_cls, _int32_cls = _install_ros_stubs()
    _install_twist_stub(saved)
    node_cls = sys.modules["rclpy.node"].Node
    node_cls.overrides = {
        "output_dir": str(tmp_path),
        "action_source": "vr",
        "robot_kind": "sim",
        "sim_world": "warehouse",
        "task_name": "pick the cube",
        **extra,
    }
    sys.modules.pop("omnibot_lerobot.teleop_recorder_node", None)
    package = sys.modules.get("omnibot_lerobot")
    if package is not None and hasattr(package, "teleop_recorder_node"):
        delattr(package, "teleop_recorder_node")
    from omnibot_lerobot import teleop_recorder_node as recorder_mod

    node = recorder_mod.TeleopRecorderNode()
    try:
        yield node, recorder_mod
    finally:
        node_cls.overrides = {}
        _restore_ros_stubs(saved)


def _joints(values, names=None):
    return types.SimpleNamespace(
        name=list(JOINT_NAMES if names is None else names),
        position=[float(item) for item in values],
    )


def _twist(vx, vy, wz):
    return types.SimpleNamespace(
        linear=types.SimpleNamespace(x=vx, y=vy, z=0.0),
        angular=types.SimpleNamespace(x=0.0, y=0.0, z=wz),
    )


def _odom(vx, vy, wz):
    return types.SimpleNamespace(twist=types.SimpleNamespace(twist=_twist(vx, vy, wz)))


def _feed_robot(node):
    node.follower_state_cb(_joints(FOLLOWER))
    node.odom_cb(_odom(float(ODOM[0]), float(ODOM[1]), float(ODOM[2])))


def _feed_teleop(node, mode="teleop"):
    node._control_mode_cb(types.SimpleNamespace(data=mode))
    node._vr_arm_cb(_joints(ARM))
    node._vr_arm_mux_cb(_joints(MUX_ARM))
    node._vr_base_cb(_twist(float(BASE[0]), float(BASE[1]), float(BASE[2])))
    node._vr_base_mux_cb(_twist(float(MUX_BASE[0]), float(MUX_BASE[1]), float(MUX_BASE[2])))


class TestVrAssembly:
    def test_action_source_accepts_only_leader_and_vr(self):
        assert normalize_action_source(" VR ") == "vr"
        assert normalize_action_source("leader") == "leader"
        with pytest.raises(ValueError):
            normalize_action_source("policy")

    def test_joint_names_accept_the_short_ohh197_order(self):
        short = [name[4:] if name.startswith("arm_") else name for name in JOINT_NAMES]
        mapped = positions_for_joints(short, ARM)
        assert mapped is not None
        assert np.allclose(mapped, ARM)
        ordered = positions_for_joints([], ARM)
        assert ordered is not None
        assert np.allclose(ordered, ARM)

    def test_dedicated_topic_beats_the_mux_and_state_is_the_follower(self):
        sample = assemble_vr_sample(
            ARM, MUX_ARM, BASE, MUX_BASE, "vla", FOLLOWER, ODOM, prefer="teleop"
        )
        assert sample.arm_source == "teleop"
        assert sample.base_source == "teleop"
        assert np.allclose(sample.action[:6], ARM)
        assert np.allclose(sample.action[6:], BASE)
        assert np.allclose(sample.state[:6], FOLLOWER)
        assert np.allclose(sample.state[6:], ODOM)

    def test_mux_is_ignored_outside_teleop_when_the_dedicated_topic_is_quiet(self):
        sample = assemble_vr_sample(
            None, MUX_ARM, None, MUX_BASE, "vla", FOLLOWER, ODOM
        )
        assert sample.arm_source == "none"
        assert sample.base_source == "none"
        assert np.allclose(sample.action, np.zeros(9))

    def test_mux_fallback_is_only_used_in_teleop_mode(self):
        sample = assemble_vr_sample(
            None, MUX_ARM, None, MUX_BASE, "teleop", FOLLOWER, ODOM
        )
        assert sample.arm_source == "mux"
        assert sample.base_source == "mux"
        assert np.allclose(sample.action[:6], MUX_ARM)
        assert np.allclose(sample.action[6:], MUX_BASE)

    def test_prefer_mux_uses_the_forwarded_command_in_teleop(self):
        chosen = assemble_vr_sample(
            ARM, MUX_ARM, BASE, MUX_BASE, "teleop", FOLLOWER, ODOM, prefer="mux"
        )
        assert chosen.arm_source == "mux"
        assert np.allclose(chosen.action[:6], MUX_ARM)
        held = assemble_vr_sample(
            ARM, MUX_ARM, BASE, MUX_BASE, "nav2", FOLLOWER, ODOM, prefer="mux"
        )
        assert held.arm_source == "teleop"
        assert np.allclose(held.action[:6], ARM)

    def test_bad_twist_components_are_not_a_command(self):
        assert twist_vector(0.1, "fast", 0.0) is None

    def test_metadata_defaults_operator_to_vr(self):
        meta = episode_metadata("  pick up the cup  ", "", "SIM", " warehouse ")
        assert meta == {
            "operator": "vr",
            "robot_kind": "sim",
            "sim_world": "warehouse",
            "task": "pick up the cup",
        }
        real = episode_metadata("carry", "vr", "real", "")
        assert real["robot_kind"] == "real"
        assert real["sim_world"] == ""


class TestVrRecorderNode:
    def test_vr_mode_subscribes_and_does_not_publish_motion(self, tmp_path):
        with _vr_node(tmp_path) as (node, _mod):
            assert node.action_source == "vr"
            assert node.operator == "vr"
            assert MOTION_TOPICS <= set(node._subscriptions)
            assert "/joy" in node._subscriptions
            assert set(node._publishers) == {"/ohho/record/state"}
            assert MOTION_TOPICS.isdisjoint(node._publishers)

    def test_services_record_teleop_topics_without_a_joystick(self, tmp_path):
        with _vr_node(tmp_path) as (node, _mod):
            node.joy_axes = [1.0, 1.0, 0.0, 1.0]
            _feed_robot(node)
            _feed_teleop(node, mode="vla")
            started = _call(node, "/ohho/record/start")
            assert started.success is True
            node._record_frame()
            frame = node.session.pending_frames()[-1]
            assert frame["arm_source"] == "teleop"
            assert frame["base_source"] == "teleop"
            assert np.allclose(frame["action"][:6], ARM)
            assert np.allclose(frame["action"][6:], BASE)
            assert np.allclose(frame["state"][:6], FOLLOWER)
            assert np.allclose(frame["state"][6:], ODOM)
            stopped = _call(node, "/ohho/record/stop")
            assert stopped.success is True
            info = json.loads((tmp_path / "meta" / "info.json").read_text(encoding="utf-8"))
            assert info["codebase_version"] == "v2.0"
            assert info["total_episodes"] == 1
            episode = json.loads(
                (tmp_path / "meta" / "episodes.jsonl").read_text(encoding="utf-8").strip()
            )
            assert episode["operator"] == "vr"
            assert episode["robot_kind"] == "sim"
            assert episode["sim_world"] == "warehouse"
            assert episode["task"] == "pick the cube"
            table = pq.read_table(tmp_path / "data" / "chunk-000" / "episode_000000.parquet")
            assert np.allclose(table["action"][0].as_py(), frame["action"])
            assert list(tmp_path.glob("episode_*")) == []

    def test_silent_teleop_topic_uses_the_mux_only_while_teleop(self, tmp_path):
        with _vr_node(tmp_path) as (node, _mod):
            _feed_robot(node)
            node._control_mode_cb(types.SimpleNamespace(data="teleop"))
            node._vr_arm_mux_cb(_joints(MUX_ARM))
            node._vr_base_mux_cb(
                _twist(float(MUX_BASE[0]), float(MUX_BASE[1]), float(MUX_BASE[2]))
            )
            assert _call(node, "/ohho/record/start").success is True
            node._record_frame()
            forwarded = node.session.pending_frames()[-1]
            assert forwarded["arm_source"] == "mux"
            assert forwarded["base_source"] == "mux"
            assert np.allclose(forwarded["action"][:6], MUX_ARM)
            node._control_mode_cb(types.SimpleNamespace(data="rl_nav"))
            node._record_frame()
            held = node.session.pending_frames()[-1]
            assert held["arm_source"] == "none"
            assert held["base_source"] == "none"
            assert np.allclose(held["action"], np.zeros(9))
            assert np.allclose(held["state"][:6], FOLLOWER)

    def test_discard_writes_no_dataset(self, tmp_path):
        with _vr_node(tmp_path) as (node, _mod):
            _feed_teleop(node)
            assert _call(node, "/ohho/record/start").success is True
            node._record_frame()
            assert _call(node, "/ohho/record/discard").success is True
            assert not (tmp_path / "meta").exists()
            assert list(tmp_path.rglob("*.parquet")) == []

    def test_unknown_action_source_stays_on_the_leader_path(self, tmp_path):
        with _vr_node(tmp_path, action_source="headset") as (node, _mod):
            assert node.action_source == "leader"
            assert DEFAULT_ARM_COMMAND_TOPIC not in node._subscriptions
            assert any("action_source" in message for message in node.get_logger().warnings)
            assert _call(node, "/ohho/record/start").success is True
            node._record_frame()
            frame = node.session.pending_frames()[-1]
            assert "arm_source" not in frame
            assert "base_source" not in frame
            assert _call(node, "/ohho/record/stop").success is True
            # Leader writer: ohho-os LeRobot v2 when that stack imports, otherwise numpy.
            # The VR writer is the one that stores operator / robot_kind / sim_world.
            if node._ohho_rec is not None:
                episode = json.loads(
                    (tmp_path / "meta" / "episodes.jsonl").read_text(encoding="utf-8").strip()
                )
                assert episode["length"] == 1
                assert "operator" not in episode
                assert "robot_kind" not in episode
                assert "sim_world" not in episode
                assert list(tmp_path.glob("episode_*")) == []
            else:
                assert (tmp_path / "episode_00000" / "data.npz").is_file()
                assert not (tmp_path / "meta" / "info.json").exists()


def _synthetic_frames(duration_s=30.0, fps=30):
    count = int(duration_s * fps) + 1
    frames = []
    for index in range(count):
        stamp = index / fps
        arm = np.array(
            [0.002 * index, 0.001 * index, -0.001 * index, 0.05, 0.0, 0.2],
            dtype=np.float32,
        )
        follower = arm - np.array([0.001, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        base = np.array([0.05, 0.01 * np.sin(stamp), 0.02], dtype=np.float32)
        sample = assemble_vr_sample(
            arm,
            np.ones(6, dtype=np.float32),
            base,
            np.ones(3, dtype=np.float32),
            "teleop",
            follower,
            base * 0.9,
            prefer="teleop",
        )
        image = np.zeros((8, 8, 3), dtype=np.uint8)
        image[:, :, 0] = (index * 17) % 256
        image[0, 0, 1] = 255 if index % 2 == 0 else 0
        frames.append(
            {
                "state": sample.state,
                "action": sample.action,
                "wrist_image": image,
                "bev_image": image.copy(),
                "timestamp": stamp,
                "wrist_timestamp": stamp,
                "bev_timestamp": stamp + 0.005,
                "arm_source": sample.arm_source,
            }
        )
    return frames


class TestSyntheticVrDataset:
    def test_thirty_second_episode_passes_validation_and_the_quality_filter(self, tmp_path):
        frames = _synthetic_frames()
        assert frames[-1]["timestamp"] - frames[0]["timestamp"] == pytest.approx(30.0)
        assert frames[0]["arm_source"] == "teleop"
        assert not np.allclose(frames[0]["action"][:6], np.ones(6))
        meta = episode_metadata("pick up the cup", "vr", "sim", "warehouse")
        append_episode(
            tmp_path,
            frames,
            meta,
            fps=30,
            episode_index=0,
            repo_id="local/vr_synthetic",
        )
        env = os.environ.copy()
        env["PYTHONPATH"] = _ENGINES + os.pathsep + env.get("PYTHONPATH", "")
        validate = subprocess.run(
            [
                sys.executable,
                "-m",
                "data_engine.scripts.validate_dataset",
                "--dataset",
                str(tmp_path),
                "--verbose",
            ],
            cwd=_ENGINES,
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        assert validate.returncode == 0, validate.stdout + validate.stderr
        filtered = subprocess.run(
            [
                sys.executable,
                "-m",
                "data_engine.scripts.filter_episodes",
                "--dataset",
                str(tmp_path),
                "--output",
                str(tmp_path / "meta" / "filter_results.json"),
            ],
            cwd=_ENGINES,
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        assert filtered.returncode == 0, filtered.stdout + filtered.stderr
        report = json.loads(
            (tmp_path / "meta" / "filter_results.json").read_text(encoding="utf-8")
        )
        assert report["kept"] == [0]
        assert report["rejected"] == []
        assert "REJECT" not in filtered.stdout
        episode = json.loads(
            (tmp_path / "meta" / "episodes.jsonl").read_text(encoding="utf-8").strip()
        )
        assert episode["length"] == len(frames)
        assert episode["operator"] == "vr"
        assert episode["robot_kind"] == "sim"
        assert episode["sim_world"] == "warehouse"
        assert episode["task"] == "pick up the cup"
