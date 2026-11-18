"""Recording session and teleop-recorder service tests.

The session tests are pure Python. The node tests stub rclpy the same way
as test_policy_guard.py so they run in the CPU job without a ROS graph.
"""

from __future__ import annotations

import json
import os
import sys
import time
import types
from unittest.mock import MagicMock

import numpy as np
import pytest

_PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from omnibot_lerobot.record_session import (
    RecordPhase,
    RecordSession,
    commit_stop,
)


def _session(clock_value: float = 100.0) -> RecordSession:
    return RecordSession(clock=lambda: clock_value)


class TestRecordSession:
    def test_start_records_task_index_and_timestamp(self):
        session = _session(100.0)
        result = session.start(task="pick up the cup", episode_index=3)
        assert result.success is True
        assert result.episode_index == 3
        assert result.message == "recording (episode_index=3)"
        snap = session.snapshot()
        assert snap == {
            "state": "recording",
            "task": "pick up the cup",
            "episode_index": 3,
            "frame_count": 0,
            "started_at": 100.0,
        }

    def test_omitted_episode_index_keeps_the_counter(self):
        session = RecordSession(episode_index=2)
        result = session.start(task="carry")
        assert result.success is True
        assert result.episode_index == 2
        assert session.episode_index == 2

    def test_start_while_recording_does_not_open_a_second_episode(self):
        session = _session(10.0)
        session.start(task="first")
        session.append_frame({"n": 1})
        result = session.start(task="second", episode_index=9)
        assert result.success is False
        assert "cannot start while recording" in result.message
        assert result.episode_index == 0
        assert session.pending_frames() == [{"n": 1}]
        assert session.task == "first"
        assert session.started_at == 10.0
        assert session.frame_count == 1
        assert session.episode_index == 0

    def test_start_while_saving_is_rejected(self):
        session = _session()
        session.start(task="cup")
        session.append_frame({"n": 1})
        _result, frames = session.request_stop()
        assert frames is not None
        result = session.start(task="other")
        assert result.success is False
        assert "cannot start while saving" in result.message
        assert session.episode_index == 0
        assert frames == [{"n": 1}]

    def test_negative_episode_index_is_rejected(self):
        session = _session()
        result = session.start(task="cup", episode_index=-1)
        assert result.success is False
        assert session.state is RecordPhase.IDLE
        assert session.episode_index == 0
        assert "episode_index must be an integer >= 0" in result.message

    def test_stop_while_idle_is_an_error_and_writes_nothing(self, tmp_path):
        session = _session()
        written = []

        def write(frames):
            written.append(frames)
            (tmp_path / "should-not-exist").write_bytes(b"x")

        result = commit_stop(session, write)
        assert result.success is False
        assert "cannot stop while idle" in result.message
        assert written == []
        assert list(tmp_path.iterdir()) == []
        assert session.state is RecordPhase.IDLE

    def test_empty_stop_writes_nothing_and_stays_on_the_same_index(self, tmp_path):
        session = _session()
        session.start(task="cup")
        written = []
        result = commit_stop(session, lambda frames: written.append(frames))
        assert result.success is False
        assert result.message == "no frames to save (episode_index=0)"
        assert written == []
        assert session.state is RecordPhase.IDLE
        assert session.episode_index == 0
        assert list(tmp_path.iterdir()) == []

    def test_stop_writes_the_full_episode_once_and_advances_the_index(self, tmp_path):
        session = _session(50.0)
        session.start(task="cup", episode_index=4)
        session.append_frame({"n": 1})
        session.append_frame({"n": 2})
        path = tmp_path / "episode.npz"
        seen = []

        def write(frames):
            seen.append(list(frames))
            path.write_bytes(b"full-episode")

        result = commit_stop(session, write)
        assert result.success is True
        assert result.episode_index == 4
        assert result.message == "saved (episode_index=4)"
        assert seen == [[{"n": 1}, {"n": 2}]]
        assert path.read_bytes() == b"full-episode"
        assert session.episode_index == 5
        assert session.pending_frames() == []
        assert session.snapshot()["state"] == "idle"
        assert session.snapshot()["frame_count"] == 0
        assert session.snapshot()["started_at"] == 0.0

    def test_failed_save_does_not_advance_the_index(self):
        session = _session()
        session.start(task="cup")
        session.append_frame({"n": 1})

        def write(_frames):
            raise RuntimeError("disk full")

        result = commit_stop(session, write)
        assert result.success is False
        assert "failed to save: disk full" in result.message
        assert result.episode_index == 0
        assert session.episode_index == 0
        assert session.state is RecordPhase.IDLE
        assert session.pending_frames() == []

    def test_discard_drops_frames_and_writes_nothing(self, tmp_path):
        session = _session()
        session.start(task="cup", episode_index=2)
        session.append_frame({"n": 1})
        session.append_frame({"n": 2})
        written = []
        result = session.discard()
        assert result.success is True
        assert result.message == "discarded; nothing written (episode_index=2)"
        assert result.episode_index == 2
        assert session.episode_index == 2
        assert session.pending_frames() == []
        assert session.state is RecordPhase.IDLE
        assert session.frame_count == 0
        follow_up = commit_stop(session, lambda frames: written.append(frames))
        assert follow_up.success is False
        assert written == []
        assert list(tmp_path.iterdir()) == []

    def test_discard_while_idle_is_an_error(self):
        session = _session()
        result = session.discard()
        assert result.success is False
        assert "cannot discard while idle" in result.message
        assert session.episode_index == 0

    def test_discard_while_saving_does_not_drop_the_detached_episode(self):
        session = _session()
        session.start(task="cup")
        session.append_frame({"n": 1})
        session.append_frame({"n": 2})
        written = []

        def write(frames):
            discarded = session.discard()
            assert discarded.success is False
            assert "cannot discard while saving" in discarded.message
            written.append(list(frames))

        result = commit_stop(session, write)
        assert result.success is True
        assert written == [[{"n": 1}, {"n": 2}]]
        assert session.episode_index == 1
        assert session.pending_frames() == []

    def test_frames_are_ignored_unless_recording(self):
        session = _session()
        assert session.append_frame({"n": 1}) is False
        session.start(task="cup")
        assert session.append_frame({"n": 1}) is True
        assert session.frame_count == 1

    def test_note_frame_count_updates_only_while_recording(self):
        session = _session()
        session.note_frame_count(5)
        assert session.frame_count == 0
        session.start(task="cup")
        session.note_frame_count(5)
        assert session.frame_count == 5
        session.note_frame_count(-1)
        assert session.frame_count == 5

    def test_blank_task_keeps_the_previous_task(self):
        session = RecordSession(task="mobile manipulation task")
        session.start(task="   ")
        assert session.task == "mobile manipulation task"


def _install_ros_stubs():
    saved = {}

    def _block(name):
        saved[name] = sys.modules.get(name)
        sys.modules[name] = None

    def _module(name):
        saved[name] = sys.modules.get(name)
        module = types.ModuleType(name)
        sys.modules[name] = module
        return module

    class _Param:
        def __init__(self, value):
            self.value = value

    class _Logger:
        def __init__(self):
            self.warnings = []
            self.infos = []
            self.errors = []

        def warn(self, message, **_kwargs):
            self.warnings.append(message)

        def info(self, message, **_kwargs):
            self.infos.append(message)

        def error(self, message, **_kwargs):
            self.errors.append(message)

    class _Node:
        overrides = {}

        def __init__(self, name):
            self._name = name
            self._params = {}
            self._services = {}
            self._subscriptions = {}
            self._timers = []
            self._publishers = {}
            self._logger = _Logger()

        def declare_parameter(self, name, default=None):
            if name in type(self).overrides:
                self._params[name] = type(self).overrides[name]
            else:
                self._params[name] = default

        def get_parameter(self, name):
            return _Param(self._params[name])

        def get_logger(self):
            return self._logger

        def create_publisher(self, msg_type, topic, qos):
            pub = MagicMock()
            self._publishers[topic] = {"pub": pub, "qos": qos, "type": msg_type}
            return pub

        def create_subscription(self, _msg_type, topic, callback, _qos):
            self._subscriptions[topic] = callback
            return MagicMock()

        def create_service(self, srv_type, name, callback):
            self._services[name] = {"type": srv_type, "callback": callback}
            return MagicMock()

        def create_timer(self, period, callback):
            self._timers.append((period, callback))
            return MagicMock()

        def destroy_node(self):
            return None

    class _QoS:
        def __init__(self, history=None, depth=None, reliability=None, durability=None):
            self.history = history
            self.depth = depth
            self.reliability = reliability
            self.durability = durability

    class DurabilityPolicy:
        TRANSIENT_LOCAL = "transient_local"

    class ReliabilityPolicy:
        RELIABLE = "reliable"

    class HistoryPolicy:
        KEEP_LAST = "keep_last"

    class StringMsg:
        def __init__(self):
            self.data = ""

    class Int32Msg:
        def __init__(self, data=0):
            self.data = data

    class JointState:
        pass

    class Joy:
        pass

    class Image:
        pass

    class Odometry:
        pass

    class Trigger:
        pass

    _block("cv_bridge")
    _block("lerobot")

    m_rclpy = _module("rclpy")
    m_rclpy.node = types.ModuleType("rclpy.node")
    m_rclpy.node.Node = _Node
    sys.modules["rclpy.node"] = m_rclpy.node
    saved["rclpy.node"] = saved.get("rclpy.node")
    m_qos = _module("rclpy.qos")
    m_qos.QoSProfile = _QoS
    m_qos.DurabilityPolicy = DurabilityPolicy
    m_qos.ReliabilityPolicy = ReliabilityPolicy
    m_qos.HistoryPolicy = HistoryPolicy
    m_rclpy.qos = m_qos

    m_sensor = _module("sensor_msgs")
    m_sensor_msg = _module("sensor_msgs.msg")
    m_sensor_msg.Image = Image
    m_sensor_msg.JointState = JointState
    m_sensor_msg.Joy = Joy
    m_sensor.msg = m_sensor_msg

    m_nav = _module("nav_msgs")
    m_nav_msg = _module("nav_msgs.msg")
    m_nav_msg.Odometry = Odometry
    m_nav.msg = m_nav_msg

    m_std = _module("std_msgs")
    m_std_msg = _module("std_msgs.msg")
    m_std_msg.String = StringMsg
    m_std_msg.Int32 = Int32Msg
    m_std.msg = m_std_msg

    m_srvs = _module("std_srvs")
    m_srvs_srv = _module("std_srvs.srv")
    m_srvs_srv.Trigger = Trigger
    m_srvs.srv = m_srvs_srv

    return saved, StringMsg, Int32Msg


def _restore_ros_stubs(saved):
    for name, module in saved.items():
        if module is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = module
    sys.modules.pop("omnibot_lerobot.teleop_recorder_node", None)
    pkg = sys.modules.get("omnibot_lerobot")
    if pkg is not None and hasattr(pkg, "teleop_recorder_node"):
        delattr(pkg, "teleop_recorder_node")


def _states(node):
    pub = node._publishers["/ohho/record/state"]["pub"]
    return [json.loads(call.args[0].data) for call in pub.publish.call_args_list]


def _call(node, service):
    response = types.SimpleNamespace(success=None, message=None)
    node._services[service]["callback"](object(), response)
    return response


def _press(node, index):
    buttons = [0] * 16
    buttons[index] = 1
    node.joy_cb(types.SimpleNamespace(axes=list(node.joy_axes), buttons=buttons))


def _release(node):
    node.joy_cb(types.SimpleNamespace(axes=list(node.joy_axes), buttons=[0] * 16))


def _uses_ohho(node) -> bool:
    return node._ohho_rec is not None


def _assert_nothing_written(tmp_path):
    assert list(tmp_path.glob("episode_*")) == []
    assert list(tmp_path.rglob("*.npz")) == []
    assert list(tmp_path.rglob("*.parquet")) == []
    assert list(tmp_path.rglob("*.mp4")) == []


def _assert_numpy_episode(tmp_path, episode_index, *, frames=None, task=None):
    path = tmp_path / f"episode_{episode_index:05d}" / "data.npz"
    assert path.is_file()
    saved = np.load(path, allow_pickle=True)
    if frames is not None:
        assert saved["actions"].shape[0] == frames
    if task is not None:
        assert str(saved["task"][0]) == task
    return saved


def _assert_v2_episode(tmp_path, dataset_index, *, frames, task=None, action_tail=None):
    import pyarrow.parquet as pq

    path = tmp_path / "data" / "chunk-000" / f"episode_{dataset_index:06d}.parquet"
    assert path.is_file()
    table = pq.read_table(path)
    assert table.num_rows == frames
    if action_tail is not None:
        action = table.column("action")[0].as_py()
        assert action[6] == pytest.approx(action_tail[0])
        assert action[7] == pytest.approx(action_tail[1])
        assert action[8] == pytest.approx(action_tail[2])
    if task is not None:
        tasks = (tmp_path / "meta" / "tasks.jsonl").read_text(encoding="utf-8")
        assert task in tasks


class TestTeleopRecorderServices:
    @pytest.fixture
    def recorder(self, tmp_path):
        saved, string_cls, int32_cls = _install_ros_stubs()
        node_cls = sys.modules["rclpy.node"].Node
        node_cls.overrides = {"output_dir": str(tmp_path)}
        sys.modules.pop("omnibot_lerobot.teleop_recorder_node", None)
        from omnibot_lerobot import teleop_recorder_node as recorder_mod

        node = recorder_mod.TeleopRecorderNode()
        node._string_cls = string_cls
        node._int32_cls = int32_cls
        yield node, recorder_mod
        node_cls.overrides = {}
        _restore_ros_stubs(saved)

    def test_services_use_trigger_and_state_is_latched(self, recorder):
        node, recorder_mod = recorder
        assert recorder_mod.MAX_LINEAR == 0.2
        assert recorder_mod.MAX_ANGULAR == 1.0
        for name in ("/ohho/record/start", "/ohho/record/stop", "/ohho/record/discard"):
            assert name in node._services
        qos = node._publishers["/ohho/record/state"]["qos"]
        assert qos.durability == "transient_local"
        assert qos.depth == 1
        periods = [period for period, _callback in node._timers]
        assert any(abs(period - 0.5) < 1e-9 for period in periods)
        assert _states(node)[-1]["state"] == "idle"

    def test_start_while_recording_does_not_save_a_second_episode(self, recorder, tmp_path):
        node, _mod = recorder
        saves = []
        real_save = node._save_episode

        def wrapped():
            saves.append("save")
            return real_save()

        node._save_episode = wrapped
        started = _call(node, "/ohho/record/start")
        assert started.success is True
        assert started.message == "recording (episode_index=0)"
        node._record_frame()
        again = _call(node, "/ohho/record/start")
        assert again.success is False
        assert "cannot start while recording" in again.message
        assert node.session.frame_count == 1
        assert node.session.episode_index == 0
        assert saves == []
        stopped = _call(node, "/ohho/record/stop")
        assert stopped.success is True
        assert stopped.message == "saved (episode_index=0)"
        assert saves == ["save"]
        if _uses_ohho(node):
            _assert_v2_episode(tmp_path, 0, frames=1)
            assert not (tmp_path / "data" / "chunk-000" / "episode_000001.parquet").exists()
        else:
            episodes = list(tmp_path.glob("episode_*"))
            assert [path.name for path in episodes] == ["episode_00000"]
            assert (episodes[0] / "data.npz").is_file()
        assert node.session.episode_index == 1
        assert _states(node)[-1]["state"] == "idle"
        assert _states(node)[-1]["frame_count"] == 0

    def test_discard_writes_no_episode(self, recorder, tmp_path):
        node, _mod = recorder
        node._save_episode = lambda: (_ for _ in ()).throw(AssertionError("discard saved"))
        task = node._string_cls()
        task.data = "pick up the cup"
        node._task_cb(task)
        index = node._int32_cls(4)
        node._episode_index_cb(index)
        started = _call(node, "/ohho/record/start")
        assert started.success is True
        assert started.message == "recording (episode_index=4)"
        node._record_frame()
        node._record_frame()
        discarded = _call(node, "/ohho/record/discard")
        assert discarded.success is True
        assert discarded.message == "discarded; nothing written (episode_index=4)"
        _assert_nothing_written(tmp_path)
        assert node.session.episode_index == 4
        assert node.session.pending_frames() == []
        snap = _states(node)[-1]
        assert snap["state"] == "idle"
        assert snap["task"] == "pick up the cup"
        assert snap["frame_count"] == 0
        assert node._pending_episode_index is None

    def test_stop_and_discard_while_idle_are_errors(self, recorder, tmp_path):
        node, _mod = recorder
        stopped = _call(node, "/ohho/record/stop")
        discarded = _call(node, "/ohho/record/discard")
        assert stopped.success is False
        assert "cannot stop while idle" in stopped.message
        assert discarded.success is False
        assert "cannot discard while idle" in discarded.message
        _assert_nothing_written(tmp_path)

    def test_topic_episode_index_is_one_shot_and_parameter_pins(self, recorder, tmp_path):
        node, _mod = recorder
        node._episode_index_cb(node._int32_cls(4))
        assert _call(node, "/ohho/record/start").success is True
        node._record_frame()
        assert _call(node, "/ohho/record/stop").success is True
        assert node.session.episode_index == 5
        if _uses_ohho(node):
            _assert_v2_episode(tmp_path, 0, frames=1)
        else:
            _assert_numpy_episode(tmp_path, 4, frames=1)
        node._params["episode_index"] = 3
        assert _call(node, "/ohho/record/start").message == "recording (episode_index=3)"
        node._record_frame()
        assert _call(node, "/ohho/record/stop").success is True
        if _uses_ohho(node):
            _assert_v2_episode(tmp_path, 1, frames=1)
        else:
            _assert_numpy_episode(tmp_path, 3, frames=1)

    def test_numpy_fallback_names_files_from_the_session_index(self, recorder, tmp_path):
        node, _mod = recorder
        node._ohho_rec = None
        node._episode_index_cb(node._int32_cls(4))
        assert _call(node, "/ohho/record/start").success is True
        node._record_frame()
        assert _call(node, "/ohho/record/stop").success is True
        assert node.session.episode_index == 5
        _assert_numpy_episode(tmp_path, 4, frames=1, task="mobile manipulation task")
        node._params["episode_index"] = 3
        assert _call(node, "/ohho/record/start").message == "recording (episode_index=3)"
        node._record_frame()
        assert _call(node, "/ohho/record/stop").success is True
        _assert_numpy_episode(tmp_path, 3, frames=1)

    def test_state_timer_reports_frame_count(self, recorder):
        node, _mod = recorder
        _call(node, "/ohho/record/start")
        node._record_frame()
        node._record_frame()
        state_timer = next(
            callback for period, callback in node._timers if abs(period - 0.5) < 1e-9
        )
        state_timer()
        snap = _states(node)[-1]
        assert snap["state"] == "recording"
        assert snap["frame_count"] == 2
        assert snap["started_at"] > 0

    def test_joystick_still_starts_stops_and_discards(self, recorder, tmp_path):
        node, recorder_mod = recorder
        _press(node, 7)
        assert node.session.state is RecordPhase.RECORDING
        node._record_frame()
        _release(node)
        _press(node, 6)
        assert node.session.state is RecordPhase.IDLE
        _assert_nothing_written(tmp_path)
        assert node.session.episode_index == 0
        _release(node)
        _press(node, 7)
        node.joy_axes = [1.0, 1.0, 0.0, 1.0]
        node._record_frame()
        action = node.session.pending_frames()[-1]["action"]
        assert action[6] == pytest.approx(recorder_mod.MAX_LINEAR)
        assert action[7] == pytest.approx(recorder_mod.MAX_LINEAR)
        assert action[8] == pytest.approx(recorder_mod.MAX_ANGULAR)
        _release(node)
        _press(node, 7)
        if _uses_ohho(node):
            _assert_v2_episode(
                tmp_path,
                0,
                frames=1,
                task="mobile manipulation task",
                action_tail=(
                    recorder_mod.MAX_LINEAR,
                    recorder_mod.MAX_LINEAR,
                    recorder_mod.MAX_ANGULAR,
                ),
            )
        else:
            _assert_numpy_episode(
                tmp_path,
                0,
                frames=1,
                task="mobile manipulation task",
            )
            saved = np.load(tmp_path / "episode_00000" / "data.npz", allow_pickle=True)
            assert saved["actions"][0][6] == pytest.approx(recorder_mod.MAX_LINEAR)
            assert saved["actions"][0][7] == pytest.approx(recorder_mod.MAX_LINEAR)
            assert saved["actions"][0][8] == pytest.approx(recorder_mod.MAX_ANGULAR)

    def test_timeout_saves_the_open_episode(self, recorder, tmp_path):
        node, _mod = recorder
        _call(node, "/ohho/record/start")
        node._record_frame()
        node.session.started_at = time.time() - node.episode_timeout_s - 1
        node.record_timer_cb()
        if _uses_ohho(node):
            _assert_v2_episode(tmp_path, 0, frames=1)
        else:
            _assert_numpy_episode(tmp_path, 0, frames=1)
        assert node.session.state is RecordPhase.IDLE
        assert node.session.episode_index == 1

    def test_empty_stop_creates_no_file(self, recorder, tmp_path):
        node, _mod = recorder
        _call(node, "/ohho/record/start")
        stopped = _call(node, "/ohho/record/stop")
        assert stopped.success is False
        assert "no frames to save" in stopped.message
        _assert_nothing_written(tmp_path)
        assert node.session.episode_index == 0

    def test_failed_write_does_not_advance_the_index(self, recorder, tmp_path):
        node, _mod = recorder
        _call(node, "/ohho/record/start")
        node._record_frame()

        def boom():
            raise RuntimeError("disk full")

        node._save_episode = boom
        stopped = _call(node, "/ohho/record/stop")
        assert stopped.success is False
        assert "disk full" in stopped.message
        assert node.session.episode_index == 0
        _assert_nothing_written(tmp_path)
        assert _states(node)[-1]["state"] == "idle"
