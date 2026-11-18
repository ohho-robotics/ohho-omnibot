# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Trial loop, summary table, Gazebo reset plan, and the real-robot gate."""

from __future__ import annotations

import base64
import json
import random
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Thread

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import eval_pick_place as harness  # noqa: E402


READY_IMAGES = {
    harness.WRIST_KEY: [[1, 0, 0]],
    harness.BEV_KEY: [[0, 1, 0]],
}


class CounterPolicy:
    def __init__(self, action=(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0)):
        self.resets = 0
        self.observations = []
        self.action = action

    def reset(self):
        self.resets += 1

    def select_action(self, obs):
        self.observations.append(dict(obs))
        return self.action


class MemoryTransport:
    """Records service calls and publishes. No ROS and no Gazebo."""

    def __init__(self, senses=None, estop=False, record=False):
        self.services = []
        self.sent = []
        self.modes = []
        self.bags = []
        self.stopped = 0
        self.senses = list(senses or [])
        self.estop = estop
        self.record = record

    def exec_service(self, call):
        self.services.append(call)

    def start_bag(self, bag_path):
        self.bags.append(bag_path)
        return self.record

    def stop_bag(self):
        self.stopped += 1

    def request_vla_mode(self):
        self.modes.append("vla")

    def estop_latched(self):
        return self.estop

    def sense(self):
        if self.senses:
            return self.senses.pop(0)
        return harness.SceneSense()

    def send_clamped(self, arm, base):
        self.sent.append((arm, base))


def test_default_trial_count_is_20():
    assert harness.DEFAULT_TRIALS == 20


def test_fake_loop_resets_every_trial_and_writes_summary(tmp_path):
    policy = CounterPolicy()
    backend = harness.FakeBackend(policy=policy)
    results = harness.run_eval(backend, trials=20, output_dir=tmp_path, seed_start=5)

    assert [row.trial for row in results] == list(range(1, 21))
    assert [row.seed for row in results] == list(range(5, 25))
    assert policy.resets == 20
    assert len(policy.observations) == 20
    assert {row.result for row in results} <= set(harness.RESULT_LABELS)
    counts = {label: 0 for label in harness.RESULT_LABELS}
    for row in results:
        counts[row.result] += 1
    assert counts == {label: 5 for label in harness.RESULT_LABELS}

    lines = (tmp_path / "trials.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(lines) == 20
    first = json.loads(lines[0])
    assert first["result"] == "success"
    assert first["duration_s"] == pytest.approx(1.0)
    assert first["bag_path"].endswith("trial_001")
    assert first["bag_recorded"] is False
    assert first["task_id"] == "OHH-98"
    assert first["scripted"] is True

    summary = (tmp_path / "summary.md").read_text(encoding="utf-8")
    assert "| Trial | Seed | Result | Duration (s) | Bag path | Recorded |" in summary
    assert "| 1 | 5 | success | 1.000 |" in summary
    assert "| 20 | 24 | intervention | 2.900 |" in summary
    assert "| success | 5 |" in summary
    assert "| collision | 5 |" in summary
    assert "| timeout | 5 |" in summary
    assert "| intervention | 5 |" in summary
    assert "not a measured policy success rate" in summary
    # Two headers, two separator rows, 20 trial rows, and four counts.
    assert summary.count("\n| ") == 28


def test_unknown_result_is_rejected():
    with pytest.raises(harness.EvalError):
        harness.FakeBackend(outcomes=("dropped",))


def test_zero_trials_are_rejected(tmp_path):
    with pytest.raises(ValueError):
        harness.run_eval(harness.FakeBackend(), trials=0, output_dir=tmp_path)


def test_real_backend_refuses_without_flag_or_confirmation():
    with pytest.raises(harness.RealEvalRefused, match="--real"):
        harness.build_backend("real", real_flag=False, confirmation=harness.REAL_CONFIRMATION)
    with pytest.raises(harness.RealEvalRefused, match="Varun"):
        harness.build_backend("real", real_flag=True, confirmation="please")


def test_authorized_real_backend_does_not_open_a_device(monkeypatch):
    def boom(*_args, **_kwargs):
        raise AssertionError("device access")

    monkeypatch.setattr(subprocess, "run", boom)
    monkeypatch.setattr(subprocess, "Popen", boom)
    monkeypatch.setattr(harness.urllib.request, "urlopen", boom)
    backend = harness.build_backend(
        "real",
        real_flag=True,
        confirmation=harness.REAL_CONFIRMATION,
    )
    with pytest.raises(harness.RealEvalRefused, match="does not open"):
        backend.reset_scene(1, 0)
    with pytest.raises(harness.RealEvalRefused, match="no hardware runner"):
        backend.run_trial(1, 0, "bag")


def test_sim_refuses_the_robot_domain(monkeypatch):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "30")
    with pytest.raises(harness.SimUnavailable, match="30"):
        harness.build_backend("sim")

    monkeypatch.setenv("ROS_DOMAIN_ID", "7")
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)
    with pytest.raises(harness.SimUnavailable, match="ROS_LOCALHOST_ONLY"):
        harness.build_backend("sim")

    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "233")
    with pytest.raises(harness.SimUnavailable, match="232"):
        harness.build_backend("sim")


def test_isolated_sim_refuses_to_invent_results_without_gazebo(monkeypatch):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "7")
    assert harness.sim_isolation_failure() is None
    with pytest.raises(harness.SimUnavailable, match="invent"):
        harness.build_backend("sim")


def test_seed_draw_order_and_fixed_table():
    rng = random.Random(0)
    cube_x = rng.uniform(*harness.CUBE_X)
    cube_y = rng.uniform(*harness.CUBE_Y)
    cube_yaw = rng.uniform(*harness.CUBE_YAW)
    bin_x = rng.uniform(*harness.BIN_X)
    bin_y = rng.uniform(*harness.BIN_Y)
    bin_yaw = rng.uniform(*harness.BIN_YAW)
    layout = harness.layout_for_seed(0)
    again = harness.layout_for_seed(0)
    assert (layout.cube.x, layout.cube.y, layout.cube.yaw) == (
        cube_x,
        cube_y,
        cube_yaw,
    )
    assert (layout.bin.x, layout.bin.y, layout.bin.yaw) == (bin_x, bin_y, bin_yaw)
    assert layout == again
    assert layout.cube.z == pytest.approx(
        harness.TABLE_TOP_Z + harness.CUBE_SIZE / 2.0 + harness.CUBE_CLEARANCE_Z
    )
    assert layout.table.name == "pick_place_table"
    assert harness.layout_for_seed(1).cube.x != layout.cube.x
    with pytest.raises(ValueError):
        harness.layout_for_seed(True)


def test_reset_plan_calls_gazebo_reset_then_model_poses():
    plan = harness.reset_plan("apartment", 4)
    assert [call.service for call in plan] == [
        "/world/apartment/control",
        "/world/apartment/set_pose",
        "/world/apartment/set_pose",
        "/world/apartment/set_pose",
    ]
    assert plan[0].request_type == "gz.msgs.WorldControl"
    assert plan[0].request == "reset: {all: true}"
    assert plan[0].argv[0] == "gz"
    assert "pick_place_cube" in plan[2].request
    assert "pick_place_bin" in plan[3].request
    with pytest.raises(ValueError):
        harness.reset_plan("apartment;rm", 0)


def test_cube_in_bin_and_fell():
    bin_pose = (0.4, 1.1, harness.TABLE_TOP_Z)
    inside = (0.4, 1.1, harness.TABLE_TOP_Z + harness.CUBE_SIZE / 2.0)
    assert harness.cube_in_bin(inside, bin_pose)
    outside = (0.4, 1.1 + 0.2, inside[2])
    assert not harness.cube_in_bin(outside, bin_pose)
    assert harness.cube_fell(0.1)
    assert not harness.cube_fell(harness.TABLE_TOP_Z + 0.02)


def test_sim_trial_resets_steps_policy_and_clamps(tmp_path):
    policy = CounterPolicy()
    transport = MemoryTransport(
        senses=[harness.SceneSense(images=READY_IMAGES)],
        record=True,
    )
    safety = harness.load_arm_safety()
    backend = harness.GazeboSimBackend(
        transport,
        world="apartment",
        policy=policy,
        max_steps=1,
        hz=10.0,
    )
    results = harness.run_eval(backend, trials=1, output_dir=tmp_path, seed_start=4)

    assert policy.resets == 1
    assert len(policy.observations) == 1
    assert policy.observations[0][harness.TASK_KEY] == harness.TASK_NAME
    assert transport.services[0].service == "/world/apartment/control"
    assert transport.services[0].request == "reset: {all: true}"
    assert any("pick_place_cube" in call.request for call in transport.services)
    assert transport.modes == ["vla"]
    assert transport.stopped == 1
    assert len(transport.sent) == 1
    arm, base = transport.sent[0]
    assert arm is not None
    for value, ref in zip(arm, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
        assert abs(value - ref) <= safety.MAX_JOINT_DELTA_RAD + 1e-9
    assert max(abs(value) for value in base[:2]) <= harness.MAX_BASE_LIN_VEL
    assert abs(base[2]) <= harness.MAX_BASE_ANG_VEL
    assert results[0].result == "timeout"
    assert results[0].bag_recorded is True
    assert results[0].scripted is False
    assert "cube pose was not observed" in results[0].detail


def test_estop_is_intervention_and_does_not_publish():
    policy = CounterPolicy()
    transport = MemoryTransport(estop=True)
    backend = harness.GazeboSimBackend(transport, policy=policy, max_steps=5)
    backend.reset_scene(1, 0)
    observation = backend.run_trial(1, 0, "bag")
    assert observation.result == "intervention"
    assert transport.sent == []
    assert transport.modes == []
    assert policy.observations == []


def test_success_collision_and_blank_frames():
    bin_pose = (0.4, 1.1, harness.TABLE_TOP_Z)
    cube = (0.4, 1.1, harness.TABLE_TOP_Z + harness.CUBE_SIZE / 2.0)
    transport = MemoryTransport(
        senses=[harness.SceneSense(cube=cube, bin_pose=bin_pose)]
    )
    backend = harness.GazeboSimBackend(transport, policy=CounterPolicy(), max_steps=5)
    assert backend.run_trial(1, 0, "bag").result == "success"
    assert transport.sent == []

    fell = MemoryTransport(senses=[harness.SceneSense(cube=(0.2, 1.0, 0.1))])
    fell_backend = harness.GazeboSimBackend(fell, max_steps=3)
    fell_obs = fell_backend.run_trial(1, 0, "bag")
    assert fell_obs.result == "collision"
    assert fell_obs.detail == "cube fell below the table"
    assert fell.sent == []

    blank = MemoryTransport(senses=[harness.SceneSense(images={harness.WRIST_KEY: [[0, 0, 0]], harness.BEV_KEY: [[0, 0, 0]]})])
    policy = CounterPolicy()
    blank_backend = harness.GazeboSimBackend(blank, policy=policy, max_steps=2)
    assert blank_backend.run_trial(1, 0, "bag").result == "timeout"
    assert policy.observations == []
    assert blank.sent == [(None, (0.0, 0.0, 0.0)), (None, (0.0, 0.0, 0.0))]


def test_clamp_reads_arm_safety_delta(monkeypatch):
    safety = harness.load_arm_safety()
    assert safety.MAX_JOINT_DELTA_RAD == 0.15
    monkeypatch.setattr(safety, "MAX_JOINT_DELTA_RAD", 0.05)
    arm, _base = harness.clamp_policy_action((1, 0, 0, 0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0))
    assert arm[0] == pytest.approx(0.05)


def test_pose_text_and_bag_command():
    text = """
    pose {
      name: "pick_place_cube"
      position { x: 0.10 y: 1.20 z: 0.752 }
    }
    pose {
      name: "pick_place_bin"
      position {
        x: 0.40
        y: 1.10
        z: 0.73
      }
    }
    """
    poses = harness.parse_pose_text(text)
    assert poses["pick_place_cube"] == pytest.approx((0.10, 1.20, 0.752))
    assert poses["pick_place_bin"][0] == pytest.approx(0.40)
    argv = harness.rosbag_record_argv("/tmp/trial_001")
    assert argv[:4] == ["ros2", "bag", "record", "-s"]
    assert "mcap" in argv
    assert "-a" not in argv
    assert "/arm/joint_commands" in argv
    assert "/cmd_vel/vla" in argv


def test_publishers_stay_on_mux_inputs():
    source = Path(harness.__file__).read_text(encoding="utf-8")
    start = source.split("def start(self)")[1].split("def close")[0]
    assert "ARM_COMMAND_TOPIC" in start
    assert "BASE_COMMAND_TOPIC" in start
    assert "joint_commands/out" not in start
    assert "ESTOP_TOPIC, self._on_estop" in start
    assert "create_publisher(Bool" not in start
    assert harness.ARM_COMMAND_TOPIC == "/arm/joint_commands"
    assert harness.BASE_COMMAND_TOPIC == "/cmd_vel/vla"


def test_main_fake_run_and_real_refusal(tmp_path, capsys):
    out = tmp_path / "out"
    assert harness.main(["--backend", "fake", "--trials", "4", "--output", str(out)]) == 0
    assert (out / "summary.md").is_file()
    captured = capsys.readouterr()
    assert "Backend: fake" in captured.out
    assert harness.main(["--backend", "real", "--output", str(tmp_path / "real")]) == 2
    assert not (tmp_path / "real").exists()
    err = capsys.readouterr().err
    assert "--real" in err
    code = harness.main(
        [
            "--backend",
            "real",
            "--real",
            "--confirm",
            harness.REAL_CONFIRMATION,
            "--output",
            str(tmp_path / "authorized"),
        ]
    )
    assert code == 2
    assert "does not open" in capsys.readouterr().err
    assert not (tmp_path / "authorized").exists()


def test_main_sim_refuses_on_this_machine(monkeypatch, capsys):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "7")
    assert harness.main(["--backend", "sim", "--trials", "1"]) == 2
    assert "invent" in capsys.readouterr().err


class _ServeHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        body = json.loads(self.rfile.read(length).decode("utf-8"))
        self.server.bodies.append(body)  # type: ignore[attr-defined]
        self.server.keys.append(self.headers.get("X-API-Key"))  # type: ignore[attr-defined]
        payload = json.dumps(
            {"action": {"vector": [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, "latency_ms": 1}
        ).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, _format, *_args):
        return


def test_serve_policy_posts_predict_and_reads_the_vector():
    server = ThreadingHTTPServer(("127.0.0.1", 0), _ServeHandler)
    server.bodies = []
    server.keys = []
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        port = server.server_address[1]
        policy = harness.HttpServePolicy(
            f"http://127.0.0.1:{port}",
            timeout_sec=2.0,
            api_key="unit-test-key",
        )
        action = policy.select_action(
            {
                harness.TASK_KEY: harness.TASK_NAME,
                harness.WRIST_KEY: [[9, 8, 7]],
                harness.BEV_KEY: [[1, 2, 3]],
            }
        )
    finally:
        server.shutdown()
        server.server_close()
    assert action[0] == pytest.approx(0.2)
    assert len(action) == 9
    body = server.bodies[0]
    assert body["instruction"] == harness.TASK_NAME
    assert "image_base64" in body
    assert "bev_image_b64" in body["config"]
    assert server.keys == ["unit-test-key"]


def test_encode_image_is_hwc_uint8():
    numpy = pytest.importorskip("numpy")
    chw = numpy.zeros((1, 3, 2, 2), dtype=numpy.float32)
    chw[0, 0, 0, 0] = 1.0
    raw = base64.b64decode(harness.encode_image_b64(chw))
    assert list(raw[:3]) == [255, 0, 0]
    assert len(raw) == 2 * 2 * 3
    assert list(base64.b64decode(harness.encode_image_b64([9, 8, 7]))) == [9, 8, 7]


def test_gz_transport_reset_and_pose_parse_without_ros():
    calls = []

    def runner(argv, timeout=10):
        del timeout
        calls.append(list(argv))
        if "topic" in argv:
            return (
                'name: "pick_place_cube"\n'
                "position { x: 0.10 y: 1.20 z: 0.752 }\n"
            )
        return ""

    transport = harness.GzRosTransport("apartment", runner=runner)
    for call in harness.reset_plan("apartment", 1):
        transport.exec_service(call)
    sense = transport.sense()
    assert calls[0][0] == "gz"
    assert "/world/apartment/control" in calls[0]
    assert calls[0][-1] == "reset: {all: true}"
    assert sense.cube == pytest.approx((0.10, 1.20, 0.752))
    assert sense.intervention is False


def test_policy_observation_is_nchw_when_numpy_is_present():
    numpy = pytest.importorskip("numpy")
    image = [
        [[255, 0, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 255, 0]],
    ]
    sense = harness.SceneSense(
        images={harness.WRIST_KEY: image, harness.BEV_KEY: image},
        arm_positions=(0.1, 0, 0, 0, 0, 0),
    )
    obs = harness.build_policy_obs(sense)
    wrist = obs[harness.WRIST_KEY]
    assert wrist.shape == (1, 3, 2, 2)
    assert wrist.dtype == numpy.float32
    assert wrist[0, 0, 0, 0] == pytest.approx(1.0)
    assert obs[harness.STATE_KEY].shape == (1, 9)
    assert obs[harness.TASK_KEY] == harness.TASK_NAME


def test_joint_limits_match_arm_params():
    text = (
        Path(__file__).resolve().parents[2]
        / "omnibot-ros2"
        / "omnibot_arm"
        / "config"
        / "arm_params.yaml"
    ).read_text(encoding="utf-8")
    assert "joint_min: [-3.14, -1.57, -1.57, -1.57, -3.14, -0.1]" in text
    assert "joint_max: [ 3.14,  1.57,  1.57,  1.57,  3.14,  0.8]" in text
    assert harness.JOINT_MIN == (-3.14, -1.57, -1.57, -1.57, -3.14, -0.1)
    assert harness.JOINT_MAX == (3.14, 1.57, 1.57, 1.57, 3.14, 0.8)
