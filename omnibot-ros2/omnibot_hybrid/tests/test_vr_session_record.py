"""VR session MCAP topic list, rotation, and size cap. No ROS."""

from __future__ import annotations

import ast
import json
from datetime import datetime, timezone
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

from omnibot_hybrid.vr_session_record import (
    DEFAULT_MAX_BAG_SIZE_BYTES,
    DEFAULT_MAX_TOTAL_BYTES,
    EXCLUDED_PROOF_TOPIC,
    PROOF_TOPICS,
    TOPIC_GLOBS,
    ZERO_TWIST,
    HeartbeatTracker,
    MetricsEndpoint,
    RecorderConfig,
    SafetyTripCounter,
    SessionRecorder,
    parse_rtt_ms,
    record_argv,
    safety_trip,
    topic_regex,
    topic_selected,
    topics_in_bag_info,
    prune_to_cap,
)

REPO = Path(__file__).resolve().parents[3]
HYBRID = REPO / "omnibot-ros2" / "omnibot_hybrid"
SIM_LAUNCH = REPO / "omnibot-ros2" / "omnibot_bringup" / "launch" / "sim.launch.py"
RECORDER_LAUNCH = HYBRID / "launch" / "vr_session_recorder.launch.py"
NODE_SOURCE = HYBRID / "omnibot_hybrid" / "vr_session_recorder.py"
FOXGLOVE = REPO / "foxglove" / "vr_session.json"
GRAFANA = REPO / "infra" / "observability" / "grafana" / "dashboards" / "vr-session.json"
SIM_SCRIPT = REPO / "scripts" / "ohh239_sim_mcap.py"
WALL = datetime(2026, 10, 14, 12, 0, 0, tzinfo=timezone.utc)

SELECTED = (
    "/cmd_vel/teleop",
    "/cmd_vel/teleop_raw",
    "/arm/joint_commands/teleop",
    "/arm/joint_commands/teleop_raw",
    "/ohho/heartbeat",
    "/ohho/telemetry",
    "/ohho/safety/state",
    "/control_mode",
    "/control_mode/active",
)
REJECTED = (
    "/cmd_vel",
    "/cmd_vel/out",
    "/ohho/heartbeat_echo",
    "/ohho/safety/state/extra",
    "/arm/joint_commands/out",
    "/arm/joint_commands/policy",
    "/camera/front/image_raw",
    "/scan",
    "/tf",
)


class FakeRunner:
    def __init__(self) -> None:
        self.commands: list[list[str]] = []
        self.stops = 0
        self._on = False

    def start(self, cmd, log_path=None) -> None:
        del log_path
        if self._on:
            raise RuntimeError("already running")
        self.commands.append(list(cmd))
        self._on = True

    def stop(self, timeout: float = 10.0) -> None:
        del timeout
        if self._on:
            self.stops += 1
        self._on = False

    def running(self) -> bool:
        return self._on


def _recorder(tmp_path: Path, sizes: dict[str, int], **kwargs) -> tuple[SessionRecorder, FakeRunner]:
    runner = FakeRunner()
    config = RecorderConfig(output_dir=tmp_path, **kwargs).validated()
    recorder = SessionRecorder(
        config,
        runner,
        size_of=lambda path: sizes.get(path.name, 0),
    )
    return recorder, runner


def _declared_launch_args(path: Path) -> list[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = getattr(func, "id", None) or getattr(func, "attr", None)
        if name != "DeclareLaunchArgument" or not node.args:
            continue
        if isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str):
            found.append(node.args[0].value)
    return found


def _layout_panels(node):
    if isinstance(node, str):
        return [node]
    if isinstance(node, dict):
        found = []
        for key in ("first", "second"):
            if key in node:
                found.extend(_layout_panels(node[key]))
        return found
    return []


def test_topic_globs_match_the_issue_list():
    assert TOPIC_GLOBS == (
        "/cmd_vel/teleop*",
        "/arm/joint_commands/teleop*",
        "/ohho/heartbeat",
        "/ohho/telemetry",
        "/ohho/safety/state",
        "/control_mode*",
    )


def test_topic_regex_selects_session_topics_only():
    pattern = topic_regex()
    assert pattern.startswith("^")
    assert pattern.endswith("$")
    for name in SELECTED:
        assert topic_selected(name), name
    for name in REJECTED:
        assert not topic_selected(name), name
    for topic, _msg_type, _payload in PROOF_TOPICS:
        assert topic_selected(topic)
    assert not topic_selected(EXCLUDED_PROOF_TOPIC[0])


def test_record_argv_is_mcap_for_those_topics_only():
    cmd = record_argv("/tmp/vr-part", use_sim_time=False)
    assert cmd[:5] == ["ros2", "bag", "record", "-s", "mcap"]
    assert "-a" not in cmd
    assert "--topics" not in cmd
    assert "--max-bag-size" not in cmd
    assert cmd[cmd.index("-o") + 1] == "/tmp/vr-part"
    assert cmd[cmd.index("-e") + 1] == topic_regex()
    assert "--include-unpublished-topics" in cmd
    assert "--use-sim-time" not in cmd
    assert "--use-sim-time" in record_argv("/tmp/vr-part", use_sim_time=True)
    assert "x: 0.0" in ZERO_TWIST
    assert "x: 0.1" not in ZERO_TWIST


def test_size_defaults_fit_the_cap():
    assert DEFAULT_MAX_BAG_SIZE_BYTES <= DEFAULT_MAX_TOTAL_BYTES
    assert DEFAULT_MAX_BAG_SIZE_BYTES > 0


def test_config_rejects_bag_larger_than_total(tmp_path):
    config = RecorderConfig(
        output_dir=tmp_path,
        max_bag_size_bytes=10,
        max_total_bytes=5,
    )
    with pytest.raises(ValueError, match="cannot exceed"):
        config.validated()


def test_config_rejects_non_positive_limits(tmp_path):
    with pytest.raises(ValueError):
        RecorderConfig(tmp_path, max_bag_size_bytes=0, max_total_bytes=10).validated()


def test_prune_deletes_oldest_parts_and_keeps_active(tmp_path):
    old = tmp_path / "20200101T000000000000Z" / "part-0000"
    new = tmp_path / "20260101T000000000000Z" / "part-0000"
    old.mkdir(parents=True)
    new.mkdir(parents=True)
    (old / "a.mcap").write_bytes(b"x" * 80)
    (new / "a.mcap").write_bytes(b"y" * 80)
    deleted = prune_to_cap(tmp_path, max_total_bytes=100, active=new)
    assert [path.name for path in deleted] == ["part-0000"]
    assert not old.exists()
    assert new.exists()
    assert new.parent.exists()


def test_prune_keeps_the_active_part_when_it_alone_exceeds_the_cap(tmp_path):
    active = tmp_path / "20260101T000000000000Z" / "part-0000"
    active.mkdir(parents=True)
    (active / "a.mcap").write_bytes(b"x" * 80)
    deleted = prune_to_cap(tmp_path, max_total_bytes=10, active=active)
    assert deleted == []
    assert active.exists()


def test_explicit_start_writes_manifest_and_mcap_command(tmp_path):
    recorder, runner = _recorder(
        tmp_path,
        {},
        max_bag_size_bytes=1000,
        max_total_bytes=5000,
        auto_session=False,
    )
    notes = recorder.start(0.0, "sim", wall=WALL)
    assert notes and notes[0].startswith("recording ")
    assert runner.running()
    cmd = runner.commands[0]
    assert "mcap" in cmd
    assert topic_regex() in cmd
    part = recorder.active_part
    assert part is not None
    assert part.name == "part-0000"
    assert cmd[cmd.index("-o") + 1] == str(part)
    manifest = json.loads((part.parent / "session.json").read_text(encoding="utf-8"))
    assert manifest["topics"] == list(TOPIC_GLOBS)
    assert manifest["storage"] == "mcap"
    assert manifest["max_bag_size_bytes"] == 1000
    assert manifest["max_total_bytes"] == 5000
    assert recorder.status_text().startswith("recording:")


def test_rotation_and_size_cap(tmp_path):
    sizes = {"part-0000": 100, "part-0001": 100, "part-0002": 0}
    recorder, runner = _recorder(
        tmp_path,
        sizes,
        max_bag_size_bytes=100,
        max_total_bytes=150,
        auto_session=False,
    )
    recorder.start(0.0, "sim", wall=WALL)
    part0 = recorder.active_part
    assert part0 is not None
    part0.mkdir(parents=True)
    recorder.poll(1.0)
    part1 = recorder.active_part
    assert part1 is not None
    assert part1.name == "part-0001"
    assert part0.exists()
    part1.mkdir(parents=True)
    recorder.poll(2.0)
    assert recorder.active_part is not None
    assert recorder.active_part.name == "part-0002"
    assert not part0.exists()
    assert part1.exists()
    assert len(runner.commands) == 3
    assert runner.stops == 2
    assert runner.running()
    assert all(topic_regex() in cmd for cmd in runner.commands)


def test_explicit_session_does_not_end_on_heartbeat_gap(tmp_path):
    recorder, runner = _recorder(
        tmp_path,
        {},
        max_bag_size_bytes=1000,
        max_total_bytes=5000,
        auto_session=True,
        session_idle_sec=2.0,
    )
    recorder.start(0.0, "manual", wall=WALL)
    recorder.heartbeat(0.2)
    assert recorder.poll(20.0) == []
    assert runner.running()
    assert "ohho_vr_heartbeat_gaps_total 1" in recorder.render_metrics()


def test_heartbeat_starts_session_and_gap_stops_it(tmp_path):
    recorder, runner = _recorder(
        tmp_path,
        {},
        max_bag_size_bytes=1000,
        max_total_bytes=5000,
        auto_session=True,
        session_idle_sec=2.0,
    )
    assert recorder.heartbeat(0.0)
    recorder.heartbeat(1.0)
    assert len(runner.commands) == 1
    notes = recorder.poll(3.1)
    assert any(note.startswith("stopped") for note in notes)
    assert not runner.running()
    assert recorder.status_text() == "idle"


def test_explicit_stop_holds_auto_until_heartbeat_goes_quiet(tmp_path):
    recorder, runner = _recorder(
        tmp_path,
        {},
        max_bag_size_bytes=1000,
        max_total_bytes=5000,
        auto_session=True,
        session_idle_sec=2.0,
    )
    recorder.start(0.0, "manual", wall=WALL)
    recorder.heartbeat(1.0)
    recorder.stop(1.1)
    assert not runner.running()
    recorder.heartbeat(1.2)
    recorder.poll(1.3)
    recorder.heartbeat(1.4)
    assert len(runner.commands) == 1
    recorder.poll(3.5)
    recorder.heartbeat(3.6)
    assert len(runner.commands) == 2
    assert runner.running()


def test_heartbeat_gap_counter_counts_one_excursion():
    tracker = HeartbeatTracker(0.5)
    tracker.beat(0.0)
    tracker.tick(0.4)
    assert tracker.gaps_total == 0
    tracker.tick(0.5)
    assert tracker.gaps_total == 1
    assert tracker.gap_sec == pytest.approx(0.5)
    tracker.tick(2.0)
    assert tracker.gaps_total == 1
    tracker.beat(2.1)
    assert tracker.gap_sec == 0.0
    tracker.tick(2.7)
    assert tracker.gaps_total == 2


def test_safety_trip_edges_and_repeats():
    clear = {"armed": True, "latched": False, "reason": ""}
    assert safety_trip(None, clear) is False
    latched = {"armed": False, "latched": True, "reason": "estop"}
    assert safety_trip(None, latched) is True
    assert safety_trip(clear, latched) is True
    assert safety_trip(latched, latched) is False
    watchdog = {"armed": False, "latched": False, "reason": "heartbeat_watchdog"}
    assert safety_trip(clear, watchdog) is True
    command = {"armed": False, "latched": False, "reason": "command_watchdog"}
    assert safety_trip(watchdog, command) is True
    counter = SafetyTripCounter()
    assert counter.observe(json.dumps(clear)) is False
    assert counter.observe(json.dumps(latched)) is True
    assert counter.observe(json.dumps(latched)) is False
    assert counter.total == 1


def test_rtt_and_metrics_text(tmp_path):
    assert parse_rtt_ms('{"link": {"rtt_ms": 12.5, "hz": 10}}') == pytest.approx(12.5)
    assert parse_rtt_ms('{"link": {"rtt_ms": -1}}') is None
    assert parse_rtt_ms("nope") is None
    recorder, _runner = _recorder(
        tmp_path,
        {},
        max_bag_size_bytes=100,
        max_total_bytes=200,
        auto_session=False,
    )
    text = recorder.render_metrics()
    assert "ohho_vr_rtt_ms" not in text
    assert "ohho_vr_heartbeat_gap_seconds" in text
    assert "ohho_vr_heartbeat_gaps_total 0" in text
    assert "ohho_vr_safety_trips_total 0" in text
    recorder.telemetry('{"link": {"rtt_ms": 12.5, "hz": 10}}')
    recorder.telemetry("not-json")
    recorder.safety('{"armed": false, "latched": true, "reason": "estop"}')
    text = recorder.render_metrics()
    assert "ohho_vr_rtt_ms 12.500000" in text
    assert "ohho_vr_safety_trips_total 1" in text


def test_metrics_endpoint_serves_prometheus():
    endpoint = MetricsEndpoint(lambda: "ohho_vr_rtt_ms 1.000000\n", 0)
    try:
        with urlopen(f"http://127.0.0.1:{endpoint.port}/metrics", timeout=2) as resp:
            body = resp.read().decode("utf-8")
            status = resp.status
        assert status == 200
        assert "ohho_vr_rtt_ms 1.000000" in body
        try:
            urlopen(f"http://127.0.0.1:{endpoint.port}/other", timeout=2)
        except HTTPError as exc:
            assert exc.code == 404
        else:
            raise AssertionError("expected 404")
    finally:
        endpoint.close()


def test_bag_info_parser_reads_topic_names():
    sample = "\n".join(
        [
            "Files:             part-0000.mcap",
            "Topic information: Topic: /cmd_vel/teleop | Type: geometry_msgs/msg/Twist | Count: 1",
            "                   Topic: /ohho/heartbeat | Type: std_msgs/msg/Empty | Count: 1",
        ]
    )
    assert topics_in_bag_info(sample) == {"/cmd_vel/teleop", "/ohho/heartbeat"}


def test_recorder_launch_declares_rotation_and_cap():
    text = RECORDER_LAUNCH.read_text(encoding="utf-8")
    args = _declared_launch_args(RECORDER_LAUNCH)
    for name in (
        "output_dir",
        "max_bag_size_bytes",
        "max_total_bytes",
        "auto_session",
        "session_idle_sec",
        "heartbeat_gap_sec",
        "metrics_port",
    ):
        assert name in args
    assert "DEFAULT_MAX_BAG_SIZE_BYTES" in text
    assert "DEFAULT_MAX_TOTAL_BYTES" in text
    assert "vr_session_recorder" in text
    assert "use_sim_time" in text and "False" in text
    assert "sim_vr.launch.py" in text
    assert "record_vr_session" in text


def test_sim_profile_opts_in_to_the_recorder():
    text = SIM_LAUNCH.read_text(encoding="utf-8")
    assert "record_vr_session" in _declared_launch_args(SIM_LAUNCH)
    assert 'default_value="false"' in text
    record_at = text.index("\n        if record_vr_session:")
    vr_at = text.rindex("\n    if vr_on:", 0, record_at)
    include_at = text.index("vr_session_recorder.launch.py", record_at)
    elif_at = text.index("\n    elif record_vr_session:", record_at)
    assert vr_at < record_at < include_at < elif_at
    assert "record_vr_session is ignored unless vr:=true" in text


def test_node_source_does_not_publish_commands():
    text = NODE_SOURCE.read_text(encoding="utf-8")
    assert "geometry_msgs" not in text
    assert "sensor_msgs" not in text
    assert "Twist" not in text
    assert "JointState" not in text
    assert "/emergency_stop" not in text
    assert "estop_reset" not in text
    assert "/cmd_vel" not in text
    assert text.count("create_publisher") == 1
    assert "/ohho/vr_session/recorder" in text
    assert "/ohho/safety/state" in text
    assert "/ohho/telemetry" in text
    assert "/ohho/heartbeat" in text
    assert 'create_subscription(Empty, "/ohho/heartbeat"' in text
    assert 'create_subscription(String, "/ohho/heartbeat"' not in text


def test_foxglove_layout_covers_session_topics():
    data = json.loads(FOXGLOVE.read_text(encoding="utf-8"))
    assert data["version"].startswith("foxglove.studio")
    assert "configById" in data and "layout" in data
    panels = _layout_panels(data["layout"])
    assert panels
    for panel in panels:
        assert panel in data["configById"]
    blob = json.dumps(data)
    for topic in (
        "/cmd_vel/teleop",
        "/cmd_vel/teleop_raw",
        "/arm/joint_commands/teleop",
        "/arm/joint_commands/teleop_raw",
        "/ohho/telemetry",
        "/ohho/safety/state",
        "/ohho/heartbeat",
        "/control_mode/active",
        "/control_mode",
    ):
        assert topic in blob


def test_grafana_panels_cover_rtt_gaps_and_trips():
    data = json.loads(GRAFANA.read_text(encoding="utf-8"))
    assert data["uid"] == "ohho-vr-session"
    assert data["title"] == "VR session"
    blob = json.dumps(data)
    for metric in (
        "ohho_vr_rtt_ms",
        "ohho_vr_heartbeat_gap_seconds",
        "ohho_vr_heartbeat_gaps_total",
        "ohho_vr_safety_trips_total",
    ):
        assert metric in blob
    titles = [panel["title"] for panel in data["panels"]]
    assert "RTT" in titles
    assert "Heartbeat gap" in titles
    assert "Heartbeat gaps" in titles
    assert "Safety trips" in titles


def test_sim_script_stays_on_localhost_with_zero_commands():
    text = SIM_SCRIPT.read_text(encoding="utf-8")
    assert "ROS_LOCALHOST_ONLY" in text
    assert 'ROS_DOMAIN_ID"] = "42"' in text or '["ROS_DOMAIN_ID"] = "42"' in text
    assert "ZERO_TWIST" in text
    assert "x: 0.1" not in text
    assert "wait_for_subscriber=False" in text
