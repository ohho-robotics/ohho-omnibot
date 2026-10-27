"""Synthetic LeRobot v2 fixtures for the VR vs leader quality comparison.

No robot, no camera, and no Hugging Face client. Parquet rows are tiny and
camera frames are 8x8 arrays so the OHH-99 frozen-video rule can run without
OpenCV.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import pytest

ENGINES = Path(__file__).resolve().parents[2]
ROOT = ENGINES.parent
if str(ENGINES) not in sys.path:
    sys.path.insert(0, str(ENGINES))

from data_engine.quality.episode_filter import filter_dataset
from data_engine.schema.camera_keys import (
    OBSERVATION_IMAGE_BEV,
    OBSERVATION_IMAGE_WRIST,
)
from lerobot_engine.tools.compare_dataset_quality import (
    COMPARISON_ROWS,
    SOURCE_LEADER,
    SOURCE_VR,
    build_report,
    main,
    motion_metrics,
    normalize_source_label,
    reason_codes,
    timestamps_to_seconds,
    _frame_drops,
)

FPS = 30.0
SCRIPT = ROOT / "omnibot-ai-engines" / "lerobot_engine" / "tools" / "compare_dataset_quality.py"


def _moving_frames(count: int, *, freeze_wrist: bool) -> tuple[np.ndarray, np.ndarray]:
    wrist = np.zeros((count, 8, 8, 3), dtype=np.uint8)
    bev = np.zeros((count, 8, 8, 3), dtype=np.uint8)
    for index in range(count):
        wrist[index] = (index * 17) % 256
        bev[index] = (index * 29) % 256
    if freeze_wrist and count > 41:
        wrist[10:41] = wrist[10]
    return wrist, bev


def _write_frames(root: Path, episode_index: int, wrist: np.ndarray, bev: np.ndarray) -> None:
    chunk = root / "videos" / "chunk-000"
    for key, frames in (
        (OBSERVATION_IMAGE_WRIST, wrist),
        (OBSERVATION_IMAGE_BEV, bev),
    ):
        directory = chunk / key
        directory.mkdir(parents=True, exist_ok=True)
        np.save(directory / f"episode_{episode_index:06d}.npy", frames)


def _episode_columns(spec: dict) -> dict:
    count = spec["n"]
    fps = spec.get("fps", FPS)
    timestamps = spec.get("timestamps")
    if timestamps is None:
        timestamps = np.arange(count, dtype=np.float64) / fps
    timestamps = np.asarray(timestamps, dtype=np.float64)
    action = spec.get("action")
    if action is None:
        action = np.zeros((count, 9), dtype=np.float64)
    states = spec.get("states")
    if states is None:
        states = np.zeros((count, 9), dtype=np.float64)
        if count > 1:
            states[:, 0] = np.linspace(0.0, 0.2, count)
    wrist_ts = np.asarray(spec.get("wrist_ts", timestamps), dtype=np.float64)
    bev_ts = np.asarray(spec.get("bev_ts", timestamps), dtype=np.float64)
    columns = {
        "observation.state": [states[index].tolist() for index in range(count)],
        "action": [np.asarray(action[index], dtype=np.float64).tolist() for index in range(count)],
        "frame_index": list(range(count)),
        "episode_index": [spec["episode_index"]] * count,
        "index": list(range(count)),
        "task_index": [0] * count,
        "next.done": [index == count - 1 for index in range(count)],
    }
    if spec.get("include_timestamp", True):
        columns["timestamp"] = timestamps.tolist()
    if spec.get("include_camera_time", True):
        columns["observation.images.wrist.timestamp"] = wrist_ts.tolist()
        columns["observation.images.bev.timestamp"] = bev_ts.tolist()
    return columns


def _meta_row(spec: dict) -> dict:
    row = {
        "episode_index": spec["episode_index"],
        "tasks": [0],
        "length": spec["n"],
        "task": spec.get("task", "pick cube into bin"),
        "operator": spec.get("operator", ""),
        "robot_kind": spec.get("robot_kind", "real"),
        "sim_world": spec.get("sim_world", ""),
    }
    if spec.get("action_source"):
        row["action_source"] = spec["action_source"]
    if spec.get("source"):
        row["source"] = spec["source"]
    return row


def write_v2(root: Path, episodes: list[dict], *, info_extra: dict | None = None) -> None:
    """One parquet per episode, plus the OHH-226 meta files."""
    meta = root / "meta"
    meta.mkdir(parents=True, exist_ok=True)
    records = []
    total = 0
    fps = FPS
    for spec in episodes:
        fps = spec.get("fps", FPS)
        columns = _episode_columns(spec)
        chunk = root / "data" / "chunk-000"
        chunk.mkdir(parents=True, exist_ok=True)
        pq.write_table(
            pa.table(columns),
            chunk / f"episode_{spec['episode_index']:06d}.parquet",
        )
        wrist, bev = _moving_frames(spec["n"], freeze_wrist=spec.get("freeze_wrist", False))
        _write_frames(root, spec["episode_index"], wrist, bev)
        records.append(_meta_row(spec))
        total += spec["n"]
    info = {
        "codebase_version": "v2.0",
        "fps": fps,
        "total_episodes": len(records),
        "total_frames": total,
        "chunks_size": 1000,
        "robot_type": "omnibot",
    }
    if info_extra:
        info.update(info_extra)
    (meta / "info.json").write_text(json.dumps(info), encoding="utf-8")
    (meta / "episodes.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in records),
        encoding="utf-8",
    )
    (meta / "tasks.jsonl").write_text(
        json.dumps({"task_index": 0, "task": "pick cube into bin"}) + "\n",
        encoding="utf-8",
    )
    (meta / "stats.json").write_text("{}\n", encoding="utf-8")


def _constant(count: int) -> np.ndarray:
    return np.zeros((count, 9), dtype=np.float64)


def _cubic(count: int, jerk: float, fps: float = FPS) -> tuple[np.ndarray, np.ndarray]:
    timestamps = np.arange(count, dtype=np.float64) / fps
    action = np.zeros((count, 9), dtype=np.float64)
    action[:, 0] = (jerk / 6.0) * timestamps**3
    return action, timestamps


def _leader_and_vr(tmp_path: Path) -> tuple[Path, Path]:
    vr = tmp_path / "vr"
    leader = tmp_path / "leader"
    write_v2(
        vr,
        [
            {"episode_index": 0, "n": 90, "operator": "vr", "action": _constant(90)},
            {"episode_index": 1, "n": 90, "operator": "vr", "action": _constant(90)},
        ],
    )
    drop_time = np.arange(90, dtype=np.float64) / FPS
    drop_time[50:] += 0.5
    cubic, _cubic_time = _cubic(90, jerk=10.0)
    # Above 1e12 so the OHH-99 nanosecond rule converts, and still precise in float64.
    nanosecond = 2e12 + np.arange(90, dtype=np.float64) * 1e9 / FPS
    write_v2(
        leader,
        [
            {
                "episode_index": 0,
                "n": 90,
                "timestamps": drop_time,
                "action": _constant(90),
            },
            {"episode_index": 1, "n": 30, "action": _constant(30)},
            {
                "episode_index": 2,
                "n": 90,
                "bev_ts": np.arange(90, dtype=np.float64) / FPS + 0.08,
                "action": _constant(90),
            },
            {
                "episode_index": 3,
                "n": 90,
                "freeze_wrist": True,
                "action": _constant(90),
            },
            {"episode_index": 4, "n": 90, "action": cubic, "timestamps": nanosecond},
        ],
    )
    return vr, leader


def test_third_difference_of_a_cubic_is_constant_jerk():
    action, timestamps = _cubic(40, jerk=2.0)
    stats = motion_metrics(
        action[:, :1],
        timestamps,
        fps=FPS,
        drop_factor=1.5,
        timestamps_synthetic=False,
    )
    assert stats["mean_jerk"] == pytest.approx(2.0, abs=1e-6)
    assert stats["max_jerk"] == pytest.approx(2.0, abs=1e-6)


def test_constant_action_is_smooth_and_has_no_drops():
    timestamps = np.arange(12, dtype=np.float64) / FPS
    stats = motion_metrics(
        np.zeros((12, 9)),
        timestamps,
        fps=FPS,
        drop_factor=1.5,
        timestamps_synthetic=False,
    )
    assert stats["mean_jerk"] == pytest.approx(0.0, abs=1e-12)
    assert stats["action_smoothness"] == pytest.approx(1.0)
    assert stats["mean_action_l2_step"] == pytest.approx(0.0, abs=1e-12)
    assert stats["frame_drop_gaps"] == 0
    assert stats["dropped_frames"] == 0


def test_short_signal_has_smoothness_but_no_jerk():
    timestamps = np.arange(3, dtype=np.float64) / FPS
    stats = motion_metrics(
        np.ones((3, 2)),
        timestamps,
        fps=FPS,
        drop_factor=1.5,
        timestamps_synthetic=False,
    )
    assert stats["mean_jerk"] is None
    assert stats["action_smoothness"] == pytest.approx(1.0)


def test_frame_drop_counts_exact_gaps():
    # 0.25 s is one frame at 4 Hz. 1.0 s is four frame periods: three missing.
    gaps, dropped, max_gap = _frame_drops(
        np.array([0.25, 0.25, 1.0, 0.25]),
        fps=4.0,
        drop_factor=1.5,
    )
    assert (gaps, dropped, max_gap) == (1, 3, 1.0)
    jitter_gaps, jitter_dropped, _ = _frame_drops(
        np.array([0.25, 0.30, 0.25]),
        fps=4.0,
        drop_factor=1.5,
    )
    assert (jitter_gaps, jitter_dropped) == (0, 0)


def test_nanosecond_timestamps_convert_before_drop_checks():
    raw = 2e12 + np.arange(10, dtype=np.float64) * 1e9 / FPS
    seconds = timestamps_to_seconds(raw)
    assert seconds[0] == pytest.approx(2e3, rel=1e-6)
    assert np.diff(seconds) == pytest.approx(1.0 / FPS, rel=1e-6)
    gaps, dropped, _ = _frame_drops(np.diff(seconds), fps=FPS, drop_factor=1.5)
    assert (gaps, dropped) == (0, 0)


def test_source_aliases_match_ohh226_names():
    assert normalize_source_label("vr") == SOURCE_VR
    assert normalize_source_label(" leader-arm ") == SOURCE_LEADER
    assert normalize_source_label("leader_arm") == SOURCE_LEADER
    assert normalize_source_label("Ada") is None
    assert normalize_source_label("") is None


def test_vr_versus_leader_aggregates(tmp_path: Path):
    vr, leader = _leader_and_vr(tmp_path)
    report = build_report([(SOURCE_VR, vr), (SOURCE_LEADER, leader)])
    by_source = {
        source: [row for row in report["episodes"] if row["source"] == source]
        for source in (SOURCE_VR, SOURCE_LEADER)
    }
    assert len(by_source[SOURCE_VR]) == 2
    assert len(by_source[SOURCE_LEADER]) == 5
    assert all(row["quality_passed"] for row in by_source[SOURCE_VR])
    assert all(row["action_smoothness"] == pytest.approx(1.0) for row in by_source[SOURCE_VR])
    assert all(row["mean_jerk"] == pytest.approx(0.0, abs=1e-9) for row in by_source[SOURCE_VR])
    assert all(row["frame_drop_gaps"] == 0 for row in by_source[SOURCE_VR])

    leader_by_index = {row["episode_index"]: row for row in by_source[SOURCE_LEADER]}
    assert leader_by_index[0]["quality_passed"] is True
    assert leader_by_index[0]["frame_drop_gaps"] == 1
    assert leader_by_index[0]["dropped_frames"] >= 1
    assert leader_by_index[1]["quality_passed"] is False
    assert "episode_too_short" in reason_codes(leader_by_index[1]["quality_reasons"])
    assert leader_by_index[2]["quality_passed"] is False
    assert "camera_timestamp_skew" in reason_codes(leader_by_index[2]["quality_reasons"])
    assert leader_by_index[2]["max_camera_skew_s"] == pytest.approx(0.08, abs=1e-6)
    assert leader_by_index[3]["quality_passed"] is False
    assert "frozen_video" in reason_codes(leader_by_index[3]["quality_reasons"])
    assert leader_by_index[4]["quality_passed"] is True
    assert leader_by_index[4]["duration_s"] == pytest.approx(89.0 / FPS, rel=1e-4)
    assert leader_by_index[4]["frame_drop_gaps"] == 0
    assert leader_by_index[4]["mean_jerk"] == pytest.approx(10.0, abs=1e-3)

    vr_agg = report["sources"][SOURCE_VR]
    leader_agg = report["sources"][SOURCE_LEADER]
    assert vr_agg["episode_count"] == 2
    assert vr_agg["kept_episodes"] == 2
    assert vr_agg["rejected_episodes"] == 0
    assert vr_agg["reject_reasons"] == "none"
    assert leader_agg["kept_episodes"] == 2
    assert leader_agg["rejected_episodes"] == 3
    assert leader_agg["reject_reason_counts"] == {
        "camera_timestamp_skew": 1,
        "episode_too_short": 1,
        "frozen_video": 1,
    }
    assert leader_agg["mean_jerk"] > vr_agg["mean_jerk"]
    assert leader_agg["max_camera_skew_ms"] == pytest.approx(80.0, abs=1e-3)
    assert vr_agg["mean_action_smoothness"] == pytest.approx(1.0)
    assert leader_agg["mean_action_smoothness"] < vr_agg["mean_action_smoothness"]
    assert leader_agg["frame_drop_gaps"] >= 1
    assert vr_agg["frame_drop_gaps"] == 0

    by_id = {row["id"]: row for row in report["comparison"]}
    assert by_id["episode_count"]["vr"] == 2
    assert by_id["episode_count"]["leader"] == 5
    assert by_id["episode_count"]["delta_vr_minus_leader"] == -3
    assert by_id["reject_reasons"]["delta_vr_minus_leader"] is None
    assert "frozen_video: 1" in by_id["reject_reasons"]["leader"]


def test_quality_reasons_match_the_ohh99_filter(tmp_path: Path):
    _vr, leader = _leader_and_vr(tmp_path)
    filtered = filter_dataset(leader)
    report = build_report([(SOURCE_LEADER, leader)])
    scored = {row["episode_index"]: row for row in report["episodes"]}
    assert set(scored) == set(filtered.episodes)
    for index, score in filtered.episodes.items():
        assert scored[index]["quality_passed"] is score.passed
        assert reason_codes(scored[index]["quality_reasons"]) == reason_codes(score.reasons)


def test_dataset_flag_uses_action_source_then_operator(tmp_path: Path):
    root = tmp_path / "mixed"
    write_v2(
        root,
        [
            {"episode_index": 0, "n": 90, "operator": "vr"},
            {"episode_index": 1, "n": 90, "operator": "", "action_source": "leader"},
            {"episode_index": 2, "n": 90, "operator": "Ada"},
        ],
    )
    report = build_report([("dataset", root)])
    sources = [row["source"] for row in report["episodes"]]
    assert sources == [SOURCE_VR, SOURCE_LEADER, "unlabeled"]
    assert report["sources"]["unlabeled"]["episode_count"] == 1
    forced = build_report([(SOURCE_LEADER, root)])
    assert {row["source"] for row in forced["episodes"]} == {SOURCE_LEADER}


def test_info_json_action_source_labels_unspecified_rows(tmp_path: Path):
    root = tmp_path / "tagged"
    write_v2(
        root,
        [{"episode_index": 0, "n": 90, "operator": ""}],
        info_extra={"action_source": "vr"},
    )
    report = build_report([("dataset", root)])
    assert report["episodes"][0]["source"] == SOURCE_VR


def test_missing_timestamp_column_does_not_invent_frame_drops(tmp_path: Path):
    root = tmp_path / "notime"
    write_v2(
        root,
        [{"episode_index": 0, "n": 90, "include_timestamp": False, "operator": "vr"}],
    )
    report = build_report([(SOURCE_VR, root)])
    episode = report["episodes"][0]
    assert episode["timestamps_synthetic"] is True
    assert episode["frame_drop_gaps"] is None
    assert episode["dropped_frames"] is None
    assert episode["duration_s"] == pytest.approx(89.0 / FPS, rel=1e-6)
    assert episode["quality_passed"] is True
    assert report["sources"][SOURCE_VR]["frame_drop_rate"] is None


def test_chunk_parquet_splits_on_episode_index(tmp_path: Path):
    root = tmp_path / "chunked"
    specs = [
        {"episode_index": 0, "n": 90, "action_source": "vr"},
        {"episode_index": 1, "n": 90, "action_source": "leader-arm"},
    ]
    merged: dict[str, list] = {}
    for spec in specs:
        for key, values in _episode_columns(spec).items():
            merged.setdefault(key, []).extend(values)
        wrist, bev = _moving_frames(spec["n"], freeze_wrist=False)
        _write_frames(root, spec["episode_index"], wrist, bev)
    data = root / "data" / "chunk-000"
    data.mkdir(parents=True, exist_ok=True)
    pq.write_table(pa.table(merged), data / "file-000.parquet")
    meta = root / "meta"
    meta.mkdir(parents=True, exist_ok=True)
    (meta / "info.json").write_text(
        json.dumps({"codebase_version": "v2.0", "fps": FPS, "chunks_size": 1000}),
        encoding="utf-8",
    )
    (meta / "episodes.jsonl").write_text(
        "".join(json.dumps(_meta_row(spec)) + "\n" for spec in specs),
        encoding="utf-8",
    )
    report = build_report([("dataset", root)])
    assert [row["source"] for row in report["episodes"]] == [SOURCE_VR, SOURCE_LEADER]
    assert all(row["quality_passed"] for row in report["episodes"])
    assert all(row["frame_count"] == 90 for row in report["episodes"])


def test_per_episode_parquet_wins_over_chunk_file(tmp_path: Path):
    root = tmp_path / "both"
    clean = {"episode_index": 0, "n": 90, "action": _constant(90), "operator": "vr"}
    write_v2(root, [clean])
    wild = _constant(90)
    wild[40:, 0] = 5.0
    extra = _episode_columns({**clean, "action": wild})
    pq.write_table(
        pa.table(extra),
        root / "data" / "chunk-000" / "file-000.parquet",
    )
    report = build_report([(SOURCE_VR, root)])
    episode = report["episodes"][0]
    assert episode["frame_count"] == 90
    assert episode["mean_jerk"] == pytest.approx(0.0, abs=1e-9)
    assert any("ignored" in warning for warning in report["datasets"][0]["warnings"])


def test_only_vr_leaves_leader_column_empty(tmp_path: Path):
    vr, _leader = _leader_and_vr(tmp_path)
    report = build_report([(SOURCE_VR, vr)])
    row = next(item for item in report["comparison"] if item["id"] == "episode_count")
    assert row["vr"] == 2
    assert row["leader"] is None
    assert row["delta_vr_minus_leader"] is None
    assert SOURCE_LEADER not in report["sources"]


def test_cli_writes_markdown_and_json(tmp_path: Path):
    vr, leader = _leader_and_vr(tmp_path)
    json_out = tmp_path / "out" / "quality.json"
    markdown_out = tmp_path / "out" / "quality.md"
    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--vr",
            str(vr),
            "--leader",
            str(leader),
            "--json-out",
            str(json_out),
            "--markdown-out",
            str(markdown_out),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    payload = json.loads(json_out.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 1
    assert payload["sources"]["leader"]["rejected_episodes"] == 3
    markdown = markdown_out.read_text(encoding="utf-8")
    assert "| Episodes | 2 | 5 | -3 |" in markdown
    assert "| Rejected by quality filter | 0 | 3 |" in markdown
    assert "camera_timestamp_skew: 1" in markdown
    assert "frozen_video" in markdown
    assert "n/a" in markdown or "Reject reasons" in markdown


def test_cli_rejects_missing_numpy_and_duplicate_paths(tmp_path: Path):
    assert _exits(main, [])
    missing = tmp_path / "nope"
    assert main(["--vr", str(missing)]) == 2
    empty = tmp_path / "empty"
    empty.mkdir()
    assert main(["--leader", str(empty)]) == 2
    numpy_root = tmp_path / "numpy"
    episode = numpy_root / "episode_00000"
    episode.mkdir(parents=True)
    (episode / "data.npz").write_bytes(b"")
    assert main(["--leader", str(numpy_root)]) == 2
    vr, _leader = _leader_and_vr(tmp_path)
    assert main(["--vr", str(vr), "--leader", str(vr)]) == 2


def _exits(func, argv: list[str]) -> bool:
    with pytest.raises(SystemExit) as caught:
        func(argv)
    return caught.value.code == 2


def test_tool_does_not_reference_training_or_upload():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "import torch" not in text
    assert "huggingface" not in text.lower()
    assert "LeRobotDataset.create" not in text


def test_docs_template_has_empty_real_columns_and_commands():
    text = (ROOT / "docs" / "vr-vs-leader-quality.md").read_text(encoding="utf-8")
    assert "omnibot-ai-engines/lerobot_engine/tools/compare_dataset_quality.py" in text
    assert "--vr ~/datasets/mobile_manipulation_vr" in text
    assert "--leader ~/datasets/mobile_manipulation_leader" in text
    assert "--json-out docs/evidence/ohh-244/vr-vs-leader-quality.json" in text
    assert "--markdown-out docs/evidence/ohh-244/vr-vs-leader-quality.md" in text
    assert "action_source:=vr" in text
    assert "action_source:=leader" in text
    assert 'task_name:="pick cube into bin"' in text
    assert "does not upload" in text
    assert "not on pypi" not in text.lower()
    marker = "<!-- real-data-table -->"
    assert marker in text
    block = text.split(marker, 1)[1].split("\n## ", 1)[0]
    rows = [line for line in block.splitlines() if line.startswith("|")]
    body = [line for line in rows[2:] if "---" not in line]
    labels = [label for _row_id, label, _field in COMPARISON_ROWS]
    assert [line.split("|")[1].strip() for line in body] == labels
    for line in body:
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        assert cells[1:] == ["", "", ""]
