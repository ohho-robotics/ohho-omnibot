"""Plotter and EKF proposal smoke tests."""

import tempfile
from pathlib import Path

from bag_tuning.ekf_proposal import propose_process_noise, write_ekf_proposal_yaml
from bag_tuning.plot_trajectory import plot_trajectory_vs_goal
from bag_tuning.topic_fixture import fixture_goal, fixture_odom, load_topic_fixture

FIXTURE = (
    Path(__file__).resolve().parents[1] / "fixtures" / "topics" / "nav_tune_mini.json"
)


def test_plot_trajectory_png():
    data = load_topic_fixture(FIXTURE)
    odom = fixture_odom(data)
    goal = fixture_goal(data)
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "plot.png"
        meta = plot_trajectory_vs_goal(
            [(p.x, p.y) for p in odom],
            (goal.x, goal.y) if goal else None,
            out,
            "test",
        )
        assert out.is_file()
        assert meta["points"] == 21


def test_ekf_proposal_yaml_written():
    from bag_tuning.drift import analyze_odom_imu_drift
    from bag_tuning.topic_fixture import fixture_imu

    data = load_topic_fixture(FIXTURE)
    report = analyze_odom_imu_drift(fixture_odom(data), fixture_imu(data))
    proposal = propose_process_noise(report)
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "ekf_proposal.yaml"
        write_ekf_proposal_yaml(path, proposal)
        text = path.read_text(encoding="utf-8")
        assert "UNVALIDATED_ON_HARDWARE" in text
        assert "vyaw_process_noise" in text
