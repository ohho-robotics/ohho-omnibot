"""Drift metric tests on JSON fixture."""

from pathlib import Path

from bag_tuning.drift import analyze_odom_imu_drift
from bag_tuning.topic_fixture import fixture_imu, fixture_odom, load_topic_fixture

FIXTURE = (
    Path(__file__).resolve().parents[1] / "fixtures" / "topics" / "nav_tune_mini.json"
)


def test_drift_fixture_has_paired_samples():
    data = load_topic_fixture(FIXTURE)
    report = analyze_odom_imu_drift(fixture_odom(data), fixture_imu(data))
    assert report.sample_count == 21
    assert report.odom_path_length_m > 0.9
    assert report.yaw_rms_rad > 0.0


def test_odom_yaw_delta_matches_fixture():
    data = load_topic_fixture(FIXTURE)
    odom = fixture_odom(data)
    report = analyze_odom_imu_drift(odom, fixture_imu(data))
    assert abs(report.odom_yaw_delta_rad - 0.2) < 1e-6
