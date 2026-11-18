"""Bag reader tests against committed rosbag2 and MCAP fixtures."""

from pathlib import Path

import pytest

from bag_tuning.bag_reader import read_bag

ROOT = Path(__file__).resolve().parents[1]
ROSBAG = ROOT / "fixtures" / "rosbag2" / "nav_tune_mini"
MCAP = ROOT / "fixtures" / "mcap" / "nav_tune_mini"


@pytest.mark.skipif(not (ROSBAG / "metadata.yaml").is_file(), reason="run scripts/build_fixtures.py")
def test_read_rosbag2_fixture():
    topics = read_bag(ROSBAG)
    assert len(topics.odom) == 21
    assert len(topics.imu) == 21
    assert topics.goal is not None
    assert abs(topics.goal.x - 1.0) < 1e-6


@pytest.mark.skipif(not (MCAP / "metadata.yaml").is_file(), reason="run scripts/build_fixtures.py")
def test_read_mcap_fixture():
    topics = read_bag(MCAP)
    assert len(topics.odom) == 21
    assert topics.goal is not None
