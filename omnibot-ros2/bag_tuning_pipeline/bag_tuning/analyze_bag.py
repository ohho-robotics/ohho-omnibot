"""Run drift analysis and write EKF proposal artifacts."""

from __future__ import annotations

import argparse
from pathlib import Path

from bag_tuning.bag_reader import read_bag
from bag_tuning.drift import analyze_odom_imu_drift
from bag_tuning.ekf_proposal import (
    propose_process_noise,
    write_ekf_proposal_markdown,
    write_ekf_proposal_yaml,
)
from bag_tuning.topic_fixture import fixture_imu, fixture_odom, load_topic_fixture


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("bag_tuning_out"),
        help="directory for ekf_proposal.yaml and ekf_proposal.md",
    )
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--goal-topic", default="/goal_pose")
    args = parser.parse_args(argv)

    if args.bag.suffix.lower() == ".json":
        data = load_topic_fixture(args.bag)
        odom = fixture_odom(data, args.odom_topic)
        imu = fixture_imu(data, args.imu_topic)
    else:
        topics = read_bag(
            args.bag,
            odom_topic=args.odom_topic,
            imu_topic=args.imu_topic,
            goal_topic=args.goal_topic,
        )
        odom, imu = topics.odom, topics.imu

    report = analyze_odom_imu_drift(odom, imu)
    proposal = propose_process_noise(report)
    for line in report.summary_lines():
        print(line)

    out_dir = args.out_dir
    write_ekf_proposal_yaml(out_dir / "ekf_proposal.yaml", proposal)
    write_ekf_proposal_markdown(out_dir / "ekf_proposal.md", report, proposal)
    print(f"wrote {out_dir / 'ekf_proposal.yaml'}")
    print(f"wrote {out_dir / 'ekf_proposal.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
