"""Plot odom trajectory vs navigation goal from a bag or topic fixture."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from bag_tuning.bag_reader import read_bag
from bag_tuning.poses import final_goal_error
from bag_tuning.topic_fixture import fixture_goal, fixture_odom, load_topic_fixture


def plot_trajectory_vs_goal(
    odom_xy: list[tuple[float, float]],
    goal_xy: tuple[float, float] | None,
    out_path: Path,
    title: str,
) -> dict:
    fig, ax = plt.subplots(figsize=(6, 6))
    if odom_xy:
        xs, ys = zip(*odom_xy)
        ax.plot(xs, ys, "-o", markersize=3, label="odom path")
        ax.plot(xs[0], ys[0], "go", label="start")
        ax.plot(xs[-1], ys[-1], "rx", label="end")
    if goal_xy is not None:
        ax.plot(goal_xy[0], goal_xy[1], "b*", markersize=12, label="goal")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title)
    ax.legend(loc="best")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return {"plot": str(out_path), "points": len(odom_xy)}


def _load_from_fixture(path: Path):
    data = load_topic_fixture(path)
    odom = fixture_odom(data)
    goal = fixture_goal(data)
    return odom, goal


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bag",
        type=Path,
        help="rosbag2 directory, .mcap file, or topics JSON fixture",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("trajectory_vs_goal.png"),
        help="output PNG path",
    )
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--goal-topic", default="/goal_pose")
    args = parser.parse_args(argv)

    bag = args.bag
    if bag.suffix.lower() == ".json":
        odom, goal = _load_from_fixture(bag)
    else:
        topics = read_bag(
            bag,
            odom_topic=args.odom_topic,
            imu_topic=args.imu_topic,
            goal_topic=args.goal_topic,
        )
        odom, goal = topics.odom, topics.goal

    odom_xy = [(p.x, p.y) for p in odom]
    goal_xy = (goal.x, goal.y) if goal else None
    meta = plot_trajectory_vs_goal(
        odom_xy,
        goal_xy,
        args.output,
        title=f"trajectory vs goal ({bag.name})",
    )
    if goal and odom:
        xy_err, yaw_err = final_goal_error(odom, goal)
        meta["final_xy_error_m"] = xy_err
        meta["final_yaw_error_rad"] = yaw_err
        print(f"final xy error: {xy_err:.3f} m, yaw error: {yaw_err:.3f} rad")
    print(f"wrote {meta['plot']} ({meta['points']} odom points)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
