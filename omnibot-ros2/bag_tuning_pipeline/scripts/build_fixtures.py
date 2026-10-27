#!/usr/bin/env python3
"""Regenerate committed nav_tune_mini rosbag2, MCAP, and JSON topic fixtures."""

from __future__ import annotations

import json
import math
import shutil
from pathlib import Path

import numpy as np

from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = REPO_ROOT / "fixtures"


def _synthetic_series():
    t0 = 1_700_000_000_000_000_000
    dt = 50_000_000
    odom_rows = []
    imu_rows = []
    for i in range(21):
        t = t0 + i * dt
        x = i * 0.05
        y = i * 0.002
        yaw_odom = 0.01 * i
        yaw_imu = 0.008 * i
        odom_rows.append(
            {"t_ns": t, "x": x, "y": y, "yaw": yaw_odom, "vx": 1.0, "vy": 0.0}
        )
        imu_rows.append({"t_ns": t, "yaw": yaw_imu, "wz": 0.008})
    goal = {"t_ns": t0, "x": 1.0, "y": 0.0, "yaw": 0.0}
    return odom_rows, imu_rows, goal


def write_json_fixture() -> Path:
    odom_rows, imu_rows, goal = _synthetic_series()
    path = FIXTURES / "topics" / "nav_tune_mini.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "description": "Synthetic straight-line odom with slight lateral slip; IMU yaw lags odom.",
        "topics": {
            "/odom": odom_rows,
            "/imu/data": imu_rows,
            "/goal_pose": [goal],
        },
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return path


def _quat_yaw(ts, yaw: float):
    Quat = ts.types["geometry_msgs/msg/Quaternion"]
    return Quat(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _header(ts, t: int, frame_id: str):
    Time = ts.types["builtin_interfaces/msg/Time"]
    Header = ts.types["std_msgs/msg/Header"]
    sec = t // 1_000_000_000
    nanosec = t % 1_000_000_000
    return Header(stamp=Time(sec=int(sec), nanosec=int(nanosec)), frame_id=frame_id)


def _odom_msg(ts, t: int, x: float, y: float, yaw: float):
    Odom = ts.types["nav_msgs/msg/Odometry"]
    Point = ts.types["geometry_msgs/msg/Point"]
    Pose = ts.types["geometry_msgs/msg/Pose"]
    Twist = ts.types["geometry_msgs/msg/Twist"]
    PWC = ts.types["geometry_msgs/msg/PoseWithCovariance"]
    TWC = ts.types["geometry_msgs/msg/TwistWithCovariance"]
    Vector3 = ts.types["geometry_msgs/msg/Vector3"]
    pose = Pose(position=Point(x=x, y=y, z=0.0), orientation=_quat_yaw(ts, yaw))
    twist = Twist(
        linear=Vector3(x=1.0, y=0.0, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=0.0),
    )
    return Odom(
        header=_header(ts, t, "odom"),
        child_frame_id="base_link",
        pose=PWC(pose=pose, covariance=np.zeros(36, dtype=np.float64)),
        twist=TWC(twist=twist, covariance=np.zeros(36, dtype=np.float64)),
    )


def _imu_msg(ts, t: int, yaw: float, wz: float):
    Imu = ts.types["sensor_msgs/msg/Imu"]
    Vector3 = ts.types["geometry_msgs/msg/Vector3"]
    return Imu(
        header=_header(ts, t, "imu_link"),
        orientation=_quat_yaw(ts, yaw),
        orientation_covariance=np.zeros(9, dtype=np.float64),
        angular_velocity=Vector3(x=0.0, y=0.0, z=wz),
        angular_velocity_covariance=np.zeros(9, dtype=np.float64),
        linear_acceleration=Vector3(x=0.0, y=0.0, z=0.0),
        linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
    )


def _goal_msg(ts, t: int):
    Pose = ts.types["geometry_msgs/msg/PoseStamped"]
    Point = ts.types["geometry_msgs/msg/Point"]
    PoseMsg = ts.types["geometry_msgs/msg/Pose"]
    return Pose(
        header=_header(ts, t, "map"),
        pose=PoseMsg(position=Point(x=1.0, y=0.0, z=0.0), orientation=_quat_yaw(ts, 0.0)),
    )


def write_rosbag2_fixture() -> Path:
    out = FIXTURES / "rosbag2" / "nav_tune_mini"
    if out.exists():
        shutil.rmtree(out)

    ts = get_typestore(Stores.ROS2_JAZZY)
    odom_rows, imu_rows, goal = _synthetic_series()

    with Writer(out, version=9) as w:
        c_odom = w.add_connection("/odom", "nav_msgs/msg/Odometry", typestore=ts)
        c_imu = w.add_connection("/imu/data", "sensor_msgs/msg/Imu", typestore=ts)
        c_goal = w.add_connection("/goal_pose", "geometry_msgs/msg/PoseStamped", typestore=ts)

        for row, imu_row in zip(odom_rows, imu_rows):
            t = int(row["t_ns"])
            odom = _odom_msg(ts, t, float(row["x"]), float(row["y"]), float(row["yaw"]))
            imu = _imu_msg(ts, t, float(imu_row["yaw"]), float(imu_row["wz"]))
            w.write(c_odom, t, ts.serialize_cdr(odom, "nav_msgs/msg/Odometry"))
            w.write(c_imu, t, ts.serialize_cdr(imu, "sensor_msgs/msg/Imu"))

        gt = int(goal["t_ns"])
        goal_msg = _goal_msg(ts, gt)
        w.write(c_goal, gt, ts.serialize_cdr(goal_msg, "geometry_msgs/msg/PoseStamped"))

    return out


def write_mcap_fixture(rosbag_path: Path) -> Path:
    """Convert committed rosbag2 fixture to MCAP via rosbags.convert."""
    import subprocess
    import sys

    path = FIXTURES / "mcap" / "nav_tune_mini"
    if path.exists():
        shutil.rmtree(path)
    cmd = [
        sys.executable,
        "-m",
        "rosbags.convert",
        "--src",
        str(rosbag_path),
        "--dst",
        str(path),
        "--dst-storage",
        "mcap",
        "--src-typestore",
        "ros2_jazzy",
        "--dst-typestore",
        "copy",
    ]
    subprocess.run(cmd, check=True)
    return path


def main() -> int:
    json_path = write_json_fixture()
    bag_path = write_rosbag2_fixture()
    mcap_path = write_mcap_fixture(bag_path)
    print(json_path)
    print(bag_path)
    print(mcap_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
