"""Read nav tuning topics from rosbag2 directories or MCAP files."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from bag_tuning.poses import ImuSample, Pose2D, quat_to_yaw


@dataclass
class BagTopics:
    odom: List[Pose2D]
    imu: List[ImuSample]
    goal: Optional[Pose2D]


def _is_mcap(path: Path) -> bool:
    return path.is_file() and path.suffix.lower() == ".mcap"


def _is_rosbag2(path: Path) -> bool:
    return path.is_dir() and (path / "metadata.yaml").is_file()


def read_bag(
    bag_path: str | Path,
    odom_topic: str = "/odom",
    imu_topic: str = "/imu/data",
    goal_topic: str = "/goal_pose",
    typestore_name: Stores = Stores.ROS2_JAZZY,
) -> BagTopics:
    path = Path(bag_path)
    if _is_rosbag2(path):
        return _read_rosbag2(path, odom_topic, imu_topic, goal_topic, typestore_name)
    if _is_mcap(path):
        return _read_mcap(path, odom_topic, imu_topic, goal_topic, typestore_name)
    raise FileNotFoundError(
        f"Unsupported bag path {path} (expected rosbag2 directory or .mcap file)"
    )


def _read_rosbag2(
    path: Path,
    odom_topic: str,
    imu_topic: str,
    goal_topic: str,
    typestore_name: Stores,
) -> BagTopics:
    typestore = get_typestore(typestore_name)
    odom: List[Pose2D] = []
    imu: List[ImuSample] = []
    goal: Optional[Pose2D] = None

    with AnyReader([path], default_typestore=typestore) as reader:
        conns = {c.topic: c for c in reader.connections}
        for topic, dest in (
            (odom_topic, "odom"),
            (imu_topic, "imu"),
            (goal_topic, "goal"),
        ):
            if topic not in conns:
                continue
            for _conn, t, raw in reader.messages(connections=[conns[topic]]):
                msg = typestore.deserialize_cdr(raw, conns[topic].msgtype)
                if dest == "odom":
                    q = msg.pose.pose.orientation
                    odom.append(
                        Pose2D(
                            int(t),
                            float(msg.pose.pose.position.x),
                            float(msg.pose.pose.position.y),
                            quat_to_yaw(q.x, q.y, q.z, q.w),
                        )
                    )
                elif dest == "imu":
                    q = msg.orientation
                    imu.append(
                        ImuSample(
                            int(t),
                            quat_to_yaw(q.x, q.y, q.z, q.w),
                            float(msg.angular_velocity.z),
                        )
                    )
                else:
                    q = msg.pose.orientation
                    goal = Pose2D(
                        int(t),
                        float(msg.pose.position.x),
                        float(msg.pose.position.y),
                        quat_to_yaw(q.x, q.y, q.z, q.w),
                    )

    odom.sort(key=lambda p: p.t_ns)
    imu.sort(key=lambda p: p.t_ns)
    return BagTopics(odom=odom, imu=imu, goal=goal)


def _read_mcap(
    path: Path,
    odom_topic: str,
    imu_topic: str,
    goal_topic: str,
    typestore_name: Stores,
) -> BagTopics:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    odom: List[Pose2D] = []
    imu: List[ImuSample] = []
    goal: Optional[Pose2D] = None
    decoder_factory = DecoderFactory()

    with path.open("rb") as f:
        reader = make_reader(f, decoder_factories=[decoder_factory])
        for _schema, channel, message, ros_msg in reader.iter_decoded_messages():
            topic = channel.topic
            t = message.log_time
            if topic == odom_topic:
                q = ros_msg.pose.pose.orientation
                odom.append(
                    Pose2D(
                        int(t),
                        float(ros_msg.pose.pose.position.x),
                        float(ros_msg.pose.pose.position.y),
                        quat_to_yaw(q.x, q.y, q.z, q.w),
                    )
                )
            elif topic == imu_topic:
                q = ros_msg.orientation
                imu.append(
                    ImuSample(
                        int(t),
                        quat_to_yaw(q.x, q.y, q.z, q.w),
                        float(ros_msg.angular_velocity.z),
                    )
                )
            elif topic == goal_topic:
                q = ros_msg.pose.orientation
                goal = Pose2D(
                    int(t),
                    float(ros_msg.pose.position.x),
                    float(ros_msg.pose.position.y),
                    quat_to_yaw(q.x, q.y, q.z, q.w),
                )

    odom.sort(key=lambda p: p.t_ns)
    imu.sort(key=lambda p: p.t_ns)
    return BagTopics(odom=odom, imu=imu, goal=goal)
