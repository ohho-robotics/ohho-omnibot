"""
LeRobot v2 teleop recording (separate from MCAP / rosbag record flows).

Brings up Xbox gamepad teleop, wrist + BEV cameras, optional leader arm, and the
``teleop_recorder_node`` (ohho-os Recorder + data_engine Parquet/MP4 writer).

Example::

    ros2 launch omnibot_lerobot lerobot_record.launch.py
    ros2 launch omnibot_lerobot lerobot_record.launch.py include_arm:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory("omnibot_bringup")
    pkg_lerobot = get_package_share_directory("omnibot_lerobot")
    pkg_arm = get_package_share_directory("omnibot_arm")

    include_arm = LaunchConfiguration("include_arm", default="true")

    declare_include_arm = DeclareLaunchArgument(
        "include_arm",
        default_value="true",
        description="Launch SO-101 leader/follower arm stack (leader arm actions)",
    )

    robot_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "robot_with_joy.launch.py")
        ),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "perception.launch.py")
        ),
        launch_arguments={"web_video": "false"}.items(),
    )

    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_arm, "launch", "arm.launch.py")
        ),
        condition=IfCondition(include_arm),
    )

    teleop_record = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lerobot, "launch", "teleop_record.launch.py")
        ),
    )

    return LaunchDescription(
        [
            declare_include_arm,
            robot_joy,
            perception,
            arm,
            teleop_record,
        ]
    )
