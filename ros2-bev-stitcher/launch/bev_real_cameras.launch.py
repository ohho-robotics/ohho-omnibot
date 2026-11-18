"""
Launch OhhO View on the physical robot's four base USB cameras.

Expects ``usb_cam`` (or equivalent) publishing::

  /camera/front/image_raw  … /camera/right/image_raw
  /camera/{name}/camera_info

and ``robot_state_publisher`` providing camera optical frames in TF.
Start the camera drivers separately, e.g.::

  ros2 launch omnibot_bringup perception.launch.py bev:=false

then::

  ros2 launch ros2_bev_stitcher bev_real_cameras.launch.py

Record a demo GIF from ``/camera/base/bev/image_raw`` (see docs/CALIBRATION.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ros2_bev_stitcher")
    default_params = os.path.join(pkg_share, "config", "bev_params_real.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS 2 params file (bev_params_real.yaml by default)",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=os.path.expanduser("~/bev_calibration.npz"),
                description="Homography .npz from calibrate_bev_ipm or bev_cal_ui",
            ),
            DeclareLaunchArgument(
                "calibration_mode",
                default_value="auto",
                description="checkerboard | ipm | auto",
            ),
            Node(
                package="ros2_bev_stitcher",
                executable="bev_stitcher",
                name="bev_stitcher",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "calibration_file": LaunchConfiguration("calibration_file"),
                        "calibration_mode": LaunchConfiguration("calibration_mode"),
                    },
                ],
            ),
        ]
    )
