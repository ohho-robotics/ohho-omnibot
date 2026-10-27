"""Launch the SO-101 arm driver and load per-robot home ticks.

calibration_file:= or robot_id:= selects a yaml file. With neither set, the
home_ticks already in arm_params.yaml are kept. Those factory values are
[2048, 2048, 2048, 2048, 2048, 2048] and are not this robot; a warning is
printed in that case. See CALIBRATION.md.

This file does not open a motor bus. The driver node connects only when
lerobot is installed.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_servo_offsets():
    """Import servo_offsets from the source tree or the installed lib dir."""
    share_launch = os.path.dirname(os.path.realpath(__file__))
    candidates = [
        os.path.abspath(os.path.join(share_launch, "..", "scripts")),
        os.path.abspath(
            os.path.join(share_launch, "..", "..", "..", "lib", "omnibot_arm")
        ),
    ]
    for directory in candidates:
        if os.path.isfile(os.path.join(directory, "servo_offsets.py")):
            if directory not in sys.path:
                sys.path.insert(0, directory)
            break
    else:
        raise ImportError(
            "servo_offsets.py was not found next to the arm package scripts"
        )
    import servo_offsets

    return servo_offsets


def _arm_driver(context, *_args, **_kwargs):
    servo_offsets = _load_servo_offsets()
    pkg = get_package_share_directory("omnibot_arm")
    arm_params = os.path.join(pkg, "config", "arm_params.yaml")
    config_dir = os.path.join(pkg, "config")
    calibration_file = LaunchConfiguration("calibration_file").perform(context)
    robot_id = LaunchConfiguration("robot_id").perform(context)
    fallback = servo_offsets.home_ticks_from_arm_params(arm_params)
    resolution = servo_offsets.resolve_home_ticks(
        calibration_file=calibration_file,
        robot_id=robot_id,
        config_dir=config_dir,
        fallback=fallback,
    )
    if resolution.warning:
        print(resolution.warning, file=sys.stderr, flush=True)
    overrides = servo_offsets.launch_parameter_overrides(resolution, robot_id)
    node = Node(
        package="omnibot_arm",
        executable="arm_driver_node.py",
        name="arm_driver_node",
        output="screen",
        parameters=[arm_params, overrides],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration_file",
                default_value="",
                description=(
                    "Path to so101_<id>.yaml. Overrides OMNIBOT_ARM_CALIBRATION "
                    "and robot_id. Empty uses the env var, then robot_id, then "
                    "arm_params.yaml home_ticks."
                ),
            ),
            DeclareLaunchArgument(
                "robot_id",
                default_value="",
                description=(
                    "Load so101_<robot_id>.yaml from config/calibration "
                    "or from OMNIBOT_ARM_CALIBRATION_DIR. Empty uses "
                    "OMNIBOT_ARM_ROBOT_ID."
                ),
            ),
            OpaqueFunction(function=_arm_driver),
        ]
    )
