# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""SO-101 arm-only simulation.

Starts the existing omnibot_arm driver with simulate:=true so the Feetech
bus stays closed, and latches the arm-only RobotDescriptor on /ohho/descriptor.

There is no mobile base in this launch: no chassis driver and no velocity
command topic. Joint commands are remapped onto the topic named by the
descriptor. Joint clamps, command timeout, and e-stop stay in the arm driver.

    ros2 launch omnibot_bringup so101_arm_sim.launch.py
    ros2 launch omnibot_bringup so101_arm_sim.launch.py rosbridge:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_params = PathJoinSubstitution(
        [FindPackageShare("omnibot_arm"), "config", "arm_params.yaml"]
    )
    descriptor_yaml = PathJoinSubstitution(
        [
            FindPackageShare("omnibot_bringup"),
            "config",
            "so101_arm_only_descriptor.yaml",
        ]
    )
    use_rosbridge = LaunchConfiguration("rosbridge")
    rosbridge_address = LaunchConfiguration("rosbridge_address")

    arm_driver = Node(
        package="omnibot_arm",
        executable="arm_driver_node.py",
        name="arm_driver_node",
        output="screen",
        parameters=[arm_params, {"simulate": True}],
        remappings=[("/arm/joint_commands/out", "/arm/joint_commands")],
    )

    descriptor = Node(
        package="omnibot_bringup",
        executable="descriptor_node.py",
        name="ohho_descriptor",
        output="screen",
        parameters=[
            {
                "descriptor_yaml": descriptor_yaml,
                "kind": "sim",
            }
        ],
    )

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        condition=IfCondition(use_rosbridge),
        parameters=[{"port": 9090, "address": rosbridge_address}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rosbridge",
                default_value="true",
                description=(
                    "Start rosbridge_websocket on port 9090, bound to "
                    "rosbridge_address (127.0.0.1 by default)."
                ),
            ),
            DeclareLaunchArgument(
                "rosbridge_address",
                default_value="127.0.0.1",
                description=(
                    "Interface rosbridge_websocket binds. "
                    "127.0.0.1 is loopback only."
                ),
            ),
            arm_driver,
            descriptor,
            rosbridge,
        ]
    )
