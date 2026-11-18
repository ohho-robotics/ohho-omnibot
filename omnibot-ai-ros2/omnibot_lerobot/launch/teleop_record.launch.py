import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_omnibot_lerobot = get_package_share_directory("omnibot_lerobot")
    policy_params = os.path.join(pkg_omnibot_lerobot, "config", "policy_params.yaml")

    teleop_recorder_node = Node(
        package="omnibot_lerobot",
        executable="teleop_recorder_node",
        name="teleop_recorder_node",
        output="screen",
        parameters=[
            policy_params,
            {
                "action_source": LaunchConfiguration("action_source"),
                "operator": LaunchConfiguration("operator"),
                "robot_kind": LaunchConfiguration("robot_kind"),
                "sim_world": LaunchConfiguration("sim_world"),
                "task_name": LaunchConfiguration("task_name"),
                "vr_action_prefer": LaunchConfiguration("vr_action_prefer"),
                "vr_arm_command_topic": LaunchConfiguration("vr_arm_command_topic"),
                "vr_arm_mux_topic": LaunchConfiguration("vr_arm_mux_topic"),
                "vr_base_command_topic": LaunchConfiguration("vr_base_command_topic"),
                "vr_base_mux_topic": LaunchConfiguration("vr_base_mux_topic"),
                "control_mode_topic": LaunchConfiguration("control_mode_topic"),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "action_source",
                default_value="leader",
                description="leader (joystick + leader arm) or vr (teleop topics)",
            ),
            DeclareLaunchArgument(
                "operator",
                default_value="",
                description="Episode operator. Empty becomes vr when action_source is vr.",
            ),
            DeclareLaunchArgument(
                "robot_kind",
                default_value="sim",
                description="sim or real",
            ),
            DeclareLaunchArgument(
                "sim_world",
                default_value="",
                description="Simulator world name stored on the episode. Empty on a real robot.",
            ),
            DeclareLaunchArgument(
                "task_name",
                default_value="mobile manipulation task",
                description="Task string stored with the episode",
            ),
            DeclareLaunchArgument(
                "vr_action_prefer",
                default_value="teleop",
                description="teleop uses /arm/joint_commands/teleop first; mux uses the mux output in teleop mode",
            ),
            DeclareLaunchArgument(
                "vr_arm_command_topic",
                default_value="/arm/joint_commands/teleop",
                description="OHH-197 teleop joint target. Point this at the post-safety stream when one exists.",
            ),
            DeclareLaunchArgument(
                "vr_arm_mux_topic",
                default_value="/arm/joint_commands/out",
                description="Mux output, recorded only while /control_mode is teleop",
            ),
            DeclareLaunchArgument(
                "vr_base_command_topic",
                default_value="/cmd_vel/teleop",
                description="Teleop base twist",
            ),
            DeclareLaunchArgument(
                "vr_base_mux_topic",
                default_value="/cmd_vel/out",
                description="Base mux output, recorded only while /control_mode is teleop",
            ),
            DeclareLaunchArgument(
                "control_mode_topic",
                default_value="/control_mode",
                description="Mux mode. The recorder only reads it.",
            ),
            teleop_recorder_node,
        ]
    )
