# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Apartment pick scene, VR safety path, and the scripted headset.

    ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=88 \
      ros2 launch omnibot_lerobot scripted_vr_pick.launch.py \
      world:=apartment pick_place:=true pick_place_seed:=237 episodes:=3

The headset publishes /arm/joint_commands/teleop_raw, /cmd_vel/teleop_raw,
and /ohho/heartbeat. teleop_safety gates those onto the mux inputs.
cmd_vel_mux and arm_cmd_mux forward them. Gazebo's base command comes from
/cmd_vel/out (cmd_vel_from_mux:=true). The recorder uses action_source:=vr
and operator:=scripted-sim.

This launch refuses to start unless ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID
is a canonical integer from 0 to 232 other than 30. It does not start foxglove.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from omnibot_lerobot.scripted_vr_pick import isolation_failure


def generate_launch_description():
    failure = isolation_failure()
    if failure:
        raise RuntimeError(failure)

    bringup = FindPackageShare("omnibot_bringup")
    sim_launch = [bringup, "/launch/sim.launch.py"]
    output_dir = LaunchConfiguration("output_dir")
    episodes = LaunchConfiguration("episodes")
    seed = LaunchConfiguration("pick_place_seed")

    # North of the table, facing it. The table's north edge is y=1.425,
    # the chassis half-length is 0.1325 m, and the approach moves 0.075 m,
    # so y=1.6575 stays clear of the tabletop. vr publishes the wrist and
    # synthetic BEV cameras the recorder subscribes to.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": "false",
            "drive": "planar",
            "arm": "true",
            "rosbridge": "false",
            "vr": "true",
            "pick_place": LaunchConfiguration("pick_place"),
            "pick_place_seed": seed,
            "cmd_vel_from_mux": "true",
            "spawn_x": "0.20",
            "spawn_y": "1.6575",
            "spawn_yaw": "-1.5707963267948966",
        }.items(),
    )

    safety = Node(
        package="omnibot_hybrid",
        executable="teleop_safety",
        name="teleop_safety",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )
    base_mux = Node(
        package="omnibot_hybrid",
        executable="cmd_vel_mux",
        name="cmd_vel_mux",
        output="screen",
        parameters=[{"default_mode": "teleop", "use_sim_time": False}],
    )
    arm_mux = Node(
        package="omnibot_rl",
        executable="arm_cmd_mux",
        name="arm_cmd_mux",
        output="screen",
        parameters=[{"default_mode": "teleop", "use_sim_time": False}],
    )
    recorder = Node(
        package="omnibot_lerobot",
        executable="teleop_recorder_node",
        name="teleop_recorder_node",
        output="screen",
        parameters=[
            {
                "action_source": "vr",
                "operator": "scripted-sim",
                "robot_kind": "sim",
                "sim_world": LaunchConfiguration("world"),
                "task_name": "scripted-sim pick cube into bin",
                "output_dir": output_dir,
                "repo_id": "local/omnibot-sim-vr-pick",
                "record_hz": 10.0,
                "episode_timeout_s": 30.0,
                "vr_action_prefer": "teleop",
                "use_sim_time": False,
            }
        ],
    )
    headset = Node(
        package="omnibot_lerobot",
        executable="scripted_vr_headset",
        name="scripted_vr_headset",
        output="screen",
        parameters=[
            {
                "episodes": episodes,
                "dataset_dir": output_dir,
                "pick_place_seed": seed,
                "use_sim_time": False,
            }
        ],
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=headset,
            on_exit=[Shutdown(reason="scripted headset finished")],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="apartment"),
            DeclareLaunchArgument(
                "pick_place",
                default_value="true",
                description="Spawn the table, cube, and bin. Passed through to sim.launch.py.",
            ),
            DeclareLaunchArgument("pick_place_seed", default_value="237"),
            DeclareLaunchArgument("episodes", default_value="3"),
            DeclareLaunchArgument(
                "output_dir",
                default_value="/tmp/omnibot-sim-vr-pick",
                description="Recorder dataset directory. Not uploaded.",
            ),
            gazebo,
            safety,
            base_mux,
            arm_mux,
            recorder,
            headset,
            shutdown,
        ]
    )
