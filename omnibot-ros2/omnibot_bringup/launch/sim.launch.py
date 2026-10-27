# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Headless Gazebo Harmonic bringup for OmniBot.

    ros2 launch omnibot_bringup sim.launch.py world:=flat
    ros2 launch omnibot_bringup sim.launch.py world:=apartment
    ros2 launch omnibot_bringup sim.launch.py world:=apartment vr:=true

Spawns omnibot_description (mecanum chassis) with a planar-move drive
(gz-sim VelocityControl + OdometryPublisher on /cmd_vel and /odom), a 2D
lidar, the front RGB camera, and the IMU. ros_gz_bridge bridges:

    /cmd_vel  /odom  /scan  /imu  /camera/front/image_raw
    /camera/front/camera_info  /clock
    /arm/sim/joint_states  /arm/sim/<joint>/cmd_pos

`vr:=true` passes vr_cameras:=true into the xacro and includes
sim_cameras.launch.py. That adds, at 960x540 and 30 Hz:

    /camera/wrist/image_raw       SO-101 gripper (arm_gripper_link)
    /camera/base/bev/image_raw    synthetic top-down camera
    /sim/chase_cam/image_raw      chase camera fixed behind base_link

and their camera_info topics. `video:=true` (the default) starts
web_video_server on port 8080, bound to rosbridge_address. The chase
view is http://<host>:8080/stream?topic=/sim/chase_cam/image_raw.
`vr:=false` (the default, including sim-smoke) does not start that
server and keeps the minimal front camera. This flag does not start
teleop_safety, the muxes, or the Feetech driver.

`rosbridge:=true` (the default) also starts rosbridge_websocket on port 9090
bound to 127.0.0.1 so a browser on this machine can drive this stack.
`rosbridge_address:=0.0.0.0` exposes it on the network without authentication.
The sim-smoke test passes `rosbridge:=false` and does not start rosbridge.
`arm:=true` (the default) starts arm_sim_driver, which consumes
/arm/joint_commands/out, applies arm_safety, and publishes /arm/joint_states.
This launch does not start foxglove, arm_cmd_mux, or the Feetech arm driver.

`bridge_config` (default `sim_bridge.yaml`) is the ros_gz_bridge file in
config/. `sim_teleop.launch.py` passes `sim_bridge_vr.yaml` so Gazebo
follows `/cmd_vel/out`. The default file still maps `/cmd_vel`, which
sim-smoke drives directly.

`drive:=mecanum` selects the gz-sim MecanumDrive plugin instead of planar move.
`gui:=true` opens the Gazebo client; the default is server-only so CI has no display.

`pick_place:=false` (the default) does not add a table, cube, or bin. The
apartment SDF is unchanged. `pick_place:=true` spawns those three models
after Gazebo is up. The cube and bin start only when the table process
exits 0. Any other table exit shuts this launch down and does not spawn
them. Cube and bin poses come from `pick_place_seed`.
`cmd_vel_from_mux:=false` (the default) still bridges ROS `/cmd_vel` into
Gazebo. `cmd_vel_from_mux:=true` bridges `/cmd_vel/out` instead, so the
base moves only through the velocity multiplexer.

GPU note (not measured): this workspace has no Gazebo install, so there
is no GPU delta on disk. `gui:=false` (the default) passes
`-s --headless-rendering`, which renders the lidar and cameras without
a window (Gazebo Harmonic / gz-sim 8). `gui:=true` drops both flags and
opens the client, which adds the user camera on top of the sensors.
`vr:=true` raises the front camera from the minimal 160x120 at 5 Hz to
960x540 at 30 Hz and adds the wrist, BEV, and chase cameras at that
size. Those sensors are always_on, so the render cost is paid even when
no browser is connected. web_video_server encodes a topic only while a
client holds it. To compare later, run `world:=apartment` with
`gui:=false` (the headless flag stays on) and with `gui:=true`, each
with `vr:=false` and `vr:=true`, and read GPU use while the chase
stream is idle and while a browser holds the URL above.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _enabled(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def _spawn_module():
    scripts = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    import pick_place_spawn

    return pick_place_spawn


def _load_camera_support():
    import importlib.util

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_cameras_support.py")
    spec = importlib.util.spec_from_file_location("omnibot_sim_cameras_support", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_cameras = _load_camera_support()


def _world_path(bringup_share, world):
    known = {
        "flat": os.path.join(bringup_share, "worlds", "flat.sdf"),
        "apartment": os.path.join(bringup_share, "worlds", "apartment.sdf"),
    }
    if world in known:
        return known[world]
    if os.path.isfile(world):
        return world
    candidate = os.path.join(bringup_share, "worlds", world)
    if os.path.isfile(candidate):
        return candidate
    if not world.endswith(".sdf"):
        with_suffix = candidate + ".sdf"
        if os.path.isfile(with_suffix):
            return with_suffix
    raise FileNotFoundError(
        f"world {world!r} is not flat, apartment, or an existing SDF path"
    )


def _launch_sim(context, *args, **kwargs):
    del args, kwargs
    world_key = LaunchConfiguration("world").perform(context)
    drive = LaunchConfiguration("drive").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes")
    vr_on = _cameras.vr_enabled(LaunchConfiguration("vr").perform(context))

    bringup = FindPackageShare("omnibot_bringup").perform(context)
    description = FindPackageShare("omnibot_description").perform(context)
    ros_gz_sim = FindPackageShare("ros_gz_sim").perform(context)

    world_path = _world_path(bringup, world_key)
    xacro_file = os.path.join(description, "urdf", "omnibot.urdf.xacro")
    from_mux = _enabled(LaunchConfiguration("cmd_vel_from_mux").perform(context))
    bridge_name = "sim_bridge_mux.yaml" if from_mux else "sim_bridge.yaml"
    if not from_mux:
        # sim_teleop.launch.py selects sim_bridge_vr.yaml through bridge_config.
        bridge_name = LaunchConfiguration("bridge_config").perform(context)
        if (
            os.path.basename(bridge_name) != bridge_name
            or not bridge_name.endswith(".yaml")
        ):
            raise RuntimeError(
                "bridge_config must be a YAML file name in the bringup config directory"
            )
    bridge_config = os.path.join(bringup, "config", bridge_name)

    robot_desc = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                f" drive:={drive}",
                " lidar:=true",
                " minimal_sensors:=true",
                f" vr_cameras:={_cameras.xacro_flag(vr_on)}",
            ]
        ),
        value_type=str,
    )

    # -s is server-only. --headless-rendering lets the lidar and cameras
    # render without a window (Gazebo Harmonic / gz-sim 8). vr:=true does
    # not drop this flag. The GPU cost of the extra cameras was not measured.
    if gui:
        gz_args = f"-r {world_path}"
    else:
        gz_args = f"-r -s --headless-rendering {world_path}"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )

    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)

    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-name",
                    "omnibot",
                    "-topic",
                    "robot_description",
                    "-x",
                    spawn_x,
                    "-y",
                    spawn_y,
                    "-z",
                    "0.05",
                    "-Y",
                    spawn_yaw,
                ],
            )
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config, "use_sim_time": False}],
    )

    actions = [gazebo, robot_state_publisher, spawn, bridge]
    descriptor_on = LaunchConfiguration("descriptor").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    if descriptor_on:
        actions.append(
            Node(
                package="omnibot_bringup",
                executable="descriptor_node.py",
                name="ohho_descriptor",
                output="screen",
                parameters=[
                    {
                        "descriptor_yaml": os.path.join(
                            bringup, "config", "robot_descriptor.yaml"
                        ),
                        "kind": "sim",
                    }
                ],
            )
        )
    arm_on = LaunchConfiguration("arm").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    if arm_on:
        arm_share = FindPackageShare("omnibot_arm").perform(context)
        arm_params = os.path.join(arm_share, "config", "arm_params.yaml")
        actions.append(
            Node(
                package="omnibot_arm",
                executable="arm_sim_driver_node.py",
                name="arm_sim_driver_node",
                output="screen",
                parameters=[
                    {
                        "arm_params_file": arm_params,
                        "use_sim_time": False,
                    }
                ],
            )
        )
    rosbridge_on = LaunchConfiguration("rosbridge").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    if rosbridge_on:
        address = LaunchConfiguration("rosbridge_address").perform(context)
        actions.append(
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                parameters=[{"port": 9090, "address": address}],
            )
        )
    # Default off. The apartment SDF is not modified.
    if _enabled(LaunchConfiguration("pick_place").perform(context)):
        spawn_api = _spawn_module()
        seed = spawn_api.parse_seed(LaunchConfiguration("pick_place_seed").perform(context))
        world_name = spawn_api.world_name_for(world_key, world_path)
        models_dir = os.path.join(bringup, "models")
        plan = spawn_api.spawn_plan(True, seed, models_dir, world_name)
        spawners = [
            Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{item['name']}",
                output="screen",
                arguments=item["arguments"],
            )
            for item in plan
        ]
        # plan order is table, cube, bin. Objects wait for a successful table.
        table_node, *objects = spawners

        def _on_table_exit(event, _context):
            # ROS launch emits this for a zero exit and for a failed create.
            if event.returncode == 0:
                return objects
            return [
                LogInfo(
                    msg=(
                        "pick-and-place table spawn failed with "
                        f"returncode {event.returncode}; stopping the run"
                    )
                ),
                Shutdown(reason="pick-and-place table spawn failed"),
            ]

        actions.append(
            TimerAction(
                period=6.0,
                actions=[
                    table_node,
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=table_node,
                            on_exit=_on_table_exit,
                        )
                    ),
                ],
            )
        )
    if vr_on:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup, "launch", "sim_cameras.launch.py")
                ),
                launch_arguments={
                    "video": LaunchConfiguration("video").perform(context),
                    "address": LaunchConfiguration("rosbridge_address").perform(context),
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="flat",
                description="flat, apartment, or a path to an SDF world",
            ),
            DeclareLaunchArgument(
                "drive",
                default_value="planar",
                description="planar (velocity control) or mecanum (MecanumDrive)",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Open the Gazebo GUI. Default is headless.",
            ),
            DeclareLaunchArgument(
                "arm",
                default_value="true",
                description=(
                    "Start arm_sim_driver. It reads arm_params.yaml and "
                    "commands the SO-101 joints in Gazebo. "
                    "Set false for a base-only launch."
                ),
            ),
            DeclareLaunchArgument(
                "rosbridge",
                default_value="true",
                description=(
                    "Start rosbridge_websocket on port 9090, bound to "
                    "rosbridge_address (127.0.0.1 by default). "
                    "The sim-smoke test sets this false."
                ),
            ),
            DeclareLaunchArgument(
                "pick_place",
                default_value="false",
                description=(
                    "Spawn a table, rigid cube, and bin. Default false leaves "
                    "the world SDF unchanged. Cube and bin poses use pick_place_seed."
                ),
            ),
            DeclareLaunchArgument(
                "pick_place_seed",
                default_value="0",
                description="Integer seed for the cube and bin poses. Ignored unless pick_place is true.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_from_mux",
                default_value="false",
                description=(
                    "Bridge Gazebo /cmd_vel from ROS /cmd_vel/out. "
                    "Default false keeps the direct /cmd_vel bridge."
                ),
            ),
            DeclareLaunchArgument(
                "descriptor",
                default_value="true",
                description="Publish the latched RobotDescriptor JSON on /ohho/descriptor",
            ),
            DeclareLaunchArgument(
                "rosbridge_address",
                default_value="127.0.0.1",
                description=(
                    "Interface rosbridge_websocket binds. "
                    "127.0.0.1 is loopback only. "
                    "0.0.0.0 exposes it on the network without authentication. "
                    "vr:=true also binds web_video_server to this address."
                ),
            ),
            DeclareLaunchArgument(
                "vr",
                default_value="false",
                description=(
                    "Enable the Quest cameras (wrist, synthetic BEV, chase) "
                    "at 960x540 and 30 Hz, bridge them, and include "
                    "sim_cameras.launch.py. Default false keeps the minimal "
                    "front camera used by sim-smoke."
                ),
            ),
            DeclareLaunchArgument(
                "video",
                default_value="true",
                description=(
                    "When vr:=true, start web_video_server on port 8080. "
                    "Ignored when vr:=false. "
                    "Chase stream: /stream?topic=/sim/chase_cam/image_raw."
                ),
            ),
            DeclareLaunchArgument(
                "bridge_config",
                default_value="sim_bridge.yaml",
                description=(
                    "ros_gz_bridge YAML name in the bringup config directory. "
                    "Default sim_bridge.yaml maps /cmd_vel. "
                    "sim_bridge_vr.yaml maps /cmd_vel/out for sim_teleop.launch.py."
                ),
            ),
            DeclareLaunchArgument("spawn_x", default_value="0"),
            DeclareLaunchArgument("spawn_y", default_value="0"),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="0",
                description="Spawn heading about +Z, radians",
            ),
            OpaqueFunction(function=_launch_sim),
        ]
    )
