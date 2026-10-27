#!/usr/bin/env python3
"""
omnibot_pi.launch.py — Master launch for the Raspberry Pi 5.

Run this ONCE on the Pi to bring up the complete robot stack.
Run omnibot_workstation.launch.py on the GPU PC at the same time.

What this starts (in order):
  ┌─ HARDWARE ──────────────────────────────────────────────────────────────┐
  │  robot_state_publisher   URDF → /robot_description + static TF          │
  │  yahboom_controller_node Yahboom serial → /odom, /imu/data, motors      │
  │  arm_driver_node         SO-101 6-DOF arm via Feetech bus                │
  ├─ PERCEPTION ────────────────────────────────────────────────────────────┤
  │  usb_cam × 5             front/rear/left/right/wrist cameras             │
  │  astra_camera_node       Orbbec Astra Pro — RGB + depth (OpenNI2)        │
  │  depthimage_to_laserscan depth → /scan for SLAM                          │
  │  bev_stitcher_node       4 base cams → /camera/base/bev/image_raw        │
  ├─ STATE ESTIMATION ──────────────────────────────────────────────────────┤
  │  ekf_node                EKF fuses /odom + /imu → /odometry/filtered     │
  ├─ MAPPING ───────────────────────────────────────────────────────────────┤
  │  slam_toolbox            2-D LiDAR SLAM → /map + TF (map→odom)           │
  │  rtabmap                 RGB-D SLAM → /rtabmap/cloud_map                  │
  │  octomap_server          3-D voxel map → /octomap_*                      │
  ├─ NAVIGATION ────────────────────────────────────────────────────────────┤
  │  Nav2 stack              controller, planner, behavior, bt_navigator      │
  │                          velocity_smoother, lifecycle_manager             │
  ├─ CONTROL ROUTING ───────────────────────────────────────────────────────┤
  │  cmd_vel_mux             routes nav2/vla/teleop/rl_nav → motors           │
  │  arm_cmd_mux             routes smolvla/rl_arm → arm driver               │
  │  mission_planner         parses /mission/command → Nav2 + VLA goals       │
  ├─ CONNECTIVITY ──────────────────────────────────────────────────────────┤
  │  ohho-gateway            wss://pi-ip:8443 (token + TLS; default)         │
  │  foxglove / web_video / rosbridge bind 127.0.0.1 behind that gateway     │
  └─────────────────────────────────────────────────────────────────────────┘

Usage:
    ros2 launch omnibot_bringup master/omnibot_pi.launch.py

Skip depth camera (save ~70% CPU):
    ros2 launch omnibot_bringup master/omnibot_pi.launch.py depth_camera:=false

Skip SLAM (cameras + drive only):
    ros2 launch omnibot_bringup master/omnibot_pi.launch.py slam:=false

Skip Nav2 (drive + SLAM, no autonomous navigation):
    ros2 launch omnibot_bringup master/omnibot_pi.launch.py nav2:=false

Headless (no foxglove/web_video):
    ros2 launch omnibot_bringup master/omnibot_pi.launch.py \
        foxglove:=false web_video:=false

Insecure opt-out (prints a warning; rosbridge, foxglove, and video listen
on every interface with no token):
    ros2 launch omnibot_bringup master/omnibot_pi.launch.py gateway:=false

Pair with workstation:
    ros2 launch omnibot_bringup master/omnibot_workstation.launch.py  ← GPU PC
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────────────
    pkg_desc = get_package_share_directory("omnibot_description")
    pkg_nav = get_package_share_directory("omnibot_navigation")
    pkg_bringup = get_package_share_directory("omnibot_bringup")

    xacro_file = os.path.join(pkg_desc, "urdf", "omnibot.urdf.xacro")
    slam_params = os.path.join(pkg_nav, "config", "slam_toolbox_params.yaml")
    rtabmap_params = os.path.join(pkg_nav, "config", "rtabmap_params.yaml")
    octomap_params = os.path.join(pkg_nav, "config", "octomap_params.yaml")
    ekf_params = os.path.join(pkg_nav, "config", "robot_localization.yaml")

    robot_desc = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_depth_camera = DeclareLaunchArgument(
        "depth_camera",
        default_value="true",
        description="Start Orbbec Astra Pro RGB-D driver (~70% CPU)",
    )
    declare_slam = DeclareLaunchArgument(
        "slam",
        default_value="true",
        description="Start SLAM stack (slam_toolbox + rtabmap + octomap)",
    )
    declare_nav2 = DeclareLaunchArgument(
        "nav2",
        default_value="true",
        description="Start Nav2 autonomous navigation stack",
    )
    declare_arm = DeclareLaunchArgument(
        "arm", default_value="true", description="Start SO-101 arm driver"
    )
    declare_calibration_file = DeclareLaunchArgument(
        "calibration_file",
        default_value="",
        description=(
            "SO-101 so101_<id>.yaml. Empty uses OMNIBOT_ARM_CALIBRATION, "
            "then robot_id. See omnibot_arm/CALIBRATION.md."
        ),
    )
    declare_robot_id = DeclareLaunchArgument(
        "robot_id",
        default_value="",
        description=(
            "Load so101_<robot_id>.yaml for the arm. Empty uses "
            "OMNIBOT_ARM_ROBOT_ID."
        ),
    )
    declare_foxglove = DeclareLaunchArgument(
        "foxglove",
        default_value="true",
        description="foxglove on 127.0.0.1:8765. LAN clients use ohho-gateway.",
    )
    declare_web_video = DeclareLaunchArgument(
        "web_video",
        default_value="true",
        description="web_video_server on 127.0.0.1:8080. LAN clients use ohho-gateway.",
    )
    declare_rosbridge = DeclareLaunchArgument(
        "rosbridge",
        default_value="true",
        description="rosbridge on 127.0.0.1:9090. LAN clients use ohho-gateway.",
    )
    # OHH-196: authenticated gateway is the default. gateway:=false is insecure.
    declare_gateway = DeclareLaunchArgument(
        "gateway",
        default_value="true",
        description=(
            "Publish rosbridge, foxglove, and web_video_server through "
            "ohho-gateway (TLS + bearer token) and bind those servers to "
            "127.0.0.1. gateway:=false exposes them on every network interface "
            "with no authentication and prints a warning."
        ),
    )

    # Camera device paths
    declare_cam_front = DeclareLaunchArgument("cam_front", default_value="/dev/video2")
    declare_cam_rear = DeclareLaunchArgument("cam_rear", default_value="/dev/video10")
    declare_cam_left = DeclareLaunchArgument("cam_left", default_value="/dev/video6")
    declare_cam_right = DeclareLaunchArgument("cam_right", default_value="/dev/video4")
    declare_cam_wrist = DeclareLaunchArgument("cam_wrist", default_value="/dev/video0")

    # ── 1. Robot State Publisher ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    # ── 2. Yahboom motor driver ───────────────────────────────────────────────
    # Publishes /odom, /imu/data, /joint_states; subscribes /cmd_vel/out
    yahboom_controller = Node(
        package="omnibot_driver",
        executable="yahboom_controller_node.py",
        name="yahboom_controller_node",
        output="screen",
        parameters=[
            {
                "serial_port": "/dev/ttyUSB0",
                "baud_rate": 115200,
                "wheel_radius": 0.04,
                "wheel_separation_width": 0.215,
                "wheel_separation_length": 0.165,
                # EKF owns odom->base_link TF (double-broadcast causes jitter)
                "publish_tf": False,
                # Android app subscribes /diagnostics — publish driver health
                "publish_diagnostics": True,
            }
        ],
        remappings=[("cmd_vel", "/cmd_vel/out")],
    )

    # -- AI perception: object distance + pose from depth camera ------------
    object_perception = Node(
        package="omnibot_perception",
        executable="object_perception_node",
        name="object_perception_node",
        output="screen",
    )

    # -- Remote rosbag recorder (Android app record button) -----------------
    rosbag_recorder = Node(
        package="omnibot_hybrid",
        executable="rosbag_recorder",
        name="rosbag_recorder",
        output="screen",
    )

    # ── 3. SO-101 Arm driver ──────────────────────────────────────────────────
    arm_driver = Node(
        package="omnibot_arm",
        executable="arm_driver_node.py",
        name="arm_driver_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("arm")),
        parameters=[
            {
                "follower_port": "/dev/ttyACM0",
                "baudrate": 1000000,
                "publish_rate": 100.0,
                "calibration_file": LaunchConfiguration("calibration_file"),
                "robot_id": LaunchConfiguration("robot_id"),
            }
        ],
        remappings=[("joint_commands", "/arm/joint_commands/out")],
    )

    # ── 4. Cameras ────────────────────────────────────────────────────────────
    _cam_params = {
        "framerate": 30.0,
        "image_width": 640,
        "image_height": 480,
        "pixel_format": "mjpeg2rgb",
        "io_method": "mmap",
    }
    cam_front = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_front",
        namespace="camera/front",
        output="screen",
        parameters=[
            {
                **_cam_params,
                "video_device": LaunchConfiguration("cam_front"),
                "camera_name": "front",
                "camera_frame_id": "camera_front_optical_frame",
            }
        ],
    )
    cam_rear = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_rear",
        namespace="camera/rear",
        output="screen",
        parameters=[
            {
                **_cam_params,
                "video_device": LaunchConfiguration("cam_rear"),
                "camera_name": "rear",
                "camera_frame_id": "camera_rear_optical_frame",
            }
        ],
    )
    cam_left = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_left",
        namespace="camera/left",
        output="screen",
        parameters=[
            {
                **_cam_params,
                "video_device": LaunchConfiguration("cam_left"),
                "camera_name": "left",
                "camera_frame_id": "camera_left_optical_frame",
            }
        ],
    )
    cam_right = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_right",
        namespace="camera/right",
        output="screen",
        parameters=[
            {
                **_cam_params,
                "video_device": LaunchConfiguration("cam_right"),
                "camera_name": "right",
                "camera_frame_id": "camera_right_optical_frame",
            }
        ],
    )
    cam_wrist = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_wrist",
        namespace="camera/wrist",
        output="screen",
        parameters=[
            {
                **_cam_params,
                "video_device": LaunchConfiguration("cam_wrist"),
                "camera_name": "wrist",
                "camera_frame_id": "camera_wrist_optical_frame",
            }
        ],
    )

    # ── 5. Orbbec Astra Pro RGB-D ─────────────────────────────────────────────
    astra_camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("astra_camera"),
                "launch",
                "astra_pro.launch.xml",
            )
        ),
        launch_arguments={
            "camera_name": "camera",
            "depth_registration": "true",
            "enable_point_cloud": "true",
            "enable_colored_point_cloud": "false",
            "color_depth_synchronization": "false",
            "oni_log_level": "none",
        }.items(),
        condition=IfCondition(LaunchConfiguration("depth_camera")),
    )

    # ── 6. depth → /scan ─────────────────────────────────────────────────────
    depth_to_scan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        output="screen",
        condition=IfCondition(LaunchConfiguration("depth_camera")),
        parameters=[
            {
                "scan_height": 1,
                "range_min": 0.3,
                "range_max": 8.0,
                "output_frame": "depth_camera_optical_frame",
            }
        ],
        remappings=[
            ("depth", "/camera/depth/image_raw"),
            ("depth_camera_info", "/camera/depth/camera_info"),
            ("scan", "/scan"),
        ],
    )

    # ── 7. BEV stitcher ───────────────────────────────────────────────────────
    bev_stitcher = Node(
        package="omnibot_lerobot",
        executable="bev_stitcher_node.py",
        name="bev_stitcher_node",
        output="screen",
    )

    # ── 8. EKF state estimation ───────────────────────────────────────────────
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params],
        remappings=[("odometry/filtered", "/odometry/filtered")],
    )

    # ── 9. SLAM toolbox (2-D) ─────────────────────────────────────────────────
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(LaunchConfiguration("slam")),
        parameters=[slam_params, {"use_sim_time": False}],
    )

    # ── 10. RTAB-Map (3-D RGB-D SLAM) ────────────────────────────────────────
    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        condition=IfCondition(LaunchConfiguration("slam")),
        parameters=[rtabmap_params, {"use_sim_time": False}],
        remappings=[
            ("rgb/image", "/camera/color/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("depth/image", "/camera/depth/image_raw"),
            ("depth/camera_info", "/camera/depth/camera_info"),
            ("odom", "/odometry/filtered"),
            ("grid_map", "/rtabmap/grid_map"),
        ],
        arguments=["--delete_db_on_start"],
    )

    # ── 11. OctoMap server ────────────────────────────────────────────────────
    octomap_server = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        condition=IfCondition(LaunchConfiguration("slam")),
        parameters=[octomap_params, {"use_sim_time": False}],
        remappings=[("cloud_in", "/rtabmap/cloud_map")],
    )

    # ── 12. Nav2 stack ────────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, "launch", "autonomous_navigation.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("nav2")),
    )

    # ── 13. cmd_vel mux ───────────────────────────────────────────────────────
    # Routes: nav2 | vla | teleop | rl_nav → /cmd_vel/out → yahboom_controller
    cmd_vel_mux = Node(
        package="omnibot_hybrid",
        executable="cmd_vel_mux",
        name="cmd_vel_mux",
        output="screen",
        parameters=[{"default_mode": "nav2"}],
    )

    # ── 14. arm_cmd_mux ───────────────────────────────────────────────────────
    # Routes: smolvla | rl_arm → /arm/joint_commands/out → arm_driver
    arm_cmd_mux = Node(
        package="omnibot_rl",
        executable="arm_cmd_mux",
        name="arm_cmd_mux",
        output="screen",
        condition=IfCondition(LaunchConfiguration("arm")),
    )

    # ── 15. Mission planner ───────────────────────────────────────────────────
    # Parses /mission/command → Nav2 + VLA + RL goals
    mission_planner = Node(
        package="omnibot_hybrid",
        executable="mission_planner",
        name="mission_planner",
        output="screen",
    )

    # ── OHH-196 ohho-gateway (default). Opt out: gateway:=false (insecure). ──
    # Delayed like the previous connectivity nodes so DDS can settle first.
    # The snippet binds rosbridge, foxglove, and web_video_server to 127.0.0.1
    # unless gateway is turned off.
    ohho_gateway = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, "launch", "ohho_gateway.launch.py")
                ),
                launch_arguments={
                    "gateway": LaunchConfiguration("gateway"),
                    "start_rosbridge": LaunchConfiguration("rosbridge"),
                    "start_foxglove": LaunchConfiguration("foxglove"),
                    "start_web_video": LaunchConfiguration("web_video"),
                }.items(),
            )
        ],
    )

    # ── Assemble ──────────────────────────────────────────────────────────────
    return LaunchDescription(
        [
            # Arguments
            declare_depth_camera,
            declare_slam,
            declare_nav2,
            declare_arm,
            declare_calibration_file,
            declare_robot_id,
            declare_foxglove,
            declare_web_video,
            declare_rosbridge,
            declare_gateway,
            declare_cam_front,
            declare_cam_rear,
            declare_cam_left,
            declare_cam_right,
            declare_cam_wrist,
            # Hardware
            robot_state_publisher,
            yahboom_controller,
            arm_driver,
            # Perception
            cam_front,
            cam_rear,
            cam_left,
            cam_right,
            cam_wrist,
            astra_camera,
            depth_to_scan,
            bev_stitcher,
            # State estimation
            ekf,
            object_perception,
            rosbag_recorder,
            # Mapping
            slam_toolbox,
            rtabmap,
            octomap_server,
            # Navigation
            nav2,
            # Control routing
            cmd_vel_mux,
            arm_cmd_mux,
            mission_planner,
            # Connectivity (delayed to let DDS settle). OHH-196 gateway.
            ohho_gateway,
        ]
    )
