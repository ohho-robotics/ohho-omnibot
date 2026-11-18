import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from omnibot_hybrid.optional_package import PackageAvailable


def _legacy_base(ekf_on):
    """Yahboom node only when ros2_control is off, so the port is opened once."""
    ekf_test = "in" if ekf_on else "not in"
    return IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("ros2_control"),
                "'.lower() not in ('1', 'true', 'yes') and '",
                LaunchConfiguration("ekf"),
                f"'.lower() {ekf_test} ('1', 'true', 'yes')",
            ]
        )
    )


def generate_launch_description():
    pkg_desc = get_package_share_directory("omnibot_description")
    pkg_bringup = get_package_share_directory("omnibot_bringup")
    pkg_arm = get_package_share_directory("omnibot_arm")

    xacro_file = os.path.join(pkg_desc, "urdf", "omnibot.urdf.xacro")
    controllers_yaml = os.path.join(pkg_bringup, "config", "controllers.yaml")
    arm_params = os.path.join(pkg_arm, "config", "arm_params.yaml")
    robot_description_config = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " use_sim:=false",
                " use_mock:=false",
                " calibration_file:=",
                LaunchConfiguration("calibration_file"),
            ]
        ),
        value_type=str,
    )

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_use_rosbridge = DeclareLaunchArgument(
        "use_rosbridge",
        default_value="true",
        description="Start ROSBridge WebSocket server on port 9090 for Android app",
    )
    declare_use_foxglove = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description="Start Foxglove bridge on port 8765 for browser-based live monitoring",
    )
    declare_ekf = DeclareLaunchArgument(
        "ekf",
        default_value="true",
        description="Run robot_localization EKF (fuses /odom + /imu/data → /odometry/filtered)",
    )
    declare_use_metrics = DeclareLaunchArgument(
        "use_metrics",
        default_value="false",
        description=(
            "Start the Prometheus metrics bridge (port 8888) from omnibot_metrics. "
            "Default false. That package is not in this repository; if it is "
            "missing, launch logs one warning and skips the node."
        ),
    )
    declare_metrics_port = DeclareLaunchArgument(
        "metrics_port",
        default_value="8888",
        description="HTTP port the Prometheus metrics bridge listens on",
    )
    declare_use_ota = DeclareLaunchArgument(
        "use_ota",
        default_value="false",
        description=(
            "Start the OTA update agent from omnibot_ota. Default false. "
            "That package is not in this repository; if it is missing, launch "
            "logs one warning and skips the node. Also set ota_manifest_url."
        ),
    )
    declare_ota_manifest_url = DeclareLaunchArgument(
        "ota_manifest_url",
        default_value="",
        description="URL to OTA manifest.json — overrides deployment.env OTA_MANIFEST_URL",
    )
    declare_descriptor = DeclareLaunchArgument(
        "descriptor",
        default_value="true",
        description="Publish the latched RobotDescriptor JSON on /ohho/descriptor",
    )
    declare_ros2_control = DeclareLaunchArgument(
        "ros2_control",
        default_value="true",
        description=(
            "Start ros2_control_node, the shared controllers.yaml spawners, "
            "and the command adapter. The Yahboom serial node stays off so "
            "it does not open the base port. Set false for yahboom_controller_node."
        ),
    )
    declare_calibration_file = DeclareLaunchArgument(
        "calibration_file",
        default_value="",
        description=(
            "OHH-94 SO-101 home-tick yaml passed to the hardware plugin. "
            "Empty keeps the factory 2048 ticks."
        ),
    )

    use_rosbridge = LaunchConfiguration("use_rosbridge")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_ekf = LaunchConfiguration("ekf")
    use_metrics = LaunchConfiguration("use_metrics")
    metrics_port = LaunchConfiguration("metrics_port")
    use_ota = LaunchConfiguration("use_ota")
    ota_manifest_url = LaunchConfiguration("ota_manifest_url")
    use_descriptor = LaunchConfiguration("descriptor")

    # ── Yahboom driver ────────────────────────────────────────────────────────
    # publish_tf is disabled when EKF is running — the EKF owns odom→base_link.
    # When ekf:=false the driver broadcasts the TF itself (legacy behaviour).
    driver_with_ekf = Node(
        package="omnibot_driver",
        executable="yahboom_controller_node.py",
        name="yahboom_driver",
        output="screen",
        condition=_legacy_base(True),
        parameters=[
            {
                "serial_port": "/dev/ttyUSB0",
                "baud_rate": 115200,
                "wheel_separation_length": 0.165,
                "wheel_separation_width": 0.215,
                "wheel_radius": 0.04,
                "publish_tf": False,  # EKF owns odom→base_link TF
                "publish_diagnostics": True,  # Android app /diagnostics panel
            }
        ],
    )

    driver_without_ekf = Node(
        package="omnibot_driver",
        executable="yahboom_controller_node.py",
        name="yahboom_driver",
        output="screen",
        condition=_legacy_base(False),
        parameters=[
            {
                "serial_port": "/dev/ttyUSB0",
                "baud_rate": 115200,
                "wheel_separation_length": 0.165,
                "wheel_separation_width": 0.215,
                "wheel_radius": 0.04,
                "publish_tf": True,  # driver broadcasts TF when EKF is off
                "publish_diagnostics": True,  # Android app /diagnostics panel
            }
        ],
    )

    use_ros2_control = LaunchConfiguration("ros2_control")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        condition=IfCondition(use_ros2_control),
        # Jazzy controller_manager ignores a robot_description parameter and
        # subscribes to /robot_description. robot_state_publisher publishes it.
        parameters=[
            controllers_yaml,
            {"use_sim_time": False},
        ],
    )
    ros2_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        condition=IfCondition(use_ros2_control),
        arguments=[
            "joint_state_broadcaster",
            "mecanum_drive_controller",
            "arm_joint_trajectory_controller",
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--param-file",
            controllers_yaml,
        ],
    )
    ros2_control_adapter = Node(
        package="omnibot_bringup",
        executable="ros2_control_adapter.py",
        name="ros2_control_adapter",
        output="screen",
        condition=IfCondition(use_ros2_control),
        parameters=[
            {
                "command_source": "joint_commands",
                "cmd_vel_topic": "/cmd_vel/out",
                "relay_odom": True,
                "publish_odom_tf": ParameterValue(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("ekf"),
                            "'.lower() not in ('1', 'true', 'yes')",
                        ]
                    ),
                    value_type=bool,
                ),
                "publish_arm_feedback_topic": "",
                "publish_arm_joint_states": True,
                "arm_params_file": arm_params,
                "use_sim_time": False,
            }
        ],
    )

    # ── Robot state publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_config}],
    )

    # ── EKF — state estimation ────────────────────────────────────────────────
    # Fuses /odom (wheel dead-reckoning) + /imu/data (Yahboom IMU)
    # → /odometry/filtered + odom→base_link TF
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "state_estimation.launch.py")
        ),
        condition=IfCondition(use_ekf),
    )

    # ── ROSBridge — Android app (ws://<pi-ip>:9090) ───────────────────────────
    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{"port": 9090}],
        condition=IfCondition(use_rosbridge),
    )

    # ── Foxglove bridge — browser-based live digital twin (ws://<pi-ip>:8765) ─
    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{"port": 8765, "address": "0.0.0.0"}],
        condition=IfCondition(use_foxglove),
    )

    # Prometheus metrics bridge. omnibot_metrics is not in this repository.
    # Exposes /metrics on <metrics_port> when use_metrics:=true and the package
    # is installed. A missing package logs one warning and skips the node.
    # Enable with: ros2 launch omnibot_bringup robot.launch.py use_metrics:=true
    metrics_bridge_node = Node(
        package="omnibot_metrics",
        executable="metrics_bridge",
        name="ros2_prometheus_bridge",
        output="screen",
        parameters=[
            {
                "metrics_port": metrics_port,
                "machine_label": "raspberry_pi",
            }
        ],
        condition=PackageAvailable("omnibot_metrics", use_metrics),
    )

    descriptor_node = Node(
        package="omnibot_bringup",
        executable="descriptor_node.py",
        name="ohho_descriptor",
        output="screen",
        condition=IfCondition(use_descriptor),
        parameters=[
            {
                "descriptor_yaml": os.path.join(
                    pkg_bringup, "config", "robot_descriptor.yaml"
                ),
                "kind": "real",
            }
        ],
    )

    # OTA update agent. omnibot_ota is not in this repository.
    # Polls manifest.json for updates; Android app calls /ota/* services.
    # Enable with: ros2 launch omnibot_bringup robot.launch.py use_ota:=true ota_manifest_url:=<URL>
    ota_agent_node = Node(
        package="omnibot_ota",
        executable="ota_agent",
        name="ota_agent_node",
        output="screen",
        parameters=[{"manifest_url": ota_manifest_url}],
        condition=PackageAvailable("omnibot_ota", use_ota),
    )

    return LaunchDescription(
        [
            declare_use_rosbridge,
            declare_use_foxglove,
            declare_ekf,
            declare_use_metrics,
            declare_metrics_port,
            declare_use_ota,
            declare_ota_manifest_url,
            declare_descriptor,
            declare_ros2_control,
            declare_calibration_file,
            driver_with_ekf,
            driver_without_ekf,
            ros2_control_node,
            ros2_control_spawner,
            ros2_control_adapter,
            robot_state_publisher,
            ekf,
            rosbridge_node,
            foxglove_node,
            metrics_bridge_node,
            ota_agent_node,
            descriptor_node,
        ]
    )
