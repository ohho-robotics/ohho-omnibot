# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""ros2_control URDF and adapter checks that do not need a ROS graph.

Runs xacro when the binary is on PATH. Otherwise the xacro source is parsed
for the ros2_control block, the macro arguments, and the absence of
transmission tags.
"""

import importlib.util
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
URDF = REPO / "omnibot-ros2" / "omnibot_description" / "urdf"
MAIN = URDF / "omnibot.urdf.xacro"
CONTROL = URDF / "omnibot.ros2_control.xacro"
CONTROLLERS = REPO / "omnibot-ros2" / "omnibot_bringup" / "config" / "controllers.yaml"
SIM_LAUNCH = REPO / "omnibot-ros2" / "omnibot_bringup" / "launch" / "sim.launch.py"
ROBOT_LAUNCH = REPO / "omnibot-ros2" / "omnibot_bringup" / "launch" / "robot.launch.py"
LOGIC = REPO / "omnibot-ros2" / "omnibot_bringup" / "scripts" / "ros2_control_adapter_logic.py"

PLUGINS = (
    "omnibot_hardware/OmniBotSystem",
    "gz_ros2_control/GazeboSimSystem",
    "mock_components/GenericSystem",
    "omnibot_bringup/PlanarWheelMock",
)
CONTROLLER_TYPES = (
    "joint_state_broadcaster/JointStateBroadcaster",
    "mecanum_drive_controller/MecanumDriveController",
    "joint_trajectory_controller/JointTrajectoryController",
    "parallel_gripper_action_controller/GripperActionController",
)


def _logic():
    spec = importlib.util.spec_from_file_location("ros2_control_adapter_logic", LOGIC)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _xacro_text():
    return MAIN.read_text(encoding="utf-8") + "\n" + CONTROL.read_text(encoding="utf-8")


def test_source_has_ros2_control_and_no_transmission():
    text = _xacro_text()
    assert "<transmission" not in text
    assert text.count('<dynamics damping="4.0" friction="0.5"/>') == 6
    assert text.count("<turnGravityOff>true</turnGravityOff>") == 7
    assert "<gravity>" not in text
    for link in (
        "arm_base_link",
        "arm_shoulder_link",
        "arm_upper_arm_link",
        "arm_lower_arm_link",
        "arm_wrist_link",
        "arm_gripper_link",
        "arm_moving_jaw_link",
    ):
        assert f'reference="{link}"' in text
    assert 'name="use_sim"' in text
    assert 'name="use_mock"' in text
    assert 'name="controllers_file"' in text
    assert 'name="calibration_file"' in text
    assert "gz-sim-joint-position-controller-system" not in text
    for plugin in PLUGINS:
        assert plugin in text
    for joint in (
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
        "arm_shoulder_pan",
        "arm_shoulder_lift",
        "arm_elbow_flex",
        "arm_wrist_flex",
        "arm_wrist_roll",
        "arm_gripper",
    ):
        assert f'name="{joint}"' in text


def test_controllers_yaml_is_shared_by_both_launches():
    yaml_text = CONTROLLERS.read_text(encoding="utf-8")
    for plugin in CONTROLLER_TYPES:
        assert plugin in yaml_text
    assert "tf_frame_prefix_enable: false" in yaml_text
    assert "reference_timeout: 0.5" in yaml_text
    assert "base_frame_id: base_footprint" in yaml_text
    for launch in (SIM_LAUNCH, ROBOT_LAUNCH):
        text = launch.read_text(encoding="utf-8")
        assert 'package="controller_manager"' in text
        assert "controllers.yaml" in text
    sim = SIM_LAUNCH.read_text(encoding="utf-8")
    robot = ROBOT_LAUNCH.read_text(encoding="utf-8")
    assert "use_sim:=true" in sim
    assert "use_mock:=false" in sim
    assert 'executable="ros2_control_node"' in robot
    assert "use_sim:=false" in robot
    assert "use_mock:=false" in robot


def test_xacro_expands_sim_robot_and_mock_when_available():
    binary = shutil.which("xacro")
    if binary is None:
        pytest.skip("xacro is not on PATH; source assertions already ran")
    cases = (
        ["use_sim:=true", "use_mock:=false", "drive:=planar", "controllers_file:=/tmp/controllers.yaml"],
        ["use_sim:=true", "use_mock:=false", "drive:=mecanum", "controllers_file:=/tmp/controllers.yaml"],
        ["use_sim:=false", "use_mock:=false"],
        ["use_sim:=false", "use_mock:=true"],
    )
    for args in cases:
        completed = subprocess.run(
            [binary, str(MAIN), *args],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
        )
        assert completed.returncode == 0, completed.stderr
        assert "<ros2_control" in completed.stdout
        assert "<transmission" not in completed.stdout
        assert "gz-sim-joint-position-controller-system" not in completed.stdout
    def expand(extra):
        completed = subprocess.run(
            [binary, str(MAIN), *extra],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
        )
        assert completed.returncode == 0, completed.stderr
        return completed.stdout

    planar = expand(
        ["use_sim:=true", "use_mock:=false", "drive:=planar",
         "controllers_file:=/tmp/controllers.yaml"]
    )
    assert 'name="OmniBotArmSim"' in planar
    assert 'name="OmniBotWheelMock"' in planar
    assert 'name="OmniBotGazebo"' not in planar
    assert "<plugin>omnibot_bringup/PlanarWheelMock</plugin>" in planar
    assert "<plugin>mock_components/GenericSystem</plugin>" not in planar
    assert "gz-sim-velocity-control-system" in planar
    mecanum = expand(
        ["use_sim:=true", "use_mock:=false", "drive:=mecanum",
         "controllers_file:=/tmp/controllers.yaml"]
    )
    assert 'name="OmniBotGazebo"' in mecanum
    assert 'name="OmniBotWheelMock"' not in mecanum
    assert "gz-sim-velocity-control-system" not in mecanum
    assert "gz-sim-mecanum-drive-system" not in mecanum
    robot = expand(["use_sim:=false", "use_mock:=false"])
    assert 'name="OmniBotSystem"' in robot
    assert "omnibot_hardware/OmniBotSystem" in robot
    assert "gz_ros2_control-system" not in robot
    assert 'name="OmniBotMock"' not in robot
    mock = expand(["use_sim:=false", "use_mock:=true"])
    assert 'name="OmniBotMock"' in mock
    assert "omnibot_hardware/OmniBotSystem" not in mock
    assert 'name="OmniBotSystem"' not in mock
    assert "<plugin>mock_components/GenericSystem</plugin>" in mock
    assert "<plugin>omnibot_bringup/PlanarWheelMock</plugin>" not in mock


def test_controller_manager_reads_robot_description_topic():
    robot = ROBOT_LAUNCH.read_text(encoding="utf-8")
    sim = SIM_LAUNCH.read_text(encoding="utf-8")
    mock = (
        REPO / "omnibot-ros2" / "omnibot_bringup" / "test" / "test_mock_controllers.py"
    ).read_text(encoding="utf-8")
    for text in (robot, sim, mock):
        assert 'package="robot_state_publisher"' in text
        assert 'executable="robot_state_publisher"' in text
    assert robot.count('{"robot_description": robot_description_config}') == 1
    assert '{"robot_description": robot_description}' not in mock
    assert '"robot_description": robot_description' in mock


def test_ros2_control_adapter_is_executable():
    # colcon --symlink-install keeps this mode. launch's which() skips a
    # non-executable file and reports it missing from libexec.
    listed = subprocess.check_output(
        [
            "git",
            "ls-files",
            "-s",
            "--",
            "omnibot-ros2/omnibot_bringup/scripts/ros2_control_adapter.py",
        ],
        cwd=REPO,
        text=True,
    )
    assert listed.startswith("100755")


def test_adapter_waits_for_arm_joints_and_times_out_twists():
    logic = _logic()
    names = ("joint_state_broadcaster", "mecanum_drive_controller")
    assert logic.controller_listing_active(
        "joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active\n"
        "mecanum_drive_controller mecanum_drive_controller/MecanumDriveController active\n",
        names,
    )
    assert logic.controller_listing_active(
        "joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] \x1b[32mactive\x1b[0m\n"
        "mecanum_drive_controller[mecanum_drive_controller/MecanumDriveController] active\n",
        names,
    )
    assert not logic.controller_listing_active(
        "joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster inactive\n"
        "mecanum_drive_controller mecanum_drive_controller/MecanumDriveController active\n",
        names,
    )
    assert logic.cmd_vel_topic_for_bridge("sim_bridge.yaml") == "/cmd_vel"
    assert logic.cmd_vel_topic_for_bridge("sim_bridge_vr.yaml") == "/cmd_vel/out"
    assert logic.cmd_vel_topic_for_bridge("sim_bridge_mux.yaml") == "/cmd_vel/out"

    tracker = logic.CmdPosTracker()
    assert tracker.trajectory_if_changed() is None
    for name in logic.ARM_JOINTS[:4]:
        tracker.update(name, 0.2)
    assert tracker.trajectory_if_changed() is None
    tracker.update(logic.ARM_JOINTS[4], 0.2)
    assert tracker.trajectory_if_changed() == (0.2, 0.2, 0.2, 0.2, 0.2)
    assert tracker.trajectory_if_changed() is None
    assert tracker.gripper_if_changed() is None
    tracker.update(logic.GRIPPER_JOINT, 0.4)
    assert tracker.gripper_if_changed() == pytest.approx(0.4)
    assert tracker.gripper_if_changed() is None

    names = ["front_left_wheel_joint", *logic.ALL_ARM_JOINTS]
    positions = [1.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    subset = logic.arm_subset(names, positions)
    assert subset[0] == list(logic.ALL_ARM_JOINTS)
    assert subset[1] == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert logic.arm_subset(logic.ARM_JOINTS, [0.0] * 5) is None

    relay = logic.TwistRelay(timeout_sec=0.25)
    assert relay.sample(10.0) == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    relay.update((0.5, 0.0, 0.0), (0.0, 0.0, 0.1), 10.0)
    assert relay.sample(10.2)[0] == pytest.approx(0.5)
    assert relay.sample(10.3) == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    relay.force_zero(11.0)
    assert relay.sample(11.1) == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert relay.stopped() is True
    relay.update((0.2, 0.0, 0.0), (0.0, 0.0, 0.0), 12.0)
    assert relay.stopped() is False
    relay.update((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 12.01)
    assert relay.stopped() is True

    mock = (
        REPO / "omnibot-ros2" / "omnibot_bringup" / "src" / "planar_wheel_mock.cpp"
    ).read_text(encoding="utf-8")
    assert "LinearVelocityCmd" in mock
    assert "AngularVelocityCmd" in mock
    assert "zero_base" in mock
    assert "drop_link_velocity_cmd" in mock
    assert "JointForceCmd(0) stops VelocityControl" in mock
    assert "if (std::abs(command) >= 1e-3) {\n        stopped = false;\n      }" in mock
    adapter = (
        REPO / "omnibot-ros2" / "omnibot_bringup" / "scripts" / "ros2_control_adapter.py"
    ).read_text(encoding="utf-8")
    assert "def _on_twist" in adapter
    assert adapter.count("self._publish_reference(now)") >= 3
    assert "ReliabilityPolicy.BEST_EFFORT" in adapter
    urdf = _xacro_text()
    assert "<odom_publish_frequency>100</odom_publish_frequency>" in urdf
    assert "<gz:system_priority>100</gz:system_priority>" in urdf
