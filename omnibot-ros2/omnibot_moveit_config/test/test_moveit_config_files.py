# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""SRDF and MoveIt YAML checks. No ROS graph.

The colcon job runs test_moveit_plan_execute.py. This file is the CPU check.
"""

import importlib.util
import itertools
import re
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
ROS = REPO / "omnibot-ros2"
PKG = ROS / "omnibot_moveit_config"
URDF = ROS / "omnibot_description" / "urdf" / "omnibot.urdf.xacro"
APRIL = ROS / "omnibot_description" / "urdf" / "apriltags.urdf.xacro"
ARM_PARAMS = ROS / "omnibot_arm" / "config" / "arm_params.yaml"
CONTROLLERS = ROS / "omnibot_bringup" / "config" / "controllers.yaml"

ARM_CHAIN = [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
]
MOVING = {
    "arm_shoulder_link",
    "arm_upper_arm_link",
    "arm_lower_arm_link",
    "arm_wrist_link",
    "arm_gripper_link",
    "arm_moving_jaw_link",
    "wrist_camera_link",
}
# lidar has collision and is inside xacro:if. The demo keeps lidar false.
# bev and chase are the same flag and have no collision geometry.
CONDITIONAL_COLLISION = {"lidar_link"}
CONDITIONAL_LINKS = {"lidar_link", "bev_camera_link", "chase_camera_link"}

JOINT_RE = re.compile(
    r'<joint name="([^"]+)" type="([^"]+)">(.*?)</joint>',
    re.DOTALL,
)
LINK_RE = re.compile(r'<link name="([^"]+)">(.*?)</link>', re.DOTALL)
PARENT_RE = re.compile(r'<parent link="([^"]+)"')
CHILD_RE = re.compile(r'<child link="([^"]+)"')
LIMIT_RE = re.compile(
    r'<limit\b[^>]*\bvelocity="([^"]+)"[^>]*\blower="([^"]+)"[^>]*\bupper="([^"]+)"'
)


def _load(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _joints(*texts: str):
    found = []
    for text in texts:
        for name, kind, body in JOINT_RE.findall(text):
            parent = PARENT_RE.search(body)
            child = CHILD_RE.search(body)
            limit = LIMIT_RE.search(body)
            if parent is None or child is None:
                continue
            found.append(
                {
                    "name": name,
                    "type": kind,
                    "parent": parent.group(1),
                    "child": child.group(1),
                    "limit": limit,
                }
            )
    return found


def _literal(name: str) -> bool:
    return "$" not in name and "{" not in name


def _collision_links(text: str) -> set[str]:
    names = set()
    for name, body in LINK_RE.findall(text):
        if _literal(name) and "<collision" in body:
            names.add(name)
    return names


def _expanded_links(text: str) -> set[str]:
    standoffs = re.findall(r'<xacro:standoff\s+prefix="([^"]+)"', text)
    cameras = re.findall(r'<xacro:base_camera\s+prefix="([^"]+)"', text)
    wheels = re.findall(r'<xacro:wheel\s+prefix="([^"]+)"', text)
    names = {f"{prefix}_standoff_link" for prefix in standoffs}
    names |= {f"{prefix}_camera_link" for prefix in cameras}
    names |= {f"{prefix}_wheel_link" for prefix in wheels}
    assert len(standoffs) == 8
    assert len(cameras) == 4
    assert len(wheels) == 4
    return names


def _model():
    main = URDF.read_text(encoding="utf-8")
    april = APRIL.read_text(encoding="utf-8")
    literal = _collision_links(main) | _collision_links(april)
    assert CONDITIONAL_COLLISION <= literal
    assert "bev_camera_link" not in literal
    assert "chase_camera_link" not in literal
    for name in CONDITIONAL_LINKS:
        assert f'<link name="{name}">' in main
    collision = (literal - CONDITIONAL_COLLISION) | _expanded_links(main)
    joints = [joint for joint in _joints(main, april) if _literal(joint["name"])]
    return main, collision, joints


def _srdf():
    import xml.etree.ElementTree as ET

    root = ET.parse(PKG / "config" / "mecanum_bot.srdf").getroot()
    disabled = {}
    for tag in root.findall("disable_collisions"):
        disabled[frozenset((tag.get("link1"), tag.get("link2")))] = tag.get("reason")
    return root, disabled


def _chain(joints, start, goal):
    edges: dict[str, list[tuple[str, str, str]]] = {}
    for joint in joints:
        edges.setdefault(joint["parent"], []).append(
            (joint["name"], joint["child"], joint["type"])
        )
    stack = [(start, [])]
    seen = set()
    while stack:
        link, revolute = stack.pop()
        if link == goal:
            return revolute
        if link in seen:
            continue
        seen.add(link)
        for name, child, kind in edges.get(link, []):
            extra = revolute + ([name] if kind == "revolute" else [])
            stack.append((child, extra))
    raise AssertionError(f"no chain from {start} to {goal}")


def test_arm_chain_and_gripper_joint_exist_in_the_urdf():
    main, _collision, joints = _model()
    assert 'name="mecanum_bot"' in main
    chain = _chain(joints, "arm_base_link", "arm_gripper_frame_link")
    assert chain == ARM_CHAIN
    gripper = [joint for joint in joints if joint["name"] == "arm_gripper"]
    assert len(gripper) == 1
    assert gripper[0]["type"] == "revolute"
    assert gripper[0]["parent"] == "arm_gripper_link"
    assert gripper[0]["child"] == "arm_moving_jaw_link"


def test_srdf_groups_states_and_virtual_joint():
    root, disabled = _srdf()
    limits = _load(PKG / "config" / "joint_limits.yaml")["joint_limits"]
    assert root.get("name") == "mecanum_bot"
    virtual = root.find("virtual_joint")
    assert virtual.get("name") == "fixed_base"
    assert virtual.get("type") == "fixed"
    assert virtual.get("parent_frame") == "world"
    assert virtual.get("child_link") == "base_footprint"

    arm = root.find("group[@name='arm']/chain")
    assert arm.get("base_link") == "arm_base_link"
    assert arm.get("tip_link") == "arm_gripper_frame_link"
    gripper = root.find("group[@name='gripper']/joint")
    assert gripper.get("name") == "arm_gripper"
    effector = root.find("end_effector")
    assert effector.get("name") == "gripper_ee"
    assert effector.get("group") == "gripper"
    assert effector.get("parent_group") == "arm"
    assert effector.get("parent_link") == "arm_gripper_frame_link"

    expected = {
        ("home", "arm"),
        ("ready", "arm"),
        ("gripper_open", "gripper"),
        ("gripper_closed", "gripper"),
    }
    found = set()
    for state in root.findall("group_state"):
        key = (state.get("name"), state.get("group"))
        found.add(key)
        for joint in state.findall("joint"):
            name = joint.get("name")
            value = float(joint.get("value"))
            assert name in limits
            low = limits[name]["min_position"]
            high = limits[name]["max_position"]
            assert low - 1e-9 <= value <= high + 1e-9
    assert expected <= found
    assert frozenset(("arm_wrist_link", "wrist_camera_link")) in disabled
    assert frozenset(("arm_lower_arm_link", "top_plate_link")) not in disabled
    assert frozenset(("arm_shoulder_link", "base_link")) not in disabled


def test_disabled_pairs_cover_the_rigid_chassis_and_adjacent_links():
    _main, collision, joints = _model()
    _root, disabled = _srdf()
    assert set().union(*disabled) == collision
    chassis = collision - MOVING
    assert "base_link" in chassis
    missing = [
        pair
        for pair in itertools.combinations(sorted(chassis), 2)
        if frozenset(pair) not in disabled
    ]
    assert missing == []
    assert "Never" in set(disabled.values())
    for joint in joints:
        pair = frozenset((joint["parent"], joint["child"]))
        if joint["parent"] in collision and joint["child"] in collision:
            assert disabled.get(pair) == "Adjacent", joint["name"]


def test_controllers_match_bringup_and_gripper_action():
    bringup = _load(CONTROLLERS)
    moveit = _load(PKG / "config" / "moveit_controllers.yaml")
    manager = moveit["moveit_simple_controller_manager"]
    assert manager["controller_names"] == [
        "arm_joint_trajectory_controller",
        "gripper_controller",
    ]
    arm = manager["arm_joint_trajectory_controller"]
    assert arm["type"] == "FollowJointTrajectory"
    assert arm["action_ns"] == "follow_joint_trajectory"
    assert arm["joints"] == ARM_CHAIN
    assert (
        arm["joints"]
        == bringup["arm_joint_trajectory_controller"]["ros__parameters"]["joints"]
    )
    gripper = manager["gripper_controller"]
    assert gripper["type"] == "ParallelGripperCommand"
    assert gripper["action_ns"] == "gripper_cmd"
    assert gripper["joints"] == ["arm_gripper"]
    assert (
        gripper["joints"][0]
        == bringup["gripper_controller"]["ros__parameters"]["joint"]
    )
    assert moveit["moveit_controller_manager"].endswith(
        "MoveItSimpleControllerManager"
    )
    assert "moveit_manage_controllers" not in moveit


def test_joint_limits_are_the_tighter_of_urdf_and_arm_params():
    _main, _collision, joints = _model()
    limits = _load(PKG / "config" / "joint_limits.yaml")["joint_limits"]
    arm = _load(ARM_PARAMS)["arm_driver_node"]["ros__parameters"]
    bringup = _load(CONTROLLERS)
    update_rate = bringup["controller_manager"]["ros__parameters"]["update_rate"]
    delta_cap = update_rate * arm["max_joint_delta_rad"]
    urdf_limits = {
        joint["name"]: joint["limit"]
        for joint in joints
        if joint["limit"] is not None and joint["name"] in arm["joint_names"]
    }
    assert set(limits) == set(arm["joint_names"])
    for index, name in enumerate(arm["joint_names"]):
        velocity, lower, upper = urdf_limits[name].groups()
        low = max(float(lower), arm["joint_min"][index])
        high = min(float(upper), arm["joint_max"][index])
        entry = limits[name]
        assert entry["min_position"] == pytest.approx(low)
        assert entry["max_position"] == pytest.approx(high)
        assert entry["has_position_limits"] is True
        assert entry["has_velocity_limits"] is True
        assert entry["has_acceleration_limits"] is True
        assert 0 < entry["max_velocity"] <= float(velocity)
        assert entry["max_velocity"] <= delta_cap
        assert entry["max_acceleration"] == pytest.approx(entry["max_velocity"])
    gripper_velocity = bringup["gripper_controller"]["ros__parameters"]["max_velocity"]
    assert limits["arm_gripper"]["max_velocity"] == pytest.approx(gripper_velocity)


def test_kinematics_and_planners():
    kinematics = _load(PKG / "config" / "kinematics.yaml")
    text = (PKG / "config" / "kinematics.yaml").read_text(encoding="utf-8")
    assert set(kinematics) == {"arm", "gripper"}
    assert "KDLKinematicsPlugin" in kinematics["arm"]["kinematics_solver"]
    assert kinematics["arm"]["position_only_ik"] is True
    assert kinematics["arm"]["kinematics_solver_timeout"] > 0
    gripper = kinematics["gripper"]
    assert "KDLKinematicsPlugin" in gripper["kinematics_solver"]
    assert gripper["position_only_ik"] is False
    assert gripper["kinematics_solver_search_resolution"] > 0
    assert gripper["kinematics_solver_timeout"] > 0
    assert "5" in text

    ompl = _load(PKG / "config" / "ompl_planning.yaml")
    assert "ompl_interface/OMPLPlanner" in ompl["planning_plugins"]
    assert ompl["arm"]["default_planner_config"] == "RRTConnect"
    assert "RRTConnect" in ompl["planner_configs"]
    assert ompl["planner_configs"]["RRTConnect"]["type"] == "geometric::RRTConnect"

    pilz_path = PKG / "config" / "pilz_industrial_motion_planner_planning.yaml"
    pilz = _load(pilz_path)
    pilz_text = pilz_path.read_text(encoding="utf-8")
    assert "pilz_industrial_motion_planner/CommandPlanner" in pilz["planning_plugins"]
    assert pilz["default_planner_config"] == "PTP"
    for planner in ("PTP", "LIN", "CIRC"):
        assert planner in pilz_text
    cartesian = _load(PKG / "config" / "pilz_cartesian_limits.yaml")["cartesian_limits"]
    assert cartesian["max_trans_vel"] > 0
    assert cartesian["max_trans_acc"] > 0
    assert cartesian["max_trans_dec"] < 0
    assert cartesian["max_rot_vel"] > 0


def test_launches_and_package_manifest():
    demo = (PKG / "launch" / "demo.launch.py").read_text(encoding="utf-8")
    robot = (PKG / "launch" / "move_group.launch.py").read_text(encoding="utf-8")
    manifest = (PKG / "package.xml").read_text(encoding="utf-8")
    readme = (REPO / "README.md").read_text(encoding="utf-8")
    assistant = (PKG / ".setup_assistant").read_text(encoding="utf-8")
    for word in ("gz", "rviz", "use_mock", "move_group", "ros2_control_node"):
        assert word in demo
    assert "ros_gz_sim" in demo
    assert "ros2_control_node" not in robot
    assert "spawner" not in robot
    assert '"use_mock": "false"' in robot
    assert '"use_sim": "false"' in robot
    for package in (
        "moveit_ros_move_group",
        "moveit_planners_ompl",
        "pilz_industrial_motion_planner",
        "moveit_simple_controller_manager",
        "parallel_gripper_controller",
        "joint_trajectory_controller",
        "hardware_interface",
        "ros2controlcli",
    ):
        assert package in manifest
    # The GenericSystem plugin lives in hardware_interface. The library name
    # is not a rosdep key, so no package may depend on it.
    assert "mock_components</" not in manifest
    control = demo.split('executable="ros2_control_node"', 1)[1].split(
        "actions.append", 1
    )[0]
    assert "moveit_config.robot_description" not in control
    assert "controllers_file" in control
    for manifest_path in REPO.rglob("package.xml"):
        if any(part in {".venv", "build", "install", ".git"} for part in manifest_path.parts):
            continue
        text = manifest_path.read_text(encoding="utf-8")
        for tag in ("depend", "exec_depend", "build_depend", "test_depend"):
            assert f"<{tag}>mock_components</{tag}>" not in text, manifest_path
    assert "omnibot_description" in assistant
    assert "config/mecanum_bot.srdf" in assistant
    assert "demo.launch.py" in readme
    assert "move_group.launch.py" in readme
    assert "5 DOF" in readme
    assert "RRTConnect" in readme
    assert "pilz_industrial_motion_planner" in readme
    assert "position_only_ik" in readme


def _load_state_validity():
    path = Path(__file__).resolve().parent / "state_validity.py"
    spec = importlib.util.spec_from_file_location("omnibot_state_validity", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_get_state_validity_uses_jazzy_response_fields():
    """The plan test must keep checking valid, not MoveItErrorCodes.

    Jazzy removed error_code from GetStateValidity.Response. The launch
    test still runs only under colcon; this checks the same formatter.
    """
    module = _load_state_validity()
    blocked = SimpleNamespace(
        valid=False,
        contacts=[
            SimpleNamespace(
                contact_body_1="arm_lower_arm_link",
                contact_body_2="top_plate_link",
            )
        ],
        constraint_result=[SimpleNamespace(result=False, distance=0.02)],
        cost_sources=[SimpleNamespace(cost_density=1.5)],
    )
    detail = module.failure_detail(blocked)
    assert blocked.valid is False
    assert "arm_lower_arm_link/top_plate_link" in detail
    assert "result=False" in detail
    assert "0.02" in detail
    assert "1.5" in detail
    assert "error_code" not in detail
    clear = SimpleNamespace(
        valid=True,
        contacts=[],
        constraint_result=[],
        cost_sources=[],
    )
    assert "valid=True" in module.failure_detail(clear)
    launch = (
        Path(__file__).resolve().parent / "test_moveit_plan_execute.py"
    ).read_text(encoding="utf-8")
    assert "response.error_code" not in launch
    assert "self.assertTrue(response.valid," in launch
    assert "failure_detail(response)" in launch
