# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""ROS-free export of the browser URDF. Needs xacro, trimesh, fast-simplification."""

import json
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import export_web_model as export  # noqa: E402

# Limits are the literals in omnibot.urdf.xacro (themselves from so101_new_calib.urdf).
ARM_LIMITS = {
    "arm_shoulder_pan": (-1.91986, 1.91986),
    "arm_shoulder_lift": (-1.74533, 1.74533),
    "arm_elbow_flex": (-1.69, 1.69),
    "arm_wrist_flex": (-1.65806, 1.65806),
    "arm_wrist_roll": (-2.74385, 2.84121),
    "arm_gripper": (-0.174533, 1.74533),
}
ARM_PARENT_CHILD = {
    "arm_shoulder_pan": ("arm_base_link", "arm_shoulder_link"),
    "arm_shoulder_lift": ("arm_shoulder_link", "arm_upper_arm_link"),
    "arm_elbow_flex": ("arm_upper_arm_link", "arm_lower_arm_link"),
    "arm_wrist_flex": ("arm_lower_arm_link", "arm_wrist_link"),
    "arm_wrist_roll": ("arm_wrist_link", "arm_gripper_link"),
    "arm_gripper": ("arm_gripper_link", "arm_moving_jaw_link"),
}
WHEEL_JOINTS = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)


def _robot(urdf_path: Path) -> ET.Element:
    return ET.parse(urdf_path).getroot()


def test_package_uri_resolves_to_checked_in_stl():
    path = export.resolve_mesh_source(
        "package://omnibot_description/meshes/so101/base_so101_v2.stl",
        description_root=export.PACKAGE_ROOT,
        urdf_dir=export.PACKAGE_ROOT / "urdf",
    )
    assert path.name == "base_so101_v2.stl"
    assert path.is_file()


def test_unknown_package_uri_is_rejected():
    with pytest.raises(export.ExportError, match="not in omnibot_description"):
        export.resolve_mesh_source(
            "package://some_other_pkg/meshes/foo.stl",
            description_root=export.PACKAGE_ROOT,
            urdf_dir=export.PACKAGE_ROOT / "urdf",
        )


def test_export_joints_meshes_and_size_budget(tmp_path, capsys):
    output = tmp_path / "omnibot-pro"
    code = export.main(["--output", str(output)])
    captured = capsys.readouterr()
    assert code == 0, captured.err
    assert captured.err == ""
    assert "uncompressed:" in captured.out
    assert "zip:" in captured.out

    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    zip_path = output.parent / "omnibot-pro.zip"
    zip_bytes = zip_path.stat().st_size
    assert zip_bytes <= export.ZIP_BUDGET_BYTES
    assert f"zip: {zip_bytes} bytes" in captured.out
    assert "zip" not in manifest["bytes"]
    assert manifest["bytes"]["uncompressed"] == sum(
        path.stat().st_size for path in output.rglob("*") if path.is_file()
    )
    assert manifest["xacro"] == "omnibot-ros2/omnibot_description/urdf/omnibot.urdf.xacro"
    assert manifest["gripper_joint"] == "arm_gripper"
    assert manifest["mecanum_wheel_joints"] == list(WHEEL_JOINTS)
    assert manifest["arm_joints"] == list(ARM_LIMITS)
    assert manifest["base_geometry"] == "primitives"

    by_name = {joint["name"]: joint for joint in manifest["joints"]}
    assert {joint["name"] for joint in manifest["joints"] if joint["type"] == "revolute"} == set(ARM_LIMITS)
    assert {joint["name"] for joint in manifest["joints"] if joint["type"] == "continuous"} == set(WHEEL_JOINTS)
    for name, (lower, upper) in ARM_LIMITS.items():
        joint = by_name[name]
        assert (joint["parent"], joint["child"]) == ARM_PARENT_CHILD[name]
        assert joint["limit"]["lower"] == pytest.approx(lower)
        assert joint["limit"]["upper"] == pytest.approx(upper)
        assert joint["limit"]["effort"] == pytest.approx(10.0)
        assert joint["limit"]["velocity"] == pytest.approx(10.0)
        assert joint["axis"] == pytest.approx([0.0, 0.0, 1.0])
    for name in WHEEL_JOINTS:
        joint = by_name[name]
        prefix = name.removesuffix("_joint")
        assert joint["parent"] == "base_link"
        assert joint["child"] == f"{prefix}_link"
        assert "lower" not in joint["limit"]
        assert "upper" not in joint["limit"]
        assert joint["limit"]["effort"] == pytest.approx(100.0)
        assert joint["limit"]["velocity"] == pytest.approx(100.0)
        assert joint["axis"] == pytest.approx([0.0, 1.0, 0.0])

    robot = _robot(output / "omnibot-pro.urdf")
    urdf_text = (output / "omnibot-pro.urdf").read_text(encoding="utf-8")
    assert "package://" not in urdf_text
    assert "<gazebo" not in urdf_text
    assert robot.find("link[@name='lidar_link']") is None
    assert robot.find("link[@name='so_fl_a_standoff_link']") is not None

    base_box = robot.find("link[@name='base_link']/visual/geometry/box")
    assert base_box is not None
    assert [float(part) for part in base_box.attrib["size"].split()] == pytest.approx([0.265, 0.250, 0.006])
    assert robot.find("link[@name='base_link']//mesh") is None
    assert robot.find("link[@name='top_plate_link']/visual/geometry/box") is not None
    assert robot.find("link[@name='front_left_wheel_link']/visual/geometry/cylinder") is not None
    assert robot.find("link[@name='front_left_wheel_link']//mesh") is None

    mesh_nodes = robot.findall(".//mesh")
    assert len(mesh_nodes) == sum(len(visuals) for visuals in export.ARM_LINK_VISUALS.values())
    for node in mesh_nodes:
        href = node.attrib["filename"]
        assert "://" not in href
        assert not Path(href).is_absolute()
        assert (output / href).is_file()
        assert href.endswith(".glb")
    for stem in (
        "base_so101_v2",
        "upper_arm_so101_v1",
        "under_arm_so101_v1",
        "wrist_roll_pitch_so101_v2",
        "moving_jaw_so101_v1",
        "sts3215_03a_v1",
    ):
        assert (output / "meshes" / f"{stem}.glb").is_file()
    assert {item["source"] for item in manifest["meshes"]} == {
        f"meshes/so101/{path.name}"
        for path in (export.PACKAGE_ROOT / "meshes" / "so101").glob("*.stl")
    }
    for item in manifest["meshes"]:
        assert item["faces"] <= item["source_faces"]
        assert item["faces"] > 0


def test_zip_over_budget_fails(tmp_path, capsys):
    code = export.main(["--output", str(tmp_path / "omnibot-pro"), "--zip-budget-bytes", "1"])
    captured = capsys.readouterr()
    assert code == 1
    assert "exceeds budget" in captured.err
