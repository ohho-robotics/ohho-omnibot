# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""URDF mesh references, joint frames, and cited limit values. No ROS install."""

import shutil
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import export_web_model as export  # noqa: E402
import generate_base_meshes as meshes  # noqa: E402

PACKAGE = export.PACKAGE_ROOT
XACRO = export.DEFAULT_XACRO
MESH_DIR = PACKAGE / "meshes"

# Frozen joint frames. These are the expanded origins from before the mesh
# visuals landed. base_link, the wheel joints, and the arm joints must stay.
WHEEL_XYZ = {
    "front_left_wheel_joint": (0.0825, 0.1075, -0.033),
    "front_right_wheel_joint": (0.0825, -0.1075, -0.033),
    "rear_left_wheel_joint": (-0.0825, 0.1075, -0.033),
    "rear_right_wheel_joint": (-0.0825, -0.1075, -0.033),
}
ARM_XYZ_RPY = {
    "arm_shoulder_pan": (
        "0.0388353 -8.97657e-09 0.0624",
        "3.14159 4.18253e-17 -3.14159",
    ),
    "arm_shoulder_lift": (
        "-0.0303992 -0.0182778 -0.0542",
        "-1.5708 -1.5708 0",
    ),
    "arm_elbow_flex": (
        "-0.11257 -0.028 1.73763e-16",
        "-3.63608e-16 8.74301e-16 1.5708",
    ),
    "arm_wrist_flex": (
        "-0.1349 0.0052 3.62355e-17",
        "4.02456e-15 8.67362e-16 -1.5708",
    ),
    "arm_wrist_roll": (
        "5.55112e-17 -0.0611 0.0181",
        "1.5708 0.0486795 3.14159",
    ),
    "arm_gripper": (
        "0.0202 0.0188 -0.0234",
        "1.5708 -5.24284e-08 -1.41553e-15",
    ),
}
TRACKED_SUFFIXES = {".stl", ".dae", ".obj", ".mtl", ".png", ".jpg", ".jpeg", ".step", ".stp"}
DOC_SUFFIXES = {".md"}


def _local(tag):
    if not isinstance(tag, str):
        return ""
    return tag.split("}", 1)[-1]


def _expand(mappings):
    return export._expand_xacro(XACRO, mappings)


def _top_joints(robot):
    joints = {}
    for element in robot:
        if _local(element.tag) != "joint" or element.find("parent") is None:
            continue
        joints[element.attrib["name"]] = element
    return joints


def _floats(text):
    return [float(part) for part in text.split()]


def _mesh_uris(robot):
    uris = []
    for element in robot.iter():
        if _local(element.tag) == "mesh":
            filename = element.attrib.get("filename")
            assert filename, "a <mesh> has no filename"
            uris.append(filename)
    return uris


def _companions(path: Path) -> set[Path]:
    if path.suffix.lower() not in {".obj", ".mtl"}:
        return set()
    found = set()
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = line.strip()
        lower = stripped.lower()
        key = None
        if path.suffix.lower() == ".obj" and lower.startswith("mtllib "):
            key = stripped.split(None, 1)[1]
        elif path.suffix.lower() == ".mtl" and lower.startswith(("map_kd ", "map_ka ", "map_ks ")):
            key = stripped.split(None, 1)[1]
        if key:
            found.add((path.parent / key.strip()).resolve())
    return found


def _referenced_files(robot) -> set[Path]:
    referenced = set()
    for filename in _mesh_uris(robot):
        path = export.resolve_mesh_source(
            filename,
            description_root=PACKAGE,
            urdf_dir=XACRO.parent,
        ).resolve()
        stack = [path]
        while stack:
            current = stack.pop()
            if current in referenced:
                continue
            assert current.is_file(), current
            referenced.add(current)
            stack.extend(_companions(current))
    return referenced


def _vertices(path: Path):
    coords = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if "vertex" in line:
            coords.append(tuple(round(float(part), 6) for part in line.split()[-3:]))
    return coords


def test_every_mesh_is_referenced_and_every_uri_exists():
    variants = [
        export.WEB_XACRO_ARGS,
        {"drive": "mecanum", "lidar": "true", "minimal_sensors": "false", "vr_cameras": "false"},
        {"drive": "mecanum", "lidar": "false", "minimal_sensors": "false", "vr_cameras": "true"},
        {"drive": "planar", "lidar": "true", "minimal_sensors": "true", "vr_cameras": "false"},
    ]
    referenced = set()
    for mappings in variants:
        referenced |= _referenced_files(_expand(mappings))
    for path in MESH_DIR.rglob("*"):
        if not path.is_file():
            continue
        suffix = path.suffix.lower()
        if suffix in DOC_SUFFIXES:
            continue
        assert suffix in TRACKED_SUFFIXES, f"unexpected file in meshes/: {path.name}"
        assert path.resolve() in referenced, f"unreferenced mesh {path.relative_to(PACKAGE)}"


def test_joint_frames_are_unchanged():
    robot = _expand(export.WEB_XACRO_ARGS)
    joints = _top_joints(robot)
    footprint = joints["base_footprint_joint"]
    assert footprint.find("parent").attrib["link"] == "base_footprint"
    assert footprint.find("child").attrib["link"] == "base_link"
    assert _floats(footprint.find("origin").attrib["xyz"]) == pytest.approx([0.0, 0.0, 0.073])
    assert footprint.find("origin").attrib["rpy"] == "0 0 0"
    for name, xyz in WHEEL_XYZ.items():
        joint = joints[name]
        assert joint.attrib["type"] == "continuous"
        assert joint.find("parent").attrib["link"] == "base_link"
        assert joint.find("child").attrib["link"] == name.removesuffix("_joint") + "_link"
        assert joint.find("axis").attrib["xyz"] == "0 1 0"
        assert _floats(joint.find("origin").attrib["xyz"]) == pytest.approx(xyz)
        assert joint.find("origin").attrib["rpy"] == "0 0 0"
    arm_base = joints["arm_base_joint"]
    assert arm_base.find("parent").attrib["link"] == "top_plate_link"
    assert arm_base.find("child").attrib["link"] == "arm_base_link"
    assert _floats(arm_base.find("origin").attrib["xyz"]) == pytest.approx([0.055, 0.0, 0.003])
    for name, (xyz, rpy) in ARM_XYZ_RPY.items():
        origin = joints[name].find("origin")
        assert origin.attrib["xyz"] == xyz
        assert origin.attrib["rpy"] == rpy
        assert joints[name].find("axis").attrib["xyz"] == "0 0 1"


def test_limits_cite_sts3215_and_mark_wheel_estimates():
    text = XACRO.read_text(encoding="utf-8")
    assert "STS3215" in text
    assert "19.5" in text
    assert "0.192" in text
    assert "yahboom_params.yaml" in text
    assert "ESTIMATES" in text
    assert "encoder_ticks_per_revolution" in text
    robot = _expand(export.WEB_XACRO_ARGS)
    joints = _top_joints(robot)
    for name in export.ARM_JOINTS:
        limit = joints[name].find("limit").attrib
        assert float(limit["effort"]) == pytest.approx(1.9123)
        assert float(limit["velocity"]) == pytest.approx(5.4542)
    for name in export.WHEEL_JOINTS:
        limit = joints[name].find("limit").attrib
        assert float(limit["effort"]) == pytest.approx(100.0)
        assert float(limit["velocity"]) == pytest.approx(100.0)
        assert "lower" not in limit and "upper" not in limit
    # sim.launch.py passes gz_arm_effort:=10. The default expansion above
    # stays the STS3215 rating; the override changes effort only.
    assert 'name="gz_arm_effort" default="${sts3215_effort}"' in text
    sim = _expand({**export.WEB_XACRO_ARGS, "gz_arm_effort": "10"})
    sim_joints = _top_joints(sim)
    for name in export.ARM_JOINTS:
        default_limit = joints[name].find("limit").attrib
        sim_limit = sim_joints[name].find("limit").attrib
        assert float(sim_limit["effort"]) == pytest.approx(10.0)
        assert float(sim_limit["velocity"]) == float(default_limit["velocity"])
        assert sim_limit["lower"] == default_limit["lower"]
        assert sim_limit["upper"] == default_limit["upper"]


def test_base_and_wheel_inertia_match_the_stated_mass_and_shape():
    robot = _expand(export.WEB_XACRO_ARGS)
    links = {
        element.attrib["name"]: element
        for element in robot
        if _local(element.tag) == "link"
    }
    base = links["base_link"].find("inertial")
    mass = 1.5
    length, width, thickness = 0.265, 0.250, 0.006
    assert float(base.find("mass").attrib["value"]) == pytest.approx(mass)
    inertia = base.find("inertia").attrib
    assert float(inertia["ixx"]) == pytest.approx(mass / 12.0 * (width**2 + thickness**2))
    assert float(inertia["iyy"]) == pytest.approx(mass / 12.0 * (length**2 + thickness**2))
    assert float(inertia["izz"]) == pytest.approx(mass / 12.0 * (length**2 + width**2))
    top = links["top_plate_link"].find("inertial")
    top_mass = 1.0
    assert float(top.find("mass").attrib["value"]) == pytest.approx(top_mass)
    top_inertia = top.find("inertia").attrib
    assert float(top_inertia["izz"]) == pytest.approx(top_mass / 12.0 * (length**2 + width**2))
    wheel = links["front_left_wheel_link"].find("inertial")
    assert wheel.find("origin").attrib["rpy"] == "0 0 0"
    wheel_mass, radius, wheel_width = 0.2, 0.04, 0.037
    assert float(wheel.find("mass").attrib["value"]) == pytest.approx(wheel_mass)
    wheel_inertia = wheel.find("inertia").attrib
    transverse = wheel_mass / 12.0 * (3.0 * radius**2 + wheel_width**2)
    assert float(wheel_inertia["ixx"]) == pytest.approx(transverse)
    assert float(wheel_inertia["iyy"]) == pytest.approx(wheel_mass / 2.0 * radius**2)
    assert float(wheel_inertia["izz"]) == pytest.approx(transverse)
    for link in links.values():
        for kind in link.findall("collision/geometry/*"):
            assert _local(kind.tag) != "mesh"
        inertial = link.find("inertial")
        if inertial is None:
            continue
        assert float(inertial.find("mass").attrib["value"]) > 0.0
        values = inertial.find("inertia").attrib
        for key in ("ixx", "iyy", "izz"):
            assert float(values[key]) > 0.0


def test_simplified_base_meshes_match_the_generator(tmp_path):
    written = meshes.write_base_meshes(XACRO, tmp_path)
    assert {path.name for path in written} == {
        "chassis_plate.stl",
        "mecanum_wheel_cs_pos.stl",
        "mecanum_wheel_cs_neg.stl",
    }
    for path in written:
        committed = MESH_DIR / "base" / path.name
        assert committed.is_file()
        assert _vertices(path) == _vertices(committed)
        assert committed.read_text(encoding="utf-8").startswith("solid simplified_")
    plate = _vertices(MESH_DIR / "base" / "chassis_plate.stl")
    xs = [vertex[0] for vertex in plate]
    ys = [vertex[1] for vertex in plate]
    zs = [vertex[2] for vertex in plate]
    assert max(xs) == pytest.approx(0.265 / 2.0)
    assert min(xs) == pytest.approx(-0.265 / 2.0)
    assert max(ys) == pytest.approx(0.250 / 2.0)
    assert min(ys) == pytest.approx(-0.250 / 2.0)
    assert max(zs) == pytest.approx(0.006 / 2.0)
    assert min(zs) == pytest.approx(-0.006 / 2.0)
    positive = _vertices(MESH_DIR / "base" / "mecanum_wheel_cs_pos.stl")
    negative = _vertices(MESH_DIR / "base" / "mecanum_wheel_cs_neg.stl")
    assert positive != negative
    assert len(positive) == len(negative) > 0


def test_check_urdf_when_the_tool_is_installed(tmp_path):
    tool = shutil.which("check_urdf")
    if tool is None:
        pytest.skip("check_urdf is not installed; the Python URDF checks above cover this machine")
    urdf_path = tmp_path / "omnibot.urdf"
    robot = _expand(export.WEB_XACRO_ARGS)
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(urdf_path, encoding="utf-8", xml_declaration=True)
    result = subprocess.run([tool, str(urdf_path)], capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr or result.stdout
