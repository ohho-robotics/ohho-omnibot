# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Assemble the OmniBot descriptor from YAML. No ROS install required."""

import copy
import math
import re
import subprocess
import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from descriptor_assemble import (  # noqa: E402
    MAX_JOINT_STEP_RAD,
    MAX_VX_MPS,
    MAX_WZ_RAD_S,
    assemble_descriptor,
    load_descriptor_yaml,
)

YAML_PATH = Path(__file__).resolve().parents[1] / "config" / "robot_descriptor.yaml"
LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"
NODE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "descriptor_node.py"

JOINT_LIMITS = {
    "arm_shoulder_pan": (-3.14, 3.14),
    "arm_shoulder_lift": (-1.57, 1.57),
    "arm_elbow_flex": (-1.57, 1.57),
    "arm_wrist_flex": (-1.57, 1.57),
    "arm_wrist_roll": (-3.14, 3.14),
    "arm_gripper": (-0.1, 0.8),
}


def test_ceilings_match_omnibot():
    assert MAX_VX_MPS == 0.2
    assert MAX_WZ_RAD_S == 1.0
    assert MAX_JOINT_STEP_RAD == 0.15


def test_yaml_assembles_within_ceilings():
    doc = load_descriptor_yaml(YAML_PATH)
    assert doc["id"] == "omnibot"
    assert doc["model"] == "OmniBot"
    assert doc["v"] == "1.0.0"
    assert doc["drive"]["kind"] == "mecanum"
    assert doc["drive"]["holonomic"] is True
    assert doc["drive"]["caps"]["vx_mps"] == 0.2
    assert doc["drive"]["caps"]["vy_mps"] == 0.2
    assert doc["drive"]["caps"]["wz_rad_s"] == 1.0
    hard = doc["safety"]["hard"]
    assert hard["max_vx_mps"] <= MAX_VX_MPS
    assert hard["max_vy_mps"] <= 0.2
    assert hard["max_wz_rad_s"] <= MAX_WZ_RAD_S
    assert hard["max_accel_x_mps2"] <= 0.5
    assert hard["max_accel_y_mps2"] <= 0.5
    assert hard["max_accel_wz_rad_s2"] <= 2.0
    assert hard["max_joint_step_rad"] <= MAX_JOINT_STEP_RAD
    assert hard["max_vx_mps"] == doc["drive"]["caps"]["vx_mps"]
    assert [tag["id"] for tag in doc["apriltags"]] == [0, 1, 2]
    for tag in doc["apriltags"]:
        assert tag["size"] == pytest.approx(0.08)
        assert len(tag["T_tag_base"]) == 16
        assert tag["T_tag_base"][12:] == [0, 0, 0, 1]
    assert doc["topics"]["cmd"] == "/cmd_vel"
    assert doc["topics"]["state"] == "/odom"
    assert doc["topics"]["estop"] == "/emergency_stop"
    assert doc["topics"]["heartbeat"] == "/ohho/heartbeat"
    assert doc["topics"]["telemetry"] == "/ohho/telemetry"
    assert doc["telemetry"]["schema_id"] == "ohho.telemetry.v1"
    assert doc["telemetry"]["robot_id"] == "omnibot"
    assert doc["telemetry"]["kind"] == "real"
    assert doc["telemetry"]["fields"] == ["base", "joints", "imu"]
    joints = doc["arms"][0]["joints"]
    assert [joint["name"] for joint in joints] == list(JOINT_LIMITS)
    for joint in joints:
        lo, hi = JOINT_LIMITS[joint["name"]]
        assert joint["min"] >= lo
        assert joint["max"] <= hi
        assert lo <= joint["home"] <= hi
    topics = set(doc["topics"]["images"])
    for camera in doc["cameras"]:
        assert camera["topic"] in topics


def test_sim_kind_override_keeps_caps():
    real = load_descriptor_yaml(YAML_PATH)
    sim = load_descriptor_yaml(YAML_PATH, kind="sim")
    assert sim["telemetry"]["kind"] == "sim"
    assert sim["safety"] == real["safety"]
    assert sim["drive"] == real["drive"]
    assert sim["apriltags"] == real["apriltags"]


def test_raised_cap_is_refused():
    doc = load_descriptor_yaml(YAML_PATH)
    raised = copy.deepcopy(doc)
    raised["safety"]["hard"]["max_vx_mps"] = 0.21
    with pytest.raises(ValueError, match="ceiling"):
        assemble_descriptor(raised)


def test_drive_cap_above_hard_cap_is_refused():
    doc = load_descriptor_yaml(YAML_PATH)
    wider = copy.deepcopy(doc)
    wider["safety"]["hard"]["max_vx_mps"] = 0.1
    wider["drive"]["caps"]["vx_mps"] = 0.2
    with pytest.raises(ValueError, match="exceeds"):
        assemble_descriptor(wider)


def test_joint_home_outside_limits_is_refused():
    doc = load_descriptor_yaml(YAML_PATH)
    bad = copy.deepcopy(doc)
    bad["arms"][0]["joints"][0]["home"] = 4.0
    with pytest.raises(ValueError, match="home"):
        assemble_descriptor(bad)


def test_descriptor_node_is_executable():
    # colcon --symlink-install keeps this mode. launch's which() skips a
    # non-executable file and reports it missing from libexec.
    root = Path(__file__).resolve().parents[3]
    listed = subprocess.check_output(
        [
            "git",
            "ls-files",
            "-s",
            "--",
            "omnibot-ros2/omnibot_bringup/scripts/descriptor_node.py",
        ],
        cwd=root,
        text=True,
    )
    assert listed.startswith("100755")


def test_publisher_is_latched_on_the_contract_topic():
    text = NODE_PATH.read_text(encoding="utf-8")
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in text
    assert '"/ohho/descriptor"' in text
    assert "descriptor_assemble" in text


def test_so101_arm_only_descriptor_has_no_base_motion():
    path = Path(__file__).resolve().parents[1] / "config" / "so101_arm_only_descriptor.yaml"
    text = path.read_text(encoding="utf-8")
    assert "cmd_vel" not in text
    doc = load_descriptor_yaml(path, kind="sim")
    assert doc["id"] == "so101-arm"
    assert doc["model"] == "SO-101"
    assert doc["v"] == "1.0.0"
    assert doc["drive"]["kind"] == "fixed-base"
    assert doc["drive"]["holonomic"] is False
    assert doc["drive"]["caps"] == {"vx_mps": 0, "vy_mps": 0, "wz_rad_s": 0}
    hard = doc["safety"]["hard"]
    assert hard["max_vx_mps"] == 0
    assert hard["max_vy_mps"] == 0
    assert hard["max_wz_rad_s"] == 0
    assert hard["max_accel_x_mps2"] == 0
    assert hard["max_accel_y_mps2"] == 0
    assert hard["max_accel_wz_rad_s2"] == 0
    assert hard["max_joint_step_rad"] == MAX_JOINT_STEP_RAD
    assert hard["max_joint_step_rad"] <= MAX_JOINT_STEP_RAD
    assert doc["topics"]["cmd"] == "/arm/joint_commands"
    assert doc["topics"]["state"] == "/arm/joint_states"
    assert doc["topics"]["estop"] == "/emergency_stop"
    assert "cmd_vel" not in doc["topics"]["cmd"]
    assert doc["telemetry"]["kind"] == "sim"
    assert doc["telemetry"]["robot_id"] == "so101-arm"
    assert doc["telemetry"]["fields"] == ["joints", "gripper"]
    assert "base" not in doc["telemetry"]["fields"]
    joints = doc["arms"][0]["joints"]
    assert doc["arms"][0]["name"] == "so101"
    assert [joint["name"] for joint in joints] == list(JOINT_LIMITS)
    for joint in joints:
        lo, hi = JOINT_LIMITS[joint["name"]]
        assert joint["min"] >= lo
        assert joint["max"] <= hi
        assert lo <= joint["home"] <= hi
    assert doc["cameras"][0]["topic"] == "/camera/wrist/image_raw"
    assert doc["cameras"][0]["topic"] in doc["topics"]["images"]


def test_so101_arm_sim_launch_publishes_descriptor_without_a_base():
    launch = (LAUNCH_DIR / "so101_arm_sim.launch.py").read_text(encoding="utf-8")
    compile(launch, "so101_arm_sim.launch.py", "exec")
    assert "arm_driver_node.py" in launch
    assert "arm_params.yaml" in launch
    assert "so101_arm_only_descriptor.yaml" in launch
    assert "descriptor_node.py" in launch
    assert '"/ohho/descriptor"' in (NODE_PATH.read_text(encoding="utf-8"))
    assert '{"simulate": True}' in launch
    assert '"kind": "sim"' in launch
    assert '("/arm/joint_commands/out", "/arm/joint_commands")' in launch
    assert "cmd_vel" not in launch
    assert "yahboom" not in launch.lower()
    assert "cmd_vel_mux" not in launch


def test_robot_and_sim_launches_wire_the_publisher():
    robot = (LAUNCH_DIR / "robot.launch.py").read_text(encoding="utf-8")
    sim = (LAUNCH_DIR / "sim.launch.py").read_text(encoding="utf-8")
    for text in (robot, sim):
        assert '"descriptor"' in text
        assert "descriptor_node.py" in text
        assert "robot_descriptor.yaml" in text
        assert "Publish the latched RobotDescriptor JSON on /ohho/descriptor" in text
    assert '"kind": "real"' in robot
    assert '"kind": "sim"' in sim


def test_empty_apriltags_still_validate():
    doc = load_descriptor_yaml(YAML_PATH)
    doc["apriltags"] = []
    assert assemble_descriptor(doc)["apriltags"] == []


def test_apriltag_schema_rejects_a_bad_tag():
    doc = load_descriptor_yaml(YAML_PATH)
    missing = copy.deepcopy(doc)
    del missing["apriltags"][0]["size"]
    with pytest.raises(ValueError, match="missing"):
        assemble_descriptor(missing)
    extra = copy.deepcopy(doc)
    extra["apriltags"][0]["family"] = "tag36h11"
    with pytest.raises(ValueError, match="unknown keys"):
        assemble_descriptor(extra)
    wide = copy.deepcopy(doc)
    wide["apriltags"][0]["size"] = 1.01
    with pytest.raises(ValueError, match="ceiling"):
        assemble_descriptor(wide)
    short = copy.deepcopy(doc)
    short["apriltags"][0]["T_tag_base"] = [0.0] * 15
    with pytest.raises(ValueError, match="16 numbers"):
        assemble_descriptor(short)


APRILTAG_DIR = Path(__file__).resolve().parents[2] / "omnibot_description" / "apriltags"
DESCRIPTION = APRILTAG_DIR.parent
sys.path.insert(0, str(APRILTAG_DIR))

from apriltag_mounts import (  # noqa: E402
    CARD_M,
    TAG36H11_CODES,
    TAG_SIZE_M,
    Chassis,
    build_pdf,
    clearance_problems,
    documentation_rows,
    jig_size_mm,
    mesh_is_closed,
    mounts,
    render_tag,
    render_xacro,
    rpy_to_axes,
    write_stl,
    _mesh_volume,
    _stl_triangles,
)

# Official tag36h11 images (apriltag-imgs), '#' black. Locks the renderer.
_OFFICIAL = {
    0: (
        "..........",
        ".########.",
        ".#..#.#.#.",
        ".##...#.#.",
        ".##..####.",
        ".#.#.####.",
        ".##.#..##.",
        ".####.###.",
        ".########.",
        "..........",
    ),
    1: (
        "..........",
        ".########.",
        ".#..#..##.",
        ".##.#...#.",
        ".#....###.",
        ".##..####.",
        ".#.#..#.#.",
        ".###.##.#.",
        ".########.",
        "..........",
    ),
    2: (
        "..........",
        ".########.",
        ".#..#...#.",
        ".##.##.##.",
        ".#.######.",
        ".###.##.#.",
        ".####.###.",
        ".###...##.",
        ".########.",
        "..........",
    ),
}


def _xacro_property(text, name):
    match = re.search(
        rf'<xacro:property name="{name}"\s+value="([^"]+)"',
        text,
    )
    assert match, name
    return float(match.group(1))


def test_tag36h11_bits_match_the_official_images():
    for tag_id, grid in _OFFICIAL.items():
        assert render_tag(TAG36H11_CODES[tag_id]) == grid


def test_nominal_poses_match_the_urdf_and_descriptor():
    assert clearance_problems() == ()
    urdf = (DESCRIPTION / "urdf" / "omnibot.urdf.xacro").read_text(encoding="utf-8")
    assert urdf.count("apriltags.urdf.xacro") == 1
    assert '<xacro:include filename="apriltags.urdf.xacro"/>' in urdf
    length = _xacro_property(urdf, "chassis_length")
    width = _xacro_property(urdf, "chassis_width")
    plate = _xacro_property(urdf, "plate_thickness")
    standoff = _xacro_property(urdf, "standoff_height")
    chassis = Chassis()
    assert length == chassis.length
    assert width == chassis.width
    assert plate == chassis.plate_thickness
    assert standoff == chassis.standoff_height
    assert 'box size="0.075 0.065 0.055"' in urdf
    assert 'xyz="-0.031 0 0.068"' in urdf
    assert 'box size="0.080 0.060 0.006"' in urdf
    arm = re.search(
        r'<joint name="arm_base_joint" type="fixed">\s*'
        r'<parent link="top_plate_link"/>\s*'
        r'<child link="arm_base_link"/>\s*'
        r'<origin xyz="0\.055 0 \$\{plate_thickness / 2\.0\}" rpy="0 0 0"/>',
        urdf,
    )
    assert arm is not None

    tag_xacro = DESCRIPTION / "urdf" / "apriltags.urdf.xacro"
    text = tag_xacro.read_text(encoding="utf-8").replace("\r\n", "\n")
    assert text == render_xacro()
    assert "<sensor" not in text
    assert 'type="camera"' not in text
    top_origin = (0.0, 0.0, plate + standoff)
    arm_origin = (0.055, 0.0, plate + standoff + plate / 2.0)
    parents = {
        "top_plate_link": top_origin,
        "arm_base_link": arm_origin,
    }
    found = re.findall(
        r'<joint name="(apriltag_[^"]+)" type="fixed">\s*'
        r'<parent link="([^"]+)"/>\s*'
        r'<child link="([^"]+)"/>\s*'
        r'<origin xyz="([^"]+)" rpy="([^"]+)"/>',
        text,
    )
    by_joint = {name: (parent, child, xyz, rpy) for name, parent, child, xyz, rpy in found}
    doc = load_descriptor_yaml(YAML_PATH)
    for mount, tag in zip(mounts(), doc["apriltags"]):
        assert tag["id"] == mount.tag_id
        assert tag["size"] == pytest.approx(TAG_SIZE_M)
        parent, child, xyz_text, rpy_text = by_joint[mount.joint]
        assert parent == mount.parent
        assert child == mount.link
        xyz = tuple(float(part) for part in xyz_text.split())
        rpy = tuple(float(part) for part in rpy_text.split())
        for got, want in zip(xyz, mount.origin_parent()):
            assert got == pytest.approx(want, abs=1e-9)
        axes = rpy_to_axes(*rpy)
        for got_axis, want_axis in zip(axes, (mount.x_axis, mount.y_axis, mount.z_axis)):
            for got, want in zip(got_axis, want_axis):
                assert got == pytest.approx(want, abs=1e-8)
        origin = tuple(parents[parent][i] + xyz[i] for i in range(3))
        for got, want in zip(origin, mount.origin_base):
            assert got == pytest.approx(want, abs=1e-9)
        for got, want in zip(tag["T_tag_base"], mount.matrix()):
            assert got == pytest.approx(want, abs=1e-9)
        # Columns of the stored matrix are the tag axes in base_link.
        matrix = tag["T_tag_base"]
        for col, axis in enumerate((mount.x_axis, mount.y_axis, mount.z_axis)):
            for row in range(3):
                assert matrix[row * 4 + col] == pytest.approx(axis[row], abs=1e-9)
        assert math.isclose(matrix[3], mount.origin_base[0], abs_tol=1e-9)
        assert math.isclose(matrix[7], mount.origin_base[1], abs_tol=1e-9)
        assert math.isclose(matrix[11], mount.origin_base[2], abs_tol=1e-9)


def test_printables_and_gazebo_textures():
    pdf_path = APRILTAG_DIR / "print" / "omnibot_tag36h11.pdf"
    stl_path = APRILTAG_DIR / "print" / "apriltag_mount_jig.stl"
    pdf = pdf_path.read_bytes()
    assert pdf.startswith(b"%PDF-1.4")
    assert pdf == build_pdf()
    assert len(pdf) < 200_000
    text = pdf.decode("latin-1")
    bar = f"{100.0 * 72.0 / 25.4:.2f}"
    assert text.count(bar) == 4
    for tag_id in (0, 1, 2):
        assert f"tag36h11 id {tag_id}" in text
    assert "nominal, to be measured" in text
    assert "matte" in text
    tris = _stl_triangles()
    assert mesh_is_closed(tris)
    assert _mesh_volume(tris) > 0.0
    depth, width, height = jig_size_mm()
    xs, ys, zs = [], [], []
    for tri in tris:
        for point in tri:
            xs.append(point[0])
            ys.append(point[1])
            zs.append(point[2])
    assert min(xs) == pytest.approx(0.0, abs=1e-6)
    assert max(xs) == pytest.approx(depth, abs=1e-4)
    assert min(ys) == pytest.approx(0.0, abs=1e-6)
    assert max(ys) == pytest.approx(width, abs=1e-4)
    assert min(zs) == pytest.approx(0.0, abs=1e-6)
    assert max(zs) == pytest.approx(height, abs=1e-4)
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        generated = Path(tmp) / "jig.stl"
        write_stl(generated)
        assert generated.read_text(encoding="utf-8") == stl_path.read_text(
            encoding="utf-8"
        ).replace("\r\n", "\n")
    for tag_id in (0, 1, 2):
        obj = (DESCRIPTION / "meshes" / "apriltags" / f"tag36h11_id{tag_id}.obj").read_text(
            encoding="utf-8"
        )
        assert f"mtllib tag36h11_id{tag_id}.mtl" in obj
        assert f"map_Kd tag36h11_id{tag_id}.png" in (
            DESCRIPTION / "meshes" / "apriltags" / f"tag36h11_id{tag_id}.mtl"
        ).read_text(encoding="utf-8")
        assert "f 1/1 2/2 3/3" in obj
        png = (DESCRIPTION / "meshes" / "apriltags" / f"tag36h11_id{tag_id}.png").read_bytes()
        assert png.startswith(b"\x89PNG\r\n\x1a\n")
    assert CARD_M == pytest.approx(0.10)


def test_apriltag_doc_lists_the_nominal_table():
    doc = (
        Path(__file__).resolve().parents[3] / "docs" / "apriltags.md"
    ).read_text(encoding="utf-8")
    assert "nominal, to be measured" in doc
    assert "tag36h11" in doc
    assert "OHH-213" in doc
    for row in documentation_rows():
        assert row in doc
