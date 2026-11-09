# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Export the OmniBot URDF to a browser folder for three.js urdf-loader.

No ROS install is required. The PyPI ``xacro`` package expands
``urdf/omnibot.urdf.xacro``. Mesh origins are read from that URDF.
The SO-101 STLs and the simplified chassis and wheel STLs are decimated
with trimesh (fast-simplification) and written as binary GLB.
``package://`` mesh URIs become paths relative to the URDF so a browser
can fetch them. Collision geometry stays the primitive shapes.
Standoffs and cameras stay primitives. Nominal AprilTag cards are omitted.

The compressed zip of the output folder must stay within 10 MiB.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import zipfile
from pathlib import Path
from typing import NamedTuple
from xml.etree import ElementTree as ET

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
DEFAULT_XACRO = PACKAGE_ROOT / "urdf" / "omnibot.urdf.xacro"
DEFAULT_OUTPUT = REPO_ROOT / "build" / "web-model" / "omnibot-pro"
URDF_NAME = "omnibot-pro.urdf"
PACKAGE_NAME = "omnibot_description"
ZIP_BUDGET_BYTES = 10 * 1024 * 1024

# Desktop simulation.launch.py defaults: mecanum plugin, no lidar, full cameras.
WEB_XACRO_ARGS = {
    "drive": "mecanum",
    "lidar": "false",
    "minimal_sensors": "false",
}

# Six actuated SO-101 joints (five arm axes + gripper), from omnibot.urdf.xacro.
ARM_JOINTS = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)
GRIPPER_JOINT = "arm_gripper"
WHEEL_JOINTS = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)

# Links whose visuals are meshes. Collision on these links stays primitive.
ARM_MESH_LINKS = (
    "arm_base_link",
    "arm_shoulder_link",
    "arm_upper_arm_link",
    "arm_lower_arm_link",
    "arm_wrist_link",
    "arm_gripper_link",
    "arm_moving_jaw_link",
)
CHASSIS_MESH_LINKS = ("base_link", "top_plate_link")
WHEEL_MESH_LINKS = (
    "front_left_wheel_link",
    "front_right_wheel_link",
    "rear_left_wheel_link",
    "rear_right_wheel_link",
)
MESH_VISUAL_LINKS = ARM_MESH_LINKS + CHASSIS_MESH_LINKS + WHEEL_MESH_LINKS


class MeshVisual(NamedTuple):
    """One visual mesh, with the origin read from the URDF."""

    link: str
    filename: str
    xyz: str
    rpy: str

# Quadric-decimation targets. Parts already under the target are kept whole.
FACE_TARGETS = {
    "base_motor_holder_so101_v1": 5000,
    "base_so101_v2": 4000,
    "motor_holder_so101_base_v1": 4000,
    "motor_holder_so101_wrist_v1": 3500,
    "moving_jaw_so101_v1": 2500,
    "rotation_pitch_so101_v1": 3000,
    "sts3215_03a_no_horn_v1": 2500,
    "sts3215_03a_v1": 2500,
    "under_arm_so101_v1": 4500,
    "upper_arm_so101_v1": 4000,
    "waveshare_mounting_plate_so101_v2": 1254,
    "wrist_roll_follower_so101_v1": 4000,
    "wrist_roll_pitch_so101_v2": 5000,
}
DEFAULT_FACE_CAP = 4000
# Quadric decimation clips thin tips, so corners move a few millimetres.
# A translated mesh would move the bounding-box centre by centimetres.
CENTER_TOLERANCE_M = 0.005


class ExportError(Exception):
    """The web model could not be produced or failed a check."""


class MeshRecord(NamedTuple):
    file: str
    source: str
    source_faces: int
    faces: int


def resolve_mesh_source(filename: str, *, description_root: Path, urdf_dir: Path) -> Path:
    """Resolve a URDF mesh filename to a file on disk.

    ``package://omnibot_description/...`` maps onto this package directory
    (the same relative layout colcon installs under share/).
    """
    raw = filename.strip().replace("\\", "/")
    if raw.startswith("package://"):
        rest = raw[len("package://") :]
        package, sep, relative = rest.partition("/")
        if not sep or not package or not relative:
            raise ExportError(f"malformed mesh URI: {filename}")
        if package != PACKAGE_NAME:
            raise ExportError(
                f"mesh URI {filename} is not in {PACKAGE_NAME}; "
                "this export only rewrites that package"
            )
        path = (description_root / relative).resolve()
    elif raw.startswith("file://"):
        path = Path(raw[len("file://") :])
        if not path.is_absolute():
            path = (urdf_dir / path).resolve()
    else:
        path = Path(raw)
        if not path.is_absolute():
            beside_urdf = (urdf_dir / path).resolve()
            path = beside_urdf if beside_urdf.is_file() else (description_root / path).resolve()
        else:
            path = path.resolve()
    if not path.is_file():
        raise ExportError(f"mesh not found: {filename} -> {path}")
    return path


def _local(tag: object) -> str:
    # Comments and processing instructions use a callable tag, not a string.
    if not isinstance(tag, str):
        return ""
    if tag.startswith("{"):
        return tag.split("}", 1)[1]
    return tag


def _expand_xacro(xacro_path: Path, mappings: dict[str, str]) -> ET.Element:
    try:
        import xacro
    except ImportError as exc:
        raise ExportError(
            "PyPI package 'xacro' is not installed. "
            "Install it with: python -m pip install xacro"
        ) from exc
    try:
        doc = xacro.process_file(str(xacro_path), mappings=dict(mappings))
    except Exception as exc:
        raise ExportError(f"xacro failed for {xacro_path}: {exc}") from exc
    root = ET.fromstring(doc.toxml(encoding="utf-8"))
    if _local(root.tag) != "robot":
        raise ExportError(f"xacro root is <{_local(root.tag)}>, expected <robot>")
    return root


def _drop_nominal_apriltag_mounts(robot: ET.Element) -> None:
    """Leave tag36h11 cards in the Gazebo model and out of the browser URDF.

    Those links are meshes for headset detection. This export converts the
    SO-101 STLs and the simplified chassis and wheel STLs.
    """
    for element in list(robot):
        kind = _local(element.tag)
        if kind == "link" and element.attrib.get("name", "").startswith("apriltag_"):
            robot.remove(element)
            continue
        if kind != "joint":
            continue
        names = [element.attrib.get("name", "")]
        for role in ("parent", "child"):
            node = element.find(role)
            if node is not None:
                names.append(node.attrib.get("link", ""))
        if any(name.startswith("apriltag_") for name in names):
            robot.remove(element)


def _strip_gazebo(robot: ET.Element) -> None:
    for child in list(robot):
        if _local(child.tag) == "gazebo":
            robot.remove(child)


def mesh_visuals(robot: ET.Element) -> list[MeshVisual]:
    """Read visual mesh filenames and origins from an expanded URDF.

    The xacro is the only place those origins are written. This does not
    substitute a second table.
    """
    found: list[MeshVisual] = []
    for element in robot:
        if _local(element.tag) != "link" or "name" not in element.attrib:
            continue
        for visual in element.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None or not mesh.attrib.get("filename"):
                continue
            origin = visual.find("origin")
            xyz = origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            rpy = origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"
            found.append(MeshVisual(element.attrib["name"], mesh.attrib["filename"], xyz, rpy))
    return found


def _bounds_center_delta(left, right) -> float:
    delta = 0.0
    for axis in range(3):
        left_center = (float(left[0][axis]) + float(left[1][axis])) / 2.0
        right_center = (float(right[0][axis]) + float(right[1][axis])) / 2.0
        delta = max(delta, abs(left_center - right_center))
    return delta


def _convert_stl(source: Path, dest: Path) -> tuple[int, int]:
    try:
        import trimesh
    except ImportError as exc:
        raise ExportError(
            "PyPI package 'trimesh' is not installed. "
            "Install it with: python -m pip install trimesh fast-simplification"
        ) from exc
    loaded = trimesh.load(source, force="mesh", process=False)
    if not isinstance(loaded, trimesh.Trimesh):
        raise ExportError(f"{source.name} did not load as a triangle mesh")
    source_faces = int(len(loaded.faces))
    if source_faces <= 0:
        raise ExportError(f"{source.name} has no triangles")
    target = FACE_TARGETS.get(source.stem, min(source_faces, DEFAULT_FACE_CAP))
    mesh = loaded
    if source_faces > target:
        try:
            mesh = loaded.simplify_quadric_decimation(face_count=int(target))
        except Exception as exc:
            raise ExportError(
                f"decimation failed for {source.name}. "
                "Install fast-simplification (trimesh uses it for quadric decimation)."
            ) from exc
    if len(mesh.faces) <= 0:
        raise ExportError(f"decimation removed every face of {source.name}")
    if _bounds_center_delta(loaded.bounds, mesh.bounds) > CENTER_TOLERANCE_M:
        raise ExportError(
            f"decimation shifted the centre of {source.name} by more than "
            f"{CENTER_TOLERANCE_M} m; refusing to write a mesh that would no longer "
            "sit on the URDF origin"
        )
    dest.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(dest, file_type="glb")
    if not dest.is_file() or dest.stat().st_size <= 0:
        raise ExportError(f"GLB write failed for {dest}")
    return source_faces, int(len(mesh.faces))


def _rewrite_meshes(robot: ET.Element, *, description_root: Path, urdf_dir: Path, mesh_dir: Path) -> list[MeshRecord]:
    cache: dict[Path, MeshRecord] = {}
    stems: dict[str, Path] = {}
    for element in robot.iter("mesh"):
        filename = element.attrib.get("filename")
        if not filename:
            raise ExportError("a <mesh> element has no filename")
        source = resolve_mesh_source(filename, description_root=description_root, urdf_dir=urdf_dir)
        record = cache.get(source)
        if record is None:
            previous = stems.get(source.stem)
            if previous is not None and previous != source:
                raise ExportError(f"two meshes share the output name {source.stem}.glb")
            relative = f"meshes/{source.stem}.glb"
            source_faces, faces = _convert_stl(source, mesh_dir / f"{source.stem}.glb")
            try:
                source_rel = source.relative_to(description_root).as_posix()
            except ValueError:
                source_rel = source.as_posix()
            record = MeshRecord(relative, source_rel, source_faces, faces)
            cache[source] = record
            stems[source.stem] = source
        element.set("filename", record.file)
    return sorted(cache.values(), key=lambda item: item.file)


def _float_attr(element: ET.Element, key: str) -> float | None:
    raw = element.attrib.get(key)
    if raw is None or raw == "":
        return None
    return float(raw)


def _joint_records(robot: ET.Element) -> list[dict]:
    records = []
    for joint in robot:
        if _local(joint.tag) != "joint":
            continue
        name = joint.attrib.get("name")
        if not name:
            raise ExportError("a <joint> has no name")
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ExportError(f"joint {name} is missing parent or child")
        axis_el = joint.find("axis")
        axis = None
        if axis_el is not None and "xyz" in axis_el.attrib:
            axis = [float(part) for part in axis_el.attrib["xyz"].split()]
        limit_el = joint.find("limit")
        limit = None
        if limit_el is not None:
            limit = {}
            for key in ("lower", "upper", "effort", "velocity"):
                value = _float_attr(limit_el, key)
                if value is not None:
                    limit[key] = value
        records.append(
            {
                "name": name,
                "type": joint.attrib.get("type"),
                "parent": parent.attrib.get("link"),
                "child": child.attrib.get("link"),
                "axis": axis,
                "limit": limit,
            }
        )
    return records


def _link_map(robot: ET.Element) -> dict[str, ET.Element]:
    return {
        element.attrib["name"]: element
        for element in robot
        if _local(element.tag) == "link" and "name" in element.attrib
    }


def _geometry_kinds(link: ET.Element, section: str) -> list[str]:
    kinds = []
    for geometry in link.findall(f"{section}/geometry"):
        kinds.extend(_local(child.tag) for child in geometry)
    return kinds


def _validate_tree(
    robot: ET.Element,
    joints: list[dict],
    meshes: list[MeshRecord],
    visuals: list[MeshVisual],
) -> None:
    by_name = {joint["name"]: joint for joint in joints}
    revolute = {joint["name"] for joint in joints if joint["type"] == "revolute"}
    continuous = {joint["name"] for joint in joints if joint["type"] == "continuous"}
    if revolute != set(ARM_JOINTS):
        raise ExportError(f"revolute joints {sorted(revolute)} != {list(ARM_JOINTS)}")
    if continuous != set(WHEEL_JOINTS):
        raise ExportError(f"continuous joints {sorted(continuous)} != {list(WHEEL_JOINTS)}")
    for name in ARM_JOINTS:
        limit = by_name[name]["limit"] or {}
        for key in ("lower", "upper", "effort", "velocity"):
            if key not in limit:
                raise ExportError(f"{name} is missing limit {key}")
        if limit["lower"] >= limit["upper"]:
            raise ExportError(f"{name} limits are not ordered: {limit}")
    for name in WHEEL_JOINTS:
        limit = by_name[name]["limit"] or {}
        for key in ("effort", "velocity"):
            if key not in limit:
                raise ExportError(f"{name} is missing limit {key}")
        if "lower" in limit or "upper" in limit:
            raise ExportError(f"continuous wheel joint {name} should not have lower/upper")
    so101_names = {path.name for path in (PACKAGE_ROOT / "meshes" / "so101").glob("*.stl")}
    base_names = {path.name for path in (PACKAGE_ROOT / "meshes" / "base").glob("*.stl")}
    used_names = {Path(item.filename).name for item in visuals}
    missing = sorted((so101_names | base_names) - used_names)
    if missing:
        raise ExportError(f"URDF does not reference meshes: {missing}")
    expected_sources = {f"meshes/so101/{name}" for name in so101_names} | {
        f"meshes/base/{name}" for name in base_names
    }
    if {record.source for record in meshes} != expected_sources:
        raise ExportError("export did not convert every SO-101 and simplified base STL")
    mesh_links = {item.link for item in visuals}
    if mesh_links != set(MESH_VISUAL_LINKS):
        raise ExportError(
            f"mesh links {sorted(mesh_links)} != {list(MESH_VISUAL_LINKS)}"
        )
    links = _link_map(robot)
    for link_name in MESH_VISUAL_LINKS:
        link = links.get(link_name)
        if link is None:
            raise ExportError(f"{link_name} is missing")
        visual_kinds = _geometry_kinds(link, "visual")
        if not visual_kinds or any(kind != "mesh" for kind in visual_kinds):
            raise ExportError(f"{link_name} visuals are not all meshes: {visual_kinds}")
    for link_name, link in links.items():
        if any(kind == "mesh" for kind in _geometry_kinds(link, "collision")):
            raise ExportError(f"{link_name} collision uses a mesh")
    base = links.get("base_link")
    if base is None or "box" not in _geometry_kinds(base, "collision"):
        raise ExportError("base_link collision box is missing")
    wheel = links.get("front_left_wheel_link")
    if wheel is None or "cylinder" not in _geometry_kinds(wheel, "collision"):
        raise ExportError("front_left wheel collision cylinder is missing")
    standoff = links.get("so_fl_a_standoff_link")
    if standoff is None or "cylinder" not in _geometry_kinds(standoff, "visual"):
        raise ExportError("standoff cylinder primitive is missing")
    for element in robot.iter("mesh"):
        filename = element.attrib.get("filename", "")
        if "://" in filename or filename.startswith("/") or "\\" in filename:
            raise ExportError(f"mesh path is not a browser-relative URI: {filename}")


def _folder_bytes(folder: Path) -> int:
    return sum(path.stat().st_size for path in folder.rglob("*") if path.is_file())


def _dump_manifest(path: Path, manifest: dict) -> None:
    path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def _settle_uncompressed(output_dir: Path, manifest: dict, manifest_path: Path) -> None:
    """Write manifest.json until its recorded folder size matches the folder."""
    for _ in range(6):
        uncompressed = _folder_bytes(output_dir)
        if manifest["bytes"]["uncompressed"] == uncompressed:
            return
        manifest["bytes"]["uncompressed"] = uncompressed
        _dump_manifest(manifest_path, manifest)
    raise ExportError("uncompressed size did not settle")


def _write_zip(folder: Path, zip_path: Path) -> int:
    if zip_path.exists():
        zip_path.unlink()
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(file for file in folder.rglob("*") if file.is_file()):
            archive.write(path, path.relative_to(folder).as_posix())
    return zip_path.stat().st_size


def _zip_output(
    output_dir: Path,
    manifest: dict,
    manifest_path: Path,
    zip_path: Path,
    budget: int,
) -> int:
    """Zip the folder once the manifest's uncompressed size matches the folder.

    The zip size is not written back into the folder: doing that changes the
    archive, so the recorded size would not be the file just built.
    """
    manifest["bytes"]["uncompressed"] = 0
    _dump_manifest(manifest_path, manifest)
    _settle_uncompressed(output_dir, manifest, manifest_path)
    zip_bytes = _write_zip(output_dir, zip_path)
    if zip_bytes > budget:
        raise ExportError(
            f"compressed zip size {zip_bytes} bytes exceeds budget {budget} bytes"
        )
    return zip_bytes


def _repo_relative(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def _wipe_output(output_dir: Path) -> None:
    resolved = output_dir.resolve()
    if resolved in {REPO_ROOT, PACKAGE_ROOT} or resolved in REPO_ROOT.parents:
        raise ExportError(f"refusing to wipe {resolved}")
    if resolved.exists():
        shutil.rmtree(resolved)
    resolved.mkdir(parents=True)


def export_web_model(
    output_dir: Path,
    *,
    xacro_path: Path = DEFAULT_XACRO,
    xacro_args: dict[str, str] | None = None,
    zip_budget_bytes: int = ZIP_BUDGET_BYTES,
) -> dict:
    """Expand the xacro, write the browser folder, and enforce the zip budget.

    Returns the manifest plus ``bytes.zip``. That zip size is not written into
    ``manifest.json``; it is the size of the archive built from the folder.
    """
    if zip_budget_bytes <= 0:
        raise ExportError("zip budget must be positive")
    output_dir = Path(output_dir)
    xacro_path = Path(xacro_path)
    if not xacro_path.is_file():
        raise ExportError(f"xacro not found: {xacro_path}")
    args = dict(WEB_XACRO_ARGS if xacro_args is None else xacro_args)
    _wipe_output(output_dir)
    robot = _expand_xacro(xacro_path, args)
    _drop_nominal_apriltag_mounts(robot)
    robot.insert(
        0,
        ET.Comment(
            " Browser model for three.js urdf-loader. "
            "Gazebo plugins removed. Mesh origins are copied from the xacro. "
            "Arm visuals are decimated SO-101 GLBs. Chassis plate and mecanum "
            "wheels are simplified procedural GLBs, not CAD. Standoffs and "
            "cameras stay box/cylinder primitives. Collision stays primitive. "
            "Nominal AprilTag cards stay in the Gazebo model only. "
        ),
    )
    _strip_gazebo(robot)
    visuals = mesh_visuals(robot)
    mesh_dir = output_dir / "meshes"
    meshes = _rewrite_meshes(
        robot,
        description_root=PACKAGE_ROOT,
        urdf_dir=xacro_path.parent,
        mesh_dir=mesh_dir,
    )
    joints = _joint_records(robot)
    _validate_tree(robot, joints, meshes, visuals)
    ET.indent(robot, space="  ")
    urdf_path = output_dir / URDF_NAME
    ET.ElementTree(robot).write(urdf_path, encoding="utf-8", xml_declaration=True)
    manifest = {
        "name": "omnibot-pro",
        "robot_name": robot.attrib.get("name"),
        "urdf": URDF_NAME,
        "xacro": _repo_relative(xacro_path),
        "xacro_args": args,
        "tooling": {
            "xacro": "PyPI xacro (no ROS install)",
            "meshes": "trimesh quadric decimation via fast-simplification, binary GLB",
        },
        "base_geometry": "simplified-mesh",
        "arm_geometry": "decimated-glb",
        "mesh_visuals": [
            {"link": item.link, "source": item.filename, "xyz": item.xyz, "rpy": item.rpy}
            for item in visuals
        ],
        "joints": joints,
        "arm_joints": list(ARM_JOINTS),
        "gripper_joint": GRIPPER_JOINT,
        "mecanum_wheel_joints": list(WHEEL_JOINTS),
        "meshes": [record._asdict() for record in meshes],
        "bytes": {"uncompressed": 0, "budget": zip_budget_bytes},
    }
    manifest_path = output_dir / "manifest.json"
    _dump_manifest(manifest_path, manifest)
    for record in meshes:
        if not (output_dir / record.file).is_file():
            raise ExportError(f"missing exported mesh {record.file}")
    urdf_text = urdf_path.read_text(encoding="utf-8")
    if "package://" in urdf_text or "<gazebo" in urdf_text:
        raise ExportError("URDF still contains package:// URIs or gazebo tags")
    zip_path = output_dir.parent / f"{output_dir.name}.zip"
    manifest["bytes"]["zip"] = _zip_output(
        output_dir, manifest, manifest_path, zip_path, zip_budget_bytes
    )
    return manifest


def _format_bytes(size: int) -> str:
    return f"{size} bytes ({size / (1024 * 1024):.2f} MiB)"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="folder for omnibot-pro.urdf, meshes/, and manifest.json",
    )
    parser.add_argument("--xacro", type=Path, default=DEFAULT_XACRO)
    parser.add_argument(
        "--zip-budget-bytes",
        type=int,
        default=ZIP_BUDGET_BYTES,
        help="fail when the zip of the output folder is larger than this",
    )
    args = parser.parse_args(argv)
    try:
        manifest = export_web_model(
            args.output,
            xacro_path=args.xacro,
            zip_budget_bytes=args.zip_budget_bytes,
        )
    except ExportError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    output_dir = args.output.resolve()
    zip_path = output_dir.parent / f"{output_dir.name}.zip"
    bytes_info = manifest["bytes"]
    print(f"omnibot-pro web model: {output_dir}")
    print(f"uncompressed: {_format_bytes(bytes_info['uncompressed'])}")
    print(f"zip: {_format_bytes(bytes_info['zip'])} ({zip_path})")
    print(f"budget: {_format_bytes(bytes_info['budget'])}")
    print("arm joints: " + ", ".join(manifest["arm_joints"]))
    print("gripper: " + manifest["gripper_joint"])
    print("mecanum wheel joints: " + ", ".join(manifest["mecanum_wheel_joints"]))
    print(f"mesh files: {len(manifest['meshes'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
