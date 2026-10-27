# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""ROS-free checks of config/waypoints.yaml and scripts/waypoint_loader.py.

Runs under plain pytest (the CPU CI job) and under colcon test. No ROS, no
Gazebo, no robot.
"""

import copy
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))

import waypoint_loader as wl  # noqa: E402

WAYPOINTS_YAML = PACKAGE / "config" / "waypoints.yaml"
APARTMENT_SDF = PACKAGE.parent / "omnibot_bringup" / "worlds" / "apartment.sdf"
SIM_LAUNCH = PACKAGE.parent / "omnibot_bringup" / "launch" / "sim.launch.py"
NAV_SIM_LAUNCH = PACKAGE / "launch" / "sim_navigation.launch.py"

# Robot footprint circumradius used in nav2_params.yaml is about 0.21 m; a
# waypoint must keep at least this much room to every furniture box.
MIN_CLEARANCE_M = 0.45


def _raw():
    return yaml.safe_load(WAYPOINTS_YAML.read_text(encoding="utf-8"))


def test_named_waypoints_not_generic():
    waypoints = wl.load_waypoints(WAYPOINTS_YAML)
    assert set(waypoints) == {"kitchen", "desk", "door"}
    assert not any(re.fullmatch(r"waypoint_\d+", name) for name in waypoints)


def test_waypoints_are_in_map_frame():
    data, waypoints = wl.load_waypoint_file(WAYPOINTS_YAML)
    assert data["frame_id"] == "map"
    assert all(w.frame_id == "map" for w in waypoints.values())


def test_yaw_deg_matches_quaternion():
    waypoints = wl.load_waypoints(WAYPOINTS_YAML)
    expected = {"kitchen": 180.0, "desk": 90.0, "door": -90.0}
    for name, degrees in expected.items():
        assert abs(wl.angle_diff(waypoints[name].yaw, math.radians(degrees))) < 1e-3


def test_initial_pose_matches_sim_spawn():
    x, y, yaw = wl.load_initial_pose(WAYPOINTS_YAML)
    spawn = SIM_LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("spawn_x", default_value="0")' in spawn
    assert 'DeclareLaunchArgument("spawn_y", default_value="0")' in spawn
    assert (x, y, yaw) == (0.0, 0.0, 0.0)
    launch = NAV_SIM_LAUNCH.read_text(encoding="utf-8")
    assert re.search(r'DEFAULT_INITIAL_POSE\s*=\s*\{"x": 0\.0, "y": 0\.0, "yaw": 0\.0\}', launch)


def _boxes_from_sdf():
    """(name, x, y, yaw, size_x, size_y) for every static box collision."""
    root = ET.parse(APARTMENT_SDF).getroot()
    boxes = []
    for model in root.iter("model"):
        pose_text = (model.findtext("pose") or "0 0 0 0 0 0").split()
        x, y, _, _, _, yaw = (float(v) for v in pose_text)
        for size in model.iter("size"):
            parent = [p for p in model.iter("box") if size in list(p)]
            if parent:
                sx, sy, _ = (float(v) for v in size.text.split())
                boxes.append((model.get("name"), x, y, yaw, sx, sy))
                break  # one collision box per model here
    return boxes


def _distance_to_box(px, py, box):
    _, bx, by, yaw, sx, sy = box
    dx, dy = px - bx, py - by
    lx = math.cos(yaw) * dx + math.sin(yaw) * dy
    ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
    ox = max(abs(lx) - sx / 2.0, 0.0)
    oy = max(abs(ly) - sy / 2.0, 0.0)
    return math.hypot(ox, oy)


def test_waypoints_clear_of_apartment_furniture():
    boxes = _boxes_from_sdf()
    assert len(boxes) >= 8, "apartment.sdf parse found too few boxes"
    for name, waypoint in wl.load_waypoints(WAYPOINTS_YAML).items():
        for box in boxes:
            gap = _distance_to_box(waypoint.x, waypoint.y, box)
            assert gap >= MIN_CLEARANCE_M, (
                f"{name} is only {gap:.2f} m from {box[0]} in apartment.sdf"
            )


def test_waypoints_inside_room():
    for name, waypoint in wl.load_waypoints(WAYPOINTS_YAML).items():
        assert abs(waypoint.x) < 3.4 and abs(waypoint.y) < 2.4, name


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda d: d["waypoints"].append(copy.deepcopy(d["waypoints"][0])), "duplicate"),
        (lambda d: d["waypoints"][0].pop("name"), "no name"),
        (lambda d: d["waypoints"][0]["orientation"].update(w=0.2), "unit length"),
        (lambda d: d["waypoints"][0].update(yaw_deg=10.0), "disagrees"),
        (lambda d: d["waypoints"][0]["position"].pop("x"), "missing"),
    ],
)
def test_loader_rejects_bad_files(mutate, message):
    data = _raw()
    mutate(data)
    with pytest.raises(ValueError, match=message):
        wl.parse_waypoints(data)


def test_select_route_order_and_unknown_name():
    waypoints = wl.load_waypoints(WAYPOINTS_YAML)
    assert [w.name for w in wl.select_route(waypoints, ["door", "kitchen"])] == [
        "door",
        "kitchen",
    ]
    assert [w.name for w in wl.select_route(waypoints, [])] == ["kitchen", "desk", "door"]
    with pytest.raises(KeyError):
        wl.select_route(waypoints, ["bathroom"])


def test_default_waypoint_path_finds_the_waypoints_file():
    found = wl.default_waypoint_path()
    assert found.read_text(encoding="utf-8") == WAYPOINTS_YAML.read_text(encoding="utf-8")
