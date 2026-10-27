# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""ROS-free checks of the Nav2 holonomic params and the checked-in sim map.

Runs under plain pytest (the CPU CI job) and under colcon test. It reads the
files only; it does not prove navigation works. test_nav_sim_scenarios.py does
that in Gazebo.
"""

import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))

import waypoint_loader as wl  # noqa: E402

CONFIG = PACKAGE / "config"
MAPS = PACKAGE / "maps"
APARTMENT_SDF = PACKAGE.parent / "omnibot_bringup" / "worlds" / "apartment.sdf"
GO_TO_WAYPOINT = PACKAGE / "scripts" / "go_to_waypoint.py"


def _yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def nav2():
    return _yaml(CONFIG / "nav2_params.yaml")


# -- Nav2 params: holonomic mecanum base --------------------------------------


def test_dwb_kept_with_holonomic_settings(nav2):
    ctrl = nav2["controller_server"]["ros__parameters"]
    follow = ctrl["FollowPath"]
    assert follow["plugin"] == "dwb_core::DWBLocalPlanner"
    # Lateral motion is allowed in both directions and sampled densely.
    assert follow["max_vel_y"] > 0.0 > follow["min_vel_y"]
    assert follow["vy_samples"] >= 5
    assert follow["acc_lim_y"] > 0.0 and follow["decel_lim_y"] < 0.0


def test_controller_does_not_zero_lateral_commands(nav2):
    """min_y_velocity_threshold used to be 0.5 > max_vel_y, dropping every strafe."""
    ctrl = nav2["controller_server"]["ros__parameters"]
    assert ctrl["min_y_velocity_threshold"] <= 0.01
    assert ctrl["min_y_velocity_threshold"] < ctrl["FollowPath"]["max_vel_y"]


def test_velocity_smoother_passes_vy(nav2):
    smoother = nav2["velocity_smoother"]["ros__parameters"]
    assert smoother["max_velocity"][1] > 0.0 > smoother["min_velocity"][1]
    assert smoother["max_accel"][1] > 0.0 and smoother["max_decel"][1] < 0.0


def test_speed_caps_match_driver_clamp(nav2):
    follow = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = nav2["velocity_smoother"]["ros__parameters"]
    assert follow["max_speed_xy"] <= 0.20 + 1e-9
    assert smoother["max_velocity"][0] <= 0.20 + 1e-9
    assert smoother["max_velocity"][1] <= 0.20 + 1e-9


def test_amcl_uses_omnidirectional_model(nav2):
    amcl = nav2["amcl"]["ros__parameters"]
    # Jazzy's class name; the older ...OmnidirectionalMotionModel does not exist.
    assert amcl["robot_model_type"] == "nav2_amcl::OmniMotionModel"


def test_goal_tolerances_are_consistent_and_documented(nav2):
    ctrl = nav2["controller_server"]["ros__parameters"]
    checker = ctrl["general_goal_checker"]
    follow = ctrl["FollowPath"]
    assert follow["xy_goal_tolerance"] == checker["xy_goal_tolerance"]
    # The integration test's independent tolerance must be looser than Nav2's.
    source = GO_TO_WAYPOINT.read_text(encoding="utf-8")
    xy = float(re.search(r"GOAL_XY_TOLERANCE_M\s*=\s*([\d.]+)", source).group(1))
    yaw = float(re.search(r"GOAL_YAW_TOLERANCE_RAD\s*=\s*([\d.]+)", source).group(1))
    assert checker["xy_goal_tolerance"] < xy
    assert checker["yaw_goal_tolerance"] < yaw


def test_costmaps_use_lidar_and_a_footprint(nav2):
    local = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    glob = nav2["global_costmap"]["global_costmap"]["ros__parameters"]
    assert "scan" in local["voxel_layer"]["observation_sources"].split()
    assert "scan" in glob["obstacle_layer"]["observation_sources"].split()
    assert "footprint" in local and "footprint" in glob


def test_no_sim_unfriendly_defaults(nav2):
    assert nav2["bt_navigator"]["ros__parameters"]["enable_groot_monitoring"] is False


# -- slam_toolbox ---------------------------------------------------------------


def test_slam_toolbox_maps_from_scratch():
    slam = _yaml(CONFIG / "slam_toolbox_params.yaml")["slam_toolbox"]["ros__parameters"]
    assert slam["mode"] == "mapping"
    # A non-empty map_file_name makes slam_toolbox try to load that pose graph at start.
    assert slam["map_file_name"] == ""
    assert slam["scan_topic"] == "/scan"


# -- Saved sim map --------------------------------------------------------------


def _read_pgm(path):
    data = path.read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos : pos + 1].isspace():
            pos += 1
        if data[pos : pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end : end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    magic, width, height, maxval = tokens
    assert magic == b"P5" and int(maxval) == 255
    pixels = np.frombuffer(data[pos + 1 :], dtype=np.uint8)
    assert pixels.size == int(width) * int(height)
    return pixels.reshape(int(height), int(width))


def load_sim_map_grid():
    meta = _yaml(MAPS / "apartment.yaml")
    pixels = _read_pgm(MAPS / meta["image"])
    res = float(meta["resolution"])
    ox, oy, oyaw = (float(v) for v in meta["origin"])
    assert oyaw == 0.0
    occupied_thresh = float(meta["occupied_thresh"])
    free_thresh = float(meta["free_thresh"])
    prob = (255.0 - pixels.astype(float)) / 255.0 if not meta.get("negate") else pixels / 255.0
    height, width = pixels.shape

    class Grid:
        resolution = res
        occupied = prob > occupied_thresh
        free = prob < free_thresh

        @staticmethod
        def cell(x, y):
            col = int(math.floor((x - ox) / res))
            row = height - 1 - int(math.floor((y - oy) / res))
            return row, col

        @staticmethod
        def world(row, col):
            return ox + (col + 0.5) * res, oy + (height - 1 - row + 0.5) * res

    Grid.meta = meta
    Grid.shape = (height, width)
    return Grid


@pytest.fixture(scope="module")
def sim_map():
    return load_sim_map_grid()


def test_map_files_checked_in():
    for name in ("apartment.yaml", "apartment.pgm", "apartment.posegraph", "apartment.data"):
        assert (MAPS / name).is_file() and (MAPS / name).stat().st_size > 0, name


def test_map_yaml_fields(sim_map):
    meta = sim_map.meta
    assert meta["image"] == "apartment.pgm"
    assert meta["resolution"] == pytest.approx(0.05)
    assert meta["mode"] == "trinary"


def _occupied_within(sim_map, x, y, radius):
    r0, c0 = sim_map.cell(x, y)
    span = int(math.ceil(radius / sim_map.resolution))
    rows, cols = sim_map.shape
    for row in range(max(r0 - span, 0), min(r0 + span + 1, rows)):
        for col in range(max(c0 - span, 0), min(c0 + span + 1, cols)):
            if sim_map.occupied[row, col]:
                wx, wy = sim_map.world(row, col)
                if math.hypot(wx - x, wy - y) <= radius:
                    return True
    return False


def test_map_frame_is_the_gazebo_world_frame(sim_map):
    """Walls and furniture sit where apartment.sdf puts them (within 0.15 m)."""
    # Inner wall faces: x = +-3.425, y = +-2.425. Furniture faces from apartment.sdf.
    probes = {
        "north wall": (0.0, 2.425),
        "east wall": (3.425, 0.0),
        "west wall (south of counter)": (-3.425, -2.0),
        "south wall, west of the door": (-1.5, -2.425),
        "south wall, east of the door": (1.5, -2.425),
        "kitchen counter east face": (-2.65, 0.7),
        "fridge east face": (-2.55, -1.15),
        "desk south face": (2.55, 0.1),
        "desk west face": (1.95, 0.4),
    }
    for label, (x, y) in probes.items():
        assert _occupied_within(sim_map, x, y, 0.15), f"{label} missing near ({x}, {y})"


def test_map_extent_matches_the_room(sim_map):
    rows, cols = np.nonzero(sim_map.occupied)
    xs = [sim_map.world(r, c)[0] for r, c in zip(rows, cols)]
    ys = [sim_map.world(r, c)[1] for r, c in zip(rows, cols)]
    assert min(xs) == pytest.approx(-3.425, abs=0.2) and max(xs) == pytest.approx(3.425, abs=0.2)
    assert max(ys) == pytest.approx(2.425, abs=0.2)
    # The door leaf is ajar and pokes out of the south wall down to y = -2.745.
    assert min(ys) == pytest.approx(-2.745, abs=0.2)


def test_start_pose_and_waypoints_are_in_free_space(sim_map):
    initial = wl.load_initial_pose(CONFIG / "waypoints.yaml")
    points = {"start": initial[:2]}
    points.update(
        {w.name: (w.x, w.y) for w in wl.load_waypoints(CONFIG / "waypoints.yaml").values()}
    )
    for name, (x, y) in points.items():
        row, col = sim_map.cell(x, y)
        assert sim_map.free[row, col], f"{name} ({x}, {y}) is not free in the saved map"
        assert not _occupied_within(sim_map, x, y, 0.35), f"{name} is within 0.35 m of an obstacle"


def test_waypoints_are_reachable_from_start_in_the_saved_map(sim_map):
    """4-connected flood fill over free cells, with obstacles inflated by 0.2 m."""
    from collections import deque

    blocked = sim_map.occupied.copy()
    grow = int(math.ceil(0.2 / sim_map.resolution))
    rows, cols = np.nonzero(sim_map.occupied)
    for r, c in zip(rows, cols):
        blocked[max(r - grow, 0) : r + grow + 1, max(c - grow, 0) : c + grow + 1] = True
    passable = sim_map.free & ~blocked

    start = sim_map.cell(*wl.load_initial_pose(CONFIG / "waypoints.yaml")[:2])
    seen = {start}
    queue = deque([start])
    while queue:
        r, c = queue.popleft()
        for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if (
                0 <= nr < passable.shape[0]
                and 0 <= nc < passable.shape[1]
                and passable[nr, nc]
                and (nr, nc) not in seen
            ):
                seen.add((nr, nc))
                queue.append((nr, nc))
    for waypoint in wl.load_waypoints(CONFIG / "waypoints.yaml").values():
        assert sim_map.cell(waypoint.x, waypoint.y) in seen, waypoint.name
