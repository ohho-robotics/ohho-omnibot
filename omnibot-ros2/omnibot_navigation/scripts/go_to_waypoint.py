#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Send Nav2 to named waypoints and report time-to-goal.

    ros2 run omnibot_navigation go_to_waypoint.py kitchen
    ros2 run omnibot_navigation go_to_waypoint.py kitchen desk door --timeout 60

Waypoints come from config/waypoints.yaml (map frame). For each name the tool
sends a NavigateToPose goal, waits for Nav2 to finish, then checks the final
pose against /odom. In the Gazebo planar-drive sim /odom is the exact world
pose and the saved map frame equals the world frame, so /odom is ground truth
for the goal-tolerance check (maps/README.md).

Time-to-goal is measured on the node clock from goal acceptance to the Nav2
result. With --use-sim-time that is Gazebo simulation time (the default for the
sim). The run fails if Nav2 does not succeed in --timeout seconds, or if it
succeeds but the final pose is outside the tolerances below. One
`NAV_RESULT {json}` line is printed per waypoint, and appended to
--results-file when given. Exit code 0 only if every waypoint passed.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
import rclpy.utilities
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from waypoint_loader import (
    Waypoint,
    angle_diff,
    load_waypoint_file,
    resolve_waypoint_path,
    yaw_from_quaternion,
)

# Independent check against ground truth, looser than Nav2's own goal checker
# (config/nav2_params.yaml general_goal_checker) to allow for localisation error.
GOAL_XY_TOLERANCE_M = 0.30
GOAL_YAW_TOLERANCE_RAD = 0.50
DEFAULT_TIMEOUT_S = 60.0
START_POSE_TOLERANCE_M = 0.15
# Lidar proximity proxy for regression reporting (not Gazebo contact events).
COLLISION_SCAN_RANGE_M = 0.20

LIFECYCLE_MANAGERS = (
    "lifecycle_manager_localization/is_active",
    "lifecycle_manager_navigation/is_active",
)


class WaypointGoalClient(Node):
    def __init__(self, use_sim_time: bool = True):
        super().__init__(
            "go_to_waypoint",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)
            ],
        )
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.odom = None
        self._collision_proximity_hits = 0
        self._track_collisions = False
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_odom(self, msg: Odometry):
        self.odom = msg

    def _on_scan(self, msg: LaserScan):
        if not self._track_collisions:
            return
        for range_m in msg.ranges:
            if msg.range_min < range_m < COLLISION_SCAN_RANGE_M:
                self._collision_proximity_hits += 1
                break

    # -- helpers ---------------------------------------------------------
    def spin_for(self, seconds_wall: float):
        end = time.monotonic() + seconds_wall
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def sim_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def ground_truth(self):
        """(x, y, yaw) from /odom, or None before the first message."""
        if self.odom is None:
            return None
        pose = self.odom.pose.pose
        q = pose.orientation
        return (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )

    def wait_until_ready(self, timeout_wall_s: float = 180.0) -> bool:
        """Sim clock running, /odom flowing, map->base_link TF, Nav2 active."""
        deadline = time.monotonic() + timeout_wall_s
        services = [self.create_client(Trigger, name) for name in LIFECYCLE_MANAGERS]
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.sim_seconds() <= 0.0 or self.odom is None:
                continue
            try:
                self.tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time()
                )
            except TransformException:
                continue
            if not self.client.server_is_ready():
                continue
            if all(self._is_active(service) for service in services):
                return True
        return False

    def _is_active(self, service) -> bool:
        if not service.service_is_ready():
            return False
        future = service.call_async(Trigger.Request())
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(future.done() and future.result() and future.result().success)

    # -- one goal --------------------------------------------------------
    def navigate_to(self, waypoint: Waypoint, timeout_s: float = DEFAULT_TIMEOUT_S):
        result = {
            "waypoint": waypoint.name,
            "goal": {"x": waypoint.x, "y": waypoint.y, "yaw": round(waypoint.yaw, 4)},
            "timeout_s": timeout_s,
            "xy_tolerance_m": GOAL_XY_TOLERANCE_M,
            "yaw_tolerance_rad": GOAL_YAW_TOLERANCE_RAD,
            "success": False,
            "reason": "",
        }
        start = self.ground_truth()
        if start is not None:
            result["start_pose"] = {
                "x": round(start[0], 3),
                "y": round(start[1], 3),
                "yaw": round(start[2], 3),
            }

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = waypoint.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint.x
        goal.pose.pose.position.y = waypoint.y
        goal.pose.pose.position.z = waypoint.z
        goal.pose.pose.orientation.x = waypoint.qx
        goal.pose.pose.orientation.y = waypoint.qy
        goal.pose.pose.orientation.z = waypoint.qz
        goal.pose.pose.orientation.w = waypoint.qw

        send_future = self.client.send_goal_async(goal)
        wall_start = time.monotonic()
        while not send_future.done() and time.monotonic() - wall_start < 15.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not send_future.done():
            result["reason"] = "goal request timed out"
            return result
        handle = send_future.result()
        if not handle.accepted:
            result["reason"] = "goal rejected by Nav2"
            return result

        self._collision_proximity_hits = 0
        self._track_collisions = True
        sim_start = self.sim_seconds()
        wall_start = time.monotonic()
        result_future = handle.get_result_async()
        # Wall guard only (sim-time limit is timeout_s). Headless CI can run <<1x RTF.
        wall_limit = max(12.0 * timeout_s, timeout_s + 540.0)
        timed_out = False
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.sim_seconds() - sim_start > timeout_s:
                timed_out = True
                break
            if time.monotonic() - wall_start > wall_limit:
                timed_out = True
                result["reason"] = "wall-clock guard hit (sim clock stalled?)"
                break

        self._track_collisions = False
        result["collision_proximity_hits"] = self._collision_proximity_hits
        if timed_out:
            cancel = handle.cancel_goal_async()
            end = time.monotonic() + 5.0
            while not cancel.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            elapsed = self.sim_seconds() - sim_start
            result["time_to_goal_s"] = None
            result["elapsed_sim_s"] = round(elapsed, 2)
            result["reason"] = result["reason"] or f"not reached within {timeout_s:.0f} s"
            self._attach_final_pose(result, waypoint)
            return result

        sim_elapsed = self.sim_seconds() - sim_start
        wall_elapsed = time.monotonic() - wall_start
        wrapped = result_future.result()
        status = wrapped.status
        result["nav2_status"] = status
        result["time_to_goal_s"] = round(sim_elapsed, 2)
        result["wall_time_s"] = round(wall_elapsed, 2)
        result["realtime_factor"] = (
            round(sim_elapsed / wall_elapsed, 2) if wall_elapsed > 0 else None
        )
        self.spin_for(0.5)  # let /odom reflect the stopped robot
        self._attach_final_pose(result, waypoint)

        if status != GoalStatus.STATUS_SUCCEEDED:
            result["reason"] = f"Nav2 finished with status {status}"
            result["time_to_goal_s"] = None
            result["elapsed_sim_s"] = round(sim_elapsed, 2)
            return result
        if sim_elapsed > timeout_s:
            result["reason"] = f"reached in {sim_elapsed:.1f} s, over {timeout_s:.0f} s"
            return result
        if result["final_error_xy_m"] is None:
            result["reason"] = "no /odom to verify the final pose"
            return result
        if result["final_error_xy_m"] > GOAL_XY_TOLERANCE_M:
            result["reason"] = (
                f"final position error {result['final_error_xy_m']:.2f} m "
                f"> {GOAL_XY_TOLERANCE_M} m"
            )
            return result
        if result["final_error_yaw_rad"] > GOAL_YAW_TOLERANCE_RAD:
            result["reason"] = (
                f"final yaw error {result['final_error_yaw_rad']:.2f} rad "
                f"> {GOAL_YAW_TOLERANCE_RAD} rad"
            )
            return result
        result["success"] = True
        result["reason"] = "reached"
        return result

    def _attach_final_pose(self, result: dict, waypoint: Waypoint):
        truth = self.ground_truth()
        if truth is None:
            result["final_error_xy_m"] = None
            result["final_error_yaw_rad"] = None
            return
        result["final_pose"] = {
            "x": round(truth[0], 3),
            "y": round(truth[1], 3),
            "yaw": round(truth[2], 3),
        }
        result["final_error_xy_m"] = round(
            math.hypot(truth[0] - waypoint.x, truth[1] - waypoint.y), 3
        )
        result["final_error_yaw_rad"] = round(
            abs(angle_diff(truth[2], waypoint.yaw)), 3
        )


def emit(result: dict, results_file: str | None):
    line = json.dumps(result, sort_keys=True)
    print(f"NAV_RESULT {line}", flush=True)
    if results_file:
        with open(results_file, "a", encoding="utf-8") as handle:
            handle.write(line + "\n")


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("names", nargs="+", help="waypoint names, e.g. kitchen desk")
    parser.add_argument("--waypoint-file", default="waypoints.yaml")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help="seconds per waypoint on the node clock (default 60)")
    parser.add_argument("--ready-timeout", type=float, default=180.0,
                        help="wall seconds to wait for sim + Nav2 (default 180)")
    parser.add_argument("--no-sim-time", action="store_true",
                        help="use the wall clock (real robot)")
    parser.add_argument("--results-file", default=None,
                        help="append one JSON line per waypoint")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:])
    _, waypoints = load_waypoint_file(resolve_waypoint_path(args.waypoint_file))
    unknown = [n for n in args.names if n not in waypoints]
    if unknown:
        print(f"unknown waypoint(s) {unknown}; known: {sorted(waypoints)}", file=sys.stderr)
        return 2

    rclpy.init(args=argv)
    node = WaypointGoalClient(use_sim_time=not args.no_sim_time)
    exit_code = 0
    try:
        if not node.wait_until_ready(args.ready_timeout):
            emit({"success": False, "reason": "sim or Nav2 not ready"}, args.results_file)
            return 1
        for name in args.names:
            outcome = node.navigate_to(waypoints[name], args.timeout)
            emit(outcome, args.results_file)
            if not outcome["success"]:
                exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
