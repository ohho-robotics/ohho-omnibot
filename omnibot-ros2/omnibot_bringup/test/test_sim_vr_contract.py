# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Headless launch test for the VR teleop contract.

Runs only in the sim-smoke job (OMNIBOT_SIM_SMOKE=1). The gate now lives
in sim_teleop.launch.py; this test starts that file with vr:=true and
world:=flat so the drive is not blocked by apartment furniture. It checks:

- cmd_vel_mux, arm_cmd_mux, teleop_safety, rosbridge, and web_video_server
  are up once
- rosbridge and web_video_server bind 127.0.0.1
- /cmd_vel/teleop moves the base only while /control_mode is teleop
- /cmd_vel (the Nav2 source) does not move it in teleop, and does in nav2
- /emergency_stop true zeros /cmd_vel/out and the base stops
"""

import math
import os
import time
import unittest

import launch_testing
import rclpy
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Bool, String


REQUIRED_NODES = (
    "cmd_vel_mux",
    "arm_cmd_mux",
    "teleop_safety",
    "ros_gz_bridge",
    "robot_state_publisher",
    "rosbridge_websocket",
    "web_video_server",
)

REQUIRED_TOPICS = (
    "/cmd_vel/out",
    "/cmd_vel/teleop",
    "/cmd_vel/teleop_raw",
    "/cmd_vel",
    "/cmd_vel/vla",
    "/cmd_vel/rl",
    "/control_mode",
    "/control_mode/active",
    "/emergency_stop",
    "/ohho/heartbeat",
    "/ohho/safety/state",
    "/arm/joint_commands/teleop",
    "/arm/joint_commands/teleop_raw",
    "/arm/joint_commands/out",
    "/arm/joint_commands",
    "/odom",
    "/scan",
    "/imu",
    "/camera/front/image_raw",
)


def generate_test_description():
    bringup = FindPackageShare("omnibot_bringup").find("omnibot_bringup")
    # The gate moved to sim_teleop.launch.py. vr:=true still brings up the
    # Quest cameras and web_video_server through sim.launch.py.
    launch_file = os.path.join(bringup, "launch", "sim_teleop.launch.py")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "world": "flat",
                    "gui": "false",
                    "drive": "planar",
                    "vr": "true",
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {}


def _log(text):
    print(text, flush=True)


def _planar(twist):
    return math.hypot(twist.linear.x, twist.linear.y) + abs(twist.angular.z)


def _speed(odom):
    linear = odom.twist.twist.linear
    return math.hypot(linear.x, linear.y)


class TestSimVrContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("sim_vr_contract_test")
        cls.odom = []
        cls.out = []
        cls.modes = []
        cls.node.create_subscription(Odometry, "/odom", cls.odom.append, 10)
        cls.node.create_subscription(Twist, "/cmd_vel/out", cls.out.append, 10)
        cls.node.create_subscription(String, "/control_mode/active", cls.modes.append, 10)
        cls.teleop = cls.node.create_publisher(Twist, "/cmd_vel/teleop", 10)
        cls.nav = cls.node.create_publisher(Twist, "/cmd_vel", 10)
        cls.mode = cls.node.create_publisher(String, "/control_mode", 10)
        cls.estop = cls.node.create_publisher(Bool, "/emergency_stop", 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, seconds, ready):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if ready():
                return True
        return False

    def _node_names(self):
        names = []
        for name, _namespace in self.node.get_node_names_and_namespaces():
            names.append(name.split("/")[-1])
        return names

    def _topics(self):
        return {name for name, _types in self.node.get_topic_names_and_types()}

    def _dist(self, start):
        pos = self.odom[-1].pose.pose.position
        return math.hypot(pos.x - start.x, pos.y - start.y)

    def _publish_mode(self, name):
        msg = String()
        msg.data = name
        self.mode.publish(msg)

    def _twist(self, vx):
        msg = Twist()
        msg.linear.x = vx
        return msg

    def _pump(self, seconds, tick):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            tick()
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _until_tick(self, seconds, tick, ready):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            tick()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if ready():
                return True
        return False

    def _max_out_since(self, index):
        peak = 0.0
        for msg in self.out[index:]:
            peak = max(peak, _planar(msg))
        return peak

    def _peak_after_zero(self, index):
        """Peak planar command after the first zero in this slice."""
        seen_zero = False
        peak = 0.0
        for msg in self.out[index:]:
            planar = _planar(msg)
            if not seen_zero:
                if planar < 1e-3:
                    seen_zero = True
                continue
            peak = max(peak, planar)
        return seen_zero, peak

    def _address(self, node_name):
        client = self.node.create_client(
            GetParameters, f"/{node_name}/get_parameters"
        )
        self.assertTrue(
            client.wait_for_service(timeout_sec=15.0),
            f"/{node_name}/get_parameters did not appear",
        )
        request = GetParameters.Request()
        request.names = ["address"]
        future = client.call_async(request)
        self.assertTrue(
            self._spin_until(10.0, future.done),
            f"get_parameters address timed out on {node_name}",
        )
        result = future.result()
        self.assertIsNotNone(result)
        self.assertEqual(len(result.values), 1)
        return result.values[0].string_value

    def _wait_mode(self, name):
        ready = self._spin_until(
            5.0, lambda: self.modes and self.modes[-1].data == name
        )
        _log(
            f"EVIDENCE sim_vr mode want={name} "
            f"got={self.modes[-1].data if self.modes else ''}"
        )
        self.assertTrue(ready, f"/control_mode/active did not become {name}")

    def test_teleop_moves_only_in_teleop_and_estop_stops(self):
        nodes_ready = self._spin_until(
            45.0,
            lambda: bool(self.odom)
            and all(name in self._node_names() for name in REQUIRED_NODES)
            and all(topic in self._topics() for topic in REQUIRED_TOPICS),
        )
        names = self._node_names()
        topics = self._topics()
        missing_nodes = [name for name in REQUIRED_NODES if name not in names]
        missing_topics = [topic for topic in REQUIRED_TOPICS if topic not in topics]
        _log(
            "EVIDENCE sim_vr graph "
            f"missing_nodes={missing_nodes} missing_topics={missing_topics}"
        )
        self.assertTrue(nodes_ready, "sim VR nodes or topics did not appear")
        for name in REQUIRED_NODES:
            self.assertEqual(
                names.count(name),
                1,
                f"{name} should be started once, saw {names}",
            )

        self.assertEqual(self._address("rosbridge_websocket"), "127.0.0.1")
        self.assertEqual(self._address("web_video_server"), "127.0.0.1")

        self._wait_mode("teleop")
        start = self.odom[-1].pose.pose.position
        mark = len(self.out)
        nav_only = self._twist(0.2)
        self._pump(1.5, lambda: self.nav.publish(nav_only))
        leaked = self._dist(start)
        nav_peak = self._max_out_since(mark)
        _log(
            "EVIDENCE sim_vr teleop_holds_nav "
            f"displacement_m={leaked:.4f} out_peak={nav_peak:.4f}"
        )
        self.assertLess(leaked, 0.08)
        self.assertLess(nav_peak, 1e-3)

        start = self.odom[-1].pose.pose.position
        drive = self._twist(0.2)
        moved = False
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            self.teleop.publish(drive)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self._dist(start) >= 0.2:
                moved = True
                break
        _log(
            "EVIDENCE sim_vr teleop_motion "
            f"moved={moved} displacement_m={self._dist(start):.4f} "
            f"speed={_speed(self.odom[-1]):.4f}"
        )
        self.assertTrue(moved, "/cmd_vel/teleop did not move the robot 0.2 m")

        # The mux publishes one zero Twist in the mode callback, then
        # /control_mode/active. spin_once can return on that string while
        # the last teleop forward (still 0.2) is queued on /cmd_vel/out.
        # Same-topic order puts that forward before the zero, so the hold
        # window starts after the zero, not at the mode sample.
        switch = len(self.out)
        self._publish_mode("nav2")
        self._wait_mode("nav2")
        zeroed = self._spin_until(
            2.0, lambda: self._peak_after_zero(switch)[0]
        )
        mark = len(self.out)
        self._pump(2.0, lambda: self.teleop.publish(drive))
        held = self._max_out_since(mark)
        stopped = _speed(self.odom[-1]) < 0.05
        _log(
            "EVIDENCE sim_vr nav2_holds_teleop "
            f"zeroed={zeroed} out_peak={held:.4f} stopped={stopped} "
            f"speed={_speed(self.odom[-1]):.4f}"
        )
        self.assertTrue(zeroed, "mode switch did not zero /cmd_vel/out")
        self.assertLess(held, 1e-3)
        self.assertTrue(stopped, "base kept moving after leaving teleop")

        start = self.odom[-1].pose.pose.position
        mark = len(self.out)
        moved_nav = False
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            self.nav.publish(nav_only)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self._dist(start) >= 0.1:
                moved_nav = True
                break
        nav_out = self._max_out_since(mark)
        _log(
            "EVIDENCE sim_vr nav2_motion "
            f"moved={moved_nav} displacement_m={self._dist(start):.4f} "
            f"out_peak={nav_out:.4f}"
        )
        self.assertGreater(nav_out, 0.05)
        self.assertTrue(moved_nav, "/cmd_vel did not move the robot in nav2")

        stop = Bool()
        stop.data = True
        mark = len(self.out)

        def _estop_only():
            self.estop.publish(stop)

        def _estop_and_nav():
            self.estop.publish(stop)
            self.nav.publish(nav_only)

        zeroed = self._until_tick(
            2.0,
            _estop_only,
            lambda: self._peak_after_zero(mark)[0],
        )
        self._pump(1.0, _estop_and_nav)
        seen_zero, held_stop = self._peak_after_zero(mark)
        odom_stopped = _speed(self.odom[-1]) < 0.05
        if not odom_stopped:
            odom_stopped = self._until_tick(
                2.0, _estop_only, lambda: _speed(self.odom[-1]) < 0.05
            )
        _log(
            "EVIDENCE sim_vr estop "
            f"zeroed={zeroed} seen_zero={seen_zero} out_peak={held_stop:.4f} "
            f"odom_stopped={odom_stopped} speed={_speed(self.odom[-1]):.4f}"
        )
        self.assertTrue(seen_zero, "/emergency_stop did not zero /cmd_vel/out")
        self.assertLess(held_stop, 1e-3)
        self.assertTrue(odom_stopped, "/emergency_stop did not stop the base")
