# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Gazebo check: killing the teleop client stops the base within 350 ms.

Runs only in the sim-smoke job (OMNIBOT_SIM_SMOKE=1), same as test_sim_launch.
sim_teleop.launch.py starts teleop_safety and the muxes. Gazebo follows
/cmd_vel/out. The client publishes /cmd_vel/teleop_raw and /ohho/heartbeat,
then stops. vr:=true on sim.launch.py is the camera profile and is not
what arms this gate.
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
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger


def generate_test_description():
    bringup = FindPackageShare("omnibot_bringup").find("omnibot_bringup")
    launch_file = os.path.join(bringup, "launch", "sim_teleop.launch.py")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "world": "flat",
                    "gui": "false",
                    "drive": "planar",
                    "rosbridge": "false",
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {}


def _log(text):
    print(text, flush=True)


def _speed(odom):
    linear = odom.twist.twist.linear
    return math.hypot(linear.x, linear.y)


def _planar(twist):
    return math.hypot(twist.linear.x, twist.linear.y) + abs(twist.angular.z)


class TestTeleopSafetyWatchdog(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("teleop_safety_watchdog_test")
        cls.odom = []
        cls.teleop = []
        cls.states = []
        cls.node.create_subscription(Odometry, "/odom", cls.odom.append, 10)
        cls.node.create_subscription(Twist, "/cmd_vel/teleop", cls._teleop, 10)
        cls.node.create_subscription(String, "/ohho/safety/state", cls.states.append, 10)
        cls.raw = cls.node.create_publisher(Twist, "/cmd_vel/teleop_raw", 10)
        cls.heartbeat = cls.node.create_publisher(Empty, "/ohho/heartbeat", 10)
        cls.reset_client = cls.node.create_client(Trigger, "/ohho/estop_reset")

    @classmethod
    def _teleop(cls, msg):
        cls.teleop.append((time.monotonic(), msg))

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, deadline, ready):
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if ready():
                return True
        return False

    def test_client_kill_stops_base_within_350ms(self):
        ready = self._spin_until(
            time.monotonic() + 30.0,
            lambda: bool(self.odom),
        )
        _log(f"EVIDENCE teleop_safety odom_ready={ready} count={len(self.odom)}")
        self.assertTrue(self.odom, "/odom did not publish within 30 s")
        self.assertTrue(
            self.reset_client.wait_for_service(timeout_sec=30.0),
            "/ohho/estop_reset is not available",
        )
        future = self.reset_client.call_async(Trigger.Request())
        reset_done = self._spin_until(
            time.monotonic() + 5.0, lambda: future.done()
        )
        self.assertTrue(reset_done, "/ohho/estop_reset did not return")
        result = future.result()
        self.assertIsNotNone(result)
        self.assertTrue(result.success, result.message)

        drive = Twist()
        drive.linear.x = 0.2
        start = self.odom[-1].pose.pose.position
        moving = False
        # The 350 ms budget starts at the last command, not after spin_once.
        last_command_at = None
        drive_deadline = time.monotonic() + 20.0
        while time.monotonic() < drive_deadline:
            self.heartbeat.publish(Empty())
            self.raw.publish(drive)
            last_command_at = time.monotonic()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if not self.odom:
                continue
            pos = self.odom[-1].pose.pose.position
            dist = math.hypot(pos.x - start.x, pos.y - start.y)
            gated = _planar(self.teleop[-1][1]) if self.teleop else 0.0
            if dist >= 0.05 and _speed(self.odom[-1]) > 0.02 and gated > 0.02:
                moving = True
                break
        _log(
            "EVIDENCE teleop_safety drive "
            f"moving={moving} teleop_msgs={len(self.teleop)} "
            f"odom={len(self.odom)} speed={_speed(self.odom[-1]):.4f}"
        )
        self.assertTrue(moving, "base did not move on /cmd_vel/teleop_raw")
        self.assertIsNotNone(
            last_command_at,
            "drive loop exited without publishing a command",
        )

        t_kill = last_command_at
        odom_count_at_kill = len(self.odom)
        cmd_zero_at = None
        odom_stop_at = None
        deadline = t_kill + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if cmd_zero_at is None:
                for stamp, msg in reversed(self.teleop):
                    if stamp < t_kill:
                        break
                    if _planar(msg) <= 1e-6:
                        cmd_zero_at = stamp
                        break
            if (
                odom_stop_at is None
                and len(self.odom) > odom_count_at_kill
                and _speed(self.odom[-1]) < 0.02
            ):
                odom_stop_at = time.monotonic()
            if cmd_zero_at is not None and odom_stop_at is not None:
                break

        cmd_ms = None if cmd_zero_at is None else (cmd_zero_at - t_kill) * 1000.0
        odom_ms = None if odom_stop_at is None else (odom_stop_at - t_kill) * 1000.0
        state = self.states[-1].data if self.states else ""
        _log(
            "EVIDENCE teleop_safety client_kill "
            f"cmd_zero_ms={cmd_ms} odom_stop_ms={odom_ms} state={state}"
        )
        self.assertIsNotNone(cmd_zero_at, "gated /cmd_vel/teleop did not go to zero")
        self.assertLessEqual(cmd_zero_at - t_kill, 0.350)
        self.assertIsNotNone(odom_stop_at, "base odom did not stop")
        self.assertLessEqual(
            odom_stop_at - t_kill,
            0.350,
            "base did not stop within 350 ms of the client dying",
        )
