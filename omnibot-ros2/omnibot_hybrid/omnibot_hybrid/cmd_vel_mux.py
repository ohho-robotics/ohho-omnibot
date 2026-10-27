#!/usr/bin/env python3
"""
cmd_vel Multiplexer for OmniBot Hybrid Control.

Selects one cmd_vel source based on the active control mode and
forwards it to /cmd_vel/out, which the robot driver reads.

Topic routing
─────────────
  Inputs (one active at a time):
    /cmd_vel          ← Nav2 velocity_smoother (native Nav2 output)
    /cmd_vel/vla      ← VLA inference node
    /cmd_vel/teleop   ← teleop_safety (clients publish /cmd_vel/teleop_raw)
    /cmd_vel/rl       ← Isaac Lab RL navigation policy (rl_nav_node)

  Mode control:
    /control_mode     ← std_msgs/String  "nav2" | "vla" | "teleop" | "rl_nav"
    /emergency_stop   ← std_msgs/Bool    true holds every source and zeros output

  Staleness:
    source_timeout_s  default 0.3 s (may be lowered; longer values clamp to
                      0.3 s; 0 or negative keeps 0.3 s). When the active
                      source goes silent, one zero Twist is published on the
                      next staleness tick at or after the timeout.

  Output:
    /cmd_vel/out      → robot driver (remapped from /cmd_vel in hybrid launch)

  Feedback:
    /control_mode/active  → currently selected mode (published at 1 Hz)

Usage
─────
  ros2 topic pub /control_mode std_msgs/msg/String "data: 'vla'"
  ros2 topic pub /control_mode std_msgs/msg/String "data: 'nav2'"
  ros2 topic pub /control_mode std_msgs/msg/String "data: 'teleop'"
  ros2 topic pub /control_mode std_msgs/msg/String "data: 'rl_nav'"
"""

import time

from omnibot_hybrid.stream_gate import (
    DEFAULT_SOURCE_TIMEOUT_S,
    StreamGate,
    normalize_mode,
)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

# How often the active source is checked for silence. The zero Twist is
# published on the first tick at or after source_timeout_s, so it is out
# by at most one tick after the timeout.
SOURCE_STALE_TICK_S = 0.05


class CmdVelMux(Node):
    """
    Mode-based cmd_vel multiplexer.

    Parameters
    ----------
    default_mode : str
        Starting mode before any /control_mode message arrives.
        One of "teleop" (default), "nav2", "vla", "rl_nav".
        An unknown value falls back to teleop.
    source_timeout_s : float
        Silence after which the active source is stale. Default 0.3 s.
        Lower positive values are kept. Values above 0.3 s clamp to 0.3 s.
        Zero, negative, and non-numeric values keep 0.3 s.
    """

    VALID_MODES = ("nav2", "vla", "teleop", "rl_nav")
    SOURCE_STALE_TICK_S = SOURCE_STALE_TICK_S

    def __init__(self):
        super().__init__("cmd_vel_mux")

        self.declare_parameter("default_mode", "teleop")
        self.declare_parameter("source_timeout_s", DEFAULT_SOURCE_TIMEOUT_S)
        requested = self.get_parameter("default_mode").value
        raw_timeout = self.get_parameter("source_timeout_s").value
        self._gate = StreamGate(
            requested if isinstance(requested, str) else "",
            source_timeout_s=raw_timeout,
        )
        if self._gate.source_timeout_warning:
            self.get_logger().warn(self._gate.source_timeout_warning)
        if not isinstance(requested, str) or normalize_mode(requested) is None:
            self.get_logger().warn(
                f'Invalid default_mode "{requested}" — using "{self._gate.mode}".'
            )

        # ── Inputs ────────────────────────────────────────────────────────────
        self.create_subscription(Twist, "/cmd_vel", self._nav2_cb, 10)
        self.create_subscription(Twist, "/cmd_vel/vla", self._vla_cb, 10)
        self.create_subscription(Twist, "/cmd_vel/teleop", self._teleop_cb, 10)
        self.create_subscription(Twist, "/cmd_vel/rl", self._rl_nav_cb, 10)
        self.create_subscription(String, "/control_mode", self._mode_cb, 10)
        self.create_subscription(Bool, "/emergency_stop", self._estop_cb, 10)

        # ── Output ────────────────────────────────────────────────────────────
        self._out_pub = self.create_publisher(Twist, "/cmd_vel/out", 10)
        self._active_mode_pub = self.create_publisher(
            String, "/control_mode/active", 10
        )

        # Publish active mode at 1 Hz so other nodes can query it
        self.create_timer(1.0, self._publish_active_mode)
        self.create_timer(self.SOURCE_STALE_TICK_S, self._check_source_staleness)

        self.get_logger().info(
            f'CmdVelMux ready. Default mode: "{self._gate.mode}". '
            f"Source timeout: {self._gate.source_timeout_s:.3f}s. "
            f"Valid modes: {self.VALID_MODES}"
        )

    @property
    def _active_mode(self) -> str:
        return self._gate.mode

    @_active_mode.setter
    def _active_mode(self, value: str) -> None:
        self._gate.mode = value

    # ── Mode switch ───────────────────────────────────────────────────────────

    def _mode_cb(self, msg: String) -> None:
        previous = self._gate.mode
        if not self._gate.set_mode(msg.data):
            shown = msg.data.strip().lower() if isinstance(msg.data, str) else msg.data
            self.get_logger().warn(
                f'Unknown mode "{shown}" ignored. Valid: {self.VALID_MODES}'
            )
            return
        if self._gate.take_base_stop():
            self._publish_zero_twist()
        if self._gate.mode != previous:
            self.get_logger().info(
                f"[CmdVelMux] Mode switch: {previous} → {self._gate.mode}"
            )
            self._publish_active_mode()

    def _estop_cb(self, msg: Bool) -> None:
        self._gate.set_emergency_stop(msg.data)
        if self._gate.take_base_stop():
            self.get_logger().warn("Emergency stop — zeroing /cmd_vel/out.")
            self._publish_zero_twist()
        elif not msg.data:
            self.get_logger().info("Emergency stop cleared.")

    def _publish_zero_twist(self) -> None:
        self._out_pub.publish(Twist())

    def _now(self) -> float:
        """Monotonic seconds. A stalled ROS clock cannot freeze this watchdog."""
        return time.monotonic()

    def _check_source_staleness(self) -> None:
        """Publish one zero Twist when the active source has gone silent."""
        if not self._gate.poll_staleness(self._now()):
            return
        self.get_logger().warn(
            f'Active source "{self._gate.mode}" silent for '
            f"{self._gate.source_timeout_s:.3f}s — zeroing /cmd_vel/out."
        )
        self._publish_zero_twist()

    def _forward(self, source: str, msg: Twist) -> None:
        if self._gate.base_allows(source):
            self._gate.note_forward(source, self._now())
            self._out_pub.publish(msg)

    # ── Source callbacks ──────────────────────────────────────────────────────

    def _nav2_cb(self, msg: Twist) -> None:
        self._forward("nav2", msg)

    def _vla_cb(self, msg: Twist) -> None:
        self._forward("vla", msg)

    def _teleop_cb(self, msg: Twist) -> None:
        self._forward("teleop", msg)

    def _rl_nav_cb(self, msg: Twist) -> None:
        self._forward("rl_nav", msg)

    # ── Periodic feedback ─────────────────────────────────────────────────────

    def _publish_active_mode(self) -> None:
        msg = String()
        msg.data = self._active_mode
        self._active_mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
