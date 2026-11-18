#!/usr/bin/env python3
"""Robot-side SafetyGate between human teleop and the muxes.

Clients (Quest and Android) publish:

    /cmd_vel/teleop_raw
    /arm/joint_commands/teleop_raw
    /ohho/heartbeat          std_msgs/Empty, 10 Hz
    /emergency_stop          std_msgs/Bool

This node publishes:

    /cmd_vel/teleop
    /arm/joint_commands/teleop
    /ohho/safety/state       std_msgs/String JSON
                             {"armed", "latched", "reason"}

``/ohho/estop_reset`` (std_srvs/Trigger) is the only way to clear a latched
e-stop. The gate starts latched, so a restart stays stopped until that
service succeeds. Publishing ``/emergency_stop`` false does not clear it.
The service refuses while the stop is still asserted.

Caps are the OmniBot ceilings in ``teleop_gate`` (0.2 m/s, 1.0 rad/s). This
node declares no speed, acceleration, or timeout parameter, so a client
cannot raise them. The watchdogs use wall time (``use_sim_time`` true is
rejected). A 5 ms thread wakes the 300 ms timeout even when subscription
callbacks fill the executor, so the zero still lands inside 350 ms after
Gazebo's 10-step odometry average.
"""

from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

from omnibot_hybrid.teleop_gate import (
    CMD_TIMEOUT_S,
    HARD_MAX_VX,
    HARD_MAX_WZ,
    TeleopGate,
    parameter_change_allowed,
)

# 300 ms timeout plus one tick stays inside the 350 ms stop budget.
# The executor timer is the backup. The thread below is what wakes on time
# when joint-state callbacks are queued in front of it.
TICK_PERIOD_S = 0.01
WATCHDOG_TICK_S = 0.005

_STATE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TeleopSafety(Node):
    """Apply TeleopGate and publish the mux inputs."""

    def __init__(self, clock=None, audit_dir=None):
        super().__init__("teleop_safety")
        self._own_clock = clock is None
        self._clock = clock or time.monotonic
        self._gate = TeleopGate(clock=self._clock, audit_dir=audit_dir)
        self._apply_lock = threading.Lock()
        self._last_state = ""
        self._last_state_pub = 0.0
        self._watchdog = None
        self._watchdog_stop = None

        self.add_on_set_parameters_callback(self._on_parameters)

        self.create_subscription(Twist, "/cmd_vel/teleop_raw", self._twist_cb, 10)
        self.create_subscription(
            JointState,
            "/arm/joint_commands/teleop_raw",
            self._joint_cb,
            10,
        )
        self.create_subscription(
            JointState, "/arm/joint_states", self._joint_state_cb, 10
        )
        self.create_subscription(Empty, "/ohho/heartbeat", self._heartbeat_cb, 10)
        # In-repo publishers use the default volatile depth-10 QoS, which does
        # not replay the last Bool to this new subscription. Transient-local
        # here would not connect to those publishers. The gate starts latched.
        self.create_subscription(Bool, "/emergency_stop", self._estop_cb, 10)

        self._twist_pub = self.create_publisher(Twist, "/cmd_vel/teleop", 10)
        self._joint_pub = self.create_publisher(
            JointState, "/arm/joint_commands/teleop", 10
        )
        self._state_pub = self.create_publisher(
            String, "/ohho/safety/state", _STATE_QOS
        )
        self.create_service(Trigger, "/ohho/estop_reset", self._reset_cb)
        self.create_timer(
            TICK_PERIOD_S,
            self._tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self._publish_state(force=True)
        # An injected clock is a unit test. The thread would race those
        # assertions. Production uses the monotonic clock and the thread.
        if self._own_clock:
            self._watchdog_stop = threading.Event()
            self._watchdog = threading.Thread(
                target=self._watchdog_loop,
                name="teleop_safety_watchdog",
                daemon=True,
            )
            self._watchdog.start()
        self.get_logger().info(
            "teleop_safety ceilings "
            f"lin={HARD_MAX_VX} m/s ang={HARD_MAX_WZ} rad/s "
            f"watchdog={CMD_TIMEOUT_S}s tick={TICK_PERIOD_S}s. "
            "Gate starts latched until /ohho/estop_reset. "
            "Clients publish /cmd_vel/teleop_raw and "
            "/arm/joint_commands/teleop_raw."
        )

    def _on_parameters(self, params):
        for param in params:
            if not parameter_change_allowed(param.name, param.value):
                return SetParametersResult(
                    successful=False,
                    reason="teleop_safety caps, watchdogs, and wall-clock are fixed",
                )
        return SetParametersResult(successful=True)

    def _twist_cb(self, msg: Twist) -> None:
        with self._apply_lock:
            self._apply_locked(
                self._gate.command_velocity(
                    msg.linear.x,
                    msg.linear.y,
                    msg.angular.z,
                    self._clock(),
                )
            )

    def _joint_cb(self, msg: JointState) -> None:
        with self._apply_lock:
            self._apply_locked(
                self._gate.command_joints(
                    list(msg.name),
                    list(msg.position),
                    self._clock(),
                )
            )

    def _joint_state_cb(self, msg: JointState) -> None:
        with self._apply_lock:
            self._gate.note_joint_state(list(msg.name), list(msg.position))

    def _heartbeat_cb(self, _msg: Empty) -> None:
        with self._apply_lock:
            self._apply_locked(self._gate.heartbeat(self._clock()))

    def _estop_cb(self, msg: Bool) -> None:
        with self._apply_lock:
            self._apply_locked(self._gate.set_emergency_stop(msg.data, self._clock()))

    def _reset_cb(self, _request, response):
        with self._apply_lock:
            outcome = self._gate.reset(self._clock())
            response.success = bool(outcome.ok)
            response.message = outcome.message
            self._apply_locked(outcome)
        return response

    def _watchdog_loop(self) -> None:
        while not self._watchdog_stop.wait(WATCHDOG_TICK_S):
            self._tick()

    def destroy_node(self):
        if self._watchdog_stop is not None:
            self._watchdog_stop.set()
            if self._watchdog is not None:
                self._watchdog.join(timeout=0.2)
        return super().destroy_node()

    def _tick(self) -> None:
        with self._apply_lock:
            self._apply_locked(self._gate.tick(self._clock()))

    def _apply_locked(self, outcome) -> None:
        if outcome.log:
            self.get_logger().warn(outcome.log)
        if outcome.twist is not None:
            self._publish_twist(*outcome.twist)
        if outcome.joints:
            self._publish_joints(outcome.joints)
        self._publish_state()

    def _publish_twist(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._twist_pub.publish(msg)

    def _publish_joints(self, joints) -> None:
        msg = JointState()
        msg.name = [name for name, _position in joints]
        msg.position = [float(position) for _name, position in joints]
        self._joint_pub.publish(msg)

    def _publish_state(self, force: bool = False) -> None:
        now = self._clock()
        payload = self._gate.state_json(now)
        due = now - self._last_state_pub >= 1.0
        if not force and payload == self._last_state and not due:
            return
        msg = String()
        msg.data = payload
        self._state_pub.publish(msg)
        self._last_state = payload
        self._last_state_pub = now


def main(args=None):
    rclpy.init(args=args)
    node = TeleopSafety()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
