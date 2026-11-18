#!/usr/bin/env python3
"""Move each SO-101 joint to a scripted target in Gazebo, then check e-stop.

Run while sim.launch.py is up:

    ros2 run omnibot_arm arm_sim_scripted_move.py

Publishes /arm/joint_commands/out (the topic arm_sim_driver consumes).
Exits 0 when every joint is within tolerance and an e-stop holds that pose.
Exits 1 otherwise. Does not talk to hardware.
"""

import sys
import time

from arm_sim_logic import FALLBACK_JOINT_NAMES
from arm_sim_logic import HOLD_TOLERANCE_RAD
from arm_sim_logic import POSITION_TOLERANCE_RAD
from arm_sim_logic import SCRIPTED_TARGETS
from arm_sim_logic import positions_within

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

JOINT_STATE_WAIT_SEC = 45.0
MOVE_WAIT_SEC = 15.0
HOLD_SETTLE_SEC = 0.5
HOLD_SEC = 2.0
PUBLISH_PERIOD_SEC = 0.05


def _log(text):
    print(text, flush=True)


class ScriptedMove(Node):
    def __init__(self):
        super().__init__("arm_sim_scripted_move")
        self.names = list(FALLBACK_JOINT_NAMES)
        self.latest = None
        self.create_subscription(JointState, "/arm/joint_states", self._on_state, 10)
        self.cmd = self.create_publisher(JointState, "/arm/joint_commands/out", 10)
        self.estop = self.create_publisher(Bool, "/emergency_stop", 10)

    def _on_state(self, msg):
        self.latest = msg

    def _positions(self):
        if self.latest is None:
            return None
        by_name = {
            name: float(position)
            for name, position in zip(self.latest.name, self.latest.position)
        }
        if not all(name in by_name for name in self.names):
            return None
        return [by_name[name] for name in self.names]

    def _publish_target(self, positions):
        msg = JointState()
        msg.name = list(self.names)
        msg.position = [float(value) for value in positions]
        self.cmd.publish(msg)

    def _publish_estop(self, active):
        msg = Bool()
        msg.data = bool(active)
        self.estop.publish(msg)

    def _spin_for(self, seconds, publish):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            publish()
            rclpy.spin_once(self, timeout_sec=PUBLISH_PERIOD_SEC)

    def wait_for_joint_states(self):
        deadline = time.monotonic() + JOINT_STATE_WAIT_SEC
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            positions = self._positions()
            if positions is not None:
                _log(
                    "EVIDENCE /arm/joint_states "
                    f"names={list(self.latest.name)} "
                    f"position={[round(v, 4) for v in positions]}"
                )
                return True
        _log("EVIDENCE /arm/joint_states missing")
        return False

    def move_to(self, target):
        deadline = time.monotonic() + MOVE_WAIT_SEC
        while time.monotonic() < deadline:
            self._publish_target(target)
            rclpy.spin_once(self, timeout_sec=PUBLISH_PERIOD_SEC)
            positions = self._positions()
            if positions is not None and positions_within(
                positions, target, POSITION_TOLERANCE_RAD
            ):
                _log(
                    "EVIDENCE reached "
                    f"target={[round(v, 4) for v in target]} "
                    f"actual={[round(v, 4) for v in positions]}"
                )
                return True
        positions = self._positions()
        _log(
            "EVIDENCE short "
            f"target={[round(v, 4) for v in target]} "
            f"actual={None if positions is None else [round(v, 4) for v in positions]}"
        )
        return False

    def check_estop_holds(self, target_before_stop):
        self._spin_for(HOLD_SETTLE_SEC, lambda: self._publish_estop(True))
        snapshot = self._positions()
        if snapshot is None:
            _log("EVIDENCE e-stop hold missing joint_states")
            return False
        _log(
            "EVIDENCE e-stop snapshot "
            f"position={[round(v, 4) for v in snapshot]}"
        )

        rejected = [0.0] * len(self.names)
        worst = 0.0

        def publish_rejected():
            self._publish_estop(True)
            self._publish_target(rejected)

        self._spin_for(HOLD_SEC, publish_rejected)
        held = self._positions()
        if held is None:
            _log("EVIDENCE e-stop hold lost joint_states")
            return False
        for actual, frozen in zip(held, snapshot):
            worst = max(worst, abs(actual - frozen))
        stayed = positions_within(held, snapshot, HOLD_TOLERANCE_RAD)
        did_not_follow = not positions_within(
            held, rejected, POSITION_TOLERANCE_RAD
        )
        _log(
            "EVIDENCE e-stop hold "
            f"frozen={[round(v, 4) for v in snapshot]} "
            f"actual={[round(v, 4) for v in held]} "
            f"max_drift_rad={worst:.4f} "
            f"rejected_target={rejected} "
            f"before={[round(v, 4) for v in target_before_stop]}"
        )
        return stayed and did_not_follow

    def run(self):
        if not self.wait_for_joint_states():
            return 1
        if list(self.latest.name) != self.names:
            _log(
                "EVIDENCE joint names "
                f"got={list(self.latest.name)} expected={self.names}"
            )
            return 1
        target = [0.0] * len(self.names)
        for index, goal in enumerate(SCRIPTED_TARGETS):
            target[index] = float(goal)
            _log(f"EVIDENCE move joint {self.names[index]} -> {goal}")
            if not self.move_to(target):
                return 1
        # The last joint is accepted as soon as it enters tolerance.
        # Keep commanding the final pose so it can finish before the stop.
        self._spin_for(0.5, lambda: self._publish_target(target))
        if not self.check_estop_holds(target):
            return 1
        _log(
            "EVIDENCE pass "
            f"tolerance_rad={POSITION_TOLERANCE_RAD} "
            f"hold_tolerance_rad={HOLD_TOLERANCE_RAD}"
        )
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = ScriptedMove()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
