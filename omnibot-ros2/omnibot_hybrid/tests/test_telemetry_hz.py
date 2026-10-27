"""~20 Hz check. Skips unless ROS 2 Jazzy can import this node.

Run, from a sourced Jazzy shell, with this package on PYTHONPATH:

    pytest omnibot-ros2/omnibot_hybrid/tests/test_telemetry_hz.py -q

Or watch the topic after a launch:

    ros2 launch omnibot_hybrid telemetry.launch.py sim:=true
    ros2 topic hz /ohho/telemetry
"""

from __future__ import annotations

import json
import os
import threading
import time

import pytest

from schema_tools import assert_valid_telemetry

# Set before any test in this process calls rclpy.init(). test/conftest.py
# owns that context for the whole pytest session.
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
os.environ.setdefault("ROS_DOMAIN_ID", "208")

_SKIP = (
    "ROS 2 Jazzy is not importable. Source /opt/ros/jazzy/setup.bash, put "
    "omnibot-ros2/omnibot_hybrid on PYTHONPATH, then run: "
    "pytest omnibot-ros2/omnibot_hybrid/tests/test_telemetry_hz.py -q. "
    "Live check: ros2 launch omnibot_hybrid telemetry.launch.py sim:=true "
    "&& ros2 topic hz /ohho/telemetry"
)


def _need(name: str):
    return pytest.importorskip(name, reason=_SKIP)


def test_telemetry_publishes_about_20hz():
    rclpy = _need("rclpy")
    _need("nav_msgs")
    _need("nav2_msgs")
    _need("action_msgs")
    _need("sensor_msgs")
    _need("diagnostic_msgs")
    _need("geometry_msgs")
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import String

    from omnibot_hybrid.telemetry_aggregator import TelemetryAggregator

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    stamps: list[float] = []
    payloads: list[str] = []

    # test/conftest.py already called rclpy.init() for this process.
    # Jazzy rejects a second init, and shutting that context down would
    # break the rest of the session.
    started_context = False
    if not rclpy.ok():
        rclpy.init()
        started_context = True
    aggregator = None
    listener = None
    try:
        aggregator = TelemetryAggregator()
        listener = rclpy.create_node("telemetry_hz_listener")

        def _on_sample(msg: String) -> None:
            stamps.append(time.monotonic())
            payloads.append(msg.data)

        listener.create_subscription(String, "/ohho/telemetry", _on_sample, qos)
        executor = SingleThreadedExecutor()
        executor.add_node(aggregator)
        executor.add_node(listener)
        stop = threading.Event()

        def _spin() -> None:
            while not stop.is_set():
                executor.spin_once(timeout_sec=0.05)

        thread = threading.Thread(target=_spin, daemon=True)
        thread.start()
        time.sleep(3.0)
        stop.set()
        thread.join(timeout=2.0)
    finally:
        if aggregator is not None:
            aggregator.destroy_node()
        if listener is not None:
            listener.destroy_node()
        if started_context:
            rclpy.shutdown()

    assert len(stamps) >= 20, f"only {len(stamps)} samples on /ohho/telemetry"
    span = stamps[-1] - stamps[0]
    rate = (len(stamps) - 1) / span
    assert 15.0 <= rate <= 25.0, f"/ohho/telemetry at {rate:.2f} Hz, expected ~20"

    document = json.loads(payloads[-1])
    assert document["v"] == "1.0.0"
    assert document["kind"] == "real"
    assert "age_ms" in document
    assert_valid_telemetry(document)
