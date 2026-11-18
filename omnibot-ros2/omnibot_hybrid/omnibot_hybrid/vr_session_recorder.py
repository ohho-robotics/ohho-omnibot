#!/usr/bin/env python3
"""Record one VR session to MCAP. Read-only: no command publishers.

Subscriptions
    /ohho/vr_session/start   std_msgs/String   session id, empty uses the time
    /ohho/vr_session/stop    std_msgs/Empty
    /ohho/heartbeat          std_msgs/Empty    auto-session only
    /ohho/telemetry          std_msgs/String
    /ohho/safety/state       std_msgs/String

Publications
    /ohho/vr_session/recorder   std_msgs/String   idle | recording:<dir>

Auto-session follows the std_msgs/Empty heartbeat that ``teleop_safety``
already uses. A JSON String on the same name is a different type, and one
node cannot subscribe to both. The ros2 bag process still records
/ohho/heartbeat. This node does not write that topic. Session idle is not
the deadman.
"""

from __future__ import annotations

import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

from omnibot_hybrid.vr_session_record import (
    DEFAULT_HEARTBEAT_GAP_SEC,
    DEFAULT_MAX_BAG_SIZE_BYTES,
    DEFAULT_MAX_TOTAL_BYTES,
    DEFAULT_METRICS_PORT,
    DEFAULT_OUTPUT_DIR,
    DEFAULT_SESSION_IDLE_SEC,
    MetricsEndpoint,
    RecorderConfig,
    SessionRecorder,
)

_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class VrSessionRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("vr_session_recorder")
        self.add_on_set_parameters_callback(self._on_parameters)
        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)
        self.declare_parameter("max_bag_size_bytes", DEFAULT_MAX_BAG_SIZE_BYTES)
        self.declare_parameter("max_total_bytes", DEFAULT_MAX_TOTAL_BYTES)
        self.declare_parameter("auto_session", True)
        self.declare_parameter("session_idle_sec", DEFAULT_SESSION_IDLE_SEC)
        self.declare_parameter("heartbeat_gap_sec", DEFAULT_HEARTBEAT_GAP_SEC)
        self.declare_parameter("bag_use_sim_time", False)
        self.declare_parameter("metrics_port", DEFAULT_METRICS_PORT)
        if bool(self.get_parameter("use_sim_time").value):
            raise RuntimeError(
                "vr_session_recorder keeps session timing on wall time; "
                "use_sim_time must be false"
            )
        config = RecorderConfig(
            output_dir=str(self.get_parameter("output_dir").value),
            max_bag_size_bytes=int(self.get_parameter("max_bag_size_bytes").value),
            max_total_bytes=int(self.get_parameter("max_total_bytes").value),
            auto_session=bool(self.get_parameter("auto_session").value),
            session_idle_sec=float(self.get_parameter("session_idle_sec").value),
            heartbeat_gap_sec=float(self.get_parameter("heartbeat_gap_sec").value),
            use_sim_time=bool(self.get_parameter("bag_use_sim_time").value),
            metrics_port=int(self.get_parameter("metrics_port").value),
        ).validated()
        self._recorder = SessionRecorder(config)
        self._metrics = None
        self._closed = False
        if config.metrics_port > 0:
            try:
                self._metrics = MetricsEndpoint(
                    self._recorder.render_metrics,
                    config.metrics_port,
                )
            except OSError as exc:
                self.get_logger().error(
                    f"metrics port {config.metrics_port} failed: {exc}"
                )
        self.create_subscription(String, "/ohho/vr_session/start", self._on_start, 10)
        self.create_subscription(Empty, "/ohho/vr_session/stop", self._on_stop, 10)
        self.create_subscription(Empty, "/ohho/heartbeat", self._on_heartbeat, 10)
        self.create_subscription(String, "/ohho/telemetry", self._on_telemetry, _LATCHED)
        self.create_subscription(
            String, "/ohho/safety/state", self._on_safety, _LATCHED
        )
        self._status = self.create_publisher(String, "/ohho/vr_session/recorder", 10)
        self.create_timer(0.2, self._on_timer)
        self.create_timer(1.0, self._publish_status)
        port = self._metrics.port if self._metrics is not None else 0
        self.get_logger().info(
            "vr_session_recorder ready "
            f"dir={config.output_dir} "
            f"max_bag={config.max_bag_size_bytes} "
            f"max_total={config.max_total_bytes} "
            f"auto_session={config.auto_session} "
            f"metrics=127.0.0.1:{port}"
        )

    def _on_parameters(self, params):
        for param in params:
            if param.name == "use_sim_time" and bool(param.value):
                return SetParametersResult(
                    successful=False,
                    reason="vr_session_recorder session timing stays on wall time",
                )
        return SetParametersResult(successful=True)

    def _on_start(self, msg: String) -> None:
        self._log(self._recorder.start(time.monotonic(), msg.data))

    def _on_stop(self, _msg: Empty) -> None:
        self._log(self._recorder.stop(time.monotonic()))

    def _on_heartbeat(self, _msg) -> None:
        self._log(self._recorder.heartbeat(time.monotonic()))

    def _on_telemetry(self, msg: String) -> None:
        self._recorder.telemetry(msg.data)

    def _on_safety(self, msg: String) -> None:
        self._recorder.safety(msg.data)

    def _on_timer(self) -> None:
        self._log(self._recorder.poll(time.monotonic()))

    def _log(self, notes: list[str]) -> None:
        for note in notes:
            if note.startswith("error:"):
                self.get_logger().error(note)
            else:
                self.get_logger().info(note)
        if notes:
            self._publish_status()

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self._recorder.status_text()
        self._status.publish(msg)

    def shutdown_recorder(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._log(self._recorder.shutdown(time.monotonic()))
        except Exception as exc:
            self.get_logger().error(f"recorder shutdown: {exc}")
        if self._metrics is not None:
            self._metrics.close()
            self._metrics = None

    def destroy_node(self):
        self.shutdown_recorder()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VrSessionRecorderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown_recorder()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
