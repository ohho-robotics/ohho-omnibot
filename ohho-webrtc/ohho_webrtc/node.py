# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""ROS 2 node. ``rclpy`` is imported inside ``main`` so unit tests can import
this module on a machine without ROS 2.
"""

from __future__ import annotations

import logging
import threading
import time

from ohho_webrtc.config import StreamerConfig, parse_streamer_config
from ohho_webrtc.encoder import EncoderChoice, detect_encoder_support, select_encoder
from ohho_webrtc.frames import decode_ros_image, test_pattern
from ohho_webrtc.peer import build_answerer
from ohho_webrtc.server import ServerThread
from ohho_webrtc.tracks import TrackRegistry

LOG = logging.getLogger("ohho_webrtc.node")

_PARAM_DEFAULTS = {
    "track_names": ["chase", "front", "wrist"],
    "track_topics": [
        "/camera/chase/image_raw",
        "/camera/front/image_raw",
        "/camera/wrist/image_raw",
    ],
    "signaling_host": "127.0.0.1",
    "signaling_port": 8766,
    "stun_urls": ["stun:stun.l.google.com:19302"],
    "turn_urls": [],
    "turn_username": "",
    "turn_credential": "",
    "width": 1280,
    "height": 720,
    "fps": 30,
    "bitrate_bps": 3000000,
    "gop": 15,
    "encoder": "auto",
    "test_pattern": False,
}


def config_from_values(values: dict) -> StreamerConfig:
    merged = dict(_PARAM_DEFAULTS)
    merged.update(values)
    return parse_streamer_config(merged)


class PatternPump:
    """Publish a coloured test card on every configured topic."""

    def __init__(self, registry: TrackRegistry, config: StreamerConfig):
        self._registry = registry
        self._config = config
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="ohho-webrtc-pattern", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)

    def _run(self) -> None:
        period = 1.0 / self._config.fps
        while not self._stop.is_set():
            stamp = time.time_ns()
            for spec in self._config.tracks:
                image = test_pattern(spec.name, self._config.width, self._config.height, stamp)
                self._registry.push(spec.topic, image, stamp)
            self._stop.wait(period)


def start_runtime(
    config: StreamerConfig,
    support=None,
) -> tuple[TrackRegistry, EncoderChoice, ServerThread, PatternPump | None]:
    registry = TrackRegistry(config.tracks)
    encoder = select_encoder(config, support if support is not None else detect_encoder_support())
    LOG.info(
        "pipeline=%s aiortc=%s bframes=%s gop=%s bitrate=%s",
        encoder.pipeline_encoder,
        encoder.aiortc_codec,
        encoder.bframes,
        encoder.gop,
        encoder.bitrate_bps,
    )
    pattern = PatternPump(registry, config) if config.test_pattern else None
    if pattern is not None:
        pattern.start()
    thread = ServerThread(config, registry, encoder, answerer_factory=build_answerer)
    thread.start()
    if not thread.ready.wait(timeout=15):
        raise RuntimeError("signaling server did not bind")
    if thread._error is not None:
        raise RuntimeError("signaling server failed") from thread._error
    return registry, encoder, thread, pattern


def main(args=None) -> None:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init(args=args)
    node = Node("ohho_webrtc")
    try:
        values = {}
        for name, default in _PARAM_DEFAULTS.items():
            node.declare_parameter(name, default)
            values[name] = node.get_parameter(name).value
        config = config_from_values(values)
        registry, encoder, thread, pattern = start_runtime(config)
        seen_errors: set[str] = set()

        def on_image(message: Image, topic: str = "") -> None:
            try:
                image, stamp_ns = decode_ros_image(
                    width=int(message.width),
                    height=int(message.height),
                    encoding=str(message.encoding),
                    step=int(message.step),
                    data=message.data,
                    stamp_sec=int(message.header.stamp.sec),
                    stamp_nanosec=int(message.header.stamp.nanosec),
                )
            except ValueError as exc:
                text = str(exc)
                if text not in seen_errors:
                    seen_errors.add(text)
                    node.get_logger().warning("skipping frame on %s: %s", topic, text)
                return
            registry.push(topic, image, stamp_ns)

        for spec in config.tracks:
            node.create_subscription(
                Image,
                spec.topic,
                lambda message, topic=spec.topic: on_image(message, topic),
                10,
            )
        node.get_logger().info(
            "signaling http://%s:%s/  gateway: ohho-gateway --webrtc http://127.0.0.1:%s"
            "  pipeline=%s aiortc=%s bframes=0",
            config.host,
            thread.port,
            thread.port,
            encoder.pipeline_encoder,
            encoder.aiortc_codec,
        )
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            thread.stop()
            if pattern is not None:
                pattern.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
