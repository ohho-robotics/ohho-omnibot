# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""``ohho-webrtc`` console entry. Without ROS, pass ``--test-pattern``."""

from __future__ import annotations

import argparse
import time

from ohho_webrtc.config import StreamerConfig, load_config, parse_streamer_config
from ohho_webrtc.node import start_runtime


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="ohho-webrtc",
        description=(
            "Serve WebRTC signaling for OmniBot cameras. "
            "The OHH-196 gateway proxies /webrtc/signaling to this process."
        ),
    )
    parser.add_argument("--config", default="", help="YAML file of ROS parameters")
    parser.add_argument("--host", default="", help="Override signaling_host")
    parser.add_argument("--port", type=int, default=-1, help="Override signaling_port")
    parser.add_argument(
        "--test-pattern",
        action="store_true",
        help="Do not require ROS. Push a timestamped test card on each track.",
    )
    parser.add_argument(
        "--track",
        action="append",
        default=[],
        help="Override one track as name=/topic. Repeat for each camera.",
    )
    args = parser.parse_args(argv)
    raw: dict = {}
    if args.config:
        config = load_config(args.config)
        raw = _config_to_raw(config)
    if args.track:
        names = []
        topics = []
        for item in args.track:
            if "=" not in item:
                raise SystemExit(f"track {item!r} must look like chase=/camera/chase/image_raw")
            name, topic = item.split("=", 1)
            names.append(name)
            topics.append(topic)
        raw["track_names"] = names
        raw["track_topics"] = topics
    if args.host:
        raw["signaling_host"] = args.host
    if args.port >= 0:
        raw["signaling_port"] = args.port
    if args.test_pattern:
        raw["test_pattern"] = True
    if not raw and not args.test_pattern:
        raise SystemExit(
            "ROS 2 is not started by this command. "
            "Use --test-pattern, or `ros2 run ohho_webrtc webrtc_streamer`."
        )
    config = parse_streamer_config(raw) if raw else parse_streamer_config({"test_pattern": True})
    registry, encoder, thread, pattern = start_runtime(config)
    page_host = "127.0.0.1" if config.host in ("0.0.0.0", "::") else config.host
    print(f"ohho_webrtc test page  http://{page_host}:{thread.port}/")
    print(f"ohho_webrtc signaling  ws://{page_host}:{thread.port}/signaling")
    print(f"gateway upstream       ohho-gateway --webrtc http://127.0.0.1:{thread.port}")
    print(
        f"encoder pipeline={encoder.pipeline_encoder} aiortc={encoder.aiortc_codec} "
        f"bframes=0 gop={encoder.gop} bitrate_bps={encoder.bitrate_bps}"
    )
    del registry
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        thread.stop()
        if pattern is not None:
            pattern.stop()
    return 0


def _config_to_raw(config: StreamerConfig) -> dict:
    return {
        "track_names": [spec.name for spec in config.tracks],
        "track_topics": [spec.topic for spec in config.tracks],
        "signaling_host": config.host,
        "signaling_port": config.port,
        "stun_urls": list(config.stun_urls),
        "turn_urls": list(config.turn_urls),
        "turn_username": config.turn_username,
        "turn_credential": config.turn_credential,
        "width": config.width,
        "height": config.height,
        "fps": config.fps,
        "bitrate_bps": config.bitrate_bps,
        "gop": config.gop,
        "encoder": config.encoder,
        "test_pattern": config.test_pattern,
    }


if __name__ == "__main__":
    raise SystemExit(main())
