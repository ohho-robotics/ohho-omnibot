# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Robot-side WebRTC streamer for OmniBot camera topics (OHH-210)."""

__version__ = "0.1.0"

# Public path the OHH-196 gateway exposes. The gateway strips ``/webrtc``
# and forwards the rest, so this process serves ``/signaling``.
GATEWAY_SIGNALING_PATH = "/webrtc/signaling"
UPSTREAM_SIGNALING_PATH = "/signaling"
GATEWAY_WEBRTC_FLAG = "--webrtc"
