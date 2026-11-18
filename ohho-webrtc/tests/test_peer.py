# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
import asyncio

import numpy as np
import pytest

from ohho_webrtc.config import TrackSpec, parse_streamer_config
from ohho_webrtc.encoder import EncoderSupport, select_encoder
from ohho_webrtc.peer import AiortcAnswerer, aiortc_available, prefer_h264
from ohho_webrtc.tracks import TrackRegistry


@pytest.mark.skipif(not aiortc_available(), reason="aiortc is not installed")
def test_aiortc_loopback_delivers_one_h264_frame():
    asyncio.run(_loopback())


async def _loopback():
    from aiortc import RTCPeerConnection, RTCSessionDescription

    from ohho_webrtc.peer import _wait_ice_gathering

    config = parse_streamer_config(
        {
            "track_names": ["chase"],
            "track_topics": ["/camera/chase/image_raw"],
            "stun_urls": [],
            "width": 320,
            "height": 240,
            "fps": 30,
            "bitrate_bps": 2_000_000,
            "gop": 15,
            "encoder": "x264",
        }
    )
    registry = TrackRegistry((TrackSpec("chase", "/camera/chase/image_raw"),))
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    image[:, :] = (40, 80, 160)
    registry.push("/camera/chase/image_raw", image, 1_700_000_000_000_000_000)
    encoder = select_encoder(config, EncoderSupport())
    answerer = AiortcAnswerer(config, registry, encoder)
    viewer = RTCPeerConnection()
    try:
        viewer.addTransceiver("video", direction="recvonly")
        prefer_h264(viewer)
        offer = await viewer.createOffer()
        await viewer.setLocalDescription(offer)
        await _wait_ice_gathering(viewer, 3.0)
        answer_sdp = await answerer.handle_offer(
            "viewer",
            ("chase",),
            viewer.localDescription.sdp,
        )
        assert "H264" in answer_sdp or "h264" in answer_sdp
        frame_future = asyncio.get_running_loop().create_future()

        @viewer.on("track")
        def on_track(track):
            async def read():
                try:
                    frame = await track.recv()
                except Exception as exc:
                    if not frame_future.done():
                        frame_future.set_exception(exc)
                    return
                if not frame_future.done():
                    frame_future.set_result(frame)

            asyncio.ensure_future(read())

        await viewer.setRemoteDescription(RTCSessionDescription(sdp=answer_sdp, type="answer"))
        frame = await asyncio.wait_for(frame_future, 20)
        assert frame.width == 320
        assert frame.height == 240
    finally:
        await answerer.remove_viewer("viewer")
        await viewer.close()
