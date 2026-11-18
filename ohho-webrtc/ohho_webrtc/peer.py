# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""aiortc answerer. Imports aiortc only when a viewer offer arrives and the
package is installed, so config and signaling tests run without it.
"""

from __future__ import annotations

import asyncio
import logging

from ohho_webrtc.config import StreamerConfig, ice_servers
from ohho_webrtc.encoder import EncoderChoice, install_low_latency_h264
from ohho_webrtc.tracks import TrackRegistry

LOG = logging.getLogger("ohho_webrtc.peer")


def aiortc_available() -> bool:
    try:
        import aiortc  # noqa: F401
        import av  # noqa: F401
    except ImportError:
        return False
    return True


class MediaUnavailable(RuntimeError):
    """aiortc is not installed, so this process cannot answer an offer."""


class UnavailableAnswerer:
    """Stand-in that tells the browser the media stack is missing."""

    async def handle_offer(self, viewer_id: str, tracks: tuple[str, ...], sdp: str) -> str:
        del viewer_id, tracks, sdp
        raise MediaUnavailable("aiortc is not installed")

    async def handle_ice(self, viewer_id, track, candidate, sdp_mid, sdp_mline_index) -> None:
        del viewer_id, track, candidate, sdp_mid, sdp_mline_index

    async def remove_viewer(self, viewer_id: str) -> None:
        del viewer_id


def build_answerer(config: StreamerConfig, registry: TrackRegistry, encoder: EncoderChoice):
    if not aiortc_available():
        LOG.warning("aiortc is not installed; signaling will report media_unavailable")
        return UnavailableAnswerer()
    return AiortcAnswerer(config, registry, encoder)


class AiortcAnswerer:
    def __init__(self, config: StreamerConfig, registry: TrackRegistry, encoder: EncoderChoice):
        if not install_low_latency_h264(
            gop=encoder.gop,
            bitrate_bps=encoder.bitrate_bps,
            codec_name=encoder.aiortc_codec,
        ):
            raise MediaUnavailable("aiortc is not installed")
        self.config = config
        self.registry = registry
        self.encoder = encoder
        self._track_cls = _registry_track_class()
        self._sessions: dict[str, object] = {}
        self._early: dict[str, list[tuple]] = {}

    async def handle_offer(self, viewer_id: str, tracks: tuple[str, ...], sdp: str) -> str:
        from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection, RTCSessionDescription

        await self.remove_viewer(viewer_id)
        servers = []
        for entry in ice_servers(self.config):
            urls = entry["urls"]
            username = entry.get("username")
            credential = entry.get("credential")
            if username:
                servers.append(RTCIceServer(urls=urls, username=username, credential=credential))
            else:
                servers.append(RTCIceServer(urls=urls))
        pc = RTCPeerConnection(RTCConfiguration(iceServers=servers))
        for name in tracks:
            pc.addTrack(
                self._track_cls(self.registry, name, self.config.width, self.config.height)
            )
        prefer_h264(pc)
        self._sessions[viewer_id] = pc
        await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
        for queued in self._early.pop(viewer_id, []):
            await _add_candidate(pc, *queued)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        await _wait_ice_gathering(pc, timeout=3.0)
        local = pc.localDescription
        if local is None or not local.sdp:
            raise MediaUnavailable("aiortc did not produce an answer")
        return local.sdp

    async def handle_ice(self, viewer_id, track, candidate, sdp_mid, sdp_mline_index) -> None:
        del track
        if not candidate:
            return
        pc = self._sessions.get(viewer_id)
        item = (candidate, sdp_mid, sdp_mline_index)
        if pc is None:
            self._early.setdefault(viewer_id, []).append(item)
            return
        await _add_candidate(pc, *item)

    async def remove_viewer(self, viewer_id: str) -> None:
        self._early.pop(viewer_id, None)
        pc = self._sessions.pop(viewer_id, None)
        if pc is not None:
            await pc.close()


def prefer_h264(pc) -> None:
    """Ask the peer connection to negotiate H.264 when the browser offers it."""
    from aiortc import RTCRtpSender

    try:
        capabilities = RTCRtpSender.getCapabilities("video")
    except Exception:
        return
    h264 = [codec for codec in capabilities.codecs if codec.mimeType.lower() == "video/h264"]
    if not h264:
        return
    for transceiver in pc.getTransceivers():
        if transceiver.kind != "video":
            continue
        setter = getattr(transceiver, "setCodecPreferences", None)
        if setter is None:
            continue
        try:
            setter(h264)
        except Exception:
            LOG.debug("could not set H264 codec preferences", exc_info=True)


def _registry_track_class():
    import av
    from aiortc import VideoStreamTrack

    class RegistryVideoTrack(VideoStreamTrack):
        def __init__(self, registry: TrackRegistry, name: str, width: int, height: int):
            super().__init__()
            self.registry = registry
            self.track_name = name
            self.width = width
            self.height = height

        async def recv(self):
            pts, time_base = await self.next_timestamp()
            array = self.registry.frame_for_encode(self.track_name, self.width, self.height)
            frame = av.VideoFrame.from_ndarray(array, format="rgb24")
            frame.pts = pts
            frame.time_base = time_base
            return frame

    return RegistryVideoTrack


async def _wait_ice_gathering(pc, timeout: float) -> None:
    if pc.iceGatheringState == "complete":
        return
    done = asyncio.Event()

    @pc.on("icegatheringstatechange")
    def _on_state():
        if pc.iceGatheringState == "complete":
            done.set()

    if pc.iceGatheringState == "complete":
        return
    try:
        await asyncio.wait_for(done.wait(), timeout)
    except asyncio.TimeoutError:
        return


async def _add_candidate(pc, candidate: str, sdp_mid, sdp_mline_index) -> None:
    from aiortc.sdp import candidate_from_sdp

    line = candidate[len("candidate:") :] if candidate.startswith("candidate:") else candidate
    try:
        parsed = candidate_from_sdp(line)
    except Exception:
        LOG.debug("ignored malformed ICE candidate")
        return
    parsed.sdpMid = sdp_mid
    parsed.sdpMLineIndex = sdp_mline_index
    await pc.addIceCandidate(parsed)
