# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Encoder selection, the GStreamer recipe, and aiortc low-latency H.264 settings.

NVENC is used for the live aiortc path only after a child process encodes one
frame with ``h264_nvenc``. A missing GPU, or a driver that faults inside
PyAV, stays on libx264. Tests inject :class:`EncoderSupport` and do not open
a device.
"""

from __future__ import annotations

import fractions
import subprocess
import sys
from dataclasses import dataclass

from ohho_webrtc.config import StreamerConfig

_NVENC_PROBE = """
import sys
try:
    import fractions
    import av
    import numpy as np
    frame = av.VideoFrame.from_ndarray(
        np.zeros((64, 64, 3), dtype=np.uint8), format="rgb24"
    )
    frame.pts = 0
    ctx = av.CodecContext.create("h264_nvenc", "w")
    ctx.width = 64
    ctx.height = 64
    ctx.pix_fmt = "yuv420p"
    ctx.bit_rate = 2000000
    ctx.framerate = fractions.Fraction(30, 1)
    ctx.time_base = fractions.Fraction(1, 30)
    ctx.options = {"preset": "p1", "tune": "ull", "rc": "cbr", "bf": "0", "g": "15"}
    ctx.max_b_frames = 0
    ctx.gop_size = 15
    packets = ctx.encode(frame)
    if not packets:
        packets = ctx.encode(None)
    if not packets:
        sys.exit(1)
except Exception:
    sys.exit(1)
"""

# Mutable so a second install updates GOP and codec without wrapping twice.
_LOW_LATENCY = {"gop": 15, "codec": "libx264", "installed": False}


@dataclass(frozen=True)
class EncoderSupport:
    nvenc_pyav: bool = False
    nvenc_gstreamer: bool = False
    v4l2h264enc: bool = False
    x264enc_gstreamer: bool = False
    openh264enc: bool = False
    nvidia_gpu: bool = False


@dataclass(frozen=True)
class EncoderChoice:
    """What the process will actually open, and the matching GStreamer element."""

    pipeline_encoder: str
    aiortc_codec: str
    bframes: int
    gop: int
    bitrate_bps: int
    width: int
    height: int
    fps: int
    fallback_from: str | None = None

    def __post_init__(self) -> None:
        if self.bframes != 0:
            raise ValueError("bframes must be 0")


def detect_encoder_support(run=None) -> EncoderSupport:
    """Probe NVENC, GStreamer elements, and ``nvidia-smi``.

    ``run`` takes an argument vector and returns a process return code. The
    default runner isolates NVENC in a child so a driver fault cannot kill
    the streamer.
    """
    if run is None:
        run = _subprocess_code
    nvenc_pyav = run([sys.executable, "-c", _NVENC_PROBE]) == 0
    return EncoderSupport(
        nvenc_pyav=nvenc_pyav,
        nvenc_gstreamer=run(["gst-inspect-1.0", "nvh264enc"]) == 0,
        v4l2h264enc=run(["gst-inspect-1.0", "v4l2h264enc"]) == 0,
        x264enc_gstreamer=run(["gst-inspect-1.0", "x264enc"]) == 0,
        openh264enc=run(["gst-inspect-1.0", "openh264enc"]) == 0,
        nvidia_gpu=run(["nvidia-smi", "-L"]) == 0,
    )


def select_encoder(config: StreamerConfig, support: EncoderSupport) -> EncoderChoice:
    """Pick a pipeline element and the PyAV codec the aiortc path opens."""
    common = dict(
        bframes=0,
        gop=config.gop,
        bitrate_bps=config.bitrate_bps,
        width=config.width,
        height=config.height,
        fps=config.fps,
    )
    preference = config.encoder
    if preference == "v4l2":
        if support.v4l2h264enc:
            return EncoderChoice("v4l2h264enc", "libx264", fallback_from=None, **common)
        return EncoderChoice("x264enc", "libx264", fallback_from="v4l2", **common)
    if preference == "openh264":
        if support.openh264enc:
            return EncoderChoice("openh264enc", "libx264", fallback_from=None, **common)
        return EncoderChoice("x264enc", "libx264", fallback_from="openh264", **common)
    if preference == "x264":
        return EncoderChoice("x264enc", "libx264", fallback_from=None, **common)

    wants_nvenc = preference in ("auto", "nvenc")
    if wants_nvenc and support.nvenc_pyav:
        return EncoderChoice("nvh264enc", "h264_nvenc", fallback_from=None, **common)
    if preference == "auto" and support.nvenc_gstreamer:
        # The element is installed. aiortc still opens libx264 until the
        # PyAV NVENC probe succeeds.
        return EncoderChoice("nvh264enc", "libx264", fallback_from=None, **common)
    if preference == "nvenc":
        return EncoderChoice("x264enc", "libx264", fallback_from="nvenc", **common)
    if preference == "auto" and support.v4l2h264enc:
        return EncoderChoice("v4l2h264enc", "libx264", fallback_from=None, **common)
    return EncoderChoice("x264enc", "libx264", fallback_from=None, **common)


def gstreamer_pipeline(choice: EncoderChoice, config: StreamerConfig) -> str:
    """A webrtcbin description with the low-latency H.264 settings.

    The string is the recipe for an RTX desktop (``nvh264enc``) or a Pi
    (``v4l2h264enc``). The Windows and CI runtime sends media with aiortc
    using :attr:`EncoderChoice.aiortc_codec` and the same GOP, bitrate, and
    B-frame policy.
    """
    stun = _gst_url(config.stun_urls[0]) if config.stun_urls else ""
    turn = ""
    if config.turn_urls:
        turn = _gst_url(config.turn_urls[0], config.turn_username, config.turn_credential)
    head = ["webrtcbin", "name=sendrecv", "bundle-policy=max-bundle"]
    if stun:
        head.append(f"stun-server={stun}")
    if turn:
        head.append(f"turn-server={turn}")
    branches = [" ".join(head)]
    for spec in config.tracks:
        branches.append(_branch(spec.name, choice))
    return " ".join(branches)


def x264_options(gop: int) -> dict[str, str]:
    return {
        "level": "31",
        "tune": "zerolatency",
        "preset": "ultrafast",
        "bf": "0",
        "g": str(gop),
        "keyint_min": str(gop),
    }


def nvenc_options(gop: int) -> dict[str, str]:
    return {
        "preset": "p1",
        "tune": "ull",
        "rc": "cbr",
        "bf": "0",
        "g": str(gop),
        "zerolatency": "1",
    }


def install_low_latency_h264(*, gop: int, bitrate_bps: int, codec_name: str) -> bool:
    """Point aiortc's H.264 encoder at a short GOP, no B-frames, and ``bitrate_bps``.

    Returns False when aiortc is not installed. Safe to call more than once.
    """
    try:
        import av
        import aiortc.codecs.h264 as h264mod
    except ImportError:
        return False

    if bitrate_bps > h264mod.MAX_BITRATE:
        h264mod.MAX_BITRATE = bitrate_bps
    h264mod.DEFAULT_BITRATE = bitrate_bps
    _LOW_LATENCY["gop"] = gop
    _LOW_LATENCY["codec"] = codec_name
    if _LOW_LATENCY["installed"]:
        return True

    def _encode_frame(self, frame, force_keyframe):
        gop_size = int(_LOW_LATENCY["gop"])
        codec_name_now = str(_LOW_LATENCY["codec"])
        if self.codec and (
            frame.width != self.codec.width
            or frame.height != self.codec.height
            or abs(self.target_bitrate - self.codec.bit_rate) / self.codec.bit_rate > 0.1
        ):
            self.buffer_data = b""
            self.buffer_pts = None
            self.codec = None

        index = getattr(self, "_ohho_index", 0)
        self._ohho_index = index + 1
        if index % gop_size == 0:
            force_keyframe = True
        if force_keyframe:
            frame.pict_type = av.video.frame.PictureType.I
        else:
            frame.pict_type = av.video.frame.PictureType.NONE

        if self.codec is None:
            self.codec = av.CodecContext.create(codec_name_now, "w")
            self.codec.width = frame.width
            self.codec.height = frame.height
            self.codec.bit_rate = self.target_bitrate
            self.codec.pix_fmt = "yuv420p"
            self.codec.framerate = fractions.Fraction(h264mod.MAX_FRAME_RATE, 1)
            self.codec.time_base = fractions.Fraction(1, h264mod.MAX_FRAME_RATE)
            if codec_name_now == "h264_nvenc":
                self.codec.options = nvenc_options(gop_size)
            else:
                self.codec.options = x264_options(gop_size)
                self.codec.profile = "Baseline"
            self.codec.gop_size = gop_size
            self.codec.max_b_frames = 0

        data_to_send = b""
        for package in self.codec.encode(frame):
            data_to_send += bytes(package)
        if data_to_send:
            yield from h264mod.H264Encoder._split_bitstream(data_to_send)

    h264mod.H264Encoder._encode_frame = _encode_frame
    _LOW_LATENCY["installed"] = True
    return True


def _branch(name: str, choice: EncoderChoice) -> str:
    bitrate_kbit = max(1, choice.bitrate_bps // 1000)
    caps = (
        f"video/x-raw,format=I420,width={choice.width},height={choice.height},"
        f"framerate={choice.fps}/1"
    )
    encoder = _element(choice, bitrate_kbit)
    return (
        f"appsrc name={name} is-live=true format=time do-timestamp=true ! "
        f"videoconvert ! videoscale ! {caps} ! {encoder} ! "
        f"video/x-h264,profile=baseline ! "
        f"rtph264pay pt=96 config-interval=1 aggregate-mode=zero-latency ! "
        f"sendrecv."
    )


def _element(choice: EncoderChoice, bitrate_kbit: int) -> str:
    name = choice.pipeline_encoder
    if name == "nvh264enc":
        return (
            "nvh264enc zerolatency=true preset=low-latency-hq rc-mode=cbr "
            f"bitrate={bitrate_kbit} gop-size={choice.gop} bframes=0 aud=false"
        )
    if name == "v4l2h264enc":
        return (
            'v4l2h264enc extra-controls="controls,video_bitrate='
            f'{choice.bitrate_bps},h264_i_frame_period={choice.gop},video_bitrate_mode=1"'
        )
    if name == "openh264enc":
        return f"openh264enc bitrate={choice.bitrate_bps} gop-size={choice.gop} complexity=low"
    return (
        "x264enc tune=zerolatency speed-preset=ultrafast "
        f"bitrate={bitrate_kbit} key-int-max={choice.gop} bframes=0 b-adapt=false byte-stream=true"
    )


def _gst_url(url: str, username: str = "", credential: str = "") -> str:
    if "://" in url:
        scheme, rest = url.split("://", 1)
    else:
        scheme, rest = url.split(":", 1)
    if username and scheme.startswith("turn"):
        return f"{scheme}://{username}:{credential}@{rest}"
    return f"{scheme}://{rest}"


def _subprocess_code(argv: list[str]) -> int:
    kwargs = {}
    if sys.platform == "win32":
        kwargs["creationflags"] = subprocess.CREATE_NO_WINDOW
    try:
        completed = subprocess.run(
            argv,
            capture_output=True,
            timeout=8,
            check=False,
            **kwargs,
        )
    except (OSError, subprocess.TimeoutExpired):
        return 1
    return int(completed.returncode)
