# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
import fractions

import numpy as np
import pytest

from ohho_webrtc.config import default_config, parse_streamer_config
from ohho_webrtc.encoder import (
    EncoderChoice,
    EncoderSupport,
    detect_encoder_support,
    gstreamer_pipeline,
    install_low_latency_h264,
    select_encoder,
    x264_options,
)


def test_auto_uses_nvenc_when_the_pyav_probe_succeeds():
    choice = select_encoder(default_config(), EncoderSupport(nvenc_pyav=True))
    assert choice.pipeline_encoder == "nvh264enc"
    assert choice.aiortc_codec == "h264_nvenc"
    assert choice.bframes == 0
    assert choice.fallback_from is None


def test_auto_without_nvenc_uses_x264_and_keeps_bframes_off():
    choice = select_encoder(default_config(), EncoderSupport())
    assert choice.pipeline_encoder == "x264enc"
    assert choice.aiortc_codec == "libx264"
    assert choice.bframes == 0
    assert choice.gop == 15
    assert choice.bitrate_bps == 3_000_000
    pipeline = gstreamer_pipeline(choice, default_config())
    assert "x264enc" in pipeline
    assert "tune=zerolatency" in pipeline
    assert "bframes=0" in pipeline
    assert "b-adapt=false" in pipeline
    assert "key-int-max=15" in pipeline
    assert "bitrate=3000" in pipeline
    assert "width=1280" in pipeline
    assert "height=720" in pipeline
    assert "framerate=30/1" in pipeline
    assert "appsrc name=chase" in pipeline
    assert "appsrc name=front" in pipeline
    assert "appsrc name=wrist" in pipeline
    assert "stun-server=stun://stun.l.google.com:19302" in pipeline
    assert "profile=baseline" in pipeline
    assert "bframes=1" not in pipeline
    assert "bframes=2" not in pipeline


def test_nvenc_pipeline_and_explicit_fallback():
    choice = select_encoder(default_config(), EncoderSupport(nvenc_pyav=True, nvenc_gstreamer=True))
    pipeline = gstreamer_pipeline(choice, default_config())
    assert "nvh264enc" in pipeline
    assert "bframes=0" in pipeline
    assert "gop-size=15" in pipeline
    assert "zerolatency=true" in pipeline

    missing = select_encoder(
        parse_streamer_config({"encoder": "nvenc"}),
        EncoderSupport(),
    )
    assert missing.aiortc_codec == "libx264"
    assert missing.fallback_from == "nvenc"
    assert missing.bframes == 0


def test_pi_v4l2_recipe_has_no_bframes():
    support = EncoderSupport(v4l2h264enc=True, x264enc_gstreamer=True)
    choice = select_encoder(default_config(), support)
    assert choice.pipeline_encoder == "v4l2h264enc"
    assert choice.aiortc_codec == "libx264"
    pipeline = gstreamer_pipeline(choice, default_config())
    assert "v4l2h264enc" in pipeline
    assert "video_bitrate=3000000" in pipeline
    assert "h264_i_frame_period=15" in pipeline
    assert "profile=baseline" in pipeline
    assert "bframes=1" not in pipeline

    fallback = select_encoder(parse_streamer_config({"encoder": "v4l2"}), EncoderSupport())
    assert fallback.pipeline_encoder == "x264enc"
    assert fallback.fallback_from == "v4l2"


def test_turn_server_is_added_to_the_pipeline():
    config = parse_streamer_config(
        {
            "turn_urls": ["turn:turn.example.com:3478"],
            "turn_username": "ohho",
            "turn_credential": "secret",
        }
    )
    choice = select_encoder(config, EncoderSupport())
    pipeline = gstreamer_pipeline(choice, config)
    assert "turn-server=turn://ohho:secret@turn.example.com:3478" in pipeline


def test_bframes_cannot_be_selected():
    with pytest.raises(ValueError, match="bframes"):
        EncoderChoice(
            pipeline_encoder="x264enc",
            aiortc_codec="libx264",
            bframes=2,
            gop=15,
            bitrate_bps=3_000_000,
            width=1280,
            height=720,
            fps=30,
        )


def test_detect_encoder_support_uses_the_injected_runner():
    def run(argv):
        text = " ".join(argv)
        if "h264_nvenc" in text or "nvh264enc" in text:
            return 0
        if "nvidia-smi" in text:
            return 0
        return 1

    support = detect_encoder_support(run)
    assert support.nvenc_pyav is True
    assert support.nvenc_gstreamer is True
    assert support.nvidia_gpu is True
    assert support.v4l2h264enc is False
    assert support.x264enc_gstreamer is False


def test_detect_encoder_support_survives_a_failing_probe():
    support = detect_encoder_support()
    assert isinstance(support.nvenc_pyav, bool)
    assert isinstance(support.nvenc_gstreamer, bool)
    assert isinstance(support.v4l2h264enc, bool)
    assert isinstance(support.nvidia_gpu, bool)


def test_aiortc_encoder_uses_short_gop_and_no_bframes():
    pytest.importorskip("aiortc")
    import av
    from aiortc.codecs.h264 import H264Encoder

    assert x264_options(15)["bf"] == "0"
    assert x264_options(15)["tune"] == "zerolatency"
    assert x264_options(15)["g"] == "15"
    assert install_low_latency_h264(gop=15, bitrate_bps=2_000_000, codec_name="libx264") is True
    encoder = H264Encoder()
    assert encoder.target_bitrate == 2_000_000
    image = np.zeros((64, 64, 3), dtype=np.uint8)
    frame = av.VideoFrame.from_ndarray(image, format="rgb24")
    frame.pts = 0
    frame.time_base = fractions.Fraction(1, 30)
    packets, _timestamp = encoder.encode(frame, force_keyframe=True)
    assert packets
    # libx264 consumes private options such as bf/tune while opening.
    # The public context fields are what the encoder keeps.
    assert encoder.codec.max_b_frames == 0
    assert encoder.codec.gop_size == 15
    assert encoder.codec.profile == "Baseline"
