# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
import numpy as np
import pytest

from ohho_webrtc.config import TrackSpec, default_config
from ohho_webrtc.frames import decode_ros_image, draw_text, scale_rgb, stamp_label
from ohho_webrtc.tracks import TrackError, TrackRegistry


def test_registers_default_tracks_and_ignores_unknown_topics():
    config = default_config()
    registry = TrackRegistry(config.tracks)
    assert registry.names() == ("chase", "front", "wrist")
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    assert registry.push("/camera/not/a/track", image, 1) is False
    assert registry.has_frame("chase") is False


def test_push_updates_only_the_matching_track():
    registry = TrackRegistry(default_config().tracks)
    front = np.zeros((48, 64, 3), dtype=np.uint8)
    front[:, :] = (0, 255, 0)
    chase = np.zeros((48, 64, 3), dtype=np.uint8)
    chase[:, :] = (0, 0, 255)
    assert registry.push("/camera/front/image_raw", front, 2_000_000_000) is True
    assert registry.push("/camera/chase/image_raw", chase, 3_000_000_000) is True
    encoded_front = registry.frame_for_encode("front", 64, 48)
    encoded_chase = registry.frame_for_encode("chase", 64, 48)
    assert encoded_front.shape == (48, 64, 3)
    assert encoded_chase.shape == (48, 64, 3)
    # Bottom-right stays the source colour; the overlay sits at the top left.
    assert tuple(int(v) for v in encoded_front[-1, -1]) == (0, 255, 0)
    assert tuple(int(v) for v in encoded_chase[-1, -1]) == (0, 0, 255)
    assert registry.has_frame("wrist") is False


def test_duplicate_registration_is_rejected():
    with pytest.raises(TrackError, match="name"):
        TrackRegistry(
            (
                TrackSpec("chase", "/camera/chase/image_raw"),
                TrackSpec("chase", "/camera/front/image_raw"),
            )
        )
    with pytest.raises(TrackError, match="topic"):
        TrackRegistry(
            (
                TrackSpec("chase", "/camera/front/image_raw"),
                TrackSpec("front", "/camera/front/image_raw"),
            )
        )


def test_overlay_uses_capture_stamp_and_scale():
    registry = TrackRegistry((TrackSpec("chase", "/camera/chase/image_raw"),))
    source = np.zeros((2, 2, 3), dtype=np.uint8)
    source[0, 0] = (10, 20, 30)
    stamp = 1_710_000_000_123_000_000
    registry.push("/camera/chase/image_raw", source, stamp)
    frame = registry.frame_for_encode("chase", 4, 4)
    assert frame.shape == (4, 4, 3)
    assert stamp_label("chase", stamp) == "chase 1710000000123"
    # Nearest neighbour duplicates the source pixel into the top-left 2x2,
    # then the timestamp bar/text may cover it. A second stamp moves the bar.
    other = registry.frame_for_encode("chase", 64, 48)
    registry.push("/camera/chase/image_raw", source, stamp + 50_000_000_000)
    moved = registry.frame_for_encode("chase", 64, 48)
    assert not np.array_equal(other, moved)


def test_waiting_frame_before_the_first_image():
    registry = TrackRegistry((TrackSpec("wrist", "/camera/wrist/image_raw"),))
    frame = registry.frame_for_encode("wrist", 32, 16)
    assert frame.shape == (16, 32, 3)
    assert int(frame.sum()) > 0


def test_digit_one_glyph_is_drawn():
    image = np.zeros((16, 16, 3), dtype=np.uint8)
    draw_text(image, "1", x=0, y=0, scale=1)
    # pad is 1, glyph row 0 is 00100, so pixel (x=3, y=1) is lit and (1, 1) is not.
    assert tuple(int(v) for v in image[1, 3]) == (255, 255, 255)
    assert tuple(int(v) for v in image[1, 1]) == (0, 0, 0)


def test_scale_nearest_neighbour():
    source = np.zeros((2, 2, 3), dtype=np.uint8)
    source[0, 0] = (1, 2, 3)
    source[1, 1] = (4, 5, 6)
    scaled = scale_rgb(source, 4, 4)
    assert tuple(int(v) for v in scaled[0, 0]) == (1, 2, 3)
    assert tuple(int(v) for v in scaled[0, 1]) == (1, 2, 3)
    assert tuple(int(v) for v in scaled[3, 3]) == (4, 5, 6)


def test_decode_bgr_mono_and_padded_rows():
    bgr = np.zeros((2, 2, 3), dtype=np.uint8)
    bgr[0, 0] = (10, 20, 30)  # B, G, R
    rgb, stamp = decode_ros_image(
        width=2,
        height=2,
        encoding="bgr8",
        step=6,
        data=bgr.tobytes(),
        stamp_sec=5,
        stamp_nanosec=250,
    )
    assert stamp == 5_000_000_250
    assert tuple(int(v) for v in rgb[0, 0]) == (30, 20, 10)

    padded = bytes([0, 255, 0xEE, 0xEE, 128, 64, 0xEE, 0xEE])
    mono, _stamp = decode_ros_image(
        width=2,
        height=2,
        encoding="mono8",
        step=4,
        data=padded,
        stamp_sec=0,
        stamp_nanosec=0,
    )
    assert tuple(int(v) for v in mono[0, 1]) == (255, 255, 255)
    assert tuple(int(v) for v in mono[1, 0]) == (128, 128, 128)

    with pytest.raises(ValueError, match="unsupported"):
        decode_ros_image(
            width=1,
            height=1,
            encoding="rgb16",
            step=2,
            data=b"\x00\x00",
            stamp_sec=0,
            stamp_nanosec=0,
        )
