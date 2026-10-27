# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""RGB frames, 720p scale, and the capture-time overlay used for latency photos."""

from __future__ import annotations

import numpy as np

# 5x7 glyphs, five bits per row, high bit on the left. Lowercase ASCII.
FONT: dict[str, tuple[int, ...]] = {
    "0": (0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110),
    "1": (0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110),
    "2": (0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111),
    "3": (0b11110, 0b00001, 0b00001, 0b01110, 0b00001, 0b00001, 0b11110),
    "4": (0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010),
    "5": (0b11111, 0b10000, 0b10000, 0b11110, 0b00001, 0b00001, 0b11110),
    "6": (0b01110, 0b10000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110),
    "7": (0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000),
    "8": (0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110),
    "9": (0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00001, 0b01110),
    "a": (0b01110, 0b00001, 0b01111, 0b10001, 0b10001, 0b10001, 0b01111),
    "c": (0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110),
    "e": (0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111),
    "f": (0b00111, 0b00100, 0b01110, 0b00100, 0b00100, 0b00100, 0b00100),
    "g": (0b01111, 0b10001, 0b10001, 0b01111, 0b00001, 0b10001, 0b01110),
    "h": (0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001),
    "i": (0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110),
    "n": (0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001),
    "o": (0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110),
    "r": (0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001),
    "s": (0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110),
    "t": (0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100),
    "w": (0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b01010, 0b01010),
    " ": (0, 0, 0, 0, 0, 0, 0),
}

_PATTERN_RGB = {
    "chase": (20, 40, 90),
    "front": (20, 90, 40),
    "wrist": (90, 30, 30),
}


def stamp_label(name: str, stamp_ns: int) -> str:
    """Text burned into the frame: track name and capture time in Unix milliseconds."""
    return f"{name} {stamp_ns // 1_000_000}"


def scale_rgb(image: np.ndarray, width: int, height: int) -> np.ndarray:
    """Nearest-neighbour scale. ``image`` is HxWx3 uint8."""
    src_h, src_w = image.shape[:2]
    if src_w == width and src_h == height:
        return image
    ys = (np.arange(height) * src_h) // height
    xs = (np.arange(width) * src_w) // width
    return np.ascontiguousarray(image[ys][:, xs])


def draw_text(image: np.ndarray, text: str, x: int = 8, y: int = 8, scale: int = 3) -> None:
    """Draw a light glyph string on ``image`` in place. Pixels outside the frame are clipped."""
    text = text.lower()
    pad = scale
    glyph_w = 6 * scale
    glyph_h = 7 * scale
    height, width = image.shape[:2]
    x0, y0 = max(0, x), max(0, y)
    x1 = min(width, x + pad * 2 + glyph_w * max(len(text), 1))
    y1 = min(height, y + pad * 2 + glyph_h)
    if x0 < x1 and y0 < y1:
        image[y0:y1, x0:x1] = 0
    cursor_x = x + pad
    cursor_y = y + pad
    for char in text:
        glyph = FONT.get(char)
        if glyph is not None:
            for row, bits in enumerate(glyph):
                for col in range(5):
                    if bits & (1 << (4 - col)):
                        px = cursor_x + col * scale
                        py = cursor_y + row * scale
                        if px >= width or py >= height or px < 0 or py < 0:
                            continue
                        image[py : min(height, py + scale), px : min(width, px + scale)] = 255
        cursor_x += glyph_w


def draw_live_bar(image: np.ndarray, stamp_ns: int) -> None:
    """A thin bar whose position changes with the capture clock."""
    height, width = image.shape[:2]
    if width <= 4:
        return
    x = int((stamp_ns // 1_000_000) % (width - 4))
    image[height // 2 :, x : x + 4] = (255, 255, 0)


def prepare_frame(
    image: np.ndarray,
    name: str,
    stamp_ns: int,
    width: int,
    height: int,
) -> np.ndarray:
    """Scale to the encoder size and burn the capture timestamp."""
    scaled = scale_rgb(image, width, height)
    if scaled is image:
        scaled = image.copy()
    draw_live_bar(scaled, stamp_ns)
    draw_text(scaled, stamp_label(name, stamp_ns))
    return scaled


def waiting_frame(name: str, width: int, height: int, stamp_ns: int) -> np.ndarray:
    image = np.zeros((height, width, 3), dtype=np.uint8)
    draw_text(image, f"{name} waiting")
    draw_live_bar(image, stamp_ns)
    return image


def test_pattern(name: str, width: int, height: int, stamp_ns: int) -> np.ndarray:
    rgb = _PATTERN_RGB.get(name, (30, 30, 30))
    image = np.empty((height, width, 3), dtype=np.uint8)
    image[:, :] = rgb
    return image


def decode_ros_image(
    *,
    width: int,
    height: int,
    encoding: str,
    step: int,
    data: object,
    stamp_sec: int,
    stamp_nanosec: int,
) -> tuple[np.ndarray, int]:
    """Convert a ``sensor_msgs/Image``-shaped payload to RGB uint8 and a nanosecond stamp.

    Supported encodings are ``rgb8``, ``bgr8``, and ``mono8``.
    """
    if width < 1 or height < 1:
        raise ValueError("image has no pixels")
    kind = encoding.lower()
    channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(kind)
    if channels is None:
        raise ValueError(f"unsupported encoding {encoding}")
    row_bytes = width * channels
    if step < row_bytes:
        raise ValueError("step is smaller than one row")
    raw = bytes(data)
    need = step * height
    if len(raw) < need:
        raise ValueError("image data is short")
    if step == row_bytes:
        packed = raw[: need]
    else:
        packed = b"".join(raw[y * step : y * step + row_bytes] for y in range(height))
    if kind == "mono8":
        gray = np.frombuffer(packed, dtype=np.uint8).reshape(height, width)
        rgb = np.repeat(gray[:, :, None], 3, axis=2)
    else:
        rgb = np.frombuffer(packed, dtype=np.uint8).reshape(height, width, 3)
        if kind == "bgr8":
            rgb = rgb[:, :, ::-1]
        rgb = np.ascontiguousarray(rgb)
    stamp_ns = int(stamp_sec) * 1_000_000_000 + int(stamp_nanosec)
    return rgb, stamp_ns
