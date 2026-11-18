# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Pure decisions for the VR sim cameras. No ROS imports.

sim.launch.py and sim_cameras.launch.py load this file by path.
The CPU test loads it the same way.
"""

LOOPBACK_ADDRESS = "127.0.0.1"
VIDEO_PORT = 8080

# Quest 3 panel size. Four always_on cameras at this size are the VR
# profile. sim-smoke stays on the minimal 160x120 front camera.
QUEST_WIDTH = 960
QUEST_HEIGHT = 540
QUEST_FPS = 30

FRONT_TOPIC = "/camera/front/image_raw"
WRIST_TOPIC = "/camera/wrist/image_raw"
BEV_TOPIC = "/camera/base/bev/image_raw"
CHASE_TOPIC = "/sim/chase_cam/image_raw"

# RobotProfileFactory.BuildTopics images, plus the third-person view.
IMAGE_TOPICS = (FRONT_TOPIC, WRIST_TOPIC, BEV_TOPIC, CHASE_TOPIC)

# Front image and camera_info stay in sim_bridge.yaml for every launch.
EXTRA_IMAGE_TOPICS = (WRIST_TOPIC, BEV_TOPIC, CHASE_TOPIC)


def truthy(raw):
    """True for 1 / true / yes. Booleans stringify to True and False."""
    return str(raw).strip().lower() in ("1", "true", "yes")


def vr_enabled(raw):
    """vr:=true turns on the Quest camera set. Default is false."""
    return truthy(raw)


def xacro_flag(enabled):
    """xacro arg value. Only the strings true and false."""
    return "true" if enabled else "false"


def start_web_video(vr, video):
    """web_video_server runs only on the VR profile, and only if video is on.

    video defaults to true. vr:=false ignores it so sim-smoke does not
    bind port 8080.
    """
    return vr_enabled(vr) and truthy(video)


def camera_info_topic(image_topic):
    suffix = "/image_raw"
    if not str(image_topic).endswith(suffix):
        raise ValueError(f"camera topic must end in {suffix}: {image_topic}")
    return str(image_topic)[: -len(suffix)] + "/camera_info"


def extra_bridge_topics():
    """Image and camera_info topics added by sim_bridge_cameras.yaml."""
    topics = []
    for image in EXTRA_IMAGE_TOPICS:
        topics.append(image)
        topics.append(camera_info_topic(image))
    return tuple(topics)


def stream_path(topic=CHASE_TOPIC):
    """Path on web_video_server. Host and port are separate."""
    return f"/stream?topic={topic}"


def bind_address(raw):
    """Return a bind address. Empty input stays on loopback."""
    if raw is None:
        return LOOPBACK_ADDRESS
    text = str(raw).strip()
    if not text:
        return LOOPBACK_ADDRESS
    return text


def public_bind_warning(address):
    """Empty when the server stays on loopback."""
    bound = bind_address(address)
    if bound == LOOPBACK_ADDRESS:
        return ""
    return (
        f"web_video_server address {bound} is not loopback. "
        "That exposes the camera streams without authentication."
    )
