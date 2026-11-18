"""Launch the OHH-210 WebRTC streamer.

The process listens on 127.0.0.1:8766. Put the OHH-196 gateway in front of it:

    ohho-gateway --webrtc http://127.0.0.1:8766

Clients then use wss://<robot>:8443/webrtc/signaling?token=<TOKEN>. The gateway
strips the /webrtc prefix, so this node serves /signaling.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("ohho_webrtc"),
        "config",
        "webrtc_params.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="ohho_webrtc",
                executable="webrtc_streamer",
                name="ohho_webrtc",
                output="screen",
                parameters=[config],
            )
        ]
    )
