#!/usr/bin/env python3
# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Publish a RobotDescriptor as latched JSON on /ohho/descriptor.

The YAML path is a parameter. OmniBot and the SO-101 arm-only sim each
pass their own file. QoS is transient-local, depth 1, so a subscriber
that joins later still receives the last document. The payload is
assembled from YAML with no safety cap raised.
"""

from __future__ import annotations

import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import descriptor_assemble

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class DescriptorPublisher(Node):
    def __init__(self):
        super().__init__("ohho_descriptor")
        self.declare_parameter("descriptor_yaml", "")
        self.declare_parameter("kind", "real")
        yaml_path = self.get_parameter("descriptor_yaml").get_parameter_value().string_value
        kind = self.get_parameter("kind").get_parameter_value().string_value
        if not yaml_path:
            raise RuntimeError("descriptor_yaml parameter is required")
        document = descriptor_assemble.load_descriptor_yaml(yaml_path, kind=kind)
        payload = descriptor_assemble.dumps_descriptor(document)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        publisher = self.create_publisher(String, "/ohho/descriptor", qos)
        message = String()
        message.data = payload
        publisher.publish(message)
        self.get_logger().info("latched /ohho/descriptor (%d bytes)" % len(payload))


def main():
    rclpy.init()
    node = DescriptorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
