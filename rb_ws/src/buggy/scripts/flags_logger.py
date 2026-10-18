#!/usr/bin/env python3
"""Publishes entire feature flag configuration to /flags
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from util.flags import Flags

TOPIC = "/flags"


class FlagsLogger(Node):
    def __init__(self):
        super().__init__("flags_logger")

        flags = json.dumps(Flags(self).as_dict(), sort_keys=True)

        # Transient local ensures this topic is recorded inside a rosbag, 
        # even if recording starts after this node is launched
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(String, TOPIC, qos)
        self._publisher.publish(String(data=flags))
        self.get_logger().info("flags: " + flags)


def main():
    rclpy.init()
    rclpy.spin(FlagsLogger())


if __name__ == "__main__":
    main()