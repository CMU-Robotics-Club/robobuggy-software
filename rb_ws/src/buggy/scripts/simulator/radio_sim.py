#! /usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
import time
import random
from nav_msgs.msg import Odometry

class Radio(Node):
    def __init__(self):
        super().__init__('radio')
        self.get_logger().info('INITIALIZED')

        self.state_subscriber = self.create_subscription(
            Odometry, "/NAND/self/state", self.republish, 1
        )

        self.state_publisher = self.create_publisher(Odometry, "/SC/other/stateNoUKF", 1)

    def republish(self, msg):
        if random.randint(0, 20) < 1:
            self.state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    radio = Radio()

    rclpy.spin(radio)

    radio.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()