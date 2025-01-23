#!/usr/bin/env python3
import argparse

import rclpy
from std_msgs.msg import Float64
import numpy as np


"""
Debug Controller
Sends oscillating steering command for firmware and system level debug
"""
class DebugController(Node):

    """
    @input: self_name, for namespace for current buggy
    Initializes steer publisher to publish steering angles
    Tick = 1ms
    """
    def __init__(self) -> None:
        super().__init__("debug_steer")
        self.steer_publisher = self.create_publisher(
            Float64, "/buggy/input/steering", 10)
        self.rate = 1000  # Hz
        self.tick_count = 0
        self.steer_cmd = 0.0

        # Create a timer to call the loop function
        self.timer = self.create_timer(1.0 / self.rate, self.loop)

    # Outputs a continuous sine wave ranging from -50 to 50, with a period of 500 ticks
    def sin_steer(self, tick_count):
        return 50 * np.sin((2 * np.pi) * tick_count/500)

    #returns a constant steering angle of 42 degrees
    def constant_steer(self, _):
        return 42.0

    #Creates a loop based on tick counter
    def loop(self):
        self.steer_cmd = self.sin_steer(self.tick_count)
        msg = Float64()
        msg.data = self.steer_cmd
        self.steer_publisher.publish(msg)

        self.tick_count += 1


def main(args=None):
    rclpy.init(args=args)

    debug_steer = DebugController()

    rclpy.spin(debug_steer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    debug_steer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
