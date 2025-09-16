#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
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
            Float64, "input/steering", 10)
        self.rate = 1000  # Hz
        self.steer_cmd = 0.0

        self.t0 = None

        # sin_steer params
        self.steer_freq = 2  # Hz
        self.steer_range = 50

        # Create a timer to call the loop function
        self.timer = self.create_timer(1.0 / self.rate, self.loop)
        self.get_logger().info("INITIALIZED")

    # Outputs a continuous sine wave
    def sin_steer(self, t):
        return self.steer_range * np.sin(2 * np.pi * self.steer_freq * t)

    # Outputs a stepped version of sin steer
    def step_steer(self, t):
        return self.steer_range * np.sign(self.sin_steer(t))

    #returns a constant steering angle of 42 degrees
    def constant_steer(self, _):
        return 42.0

    #Creates a loop based on tick counter
    def loop(self):
        if self.t0 is None:
            self.t0 = time.time()

        t = time.time() - self.t0

        self.steer_cmd = self.sin_steer(t)
        msg = Float64()
        msg.data = self.steer_cmd
        if self.tick_count % 10 == 0:
            self.get_logger().info(f"STEER: {self.steer_cmd}")
        self.steer_publisher.publish(msg)


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
