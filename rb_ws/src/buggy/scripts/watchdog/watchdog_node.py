#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

class Watchdog(Node):

    def __init__(self):
        """
        Constructor for Watchdog class.

        Creates a ROS node with a publisher that periodically sends a message
        indicating whether the node is still alive.
        
        """
        super().__init__('watchdog')

        # Publishers
        self.heartbeat_publisher = self.create_publisher(Bool, 'self/debug/heartbeat', 1)

        # Subscribers
        self.heartbeat_subscriber = self.create_subscription(Bool, 'self/debug/heartbeat', self.heartbeat_listener, 1)

        timer_period = 0.01  # seconds (100 Hz)
        self.timer = self.create_timer(timer_period, self.loop)
        self.i = 0 # Loop Counter

    def loop(self):
        # Loop for the code that operates every 10ms
        msg = Bool()
        msg.data = True
        self.heartbeat_publisher.publish(msg)

    def heartbeat_listener(self, msg : Node):
        """
        Subscriber Function that checks if Heartbeat is ever alse
        It never actually is false, this is just a demonstration of a subscriber
        If it ever actually is false, something cursed has happened
        """
        if msg.data == False:
            self.get_logger().error("Hearbeat Failed!")


def main(args=None):
    rclpy.init(args=args)

    watchdog = Watchdog()

    rclpy.spin(watchdog)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    watchdog.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()