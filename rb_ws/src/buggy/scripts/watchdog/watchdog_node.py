#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher

from std_msgs.msg import Int32, Int8

from util.errors import BuggyError


class Watchdog(Node):

    curr_errors: BuggyError = BuggyError(0)
    error_subscriber: Subscription
    led_publisher: Publisher

    def __init__(self):
        """ Constructor for Watchdog class.
        
        """
        super().__init__('watchdog')

        self.error_subscriber = self.create_subscription(
            Int32,
            BuggyError.ros_topic_name,
            self.error_callback,
            10
        )

        self.led_publisher = self.create_publisher(
            Int32,
            "/input/sanity_warning",
            10
        )
        
    def update_status_led(self):
        # Errors
        if BuggyError.REALLY_FREAKING_BAD in self.curr_errors\
                or BuggyError.PATH_PLANNING_FAULT in self.curr_errors\
                or BuggyError.PNEUMATIC_FAULT in self.curr_errors\
                or BuggyError.MOTOR_FAULT in self.curr_errors\
                or BuggyError.SENSOR_FAULT in self.curr_errors\
                or BuggyError.CONTROLLER_CONNECTION_LOST in self.curr_errors:
            self.led_publisher.publish(Int8(data=2))
        # Warnings
        elif BuggyError.LOW_BATTERY in self.curr_errors\
                or BuggyError.SENSOR_UNAVAILABLE in self.curr_errors\
                or BuggyError.VISION_UNAVAILABLE in self.curr_errors\
                or BuggyError.STANLEY_CRAPPED_ITSELF in self.curr_errors\
                or BuggyError.INTERBUGGY_COMMUNICATION_LOST in self.curr_errors\
                or BuggyError.GPS_UNAVAILABLE in self.curr_errors:
            self.led_publisher.publish(Int8(data=1))
        # All good
        else:
            self.led_publisher.publish(Int8(data=0))

    def error_callback(self, msg: Int32):
        if msg.data:
            curr = self.curr_errors
            self.curr_errors |= BuggyError(msg.data)
            if curr != self.curr_errors:
                self.update_status_led()


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