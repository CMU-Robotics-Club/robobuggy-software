#! /usr/bin/env python3
# Runs the conversion script for all telematics data

import rclpy
import utm
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class Telematics(Node):
    """
    Converts subscribers and publishers that need to be reformated, so that they are readible.
    """

    def __init__(self):
        """Generate all the subscribers and publishers that need to be reformatted.
        """
        super().__init__('telematics')

        # Implements behavior of callback_args from rospy.Subscriber
        def wrap_args(callback, callback_args):
            return lambda msg: callback(msg, callback_args)

        self.self_publisher = self.create_publisher(NavSatFix, "self/state_navsatfix", 10)
        self.self_subscriber = self.create_subscription(Odometry, "self/state", wrap_args(self.convert_buggystate, self.self_publisher), 1)

        self.other_publisher = self.create_publisher(NavSatFix, "other/state_navsatfix", 10)
        self.other_subscriber = self.create_subscription(Odometry, "other/state", wrap_args(self.convert_buggystate, self.other_publisher), 1)
        
        self.other_estim_publisher = self.create_publisher(NavSatFix, "other/stateNoUKF_navsatfix", 10)
        self.other_estim_subscriber = self.create_subscription(Odometry, "other/stateNoUKF", wrap_args(self.convert_buggystate, self.other_estim_publisher), 1)

    # TODO Make this a static method?
    def convert_buggystate(self, msg, publisher):
        """Converts BuggyState/Odometry message to NavSatFix and publishes to specified publisher
        
        Args:
            msg (Odometry): Buggy state to convert
            publisher (Publisher): Publisher to send NavSatFix message to
        """
        try:
            y = msg.pose.pose.position.y
            x = msg.pose.pose.position.x
            lat, long = utm.to_latlon(x, y, 17, "T")
            down = msg.pose.pose.position.z
            new_msg = NavSatFix()
            new_msg.header = msg.header
            new_msg.latitude = lat
            new_msg.longitude = long
            new_msg.altitude = down
            publisher.publish(new_msg)
            # self.get_logger().info(
            #     f"Converted other buggy estimate position to lat long for navsat: {lat}, {long}"
            # )

        except Exception as e:
            self.get_logger().debug(
                "Unable to convert buggy position to lon lat" + str(e)
            )


if __name__ == "__main__":
    rclpy.init()
    telem = Telematics()
    rclpy.spin(telem)

    telem.destroy_node()
    rclpy.shutdown()