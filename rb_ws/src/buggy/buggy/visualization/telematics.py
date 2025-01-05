#! /usr/bin/env python3
# Runs the conversion script for all telematics data

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Int8

from microstrain_inertial_msgs.msg import MipGnssFixInfo

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

        self.odom_subscriber = self.create_subscription(Odometry, "/NAND/nav/odom", self.convert_odometry_to_navsatfix, 1)
        self.odom_publisher = self.create_publisher(NavSatFix, "/NAND/nav/odom_NavSatFix", 10)

        self.gnss1_pose_publisher = self.create_publisher(PoseStamped, "/gnss1/fix_Pose", 10)
        self.gnss1_covariance_publisher = self.create_publisher(Float64MultiArray, "/gnss1/fix_FloatArray_Covariance", 10)
        self.gnss1_subscriber = self.create_subscription(NavSatFix, "/gnss1/llh_position", wrap_args(self.convert_navsatfix_to_pose_covariance, (self.gnss1_pose_publisher, self.gnss1_covariance_publisher)), 1)

        self.gnss2_pose_publisher = self.create_publisher(PoseStamped, "/gnss2/fix_Pose", 10)
        self.gnss2_covariance_publisher = self.create_publisher(Float64MultiArray, "/gnss2/fix_FloatArray_Covariance", 10)
        self.gnss2_subscriber = self.create_subscription(NavSatFix, "/gnss2/llh_position", wrap_args(self.convert_navsatfix_to_pose_covariance, (self.gnss2_pose_publisher, self.gnss2_covariance_publisher)), 1)

        self.gnss1_fixinfo_publisher = self.create_publisher(String, "/gnss1/fix_info_republished", 10)
        self.gnss1_fixinfo_int_publisher = self.create_publisher(Int8, "/gnss1/fix_info_republished_int", 10)
        self.gnss1_fixinfo_subscriber = self.create_subscription(MipGnssFixInfo, "/mip/gnss1/fix_info", wrap_args(self.republish_fixinfo, (self.gnss1_fixinfo_publisher, self.gnss1_fixinfo_int_publisher)), 1)

        self.gnss2_fixinfo_publisher = self.create_publisher(String, "/gnss2/fix_info_republished", 10)
        self.gnss2_fixinfo_int_publisher = self.create_publisher(Int8, "/gnss2/fix_info_republished_int", 10)
        self.gnss2_fixinfo_subscriber = self.create_subscription(MipGnssFixInfo, "/mip/gnss2/fix_info", wrap_args(self.republish_fixinfo, (self.gnss2_fixinfo_publisher, self.gnss2_fixinfo_int_publisher)), 1)

    def convert_odometry_to_navsatfix(self, msg):
        """Convert Odometry-type to NavSatFix-type for plotting on Foxglove
        Args:
            msg (Odometry): odometry as per INS
        """
        lat = msg.pose.pose.position.y
        long = msg.pose.pose.position.x
        down = msg.pose.pose.position.z
        new_msg = NavSatFix()
        new_msg.header = msg.header
        new_msg.latitude = lat
        new_msg.longitude = long
        new_msg.altitude = down
        self.odom_publisher.publish(new_msg)

    def convert_navsatfix_to_pose_covariance(self, msg, publishers):
        """Convert NavSatFix-type and related covariance matrix to Pose-type and array
        respectively for easy visualization in Foxglove.

        Args:
            msg (NavSatFix): original msg
            publishers (tuple): tuple of publishes
        """
        pose = PoseStamped()
        pose.pose.position.y = msg.latitude
        pose.pose.position.x = msg.longitude
        pose.pose.position.z = msg.altitude
        pose.header = msg.header
        publishers[0].publish(pose)

        covariance = Float64MultiArray()
        covariance.data = [
            round(msg.position_covariance[0], 4),
            round(msg.position_covariance[4], 4),
            round(msg.position_covariance[8], 4)]
        publishers[1].publish(covariance)

    def republish_fixinfo(self, msg, publishers):
        """
        convert gnss/fixinfo to a string for visualization in foxglove
        """
        fix_type = msg.fix_type
        fix_string = "fix type: "

        if (fix_type == 0):
            fix_string += "FIX_3D"
        elif (fix_type == 1):
            fix_string += "FIX_2D"
        elif (fix_type == 2):
            fix_string += "FIX_TIME_ONLY"
        elif (fix_type == 3):
            fix_string += "FIX_NONE"
        elif (fix_type == 4):
            fix_string += "FIX_INVALID"
        elif (fix_type == 5):
            fix_string += "FIX_RTK_FLOAT"
        else:
            fix_string += "FIX_RTK_FIXED"

        # fix_string += "\nsbas_used: "  + str(msg.sbas_used)
        # fix_string += "\ndngss_used: " + str(msg.dngss_used)
        publishers[0].publish(fix_string)
        publishers[1].publish(fix_type)

if __name__ == "__main__":
    rclpy.init()
    telem = Telematics()
    rclpy.spin(telem)

    telem.destroy_node()
    rclpy.shutdown()