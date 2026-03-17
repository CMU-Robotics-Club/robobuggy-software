#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import pyproj
from scipy.spatial.transform import Rotation
import math

def is_reasonable (value, minimum, maximum, nan_allowed=False):
    if value is None:
        return False
    if math.is_nan(value):
        return nan_allowed
    if value >= minimum & value <= maximum:
        return True
    return False

def is_reasonable_pos(position, minimum=1000, maximum=5000, nan_allowed=False):
    return (
        is_reasonable(position.x, minimum, maximum, nan_allowed)
        and is_reasonable(position.y, minimum, maximum, nan_allowed)
        and is_reasonable(position.z, minimum, maximum, nan_allowed)
    )

def is_reasonable_orientation(orientation, minimum=-1.0, maximum=1.0, nan_allowed=False):
    return (
        is_reasonable(orientation.x, minimum, maximum, nan_allowed)
        and is_reasonable(orientation.y, minimum, maximum, nan_allowed)
        and is_reasonable(orientation.z, minimum, maximum, nan_allowed)
        and is_reasonable(orientation.w, minimum, maximum, nan_allowed)
    )

def is_reasonable_covariance(covariance, minimum=0.0, maximum=10.0, nan_allowed=False):
    return all(
        is_reasonable(value, minimum, maximum, nan_allowed)
        for value in covariance
    )


def is_reasonable_linear_twist(linear, minimum=-20.0, maximum=20.0, nan_allowed=False):
    return (
        is_reasonable(linear.x, minimum, maximum, nan_allowed)
        and is_reasonable(linear.y, minimum, maximum, nan_allowed)
        and is_reasonable(linear.z, minimum, maximum, nan_allowed)
    )

def is_reasonable_angular_twist(angular, minimum=-10.0, maximum=10.0, nan_allowed=False):
    return (
        is_reasonable(angular.x, minimum, maximum, nan_allowed)
        and is_reasonable(angular.y, minimum, maximum, nan_allowed)
        and is_reasonable(angular.z, minimum, maximum, nan_allowed)
    )

def is_reasonable_msg(msg):
    return (
        is_reasonable_pos(msg.pose.pose.position)
        and is_reasonable_orientation(msg.pose.pose.orientation)
        and is_reasonable_covariance(msg.pose.covariance)
        and is_reasonable_linear_twist(msg.twist.twist.linear)
        and is_reasonable_angular_twist(msg.twist.twist.angular)
        and is_reasonable_covariance(msg.twist.covariance)
    )

class BuggyStateConverter(Node):
    def __init__(self):
        super().__init__("buggy_state_converter")
        self.get_logger().info('INITIALIZED.')

        namespace = self.get_namespace()
        if namespace == "/SC":
            self.SC_raw_state_subscriber = self.create_subscription(
                Odometry, "/ekf/odometry_earth", self.convert_SC_state_callback, 1
            )

            self.NAND_other_raw_state_subscriber = self.create_subscription(
                Odometry, "NAND_raw_state", self.convert_NAND_other_state_callback, 1
            )

            self.other_state_publisher = self.create_publisher(Odometry, "other/stateNoUKF", 1)

        elif namespace == "/NAND":
            self.NAND_raw_state_subscriber = self.create_subscription(
                Odometry, "raw_state", self.convert_NAND_state_callback, 1
            )

        else:
            self.get_logger().warn(f"Namespace not recognized for buggy state conversion: {namespace}")

        self.self_state_publisher = self.create_publisher(Odometry, "self/state", 1)

        # Initialize pyproj Transformer for ECEF -> UTM conversion for /SC
        self.ecef_to_utm_transformer = pyproj.Transformer.from_crs(
            "epsg:4978", "epsg:32617", always_xy=True
        )  # TODO: Confirm UTM EPSG code, using EPSG:32617 for UTM Zone 17N


    def convert_SC_state_callback(self, msg):
        """ Callback for processing SC/raw_state messages and publishing to self/state """
        converted_msg = self.convert_SC_state(msg)
        self.self_state_publisher.publish(converted_msg)

    def convert_NAND_state_callback(self, msg):
        """ Callback for processing NAND/raw_state messages and publishing to self/state """
        converted_msg = self.convert_NAND_state(msg)
        self.self_state_publisher.publish(converted_msg)

    def convert_NAND_other_state_callback(self, msg):
        """ Callback for processing SC/NAND_raw_state messages and publishing to other/state """
        converted_msg = self.convert_NAND_other_state(msg)
        self.other_state_publisher.publish(converted_msg)

    def convert_SC_state(self, msg):
        """
        Converts self/raw_state in SC namespace to clean state units and structure

        Takes in ROS message in nav_msgs/Odometry format
        Assumes that the SC namespace is using ECEF coordinates and quaternion orientation
        """

        converted_msg = Odometry()
        converted_msg.header = msg.header

        # Header timestamps/frame_id are overwritten to track control stack latency
        ns = time.time_ns()

        # Arbitrary frame_id firmware timestamp for INS sourced data
        converted_msg.header.frame_id = "0"

        converted_msg.header.stamp.sec = ((ns // int(1e9)) + 2**31) % 2**32 - 2**31
        converted_msg.header.stamp.nanosec = ns % int(1e9)

        # ---- 1. Convert ECEF Position to UTM Coordinates ----
        ecef_x = msg.pose.pose.position.x
        ecef_y = msg.pose.pose.position.y
        ecef_z = msg.pose.pose.position.z

        # Convert ECEF to UTM
        utm_x, utm_y, utm_z = self.ecef_to_utm_transformer.transform(ecef_x, ecef_y, ecef_z)
        converted_msg.pose.pose.position.x = utm_x  # UTM Easting
        converted_msg.pose.pose.position.y = utm_y  # UTM Northing
        converted_msg.pose.pose.position.z = utm_z  # UTM Altitude

        # ---- 2. Convert Quaternion to Heading (Radians) ----
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w

        # Use Rotation.from_quat to get roll, pitch, yaw
        roll, pitch, yaw = Rotation.from_quat([qx, qy, qz, qw]).as_euler('xyz')
        # roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])  # tf_transformations bad

        # Store the heading in the x component of the orientation
        converted_msg.pose.pose.orientation.x = roll
        converted_msg.pose.pose.orientation.y = pitch
        converted_msg.pose.pose.orientation.z = yaw
        converted_msg.pose.pose.orientation.w = 0.0   # fourth (quaternion) term irrelevant for euler angles

        # ---- 3. Copy Covariances (Unchanged) ----
        converted_msg.pose.covariance = msg.pose.covariance
        converted_msg.twist.covariance = msg.twist.covariance

        # ---- 4. Copy Linear/Angular Velocities (Unchanged) ----
        converted_msg.twist.twist = msg.twist.twist

        return converted_msg

    def convert_NAND_state(self, msg):
        """
        Converts self/raw_state in NAND namespace to clean state units and structure
        Takes in ROS message in nav_msgs/Odometry format
        """

        converted_msg = Odometry()
        converted_msg.header = msg.header

        # Add software timestamp to header to track control stack latency
        ns = time.time_ns()

        # frame_id is already set in ros2bnyahaj to firmware timestamp

        # Avoid y2k38 (robobuggy WILL exist in 2038)
        # this actually throws an error if we try to assign something outside a 32 bit range
        converted_msg.header.stamp.sec = ((ns // int(1e9)) + 2**31) % 2**32 - 2**31
        converted_msg.header.stamp.nanosec = ns % int(1e9)

        # ---- 1. Directly use UTM Coordinates ----
        converted_msg.pose.pose.position.x = msg.pose.pose.position.x   # UTM Easting
        converted_msg.pose.pose.position.y = msg.pose.pose.position.y   # UTM Northing
        converted_msg.pose.pose.position.z = msg.pose.pose.position.z   # UTM Altitude

        # ---- 2. Orientation in Radians ----
        converted_msg.pose.pose.orientation.x = msg.pose.pose.orientation.x
        converted_msg.pose.pose.orientation.y = msg.pose.pose.orientation.y
        converted_msg.pose.pose.orientation.z = msg.pose.pose.orientation.z
        converted_msg.pose.pose.orientation.w = 0.0   # fourth (quaternion) term irrelevant for euler angles

        # ---- 3. Copy Covariances (Unchanged) ----
        converted_msg.pose.covariance = msg.pose.covariance
        converted_msg.twist.covariance = msg.twist.covariance

        # ---- 4. Linear Velocities in m/s ----
        # Convert scalar speed to velocity x/y components using heading (orientation.z)
        speed = msg.twist.twist.linear.x        # m/s scalar velocity
        heading = msg.pose.pose.orientation.z   # heading in radians

        # Calculate velocity components
        converted_msg.twist.twist.linear.x = speed * np.cos(heading)    # m/s in x-direction
        converted_msg.twist.twist.linear.y = speed * np.sin(heading)    # m/s in y-direction
        converted_msg.twist.twist.linear.z = 0.0

        # ---- 5. Copy Angular Velocities ----
        converted_msg.twist.twist.angular.x = msg.twist.twist.angular.x   # copying over
        converted_msg.twist.twist.angular.y = msg.twist.twist.angular.y   # copying over
        converted_msg.twist.twist.angular.z = msg.twist.twist.angular.z   # rad/s, heading change rate

        return converted_msg

    def convert_NAND_other_state(self, msg):
        """ Converts other/raw_state in SC namespace (NAND data) to clean state units and structure """
        converted_msg = Odometry()

        # No actual changes as the other state is just easting northing, everything else is zeroed
        converted_msg = msg

        return converted_msg


def main(args=None):
    rclpy.init(args=args)

    # Create the BuggyStateConverter node and spin it
    state_converter = BuggyStateConverter()
    rclpy.spin(state_converter)

    # Shutdown when done
    state_converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
