#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import pyproj
from scipy.spatial.transform import Rotation
import math

def is_reasonable(value, minimum, maximum, nan_allowed=False):
    """
    Takes in a floating point value (either normal maths or numpy)
    And checks whether its in the range
    If nan_allowed is true, and the value is nan, it returns true, else false
    """
    if value is None:
        return False
    if math.isnan(value):
        return nan_allowed
    if minimum <= value <= maximum:
        return True
    return False


def is_reasonable_msg(
    msg,
    POSITION_MIN=(-1.0e7, -1.0e7, -1.0e7),
    POSITION_MAX=(1.0e7, 1.0e7, 1.0e7),
    ORIENTATION_MIN=(-2.0 * math.pi, -2.0 * math.pi, -2.0 * math.pi, -1.1),
    ORIENTATION_MAX=(2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi, 1.1),
    COVARIANCE_VAL_MIN=0.0,
    COVARIANCE_VAL_MAX=1.0e6,
    LINEAR_TWIST_MIN=(-100.0, -100.0, -100.0),
    LINEAR_TWIST_MAX=(100.0, 100.0, 100.0),
    ANGULAR_TWIST_MIN=(-20.0, -20.0, -20.0),
    ANGULAR_TWIST_MAX=(20.0, 20.0, 20.0),
    NAN_ALLOWED=False,
):
    """
    Check whether an Odometry message stays within configured sanity bounds.
    Position, orientation, linear twist, and angular twist limits are passed as
    per-axis tuples. Covariance limits are scalar bounds applied to every
    covariance entry. The defaults are intentionally broad and meant to catch
    obvious corruption; individual callers should override them with tighter
    ranges that match their message units.
    """
    return (
        # Check pose position values.
        is_reasonable(msg.pose.pose.position.x, POSITION_MIN[0], POSITION_MAX[0], NAN_ALLOWED)
        and is_reasonable(msg.pose.pose.position.y, POSITION_MIN[1], POSITION_MAX[1], NAN_ALLOWED)
        and is_reasonable(msg.pose.pose.position.z, POSITION_MIN[2], POSITION_MAX[2], NAN_ALLOWED)
        # Check pose orientation values.
        and is_reasonable(msg.pose.pose.orientation.x, ORIENTATION_MIN[0], ORIENTATION_MAX[0], NAN_ALLOWED)
        and is_reasonable(msg.pose.pose.orientation.y, ORIENTATION_MIN[1], ORIENTATION_MAX[1], NAN_ALLOWED)
        and is_reasonable(msg.pose.pose.orientation.z, ORIENTATION_MIN[2], ORIENTATION_MAX[2], NAN_ALLOWED)
        and is_reasonable(msg.pose.pose.orientation.w, ORIENTATION_MIN[3], ORIENTATION_MAX[3], NAN_ALLOWED)
        # Check pose covariance values.
        and all(
            is_reasonable(value, COVARIANCE_VAL_MIN, COVARIANCE_VAL_MAX, NAN_ALLOWED)
            for value in msg.pose.covariance
        )
        # Check linear twist values.
        and is_reasonable(msg.twist.twist.linear.x, LINEAR_TWIST_MIN[0], LINEAR_TWIST_MAX[0], NAN_ALLOWED)
        and is_reasonable(msg.twist.twist.linear.y, LINEAR_TWIST_MIN[1], LINEAR_TWIST_MAX[1], NAN_ALLOWED)
        and is_reasonable(msg.twist.twist.linear.z, LINEAR_TWIST_MIN[2], LINEAR_TWIST_MAX[2], NAN_ALLOWED)
        # Check angular twist values.
        and is_reasonable(msg.twist.twist.angular.x, ANGULAR_TWIST_MIN[0], ANGULAR_TWIST_MAX[0], NAN_ALLOWED)
        and is_reasonable(msg.twist.twist.angular.y, ANGULAR_TWIST_MIN[1], ANGULAR_TWIST_MAX[1], NAN_ALLOWED)
        and is_reasonable(msg.twist.twist.angular.z, ANGULAR_TWIST_MIN[2], ANGULAR_TWIST_MAX[2], NAN_ALLOWED)
        # Check twist covariance values.
        and all(
            is_reasonable(value, COVARIANCE_VAL_MIN, COVARIANCE_VAL_MAX, NAN_ALLOWED)
            for value in msg.twist.covariance
        )
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
        if not is_reasonable_msg(
            msg,
            # approx range of pittsburgh in ECEF
            POSITION_MIN=(0.8e6, -4.9e6, 4.0e6),
            POSITION_MAX=(0.9e6, -4.7e6, 4.2e6),
            # quaternion orientation of xyzw
            ORIENTATION_MIN=(-1.1, -1.1, -1.1, -1.1),
            ORIENTATION_MAX=(1.1, 1.1, 1.1, 1.1),
            COVARIANCE_VAL_MIN=0.0,
            COVARIANCE_VAL_MAX=1.0e6,
            # m/s
            LINEAR_TWIST_MIN=(-100.0, -100.0, -100.0),
            LINEAR_TWIST_MAX=(100.0, 100.0, 100.0),
            # rad/s
            ANGULAR_TWIST_MIN=(-20.0, -20.0, -20.0),
            ANGULAR_TWIST_MAX=(20.0, 20.0, 20.0),
        ):
            return
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
        if not is_reasonable_msg(
            msg,
            # UTM (easting, northing, altitude) for Pittsburgbh
            POSITION_MIN=(5.0e5, 4.4e6, -250),
            POSITION_MAX=(6.1e5, 4.6e6, 2000),
            # -2pi to 2 pi for x,y,z, -1, 1 for w
            ORIENTATION_MIN=(-2.0 * math.pi, -2.0 * math.pi, -2.0 * math.pi, -1.0),
            ORIENTATION_MAX=(2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi, 1.0),
            COVARIANCE_VAL_MIN=0.0,
            COVARIANCE_VAL_MAX=1.0e6,
            # -100 to 100 m/s
            LINEAR_TWIST_MIN=(-100.0, -100.0, -100.0),
            LINEAR_TWIST_MAX=(100.0, 100.0, 100.0),
            # -20 to 20 radians per second
            ANGULAR_TWIST_MIN=(-20.0, -20.0, -20.0),
            ANGULAR_TWIST_MAX=(20.0, 20.0, 20.0),
        ):
            return

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
