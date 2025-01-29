#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import pyproj
from scipy.spatial.transform import Rotation

class BuggyStateConverter(Node):
    def __init__(self):
        super().__init__("buggy_state_converter")
        self.get_logger().info('INITIALIZED.')

        namespace = self.get_namespace()
        if namespace == "/SC":
            self.SC_raw_state_subscriber = self.create_subscription(
                Odometry, "/ekf/odometry_earth", self.convert_SC_state_callback, 10
            )

            self.NAND_other_raw_state_subscriber = self.create_subscription(
                Odometry, "NAND_raw_state", self.convert_NAND_other_state_callback, 10
            )

            self.other_state_publisher = self.create_publisher(Odometry, "other/state", 10)

        elif namespace == "/NAND":
            self.NAND_raw_state_subscriber = self.create_subscription(
                Odometry, "raw_state", self.convert_NAND_state_callback, 10
            )

        else:
            self.get_logger().warn(f"Namespace not recognized for buggy state conversion: {namespace}")

        self.self_state_publisher = self.create_publisher(Odometry, "self/state", 10)

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

        # ---- 4. Copy Linear Velocities ----
        converted_msg.twist.twist.linear.x = msg.twist.twist.linear.x   # m/s in x-direction
        converted_msg.twist.twist.linear.y = msg.twist.twist.linear.y   # m/s in x-direction
        converted_msg.twist.twist.linear.z = msg.twist.twist.linear.z   # keep original Z velocity

        # ---- 5. Copy Angular Velocities ----
        converted_msg.twist.twist.angular.x = msg.twist.twist.angular.x   # copying over
        converted_msg.twist.twist.angular.y = msg.twist.twist.angular.y   # copying over
        converted_msg.twist.twist.angular.z = msg.twist.twist.angular.z   # rad/s, heading change rate

        return converted_msg

    def convert_NAND_state(self, msg):
        """
        Converts self/raw_state in NAND namespace to clean state units and structure
        Takes in ROS message in nav_msgs/Odometry format
        """

        converted_msg = Odometry()
        converted_msg.header = msg.header

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
        converted_msg.header = msg.header

        # ---- 1. Directly use UTM Coordinates ----
        converted_msg.pose.pose.position.x = msg.x    # UTM Easting
        converted_msg.pose.pose.position.y = msg.y    # UTM Northing
        converted_msg.pose.pose.position.z = msg.z    # UTM Altitude (not provided in other/raw_state, defaults to 0.0)

        # ---- 2. Orientation in Radians ----
        converted_msg.pose.pose.orientation.x = msg.roll      # (roll not provided in other/raw_state, defaults to 0.0)
        converted_msg.pose.pose.orientation.y = msg.pitch     # (pitch not provided in other/raw_state, defaults to 0.0)
        converted_msg.pose.pose.orientation.z = msg.heading   # heading in radians
        converted_msg.pose.pose.orientation.w = 0.0           # fourth quaternion term irrelevant for euler angles

        # ---- 3. Copy Covariances (Unchanged) ----
        converted_msg.pose.covariance = msg.pose_covariance     # (not provided in other/raw_state)
        converted_msg.twist.covariance = msg.twist_covariance   # (not provided in other/raw_state)

        # ---- 4. Linear Velocities in m/s ----
        # Convert scalar speed to velocity x/y components using heading (msg.heading)
        speed = msg.speed       # m/s scalar velocity
        heading = msg.heading   # heading in radians

        # Calculate velocity components
        converted_msg.twist.twist.linear.x = speed * np.cos(heading)    # m/s in x-direction
        converted_msg.twist.twist.linear.y = speed * np.sin(heading)    # m/s in y-direction
        converted_msg.twist.twist.linear.z = 0.0

        # ---- 5. Angular Velocities ----
        converted_msg.twist.twist.angular.x = msg.roll_rate      # (roll rate not provided in other/raw_state, defaults to 0.0)
        converted_msg.twist.twist.angular.y = msg.pitch_rate     # (pitch rate not provided in other/raw_state, defaults to 0.0)
        converted_msg.twist.twist.angular.z = msg.heading_rate   # rad/s, heading change rate

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
