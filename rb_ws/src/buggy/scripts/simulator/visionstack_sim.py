#! /usr/bin/env python3
import random
import random
import rclpy
import math
import copy

from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from util.odomToNavsatFix import odom_to_navsat

class VisionStack(Node):
    def __init__(self):
        super().__init__('visionstack')
        self.get_logger().info('INITIALIZED')

        self.self_state_subscriber = self.create_subscription(
            Odometry, "/SC/self/state", self.update_selfstate, 1
        )

        self.other_state_subscriber = self.create_subscription(
            Odometry, "/NAND/self/state", self.republish, 1
        )

        self.cam_publisher = self.create_publisher(Odometry, "/SC/camera/other/state", 1)
        self.navsat_cam_publisher = self.create_publisher(NavSatFix, "/SC/camera/other/pose_navsat", 1)

        self.lidar_publisher = self.create_publisher(Odometry, "/SC/lidar/other/state", 1)
        self.navsat_lidar_publisher = self.create_publisher(NavSatFix, "/SC/lidar/other/pose_navsat", 1)
        
        self.sc_x = None
        self.sc_y = None
        self.sc_heading = None 

    def corrupt_odom(self, msg: Odometry, pos_noise: float = 5.0, angle_noise: float = math.pi) -> Odometry:
        """Return a deep-copied Odometry msg with randomized/noisy values."""
        corrupted = copy.deepcopy(msg)

        # Corrupt position with large gaussian noise
        corrupted.pose.pose.position.x += random.gauss(0, pos_noise)
        corrupted.pose.pose.position.y += random.gauss(0, pos_noise)
        corrupted.pose.pose.position.z += random.gauss(0, angle_noise)  # z = heading in radians

        # Corrupt velocity
        corrupted.twist.twist.linear.x  += random.gauss(0, pos_noise)
        corrupted.twist.twist.linear.y  += random.gauss(0, pos_noise)
        corrupted.twist.twist.angular.z += random.gauss(0, angle_noise)

        return corrupted

    def is_within_fov(self, observer, target, heading, fov_degrees=100):
        """
        Checks if a target point is within the FOV of an observer.
        
        :param observer: (x, y) tuple for the observer
        :param target: (x, y) tuple for the target point
        :param heading: The direction the observer is facing (in degrees)
        :param fov_degrees: The total width of the FOV (default 100)
        :return: Boolean
        """
        # 1. Calculate the angle from observer to target
        dx = target[0] - observer[0]
        dy = target[1] - observer[1]
        
        # math.atan2(y, x) returns the angle in radians
        angle_to_target_rad = math.atan2(dy, dx)
        angle_to_target_deg = math.degrees(angle_to_target_rad)
        
        # 2. Calculate the difference between heading and angle to target
        # We want to know how far "off-center" the target is
        diff = angle_to_target_deg - heading
        
        # 3. Normalize the angle to be between -180 and 180
        # This handles cases where the observer faces 0 and the target is at 350
        diff = (diff + 180) % 360 - 180
        
        # 4. Check if the absolute difference is within half of the FOV
        return abs(diff) <= fov_degrees / 2

        
    def update_selfstate(self, msg):
        self.sc_x = msg.pose.pose.position.x 
        self.sc_y = msg.pose.pose.position.y 
        self.sc_heading = msg.pose.pose.orientation.z 

    def republish(self, msg):
        if self.sc_x is None or self.sc_y is None or self.sc_heading is None:
            self.get_logger().warn("Waiting for self-state position data...")
            return
        nand_x = msg.pose.pose.position.x
        nand_y = msg.pose.pose.position.y
        distance = math.sqrt((nand_x - self.sc_x)**2 + (nand_y - self.sc_y)**2)
        # camera should only republish if nand position is within view of sc and within 20 meters 
        in_fov = self.is_within_fov((self.sc_x, self.sc_y), (nand_x, nand_y), self.sc_heading * 180/math.pi)
        self.get_logger().warn(f"FOV: {in_fov}")
        
        in_cam_view = in_fov and distance < 25
        # lidar should only republish if nand position is within 10 meters of sc 
        in_lidar_view = distance < 10

        # if within camera range: output correctly with 60% accuracy, output incorrectly with 20% accuracy
        # Camera: 60% correct, 20% incorrect, 20% no output
        if in_cam_view:
            cam_state = random.randint(0, 100)
            if cam_state < 60:
                # Correct detection
                self.cam_publisher.publish(msg)
                self.navsat_cam_publisher.publish(odom_to_navsat(msg))
            elif cam_state < 80:
                # Incorrect detection — publish corrupted values
                bad_msg = self.corrupt_odom(msg, pos_noise=3.0)
                self.cam_publisher.publish(bad_msg)
                self.navsat_cam_publisher.publish(odom_to_navsat(bad_msg))
            # else: 20% missed detection — publish nothing

        # Lidar: 80% correct, 10% incorrect, 10% no output
        if in_lidar_view:
            lidar_state = random.randint(0, 100)
            if lidar_state < 80:
                # Correct detection
                self.lidar_publisher.publish(msg)
                self.navsat_lidar_publisher.publish(odom_to_navsat(msg))
            elif lidar_state < 90:
                # Incorrect detection — publish corrupted values
                bad_msg = self.corrupt_odom(msg, pos_noise=1.5)  # lidar is more precise, so less noise
                self.lidar_publisher.publish(bad_msg)
                self.navsat_lidar_publisher.publish(odom_to_navsat(bad_msg))
            # else: 10% missed detection — publish nothing


def main(args=None):
    rclpy.init(args=args)
    visionstack = VisionStack()

    rclpy.spin(visionstack)

    visionstack.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()