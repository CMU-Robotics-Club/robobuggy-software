#! /usr/bin/env python3
import random
import random
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from utils.odom_to_navsat import odom_to_navsat

class VisionStack(Node):
    def __init__(self):
        super().__init__('visionstack')
        self.get_logger().info('INITIALIZED')

        self.self_state_subscriber = self.create_subscription(
            Pose, "/SC/self/state", self.update_selfstate, 1
        )

        self.other_state_subscriber = self.create_subscription(
            Odometry, "/NAND/self/state", self.republish, 1
        )

        self.cam_publisher = self.create_publisher(NavSatFix, "/vision/other/state", 1)
        self.lidar_publisher = self.create_publisher(NavSatFix, "/lidar/other/state", 1)
        
        self.sc_x = None
        self.sc_y = None
        self.sc_heading = None 


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
        self.sc_x = msg.position.x 
        self.sc_y = msg.position.y 
        self.sc_heading = msg.position.z 

    def republish(self, msg):
        if self.sc_x is None or self.sc_y is None or self.sc_heading is None:
            # self.get_logger().warn("Waiting for self-state position data...")
            return
        nand_x = msg.pose.pose.position.x
        nand_y = msg.pose.pose.position.y
        distance = math.sqrt((nand_x - self.sc_x)**2 + (nand_y - self.sc_y)**2)
        # camera should only republish if nand position is within view of sc and within 20 meters 
        in_fov = self.is_within_fov((self.sc_x, self.sc_y), (nand_x, nand_y), self.sc_heading)
        in_cam_view = in_fov and distance < 20
        # lidar should only republish if nand position is within 5 meters of sc 
        in_lidar_view = distance < 5

        # if within camera range: output correctly with 60% accuracy, output incorrectly with 20% accuracy
        if in_cam_view:
            cam_state = random.randint(0, 100)
            if cam_state < 60:
                # output correctly
                self.cam_publisher.publish(odom_to_navsat(msg))
            elif cam_state < 80:
                # output incorrectly 
                self.cam_publisher.publish(odom_to_navsat(msg))


         # if within lidar range: output correctly with 80% accuracy, output incorrectly with 10% accuracy
        if in_cam_view:
            lidar_state = random.randint(0, 100)
            if lidar_state < 80:
                # output correctly
                self.lidar_publisher.publish(odom_to_navsat(msg))
            elif lidar_state < 90:
                # output incorrectly 
                self.cam_publisher.publish(odom_to_navsat(msg))


def main(args=None):
    rclpy.init(args=args)
    visionstack = VisionStack()

    rclpy.spin(visionstack)

    visionstack.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()