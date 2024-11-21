#! /usr/bin/env python3
import sys
import math
import threading
import rclpy
from rclpy.node import Node
# from controller_2d import Controller
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

class VelocityUpdater(Node):
    RATE = 100
    # Bubbles for updating acceleration based on position
    # represented as 4-tuples: (x-pos, y-pos, radius, acceleration)
    # 'list[tuple[float,float,float,float]]'
    # need further update such as more data or import data from certain files
    CHECKPOINTS = [
        (589701, 4477160, 20, 0.5)
    ]

    def __init__(self):
        super().__init__('velocity_updater')
        self.get_logger().info('INITIALIZED.')

        # Declare parameters with default values
        self.declare_parameter('init_vel', 12)
        self.declare_parameter('buggy_name', 'SC')
        # Get the parameter values
        self.init_vel = self.get_parameter("init_vel").value
        self.buggy_name = self.get_parameter("buggy_name").value

        # initialize variables
        self.buggy_vel = self.init_vel
        self.accel = 0.0
        self.position = Point()
        # TODO: uncomment after controller is implemented
        # self.controller = Controller(self.buggy_name)
        # self.lock = threading.Lock()

        # ROS2 subscription
        self.pose_subscriber = self.create_subscription(
            Pose,
            f"{self.buggy_name}/sim_2d/utm",
            self.update_position,
            10  # QoS profile
        )

        # ROS2 timer for stepping
        self.timer = self.create_timer(1.0 / self.RATE, self.step)

    def update_position(self, new_pose: Pose):
        '''Callback function to update internal position variable when new
        buggy position is published
        
        Args:
            new_pose (Pose): Pose object from topic
        '''
        with self.lock:
            self.position = new_pose.position

    def calculate_accel(self):
        '''Check if the position of the buggy is in any of the checkpoints set
        in self.CHECKPOINTS, and update acceleration of buggy accordingly
        '''
        for (x, y, r, a) in self.CHECKPOINTS:
            dist = math.sqrt((x-self.position.x)**2 + (y-self.position.y)**2)
            if dist < r:
                self.accel = a
                break

    def step(self):
        '''Update acceleration and velocity of buggy for one timestep'''
        self.calculate_accel()
        new_velocity = self.buggy_vel + self.accel / self.RATE
        self.buggy_vel = new_velocity
        # TODO: uncomment after controller is implemented
        # self.controller.set_velocity(new_velocity)

def main(args=None):
    rclpy.init(args=args)
    vel_updater = VelocityUpdater()
    rclpy.spin(vel_updater)
    rclpy.shutdown()

if __name__ == "__main__":
    main()