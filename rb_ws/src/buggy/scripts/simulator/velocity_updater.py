#! /usr/bin/env python3
import math
import threading
import json
import rclpy
from rclpy.node import Node
# from controller_2d import Controller
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class VelocityUpdater(Node):
    RATE = 100
    # Bubbles for updating acceleration based on position
    # represented as 4-tuples: (x-pos, y-pos, radius, acceleration)
    # 'list[tuple[float,float,float,float]]'
    # need further update such as more data or import data from certain files

    # TODO: remove after new version with json is tested
    # CHECKPOINTS = [
    #     # (589701, 4477160, 20, 0.5)
    #     (589701, 4477160, 10000000000, 100) # for testing
    # ]

    checkpoints_path = "/rb_ws/src/buggy/scripts/simulator/checkpoints.json"

    with open(checkpoints_path, 'r') as checkpoints_file:
        CHECKPOINTS = (json.load(checkpoints_file))["checkpoints"]

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
        self.update_vel = 0.0
        self.position = Point()

        # TODO: uncomment after controller is implemented
        # self.controller = Controller(self.buggy_name)
        # self.lock = threading.Lock()

        # subscribe pose
        self.pose_subscriber = self.create_subscription(
            Pose,
            f"{self.buggy_name}/sim_2d/utm",
            self.update_position,
            10  # QoS profile
        )

        # TODO: remove after controller is implemented
        # publish velocity
        self.velocity_publisher = self.create_publisher(Float64, "velocity", 1)

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

        # TODO: remove after new version with json is tested
        # for (x, y, r, a) in self.CHECKPOINTS:
        #     dist = math.sqrt((x-self.position.x)**2 + (y-self.position.y)**2)
        #     if dist < r:
        #         self.accel = a
        #         break

        for checkpoint in self.CHECKPOINTS:
            x = checkpoint["x-pos"]
            y = checkpoint["y-pos"]
            r = checkpoint["radius"]
            v = checkpoint["velocity"]
            dist = math.sqrt((x-self.position.x)**2 + (y-self.position.y)**2)
            if dist < r:
                self.update_vel = v
                break

    def step(self):
        '''Update acceleration and velocity of buggy for one timestep'''
        # self.calculate_accel()
        for checkpoint in self.CHECKPOINTS:
            x = checkpoint["x-pos"]
            y = checkpoint["y-pos"]
            r = checkpoint["radius"]
            v = checkpoint["velocity"]
            dist = math.sqrt((x-self.position.x)**2 + (y-self.position.y)**2)
            if dist < r:
                self.update_vel = v
                break

        self.buggy_vel = self.update_vel

        # TODO: uncomment after controller is implemented
        # self.controller.set_velocity(self.update_vel)

        # TODO: remove after controller is implemented
        float_64_velocity = Float64()
        float_64_velocity.data = float(self.buggy_vel)
        self.velocity_publisher.publish(float_64_velocity)

def main(args=None):
    rclpy.init(args=args)
    vel_updater = VelocityUpdater()
    rclpy.spin(vel_updater)
    rclpy.shutdown()

if __name__ == "__main__":
    main()