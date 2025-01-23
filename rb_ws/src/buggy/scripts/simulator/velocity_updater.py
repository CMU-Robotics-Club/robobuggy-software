#! /usr/bin/env python3
import math
import threading
import json
import rclpy
from rclpy.node import Node
from engine import Simulator
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class VelocityUpdater(Node):
    RATE = 100

    # Bubbles for updating acceleration based on position
    # represented as {x-pos, y-pos, radius, velocity}
    # imported from checkpoints.json
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
        self.debug_dist = 0

        self.position = Point()
        self.lock = threading.Lock()

        # subscribe sim_2d/utm for position values
        self.pose_subscriber = self.create_subscription(
            Pose,
            "sim_2d/utm",
            self.update_position,
            1
        )

        # publish velocity to "sim/velocity"
        self.velocity_publisher = self.create_publisher(Float64, "sim/velocity", 1)

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

    def check_velocity(self):
        '''Check if the position of the buggy is in any of the checkpoints set
        in self.CHECKPOINTS, and update velocity of buggy accordingly
        '''
        for checkpoint in self.CHECKPOINTS:
            x = checkpoint["x-pos"]
            y = checkpoint["y-pos"]
            r = checkpoint["radius"]
            v = checkpoint["velocity"]
            dist = math.sqrt((x-self.position.x)**2 + (y-self.position.y)**2)
            self.debug_dist = dist
            if dist < r:
                self.buggy_vel = v
                break

    def step(self):
        '''Update velocity of buggy for one timestep
        and publish it so that the simulator engine can subscribe and use it'''

        # update velocity
        self.check_velocity()

        # publish velocity
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