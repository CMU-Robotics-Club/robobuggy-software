#! /usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from controller_2d import Controller
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import threading

class VelocityUpdater(Node):
    RATE = 100
    # Bubbles for updating acceleration based on position
    # represented as 4-tuples: (x-pos, y-pos, radius, acceleration)
    # need further update such as more data or import data from certain files
    CHECKPOINTS = [
        (589701, 4477160, 20, 0.5)
    ]

    def __init__(self, init_vel: float, buggy_name: str):
        super().__init__('vel_updater')

        self.buggy_vel = init_vel
        self.accel = 0.0
        self.position = Point()
        self.controller = Controller(buggy_name)

        self.lock = threading.Lock()

        # ROS2 subscription
        self.pose_subscriber = self.create_subscription(
            Pose,
            f"{buggy_name}/sim_2d/utm",
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
        self.controller.set_velocity(new_velocity)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: vel_updater <initial_velocity> <buggy_name>")
        sys.exit(1)

    init_vel = float(sys.argv[1])
    buggy_name = sys.argv[2]

    vel_updater = VelocityUpdater(init_vel, buggy_name)

    try:
        rclpy.spin(vel_updater)
    except KeyboardInterrupt:
        print("Shutting down velocity updater")
    finally:
        vel_updater.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()