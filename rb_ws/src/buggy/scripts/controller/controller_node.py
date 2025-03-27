#!/usr/bin/env python3

import os
import threading
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float64, Bool
from nav_msgs.msg import Odometry
from buggy.msg import TrajectoryMsg

from util.trajectory import Trajectory
from controller.stanley_controller import StanleyController

class Controller(Node):

    def __init__(self):
        """
        Constructor for Controller class.

        Creates a ROS node with a publisher that periodically sends a message
        indicating whether the node is still alive.

        """
        super().__init__('controller')
        self.get_logger().info('INITIALIZED.')


        #Parameters
        self.declare_parameter("dist", 0.0) #Starting Distance along path
        start_dist = self.get_parameter("dist").value

        self.declare_parameter("controllerName", "controller")

        self.declare_parameter("traj_name", "buggycourse_safe.json")
        traj_name = self.get_parameter("traj_name").value
        self.cur_traj = Trajectory(json_filepath=os.environ["TRAJPATH"] + traj_name)

        self.declare_parameter("stateTopic", "self/state")
        self.declare_parameter("steeringTopic", "input/steering")
        self.declare_parameter("trajectoryTopic", "self/cur_traj")

        start_index = self.cur_traj.get_index_from_distance(start_dist)

        self.declare_parameter("controller", "stanley")


        self.declare_parameter("useHeadingRate", True)

        controller_name = self.get_parameter("controller").value
        print(controller_name.lower)
        if (controller_name.lower() == "stanley"):
            self.controller = StanleyController(start_index = start_index, namespace = self.get_namespace(),
                                                node=self, usingHeadingRateError=self.get_parameter("useHeadingRate").value,
                                                controllerName=self.get_parameter("controllerName").value) #IMPORT STANLEY
        else:
            self.get_logger().error("Invalid Controller Name: " + controller_name.lower())
            raise Exception("Invalid Controller Argument")

        # Publishers
        self.init_check_publisher = self.create_publisher(Bool,
            "debug/init_safety_check", 1
        )
        self.steer_publisher = self.create_publisher(
            Float64, self.get_parameter("steeringTopic").value, 1
        )
        self.heading_publisher = self.create_publisher(
            Float32, "debug/heading", 1
        )

        # Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, self.get_parameter("stateTopic").value, self.odom_listener, 1)
        self.traj_subscriber = self.create_subscription(TrajectoryMsg, self.get_parameter("trajectoryTopic").value, self.traj_listener, 1)

        self.lock = threading.Lock()

        self.odom = None
        self.passed_init = False

        timer_period = 0.01  # seconds (100 Hz)
        self.timer = self.create_timer(timer_period, self.loop)

    def odom_listener(self, msg : Odometry):
        '''
        This is the subscriber that updates the buggies state for navigation
        msg, should be a CLEAN state as defined in the wiki
        '''
        with self.lock:
            self.odom = msg

    def traj_listener(self, msg):
        '''
        This is the subscriber that updates the buggies trajectory for navigation
        '''
        with self.lock:
            self.cur_traj, self.controller.current_traj_index = Trajectory.unpack(msg)

    def init_check(self):
        """
        Checks if it's safe to switch the buggy into autonomous driving mode.
        Specifically, it checks:
            if we can recieve odometry messages from the buggy
            if the covariance is acceptable (less than 1 meter)
            if the buggy thinks it is facing in the correct direction wrt the local trajectory (not 180 degrees flipped)

        Returns:
           A boolean describing the status of the buggy (safe for auton or unsafe for auton)
        """
        if (self.odom == None):
            self.get_logger().warn("WARNING: no available position estimate")
            return False

        elif (self.odom.pose.covariance[0] ** 2 + self.odom.pose.covariance[7] ** 2 > 1):
            self.get_logger().warn("checking position estimate certainty | current covariance: " + str(self.odom.pose.covariance[0] ** 2 + self.odom.pose.covariance[7] ** 2 ))
            return False

        #Originally under a lock, doesn't seem necessary?
        current_heading = np.arctan2(np.sin(self.odom.pose.pose.orientation.z), np.cos(self.odom.pose.pose.orientation.z))
        closest_heading = self.cur_traj.get_heading_by_index(self.cur_traj.get_closest_index_on_path(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y))

        self.get_logger().info("current heading: " + str(np.rad2deg(current_heading)))
        msg = Float32()
        msg.data = np.rad2deg(current_heading)
        self.heading_publisher.publish(msg)

        #Converting headings from [-pi, pi] to [0, 2pi]
        if (current_heading < 0):
            current_heading = 2 * np.pi + current_heading
        if (closest_heading < 0):
            closest_heading = 2 * np.pi + closest_heading

        if (abs(current_heading - closest_heading) >= np.pi/2):
            self.get_logger().warn("WARNING: INCORRECT HEADING! restart stack. Current heading [-180, 180]: " + str(np.rad2deg(current_heading)))
            return False

        return True

    def loop(self):
        if not self.passed_init:
            self.passed_init = self.init_check()
            msg = Bool()
            msg.data = self.passed_init
            self.init_check_publisher.publish(msg)
            if self.passed_init:
                self.get_logger().info("Passed Initialization Check")
            else:
                return

        self.heading_publisher.publish(Float32(data=np.rad2deg(self.odom.pose.pose.orientation.z)))
        steering_angle = self.controller.compute_control(self.odom, self.cur_traj)
        steering_angle_deg = np.rad2deg(steering_angle)
        self.steer_publisher.publish(Float64(data=float(steering_angle_deg.item())))



def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()