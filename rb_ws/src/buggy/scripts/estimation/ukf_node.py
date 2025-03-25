#!/usr/bin/env python3
from threading import Lock
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


from ukf import *

class UKF(Node):
    def __init__(self):
        super().__init__("NAND_state_estimator")
        self.get_logger().info('Initialized')

        self.start = False

        self.x_hat = None
        self.Sigma = np.diag([1e-4, 1e-4, 1e-2, 1e-2]) #state covariance
        self.R = self.accuracy_to_mat(50)
        self.Q = np.diag([1e-4, 1e-4, 1e-2, 2.4e-1])

        self.create_subscription(Odometry, "other/state", self.update, 1)
        self.create_subscription(Float64, "other/steering", self.updateSteering, 1)
        self.nand_publisher = self.create_publisher(Odometry, "other/stateEstimate", 10)

        self.steering = 0

        self.timer = self.create_timer(0.01, self.loop)

    def updateSteering(self, msg):
        self.steering = np.deg2rad(msg.data)

    def update(self, msg):
        if not self.start:
            self.start = True
            self.x_hat = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, -np.pi/2, 0])

        y = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.x_hat, self.Sigma = ukf_update(self.x_hat, self.Sigma, y, self.R)
    
    def loop(self):
        if not self.start:
            return
        self.x_hat, self.Sigma = ukf_predict(self.x_hat, self.Sigma, self.Q, [self.steering], 0.01, [1.3])

        newMsg = Odometry()
        newMsg.pose.pose.position.x = self.x_hat[0]
        newMsg.pose.pose.position.y = self.x_hat[1]
        newMsg.pose.pose.orientation.z = self.x_hat[2]
        newMsg.twist.twist.linear.x = self.x_hat[3]
        self.nand_publisher.publish(newMsg)

        

    def accuracy_to_mat(self, accuracy):
        accuracy /= 1000.0
        sigma = (accuracy / (0.848867684498)) * (accuracy / (0.848867684498))
        return np.diag([sigma, sigma])

def main(args=None):
    rclpy.init(args=args)

    # Create the BuggyStateConverter node and spin it
    ukf = UKF()
    rclpy.spin(ukf)

    # Shutdown when done
    ukf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()