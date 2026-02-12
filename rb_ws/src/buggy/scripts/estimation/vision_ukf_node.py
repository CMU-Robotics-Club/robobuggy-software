#!/usr/bin/env python3

# VARIABLE NAMES
# x_hat = state estimate
# sigma = covariance of the state estimate
# y = measurement
# R = covariance of the measurement
# Q = process noise covariance (uncertainty in the dynamics model)
# u_curr = control input applied over this timestep
# dt = timestep duration
# params = model parameters passed to the nonlinear dynamics

import numpy as np

from rb_ws.build.buggy.rosidl_generator_py.buggy import msg
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


from ukf import *

class VisionUKF(Node):
    def __init__(self):
        super().__init__("Vision_sensor_fusion")
        self.get_logger().info('Initialized')

        self.start = False

        self.x_hat = None   
        # TODO update these values, use the new functions in ukf.py for readability
        self.Sigma = np.diag([1e-4, 1e-4, 1e-2, 1e-2]) #state covariance
        
        # TODO update these values after testing the camera/lidar
        self.R_lidar = self.get_lidar_acc_matrix()
        self.R_camera = self.accuracy_to_mat(50)

        self.Q = np.diag([1e-4, 1e-4, 1e-2, 2.4e-1])

        # TODO is it the right name
        self.create_subscription(Odometry, "vision/other/state", self.update_camera, 1)
        self.create_subscription(Odometry, "lidar/other/state", self.update_lidar, 1)
        
        self.nand_publisher = self.create_publisher(Odometry, "other/vision_fusion", 10)

        self.steering = 0

        self.timer = self.create_timer(0.01, self.loop)

    def update_lidar(self, msg):
        if not self.start:
            self.start = True
            self.x_hat = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, -np.pi/2, 0])

            y = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            self.x_hat, self.Sigma = ukf_update(self.x_hat, self.Sigma, y, self.R_lidar)

    def update_camera(self, msg):
        if not self.start:
            self.start = True
            self.x_hat = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, -np.pi/2, 0])

        y = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.x_hat, self.Sigma = ukf_update(self.x_hat, self.Sigma, y, self.R_camera)

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

    def get_lidar_acc_matrix(self):
        # [sensor accuracy, estimation accuracy]
        # according to the datasheet, lidar is about +- 3cm 
        x_acc = [0, 0]
        y_acc = [0, 0]
        z_acc = [0, 0]

 
    def accuracy_to_mat(self, accuracy):
        accuracy /= 1000.0
        sigma = (accuracy / (0.848867684498)) * (accuracy / (0.848867684498))
        return np.diag([sigma, sigma])



def main(args=None):
    rclpy.init(args=args)

    # Create the BuggyStateConverter node and spin it
    ukf = VisionUKF()
    rclpy.spin(ukf)

    # Shutdown when done
    ukf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()