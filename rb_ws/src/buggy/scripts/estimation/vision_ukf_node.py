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

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

from sensor_msgs.msg import NavSatFix

from util.odomToNavsatFix import odom_to_navsat


from ukf_utils import *

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
        self.R_camera = self.get_camera_acc_matrix()

        self.Q = np.diag([1e-4, 1e-4, 1e-2, 2.4e-1])

        self.create_subscription(Odometry, "vision/other/state", self.update_camera, 1)
        self.create_subscription(Odometry, "lidar/other/state", self.update_lidar, 1)
        self.create_subscription(Odometry, "self/state", self.update_self, 1)

        self.nand_publisher = self.create_publisher(NavSatFix, "other/vision_fusion", 10)

        self.steering = 0
        self.self_pos = None

        self.timer = self.create_timer(0.01, self.loop)

    def update_self(self, msg):
        self.self_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])


    def update_lidar(self, msg):
        if not self.start:
            if self.self_pos is None:
                return
            self.start = True
            self.x_hat = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, -np.pi/2, 0])
            dist = np.linalg.norm(self.x_hat[:2] - self.self_pos)
            self.R_lidar = self.get_lidar_acc_matrix(dist=dist)

        y = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.x_hat, self.Sigma = ukf_update(self.x_hat, self.Sigma, y, self.R_lidar)

    def update_camera(self, msg):

        if not self.start:
            if self.self_pos is None:
                return
            self.start = True
            self.x_hat = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, -np.pi/2, 0])
            dist = np.linalg.norm(self.x_hat[:2] - self.self_pos)
            self.R_camera = self.get_camera_acc_matrix(dist=dist)

        y = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.x_hat, self.Sigma = ukf_update(self.x_hat, self.Sigma, y, self.R_camera)
    
    def rk4_dynamics(cls, x_curr, u_curr, params, dt):
        """Approximately integrate dynamics over a timestep dt using RK4 to get a discrete update function."""
        k1 = cls.dynamics(x_curr, u_curr, params)
        k2 = cls.dynamics(x_curr + k1 * dt / 2, u_curr, params)
        k3 = cls.dynamics(x_curr + k2 * dt / 2, u_curr, params)
        k4 = cls.dynamics(x_curr + k3 * dt, u_curr, params)

        x_next = x_curr + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
        return x_next

    def loop(self):
        if not self.start:
            return
        self.x_hat, self.Sigma = ukf_predict(rk4_dynamics, self.x_hat, self.Sigma, self.Q, [self.steering], 0.01, [1.3])

        newMsg = Odometry()
        newMsg.pose.pose.position.x = self.x_hat[0]
        newMsg.pose.pose.position.y = self.x_hat[1]
        newMsg.pose.pose.orientation.z = self.x_hat[2]
        newMsg.twist.twist.linear.x = self.x_hat[3]
        self.nand_publisher.publish(odom_to_navsat(newMsg))

    def get_lidar_acc_matrix(self, dist=1):
        _ = dist

        # --- SENSOR UNCERTAINTY ---
        sigma_range = 0.03  # meters (datasheet ±3cm)
        R_sensor = np.diag([sigma_range**2,
                            sigma_range**2,])

        # --- ESTIMATOR UNCERTAINTY ---
        sigma_estimator = 0.10  # meters (temporary assumption)
        R_estimator = np.diag([sigma_estimator**2,
                            sigma_estimator**2])

        # --- TOTAL MEASUREMENT COVARIANCE ---
        R_total = R_sensor + R_estimator

        return R_total


    def get_camera_acc_matrix(self, dist=1):
        # --- SENSOR UNCERTAINTY ---
        sigma_range = 0.05 * dist  # meters (datasheet 5% of distance)
        R_sensor = np.diag([sigma_range**2,
                            sigma_range**2])

        # --- ESTIMATOR UNCERTAINTY ---
        sigma_estimator = 0.10  # meters (temporary assumption)
        R_estimator = np.diag([sigma_estimator**2,
                            sigma_estimator**2])

        # --- TOTAL MEASUREMENT COVARIANCE ---
        R_total = R_sensor + R_estimator

        return R_total




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