#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from util.odomToNavsatFix import odom_to_navsat

from ukf_utils import *


class VisionUKF(Node):
    """
    UKF-based fusion of camera and lidar measurements for the other buggy.

    Model:
    - Kinematic bicycle over the back wheel.
    - Continuous-time dynamics discretized using RK4.
    - Estimates planar pose and forward velocity.

    State vector (x):
    - x[0]: northing / x position (m)
    - x[1]: easting / y position (m)
    - x[2]: heading theta (rad)
    - x[3]: velocity v (m/s)

    Covariances:
    - Sigma: state covariance, shape (N, N)
    - Sigma_init: initial/reference covariance, shape (N, N)
    - Q: process covariance, shape (N, N)
    - R_camera: camera measurement covariance, shape (2, 2)
    - R_lidar: lidar measurement covariance, shape (2, 2)

    Inputs and measurements:
    - u: control vector (steering), shape (1,)
    - y: measurement vector [x, y], shape (2,)
    """

    @classmethod
    def dynamics(cls, x, u, params):
        """
        Continuous-time bicycle dynamics for the state derivative.

        Args:
            x: State vector [x, y, heading, velocity].
            u: Control input, steering angle (rad).
            params: Model parameters; params[0] is the wheelbase.

        Returns:
            State time derivative dx/dt as a NumPy array.
        """
        l = params[0]
        _, _, theta, v = x
        delta = u[0]
        x_dot = np.array(
            [v * np.cos(theta), v * np.sin(theta), v * np.tan(delta) / l, 0.0]
        )
        return x_dot

    @classmethod
    def rk4_dynamics(cls, x_curr, u_curr, params, dt):
        """Approximately integrate dynamics over a timestep dt using RK4."""
        k1 = cls.dynamics(x_curr, u_curr, params)
        k2 = cls.dynamics(x_curr + k1 * dt / 2, u_curr, params)
        k3 = cls.dynamics(x_curr + k2 * dt / 2, u_curr, params)
        k4 = cls.dynamics(x_curr + k3 * dt, u_curr, params)

        x_next = x_curr + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
        return x_next

    def __init__(self):
        """
        Initialize the vision UKF node.

        - Sets up UKF state, covariance, and noise matrices.
        - Subscribes to camera, lidar, and self-state topics.
        - Publishes fused NavSatFix output and a singularity debug flag.
        """
        super().__init__("Vision_sensor_fusion")
        self.get_logger().info("Initialized")

        self.start = False

        self.x_hat = None
        self.Sigma_init = np.diag([1e-4, 1e-4, 1e-2, 1e-2])  # initial state covariance
        self.Sigma = self.Sigma_init  # state covariance

        self.R_lidar = self.get_lidar_acc_matrix()
        self.R_camera = self.get_camera_acc_matrix()
        self.Q = np.diag([1e-4, 1e-4, 1e-2, 2.4e-1])

        self.singular_flag = False

        self.create_subscription(Odometry, "vision/other/state", self.update_camera, 1)
        self.create_subscription(Odometry, "lidar/other/state", self.update_lidar, 1)
        self.create_subscription(Odometry, "self/state", self.update_self, 1)

        self.vision_publisher = self.create_publisher(NavSatFix, "other/vision_fusion", 10)
        self.singular_flag_publisher = self.create_publisher(Bool, "debug/VisionSingularFlag", 1)

        self.steering = 0.0
        self.self_pos = None

        self.timer = self.create_timer(0.01, self.loop)

    def update_self(self, msg):
        """Store the current self position for range-based sensor covariance updates."""
        self.self_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def update_lidar(self, msg):
        """Perform UKF measurement update using lidar position."""
        if not self.start:
            if self.self_pos is None:
                return
            self.start = True
            self.x_hat = np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y, -np.pi / 2, 0.0]
            )

        dist = 1.0
        if self.self_pos is not None:
            dist = np.linalg.norm(self.x_hat[:2] - self.self_pos)
        self.R_lidar = self.get_lidar_acc_matrix(dist=dist)

        y = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.x_hat, self.Sigma, self.singular_flag = ukf_update(
            self.x_hat, self.Sigma, self.Sigma_init, y, self.R_lidar
        )

        singular_flag_msg = Bool(data=self.singular_flag)
        self.singular_flag_publisher.publish(singular_flag_msg)

    def update_camera(self, msg):
        """Perform UKF measurement update using camera position."""
        if not self.start:
            if self.self_pos is None:
                return
            self.start = True
            self.x_hat = np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y, -np.pi / 2, 0.0]
            )

        dist = 1.0
        if self.self_pos is not None:
            dist = np.linalg.norm(self.x_hat[:2] - self.self_pos)
        self.R_camera = self.get_camera_acc_matrix(dist=dist)

        y = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.x_hat, self.Sigma, self.singular_flag = ukf_update(
            self.x_hat, self.Sigma, self.Sigma_init, y, self.R_camera
        )

        singular_flag_msg = Bool(data=self.singular_flag)
        self.singular_flag_publisher.publish(singular_flag_msg)

    def loop(self):
        """
        Predict loop callback, runs at 100 Hz.

        - Runs the predict step using the RK4-discretized dynamics.
        - Publishes fused state and singularity flag at 100 Hz.
        """
        if not self.start:
            return

        self.x_hat, self.Sigma, self.singular_flag = ukf_predict(
            self.rk4_dynamics,
            self.x_hat,
            self.Sigma,
            self.Sigma_init,
            self.Q,
            [self.steering],
            0.01,
            [1.3],
        )

        new_msg = Odometry()
        new_msg.pose.pose.position.x = self.x_hat[0]
        new_msg.pose.pose.position.y = self.x_hat[1]
        new_msg.pose.pose.orientation.z = self.x_hat[2]
        new_msg.twist.twist.linear.x = self.x_hat[3]

        Sigma = self.Sigma
        if Sigma is not None:
            pose_cov = np.zeros((6, 6))
            pose_cov[0:2, 0:2] = Sigma[0:2, 0:2]
            pose_cov[5, 5] = Sigma[2, 2]
            pose_cov[0:2, 5] = Sigma[0:2, 2]
            pose_cov[5, 0:2] = Sigma[2, 0:2]
            new_msg.pose.covariance = pose_cov.flatten().tolist()

            twist_cov = np.zeros((6, 6))
            twist_cov[0, 0] = Sigma[3, 3]
            new_msg.twist.covariance = twist_cov.flatten().tolist()

        singular_flag_msg = Bool(data=self.singular_flag)
        self.singular_flag_publisher.publish(singular_flag_msg)

        self.vision_publisher.publish(odom_to_navsat(new_msg))

    def get_lidar_acc_matrix(self, dist=1):
        _ = dist

        # Sensor uncertainty
        sigma_range = 0.03  # meters
        R_sensor = np.diag([sigma_range**2, sigma_range**2])

        # Estimator uncertainty
        sigma_estimator = 0.10  # meters
        R_estimator = np.diag([sigma_estimator**2, sigma_estimator**2])

        return R_sensor + R_estimator

    def get_camera_acc_matrix(self, dist=1):
        # Sensor uncertainty
        sigma_range = 0.05 * dist  # meters
        R_sensor = np.diag([sigma_range**2, sigma_range**2])

        # Estimator uncertainty
        sigma_estimator = 0.10  # meters
        R_estimator = np.diag([sigma_estimator**2, sigma_estimator**2])

        return R_sensor + R_estimator

    def accuracy_to_mat(self, accuracy):
        accuracy /= 1000.0
        sigma = (accuracy / 0.848867684498) * (accuracy / 0.848867684498)
        return np.diag([sigma, sigma])


def main(args=None):
    rclpy.init(args=args)

    ukf = VisionUKF()
    rclpy.spin(ukf)

    ukf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()