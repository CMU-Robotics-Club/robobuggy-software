#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from buggy.msg import StampedFloat64Msg

from util.constants import Constants

from ukf_utils import *

class NANDStateEstimator(Node):
    """
    UKF-based state estimator for NAND using a kinematic bicycle model.

    Model:
    - Kinematic bicycle over the back wheel.
    - Continuous-time dynamics discretized using RK4.
    - Estimates planar pose and forward velocity.

    State vector (x):
    - x[0]: northing (m)
    - x[1]: easting (m)
    - x[2]: heading theta (rad)
    - x[3]: velocity v (m/s)

    Covariances:
    - Sigma: state covariance, shape (N, N)
    - Q: process covariance, shape (N, N), timestep-size dependent
    - R: sensor covariance, shape (M, M)

    Inputs and measurements:
    - u: control vector (steering), shape (1,)
    - y: measurement vector, shape (M,)

    Notation:
    - x_hat: state estimate
    - v: velocity
    - l: wheelbase length
    - theta: heading
    - delta: commanded steering
    - _dot indicates a first-order time derivative.
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
        """Approximately integrate dynamics over a timestep dt using RK4 to get a discrete update function."""
        k1 = cls.dynamics(x_curr, u_curr, params)
        k2 = cls.dynamics(x_curr + k1 * dt / 2, u_curr, params)
        k3 = cls.dynamics(x_curr + k2 * dt / 2, u_curr, params)
        k4 = cls.dynamics(x_curr + k3 * dt, u_curr, params)

        x_next = x_curr + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
        return x_next

    def __init__(self):
        """
        Initialize the NAND state estimator node.

        - Sets up UKF state, covariance, and noise matrices.
        - Subscribes to noUKF state (other/stateNoUKF) and steering topics.
        - Publishes filtered state and a singularity flag.
        """
        super().__init__("NAND_state_estimator")
        self.get_logger().info('Initialized')

        self.init_ukf()

        self.create_subscription(Odometry, "other/stateNoUKF", self.update_measurement, 1)
        self.create_subscription(StampedFloat64Msg, "other/steering", self.update_steering, 1)
        self.nand_publisher = self.create_publisher(Odometry, "other/state", 1)
        self.singular_flag_publisher = self.create_publisher(Bool, "debug/NANDSingularFlag", 1)

        self.steering = 0

        self.timer = self.create_timer(0.01, self.loop)

    def update_steering(self, msg):
        self.steering = np.deg2rad(msg.data)

    def update_measurement(self, msg):
        """Perform UKF measurement update using pose from other/stateNoUKF."""
        if not self.start:
            self.get_logger().info("STARTED")
            self.start = True
            self.x_hat = np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y, -np.pi / 2, 0.0]
            )

        y = [msg.pose.pose.position.x, msg.pose.pose.position.y]

        self.x_hat, self.Sigma, self.singular_flag = ukf_update(
            self.x_hat, self.Sigma, self.Sigma_init, y, self.R
        )

        # publish singular flag immediately after measurement update, because prediction also writes to the debug singular flag
        singular_flag_msg = Bool(data=self.singular_flag)
        self.singular_flag_publisher.publish(singular_flag_msg)

    def init_ukf(self):
        """Reset the UKF and wait for the next measurement to initialize state."""
        self.start = False
        self.x_hat = None
        self.Sigma_init = np.diag([1e-2, 1e-2, 1e-2, 1e-1])  # initial state covariance
        self.Sigma = self.Sigma_init.copy()  # state covariance
        self.R = self.accuracy_to_mat(50)
        self.Q = np.diag([1e-4, 1e-4, 1e-2, 2.4e-1])
        self.singular_flag = False
        self.ukf_converged = False


    def loop(self):
        """
        Predict loop callback, runs at 100 Hz.

        - Runs the predict step using the RK4-discretized dynamics.
        - Publishes filtered NAND state and singularity flag at 100 Hz.
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
            [Constants.WHEELBASE_NAND],
        )

        nand_ukf_msg = Odometry()
        nand_ukf_msg.pose.pose.position.x = self.x_hat[0]
        nand_ukf_msg.pose.pose.position.y = self.x_hat[1]
        nand_ukf_msg.pose.pose.orientation.z = self.x_hat[2]
        nand_ukf_msg.twist.twist.linear.x = self.x_hat[3]

        Sigma = self.Sigma
        if Sigma is not None:
            pose_cov = np.zeros((6, 6))
            twist_cov = np.zeros((6, 6))

            # Pose covariance: 6x6 matrix for [x, y, z, roll, pitch, yaw]
            pose_cov[0:2, 0:2] = Sigma[0:2, 0:2]  # x, y variances & cross-covariances
            pose_cov[5, 5] = Sigma[2, 2]          # heading (yaw) variance
            pose_cov[0:2, 5] = Sigma[0:2, 2]      # cross-covariance x,y and yaw
            pose_cov[5, 0:2] = Sigma[2, 0:2]      # cross-covariance yaw and x,y

            # Twist covariance: 6x6 matrix for [v_x, v_y, v_z, w_x, w_y, w_z]
            twist_cov[0, 0] = Sigma[3, 3]         # linear velocity x variance

            pose_var = np.array([
                    pose_cov[0, 0],  # x
                    pose_cov[1, 1],  # y
                    pose_cov[5, 5],  # yaw
                ])

            twist_var = twist_cov[0, 0]

            if (np.any(pose_var > Constants.NAND_UKF_POSE_DIVERGENCE_THRESHOLD)
                    or twist_var > Constants.NAND_UKF_TWIST_DIVERGENCE_THRESHOLD):
                if self.ukf_converged:
                    self.ukf_converged = False
                    self.get_logger().warn(
                        f"WARNING: NAND State UKF diverged! "
                        f"Current Pose Covariance: {pose_cov}, "
                        f"Current Twist Covariance: {twist_cov}"
                    )
                    self.get_logger().info("Reinitializing UKF")
                    self.init_ukf()
                    return

            elif (np.any(pose_var < Constants.NAND_UKF_POSE_CONVERGENCE_THRESHOLD)
                    or twist_var < Constants.NAND_UKF_TWIST_CONVERGENCE_THRESHOLD) and not self.ukf_converged:
                self.get_logger().info("NAND State UKF converged!")
                self.ukf_converged = True

            nand_ukf_msg.pose.covariance = pose_cov.flatten().tolist()
            nand_ukf_msg.twist.covariance = twist_cov.flatten().tolist()

        singular_flag_msg = Bool(data=self.singular_flag)
        self.singular_flag_publisher.publish(singular_flag_msg)

        self.nand_publisher.publish(nand_ukf_msg)

    def accuracy_to_mat(self, accuracy):
        """
        Convert a scalar *circular* position accuracy to a 2x2 position covariance matrix.

        Args:
            accuracy: Position accuracy (radius) in millimeters.

        We assume the position error in x and y follows a 2D Gaussian
        (normal) distribution that is:
        * centered at zero (no bias)
        * has the same spread in x and y
        * has no correlation between x and y

        The constant CEP50_to_STD=0.8493218003 is a precomputed factor that relates
        this circular accuracy radius to the underlying standard deviation 
        of that Gaussian.

        Once we have that standard deviation σ (in meters), the covariance
        matrix for (x, y) is simply:
        [ σ²  0  ]
        [ 0   σ² ]

        Returns:
            2x2 diagonal covariance matrix for x/y position (in m²).
        """
        # convert radius from millimeters to meters
        accuracy /= 1000.0

        # accuracy is the circular error radius (meters)
        # k is the factor that maps between this radius and the Gaussian σ
        k = Constants.CEP50_to_STD
        sigma = accuracy * k

        # variance in each axis (assuming isotropic uncertainty)
        sigma_sq = sigma * sigma

        return np.diag([sigma_sq, sigma_sq])

def main(args=None):
    rclpy.init(args=args)

    # Create the NANDStateEstimator node and spin it
    ukf = NANDStateEstimator()
    rclpy.spin(ukf)

    # Shutdown when done
    ukf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
