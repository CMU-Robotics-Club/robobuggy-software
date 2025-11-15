#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from buggy.msg import StampedFloat64Msg


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

        self.start = False

        self.x_hat = None
        self.Sigma = np.diag([1e-4, 1e-4, 1e-2, 1e-2]) #state covariance
        self.R = self.accuracy_to_mat(50)
        self.Q = np.diag([1e-4, 1e-4, 1e-2, 2.4e-1])

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
            self.start = True
            self.x_hat = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, -np.pi/2, 0])

        y = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.x_hat, self.Sigma, self.debug = ukf_update(self.x_hat, self.Sigma, y, self.R)

    def loop(self):
        """
        Predict loop callback, runs at 100 Hz.


        - Runs the predict step using the RK4-discretized dynamics.
        - Publishes filtered NAND state and singularity flag at 100 Hz.
        """
        if not self.start:
            return
        self.x_hat, self.Sigma = ukf_predict(self.rk4_dynamics, self.x_hat, self.Sigma, self.Q, [self.steering], 0.01, [1.3])

        nand_ukf_msg = Odometry()
        nand_ukf_msg.pose.pose.position.x = self.x_hat[0]
        nand_ukf_msg.pose.pose.position.y = self.x_hat[1]
        nand_ukf_msg.pose.pose.orientation.z = self.x_hat[2]
        nand_ukf_msg.twist.twist.linear.x = self.x_hat[3]

        # y is 2 elements long
        # S is a 2x2 matrix
        # must be of length 36 to match Odometry specs\
        S = self.debug["S"]
        singular_flag = self.debug["singular_flag"]
        data = np.pad(S.flatten(), (0, 32)).tolist()
        nand_ukf_msg.pose.covariance = data

        singular_flag_msg = Bool()
        singular_flag_msg.data = singular_flag
        self.nand_publisher.publish(nand_ukf_msg)
        self.singular_flag_publisher.publish(singular_flag_msg)

    def accuracy_to_mat(self, accuracy):
        """
        Convert a scalar accuracy to a 2x2 position covariance matrix.

        Args:
            accuracy: Position accuracy.

        Returns:
            2x2 diagonal covariance matrix for x/y position.
        """
        accuracy /= 1000.0
        sigma = (accuracy / (0.848867684498)) * (accuracy / (0.848867684498))
        return np.diag([sigma, sigma])

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