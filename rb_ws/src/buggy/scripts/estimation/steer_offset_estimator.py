#!/usr/bin/env python3
import time
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from buggy.msg import StampedFloat64Msg, SCDebugInfoMsg, NANDDebugInfoMsg


from estimation import ukf_utils
from util.constants import Constants

class SteerOffsetEstimator(Node):
    """
    UKF-based estimator for steering offset using a kinematic bicycle model.

    Model:
    - Kinematic bicycle over the back wheel.
    - Continuous-time dynamics discretized using RK4.
    - Used to estimate a (possibly varying) steering offset term.

    State vector (x):
    - x[0]: northing (m)
    - x[1]: easting (m)
    - x[2]: heading theta (rad)
    - x[3]: velocity v (m/s)
    - x[4]: steering offset delta_0 (rad)

    Covariances:
    - Sigma: state covariance, shape (N, N)
    - Q: process covariance, shape (N, N)
    - R: sensor covariance, shape (M, M)

    Inputs and measurements:
    - u: control vector (steering), shape (1,)
    - y: measurement vector, shape (M,)

    Notation:
    - x_hat: state estimate
    - v: velocity
    - l: wheelbase length of buggy
    - theta: heading
    - delta: commanded steering
    - delta_0: steering offset
    - _dot indicates a first-order time derivative.
    """

    @classmethod
    def dynamics(cls, x, u, params):
        """
        Continuous-time bicycle dynamics for the state derivative.

        Args:
            x: State vector [x, y, heading, velocity, steer_offset].
            u: Control input, steering angle (rad).
            params: Model parameters; params[0] is the wheelbase.

        Returns:
            State time derivative dx/dt as a NumPy array.
        """
        l = params[0]
        _, _, theta, v, delta_0 = x
        delta = u[0]
        x_dot = np.array(
            [v * np.cos(theta), v * np.sin(theta), v * np.tan(delta + delta_0) / l, 0.0, 0.0]
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
        Initialize the steering offset estimator node.

        - Sets up UKF state, covariance, and noise matrices.
        - Configures wheelbase based on ROS namespace (SC/NAND).
        - Subscribes to firmware debug, self/state, and input/steering topics.
        - Publishes steer offset estimate, UKF state, and covariance.
        """
        super().__init__("offset_estimator")
        self.get_logger().info('INITIALIZED')

        self.enabled = True  # estimator enabled
        self.auton_enabled_prev = None  # previous auton flag for edge detection

        # moved to reset_filter()
        # self.x_hat: np.ndarray = None
        # self.Sigma: np.ndarray = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1.2e-3]) # state covariance
        # self.Q = np.diag([1e-4, 1e-4, 1e-4, 2.4e-1, 1e-6])  # init process covariance values (2.4e-1 for velocity based on 3 x std dev of 0.16)
        # self.R = np.diag([1e-2, 1e-2])  # init sensor covariance values
        # self.last_time = None

        self.reset_filter() # initialize filter state

        if (self.get_namespace() == "/SC"):
            self.wheelbase = Constants.WHEELBASE_SC
            self.create_subscription(SCDebugInfoMsg, "debug/firmware", self.firmware_debug_callback, 1)
        elif (self.get_namespace() == "/NAND"):
            self.wheelbase = Constants.WHEELBASE_NAND
            self.create_subscription(NANDDebugInfoMsg, "debug/firmware", self.firmware_debug_callback, 1)
        self.create_subscription(Odometry, "self/state", self.update_measurement, 1) # Using EKF output for simplicity
        self.create_subscription(StampedFloat64Msg, "input/steering", self.update_steering, 1)
        self.offset_publisher = self.create_publisher(Float64, "self/steer_offset", 1)
        self.state_publisher = self.create_publisher(Float64MultiArray, "self/offset_estimator/state", 1)
        self.state_covar_publisher = self.create_publisher(Float64MultiArray, "self/offset_estimator/covariance", 1)

        self.steering = 0

        self.timer = self.create_timer(0.01, self.loop)

    def reset_filter(self):
        """Reset the UKF so next measurement initializes state."""
        self.start = False
        self.x_hat: np.ndarray = np.zeros((5,))  # state vector
        self.Sigma: np.ndarray = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1.2e-3]) # state covariance
        self.Q = np.diag([1e-4, 1e-4, 1e-4, 2.4e-1, 1e-6])  # init process covariance values (2.4e-1 for velocity based on 3 x std dev of 0.16)
        self.R = np.diag([1e-2, 1e-2])  # init sensor covariance values
        self.last_time = None

    def firmware_debug_callback(self, msg):
        """
        Handle debug/firmware messages to enable/disable and re-init the estimator.
        Uses auton steer flag edges to reset or pause the UKF.
        """
        auton = bool(msg.auton_steer)

        if self.auton_enabled_prev is None:
            # first message: align state
            self.enabled = auton
            if not auton:
                self.get_logger().info("Auton initially disabled: estimator paused")
                self.reset_filter()

        # Rising edge False -> True
        elif not self.auton_enabled_prev and auton:
            self.get_logger().info("Auton enabled: re-initializing offset UKF")
            self.enabled = True
            self.reset_filter()

        # Falling edge True -> False
        elif self.auton_enabled_prev and not auton:
            self.get_logger().info("Auton disabled: pausing offset estimator")
            self.enabled = False
            self.reset_filter()

        self.auton_enabled_prev = auton

    @classmethod
    def wrap_angle(cls, angle, limit=np.pi):
        """Wrap an angle to the interval (-limit, limit]."""
        return (angle + limit) % (2 * limit) - limit

    def update_steering(self, msg):
        self.steering = np.deg2rad(msg.data)

    def update_measurement(self, msg):
        """Perform UKF measurement update using pose from self/state."""
        if not self.enabled:
            return

        # initialize state on first measurement
        if not self.start:
            self.get_logger().info("STARTED")
            self.start = True
            self.x_hat = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.orientation.z,
                msg.twist.twist.linear.x,
                0])
            # extract 2x2 position covariance from the 6x6 pose covariance
            self.R = np.reshape(np.stack((msg.pose.covariance[:2], msg.pose.covariance[6:8]), axis=0), (2, 2))

        # measurement vector
        y = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        # perform measurement update
        self.x_hat, self.Sigma, self.debug = ukf_utils.ukf_update(self.x_hat, self.Sigma, y, self.R)

        self.x_hat[2] = self.wrap_angle(self.x_hat[2], np.pi)     # wrap heading
        self.x_hat[4] = self.wrap_angle(self.x_hat[4], np.pi/2)   # wrap steer offset

    def loop(self):
        """
        Predict loop callback, runs at 100 Hz.

        - Runs the predict step using the RK4-discretized dynamics.
        - Wraps heading and steering offset to keep them in valid ranges.
        - Publishes steer offset, full state, and covariance at 100 Hz.
        """
        if (not self.enabled) or (not self.start):
            return

        time_delta = 0.01 if not self.last_time else time.time() - self.last_time
        self.x_hat, self.Sigma = ukf_utils.ukf_predict(self.rk4_dynamics, self.x_hat, self.Sigma, self.Q, [self.steering], time_delta, [self.wheelbase])
        self.x_hat[2] = self.wrap_angle(self.x_hat[2], np.pi)     # wrap heading
        self.x_hat[4] = self.wrap_angle(self.x_hat[4], np.pi/2)   # wrap steer offset
        self.last_time = time.time()

        # wrap the steering offset to (-pi/2, pi/2]
        steer_offset = np.rad2deg(self.wrap_angle(self.x_hat[4], np.pi/2))
        self.offset_publisher.publish(Float64(data=steer_offset))

        state_msg = Float64MultiArray()
        state_msg.data = self.x_hat.tolist()
        state_msg.data[2] = np.rad2deg(state_msg.data[2]) # convert heading to deg
        state_msg.data[4] = np.rad2deg(state_msg.data[4]) # steer offset to deg

        self.state_publisher.publish(state_msg)

        covar_msg = Float64MultiArray()
        covar_msg.data = self.Sigma.flatten().tolist()
        self.state_covar_publisher.publish(covar_msg)


def main(args=None):
    rclpy.init(args=args)

    # Create the SteerOffsetEstimator node and spin it
    ukf = SteerOffsetEstimator()
    rclpy.spin(ukf)

    # Shutdown when done
    ukf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()