#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, Float64MultiArray
from nav_msgs.msg import Odometry
from buggy.msg import StampedFloat64Msg


from ukf_utils import *
import time

"""
Variable Legend:
x: State Vector, Shape (N,)
Sigma: State Covariance, Shape (N, N)
Q: Process Covariance, Shape (N, N)
^ timestep size dependent

u: Control Vector: (steering), shape (1,)

y: Measurement Vector, Shape (M, )
R: Sensor Covariances Shape: (M, M) 

x_hat: estimation of state
v: velocity
l: length of buggy
theta: heading
delta: steering
delta_0: steering offset

_dot suggests a single order derivative
"""

class Offset_Estim(Node):
    # STATE: northing, easting, heading, velocity, steer_offset
    # Kinematic bicyle over back wheel
    @classmethod
    def dynamics(cls, x, u, params):
        l = params[0]
        _, _, theta, v, delta_0 = x
        delta = u[0]
        x_dot = np.array(
            [v * np.cos(theta), v * np.sin(theta), v * np.tan(delta + delta_0) / l, 0.0, 0.0]
        )
        return x_dot
    

    # Approximately integrate dynamics over a timestep dt to get a discrete update function
    @classmethod
    def rk4_dynamics(cls, x_curr, u_curr, params, dt):
        k1 = cls.dynamics(x_curr, u_curr, params)
        k2 = cls.dynamics(x_curr + k1 * dt / 2, u_curr, params)
        k3 = cls.dynamics(x_curr + k2 * dt / 2, u_curr, params)
        k4 = cls.dynamics(x_curr + k3 * dt, u_curr, params)

        x_next = x_curr + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
        return x_next

    def __init__(self):
        super().__init__("Offset_state_estimator")
        self.get_logger().info('INITIALIZED')

        self.start = False

        self.x_hat : np.ndarray = None
        self.Sigma : np.ndarray = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1.2e-3]) #state covariance
        self.R = np.diag([1e-4, 1e-4]) # TODO: From NAND, devise SC sensor covariance
        self.Q = np.diag([1e-4, 1e-4, 1e-4, 2.4e-1, 1e-6])

        self.create_subscription(Odometry, "/SC/self/state", self.update_measurement, 1) # Using EKF output for simplicity
        self.create_subscription(StampedFloat64Msg, "input/steering", self.updateSteering, 1)
        self.offset_publisher = self.create_publisher(Float64, "self/steer_offset", 1)
        self.state_publisher = self.create_publisher(Float64MultiArray, "self/offset_estimator/state", 1)
        self.state_covar_publisher = self.create_publisher(Float64MultiArray, "self/offset_estimator/covar", 1)

        self.steering = 0

        self.timer = self.create_timer(0.01, self.loop)

        self.last_time = None
    

    def updateSteering(self, msg):
        self.steering = np.deg2rad(msg.data)

    def update_measurement(self, msg):
        if not self.start:
            self.get_logger().info("STARTED")
            self.start = True
            self.x_hat = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, -np.pi/2, 0, 0])
            self.R = np.reshape(np.stack((msg.pose.covariance[:2], msg.pose.covariance[6:8]), axis=0), (2, 2))

        y = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.x_hat, self.Sigma, self.debug = ukf_update(self.x_hat, self.Sigma, y, self.R)

    def loop(self):
        # self.get_logger().info("about to PUBLISHED")
        if not self.start:
            return
        
        time_delta = 0.01 if not self.last_time else time.time() - self.last_time
        self.x_hat, self.Sigma = ukf_predict(self.rk4_dynamics, self.x_hat, self.Sigma, self.Q, [self.steering], time_delta, [1.3])
        self.last_time = time.time()
        

        self.offset_publisher.publish(Float64(data=self.x_hat[4]))

        state_msg = Float64MultiArray()
        state_msg.data = self.x_hat.tolist()
        state_msg.data[2] *= (180 / np.pi) # heading
        state_msg.data[4] *= (180 / np.pi) # steer offset

        covar_msg = Float64MultiArray()
        covar_msg.data = self.Sigma.flatten().tolist()
        self.state_covar_publisher.publish(covar_msg)

        self.state_publisher.publish(state_msg)
        self.offset_publisher.publish(Float64(data=((self.x_hat[4] * 180 / np.pi) + np.rad2deg(self.steering))))


    def accuracy_to_mat(self, accuracy):
        accuracy /= 1000.0
        sigma = (accuracy / (0.848867684498)) * (accuracy / (0.848867684498))
        return np.diag([sigma, sigma])

def main(args=None):
    rclpy.init(args=args)

    # Create the BuggyStateConverter node and spin it
    ukf = Offset_Estim()
    rclpy.spin(ukf)

    # Shutdown when done
    ukf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()