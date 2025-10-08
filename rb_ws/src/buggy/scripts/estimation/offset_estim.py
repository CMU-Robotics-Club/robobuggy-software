#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from buggy.msg import StampedFloat64Msg


from ukf_utils import *

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
l: length
theta: heading
delta: steering
delta_0: steering offset

_dot suggests a single order derivative
"""

class Offset_Estim(Node):

    # Kinematic bicyle over back wheel
    @classmethod
    def dynamics(cls, x, u, params):
        l = params[0]
        _, _, theta, v, delta_0 = x
        delta = u[0]
        x_dot = np.array(
            [v * np.cos(theta), v * np.sin(theta), v * np.tan(delta) / l, 0.0, 0.0]
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
        super().__init__("NAND_state_estimator")
        self.get_logger().info('Initialized')

        self.start = False

        self.x_hat = None
        self.Sigma = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1.69]) #state covariance
        self.R = self.accuracy_to_mat(50)
        self.Q = np.diag([1e-4, 1e-4, 1e-2, 2.4e-1, 2e-1])

        self.create_subscription(Odometry, "self/state", self.update_measurement, 1)
        self.create_subscription(StampedFloat64Msg, "input/steering", self.updateSteering, 1)
        self.offset_publisher = self.create_publisher(Odometry, "self/steer_offset", 10)
        self.singular_flag_publisher = self.create_publisher(Bool, "debug/NANDSingularFlag", 10)

        self.steering = 0

        self.timer = self.create_timer(0.01, self.loop)
    

    def updateSteering(self, msg):
        self.steering = np.deg2rad(msg.data)

    def update_measurement(self, msg):
        if not self.start:
            self.start = True
            velocity = np.linalg.norm([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
            self.x_hat = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, velocity , 0])

        y = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.x_hat, self.Sigma, self.debug = ukf_update(self.x_hat, self.Sigma, y, self.R)

    def loop(self):
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