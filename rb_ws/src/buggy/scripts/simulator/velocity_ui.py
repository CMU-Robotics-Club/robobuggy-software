#! /usr/bin/env python3
import sys
import threading
import tkinter as tk
# from controller_2d import Controller
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class VelocityUI(Node):
    def __init__(self):
        super().__init__('velocity_ui')
        self.get_logger().info('INITIALIZED.')

        # Declare parameters with default values
        self.declare_parameter('init_vel', 12)
        self.declare_parameter('buggy_name', 'SC')
        # Get the parameter values
        self.init_vel = self.get_parameter("init_vel").value
        self.buggy_name = self.get_parameter("buggy_name").value

        # initialize variables
        # TODO: uncomment after controller is implemented
        # # So the buggy doesn't start moving without user input
        # self.buggy_vel = 0
        # # self.controller = Controller(buggy_name)
        # self.lock = threading.Lock()

        self.buggy_vel = self.init_vel

        # TODO: remove after controller is implemented
        # publish velocity 
        self.velocity_publisher = self.create_publisher(Float64, "velocity", 1)


        # TODO: tk is not displaying rn
        self.root = tk.Tk()

        self.root.title(self.buggy_name + ' Manual Velocity: scale = 0.1m/s')
        self.root.geometry('600x100')
        self.root.configure(background='#342d66')

        self.scale = tk.Scale(self.root, from_=0, to=300, orient=tk.HORIZONTAL,
                              length=500, width=30)
        self.scale.pack()

        self.root.bind("<Up>", lambda i: self.scale.set(self.scale.get() + 2))
        self.root.bind("<Down>", lambda d: self.scale.set(self.scale.get() - 2))

        # ROS2 timer for stepping
        # 0.01 is equivalent to 100Hz (100 times per second)
        # self.create_timer(0.01, self.step)

    def step(self):
        # Sets the velocity of the buggy to the current scale value
        # called every tick
        self.root.update_idletasks()
        self.root.update()
        # Update velocity of the buggy
        # '/10' set velocity with 0.1 precision
        self.buggy_vel = self.scale.get() / 10

        # TODO: uncomment after controller is implemented
        # self.controller.set_velocity(self.buggy_vel)

        # TODO: remove after controller is implemented
        float_64_velocity = Float64()
        float_64_velocity.data = float(self.buggy_vel)
        self.velocity_publisher .publish(float_64_velocity)

def main(args=None):
    rclpy.init(args=args)
    vel_ui = VelocityUI()
    rclpy.spin(vel_ui)
    rclpy.shutdown()

if __name__ == "__main__":
    main()