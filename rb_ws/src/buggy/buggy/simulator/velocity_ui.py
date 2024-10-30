#! /usr/bin/env python3
import sys
import threading
import tkinter as tk
from controller_2d import Controller
import rclpy
from rclpy.node import Node

class VelocityUI(Node):
    def __init__(self, init_vel: float, buggy_name: str):
        super().__init__('velocity_ui')
        
        # So the buggy doesn't start moving without user input
        self.buggy_vel = 0  
        self.controller = Controller(buggy_name)
        self.lock = threading.Lock()

        self.root = tk.Tk()

        self.root.title(buggy_name + ' Manual Velocity: scale = 0.1m/s')
        self.root.geometry('600x100')
        self.root.configure(background='#342d66')

        self.scale = tk.Scale(self.root, from_=0, to=300, orient=tk.HORIZONTAL, length=500, width=30)
        self.scale.pack()

        self.root.bind("<Up>", lambda i: self.scale.set(self.scale.get() + 2))
        self.root.bind("<Down>", lambda d: self.scale.set(self.scale.get() - 2))

        # ROS2 timer for stepping
        self.create_timer(0.01, self.step)  # equivalent to 100Hz (100 times per second)

    def step(self):
        # Sets the velocity of the buggy to the current scale value
        # called every tick
        self.root.update_idletasks()
        self.root.update()
        # Update velocity of the buggy
        self.buggy_vel = self.scale.get() / 10  # set velocity with 0.1 precision
        self.controller.set_velocity(self.buggy_vel)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: velocity_ui <initial_velocity> <buggy_name>")
        sys.exit(1)

    init_vel = float(sys.argv[1])
    buggy_name = sys.argv[2]

    vel_ui = VelocityUI(init_vel, buggy_name)

    try:
        rclpy.spin(vel_ui)
    except KeyboardInterrupt:
        print("Shutting down velocity UI")
    finally:
        vel_ui.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()