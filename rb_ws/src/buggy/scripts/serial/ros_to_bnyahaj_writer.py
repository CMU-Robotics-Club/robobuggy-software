#!/usr/bin/env python3

import argparse
from threading import Lock
import threading
import rclpy
from host_comm import *
from rclpy.node import Node

from std_msgs.msg import Float64, Int8, Int32, UInt8, Bool, UInt64
from nav_msgs.msg import Odometry

class Translator(Node):
    """
    Translates the output from bnyahaj serial (interpreted from host_comm) to ros topics and vice versa.
    Performs reading (from Bnya Serial) and writing (from Ros Topics) on different python threads, so
    be careful of multithreading synchronizaiton issues.
    """

    def __init__(self, teensy_name):
        """
        teensy_name: required for communication, different for SC and NAND

        Initializes the subscribers, rates, and ros topics (including debug topics)
        """

        super().__init__('ROS_serial_translator')

        self.comms = Comms("/dev/" + teensy_name)
        namespace = self.get_namespace()
        if namespace == "/SC":
            self.self_name = "SC"
        else:
            self.self_name = "NAND"

        self.steer_angle = 0
        self.alarm = 0
        self.fresh_steer = False
        self.lock = Lock()

        self.create_subscription(
            Float64, "/input/steering", self.set_steering, 1
        )
        self.create_subscription(Int8, "/input/sanity_warning", self.set_alarm, 1)

        # upper bound of steering update rate, make sure auton sends slower than 500 Hz or update / 2ms
        self.writer_timer = self.create_timer(0.001, self.writer_loop)


    def set_alarm(self, msg):
        """
        alarm ros topic reader, locked so that only one of the setters runs at once
        """
        with self.lock:
            self.get_logger().debug(f"Reading alarm of {msg.data}")
            self.alarm = msg.data

    def set_steering(self, msg):
        """
        Steering Angle Updater, updates the steering angle locally if updated on ros stopic
        """
        self.get_logger().debug(f"Read steering angle of: {msg.data}")
        with self.lock:
            self.steer_angle = msg.data
            self.fresh_steer = True

    def writer_loop(self):
        """
        Sends ROS Topics to bnayhaj serial, only sends a steering angle when we receive a fresh one
        Will send steering and alarm node.
        """
        if self.fresh_steer:
            with self.lock:
                self.comms.send_steering(self.steer_angle)
                self.get_logger().info(f"Sent steering angle of: {self.steer_angle}")
                self.fresh_steer = False

        with self.lock:
            self.comms.send_alarm(self.alarm)
        with self.lock:
            self.comms.send_timestamp(time.time())



# Initializes ros nodes, using self and other name
# other name is not requires, and if not submitted, use NONE
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--self_name", type=str, help="name of ego-buggy", required=True
    )
    parser.add_argument(
        "--other_name",
        type=str,
        help="name of other buggy",
        required=False,
        default=None,
    )
    parser.add_argument(
        "--teensy_name", type=str, help="name of teensy port", required=True
    )
    args, _ = parser.parse_known_args()
    self_name = args.self_name
    other_name = args.other_name
    teensy_name = args.teensy_name

    rclpy.init()

    translate = Translator(teensy_name)

    if self_name == "SC" and other_name is None:
        translate.get_logger().warn(
            "Not reading NAND Odometry messages, double check roslaunch files for ros_to_bnyahaj"
        )
    elif other_name is None:
        translate.get_logger().info("No other name passed in, presuming that this is NAND ")

    rclpy.spin(translate)

    rclpy.shutdown()
