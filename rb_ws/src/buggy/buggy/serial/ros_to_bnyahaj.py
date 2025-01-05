import argparse
from threading import Lock
import threading

import rclpy
from rclpy import Node
from scipy.spatial.transform import Rotation
import utm

from example_interfaces.msg import Float64, Int8, UInt8, Bool

from host_comm import *

from nav_msgs.msg import Odometry as ROSOdom

class Translator(Node):
    """
    Translates the output from bnyahaj serial (interpreted from host_comm) to ros topics and vice versa.
    Performs reading (from Bnya Serial) and writing (from Ros Topics) on different python threads, so
    be careful of multithreading synchronizaiton issues.
    SC:
    (ROS) Self Steering topic --> (Bnyahaj) Stepper Motor
    (Bnyahaj) NAND Odom --> (ROS) NAND Odom topic

    NAND:
    (ROS) Self Steering --> (Bnyahaj) Stepper Motor
    (Bnyahaj) Self Odom from UKF --> (ROS) NAND Odom topic
    """

    def __init__(self, self_name, other_name, teensy_name):
        """
        self_name: Buggy namespace
        other_name: Only requried by SC for passing buggy namespace
        teensy_name: required for communication, different for SC and NAND

        Initializes the subscribers, rates, and ros topics (including debug topics)
        """
        super().__init__('ROS_serial_translator')

        self.comms = Comms("/dev/" + teensy_name)
        self.steer_angle = 0
        self.alarm = 0
        self.fresh_steer = False
        self.lock = Lock()

        self.create_subscription(
            Float64, self_name + "/buggy/input/steering", self.set_steering, 1
        )

        self.create_subscription(Int8, self_name + "/debug/sanity_warning", self.set_alarm, 1)

        # ISSUE: https://github.com/CMU-Robotics-Club/RoboBuggy2/issues/83
        if other_name is None and self_name == "NAND":
            self.odom_publisher = self.create_publisher(
                ROSOdom, self_name + "/nav/odom", 1
            )
        else:
            self.odom_publisher = self.create_publisher(
                ROSOdom, other_name + "/nav/odom", 1
            )

        # upper bound of steering update rate, make sure auton sends slower than 500 Hz or update / 2ms
        self.steer_send_rate = self.create_rate(500)

        # upper bound of reading data from Bnyahaj Serial, at 1ms
        self.read_rate = self.create_rate(1000)

        # ISSUE: https://github.com/CMU-Robotics-Club/RoboBuggy2/issues/90
        self.rc_steering_angle_publisher = self.create_publisher(
            Float64, self_name + "/buggy/debug/rc_steering_angle", 1
        )
        self.steering_angle_publisher = self.create_publisher(
            Float64, self_name + "/buggy/debug/steering_angle", 1
        )
        self.battery_voltage_publisher = self.create_publisher(
            Float64, self_name + "/buggy/debug/battery_voltage", 1
        )
        self.operator_ready_publisher = self.create_publisher(
            Bool, self_name + "/buggy/debug/operator_ready", 1
        )
        self.steering_alarm_publisher = self.create_publisher(
            Bool, self_name + "/buggy/debug/steering_alarm", 1
        )
        self.brake_status_publisher = self.create_publisher(
            Bool, self_name + "/buggy/debug/brake_status", 1
        )
        self.use_auton_steer_publisher = self.create_publisher(
            Bool, self_name + "/buggy/debug/use_auton_steer", 1
        )
        self.rc_uplink_qual_publisher = self.create_publisher(
            UInt8, self_name + "/buggy/debug/rc_uplink_quality", 1
        )
        self.nand_fix_publisher = self.create_publisher(
            UInt8, self_name + "/buggy/debug/nand_fix", 1
        )

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
        self.get_logger().debug(f"Read steeering angle of: {msg.data}")
        # print("Steering angle: " + str(msg.data))
        # print("SET STEERING: " + str(msg.data))
        with self.lock:
            self.steer_angle = msg.data
            self.fresh_steer = True

    def writer_thread(self):
        """
        Sends ROS Topics to bnayhaj serial, only sends a steering angle when we receive a fresh one
        Will send steering and alarm node.
        TODO: Does alarm node exist for NAND?
        """
        self.get_logger().info("Starting sending alarm and steering to teensy!")

        while rclpy.ok():
            if self.fresh_steer:
                with self.lock:
                    self.comms.send_steering(self.steer_angle)
                    self.get_logger().debug(f"Sent steering angle of: {self.steer_angle}")
                    self.fresh_steer = False

            with self.lock:
                self.comms.send_alarm(self.alarm)

            self.steer_send_rate.sleep()

    def reader_thread(self):
        """
        Reads three different types of packets, that should have better ways of differntiating
        Odometry -> (SC) NAND Odomotery
        BnyaTelemetry -> (NAND) Self Odom
        tuple -> (SC, maybe NAND?) Debug Info
        """
        self.get_logger().info("Starting reading odom from teensy!")
        while rclpy.ok():
            packet = self.comms.read_packet()
            self.get_logger().debug("packet" + str(packet))

            if isinstance(packet, Odometry):
                # Publish to odom topic x and y coord
                odom = ROSOdom()
                # convert to long lat
                try:
                    # TODO check if the order of y and x here is accurate
                    lat, long = utm.to_latlon(packet.y, packet.x, 17, "T")
                    odom.pose.pose.position.x = long
                    odom.pose.pose.position.y = lat
                    self.odom_publisher.publish(odom)
                except Exception as e:
                    self.get_logger().warn(
                        "Unable to convert other buggy position to lon lat" + str(e)
                    )

            elif isinstance(packet, BnyaTelemetry):
                odom = ROSOdom()

                # TODO: Not mock rolled accurately (Needs to be Fact Checked)
                try:
                    lat, long = utm.to_latlon(packet.x, packet.y, 17, "T")
                    odom.pose.pose.position.x = long
                    odom.pose.pose.position.y = lat
                    odom.twist.twist.angular.z = packet.heading_rate
                    heading = Rotation.from_euler('x', packet.heading, degrees=False).as_quat()

                    odom.pose.pose.orientation.x = heading[0]
                    odom.pose.pose.orientation.y = heading[1]
                    odom.pose.pose.orientation.z = heading[2]
                    odom.pose.pose.orientation.w = heading[3]

                    speed = packet.velocity
                    # TODO: fix this
                    # why this works: autonsystem only cares about magnitude of velocity
                    # so setting an arbitrary axis to the speed and leave other at 0
                    # works. However, this should be done properly ASAP
                    odom.twist.twist.linear.x = speed
                    self.odom_publisher.publish(odom)
                except Exception as e:
                    self.get_logger().warn(
                        "Unable to convert other buggy position to lon lat" + str(e)
                    )

            elif isinstance(
                    packet, tuple
            ):  # Are there any other packet that is a tuple
                self.rc_steering_angle_publisher.publish(Float64(data=packet[0]))
                self.steering_angle_publisher.publish(Float64(data=packet[1]))
                self.battery_voltage_publisher.publish(Float64(data=packet[2]))
                self.operator_ready_publisher.publish(Bool(data=packet[3]))
                self.steering_alarm_publisher.publish(Bool(data=packet[4]))
                self.brake_status_publisher.publish(Bool(data=packet[5]))
                self.use_auton_steer_publisher.publish(Bool(data=packet[6]))
                self.rc_uplink_qual_publisher.publish(UInt8(data=packet[7]))
                self.nand_fix_publisher.publish(UInt8(data=packet[8]))

            self.read_rate.sleep()

    def loop(self):
        """
        Initialies the reader and writer thread, should theoretically never finish as there are while loops
        """
        p1 = threading.Thread(target=self.writer_thread)
        p2 = threading.Thread(target=self.reader_thread)

        p1.start()
        p2.start()

        p1.join()
        p2.join()


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

    translate = Translator(self_name, other_name, teensy_name)

    if self_name == "SC" and other_name is None:
        translate.get_logger().warn(
            "Not reading NAND Odometry messages, double check roslaunch files for ros_to_bnyahaj"
        )
    elif other_name is None:
        translate.get_logger().info("No other name passed in, presuming that this is NAND ")

    rclpy.spin(translate)

    rclpy.shutdown()
