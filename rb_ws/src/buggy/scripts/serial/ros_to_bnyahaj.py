#!/usr/bin/env python3

import argparse
from threading import Lock
import threading
import rclpy
from host_comm import *
from rclpy.node import Node

from std_msgs.msg import Float64, Int8, Int32, UInt8, Bool, UInt64
from nav_msgs.msg import Odometry
from buggy.msg import *
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

        # upper bound of reading data from Bnyahaj Serial, at 1ms
        self.timer = self.create_timer(0.001, self.loop)


        # DEBUG MESSAGE PUBLISHERS:
        self.sc_debug_info_publisher = self.create_publisher(SCDebugInfoMsg, "/debug/firmware", 1)
        self.sc_sensor_publisher = self.create_publisher(SCSensorMsg, "/debug/sensor", 1)
        self.heading_rate_publisher = self.create_publisher(
            Float64, "/debug/heading_rate", 1
        )
        self.encoder_angle_publisher = self.create_publisher(
            Float64, "/debug/encoder_angle", 1
        )
        self.rc_steering_angle_publisher = self.create_publisher(
            Float64, "/debug/rc_steering_angle", 1
        )
        self.software_steering_angle_publisher = self.create_publisher(
            Float64, "/debug/software_steering_angle", 1
        )
        self.true_steering_angle_publisher = self.create_publisher(
            Float64, "/debug/true_steering_angle", 1
        )
        self.rfm69_timeout_num_publisher = self.create_publisher(
            Int32, "/debug/rfm_timeout_num", 1
        )
        self.operator_ready_publisher = self.create_publisher(
            Bool, "/debug/operator_ready", 1
        )
        self.brake_status_publisher = self.create_publisher(
            Bool, "/debug/brake_status", 1
        )
        self.use_auton_steer_publisher = self.create_publisher(
            Bool, "/debug/use_auton_steer", 1
        )
        self.tx12_state_publisher = self.create_publisher(
            Bool, "/debug/tx12_connected", 1
        )
        self.stepper_alarm_publisher = self.create_publisher(
            UInt8, "/debug/steering_alarm", 1
        )
        self.rc_uplink_qual_publisher = self.create_publisher(
            UInt8, "/debug/rc_uplink_quality", 1
        )
        self.nand_gps_seqnum_publisher = self.create_publisher(
            Int32, "/debug/NAND_gps_seqnum", 1
        )

        # SERIAL DEBUG PUBLISHERS
        self.roundtrip_time_publisher = self.create_publisher(
            Float64, "/debug/roundtrip_time", 1
        )

        if self.self_name == "NAND":
            # NAND POSITION PUBLISHERS
            self.nand_ukf_odom_publisher = self.create_publisher(
                Odometry, "/raw_state", 1
            )
            self.nand_gps_odom_publisher = self.create_publisher(
                Odometry, "/debug/gps_odom", 1
            )

            self.nand_gps_fix_publisher = self.create_publisher(
                UInt8, "/debug/gps_fix", 1
            )
            self.nand_gps_acc_publisher = self.create_publisher(
                Float64, "/debug/gps_accuracy", 1
            )

            self.nand_gps_time_publisher = self.create_publisher(
                UInt64, "/debug/gps_time", 1
            )

        if self.self_name == "SC":

            # SC SENSOR PUBLISHERS
            self.sc_velocity_publisher = self.create_publisher(
                Float64, "/sensors/velocity", 1
            )
            self.sc_steering_angle_publisher = self.create_publisher(
                Float64, "/sensors/steering_angle", 1
            )

            # RADIO DATA PUBLISHER
            self.observed_nand_odom_publisher = self.create_publisher(
                    Odometry, "/NAND_raw_state", 1
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
        self.get_logger().debug(f"Read steering angle of: {msg.data}")
        with self.lock:
            self.steer_angle = msg.data
            self.fresh_steer = True

    def loop(self):
        packet_on_buffer = True
        while packet_on_buffer:
            packet = self.comms.read_packet()
            if (packet is None): packet_on_buffer = False

            if isinstance(packet, NANDDebugInfo):
                self.heading_rate_publisher.publish(data=packet.heading_rate)
                self.encoder_angle_publisher.publish(data=packet.encoder_angle)
                self.rc_steering_angle_publisher.publish(data=packet.rc_steering_angle)
                self.software_steering_angle_publisher.publish(data=packet.software_steering_angle)
                self.true_steering_angle_publisher.publish(data=packet.true_steering_angle)
                self.rfm69_timeout_num_publisher.publish(data=packet.rfm69_timeout_num)
                self.operator_ready_publisher.publish(data=packet.operator_ready)
                self.brake_status_publisher.publish(data=packet.brake_status)
                self.use_auton_steer_publisher.publish(data=packet.auton_steer)
                self.tx12_state_publisher.publish(data=packet.tx12_state)
                self.stepper_alarm_publisher.publish(data=packet.stepper_alarm)
                self.rc_uplink_qual_publisher.publish(data=packet.rc_uplink_quality)
                self.get_logger().debug(f'NAND Debug Timestamp: {packet.timestamp}')
            elif isinstance(packet, NANDUKF):
                odom = Odometry()
                odom.pose.pose.position.x = packet.easting
                odom.pose.pose.position.y = packet.northing
                odom.pose.pose.orientation.z = packet.theta

                odom.twist.twist.linear.x = packet.velocity
                odom.twist.twist.angular.z = packet.heading_rate

                self.nand_ukf_odom_publisher.publish(data=odom)
                self.get_logger().debug(f'NAND UKF Timestamp: {packet.timestamp}')


            elif isinstance(packet, NANDRawGPS):
                odom = Odometry()
                odom.pose.pose.position.x = packet.easting
                odom.pose.pose.position.y = packet.northing
                odom.pose.pose.orientation.z = 0
                odom.twist.twist.linear.x = 0
                odom.twist.twist.linear.y = 0
                odom.twist.twist.angular.z = 0

                self.nand_gps_odom_publisher.publish(data=odom)
                self.nand_gps_fix_publisher.publish(data=packet.gps_fix)
                self.nand_gps_acc_publisher.publish(data=packet.accuracy)
                self.nand_gps_seqnum_publisher.publish(data=packet.gps_seqnum)
                self.nand_gps_time_publisher.publish(data=packet.gps_time)
                self.get_logger().debug(f'NAND Raw GPS Timestamp: {packet.timestamp}')


            # this packet is received on Short Circuit
            elif isinstance(packet, Radio):

                # Publish to odom topic x and y coord
                odom = Odometry()

                odom.pose.pose.position.x = packet.nand_east_gps
                odom.pose.pose.position.y = packet.nand_north_gps
                self.observed_nand_odom_publisher.publish(data=odom)
                self.nand_gps_seqnum_publisher.publish(data=packet.gps_seqnum)

            elif isinstance(packet, SCDebugInfo):
                rospacket = SCDebugInfoMsg()
                rospacket.rc_steering_angle = packet.rc_steering_angle
                rospacket.software_steering_angle = packet.software_steering_angle
                rospacket.true_steering_angle = packet.true_steering_angle
                rospacket.operator_ready = packet.operator_ready
                rospacket.brake_status = packet.brake_status
                rospacket.use_auton_steer = packet.auton_steer
                rospacket.tx12_state = packet.tx12_state
                rospacket.stepper_alarm = packet.stepper_alarm
                rospacket.rc_uplink_qual = packet.rc_uplink_quality
                self.sc_debug_info_publisher.publish(rospacket)
                self.get_logger().debug(f'SC Debug Timestamp: {packet.timestamp}')


            elif isinstance(packet, SCSensors):
                rospacket = SCSensorMsg()
                rospacket.velocity = packet.velocity
                rospacket.steering_angle = packet.steering_angle
                self.sc_sensor_publisher.publish(rospacket)
                
                self.get_logger().debug(f'SC Sensors Timestamp: {packet.timestamp}')


            elif isinstance(packet, RoundtripTimestamp):

                self.get_logger().info(f'Roundtrip Timestamp: {packet.returned_time}, {(time.time_ns() * 1e-6 - packet.returned_time) * 1e-3}')
                self.roundtrip_time_publisher.publish(Float64(data=(time.time_ns() * 1e-6 - packet.returned_time) * 1e-3))
        
        if self.fresh_steer:
            with self.lock:
                self.comms.send_steering(self.steer_angle)
                # self.get_logger().info(f"Sent steering angle of: {self.steer_angle}")
                self.fresh_steer = False

        with self.lock:
            self.comms.send_alarm(self.alarm)
        with self.lock:
            self.comms.send_timestamp(time.time_ns() * 1e-6)


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
