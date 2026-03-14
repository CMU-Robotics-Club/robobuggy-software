#!/usr/bin/env python3
"""
This is the only file that uses host_comm to read/write packets to firmware.
Further, it runs on the (default) SingleThreadedExecutor, so callback/loop
execution is implicitly mutually exclusive. Hence, we don't need locks here.
"""

import time

import rclpy
from host_comm import *
from rclpy.node import Node

from std_msgs.msg import Float64, Int8
from nav_msgs.msg import Odometry
from buggy.msg import *
import numpy as np

from buggy.msg import StampedFloat64Msg

# max number of packets to read from the buffer in each loop iteration
PACKET_READ_LIMIT = 20

class Translator(Node):
    """
    Translates the output from bnyahaj serial (interpreted from host_comm) to ROS topics and vice versa.

    When used with the default SingleThreadedExecutor, all callbacks and the main loop run in the same
    thread, so access to shared state in this class is implicitly serialized and no explicit locking is
    required.
    """

    def __init__(self):
        """
        teensy_name: required for communication, different for SC and NAND

        Initializes the subscribers, rates, and ros topics (including debug topics)
        """

        super().__init__('ROS_serial_translator')
        self.get_logger().info('INITIALIZED.')

        #Parameters
        self.declare_parameter("teensy_name", "ttyUSB0") #Default is SC's port
        teensy_name = self.get_parameter("teensy_name").value


        self.comms = Comms("/dev/" + teensy_name)
        namespace = self.get_namespace()
        if namespace == "/SC":
            self.self_name = "SC"
        else:
            self.self_name = "NAND"
        self.get_logger().info("BUGGY" + self.self_name)

        self.steer_angle = 0
        self.steer_fw_timestamp = 0
        self.steer_sw_timestamp = 0

        self.alarm = 0

        self.create_subscription(
            StampedFloat64Msg, "input/steering", self.set_steering, 1
        )
        self.create_subscription(Int8, "input/sanity_warning", self.set_alarm, 1)

        # upper bound of reading data from Bnyahaj Serial, at 1ms
        self.timer = self.create_timer(0.001, self.loop)

        # slower loop to send timestamp to teensy, at 10ms
        self.timestamp_timer = self.create_timer(0.01, self.send_timestamp)

        # DEBUG MESSAGE PUBLISHERS:
        if self.self_name == "SC":
            self.sc_debug_info_publisher = self.create_publisher(SCDebugInfoMsg, "debug/firmware", 1)
            self.sc_sensor_publisher = self.create_publisher(SCSensorMsg, "debug/sensor", 1)

            # RADIO DATA PUBLISHER
            self.observed_nand_odom_publisher = self.create_publisher(
                    Odometry, "NAND_raw_state", 1
                )
        else:
            self.nand_debug_info_publisher = self.create_publisher(NANDDebugInfoMsg, "debug/firmware", 1)
            self.nand_raw_gps_publisher = self.create_publisher(NANDRawGPSMsg, "debug/raw_gps", 1)
            self.nand_ukf_odom_publisher = self.create_publisher(
                Odometry, "raw_state", 1
            )
            self.CIRCLEN = 20
            self.nandCircArray = np.zeros(self.CIRCLEN)
            self.nandIndex = 0

        # SERIAL DEBUG PUBLISHERS
        self.roundtrip_time_publisher = self.create_publisher(
            Float64, "debug/roundtrip_time", 1
        )
        self.teensycycle_time_publisher = self.create_publisher(
            Float64, "debug/teensycycle_time", 1
        )
        self.control_latency_publisher = self.create_publisher(
            Float64, "debug/control_latency", 1
        )

    def set_alarm(self, msg):
        """
        alarm ros topic reader
        """
        self.get_logger().debug(f"Reading alarm of {msg.data}")
        self.alarm = msg.data
        self.comms.send_alarm(self.alarm)

    def set_steering(self, msg: StampedFloat64Msg):
        """
        Steering Angle Updater, updates the steering angle and software/firmware timestamps locally
        if updated on rostopic
        """
        self.get_logger().debug(f"Read steering angle of: {msg.data}")

        try:
            fw_stamp = int(msg.header.frame_id)
        except ValueError:
            fw_stamp = 0

        sw_stamp = msg.header.stamp.sec * int(1e9) + msg.header.stamp.nanosec

        self.steer_angle = msg.data
        self.steer_fw_timestamp = fw_stamp
        self.steer_sw_timestamp = sw_stamp

        self.comms.send_steering(self.steer_angle, self.steer_fw_timestamp)
        sw_dt = (time.time_ns() - self.steer_sw_timestamp) * 1e-9
        self.control_latency_publisher.publish(Float64(data=sw_dt))

        self.get_logger().debug(f"Sent steering angle of: {self.steer_angle}")

    def loop(self):
        packet_on_buffer = True
        # 20 packet limit to prevent starvation of write operations if there are too many packets on the buffer
        packets_processed = 0
        while packet_on_buffer and packets_processed < PACKET_READ_LIMIT:
            packet = self.comms.read_packet()
            if (packet is None):
                packet_on_buffer = False
                self.get_logger().debug("NO PACKET")
                continue
            else:
                self.get_logger().debug("PACKET")
                packets_processed += 1

            if isinstance(packet, NANDDebugInfo):
                rospacket = NANDDebugInfoMsg()
                rospacket.heading_rate = packet.heading_rate
                rospacket.encoder_angle = packet.encoder_angle
                rospacket.rc_steering_angle = packet.rc_steering_angle
                rospacket.software_steering_angle = packet.software_steering_angle
                rospacket.true_steering_angle = packet.true_steering_angle
                rospacket.rfm69_timeout_num = packet.rfm69_timeout_num
                rospacket.operator_ready = packet.operator_ready
                rospacket.brake_status = packet.brake_status
                rospacket.auton_steer = packet.auton_steer
                rospacket.tx12_state = packet.tx12_state
                rospacket.stepper_alarm = packet.stepper_alarm
                rospacket.rc_uplink_quality = packet.rc_uplink_quality
                self.nand_debug_info_publisher.publish(rospacket)

                self.get_logger().debug(f'NAND Debug Timestamp: {packet.timestamp}')
            elif isinstance(packet, NANDUKF):
                odom = Odometry()
                odom.pose.pose.position.x = packet.easting
                odom.pose.pose.position.y = packet.northing
                odom.pose.pose.orientation.z = packet.theta

                self.nandCircArray[self.nandIndex] = packet.velocity
                self.nandIndex = (self.nandIndex + 1) % self.CIRCLEN
                odom.twist.twist.linear.x = np.mean(self.nandCircArray)
                odom.twist.twist.angular.z = packet.heading_rate

                odom.header.frame_id = str(packet.timestamp)

                self.nand_ukf_odom_publisher.publish(odom)
                self.get_logger().debug(f'NAND UKF Timestamp: {packet.timestamp}')


            elif isinstance(packet, NANDRawGPS):
                rospacket = NANDRawGPSMsg()
                rospacket.easting = packet.easting
                rospacket.northing = packet.northing
                rospacket.accuracy = packet.accuracy
                rospacket.gps_time = packet.gps_time
                rospacket.gps_seqnum = packet.gps_seqnum
                rospacket.gps_fix = packet.gps_fix
                self.nand_raw_gps_publisher.publish(rospacket)

                self.get_logger().debug(f'NAND Raw GPS Timestamp: {packet.timestamp}')


            # this packet is received on Short Circuit
            elif isinstance(packet, Radio):

                # Publish to odom topic x and y coord
                self.get_logger().debug("GOT RADIO PACKET")
                odom = Odometry()

                odom.pose.pose.position.x = packet.nand_east_gps
                odom.pose.pose.position.y = packet.nand_north_gps
                self.observed_nand_odom_publisher.publish(odom)

            elif isinstance(packet, SCDebugInfo):
                self.get_logger().debug("GOT DEBUG PACKET")
                rospacket = SCDebugInfoMsg()
                rospacket.rc_steering_angle = packet.rc_steering_angle
                rospacket.software_steering_angle = packet.software_steering_angle
                rospacket.true_steering_angle = packet.true_steering_angle
                rospacket.operator_ready = packet.operator_ready
                rospacket.brake_status = packet.brake_status
                rospacket.auton_steer = packet.auton_steer
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
                rtt = (time.time_ns() - packet.returned_time) * 1e-9
                self.get_logger().debug(f'Roundtrip Timestamp: {packet.returned_time}, RTT: {rtt}')
                self.roundtrip_time_publisher.publish(Float64(data=rtt))
                self.teensycycle_time_publisher.publish(Float64(data=packet.teensy_cycle_time * 1e-6))

    def send_timestamp(self):
        self.comms.send_timestamp(time.time_ns())


def main(args=None):
    rclpy.init(args=args)

    translator = Translator()
    rclpy.spin(translator)

    translator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()