#!/usr/bin/env python3

from threading import Lock
import rclpy
from host_comm import *
from rclpy.node import Node

from std_msgs.msg import Float64, Int8
from nav_msgs.msg import Odometry
from buggy.msg import *
class Translator(Node):
    """
    Translates the output from bnyahaj serial (interpreted from host_comm) to ros topics and vice versa.
    Performs reading (from Bnya Serial) and writing (from Ros Topics) on different python threads, so
    be careful of multithreading synchronizaiton issues.
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

        self.steer_angle = 0
        self.alarm = 0
        self.fresh_steer = False
        self.lock = Lock()

        self.create_subscription(
            Float64, "input/steering", self.set_steering, 1
        )
        self.create_subscription(Int8, "input/sanity_warning", self.set_alarm, 1)

        # upper bound of reading data from Bnyahaj Serial, at 1ms
        self.timer = self.create_timer(0.001, self.loop)


        # DEBUG MESSAGE PUBLISHERS:
        if self.self_name == "SC":
            self.sc_debug_info_publisher = self.create_publisher(SCDebugInfoMsg, "debug/firmware", 1)
            self.sc_sensor_publisher = self.create_publisher(SCSensorMsg, "debug/sensor", 1)
        else:
            self.nand_debug_info_publisher = self.create_publisher(NANDDebugInfoMsg, "debug/firmware", 1)
            self.nand_raw_gps_publisher = self.create_publisher(NANDRawGPSMsg, "debug/raw_gps", 1)

        # SERIAL DEBUG PUBLISHERS
        self.roundtrip_time_publisher = self.create_publisher(
            Float64, "debug/roundtrip_time", 1
        )

        if self.self_name == "NAND":
            # NAND POSITION PUBLISHERS
            self.nand_ukf_odom_publisher = self.create_publisher(
                Odometry, "raw_state", 1
            )

        if self.self_name == "SC":

            # RADIO DATA PUBLISHER
            self.observed_nand_odom_publisher = self.create_publisher(
                    Odometry, "NAND_raw_state", 1
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
            if (packet is None): 
                packet_on_buffer = False
                self.get_logger().debug("NO PACKET")
            else:
                self.get_logger().debug("PACKET")

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

                odom.twist.twist.linear.x = packet.velocity
                odom.twist.twist.angular.z = packet.heading_rate

                self.nand_ukf_odom_publisher.publish(data=odom)
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
                self.get_logger().info("GOT PACKET")
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

                self.get_logger().debug(f'Roundtrip Timestamp: {packet.returned_time}, {(time.time_ns() * 1e-6 - packet.returned_time) * 1e-3}')
                self.roundtrip_time_publisher.publish(Float64(data=(time.time_ns() * 1e-6 - packet.returned_time) * 1e-3))

        if self.fresh_steer:
            with self.lock:
                self.comms.send_steering(self.steer_angle)
                self.get_logger().debug(f"Sent steering angle of: {self.steer_angle}")
                self.fresh_steer = False

        with self.lock:
            self.comms.send_alarm(self.alarm)
        with self.lock:
            self.comms.send_timestamp(time.time_ns() * 1e-6)


def main(args=None):
    rclpy.init(args=args)

    translator = Translator()
    rclpy.spin(translator)

    translator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()