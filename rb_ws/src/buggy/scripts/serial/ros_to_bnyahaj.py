#!/usr/bin/env python3

# import random
from threading import Lock #threading alows parallel processing for smaller units, and Lock makes sure that one of the points accesses a specific part(like a variable or change in variable) to ensure that no over-editing happens and it's more sequential
import rclpy #the rclpy library allows the creation of nodes, which is an individual processor within a graph
from host_comm import * #importing all named entities within a class that facilitates communication between NAND and SC
from rclpy.node import Node

from std_msgs.msg import Float64, Int8 #it's a message package and it definnes specific precision in terms of sigfigs and holding 8-bit numbers
from nav_msgs.msg import Odometry #the msg is involving with exploring through mapping and location while Odometry estimates where and how the robot will move
from buggy.msg import *
import numpy as np
class Translator(Node):
    """
    Translates the output from bnyahaj serial (interpreted from host_comm) to ros topics and vice versa.
    Performs reading (from Bnya Serial) and writing (from Ros Topics) on different python threads, so
    be careful of multithreading synchronizaiton issues.
    """ #Byna Serial is bridge between Software and Firmware so it's translating for both sides

    def __init__(self):
        """
        teensy_name: required for communication, different for SC and NAND

        Initializes the subscribers, rates, and ros topics (including debug topics)
        """
        #finds initial conditions
        super().__init__('ROS_serial_translator')
        self.get_logger().info('INITIALIZED.')

        #Parameters
        self.declare_parameter("teensy_name", "ttyUSB0") #Default is SC's port
        teensy_name = self.get_parameter("teensy_name").value

        #assigning the communication message to a specific buggy
        self.comms = Comms("/dev/" + teensy_name)
        namespace = self.get_namespace()
        if namespace == "/SC":
            self.self_name = "SC"
        else:
            self.self_name = "NAND"
        self.get_logger().info("BUGGY" + self.self_name)

        #initializing the angles and alarms and telling ros2b to hold off until stanley control to send you another angle
        self.steer_angle = 0
        self.steer_fw_timestamp = 0
        self.steer_sw_timestamp = 0

        self.alarm = 0
        self.fresh_steer = False
        self.lock = Lock()

        #creates two new recievers based on new sent messages from control to update the steering angle and the alarm
        self.create_subscription(
            StampedFloat64Msg, "input/steering", self.set_steering, 1
        )
        self.create_subscription(Int8, "input/sanity_warning", self.set_alarm, 1)

        # upper bound of reading data from Bnyahaj Serial, at 1ms
        self.timer = self.create_timer(0.001, self.loop)


        # DEBUG MESSAGE PUBLISHERS: it's finding out the data if it's SC then it appends it to debug data that; if NAND then it not only does that for what it's sending to SC and creates a limit to the buffer if it's too behind on information
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

        # SERIAL DEBUG PUBLISHERS: sends the debugged code and information to Firmware- this is mainly for refernce so we can figure out what's going on and know what to improve, it's not for them
        self.roundtrip_time_publisher = self.create_publisher(
            Float64, "debug/roundtrip_time", 1
        )
        self.teensycycle_time_publisher = self.create_publisher(
            Float64, "debug/teensycycle_time", 1
        )
        self.control_latency_publisher = self.create_publisher(
            Float64, "debug/control_latency", 1
        )

    def set_alarm(self, msg): #it identifies any errors and records them
        """
        alarm ros topic reader, locked so that only one of the setters runs at once
        """
        with self.lock:
            self.get_logger().debug(f"Reading alarm of {msg.data}")
            self.alarm = msg.data

    def set_steering(self, msg: StampedFloat64Msg):
        """
        Steering Angle Updater, updates the steering angle and software/firmware timestamps locally
        if updated on rostopic
        """
        self.get_logger().debug(f"Read steering angle of: {msg.data}")

        try:
            fw_stamp = int(msg.header.frame_id) #convert a string into an interger- but if it's not a string, then we assign it a default values
        except ValueError:
            fw_stamp = 0

        sw_stamp = msg.header.stamp.sec * int(1e9) + msg.header.stamp.nanosec

        with self.lock: #record timestamp based on the headerID conversion
            self.steer_angle = msg.data
            self.steer_fw_timestamp = fw_stamp
            self.steer_sw_timestamp = sw_stamp
            self.fresh_steer = True

    def loop(self):
        packet_on_buffer = True
        while packet_on_buffer: #reads the packet and decides which to log to debugger based on if it is a packet or not
            packet = self.comms.read_packet()
            if (packet is None):
                packet_on_buffer = False
                self.get_logger().debug("NO PACKET")
            else:
                self.get_logger().debug("PACKET")

            if isinstance(packet, NANDDebugInfo): #if the packet is from NANDDebugInfo, it updates the information for the rospacket so it sends specific information to the debug info so we know all the details for the frame of reference
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
            elif isinstance(packet, NANDUKF): #updates odom position and angle if the pcket is from NAND's UFK
                odom = Odometry() #figuring out the current position of the buggy
                odom.pose.pose.position.x = packet.easting
                odom.pose.pose.position.y = packet.northing
                odom.pose.pose.orientation.z = packet.theta

                self.nandCircArray[self.nandIndex] = packet.velocity #figures out the movement part of the buggy
                self.nandIndex = (self.nandIndex + 1) % self.CIRCLEN
                odom.twist.twist.linear.x = np.mean(self.nandCircArray)
                odom.twist.twist.angular.z = packet.heading_rate

                odom.header.frame_id = str(packet.timestamp)

                self.nand_ukf_odom_publisher.publish(odom)#publishes updates odom to the odm log
                self.get_logger().debug(f'NAND UKF Timestamp: {packet.timestamp}')#logs the timing


            elif isinstance(packet, NANDRawGPS): #giving qualities to the rospacket based on position if the packet is NAND's raw gps
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
            elif isinstance(packet, Radio): #publishes to the odom if the packet is Radio

                # Publish to odom topic x and y coord
                self.get_logger().debug("GOT RADIO PACKET")
                odom = Odometry()

                odom.pose.pose.position.x = packet.nand_east_gps
                odom.pose.pose.position.y = packet.nand_north_gps
                self.observed_nand_odom_publisher.publish(odom)

            elif isinstance(packet, SCDebugInfo): #attributing driving qualities to the rospacket if the packet is in SCDebugInfo
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


            elif isinstance(packet, SCSensors): #if the packet is from SC's Sensor, it updates the velocity and steering angle
                rospacket = SCSensorMsg()
                rospacket.velocity = packet.velocity
                rospacket.steering_angle = packet.steering_angle
                self.sc_sensor_publisher.publish(rospacket)

                self.get_logger().debug(f'SC Sensors Timestamp: {packet.timestamp}')


            elif isinstance(packet, RoundtripTimestamp): #If the packet is from the roundtriptimestamp then it sends a 64 float point
                rtt = (time.time_ns() - packet.returned_time) * 1e-9
                self.get_logger().debug(f'Roundtrip Timestamp: {packet.returned_time}, RTT: {rtt}')
                self.roundtrip_time_publisher.publish(Float64(data=rtt))
                self.teensycycle_time_publisher.publish(Float64(data=packet.teensy_cycle_time * 1e-6))

        if self.fresh_steer: #if it's a new, updated angle, then we send an update to firmware
            with self.lock:
                self.comms.send_steering(self.steer_angle, self.steer_fw_timestamp)
                sw_dt = (time.time_ns() - self.steer_sw_timestamp) * 1e-9
                self.control_latency_publisher.publish(Float64(data=sw_dt))

                self.get_logger().debug(f"Sent steering angle of: {self.steer_angle}")
                self.fresh_steer = False

        with self.lock:
            self.comms.send_alarm(self.alarm)
        with self.lock:
            self.comms.send_timestamp(time.time_ns())


def main(args=None):
    rclpy.init(args=args)

    translator = Translator()
    rclpy.spin(translator)

    translator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()