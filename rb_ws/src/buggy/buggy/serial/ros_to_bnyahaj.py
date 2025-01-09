import argparse
from threading import Lock
import threading
import rclpy
from rclpy import Node

from std_msgs.msg import Float64, Int8, Int32, UInt8, Bool, UInt64
from host_comm import *
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
            Float64, self_name + "/input/steering", self.set_steering, 1
        )
        self.create_subscription(Int8, self_name + "/input/sanity_warning", self.set_alarm, 1)

        # upper bound of steering update rate, make sure auton sends slower than 500 Hz or update / 2ms
        self.steer_send_rate = self.create_rate(500)

        # upper bound of reading data from Bnyahaj Serial, at 1ms
        self.read_rate = self.create_rate(1000)

        # DEBUG MESSAGE PUBLISHERS:
        self.heading_rate_publisher = self.create_publisher(
            Float64, self_name + "/debug/heading_rate", 1
        )
        self.encoder_angle_publisher = self.create_publisher(
            Float64, self_name + "/debug/encoder_angle", 1
        )
        self.rc_steering_angle_publisher = self.create_publisher(
            Float64, self_name + "/debug/rc_steering_angle", 1
        )
        self.software_steering_angle_publisher = self.create_publisher(
            Float64, self_name + "/debug/software_steering_angle", 1
        )
        self.true_steering_angle_publisher = self.create_publisher(
            Float64, self_name + "/debug/true_steering_angle", 1
        )
        self.rfm69_timeout_num_publisher = self.create_publisher(
            Int32, self_name + "/debug/rfm_timeout_num", 1
        )
        self.operator_ready_publisher = self.create_publisher(
            Bool, self_name + "/debug/operator_ready", 1
        )
        self.brake_status_publisher = self.create_publisher(
            Bool, self_name + "/debug/brake_status", 1
        )
        self.use_auton_steer_publisher = self.create_publisher(
            Bool, self_name + "/debug/use_auton_steer", 1
        )
        self.tx12_state_publisher = self.create_publisher(
            Bool, self_name + "/debug/tx12_connected", 1
        )
        self.stepper_alarm_publisher = self.create_publisher(
            UInt8, self_name + "/debug/steering_alarm", 1
        )
        self.rc_uplink_qual_publisher = self.create_publisher(
            UInt8, self_name + "/debug/rc_uplink_quality", 1
        )

        # NAND POSITION PUBLISHERS
        self.nand_ukf_odom_publisher = self.create_publisher(
            Odometry, "NAND/nav/odom", 1
        )
        self.nand_gps_odom_publisher = self.create_publisher(
            Odometry, "NAND/debug/gps_odom", 1
        )
        self.observed_nand_odom_publisher = self.create_publisher(
            Odometry, "SC/nav/NAND_odom", 1
        )
        self.nand_gps_fix_publisher = self.create_publisher(
            UInt8, "NAND/debug/gps_fix", 1
        )
        self.nand_gps_acc_publisher = self.create_publisher(
            Float64, "NAND/debug/gps_accuracy", 1
        )
        self.nand_gps_seqnum_publisher = self.create_publisher(
            Int32, "NAND/debug/gps_seqnum", 1
        )
        self.nand_gps_time_publisher = self.create_publisher(
            UInt64, "NAND/debug/gps_time", 1
        )

        # SC SENSOR PUBLISHERS
        self.sc_velocity_publisher = self.create_publisher(
            Float64, "SC/sensors/velocity", 1
        )
        self.sc_steering_angle_publisher = self.create_publisher(
            Float64, "SC/sensors/steering_angle", 1
        )

        # SERIAL DEBUG PUBLISHERS
        self.roundtrip_time_publisher = self.create_publisher(
            Float64, self_name + "/debug/roundtrip_time", 1
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
        # print("Steering angle: " + str(msg.data))
        # print("SET STEERING: " + str(msg.data))
        with self.lock:
            self.steer_angle = msg.data
            self.fresh_steer = True


    def writer_thread(self):
        """
        Sends ROS Topics to bnayhaj serial, only sends a steering angle when we receive a fresh one
        Will send steering and alarm node.
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
            with self.lock:
                self.comms.send_timestamp(time.time())

            self.steer_send_rate.sleep()

    def reader_thread(self):
        self.get_logger().info("Starting reading odom from teensy!")
        while rclpy.ok():
            packet = self.comms.read_packet()
            self.get_logger().debug("packet" + str(packet))

            if isinstance(packet, NANDDebugInfo):
                self.heading_rate_publisher.publish(packet.heading_rate)
                self.encoder_angle_publisher.publish(packet.encoder_angle)
                self.rc_steering_angle_publisher.publish(packet.rc_steering_angle)
                self.software_steering_angle_publisher.publish(packet.software_steering_angle)
                self.true_steering_angle_publisher.publish(packet.true_steering_angle)
                self.rfm69_timeout_num_publisher.publish(packet.rfm69_timeout_num)
                self.operator_ready_publisher.publish(packet.operator_ready)
                self.brake_status_publisher.publish(packet.brake_status)
                self.use_auton_steer_publisher.publish(packet.auton_steer)
                self.tx12_state_publisher.publish(packet.tx12_state)
                self.stepper_alarm_publisher.publish(packet.stepper_alarm)
                self.rc_uplink_qual_publisher.publish(packet.rc_uplink_quality)

            elif isinstance(packet, NANDUKF):
                odom = Odometry()

                # NOTE: this data is published as an Odometry object per the BuggyState specs: https://github.com/CMU-Robotics-Club/robobuggy-software/wiki/BuggyState#buggystate-definition-as-a-navodom-ros-msg
                odom.pose.pose.position.x = packet.easting
                odom.pose.pose.position.y = packet.northing
                odom.pose.pose.orientation.z = packet.theta

                # why this works: autonsystem only cares about magnitude of velocity so setting an arbitrary axis to the speed and leave other at 0 works.
                odom.twist.twist.linear.x = packet.velocity
                odom.twist.twist.angular.z = packet.heading_rate

                self.nand_ukf_odom_publisher.publish(odom)

            elif isinstance(packet, NANDRawGPS):
                odom = Odometry()
                # NOTE: this data is published as an Odometry object per the BuggyState specs: https://github.com/CMU-Robotics-Club/robobuggy-software/wiki/BuggyState#buggystate-definition-as-a-navodom-ros-msg
                odom.pose.pose.position.x = packet.easting
                odom.pose.pose.position.y = packet.northing
                odom.pose.pose.orientation.z = 0
                odom.twist.twist.linear.x = 0
                odom.twist.twist.linear.y = 0
                odom.twist.twist.angular.z = 0

                self.nand_gps_odom_publisher.publish(odom)
                self.nand_gps_fix_publisher.publish(packet.gps_fix)
                self.nand_gps_acc_publisher.publish(packet.accuracy)
                self.nand_gps_seqnum_publisher.publish(packet.gps_seqnum)
                self.nand_gps_time_publisher.publish(packet.gps_time)

            elif isinstance(packet, Radio):

                # Publish to odom topic x and y coord
                odom = Odometry()

                odom.pose.pose.position.x = packet.nand_east_gps
                odom.pose.pose.position.y = packet.nand_north_gps
                self.observed_nand_odom_publisher.publish(odom)

            elif isinstance(packet, SCDebugInfo):
                self.encoder_angle_publisher.publish(packet.encoder_angle)
                self.rc_steering_angle_publisher.publish(packet.rc_steering_angle)
                self.software_steering_angle_publisher.publish(packet.software_steering_angle)
                self.true_steering_angle_publisher.publish(packet.true_steering_angle)
                self.operator_ready_publisher.publish(packet.operator_ready)
                self.brake_status_publisher.publish(packet.brake_status)
                self.use_auton_steer_publisher.publish(packet.auton_steer)
                self.tx12_state_publisher.publish(packet.tx12_state)
                self.stepper_alarm_publisher.publish(packet.stepper_alarm)
                self.rc_uplink_qual_publisher.publish(packet.rc_uplink_quality)

            elif isinstance(packet, SCSensors):
                self.sc_velocity_publisher.publish(packet.velocity)
                self.sc_steering_angle_publisher.publish(packet.steering_angle)

            elif isinstance(packet, RoundtripTimestamp):
                self.roundtrip_time_publisher.publish(time.time() - packet.returned_time)

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
