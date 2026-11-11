#!/usr/bin/env python3
"""
Translates the output from bnyahaj serial (interpreted from host_comm) to ROS topics and vice versa.

This node uses a MultiThreadedExecutor. Reads and writes occur asynchronously across different threads.
A standard Python threading.Lock is also used to ensure sends from timer loop and topic subscribers to
the firmware are strictly mutually exclusive to prevent serial collisions. Topic subscribers share a
MutuallyExclusiveCallbackGroup so only one subscriber callback executes at a time.
"""

import time
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from host_comm import *
from std_msgs.msg import Float64, Int8
from nav_msgs.msg import Odometry
from buggy.msg import *
import numpy as np
import pyproj

from buggy.msg import StampedFloat64Msg

from microstrain_inertial_msgs.msg import MipGnssFixInfo

from util.trajectory import Trajectory

class Translator(Node):

    # A path point the firmware already holds is resent once any of these is
    # exceeded. They are separate rather than one lateral tolerance because the
    # firmware uses a point's heading and curvature pointwise for feedforward, so a
    # shape change that barely moves the point can still matter: 0.002 1/m of
    # curvature is still under 0.15 deg of steering feedforward on SC's wheelbase.
    PATH_POS_TOL = 0.05  # m
    PATH_HEADING_TOL = 0.01  # rad, ~0.6 deg
    PATH_CURVATURE_TOL = 0.002  # 1/m

    # Indices this far behind the window start are kept in the mirror of the
    # firmware's path, so that small backwards jitter in the state estimate doesn't
    # drop entries the next window still needs and force a full resend.
    PATH_STORE_MARGIN_PTS = 8

    # Keeps the grid index arithmetic from landing on the wrong side of a ceil when
    # start_dist is an exact multiple of the point spacing
    PATH_GRID_EPS = 1e-9

    def __init__(self):
        """
        teensy_name: required for communication, different for SC and NAND

        Initializes the subscribers, rates, and ros topics (including debug topics)
        """

        super().__init__('ROS_serial_translator')
        self.get_logger().info('INITIALIZED.')

        # TODO - use fflag module once implemented instead of direct config param
        self.declare_parameter("ff_enable_v1_control_stack", False)
        # Enables INS packet forwarding + firmware debug state estimate topic publishing
        self.enable_v1_cstk = self.get_parameter("ff_enable_v1_control_stack").value

        self.declare_parameter("ff_enable_send_path_segments", False)
        # Enables forwarding path segments to firmware
        self.enable_path_segment_send = self.get_parameter("ff_enable_send_path_segments").value

        # Threading lock to protect the serial transmission lines
        self.tx_lock = Lock()

        # Callback Groups to allow parallel execution of orthogonal tasks
        # Timer loop and subscribers are in separate groups to prevent starvation issues since there is more overhead
        # in enforcing mutual exclusion at ros level than via basic lock

        # Subscribers
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()

        # Serial read loop
        self.read_cb_group = MutuallyExclusiveCallbackGroup()

        # Timer (write) loop
        self.misc_cb_group = MutuallyExclusiveCallbackGroup()

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

        # Assigned to the shared subscriber callback group
        self.create_subscription(
            StampedFloat64Msg, "input/steering", self.set_steering, 1,
            callback_group=self.sub_cb_group
        )
        self.create_subscription(
            Int8, "input/sanity_warning", self.set_alarm, 1,
            callback_group=self.sub_cb_group
        )

        # GPS relay to firmware. SC only: /ekf/odometry_earth is published by the
        # MicroStrain INS driver and only exists on Short Circuit.
        if self.self_name == "SC":
            # The INS reports position in ECEF; firmware wants UTM. Same transform
            # and zone as buggy_state_converter (EPSG:32617 is UTM Zone 17N).
            self.ecef_to_utm_transformer = pyproj.Transformer.from_crs(
                "epsg:4978", "epsg:32617", always_xy=True
            )
            self.gps_seq_num = 0

            if self.enable_v1_cstk:
                self.create_subscription(
                    Odometry, "/ekf/odometry_earth", self.send_raw_gps, 1,
                    callback_group=self.sub_cb_group
                )

                self.gps_SIV = 0
                self.gps_fix = 0
                self.rtk_fix = 0
                self.create_subscription(
                    MipGnssFixInfo,
                    "/gnss1/fix_info",
                    self.update_gnss_fix_info,
                    1,
                    callback_group=self.sub_cb_group,
                )

        if self.enable_path_segment_send:
            self.declare_parameter("trajectoryTopic", "self/cur_traj")
            # Arc length between the waypoints handed to the firmware, in meters. Also
            # sets the horizon, since one segment is one packet: the firmware is given
            # MAX_PATH_PTS_PER_MSG * spacing meters of path ahead of the buggy.
            self.declare_parameter("pathPointSpacing", 1.0)

            self.path_point_spacing = float(self.get_parameter("pathPointSpacing").value)

            # Mirror of the path the firmware holds, keyed by the same absolute point
            # index the firmware files it under. Used to skip resending points that
            # haven't changed; see send_path.
            self.fw_path_pts = {}

            if self.path_point_spacing <= 0.0:
                raise ValueError(
                    f"pathPointSpacing must be positive, got {self.path_point_spacing}"
                )

            self.create_subscription(
                TrajectoryMsg,
                self.get_parameter("trajectoryTopic").value,
                self.send_path,
                1,
                callback_group=self.sub_cb_group,
            )

        # High-frequency read loop, assigned to its own group so it never blocks writers
        self.timer = self.create_timer(0.001, self.loop, callback_group=self.read_cb_group)

        # Slower loop to send timestamp to teensy, at 10ms. Assigned to misc group.
        self.timestamp_timer = self.create_timer(0.01, self.send_timestamp, callback_group=self.misc_cb_group)




        # DEBUG MESSAGE PUBLISHERS:
        if self.self_name == "SC":
            self.sc_debug_info_publisher = self.create_publisher(SCDebugInfoMsg, "debug/firmware", 1)
            self.sc_sensor_publisher = self.create_publisher(SCSensorMsg, "debug/sensor", 1)

            # RADIO DATA PUBLISHER
            self.nand_radio_publisher = self.create_publisher(SCRadioNANDMsg, "debug/NAND_radio", 1)
            self.observed_nand_odom_publisher = self.create_publisher(Odometry, "NAND_raw_state", 1)

            self.sc_fw_ukf_publisher = self.create_publisher(Odometry, "debug/firmware_state", 1)

            # Firmware's Stanley controller state. Only published when the v1 control
            # stack is enabled, since nothing else runs Stanley on the teensy.
            self.stanley_debug_publisher = self.create_publisher(
                StanleyDebugMsg, "debug/firmware_stanley", 1
            )
        else:
            self.nand_debug_info_publisher = self.create_publisher(NANDDebugInfoMsg, "debug/firmware", 1)
            self.nand_raw_gps_publisher = self.create_publisher(NANDRawGPSMsg, "debug/raw_gps", 1)
            self.nand_ukf_odom_publisher = self.create_publisher(Odometry, "raw_state", 1)
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

        with self.tx_lock:
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

        with self.tx_lock:
            self.comms.send_steering(self.steer_angle, self.steer_fw_timestamp)

        sw_dt = (time.time_ns() - self.steer_sw_timestamp) * 1e-9
        self.control_latency_publisher.publish(Float64(data=sw_dt))

        self.get_logger().debug(f"Sent steering angle of: {self.steer_angle}")

    def loop(self):
        """
        Continuously drain the OS serial buffer until empty.
        Runs on its own thread and does not require the tx_lock.
        """
        while True:
            packet = self.comms.read_packet()

            if packet is None:
                # Buffer is empty, yield the thread until the next 1ms timer tick
                self.get_logger().debug("NO PACKET")
                break

            self.get_logger().debug("PACKET")

            if isinstance(packet, NANDDebugInfo):
                rospacket = NANDDebugInfoMsg()
                rospacket.heading_rate = packet.heading_rate
                rospacket.encoder_speed = packet.encoder_speed
                rospacket.rc_steering_angle = packet.rc_steering_angle
                rospacket.software_steering_angle = packet.software_steering_angle
                rospacket.true_steering_angle = packet.true_steering_angle
                rospacket.rfm69_timeout_num = packet.rfm69_timeout_num
                rospacket.encoder_last_packet = packet.encoder_last_packet
                rospacket.operator_ready = packet.operator_ready
                rospacket.brake_status = packet.brake_status
                rospacket.auton_steer = packet.auton_steer
                rospacket.tx12_state = packet.tx12_state
                rospacket.stepper_alarm = packet.stepper_alarm
                rospacket.encoder_error = packet.encoder_error
                rospacket.rc_uplink_quality = packet.rc_uplink_quality
                self.nand_debug_info_publisher.publish(rospacket)

                self.get_logger().debug(f'NAND Debug Timestamp: {packet.timestamp}')

            elif isinstance(packet, NANDUKF):
                odom = Odometry()
                odom.pose.pose.position.x = packet.easting
                odom.pose.pose.position.y = packet.northing
                odom.pose.pose.orientation.z = packet.theta

                # Updated the eastern, northing, and heading (yaw) index of variance
                # Important note: covariance operates on (x, y, z, pitch, roll, yaw)
                odom.pose.covariance = np.diag([packet.eastern_cov, packet.northern_cov, 0, 0, 0, packet.heading_cov]).reshape(-1).tolist()

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
                rospacket.gps_seqnum = packet.gps_seqnum
                rospacket.timestamp = packet.timestamp
                rospacket.gps_siv = packet.gps_SIV
                rospacket.gps_fix = packet.gps_fix
                rospacket.rtk_fix = packet.rtk_fix
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

                rospacket = SCRadioNANDMsg()
                rospacket.easting = packet.nand_east_gps
                rospacket.northing = packet.nand_north_gps
                rospacket.gps_seqnum = packet.gps_seqnum
                rospacket.rx_rssi = packet.rx_rssi
                rospacket.gps_fix = packet.nand_gps_fix
                rospacket.auton_steer = packet.nand_auton
                self.nand_radio_publisher.publish(rospacket)

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

            elif isinstance(packet, StanleyDebug) and self.enable_v1_cstk:
                rospacket = StanleyDebugMsg()
                rospacket.steering_command = packet.steering_command
                rospacket.error_pose_x = packet.error_pose_x
                rospacket.error_pose_y = packet.error_pose_y
                rospacket.reference_navsat_easting = packet.reference_navsat_easting
                rospacket.reference_navsat_northing = packet.reference_navsat_northing
                rospacket.error_dist = packet.error_dist
                rospacket.error_heading = packet.error_heading
                rospacket.yaw_rate_error = packet.yaw_rate_error
                self.stanley_debug_publisher.publish(rospacket)
                self.get_logger().debug(
                    f'Stanley Debug: steer {packet.steering_command:.4f}, '
                    f'xtrack {packet.error_pose_y:.3f} m'
                )

            elif isinstance(packet, SCUKF) and self.enable_v1_cstk:
                odom = Odometry()
                odom.pose.pose.position.x = packet.eastern
                odom.pose.pose.position.y = packet.northern
                odom.pose.pose.orientation.z = packet.heading

                # Updated the eastern, northing, and heading (yaw) index of variance
                # Important note: covariance operates on (x, y, z, pitch, roll, yaw)
                odom.pose.covariance = np.diag([packet.eastern_cov, packet.northern_cov, 0, 0, 0, packet.heading_cov]).reshape(-1).tolist()

                odom.twist.twist.linear.x = packet.front_speed
                odom.twist.twist.angular.z = packet.heading_rate

                odom.header.frame_id = str(packet.timestamp)

                self.sc_fw_ukf_publisher.publish(odom)
                self.get_logger().debug(f'SC UKF Timestamp: {packet.timestamp}')

    def _path_point_matches(self, new_pt, fw_pt):
        """
        Whether the firmware's copy of a path point is close enough to a freshly
        sampled one that it doesn't need to be resent.
        """
        if fw_pt is None:
            return False

        heading_err = (
            new_pt.heading_tangent - fw_pt.heading_tangent + np.pi
        ) % (2 * np.pi) - np.pi

        return (
            np.hypot(new_pt.x - fw_pt.x, new_pt.y - fw_pt.y) <= self.PATH_POS_TOL
            and abs(heading_err) <= self.PATH_HEADING_TOL
            and abs(new_pt.curvature - fw_pt.curvature) <= self.PATH_CURVATURE_TOL
        )

    def send_path(self, msg: TrajectoryMsg):
        """
        Resamples the planner's local trajectory at a fixed arc length spacing and
        forwards the points the firmware doesn't already have as a single PathMsg.

        Runs on every trajectory message, so the path is refreshed at the planner's
        10 Hz. One segment is one packet: at most MAX_PATH_PTS_PER_MSG points are
        sampled, which is the most that fit in a payload, so the firmware is handed
        that many points of path ahead of the buggy and the rest of the planner's
        window is dropped. At 10 Hz the buggy re-reads the far part of the plan many
        times over before it arrives there.

        Points are sampled on a global grid of multiples of the spacing, measured
        from the start of the nominal course, so index * dist is a point's arc length
        along that course and a given index names the same point from one tick to the
        next. The planner's window starts wherever the buggy is, which is generally
        mid-interval, so the first sample is taken at the first grid point at or after
        msg.start_dist rather than at the window start itself.

        self.fw_path_pts mirrors what the firmware holds. A tick usually just slides
        the window forward by a point or so and nudges the passing offset, so the
        leading run of points that still match the mirror is dropped and the segment
        sent starts at the first index that actually changed. Everything from there to
        the end of the horizon goes out, since the firmware overwrites in place and
        must never be left holding the tail of a superseded plan. In the steady state
        that is the one newly revealed point per tick; nothing is sent at all while
        the buggy sits still.

        Note that index i sits i * dist along the *driven* path, not along the nominal
        course. The planner offsets the path sideways to pass, which lengthens it on a
        curve, so index-to-course-position drifts toward the tail by roughly
        offset * (horizon span / turn radius) -- about 0.9 m at the end of a 31 m
        horizon for a 3 m offset on a 100 m radius turn, and proportionally less for a
        shorter horizon. Spacing follows the driven path because that is the path the
        firmware integrates along, per the packet spec.
        """
        # A cubic spline fit needs at least a couple of points to interpolate
        if len(msg.easting) < 2:
            self.get_logger().warn(f"Ignoring trajectory with {len(msg.easting)} points")
            return

        traj, _ = Trajectory.unpack(msg)
        spacing = self.path_point_spacing

        # First grid index at or after the window start, and how far into the window
        # that index falls. Sampling from the window start instead would place every
        # index a different fraction of an interval off the grid on each tick, which
        # breaks both idx * dist == arc length and the match against the mirror.
        start_idx = int(np.ceil(msg.start_dist / spacing - self.PATH_GRID_EPS))
        first_offset = max(start_idx * spacing - msg.start_dist, 0.0)

        # traj.distances is cumulative arc length, so the last entry is the window length
        window_len = float(traj.distances[-1])
        if first_offset > window_len + self.PATH_GRID_EPS:
            self.get_logger().warn(
                f"No {spacing} m grid point falls inside the {window_len:.2f} m "
                f"trajectory window starting at {msg.start_dist:.2f} m"
            )
            return
        num_pts = int((window_len - first_offset) / spacing + self.PATH_GRID_EPS) + 1

        # Capped so the segment always fits one payload, which keeps a tick's path
        # update atomic: the firmware either has the whole new segment or none of it,
        # with no partial plan in between if a packet is dropped.
        num_pts = min(num_pts, MAX_PATH_PTS_PER_MSG)

        # Equal arc length steps along the window. These accessors are vectorized, so
        # this is three spline evaluations rather than three per point.
        dists = first_offset + np.arange(num_pts) * spacing
        positions = traj.get_position_by_distance(dists)
        headings = traj.get_heading_by_distance(dists)
        curvatures = traj.get_curvature_by_distance(dists)

        pts = [
            PathPoint(
                x=float(positions[i, 0]),
                y=float(positions[i, 1]),
                heading_tangent=float(headings[i]),
                curvature=float(curvatures[i]),
            )
            for i in range(num_pts)
        ]

        # Forget indices far enough behind the window that no future window can match
        # against them, so the mirror stays bounded to roughly one horizon.
        self.fw_path_pts = {
            idx: pt
            for idx, pt in self.fw_path_pts.items()
            if idx >= start_idx - self.PATH_STORE_MARGIN_PTS
        }

        # Truncate the leading points the firmware already has, stopping at the first
        # one that differs
        unchanged = 0
        while unchanged < num_pts and self._path_point_matches(
            pts[unchanged], self.fw_path_pts.get(start_idx + unchanged)
        ):
            unchanged += 1

        if unchanged == num_pts:
            self.get_logger().debug(
                f"Firmware already has all {num_pts} path pts from idx {start_idx}"
            )
            return

        pts = pts[unchanged:]
        start_idx += unchanged

        packet = PathMsg(
            num_pts=len(pts),
            start_idx=start_idx,
            dist=spacing,
            pts=pts,
        )
        with self.tx_lock:
            self.comms.send_path(packet)

        # Only the points actually sent are updated, so one left in place keeps the
        # value the firmware really has rather than the fresh sample that happened to
        # land within tolerance of it. Otherwise a run of within-tolerance nudges
        # would walk the two copies arbitrarily far apart without ever resending.
        self.fw_path_pts.update({start_idx + i: pt for i, pt in enumerate(pts)})

        self.get_logger().debug(
            f"Sent {len(pts)} path pts from idx {start_idx} at {spacing} m spacing "
            f"({unchanged} unchanged, {window_len:.1f} m window)"
        )

    def send_raw_gps(self, msg: Odometry):
        """
        Relays the INS EKF solution to the firmware as an SCRawGPS packet.

        /ekf/odometry_earth carries ECEF position, so it is converted to UTM
        easting/northing before being sent.
        """
        easting, northing, _ = self.ecef_to_utm_transformer.transform(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

        # 2D accuracy from the position covariance, which is ordered (x, y, z, r, p, y),
        # so index 0 is the x variance and index 7 the y variance.
        accuracy = float(np.sqrt(msg.pose.covariance[0] + msg.pose.covariance[7]))

        # Firmware keeps a millis() clock, so a wrapping 32-bit millisecond stamp matches it
        stamp_ms = (msg.header.stamp.sec * 1000) + (msg.header.stamp.nanosec // 1_000_000)

        packet = SCRawGPS(
            eastern=float(easting),
            northern=float(northing),
            accuracy=accuracy,
            gps_seq_num=self.gps_seq_num,
            timestamp=stamp_ms & 0xFFFFFFFF,
            gps_SIV=self.gps_SIV,
            gps_fix=self.gps_fix,  # claim a 3D fix so firmware does not discard the packet
            rtk_fix=self.rtk_fix,
        )

        self.gps_seq_num = (self.gps_seq_num + 1) & 0xFFFFFFFF

        with self.tx_lock:
            self.comms.send_gps(packet)

        self.get_logger().debug(
            f"Sent SCRawGPS seq={packet.gps_seq_num} "
            f"easting={easting:.2f} northing={northing:.2f} accuracy={accuracy:.2f}"
        )

    def send_timestamp(self):
        with self.tx_lock:
            self.comms.send_timestamp(time.time_ns())

    def update_gnss_fix_info(self, msg: MipGnssFixInfo):
        # This function is called by the subscription, constantly
        # updating the local values gps_SIV, gps_fix, and rtk_fix.

        self.gps_SIV = msg.num_sv


        # 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix

        microstrain_to_gps = {
            0 : 3,
            1 : 2,
            2 : 5,
            3 : 0,
            4 : 0,
            5 : 4,
            6 : 4,
        }

        self.gps_fix = microstrain_to_gps.get(msg.fix_type)

        # Firmware RTK codes: 0 = none, 1 = float, 2 = fixed.
        if msg.fix_type == MipGnssFixInfo.FIX_TYPE_FIX_RTK_FLOAT: # fix_type 5
            self.rtk_fix = 1
        elif msg.fix_type == MipGnssFixInfo.FIX_TYPE_FIX_RTK_FIXED: # fix_type 6
            self.rtk_fix = 2
        else:
            self.rtk_fix = 0


def main(args=None):
    rclpy.init(args=args)

    translator = Translator()

    # Initialize MultiThreadedExecutor to allow callback groups to run concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(translator)

    executor.spin()
    translator.destroy_node()
    rclpy.try_shutdown()

if __name__ == "__main__":
    main()