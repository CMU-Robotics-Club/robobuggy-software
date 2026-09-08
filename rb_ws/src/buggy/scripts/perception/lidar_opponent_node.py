#!/usr/bin/env python3
"""
lidar_opponent_node.py
----------------------
Turns the lidar pipeline's obstacle centroid (lidar frame, metres) into a UTM
position of the other buggy that the NAND estimator can fuse.

Input
    /lidar/obstacle_centroid   geometry_msgs/PointStamped, from buggy_lidar.py on
                               the lidar branch: x forward, y left, z up in the
                               lidar frame (set lidar_yaw if the sensor is rotated)
    self/state                 nav_msgs/Odometry, ego pose in UTM with heading in
                               orientation.z (team convention)
Output
    lidar/other/state          nav_msgs/Odometry, opponent position in UTM with a
                               2x2 position covariance that grows with range, so
                               the estimator trusts near detections more

Mount extrinsics are parameters: lidar_x / lidar_y are the sensor's offset from
the point the INS reports (rear axle centre), in the buggy frame.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

from buggy.msg import DetectionsMsg


class LidarOpponent(Node):
    def __init__(self):
        super().__init__("lidar_opponent")
        self.declare_parameter("centroid_topic", "/lidar/obstacle_centroid")
        self.declare_parameter("lidar_x", 0.9)          # metres forward of the INS reference point
        self.declare_parameter("lidar_y", 0.0)          # metres left of it
        self.declare_parameter("lidar_yaw_deg", 0.0)    # sensor yaw relative to the buggy's forward axis
        self.declare_parameter("base_std_m", 0.25)      # position std at zero range
        self.declare_parameter("std_per_m", 0.02)       # added std per metre of range
        self.declare_parameter("max_range_m", 40.0)
        self.declare_parameter("max_state_age_s", 0.2)  # ignore detections when ego pose is stale

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.offset = np.array([float(p("lidar_x")), float(p("lidar_y"))])
        self.yaw = np.deg2rad(float(p("lidar_yaw_deg")))
        self.base_std = float(p("base_std_m"))
        self.std_per_m = float(p("std_per_m"))
        self.max_range = float(p("max_range_m"))
        self.max_age = float(p("max_state_age_s"))

        self.state = None
        self.state_time = None
        self.create_subscription(Odometry, "self/state", self.on_state, 1)
        self.create_subscription(PointStamped, p("centroid_topic"), self.on_centroid, 10)
        self.publisher = self.create_publisher(Odometry, "lidar/other/state", 1)
        self.det_publisher = self.create_publisher(DetectionsMsg, "lidar/detections", 1)

    def on_state(self, msg):
        self.state = msg
        self.state_time = self.get_clock().now()

    def on_centroid(self, msg: PointStamped):
        if self.state is None:
            return
        age = (self.get_clock().now() - self.state_time).nanoseconds * 1e-9
        if age > self.max_age:
            self.get_logger().warn("ego state stale, dropping lidar detection", throttle_duration_sec=2.0)
            return

        # lidar frame -> buggy frame (rotate by sensor yaw, translate by mount offset)
        pl = np.array([msg.point.x, msg.point.y])
        rng = float(np.hypot(pl[0], pl[1]))
        if rng > self.max_range or rng < 0.3:
            return
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        pb = np.array([c * pl[0] - s * pl[1], s * pl[0] + c * pl[1]]) + self.offset

        # buggy frame -> UTM (rotate by heading, translate by ego position)
        h = self.state.pose.pose.orientation.z
        ch, sh = np.cos(h), np.sin(h)
        ex, ey = self.state.pose.pose.position.x, self.state.pose.pose.position.y
        ux = ex + ch * pb[0] - sh * pb[1]
        uy = ey + sh * pb[0] + ch * pb[1]

        std = self.base_std + self.std_per_m * rng
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "utm"
        out.pose.pose.position.x = float(ux)
        out.pose.pose.position.y = float(uy)
        cov = [0.0] * 36
        cov[0] = std * std
        cov[7] = std * std
        out.pose.covariance = cov
        self.publisher.publish(out)
        det = DetectionsMsg()
        det.header = out.header
        det.source = "lidar"
        det.easting = [float(ux)]
        det.northing = [float(uy)]
        det.pos_std = [float(std)]
        self.det_publisher.publish(det)


def main(args=None):
    rclpy.init(args=args)
    node = LidarOpponent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
