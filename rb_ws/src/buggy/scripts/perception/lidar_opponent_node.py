#!/usr/bin/env python3
"""
lidar_opponent_node.py
----------------------
Turns lidar clusters (sensor frame, metres) into UTM detections for the tracker
(DECISIONS.md D4, D6).

Inputs
    /lidar/detections_sensor   buggy/DetectionArrayMsg from buggy_lidar.py: EVERY cluster that
                               survived the ellipse filter, centroid and extent in the lidar frame
    /lidar/obstacle_centroid   geometry_msgs/PointStamped, the legacy single best cluster (used
                               only when no array arrives, for older lidar-branch builds)
    self/state                 nav_msgs/Odometry, ego pose in UTM, heading in orientation.z

Outputs
    lidar/detection_array      buggy/DetectionArrayMsg in UTM, capture-time stamped, every cluster
    lidar/other/state          legacy nav_msgs/Odometry of the nearest cluster (nand_estimator.py)
    lidar/detections           legacy buggy/DetectionsMsg

Frames and time: the ego pose is interpolated at the scan's capture stamp from a
short history of self/state, so a 100 ms old scan is not placed with the pose of
now (1.2 m at 12 m/s). The sensor-to-buggy extrinsics are PARAMETERS with a
provenance string; they are not measured yet. The lidar branch's own code treats
negative X as forward (lidar_helpers.py), so the default `sensor_forward_axis` is
"neg_x"; set it from a real calibration, not from this comment. Scans are not
motion compensated; the message says so.
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

from racing.health import stamp_seconds

from buggy.msg import DetectionArrayMsg, DetectionMsg, DetectionsMsg


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class LidarOpponent(Node):
    def __init__(self):
        super().__init__("lidar_opponent")
        self.declare_parameter("centroid_topic", "/lidar/obstacle_centroid")
        self.declare_parameter("cluster_topic", "/lidar/detections_sensor")
        self.declare_parameter("lidar_x", 0.9)             # metres forward of the INS reference point
        self.declare_parameter("lidar_y", 0.0)             # metres left of it
        self.declare_parameter("lidar_yaw_deg", 0.0)       # sensor yaw relative to the buggy forward axis
        self.declare_parameter("sensor_forward_axis", "neg_x")   # "x" or "neg_x" (lidar branch convention)
        self.declare_parameter("extrinsics_status", "assumed")   # assumed | measured:<date/method>
        self.declare_parameter("base_std_m", 0.25)
        self.declare_parameter("std_per_m", 0.02)
        self.declare_parameter("max_range_m", 40.0)
        self.declare_parameter("pose_history_s", 1.0)
        self.declare_parameter("max_pose_extrapolation_s", 0.1)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.offset = np.array([float(p("lidar_x")), float(p("lidar_y"))])
        self.yaw = math.radians(float(p("lidar_yaw_deg")))
        self.flip = str(p("sensor_forward_axis")).lower() == "neg_x"
        self.extrinsics_status = str(p("extrinsics_status"))
        self.base_std = float(p("base_std_m"))
        self.std_per_m = float(p("std_per_m"))
        self.max_range = float(p("max_range_m"))
        self.history_s = float(p("pose_history_s"))
        self.max_extrap = float(p("max_pose_extrapolation_s"))
        if self.extrinsics_status == "assumed":
            self.get_logger().warn(
                f"lidar extrinsics are ASSUMED (x {self.offset[0]:.2f} m, y {self.offset[1]:.2f} m, yaw "
                f"{math.degrees(self.yaw):.1f} deg, forward axis {'-X' if self.flip else '+X'}); "
                "measure them before trusting detection positions"
            )

        self.poses = deque()   # (t, x, y, heading)
        self.have_array = False
        self.create_subscription(Odometry, "self/state", self.on_state, 5)
        self.create_subscription(DetectionArrayMsg, p("cluster_topic"), self.on_clusters, 10)
        self.create_subscription(PointStamped, p("centroid_topic"), self.on_centroid, 10)
        self.array_pub = self.create_publisher(DetectionArrayMsg, "lidar/detection_array", 1)
        self.legacy_pub = self.create_publisher(Odometry, "lidar/other/state", 1)
        self.legacy_det_pub = self.create_publisher(DetectionsMsg, "lidar/detections", 1)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ ego pose history
    def on_state(self, msg):
        t = stamp_seconds(msg.header.stamp)
        if t <= 0:
            t = self.now_s()
        self.poses.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z))
        while self.poses and t - self.poses[0][0] > self.history_s:
            self.poses.popleft()

    def pose_at(self, t):
        """Ego (x, y, heading) interpolated at time t, or None when the history cannot cover it."""
        if not self.poses:
            return None
        ts = [q[0] for q in self.poses]
        if t < ts[0] - self.max_extrap or t > ts[-1] + self.max_extrap:
            return None
        if len(ts) == 1 or t <= ts[0]:
            return self.poses[0][1:]
        if t >= ts[-1]:
            return self.poses[-1][1:]
        i = int(np.searchsorted(ts, t))
        t0, x0, y0, h0 = self.poses[i - 1]
        t1, x1, y1, h1 = self.poses[i]
        a = (t - t0) / max(t1 - t0, 1e-6)
        return (x0 + a * (x1 - x0), y0 + a * (y1 - y0), h0 + a * wrap(h1 - h0))

    # ------------------------------------------------------------------ transforms
    def sensor_to_utm(self, sx, sy, pose):
        ex, ey, heading = pose
        if self.flip:
            sx, sy = -sx, -sy
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        bx = c * sx - s * sy + self.offset[0]
        by = s * sx + c * sy + self.offset[1]
        ch, sh = math.cos(heading), math.sin(heading)
        return ex + ch * bx - sh * by, ey + sh * bx + ch * by

    def publish_all(self, stamp, items):
        """items: list of (sensor_x, sensor_y, extent xyz or None, confidence)."""
        t = stamp_seconds(stamp)
        if t <= 0:
            t = self.now_s()
        pose = self.pose_at(t)
        if pose is None:
            self.get_logger().warn("no ego pose covering the scan time; dropping lidar detections",
                                   throttle_duration_sec=2.0)
            return
        arr = DetectionArrayMsg()
        arr.header.stamp = stamp
        arr.header.frame_id = "utm"
        arr.source = "lidar"
        arr.motion_compensated = False
        arr.source_age_unknown = False
        legacy = DetectionsMsg()
        legacy.header = arr.header
        legacy.source = "lidar"
        nearest = None
        for sx, sy, extent, confidence in items:
            rng = math.hypot(sx, sy)
            if rng > self.max_range or rng < 0.3:
                continue
            ux, uy = self.sensor_to_utm(sx, sy, pose)
            std = self.base_std + self.std_per_m * rng
            det = DetectionMsg()
            det.position.x, det.position.y = float(ux), float(uy)
            det.position_covariance = [std * std, 0.0, 0.0, 0.0, std * std, 0.0, 0.0, 0.0, 0.0]
            if extent is not None and all(math.isfinite(v) and v > 0 for v in extent):
                det.extent.x, det.extent.y, det.extent.z = [float(v) for v in extent]
                det.extent_known = True
            det.class_id = "cluster"
            det.confidence = float(confidence)
            det.observed = True
            arr.detections.append(det)
            legacy.easting.append(float(ux))
            legacy.northing.append(float(uy))
            legacy.pos_std.append(float(std))
            if nearest is None or rng < nearest[0]:
                nearest = (rng, ux, uy, std)
        self.array_pub.publish(arr)
        self.legacy_det_pub.publish(legacy)
        if nearest is not None:
            _, ux, uy, std = nearest
            out = Odometry()
            out.header = arr.header
            out.pose.pose.position.x = float(ux)
            out.pose.pose.position.y = float(uy)
            cov = [0.0] * 36
            cov[0] = std * std
            cov[7] = std * std
            out.pose.covariance = cov
            self.legacy_pub.publish(out)

    # ------------------------------------------------------------------ inputs
    def on_clusters(self, msg: DetectionArrayMsg):
        self.have_array = True
        items = [(d.position.x, d.position.y,
                  (d.extent.x, d.extent.y, d.extent.z) if d.extent_known else None, d.confidence)
                 for d in msg.detections if d.observed]
        self.publish_all(msg.header.stamp, items)

    def on_centroid(self, msg: PointStamped):
        if self.have_array:
            return  # the all-cluster array already covers this scan
        self.publish_all(msg.header.stamp, [(msg.point.x, msg.point.y, None, 1.0)])


def main(args=None):
    rclpy.init(args=args)
    node = LidarOpponent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
