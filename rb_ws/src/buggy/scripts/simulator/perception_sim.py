#!/usr/bin/env python3
"""
perception_sim.py
-----------------
Fake lidar / camera for the 2D simulator: publishes noisy detections of every
buggy the sensor could see, so the tracker, the fused NAND estimator and the
planner can be exercised in traffic without hardware.

Targets come from two places:
  * real simulated buggies: the Odometry topics in `target_topics`
    (default just /NAND/self/state)
  * ghost buggies: `ghosts`, a list of strings "s0:<m>,v:<m/s>,d:<m>" that move
    along the reference track at constant speed and lateral offset. Ghosts stand
    in for other teams' buggies that send no radio and only exist to the sensors.

Per target, per cycle: check range and field of view relative to the observer,
maybe drop it (dropout_prob), add range-dependent noise, maybe replace it by a
wild outlier (outlier_prob), delay it (latency_s). Everything visible goes into
one buggy/DetectionsMsg on lidar/detections or vision/detections. For backward
compatibility the first real target is also published as a single Odometry on
lidar/other/state or vision/other/state (what nand_estimator.py consumes).

Ghost ground truth is published on perception_sim/ghost_truth (PoseArray) so
pass_metrics.py can score clearance against them.

Example (SC namespace):
    ros2 run buggy perception_sim.py --ros-args -r __ns:=/SC -p kind:=lidar \
        -p ghosts:="['s0:70,v:9.0,d:-0.3']"
"""

import os
import random
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry

from buggy.msg import DetectionsMsg
from util.track import Track


def parse_ghost(spec):
    out = {"s0": 50.0, "v": 9.0, "d": 0.0}
    for part in str(spec).split(","):
        if ":" in part:
            k, v = part.split(":", 1)
            out[k.strip()] = float(v)
    return out


class PerceptionSim(Node):
    def __init__(self):
        super().__init__("perception_sim")
        self.declare_parameter("kind", "lidar")
        self.declare_parameter("observer_topic", "self/state")
        self.declare_parameter("target_topics", ["/NAND/self/state"])
        self.declare_parameter("ghosts", [""])
        self.declare_parameter("traj_name", "buggycourse_sc.json")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("max_range_m", 40.0)
        self.declare_parameter("fov_deg", 360.0)          # camera: e.g. 100; lidar: 360
        self.declare_parameter("base_std_m", 0.25)
        self.declare_parameter("std_per_m", 0.02)
        self.declare_parameter("dropout_prob", 0.1)
        self.declare_parameter("outlier_prob", 0.0)
        self.declare_parameter("outlier_offset_m", 15.0)
        self.declare_parameter("latency_s", 0.0)
        self.declare_parameter("seed", 0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.kind = str(p("kind"))
        self.max_range = float(p("max_range_m"))
        self.fov = np.deg2rad(float(p("fov_deg")))
        self.base_std = float(p("base_std_m"))
        self.std_per_m = float(p("std_per_m"))
        self.dropout = float(p("dropout_prob"))
        self.outlier = float(p("outlier_prob"))
        self.outlier_offset = float(p("outlier_offset_m"))
        self.latency = float(p("latency_s"))
        rate = float(p("rate_hz"))
        seed = int(p("seed"))
        if seed:
            random.seed(seed)
            np.random.seed(seed)

        self.ghosts = [parse_ghost(g) for g in p("ghosts") if str(g).strip()]
        self.track = None
        if self.ghosts:
            self.track = Track.from_files(os.environ["TRAJPATH"] + p("traj_name"), ds=1.0)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9

        base = "lidar" if self.kind == "lidar" else "vision"
        self.det_pub = self.create_publisher(DetectionsMsg, f"{base}/detections", 1)
        self.legacy_pub = self.create_publisher(Odometry, f"{base}/other/state", 1)
        self.truth_pub = self.create_publisher(PoseArray, "perception_sim/ghost_truth", 1)

        self.observer = None
        self.targets = {}
        self.target_order = list(p("target_topics"))
        for topic in self.target_order:
            self.create_subscription(Odometry, topic, lambda m, t=topic: self.on_target(t, m), 1)
        self.create_subscription(Odometry, p("observer_topic"), self.on_observer, 1)

        self.queue = deque()  # (release_time, det_msg, legacy_msg or None)
        self.create_timer(1.0 / rate, self.sample)
        self.create_timer(0.01, self.flush)
        self.get_logger().info(
            f"perception_sim kind={self.kind} targets={self.target_order} ghosts={len(self.ghosts)} -> {base}/detections"
        )

    def on_observer(self, msg):
        self.observer = msg

    def on_target(self, topic, msg):
        self.targets[topic] = msg

    # ------------------------------------------------------------------ truth
    def ghost_positions(self):
        """Ghost UTM positions at the current time."""
        if not self.ghosts:
            return []
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        out = []
        for g in self.ghosts:
            s = g["s0"] + g["v"] * t
            if s >= self.track.length - 1.0:
                continue
            x, y = self.track.cartesian(s, g["d"])[0]
            out.append((float(x), float(y)))
        return out

    def real_positions(self):
        out = []
        for topic in self.target_order:
            m = self.targets.get(topic)
            if m is not None:
                out.append((m.pose.pose.position.x, m.pose.pose.position.y))
            else:
                out.append(None)
        return out

    # ------------------------------------------------------------------ sensing
    def sample(self):
        # ghost truth for the metrics harness, whether or not anything is visible
        ghosts = self.ghost_positions()
        truth = PoseArray()
        truth.header.stamp = self.get_clock().now().to_msg()
        truth.header.frame_id = "utm"
        for gx, gy in ghosts:
            pose = Pose()
            pose.position.x, pose.position.y = gx, gy
            truth.poses.append(pose)
        self.truth_pub.publish(truth)

        if self.observer is None:
            return
        ox, oy = self.observer.pose.pose.position.x, self.observer.pose.pose.position.y
        oh = self.observer.pose.pose.orientation.z

        det = DetectionsMsg()
        det.header.stamp = self.get_clock().now().to_msg()
        det.header.frame_id = "utm"
        det.source = self.kind
        legacy = None

        candidates = [(pos, idx == 0) for idx, pos in enumerate(self.real_positions()) if pos is not None]
        candidates += [(g, False) for g in ghosts]

        for (tx, ty), is_primary in candidates:
            dx, dy = tx - ox, ty - oy
            rng = float(np.hypot(dx, dy))
            if rng > self.max_range or rng < 0.5:
                continue
            bearing = (np.arctan2(dy, dx) - oh + np.pi) % (2 * np.pi) - np.pi
            if abs(bearing) > self.fov / 2.0:
                continue
            if random.random() < self.dropout:
                continue
            std = self.base_std + self.std_per_m * rng
            mx = tx + np.random.normal(0.0, std)
            my = ty + np.random.normal(0.0, std)
            if self.outlier > 0.0 and random.random() < self.outlier:
                ang = random.random() * 2 * np.pi
                mx += self.outlier_offset * np.cos(ang)
                my += self.outlier_offset * np.sin(ang)
            det.easting.append(float(mx))
            det.northing.append(float(my))
            det.pos_std.append(float(std))
            if is_primary:
                legacy = Odometry()
                legacy.header = det.header
                legacy.pose.pose.position.x = float(mx)
                legacy.pose.pose.position.y = float(my)
                cov = [0.0] * 36
                cov[0] = std * std
                cov[7] = std * std
                legacy.pose.covariance = cov

        release = self.get_clock().now().nanoseconds * 1e-9 + self.latency
        self.queue.append((release, det, legacy))

    def flush(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.queue and self.queue[0][0] <= now:
            _, det, legacy = self.queue.popleft()
            self.det_pub.publish(det)
            if legacy is not None:
                self.legacy_pub.publish(legacy)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
