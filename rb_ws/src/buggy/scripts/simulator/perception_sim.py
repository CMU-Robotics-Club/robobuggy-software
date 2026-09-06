#!/usr/bin/env python3
"""
perception_sim.py
-----------------
Fake lidar / camera detections of the other buggy for the 2D simulator, so the
multi-source NAND estimator and the planner can be exercised without hardware.

Reads the ground-truth state of the observed buggy and of the observer, and
publishes a noisy UTM position as nav_msgs/Odometry with a filled position
covariance, on the same topic the real sensor node would use:

    kind: "lidar"  -> lidar/other/state   (range-limited, low noise, occasional dropouts)
    kind: "camera" -> vision/other/state  (forward field of view, noise grows with range, latency)

Parameters let you inject the failure modes the research flagged: dropout
probability, added latency, a range cap, and an outlier probability that emits
a wildly wrong position (to check the estimator's chi-square gate).

Example (SC namespace):
    ros2 run buggy perception_sim.py --ros-args -r __ns:=/SC -p kind:=lidar
"""

import random
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class PerceptionSim(Node):
    def __init__(self):
        super().__init__("perception_sim")
        self.declare_parameter("kind", "lidar")
        self.declare_parameter("observer_topic", "self/state")
        self.declare_parameter("target_topic", "/NAND/self/state")
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

        topic = "lidar/other/state" if self.kind == "lidar" else "vision/other/state"
        self.publisher = self.create_publisher(Odometry, topic, 1)
        self.observer = None
        self.target = None
        self.queue = deque()  # (release_time, msg) for latency injection

        self.create_subscription(Odometry, p("observer_topic"), self.on_observer, 1)
        self.create_subscription(Odometry, p("target_topic"), self.on_target, 1)
        self.create_timer(1.0 / rate, self.sample)
        self.create_timer(0.01, self.flush)
        self.get_logger().info(f"perception_sim kind={self.kind} -> {topic}")

    def on_observer(self, msg):
        self.observer = msg

    def on_target(self, msg):
        self.target = msg

    def sample(self):
        if self.observer is None or self.target is None:
            return
        ox, oy = self.observer.pose.pose.position.x, self.observer.pose.pose.position.y
        oh = self.observer.pose.pose.orientation.z
        tx, ty = self.target.pose.pose.position.x, self.target.pose.pose.position.y

        dx, dy = tx - ox, ty - oy
        rng = float(np.hypot(dx, dy))
        if rng > self.max_range:
            return
        bearing = np.arctan2(dy, dx) - oh
        bearing = (bearing + np.pi) % (2 * np.pi) - np.pi
        if abs(bearing) > self.fov / 2.0:
            return
        if random.random() < self.dropout:
            return

        std = self.base_std + self.std_per_m * rng
        mx = tx + np.random.normal(0.0, std)
        my = ty + np.random.normal(0.0, std)
        if self.outlier > 0.0 and random.random() < self.outlier:
            ang = random.random() * 2 * np.pi
            mx += self.outlier_offset * np.cos(ang)
            my += self.outlier_offset * np.sin(ang)

        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "utm"
        out.pose.pose.position.x = float(mx)
        out.pose.pose.position.y = float(my)
        cov = [0.0] * 36
        cov[0] = std * std
        cov[7] = std * std
        out.pose.covariance = cov

        release = self.get_clock().now().nanoseconds * 1e-9 + self.latency
        self.queue.append((release, out))

    def flush(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.queue and self.queue[0][0] <= now:
            _, msg = self.queue.popleft()
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
