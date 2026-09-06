#!/usr/bin/env python3
"""
sim_metrics.py
--------------
Records tracking quality while a launch is running and prints a JSON summary,
so two trajectories or two planner settings can be compared with numbers.

Run alongside a sim launch, in the buggy's namespace:
    ros2 run buggy sim_metrics.py --ros-args -r __ns:=/SC -p duration:=90.0
or, from anywhere:
    ros2 run buggy sim_metrics.py --ros-args -r __ns:=/SC -p duration:=90.0 -p out:=/tmp/metrics.json

Metrics:
    distance_m           how far the buggy travelled along its own track
    xte_rms_m, xte_max_m cross-track error of the front axle (from the controller)
    steer_rms_deg, steer_max_deg      commanded steering
    steer_rate_rms_dps   how frantic the steering is (deg per second, RMS)
    heading_change_per_m mean |d heading| per metre travelled: a curvature proxy
    speed_mean_mps       from the state message
Lower is better for everything except distance_m.
"""

import json
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from buggy.msg import StampedFloat64Msg


class SimMetrics(Node):
    def __init__(self):
        super().__init__("sim_metrics")
        self.declare_parameter("duration", 60.0)
        self.declare_parameter("out", "")
        self.declare_parameter("xte_topic", "controller/controller/debug/cross_track_error")
        self.declare_parameter("steer_topic", "input/steering")
        self.declare_parameter("state_topic", "self/state")

        self.duration = float(self.get_parameter("duration").value)
        self.out = str(self.get_parameter("out").value)

        self.create_subscription(Odometry, self.get_parameter("state_topic").value, self.on_state, 10)
        self.create_subscription(Float64, self.get_parameter("xte_topic").value, self.on_xte, 10)
        self.create_subscription(StampedFloat64Msg, self.get_parameter("steer_topic").value, self.on_steer, 10)

        self.last_xy = None
        self.last_heading = None
        self.distance = 0.0
        self.heading_change = 0.0
        self.speeds = []
        self.xte = []
        self.steer = []
        self.steer_t = []

        self.t0 = self.get_clock().now()
        self.create_timer(0.5, self.check_done)
        self.get_logger().info(f"recording for {self.duration:.0f} s")

    def on_state(self, msg: Odometry):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        h = msg.pose.pose.orientation.z
        if self.last_xy is not None:
            step = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            self.distance += step
            dh = (h - self.last_heading + math.pi) % (2 * math.pi) - math.pi
            self.heading_change += abs(dh)
        self.last_xy = (x, y)
        self.last_heading = h
        self.speeds.append(math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y))

    def on_xte(self, msg: Float64):
        self.xte.append(abs(msg.data))

    def on_steer(self, msg: StampedFloat64Msg):
        self.steer.append(msg.data)
        self.steer_t.append(self.get_clock().now().nanoseconds * 1e-9)

    def check_done(self):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if elapsed >= self.duration:
            self.report(elapsed)
            raise SystemExit(0)

    def report(self, elapsed):
        def rms(v):
            return math.sqrt(sum(a * a for a in v) / len(v)) if v else float("nan")

        rates = []
        for i in range(1, len(self.steer)):
            dt = self.steer_t[i] - self.steer_t[i - 1]
            if dt > 1e-4:
                rates.append((self.steer[i] - self.steer[i - 1]) / dt)

        summary = {
            "elapsed_s": round(elapsed, 1),
            "distance_m": round(self.distance, 1),
            "speed_mean_mps": round(sum(self.speeds) / len(self.speeds), 2) if self.speeds else None,
            "xte_rms_m": round(rms(self.xte), 3),
            "xte_max_m": round(max(self.xte), 3) if self.xte else None,
            "steer_rms_deg": round(rms(self.steer), 2),
            "steer_max_deg": round(max(abs(s) for s in self.steer), 2) if self.steer else None,
            "steer_rate_rms_dps": round(rms(rates), 1),
            "heading_change_per_m": round(self.heading_change / self.distance, 4) if self.distance > 0 else None,
            "samples": {"state": len(self.speeds), "xte": len(self.xte), "steer": len(self.steer)},
        }
        text = json.dumps(summary, indent=2)
        print("SIM_METRICS " + json.dumps(summary))
        if self.out:
            with open(self.out, "w") as f:
                f.write(text)


def main(args=None):
    rclpy.init(args=args)
    node = SimMetrics()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
