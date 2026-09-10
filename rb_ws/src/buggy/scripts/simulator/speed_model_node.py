#!/usr/bin/env python3
"""
speed_model_node.py
-------------------
Drives the 2D simulator's speed from the gravity speed model instead of a
constant, so a smoother line really does arrive faster in the sim.

Subscribes self/state, projects the buggy onto the reference track, looks up
v(s) from util/speed_model.py (config/course_zones.yaml) and publishes it on
sim/velocity, which engine.py already consumes. Also publishes
debug/speed_model/s and debug/speed_model/lap_time_s for Foxglove.

The speed is looked up on the line the buggy is actually driving? No: it is
looked up on the reference track by arc length, and the curvature term uses the
curvature of the buggy's own recent motion (from heading change per metre), so
a wider, smoother line loses less speed than a tight one. That is the coupling
the plain simulator was missing.
"""

import os

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from util.track import Track
from util.speed_model import SpeedModel, G


class SpeedModelNode(Node):
    def __init__(self):
        super().__init__("speed_model")
        self.declare_parameter("traj_name", "buggycourse_sc.json")
        self.declare_parameter("zones_file", "")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("curvature_window_m", 8.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        trajpath = os.environ["TRAJPATH"]
        zones = p("zones_file") or os.path.join(os.environ["RBROOT"], "src/buggy/config/course_zones.yaml")

        self.track = Track.from_files(trajpath + p("traj_name"), ds=1.0)
        self.model = SpeedModel(zones, self.track)
        self.get_logger().info(
            f"speed model ready: nominal lap {self.model.lap_time():.1f} s, "
            f"v max {self.model.v.max():.1f} m/s"
        )

        self.window = float(p("curvature_window_m"))
        self.hist = []  # (x, y, heading)
        self.v = None
        self.last_s = None
        self.state = None

        self.create_subscription(Odometry, "self/state", self.on_state, 1)
        self.vel_pub = self.create_publisher(Float64, "sim/velocity", 1)
        self.s_pub = self.create_publisher(Float64, "debug/speed_model/s", 1)
        self.lap_pub = self.create_publisher(Float64, "debug/speed_model/lap_time_s", 1)
        self.create_timer(1.0 / float(p("rate_hz")), self.step)
        self.lap_pub.publish(Float64(data=self.model.lap_time()))

    def on_state(self, msg):
        self.state = msg

    def own_curvature(self):
        """|d heading / d s| over the last few metres of actual motion."""
        if len(self.hist) < 3:
            return 0.0
        x0, y0, h0 = self.hist[0]
        x1, y1, h1 = self.hist[-1]
        dist = float(np.hypot(x1 - x0, y1 - y0))
        if dist < 0.5:
            return 0.0
        dh = (h1 - h0 + np.pi) % (2 * np.pi) - np.pi
        return abs(dh) / dist

    def step(self):
        if self.state is None:
            return
        x = self.state.pose.pose.position.x
        y = self.state.pose.pose.position.y
        h = self.state.pose.pose.orientation.z
        self.hist.append((x, y, h))
        while len(self.hist) > 2 and np.hypot(x - self.hist[0][0], y - self.hist[0][1]) > self.window:
            self.hist.pop(0)

        s, _ = self.track.frenet(x, y)
        zone = self.model.zone_at(s)
        dt = 1.0 / float(self.get_parameter("rate_hz").value)

        if zone["type"] == "pushed" or self.v is None:
            self.v = float(zone.get("push_speed_mps", 5.0)) if zone["type"] == "pushed" else self.model.speed_at(s)
        else:
            k = self.own_curvature()
            vv = max(self.v, self.model.v_min)
            accel = (-G * self.model.grade_at(s) - G * self.model.crr
                     - self.model.cda_over_m * vv * vv
                     - self.model.c_scrub * vv * vv * k * k)
            self.v = float(np.clip(vv + accel * dt, self.model.v_min, self.model.v_max))

        self.vel_pub.publish(Float64(data=self.v))
        self.s_pub.publish(Float64(data=float(s)))


def main(args=None):
    rclpy.init(args=args)
    node = SpeedModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
