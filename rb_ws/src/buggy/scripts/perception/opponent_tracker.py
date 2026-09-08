#!/usr/bin/env python3
"""
opponent_tracker.py
-------------------
Turns raw detections from any number of sensors into a list of tracked buggies.

The camera and the lidar each report "there is something at this position"
several times a second, with noise, misses, and the occasional false alarm.
This node keeps one small Kalman filter per object (position and velocity,
constant-velocity model), decides which detection belongs to which object
(nearest neighbour with a statistical gate), confirms an object only after it
has been seen several times, and forgets it when it has not been seen for a
while. That is the standard "tracking by detection" recipe (AB3DMOT style) and
it is what lets the planner reason about buggies that never told us where they
are.

Inputs
    detection_topics   buggy/DetectionsMsg from lidar_opponent_node, detector_node,
                       or perception_sim (default: lidar/detections, vision/detections)
    other/state        optional: NAND's radio-based estimate, treated as one more
                       detection source so NAND appears in the same list
Output
    perception/tracks  buggy/TrackedObjectsMsg with every confirmed track
    debug/tracker/status  std_msgs/String, counts for Foxglove
"""

import json

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from buggy.msg import DetectionsMsg, TrackedObjectsMsg


class Track:
    def __init__(self, tid, x, y, std, now):
        self.id = tid
        self.x = np.array([x, y, 0.0, 0.0])  # x, y, vx, vy
        self.P = np.diag([std * std, std * std, 25.0, 25.0])
        self.last_update = now
        self.last_predict = now
        self.hits = 1
        self.misses = 0

    def predict(self, now, q_pos, q_vel):
        dt = max(now - self.last_predict, 0.0)
        if dt <= 0.0:
            return
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        Q = np.diag([q_pos * dt, q_pos * dt, q_vel * dt, q_vel * dt])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.last_predict = now

    def mahalanobis(self, z, std):
        S = self.P[0:2, 0:2] + np.eye(2) * std * std
        nu = z - self.x[0:2]
        try:
            return float(nu @ np.linalg.solve(S, nu)), S
        except np.linalg.LinAlgError:
            return float("inf"), S

    def update(self, z, std, now):
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        R = np.eye(2) * std * std
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x)
        self.P = (np.eye(4) - K @ H) @ self.P
        self.last_update = now
        self.hits += 1
        self.misses = 0


class OpponentTracker(Node):
    def __init__(self):
        super().__init__("opponent_tracker")
        self.declare_parameter("detection_topics", ["lidar/detections", "vision/detections"])
        self.declare_parameter("include_other_state", True)
        self.declare_parameter("other_state_std_m", 1.0)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("gate_chi2", 9.21)      # 99 % for 2 dof
        self.declare_parameter("gate_max_m", 6.0)      # never associate beyond this, whatever the covariance says
        self.declare_parameter("confirm_hits", 3)
        self.declare_parameter("max_age_s", 1.5)
        self.declare_parameter("q_pos", 0.5)           # position process noise per second (m^2/s)
        self.declare_parameter("q_vel", 4.0)           # velocity process noise per second ((m/s)^2/s)
        self.declare_parameter("merge_distance_m", 1.5)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.gate_chi2 = float(p("gate_chi2"))
        self.gate_max = float(p("gate_max_m"))
        self.confirm_hits = int(p("confirm_hits"))
        self.max_age = float(p("max_age_s"))
        self.q_pos = float(p("q_pos"))
        self.q_vel = float(p("q_vel"))
        self.merge_d = float(p("merge_distance_m"))
        self.other_std = float(p("other_state_std_m"))

        self.tracks = []
        self.next_id = 1
        self.counts = {"detections": 0, "associated": 0, "new_tracks": 0, "dropped": 0}

        for topic in p("detection_topics"):
            self.create_subscription(DetectionsMsg, topic, self.on_detections, 10)
        if p("include_other_state"):
            self.create_subscription(Odometry, "other/state", self.on_other_state, 1)

        self.tracks_pub = self.create_publisher(TrackedObjectsMsg, "perception/tracks", 1)
        self.status_pub = self.create_publisher(String, "debug/tracker/status", 1)
        self.create_timer(1.0 / float(p("rate_hz")), self.publish)

    # ------------------------------------------------------------------ inputs
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_other_state(self, msg):
        c = msg.pose.covariance
        std = float(np.sqrt(max(c[0], 0.0) + max(c[7], 0.0))) if c[0] > 0 else self.other_std
        std = max(std, 0.3)
        self.ingest([(msg.pose.pose.position.x, msg.pose.pose.position.y, std)])

    def on_detections(self, msg: DetectionsMsg):
        dets = []
        for i in range(len(msg.easting)):
            std = msg.pos_std[i] if i < len(msg.pos_std) else 1.0
            dets.append((msg.easting[i], msg.northing[i], max(float(std), 0.05)))
        self.ingest(dets)

    # ------------------------------------------------------------------ core
    def ingest(self, dets):
        now = self.now_s()
        for t in self.tracks:
            t.predict(now, self.q_pos, self.q_vel)
        self.counts["detections"] += len(dets)

        # greedy nearest-neighbour association by Mahalanobis distance
        unused = set(range(len(dets)))
        pairs = []
        for ti, t in enumerate(self.tracks):
            for di in unused:
                z = np.array(dets[di][0:2])
                m2, _ = t.mahalanobis(z, dets[di][2])
                if m2 < self.gate_chi2 and np.linalg.norm(z - t.x[0:2]) < self.gate_max:
                    pairs.append((m2, ti, di))
        pairs.sort()
        used_tracks = set()
        for m2, ti, di in pairs:
            if ti in used_tracks or di not in unused:
                continue
            z = np.array(dets[di][0:2])
            self.tracks[ti].update(z, dets[di][2], now)
            used_tracks.add(ti)
            unused.discard(di)
            self.counts["associated"] += 1

        # leftovers start new tentative tracks (unless they sit on an existing one)
        for di in unused:
            z = np.array(dets[di][0:2])
            if any(np.linalg.norm(z - t.x[0:2]) < self.merge_d for t in self.tracks):
                continue
            self.tracks.append(Track(self.next_id, z[0], z[1], dets[di][2], now))
            self.next_id += 1
            self.counts["new_tracks"] += 1

        self.prune(now)

    def prune(self, now):
        keep = []
        for t in self.tracks:
            age = now - t.last_update
            # tentative tracks die fast, confirmed ones survive a short blackout
            limit = self.max_age if t.hits >= self.confirm_hits else 0.5
            if age > limit:
                self.counts["dropped"] += 1
                continue
            keep.append(t)
        # merge duplicates that converged onto the same object
        merged = []
        for t in keep:
            dup = next((m for m in merged if np.linalg.norm(m.x[0:2] - t.x[0:2]) < self.merge_d), None)
            if dup is None:
                merged.append(t)
            elif t.hits > dup.hits:
                merged.remove(dup)
                merged.append(t)
        self.tracks = merged

    # ------------------------------------------------------------------ output
    def publish(self):
        now = self.now_s()
        for t in self.tracks:
            t.predict(now, self.q_pos, self.q_vel)
        self.prune(now)
        msg = TrackedObjectsMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "utm"
        for t in self.tracks:
            if t.hits < self.confirm_hits:
                continue
            msg.ids.append(int(t.id))
            msg.easting.append(float(t.x[0]))
            msg.northing.append(float(t.x[1]))
            msg.vx.append(float(t.x[2]))
            msg.vy.append(float(t.x[3]))
            msg.pos_std.append(float(np.sqrt(max(t.P[0, 0] + t.P[1, 1], 0.0))))
            msg.age_s.append(float(now - t.last_update))
            msg.hits.append(int(t.hits))
        self.tracks_pub.publish(msg)
        self.status_pub.publish(String(data=json.dumps({
            "confirmed": len(msg.ids), "tentative": len(self.tracks) - len(msg.ids), **self.counts,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = OpponentTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
