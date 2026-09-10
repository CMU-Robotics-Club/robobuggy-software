#!/usr/bin/env python3
"""
opponent_tracker.py
-------------------
The single fusion authority for other buggies (DECISIONS.md D5).

Inputs (buggy/DetectionArrayMsg, capture-time stamped):
    lidar/detection_array, vision/detection_array   from the sensor adapters
    radio/detection_array                           from radio_detection_adapter.py (optional)
    other/stateNoUKF (nav_msgs/Odometry)            optional raw radio position as a "radio"
                                                    source (simulation), NOT the UKF output
Output:
    perception/tracking   buggy/TrackingResultMsg: every live track with covariance,
                          last observation time, observation count, confirmation, sources
    perception/tracks     legacy buggy/TrackedObjectsMsg (confirmed tracks) for old tools
    debug/tracker/status  JSON counters, including suspected duplicates

The algorithm lives in racing/tracking.py: one-to-one gated assignment per
message (scipy linear_sum_assignment), constant-velocity prediction, no
post-hoc merging, observations counted once per capture stamp, prediction-only
outputs never counted. Messages are held in a short reorder buffer so sources
are ingested in capture-time order; late or repeated stamps are rejected and
counted.
"""

import heapq
import json

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from racing.health import stamp_seconds
from racing.tracking import MultiObjectTracker, Observation

from buggy.msg import DetectionArrayMsg, TrackedObjectsMsg, TrackingResultMsg, TrackMsg


class OpponentTracker(Node):
    def __init__(self):
        super().__init__("opponent_tracker")
        self.declare_parameter("detection_topics", ["lidar/detection_array", "vision/detection_array"])
        self.declare_parameter("radio_topic", "")                 # radio_detection_adapter output
        self.declare_parameter("raw_radio_odometry_topic", "")    # sim: other/stateNoUKF
        self.declare_parameter("raw_radio_std_m", 1.5)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("reorder_buffer_s", 0.15)
        self.declare_parameter("max_input_age_s", 0.5)
        self.declare_parameter("max_age_s", 1.5)
        self.declare_parameter("confirm_hits", 3)
        self.declare_parameter("gate_chi2", 9.21)
        self.declare_parameter("gate_max_m", 6.0)
        self.declare_parameter("acceleration_std", 3.0)   # m/s^2 process noise; see racing/tracking.py
        self.declare_parameter("min_confidence", 0.0)
        # per-source calibration uncertainty added to every measurement covariance (metres)
        self.declare_parameter("lidar_calibration_std_m", 0.15)
        self.declare_parameter("camera_calibration_std_m", 0.30)
        self.declare_parameter("radio_calibration_std_m", 0.50)
        self.declare_parameter("duplicate_distance_m", 1.5)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.buffer_s = float(p("reorder_buffer_s"))
        self.max_input_age = float(p("max_input_age_s"))
        self.confirm_hits = int(p("confirm_hits"))
        self.min_confidence = float(p("min_confidence"))
        self.raw_radio_std = float(p("raw_radio_std_m"))
        self.duplicate_distance = float(p("duplicate_distance_m"))
        self.tracker = MultiObjectTracker(
            max_age=float(p("max_age_s")), confirm_hits=self.confirm_hits,
            gate_chi2=float(p("gate_chi2")), gate_distance=float(p("gate_max_m")),
            acceleration_std=float(p("acceleration_std")),
            source_std={"lidar": float(p("lidar_calibration_std_m")),
                        "camera": float(p("camera_calibration_std_m")),
                        "vision": float(p("camera_calibration_std_m")),
                        "radio": float(p("radio_calibration_std_m"))},
        )
        self.heap = []          # (stamp, seq, source, observations, flags)
        self.seq = 0
        self.source_flags = {}  # source -> last flags (age unknown, uncompensated)
        self.counts = {"messages": 0, "detections": 0, "unobserved_skipped": 0, "low_confidence": 0}

        for topic in list(p("detection_topics")) + ([p("radio_topic")] if p("radio_topic") else []):
            self.create_subscription(DetectionArrayMsg, topic, self.on_detections, 10)
        if p("raw_radio_odometry_topic"):
            self.create_subscription(Odometry, p("raw_radio_odometry_topic"), self.on_raw_radio, 5)

        self.result_pub = self.create_publisher(TrackingResultMsg, "perception/tracking", 1)
        self.legacy_pub = self.create_publisher(TrackedObjectsMsg, "perception/tracks", 1)
        self.status_pub = self.create_publisher(String, "debug/tracker/status", 1)
        self.create_timer(1.0 / float(p("rate_hz")), self.publish)
        self.get_logger().info(
            f"tracker: topics {p('detection_topics')} radio '{p('radio_topic')}' raw radio '{p('raw_radio_odometry_topic')}'"
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def on_detections(self, msg: DetectionArrayMsg):
        self.counts["messages"] += 1
        observations = []
        for det in msg.detections:
            if not det.observed:
                self.counts["unobserved_skipped"] += 1
                continue
            if det.confidence < self.min_confidence:
                self.counts["low_confidence"] += 1
                continue
            cov = np.asarray(det.position_covariance, dtype=float).reshape(3, 3)[:2, :2]
            extent = np.array([det.extent.x, det.extent.y, det.extent.z], dtype=float)
            observations.append(Observation(
                np.array([det.position.x, det.position.y], dtype=float), cov, extent,
                bool(det.extent_known), True, str(det.object_id)))
        self.counts["detections"] += len(observations)
        flags = []
        if msg.source_age_unknown:
            flags.append(f"{msg.source}_age_unknown")
        if not msg.motion_compensated:
            flags.append(f"{msg.source}_uncompensated")
        self.source_flags[msg.source] = flags
        self.enqueue(stamp_seconds(msg.header.stamp), str(msg.source), observations)

    def on_raw_radio(self, msg: Odometry):
        """Raw relayed GPS position (not the UKF output) as a radio observation."""
        self.counts["messages"] += 1
        c = msg.pose.covariance
        std = float(np.sqrt(max(c[0], 0.0) + max(c[7], 0.0))) if c[0] > 0 else self.raw_radio_std
        obs = Observation(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], dtype=float),
                          np.eye(2) * std * std, object_id="NAND")
        self.source_flags["radio"] = ["radio_age_unknown"]
        self.counts["detections"] += 1
        stamp = stamp_seconds(msg.header.stamp)
        self.enqueue(stamp if stamp > 0 else self.now_s(), "radio", [obs])

    def enqueue(self, stamp, source, observations):
        self.seq += 1
        heapq.heappush(self.heap, (stamp, self.seq, source, observations))

    def drain(self, now):
        """Ingest everything older than the reorder buffer, in capture-time order."""
        while self.heap and self.heap[0][0] <= now - self.buffer_s:
            stamp, _, source, observations = heapq.heappop(self.heap)
            self.tracker.ingest(observations, stamp, source, now, self.max_input_age)

    # ------------------------------------------------------------------ output
    def publish(self):
        now = self.now_s()
        self.drain(now)
        live = self.tracker.snapshot(now)

        out = TrackingResultMsg()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "utm"
        last = self.tracker.last_stamp
        if last > 0:
            out.source_stamp.sec = int(last)
            out.source_stamp.nanosec = int((last - int(last)) * 1e9)
        reasons = sorted({f for flags in self.source_flags.values() for f in flags})
        fresh_sources = [s for s, t in self.tracker.source_stamps.items() if now - t <= self.max_input_age]
        out.perception_ready = bool(fresh_sources)
        if not fresh_sources:
            reasons.append("no_fresh_source")
        out.reasons = reasons

        legacy = TrackedObjectsMsg()
        legacy.header = out.header
        for track, state, cov in live:
            t = TrackMsg()
            t.id = int(track.id)
            t.position.x, t.position.y = float(state[0]), float(state[1])
            t.velocity.x, t.velocity.y = float(state[2]), float(state[3])
            t.covariance = [float(v) for v in np.asarray(cov).reshape(-1)]
            t.extent.x, t.extent.y, t.extent.z = [float(v) for v in track.extent]
            t.extent_known = bool(track.extent_known)
            t.last_observed_stamp.sec = int(track.last_observed)
            t.last_observed_stamp.nanosec = int((track.last_observed - int(track.last_observed)) * 1e9)
            t.observation_count = int(track.hits)
            t.confirmed = bool(track.hits >= self.confirm_hits)
            t.sources = sorted(track.sources)
            out.tracks.append(t)
            if t.confirmed:
                legacy.ids.append(t.id)
                legacy.easting.append(t.position.x)
                legacy.northing.append(t.position.y)
                legacy.vx.append(t.velocity.x)
                legacy.vy.append(t.velocity.y)
                legacy.pos_std.append(float(np.sqrt(max(cov[0, 0] + cov[1, 1], 0.0))))
                legacy.age_s.append(float(now - track.last_observed))
                legacy.hits.append(t.observation_count)
        self.result_pub.publish(out)
        self.legacy_pub.publish(legacy)
        status = {
            "confirmed": len(legacy.ids), "live": len(live),
            "suspected_duplicates": int(self.tracker.suspected_duplicates(now, self.duplicate_distance)),
            "queued": len(self.heap), "fresh_sources": fresh_sources,
            **self.counts, **{f"tracker_{k}": v for k, v in self.tracker.counts.items()},
        }
        self.status_pub.publish(String(data=json.dumps(status)))
        self.get_logger().info(f"tracker status {status}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
