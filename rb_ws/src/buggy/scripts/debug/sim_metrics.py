#!/usr/bin/env python3
"""
sim_metrics.py
--------------
Records tracking quality while a launch is running, prints a JSON summary and
EXITS NON-ZERO when the run failed a gate (DECISIONS.md D12), so it can be used
as a regression check and not only as a telemetry summary.

Run alongside a sim launch, in the buggy's namespace:
    ros2 run buggy sim_metrics.py --ros-args -r __ns:=/SC -p duration:=90.0 -p out:=/tmp/metrics.json

Metrics:
    distance_m           how far the buggy travelled along its own track
    xte_rms_m, xte_max_m cross-track error of the front axle (from the controller)
    steer_rms_deg, steer_max_deg      commanded steering
    steer_rate_rms_dps   how frantic the steering is (deg per second, RMS)
    heading_change_per_m mean |d heading| per metre travelled: a curvature proxy
    speed_mean_mps       from the state message
    plan_source_s        seconds the controller spent on the envelope vs the reference fallback
    planning_status      count of planner envelopes by status
    ineligible_steer_events   envelope-mode controller seen steering on a plan the planner
                              had marked not control-eligible (must be 0)

Gates (exit 1): no state samples; ineligible_steer_events > 0; envelope fraction below
min_envelope_fraction when the controller runs in envelope mode; xte_max_m above max_xte_m
when that parameter is > 0.
"""

import json
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

from buggy.msg import StampedFloat64Msg


class EnvelopeWatch:
    """Cross-checks controller/plan_source against planning/status (shared with pass_metrics)."""

    def __init__(self, node):
        self.eligible_by_id = {}
        self.status_counts = {}
        self.source_time = {}
        self.last_source = None
        self.last_source_t = None
        self.ineligible_steer_events = 0
        self.node = node
        node.create_subscription(String, "planning/status", self.on_status, 10)
        node.create_subscription(String, "controller/plan_source", self.on_source, 10)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        self.eligible_by_id[int(data.get("plan_id", -1))] = bool(data.get("control_eligible", False))
        status = str(data.get("status", "?"))
        self.status_counts[status] = self.status_counts.get(status, 0) + 1
        if len(self.eligible_by_id) > 5000:
            for key in sorted(self.eligible_by_id)[:2500]:
                del self.eligible_by_id[key]

    def on_source(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        now = self.now()
        source = str(data.get("source"))
        if self.last_source is not None and self.last_source_t is not None:
            self.source_time[self.last_source] = self.source_time.get(self.last_source, 0.0) + (now - self.last_source_t)
        self.last_source, self.last_source_t = source, now
        plan_id = data.get("plan_id")
        if source == "envelope" and plan_id is not None and self.eligible_by_id.get(int(plan_id)) is False:
            self.ineligible_steer_events += 1

    def finish(self):
        if self.last_source is not None and self.last_source_t is not None:
            now = self.now()
            self.source_time[self.last_source] = self.source_time.get(self.last_source, 0.0) + (now - self.last_source_t)
            self.last_source_t = now

    def envelope_fraction(self):
        total = sum(self.source_time.values())
        return (self.source_time.get("envelope", 0.0) / total) if total > 0 else None

    def summary(self):
        return {
            "plan_source_s": {k: round(v, 1) for k, v in self.source_time.items()},
            "envelope_fraction": None if self.envelope_fraction() is None else round(self.envelope_fraction(), 3),
            "planning_status": dict(self.status_counts),
            "ineligible_steer_events": self.ineligible_steer_events,
        }


class SimMetrics(Node):
    def __init__(self):
        super().__init__("sim_metrics")
        self.declare_parameter("duration", 60.0)
        self.declare_parameter("out", "")
        self.declare_parameter("xte_topic", "controller/controller/debug/cross_track_error")
        self.declare_parameter("steer_topic", "input/steering")
        self.declare_parameter("state_topic", "self/state")
        self.declare_parameter("min_envelope_fraction", 0.0)
        self.declare_parameter("max_xte_m", 0.0)

        self.duration = float(self.get_parameter("duration").value)
        self.out = str(self.get_parameter("out").value)
        self.min_envelope = float(self.get_parameter("min_envelope_fraction").value)
        self.max_xte = float(self.get_parameter("max_xte_m").value)

        self.create_subscription(Odometry, self.get_parameter("state_topic").value, self.on_state, 10)
        self.create_subscription(Float64, self.get_parameter("xte_topic").value, self.on_xte, 10)
        self.create_subscription(StampedFloat64Msg, self.get_parameter("steer_topic").value, self.on_steer, 10)
        self.envelope = EnvelopeWatch(self)

        self.last_xy = None
        self.last_heading = None
        self.distance = 0.0
        self.heading_change = 0.0
        self.speeds = []
        self.xte = []
        self.steer = []
        self.steer_t = []
        self.exit_code = 0

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
            raise SystemExit(self.exit_code)

    def report(self, elapsed):
        def rms(v):
            return math.sqrt(sum(a * a for a in v) / len(v)) if v else float("nan")

        rates = []
        for i in range(1, len(self.steer)):
            dt = self.steer_t[i] - self.steer_t[i - 1]
            if dt > 1e-4:
                rates.append((self.steer[i] - self.steer[i - 1]) / dt)
        self.envelope.finish()

        failures = []
        if not self.speeds:
            failures.append("no_state_samples")
        env = self.envelope.summary()
        if env["ineligible_steer_events"] > 0:
            failures.append("ineligible_plan_steered")
        frac = self.envelope.envelope_fraction()
        if self.min_envelope > 0 and (frac is None or frac < self.min_envelope):
            failures.append(f"envelope_fraction_below_{self.min_envelope}")
        if self.max_xte > 0 and self.xte and max(self.xte) > self.max_xte:
            failures.append(f"xte_max_above_{self.max_xte}")
        self.exit_code = 1 if failures else 0

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
            **env,
            "failures": failures,
            "passed_gates": not failures,
        }
        text = json.dumps(summary, indent=2)
        print("SIM_METRICS " + json.dumps(summary))
        if self.out:
            with open(self.out, "w", encoding="utf-8") as f:
                f.write(text)


def main(args=None):
    rclpy.init(args=args)
    node = SimMetrics()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = int(exc.code or 0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
