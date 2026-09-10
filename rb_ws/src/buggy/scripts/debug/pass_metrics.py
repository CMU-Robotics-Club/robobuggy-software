#!/usr/bin/env python3
"""
pass_metrics.py
---------------
Regression harness for overtaking in sim_2d_double.xml. Watches both buggies,
the ghosts and the planner, prints one JSON line and EXITS NON-ZERO when a gate
fails (DECISIONS.md D12):

    passed              SC finished ahead of NAND by more than pass_margin_m
    pass_time_s         when SC's projection first got ahead of NAND (None if never)
    min_separation_m    closest centre-to-centre distance to NAND during the run
    min_lateral_gap_m   smallest lateral gap while alongside NAND (negative = SC to the right)
    min_ghost_separation_m / ghost_collision   same against the fake-sensor ghosts
    planner_states_s    seconds spent in RACELINE / PASS / REJOIN
    target_offset_min/max   extreme lateral targets the planner committed to (sign = side)
    plan_source_s, planning_status, ineligible_steer_events   envelope bookkeeping
    sc_xte_rms_m        SC cross-track error while following its local path

Gates (exit 1): no SC state; collision with NAND; ghost collision unless allow_ghost_collision;
ineligible_steer_events > 0; require_pass and not passed; expect_pass_side ("left"/"right")
and the planner never committed to that side; require_fallback_reported and the controller
never reported reference_fallback (used by the blocked scenario).

Run in the SC namespace next to the double sim:
    ros2 run buggy pass_metrics.py --ros-args -r __ns:=/SC -p duration:=120.0
"""

import json
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int8

from debug.sim_metrics import EnvelopeWatch
from util.track import Track


class PassMetrics(Node):
    def __init__(self):
        super().__init__("pass_metrics")
        self.declare_parameter("duration", 120.0)
        self.declare_parameter("traj_name", "buggycourse_sc.json")
        self.declare_parameter("other_state_topic", "/NAND/self/state")
        self.declare_parameter("pass_margin_m", 5.0)
        self.declare_parameter("alongside_window_m", 6.0)
        self.declare_parameter("collision_radius_m", 1.2)
        self.declare_parameter("out", "")
        self.declare_parameter("require_pass", False)
        self.declare_parameter("expect_pass_side", "")
        self.declare_parameter("allow_ghost_collision", False)
        self.declare_parameter("require_fallback_reported", False)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.duration = float(p("duration"))
        self.pass_margin = float(p("pass_margin_m"))
        self.window = float(p("alongside_window_m"))
        self.collision_r = float(p("collision_radius_m"))
        self.out = str(p("out"))
        self.require_pass = bool(p("require_pass"))
        self.expect_side = str(p("expect_pass_side")).lower()
        self.allow_ghost_collision = bool(p("allow_ghost_collision"))
        self.require_fallback = bool(p("require_fallback_reported"))
        self.track = Track.from_files(os.environ["TRAJPATH"] + p("traj_name"), ds=1.0)

        self.sc = None
        self.nand = None
        self.t0 = self.get_clock().now()
        self.pass_time = None
        self.min_sep = float("inf")
        self.min_lat_gap = None
        self.state_time = {0: 0.0, 1: 0.0, 2: 0.0}
        self.cur_state = 0
        self.last_state_t = None
        self.xte = []
        self.last_s = (None, None)
        self.finished = False
        self.final_gap = None
        self.ghosts = []
        self.min_ghost_sep = float("inf")
        self.target_min = None
        self.target_max = None
        self.exit_code = 0

        self.create_subscription(Odometry, "self/state", self.on_sc, 10)
        self.create_subscription(Odometry, p("other_state_topic"), self.on_nand, 10)
        self.create_subscription(Int8, "debug/planner/state", self.on_state, 10)
        self.create_subscription(Float64, "debug/planner/target_offset", self.on_target, 10)
        self.create_subscription(PoseArray, "perception_sim/ghost_truth", self.on_ghosts, 10)
        self.create_subscription(Float64, "controller/controller/debug/cross_track_error", self.on_xte, 10)
        self.envelope = EnvelopeWatch(self)
        self.create_timer(0.1, self.tick)

    def on_sc(self, msg):
        self.sc = msg

    def on_nand(self, msg):
        self.nand = msg

    def on_ghosts(self, msg):
        self.ghosts = [(q.position.x, q.position.y) for q in msg.poses]

    def on_target(self, msg):
        v = float(msg.data)
        self.target_min = v if self.target_min is None else min(self.target_min, v)
        self.target_max = v if self.target_max is None else max(self.target_max, v)

    def on_state(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_state_t is not None:
            self.state_time[self.cur_state] += now - self.last_state_t
        self.cur_state = int(msg.data)
        self.last_state_t = now

    def on_xte(self, msg):
        self.xte.append(abs(msg.data))

    def tick(self):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if self.sc is not None:
            sx, sy = self.sc.pose.pose.position.x, self.sc.pose.pose.position.y
            for gx, gy in self.ghosts:
                self.min_ghost_sep = min(self.min_ghost_sep, math.hypot(sx - gx, sy - gy))
        if self.sc is not None and self.nand is not None:
            sx, sy = self.sc.pose.pose.position.x, self.sc.pose.pose.position.y
            nx, ny = self.nand.pose.pose.position.x, self.nand.pose.pose.position.y
            sep = math.hypot(sx - nx, sy - ny)
            self.min_sep = min(self.min_sep, sep)
            s_sc, d_sc = self.track.frenet(sx, sy)
            s_nd, d_nd = self.track.frenet(nx, ny)
            self.last_s = (s_sc, s_nd)
            if abs(s_sc - s_nd) < self.window:
                gap = d_sc - d_nd
                self.min_lat_gap = gap if self.min_lat_gap is None else min(self.min_lat_gap, gap, key=abs)
            if self.pass_time is None and s_sc > s_nd + self.pass_margin:
                self.pass_time = round(elapsed, 1)
            # freeze the verdict once SC reaches the end of the course: beyond the last
            # waypoint the projection wraps and the gap becomes meaningless
            if s_sc > self.track.length - 20.0 and not self.finished:
                self.finished = True
                self.final_gap = s_sc - s_nd
        if elapsed >= self.duration:
            self.report(elapsed)
            raise SystemExit(self.exit_code)

    def report(self, elapsed):
        s_sc, s_nd = self.last_s
        if self.final_gap is None and s_sc is not None:
            self.final_gap = s_sc - s_nd
        self.envelope.finish()
        env = self.envelope.summary()
        passed = bool(self.final_gap is not None and self.final_gap > self.pass_margin)
        collision = bool(self.min_sep < self.collision_r)
        ghost_collision = bool(self.min_ghost_sep < self.collision_r)

        failures = []
        if self.sc is None:
            failures.append("no_sc_state")
        if collision:
            failures.append("collision_with_nand")
        if ghost_collision and not self.allow_ghost_collision:
            failures.append("collision_with_ghost")
        if env["ineligible_steer_events"] > 0:
            failures.append("ineligible_plan_steered")
        if self.require_pass and not passed:
            failures.append("pass_not_completed")
        if self.expect_side == "left" and not (self.target_max is not None and self.target_max > 0.5):
            failures.append("expected_left_pass")
        if self.expect_side == "right" and not (self.target_min is not None and self.target_min < -0.5):
            failures.append("expected_right_pass")
        if self.require_fallback and env["plan_source_s"].get("reference_fallback", 0.0) <= 0.0:
            failures.append("fallback_never_reported")
        self.exit_code = 1 if failures else 0

        summary = {
            "elapsed_s": round(elapsed, 1),
            "passed": passed,
            "pass_time_s": self.pass_time,
            "sc_finished_course": self.finished,
            "final_gap_m": None if self.final_gap is None else round(self.final_gap, 1),
            "min_separation_m": round(self.min_sep, 2) if self.min_sep < float("inf") else None,
            "min_lateral_gap_m": None if self.min_lat_gap is None else round(self.min_lat_gap, 2),
            "collision": collision,
            "min_ghost_separation_m": round(self.min_ghost_sep, 2) if self.min_ghost_sep < float("inf") else None,
            "ghost_collision": ghost_collision,
            "planner_states_s": {"RACELINE": round(self.state_time[0], 1), "PASS": round(self.state_time[1], 1), "REJOIN": round(self.state_time[2], 1)},
            "target_offset_min": None if self.target_min is None else round(self.target_min, 2),
            "target_offset_max": None if self.target_max is None else round(self.target_max, 2),
            "sc_xte_rms_m": round(float(np.sqrt(np.mean(np.square(self.xte)))), 3) if self.xte else None,
            "sc_xte_max_m": round(max(self.xte), 3) if self.xte else None,
            **env,
            "failures": failures,
            "passed_gates": not failures,
        }
        print("PASS_METRICS " + json.dumps(summary))
        if self.out:
            with open(self.out, "w", encoding="utf-8") as f:
                json.dump(summary, f, indent=2)


def main(args=None):
    rclpy.init(args=args)
    node = PassMetrics()
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
