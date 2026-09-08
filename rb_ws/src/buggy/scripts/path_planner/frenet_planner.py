#!/usr/bin/env python3
"""
frenet_planner.py
-----------------
Online local planner for Short Circuit: samples smooth lateral offsets from
the global raceline in the track's Frenet frame, scores each candidate, and
publishes the best one as self/cur_traj for the Stanley controller.

Replaces the sigmoid "bend left near NAND" rule in path_planner.py with a
search that can (a) overtake on the left with a clearance that depends on how
fast we are closing, (b) return to the raceline as soon as it is safe, and
(c) never leave the road, all while preferring the flattest (lowest curvature)
option because a gravity vehicle only keeps speed by not steering.

Pipeline, at FREQUENCY Hz:
    1. Project ego and every known opponent onto the track: (s, d). Opponents
       come from perception/tracks (camera + lidar + radio fused by
       opponent_tracker.py) plus NAND's radio estimate on other/state.
    2. Predict each opponent along the track at its measured speed.
    3. Build candidate lateral profiles d(s) over a horizon: quintic smoothstep
       from the currently committed offset to a target offset over a transition
       length, then hold. Targets span the drivable width on a fixed grid.
    4. Reject candidates that leave the road, exceed the steering limit, or
       intersect any opponent's predicted footprint when both are at the same s
       at the same time. The side to pass each opponent on comes from
       side_policy: whichever side has more room (default), or fixed left/right.
    5. Cost = curvature^2 (integrated) + deviation from raceline + opponent
       proximity + change from the previous decision. Pick the minimum.
    6. Publish as a TrajectoryMsg (easting/northing lists + cur_idx).

All lengths in metres, angles in radians, UTM zone 17T like the rest of the
stack. Tunables are ROS parameters so they can be changed per config file.
"""

import os
from threading import Lock

import yaml

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int8

from util.trajectory import Trajectory
from util.track import Track

from buggy.msg import TrajectoryMsg, TrackedObjectsMsg


def smoothstep5(t):
    """Quintic smoothstep: zero 1st and 2nd derivative at both ends."""
    t = np.clip(t, 0.0, 1.0)
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


class FrenetPlanner(Node):

    def __init__(self):
        super().__init__("frenet_planner")
        self.get_logger().info("INITIALIZED.")

        # ---- files
        self.declare_parameter("traj_name", "buggycourse_sc.json")
        self.declare_parameter("curb_name", "buggycourse_curb.json")
        self.declare_parameter("right_boundary_name", "")
        self.declare_parameter("right_width", 0.5)      # drivable metres right of the raceline if no right file
        self.declare_parameter("vehicle_width", 1.2)
        self.declare_parameter("boundary_margin", 0.5)  # extra clearance from curbs
        self.declare_parameter("track_ds", 1.0)

        # ---- horizon and sampling
        self.declare_parameter("frequency", 10.0)
        self.declare_parameter("lookahead", 2.0)        # start the local path this far ahead of ego
        self.declare_parameter("horizon", 60.0)         # length of the published local path
        self.declare_parameter("resolution", 150)       # points in the published path
        self.declare_parameter("offset_step", 0.25)     # lateral grid spacing for targets
        self.declare_parameter("transition_lengths", [15.0, 25.0, 40.0])

        # ---- opponent handling
        self.declare_parameter("opponent_radius", 1.0)          # half length of the other buggy plus slack
        self.declare_parameter("lateral_clearance", 1.6)        # centre-to-centre lateral gap required when alongside
        self.declare_parameter("longitudinal_window", 6.0)      # |s_ego - s_opp| inside which we are "alongside"
        self.declare_parameter("opponent_timeout", 2.0)         # seconds before a stale opponent estimate is ignored
        self.declare_parameter("pass_left_only", True)   # superseded by side_policy, kept for old configs
        # which side to pass an opponent on: "more_room" (per opponent, whichever side has more
        # drivable width at its position), "left", "right", or "any"
        self.declare_parameter("side_policy", "more_room")
        # tracked buggies from opponent_tracker.py (camera + lidar + radio); NAND's radio estimate on
        # other/state is still used, deduplicated against the tracks
        self.declare_parameter("tracks_topic", "perception/tracks")
        self.declare_parameter("tracks_timeout", 1.0)
        self.declare_parameter("dedupe_distance_m", 3.0)
        self.declare_parameter("min_speed_for_prediction", 1.0)

        # ---- physical limits
        # Stanley clips at +/-20 deg; with the SC wheelbase that is tan(20 deg)/1.104 = 0.33 1/m of path
        # curvature. Anything above kappa_max would saturate the steering and the buggy leaves the line.
        self.declare_parameter("kappa_max", 0.25)
        # lateral acceleration the buggy can carry without scrubbing badly (soft penalty, m/s^2)
        self.declare_parameter("a_lat_max", 4.0)
        self.declare_parameter("w_a_lat", 30.0)
        # localization health gate: 0 ok, 1 degraded (no passing), 2 bad (hold raceline)
        self.declare_parameter("health_topic", "localization/health")
        self.declare_parameter("passing_needs_health_ok", True)
        # extra boundary margin (m) when localization is DEGRADED (health 1)
        self.declare_parameter("degraded_extra_margin", 0.5)
        # course zones file with no_pass_zones (empty string = none)
        self.declare_parameter("zones_file", "")

        # ---- cost weights
        self.declare_parameter("w_curvature", 400.0)
        self.declare_parameter("w_deviation", 1.0)
        self.declare_parameter("w_proximity", 6.0)
        self.declare_parameter("w_change", 2.0)
        self.declare_parameter("w_margin", 3.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        trajpath = os.environ["TRAJPATH"]

        right_name = p("right_boundary_name")
        self.track = Track.from_files(
            trajpath + p("traj_name"),
            left_boundary_json=trajpath + p("curb_name") if p("curb_name") else None,
            right_boundary_json=trajpath + right_name if right_name else None,
            default_right=float(p("right_width")),
            margin=float(p("boundary_margin")) + 0.5 * float(p("vehicle_width")),
            ds=float(p("track_ds")),
        )
        self.get_logger().info(
            f"track loaded: {self.track.length:.0f} m, left width median {np.median(self.track.w_left):.2f} m, "
            f"right width median {np.median(self.track.w_right):.2f} m"
        )

        self.lookahead = float(p("lookahead"))
        self.horizon = float(p("horizon"))
        self.resolution = int(p("resolution"))
        self.offset_step = float(p("offset_step"))
        self.transition_lengths = [float(v) for v in p("transition_lengths")]
        self.opp_radius = float(p("opponent_radius"))
        self.lat_clear = float(p("lateral_clearance"))
        self.long_window = float(p("longitudinal_window"))
        self.opp_timeout = float(p("opponent_timeout"))
        self.pass_left_only = bool(p("pass_left_only"))
        self.side_policy = str(p("side_policy")).lower()
        if self.side_policy not in ("more_room", "left", "right", "any"):
            self.get_logger().warn(f"unknown side_policy '{self.side_policy}', using more_room")
            self.side_policy = "more_room"
        self.tracks_timeout = float(p("tracks_timeout"))
        self.dedupe_d = float(p("dedupe_distance_m"))
        self.tracks_msg = None
        self.tracks_stamp = None
        self.min_pred_speed = float(p("min_speed_for_prediction"))
        self.w_curv = float(p("w_curvature"))
        self.w_dev = float(p("w_deviation"))
        self.w_prox = float(p("w_proximity"))
        self.w_change = float(p("w_change"))
        self.w_margin = float(p("w_margin"))
        self.kappa_max = float(p("kappa_max"))
        self.a_lat_max = float(p("a_lat_max"))
        self.w_a_lat = float(p("w_a_lat"))
        self.passing_needs_health_ok = bool(p("passing_needs_health_ok"))
        self.health = 0
        self.state = "RACELINE"   # RACELINE | PASS | REJOIN, with hysteresis so a 10 Hz replanner cannot flap
        self.degraded_margin = float(p("degraded_extra_margin"))
        self.no_pass = []
        self.hold_zones = []
        zones_file = p("zones_file") or os.path.join(os.environ.get("RBROOT", "/rb_ws"), "src/buggy/config/course_zones.yaml")
        try:
            with open(zones_file, "r") as f:
                cfg = yaml.safe_load(f) or {}
            self.no_pass = [(float(z["s_start"]), float(z["s_end"]), z.get("name", "")) for z in cfg.get("no_pass_zones", [])]
            # pusher transition zones: no lateral change at all (a pusher may be running alongside)
            self.hold_zones = [(float(z["s_start"]), float(z["s_end"]), z.get("name", "")) for z in cfg.get("no_lateral_change_zones", [])]
            self.get_logger().info(f"no-pass zones: {[n for _, _, n in self.no_pass]}; hold zones: {[n for _, _, n in self.hold_zones]}")
        except FileNotFoundError:
            self.get_logger().warn(f"zones file not found ({zones_file}); no no-pass zones")

        # ---- io
        self.traj_publisher = self.create_publisher(TrajectoryMsg, "self/cur_traj", 1)
        self.offset_publisher = self.create_publisher(Float64, "debug/planner/target_offset", 1)
        self.cost_publisher = self.create_publisher(Float64, "debug/planner/best_cost", 1)
        self.opp_gap_publisher = self.create_publisher(Float64, "debug/planner/opponent_gap_m", 1)
        self.max_kappa_publisher = self.create_publisher(Float64, "debug/planner/max_curvature", 1)
        self.rejected_publisher = self.create_publisher(Float64, "debug/planner/rejected_fraction", 1)

        self.create_subscription(Odometry, "self/state", self.on_self, 1)
        self.create_subscription(Odometry, "other/state", self.on_other, 1)
        self.create_subscription(TrackedObjectsMsg, p("tracks_topic"), self.on_tracks, 1)
        self.create_subscription(Int8, p("health_topic"), self.on_health, 1)
        self.n_opp_publisher = self.create_publisher(Float64, "debug/planner/num_opponents", 1)
        self.state_publisher = self.create_publisher(Int8, "debug/planner/state", 1)

        self.lock = Lock()
        self.self_odom = None
        self.other_odom = None
        self.other_stamp = None

        self.prev_target = 0.0
        self.prev_plan = None  # (s_array, d_array) of the last published path

        self.timer = self.create_timer(1.0 / float(p("frequency")), self.plan)

    # ------------------------------------------------------------------ callbacks
    def on_self(self, msg):
        with self.lock:
            self.self_odom = msg

    def on_other(self, msg):
        with self.lock:
            self.other_odom = msg
            self.other_stamp = self.get_clock().now()

    def on_health(self, msg):
        self.health = int(msg.data)

    def on_tracks(self, msg):
        with self.lock:
            self.tracks_msg = msg
            self.tracks_stamp = self.get_clock().now()

    def pass_side(self, s_o, d_o):
        """Side to pass an opponent at track position s_o, lateral offset d_o."""
        if self.side_policy in ("left", "right", "any"):
            return self.side_policy
        wl, wr = self.track.width_at(s_o)
        room_left = float(wl) - d_o
        room_right = float(wr) + d_o
        return "left" if room_left >= room_right else "right"

    def collect_opponents(self):
        """
        Every buggy we know about as (s, d, along-track speed, side-to-pass).
        Tracks from the perception tracker first; NAND's radio estimate is added
        only if no track already covers that position.
        """
        with self.lock:
            tmsg, tstamp = self.tracks_msg, self.tracks_stamp
            omsg, ostamp = self.other_odom, self.other_stamp
        now = self.get_clock().now()
        out = []
        positions = []
        if tmsg is not None and tstamp is not None and (now - tstamp).nanoseconds * 1e-9 <= self.tracks_timeout:
            for i in range(len(tmsg.ids)):
                x, y = tmsg.easting[i], tmsg.northing[i]
                s_o, d_o = self.track.frenet(x, y)
                h = float(self.track.heading_at(s_o))
                v_along = tmsg.vx[i] * np.cos(h) + tmsg.vy[i] * np.sin(h)
                out.append((s_o, d_o, max(float(v_along), 0.0), self.pass_side(s_o, d_o)))
                positions.append((x, y))
        if omsg is not None and ostamp is not None and (now - ostamp).nanoseconds * 1e-9 <= self.opp_timeout:
            x, y = omsg.pose.pose.position.x, omsg.pose.pose.position.y
            if not any(np.hypot(x - px, y - py) < self.dedupe_d for px, py in positions):
                s_o, d_o = self.track.frenet(x, y)
                ov = float(np.hypot(omsg.twist.twist.linear.x, omsg.twist.twist.linear.y))
                out.append((s_o, d_o, ov, self.pass_side(s_o, d_o)))
        return out

    # ------------------------------------------------------------------ planning
    def plan(self):
        with self.lock:
            ego = self.self_odom
        if ego is None:
            return

        ex, ey = ego.pose.pose.position.x, ego.pose.pose.position.y
        ev = float(np.hypot(ego.twist.twist.linear.x, ego.twist.twist.linear.y))
        s_e, d_e = self.track.frenet(ex, ey)

        # every opponent we know about, in the track frame
        opponents = self.collect_opponents()
        self.n_opp_publisher.publish(Float64(data=float(len(opponents))))

        # local path support
        s0 = s_e + self.lookahead
        s1 = min(s0 + self.horizon, self.track.length - 1e-3)
        if s1 - s0 < 5.0:
            return  # end of course
        s = np.linspace(s0, s1, self.resolution)
        w_left, w_right = self.track.width_at(s)
        # The hard curvature limit must not blame a candidate for curvature that the
        # reference line itself already has (hand-clicked waypoints can exceed the
        # steering limit locally); otherwise every candidate is rejected and the
        # planner freezes. Limit = the steering cap or the reference's own peak.
        ref_kappa_peak = float(np.max(np.abs(self.track.curvature_at(s))))
        kappa_limit = max(self.kappa_max, ref_kappa_peak * 1.05 + 0.005)
        if self.health == 1:
            w_left = np.maximum(w_left - self.degraded_margin, 0.0)
            w_right = np.maximum(w_right - self.degraded_margin, 0.0)

        # committed starting offset: what we previously published at s0, else where we are
        if self.prev_plan is not None and self.prev_plan[0][0] <= s0 <= self.prev_plan[0][-1]:
            d_start = float(np.interp(s0, self.prev_plan[0], self.prev_plan[1]))
        else:
            d_start = float(np.clip(d_e, -w_right[0], w_left[0]))

        # opponent predictions sampled at the times ego reaches each s: each entry is
        # (predicted s array, lateral offset, side to pass on)
        t_at = (s - s_e) / max(ev, self.min_pred_speed)
        preds = [(s_o + v_o * t_at, d_o, side) for s_o, d_o, v_o, side in opponents]
        if opponents:
            ahead = [s_o - s_e for s_o, _, _, _ in opponents if s_o >= s_e]
            gap = min(ahead) if ahead else max(s_o - s_e for s_o, _, _, _ in opponents)
            self.opp_gap_publisher.publish(Float64(data=float(gap)))

        # ---- behaviour state with hysteresis
        opp_ahead_or_alongside = any(s_o + self.opp_radius > s_e - self.long_window for s_o, _, _, _ in opponents)
        passing_allowed = (self.health == 0) or not self.passing_needs_health_ok
        # no new pass may START inside a no-pass zone (a pass already under way continues)
        in_no_pass = any(a <= s_e <= b for a, b, _ in self.no_pass)
        if in_no_pass:
            passing_allowed = False
        # any lateral change is forbidden while the local path starts inside a hold zone
        in_hold = any(a <= s_e <= b for a, b, _ in self.hold_zones)
        if self.health >= 2:
            self.state = "RACELINE"
        elif self.state == "PASS" and not opp_ahead_or_alongside:
            self.state = "REJOIN"
        elif self.state == "REJOIN" and abs(self.prev_target) < 0.3:
            self.state = "RACELINE"
        state_code = {"RACELINE": 0, "PASS": 1, "REJOIN": 2}[self.state]
        self.state_publisher.publish(Int8(data=state_code))

        # candidate targets on a lateral grid within the local width envelope
        lo = float(np.max(-w_right))
        hi = float(np.min(w_left))
        if hi < lo:
            lo, hi = -w_right.min(), w_left.min()
        targets = np.arange(np.floor(lo / self.offset_step) * self.offset_step,
                            hi + 1e-6, self.offset_step)
        targets = np.clip(targets, lo, hi)
        targets = np.unique(np.concatenate([targets, [0.0, d_start]]))

        best = None
        n_cand = 0
        n_rej = 0
        respect_opponent = True
        for L in self.transition_lengths:
            u = smoothstep5((s - s0) / L)
            for d_end in targets:
                n_cand += 1
                if in_hold and abs(d_end - d_start) > 1e-6:
                    n_rej += 1
                    continue
                d = d_start + (d_end - d_start) * u

                # hard: stay on the road
                if np.any(d > w_left) or np.any(d < -w_right):
                    n_rej += 1
                    continue

                # hard: bad localization -> raceline only; degraded -> no new pass
                if self.health >= 2 and abs(d_end) > 1e-6:
                    continue
                if not passing_allowed and self.state != "PASS" and d_end > 0.3:
                    continue

                # hard: once committed to a pass, do not drift back toward the raceline while an
                # opponent is still ahead or alongside
                if self.state == "PASS" and opp_ahead_or_alongside and abs(d_end) < abs(self.prev_target) - self.offset_step:
                    continue

                # hard: do not drive through any opponent; keep the clearance on the chosen side
                prox_cost = 0.0
                blocked = False
                if preds and respect_opponent:
                    for p_s, p_d, side in preds:
                        alongside = np.abs(s - p_s) < (self.long_window + self.opp_radius)
                        if np.any(alongside):
                            gap = d[alongside] - p_d
                            if side == "left" and np.any(gap < self.lat_clear):
                                blocked = True
                                break
                            if side == "right" and np.any(-gap < self.lat_clear):
                                blocked = True
                                break
                            if side == "any" and np.any(np.abs(gap) < self.lat_clear):
                                blocked = True
                                break
                        # soft: prefer more room when close in s
                        long_gap = np.abs(s - p_s)
                        prox_cost += float(np.sum(np.exp(-long_gap / 10.0) / (0.5 + np.abs(d - p_d))))
                if blocked:
                    n_rej += 1
                    continue

                xy = self.track.cartesian(s, d)
                ds = np.gradient(s)
                dx = np.gradient(xy[:, 0]) / ds
                dy = np.gradient(xy[:, 1]) / ds
                ddx = np.gradient(dx) / ds
                ddy = np.gradient(dy) / ds
                kappa = (dx * ddy - dy * ddx) / np.power(dx * dx + dy * dy, 1.5)

                # hard: steering saturation (relative to what the reference already demands)
                if np.max(np.abs(kappa[2:-2])) > kappa_limit:
                    n_rej += 1
                    continue

                curv_cost = float(np.sum(kappa[2:-2] ** 2) * (s[1] - s[0]))
                # soft: lateral acceleration at the current speed (the buggy cannot brake)
                a_lat = np.abs(kappa[2:-2]) * max(ev, 1.0) ** 2
                a_lat_cost = float(np.mean(np.maximum(a_lat - self.a_lat_max, 0.0) ** 2))
                dev_cost = float(np.mean(d * d))
                margin_cost = float(np.mean(np.exp(-(w_left - d)) + np.exp(-(w_right + d))))
                change_cost = float((d_end - self.prev_target) ** 2)

                cost = (self.w_curv * curv_cost + self.w_dev * dev_cost + self.w_prox * prox_cost
                        + self.w_margin * margin_cost + self.w_change * change_cost
                        + self.w_a_lat * a_lat_cost)

                if best is None or cost < best[0]:
                    best = (cost, d_end, d, xy)

        if best is None and preds:
            # Tier 2: no candidate keeps the full clearance from every opponent (a narrow
            # section, a bad opponent estimate, or simply no room). The buggy cannot brake,
            # so the least-bad option is the road-legal, steerable path that keeps the
            # LARGEST minimum gap to any opponent while alongside; cost breaks ties.
            self.get_logger().warn("no candidate clears every opponent; choosing the path with the largest gap",
                                   throttle_duration_sec=1.0)
            best_key = None
            for L in self.transition_lengths:
                u = smoothstep5((s - s0) / L)
                for d_end in targets:
                    if in_hold and abs(d_end - d_start) > 1e-6:
                        continue
                    d = d_start + (d_end - d_start) * u
                    if np.any(d > w_left) or np.any(d < -w_right):
                        continue
                    if self.health >= 2 and abs(d_end) > 1e-6:
                        continue
                    xy = self.track.cartesian(s, d)
                    ds = np.gradient(s)
                    dx = np.gradient(xy[:, 0]) / ds
                    dy = np.gradient(xy[:, 1]) / ds
                    ddx = np.gradient(dx) / ds
                    ddy = np.gradient(dy) / ds
                    kappa = (dx * ddy - dy * ddx) / np.power(dx * dx + dy * dy, 1.5)
                    if np.max(np.abs(kappa[2:-2])) > kappa_limit:
                        continue
                    clearance = float("inf")
                    prox_cost = 0.0
                    for p_s, p_d, _ in preds:
                        alongside = np.abs(s - p_s) < (self.long_window + self.opp_radius)
                        if np.any(alongside):
                            clearance = min(clearance, float(np.min(np.abs(d[alongside] - p_d))))
                        long_gap = np.abs(s - p_s)
                        prox_cost += float(np.sum(np.exp(-long_gap / 10.0) / (0.5 + np.abs(d - p_d))))
                    cost = (self.w_curv * float(np.sum(kappa[2:-2] ** 2) * (s[1] - s[0]))
                            + self.w_dev * float(np.mean(d * d)) + self.w_prox * prox_cost
                            + self.w_change * float((d_end - self.prev_target) ** 2))
                    key = (-min(clearance, self.lat_clear), cost)
                    if best_key is None or key < best_key:
                        best_key = key
                        best = (cost, d_end, d, xy)

        if best is None:
            # Tier 3: even the road bounds cannot be met from the committed offset (road narrows
            # faster than any transition). Steer back toward the raceline as fast as allowed.
            self.get_logger().warn("no feasible local path; returning toward the raceline", throttle_duration_sec=1.0)
            u = smoothstep5((s - s0) / self.transition_lengths[0])
            d = d_start * (1.0 - u)
            d = np.clip(d, -w_right, w_left)
            xy = self.track.cartesian(s, d)
            best = (float("inf"), 0.0, d, xy)

        cost, d_end, d, xy = best
        ds_ = np.gradient(s)
        dx_ = np.gradient(xy[:, 0]) / ds_
        dy_ = np.gradient(xy[:, 1]) / ds_
        kap = (dx_ * (np.gradient(dy_) / ds_) - dy_ * (np.gradient(dx_) / ds_)) / np.power(dx_ * dx_ + dy_ * dy_, 1.5)
        self.max_kappa_publisher.publish(Float64(data=float(np.max(np.abs(kap[2:-2])))))
        self.rejected_publisher.publish(Float64(data=float(n_rej / max(n_cand, 1))))
        if self.state == "RACELINE" and d_end > 0.3 and opp_ahead_or_alongside:
            self.state = "PASS"
        self.prev_target = float(d_end)
        self.prev_plan = (s, d)

        local = Trajectory(json_filepath=None, positions=xy)
        self.traj_publisher.publish(local.pack(ex, ey))
        self.offset_publisher.publish(Float64(data=float(d_end)))
        self.cost_publisher.publish(Float64(data=float(cost)))


def main(args=None):
    rclpy.init(args=args)
    node = FrenetPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
