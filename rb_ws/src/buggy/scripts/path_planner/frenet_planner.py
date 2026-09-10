#!/usr/bin/env python3
"""
frenet_planner.py
-----------------
Online local planner for Short Circuit. Samples smooth lateral offsets from the
reference line in the track's Frenet frame, scores them, validates the best one
on the exact curve the controller will follow, and publishes ONE planning
result envelope (buggy/PlanningResultMsg) on planning/result.

Contract (DECISIONS.md D1, D3, D9, D11):
  * A plan is control-eligible only if every HARD check passes on the
    reconstructed spline: finite geometry, curvature at or below the vehicle
    profile's cap (no tolerance), inside the KNOWN road with the hard margin,
    outside every observed opponent's footprint with the hard margin, fresh
    ego state, known localization health that is not BAD.
  * PREFERRED limits (boundary margin, lateral clearance while alongside) only
    downgrade an eligible plan to DEGRADED.
  * When nothing is eligible the envelope still carries the best diagnostic
    candidate (largest opponent gap, or the return toward the reference) with
    status INELIGIBLE and control_eligible false. The controller ignores it.
  * Unknown road width is unknown: never planned through.
  * Every observed opponent takes part in the hard checks. The side-preference
    policy is applied to the nearest `max_joint_opponents`; beyond that the
    envelope sets search_budget_exhausted (a search limit, not an optimality
    claim).
  * Passing rules are symmetric: a maneuver is |d| > maneuver_threshold on
    either side; no new maneuver may START while its transition overlaps a
    no-pass zone; no lateral change at all while the transition overlaps a
    no-lateral-change (pusher) zone; new maneuvers need localization health OK.
    There is no emergency override of these rules.
  * The plan starts from the offset AND slope the previous eligible plan had at
    the new start station, so consecutive plans are continuous; when there is
    no valid previous plan it starts from the measured ego state.

Legacy self/cur_traj is published only when publish_legacy_trajectory is true,
and only for eligible plans. Diagnostics never reach that topic.
"""

import json
import math
import os
from threading import Lock

import numpy as np
import yaml
from scipy.interpolate import BPoly

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int8, String

from util.trajectory import Trajectory
from util.track import Track
from racing.health import stamp_seconds
from racing.planning import (HardLimits, OpponentPrediction, PreferredLimits, STATUS_INELIGIBLE,
                             STATUS_NAMES, footprint_gap, recovers_into_road, validate_plan,
                             validate_reference)
from racing.profile import VehicleProfile, default_profile_path

from buggy.msg import LocalizationHealthMsg, PlanningResultMsg, TrackingResultMsg, TrajectoryMsg


def zones_overlap(zones, s_a, s_b):
    """Names of zones whose [start, end] overlaps [s_a, s_b]."""
    return [name for a, b, name in zones if a <= s_b and b >= s_a]


def quintic_profile(u_norm):
    """Quintic smoothstep on [0, 1]: zero first and second derivative at both ends."""
    t = np.clip(u_norm, 0.0, 1.0)
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def lateral_profile(s, s0, length, d_start, slope_start, d_end):
    """d(s): quintic Hermite from (d_start, slope_start, 0) to (d_end, 0, 0) over `length`, then hold."""
    if abs(slope_start) < 1e-9:
        return d_start + (d_end - d_start) * quintic_profile((s - s0) / length)
    poly = BPoly.from_derivatives([0.0, length], [[d_start, slope_start, 0.0], [d_end, 0.0, 0.0]])
    u = np.clip(s - s0, 0.0, length)
    return poly(u)


def curvature_estimate(kappa_ref, d, s):
    """Fast-stage curvature of the lateral profile d(s) laid over a reference with curvature kappa_ref.

    kappa = kappa_ref / (1 - kappa_ref * d) + d''(s), the standard Frenet approximation for small
    slopes. kappa_ref is the track's smoothed curvature and d is analytic, so this has none of the
    knot spikes a finite difference over the 1 m track samples would produce. The authoritative
    check is still validate_plan() on the reconstructed spline.
    """
    denom = 1.0 - kappa_ref * d
    denom = np.where(np.abs(denom) < 0.2, np.sign(denom) * 0.2 + (denom == 0) * 0.2, denom)
    d1 = np.gradient(d, s)
    d2 = np.gradient(d1, s)
    return kappa_ref / denom + d2


class FrenetPlanner(Node):

    def __init__(self):
        super().__init__("frenet_planner")
        self.get_logger().info("INITIALIZED.")

        # ---- files and profile
        self.declare_parameter("traj_name", "buggycourse_sc.json")
        self.declare_parameter("curb_name", "buggycourse_curb.json")
        self.declare_parameter("right_boundary_name", "")
        # ASSUMED corridor when no boundary file exists; <= 0 means unknown (nothing eligible there)
        self.declare_parameter("left_width", 3.0)
        self.declare_parameter("right_width", 0.5)
        self.declare_parameter("vehicle_profile", "")
        self.declare_parameter("hardware_profile", False)   # refuse unverified limits
        self.declare_parameter("hard_boundary_margin", 0.2)
        self.declare_parameter("boundary_margin", 0.5)       # preferred, on top of the hard margin
        self.declare_parameter("track_ds", 1.0)

        # ---- horizon and sampling
        self.declare_parameter("frequency", 10.0)
        self.declare_parameter("lookahead", 2.0)
        self.declare_parameter("horizon", 60.0)
        self.declare_parameter("resolution", 150)
        self.declare_parameter("offset_step", 0.25)
        self.declare_parameter("transition_lengths", [15.0, 25.0, 40.0])
        self.declare_parameter("maneuver_threshold", 0.3)
        self.declare_parameter("max_lateral_offset", 3.0)     # never target farther from the line than this
        self.declare_parameter("recovery_length", 25.0)       # see racing.planning.HardLimits
        self.declare_parameter("target_window", 30.0)         # metres ahead over which targets must fit
        self.declare_parameter("reuse_min_length", 20.0)      # a previous plan is reused only with this much left
        self.declare_parameter("validate_top_k", 4)
        self.declare_parameter("plan_validity_s", 0.5)
        self.declare_parameter("state_max_age_s", 0.3)

        # ---- opponents
        self.declare_parameter("tracks_topic", "perception/tracking")
        self.declare_parameter("tracks_max_age_s", 1.0)
        self.declare_parameter("require_perception", False)
        self.declare_parameter("opponent_half_length", 1.25)   # used when a track has no extent
        self.declare_parameter("opponent_half_width", 0.6)
        self.declare_parameter("hard_opponent_margin", 0.2)
        self.declare_parameter("opponent_sigma_gain", 2.0)
        self.declare_parameter("lateral_clearance", 1.6)       # preferred
        self.declare_parameter("longitudinal_window", 6.0)
        self.declare_parameter("min_speed_for_prediction", 1.0)
        self.declare_parameter("prediction_horizon_s", 6.0)
        # lateral motion model (documented assumption): tracker lateral speed below the deadband is
        # noise and ignored; above it, the opponent keeps drifting with a 2 s time constant, never
        # farther than max_lateral_drift_m
        self.declare_parameter("lateral_velocity_decay_s", 2.0)
        self.declare_parameter("lateral_velocity_deadband", 0.5)
        self.declare_parameter("lateral_velocity_max_sigma", 0.5)   # ignore lateral speed from tracks less certain than this (m/s)
        self.declare_parameter("max_lateral_drift_m", 1.0)
        self.declare_parameter("max_joint_opponents", 3)
        self.declare_parameter("side_policy", "more_room")     # more_room | left | right | any

        # ---- localization health
        self.declare_parameter("health_topic", "localization/health_stamped")
        self.declare_parameter("require_health", True)
        self.declare_parameter("passing_needs_health_ok", True)
        self.declare_parameter("degraded_extra_margin", 0.5)

        # ---- zones and outputs
        self.declare_parameter("zones_file", "")
        self.declare_parameter("result_topic", "planning/result")
        self.declare_parameter("publish_legacy_trajectory", False)

        # ---- cost weights
        self.declare_parameter("w_curvature", 400.0)
        self.declare_parameter("w_deviation", 1.0)
        self.declare_parameter("w_proximity", 6.0)
        self.declare_parameter("w_change", 10.0)   # per m^2 of change in the committed offset; damps target hopping
        self.declare_parameter("w_margin", 3.0)
        self.declare_parameter("a_lat_max", 4.0)
        self.declare_parameter("w_a_lat", 30.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        trajpath = os.environ["TRAJPATH"]

        # profile: the only source of limits
        self.profile = VehicleProfile.load(p("vehicle_profile") or default_profile_path())
        if bool(p("hardware_profile")):
            self.profile.require_measured(["wheelbase_m", "width_m", "software_steering_clip_deg"])
        self.half_width = self.profile.half_width()
        self.kappa_cap = self.profile.planning_curvature_cap()
        self.get_logger().info(
            f"vehicle profile {self.profile.path}: curvature cap {self.kappa_cap:.3f} 1/m "
            f"(ceiling {self.profile.curvature_ceiling():.3f}), provenance {self.profile.provenance()}"
        )

        hard_margin = float(p("hard_boundary_margin"))
        self.pref_margin = float(p("boundary_margin"))
        left_w = float(p("left_width"))
        right_w = float(p("right_width"))
        right_name = p("right_boundary_name")
        self.track = Track.from_files(
            trajpath + p("traj_name"),
            left_boundary_json=trajpath + p("curb_name") if p("curb_name") else None,
            right_boundary_json=trajpath + right_name if right_name else None,
            default_left=left_w if left_w > 0 else None,
            default_right=right_w if right_w > 0 else None,
            margin=hard_margin + self.half_width,
            ds=float(p("track_ds")),
        )
        self.width_reasons = []
        if self.track.left_source == "assumed":
            self.width_reasons.append("left_width_assumed")
        if self.track.right_source == "assumed":
            self.width_reasons.append("right_width_assumed")
        known = self.track.width_known()
        self.get_logger().info(
            f"track {self.track.length:.0f} m; left {self.track.left_source}, right {self.track.right_source}; "
            f"known width at {100.0 * known.mean():.0f}% of stations; hard widths median "
            f"L {np.nanmedian(self.track.w_left):.2f} R {np.nanmedian(self.track.w_right):.2f} m"
        )
        ok, peak = validate_reference(self.track, self.kappa_cap)
        if not ok:
            raise SystemExit(
                f"reference {p('traj_name')} exceeds the curvature cap: {peak:.3f} > {self.kappa_cap:.3f} 1/m; "
                "fix the reference line (raceline_optimizer.py --kappa-max) before planning"
            )
        self.get_logger().info(f"reference validated: peak curvature {peak:.3f} <= cap {self.kappa_cap:.3f}")

        self.recovery_length = float(p("recovery_length"))
        self.max_lateral = float(p("max_lateral_offset"))
        self.target_window = float(p("target_window"))
        self.reuse_min_length = float(p("reuse_min_length"))
        self.hard = HardLimits(curvature_cap=self.kappa_cap, half_width=self.half_width,
                               half_length=self.profile.half_length(),
                               hard_opponent_margin=float(p("hard_opponent_margin")),
                               opponent_sigma_gain=float(p("opponent_sigma_gain")),
                               recovery_length=self.recovery_length,
                               prediction_horizon_s=float(p("prediction_horizon_s")))
        self.preferred = PreferredLimits(road_margin=self.pref_margin,
                                         lateral_clearance=float(p("lateral_clearance")),
                                         longitudinal_window=float(p("longitudinal_window")))

        self.lookahead = float(p("lookahead"))
        self.horizon = float(p("horizon"))
        self.resolution = int(p("resolution"))
        self.offset_step = float(p("offset_step"))
        self.transition_lengths = [float(v) for v in p("transition_lengths")]
        self.maneuver_threshold = float(p("maneuver_threshold"))
        self.validate_top_k = int(p("validate_top_k"))
        self.plan_validity = float(p("plan_validity_s"))
        self.state_max_age = float(p("state_max_age_s"))
        self.opp_half_length = float(p("opponent_half_length"))
        self.opp_half_width = float(p("opponent_half_width"))
        self.min_pred_speed = float(p("min_speed_for_prediction"))
        self.lat_decay = float(p("lateral_velocity_decay_s"))
        self.lat_deadband = float(p("lateral_velocity_deadband"))
        self.lat_vel_max_sigma = float(p("lateral_velocity_max_sigma"))
        self.max_lat_drift = float(p("max_lateral_drift_m"))
        self.max_joint = int(p("max_joint_opponents"))
        self.tracks_max_age = float(p("tracks_max_age_s"))
        self.require_perception = bool(p("require_perception"))
        self.side_policy = str(p("side_policy")).lower()
        if self.side_policy not in ("more_room", "left", "right", "any"):
            self.get_logger().warn(f"unknown side_policy '{self.side_policy}', using more_room")
            self.side_policy = "more_room"
        self.require_health = bool(p("require_health"))
        self.passing_needs_health_ok = bool(p("passing_needs_health_ok"))
        self.degraded_margin = float(p("degraded_extra_margin"))
        self.w_curv = float(p("w_curvature"))
        self.w_dev = float(p("w_deviation"))
        self.w_prox = float(p("w_proximity"))
        self.w_change = float(p("w_change"))
        self.w_margin = float(p("w_margin"))
        self.a_lat_max = float(p("a_lat_max"))
        self.w_a_lat = float(p("w_a_lat"))
        self.publish_legacy = bool(p("publish_legacy_trajectory"))

        self.no_pass = []
        self.hold_zones = []
        zones_file = p("zones_file") or os.path.join(os.environ.get("RBROOT", "/rb_ws"),
                                                     "src/buggy/config/course_zones.yaml")
        try:
            with open(zones_file, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
            self.no_pass = [(float(z["s_start"]), float(z["s_end"]), z.get("name", ""))
                            for z in cfg.get("no_pass_zones", [])]
            self.hold_zones = [(float(z["s_start"]), float(z["s_end"]), z.get("name", ""))
                               for z in cfg.get("no_lateral_change_zones", [])]
            self.get_logger().info(f"no-pass zones: {[n for _, _, n in self.no_pass]}; "
                                   f"hold zones: {[n for _, _, n in self.hold_zones]}")
        except FileNotFoundError:
            self.get_logger().warn(f"zones file not found ({zones_file}); no zone rules")

        # ---- io
        self.result_publisher = self.create_publisher(PlanningResultMsg, p("result_topic"), 1)
        self.status_publisher = self.create_publisher(String, "planning/status", 1)
        self.traj_publisher = self.create_publisher(TrajectoryMsg, "self/cur_traj", 1) if self.publish_legacy else None
        self.offset_publisher = self.create_publisher(Float64, "debug/planner/target_offset", 1)
        self.cost_publisher = self.create_publisher(Float64, "debug/planner/best_cost", 1)
        self.opp_gap_publisher = self.create_publisher(Float64, "debug/planner/opponent_gap_m", 1)
        self.max_kappa_publisher = self.create_publisher(Float64, "debug/planner/max_curvature", 1)
        self.rejected_publisher = self.create_publisher(Float64, "debug/planner/rejected_fraction", 1)
        self.n_opp_publisher = self.create_publisher(Float64, "debug/planner/num_opponents", 1)
        self.state_publisher = self.create_publisher(Int8, "debug/planner/state", 1)
        self.cycle_ms_publisher = self.create_publisher(Float64, "debug/planner/cycle_ms", 1)

        self.create_subscription(Odometry, "self/state", self.on_self, 1)
        self.create_subscription(TrackingResultMsg, p("tracks_topic"), self.on_tracks, 1)
        self.create_subscription(LocalizationHealthMsg, p("health_topic"), self.on_health, 1)

        self.lock = Lock()
        self.self_odom = None
        self.self_rx = None
        self.tracks_msg = None
        self.health_msg = None

        self.state = "RACELINE"      # RACELINE | PASS | REJOIN, symmetric
        self.prev_target = 0.0
        self.prev_plan = None        # (s, d, eligible, deadline_s) of the last published plan
        self.prev_eligible = None    # (s, d, xy) of the last control-eligible plan, for re-validated reuse
        self.plan_id = 0
        self.timer = self.create_timer(1.0 / float(p("frequency")), self.plan)

    # ------------------------------------------------------------------ callbacks
    def on_self(self, msg):
        with self.lock:
            self.self_odom = msg
            self.self_rx = self.now_s()

    def on_tracks(self, msg):
        with self.lock:
            self.tracks_msg = msg

    def on_health(self, msg):
        with self.lock:
            self.health_msg = msg

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def current_health(self, now):
        """(level, reasons) with unknown/expired health treated as BAD when required."""
        msg = self.health_msg
        if msg is None:
            return (2, ["health_unknown"]) if self.require_health else (0, [])
        if now > stamp_seconds(msg.valid_until):
            return (2, ["health_expired"]) if self.require_health else (int(msg.level), [])
        return int(msg.level), list(msg.reasons)

    def collect_opponents(self, now):
        """Every tracked object as an OpponentPrediction, plus freshness info."""
        msg = self.tracks_msg
        if msg is None:
            return [], None, "no_tracking" if self.require_perception else None
        age = now - stamp_seconds(msg.header.stamp)
        if age > self.tracks_max_age:
            return [], msg.header.stamp, f"tracking_stale:{age:.2f}s" if self.require_perception else None
        if self.require_perception and not msg.perception_ready:
            # hardware: a fresh but sensor-less tracker output is not "nobody around"
            return [], msg.header.stamp, "perception_not_ready:" + ",".join(msg.reasons)
        out = []
        for t in msg.tracks:
            if not t.confirmed:
                continue
            s_o, d_o = self.track.frenet(float(t.position.x), float(t.position.y))
            # the tracker reports the object where it was last seen; advance it ALONG THE ROAD by its
            # age (straight-line extrapolation in UTM is wrong on curves)
            track_age = float(np.clip(now - stamp_seconds(t.last_observed_stamp), 0.0, self.tracks_max_age))
            h = float(self.track.heading_at(s_o))
            tangent = np.array([math.cos(h), math.sin(h)])
            normal = np.array([-tangent[1], tangent[0]])
            vel = np.array([t.velocity.x, t.velocity.y])
            v_along = max(float(vel @ tangent), 0.0)
            v_lat = float(vel @ normal)
            cov = np.asarray(t.covariance, dtype=float).reshape(4, 4)
            sigma = float(math.sqrt(max(cov[0, 0], cov[1, 1], 0.0)))
            sigma_v = float(math.sqrt(max(cov[2, 2], cov[3, 3], 0.0)))
            # lateral speed is used only when the track's velocity is certain enough and clearly
            # above the noise deadband; otherwise the opponent is predicted to hold its line
            v_lat = math.copysign(max(abs(v_lat) - self.lat_deadband, 0.0), v_lat) if sigma_v <= self.lat_vel_max_sigma else 0.0
            half_length, half_width = self.opp_half_length, self.opp_half_width
            if t.extent_known:
                # extents are not oriented; the larger horizontal side is taken as the length
                # along the track and the smaller as the width, never smaller than the defaults
                sides = sorted([float(t.extent.x), float(t.extent.y)])
                half_length = max(half_length, 0.5 * sides[1])
                half_width = max(half_width, 0.5 * sides[0])
            out.append(OpponentPrediction(int(t.id), s_o + v_along * track_age, d_o, v_along, half_length,
                                          half_width, sigma, v_lat, self.lat_decay, self.max_lat_drift))
        return out, msg.header.stamp, None

    def pass_side(self, opp):
        if self.side_policy in ("left", "right", "any"):
            return self.side_policy
        wl, wr = self.track.width_at(opp.station)
        if not (np.isfinite(wl) and np.isfinite(wr)):
            return "any"
        return "left" if (float(wl) - opp.offset) >= (float(wr) + opp.offset) else "right"

    # ------------------------------------------------------------------ planning
    def plan(self):  # pylint: disable=too-many-locals,too-many-branches,too-many-statements
        t_start = self.now_s()
        with self.lock:
            ego = self.self_odom
            ego_rx = self.self_rx
        if ego is None:
            return
        now = self.now_s()
        hard_failures = []
        degraded = list(self.width_reasons)

        state_age = now - ego_rx
        if state_age > self.state_max_age:
            hard_failures.append(f"state_stale:{state_age:.2f}s")
        health, health_reasons = self.current_health(now)
        if health >= 2:
            hard_failures.append("localization_bad:" + ",".join(health_reasons))
        elif health == 1:
            degraded.append("localization_degraded")

        ex, ey = ego.pose.pose.position.x, ego.pose.pose.position.y
        ev = float(np.hypot(ego.twist.twist.linear.x, ego.twist.twist.linear.y))
        heading_e = float(ego.pose.pose.orientation.z)
        if not all(math.isfinite(v) for v in (ex, ey, ev, heading_e)):
            # Indoors or before the first fix the state converter emits NaN / out-of-range UTM.
            # The KD-tree query raises on NaN and took the node down on the buggy (2026-09-09).
            self.publish_no_geometry(ego, hard_failures + ["state_not_finite"], t_start)
            return
        s_e, d_e = self.track.frenet(ex, ey)

        opponents, tracks_stamp, perception_failure = self.collect_opponents(now)
        if perception_failure:
            hard_failures.append(perception_failure)
        self.n_opp_publisher.publish(Float64(data=float(len(opponents))))

        s0 = s_e + self.lookahead
        s1 = min(s0 + self.horizon, self.track.length - 1e-3)
        if s1 - s0 < 5.0:
            return  # end of course
        s = np.linspace(s0, s1, self.resolution)
        kappa_ref = np.asarray(self.track.curvature_at(s), dtype=float)
        w_left_h, w_right_h = self.track.width_at(s)          # hard widths (may be NaN or negative)
        extra = self.degraded_margin if health == 1 else 0.0
        w_left_p = w_left_h - self.pref_margin - extra           # preferred envelope
        w_right_p = w_right_h - self.pref_margin - extra
        known = np.isfinite(w_left_h) & np.isfinite(w_right_h)

        # ---- committed start: offset and slope from the previous eligible, unexpired plan
        prev = self.prev_plan
        if prev is not None and prev[2] and now <= prev[3] and prev[0][0] <= s0 <= prev[0][-1]:
            d_start = float(np.interp(s0, prev[0], prev[1]))
            slope_start = float(np.interp(s0, prev[0], np.gradient(prev[1], prev[0])))
        else:
            d_start = float(d_e)
            slope_start = math.tan(float((heading_e - self.track.heading_at(s_e) + np.pi) % (2 * np.pi) - np.pi))
            slope_start = float(np.clip(slope_start, -0.5, 0.5))

        # ---- opponent timing (checked only where ego arrives within the prediction horizon)
        arrival = np.maximum((s - s_e) / max(ev, self.min_pred_speed), 0.0)
        within = arrival <= self.hard.prediction_horizon_s
        preds = [(o, o.station_at(arrival), o.offset_at(arrival)) for o in opponents]
        # opponents that matter for the maneuver decision: not yet clearly behind, and reachable within
        # the prediction horizon (a buggy 300 m ahead is not a passing situation yet)
        reach = max(ev, self.min_pred_speed) * self.hard.prediction_horizon_s
        ahead = sorted((o for o in opponents
                        if s_e - self.preferred.longitudinal_window < o.station + o.half_length
                        and o.station - s_e <= reach),
                       key=lambda o: o.station)
        budget_exhausted = len(ahead) > self.max_joint
        policy_opps = ahead[:self.max_joint]
        sides = {o.ident: self.pass_side(o) for o in policy_opps}
        if opponents:
            gaps = [o.station - s_e for o in opponents if o.station >= s_e]
            self.opp_gap_publisher.publish(Float64(data=float(min(gaps) if gaps else max(o.station - s_e for o in opponents))))

        # ---- behaviour state (symmetric): RACELINE -> PASS on the first committed maneuver with an
        # opponent in reach; PASS holds side and amplitude while any opponent is in reach; PASS -> REJOIN
        # when none is; REJOIN only shrinks the offset (or re-enters PASS if an opponent turns up)
        opp_ahead_or_alongside = bool(ahead)
        if health >= 2:
            self.state = "RACELINE"
        elif self.state == "PASS" and not opp_ahead_or_alongside:
            self.state = "REJOIN"
        elif self.state == "REJOIN" and abs(self.prev_target) < self.maneuver_threshold:
            self.state = "RACELINE"
        self.state_publisher.publish(Int8(data={"RACELINE": 0, "PASS": 1, "REJOIN": 2}[self.state]))
        passing_allowed = (health == 0) or not self.passing_needs_health_ok

        # ---- candidate targets: must fit the preferred envelope over the next target_window metres
        # (a narrow spot farther ahead is handled by the pass-and-return profile, not by refusing
        # every target now)
        window = known & (s <= s0 + self.target_window)
        if window.any():
            lo = float(np.max(-w_right_p[window]))
            hi = float(np.min(w_left_p[window]))
        else:
            lo, hi = 0.0, 0.0
        lo, hi = max(lo, -self.max_lateral), min(hi, self.max_lateral)
        if hi < lo:
            lo, hi = min(0.0, d_start), max(0.0, d_start)
        targets = np.arange(math.floor(lo / self.offset_step) * self.offset_step, hi + 1e-6, self.offset_step)
        targets = np.unique(np.round(np.concatenate([np.clip(targets, lo, hi), [0.0, d_start]]), 4))
        clear_target = self.preferred.lateral_clearance + 0.5   # no reward for more room than this

        # ---- stage 1: fast numpy screening and scoring of every candidate
        candidates = []       # (cost, d_end, L, d, xy, degraded_flags)
        diagnostics = []      # road-legal, steerable candidates ignoring opponents: (min_gap, cost, ...)
        n_cand = n_rej = 0
        commit_released = False
        for commit in (True, False):
            if not commit:
                # a committed side with no candidate at all releases the commitment: re-decide
                # (reported as commit_released) instead of falling back to the reference
                if candidates or self.state != "PASS":
                    break
                commit_released = True
            for L in self.transition_lengths:
                seg_end = s0 + L
                hold_hit = zones_overlap(self.hold_zones, s0, seg_end)
                nopass_hit = zones_overlap(self.no_pass, s0, seg_end)
                for d_end in targets:
                    n_cand += 1
                    is_maneuver = abs(d_end) > self.maneuver_threshold
                    changes = abs(d_end - d_start) > 1e-3
                    # policy: hold zones forbid any lateral change during the transition
                    if hold_hit and changes:
                        n_rej += 1
                        continue
                    # policy: no NEW maneuver while degraded/bad health or in a no-pass zone
                    if is_maneuver and self.state != "PASS" and (not passing_allowed or nopass_hit):
                        n_rej += 1
                        continue
                    # policy: a committed pass keeps its side and does not drift back toward the reference
                    # while an opponent is still in reach (no mid-pass side switch); while rejoining, the
                    # offset may only shrink unless an opponent is in reach again (then a new pass may start)
                    if commit and self.state == "PASS" and opp_ahead_or_alongside and abs(self.prev_target) > self.maneuver_threshold:
                        if d_end * self.prev_target < 0 or abs(d_end) < abs(self.prev_target) - self.offset_step:
                            n_rej += 1
                            continue
                    if self.state == "REJOIN" and not opp_ahead_or_alongside:
                        if d_end * self.prev_target < 0 or abs(d_end) > abs(self.prev_target) + 1e-6:
                            n_rej += 1
                            continue

                    d = lateral_profile(s, s0, L, d_start, slope_start, d_end)
                    # pass-and-return: if the held offset stops fitting the preferred envelope farther
                    # ahead, come back to the reference line before that point instead of giving up
                    if abs(d_end) > 1e-6:
                        misfit = (s >= s0 + L) & ((d_end > w_left_p) | (d_end < -w_right_p))
                        if misfit.any():
                            s_ret = max(float(s[np.argmax(misfit)]) - L, s0 + L)
                            back = d_end * (1.0 - quintic_profile((s - s_ret) / L))
                            d = np.where(s < s_ret, d, back)
                    # hard (fast form): known road, hard widths; a plan may start outside only to recover
                    if (~known).any():
                        n_rej += 1
                        continue
                    flags = []
                    inside = (d <= w_left_h) & (d >= -w_right_h)
                    if not inside.all():
                        if not recovers_into_road(inside, s, self.recovery_length):
                            n_rej += 1
                            continue
                        flags.append("recovering_from_outside_road")
                    elif np.any(d > w_left_p) or np.any(d < -w_right_p):
                        flags.append("boundary_margin_reduced")
                    xy = self.track.cartesian(s, d)
                    kappa = curvature_estimate(kappa_ref, d, s)
                    if not np.isfinite(kappa).all() or np.max(np.abs(kappa[2:-2])) > self.kappa_cap:
                        n_rej += 1
                        continue
                    curv_cost = float(np.sum(kappa[2:-2] ** 2) * (s[1] - s[0]))
                    a_lat = np.abs(kappa[2:-2]) * max(ev, 1.0) ** 2
                    cost = (self.w_curv * curv_cost
                            + self.w_a_lat * float(np.mean(np.maximum(a_lat - self.a_lat_max, 0.0) ** 2))
                            + self.w_dev * float(np.mean(d * d))
                            + self.w_margin * float(np.mean(np.exp(-(w_left_h - d)) + np.exp(-(w_right_h + d))))
                            + self.w_change * float((d_end - self.prev_target) ** 2))

                    # opponents: hard footprint gap against EVERY opponent, preferred lateral clearance,
                    # side preference for the nearest few
                    min_gap = float("inf")
                    blocked = False
                    prox_cost = 0.0
                    for opp, p_s, p_d in preds:
                        ds_ = s - p_s
                        dd = d - p_d
                        gap = np.where(within, footprint_gap(ds_, dd, self.hard, opp), np.inf)
                        min_gap = min(min_gap, float(np.min(gap)))
                        if float(np.min(gap)) < 0.0:
                            blocked = True
                        alongside = within & (np.abs(ds_) < (self.preferred.longitudinal_window + opp.half_length))
                        if alongside.any():
                            lat = dd[alongside]
                            if float(np.min(np.abs(lat))) < self.preferred.lateral_clearance:
                                flags.append(f"lateral_clearance_reduced_{opp.ident}")
                            side = sides.get(opp.ident)
                            if side == "left" and np.any(lat < 0):
                                prox_cost += 5.0
                            elif side == "right" and np.any(lat > 0):
                                prox_cost += 5.0
                        # soft: want clear_target of lateral room while close in s; saturates, so more room
                        # than that is never rewarded (deviation and curvature then decide)
                        prox_cost += float(np.sum(np.maximum(clear_target - np.abs(dd), 0.0) * np.exp(-np.abs(ds_) / 10.0)))
                    cost += self.w_prox * prox_cost
                    diagnostics.append((min_gap, cost, d_end, L, d, xy))
                    if blocked:
                        n_rej += 1
                        continue
                    candidates.append((cost, d_end, L, d, xy, flags))

        if commit_released:
            degraded.append("commit_released")
        self.rejected_publisher.publish(Float64(data=float(n_rej / max(n_cand, 1))))
        candidates.sort(key=lambda c: c[0])

        # ---- stage 2: authoritative validation on the reconstructed spline, best first
        chosen = None
        result = None
        for cost, d_end, L, d, xy, flags in candidates[:self.validate_top_k]:
            res = validate_plan(xy, self.track, self.hard, self.preferred, opponents, s_e, ev,
                                self.min_pred_speed, hard_failures, degraded + flags)
            if res.control_eligible:
                chosen = (cost, d_end, L, d, xy)
                result = res
                break
            if result is None:
                result = res
                chosen = (cost, d_end, L, d, xy)
        eligible = result is not None and result.control_eligible

        if not eligible and self.prev_eligible is not None:
            # R2.1(5): the previous eligible plan may be reused only after re-validating its remaining
            # segment against the CURRENT state, health, obstacles and constraints
            ps, pd, pxy = self.prev_eligible
            keep = ps >= s_e
            if keep.sum() >= 4 and float(ps[keep][-1] - ps[keep][0]) >= self.reuse_min_length:
                res = validate_plan(pxy[keep], self.track, self.hard, self.preferred, opponents, s_e, ev,
                                    self.min_pred_speed, hard_failures, degraded + ["reusing_previous_plan"])
                if res.control_eligible:
                    chosen = (float("nan"), self.prev_target, self.transition_lengths[0], pd[keep], pxy[keep])
                    s = ps[keep]
                    result = res
                    eligible = True

        if not eligible:
            # diagnostic geometry: the largest-gap steerable candidate, else the return toward the reference
            if diagnostics:
                diagnostics.sort(key=lambda c: (-min(c[0], self.preferred.lateral_clearance), c[1]))
                min_gap, cost, d_end, L, d, xy = diagnostics[0]
                reason = "no_eligible_candidate:max_gap_diagnostic"
            else:
                L = self.transition_lengths[0]
                d = np.clip(lateral_profile(s, s0, L, d_start, slope_start, 0.0),
                            np.nan_to_num(-w_right_h, nan=0.0), np.nan_to_num(w_left_h, nan=0.0))
                xy = self.track.cartesian(s, d)
                cost, d_end = float("inf"), 0.0
                reason = "no_eligible_candidate:return_to_reference_diagnostic"
            chosen = (cost, d_end, L, d, xy)
            result = validate_plan(xy, self.track, self.hard, self.preferred, opponents, s_e, ev,
                                   self.min_pred_speed, hard_failures + [reason], degraded)
            result.status = STATUS_INELIGIBLE
            result.control_eligible = False

        cost, d_end, L, d, xy = chosen
        if eligible:
            if self.state in ("RACELINE", "REJOIN") and abs(d_end) > self.maneuver_threshold and opp_ahead_or_alongside:
                self.state = "PASS"
            self.prev_target = float(d_end)
            self.prev_eligible = (s, d, xy)
        deadline = now + self.plan_validity
        self.prev_plan = (s, d, eligible, deadline)

        # ---- envelope
        self.plan_id += 1
        msg = PlanningResultMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "utm"
        msg.plan_id = int(self.plan_id)
        msg.state_stamp = ego.header.stamp
        if tracks_stamp is not None:
            msg.tracks_stamp = tracks_stamp
        msg.valid_until = (self.get_clock().now() + Duration(seconds=self.plan_validity)).to_msg()
        msg.start_station = float(s[0])
        msg.end_station = float(s[-1])
        local = Trajectory(json_filepath=None, positions=xy)
        msg.trajectory = local.pack(ex, ey)
        msg.status = int(result.status)
        msg.control_eligible = bool(result.control_eligible)
        msg.reasons = [str(r) for r in result.reasons]
        msg.max_curvature = float(result.max_curvature) if math.isfinite(result.max_curvature) else -1.0
        msg.min_hard_clearance = float(result.min_hard_clearance) if math.isfinite(result.min_hard_clearance) else 1e9
        msg.min_road_margin = float(result.min_road_margin) if math.isfinite(result.min_road_margin) else 1e9
        msg.search_budget_exhausted = bool(budget_exhausted)
        self.result_publisher.publish(msg)
        if self.publish_legacy and eligible and self.traj_publisher is not None:
            self.traj_publisher.publish(msg.trajectory)

        cycle_ms = (self.now_s() - t_start) * 1000.0
        self.cycle_ms_publisher.publish(Float64(data=float(cycle_ms)))
        if eligible:
            # the committed offset; diagnostics are not commitments and only appear in planning/status
            self.offset_publisher.publish(Float64(data=float(d_end)))
        self.cost_publisher.publish(Float64(data=float(cost) if math.isfinite(cost) else 1e9))
        self.max_kappa_publisher.publish(Float64(data=msg.max_curvature))
        self.status_publisher.publish(String(data=json.dumps({
            "plan_id": msg.plan_id, "status": STATUS_NAMES[result.status], "control_eligible": msg.control_eligible,
            "reasons": msg.reasons, "state": self.state, "target_offset": float(d_end),
            "max_curvature": msg.max_curvature, "min_hard_clearance": msg.min_hard_clearance,
            "min_road_margin": msg.min_road_margin, "opponents": len(opponents),
            "budget_exhausted": budget_exhausted, "candidates": n_cand, "rejected": n_rej,
            "cycle_ms": round(cycle_ms, 1),
        })))
        if not eligible:
            self.get_logger().warn(f"no eligible plan: {msg.reasons}", throttle_duration_sec=1.0)
        self.get_logger().info(
            f"plan {msg.plan_id} {STATUS_NAMES[result.status]} target {d_end:+.2f} m, {n_cand} candidates, "
            f"{len(opponents)} opponents, cycle {cycle_ms:.1f} ms", throttle_duration_sec=5.0)

    def publish_no_geometry(self, ego, reasons, t_start):
        """INELIGIBLE envelope without a trajectory, for cycles where no plan geometry can exist."""
        self.plan_id += 1
        now_ros = self.get_clock().now()
        msg = PlanningResultMsg()
        msg.header.stamp = now_ros.to_msg()
        msg.header.frame_id = "utm"
        msg.plan_id = int(self.plan_id)
        msg.state_stamp = ego.header.stamp
        msg.valid_until = (now_ros + Duration(seconds=self.plan_validity)).to_msg()
        msg.status = STATUS_INELIGIBLE
        msg.control_eligible = False
        msg.reasons = [str(r) for r in reasons]
        msg.max_curvature = -1.0
        msg.min_hard_clearance = 1e9
        msg.min_road_margin = 1e9
        self.result_publisher.publish(msg)
        cycle_ms = (self.now_s() - t_start) * 1000.0
        self.cycle_ms_publisher.publish(Float64(data=float(cycle_ms)))
        self.status_publisher.publish(String(data=json.dumps({
            "plan_id": msg.plan_id, "status": STATUS_NAMES[STATUS_INELIGIBLE], "control_eligible": False,
            "reasons": msg.reasons, "state": self.state, "target_offset": None, "opponents": 0,
            "cycle_ms": round(cycle_ms, 1),
        })))
        self.get_logger().warn(f"no plan geometry: {msg.reasons}", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = FrenetPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
