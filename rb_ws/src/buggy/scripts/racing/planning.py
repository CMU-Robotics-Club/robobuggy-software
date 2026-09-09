"""Plan validation: hard eligibility versus preferred quality (D1, D3, D11).

A candidate becomes control-eligible only when every HARD check passes on the
curve the controller will evaluate: finite geometry, curvature at or below the
vehicle profile's cap (no tolerance), inside the known road with the hard
margin, and outside every observed opponent's footprint with the hard margin.
PREFERRED limits (extra boundary margin, lateral clearance while alongside) can
only downgrade an eligible plan to DEGRADED. Nothing here relaxes a hard limit.
"""

from dataclasses import dataclass, field

import numpy as np

from racing.geometry import ReconstructedCurve

# Mirror of PlanningResultMsg.status constants (kept in sync by test_planning.py).
STATUS_NONE = 0
STATUS_ELIGIBLE = 1
STATUS_DEGRADED = 2
STATUS_INELIGIBLE = 3
STATUS_NAMES = {STATUS_NONE: "NONE", STATUS_ELIGIBLE: "ELIGIBLE",
                STATUS_DEGRADED: "DEGRADED", STATUS_INELIGIBLE: "INELIGIBLE"}


@dataclass(frozen=True)
class HardLimits:
    curvature_cap: float            # 1/m from VehicleProfile.planning_curvature_cap()
    half_width: float               # ego half width (m); Track widths already exclude it
    half_length: float = 1.25       # ego half length (m) along the track
    hard_opponent_margin: float = 0.2   # metres between footprints, always
    opponent_sigma_gain: float = 2.0    # opponent position uncertainty counted at this many sigma
    sample_step: float = 0.25           # metres between validation samples on the spline
    # a plan may START outside the road (the buggy is already there) if it is back inside within
    # this many metres and stays inside; it is then DEGRADED, never silently fine
    recovery_length: float = 25.0
    # opponents are checked where ego arrives within this many seconds; constant-velocity
    # prediction beyond that is not information, and the plan is redone every cycle
    prediction_horizon_s: float = 6.0


@dataclass(frozen=True)
class PreferredLimits:
    road_margin: float = 0.5        # extra metres from the hard road edge we would like to keep
    lateral_clearance: float = 1.6  # centre-to-centre lateral gap wanted while alongside
    longitudinal_window: float = 6.0


@dataclass(frozen=True)
class OpponentPrediction:
    """One observed obstacle, moving along the track at constant speed.

    Footprints are rectangles in track coordinates: buggies are long and narrow and
    travel along the track, so separation is checked along the track (half lengths)
    and across it (half widths) independently; a circle would demand far more lateral
    room than the vehicles need. Lateral motion is a documented model assumption: the
    observed lateral speed decays exponentially with `lateral_decay_s`.
    """
    ident: int
    station: float          # s at the time of the observation
    offset: float           # d (left positive)
    speed: float            # along-track speed (m/s), >= 0
    half_length: float = 1.25   # along the track (m)
    half_width: float = 0.6     # across the track (m)
    sigma: float = 0.0          # position standard deviation (m)
    lateral_speed: float = 0.0  # already dead-banded by the caller (tracker velocity noise)
    lateral_decay_s: float = 2.0
    max_lateral_drift: float = 1.0   # the predicted sideways move is never larger than this (m)

    def station_at(self, t):
        return self.station + self.speed * np.asarray(t, dtype=float)

    def offset_at(self, t):
        t = np.asarray(t, dtype=float)
        if self.lateral_speed == 0.0:
            return np.full(t.shape, self.offset)
        tau = max(float(self.lateral_decay_s), 1e-3)
        drift = self.lateral_speed * tau * (1.0 - np.exp(-t / tau))
        return self.offset + np.clip(drift, -self.max_lateral_drift, self.max_lateral_drift)


@dataclass
class ValidationResult:
    status: int = STATUS_NONE
    control_eligible: bool = False
    reasons: list = field(default_factory=list)
    max_curvature: float = float("nan")
    min_hard_clearance: float = float("inf")
    min_road_margin: float = float("inf")
    samples: int = 0

    @property
    def status_name(self):
        return STATUS_NAMES[self.status]


def footprint_gap(ds, dd, hard, opp):
    """Signed separation between two track-aligned rectangles, inflated by the hard margin and
    `opponent_sigma_gain` times the opponent's position sigma. >= 0 means clear (in at least one
    axis); < 0 means the footprints overlap."""
    inflate = hard.hard_opponent_margin + hard.opponent_sigma_gain * opp.sigma
    long_limit = hard.half_length + opp.half_length + inflate
    lat_limit = hard.half_width + opp.half_width + inflate
    return np.maximum(np.abs(ds) - long_limit, np.abs(dd) - lat_limit)


def recovers_into_road(inside, distance, recovery_length):
    """True when the only out-of-road samples are a contiguous run at the START that ends within
    `recovery_length` metres: the vehicle is already outside and the plan brings it back."""
    inside = np.asarray(inside, dtype=bool)
    if inside.all() or not inside.any() or inside[0]:
        return False
    first = int(np.argmax(inside))
    return bool(inside[first:].all() and (distance[first] - distance[0]) <= recovery_length)


def _finish(result, hard_failures, degraded):
    if hard_failures:
        result.status = STATUS_INELIGIBLE
        result.control_eligible = False
        result.reasons = list(dict.fromkeys(hard_failures + degraded))
    elif degraded:
        result.status = STATUS_DEGRADED
        result.control_eligible = True
        result.reasons = list(dict.fromkeys(degraded))
    else:
        result.status = STATUS_ELIGIBLE
        result.control_eligible = True
        result.reasons = []
    return result


def validate_plan(xy, track, hard, preferred, opponents=(), ego_station=0.0, ego_speed=0.0,
                  min_prediction_speed=1.0, extra_hard_failures=(), extra_degraded=()):
    """Validate a published path against hard and preferred limits.

    xy: (n, 2) waypoints exactly as they will be packed for the controller.
    track: util.track.Track whose widths already exclude half width plus the hard margin;
           NaN width means unknown and is never driven through.
    opponents: OpponentPrediction list; every one is checked, none is skipped.
    ego_station/ego_speed: where and how fast ego is now, for arrival-time prediction.
    """
    result = ValidationResult()
    hard_failures = list(extra_hard_failures)
    degraded = list(extra_degraded)

    try:
        curve = ReconstructedCurve(xy)
    except ValueError as exc:
        hard_failures.append(f"nonfinite_or_degenerate_geometry:{exc}")
        return _finish(result, hard_failures, degraded)

    samples = curve.sample(hard.sample_step)
    result.samples = int(len(samples.distance))
    if not (np.isfinite(samples.xy).all() and np.isfinite(samples.curvature).all()):
        hard_failures.append("nonfinite_geometry")
        return _finish(result, hard_failures, degraded)

    result.max_curvature = samples.max_abs_curvature
    if result.max_curvature > hard.curvature_cap:
        hard_failures.append(f"curvature_exceeds_cap:{result.max_curvature:.3f}>{hard.curvature_cap:.3f}")

    # ---- road: hard bounds with unknown-width refusal, preferred margin as a downgrade
    s, d = track.frenet(samples.xy[:, 0], samples.xy[:, 1])
    w_left, w_right = track.width_at(s)
    unknown = ~(np.isfinite(w_left) & np.isfinite(w_right))
    if unknown.any():
        hard_failures.append("road_width_unknown")
    known = ~unknown
    if known.any():
        margin = np.minimum(w_left[known] - d[known], w_right[known] + d[known])
        result.min_road_margin = float(np.min(margin))
        if result.min_road_margin < 0.0:
            if recovers_into_road(margin >= 0.0, samples.distance[known], hard.recovery_length):
                degraded.append(f"recovering_from_outside_road:{result.min_road_margin:.2f}m")
            else:
                hard_failures.append(f"leaves_road:{result.min_road_margin:.2f}m")
        elif result.min_road_margin < preferred.road_margin:
            degraded.append(f"boundary_margin_reduced:{result.min_road_margin:.2f}m")

    # ---- opponents: every observed obstacle, hard footprint gap and preferred lateral gap
    if opponents:
        arrival = (s - ego_station) / max(float(ego_speed), float(min_prediction_speed))
        arrival = np.maximum(arrival, 0.0)
        within = arrival <= hard.prediction_horizon_s
        for opp in opponents:
            s_o = opp.station_at(arrival)
            ds = s - s_o
            dd = d - opp.offset_at(arrival)
            gap = np.where(within, footprint_gap(ds, dd, hard, opp), np.inf)
            min_gap = float(np.min(gap))
            result.min_hard_clearance = min(result.min_hard_clearance, min_gap)
            if min_gap < 0.0:
                hard_failures.append(f"opponent_footprint_{opp.ident}:{min_gap:.2f}m")
            alongside = within & (np.abs(ds) < (preferred.longitudinal_window + opp.half_length))
            if alongside.any() and float(np.min(np.abs(dd[alongside]))) < preferred.lateral_clearance:
                degraded.append(f"lateral_clearance_reduced_{opp.ident}")

    return _finish(result, hard_failures, degraded)


def validate_reference(track, cap, sample_step=0.25):
    """The reference line itself must be drivable; the planner refuses to start otherwise."""
    curve = ReconstructedCurve(track.xy)
    peak = curve.sample(sample_step).max_abs_curvature
    return peak <= cap, peak
