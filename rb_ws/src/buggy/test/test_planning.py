"""Hard checks decide eligibility; preferred checks only downgrade (D1, D3, D9, D11)."""

import re
from pathlib import Path

import numpy as np
import pytest

from racing import planning
from racing.planning import (HardLimits, OpponentPrediction, PreferredLimits, validate_plan,
                             validate_reference)
from util.track import Track

MSG = Path(__file__).resolve().parents[1] / "msg" / "PlanningResultMsg.msg"
HARD = HardLimits(curvature_cap=0.25, half_width=0.6)
PREF = PreferredLimits()


def straight_track(width=4.0, length=200.0):
    return Track(np.array([[0.0, 0.0], [length, 0.0]]), width, width, ds=1.0, smooth=False)


def path_at(track, offset, s0=2.0, s1=62.0, n=150):
    s = np.linspace(s0, s1, n)
    d = np.broadcast_to(np.asarray(offset, dtype=float), s.shape)
    return s, track.cartesian(s, d)


def test_status_constants_match_the_message_definition():
    text = MSG.read_text(encoding="utf-8")
    for name, value in (("NONE", planning.STATUS_NONE), ("ELIGIBLE", planning.STATUS_ELIGIBLE),
                        ("DEGRADED", planning.STATUS_DEGRADED), ("INELIGIBLE", planning.STATUS_INELIGIBLE)):
        assert re.search(rf"uint8 {name}={value}\b", text), name


def test_centreline_on_a_wide_road_is_eligible():
    track = straight_track()
    _, xy = path_at(track, 0.0)
    result = validate_plan(xy, track, HARD, PREF)
    assert result.status == planning.STATUS_ELIGIBLE and result.control_eligible
    assert result.reasons == []
    assert result.max_curvature < 1e-3
    assert result.min_road_margin == pytest.approx(4.0, abs=0.05)
    assert result.samples >= 60 / 0.25


def test_reduced_preferred_margin_downgrades_but_stays_eligible():
    track = straight_track()
    _, xy = path_at(track, 3.7)
    result = validate_plan(xy, track, HARD, PREF)
    assert result.status == planning.STATUS_DEGRADED and result.control_eligible
    assert any(r.startswith("boundary_margin_reduced") for r in result.reasons)


def test_leaving_the_road_is_ineligible():
    track = straight_track()
    _, xy = path_at(track, 4.5)
    result = validate_plan(xy, track, HARD, PREF)
    assert result.status == planning.STATUS_INELIGIBLE and not result.control_eligible
    assert any(r.startswith("leaves_road") for r in result.reasons)


def test_unknown_width_is_never_driven_through():
    track = straight_track()
    track.w_right[:] = np.nan
    _, xy = path_at(track, 0.0)
    result = validate_plan(xy, track, HARD, PREF)
    assert not result.control_eligible
    assert "road_width_unknown" in result.reasons


def test_corridor_narrower_than_the_vehicle_is_ineligible_not_zero_width_feasible():
    track = straight_track()
    track.w_left[:] = -0.6   # 1.0 m road, 2.2 m needed: negative usable width
    track.w_right[:] = -0.6
    _, xy = path_at(track, 0.0)
    result = validate_plan(xy, track, HARD, PREF)
    assert not result.control_eligible
    assert any(r.startswith("leaves_road") for r in result.reasons)


def test_blocked_corridor_publishes_no_eligible_plan():
    track = straight_track()
    _, xy = path_at(track, 0.0)
    blocker = OpponentPrediction(ident=1, station=20.0, offset=0.0, speed=0.0)
    result = validate_plan(xy, track, HARD, PREF, [blocker], ego_station=0.0, ego_speed=10.0)
    assert result.status == planning.STATUS_INELIGIBLE and not result.control_eligible
    assert any(r.startswith("opponent_footprint_1") for r in result.reasons)
    assert result.min_hard_clearance < 0.0


@pytest.mark.parametrize("offset,half_width,expected", [
    (2.5, 0.6, planning.STATUS_ELIGIBLE),      # lateral limit 0.6+0.6+0.2 = 1.4 m: 1.1 m spare
    (1.5, 0.3, planning.STATUS_DEGRADED),      # limit 1.1 m: clear, but under the preferred 1.6 m
    (1.3, 0.6, planning.STATUS_INELIGIBLE),    # limit 1.4 m: footprints overlap by 0.1 m
])
def test_opponent_gap_hard_versus_preferred(offset, half_width, expected):
    track = straight_track()
    _, xy = path_at(track, offset)
    opp = OpponentPrediction(ident=7, station=30.0, offset=0.0, speed=0.0, half_width=half_width)
    result = validate_plan(xy, track, HARD, PREF, [opp], ego_station=0.0, ego_speed=10.0)
    assert result.status == expected
    assert result.control_eligible == (expected != planning.STATUS_INELIGIBLE)
    if expected == planning.STATUS_DEGRADED:
        assert "lateral_clearance_reduced_7" in result.reasons


def test_footprints_are_rectangles_not_circles():
    """Longitudinal separation alone clears two buggies in line; lateral alone clears side by side."""
    track = straight_track()
    _, xy = path_at(track, 0.0)
    opp = OpponentPrediction(ident=1, station=30.0, offset=0.0, speed=0.0)
    gap_in_line = planning.footprint_gap(np.array([3.0]), np.array([0.0]), HARD, opp)
    gap_side = planning.footprint_gap(np.array([0.0]), np.array([1.5]), HARD, opp)
    gap_corner = planning.footprint_gap(np.array([2.0]), np.array([1.0]), HARD, opp)
    assert gap_in_line[0] > 0 and gap_side[0] > 0 and gap_corner[0] < 0
    assert validate_plan(xy, track, HARD, PREF, [opp], 0.0, 10.0).status == planning.STATUS_INELIGIBLE


def test_opponent_uncertainty_widens_its_footprint():
    track = straight_track()
    _, xy = path_at(track, 2.5)
    sharp = OpponentPrediction(ident=1, station=30.0, offset=0.0, speed=0.0, sigma=0.0)
    fuzzy = OpponentPrediction(ident=1, station=30.0, offset=0.0, speed=0.0, sigma=0.6)
    assert validate_plan(xy, track, HARD, PREF, [sharp], 0.0, 10.0).control_eligible
    assert not validate_plan(xy, track, HARD, PREF, [fuzzy], 0.0, 10.0).control_eligible


def test_moving_opponent_is_predicted_at_ego_arrival_time():
    track = straight_track()
    _, xy = path_at(track, 0.0, s0=2.0, s1=120.0, n=300)
    runner = OpponentPrediction(ident=3, station=50.0, offset=0.0, speed=10.0)
    same_speed = validate_plan(xy, track, HARD, PREF, [runner], ego_station=0.0, ego_speed=10.0)
    faster = validate_plan(xy, track, HARD, PREF, [runner], ego_station=0.0, ego_speed=20.0)
    assert same_speed.control_eligible
    assert not faster.control_eligible


def test_every_opponent_is_checked_not_only_the_nearest():
    track = straight_track()
    _, xy = path_at(track, 0.0)
    far_first = [OpponentPrediction(1, 200.0, 3.0, 0.0), OpponentPrediction(2, 40.0, 0.0, 0.0)]
    result = validate_plan(xy, track, HARD, PREF, far_first, 0.0, 10.0)
    assert any(r.startswith("opponent_footprint_2") for r in result.reasons)


def test_curvature_above_the_cap_is_ineligible_with_no_tolerance():
    track = straight_track()
    s = np.linspace(2.0, 62.0, 150)
    d = np.where(s < 10.0, 0.0, 3.0)   # step change: unbounded curvature at the jump
    xy = track.cartesian(s, d)
    result = validate_plan(xy, track, HARD, PREF)
    assert not result.control_eligible
    assert any(r.startswith("curvature_exceeds_cap") for r in result.reasons)
    just_over = HardLimits(curvature_cap=0.25, half_width=0.6)
    gentle = validate_plan(path_at(track, 0.0)[1], track, just_over, PREF)
    assert gentle.control_eligible


def test_between_sample_violations_are_caught_by_dense_sampling():
    track = straight_track()
    s = np.arange(2.0, 62.0, 4.0)          # 4 m waypoints
    d = np.zeros_like(s)
    d[7] = 1.3                             # one displaced waypoint: the spline overshoots between them
    xy = track.cartesian(s, d)
    coarse = validate_plan(xy, track, HardLimits(0.25, 0.6, sample_step=4.0), PREF)
    dense = validate_plan(xy, track, HardLimits(0.25, 0.6, sample_step=0.25), PREF)
    assert dense.max_curvature > coarse.max_curvature
    assert not dense.control_eligible


def test_plan_starting_outside_the_road_may_recover_but_not_stay_out():
    track = straight_track()
    s = np.linspace(2.0, 62.0, 150)

    def smooth_ramp(length):   # 10 m outside a 4 m road, smoothly back to the line over `length` metres
        t = np.clip((s - 2.0) / length, 0.0, 1.0)
        return 10.0 * (1.0 - t * t * t * (t * (t * 6.0 - 15.0) + 10.0))

    recovering = validate_plan(track.cartesian(s, smooth_ramp(20.0)), track, HARD, PREF)
    assert recovering.status == planning.STATUS_DEGRADED and recovering.control_eligible
    assert any(r.startswith("recovering_from_outside_road") for r in recovering.reasons)
    slow = validate_plan(track.cartesian(s, smooth_ramp(55.0)), track, HARD, PREF)   # too long outside
    assert not slow.control_eligible and any(r.startswith("leaves_road") for r in slow.reasons)
    leaves = np.where(s > 40.0, 5.0, 0.0)                                   # starts inside, leaves later
    assert not validate_plan(track.cartesian(s, leaves), track, HARD, PREF).control_eligible


def test_extra_hard_failures_from_the_caller_make_it_ineligible():
    track = straight_track()
    _, xy = path_at(track, 0.0)
    result = validate_plan(xy, track, HARD, PREF, extra_hard_failures=["state_stale"])
    assert result.status == planning.STATUS_INELIGIBLE
    assert "state_stale" in result.reasons


def test_nonfinite_geometry_is_ineligible():
    track = straight_track()
    _, xy = path_at(track, 0.0)
    xy[10, 0] = np.nan
    result = validate_plan(xy, track, HARD, PREF)
    assert not result.control_eligible
    assert result.reasons and "geometry" in result.reasons[0]


def test_reference_validation():
    ok, peak = validate_reference(straight_track(), 0.25)
    assert ok and peak < 1e-3
    a = np.linspace(0.0, np.pi, 40)
    tight = Track(np.c_[2.0 * np.cos(a), 2.0 * np.sin(a)], 4.0, 4.0, ds=0.25)
    ok, peak = validate_reference(tight, 0.25)
    assert not ok and peak > 0.25


def test_vectorised_frenet_matches_scalar_projection():
    a = np.linspace(0.0, np.pi / 2, 60)
    track = Track(np.c_[50.0 * np.cos(a), 50.0 * np.sin(a)], 3.0, 3.0, ds=1.0)
    s = np.array([5.0, 20.5, 40.0])
    d = np.array([1.0, -0.5, 2.0])
    xy = track.cartesian(s, d)
    s_vec, d_vec = track.frenet(xy[:, 0], xy[:, 1])
    for i in range(3):
        s_i, d_i = track.frenet(float(xy[i, 0]), float(xy[i, 1]))
        assert s_vec[i] == pytest.approx(s_i) and d_vec[i] == pytest.approx(d_i)
        assert s_vec[i] == pytest.approx(s[i], abs=0.05) and d_vec[i] == pytest.approx(d[i], abs=0.02)
