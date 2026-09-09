"""Raceline smoother: speed reprojection, solver and validation gates, atomic write."""

import json
from types import SimpleNamespace

import numpy as np
import pytest
import utm

from path_planner import raceline_optimizer as ro
from util.track import Track

# A point in Pittsburgh, so lat/lon round trips land in the repo's fixed UTM zone 17T.
ORIGIN_UTM = utm.from_latlon(40.4433, -79.9436)[:2]


def semicircle(radius=15.0, n=50):
    a = np.linspace(0, np.pi, n)
    return ORIGIN_UTM + radius * np.c_[np.cos(a), np.sin(a)]


def semicircle_track():
    tr = Track(semicircle(), 5, 5, ds=1)
    return tr, tr.xy + 5 * tr.normal, tr.xy - 5 * tr.normal


def test_weighted_semicircle_completes_when_the_sample_count_changes():
    tr, left, right = semicircle_track()
    keepout = 0.6 + 0.5
    line = ro.optimise(tr, left, right, 0.6, 0.5, iterations=3,
                       speeds=np.full(len(tr.xy), 10.0), verbose=False)
    assert np.all(np.isfinite(line))
    # The regression trigger: the inner line is shorter, so n differs from the reference.
    assert len(line) != len(tr.xy), "resampling no longer changes n; the regression is untested"
    # Stated bound: the per-sample second-difference objective drifts the line onto the
    # inner edge of the corridor (the documented linearisation limitation), and bending
    # back out to the pinned ends adds a few percent over the tightest inscribed circle.
    tightest = 1.0 / (15.0 - (5.0 - keepout))
    assert ro.max_abs_curvature(line) <= 1.05 * tightest
    assert ro.max_abs_curvature(line) <= ro.DEFAULT_KAPPA_MAX
    ok, report = ro.validate_line(line, left, right, ro.DEFAULT_KAPPA_MAX)
    assert ok, report["failures"]
    # The corridor bounds were respected, allowing for the resampling shift.
    assert min(report["min_left_clearance"], report["min_right_clearance"]) >= keepout - 0.1


def test_unweighted_and_weighted_agree_for_a_flat_speed_profile():
    tr, left, right = semicircle_track()
    plain = ro.optimise(tr, left, right, 0.6, 0.5, iterations=2, verbose=False)
    flat = ro.optimise(tr, left, right, 0.6, 0.5, iterations=2,
                       speeds=np.full(len(tr.xy), 7.0), verbose=False)
    assert plain.shape == flat.shape
    assert np.allclose(plain, flat, atol=1e-6)


def test_speed_profile_is_reprojected_by_arc_length_when_n_changes():
    tr, _, _ = semicircle_track()
    s0 = ro.arc_length(tr.xy)
    v0 = np.linspace(5.0, 10.0, len(tr.xy))
    inner = Track.resample_polyline(ORIGIN_UTM + 0.9 * (tr.xy - ORIGIN_UTM), tr.ds)
    assert len(inner) != len(tr.xy)

    s_new = ro.arc_length(inner)
    v_new = ro.reproject_speeds(s0, v0, s_new)
    assert v_new.shape == (len(inner),)
    assert v_new[0] == pytest.approx(v0[0])
    mid = len(inner) // 2
    assert v_new[mid] == pytest.approx(np.interp(s_new[mid], s0, v0))

    w_rows = ro.row_weights(v_new)
    assert w_rows.shape == (len(inner) - 2,)
    d2 = ro.second_difference(len(inner), tr.ds)
    assert (d2 * w_rows[:, None]).shape == d2.shape
    assert w_rows.max() <= 1.0 and w_rows.min() > 0.0

    with pytest.raises(ValueError):
        ro.reproject_speeds(s0, v0[:-1], s_new)
    with pytest.raises(ValueError):
        ro.optimise(tr, tr.xy + 5 * tr.normal, tr.xy - 5 * tr.normal, 0.6, 0.5,
                    speeds=np.ones(len(tr.xy) + 1), verbose=False)


def test_solver_failure_raises_instead_of_returning_a_line(monkeypatch):
    tr, left, right = semicircle_track()

    def failed(a, b, **_kwargs):
        del b
        return SimpleNamespace(x=np.zeros(a.shape[1]), success=False, status=0,
                               message="The maximum number of iterations is exceeded.")

    monkeypatch.setattr(ro, "lsq_linear", failed)
    with pytest.raises(ro.OptimiserError, match="did not converge"):
        ro.optimise(tr, left, right, 0.6, 0.5, verbose=False)

    def not_finite(a, b, **_kwargs):
        del b
        return SimpleNamespace(x=np.full(a.shape[1], np.nan), success=True, status=1, message="ok")

    monkeypatch.setattr(ro, "lsq_linear", not_finite)
    with pytest.raises(ro.OptimiserError, match="finite offsets=False"):
        ro.optimise(tr, left, right, 0.6, 0.5, verbose=False)


def test_validation_rejects_curvature_above_cap_and_boundary_crossings():
    tr, left, right = semicircle_track()
    line = ro.optimise(tr, left, right, 0.6, 0.5, iterations=1, verbose=False)

    ok, report = ro.validate_line(line, left, right, ro.DEFAULT_KAPPA_MAX)
    assert ok and not report["failures"]

    ok, report = ro.validate_line(line, left, right, 0.01)  # a 100 m radius cap on a 15 m arc
    assert not ok
    assert report["max_abs_curvature"] > 0.01
    assert any("kappa-max" in failure for failure in report["failures"])

    ok, report = ro.validate_line(tr.xy + 6 * tr.normal, left, right, ro.DEFAULT_KAPPA_MAX)
    assert not ok and report["min_left_clearance"] < 0
    assert any("left limit" in failure for failure in report["failures"])

    ok, report = ro.validate_line(tr.xy - 6 * tr.normal, left, right, ro.DEFAULT_KAPPA_MAX)
    assert not ok and report["min_right_clearance"] < 0
    assert any("right limit" in failure for failure in report["failures"])

    broken = line.copy()
    broken[len(broken) // 2] = np.nan
    ok, report = ro.validate_line(broken, left, right, ro.DEFAULT_KAPPA_MAX)
    assert not ok


def write_course_files(tmp_path):
    tr, left, _ = semicircle_track()
    center_file = tmp_path / "center.json"
    left_file = tmp_path / "left.json"
    Track.save_waypoints_latlon(semicircle(), str(center_file))
    Track.save_waypoints_latlon(left, str(left_file))
    del tr
    return center_file, left_file


def main_args(center_file, left_file, out_file, **extra):
    args = ["--center", str(center_file), "--left-boundary", str(left_file),
            "--left-width", "5", "--right-width", "5", "--ds", "1", "--iterations", "1",
            "--out", str(out_file)]
    for key, value in extra.items():
        args += [f"--{key.replace('_', '-')}", str(value)]
    return args


def test_main_exits_non_zero_and_writes_nothing_when_the_cap_is_violated(tmp_path, capsys):
    center_file, left_file = write_course_files(tmp_path)
    out_file = tmp_path / "line.json"
    assert ro.main(main_args(center_file, left_file, out_file, kappa_max=0.01)) != 0
    assert not out_file.exists()
    assert not (tmp_path / "line.json.tmp").exists()
    err = capsys.readouterr().err
    assert "exceeds kappa-max" in err and "nothing written" in err


def test_main_exits_non_zero_when_the_solver_fails(tmp_path, monkeypatch):
    center_file, left_file = write_course_files(tmp_path)
    out_file = tmp_path / "line.json"
    monkeypatch.setattr(ro, "lsq_linear", lambda a, b, **_kwargs: SimpleNamespace(
        x=np.zeros(a.shape[1]), success=False, status=-1, message="no progress"))
    assert ro.main(main_args(center_file, left_file, out_file)) == 2
    assert not out_file.exists()


def test_main_writes_a_valid_waypoint_file_when_the_checks_pass(tmp_path, capsys):
    center_file, left_file = write_course_files(tmp_path)
    out_file = tmp_path / "line.json"
    assert ro.main(main_args(center_file, left_file, out_file)) == 0
    assert out_file.exists()
    assert not (tmp_path / "line.json.tmp").exists()
    with open(out_file, "r") as f:
        waypoints = json.load(f)
    assert len(waypoints) > 10
    assert {"key", "lat", "lon", "active"} <= set(waypoints[0])
    out = capsys.readouterr().out
    assert "usable room after keepout" in out and "final check" in out


def test_failed_write_leaves_no_output_file(tmp_path, monkeypatch):
    out_file = tmp_path / "line.json"

    def explode(_xy, path):
        with open(path, "w") as f:
            f.write("[")  # a truncated file at the temp path
        raise OSError("disk full")

    monkeypatch.setattr(Track, "save_waypoints_latlon", staticmethod(explode))
    with pytest.raises(OSError, match="disk full"):
        ro.write_waypoints_atomic(np.zeros((3, 2)), str(out_file))
    assert not out_file.exists()
    assert not (tmp_path / "line.json.tmp").exists()


def test_successful_write_replaces_the_file_and_removes_the_temp(tmp_path):
    out_file = tmp_path / "line.json"
    out_file.write_text("stale")
    ro.write_waypoints_atomic(semicircle(), str(out_file))
    assert not (tmp_path / "line.json.tmp").exists()
    with open(out_file, "r") as f:
        assert len(json.load(f)) == 50
