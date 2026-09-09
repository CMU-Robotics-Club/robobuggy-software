#!/usr/bin/env python3
"""
raceline_optimizer.py
---------------------
Bounded second-difference curvature smoother (approximate minimum-curvature
line) for the buggy course. Offline tool: reads waypoint JSON, writes waypoint
JSON.

What it is NOT: a minimum-time optimiser. There is no vehicle model, no speed
optimisation and no lap-time objective. It moves the reference line sideways
inside the drivable corridor to reduce the second finite difference of the
line, which approximates curvature for an arc-length parameterised polyline.

Why smaller curvature: the buggy has no motor and the software never brakes,
so the only way to keep speed is to steer as little as possible. Tyre scrub
and the driver's instinct to brake both grow with curvature, and lateral
acceleration is v^2 * kappa. Minimising the integral of kappa^2 along the line,
subject to staying inside the road, is the standard first step used by racing
teams (Heilmeier et al. 2019, "Minimum curvature trajectory planning and
control for an autonomous race car"). This script is a simplified version of
that step, not the full method.

Formulation (one linearisation step):
    line(alpha) = r_i + alpha_i * n_i          r = reference samples, n = left unit normals
    minimise   || W D2 line(alpha) ||^2  +  w_s || D1 alpha ||^2
    subject to -w_right_i <= alpha_i <= w_left_i,   alpha fixed to 0 at both ends
D2 is the second finite difference along arc length, so for an arc-length
parameterised line D2 approximates the curvature vector. This is a bounded
least-squares problem, solved with scipy (no extra dependencies). We re-linearise
a few times: the solution becomes the new reference, widths are re-measured
against the original boundaries, and the problem is solved again.

Known limitation of the linearisation: D2 is taken per original sample, so on
a constant-radius arc the shorter inner line has smaller second differences
and a step drifts inward even though its true curvature is higher. The
re-linearisation and the smoothing term limit but do not remove this. Judge
the result by the printed curvature statistics and the final validation, not
by the objective value.

Speed weighting (optional): --speed-profile takes a JSON list of
{"lat", "lon", "velocity"} (or {"x", "y", "velocity"} in UTM); --zones uses
the gravity speed model. Rows are weighted by W = (v / v_max)^2, so each
squared residual is (v^2 kappa)^2 = v^4 kappa^2 up to the v_max normalisation:
the objective becomes squared lateral acceleration, which is what scrubs speed
off a gravity vehicle. The profile is stored once as v(s) against the arc
length of the initial reference and re-interpolated onto the current line's
stations after every re-linearisation, because resampling changes the sample
count whenever the line gets shorter or longer.

Corridor: the left limit comes from --left-boundary (the mapped curb). The
course has no mapped right boundary, so --right-width is an ASSUMED corridor
to the right of the reference line, not a measured one. Usable room on each
side is the measured width minus the keepout (vehicle_width / 2 + margin).
The script prints the minimum usable right room so a zero-room configuration
is visible: the defaults right_width 0.5 with keepout 1.2 leave no room at all.

Gates before anything is written:
  * the solver must report success with finite offsets at every iteration;
  * max |kappa| of the final line must not exceed --kappa-max (default 0.25 1/m;
    the software steering ceiling tan(20 deg) / 1.104 m is 0.330);
  * the final line must not cross either boundary (clearance >= 0).
On failure the script prints the numbers, exits non-zero and writes nothing.
The output goes to <out>.tmp and is renamed into place, so a crash mid-write
never leaves a truncated waypoint file at <out>.

Usage inside the container (paths relative to $TRAJPATH unless absolute):
    python3 raceline_optimizer.py --center buggycourse_sc.json \
        --left-boundary buggycourse_curb.json --right-width 0.5 \
        --margin 0.6 --vehicle-width 1.2 --out buggycourse_sc_raceline.json
"""

import argparse
import json
import os
import sys

import numpy as np
import utm
from scipy.optimize import lsq_linear
from scipy.spatial import cKDTree

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from util.track import Track  # noqa: E402

# Software steering ceiling: tan(20 deg) over the 1.104 m wheelbase.
STEERING_KAPPA_MAX = float(np.tan(np.radians(20.0)) / 1.104)
# Planning cap, deliberately below the steering ceiling.
DEFAULT_KAPPA_MAX = 0.25


class OptimiserError(RuntimeError):
    """The bounded least-squares solve did not produce a usable step."""


def second_difference(n, ds):
    """(n-2, n) matrix approximating d^2/ds^2 with central differences."""
    D = np.zeros((n - 2, n))
    idx = np.arange(n - 2)
    D[idx, idx] = 1.0
    D[idx, idx + 1] = -2.0
    D[idx, idx + 2] = 1.0
    return D / (ds * ds)


def first_difference(n, ds):
    """(n-1, n) matrix approximating d/ds with forward differences."""
    D = np.zeros((n - 1, n))
    idx = np.arange(n - 1)
    D[idx, idx] = -1.0
    D[idx, idx + 1] = 1.0
    return D / ds


def arc_length(xy):
    """Cumulative arc length (station) along a polyline, starting at 0."""
    xy = np.asarray(xy, dtype=float)
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def max_abs_curvature(xy):
    """Largest |kappa| of a polyline, unsmoothed, measured on its own arc length."""
    xy = np.asarray(xy, dtype=float)
    return float(np.abs(Track._curvature(xy, arc_length(xy), smooth=0)).max())


def curvature_stats(xy, ds):
    s = np.arange(len(xy)) * ds
    k = Track._curvature(xy, s, smooth=5)
    k_abs = np.abs(k[3:-3])
    return {
        "length_m": float(s[-1]),
        "min_radius_m": float(1.0 / max(k_abs.max(), 1e-9)),
        "rms_curvature": float(np.sqrt(np.mean(k_abs ** 2))),
        "sum_k2_ds": float(np.sum(k_abs ** 2) * ds),
        "frac_radius_below_30m": float(np.mean(k_abs > 1.0 / 30.0)),
    }


def load_speed_profile(path, ref_xy):
    """Map a checkpoint list to a per-sample speed array via nearest neighbour."""
    with open(path, "r") as f:
        data = json.load(f)
    if isinstance(data, dict) and "checkpoints" in data:
        data = data["checkpoints"]
    pts, vs = [], []
    for c in data:
        if "lat" in c:
            x, y, _, _ = utm.from_latlon(c["lat"], c["lon"])
        else:
            x, y = c.get("x", c.get("x-pos")), c.get("y", c.get("y-pos"))
        pts.append((x, y))
        vs.append(float(c["velocity"]))
    tree = cKDTree(np.array(pts))
    _, idx = tree.query(ref_xy)
    return np.array(vs)[idx]


def reproject_speeds(s_original, v_original, s_new):
    """
    Speed at each station of the current line.

    The profile is stored once as v(s) against the arc length of the initial
    reference. Every re-linearisation resamples the line, which changes the
    sample count, so the per-row weights have to be rebuilt for the new
    stations rather than reused by index (reusing them by index is what used
    to crash with a shape mismatch). Stations beyond the end of the original
    profile hold its last value (np.interp clamps).
    """
    s_original = np.asarray(s_original, dtype=float)
    v_original = np.asarray(v_original, dtype=float)
    if s_original.shape != v_original.shape:
        raise ValueError(f"speed profile has {v_original.shape[0]} values for "
                         f"{s_original.shape[0]} stations")
    return np.interp(np.asarray(s_new, dtype=float), s_original, v_original)


def row_weights(speeds):
    """
    Weight for each second-difference row: (v / v_max)^2 per sample, averaged
    over the two outer samples of the three-point stencil. Squaring the weighted
    residual gives (v^2 kappa)^2, i.e. squared lateral acceleration.
    """
    speeds = np.asarray(speeds, dtype=float)
    v_max = speeds.max()
    if not np.isfinite(v_max) or v_max <= 0.0:
        raise ValueError("speed profile must be finite with a positive maximum")
    w = (speeds / v_max) ** 2
    return 0.5 * (w[:-2] + w[2:])


def usable_room(ref, left_boundary_xy, right_boundary_xy, keepout):
    """Metres the line may move left / right at each sample once the keepout is removed."""
    w_left = np.maximum(Track.signed_lateral_offset(ref, left_boundary_xy) - keepout, 0.0)
    w_right = np.maximum(-Track.signed_lateral_offset(ref, right_boundary_xy) - keepout, 0.0)
    return w_left, w_right


def optimise(track, left_boundary_xy, right_boundary_xy, half_width, margin,
             smooth_weight=0.2, iterations=3, fixed_ends=3, speeds=None, verbose=True):
    """
    Returns the smoothed line as (N, 2) UTM points at track.ds spacing.

    left_boundary_xy / right_boundary_xy: dense polylines of the true limits.
    half_width + margin is subtracted from the measured widths.
    speeds: optional (len(track.xy),) speed at each initial sample. Kept as
    v(s) and re-interpolated onto every iteration's stations.

    Raises OptimiserError when lsq_linear reports failure or returns
    non-finite offsets; callers must not write anything in that case.
    """
    ds = track.ds
    ref = track.xy.copy()
    keepout = half_width + margin

    speed_profile = None
    if speeds is not None:
        speeds = np.asarray(speeds, dtype=float)
        if speeds.shape != (len(ref),):
            raise ValueError(f"speeds has shape {speeds.shape}, expected ({len(ref)},) "
                             "to match the track samples")
        speed_profile = (arc_length(ref), speeds)

    for it in range(iterations):
        n = len(ref)
        s = np.arange(n) * ds
        dx = np.gradient(ref[:, 0], s)
        dy = np.gradient(ref[:, 1], s)
        norm = np.hypot(dx, dy)
        norm[norm == 0] = 1.0
        normal = np.c_[-dy / norm, dx / norm]

        w_left, w_right = usable_room(ref, left_boundary_xy, right_boundary_xy, keepout)

        D2 = second_difference(n, ds)
        # rows: x'' then y''  ->  A alpha ~ -D2 r
        A_x = D2 @ np.diag(normal[:, 0])
        A_y = D2 @ np.diag(normal[:, 1])
        b_x = -(D2 @ ref[:, 0])
        b_y = -(D2 @ ref[:, 1])

        if speed_profile is not None:
            v_here = reproject_speeds(speed_profile[0], speed_profile[1], arc_length(ref))
            w_rows = row_weights(v_here)
            A_x = A_x * w_rows[:, None]
            A_y = A_y * w_rows[:, None]
            b_x = b_x * w_rows
            b_y = b_y * w_rows

        D1 = first_difference(n, ds) * smooth_weight
        A = np.vstack([A_x, A_y, D1])
        b = np.concatenate([b_x, b_y, np.zeros(n - 1)])

        lb = -w_right
        ub = w_left
        lb[:fixed_ends] = -1e-9
        ub[:fixed_ends] = 1e-9
        lb[-fixed_ends:] = -1e-9
        ub[-fixed_ends:] = 1e-9
        ub = np.maximum(ub, lb + 1e-9)

        res = lsq_linear(A, b, bounds=(lb, ub), method="bvls", lsmr_tol="auto", verbose=0)
        alpha = res.x
        finite = bool(np.all(np.isfinite(alpha)))
        if not res.success or res.status < 0 or not finite:
            raise OptimiserError(f"iteration {it + 1}: lsq_linear did not converge "
                                 f"(success={res.success}, status={res.status}, "
                                 f"finite offsets={finite}): {res.message}")
        new_ref = ref + alpha[:, None] * normal
        new_ref = Track.resample_polyline(new_ref, ds)

        if verbose:
            st = curvature_stats(new_ref, ds)
            print(f"iter {it + 1}: max|alpha|={np.abs(alpha).max():.2f} m  "
                  f"min radius={st['min_radius_m']:.1f} m  sum k^2 ds={st['sum_k2_ds']:.4f}  "
                  f"n={len(new_ref)}")
        ref = new_ref

    return ref


def validate_line(line, left_boundary_xy, right_boundary_xy, kappa_max):
    """
    The acceptance check main() applies before writing anything.

    Returns (ok, report). report holds max_abs_curvature, kappa_max,
    min_left_clearance, min_right_clearance and a list of failure strings
    (empty when ok). Comparisons are written so that NaN fails.
    """
    line = np.asarray(line, dtype=float)
    failures = []
    if line.ndim != 2 or line.shape[0] < 3 or not np.all(np.isfinite(line)):
        failures.append("line has non-finite points or fewer than 3 samples")
        kappa = l_min = r_min = float("nan")
    else:
        kappa = max_abs_curvature(line)
        l_min = float(Track.signed_lateral_offset(line, left_boundary_xy).min())
        r_min = float((-Track.signed_lateral_offset(line, right_boundary_xy)).min())
        if not kappa <= kappa_max:
            failures.append(f"max |curvature| {kappa:.4f} 1/m exceeds kappa-max {kappa_max:.4f} 1/m "
                            f"(min radius {1.0 / max(kappa, 1e-9):.1f} m)")
        if not l_min >= 0.0:
            failures.append(f"line crosses the left limit: min clearance {l_min:.2f} m")
        if not r_min >= 0.0:
            failures.append(f"line crosses the right limit: min clearance {r_min:.2f} m")
    report = {
        "max_abs_curvature": kappa,
        "kappa_max": float(kappa_max),
        "min_left_clearance": l_min,
        "min_right_clearance": r_min,
        "failures": failures,
    }
    return not failures, report


def write_waypoints_atomic(line, out_path):
    """Write to <out>.tmp and rename, so a failure never leaves a partial file at out_path."""
    tmp_path = out_path + ".tmp"
    done = False
    try:
        Track.save_waypoints_latlon(line, tmp_path)
        os.replace(tmp_path, out_path)
        done = True
    finally:
        if not done and os.path.exists(tmp_path):
            os.remove(tmp_path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--center", required=True, help="reference waypoint JSON (e.g. buggycourse_sc.json)")
    ap.add_argument("--left-boundary", default=None, help="left limit waypoint JSON (e.g. buggycourse_curb.json)")
    ap.add_argument("--right-boundary", default=None, help="right limit waypoint JSON, if you have one")
    ap.add_argument("--left-width", type=float, default=3.0, help="fallback drivable metres left of center if no left file")
    ap.add_argument("--right-width", type=float, default=0.5,
                    help="ASSUMED drivable metres right of center when no right file is given")
    ap.add_argument("--vehicle-width", type=float, default=1.2, help="buggy width in metres")
    ap.add_argument("--margin", type=float, default=0.6, help="extra clearance to keep from every boundary, metres")
    ap.add_argument("--ds", type=float, default=2.0, help="sample spacing, metres")
    ap.add_argument("--smooth-weight", type=float, default=0.2, help="penalty on offset change between samples")
    ap.add_argument("--iterations", type=int, default=3)
    ap.add_argument("--speed-profile", default=None,
                    help="optional checkpoint JSON; rows are weighted by (v/vmax)^2 (squared lateral acceleration)")
    ap.add_argument("--zones", default=None, help="course_zones.yaml: weight rows by the gravity speed model v(s)^2")
    ap.add_argument("--kappa-max", type=float, default=DEFAULT_KAPPA_MAX,
                    help="planning curvature cap; the software steering ceiling tan(20 deg)/1.104 m is 0.330")
    ap.add_argument("--out", required=True, help="output waypoint JSON name")
    args = ap.parse_args(argv)

    trajpath = os.environ.get("TRAJPATH", "")

    def resolve(p):
        if p is None:
            return None
        return p if os.path.isabs(p) or os.path.exists(p) else os.path.join(trajpath, p)

    center_xy = Track.load_waypoints_utm(resolve(args.center))
    track = Track(center_xy, args.left_width, args.right_width, ds=args.ds)

    if args.left_boundary:
        left_xy = Track.resample_polyline(Track.load_waypoints_utm(resolve(args.left_boundary)), 0.5)
    else:
        left_xy = track.xy + args.left_width * track.normal
        print(f"note: no left boundary file; --left-width {args.left_width} m is an ASSUMED corridor")
    if args.right_boundary:
        right_xy = Track.resample_polyline(Track.load_waypoints_utm(resolve(args.right_boundary)), 0.5)
    else:
        right_xy = track.xy - args.right_width * track.normal
        print(f"note: no right boundary file; --right-width {args.right_width} m is an ASSUMED corridor")

    speeds = load_speed_profile(resolve(args.speed_profile), track.xy) if args.speed_profile else None
    if args.zones:
        from util.speed_model import SpeedModel
        sm = SpeedModel(args.zones, track)
        speeds = np.array([sm.speed_at(si) for si in track.s])
        print(f"speed model: nominal lap {sm.lap_time():.1f} s, v max {speeds.max():.1f} m/s")

    half_width = args.vehicle_width / 2.0
    keepout = half_width + args.margin
    room_left, room_right = usable_room(track.xy, left_xy, right_xy, keepout)
    print(f"keepout {keepout:.2f} m (half width {half_width:.2f} + margin {args.margin:.2f}); "
          f"usable room after keepout: left min {room_left.min():.2f} m mean {room_left.mean():.2f} m, "
          f"right min {room_right.min():.2f} m max {room_right.max():.2f} m")
    if room_right.max() <= 0.0:
        print("WARNING: zero usable right room at every station; the line can only move left of the reference")

    before = curvature_stats(track.xy, args.ds)
    print("reference:", json.dumps(before))
    try:
        line = optimise(track, left_xy, right_xy, half_width, args.margin,
                        smooth_weight=args.smooth_weight, iterations=args.iterations, speeds=speeds)
    except OptimiserError as exc:
        print(f"ERROR: {exc}; nothing written", file=sys.stderr)
        return 2
    after = curvature_stats(line, args.ds)
    print("optimised:", json.dumps(after))

    ok, report = validate_line(line, left_xy, right_xy, args.kappa_max)
    print(f"final check: max |curvature| {report['max_abs_curvature']:.4f} 1/m "
          f"(cap {args.kappa_max:.4f}, steering ceiling {STEERING_KAPPA_MAX:.3f})  "
          f"clearance to left limit: min {report['min_left_clearance']:.2f} m   "
          f"clearance to right limit: min {report['min_right_clearance']:.2f} m")
    if not ok:
        for failure in report["failures"]:
            print(f"ERROR: {failure}", file=sys.stderr)
        print("validation failed; nothing written", file=sys.stderr)
        return 1
    if min(report["min_left_clearance"], report["min_right_clearance"]) < half_width:
        print(f"WARNING: line centre is within half the vehicle width ({half_width:.2f} m) of a limit; "
              "the body would overhang it")

    out_path = resolve(args.out)
    write_waypoints_atomic(line, out_path)
    print(f"wrote {len(line)} waypoints to {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
