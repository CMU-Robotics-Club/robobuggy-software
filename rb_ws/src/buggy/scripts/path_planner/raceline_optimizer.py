#!/usr/bin/env python3
"""
raceline_optimizer.py
---------------------
Offline minimum-curvature raceline for the buggy course.

Why minimum curvature: the buggy has no motor and the software never brakes,
so the only way to keep speed is to steer as little as possible. Tyre scrub
and the driver's instinct to brake both grow with curvature, and lateral
acceleration is v^2 * kappa. Minimising the integral of kappa^2 along the line,
subject to staying inside the road, is the standard first step used by racing
teams (Heilmeier et al. 2019, "Minimum curvature trajectory planning and
control for an autonomous race car").

Formulation (one linearisation step):
    line(alpha) = r_i + alpha_i * n_i          r = reference samples, n = left unit normals
    minimise   || D2 line(alpha) ||^2  +  w_s || D1 alpha ||^2
    subject to -w_right_i <= alpha_i <= w_left_i,   alpha fixed to 0 at both ends
D2 is the second finite difference along arc length, so for an arc-length
parameterised line D2 approximates the curvature vector. This is a bounded
least-squares problem, solved with scipy (no extra dependencies). We re-linearise
a few times: the solution becomes the new reference, widths are re-measured
against the original boundaries, and the problem is solved again.

Optional speed weighting: pass --speed-profile with a JSON list of
{"lat", "lon", "velocity"} (or {"x", "y", "velocity"} in UTM). Rows are then
weighted by (v / v_max)^2 so fast sections get straighter lines, which is what
matters for a gravity vehicle (scrub ~ v^2 kappa).

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
from scipy.optimize import lsq_linear
from scipy.spatial import cKDTree

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from util.track import Track  # noqa: E402

import utm  # noqa: E402


def second_difference(n, ds):
    """(n-2, n) matrix approximating d^2/ds^2 with central differences."""
    D = np.zeros((n - 2, n))
    idx = np.arange(n - 2)
    D[idx, idx] = 1.0
    D[idx, idx + 1] = -2.0
    D[idx, idx + 2] = 1.0
    return D / (ds * ds)


def first_difference(n, ds):
    D = np.zeros((n - 1, n))
    idx = np.arange(n - 1)
    D[idx, idx] = -1.0
    D[idx, idx + 1] = 1.0
    return D / ds


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


def optimise(track, left_boundary_xy, right_boundary_xy, half_width, margin,
             smooth_weight=0.2, iterations=3, fixed_ends=3, speeds=None, verbose=True):
    """
    Returns the optimised line as (N, 2) UTM points at track.ds spacing.

    left_boundary_xy / right_boundary_xy: dense polylines of the true limits.
    half_width + margin is subtracted from the measured widths.
    """
    ds = track.ds
    ref = track.xy.copy()
    keepout = half_width + margin

    for it in range(iterations):
        n = len(ref)
        s = np.arange(n) * ds
        dx = np.gradient(ref[:, 0], s)
        dy = np.gradient(ref[:, 1], s)
        norm = np.hypot(dx, dy)
        norm[norm == 0] = 1.0
        normal = np.c_[-dy / norm, dx / norm]

        w_left = np.maximum(Track.signed_lateral_offset(ref, left_boundary_xy) - keepout, 0.0)
        w_right = np.maximum(-Track.signed_lateral_offset(ref, right_boundary_xy) - keepout, 0.0)

        D2 = second_difference(n, ds)
        # rows: x'' then y''  ->  A alpha ~ -D2 r
        A_x = D2 @ np.diag(normal[:, 0])
        A_y = D2 @ np.diag(normal[:, 1])
        b_x = -(D2 @ ref[:, 0])
        b_y = -(D2 @ ref[:, 1])

        if speeds is not None:
            w = (speeds / speeds.max()) ** 2
            w_rows = 0.5 * (w[:-2] + w[2:])
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
        new_ref = ref + alpha[:, None] * normal
        new_ref = Track.resample_polyline(new_ref, ds)

        if verbose:
            st = curvature_stats(new_ref, ds)
            print(f"iter {it + 1}: max|alpha|={np.abs(alpha).max():.2f} m  "
                  f"min radius={st['min_radius_m']:.1f} m  sum k^2 ds={st['sum_k2_ds']:.4f}")
        ref = new_ref

    return ref


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--center", required=True, help="reference waypoint JSON (e.g. buggycourse_sc.json)")
    ap.add_argument("--left-boundary", default=None, help="left limit waypoint JSON (e.g. buggycourse_curb.json)")
    ap.add_argument("--right-boundary", default=None, help="right limit waypoint JSON, if you have one")
    ap.add_argument("--left-width", type=float, default=3.0, help="fallback drivable metres left of center if no left file")
    ap.add_argument("--right-width", type=float, default=0.5, help="drivable metres right of center if no right file")
    ap.add_argument("--vehicle-width", type=float, default=1.2, help="buggy width in metres")
    ap.add_argument("--margin", type=float, default=0.6, help="extra clearance to keep from every boundary, metres")
    ap.add_argument("--ds", type=float, default=2.0, help="sample spacing, metres")
    ap.add_argument("--smooth-weight", type=float, default=0.2, help="penalty on offset change between samples")
    ap.add_argument("--iterations", type=int, default=3)
    ap.add_argument("--speed-profile", default=None, help="optional checkpoint JSON to weight fast sections")
    ap.add_argument("--zones", default=None, help="course_zones.yaml: weight rows by the gravity speed model v(s)^2")
    ap.add_argument("--out", required=True, help="output waypoint JSON name")
    args = ap.parse_args()

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
    if args.right_boundary:
        right_xy = Track.resample_polyline(Track.load_waypoints_utm(resolve(args.right_boundary)), 0.5)
    else:
        right_xy = track.xy - args.right_width * track.normal

    speeds = load_speed_profile(resolve(args.speed_profile), track.xy) if args.speed_profile else None
    if args.zones:
        from util.speed_model import SpeedModel
        sm = SpeedModel(args.zones, track)
        speeds = np.array([sm.speed_at(si) for si in track.s])
        print(f"speed model: nominal lap {sm.lap_time():.1f} s, v max {speeds.max():.1f} m/s")

    before = curvature_stats(track.xy, args.ds)
    print("reference:", json.dumps(before))
    line = optimise(track, left_xy, right_xy, args.vehicle_width / 2.0, args.margin,
                    smooth_weight=args.smooth_weight, iterations=args.iterations, speeds=speeds)
    after = curvature_stats(line, args.ds)
    print("optimised:", json.dumps(after))

    # sanity: stay inside the true boundaries (minus half vehicle width)
    l_off = Track.signed_lateral_offset(line, left_xy)
    r_off = -Track.signed_lateral_offset(line, right_xy)
    print(f"clearance to left limit: min {l_off.min():.2f} m   clearance to right limit: min {r_off.min():.2f} m")

    out_path = resolve(args.out)
    Track.save_waypoints_latlon(line, out_path)
    print(f"wrote {len(line)} waypoints to {out_path}")


if __name__ == "__main__":
    main()
