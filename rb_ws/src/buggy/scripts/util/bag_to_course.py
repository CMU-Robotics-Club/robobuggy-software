#!/usr/bin/env python3
"""
bag_to_course.py
----------------
Turn a course-survey recording (see launch/record_course.xml and
docs/course_survey_checklist.md) into the files the stack needs:

    --kind centerline   the driven line as a waypoint JSON, plus the elevation
                        profile along it (config/course_elevation.csv: s_m, z_m)
    --kind left         the left limit (walked along the curb) as a waypoint JSON
    --kind right        the right limit as a waypoint JSON

Input topics (recorded by record_course.xml):
    /SC/self/state          nav_msgs/Odometry, UTM easting/northing/altitude, the
                            same message the controller uses, so what you record is
                            what the buggy believes
    /mip/gnss_1/fix_info    microstrain MipGnssFixInfo, fix_type 6 = RTK fixed,
                            5 = RTK float; used by --require-rtk
Falls back to /ekf/odometry_earth (ECEF) converted with pyproj if self/state is
missing from the bag.

Fix freshness: a position sample gets the fix type of the most recent fix-info
message, but only if that message is at most --fix-max-age seconds older than the
sample; otherwise its fix is "unknown" and --require-rtk drops it. A single
RTK-fixed status at the start of a bag therefore never labels a whole pass.

Quality filters: --require-rtk keeps only samples taken while the fix was RTK
fixed (or float with --allow-float); --max-std drops samples whose reported
position standard deviation is larger than that; --min-speed drops samples taken
while standing still. Kept samples are thinned to --spacing metres and smoothed
with a --smooth-window moving average (1 disables).

Gaps: where consecutive kept samples are more than --max-gap-m apart the output
is split into segments (<out>_seg1.json, <out>_seg2.json, ...) instead of drawing
a straight line across the dropout; with --fail-on-gap the tool exits non-zero
and writes nothing.

Physical offsets: the bag contains the INS position while the buggy was driven or
walked, not the curb. --edge-offset-m moves the line laterally along the local
path normal (toward the left for --kind left, toward the right for --kind right)
and --lever-arm-m moves it along the direction of travel (positive: the reference
wheel is that far ahead of the INS). Both default to 0.0, in which case the
output IS the driven/walked reference line; the manifest then records
calibration_unknown: true. Measure the offsets and pass them explicitly.

Outputs are written atomically (temporary file in the destination directory,
renamed into place only after every check passed) together with
<out stem>.manifest.json recording the bag, topics, sample counts, fix histogram,
gaps, segments, offsets, the --reference file name and its SHA-256 (so zones,
boundaries and elevation can be checked against one versioned line) and the
tool arguments.

Examples (inside the container, in /rb_ws):
    python3 src/buggy/scripts/util/bag_to_course.py --bag bags/center_20260915_101500 \
        --kind centerline --out buggycourse_survey_center.json --require-rtk
    python3 src/buggy/scripts/util/bag_to_course.py --bag bags/left_curb_20260915_104000 \
        --kind left --out buggycourse_survey_left.json --require-rtk --edge-offset-m 0.3
"""

import argparse
import csv
import functools
import glob
import hashlib
import json
import os
import sys
from datetime import datetime, timezone

import numpy as np

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from util.track import Track  # noqa: E402

UNKNOWN_FIX = -1
RTK_FLOAT = 5
RTK_FIXED = 6
FIX_NAMES = {UNKNOWN_FIX: "unknown", 0: "3D", 1: "2D", 2: "time only", 3: "none", 4: "invalid",
             RTK_FLOAT: "RTK float", RTK_FIXED: "RTK fixed"}
LATERAL_SIGN = {"left": 1.0, "right": -1.0, "centerline": 0.0}
UNCALIBRATED_NOTE = ("--edge-offset-m and --lever-arm-m are both 0: this is the driven/walked "
                     "reference line (the INS track), NOT the curb or road edge")


# --------------------------------------------------------------------------- bag input
def open_reader(path):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    storage = "mcap"
    if os.path.isdir(path):
        if not glob.glob(os.path.join(path, "*.mcap")) and glob.glob(os.path.join(path, "*.db3")):
            storage = "sqlite3"
    elif path.endswith(".db3"):
        storage = "sqlite3"
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id=storage), ConverterOptions("", ""))
    return reader


def read_samples(path, state_topic, fix_topic, ecef_topic):
    """
    Read the bag once.

    Returns (samples, fixes, topics_used): samples is an (N, 5) float array of
    t, x, y, z, pos_std (bag receipt time in seconds, UTM metres); fixes is an
    (M, 2) float array of t, fix_type. Fix types are attached to samples later by
    assign_fix_types(), which is where the freshness rule lives.
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = open_reader(path)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    have_state = state_topic in types
    if not have_state and ecef_topic not in types:
        raise SystemExit(f"neither {state_topic} nor {ecef_topic} in bag; topics: {sorted(types)}")
    transformer = None
    if not have_state:
        import pyproj
        transformer = pyproj.Transformer.from_crs("epsg:4978", "epsg:32617", always_xy=True)
        print(f"warning: {state_topic} not in bag, converting {ecef_topic} from ECEF")
    pos_topic = state_topic if have_state else ecef_topic
    topics_used = {"position": pos_topic, "fix": fix_topic if fix_topic in types else None}
    if topics_used["fix"] is None:
        print(f"warning: {fix_topic} not in bag, every sample has unknown fix type")

    msg_types = {name: get_message(tname) for name, tname in types.items()
                 if name in (pos_topic, fix_topic)}
    rows, fixes = [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == fix_topic:
            m = deserialize_message(data, msg_types[topic])
            fixes.append((t_ns * 1e-9, int(getattr(m, "fix_type", UNKNOWN_FIX))))
        elif topic == pos_topic:
            m = deserialize_message(data, msg_types[topic])
            p = m.pose.pose.position
            x, y, z = (p.x, p.y, p.z) if transformer is None else transformer.transform(p.x, p.y, p.z)
            c = m.pose.covariance
            std = float(np.sqrt(max(c[0], 0.0) + max(c[7], 0.0)))
            rows.append((t_ns * 1e-9, x, y, z, std))
    if not rows:
        raise SystemExit("no position samples found")
    print(f"read {len(rows)} position samples and {len(fixes)} fix-info messages")
    return np.array(rows, dtype=float), np.array(fixes, dtype=float).reshape(-1, 2), topics_used


# --------------------------------------------------------------------------- pure helpers
def assign_fix_types(sample_t, fix_t, fix_values, max_age):
    """
    Fix type per sample: the most recent fix-info message at or before the sample,
    provided it is at most max_age seconds old; UNKNOWN_FIX otherwise. A fix received
    long before a sample (or no fix at all) never labels it, whatever the bag order.
    """
    sample_t = np.asarray(sample_t, dtype=float)
    out = np.full(len(sample_t), UNKNOWN_FIX, dtype=int)
    fix_t = np.asarray(fix_t, dtype=float)
    if len(fix_t) == 0:
        return out
    order = np.argsort(fix_t, kind="stable")
    fix_t = fix_t[order]
    fix_values = np.asarray(fix_values)[order].astype(int)
    idx = np.searchsorted(fix_t, sample_t, side="right") - 1
    has_prev = idx >= 0
    age = np.full(len(sample_t), np.inf)
    age[has_prev] = sample_t[has_prev] - fix_t[idx[has_prev]]
    fresh = has_prev & (age <= max_age)
    out[fresh] = fix_values[idx[fresh]]
    return out


def fix_histogram(fix_types):
    """{name: count} of fix types; unknown, RTK float and RTK fixed are always listed."""
    hist = {FIX_NAMES[k]: 0 for k in (UNKNOWN_FIX, RTK_FLOAT, RTK_FIXED)}
    vals, cnt = np.unique(np.asarray(fix_types, dtype=int), return_counts=True)
    for v, c in zip(vals, cnt):
        hist[FIX_NAMES.get(int(v), str(int(v)))] = int(c)
    return hist


def select_samples(t, xy, std, fix, require_rtk=False, allow_float=False, max_std=1.0, min_speed=0.2):
    """Boolean mask of the samples that pass the quality filters."""
    t = np.asarray(t, dtype=float)
    xy = np.asarray(xy, dtype=float)
    fix = np.asarray(fix, dtype=int)
    keep = np.ones(len(t), dtype=bool)
    if require_rtk:
        good = fix == RTK_FIXED
        if allow_float:
            good |= fix == RTK_FLOAT
        keep &= good
    keep &= np.asarray(std, dtype=float) <= max_std
    # speed from consecutive raw samples; the first one has no predecessor and counts as stopped
    dt = np.diff(t, prepend=t[0])
    dist = np.linalg.norm(np.diff(xy, axis=0, prepend=xy[:1]), axis=1)
    speed = np.zeros(len(t))
    moving = dt > 0
    speed[moving] = dist[moving] / dt[moving]
    keep &= speed >= min_speed
    return keep


def split_segments(xy, max_gap):
    """
    Split a polyline where consecutive points are more than max_gap metres apart.

    Returns (segments, gaps): segments is a list of (start, stop) index ranges into
    xy, gaps a list of {"after_sample", "gap_m"} for every split.
    """
    xy = np.asarray(xy, dtype=float)
    if len(xy) == 0:
        return [], []
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    breaks = np.nonzero(step > max_gap)[0]
    segments, start = [], 0
    for b in breaks:
        segments.append((start, int(b) + 1))
        start = int(b) + 1
    segments.append((start, len(xy)))
    gaps = [{"after_sample": int(b), "gap_m": float(step[b])} for b in breaks]
    return segments, gaps


def thin_and_smooth(xy, spacing, smooth=5):
    """Keep one sample per `spacing` metres of travel, then moving-average (odd window, 1 = off)."""
    if smooth < 1 or smooth % 2 == 0:
        raise ValueError("smoothing window must be 1 or an odd number")
    xy = np.asarray(xy, dtype=float)
    keep = [0]
    acc = 0.0
    for i in range(1, len(xy)):
        acc += float(np.linalg.norm(xy[i] - xy[i - 1]))
        if acc >= spacing:
            keep.append(i)
            acc = 0.0
    out = xy[keep]
    if smooth > 1 and len(out) > smooth:
        k = np.ones(smooth) / smooth
        pad = smooth // 2
        sm = np.vstack([
            np.convolve(np.pad(out[:, 0], (pad, pad), mode="edge"), k, mode="valid"),
            np.convolve(np.pad(out[:, 1], (pad, pad), mode="edge"), k, mode="valid"),
        ]).T
        sm[0], sm[-1] = out[0], out[-1]
        out = sm
    return out, keep


def path_frames(xy):
    """
    Unit tangent (direction of travel, i.e. sample order) and left unit normal at
    every point of a polyline. Left = tangent rotated 90 degrees counter-clockwise,
    the same convention as util/track.py.
    """
    xy = np.asarray(xy, dtype=float)
    if len(xy) < 2:
        tangent = np.tile([1.0, 0.0], (len(xy), 1))
    else:
        dx = np.gradient(xy[:, 0])
        dy = np.gradient(xy[:, 1])
        norm = np.hypot(dx, dy)
        norm[norm == 0] = 1.0
        tangent = np.c_[dx / norm, dy / norm]
    normal = np.c_[-tangent[:, 1], tangent[:, 0]]
    return tangent, normal


def apply_offsets(xy, kind, edge_offset=0.0, lever_arm=0.0):
    """
    Move the line by the physical offsets between the INS and the reference point.

    edge_offset (m) is applied along the local left normal: +edge_offset for kind
    "left" (toward the curb side), -edge_offset for "right", nothing for
    "centerline". lever_arm (m) is applied along the direction of travel: positive
    means the reference wheel is that far ahead of the INS.
    """
    xy = np.asarray(xy, dtype=float)
    tangent, normal = path_frames(xy)
    return xy + LATERAL_SIGN[kind] * edge_offset * normal + lever_arm * tangent


def elevation_profile(s, z, length, bin_m=5.0, min_samples=3):
    """Median altitude per bin_m of arc length (kills GNSS altitude noise); (centers, medians)."""
    s = np.asarray(s, dtype=float)
    z = np.asarray(z, dtype=float)
    edges = np.arange(0.0, length + bin_m, bin_m)
    centers, medians = [], []
    for a, b in zip(edges[:-1], edges[1:]):
        m = (s >= a) & (s < b)
        if m.sum() >= min_samples:
            centers.append(0.5 * (a + b))
            medians.append(float(np.median(z[m])))
    return np.array(centers), np.array(medians)


def sha256_of_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def reference_record(ref_path):
    """Name, path and SHA-256 of the reference line file (hash None if it does not exist)."""
    exists = os.path.isfile(ref_path)
    return {"name": os.path.basename(ref_path), "path": os.path.abspath(ref_path),
            "sha256": sha256_of_file(ref_path) if exists else None}


def build_manifest(bag, kind, topics, samples_read, samples_kept, histogram, gaps, segments,
                   edge_offset, lever_arm, reference, arguments, outputs):
    """The JSON written next to the waypoint file; every key here is what a reviewer needs."""
    calibration_unknown = edge_offset == 0.0 and lever_arm == 0.0
    return {
        "tool": "bag_to_course.py",
        "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "bag": os.path.abspath(bag),
        "kind": kind,
        "topics": topics,
        "samples_read": int(samples_read),
        "samples_kept": int(samples_kept),
        "fix_histogram": histogram,
        "largest_gap_m": max([g["gap_m"] for g in gaps] + [0.0]),
        "gaps": gaps,
        "segments": segments,
        "offsets": {"edge_offset_m": float(edge_offset), "lever_arm_m": float(lever_arm),
                    "lateral_sign": LATERAL_SIGN[kind]},
        "calibration_unknown": calibration_unknown,
        "line_is": ("driven/walked reference line (INS track), NOT the curb" if calibration_unknown
                    else "reference line shifted by the offsets above"),
        "reference": reference,
        "arguments": arguments,
        "outputs": outputs,
    }


def write_json(obj, path):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)
        f.write("\n")


def write_elevation_csv(centers, medians, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["s_m", "z_m"])
        for c, zz in zip(centers, medians):
            w.writerow([f"{c:.1f}", f"{zz:.2f}"])


def write_atomically(outputs):
    """
    outputs: list of (final_path, write_fn). Each write_fn(tmp_path) writes one file
    to a temporary name in the destination directory. Only after every writer
    succeeded are the temporaries renamed into place with os.replace(), so a
    failure part-way leaves neither partial files nor a half-updated set.
    """
    temps = []
    committed = False
    try:
        for final, write_fn in outputs:
            directory = os.path.dirname(os.path.abspath(final))
            os.makedirs(directory, exist_ok=True)
            tmp = os.path.join(directory, f".{os.path.basename(final)}.{os.getpid()}.tmp")
            temps.append((tmp, final))      # tracked before writing so a failed writer's file is removed too
            write_fn(tmp)
        for tmp, final in temps:
            os.replace(tmp, final)
        committed = True
    finally:
        if not committed:
            for tmp, _ in temps:
                if os.path.exists(tmp):
                    os.remove(tmp)
    return [final for final, _ in outputs]


# --------------------------------------------------------------------------- main
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="bag directory (mcap) or single .mcap/.db3 file")
    ap.add_argument("--kind", required=True, choices=sorted(LATERAL_SIGN))
    ap.add_argument("--out", required=True, help="output waypoint JSON name (relative names go to $TRAJPATH)")
    ap.add_argument("--state-topic", default="/SC/self/state")
    ap.add_argument("--fix-topic", default="/mip/gnss_1/fix_info")
    ap.add_argument("--ecef-topic", default="/ekf/odometry_earth")
    ap.add_argument("--require-rtk", action="store_true", help="keep only RTK-fixed samples")
    ap.add_argument("--allow-float", action="store_true", help="with --require-rtk, also keep RTK-float samples")
    ap.add_argument("--fix-max-age", type=float, default=2.0,
                    help="a fix-info message older than this (s) no longer labels a sample; its fix is unknown")
    ap.add_argument("--max-std", type=float, default=1.0, help="drop samples with reported position std above this (m)")
    ap.add_argument("--min-speed", type=float, default=0.2, help="drop samples while standing still (m/s)")
    ap.add_argument("--spacing", type=float, default=1.0, help="output waypoint spacing (m)")
    ap.add_argument("--smooth-window", type=int, default=5,
                    help="moving-average window over the thinned waypoints (odd number; 1 disables)")
    ap.add_argument("--max-gap-m", type=float, default=10.0,
                    help="split the output into _segN files where kept samples are farther apart than this (m)")
    ap.add_argument("--fail-on-gap", action="store_true",
                    help="exit non-zero and write nothing if any gap exceeds --max-gap-m")
    ap.add_argument("--edge-offset-m", type=float, default=0.0,
                    help="lateral INS-to-edge offset (m) applied along the path normal: toward the left for "
                         "--kind left, toward the right for --kind right; 0 = the walked line, not the curb")
    ap.add_argument("--lever-arm-m", type=float, default=0.0,
                    help="longitudinal INS-to-reference-wheel offset (m) along the direction of travel, "
                         "positive if the wheel is ahead of the INS")
    ap.add_argument("--elevation-out", default=None,
                    help="centerline only: CSV of s_m,z_m (default $RBROOT/src/buggy/config/course_elevation.csv)")
    ap.add_argument("--reference", default="buggycourse_sc.json",
                    help="reference line (relative names go to $TRAJPATH): elevation s is measured along it and "
                         "its name and SHA-256 are recorded in the manifest")
    args = ap.parse_args(argv)
    if args.smooth_window < 1 or args.smooth_window % 2 == 0:
        ap.error("--smooth-window must be 1 or an odd number")
    if args.fix_max_age <= 0 or args.max_gap_m <= 0 or args.spacing <= 0:
        ap.error("--fix-max-age, --max-gap-m and --spacing must be positive")
    if args.kind == "centerline" and args.edge_offset_m != 0.0:
        ap.error("--edge-offset-m only applies to --kind left/right")
    return args


def main(argv=None):
    args = parse_args(argv)
    trajpath = os.environ.get("TRAJPATH", "")
    out_path = args.out if os.path.isabs(args.out) else os.path.join(trajpath, args.out)
    ref_path = args.reference if os.path.isabs(args.reference) else os.path.join(trajpath, args.reference)

    samples, fixes, topics = read_samples(args.bag, args.state_topic, args.fix_topic, args.ecef_topic)
    t, x, y, z, std = samples.T
    fix = assign_fix_types(t, fixes[:, 0], fixes[:, 1], args.fix_max_age)
    hist = fix_histogram(fix)
    print(f"fix types (fresh within {args.fix_max_age:g} s): " + ", ".join(f"{k}={v}" for k, v in hist.items()))

    xy_all = np.c_[x, y]
    keep = select_samples(t, xy_all, std, fix, require_rtk=args.require_rtk, allow_float=args.allow_float,
                          max_std=args.max_std, min_speed=args.min_speed)
    kept = int(keep.sum())
    print(f"kept {kept} of {len(t)} samples after filters")
    if kept < 10:
        raise SystemExit("too few samples left; relax --require-rtk/--max-std/--fix-max-age or check the bag")

    xy = xy_all[keep]
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    print(f"largest gap between kept samples: {step.max():.1f} m (at sample {int(step.argmax())}); "
          f"total length {step.sum():.0f} m")
    segments, gaps = split_segments(xy, args.max_gap_m)
    for g in gaps:
        print(f"gap of {g['gap_m']:.1f} m after kept sample {g['after_sample']} (limit {args.max_gap_m:g} m)")
    if gaps and args.fail_on_gap:
        raise SystemExit(f"{len(gaps)} gap(s) over --max-gap-m {args.max_gap_m:g} m with --fail-on-gap: nothing written")
    if gaps:
        print(f"splitting the output into {len(segments)} segments; check where the fix dropped and join by hand")

    stem, ext = os.path.splitext(out_path)
    ext = ext or ".json"
    outputs, seg_records = [], []
    for i, (a, b) in enumerate(segments, start=1):
        seg_xy = xy[a:b]
        record = {"index": i, "samples": int(b - a), "file": None, "waypoints": 0,
                  "length_m": float(np.linalg.norm(np.diff(seg_xy, axis=0), axis=1).sum())}
        if len(seg_xy) >= 2:
            line, _ = thin_and_smooth(seg_xy, args.spacing, args.smooth_window)
            line = apply_offsets(line, args.kind, args.edge_offset_m, args.lever_arm_m)
            path = out_path if len(segments) == 1 else f"{stem}_seg{i}{ext}"
            outputs.append((path, functools.partial(Track.save_waypoints_latlon, line)))
            record.update(file=os.path.abspath(path), waypoints=int(len(line)))
        else:
            print(f"segment {i} has a single sample, not written")
        seg_records.append(record)
    if not outputs:
        raise SystemExit("no segment with at least two samples; nothing written")

    if args.kind == "centerline":
        if not os.path.isfile(ref_path):
            raise SystemExit(f"reference line {ref_path} not found; elevation needs it (--reference)")
        ref = Track.from_files(ref_path, ds=1.0)
        s_arr = np.array([ref.frenet(px, py)[0] for px, py in xy])
        centers, medians = elevation_profile(s_arr, z[keep], ref.length)
        if len(centers) < 2:
            raise SystemExit("elevation profile empty: the pass does not run along --reference")
        elev_out = args.elevation_out or os.path.join(os.environ.get("RBROOT", "/rb_ws"),
                                                       "src/buggy/config/course_elevation.csv")
        outputs.append((elev_out, functools.partial(write_elevation_csv, centers, medians)))
        print(f"elevation profile: {len(centers)} points, total rise {medians.max() - medians[0]:.1f} m, "
              f"net {medians[-1] - medians[0]:+.1f} m")
        print("next: add 'elevation_profile: config/course_elevation.csv' to config/course_zones.yaml")

    reference = reference_record(ref_path)
    if reference["sha256"] is None:
        print(f"warning: reference line {ref_path} not found; manifest records no hash")
    manifest_path = stem + ".manifest.json"
    manifest = build_manifest(args.bag, args.kind, topics, len(t), kept, hist, gaps, seg_records,
                              args.edge_offset_m, args.lever_arm_m, reference, vars(args),
                              [p for p, _ in outputs])
    if manifest["calibration_unknown"]:
        print("NOTE: " + UNCALIBRATED_NOTE + "; measure the offsets and re-run to get the physical edge")
    else:
        print(f"offsets applied: edge {args.edge_offset_m:+.2f} m (lateral sign {LATERAL_SIGN[args.kind]:+g}), "
              f"lever arm {args.lever_arm_m:+.2f} m")
    outputs.append((manifest_path, functools.partial(write_json, manifest)))

    for path in write_atomically(outputs):
        print(f"wrote {path}")
    summary = {
        "bag": args.bag, "kind": args.kind, "samples_read": int(len(t)), "samples_kept": kept,
        "length_m": float(step.sum()), "largest_gap_m": float(step.max()), "segments": len(seg_records),
        "waypoints": sum(r["waypoints"] for r in seg_records), "require_rtk": bool(args.require_rtk),
        "fix_histogram": hist, "calibration_unknown": manifest["calibration_unknown"],
    }
    print("SUMMARY " + json.dumps(summary))


if __name__ == "__main__":
    main()
