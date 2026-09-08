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

Quality filters: --require-rtk keeps only samples taken while the most recent fix
was RTK fixed (or float with --allow-float); --max-std drops samples whose reported
position standard deviation is larger than that. Samples are then thinned to
--spacing metres and lightly smoothed. The tool prints what it kept and the
largest gap between kept samples, so a GNSS dropout is visible, not silent.

Examples (inside the container, in /rb_ws):
    python3 src/buggy/scripts/util/bag_to_course.py --bag bags/center_20260915_101500 \
        --kind centerline --out buggycourse_survey_center.json --require-rtk
    python3 src/buggy/scripts/util/bag_to_course.py --bag bags/left_curb_20260915_104000 \
        --kind left --out buggycourse_survey_left.json --require-rtk
"""

import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from util.track import Track  # noqa: E402


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
    """Return arrays: t, x, y, z, pos_std, fix_type (fix carried forward in time)."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = open_reader(path)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    have_state = state_topic in types
    have_ecef = ecef_topic in types
    if not have_state and not have_ecef:
        raise SystemExit(f"neither {state_topic} nor {ecef_topic} in bag; topics: {sorted(types)}")
    transformer = None
    if not have_state:
        import pyproj
        transformer = pyproj.Transformer.from_crs("epsg:4978", "epsg:32617", always_xy=True)
        print(f"warning: {state_topic} not in bag, converting {ecef_topic} from ECEF")

    msg_types = {name: get_message(tname) for name, tname in types.items()
                 if name in (state_topic, fix_topic, ecef_topic)}
    rows = []
    fix = -1
    counts = {"state": 0, "fix": 0}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == fix_topic:
            m = deserialize_message(data, msg_types[topic])
            fix = int(getattr(m, "fix_type", -1))
            counts["fix"] += 1
        elif topic == state_topic and have_state:
            m = deserialize_message(data, msg_types[topic])
            p = m.pose.pose.position
            c = m.pose.covariance
            std = float(np.sqrt(max(c[0], 0.0) + max(c[7], 0.0)))
            rows.append((t_ns * 1e-9, p.x, p.y, p.z, std, fix))
            counts["state"] += 1
        elif topic == ecef_topic and not have_state:
            m = deserialize_message(data, msg_types[topic])
            p = m.pose.pose.position
            x, y, z = transformer.transform(p.x, p.y, p.z)
            c = m.pose.covariance
            std = float(np.sqrt(max(c[0], 0.0) + max(c[7], 0.0)))
            rows.append((t_ns * 1e-9, x, y, z, std, fix))
            counts["state"] += 1
    if not rows:
        raise SystemExit("no position samples found")
    arr = np.array(rows, dtype=float)
    print(f"read {counts['state']} position samples and {counts['fix']} fix-info messages")
    return arr


def thin_and_smooth(xy, spacing, smooth=5):
    """Keep one sample per `spacing` metres of travel, then moving-average."""
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="bag directory (mcap) or single .mcap/.db3 file")
    ap.add_argument("--kind", required=True, choices=["centerline", "left", "right"])
    ap.add_argument("--out", required=True, help="output waypoint JSON name (relative names go to $TRAJPATH)")
    ap.add_argument("--state-topic", default="/SC/self/state")
    ap.add_argument("--fix-topic", default="/mip/gnss_1/fix_info")
    ap.add_argument("--ecef-topic", default="/ekf/odometry_earth")
    ap.add_argument("--require-rtk", action="store_true", help="keep only RTK-fixed samples")
    ap.add_argument("--allow-float", action="store_true", help="with --require-rtk, also keep RTK-float samples")
    ap.add_argument("--max-std", type=float, default=1.0, help="drop samples with reported position std above this (m)")
    ap.add_argument("--min-speed", type=float, default=0.2, help="drop samples while standing still (m/s)")
    ap.add_argument("--spacing", type=float, default=1.0, help="output waypoint spacing (m)")
    ap.add_argument("--elevation-out", default=None,
                    help="centerline only: CSV of s_m,z_m (default $RBROOT/src/buggy/config/course_elevation.csv)")
    ap.add_argument("--reference", default="buggycourse_sc.json",
                    help="centerline only: reference line used to express elevation by arc length")
    args = ap.parse_args()

    trajpath = os.environ.get("TRAJPATH", "")
    out_path = args.out if os.path.isabs(args.out) else os.path.join(trajpath, args.out)

    arr = read_samples(args.bag, args.state_topic, args.fix_topic, args.ecef_topic)
    t, x, y, z, std, fix = arr.T

    # fix-type histogram before filtering
    vals, cnt = np.unique(fix.astype(int), return_counts=True)
    names = {-1: "unknown", 0: "3D", 1: "2D", 2: "time only", 3: "none", 4: "invalid", 5: "RTK float", 6: "RTK fixed"}
    print("fix types: " + ", ".join(f"{names.get(int(v), v)}={c}" for v, c in zip(vals, cnt)))

    keep = np.ones(len(t), dtype=bool)
    if args.require_rtk:
        good = (fix == 6) | ((fix == 5) if args.allow_float else False)
        keep &= good
    keep &= std <= args.max_std
    # speed from consecutive samples
    dt = np.diff(t, prepend=t[0])
    dt[dt <= 0] = np.nan
    dist = np.hypot(np.diff(x, prepend=x[0]), np.diff(y, prepend=y[0]))
    speed = np.nan_to_num(dist / dt, nan=0.0)
    keep &= speed >= args.min_speed
    kept = int(keep.sum())
    print(f"kept {kept} of {len(t)} samples after filters")
    if kept < 10:
        raise SystemExit("too few samples left; relax --require-rtk/--max-std or check the bag")

    xy = np.c_[x[keep], y[keep]]
    gaps = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    print(f"largest gap between kept samples: {gaps.max():.1f} m (at sample {int(gaps.argmax())}); "
          f"total length {gaps.sum():.0f} m")
    if gaps.max() > 10.0:
        print("WARNING: gap over 10 m, the line will be straight across it; check where the fix dropped")

    line, _ = thin_and_smooth(xy, args.spacing)
    Track.save_waypoints_latlon(line, out_path)
    print(f"wrote {len(line)} waypoints to {out_path}")

    if args.kind == "centerline":
        ref = Track.from_files(os.path.join(trajpath, args.reference), ds=1.0)
        zk = z[keep]
        s_list, z_list = [], []
        for (px, py), pz in zip(xy, zk):
            s, _ = ref.frenet(px, py)
            s_list.append(s)
            z_list.append(pz)
        s_arr = np.array(s_list)
        z_arr = np.array(z_list)
        order = np.argsort(s_arr)
        s_arr, z_arr = s_arr[order], z_arr[order]
        # bin to 5 m and median-filter to kill GNSS altitude noise
        bins = np.arange(0.0, ref.length + 5.0, 5.0)
        centers, zs = [], []
        for a, b in zip(bins[:-1], bins[1:]):
            m = (s_arr >= a) & (s_arr < b)
            if m.sum() >= 3:
                centers.append(0.5 * (a + b))
                zs.append(float(np.median(z_arr[m])))
        elev_out = args.elevation_out or os.path.join(os.environ.get("RBROOT", "/rb_ws"), "src/buggy/config/course_elevation.csv")
        with open(elev_out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["s_m", "z_m"])
            for c, zz in zip(centers, zs):
                w.writerow([f"{c:.1f}", f"{zz:.2f}"])
        print(f"wrote elevation profile ({len(centers)} points) to {elev_out}; "
              f"total rise {max(zs) - zs[0]:.1f} m, net {zs[-1] - zs[0]:+.1f} m")
        print("next: add 'elevation_profile: config/course_elevation.csv' to config/course_zones.yaml")

    summary = {
        "bag": args.bag, "kind": args.kind, "samples_read": int(len(t)), "samples_kept": kept,
        "length_m": float(gaps.sum()), "largest_gap_m": float(gaps.max()), "waypoints": int(len(line)),
        "require_rtk": bool(args.require_rtk),
    }
    print("SUMMARY " + json.dumps(summary))


if __name__ == "__main__":
    main()
