#!/usr/bin/env python3
"""
scenario_timeline.py
--------------------
Print a one-line-per-interval timeline of a scenario bag recorded by
run_sim_scenario.sh --bag: which trajectory source the controller used, the
planner status and committed offset, the cross-track error, the closest ghost /
NAND distance and the tracker's live track count. Use it to see WHEN an
excursion or a near miss happened and what the planner and controller were
doing at that moment.

    python3 src/buggy/scripts/debug/scenario_timeline.py --bag /tmp/runs/traffic_bag --step 1.0
    python3 src/buggy/scripts/debug/scenario_timeline.py --bag ... --around-max-xte 8
"""

import argparse
import json
import math
from collections import defaultdict

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def read(bag):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    series = defaultdict(list)
    t0 = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t0 = t_ns if t0 is None else t0
        series[topic].append(((t_ns - t0) * 1e-9, deserialize_message(data, types[topic])))
    return series


def latest(seq, t):
    """Most recent (time, msg) at or before t, or None."""
    out = None
    for ts, msg in seq:
        if ts > t:
            break
        out = msg
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--around-max-xte", type=float, default=0.0,
                    help="if > 0, print at 0.2 s resolution within this many seconds of the XTE peak")
    ap.add_argument("--around-min-ghost", type=float, default=0.0,
                    help="if > 0, print at 0.2 s resolution within this many seconds of the closest ghost approach")
    args = ap.parse_args()
    s = read(args.bag)

    xte = s.get("/SC/controller/controller/debug/cross_track_error", [])
    sc = s.get("/SC/self/state", [])
    nand = s.get("/NAND/self/state", [])
    ghosts = s.get("/SC/perception_sim/ghost_truth", [])
    status = s.get("/SC/planning/status", [])
    source = s.get("/SC/controller/plan_source", [])
    tracking = s.get("/SC/perception/tracking", [])
    steer = s.get("/SC/input/steering", [])
    pstate = s.get("/SC/debug/planner/state", [])
    if not sc:
        raise SystemExit("no /SC/self/state in bag")

    def ghost_distance(t):
        pose = latest(sc, t)
        g = latest(ghosts, t)
        if pose is None or g is None:
            return float("nan")
        x, y = pose.pose.pose.position.x, pose.pose.pose.position.y
        return min((math.hypot(x - p.position.x, y - p.position.y) for p in g.poses), default=float("nan"))

    end = sc[-1][0]
    if args.around_max_xte > 0 and xte:
        peak_t = max(xte, key=lambda p: abs(p[1].data))[0]
        times = list(frange(max(0.0, peak_t - args.around_max_xte), min(end, peak_t + args.around_max_xte), 0.2))
        print(f"XTE peak at t={peak_t:.1f}s; window +/- {args.around_max_xte:.0f}s")
    elif args.around_min_ghost > 0 and ghosts:
        closest_t = min(frange(0.0, end, 0.1), key=lambda t: ghost_distance(t) if not math.isnan(ghost_distance(t)) else 1e9)
        times = list(frange(max(0.0, closest_t - args.around_min_ghost), min(end, closest_t + args.around_min_ghost), 0.2))
        print(f"closest ghost approach at t={closest_t:.1f}s ({ghost_distance(closest_t):.2f} m); window +/- {args.around_min_ghost:.0f}s")
    else:
        times = list(frange(0.0, end, args.step))

    names = {0: "RACE", 1: "PASS", 2: "REJN"}
    print(f"{'t':>6} {'src':<19} {'status':<10} {'st':<4} {'tgt':>6} {'xte':>6} {'steer':>6} {'dNAND':>6} {'dGhost':>7} {'trk':>3}  reasons")
    for t in times:
        pose = latest(sc, t)
        if pose is None:
            continue
        x, y = pose.pose.pose.position.x, pose.pose.pose.position.y
        n = latest(nand, t)
        d_nand = math.hypot(x - n.pose.pose.position.x, y - n.pose.pose.position.y) if n else float("nan")
        st = latest(status, t)
        st = json.loads(st.data) if st else {}
        src = latest(source, t)
        src = json.loads(src.data) if src else {}
        tr = latest(tracking, t)
        e = latest(xte, t)
        sw = latest(steer, t)
        ps = latest(pstate, t)
        print(f"{t:6.1f} {str(src.get('source', '-'))[:19]:<19} {str(st.get('status', '-')):<10} "
              f"{names.get(ps.data if ps else -1, '-'):<4} "
              f"{st.get('target_offset', float('nan')):6.2f} {(e.data if e else float('nan')):6.2f} "
              f"{(sw.data if sw else float('nan')):6.1f} {d_nand:6.2f} {ghost_distance(t):7.2f} "
              f"{(len(tr.tracks) if tr else 0):3d}  {','.join(st.get('reasons', []))[:70]}")


def frange(a, b, step):
    t = a
    while t <= b + 1e-9:
        yield t
        t += step


if __name__ == "__main__":
    main()
