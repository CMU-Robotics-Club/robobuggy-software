#! /usr/bin/env python3
"""Analyze RTK reception across one or more NAND / SC rosbags.

Usage:
    python rtkReceptionStats.py BAG [BAG ...] [--out DIR] [--show]

Each BAG may be a rosbag2 directory (with metadata.yaml), a bare .mcap/.db3 file,
or any directory tree -- directories are searched recursively for bags. Whether a
bag came from NAND or SC is autodetected from its topics.

NAND: /NAND/debug/raw_gps (NANDRawGPSMsg) carries UTM position + the ublox
      carrier-solution status (rtk_fix: 0 none, 1 float, 2 fixed).
SC:   the MicroStrain GQ7 INS publishes /mip/gnss_{1,2}/fix_info (fix_type: 5 RTK
      float, 6 RTK fixed) and /gnss_{1,2}/llh_position (lat/lon). The two receivers
      are combined with --sc-rtk-mode (default: both must agree).
"""
import argparse
import csv
import sys
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import IntEnum
from pathlib import Path
from zoneinfo import ZoneInfo

import matplotlib.pyplot as plt
import numpy as np
import utm
from matplotlib.lines import Line2D
from rosbags.highlevel import AnyReader


class RtkFix(IntEnum):
    NONE = 0
    FLOAT = 1
    FIXED = 2


FIX_LABEL = {RtkFix.NONE: "no RTK", RtkFix.FLOAT: "RTK float", RtkFix.FIXED: "RTK fixed"}
# status colours (critical / warning / good); shape differs too so colour is never the only cue
FIX_COLOR = {RtkFix.NONE: "#d03b3b", RtkFix.FLOAT: "#fab219", RtkFix.FIXED: "#0ca30c"}
FIX_MARKER = {RtkFix.NONE: "x", RtkFix.FLOAT: "^", RtkFix.FIXED: "o"}

# NAND firmware forwards the ublox carrSoln field as NANDRawGPSMsg.rtk_fix
NAND_RTK_FIX = {0: RtkFix.NONE, 1: RtkFix.FLOAT, 2: RtkFix.FIXED}
NAND_RTK_LABEL = {0: "none", 1: "float", 2: "fixed"}
NAND_RAW_GPS_TOPIC = "/NAND/debug/raw_gps"

# microstrain_inertial_msgs/MipGnssFixInfo.fix_type
SC_FIX_TYPE_LABEL = {0: "3D", 1: "2D", 2: "time only", 3: "none", 4: "invalid", 5: "RTK float", 6: "RTK fixed"}
SC_RECEIVERS = ("gnss_1", "gnss_2")
SC_FIX_TOPIC = {r: f"/mip/{r}/fix_info" for r in SC_RECEIVERS}
SC_POS_TOPIC = {r: f"/{r}/llh_position" for r in SC_RECEIVERS}
# fix_info and llh_position from one receiver share a header stamp; the receivers
# themselves run at 2 Hz and are offset by <100 ms, so this pairs them safely.
SC_PAIR_TOLERANCE_NS = 250_000_000

# Course-local origin, mirrors Constants.UTM_*_ZERO in rb_ws/src/buggy/scripts/util/constants.py
UTM_ZONE_NUM, UTM_ZONE_LETTER = 17, "T"
UTM_EAST_ZERO, UTM_NORTH_ZERO = 589702.87, 4477172.947

BAG_FILE_SUFFIXES = {".mcap", ".db3"}
TIMELINE_GAP_S = 5.0  # a hole in the data longer than this is drawn as a gap, not a fix state
# A buggy that boots without a time source stamps messages near the 1970 epoch until its clock
# syncs. Such samples keep their position and fix status but are excluded from anything time based.
MIN_VALID_TIME_NS = 946_684_800 * 1_000_000_000  # 2000-01-01


def clock_valid(t_ns: int) -> bool:
    return t_ns >= MIN_VALID_TIME_NS


def sc_fix_type_to_rtk(fix_type: int) -> RtkFix:
    if fix_type == 6:
        return RtkFix.FIXED
    if fix_type == 5:
        return RtkFix.FLOAT
    return RtkFix.NONE


@dataclass
class GpsSample:
    t_ns: int  # unix time
    east: float  # course-local metres
    north: float
    fix: RtkFix


@dataclass
class BagResult:
    path: Path
    buggy: str
    start_ns: int
    end_ns: int
    samples: list[GpsSample] = field(default_factory=list)
    raw_counts: dict[str, Counter] = field(default_factory=dict)  # receiver -> Counter(raw fix label)
    warnings: list[str] = field(default_factory=list)

    @property
    def name(self) -> str:
        return self.path.stem if self.path.is_file() else self.path.name

    @property
    def timed_samples(self) -> list[GpsSample]:
        return [s for s in self.samples if clock_valid(s.t_ns)]

    @property
    def first_valid_ns(self) -> int | None:
        ts = self.timed_samples
        return ts[0].t_ns if ts else (self.start_ns if clock_valid(self.start_ns) else None)


# ---------------------------------------------------------------- bag discovery


def find_bags(inputs: list[Path]) -> list[Path]:
    """Expand the CLI inputs into a list of bag directories / bare bag files."""
    found: list[Path] = []

    def walk(d: Path):
        if (d / "metadata.yaml").exists():
            found.append(d)
            return
        children = sorted(p for p in d.iterdir() if not p.name.startswith("."))
        for c in children:
            if c.is_file() and c.suffix in BAG_FILE_SUFFIXES:
                found.append(c)
        for c in children:
            if c.is_dir():
                walk(c)

    for p in inputs:
        if not p.exists():
            print(f"warning: {p} does not exist, skipping", file=sys.stderr)
        elif p.is_file():
            found.append(p)
        else:
            walk(p)
    return found


def bag_reader_paths(bag: Path) -> list[Path]:
    """Paths to hand to AnyReader. Falls back to the bare bag files when metadata.yaml
    references file names that no longer exist (e.g. ':' in the name was replaced by
    '_' when the bag was copied to macOS)."""
    if bag.is_file():
        return [bag]
    try:
        with AnyReader([bag]):
            return [bag]
    except Exception:
        files = sorted(p for p in bag.iterdir() if p.suffix in BAG_FILE_SUFFIXES)
        if not files:
            raise
        return files


def detect_buggy(topics: set[str]) -> str | None:
    if NAND_RAW_GPS_TOPIC in topics:
        return "NAND"
    if any(t in topics for t in SC_FIX_TOPIC.values()):
        return "SC"
    # no GPS topics recorded -- fall back on the namespace so the bag is at least labelled
    if any(t.startswith("/SC/") for t in topics):
        return "SC"
    if any(t.startswith("/NAND/") for t in topics):
        return "NAND"
    return None


# ---------------------------------------------------------------- extraction


def latlon_to_local(lat: float, lon: float) -> tuple[float, float]:
    e, n, _, _ = utm.from_latlon(lat, lon, force_zone_number=UTM_ZONE_NUM, force_zone_letter=UTM_ZONE_LETTER)
    return e - UTM_EAST_ZERO, n - UTM_NORTH_ZERO


def stamp_ns(header) -> int:
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def extract_nand(reader: AnyReader, result: BagResult):
    conns = [c for c in reader.connections if c.topic == NAND_RAW_GPS_TOPIC]
    counts = Counter()
    missing_field = 0
    for conn, t_bag, raw in reader.messages(connections=conns):
        msg = reader.deserialize(raw, conn.msgtype)
        if not hasattr(msg, "rtk_fix"):
            missing_field += 1
            continue
        fix = NAND_RTK_FIX.get(msg.rtk_fix, RtkFix.NONE)
        counts[NAND_RTK_LABEL.get(msg.rtk_fix, f"?{msg.rtk_fix}")] += 1
        result.samples.append(GpsSample(t_bag, msg.easting - UTM_EAST_ZERO, msg.northing - UTM_NORTH_ZERO, fix))
    result.raw_counts["ublox"] = counts
    if missing_field:
        result.warnings.append(f"{missing_field} raw_gps messages use the old NANDRawGPSMsg schema without rtk_fix; skipped")


def _nearest(sorted_ts: np.ndarray, t: int) -> int | None:
    """Index of the entry in sorted_ts nearest to t if within tolerance, else None."""
    if len(sorted_ts) == 0:
        return None
    i = int(np.searchsorted(sorted_ts, t))
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(sorted_ts) and abs(int(sorted_ts[j]) - t) <= SC_PAIR_TOLERANCE_NS:
            if best is None or abs(int(sorted_ts[j]) - t) < abs(int(sorted_ts[best]) - t):
                best = j
    return best


def extract_sc(reader: AnyReader, result: BagResult, rtk_mode: str):
    wanted = set(SC_FIX_TOPIC.values()) | set(SC_POS_TOPIC.values())
    conns = [c for c in reader.connections if c.topic in wanted]
    fixes: dict[str, list[tuple[int, int]]] = {r: [] for r in SC_RECEIVERS}
    positions: dict[str, list[tuple[int, float, float]]] = {r: [] for r in SC_RECEIVERS}
    for conn, t_bag, raw in reader.messages(connections=conns):
        msg = reader.deserialize(raw, conn.msgtype)
        for r in SC_RECEIVERS:
            if conn.topic == SC_FIX_TOPIC[r]:
                fixes[r].append((stamp_ns(msg.header.header), int(msg.fix_type)))
            elif conn.topic == SC_POS_TOPIC[r]:
                positions[r].append((stamp_ns(msg.header), msg.latitude, msg.longitude))

    for r in SC_RECEIVERS:
        result.raw_counts[r] = Counter(SC_FIX_TYPE_LABEL.get(ft, f"?{ft}") for _, ft in fixes[r])

    have = [r for r in SC_RECEIVERS if fixes[r]]
    if not have:
        return
    if rtk_mode in SC_RECEIVERS:
        if rtk_mode not in have:
            result.warnings.append(f"--sc-rtk-mode {rtk_mode} requested but that receiver has no fix_info messages")
            return
        primary, others = rtk_mode, []
    else:
        primary, others = have[0], have[1:]
        if len(have) < 2:
            result.warnings.append(f"only {have[0]} has fix_info messages; combined status uses it alone")

    other_ts = {r: np.array([t for t, _ in fixes[r]], dtype=np.int64) for r in others}
    other_fix = {r: [ft for _, ft in fixes[r]] for r in others}
    pos_ts = {r: np.array([t for t, _, _ in positions[r]], dtype=np.int64) for r in SC_RECEIVERS}
    unmatched_other = unmatched_pos = 0

    for t, ft in fixes[primary]:
        rtk = sc_fix_type_to_rtk(ft)
        for r in others:
            j = _nearest(other_ts[r], t)
            if j is None:
                unmatched_other += 1
                continue
            o = sc_fix_type_to_rtk(other_fix[r][j])
            rtk = max(rtk, o) if rtk_mode == "either" else min(rtk, o)
        east = north = None
        for r in (primary, *others):
            j = _nearest(pos_ts[r], t)
            if j is not None:
                _, lat, lon = positions[r][j]
                east, north = latlon_to_local(lat, lon)
                break
        if east is None:
            unmatched_pos += 1
            continue
        result.samples.append(GpsSample(t, east, north, rtk))

    if unmatched_other:
        result.warnings.append(f"{unmatched_other} fix_info messages had no matching message from the other receiver")
    if unmatched_pos:
        result.warnings.append(f"{unmatched_pos} fix_info messages had no matching llh_position; dropped")


def read_bag(bag: Path, rtk_mode: str, buggy_override: str | None) -> BagResult | None:
    paths = bag_reader_paths(bag)
    with AnyReader(paths) as reader:
        topics = {c.topic for c in reader.connections}
        buggy = buggy_override or detect_buggy(topics)
        if buggy is None:
            print(f"warning: {bag}: cannot tell whether this is a NAND or SC bag, skipping (use --buggy)", file=sys.stderr)
            return None
        result = BagResult(bag, buggy, reader.start_time, reader.end_time)
        if buggy == "NAND":
            extract_nand(reader, result)
        else:
            extract_sc(reader, result, rtk_mode)
    result.samples.sort(key=lambda s: s.t_ns)
    n_bad = len(result.samples) - len(result.timed_samples)
    if n_bad:
        result.warnings.append(f"{n_bad} samples stamped before 2000 (clock not set yet); excluded from time-of-day stats")
    return result


# ---------------------------------------------------------------- statistics


def fix_runs(samples: list[GpsSample]) -> list[tuple[int, int, RtkFix]]:
    """Contiguous (start_ns, end_ns, fix) runs. Data holes longer than TIMELINE_GAP_S split runs."""
    if not samples:
        return []
    dts = np.diff([s.t_ns for s in samples]) if len(samples) > 1 else np.array([500_000_000])
    typical_dt = int(np.median(dts))
    gap_ns = int(TIMELINE_GAP_S * 1e9)
    runs = []
    start = samples[0].t_ns
    for prev, cur in zip(samples, samples[1:]):
        if cur.fix != prev.fix or cur.t_ns - prev.t_ns > gap_ns:
            end = cur.t_ns if cur.t_ns - prev.t_ns <= gap_ns else prev.t_ns + typical_dt
            runs.append((start, end, prev.fix))
            start = cur.t_ns
    runs.append((start, samples[-1].t_ns + typical_dt, samples[-1].fix))
    return runs


def longest_without_fixed_s(samples: list[GpsSample]) -> float:
    longest = cur_start = prev_end = None
    for start, end, fix in fix_runs(samples):
        if fix == RtkFix.FIXED or (prev_end is not None and start != prev_end):
            cur_start = None  # fixed again, or a hole in the data: stop accumulating
        prev_end = end
        if fix == RtkFix.FIXED:
            continue
        if cur_start is None:
            cur_start = start
        span = (end - cur_start) / 1e9
        longest = span if longest is None else max(longest, span)
    return longest or 0.0


def fix_fractions(samples: list[GpsSample]) -> dict[RtkFix, float]:
    n = len(samples)
    counts = Counter(s.fix for s in samples)
    return {f: (counts[f] / n if n else 0.0) for f in RtkFix}


def fmt_fractions(samples: list[GpsSample]) -> str:
    fr = fix_fractions(samples)
    return "  ".join(f"{FIX_LABEL[f]} {100 * fr[f]:5.1f}%" for f in reversed(RtkFix))


def local_dt(t_ns: int, tz: ZoneInfo) -> datetime:
    return datetime.fromtimestamp(t_ns / 1e9, tz=timezone.utc).astimezone(tz)


def print_report(results: list[BagResult], tz: ZoneInfo):
    print("=" * 100)
    for r in results:
        t1 = local_dt(r.end_ns, tz)
        t0 = local_dt(r.first_valid_ns, tz) if r.first_valid_ns is not None else None
        when = f"{t0:%Y-%m-%d %H:%M}-{t1:%H:%M %Z}  ({(r.end_ns - t0_ns) / 1e9:.0f} s)" if (t0_ns := r.first_valid_ns) is not None else "clock never set"
        print(f"{r.name}  [{r.buggy}]  {when}")
        if not r.samples:
            print("  no GPS fix data in this bag")
        else:
            print(f"  {len(r.samples):5d} samples   {fmt_fractions(r.samples)}")
            print(f"  longest stretch without RTK fixed: {longest_without_fixed_s(r.samples):.0f} s")
        for rx, counts in r.raw_counts.items():
            if counts:
                total = sum(counts.values())
                breakdown = ", ".join(f"{k} {100 * v / total:.0f}%" for k, v in counts.most_common())
                print(f"  {rx:6s}: {breakdown}")
        for w in r.warnings:
            print(f"  warning: {w}")
    print("=" * 100)

    all_samples = [s for r in results for s in r.samples]
    if not all_samples:
        print("no GPS fix data found in any bag")
        return
    print(f"ALL  {len(all_samples):5d} samples   {fmt_fractions(all_samples)}")
    for buggy in ("NAND", "SC"):
        bs = [s for r in results if r.buggy == buggy for s in r.samples]
        if bs:
            print(f"{buggy:4s} {len(bs):5d} samples   {fmt_fractions(bs)}")

    print("\nby hour of day (local):")
    by_hour: dict[int, list[GpsSample]] = {}
    for s in all_samples:
        if clock_valid(s.t_ns):
            by_hour.setdefault(local_dt(s.t_ns, tz).hour, []).append(s)
    for hour in sorted(by_hour):
        print(f"  {hour:02d}:00  {len(by_hour[hour]):5d} samples   {fmt_fractions(by_hour[hour])}")


def write_csv(results: list[BagResult], path: Path, tz: ZoneInfo):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bag", "buggy", "unix_ns", "local_time", "east_m", "north_m", "rtk_fix"])
        for r in results:
            for s in r.samples:
                w.writerow([r.name, r.buggy, s.t_ns, local_dt(s.t_ns, tz).isoformat(), f"{s.east:.3f}", f"{s.north:.3f}", FIX_LABEL[s.fix]])
    print(f"wrote {path}")


# ---------------------------------------------------------------- plots


def fix_legend_handles():
    return [
        Line2D([], [], linestyle="", marker=FIX_MARKER[f], color=FIX_COLOR[f], markersize=7, label=FIX_LABEL[f])
        for f in reversed(RtkFix)
    ]


def scatter_by_fix(ax, samples: list[GpsSample], size=9):
    # draw fixed first, dropouts last so problem areas stay visible
    for f in RtkFix:
        pts = [(s.east, s.north) for s in samples if s.fix == f]
        if pts:
            e, n = zip(*pts)
            ax.scatter(e, n, s=size, marker=FIX_MARKER[f], color=FIX_COLOR[f], alpha=0.7, linewidths=0.8, label=FIX_LABEL[f])
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)


def plot_positions(results: list[BagResult]):
    buggies = [b for b in ("NAND", "SC") if any(r.buggy == b and r.samples for r in results)]
    fig, axes = plt.subplots(1, len(buggies), figsize=(8 * len(buggies), 8), squeeze=False)
    for ax, buggy in zip(axes[0], buggies):
        samples = [s for r in results if r.buggy == buggy for s in r.samples]
        nbags = sum(1 for r in results if r.buggy == buggy and r.samples)
        scatter_by_fix(ax, samples)
        ax.set_title(f"{buggy}: RTK status by position ({nbags} bag{'s' if nbags != 1 else ''}, {len(samples)} samples)")
        ax.set_xlabel("east (m, course-local UTM)")
        ax.set_ylabel("north (m)")
        ax.legend(handles=fix_legend_handles(), loc="best")
    fig.tight_layout()
    return fig


def plot_time_of_day(results: list[BagResult], tz: ZoneInfo, tod_range: tuple[float, float] | None = None):
    """Top: per-bag status timeline. Below: one stacked RTK-fraction bar per bag, placed at the bag's
    start time on a time-of-day axis (one row per buggy so simultaneous rolls do not overlap)."""
    with_data = [r for r in results if r.timed_samples]
    if not with_data:
        return None
    buggies = [b for b in ("NAND", "SC") if any(r.buggy == b for r in with_data)]
    fig, axes = plt.subplots(
        1 + len(buggies), 1,
        figsize=(13, 2 + 0.45 * len(with_data) + 3.2 * len(buggies)),
        gridspec_kw={"height_ratios": [max(1, 0.45 * len(with_data))] + [3] * len(buggies)},
    )
    ax_tl, bar_axes = axes[0], axes[1:]

    # per-bag timeline strips: elapsed time since the first validly-stamped sample; label carries local start time
    labels = []
    for i, r in enumerate(with_data):
        t0 = r.first_valid_ns
        for start, end, fix in fix_runs(r.timed_samples):
            m0, m1 = (start - t0) / 6e10, (end - t0) / 6e10
            ax_tl.broken_barh([(m0, m1 - m0)], (i - 0.4, 0.8), color=FIX_COLOR[fix])
        labels.append(f"{local_dt(t0, tz):%Y-%m-%d %H:%M} {r.buggy}  {r.name}")
    ax_tl.set_yticks(range(len(with_data)))
    ax_tl.set_yticklabels(labels, fontsize=8)
    ax_tl.invert_yaxis()
    ax_tl.set_xlim(left=0)
    ax_tl.set_xlabel("minutes into bag")
    ax_tl.set_title("RTK status over each bag")
    ax_tl.grid(True, axis="x", alpha=0.3)
    ax_tl.legend(handles=fix_legend_handles(), loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)

    # stacked fraction per bag at its start time of day
    MIN_BAR_H = 4 / 60  # a very short bag still gets a visible bar
    lo, hi = tod_range if tod_range else (0.0, 24.0)
    for ax, buggy in zip(bar_axes, buggies):
        for r in with_data:
            if r.buggy != buggy:
                continue
            t0 = r.first_valid_ns
            day0 = local_dt(t0, tz).replace(hour=0, minute=0, second=0, microsecond=0)
            x0 = (t0 - int(day0.timestamp() * 1e9)) / 3.6e12
            w = max((r.end_ns - t0) / 3.6e12, MIN_BAR_H)
            fr = fix_fractions(r.timed_samples)
            bottom = 0.0
            for f in reversed(RtkFix):
                ax.bar(x0, fr[f], bottom=bottom, width=w, align="edge", color=FIX_COLOR[f], edgecolor="white", linewidth=1, label=FIX_LABEL[f])
                bottom += fr[f]
            if w >= (hi - lo) / 80:  # labels would pile up on slivers; use --tod-range to zoom in
                ax.text(x0 + w / 2, 1.01, f"{local_dt(t0, tz):%H:%M}\nn={len(r.timed_samples)}", ha="center", va="bottom", fontsize=7)
        ax.set_xlim(lo, hi)
        step = 1.0 if hi - lo > 6 else 0.5 if hi - lo > 3 else 0.25
        ticks = np.arange(np.floor(lo / step) * step, hi + 1e-9, step)
        ax.set_xticks(ticks)
        ax.set_xticklabels([f"{int(t):02d}:{int(round((t % 1) * 60)):02d}" for t in ticks], fontsize=8)
        ax.set_ylim(0, 1.15)
        ax.set_ylabel("fraction of samples")
        ax.set_title(f"{buggy}: RTK status per bag by start time of day")
        ax.grid(True, axis="x", alpha=0.3)
        handles, labels_ = ax.get_legend_handles_labels()
        uniq = dict(zip(labels_, handles))
        ax.legend([uniq[FIX_LABEL[f]] for f in reversed(RtkFix) if FIX_LABEL[f] in uniq],
                  [FIX_LABEL[f] for f in reversed(RtkFix) if FIX_LABEL[f] in uniq],
                  loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)
    bar_axes[-1].set_xlabel(f"time of day ({tz.key})")
    fig.tight_layout()
    return fig


def plot_positions_by_hour(results: list[BagResult], tz: ZoneInfo, max_panels=6):
    """Small multiples: the position scatter, one panel per hour-of-day bin."""
    samples = [s for r in results for s in r.timed_samples]
    hours = sorted({local_dt(s.t_ns, tz).hour for s in samples})
    if not hours:
        return None
    span = max(hours) - min(hours) + 1
    bin_h = max(1, int(np.ceil(span / max_panels)))
    bins = sorted({(local_dt(s.t_ns, tz).hour - min(hours)) // bin_h for s in samples})
    fig, axes = plt.subplots(1, len(bins), figsize=(6 * len(bins), 6), squeeze=False, sharex=True, sharey=True)
    for ax, b in zip(axes[0], bins):
        h0 = min(hours) + b * bin_h
        sub = [s for s in samples if (local_dt(s.t_ns, tz).hour - min(hours)) // bin_h == b]
        scatter_by_fix(ax, sub)
        fr = fix_fractions(sub)
        ax.set_title(f"{h0:02d}:00-{h0 + bin_h:02d}:00  n={len(sub)}  fixed {100 * fr[RtkFix.FIXED]:.0f}%  float {100 * fr[RtkFix.FLOAT]:.0f}%")
        ax.set_xlabel("east (m)")
    axes[0][0].set_ylabel("north (m)")
    axes[0][0].legend(handles=fix_legend_handles(), loc="best")
    fig.suptitle(f"RTK status by position, split by time of day ({tz.key})")
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------- main


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bags", nargs="+", type=Path, help="bag directories, bag files, or directories to search")
    parser.add_argument("--buggy", choices=["NAND", "SC"], help="override autodetection for every bag")
    parser.add_argument(
        "--sc-rtk-mode",
        choices=["both", "either", "gnss_1", "gnss_2"],
        default="both",
        help="how SC's two receivers combine: both = worst of the two (default), either = best, or a single receiver",
    )
    parser.add_argument("--tz", default="America/New_York", help="time zone for time-of-day analysis")
    parser.add_argument("--tod-range", metavar="HH:MM-HH:MM", help="also write a time-of-day plot cropped to this window, e.g. 07:00-09:00")
    parser.add_argument("--out", type=Path, help="directory to save plots (and csv) into")
    parser.add_argument("--csv", action="store_true", help="also write every sample to rtk_samples.csv in --out")
    parser.add_argument("--show", action="store_true", help="open the plots interactively")
    args = parser.parse_args()
    tz = ZoneInfo(args.tz)

    bags = find_bags(args.bags)
    if not bags:
        sys.exit("no bags found")
    print(f"found {len(bags)} bag(s)")

    results = []
    for bag in bags:
        try:
            r = read_bag(bag, args.sc_rtk_mode, args.buggy)
        except Exception as e:  # keep going: one corrupt bag should not kill a directory sweep
            print(f"warning: {bag}: {e}", file=sys.stderr)
            continue
        if r is not None:
            results.append(r)
    results.sort(key=lambda r: r.start_ns)

    print_report(results, tz)
    if not any(r.samples for r in results):
        return

    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        if args.csv:
            write_csv(results, args.out / "rtk_samples.csv", tz)

    figs = {
        "rtk_positions.png": plot_positions(results),
        "rtk_time_of_day.png": plot_time_of_day(results, tz),
        "rtk_positions_by_hour.png": plot_positions_by_hour(results, tz),
    }
    if args.tod_range:
        lo, hi = (sum(int(x) * k for x, k in zip(part.split(":"), (1, 1 / 60))) for part in args.tod_range.split("-"))
        figs["rtk_time_of_day_cropped.png"] = plot_time_of_day(results, tz, (lo, hi))
    if args.out:
        for name, fig in figs.items():
            if fig is not None:
                fig.savefig(args.out / name, dpi=150)
                print(f"wrote {args.out / name}")
    if args.show or not args.out:
        plt.show()


if __name__ == "__main__":
    main()
