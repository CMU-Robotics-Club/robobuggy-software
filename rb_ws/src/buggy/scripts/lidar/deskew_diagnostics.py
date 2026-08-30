#! /usr/bin/env python3

import argparse
import csv
from pathlib import Path

import numpy as np

from lidar_deskew import PoseTimeline, deskew_frame_block


SC_LIDAR_TO_BODY_ROTATION = np.diag([-1.0, -1.0, 1.0])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Diagnose deskew outliers by comparing translation, rotation, and full compensation."
    )
    parser.add_argument(
        "--input",
        default="sc_feb_21_26_roll_1.npz",
        help="NPZ file produced by bag_to_numpy.py.",
    )
    parser.add_argument(
        "--metrics-csv",
        default="",
        help="Deskew metrics CSV. Defaults to <input_stem>_deskew_metrics.csv.",
    )
    parser.add_argument(
        "--merge-size",
        type=int,
        default=4,
        help="Merged-frame size used when producing the metrics CSV.",
    )
    parser.add_argument(
        "--top-blocks",
        type=int,
        default=8,
        help="Number of worst merged frames to inspect.",
    )
    parser.add_argument(
        "--top-points",
        type=int,
        default=8,
        help="Number of worst points to print per mode.",
    )
    return parser.parse_args()


def load_metric_rows(csv_path: Path) -> list[dict]:
    rows = []
    with csv_path.open(encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                {
                    key: int(float(value))
                    if key in {"merged_idx", "raw_start", "raw_end", "point_count"}
                    else float(value)
                    for key, value in row.items()
                }
            )
    if not rows:
        raise ValueError(f"{csv_path} contained no metric rows.")
    return rows


def zero_orientation_timeline(pose_timeline: PoseTimeline) -> PoseTimeline:
    return PoseTimeline(
        timestamps=pose_timeline.timestamps,
        positions=pose_timeline.positions,
        orientations_rpy=np.zeros_like(pose_timeline.orientations_rpy),
    )


def zero_position_timeline(pose_timeline: PoseTimeline) -> PoseTimeline:
    return PoseTimeline(
        timestamps=pose_timeline.timestamps,
        positions=np.zeros_like(pose_timeline.positions),
        orientations_rpy=pose_timeline.orientations_rpy,
    )


def frame_block(npz_data, start: int, end: int):
    frames = [np.asarray(npz_data["frames"][idx], dtype=np.float64)[:, :3] for idx in range(start, end)]
    point_times = None
    if "point_times" in npz_data:
        point_times = [np.asarray(npz_data["point_times"][idx], dtype=np.float64).reshape(-1) for idx in range(start, end)]
    rings = None
    if "rings" in npz_data:
        rings = [np.asarray(npz_data["rings"][idx]).reshape(-1) for idx in range(start, end)]
    intensities = None
    if "intensities" in npz_data:
        intensities = [np.asarray(npz_data["intensities"][idx]).reshape(-1) for idx in range(start, end)]
    timestamps = np.asarray(npz_data["timestamps"][start:end], dtype=np.float64)
    return frames, point_times, rings, intensities, timestamps


def collect_point_rows(frames, corrected_frames, point_times_seq, rings_seq, intensities_seq, raw_start: int):
    rows = []
    for local_idx, (original, corrected) in enumerate(zip(frames, corrected_frames)):
        displacement = np.linalg.norm(corrected - original, axis=1)
        point_times = None if point_times_seq is None else point_times_seq[local_idx]
        rings = None if rings_seq is None else rings_seq[local_idx]
        intensities = None if intensities_seq is None else intensities_seq[local_idx]
        ranges = np.linalg.norm(original, axis=1)
        azimuth_deg = np.degrees(np.arctan2(original[:, 1], original[:, 0]))

        for point_idx in range(len(original)):
            rows.append(
                {
                    "raw_idx": raw_start + local_idx,
                    "point_idx": point_idx,
                    "disp_m": float(displacement[point_idx]),
                    "range_m": float(ranges[point_idx]),
                    "point_time_s": None if point_times is None else float(point_times[point_idx]),
                    "ring": None if rings is None else int(rings[point_idx]),
                    "intensity": None if intensities is None else float(intensities[point_idx]),
                    "azimuth_deg": float(azimuth_deg[point_idx]),
                    "orig_xyz": original[point_idx],
                    "desk_xyz": corrected[point_idx],
                }
            )
    return rows


def print_point_rows(rows: list[dict], top_points: int):
    for row in rows[:top_points]:
        print(
            "    "
            f"disp={row['disp_m']:.3f}m "
            f"range={row['range_m']:.3f}m "
            f"pt_time={row['point_time_s'] if row['point_time_s'] is not None else float('nan'):.6f}s "
            f"ring={row['ring'] if row['ring'] is not None else 'NA'} "
            f"intensity={row['intensity'] if row['intensity'] is not None else float('nan'):.3f} "
            f"az={row['azimuth_deg']:.1f}deg "
            f"raw={row['raw_idx']} point={row['point_idx']} "
            f"orig=({row['orig_xyz'][0]:.3f},{row['orig_xyz'][1]:.3f},{row['orig_xyz'][2]:.3f}) "
            f"desk=({row['desk_xyz'][0]:.3f},{row['desk_xyz'][1]:.3f},{row['desk_xyz'][2]:.3f})"
        )


def print_distribution(label: str, values: np.ndarray):
    if values.size == 0:
        print(f"    {label}: no data")
        return
    print(
        f"    {label}: "
        f"mean={np.mean(values):.3f} "
        f"median={np.median(values):.3f} "
        f"p95={np.percentile(values, 95):.3f} "
        f"max={np.max(values):.3f}"
    )


def print_ring_summary(rows: list[dict], top_points: int):
    ring_counts = {}
    for row in rows[:top_points]:
        if row["ring"] is None:
            continue
        ring_counts[row["ring"]] = ring_counts.get(row["ring"], 0) + 1
    if not ring_counts:
        print("    top-point rings: no data")
        return
    ordered = sorted(ring_counts.items(), key=lambda item: (-item[1], item[0]))
    print("    top-point rings:", ", ".join(f"{ring}:{count}" for ring, count in ordered))


def print_mode_summary(name: str, rows: list[dict], top_points: int):
    displacements = np.array([row["disp_m"] for row in rows], dtype=np.float64)
    ranges = np.array([row["range_m"] for row in rows], dtype=np.float64)
    point_times = np.array(
        [row["point_time_s"] for row in rows if row["point_time_s"] is not None],
        dtype=np.float64,
    )
    azimuths = np.array([row["azimuth_deg"] for row in rows], dtype=np.float64)

    print(f"  {name}")
    print_distribution("displacement_m", displacements)
    print_distribution("range_m", ranges)
    print_distribution("point_time_s", point_times)
    print_distribution("azimuth_deg", azimuths)
    print_ring_summary(rows, top_points)
    print("    worst points:")
    print_point_rows(rows, top_points)


def deskew_mode_rows(frames, timestamps, point_times, rings, intensities, pose_timeline: PoseTimeline, raw_start: int):
    modes = {
        "translation_only": zero_orientation_timeline(pose_timeline),
        "rotation_only": zero_position_timeline(pose_timeline),
        "full": pose_timeline,
    }
    mode_rows = {}
    for mode_name, timeline in modes.items():
        corrected_frames, _ = deskew_frame_block(
            frames=frames,
            frame_timestamps=timestamps,
            pose_timeline=timeline,
            point_times_seq=point_times,
            sensor_to_body_rotation=SC_LIDAR_TO_BODY_ROTATION,
        )
        rows = collect_point_rows(frames, corrected_frames, point_times, rings, intensities, raw_start)
        rows.sort(key=lambda row: row["disp_m"], reverse=True)
        mode_rows[mode_name] = rows
    return mode_rows


def main():
    args = parse_args()
    input_path = Path(args.input).expanduser()
    metrics_csv = Path(args.metrics_csv).expanduser() if args.metrics_csv else input_path.with_suffix("")
    if not args.metrics_csv:
        metrics_csv = metrics_csv.parent / f"{metrics_csv.name}_deskew_metrics.csv"

    npz_data = np.load(input_path, allow_pickle=True)
    pose_timeline = PoseTimeline.from_npz(npz_data)
    if pose_timeline is None:
        raise ValueError(f"{input_path} does not contain state data for deskew diagnostics.")

    rows = load_metric_rows(metrics_csv)
    top_rows = sorted(rows, key=lambda row: row["mean_cm"], reverse=True)[: args.top_blocks]

    print(f"Input: {input_path}")
    print(f"Metrics: {metrics_csv}")
    print(f"Merge size: {args.merge_size}")
    print(f"Inspecting {len(top_rows)} merged frames by descending mean deskew displacement")

    for metric_row in top_rows:
        block_idx = metric_row["merged_idx"]
        start = metric_row["raw_start"]
        end = metric_row["raw_end"] + 1
        frames, point_times, rings, intensities, timestamps = frame_block(npz_data, start, end)
        mode_rows = deskew_mode_rows(
            frames=frames,
            timestamps=timestamps,
            point_times=point_times,
            rings=rings,
            intensities=intensities,
            pose_timeline=pose_timeline,
            raw_start=start,
        )

        print()
        print(
            f"Merged frame {block_idx} raw={start}-{end - 1} "
            f"csv_mean={metric_row['mean_cm']:.3f}cm csv_std={metric_row['std_cm']:.3f}cm "
            f"csv_max={metric_row['max_cm']:.3f}cm "
            f"span={metric_row['end_time'] - metric_row['start_time']:.6f}s"
        )
        for mode_name in ("translation_only", "rotation_only", "full"):
            print_mode_summary(mode_name, mode_rows[mode_name], args.top_points)


if __name__ == "__main__":
    main()
