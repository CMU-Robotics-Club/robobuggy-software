#! /usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from lidar_deskew import PoseTimeline, rpy_to_rotation_matrices_batch


def parse_args():
    parser = argparse.ArgumentParser(
        description="Locate impossible orientation changes in exported odometry state."
    )
    parser.add_argument(
        "--input",
        default="sc_feb_21_26_roll_1.npz",
        help="NPZ file produced by bag_to_numpy.py.",
    )
    parser.add_argument(
        "--top",
        type=int,
        default=12,
        help="Number of largest state-to-state rotation jumps to print.",
    )
    parser.add_argument(
        "--around-raw",
        type=int,
        default=1040,
        help="Raw LiDAR frame index whose surrounding state samples should be inspected.",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=0.35,
        help="Seconds around --around-raw to summarize.",
    )
    parser.add_argument(
        "--rate-threshold-deg-s",
        type=float,
        default=90.0,
        help="Angular-rate threshold used to count suspect state intervals.",
    )
    return parser.parse_args()


def rotation_angle_deg(rotation: np.ndarray) -> float:
    cos_theta = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_theta)))


def wrap_deg(angle_deg: np.ndarray) -> np.ndarray:
    return (angle_deg + 180.0) % 360.0 - 180.0


def relative_rotation_angles_deg(rpy: np.ndarray) -> np.ndarray:
    rotations = rpy_to_rotation_matrices_batch(rpy)
    angles = np.empty(len(rotations) - 1, dtype=np.float64)
    for idx in range(len(angles)):
        angles[idx] = rotation_angle_deg(rotations[idx + 1] @ rotations[idx].T)
    return angles


def print_interval(
    label: str,
    idx: int,
    timestamps: np.ndarray,
    positions: np.ndarray,
    rpy_deg: np.ndarray,
    rotation_angles: np.ndarray,
    rates_deg_s: np.ndarray,
):
    dt = timestamps[idx + 1] - timestamps[idx]
    dpos = np.linalg.norm(positions[idx + 1] - positions[idx])
    drpy = wrap_deg(rpy_deg[idx + 1] - rpy_deg[idx])
    print(
        f"{label} idx={idx}->{idx + 1} "
        f"t={timestamps[idx]:.9f}->{timestamps[idx + 1]:.9f} "
        f"dt={dt:.6f}s "
        f"dpos={dpos:.3f}m "
        f"drot={rotation_angles[idx]:.3f}deg "
        f"rate={rates_deg_s[idx]:.1f}deg/s "
        f"drpy=({drpy[0]:.2f},{drpy[1]:.2f},{drpy[2]:.2f})deg "
        f"rpy0=({rpy_deg[idx, 0]:.2f},{rpy_deg[idx, 1]:.2f},{rpy_deg[idx, 2]:.2f})deg "
        f"rpy1=({rpy_deg[idx + 1, 0]:.2f},{rpy_deg[idx + 1, 1]:.2f},{rpy_deg[idx + 1, 2]:.2f})deg"
    )


def lidar_scan_time(npz_data, raw_idx: int) -> float:
    timestamps = np.asarray(npz_data["timestamps"], dtype=np.float64)
    if raw_idx < 0 or raw_idx >= len(timestamps):
        raise ValueError(f"--around-raw must be in [0, {len(timestamps) - 1}]")
    return float(timestamps[raw_idx])


def summarize_query_samples(pose_timeline: PoseTimeline, scan_time: float, window: float):
    query_times = np.linspace(scan_time - window * 0.5, scan_time + window * 0.5, 9)
    _, sampled_rpy = pose_timeline.sample(query_times)
    sampled_deg = np.degrees(sampled_rpy)
    sampled_dyaw = wrap_deg(np.diff(sampled_deg[:, 2]))
    print()
    print("sampled_pose_timeline_around_scan")
    for timestamp, rpy_deg in zip(query_times, sampled_deg):
        print(
            f"  t={timestamp:.9f} "
            f"rpy=({rpy_deg[0]:.2f},{rpy_deg[1]:.2f},{rpy_deg[2]:.2f})deg"
        )
    print(
        "  yaw_step_deg: "
        + ", ".join(f"{value:.2f}" for value in sampled_dyaw)
    )


def main():
    args = parse_args()
    input_path = Path(args.input).expanduser()
    npz_data = np.load(input_path, allow_pickle=True)
    pose_timeline = PoseTimeline.from_npz(npz_data)
    if pose_timeline is None:
        raise ValueError(f"{input_path} does not contain exported odometry state.")

    timestamps = pose_timeline.timestamps
    positions = pose_timeline.positions
    rpy = pose_timeline.orientations_rpy
    rpy_deg = np.degrees(rpy)

    dt = np.diff(timestamps)
    valid_dt = dt > 0.0
    rotation_angles = relative_rotation_angles_deg(rpy)
    rates = np.full_like(rotation_angles, np.nan)
    rates[valid_dt] = rotation_angles[valid_dt] / dt[valid_dt]

    finite_rates = rates[np.isfinite(rates)]
    print(f"input={input_path}")
    print(f"state_samples={len(timestamps)}")
    print(f"state_time_range={timestamps[0]:.9f}->{timestamps[-1]:.9f}")
    print(
        f"state_dt_s median={np.median(dt[valid_dt]):.6f} "
        f"min={np.min(dt[valid_dt]):.6f} max={np.max(dt[valid_dt]):.6f}"
    )
    print(
        f"roll_deg median={np.median(rpy_deg[:, 0]):.2f} "
        f"p05={np.percentile(rpy_deg[:, 0], 5):.2f} p95={np.percentile(rpy_deg[:, 0], 95):.2f}"
    )
    print(
        f"pitch_deg median={np.median(rpy_deg[:, 1]):.2f} "
        f"p05={np.percentile(rpy_deg[:, 1], 5):.2f} p95={np.percentile(rpy_deg[:, 1], 95):.2f}"
    )
    print(
        f"yaw_deg median={np.median(rpy_deg[:, 2]):.2f} "
        f"p05={np.percentile(rpy_deg[:, 2], 5):.2f} p95={np.percentile(rpy_deg[:, 2], 95):.2f}"
    )
    print(
        f"angular_rate_deg_s median={np.median(finite_rates):.2f} "
        f"p95={np.percentile(finite_rates, 95):.2f} "
        f"p99={np.percentile(finite_rates, 99):.2f} "
        f"max={np.max(finite_rates):.2f}"
    )

    suspect_mask = finite_rates > args.rate_threshold_deg_s
    print(
        f"suspect_intervals_over_{args.rate_threshold_deg_s:.1f}deg_s="
        f"{np.count_nonzero(rates > args.rate_threshold_deg_s)}"
    )

    print()
    print("largest_state_rotation_jumps")
    ordered = np.argsort(np.nan_to_num(rates, nan=-np.inf))[::-1]
    for idx in ordered[: args.top]:
        print_interval("  jump", int(idx), timestamps, positions, rpy_deg, rotation_angles, rates)

    scan_time = lidar_scan_time(npz_data, args.around_raw)
    start_time = scan_time - args.window * 0.5
    end_time = scan_time + args.window * 0.5
    interval_mask = (timestamps[:-1] >= start_time) & (timestamps[:-1] <= end_time)
    local_indices = np.flatnonzero(interval_mask)
    local_bad = local_indices[rates[local_indices] > args.rate_threshold_deg_s]

    print()
    print(
        f"around_raw={args.around_raw} scan_time={scan_time:.9f} "
        f"window={start_time:.9f}->{end_time:.9f}"
    )
    print(f"  local_state_intervals={len(local_indices)}")
    print(f"  local_suspect_intervals={len(local_bad)}")
    for idx in local_bad[: args.top]:
        print_interval("  local", int(idx), timestamps, positions, rpy_deg, rotation_angles, rates)

    summarize_query_samples(pose_timeline, scan_time, args.window)


if __name__ == "__main__":
    main()
