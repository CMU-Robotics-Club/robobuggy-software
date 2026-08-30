#! /usr/bin/env python3

import argparse
from itertools import permutations, product
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from lidar_deskew import PoseTimeline, rpy_to_rotation_matrices_batch


def parse_args():
    parser = argparse.ArgumentParser(
        description="Rank signed-axis LiDAR-to-IMU rotations by odometry-predicted scan alignment."
    )
    parser.add_argument("--input", default="sc_feb_21_26_roll_1.npz")
    parser.add_argument("--start", type=int, default=1400)
    parser.add_argument("--count", type=int, default=80)
    parser.add_argument("--step", type=int, default=4)
    parser.add_argument("--max-range", type=float, default=45.0)
    parser.add_argument("--sample-points", type=int, default=2500)
    parser.add_argument("--yaw-only", action="store_true")
    return parser.parse_args()


def signed_permutation_rotations():
    rotations = []
    for perm in permutations(range(3)):
        for signs in product((-1.0, 1.0), repeat=3):
            mat = np.zeros((3, 3), dtype=np.float64)
            for row, axis in enumerate(perm):
                mat[row, axis] = signs[row]
            if np.linalg.det(mat) > 0.5:
                rotations.append(mat)
    return rotations


def rotation_name(rotation: np.ndarray) -> str:
    labels = ["Lx", "Ly", "Lz"]
    parts = []
    for row_name, row in zip(["Ix", "Iy", "Iz"], rotation):
        axis = int(np.argmax(np.abs(row)))
        sign = "+" if row[axis] > 0 else "-"
        parts.append(f"{row_name}={sign}{labels[axis]}")
    return " ".join(parts)


def filtered_points(frame, max_range: float):
    points = np.asarray(frame, dtype=np.float64)[:, :3]
    mask = np.isfinite(points).all(axis=1)
    points = points[mask]
    xy_range = np.linalg.norm(points[:, :2], axis=1)
    return points[(xy_range > 2.0) & (xy_range < max_range) & (points[:, 2] > -2.0) & (points[:, 2] < 3.0)]


def sample_points(points: np.ndarray, max_points: int):
    if len(points) <= max_points:
        return points
    stride = max(1, len(points) // max_points)
    return points[::stride][:max_points]


def pose_matrices(pose_timeline: PoseTimeline, query_times: np.ndarray, yaw_only: bool):
    positions, rpy = pose_timeline.sample(query_times)
    if yaw_only:
        rpy[:, 0] = 0.0
        rpy[:, 1] = 0.0
    rotations = rpy_to_rotation_matrices_batch(rpy)
    return positions, rotations


def transform_source_to_target_lidar(source_points, p0, r0, p1, r1, lidar_to_imu):
    source_imu = np.einsum("ij,nj->ni", lidar_to_imu, source_points)
    source_world = np.einsum("ij,nj->ni", r0, source_imu) + p0
    target_imu = np.einsum("ji,nj->ni", r1, source_world - p1)
    return np.einsum("ji,nj->ni", lidar_to_imu, target_imu)


def score_rotation(frames, timestamps, positions, rotations, pair_indices, rotation, max_range, sample_count):
    medians = []
    p90s = []
    usable_pairs = 0
    for pair_idx, source_idx in enumerate(pair_indices):
        target_idx = source_idx + 1
        source = sample_points(filtered_points(frames[source_idx], max_range), sample_count)
        target = filtered_points(frames[target_idx], max_range)
        if len(source) < 100 or len(target) < 100:
            continue

        transformed = transform_source_to_target_lidar(
            source,
            positions[2 * pair_idx],
            rotations[2 * pair_idx],
            positions[2 * pair_idx + 1],
            rotations[2 * pair_idx + 1],
            rotation,
        )
        tree = cKDTree(target)
        distances, _ = tree.query(transformed, k=1, workers=-1)
        distances = distances[np.isfinite(distances)]
        if len(distances) == 0:
            continue
        medians.append(float(np.median(distances)))
        p90s.append(float(np.percentile(distances, 90)))
        usable_pairs += 1

    if usable_pairs == 0:
        return None
    return {
        "median_nn_m": float(np.median(medians)),
        "p90_nn_m": float(np.median(p90s)),
        "usable_pairs": usable_pairs,
    }


def main():
    args = parse_args()
    data = np.load(Path(args.input).expanduser(), allow_pickle=True)
    pose_timeline = PoseTimeline.from_npz(data)
    if pose_timeline is None:
        raise ValueError("Input NPZ does not contain state data.")

    frames = data["frames"]
    timestamps = np.asarray(data["timestamps"], dtype=np.float64)
    pair_indices = list(range(args.start, min(args.start + args.count * args.step, len(frames) - 1), args.step))
    query_times = np.array([timestamps[idx] for idx in pair_indices for _ in (0, 1)], dtype=np.float64)
    query_times[1::2] = [timestamps[idx + 1] for idx in pair_indices]
    positions, rotations = pose_matrices(pose_timeline, query_times, args.yaw_only)

    results = []
    for rotation in signed_permutation_rotations():
        score = score_rotation(
            frames,
            timestamps,
            positions,
            rotations,
            pair_indices,
            rotation,
            args.max_range,
            args.sample_points,
        )
        if score is None:
            continue
        results.append((score["median_nn_m"], score["p90_nn_m"], score["usable_pairs"], rotation))

    results.sort(key=lambda item: (item[0], item[1]))
    print(f"input={args.input} start={args.start} count={args.count} step={args.step} yaw_only={args.yaw_only}")
    for median_nn, p90_nn, usable_pairs, rotation in results[:12]:
        print(
            f"median_nn={median_nn:.4f}m p90_nn={p90_nn:.4f}m pairs={usable_pairs} "
            f"{rotation_name(rotation)}"
        )


if __name__ == "__main__":
    main()
