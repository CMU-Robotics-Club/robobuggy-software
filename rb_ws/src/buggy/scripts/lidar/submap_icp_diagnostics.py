#! /usr/bin/env python3

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from lidar_deskew import PoseTimeline, deskew_frame_block, rpy_to_rotation_matrices_batch


SC_LIDAR_TO_BODY_ROTATION = np.diag([-1.0, -1.0, 1.0])


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Register LiDAR submaps instead of adjacent raw scans so ICP has enough scene overlap "
            "to diagnose deskew and frame-semantic issues."
        )
    )
    parser.add_argument(
        "--input",
        default="sc_feb_21_26_roll_1.npz",
        help="NPZ file produced by bag_to_numpy.py.",
    )
    parser.add_argument(
        "--start",
        type=int,
        default=1036,
        help="First raw-frame index used to build the first source submap.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=6,
        help="Number of source/target submap pairs to inspect.",
    )
    parser.add_argument(
        "--submap-frames",
        type=int,
        default=4,
        help="Number of consecutive raw frames to deskew and merge into each submap.",
    )
    parser.add_argument(
        "--pair-separation",
        type=int,
        default=4,
        help="Raw-frame offset between the starts of the source and target submaps.",
    )
    parser.add_argument(
        "--pair-stride",
        type=int,
        default=4,
        help="Raw-frame increment between inspected source/target submap pairs.",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=35.0,
        help="Maximum XY range retained for registration.",
    )
    parser.add_argument(
        "--min-range",
        type=float,
        default=2.0,
        help="Minimum XY range retained for registration.",
    )
    parser.add_argument(
        "--z-min",
        type=float,
        default=-2.0,
        help="Minimum Z retained for registration.",
    )
    parser.add_argument(
        "--z-max",
        type=float,
        default=3.0,
        help="Maximum Z retained for registration.",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.25,
        help="Voxel size for deterministic submap downsampling. Use 0 to disable.",
    )
    parser.add_argument(
        "--sample-points",
        type=int,
        default=6000,
        help="Maximum number of source points used per ICP pair after filtering/downsampling.",
    )
    parser.add_argument(
        "--icp-threshold",
        type=float,
        default=1.5,
        help="Maximum correspondence distance used for ICP and nearest-neighbor scoring.",
    )
    parser.add_argument(
        "--icp-iterations",
        type=int,
        default=30,
        help="Maximum ICP iterations per pair.",
    )
    parser.add_argument(
        "--top-pairs",
        type=int,
        default=4,
        help="How many worst pairs to print per mode.",
    )
    return parser.parse_args()


@dataclass
class PairResult:
    mode: str
    source_start: int
    source_end: int
    target_start: int
    target_end: int
    source_time: float
    target_time: float
    predicted_translation_m: float
    predicted_yaw_deg: float
    initial_mean_nn_m: float
    initial_median_nn_m: float
    initial_p90_nn_m: float
    initial_max_nn_m: float
    icp_fitness: float
    icp_rmse_m: float
    final_mean_nn_m: float
    final_median_nn_m: float
    final_p90_nn_m: float
    final_max_nn_m: float
    correction_translation_m: float
    correction_yaw_deg: float
    source_points: int
    target_points: int


class FrameCache:
    def __init__(self, npz_data):
        self.frames = npz_data["frames"]
        self.timestamps = np.asarray(npz_data["timestamps"], dtype=np.float64)
        self.point_times = npz_data["point_times"] if "point_times" in npz_data else None
        self._cache: dict[int, tuple[np.ndarray, np.ndarray | None, float]] = {}

    def get(self, idx: int) -> tuple[np.ndarray, np.ndarray | None, float]:
        if idx not in self._cache:
            raw_frame = np.asarray(self.frames[idx], dtype=np.float64)
            if raw_frame.ndim != 2 or raw_frame.shape[1] < 3:
                raise ValueError(f"Expected frame {idx} to have shape (N, 3+), got {raw_frame.shape}")

            xyz = raw_frame[:, :3]
            valid_mask = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid_mask]

            point_times = None
            if self.point_times is not None:
                raw_times = np.asarray(self.point_times[idx], dtype=np.float64).reshape(-1)
                if raw_times.shape[0] != raw_frame.shape[0]:
                    raise ValueError(
                        f"Point-time array for frame {idx} had {raw_times.shape[0]} values but "
                        f"the frame had {raw_frame.shape[0]} points."
                    )
                point_times = raw_times[valid_mask]

            self._cache[idx] = (xyz, point_times, float(self.timestamps[idx]))
        return self._cache[idx]


def zero_orientation_timeline(pose_timeline: PoseTimeline) -> PoseTimeline:
    return PoseTimeline(
        timestamps=pose_timeline.timestamps,
        positions=pose_timeline.positions,
        orientations_rpy=np.zeros_like(pose_timeline.orientations_rpy),
    )


def yaw_only_timeline(pose_timeline: PoseTimeline) -> PoseTimeline:
    yaw_only = pose_timeline.orientations_rpy.copy()
    yaw_only[:, 0] = 0.0
    yaw_only[:, 1] = 0.0
    return PoseTimeline(
        timestamps=pose_timeline.timestamps,
        positions=pose_timeline.positions,
        orientations_rpy=yaw_only,
    )


def mode_timelines(pose_timeline: PoseTimeline) -> dict[str, PoseTimeline]:
    return {
        "translation_only": zero_orientation_timeline(pose_timeline),
        "yaw_only": yaw_only_timeline(pose_timeline),
        "full": pose_timeline,
    }


def build_submap(
    frame_cache: FrameCache,
    start_idx: int,
    frame_count: int,
    pose_timeline: PoseTimeline,
    sensor_to_body_rotation: np.ndarray,
) -> tuple[np.ndarray, float]:
    frames = []
    timestamps = []
    point_times_seq = [] if frame_cache.point_times is not None else None

    for idx in range(start_idx, start_idx + frame_count):
        frame, point_times, timestamp = frame_cache.get(idx)
        frames.append(frame)
        timestamps.append(timestamp)
        if point_times_seq is not None:
            point_times_seq.append(point_times)

    corrected_frames, reference_time = deskew_frame_block(
        frames=frames,
        frame_timestamps=np.asarray(timestamps, dtype=np.float64),
        pose_timeline=pose_timeline,
        point_times_seq=point_times_seq,
        sensor_to_body_rotation=sensor_to_body_rotation,
    )
    return np.vstack(corrected_frames), float(reference_time)


def filter_points(
    points: np.ndarray,
    min_range: float,
    max_range: float,
    z_min: float,
    z_max: float,
) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    valid_mask = np.isfinite(points).all(axis=1)
    points = points[valid_mask]
    xy_range = np.linalg.norm(points[:, :2], axis=1)
    range_mask = (xy_range >= min_range) & (xy_range <= max_range)
    z_mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    return points[range_mask & z_mask]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0.0 or len(points) == 0:
        return points

    voxels = np.floor(points / voxel_size).astype(np.int64)
    _, keep_indices = np.unique(voxels, axis=0, return_index=True)
    keep_indices.sort()
    return points[keep_indices]


def sample_points(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points
    stride = max(1, len(points) // max_points)
    return points[::stride][:max_points]


def tree_query(tree: cKDTree, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    try:
        return tree.query(points, k=1, workers=-1)
    except TypeError:
        return tree.query(points, k=1)


def nearest_neighbor_stats(source_points: np.ndarray, target_points: np.ndarray) -> dict[str, float]:
    if len(source_points) == 0 or len(target_points) == 0:
        return {
            "mean_m": float("nan"),
            "median_m": float("nan"),
            "p90_m": float("nan"),
            "max_m": float("nan"),
        }

    tree = cKDTree(target_points)
    distances, _ = tree_query(tree, source_points)
    finite = distances[np.isfinite(distances)]
    if len(finite) == 0:
        return {
            "mean_m": float("nan"),
            "median_m": float("nan"),
            "p90_m": float("nan"),
            "max_m": float("nan"),
        }

    return {
        "mean_m": float(np.mean(finite)),
        "median_m": float(np.median(finite)),
        "p90_m": float(np.percentile(finite, 90)),
        "max_m": float(np.max(finite)),
    }


def pose_at_time(pose_timeline: PoseTimeline, timestamp: float) -> tuple[np.ndarray, np.ndarray]:
    positions, rpy = pose_timeline.sample(np.array([timestamp], dtype=np.float64))
    rotations = rpy_to_rotation_matrices_batch(rpy)
    return positions[0], rotations[0]


def transform_source_to_target_lidar(
    source_points: np.ndarray,
    source_time: float,
    target_time: float,
    pose_timeline: PoseTimeline,
    sensor_to_body_rotation: np.ndarray,
    sensor_to_body_translation: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    sensor_to_body_translation = (
        np.zeros(3, dtype=np.float64)
        if sensor_to_body_translation is None
        else np.asarray(sensor_to_body_translation, dtype=np.float64).reshape(3)
    )

    source_position, source_rotation = pose_at_time(pose_timeline, source_time)
    target_position, target_rotation = pose_at_time(pose_timeline, target_time)

    transform = np.eye(4, dtype=np.float64)
    body_rotation = target_rotation.T @ source_rotation
    transform[:3, :3] = sensor_to_body_rotation.T @ body_rotation @ sensor_to_body_rotation
    transform[:3, 3] = sensor_to_body_rotation.T @ (
        target_rotation.T @ (source_rotation @ sensor_to_body_translation + source_position - target_position)
        - sensor_to_body_translation
    )
    return apply_transform(source_points, transform), transform


def best_fit_transform(source_points: np.ndarray, target_points: np.ndarray) -> np.ndarray:
    source_center = np.mean(source_points, axis=0)
    target_center = np.mean(target_points, axis=0)
    source_centered = source_points - source_center
    target_centered = target_points - target_center

    covariance = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T

    translation = target_center - rotation @ source_center

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def apply_transform(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def rotation_angle_deg(rotation: np.ndarray) -> float:
    trace = np.trace(rotation)
    cos_theta = np.clip((trace - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_theta)))


def yaw_deg_from_rotation(rotation: np.ndarray) -> float:
    return float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])))


def run_icp(
    source_points: np.ndarray,
    target_points: np.ndarray,
    threshold: float,
    max_iterations: int,
) -> tuple[np.ndarray, dict[str, float]]:
    if len(source_points) == 0 or len(target_points) == 0:
        return np.eye(4, dtype=np.float64), {
            "fitness": float("nan"),
            "rmse_m": float("nan"),
            "mean_m": float("nan"),
            "median_m": float("nan"),
            "p90_m": float("nan"),
            "max_m": float("nan"),
        }

    current_points = source_points.copy()
    cumulative_transform = np.eye(4, dtype=np.float64)
    target_tree = cKDTree(target_points)

    previous_rmse = None
    for _ in range(max_iterations):
        distances, indices = tree_query(target_tree, current_points)
        inlier_mask = np.isfinite(distances) & (distances <= threshold)
        if np.count_nonzero(inlier_mask) < 20:
            break

        matched_source = current_points[inlier_mask]
        matched_target = target_points[indices[inlier_mask]]
        delta_transform = best_fit_transform(matched_source, matched_target)
        current_points = apply_transform(current_points, delta_transform)
        cumulative_transform = delta_transform @ cumulative_transform

        current_rmse = float(np.sqrt(np.mean(distances[inlier_mask] ** 2)))
        step_translation = float(np.linalg.norm(delta_transform[:3, 3]))
        step_rotation = rotation_angle_deg(delta_transform[:3, :3])
        if (
            previous_rmse is not None
            and abs(previous_rmse - current_rmse) < 1e-4
            and step_translation < 1e-3
            and step_rotation < 1e-2
        ):
            break
        previous_rmse = current_rmse

    final_distances, _ = tree_query(target_tree, current_points)
    final_finite = final_distances[np.isfinite(final_distances)]
    final_mask = np.isfinite(final_distances) & (final_distances <= threshold)
    inlier_distances = final_distances[final_mask]
    fitness = float(np.count_nonzero(final_mask) / len(current_points)) if len(current_points) else float("nan")
    stats = {
        "fitness": fitness,
        "rmse_m": float(np.sqrt(np.mean(inlier_distances ** 2))) if len(inlier_distances) else float("nan"),
        "mean_m": float(np.mean(final_finite)) if len(final_finite) else float("nan"),
        "median_m": float(np.median(final_finite)) if len(final_finite) else float("nan"),
        "p90_m": float(np.percentile(final_finite, 90)) if len(final_finite) else float("nan"),
        "max_m": float(np.max(final_finite)) if len(final_finite) else float("nan"),
    }
    return cumulative_transform, stats


def submap_pair_result(
    frame_cache: FrameCache,
    pose_timeline: PoseTimeline,
    sensor_to_body_rotation: np.ndarray,
    source_start: int,
    target_start: int,
    frame_count: int,
    mode_name: str,
    args,
) -> PairResult:
    source_points, source_time = build_submap(
        frame_cache=frame_cache,
        start_idx=source_start,
        frame_count=frame_count,
        pose_timeline=pose_timeline,
        sensor_to_body_rotation=sensor_to_body_rotation,
    )
    target_points, target_time = build_submap(
        frame_cache=frame_cache,
        start_idx=target_start,
        frame_count=frame_count,
        pose_timeline=pose_timeline,
        sensor_to_body_rotation=sensor_to_body_rotation,
    )

    source_points = filter_points(
        source_points,
        min_range=args.min_range,
        max_range=args.max_range,
        z_min=args.z_min,
        z_max=args.z_max,
    )
    target_points = filter_points(
        target_points,
        min_range=args.min_range,
        max_range=args.max_range,
        z_min=args.z_min,
        z_max=args.z_max,
    )

    source_points = voxel_downsample(source_points, args.voxel_size)
    target_points = voxel_downsample(target_points, args.voxel_size)
    source_points = sample_points(source_points, args.sample_points)

    if len(source_points) == 0 or len(target_points) == 0:
        return PairResult(
            mode=mode_name,
            source_start=source_start,
            source_end=source_start + frame_count - 1,
            target_start=target_start,
            target_end=target_start + frame_count - 1,
            source_time=source_time,
            target_time=target_time,
            predicted_translation_m=float("nan"),
            predicted_yaw_deg=float("nan"),
            initial_mean_nn_m=float("nan"),
            initial_median_nn_m=float("nan"),
            initial_p90_nn_m=float("nan"),
            initial_max_nn_m=float("nan"),
            icp_fitness=float("nan"),
            icp_rmse_m=float("nan"),
            final_mean_nn_m=float("nan"),
            final_median_nn_m=float("nan"),
            final_p90_nn_m=float("nan"),
            final_max_nn_m=float("nan"),
            correction_translation_m=float("nan"),
            correction_yaw_deg=float("nan"),
            source_points=len(source_points),
            target_points=len(target_points),
        )

    predicted_source_in_target, predicted_transform = transform_source_to_target_lidar(
        source_points=source_points,
        source_time=source_time,
        target_time=target_time,
        pose_timeline=pose_timeline,
        sensor_to_body_rotation=sensor_to_body_rotation,
    )
    initial_stats = nearest_neighbor_stats(predicted_source_in_target, target_points)
    correction_transform, icp_stats = run_icp(
        source_points=predicted_source_in_target,
        target_points=target_points,
        threshold=args.icp_threshold,
        max_iterations=args.icp_iterations,
    )

    predicted_translation_m = float(np.linalg.norm(predicted_transform[:3, 3]))
    predicted_yaw_deg = yaw_deg_from_rotation(predicted_transform[:3, :3])
    correction_translation_m = float(np.linalg.norm(correction_transform[:3, 3]))
    correction_yaw_deg = yaw_deg_from_rotation(correction_transform[:3, :3])

    return PairResult(
        mode=mode_name,
        source_start=source_start,
        source_end=source_start + frame_count - 1,
        target_start=target_start,
        target_end=target_start + frame_count - 1,
        source_time=source_time,
        target_time=target_time,
        predicted_translation_m=predicted_translation_m,
        predicted_yaw_deg=predicted_yaw_deg,
        initial_mean_nn_m=initial_stats["mean_m"],
        initial_median_nn_m=initial_stats["median_m"],
        initial_p90_nn_m=initial_stats["p90_m"],
        initial_max_nn_m=initial_stats["max_m"],
        icp_fitness=icp_stats["fitness"],
        icp_rmse_m=icp_stats["rmse_m"],
        final_mean_nn_m=icp_stats["mean_m"],
        final_median_nn_m=icp_stats["median_m"],
        final_p90_nn_m=icp_stats["p90_m"],
        final_max_nn_m=icp_stats["max_m"],
        correction_translation_m=correction_translation_m,
        correction_yaw_deg=correction_yaw_deg,
        source_points=len(source_points),
        target_points=len(target_points),
    )


def finite_values(rows: list[PairResult], attr: str) -> np.ndarray:
    values = np.array([getattr(row, attr) for row in rows], dtype=np.float64)
    return values[np.isfinite(values)]


def print_summary(label: str, values: np.ndarray, units: str = ""):
    if len(values) == 0:
        print(f"  {label}: no data")
        return
    print(
        f"  {label}: "
        f"mean={np.mean(values):.3f}{units} "
        f"median={np.median(values):.3f}{units} "
        f"p90={np.percentile(values, 90):.3f}{units} "
        f"max={np.max(values):.3f}{units}"
    )


def print_mode_report(mode_name: str, rows: list[PairResult], top_pairs: int):
    usable_rows = [row for row in rows if np.isfinite(row.initial_median_nn_m)]

    print()
    print(f"{mode_name}")
    print(f"  usable_pairs={len(usable_rows)}/{len(rows)}")
    print_summary("predicted_translation_m", finite_values(usable_rows, "predicted_translation_m"), "m")
    print_summary("predicted_yaw_deg", finite_values(usable_rows, "predicted_yaw_deg"), "deg")
    print_summary("initial_median_nn_m", finite_values(usable_rows, "initial_median_nn_m"), "m")
    print_summary("initial_p90_nn_m", finite_values(usable_rows, "initial_p90_nn_m"), "m")
    print_summary("icp_fitness", finite_values(usable_rows, "icp_fitness"))
    print_summary("icp_rmse_m", finite_values(usable_rows, "icp_rmse_m"), "m")
    print_summary("final_median_nn_m", finite_values(usable_rows, "final_median_nn_m"), "m")
    print_summary("final_p90_nn_m", finite_values(usable_rows, "final_p90_nn_m"), "m")
    print_summary(
        "correction_translation_m",
        finite_values(usable_rows, "correction_translation_m"),
        "m",
    )
    print_summary(
        "abs_correction_yaw_deg",
        np.abs(finite_values(usable_rows, "correction_yaw_deg")),
        "deg",
    )

    ranked_rows = sorted(
        usable_rows,
        key=lambda row: (
            abs(row.correction_yaw_deg),
            row.final_p90_nn_m if np.isfinite(row.final_p90_nn_m) else -np.inf,
        ),
        reverse=True,
    )
    print("  worst_pairs:")
    for row in ranked_rows[:top_pairs]:
        print(
            "    "
            f"raw={row.source_start}-{row.source_end} -> {row.target_start}-{row.target_end} "
            f"dt={row.target_time - row.source_time:.3f}s "
            f"pred_t={row.predicted_translation_m:.3f}m "
            f"pred_yaw={row.predicted_yaw_deg:.2f}deg "
            f"init_med={row.initial_median_nn_m:.3f}m "
            f"init_p90={row.initial_p90_nn_m:.3f}m "
            f"fit={row.icp_fitness:.3f} "
            f"rmse={row.icp_rmse_m:.3f}m "
            f"corr_t={row.correction_translation_m:.3f}m "
            f"corr_yaw={row.correction_yaw_deg:.2f}deg "
            f"final_p90={row.final_p90_nn_m:.3f}m "
            f"pts={row.source_points}/{row.target_points}"
        )


def main():
    args = parse_args()
    input_path = Path(args.input).expanduser()
    data = np.load(input_path, allow_pickle=True)
    pose_timeline = PoseTimeline.from_npz(data)
    if pose_timeline is None:
        raise ValueError(f"{input_path} does not contain state data.")

    total_frames = len(data["frames"])
    if args.submap_frames < 1:
        raise ValueError("--submap-frames must be >= 1")
    if args.pair_separation < 1:
        raise ValueError("--pair-separation must be >= 1")
    if args.pair_stride < 1:
        raise ValueError("--pair-stride must be >= 1")
    if args.count < 1:
        raise ValueError("--count must be >= 1")

    final_target_end = (
        args.start
        + (args.count - 1) * args.pair_stride
        + args.pair_separation
        + args.submap_frames
    )
    if args.start < 0 or final_target_end > total_frames:
        raise ValueError(
            f"Requested raw-frame window exceeds the bag: need frames up to {final_target_end - 1}, "
            f"but only {total_frames} are available."
        )

    frame_cache = FrameCache(data)
    timelines = mode_timelines(pose_timeline)
    results: dict[str, list[PairResult]] = {mode_name: [] for mode_name in timelines}

    for pair_idx in range(args.count):
        source_start = args.start + pair_idx * args.pair_stride
        target_start = source_start + args.pair_separation
        for mode_name, mode_timeline in timelines.items():
            row = submap_pair_result(
                frame_cache=frame_cache,
                pose_timeline=mode_timeline,
                sensor_to_body_rotation=SC_LIDAR_TO_BODY_ROTATION,
                source_start=source_start,
                target_start=target_start,
                frame_count=args.submap_frames,
                mode_name=mode_name,
                args=args,
            )
            results[mode_name].append(row)

    print(f"input={input_path}")
    print(
        f"submap_frames={args.submap_frames} pair_separation={args.pair_separation} "
        f"pair_stride={args.pair_stride} count={args.count} start={args.start}"
    )
    print(
        f"filters: range=[{args.min_range:.1f}, {args.max_range:.1f}]m "
        f"z=[{args.z_min:.1f}, {args.z_max:.1f}]m voxel={args.voxel_size:.2f}m "
        f"sample_points={args.sample_points} icp_threshold={args.icp_threshold:.2f}m"
    )

    for mode_name in ("translation_only", "yaw_only", "full"):
        print_mode_report(mode_name, results[mode_name], args.top_pairs)


if __name__ == "__main__":
    main()
