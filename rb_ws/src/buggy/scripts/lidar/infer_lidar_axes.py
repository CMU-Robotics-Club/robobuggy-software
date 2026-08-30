#! /usr/bin/env python3

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from lidar_deskew import PoseTimeline


def parse_args():
    parser = argparse.ArgumentParser(
        description="Infer LiDAR horizontal axis conventions from scan-to-scan scene motion."
    )
    parser.add_argument(
        "--input",
        default="sc_feb_21_26_roll_1.npz",
        help="NPZ file produced by bag_to_numpy.py.",
    )
    parser.add_argument(
        "--start",
        type=int,
        default=3600,
        help="First raw frame index to inspect.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=80,
        help="Number of consecutive raw-frame pairs to inspect.",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=2,
        help="Raw-frame stride between inspected pairs.",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=45.0,
        help="Maximum XY range for ICP points.",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.25,
        help="Voxel downsample size before ICP.",
    )
    parser.add_argument(
        "--icp-threshold",
        type=float,
        default=1.5,
        help="ICP correspondence threshold.",
    )
    return parser.parse_args()


def filtered_cloud(points: np.ndarray, max_range: float, voxel_size: float):
    points = np.asarray(points, dtype=np.float64)[:, :3]
    mask = np.isfinite(points).all(axis=1)
    points = points[mask]
    xy_range = np.linalg.norm(points[:, :2], axis=1)
    points = points[(xy_range > 2.0) & (xy_range < max_range)]
    points = points[(points[:, 2] > -2.0) & (points[:, 2] < 3.0)]

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    return cloud.voxel_down_sample(voxel_size)


def yaw_from_rotation(rotation: np.ndarray) -> float:
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))


def state_delta_in_world(pose_timeline: PoseTimeline, t0: float, t1: float) -> np.ndarray:
    positions, _ = pose_timeline.sample(np.array([t0, t1], dtype=np.float64))
    return positions[1] - positions[0]


def summarize_vectors(label: str, vectors: np.ndarray):
    if len(vectors) == 0:
        print(f"{label}: no usable vectors")
        return

    xy = vectors[:, :2]
    norms = np.linalg.norm(xy, axis=1)
    valid = norms > 1e-9
    xy = xy[valid]
    norms = norms[valid]
    if len(xy) == 0:
        print(f"{label}: no nonzero XY vectors")
        return

    unit = xy / norms[:, None]
    mean_unit = unit.mean(axis=0)
    mean_unit /= np.linalg.norm(mean_unit)
    angles = np.degrees(np.arctan2(unit[:, 1], unit[:, 0]))
    print(
        f"{label}: mean_dir=({mean_unit[0]:.3f}, {mean_unit[1]:.3f}) "
        f"mean_angle={np.degrees(np.arctan2(mean_unit[1], mean_unit[0])):.1f}deg "
        f"median_step={np.median(norms):.3f}m "
        f"angle_std={np.std(angles):.1f}deg"
    )


def main():
    args = parse_args()
    input_path = Path(args.input).expanduser()
    npz_data = np.load(input_path, allow_pickle=True)
    pose_timeline = PoseTimeline.from_npz(npz_data)
    if pose_timeline is None:
        raise ValueError(f"{input_path} does not contain state data.")

    frames = npz_data["frames"]
    timestamps = np.asarray(npz_data["timestamps"], dtype=np.float64)
    end = min(args.start + args.count * args.step, len(frames) - args.step)

    apparent_scene_translations = []
    vehicle_translations_world = []
    accepted = []

    for idx in range(args.start, end, args.step):
        source = filtered_cloud(frames[idx], args.max_range, args.voxel_size)
        target = filtered_cloud(frames[idx + args.step], args.max_range, args.voxel_size)
        if len(source.points) < 100 or len(target.points) < 100:
            continue

        result = o3d.pipelines.registration.registration_icp(
            source,
            target,
            args.icp_threshold,
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40),
        )
        if result.fitness < 0.2:
            continue

        transform = result.transformation
        apparent_t = transform[:3, 3]
        apparent_scene_translations.append(apparent_t)
        vehicle_translations_world.append(
            state_delta_in_world(pose_timeline, timestamps[idx], timestamps[idx + args.step])
        )
        accepted.append((idx, result.fitness, result.inlier_rmse, apparent_t, yaw_from_rotation(transform[:3, :3])))

    apparent_scene_translations = np.asarray(apparent_scene_translations, dtype=np.float64)
    vehicle_translations_world = np.asarray(vehicle_translations_world, dtype=np.float64)

    print(f"input={input_path}")
    print(f"raw_frame_range={args.start}-{max(args.start, end - args.step)} step={args.step}")
    print(f"accepted_pairs={len(accepted)}")
    summarize_vectors("apparent_scene_motion_lidar_xy", apparent_scene_translations)
    summarize_vectors("vehicle_motion_world_xy", vehicle_translations_world)

    if len(apparent_scene_translations):
        vehicle_in_lidar = -apparent_scene_translations
        summarize_vectors("inferred_vehicle_motion_lidar_xy", vehicle_in_lidar)
        print("sample_pairs:")
        for idx, fitness, rmse, apparent_t, apparent_yaw in accepted[:12]:
            vehicle_t = -apparent_t
            print(
                f"  raw={idx}->{idx + args.step} "
                f"fitness={fitness:.3f} rmse={rmse:.3f} "
                f"scene_t=({apparent_t[0]:.3f},{apparent_t[1]:.3f},{apparent_t[2]:.3f}) "
                f"vehicle_t_lidar=({vehicle_t[0]:.3f},{vehicle_t[1]:.3f},{vehicle_t[2]:.3f}) "
                f"apparent_yaw={np.degrees(apparent_yaw):.2f}deg"
            )


if __name__ == "__main__":
    main()
