from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class PoseTimeline:
    timestamps: np.ndarray
    positions: np.ndarray
    orientations_rpy: np.ndarray
    _unwrapped_orientations_rpy: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        timestamps = np.asarray(self.timestamps, dtype=np.float64).reshape(-1)
        positions = np.asarray(self.positions, dtype=np.float64)
        orientations_rpy = np.asarray(self.orientations_rpy, dtype=np.float64)

        if timestamps.ndim != 1:
            raise ValueError("Pose timestamps must be one-dimensional.")
        if positions.ndim != 2 or positions.shape[1] != 3:
            raise ValueError("Pose positions must have shape (N, 3).")
        if orientations_rpy.ndim != 2 or orientations_rpy.shape[1] != 3:
            raise ValueError("Pose orientations must have shape (N, 3).")
        if len(timestamps) != len(positions) or len(timestamps) != len(orientations_rpy):
            raise ValueError("Pose timeline arrays must all have the same length.")
        if len(timestamps) < 2:
            raise ValueError("Need at least two state samples to interpolate poses.")

        order = np.argsort(timestamps, kind="mergesort")
        timestamps = timestamps[order]
        positions = positions[order]
        orientations_rpy = orientations_rpy[order]

        # Repeated timestamps are possible in recorded bags; keeping the newest
        # sample for each duplicated time keeps interpolation well-defined.
        keep_mask = np.ones(len(timestamps), dtype=bool)
        keep_mask[:-1] = timestamps[:-1] != timestamps[1:]

        timestamps = timestamps[keep_mask]
        positions = positions[keep_mask]
        orientations_rpy = orientations_rpy[keep_mask]

        if len(timestamps) < 2:
            raise ValueError("State timeline collapsed to fewer than two unique timestamps.")

        self.timestamps = timestamps
        self.positions = positions
        self.orientations_rpy = orientations_rpy
        self._unwrapped_orientations_rpy = np.unwrap(orientations_rpy, axis=0)

    @classmethod
    def from_npz(cls, npz_data) -> PoseTimeline | None:
        if "state_timestamps" not in npz_data:
            return None

        if "state_positions" not in npz_data:
            return None

        orientation_key = None
        for key in ("state_orientations_rpy", "state_rpy"):
            if key in npz_data:
                orientation_key = key
                break
        if orientation_key is None:
            return None

        return cls(
            timestamps=npz_data["state_timestamps"],
            positions=npz_data["state_positions"],
            orientations_rpy=npz_data[orientation_key],
        )

    def sample(self, query_times) -> tuple[np.ndarray, np.ndarray]:
        query_times = np.asarray(query_times, dtype=np.float64).reshape(-1)
        clamped_times = np.clip(query_times, self.timestamps[0], self.timestamps[-1])

        positions = np.column_stack(
            [
                np.interp(clamped_times, self.timestamps, self.positions[:, axis])
                for axis in range(3)
            ]
        )
        orientations = np.column_stack(
            [
                np.interp(
                    clamped_times,
                    self.timestamps,
                    self._unwrapped_orientations_rpy[:, axis],
                )
                for axis in range(3)
            ]
        )
        return positions, orientations

def normalise_point_times(point_times, expected_count: int) -> np.ndarray:
    if point_times is None:
        return np.zeros(expected_count, dtype=np.float64)

    point_times = np.asarray(point_times, dtype=np.float64).reshape(-1)
    if point_times.size != expected_count:
        raise ValueError(
            f"Point-time array length {point_times.size} does not match point count {expected_count}."
        )
    if point_times.size == 0:
        return point_times

    finite_mask = np.isfinite(point_times)
    if not np.any(finite_mask):
        return np.zeros(expected_count, dtype=np.float64)

    normalised = point_times.copy()
    normalised[~finite_mask] = normalised[finite_mask][0]
    return normalised - np.min(normalised)


def rpy_to_rotation_matrices_batch(rpy: np.ndarray) -> np.ndarray:
    roll = rpy[:, 0]
    pitch = rpy[:, 1]
    yaw = rpy[:, 2]

    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    rotation = np.empty((len(rpy), 3, 3), dtype=np.float64)
    rotation[:, 0, 0] = cy * cp
    rotation[:, 0, 1] = cy * sp * sr - sy * cr
    rotation[:, 0, 2] = cy * sp * cr + sy * sr
    rotation[:, 1, 0] = sy * cp
    rotation[:, 1, 1] = sy * sp * sr + cy * cr
    rotation[:, 1, 2] = sy * sp * cr - cy * sr
    rotation[:, 2, 0] = -sp
    rotation[:, 2, 1] = cp * sr
    rotation[:, 2, 2] = cp * cr
    return rotation


def deskew_frame(
    points,
    frame_timestamp: float | None,
    pose_timeline: PoseTimeline | None,
    point_times=None,
    target_time: float | None = None,
    sensor_to_body_rotation=None,
    sensor_to_body_translation=None,
) -> tuple[np.ndarray, float | None]:
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"Expected points with shape (N, >=3), got {points.shape}")
    points = points[:, :3]

    if len(points) == 0:
        return points, target_time if target_time is not None else frame_timestamp

    if frame_timestamp is None or pose_timeline is None:
        return points.copy(), target_time if target_time is not None else frame_timestamp

    sensor_to_body_rotation = (
        np.eye(3, dtype=np.float64)
        if sensor_to_body_rotation is None
        else np.asarray(sensor_to_body_rotation, dtype=np.float64)
    )
    sensor_to_body_translation = (
        np.zeros(3, dtype=np.float64)
        if sensor_to_body_translation is None
        else np.asarray(sensor_to_body_translation, dtype=np.float64).reshape(3)
    )

    relative_times = normalise_point_times(point_times, len(points))
    source_times = float(frame_timestamp) + relative_times
    reference_time = float(target_time) if target_time is not None else float(np.max(source_times))
    body_points = (
        np.einsum("ij,nj->ni", sensor_to_body_rotation, points)
        + sensor_to_body_translation
    )

    if np.allclose(source_times, source_times[0]):
        source_positions, source_rpy = pose_timeline.sample(np.array([source_times[0]]))
        target_positions, target_rpy = pose_timeline.sample(np.array([reference_time]))

        source_rotation = rpy_to_rotation_matrices_batch(source_rpy)[0]
        target_rotation = rpy_to_rotation_matrices_batch(target_rpy)[0]

        world_points = np.einsum("ij,nj->ni", source_rotation, body_points) + source_positions[0]
        target_body_points = np.einsum(
            "ji,nj->ni",
            target_rotation,
            world_points - target_positions[0],
        )
        corrected_points = np.einsum(
            "ji,nj->ni",
            sensor_to_body_rotation,
            target_body_points - sensor_to_body_translation,
        )
        return corrected_points, reference_time

    source_positions, source_rpy = pose_timeline.sample(source_times)
    target_positions, target_rpy = pose_timeline.sample(np.full(len(points), reference_time))

    source_rotation = rpy_to_rotation_matrices_batch(source_rpy)
    target_rotation = rpy_to_rotation_matrices_batch(target_rpy)

    world_points = np.einsum("nij,nj->ni", source_rotation, body_points) + source_positions
    target_body_points = np.einsum(
        "nji,nj->ni",
        target_rotation,
        world_points - target_positions,
    )
    corrected_points = np.einsum(
        "ji,nj->ni",
        sensor_to_body_rotation,
        target_body_points - sensor_to_body_translation,
    )
    return corrected_points, reference_time


def deskew_frame_block(
    frames,
    frame_timestamps,
    pose_timeline: PoseTimeline | None,
    point_times_seq=None,
    target_time: float | None = None,
    sensor_to_body_rotation=None,
    sensor_to_body_translation=None,
) -> tuple[list[np.ndarray], float | None]:
    frame_count = len(frames)
    if frame_count == 0:
        return [], target_time

    if pose_timeline is None or frame_timestamps is None:
        return [np.asarray(frame, dtype=np.float64)[:, :3] for frame in frames], target_time

    if len(frame_timestamps) != frame_count:
        raise ValueError("Frame timestamp count does not match frame count.")

    if target_time is None:
        last_idx = frame_count - 1
        last_points = np.asarray(frames[last_idx], dtype=np.float64)
        last_point_times = None
        if point_times_seq is not None:
            last_point_times = point_times_seq[last_idx]
        relative_times = normalise_point_times(last_point_times, len(last_points))
        target_time = float(frame_timestamps[last_idx]) + (
            float(np.max(relative_times)) if relative_times.size > 0 else 0.0
        )

    deskewed_frames = []
    for idx, frame in enumerate(frames):
        point_times = None if point_times_seq is None else point_times_seq[idx]
        corrected_frame, _ = deskew_frame(
            frame,
            frame_timestamp=float(frame_timestamps[idx]),
            pose_timeline=pose_timeline,
            point_times=point_times,
            target_time=target_time,
            sensor_to_body_rotation=sensor_to_body_rotation,
            sensor_to_body_translation=sensor_to_body_translation,
        )
        deskewed_frames.append(corrected_frame)

    return deskewed_frames, target_time


def deskew_displacement_stats(original_frames, deskewed_frames) -> dict[str, float]:
    if len(original_frames) != len(deskewed_frames):
        raise ValueError("Original and deskewed frame counts must match.")

    displacement_chunks = []
    total_points = 0
    for original_frame, deskewed_frame in zip(original_frames, deskewed_frames):
        original = np.asarray(original_frame, dtype=np.float64)
        deskewed = np.asarray(deskewed_frame, dtype=np.float64)
        if original.shape != deskewed.shape:
            raise ValueError("Original and deskewed frame shapes must match.")
        if len(original) == 0:
            continue
        displacement_chunks.append(np.linalg.norm(deskewed - original, axis=1))
        total_points += len(original)

    if not displacement_chunks:
        return {
            "point_count": 0.0,
            "mean_m": 0.0,
            "std_m": 0.0,
            "rms_m": 0.0,
            "max_m": 0.0,
        }

    displacements = np.concatenate(displacement_chunks)
    return {
        "point_count": float(total_points),
        "mean_m": float(np.mean(displacements)),
        "std_m": float(np.std(displacements)),
        "rms_m": float(np.sqrt(np.mean(displacements ** 2))),
        "max_m": float(np.max(displacements)),
    }
