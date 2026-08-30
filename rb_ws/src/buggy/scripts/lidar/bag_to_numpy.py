#! /usr/bin/env python3

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation
from tqdm import tqdm

"""
This script extracts Velodyne LiDAR PointCloud2 data from a ROS 2 MCAP bag
and converts it into NumPy arrays for offline analysis.

Purpose:
    - Read /velodyne_points from a ROS2 MCAP bag
    - Flatten each PointCloud2 message to per-point rows
    - Export convenient arrays for xyz, intensity, ring, and time
    - Preserve frame timestamps and per-point timing offsets for deskew
    - Export odometry state so playback can motion-compensate merged scans

Output:
    velodyne_points.npz containing:
        - frames: list of (N,3) float32 arrays
        - intensities/rings/point_times: flattened per-point auxiliary fields
        - timestamps: list of frame timestamps (seconds)
        - PointCloud2 field-layout metadata
        - state_timestamps/state_positions/state_orientations_rpy for deskew
        - state_orientations_raw_xyzw preserving the original orientation fields
Notes:
    - Each LiDAR scan may have a different number of valid returns
"""
def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pointcloud2_schema(msg) -> dict:
    return {
        "field_names": np.array([field.name for field in msg.fields], dtype=object),
        "field_offsets": np.array([field.offset for field in msg.fields], dtype=np.int32),
        "field_datatypes": np.array([field.datatype for field in msg.fields], dtype=np.int32),
        "field_counts": np.array([field.count for field in msg.fields], dtype=np.int32),
        "height": int(msg.height),
        "width": int(msg.width),
        "point_step": int(msg.point_step),
        "row_step": int(msg.row_step),
        "is_bigendian": bool(msg.is_bigendian),
        "is_dense": bool(msg.is_dense),
    }


def point_layout_matches(reference: dict, candidate: dict) -> bool:
    scalar_keys = ("point_step", "is_bigendian")
    array_keys = ("field_names", "field_offsets", "field_datatypes", "field_counts")
    return (
        all(reference[key] == candidate[key] for key in scalar_keys)
        and all(np.array_equal(reference[key], candidate[key]) for key in array_keys)
    )


def extract_lidar_frame(msg):
    schema = pointcloud2_schema(msg)
    dtype = point_cloud2.dtype_from_fields(msg.fields, msg.point_step)
    point_count = int(msg.width) * int(msg.height)

    structured = np.frombuffer(msg.data, dtype=dtype, count=point_count)
    if msg.is_bigendian and structured.dtype.byteorder not in ("=", "|"):
        structured = structured.byteswap().newbyteorder()
    elif msg.is_bigendian:
        structured = structured.byteswap().view(structured.dtype.newbyteorder())

    structured = structured.copy()

    xyz = np.stack(
        (
            structured["x"],
            structured["y"],
            structured["z"],
        ),
        axis=-1,
    ).astype(np.float32, copy=False)
    valid_mask = np.isfinite(xyz).all(axis=-1)
    xyz = xyz[valid_mask]

    field_names = list(schema["field_names"])

    intensities = None
    if "intensity" in field_names:
        intensities = np.asarray(structured["intensity"], dtype=np.float32)[valid_mask]

    rings = None
    if "ring" in field_names:
        rings = np.asarray(structured["ring"], dtype=np.uint16)[valid_mask]

    point_times = None
    if "time" in field_names:
        point_times = np.asarray(structured["time"], dtype=np.float32)[valid_mask]

    return xyz, intensities, rings, point_times, schema


def extract_state_pose(msg):
    position = np.array(
        [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ],
        dtype=np.float64,
    )

    ox = msg.pose.pose.orientation.x
    oy = msg.pose.pose.orientation.y
    oz = msg.pose.pose.orientation.z
    ow = msg.pose.pose.orientation.w

    orientation_raw_xyzw = np.array([ox, oy, oz, ow], dtype=np.float64)
    quat_norm = np.linalg.norm(orientation_raw_xyzw)
    if np.isfinite(quat_norm) and quat_norm > 1e-6 and np.isclose(quat_norm, 1.0, rtol=1e-3, atol=1e-3):
        orientation_rpy = Rotation.from_quat(orientation_raw_xyzw).as_euler("xyz")
    elif abs(ow) <= 1e-9:
        orientation_rpy = np.array([ox, oy, oz], dtype=np.float64)
    else:
        raise ValueError(
            "State orientation is neither a unit quaternion nor the legacy w=0 Euler encoding."
        )

    return position, orientation_raw_xyzw, np.asarray(orientation_rpy, dtype=np.float64)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file", help="Path to ROS2 MCAP bag")
    parser.add_argument(
        "--lidar-topic",
        default="/velodyne_points",
        help="PointCloud2 topic to export.",
    )
    parser.add_argument(
        "--state-topic",
        default="/ekf/odometry_earth",
        help="Odometry topic used for playback deskew.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Output .npz path. Defaults to the bag path with a .npz suffix.",
    )
    args = parser.parse_args()

    bag_path = Path(args.bag_file).expanduser()
    lidar_topic = args.lidar_topic
    state_topic = args.state_topic.strip()
    output_path = Path(args.output).expanduser() if args.output else bag_path.with_suffix(".npz")

    # Open ROS2 bag
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id="mcap"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    # Resolve message type
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if lidar_topic not in topic_types:
        raise RuntimeError(f"Topic {lidar_topic} not found in bag")

    lidar_msg_type = get_message(topic_types[lidar_topic])
    state_msg_type = None
    if state_topic:
        if state_topic in topic_types:
            state_msg_type = get_message(topic_types[state_topic])
        else:
            print(f"Warning: state topic {state_topic} not found in bag. Deskew metadata will be omitted.")

    frames = []
    intensities = []
    rings = []
    timestamps = []
    point_times = []
    has_point_times = False
    has_intensity = False
    has_ring = False
    state_timestamps = []
    state_positions = []
    state_orientations_raw_xyzw = []
    state_orientations_rpy = []
    schema_reference = None
    # Get total message count for the progress bar
    metadata = reader.get_metadata()
    total_msgs = sum(t.message_count for t in metadata.topics_with_message_count if t.topic_metadata.name == lidar_topic)

    # Read bag
    pbar = tqdm(total=total_msgs, desc="Extracting LiDAR frames")

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic != lidar_topic and (state_msg_type is None or topic != state_topic):
            continue

        if topic == state_topic:
            msg = deserialize_message(data, state_msg_type)
            state_timestamps.append(stamp_to_seconds(msg.header.stamp))
            position, orientation_raw_xyzw, orientation_rpy = extract_state_pose(msg)
            state_positions.append(position)
            state_orientations_raw_xyzw.append(orientation_raw_xyzw)
            state_orientations_rpy.append(orientation_rpy)
            continue

        pbar.update(1)
        msg = deserialize_message(data, lidar_msg_type)

        cloud_np, scan_intensity, scan_ring, scan_point_times, schema = extract_lidar_frame(msg)
        if schema_reference is None:
            schema_reference = schema
        elif not point_layout_matches(schema_reference, schema):
            raise RuntimeError(
                "PointCloud2 field layout changed within the bag; export expects consistent point fields."
            )

        frames.append(cloud_np)
        timestamps.append(stamp_to_seconds(msg.header.stamp))
        if scan_intensity is not None:
            has_intensity = True
            intensities.append(scan_intensity)
        else:
            intensities.append(np.zeros(len(cloud_np), dtype=np.float32))
        if scan_ring is not None:
            has_ring = True
            rings.append(scan_ring)
        else:
            rings.append(np.zeros(len(cloud_np), dtype=np.uint16))
        if scan_point_times is not None:
            has_point_times = True
            point_times.append(scan_point_times)
        else:
            point_times.append(np.zeros(len(cloud_np), dtype=np.float32))
    
    pbar.close()

    # Save
    output_kwargs = {
        "frames": np.array(frames, dtype=object),
        "timestamps": np.asarray(timestamps, dtype=np.float64),
    }

    if has_point_times:
        output_kwargs["point_times"] = np.array(point_times, dtype=object)
    if has_intensity:
        output_kwargs["intensities"] = np.array(intensities, dtype=object)
    if has_ring:
        output_kwargs["rings"] = np.array(rings, dtype=object)
    if schema_reference is not None:
        output_kwargs["pointcloud2_field_names"] = schema_reference["field_names"]
        output_kwargs["pointcloud2_field_offsets"] = schema_reference["field_offsets"]
        output_kwargs["pointcloud2_field_datatypes"] = schema_reference["field_datatypes"]
        output_kwargs["pointcloud2_field_counts"] = schema_reference["field_counts"]
        output_kwargs["pointcloud2_point_step"] = np.int32(schema_reference["point_step"])
        output_kwargs["pointcloud2_is_bigendian"] = np.bool_(schema_reference["is_bigendian"])
        output_kwargs["pointcloud2_width"] = np.int32(schema_reference["width"])

    if state_timestamps:
        output_kwargs["state_timestamps"] = np.asarray(state_timestamps, dtype=np.float64)
        output_kwargs["state_positions"] = np.asarray(state_positions, dtype=np.float64)
        output_kwargs["state_orientations_raw_xyzw"] = np.asarray(
            state_orientations_raw_xyzw,
            dtype=np.float64,
        )
        output_kwargs["state_orientations_rpy"] = np.asarray(
            state_orientations_rpy,
            dtype=np.float64,
        )

    np.savez_compressed(output_path, **output_kwargs)

    print("Extraction complete.")
    print(f"Frames extracted: {len(frames)}")
    if frames:
        print(f"Example frame shape: {frames[0].shape}")
    print(f"Timestamps exported: {len(timestamps)}")
    print(f"State samples exported: {len(state_timestamps)}")
    print(f"Saved to: {output_path}")

if __name__ == "__main__":
    main()
