#! /usr/bin/env python3

import argparse
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2
import numpy as np
import pyvista as pv

"""
This script extracts Velodyne LiDAR PointCloud2 data from a ROS 2 MCAP bag
and converts it into NumPy arrays for offline analysis.

Purpose:
    - Read /velodyne_points from a ROS2 MCAP bag
    - Convert each LiDAR frame into a NumPy array (x, y, z, intensity)
    - Save results in a memory-safe NPZ file

Output:
    velodyne_points.npz containing:
        - points: list of (N,4) float32 arrays
        - timestamps: list of float timestamps (seconds)
python
Notes:
    - Each LiDAR scan may have a different number of points
    - This avoids flattening everything into one massive array
"""

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file", help="Path to ROS2 MCAP bag")
    args = parser.parse_args()

    lidar_topic = "/velodyne_points"

    # Open ROS2 bag
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=args.bag_file,
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

    frames = []

    # Read bag

    # plotter = pv.Plotter()
    i = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic != lidar_topic:
            continue
        i += 1

        msg = deserialize_message(data, lidar_msg_type)

        # Convert PointCloud2 → NumPy
        points = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )

        cloud_struct = np.fromiter(
            points,
            dtype=np.dtype([
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
            ])
        )

        cloud_np = np.stack(
            (cloud_struct["x"],
             cloud_struct["y"],
             cloud_struct["z"]),
            axis=-1
        )

        # print(cloud_np.shape)

        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

                # frame: (N, 3) NumPy array
        # cloud = pv.PolyData(cloud_np)

        # plotter.add_points(
        #     cloud,
        #     point_size=2,
        #     render_points_as_spheres=False
        # )

        frames.append(cloud_np)
        # if i >= 5:
        #     break
    # plotter.add_mesh(pv.Sphere())
    # plotter.add_axes()
    # plotter.show_grid()
    # plotter.show(screenshot='output.png')

    # Save

    # final = np.concatenate(frames)
    # print(final.shape)
    np.savez_compressed(
        args.bag_file[:-5] + ".npz",
        frames=np.array(frames, dtype=object)
    )

    print("Extraction complete.")
    print(f"Frames extracted: {len(frames)}")
    if frames:
        print(f"Example frame shape: {frames[0].shape}")

if __name__ == "__main__":
    main()
