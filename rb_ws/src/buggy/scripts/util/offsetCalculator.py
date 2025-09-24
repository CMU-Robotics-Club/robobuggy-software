#! /usr/bin/env python3
import argparse
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
import matplotlib.pyplot as plt

def main():
    # Read in bag path from command line
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file", help="Path to bag file")
    args = parser.parse_args()

    CROSS_TRACK_GAIN = 1.3

    # Open ROS2 bag
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=args.bag_file, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    # Get topic and message type

    error_topic = "/SC/controller/debug/stanley_error"
    state_topic = "/SC/self/state"
    velocity = 0.0001

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    error_type = get_message(topic_types[error_topic])
    state_type = get_message(topic_types[state_topic])

    # Create data structure
    offsets = []

    # Loop through bag
    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic == error_topic:
            msg = deserialize_message(data, error_type)
            error = msg.position.y

            if velocity > 0.1:
                offsets.append(np.rad2deg(np.arctan(error * CROSS_TRACK_GAIN/velocity)))
        elif topic == state_topic:
            msg = deserialize_message(data, state_type)
            velocity = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
    plt.plot(offsets)  # plotting by columns
    plt.ylim(ymin=0)
    plt.ylim(ymax=20)
    plt.savefig("test.jpg")

if __name__ == "__main__":
    main()
