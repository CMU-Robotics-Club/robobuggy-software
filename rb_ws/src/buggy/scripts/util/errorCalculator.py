#! /usr/bin/env python3
import argparse
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np

def main():
    # Read in bag path from command line
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file", help="Path to bag file")
    args = parser.parse_args()


    # Open ROS2 bag
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=args.bag_file, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    # Get topic and message type

    error_topic = "/SC/controller/debug/stanley_error"
    state_topic = "/SC/self/state"
    auton_topic = "/SC/debug/firmware"

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    error_type = get_message(topic_types[error_topic])
    state_type = get_message(topic_types[state_topic])
    auton_type = get_message(topic_types[auton_topic])

    in_auton = True
    cur_velocity = 0.0001

    # Create data structure
    errors = []

    # Loop through bag
    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic == error_topic:
            msg = deserialize_message(data, error_type)
            if cur_velocity > 0.1 and in_auton:
                errors.append(abs(msg.position.y))
        elif topic == state_topic:
            msg = deserialize_message(data, state_type)
            cur_velocity = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        elif topic == auton_topic:
            msg = deserialize_message(data, auton_type)
            in_auton = msg.use_auton_steer

    errors = np.array(errors)
    print(errors.shape)
    print("CROSS TRACK ERROR STATS (m)")
    print(f"MAX ERROR: {np.max(errors)}")
    print(f"AVG ERROR: {np.mean(errors)}, STDDEV ERROR: {np.std(errors)}")

if __name__ == "__main__":
    main()
