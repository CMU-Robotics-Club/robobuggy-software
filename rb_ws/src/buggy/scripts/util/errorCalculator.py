#! /usr/bin/env python3
import argparse
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np

"""
This script calculates cross-track errors for a robotic buggy using data from a ROS2 bag file.

Purpose:
    - To compute and summarize the cross-track error (distance from the desired path) during autonomous operation.

How it works:
    - Inputs:
        1. buggy_name: The ROS name for the buggy (used to construct topic names)
        2. bag_file: Path to the ROS2 bag file containing recorded messages
    - The script reads three topics from the bag:
        - /<buggy_name>/controller/debug/stanley_error: Contains error messages (position.y)
        - /<buggy_name>/self/state: Contains velocity information
        - /<buggy_name>/debug/firmware: Contains autonomous mode status
    - For each message, if the buggy is in autonomous mode and moving above a velocity threshold, the absolute cross-track error is recorded.
    - Outputs:
        - Prints the shape, max, mean, and standard deviation of the error array.

Note:
    There is an inherent off-by-one error in the results due to the non-deterministic order in which packets are recorded and read from the bag file. This means that the error and state messages may not always be perfectly synchronized, leading to a possible mismatch by one message.
"""

def main():
    # Read in bag path from command line
    parser = argparse.ArgumentParser()
    parser.add_argument("buggy_name", help="ROS Name for buggy")
    parser.add_argument("bag_file", help="Path to bag file")
    parser.add_argument("start_time", help="Start time of segment to analyze (in seconds)", default=0.0, type=float)
    args = parser.parse_args()


    # Open ROS2 bag
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=args.bag_file, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    # Get topic and message type

    error_topic = f"/{args.buggy_name}/controller/debug/stanley_error"
    state_topic = f"/{args.buggy_name}/self/state"
    auton_topic = f"/{args.buggy_name}/debug/firmware"

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    error_type = get_message(topic_types[error_topic])
    state_type = get_message(topic_types[state_topic])
    auton_type = get_message(topic_types[auton_topic])

    in_auton = True
    cur_velocity = 0.0001
    cur_time_delta = 0.0
    init_time = None

    # Create data structure
    errors = []

    # Loop through bag
    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic == error_topic:
            msg = deserialize_message(data, error_type)
            if (cur_velocity > 0.1 and in_auton and
                cur_time_delta > args.start_time):
                errors.append(abs(msg.position.y))
        elif topic == state_topic:
            msg = deserialize_message(data, state_type)
            cur_velocity = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
            time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
            if init_time is None:
                init_time = time
            cur_time_delta = time - init_time
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
