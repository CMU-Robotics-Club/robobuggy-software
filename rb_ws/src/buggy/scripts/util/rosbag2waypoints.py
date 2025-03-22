#! /usr/bin/env python3
import argparse
import uuid
import json
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def main():
    # Read in bag path from command line
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file", help="Path to bag file")
    parser.add_argument("output_file", help="Path to output file")
    parser.add_argument(
        "subsample", help="Subsample rate (1 = don't skip any waypoints)", type=int
    )
    args = parser.parse_args()

    # Open ROS2 bag
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=args.bag_file, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    # Get topic and message type
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_type = get_message(topic_types["/SC/self/state_navsatfix"])

    # Create data structure
    waypoints = []
    i = 0

    # Loop through bag
    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic == "/SC/self/state_navsatfix":
            msg = deserialize_message(data, msg_type)

            # Skip waypoints based on subsample rate
            if i % args.subsample != 0:
                i += 1
                continue
            i += 1

            lon = msg.longitude
            lat = msg.latitude

            waypoints.append(
                {
                    "key": str(uuid.uuid4()),
                    "lat": lat,
                    "lon": lon,
                    "active": False,
                }
            )

    # Write to JSON file
    with open(args.output_file, "w") as f:
        json.dump(waypoints, f, indent=4)

if __name__ == "__main__":
    main()
