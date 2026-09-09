#!/usr/bin/env python3
"""
radio_detection_adapter.py
--------------------------
Turns NAND's relayed GPS position (debug/NAND_radio, buggy/SCRadioNANDMsg) into a
"radio" observation for opponent_tracker.py (DECISIONS.md D5).

What the packet carries: easting, northing, gps_seqnum, rx_rssi, gps_fix,
auton_steer. What it does NOT carry: a capture timestamp or a covariance. So:
  * header.stamp is the RECEIPT time and source_age_unknown is true;
  * repeated sequence numbers are dropped; a decreasing sequence number is
    treated as a reset and bumps source_generation;
  * gps_fix is passed through as raw_fix_type. Its enumeration belongs to
    NAND's firmware and is NOT the Microstrain MIP table; the position std comes
    from the parameter table `fix_std_table` ("<fix>:<std_m>" entries) and the
    default otherwise. Nothing here assumes 5/6 mean RTK.

Off by default in every launch: enable it only where the radio is known to be
fitted and the sequence and quality semantics have been checked on a log.
"""

import rclpy
from rclpy.node import Node

from buggy.msg import DetectionArrayMsg, DetectionMsg, SCRadioNANDMsg


class RadioDetectionAdapter(Node):
    def __init__(self):
        super().__init__("radio_detection_adapter")
        self.declare_parameter("radio_topic", "debug/NAND_radio")
        self.declare_parameter("output_topic", "radio/detection_array")
        self.declare_parameter("default_std_m", 1.5)
        self.declare_parameter("fix_std_table", [""])   # e.g. ["6:0.05", "5:0.3"] once verified
        self.declare_parameter("object_id", "NAND")

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.default_std = float(p("default_std_m"))
        self.std_table = {}
        for entry in p("fix_std_table"):
            if ":" in str(entry):
                fix, std = str(entry).split(":", 1)
                self.std_table[int(fix)] = float(std)
        self.object_id = str(p("object_id"))
        self.last_seq = None
        self.generation = 0
        self.dropped_repeats = 0

        self.create_subscription(SCRadioNANDMsg, p("radio_topic"), self.on_radio, 5)
        self.pub = self.create_publisher(DetectionArrayMsg, p("output_topic"), 1)
        self.get_logger().info(f"radio adapter: {p('radio_topic')} -> {p('output_topic')}; std table {self.std_table}")

    def on_radio(self, msg: SCRadioNANDMsg):
        seq = int(msg.gps_seqnum)
        if self.last_seq is not None:
            if seq == self.last_seq:
                self.dropped_repeats += 1
                return
            if seq < self.last_seq:
                self.generation += 1
                self.get_logger().warn(f"radio sequence went backwards ({self.last_seq} -> {seq}); generation {self.generation}")
        self.last_seq = seq
        std = self.std_table.get(int(msg.gps_fix), self.default_std)

        out = DetectionArrayMsg()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "utm"
        out.source = "radio"
        out.source_age_unknown = True
        out.motion_compensated = False
        out.source_generation = int(self.generation)
        out.sequence_id = int(seq)
        out.raw_fix_type = int(msg.gps_fix)
        out.rx_rssi = float(msg.rx_rssi)
        det = DetectionMsg()
        det.position.x = float(msg.easting)
        det.position.y = float(msg.northing)
        det.position_covariance = [std * std, 0.0, 0.0, 0.0, std * std, 0.0, 0.0, 0.0, 0.0]
        det.object_id = self.object_id
        det.class_id = "buggy"
        det.confidence = 1.0
        det.observed = True
        out.detections.append(det)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RadioDetectionAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
