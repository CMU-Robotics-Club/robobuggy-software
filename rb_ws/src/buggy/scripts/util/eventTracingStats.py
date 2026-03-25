#! /usr/bin/env python3
import argparse
from dataclasses import dataclass
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
import matplotlib.pyplot as plt

from util.constants import TracingEvent

@dataclass
class MsgTimes:
    serial: int
    stateconv: int
    ctrl_rx: int
    ctrl_tx_min: int
    ctrl_tx_max: int
    setsteer_min: int
    setsteer_max: int

    def set_ctrl_tx(self, t):
        if self.ctrl_tx_min is None:
            self.ctrl_tx_min = t
            self.ctrl_tx_max = t
        else:
            self.ctrl_tx_min = min(self.ctrl_tx_min, t)
            self.ctrl_tx_max = max(self.ctrl_tx_max, t)
    
    def set_setsteer(self, t):
        if self.setsteer_min is None:
            self.setsteer_min = t
            self.setsteer_max = t
        else:
            self.setsteer_min = min(self.setsteer_min, t)
            self.setsteer_max = max(self.setsteer_max, t)

    def is_complete(self):
        return self.serial is not None and\
                self.stateconv is not None and\
                self.ctrl_rx is not None and\
                self.ctrl_tx_min is not None and\
                self.ctrl_tx_max is not None and\
                self.setsteer_min is not None and\
                self.setsteer_max is not None



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
    evt_topic = "debug/trace_events"

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    evt_type = get_message(topic_types[evt_topic])

    msgs = {}

    # Loop through bag
    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic == evt_topic:
            msg = deserialize_message(data, evt_type)
            t = msg.header.stamp.sec * int(1e9) + msg.header.stamp.nanosec
            frame = int(msg.header.frame_id)

            if frame not in msgs:
                msgs[frame] = MsgTimes(
                    serial=None, 
                    stateconv=None, 
                    ctrl_rx=None, 
                    ctrl_tx_min=None, 
                    setsteer_min=None, 
                    ctrl_tx_max=None, 
                    setsteer_max=None
                )
            
            if msg.evt_type == TracingEvent.SERIAL_RXTX:
                msgs[frame].serial = t
            elif msg.evt_type == TracingEvent.STATECONV_RXTX:
                msgs[frame].stateconv = t
            elif msg.evt_type == TracingEvent.CTRL_RX:
                msgs[frame].set_ctrl_rx(t)
            elif msg.evt_type == TracingEvent.CTRL_TX:
                msgs[frame].set_ctrl_tx(t)
            elif msg.evt_type == TracingEvent.SETSTEER_RXTX:
                msgs[frame].set_setsteer(t)

    for frame, msg in list(msgs.items()):
        if not msg.is_complete():
            del msgs[frame]

    t = np.array(sorted(msgs.keys()))
    t -= t[0]

    # Construct series for each tracingevent
    serial_to_stateconv = []
    stateconv_to_ctrlrx = []
    ctrlrx_to_ctrltx_min = []
    ctrlrx_to_ctrltx_max = []
    ctrltx_to_setsteer = []

    for frame in t:
        msg = msgs[frame]
        serial_to_stateconv.append((msg.stateconv - msg.serial) * 1e-9)
        stateconv_to_ctrlrx.append((msg.ctrl_rx - msg.stateconv) * 1e-9)
        ctrlrx_to_ctrltx_min.append((msg.ctrl_tx_min - msg.ctrl_rx) * 1e-9)
        ctrlrx_to_ctrltx_max.append((msg.ctrl_tx_max - msg.ctrl_rx) * 1e-9)

        # avg min-min and max-max time for single set_steer series
        ctrltx_to_setsteer.append(
            (msg.setsteer_min + msg.setsteer_max - (msg.ctrl_tx_min + msg.ctrl_tx_max)) * 1e-9 / 2
        )

    serial_to_stateconv = np.array(serial_to_stateconv)
    stateconv_to_ctrlrx = np.array(stateconv_to_ctrlrx)
    ctrlrx_to_ctrltx_min = np.array(ctrlrx_to_ctrltx_min)
    ctrlrx_to_ctrltx_max = np.array(ctrlrx_to_ctrltx_max)
    ctrltx_to_setsteer = np.array(ctrltx_to_setsteer)

    # Plotting
    plt.figure(figsize=(12, 8))
    plt.plot(t, serial_to_stateconv, label='Serial -> StateConv')
    plt.plot(t, stateconv_to_ctrlrx, label='StateConv -> Ctrl RX')
    plt.plot(t, ctrlrx_to_ctrltx_max, label='Ctrl RX -> Ctrl TX (Max)')
    plt.plot(t, ctrlrx_to_ctrltx_min, label='Ctrl RX -> Ctrl TX (Min)')
    plt.plot(t, ctrltx_to_setsteer, label='Ctrl TX -> SetSteer')
    
    plt.xlabel('Firmware send time (s)')
    plt.ylabel('Latency (s)')
    plt.title('Control Stack Latency Breakdown')
    plt.legend()
    plt.grid(True)
    plt.savefig("latency_stats.png")
    
    # Print avg for each series
    print(f"Average Serial -> StateConv: {np.mean(serial_to_stateconv):.6f}s")
    print(f"Average StateConv -> Ctrl RX: {np.mean(stateconv_to_ctrlrx):.6f}s")
    print(f"Average Ctrl RX -> Ctrl TX (Max): {np.mean(ctrlrx_to_ctrltx_max):.6f}s")
    print(f"Average Ctrl RX -> Ctrl TX (Min): {np.mean(ctrlrx_to_ctrltx_min):.6f}s")
    print(f"Average Ctrl TX -> SetSteer: {np.mean(ctrltx_to_setsteer):.6f}s")
    
    # Tail CDF for each series, log x axis
    plt.figure(figsize=(12, 8))
    for label, series in [
        ('Serial -> StateConv', serial_to_stateconv),
        ('StateConv -> Ctrl RX', stateconv_to_ctrlrx),
        ('Ctrl RX -> Ctrl TX (Max)', ctrlrx_to_ctrltx_max),
        ('Ctrl RX -> Ctrl TX (Min)', ctrlrx_to_ctrltx_min),
        ('Ctrl TX -> SetSteer', ctrltx_to_setsteer)
    ]:
        sorted_series = np.sort(series)
        y = 1.0 - np.arange(len(sorted_series)) / float(len(sorted_series))
        plt.plot(sorted_series, y, label=label)

    plt.xscale('log')
    plt.xlabel('Latency (s)')
    plt.ylabel('1 - CDF')
    plt.title('Latency Tail CDF')
    plt.legend()
    plt.grid(True, which="both", ls="-")
    plt.savefig("latency_tail_cdf.png")
    
    

if __name__ == "__main__":
    main()
