#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SetSteer(Node):
    """
    Debug node for testing steering calibration.
    Starts an interactive terminal to publish steering angles to input/steering.

    This node uses threading to separate the ROS2 spinning from the interactive input.
    The input loop runs on a separate thread to avoid blocking ROS2 operations.
    """

    def __init__(self) -> None:
        super().__init__("set_steer")
        self.steer_publisher = self.create_publisher(Float64, "input/steering", 10)
        self.running = True
        self.tty_in = None
        self.tty_out = None

        # Try to open the controlling terminal directly
        try:
            self.tty_in = open("/dev/tty", "r")
            self.tty_out = open("/dev/tty", "w")
        except (OSError, IOError):
            self.get_logger().error("Cannot access terminal (/dev/tty) for input.")
            self.running = False
            return

        self.get_logger().info(
            "set_steer initialized; type a number and press Enter to publish; 'q' to quit"
        )

    def run_input_loop(self) -> None:
        """
        Run a blocking interactive loop that reads angles from stdin and publishes.
        This runs in a separate thread.
        """
        if not self.running or self.tty_in is None or self.tty_out is None:
            return

        try:
            while self.running and rclpy.ok():
                try:
                    self.tty_out.write("Steer angle (deg) > ")
                    self.tty_out.flush()
                    raw = self.tty_in.readline().strip()
                except (EOFError, KeyboardInterrupt):
                    # End of input, exit
                    break

                # Empty line -> ignore and continue prompting
                if not raw:
                    continue

                cmd = raw.strip().lower()

                # Quit the CLI
                if cmd in ("q", "quit", "exit"):
                    break

                # Try to parse a floating point angle
                try:
                    angle = float(raw)
                except ValueError:
                    self.get_logger().warning(
                        "Invalid input; enter a numeric angle (degrees) or 'q' to quit"
                    )
                    continue

                # Publish the steering command
                msg = Float64()
                msg.data = angle
                self.steer_publisher.publish(msg)
                # Write confirmation to terminal instead of using logger to avoid interference
                self.tty_out.write(f"Published steer: {angle}\n")
                self.tty_out.flush()

        except KeyboardInterrupt:
            # allow Ctrl-C to exit the loop
            pass
        finally:
            self.stop()

    def stop(self):
        self.running = False
        if self.tty_in:
            self.tty_in.close()
        if self.tty_out:
            self.tty_out.close()


def main(args=None):
    rclpy.init(args=args)

    node = SetSteer()

    # Start the input loop in a separate thread
    input_thread = threading.Thread(target=node.run_input_loop, daemon=True)
    input_thread.start()

    try:
        # Spin the node to handle ROS2 callbacks and keep it alive
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the node and wait for input thread to finish
        node.stop()
        if input_thread.is_alive():
            input_thread.join(timeout=1.0)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
