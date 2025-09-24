#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SetSteer(Node):
    """
    Debug node for testing steering calibration.
    Starts an interactive terminal to publish steering angles to input/steering.

    - This node runs a simple blocking CLI loop using Python's built-in input().
        input() blocks the current thread until the user types a line and presses Enter.

    - Because input() is blocking, this script performs the CLI loop on the
        main thread and directly publishes messages from that loop.
    """

    def __init__(self) -> None:
        super().__init__("set_steer")
        self.steer_publisher = self.create_publisher(Float64, "input/steering", 10)
        self.get_logger().info(
            "set_steer initialized; type a number and press Enter to publish; 'q' to quit"
        )

    def run(self) -> None:
        """
        Run a blocking interactive loop that reads angles from stdin and publishes.
        """
        try:
            while rclpy.ok():
                try:
                    raw = input("Steer angle (deg) > ")
                except EOFError:
                    # End of input (e.g. Ctrl-D on posix), exit gracefully
                    break

                # Empty line -> ignore and continue prompting
                if not raw:
                    continue

                cmd = raw.strip().lower()
                if cmd in ("q", "quit", "exit"):
                    # Allow the user a convenient way to quit the CLI
                    break

                # Try to parse a floating point angle
                try:
                    angle = float(raw)
                except ValueError:
                    # Friendly guidance for incorrect input
                    self.get_logger().warning(
                        "Invalid input; please enter a numeric angle (degrees) or 'q' to quit"
                    )
                    continue

                # Publish the steering command
                msg = Float64()
                msg.data = angle
                self.steer_publisher.publish(msg)
                self.get_logger().info(f"Published steer: {angle}")

        except KeyboardInterrupt:
            # allow Ctrl-C to exit the loop cleanly
            pass


def main(args=None):
    rclpy.init(args=args)

    node = SetSteer()

    # rclpy.spin(node) blocks the ROS executor and is intended for callback-driven nodes;
    # it doesn't provide a place to put a blocking stdin loop on the same thread.
    node.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
