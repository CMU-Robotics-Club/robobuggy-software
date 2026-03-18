import rclpy
from rclpy.node import Node

class BuggyLidar(Node):
    def __init__(self):
        super().__init__("buggy_lidar")
        self.get_logger().info('INITIALIZED.')

        self.lidar_points_subscriber = self.create_subscription(
            'velodyne_points', None,
        )


def main(args=None):
    rclpy.init(args=args)

    # Create the BuggyLidar node and spin it
    state_converter = BuggyLidar()
    rclpy.spin(state_converter)

    # Shutdown when done
    state_converter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()