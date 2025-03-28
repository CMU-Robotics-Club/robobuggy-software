import rclpy
from rclpy.node import Node
from sensor_msgs import Image
from zed_msgs import Object


# this node is an experiment with integrating SC with the builtin ZED node

# https://github.com/stereolabs/zed-ros2-wrapper/blob/master/README.md
# https://wiki.ros.org/zed-ros-wrapper (cant find ros2 equiv)
# https://github.com/stereolabs/zed-ros2-interfaces

class Detector(Node):

    def __init__(self):
        super().__init__('detector')

        self.curr_frame = None
        self.detected_objects = []

        # Publishers
        self.observed_nand_odom_publisher = self.create_publisher(
                    Odometry, "/NAND_raw_state", 1
                )

        # Subscribers
        self.frame_subscriber = self.create_subscription(Image, "/rgb/image_rect_color", self.update_current_frame, 1)
        self.objects_subscriber = self.create_subscription(Object, "/objects", self.update_object_detections, 1)

        timer_period = 0.01  # seconds (100 Hz)
        self.timer = self.create_timer(timer_period, self.loop)
        self.i = 0 # Loop Counter

    def loop(self):
        # Loop for the code that operates every 10ms

    def update_current_frame (self, msg):
        self.curr_frame = msg.data

    def update_object_detections (self, msg):
        self.detected_objects = msg.data

def main(args=None):
    rclpy.init(args=args)

    detector_node = Detector()

    rclpy.spin(detector_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()