import threading
import sys

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float64, Bool
from nav_msgs.msg import Odometry

sys.path.append("/rb_ws/src/buggy/buggy")
from util.trajectory_old import Trajectory
from controller.stanley_controller import StanleyController

class Controller(Node):

    def __init__(self):
        """
        Constructor for Controller class.

        Creates a ROS node with a publisher that periodically sends a message
        indicating whether the node is still alive.
        
        """
        super().__init__('controller')
            

        #Parameters
        self.declare_parameter("dist", 0.0) #Starting Distance along path
        start_dist = self.get_parameter("dist")

        self.declare_parameter("traj_name", "buggycourse_path.json")
        traj_name = self.get_parameter("traj_name")
        self.cur_traj = Trajectory(json_filepath="/rb_ws/src/buggy/paths/" + traj_name) #TODO: Fixed filepath, not good

        start_index = self.cur_traj.get_index_from_distance(start_dist)

        self.declare_parameter("controller_name", "stanley")
        match self.get_parameter("controller_name"):
            case "stanley":
                self.controller = StanleyController(start_index = start_index, buggy_name = self.get_namespace(), node=self) #IMPORT STANLEY
            case _:
                self.get_logger().error("Invalid Controller Name!")
                raise Exception("Invalid Controller Argument")

        # Publishers
        self.init_check_publisher = self.create_publisher(Bool,
            "debug/init_safety_check", 1
        )
        self.steer_publisher = self.create_publisher(
            Float64, "/buggy/steering", 1
        )
        self.heading_publisher = self.create_publisher(
            Float32, "/auton/debug/heading", 1
        )

        # Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, 'self/buggy/state', self.odom_listener, 1)
        self.traj_subscriber = self.create_subscription(Odometry, 'self/cur_traj', self.traj_listener, 1)

        self.lock = threading.Lock()

        self.odom = None
        self.passed_init = False

        timer_period = 0.01  # seconds (100 Hz)
        self.timer = self.create_timer(timer_period, self.loop)
    
    def odom_listener(self, msg : Odometry):
        '''
        This is the subscriber that updates the buggies state for navigation
        msg, should be a CLEAN state as defined in the wiki
        '''
        with self.lock:
            self.odom = msg
    
    def traj_listener(self, msg): #TYPE UNKOWN AS OF NOW?? CUSTOM TYPE WHEN #TODO: FIGURE OUT BEFORE MERGE
        '''
        This is the subscriber that updates the buggies trajectory for navigation
        '''
        with self.lock:
            self.cur_traj, self.controller.current_traj_index = Trajectory.unpack(msg)

    def init_check(self):
        """
        Checks if it's safe to switch the buggy into autonomous driving mode.
        Specifically, it checks:
            if we can recieve odometry messages from the buggy
            if the covariance is acceptable (less than 1 meter)
            if the buggy thinks it is facing in the correct direction wrt the local trajectory (not 180 degrees flipped)

        Returns:
           A boolean describing the status of the buggy (safe for auton or unsafe for auton)
        """
        if (self.odom == None):
            self.get_logger().warn("WARNING: no available position estimate")
            return False
    
        elif (self.odom.pose.covariance[0] ** 2 + self.odom.pose.covariance[7] ** 2 > 1**2):
            self.get_logger().warn("checking position estimate certainty")
            return False
        
        #Originally under a lock, doesn't seem necessary?
        current_heading = self.odom.pose.pose.orientation.z
        closest_heading = self.cur_traj.get_heading_by_index(self.cur_traj.get_closest_index_on_path(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y))

        self.get_logger().info("current heading: " + str(np.rad2deg(current_heading)))
        self.heading_publisher.publish(Float32(np.rad2deg(current_heading)))

        #Converting headings from [-pi, pi] to [0, 2pi]
        if (current_heading < 0):
            current_heading = 2 * np.pi + current_heading
        if (closest_heading < 0):
            closest_heading = 2 * np.pi + closest_heading
        
        if (abs(current_heading - closest_heading) >= np.pi/2):
            self.get_logger().warn("WARNING: INCORRECT HEADING! restart stack. Current heading [-180, 180]: " + str(np.rad2deg(current_heading)))
            return False
    
        return True

    def loop(self):
        if not self.passed_init:
            self.passed_init = self.init_check()
            self.init_check_publisher.publish(self.passed_init)
            if self.passed_init:
                self.get_logger.info("Passed Initialization Check")
            else:
                return
        
        self.heading_publisher.publish(Float32(np.rad2deg(self.odom.pose.pose.orientation.z)))
        steering_angle = self.controller.compute_control(self.odom, self.cur_traj)
        steering_angle_deg = np.rad2deg(steering_angle)
        self.steer_publisher.publish(Float64(steering_angle_deg))

    

def main(args=None):
    rclpy.init(args=args)

    watchdog = Controller()

    rclpy.spin(watchdog)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    watchdog.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()