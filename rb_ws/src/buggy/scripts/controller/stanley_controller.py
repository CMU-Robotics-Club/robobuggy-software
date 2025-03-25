import numpy as np

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose as ROSPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


from util.trajectory import Trajectory
from controller.controller_superclass import Controller
from util.pose import Pose

import utm


class StanleyController(Controller):
    """
    Stanley Controller (front axle used as reference point)
    Referenced from this paper: https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf
    """

    LOOK_AHEAD_DIST_CONST = 0.05 # s
    MIN_LOOK_AHEAD_DIST = 0.1 #m
    MAX_LOOK_AHEAD_DIST = 2.0 #m

    CROSS_TRACK_GAIN = 1.3
    K_SOFT = 1.0 # m/s
    K_D_YAW = 0.012 # rad / (rad/s)

    def __init__(self, start_index, namespace, node, usingHeadingRateError, controllerName):
        super(StanleyController, self).__init__(start_index, namespace, node)
        self.debug_reference_pos_publisher = self.node.create_publisher(
            NavSatFix, controllerName + "/debug/reference_navsat", 1
        )
        self.debug_error_publisher = self.node.create_publisher(
            ROSPose, controllerName + "/debug/stanley_error", 1
        )
        self.debug_yaw_rate_publisher = self.node.create_publisher(
            Float64, controllerName + "/debug/yaw", 1
        )
        self.debug_yaw_rate2_publisher = self.node.create_publisher(
            Float64, controllerName + "/debug/yaw2", 1
        )

        self.usingHeadingRateError = usingHeadingRateError

    def compute_control(self, state_msg : Odometry, trajectory : Trajectory):
        """Computes the steering angle determined by Stanley controller.
        Does this by looking at the crosstrack error + heading error

        Args:
            state_msg: ros Odometry message
            trajectory (Trajectory): reference trajectory

        Returns:
            float (desired steering angle)
        """
        if self.current_traj_index >= trajectory.get_num_points() - 1:
            self.node.get_logger().error("[Stanley]: Ran out of path to follow!")
            raise Exception("[Stanley]: Ran out of path to follow!")

        current_rospose = state_msg.pose.pose
        current_speed = np.sqrt(
            state_msg.twist.twist.linear.x**2 + state_msg.twist.twist.linear.y**2
        )
        yaw_rate = state_msg.twist.twist.angular.z
        heading = current_rospose.orientation.z
        x, y = current_rospose.position.x, current_rospose.position.y #(Easting, Northing)

        front_x = x + StanleyController.WHEELBASE * np.cos(heading)
        front_y = y + StanleyController.WHEELBASE * np.sin(heading)

        # setting range of indices to search so we don't have to search the entire path
        traj_index = trajectory.get_closest_index_on_path(
            front_x,
            front_y,
            start_index=self.current_traj_index - 20,
            end_index=self.current_traj_index + 50,
        )
        self.current_traj_index = max(traj_index, self.current_traj_index)


        lookahead_dist = np.clip(
            self.LOOK_AHEAD_DIST_CONST * current_speed,
            self.MIN_LOOK_AHEAD_DIST,
            self.MAX_LOOK_AHEAD_DIST)

        traj_dist = trajectory.get_distance_from_index(self.current_traj_index) + lookahead_dist

        ref_heading = trajectory.get_heading_by_index(
            trajectory.get_index_from_distance(traj_dist))

        error_heading = ref_heading - heading
        error_heading = np.arctan2(np.sin(error_heading), np.cos(error_heading)) #Bounds error_heading

        # Calculate cross track error by finding the distance from the buggy to the tangent line of
        # the reference trajectory
        closest_position = trajectory.get_position_by_index(self.current_traj_index)
        next_position = trajectory.get_position_by_index(
            self.current_traj_index + 0.0001
        )
        x1 = closest_position[0]
        y1 = closest_position[1]
        x2 = next_position[0]
        y2 = next_position[1]
        error_dist = -((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)) / np.sqrt(
            (y2 - y1) ** 2 + (x2 - x1) ** 2
        )


        cross_track_component = -np.arctan2(
            StanleyController.CROSS_TRACK_GAIN * error_dist, current_speed + StanleyController.K_SOFT
        )

        accel_x, accel_y = trajectory.get_acceleration_by_index(trajectory.get_index_from_distance(traj_dist))
        # this works because tan(heading) = dydt/dxdt (do the math)
        dxdt, dydt = np.cos(ref_heading), np.sin(ref_heading)

        # this was dervied by doing the chain rule on the target derivative of theta.
        # dtheta/dt = d/dt (arctan (dydt/dxdt)) << do math.
        r_traj_2 = (1/(1 + (dydt/dxdt)**2)) * (accel_y/dxdt - (dydt * accel_x)/(dxdt ** 2))

        # Calculate yaw rate error
        r_meas = yaw_rate

        r_traj = current_speed * (trajectory.get_heading_by_index(trajectory.get_index_from_distance(traj_dist) + 0.05) -trajectory.get_heading_by_index(trajectory.get_index_from_distance(traj_dist))) / 0.05

        # if (abs(StanleyController.K_D_YAW * (r_meas - r_traj)) > 6.0):
        #     rclpy.logwarn(f"spiked yaw_rate: actual: {r_meas}, expected_basic: {r_traj}, expected_analytic: {r_traj_2}")

        yaw = Float64()
        yaw.data = float(StanleyController.K_D_YAW * (r_traj - r_meas))
        self.debug_yaw_rate_publisher.publish(yaw)

        yaw2 = Float64()
        yaw2.data = float(StanleyController.K_D_YAW * (r_traj_2 - r_meas))
        self.debug_yaw_rate2_publisher.publish(yaw2)


        #Determine steering_command


        steering_cmd = error_heading + cross_track_component
        if self.usingHeadingRateError:  steering_cmd += StanleyController.K_D_YAW * (r_traj - r_meas)
        steering_cmd = np.clip(steering_cmd, -np.pi / 9, np.pi / 9)


        #Calculate error, where x is in orientation of buggy, y is cross track error
        current_pose = Pose(current_rospose.position.x, current_rospose.position.y, heading)
        reference_error = current_pose.convert_point_from_global_to_local_frame(closest_position)
        reference_error -= np.array([StanleyController.WHEELBASE, 0]) #Translae back to back wheel to get accurate error

        error_pose = ROSPose()
        error_pose.position.x = reference_error[0]
        error_pose.position.y = reference_error[1]
        self.debug_error_publisher.publish(error_pose)

        # Publish reference position for debugging
        try:
            reference_navsat = NavSatFix()
            lat, lon = utm.to_latlon(closest_position[0], closest_position[1], 17, "T")
            reference_navsat.latitude = lat
            reference_navsat.longitude = lon
            self.debug_reference_pos_publisher.publish(reference_navsat)
        except Exception as e:
            self.node.get_logger().warn(
                "[Stanley] Unable to convert closest track position lat lon; Error: " + str(e)
            )

        return steering_cmd











