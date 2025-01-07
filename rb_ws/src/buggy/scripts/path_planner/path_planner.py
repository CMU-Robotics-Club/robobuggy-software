#!/usr/bin/env python3

import sys
from threading import Lock

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from buggy.msg import TrajectoryMsg

sys.path.append("/rb_ws/src/buggy/scripts")
from util.pose import Pose
from util.trajectory import Trajectory

class PathPlanner(Node):
    """
    Class to generate new trajectory splices for SC autonomous system.

    Takes in a default trajectory and an inner curb trajectory.

    """

    # move the curb towards the center of the course by CURB_MARGIN meters
    # for a margin of safety
    CURB_MARGIN = 1 #m

    # the offset is calculated as a mirrored sigmoid function of distance
    OFFSET_SCALE_CROSS_TRACK = 2 #m
    OFFSET_SCALE_ALONG_TRACK = 0.2
    ACTIVATE_OTHER_SCALE_ALONG_TRACK = 0.1
    OFFSET_SHIFT_ALONG_TRACK = 4 #m

    # number of meters ahead of the buggy to generate local trajectory for
    LOCAL_TRAJ_LEN = 50#m

    # start generating local trajectory this many meters ahead of current position
    LOOKAHEAD = 2#m

    # number of points to sample along the nominal trajectory
    RESOLUTION = 150

    # frequency to run in hz
    FREQUENCY = 10

    def __init__(self) -> None:
        super().__init__('path_planner')
        self.get_logger().info('INITIALIZED.')


        #Parameters
        self.declare_parameter("traj_name", "buggycourse_safe.json")
        traj_name = self.get_parameter("traj_name").value
        self.nominal_traj = Trajectory(json_filepath="/rb_ws/src/buggy/paths/" + traj_name) #TODO: Fixed filepath, not good

        self.declare_parameter("curb_name", "")
        curb_name = self.get_parameter("curb_name").value
        curb_name = None if curb_name == "" else curb_name
        if curb_name is None:
            self.left_curb = None
        else:
            self.left_curb = Trajectory(json_filepath="/rb_ws/src/buggy/paths/" + curb_name) #TODO: Fixed filepath, not good

        #Publishers
        self.other_buggy_xtrack_publisher = self.create_publisher(Float64, "self/debug/other_buggy_xtrack", 10)
        self.traj_publisher = self.create_publisher(TrajectoryMsg, "self/cur_traj", 10)
        
        #Subscribers
        self.self_pose_subscriber = self.create_subscription(Odometry, 'self/state', self.self_pose_callback, 1)
        self.other_pose_subscriber = self.create_subscription(Odometry, 'other/state', self.other_pose_callback, 1)

        self.timer = self.create_timer(1/self.FREQUENCY, self.timer_callback)

        # set up subscriber and callback for pose variables
        self.self_pose = None
        self.other_pose = None

        self.self_pose_lock = Lock()
        self.other_pose_lock = Lock()

    def self_pose_callback(self, msg):
        with self.self_pose_lock:
            self.self_pose = Pose.rospose_to_pose(msg.pose.pose)

    def other_pose_callback(self, msg):
        with self.other_pose_lock:
            self.other_pose = Pose.rospose_to_pose(msg.pose.pose)
            self.get_logger().debug("RECEIVED FROM NAND")

    def timer_callback(self):
        with self.self_pose_lock and self.other_pose_lock:
            if (self.self_pose is not None) and (self.other_pose is not None):
                self.compute_traj(self.self_pose, self.other_pose)

    def offset_func(self, dist):
        """
        Args:
            dist: (N, ) numpy array, distances between ego-buggy and obstacle,
            along the trajectory
        Returns:
            (N, ) numpy array, offsets from nominal trajectory required to overtake,
            defined by a sigmoid function
        """

        return self.OFFSET_SCALE_CROSS_TRACK / \
            (1 + np.exp(-(-self.OFFSET_SCALE_ALONG_TRACK * dist +
            self.OFFSET_SHIFT_ALONG_TRACK)))

    def activate_other_crosstrack_func(self, dist):
        """
        Args:
            dist: (N, ) numpy array, distances between ego-buggy and obstacle,
            along the trajectory
        Returns:
            (N, ) numpy array, multiplier used to weigh the cross-track distance of
            the obstacle into the passing offset calculation.
        """
        return 1 / \
            (1 + np.exp(-(-self.ACTIVATE_OTHER_SCALE_ALONG_TRACK * dist +
            self.OFFSET_SHIFT_ALONG_TRACK)))


    def compute_traj(
        self,
        self_pose: Pose,
        other_pose: Pose,
        ) -> None:
        """
        draw trajectory starting at the current pose and ending at a fixed distance
        ahead. For each trajectory point, calculate the required offset perpendicular to the nominal
        trajectory. A sigmoid function of the distance along track to the other buggy is used to
        weigh the other buggy's cross-track distance. This calculation produces a line that
        allows the ego-buggy's trajectory to go through the other buggy. Since we want to pass
        the other buggy at some constant distance to the left, another sigmoid function is multiplied
        by that constant distance to produce a smooth trajectory that passes the other buggy.

        Finally, the trajectory is bounded to the left by the left curb (if it exists), and to the right
        by the nominal trajectory. (we never pass on the right)

        passing offsets =
            activate_other_crosstrack_func(distance to other buggy along track) *
            other buggy cross track distance +
            offset_func(distance to other buggy along track)

        trajectory = nominal trajectory +
            left nominal trajectory unit normal vector *
            clamp(passing offsets, 0, distance from nominal trajectory to left curb)

        Args:
            other_pose (Pose): Pose containing NAND's easting (x),
                northing(y), and heading (theta), in UTM

            other_speed (float): speed in m/s of NAND
        Publishes:
            Trajectory: list of x,y coords for ego-buggy to follow.
        """
        # grab slice of nominal trajectory
        nominal_idx = self.nominal_traj.get_closest_index_on_path(self_pose.x, self_pose.y)
        nominal_dist_along = self.nominal_traj.get_distance_from_index(nominal_idx)

        nominal_slice = np.empty((self.RESOLUTION, 2))

        # compute the distance along nominal trajectory between samples and the obstacle
        nominal_slice_dists = np.linspace(
            nominal_dist_along + self.LOOKAHEAD,
            nominal_dist_along + self.LOOKAHEAD + self.LOCAL_TRAJ_LEN,
            self.RESOLUTION)

        for i in range(self.RESOLUTION):
            nominal_slice[i, :] = np.array(self.nominal_traj.get_position_by_distance(
               nominal_slice_dists[i]
            ))

        # get index of the other buggy along the trajetory and convert to distance
        other_idx = self.nominal_traj.get_closest_index_on_path(other_pose.x, other_pose.y)
        other_dist = self.nominal_traj.get_distance_from_index(other_idx)
        nominal_slice_to_other_dist = np.abs(nominal_slice_dists - other_dist)

        passing_offsets = self.offset_func(nominal_slice_to_other_dist)

        # compute signed cross-track distance between NAND and nominal
        nominal_to_other = np.array((other_pose.x, other_pose.y)) - \
            np.array(self.nominal_traj.get_position_by_index(other_idx))

        # dot product with the unit normal to produce left-positive signed distance
        other_normal = self.nominal_traj.get_unit_normal_by_index(np.array(other_idx.ravel()))
        other_cross_track_dist = np.sum(
            nominal_to_other * other_normal, axis=1)

        self.other_buggy_xtrack_publisher.publish(Float64(data=float(other_cross_track_dist)))

        # here, use passing offsets to weight NAND's cross track signed distance:
        # if the sample point is far from SC, the cross track distance doesn't matter
        # if the sample point is near, we add cross track distance to the offset,
        # such that the resulting offset is adjusted by position of NAND

        passing_offsets = passing_offsets + \
            self.activate_other_crosstrack_func(nominal_slice_to_other_dist) * other_cross_track_dist

        # clamp passing offset distances to distance to the curb
        if self.left_curb is not None:
            # grab slice of curb correponding to slice of nominal trajectory.
            curb_idx = self.left_curb.get_closest_index_on_path(self_pose.x, self_pose.y)
            curb_dist_along = self.left_curb.get_distance_from_index(curb_idx)
            curb_idx_end = self.left_curb.get_closest_index_on_path(nominal_slice[-1, 0], nominal_slice[-1, 1])
            curb_dist_along_end = self.left_curb.get_distance_from_index(curb_idx_end)
            curb_dists = np.linspace(curb_dist_along, curb_dist_along_end, self.RESOLUTION)

            curb_slice = np.empty((self.RESOLUTION, 2))
            for i in range(self.RESOLUTION):
                curb_slice[i, :] = np.array(self.left_curb.get_position_by_distance(
                    curb_dists[i]
                ))

            # compute distances from the sample points to the curb
            nominal_slice_to_curb_dist = np.linalg.norm(curb_slice - nominal_slice, axis=1)
            passing_offsets = np.minimum(passing_offsets, nominal_slice_to_curb_dist - self.CURB_MARGIN)

        # clamp negative passing offsets to zero, since we always pass on the left,
        # the passing offsets should never pull SC to the right.
        passing_offsets = np.maximum(0, passing_offsets)

        # shift the nominal slice by passing offsets
        nominal_normals = self.nominal_traj.get_unit_normal_by_index(
            self.nominal_traj.get_index_from_distance(nominal_slice_dists)
        )
        positions = nominal_slice + (passing_offsets[:, None] * nominal_normals)

        local_traj = Trajectory(json_filepath=None, positions=positions)
        self.traj_publisher.publish(local_traj.pack(self_pose.x, self_pose.y ))


def main(args=None):
    rclpy.init(args=args)
    # TODO: set file paths here
    # At the time of writing the below snippet, config management is TBD

    path_planner = PathPlanner()
    rclpy.spin(path_planner)

    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()