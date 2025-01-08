import json
import time

# from buggy.msg import TrajectoryMsg

import numpy as np
from scipy.interpolate import Akima1DInterpolator, CubicSpline
from buggy.msg import TrajectoryMsg

import utm

class Trajectory:
    """A wrapper around a trajectory JSON file that does some under-the-hood math. Will
    interpolate the data points and calculate other information such as distance along the trajectory
    and instantaneous curvature.

    Currently, the trajectory is assumed to be a straight line between each waypoint. This is
    not a good assumption, but it's good enough for now. This means that the curvature is zero
    anywhere between waypoints.

    This class also has a method that will find the closest point on the trajectory to a given
    point in the world. This is useful for finding where the buggy is on the trajectory.

    Use https://rdudhagra.github.io/eracer-portal/ to make trajectories and save the JSON file
    """

    def __init__(self, json_filepath=None, positions = None, interpolator="CubicSpline") -> None:
        """
        Args:
            json_filepath (String): file path to the path json file (begins at /rb_ws)
            positions [[float, float]]: reference trajectory in world coordinates
            current_speed (float): current speed of the buggy

        Returns:
            float (desired steering angle)
        """
        self.distances = np.zeros((0, 1))  # (N/dt x 1) [d, d, ...]
        self.positions = np.zeros((0, 2))  # (N x 2) [(x,y), (x,y), ...]
        self.indices = None  # (N x 1) [0, 1, 2, ...]
        self.interpolation = None  # scipy.interpolate.PPoly

        # read positions from file
        if positions is None:
            positions = []
            # Load the json file
            with open(json_filepath, "r") as f:
                data = json.load(f)

            # Iterate through the waypoints and extract the positions
            num_waypoints = len(data)
            for i in range(0, num_waypoints):

                waypoint = data[i]

                lat = waypoint["lat"]
                lon = waypoint["lon"]

                # Convert to world coordinates
                x, y, _, _ = utm.from_latlon(lat, lon)
                positions.append([x, y])

            positions = np.array(positions)

        num_indices = positions.shape[0]

        if interpolator == "Akima":
            self.positions = positions
            self.indices = np.arange(num_indices)
            self.interpolation = Akima1DInterpolator(self.indices, self.positions)
            self.interpolation.extrapolate = True
        elif interpolator == "CubicSpline":
            temp_traj = Trajectory(positions=positions, interpolator="Akima")
            tot_len = temp_traj.distances[-1]
            interp_dists = np.linspace(0, tot_len, num_indices)

            self.indices = np.arange(num_indices)
            self.positions = [
                temp_traj.get_position_by_distance(interp_dist)
                for interp_dist in interp_dists
            ]
            self.positions = np.array(self.positions)

            self.interpolation = CubicSpline(self.indices, self.positions)
            self.interpolation.extrapolate = True

        # TODO: check units
        # Calculate the distances along the trajectory
        dt = 0.01 #dt is time step (in seconds (?))
        ts = np.arange(len(self.positions) - 1, step=dt)  # times corresponding to each position (?)
        drdt = self.interpolation(
            ts, nu=1
        )  # Calculate derivatives of interpolated path wrt indices
        ds = np.sqrt(drdt[:, 0] ** 2 + drdt[:, 1] ** 2) * dt # distances to each interpolated point
        s = np.cumsum(np.hstack([[0], ds[:-1]]))
        self.distances = s
        self.dt = dt

    def get_num_points(self):
        """Gets the number of points along the trajectory

        Returns:
            int: number of points
        """
        return len(self.positions)

    def get_heading_by_index(self, index):
        """Gets the heading at given index along trajectory,
        interpolating if necessary

        Args:
            index (float): index along the trajectory

        Returns:
            float: (theta) in rads
        """
        # theta = np.interp(index, self.indices, self.positions[:, 2])
        dxdt, dydt = self.interpolation(index, nu=1).reshape((-1, 2)).T
        theta = np.arctan2(dydt, dxdt)

        return theta

    def get_heading_by_distance(self, distance):
        """Gets the heading at given distance along trajectory,
        interpolating if necessary

        Args:
            distance (float): distance along the trajectory in meters

        Returns:
            float: (theta) in rads
        """
        index = self.get_index_from_distance(distance)
        return self.get_heading_by_index(index)

    def get_position_by_index(self, index):
        """Gets the position at a given index along the trajectory,
        interpolating if necessary

        Args:
            index (float): index along the trajectory

        Returns:
            tuple: (x, y)
        """
        # Interpolate the position

        return self.interpolation(index)

    def get_position_by_distance(self, distance):
        """Gets the position at a given distance along the trajectory,
        interpolating if necessary

        Args:
            distance (float): distance along the trajectory in meters

        Returns:
            tuple: (x, y)
        """
        # Interpolate the position
        index = self.get_index_from_distance(distance)
        return self.get_position_by_index(index)

    def get_steering_angle_by_index(self, index, wheelbase):
        """Gets the bicycle-model steering angle at a given distance along the trajectory,
        interpolating if necessary. Assumes that the origin point of the buggy is at the
        center of the rear axle.

        Args:
            index (float): index along the trajectory
            wheelbase (float): wheelbase of the buggy in meters

        Returns:
            float: steering angle in rads
        """
        curvature = self.get_curvature_by_index(index)
        return np.arctan(wheelbase * curvature)

    def get_steering_angle_by_distance(self, distance, wheelbase):
        """Gets the bicycle-model steering angle at a given distance along the trajectory,
        interpolating if necessary. Assumes that the origin point of the buggy is at the
        center of the rear axle.

        Args:
            distance (float): distance along the trajectory in meters
            wheelbase (float): wheelbase of the buggy in meters

        Returns:
            float: steering angle in rads
        """
        index = self.get_index_from_distance(distance)
        return self.get_steering_angle_by_index(index, wheelbase)

    def get_index_from_distance(self, distance):
        """Gets the index at a given distance along the trajectory,
        interpolating if necessary

        Args:
            distance (float): distance along the trajectory in meters

        Returns:
            int: index along the trajectory
        """
        # Interpolate the index
        index = np.interp(
            distance,
            self.distances,
            np.linspace(0, len(self.distances), len(self.distances)),
        )

        return index * self.dt

    def get_distance_from_index(self, index):
        """Gets the distance at a given index along the trajectory,
        interpolating if necessary

        Args:
            index (float): index along the trajectory

        Returns:
            float: distance along the trajectory in meters
        """
        # Interpolate the distance
        distance = np.interp(
            index / self.dt, np.arange(len(self.distances)), self.distances
        )

        return distance

    def get_curvature_by_index(self, index):
        """Gets the curvature at a given index along the trajectory,
        interpolating if necessary

        Args:
            index (float): index along the trajectory

        Returns:
            float: curvature
        """
        # Interpolate the curvature
        dxdt, dydt = self.interpolation(index, nu=1).reshape((-1, 2)).T
        ddxdtt, ddydtt = self.interpolation(index, nu=2).reshape((-1, 2)).T

        curvature = (dxdt * ddydtt - dydt * ddxdtt) / (
            (dxdt**2 + dydt**2) ** (3 / 2)
        )

        return curvature

    def get_curvature_by_distance(self, distance):
        """Gets the curvature at a given distance along the trajectory,
        interpolating if necessary

        Args:
            distance (float): distance along the trajectory in meters

        Returns:
            float: curvature
        """
        # Interpolate the curvature
        index = self.get_index_from_distance(distance)

        return self.get_curvature_by_index(index)

    def get_dynamics_by_index(self, index, wheelbase):
        """Gets the dynamics at a given index along the trajectory,
        interpolating if necessary. Convenience function that returns
        all of the dynamics at once rather than requiring multiple
        calls to other helper functions. In this way, we can reuse calls
        to the interpolator, improving performance.

        Args:
            index (float): index along the trajectory

        Returns:
            tuple: (x, y, theta, steering_angle)
        """
        # Interpolate the dynamics
        x, y = self.interpolation(index).reshape((-1, 2)).T
        dxdt, dydt = self.interpolation(index, nu=1).reshape((-1, 2)).T
        ddxdtt, ddydtt = self.interpolation(index, nu=2).reshape((-1, 2)).T

        curvature = (dxdt * ddydtt - dydt * ddxdtt) / (
            (dxdt**2 + dydt**2) ** (3 / 2)
        )
        theta = np.arctan2(dydt, dxdt)

        return np.stack((x, y, theta, np.arctan(wheelbase * curvature)), axis=-1)

    def get_unit_normal_by_index(self, index):
        """Gets the index of the closest point on the trajectory to the given point

        Args:
            index: Nx1 numpy array: indexes along trajectory
        Returns:
            Nx2 numpy array: unit normal of the trajectory at index
        """

        derivative = self.interpolation(index, nu=1)
        unit_derivative = derivative / np.linalg.norm(derivative, axis=1)[:, None]

        # (x, y), rotated by 90 deg ccw = (-y, x)
        unit_normal = np.vstack((-unit_derivative[:, 1], unit_derivative[:, 0])).T
        return unit_normal

    def get_closest_index_on_path(
        self, x, y, start_index=0, end_index=None, subsample_resolution=1000
    ):
        """Gets the index of the closest point on the trajectory to the given point

        Args:
            x (float): x coordinate
            y (float): y coordinate
            start_index (int, optional): index to start searching from. Defaults to 0.
            end_index (int, optional): index to end searching at. Defaults to None (disable).
            subsample_resolution: resolution of the resulting interpolation
        Returns:
            float: index along the trajectory
        """
        # If end_index is not specified, use the length of the trajectory
        if end_index is None:
            end_index = len(self.positions) #sketch, 0-indexing where??

        # Floor/ceil the start/end indices
        start_index = max(0, int(np.floor(start_index)))
        end_index = int(np.ceil(end_index))

        # Calculate the distance from the point to each point on the trajectory
        distances = (self.positions[start_index : end_index + 1, 0] - x) ** 2 + (
            self.positions[start_index : end_index + 1, 1] - y
        ) ** 2 #Don't need to squareroot as it is a relative distance

        min_ind = np.argmin(distances) + start_index

        start_index = max(0, min_ind - 1) #Protection in case min_ind is too low
        end_index = min(len(self.positions), min_ind + 1)  #Prtoecting in case min_ind too high
        #Theoretically start_index and end_index are just two apart

        # Now interpolate at a higher resolution to get a more accurate result
        r_interp = self.interpolation(
            np.linspace(start_index, end_index, subsample_resolution + 1))
        x_interp, y_interp = r_interp[:, 0], r_interp[:, 1] #x_interp, y_interp are numpy column vectors

        distances = (x_interp - x) ** 2 + (y_interp - y) ** 2 #Again distances are relative

        # Return the rational index of the closest point
        return (
            np.argmin(distances) / subsample_resolution * (end_index - start_index)
            + start_index
        )

    def pack(self, x, y) -> TrajectoryMsg:
        traj = TrajectoryMsg()
        traj.easting = list(self.positions[:, 0])
        traj.northing = list(self.positions[:, 1])
        traj.time = time.time()
        traj.cur_idx = self.get_closest_index_on_path(x,y)
        return traj

    def unpack(trajMsg : TrajectoryMsg):
        pos = np.array([trajMsg.easting, trajMsg.northing]).transpose(1, 0)
        cur_idx = trajMsg.cur_idx
        return Trajectory(positions=pos), cur_idx

