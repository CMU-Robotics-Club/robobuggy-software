import math

import numpy as np

from geometry_msgs.msg import Pose as ROSPose


class Pose:
    """
    A data structure for storing 2D poses, as well as a set of
    convenience methods for transforming/manipulating poses

    TODO: All Pose objects are with respect to World coordinates.
    """

    __x = None
    __y = None
    __theta = None

    # https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        """

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    @staticmethod
    def rospose_to_pose(rospose: ROSPose):
        """
        Converts a geometry_msgs/Pose to a pose3d/Pose

        Args:
            posestamped (geometry_msgs/Pose): pose to convert

        Returns:
            Pose: converted pose
        """
        (_, _, yaw) = Pose.euler_from_quaternion(
            rospose.orientation.x,
            rospose.orientation.y,
            rospose.orientation.z,
            rospose.orientation.w,
        )

        p = Pose(rospose.position.x, rospose.position.y, yaw)
        return p

    def heading_to_quaternion(heading):
        q = Pose.quaternion_from_euler(0, 0, heading)
        return q

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __repr__(self) -> str:
        return f"Pose(x={self.x}, y={self.y}, theta={self.theta})"

    def copy(self):
        return Pose(self.x, self.y, self.theta)

    @property
    def x(self):
        return self.__x

    @x.setter
    def x(self, x):
        self.__x = x

    @property
    def y(self):
        return self.__y

    @y.setter
    def y(self, y):
        self.__y = y

    @property
    def theta(self):
        if self.__theta > np.pi or self.__theta < -np.pi:
            raise ValueError("Theta is not in [-pi, pi]")
        return self.__theta

    @theta.setter
    def theta(self, theta):
        # normalize theta to [-pi, pi]
        self.__theta = np.arctan2(np.sin(theta), np.cos(theta))

    def to_mat(self):
        """
        Returns the pose as a 3x3 homogeneous transformation matrix
        """
        return np.array(
            [
                [np.cos(self.theta), -np.sin(self.theta), self.x],
                [np.sin(self.theta), np.cos(self.theta), self.y],
                [0, 0, 1],
            ]
        )

    @staticmethod
    def from_mat(mat):
        """
        Creates a pose from a 3x3 homogeneous transformation matrix
        """
        return Pose(mat[0, 2], mat[1, 2], np.arctan2(mat[1, 0], mat[0, 0]))

    def invert(self):
        """
        Inverts the pose
        """
        return Pose.from_mat(np.linalg.inv(self.to_mat()))

    def convert_pose_from_global_to_local_frame(self, pose):
        """
        Converts the inputted pose from the global frame to the local frame
        (relative to the current pose)
        """
        return pose / self

    def convert_pose_from_local_to_global_frame(self, pose):
        """
        Converts the inputted pose from the local frame to the global frame
        (relative to the current pose)
        """
        return pose * self

    def convert_point_from_global_to_local_frame(self, point):
        """
        Converts the inputted point from the global frame to the local frame
        (relative to the current pose)
        """
        point_hg = np.array([point[0], point[1], 1])
        point_hg_local = np.linalg.inv(self.to_mat()) @ point_hg
        return (
            point_hg_local[0] / point_hg_local[2],
            point_hg_local[1] / point_hg_local[2],
        )

    def convert_point_from_local_to_global_frame(self, point):
        """
        Converts the inputted point from the local frame to the global frame
        (relative to the current pose)
        """
        point_hg = np.array([point[0], point[1], 1])
        point_hg_global = self.to_mat() @ point_hg
        return (
            point_hg_global[0] / point_hg_global[2],
            point_hg_global[1] / point_hg_global[2],
        )

    def convert_point_array_from_global_to_local_frame(self, points):
        """
        Converts the inputted point array from the global frame to the local frame
        (relative to the current pose)

        Args:
            points (np.ndarray) [Size: (N,2)]: array of points to convert
        """
        points_hg = np.array([points[:, 0], points[:, 1], np.ones(len(points))])
        points_hg_local = np.linalg.inv(self.to_mat()) @ points_hg.T
        return (points_hg_local[:2, :] / points_hg_local[2, :]).T

    def convert_point_array_from_local_to_global_frame(self, points):
        """
        Converts the inputted point array from the local frame to the global frame
        (relative to the current pose)

        Args:
            points (np.ndarray) [Size: (N,2)]: array of points to convert
        """
        points_hg = np.array([points[:, 0], points[:, 1], np.ones(len(points))])
        points_hg_global = self.to_mat() @ points_hg.T
        return (points_hg_global[:2, :] / points_hg_global[2, :]).T

    def rotate(self, angle):
        """
        Rotates the pose by the given angle
        """
        self.theta += angle

    def translate(self, x, y):
        """
        Translates the pose by the given x and y distances
        """
        self.x += x
        self.y += y

    def __add__(self, other):
        return Pose(self.x + other.x, self.y + other.y, self.theta + other.theta)

    def __sub__(self, other):
        return Pose(self.x - other.x, self.y - other.y, self.theta - other.theta)

    def __neg__(self):
        return Pose(-self.x, -self.y, -self.theta)

    def __mul__(self, other):
        p1_mat = self.to_mat()
        p2_mat = other.to_mat()

        return Pose.from_mat(p1_mat @ p2_mat)

    def __truediv__(self, other):
        p1_mat = self.to_mat()
        p2_mat = other.to_mat()

        return Pose.from_mat(np.linalg.inv(p2_mat) @ p1_mat)


if __name__ == "__main__":
    # TODO: again do we want example code in these classes
    rospose = ROSPose()
    rospose.position.x = 1
    rospose.position.y = 2
    rospose.position.z = 3
    rospose.orientation.x = 0
    rospose.orientation.y = 0
    rospose.orientation.z = -0.061461
    rospose.orientation.w = 0.9981095

    pose = Pose.rospose_to_pose(rospose)
    print(pose)  # Pose(x=1, y=2, theta=-0.123)