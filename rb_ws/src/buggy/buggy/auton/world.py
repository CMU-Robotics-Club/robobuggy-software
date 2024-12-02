import utm
import numpy as np

from pose import Pose


class World:
    """Abstraction for the world coordinate system

    The real world uses GPS coordinates, aka latitude and longitude. However,
    using lat/lon is bad for path planning for several reasons. First, the difference
    in numbers would be very tiny for small distances, alluding to roundoff errors.
    Additionally, lat/lon follows a North-East-Down coordinate system, with headings
    in the clockwise direction. We want to use an East-North-Up coordinate system, so
    that the heading is in the counter-clockwise direction.

    We do this by converting GPS coordinates to UTM coordinates, which are in meters.
    UTM works by dividing the world into a grid, where each grid cell has a unique
    identifier. A UTM coordinate consists of the grid cell identifier and the "easting"
    and "northing" within that grid cell. The easting (x) and northing (y) are in meters,
    and are relative to the southwest corner of the grid cell.

    Last, we offset the UTM coordinates to be relative to some arbitrary zero point. That
    way, the final world coordinates are relatively close to zero, which makes debugging
    easier.

    This class provides methods to convert between GPS and world coordinates. There is
    a version for single coordinates and a version for numpy arrays.
    """

    # Geolocates to around the southwest corner of Phipps
    WORLD_EAST_ZERO = 589106
    WORLD_NORTH_ZERO = 4476929

    @staticmethod
    def gps_to_world(lat, lon):
        """Converts GPS coordinates to world coordinates

        Args:
            lat (float): latitude
            lon (float): longitude

        Returns:
            tuple: (x, y) in meters from some arbitrary zero point
        """
        utm_coords = utm.from_latlon(lat, lon)
        x = utm_coords[0] - World.WORLD_EAST_ZERO
        y = utm_coords[1] - World.WORLD_NORTH_ZERO

        return x, y

    @staticmethod
    def utm_to_world_pose(pose: Pose) -> Pose:
        """Converts UTM coordinates to world coordinates

        Args:
            pose (Pose): pose with utm coordinates

        Returns:
            Pose: pose with world coordinates
        """

        utm_e = pose.x
        utm_n = pose.y
        x = utm_e - World.WORLD_EAST_ZERO
        y = utm_n - World.WORLD_NORTH_ZERO
        return Pose(x, y, pose.theta)

    @staticmethod
    def world_to_utm_pose(pose: Pose) -> Pose:
        """Converts world coordinates to utm coordinates

        Args:
            pose (Pose): pose with world coordinates

        Returns:
            Pose: pose with world coordinates
        """

        utm_e = pose.x + World.WORLD_EAST_ZERO
        utm_n = pose.y + World.WORLD_NORTH_ZERO
        return Pose(utm_e, utm_n, pose.theta)

    @staticmethod
    def world_to_utm_numpy(coords):
        """Converts world coordinates to utm coordinates

        Args:
            coords (numpy.ndarray [size: (N,2)]): array of x, y pairs

        Returns:
            numpy.ndarray [size: (N,2)]: array of utm_e, utm_n pairs
        """

        N = np.shape(coords)[0]
        utm_offset_e = np.ones((N, )) * World.WORLD_EAST_ZERO
        utm_offset_n = np.ones((N, )) * World.WORLD_NORTH_ZERO
        utm_offset = np.stack((utm_offset_e, utm_offset_n), axis=1)

        return coords + utm_offset

    @staticmethod
    def utm_to_world_numpy(coords):
        """Converts utm coordinates to world coordinates

        Args:
            coords (numpy.ndarray [size: (N,2)]): array of utm_e, utm_n pairs

        Returns:
            numpy.ndarray [size: (N,2)]: array of x, y pairs
        """

        N = np.shape(coords)[0]
        utm_offset_e = np.ones((N, )) * World.WORLD_EAST_ZERO
        utm_offset_n = np.ones((N, )) * World.WORLD_NORTH_ZERO
        utm_offset = np.stack((utm_offset_e, utm_offset_n), axis=1)

        return coords - utm_offset


    @staticmethod
    def world_to_gps(x, y):
        """Converts world coordinates to GPS coordinates

        Args:
            x (float): x in meters from some arbitrary zero point
            y (float): y in meters from some arbitrary zero point

        Returns:
            tuple: (lat, lon)
        """
        utm_coords = utm.to_latlon(
            x + World.WORLD_EAST_ZERO, y + World.WORLD_NORTH_ZERO, 17, "T"
        )
        lat = utm_coords[0]
        lon = utm_coords[1]

        return lat, lon

    @staticmethod
    def utm_to_gps(x, y):
        """Converts utm coordinates to GPS coordinates

        Args:
            x (float): x in meters, in utm frame
            y (float): y in meters, in utm frame

        Returns:
            tuple: (lat, lon)
        """
        utm_coords = utm.to_latlon(
            x, y, 17, "T"
        )
        lat = utm_coords[0]
        lon = utm_coords[1]

        return lat, lon

    @staticmethod
    def gps_to_world_numpy(coords):
        """Converts GPS coordinates to world coordinates

        Args:
            coords (numpy.ndarray [size: (N,2)]): array of lat, lon pairs

        Returns:
            numpy.ndarray [size: (N,2)]: array of x, y pairs
        """
        utm_coords = utm.from_latlon(coords[:, 0], coords[:, 1])
        x = utm_coords[0] - World.WORLD_EAST_ZERO
        y = utm_coords[1] - World.WORLD_NORTH_ZERO

        return np.stack((x, y), axis=1)

    @staticmethod
    def world_to_gps_numpy(coords):
        """Converts world coordinates to GPS coordinates

        Args:
            coords (numpy.ndarray [size: (N,2)]): array of x, y pairs

        Returns:
            numpy.ndarray [size: (N,2)]: array of lat, lon pairs
        """

        # Pittsburgh is in UTM zone 17T.
        utm_coords = utm.to_latlon(
            coords[:, 0] + World.WORLD_EAST_ZERO,
            coords[:, 1] + World.WORLD_NORTH_ZERO,
            17,
            "T",
            )
        lat = utm_coords[0]
        lon = utm_coords[1]

        return np.stack((lat, lon), axis=1)

    @staticmethod
    def gps_to_world_pose(pose: Pose):
        """Converts GPS coordinates to world coordinates

        Args:
            pose (Pose): pose with lat, lon coordinates (x=lon, y=lat)

        Returns:
            Pose: pose with x, y coordinates
        """
        world_coords = World.gps_to_world(pose.y, pose.x)
        new_heading = pose.theta  # INS uses ENU frame so no heading conversion needed

        return Pose(world_coords[0], world_coords[1], new_heading)

    @staticmethod
    def world_to_gps_pose(pose: Pose):
        """Converts world coordinates to GPS coordinates

        Args:
            pose (Pose): pose with x, y coordinates

        Returns:
            Pose: pose with lat, lon coordinates
        """
        gps_coords = World.world_to_gps(pose.x, pose.y)
        new_heading = pose.theta

        return Pose(gps_coords[1], gps_coords[0], new_heading)