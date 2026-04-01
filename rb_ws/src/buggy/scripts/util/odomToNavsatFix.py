from sensor_msgs.msg import NavSatFix, NavSatStatus
import utm
import numpy as np

from util.constants import Constants

# NOTE: covariances are in UTM in the NavsatFix object, not lat/lon

def odom_to_navsat(odom_msg, Sigma=None):
    east = odom_msg.pose.pose.position.x
    north = odom_msg.pose.pose.position.y
    alt = odom_msg.pose.pose.position.z

    try:
        lat, lon = utm.to_latlon(
            east,
            north,
            Constants.UTM_ZONE_NUM,
            Constants.UTM_ZONE_LETTER,
        )
    except (ValueError, utm.error.OutOfRangeError):
        # Return an explicit "no fix" NavSatFix instead of crashing the node
        nav = NavSatFix()
        nav.header = odom_msg.header
        nav.status.status = NavSatStatus.STATUS_NO_FIX
        nav.status.service = 0
        nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return nav

    nav = NavSatFix()
    nav.header = odom_msg.header
    nav.status.status = NavSatStatus.STATUS_FIX
    nav.status.service = NavSatStatus.SERVICE_GPS
    nav.latitude = float(lat)
    nav.longitude = float(lon)
    nav.altitude = float(alt)

    # NavSatFix.position_covariance is row-major 3x3:
    # [lat, lon, alt]
    #
    # If Sigma is provided, interpret it as:
    # Sigma[0] = x/east
    # Sigma[1] = y/north
    # Sigma[2] = yaw
    # Sigma[3] = speed
    #
    # and transfer the planar position covariance directly into the
    # latitude/longitude covariance slots.
    if Sigma is not None:
        pos_cov = np.zeros((3, 3))
        pos_cov[0:2, 0:2] = Sigma[0:2, 0:2]

        # Preserve z variance from odometry if available.
        if hasattr(odom_msg.pose, "covariance") and len(odom_msg.pose.covariance) == 36:
            pos_cov[2, 2] = odom_msg.pose.covariance[14]

        nav.position_covariance = pos_cov.flatten().tolist()
        nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN

    # Otherwise, fall back to the odometry pose covariance if present.
    elif hasattr(odom_msg.pose, "covariance") and len(odom_msg.pose.covariance) == 36:
        odom_cov = np.array(odom_msg.pose.covariance).reshape((6, 6))

        pos_cov = np.zeros((3, 3))
        pos_cov[0, 0] = odom_cov[0, 0]
        pos_cov[0, 1] = odom_cov[0, 1]
        pos_cov[1, 0] = odom_cov[1, 0]
        pos_cov[1, 1] = odom_cov[1, 1]
        pos_cov[2, 2] = odom_cov[2, 2]

        nav.position_covariance = pos_cov.flatten().tolist()
        nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN

    else:
        nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    return nav