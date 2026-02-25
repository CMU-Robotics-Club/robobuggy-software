from sensor_msgs.msg import NavSatFix, NavSatStatus
import utm
from util.constants import Constants

def odom_to_navsat(odom_msg):
    east  = odom_msg.pose.pose.position.x
    north = odom_msg.pose.pose.position.y
    alt   = odom_msg.pose.pose.position.z

    lat, lon = utm.to_latlon(
        east, north,
        Constants.UTM_ZONE_NUM,
        Constants.UTM_ZONE_LETTER
    )

    nav = NavSatFix()
    nav.header = odom_msg.header
    nav.status.status = NavSatStatus.STATUS_FIX
    nav.status.service = NavSatStatus.SERVICE_GPS
    nav.latitude  = float(lat)
    nav.longitude = float(lon)
    nav.altitude  = float(alt)
    return nav
