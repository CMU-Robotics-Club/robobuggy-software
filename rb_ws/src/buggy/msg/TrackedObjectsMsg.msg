# Every buggy (or other object) the tracker is currently confident about.
# Positions and velocities in UTM metres and metres per second. age_s is the time
# since that track was last updated by a detection; hits is how many detections
# have supported it. Arrays are parallel; entry i describes track ids[i].
std_msgs/Header header
int32[] ids
float64[] easting
float64[] northing
float64[] vx
float64[] vy
float64[] pos_std
float64[] age_s
int32[] hits
