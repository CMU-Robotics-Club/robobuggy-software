# A set of object detections from one sensor in one frame, positions in UTM metres.
# One entry per detected object. pos_std is the 1-sigma position uncertainty of that
# detection in metres (range dependent for lidar and camera).
std_msgs/Header header
string source
float64[] easting
float64[] northing
float64[] pos_std
