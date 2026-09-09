# Position and its covariance are in the containing array's capture-time frame.
geometry_msgs/Point position
float64[9] position_covariance
geometry_msgs/Vector3 extent
bool extent_known
string object_id
string class_id
float32 confidence
# SDK predictions must never count as another observation/confirmation.
bool observed
