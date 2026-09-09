uint64 id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
# Row-major covariance of [x, y, vx, vy].
float64[16] covariance
geometry_msgs/Vector3 extent
bool extent_known
builtin_interfaces/Time last_observed_stamp
uint32 observation_count
bool confirmed
string[] sources
