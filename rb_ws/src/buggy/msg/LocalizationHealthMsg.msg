# All stamps use the node's ROS clock, including replay /clock.
std_msgs/Header header
builtin_interfaces/Time state_stamp
builtin_interfaces/Time valid_until
uint8 OK=0
uint8 DEGRADED=1
uint8 BAD=2
uint8 level
string[] reasons
