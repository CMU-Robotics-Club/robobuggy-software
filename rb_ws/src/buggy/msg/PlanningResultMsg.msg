# Only this envelope authorizes the experimental controller. Diagnostic geometry
# is not an authorization. The controller checks the deadline every cycle.
std_msgs/Header header
uint64 plan_id
builtin_interfaces/Time state_stamp
builtin_interfaces/Time tracks_stamp
builtin_interfaces/Time valid_until
float64 start_station
float64 end_station
buggy/TrajectoryMsg trajectory
uint8 NONE=0
uint8 ELIGIBLE=1
uint8 DEGRADED=2
uint8 INELIGIBLE=3
uint8 status
bool control_eligible
string[] reasons
float64 max_curvature
float64 min_hard_clearance
float64 min_road_margin
bool search_budget_exhausted
