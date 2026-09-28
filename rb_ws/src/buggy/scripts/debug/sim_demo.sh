#!/bin/bash
# sim_demo.sh : (re)start the visual simulation inside the dev container and leave it running.
#
#   From Windows/macOS, with the container up:
#       docker exec robobuggy-software-main-1 /rb_ws/src/buggy/scripts/debug/sim_demo.sh
#       docker exec robobuggy-software-main-1 /rb_ws/src/buggy/scripts/debug/sim_demo.sh single
#       docker exec robobuggy-software-main-1 /rb_ws/src/buggy/scripts/debug/sim_demo.sh double use_speed_model:=true
#   Then Foxglove -> Open connection -> ws://localhost:8765, layout foxglove/racing_sim_layout.json.
#
#   double (default): SC + NAND + two ghost buggies, envelope planner, camera sim, and the simulated
#                     VLP-16 running through the REAL clustering and UTM adapter into the tracker.
#   single:           SC alone on the envelope planner.
#   Extra arguments are passed to ros2 launch. Log: /tmp/sim.log inside the container.
set -e
source /opt/ros/humble/setup.bash
cd /rb_ws && source install/local_setup.bash && source environments/docker_env.bash 2>/dev/null
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
MODE=${1:-double}; shift || true
case "$MODE" in
  double) LF=sim_2d_double.xml; ARGS="use_frenet:=true use_perception_sim:=true use_lidar_sim:=true use_fake_lidar:=false" ;;
  single) LF=sim_2d_single.xml; ARGS="use_frenet:=true" ;;
  *) echo "usage: sim_demo.sh [double|single] [launch args]"; exit 2 ;;
esac
for p in $(pgrep -f "ros2 launch buggy" || true); do kill "$p" 2>/dev/null || true; done
sleep 2
pkill -9 -f "install/buggy/lib/buggy/" 2>/dev/null || true
pkill -9 foxglove_bridge 2>/dev/null || true
sleep 1
setsid nohup ros2 launch buggy "$LF" $ARGS "$@" > /tmp/sim.log 2>&1 < /dev/null &
echo "started: ros2 launch buggy $LF $ARGS $* (log /tmp/sim.log)"
