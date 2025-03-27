#!/bin/bash

set -x

# Accepts env vars:
# PROJECT_ROOT  the directory where CMU-Robotics-Club/robobuggy-software is pulled (default: $PWD)
# BAG_DIR	the directory to write bags to (default: $PROJECT_ROOT/bags)

: ${PROJECT_ROOT=$PWD}
: ${BAG_DIR="$PROJECT_ROOT/bags"}

# ROS environment
source /opt/ros/humble/setup.bash
# python virtualenv
source "${PROJECT_ROOT}/.${BUGGY}/bin/activate"
# colcon bash completions
source "${PROJECT_ROOT}/rb_ws/install/local_setup.bash"
# our environment variables
source "${PROJECT_ROOT}/rb_ws/environments/${BUGGY}_env.bash"

# Run everything in rb_ws
cd "${PROJECT_ROOT}/rb_ws"

# Build the project
colcon build --symlink-install

# Function to create a properly formatted date-based filename
get_datetime_filename() {
    date +%Y-%m-%d_%H-%M-%S
}


# Kill existing tmux session if it exists
tmux kill-session -t buggy 2>/dev/null || true

# Create new tmux session with the first pane running ${BUGGY}-system.xml
tmux new-session -d -s buggy "source /opt/ros/humble/setup.bash && ros2 launch buggy ${BUGGY}-system.xml"

tmux set-option -t buggy remain-on-exit on

# Create second pane running ${BUGGY}-main.xml
tmux split-window -h -t buggy "source /opt/ros/humble/setup.bash && ros2 launch buggy ${BUGGY}-main.xml"

# Create third pane to record a timestamped bag
DATETIME=$(get_datetime_filename)
BAG_FILE="${BAG_DIR}/${DATETIME}"
mkdir -p "${BAG_DIR}"
tmux split-window -h -t buggy

# fix layout to be equal width columns
tmux select-layout -t buggy even-horizontal

echo "RoboBuggy tmux session started"
