#!/bin/bash

set -x

# Accepts env vars:
# PROJECT_ROOT  the directory where CMU-Robotics-Club/robobuggy-software is pulled (default: $PWD)
# BAG_DIR       the directory to write bags to (default: $PROJECT_ROOT/bags)

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


# Check if tmux session exists
if tmux has-session -t buggy 2>/dev/null; then
  # see stop_buggy
  tmux respawn-pane -k -t buggy.0
  tmux respawn-pane -k -t buggy.1
  tmux send-keys -t buggy.2 C-c
  
  # Wait a moment for processes to terminate
  sleep 1
fi

