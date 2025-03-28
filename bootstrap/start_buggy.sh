#!/bin/bash

set -x

# Accepts env vars:
# PROJECT_ROOT  the directory where CMU-Robotics-Club/robobuggy-software is pulled (default: $PWD)

: ${PROJECT_ROOT=$PWD}

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

# Check if tmux session exists
if tmux has-session -t buggy 2>/dev/null; then
  # If it exists, send C-c to all panes to stop current processes
  tmux send-keys -t buggy.0 C-c
  tmux send-keys -t buggy.1 C-c
  
  # Wait a moment for processes to terminate
  sleep 1
else
  # Create new tmux session if it doesn't exist
  tmux new-session -d -s buggy

  # don't quit when all panes stop
  tmux set-option -t buggy remain-on-exit on
  
  # Create additional panes if this is the first run
  tmux split-window -h -t buggy
  tmux split-window -h -t buggy
  
  # Set layout to be equal width columns
  tmux select-layout -t buggy even-horizontal

  # add another row on the bottom for running things
  tmux split-window -v -f -t buggy
  
  # Set option to keep panes open after commands exit
  tmux set-option -t buggy remain-on-exit on
fi

# makes it work better
# Maybe the shell isn't initializing fast enough?
sleep 1

# Send commands to each pane
tmux send-keys -t buggy.0 "ros2 launch buggy ${BUGGY}-system.xml" Enter
tmux send-keys -t buggy.1 "ros2 launch buggy ${BUGGY}-main.xml" Enter

# Set up bag recording in the third pane
tmux send-keys -t buggy.2 "startbag"

echo "RoboBuggy tmux session refreshed"

