#!/usr/bin/env zsh
# ── zsh: abort on error + undefined var, then enable pipefail ───────────────
set -euo pipefail
# go into your workspace
cd /home/zen/RSL_Projects/multi-rover/multirobot_controller || exit 1
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}
export COLCON_TRACE=${COLCON_TRACE:-0}
export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-$(which python3)}
export COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# source ROS 2 & overlay
source /opt/ros/humble/setup.zsh
source install/setup.zsh

# launch joy in background
ros2 run joy joy_node &
JOY_PID=$!

# on exit (INT/TERM/normal), kill just the joy node

trap 'kill $JOY_PID 2>/dev/null || true' INT TERM EXIT
trap 'pkill -f joy_node'

# run your UI in foreground
exec ros2 run multirobot_controller multirobot_controller

