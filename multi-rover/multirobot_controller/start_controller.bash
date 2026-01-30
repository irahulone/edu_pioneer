#!/usr/bin/env bash
# ── bash: abort on error, undefined var, and enable pipefail ───────────────
set -euo pipefail

# Go into your workspace
cd /home/toughbook/multirobot/multi-rover/multirobot_controller || exit 1

# Export environment variables with defaults
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}
export COLCON_TRACE=${COLCON_TRACE:-0}
export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-$(which python3)}
export COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-$(which python3)}

# Source ROS 2 (Humble) & overlay workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch joy_node in background
ros2 run joy joy_node &
JOY_PID=$!

# On exit (INT/TERM/EXIT), kill just the joy_node
trap 'kill "$JOY_PID" 2>/dev/null || true; pkill -f joy_node' INT TERM EXIT

# Run your multirobot controller UI in foreground
exec ros2 run multirobot_controller multirobot_controller
