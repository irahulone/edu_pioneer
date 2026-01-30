#!/bin/bash

cd ~/multirobot/multi-rover/rover_ws

# if ROS_DISTRO isn’t already set (e.g. on a fresh shell), try to pick one up:
if [ -z "$ROS_DISTRO" ]; then
  # if you have rosversion available, use it:
  if command -v rosversion &>/dev/null; then
    export ROS_DISTRO=$(rosversion -d)
  else
    # otherwise just grab the first folder name under /opt/ros
    export ROS_DISTRO=$(basename "$(ls -1 /opt/ros | head -n1)")
  fi
fi

# now source the correct setup.bash
if [ -d "/opt/ros/$ROS_DISTRO" ]; then
  source "/opt/ros/$ROS_DISTRO/setup.bash"
else
  echo "WARNING: /opt/ros/$ROS_DISTRO does not exist"
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
. install/setup.bash
ros2 run rover_core rover_main
cd -

