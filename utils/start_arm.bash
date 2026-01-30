#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch open_manipulator_bringup omx.launch.py port_name:=/dev/ttyACM1 init_position:=false
