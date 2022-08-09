#!/bin/bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="192.168.1.9:11811"
export ROS_IP=192.168.1.9
export ROS_HOSTNAME=192.168.1.9
export ROS_MASTER_URI=http://192.168.1.9:11311
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
