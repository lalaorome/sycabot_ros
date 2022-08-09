#!/bin/bash

source /opt/ros/noetic/setup.bash
export ROS_IP=192.168.1.9
export ROS_HOSTNAME=192.168.1.9
export ROS_MASTER_URI=http://192.168.1.9:11311
roslaunch vrpn_client_ros sample.launch server:=192.168.1.11 
