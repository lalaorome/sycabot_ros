#!/bin/bash

cd home/jetbot/syca_ws
source config/setup_ROS.sh --pkg sycabot --srv 9
ros2 launch sycabot_launch init.launch.py