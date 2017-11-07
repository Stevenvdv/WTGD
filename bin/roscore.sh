#!/bin/bash
# file: roscore.sh

source /opt/ros/indigo/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/Documents/driving-willy/WTGD

roscore
