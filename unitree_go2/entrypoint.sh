#!/bin/bash
set -e

# Source ROS and Catkin workspace
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Allow GUI access
xhost +local:root

# Start roscore in the background
roscore &

