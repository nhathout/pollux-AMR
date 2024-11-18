#!/bin/bash

# Source ROS setup files (if needed)
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Install dependencies
pip install -r ~/catkin_ws/src/pollux-AMR/requirements.txt

# Reinstall custom Gymnasium environment
pip install -e ~/catkin_ws/src/pollux-AMR/gym_pollux
