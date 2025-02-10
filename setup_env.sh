#!/bin/bash

# Source ROS setup files (if needed)
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Install dependencies
pip install -r ./requirements.txt

# Reinstall custom Gymnasium environment
pip install -e ./pollux-AMR/gym_pollux
