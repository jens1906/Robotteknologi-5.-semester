#!/bin/bash
# Follow the simulated line path using Cartesian planning with collision avoidance

cd "$(dirname "$0")"

# Source ROS2 workspace
source install/setup.bash

# Run the line path follower
ros2 run ur3e_workstation follow_line_cartesian.py
