#!/bin/bash
# Follow the path from /tool_orientation/path topic using Cartesian planning

cd "$(dirname "$0")"

# Source ROS2 workspace
source install/setup.bash

# Run the path follower
ros2 run ur3e_workstation follow_path_cartesian.py
