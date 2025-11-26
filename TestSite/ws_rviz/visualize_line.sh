#!/bin/bash
# Visualize line data from /tool_orientation/xyz_rotation in RViz
# Converts Float64MultiArray to MarkerArray and Marker for visualization

cd "$(dirname "$0")"

# Source ROS2 workspace
source install/setup.bash

# Run the line visualizer
ros2 run ur3e_workstation line_visualizer.py
