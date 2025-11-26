#!/bin/bash
# Simulate a configurable line in robot base frame
# Publishes to /tool_orientation/xyz_rotation

cd "$(dirname "$0")"

# Source ROS2 workspace
source install/setup.bash

# Run the line simulator
ros2 run ur3e_workstation line_simulator.py
