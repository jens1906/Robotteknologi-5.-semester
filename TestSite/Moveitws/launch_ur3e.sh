#!/bin/bash

# Quick launch script for UR3e MoveIt setup
# This script sources the workspace and launches the system

echo "=========================================="
echo "UR3e MoveIt Launcher"
echo "=========================================="

# Navigate to workspace
cd "$(dirname "$0")"

# Source ROS2 Jazzy
echo "Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# Source workspace
echo "Sourcing workspace..."
source install/setup.bash

# Launch the system
echo "Launching UR3e with MoveIt and RViz2..."
echo ""
ros2 launch ur3e_moveit_config ur3e_moveit.launch.py
