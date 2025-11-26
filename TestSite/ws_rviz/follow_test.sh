#!/bin/bash
# Test script - Move robot to single test position
# Position: [-0.150, 0.300, 0.200]

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "=========================================="
echo "Test Move to Single Position"
echo "=========================================="
echo ""
echo "Target: [-0.150, 0.300, 0.200]"
echo ""
echo "This will test basic MoveIt functionality"
echo "by moving to just the first point from"
echo "the line simulator."
echo ""

python3 src/ur3e_workstation/scripts/move_to_test_point.py
