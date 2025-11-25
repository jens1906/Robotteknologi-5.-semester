#!/bin/bash

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run the first waypoint tester
echo "Starting First Waypoint Tester"
echo "=============================="
echo ""

python3 src/ur3e_workstation/scripts/follow_test.py
