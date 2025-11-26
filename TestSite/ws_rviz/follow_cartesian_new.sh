#!/bin/bash

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run the new Cartesian path follower
echo "Starting Cartesian Path Follower (New Method - compute_cartesian_path)"
echo "====================================================================="
echo ""

python3 src/ur3e_workstation/scripts/follow_path_cartesian_new.py
