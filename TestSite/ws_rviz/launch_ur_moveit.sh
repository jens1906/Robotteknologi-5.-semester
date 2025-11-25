#!/bin/bash
# Launch script for MoveIt RViz client - connects to existing robot on another PC

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to the script directory (workspace root)
cd "$SCRIPT_DIR"

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "=========================================="
echo "Launching MoveIt RViz Client"
echo "=========================================="
echo ""
echo "This will connect to an existing robot and launch:"
echo "- RViz with MoveIt planning interface"
echo "- Your test setup visualization"
echo "- Motion planning capabilities"
echo ""
echo "Make sure the robot driver is running on the other PC!"
echo ""

# Launch only the MoveIt RViz client (no robot drivers)
ros2 launch ur3e_workstation moveit_rviz_client.launch.py ur_type:=ur3e

echo ""
echo "MoveIt RViz client terminated."
