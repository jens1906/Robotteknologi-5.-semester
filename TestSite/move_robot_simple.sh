#!/bin/bash
# Simple Python example to move the robot

echo "=========================================="
echo "Simple Robot Movement Example"
echo "=========================================="
echo ""
echo "This uses a basic Python node to move the robot."
echo "WARNING: This has minimal safety checks!"
echo ""
echo "Make sure:"
echo "  1. The robot driver is running"
echo "  2. External Control is playing"
echo "  3. The workspace is CLEAR"
echo "  4. You can reach the emergency stop"
echo ""
read -p "Press Enter to move the robot (Ctrl+C to abort)..."

source /opt/ros/jazzy/setup.bash

# Run the example move script
ros2 run ur_robot_driver example_move.py

echo "Movement completed."
