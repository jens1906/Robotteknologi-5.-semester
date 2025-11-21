#!/bin/bash
# Test moving the UR3e robot using the scaled joint trajectory controller

echo "=========================================="
echo "Testing UR3e Robot Movement"
echo "=========================================="
echo ""
echo "This will move the robot through a test trajectory."
echo "Make sure:"
echo "  1. The robot driver is running (./launch_ur3e_driver.sh)"
echo "  2. External Control is playing on teach pendant"
echo "  3. The workspace is clear"
echo "  4. Emergency stop is accessible"
echo ""
read -p "Press Enter to start robot movement test (Ctrl+C to abort)..."

source /opt/ros/jazzy/setup.bash

# Launch the test trajectory
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py

echo "Movement test completed."
