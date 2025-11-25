#!/bin/bash
# Start ONLY the UR3e driver (no RViz, no MoveIt)
# This provides the hardware interface for the robot

echo "=========================================="
echo "Starting UR3e Driver (Hardware Interface)"
echo "Robot IP: 192.168.0.100"
echo "PC IP: 192.168.0.102"
echo "=========================================="
echo ""
echo "This will start the robot driver and wait for connection."
echo "After this starts, you need to:"
echo "  1. Press Play on the External Control program (teach pendant)"
echo "  2. Launch MoveIt in another terminal"
echo ""
read -p "Press Enter to continue..."

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Launch ONLY the driver (no RViz)
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=192.168.56.2 \
    use_mock_hardware:=true \
    launch_rviz:=false \
    kinematics_params_file:=${HOME}/ur3e_calibration.yaml

echo "Driver stopped."
