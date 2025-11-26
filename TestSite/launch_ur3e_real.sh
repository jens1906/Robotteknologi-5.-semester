#!/bin/bash
# Start UR3e with REAL hardware (connects to physical robot)
# Use this when you have a real UR3e connected

echo "=========================================="
echo "Starting UR3e Driver (REAL HARDWARE)"
echo "Robot IP: 192.168.56.2"
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

# Launch with REAL hardware
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=192.168.56.101 \
    use_mock_hardware:=false \
    launch_rviz:=false \
    controller_manager.ros__parameters.update_rate:=100 \
    kinematics_params_file:=${HOME}/ur3e_calibration.yaml

echo "Real robot driver stopped."
