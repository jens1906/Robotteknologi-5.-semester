#!/bin/bash
# Launch script for connecting to real UR3e robot
# Robot IP: 192.168.0.100

cd /home/andr465m/Documents/GitHub/Robotteknologi-5.-semester/TestSite/ws_rviz
source install/setup.bash

echo "=========================================="
echo "Connecting to REAL UR3e Robot"
echo "Robot IP: 192.168.0.100"
echo "PC IP: 192.168.0.102"
echo "Using calibration file: ~/ur3e_calibration.yaml"
echo "=========================================="
echo ""
echo "IMPORTANT: Before running this script, make sure:"
echo "1. The robot is powered on and in remote control mode"
echo "2. The 'External Control' URCap program is loaded on the robot"
echo "3. The URCap Host IP is set to: 192.168.0.102"
echo "4. You are on the same network as the robot (192.168.0.x)"
echo "5. The robot is in a safe configuration"
echo ""
read -p "Press Enter to continue or Ctrl+C to abort..."

# Launch the workstation with REAL hardware (not mock)
ros2 launch ur3e_workstation workstation_with_ur_moveit.launch.py \
    robot_ip:=192.168.0.100 \
    use_mock_hardware:=false \
    reverse_ip:=192.168.0.102 \
    kinematics_params_file:=${HOME}/ur3e_calibration.yaml

echo "Connection to real robot terminated."
