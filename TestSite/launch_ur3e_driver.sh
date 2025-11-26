#!/bin/bash
# Start UR3e with MOCK hardware (simulation without real robot)
# This is for testing/development without a physical robot

echo "=========================================="
echo "Starting UR3e Driver (SIMULATION MODE)"
echo "No real robot required"
echo "=========================================="
echo ""
echo "This will start the simulated robot driver."
echo "After this starts, launch MoveIt in another terminal"
echo ""
read -p "Press Enter to continue..."

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Launch with mock hardware (simulation)
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=192.168.56.2 \
    use_mock_hardware:=true \
    launch_rviz:=false \
    controller_manager.ros__parameters.update_rate:=100 \
    kinematics_params_file:=${HOME}/ur3e_calibration.yaml

echo "Driver stopped."
