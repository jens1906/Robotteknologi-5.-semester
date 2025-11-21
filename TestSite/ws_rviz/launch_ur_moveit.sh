#!/bin/bash
# Launch script for UR3e with official MoveIt configuration and workstation test setup

cd /home/andr465m/Documents/GitHub/Robotteknologi-5.-semester/TestSite/ws_rviz

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch UR control with mock hardware and workstation description in the background
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true \
  initial_joint_controller:=scaled_joint_trajectory_controller \
  update_rate_config_file:=$(ros2 pkg prefix ur_robot_driver)/share/ur_robot_driver/config/ur3e_update_rate.yaml \
  description_launchfile:=$(ros2 pkg prefix ur3e_workstation)/share/ur3e_workstation/launch/workstation_description.launch.py &

# Wait for the driver to start
echo "Starting UR3e driver with workstation test setup..."
sleep 5

# Launch MoveIt with RViz in the background
echo "Launching MoveIt motion planner with test setup..."
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e \
  launch_rviz:=true \
  description_launchfile:=$(ros2 pkg prefix ur3e_workstation)/share/ur3e_workstation/launch/workstation_description.launch.py &

# Wait for MoveIt to start
echo "Waiting for MoveIt to initialize..."
sleep 5

# Publish collision matrix to allow camera and holder to move without collision errors
echo "Publishing collision matrix..."
ros2 run ur3e_workstation publish_collision_matrix.py

echo ""
echo "=========================================="
echo "UR3e with MoveIt and test setup is ready!"
echo "=========================================="
echo ""
echo "You can now plan and execute motions in RViz."
echo "The camera and holder will not cause collision errors."
echo ""
echo "Press Ctrl+C to stop all nodes."
echo ""

# When user presses Ctrl+C, kill all background processes
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
wait
