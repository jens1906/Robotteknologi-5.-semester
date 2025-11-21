#!/bin/bash
# Launch script for UR3e with REAL ROBOT connection and MoveIt configuration
# Robot IP: 192.168.0.100
# Make sure the robot is in "Remote" mode before running this script

echo "=========================================="
echo "Launching UR3e MoveIt with REAL ROBOT"
echo "Robot IP: 192.168.0.100"
echo "=========================================="
echo ""
echo "IMPORTANT: Before running this script, make sure:"
echo "1. The robot is powered on and in REMOTE control mode"
echo "2. You are on the same network (192.168.0.x)"
echo "3. The robot is ready to accept external connections"
echo "4. Emergency stop is accessible"
echo ""
read -p "Press Enter to continue with REAL robot connection (Ctrl+C to abort)..."

cd /home/andr465m/Documents/GitHub/Robotteknologi-5.-semester/TestSite/ws_rviz

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch UR control with REAL HARDWARE using our configuration file
echo "Starting UR3e driver with REAL robot connection..."
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.0.100 \
  use_mock_hardware:=false \
  initial_joint_controller:=scaled_joint_trajectory_controller \
  activate_joint_controller:=true \
  launch_rviz:=false \
  headless_mode:=true \
  robot_driver_params_file:=$(pwd)/ur3e_robot_params.yaml \
  description_launchfile:=$(ros2 pkg prefix ur3e_workstation)/share/ur3e_workstation/launch/workstation_description.launch.py &

# Store the PID of the driver process
DRIVER_PID=$!

# Wait for the driver to establish connection
echo "Waiting for robot connection to establish..."
sleep 15

# Check if the driver process is still running
if ! kill -0 $DRIVER_PID 2>/dev/null; then
    echo "ERROR: Robot driver process crashed. Check the robot connection."
    echo "Please verify:"
    echo "1. Robot is powered on and in Remote mode"
    echo "2. Network connection to 192.168.0.100 is working"
    echo "3. Robot may need External Control URCap program running"
    echo ""
    echo "For UR robots, you may need to:"
    echo "- Load and start an External Control program on the robot"
    echo "- Set Host IP to your computer's IP address"
    exit 1
fi

# Better check for controller manager
echo "Checking if robot driver is working..."
for i in {1..10}; do
    if ros2 service list | grep -q "/controller_manager/list_controllers"; then
        echo "✓ Controller manager is running"
        break
    elif [ $i -eq 10 ]; then
        echo "ERROR: Controller manager not available after 10 attempts."
        echo "The robot driver failed to connect properly."
        echo ""
        echo "Common solutions:"
        echo "1. Start External Control program on robot teach pendant"
        echo "2. Check if robot is in Remote mode"
        echo "3. Verify network connectivity: ping 192.168.0.100"
        echo "4. Check firewall settings"
        exit 1
    else
        echo "Attempt $i/10: Waiting for controller manager..."
        sleep 2
    fi
done

# Check joint states
echo "Checking joint states..."
timeout 5s ros2 topic echo /joint_states --once &>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ Joint states are being published"
else
    echo "⚠  Warning: Joint states not available yet"
    echo "The robot may need an External Control program running"
fi

# Launch MoveIt with RViz in the background
echo "Launching MoveIt motion planner with REAL robot..."
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e \
  launch_rviz:=true \
  use_sim_time:=false \
  description_launchfile:=$(ros2 pkg prefix ur3e_workstation)/share/ur3e_workstation/launch/workstation_description.launch.py &

# Wait for MoveIt to start
echo "Waiting for MoveIt to initialize..."
sleep 8

# Publish collision matrix to allow camera and holder to move without collision errors
echo "Publishing collision matrix for workstation components..."
ros2 run ur3e_workstation publish_collision_matrix.py

echo ""
echo "=========================================="
echo "UR3e REAL ROBOT with MoveIt is ready!"
echo "=========================================="
echo ""
echo "You can now plan and execute motions in RViz."
echo "⚠️  WARNING: This will move the REAL robot!"
echo "⚠️  Keep emergency stop accessible at all times!"
echo ""
echo "Robot Status:"
echo "- Connection: Real robot at 192.168.0.100"
echo "- Controller: scaled_joint_trajectory_controller"
echo "- Safety: Emergency stop should be accessible"
echo ""
echo "To test the connection, try:"
echo "  ros2 topic echo /joint_states"
echo "  ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers"
echo ""
echo "If robot doesn't move, you may need to:"
echo "1. Start External Control program on robot teach pendant"
echo "2. Set Host IP to your computer's IP in the External Control program"
echo ""
echo "Press Ctrl+C to stop all nodes and disconnect from robot."
echo ""

# When user presses Ctrl+C, kill all background processes
trap "echo 'Stopping robot connection and shutting down...'; trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
wait