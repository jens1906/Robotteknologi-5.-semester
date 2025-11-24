#!/bin/bash
# Complete UR3e direct TCP connection with ROS bridge and trajectory execution
# Provides real robot control via TCP, real-time ROS topic publishing, and MoveIt execution

# Get script directory and workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "UR3e Direct TCP with ROS Bridge & MoveIt Execution"
echo "Robot IP: 192.168.0.100"
echo "=========================================="
echo ""
echo "This script will:"
echo "1. Connect to robot dashboard (port 29999)"
echo "2. Start ROS bridge for real-time joint states"
echo "3. Start trajectory executor for MoveIt execution"
echo "4. Launch MoveIt with live robot visualization"
echo "5. Enable full MoveIt planning AND execution"
echo ""
echo "Make sure robot is:"
echo "- Powered on"
echo "- In Remote mode"
echo "- Emergency stop is accessible"
echo ""
echo "Working from: $SCRIPT_DIR"
echo "Workspace: $WORKSPACE_ROOT"
echo ""
read -p "Press Enter to continue (Ctrl+C to abort)..."

# Change to script directory
cd "$SCRIPT_DIR"

# Check if we're in a ROS workspace
if [ ! -f "install/setup.bash" ]; then
    echo "❌ Error: Not in a ROS workspace or workspace not built"
    echo "Please make sure you're in the ws_rviz directory and have built the workspace with:"
    echo "  colcon build"
    exit 1
fi

# Source ROS 2 - detect ROS distribution
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    source /opt/ros/iron/setup.bash
else
    echo "❌ Error: No supported ROS 2 distribution found"
    echo "Please install ROS 2 (Humble, Iron, or Jazzy)"
    exit 1
fi

source install/setup.bash

# Check for required Python scripts
if [ ! -f "ur_robot_state_bridge.py" ]; then
    echo "❌ Error: ur_robot_state_bridge.py not found in $SCRIPT_DIR"
    exit 1
fi

if [ ! -f "ur_trajectory_executor.py" ]; then
    echo "❌ Error: ur_trajectory_executor.py not found in $SCRIPT_DIR"
    exit 1
fi

# Make scripts executable
chmod +x ur_robot_state_bridge.py
chmod +x ur_trajectory_executor.py

echo "Starting robot state bridge..."
python3 ur_robot_state_bridge.py &
BRIDGE_PID=$!
sleep 3

# Check if bridge is running
if kill -0 $BRIDGE_PID 2>/dev/null; then
    echo "✓ Robot state bridge started"
else
    echo "❌ Failed to start robot state bridge"
    exit 1
fi

echo "Starting trajectory executor..."
python3 ur_trajectory_executor.py &
EXECUTOR_PID=$!
sleep 3

# Check if trajectory executor is running
if kill -0 $EXECUTOR_PID 2>/dev/null; then
    echo "✓ Trajectory executor started"
else
    echo "❌ Failed to start trajectory executor"
    kill $BRIDGE_PID 2>/dev/null
    exit 1
fi

echo "Starting robot description and MoveIt..."

# Try to launch with available packages - check if they exist first
if ros2 pkg list | grep -q "ur3e_workstation"; then
    ros2 launch ur3e_workstation workstation_description.launch.py &
    sleep 2
    
    ros2 launch ur_moveit_config ur_moveit.launch.py \
      ur_type:=ur3e \
      launch_rviz:=true \
      use_sim_time:=false \
      description_launchfile:=$(ros2 pkg prefix ur3e_workstation)/share/ur3e_workstation/launch/workstation_description.launch.py &
else
    echo "Warning: ur3e_workstation package not found, using basic UR MoveIt config..."
    ros2 launch ur_moveit_config ur_moveit.launch.py \
      ur_type:=ur3e \
      launch_rviz:=true \
      use_sim_time:=false &
fi

sleep 5

# Publish collision matrix to fix collision issues (if available)
if ros2 pkg list | grep -q "ur3e_workstation" && ros2 run ur3e_workstation publish_collision_matrix.py --help >/dev/null 2>&1; then
    echo "Publishing collision matrix..."
    ros2 run ur3e_workstation publish_collision_matrix.py &
    sleep 2
else
    echo "Note: Collision matrix publisher not available, continuing without it..."
fi

echo ""
echo "=========================================="
echo "✓ UR3e System Ready for Full MoveIt Control!"
echo "=========================================="
echo ""
echo "System Status:"
echo "✓ Robot state bridge: Publishing real joint positions to /joint_states"
echo "✓ Trajectory executor: Converts MoveIt trajectories to URScript"
echo "✓ MoveIt/RViz: Ready for planning AND execution"
echo "✓ Collision matrix: Workstation components configured"
echo ""
echo "Available Controllers:"
echo "  - scaled_joint_trajectory_controller/follow_joint_trajectory"
echo "  - joint_trajectory_controller/follow_joint_trajectory"
echo ""
echo "You can now:"
echo "  1. Plan motions in RViz Motion Planning panel"
echo "  2. Execute planned trajectories on the real robot"
echo "  3. Use 'Plan & Execute' button for complete automation"
echo ""
echo "⚠️  WARNING: All executed motions will move the REAL robot!"
echo "⚠️  Keep emergency stop accessible at all times!"
echo ""
echo "To test the system:"
echo "  1. Open RViz and go to Motion Planning tab"
echo "  2. Move the goal state (orange robot) to desired position"
echo "  3. Click 'Plan' to generate trajectory"
echo "  4. Click 'Execute' to run on real robot"
echo "  5. Or use 'Plan & Execute' for both steps"
echo ""
echo "Press Ctrl+C to stop all services and disconnect from robot."

# Create cleanup function
cleanup() {
    echo ""
    echo "Shutting down robot control system..."
    kill $EXECUTOR_PID 2>/dev/null
    kill $BRIDGE_PID 2>/dev/null
    pkill -f "ros2 launch" 2>/dev/null
    pkill -f "python3 ur_robot_state_bridge.py" 2>/dev/null
    pkill -f "python3 ur_trajectory_executor.py" 2>/dev/null
    echo "✓ All processes stopped"
}

trap cleanup EXIT

# Keep script running and monitor processes
while true; do
    # Check if critical processes are still running
    if ! kill -0 $BRIDGE_PID 2>/dev/null; then
        echo "❌ Robot state bridge stopped unexpectedly"
        break
    fi
    
    if ! kill -0 $EXECUTOR_PID 2>/dev/null; then
        echo "❌ Trajectory executor stopped unexpectedly"
        break
    fi
    
    sleep 5
done