#!/bin/bash

# Direct launch script using official UR packages
# This bypasses our custom package and uses UR packages directly

echo "=========================================="
echo "UR3e MoveIt Launcher (Direct Method)"
echo "=========================================="

# Source ROS2 Jazzy
echo "Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# Launch directly from ur_moveit_config
echo "Launching UR3e with MoveIt and RViz2..."
echo ""
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e use_fake_hardware:=true launch_rviz:=true
