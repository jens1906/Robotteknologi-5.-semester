#!/bin/bash

# Test script to verify the UR3e MoveIt setup works correctly

echo "=========================================="
echo "UR3e MoveIt Setup Test"
echo "=========================================="
echo ""

# Check if in correct directory
if [ ! -f "launch_ur3e.sh" ]; then
    echo "❌ Error: Not in Moveitws directory"
    echo "Run this from: ~/Documents/GitHub/Robotteknologi-5.-semester/TestSite/Moveitws"
    exit 1
fi

echo "✓ In correct directory"

# Check if ROS2 Jazzy is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠ Warning: ROS2 not sourced yet (will be sourced by launch script)"
else
    echo "✓ ROS2 $ROS_DISTRO is sourced"
fi

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Error: Workspace not built"
    echo "Run: colcon build --symlink-install"
    exit 1
fi

echo "✓ Workspace is built"

# Check for required files
required_files=(
    "src/ur3e_moveit_config/package.xml"
    "src/ur3e_moveit_config/launch/ur3e_moveit.launch.py"
    "src/ur3e_moveit_config/urdf/ur3e.urdf"
    "src/ur3e_moveit_config/config/ur3e_moveit.yaml"
    "src/ur3e_moveit_config/config/kinematics.yaml"
)

for file in "${required_files[@]}"; do
    if [ ! -f "$file" ]; then
        echo "❌ Missing: $file"
        exit 1
    fi
done

echo "✓ All required files present"
echo ""
echo "=========================================="
echo "Setup verified! ✓"
echo "=========================================="
echo ""
echo "To launch the system, run:"
echo "  ./launch_ur3e.sh"
echo ""
echo "Or manually:"
echo "  source install/setup.bash"
echo "  ros2 launch ur3e_moveit_config ur3e_moveit.launch.py"
