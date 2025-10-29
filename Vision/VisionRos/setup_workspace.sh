#!/bin/bash

# ROS2 Workspace Setup Script

echo "Setting up ROS2 workspace..."

# Source ROS2 Jazzy
echo "Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo "Error: ROS2 is not installed or not in PATH"
    echo "Please install ROS2 Jazzy first:"
    echo "sudo apt update && sudo apt install ros-jazzy-desktop"
    exit 1
fi

# Navigate to workspace
cd "$(dirname "$0")"

echo "Building workspace..."
colcon build

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "Sourcing workspace..."
    source install/setup.bash
    
    echo ""
    echo "ROS2 workspace is ready!"
    echo ""
    echo "To run the example nodes:"
    echo "Terminal 1: ros2 run my_robot_package publisher_node"
    echo "Terminal 2: ros2 run my_robot_package subscriber_node"
    echo ""
    echo "Don't forget to source the workspace in each new terminal:"
    echo "source install/setup.bash"
else
    echo "Build failed. Please check the error messages above."
    exit 1
fi
