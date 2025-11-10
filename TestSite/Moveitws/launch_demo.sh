#!/bin/bash

# UR3e MoveIt Demo with snap workaround
# This fixes the libpthread symbol lookup error from snap packages

echo "=========================================="
echo "UR3e MoveIt Demo (Simulation Only)"
echo "=========================================="

# Navigate to workspace
cd "$(dirname "$0")"

# Source ROS2 Jazzy
echo "Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# Source our workspace
echo "Sourcing workspace..."
source install/setup.bash

# Unset snap-related environment variables that cause conflicts
echo "Removing snap library conflicts..."
unset GTK_PATH
unset LD_LIBRARY_PATH_BACKUP
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

# Remove snap paths from LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:$//')

# Launch our standalone demo
echo "Launching UR3e in demo/simulation mode..."
echo "This will open RViz with the robot model"
echo ""
ros2 launch ur3e_moveit_config ur3e_standalone_demo.launch.py
