#!/bin/bash
# Follow the scanning path using OMPL RRT-Connect planner with relaxed constraints

cd /home/andr465m/Documents/GitHub/Robotteknologi-5.-semester/TestSite/ws_rviz
source install/setup.bash

echo "=========================================="
echo "OMPL RRT-Connect Path Follower"
echo "=========================================="
echo ""
echo "This will follow the scanning path waypoint-by-waypoint"
echo "using the OMPL RRT-Connect planner."
echo ""
echo "Relaxed constraints:"
echo "  - Position tolerance: ±5mm (0.5cm)"
echo "  - Orientation tolerance: ±15 degrees"
echo "  - Collision avoidance: ENABLED"
echo ""
echo "The planner will automatically find collision-free"
echo "paths to each waypoint, even if it needs to deviate"
echo "from the exact trajectory to avoid obstacles."
echo ""

ros2 run ur3e_workstation follow_path_ompl.py

echo ""
echo "Path execution complete!"
