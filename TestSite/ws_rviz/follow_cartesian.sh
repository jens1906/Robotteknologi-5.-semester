#!/bin/bash
# Follow the scanning path using Cartesian path planning
# This creates ONE SMOOTH continuous motion without stopping

cd /home/andr465m/Documents/GitHub/Robotteknologi-5.-semester/TestSite/ws_rviz
source install/setup.bash

echo "=========================================="
echo "Cartesian Path Follower"
echo "=========================================="
echo ""
echo "This computes ONE continuous Cartesian trajectory"
echo "through ALL waypoints and executes it smoothly."
echo ""
echo "How it works:"
echo "  1. Sends all waypoints to MoveIt's Cartesian planner"
echo "  2. MoveIt computes a single smooth trajectory"
echo "  3. Trajectory is time-parameterized for smooth motion"
echo "  4. Executed as ONE continuous motion"
echo ""
echo "Result:"
echo "  ✓ NO stopping between waypoints"
echo "  ✓ Smooth continuous motion"
echo "  ✓ Precise end-effector path following"
echo "  ✓ Room to reorient if needed (within tolerance)"
echo ""

ros2 run ur3e_workstation follow_path_cartesian.py

echo ""
echo "Cartesian path execution complete!"
