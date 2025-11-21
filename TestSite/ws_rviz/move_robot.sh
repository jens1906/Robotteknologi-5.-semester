#!/bin/bash
# Move robot to a single target position

cd /home/andr465m/Documents/GitHub/Robotteknologi-5.-semester/TestSite/ws_rviz
source install/setup.bash

echo "=========================================="
echo "Move Robot to Target Position"
echo "=========================================="
echo ""
echo "This will actually MOVE the robot!"
echo "Make sure launch_ur_moveit.sh is running."
echo ""

ros2 run ur3e_workstation move_to_single_point.py

echo ""
echo "Movement complete!"
