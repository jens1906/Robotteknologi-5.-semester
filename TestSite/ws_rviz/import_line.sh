#!/bin/bash
# Import scanning path visualization into running RViz
# Run this AFTER launching ./launch_ur_moveit.sh

cd /home/andr465m/Documents/GitHub/Robotteknologi-5.-semester/TestSite/ws_rviz
source install/setup.bash

echo "=========================================="
echo "Importing Scanning Path Visualization"
echo "=========================================="
echo ""
echo "Make sure launch_ur_moveit.sh is already running!"
echo ""

# Start the path follower node for visualization
echo "Starting path visualization node..."
ros2 run ur3e_workstation path_follower.py \
  --ros-args \
  -p planning_group:=ur_manipulator \
  -p end_effector_link:=tool0 \
  -p reference_frame:=world &

PATH_FOLLOWER_PID=$!

# Wait a moment for the path follower to initialize
sleep 2

# Start the test plate path publisher to generate and publish the scanning path
echo "Publishing scanning path along test plate..."
ros2 run ur3e_workstation testplate_path_publisher.py &

TESTPLATE_PID=$!

# Wait for path to be generated
sleep 2

echo ""
echo "=========================================="
echo "Path visualization is now running!"
echo "=========================================="
echo ""
echo "TO SEE THE PATH IN RViz, ADD THESE DISPLAYS:"
echo ""
echo "1. In RViz, click the 'Add' button (bottom left)"
echo "2. Go to 'By topic' tab"
echo "3. Find and add:"
echo "   - /path_visualization -> MarkerArray"
echo "   - /planned_path_line -> Marker"
echo ""
echo "You will then see:"
echo "  - Green line showing the scanning path"
echo "  - Colored arrows (red->blue) showing orientations"
echo ""
echo "Topics being published:"
echo "  - /scanning_path (raw path data)"
echo "  - /path_visualization (orientation arrows)"
echo "  - /planned_path_line (green path line)"
echo ""
echo "Press Ctrl+C to stop the path visualization"
echo ""

# Wait for user to stop
trap "kill $PATH_FOLLOWER_PID $TESTPLATE_PID 2>/dev/null; exit 0" SIGINT SIGTERM EXIT
wait
