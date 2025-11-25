#!/bin/bash
# Import external scanning path from /tool_orientation/xyz_rotation topic into RViz
# This connects to a path being published from another PC

cd /home/andr465m/Documents/GitHub/Robotteknologi-5.-semester/TestSite/ws_rviz
source install/setup.bash

echo "=========================================="
echo "Importing External Tool Orientation Path"
echo "=========================================="
echo ""
echo "Connecting to topic: /tool_orientation/xyz_rotation"
echo "Expected format: [x1,y1,z1,r11,r12,r13,r21,r22,r23,r31,r32,r33,x2,y2,z2,...]"
echo ""
echo "Make sure:"
echo "1. launch_ur_moveit.sh is already running!"
echo "2. The other PC is publishing to /tool_orientation/xyz_rotation"
echo ""

# Check if the topic exists
echo "Checking if topic /tool_orientation/xyz_rotation is available..."
if ! ros2 topic list | grep -q "/tool_orientation/xyz_rotation"; then
    echo "⚠️  WARNING: Topic /tool_orientation/xyz_rotation not found!"
    echo "   Make sure the other PC is publishing the path data."
    echo ""
fi

# Start the path follower node for visualization and execution
echo "Starting path visualization node (connecting to external topic)..."
ros2 run ur3e_workstation path_follower.py \
  --ros-args \
  -p planning_group:=ur_manipulator \
  -p end_effector_link:=tool0 \
  -p reference_frame:=world \
  -p path_topic:=/tool_orientation/xyz_rotation &

PATH_FOLLOWER_PID=$!

# Wait for the path follower to initialize
sleep 3

echo ""
echo "=========================================="
echo "External path visualization is now running!"
echo "=========================================="
echo ""
echo "The system is now listening for path data on:"
echo "  Topic: /tool_orientation/xyz_rotation"
echo "  Format: [x1,y1,z1,r11,r12,r13,r21,r22,r23,r31,r32,r33,...]"
echo ""
echo "TO SEE THE PATH IN RViz, ADD THESE DISPLAYS:"
echo ""
echo "1. In RViz, click the 'Add' button (bottom left)"
echo "2. Go to 'By topic' tab"
echo "3. Find and add:"
echo "   - /path_visualization -> MarkerArray"
echo "   - /planned_path_line -> Marker"
echo ""
echo "You will see:"
echo "  - Green line showing the scanning path"
echo "  - Colored arrows (red->blue) showing tool orientations"
echo ""
echo "Topics being subscribed to:"
echo "  - /tool_orientation/xyz_rotation (external path data)"
echo ""
echo "Topics being published:"
echo "  - /path_visualization (orientation arrows)"
echo "  - /planned_path_line (green path line)"
echo ""
echo "To execute the path, call:"
echo "  ros2 service call /execute_cartesian_path std_srvs/srv/Trigger"
echo ""
echo "Press Ctrl+C to stop the path visualization"
echo ""

# Wait for user to stop
trap "kill $PATH_FOLLOWER_PID 2>/dev/null; exit 0" SIGINT SIGTERM EXIT
wait