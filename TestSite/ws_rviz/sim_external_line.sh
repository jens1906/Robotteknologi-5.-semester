#!/bin/bash
# Simulate external tool orientation topic for testing import_external_line.sh
# This publishes to /tool_orientation/xyz_rotation topic like another PC would

cd /home/andr465m/Documents/GitHub/Robotteknologi-5.-semester/TestSite/ws_rviz
source install/setup.bash

echo "=========================================="
echo "Simulating External Tool Orientation Topic"
echo "=========================================="
echo ""
echo "Publishing to: /tool_orientation/xyz_rotation"
echo "Format: [x1,y1,z1,r11,r12,r13,r21,r22,r23,r31,r32,r33,x2,y2,z2,...]"
echo ""
echo "This simulates what another PC would publish."
echo "Use this to test ./import_external_line.sh"
echo ""

# Start the simulated external topic publisher
echo "Starting simulated external topic publisher..."
ros2 run ur3e_workstation external_topic_simulator.py &

SIMULATOR_PID=$!

# Wait for the simulator to start
sleep 2

echo ""
echo "=========================================="
echo "External topic simulation is running!"
echo "=========================================="
echo ""
echo "Topic being published:"
echo "  /tool_orientation/xyz_rotation"
echo ""
echo "To test the import:"
echo "  1. Open another terminal"
echo "  2. Run: ./launch_ur_moveit.sh"
echo "  3. In a third terminal, run: ./import_external_line.sh"
echo ""
echo "You should see the simulated path in RViz!"
echo ""
echo "Press Ctrl+C to stop the simulation"
echo ""

# Wait for user to stop
trap "kill $SIMULATOR_PID 2>/dev/null; exit 0" SIGINT SIGTERM EXIT
wait