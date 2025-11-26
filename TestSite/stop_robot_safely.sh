#!/bin/bash
# Gracefully stop the robot and controllers

echo "Stopping robot safely..."

# Stop the scaled_joint_trajectory_controller first
ros2 control switch_controllers \
    --deactivate scaled_joint_trajectory_controller \
    --activate-asap

# Give it time to stop
sleep 1

# Now you can safely Ctrl-C the driver
echo "Controllers stopped. Safe to terminate driver now."
