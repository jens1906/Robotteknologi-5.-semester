#!/bin/bash
# Check the status of all controllers

echo "Checking controller status..."
source /opt/ros/jazzy/setup.bash
ros2 control list_controllers
