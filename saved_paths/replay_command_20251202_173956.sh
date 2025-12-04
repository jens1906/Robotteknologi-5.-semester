#!/bin/bash
# Replay saved PoseArray to /tool_orientation/path topic

ros2 topic pub --rate 2 /tool_orientation/path geometry_msgs/msg/PoseArray \
  "$(cat )"
