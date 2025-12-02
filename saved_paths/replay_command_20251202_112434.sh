#!/bin/bash
# Replay saved PoseArray to /tool_orientation/path topic

ros2 topic pub --once /tool_orientation/path geometry_msgs/msg/PoseArray \
  "$(cat posearray_20251202_112434.yaml)"
