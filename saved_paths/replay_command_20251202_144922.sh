#!/bin/bash
# Replay saved PoseArray to /tool_orientation/path topic

ros2 topic pub --once /tool_orientation/path geometry_msgs/msg/PoseArray \
  "$(cat /home/jens/Documents/GitHub/Robotteknologi-5.-semester/saved_paths/posearray_20251202_144922.yaml)"
