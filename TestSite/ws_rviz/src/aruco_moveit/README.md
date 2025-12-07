# ArUco MoveIt Package

This package provides a ROS 2 node that tracks ArUco markers and moves the UR3e robot to a position above detected markers using MoveIt's Cartesian path planning.

## Overview

The `ur5_moveit_client` node:

- Subscribes to `/aruco_pose` topic for marker detection
- Uses MoveIt's Cartesian path planning for smooth motion
- Moves the robot end-effector to a configurable height above the first detected marker
- Applies velocity and acceleration scaling for safe operation

## Building

```bash
cd /home/daniel/Documents/GitHub/Robotteknologi-5.-semester/TestSite/ws_rviz
colcon build --packages-select aruco_moveit
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch aruco_moveit aruco_moveit.launch.py
```

### With Custom Parameters

```bash
ros2 launch aruco_moveit aruco_moveit.launch.py \
    lift:=0.25 \
    velocity_scaling:=0.2 \
    acceleration_scaling:=0.2 \
    execute_motion:=true
```

### Launch Parameters

- `base_frame` (default: `base_link`) - Robot base frame
- `ee_frame` (default: `tool0`) - End-effector frame
- `lift` (default: `0.20`) - Height above marker in meters
- `execute_motion` (default: `true`) - Execute the planned motion
- `velocity_scaling` (default: `0.1`) - Velocity scaling factor (0.0-1.0)
- `acceleration_scaling` (default: `0.1`) - Acceleration scaling factor (0.0-1.0)

## Prerequisites

This node requires:

1. MoveIt motion planning framework running
2. ArUco marker detection publishing to `/aruco_pose`
3. Robot hardware or simulation with `/joint_states` topic
4. TF transforms between `base_link`, marker frame, and `tool0`

## Key Features

### Cartesian Path Planning

Uses MoveIt's `compute_cartesian_path` service for smooth, collision-aware motion planning.

### Orientation Handling

Applies automatic orientation transformations:

- 90° rotation around Z-axis
- 180° rotation around X-axis
- Quaternion normalization

### Safety Features

- Configurable velocity and acceleration scaling
- Path validation (requires 95% path completion)
- Single-move limit (only moves to first detected marker)
- Collision avoidance enabled

## Troubleshooting

### "compute_cartesian_path not available"

Ensure MoveIt is running:

```bash
ros2 service list | grep compute_cartesian_path
```

### "execute_trajectory not available"

Check that the MoveIt execution action server is running:

```bash
ros2 action list | grep execute_trajectory
```

### "Cartesian path planning failed"

- Check if target pose is reachable
- Verify no collisions in the path
- Try increasing `lift` parameter
- Reduce velocity/acceleration scaling

### No ArUco detection

Verify ArUco publisher is running:

```bash
ros2 topic echo /aruco_pose
```

## Integration Example

To use with your existing UR3e setup:

```bash
# Terminal 1: Launch robot with MoveIt
ros2 launch ur3e_moveit_config workstation_with_ur_moveit.launch.py

# Terminal 2: Launch ArUco detection (if you have one)
ros2 run your_aruco_package aruco_detector

# Terminal 3: Launch this client
ros2 launch aruco_moveit aruco_moveit.launch.py
```

## Node Details

**Node name:** `ur5_moveit_client`

**Subscribed topics:**

- `/aruco_pose` (geometry_msgs/PoseStamped) - ArUco marker pose
- `/joint_states` (sensor_msgs/JointState) - Current joint positions

**Service clients:**

- `/compute_cartesian_path` (moveit_msgs/GetCartesianPath) - Cartesian planning

**Action clients:**

- `/execute_trajectory` (moveit_msgs/ExecuteTrajectory) - Trajectory execution
