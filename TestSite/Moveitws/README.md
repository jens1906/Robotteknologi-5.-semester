# UR3e MoveIt Configuration for ROS2 Jazzy

This package provides a complete MoveIt setup for the UR3e robot with RViz2 visualization.

## What's Included

- UR3e robot URDF model
- MoveIt motion planning configuration
- OMPL planner settings (RRTConnect, RRT, TRRT)
- KDL kinematics solver
- RViz2 configuration with MotionPlanning plugin
- Launch file to start everything

## Prerequisites

Make sure you have ROS2 Jazzy and MoveIt installed:

```bash
sudo apt install ros-jazzy-desktop
sudo apt install ros-jazzy-moveit
sudo apt install ros-jazzy-moveit-planners-ompl
sudo apt install ros-jazzy-moveit-ros-visualization
sudo apt install ros-jazzy-joint-state-publisher-gui
```

## Build Instructions

```bash
# Navigate to workspace
cd ~/Documents/GitHub/Robotteknologi-5.-semester/TestSite/Moveitws

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Launch Instructions

To launch the UR3e robot with MoveIt and RViz2:

```bash
# Make sure you're in the workspace and have sourced it
cd ~/Documents/GitHub/Robotteknologi-5.-semester/TestSite/Moveitws
source install/setup.bash

# Launch the system
ros2 launch ur3e_moveit_config ur3e_moveit.launch.py
```

This will open:
- **RViz2** with the UR3e robot model
- **Joint State Publisher GUI** to manually move joints
- **MoveIt Move Group** for motion planning

## Using the System

### In RViz2:

1. **MotionPlanning Panel** (left side):
   - Select planning group: `ur_manipulator`
   - Choose planner: `RRTConnect` (default)
   - Use interactive markers to set goal pose
   - Click "Plan" to compute trajectory
   - Click "Execute" to move robot (simulation only)

2. **Joint State Publisher GUI** (separate window):
   - Use sliders to manually control each joint
   - Watch the robot move in RViz2

### Available Joints:

- `shoulder_pan_joint` - Base rotation
- `shoulder_lift_joint` - Shoulder pitch
- `elbow_joint` - Elbow pitch
- `wrist_1_joint` - Wrist roll
- `wrist_2_joint` - Wrist pitch
- `wrist_3_joint` - Wrist yaw

## Planning Groups

- **ur_manipulator**: All 6 joints of the UR3e arm

## Planning Algorithms

- **RRTConnect** (default) - Fast, bidirectional RRT
- **RRT** - Standard Rapidly-exploring Random Tree
- **TRRT** - Transition-based RRT

## Workspace Structure

```
ur3e_moveit_config/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── ur3e_moveit.launch.py
├── config/
│   ├── ur3e_moveit.yaml       # MoveIt configuration
│   ├── kinematics.yaml         # IK solver settings
│   ├── joint_limits.yaml       # Joint velocity/acceleration limits
│   └── moveit.rviz             # RViz configuration
└── urdf/
    └── ur3e.urdf               # Robot description
```

## Troubleshooting

### Robot doesn't appear in RViz
```bash
# Check if robot_description is published
ros2 topic echo /robot_description --once
```

### Planning fails
- Make sure the goal pose is reachable
- Try increasing planning time in MotionPlanning panel
- Check joint limits in `config/joint_limits.yaml`

### Build errors
```bash
# Make sure all dependencies are installed
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build install log
colcon build --symlink-install
```

### Can't find package after building
```bash
# Make sure to source the workspace
source install/setup.bash
```

## Next Steps

1. **Test motion planning**: Use RViz interactive markers to plan motions
2. **Write Python scripts**: Use MoveIt Python API to control the robot programmatically
3. **Add collision objects**: Use Planning Scene to add obstacles
4. **Connect to real robot**: Replace fake hardware with real UR3e driver

## Python Control Example

After launching, you can control the robot with Python:

```python
import rclpy
from moveit.planning import MoveItPy

rclpy.init()
moveit = MoveItPy(node_name="ur3e_controller")
ur_manipulator = moveit.get_planning_component("ur_manipulator")

# Plan to home position
ur_manipulator.set_start_state_to_current_state()
# ... add planning code here

rclpy.shutdown()
```

## License

Apache 2.0
