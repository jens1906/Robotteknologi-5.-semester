# ROS2 Workspace

This is a ROS2 workspace with Python nodes for robot development.

## Workspace Structure

```
VisionRos/
├── src/
│   └── my_robot_package/
│       ├── my_robot_package/
│       │   ├── __init__.py
│       │   ├── publisher_node.py
│       │   └── subscriber_node.py
│       ├── resource/
│       ├── package.xml
│       └── setup.py
├── build/
├── install/
└── log/
```

## Setup Instructions

1. **Install ROS2** (if not already installed):
   ```bash
   # For Ubuntu 24.04 (Jazzy)
   sudo apt update && sudo apt install ros-jazzy-desktop
   ```

2. **Source ROS2**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

3. **Build the workspace**:
   ```bash
   cd /home/jens/Documents/GitHub/Robotteknologi-5.-semester/Vision/VisionRos
   colcon build
   ```

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Running the Nodes

### Terminal 1 - Publisher Node:
```bash
ros2 run my_robot_package publisher_node
```

### Terminal 2 - Subscriber Node:
```bash
ros2 run my_robot_package subscriber_node
```

## Creating New Nodes

To add a new Python node:

1. Create a new Python file in `src/my_robot_package/my_robot_package/your_new_node.py`
2. Add the entry point in `setup.py` under `console_scripts`
3. Rebuild the workspace with `colcon build`

## Useful Commands

- **List all nodes**: `ros2 node list`
- **List all topics**: `ros2 topic list`
- **Echo a topic**: `ros2 topic echo /robot_topic`
- **Check node info**: `ros2 node info /publisher_node`
- **Build specific package**: `colcon build --packages-select my_robot_package`
