# Path Following Guide

This guide explains how to visualize and execute a scanning path with the UR3e robot.

## 🚀 Quick Start

### Step 1: Launch the Robot with MoveIt (Terminal 1)
```bash
./launch_ur_moveit.sh
```
This will:
- Start the UR3e robot with the test setup
- Launch MoveIt motion planning with smooth 500Hz execution
- Publish the collision matrix (allows camera movement without errors)
- Open two RViz windows with the path visualization markers pre-configured

**Wait for both RViz windows to fully load.**

---

### Step 2: Import the Scanning Path (Terminal 2)
```bash
./import_line.sh
```
This will:
- Start the path follower node (visualizes paths in RViz)
- Generate and publish a scanning path along the curved test plate
- Continuously visualize the path with:
  - **Green line** showing the scanning trajectory
  - **Colored arrows** (red→blue) showing end effector orientations at each waypoint

**You should now see the path visualization in RViz automatically!**

---

### Step 3: Execute the Path (Terminal 3)
```bash
./follow_path.sh
```
This will:
- Compute the Cartesian path with MoveIt
- Validate that all waypoints are reachable
- Execute the robot motion following the path with correct orientations

**Important:** The robot must be in a safe starting position before executing the path.

---

## 📊 What You'll See

### In RViz:
1. **Complete test setup**: table, mount, robot, curved test plate, camera holder
2. **Green line**: The scanning path trajectory
3. **Red to blue arrows**: End effector orientations at each waypoint
4. **MoveIt planning panel**: For manual robot control

### Path Details:
- **15 waypoints** along the curved test plate
- **30cm horizontal sweep** (-0.15m to +0.15m in X)
- **15cm standoff distance** from the test plate surface
- **Parabolic Z-curve** following the test plate shape
- **Orientations**: Camera pointing toward the surface throughout the scan

---

## 🛠️ Customizing the Path

To modify the scanning path, edit:
```bash
src/ur3e_workstation/scripts/testplate_path_publisher.py
```

Key parameters you can adjust:
- `num_points`: Number of waypoints (default: 15)
- `standoff_distance`: Distance from surface (default: 0.15m)
- `x` range: Horizontal sweep distance
- `z` curve: Height variation following the curve
- `euler_angles`: Camera orientation at each waypoint

After editing, rebuild:
```bash
colcon build --symlink-install --packages-select ur3e_workstation
```

---

## 🔧 Troubleshooting

### Path visualization not showing in RViz?
- The visualization markers are now pre-configured in the RViz config
- If you still don't see them, manually add:
  - `/path_visualization` → MarkerArray
  - `/planned_path_line` → Marker

### Path execution fails?
- Check that the robot is in a safe starting position
- Verify all waypoints are within robot reach
- Look at the terminal output for specific error messages
- The path might be adjusted based on robot capabilities

### "No path loaded" error?
- Make sure `import_line.sh` is running before calling `follow_path.sh`
- Check that the path follower node is active

---

## 📡 ROS Topics

- `/scanning_path` - Raw path data (rotation matrices + XYZ positions)
- `/path_visualization` - Marker array with orientation arrows
- `/planned_path_line` - Green line strip showing the path

## 🔌 ROS Services

- `/execute_cartesian_path` - Execute the loaded scanning path

## 🎯 Workflow Summary

```
Terminal 1: ./launch_ur_moveit.sh    (Start robot + MoveIt)
              ↓
Terminal 2: ./import_line.sh         (Load & visualize path)
              ↓
Terminal 3: ./follow_path.sh         (Execute the path)
```

---

## 📝 Notes

- The path orientations are automatically calculated to keep the camera pointing at the test plate surface
- Each waypoint includes a full 3x3 rotation matrix for precise orientation control
- The Cartesian path execution ensures smooth motion between waypoints
- Collision checking is active during path execution to ensure safety
