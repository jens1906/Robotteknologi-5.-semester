# Quick Start Guide - UR3e MoveIt

## Step-by-Step: Launch and Control the Robot

### Step 1: Launch the System

Open a terminal and run:

```bash
cd ~/Documents/GitHub/Robotteknologi-5.-semester/TestSite/Moveitws
./launch_ur3e.sh
```

This will open **3 windows**:
1. **RViz2** - The 3D visualization with the robot
2. **Joint State Publisher GUI** - A window with sliders for each joint
3. **Terminal** - Shows MoveIt status messages

### Step 2: Move the Robot with Sliders (Easy Way)

In the **Joint State Publisher GUI** window:
- You'll see 6 sliders, one for each joint
- Drag any slider left or right
- **The robot moves immediately in RViz!**

Try moving these:
- `shoulder_pan_joint` - Rotates the base
- `elbow_joint` - Bends the elbow
- `wrist_3_joint` - Rotates the wrist

### Step 3: Use MoveIt Motion Planning (Advanced Way)

In **RViz2**:

1. **Enable Interactive Markers**:
   - In the MotionPlanning panel (left side), find "Query Goal State"
   - Check the box or click "Update"
   - You should see a **colored ball/sphere** appear at the robot's end effector

2. **Move the Goal**:
   - Click and drag the **colored axes/sphere** to move the goal position
   - The robot will show a "ghost" image at the goal

3. **Plan a Path**:
   - In the MotionPlanning panel, click **"Plan"** button
   - MoveIt will compute a collision-free path
   - You'll see the planned trajectory animated

4. **Execute the Motion**:
   - Click **"Execute"** button
   - The robot will move along the planned path!

## Controls Summary

### Joint State Publisher GUI
- **Immediate control** of each joint
- Best for: Quick testing, manual positioning
- Each slider = one joint angle

### RViz2 MotionPlanning Panel
- **Plan & Execute** for complex motions
- Best for: Collision avoidance, smooth trajectories
- Uses MoveIt's motion planning

## Common Issues

### ❌ "No Interactive Markers visible"
**Solution**: 
- Go to MotionPlanning panel → Planning tab
- Click "Update" under Query Goal State
- Or restart RViz

### ❌ "Plan button is grayed out"
**Solution**:
- Make sure "Planning Group" is set to `ur_manipulator`
- Check that robot model is visible in RViz

### ❌ "Nothing happens when I click Execute"
**Solution**:
- This is simulation only - robot should move in RViz
- Check terminal for error messages

## What You Should See

When launched successfully:

```
✓ RViz2 shows a blue/gray UR3e robot
✓ Joint State Publisher has 6 working sliders  
✓ Moving sliders immediately moves robot in RViz
✓ MotionPlanning panel shows "Planning succeeded" when planning
```

## Quick Test

1. Launch the system: `./launch_ur3e.sh`
2. In Joint State Publisher GUI, move `shoulder_pan_joint` slider all the way left, then right
3. You should see the robot base rotate in RViz! ✓

## Next: Control from Python

Want to control the robot with code instead of GUI? See the Python example in the main README.md
