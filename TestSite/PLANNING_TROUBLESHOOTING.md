# MoveIt Planning Failure Troubleshooting

## Common Error: "Unable to sample any valid states for goal tree"

### What This Means
The motion planner cannot find a valid path because:
1. **Goal is in collision** - Target pose causes robot to collide with itself or environment
2. **Goal is unreachable** - Outside workspace or violates joint limits
3. **Start state is invalid** - Current position has issues

## Common Error: "Start state out of bounds"

### What This Means
**This is the most common issue with mock hardware!**

The robot's current joint angles are outside acceptable limits. This typically happens with:
- **Continuous joints** (wrist_3_joint, shoulder_pan_joint) accumulating rotations
- **Mock hardware** not normalizing angles to [-π, π]
- Rotating end effector repeatedly

### Symptoms:
```
[ERROR] PlanningRequestAdapter 'CheckStartStateBounds' failed
START_STATE_INVALID
```

### Why It Happens:
Mock hardware doesn't wrap continuous joint angles. After multiple rotations:
- Joint 6 might be at `12.5` rad instead of normalized `0.3` rad
- MoveIt rejects this as "out of bounds"
- Planning fails even though the position is geometrically valid

### Quick Fixes:

**1. Check Current Joint Values:**
```bash
ros2 topic echo /joint_states --once
```
Look for values > 6.28 or < -6.28 (outside ±2π).

**2. Reset to Home Position:**
```bash
ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
  points: [{
    positions: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
    time_from_start: {sec: 3}
  }]
}"
```

**3. Run Joint Normalizer (Automatic Fix):**
```bash
# In a separate terminal
cd /home/andr465m/Documents/GitHub/Robotteknologi-5.-semester/TestSite
python3 normalize_joint_angles.py
```

**4. Restart Both Driver and MoveIt:**
Most reliable solution - fresh start resets all joint values.

### Prevention:
- Don't rotate continuous joints excessively in mock mode
- Use "Plan only" instead of "Plan & Execute" to check before committing
- Restart simulation periodically during testing
- **Use real hardware** - it doesn't have this issue!

## How to Fix

### 1. Check for Collisions in RViz

**Before planning:**
- Look at the interactive marker (orange ball)
- Check if robot preview is RED = collision detected
- Move the goal to a position where robot is GREEN

**In your case:** The error shows `forearm_link` colliding with `rsd435` camera.

### 2. Adjust Planning Parameters

Edit your MoveIt config to be more permissive:

```yaml
# In ompl_planning.yaml or similar
planning_time: 10.0  # Give planner more time (default is 5s)
num_planning_attempts: 5  # Try multiple times
goal_joint_tolerance: 0.01  # More lenient (yours is 0.0001)
goal_position_tolerance: 0.005  # For Cartesian goals
goal_orientation_tolerance: 0.05
```

### 3. Disable Collision Checking Temporarily (Testing Only)

To verify if collision is the issue:

```bash
# In RViz, Motion Planning panel:
# Scene Objects tab -> Uncheck "Enable collision checking"
```

**WARNING:** Only for testing! Re-enable for real robot.

### 4. Check Joint Limits

Your robot might have custom joint limits that are too restrictive:

```bash
# Check current limits
ros2 param get /move_group robot_description_planning

# Look for ur3e joint limits in your URDF/SRDF
```

### 5. Use Different Planner

RRTConnect fails quickly if goal is invalid. Try:
- **RRTstar**: More thorough exploration
- **TRRT**: Better with tight spaces
- **PRM**: Good for complex environments

Change in RViz: Planning tab -> Algorithm dropdown

### 6. Increase Planning Time

```python
# In your Python scripts:
move_group.set_planning_time(15.0)  # seconds
move_group.set_num_planning_attempts(10)
```

### 7. Fix Model Mismatch Warning

You're also getting this warning constantly:
```
Setting the scene for model 'ur3e_workstation' but model 'ur3e' is loaded.
```

This suggests URDF/SRDF mismatch. Check your config files have consistent robot names.

## Diagnostic Commands

### Check current robot state:
```bash
ros2 topic echo /joint_states --once
```

### Check if goal is reachable (FK/IK):
```bash
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "..."
```

### Visualize collision geometry:
In RViz: Add -> TF, MarkerArray, etc.

## Quick Fixes for Your Specific Error

### Option 1: Move Camera in URDF
If `rsd435` camera is in the way, adjust its mounting position in your URDF:

```xml
<!-- Find the camera joint and adjust the xyz position -->
<joint name="camera_joint" type="fixed">
  <origin xyz="0.05 0.0 0.05" rpy="0 0 0"/>  <!-- Adjust these values -->
</joint>
```

### Option 2: Adjust Allowed Collision Matrix
Allow collision between forearm and camera if they should never actually touch:

```bash
# Add to your SRDF (ur3e.srdf):
<disable_collisions link1="forearm_link" link2="rsd435" reason="Never in contact"/>
```

### Option 3: Choose Safer Goals
When dragging the interactive marker:
1. Keep it away from the camera
2. Watch for color changes (green = safe, red = collision)
3. Start with simple, known-safe positions

## Testing Workflow

1. **Start with home position:**
   ```
   ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "..."
   ```

2. **Use named targets** (if configured):
   - "home"
   - "up"
   - etc.

3. **Gradually test more complex poses**

## Real Robot vs Simulation

- Mock hardware has no physical feedback, so state can drift
- Real robot enforces actual limits and collisions
- Always test carefully with real hardware

## Still Failing?

1. Restart both driver and MoveIt (state may be corrupted)
2. Check collision geometry is correct (view in RViz)
3. Verify camera/sensor positions match real setup
4. Consider using joint-space goals instead of Cartesian
5. Enable MoveIt debug logging:
   ```bash
   ros2 run move_group move_group --ros-args --log-level debug
   ```
