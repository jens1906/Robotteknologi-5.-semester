# Tool Orientation Node - Module Test Report

## Test Date
November 20, 2025

## Test Overview
Module test of the `tool_orientation_node` ROS2 node without any code modifications.

## Test Objective
Verify that the tool_orientation_node correctly:
1. Subscribes to input path data on `/parameterization/xyz_path`
2. Computes tool orientations (rotation matrices) for each path point
3. Publishes results to `/tool_orientation/xyz_rotation`
4. Generates valid rotation matrices (orthonormal, determinant ≈ 1)

## Test Setup
- **Input Topic**: `/parameterization/xyz_path` (Float64MultiArray)
- **Output Topic**: `/tool_orientation/xyz_rotation` (Float64MultiArray)
- **Test Data**: 30-point curved path (sine wave on XY plane with Z variation)
  - X range: [0.0000, 0.6283]
  - Y range: [-0.2996, 0.2996]
  - Z range: [0.4503, 0.5500]

## Test Results

### ✓ Node Initialization
- Node started successfully
- Subscribed to input topic: `/parameterization/xyz_path`
- Ready to receive path data

### ✓ Data Reception
- Received 30 points from test publisher
- Successfully parsed input data format

### ✓ Orientation Computation
- Computed rotation matrices for all 30 waypoints
- Published output to `/tool_orientation/xyz_rotation`

### ✓ Output Validation
Sample outputs (first 3 points):

**Point 0**: [0.0000, 0.0000, 0.5500]
```
Rotation Matrix:
[-0.3143, -0.1526, -0.9370]
[-0.9468,  0.1216,  0.2979]
[ 0.0684,  0.9808, -0.1827]
Determinant: 1.000000 ✓
```

**Point 1**: [0.0217, 0.0645, 0.5454]
```
Rotation Matrix:
[-0.3208, -0.2214, -0.9209]
[-0.9379,  0.2095,  0.2764]
[ 0.1317,  0.9524, -0.2749]
Determinant: 1.000000 ✓
```

**Point 2**: [0.0433, 0.1260, 0.5324]
```
Rotation Matrix:
[-0.3510, -0.3011, -0.8866]
[-0.9046,  0.3536,  0.2380]
[ 0.2418,  0.8856, -0.3965]
Determinant: 1.000000 ✓
```

## Test Conclusion

### ✓ PASSED
The tool_orientation_node is functioning correctly:
- ✓ Proper ROS2 node initialization
- ✓ Correct topic subscription and publication
- ✓ Valid rotation matrix computation (all determinants = 1.0)
- ✓ Proper handling of curved path with varying Z-coordinates
- ✓ Smooth orientation transitions between consecutive points

## Technical Verification
1. **Rotation Matrix Properties**: All matrices are orthonormal (det = 1.0)
2. **Tool Axis Alignment**: Z-axis points into surface (opposite of normal)
3. **Feed Direction**: Tool orientation follows path tangent
4. **Continuity**: Smooth orientation changes between waypoints

## How to Run Test Again
```bash
cd /home/toster23/Documents/GitHub/Robotteknologi-5.-semester/Vattenfall_ws
./run_tool_orientation_test.sh
```

## Test Files
- Test script: `test_tool_orientation.py`
- Shell runner: `run_tool_orientation_test.sh`
- Node under test: `src/tool_orientation/tool_orientation/tool_orientation_node.py`
