# Path Planning Migration Guide

## Summary of Changes

The path planning node has been migrated to work with the **Amersdorfer et al. (2021) conformal parameterization approach**. The simple zigzag path planning has been replaced with **iso-parametric equidistant path planning** that uses surface metrics for accurate distance control.

## What Changed

### Removed
- ❌ `d` parameter (replaced by `spacing`)
- ❌ `line_n` parameter (now calculated automatically)
- ❌ `bezier_curviture` parameter (replaced by `bezier_curvature_factor`)
- ❌ Simple grid-based UV spacing calculation

### Added
- ✅ `spacing` parameter: Desired spacing on surface in meters (default: 0.05)
- ✅ `iso_direction` parameter: 'u' or 'v' for path direction (default: 'u')
- ✅ `bezier_curvature_factor` parameter: Curvature control (default: 0.5)
- ✅ Service client to get UV bounds from parameterization
- ✅ Automatic calculation of line count based on surface spacing
- ✅ UV spacing computation for equidistant surface paths

## Launch/Run Changes

### Before (Old)
```bash
ros2 run path_planning path_planning_node
# Used fixed parameters: d=0.05, line_n=10
```

### After (New)
```bash
ros2 run path_planning path_planning_node --ros-args \
  -p spacing:=0.05 \
  -p iso_direction:=u
```

### Common Use Cases

**5cm spacing in U direction (coating):**
```bash
ros2 run path_planning path_planning_node --ros-args \
  -p spacing:=0.05 \
  -p iso_direction:=u
```

**10cm spacing in V direction (scanning):**
```bash
ros2 run path_planning path_planning_node --ros-args \
  -p spacing:=0.10 \
  -p iso_direction:=v
```

**Fine spacing with smoother transitions:**
```bash
ros2 run path_planning path_planning_node --ros-args \
  -p spacing:=0.03 \
  -p iso_direction:=u \
  -p bezier_curvature_factor:=0.3
```

## Parameter Reference

### Required Parameters
- **spacing** (float, default: 0.05)
  - Desired spacing on the surface in meters
  - Example: 0.05 = 5cm between paths
  - Smaller values = more paths, better coverage
  - Larger values = fewer paths, faster execution

### Optional Parameters
- **iso_direction** (string, default: 'u')
  - Direction of iso-parametric paths
  - 'u': Constant-u lines (paths vary in v direction)
  - 'v': Constant-v lines (paths vary in u direction)
  - Choose based on surface geometry and tool orientation

- **n_bezier** (int, default: 50)
  - Number of points in Bézier transition curves
  - Higher values = smoother transitions
  - Lower values = faster computation

- **bezier_curvature_factor** (float, default: 0.5)
  - Controls the curvature of Bézier transitions
  - Range: 0.0 to 1.0
  - Lower values = tighter turns
  - Higher values = wider, smoother turns

## Code Changes

### Before (Old Internal Logic)
```python
# Fixed number of lines
self.line_n = 10
self.d = 0.05

# Simple UV grid
u_lin = np.linspace(u_min, u_max, self.line_n)
v_lin = np.linspace(v_min, v_max, self.line_n)
```

### After (New Internal Logic)
```python
# Dynamic line count based on surface spacing
spacing_uv = self.compute_uv_spacing()
n_lines = int(np.ceil((u_max - u_min) / spacing_uv)) + 1

# Equidistant iso-parametric lines
u_values = np.linspace(u_min, u_max, n_lines)
```

## New Features

### 1. Service Integration
The node now communicates with the parameterization service to get accurate UV bounds:
```python
self.bounds_client = self.create_client(GetUVBounds, '/parameterization/get_uv_bounds')
```

### 2. Automatic Line Count Calculation
Number of lines is calculated automatically based on:
- Desired surface spacing
- UV parameter range
- Surface metrics (approximated)

### 3. Iso-parametric Path Generation
Generates either:
- **Constant-u lines** (iso_direction='u'): Path follows u=constant, varies v
- **Constant-v lines** (iso_direction='v'): Path follows v=constant, varies u

### 4. Enhanced Bézier Smoothing
- Curvature is now based on surface spacing
- Better control through `bezier_curvature_factor`
- Improved direction vector calculation

## Benefits of Migration

### ✅ Consistent Surface Coverage
- Spacing is now in real surface distance (meters)
- More predictable path density
- Better for coating and machining applications

### ✅ Adaptive Path Count
- Automatically adjusts number of paths to surface size
- No more manual tuning of `line_n`
- Scales properly with different surfaces

### ✅ Integration with Conformal Parameterization
- Works seamlessly with the new parameterization approach
- Uses UV bounds from parameterization service
- Ready for future metric-based spacing improvements

### ✅ Clearer Parameters
- `spacing` is intuitive (5cm = 0.05m)
- `iso_direction` clearly defines path orientation
- Less confusion than old `d` and `line_n` parameters

## Migration Checklist

- [x] Update path planning node code
- [x] Add new parameters (spacing, iso_direction)
- [x] Add service client for UV bounds
- [x] Replace fixed line count with automatic calculation
- [x] Update Bézier smoothing logic
- [x] Add better error handling and logging
- [ ] Update launch files (if any)
- [ ] Update documentation
- [ ] Test with real surfaces

## Backward Compatibility

### Breaking Changes
The old parameters **are not supported**:
- `d` → Use `spacing` instead
- `line_n` → Now calculated automatically
- `bezier_curviture` → Use `bezier_curvature_factor` instead

### Parameter Mapping
```yaml
# Old parameters → New parameters
d: 0.05              → spacing: 0.05
line_n: 10           → (automatic, no parameter needed)
bezier_curviture: 0.025  → bezier_curvature_factor: 0.5  # (0.025 / 0.05 = 0.5)
```

## Future Improvements

### Planned Enhancements
1. **Metric-based spacing service**: Call parameterization service to get exact UV spacing based on surface metric tensor
2. **Variable spacing**: Adapt spacing based on local surface curvature
3. **Offset paths**: Generate parallel offset paths for multi-layer coating
4. **Path optimization**: Minimize total path length and transitions

### Service Request Example (Future)
```python
# Future: Get UV spacing from parameterization service
request = ComputeEquidistantSpacing.Request()
request.desired_spacing = 0.05
request.uv_direction = 'u'
response = self.spacing_client.call(request)
spacing_uv = response.spacing_uv
```

## Testing

After migration, verify:

1. **Path count makes sense**:
   - Check the number of generated lines
   - Should be approximately: surface_width / spacing

2. **Path spacing is uniform**:
   - Paths should be evenly distributed in UV space
   - Visual inspection in RViz or similar tool

3. **Bézier transitions are smooth**:
   - No sharp corners between lines
   - Adjust `bezier_curvature_factor` if needed

4. **Surface coverage is complete**:
   - All areas should be covered
   - No large gaps between paths

## Troubleshooting

### Problem: Too many/few paths
**Solution**: Adjust the `spacing` parameter
```bash
# More paths (smaller spacing)
-p spacing:=0.03

# Fewer paths (larger spacing)
-p spacing:=0.10
```

### Problem: Sharp transitions between lines
**Solution**: Increase `bezier_curvature_factor` or `n_bezier`
```bash
-p bezier_curvature_factor:=0.7
-p n_bezier:=100
```

### Problem: Paths don't follow desired direction
**Solution**: Switch `iso_direction`
```bash
# Try the other direction
-p iso_direction:=v  # if you were using 'u'
```

### Problem: Service not available error
**Solution**: Make sure parameterization node is running first
```bash
# Terminal 1: Start parameterization
ros2 run parameterization parameterization_node

# Terminal 2: Start path planning
ros2 run path_planning path_planning_node
```

## Questions?

- See the updated README.md for API documentation
- Check AMERSDORFER_IMPLEMENTATION.md in parameterization package
- Review the migration guide in parameterization package
- Contact the development team

---

**Migration Date**: November 2025  
**Compatible with**: Parameterization package using Amersdorfer approach
