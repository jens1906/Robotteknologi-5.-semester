# Migration to Amersdorfer Approach (Conformal Parameterization Only)

## Summary of Changes

The parameterization package now **exclusively uses the Amersdorfer et al. (2021) conformal parameterization approach**. The simple PCA projection method has been removed in favor of the more accurate metric-aware implementation.

## What Changed

### Removed
- ❌ `parameterization_type` parameter (no longer needed)
- ❌ `normalize` parameter (not applicable to conformal approach)
- ❌ `SurfaceParameterization` class (replaced by `ConformalParameterization`)
- ❌ Simple grid-based zigzag path planning
- ❌ `path_type` parameter from path planner

### Added
- ✅ Always uses `ConformalParameterization`
- ✅ Surface metric tensor computation
- ✅ Conformal correction iterations
- ✅ `metric_neighbors` parameter for metric computation
- ✅ Isotropy and orthogonality quality metrics
- ✅ Equidistant iso-parametric path planning only

## Launch File Changes

### Before (Old)
```bash
ros2 launch parameterization parameterization.launch.py \
  parameterization_type:=simple \
  normalize:=false
```

### After (New)
```bash
ros2 launch parameterization parameterization.launch.py
```

That's it! The conformal approach is now the default and only option.

### Optional: Custom Parameters
```bash
ros2 launch parameterization parameterization.launch.py \
  conformal_iterations:=10 \
  metric_neighbors:=30 \
  neighbors:=50
```

## Path Planning Changes

### Before (Old)
```bash
ros2 run path_planning path_planning_node --ros-args \
  -p path_type:=simple \
  -p d:=0.05
```

### After (New)
```bash
ros2 run path_planning path_planning_node --ros-args \
  -p spacing:=0.05 \
  -p iso_direction:=u
```

The path planner now always generates iso-parametric paths.

## Code Changes

### Before (Old Python API)
```python
from parameterization.surface_parameterization import SurfaceParameterization

surf = SurfaceParameterization()
surf.set_points(points)
surf.compute_local_frame()
surf.compute_uv_parameterization(method='projection', normalize=False)
surf.build_inverse_interpolation(method='rbf', neighbors=50)
```

### After (New Python API)
```python
from parameterization.conformal_parameterization import ConformalParameterization

surf = ConformalParameterization()
surf.set_points(points)
surf.compute_local_frame()
surf.compute_initial_parameterization(method='projection')
surf.compute_surface_metric(k_neighbors=20)
surf.apply_conformal_correction(iterations=5, alpha=0.5)
surf.build_inverse_interpolation(method='rbf', neighbors=50)
```

## New Features Available

### 1. Surface Metric Computation
```python
# Get surface metric tensor
metrics = surf.compute_surface_metric(k_neighbors=20)
# Returns E, F, G for each point
```

### 2. Equidistant UV Spacing
```python
# Calculate UV spacing for desired surface distance
spacing_uv = surf.compute_equidistant_uv_spacing(
    desired_spacing=0.05,  # 5cm on surface
    uv_direction='u'
)
```

### 3. Surface Distance Calculation
```python
# Get actual surface distance between UV points
distance = surf.get_surface_distance(uv1, uv2)
```

### 4. Enhanced Quality Metrics
```python
metrics = surf.evaluate_quality(sample_size=1000)
# Now includes:
# - mean_isotropy_error (how uniform the parameterization is)
# - mean_orthogonality_error (how perpendicular u/v are)
# - mean_scale_u, mean_scale_v (scale factors)
```

## Benefits of the Change

### ✅ Improved Accuracy
- True equidistant spacing on curved surfaces
- Distance preservation through metric correction
- Better path quality for machining/coating

### ✅ Follows Industry Standards
- Implements peer-reviewed approach (Amersdorfer et al. 2021)
- Published in IEEE TASE (top-tier journal)
- Validated for robotic machining

### ✅ Better Quality Metrics
- Isotropy and orthogonality measurements
- Clear targets for good parameterization
- Helps diagnose issues with curved surfaces

## Tuning Parameters

### For Better Quality (Slower)
```yaml
conformal_iterations: 10      # More iterations
conformal_alpha: 0.3          # Smaller steps
metric_neighbors: 30          # More neighbors
```

### For Faster Processing (Lower Quality)
```yaml
conformal_iterations: 3       # Fewer iterations
conformal_alpha: 0.7          # Larger steps
metric_neighbors: 10          # Fewer neighbors
```

### Recommended (Balanced)
```yaml
conformal_iterations: 5       # Default
conformal_alpha: 0.5          # Default
metric_neighbors: 20          # Default
```

## Backward Compatibility

### API Compatibility
The old `SurfaceParameterization` class is **no longer available**. All code must be updated to use `ConformalParameterization`.

### Data Compatibility
- UV parameter format is the same (Nx2 array)
- XYZ point format is the same (Nx3 array)
- Service interfaces unchanged
- Topic messages unchanged

### Migration Script
If you have existing code using the old API:

1. Replace import:
   ```python
   # Old
   from parameterization.surface_parameterization import SurfaceParameterization
   
   # New
   from parameterization.conformal_parameterization import ConformalParameterization
   ```

2. Update parameterization calls:
   ```python
   # Old
   surf.compute_uv_parameterization(method='projection', normalize=False)
   
   # New
   surf.compute_initial_parameterization(method='projection')
   surf.compute_surface_metric(k_neighbors=20)
   surf.apply_conformal_correction(iterations=5, alpha=0.5)
   ```

3. Update class instantiation:
   ```python
   # Old
   surf = SurfaceParameterization()
   
   # New
   surf = ConformalParameterization()
   ```

## Testing

After migration, verify:

1. **Check quality metrics**:
   - Isotropy error < 0.1
   - Orthogonality error < 0.1
   - RMSE similar or better than before

2. **Verify path spacing**:
   - Paths should be more uniform on curved surfaces
   - Check spacing variation (should be < 15%)

3. **Performance**:
   - Slightly slower due to metric computation
   - Still real-time capable for typical point clouds

## Questions?

- See `AMERSDORFER_IMPLEMENTATION.md` for technical details
- See `QUICK_START_AMERSDORFER.md` for quick reference
- See `VISUAL_GUIDE.md` for visual explanations
- Check the updated `README.md` for API documentation

---

**Note:** The old simple projection approach is still available in the `surface_parameterization.py` file for reference, but it is not integrated into the ROS node anymore.
