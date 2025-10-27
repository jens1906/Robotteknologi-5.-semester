# Surface Parameterization - Inverse Interpolation Approach

Implementation of surface parameterization based on the inverse interpolation approach described in IEEE TASE 2021 paper (dx.doi.org/10.1109/tase.2021.3117691).

## Overview

This implementation provides surface parameterization for 3D point clouds, mapping them to a 2D parametric domain (u,v) and enabling inverse mapping from (u,v) back to 3D space. This is particularly useful for robotic applications such as:

- Automated grinding and polishing
- Spray painting
- Surface inspection
- Path planning on complex surfaces

## Features

- **Point Cloud Loading**: Load 3D point clouds from PLY files
- **Coordinate Frame Alignment**: Automatic alignment using PCA
- **UV Parameterization**: Multiple methods (projection, distance-based)
- **Inverse Interpolation**: RBF and grid-based interpolation
- **Surface Normal Computation**: Calculate normals at any UV coordinate
- **Regular Grid Generation**: Create uniform grids for path planning
- **Quality Metrics**: Evaluate parameterization accuracy
- **Visualization**: 3D visualization of original and parameterized surface

## Installation

```bash
pip install -r requirements.txt
```

## Usage

### Basic Usage

```python
from surface_parameterization import SurfaceParameterization

# Load point cloud
surf_param = SurfaceParameterization(point_cloud_path="point_cloud.ply")

# Compute parameterization
surf_param.compute_local_frame()
surf_param.compute_uv_parameterization(method='projection')
surf_param.build_inverse_interpolation(method='rbf', neighbors=50)

# Evaluate quality
metrics = surf_param.evaluate_quality()

# Interpolate new points
uv_query = [[0.5, 0.5], [0.25, 0.75]]
xyz_points = surf_param.interpolate(uv_query)

# Get surface normals
normals = surf_param.compute_surface_normals(uv_query)

# Visualize
surf_param.visualize()
```

### Run the Complete Example

```bash
python surface_parameterization.py
```

This will:
1. Load the point cloud from `point_cloud.ply`
2. Compute the surface parameterization
3. Build inverse interpolation
4. Evaluate quality metrics
5. Test interpolation at sample points
6. Display visualization
7. Save a regular grid to `surface_grid_xyz.npy` and `surface_grid_uv.npy`

## Method Details

### UV Parameterization Methods

- **projection**: Simple projection onto principal plane (fast, good for nearly planar surfaces)
- **distance**: Cumulative distance-based parameterization
- **conformal**: Angle-preserving mapping (future extension)

### Inverse Interpolation Methods

- **rbf**: Radial Basis Function interpolation using thin-plate splines
  - `neighbors=None`: Global RBF (slower but more accurate)
  - `neighbors=N`: Local RBF with N nearest neighbors (faster)
- **linear**: Linear interpolation
- **cubic**: Cubic interpolation

## Output

The implementation provides:

- UV parameters for each point in the cloud
- Interpolation functions for mapping (u,v) → (x,y,z)
- Surface normals at any UV coordinate
- Regular grids suitable for robotic path planning
- Quality metrics (RMSE, mean/max error)

## Applications

### Robotic Path Planning

```python
# Generate a regular grid for robot paths
grid_xyz, grid_uv = surf_param.create_regular_grid(u_samples=50, v_samples=50)

# Generate paths in UV space (simple scanning pattern)
u_path = np.linspace(0, 1, 100)
v_path = np.linspace(0, 1, 100)

# Convert to 3D coordinates
path_3d = surf_param.interpolate(np.column_stack([u_path, v_path]))
```

### Surface Normal Extraction

```python
# Get normals for tool orientation
normals = surf_param.compute_surface_normals(grid_uv.reshape(-1, 2))
```

## Quality Metrics

The `evaluate_quality()` method computes:
- Mean reconstruction error
- Maximum reconstruction error
- Root Mean Square Error (RMSE)
- Standard deviation of errors

## Visualization

The `visualize()` method creates three plots:
1. 3D view of original points and interpolated surface
2. UV parameterization colored by Z coordinate
3. Height map showing (u,v,Z) relationship

## References

IEEE Transactions on Automation Science and Engineering (TASE), 2021
DOI: 10.1109/tase.2021.3117691

## Notes

- For large point clouds (>10,000 points), use local RBF interpolation (`neighbors=50-100`) for better performance
- The quality of parameterization depends on the surface shape; nearly planar or developable surfaces work best
- Adjust grid resolution based on your application requirements
