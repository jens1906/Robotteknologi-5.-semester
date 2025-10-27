# Notation Convention

This implementation uses the following notation convention:

## Coordinate Systems

### Parameter Space (2D)
- **x, y** = 2D parameter space coordinates
- Range: typically normalized to [0, 1] × [0, 1]
- This is the simplified 2D domain for path planning

### Cartesian Space (3D)
- **u, v, w** = 3D Cartesian world coordinates
- These are the actual robot workspace coordinates
- Represents the physical surface in 3D space

## Mapping Direction

The **inverse interpolation** maps from parameter space to Cartesian space:

```
(x, y) → (u, v, w)
```

Where:
- **Input**: Parameter space coordinates (x, y)
- **Output**: Cartesian space coordinates (u, v, w)

## Why This Convention?

This notation follows mathematical convention where:
- **x, y** are typically used for planar/parametric coordinates
- **u, v, w** represent spatial coordinates (similar to x, y, z)
- The mapping is "inverse" because we go from lower-dimensional (2D) to higher-dimensional (3D) space

## Usage in Code

```python
# Create parameterization
surf = SurfaceParameterization(point_cloud_path="point_cloud.ply")

# Compute parameter space mapping
surf.compute_xy_parameterization()  # Maps (u,v,w) → (x,y)

# Build inverse interpolation
surf.build_inverse_interpolation()  # Creates (x,y) → (u,v,w) mapping

# Query a point in parameter space
xy_query = np.array([[0.5, 0.5]])  # Center of parameter space
uvw_result = surf.interpolate(xy_query)  # Get Cartesian coordinates

print(f"Parameter (x,y): {xy_query[0]}")
print(f"Cartesian (u,v,w): {uvw_result[0]}")
```

## Class Attributes

- `self.xy_params` - N×2 array of parameter space coordinates (x, y)
- `self.points` - N×3 array of Cartesian space coordinates (u, v, w)
- `self.interpolator_u` - Interpolator for u coordinate
- `self.interpolator_v` - Interpolator for v coordinate
- `self.interpolator_w` - Interpolator for w coordinate

## Method Signatures

```python
def compute_xy_parameterization(self, method='projection'):
    """Maps Cartesian (u,v,w) → Parameter (x,y)"""
    
def build_inverse_interpolation(self, method='rbf', neighbors=None):
    """Creates interpolator for Parameter (x,y) → Cartesian (u,v,w)"""
    
def interpolate(self, xy_query):
    """Parameter (x,y) → Cartesian (u,v,w)"""
    
def compute_surface_normals(self, xy_query):
    """Normals at parameter space points (x,y)"""
    
def create_regular_grid(self, x_samples=50, y_samples=50):
    """Regular grid in parameter space (x,y)"""
```

## Visualization Labels

- **Parameter Space Plot**: x-axis = "x (parameter)", y-axis = "y (parameter)"
- **Cartesian Space Plot**: x-axis = "U", y-axis = "V", z-axis = "W"

## Comparison with Alternative Notation

| Our Notation | Alternative | Description |
|--------------|-------------|-------------|
| (x, y) | (u, v) | Parameter space |
| (u, v, w) | (x, y, z) | Cartesian space |

We chose this convention to clearly distinguish between:
- The **abstract parameter space** (x, y) used for path planning
- The **physical Cartesian space** (u, v, w) where the robot operates
