# Surface Parameterization ROS 2 Package

This package provides **conformal surface parameterization** for robotic applications following the approach from Amersdorfer et al. (2021). It implements equidistant path planning on curved freeform surfaces.

## Reference

**Amersdorfer, M., Meurer, T., & Glück, T. (2021)**  
*"Equidistant Tool Path and Cartesian Trajectory Planning for Robotic Machining of Curved Freeform Surfaces"*  
IEEE Transactions on Automation Science and Engineering  
DOI: 10.1109/TASE.2021.3117691

## Features

- **Conformal parameterization** with metric tensor computation
- Maps 3D Cartesian points (x,y,z) to 2D parameter space (u,v) 
- **Preserves distances** through surface metric correction
- Enables **equidistant path planning** on curved surfaces
- Local frame computation using PCA
- Inverse interpolation from (u,v) → (x,y,z)
- Quality metrics evaluation (isotropy, orthogonality, reconstruction error)

## Prerequisites

Install required Python packages:
```bash
pip install numpy scipy scikit-learn
```

## Building

```bash
cd ~/Vattenfall_ws
colcon build --packages-select parameterization
source install/setup.bash
```

## Running the Node

```bash
ros2 run parameterization parameterization_node
```

### Parameters

- `interpolation_method` (string, default: 'rbf'): Interpolation method for UV → XYZ
- `neighbors` (int, default: 50): Number of neighbors for local RBF interpolation
- `metric_neighbors` (int, default: 20): Number of neighbors for metric tensor computation
- `quality_sample_size` (int, default: 1000): Sample size for quality evaluation
- `status_publish_rate` (float, default: 1.0): Rate to publish status (Hz)
- `conformal_iterations` (int, default: 5): Number of conformal correction iterations
- `conformal_alpha` (float, default: 0.5): Conformal correction step size (0 < α < 1)

Example with custom parameters:
```bash
ros2 run parameterization parameterization_node --ros-args \
  -p neighbors:=100 \
  -p metric_neighbors:=30 \
  -p conformal_iterations:=10 \
  -p conformal_alpha:=0.3
```

## Topics

### Subscribers

- `/point_cloud` (sensor_msgs/PointCloud2): Input point cloud

### Publishers

- `/parameterization/status` (ParameterizationStatus): Status and quality metrics
  - `is_ready`: Whether parameterization is ready
  - `num_points`: Number of points in the cloud
  - `mean_error`, `max_error`, `rmse`, `std_error`: Reconstruction quality metrics
  - Additional conformal metrics (isotropy, orthogonality) in extended status

## Services

### `/parameterization/interpolate`

Interpolate 3D Cartesian coordinates from (u,v) parameter space.

**Request:**
```
parameterization/UVPoint[] uv_points
```

**Response:**
```
geometry_msgs/Point[] points
bool success
string message
```

**Example:**
```bash
ros2 service call /parameterization/interpolate parameterization/srv/InterpolatePoint \
  "{uv_points: [{u: 0.0, v: 0.0}, {u: 1.0, v: 1.0}]}"
```

### `/parameterization/get_uv_bounds`

Get the bounds of the parameter space.

**Request:**
```
(empty)
```

**Response:**
```
float64 u_min
float64 u_max
float64 v_min
float64 v_max
bool success
string message
```

**Example:**
```bash
ros2 service call /parameterization/get_uv_bounds parameterization/srv/GetUVBounds
```

## Python API

You can also use the conformal parameterization module directly in your Python code:

```python
from parameterization.conformal_parameterization import ConformalParameterization
import numpy as np

# Create instance
surf = ConformalParameterization()

# Set points
points = np.array([[x1, y1, z1], [x2, y2, z2], ...])
surf.set_points(points)

# Compute local frame
surf.compute_local_frame()

# Compute conformal parameterization (Amersdorfer et al. 2021)
surf.compute_initial_parameterization(method='projection')
surf.compute_surface_metric(k_neighbors=20)
surf.apply_conformal_correction(iterations=5, alpha=0.5)

# Build interpolation
surf.build_inverse_interpolation(method='rbf', neighbors=50)

# Interpolate
uv_query = np.array([[u1, v1], [u2, v2]])
xyz = surf.interpolate(uv_query)

# Get UV spacing for equidistant paths
spacing_uv = surf.compute_equidistant_uv_spacing(
    desired_spacing=0.05,  # 5cm on surface
    uv_direction='u'
)

# Get bounds
bounds = surf.get_uv_bounds()
print(f"U: [{bounds['u_min']}, {bounds['u_max']}]")
print(f"V: [{bounds['v_min']}, {bounds['v_max']}]")

# Evaluate quality
metrics = surf.evaluate_quality(sample_size=1000)
print(f"RMSE: {metrics['rmse']}")
print(f"Isotropy error: {metrics['mean_isotropy_error']}")
print(f"Orthogonality error: {metrics['mean_orthogonality_error']}")
```

## Integration Example

Example of using this node in a larger ROS system:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from parameterization.srv import InterpolatePoint
from parameterization.msg import UVPoint

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        
        # Create service client
        self.interpolate_client = self.create_client(
            InterpolatePoint,
            '/parameterization/interpolate'
        )
        
    def get_waypoint(self, u, v):
        """Get a waypoint at (u,v)"""
        # Create UV point
        uv_pt = UVPoint()
        uv_pt.u = u
        uv_pt.v = v
        
        # Get 3D position
        req = InterpolatePoint.Request()
        req.uv_points = [uv_pt]
        
        future = self.interpolate_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            position = future.result().points[0]
            return position
        
        return None
```

## Message Definitions

### ParameterizationStatus.msg
```
std_msgs/Header header
bool is_ready
int32 num_points
float64 mean_error
float64 max_error
float64 rmse
float64 std_error
```

### UVPoint.msg
```
float64 u
float64 v
```

## Workflow

1. Node receives a point cloud via `/corrosion/scatter_plot_pub` topic
2. Computes local coordinate frame using PCA
3. Computes initial UV parameterization via projection
4. **Computes surface metric tensor (E, F, G) - key for equidistant paths**
5. **Applies conformal correction to minimize distortion**
6. Builds inverse interpolation using RBF with local neighbors
7. Publishes status with quality metrics including isotropy and orthogonality
8. Other nodes can query interpolation and UV bounds via services
9. Path planner uses metric-aware spacing for equidistant paths

## Key Concepts

### Surface Metric Tensor
The metric tensor describes how distances in UV space relate to distances on the surface:
- **E = ||∂r/∂u||²** - scale factor in u-direction
- **F = <∂r/∂u, ∂r/∂v>** - cross term (should be ≈0)
- **G = ||∂r/∂v||²** - scale factor in v-direction

For equidistant paths: `ds² = E du² + 2F du dv + G dv²`

### Conformal Correction
Iteratively adjusts UV coordinates to make:
- **E ≈ G** (isotropy) - uniform scaling in all directions
- **F ≈ 0** (orthogonality) - perpendicular parameter lines

This ensures UV spacing corresponds to surface spacing.

## Quality Metrics

- **Reconstruction RMSE**: How well (u,v) → (x,y,z) works (target: < 1mm)
- **Isotropy error**: `|E - G| / (E + G)` (target: < 0.1)
- **Orthogonality error**: `|F| / √(EG)` (target: < 0.1)
- **Scale factors**: `√E` and `√G` (target: ≈ 1.0)

Good metrics mean equidistant paths in UV space = equidistant paths on the surface!

## Notes

- Uses **conformal parameterization** following Amersdorfer et al. (2021)
- Computes surface metric tensor for distance preservation
- Enables **equidistant path planning** on curved surfaces
- Quality metrics include isotropy and orthogonality
- Especially effective for curved surfaces (cylinders, complex shapes)

## License

Apache-2.0
