# Surface Parameterization ROS 2 Package

This package provides surface parameterization for robotic applications using inverse interpolation. It maps 3D Cartesian points (x,y,z) to 2D parameter space (u,v) and provides interpolation services.

## Features

- Receives point clouds and computes surface parameterization
- Local frame computation using PCA (not normalized)
- UV parameterization using projection method
- Inverse interpolation from (u,v) → (x,y,z)
- Surface normal computation
- Quality metrics evaluation

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

- `interpolation_method` (string, default: 'rbf'): Interpolation method
- `neighbors` (int, default: 50): Number of neighbors for local RBF
- `normalize` (bool, default: false): Normalize UV to [0,1] range
- `quality_sample_size` (int, default: 1000): Sample size for quality evaluation
- `status_publish_rate` (float, default: 1.0): Rate to publish status (Hz)

Example with custom parameters:
```bash
ros2 run parameterization parameterization_node --ros-args \
  -p neighbors:=100 \
  -p quality_sample_size:=500
```

## Topics

### Subscribers

- `/point_cloud` (sensor_msgs/PointCloud2): Input point cloud

### Publishers

- `/parameterization/status` (ParameterizationStatus): Status and quality metrics
  - `is_ready`: Whether parameterization is ready
  - `num_points`: Number of points in the cloud
  - `mean_error`, `max_error`, `rmse`, `std_error`: Quality metrics

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

### `/parameterization/get_normal`

Get surface normals at (u,v) coordinates.

**Request:**
```
parameterization/UVPoint[] uv_points
```

**Response:**
```
geometry_msgs/Vector3[] normals
bool success
string message
```

**Example:**
```bash
ros2 service call /parameterization/get_normal parameterization/srv/GetSurfaceNormal \
  "{uv_points: [{u: 0.5, v: 0.5}]}"
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

You can also use the parameterization module directly in your Python code:

```python
from parameterization.surface_parameterization import SurfaceParameterization
import numpy as np

# Create instance
surf = SurfaceParameterization()

# Set points
points = np.array([[x1, y1, z1], [x2, y2, z2], ...])
surf.set_points(points)

# Compute local frame
surf.compute_local_frame()

# Compute UV parameterization
surf.compute_uv_parameterization(method='projection', normalize=False)

# Build interpolation
surf.build_inverse_interpolation(method='rbf', neighbors=50)

# Interpolate
uv_query = np.array([[u1, v1], [u2, v2]])
xyz = surf.interpolate(uv_query)

# Get normals
normals = surf.compute_surface_normals(uv_query)

# Get bounds
bounds = surf.get_uv_bounds()
print(f"U: [{bounds['u_min']}, {bounds['u_max']}]")
print(f"V: [{bounds['v_min']}, {bounds['v_max']}]")

# Evaluate quality
metrics = surf.evaluate_quality(sample_size=1000)
print(f"RMSE: {metrics['rmse']}")
```

## Integration Example

Example of using this node in a larger ROS system:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from parameterization.srv import InterpolatePoint, GetSurfaceNormal
from parameterization.msg import UVPoint

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        
        # Create service clients
        self.interpolate_client = self.create_client(
            InterpolatePoint,
            '/parameterization/interpolate'
        )
        
        self.normal_client = self.create_client(
            GetSurfaceNormal,
            '/parameterization/get_normal'
        )
        
    def get_waypoint(self, u, v):
        """Get a waypoint at (u,v) with surface normal"""
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
            
            # Get surface normal
            req = GetSurfaceNormal.Request()
            req.uv_points = [uv_pt]
            
            future = self.normal_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result().success:
                normal = future.result().normals[0]
                return position, normal
        
        return None, None
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

1. Node receives a point cloud via `/point_cloud` topic
2. Computes local coordinate frame using PCA (not normalized)
3. Computes UV parameterization using projection method
4. Builds inverse interpolation using RBF with local neighbors
5. Publishes status with quality metrics
6. Other nodes can query interpolation and normals via services

## Notes

- The local frame is computed using PCA but is **not normalized**, preserving actual scale
- UV parameterization uses the projection method by default
- Interpolation uses local RBF with 50 neighbors by default for efficiency
- Quality metrics are computed on a sample of points to avoid performance issues with large clouds

## License

Apache-2.0
