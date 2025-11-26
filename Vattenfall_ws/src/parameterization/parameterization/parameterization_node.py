#!/usr/bin/env python3
"""
Surface Parameterization ROS 2 Node
interpolation and surface normal computation services.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Float64MultiArray, Float32MultiArray, Bool
import numpy as np
from scipy.spatial import cKDTree, ConvexHull

from parameterization.msg import ParameterizationStatus, UVPoint, UVBoundary
from parameterization.srv import InterpolatePoint, GetUVBounds

# Import the parameterization module
from parameterization.surface_parameterization import Parameterization

class ParameterizationNode(Node):
    """
    ROS 2 Node for arc-length-based isometric surface parameterization.
    
    Implements isometric parameterization
    with cubic spline interpolation for smooth surface reconstruction.
    Uses distance-preserving arc-length parameterization for equidistant path planning.
    
    Subscribes to:
        - /corrosion/scatter_plot_pub (std_msgs/Float32MultiArray): Input XYZ points from corrosion detection
        
    Publishes:
        - /parameterization/status (ParameterizationStatus): Status and quality metrics
        
    Services:
        - /parameterization/interpolate (InterpolatePoint): Interpolate 3D points from (u,v)
        - /parameterization/get_uv_bounds (GetUVBounds): Get parameter space bounds
        
    Parameters:
        - quality_sample_size (int): Sample size for quality evaluation
        - status_publish_rate (float): Rate to publish status (Hz)
        - metric_neighbors (int): Number of neighbors for metric tensor computation (default: 20)
    """
    
    def __init__(self):
        super().__init__('parameterization_node')
        
        # Declare parameters
        self.declare_parameter('quality_sample_size', 1000)
        self.declare_parameter('status_publish_rate', 1.0)
        self.declare_parameter('metric_neighbors', 20)
        
        # Get parameters
        self.quality_sample_size = self.get_parameter('quality_sample_size').value
        status_rate = self.get_parameter('status_publish_rate').value
        self.metric_neighbors = self.get_parameter('metric_neighbors').value
        
        # Initialize arc-length-based parameterization
        self.surf = Parameterization()
        self.get_logger().info('Using Arc-Length-Based Isometric Parameterization')
        self.get_logger().info('Interpolation: Cubic Spline (CloughTocher2D)')

        # Storage for corrosion points and their UV bounds
        self.corrosion_points = None
        self.corrosion_uv_bounds = None
        self.corrosion_boundary_uv = None  # Boundary points in UV space
        self.pending_corrosion_data = None  # Store corrosion data if received before parameterization ready
        self.retry_timer = None  # Timer for retrying corrosion processing
        
        # Create subscriber for corrosion scatter plot
        self.corrosion_plot_sub = self.create_subscription(
            Float32MultiArray,
            '/corrosion/corrosion',
            self.scatter_plot_corrosion_callback,
            10
        )

        self.scatter_plot_sub = self.create_subscription(
            Float32MultiArray,
            '/corrosion/workspace',
            self.scatter_plot_callback,
            10
        )
        
        # Create publishers
        self.status_pub = self.create_publisher(
            ParameterizationStatus,
            '/parameterization/status',
            10
        )
        
        # Additional publishers for path_planning node (standard ROS messages)
        self.ready_pub = self.create_publisher(
            Bool,
            '/parameterization/ready',
            10
        )
        
        self.uv_bounds_pub = self.create_publisher(
            Float64MultiArray,
            '/parameterization/uv_bounds',
            10
        )
        
        self.uv_boundary_pub = self.create_publisher(
            Float64MultiArray,
            '/parameterization/uv_boundary',
            10
        )

        self.param_uv_pub = self.create_publisher(
            Float64MultiArray,
            '/parameterization/param_uv',
            10
        )

        self.xyz_path_pub = self.create_publisher(
            Float64MultiArray,
            '/parameterization/xyz_path',
            10
        )
        
        # Create services
        self.interpolate_srv = self.create_service(
            InterpolatePoint,
            '/parameterization/interpolate',
            self.interpolate_callback
        )
        
        self.bounds_srv = self.create_service(
            GetUVBounds,
            '/parameterization/get_uv_bounds',
            self.get_uv_bounds_callback
        )

        # Subscribe to UV path from path planner
        self.create_subscription(
            Float64MultiArray,
            '/path/uv_path',
            self.uv_path_callback,
            10
        )
        
        # Create timer for status publishing
        self.status_timer = self.create_timer(
            1.0 / status_rate,
            self.publish_status
        )
        
        self.get_logger().info('Parameterization node initialized')
        self.get_logger().info(f'  Parameterization: Arc-length-based (isometric)')
        self.get_logger().info(f'  Interpolation: Cubic splines (CloughTocher2D)')
        self.get_logger().info(f'  Metric neighbors: {self.metric_neighbors}')
    

    def scatter_plot_callback(self, msg):
        """
        Callback for scatter plot messages from corrosion detection.
        Processes the XYZ points and builds parameterization.
        """
        try:
            self.get_logger().info('Received scatter plot data, processing...')
            
            # Convert Float32MultiArray to numpy array
            # The data is expected to be flattened XYZ points: [x1, y1, z1, x2, y2, z2, ...]
            data = np.array(msg.data, dtype=np.float64)
            
            if len(data) == 0:
                self.get_logger().error('Received empty scatter plot data')
                return
            
            # Reshape to Nx3 array (each row is [x, y, z])
            if len(data) % 3 != 0:
                self.get_logger().error(f'Invalid data length: {len(data)} (must be multiple of 3)')
                return
            
            points = data.reshape(-1, 3)
            
            self.get_logger().info(f'Processing {len(points)} points')
            
            # Set points
            self.surf.set_points(points)
            
            # Compute local frame using PCA
            self.surf.compute_local_frame()
            self.get_logger().info('Local frame computed')
            
            # Arc-length-based UV parameterization (isometric)
            self.surf.compute_initial_parameterization()
            self.get_logger().info('Arc-length UV parameterization computed')
            
            # Compute surface metric tensor (E, F, G)
            #self.surf.compute_surface_metric(k_neighbors=self.metric_neighbors)
            #self.get_logger().info('Surface metric tensor computed')
            
            # Build cubic spline inverse interpolation
            self.surf.build_inverse_interpolation()
            self.get_logger().info('Cubic spline inverse interpolation built')
            
            # Evaluate quality
            metrics = self.surf.evaluate_quality(
                sample_size=self.quality_sample_size
            )
            
            self.get_logger().info('Parameterization complete')
            self.get_logger().info(f'  Mean error: {metrics["mean_error"]:.6f}')
            self.get_logger().info(f'  Max error: {metrics["max_error"]:.6f}')
            self.get_logger().info(f'  RMSE: {metrics["rmse"]:.6f}')
            
            # Get UV bounds
            bounds = self.surf.get_uv_bounds()
            self.get_logger().info(f'  UV bounds: U=[{bounds["u_min"]:.3f}, {bounds["u_max"]:.3f}], '
                                 f'V=[{bounds["v_min"]:.3f}, {bounds["v_max"]:.3f}]')
            
            # Publish UV parameters
            uv_msg = Float64MultiArray()
            uv_msg.data = self.surf.uv_params.flatten().tolist()
            self.param_uv_pub.publish(uv_msg)
            self.get_logger().info(f'Published UV parameters: {len(self.surf.uv_params)} points')
            
        except Exception as e:
            self.get_logger().error(f'Error processing scatter plot: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    

    def scatter_plot_corrosion_callback(self, msg):
        """
        Callback for corrosion scatter plot messages.
        Maps corrosion points to UV coordinates and computes their bounds.
        Retries if parameterization is not ready yet.
        """
        try:
            self.get_logger().info('Received corrosion scatter plot data')
            
            # Convert Float32MultiArray to numpy array
            data = np.array(msg.data, dtype=np.float64)
            
            if len(data) == 0:
                self.get_logger().warn('Received empty corrosion data')
                return
            
            # Reshape to Nx3 array (each row is [x, y, z])
            if len(data) % 3 != 0:
                self.get_logger().error(f'Invalid corrosion data length: {len(data)} (must be multiple of 3)')
                return
            
            corrosion_points = data.reshape(-1, 3)
            self.get_logger().info(f'Received {len(corrosion_points)} corrosion points')
            
            # Check if workspace parameterization is ready
            if not self.surf.is_ready:
                self.get_logger().warn('Workspace parameterization not ready yet. Storing data and will retry...')
                self.pending_corrosion_data = corrosion_points
                
                # Set up retry timer if not already running
                if self.retry_timer is None:
                    self.retry_timer = self.create_timer(1.0, self._retry_corrosion_processing)
                return
            
            # Process the corrosion data
            self._process_corrosion_data(corrosion_points)
            
        except Exception as e:
            self.get_logger().error(f'Error in corrosion callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _retry_corrosion_processing(self):
        """
        Timer callback to retry processing pending corrosion data.
        Stops when parameterization is ready and data is processed.
        """
        if self.pending_corrosion_data is None:
            # No pending data, cancel timer
            if self.retry_timer is not None:
                self.retry_timer.cancel()
                self.retry_timer = None
            return
        
        if self.surf.is_ready:
            self.get_logger().info('Parameterization ready - processing pending corrosion data')
            try:
                self._process_corrosion_data(self.pending_corrosion_data)
                self.pending_corrosion_data = None
                
                # Cancel the retry timer
                if self.retry_timer is not None:
                    self.retry_timer.cancel()
                    self.retry_timer = None
            except Exception as e:
                self.get_logger().error(f'Error processing pending corrosion data: {str(e)}')
        else:
            self.get_logger().debug('Still waiting for parameterization to be ready...')
    
    def _process_corrosion_data(self, corrosion_points):
        """
        Process corrosion points: map to UV and compute bounds.
        
        Args:
            corrosion_points: Nx3 array of XYZ coordinates
        """
        self.corrosion_points = corrosion_points
        
        # Map corrosion points to UV coordinates using the workspace parameterization
        corrosion_uv = self._map_xyz_to_uv(self.corrosion_points)
        
        # Get workspace UV bounds for validation
        workspace_bounds = self.surf.get_uv_bounds()
        
        # Compute UV bounds of the corrosion area
        corrosion_u_min = float(np.min(corrosion_uv[:, 0]))
        corrosion_u_max = float(np.max(corrosion_uv[:, 0]))
        corrosion_v_min = float(np.min(corrosion_uv[:, 1]))
        corrosion_v_max = float(np.max(corrosion_uv[:, 1]))
        
        # Clamp corrosion bounds to workspace bounds to ensure interpolation validity
        # Add small margin to avoid edge issues
        margin_u = (workspace_bounds['u_max'] - workspace_bounds['u_min']) * 0.01
        margin_v = (workspace_bounds['v_max'] - workspace_bounds['v_min']) * 0.01
        
        self.corrosion_uv_bounds = {
            'u_min': max(corrosion_u_min, workspace_bounds['u_min'] + margin_u),
            'u_max': min(corrosion_u_max, workspace_bounds['u_max'] - margin_u),
            'v_min': max(corrosion_v_min, workspace_bounds['v_min'] + margin_v),
            'v_max': min(corrosion_v_max, workspace_bounds['v_max'] - margin_v)
        }
        
        # Log if bounds were clamped
        if (corrosion_u_min < workspace_bounds['u_min'] or corrosion_u_max > workspace_bounds['u_max'] or
            corrosion_v_min < workspace_bounds['v_min'] or corrosion_v_max > workspace_bounds['v_max']):
            self.get_logger().warn('Corrosion bounds exceed workspace bounds - clamping to valid range')
            self.get_logger().warn(f'  Original: U=[{corrosion_u_min:.3f}, {corrosion_u_max:.3f}], V=[{corrosion_v_min:.3f}, {corrosion_v_max:.3f}]')
            self.get_logger().warn(f'  Workspace: U=[{workspace_bounds["u_min"]:.3f}, {workspace_bounds["u_max"]:.3f}], V=[{workspace_bounds["v_min"]:.3f}, {workspace_bounds["v_max"]:.3f}]')
        
        # Compute boundary/perimeter points using ConvexHull
        if len(corrosion_uv) >= 3:  # Need at least 3 points for convex hull
            try:
                hull = ConvexHull(corrosion_uv)
                # Get the boundary points in order (hull.vertices gives indices)
                self.corrosion_boundary_uv = corrosion_uv[hull.vertices]
                self.get_logger().info(f'Computed {len(self.corrosion_boundary_uv)} boundary points from convex hull')
            except Exception as e:
                self.get_logger().warn(f'Could not compute convex hull: {str(e)}. Using all points as boundary.')
                self.corrosion_boundary_uv = corrosion_uv
        else:
            # Not enough points for convex hull, use all points
            self.corrosion_boundary_uv = corrosion_uv
        
        self.get_logger().info(f'Corrosion UV bounds (clamped): U=[{self.corrosion_uv_bounds["u_min"]:.3f}, {self.corrosion_uv_bounds["u_max"]:.3f}], '
                             f'V=[{self.corrosion_uv_bounds["v_min"]:.3f}, {self.corrosion_uv_bounds["v_max"]:.3f}]')
    
    
    def _map_xyz_to_uv(self, xyz_points):
        """
        Map XYZ points to UV coordinates using the workspace parameterization.
        
        This finds the nearest point in the parameterized workspace and returns its UV coordinates.
        
        Args:
            xyz_points: Nx3 array of XYZ coordinates
            
        Returns:
            Nx2 array of UV coordinates
        """
        if not self.surf.is_ready:
            raise ValueError("Parameterization not ready")
        
        # Transform points to local frame (same as workspace)
        centered_points = xyz_points - self.surf.centroid
        points_local = centered_points @ self.surf.principal_axes.T
        
        # Build KD-tree from workspace points in local frame if not already built
        if self.surf.kdtree_xyz is None:
            self.surf.kdtree_xyz = cKDTree(self.surf.points_local)
        
        # Find nearest neighbors in workspace
        distances, indices = self.surf.kdtree_xyz.query(points_local, k=1)
        
        # Get UV coordinates of nearest workspace points
        uv_coords = self.surf.uv_params[indices]
        
        self.get_logger().info(f'Mapped {len(xyz_points)} XYZ points to UV (mean distance: {np.mean(distances):.4f})')
        
        return uv_coords
    

    def uv_path_callback(self, msg):
        """Convert UV path to XYZ path."""
        try:
            if not self.surf.is_ready:
                self.get_logger().warn('Parameterization not ready yet')
                return
            
            # Convert message to UV array
            uv_path = np.array(msg.data).reshape(-1, 2)
            
            if len(uv_path) == 0:
                self.get_logger().warn('Received empty UV path')
                return
            
            # Interpolate to XYZ
            xyz_path = self.surf.interpolate(uv_path)
            
            # Publish result to path planner's XYZ topic
            xyz_msg = Float64MultiArray()
            xyz_msg.data = xyz_path.flatten().tolist()
            self.xyz_path_pub.publish(xyz_msg)
            
            self.get_logger().info(f'Converted UV path to XYZ: {len(xyz_path)} points')
        
        except Exception as e:
            self.get_logger().error(f'Error converting UV path: {str(e)}')


    def interpolate_callback(self, request, response):
        """
        Service callback for interpolation.
        Maps (u,v) → (x,y,z).
        """
        try:
            if not self.surf.is_ready:
                response.success = False
                response.message = 'Parameterization not ready. No point cloud received yet.'
                return response
            
            # Validate input
            if len(request.uv_points) == 0:
                response.success = False
                response.message = 'Empty input array'
                return response
            
            # Extract u and v values from UVPoint list
            u_values = [uv_pt.u for uv_pt in request.uv_points]
            v_values = [uv_pt.v for uv_pt in request.uv_points]
            
            # Create UV query array
            uv_query = np.column_stack([u_values, v_values])
            
            # Interpolate
            xyz = self.surf.interpolate(uv_query)
            
            # Convert to Point messages
            response.points = []
            for point in xyz:
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = float(point[2])
                response.points.append(p)
            
            response.success = True
            response.message = f'Interpolated {len(xyz)} points'
            
        except Exception as e:
            response.success = False
            response.message = f'Interpolation error: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    

    def get_uv_bounds_callback(self, request, response):
        """
        Service callback for getting UV parameter space bounds.
        Returns corrosion UV bounds and boundary points.
        Requires both workspace parameterization and corrosion bounds to be ready.
        """
        try:
            if not self.surf.is_ready:
                response.success = False
                response.message = 'Parameterization not ready. No point cloud received yet.'
                return response
            
            # Check if corrosion bounds are available
            if self.corrosion_uv_bounds is None:
                response.success = False
                response.message = 'Corrosion bounds not available yet. Waiting for corrosion data to be processed.'
                self.get_logger().warn('UV bounds requested but corrosion data not processed yet')
                return response
            
            # Initialize boundary to empty
            response.boundary = UVBoundary()
            response.boundary.points = []
            
            # Return corrosion bounds
            bounds = self.corrosion_uv_bounds
            self.get_logger().info('Returning corrosion UV bounds')
            
            # Add boundary points if available
            if self.corrosion_boundary_uv is not None:
                for uv in self.corrosion_boundary_uv:
                    point = UVPoint()
                    point.u = float(uv[0])
                    point.v = float(uv[1])
                    response.boundary.points.append(point)
                
                self.get_logger().info(f'Returning {len(self.corrosion_boundary_uv)} boundary points')
            
            response.u_min = bounds['u_min']
            response.u_max = bounds['u_max']
            response.v_min = bounds['v_min']
            response.v_max = bounds['v_max']
            response.success = True
            response.message = 'UV bounds retrieved successfully'
            
        except Exception as e:
            response.success = False
            response.message = f'Error getting UV bounds: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    

    def publish_status(self):
        """
        Publish status message periodically.
        """
        msg = ParameterizationStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'parameterization'
        
        # Only report ready when both workspace parameterization AND corrosion bounds are available
        # This prevents path planner from starting with workspace bounds instead of corrosion bounds
        is_ready = self.surf.is_ready and self.corrosion_uv_bounds is not None
        msg.is_ready = is_ready
        
        if self.surf.is_ready:
            try:
                metrics = self.surf.evaluate_quality(
                    sample_size=min(100, self.quality_sample_size)
                )
                msg.num_points = metrics['total_points']
                msg.mean_error = metrics['mean_error']
                msg.max_error = metrics['max_error']
                msg.rmse = metrics['rmse']
                msg.std_error = metrics['std_error']
            except Exception as e:
                self.get_logger().warn(f'Error computing status metrics: {str(e)}')
                msg.num_points = 0
                msg.mean_error = 0.0
                msg.max_error = 0.0
                msg.rmse = 0.0
                msg.std_error = 0.0
        else:
            msg.num_points = 0
            msg.mean_error = 0.0
            msg.max_error = 0.0
            msg.rmse = 0.0
            msg.std_error = 0.0
        
        self.status_pub.publish(msg)
        
        # Publish ready status for path planning
        ready_msg = Bool()
        ready_msg.data = is_ready
        self.ready_pub.publish(ready_msg)
        
        # Publish corrosion UV bounds and boundary when ready
        if is_ready:
            # Publish UV parameter space bounds
            bounds_msg = Float64MultiArray()
            bounds_msg.data = [
                self.corrosion_uv_bounds['u_min'],
                self.corrosion_uv_bounds['u_max'],
                self.corrosion_uv_bounds['v_min'],
                self.corrosion_uv_bounds['v_max']
            ]
            self.uv_bounds_pub.publish(bounds_msg)
            
            # Publish corrosion boundary points
            if self.corrosion_boundary_uv is not None:
                boundary_msg = Float64MultiArray()
                boundary_msg.data = self.corrosion_boundary_uv.flatten().tolist()
                self.uv_boundary_pub.publish(boundary_msg)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = ParameterizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
