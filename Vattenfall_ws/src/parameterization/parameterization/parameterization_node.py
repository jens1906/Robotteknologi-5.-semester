#!/usr/bin/env python3
"""
Surface Parameterization ROS 2 No        self.declare_parameter('quality_sample_size', 1000)
        self.declare_parameter('status_publish_rate', 1.0)
        self.declare_parameter('conformal_iterations', 0)  # Set to 0 to skip (too slow for large clouds)
        self.declare_parameter('conformal_alpha', 0.5)
        self.declare_parameter('metric_neighbors', 20)his node receives point clouds and provides surface parameterization services.
It computes a 2D parameter space (u,v) mapping from 3D points (x,y,z) and offers
interpolation and surface normal computation services.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Float64MultiArray, Float32MultiArray
import numpy as np

from parameterization.msg import ParameterizationStatus, UVPoint
from parameterization.srv import InterpolatePoint, GetUVBounds

# Import the parameterization module
from parameterization.conformal_parameterization import ConformalParameterization

class ParameterizationNode(Node):
    """
    ROS 2 Node for surface parameterization using Amersdorfer et al. (2021) approach.
    
    Implements conformal parameterization with metric tensor computation for
    equidistant path planning on curved surfaces.
    
    Subscribes to:
        - /corrosion/scatter_plot_pub (std_msgs/Float32MultiArray): Input XYZ points from corrosion detection
        
    Publishes:
        - /parameterization/status (ParameterizationStatus): Status and quality metrics
        
    Services:
        - /parameterization/interpolate (InterpolatePoint): Interpolate 3D points from (u,v)
        - /parameterization/get_uv_bounds (GetUVBounds): Get parameter space bounds
        
    Parameters:
        - interpolation_method (string): 'rbf' for radial basis function
        - neighbors (int): Number of neighbors for local RBF interpolation and metric computation
        - quality_sample_size (int): Sample size for quality evaluation
        - status_publish_rate (float): Rate to publish status (Hz)
        - conformal_iterations (int): Number of conformal correction iterations (default: 5)
        - conformal_alpha (float): Conformal correction step size (default: 0.5)
        - metric_neighbors (int): Number of neighbors for metric tensor computation (default: 20)
    """
    
    def __init__(self):
        super().__init__('parameterization_node')
        
        # Declare parameters
        self.declare_parameter('interpolation_method', 'rbf')
        self.declare_parameter('neighbors', 50)
        self.declare_parameter('quality_sample_size', 1000)
        self.declare_parameter('status_publish_rate', 1.0)
        self.declare_parameter('conformal_iterations', 1)
        self.declare_parameter('conformal_alpha', 0.5)
        self.declare_parameter('metric_neighbors', 20)
        
        # Get parameters
        self.interpolation_method = self.get_parameter('interpolation_method').value
        self.neighbors = self.get_parameter('neighbors').value
        self.quality_sample_size = self.get_parameter('quality_sample_size').value
        status_rate = self.get_parameter('status_publish_rate').value
        self.conformal_iterations = self.get_parameter('conformal_iterations').value
        self.conformal_alpha = self.get_parameter('conformal_alpha').value
        self.metric_neighbors = self.get_parameter('metric_neighbors').value
        
        # Initialize conformal parameterization (Amersdorfer et al. 2021)
        self.surf = ConformalParameterization()
        self.get_logger().info('Using Conformal Parameterization')

        
        # Create subscriber for corrosion scatter plot
        self.scatter_plot_sub = self.create_subscription(
            Float32MultiArray,
            '/corrosion/scatter_plot_pub',
            self.scatter_plot_callback,
            10
        )
        
        # Create publisher
        self.status_pub = self.create_publisher(
            ParameterizationStatus,
            '/parameterization/status',
            10
        )

        self.param_uv_pub = self.create_publisher(
            Float64MultiArray,
            '/parameterization/param_uv',
            10
        )

        self.xyz_path_pub = self.create_publisher(
            Float64MultiArray,
            '/path/xyz_path',
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
        self.get_logger().info(f'  Interpolation method: {self.interpolation_method}')
        self.get_logger().info(f'  Interpolation neighbors: {self.neighbors}')
        self.get_logger().info(f'  Metric neighbors: {self.metric_neighbors}')
        self.get_logger().info(f'  Conformal iterations: {self.conformal_iterations}')
        self.get_logger().info(f'  Conformal alpha: {self.conformal_alpha}')
    

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
            
            # Conformal parameterization (Amersdorfer et al. 2021)
            # Step 1: Initial UV parameterization via projection
            self.surf.compute_initial_parameterization(method='projection')
            self.get_logger().info('Initial UV parameterization computed')
            
            # Step 2: Compute surface metric tensor (E, F, G)
            self.surf.compute_surface_metric(k_neighbors=self.metric_neighbors)
            self.get_logger().info('Surface metric tensor computed')
            
            # Step 3: Apply conformal correction to minimize distortion
            self.surf.apply_conformal_correction(
                iterations=self.conformal_iterations,
                alpha=self.conformal_alpha
            )
            self.get_logger().info('Conformal correction applied')
            
            # Build inverse interpolation
            self.surf.build_inverse_interpolation(
                method=self.interpolation_method,
                neighbors=self.neighbors
            )
            self.get_logger().info('Inverse interpolation built')
            
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
        """
        try:
            if not self.surf.is_ready:
                response.success = False
                response.message = 'Parameterization not ready. No point cloud received yet.'
                return response
            
            bounds = self.surf.get_uv_bounds()
            
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
        
        msg.is_ready = self.surf.is_ready
        
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
