#!/usr/bin/env python3
"""
Surface Parameterization ROS 2 Node

This node receives point clouds and provides surface parameterization services.
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
from parameterization.surface_parameterization import SurfaceParameterization

class ParameterizationNode(Node):
    """
    ROS 2 Node for surface parameterization.
    
    Subscribes to:
        - /corrosion/scatter_plot_pub (std_msgs/Float32MultiArray): Input XYZ points from corrosion detection
        
    Publishes:
        - /parameterization/status (ParameterizationStatus): Status and quality metrics
        
    Services:
        - /parameterization/interpolate (InterpolatePoint): Interpolate 3D points from (u,v)
        - /parameterization/get_uv_bounds (GetUVBounds): Get parameter space bounds
        
    Parameters:
        - interpolation_method (string): 'rbf' for radial basis function
        - neighbors (int): Number of neighbors for local RBF interpolation
        - normalize (bool): Normalize UV to [0,1] (default: False)
        - quality_sample_size (int): Sample size for quality evaluation
        - status_publish_rate (float): Rate to publish status (Hz)
    """
    
    def __init__(self):
        super().__init__('parameterization_node')
        
        # Declare parameters
        self.declare_parameter('interpolation_method', 'rbf')
        self.declare_parameter('neighbors', 50)
        self.declare_parameter('normalize', False)
        self.declare_parameter('quality_sample_size', 1000)
        self.declare_parameter('status_publish_rate', 1.0)
        
        # Get parameters
        self.interpolation_method = self.get_parameter('interpolation_method').value
        self.neighbors = self.get_parameter('neighbors').value
        self.normalize = self.get_parameter('normalize').value
        self.quality_sample_size = self.get_parameter('quality_sample_size').value
        status_rate = self.get_parameter('status_publish_rate').value
        
        # Initialize parameterization
        self.surf = SurfaceParameterization()
        
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

        # UV parameterization publisher
        self.param_uv_pub = self.create_publisher(
            Float64MultiArray,
            '/parameterization/param_uv',
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
        
        # Create timer for status publishing
        self.status_timer = self.create_timer(
            1.0 / status_rate,
            self.publish_status
        )
        
        self.get_logger().info('Parameterization node initialized')
        self.get_logger().info(f'  Method: {self.interpolation_method}')
        self.get_logger().info(f'  Neighbors: {self.neighbors}')
        self.get_logger().info(f'  Normalize: {self.normalize}')
    
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
            
            # Compute local frame (not normalized)
            self.surf.compute_local_frame()
            self.get_logger().info('Local frame computed')
            
            # Compute parameterization using projection
            self.surf.compute_uv_parameterization(
                method='projection',
                normalize=self.normalize
            )
            self.get_logger().info('UV parameterization computed')
            
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
