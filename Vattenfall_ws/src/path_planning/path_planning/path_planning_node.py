import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from parameterization.msg import ParameterizationStatus
from parameterization.srv import GetUVBounds

"""

The functionality of the Path Planner Node

- Subscribes to parameterization status to know when it's ready.
- Uses the GetUVBounds service to get UV parameter space bounds.
- Generates zigzag paths in UV space with interconnection between Bézier curves.
- Publishes the generated UV path data for conversion to XYZ by parameterization node.

Subscribers:
- /parameterization/status (ParameterizationStatus): Parameterization status and readiness.

Publishers:
- /path/uv_path (Float64MultiArray): Generated UV path data (flattened Nx2 array).

Services Used:
- /parameterization/get_uv_bounds (GetUVBounds): Get UV parameter space bounds.

Parameters:
- point_spacing (float): Spacing between points along scan lines in UV units (default: 0.5)
- line_spacing (float): Spacing between parallel scan lines in UV units (default: 5.0)
- n_bezier (int): Number of points in Bézier curves connecting lines (default: 25)
- auto_generate (bool): Auto-generate path when parameterization ready (default: True)

"""

class PathPlanner(Node):
    """ Path Planner"""
    
    def __init__(self):
        super().__init__('path_planner_node') # Initialize ROS2 node

        # Parameters (values are in UV space, typically 0-100 range)
        self.declare_parameter('point_spacing', 0.5)  # Spacing between points along lines (V direction)
        self.declare_parameter('line_spacing', 5.0)  # Spacing between scan lines (U direction)
        self.declare_parameter('n_bezier', 25)  # Number of points for generating connecting curves
        self.declare_parameter('auto_generate', True)  # Automatically generate path when parameterization is ready
        self.declare_parameter('test', False)  # Test mode

        self.test = self.get_parameter('test').value
        self.point_spacing = self.get_parameter('point_spacing').value
        self.line_spacing = self.get_parameter('line_spacing').value
        self.n_bezier = self.get_parameter('n_bezier').value
        self.auto_generate = self.get_parameter('auto_generate').value
        self.bezier_curvature = self.line_spacing * 0.75  # Smoother curves
        self.tau = np.linspace(0, 1, self.n_bezier)

        # Data holders
        self.uv_bounds = None
        self.paths_uv = None
        self.parameterization_ready = False

        # Create service client for UV bounds
        self.bounds_client = self.create_client(
            GetUVBounds,
            '/parameterization/get_uv_bounds'
        )

        # Subscribers
        if self.test != True:
            self.create_subscription(
                ParameterizationStatus,
                '/parameterization/status',
                self.status_callback,
                10
            )
        else:
            self.testing_mode()

        # Publishers
        self.uv_path_pub = self.create_publisher(Float64MultiArray, '/path/uv_path', 10)
        
        # Debug info
        self.get_logger().info('Path Planner node initialized')
        self.get_logger().info(f'  Test mode: {self.test}')
        self.get_logger().info(f'  Point spacing (V direction): {self.point_spacing}')
        self.get_logger().info(f'  Line spacing (U direction): {self.line_spacing}')
        self.get_logger().info(f'  Bezier points: {self.n_bezier}')
        self.get_logger().info(f'  Auto-generate: {self.auto_generate}')


    def plot_uv_points_and_path(self):
        """ Plot bounds and Path"""
        try:
            import matplotlib.pyplot as plt
            if self.uv_bounds is None or self.paths_uv is None or self.uv_path is None:
                self.get_logger().error('Cannot plot: UV bounds, paths, or uv_path not available')
                return
            u_min = self.uv_bounds['u_min']
            u_max = self.uv_bounds['u_max']
            v_min = self.uv_bounds['v_min']
            v_max = self.uv_bounds['v_max']
            plt.figure(figsize=(10, 6))
            # Plot bounds
            plt.plot([u_min, u_max, u_max, u_min, u_min],
                     [v_min, v_min, v_max, v_max, v_min], 'k--', label='UV Bounds')
            # Plot zigzag lines
            for u_line, v_line in zip(self.paths_uv[0], self.paths_uv[1]):
                plt.plot(u_line, v_line, 'b.-', alpha=0.5)
            # Plot continuous path
            plt.plot(self.uv_path[:, 0], self.uv_path[:, 1], 'r-', linewidth=2, label='Continuous Path')
            plt.title('UV Path Planning')
            plt.xlabel('U')
            plt.ylabel('V')
            plt.legend()
            plt.axis('equal')
            plt.grid(True)
            plt.show()
        except ImportError:
            self.get_logger().error('matplotlib not installed, cannot plot UV points and path.')
        except Exception as e:
            self.get_logger().error(f'Error in plot_uv_points_and_path: {e}')

    def testing_mode(self):
        """Test mode with synthetic UV bounds - validates path generation without parameterization node."""
        self.get_logger().info('RUNNING IN TEST MODE')
        
        # Create realistic UV bounds (typical range 0-100)
        self.uv_bounds = {
            'u_min': 0.0,
            'u_max': 100.0,
            'v_min': 0.0,
            'v_max': 50.0
        }
        
        self.get_logger().info(f'Test UV bounds: U=[{self.uv_bounds["u_min"]}, {self.uv_bounds["u_max"]}], '
                             f'V=[{self.uv_bounds["v_min"]}, {self.uv_bounds["v_max"]}]')
        
        # Generate zigzag paths
        self.get_logger().info('Generating zigzag paths...')
        self.generate_zigzag_paths()
        
        # Validate zigzag paths
        if self.paths_uv is None:
            self.get_logger().error('TEST FAILED: Path generation returned None')
            return
        
        u_lines, v_lines = self.paths_uv
        expected_lines = max(2, int(np.ceil((self.uv_bounds['u_max'] - self.uv_bounds['u_min']) / self.line_spacing)) + 1)
        
        if len(u_lines) != expected_lines:
            self.get_logger().warn(f'Line count mismatch: expected ~{expected_lines}, got {len(u_lines)}')
        
        # Publish continuous path
        self.get_logger().info('Creating continuous path with Bézier curves...')
        self.uv_path = self.create_continuous_path()
        
        # Summary
        self.get_logger().info(f'TEST SUMMARY:')
        self.get_logger().info(f'  ✓ Generated {len(u_lines)} scan lines')
        self.get_logger().info(f'  ✓ {len(v_lines[0])} points per line')
        self.get_logger().info(f'  ✓ Total waypoints: {sum(len(line) for line in u_lines)}')
        self.get_logger().info(f'  ✓ Published continuous path to /path/uv_path')
        self.get_logger().info('Test mode complete - node will continue running for integration tests')
        self.plot_uv_points_and_path()


    def status_callback(self, msg):
        """
        Callback for parameterization status.
        When parameterization becomes ready, fetch UV bounds and generate path.
        """
        try:
            # Track previous state to detect transitions
            previously_ready = self.parameterization_ready
            self.parameterization_ready = msg.is_ready
            
            # Only process if we have valid data (num_points > 0)
            if not msg.is_ready or msg.num_points == 0:
                if previously_ready:
                    self.get_logger().warn('Parameterization no longer ready')
                self.parameterization_ready = False
                return
            
            # If just became ready (state transition), generate path
            if self.parameterization_ready and not previously_ready:
                self.get_logger().info(f'Parameterization is ready with {msg.num_points} points. Fetching UV bounds...')
                
                if self.auto_generate:
                    self.fetch_uv_bounds_async()
                else:
                    self.get_logger().info('Auto-generate disabled. Call generate_path() manually.')
                    
        except Exception as e:
            self.get_logger().error(f"Error in status_callback: {e}")


    def fetch_uv_bounds_async(self):
        """
        Fetch UV bounds asynchronously from parameterization service.
        """
        try:
            # Wait for service
            if not self.bounds_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('UV bounds service not available')
                return
            
            # Call service asynchronously
            request = GetUVBounds.Request()
            future = self.bounds_client.call_async(request)
            future.add_done_callback(self.uv_bounds_callback)
                
        except Exception as e:
            self.get_logger().error(f"Error fetching UV bounds: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


    def uv_bounds_callback(self, future):
        """
        Callback when UV bounds service call completes.
        """
        try:
            response = future.result()
            
            if response.success:
                self.uv_bounds = {
                    'u_min': response.u_min,
                    'u_max': response.u_max,
                    'v_min': response.v_min,
                    'v_max': response.v_max
                }
                self.get_logger().info(
                    f'UV bounds received: U=[{response.u_min:.3f}, {response.u_max:.3f}], '
                    f'V=[{response.v_min:.3f}, {response.v_max:.3f}]'
                )
                
                # Generate and publish path
                self.generate_zigzag_paths()
                self.publish_path()
            else:
                self.get_logger().error(f'Failed to get UV bounds: {response.message}')
                
        except Exception as e:
            self.get_logger().error(f'Error in UV bounds callback: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


    def fetch_uv_bounds(self):
        """
        Fetch UV bounds from parameterization service.
        Returns True if successful, False otherwise.
        """
        try:
            # Wait for service
            if not self.bounds_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('UV bounds service not available')
                return False
            
            # Call service
            request = GetUVBounds.Request()
            future = self.bounds_client.call_async(request)
            
            # Wait for the future to complete
            import time
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.done():
                try:
                    response = future.result()
                    
                    if response.success:
                        self.uv_bounds = {
                            'u_min': response.u_min,
                            'u_max': response.u_max,
                            'v_min': response.v_min,
                            'v_max': response.v_max
                        }
                        self.get_logger().info(
                            f'UV bounds received: U=[{response.u_min:.3f}, {response.u_max:.3f}], '
                            f'V=[{response.v_min:.3f}, {response.v_max:.3f}]'
                        )
                        return True
                    else:
                        self.get_logger().error(f'Failed to get UV bounds: {response.message}')
                        return False
                except Exception as e:
                    self.get_logger().error(f'Exception getting result: {e}')
                    return False
            else:
                self.get_logger().error('Service call timed out')
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error fetching UV bounds: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False


    def generate_path(self):
        """
        Manually trigger path generation.
        Useful when auto_generate is disabled.
        """
        try:
            if not self.parameterization_ready:
                self.get_logger().error('Cannot generate path: parameterization not ready')
                return False
            
            if self.fetch_uv_bounds():
                self.generate_zigzag_paths()
                self.publish_path()
                return True
            else:
                self.get_logger().error('Failed to fetch UV bounds')
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error in generate_path: {e}")
            return False


    def cubic_bezier(self, b0, b1, b2, b3):
        """Cubic Bézier curve."""
        t = self.tau[:, None]
        return (1-t)**3 * b0 + 3*(1-t)**2 * t * b1 + 3*(1-t)*t**2 * b2 + t**3 * b3


    def generate_zigzag_paths(self):
        """Generate zigzag lines in UV space using bounds from service."""
        try:
            if self.uv_bounds is None:
                raise ValueError("Error: generate_zigzag_paths(): UV bounds not available.")
            
            u_min = self.uv_bounds['u_min']
            u_max = self.uv_bounds['u_max']
            v_min = self.uv_bounds['v_min']
            v_max = self.uv_bounds['v_max']

            # Calculate number of lines based on line_spacing to ensure full coverage
            u_range = u_max - u_min
            line_n = max(2, int(np.ceil(u_range / self.line_spacing)) + 1)
            
            self.get_logger().info(f'UV bounds: U=[{u_min:.3f}, {u_max:.3f}] (range={u_range:.3f}), '
                                 f'V=[{v_min:.3f}, {v_max:.3f}] (range={v_max - v_min:.3f})')

            # Calculate number of points per line based on point_spacing
            v_range = v_max - v_min
            points_per_line = max(2, int(np.ceil(v_range / self.point_spacing)) + 1)

            # Use linspace to ensure coverage from u_min to u_max
            u_lin = np.linspace(u_min, u_max, line_n)
            v_lin = np.linspace(v_min, v_max, points_per_line)
            
            # Calculate actual spacing achieved
            actual_u_spacing = (u_max - u_min) / (line_n - 1) if line_n > 1 else 0
            actual_v_spacing = (v_max - v_min) / (points_per_line - 1) if points_per_line > 1 else 0
            max_distance_u = actual_u_spacing / 2  # Maximum distance from any point to nearest line
            max_distance_v = actual_v_spacing / 2  # Maximum distance from any point to nearest point on line

            u_lines, v_lines = [], []

            # If odd number of lines, start from max to ensure ending at top-right
            start_reversed = (line_n % 2 == 0)
            
            for i, u in enumerate(u_lin):
                # Alternate direction for zigzag
                should_reverse = (i % 2 == 1) if not start_reversed else (i % 2 == 0)
                v_line = v_lin[::-1] if should_reverse else v_lin
                    
                u_lines.append(np.full_like(v_line, u))
                v_lines.append(v_line)

            self.paths_uv = (u_lines, v_lines)
            self.get_logger().info(f'Generated zigzag path: {line_n} lines × {points_per_line} points = {line_n * points_per_line} total points')
            self.get_logger().info(f'Actual spacing: U={actual_u_spacing:.4f} (target={self.line_spacing}), '
                                 f'V={actual_v_spacing:.4f} (target={self.point_spacing})')

        except Exception as e:
            self.get_logger().error(f"Error: generate_zigzag_paths(): {e}")


    def create_continuous_path(self):
        """Create continuous path with Bézier smoothing."""
        try:
            if self.paths_uv is None:
                raise ValueError("Error: create_continuous_path(): Paths UV data is not available.")
        
            path = []
            n_lines = len(self.paths_uv[0])

            for i in range(n_lines):
                u_line, v_line = self.paths_uv[0][i], self.paths_uv[1][i]
                
                # Add line - don't reverse the line, the zigzag is already in the data
                path.append(np.column_stack([u_line, v_line]))

                # Add Bézier curve to next line
                if i < n_lines - 1:
                    end = np.array([u_line[-1], v_line[-1]])
                    next_u, next_v = self.paths_uv[0][i+1], self.paths_uv[1][i+1]
                    next_start = np.array([next_u[0], next_v[0]])

                    # Calculate direction vectors for smooth transition
                    # Use the direction from second-to-last to last point for current line
                    if len(u_line) > 1:
                        vec_curr = end - np.array([u_line[-2], v_line[-2]])
                    else:
                        vec_curr = np.array([1.0, 0.0])  # Default horizontal direction
                    
                    # Use the direction from first to second point for next line
                    if len(next_u) > 1:
                        vec_next = np.array([next_u[1], next_v[1]]) - next_start
                    else:
                        vec_next = np.array([1.0, 0.0])  # Default horizontal direction
                    
                    norm_curr = np.linalg.norm(vec_curr)
                    norm_next = np.linalg.norm(vec_next)

                    if norm_curr > 1e-6 and norm_next > 1e-6:
                        b0 = end
                        b1 = end + self.bezier_curvature * vec_curr / norm_curr
                        b2 = next_start - self.bezier_curvature * vec_next / norm_next
                        b3 = next_start

                        path.append(self.cubic_bezier(b0, b1, b2, b3))

            return np.vstack(path)
    
        except Exception as e:
            self.get_logger().error(f"Error in create_continuous_path: {str(e)}")
            return None


    def publish_path(self):
        """Generate and publish UV path."""
        try:
            uv_path = self.create_continuous_path()
            if uv_path is None:
                raise ValueError("Error: publish_path(): Path generation failed.")
            
            # Publish UV path
            uv_msg = Float64MultiArray()
            uv_msg.data = uv_path.flatten().tolist()
            self.uv_path_pub.publish(uv_msg)
            self.get_logger().info(f'Published UV path: {len(uv_path)} points')
        
        except Exception as e:
            self.get_logger().error(f"Error: publish_path(): {e}")


def main():
    try:
        rclpy.init()
        rclpy.spin(PathPlanner())
        rclpy.shutdown()
    
    except Exception as e:
        print(f"Path - Error: main(): {e}")

if __name__ == '__main__':
    main()

