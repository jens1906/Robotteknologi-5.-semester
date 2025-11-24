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

        self.declare_parameter('point_spacing', 5)  # Spacing between points along lines (V direction) - mm
        self.declare_parameter('line_spacing', 50.0)  # Spacing between scan lines (U direction) - mm
        self.declare_parameter('n_bezier', 50)  # Number of points for generating connecting curves - n
        self.declare_parameter('auto_generate', True)  # Automatically generate path when parameterization is ready
        self.declare_parameter('test', False)  # Test mode

        # Load parameters
        self.test = self.get_parameter('test').value
        self.point_spacing = self.get_parameter('point_spacing').value
        self.line_spacing = self.get_parameter('line_spacing').value
        self.n_bezier = self.get_parameter('n_bezier').value
        self.auto_generate = self.get_parameter('auto_generate').value
        self.tau = np.linspace(0, 1, self.n_bezier)

        # Data holders
        self.uv_boundary = None  # Boundary points from UVBoundary
        self.paths_uv = None
        self.parameterization_ready = False
        

        # Service client
        self.bounds_client = self.create_client(
            GetUVBounds,
            '/parameterization/get_uv_bounds'
        )

        if self.test != True:
            self.create_subscription(
                Float64MultiArray,
                '/corrosion/tool_size',
                self.tool_size_callback,
                10
            )

            self.create_subscription(
                ParameterizationStatus,
                '/parameterization/status',
                self.status_callback,
                10
            )
        else:
            self.tool_size = 25
            self.line_spacing = 2 * self.tool_size - 5  # Full coverage with overlap
            self.testing_mode()

        # Publishers
        self.uv_path_pub = self.create_publisher(
            Float64MultiArray,
            '/path/uv_path',
            10)
        
        self.on_surface_pub = self.create_publisher(
            Float64MultiArray,
            '/path/on_surface',
            10)
                
        # Debug info
        self.get_logger().info('Path Planner node initialized')
        self.get_logger().info(f'  Test mode: {self.test}')
        self.get_logger().info(f'  Point spacing (V direction): {self.point_spacing}')
        self.get_logger().info(f'  Line spacing (U direction): {self.line_spacing}')
        self.get_logger().info(f'  Bezier points: {self.n_bezier}')
        self.get_logger().info(f'  Auto-generate: {self.auto_generate}')


    def tool_size_callback(self, msg):
        """Update tool size and line spacing from corrosion detection (expects data[1] = tool size)."""
        if len(msg.data) > 1:
            self.tool_size = msg.data[1]
            # Line spacing should be 2 * tool_radius for full coverage with overlap
            self.line_spacing = 2 * self.tool_size
            self.get_logger().info(f'Tool size updated: {self.tool_size:.3f}')
            self.get_logger().info(f'Line spacing updated: {self.line_spacing:.3f}')

    def test_plot(self):
        """ Plot bounds, path, and tool coverage"""
        try:
            import matplotlib.pyplot as plt
            from matplotlib.patches import Circle
            if self.uv_bounds is None or self.paths_uv is None or self.uv_path is None:
                self.get_logger().error('Cannot plot: UV bounds, paths, or uv_path not available')
                return
            
            u_min = self.uv_bounds['u_min']
            u_max = self.uv_bounds['u_max']
            v_min = self.uv_bounds['v_min']
            v_max = self.uv_bounds['v_max']
            
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
            
            # Left plot: Path visualization
            ax1.plot([u_min, u_max, u_max, u_min, u_min],
                     [v_min, v_min, v_max, v_max, v_min], 'k--', linewidth=2, label='UV Bounds')
            
            # Plot zigzag lines
            for u_line, v_line in zip(self.paths_uv[0], self.paths_uv[1]):
                ax1.plot(u_line, v_line, 'b.-', alpha=0.5)
            
            # Plot continuous path
            ax1.plot(self.uv_path[:, 0], self.uv_path[:, 1], 'r-', linewidth=2, label='Continuous Path')
            
            ax1.set_title('UV Path Planning')
            ax1.set_xlabel('U')
            ax1.set_ylabel('V')
            ax1.legend()
            ax1.axis('equal')
            ax1.grid(True)
            
            # Right plot: Tool coverage visualization
            ax2.plot([u_min, u_max, u_max, u_min, u_min],
                     [v_min, v_min, v_max, v_max, v_min], 'k--', linewidth=2, label='UV Bounds')
                        
            # Plot tool coverage circles along the full continuous path (including Bézier curves)
            # Sample every N points to avoid too many circles
            step = max(1, len(self.uv_path) // 100)
            for i in range(0, len(self.uv_path), step):
                circle = Circle((self.uv_path[i, 0], self.uv_path[i, 1]), self.tool_size, 
                              color='green', alpha=0.1, linewidth=0)
                ax2.add_patch(circle)
            
            # Plot the continuous path
            ax2.plot(self.uv_path[:, 0], self.uv_path[:, 1], 'b-', linewidth=1, alpha=0.7)
            
            # Add coverage info
            coverage_text = f'Tool radius: {self.tool_size:.2f}\n'
            coverage_text += f'Line spacing: {self.line_spacing:.2f}\n'
            coverage_text += f'Point spacing: {self.point_spacing:.2f}\n'
            coverage_text += f'Max gap (U): {self.line_spacing:.2f}\n'
            coverage_text += f'Max gap (V): {self.point_spacing:.2f}'
            
            ax2.text(0.02, 0.98, coverage_text, transform=ax2.transAxes,
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                    fontsize=9, family='monospace')
            
            ax2.set_title('Tool Coverage Visualization')
            ax2.set_xlabel('U')
            ax2.set_ylabel('V')
            ax2.axis('equal')
            ax2.grid(True)
            
            plt.tight_layout()
            plt.show()
            
        except ImportError:
            self.get_logger().error('matplotlib not installed, cannot plot UV points and path.')
        except Exception as e:
            self.get_logger().error(f'Error in test_plot: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def testing_mode(self):
        """Test mode with synthetic UV bounds - validates path generation without parameterization node."""
        self.get_logger().info('RUNNING IN TEST MODE')
        
        # Create realistic UV bounds (typical range 0-100)
        self.uv_bounds = {
            'u_min': 0.0,
            'u_max': 200.0,
            'v_min': 0.0,
            'v_max': 100.0
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
        self.test_plot()

    def status_callback(self, msg):
        """Callback for parameterization status."""
        previously_ready = self.parameterization_ready
        self.parameterization_ready = msg.is_ready and msg.num_points > 0
        
        if not self.parameterization_ready:
            if previously_ready:
                self.get_logger().warn('Parameterization no longer ready')
            return
        
        if not previously_ready and self.auto_generate:
            self.get_logger().info(f'Parameterization ready ({msg.num_points} points). Fetching UV bounds...')
            if self.bounds_client.wait_for_service(timeout_sec=1.0):
                future = self.bounds_client.call_async(GetUVBounds.Request())
                future.add_done_callback(self._process_uv_bounds)
            else:
                self.get_logger().error('UV bounds service not available')

    def _extract_boundary(self, response):
        """Extract boundary points from service response."""
        if response.boundary and len(response.boundary.points) > 0:
            self.uv_boundary = np.array([[pt.u, pt.v] for pt in response.boundary.points])
            self.get_logger().info(f'UV boundary: {len(self.uv_boundary)} points')
        else:
            self.uv_boundary = np.array([
                [response.u_min, response.v_min], [response.u_max, response.v_min],
                [response.u_max, response.v_max], [response.u_min, response.v_max]
            ])
            self.get_logger().info('Using rectangular bounds')

    def _process_uv_bounds(self, future):
        """Process UV bounds response and generate path."""
        try:
            response = future.result()
            if response.success:
                self._extract_boundary(response)
                self.generate_lines()
                self.adjust_lines()






                self.generate_zigzag_paths()
                self.publish_path()
            else:
                self.get_logger().error(f'Failed to get UV bounds: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Error processing UV bounds: {e}')

    def fetch_uv_bounds(self):
        """Fetch UV bounds synchronously. Returns True if successful."""
        if not self.bounds_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('UV bounds service not available')
            return False
        
        future = self.bounds_client.call_async(GetUVBounds.Request())
        
        import time
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not future.done():
            self.get_logger().error('Service call timed out')
            return False
        
        try:
            response = future.result()
            if response.success:
                self._extract_boundary(response)
                return True
            self.get_logger().error(f'Failed to get UV bounds: {response.message}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error fetching UV bounds: {e}')
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
                        b1 = end + self.tool_size * vec_curr / norm_curr
                        b2 = next_start - self.tool_size * vec_next / norm_next
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
            



            # make an array that is true with the same length as uv_path                    delete this later
            on_surface = np.ones(len(uv_path), dtype=bool)
            on_surface_msg = Float64MultiArray()
            on_surface_msg.data = on_surface.astype(np.float64).tolist()
            self.on_surface_pub.publish(on_surface_msg)





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

