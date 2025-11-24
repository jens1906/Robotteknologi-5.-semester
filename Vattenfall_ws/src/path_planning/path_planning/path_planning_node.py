import numpy as np
from shapely.geometry import Polygon, Point

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
<<<<<<< Updated upstream
        self.declare_parameter('line_spacing', 50.0)  # Spacing between scan lines (U direction) - mm
=======
        self.declare_parameter('line_spacing', 25.0)  # Spacing between scan lines (U direction) - mm
>>>>>>> Stashed changes
        self.declare_parameter('n_bezier', 50)  # Number of points for generating connecting curves - n
        self.declare_parameter('auto_generate', True)  # Automatically generate path when parameterization is ready

        # Load parameters
        self.point_spacing = self.get_parameter('point_spacing').value
        self.line_spacing = self.get_parameter('line_spacing').value
        self.n_bezier = self.get_parameter('n_bezier').value
        self.auto_generate = self.get_parameter('auto_generate').value
        self.tau = np.linspace(0, 1, self.n_bezier)

        # Data holders
        self.uv_boundary = None
        self.paths_uv = None
        self.on_surface = None
        self.parameterization_ready = False
        
        # Service client
        self.bounds_client = self.create_client(
            GetUVBounds,
            '/parameterization/get_uv_bounds'
        )

        # Subscriptions
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

    def status_callback(self, msg):
        """Callback for parameterization status."""
        previously_ready = self.parameterization_ready
        self.parameterization_ready = msg.is_ready and msg.num_points > 0
        
        if not self.parameterization_ready:
            if previously_ready:
                self.get_logger().warn('Parameterization no longer ready')
            return
        
        if previously_ready or not self.auto_generate:
            return
            
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
                self.generate_lines()  # This calls adjust_lines() internally
                self.uv_path = self.create_continuous_path()
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
                self.generate_lines()
                self.uv_path = self.create_continuous_path()
                self.publish_path()
                return True
            else:
                self.get_logger().error('Failed to fetch UV bounds')
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error in generate_path: {e}")
            return False

    def adjust_lines(self):
        """Mark which points are on/off surface"""
        polygon = Polygon(self.uv_boundary)
        self.on_surface = [
            np.array([polygon.contains(Point(u, v)) for u, v in zip(u_line, v_line)])
            for u_line, v_line in zip(self.lines[0], self.lines[1])
        ]

    def generate_lines(self):
        """Generate alternating-direction scan lines"""
        u_min, u_max = self.uv_bounds['u_min'], self.uv_bounds['u_max']
        v_min, v_max = self.uv_bounds['v_min'], self.uv_bounds['v_max']

        # Apply tool_size/2 offset from borders
        offset = self.tool_size / 2
        u_min_offset = u_min + offset
        u_max_offset = u_max - offset
        v_min_offset = v_min + offset
        v_max_offset = v_max - offset

        line_n = int(np.ceil((v_max_offset - v_min_offset) / self.line_spacing)) + 1
        points_per_line = int(np.ceil((u_max_offset - u_min_offset) / self.point_spacing)) + 1

        v_lines_pos = np.linspace(v_min_offset, v_max_offset, line_n)
        u_base = np.linspace(u_min_offset, u_max_offset, points_per_line)

        u_lines, v_lines = [], []
        for i, v_pos in enumerate(v_lines_pos):
            # Alternate direction: reverse on odd lines if line_n is odd, even lines if line_n is even
            should_reverse = (i % 2 == 1) != (line_n % 2 == 0)
            u_lines.append(u_base[::-1] if should_reverse else u_base.copy())
            v_lines.append(np.full(points_per_line, v_pos))

        self.lines = (u_lines, v_lines)
        self.adjust_lines()

    def cubic_bezier(self, b0, b1, b2, b3):
        """Cubic Bézier curve."""
        t = self.tau[:, None]
        return (1-t)**3 * b0 + 3*(1-t)**2 * t * b1 + 3*(1-t)*t**2 * b2 + t**3 * b3

    def create_continuous_path(self):
        """Create continuous path with Bézier smoothing."""
        try:
            if self.lines is None:
                raise ValueError("Error: create_continuous_path(): Line data is not available.")
        
            path = []
            on_surface = []
            n_lines = len(self.lines[0])

            for i in range(n_lines):
                u_line, v_line = self.lines[0][i], self.lines[1][i]
                
                # Add line - don't reverse the line, the zigzag is already in the data
                path.append(np.column_stack([u_line, v_line]))
                
                # Add corresponding on_surface flags if available
                if self.on_surface is not None and i < len(self.on_surface):
                    on_surface.extend(self.on_surface[i])
                else:
                    on_surface.extend([True] * len(u_line))

                # Add Bézier curve to next line
                if i < n_lines - 1:
                    end = np.array([u_line[-1], v_line[-1]])
                    next_u, next_v = self.lines[0][i+1], self.lines[1][i+1]
                    next_start = np.array([next_u[0], next_v[0]])

                    # Calculate direction vectors for smooth Bezier transition
                    vec_curr = (end - np.array([u_line[-2], v_line[-2]])) if len(u_line) > 1 else np.array([1.0, 0.0])
                    vec_next = (np.array([next_u[1], next_v[1]]) - next_start) if len(next_u) > 1 else np.array([1.0, 0.0])
                    
                    norm_curr, norm_next = np.linalg.norm(vec_curr), np.linalg.norm(vec_next)

                    if norm_curr > 1e-6 and norm_next > 1e-6:
                        bezier_curve = self.cubic_bezier(
                            end,
                            end + self.tool_size * vec_curr / norm_curr,
                            next_start - self.tool_size * vec_next / norm_next,
                            next_start
                        )
                        path.append(bezier_curve)
                        on_surface.extend([False] * len(bezier_curve))

            # Store on_surface flags for use in publish_path
            self.continuous_on_surface = np.array(on_surface)
            
            return np.vstack(path)
    
        except Exception as e:
            self.get_logger().error(f"Error in create_continuous_path: {str(e)}")
            return None


    def publish_path(self):
        """Publish UV path and on-surface flags."""
        if self.uv_path is None:
            self.get_logger().error('Cannot publish path: UV path not generated')
            return
        
        # Publish UV path
        uv_msg = Float64MultiArray(data=self.uv_path.flatten().tolist())
        self.uv_path_pub.publish(uv_msg)
        self.get_logger().info(f'Published UV path with {len(self.uv_path)} points')

        # Publish on-surface flags
        if hasattr(self, 'continuous_on_surface'):
            on_surface_msg = Float64MultiArray(data=self.continuous_on_surface.astype(float).tolist())
            self.on_surface_pub.publish(on_surface_msg)
            self.get_logger().info('Published on-surface flags')
        else:
            self.get_logger().warn('On-surface flags not available')

def main():
    try:
        rclpy.init()
        rclpy.spin(PathPlanner())
        rclpy.shutdown()
    
    except Exception as e:
        print(f"Path - Error: main(): {e}")

if __name__ == '__main__':
    main()
