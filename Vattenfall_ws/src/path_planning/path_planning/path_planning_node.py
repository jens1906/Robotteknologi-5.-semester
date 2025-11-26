import numpy as np
from shapely.geometry import Polygon, Point

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, ByteMultiArray, Bool, Float32MultiArray

"""
Path Planner Node

Functionality:
- Subscribes to parameterization status to know when it's ready.
- Uses the GetUVBounds service to get UV parameter space bounds.
- Generates zigzag scan lines in UV space with Bézier transitions between lines.
- Publishes the generated UV path data and on-surface flags for downstream conversion.

Subscribers:
- /parameterization/status (ParameterizationStatus): Parameterization status and readiness.
- /corrosion/tool_size (Float64MultiArray): Tool size updates (used to set line spacing and offsets).

Publishers:
- /path/uv_path (Float64MultiArray): Generated UV path data (flattened Nx2 array).
- /path/on_surface (ByteMultiArray): Corresponding on-surface flags (0/1 as float).

Services Used:
- /parameterization/get_uv_bounds (GetUVBounds): Get UV parameter space bounds.

Parameters (declared defaults in node):
- point_spacing (float): Spacing between points along scan lines (V direction) in UV units (default: 5).
- line_spacing (float): Spacing between parallel scan lines (U direction) in UV units (default: 25.0).
    Note: line_spacing will be updated to 2 * tool_size when a tool size message is received.
- n_bezier (int): Number of points in Bézier curves connecting lines (default: 50).

Notes:
- The node applies a tool_size/2 offset from bounds when generating lines.
- on_surface flags are computed using shapely Polygon.contains for the UV boundary.
"""

class PathPlanner(Node):
    """ Path Planner"""
    
    def __init__(self, point_spacing=None, line_spacing=None, n_bezier=None, 
                 uv_bounds=None, uv_boundary=None, tool_size=None, test_active=False):
        """
        Initialize PathPlanner.
        
        Args:
            point_spacing: Spacing between points along lines (V direction) in mm
            line_spacing: Spacing between lines (U direction) in mm
            n_bezier: Number of n points to make the Bézier curves
            tool_size: Tool size in mm
            test_active: If True, do not initialize ROS (for testing)
        """
        self.test_active = test_active
        
        if not test_active:
            super().__init__('path_planner_node') # Initialize ROS2 node

            self.declare_parameter('point_spacing', 1)
            self.declare_parameter('line_spacing', 25.0)
            self.declare_parameter('n_bezier', 50)

            # Load parameters from ROS if not provided
            self.point_spacing = point_spacing if point_spacing is not None else self.get_parameter('point_spacing').value
            self.line_spacing = line_spacing if line_spacing is not None else self.get_parameter('line_spacing').value
            self.n_bezier = n_bezier if n_bezier is not None else self.get_parameter('n_bezier').value
        else:
            # Test mode - use provided values or defaults
            self.point_spacing = point_spacing if point_spacing is not None else 1
            self.line_spacing = line_spacing if line_spacing is not None else 25.0
            self.n_bezier = n_bezier if n_bezier is not None else 50
        
        self.tau = np.linspace(0, 1, self.n_bezier)

        # Data holders
        self.uv_boundary = uv_boundary  # Detected area with corrosion
        self.uv_bounds = uv_bounds  # Bounds of the boundary
        self.uv_path = None # Final path in UV space
        self.on_surface = None # Flags for points on/off surface
        self.parameterization_ready = test_active # True when the module should generate path
        self.tool_size = tool_size
        self.received_bounds = uv_bounds is not None # True if uv_bounds has been received
        self.received_boundary = uv_boundary is not None # True if uv_boundary has been received
        self.path_generated = False  # Track if path has been generated for current data

        if not test_active:
            # Subscriptions
            self.create_subscription(
                Float32MultiArray,
                '/corrosion/tool_size',
                self.tool_size_callback,
                10
            )

            self.create_subscription(
                Bool,
                '/parameterization/ready',
                self.ready_callback,
                10
            )
            
            self.create_subscription(
                Float64MultiArray,
                '/parameterization/uv_bounds',
                self.uv_bounds_callback,
                10
            )
            
            self.create_subscription(
                Float64MultiArray,
                '/parameterization/uv_boundary',
                self.uv_boundary_callback,
                10
            )

            # Publishers
            self.uv_path_pub = self.create_publisher(
                Float64MultiArray,
                '/path/uv_path',
                10)
            
            self.on_surface_pub = self.create_publisher(
                ByteMultiArray,
                '/path/on_surface',
                10)
                    
            # Debug info
            self.get_logger().info('Path Planner node initialized')
            self.get_logger().info(f'  Point spacing (V direction): {self.point_spacing}')
            self.get_logger().info(f'  Line spacing (U direction): {self.line_spacing}')
            self.get_logger().info(f'  Bezier points: {self.n_bezier}')
        else:
            # Test mode - Info
            print('Path Planner initialized (test_active mode)')
            print(f'  Point spacing (V direction): {self.point_spacing}')
            print(f'  Line spacing (U direction): {self.line_spacing}')
            print(f'  Bezier points: {self.n_bezier}')


    def ready_callback(self, msg):
        """Callback for parameterization ready status."""
        previously_ready = self.parameterization_ready
        self.parameterization_ready = msg.data
        
        if not self.parameterization_ready:
            if previously_ready:
                self.get_logger().warn('Parameterization no longer ready')
            return
        
        if previously_ready:
            return
            
        self.get_logger().info('Parameterization ready. Waiting for bounds and boundary data...')
        self._try_generate_path()


    def uv_bounds_callback(self, msg):
        """Callback for UV bounds data.
        Expected format: [u_min, u_max, v_min, v_max]
        """
        if len(msg.data) != 4:
            self.get_logger().error(f'Invalid UV bounds data length: {len(msg.data)} (expected 4)')
            return
        
        new_bounds = {
            'u_min': msg.data[0],
            'u_max': msg.data[1],
            'v_min': msg.data[2],
            'v_max': msg.data[3]
        }
        
        # Ignore if bounds haven't changed
        if self.uv_bounds == new_bounds:
            return
        
        self.uv_bounds = new_bounds
        self.received_bounds = True
        self.path_generated = False  # New data, need to regenerate path
        self.get_logger().info(f'Received UV bounds: U=[{self.uv_bounds["u_min"]:.3f}, {self.uv_bounds["u_max"]:.3f}], '
                             f'V=[{self.uv_bounds["v_min"]:.3f}, {self.uv_bounds["v_max"]:.3f}]')
        self._try_generate_path()


    def uv_boundary_callback(self, msg):
        """Callback for UV boundary points.
        Expected format: flattened Nx2 array [u1, v1, u2, v2, ...]
        """
        if len(msg.data) == 0:
            self.get_logger().warn('Received empty boundary data')
            return
        
        if len(msg.data) % 2 != 0:
            self.get_logger().error(f'Invalid boundary data length: {len(msg.data)} (must be even)')
            return
        
        # Reshape to Nx2
        self.uv_boundary = np.array(msg.data).reshape(-1, 2)
        self.received_boundary = True
        self.get_logger().info(f'Received UV boundary: {len(self.uv_boundary)} points')
        self._try_generate_path()


    def _try_generate_path(self):
        """Attempt to generate path if all required data is available."""
        # Skip if path already generated for current data
        if self.path_generated:
            return
        
        if not self.parameterization_ready:
            self.get_logger().debug('Waiting for parameterization to be ready...')
            return
        
        if not self.received_bounds:
            self.get_logger().debug('Waiting for UV bounds...')
            return
        
        if not self.received_boundary:
            self.get_logger().debug('Waiting for UV boundary...')
            return
        
        if self.tool_size is None:
            self.get_logger().debug('Waiting for tool size...')
            return
        
        # All data available, generate path
        self.get_logger().info('All data received. Generating path...')
        try:
            self.generate_lines()
            self.create_continuous_path()
            self.publish_path()
            self.path_generated = True  # Mark as generated
        except Exception as e:
            self.get_logger().error(f'Error generating path: {e}')
    

    def tool_size_callback(self, msg):
        """Update tool size and line spacing from corrosion detection (expects data[1] = tool size)."""
        if len(msg.data) > 1:
            self.tool_size = msg.data[1]
            # Line spacing should be 2 * tool_radius for full coverage with overlap
            self.line_spacing = 2 * self.tool_size
            if not self.test_active:
                self.get_logger().info(f'Tool size updated: {self.tool_size:.3f}')
                self.get_logger().info(f'Line spacing updated: {self.line_spacing:.3f}')
            else:
                print(f'Tool size updated: {self.tool_size:.3f}')
                print(f'Line spacing updated: {self.line_spacing:.3f}')


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

        if self.tool_size is None:
            self.get_logger().error('Tool size not set. Cannot generate lines.')
            return

        # Apply tool_size/2 offset from borders
        offset = self.tool_size / 2
        v_min_offset = v_min + offset
        v_max_offset = v_max - offset

        line_n = int(np.ceil((v_max_offset - v_min_offset) / self.line_spacing)) + 1
        points_per_line = int(np.ceil((u_max - u_min) / self.point_spacing)) + 1

        v_lines_pos = np.linspace(v_max_offset, v_min_offset, line_n)
        u_base = np.linspace(u_min, u_max, points_per_line)

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
                
                # Add line - direction is already handled in line generation
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
            self.uv_path = np.vstack(path)
            
    
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
            on_surface_msg = ByteMultiArray()
            on_surface_msg.data = bytes(self.continuous_on_surface.astype(np.uint8).tolist())
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
