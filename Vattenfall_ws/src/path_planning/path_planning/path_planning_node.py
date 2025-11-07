import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


"""

The functionality of the Path Planner Node

- Subscribes to UV parameterization data.
- Generates zigzag paths in UV space with interconnection between Bézier curves.
- Publishes the generated UV path data.

Subscribers:
- /parameterization/param_uv (Float64MultiArray): UV parameterization data.

Publishers:
- /path/uv_path (Float64MultiArray): Generated UV path data.

"""


class PathPlanner(Node):
    """ Path Planner"""
    
    def __init__(self):
        super().__init__('path_planner_node') # Initialize ROS2 node

        # Parameters
        self.d = 0.05
        self.n_bezier = 50
        self.line_n = 10
        self.bezier_curviture = self.d / 2
        self.tau = np.linspace(0, 1, self.n_bezier)

        # Data holders
        self.uv_data = None
        self.paths_uv = None

        # Subscribe to UV parameterization data
        self.create_subscription(Float64MultiArray, '/parameterization/param_uv', self.uv_callback, 10)
        self.create_subscription(Float64MultiArray, '/parameterization/param_uv', self.uv_callback, 10)


        # Publisher for UV path
        self.uv_path_pub = self.create_publisher(Float64MultiArray, '/path/xyz_path', 10)


    def uv_callback(self, msg):
        """Receive UV data and generate path."""
        try:
            self.uv_data = np.array(msg.data).reshape(-1, 2)
            self.generate_zigzag_paths()
            self.publish_path()
        except Exception as e:
            self.get_logger().error(f"Error: uv_callback(): {e}")

    def cubic_bezier(self, b0, b1, b2, b3):
        """Cubic Bézier curve."""
        t = self.tau[:, None]
        return (1-t)**3 * b0 + 3*(1-t)**2 * t * b1 + 3*(1-t)*t**2 * b2 + t**3 * b3

    def generate_zigzag_paths(self):
        """Generate zigzag lines in UV space."""
        try:
            if self.uv_data is None or len(self.uv_data) == 0:
                raise ValueError("Error: generate_zigzag_paths(): UV data is not available or empty.")
            
            u_min, u_max = self.uv_data[:, 0].min(), self.uv_data[:, 0].max()
            v_min, v_max = self.uv_data[:, 1].min(), self.uv_data[:, 1].max()

            if self.line_n < 3:
                raise ValueError("Error: generate_zigzag_paths(): line_n must be at least 3")

            u_lin = np.linspace(u_min, u_max, self.line_n)
            v_lin = np.linspace(v_min, v_max, self.line_n)

            u_lines, v_lines = [], []

            for i, u in enumerate(u_lin):
                v_line = v_lin if i % 2 == 0 else v_lin[::-1]
                u_lines.append(np.full_like(v_line, u))
                v_lines.append(v_line)

            self.paths_uv = (u_lines, v_lines)

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
                
                # Add line (reversed every other iteration)
                if i % 2 == 0:
                    path.append(np.column_stack([u_line, v_line]))
                else:
                    path.append(np.column_stack([u_line[::-1], v_line[::-1]]))

                # Add Bézier curve to next line
                if i < n_lines - 1:
                    end = np.array([u_line[-1], v_line[-1]])
                    next_u, next_v = self.paths_uv[0][i+1], self.paths_uv[1][i+1]
                    next_start = np.array([next_u[0], next_v[0]])

                    vec_curr = end - np.array([u_line[0], v_line[0]])
                    vec_next = next_start - end
                    
                    norm_curr = np.linalg.norm(vec_curr)
                    norm_next = np.linalg.norm(vec_next)

                    if norm_curr > 1e-6 and norm_next > 1e-6:
                        b0 = end
                        b1 = end + self.bezier_curviture * vec_curr / norm_curr
                        b2 = next_start - self.bezier_curviture * vec_next / norm_next
                        b3 = next_start

                        path.append(self.cubic_bezier(b0, b1, b2, b3))

            return np.vstack(path)
    
        except Exception as e:
            self.get_logger().error(f"Error in create_continuous_path: {str(e)}")
            return None

    def publish_path(self):
        """Generate and publish path."""
        try:
            path = self.create_continuous_path()
            if path is None:
                raise ValueError("Error: publish_path(): Path generation failed.")
            
            msg = Float64MultiArray()
            msg.data = path.flatten().tolist()
            self.uv_path_pub.publish(msg)
        
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

