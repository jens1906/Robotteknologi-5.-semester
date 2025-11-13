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
- Publishes the generated UV path data.

Subscribers:
- /parameterization/status (ParameterizationStatus): Parameterization status and readiness.

Publishers:
- /path/uv_path (Float64MultiArray): Generated UV path data.

Services:
- /parameterization/get_uv_bounds (GetUVBounds): Get UV parameter space bounds.

"""


testing = True

def testingmode():
    # create large uv_data array for testing
    uv_data = np.random.rand(1000, 2)

    #plot uv_data
    import matplotlib.pyplot as plt
    plt.scatter(uv_data[:, 0], uv_data[:, 1], color='red')
    plt.title('UV Data Points')
    plt.xlabel('U')
    plt.ylabel('V')
    plt.grid()
    plt.show()

testingmode()






class PathPlanner(Node):
    """ Path Planner"""
    
    def __init__(self):
        super().__init__('path_planner_node') # Initialize ROS2 node

        # Parameters
        self.declare_parameter('spacing', 0.05)
        self.declare_parameter('n_bezier', 50)
        self.declare_parameter('line_n', 10)
        self.declare_parameter('auto_generate', True)
        
        self.d = self.get_parameter('spacing').value
        self.n_bezier = self.get_parameter('n_bezier').value
        self.line_n = self.get_parameter('line_n').value
        self.auto_generate = self.get_parameter('auto_generate').value
        self.bezier_curviture = self.d / 2
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
        self.create_subscription(
            ParameterizationStatus,
            '/parameterization/status',
            self.status_callback,
            10
        )

        # Publishers
        self.uv_path_pub = self.create_publisher(Float64MultiArray, '/path/uv_path', 10)
        
        self.get_logger().info('Path Planner node initialized')
        self.get_logger().info(f'  Spacing: {self.d}')
        self.get_logger().info(f'  Bezier points: {self.n_bezier}')
        self.get_logger().info(f'  Number of lines: {self.line_n}')
        self.get_logger().info(f'  Auto-generate: {self.auto_generate}')


    def status_callback(self, msg):
        """
        Callback for parameterization status.
        When parameterization is ready, fetch UV bounds and generate path.
        """
        try:
            # Update readiness status
            was_ready = self.parameterization_ready
            self.parameterization_ready = msg.is_ready
            
            # Only process if we have valid data (num_points > 0)
            if not msg.is_ready or msg.num_points == 0:
                if was_ready:
                    self.get_logger().warn('Parameterization no longer ready')
                self.parameterization_ready = False
                return
            
            # If just became ready, generate path
            if self.parameterization_ready and not was_ready:
                self.get_logger().info(f'Parameterization is ready with {msg.num_points} points. Fetching UV bounds...')
                
                if self.auto_generate:
                    # Initiate async service call
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
            self.get_logger().info(f'Generated zigzag path with {len(u_lines)} lines')

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

#if __name__ == '__main__':
#    main()

