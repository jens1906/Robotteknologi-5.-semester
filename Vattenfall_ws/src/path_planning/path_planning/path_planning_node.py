import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray

class PathPlanner:
    """
    Plan zigzag paths in UV space and create continuous paths with Bézier smoothing.
    """
    def __init__(self): # uv format u = uv[0], v = uv[1]
        """
        uv: instance of Parameterization of XYZ surface
        """
        self.d = 0.05
        self.n_bezier = 50
        self.line_n = 10

        self.path = None
        self.path_uv = self.generate_continuous_path()

        # ROS Publishers & Subscribers
        self.uv_path_pub = self.create_publisher(Float64MultiArray, 'path_uv', 10)
        self.uv = self.create_subscription(Float64MultiArray, 'param_uv', 10)
    

    @staticmethod
    def cubic_bezier(tau, b0, b1, b2, b3):
        """Compute points on a cubic Bézier curve."""
        return (1-tau)**3 * b0 + 3*(1-tau)**2 * tau * b1 + 3*(1-tau)*tau**2 * b2 + tau**3 * b3

    def get_start_end(self, i):
        """Return start and end points of a line."""
        u_line, v_line = self.paths_uv[0][i], self.paths_uv[1][i]
        start = np.array([u_line[0], v_line[0]])
        end = np.array([u_line[-1], v_line[-1]])
        return start, end

    def generate_zigzag_paths(self):
        """Generate discrete zigzag lines in UV space."""
        u_min, u_max = np.min(self.uv[0]), np.max(self.uv[0])
        v_min, v_max = np.min(self.uv[1]), np.max(self.uv[1])

        u_lin = np.linspace(u_min, u_max, self.line_n)
        v_lin = np.linspace(v_min, v_max, self.line_n)

        u_lines = []
        v_lines = []

        for i, u in enumerate(u_lin):
            if i % 2 == 0:
                v_line = v_lin
            else:
                v_line = v_lin[::-1]
            u_line = np.full_like(v_line, u)
            u_lines.append(u_line)
            v_lines.append(v_line)

        self.paths_uv = (u_lines, v_lines)
        return self.paths_uv

    def create_continuous_path(self):
        """
        Create a continuous path connecting lines using cubic Bézier curves.
        Uses self.paths_uv, self.d, and self.n_bezier from instance variables.
        """
        b = self.d / 2
        tau = np.linspace(0, 1, self.n_bezier)
        self.path = []

        for i in range(len(self.paths_uv[0])):
            start, end = self.get_start_end(i)

            # Zigzag reversal
            if i % 2 == 0:
                line_points = np.column_stack([self.paths_uv[0][i], self.paths_uv[1][i]])
            else:
                line_points = np.column_stack([self.paths_uv[0][i][::-1], self.paths_uv[1][i][::-1]])

            self.path.extend(line_points)

            # Bézier connection to next line
            if i < len(self.paths_uv[0]) - 1:
                next_start, _ = self.get_start_end(i + 1)

                vec_curr = end - start
                vec_next = next_start - end
                norm_curr = np.linalg.norm(vec_curr)
                norm_next = np.linalg.norm(vec_next)

                if norm_curr > 0 and norm_next > 0:
                    rho_curr = vec_curr / norm_curr
                    rho_next = vec_next / norm_next

                    b0 = end
                    b1 = end + b * rho_curr
                    b2 = next_start - b * rho_next
                    b3 = next_start

                    self.path.extend(self.cubic_bezier(tau[:, None], b0, b1, b2, b3))

        self.path = np.array(self.path)
        return self.path

    def generate_continuous_path(self):
        """Generate a continuous 3D path using Bézier smoothing."""
        if self.paths_uv is None:
            self.generate_zigzag_paths()
        self.path_uv = self.create_continuous_path()

        # publish UV path
        


        return self.path_uv

def main():
    rclpy.init()
    node = PathPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
