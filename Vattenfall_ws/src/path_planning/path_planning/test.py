#!/usr/bin/env python3
"""
Test script for path planning node.
Runs path planning with synthetic data and visualization.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from shapely.geometry import Polygon, Point


class PathPlannerTest:
    """Test path planning algorithms without ROS"""
    
    def __init__(self):
        # Parameters
        self.point_spacing = 5  # mm
        self.line_spacing = 45.0  # mm (2 * tool_size - 5)
        self.tool_size = 25  # mm
        self.n_bezier = 50
        self.tau = np.linspace(0, 1, self.n_bezier)
        
        # Data holders
        self.uv_bounds = None
        self.uv_boundary = None
        self.lines = None
        self.on_surface = None
        self.uv_path = None
        self.continuous_on_surface = None
    
    def setup_test_data(self):
        """Create synthetic UV bounds and boundary"""
        self.uv_bounds = {
            'u_min': 0.0,
            'u_max': 200.0,
            'v_min': 0.0,
            'v_max': 100.0
        }
        
        print(f'Test UV bounds: U=[{self.uv_bounds["u_min"]}, {self.uv_bounds["u_max"]}], '
              f'V=[{self.uv_bounds["v_min"]}, {self.uv_bounds["v_max"]}]')
        
        # Create rectangular boundary
        self.uv_boundary = np.array([
            [self.uv_bounds['u_min'], self.uv_bounds['v_min']],
            [self.uv_bounds['u_max'], self.uv_bounds['v_min']],
            [self.uv_bounds['u_max'], self.uv_bounds['v_max']],
            [self.uv_bounds['u_min'], self.uv_bounds['v_max']]
        ])
    
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
            should_reverse = (i % 2 == 1) != (line_n % 2 == 0)
            u_lines.append(u_base[::-1] if should_reverse else u_base.copy())
            v_lines.append(np.full(points_per_line, v_pos))

        self.lines = (u_lines, v_lines)
        self.adjust_lines()
    
    def adjust_lines(self):
        """Mark which points are on/off surface"""
        polygon = Polygon(self.uv_boundary)
        self.on_surface = [
            np.array([polygon.contains(Point(u, v)) for u, v in zip(u_line, v_line)])
            for u_line, v_line in zip(self.lines[0], self.lines[1])
        ]
    
    def cubic_bezier(self, b0, b1, b2, b3):
        """Cubic Bézier curve"""
        t = self.tau[:, None]
        return (1-t)**3 * b0 + 3*(1-t)**2 * t * b1 + 3*(1-t)*t**2 * b2 + t**3 * b3
    
    def create_continuous_path(self):
        """Create continuous path with Bézier smoothing"""
        path = []
        on_surface = []
        n_lines = len(self.lines[0])

        for i in range(n_lines):
            u_line, v_line = self.lines[0][i], self.lines[1][i]
            
            path.append(np.column_stack([u_line, v_line]))
            
            if self.on_surface is not None and i < len(self.on_surface):
                on_surface.extend(self.on_surface[i])
            else:
                on_surface.extend([True] * len(u_line))

            # Add Bézier curve to next line
            if i < n_lines - 1:
                end = np.array([u_line[-1], v_line[-1]])
                next_u, next_v = self.lines[0][i+1], self.lines[1][i+1]
                next_start = np.array([next_u[0], next_v[0]])

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

        self.continuous_on_surface = np.array(on_surface)
        return np.vstack(path)
    
    def plot_results(self):
        """Plot bounds, path, and tool coverage"""
        if self.uv_bounds is None or self.lines is None or self.uv_path is None:
            print('ERROR: Cannot plot - missing data')
            return
        
        u_min = self.uv_bounds['u_min']
        u_max = self.uv_bounds['u_max']
        v_min = self.uv_bounds['v_min']
        v_max = self.uv_bounds['v_max']
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        
        # Left plot: Path visualization
        ax1.plot([u_min, u_max, u_max, u_min, u_min],
                 [v_min, v_min, v_max, v_max, v_min], 'k--', linewidth=2, label='UV Bounds')
        
        for u_line, v_line in zip(self.lines[0], self.lines[1]):
            ax1.plot(u_line, v_line, 'b.-', alpha=0.5)
        
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
                    
        step = max(1, len(self.uv_path) // 100)
        for i in range(0, len(self.uv_path), step):
            circle = Circle((self.uv_path[i, 0], self.uv_path[i, 1]), self.tool_size, 
                          color='green', alpha=0.1, linewidth=0)
            ax2.add_patch(circle)
        
        ax2.plot(self.uv_path[:, 0], self.uv_path[:, 1], 'b-', linewidth=1, alpha=0.7)
        
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
    
    def run_test(self):
        """Run complete test"""
        print('='*60)
        print('PATH PLANNING TEST')
        print('='*60)
        
        # Setup
        self.setup_test_data()
        
        # Generate lines
        print('Generating scan lines...')
        self.generate_lines()
        
        if self.lines is None:
            print('ERROR: Line generation failed')
            return
        
        u_lines, v_lines = self.lines
        expected = max(2, int(np.ceil((self.uv_bounds['v_max'] - self.uv_bounds['v_min']) / self.line_spacing)) + 1)
        
        if len(u_lines) != expected:
            print(f'WARNING: Line count mismatch - expected ~{expected}, got {len(u_lines)}')
        
        # Create continuous path
        print('Creating continuous path with Bézier curves...')
        self.uv_path = self.create_continuous_path()
        
        # Summary
        print('\n' + '='*60)
        print('TEST SUMMARY')
        print('='*60)
        print(f'  ✓ Generated {len(u_lines)} scan lines')
        print(f'  ✓ {len(v_lines[0])} points per line')
        print(f'  ✓ Total waypoints: {sum(len(line) for line in u_lines)}')
        print(f'  ✓ Continuous path points: {len(self.uv_path)}')
        
        if self.continuous_on_surface is not None:
            on_count = np.sum(self.continuous_on_surface)
            off_count = np.sum(~self.continuous_on_surface)
            print(f'  ✓ On surface: {on_count} points ({100*on_count/len(self.continuous_on_surface):.1f}%)')
            print(f'  ✓ Off surface: {off_count} points ({100*off_count/len(self.continuous_on_surface):.1f}%)')
        
        print('='*60)
        print('Test complete!')
        print('='*60)
        
        # Plot
        self.plot_results()


if __name__ == '__main__':
    test = PathPlannerTest()
    test.run_test()
