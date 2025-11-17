import numpy as np
import matplotlib.pyplot as plt


def cubic_bezier(b0, b1, b2, b3, n_points=25):
    """Cubic Bézier curve - standalone testable function."""
    tau = np.linspace(0, 1, n_points)[:, None]
    return (1-tau)**3 * b0 + 3*(1-tau)**2 * tau * b1 + 3*(1-tau)*tau**2 * b2 + tau**3 * b3


def generate_zigzag_paths(uv_bounds, point_spacing, line_spacing):
    """Generate zigzag paths - standalone testable function."""
    u_min = uv_bounds['u_min']
    u_max = uv_bounds['u_max']
    v_min = uv_bounds['v_min']
    v_max = uv_bounds['v_max']

    # Calculate number of lines based on line_spacing to ensure full coverage
    u_range = u_max - u_min
    line_n = int(np.ceil(u_range / (2 * line_spacing))) + 1
    
    if line_n < 2:
        raise ValueError(f"line_spacing {line_spacing} is too large for U range {u_range}")

    # Calculate number of points per line based on point_spacing
    v_range = v_max - v_min
    points_per_line = int(np.ceil(v_range / (2 * point_spacing))) + 1

    # Use linspace to ensure coverage from u_min to u_max
    u_lin = np.linspace(u_min, u_max, line_n)
    v_lin = np.linspace(v_min, v_max, points_per_line)

    u_lines, v_lines = [], []

    # If odd number of lines, start from max to ensure ending at top-right
    start_reversed = (line_n % 2 == 0)
    
    for i, u in enumerate(u_lin):
        # Alternate direction for zigzag
        should_reverse = (i % 2 == 1) if not start_reversed else (i % 2 == 0)
        v_line = v_lin[::-1] if should_reverse else v_lin
            
        u_lines.append(np.full_like(v_line, u))
        v_lines.append(v_line)

    return (u_lines, v_lines)


def create_continuous_path(paths_uv, bezier_curvature, n_bezier=25):
    """Create continuous path with Bézier smoothing - standalone testable function."""
    path = []
    n_lines = len(paths_uv[0])

    for i in range(n_lines):
        u_line, v_line = paths_uv[0][i], paths_uv[1][i]
        
        # Add line
        path.append(np.column_stack([u_line, v_line]))

        # Add Bézier curve to next line
        if i < n_lines - 1:
            end = np.array([u_line[-1], v_line[-1]])
            next_u, next_v = paths_uv[0][i+1], paths_uv[1][i+1]
            next_start = np.array([next_u[0], next_v[0]])

            # Calculate direction vectors for smooth transition
            if len(u_line) > 1:
                vec_curr = end - np.array([u_line[-2], v_line[-2]])
            else:
                vec_curr = np.array([1.0, 0.0])
            
            if len(next_u) > 1:
                vec_next = np.array([next_u[1], next_v[1]]) - next_start
            else:
                vec_next = np.array([1.0, 0.0])
            
            norm_curr = np.linalg.norm(vec_curr)
            norm_next = np.linalg.norm(vec_next)

            if norm_curr > 1e-6 and norm_next > 1e-6:
                b0 = end
                b1 = end + bezier_curvature * vec_curr / norm_curr
                b2 = next_start - bezier_curvature * vec_next / norm_next
                b3 = next_start

                path.append(cubic_bezier(b0, b1, b2, b3, n_bezier))

    return np.vstack(path)


def test_path_generation():
    """Test that UV bounds input produces valid 2D path output."""
    print("\n=== Test: UV Input → 2D Path Output ===")
    
    # Input: UV bounds (dictionary with min/max values)
    point_spacing = 0.5
    line_spacing = 1.0
    n_bezier = 25
    bezier_curvature = line_spacing * 1.25
    uv_bounds = {'u_min': 0.0, 'u_max': 10.0, 'v_min': 0.0, 'v_max': 10.0}
    
    print(f"Input: UV bounds {uv_bounds}")
    
    # Generate zigzag paths
    paths_uv = generate_zigzag_paths(uv_bounds, point_spacing, line_spacing)
    
    # Create continuous path
    path = create_continuous_path(paths_uv, bezier_curvature, n_bezier)
    
    # Output: Validate it's a 2D path (N×2 array of points)
    assert path is not None, "Path should not be None"
    assert isinstance(path, np.ndarray), "Path should be a numpy array"
    assert len(path.shape) == 2, "Path should be 2D array"
    assert path.shape[1] == 2, "Path should have 2 columns [u, v]"
    assert path.shape[0] > 0, "Path should have points"
    
    print(f"✓ Output: 2D path with shape {path.shape} ({path.shape[0]} points)")
    print(f"✓ First point: [{path[0, 0]:.3f}, {path[0, 1]:.3f}]")
    print(f"✓ Last point:  [{path[-1, 0]:.3f}, {path[-1, 1]:.3f}]")
    print("✓ Test passed!")
    
    # Plot the path
    plot_path(path, uv_bounds)
    
    return path


def plot_path(path, uv_bounds):
    """Plot the 2D path and save to file."""
    plt.figure(figsize=(10, 8))
    
    # Plot path
    plt.plot(path[:, 0], path[:, 1], 'b-', linewidth=1.5, label='Path')
    plt.plot(path[0, 0], path[0, 1], 'go', markersize=10, label='Start')
    plt.plot(path[-1, 0], path[-1, 1], 'ro', markersize=10, label='End')
    
    # Plot UV bounds rectangle
    u_min, u_max = uv_bounds['u_min'], uv_bounds['u_max']
    v_min, v_max = uv_bounds['v_min'], uv_bounds['v_max']
    plt.plot([u_min, u_max, u_max, u_min, u_min], 
             [v_min, v_min, v_max, v_max, v_min], 
             'k--', linewidth=1, alpha=0.5, label='UV Bounds')
    
    plt.xlabel('U')
    plt.ylabel('V')
    plt.title('2D Path Planning in UV Space')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    
    # Save to file
    output_file = 'path_planning_output.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✓ Plot saved to: {output_file}")
    plt.close()


if __name__ == '__main__':
    print("=" * 50)
    print("Path Planning Test: UV Input → 2D Path Output")
    print("=" * 50)
    
    try:
        result_path = test_path_generation()
        print("\n" + "=" * 50)
        print("✓ SUCCESS: Test passed")
        print("=" * 50)
        exit(0)
    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        exit(1)
    except Exception as e:
        print(f"\n✗ Test error: {e}")
        exit(1)
