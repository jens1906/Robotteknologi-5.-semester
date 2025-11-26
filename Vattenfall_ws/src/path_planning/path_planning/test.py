from path_planning.path_planning_node import PathPlanner
import numpy as np
import matplotlib.pyplot as plt


def main(args=None):
    """Test mode without ROS"""
    
    # Define test parameters
    tool_size = 10.0
    
    uv_boundary = np.array([
        [20.0, 20.0],
        [180.0, 30.0],
        [190.0, 70.0],
        [170.0, 130.0],
        [100.0, 140.0],
        [30.0, 120.0],
        [10.0, 60.0]
    ])

    uv_bounds = {
        'u_min': min(uv_boundary[:, 0]),
        'u_max': max(uv_boundary[:, 0]),
        'v_min': min(uv_boundary[:, 1]),
        'v_max': max(uv_boundary[:, 1])
    }

    point_spacing = 1
    n_bezier = 50

    # Create planner in test mode
    planner = PathPlanner(
        point_spacing=point_spacing,
        line_spacing=2*tool_size,
        n_bezier=n_bezier,
        uv_bounds=uv_bounds,
        uv_boundary=uv_boundary,
        tool_size=tool_size,
        test_active=True
    )
    
    # Generate path
    print("\nGenerating path...")
    planner.generate_lines()
    planner.create_continuous_path()


    
    if planner.uv_path is not None:
        print(f"Generated planner.uv_path with {len(planner.uv_path)} points")
        print(f"On-surface points: {np.sum(planner.continuous_on_surface)}")
        print(f"Off-surface points: {np.sum(~planner.continuous_on_surface)}")
        
        # Visualize
        plt.figure(figsize=(12, 8))
        
        # Plot boundary
        boundary_closed = np.vstack([uv_boundary, uv_boundary[0]])
        plt.fill(uv_boundary[:, 0], uv_boundary[:, 1], alpha=0.2, color='lightblue', label='Boundary')
        plt.plot(boundary_closed[:, 0], boundary_closed[:, 1], 'b-', linewidth=2)
        
        # Plot bounds
        plt.plot([uv_bounds['u_min'], uv_bounds['u_max'], uv_bounds['u_max'], uv_bounds['u_min'], uv_bounds['u_min']],
                [uv_bounds['v_min'], uv_bounds['v_min'], uv_bounds['v_max'], uv_bounds['v_max'], uv_bounds['v_min']],
                'k--', linewidth=1, label='UV bounds', alpha=0.5)
        
        # Plot path with colors
        on_idx = np.where(planner.continuous_on_surface)[0]
        off_idx = np.where(~planner.continuous_on_surface)[0]
        
        if len(on_idx) > 0:
            plt.scatter(planner.uv_path[on_idx, 0], planner.uv_path[on_idx, 1], c='green', s=3, alpha=0.7, label='On surface')
        if len(off_idx) > 0:
            plt.scatter(planner.uv_path[off_idx, 0], planner.uv_path[off_idx, 1], c='red', s=3, alpha=0.7, label='Off surface')
        
        # Mark start/end
        plt.plot(planner.uv_path[0, 0], planner.uv_path[0, 1], 'go', markersize=12, label='Start', markeredgecolor='black', markeredgewidth=2)
        plt.plot(planner.uv_path[-1, 0], planner.uv_path[-1, 1], 'rs', markersize=12, label='End', markeredgecolor='black', markeredgewidth=2)
        
        plt.xlabel('U parameter')
        plt.ylabel('V parameter')
        plt.title(f'Path Planning Test (test_active mode)\n{len(planner.uv_path)} points total')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.tight_layout()
        plt.show()
    else:
        print("Failed to generate path")


if __name__ == '__main__':
    main()