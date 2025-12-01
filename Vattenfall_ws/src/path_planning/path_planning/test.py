from path_planning.path_planning_node import PathPlanner
import numpy as np
import matplotlib.pyplot as plt

def main(args=None):
    """Test mode without ROS"""
    
    # Define test parameters
    tool_size = 20

    # create a random star-shaped (non-self-intersecting) polygon
    rng = np.random.default_rng()
    points = 10
    center = np.array([100.0, 80.0])
    angles = np.sort(rng.random(points) * 2 * np.pi)
    radii = rng.uniform(20.0, 90.0, size=points)
    uv_boundary = center + np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])

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
        line_spacing=tool_size,
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
        print(f"Ratio on-surface vs off-surface: {np.sum(planner.continuous_on_surface) / len(planner.uv_path):.2%}")
        
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
        
        # Common elements for both plots
        boundary_closed = np.vstack([uv_boundary, uv_boundary[0]])
        on_idx = np.where(planner.continuous_on_surface)[0]
        off_idx = np.where(~planner.continuous_on_surface)[0]
        
        # Plot 1: WITHOUT tool size visualization
        ax1.fill(uv_boundary[:, 0], uv_boundary[:, 1], alpha=0.2, color='lightblue', label='Boundary')
        ax1.plot(boundary_closed[:, 0], boundary_closed[:, 1], 'b-', linewidth=2)
        ax1.plot([uv_bounds['u_min'], uv_bounds['u_max'], uv_bounds['u_max'], uv_bounds['u_min'], uv_bounds['u_min']],
                [uv_bounds['v_min'], uv_bounds['v_min'], uv_bounds['v_max'], uv_bounds['v_max'], uv_bounds['v_min']],
                'k--', linewidth=1, label='UV bounds', alpha=0.5)
        
        if len(on_idx) > 0:
            ax1.scatter(planner.uv_path[on_idx, 0], planner.uv_path[on_idx, 1], c='green', s=5, alpha=0.7, label='On surface')
        
        if len(off_idx) > 0:
            ax1.scatter(planner.uv_path[off_idx, 0], planner.uv_path[off_idx, 1], c='red', s=5, alpha=0.7, label='Off surface')
        
        ax1.plot(planner.uv_path[0, 0], planner.uv_path[0, 1], 'go', markersize=12, label='Start', markeredgecolor='black', markeredgewidth=2)
        ax1.plot(planner.uv_path[-1, 0], planner.uv_path[-1, 1], 'rs', markersize=12, label='End', markeredgecolor='black', markeredgewidth=2)
        
        ax1.set_xlabel('U parameter', fontsize=12)
        ax1.set_ylabel('V parameter', fontsize=12)
        ax1.set_title(f'UV path \n{len(planner.uv_path)} points total', fontsize=16)
        ax1.set_title(f'Generated Path \n On-surface points: {np.sum(planner.continuous_on_surface) / len(planner.uv_path):.2%}', fontsize=16)
        ax1.legend(ncol=4, fontsize=12, loc='upper center', bbox_to_anchor=(0.5, -0.15))
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        ax1.tick_params(axis='both', labelsize=12)
        
        # Plot 2: WITH tool size visualization
        ax2.fill(uv_boundary[:, 0], uv_boundary[:, 1], alpha=0.2, color='lightblue', label='Boundary')
        ax2.plot(boundary_closed[:, 0], boundary_closed[:, 1], 'b-', linewidth=2)
        ax2.plot([uv_bounds['u_min'], uv_bounds['u_max'], uv_bounds['u_max'], uv_bounds['u_min'], uv_bounds['u_min']],
                [uv_bounds['v_min'], uv_bounds['v_min'], uv_bounds['v_max'], uv_bounds['v_max'], uv_bounds['v_min']],
                'k--', linewidth=1, label='UV bounds', alpha=0.5)
        
        if len(on_idx) > 0:
            ax2.scatter(planner.uv_path[on_idx, 0], planner.uv_path[on_idx, 1], c='green', s=5, alpha=0.7, label='On surface')
        if len(off_idx) > 0:
            ax2.scatter(planner.uv_path[off_idx, 0], planner.uv_path[off_idx, 1], c='red', s=5, alpha=0.7, label='Off surface')
        
        # Add tool size circles for all on-surface points
        for i in range(len(planner.uv_path)):
            if planner.continuous_on_surface[i]:
                circle = plt.Circle((planner.uv_path[i, 0], planner.uv_path[i, 1]), 
                                  tool_size/2, 
                                  color='orange',
                                  fill=False, 
                                  alpha=0.3,
                                  linewidth=1)
                ax2.add_patch(circle)
                
        # Add a legend entry for tool size
        tool_circle = plt.Circle((0, 0), 0, color='orange', fill=False, alpha=0.6, linewidth=2, label=f'Tool coverage (r={tool_size/2}mm)')
        ax2.add_patch(tool_circle)
        
        ax2.plot(planner.uv_path[0, 0], planner.uv_path[0, 1], 'go', markersize=12, label='Start', markeredgecolor='black', markeredgewidth=2)
        ax2.plot(planner.uv_path[-1, 0], planner.uv_path[-1, 1], 'rs', markersize=12, label='End', markeredgecolor='black', markeredgewidth=2)
        
        ax2.set_xlabel('U parameter', fontsize=12)
        ax2.set_ylabel('V parameter', fontsize=12)
        ax2.set_title(f'Path With Tool Visualisation\nTool diameter: {tool_size}mm', fontsize=18)
        ax2.legend(ncol=4, fontsize=12, loc='upper center', bbox_to_anchor=(0.5, -0.15))
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        ax2.tick_params(axis='both', labelsize=12)
        
        plt.tight_layout()
        plt.show()
    else:
        print("Failed to generate path")


if __name__ == '__main__':
    main()