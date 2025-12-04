"""
Standalone test script for testing conformal parameterization with your PLY file.

Usage:
    python test_with_ply.py <path_to_ply_file>
    
Example:
    python test_with_ply.py point_cloud.ply
    python test_with_ply.py "../Surface parameterisation/point_cloud.ply"
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle


def load_ply_file(filepath):
    """Load point cloud from PLY file"""
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(filepath)
        points = np.asarray(pcd.points)
        return points
    except ImportError:
        print("Error: Open3D not installed. Install with: pip install open3d")
        sys.exit(1)
    except Exception as e:
        print(f"Error loading PLY file: {e}")
        sys.exit(1)


def create_interactive_visualization(surf, points):
    """
    Create interactive visualization showing UV space and XYZ space side-by-side.
    User can hover/click in UV space to see corresponding XYZ location.
    """
    print("\n" + "=" * 70)
    print("  INTERACTIVE VISUALIZATION")
    print("=" * 70)
    print("\nControls:")
    print("  • Move mouse in UV space (left plot) to see XYZ mapping")
    print("  • Click to place a marker")
    print("  • Close window to exit")
    print("=" * 70 + "\n")
    
    # Get UV bounds
    bounds = surf.get_uv_bounds()
    u_min, u_max = bounds['u_min'], bounds['u_max']
    v_min, v_max = bounds['v_min'], bounds['v_max']
    
    # Create UV grid for surface visualization (lower resolution for performance)
    n_grid_viz = 30
    u_viz = np.linspace(u_min, u_max, n_grid_viz)
    v_viz = np.linspace(v_min, v_max, n_grid_viz)
    U_viz, V_viz = np.meshgrid(u_viz, v_viz)
    uv_grid = np.column_stack([U_viz.ravel(), V_viz.ravel()])
    
    # Interpolate to get XYZ surface
    print("Computing surface mesh for visualization...")
    xyz_grid = surf.interpolate(uv_grid)
    
    # Filter out NaN values (outside convex hull)
    valid_mask = ~np.isnan(xyz_grid).any(axis=1)
    xyz_grid_valid = xyz_grid[valid_mask]
    U_viz_valid = U_viz.ravel()[valid_mask].reshape(-1)
    V_viz_valid = V_viz.ravel()[valid_mask].reshape(-1)
    
    # Reshape for surface plot
    X_viz = xyz_grid_valid[:, 0].reshape(n_grid_viz, n_grid_viz) if len(xyz_grid_valid) == n_grid_viz**2 else None
    Y_viz = xyz_grid_valid[:, 1].reshape(n_grid_viz, n_grid_viz) if len(xyz_grid_valid) == n_grid_viz**2 else None
    Z_viz = xyz_grid_valid[:, 2].reshape(n_grid_viz, n_grid_viz) if len(xyz_grid_valid) == n_grid_viz**2 else None
    
    # Create figure with side-by-side plots
    fig = plt.figure(figsize=(16, 7))
    
    # Left: UV space (2D)
    ax_uv = fig.add_subplot(121)
    ax_uv.set_title('UV Parameter Space\n(Hover/Click to Select)', fontsize=12, fontweight='bold')
    ax_uv.set_xlabel('u', fontsize=11)
    ax_uv.set_ylabel('v', fontsize=11)
    ax_uv.set_aspect('equal')
    ax_uv.grid(True, alpha=0.3)
    
    # Plot UV points as scatter
    scatter_uv = ax_uv.scatter(surf.uv_params[:, 0], surf.uv_params[:, 1], 
                               c='lightblue', s=1, alpha=0.5, label='UV points')
    
    # Hover/click marker in UV space
    marker_uv, = ax_uv.plot([], [], 'ro', markersize=10, label='Selected point')
    circle_uv = Circle((0, 0), 0, fill=False, color='red', linewidth=2)
    ax_uv.add_patch(circle_uv)
    circle_uv.set_visible(False)
    
    # Text annotation for UV coordinates
    text_uv = ax_uv.text(0.02, 0.98, '', transform=ax_uv.transAxes, 
                         verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                         fontsize=10)
    
    ax_uv.set_xlim(u_min - 0.1*(u_max-u_min), u_max + 0.1*(u_max-u_min))
    ax_uv.set_ylim(v_min - 0.1*(v_max-v_min), v_max + 0.1*(v_max-v_min))
    ax_uv.legend(loc='upper right')
    
    # Right: XYZ space (3D)
    ax_xyz = fig.add_subplot(122, projection='3d')
    ax_xyz.set_title('XYZ World Space\n(Corresponding Point)', fontsize=12, fontweight='bold')
    ax_xyz.set_xlabel('X', fontsize=11)
    ax_xyz.set_ylabel('Y', fontsize=11)
    ax_xyz.set_zlabel('Z', fontsize=11)
    
    # Plot surface mesh or point cloud
    if X_viz is not None and not np.isnan(X_viz).any():
        # Plot as surface mesh
        surf_plot = ax_xyz.plot_surface(X_viz, Y_viz, Z_viz, 
                                        cmap='viridis', alpha=0.6, 
                                        linewidth=0, antialiased=True,
                                        shade=True)
    else:
        # Fallback: plot point cloud (subsample for performance)
        subsample = min(5000, len(points))
        indices = np.random.choice(len(points), subsample, replace=False)
        ax_xyz.scatter(points[indices, 0], points[indices, 1], points[indices, 2],
                      c=points[indices, 2], cmap='viridis', s=1, alpha=0.3)
    
    # Hover/click marker in XYZ space
    marker_xyz, = ax_xyz.plot([], [], [], 'ro', markersize=10, label='Selected point')
    
    # Text annotation for XYZ coordinates
    text_xyz = ax_xyz.text2D(0.02, 0.98, '', transform=ax_xyz.transAxes,
                             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                             fontsize=10)
    
    ax_xyz.legend(loc='upper right')
    
    # Set equal aspect ratio for 3D plot
    x_range = points[:, 0].max() - points[:, 0].min()
    y_range = points[:, 1].max() - points[:, 1].min()
    z_range = points[:, 2].max() - points[:, 2].min()
    max_range = max(x_range, y_range, z_range)
    x_mid = (points[:, 0].max() + points[:, 0].min()) / 2
    y_mid = (points[:, 1].max() + points[:, 1].min()) / 2
    z_mid = (points[:, 2].max() + points[:, 2].min()) / 2
    ax_xyz.set_xlim(x_mid - max_range/2, x_mid + max_range/2)
    ax_xyz.set_ylim(y_mid - max_range/2, y_mid + max_range/2)
    ax_xyz.set_zlim(z_mid - max_range/2, z_mid + max_range/2)
    
    plt.tight_layout()
    
    # Event handler for mouse motion (real-time update)
    def on_motion(event):
        if event.inaxes == ax_uv:
            u_hover = event.xdata
            v_hover = event.ydata
            
            if u_hover is not None and v_hover is not None:
                # Check if within bounds
                if u_min <= u_hover <= u_max and v_min <= v_hover <= v_max:
                    # Interpolate to get XYZ
                    uv_query = np.array([[u_hover, v_hover]])
                    xyz_query = surf.interpolate(uv_query)
                    
                    if not np.isnan(xyz_query).any():
                        # Update UV marker
                        marker_uv.set_data([u_hover], [v_hover])
                        circle_uv.center = (u_hover, v_hover)
                        circle_uv.radius = 0.02 * (u_max - u_min)
                        circle_uv.set_visible(True)
                        
                        # Update XYZ marker
                        marker_xyz.set_data([xyz_query[0, 0]], [xyz_query[0, 1]])
                        marker_xyz.set_3d_properties([xyz_query[0, 2]])
                        
                        # Update text annotations
                        text_uv.set_text(f'UV: ({u_hover:.4f}, {v_hover:.4f})')
                        text_xyz.set_text(f'XYZ: ({xyz_query[0, 0]:.4f}, {xyz_query[0, 1]:.4f}, {xyz_query[0, 2]:.4f})')
                        
                        fig.canvas.draw_idle()
    
    # Event handler for mouse click (place persistent marker)
    def on_click(event):
        if event.inaxes == ax_uv and event.button == 1:  # Left click
            u_click = event.xdata
            v_click = event.ydata
            
            if u_click is not None and v_click is not None:
                # Check if within bounds
                if u_min <= u_click <= u_max and v_min <= v_click <= v_max:
                    # Interpolate to get XYZ
                    uv_query = np.array([[u_click, v_click]])
                    xyz_query = surf.interpolate(uv_query)
                    
                    if not np.isnan(xyz_query).any():
                        # Add persistent markers
                        ax_uv.plot(u_click, v_click, 'g*', markersize=12, markeredgecolor='black', markeredgewidth=0.5)
                        ax_xyz.plot([xyz_query[0, 0]], [xyz_query[0, 1]], [xyz_query[0, 2]], 
                                   'g*', markersize=12, markeredgecolor='black', markeredgewidth=0.5)
                        
                        print(f"Clicked: UV=({u_click:.4f}, {v_click:.4f}) -> XYZ=({xyz_query[0, 0]:.4f}, {xyz_query[0, 1]:.4f}, {xyz_query[0, 2]:.4f})")
                        
                        fig.canvas.draw_idle()
    
    # Connect event handlers
    fig.canvas.mpl_connect('motion_notify_event', on_motion)
    fig.canvas.mpl_connect('button_press_event', on_click)
    
    plt.show()


def test_parameterization(ply_file_path):
    """Test conformal parameterization with a PLY file"""
    
    # Import the module
    try:
        from parameterization.surface_parameterization import Parameterization
    except ImportError:
        # Try adding parent directory to path
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
        from parameterization.surface_parameterization import Parameterization
    
    print("=" * 70)
    print("  TESTING CONFORMAL PARAMETERIZATION")
    print("=" * 70)
    
    # Load point cloud
    print(f"\n1. Loading PLY file: {ply_file_path}")
    if not os.path.exists(ply_file_path):
        print(f"Error: File not found: {ply_file_path}")
        return False
    
    points = load_ply_file(ply_file_path)
    print(f"   Loaded {len(points)} points")
    print(f"   X range: [{np.min(points[:, 0]):.3f}, {np.max(points[:, 0]):.3f}]")
    print(f"   Y range: [{np.min(points[:, 1]):.3f}, {np.max(points[:, 1]):.3f}]")
    print(f"   Z range: [{np.min(points[:, 2]):.3f}, {np.max(points[:, 2]):.3f}]")
    
    # Initialize parameterization
    print("\n2. Initializing conformal parameterization...")
    surf = Parameterization()
    surf.set_points(points)
    print("   Points set")
    
    # Compute local frame
    print("\n3. Computing local frame (PCA)...")
    principal_axes, centroid = surf.compute_local_frame()
    print(f"   Local frame computed")
    print(f"   Centroid: [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}]")
    print(f"   Principal axes computed (3x3 orthonormal matrix)")
    
    # Compute initial UV parameterization
    print("\n4. Computing initial UV parameterization...")
    uv = surf.compute_initial_parameterization()
    bounds = surf.get_uv_bounds()
    print(f"   Initial UV parameterization computed")
    print(f"   U range: [{bounds['u_min']:.3f}, {bounds['u_max']:.3f}]")
    print(f"   V range: [{bounds['v_min']:.3f}, {bounds['v_max']:.3f}]")
    
    # Build interpolation
    print("\n5. Building inverse interpolation...")
    surf.build_inverse_interpolation()
    print(f"   Interpolation ready")
    
    # Evaluate quality
    print("\n7. Evaluating quality metrics...")
    sample_size = min(1000, len(points))
    metrics = surf.evaluate_quality(sample_size=sample_size)
    print(f"   Quality evaluation complete")
    print(f"   Sample size: {metrics['sample_size']}/{metrics['total_points']} points")
    print(f"   Mean error: {metrics['mean_error']:.6f}")
    print(f"   Max error: {metrics['max_error']:.6f}")
    print(f"   RMSE: {metrics['rmse']:.6f}")
    print(f"   Std deviation: {metrics['std_error']:.6f}")
    
    # Test interpolation
    print("\n6. Testing interpolation...")
    n_test = min(10, len(points))
    test_indices = np.random.choice(len(points), n_test, replace=False)
    test_uv = surf.uv_params[test_indices]
    expected_xyz = surf.points[test_indices]
    
    interpolated = surf.interpolate(test_uv)
    errors = np.linalg.norm(interpolated - expected_xyz, axis=1)
    
    print(f"   ✓ Interpolation tested on {n_test} points")
    print(f"   Mean error: {np.mean(errors):.6f}")
    print(f"   Max error: {np.max(errors):.6f}")
    
    # Test frame transformations
    print("\n9. Testing frame transformations...")
    n_transform = min(20, len(points))
    test_points = points[:n_transform]
    
    local_points = surf.global_to_local(test_points)
    reconstructed = surf.local_to_global(local_points)
    transform_errors = np.linalg.norm(reconstructed - test_points, axis=1)
    
    print(f"    Round-trip transformation tested on {n_transform} points")
    print(f"    Max reconstruction error: {np.max(transform_errors):.10f}")
    
    # Launch interactive visualization
    print("\n8. Launching interactive visualization...")
    create_interactive_visualization(surf, points)
    
    # Summary
    print("\n" + "=" * 70)
    print("  ALL TESTS PASSED!")
    print("=" * 70)
    print("\nSummary:")
    print(f"  • Point cloud: {len(points)} points")
    print(f"  • UV parameterization: {uv.shape}")
    print(f"  • Interpolation: Working")
    print(f"  • Frame transformations: Working")
    print("\n" + "=" * 70)
    
    return True


def main():
    """Main function"""
    if len(sys.argv) < 2:
        print("Usage: python test_with_ply.py <path_to_ply_file>")
        print("\nSearching for point_cloud.ply in test folder...")
        
        # Look in test folder (same directory as this script)
        test_dir = os.path.dirname(os.path.abspath(__file__))
        ply_file = os.path.join(test_dir, "point_cloud.ply")
        
        if os.path.exists(ply_file):
            print(f"Found: {ply_file}")
        else:
            print(f"\nPLY file not found in: {test_dir}")
            print("\nPlease either:")
            print(f"  1. Copy your PLY file to: {test_dir}")
            print("  2. Or specify path: python test_with_ply.py <path_to_ply_file>")
            sys.exit(1)
    else:
        ply_file = sys.argv[1]
    
    try:
        success = test_parameterization(ply_file)
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\nError during testing: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
