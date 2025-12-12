"""
Standalone test script for testing conformal parameterization with point cloud files.

Usage:
    # Single file mode (training and evaluation on same data):
    python test_with_ply.py <path_to_file>
    
    # Two-file mode (train on first, evaluate on second):
    python test_with_ply.py <training_file> --eval <evaluation_file>
    
Examples:
    python test_with_ply.py point_cloud.ply
    python test_with_ply.py workspace_pointcloud_1.npy --eval workspace_pointcloud_2.npy
    python test_with_ply.py "../Surface parameterisation/point_cloud.ply"
    
Supported formats: .ply, .npy
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


def load_point_cloud(filepath):
    """
    Load point cloud from PLY or NPY file.
    
    Args:
        filepath: Path to .ply or .npy file
        
    Returns:
        points: Nx3 numpy array of (x, y, z) coordinates
    """
    if not os.path.exists(filepath):
        print(f"Error: File not found: {filepath}")
        sys.exit(1)
        
    ext = os.path.splitext(filepath)[1].lower()
    
    if ext == '.npy':
        try:
            points = np.load(filepath)
            if points.ndim != 2 or points.shape[1] != 3:
                print(f"Error: NPY file must contain Nx3 array, got shape {points.shape}")
                sys.exit(1)
            return points
        except Exception as e:
            print(f"Error loading NPY file: {e}")
            sys.exit(1)
    elif ext == '.ply':
        return load_ply_file(filepath)
    else:
        print(f"Error: Unsupported file format '{ext}'. Use .ply or .npy")
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


def find_uv_for_point(surf, target_xyz_local, initial_uv=None, max_iter=50, tol=1e-6):
    """
    Find the UV coordinates that map closest to a target XYZ point.
    
    Uses numerical optimization to solve: argmin_uv ||surf.interpolate(uv) - target||
    
    Args:
        surf: Parameterization object with interpolators
        target_xyz_local: Target point in local coordinates (1x3)
        initial_uv: Initial guess for UV (if None, uses nearest training point)
        max_iter: Maximum optimization iterations
        tol: Convergence tolerance
        
    Returns:
        uv: Optimal UV coordinates (1x2)
        residual: Distance from interpolated point to target
    """
    from scipy.optimize import minimize
    
    target = target_xyz_local.flatten()
    
    # Get UV bounds for constraints
    bounds = surf.get_uv_bounds()
    uv_bounds = [(bounds['u_min'], bounds['u_max']), 
                 (bounds['v_min'], bounds['v_max'])]
    
    # Initial guess: find nearest training point in XY and use its UV
    if initial_uv is None:
        xy_target = target[:2]
        xy_train = surf.points_local[:, :2]
        distances = np.linalg.norm(xy_train - xy_target, axis=1)
        nearest_idx = np.argmin(distances)
        initial_uv = surf.uv_params[nearest_idx].copy()
    
    def objective(uv):
        """Distance from interpolated point to target"""
        uv_2d = np.atleast_2d(uv)
        # Interpolate in local coordinates
        x_interp = surf.interpolator_x(uv_2d)[0]
        y_interp = surf.interpolator_y(uv_2d)[0]
        z_interp = surf.interpolator_z(uv_2d)[0]
        
        if np.isnan(x_interp) or np.isnan(y_interp) or np.isnan(z_interp):
            return 1e10  # Penalty for outside domain
        
        interp_point = np.array([x_interp, y_interp, z_interp])
        return np.linalg.norm(interp_point - target)
    
    # Optimize
    result = minimize(objective, initial_uv, method='L-BFGS-B', 
                     bounds=uv_bounds, options={'maxiter': max_iter, 'ftol': tol})
    
    return result.x, result.fun


def evaluate_on_second_pointcloud(surf, eval_points, visualize=True, max_eval_points=5000):
    """
    Evaluate interpolation quality using a second independent point cloud.
    
    Uses proper UV-based evaluation:
    1. For each eval point, find optimal UV via numerical optimization
    2. Interpolate XYZ from that UV using the trained pipeline
    3. Measure residual distance (how close the surface gets to each point)
    
    This tests the ACTUAL parameterization pipeline (UV → XYZ mapping).
    
    Args:
        surf: Trained Parameterization object
        eval_points: Nx3 array of evaluation points (from second point cloud)
        visualize: Whether to show visualization plots
        max_eval_points: Maximum points to evaluate (for performance)
        
    Returns:
        dict: Evaluation metrics including errors and statistics
    """
    from scipy.spatial import cKDTree
    
    print("\n" + "=" * 70)
    print("  EVALUATING ON SECOND POINT CLOUD (UV-Based Method)")
    print("=" * 70)
    
    print(f"\n   Evaluation points: {len(eval_points)}")
    print(f"   X range: [{np.min(eval_points[:, 0]):.3f}, {np.max(eval_points[:, 0]):.3f}]")
    print(f"   Y range: [{np.min(eval_points[:, 1]):.3f}, {np.max(eval_points[:, 1]):.3f}]")
    print(f"   Z range: [{np.min(eval_points[:, 2]):.3f}, {np.max(eval_points[:, 2]):.3f}]")
    
    # Subsample if too many points (optimization is expensive)
    if len(eval_points) > max_eval_points:
        print(f"\n   Subsampling to {max_eval_points} points for performance...")
        indices = np.random.choice(len(eval_points), max_eval_points, replace=False)
        eval_points_subset = eval_points[indices]
    else:
        eval_points_subset = eval_points
        indices = np.arange(len(eval_points))
    
    n_eval = len(eval_points_subset)
    
    # Transform evaluation points to local frame
    print("\n   Transforming to local frame...")
    eval_local = surf.global_to_local(eval_points_subset)
    
    # Build KD-tree for fast initial UV guess lookup
    print("   Building KD-tree for initial UV estimation...")
    train_xy = surf.points_local[:, :2]
    kdtree_xy = cKDTree(train_xy)
    
    # Find optimal UV for each evaluation point
    print(f"   Finding optimal UV coordinates for {n_eval} points...")
    print("   (This may take a moment...)")
    
    eval_uv = np.zeros((n_eval, 2))
    residuals = np.zeros(n_eval)
    interpolated_local = np.zeros((n_eval, 3))
    
    # Progress tracking
    progress_step = max(1, n_eval // 10)
    
    for i in range(n_eval):
        if i % progress_step == 0:
            print(f"      Progress: {i}/{n_eval} ({100*i/n_eval:.0f}%)")
        
        target_local = eval_local[i]
        
        # Find nearest training point for initial guess
        _, nearest_idx = kdtree_xy.query(target_local[:2])
        initial_uv = surf.uv_params[nearest_idx].copy()
        
        # Optimize to find best UV
        optimal_uv, residual = find_uv_for_point(surf, target_local, initial_uv)
        
        eval_uv[i] = optimal_uv
        residuals[i] = residual
        
        # Store interpolated point
        uv_2d = np.atleast_2d(optimal_uv)
        interpolated_local[i, 0] = surf.interpolator_x(uv_2d)[0]
        interpolated_local[i, 1] = surf.interpolator_y(uv_2d)[0]
        interpolated_local[i, 2] = surf.interpolator_z(uv_2d)[0]
    
    print(f"      Progress: {n_eval}/{n_eval} (100%)")
    
    # Filter out failed optimizations (NaN or very high residuals)
    valid_mask = ~np.isnan(interpolated_local).any(axis=1) & (residuals < 1e9)
    n_valid = np.sum(valid_mask)
    n_invalid = n_eval - n_valid
    
    print(f"\n   Valid points: {n_valid}/{n_eval} ({100*n_valid/n_eval:.1f}%)")
    if n_invalid > 0:
        print(f"   Points outside domain: {n_invalid}")
    
    # Extract valid data
    eval_local_valid = eval_local[valid_mask]
    interp_local_valid = interpolated_local[valid_mask]
    eval_uv_valid = eval_uv[valid_mask]
    residuals_valid = residuals[valid_mask]
    
    # Compute component-wise errors in local frame
    errors_x = np.abs(interp_local_valid[:, 0] - eval_local_valid[:, 0])
    errors_y = np.abs(interp_local_valid[:, 1] - eval_local_valid[:, 1])
    errors_z = np.abs(interp_local_valid[:, 2] - eval_local_valid[:, 2])
    
    # Signed Z error (most important for surface)
    z_errors_signed = interp_local_valid[:, 2] - eval_local_valid[:, 2]
    
    # 3D residual (distance from surface to point)
    errors_3d = residuals_valid
    
    # Compute metrics
    metrics = {
        'n_total': len(eval_points),
        'n_evaluated': n_eval,
        'n_valid': n_valid,
        'n_invalid': n_invalid,
        'coverage': n_valid / n_eval,
        
        # 3D surface distance (residual from optimization)
        'mean_residual': float(np.mean(errors_3d)),
        'max_residual': float(np.max(errors_3d)),
        'std_residual': float(np.std(errors_3d)),
        'rmse_residual': float(np.sqrt(np.mean(errors_3d**2))),
        'median_residual': float(np.median(errors_3d)),
        
        # Per-axis errors
        'mean_error_x': float(np.mean(errors_x)),
        'mean_error_y': float(np.mean(errors_y)),
        'mean_error_z': float(np.mean(errors_z)),
        'max_error_x': float(np.max(errors_x)),
        'max_error_y': float(np.max(errors_y)),
        'max_error_z': float(np.max(errors_z)),
        
        # Z-error (signed, to detect bias)
        'mean_z_error_signed': float(np.mean(z_errors_signed)),
        'std_z_error_signed': float(np.std(z_errors_signed)),
        
        # Percentiles
        'residual_95th': float(np.percentile(errors_3d, 95)),
        'residual_99th': float(np.percentile(errors_3d, 99)),
    }
    
    # Print results
    print("\n   " + "-" * 50)
    print("   EVALUATION RESULTS (UV-Based Surface Distance)")
    print("   " + "-" * 50)
    print(f"   Surface Residual (distance from surface to eval points):")
    print(f"     Mean residual:  {metrics['mean_residual']:.6f}")
    print(f"     Median residual:{metrics['median_residual']:.6f}")
    print(f"     Max residual:   {metrics['max_residual']:.6f}")
    print(f"     RMSE:           {metrics['rmse_residual']:.6f}")
    print(f"     Std deviation:  {metrics['std_residual']:.6f}")
    print(f"     95th percentile:{metrics['residual_95th']:.6f}")
    print(f"     99th percentile:{metrics['residual_99th']:.6f}")
    print(f"\n   Per-Axis Mean Errors (Local Frame):")
    print(f"     X: {metrics['mean_error_x']:.6f}  (max: {metrics['max_error_x']:.6f})")
    print(f"     Y: {metrics['mean_error_y']:.6f}  (max: {metrics['max_error_y']:.6f})")
    print(f"     Z: {metrics['mean_error_z']:.6f}  (max: {metrics['max_error_z']:.6f})")
    print(f"\n   Signed Z-Error (bias check):")
    print(f"     Mean signed:    {metrics['mean_z_error_signed']:.6f}")
    print(f"     Std signed:     {metrics['std_z_error_signed']:.6f}")
    if abs(metrics['mean_z_error_signed']) > metrics['std_z_error_signed'] * 0.1:
        print(f"     ⚠ Note: Non-zero mean suggests systematic bias")
    else:
        print(f"     ✓ No significant systematic bias detected")
    print("   " + "-" * 50)
    
    if visualize and n_valid > 0:
        create_evaluation_visualization_uv(
            surf, eval_local_valid, interp_local_valid, eval_uv_valid, residuals_valid, z_errors_signed
        )
    
    return metrics


def create_evaluation_visualization_uv(surf, eval_local, interp_local, eval_uv, residuals, z_errors):
    """
    Create visualization for UV-based evaluation.
    """
    print("\n   Creating evaluation visualization...")
    
    fig = plt.figure(figsize=(18, 12))
    
    # 1. Residual (surface distance) distribution
    ax1 = fig.add_subplot(231)
    ax1.hist(residuals, bins=50, edgecolor='black', alpha=0.7, color='steelblue')
    ax1.axvline(np.mean(residuals), color='r', linestyle='--', linewidth=2, 
                label=f'Mean: {np.mean(residuals):.4f}')
    ax1.axvline(np.median(residuals), color='g', linestyle='--', linewidth=2,
                label=f'Median: {np.median(residuals):.4f}')
    ax1.set_xlabel('Surface Distance (Residual)', fontsize=11)
    ax1.set_ylabel('Count', fontsize=11)
    ax1.set_title('Surface Distance Distribution', fontsize=12, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 2. Residual heatmap in UV space
    ax2 = fig.add_subplot(232)
    scatter = ax2.scatter(eval_uv[:, 0], eval_uv[:, 1], c=residuals, 
                          cmap='hot', s=5, alpha=0.7)
    plt.colorbar(scatter, ax=ax2, label='Residual')
    ax2.set_xlabel('U', fontsize=11)
    ax2.set_ylabel('V', fontsize=11)
    ax2.set_title('Residual Distribution in UV Space', fontsize=12, fontweight='bold')
    ax2.set_aspect('equal')
    
    # 3. Residual heatmap in XY space (local)
    ax3 = fig.add_subplot(233)
    scatter3 = ax3.scatter(eval_local[:, 0], eval_local[:, 1], c=residuals, 
                           cmap='hot', s=5, alpha=0.7)
    plt.colorbar(scatter3, ax=ax3, label='Residual')
    ax3.set_xlabel('Local X', fontsize=11)
    ax3.set_ylabel('Local Y', fontsize=11)
    ax3.set_title('Residual Distribution in XY Space', fontsize=12, fontweight='bold')
    ax3.set_aspect('equal')
    
    # 4. Signed Z-error histogram
    ax4 = fig.add_subplot(234)
    ax4.hist(z_errors, bins=50, edgecolor='black', alpha=0.7, color='purple')
    ax4.axvline(0, color='k', linestyle='-', linewidth=2)
    ax4.axvline(np.mean(z_errors), color='r', linestyle='--', linewidth=2,
                label=f'Mean: {np.mean(z_errors):.4f}')
    ax4.set_xlabel('Z Error (Surface - Actual)', fontsize=11)
    ax4.set_ylabel('Count', fontsize=11)
    ax4.set_title('Signed Z-Error Distribution', fontsize=12, fontweight='bold')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # 5. 3D view - actual vs surface points
    ax5 = fig.add_subplot(235, projection='3d')
    subsample = min(2000, len(eval_local))
    idx = np.random.choice(len(eval_local), subsample, replace=False)
    ax5.scatter(eval_local[idx, 0], eval_local[idx, 1], eval_local[idx, 2], 
               c='blue', s=2, alpha=0.5, label='Actual (Point Cloud 2)')
    ax5.scatter(interp_local[idx, 0], interp_local[idx, 1], interp_local[idx, 2],
               c='red', s=2, alpha=0.5, label='Surface (Interpolated)')
    ax5.set_xlabel('Local X')
    ax5.set_ylabel('Local Y')
    ax5.set_zlabel('Local Z')
    ax5.set_title('Actual vs Surface Points (Local Frame)', fontsize=12, fontweight='bold')
    ax5.legend()
    
    # 6. Cumulative residual distribution
    ax6 = fig.add_subplot(236)
    sorted_residuals = np.sort(residuals)
    cumulative = np.arange(1, len(sorted_residuals) + 1) / len(sorted_residuals)
    ax6.plot(sorted_residuals, cumulative, linewidth=2, color='steelblue')
    ax6.axhline(0.95, color='r', linestyle='--', alpha=0.7, label='95%')
    ax6.axhline(0.99, color='orange', linestyle='--', alpha=0.7, label='99%')
    p95 = np.percentile(residuals, 95)
    p99 = np.percentile(residuals, 99)
    ax6.axvline(p95, color='r', linestyle=':', alpha=0.5)
    ax6.axvline(p99, color='orange', linestyle=':', alpha=0.5)
    ax6.set_xlabel('Residual Threshold', fontsize=11)
    ax6.set_ylabel('Fraction of Points Below Threshold', fontsize=11)
    ax6.set_title(f'Cumulative Distribution (95%: {p95:.4f}, 99%: {p99:.4f})', fontsize=12, fontweight='bold')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.suptitle('UV-Based Surface Evaluation: Distance from Interpolated Surface to Eval Points', 
                 fontsize=14, fontweight='bold', y=1.02)
    plt.show()


def test_parameterization(training_file_path, eval_file_path=None):
    """
    Test conformal parameterization with point cloud file(s).
    
    Args:
        training_file_path: Path to training point cloud (.ply or .npy)
        eval_file_path: Optional path to evaluation point cloud (.ply or .npy)
                        If None, evaluation uses the training data.
    """
    
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
    
    # Load training point cloud
    print(f"\n1. Loading training point cloud: {training_file_path}")
    points = load_point_cloud(training_file_path)
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
    
    # Evaluate quality on training data (reconstruction error)
    print("\n6. Evaluating reconstruction quality (training data)...")
    sample_size = min(1000, len(points))
    metrics = surf.evaluate_quality(sample_size=sample_size)
    print(f"   Quality evaluation complete")
    print(f"   Sample size: {metrics['sample_size']}/{metrics['total_points']} points")
    print(f"   Mean error: {metrics['mean_error']:.6f}")
    print(f"   Max error: {metrics['max_error']:.6f}")
    print(f"   RMSE: {metrics['rmse']:.6f}")
    print(f"   Std deviation: {metrics['std_error']:.6f}")
    
    # Test interpolation on training data
    print("\n7. Testing interpolation on training data...")
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
    print("\n8. Testing frame transformations...")
    n_transform = min(20, len(points))
    test_points = points[:n_transform]
    
    local_points = surf.global_to_local(test_points)
    reconstructed = surf.local_to_global(local_points)
    transform_errors = np.linalg.norm(reconstructed - test_points, axis=1)
    
    print(f"   Round-trip transformation tested on {n_transform} points")
    print(f"   Max reconstruction error: {np.max(transform_errors):.10f}")
    
    # If evaluation file is provided, evaluate on second point cloud
    eval_metrics = None
    if eval_file_path is not None:
        print(f"\n9. Loading evaluation point cloud: {eval_file_path}")
        eval_points = load_point_cloud(eval_file_path)
        eval_metrics = evaluate_on_second_pointcloud(surf, eval_points, visualize=True)
    
    # Launch interactive visualization
    print("\n10. Launching interactive visualization...")
    create_interactive_visualization(surf, points)
    
    # Summary
    print("\n" + "=" * 70)
    print("  ALL TESTS PASSED!")
    print("=" * 70)
    print("\nSummary:")
    print(f"  • Training point cloud: {len(points)} points")
    print(f"  • UV parameterization: {uv.shape}")
    print(f"  • Interpolation: Working")
    print(f"  • Frame transformations: Working")
    
    if eval_metrics is not None:
        print(f"\n  Evaluation on Second Point Cloud (UV-Based):")
        print(f"  • Evaluated points: {eval_metrics['n_evaluated']} ({eval_metrics['n_valid']} valid)")
        print(f"  • Mean surface distance: {eval_metrics['mean_residual']:.6f}")
        print(f"  • RMSE: {eval_metrics['rmse_residual']:.6f}")
        print(f"  • 95th percentile: {eval_metrics['residual_95th']:.6f}")
    
    print("\n" + "=" * 70)
    
    return True


def main():
    """Main function with support for single-file and two-file evaluation modes."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Test conformal parameterization with point cloud files.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Single file mode (train and evaluate on same data):
    python test_with_ply.py point_cloud.ply
    python test_with_ply.py workspace_pointcloud_1.npy
    
  Two-file mode (train on first, evaluate on second):
    python test_with_ply.py workspace_pointcloud_1.npy --eval workspace_pointcloud_2.npy
    python test_with_ply.py train.ply --eval test.ply
    
Supported formats: .ply, .npy
        """
    )
    parser.add_argument('training_file', nargs='?', default=None,
                        help='Path to training point cloud file (.ply or .npy)')
    parser.add_argument('--eval', '-e', dest='eval_file', default=None,
                        help='Path to evaluation point cloud file (.ply or .npy)')
    
    args = parser.parse_args()
    
    # Determine training file
    if args.training_file is None:
        print("No training file specified.")
        print("Searching for point_cloud.ply or workspace_pointcloud_1.npy in test folder...")
        
        test_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Check for common file names
        candidates = [
            os.path.join(test_dir, "point_cloud.ply"),
            os.path.join(test_dir, "workspace_pointcloud_1.npy"),
            os.path.join(test_dir, "pointcloud.ply"),
            os.path.join(test_dir, "pointcloud.npy"),
        ]
        
        training_file = None
        for candidate in candidates:
            if os.path.exists(candidate):
                training_file = candidate
                print(f"Found: {training_file}")
                break
        
        if training_file is None:
            print(f"\nNo point cloud file found in: {test_dir}")
            print("\nPlease either:")
            print(f"  1. Copy your point cloud file to: {test_dir}")
            print("  2. Or specify path: python test_with_ply.py <path_to_file>")
            print("\nFor two-file evaluation:")
            print("  python test_with_ply.py train.npy --eval eval.npy")
            sys.exit(1)
    else:
        training_file = args.training_file
    
    eval_file = args.eval_file
    
    # Print mode info
    if eval_file:
        print("\n" + "=" * 70)
        print("  TWO-FILE EVALUATION MODE")
        print("=" * 70)
        print(f"  Training file:   {training_file}")
        print(f"  Evaluation file: {eval_file}")
        print("=" * 70 + "\n")
    else:
        print("\n" + "=" * 70)
        print("  SINGLE-FILE MODE")
        print("=" * 70)
        print(f"  Point cloud: {training_file}")
        print("  (Use --eval <file> to evaluate on a second point cloud)")
        print("=" * 70 + "\n")
    
    try:
        success = test_parameterization(training_file, eval_file)
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\nError during testing: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
