#!/usr/bin/env python3
"""
Test script for conformal parameterization following Amersdorfer et al. (2021)

This demonstrates the key differences between simple projection and 
conformal parameterization for equidistant path planning.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add the package to path
sys.path.insert(0, '../parameterization')

from surface_parameterization import SurfaceParameterization
from conformal_parameterization import ConformalParameterization


def generate_curved_surface(n_points=500):
    """
    Generate a curved surface for testing.
    
    Creates a cylindrical surface section to test the approach on a curved surface
    where distance preservation matters.
    """
    # Cylindrical surface
    theta = np.linspace(0, np.pi/2, 30)  # 90 degrees
    z = np.linspace(0, 1.0, 20)
    
    theta_grid, z_grid = np.meshgrid(theta, z)
    
    # Cylinder with radius 0.5m
    radius = 0.5
    x = radius * np.cos(theta_grid.flatten())
    y = radius * np.sin(theta_grid.flatten())
    z = z_grid.flatten()
    
    points = np.column_stack([x, y, z])
    
    # Add some noise to make it more realistic
    noise = np.random.normal(0, 0.002, points.shape)
    points += noise
    
    return points


def test_simple_vs_conformal():
    """
    Compare simple projection vs conformal parameterization.
    """
    print("=" * 80)
    print("Testing: Simple vs Conformal Parameterization")
    print("Following Amersdorfer et al. (2021)")
    print("=" * 80)
    
    # Generate curved surface
    print("\n1. Generating curved cylindrical surface...")
    points = generate_curved_surface()
    print(f"   Generated {len(points)} points")
    
    # Test simple parameterization
    print("\n2. Testing SIMPLE projection parameterization...")
    surf_simple = SurfaceParameterization()
    surf_simple.set_points(points)
    surf_simple.compute_local_frame()
    surf_simple.compute_uv_parameterization(method='projection', normalize=False)
    surf_simple.build_inverse_interpolation(method='rbf', neighbors=50)
    
    metrics_simple = surf_simple.evaluate_quality(sample_size=200)
    print(f"   Reconstruction RMSE: {metrics_simple['rmse']:.6f} m")
    print(f"   Mean error: {metrics_simple['mean_error']:.6f} m")
    print(f"   Max error: {metrics_simple['max_error']:.6f} m")
    
    # Test conformal parameterization
    print("\n3. Testing CONFORMAL parameterization (Amersdorfer)...")
    surf_conformal = ConformalParameterization()
    surf_conformal.set_points(points)
    surf_conformal.compute_local_frame()
    surf_conformal.compute_initial_parameterization(method='projection')
    
    print("   Computing surface metric tensor...")
    surf_conformal.compute_surface_metric(k_neighbors=20)
    
    print("   Applying conformal correction...")
    surf_conformal.apply_conformal_correction(iterations=5, alpha=0.5)
    
    surf_conformal.build_inverse_interpolation(method='rbf', neighbors=50)
    
    metrics_conformal = surf_conformal.evaluate_quality(sample_size=200)
    print(f"   Reconstruction RMSE: {metrics_conformal['rmse']:.6f} m")
    print(f"   Mean error: {metrics_conformal['mean_error']:.6f} m")
    print(f"   Max error: {metrics_conformal['max_error']:.6f} m")
    
    if 'mean_isotropy_error' in metrics_conformal:
        print(f"   Isotropy error: {metrics_conformal['mean_isotropy_error']:.6f}")
        print(f"   Orthogonality error: {metrics_conformal['mean_orthogonality_error']:.6f}")
        print(f"   Scale U: {metrics_conformal['mean_scale_u']:.6f}")
        print(f"   Scale V: {metrics_conformal['mean_scale_v']:.6f}")
    
    # Test equidistant spacing
    print("\n4. Testing equidistant spacing...")
    desired_spacing = 0.05  # 5cm
    
    if surf_conformal.metric_tensor is not None:
        spacing_u = surf_conformal.compute_equidistant_uv_spacing(desired_spacing, 'u')
        spacing_v = surf_conformal.compute_equidistant_uv_spacing(desired_spacing, 'v')
        
        print(f"   Desired surface spacing: {desired_spacing:.4f} m")
        print(f"   Required UV spacing in u-direction: {spacing_u:.6f}")
        print(f"   Required UV spacing in v-direction: {spacing_v:.6f}")
        
        # For simple parameterization, spacing is just used directly
        print(f"   Simple parameterization would use: {desired_spacing:.6f} (no correction)")
    
    # Visualization
    print("\n5. Creating visualization...")
    visualize_comparison(points, surf_simple, surf_conformal)
    
    print("\n" + "=" * 80)
    print("SUMMARY:")
    print("=" * 80)
    print(f"Simple Parameterization:")
    print(f"  - Uses direct PCA projection")
    print(f"  - UV spacing ≠ surface spacing on curved surfaces")
    print(f"  - RMSE: {metrics_simple['rmse']:.6f} m")
    print(f"\nConformal Parameterization (Amersdorfer et al.):")
    print(f"  - Uses metric-corrected parameterization")
    print(f"  - UV spacing adjusted for surface curvature")
    print(f"  - RMSE: {metrics_conformal['rmse']:.6f} m")
    if 'mean_isotropy_error' in metrics_conformal:
        print(f"  - Isotropy: {(1-metrics_conformal['mean_isotropy_error'])*100:.1f}%")
    print("=" * 80)


def visualize_comparison(points, surf_simple, surf_conformal):
    """
    Visualize the differences between simple and conformal parameterization.
    """
    fig = plt.figure(figsize=(15, 10))
    
    # 3D surface plot
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.scatter(points[:, 0], points[:, 1], points[:, 2], c='blue', alpha=0.5, s=1)
    ax1.set_title('Original 3D Surface')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    
    # Simple UV space
    ax2 = fig.add_subplot(2, 3, 2)
    uv_simple = surf_simple.uv_params
    ax2.scatter(uv_simple[:, 0], uv_simple[:, 1], c='green', alpha=0.5, s=1)
    ax2.set_title('Simple Projection UV Space')
    ax2.set_xlabel('U')
    ax2.set_ylabel('V')
    ax2.grid(True)
    ax2.set_aspect('equal')
    
    # Conformal UV space
    ax3 = fig.add_subplot(2, 3, 3)
    uv_conformal = surf_conformal.uv_params
    ax3.scatter(uv_conformal[:, 0], uv_conformal[:, 1], c='red', alpha=0.5, s=1)
    ax3.set_title('Conformal UV Space (Amersdorfer)')
    ax3.set_xlabel('U')
    ax3.set_ylabel('V')
    ax3.grid(True)
    ax3.set_aspect('equal')
    
    # Metric tensor visualization for conformal
    if surf_conformal.metric_tensor is not None:
        ax4 = fig.add_subplot(2, 3, 4)
        E_values = surf_conformal.metric_tensor[:, 0]
        scale_u = np.sqrt(E_values)
        scatter = ax4.scatter(uv_conformal[:, 0], uv_conformal[:, 1], 
                            c=scale_u, cmap='viridis', s=5)
        ax4.set_title('Scale Factor in U Direction (√E)')
        ax4.set_xlabel('U')
        ax4.set_ylabel('V')
        plt.colorbar(scatter, ax=ax4)
        
        ax5 = fig.add_subplot(2, 3, 5)
        G_values = surf_conformal.metric_tensor[:, 2]
        scale_v = np.sqrt(G_values)
        scatter = ax5.scatter(uv_conformal[:, 0], uv_conformal[:, 1], 
                            c=scale_v, cmap='viridis', s=5)
        ax5.set_title('Scale Factor in V Direction (√G)')
        ax5.set_xlabel('U')
        ax5.set_ylabel('V')
        plt.colorbar(scatter, ax=ax5)
        
        # Isotropy
        ax6 = fig.add_subplot(2, 3, 6)
        isotropy = np.abs(E_values - G_values) / (E_values + G_values + 1e-6)
        scatter = ax6.scatter(uv_conformal[:, 0], uv_conformal[:, 1], 
                            c=isotropy, cmap='RdYlGn_r', s=5, vmin=0, vmax=0.5)
        ax6.set_title('Anisotropy (lower is better)')
        ax6.set_xlabel('U')
        ax6.set_ylabel('V')
        plt.colorbar(scatter, ax=ax6)
    
    plt.tight_layout()
    plt.savefig('conformal_vs_simple_parameterization.png', dpi=150)
    print("   Saved visualization to: conformal_vs_simple_parameterization.png")
    plt.show()


def test_equidistant_paths():
    """
    Test generation of equidistant iso-parametric paths.
    """
    print("\n" + "=" * 80)
    print("Testing: Equidistant Iso-Parametric Path Generation")
    print("=" * 80)
    
    # Import here to avoid issues if not available
    try:
        sys.path.insert(0, '../../path_planning/path_planning')
        from equidistant_path_planner import EquidistantPathPlanner
    except ImportError:
        print("Could not import EquidistantPathPlanner")
        return
    
    # Generate surface
    points = generate_curved_surface()
    
    # Create conformal parameterization
    surf = ConformalParameterization()
    surf.set_points(points)
    surf.compute_local_frame()
    surf.compute_initial_parameterization(method='projection')
    surf.compute_surface_metric(k_neighbors=20)
    surf.apply_conformal_correction(iterations=5, alpha=0.5)
    surf.build_inverse_interpolation(method='rbf', neighbors=50)
    
    # Create path planner
    planner = EquidistantPathPlanner(surf)
    
    # Generate equidistant paths
    print("\n1. Generating iso-parametric paths...")
    spacing = 0.05  # 5cm
    paths_uv = planner.generate_iso_parametric_paths(
        spacing=spacing,
        direction='u',
        line_density=100
    )
    
    print(f"   Generated {len(paths_uv)} paths")
    
    # Create continuous path
    print("\n2. Creating continuous path with transitions...")
    continuous_path = planner.create_continuous_path(bezier_points=20)
    print(f"   Continuous path has {len(continuous_path)} points")
    
    # Verify spacing
    print("\n3. Verifying spacing on surface...")
    spacing_metrics = planner.verify_spacing(continuous_path, n_samples=100)
    print(f"   Mean spacing: {spacing_metrics['mean_spacing']:.6f} m")
    print(f"   Std deviation: {spacing_metrics['std_spacing']:.6f} m")
    print(f"   Min spacing: {spacing_metrics['min_spacing']:.6f} m")
    print(f"   Max spacing: {spacing_metrics['max_spacing']:.6f} m")
    print(f"   Coefficient of variation: {spacing_metrics['cv']:.3f}")
    
    print("\n" + "=" * 80)


if __name__ == '__main__':
    try:
        # Test comparison
        test_simple_vs_conformal()
        
        # Test path generation
        # test_equidistant_paths()  # Uncomment if path planner is available
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
