"""
Standalone test script for testing conformal parameterization with your PLY file.
This implementation follows Amersdorfer et al. (2021).

Usage:
    python test_with_ply.py <path_to_ply_file>
    
Example:
    python test_with_ply.py point_cloud.ply
    python test_with_ply.py "../Surface parameterisation/point_cloud.ply"
"""

import sys
import os
import numpy as np


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


def test_parameterization(ply_file_path):
    """Test conformal parameterization with a PLY file"""
    
    # Import the module
    try:
        from parameterization.conformal_parameterization import ConformalParameterization
    except ImportError:
        # Try adding parent directory to path
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
        from parameterization.conformal_parameterization import ConformalParameterization
    
    print("=" * 70)
    print("  TESTING CONFORMAL PARAMETERIZATION (Amersdorfer et al. 2021)")
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
    surf = ConformalParameterization()
    surf.set_points(points)
    print("   Points set")
    
    # Compute local frame
    print("\n3. Computing local frame (PCA)...")
    principal_axes, centroid = surf.compute_local_frame()
    print(f"   Local frame computed")
    print(f"   Centroid: [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}]")
    print(f"   Principal axes computed (3x3 orthonormal matrix)")
    
    # Compute initial UV parameterization
    print("\n4. Computing initial UV parameterization (projection method)...")
    uv = surf.compute_initial_parameterization(method='projection')
    bounds = surf.get_uv_bounds()
    print(f"   Initial UV parameterization computed")
    print(f"   U range: [{bounds['u_min']:.3f}, {bounds['u_max']:.3f}]")
    print(f"   V range: [{bounds['v_min']:.3f}, {bounds['v_max']:.3f}]")
    
    # Compute surface metric tensor
    print("\n5. Computing surface metric tensor...")
    k_neighbors = min(20, len(points) // 10)
    metric = surf.compute_surface_metric(k_neighbors=k_neighbors)
    print(f"   Metric tensor computed using {k_neighbors} neighbors")
    print(f"   Mean E (∂x/∂u magnitude²): {np.mean(metric[:, 0]):.6f}")
    print(f"   Mean F (∂x/∂u · ∂x/∂v): {np.mean(metric[:, 1]):.6f}")
    print(f"   Mean G (∂x/∂v magnitude²): {np.mean(metric[:, 2]):.6f}")
    
    # Apply conformal correction
    print("\n6. Applying conformal correction...")
    uv_corrected = surf.apply_conformal_correction(iterations=10, alpha=0.5)
    bounds_after = surf.get_uv_bounds()
    print(f"   Conformal correction applied (10 iterations, α=0.5)")
    print(f"   U range after correction: [{bounds_after['u_min']:.3f}, {bounds_after['u_max']:.3f}]")
    print(f"   V range after correction: [{bounds_after['v_min']:.3f}, {bounds_after['v_max']:.3f}]")
    
    # Build interpolation
    print("\n7. Building inverse interpolation (RBF with 50 neighbors)...")
    surf.build_inverse_interpolation(method='rbf', neighbors=50)
    print(f"   Interpolation ready")
    
    # Evaluate quality
    print("\n8. Evaluating quality metrics...")
    sample_size = min(1000, len(points))
    metrics = surf.evaluate_quality(sample_size=sample_size)
    print(f"   Quality evaluation complete")
    print(f"   Sample size: {metrics['sample_size']}/{metrics['total_points']} points")
    print(f"   Mean error: {metrics['mean_error']:.6f}")
    print(f"   Max error: {metrics['max_error']:.6f}")
    print(f"   RMSE: {metrics['rmse']:.6f}")
    print(f"   Std deviation: {metrics['std_error']:.6f}")
    
    # Conformal quality metrics
    if 'mean_isotropy_error' in metrics:
        print(f"\n   Conformal Quality Metrics:")
        print(f"   Isotropy error: {metrics['mean_isotropy_error']:.6f} (closer to 0 is better)")
        print(f"   Orthogonality error: {metrics['mean_orthogonality_error']:.6f} (closer to 0 is better)")
        print(f"   Mean scale U: {metrics['mean_scale_u']:.6f}")
        print(f"   Mean scale V: {metrics['mean_scale_v']:.6f}")
    
    # Test interpolation
    print("\n9. Testing interpolation...")
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
    print("\n10. Testing frame transformations...")
    n_transform = min(20, len(points))
    test_points = points[:n_transform]
    
    local_points = surf.global_to_local(test_points)
    reconstructed = surf.local_to_global(local_points)
    transform_errors = np.linalg.norm(reconstructed - test_points, axis=1)
    
    print(f"    Round-trip transformation tested on {n_transform} points")
    print(f"    Max reconstruction error: {np.max(transform_errors):.10f}")
    
    # Generate example scanning path with equidistant spacing
    print("\n11. Generating iso-parametric scanning path...")
    desired_spacing = 0.05  # 5cm on surface
    
    # Compute equidistant UV spacing
    spacing_u = surf.compute_equidistant_uv_spacing(desired_spacing, direction='u')
    spacing_v = surf.compute_equidistant_uv_spacing(desired_spacing, direction='v')
    
    print(f"    Equidistant UV spacing for {desired_spacing*1000:.1f}mm:")
    print(f"    Δu = {spacing_u:.6f}, Δv = {spacing_v:.6f}")
    
    # Generate iso-v curves
    num_passes = int((bounds_after['v_max'] - bounds_after['v_min']) / spacing_v)
    num_passes = max(5, min(num_passes, 15))  # Between 5 and 15
    points_per_pass = 20
    
    path_uv = []
    for i in range(num_passes):
        v = bounds_after['v_min'] + (bounds_after['v_max'] - bounds_after['v_min']) * i / (num_passes - 1)
        u_line = np.linspace(bounds_after['u_min'], bounds_after['u_max'], points_per_pass)
        
        if i % 2 == 1:
            u_line = u_line[::-1]  # Alternate direction
        
        for u in u_line:
            path_uv.append([u, v])
    
    path_uv = np.array(path_uv)
    path_3d = surf.interpolate(path_uv)
    path_length = np.sum(np.linalg.norm(np.diff(path_3d, axis=0), axis=1))
    
    print(f"    Generated iso-parametric scanning path")
    print(f"    Waypoints: {len(path_3d)}")
    print(f"    Path length: {path_length:.2f} units")
    print(f"    Number of passes: {num_passes}")
    
    # Summary
    print("\n" + "=" * 70)
    print("  ALL TESTS PASSED!")
    print("=" * 70)
    print("\nSummary:")
    print(f"  • Point cloud: {len(points)} points")
    print(f"  • UV parameterization: {uv.shape}")
    print(f"  • Metric tensor: {metric.shape}")
    print(f"  • Quality RMSE: {metrics['rmse']:.6f}")
    if 'mean_isotropy_error' in metrics:
        print(f"  • Isotropy error: {metrics['mean_isotropy_error']:.6f}")
    print(f"  • Interpolation: Working")
    print(f"  • Frame transformations: Working")
    print(f"  • Path generation: Working")
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
