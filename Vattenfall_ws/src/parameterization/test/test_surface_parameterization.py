"""
Unit tests for conformal surface parameterization module.

Tests the core functionality of the ConformalParameterization class following
Amersdorfer et al. (2021) approach including:
- Point cloud loading and initialization
- Local frame computation with PCA
- Conformal UV parameterization with metric tensor
- Inverse interpolation
- Frame transformations
- Quality metrics (RMSE, isotropy, orthogonality)
- Equidistant spacing computation
"""

import pytest
import numpy as np
import os
from parameterization.conformal_parameterization import ConformalParameterization


def load_ply_file(filepath):
    """
    Load point cloud from PLY file using Open3D.
    Returns None if file doesn't exist or Open3D is not available.
    """
    if not os.path.exists(filepath):
        return None
    
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(filepath)
        points = np.asarray(pcd.points)
        return points if len(points) > 0 else None
    except ImportError:
        print("Warning: Open3D not installed. PLY tests will be skipped.")
        return None
    except Exception as e:
        print(f"Warning: Could not load PLY file: {e}")
        return None


class TestConformalParameterization:
    """Test suite for ConformalParameterization class (Amersdorfer et al. 2021)"""
    
    @pytest.fixture
    def simple_plane(self):
        """Create a simple planar surface for testing"""
        x = np.linspace(-5, 5, 20)
        y = np.linspace(-5, 5, 20)
        X, Y = np.meshgrid(x, y)
        Z = np.ones_like(X) * 2.0  # Flat plane at z=2
        
        points = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
        return points
    
    @pytest.fixture
    def curved_surface(self):
        """Create a curved surface for testing"""
        x = np.linspace(-5, 5, 30)
        y = np.linspace(-5, 5, 30)
        X, Y = np.meshgrid(x, y)
        Z = 2 * np.sin(X) * np.cos(Y) + 5  # Wavy surface
        
        points = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
        return points
    
    @pytest.fixture
    def parameterization_simple(self, simple_plane):
        """Create conformal parameterization instance with simple plane"""
        surf = ConformalParameterization()
        surf.set_points(simple_plane)
        surf.compute_local_frame()
        surf.compute_initial_parameterization(method='projection')
        surf.compute_surface_metric(k_neighbors=20)
        surf.apply_conformal_correction(iterations=5, alpha=0.5)
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        return surf
    
    @pytest.fixture
    def parameterization_curved(self, curved_surface):
        """Create conformal parameterization instance with curved surface"""
        surf = ConformalParameterization()
        surf.set_points(curved_surface)
        surf.compute_local_frame()
        surf.compute_initial_parameterization(method='projection')
        surf.compute_surface_metric(k_neighbors=20)
        surf.apply_conformal_correction(iterations=5, alpha=0.5)
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        return surf
    
    def test_initialization(self):
        """Test basic initialization"""
        surf = ConformalParameterization()
    def test_set_points(self, simple_plane):
        """Test setting point cloud data"""
        surf = ConformalParameterization()
        surf.set_points(simple_plane)
        
        assert surf.points is not None
        assert len(surf.points) == len(simple_plane)
        assert surf.points.shape[1] == 3  # XYZ coordinates
        assert surf.is_ready is False  # Not ready until interpolation built
    
    def test_set_points_empty(self):
        """Test error handling for empty points"""
        surf = ConformalParameterization()
        surf.set_points(np.array([]))
        
        with pytest.raises(ValueError):
            surf.compute_local_frame()
    
    def test_compute_local_frame(self, simple_plane):
        """Test local frame computation using PCA"""
        surf = ConformalParameterization()
        surf.set_points(simple_plane)
        
        principal_axes, centroid = surf.compute_local_frame()
        
        # Check dimensions
        assert principal_axes.shape == (3, 3)
        assert centroid.shape == (3,)
        
        # Check orthogonality of principal axes
        for i in range(3):
            for j in range(3):
                if i == j:
                    # Diagonal should be ~1 (normalized)
                    dot = np.dot(principal_axes[i], principal_axes[j])
                    assert np.isclose(dot, 1.0, atol=1e-6)
                else:
                    # Off-diagonal should be ~0 (orthogonal)
                    dot = np.dot(principal_axes[i], principal_axes[j])
                    assert np.isclose(dot, 0.0, atol=1e-6)
        
        # Check that local points were computed
        assert surf.points_local is not None
        assert surf.points_local.shape == simple_plane.shape
    
    def test_compute_local_frame_centroid(self, simple_plane):
        """Test that centroid is correctly computed"""
        surf = ConformalParameterization()
        surf.set_points(simple_plane)
        
        expected_centroid = np.mean(simple_plane, axis=0)
        _, centroid = surf.compute_local_frame()
        
        np.testing.assert_allclose(centroid, expected_centroid, rtol=1e-10)
    
    def test_global_to_local_to_global(self, simple_plane):
        """Test round-trip transformation: global -> local -> global"""
        surf = ConformalParameterization()
        surf.set_points(simple_plane)
        surf.compute_local_frame()
        
        # Take a subset of points
        test_points = simple_plane[:10]
        
        # Transform to local and back
        local_points = surf.global_to_local(test_points)
        reconstructed = surf.local_to_global(local_points)
        
        # Should match original
        np.testing.assert_allclose(reconstructed, test_points, rtol=1e-10)
    
    def test_local_to_global_single_point(self, parameterization_simple):
        """Test frame transformation for a single point"""
        # Test with a single point
        local_pt = np.array([[1.0, 2.0, 3.0]])
        global_pt = parameterization_simple.local_to_global(local_pt)
        
        assert global_pt.shape == (1, 3)
        
        # Transform back
        local_again = parameterization_simple.global_to_local(global_pt)
        np.testing.assert_allclose(local_again, local_pt, rtol=1e-10)
    
    def test_compute_initial_parameterization(self, simple_plane):
        """Test initial UV parameterization computation"""
        surf = ConformalParameterization()
        surf.set_points(simple_plane)
        surf.compute_local_frame()
        
        uv = surf.compute_initial_parameterization(method='projection')
        
        # Check dimensions
        assert uv.shape == (len(simple_plane), 2)
        assert surf.uv_params is not None
        
        # Check that UV parameters are reasonable
        assert np.all(np.isfinite(uv))
    
    def test_compute_surface_metric(self, simple_plane):
        """Test surface metric tensor computation"""
        surf = ConformalParameterization()
        surf.set_points(simple_plane)
        surf.compute_local_frame()
        surf.compute_initial_parameterization(method='projection')
        
        metric = surf.compute_surface_metric(k_neighbors=10)
        
        # Check dimensions: Nx3 array [E, F, G]
        assert metric.shape == (len(simple_plane), 3)
        assert surf.metric_tensor is not None
        
        # Check all values are finite and positive (E, G should be positive)
        assert np.all(np.isfinite(metric))
        assert np.all(metric[:, 0] > 0)  # E > 0
        assert np.all(metric[:, 2] > 0)  # G > 0
    
    def test_apply_conformal_correction(self, simple_plane):
        """Test conformal correction application"""
        surf = ConformalParameterization()
        surf.set_points(simple_plane)
        surf.compute_local_frame()
        surf.compute_initial_parameterization(method='projection')
        surf.compute_surface_metric(k_neighbors=10)
        
        uv_before = surf.uv_params.copy()
        
        # Apply correction
        surf.apply_conformal_correction(iterations=5, alpha=0.5)
        
        # Check that correction was applied  
        assert surf.uv_params is not None
        assert surf.uv_params.shape == uv_before.shape
    
    def test_build_inverse_interpolation(self, simple_plane):
        """Test building inverse interpolation"""
        surf = ConformalParameterization()
        surf.set_points(simple_plane)
        surf.compute_local_frame()
        surf.compute_initial_parameterization()
        
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        
        assert surf.is_ready is True
        assert surf.kdtree_uv is not None
        assert surf.neighbors == 50
        
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        
        assert surf.is_ready is True
        assert surf.interpolation_method == 'rbf_local'
        assert surf.kdtree_uv is not None
        assert surf.neighbors == 50
    
    def test_interpolate_exact_points(self, parameterization_simple):
        """Test interpolation at exact input points"""
        surf = parameterization_simple
        
        # Sample some UV coordinates from the original data
        sample_indices = [0, 50, 100, 150]
        sample_uv = surf.uv_params[sample_indices]
        expected_xyz = surf.points[sample_indices]
        
        # Interpolate
        interpolated = surf.interpolate(sample_uv)
        
        # Should be very close to original points
        np.testing.assert_allclose(interpolated, expected_xyz, rtol=1e-3, atol=1e-3)
    
    def test_interpolate_single_point(self, parameterization_simple):
        """Test interpolation of a single point"""
        surf = parameterization_simple
        
        # Get center UV coordinate
        uv_center = np.mean(surf.uv_params, axis=0, keepdims=True)
        
        # Interpolate
        xyz = surf.interpolate(uv_center)
        
        assert xyz.shape == (1, 3)
        assert np.all(np.isfinite(xyz))
    
    def test_interpolate_multiple_points(self, parameterization_curved):
        """Test interpolation of multiple points"""
        surf = parameterization_curved
        
        # Sample several UV coordinates
        n_samples = 10
        indices = np.random.choice(len(surf.uv_params), n_samples, replace=False)
        sample_uv = surf.uv_params[indices]
        
        # Interpolate
        xyz = surf.interpolate(sample_uv)
        
        assert xyz.shape == (n_samples, 3)
        assert np.all(np.isfinite(xyz))
    
    def test_interpolate_not_ready(self, simple_plane):
        """Test that interpolation fails when not ready"""
        surf = ConformalParameterization()
        surf.set_points(simple_plane)
        
        with pytest.raises(ValueError, match="Interpolation not ready"):
            surf.interpolate(np.array([[0.0, 0.0]]))
    
    def test_get_uv_bounds(self, parameterization_simple):
        """Test getting UV parameter space bounds"""
        surf = parameterization_simple
        
        bounds = surf.get_uv_bounds()
        
        assert 'u_min' in bounds
        assert 'u_max' in bounds
        assert 'v_min' in bounds
        assert 'v_max' in bounds
        
        # Min should be less than max
        assert bounds['u_min'] < bounds['u_max']
        assert bounds['v_min'] < bounds['v_max']
    
    def test_get_uv_bounds_not_computed(self):
        """Test error when getting bounds before UV computation"""
        surf = ConformalParameterization()
        
        with pytest.raises(ValueError):
            surf.get_uv_bounds()
    
    def test_evaluate_quality(self, parameterization_curved):
        """Test quality metric evaluation with conformal metrics"""
        surf = parameterization_curved
        
        metrics = surf.evaluate_quality(sample_size=100)
        
        # Check all metrics are present
        assert 'mean_error' in metrics
        assert 'max_error' in metrics
        assert 'std_error' in metrics
        assert 'rmse' in metrics
        assert 'sample_size' in metrics
        assert 'total_points' in metrics
        
        # Check conformal-specific metrics
        if surf.metric_tensor is not None:
            assert 'mean_isotropy_error' in metrics
            assert 'mean_orthogonality_error' in metrics
            assert 'mean_scale_u' in metrics
            assert 'mean_scale_v' in metrics
        
    def test_evaluate_quality_small_dataset(self, simple_plane):
        """Test quality evaluation with small dataset"""
        surf = ConformalParameterization()
        surf.set_points(simple_plane[:50])  # Only 50 points
        surf.compute_local_frame()
        surf.compute_initial_parameterization()
        surf.build_inverse_interpolation(method='rbf', neighbors=10)
        
        # Request more samples than available
        metrics = surf.evaluate_quality(sample_size=1000)
        
        # Should use all available points
        assert metrics['sample_size'] == 50
        surf.build_inverse_interpolation(method='rbf', neighbors=10)
        
        # Request more samples than available
        metrics = surf.evaluate_quality(sample_size=1000)
        
        # Should use all available points
        assert metrics['sample_size'] == 50
    
    def test_workflow_complete(self, curved_surface):
        """Test complete conformal parameterization workflow from start to finish"""
        # Initialize
        surf = ConformalParameterization()
        assert not surf.is_ready
        
        # Set points
        surf.set_points(curved_surface)
        assert surf.points is not None
        
        # Compute local frame
        surf.compute_local_frame()
        assert surf.principal_axes is not None
        assert surf.centroid is not None
        
        # Compute initial UV parameterization
        surf.compute_initial_parameterization(method='projection')
        assert surf.uv_params is not None
        
        # Compute surface metric
        surf.compute_surface_metric(k_neighbors=20)
        assert surf.metric_tensor is not None
        
        # Apply conformal correction
        surf.apply_conformal_correction(iterations=5, alpha=0.5)
        assert surf.uv_params is not None
        
        # Build interpolation
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        assert surf.is_ready
        
        # Test interpolation
        uv_test = surf.uv_params[:5]
        xyz = surf.interpolate(uv_test)
        assert xyz.shape == (5, 3)
        
        # Evaluate quality
        metrics = surf.evaluate_quality(sample_size=100)
        assert metrics['rmse'] >= 0
    
    def test_equidistant_uv_spacing(self, parameterization_simple):
        """Test computation of equidistant UV spacing"""
        surf = parameterization_simple
        
        # Compute required UV spacing for 5cm surface distance
        desired_spacing = 0.05
        spacing_u = surf.compute_equidistant_uv_spacing(desired_spacing, 'u')
        spacing_v = surf.compute_equidistant_uv_spacing(desired_spacing, 'v')
        
        # Spacing should be positive
        assert spacing_u > 0
        assert spacing_v > 0
        assert np.isfinite(spacing_u)
        assert np.isfinite(spacing_v)
    
    def test_surface_distance_calculation(self, parameterization_simple):
        """Test surface distance calculation using metric tensor"""
        surf = parameterization_simple
        
        if surf.metric_tensor is None:
            pytest.skip("Metric tensor not computed")
        
        # Pick two nearby UV points
        uv1 = surf.uv_params[0]
        uv2 = surf.uv_params[1]
        
        # Calculate surface distance
        distance = surf.get_surface_distance(uv1, uv2)
        
        # Distance should be positive and finite
        assert distance > 0
        assert np.isfinite(distance)


class TestEdgeCases:
    """Test edge cases and error handling"""
    
    def test_frame_transformation_before_computation(self):
        """Test error when transforming before computing frame"""
        surf = ConformalParameterization()
        
        with pytest.raises(ValueError):
            surf.local_to_global(np.array([[1, 2, 3]]))
        
        with pytest.raises(ValueError):
            surf.global_to_local(np.array([[1, 2, 3]]))
    
    def test_interpolate_outside_bounds(self, parameterization_simple):
        """Test interpolation outside UV bounds (should still work)"""
        surf = parameterization_simple
        bounds = surf.get_uv_bounds()
        
        # Query point outside bounds
        uv_outside = np.array([[
            bounds['u_max'] + 10,
            bounds['v_max'] + 10
        ]])
        
        # Should not crash (extrapolation)
        xyz = surf.interpolate(uv_outside)
        assert xyz.shape == (1, 3)
        assert np.all(np.isfinite(xyz))
    
    def test_degenerate_surface(self):
        """Test with degenerate surface (all same points)"""
        points = np.ones((100, 3))  # All points at (1, 1, 1)
        
        surf = ConformalParameterization()
        surf.set_points(points)
        
        # Should handle gracefully
        surf.compute_local_frame()
        surf.compute_initial_parameterization()
        
        # UV params might be degenerate but shouldn't crash
        assert surf.uv_params is not None


class TestRealPointCloud:
    """Test with real point cloud data from PLY files"""
    
    @pytest.fixture
    def ply_file_path(self):
        """
        Path to PLY file for testing.
        The PLY file should be placed in the test folder.
        """
        # Primary location: test folder
        test_dir = os.path.dirname(__file__)
        ply_path = os.path.join(test_dir, "point_cloud.ply")
        
        if os.path.exists(ply_path):
            return ply_path
        
        # If not found, return None (will skip test)
        return None
    
    @pytest.fixture
    def real_point_cloud(self, ply_file_path):
        """Load real point cloud from PLY file"""
        if ply_file_path is None:
            pytest.skip("PLY file not found. Provide path to point_cloud.ply")
        
        points = load_ply_file(ply_file_path)
        if points is None:
            pytest.skip("Could not load PLY file or Open3D not installed")
        
        return points
    
    def test_load_real_ply_file(self, real_point_cloud):
        """Test that we can load a real PLY file"""
        assert real_point_cloud is not None
        assert len(real_point_cloud) > 0
        assert real_point_cloud.shape[1] == 3
        
        print(f"\nLoaded point cloud with {len(real_point_cloud)} points")
        print(f"X range: [{np.min(real_point_cloud[:, 0]):.3f}, {np.max(real_point_cloud[:, 0]):.3f}]")
        print(f"Y range: [{np.min(real_point_cloud[:, 1]):.3f}, {np.max(real_point_cloud[:, 1]):.3f}]")
        print(f"Z range: [{np.min(real_point_cloud[:, 2]):.3f}, {np.max(real_point_cloud[:, 2]):.3f}]")
    
    def test_full_workflow_with_real_data(self, real_point_cloud):
        """Test complete conformal parameterization workflow with real point cloud"""
        print(f"\nTesting Amersdorfer approach with {len(real_point_cloud)} real points...")
        
        # Initialize
        surf = ConformalParameterization()
        surf.set_points(real_point_cloud)
        
        # Compute local frame
        principal_axes, centroid = surf.compute_local_frame()
        assert principal_axes is not None
        assert centroid is not None
        print(f"Centroid: {centroid}")
        
        # Compute initial UV parameterization
        uv = surf.compute_initial_parameterization(method='projection')
        assert uv is not None
        bounds = surf.get_uv_bounds()
        print(f"UV bounds: U=[{bounds['u_min']:.3f}, {bounds['u_max']:.3f}], "
              f"V=[{bounds['v_min']:.3f}, {bounds['v_max']:.3f}]")
        
        # Compute surface metric
        surf.compute_surface_metric(k_neighbors=20)
        assert surf.metric_tensor is not None
        print("Surface metric tensor computed")
        
        # Apply conformal correction
        surf.apply_conformal_correction(iterations=5, alpha=0.5)
        print("Conformal correction applied")
        
        # Build interpolation
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        assert surf.is_ready
        
        # Evaluate quality
        metrics = surf.evaluate_quality(sample_size=min(1000, len(real_point_cloud)))
        print(f"Quality metrics:")
        print(f"  Mean error: {metrics['mean_error']:.6f}")
        print(f"  Max error: {metrics['max_error']:.6f}")
        print(f"  RMSE: {metrics['rmse']:.6f}")
        print(f"  Std error: {metrics['std_error']:.6f}")
        
        if 'mean_isotropy_error' in metrics:
            print(f"  Isotropy error: {metrics['mean_isotropy_error']:.6f}")
            print(f"  Orthogonality error: {metrics['mean_orthogonality_error']:.6f}")
            print(f"  Scale U: {metrics['mean_scale_u']:.6f}")
            print(f"  Scale V: {metrics['mean_scale_v']:.6f}")
        
        # Quality should be reasonable
        assert metrics['rmse'] < 1.0  # Adjust threshold based on your data
    
    def test_interpolation_accuracy_real_data(self, real_point_cloud):
        """Test interpolation accuracy on real data"""
        surf = ConformalParameterization()
        surf.set_points(real_point_cloud)
        surf.compute_local_frame()
        surf.compute_initial_parameterization(method='projection')
        surf.compute_surface_metric(k_neighbors=20)
        surf.apply_conformal_correction(iterations=5, alpha=0.5)
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        
        # Sample some points
        n_samples = min(20, len(real_point_cloud))
        sample_indices = np.random.choice(len(real_point_cloud), n_samples, replace=False)
        sample_uv = surf.uv_params[sample_indices]
        expected_xyz = surf.points[sample_indices]
        
        # Interpolate
        interpolated = surf.interpolate(sample_uv)
        
        # Compute errors
        errors = np.linalg.norm(interpolated - expected_xyz, axis=1)
        mean_error = np.mean(errors)
        max_error = np.max(errors)
        
        print(f"\nInterpolation test on {n_samples} samples:")
        print(f"  Mean error: {mean_error:.6f}")
        print(f"  Max error: {max_error:.6f}")
        
        # Errors should be small for exact points
        assert mean_error < 0.1  # Adjust based on your data scale
    
    def test_conformal_metrics_real_data(self, real_point_cloud):
        """Test conformal parameterization metrics on real data"""
        surf = ConformalParameterization()
        surf.set_points(real_point_cloud)
        surf.compute_local_frame()
        surf.compute_initial_parameterization(method='projection')
        surf.compute_surface_metric(k_neighbors=20)
        surf.apply_conformal_correction(iterations=5, alpha=0.5)
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        
        # Sample some UV points
        n_samples = min(10, len(real_point_cloud))
        sample_indices = np.random.choice(len(surf.uv_params), n_samples, replace=False)
        sample_uv = surf.uv_params[sample_indices]
        
        # Test interpolation
        xyz = surf.interpolate(sample_uv)
        assert xyz.shape == (n_samples, 3)
    
    def test_frame_transformations_real_data(self, real_point_cloud):
        """Test frame transformations on real data"""
        surf = ConformalParameterization()
        surf.set_points(real_point_cloud)
        surf.compute_local_frame()
        
        # Test round-trip transformation
        n_samples = min(50, len(real_point_cloud))
        test_points = real_point_cloud[:n_samples]
        
        local_points = surf.global_to_local(test_points)
        reconstructed = surf.local_to_global(local_points)
        
        # Should match original
        max_error = np.max(np.linalg.norm(reconstructed - test_points, axis=1))
        print(f"\nFrame transformation test:")
        print(f"  Max reconstruction error: {max_error:.10f}")
        
        np.testing.assert_allclose(reconstructed, test_points, rtol=1e-10, atol=1e-10)
    
    def test_generate_scanning_path_real_data(self, real_point_cloud):
        """Test generating a scanning path on real data with conformal parameterization"""
        surf = ConformalParameterization()
        surf.set_points(real_point_cloud)
        surf.compute_local_frame()
        surf.compute_initial_parameterization(method='projection')
        surf.compute_surface_metric(k_neighbors=20)
        surf.apply_conformal_correction(iterations=5, alpha=0.5)
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        
        # Get UV bounds
        bounds = surf.get_uv_bounds()
        
        # Generate a simple scanning path
        num_passes = 5
        points_per_pass = 10
        path_uv = []
        
        for i in range(num_passes):
            v = bounds['v_min'] + (bounds['v_max'] - bounds['v_min']) * i / (num_passes - 1)
            u_line = np.linspace(bounds['u_min'], bounds['u_max'], points_per_pass)
            
            if i % 2 == 1:
                u_line = u_line[::-1]
            
            for u in u_line:
                path_uv.append([u, v])
        
        path_uv = np.array(path_uv)
        
        # Interpolate to get 3D path
        path_3d = surf.interpolate(path_uv)
        
        # Calculate path length
        path_length = np.sum(np.linalg.norm(np.diff(path_3d, axis=0), axis=1))
        
        print(f"\nGenerated scanning path:")
        print(f"  Waypoints: {len(path_3d)}")
        print(f"  Path length: {path_length:.2f} units")
        
        assert len(path_3d) == len(path_uv)
        assert path_length > 0


if __name__ == '__main__':
    # Run tests with pytest
    pytest.main([__file__, '-v'])
