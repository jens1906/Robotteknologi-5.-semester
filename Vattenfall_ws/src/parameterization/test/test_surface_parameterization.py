"""
Unit tests for surface_parameterization module.

Tests the core functionality of the SurfaceParameterization class including:
- Point cloud loading and initialization
- Local frame computation with PCA
- UV parameterization
- Inverse interpolation
- Surface normal computation
- Frame transformations
- Quality metrics
"""

import pytest
import numpy as np
import os
from parameterization.surface_parameterization import SurfaceParameterization


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


class TestSurfaceParameterization:
    """Test suite for SurfaceParameterization class"""
    
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
        """Create parameterization instance with simple plane"""
        surf = SurfaceParameterization()
        surf.set_points(simple_plane)
        surf.compute_local_frame()
        surf.compute_uv_parameterization(method='projection', normalize=False)
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        return surf
    
    @pytest.fixture
    def parameterization_curved(self, curved_surface):
        """Create parameterization instance with curved surface"""
        surf = SurfaceParameterization()
        surf.set_points(curved_surface)
        surf.compute_local_frame()
        surf.compute_uv_parameterization(method='projection', normalize=False)
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        return surf
    
    def test_initialization(self):
        """Test basic initialization"""
        surf = SurfaceParameterization()
        assert surf.points is None
        assert surf.uv_params is None
        assert surf.is_ready is False
    
    def test_set_points(self, simple_plane):
        """Test setting point cloud data"""
        surf = SurfaceParameterization()
        surf.set_points(simple_plane)
        
        assert surf.points is not None
        assert len(surf.points) == len(simple_plane)
        assert surf.points.shape[1] == 3  # XYZ coordinates
        assert surf.is_ready is False  # Not ready until interpolation built
    
    def test_set_points_empty(self):
        """Test error handling for empty points"""
        surf = SurfaceParameterization()
        surf.set_points(np.array([]))
        
        with pytest.raises(ValueError):
            surf.compute_local_frame()
    
    def test_compute_local_frame(self, simple_plane):
        """Test local frame computation using PCA"""
        surf = SurfaceParameterization()
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
        surf = SurfaceParameterization()
        surf.set_points(simple_plane)
        
        expected_centroid = np.mean(simple_plane, axis=0)
        _, centroid = surf.compute_local_frame()
        
        np.testing.assert_allclose(centroid, expected_centroid, rtol=1e-10)
    
    def test_global_to_local_to_global(self, simple_plane):
        """Test round-trip transformation: global -> local -> global"""
        surf = SurfaceParameterization()
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
    
    def test_compute_uv_parameterization(self, simple_plane):
        """Test UV parameterization computation"""
        surf = SurfaceParameterization()
        surf.set_points(simple_plane)
        surf.compute_local_frame()
        
        uv = surf.compute_uv_parameterization(method='projection', normalize=False)
        
        # Check dimensions
        assert uv.shape == (len(simple_plane), 2)
        assert surf.uv_params is not None
        
        # Check that UV parameters are reasonable
        assert np.all(np.isfinite(uv))
    
    def test_compute_uv_parameterization_normalized(self, simple_plane):
        """Test normalized UV parameterization"""
        surf = SurfaceParameterization()
        surf.set_points(simple_plane)
        surf.compute_local_frame()
        
        uv = surf.compute_uv_parameterization(method='projection', normalize=True)
        
        # Check that UV is in [0, 1] range
        assert np.all(uv >= 0.0)
        assert np.all(uv <= 1.0)
        
        # Check that min and max are close to bounds
        uv_min = np.min(uv, axis=0)
        uv_max = np.max(uv, axis=0)
        assert np.allclose(uv_min, 0.0, atol=1e-10)
        assert np.allclose(uv_max, 1.0, atol=1e-10)
    
    def test_build_inverse_interpolation(self, simple_plane):
        """Test building inverse interpolation"""
        surf = SurfaceParameterization()
        surf.set_points(simple_plane)
        surf.compute_local_frame()
        surf.compute_uv_parameterization()
        
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
        surf = SurfaceParameterization()
        surf.set_points(simple_plane)
        
        with pytest.raises(ValueError, match="Interpolation not ready"):
            surf.interpolate(np.array([[0.0, 0.0]]))
    
    def test_compute_surface_normals(self, parameterization_simple):
        """Test surface normal computation"""
        surf = parameterization_simple
        
        # Get center UV coordinate
        uv_center = np.mean(surf.uv_params, axis=0, keepdims=True)
        
        # Compute normals
        normals = surf.compute_surface_normals(uv_center)
        
        assert normals.shape == (1, 3)
        
        # Normal should be unit length
        norm_length = np.linalg.norm(normals[0])
        assert np.isclose(norm_length, 1.0, rtol=1e-6)
    
    def test_compute_surface_normals_flat_plane(self, parameterization_simple):
        """Test normals on a flat plane should point in same direction"""
        surf = parameterization_simple
        
        # Sample several points
        sample_indices = np.random.choice(len(surf.uv_params), 10, replace=False)
        sample_uv = surf.uv_params[sample_indices]
        
        normals = surf.compute_surface_normals(sample_uv)
        
        # All normals should be similar (plane is flat)
        for i in range(len(normals) - 1):
            dot_product = np.dot(normals[i], normals[i + 1])
            # Normals should be nearly parallel
            assert dot_product > 0.9  # Allow some numerical variation
    
    def test_compute_surface_normals_multiple(self, parameterization_curved):
        """Test normal computation for multiple points"""
        surf = parameterization_curved
        
        n_samples = 20
        indices = np.random.choice(len(surf.uv_params), n_samples, replace=False)
        sample_uv = surf.uv_params[indices]
        
        normals = surf.compute_surface_normals(sample_uv)
        
        assert normals.shape == (n_samples, 3)
        
        # All normals should be unit length
        for normal in normals:
            norm_length = np.linalg.norm(normal)
            assert np.isclose(norm_length, 1.0, rtol=1e-6)
    
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
        surf = SurfaceParameterization()
        
        with pytest.raises(ValueError, match="UV parameterization not computed"):
            surf.get_uv_bounds()
    
    def test_evaluate_quality(self, parameterization_curved):
        """Test quality metric evaluation"""
        surf = parameterization_curved
        
        metrics = surf.evaluate_quality(sample_size=100)
        
        # Check all metrics are present
        assert 'mean_error' in metrics
        assert 'max_error' in metrics
        assert 'std_error' in metrics
        assert 'rmse' in metrics
        assert 'sample_size' in metrics
        assert 'total_points' in metrics
        
        # Check values are reasonable
        assert metrics['mean_error'] >= 0
        assert metrics['max_error'] >= metrics['mean_error']
        assert metrics['std_error'] >= 0
        assert metrics['rmse'] >= 0
        assert metrics['sample_size'] <= metrics['total_points']
    
    def test_evaluate_quality_small_dataset(self, simple_plane):
        """Test quality evaluation with small dataset"""
        surf = SurfaceParameterization()
        surf.set_points(simple_plane[:50])  # Only 50 points
        surf.compute_local_frame()
        surf.compute_uv_parameterization()
        surf.build_inverse_interpolation(method='rbf', neighbors=10)
        
        # Request more samples than available
        metrics = surf.evaluate_quality(sample_size=1000)
        
        # Should use all available points
        assert metrics['sample_size'] == 50
    
    def test_workflow_complete(self, curved_surface):
        """Test complete workflow from start to finish"""
        # Initialize
        surf = SurfaceParameterization()
        assert not surf.is_ready
        
        # Set points
        surf.set_points(curved_surface)
        assert surf.points is not None
        
        # Compute local frame
        surf.compute_local_frame()
        assert surf.principal_axes is not None
        assert surf.centroid is not None
        
        # Compute UV parameterization
        surf.compute_uv_parameterization(method='projection', normalize=False)
        assert surf.uv_params is not None
        
        # Build interpolation
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        assert surf.is_ready
        
        # Test interpolation
        uv_test = surf.uv_params[:5]
        xyz = surf.interpolate(uv_test)
        assert xyz.shape == (5, 3)
        
        # Test normals
        normals = surf.compute_surface_normals(uv_test)
        assert normals.shape == (5, 3)
        
        # Evaluate quality
        metrics = surf.evaluate_quality(sample_size=100)
        assert metrics['rmse'] >= 0


class TestEdgeCases:
    """Test edge cases and error handling"""
    
    def test_frame_transformation_before_computation(self):
        """Test error when transforming before computing frame"""
        surf = SurfaceParameterization()
        
        with pytest.raises(ValueError, match="Local frame not computed"):
            surf.local_to_global(np.array([[1, 2, 3]]))
        
        with pytest.raises(ValueError, match="Local frame not computed"):
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
        
        surf = SurfaceParameterization()
        surf.set_points(points)
        
        # Should handle gracefully
        surf.compute_local_frame()
        surf.compute_uv_parameterization()
        
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
        """Test complete parameterization workflow with real point cloud"""
        print(f"\nTesting with {len(real_point_cloud)} real points...")
        
        # Initialize
        surf = SurfaceParameterization()
        surf.set_points(real_point_cloud)
        
        # Compute local frame
        principal_axes, centroid = surf.compute_local_frame()
        assert principal_axes is not None
        assert centroid is not None
        print(f"Centroid: {centroid}")
        
        # Compute UV parameterization
        uv = surf.compute_uv_parameterization(method='projection', normalize=False)
        assert uv is not None
        bounds = surf.get_uv_bounds()
        print(f"UV bounds: U=[{bounds['u_min']:.3f}, {bounds['u_max']:.3f}], "
              f"V=[{bounds['v_min']:.3f}, {bounds['v_max']:.3f}]")
        
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
        
        # Quality should be reasonable
        assert metrics['rmse'] < 1.0  # Adjust threshold based on your data
    
    def test_interpolation_accuracy_real_data(self, real_point_cloud):
        """Test interpolation accuracy on real data"""
        surf = SurfaceParameterization()
        surf.set_points(real_point_cloud)
        surf.compute_local_frame()
        surf.compute_uv_parameterization(method='projection', normalize=False)
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
    
    def test_surface_normals_real_data(self, real_point_cloud):
        """Test surface normal computation on real data"""
        surf = SurfaceParameterization()
        surf.set_points(real_point_cloud)
        surf.compute_local_frame()
        surf.compute_uv_parameterization(method='projection', normalize=False)
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        
        # Sample some UV points
        n_samples = min(10, len(real_point_cloud))
        sample_indices = np.random.choice(len(surf.uv_params), n_samples, replace=False)
        sample_uv = surf.uv_params[sample_indices]
        
        # Compute normals
        normals = surf.compute_surface_normals(sample_uv)
        
        assert normals.shape == (n_samples, 3)
        
        print(f"\nSurface normals test on {n_samples} samples:")
        # All normals should be unit length
        for i, normal in enumerate(normals):
            norm_length = np.linalg.norm(normal)
            print(f"  Normal {i}: ({normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f}), length: {norm_length:.6f}")
            assert np.isclose(norm_length, 1.0, rtol=1e-5)
    
    def test_frame_transformations_real_data(self, real_point_cloud):
        """Test frame transformations on real data"""
        surf = SurfaceParameterization()
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
        """Test generating a scanning path on real data"""
        surf = SurfaceParameterization()
        surf.set_points(real_point_cloud)
        surf.compute_local_frame()
        surf.compute_uv_parameterization(method='projection', normalize=False)
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
        
        # Compute normals along path
        normals = surf.compute_surface_normals(path_uv)
        
        # Calculate path length
        path_length = np.sum(np.linalg.norm(np.diff(path_3d, axis=0), axis=1))
        
        print(f"\nGenerated scanning path:")
        print(f"  Waypoints: {len(path_3d)}")
        print(f"  Path length: {path_length:.2f} units")
        
        assert len(path_3d) == len(path_uv)
        assert len(normals) == len(path_uv)
        assert path_length > 0


if __name__ == '__main__':
    # Run tests with pytest
    pytest.main([__file__, '-v'])
