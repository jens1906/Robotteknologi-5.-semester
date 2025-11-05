"""
This module provides surface parameterization functionality using inverse interpolation.
Maps 3D Cartesian points (x,y,z) to 2D parameter space (u,v) and provides interpolation.
"""

import numpy as np
from scipy.interpolate import RBFInterpolator
from scipy.spatial import cKDTree
from sklearn.decomposition import PCA


class SurfaceParameterization:
    """
    Surface parameterization using inverse interpolation approach.
    Maps 3D Cartesian points (x,y,z) to 2D parameter space (u,v).
    """
    
    def __init__(self):
        """Initialize the surface parameterization."""
        self.points = None
        self.uv_params = None
        self.interpolator_x = None
        self.interpolator_y = None
        self.interpolator_z = None
        self.principal_axes = None
        self.centroid = None
        self.points_local = None
        self.interpolation_method = None
        self.kdtree_uv = None
        self.neighbors = None
        self.stored_points = None
        self.is_ready = False
        
    def set_points(self, points):
        """Set point cloud data."""
        self.points = np.asarray(points, dtype=np.float64)
        self.is_ready = False
        
    def compute_local_frame(self):
        """Compute local coordinate frame using PCA."""
        if self.points is None or len(self.points) == 0:
            raise ValueError("No points available. Call set_points() first.")
            
        centroid = np.mean(self.points, axis=0)
        centered_points = self.points - centroid
        
        pca = PCA(n_components=3)
        pca.fit(centered_points)
        
        self.principal_axes = pca.components_
        self.centroid = centroid
        self.points_local = centered_points @ self.principal_axes.T
        
        return self.principal_axes, centroid
    
    def local_to_global(self, points_local):
        """
        Transform points from local coordinate frame back to global frame.
        
        Args:
            points_local: Nx3 array of points in local frame
            
        Returns:
            points_global: Nx3 array of points in global frame
        """
        if self.principal_axes is None or self.centroid is None:
            raise ValueError("Local frame not computed yet. Call compute_local_frame() first.")
        
        points_local = np.atleast_2d(points_local)
        points_global = points_local @ self.principal_axes + self.centroid
        
        return points_global
    
    def global_to_local(self, points_global):
        """
        Transform points from global coordinate frame to local frame.
        
        Args:
            points_global: Nx3 array of points in global frame
            
        Returns:
            points_local: Nx3 array of points in local frame
        """
        if self.principal_axes is None or self.centroid is None:
            raise ValueError("Local frame not computed yet. Call compute_local_frame() first.")
        
        points_global = np.atleast_2d(points_global)
        points_local = (points_global - self.centroid) @ self.principal_axes.T
        
        return points_local
    
    def compute_uv_parameterization(self, method='projection', normalize=False):
        """
        Compute UV parameterization mapping (x,y,z) → (u,v).
        
        Args:
            method: 'projection' (recommended for ROS integration)
            normalize: If False, keeps actual XYZ scale (recommended for ROS)
        """
        if self.points_local is None:
            self.compute_local_frame()
        
        if method == 'projection':
            uv = self.points_local[:, :2]
        
        if not normalize:
            # Keep actual XYZ scale (recommended for ROS)
            self.uv_params = uv
        else:
            # Normalize to [0:1] range
            uv_min = np.min(uv, axis=0)
            uv_max = np.max(uv, axis=0)
            uv_range = uv_max - uv_min
            uv_range[uv_range == 0] = 1
            self.uv_params = (uv - uv_min) / uv_range
                
        return self.uv_params
    
    def build_inverse_interpolation(self, method='rbf', neighbors=50):
        """
        Build inverse interpolation from (u,v) → (x,y,z).
        
        Args:
            method: 'rbf' for radial basis function interpolation
            neighbors: Number of neighbors for local RBF (default 50)
        """
        if self.uv_params is None:
            self.compute_uv_parameterization()
        
        if method == 'rbf':
            kernel = 'thin_plate_spline'
            
            if neighbors is not None:
                self.interpolation_method = 'rbf_local'
                self.kdtree_uv = cKDTree(self.uv_params)
                self.neighbors = neighbors
                self.stored_points = self.points.copy()
            else:
                self.interpolator_x = RBFInterpolator(
                    self.uv_params, self.points[:, 0], kernel=kernel
                )
                self.interpolator_y = RBFInterpolator(
                    self.uv_params, self.points[:, 1], kernel=kernel
                )
                self.interpolator_z = RBFInterpolator(
                    self.uv_params, self.points[:, 2], kernel=kernel
                )
                self.interpolation_method = 'rbf_global'
        else:
            self.interpolation_method = method
        
        self.is_ready = True
    
    def interpolate(self, uv_query):
        """
        Interpolate Cartesian coordinates from parameter space.
        Maps (u,v) → (x,y,z).
        
        Args:
            uv_query: Nx2 array of (u,v) coordinates
            
        Returns:
            xyz: Nx3 array of (x,y,z) coordinates
        """
        if not self.is_ready:
            raise ValueError("Interpolation not ready. Call build_inverse_interpolation() first.")
        
        uv_query = np.atleast_2d(uv_query)
        
        if self.interpolation_method == 'rbf_global':
            x = self.interpolator_x(uv_query)
            y = self.interpolator_y(uv_query)
            z = self.interpolator_z(uv_query)
            xyz = np.column_stack([x, y, z])
            
        elif self.interpolation_method == 'rbf_local':
            xyz = np.zeros((len(uv_query), 3))
            
            for i, uv in enumerate(uv_query):
                distances, indices = self.kdtree_uv.query(uv, k=min(self.neighbors, len(self.uv_params)))
                
                if distances[0] < 1e-10:
                    xyz[i] = self.stored_points[indices[0]]
                else:
                    weights = 1.0 / (distances + 1e-10)
                    weights = weights / weights.sum()
                    xyz[i] = (self.stored_points[indices].T @ weights).T
        else:
            raise ValueError(f"Unknown interpolation method: {self.interpolation_method}")
        
        return xyz
    
    def get_uv_bounds(self):
        """
        Get the bounds of the parameter space.
        
        Returns:
            dict with keys: u_min, u_max, v_min, v_max
        """
        if self.uv_params is None:
            raise ValueError("UV parameterization not computed. Call compute_uv_parameterization() first.")
        
        uv_min = np.min(self.uv_params, axis=0)
        uv_max = np.max(self.uv_params, axis=0)
        
        return {
            'u_min': float(uv_min[0]),
            'u_max': float(uv_max[0]),
            'v_min': float(uv_min[1]),
            'v_max': float(uv_max[1])
        }
    
    def evaluate_quality(self, sample_size=1000):
        """
        Evaluate parameterization quality.
        
        Args:
            sample_size: Number of points to sample for evaluation
            
        Returns:
            dict with quality metrics
        """
        if not self.is_ready:
            raise ValueError("Interpolation not ready. Call build_inverse_interpolation() first.")
        
        n_points = len(self.points)
        if n_points > sample_size:
            indices = np.random.choice(n_points, sample_size, replace=False)
            sample_uv = self.uv_params[indices]
            sample_points = self.points[indices]
        else:
            sample_uv = self.uv_params
            sample_points = self.points
        
        reconstructed = self.interpolate(sample_uv)
        errors = np.linalg.norm(sample_points - reconstructed, axis=1)
        
        metrics = {
            'mean_error': float(np.mean(errors)),
            'max_error': float(np.max(errors)),
            'std_error': float(np.std(errors)),
            'rmse': float(np.sqrt(np.mean(errors**2))),
            'sample_size': len(sample_points),
            'total_points': n_points
        }
        
        return metrics
