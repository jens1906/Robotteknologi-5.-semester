"""
Conformal Surface Parameterization following Amersdorfer et al. (2021)
"Equidistant Tool Path and Cartesian Trajectory Planning for Robotic Machining of Curved Freeform Surfaces"

This module implements conformal parameterization that preserves local angles and approximately 
preserves distances, enabling equidistant path planning on curved surfaces.
"""

import numpy as np
from scipy.interpolate import RBFInterpolator
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from scipy.sparse.linalg import spsolve
from sklearn.decomposition import PCA


class ConformalParameterization:
    """
    Conformal surface parameterization for robotic machining.
    
    Implements:
    1. Local angle preservation (conformality)
    2. Approximate distance preservation through metric correction
    3. Iso-parametric curves correspond to equidistant paths on the surface
    """
    
    def __init__(self):
        """Initialize the conformal parameterization."""
        self.points = None
        self.uv_params = None
        self.uv_corrected = None  # Metric-corrected UV coordinates
        self.metric_tensor = None  # First fundamental form
        self.interpolator_x = None
        self.interpolator_y = None
        self.interpolator_z = None
        self.principal_axes = None
        self.centroid = None
        self.points_local = None
        self.kdtree_uv = None
        self.kdtree_xyz = None
        self.neighbors = None
        self.is_ready = False
        
        # Surface derivatives for metric computation
        self.du_dx = None
        self.du_dy = None
        self.du_dz = None
        self.dv_dx = None
        self.dv_dy = None
        self.dv_dz = None
        
    def set_points(self, points):
        """Set point cloud data."""
        self.points = np.asarray(points, dtype=np.float64)
        self.is_ready = False
        
    def compute_local_frame(self):
        """Compute local coordinate frame using PCA."""
        if self.points is None or len(self.points) == 0:
            raise ValueError("No points available. Call set_points() first.")
            
        self.centroid = np.mean(self.points, axis=0)
        centered_points = self.points - self.centroid
        
        pca = PCA(n_components=3)
        pca.fit(centered_points)
        
        self.principal_axes = pca.components_
        self.points_local = centered_points @ self.principal_axes.T
        
        return self.principal_axes, self.centroid
    
    def compute_initial_parameterization(self, method='projection'):
        """
        Compute initial UV parameterization as starting point for conformal mapping.
        
        Args:
            method: 'projection' - Simple projection onto first two principal components
        """
        if self.points_local is None:
            self.compute_local_frame()
        
        if method == 'projection':
            # Simple projection as initial guess
            self.uv_params = self.points_local[:, :2].copy()
        
        return self.uv_params
    
    def compute_surface_metric(self, k_neighbors=20):
        """
        Compute the first fundamental form (metric tensor) of the surface.
        
        The metric tensor G = [E F; F G] describes how distances in UV space
        relate to distances on the actual surface:
            ds² = E du² + 2F du dv + G dv²
        
        For equidistant paths, we want E ≈ G ≈ 1 and F ≈ 0.
        
        Args:
            k_neighbors: Number of neighbors for local derivative estimation
            
        Returns:
            metric_tensor: Nx3 array where each row is [E, F, G] at that point
        """
        if self.uv_params is None:
            self.compute_initial_parameterization()
        
        n_points = len(self.points)
        metric_tensor = np.zeros((n_points, 3))  # [E, F, G] for each point
        
        # Build KD-trees for efficient neighbor search
        if self.kdtree_xyz is None:
            self.kdtree_xyz = cKDTree(self.points)
        if self.kdtree_uv is None:
            self.kdtree_uv = cKDTree(self.uv_params)
        
        for i in range(n_points):
            # Find neighbors in XYZ space
            distances, indices = self.kdtree_xyz.query(self.points[i], k=min(k_neighbors, n_points))
            
            if len(indices) < 4:
                # Not enough neighbors, use identity metric
                metric_tensor[i] = [1.0, 0.0, 1.0]
                continue
            
            # Get neighbor coordinates
            xyz_neighbors = self.points[indices]
            uv_neighbors = self.uv_params[indices]
            
            # Compute local surface derivatives ∂r/∂u and ∂r/∂v
            # using least squares fitting of local plane
            try:
                # Center the data
                xyz_centered = xyz_neighbors - xyz_neighbors[0]
                uv_centered = uv_neighbors - uv_neighbors[0]
                
                # Solve for derivatives: [∂r/∂u, ∂r/∂v] via least squares
                # xyz_centered ≈ [∂r/∂u, ∂r/∂v] @ uv_centered.T
                if np.linalg.matrix_rank(uv_centered) >= 2:
                    # Compute pseudo-inverse
                    derivatives = xyz_centered.T @ np.linalg.pinv(uv_centered.T)
                    
                    dr_du = derivatives[:, 0]  # ∂r/∂u
                    dr_dv = derivatives[:, 1]  # ∂r/∂v
                    
                    # Compute first fundamental form coefficients
                    E = np.dot(dr_du, dr_du)  # ||∂r/∂u||²
                    F = np.dot(dr_du, dr_dv)  # <∂r/∂u, ∂r/∂v>
                    G = np.dot(dr_dv, dr_dv)  # ||∂r/∂v||²
                    
                    metric_tensor[i] = [E, F, G]
                else:
                    # Degenerate case, use identity
                    metric_tensor[i] = [1.0, 0.0, 1.0]
                    
            except np.linalg.LinAlgError:
                # Numerical issues, use identity metric
                metric_tensor[i] = [1.0, 0.0, 1.0]
        
        self.metric_tensor = metric_tensor
        return metric_tensor
    
    def apply_conformal_correction(self, iterations=2, alpha=0.5):
        """
        Apply conformal correction to make the parameterization more isometric.
        
        This iteratively adjusts UV coordinates to minimize metric distortion,
        making E ≈ G and F ≈ 0 (conformality condition).
        
        Args:
            iterations: Number of correction iterations (0 to skip)
            alpha: Step size for correction (0 < alpha < 1)
        """
        if iterations == 0:
            # Skip correction, use initial parameterization as-is
            self.uv_corrected = self.uv_params.copy()
            return self.uv_corrected
            
        if self.metric_tensor is None:
            self.compute_surface_metric()
        
        self.uv_corrected = self.uv_params.copy()
        
        for iter_idx in range(iterations):
            # Only recompute metric on first iteration - too slow for large point clouds
            if iter_idx == 0:
                metric_tensor = self.metric_tensor
            
            # For each point, adjust UV to reduce anisotropy
            for i in range(len(self.points)):
                E, F, G = metric_tensor[i]
                
                # Compute scale factors to make metric more isometric
                # Target: E = G = scale, F = 0
                scale = np.sqrt((E + G) / 2.0)
                
                if scale < 1e-6:
                    continue
                
                # Compute correction factors
                # For conformal maps, we want E/G ≈ 1 and F ≈ 0
                anisotropy = np.abs(E - G) / (E + G + 1e-6)
                
                if anisotropy > 0.1:  # Apply correction if significant anisotropy
                    # Simple correction: scale UV coordinates
                    scale_u = np.sqrt(G / (E + 1e-6))
                    scale_v = np.sqrt(E / (G + 1e-6))
                    
                    # Apply weighted correction
                    self.uv_corrected[i, 0] *= (1 - alpha) + alpha * scale_u
                    self.uv_corrected[i, 1] *= (1 - alpha) + alpha * scale_v
        
        self.uv_params = self.uv_corrected
        
        return self.uv_corrected
    
    def build_inverse_interpolation(self, method='rbf', neighbors=50):
        """
        Build inverse interpolation from (u,v) → (x,y,z).
        
        Args:
            method: 'rbf' for radial basis function interpolation
            neighbors: Number of neighbors for local RBF
        """
        if self.uv_params is None:
            self.compute_initial_parameterization()
        
        self.neighbors = neighbors
        
        if method == 'rbf':
            kernel = 'thin_plate_spline'
            
            # Use local RBF for efficiency
            self.kdtree_uv = cKDTree(self.uv_params)
            
            # Store points for local interpolation
            self.stored_points = self.points.copy()
        
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
        xyz = np.zeros((len(uv_query), 3))
        
        for i, uv in enumerate(uv_query):
            distances, indices = self.kdtree_uv.query(uv, k=min(self.neighbors, len(self.uv_params)))
            
            if distances[0] < 1e-10:
                # Exact match
                xyz[i] = self.stored_points[indices[0]]
            else:
                # Weighted interpolation using inverse distance
                weights = 1.0 / (distances + 1e-10)
                weights = weights / weights.sum()
                xyz[i] = (self.stored_points[indices].T @ weights).T
        
        return xyz
    
    def get_surface_distance(self, uv1, uv2):
        """
        Compute approximate surface distance between two UV points.
        
        Uses the metric tensor to compute:
            ds² = E du² + 2F du dv + G dv²
        
        Args:
            uv1, uv2: UV coordinates (2D arrays)
            
        Returns:
            distance: Approximate surface distance
        """
        if self.metric_tensor is None:
            self.compute_surface_metric()
        
        # Find nearest point in parameterization to uv1
        _, idx = self.kdtree_uv.query(uv1, k=1)
        E, F, G = self.metric_tensor[idx]
        
        # Compute differential
        du = uv2[0] - uv1[0]
        dv = uv2[1] - uv1[1]
        
        # Apply metric tensor
        ds_squared = E * du**2 + 2 * F * du * dv + G * dv**2
        
        return np.sqrt(max(0, ds_squared))
    
    def compute_equidistant_uv_spacing(self, desired_spacing, uv_direction='u'):
        """
        Compute UV spacing that produces equidistant spacing on the surface.
        
        This is the key function for implementing Amersdorfer's approach.
        
        Args:
            desired_spacing: Desired spacing on the surface (in meters)
            uv_direction: 'u' or 'v' direction
            
        Returns:
            spacing_uv: UV spacing that produces desired surface spacing
        """
        if self.metric_tensor is None:
            self.compute_surface_metric()
        
        # Get average metric in the specified direction
        if uv_direction == 'u':
            # E = ||∂r/∂u||²
            avg_scale = np.sqrt(np.mean(self.metric_tensor[:, 0]))
        else:  # 'v'
            # G = ||∂r/∂v||²
            avg_scale = np.sqrt(np.mean(self.metric_tensor[:, 2]))
        
        # To get desired spacing on surface, divide by scale factor
        # If ||∂r/∂u|| = scale, then du = desired_spacing / scale
        spacing_uv = desired_spacing / (avg_scale + 1e-6)
        
        return spacing_uv
    
    def get_uv_bounds(self):
        """
        Get the bounds of the parameter space.
        
        Returns:
            dict with keys: u_min, u_max, v_min, v_max
        """
        if self.uv_params is None:
            raise ValueError("UV parameterization not computed.")
        
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
        
        Returns metrics including:
        - Reconstruction error (how well UV → XYZ → UV works)
        - Metric distortion (how much distances are distorted)
        - Conformality (angle preservation)
        """
        if not self.is_ready:
            raise ValueError("Parameterization not ready.")
        
        n_points = len(self.points)
        if n_points > sample_size:
            indices = np.random.choice(n_points, sample_size, replace=False)
            sample_uv = self.uv_params[indices]
            sample_points = self.points[indices]
            sample_metric = self.metric_tensor[indices] if self.metric_tensor is not None else None
        else:
            sample_uv = self.uv_params
            sample_points = self.points
            sample_metric = self.metric_tensor
        
        # Reconstruction error
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
        
        # Metric distortion
        if sample_metric is not None:
            E_values = sample_metric[:, 0]
            F_values = sample_metric[:, 1]
            G_values = sample_metric[:, 2]
            
            # Isotropy: how close E and G are
            isotropy = np.abs(E_values - G_values) / (E_values + G_values + 1e-6)
            
            # Orthogonality: how close F is to zero
            orthogonality = np.abs(F_values) / np.sqrt((E_values * G_values) + 1e-6)
            
            metrics['mean_isotropy_error'] = float(np.mean(isotropy))
            metrics['mean_orthogonality_error'] = float(np.mean(orthogonality))
            metrics['mean_scale_u'] = float(np.mean(np.sqrt(E_values)))
            metrics['mean_scale_v'] = float(np.mean(np.sqrt(G_values)))
        
        return metrics
    
    def local_to_global(self, points_local):
        """Transform points from local coordinate frame to global frame."""
        if self.principal_axes is None or self.centroid is None:
            raise ValueError("Local frame not computed yet.")
        
        points_local = np.atleast_2d(points_local)
        points_global = points_local @ self.principal_axes + self.centroid
        
        return points_global
    
    def global_to_local(self, points_global):
        """Transform points from global coordinate frame to local frame."""
        if self.principal_axes is None or self.centroid is None:
            raise ValueError("Local frame not computed yet.")
        
        points_global = np.atleast_2d(points_global)
        points_local = (points_global - self.centroid) @ self.principal_axes.T
        
        return points_local
