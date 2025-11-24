"""
This module implements arc-length-based isometric surface parameterization 
for robotic machining on curved surfaces.

Uses distance-preserving parameterization:
- Arc-length based: u = ∫√(1+(∂z/∂x)²)dx, v = ∫√(1+(∂z/∂y)²)dy
- Results in metric tensor E ≈ 1, G ≈ 1, F ≈ 0
- Enables true equidistant path planning on curved surfaces
"""

import numpy as np
from scipy.interpolate import CloughTocher2DInterpolator
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from scipy.sparse.linalg import spsolve
from sklearn.decomposition import PCA


class Parameterization:
    """
    Arc-length-based isometric surface parameterization for robotic machining.
    
    Implements:
    - Distance-preserving (isometric) parameterization
    - UV spacing ≈ surface distance
    - Arc-length based: u = ∫√(1+(∂z/∂x)²)dx, v = ∫√(1+(∂z/∂y)²)dy
    - Results in metric tensor E ≈ 1, G ≈ 1, F ≈ 0
    - Enables simple equidistant path planning on curved surfaces
    
    Key Features:
    - Bivariate cubic spline interpolation (UV → XYZ mapping)
    - First fundamental form (metric tensor) computation
    - Equidistant spacing calculation for path planning
    - Iso-parametric curves for equidistant tool paths
    """
    
    def __init__(self):
        """Initialize the arc-length-based parameterization."""
        self.points = None
        self.uv_params = None
        self.metric_tensor = None  # First fundamental form
        self.interpolator_x = None
        self.interpolator_y = None
        self.interpolator_z = None
        self.interpolator_E = None
        self.interpolator_F = None
        self.interpolator_G = None
        self.principal_axes = None
        self.centroid = None
        self.points_local = None
        self.kdtree_uv = None
        self.kdtree_xyz = None
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
    
    def compute_initial_parameterization(self):
        """
        Compute arc-length-based UV parameterization.
        
        Uses isometric (distance-preserving) parameterization
        """
        if self.points_local is None:
            self.compute_local_frame()
        
        # Arc-length based parameterization (isometric)
        self.uv_params = self._compute_arc_length_uv()
        
        return self.uv_params
    
    def _compute_arc_length_uv(self):
        """
        Compute arc-length-based UV parameterization for point clouds.
        
        Optimized approach using grid-based interpolation:
        - Sort points and compute cumulative arc-lengths along x and y directions
        - Use spatial binning to compute arc-lengths in 2D grid
        - Interpolate arc-length values for each point
        
        This reduces complexity from O(N²) to O(N log N + G²) where G is grid size.
        
        Returns:
            uv_params: Nx2 array of (u,v) coordinates based on arc-lengths
        """
        print("Computing optimized arc-length-based parameterization...")
        
        n_points = len(self.points_local)
        
        # Use local coordinates (x, y, z) from PCA
        x = self.points_local[:, 0]
        y = self.points_local[:, 1]
        z = self.points_local[:, 2]
        
        # Find reference point
        x_min, y_min = np.min(x), np.min(y)
        x_max, y_max = np.max(x), np.max(y)
        
        # Create adaptive grid based on point density
        n_grid = min(100, max(20, int(np.sqrt(n_points))))
        
        # Build 2D grid for arc-length computation
        x_grid = np.linspace(x_min, x_max, n_grid)
        y_grid = np.linspace(y_min, y_max, n_grid)
        
        # Initialize arc-length grids
        u_grid = np.zeros((n_grid, n_grid))
        v_grid = np.zeros((n_grid, n_grid))
        
        # Bin points into grid cells for local arc-length computation
        x_indices = np.digitize(x, x_grid) - 1
        y_indices = np.digitize(y, y_grid) - 1
        
        # Clamp indices to valid range
        x_indices = np.clip(x_indices, 0, n_grid - 1)
        y_indices = np.clip(y_indices, 0, n_grid - 1)
        
        # Compute arc-length in u-direction (along x-axis for each y-slice)
        for j in range(n_grid):
            # Find points in this y-slice
            y_slice_mask = (y_indices == j)
            if not np.any(y_slice_mask):
                continue
            
            # Get points in this slice and sort by x
            slice_x = x[y_slice_mask]
            slice_z = z[y_slice_mask]
            sort_idx = np.argsort(slice_x)
            slice_x_sorted = slice_x[sort_idx]
            slice_z_sorted = slice_z[sort_idx]
            
            # Compute cumulative arc-length along this slice
            if len(slice_x_sorted) > 1:
                dx = np.diff(slice_x_sorted)
                dz = np.diff(slice_z_sorted)
                arc_increments = np.sqrt(dx**2 + dz**2)
                cumulative_arc = np.concatenate([[0], np.cumsum(arc_increments)])
                
                # Interpolate arc-length values onto regular grid
                u_grid[:, j] = np.interp(x_grid, slice_x_sorted, cumulative_arc)
        
        # Compute arc-length in v-direction (along y-axis for each x-slice)
        for i in range(n_grid):
            # Find points in this x-slice
            x_slice_mask = (x_indices == i)
            if not np.any(x_slice_mask):
                continue
            
            # Get points in this slice and sort by y
            slice_y = y[x_slice_mask]
            slice_z = z[x_slice_mask]
            sort_idx = np.argsort(slice_y)
            slice_y_sorted = slice_y[sort_idx]
            slice_z_sorted = slice_z[sort_idx]
            
            # Compute cumulative arc-length along this slice
            if len(slice_y_sorted) > 1:
                dy = np.diff(slice_y_sorted)
                dz = np.diff(slice_z_sorted)
                arc_increments = np.sqrt(dy**2 + dz**2)
                cumulative_arc = np.concatenate([[0], np.cumsum(arc_increments)])
                
                # Interpolate arc-length values onto regular grid
                v_grid[i, :] = np.interp(y_grid, slice_y_sorted, cumulative_arc)
        
        # Interpolate arc-length values for all points using bilinear interpolation
        uv_params = np.zeros((n_points, 2))
        
        # Normalize coordinates to grid indices
        x_norm = (x - x_min) / (x_max - x_min + 1e-10) * (n_grid - 1)
        y_norm = (y - y_min) / (y_max - y_min + 1e-10) * (n_grid - 1)
        
        # Bilinear interpolation for u values
        x_floor = np.floor(x_norm).astype(int)
        y_floor = np.floor(y_norm).astype(int)
        x_ceil = np.minimum(x_floor + 1, n_grid - 1)
        y_ceil = np.minimum(y_floor + 1, n_grid - 1)
        
        # Interpolation weights
        wx = x_norm - x_floor
        wy = y_norm - y_floor
        
        # Bilinear interpolation for u
        u_00 = u_grid[x_floor, y_floor]
        u_10 = u_grid[x_ceil, y_floor]
        u_01 = u_grid[x_floor, y_ceil]
        u_11 = u_grid[x_ceil, y_ceil]
        
        uv_params[:, 0] = (1 - wx) * (1 - wy) * u_00 + \
                          wx * (1 - wy) * u_10 + \
                          (1 - wx) * wy * u_01 + \
                          wx * wy * u_11
        
        # Bilinear interpolation for v
        v_00 = v_grid[x_floor, y_floor]
        v_10 = v_grid[x_ceil, y_floor]
        v_01 = v_grid[x_floor, y_ceil]
        v_11 = v_grid[x_ceil, y_ceil]
        
        uv_params[:, 1] = (1 - wx) * (1 - wy) * v_00 + \
                          wx * (1 - wy) * v_10 + \
                          (1 - wx) * wy * v_01 + \
                          wx * wy * v_11
        
        print(f"Arc-length parameterization complete (optimized). UV range: u=[{uv_params[:,0].min():.3f}, {uv_params[:,0].max():.3f}], v=[{uv_params[:,1].min():.3f}, {uv_params[:,1].max():.3f}]")
        
        return uv_params
    
    def compute_surface_metric(self, k_neighbors=20):
        """
        Compute the first fundamental form (metric tensor) of the surface.
        
        The metric tensor G = [E F; F G] describes how distances in UV space
        relate to distances on the actual surface:
            ds² = E du² + 2F du dv + G dv²
        
        For isometric (arc-length) parameterization: E ≈ 1, G ≈ 1, F ≈ 0
        For projection parameterization: E and G depend on surface curvature
        
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
       
    def build_inverse_interpolation(self):
        """
        Build cubic spline inverse interpolation from (u,v) → (x,y,z).
        
        Uses CloughTocher2D interpolation which provides:
        - Piecewise cubic spline interpolation
        - C1 continuity (continuous first derivatives)
        - Good balance of smoothness and computational efficiency
        """
        if self.uv_params is None:
            self.compute_initial_parameterization()
        
        # CloughTocher2D provides piecewise cubic spline interpolation
        # C1 continuous (continuous first derivatives)
        print("Building cubic spline interpolators (CloughTocher2D)...")
        self.interpolator_x = CloughTocher2DInterpolator(self.uv_params, self.points[:, 0])
        self.interpolator_y = CloughTocher2DInterpolator(self.uv_params, self.points[:, 1])
        self.interpolator_z = CloughTocher2DInterpolator(self.uv_params, self.points[:, 2])
        
        # Build KD-tree for utility functions
        if self.kdtree_uv is None:
            self.kdtree_uv = cKDTree(self.uv_params)
        
        self.is_ready = True
        print("Cubic spline interpolation ready.")
    
    def interpolate(self, uv_query):
        """
        Interpolate Cartesian coordinates from parameter space using cubic splines.
        Maps (u,v) → (x,y,z).
        
        Args:
            uv_query: Nx2 array of (u,v) coordinates
            
        Returns:
            xyz: Nx3 array of (x,y,z) coordinates
        """
        if not self.is_ready:
            raise ValueError("Interpolation not ready. Call build_inverse_interpolation() first.")
        
        uv_query = np.atleast_2d(uv_query)
        
        # Use cubic spline interpolators
        x = self.interpolator_x(uv_query)
        y = self.interpolator_y(uv_query)
        z = self.interpolator_z(uv_query)
        xyz = np.column_stack([x, y, z])
        
        return xyz
    
    def get_metric_at_uv(self, uv_query):
        """
        Get metric tensor components at arbitrary UV coordinates.
        
        Args:
            uv_query: Nx2 array of (u,v) coordinates
            
        Returns:
            metric: Nx3 array where each row is [E, F, G] at that UV point
        """
        if not self.is_ready:
            raise ValueError("Interpolation not ready. Call build_inverse_interpolation() first.")
        
        uv_query = np.atleast_2d(uv_query)
        
        # Use interpolators if available, otherwise use nearest neighbor
        if self.interpolator_E is not None:
            E = self.interpolator_E(uv_query)
            F = self.interpolator_F(uv_query)
            G = self.interpolator_G(uv_query)
            metric = np.column_stack([E, F, G])
        else:
            # Fallback to nearest neighbor
            metric = np.zeros((len(uv_query), 3))
            for i, uv in enumerate(uv_query):
                _, idx = self.kdtree_uv.query(uv, k=1)
                metric[i] = self.metric_tensor[idx]
        
        return metric
    
    def get_surface_distance(self, uv1, uv2):
        """
        Compute approximate surface distance between two UV points.
        
        Uses the metric tensor to compute:
            ds² = E du² + 2F du dv + G dv²
        
        Args:
            uv1, uv2: UV coordinates (2D arrays or lists)
            
        Returns:
            distance: Approximate surface distance
        """
        if self.metric_tensor is None:
            self.compute_surface_metric()
        
        # Get metric at uv1 (uses interpolation if available)
        uv1 = np.atleast_2d(uv1)
        metric = self.get_metric_at_uv(uv1)
        E, F, G = metric[0]
        
        # Compute differential
        du = uv2[0] - uv1[0, 0]
        dv = uv2[1] - uv1[0, 1]
        
        # Apply metric tensor
        ds_squared = E * du**2 + 2 * F * du * dv + G * dv**2
        
        return np.sqrt(max(0, ds_squared))
    
    def compute_equidistant_uv_spacing(self, desired_spacing, uv_direction='u'):
        """
        Compute UV spacing that produces equidistant spacing on the surface.
        
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
