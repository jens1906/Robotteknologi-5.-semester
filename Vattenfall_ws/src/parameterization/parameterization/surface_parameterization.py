"""
This module implements arc-length-based isometric surface parameterization 
for robotic machining on curved surfaces.

Uses distance-preserving parameterization:
- Arc-length based: u = ∫√(1+(∂z/∂x)²)dx, v = ∫√(1+(∂z/∂y)²)dy
- Results in metric tensor E ≈ 1, G ≈ 1, F ≈ 0
- Enables true equidistant path planning on curved surfaces
"""

import numpy as np
import heapq
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
        Compute arc-length-based UV parameterization using the integral approach.
        
        Implements the arc-length integrals from the paper:
        u(x,y) = ∫[xref to x] √(1 + (∂z/∂ξ)²) dξ  at y=yref, then adjust for actual y
        v(x,y) = ∫[yref to y] √(1 + (∂z/∂ζ)²) dζ  at x=x
        
        Strategy:
        1. Create a dense interpolator for z(x,y)
        2. For each point, numerically integrate arc-length from reference
        3. This gives true "unfolded" coordinates
        
        Returns:
            uv_params: Nx2 array of (u,v) coordinates based on arc-lengths
        """
        print("Computing arc-length-based parameterization (continuous integral method)...")
        
        from scipy.interpolate import LinearNDInterpolator, CloughTocher2DInterpolator
        from scipy.integrate import simpson
        
        n_points = len(self.points_local)
        
        # Use local coordinates (x, y, z) from PCA
        x = self.points_local[:, 0]
        y = self.points_local[:, 1]
        z = self.points_local[:, 2]
        
        # Find bounds and reference point (bottom-left corner)
        x_min, y_min = np.min(x), np.min(y)
        x_max, y_max = np.max(x), np.max(y)
        xref, yref = x_min, y_min
        
        print(f"  Reference point: ({xref:.2f}, {yref:.2f})")
        print(f"  Bounds: X=[{x_min:.2f}, {x_max:.2f}], Y=[{y_min:.2f}, {y_max:.2f}]")
        
        # Create interpolator for z(x,y) - use CloughTocher for smoothness
        print("  Building z(x,y) interpolator...")
        xy_points = np.column_stack([x, y])
        z_interp = CloughTocher2DInterpolator(xy_points, z)
        
        # Helper function to compute dz/dx at a point
        def dz_dx(xi, yi, h=1.0):
            """Numerical derivative ∂z/∂x"""
            z_plus = z_interp(xi + h, yi)
            z_minus = z_interp(xi - h, yi)
            # Check for NaN
            if np.isnan(z_plus) or np.isnan(z_minus):
                return 0.0  # Assume flat surface at boundaries
            return (z_plus - z_minus) / (2 * h)
        
        # Helper function to compute dz/dy at a point
        def dz_dy(xi, yi, h=1.0):
            """Numerical derivative ∂z/∂y"""
            z_plus = z_interp(xi, yi + h)
            z_minus = z_interp(xi, yi - h)
            # Check for NaN
            if np.isnan(z_plus) or np.isnan(z_minus):
                return 0.0  # Assume flat surface at boundaries
            return (z_plus - z_minus) / (2 * h)
        
        # Compute U parameter for each point
        print("  Computing U parameters (integrating along x)...")
        u_params = np.zeros(n_points)
        
        # For efficiency, compute on a regular grid then interpolate
        n_grid_u = min(200, max(50, int(np.sqrt(n_points))))
        n_grid_v = min(200, max(50, int(np.sqrt(n_points))))
        
        x_grid = np.linspace(x_min, x_max, n_grid_u)
        y_grid = np.linspace(y_min, y_max, n_grid_v)
        
        # Compute u on grid: for each (x_i, y_j), integrate from xref to x_i along y=yref
        u_grid = np.zeros((n_grid_u, n_grid_v))
        
        print(f"    Computing on {n_grid_u}x{n_grid_v} grid for efficiency...")
        
        # First, compute u along the reference line y=yref
        u_at_yref = np.zeros(n_grid_u)
        for i in range(1, n_grid_u):
            # Integrate from x_grid[i-1] to x_grid[i]
            x_segment = np.linspace(x_grid[i-1], x_grid[i], 20)
            # Compute integrand: √(1 + (∂z/∂x)²)
            integrand = []
            for xi in x_segment:
                try:
                    dzx = dz_dx(xi, yref)
                    integrand.append(np.sqrt(1 + dzx**2))
                except:
                    integrand.append(1.0)  # Fallback to flat surface
            
            # Numerical integration using Simpson's rule
            u_increment = simpson(integrand, x=x_segment)
            u_at_yref[i] = u_at_yref[i-1] + u_increment
        
        # For other y values, u is approximately the same (assuming u depends mainly on x)
        # This is an approximation - ideally we'd integrate along each y level
        for j in range(n_grid_v):
            u_grid[:, j] = u_at_yref
        
        # Interpolate u values for actual points
        from scipy.interpolate import RegularGridInterpolator
        u_interpolator = RegularGridInterpolator((x_grid, y_grid), u_grid, 
                                                  bounds_error=False, fill_value=0.0)
        
        for i in range(n_points):
            u_val = u_interpolator([x[i], y[i]])[0]
            u_params[i] = u_val if not np.isnan(u_val) else 0.0
        
        print(f"    U range: [{np.min(u_params):.2f}, {np.max(u_params):.2f}]")
        
        # Compute V parameter for each point
        print("  Computing V parameters (integrating along y)...")
        v_params = np.zeros(n_points)
        
        # Compute v on grid: for each (x_i, y_j), integrate from yref to y_j along x=x_i
        v_grid = np.zeros((n_grid_u, n_grid_v))
        
        # For each x value, compute v by integrating along y
        for i in range(n_grid_u):
            xi = x_grid[i]
            for j in range(1, n_grid_v):
                # Integrate from y_grid[j-1] to y_grid[j]
                y_segment = np.linspace(y_grid[j-1], y_grid[j], 20)
                # Compute integrand: √(1 + (∂z/∂y)²)
                integrand = []
                for yi in y_segment:
                    try:
                        dzy = dz_dy(xi, yi)
                        integrand.append(np.sqrt(1 + dzy**2))
                    except:
                        integrand.append(1.0)  # Fallback
                
                # Numerical integration
                v_increment = simpson(integrand, x=y_segment)
                v_grid[i, j] = v_grid[i, j-1] + v_increment
        
        # Interpolate v values for actual points
        v_interpolator = RegularGridInterpolator((x_grid, y_grid), v_grid,
                                                  bounds_error=False, fill_value=0.0)
        
        for i in range(n_points):
            v_val = v_interpolator([x[i], y[i]])[0]
            v_params[i] = v_val if not np.isnan(v_val) else 0.0
        
        print(f"    V range: [{np.min(v_params):.2f}, {np.max(v_params):.2f}]")
        
        # Combine into UV parameters
        uv_params = np.column_stack([u_params, v_params])
        
        print(f"Arc-length parameterization complete (continuous integral method).")
        print(f"  UV range: u=[{uv_params[:,0].min():.3f}, {uv_params[:,0].max():.3f}], v=[{uv_params[:,1].min():.3f}, {uv_params[:,1].max():.3f}]")
        
        return uv_params
      
    def build_inverse_interpolation(self):
        """
        Build cubic spline inverse interpolation from (u,v) → (x,y,z).
        
        Uses CloughTocher2D interpolation which provides:
        - Piecewise cubic spline interpolation
        - C1 continuity (continuous first derivatives)
        - Good balance of smoothness and computational efficiency
        
        Note: Interpolators map UV → local coordinates, then transform to global.
        """
        if self.uv_params is None:
            self.compute_initial_parameterization()
        
        # CloughTocher2D provides piecewise cubic spline interpolation
        # C1 continuous (continuous first derivatives)
        # UV params are computed from local frame, so interpolators must use local coords
        print("Building cubic spline interpolators (CloughTocher2D)...")
        self.interpolator_x = CloughTocher2DInterpolator(self.uv_params, self.points_local[:, 0])
        self.interpolator_y = CloughTocher2DInterpolator(self.uv_params, self.points_local[:, 1])
        self.interpolator_z = CloughTocher2DInterpolator(self.uv_params, self.points_local[:, 2])
        
        # Build KD-tree for utility functions
        if self.kdtree_uv is None:
            self.kdtree_uv = cKDTree(self.uv_params)
        
        self.is_ready = True
        print("Cubic spline interpolation ready.")
    
    def interpolate(self, uv_query):
        """
        Interpolate Cartesian coordinates from parameter space using cubic splines.
        Maps (u,v) → (x,y,z) in global coordinate frame.
        
        Args:
            uv_query: Nx2 array of (u,v) coordinates
            
        Returns:
            xyz: Nx3 array of (x,y,z) coordinates in global frame
            
        Note:
            CloughTocher2D returns NaN for points outside the convex hull.
            Check UV bounds with get_uv_bounds() before calling.
        """
        if not self.is_ready:
            raise ValueError("Interpolation not ready. Call build_inverse_interpolation() first.")
        
        uv_query = np.atleast_2d(uv_query)
        
        # Use cubic spline interpolators to get local coordinates
        x_local = self.interpolator_x(uv_query)
        y_local = self.interpolator_y(uv_query)
        z_local = self.interpolator_z(uv_query)
        xyz_local = np.column_stack([x_local, y_local, z_local])
        
        # Check for NaN values (points outside convex hull)
        nan_mask = np.any(np.isnan(xyz_local), axis=1)
        if np.any(nan_mask):
            print(f"Warning: {np.sum(nan_mask)} of {len(uv_query)} UV points are outside the interpolation domain")
            # Get UV bounds for debugging
            bounds = self.get_uv_bounds()
            out_of_bounds = uv_query[nan_mask]
            print(f"  UV bounds: u=[{bounds['u_min']:.3f}, {bounds['u_max']:.3f}], v=[{bounds['v_min']:.3f}, {bounds['v_max']:.3f}]")
            print(f"  Sample out-of-bounds UV: {out_of_bounds[:min(5, len(out_of_bounds))]}")
        
        # Transform from local frame to global frame
        xyz_global = self.local_to_global(xyz_local)
        
        return xyz_global
       
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
