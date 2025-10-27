"""
Surface Parameterization using Inverse Interpolation Approach
Based on IEEE TASE 2021 paper (dx.doi.org/10.1109/tase.2021.3117691)

This implementation performs surface parameterization of point clouds
using inverse interpolation for robotic applications.
"""

import numpy as np
from scipy.interpolate import griddata, RBFInterpolator
from scipy.spatial import cKDTree
import open3d as o3d
from sklearn.decomposition import PCA
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for VS Code
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class SurfaceParameterization:
    """
    Surface parameterization using inverse interpolation approach.
    
    The method maps 3D Cartesian points (u,v,w) to a 2D parametric domain (x,y) 
    and enables inverse mapping from (x,y) back to 3D Cartesian space (u,v,w).
    
    Notation:
        - (x, y) = 2D parameter space
        - (u, v, w) = 3D Cartesian space
    """
    
    def __init__(self, point_cloud_path=None, points=None):
        """
        Initialize the surface parameterization.
        
        Args:
            point_cloud_path: Path to PLY file
            points: Numpy array of Cartesian points (N x 3) if not loading from file
        """
        if point_cloud_path is not None:
            self.load_point_cloud(point_cloud_path)
        elif points is not None:
            self.points = np.asarray(points)
        else:
            raise ValueError("Either point_cloud_path or points must be provided")
        
        self.xy_params = None  # Parameter space coordinates
        self.interpolator_u = None  # Interpolator for u coordinate
        self.interpolator_v = None  # Interpolator for v coordinate
        self.interpolator_w = None  # Interpolator for w coordinate
        self.principal_axes = None
        
    def load_point_cloud(self, path):
        """Load point cloud from PLY file."""
        pcd = o3d.io.read_point_cloud(path)
        self.points = np.asarray(pcd.points)
        print(f"Loaded {len(self.points)} points from {path}")
        
    def compute_local_frame(self):
        """
        Compute local coordinate frame using PCA.
        This aligns the surface with the principal directions.
        """
        # Center the points
        centroid = np.mean(self.points, axis=0)
        centered_points = self.points - centroid
        
        # Perform PCA to find principal axes
        pca = PCA(n_components=3)
        pca.fit(centered_points)
        
        self.principal_axes = pca.components_
        self.centroid = centroid
        
        # Transform points to local frame
        self.points_local = centered_points @ self.principal_axes.T
        
        print("Local coordinate frame computed")
        return self.principal_axes, centroid
    
    def compute_xy_parameterization(self, method='projection'):
        """
        Compute XY parameterization of the surface.
        Maps 3D Cartesian points (u,v,w) to 2D parameter space (x,y).
        
        Args:
            method: 'projection' - simple projection onto principal plane
                   'distance' - based on cumulative distance
                   'conformal' - angle-preserving mapping
        
        Returns:
            xy_params: N x 2 array of (x,y) parameters in parameter space
        """
        if not hasattr(self, 'points_local'):
            self.compute_local_frame()
        
        if method == 'projection':
            # Project Cartesian coordinates onto the first two principal components
            # This works well for approximately planar or developable surfaces
            xy = self.points_local[:, :2]
            
        elif method == 'distance':
            # Use cumulative distance parameterization
            # Sort points by first principal component
            idx = np.argsort(self.points_local[:, 0])
            sorted_points = self.points_local[idx]
            
            # Compute x parameter (cumulative distance along first direction)
            x = np.zeros(len(sorted_points))
            for i in range(1, len(sorted_points)):
                x[i] = x[i-1] + np.linalg.norm(sorted_points[i] - sorted_points[i-1])
            
            # Normalize x to [0, 1]
            x = x / x[-1] if x[-1] > 0 else x
            
            # Restore original order
            x_orig = np.zeros_like(x)
            x_orig[idx] = x
            
            # y parameter is based on distance from the curve
            y = self.points_local[:, 1]
            
            xy = np.column_stack([x_orig, y])
            
        else:  # conformal or default to projection
            xy = self.points_local[:, :2]
        
        # Normalize XY to [0, 1] range
        xy_min = np.min(xy, axis=0)
        xy_max = np.max(xy, axis=0)
        xy_range = xy_max - xy_min
        xy_range[xy_range == 0] = 1  # Avoid division by zero
        
        self.xy_params = (xy - xy_min) / xy_range
        
        print(f"XY parameterization computed using {method} method")
        print(f"Parameter space range: x=[{self.xy_params[:, 0].min():.3f}, {self.xy_params[:, 0].max():.3f}], "
              f"y=[{self.xy_params[:, 1].min():.3f}, {self.xy_params[:, 1].max():.3f}]")
        
        return self.xy_params
    
    def build_inverse_interpolation(self, method='rbf', neighbors=None):
        """
        Build inverse interpolation from parameter space (x,y) to Cartesian space (u,v,w).
        
        This is the core of the inverse interpolation approach where we
        create interpolators that map from 2D parameter space to 3D Cartesian space.
        
        Args:
            method: 'rbf' - Radial Basis Function interpolation
                   'linear' - Linear interpolation
                   'cubic' - Cubic interpolation
            neighbors: Number of neighbors for local RBF (None for global)
        """
        if self.xy_params is None:
            self.compute_xy_parameterization()
        
        if method == 'rbf':
            # Use RBF interpolation for smooth surface reconstruction
            kernel = 'thin_plate_spline'  # Good for surface fitting
            
            if neighbors is not None:
                # Use only local neighbors for faster computation
                self.interpolation_method = 'rbf_local'
                self.kdtree_xy = cKDTree(self.xy_params)
                self.neighbors = neighbors
                # Store Cartesian points for local interpolation
                self.stored_points = self.points.copy()
            else:
                # Global RBF interpolation
                print("Building RBF interpolators (this may take a while)...")
                self.interpolator_u = RBFInterpolator(
                    self.xy_params, self.points[:, 0], kernel=kernel
                )
                self.interpolator_v = RBFInterpolator(
                    self.xy_params, self.points[:, 1], kernel=kernel
                )
                self.interpolator_w = RBFInterpolator(
                    self.xy_params, self.points[:, 2], kernel=kernel
                )
                self.interpolation_method = 'rbf_global'
                
        else:
            # Use scipy.interpolate.griddata for linear/cubic
            self.interpolation_method = method
        
        print(f"Inverse interpolation built using {method} method")
    
    def interpolate(self, xy_query):
        """
        Interpolate 3D Cartesian coordinates from parameter space coordinates.
        Maps from parameter space (x,y) to Cartesian space (u,v,w).
        
        Args:
            xy_query: N x 2 array of (x,y) query points in parameter space
            
        Returns:
            uvw: N x 3 array of interpolated Cartesian coordinates (u,v,w)
        """
        xy_query = np.atleast_2d(xy_query)
        
        if self.interpolation_method == 'rbf_global':
            u = self.interpolator_u(xy_query)
            v = self.interpolator_v(xy_query)
            w = self.interpolator_w(xy_query)
            uvw = np.column_stack([u, v, w])
            
        elif self.interpolation_method == 'rbf_local':
            # Use local RBF interpolation with batching for efficiency
            uvw = np.zeros((len(xy_query), 3))
            
            # Process in batches or use vectorized nearest neighbor lookup
            if len(xy_query) > 100:
                # For large queries, use faster weighted average instead of full RBF
                for i, xy in enumerate(xy_query):
                    distances, indices = self.kdtree_xy.query(xy, k=min(self.neighbors, 10))
                    
                    # Inverse distance weighting (faster than RBF)
                    if distances[0] < 1e-10:
                        # Exact match
                        uvw[i] = self.stored_points[indices[0]]
                    else:
                        weights = 1.0 / (distances + 1e-10)
                        weights = weights / weights.sum()
                        uvw[i] = (self.stored_points[indices].T @ weights).T
            else:
                # For small queries, use full local RBF
                for i, xy in enumerate(xy_query):
                    distances, indices = self.kdtree_xy.query(xy, k=self.neighbors)
                    
                    local_xy = self.xy_params[indices]
                    local_uvw = self.stored_points[indices]
                    
                    try:
                        interp = RBFInterpolator(
                            local_xy, local_uvw, kernel='thin_plate_spline'
                        )
                        uvw[i] = interp(xy.reshape(1, -1))[0]
                    except:
                        uvw[i] = local_uvw[0]
                    
        else:
            # Use griddata for linear/cubic interpolation
            uvw = griddata(
                self.xy_params, self.points, xy_query, 
                method=self.interpolation_method
            )
        
        return uvw
    
    def create_regular_grid(self, x_samples=50, y_samples=50):
        """
        Create a regular grid in parameter space (x,y) and interpolate to Cartesian space (u,v,w).
        
        Args:
            x_samples: Number of samples in x direction (parameter space)
            y_samples: Number of samples in y direction (parameter space)
            
        Returns:
            grid_uvw: (y_samples x x_samples x 3) array of Cartesian points (u,v,w)
            grid_xy: (y_samples x x_samples x 2) array of parameter space coordinates (x,y)
        """
        x = np.linspace(0, 1, x_samples)
        y = np.linspace(0, 1, y_samples)
        x_grid, y_grid = np.meshgrid(x, y)
        
        grid_xy = np.column_stack([x_grid.ravel(), y_grid.ravel()])
        grid_uvw = self.interpolate(grid_xy)
        
        grid_uvw = grid_uvw.reshape(y_samples, x_samples, 3)
        grid_xy_reshaped = grid_xy.reshape(y_samples, x_samples, 2)
        
        return grid_uvw, grid_xy_reshaped
    
    def compute_surface_normals(self, xy_query):
        """
        Compute surface normals at given parameter space coordinates.
        
        Args:
            xy_query: N x 2 array of (x,y) query points in parameter space
            
        Returns:
            normals: N x 3 array of unit normal vectors in Cartesian space
        """
        xy_query = np.atleast_2d(xy_query)
        normals = np.zeros((len(xy_query), 3))
        
        epsilon = 1e-5
        
        for i, xy in enumerate(xy_query):
            # Compute partial derivatives using finite differences
            x, y = xy
            
            # Partial derivative with respect to x (∂/∂x of position in Cartesian space)
            xy_dx = np.array([[x + epsilon, y]])
            p_dx = self.interpolate(xy_dx)[0] - self.interpolate(xy.reshape(1, -1))[0]
            p_dx = p_dx / epsilon
            
            # Partial derivative with respect to y (∂/∂y of position in Cartesian space)
            xy_dy = np.array([[x, y + epsilon]])
            p_dy = self.interpolate(xy_dy)[0] - self.interpolate(xy.reshape(1, -1))[0]
            p_dy = p_dy / epsilon
            
            # Normal is cross product of partial derivatives
            normal = np.cross(p_dx, p_dy)
            norm = np.linalg.norm(normal)
            if norm > 0:
                normals[i] = normal / norm
            else:
                normals[i] = np.array([0, 0, 1])  # Default normal
        
        return normals
    
    def visualize(self, show_grid=True, show_original=True, grid_samples=30, save_path='surface_visualization.png'):
        """
        Visualize the parameterized surface and save to file.
        
        Args:
            show_grid: Whether to show the interpolated grid
            show_original: Whether to show original Cartesian points
            grid_samples: Number of samples for the grid
            save_path: Path to save the visualization (default: 'surface_visualization.png')
        """
        fig = plt.figure(figsize=(15, 5))
        
        # 3D view of original Cartesian points (u,v,w)
        ax1 = fig.add_subplot(131, projection='3d')
        if show_original:
            ax1.scatter(self.points[:, 0], self.points[:, 1], self.points[:, 2], 
                       c='blue', s=1, alpha=0.5, label='Original points')
        
        if show_grid:
            grid_uvw, _ = self.create_regular_grid(grid_samples, grid_samples)
            ax1.plot_surface(grid_uvw[:, :, 0], grid_uvw[:, :, 1], grid_uvw[:, :, 2],
                           alpha=0.7, cmap='viridis', edgecolor='none')
        
        ax1.set_xlabel('U')
        ax1.set_ylabel('V')
        ax1.set_zlabel('W')
        ax1.set_title('3D Cartesian Surface (u,v,w)')
        ax1.legend()
        
        # XY parameter space
        ax2 = fig.add_subplot(132)
        scatter = ax2.scatter(self.xy_params[:, 0], self.xy_params[:, 1], 
                            c=self.points[:, 2], s=5, cmap='viridis')
        ax2.set_xlabel('x (parameter)')
        ax2.set_ylabel('y (parameter)')
        ax2.set_title('Parameter Space (x,y) - colored by W')
        plt.colorbar(scatter, ax=ax2, label='W coordinate')
        ax2.set_aspect('equal')
        
        # Height map
        ax3 = fig.add_subplot(133, projection='3d')
        ax3.scatter(self.xy_params[:, 0], self.xy_params[:, 1], self.points[:, 2],
                   c=self.points[:, 2], s=5, cmap='viridis')
        ax3.set_xlabel('x (parameter)')
        ax3.set_ylabel('y (parameter)')
        ax3.set_zlabel('W')
        ax3.set_title('Height Map (x,y,W)')
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Visualization saved to: {save_path}")
        plt.close(fig)  # Close to free memory
    
    def evaluate_quality(self, sample_size=1000):
        """
        Evaluate the quality of the parameterization.
        
        Args:
            sample_size: Number of points to sample for evaluation (for large clouds)
        
        Returns:
            metrics: Dictionary with quality metrics
        """
        # For large point clouds, sample a subset for evaluation
        n_points = len(self.points)
        if n_points > sample_size:
            print(f"  Sampling {sample_size} points from {n_points} for quality evaluation...")
            indices = np.random.choice(n_points, sample_size, replace=False)
            sample_xy = self.xy_params[indices]
            sample_points = self.points[indices]
        else:
            sample_xy = self.xy_params
            sample_points = self.points
        
        # Reconstruct sampled points from parameter space
        reconstructed = self.interpolate(sample_xy)
        
        # Compute reconstruction error
        errors = np.linalg.norm(sample_points - reconstructed, axis=1)
        
        metrics = {
            'mean_error': np.mean(errors),
            'max_error': np.max(errors),
            'std_error': np.std(errors),
            'rmse': np.sqrt(np.mean(errors**2)),
            'sample_size': len(sample_points)
        }
        
        print("\nParameterization Quality Metrics:")
        print(f"  Sample size: {metrics['sample_size']}/{n_points} points")
        print(f"  Mean reconstruction error: {metrics['mean_error']:.6f}")
        print(f"  Max reconstruction error: {metrics['max_error']:.6f}")
        print(f"  RMSE: {metrics['rmse']:.6f}")
        print(f"  Std deviation: {metrics['std_error']:.6f}")
        
        return metrics


def main():
    """
    Example usage of the surface parameterization.
    Notation: (x,y) = parameter space, (u,v,w) = Cartesian space
    """
    # Load point cloud
    point_cloud_path = "point_cloud.ply"
    
    print("=" * 60)
    print("Surface Parameterization - Inverse Interpolation Approach")
    print("Notation: (x,y) = parameter space | (u,v,w) = Cartesian space")
    print("=" * 60)
    
    # Initialize parameterization
    surf_param = SurfaceParameterization(point_cloud_path=point_cloud_path)
    
    # Compute local coordinate frame
    surf_param.compute_local_frame()
    
    # Compute XY parameterization
    surf_param.compute_xy_parameterization(method='projection')
    
    # Build inverse interpolation
    surf_param.build_inverse_interpolation(method='rbf', neighbors=50)
    
    # Evaluate quality
    metrics = surf_param.evaluate_quality()
    
    # Example: Sample some points from the surface
    print("\n" + "=" * 60)
    print("Testing inverse interpolation:")
    print("=" * 60)
    
    test_xy = np.array([
        [0.5, 0.5],   # Center
        [0.0, 0.0],   # Corner
        [1.0, 1.0],   # Opposite corner
        [0.25, 0.75]  # Random point
    ])
    
    interpolated_points = surf_param.interpolate(test_xy)
    normals = surf_param.compute_surface_normals(test_xy)
    
    for i, (xy, uvw, normal) in enumerate(zip(test_xy, interpolated_points, normals)):
        print(f"\nPoint {i+1}:")
        print(f"  Parameter (x,y): ({xy[0]:.3f}, {xy[1]:.3f})")
        print(f"  Cartesian (u,v,w): ({uvw[0]:.3f}, {uvw[1]:.3f}, {uvw[2]:.3f})")
        print(f"  Normal: ({normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f})")
    
    # Visualize
    print("\n" + "=" * 60)
    print("Generating visualization...")
    print("=" * 60)
    surf_param.visualize(show_grid=True, show_original=True, grid_samples=30, 
                        save_path='surface_visualization.png')
    
    # Export regular grid for robotic path planning
    print("\nCreating regular grid for path planning...")
    grid_uvw, grid_xy = surf_param.create_regular_grid(x_samples=50, y_samples=50)
    
    # Save grid to file
    np.save('surface_grid_uvw.npy', grid_uvw)
    np.save('surface_grid_xy.npy', grid_xy)
    print(f"Grid saved: Cartesian shape {grid_uvw.shape}, Parameter shape {grid_xy.shape}")
    
    return surf_param


if __name__ == "__main__":
    surf_param = main()
