"""
Surface Parameterization - Standalone Script
Run this file directly in VS Code (F5 or right-click > Run Python File)

This implementation performs surface parameterization of point clouds
using inverse interpolation for robotic applications.

Notation:
    (x, y) = 2D parameter space
    (u, v, w) = 3D Cartesian space
"""

import numpy as np
from scipy.interpolate import griddata, RBFInterpolator
from scipy.spatial import cKDTree
import open3d as o3d
from sklearn.decomposition import PCA
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for VS Code
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os


class SurfaceParameterization:
    """
    Surface parameterization using inverse interpolation approach.
    Maps 3D Cartesian points (u,v,w) to 2D parameter space (x,y).
    """
    
    def __init__(self, point_cloud_path=None, points=None):
        """Initialize the surface parameterization."""
        if point_cloud_path is not None:
            self.load_point_cloud(point_cloud_path)
        elif points is not None:
            self.points = np.asarray(points)
        else:
            raise ValueError("Either point_cloud_path or points must be provided")
        
        self.xy_params = None
        self.interpolator_u = None
        self.interpolator_v = None
        self.interpolator_w = None
        self.principal_axes = None
        
    def load_point_cloud(self, path):
        """Load point cloud from PLY file."""
        pcd = o3d.io.read_point_cloud(path)
        self.points = np.asarray(pcd.points)
        print(f"Loaded {len(self.points)} points from {path}")
        
    def compute_local_frame(self):
        """Compute local coordinate frame using PCA."""
        centroid = np.mean(self.points, axis=0)
        centered_points = self.points - centroid
        
        pca = PCA(n_components=3)
        pca.fit(centered_points)
        
        self.principal_axes = pca.components_
        self.centroid = centroid
        self.points_local = centered_points @ self.principal_axes.T
        
        print("Local coordinate frame computed")
        return self.principal_axes, centroid
    
    def compute_xy_parameterization(self, method='projection'):
        """
        Compute XY parameterization mapping (u,v,w) → (x,y).
        """
        if not hasattr(self, 'points_local'):
            self.compute_local_frame()
        
        if method == 'projection':
            xy = self.points_local[:, :2]
        elif method == 'distance':
            idx = np.argsort(self.points_local[:, 0])
            sorted_points = self.points_local[idx]
            
            x = np.zeros(len(sorted_points))
            for i in range(1, len(sorted_points)):
                x[i] = x[i-1] + np.linalg.norm(sorted_points[i] - sorted_points[i-1])
            
            x = x / x[-1] if x[-1] > 0 else x
            x_orig = np.zeros_like(x)
            x_orig[idx] = x
            y = self.points_local[:, 1]
            xy = np.column_stack([x_orig, y])
        else:
            xy = self.points_local[:, :2]
        
        xy_min = np.min(xy, axis=0)
        xy_max = np.max(xy, axis=0)
        xy_range = xy_max - xy_min
        xy_range[xy_range == 0] = 1
        
        self.xy_params = (xy - xy_min) / xy_range
        
        print(f"XY parameterization computed using {method} method")
        return self.xy_params
    
    def build_inverse_interpolation(self, method='rbf', neighbors=None):
        """
        Build inverse interpolation from (x,y) → (u,v,w).
        """
        if self.xy_params is None:
            self.compute_xy_parameterization()
        
        if method == 'rbf':
            kernel = 'thin_plate_spline'
            
            if neighbors is not None:
                self.interpolation_method = 'rbf_local'
                self.kdtree_xy = cKDTree(self.xy_params)
                self.neighbors = neighbors
                self.stored_points = self.points.copy()
            else:
                print("  Building RBF interpolators (may take a moment)...")
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
            self.interpolation_method = method
        
        print(f" Inverse interpolation built using {method} method")
    
    def interpolate(self, xy_query):
        """
        Interpolate Cartesian coordinates from parameter space.
        Maps (x,y) → (u,v,w).
        """
        xy_query = np.atleast_2d(xy_query)
        
        if self.interpolation_method == 'rbf_global':
            u = self.interpolator_u(xy_query)
            v = self.interpolator_v(xy_query)
            w = self.interpolator_w(xy_query)
            uvw = np.column_stack([u, v, w])
            
        elif self.interpolation_method == 'rbf_local':
            uvw = np.zeros((len(xy_query), 3))
            
            if len(xy_query) > 100:
                for i, xy in enumerate(xy_query):
                    distances, indices = self.kdtree_xy.query(xy, k=min(self.neighbors, 10))
                    
                    if distances[0] < 1e-10:
                        uvw[i] = self.stored_points[indices[0]]
                    else:
                        weights = 1.0 / (distances + 1e-10)
                        weights = weights / weights.sum()
                        uvw[i] = (self.stored_points[indices].T @ weights).T
            else:
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
            uvw = griddata(
                self.xy_params, self.points, xy_query, 
                method=self.interpolation_method
            )
        
        return uvw
    
    def compute_surface_normals(self, xy_query):
        """Compute surface normals at parameter space coordinates."""
        xy_query = np.atleast_2d(xy_query)
        normals = np.zeros((len(xy_query), 3))
        epsilon = 1e-5
        
        for i, xy in enumerate(xy_query):
            x, y = xy
            
            xy_dx = np.array([[x + epsilon, y]])
            p_dx = self.interpolate(xy_dx)[0] - self.interpolate(xy.reshape(1, -1))[0]
            p_dx = p_dx / epsilon
            
            xy_dy = np.array([[x, y + epsilon]])
            p_dy = self.interpolate(xy_dy)[0] - self.interpolate(xy.reshape(1, -1))[0]
            p_dy = p_dy / epsilon
            
            normal = np.cross(p_dx, p_dy)
            norm = np.linalg.norm(normal)
            if norm > 0:
                normals[i] = normal / norm
            else:
                normals[i] = np.array([0, 0, 1])
        
        return normals
    
    def create_regular_grid(self, x_samples=50, y_samples=50):
        """Create a regular grid in parameter space."""
        x = np.linspace(0, 1, x_samples)
        y = np.linspace(0, 1, y_samples)
        x_grid, y_grid = np.meshgrid(x, y)
        
        grid_xy = np.column_stack([x_grid.ravel(), y_grid.ravel()])
        grid_uvw = self.interpolate(grid_xy)
        
        grid_uvw = grid_uvw.reshape(y_samples, x_samples, 3)
        grid_xy_reshaped = grid_xy.reshape(y_samples, x_samples, 2)
        # store last generated grid for optional interactive visualization
        self.last_grid_uvw = grid_uvw
        
        return grid_uvw, grid_xy_reshaped
    
    def evaluate_quality(self, sample_size=1000):
        """Evaluate parameterization quality."""
        n_points = len(self.points)
        if n_points > sample_size:
            print(f"  Sampling {sample_size} points from {n_points} for quality evaluation...")
            indices = np.random.choice(n_points, sample_size, replace=False)
            sample_xy = self.xy_params[indices]
            sample_points = self.points[indices]
        else:
            sample_xy = self.xy_params
            sample_points = self.points
        
        reconstructed = self.interpolate(sample_xy)
        errors = np.linalg.norm(sample_points - reconstructed, axis=1)
        
        metrics = {
            'mean_error': np.mean(errors),
            'max_error': np.max(errors),
            'std_error': np.std(errors),
            'rmse': np.sqrt(np.mean(errors**2)),
            'sample_size': len(sample_points)
        }
        
        print("\n Parameterization Quality Metrics:")
        print(f"  Sample size: {metrics['sample_size']}/{n_points} points")
        print(f"  Mean error: {metrics['mean_error']:.6f}")
        print(f"  Max error: {metrics['max_error']:.6f}")
        print(f"  RMSE: {metrics['rmse']:.6f}")
        print(f"  Std deviation: {metrics['std_error']:.6f}")
        
        return metrics
    
    def visualize(self, save_path='surface_visualization.png', grid_samples=30):
        """Visualize the parameterized surface and save to file."""
        print(f"\n Generating visualization...")
        
        fig = plt.figure(figsize=(15, 5))
        
        # 3D Cartesian surface
        ax1 = fig.add_subplot(131, projection='3d')
        ax1.scatter(self.points[:, 0], self.points[:, 1], self.points[:, 2], 
                   c='blue', s=1, alpha=0.5, label='Original points')
        
        grid_uvw, _ = self.create_regular_grid(grid_samples, grid_samples)
        ax1.plot_surface(grid_uvw[:, :, 0], grid_uvw[:, :, 1], grid_uvw[:, :, 2],
                       alpha=0.7, cmap='viridis', edgecolor='none')
        
        ax1.set_xlabel('U')
        ax1.set_ylabel('V')
        ax1.set_zlabel('W')
        ax1.set_title('3D Cartesian Surface (u,v,w)')
        ax1.legend()
        
        # Parameter space
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
        print(f" Visualization saved to: {save_path}")
        plt.close(fig)

    def visualize_interactive(self, path_3d=None, show_mesh=True, point_color=(0.6, 0.6, 0.6)):
        """Open an interactive Open3D viewer showing the point cloud and optional path.

        - path_3d: optional Nx3 array of waypoints to draw as a red line
        - show_mesh: if a grid was previously generated (self.last_grid_uvw), try to show it as a mesh
        """
        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.points)
            colors = np.tile(np.array(point_color, dtype=float), (len(self.points), 1))
            pcd.colors = o3d.utility.Vector3dVector(colors)

            geometries = [pcd]

            if path_3d is not None:
                path_pts = np.asarray(path_3d)
                if len(path_pts) >= 2:
                    lines = [[i, i+1] for i in range(len(path_pts)-1)]
                    line_set = o3d.geometry.LineSet()
                    line_set.points = o3d.utility.Vector3dVector(path_pts)
                    line_set.lines = o3d.utility.Vector2iVector(lines)
                    line_set.colors = o3d.utility.Vector3dVector([[1.0, 0.0, 0.0] for _ in lines])
                    geometries.append(line_set)

                    # start / end markers
                    bbox_size = np.linalg.norm(np.max(self.points, axis=0) - np.min(self.points, axis=0))
                    sphere_r = max(1e-4, 0.01 * bbox_size)
                    start_s = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_r)
                    start_s.translate(path_pts[0])
                    start_s.paint_uniform_color([0.0, 1.0, 0.0])
                    end_s = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_r)
                    end_s.translate(path_pts[-1])
                    end_s.paint_uniform_color([1.0, 0.0, 0.0])
                    geometries.extend([start_s, end_s])

            # optionally add mesh built from last grid
            if show_mesh and hasattr(self, 'last_grid_uvw'):
                try:
                    grid = self.last_grid_uvw
                    ny, nx, _ = grid.shape
                    vertices = grid.reshape(-1, 3)
                    triangles = []
                    for iy in range(ny - 1):
                        for ix in range(nx - 1):
                            i0 = iy * nx + ix
                            i1 = i0 + 1
                            i2 = i0 + nx
                            i3 = i2 + 1
                            triangles.append([i0, i2, i1])
                            triangles.append([i1, i2, i3])
                    mesh = o3d.geometry.TriangleMesh()
                    mesh.vertices = o3d.utility.Vector3dVector(vertices)
                    mesh.triangles = o3d.utility.Vector3iVector(triangles)
                    mesh.compute_vertex_normals()
                    mesh.paint_uniform_color([0.8, 0.8, 0.9])
                    geometries.insert(0, mesh)
                except Exception:
                    # don't fail visualization for mesh build errors
                    pass

            # Launch interactive viewer
            o3d.visualization.draw_geometries(geometries, window_name='Surface Viewer', width=1024, height=768)
        except Exception as e:
            print(f"Interactive Open3D visualization failed: {e}")
            print("Make sure you have a GUI available and Open3D installed with GUI support. Falling back to static images.")


def main():
    """
    Main function - Run surface parameterization workflow.
    """
    print("=" * 70)
    print("  SURFACE PARAMETERIZATION - Inverse Interpolation Approach")
    print("  Notation: (x,y) = parameter space | (u,v,w) = Cartesian space")
    print("=" * 70)
    
    # Configuration
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    POINT_CLOUD_FILE = os.path.join(script_dir, "point_cloud.ply")
    
    # Check if file exists
    if not os.path.exists(POINT_CLOUD_FILE):
        print(f"\n Error: '{os.path.basename(POINT_CLOUD_FILE)}' not found!")
        print(f"   Please ensure the PLY file is in: {script_dir}")
        return
    
    print(f"\n Loading point cloud: {POINT_CLOUD_FILE}")
    
    # Step 1: Initialize
    surf = SurfaceParameterization(point_cloud_path=POINT_CLOUD_FILE)
    
    # Step 2: Compute local frame
    print("\n Computing local coordinate frame...")
    surf.compute_local_frame()
    
    # Step 3: Compute parameterization
    print("\n Computing XY parameterization...")
    surf.compute_xy_parameterization(method='projection')
    
    # Step 4: Build interpolation
    print("\n Building inverse interpolation...")
    surf.build_inverse_interpolation(method='rbf', neighbors=50)
    
    # Step 5: Evaluate quality
    print("\n Evaluating quality...")
    metrics = surf.evaluate_quality(sample_size=1000)
    
    # Step 6: Test interpolation
    print("\n" + "=" * 70)
    print("  TESTING INVERSE INTERPOLATION")
    print("=" * 70)
    
    test_xy = np.array([
        [0.5, 0.5],   # Center
        [0.0, 0.0],   # Corner
        [1.0, 1.0],   # Opposite corner
        [0.25, 0.75]  # Random point
    ])
    
    interpolated = surf.interpolate(test_xy)
    normals = surf.compute_surface_normals(test_xy)
    
    for i, (xy, uvw, normal) in enumerate(zip(test_xy, interpolated, normals)):
        print(f"\n  Point {i+1}:")
        print(f"    Parameter (x,y): ({xy[0]:.3f}, {xy[1]:.3f})")
        print(f"    Cartesian (u,v,w): ({uvw[0]:.1f}, {uvw[1]:.1f}, {uvw[2]:.1f})")
        print(f"    Normal: ({normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f})")
    
    # Step 7: Generate path example
    print("\n" + "=" * 70)
    print("  GENERATING ROBOTIC SCANNING PATH")
    print("=" * 70)
    
    num_passes = 10
    points_per_pass = 20
    path_xy = []
    
    for i in range(num_passes):
        y = i / (num_passes - 1)
        x_line = np.linspace(0, 1, points_per_pass)
        if i % 2 == 1:
            x_line = x_line[::-1]
        for x in x_line:
            path_xy.append([x, y])
    
    path_xy = np.array(path_xy)
    path_3d = surf.interpolate(path_xy)
    path_length = np.sum(np.linalg.norm(np.diff(path_3d, axis=0), axis=1))
    
    print(f"\nGenerated scanning path:")
    print(f"  Waypoints: {len(path_3d)}")
    print(f"  Path length: {path_length:.2f} units")
    
    # Step 8: Create and export grid
    print("\n" + "=" * 70)
    print("  CREATING REGULAR GRID FOR PATH PLANNING")
    print("=" * 70)
    
    grid_uvw, grid_xy = surf.create_regular_grid(x_samples=50, y_samples=50)
    
    # Save files in the same directory as the script
    np.save(os.path.join(script_dir, 'surface_grid_uvw.npy'), grid_uvw)
    np.save(os.path.join(script_dir, 'surface_grid_xy.npy'), grid_xy)
    np.save(os.path.join(script_dir, 'scanning_path_uvw.npy'), path_3d)
    np.save(os.path.join(script_dir, 'scanning_path_xy.npy'), path_xy)
    
    print(f"\nGrid created: {grid_uvw.shape}")
    print(f"Saved files:")
    print(f"  - surface_grid_uvw.npy (Cartesian grid)")
    print(f"  - surface_grid_xy.npy (Parameter grid)")
    print(f"  - scanning_path_uvw.npy (3D path)")
    print(f"  - scanning_path_xy.npy (2D path)")
    
    # Step 9: Visualize
    surf.visualize(save_path=os.path.join(script_dir, 'surface_visualization.png'), grid_samples=30)
<<<<<<< HEAD
    # Try interactive Open3D viewer first (allows moving/rotating/zooming)
    try:
        print("\n Opening interactive 3D viewer (rotate/zoom/translate)...")
        # store grid so interactive mesh can be built if desired
        surf.last_grid_uvw = grid_uvw
        surf.visualize_interactive(path_3d=path_3d)
        print("Interactive viewer closed.")
    except Exception as e:
        print(f"Interactive viewer failed: {e}\nFalling back to static matplotlib visualization.")
        # Create path visualization (static fallback)
        print("\n Generating path visualization...")
        fig = plt.figure(figsize=(12, 5))

        ax1 = fig.add_subplot(121, projection='3d')
        ax1.scatter(surf.points[:, 0], surf.points[:, 1], surf.points[:, 2],
                   c='lightblue', s=1, alpha=0.2)
        ax1.plot(path_3d[:, 0], path_3d[:, 1], path_3d[:, 2],
                'r-', linewidth=2, label='Path')
        ax1.scatter(path_3d[0, 0], path_3d[0, 1], path_3d[0, 2],
                   c='green', s=100, marker='o', label='Start')
        ax1.scatter(path_3d[-1, 0], path_3d[-1, 1], path_3d[-1, 2],
                   c='red', s=100, marker='s', label='End')
        ax1.set_xlabel('U')
        ax1.set_ylabel('V')
        ax1.set_zlabel('W')
        ax1.set_title('3D Scanning Path')
        ax1.legend()

        ax2 = fig.add_subplot(122)
        ax2.scatter(surf.xy_params[:, 0], surf.xy_params[:, 1],
                   c='lightblue', s=5, alpha=0.5)
        ax2.plot(path_xy[:, 0], path_xy[:, 1], 'r-', linewidth=2, label='Path')
        ax2.scatter(path_xy[0, 0], path_xy[0, 1],
                   c='green', s=100, marker='o', label='Start')
        ax2.scatter(path_xy[-1, 0], path_xy[-1, 1],
                   c='red', s=100, marker='s', label='End')
        ax2.set_xlabel('x (parameter)')
        ax2.set_ylabel('y (parameter)')
        ax2.set_title('Parameter Space Path')
        ax2.set_aspect('equal')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(os.path.join(script_dir, 'scanning_path_visualization.png'), dpi=150, bbox_inches='tight')
        print(f"✓ Path visualization saved to: scanning_path_visualization.png")
        plt.close(fig)
=======
    
    # Create path visualization
    print("\n Generating path visualization...")
    fig = plt.figure(figsize=(12, 5))
    
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(surf.points[:, 0], surf.points[:, 1], surf.points[:, 2],
               c='lightblue', s=1, alpha=0.2)
    ax1.plot(path_3d[:, 0], path_3d[:, 1], path_3d[:, 2],
            'r-', linewidth=2, label='Path')
    ax1.scatter(path_3d[0, 0], path_3d[0, 1], path_3d[0, 2],
               c='green', s=100, marker='o', label='Start')
    ax1.scatter(path_3d[-1, 0], path_3d[-1, 1], path_3d[-1, 2],
               c='red', s=100, marker='s', label='End')
    ax1.set_xlabel('U')
    ax1.set_ylabel('V')
    ax1.set_zlabel('W')
    ax1.set_title('3D Scanning Path')
    ax1.legend()
    
    ax2 = fig.add_subplot(122)
    ax2.scatter(surf.xy_params[:, 0], surf.xy_params[:, 1],
               c='lightblue', s=5, alpha=0.5)
    ax2.plot(path_xy[:, 0], path_xy[:, 1], 'r-', linewidth=2, label='Path')
    ax2.scatter(path_xy[0, 0], path_xy[0, 1],
               c='green', s=100, marker='o', label='Start')
    ax2.scatter(path_xy[-1, 0], path_xy[-1, 1],
               c='red', s=100, marker='s', label='End')
    ax2.set_xlabel('x (parameter)')
    ax2.set_ylabel('y (parameter)')
    ax2.set_title('Parameter Space Path')
    ax2.set_aspect('equal')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(script_dir, 'scanning_path_visualization.png'), dpi=150, bbox_inches='tight')
    print(f"Path visualization saved to: scanning_path_visualization.png")
    plt.close(fig)
>>>>>>> b913a706d752b9ca1f872c89fd5cbc838e0f95d4
    
    # Final summary
    print("\n" + "=" * 70)
    print("  SURFACE PARAMETERIZATION COMPLETE!")
    print("=" * 70)
    print("\n Generated files:")
    print("  1. surface_visualization.png - 3D surface visualization")
    print("  2. scanning_path_visualization.png - Path visualization")
    print("  3. surface_grid_uvw.npy - Regular grid (Cartesian)")
    print("  4. surface_grid_xy.npy - Regular grid (Parameter)")
    print("  5. scanning_path_uvw.npy - Example scanning path (3D)")
    print("  6. scanning_path_xy.npy - Example scanning path (2D)")
    
    print("\n Usage:")
    print("  To load grids: grid_uvw = np.load('surface_grid_uvw.npy')")
    print("  To load path: path = np.load('scanning_path_uvw.npy')")
    
    print("\n All done! Check the PNG files for visualizations.")
    print("=" * 70)
    
    return surf


if __name__ == "__main__":
    """
    Run this file directly in VS Code:
    - Press F5 (Run)
    - Or right-click and select "Run Python File in Terminal"
    - Or use the play button in the top-right corner
    """
    try:
        surf = main()
    except Exception as e:
        print(f"\n Error occurred: {str(e)}")
        import traceback
        traceback.print_exc()
        print("\nPlease check:")
        print("  1. All required packages are installed (see requirements.txt)")
        print("  2. point_cloud.ply exists in the current directory")
        print("  3. You have write permissions in the current directory")
