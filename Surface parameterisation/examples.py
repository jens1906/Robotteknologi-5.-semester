"""
Simple example demonstrating surface parameterization usage
"""

from surface_parameterization import SurfaceParameterization
import numpy as np
import matplotlib.pyplot as plt


def example_basic_usage():
    """Basic usage example."""
    print("\n" + "="*60)
    print("Example 1: Basic Surface Parameterization")
    print("="*60)
    
    # Load and parameterize
    surf = SurfaceParameterization(point_cloud_path="point_cloud.ply")
    surf.compute_local_frame()
    surf.compute_uv_parameterization(method='projection')
    surf.build_inverse_interpolation(method='rbf', neighbors=50)
    
    # Evaluate
    surf.evaluate_quality()
    
    return surf


def example_interpolation(surf):
    """Example of interpolating points."""
    print("\n" + "="*60)
    print("Example 2: Point Interpolation")
    print("="*60)
    
    # Define test points in UV space
    test_uv = np.array([
        [0.5, 0.5],
        [0.25, 0.25],
        [0.75, 0.75],
    ])
    
    # Interpolate to 3D
    xyz = surf.interpolate(test_uv)
    
    print("\nInterpolated points:")
    for i, (uv, pt) in enumerate(zip(test_uv, xyz)):
        print(f"  UV({uv[0]:.2f}, {uv[1]:.2f}) -> XYZ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f})")
    
    return xyz


def example_surface_normals(surf):
    """Example of computing surface normals."""
    print("\n" + "="*60)
    print("Example 3: Surface Normals")
    print("="*60)
    
    # Sample grid of UV points
    u = np.linspace(0.1, 0.9, 5)
    v = np.linspace(0.1, 0.9, 5)
    u_grid, v_grid = np.meshgrid(u, v)
    uv_samples = np.column_stack([u_grid.ravel(), v_grid.ravel()])
    
    # Compute normals
    normals = surf.compute_surface_normals(uv_samples)
    
    print(f"\nComputed {len(normals)} surface normals")
    print(f"Sample normal at UV(0.5, 0.5): {normals[12]}")
    
    return normals


def example_path_generation(surf):
    """Example of generating a scanning path."""
    print("\n" + "="*60)
    print("Example 4: Robotic Path Generation")
    print("="*60)
    
    # Generate a zigzag scanning pattern in UV space
    num_passes = 10
    points_per_pass = 20
    
    path_uv = []
    for i in range(num_passes):
        v = i / (num_passes - 1)
        u_line = np.linspace(0, 1, points_per_pass)
        
        # Reverse direction on alternate passes
        if i % 2 == 1:
            u_line = u_line[::-1]
        
        for u in u_line:
            path_uv.append([u, v])
    
    path_uv = np.array(path_uv)
    
    # Convert to 3D coordinates
    path_3d = surf.interpolate(path_uv)
    
    # Get tool orientations (normals)
    path_normals = surf.compute_surface_normals(path_uv)
    
    print(f"\nGenerated scanning path:")
    print(f"  Number of waypoints: {len(path_3d)}")
    print(f"  Path length: {np.sum(np.linalg.norm(np.diff(path_3d, axis=0), axis=1)):.3f} units")
    
    # Visualize the path
    fig = plt.figure(figsize=(12, 5))
    
    # 3D path
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(surf.points[:, 0], surf.points[:, 1], surf.points[:, 2],
               c='lightblue', s=1, alpha=0.3, label='Surface')
    ax1.plot(path_3d[:, 0], path_3d[:, 1], path_3d[:, 2],
            'r-', linewidth=2, label='Path')
    ax1.scatter(path_3d[0, 0], path_3d[0, 1], path_3d[0, 2],
               c='green', s=100, marker='o', label='Start')
    ax1.scatter(path_3d[-1, 0], path_3d[-1, 1], path_3d[-1, 2],
               c='red', s=100, marker='s', label='End')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('3D Scanning Path')
    ax1.legend()
    
    # UV space path
    ax2 = fig.add_subplot(122)
    ax2.scatter(surf.uv_params[:, 0], surf.uv_params[:, 1],
               c='lightblue', s=5, alpha=0.5, label='Surface points')
    ax2.plot(path_uv[:, 0], path_uv[:, 1], 'r-', linewidth=2, label='Path')
    ax2.scatter(path_uv[0, 0], path_uv[0, 1],
               c='green', s=100, marker='o', label='Start')
    ax2.scatter(path_uv[-1, 0], path_uv[-1, 1],
               c='red', s=100, marker='s', label='End')
    ax2.set_xlabel('u')
    ax2.set_ylabel('v')
    ax2.set_title('UV Space Scanning Pattern')
    ax2.set_aspect('equal')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('scanning_path.png', dpi=150)
    print("  Path visualization saved to 'scanning_path.png'")
    plt.show()
    
    return path_3d, path_normals


def example_grid_export(surf):
    """Example of creating and exporting a regular grid."""
    print("\n" + "="*60)
    print("Example 5: Regular Grid Export")
    print("="*60)
    
    # Create regular grid
    grid_xyz, grid_uv = surf.create_regular_grid(u_samples=30, v_samples=30)
    
    print(f"\nCreated regular grid:")
    print(f"  Shape: {grid_xyz.shape}")
    print(f"  Total points: {grid_xyz.shape[0] * grid_xyz.shape[1]}")
    
    # Save to files
    np.save('surface_grid_xyz.npy', grid_xyz)
    np.save('surface_grid_uv.npy', grid_uv)
    
    # Also save as text for easy inspection
    grid_xyz_flat = grid_xyz.reshape(-1, 3)
    np.savetxt('surface_grid_xyz.txt', grid_xyz_flat, 
               header='X Y Z coordinates of regular grid', fmt='%.6f')
    
    print(f"  Saved to: surface_grid_xyz.npy, surface_grid_uv.npy")
    print(f"  Also saved as text: surface_grid_xyz.txt")
    
    return grid_xyz, grid_uv


if __name__ == "__main__":
    print("\n" + "="*70)
    print("  SURFACE PARAMETERIZATION EXAMPLES")
    print("  Based on Inverse Interpolation Approach (IEEE TASE 2021)")
    print("="*70)
    
    # Run examples
    surf = example_basic_usage()
    example_interpolation(surf)
    example_surface_normals(surf)
    example_path_generation(surf)
    example_grid_export(surf)
    
    # Final visualization
    print("\n" + "="*60)
    print("Showing complete visualization...")
    print("="*60)
    surf.visualize(show_grid=True, show_original=True, grid_samples=30)
    
    print("\n" + "="*60)
    print("All examples completed successfully!")
    print("="*60)
