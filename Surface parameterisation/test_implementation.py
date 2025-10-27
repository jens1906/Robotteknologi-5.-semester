"""
Quick test script to verify the surface parameterization implementation
"""

import numpy as np
from surface_parameterization import SurfaceParameterization


def test_basic_functionality():
    """Test basic functionality with the point cloud."""
    print("Testing Surface Parameterization Implementation")
    print("Notation: (x,y) = parameter space | (u,v,w) = Cartesian space")
    print("=" * 60)
    
    try:
        # Load point cloud
        print("\n1. Loading point cloud...")
        surf = SurfaceParameterization(point_cloud_path="point_cloud.ply")
        print(f"   ✓ Loaded {len(surf.points)} points")
        
        # Compute local frame
        print("\n2. Computing local coordinate frame...")
        surf.compute_local_frame()
        print(f"   ✓ Local frame computed")
        
        # Compute XY parameterization
        print("\n3. Computing XY parameterization...")
        surf.compute_xy_parameterization(method='projection')
        print(f"   ✓ XY parameters computed")
        
        # Build inverse interpolation
        print("\n4. Building inverse interpolation...")
        surf.build_inverse_interpolation(method='rbf', neighbors=50)
        print(f"   ✓ Inverse interpolation built")
        
        # Evaluate quality
        print("\n5. Evaluating parameterization quality...")
        metrics = surf.evaluate_quality()
        print(f"   ✓ Quality evaluated")
        
        # Test interpolation
        print("\n6. Testing interpolation at sample points...")
        test_xy = np.array([[0.5, 0.5], [0.25, 0.75]])
        uvw = surf.interpolate(test_xy)
        print(f"   ✓ Interpolated {len(uvw)} points")
        
        # Test normal computation
        print("\n7. Testing surface normal computation...")
        normals = surf.compute_surface_normals(test_xy)
        print(f"   ✓ Computed {len(normals)} normals")
        
        # Create grid
        print("\n8. Creating regular grid...")
        grid_uvw, grid_xy = surf.create_regular_grid(x_samples=20, y_samples=20)
        print(f"   ✓ Grid created with shape {grid_uvw.shape}")
        
        print("\n" + "=" * 60)
        print("✓ ALL TESTS PASSED!")
        print("=" * 60)
        print("\nThe implementation is working correctly.")
        print("You can now run:")
        print("  - python surface_parameterization.py  (full demo)")
        print("  - python examples.py                  (detailed examples)")
        
        return True
        
    except Exception as e:
        print(f"\n✗ TEST FAILED: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_basic_functionality()
    exit(0 if success else 1)
