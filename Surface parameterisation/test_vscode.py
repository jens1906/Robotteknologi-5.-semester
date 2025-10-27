"""
Quick test to verify VS Code compatibility
"""

import numpy as np
from surface_parameterization import SurfaceParameterization

print("Testing VS Code compatibility...")
print("=" * 60)

# Load and parameterize
surf = SurfaceParameterization(point_cloud_path="point_cloud.ply")
surf.compute_local_frame()
surf.compute_xy_parameterization(method='projection')
surf.build_inverse_interpolation(method='rbf', neighbors=50)

print("\n✓ Core functionality works!")

# Test visualization (should save to file, not display)
print("\nTesting visualization (saving to file)...")
surf.visualize(show_grid=False, show_original=True, grid_samples=10, 
              save_path='test_viz.png')

print("\n✓ Visualization saved successfully!")
print("=" * 60)
print("All tests passed - VS Code compatible!")
print("Check test_viz.png in the current directory")
