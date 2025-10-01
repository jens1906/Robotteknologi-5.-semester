"""
Simple RoboDK Setup: Just Cylinder and Rover
Load the basic objects without complex positioning
"""

from robodk import robolink
from RDK_CylGen import make_smooth_cylinder
from load_rover import load_ideal_rover
from robodk import robomath

RDK = robolink.Robolink()

def setup_basic_scene():
    """
    Create just the cylinder and rover - no complex positioning
    User can manually position the rover for driving simulation
    """
    print("=== Basic RoboDK Scene Setup ===\n")
    
    # Create cylinder
    print("1. Creating cylinder...")
    cylinder = make_smooth_cylinder(6.0, 10.0, "DrivingCylinder")
    
    if not cylinder:
        print("✗ Failed to create cylinder")
        return None, None
    
    # Load rover
    print("\n2. Loading rover...")
    rover = load_ideal_rover("DrivingRover")
    
    if not rover:
        print("✗ Failed to load rover")
        return cylinder, None

    # Automatically reposition rover: move it back along -Y and up in Z.
    # Updated requirement: Y shift = -(cylinder_radius + 300 mm), Z shift = +700 mm.
    # Cylinder diameter is provided in meters to make_smooth_cylinder (6.0 m).
    cylinder_diameter_m = 6.0
    cylinder_radius_mm = (cylinder_diameter_m * 1000.0) / 2.0  # 3000 mm
    # Requested: use full radius plus 300 mm clearance => -(3000 + 300) = -3300 mm
    target_y = cylinder_radius_mm + 300.0
    target_z = 700.0
    target_x = 0.0

    # Keep rover orientation as-is (identity rotation) while translating.
    pose = robomath.transl(target_x, target_y, target_z)
    try:
        rover.setPose(pose)
        print(f"\n✓ Repositioned rover to (x={target_x:.1f}, y={target_y:.1f}, z={target_z:.1f}) mm")
        print(f"  (Computed using radius + 300mm => {cylinder_radius_mm:.1f} + 300 = {abs(target_y):.1f} mm offset)")
    except Exception as e:
        print(f"Warning: Failed to move rover automatically: {e}")
    
    print("\n✓ Scene ready!")
    print("✓ Cylinder: 6m diameter, 10m height")
    print("✓ Rover: IdealRover.stl loaded")
    print("\nNow manually position the rover in RoboDK:")
    print("- Right-click rover → Move")
    print("- Position it tangent to cylinder surface for driving simulation")
    
    return cylinder, rover

if __name__ == "__main__":
    print("RoboDK Basic Scene Setup\n")
    cylinder, rover = setup_basic_scene()
    
    if cylinder and rover:
        print("\n=== Success! ===")
        print("Both objects are loaded and rover repositioned.")
        print("Adjust further manually if needed in RoboDK.")
    else:
        print("\n✗ Setup failed - check RoboDK connection and STL file.")