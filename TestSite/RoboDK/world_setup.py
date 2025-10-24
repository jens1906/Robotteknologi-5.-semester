"""
Scene Overhaul Setup

Creates:
  - Reference frame "WorldOrigo" at global (0,0,0)
  - Cylinder (DrivingCylinder) positioned at WorldOrigo origin
  - Reference frame "CylinderOffset" located at +Y = cylinder diameter from WorldOrigo

Re-run safe: will reuse existing items if they already exist.
"""
from robodk import robolink, robomath
from RDK_CylGen import make_smooth_cylinder
from load_rover import load_ideal_rover

WORLD_NAME = "WorldOrigo"
CYL_NAME = "WindTurbineTower"
OFFSET_NAME = "RoverCoordinate"
ROVER_NAME = "DrivingRover"

# Cylinder parameters (meters)
CYL_DIA_M = 6.0
CYL_HEIGHT_M = 10.0

RDK = robolink.Robolink()


def get_or_create_frame(name: str, parent=None):
    item = RDK.Item(name, robolink.ITEM_TYPE_FRAME)
    if item.Valid():
        return item
    frame = RDK.AddFrame(name, parent if parent else RDK.ActiveStation())
    # Identity pose (already at 0,0,0 relative to parent)
    frame.setPose(robomath.Mat([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]))
    return frame


def get_or_create_cylinder(name: str, parent=None):
    cyl = RDK.Item(name, robolink.ITEM_TYPE_OBJECT)
    if cyl.Valid():
        # Reparent and reset pose if a parent is specified
        if parent:
            cyl.setParent(parent)
            cyl.setPose(robomath.Mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]))
        return cyl
    cyl = make_smooth_cylinder(CYL_DIA_M, CYL_HEIGHT_M, name)
    if parent and cyl and cyl.Valid():
        cyl.setParent(parent)
        cyl.setPose(robomath.Mat([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]))
    return cyl


def main():
    print("=== World Scene Setup ===")
    world = get_or_create_frame(WORLD_NAME)
    # Ensure world at absolute origin
    world.setPoseAbs(robomath.Mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]))

    cyl = get_or_create_cylinder(CYL_NAME, world)
    if not cyl or not cyl.Valid():
        print("✗ Failed to create or locate cylinder")
        return

    # Determine cylinder diameter in mm (original function used meters input -> mm geometry)
    cyl_diameter_mm = CYL_DIA_M * 1000.0

    # Create offset frame at +Y = diameter
    offset_frame = get_or_create_frame(OFFSET_NAME, world)
    # Reset offset frame first to avoid compounding previous orientation
    offset_frame.setPose(robomath.Mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]))
    # We want its Z to point radially outward (original +Y direction) and X keep original X.
    # A pure -90 deg rotation about X achieves: Y->Z, Z->-Y.
    # Build explicit homogeneous matrix to avoid Euler re-interpretation drift:
    # Rotation Rx(-90): [[1,0,0],[0,0,1],[0,-1,0]]
    offset_pose = robomath.Mat([
        [1, 0, 0, 0],
        [0, 0, 1, cyl_diameter_mm/2],  # translate along original +Y AFTER rotation effect encoded directly
        [0,-1, 0, 0],
        [0, 0, 0, 1]
    ])
    offset_frame.setPose(offset_pose)
    # Verification: print resulting Euler decomposition (RoboDK default) and raw matrix
    pose_check = offset_frame.Pose()
    print("  RoverCoordinate raw rotation rows:")
    print(f"    {pose_check[0,0]: .3f} {pose_check[0,1]: .3f} {pose_check[0,2]: .3f}")
    print(f"    {pose_check[1,0]: .3f} {pose_check[1,1]: .3f} {pose_check[1,2]: .3f}")
    print(f"    {pose_check[2,0]: .3f} {pose_check[2,1]: .3f} {pose_check[2,2]: .3f}")

    # Load rover and parent under offset frame
    rover = RDK.Item(ROVER_NAME, robolink.ITEM_TYPE_OBJECT)
    if not rover.Valid():
        rover = load_ideal_rover(ROVER_NAME)
    if rover and rover.Valid():
        rover.setParent(offset_frame)
        # Place rover so its origin sits slightly above ground (adjust Z if needed)
        rover_local_pose = robomath.transl(0, 0, 130)
        rover.setPose(rover_local_pose)
        print(f"✓ Rover '{ROVER_NAME}' loaded & parented under '{OFFSET_NAME}'")
    else:
        print(f"⚠ Rover '{ROVER_NAME}' not loaded (check STL path).")

    print(f"✓ Frame '{WORLD_NAME}' at origin")
    print(f"✓ Cylinder '{CYL_NAME}' at origin under '{WORLD_NAME}' (Diameter {cyl_diameter_mm:.0f} mm)")
    print(f"✓ Offset frame '{OFFSET_NAME}' at +Y={cyl_diameter_mm/2:.0f} mm (Z axis outward) relative to '{WORLD_NAME}' (target Rx=-90°)")
    print("Done.")

if __name__ == "__main__":
    main()
