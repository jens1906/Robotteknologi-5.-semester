"""
Auto orientation script:
Keeps the frame 'RoverCoordinate' oriented so that:
    - Its Z axis points orthogonally outward from the tower surface (radial from tower center in XY plane)
    - X and Y axes are left unchanged (only re-orthogonalized if they become invalid)

Assumptions:
  - Tower center is at (0,0) in the parent frame (WorldOrigo) of RoverCoordinate.
  - RoverCoordinate is parented directly under WorldOrigo (or a frame aligned with it) so XY plane is tower cross-section.
  - Run world_setup.py first to create items.

Behavior:
  - Loops at a small interval, reads current translation of RoverCoordinate, re-normalizes orientation.
  - Leaves translation untouched.

Stop:
  - Press Ctrl+C in the RoboDK script console or stop the script from RoboDK.
"""
import time
import math
from robodk import robolink, robomath

FRAME_NAME = "RoverCoordinate"
SLEEP_SEC = 0.05  # update rate (~20 Hz)

RDK = robolink.Robolink()

def get_frame(name: str):
    item = RDK.Item(name, robolink.ITEM_TYPE_FRAME)
    return item if item.Valid() else None

def build_pose_from_axes(x_axis, y_axis, z_axis, translation):
    return robomath.Mat([
        [x_axis[0], y_axis[0], z_axis[0], translation[0]],
        [x_axis[1], y_axis[1], z_axis[1], translation[1]],
        [x_axis[2], y_axis[2], z_axis[2], translation[2]],
        [0,0,0,1]
    ])

def normalize(v):
    n = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if n < 1e-9:
        return [0.0,0.0,0.0]
    return [v[0]/n, v[1]/n, v[2]/n]

def cross(a,b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def auto_orient_loop():
    frame = get_frame(FRAME_NAME)
    if not frame:
        print(f"✗ Frame '{FRAME_NAME}' not found. Run world_setup.py first.")
        return
    print("Auto orienting RoverCoordinate (Z=radial outward, preserving X/Y)")
    while True:
        pose = frame.Pose()
        tx = pose[0,3]; ty = pose[1,3]; tz = pose[2,3]
        # Radial vector in XY from tower center
        radial = [tx, ty, 0.0]
        radial_norm = math.hypot(radial[0], radial[1])
        if radial_norm < 1e-6:
            # If at center, default outward along +Y
            radial = [0.0, 1.0, 0.0]
        else:
            radial = [radial[0]/radial_norm, radial[1]/radial_norm, 0.0]
        z_axis = radial  # outward normal
        # Preserve existing X,Y axes (columns 0,1) as much as possible
        x_axis = [pose[0,0], pose[1,0], pose[2,0]]
        y_axis = [pose[0,1], pose[1,1], pose[2,1]]
        # Check degeneracy: if X or Y nearly parallel to new Z, attempt minimal fix
        def is_parallel(a,b):
            dot = a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
            na = math.sqrt(a[0]**2+a[1]**2+a[2]**2)
            nb = math.sqrt(b[0]**2+b[1]**2+b[2]**2)
            if na<1e-9 or nb<1e-9:
                return True
            cosang = abs(dot/(na*nb))
            return cosang > 0.999
        # If X parallel to Z, rebuild X from cross(Y,Z)
        if is_parallel(x_axis, z_axis):
            # Ensure Y not parallel; if it is, pick an arbitrary orthogonal vector
            if is_parallel(y_axis, z_axis):
                # Choose world X as fallback then project
                y_axis = [1.0,0.0,0.0]
                d = y_axis[0]*z_axis[0]+y_axis[1]*z_axis[1]+y_axis[2]*z_axis[2]
                y_axis = [y_axis[0]-d*z_axis[0], y_axis[1]-d*z_axis[1], y_axis[2]-d*z_axis[2]]
                y_axis = normalize(y_axis)
            x_axis = cross(y_axis, z_axis)
        # If Y parallel to Z, rebuild Y from cross(Z,X)
        if is_parallel(y_axis, z_axis):
            y_axis = cross(z_axis, x_axis)
        # Orthonormalize minimally: recompute X to ensure orthogonality, then Y again
        x_axis = normalize(cross(y_axis, z_axis))
        y_axis = normalize(cross(z_axis, x_axis))
        new_pose = build_pose_from_axes(x_axis, y_axis, z_axis, [tx, ty, tz])
        frame.setPose(new_pose)
        time.sleep(SLEEP_SEC)

def main():
    auto_orient_loop()

if __name__ == "__main__":
    main()
