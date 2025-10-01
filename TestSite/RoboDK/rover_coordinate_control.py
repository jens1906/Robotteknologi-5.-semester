"""
RoverCoordinate Frame Rotation Control

Controls:
    W : Translate along local -Y (inward/outward axis depending on orientation)
    S : Translate along local +Y
    A : Rotate RoverCoordinate +STEP_DEG around its local Z axis (counter-clockwise when looking from +Z)
    D : Rotate RoverCoordinate -STEP_DEG around its local Z axis
    Q / ESC : Quit

Prerequisites:
  Run world_setup.py first so the frame 'RoverCoordinate' exists and is oriented (Z outward).

Adjust STEP_DEG to change rotation increment. Adjust STEP_MOVE_MM to change translation step.
"""
import time
import math
import sys
from robodk import robolink, robomath

STEP_DEG = 0.5            # yaw rotation step (A/D)
STEP_MOVE_MM = 50.0       # translation step along local Y (W/S)
# Optional fallback radius (mm) if cylinder lookup fails (6.0 m diameter -> 3000 mm radius)
FALLBACK_RADIUS_MM = 3000.0
FRAME_NAME = "RoverCoordinate"

try:
    import msvcrt
    WINDOWS = True
except ImportError:
    WINDOWS = False

RDK = robolink.Robolink()
_TOWER_RADIUS = None  # cached radius in mm

def get_frame(name: str):
    frm = RDK.Item(name, robolink.ITEM_TYPE_FRAME)
    return frm if frm.Valid() else None

def rotate_frame_z(frame, delta_deg):
    # Current pose
    pose = frame.Pose()
    # Extract rotation part
    rz = robomath.rotz(delta_deg)
    # New pose: pose * rz (post-multiply to rotate in local coordinates)
    new_pose = pose * rz
    frame.setPose(new_pose)

def translate_frame_local_y(frame, delta_mm):
    pose = frame.Pose()
    local_y = [pose[0,1], pose[1,1], pose[2,1]]
    tx = pose[0,3] + local_y[0]*delta_mm
    ty = pose[1,3] + local_y[1]*delta_mm
    tz = pose[2,3] + local_y[2]*delta_mm
    new_pose = robomath.Mat([
        [pose[0,0], pose[0,1], pose[0,2], tx],
        [pose[1,0], pose[1,1], pose[1,2], ty],
        [pose[2,0], pose[2,1], pose[2,2], tz],
        [0,0,0,1]
    ])
    # Optionally keep radius locked if desired: uncomment next line
    new_pose = _enforce_radius(new_pose)
    frame.setPose(new_pose)

def _compute_tower_radius():
    """Attempt to derive tower radius from the cylinder object bounding box."""
    global _TOWER_RADIUS
    if _TOWER_RADIUS is not None:
        return _TOWER_RADIUS
    cyl = RDK.Item('WindTurbineTower', robolink.ITEM_TYPE_OBJECT)
    if cyl.Valid():
        try:
            bb = cyl.BoundingBox()  # [x_min,y_min,z_min,x_max,y_max,z_max]
            dx = bb[3] - bb[0]
            dy = bb[4] - bb[1]
            diameter = max(abs(dx), abs(dy))
            if diameter > 0:
                _TOWER_RADIUS = diameter / 2.0
                return _TOWER_RADIUS
        except Exception:
            pass
    _TOWER_RADIUS = FALLBACK_RADIUS_MM
    return _TOWER_RADIUS

def _enforce_radius(pose: robomath.Mat) -> robomath.Mat:
    """Project translation component onto circle of tower radius in XY plane, preserving Z."""
    radius = _compute_tower_radius()
    tx = pose[0,3]
    ty = pose[1,3]
    # If at/near origin, nudge to +Y (initial placement)
    r = math.hypot(tx, ty)
    if r < 1e-6:
        tx, ty = 0.0, radius
    else:
        scale = radius / r
        tx *= scale
        ty *= scale
    # Rebuild pose with adjusted translation
    adj_pose = robomath.Mat([
        [pose[0,0], pose[0,1], pose[0,2], tx],
        [pose[1,0], pose[1,1], pose[1,2], ty],
        [pose[2,0], pose[2,1], pose[2,2], pose[2,3]],
        [0,0,0,1]
    ])
    return adj_pose


def interactive_loop():
    frame = get_frame(FRAME_NAME)
    if not frame:
        print(f"✗ Frame '{FRAME_NAME}' not found. Run world_setup.py first.")
        return
    print("RoverCoordinate Control")
    print("W=-Y trans, S=+Y trans, A=+Z rot, D=-Z rot, Q/ESC=quit")
    if not WINDOWS:
        print("Non-Windows environment: msvcrt not available, exiting.")
        return
    running = True
    while running:
        if msvcrt.kbhit():
            key = msvcrt.getch().lower()
            if key in (b'q', b'\x1b'):
                running = False
            elif key == b'w':
                translate_frame_local_y(frame, -STEP_MOVE_MM)
            elif key == b's':
                translate_frame_local_y(frame, STEP_MOVE_MM)
            elif key == b'a':
                rotate_frame_z(frame, STEP_DEG)
            elif key == b'd':
                rotate_frame_z(frame, -STEP_DEG)
        time.sleep(0.02)


def main():
    if '--auto' in sys.argv:
        frame = get_frame(FRAME_NAME)
        if not frame:
            print(f"✗ Frame '{FRAME_NAME}' not found.")
            return
        for i in range(36):  # 36 * 10 deg = full circle when STEP_DEG=10
            rotate_frame_z(frame, STEP_DEG)
            time.sleep(0.1)
        print("Auto rotation complete.")
    else:
        interactive_loop()

if __name__ == '__main__':
    main()
