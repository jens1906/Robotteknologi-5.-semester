"""
Combined auto-orientation + manual motion control for 'RoverCoordinate'.

Features:
  - Continuously keeps Z axis radial (outward in XY plane from tower center)
  - Preserves existing X,Y as much as possible (only re-orthogonalized if degenerate)
  - Interactive keys (Windows msvcrt):
      W : translate along local -Y
      S : translate along local +Y
      A : rotate +STEP_DEG about local Z
      D : rotate -STEP_DEG about local Z
      Q / ESC : quit

Usage:
  1. Run world_setup.py to create scene and frames.
  2. Run this script. Move with W/S/A/D; orientation of Z stays radial automatically.

Notes:
  - Translation does not clamp radius; if you need to remain on a cylinder surface, add radius enforcement.
  - Auto-orientation thread updates at ~20 Hz.
"""
import time
import math
import threading
from robodk import robolink, robomath

FRAME_NAME = "RoverCoordinate"
SLEEP_SEC = 0.05
STEP_DEG = 0.5
STEP_MOVE_MM = 50.0

RDK = robolink.Robolink()

try:
    import msvcrt
    WINDOWS = True
except ImportError:
    WINDOWS = False

_stop_flag = False


def get_frame(name: str):
    item = RDK.Item(name, robolink.ITEM_TYPE_FRAME)
    return item if item.Valid() else None


def normalize(v):
    n = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if n < 1e-9:
        return [0.0,0.0,0.0]
    return [v[0]/n, v[1]/n, v[2]/n]


def cross(a,b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def set_pose_axes(frame, x_axis, y_axis, z_axis, t):
    pose = robomath.Mat([
        [x_axis[0], y_axis[0], z_axis[0], t[0]],
        [x_axis[1], y_axis[1], z_axis[1], t[1]],
        [x_axis[2], y_axis[2], z_axis[2], t[2]],
        [0,0,0,1]
    ])
    frame.setPose(pose)


def auto_orient_loop():
    global _stop_flag
    frame = get_frame(FRAME_NAME)
    if not frame:
        print(f"✗ Frame '{FRAME_NAME}' not found. Run world_setup.py first.")
        return
    print("[Auto] Maintaining Z radial")
    while not _stop_flag:
        pose = frame.Pose()
        tx = pose[0,3]; ty = pose[1,3]; tz = pose[2,3]
        radial = [tx, ty, 0.0]
        rnorm = math.hypot(radial[0], radial[1])
        if rnorm < 1e-6:
            radial = [0.0, 1.0, 0.0]
        else:
            radial = [radial[0]/rnorm, radial[1]/rnorm, 0.0]
        z_axis = radial
        x_axis = [pose[0,0], pose[1,0], pose[2,0]]
        y_axis = [pose[0,1], pose[1,1], pose[2,1]]
        # Check for degeneracy with new Z and re-orthogonalize minimally
        def is_parallel(a,b):
            dot = a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
            na = math.sqrt(a[0]**2+a[1]**2+a[2]**2)
            nb = math.sqrt(b[0]**2+b[1]**2+b[2]**2)
            if na<1e-9 or nb<1e-9:
                return True
            return abs(dot/(na*nb)) > 0.999
        changed = False
        if is_parallel(x_axis, z_axis):
            # rebuild from Y
            if is_parallel(y_axis, z_axis):
                y_axis = [1.0,0.0,0.0]
                d = y_axis[0]*z_axis[0]+y_axis[1]*z_axis[1]+y_axis[2]*z_axis[2]
                y_axis = [y_axis[0]-d*z_axis[0], y_axis[1]-d*z_axis[1], y_axis[2]-d*z_axis[2]]
            x_axis = cross(y_axis, z_axis)
            changed = True
        if is_parallel(y_axis, z_axis):
            y_axis = cross(z_axis, x_axis)
            changed = True
        if changed:
            x_axis = normalize(x_axis)
            y_axis = normalize(y_axis)
        # Final orthonormal tidy
        x_axis = normalize(cross(y_axis, z_axis))
        y_axis = normalize(cross(z_axis, x_axis))
        set_pose_axes(frame, x_axis, y_axis, z_axis, [tx,ty,tz])
        time.sleep(SLEEP_SEC)


def rotate_z(frame, delta_deg):
    pose = frame.Pose()
    rz = robomath.rotz(delta_deg)
    frame.setPose(pose * rz)


def translate_local_y(frame, delta_mm):
    pose = frame.Pose()
    # local Y column
    ly = [pose[0,1], pose[1,1], pose[2,1]]
    new_t = [pose[0,3] + ly[0]*delta_mm,
             pose[1,3] + ly[1]*delta_mm,
             pose[2,3] + ly[2]*delta_mm]
    new_pose = robomath.Mat([
        [pose[0,0], pose[0,1], pose[0,2], new_t[0]],
        [pose[1,0], pose[1,1], pose[1,2], new_t[1]],
        [pose[2,0], pose[2,1], pose[2,2], new_t[2]],
        [0,0,0,1]
    ])
    frame.setPose(new_pose)


def interactive_loop():
    global _stop_flag
    frame = get_frame(FRAME_NAME)
    if not frame:
        print(f"✗ Frame '{FRAME_NAME}' not found. Run world_setup.py first.")
        _stop_flag = True
        return
    if not WINDOWS:
        print("Windows-only interactive control (msvcrt not available)")
        _stop_flag = True
        return
    print("Controls: W=-Y, S=+Y, A=+Z rot, D=-Z rot, Q/ESC=quit")
    while not _stop_flag:
        if msvcrt.kbhit():
            key = msvcrt.getch().lower()
            if key in (b'q', b'\x1b'):
                _stop_flag = True
            elif key == b'w':
                translate_local_y(frame, -STEP_MOVE_MM)
            elif key == b's':
                translate_local_y(frame, STEP_MOVE_MM)
            elif key == b'a':
                rotate_z(frame, STEP_DEG)
            elif key == b'd':
                rotate_z(frame, -STEP_DEG)
        time.sleep(0.02)


def main():
    t = threading.Thread(target=auto_orient_loop, daemon=True)
    t.start()
    try:
        interactive_loop()
    finally:
        print("Stopping...")
        global _stop_flag
        _stop_flag = True
        t.join(timeout=1.0)
        print("Done.")

if __name__ == '__main__':
    main()
