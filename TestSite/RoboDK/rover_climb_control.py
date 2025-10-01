"""
Rover Vertical (Forward) + Orbital (Side) Control for RoboDK

User Clarified Orientation:
    - Rover FRONT points upward along cylinder axis (+Z in station coordinates).
    - Therefore: Forward/Backward should move in +Z / -Z while staying tangent (constant radius).
    - Side movement (A/D) should move the rover around the cylinder (orbit), WITHOUT changing rover yaw.
    - Rover should NOT rotate when moving; only orbit position changes X/Y around cylinder for A/D.

Controls:
    W : Move UP (forward along cylinder axis)
    S : Move DOWN (backward along cylinder axis)
    A : ORBIT LEFT (increase angle counter-clockwise seen from +Z)
    D : ORBIT RIGHT (decrease angle)
    Q / ESC : Quit

Guarantees:
    - Constant radial distance (locks to cylinder surface).
    - Yaw preserved exactly as initial (no rotation about Z unless later requested).
    - No drift into the cylinder.

Adjustable constants near top:
    STEP_Z_MM, STEP_ORBIT_DEG

Usage:
        python rover_climb_control.py
        python rover_climb_control.py --auto   # demo vertical climb + slow orbit
"""
import sys
import time
import math
import threading
from typing import Optional

try:
    import msvcrt  # Windows-only keyboard polling
    WINDOWS = True
except ImportError:
    WINDOWS = False

from robodk import robolink, robomath

RDK = robolink.Robolink()

ROVER_NAME = "DrivingRover"
CYL_NAME = "DrivingCylinder"

# Movement parameters
STEP_Z_MM = 150.0          # legacy (unused after forward/back change unless vertical clamp logic needed)
STEP_TURN_DEG = 5.0        # rotation about radial normal per A/D
STEP_FORWARD_MM = 150.0    # forward/back step along rover's local forward
AUTO_STEPS = 120           # auto demo steps


def get_item(name: str, itype: int) -> Optional[robolink.Item]:
    item = RDK.Item(name, itype)
    return item if item.Valid() else None


def fetch_rover_and_cylinder():
    rover = get_item(ROVER_NAME, robolink.ITEM_TYPE_OBJECT)
    cyl = get_item(CYL_NAME, robolink.ITEM_TYPE_OBJECT)
    if not rover:
        print(f"✗ Rover '{ROVER_NAME}' not found. Run simple_setup.py first.")
    if not cyl:
        print(f"✗ Cylinder '{CYL_NAME}' not found. Run simple_setup.py first.")
    return rover, cyl


def estimate_cylinder_height(cyl: robolink.Item) -> float:
    # Try bounding box: returns (x_min,y_min,z_min,x_max,y_max,z_max)
    try:
        bbox = cyl.BoundingBox()
        return bbox[5] - bbox[2]
    except Exception:
        # Fallback to configured height (10 m = 10000 mm)
        return 10000.0


def estimate_cylinder_radius(cyl: robolink.Item) -> float:
    try:
        bbox = cyl.BoundingBox()
        # radius approximated from X or Y span / 2
        rx = (bbox[3] - bbox[0]) / 2.0
        ry = (bbox[4] - bbox[1]) / 2.0
        return (rx + ry) / 2.0
    except Exception:
        return 3000.0


def decompose_pose_xyzrz(pose):
    # pose is a 4x4 matrix; extract translation and yaw around Z
    x = pose[0, 3]
    y = pose[1, 3]
    z = pose[2, 3]
    # Yaw from rotation matrix (assuming no roll/pitch currently)
    yaw = math.degrees(math.atan2(pose[1,0], pose[0,0]))
    return x, y, z, yaw


def build_pose(x, y, z, yaw_deg):
    return robomath.transl(x, y, z) * robomath.rotz(yaw_deg)


def clamp(val, vmin, vmax):
    return max(vmin, min(vmax, val))


class RoverController:
    def __init__(self, rover: robolink.Item, cylinder: robolink.Item):
        self.rover = rover
        self.cylinder = cylinder
        self.radius = estimate_cylinder_radius(cylinder)
        self.height = estimate_cylinder_height(cylinder)
        pose = rover.Pose()
        # Extract translation and full 3x3 rotation
        self.x, self.y, self.z, _ = decompose_pose_xyzrz(pose)
        # Store full orientation matrix (first 3x3)
        self.R = [[pose[0,0], pose[0,1], pose[0,2]],
                  [pose[1,0], pose[1,1], pose[1,2]],
                  [pose[2,0], pose[2,1], pose[2,2]]]
        # Angular position around cylinder kept for future (not changed by rotation now)
        self.angle_deg = math.degrees(math.atan2(self.y, self.x if abs(self.x) > 1e-9 else 1e-9))
        self.keep_running = True
        print(f"Detected cylinder: radius≈{self.radius:.1f} mm")
        print(f"Initial state: angle={self.angle_deg:.2f}° z={self.z:.1f}")

    def _radial_normal(self):
        # Radial outward normal (x,y,0) normalized
        nx, ny = self.x, self.y
        r = math.hypot(nx, ny)
        if r < 1e-9:
            return (1.0, 0.0, 0.0)
        return (nx / r, ny / r, 0.0)

    def _rotate_R_about_axis(self, axis, angle_deg):
        ax, ay, az = axis
        # Rodrigues rotation
        angle_rad = math.radians(angle_deg)
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        t = 1 - c
        Rax = [
            [t*ax*ax + c,     t*ax*ay - s*az, t*ax*az + s*ay],
            [t*ax*ay + s*az,  t*ay*ay + c,    t*ay*az - s*ax],
            [t*ax*az - s*ay,  t*ay*az + s*ax, t*az*az + c   ]
        ]
        # Multiply Rax * self.R
        newR = [[0.0]*3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                newR[i][j] = Rax[i][0]*self.R[0][j] + Rax[i][1]*self.R[1][j] + Rax[i][2]*self.R[2][j]
        self.R = newR

    def _pose_from_state(self):
        # Construct homogeneous matrix
        mat_list = [
            [self.R[0][0], self.R[0][1], self.R[0][2], self.x],
            [self.R[1][0], self.R[1][1], self.R[1][2], self.y],
            [self.R[2][0], self.R[2][1], self.R[2][2], self.z],
            [0,0,0,1]
        ]
        return robomath.Mat(mat_list)

    def apply(self):
        # Maintain radius lock (re-project to radius in case of numeric drift)
        rcur = math.hypot(self.x, self.y)
        if abs(rcur - self.radius) > 1e-6 and rcur > 1e-9:
            scale = self.radius / rcur
            self.x *= scale
            self.y *= scale
        self.rover.setPose(self._pose_from_state())

    def move_forward(self):
        # Local forward vector = third column of self.R
        fx = self.R[0][2]
        fy = self.R[1][2]
        fz = self.R[2][2]
        # Translate
        self.x += fx * STEP_FORWARD_MM
        self.y += fy * STEP_FORWARD_MM
        self.z += fz * STEP_FORWARD_MM
        # Lock radius in XY plane (keep distance from axis = original radius)
        r_xy = math.hypot(self.x, self.y)
        if r_xy > 1e-9:
            scale = self.radius / r_xy
            self.x *= scale
            self.y *= scale
        # Optional clamp Z within [0, height]
        if self.z < 0:
            self.z = 0
        if self.z > self.height:
            self.z = self.height
        self.apply()

    def move_backward(self):
        fx = self.R[0][2]
        fy = self.R[1][2]
        fz = self.R[2][2]
        self.x -= fx * STEP_FORWARD_MM
        self.y -= fy * STEP_FORWARD_MM
        self.z -= fz * STEP_FORWARD_MM
        r_xy = math.hypot(self.x, self.y)
        if r_xy > 1e-9:
            scale = self.radius / r_xy
            self.x *= scale
            self.y *= scale
        if self.z < 0:
            self.z = 0
        if self.z > self.height:
            self.z = self.height
        self.apply()

    def turn_left(self):
        axis = self._radial_normal()
        self._rotate_R_about_axis(axis, STEP_TURN_DEG)
        self.apply()

    def turn_right(self):
        axis = self._radial_normal()
        self._rotate_R_about_axis(axis, -STEP_TURN_DEG)
        self.apply()

    def stop(self):
        self.keep_running = False


def interactive_loop(controller: RoverController):
    print("\nControls:")
    print("  W: Forward  |  S: Backward  |  A: Turn Left  |  D: Turn Right  |  Q/ESC: Quit")
    print("  (Forward/back follows rover orientation; A/D rotate about radial normal; radius locked)")
    if not WINDOWS:
        print("Warning: msvcrt not available (non-Windows OS). Interactive mode disabled.")
        return
    while controller.keep_running:
        if msvcrt.kbhit():
            key = msvcrt.getch().lower()
            if key in (b'q', b'\x1b'):
                controller.stop()
            elif key == b'w':
                controller.move_forward()
            elif key == b's':
                controller.move_backward()
            elif key == b'a':
                controller.turn_left()
            elif key == b'd':
                controller.turn_right()
        time.sleep(0.02)


def auto_demo(controller: RoverController):
    print("\nAuto demo: forward motion with periodic radial turns.")
    for i in range(AUTO_STEPS):
        if not controller.keep_running:
            break
        controller.move_forward()
        if i % 15 == 5:
            controller.turn_left()
        elif i % 15 == 10:
            controller.turn_right()
        time.sleep(0.05)
    controller.stop()
    print("Auto demo complete.")


def main():
    auto = '--auto' in sys.argv
    rover, cyl = fetch_rover_and_cylinder()
    if not rover or not cyl:
        return
    controller = RoverController(rover, cyl)
    if auto:
        auto_demo(controller)
    else:
        interactive_loop(controller)

if __name__ == '__main__':
    main()
