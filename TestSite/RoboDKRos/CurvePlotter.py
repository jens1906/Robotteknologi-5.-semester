#!/usr/bin/env python3
from robodk.robolink import *
from robodk.robomath import *
import math
import numpy as np

def safe_normalize(v, eps=1e-9):
    """Normalize a vector safely and remove floating-point noise."""
    n = math.sqrt(sum(vi * vi for vi in v))
    if n < eps:
        return [0, 0, 1]
    v = [vi / n for vi in v]
    # Remove tiny floating-point residuals
    v = [0 if abs(vi) < 1e-10 else vi for vi in v]
    return v

def normalize(v):
    """Kept for compatibility, uses the safe version."""
    return safe_normalize(v)

def rot_about_axis(v, axis, angle_deg):
    """Rotate vector v about axis by angle_deg (Rodrigues' rotation)."""
    a = safe_normalize(axis)
    theta = math.radians(angle_deg)
    c, s = math.cos(theta), math.sin(theta)
    ax, ay, az = a
    R = np.array([
        [c + ax*ax*(1-c),   ax*ay*(1-c) - az*s, ax*az*(1-c) + ay*s],
        [ay*ax*(1-c) + az*s, c + ay*ay*(1-c),   ay*az*(1-c) - ax*s],
        [az*ax*(1-c) - ay*s, az*ay*(1-c) + ax*s, c + az*az*(1-c)]
    ])
    return (R @ np.array(v)).tolist()

def build_curve_with_normals(sx, sy, sz, L=200, S=40, N=6, rot_deg=0, tilt_deg=45):
    """Generate a serpentine curve with normal vectors for RoboDK."""
    pts = []
    for n in range(N):
        y = n * S
        seq = range(21) if n % 2 == 0 else reversed(range(21))
        for i in seq:
            x = L * i / 20.0
            pts.append((x, y, 0.0))
        if n < N - 1:
            ny, R = (n + 1) * S, S / 2.0
            for i in range(1, 19):
                a = math.radians(-90 + 180 * i / 18.0)
                x = (L if n % 2 == 0 else 0) + (-1 if n % 2 else 1) * R * math.cos(a)
                ya = (y + ny) / 2.0 + R * math.sin(a)
                pts.append((x, ya, 0.0))

    # Optional global XY rotation
    if rot_deg:
        a = math.radians(rot_deg)
        pts = [(px * math.cos(a) - py * math.sin(a),
                px * math.sin(a) + py * math.cos(a),
                pz) for (px, py, pz) in pts]

    positions = []
    normals = []

    for i, p in enumerate(pts):
        x, y, z = p
        if i < len(pts) - 1:
            nx, ny, nz = pts[i + 1]
            tangent = [nx - x, ny - y, nz - z]
        else:
            px, py, pz = pts[i - 1]
            tangent = [x - px, y - py, z - pz]

        tangent = safe_normalize(tangent)

        # World "down" vector
        down = [0, 0, -1]

        # Rotate the down vector about tangent by tilt_deg
        approach = rot_about_axis(down, tangent, tilt_deg)
        approach = safe_normalize(approach)

        # Ensure consistent hemisphere (avoid random -1 flips)
        if approach[2] < 0:
            approach = [-a for a in approach]

        positions.append([x + sx, y + sy, z + sz])
        normals.append(approach)

    pos_arr = np.array(positions).T
    norm_arr = np.array(normals).T
    mat6 = np.vstack((pos_arr, norm_arr))
    return Mat(mat6.tolist())

if __name__ == "__main__":
    RDK = Robolink()
    mat6 = build_curve_with_normals(0, 0, 0, L=100, S=20, N=4, rot_deg=0, tilt_deg=180)
    print(mat6)
    curve = RDK.AddCurve(mat6)
    curve.setName("Curve_with_correct_tilt")
    curve.setParent(RDK.Item("Frame 3"))
