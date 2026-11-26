"""The only place WebXR <-> Isaac conversion happens. Sim code uses Isaac frame, quats (w,x,y,z)."""
import numpy as np


def xr_pos_to_isaac(p):
    x, y, z = p  # WebXR: +x right, +y up, -z forward
    return np.array([-z, -x, y], dtype=float)  # Isaac: +x forward, +y left, +z up


def xr_quat_to_isaac(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array([w, *xr_pos_to_isaac([x, y, z])])  # axis maps like a vector (proper rotation)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def quat_from_yaw(yaw):
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


def yaw_of(q):
    w, x, y, z = q
    return float(np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)))


def quat_rotate(q, v):
    w, u = q[0], np.asarray(q[1:], dtype=float)
    v = np.asarray(v, dtype=float)
    t = 2 * np.cross(u, v)
    return v + w * t + np.cross(u, t)
