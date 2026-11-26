"""Head-camera math for the stereo view. Isaac frame, quats (w,x,y,z)."""
import numpy as np

from sim.coords import quat_from_yaw, quat_mul, quat_rotate, yaw_of


class Recenter:
    """Removes the user's head yaw at the first update and on each rising edge of `pressed`."""

    def __init__(self):
        self.yaw0 = None
        self._was = False

    def update(self, head_q, pressed: bool) -> np.ndarray:
        if self.yaw0 is None or (pressed and not self._was):
            self.yaw0 = yaw_of(head_q)
        self._was = pressed
        return quat_mul(quat_from_yaw(-self.yaw0), head_q)


def smooth(prev, new, dt, tau=0.3):
    return prev + (1 - np.exp(-dt / tau)) * (new - prev)


def eye_poses(cam_pos, robot_yaw, head_rel_q, ipd=0.064):
    q = quat_mul(quat_from_yaw(robot_yaw), head_rel_q)
    return ((cam_pos + quat_rotate(q, [0, ipd / 2, 0]), q),
            (cam_pos + quat_rotate(q, [0, -ipd / 2, 0]), q))
