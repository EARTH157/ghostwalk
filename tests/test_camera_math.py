import numpy as np

from sim.camera_math import Recenter, eye_poses, smooth
from sim.coords import quat_from_yaw, yaw_of


def test_recenter_zeroes_initial_yaw():
    r = Recenter(); q = quat_from_yaw(1.0)
    assert np.isclose(yaw_of(r.update(q, False)), 0.0)
def test_recenter_on_button_edge_only():
    r = Recenter(); r.update(quat_from_yaw(0.0), False)
    r.update(quat_from_yaw(0.5), True)               # edge → yaw0 = 0.5
    assert np.isclose(yaw_of(r.update(quat_from_yaw(0.5), True)), 0.0)
def test_smooth_reaches_63_percent_at_tau():
    assert np.allclose(smooth(np.zeros(3), np.ones(3), dt=0.3, tau=0.3), 1 - np.exp(-1))
def test_eyes_separated_sideways_after_robot_yaw():
    (lp, _), (rp, _) = eye_poses(np.zeros(3), np.pi/2, np.array([1,0,0,0]))
    assert np.allclose(lp - rp, [-0.064, 0, 0])      # หุ่นหันซ้าย 90° → ตาซ้ายอยู่ทาง -x
