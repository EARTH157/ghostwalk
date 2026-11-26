import numpy as np

from sim.coords import quat_from_yaw, quat_rotate, xr_pos_to_isaac, xr_quat_to_isaac, yaw_of


def test_xr_forward_is_isaac_x():   assert np.allclose(xr_pos_to_isaac([0,0,-1]), [1,0,0])
def test_xr_up_is_isaac_z():        assert np.allclose(xr_pos_to_isaac([0,1,0]), [0,0,1])
def test_xr_right_is_isaac_minus_y(): assert np.allclose(xr_pos_to_isaac([1,0,0]), [0,-1,0])
def test_identity_quat():           assert np.allclose(xr_quat_to_isaac([0,0,0,1]), [1,0,0,0])
def test_xr_yaw_left_is_isaac_yaw_left():
    s = np.sin(np.pi/4); q_xr = [0, s, 0, s]          # +90° about XR +y (turn left)
    assert np.isclose(yaw_of(xr_quat_to_isaac(q_xr)), np.pi/2)
def test_rotate_forward_by_yaw():
    assert np.allclose(quat_rotate(quat_from_yaw(np.pi/2), [1,0,0]), [0,1,0])
