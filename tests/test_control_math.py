import numpy as np
import pytest

from sim.control_math import WRIST_OFFSET, HandTracker, clamp_reach, grip_targets, hand_target_in_base, stick_to_velocity, wrist_q_in_base
from sim.coords import quat_from_yaw
from sim.protocol import Hand, Pose


def test_hand_target_maps_forward_reach():
    t = hand_target_in_base(np.array([0.4,0,1.2]), np.array([0,0,1.6]), 0.0, np.array([0,0,1.2]))
    assert np.allclose(t, [0.4, 0, 0.8])
def test_hand_target_removes_user_yaw():
    t = hand_target_in_base(np.array([0,0.4,1.6]), np.array([0,0,1.6]), np.pi/2, np.zeros(3))
    assert np.allclose(t, [0.4, 0, 0])               # ผู้ใช้หันซ้าย 90° แล้วยื่นมือไปข้างหน้า → หน้าหุ่น
def test_clamp_reach_limits_distance():
    t = clamp_reach(np.array([2.0,0,0]), np.zeros(3), 0.45); assert np.isclose(np.linalg.norm(t), 0.45)
def test_clamp_reach_keeps_near_target():
    assert np.allclose(clamp_reach(np.array([0.1,0,0]), np.zeros(3)), [0.1,0,0])
def test_grip_half():
    assert np.allclose(grip_targets(0.5, np.zeros(2), np.ones(2)), [0.5,0.5])
def test_wrist_q_identity_controller_gives_offset():
    assert np.allclose(wrist_q_in_base(np.array([1.0, 0, 0, 0]), 0.0), WRIST_OFFSET)
def test_wrist_q_removes_user_yaw():
    assert np.allclose(wrist_q_in_base(quat_from_yaw(0.7), 0.7), WRIST_OFFSET)  # ผู้ใช้หันไป 0.7 rad ตอน recenter → เหมือนหันตรง
def test_lost_tracking_holds_last_target():
    q0 = quat_from_yaw(0.3)
    tr = HandTracker(np.array([0.2, -0.2, 0.0]), q0)
    p, q = tr.update(None, np.zeros(3), 0.0, np.zeros(3), np.zeros(3))  # ยังไม่มีเป้า → ท่าเริ่มต้น
    assert np.allclose(p, [0.2, -0.2, 0]) and np.allclose(q, q0)
    hand = Hand(Pose((0, 0, -0.3), (0, 0, 0, 1)), 0.0, (0, 0), False)  # WebXR -z = ข้างหน้า
    p, q = tr.update(hand, np.zeros(3), 0.0, np.zeros(3), np.zeros(3))
    assert np.allclose(p, [0.3, 0, 0]) and np.allclose(q, WRIST_OFFSET)
    p2, q2 = tr.update(None, np.zeros(3), 0.0, np.zeros(3), np.zeros(3))
    assert np.allclose(p2, p) and np.allclose(q2, q)
def test_push_forward_full():       assert stick_to_velocity((0,-1),(0,0)) == pytest.approx((0.6,0,0))
def test_strafe_right_is_minus_y(): assert stick_to_velocity((1,0),(0,0)) == pytest.approx((0,-0.3,0))
def test_turn_right_is_minus_wz():  assert stick_to_velocity((0,0),(1,0)) == pytest.approx((0,0,-1.0))
def test_drift_inside_deadzone_is_zero(): assert stick_to_velocity((0.1,-0.14),(0.12,0)) == (0,0,0)
def test_just_outside_deadzone_is_small():
    assert 0 < stick_to_velocity((0,-0.2),(0,0))[0] < 0.05
