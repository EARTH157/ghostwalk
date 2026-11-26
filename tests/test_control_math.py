import numpy as np

from sim.control_math import HandTracker, clamp_reach, grip_targets, hand_target_in_base
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
def test_lost_tracking_holds_last_target():
    tr = HandTracker(np.array([0.2, -0.2, 0.0]))
    assert np.allclose(tr.update(None, np.zeros(3), 0.0, np.zeros(3), np.zeros(3)), [0.2, -0.2, 0])  # ยังไม่มีเป้า → ท่าเริ่มต้น
    hand = Hand(Pose((0, 0, -0.3), (0, 0, 0, 1)), 0.0, (0, 0), False)  # WebXR -z = ข้างหน้า
    first = tr.update(hand, np.zeros(3), 0.0, np.zeros(3), np.zeros(3))
    assert np.allclose(first, [0.3, 0, 0])
    assert np.allclose(tr.update(None, np.zeros(3), 0.0, np.zeros(3), np.zeros(3)), first)
