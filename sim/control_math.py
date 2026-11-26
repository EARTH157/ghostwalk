"""Arm and hand targets from the Quest controllers. Isaac frame; base = robot pelvis position + yaw."""
import numpy as np

from sim.coords import quat_from_yaw, quat_mul, quat_rotate, xr_pos_to_isaac, xr_quat_to_isaac

# NVIDIA's G1 locomanip Quest mapping (Se3AbsRetargeter: grip_q * offset, intrinsic XYZ (-135, 0, 90) right,
# (45, 180, -90) left -- the same rotation), moved into our frame through upstream's OXR->USD axis map: 45 deg about +y.
WRIST_OFFSET = np.array([np.cos(np.pi / 8), 0.0, np.sin(np.pi / 8), 0.0])


def hand_target_in_base(ctrl_p_isaac, head_p_isaac, head_yaw0, robot_head_in_base):
    return quat_rotate(quat_from_yaw(-head_yaw0), ctrl_p_isaac - head_p_isaac) + robot_head_in_base


def wrist_q_in_base(ctrl_q_isaac, head_yaw0, offset=WRIST_OFFSET):
    return quat_mul(quat_mul(quat_from_yaw(-head_yaw0), ctrl_q_isaac), offset)


def clamp_reach(target, shoulder, max_reach: float = 0.45):
    d = target - shoulder
    n = np.linalg.norm(d)
    return target if n <= max_reach else shoulder + d * (max_reach / n)


def grip_targets(trigger: float, open_q, closed_q):
    return open_q + trigger * (closed_q - open_q)


class HandTracker:
    """Holds the last wrist target (position, quat) while the controller is not tracked; starts at the asset's default."""

    def __init__(self, default_target, default_q):
        self.target, self.q = default_target, default_q

    def update(self, hand, head_p_isaac, head_yaw0, robot_head_in_base, shoulder):
        if hand is not None:
            t = hand_target_in_base(xr_pos_to_isaac(hand.pose.p), head_p_isaac, head_yaw0, robot_head_in_base)
            self.target = clamp_reach(t, shoulder)
            self.q = wrist_q_in_base(xr_quat_to_isaac(hand.pose.q), head_yaw0)
        return self.target, self.q
