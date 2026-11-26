"""Arm and hand targets from the Quest controllers. Isaac frame; base = robot pelvis position + yaw."""
import numpy as np

from sim.coords import quat_from_yaw, quat_rotate, xr_pos_to_isaac


def hand_target_in_base(ctrl_p_isaac, head_p_isaac, head_yaw0, robot_head_in_base):
    return quat_rotate(quat_from_yaw(-head_yaw0), ctrl_p_isaac - head_p_isaac) + robot_head_in_base


def clamp_reach(target, shoulder, max_reach: float = 0.45):
    d = target - shoulder
    n = np.linalg.norm(d)
    return target if n <= max_reach else shoulder + d * (max_reach / n)


def grip_targets(trigger: float, open_q, closed_q):
    return open_q + trigger * (closed_q - open_q)


class HandTracker:
    """Holds the last hand target while the controller is not tracked; starts at the asset's default."""

    def __init__(self, default_target):
        self.target = default_target

    def update(self, hand, head_p_isaac, head_yaw0, robot_head_in_base, shoulder):
        if hand is not None:
            t = hand_target_in_base(xr_pos_to_isaac(hand.pose.p), head_p_isaac, head_yaw0, robot_head_in_base)
            self.target = clamp_reach(t, shoulder)
        return self.target
