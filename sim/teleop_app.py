"""G1 teleop in Isaac Lab: stereo head cameras that follow the headset, streamed over the teleop WebSocket.

Run: ~/IsaacLab/isaaclab.sh -p sim/teleop_app.py   (headless with cameras by default)
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(headless=True, enable_cameras=True)
args = parser.parse_args()
app = AppLauncher(args).app

import time

import numpy as np
import torch

import isaaclab.sim as sim_utils
import isaaclab.utils.math as mu
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.sensors import CameraCfg
from isaaclab_tasks.manager_based.locomanipulation.pick_place.locomanipulation_g1_env_cfg import (
    LocomanipulationG1EnvCfg,
)

from sim.camera_math import Recenter, eye_poses, smooth
from sim.coords import quat_rotate, xr_quat_to_isaac, yaw_of
from sim.protocol import encode_jpeg
from sim.server import TeleopServer

HEAD_OFFSET = np.array([0.06, 0.0, 0.45])  # eye centre in torso_link, estimate from docs/milestone0.md
IDENTITY = {"p": [0, 0, 0], "q": [0, 0, 0, 1]}


def to_wxyz(q_xyzw):
    return np.roll(q_xyzw, 1)  # Isaac Lab 3.0 data is (x,y,z,w); sim/coords math is (w,x,y,z)


cfg = LocomanipulationG1EnvCfg()
# strip the pick-place task (table, object and everything that reads them) and the episode timeout
cfg.scene.packing_table = None
cfg.scene.object = None
for t in ("object_pos", "object_rot", "object"):
    setattr(cfg.observations.policy, t, None)
for t in ("time_out", "object_dropping", "object_too_far", "success"):
    setattr(cfg.terminations, t, None)
# free cameras in the env, posed in world each step; focal 10 / aperture 20 -> HFOV 90 deg
for name in ("cam_left", "cam_right"):
    setattr(cfg.scene, name, CameraCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{name}", update_period=0.0, width=640, height=480, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=10.0, horizontal_aperture=20.0, clipping_range=(0.05, 100.0)),
    ))

env = ManagerBasedRLEnv(cfg)
# upstream AgileBasedLowerBodyAction passes a grad-tracking tensor to warp on CUDA; no_grad avoids it
torch.set_grad_enabled(False)
env.reset()
robot = env.scene["robot"]
cams = [env.scene["cam_left"], env.scene["cam_right"]]
dev = env.device
pelvis = robot.body_names.index("pelvis")
torso = robot.body_names.index("torso_link")
wrists = [robot.body_names.index(n) for n in ("left_wrist_yaw_link", "right_wrist_yaw_link")]
origin = env.scene.env_origins[0:1]


def pose(i):
    p = robot.data.body_link_pose_w.torch[:, i]
    return p[:, :3] - origin, p[:, 3:7]


# shortcut: arms hold their start pose relative to the pelvis; Task 6 drives them from the controllers
pp, pq = pose(pelvis)
rel = [mu.subtract_frame_transforms(pp, pq, *pose(w)) for w in wrists]
hands = torch.zeros(1, 14, device=dev)
stand = torch.tensor([[0.0, 0.0, 0.0, 0.72]], device=dev)  # walking policy, zero velocity (it cannot stand unactuated)

server = TeleopServer()
server.start()
print("teleop_app serving ws://127.0.0.1:8765", flush=True)
recenter = Recenter()
head_pos = None
seq, frames, t_log = 0, 0, time.monotonic()
try:
    while app.is_running():
        msg = server.latest_input()
        if msg:
            head = {"p": list(msg.head.p), "q": list(msg.head.q)}
            head_rel = recenter.update(xr_quat_to_isaac(msg.head.q), bool(msg.right and msg.right.a))
        else:
            head, head_rel = IDENTITY, np.array([1.0, 0, 0, 0])

        tp = robot.data.body_link_pose_w.torch[0, torso].cpu().numpy()
        tq = to_wxyz(tp[3:7])
        raw = tp[:3] + quat_rotate(tq, HEAD_OFFSET)
        head_pos = raw if head_pos is None else smooth(head_pos, raw, env.step_dt, tau=0.3)
        eyes = eye_poses(head_pos, yaw_of(tq), head_rel)
        for cam, (p, q) in zip(cams, eyes):
            cam.set_world_poses(p[None], np.roll(q, -1)[None], convention="world")

        pp, pq = pose(pelvis)
        ee = [torch.cat(mu.combine_frame_transforms(pp, pq, *r), -1) for r in rel]
        env.step(torch.cat(ee + [hands, stand], -1))

        left, right = torch.stack([c.data.output["rgb"][0] for c in cams]).cpu().numpy()  # one readback
        server.publish_frame({"seq": seq, "head": head}, encode_jpeg(left), encode_jpeg(right))
        seq += 1
        frames += 1
        now = time.monotonic()
        if now - t_log >= 5.0:
            print(f"fps {frames / (now - t_log):.1f}", flush=True)
            frames, t_log = 0, now
except KeyboardInterrupt:
    pass
finally:
    server.stop()
    env.close()
    app.close()
