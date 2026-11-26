"""Milestone-0 spike: G1 + AGILE lower-body policy walking, 2 head RGB cams 640x480 HFOV 90, measure fps.

Run: ~/IsaacLab/isaaclab.sh -p spike/m0_spike.py --headless --enable_cameras [--probe] [--render_interval 4]
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--probe", action="store_true", help="print joint/link names and exit")
parser.add_argument("--vx", type=float, default=0.4)
parser.add_argument("--walk_s", type=float, default=10.0)
parser.add_argument("--cam_s", type=float, default=30.0)
parser.add_argument("--render_interval", type=int, default=None, help="override cfg.sim.render_interval")
parser.add_argument("--head_link", default="torso_link")
parser.add_argument("--save_png", default=None, help="dir to save one stereo frame")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import os
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

cfg = LocomanipulationG1EnvCfg()
cfg.episode_length_s = 1000.0
# strip the pick-place task: table, object and everything that reads them
cfg.scene.packing_table = None
cfg.scene.object = None
for t in ("object_pos", "object_rot", "object"):
    setattr(cfg.observations.policy, t, None)
for t in ("object_dropping", "object_too_far", "success"):
    setattr(cfg.terminations, t, None)
if args.render_interval:
    cfg.sim.render_interval = args.render_interval
if not args.probe:
    # ROS convention (+Z fwd, -Y up) on a URDF link (+X fwd, +Z up); focal 10 / aperture 20 -> HFOV 90 deg
    for name, y in (("cam_left", 0.032), ("cam_right", -0.032)):
        setattr(cfg.scene, name, CameraCfg(
            prim_path=f"{{ENV_REGEX_NS}}/Robot/{args.head_link}/{name}",
            update_period=0.0, width=640, height=480, data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=10.0, horizontal_aperture=20.0, clipping_range=(0.05, 100.0)),
            offset=CameraCfg.OffsetCfg(pos=(0.06, y, 0.45), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
        ))

env = ManagerBasedRLEnv(cfg)
robot = env.scene["robot"]
print("ACTION_DIM", env.action_manager.total_action_dim, env.action_manager.active_terms)

if args.probe:
    print("BODIES", robot.body_names)
    print("JOINTS", robot.joint_names)
    lim = robot.data.joint_pos_limits.torch[0].cpu().numpy()
    for i, n in enumerate(robot.joint_names):
        print(f"LIMIT {n} {lim[i, 0]:+.3f} {lim[i, 1]:+.3f}")
    from pxr import Usd
    stage = env.scene.stage
    for p in Usd.PrimRange(stage.GetPrimAtPath("/World/envs/env_0/Robot")):
        if p.GetPath().pathElementCount <= 5:
            print("PRIM", p.GetPath(), p.GetTypeName())
    lb = env.action_manager.get_term("lower_body_joint_pos")
    print("POLICY_JOINTS", lb._joint_names)
    print("POLICY_OBS_DIM", env.observation_manager.group_obs_dim["lower_body_policy"])
    env.close()
    app.close()
    raise SystemExit

# upstream AgileBasedLowerBodyAction passes a grad-tracking tensor to warp on CUDA; no_grad avoids it
torch.set_grad_enabled(False)
env.reset()
dev = env.device
pelvis = robot.body_names.index("pelvis")
wrists = [robot.body_names.index(n) for n in ("left_wrist_yaw_link", "right_wrist_yaw_link")]
origin = env.scene.env_origins[0:1]


def pose(i):
    p = robot.data.body_link_pose_w.torch[:, i]
    return p[:, :3] - origin, p[:, 3:7]


# hold the wrists where they start, relative to the pelvis, so the IK runs every step like in teleop
pp, pq = pose(pelvis)
rel = [mu.subtract_frame_transforms(pp, pq, *pose(w)) for w in wrists]


def action(vx):
    pp, pq = pose(pelvis)
    ee = [torch.cat(mu.combine_frame_transforms(pp, pq, *r), -1) for r in rel]
    hands = torch.zeros(1, 14, device=dev)
    loco = torch.tensor([[vx, 0.0, 0.0, 0.72]], device=dev)
    return torch.cat(ee + [hands, loco], -1)


step_dt = env.step_dt
z_min = 9.9
start = robot.data.root_pos_w.torch[0].clone()
t0 = time.perf_counter()
for _ in range(int(args.walk_s / step_dt)):
    env.step(action(args.vx))
    z_min = min(z_min, robot.data.root_pos_w.torch[0, 2].item())
wall = time.perf_counter() - t0
d = (robot.data.root_pos_w.torch[0] - start).cpu().numpy()
print(f"WALK sim={args.walk_s}s wall={wall:.1f}s pelvis_z_min={z_min:.3f} displacement_xy=({d[0]:+.2f},{d[1]:+.2f}) "
      f"fell={z_min < 0.5}")

cams = [env.scene["cam_left"], env.scene["cam_right"]]
n = 0
t0 = time.perf_counter()
while time.perf_counter() - t0 < args.cam_s:
    env.step(action(args.vx))
    imgs = [c.data.output["rgb"][0].cpu().numpy() for c in cams]
    n += 1
wall = time.perf_counter() - t0
assert imgs[0].shape == (480, 640, 3) and imgs[0].dtype == np.uint8, (imgs[0].shape, imgs[0].dtype)
z = robot.data.root_pos_w.torch[0, 2].item()
print(f"CAM steps={n} wall={wall:.1f}s fps={n / wall:.1f} realtime_factor={n * step_dt / wall:.2f} "
      f"img={imgs[0].shape} mean_px=({imgs[0].mean():.0f},{imgs[1].mean():.0f}) pelvis_z={z:.3f}")
if args.save_png:
    from PIL import Image

    os.makedirs(args.save_png, exist_ok=True)
    Image.fromarray(np.hstack(imgs)).save(os.path.join(args.save_png, "stereo.png"))
env.close()
app.close()
