"""G1 teleop in Isaac Lab: stereo head cameras follow the headset, arms and hands follow the controllers.

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
from isaaclab.assets import RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.sensors import CameraCfg
from isaaclab_tasks.manager_based.locomanipulation.pick_place.locomanipulation_g1_env_cfg import (
    LocomanipulationG1EnvCfg,
)

from sim.camera_math import Recenter, eye_poses, smooth
from sim.control_math import HandTracker, grip_targets
from sim.coords import quat_from_yaw, quat_mul, quat_rotate, xr_pos_to_isaac, xr_quat_to_isaac, yaw_of
from sim.protocol import encode_jpeg
from sim.server import TeleopServer

HEAD_OFFSET = np.array([0.06, 0.0, 0.45])  # eye centre in torso_link, estimate from docs/milestone0.md
IDENTITY = {"p": [0, 0, 0], "q": [0, 0, 0, 1]}
# upstream 14-joint hand action order: (L,R) x {index_0, middle_0, thumb_0}, then the _1 joints, then thumb_2.
# Closed = NVIDIA TriHandMotionControllerRetargeter at trigger = squeeze = 1 (docs/milestone0.md): our one trigger
# closes the whole hand, so thumb_0 = 0.5*trigger - 0.5*squeeze = 0.
LEFT = np.array([1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0], bool)
OPEN = np.zeros(14)
CLOSED = np.array([-1, -1, 0, 1, 1, 0, -1, -1, 0.4, 1, 1, -0.4, 0.7, -0.7])
BOX_POS = (0.12, 0.27, 0.75)  # on the upstream packing table, in reach of the right hand (robot faces +y)


def to_wxyz(q_xyzw):
    return np.roll(q_xyzw, 1)  # Isaac Lab 3.0 data is (x,y,z,w); sim/coords math is (w,x,y,z)


cfg = LocomanipulationG1EnvCfg()
# keep the upstream table; swap its steering wheel for a 5 cm box; drop the task obs/terminations that read it
cfg.scene.object = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/Object",
    init_state=RigidObjectCfg.InitialStateCfg(pos=BOX_POS),
    spawn=sim_utils.CuboidCfg(
        size=(0.05, 0.05, 0.05), rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.1), collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.3, 0.1)),
    ),
)
# R4: IK moves only the 7 arm joints per side; the waist keeps its default target
cfg.actions.upper_body_ik.pink_controlled_joint_names = [
    n for n in cfg.actions.upper_body_ik.pink_controlled_joint_names if not n.startswith("waist")
]
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

server = TeleopServer()
server.start()  # R10: before the ~80 s scene build so a busy port fails fast
print("teleop_app serving ws://127.0.0.1:8765", flush=True)
env = ManagerBasedRLEnv(cfg)
try:
    # upstream AgileBasedLowerBodyAction passes a grad-tracking tensor to warp on CUDA; no_grad avoids it
    torch.set_grad_enabled(False)
    env.reset()
    robot = env.scene["robot"]
    cams = [env.scene["cam_left"], env.scene["cam_right"]]
    dev = env.device
    pelvis = robot.body_names.index("pelvis")
    torso = robot.body_names.index("torso_link")
    wrists = [robot.body_names.index(f"{s}_wrist_yaw_link") for s in ("left", "right")]
    shoulders = [robot.body_names.index(f"{s}_shoulder_pitch_link") for s in ("left", "right")]
    origin = env.scene.env_origins[0].cpu().numpy()

    # base frame = pelvis position + pelvis yaw; wrists start at, and keep the orientation of, the asset's default pose
    bp = robot.data.body_link_pose_w.torch[0].cpu().numpy()
    to_base = quat_from_yaw(-yaw_of(to_wxyz(bp[pelvis, 3:7])))
    trackers = [HandTracker(quat_rotate(to_base, bp[w, :3] - bp[pelvis, :3])) for w in wrists]
    wrist_q = [quat_mul(to_base, to_wxyz(bp[w, 3:7])) for w in wrists]
    grip = [0.0, 0.0]
    stand = torch.tensor([[0.0, 0.0, 0.0, 0.72]], device=dev)  # walking policy, zero velocity (it cannot stand unactuated)

    recenter = Recenter()
    head_pos = None
    seq, frames, t_log = 0, 0, time.monotonic()
    while app.is_running():
        msg = server.latest_input()
        if msg:
            head = {"p": list(msg.head.p), "q": list(msg.head.q)}
            head_rel = recenter.update(xr_quat_to_isaac(msg.head.q), bool(msg.right and msg.right.a))
            user_head, ctrl = xr_pos_to_isaac(msg.head.p), (msg.left, msg.right)
        else:
            head, head_rel = IDENTITY, np.array([1.0, 0, 0, 0])
            user_head, ctrl = None, (None, None)

        bp = robot.data.body_link_pose_w.torch[0].cpu().numpy()
        tq = to_wxyz(bp[torso, 3:7])
        raw = bp[torso, :3] + quat_rotate(tq, HEAD_OFFSET)
        head_pos = raw if head_pos is None else smooth(head_pos, raw, env.step_dt, tau=0.3)
        eyes = eye_poses(head_pos, yaw_of(tq), head_rel)
        for cam, (p, q) in zip(cams, eyes):
            cam.set_world_poses(p[None], np.roll(q, -1)[None], convention="world")

        pp, pyaw = bp[pelvis, :3], yaw_of(to_wxyz(bp[pelvis, 3:7]))
        to_base, to_world = quat_from_yaw(-pyaw), quat_from_yaw(pyaw)
        robot_head = quat_rotate(to_base, raw - pp)
        ee = []
        for tr, h, s, q in zip(trackers, ctrl, shoulders, wrist_q):
            t = tr.update(h, user_head, recenter.yaw0, robot_head, quat_rotate(to_base, bp[s, :3] - pp))
            ee += [pp - origin + quat_rotate(to_world, t), np.roll(quat_mul(to_world, q), -1)]
        grip = [h.trigger if h else g for h, g in zip(ctrl, grip)]  # keep the grip while tracking is lost
        ee.append(np.where(LEFT, grip_targets(grip[0], OPEN, CLOSED), grip_targets(grip[1], OPEN, CLOSED)))
        arms = torch.tensor(np.concatenate(ee)[None], dtype=torch.float32, device=dev)
        env.step(torch.cat([arms, stand], -1))

        left, right = torch.stack([c.data.output["rgb"][0] for c in cams]).cpu().numpy()  # one readback
        server.publish_frame({"seq": seq, "head": head}, encode_jpeg(left), encode_jpeg(right))
        seq += 1
        frames += 1
        now = time.monotonic()
        if now - t_log >= 5.0:
            print(f"fps {frames / (now - t_log):.1f} pelvis_z {bp[pelvis, 2]:.3f}", flush=True)
            frames, t_log = 0, now
except KeyboardInterrupt:
    pass
finally:
    server.stop()
    env.close()
    app.close()
