# Milestone 0: machine go/no-go

Measured 2026-11-26 on an RTX 5060 Laptop GPU (8 GB), Ryzen 7 250, 14 GiB RAM + 16 GiB swap, Ubuntu 24.04.5, NVIDIA driver 620.70.06.
Spike code: `spike/m0_spike.py` (scene + walk + cameras + fps), `spike/sample_mem.sh` (RAM/VRAM/swap sampler).

## Versions

- **Isaac Sim 6.0.1-rc.7**: source build at `~/isaacsim` (kit-kernel 110.1.2), symlinked as `~/IsaacLab/_isaac_sim`. Python 3.12, torch 2.10.0+cu128, warp 1.13.0.
- **Isaac Lab 3.0.0** (`v3.0.0-beta2`, commit `ffff603`) at `~/IsaacLab`, with `isaaclab_teleop` 0.5.2, `isaacteleop` 1.3.131, pinocchio 4.1.0, pink 3.1.0 (all already in the Isaac Sim python).
- We do not use Isaac Lab-Arena. Isaac Lab 3.0 already has the same G1 locomanipulation setup (`Isaac-PickPlace-Locomanipulation-G1-Abs-v0`).
- **Install:** nothing was installed. The machine already had both, built from source per the official docs: Isaac Sim via `./build.sh`, Isaac Lab via `./isaaclab.sh --install`. Assets (G1 USD, policy, IK URDF) are downloaded automatically at runtime from NVIDIA's asset server (`https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0`) into `/tmp/Assets/` (~174 MB for this scene).
- **Run:** `~/IsaacLab/isaaclab.sh -p <script> --headless --enable_cameras`.
- **Compatibility checker:** `~/isaacsim/_build/linux-x86_64/release/isaac-sim.compatibility_check.sh --no-window --/app/quitAfter=200` reports **FAILED**:
  - driver supported
  - GPU supported
  - VRAM 8.55 GB is "not enough" (minimum 10)
  - RAM 16 GB is "not enough" (minimum 32)
  - CPU 16 threads excellent
  - CPU governor is powersave (performance recommended)
  - disk 284 GB enough
  - Ubuntu 24.04 supported

  The real run below still works.
- **Gotcha:** on CUDA, the upstream `AgileBasedLowerBodyAction` passes a tensor that tracks gradients to warp and crashes with `Can't get __cuda_array_interface__ on Variable that requires grad`. Fix: call `torch.set_grad_enabled(False)` (or step inside `torch.no_grad()`). Upstream docs avoid it with `--device cpu`.

## G1 asset

- **Config:** `isaaclab_assets.robots.unitree.G1_29DOF_CFG`
  - USD: `{ISAAC_NUCLEUS_DIR}/Robots/Unitree/G1/g1.usd`
  - spawn pose: z = 0.75; rot (xyzw) = (0, 0, 0.7071, 0.7071), i.e. yawed +90°, so the robot faces world **+Y**
  - root link: `pelvis`
  - easiest base scene: `LocomanipulationG1EnvCfg` (minus table and object), see the spike.
- **Hand model:** Unitree G1 29-DoF + **Dex3-1 three-finger hands**, 7 DoF per hand (URDF name `g1_29dof_with_hand_rev_1_0`). 43 joints in total.
- **Left arm (7):** `left_shoulder_pitch_joint`, `left_shoulder_roll_joint`, `left_shoulder_yaw_joint`, `left_elbow_joint`, `left_wrist_roll_joint`, `left_wrist_pitch_joint`, `left_wrist_yaw_joint`
- **Right arm (7):** `right_shoulder_pitch_joint`, `right_shoulder_roll_joint`, `right_shoulder_yaw_joint`, `right_elbow_joint`, `right_wrist_roll_joint`, `right_wrist_pitch_joint`, `right_wrist_yaw_joint`
- **Waist (3):** `waist_yaw_joint`, `waist_roll_joint`, `waist_pitch_joint`. The upstream Pink IK action also drives these. The walking policy does not drive them, but it observes them.
- **Hand joints (7 per side, `<s>` = left/right):** `<s>_hand_thumb_0_joint`, `<s>_hand_thumb_1_joint`, `<s>_hand_thumb_2_joint`, `<s>_hand_index_0_joint`, `<s>_hand_index_1_joint`, `<s>_hand_middle_0_joint`, `<s>_hand_middle_1_joint`

  | joint | limits L / R (rad) | open (L, R) | closed L | closed R |
  |---|---|---|---|---|
  | thumb_0 | [-1.047, 1.047] / [-1.047, 1.047] | 0 | 0.0 | 0.0 |
  | thumb_1 | [-0.724, 1.047] / [-1.047, 0.724] | 0 | +0.4 | -0.4 |
  | thumb_2 | [0, 1.745] / [-1.745, 0] | 0 | +0.7 | -0.7 |
  | index_0 | [-1.571, 0] / [0, 1.571] | 0 | -1.0 | +1.0 |
  | index_1 | [-1.745, 0] / [0, 1.745] | 0 | -1.0 | +1.0 |
  | middle_0 | [-1.571, 0] / [0, 1.571] | 0 | -1.0 | +1.0 |
  | middle_1 | [-1.745, 0] / [0, 1.745] | 0 | -1.0 | +1.0 |

  - The open/closed values come from NVIDIA's `isaacteleop.retargeters.G1.TriHandMotionControllerRetargeter` (the Quest-controller mapping used by the G1 locomanip teleop) at trigger = squeeze = 0 (open) and trigger = squeeze = 1 (closed).
  - The left hand is the mirror (negation) of the right hand.
  - Trigger alone curls index and thumb (thumb_0 = −0.5·trigger on both sides). Squeeze alone curls middle and thumb (thumb_0 = +0.5·squeeze), so thumb_0 is 0 with both pressed.
  - All values are inside the joint limits above, which were read from the sim.
- **Head link:** there is no separate head body. The head is merged into **`torso_link`**, and the D435/head fixed links are merged too.
  - Spike cameras: prim `/World/envs/env_0/Robot/torso_link/cam_left|cam_right`
  - offset pos (0.06, ±0.032, 0.45) m in `torso_link`, rot (xyzw) (0.5, -0.5, 0.5, -0.5), convention `ros`
  - This gives a level forward view at about 1.2 m eye height. The rendered frame shows the ground grid and horizon, with nothing in the way.
  - The 0.45 m height and 0.06 m forward values are estimates from the frame, not measured against the head mesh. Fine-tune in later tasks.
- **Shoulder links:** `left_shoulder_pitch_link`, `right_shoulder_pitch_link` (first link of each arm chain; followed by `*_shoulder_roll_link`, `*_shoulder_yaw_link`).
- **End-effector links:** `left_wrist_yaw_link`, `right_wrist_yaw_link`. These are the IK targets upstream. The palms are `left_hand_palm_link` / `right_hand_palm_link`.
- **IK base:** `pelvis`. The IK URDF is `{ISAACLAB_NUCLEUS_DIR}/Controllers/LocomanipulationAssets/unitree_g1_kinematics_asset/g1_29dof_with_hand_only_kinematics.urdf`, where frame names carry the prefix `g1_29dof_with_hand_rev_1_0_`.
- **All bodies (sim order):** `pelvis`, then for each leg `hip_pitch`/`hip_roll`/`hip_yaw`/`knee`/`ankle_pitch`/`ankle_roll` `_link`, then `waist_yaw_link`, `waist_roll_link`, `torso_link`, then for each arm `shoulder_pitch`/`shoulder_roll`/`shoulder_yaw`/`elbow`/`wrist_roll`/`wrist_pitch`/`wrist_yaw` `_link`, then `*_hand_palm_link` and `*_hand_{index_0,middle_0,thumb_0,index_1,middle_1,thumb_1,thumb_2}_link`.

## Walking policy

- **File:** `{ISAACLAB_NUCLEUS_DIR}/Policies/Agile/agile_locomotion.pt`
  - TorchScript, 500 KB
  - resolves to `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/IsaacLab/Policies/Agile/agile_locomotion.pt`
  - cached by `retrieve_file_path` to `/tmp/Assets/...`
  - trained with NVIDIA's AGILE framework (github.com/nvidia-isaac/WBC-AGILE)
- **How to use:** reuse the upstream action term `AgileBasedLowerBodyActionCfg` (in `isaaclab_tasks/.../locomanipulation/pick_place/configs/action_cfg.py`) with the observation group `AgileTeacherPolicyObservationsCfg`:

  ```python
  AgileBasedLowerBodyActionCfg(asset_name="robot", joint_names=[".*_hip_.*_joint", ".*_knee_joint", ".*_ankle_.*_joint"],
                               policy_output_scale=0.25, obs_group_name="lower_body_policy", policy_path=...)
  ```
- **Command input (the action for this term), 4D:** `[vx, vy, wz, hip_height]`
  - vx, vy in m/s, wz in rad/s, robot base frame (+x forward, +y left)
  - hip_height in m, default **0.72** (the retargeter clamps it to 0.4–1.0)
  - The upstream Quest mapping (`LocomotionRootCmdRetargeter`):
    - vx = 0.5·leftStickY
    - vy = −0.5·leftStickX
    - wz = −1.0·rightStickX
    - right stick Y nudges hip_height by 0.35 m/s
- **Policy network input:** 83 values = command(4) + obs(79). The obs are:
  - base_lin_vel (3)
  - base_ang_vel (3)
  - projected_gravity (3)
  - joint_pos_rel (29)
  - joint_vel_rel×0.1 (29)
  - last lower-body action (12)

  The joints in the obs are arms + legs + waist (hands excluded). The action term builds this input itself.
- **Policy output:** 12 joint-position offsets, used as ×0.25 + default pose. Control rate is 50 Hz (sim dt 1/200, decimation 4).
- **Joints the policy controls (12):** `left/right_hip_pitch_joint`, `left/right_hip_roll_joint`, `left/right_hip_yaw_joint`, `left/right_knee_joint`, `left/right_ankle_pitch_joint`, `left/right_ankle_roll_joint`
  - The policy does not drive the arms, waist or hands, but it does observe arms and waist.
- **Verified:** the robot walks and stays up while the arms run Pink IK every step (wrists held fixed relative to the pelvis).
  - command (0.4, 0, 0, 0.72) for 10 s sim time
  - pelvis z min 0.719 m, no fall
  - moved 3.77 m along +Y (the robot's forward direction), so about 0.38 m/s against 0.4 commanded
  - then 30 s more walking, pelvis z 0.724 at the end
- **License / redistribution:**
  - The AGILE source code is Apache-2.0.
  - The `.pt` file and the G1 USD are NVIDIA-hosted Isaac Sim assets, covered by the *NVIDIA Isaac Sim Additional Software and Materials License* (linked from `~/isaacsim/LICENSE`), not by an open-source license.
  - **Do not commit or redistribute them. Users fetch them at runtime from NVIDIA's asset server** (automatic through `retrieve_file_path`, needs internet on first run).
  - We did not read the full terms of that license. "No redistribution" is the conservative ruling.

## Performance

Setup:
- `spike/m0_spike.py`: one G1, flat ground and dome light
- walking policy and Pink IK for both arms (the full 32D action) every step
- two RGB cameras, 640×480, HFOV 90° (focal 10, aperture 20)
- both images read to numpy (`.cpu().numpy()`) every step
- GPU device, headless

| run | stereo image fps (30 s average) | sim real-time factor | RAM peak | spike process RSS peak | swap | VRAM peak |
|---|---|---|---|---|---|---|
| default `render_interval=2` | **16.9** | 0.34 | 10554 MiB (10.3 GiB) used system-wide; baseline before launch 4006 MiB | 9251 MiB | 1.2 GiB already in swap before launch, no growth; swap-out 0 pages/s throughout | **4.10 GiB** (4195 MiB) |
| `render_interval=4` (1 render per control step) | **16.6** | 0.33 | 10683 MiB (10.4 GiB); baseline 4679 MiB | 9340 MiB | no growth, swap-out 0 | 4.10 GiB (4195 MiB) |

- RAM and VRAM were sampled once per second by `spike/sample_mem.sh` (`free -m`, `nvidia-smi`, `ps rss`, `vmstat` si/so) over the whole process lifetime, including startup.
- Without reading the images to numpy (the 10 s walk phase, cameras present), the env runs at about 27 steps/s (500 steps in 18.4 s).
- Reading back two 640×480 images costs about 22 ms per step. Rendering every 2 vs every 4 physics steps makes no difference.
- **The sim runs at about 1/3 real time.** The 50 Hz control loop actually runs at about 17 Hz, so the walk looks slow-motion in wall-clock time.
- An earlier run, which crashed on the grad bug above, peaked at 13573 MiB (13.3 GiB) system RAM with 3.3 GiB swap growth during first-time startup (asset download and shader cache build). Later runs, with a warm cache, used far less.

## Decision

**GO.**
- The stereo image rate is 16.6–16.9 fps, above the 15 fps bar.
- RAM peaked at about 10.4 of 14.9 GiB with zero swap-out, so it does not thrash.
- VRAM is 4.1 of 8 GiB.

Caveats for later tasks:
1. There is little headroom over 15 fps, and the sim runs at only about 0.33× real time. Do not add more scene content or render work without re-measuring.
2. Keep GPU→CPU image readback to once per frame. It is the biggest single cost.
3. Close Chrome and other memory-heavy apps before running. They are part of the 4 GiB baseline.
4. Every Isaac process needs `torch.set_grad_enabled(False)` on CUDA.
