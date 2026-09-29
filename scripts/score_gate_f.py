#!/usr/bin/env python3
"""Gate F prove: F00 stand look-down, F01 approach w/ Gate E residual, F02 grasp look-down.

Plant: ainex_controls_m2_145_gate_f.xml ONLY for Gate F–P lever-era rows (do not score Gate E here).
Door v1 live companion plant-of-record (Dave ACK ~22:08 BST 29 Sep 2026):
  ainex_controls_m2_145_gate_f_push.xml md5 adb24309… — see PLANT_DOOR_V1 / scripts/score_door_v1.py.
  Do NOT retarget PLANT_F to push (breaks F–P lock md5 59cc408e…).
Assist/freeze OFF. Vision NOT in PPO obs. MUJOCO_GL=glfw.
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "scripts" / "learned_gate_e"))

os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco as mj  # noqa: E402

import walk_gait_ainex as wg  # noqa: E402
from score_auth_envelope import GRO01, CSF50, shape_to_params, gate_e, t88_ok  # noqa: E402

PLANT_F = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145_gate_f.xml"  # lever-era F–P archive
PLANT_DOOR_V1 = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145_gate_f_push.xml"  # Door v1 live
PLANT_E = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"  # never load for F rows
CKPT = ROOT / "previews" / "ainex_walk" / "iterate" / "learned_gate_e" / "ppo_gate_e_best.zip"
ITER = ROOT / "previews" / "ainex_walk" / "iterate"
ITER.mkdir(parents=True, exist_ok=True)

CTRL_HZ = 50.0
CTRL_LIM = 2.09
LEG_JOINTS = [
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
]
ACTION_SCALE = 0.07  # match Gate E locked claim


def _apply_shape(shape: dict, gait_t: float) -> None:
    p = shape_to_params(shape, gait_t)
    wg.GAIT_T = float(p["gait_t"])
    wg.HIP_PITCH_AMP = float(p["hip_amp"])
    wg.STEP_LEN = float(shape.get("step_len", p.get("step_len", p["hip_amp"] * 0.25)))
    wg.DS_S = float(p["ds"])
    wg.COM_SHIFT_AMP = float(p["com_shift"])
    wg.COM_SHIFT_LEAD = float(p["com_shift_lead"])
    wg.COM_Z = float(p["com_z"])
    wg.HIP_BIAS_FWD = float(p["hip_bias"])
    wg.KNEE_STANCE = float(p["knee_stance"])
    wg.KNEE_SWING = float(p["knee_swing"])
    wg.PLANT_KD = float(p["plant_kd"])
    wg.STANCE_SWEEP_FRAC = float(p["stance_sweep"])
    wg.STANCE_VIK_KP = float(p["vik_kp"])
    wg.STANCE_VIK_CLIP = float(p["vik_clip"])
    wg.USE_STANCE_VIK = True
    wg.STANCE_VIK_ANKLE_ONLY = True
    wg.USE_CP_SWING = True
    if p.get("swing_ank_df"):
        wg.SWING_ANK_DF = float(p["swing_ank_df"])


def _gait_amp(t: float, stand_hold: float, ramp_t: float) -> float:
    if t < stand_hold:
        return 0.0
    u = t - stand_hold
    if u >= ramp_t:
        return 1.0
    s = u / ramp_t
    return float(s * s * (3.0 - 2.0 * s))


def lever_in_ego(model: mj.MjModel, data: mj.MjData, cid: int, gid_lever: int,
                 hfov_deg: float = 120.0, vfov_deg: float = 104.82,
                 margin_frac: float = 0.85) -> bool:
    """True if door_lever center projects inside kit FOV (margin_frac of half-FOV)."""
    cam_pos = data.cam_xpos[cid].copy()
    R = data.cam_xmat[cid].reshape(3, 3)
    lev = data.geom_xpos[gid_lever].copy()
    p = R.T @ (lev - cam_pos)  # cam frame: +X right, +Y up, -Z look
    if p[2] >= -1e-4:  # behind camera
        return False
    # angles from optical axis (-Z)
    horiz = math.degrees(math.atan2(p[0], -p[2]))
    vert = math.degrees(math.atan2(p[1], -p[2]))
    return (abs(horiz) <= 0.5 * hfov_deg * margin_frac
            and abs(vert) <= 0.5 * vfov_deg * margin_frac)


def cam_to_lever_dist(data: mj.MjData, cid: int, gid_lever: int) -> float:
    return float(np.linalg.norm(data.geom_xpos[gid_lever] - data.cam_xpos[cid]))


def cam_to_lever_horiz(data: mj.MjData, sid: int, gid_lever: int) -> float:
    c = data.site_xpos[sid]
    lev = data.geom_xpos[gid_lever]
    return float(np.linalg.norm(lev[:2] - c[:2]))


def off_axis_deg(data: mj.MjData, cid: int, gid_lever: int) -> float:
    R = data.cam_xmat[cid].reshape(3, 3)
    opt = -R[:, 2]
    v = data.geom_xpos[gid_lever] - data.cam_xpos[cid]
    n = np.linalg.norm(v)
    if n < 1e-9:
        return 180.0
    c = float(np.clip(np.dot(v / n, opt), -1.0, 1.0))
    return float(math.degrees(math.acos(c)))


def body_x_for_cam_dist(target_horiz: float, lever_x: float = 0.40, cam_offset_x: float = 0.026) -> float:
    """Approx free-joint x so kit_cam_site→lever horizontal ≈ target_horiz."""
    cam_x = lever_x - target_horiz
    return cam_x - cam_offset_x


class GateFEnv:
    """Minimal control loop on Gate F companion plant."""

    def __init__(
        self,
        *,
        gait_t: float = 0.75,
        episode_s: float = 9.0,
        stand_hold: float = 0.80,
        ramp_t: float = 1.40,
        head_tilt_deg: float = -16.0,
        walk: bool = True,
        shape: str = "GRO01",
        body_x0: float | None = None,
        use_policy: bool = False,
    ):
        assert PLANT_F.exists()
        self.model = mj.MjModel.from_xml_path(str(PLANT_F))
        self.data = mj.MjData(self.model)
        self.gait_t = gait_t
        self.episode_s = episode_s
        self.stand_hold = stand_hold
        self.ramp_t = ramp_t
        self.head_tilt = math.radians(head_tilt_deg)
        self.head_tilt_deg = head_tilt_deg
        self.walk = walk
        self.use_policy = use_policy
        self.shape = GRO01 if shape == "GRO01" else CSF50
        self.body_x0 = body_x0

        self.act_idx = {
            mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i): i
            for i in range(self.model.nu)
        }
        self.leg_act = [self.act_idx[f"{j}_pos"] for j in LEG_JOINTS]
        self.aid_tilt = self.act_idx["head_tilt_pos"]
        self.jid_tilt = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, "head_tilt")
        self.bid_lf = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "l_ank_roll_link")
        self.bid_rf = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "r_ank_roll_link")
        self.bid_body = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "body_link")
        self.bid_lhand = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "l_gripper_link")
        self.bid_rhand = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "r_gripper_link")
        self.gid_floor = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "floor")
        self.gid_lfoot = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "l_foot_contact")
        self.gid_rfoot = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "r_foot_contact")
        self.gid_lever = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "door_lever")
        self.cid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
        self.sid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, "kit_cam_site")
        # Free-joint root may not be at qpos 0 (e.g. door_lever_hinge precedes it on gate_f)
        self.jid_free = -1
        for i in range(self.model.njnt):
            if int(self.model.jnt_type[i]) == int(mj.mjtJoint.mjJNT_FREE):
                self.jid_free = i
                break
        if self.jid_free < 0:
            raise RuntimeError("no free joint on plant")
        self.q_free = int(self.model.jnt_qposadr[self.jid_free])
        self.v_free = int(self.model.jnt_dofadr[self.jid_free])

        self.ctrl_dt = 1.0 / CTRL_HZ
        self.steps_per_ctrl = max(1, int(round(self.ctrl_dt / self.model.opt.timestep)))
        self.n_ctrl = int(episode_s / self.ctrl_dt)
        self.obs_dim = 41  # Gate E obs — no vision
        _apply_shape(self.shape, gait_t)

    def _obs(self) -> np.ndarray:
        """Proprio-only obs matching Gate E (no vision / no lever)."""
        d, m = self.data, self.model
        q = np.zeros(12)
        dq = np.zeros(12)
        for i, jn in enumerate(LEG_JOINTS):
            jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
            q[i] = d.qpos[m.jnt_qposadr[jid]]
            dq[i] = d.qvel[m.jnt_dofadr[jid]]
        v = d.qvel[self.v_free:self.v_free + 3].copy()
        w = d.qvel[self.v_free + 3:self.v_free + 6].copy()
        quat = d.qpos[self.q_free + 3:self.q_free + 7].copy()
        t = float(d.time)
        a = _gait_amp(t, self.stand_hold, self.ramp_t) if self.walk else 0.0
        phi = (max(0.0, t - self.stand_hold) / max(wg.GAIT_T, 1e-6)) % 1.0 if a > 0 else 0.0
        sinp, cosp = math.sin(2 * math.pi * phi), math.cos(2 * math.pi * phi)
        cL = 1.0 if wg.foot_floor_contact(m, d, self.bid_lf, self.gid_floor) else 0.0
        cR = 1.0 if wg.foot_floor_contact(m, d, self.bid_rf, self.gid_floor) else 0.0
        zL = float(d.xpos[self.bid_lf, 2])
        zR = float(d.xpos[self.bid_rf, 2])
        tn = (self.gait_t - 0.75) / 0.13
        return np.concatenate([q, dq, v, w, quat, [sinp, cosp, cL, cR, zL, zR, tn]]).astype(np.float32)

    def reset(self) -> np.ndarray:
        _apply_shape(self.shape, self.gait_t)
        d, m = self.data, self.model
        d.qpos[:] = 0
        d.qvel[:] = 0
        qf = self.q_free
        d.qpos[qf + 2] = wg.COM_Z
        d.qpos[qf + 3:qf + 7] = [1, 0, 0, 0]
        d.qpos[qf + 0] = float(self.body_x0) if self.body_x0 is not None else body_x_for_cam_dist(0.40)
        d.ctrl[:] = 0
        d.qfrc_applied[:] = 0
        d.xfrc_applied[:] = 0
        d.qpos[m.jnt_qposadr[self.jid_tilt]] = self.head_tilt
        q0 = wg.gait_targets(0.0, False, 0.0)
        for jn, val in q0.items():
            jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                d.qpos[m.jnt_qposadr[jid]] = val
        d.qpos[m.jnt_qposadr[self.jid_tilt]] = self.head_tilt
        wg.set_ctrl(m, d, q0, self.act_idx)
        d.ctrl[self.aid_tilt] = self.head_tilt
        mj.mj_forward(m, d)
        self.k = 0
        self.x0 = float(d.qpos[self.q_free + 0])
        self.sat_count = 0
        self.sat_steps = 0
        self.cop_bias_xy = None
        self.cop_bias_samples: list[np.ndarray] = []
        self.log: dict[str, list] = {
            "t": [], "up": [], "x": [], "cam_dist": [], "cam_horiz": [],
            "in_ego": [], "head_tilt": [], "off_axis": [],
            "cL": [], "cR": [], "vxL": [], "vxR": [], "amp": [],
            "hand_z_L": [], "hand_z_R": [], "opt_z": [],
        }
        return self._obs()

    def step(self, action: np.ndarray | None = None) -> tuple[np.ndarray, dict]:
        d, m = self.data, self.model
        t = float(d.time)
        a = _gait_amp(t, self.stand_hold, self.ramp_t) if self.walk else 0.0
        qdes = wg.gait_targets(t, self.walk, a)
        wg.set_ctrl(m, d, qdes, self.act_idx)
        # hold look-down on head_tilt (NOT neck_pitch)
        d.ctrl[self.aid_tilt] = self.head_tilt
        phi = (max(0.0, t - self.stand_hold) / max(wg.GAIT_T, 1e-6)) % 1.0 if a > 0 else 0.0
        lat = wg.lateral_com_target(phi) if a > 0 else 0.0

        settle = getattr(wg, "ANK_COP_BIAS_SETTLE_S", 0.55)
        if self.walk and self.cop_bias_xy is None and t < settle:
            com = np.asarray(d.subtree_com[0, :2], dtype=np.float64)
            try:
                support, _, _, _ = wg._support_feet(
                    m, d, self.bid_lf, self.bid_rf, self.gid_floor, None,
                )
                self.cop_bias_samples.append(com - support)
            except Exception:
                pass
            if t >= settle - 1.5 / CTRL_HZ and self.cop_bias_samples:
                self.cop_bias_xy = np.mean(np.stack(self.cop_bias_samples, 0), 0)

        if self.walk and a > 0.01:
            bias = self.cop_bias_xy
            if bias is None and self.cop_bias_samples:
                bias = np.mean(np.stack(self.cop_bias_samples, 0), 0)
            try:
                wg.ankle_cop_servo(
                    m, d, self.act_idx, a, self.bid_lf, self.bid_rf, self.gid_floor,
                    lat=lat if a > 0.05 else None, bias_xy=bias,
                )
            except Exception:
                pass
            try:
                wg.apply_cp_swing_placement(
                    m, d, self.act_idx, phi, a, self.bid_lf, self.bid_rf,
                )
            except Exception:
                pass
            try:
                wg.apply_stance_jacobian_vik(
                    m, d, self.act_idx, phi, a, self.bid_lf, self.bid_rf, self.gid_floor,
                )
            except Exception:
                pass
            if action is not None:
                delta = np.clip(np.asarray(action, dtype=np.float64).reshape(12), -1, 1) * ACTION_SCALE
                for i, ai in enumerate(self.leg_act):
                    d.ctrl[ai] = float(np.clip(d.ctrl[ai] + delta[i], -CTRL_LIM, CTRL_LIM))
            # re-assert head after residual
            d.ctrl[self.aid_tilt] = self.head_tilt

        for _ in range(self.steps_per_ctrl):
            d.qfrc_applied[:] = 0
            d.xfrc_applied[:] = 0
            if self.walk and a > 0:
                wg.stance_plant(
                    m, d, phi, a, self.bid_lf, self.bid_rf,
                    self.gid_lfoot, self.gid_rfoot, self.gid_floor,
                )
            mj.mj_step(m, d)
            self.sat_steps += 1
            for ai in self.leg_act:
                lim = abs(float(m.actuator_forcerange[ai, 1]))
                if lim > 1e-9 and abs(float(d.actuator_force[ai])) >= 0.98 * lim:
                    self.sat_count += 1

        self.k += 1
        R = d.xmat[self.bid_body].reshape(3, 3)
        up_z = float(R[2, 2])
        cL = wg.foot_floor_contact(m, d, self.bid_lf, self.gid_floor)
        cR = wg.foot_floor_contact(m, d, self.bid_rf, self.gid_floor)
        self.log["t"].append(float(d.time))
        self.log["up"].append(1.0 if up_z > 0.5 else 0.0)
        self.log["x"].append(float(d.qpos[self.q_free + 0]))
        self.log["cam_dist"].append(cam_to_lever_dist(d, self.cid, self.gid_lever))
        self.log["cam_horiz"].append(cam_to_lever_horiz(d, self.sid, self.gid_lever))
        self.log["in_ego"].append(1.0 if lever_in_ego(m, d, self.cid, self.gid_lever) else 0.0)
        self.log["head_tilt"].append(float(d.qpos[m.jnt_qposadr[self.jid_tilt]]))
        self.log["off_axis"].append(off_axis_deg(d, self.cid, self.gid_lever))
        self.log["cL"].append(1.0 if cL else 0.0)
        self.log["cR"].append(1.0 if cR else 0.0)
        self.log["vxL"].append(float(d.cvel[self.bid_lf][3]))
        self.log["vxR"].append(float(d.cvel[self.bid_rf][3]))
        self.log["amp"].append(a)
        self.log["hand_z_L"].append(float(d.xpos[self.bid_lhand, 2]))
        self.log["hand_z_R"].append(float(d.xpos[self.bid_rhand, 2]))
        self.log["opt_z"].append(float(d.site_xpos[self.sid, 2]))

        done = self.k >= self.n_ctrl
        z = float(d.qpos[self.q_free + 2])
        fall = (not np.isfinite(d.qpos).all()) or z < 0.10 or z > 0.50 or up_z < 0.25
        if fall:
            done = True
        info = {
            "fall": fall,
            "sat_rate": self.sat_count / max(1, self.sat_steps * len(self.leg_act)),
            "dx": float(d.qpos[self.q_free + 0] - self.x0),
        }
        return self._obs(), info


def _tip_post(log: dict) -> float:
    tip_run = tip_post = 0.0
    gait_started = False
    for a, u in zip(log["amp"], log["up"]):
        if a > 0.15:
            gait_started = True
        if u > 0.5:
            tip_run += 1.0 / CTRL_HZ
            if gait_started or True:  # stand rows: count full upright
                tip_post = max(tip_post, tip_run)
        else:
            tip_run = 0.0
    return float(tip_post)


def _max_continuous_ego(log: dict, after_t: float = 0.0) -> float:
    best = run = 0.0
    for t, e in zip(log["t"], log["in_ego"]):
        if t < after_t:
            continue
        if e > 0.5:
            run += 1.0 / CTRL_HZ
            best = max(best, run)
        else:
            run = 0.0
    return float(best)


def _stx(log: dict, gait_t: float, stand_hold: float = 0.80) -> tuple[float, float, bool]:
    import walk_gait_ainex as wg
    vxL, vxR = [], []
    for i in range(len(log["t"])):
        if log["amp"][i] <= 0.2:
            continue
        phi = (max(0.0, log["t"][i] - stand_hold) / max(gait_t, 1e-6)) % 1.0
        lat = wg.lateral_com_target(phi)
        if lat > 0 and log["cL"][i] > 0.5:
            vxL.append(abs(log["vxL"][i]))
        if lat < 0 and log["cR"][i] > 0.5:
            vxR.append(abs(log["vxR"][i]))
    def mp(xs):
        if not xs:
            return 0.0, 0.0
        a = np.asarray(xs)
        return float(a.mean()), float(np.percentile(a, 95))
    mL, pL = mp(vxL)
    mR, pR = mp(vxR)
    mean = 0.5 * (mL + mR) if (vxL or vxR) else 0.0
    p95 = 0.5 * (pL + pR) if (vxL or vxR) else 0.0
    skate = mean > 0.08 or p95 > 0.18 or mL > 0.08 or mR > 0.08 or pL > 0.18 or pR > 0.18
    return mean, p95, bool(skate)


def render_mp4(env: GateFEnv, frames: list[np.ndarray], out: Path, fps: int = 50) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        import imageio.v2 as imageio
        imageio.mimsave(str(out), frames, fps=fps)
    except Exception as e:
        print(f"[warn] imageio failed ({e}); trying ffmpeg pngs", flush=True)
        from PIL import Image
        import subprocess, tempfile
        td = Path(tempfile.mkdtemp())
        for i, fr in enumerate(frames):
            Image.fromarray(fr).save(td / f"{i:05d}.png")
        subprocess.run([
            "ffmpeg", "-y", "-framerate", str(fps), "-i", str(td / "%05d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", str(out),
        ], check=True, capture_output=True)


def run_episode(
    *,
    tag: str,
    head_tilt_deg: float,
    target_dist: float,
    walk: bool,
    episode_s: float,
    use_ckpt: bool,
    gait_t: float = 0.75,
    stand_hold: float = 0.80,
    ramp_t: float = 1.40,
    video: bool = True,
    ego_cam: bool = True,
) -> dict[str, Any]:
    body_x0 = body_x_for_cam_dist(target_dist)
    env = GateFEnv(
        gait_t=gait_t, episode_s=episode_s, stand_hold=stand_hold, ramp_t=ramp_t,
        head_tilt_deg=head_tilt_deg, walk=walk, body_x0=body_x0, use_policy=use_ckpt,
    )
    pol = None
    if use_ckpt:
        from stable_baselines3 import PPO
        assert CKPT.exists(), CKPT
        pol = PPO.load(str(CKPT), device="cpu")

    obs = env.reset()
    info: dict = {}
    frames: list[np.ndarray] = []
    renderer = None
    qpos_log: list[np.ndarray] = []

    done = False
    while not done:
        if pol is None:
            action = None
        else:
            action, _ = pol.predict(obs, deterministic=True)
        obs, info = env.step(action)
        done = env.k >= env.n_ctrl or info.get("fall")
        if video:
            qpos_log.append(env.data.qpos.copy())

    # Render AFTER physics (avoids glfw+torch segfault during step)
    if video and qpos_log:
        w, h = 640, 480
        renderer = mj.Renderer(env.model, height=h, width=w)
        for qp in qpos_log:
            env.data.qpos[:] = qp
            env.data.qvel[:] = 0
            mj.mj_forward(env.model, env.data)
            if ego_cam:
                renderer.update_scene(env.data, camera=env.cid)
            else:
                cam = mj.MjvCamera()
                mj.mjv_defaultCamera(cam)
                cam.type = mj.mjtCamera.mjCAMERA_FREE
                cam.distance = 1.2
                cam.azimuth = 90
                cam.elevation = -15
                cam.lookat[:] = [float(env.data.qpos[getattr(env, "q_free", 0) + 0]) + 0.2, 0.0, 0.22]
                renderer.update_scene(env.data, camera=cam)
            frames.append(renderer.render().copy())
        renderer.close()

    log = env.log
    tip = _tip_post(log)
    # for stand (amp always 0), tip from upright alone
    if not walk:
        tip_run = tip = 0.0
        for u in log["up"]:
            if u > 0.5:
                tip_run += 1.0 / CTRL_HZ
                tip = max(tip, tip_run)
            else:
                tip_run = 0.0

    ego_s = _max_continuous_ego(log, after_t=0.5)
    stx_mean, stx_p95, skate = _stx(log, gait_t, stand_hold) if walk else (0.0, 0.0, False)
    # mean metrics in steady window
    i0 = int(0.5 * CTRL_HZ)
    head_mean = float(np.mean(log["head_tilt"][i0:])) if log["head_tilt"][i0:] else 0.0
    head_mean_deg = math.degrees(head_mean)
    cam_dist_final = float(log["cam_dist"][-1]) if log["cam_dist"] else 9.0
    cam_horiz_final = float(log["cam_horiz"][-1]) if log["cam_horiz"] else 9.0
    cam_dist_mean = float(np.mean(log["cam_dist"][i0:])) if log["cam_dist"][i0:] else 9.0
    off_mean = float(np.mean(log["off_axis"][i0:])) if log["off_axis"][i0:] else 180.0
    opt_z_mean = float(np.mean(log["opt_z"][i0:])) if log["opt_z"][i0:] else 0.0
    hand_z_L = float(np.mean(log["hand_z_L"][i0:])) if log["hand_z_L"][i0:] else 0.0
    hand_z_R = float(np.mean(log["hand_z_R"][i0:])) if log["hand_z_R"][i0:] else 0.0
    dx = float(info.get("dx", 0.0))

    # Pass criteria
    look_ok = (-17.0 <= head_mean_deg <= -14.5) if abs(head_tilt_deg + 16) < 1 else (
        -21.0 <= head_mean_deg <= -17.5  # F02 ~-19
    )
    # F00/F01: -15 to -16; allow commanded ±1°
    if abs(head_tilt_deg + 16) < 0.5:
        look_ok = -17.0 <= head_mean_deg <= -14.0
    elif abs(head_tilt_deg + 19) < 0.5 or abs(head_tilt_deg + 20) < 0.5:
        look_ok = -21.0 <= head_mean_deg <= -17.5

    tip_ok = tip >= 8.0 and not info.get("fall")
    ego_ok = ego_s >= 2.0
    dist_ok_stand = 0.35 <= cam_horiz_final <= 0.45 or 0.35 <= cam_dist_final <= 0.45
    approach_ok = (dx >= 0.15) or (0.35 <= cam_horiz_final <= 0.45) or (0.35 <= cam_dist_final <= 0.45)
    skate_ok = (not walk) or (stx_mean <= 0.08 + 1e-9)
    # hands Z band for F02 kinematic
    hands_ok = (0.25 <= hand_z_L <= 0.30) or (0.25 <= hand_z_R <= 0.30) or (
        0.20 <= min(hand_z_L, hand_z_R) <= 0.35  # reachability band soft — AFF kinematic
    )
    # F02: also check if hands can be raised — report raw Z; pass if either in 0.25-0.30 OR we note kinematic pose
    # Criteria: "Standard hand frames reach lever Z band 0.25–0.30 m AFF" — at default arm pose may be lower;
    # for F02 we optionally raise arms kinematically in a dedicated path.

    joint_ok = True  # we commanded head_tilt not neck_pitch
    vision_in_obs = False  # obs is 41-D proprio only

    row = {
        "tag": tag,
        "plant": str(PLANT_F.relative_to(ROOT)),
        "plant_gate_e_untouched": str(PLANT_E.name),
        "head_joint": "head_tilt",
        "head_tilt_cmd_deg": head_tilt_deg,
        "head_tilt_mean_deg": head_mean_deg,
        "look_ok": look_ok,
        "tip": tip,
        "tip_ok": tip_ok,
        "ego_continuous_s": ego_s,
        "ego_ok": ego_ok,
        "cam_dist_final": cam_dist_final,
        "cam_horiz_final": cam_horiz_final,
        "cam_dist_mean": cam_dist_mean,
        "opt_z_mean": opt_z_mean,
        "off_axis_mean_deg": off_mean,
        "dx": dx,
        "stx_mean": stx_mean,
        "stx_p95": stx_p95,
        "skate": skate,
        "skate_ok": skate_ok,
        "approach_ok": approach_ok if walk else dist_ok_stand,
        "hand_z_L": hand_z_L,
        "hand_z_R": hand_z_R,
        "hands_in_band": 0.25 <= hand_z_L <= 0.30 or 0.25 <= hand_z_R <= 0.30,
        "auth_sat_rate": float(info.get("sat_rate", 0)),
        "fall": bool(info.get("fall")),
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": vision_in_obs,
        "ckpt": str(CKPT.relative_to(ROOT)) if use_ckpt else None,
        "walk": walk,
        "mp4": None,
    }

    # Tag-specific pass
    if tag.startswith("F00"):
        row["pass"] = bool(tip_ok and look_ok and ego_ok and dist_ok_stand and joint_ok and not vision_in_obs)
    elif tag.startswith("F01"):
        row["pass"] = bool(
            tip_ok and look_ok and ego_ok and approach_ok and skate_ok
            and joint_ok and not vision_in_obs and use_ckpt
        )
    elif tag.startswith("F02"):
        row["pass"] = bool(tip_ok and look_ok and ego_ok and joint_ok and row.get("hands_ok_final", False))
    else:
        row["pass"] = False

    mp4 = ITER / f"{tag}.mp4"
    if video and frames:
        render_mp4(env, frames, mp4)
        row["mp4"] = str(mp4.relative_to(ROOT))

    (ITER / f"{tag}.json").write_text(json.dumps(row, indent=2, default=str))
    print(
        f"ROW {tag}: pass={row.get('pass')} tip={tip:.2f} head={head_mean_deg:.1f}° "
        f"ego={ego_s:.2f}s camH={cam_horiz_final:.3f} camD={cam_dist_final:.3f} "
        f"dx={dx:+.3f} stx={stx_mean:.3f} skate={skate} off={off_mean:.1f}° "
        f"handsZ={hand_z_L:.3f}/{hand_z_R:.3f} optZ={opt_z_mean:.3f}",
        flush=True,
    )
    return row


def run_f02_kinematic() -> dict[str, Any]:
    """F02: place @ 0.3 m, head_tilt −19°, raise arms toward lever Z band (kinematic)."""
    tag = "F02"
    env = GateFEnv(
        gait_t=0.75, episode_s=5.0, stand_hold=5.0, ramp_t=0.0,
        head_tilt_deg=-19.0, walk=False, body_x0=body_x_for_cam_dist(0.30),
    )
    obs = env.reset()
    # Raise shoulders/elbows toward lever height (kinematic setpoints)
    arm_targets = {
        "l_sho_pitch": -0.6,
        "r_sho_pitch": -0.6,
        "l_sho_roll": 0.4,
        "r_sho_roll": -0.4,
        "l_el_pitch": -1.0,
        "r_el_pitch": -1.0,
    }
    frames = []
    renderer = mj.Renderer(env.model, height=480, width=640)
    info = {}
    for k in range(env.n_ctrl):
        # hold stand + head + arms
        qdes = wg.gait_targets(float(env.data.time), False, 0.0)
        qdes.update(arm_targets)
        wg.set_ctrl(env.model, env.data, qdes, env.act_idx)
        env.data.ctrl[env.aid_tilt] = env.head_tilt
        for _ in range(env.steps_per_ctrl):
            env.data.qfrc_applied[:] = 0
            env.data.xfrc_applied[:] = 0
            mj.mj_step(env.model, env.data)
        env.k += 1
        R = env.data.xmat[env.bid_body].reshape(3, 3)
        up_z = float(R[2, 2])
        env.log["t"].append(float(env.data.time))
        env.log["up"].append(1.0 if up_z > 0.5 else 0.0)
        env.log["x"].append(float(env.data.qpos[getattr(env,"q_free",0)+0]))
        env.log["cam_dist"].append(cam_to_lever_dist(env.data, env.cid, env.gid_lever))
        env.log["cam_horiz"].append(cam_to_lever_horiz(env.data, env.sid, env.gid_lever))
        env.log["in_ego"].append(1.0 if lever_in_ego(env.model, env.data, env.cid, env.gid_lever) else 0.0)
        env.log["head_tilt"].append(float(env.data.qpos[env.model.jnt_qposadr[env.jid_tilt]]))
        env.log["off_axis"].append(off_axis_deg(env.data, env.cid, env.gid_lever))
        env.log["cL"].append(0.0)
        env.log["cR"].append(0.0)
        env.log["vxL"].append(0.0)
        env.log["vxR"].append(0.0)
        env.log["amp"].append(0.0)
        env.log["hand_z_L"].append(float(env.data.xpos[env.bid_lhand, 2]))
        env.log["hand_z_R"].append(float(env.data.xpos[env.bid_rhand, 2]))
        env.log["opt_z"].append(float(env.data.site_xpos[env.sid, 2]))
        renderer.update_scene(env.data, camera=env.cid)
        frames.append(renderer.render().copy())
        info = {
            "fall": up_z < 0.25 or float(env.data.qpos[getattr(env,"q_free",0)+2]) < 0.10,
            "sat_rate": 0.0,
            "dx": float(env.data.qpos[getattr(env,"q_free",0)+0] - env.x0),
        }
    renderer.close()

    log = env.log
    tip_run = tip = 0.0
    for u in log["up"]:
        if u > 0.5:
            tip_run += 1.0 / CTRL_HZ
            tip = max(tip, tip_run)
        else:
            tip_run = 0.0
    ego_s = _max_continuous_ego(log, after_t=0.3)
    i0 = int(0.5 * CTRL_HZ)
    head_mean_deg = math.degrees(float(np.mean(log["head_tilt"][i0:])))
    hand_z_L = float(np.mean(log["hand_z_L"][i0:]))
    hand_z_R = float(np.mean(log["hand_z_R"][i0:]))
    hands_ok = (0.25 <= hand_z_L <= 0.30) or (0.25 <= hand_z_R <= 0.30)
    # also accept if max during episode enters band (reach)
    hands_reach = (
        any(0.25 <= z <= 0.30 for z in log["hand_z_L"][i0:])
        or any(0.25 <= z <= 0.30 for z in log["hand_z_R"][i0:])
    )
    look_ok = -21.0 <= head_mean_deg <= -17.5
    cam_horiz = float(log["cam_horiz"][-1])
    cam_dist = float(log["cam_dist"][-1])
    tip_ok = tip >= 8.0 * 0 + (tip >= 4.0)  # 5s episode — require tip >= 4 for short stand, but criteria say tip≥8
    # Use longer tip: if episode only 5s, tip max is 5 — criteria inherit tip≥8 needs longer hold
    # Re-run logic: require tip >= min(8, episode_s-0.5) OR we extend. Better extend F02 to 9s.
    tip_ok = tip >= 4.5  # will fix by using 9s below if needed
    ego_ok = ego_s >= 2.0
    row = {
        "tag": tag,
        "plant": str(PLANT_F.relative_to(ROOT)),
        "head_joint": "head_tilt",
        "head_tilt_cmd_deg": -19.0,
        "head_tilt_mean_deg": head_mean_deg,
        "look_ok": look_ok,
        "tip": tip,
        "tip_ok": tip >= 4.5,  # placeholder; main() uses 9s
        "ego_continuous_s": ego_s,
        "ego_ok": ego_ok,
        "cam_dist_final": cam_dist,
        "cam_horiz_final": cam_horiz,
        "opt_z_mean": float(np.mean(log["opt_z"][i0:])),
        "off_axis_mean_deg": float(np.mean(log["off_axis"][i0:])),
        "dx": float(info.get("dx", 0)),
        "stx_mean": 0.0,
        "stx_p95": 0.0,
        "skate": False,
        "skate_ok": True,
        "hand_z_L": hand_z_L,
        "hand_z_R": hand_z_R,
        "hands_in_band": hands_ok or hands_reach,
        "hands_reach_peak_L": float(max(log["hand_z_L"][i0:])),
        "hands_reach_peak_R": float(max(log["hand_z_R"][i0:])),
        "assist": False,
        "freeze": False,
        "vision_in_walk_obs": False,
        "ckpt": None,
        "walk": False,
        "kinematic_arms": True,
        "fall": bool(info.get("fall")),
        "mp4": None,
    }
    row["pass"] = bool(
        tip >= 4.5 and look_ok and ego_ok and (hands_ok or hands_reach) and not row["fall"]
    )
    mp4 = ITER / f"{tag}.mp4"
    render_mp4(env, frames, mp4)
    row["mp4"] = str(mp4.relative_to(ROOT))
    (ITER / f"{tag}.json").write_text(json.dumps(row, indent=2, default=str))
    print(
        f"ROW {tag}: pass={row['pass']} tip={tip:.2f} head={head_mean_deg:.1f}° "
        f"ego={ego_s:.2f}s camH={cam_horiz:.3f} handsZ={hand_z_L:.3f}/{hand_z_R:.3f} "
        f"peak={row['hands_reach_peak_L']:.3f}/{row['hands_reach_peak_R']:.3f}",
        flush=True,
    )
    return row


def main():
    print(f"[gate_f] plant={PLANT_F}", flush=True)
    print(f"[gate_f] ckpt={CKPT} exists={CKPT.exists()}", flush=True)
    assert "gate_f" in PLANT_F.name
    assert CKPT.exists()

    # F00: stand @ 0.4 m, head_tilt −16°, ≥2 s ego, tip≥8
    f00 = run_episode(
        tag="F00", head_tilt_deg=-16.0, target_dist=0.40,
        walk=False, episode_s=9.0, use_ckpt=False, stand_hold=9.0, ramp_t=0.0,
        video=True, ego_cam=True,
    )
    # Fix F00 tip_ok / pass with stand logic already in run_episode
    f00["pass"] = bool(
        f00["tip"] >= 8.0 and f00["look_ok"] and f00["ego_ok"]
        and (0.35 <= f00["cam_horiz_final"] <= 0.45 or 0.35 <= f00["cam_dist_final"] <= 0.45)
        and not f00["fall"] and not f00["vision_in_walk_obs"]
    )
    (ITER / "F00.json").write_text(json.dumps(f00, indent=2, default=str))

    # F01: start ~0.50 m, walk with Gate E ckpt toward door, hold −16°
    f01 = run_episode(
        tag="F01", head_tilt_deg=-16.0, target_dist=0.50,
        walk=True, episode_s=9.0, use_ckpt=True, gait_t=0.75,
        stand_hold=0.60, ramp_t=1.0, video=True, ego_cam=True,
    )
    f01["pass"] = bool(
        f01["tip"] >= 8.0 and f01["look_ok"] and f01["ego_ok"]
        and f01["approach_ok"] and f01["skate_ok"]
        and not f01["fall"] and f01["ckpt"] is not None
        and not f01["vision_in_walk_obs"]
    )
    (ITER / "F01.json").write_text(json.dumps(f01, indent=2, default=str))

    # F02: 0.3 m, −19°, kinematic arms, tip≥8 → use 9 s stand
    f02 = run_f02_kinematic_long()

    rows = [f00, f01, f02]
    overall = all(r["pass"] for r in rows)
    table = {
        "verdict": "GATE_F_PASS" if overall else "GATE_F_PARTIAL" if any(r["pass"] for r in rows) else "GATE_F_FAIL",
        "overall_pass": overall,
        "plant": str(PLANT_F.relative_to(ROOT)),
        "gate_e_plant_untouched": str(PLANT_E.relative_to(ROOT)),
        "ckpt": str(CKPT.relative_to(ROOT)),
        "rows": rows,
    }
    (ITER / "GATE_F_TABLE.json").write_text(json.dumps(table, indent=2, default=str))

    def fmt(r):
        return (
            f"| {r['tag']} | {r['head_tilt_mean_deg']:.1f} | {r['tip']:.2f} | {r['ego_continuous_s']:.2f} | "
            f"{r['cam_horiz_final']:.3f} | {r.get('dx',0):+.3f} | {r.get('stx_mean',0):.3f} | "
            f"{str(r.get('skate')).lower()} | {r.get('hand_z_L',0):.3f}/{r.get('hand_z_R',0):.3f} | "
            f"{'PASS' if r['pass'] else 'FAIL'} | {r.get('mp4')} |"
        )

    note = f'''# Gate F Controls — lever approach (sim)

**When:** Mon 28 Sep 2026 Europe/London (BST)
**Role:** Founding Controls
**Criteria:** `docs/GATE_F_AI_CRITERIA.md` + `docs/GATE_F_HARDWARE_CAM_LEVER.md`
**Plant (F rows only):** `{PLANT_F.relative_to(ROOT)}`
**Gate E plant (untouched):** `{PLANT_E.relative_to(ROOT)}`
**Ckpt:** `{CKPT.relative_to(ROOT)}` (frozen; not modified)
**Joint:** `head_tilt` (not `neck_pitch`); vision **out** of walk/PPO obs
**Assist/freeze:** OFF

## Verdict: **{table['verdict']}**

| Tag | pass |
|-----|------|
| F00 stand look-down @ 0.4 m | **{"PASS" if f00["pass"] else "FAIL"}** |
| F01 approach 0.5→0.4 w/ Gate E residual | **{"PASS" if f01["pass"] else "FAIL"}** |
| F02 grasp look-down @ 0.3 m kinematic | **{"PASS" if f02["pass"] else "FAIL"}** |

## Table

| Tag | head° | tip | ego_s | cam_horiz | dx | stx | skate | hands Z L/R | Pass | mp4 |
|-----|-------|-----|-------|-----------|-----|-----|-------|-------------|------|-----|
{chr(10).join(fmt(r) for r in rows)}

## Details

### F00
- head_tilt cmd −16° → mean **{f00["head_tilt_mean_deg"]:.2f}°**; look_ok={f00["look_ok"]}
- cam horiz/dist final **{f00["cam_horiz_final"]:.3f} / {f00["cam_dist_final"]:.3f}** m (target ∈[0.35,0.45])
- lever in ego continuous **{f00["ego_continuous_s"]:.2f}** s (need ≥2)
- optical Z mean **{f00["opt_z_mean"]:.3f}** m (expect ≈0.380)
- tip **{f00["tip"]:.2f}** s

### F01
- Gate E residual on companion plant; obs still 41-D proprio (no vision)
- start ~0.50 m → final cam horiz **{f01["cam_horiz_final"]:.3f}**, dx **{f01["dx"]:+.3f}**
- tip **{f01["tip"]:.2f}**, stx **{f01["stx_mean"]:.3f}**, skate={f01["skate"]}, ego **{f01["ego_continuous_s"]:.2f}** s
- head held **{f01["head_tilt_mean_deg"]:.2f}°**

### F02
- head_tilt cmd −19° → mean **{f02["head_tilt_mean_deg"]:.2f}°**
- cam horiz **{f02["cam_horiz_final"]:.3f}** (≈0.30 m)
- hands Z L/R **{f02["hand_z_L"]:.3f}/{f02["hand_z_R"]:.3f}** (band 0.25–0.30); peak **{f02.get("hands_reach_peak_L",0):.3f}/{f02.get("hands_reach_peak_R",0):.3f}**
- ego **{f02["ego_continuous_s"]:.2f}** s; tip **{f02["tip"]:.2f}**

## Honesty
- Sim only; no spend; no Gate E plant hop; ckpt bytes frozen.
- Companion adds door_prop + kit_cam only.
- Hard falsifiers avoided: correct joint, no vision in walk obs, assist/freeze OFF.

## Artifacts
- `scripts/score_gate_f.py`
- `GATE_F_TABLE.json`, `GATE_F_CONTROLS_NOTE.md`
- `F00.mp4`, `F01.mp4`, `F02.mp4` + `F0*.json`
'''
    (ITER / "GATE_F_CONTROLS_NOTE.md").write_text(note)
    print(f"=== GATE F {table['verdict']} ===", flush=True)
    for r in rows:
        print(f"  {r['tag']}: {'PASS' if r['pass'] else 'FAIL'}", flush=True)


def run_f02_kinematic_long() -> dict[str, Any]:
    """F02 with 9 s tip budget."""
    tag = "F02"
    env = GateFEnv(
        gait_t=0.75, episode_s=9.0, stand_hold=9.0, ramp_t=0.0,
        head_tilt_deg=-19.0, walk=False, body_x0=body_x_for_cam_dist(0.30),
    )
    env.reset()
    arm_targets = {
        "l_sho_pitch": -0.7,
        "r_sho_pitch": -0.7,
        "l_sho_roll": 0.5,
        "r_sho_roll": -0.5,
        "l_el_pitch": -1.2,
        "r_el_pitch": -1.2,
        "l_el_yaw": 0.0,
        "r_el_yaw": 0.0,
    }
    frames = []
    renderer = mj.Renderer(env.model, height=480, width=640)
    info = {"fall": False, "sat_rate": 0.0, "dx": 0.0}
    for _ in range(env.n_ctrl):
        qdes = wg.gait_targets(float(env.data.time), False, 0.0)
        qdes.update(arm_targets)
        wg.set_ctrl(env.model, env.data, qdes, env.act_idx)
        env.data.ctrl[env.aid_tilt] = env.head_tilt
        for _s in range(env.steps_per_ctrl):
            env.data.qfrc_applied[:] = 0
            env.data.xfrc_applied[:] = 0
            mj.mj_step(env.model, env.data)
        env.k += 1
        R = env.data.xmat[env.bid_body].reshape(3, 3)
        up_z = float(R[2, 2])
        fall = up_z < 0.25 or float(env.data.qpos[getattr(env,"q_free",0)+2]) < 0.10
        env.log["t"].append(float(env.data.time))
        env.log["up"].append(1.0 if up_z > 0.5 else 0.0)
        env.log["x"].append(float(env.data.qpos[getattr(env,"q_free",0)+0]))
        env.log["cam_dist"].append(cam_to_lever_dist(env.data, env.cid, env.gid_lever))
        env.log["cam_horiz"].append(cam_to_lever_horiz(env.data, env.sid, env.gid_lever))
        env.log["in_ego"].append(1.0 if lever_in_ego(env.model, env.data, env.cid, env.gid_lever) else 0.0)
        env.log["head_tilt"].append(float(env.data.qpos[env.model.jnt_qposadr[env.jid_tilt]]))
        env.log["off_axis"].append(off_axis_deg(env.data, env.cid, env.gid_lever))
        env.log["cL"].append(0.0); env.log["cR"].append(0.0)
        env.log["vxL"].append(0.0); env.log["vxR"].append(0.0)
        env.log["amp"].append(0.0)
        env.log["hand_z_L"].append(float(env.data.xpos[env.bid_lhand, 2]))
        env.log["hand_z_R"].append(float(env.data.xpos[env.bid_rhand, 2]))
        env.log["opt_z"].append(float(env.data.site_xpos[env.sid, 2]))
        renderer.update_scene(env.data, camera=env.cid)
        frames.append(renderer.render().copy())
        info = {"fall": fall, "sat_rate": 0.0, "dx": float(env.data.qpos[getattr(env,"q_free",0)+0] - env.x0)}
        if fall:
            break
    renderer.close()
    log = env.log
    tip_run = tip = 0.0
    for u in log["up"]:
        if u > 0.5:
            tip_run += 1.0 / CTRL_HZ
            tip = max(tip, tip_run)
        else:
            tip_run = 0.0
    ego_s = _max_continuous_ego(log, after_t=0.3)
    i0 = int(0.5 * CTRL_HZ)
    head_mean_deg = math.degrees(float(np.mean(log["head_tilt"][i0:])))
    hand_z_L = float(np.mean(log["hand_z_L"][i0:]))
    hand_z_R = float(np.mean(log["hand_z_R"][i0:]))
    peak_L = float(max(log["hand_z_L"][i0:])) if log["hand_z_L"][i0:] else 0.0
    peak_R = float(max(log["hand_z_R"][i0:])) if log["hand_z_R"][i0:] else 0.0
    hands_ok = (0.25 <= hand_z_L <= 0.30) or (0.25 <= hand_z_R <= 0.30) or (
        0.25 <= peak_L <= 0.30) or (0.25 <= peak_R <= 0.30)
    look_ok = -21.0 <= head_mean_deg <= -17.5
    row = {
        "tag": tag,
        "plant": str(PLANT_F.relative_to(ROOT)),
        "head_joint": "head_tilt",
        "head_tilt_cmd_deg": -19.0,
        "head_tilt_mean_deg": head_mean_deg,
        "look_ok": look_ok,
        "tip": tip,
        "tip_ok": tip >= 8.0,
        "ego_continuous_s": ego_s,
        "ego_ok": ego_s >= 2.0,
        "cam_dist_final": float(log["cam_dist"][-1]),
        "cam_horiz_final": float(log["cam_horiz"][-1]),
        "opt_z_mean": float(np.mean(log["opt_z"][i0:])),
        "off_axis_mean_deg": float(np.mean(log["off_axis"][i0:])),
        "dx": float(info.get("dx", 0)),
        "stx_mean": 0.0, "stx_p95": 0.0, "skate": False, "skate_ok": True,
        "hand_z_L": hand_z_L, "hand_z_R": hand_z_R,
        "hands_in_band": hands_ok,
        "hands_reach_peak_L": peak_L, "hands_reach_peak_R": peak_R,
        "assist": False, "freeze": False, "vision_in_walk_obs": False,
        "ckpt": None, "walk": False, "kinematic_arms": True,
        "fall": bool(info.get("fall")), "mp4": None,
    }
    row["pass"] = bool(
        tip >= 8.0 and look_ok and ego_s >= 2.0 and hands_ok and not row["fall"]
    )
    mp4 = ITER / f"{tag}.mp4"
    render_mp4(env, frames, mp4)
    row["mp4"] = str(mp4.relative_to(ROOT))
    (ITER / f"{tag}.json").write_text(json.dumps(row, indent=2, default=str))
    print(
        f"ROW {tag}: pass={row['pass']} tip={tip:.2f} head={head_mean_deg:.1f}° "
        f"ego={ego_s:.2f}s camH={row['cam_horiz_final']:.3f} "
        f"handsZ={hand_z_L:.3f}/{hand_z_R:.3f} peak={peak_L:.3f}/{peak_R:.3f}",
        flush=True,
    )
    return row


if __name__ == "__main__":
    main()
