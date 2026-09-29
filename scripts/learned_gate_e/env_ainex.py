"""Gymnasium AiNex env: open-loop CPG/VIK/CP/plant base + MLP residual under kit HX."""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

import gymnasium as gym
import mujoco as mj
import numpy as np
from gymnasium import spaces

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import walk_gait_ainex as wg  # noqa: E402
from score_auth_envelope import GRO01, CSF50, shape_to_params  # noqa: E402

PLANT = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
CTRL_LIM = 2.09  # kit position

LEG_JOINTS = [
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
]


def _apply_shape(shape: dict, gait_t: float) -> None:
    p = shape_to_params(shape, gait_t)
    wg.GAIT_T = float(p["gait_t"])
    wg.HIP_PITCH_AMP = float(p["hip_amp"])
    # prefer explicit step_len from seed dict
    step = shape.get("step_len", p.get("step_len", p["hip_amp"] * 0.25))
    wg.STEP_LEN = float(step)
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
    wg.USE_CAPTURE_STEP = False
    wg.USE_HIP_STRAT = False
    if p.get("swing_ank_df"):
        wg.SWING_ANK_DF = float(p["swing_ank_df"])


class AinexResidualEnv(gym.Env):
    """50 Hz control; action = Δctrl on 12 leg joints, clipped to kit HX pos.

    Open-loop stack mirrors AUTH00: gait_targets + ankle CoP + CP swing +
    stance VIK ankle-only + stance plant damper. Assist/freeze/xfrc OFF.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        *,
        shape: str = "GRO01",
        gait_t: float = 0.88,
        episode_s: float = 6.0,
        stand_hold: float = 0.80,
        ramp_t: float = 1.40,
        action_scale: float = 0.07,
        use_vik: bool = True,
        use_plant: bool = True,
        use_cp: bool = True,
        seed: int | None = None,
    ):
        super().__init__()
        self.shape_name = shape
        self.shape = GRO01 if shape == "GRO01" else CSF50
        self.gait_t = float(gait_t)
        self.episode_s = float(episode_s)
        self.stand_hold = float(stand_hold)
        self.ramp_t = float(ramp_t)
        self.action_scale = float(action_scale)
        self.use_vik = bool(use_vik)
        self.use_plant = bool(use_plant)
        self.use_cp = bool(use_cp)

        self.model = mj.MjModel.from_xml_path(str(PLANT))
        self.data = mj.MjData(self.model)
        self.act_idx = {
            mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i): i
            for i in range(self.model.nu)
        }
        self.leg_act = [self.act_idx[f"{j}_pos"] for j in LEG_JOINTS]
        self.bid_lf = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "l_ank_roll_link")
        self.bid_rf = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "r_ank_roll_link")
        self.bid_body = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "body_link")
        self.gid_floor = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "floor")
        self.gid_lfoot = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "l_foot_contact")
        self.gid_rfoot = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, "r_foot_contact")

        self.ctrl_dt = 1.0 / wg.CTRL_HZ
        self.steps_per_ctrl = max(1, int(round(self.ctrl_dt / self.model.opt.timestep)))
        self.n_ctrl = int(self.episode_s / self.ctrl_dt)

        # obs: q(12), dq(12), v(3), w(3), quat(4), phase(2), contact(2), foot_z(2), T_norm(1) = 41
        self.obs_dim = 41
        self.observation_space = spaces.Box(-np.inf, np.inf, (self.obs_dim,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (12,), np.float32)

        self._apply_shape()
        self.k = 0
        self.x0 = 0.0
        self.sat_count = 0
        self.sat_steps = 0
        self.prev_com_xy = None
        self.cop_bias_xy = None
        self.cop_bias_samples: list[np.ndarray] = []
        self._rng = np.random.default_rng(seed)
        self.log: dict[str, list] = {}
        self._hip_buf_L: list[float] = []
        self._hip_buf_R: list[float] = []

    def set_gait_t(self, gait_t: float) -> None:
        self.gait_t = float(gait_t)
        self._apply_shape()

    def _apply_shape(self) -> None:
        _apply_shape(self.shape, self.gait_t)

    def _gait_amp(self, t: float) -> float:
        if t < self.stand_hold:
            return 0.0
        u = t - self.stand_hold
        if u >= self.ramp_t:
            return 1.0
        s = u / self.ramp_t
        return float(s * s * (3.0 - 2.0 * s))

    def _contact(self, bid: int) -> bool:
        return wg.foot_floor_contact(self.model, self.data, bid, self.gid_floor)

    def _obs(self) -> np.ndarray:
        d, m = self.data, self.model
        q = np.zeros(12, dtype=np.float64)
        dq = np.zeros(12, dtype=np.float64)
        for i, jn in enumerate(LEG_JOINTS):
            jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
            q[i] = d.qpos[m.jnt_qposadr[jid]]
            dq[i] = d.qvel[m.jnt_dofadr[jid]]
        v = d.qvel[0:3].copy()
        w = d.qvel[3:6].copy()
        quat = d.qpos[3:7].copy()
        t = float(d.time)
        a = self._gait_amp(t)
        phi = (max(0.0, t - self.stand_hold) / max(wg.GAIT_T, 1e-6)) % 1.0 if a > 0 else 0.0
        sinp, cosp = math.sin(2 * math.pi * phi), math.cos(2 * math.pi * phi)
        cL = 1.0 if self._contact(self.bid_lf) else 0.0
        cR = 1.0 if self._contact(self.bid_rf) else 0.0
        zL = float(d.xpos[self.bid_lf, 2])
        zR = float(d.xpos[self.bid_rf, 2])
        tn = (self.gait_t - 0.75) / 0.13
        return np.concatenate([
            q, dq, v, w, quat,
            [sinp, cosp, cL, cR, zL, zR, tn],
        ]).astype(np.float32)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self._apply_shape()
        d, m = self.data, self.model
        d.qpos[:] = 0
        d.qpos[2] = wg.COM_Z
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qpos[0] = -0.12
        d.qvel[:] = 0
        d.ctrl[:] = 0
        d.qfrc_applied[:] = 0
        d.xfrc_applied[:] = 0
        q0 = wg.gait_targets(0.0, False, 0.0)
        for jn, val in q0.items():
            jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                d.qpos[m.jnt_qposadr[jid]] = val
        wg.set_ctrl(m, d, q0, self.act_idx)
        mj.mj_forward(m, d)
        self.k = 0
        self.x0 = float(d.qpos[0])
        self.sat_count = 0
        self.sat_steps = 0
        self.prev_com_xy = d.subtree_com[0, :2].copy()
        self.cop_bias_xy = None
        self.cop_bias_samples = []
        self._hip_buf_L: list[float] = []
        self._hip_buf_R: list[float] = []
        self.log = {
            "t": [], "cL": [], "cR": [], "zL": [], "zR": [],
            "vxL": [], "vxR": [], "amp": [], "up": [], "x": [],
            "hipL": [], "hipR": [],
        }
        return self._obs(), {}

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float64).reshape(12)
        action = np.clip(action, -1.0, 1.0)
        d, m = self.data, self.model
        t = float(d.time)
        a = self._gait_amp(t)
        qdes = wg.gait_targets(t, True, a)
        wg.set_ctrl(m, d, qdes, self.act_idx)
        phi = (max(0.0, t - self.stand_hold) / max(wg.GAIT_T, 1e-6)) % 1.0 if a > 0 else 0.0
        lat = wg.lateral_com_target(phi) if a > 0 else 0.0

        # CoP settle bias (mirror walk_gait)
        settle = getattr(wg, "ANK_COP_BIAS_SETTLE_S", 0.55)
        if self.cop_bias_xy is None and t < settle:
            com = np.asarray(d.subtree_com[0, :2], dtype=np.float64)
            try:
                support, _, _, _ = wg._support_feet(
                    m, d, self.bid_lf, self.bid_rf, self.gid_floor, None,
                )
                self.cop_bias_samples.append(com - support)
            except Exception:
                pass
            if t >= settle - 1.5 / wg.CTRL_HZ and self.cop_bias_samples:
                self.cop_bias_xy = np.mean(np.stack(self.cop_bias_samples, axis=0), axis=0)

        if a > 0.01:
            bias = self.cop_bias_xy
            if bias is None and self.cop_bias_samples:
                bias = np.mean(np.stack(self.cop_bias_samples, axis=0), axis=0)
            try:
                wg.ankle_cop_servo(
                    m, d, self.act_idx, a,
                    self.bid_lf, self.bid_rf, self.gid_floor,
                    lat=lat if a > 0.05 else None, bias_xy=bias,
                )
            except Exception:
                pass

        if self.use_cp and a > 0.05:
            try:
                wg.apply_cp_swing_placement(
                    m, d, self.act_idx, phi, a, self.bid_lf, self.bid_rf,
                )
            except Exception:
                pass

        if self.use_vik and a > 0.05:
            try:
                wg.apply_stance_jacobian_vik(
                    m, d, self.act_idx, phi, a,
                    self.bid_lf, self.bid_rf, self.gid_floor,
                )
            except Exception:
                pass

        # residual Δctrl after open-loop stack
        if a > 0.05:
            delta = action * self.action_scale
            for i, ai in enumerate(self.leg_act):
                d.ctrl[ai] = float(np.clip(d.ctrl[ai] + delta[i], -CTRL_LIM, CTRL_LIM))

        for _ in range(self.steps_per_ctrl):
            d.qfrc_applied[:] = 0
            d.xfrc_applied[:] = 0
            if self.use_plant and a > 0:
                wg.stance_plant(
                    m, d, phi, a,
                    self.bid_lf, self.bid_rf,
                    self.gid_lfoot, self.gid_rfoot, self.gid_floor,
                )
            mj.mj_step(m, d)
            self.sat_steps += 1
            for ai in self.leg_act:
                lim = abs(float(m.actuator_forcerange[ai, 1]))
                if lim > 1e-9 and abs(float(d.actuator_force[ai])) >= 0.98 * lim:
                    self.sat_count += 1

        self.k += 1
        cL = self._contact(self.bid_lf)
        cR = self._contact(self.bid_rf)
        vxL = float(d.cvel[self.bid_lf][3])
        vxR = float(d.cvel[self.bid_rf][3])
        R = d.xmat[self.bid_body].reshape(3, 3)
        up_z = float(R[2, 2])
        # hip pitch for corr
        jL = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "l_hip_pitch")
        jR = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "r_hip_pitch")
        self.log["t"].append(float(d.time))
        self.log["cL"].append(1.0 if cL else 0.0)
        self.log["cR"].append(1.0 if cR else 0.0)
        self.log["zL"].append(float(d.xpos[self.bid_lf, 2]))
        self.log["zR"].append(float(d.xpos[self.bid_rf, 2]))
        self.log["vxL"].append(vxL)
        self.log["vxR"].append(vxR)
        self.log["amp"].append(a)
        self.log["up"].append(1.0 if up_z > 0.5 else 0.0)
        self.log["x"].append(float(d.qpos[0]))
        self.log["hipL"].append(float(d.qpos[m.jnt_qposadr[jL]]))
        self.log["hipR"].append(float(d.qpos[m.jnt_qposadr[jR]]))
        self._hip_buf_L.append(float(d.qpos[m.jnt_qposadr[jL]]))
        self._hip_buf_R.append(float(d.qpos[m.jnt_qposadr[jR]]))
        if len(self._hip_buf_L) > 100:
            self._hip_buf_L = self._hip_buf_L[-100:]
            self._hip_buf_R = self._hip_buf_R[-100:]

        reward = 0.0
        com_xy = d.subtree_com[0, :2].copy()
        dxy = float(com_xy[0] - self.prev_com_xy[0]) if self.prev_com_xy is not None else 0.0
        self.prev_com_xy = com_xy
        if a > 0.2:
            reward += 3.0 * dxy
            # skate penalty (stance foot vx)
            if cL and not cR:
                reward -= 4.0 * min(abs(vxL), 0.5)
            elif cR and not cL:
                reward -= 4.0 * min(abs(vxR), 0.5)
            elif cL and cR:
                reward -= 1.5 * (min(abs(vxL), 0.5) + min(abs(vxR), 0.5))
            reward += 0.08 * max(0.0, up_z)
            if cL and not cR:
                # Gate E peak_clear≥0.012 → z ≥ SOLE_OFFSET+0.012 = 0.038
                z_sw = float(d.xpos[self.bid_rf, 2])
                reward += 5.0 * max(0.0, z_sw - 0.038)
                reward -= 3.0 * max(0.0, 0.038 - z_sw)
            if cR and not cL:
                z_sw = float(d.xpos[self.bid_lf, 2])
                reward += 5.0 * max(0.0, z_sw - 0.038)
                reward -= 3.0 * max(0.0, 0.038 - z_sw)
            # prefer residual near 0 (keep OL structure)
            reward -= 0.01 * float(np.dot(action, action))
            # preserve hip anti-phase (Gate E / T88 hip_corr ≤ -0.45)
            if len(self._hip_buf_L) >= 40:
                fl = -np.asarray(self._hip_buf_L, dtype=np.float64)
                fr = +np.asarray(self._hip_buf_R, dtype=np.float64)
                if fl.std() > 1e-4 and fr.std() > 1e-4:
                    hc = float(np.corrcoef(fl, fr)[0, 1])
                    if hc < -0.45:
                        reward += 0.05
                    else:
                        reward -= 0.15 * max(0.0, hc + 0.45)

        terminated = False
        truncated = False
        info: dict[str, Any] = {}
        z = float(d.qpos[2])
        if (not np.isfinite(d.qpos).all()) or z < 0.10 or z > 0.50 or up_z < 0.25:
            reward -= 25.0
            terminated = True
            info["fall"] = True
        if self.k >= self.n_ctrl:
            truncated = True

        info["sat_rate"] = self.sat_count / max(1, self.sat_steps * len(self.leg_act))
        info["dx"] = float(d.qpos[0] - self.x0)
        info["gait_t"] = self.gait_t
        info["fall"] = bool(info.get("fall", False))
        return self._obs(), float(reward), terminated, truncated, info


def make_env(gait_t: float = 0.88, rank: int = 0, seed: int = 0, **kwargs):
    def _thunk():
        env = AinexResidualEnv(gait_t=gait_t, seed=seed + rank, **kwargs)
        env.reset(seed=seed + rank)
        return env
    return _thunk


# --- T4 reverse-native residual env (thin finetune; same 41-D obs) -----------------
# Opt B: vx_cmd encoded in obs[40] (tn slot; always 0 at T=0.75 for Gate Q)
# Opt C: reverse phase encoding in sin/cos inside NEW policy obs (not GATE_Q_RET_REV_PHASE)
# Opt F: asymmetric clear-SS reverse progress ≫ planted progress; Root B / plant-ε proxy

SOLE_OFFSET = float(getattr(wg, "SOLE_OFFSET", 0.026))
PLANT_REST_CLEAR = 0.0086  # documented plant rest clearance
FOOT_LIFT_AR = 0.02  # ≥2 cm above plant rest = sole clear (FOOT-LIFT honesty)


class AinexReverseResidualEnv(AinexResidualEnv):
    """Retreat-only residual train env. Bootstrap-compatible 41-D obs.

    Sagittal command spans negatives (opt B). Residual credit only while sole clear.
    Reward: clear_frac↑ (−dx while clear), skate↓, tip floor, heavy planted-progress
    / Root B penalty (opt F). Optional reverse phase encoding (opt C).
    """

    def __init__(
        self,
        *,
        shape: str = "GRO01",
        gait_t: float = 0.75,
        episode_s: float = 4.0,
        stand_hold: float = 0.60,
        ramp_t: float = 0.80,
        action_scale: float = 0.07,
        use_vik: bool = True,
        use_plant: bool = True,
        use_cp: bool = True,
        seed: int | None = None,
        vx_cmd: float = -0.08,
        target_dx: float = 0.12,
        rev_phase: bool = True,
        clear_only_residual: bool = True,
        vx_cmd_rand: bool = True,
        vx_lo: float = -0.12,
        vx_hi: float = -0.04,
    ):
        # Set reverse knobs BEFORE super() so _apply_shape sees them
        self.vx_cmd_nom = float(vx_cmd)
        self.vx_cmd = float(vx_cmd)
        self.target_dx = float(target_dx)
        self.rev_phase = bool(rev_phase)
        self.clear_only_residual = bool(clear_only_residual)
        self.vx_cmd_rand = bool(vx_cmd_rand)
        self.vx_lo = float(vx_lo)
        self.vx_hi = float(vx_hi)
        super().__init__(
            shape=shape, gait_t=gait_t, episode_s=episode_s,
            stand_hold=stand_hold, ramp_t=ramp_t, action_scale=action_scale,
            use_vik=use_vik, use_plant=use_plant, use_cp=use_cp, seed=seed,
        )
        self._dx_clear = 0.0
        self._dx_plant = 0.0
        self._plant_prog_cum = 0.0  # proxy for plant-cam cum ε
        self._plant_prog_any = 0.0
        self._clear_steps = 0
        self._gait_steps = 0
        self._tip_run = 0.0
        self._tip_max = 0.0

    def set_curriculum(self, *, target_dx: float | None = None, episode_s: float | None = None,
                       vx_lo: float | None = None, vx_hi: float | None = None) -> None:
        if target_dx is not None:
            self.target_dx = float(target_dx)
        if episode_s is not None:
            self.episode_s = float(episode_s)
            self.n_ctrl = int(self.episode_s / self.ctrl_dt)
        if vx_lo is not None:
            self.vx_lo = float(vx_lo)
        if vx_hi is not None:
            self.vx_hi = float(vx_hi)

    def _apply_shape(self) -> None:
        _apply_shape(self.shape, self.gait_t)
        # Reverse sagittal: negate step / hip amp+bias (mirror score_gate_q reverse-hip)
        wg.STEP_LEN = -abs(float(wg.STEP_LEN))
        wg.HIP_PITCH_AMP = -abs(float(wg.HIP_PITCH_AMP))
        wg.HIP_BIAS_FWD = -abs(float(wg.HIP_BIAS_FWD))
        # Scale magnitude lightly with |vx_cmd| (opt B library)
        vx = float(getattr(self, "vx_cmd", -0.08))
        v_scale = min(1.35, max(0.55, abs(vx) / 0.08))
        wg.STEP_LEN *= v_scale
        wg.HIP_PITCH_AMP *= v_scale
        wg.HIP_BIAS_FWD *= v_scale

    def _phi_enc(self, t: float, a: float) -> float:
        phi = (max(0.0, t - self.stand_hold) / max(wg.GAIT_T, 1e-6)) % 1.0 if a > 0 else 0.0
        if self.rev_phase and a > 0:
            phi = (phi + 0.5) % 1.0  # opt C: half-cycle reverse phase inside NEW weights
        return float(phi)

    def _sole_clear(self) -> tuple[bool, bool, bool]:
        """Return (any_clear, L_clear, R_clear) FOOT-LIFT ≥2 cm above plant rest."""
        d = self.data
        cL = float(d.xpos[self.bid_lf, 2]) - SOLE_OFFSET
        cR = float(d.xpos[self.bid_rf, 2]) - SOLE_OFFSET
        L = (cL - PLANT_REST_CLEAR) >= FOOT_LIFT_AR - 1e-9 and cL >= 0.03 - 1e-9
        R = (cR - PLANT_REST_CLEAR) >= FOOT_LIFT_AR - 1e-9 and cR >= 0.03 - 1e-9
        return (L or R), L, R

    def _obs(self) -> np.ndarray:
        d, m = self.data, self.model
        q = np.zeros(12, dtype=np.float64)
        dq = np.zeros(12, dtype=np.float64)
        for i, jn in enumerate(LEG_JOINTS):
            jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
            q[i] = d.qpos[m.jnt_qposadr[jid]]
            dq[i] = d.qvel[m.jnt_dofadr[jid]]
        v = d.qvel[0:3].copy()
        w = d.qvel[3:6].copy()
        quat = d.qpos[3:7].copy()
        t = float(d.time)
        a = self._gait_amp(t)
        phi = self._phi_enc(t, a)
        sinp, cosp = math.sin(2 * math.pi * phi), math.cos(2 * math.pi * phi)
        cL = 1.0 if self._contact(self.bid_lf) else 0.0
        cR = 1.0 if self._contact(self.bid_rf) else 0.0
        zL = float(d.xpos[self.bid_lf, 2])
        zR = float(d.xpos[self.bid_rf, 2])
        # Match score_gate_q retreat: flip observed vx so residual sees reverse as +cmd frame
        v[0] = -float(v[0])
        # Opt B: commanded Vx (normalized) in tn slot; remaining retreat Δ co-encoded lightly
        vx_n = float(np.clip(self.vx_cmd / 0.15, -1.0, 1.0))
        dx_done = max(0.0, self.x0 - float(d.qpos[0]))  # reverse progress (+ when going −X)
        remain = float(np.clip((self.target_dx - dx_done) / max(self.target_dx, 1e-6), -1.0, 1.0))
        # Pack: primarily vx_cmd; remaining Δ in low bits via 0.7*vx + 0.3*remain (both ∈[-1,1])
        cmd_slot = 0.70 * vx_n + 0.30 * remain
        return np.concatenate([
            q, dq, v, w, quat,
            [sinp, cosp, cL, cR, zL, zR, cmd_slot],
        ]).astype(np.float32)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        if self.vx_cmd_rand:
            self.vx_cmd = float(self._rng.uniform(self.vx_lo, self.vx_hi))
        else:
            self.vx_cmd = float(self.vx_cmd_nom)
        obs, info = super().reset(seed=seed, options=options)
        self._dx_clear = 0.0
        self._dx_plant = 0.0
        self._plant_prog_cum = 0.0
        self._plant_prog_any = 0.0
        self._clear_steps = 0
        self._gait_steps = 0
        self._tip_run = 0.0
        self._tip_max = 0.0
        return obs, info

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float64).reshape(12)
        action = np.clip(action, -1.0, 1.0)
        any_clear, _, _ = self._sole_clear()
        # CLEAR_TRACK honesty: zero residual while planted (no planted soft-XY credit)
        if self.clear_only_residual and not any_clear:
            action = np.zeros(12, dtype=np.float64)

        d, m = self.data, self.model
        t = float(d.time)
        a = self._gait_amp(t)
        qdes = wg.gait_targets(t, True, a)
        wg.set_ctrl(m, d, qdes, self.act_idx)
        # Use reverse-phase for CP/VIK lat when opt C on
        phi_ol = (max(0.0, t - self.stand_hold) / max(wg.GAIT_T, 1e-6)) % 1.0 if a > 0 else 0.0
        phi = self._phi_enc(t, a) if self.rev_phase else phi_ol
        lat = wg.lateral_com_target(phi) if a > 0 else 0.0

        settle = getattr(wg, "ANK_COP_BIAS_SETTLE_S", 0.55)
        if self.cop_bias_xy is None and t < settle:
            com = np.asarray(d.subtree_com[0, :2], dtype=np.float64)
            try:
                support, _, _, _ = wg._support_feet(
                    m, d, self.bid_lf, self.bid_rf, self.gid_floor, None,
                )
                self.cop_bias_samples.append(com - support)
            except Exception:
                pass
            if t >= settle - 1.5 / wg.CTRL_HZ and self.cop_bias_samples:
                self.cop_bias_xy = np.mean(np.stack(self.cop_bias_samples, axis=0), axis=0)

        if a > 0.01:
            bias = self.cop_bias_xy
            if bias is None and self.cop_bias_samples:
                bias = np.mean(np.stack(self.cop_bias_samples, axis=0), axis=0)
            try:
                wg.ankle_cop_servo(
                    m, d, self.act_idx, a,
                    self.bid_lf, self.bid_rf, self.gid_floor,
                    lat=lat if a > 0.05 else None, bias_xy=bias,
                )
            except Exception:
                pass

        if self.use_cp and a > 0.05:
            try:
                wg.apply_cp_swing_placement(
                    m, d, self.act_idx, phi, a, self.bid_lf, self.bid_rf,
                )
            except Exception:
                pass

        if self.use_vik and a > 0.05:
            try:
                wg.apply_stance_jacobian_vik(
                    m, d, self.act_idx, phi, a,
                    self.bid_lf, self.bid_rf, self.gid_floor,
                )
            except Exception:
                pass

        if a > 0.05:
            delta = action * self.action_scale
            for i, ai in enumerate(self.leg_act):
                d.ctrl[ai] = float(np.clip(d.ctrl[ai] + delta[i], -CTRL_LIM, CTRL_LIM))

        for _ in range(self.steps_per_ctrl):
            d.qfrc_applied[:] = 0
            d.xfrc_applied[:] = 0
            if self.use_plant and a > 0:
                # Soft cancel proxy: skip freejoint XY plant shove while planted (mirror plant_gate)
                if any_clear:
                    wg.stance_plant(
                        m, d, phi_ol, a,
                        self.bid_lf, self.bid_rf,
                        self.gid_lfoot, self.gid_rfoot, self.gid_floor,
                    )
            mj.mj_step(m, d)
            self.sat_steps += 1
            for ai in self.leg_act:
                lim = abs(float(m.actuator_forcerange[ai, 1]))
                if lim > 1e-9 and abs(float(d.actuator_force[ai])) >= 0.98 * lim:
                    self.sat_count += 1

        self.k += 1
        cL = self._contact(self.bid_lf)
        cR = self._contact(self.bid_rf)
        vxL = float(d.cvel[self.bid_lf][3])
        vxR = float(d.cvel[self.bid_rf][3])
        R = d.xmat[self.bid_body].reshape(3, 3)
        up_z = float(R[2, 2])
        jL = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "l_hip_pitch")
        jR = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "r_hip_pitch")
        self.log["t"].append(float(d.time))
        self.log["cL"].append(1.0 if cL else 0.0)
        self.log["cR"].append(1.0 if cR else 0.0)
        self.log["zL"].append(float(d.xpos[self.bid_lf, 2]))
        self.log["zR"].append(float(d.xpos[self.bid_rf, 2]))
        self.log["vxL"].append(vxL)
        self.log["vxR"].append(vxR)
        self.log["amp"].append(a)
        self.log["up"].append(1.0 if up_z > 0.5 else 0.0)
        self.log["x"].append(float(d.qpos[0]))
        self.log["hipL"].append(float(d.qpos[m.jnt_qposadr[jL]]))
        self.log["hipR"].append(float(d.qpos[m.jnt_qposadr[jR]]))
        self._hip_buf_L.append(float(d.qpos[m.jnt_qposadr[jL]]))
        self._hip_buf_R.append(float(d.qpos[m.jnt_qposadr[jR]]))
        if len(self._hip_buf_L) > 100:
            self._hip_buf_L = self._hip_buf_L[-100:]
            self._hip_buf_R = self._hip_buf_R[-100:]

        # --- T4 reward (opt F asymmetric; Prefer FAIL plant-ε proxy) ---
        reward = 0.0
        com_xy = d.subtree_com[0, :2].copy()
        dxy = float(com_xy[0] - self.prev_com_xy[0]) if self.prev_com_xy is not None else 0.0
        self.prev_com_xy = com_xy
        # reverse progress = −dxy (want to go −X)
        rev_step = -dxy
        any_clear, _, _ = self._sole_clear()
        if a > 0.2:
            self._gait_steps += 1
            if any_clear:
                self._clear_steps += 1
                self._dx_clear += max(0.0, rev_step)
                # Primary: clear-SS reverse bank
                reward += 8.0 * rev_step
                # Clear height bonus
                z_sw = max(float(d.xpos[self.bid_lf, 2]), float(d.xpos[self.bid_rf, 2]))
                reward += 4.0 * max(0.0, z_sw - (SOLE_OFFSET + FOOT_LIFT_AR))
            else:
                # Root B / plant-cam ε proxy: planted reverse progress heavily penalized
                plant_prog = max(0.0, rev_step)
                self._dx_plant += plant_prog
                self._plant_prog_cum += plant_prog
                self._plant_prog_any = max(self._plant_prog_any, plant_prog)
                reward -= 25.0 * plant_prog  # heavy Root B
                # Hard Prefer FAIL-style spike if any plant window ≫ ε (0.02) or cum ≫ 0.05
                if plant_prog > 0.02:
                    reward -= 5.0
                if self._plant_prog_cum > 0.05:
                    reward -= 2.0
            # Skate (stance foot vx) — bars ≤0.08/0.18
            if cL and not cR:
                reward -= 6.0 * min(abs(vxL), 0.5)
            elif cR and not cL:
                reward -= 6.0 * min(abs(vxR), 0.5)
            elif cL and cR:
                reward -= 2.5 * (min(abs(vxL), 0.5) + min(abs(vxR), 0.5))
            # Tip floor
            reward += 0.12 * max(0.0, up_z)
            if up_z > 0.5:
                self._tip_run += self.ctrl_dt
                self._tip_max = max(self._tip_max, self._tip_run)
            else:
                self._tip_run = 0.0
            reward -= 0.015 * float(np.dot(action, action))
            # Target Δ shaping
            dx_done = max(0.0, self.x0 - float(d.qpos[0]))
            if dx_done >= self.target_dx - 1e-3:
                reward += 1.5

        terminated = False
        truncated = False
        info: dict[str, Any] = {}
        z = float(d.qpos[2])
        if (not np.isfinite(d.qpos).all()) or z < 0.10 or z > 0.50 or up_z < 0.25:
            reward -= 25.0
            terminated = True
            info["fall"] = True
        if self.k >= self.n_ctrl:
            truncated = True
            # Terminal clear_frac bonus / plant Prefer FAIL
            cf = self._dx_clear / max(self._dx_clear + self._dx_plant, 1e-6)
            reward += 4.0 * cf
            if self._tip_max < 8.0:
                reward -= 3.0
            if self._plant_prog_any > 0.02 or self._plant_prog_cum > 0.05:
                reward -= 8.0  # Prefer FAIL plant-ε terminal
            info["clear_frac"] = float(cf)
            info["dx_clear"] = float(self._dx_clear)
            info["dx_plant"] = float(self._dx_plant)
            info["tip_max"] = float(self._tip_max)
            info["plant_prog_cum"] = float(self._plant_prog_cum)
            info["plant_prog_any"] = float(self._plant_prog_any)
            info["target_dx"] = float(self.target_dx)
            info["vx_cmd"] = float(self.vx_cmd)

        info["sat_rate"] = self.sat_count / max(1, self.sat_steps * len(self.leg_act))
        info["dx"] = float(self.x0 - d.qpos[0])  # reverse Δ positive
        info["gait_t"] = self.gait_t
        info["fall"] = bool(info.get("fall", False))
        return self._obs(), float(reward), terminated, truncated, info


def make_reverse_env(
    gait_t: float = 0.75,
    rank: int = 0,
    seed: int = 0,
    **kwargs,
):
    def _thunk():
        env = AinexReverseResidualEnv(gait_t=gait_t, seed=seed + rank, **kwargs)
        env.reset(seed=seed + rank)
        return env
    return _thunk


# --- T5-A SKATE-PRIMARY reverse residual (invert T4 priority) -----------------------
# Cospec: docs/GATE_Q_AI_COSPEC_T5.md
# Reward dominated by skate mean + heavy p95; cf≥0.55 as floor/constraint.
# Clear-only residual. Optional SLR/TDVM + ANK DF + DXB (in-family).
# Planted: cancel×0.70 path (clear-only residual zero) — zero planted soft-XY credit.
# Keep T4 opts B (neg Vx obs) · C (rev phase) · F (Root B / plant-ε) as conditioning.


class AinexT5SkateEnv(AinexReverseResidualEnv):
    """T5-A skate-primary reverse residual. Same 41-D obs as T4/E7lock.

    Primary: skate mean + heavy p95 (stance |vx|).
    Constrain: clear_frac≥0.55, tip≥8, plant-cam ε, apps held at eval.
    Optional: SLR late-swing tangential match, ANK DF clear-only, DXB short-step.
    """

    def __init__(
        self,
        *,
        shape: str = "GRO01",
        gait_t: float = 0.75,
        episode_s: float = 4.0,
        stand_hold: float = 0.60,
        ramp_t: float = 0.80,
        action_scale: float = 0.07,
        use_vik: bool = True,
        use_plant: bool = True,
        use_cp: bool = True,
        seed: int | None = None,
        vx_cmd: float = -0.08,
        target_dx: float = 0.10,
        rev_phase: bool = True,
        clear_only_residual: bool = True,
        vx_cmd_rand: bool = True,
        vx_lo: float = -0.10,
        vx_hi: float = -0.04,
        use_slr: bool = True,
        use_ank: bool = True,
        use_dxb: bool = True,
        dxb_cap: float = 0.045,
    ):
        self.use_slr = bool(use_slr)
        self.use_ank = bool(use_ank)
        self.use_dxb = bool(use_dxb)
        self.dxb_cap = float(dxb_cap)
        super().__init__(
            shape=shape, gait_t=gait_t, episode_s=episode_s,
            stand_hold=stand_hold, ramp_t=ramp_t, action_scale=action_scale,
            use_vik=use_vik, use_plant=use_plant, use_cp=use_cp, seed=seed,
            vx_cmd=vx_cmd, target_dx=target_dx, rev_phase=rev_phase,
            clear_only_residual=clear_only_residual, vx_cmd_rand=vx_cmd_rand,
            vx_lo=vx_lo, vx_hi=vx_hi,
        )
        # skate running buffer for p95 proxy
        self._skate_buf: list[float] = []
        self._v_sw_tang_preTD = 0.0
        self._dx_back_max = 0.0
        self._prev_x = None
        self._ank_df_bonus_cum = 0.0

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        obs, info = super().reset(seed=seed, options=options)
        self._skate_buf = []
        self._v_sw_tang_preTD = 0.0
        self._dx_back_max = 0.0
        self._prev_x = float(self.data.qpos[0])
        self._ank_df_bonus_cum = 0.0
        return obs, info

    def _late_swing_clear(self) -> tuple[bool, int]:
        """Late-swing: sole clear but descending toward plant (clearance shrinking zone).
        Returns (is_late, swing_side) where swing_side: 0=L 1=R -1=none.
        """
        d = self.data
        cL = float(d.xpos[self.bid_lf, 2]) - SOLE_OFFSET
        cR = float(d.xpos[self.bid_rf, 2]) - SOLE_OFFSET
        # clearance above plant rest
        cl = cL - PLANT_REST_CLEAR
        cr = cR - PLANT_REST_CLEAR
        # late swing window: above FOOT_LIFT honesty floor but not high aerial
        # 2–5 cm above plant rest
        L_late = FOOT_LIFT_AR - 1e-9 <= cl <= 0.05 + 1e-9 and cL >= 0.03 - 1e-9
        R_late = FOOT_LIFT_AR - 1e-9 <= cr <= 0.05 + 1e-9 and cR >= 0.03 - 1e-9
        # prefer the higher foot as swing if both somehow late
        if L_late and (not R_late or cl >= cr):
            return True, 0
        if R_late:
            return True, 1
        return False, -1

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float64).reshape(12)
        action = np.clip(action, -1.0, 1.0)
        any_clear, L_clear, R_clear = self._sole_clear()

        # Optional ANK: soft clear-only ankle-DF residual bias (indices 4=l_ank_pitch, 10=r_ank_pitch)
        # Positive ank_pitch convention in AiNex = DF (toes up) for soft TD — clear-only.
        if self.use_ank and any_clear:
            late, side = self._late_swing_clear()
            if late and side == 0 and L_clear:
                action = action.copy()
                action[4] = float(np.clip(action[4] + 0.25, -1.0, 1.0))
            elif late and side == 1 and R_clear:
                action = action.copy()
                action[10] = float(np.clip(action[10] + 0.25, -1.0, 1.0))

        # CLEAR_TRACK honesty: zero residual while planted
        if self.clear_only_residual and not any_clear:
            action = np.zeros(12, dtype=np.float64)

        d, m = self.data, self.model
        t = float(d.time)
        a = self._gait_amp(t)
        qdes = wg.gait_targets(t, True, a)
        wg.set_ctrl(m, d, qdes, self.act_idx)
        phi_ol = (max(0.0, t - self.stand_hold) / max(wg.GAIT_T, 1e-6)) % 1.0 if a > 0 else 0.0
        phi = self._phi_enc(t, a) if self.rev_phase else phi_ol
        lat = wg.lateral_com_target(phi) if a > 0 else 0.0

        settle = getattr(wg, "ANK_COP_BIAS_SETTLE_S", 0.55)
        if self.cop_bias_xy is None and t < settle:
            com = np.asarray(d.subtree_com[0, :2], dtype=np.float64)
            try:
                support, _, _, _ = wg._support_feet(
                    m, d, self.bid_lf, self.bid_rf, self.gid_floor, None,
                )
                self.cop_bias_samples.append(com - support)
            except Exception:
                pass
            if t >= settle - 1.5 / wg.CTRL_HZ and self.cop_bias_samples:
                self.cop_bias_xy = np.mean(np.stack(self.cop_bias_samples, axis=0), axis=0)

        if a > 0.01:
            bias = self.cop_bias_xy
            if bias is None and self.cop_bias_samples:
                bias = np.mean(np.stack(self.cop_bias_samples, axis=0), axis=0)
            try:
                wg.ankle_cop_servo(
                    m, d, self.act_idx, a,
                    self.bid_lf, self.bid_rf, self.gid_floor,
                    lat=lat if a > 0.05 else None, bias_xy=bias,
                )
            except Exception:
                pass

        if self.use_cp and a > 0.05:
            try:
                wg.apply_cp_swing_placement(
                    m, d, self.act_idx, phi, a, self.bid_lf, self.bid_rf,
                )
            except Exception:
                pass

        if self.use_vik and a > 0.05:
            try:
                wg.apply_stance_jacobian_vik(
                    m, d, self.act_idx, phi, a,
                    self.bid_lf, self.bid_rf, self.gid_floor,
                )
            except Exception:
                pass

        if a > 0.05:
            delta = action * self.action_scale
            for i, ai in enumerate(self.leg_act):
                d.ctrl[ai] = float(np.clip(d.ctrl[ai] + delta[i], -CTRL_LIM, CTRL_LIM))

        for _ in range(self.steps_per_ctrl):
            d.qfrc_applied[:] = 0
            d.xfrc_applied[:] = 0
            if self.use_plant and a > 0:
                if any_clear:
                    wg.stance_plant(
                        m, d, phi_ol, a,
                        self.bid_lf, self.bid_rf,
                        self.gid_lfoot, self.gid_rfoot, self.gid_floor,
                    )
            mj.mj_step(m, d)
            self.sat_steps += 1
            for ai in self.leg_act:
                lim = abs(float(m.actuator_forcerange[ai, 1]))
                if lim > 1e-9 and abs(float(d.actuator_force[ai])) >= 0.98 * lim:
                    self.sat_count += 1

        self.k += 1
        cL = self._contact(self.bid_lf)
        cR = self._contact(self.bid_rf)
        vxL = float(d.cvel[self.bid_lf][3])
        vxR = float(d.cvel[self.bid_rf][3])
        R = d.xmat[self.bid_body].reshape(3, 3)
        up_z = float(R[2, 2])
        jL = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "l_hip_pitch")
        jR = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "r_hip_pitch")
        self.log["t"].append(float(d.time))
        self.log["cL"].append(1.0 if cL else 0.0)
        self.log["cR"].append(1.0 if cR else 0.0)
        self.log["zL"].append(float(d.xpos[self.bid_lf, 2]))
        self.log["zR"].append(float(d.xpos[self.bid_rf, 2]))
        self.log["vxL"].append(vxL)
        self.log["vxR"].append(vxR)
        self.log["amp"].append(a)
        self.log["up"].append(1.0 if up_z > 0.5 else 0.0)
        self.log["x"].append(float(d.qpos[0]))
        self.log["hipL"].append(float(d.qpos[m.jnt_qposadr[jL]]))
        self.log["hipR"].append(float(d.qpos[m.jnt_qposadr[jR]]))
        self._hip_buf_L.append(float(d.qpos[m.jnt_qposadr[jL]]))
        self._hip_buf_R.append(float(d.qpos[m.jnt_qposadr[jR]]))
        if len(self._hip_buf_L) > 100:
            self._hip_buf_L = self._hip_buf_L[-100:]
            self._hip_buf_R = self._hip_buf_R[-100:]

        # --- T5-A SKATE-PRIMARY reward ---
        reward = 0.0
        com_xy = d.subtree_com[0, :2].copy()
        dxy = float(com_xy[0] - self.prev_com_xy[0]) if self.prev_com_xy is not None else 0.0
        self.prev_com_xy = com_xy
        rev_step = -dxy  # want −X
        any_clear, L_clear, R_clear = self._sole_clear()

        # DXB: track per-step reverse Δ and soft-cap
        x_now = float(d.qpos[0])
        if self._prev_x is not None:
            step_back = max(0.0, self._prev_x - x_now)
            self._dx_back_max = max(self._dx_back_max, step_back)
            if self.use_dxb and a > 0.2 and step_back > self.dxb_cap:
                reward -= 4.0 * (step_back - self.dxb_cap)
        self._prev_x = x_now

        if a > 0.2:
            self._gait_steps += 1
            # ---- SKATE PRIMARY (dominate) ----
            sk = 0.0
            if cL and not cR:
                sk = abs(vxL)
                reward -= 14.0 * min(sk, 0.5)
            elif cR and not cL:
                sk = abs(vxR)
                reward -= 14.0 * min(sk, 0.5)
            elif cL and cR:
                sk = 0.5 * (abs(vxL) + abs(vxR))
                reward -= 6.0 * (min(abs(vxL), 0.5) + min(abs(vxR), 0.5))
            if cL or cR:
                self._skate_buf.append(float(sk))
                # Heavy p95 proxy: quadratic above bars
                if sk > 0.08:
                    reward -= 20.0 * (sk - 0.08) ** 2
                if sk > 0.18:
                    reward -= 40.0 * (sk - 0.18)  # spike Prefer FAIL zone

            # ---- Clear progress as FLOOR/CONSTRAINT (not primary climb) ----
            if any_clear:
                self._clear_steps += 1
                self._dx_clear += max(0.0, rev_step)
                # Mild clear-SS reverse (was 8.0 in T4 — demoted)
                reward += 3.0 * rev_step
                z_sw = max(float(d.xpos[self.bid_lf, 2]), float(d.xpos[self.bid_rf, 2]))
                reward += 1.5 * max(0.0, z_sw - (SOLE_OFFSET + FOOT_LIFT_AR))

                # ---- SLR / TDVM: late-swing tangential → ground match / slight swing-back ----
                if self.use_slr:
                    late, side = self._late_swing_clear()
                    if late and side >= 0:
                        v_sw = vxL if side == 0 else vxR
                        # COM traveling −X on reverse; slight swing-backward vs travel = +vx world
                        # Ground-speed match target ≈ 0; prefer mild +0.02..+0.06 (swing-back)
                        target = 0.03
                        err = abs(float(v_sw) - target)
                        reward -= 8.0 * min(err, 0.4)
                        # bonus if within soft band
                        if abs(float(v_sw) - target) < 0.05:
                            reward += 0.4
                        self._v_sw_tang_preTD = float(v_sw)

                # ---- ANK DF shaping reward (clear late-swing) ----
                if self.use_ank:
                    late, side = self._late_swing_clear()
                    if late and side >= 0:
                        jn = "l_ank_pitch" if side == 0 else "r_ank_pitch"
                        jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
                        ank = float(d.qpos[m.jnt_qposadr[jid]])
                        # DF positive → soft TD (reward mild positive ank)
                        df_bonus = 0.3 * max(0.0, min(ank, 0.35))
                        reward += df_bonus
                        self._ank_df_bonus_cum += df_bonus
            else:
                # Root B / plant-cam ε — Prefer FAIL heavy (kept from T4 opt F)
                plant_prog = max(0.0, rev_step)
                self._dx_plant += plant_prog
                self._plant_prog_cum += plant_prog
                self._plant_prog_any = max(self._plant_prog_any, plant_prog)
                reward -= 25.0 * plant_prog
                if plant_prog > 0.02:
                    reward -= 5.0
                if self._plant_prog_cum > 0.05:
                    reward -= 2.0

            # Tip floor
            reward += 0.12 * max(0.0, up_z)
            if up_z > 0.5:
                self._tip_run += self.ctrl_dt
                self._tip_max = max(self._tip_max, self._tip_run)
            else:
                self._tip_run = 0.0
            reward -= 0.015 * float(np.dot(action, action))

            dx_done = max(0.0, self.x0 - float(d.qpos[0]))
            if dx_done >= self.target_dx - 1e-3:
                reward += 1.0  # mild (not primary)

        terminated = False
        truncated = False
        info: dict[str, Any] = {}
        z = float(d.qpos[2])
        if (not np.isfinite(d.qpos).all()) or z < 0.10 or z > 0.50 or up_z < 0.25:
            reward -= 25.0
            terminated = True
            info["fall"] = True
        if self.k >= self.n_ctrl:
            truncated = True
            cf = self._dx_clear / max(self._dx_clear + self._dx_plant, 1e-6)
            # cf as FLOOR: bonus only if ≥0.55; Prefer FAIL dump if traded away
            if cf >= 0.55 - 1e-9:
                reward += 2.0 * cf
            else:
                reward -= 6.0 * (0.55 - cf)  # constrain — do not trade cf for fake skate
            # Terminal skate mean/p95 dominate
            if self._skate_buf:
                arr = np.asarray(self._skate_buf, dtype=np.float64)
                sk_mean = float(np.mean(arr))
                sk_p95 = float(np.percentile(arr, 95))
                reward -= 12.0 * max(0.0, sk_mean - 0.08)
                reward -= 25.0 * max(0.0, sk_p95 - 0.18)
                # mild bonus under bars
                if sk_mean <= 0.08 + 1e-9 and sk_p95 <= 0.18 + 1e-9:
                    reward += 8.0
                info["skate_mean"] = sk_mean
                info["skate_p95"] = sk_p95
            if self._tip_max < 8.0:
                reward -= 3.0
            if self._plant_prog_any > 0.02 or self._plant_prog_cum > 0.05:
                reward -= 8.0
            info["clear_frac"] = float(cf)
            info["dx_clear"] = float(self._dx_clear)
            info["dx_plant"] = float(self._dx_plant)
            info["tip_max"] = float(self._tip_max)
            info["plant_prog_cum"] = float(self._plant_prog_cum)
            info["plant_prog_any"] = float(self._plant_prog_any)
            info["target_dx"] = float(self.target_dx)
            info["vx_cmd"] = float(self.vx_cmd)
            info["v_sw_tang_preTD"] = float(self._v_sw_tang_preTD)
            info["dx_back_max"] = float(self._dx_back_max)
            info["ank_df_bonus_cum"] = float(self._ank_df_bonus_cum)

        info["sat_rate"] = self.sat_count / max(1, self.sat_steps * len(self.leg_act))
        info["dx"] = float(self.x0 - d.qpos[0])
        info["gait_t"] = self.gait_t
        info["fall"] = bool(info.get("fall", False))
        return self._obs(), float(reward), terminated, truncated, info


def make_t5_env(
    gait_t: float = 0.75,
    rank: int = 0,
    seed: int = 0,
    **kwargs,
):
    def _thunk():
        env = AinexT5SkateEnv(gait_t=gait_t, seed=seed + rank, **kwargs)
        env.reset(seed=seed + rank)
        return env
    return _thunk


# ---------------------------------------------------------------------------
# T5-B — ALIP/DCM + ankle model-based reverse base + residual (NEW weights)
# Cospec: docs/GATE_Q_AI_COSPEC_T5B.md  ·  ≠ S2 twin flags on frozen E7lock
# ---------------------------------------------------------------------------

def _alip_foothold_prior(com_xy, com_vxy, com_z, vx_cmd, *, dx_cap=0.04, dx_remain=None):
    """ALIP S2S foothold prior for reverse Vx. F ≤ 0 (land further −X)."""
    g = 9.81
    z = max(float(com_z), 0.12)
    omega = max(math.sqrt(g / z), 0.5)
    vx = float(com_vxy[0])
    vxc = float(vx_cmd)
    F = vxc / max(omega * 0.35, 0.5) + 0.25 * (vx - vxc) / omega
    if dx_remain is not None and float(dx_remain) > 1e-4:
        share = min(float(dx_remain) * 0.12, abs(float(dx_cap)) if float(dx_cap) > 1e-9 else 0.08)
        F = 0.75 * F - 0.25 * share
    cap = abs(float(dx_cap)) if float(dx_cap) > 1e-9 else 0.08
    F = float(np.clip(F, -cap, 0.0))
    return F, omega


def _dcm_reverse_bias(com_xy, com_vxy, com_z, support_x) -> float:
    """DCM ξ = x + vx/ω; return extra reverse foothold bias (≤0) if DCM overshoots +X."""
    g = 9.81
    z = max(float(com_z), 0.12)
    omega = max(math.sqrt(g / z), 0.5)
    xi = float(com_xy[0]) + float(com_vxy[0]) / omega
    # Overshoot beyond support toward door (+X) → push land further −X
    over = xi - float(support_x)
    if over > 0.0:
        return float(np.clip(-0.35 * over, -0.03, 0.0))
    return 0.0


def _apply_alip_fp_track_env(m, d, act_idx, phi, amp, bid_lf, bid_rf, F_cmd, gain=0.22):
    """Clear-only hip+ankle track toward support+F. No planted soft-XY."""
    if amp < 0.05 or abs(float(F_cmd)) < 1e-6:
        return 0.0
    rest = PLANT_REST_CLEAR
    ar_m = FOOT_LIFT_AR
    abs_m = 0.03
    ds_frac = float(np.clip(wg.DS_S / max(wg.GAIT_T, 1e-3), 0.08, 0.55))
    ds_end = 0.5 + 0.5 * ds_frac
    gain = max(float(gain), 0.05)
    zL = float(d.xpos[bid_lf, 2]); zR = float(d.xpos[bid_rf, 2])
    cL = zL - SOLE_OFFSET; cR = zR - SOLE_OFFSET
    L_plant = not ((cL - rest) >= ar_m - 1e-9 and cL >= abs_m - 1e-9)
    R_plant = not ((cR - rest) >= ar_m - 1e-9 and cR >= abs_m - 1e-9)
    if L_plant and not R_plant:
        support_x = float(d.xpos[bid_lf, 0])
    elif R_plant and not L_plant:
        support_x = float(d.xpos[bid_rf, 0])
    else:
        support_x = 0.5 * (float(d.xpos[bid_lf, 0]) + float(d.xpos[bid_rf, 0]))
    des_x = support_x + float(F_cmd)
    ank_cmd = 0.0
    for side, bid, sign in (("L", bid_lf, -1.0), ("R", bid_rf, +1.0)):
        z = float(d.xpos[bid, 2])
        clr = z - SOLE_OFFSET
        if (clr - rest) < ar_m - 1e-9 or clr < abs_m - 1e-9:
            continue
        p_leg = wg.phase_leg(phi, side)
        if p_leg < ds_end:
            continue
        s = (p_leg - ds_end) / max(1e-6, 1.0 - ds_end)
        if s < 0.15 or s > 0.95:
            continue
        sw = wg.swing_blend(min(1.0, s / 0.7))
        foot_x = float(d.xpos[bid, 0])
        err = des_x - foot_x
        d_fwd = float(np.clip(err / gain, -0.30, 0.30)) * float(amp) * sw
        pref = "l_" if side == "L" else "r_"
        an = f"{pref}hip_pitch_pos"
        if an in act_idx:
            d.ctrl[act_idx[an]] = float(np.clip(d.ctrl[act_idx[an]] + sign * d_fwd, -1.5, 1.5))
        an_a = f"{pref}ank_pitch_pos"
        if an_a in act_idx:
            d_ank = sign * 0.45 * d_fwd
            d.ctrl[act_idx[an_a]] = float(np.clip(d.ctrl[act_idx[an_a]] + d_ank, -1.2, 1.2))
            ank_cmd += abs(d_ank)
    return float(ank_cmd)


class AinexT5BModelBaseEnv(AinexT5SkateEnv):
    """T5-B: ALIP/DCM reverse foothold + ankle prior as BASE, PPO residual closes gap.

    Live in NEW trained weights only. Clear-only authority. cancel×0.70 planted.
    Attack: skate mean/p95 (bout1 wall) while holding cf≥0.55 + tip≥8 + plant-cam ε.
    """

    def __init__(
        self,
        *,
        use_alip: bool = True,
        use_dcm: bool = True,
        use_ank_nmpc: bool = True,
        n_replan: int = 2,
        alip_dx_cap: float = 0.04,
        alip_smooth: float = 0.45,
        alip_track_gain: float = 0.22,
        **kwargs,
    ):
        # Keep T5-A conditioning (SLR/ANK/DXB) as secondary; ALIP/DCM is the family
        kwargs.setdefault("use_slr", True)
        kwargs.setdefault("use_ank", True)
        kwargs.setdefault("use_dxb", True)
        super().__init__(**kwargs)
        self.use_alip = bool(use_alip)
        self.use_dcm = bool(use_dcm)
        self.use_ank_nmpc = bool(use_ank_nmpc)
        self.n_replan = max(1, int(n_replan))
        self.alip_dx_cap = float(alip_dx_cap)
        self.alip_smooth = float(np.clip(alip_smooth, 0.0, 0.95))
        self.alip_track_gain = float(alip_track_gain)
        self._alip_fp_prior = 0.0
        self._alip_du_fp = 0.0
        self._alip_du_prev = 0.0
        self._alip_n_replan = 0
        self._alip_fp_jump_max = 0.0
        self._alip_replan_idx = -1
        self._alip_swing_side = None
        self._ank_cmd_cum = 0.0
        self._fp_track_err_cum = 0.0

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        obs, info = super().reset(seed=seed, options=options)
        self._alip_fp_prior = 0.0
        self._alip_du_fp = 0.0
        self._alip_du_prev = 0.0
        self._alip_n_replan = 0
        self._alip_fp_jump_max = 0.0
        self._alip_replan_idx = -1
        self._alip_swing_side = None
        self._ank_cmd_cum = 0.0
        self._fp_track_err_cum = 0.0
        return obs, info

    def _update_alip_state(self, phi: float, a: float) -> float:
        """Mid-swing ALIP(+DCM) replan. Returns F_cmd = du_fp. Clear-only."""
        if not self.use_alip or a < 0.05:
            self._alip_replan_idx = -1
            self._alip_swing_side = None
            return 0.0
        d, m = self.data, self.model
        any_clear, L_clear, R_clear = self._sole_clear()
        if not any_clear:
            self._alip_replan_idx = -1
            self._alip_swing_side = None
            return float(self._alip_du_fp)
        ds_frac = float(np.clip(wg.DS_S / max(wg.GAIT_T, 1e-3), 0.08, 0.55))
        ds_end = 0.5 + 0.5 * ds_frac
        swing_side = None
        s_sw = None
        for side, clr in (("L", L_clear), ("R", R_clear)):
            if not clr:
                continue
            pl = wg.phase_leg(phi, side)
            if pl < ds_end:
                continue
            s = (pl - ds_end) / max(1e-6, 1.0 - ds_end)
            if 0.05 <= s <= 0.98:
                swing_side = side
                s_sw = float(s)
                break
        if swing_side is None:
            self._alip_replan_idx = -1
            self._alip_swing_side = None
            return float(self._alip_du_fp)
        ridx = min(int(s_sw * self.n_replan), self.n_replan - 1)
        side_chg = self._alip_swing_side != swing_side
        if side_chg or self._alip_replan_idx != ridx:
            try:
                com_xy, com_vxy, com_z = wg.estimate_com_state(m, d)
            except Exception:
                com_xy = d.subtree_com[0, :2].copy()
                com_vxy = np.zeros(2)
                com_z = float(d.qpos[2])
            dx_done = max(0.0, self.x0 - float(d.qpos[0]))
            dx_rem = max(0.0, self.target_dx - dx_done)
            fp, _om = _alip_foothold_prior(
                com_xy, com_vxy, com_z, self.vx_cmd,
                dx_cap=self.alip_dx_cap, dx_remain=dx_rem,
            )
            if self.use_dcm:
                # support ≈ planted foot
                zL = float(d.xpos[self.bid_lf, 2]); zR = float(d.xpos[self.bid_rf, 2])
                cL = zL - SOLE_OFFSET; cR = zR - SOLE_OFFSET
                L_plant = not ((cL - PLANT_REST_CLEAR) >= FOOT_LIFT_AR - 1e-9 and cL >= 0.03 - 1e-9)
                R_plant = not ((cR - PLANT_REST_CLEAR) >= FOOT_LIFT_AR - 1e-9 and cR >= 0.03 - 1e-9)
                if L_plant and not R_plant:
                    sx = float(d.xpos[self.bid_lf, 0])
                elif R_plant and not L_plant:
                    sx = float(d.xpos[self.bid_rf, 0])
                else:
                    sx = 0.5 * (float(d.xpos[self.bid_lf, 0]) + float(d.xpos[self.bid_rf, 0]))
                fp = float(np.clip(fp + _dcm_reverse_bias(com_xy, com_vxy, com_z, sx),
                                   -self.alip_dx_cap if self.alip_dx_cap > 1e-9 else -0.08, 0.0))
            du_raw = float(fp)
            du_prev = float(self._alip_du_fp) if self._alip_du_fp else du_raw
            if side_chg or self._alip_replan_idx < 0:
                du = du_raw
                jump = 0.0
            else:
                du = (1.0 - self.alip_smooth) * du_raw + self.alip_smooth * du_prev
                jump = abs(du - du_prev)
            self._alip_fp_prior = float(fp)
            self._alip_du_fp = float(du)
            self._alip_du_prev = float(du_prev)
            self._alip_replan_idx = int(ridx)
            self._alip_swing_side = swing_side
            self._alip_n_replan += 1
            self._alip_fp_jump_max = max(self._alip_fp_jump_max, jump)
        return float(self._alip_du_fp)

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float64).reshape(12)
        action = np.clip(action, -1.0, 1.0)
        any_clear, L_clear, R_clear = self._sole_clear()

        # T5-A ANK soft DF (conditioning) — clear late-swing only
        if self.use_ank and any_clear:
            late, side = self._late_swing_clear()
            if late and side == 0 and L_clear:
                action = action.copy()
                action[4] = float(np.clip(action[4] + 0.25, -1.0, 1.0))
            elif late and side == 1 and R_clear:
                action = action.copy()
                action[10] = float(np.clip(action[10] + 0.25, -1.0, 1.0))

        if self.clear_only_residual and not any_clear:
            action = np.zeros(12, dtype=np.float64)

        d, m = self.data, self.model
        t = float(d.time)
        a = self._gait_amp(t)
        qdes = wg.gait_targets(t, True, a)
        wg.set_ctrl(m, d, qdes, self.act_idx)
        phi_ol = (max(0.0, t - self.stand_hold) / max(wg.GAIT_T, 1e-6)) % 1.0 if a > 0 else 0.0
        phi = self._phi_enc(t, a) if self.rev_phase else phi_ol
        lat = wg.lateral_com_target(phi) if a > 0 else 0.0

        settle = getattr(wg, "ANK_COP_BIAS_SETTLE_S", 0.55)
        if self.cop_bias_xy is None and t < settle:
            com = np.asarray(d.subtree_com[0, :2], dtype=np.float64)
            try:
                support, _, _, _ = wg._support_feet(
                    m, d, self.bid_lf, self.bid_rf, self.gid_floor, None,
                )
                self.cop_bias_samples.append(com - support)
            except Exception:
                pass
            if t >= settle - 1.5 / wg.CTRL_HZ and self.cop_bias_samples:
                self.cop_bias_xy = np.mean(np.stack(self.cop_bias_samples, axis=0), axis=0)

        if a > 0.01:
            bias = self.cop_bias_xy
            if bias is None and self.cop_bias_samples:
                bias = np.mean(np.stack(self.cop_bias_samples, axis=0), axis=0)
            try:
                wg.ankle_cop_servo(
                    m, d, self.act_idx, a,
                    self.bid_lf, self.bid_rf, self.gid_floor,
                    lat=lat if a > 0.05 else None, bias_xy=bias,
                )
            except Exception:
                pass

        if self.use_cp and a > 0.05:
            try:
                wg.apply_cp_swing_placement(
                    m, d, self.act_idx, phi, a, self.bid_lf, self.bid_rf,
                )
            except Exception:
                pass

        if self.use_vik and a > 0.05:
            try:
                wg.apply_stance_jacobian_vik(
                    m, d, self.act_idx, phi, a,
                    self.bid_lf, self.bid_rf, self.gid_floor,
                )
            except Exception:
                pass

        # ---- T5-B MODEL BASE: ALIP/DCM foothold + ankle prior (clear-only) ----
        F_cmd = self._update_alip_state(phi, a)
        if self.use_alip and a > 0.05 and any_clear and abs(F_cmd) > 1e-6:
            ank = _apply_alip_fp_track_env(
                m, d, self.act_idx, phi, a,
                self.bid_lf, self.bid_rf, F_cmd, gain=self.alip_track_gain,
            )
            self._ank_cmd_cum += float(ank)
        # Ankle-NMPC soft TD prior (clear late-swing) — stronger than T5-A conditioning
        if self.use_ank_nmpc and any_clear and a > 0.05:
            late, side = self._late_swing_clear()
            if late and side >= 0:
                pref = "l_" if side == 0 else "r_"
                an_a = f"{pref}ank_pitch_pos"
                if an_a in self.act_idx:
                    # DF soft-TD bias on ctrl (not planted)
                    d.ctrl[self.act_idx[an_a]] = float(np.clip(
                        d.ctrl[self.act_idx[an_a]] + 0.08, -1.2, 1.2))
                    self._ank_cmd_cum += 0.08

        # Residual on top of model base
        if a > 0.05:
            delta = action * self.action_scale
            for i, ai in enumerate(self.leg_act):
                d.ctrl[ai] = float(np.clip(d.ctrl[ai] + delta[i], -CTRL_LIM, CTRL_LIM))

        for _ in range(self.steps_per_ctrl):
            d.qfrc_applied[:] = 0
            d.xfrc_applied[:] = 0
            if self.use_plant and a > 0:
                if any_clear:
                    wg.stance_plant(
                        m, d, phi_ol, a,
                        self.bid_lf, self.bid_rf,
                        self.gid_lfoot, self.gid_rfoot, self.gid_floor,
                    )
            mj.mj_step(m, d)
            self.sat_steps += 1
            for ai in self.leg_act:
                lim = abs(float(m.actuator_forcerange[ai, 1]))
                if lim > 1e-9 and abs(float(d.actuator_force[ai])) >= 0.98 * lim:
                    self.sat_count += 1

        self.k += 1
        cL = self._contact(self.bid_lf)
        cR = self._contact(self.bid_rf)
        vxL = float(d.cvel[self.bid_lf][3])
        vxR = float(d.cvel[self.bid_rf][3])
        R = d.xmat[self.bid_body].reshape(3, 3)
        up_z = float(R[2, 2])
        jL = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "l_hip_pitch")
        jR = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "r_hip_pitch")
        self.log["t"].append(float(d.time))
        self.log["cL"].append(1.0 if cL else 0.0)
        self.log["cR"].append(1.0 if cR else 0.0)
        self.log["zL"].append(float(d.xpos[self.bid_lf, 2]))
        self.log["zR"].append(float(d.xpos[self.bid_rf, 2]))
        self.log["vxL"].append(vxL)
        self.log["vxR"].append(vxR)
        self.log["amp"].append(a)
        self.log["up"].append(1.0 if up_z > 0.5 else 0.0)
        self.log["x"].append(float(d.qpos[0]))
        self.log["hipL"].append(float(d.qpos[m.jnt_qposadr[jL]]))
        self.log["hipR"].append(float(d.qpos[m.jnt_qposadr[jR]]))
        self._hip_buf_L.append(float(d.qpos[m.jnt_qposadr[jL]]))
        self._hip_buf_R.append(float(d.qpos[m.jnt_qposadr[jR]]))
        if len(self._hip_buf_L) > 100:
            self._hip_buf_L = self._hip_buf_L[-100:]
            self._hip_buf_R = self._hip_buf_R[-100:]

        # --- T5-B reward: skate-primary attack + ALIP track + cf floor ---
        reward = 0.0
        com_xy = d.subtree_com[0, :2].copy()
        dxy = float(com_xy[0] - self.prev_com_xy[0]) if self.prev_com_xy is not None else 0.0
        self.prev_com_xy = com_xy
        rev_step = -dxy
        any_clear, L_clear, R_clear = self._sole_clear()

        x_now = float(d.qpos[0])
        if self._prev_x is not None:
            step_back = max(0.0, self._prev_x - x_now)
            self._dx_back_max = max(self._dx_back_max, step_back)
            if self.use_dxb and a > 0.2 and step_back > self.dxb_cap:
                reward -= 4.0 * (step_back - self.dxb_cap)
        self._prev_x = x_now

        if a > 0.2:
            self._gait_steps += 1
            # SKATE PRIMARY — heavier than T5-A (bout1 p95 wall attack)
            sk = 0.0
            if cL and not cR:
                sk = abs(vxL)
                reward -= 16.0 * min(sk, 0.5)
            elif cR and not cL:
                sk = abs(vxR)
                reward -= 16.0 * min(sk, 0.5)
            elif cL and cR:
                sk = 0.5 * (abs(vxL) + abs(vxR))
                reward -= 7.0 * (min(abs(vxL), 0.5) + min(abs(vxR), 0.5))
            if cL or cR:
                self._skate_buf.append(float(sk))
                if sk > 0.08:
                    reward -= 24.0 * (sk - 0.08) ** 2
                if sk > 0.18:
                    reward -= 50.0 * (sk - 0.18)

            if any_clear:
                self._clear_steps += 1
                self._dx_clear += max(0.0, rev_step)
                reward += 3.0 * rev_step
                z_sw = max(float(d.xpos[self.bid_lf, 2]), float(d.xpos[self.bid_rf, 2]))
                reward += 1.5 * max(0.0, z_sw - (SOLE_OFFSET + FOOT_LIFT_AR))

                # Mild credit for nonzero reverse foothold prior (engaged model-base)
                if abs(self._alip_du_fp) > 1e-4:
                    reward += 0.15
                # Prefer FAIL foothold-jump tip/skate
                if self._alip_fp_jump_max > 0.025:
                    reward -= 2.0 * (self._alip_fp_jump_max - 0.025)

                if self.use_slr:
                    late, side = self._late_swing_clear()
                    if late and side >= 0:
                        v_sw = vxL if side == 0 else vxR
                        target = 0.03
                        err = abs(float(v_sw) - target)
                        reward -= 8.0 * min(err, 0.4)
                        if abs(float(v_sw) - target) < 0.05:
                            reward += 0.4
                        self._v_sw_tang_preTD = float(v_sw)

                if self.use_ank:
                    late, side = self._late_swing_clear()
                    if late and side >= 0:
                        jn = "l_ank_pitch" if side == 0 else "r_ank_pitch"
                        jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
                        ank = float(d.qpos[m.jnt_qposadr[jid]])
                        df_bonus = 0.3 * max(0.0, min(ank, 0.35))
                        reward += df_bonus
                        self._ank_df_bonus_cum += df_bonus
            else:
                plant_prog = max(0.0, rev_step)
                self._dx_plant += plant_prog
                self._plant_prog_cum += plant_prog
                self._plant_prog_any = max(self._plant_prog_any, plant_prog)
                reward -= 25.0 * plant_prog
                if plant_prog > 0.02:
                    reward -= 5.0
                if self._plant_prog_cum > 0.05:
                    reward -= 2.0

            reward += 0.12 * max(0.0, up_z)
            if up_z > 0.5:
                self._tip_run += self.ctrl_dt
                self._tip_max = max(self._tip_max, self._tip_run)
            else:
                self._tip_run = 0.0
            reward -= 0.015 * float(np.dot(action, action))

            dx_done = max(0.0, self.x0 - float(d.qpos[0]))
            if dx_done >= self.target_dx - 1e-3:
                reward += 1.0

        terminated = False
        truncated = False
        info: dict[str, Any] = {}
        z = float(d.qpos[2])
        if (not np.isfinite(d.qpos).all()) or z < 0.10 or z > 0.50 or up_z < 0.25:
            reward -= 25.0
            terminated = True
            info["fall"] = True
        if self.k >= self.n_ctrl:
            truncated = True
            cf = self._dx_clear / max(self._dx_clear + self._dx_plant, 1e-6)
            if cf >= 0.55 - 1e-9:
                reward += 2.0 * cf
            else:
                reward -= 6.0 * (0.55 - cf)
            if self._skate_buf:
                arr = np.asarray(self._skate_buf, dtype=np.float64)
                sk_mean = float(np.mean(arr))
                sk_p95 = float(np.percentile(arr, 95))
                reward -= 14.0 * max(0.0, sk_mean - 0.08)
                reward -= 30.0 * max(0.0, sk_p95 - 0.18)  # bout1 wall attack
                if sk_mean <= 0.08 + 1e-9 and sk_p95 <= 0.18 + 1e-9:
                    reward += 10.0
                info["skate_mean"] = sk_mean
                info["skate_p95"] = sk_p95
            if self._tip_max < 8.0:
                reward -= 3.0
            if self._plant_prog_any > 0.02 or self._plant_prog_cum > 0.05:
                reward -= 8.0
            info["clear_frac"] = float(cf)
            info["dx_clear"] = float(self._dx_clear)
            info["dx_plant"] = float(self._dx_plant)
            info["tip_max"] = float(self._tip_max)
            info["plant_prog_cum"] = float(self._plant_prog_cum)
            info["plant_prog_any"] = float(self._plant_prog_any)
            info["target_dx"] = float(self.target_dx)
            info["vx_cmd"] = float(self.vx_cmd)
            info["v_sw_tang_preTD"] = float(self._v_sw_tang_preTD)
            info["dx_back_max"] = float(self._dx_back_max)
            info["ank_df_bonus_cum"] = float(self._ank_df_bonus_cum)
            info["fp_prior"] = float(self._alip_fp_prior)
            info["du_fp"] = float(self._alip_du_fp)
            info["n_replan"] = int(self._alip_n_replan)
            info["fp_jump"] = float(self._alip_fp_jump_max)
            info["alip_or_dcm"] = "ALIP+DCM" if (self.use_alip and self.use_dcm) else (
                "ALIP" if self.use_alip else ("DCM" if self.use_dcm else "none"))
            info["ankle_cmd"] = float(self._ank_cmd_cum)

        info["sat_rate"] = self.sat_count / max(1, self.sat_steps * len(self.leg_act))
        info["dx"] = float(self.x0 - d.qpos[0])
        info["gait_t"] = self.gait_t
        info["fall"] = bool(info.get("fall", False))
        return self._obs(), float(reward), terminated, truncated, info


def make_t5b_env(
    gait_t: float = 0.75,
    rank: int = 0,
    seed: int = 0,
    **kwargs,
):
    def _thunk():
        env = AinexT5BModelBaseEnv(gait_t=gait_t, seed=seed + rank, **kwargs)
        env.reset(seed=seed + rank)
        return env
    return _thunk



# ---------------------------------------------------------------------------
# T5-C — reverse teacher BC + multi-seed (finish T4 opts A+E), skate-honest
# Cospec: docs/GATE_Q_AI_COSPEC_T5C.md  ·  ≠ R*/S1–S3/T5-A/T5-B twins on frozen
# Teacher NEVER credits plant XY as stepped. Clear-only residual. cancel×0.70.
# ---------------------------------------------------------------------------

def _csf_teacher_foothold(com_xy, com_vxy, com_z, vx_cmd, *, dx_cap=0.03, dx_remain=None):
    """CSF50-flavored reverse capture foothold for teacher demos (≤0). Clear-only use."""
    g = 9.81
    z = max(float(com_z), 0.12)
    omega = max(math.sqrt(g / z), 0.5)
    vx = float(com_vxy[0])
    vxc = float(vx_cmd)
    # Missura-ish capture: F ≈ vx/ω + bias toward commanded reverse
    F = vxc / max(omega * 0.40, 0.5) + 0.30 * (vx - vxc) / omega
    if dx_remain is not None and float(dx_remain) > 1e-4:
        share = min(float(dx_remain) * 0.10, abs(float(dx_cap)) if float(dx_cap) > 1e-9 else 0.06)
        F = 0.70 * F - 0.30 * share
    # Placo-style short reverse: tighter cap than T5-B (0.03 default)
    cap = abs(float(dx_cap)) if float(dx_cap) > 1e-9 else 0.03
    F = float(np.clip(F, -cap, 0.0))
    return F, omega


class AinexT5CTeacherEnv(AinexT5BModelBaseEnv):
    """T5-C: reverse teacher BC outer + residual RL. Same 41-D obs.

    Teacher source: CSF50-flavored reverse foothold + Placo short dx_back +
    optional Walk-This-Way-style footstep timing (swing-window only).
    NEVER credits plant XY. Clear sole ≥2 cm for stride credit.
    Attack: bout0 skate p95 + plant-cam ε while holding bout1 skate/cf/apps.
    May carry T5-B ALIP/DCM/ANK + T5-A SLR/DXB as conditioning.
    """

    TEACHER_SRC = "CSF50_oracle+Placo_dx0.03+footstep_timing"

    def __init__(
        self,
        *,
        teacher_dx_cap: float = 0.030,
        teacher_gain: float = 0.55,
        use_footstep_timing: bool = True,
        footstep_s_lo: float = 0.18,
        footstep_s_hi: float = 0.88,
        eps_any_pen: float = 40.0,
        eps_cum_pen: float = 25.0,
        skate_bout0_boost: float = 1.35,
        **kwargs,
    ):
        # Keep T5-B model-base + T5-A conditioning as secondary inside NEW weights
        kwargs.setdefault("use_alip", True)
        kwargs.setdefault("use_dcm", True)
        kwargs.setdefault("use_ank_nmpc", True)
        kwargs.setdefault("use_slr", True)
        kwargs.setdefault("use_ank", True)
        kwargs.setdefault("use_dxb", True)
        # Tighter Placo-style dxb than T5-B default
        kwargs.setdefault("dxb_cap", 0.030)
        kwargs.setdefault("alip_dx_cap", 0.030)
        super().__init__(**kwargs)
        self.teacher_dx_cap = float(teacher_dx_cap)
        self.teacher_gain = float(teacher_gain)
        self.use_footstep_timing = bool(use_footstep_timing)
        self.footstep_s_lo = float(footstep_s_lo)
        self.footstep_s_hi = float(footstep_s_hi)
        self.eps_any_pen = float(eps_any_pen)
        self.eps_cum_pen = float(eps_cum_pen)
        self.skate_bout0_boost = float(skate_bout0_boost)
        self._teacher_n = 0
        self._teacher_plant_reject = 0
        self._teacher_F_sum = 0.0
        self._bc_mix = 0.0  # 1=pure teacher action blend into residual during BC warmup
        self._last_teacher_act = np.zeros(12, dtype=np.float64)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        obs, info = super().reset(seed=seed, options=options)
        self._teacher_n = 0
        self._teacher_plant_reject = 0
        self._teacher_F_sum = 0.0
        self._last_teacher_act = np.zeros(12, dtype=np.float64)
        return obs, info

    def set_bc_mix(self, mix: float) -> None:
        """During BC warmup, mix∈[0,1] blends teacher action over policy action."""
        self._bc_mix = float(np.clip(mix, 0.0, 1.0))

    def teacher_action(self) -> np.ndarray:
        """Oracle reverse residual — clear-only. Prefer FAIL if would credit plant XY."""
        act = np.zeros(12, dtype=np.float64)
        any_clear, L_clear, R_clear = self._sole_clear()
        if not any_clear:
            self._teacher_plant_reject += 1
            self._last_teacher_act = act
            return act  # ZERO planted soft-XY — teacher honesty

        d, m = self.data, self.model
        t = float(d.time)
        a = self._gait_amp(t)
        if a < 0.05:
            self._last_teacher_act = act
            return act

        phi_ol = (max(0.0, t - self.stand_hold) / max(wg.GAIT_T, 1e-6)) % 1.0
        phi = self._phi_enc(t, a) if self.rev_phase else phi_ol

        # Footstep timing: only mid-swing windows (Walk-This-Way-style)
        ds_frac = float(np.clip(wg.DS_S / max(wg.GAIT_T, 1e-3), 0.08, 0.55))
        ds_end = 0.5 + 0.5 * ds_frac
        swing_side = None
        s_sw = None
        for side, clr in (("L", L_clear), ("R", R_clear)):
            if not clr:
                continue
            pl = wg.phase_leg(phi, side)
            if pl < ds_end:
                continue
            s = (pl - ds_end) / max(1e-6, 1.0 - ds_end)
            if self.use_footstep_timing:
                if not (self.footstep_s_lo <= s <= self.footstep_s_hi):
                    continue
            elif not (0.05 <= s <= 0.98):
                continue
            swing_side = side
            s_sw = float(s)
            break
        if swing_side is None:
            self._last_teacher_act = act
            return act

        try:
            com_xy, com_vxy, com_z = wg.estimate_com_state(m, d)
        except Exception:
            com_xy = d.subtree_com[0, :2].copy()
            com_vxy = np.zeros(2)
            com_z = float(d.qpos[2])
        dx_done = max(0.0, self.x0 - float(d.qpos[0]))
        dx_rem = max(0.0, self.target_dx - dx_done)
        F, _om = _csf_teacher_foothold(
            com_xy, com_vxy, com_z, self.vx_cmd,
            dx_cap=self.teacher_dx_cap, dx_remain=dx_rem,
        )
        # DCM overshoot trim (conditioning)
        if self.use_dcm:
            zL = float(d.xpos[self.bid_lf, 2]); zR = float(d.xpos[self.bid_rf, 2])
            cL = zL - SOLE_OFFSET; cR = zR - SOLE_OFFSET
            L_plant = not ((cL - PLANT_REST_CLEAR) >= FOOT_LIFT_AR - 1e-9 and cL >= 0.03 - 1e-9)
            R_plant = not ((cR - PLANT_REST_CLEAR) >= FOOT_LIFT_AR - 1e-9 and cR >= 0.03 - 1e-9)
            if L_plant and not R_plant:
                sx = float(d.xpos[self.bid_lf, 0])
            elif R_plant and not L_plant:
                sx = float(d.xpos[self.bid_rf, 0])
            else:
                sx = 0.5 * (float(d.xpos[self.bid_lf, 0]) + float(d.xpos[self.bid_rf, 0]))
            F = float(np.clip(F + _dcm_reverse_bias(com_xy, com_vxy, com_z, sx),
                              -self.teacher_dx_cap, 0.0))

        # Map foothold F (≤0, land further −X) → hip_pitch / ank_pitch residual
        # indices: 2=l_hip_pitch, 4=l_ank_pitch, 8=r_hip_pitch, 10=r_ank_pitch
        # sign convention matches reverse residual: +hip drives reverse for that side
        mag = float(np.clip(abs(F) / max(self.teacher_dx_cap, 1e-6), 0.0, 1.0))
        # Swing blend mid-window peak
        sw = 1.0
        if s_sw is not None:
            # triangle peak at mid of allowed window
            mid = 0.5 * (self.footstep_s_lo + self.footstep_s_hi)
            half = max(0.5 * (self.footstep_s_hi - self.footstep_s_lo), 1e-3)
            sw = float(np.clip(1.0 - abs(s_sw - mid) / half, 0.15, 1.0))
        amp = self.teacher_gain * mag * sw
        if swing_side == "L":
            act[2] = +amp          # l_hip_pitch reverse
            act[4] = +0.45 * amp   # soft DF ank
            act[3] = +0.20 * amp   # mild knee
        else:
            act[8] = +amp
            act[10] = +0.45 * amp
            act[9] = +0.20 * amp

        # Late-swing SLR: nudge tangential match (small)
        late, side = self._late_swing_clear()
        if late and side >= 0:
            if side == 0:
                act[4] = float(np.clip(act[4] + 0.15, -1.0, 1.0))
            else:
                act[10] = float(np.clip(act[10] + 0.15, -1.0, 1.0))

        act = np.clip(act, -1.0, 1.0)
        self._teacher_n += 1
        self._teacher_F_sum += float(F)
        self._last_teacher_act = act.copy()
        return act

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float64).reshape(12)
        action = np.clip(action, -1.0, 1.0)
        # BC mix: blend teacher over incoming action (clear-only teacher already zeros plant)
        if self._bc_mix > 1e-9:
            t_act = self.teacher_action()
            action = (1.0 - self._bc_mix) * action + self._bc_mix * t_act
            action = np.clip(action, -1.0, 1.0)

        obs, reward, terminated, truncated, info = super().step(action)

        # Extra skate-bout0 + plant-cam ε Prefer FAIL shaping (on top of T5-B)
        if not terminated and self._gait_steps > 0:
            # Heavier skate p95 proxy during episode (bout0 wall attack)
            if self._skate_buf:
                sk = float(self._skate_buf[-1])
                if sk > 0.08:
                    reward -= (self.skate_bout0_boost - 1.0) * 12.0 * (sk - 0.08) ** 2
                if sk > 0.18:
                    reward -= (self.skate_bout0_boost - 1.0) * 30.0 * (sk - 0.18)
            # Stronger plant-progress ε Prefer FAIL (proxy for plant-cam steal)
            if self._plant_prog_any > 0.02:
                reward -= self.eps_any_pen * (self._plant_prog_any - 0.02)
            if self._plant_prog_cum > 0.05:
                reward -= self.eps_cum_pen * (self._plant_prog_cum - 0.05)

        if truncated or terminated:
            info["teacher_src"] = self.TEACHER_SRC
            info["teacher_n"] = int(self._teacher_n)
            info["teacher_plant_reject"] = int(self._teacher_plant_reject)
            info["teacher_F_avg"] = (
                float(self._teacher_F_sum) / max(1, self._teacher_n)
            )
            info["bc_mix"] = float(self._bc_mix)
            # Prefer FAIL flag if teacher somehow banked plant (should be 0)
            info["teacher_plant_xy_credit"] = False  # by construction
            if self._skate_buf:
                arr = np.asarray(self._skate_buf, dtype=np.float64)
                sk_p95 = float(np.percentile(arr, 95))
                # Extra terminal bout0-style p95 attack
                reward -= 18.0 * max(0.0, sk_p95 - 0.18) * self.skate_bout0_boost
                if sk_p95 <= 0.18 + 1e-9 and float(np.mean(arr)) <= 0.08 + 1e-9:
                    reward += 6.0
            if self._plant_prog_any > 0.02 or self._plant_prog_cum > 0.05:
                reward -= 12.0  # ε Prefer FAIL terminal

        return obs, float(reward), terminated, truncated, info


def make_t5c_env(
    gait_t: float = 0.75,
    rank: int = 0,
    seed: int = 0,
    **kwargs,
):
    def _thunk():
        env = AinexT5CTeacherEnv(gait_t=gait_t, seed=seed + rank, **kwargs)
        env.reset(seed=seed + rank)
        return env
    return _thunk


# ---------------------------------------------------------------------------
# T5-D1 — OUTER-PRIMARY + CLEAR-CORRECTOR (≠ T5-A/B/C residual-primary)
# Cospec: docs/GATE_Q_AI_COSPEC_T5D.md  ·  Placo missing → OUTER=CSF+ALIP
# Outer owns F,T + footholds from −Vx / capture. Residual = corrector only
# while sole clear ≥2 cm (mid-swing + late-swing TDVM/SLR). Planted: cancel×0.70.
# Prefer FAIL if outer idle and residual owns polarity.
# ---------------------------------------------------------------------------

def _csf_outer_FT(com_xy, com_vxy, com_z, vx_cmd, *, T_nom=0.75, dx_cap=0.035):
    """CSF50-shaped reverse capture F + support-exchange T. F≤0. Clear-only caller."""
    g = 9.81
    z = max(float(com_z), 0.12)
    omega = max(math.sqrt(g / z), 0.5)
    vx = float(com_vxy[0])
    vy = float(com_vxy[1])
    vxc = float(vx_cmd)
    y_ref = 0.04
    T_lat = (1.0 / omega) * math.asinh(max(1e-6, abs(vy) / max(omega * y_ref, 1e-3)))
    T = float(T_nom) * (0.85 + 0.15 * min(T_lat / 0.40, 1.5))
    T = float(np.clip(T, 0.50, 1.10))
    F_raw = vxc * T + (vx - vxc) / omega * 0.35
    cap = abs(float(dx_cap)) if float(dx_cap) > 1e-9 else 0.035
    F = float(np.clip(F_raw, -cap, 0.0))
    return F, T, omega


def _apply_outer_fp_track(m, d, act_idx, phi, amp, bid_lf, bid_rf, F_cmd, gain=0.14):
    """Outer PRIMARY clear-only foothold track. Stronger than T5-B residual base."""
    if amp < 0.05 or abs(float(F_cmd)) < 1e-6:
        return 0.0
    rest = PLANT_REST_CLEAR
    ar_m = FOOT_LIFT_AR
    abs_m = 0.03
    ds_frac = float(np.clip(wg.DS_S / max(wg.GAIT_T, 1e-3), 0.08, 0.55))
    ds_end = 0.5 + 0.5 * ds_frac
    gain = max(float(gain), 0.05)
    zL = float(d.xpos[bid_lf, 2]); zR = float(d.xpos[bid_rf, 2])
    cL = zL - SOLE_OFFSET; cR = zR - SOLE_OFFSET
    L_plant = not ((cL - rest) >= ar_m - 1e-9 and cL >= abs_m - 1e-9)
    R_plant = not ((cR - rest) >= ar_m - 1e-9 and cR >= abs_m - 1e-9)
    if L_plant and not R_plant:
        support_x = float(d.xpos[bid_lf, 0])
    elif R_plant and not L_plant:
        support_x = float(d.xpos[bid_rf, 0])
    else:
        support_x = 0.5 * (float(d.xpos[bid_lf, 0]) + float(d.xpos[bid_rf, 0]))
    des_x = support_x + float(F_cmd)
    applied = 0.0
    for side, bid, sign in (("L", bid_lf, -1.0), ("R", bid_rf, +1.0)):
        z = float(d.xpos[bid, 2])
        clr = z - SOLE_OFFSET
        if (clr - rest) < ar_m - 1e-9 or clr < abs_m - 1e-9:
            continue
        p_leg = wg.phase_leg(phi, side)
        if p_leg < ds_end:
            continue
        s = (p_leg - ds_end) / max(1e-6, 1.0 - ds_end)
        if s < 0.12 or s > 0.96:
            continue
        sw = wg.swing_blend(min(1.0, s / 0.7))
        foot_x = float(d.xpos[bid, 0])
        err = des_x - foot_x
        d_fwd = float(np.clip(err / gain, -0.35, 0.35)) * float(amp) * sw
        pref = "l_" if side == "L" else "r_"
        an = f"{pref}hip_pitch_pos"
        if an in act_idx:
            d.ctrl[act_idx[an]] = float(np.clip(d.ctrl[act_idx[an]] + sign * d_fwd, -1.5, 1.5))
            applied += abs(d_fwd)
        an_a = f"{pref}ank_pitch_pos"
        if an_a in act_idx:
            d_ank = sign * 0.45 * d_fwd
            d.ctrl[act_idx[an_a]] = float(np.clip(d.ctrl[act_idx[an_a]] + d_ank, -1.2, 1.2))
            applied += abs(d_ank)
    return float(applied)


class AinexT5D1OuterPrimaryEnv(AinexT5BModelBaseEnv):
    """T5-D1: reverse-native outer PRIMARY (CSF+ALIP) + clear-only residual corrector.

    Outer owns support-exchange F,T and next foothold targets from −Vx / capture.
    Residual = corrector only (mid-swing track + late-swing TDVM/SLR); authority
    only while sole clear ≥2 cm. Planted: cancel×0.70 + vel-oppose — zero soft-XY.
    OUTER=CSF+ALIP (Placo not in stack). Prefer FAIL if outer idle + residual owns.
    """

    OUTER_SRC = "CSF+ALIP"  # Placo unavailable — in-process CSF capture + ALIP-MPC
    FAMILY = "T5D1_OUTER-PRIMARY+CLEAR-CORRECTOR"

    def __init__(
        self,
        *,
        outer_dx_cap: float = 0.035,
        outer_track_gain: float = 0.14,
        corrector_scale: float = 0.028,  # residual demoted vs T5-A/B/C 0.07
        residual_primary_forbid: bool = True,
        **kwargs,
    ):
        # Keep ALIP/DCM/SLR/ANK as outer co-primary + late-swing corrector tools
        kwargs.setdefault("use_alip", True)
        kwargs.setdefault("use_dcm", True)
        kwargs.setdefault("use_ank_nmpc", True)
        kwargs.setdefault("use_slr", True)
        kwargs.setdefault("use_ank", True)
        kwargs.setdefault("use_dxb", True)
        kwargs.setdefault("dxb_cap", 0.035)
        kwargs.setdefault("alip_dx_cap", 0.035)
        kwargs.setdefault("alip_track_gain", float(outer_track_gain))
        kwargs.setdefault("n_replan", 3)
        kwargs.setdefault("alip_smooth", 0.35)
        # Demote residual action_scale to corrector band
        kwargs.setdefault("action_scale", float(corrector_scale))
        kwargs.setdefault("clear_only_residual", True)
        super().__init__(**kwargs)
        self.outer_dx_cap = float(outer_dx_cap)
        self.outer_track_gain = float(outer_track_gain)
        self.corrector_scale = float(corrector_scale)
        self.residual_primary_forbid = bool(residual_primary_forbid)
        self._outer_F = 0.0
        self._outer_T = float(kwargs.get("gait_t", 0.75) or 0.75)
        self._outer_n = 0
        self._outer_F_sum = 0.0
        self._outer_T_sum = 0.0
        self._outer_applied_sum = 0.0
        self._outer_footholds: list[float] = []
        self._residual_rms_sum = 0.0
        self._residual_n = 0
        self._outer_idle_steps = 0
        self._gait_steps_outer = 0
        self._phi_outer_t0 = None
        self._last_outer_F = 0.0
        self._last_outer_T = float(self._outer_T)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        obs, info = super().reset(seed=seed, options=options)
        self._outer_F = 0.0
        self._outer_T = float(self.gait_t)
        self._outer_n = 0
        self._outer_F_sum = 0.0
        self._outer_T_sum = 0.0
        self._outer_applied_sum = 0.0
        self._outer_footholds = []
        self._residual_rms_sum = 0.0
        self._residual_n = 0
        self._outer_idle_steps = 0
        self._gait_steps_outer = 0
        self._phi_outer_t0 = None
        self._last_outer_F = 0.0
        self._last_outer_T = float(self._outer_T)
        # Ensure action_scale stays in corrector band (forbid residual-primary collapse)
        if self.residual_primary_forbid and self.action_scale > 0.04:
            self.action_scale = float(self.corrector_scale)
        return obs, info

    def _update_outer_primary(self, phi: float, a: float) -> tuple[float, float]:
        """CSF+ALIP outer PRIMARY: compute F,T and apply foothold track. Clear-only."""
        if a < 0.05:
            return 0.0, float(self._outer_T)
        d, m = self.data, self.model
        any_clear, L_clear, R_clear = self._sole_clear()
        if not any_clear:
            self._outer_idle_steps += 1
            return float(self._outer_F), float(self._outer_T)

        try:
            com_xy, com_vxy, com_z = wg.estimate_com_state(m, d)
        except Exception:
            com_xy = d.subtree_com[0, :2].copy()
            com_vxy = np.zeros(2)
            com_z = float(d.qpos[2])

        # CSF capture F + support-exchange T (outer owns polarity)
        F_csf, T_csf, _om = _csf_outer_FT(
            com_xy, com_vxy, com_z, self.vx_cmd,
            T_nom=float(self.gait_t), dx_cap=self.outer_dx_cap,
        )
        # ALIP-MPC co-primary foothold (blend with CSF)
        dx_done = max(0.0, self.x0 - float(d.qpos[0]))
        dx_rem = max(0.0, self.target_dx - dx_done)
        F_alip, _ = _alip_foothold_prior(
            com_xy, com_vxy, com_z, self.vx_cmd,
            dx_cap=self.outer_dx_cap, dx_remain=dx_rem,
        )
        if self.use_dcm:
            zL = float(d.xpos[self.bid_lf, 2]); zR = float(d.xpos[self.bid_rf, 2])
            cL = zL - SOLE_OFFSET; cR = zR - SOLE_OFFSET
            L_plant = not ((cL - PLANT_REST_CLEAR) >= FOOT_LIFT_AR - 1e-9 and cL >= 0.03 - 1e-9)
            R_plant = not ((cR - PLANT_REST_CLEAR) >= FOOT_LIFT_AR - 1e-9 and cR >= 0.03 - 1e-9)
            if L_plant and not R_plant:
                sx = float(d.xpos[self.bid_lf, 0])
            elif R_plant and not L_plant:
                sx = float(d.xpos[self.bid_rf, 0])
            else:
                sx = 0.5 * (float(d.xpos[self.bid_lf, 0]) + float(d.xpos[self.bid_rf, 0]))
            F_alip = float(np.clip(
                F_alip + _dcm_reverse_bias(com_xy, com_vxy, com_z, sx),
                -self.outer_dx_cap, 0.0))

        # Outer fusion: CSF owns T; F = 0.55 CSF + 0.45 ALIP (both ≤0 reverse)
        F = float(np.clip(0.55 * F_csf + 0.45 * F_alip, -self.outer_dx_cap, 0.0))
        T = float(T_csf)

        # Mid-swing replan gate (outer must fire during clear swing)
        ds_frac = float(np.clip(wg.DS_S / max(wg.GAIT_T, 1e-3), 0.08, 0.55))
        ds_end = 0.5 + 0.5 * ds_frac
        in_swing = False
        for side, clr in (("L", L_clear), ("R", R_clear)):
            if not clr:
                continue
            pl = wg.phase_leg(phi, side)
            if pl < ds_end:
                continue
            s = (pl - ds_end) / max(1e-6, 1.0 - ds_end)
            if 0.08 <= s <= 0.96:
                in_swing = True
                break

        if in_swing and abs(F) > 1e-5:
            applied = _apply_outer_fp_track(
                m, d, self.act_idx, phi, a,
                self.bid_lf, self.bid_rf, F, gain=self.outer_track_gain,
            )
            self._outer_n += 1
            self._outer_F_sum += float(F)
            self._outer_T_sum += float(T)
            self._outer_applied_sum += float(applied)
            self._outer_footholds.append(float(F))
            if len(self._outer_footholds) > 64:
                self._outer_footholds = self._outer_footholds[-64:]
            # Sync ALIP state for logging compatibility
            self._alip_fp_prior = float(F_alip)
            self._alip_du_fp = float(F)
            self._alip_n_replan += 1
        else:
            self._outer_idle_steps += 1
            applied = 0.0

        self._outer_F = float(F)
        self._outer_T = float(T)
        self._last_outer_F = float(F)
        self._last_outer_T = float(T)
        return float(F), float(T)

    def _gate_corrector_action(self, action: np.ndarray) -> np.ndarray:
        """Residual = corrector only: mid-swing clear + late-swing TDVM/SLR authority."""
        action = np.asarray(action, dtype=np.float64).reshape(12)
        action = np.clip(action, -1.0, 1.0)
        any_clear, L_clear, R_clear = self._sole_clear()
        if not any_clear:
            return np.zeros(12, dtype=np.float64)

        # Soft-gate: shrink residual outside mid/late swing (outer owns polarity)
        d = self.data
        t = float(d.time)
        a = self._gait_amp(t)
        if a < 0.05:
            return np.zeros(12, dtype=np.float64)
        phi_ol = (max(0.0, t - self.stand_hold) / max(wg.GAIT_T, 1e-6)) % 1.0
        # Use outer T for phase clock when engaged
        gait_t = max(float(self._outer_T or self.gait_t), 1e-3)
        if self._phi_outer_t0 is None:
            self._phi_outer_t0 = t
        phi = ((t - self._phi_outer_t0) / gait_t) % 1.0 if self.rev_phase else phi_ol
        if self.rev_phase:
            phi = self._phi_enc(t, a)

        ds_frac = float(np.clip(wg.DS_S / max(wg.GAIT_T, 1e-3), 0.08, 0.55))
        ds_end = 0.5 + 0.5 * ds_frac
        gate = 0.0
        late, side = self._late_swing_clear()
        if late:
            gate = 1.0  # full corrector for TDVM/SLR late-swing
        else:
            for sname, clr in (("L", L_clear), ("R", R_clear)):
                if not clr:
                    continue
                pl = wg.phase_leg(phi, sname)
                if pl < ds_end:
                    continue
                s = (pl - ds_end) / max(1e-6, 1.0 - ds_end)
                if 0.20 <= s <= 0.85:
                    gate = 0.55  # mid-swing track corrector only (demoted)
                    break
        if gate < 1e-6:
            return np.zeros(12, dtype=np.float64)
        return action * gate

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float64).reshape(12)
        action = np.clip(action, -1.0, 1.0)

        # Demote residual: gate to clear mid/late-swing corrector only
        action = self._gate_corrector_action(action)
        res_rms = float(np.sqrt(np.mean(action ** 2)))
        self._residual_rms_sum += res_rms
        self._residual_n += 1

        # Force corrector-scale (forbid residual-primary twin collapse)
        if self.residual_primary_forbid and self.action_scale > 0.04:
            self.action_scale = float(self.corrector_scale)

        any_clear, L_clear, R_clear = self._sole_clear()

        # Optional ANK soft DF — clear late-swing corrector only
        if self.use_ank and any_clear:
            late, side = self._late_swing_clear()
            if late and side == 0 and L_clear:
                action = action.copy()
                action[4] = float(np.clip(action[4] + 0.20, -1.0, 1.0))
            elif late and side == 1 and R_clear:
                action = action.copy()
                action[10] = float(np.clip(action[10] + 0.20, -1.0, 1.0))

        if self.clear_only_residual and not any_clear:
            action = np.zeros(12, dtype=np.float64)

        d, m = self.data, self.model
        t = float(d.time)
        a = self._gait_amp(t)
        qdes = wg.gait_targets(t, True, a)
        wg.set_ctrl(m, d, qdes, self.act_idx)
        phi_ol = (max(0.0, t - self.stand_hold) / max(wg.GAIT_T, 1e-6)) % 1.0 if a > 0 else 0.0
        # Outer T owns support-exchange period for phase when engaged
        if a > 0.05 and self._outer_n > 0 and self._outer_T > 1e-3:
            if self._phi_outer_t0 is None:
                self._phi_outer_t0 = t - self.stand_hold
            phi_outer = (max(0.0, t - self.stand_hold) / max(self._outer_T, 1e-6)) % 1.0
        else:
            phi_outer = phi_ol
        phi = self._phi_enc(t, a) if self.rev_phase else phi_outer
        lat = wg.lateral_com_target(phi) if a > 0 else 0.0

        settle = getattr(wg, "ANK_COP_BIAS_SETTLE_S", 0.55)
        if self.cop_bias_xy is None and t < settle:
            com = np.asarray(d.subtree_com[0, :2], dtype=np.float64)
            try:
                support, _, _, _ = wg._support_feet(
                    m, d, self.bid_lf, self.bid_rf, self.gid_floor, None,
                )
                self.cop_bias_samples.append(com - support)
            except Exception:
                pass
            if t >= settle - 1.5 / wg.CTRL_HZ and self.cop_bias_samples:
                self.cop_bias_xy = np.mean(np.stack(self.cop_bias_samples, axis=0), axis=0)

        if a > 0.01:
            bias = self.cop_bias_xy
            if bias is None and self.cop_bias_samples:
                bias = np.mean(np.stack(self.cop_bias_samples, axis=0), axis=0)
            try:
                wg.ankle_cop_servo(
                    m, d, self.act_idx, a,
                    self.bid_lf, self.bid_rf, self.gid_floor,
                    lat=lat if a > 0.05 else None, bias_xy=bias,
                )
            except Exception:
                pass

        if self.use_cp and a > 0.05:
            try:
                wg.apply_cp_swing_placement(
                    m, d, self.act_idx, phi, a, self.bid_lf, self.bid_rf,
                )
            except Exception:
                pass

        if self.use_vik and a > 0.05:
            try:
                wg.apply_stance_jacobian_vik(
                    m, d, self.act_idx, phi, a,
                    self.bid_lf, self.bid_rf, self.gid_floor,
                )
            except Exception:
                pass

        # ---- OUTER PRIMARY: CSF+ALIP owns F,T + footholds (BEFORE residual) ----
        if a > 0.05:
            self._gait_steps_outer += 1
            F_cmd, T_cmd = self._update_outer_primary(phi, a)
            _ = (F_cmd, T_cmd)  # logged in state

        # Late-swing ankle-NMPC soft TD (corrector band — clear only)
        if self.use_ank_nmpc and any_clear and a > 0.05:
            late, side = self._late_swing_clear()
            if late and side >= 0:
                pref = "l_" if side == 0 else "r_"
                an_a = f"{pref}ank_pitch_pos"
                if an_a in self.act_idx:
                    d.ctrl[self.act_idx[an_a]] = float(np.clip(
                        d.ctrl[self.act_idx[an_a]] + 0.06, -1.2, 1.2))
                    self._ank_cmd_cum += 0.06

        # Residual CORRECTOR on top of outer (demoted scale)
        if a > 0.05:
            delta = action * self.action_scale
            for i, ai in enumerate(self.leg_act):
                d.ctrl[ai] = float(np.clip(d.ctrl[ai] + delta[i], -CTRL_LIM, CTRL_LIM))

        for _ in range(self.steps_per_ctrl):
            d.qfrc_applied[:] = 0
            d.xfrc_applied[:] = 0
            if self.use_plant and a > 0:
                if any_clear:
                    wg.stance_plant(
                        m, d, phi_ol, a,
                        self.bid_lf, self.bid_rf,
                        self.gid_lfoot, self.gid_rfoot, self.gid_floor,
                    )
            mj.mj_step(m, d)
            self.sat_steps += 1
            for ai in self.leg_act:
                lim = abs(float(m.actuator_forcerange[ai, 1]))
                if lim > 1e-9 and abs(float(d.actuator_force[ai])) >= 0.98 * lim:
                    self.sat_count += 1

        self.k += 1
        cL = self._contact(self.bid_lf)
        cR = self._contact(self.bid_rf)
        vxL = float(d.cvel[self.bid_lf][3])
        vxR = float(d.cvel[self.bid_rf][3])
        R = d.xmat[self.bid_body].reshape(3, 3)
        up_z = float(R[2, 2])
        jL = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "l_hip_pitch")
        jR = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "r_hip_pitch")
        self.log["t"].append(float(d.time))
        self.log["cL"].append(1.0 if cL else 0.0)
        self.log["cR"].append(1.0 if cR else 0.0)
        self.log["zL"].append(float(d.xpos[self.bid_lf, 2]))
        self.log["zR"].append(float(d.xpos[self.bid_rf, 2]))
        self.log["vxL"].append(vxL)
        self.log["vxR"].append(vxR)
        self.log["amp"].append(a)
        self.log["up"].append(1.0 if up_z > 0.5 else 0.0)
        self.log["x"].append(float(d.qpos[0]))
        self.log["hipL"].append(float(d.qpos[m.jnt_qposadr[jL]]))
        self.log["hipR"].append(float(d.qpos[m.jnt_qposadr[jR]]))
        self._hip_buf_L.append(float(d.qpos[m.jnt_qposadr[jL]]))
        self._hip_buf_R.append(float(d.qpos[m.jnt_qposadr[jR]]))
        if len(self._hip_buf_L) > 100:
            self._hip_buf_L = self._hip_buf_L[-100:]
            self._hip_buf_R = self._hip_buf_R[-100:]

        # --- T5-D1 reward: outer primary engagement + skate + clear + ε Prefer FAIL ---
        reward = 0.0
        com_xy = d.subtree_com[0, :2].copy()
        dxy = float(com_xy[0] - self.prev_com_xy[0]) if self.prev_com_xy is not None else 0.0
        self.prev_com_xy = com_xy
        rev_step = -dxy
        any_clear, L_clear, R_clear = self._sole_clear()

        x_now = float(d.qpos[0])
        if self._prev_x is not None:
            step_back = max(0.0, self._prev_x - x_now)
            self._dx_back_max = max(self._dx_back_max, step_back)
            if self.use_dxb and a > 0.2 and step_back > self.dxb_cap:
                reward -= 4.0 * (step_back - self.dxb_cap)
        self._prev_x = x_now

        if a > 0.2:
            self._gait_steps += 1
            # SKATE bars
            sk = 0.0
            if cL and not cR:
                sk = abs(vxL)
                reward -= 16.0 * min(sk, 0.5)
            elif cR and not cL:
                sk = abs(vxR)
                reward -= 16.0 * min(sk, 0.5)
            elif cL and cR:
                sk = 0.5 * (abs(vxL) + abs(vxR))
                reward -= 7.0 * (min(abs(vxL), 0.5) + min(abs(vxR), 0.5))
            if cL or cR:
                self._skate_buf.append(float(sk))
                if sk > 0.08:
                    reward -= 24.0 * (sk - 0.08) ** 2
                if sk > 0.18:
                    reward -= 50.0 * (sk - 0.18)

            if any_clear:
                self._clear_steps += 1
                self._dx_clear += max(0.0, rev_step)
                reward += 3.0 * rev_step
                z_sw = max(float(d.xpos[self.bid_lf, 2]), float(d.xpos[self.bid_rf, 2]))
                reward += 1.5 * max(0.0, z_sw - (SOLE_OFFSET + FOOT_LIFT_AR))

                # Outer PRIMARY engagement credit (must not be idle)
                if abs(self._outer_F) > 1e-4 and self._outer_n > 0:
                    reward += 0.35
                # Prefer FAIL tip: residual owns polarity while outer idle
                if self._outer_n == 0 and res_rms > 0.25:
                    reward -= 3.0 * res_rms

                # Mild residual corrector regularizer (keep demoted)
                reward -= 0.04 * float(np.dot(action, action))

                if self.use_slr:
                    late, side = self._late_swing_clear()
                    if late and side >= 0:
                        v_sw = vxL if side == 0 else vxR
                        target = 0.03
                        err = abs(float(v_sw) - target)
                        reward -= 8.0 * min(err, 0.4)
                        if abs(float(v_sw) - target) < 0.05:
                            reward += 0.4
                        self._v_sw_tang_preTD = float(v_sw)

                if self.use_ank:
                    late, side = self._late_swing_clear()
                    if late and side >= 0:
                        jn = "l_ank_pitch" if side == 0 else "r_ank_pitch"
                        jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
                        ank = float(d.qpos[m.jnt_qposadr[jid]])
                        df_bonus = 0.3 * max(0.0, min(ank, 0.35))
                        reward += df_bonus
                        self._ank_df_bonus_cum += df_bonus
            else:
                # Planted: ZERO soft-XY / stride credit — cancel×0.70 path only
                plant_prog = max(0.0, rev_step)
                self._dx_plant += plant_prog
                self._plant_prog_cum += plant_prog
                self._plant_prog_any = max(self._plant_prog_any, plant_prog)
                reward -= 25.0 * plant_prog
                if plant_prog > 0.02:
                    reward -= 5.0
                if self._plant_prog_cum > 0.05:
                    reward -= 2.0

            reward += 0.12 * max(0.0, up_z)
            if up_z > 0.5:
                self._tip_run += self.ctrl_dt
                self._tip_max = max(self._tip_max, self._tip_run)
            else:
                self._tip_run = 0.0

            dx_done = max(0.0, self.x0 - float(d.qpos[0]))
            if dx_done >= self.target_dx - 1e-3:
                reward += 1.0

        terminated = False
        truncated = False
        info: dict[str, Any] = {}
        z = float(d.qpos[2])
        if (not np.isfinite(d.qpos).all()) or z < 0.10 or z > 0.50 or up_z < 0.25:
            reward -= 25.0
            terminated = True
            info["fall"] = True
        if self.k >= self.n_ctrl:
            truncated = True
            cf = self._dx_clear / max(self._dx_clear + self._dx_plant, 1e-6)
            if cf >= 0.55 - 1e-9:
                reward += 2.0 * cf
            else:
                reward -= 6.0 * (0.55 - cf)
            if self._skate_buf:
                arr = np.asarray(self._skate_buf, dtype=np.float64)
                sk_mean = float(np.mean(arr))
                sk_p95 = float(np.percentile(arr, 95))
                reward -= 14.0 * max(0.0, sk_mean - 0.08)
                reward -= 30.0 * max(0.0, sk_p95 - 0.18)
                if sk_mean <= 0.08 + 1e-9 and sk_p95 <= 0.18 + 1e-9:
                    reward += 10.0
                info["skate_mean"] = sk_mean
                info["skate_p95"] = sk_p95
            if self._tip_max < 8.0:
                reward -= 3.0
            if self._plant_prog_any > 0.02 or self._plant_prog_cum > 0.05:
                reward -= 8.0

            outer_F_avg = (
                float(self._outer_F_sum) / max(1, self._outer_n)
            )
            outer_T_avg = (
                float(self._outer_T_sum) / max(1, self._outer_n)
            )
            res_rms_avg = (
                float(self._residual_rms_sum) / max(1, self._residual_n)
            )
            outer_idle = bool(self._outer_n < 3)
            residual_owns = bool(
                outer_idle and res_rms_avg > 0.20
            ) or bool(
                self._outer_n > 0 and res_rms_avg > 0.55 and abs(outer_F_avg) < 1e-4
            )
            if residual_owns:
                reward -= 15.0  # Prefer FAIL: residual-primary collapse
            if outer_idle:
                reward -= 8.0

            info["clear_frac"] = float(cf)
            info["dx_clear"] = float(self._dx_clear)
            info["dx_plant"] = float(self._dx_plant)
            info["tip_max"] = float(self._tip_max)
            info["plant_prog_cum"] = float(self._plant_prog_cum)
            info["plant_prog_any"] = float(self._plant_prog_any)
            info["target_dx"] = float(self.target_dx)
            info["vx_cmd"] = float(self.vx_cmd)
            info["v_sw_tang_preTD"] = float(self._v_sw_tang_preTD)
            info["dx_back_max"] = float(self._dx_back_max)
            info["ank_df_bonus_cum"] = float(self._ank_df_bonus_cum)
            info["fp_prior"] = float(self._alip_fp_prior)
            info["du_fp"] = float(self._alip_du_fp)
            info["n_replan"] = int(self._alip_n_replan)
            info["fp_jump"] = float(self._alip_fp_jump_max)
            info["alip_or_dcm"] = "ALIP+DCM"
            info["ankle_cmd"] = float(self._ank_cmd_cum)
            # --- T5-D1 outer PRIMARY logs (must every score) ---
            info["outer_src"] = self.OUTER_SRC
            info["outer_F"] = float(outer_F_avg)
            info["outer_T"] = float(outer_T_avg if self._outer_n else self._outer_T)
            info["outer_n"] = int(self._outer_n)
            info["outer_applied"] = float(self._outer_applied_sum)
            info["outer_footholds"] = list(self._outer_footholds[-8:])
            info["outer_idle"] = bool(outer_idle)
            info["residual_rms"] = float(res_rms_avg)
            info["residual_owns_polarity"] = bool(residual_owns)
            info["corrector_scale"] = float(self.action_scale)
            info["family"] = self.FAMILY

        info["sat_rate"] = self.sat_count / max(1, self.sat_steps * len(self.leg_act))
        info["dx"] = float(self.x0 - d.qpos[0])
        info["gait_t"] = self.gait_t
        info["fall"] = bool(info.get("fall", False))
        info["outer_F_live"] = float(self._last_outer_F)
        info["outer_T_live"] = float(self._last_outer_T)
        # Always expose outer PRIMARY counters (Prefer FAIL if idle)
        if "outer_n" not in info:
            _on = int(self._outer_n)
            _of = float(self._outer_F_sum) / max(1, _on)
            _ot = float(self._outer_T_sum) / max(1, _on) if _on else float(self._outer_T)
            _rr = float(self._residual_rms_sum) / max(1, self._residual_n)
            info["outer_src"] = self.OUTER_SRC
            info["outer_F"] = float(_of)
            info["outer_T"] = float(_ot)
            info["outer_n"] = _on
            info["outer_applied"] = float(self._outer_applied_sum)
            info["outer_footholds"] = list(self._outer_footholds[-8:])
            info["outer_idle"] = bool(_on < 3)
            info["residual_rms"] = float(_rr)
            info["residual_owns_polarity"] = bool(_on < 3 and _rr > 0.20)
            info["corrector_scale"] = float(self.action_scale)
            info["family"] = self.FAMILY
        return self._obs(), float(reward), terminated, truncated, info


def make_t5d1_env(
    gait_t: float = 0.75,
    rank: int = 0,
    seed: int = 0,
    **kwargs,
):
    def _thunk():
        env = AinexT5D1OuterPrimaryEnv(gait_t=gait_t, seed=seed + rank, **kwargs)
        env.reset(seed=seed + rank)
        return env
    return _thunk


# ---------------------------------------------------------------------------
# T5-D2 — ICP-ΔT+ΔFOOT OUTER (≠ T5-D1 schedule twin)
# Cospec: docs/GATE_Q_AI_COSPEC_T5D2.md  ·  arXiv:1703.00477
# Keep CSF+ALIP schedule as PRIOR; ADD online ICP error → ΔT + Δfoot feedback.
# Residual = clear corrector only. Prefer FAIL if ICP loop idle / residual-primary.
# Planted: cancel×0.70 + vel-oppose — zero soft-XY. Soft-pass NEVER. H2 OUT.
# ---------------------------------------------------------------------------

def _icp_capture_point(com_xy, com_vxy, com_z):
    """Instantaneous Capture Point (CoM-proxy): ξ = x + v/ω, ω=√(g/z)."""
    g = 9.81
    z = max(float(com_z), 0.12)
    omega = max(math.sqrt(g / z), 0.5)
    icp_x = float(com_xy[0]) + float(com_vxy[0]) / omega
    icp_y = float(com_xy[1]) + float(com_vxy[1]) / omega
    return icp_x, icp_y, omega


def _icp_delta_TF(
    com_xy, com_vxy, com_z, support_x, vx_cmd, F_prior, T_prior, *,
    dx_cap=0.035, k_T=3.0, k_foot=1.4, T_lo=0.50, T_hi=1.10,
):
    """IHMC-style online ICP error → Δ support-exchange time + Δ foothold.

    Cite arXiv:1703.00477 (Atlas step timing + location from capture error).
    Retreat polarity: vx_cmd ≤ 0. Escape in −X → speed exchange (↓T) and
    expand BoS (more negative F). ≠ R4 plant-dwell flush; ≠ open-loop schedule.
    Returns: F_new, T_new, icp_x, e_icp, dT, dfoot, omega
    """
    icp_x, _icp_y, omega = _icp_capture_point(com_xy, com_vxy, com_z)
    sx = float(support_x)
    e_icp = float(icp_x) - sx  # signed offset of ICP from stance
    F_p = float(F_prior)
    T_p = float(T_prior)
    cap = abs(float(dx_cap)) if float(dx_cap) > 1e-9 else 0.035

    # Capture foothold from ICP (reverse: F ≤ 0)
    F_icp = float(np.clip(e_icp, -cap, 0.0))
    dfoot = float(np.clip(F_icp - F_p, -cap, cap))

    # Timing from escape error in commanded reverse direction
    # e_escape > 0 ⇒ ICP is further −X than planned foot → speed up
    if float(vx_cmd) < 0.0:
        e_escape = max(0.0, -(e_icp) - abs(F_p))  # excess retreatward ICP
        e_recover = max(0.0, e_icp)  # ICP toward +X (door) — slight slow
    else:
        e_escape = max(0.0, e_icp - abs(F_p))
        e_recover = max(0.0, -e_icp)
    dT = float(np.clip(-k_T * e_escape + 0.35 * k_T * e_recover, -0.28, 0.18))

    F_new = float(np.clip(F_p + dfoot, -cap, 0.0))
    T_new = float(np.clip(T_p + dT, T_lo, T_hi))
    return F_new, T_new, float(icp_x), float(e_icp), float(dT), float(dfoot), float(omega)


class AinexT5D2ICPEnv(AinexT5D1OuterPrimaryEnv):
    """T5-D2: CSF+ALIP schedule PRIOR + online ICP→ΔT+Δfoot feedback.

    Outer owns F,T + footholds. ICP loop MUST be active on retreat (log every
    score). Residual = clear corrector only (warm-start optional). Prefer FAIL
    if ICP idle / residual-primary / D1-schedule twin without ICP feedback.
    """

    OUTER_SRC = "CSF+ALIP+ICP"  # schedule prior + online ICP ΔT+Δfoot
    FAMILY = "T5D2_ICP-ΔT+ΔFOOT"
    CITE = "arXiv:1703.00477"

    def __init__(
        self,
        *,
        icp_k_T: float = 3.0,
        icp_k_foot: float = 1.4,
        icp_enabled: bool = True,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.icp_k_T = float(icp_k_T)
        self.icp_k_foot = float(icp_k_foot)
        self.icp_enabled = bool(icp_enabled)
        self._icp_err_sum = 0.0
        self._icp_dT_sum = 0.0
        self._icp_dfoot_sum = 0.0
        self._icp_n = 0
        self._icp_last_err = 0.0
        self._icp_last_dT = 0.0
        self._icp_last_dfoot = 0.0
        self._icp_last_x = 0.0
        self._schedule_F = 0.0
        self._schedule_T = float(kwargs.get("gait_t", 0.75) or 0.75)
        self._cmp_ank_cum = 0.0

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        obs, info = super().reset(seed=seed, options=options)
        self._icp_err_sum = 0.0
        self._icp_dT_sum = 0.0
        self._icp_dfoot_sum = 0.0
        self._icp_n = 0
        self._icp_last_err = 0.0
        self._icp_last_dT = 0.0
        self._icp_last_dfoot = 0.0
        self._icp_last_x = 0.0
        self._schedule_F = 0.0
        self._schedule_T = float(self.gait_t)
        self._cmp_ank_cum = 0.0
        return obs, info

    def _support_x(self) -> float:
        d = self.data
        zL = float(d.xpos[self.bid_lf, 2]); zR = float(d.xpos[self.bid_rf, 2])
        cL = zL - SOLE_OFFSET; cR = zR - SOLE_OFFSET
        L_plant = not ((cL - PLANT_REST_CLEAR) >= FOOT_LIFT_AR - 1e-9 and cL >= 0.03 - 1e-9)
        R_plant = not ((cR - PLANT_REST_CLEAR) >= FOOT_LIFT_AR - 1e-9 and cR >= 0.03 - 1e-9)
        if L_plant and not R_plant:
            return float(d.xpos[self.bid_lf, 0])
        if R_plant and not L_plant:
            return float(d.xpos[self.bid_rf, 0])
        return 0.5 * (float(d.xpos[self.bid_lf, 0]) + float(d.xpos[self.bid_rf, 0]))

    def _apply_cmp_ankle(self, e_icp: float, a: float) -> None:
        """CMP→ankle under stance honesty — not planted soft-XY (cospec CMP_ANK)."""
        if abs(e_icp) < 1e-4 or a < 0.05:
            return
        d = self.data
        # Stance feet only: oppose ICP error via mild ankle pitch (COP shift)
        zL = float(d.xpos[self.bid_lf, 2]); zR = float(d.xpos[self.bid_rf, 2])
        cL = zL - SOLE_OFFSET; cR = zR - SOLE_OFFSET
        L_clear = (cL - PLANT_REST_CLEAR) >= FOOT_LIFT_AR - 1e-9 and cL >= 0.03 - 1e-9
        R_clear = (cR - PLANT_REST_CLEAR) >= FOOT_LIFT_AR - 1e-9 and cR >= 0.03 - 1e-9
        # ank_pitch sign: positive DF; shift COP opposite ICP escape
        d_ank = float(np.clip(-0.18 * e_icp, -0.08, 0.08)) * float(a)
        for planted, pref in ((not L_clear, "l_"), (not R_clear, "r_")):
            if not planted:
                continue
            an = f"{pref}ank_pitch_pos"
            if an in self.act_idx:
                d.ctrl[self.act_idx[an]] = float(np.clip(
                    d.ctrl[self.act_idx[an]] + d_ank, -1.2, 1.2))
                self._cmp_ank_cum += abs(d_ank)

    def _update_outer_primary(self, phi: float, a: float) -> tuple[float, float]:
        """CSF+ALIP schedule PRIOR, then online ICP→ΔT+Δfoot (D2 delta)."""
        if a < 0.05:
            return 0.0, float(self._outer_T)
        d, m = self.data, self.model
        any_clear, L_clear, R_clear = self._sole_clear()
        if not any_clear:
            self._outer_idle_steps += 1
            # CMP→ankle may still act planted (stance honesty)
            if self.icp_enabled and abs(self._icp_last_err) > 1e-4:
                self._apply_cmp_ankle(self._icp_last_err, a)
            return float(self._outer_F), float(self._outer_T)

        try:
            com_xy, com_vxy, com_z = wg.estimate_com_state(m, d)
        except Exception:
            com_xy = d.subtree_com[0, :2].copy()
            com_vxy = np.zeros(2)
            com_z = float(d.qpos[2])

        # --- Schedule PRIOR (same CSF+ALIP fusion as T5-D1) ---
        F_csf, T_csf, _om = _csf_outer_FT(
            com_xy, com_vxy, com_z, self.vx_cmd,
            T_nom=float(self.gait_t), dx_cap=self.outer_dx_cap,
        )
        dx_done = max(0.0, self.x0 - float(d.qpos[0]))
        dx_rem = max(0.0, self.target_dx - dx_done)
        F_alip, _ = _alip_foothold_prior(
            com_xy, com_vxy, com_z, self.vx_cmd,
            dx_cap=self.outer_dx_cap, dx_remain=dx_rem,
        )
        if self.use_dcm:
            sx = self._support_x()
            F_alip = float(np.clip(
                F_alip + _dcm_reverse_bias(com_xy, com_vxy, com_z, sx),
                -self.outer_dx_cap, 0.0))
        F_sched = float(np.clip(0.55 * F_csf + 0.45 * F_alip, -self.outer_dx_cap, 0.0))
        T_sched = float(T_csf)
        self._schedule_F = F_sched
        self._schedule_T = T_sched

        # --- ONLINE ICP feedback (D2 — Prefer FAIL if this stays idle) ---
        sx = self._support_x()
        if self.icp_enabled:
            F, T, icp_x, e_icp, dT, dfoot, _om2 = _icp_delta_TF(
                com_xy, com_vxy, com_z, sx, self.vx_cmd, F_sched, T_sched,
                dx_cap=self.outer_dx_cap,
                k_T=self.icp_k_T, k_foot=self.icp_k_foot,
            )
            self._icp_last_err = float(e_icp)
            self._icp_last_dT = float(dT)
            self._icp_last_dfoot = float(dfoot)
            self._icp_last_x = float(icp_x)
            # Count as ICP engage when feedback is non-trivial OR ICP estimated
            if abs(dT) > 1e-5 or abs(dfoot) > 1e-5 or abs(e_icp) > 1e-4:
                self._icp_n += 1
                self._icp_err_sum += float(e_icp)
                self._icp_dT_sum += float(dT)
                self._icp_dfoot_sum += float(dfoot)
            # CMP→ankle from ICP error (stance)
            self._apply_cmp_ankle(e_icp, a)
        else:
            F, T = F_sched, T_sched
            dT, dfoot, e_icp = 0.0, 0.0, 0.0

        # Mid-swing replan gate (outer must fire during clear swing)
        ds_frac = float(np.clip(wg.DS_S / max(wg.GAIT_T, 1e-3), 0.08, 0.55))
        ds_end = 0.5 + 0.5 * ds_frac
        in_swing = False
        for side, clr in (("L", L_clear), ("R", R_clear)):
            if not clr:
                continue
            pl = wg.phase_leg(phi, side)
            if pl < ds_end:
                continue
            s = (pl - ds_end) / max(1e-6, 1.0 - ds_end)
            if 0.08 <= s <= 0.96:
                in_swing = True
                break

        if in_swing and abs(F) > 1e-5:
            applied = _apply_outer_fp_track(
                m, d, self.act_idx, phi, a,
                self.bid_lf, self.bid_rf, F, gain=self.outer_track_gain,
            )
            self._outer_n += 1
            self._outer_F_sum += float(F)
            self._outer_T_sum += float(T)
            self._outer_applied_sum += float(applied)
            self._outer_footholds.append(float(F))
            if len(self._outer_footholds) > 64:
                self._outer_footholds = self._outer_footholds[-64:]
            self._alip_fp_prior = float(F_sched)
            self._alip_du_fp = float(F)
            self._alip_n_replan += 1
        else:
            self._outer_idle_steps += 1
            applied = 0.0

        self._outer_F = float(F)
        self._outer_T = float(T)
        self._last_outer_F = float(F)
        self._last_outer_T = float(T)
        return float(F), float(T)

    def step(self, action: np.ndarray):
        obs, reward, terminated, truncated, info = super().step(action)
        # Enrich info with ICP suite every step / episode end
        info["icp_err"] = float(self._icp_last_err)
        info["icp_dT"] = float(self._icp_last_dT)
        info["icp_dfoot"] = float(self._icp_last_dfoot)
        info["icp_x"] = float(self._icp_last_x)
        info["icp_n"] = int(self._icp_n)
        info["schedule_F"] = float(self._schedule_F)
        info["schedule_T"] = float(self._schedule_T)
        info["cmp_ank"] = float(self._cmp_ank_cum)
        info["outer_src"] = self.OUTER_SRC
        info["family"] = self.FAMILY
        info["cite"] = self.CITE
        if truncated or terminated:
            icp_idle = bool(self._icp_n < 3)
            info["icp_idle"] = icp_idle
            info["icp_err_avg"] = (
                float(self._icp_err_sum) / max(1, self._icp_n)
            )
            info["icp_dT_avg"] = (
                float(self._icp_dT_sum) / max(1, self._icp_n)
            )
            info["icp_dfoot_avg"] = (
                float(self._icp_dfoot_sum) / max(1, self._icp_n)
            )
            # Prefer FAIL tip: ICP idle while claiming outer — residual / D1 twin
            if icp_idle:
                reward = float(reward) - 10.0
            else:
                reward = float(reward) + 1.5  # ICP engaged credit
            # Stronger plant-cam ε Prefer FAIL shaping (attack Root B)
            if self._plant_prog_any > 0.02 or self._plant_prog_cum > 0.05:
                reward = float(reward) - 4.0
        return obs, float(reward), terminated, truncated, info


def make_t5d2_env(
    gait_t: float = 0.75,
    rank: int = 0,
    seed: int = 0,
    **kwargs,
):
    def _thunk():
        env = AinexT5D2ICPEnv(gait_t=gait_t, seed=seed + rank, **kwargs)
        env.reset(seed=seed + rank)
        return env
    return _thunk


# ---------------------------------------------------------------------------
# T5-E — FOOTSTEP-SEQ OUTER (≠ T5-D1 schedule twin · ≠ T5-D2 ICP)
# Cospec: docs/GATE_Q_AI_COSPEC_T5E.md  ·  Walk This Way doi:10.1145/3747865
# External ordered clear TDs PRIMARY (Placo-style short |dx_back| list).
# Residual = clear corrector only. Prefer FAIL if seq_idle / residual-primary /
# D1/D2 twin without TD-list ownership. Planted: cancel×0.70 + vel-oppose.
# Soft-pass NEVER. H2 OUT.
# ---------------------------------------------------------------------------

def _emit_retreat_td_list(
    target_dx: float,
    *,
    dx_back: float = 0.028,
    n_max: int = 10,
    n_min: int = 3,
    lat_bos: float = 0.0,
) -> list[dict]:
    """Placo-style short reverse foothold sequence (ordered clear TDs).

    Emits N steps of F=−|dx_back| alternating L/R that bank dest cam from
    clear Δ. First-class TD-list ownership — ≠ CSF+ALIP schedule alone (D1)
    and ≠ online ICP→ΔT+Δfoot (D2).
    """
    dxb = float(max(0.012, min(abs(float(dx_back)), 0.045)))
    budget = float(max(0.04, abs(float(target_dx))))
    n = int(math.ceil(budget / dxb))
    n = max(int(n_min), min(int(n_max), n))
    tds: list[dict] = []
    for i in range(n):
        side = "L" if (i % 2 == 0) else "R"
        lat = float(lat_bos) if side == "L" else -float(lat_bos)
        tds.append({
            "i": int(i),
            "F": float(-dxb),
            "side": side,
            "lat": float(lat),
            "dx_back": float(dxb),
        })
    return tds


class AinexT5EFootstepSeqEnv(AinexT5D1OuterPrimaryEnv):
    """T5-E: external retreat footstep sequence PRIMARY + clear-only residual.

    Outer owns ordered clear TD list / next-TD tracking (log every score).
    Residual = corrector only (warm-start optional from T5D1_13). Prefer FAIL
    if seq_idle or residual owns polarity or D1/D2 twin without TD-list.
    """

    OUTER_SRC = "FOOTSTEP-SEQ"  # Placo-style short |dx_back| TD list
    FAMILY = "T5E_FOOTSTEP-SEQ"
    CITE = "doi:10.1145/3747865;arXiv:2203.07589"

    def __init__(
        self,
        *,
        seq_dx_back: float = 0.028,
        seq_n_max: int = 10,
        seq_n_min: int = 3,
        seq_lat_bos: float = 0.0,
        seq_advance_err: float = 0.018,
        seq_enabled: bool = True,
        **kwargs,
    ):
        # Keep ALIP/DCM tools as late-swing corrector aids — sequence owns F
        kwargs.setdefault("outer_dx_cap", float(seq_dx_back))
        kwargs.setdefault("dxb_cap", float(seq_dx_back))
        kwargs.setdefault("alip_dx_cap", float(seq_dx_back))
        super().__init__(**kwargs)
        self.seq_dx_back = float(seq_dx_back)
        self.seq_n_max = int(seq_n_max)
        self.seq_n_min = int(seq_n_min)
        self.seq_lat_bos = float(seq_lat_bos)
        self.seq_advance_err = float(seq_advance_err)
        self.seq_enabled = bool(seq_enabled)
        self._td_list: list[dict] = []
        self._td_idx = 0
        self._td_n_advance = 0
        self._td_n_track = 0
        self._td_err_sum = 0.0
        self._td_err_n = 0
        self._td_last_err = 0.0
        self._td_last_F = 0.0
        self._td_last_side = "L"
        self._td_list_id = "empty"
        self._prev_swing_side = None
        self._seq_idle_flag = True

    def _rebuild_td_list(self) -> None:
        self._td_list = _emit_retreat_td_list(
            float(self.target_dx),
            dx_back=self.seq_dx_back,
            n_max=self.seq_n_max,
            n_min=self.seq_n_min,
            lat_bos=self.seq_lat_bos,
        )
        self._td_idx = 0
        self._td_list_id = (
            f"n{len(self._td_list)}_dxb{self.seq_dx_back:.3f}"
            f"_lat{self.seq_lat_bos:.3f}_tgt{float(self.target_dx):.3f}"
        )

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        obs, info = super().reset(seed=seed, options=options)
        self._rebuild_td_list()
        self._td_n_advance = 0
        self._td_n_track = 0
        self._td_err_sum = 0.0
        self._td_err_n = 0
        self._td_last_err = 0.0
        self._td_last_F = float(self._td_list[0]["F"]) if self._td_list else 0.0
        self._td_last_side = self._td_list[0]["side"] if self._td_list else "L"
        self._prev_swing_side = None
        self._seq_idle_flag = True
        return obs, info

    def set_curriculum(self, *, target_dx=None, episode_s=None, vx_lo=None, vx_hi=None):
        super().set_curriculum(
            target_dx=target_dx, episode_s=episode_s, vx_lo=vx_lo, vx_hi=vx_hi)
        # Rebuild sequence when curriculum target changes (new TD list)
        if target_dx is not None:
            self._rebuild_td_list()
            self._td_idx = 0

    def _next_td(self) -> dict | None:
        if not self._td_list:
            return None
        if self._td_idx >= len(self._td_list):
            # wrap / extend: append one more short reverse TD (keep sequence alive)
            side = "L" if (len(self._td_list) % 2 == 0) else "R"
            self._td_list.append({
                "i": len(self._td_list),
                "F": float(-abs(self.seq_dx_back)),
                "side": side,
                "lat": float(self.seq_lat_bos) if side == "L" else -float(self.seq_lat_bos),
                "dx_back": float(abs(self.seq_dx_back)),
            })
        return self._td_list[self._td_idx]

    def _swing_foot_xy(self, side: str) -> tuple[float, float]:
        d = self.data
        bid = self.bid_lf if side == "L" else self.bid_rf
        return float(d.xpos[bid, 0]), float(d.xpos[bid, 1])

    def _support_xy(self, swing_side: str) -> tuple[float, float]:
        d = self.data
        bid = self.bid_rf if swing_side == "L" else self.bid_lf
        return float(d.xpos[bid, 0]), float(d.xpos[bid, 1])

    def _update_outer_primary(self, phi: float, a: float) -> tuple[float, float]:
        """FOOTSTEP-SEQ outer: track next clear TD from external list (≠ D1/D2)."""
        if a < 0.05 or not self.seq_enabled:
            return 0.0, float(self._outer_T)
        d, m = self.data, self.model
        any_clear, L_clear, R_clear = self._sole_clear()
        if not any_clear:
            self._outer_idle_steps += 1
            return float(self._outer_F), float(self._outer_T)

        td = self._next_td()
        if td is None:
            self._outer_idle_steps += 1
            self._seq_idle_flag = True
            return float(self._outer_F), float(self._outer_T)

        F = float(td["F"])  # ≤0 short |dx_back|
        T = float(self.gait_t)  # sequence owns location; period from gait (≠ ICP ΔT)
        # Cap to outer_dx_cap / SHORT_DX co-constraint
        F = float(np.clip(F, -abs(self.outer_dx_cap), 0.0))
        self._td_last_F = float(F)
        self._td_last_side = str(td["side"])

        # Mid-swing replan gate — clear swing only (dest cam from clear Δ)
        ds_frac = float(np.clip(wg.DS_S / max(wg.GAIT_T, 1e-3), 0.08, 0.55))
        ds_end = 0.5 + 0.5 * ds_frac
        in_swing = False
        swing_side = None
        s_sw = None
        for side, clr in (("L", L_clear), ("R", R_clear)):
            if not clr:
                continue
            pl = wg.phase_leg(phi, side)
            if pl < ds_end:
                continue
            s = (pl - ds_end) / max(1e-6, 1.0 - ds_end)
            if 0.08 <= s <= 0.96:
                in_swing = True
                swing_side = side
                s_sw = float(s)
                break

        # Prefer matching planned side when both clear; else take available clear
        if in_swing and swing_side is not None:
            want = str(td["side"])
            if swing_side != want:
                # If wanted side is also clear in swing, prefer it
                want_clr = L_clear if want == "L" else R_clear
                if want_clr:
                    plw = wg.phase_leg(phi, want)
                    if plw >= ds_end:
                        sw = (plw - ds_end) / max(1e-6, 1.0 - ds_end)
                        if 0.08 <= sw <= 0.96:
                            swing_side = want
                            s_sw = float(sw)

        if in_swing and abs(F) > 1e-5 and swing_side is not None:
            # Next-TD error: swing foot vs support + F (clear track)
            sx, sy = self._support_xy(swing_side)
            fx, fy = self._swing_foot_xy(swing_side)
            tgt_x = sx + F
            tgt_y = sy + float(td.get("lat", 0.0))
            err = float(math.hypot(fx - tgt_x, fy - tgt_y))
            self._td_last_err = err
            self._td_err_sum += err
            self._td_err_n += 1

            applied = _apply_outer_fp_track(
                m, d, self.act_idx, phi, a,
                self.bid_lf, self.bid_rf, F, gain=self.outer_track_gain,
            )
            self._outer_n += 1
            self._outer_F_sum += float(F)
            self._outer_T_sum += float(T)
            self._outer_applied_sum += float(applied)
            self._outer_footholds.append(float(F))
            if len(self._outer_footholds) > 64:
                self._outer_footholds = self._outer_footholds[-64:]
            self._td_n_track += 1
            self._seq_idle_flag = False
            # Sync ALIP log fields for scorer compatibility
            self._alip_fp_prior = float(F)
            self._alip_du_fp = float(F)
            self._alip_n_replan += 1

            # Advance on late-swing near-TD OR swing-side exchange (touchdown proxy)
            advance = False
            if s_sw is not None and s_sw >= 0.85 and err <= self.seq_advance_err:
                advance = True
            if self._prev_swing_side is not None and swing_side != self._prev_swing_side:
                # support exchange completed prior TD
                advance = True
            if advance and self._td_idx < len(self._td_list) + 8:
                self._td_idx += 1
                self._td_n_advance += 1
                # peek next
                _ = self._next_td()
            self._prev_swing_side = swing_side
        else:
            self._outer_idle_steps += 1
            applied = 0.0

        self._outer_F = float(F)
        self._outer_T = float(T)
        self._last_outer_F = float(F)
        self._last_outer_T = float(T)
        return float(F), float(T)

    def step(self, action: np.ndarray):
        obs, reward, terminated, truncated, info = super().step(action)
        # Enrich with FOOTSTEP-SEQ suite every step / episode end
        td = self._next_td()
        info["seq_list_id"] = str(self._td_list_id)
        info["seq_n"] = int(len(self._td_list))
        info["seq_idx"] = int(self._td_idx)
        info["seq_next_F"] = float(td["F"]) if td else 0.0
        info["seq_next_side"] = str(td["side"]) if td else ""
        info["seq_next_err"] = float(self._td_last_err)
        info["seq_n_advance"] = int(self._td_n_advance)
        info["seq_n_track"] = int(self._td_n_track)
        info["seq_dx_back"] = float(self.seq_dx_back)
        info["outer_src"] = self.OUTER_SRC
        info["family"] = self.FAMILY
        info["cite"] = self.CITE
        # foothold list snapshot (F values)
        info["seq_footholds"] = [float(t["F"]) for t in self._td_list[:12]]
        if truncated or terminated:
            seq_idle = bool(
                self._td_n_track < 3
                or self._td_n_advance < 1
                or (not self._td_list)
            )
            self._seq_idle_flag = seq_idle
            info["seq_idle"] = bool(seq_idle)
            info["seq_err_avg"] = (
                float(self._td_err_sum) / max(1, self._td_err_n)
            )
            # Prefer FAIL: seq idle while claiming FOOTSTEP-SEQ outer
            if seq_idle:
                reward = float(reward) - 12.0
            else:
                reward = float(reward) + 2.0  # sequence engaged credit
            # Stronger plant-cam ε Prefer FAIL (attack Root B / beat maxG 0.055)
            if self._plant_prog_any > 0.02 or self._plant_prog_cum > 0.05:
                reward = float(reward) - 5.0
            # Hold D1 locomotor: do not reward ε cosmetics alone
            info["outer_idle"] = bool(seq_idle or info.get("outer_idle", False))
        else:
            info["seq_idle"] = bool(self._seq_idle_flag)
        return obs, float(reward), terminated, truncated, info


def make_t5e_env(
    gait_t: float = 0.75,
    rank: int = 0,
    seed: int = 0,
    **kwargs,
):
    def _thunk():
        env = AinexT5EFootstepSeqEnv(gait_t=gait_t, seed=seed + rank, **kwargs)
        env.reset(seed=seed + rank)
        return env
    return _thunk
