#!/usr/bin/env python3
"""Dronable Controls — open-loop 50 Hz PD walk on kit-matched AiNex mesh.

Model: mujoco/ainex_hiwonder/ainex_controls.xml
  = Hardware ainex.xml (Hiwonder URDF + 25 STL, NOT OEM STEP)
  + Controls overlay: HX torque clips (±2.1 leg / ±0.7 arm-head),
    mesh collision off + foot contact boxes, L=orange / R=green.

Hardening (2026-09-27 evening BST):
  - Lateral COM shift via hip_roll *before* swing (phase lead)
  - Stance foot plant damper (tangential slip oppose while in contact)
  - Ankle-strategy CoP servo: COM-xy vs support-foot xy → ank_roll/ank_pitch @ 50 Hz
  - Optional world-frame stance-foot freeze ablation (single support)
  - Stand disturbance recovery mode (--disturb-stand)
  - WORLD_FIXED acceptance: upright time, contacts, dx, skate gates
  - Default: plant ON, ankle CoP ON, balance assist OFF (no soft-pass)

Axis note (differs from dronable_v0):
  Forward = +X (URDF). Hip pitch axes mirrored L(+Y) / R(-Y).
  L foot at +Y, R foot at -Y. Map: joint_hip_L = -fwd, joint_hip_R = +fwd;
  joint_knee_L = +flex, joint_knee_R = -flex; ank = hip_j + knee_j (flat foot).

Does NOT break scripts/walk_gait.py (still wired to dronable_v0.xml).

Usage:
  MUJOCO_GL=egl python scripts/walk_gait_ainex.py
  MUJOCO_GL=egl python scripts/walk_gait_ainex.py --disturb-stand   # ankle CoP recovery proof
  MUJOCO_GL=egl python scripts/walk_gait_ainex.py --assist          # upright+speed (NOT clean walk)
  MUJOCO_GL=egl python scripts/walk_gait_ainex.py --no-ankle-cop
  MUJOCO_GL=egl python scripts/walk_gait_ainex.py --stance-freeze   # ablation
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

import mujoco as mj
import numpy as np

from residual_stance_vx import (
    apply_residual_stance_vx,
    clip_total_hx,
    load_npz as load_residual_npz,
    AUTH_CTRL_LIM as _AUTH_CTRL_LIM_DEFAULT,
)
import residual_stance_vx as _residual_mod
from wbc_stance_qp import apply_wbc_stance_qp
from hybrid_mpc_t88 import apply_hybrid_mpc_t88, reset_mpc_state

ROOT = Path(__file__).resolve().parents[1]
XML_DEFAULT = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls.xml"
OUT_DIR = ROOT / "previews" / "ainex_walk"
HIWONDER_DIR = ROOT / "mujoco" / "ainex_hiwonder"

# --- Controls params (kit-plausible, not cartoon) ---
CTRL_HZ = 50.0
GAIT_T = 0.72          # Phase B best basin
STEP_LEN = 0.035       # m cmd (≤0.04)
HIP_PITCH_AMP = 0.12   # rad — BEST_C11 basin
HIP_BIAS_FWD = 0.07
KNEE_STANCE = 0.42
KNEE_SWING = 0.72
ANK_BIAS = 0.0
COM_Z = 0.225          # low stance
V_DES_X = 0.12         # only used when --assist
FOOT_CLEAR = 0.025
SWING_ABDUCT = 0.05   # rad * swing_blend — Phase F clearance
SWING_ANK_DF = 0.18  # rad * swing_blend dorsiflex clearance
COM_SHIFT_AMP = 0.26
COM_SHIFT_LEAD = 0.22
DS_S = 0.20
PLANT_KD = 60.0
PLANT_MAX_F = 20.0     # N clip
CONTACT_Z_THR = 0.025  # foot body z below this ≈ planted (box half-h≈8mm + margin)
# Phase B/C feature flags (CLI)
USE_CP_SWING = True    # Phase B ON by default (BEST_C11)
USE_HIP_STRAT = False  # residual COM accel hip strategy
USE_CAPTURE_STEP = False  # reactive capture step on tip detect
CP_OMEGA = 3.5         # rad/s √(g/z) approx for z≈0.23 → ~6.5; start milder
CP_STEP_GAIN = 1.0     # place foot at CP * gain beyond stance
CP_MAX_STEP = 0.045    # m sagittal swing placement clip
HIP_STRAT_KP = 0.12    # rad per (m/s^2) COM accel share to hip (mild)
HIP_STRAT_CLIP = 0.08
CAPTURE_UP_Z = 0.78    # tip detect threshold
CAPTURE_STEP_M = 0.04
STANCE_V_NULL_K = 0.55   # rad per (m/s) stance foot vx → hip fwd correction
STANCE_V_NULL_CLIP = 0.12
USE_STANCE_V_NULL = False
STANCE_SWEEP_FRAC = 1.00  # full stance sweep (B02 basin); <1 cuts dx more than skate)
# Phase D: stance-foot Jacobian velocity IK (drive world vx,vy→0 in contact)
USE_STANCE_VIK = False
STANCE_VIK_KP = 1.20      # 1/s — v_des = -KP * v_foot (deadbeat≈1/dt too strong)
STANCE_VIK_KDAMP = 1e-3   # damped LS ridge
STANCE_VIK_CLIP = 0.10    # rad/ctrl-step joint clip
STANCE_VIK_W_OMEGA = 0.0   # optional ωz null weight (0=off)
STANCE_VIK_DEADZONE = 0.015  # m/s — ignore tiny slip
USE_ZMP_QP = False
ZMP_QP_KP = 0.35           # rad per m CP-support error
ZMP_QP_CLIP = 0.08
USE_WBC_STANCE = False     # friction-cone stance wrench QP (replaces residual class)
WBC_MU = 0.8               # friction cone μ for QP (A/B 0.6–1.0)
WBC_SCRUB_K = 18.0         # N per (m/s) soft skate scrub in cone
WBC_TRIM_CLIP = 0.12       # rad Δctrl per joint from τ/kp
WBC_TAU_SCALE = 1.0
WBC_W_SCRUB = 0.35
USE_HYBRID_MPC = False      # hybrid MPC + T88 hard-reject (post WBC falsifier)
AUTH_K = 1.0               # HX torque/position envelope scale (sim auth A/B)
AUTH_SAT_COUNT = 0
AUTH_SAT_STEPS = 0
AUTH_LEG_ACT_IDS: list[int] = []
MPC_N = 12
MPC_U_MAX = 0.10
MPC_APPLY_THR = 0.11
MPC_DISABLE_ABOVE_T = 0.86
MPC_ANK_RATIO = 0.6
STANCE_VIK_ANKLE_ONLY = False  # restrict Jacobian cols to ank_pitch/ank_roll
USE_STANCE_HOLD = False        # hold stance-leg hip/knee setpoints at touchdown (joint-space)
USE_FOOT_POS_IK = False        # Jacobian position IK: hold stance foot world xy at touchdown
USE_RESIDUAL_STANCE_VX = False  # additive residual (not open-loop μ/VIK retune)
RESIDUAL_FIT = None             # dict from load_residual_npz
RESIDUAL_GAIN = 1.0
RESIDUAL_PATH = "none"
FOOT_POS_IK_KP = 8.0           # 1/s — dq from J+ * KP * (xy_hold - xy)
FOOT_POS_IK_CLIP = 0.08

# Ankle-strategy balance servo (lean-primary + relative CoP @ CTRL_HZ; HX clips)
# Tuned mild: strong lean/CoP gains *destabilize* quiet stand under position
# actuators + HX ±2.1 Nm. Absolute COM-foot without bias also tips.
# Tip-threshold impulse recovery requires --stance-freeze ablation (documented).
ANK_COP_KP_LEAN_X = 0.45   # rad / up_x  — lean forward → plantarflex
ANK_COP_KD_LEAN_X = 0.02   # light rate damp (wrong sign explodes)
ANK_COP_KP_LEAN_Y = 0.55   # rad / up_y
ANK_COP_KD_LEAN_Y = 0.02
ANK_COP_KP_X = 0.80        # rad/m relative COM-support (after bias)
ANK_COP_KP_Y = 1.00        # rad/m
ANK_COP_DEADZONE = 0.005   # m
ANK_COP_CLIP = 0.30        # rad additive clip per ankle
ANK_COP_HIP_SHARE = 0.30
ANK_COP_BIAS_SETTLE_S = 0.50

# Stance-foot world freeze ablation (external wrench on foot body; disclosed)
FREEZE_KP = 280.0       # N/m
FREEZE_KD = 50.0        # N/(m/s)
FREEZE_FMAX = 35.0      # N

# WORLD_FIXED side cam: walk +X through frame; camera on +Y looking -Y
CAM_FIXED_LOOKAT = np.array([0.30, 0.0, 0.12], dtype=np.float64)
CAM_FIXED_DISTANCE = 1.05
CAM_FIXED_AZIMUTH = 90.0
CAM_FIXED_ELEVATION = -12.0

# --- WORLD_FIXED acceptance gates (measurable; all required for clean PASS) ---
ACC = {
    "min_gait_s": 4.0,
    "upright_frac": 0.90,       # fraction up_z>0.85 and z>0.18
    "min_up_z": 0.85,
    "min_body_z": 0.18,
    "contact_duty_lo": 0.20,    # each foot contact fraction
    "contact_duty_hi": 0.85,
    "min_foot_lead_cycles": 3,
    "min_hip_ptp": 0.18,
    "min_knee_ptp": 0.18,
    "min_dx_m": 0.05,           # forward progress
    "max_avg_speed": 0.22,      # m/s kit-class
    "max_stance_vx_mean": 0.08, # m/s — above this = skate
    "max_stance_vx_p95": 0.18,
    "require_assist_off": True, # clean walk claim needs balance assist OFF
}


def act_name(jname: str) -> str:
    return f"{jname}_pos"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def phase_leg(phi: float, side: str) -> float:
    if side == "R":
        phi = (phi + 0.5) % 1.0
    return phi


def swing_blend(s: float) -> float:
    s = clamp(s, 0.0, 1.0)
    return math.sin(math.pi * (s ** 0.55))


def to_joint(side: str, fwd: float, flex: float) -> tuple[float, float, float]:
    """Map signed kinematic (fwd thigh, knee flex) → (hip_j, knee_j, ank_j)."""
    if side == "L":
        hip = -fwd
        knee = +flex
    else:
        hip = +fwd
        knee = -flex
    ank = hip + knee
    return hip, knee, ank


def lateral_com_target(phi: float) -> float:
    """Signed lateral weight target: +1 = COM toward L (+Y), -1 toward R (-Y).

    Lead the gait phase so weight sits on the upcoming stance foot *before*
    the opposite swing lifts.
    """
    return math.cos(2.0 * math.pi * (phi + COM_SHIFT_LEAD))


def gait_targets(t: float, gait_on: bool, amp: float = 1.0) -> dict[str, float]:
    """Open-loop CPG → AiNex joint targets (rad), with pre-swing COM shift."""
    q = {
        "head_pan": 0.0,
        "head_tilt": 0.0,
        "l_sho_pitch": 0.0,
        "l_sho_roll": -1.40,
        "l_el_pitch": 0.38,
        "l_el_yaw": 0.0,
        "l_gripper": 0.0,
        "r_sho_pitch": 0.0,
        "r_sho_roll": 1.40,
        "r_el_pitch": 0.38,
        "r_el_yaw": 0.0,
        "r_gripper": 0.0,
        "l_hip_yaw": 0.0,
        "l_hip_roll": -0.05,
        "r_hip_yaw": 0.0,
        "r_hip_roll": 0.05,
        "l_ank_roll": 0.0,
        "r_ank_roll": 0.0,
    }
    for side in ("L", "R"):
        hip, knee, ank = to_joint(side, HIP_BIAS_FWD, KNEE_STANCE)
        p = "l_" if side == "L" else "r_"
        q[f"{p}hip_pitch"] = hip
        q[f"{p}knee"] = knee
        q[f"{p}ank_pitch"] = ank

    if not gait_on or amp <= 0.0:
        return q

    a = clamp(amp, 0.0, 1.0)
    phi = (t / GAIT_T) % 1.0
    # +shift_cmd → both hip rolls more negative → empirically moves COM (see plant trial)
    # We map: want COM toward L (+Y) when lat>0.
    lat = lateral_com_target(phi)  # +1 → want COM toward L (+Y), -1 → R
    # Empirically on this URDF: common-mode (both rolls -= s) with s<0
    # brings body closer to L foot. So s = -amp*lat.
    shift = -COM_SHIFT_AMP * a * lat

    # Base abduct + common lateral shift
    q["l_hip_roll"] = -0.05 - shift
    q["r_hip_roll"] = 0.05 - shift
    # Ankle rolls co-lean to push COM (same sign as hip common-mode intent)
    q["l_ank_roll"] = -0.70 * shift
    q["r_ank_roll"] = -0.70 * shift

    # Phase fractions: stance → DS hold → swing. DS_S ≥ 0.15 s.
    ds_frac = clamp(DS_S / max(GAIT_T, 1e-3), 0.08, 0.55)  # was 0.35; dual-T reopt needs DS duty room
    stance_end = 0.50
    ds_end = stance_end + ds_frac
    swing_len = max(0.18, 1.0 - ds_end)

    for side in ("L", "R"):
        p = phase_leg(phi, side)
        pref = "l_" if side == "L" else "r_"
        if p < stance_end:
            # stance: push from fwd=+amp → -amp (trailing)
            s = p / stance_end
            s = s * s * (3.0 - 2.0 * s)
            # Reduced stance sweep → less required body velocity → less skate
            sweep = STANCE_SWEEP_FRAC
            fwd = HIP_BIAS_FWD + HIP_PITCH_AMP * a * sweep * (1.0 - 2.0 * s)
            flex = KNEE_STANCE
            sw = 0.0
            push = max(0.0, (s - 0.55) / 0.45)
            ank_extra = 0.0 * a * push
        elif p < ds_end:
            # double-support hold (no lift) — weight already on upcoming stance
            s = (p - stance_end) / max(ds_frac, 1e-6)
            fwd = HIP_BIAS_FWD + HIP_PITCH_AMP * a * (-STANCE_SWEEP_FRAC)
            flex = KNEE_STANCE + 0.06 * s
            sw = 0.0
            ank_extra = 0.0
        else:
            # swing: from -sweep → +1.0 (overshoot to place foot ahead)
            s = (p - ds_end) / swing_len
            sw = swing_blend(s)
            s_sm = s * s * (3.0 - 2.0 * s)
            fwd_lo = -STANCE_SWEEP_FRAC
            fwd_hi = 1.0
            fwd = HIP_BIAS_FWD + HIP_PITCH_AMP * a * (fwd_lo + (fwd_hi - fwd_lo) * s_sm)
            flex = KNEE_STANCE + (KNEE_SWING - KNEE_STANCE) * sw
            ank_extra = SWING_ANK_DF * sw  # dorsiflex clearance

        hip, knee, ank = to_joint(side, fwd, flex)
        if side == "L":
            ank = ank + ank_extra
        else:
            ank = ank - ank_extra

        q[f"{pref}hip_pitch"] = clamp(hip, -1.5, 1.5)
        q[f"{pref}knee"] = clamp(knee, -2.0, 2.0)
        q[f"{pref}ank_pitch"] = clamp(ank, -1.2, 1.2)

        # extra abduct on swing leg for clearance (cut contact_duty)
        if side == "L":
            q[f"{pref}hip_roll"] = q["l_hip_roll"] - SWING_ABDUCT * sw
        else:
            q[f"{pref}hip_roll"] = q["r_hip_roll"] + SWING_ABDUCT * sw

        arm = -0.22 * (fwd - HIP_BIAS_FWD)
        q[f"{pref}sho_pitch"] = arm
        q[f"{pref}el_pitch"] = 0.32 + 0.08 * abs(arm)

    return q


def _lock_upright_yaw0(qpos: np.ndarray, qvel: np.ndarray, blend: float = 1.0) -> None:
    tgt = np.array([1.0, 0.0, 0.0, 0.0])
    cur = np.array([float(v) for v in qpos[3:7]])
    if np.dot(cur, tgt) < 0:
        tgt = -tgt
    out = cur + blend * (tgt - cur)
    out = out / (np.linalg.norm(out) + 1e-12)
    qpos[3:7] = out
    qvel[3:6] *= (1.0 - 0.85 * blend)


def balance_assist(
    model: mj.MjModel,
    data: mj.MjData,
    z_des: float = COM_Z,
    fx_bias: float = 0.0,
    v_des_x: float = 0.0,
    speed_gate: float = 1.0,
):
    """Temporary upright/height/speed assist — NOT production; marks clean_walk_claim False."""
    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    R = data.xmat[bid].reshape(3, 3)
    up = R[:, 2]
    axis = np.cross(up, np.array([0.0, 0.0, 1.0]))
    lean = float(np.linalg.norm(axis))
    if lean > 0.25:
        axis = axis * (0.25 / lean)

    omega = data.qvel[3:6]
    kp_ori, kd_ori = 55.0, 7.0
    torque = np.clip(kp_ori * axis - kd_ori * omega, -10.0, 10.0)

    mass = mj.mj_getTotalmass(model)
    z = data.qpos[2]
    fz = 0.90 * mass * 9.81 + 180.0 * (z_des - z) - 30.0 * data.qvel[2]
    fz = float(np.clip(fz, -40.0, 55.0))
    fy = float(np.clip(-14.0 * data.qpos[1] - 5.0 * data.qvel[1], -10.0, 10.0))
    vx = float(data.qvel[0])
    v_tgt = float(v_des_x) * float(speed_gate)
    fx = float(np.clip(fx_bias + 18.0 * (v_tgt - vx), -8.0, 8.0))
    data.qfrc_applied[0:6] = [fx, fy, fz, torque[0], torque[1], torque[2]]

    blend = 0.50 if up[2] > 0.85 else 0.85
    _lock_upright_yaw0(data.qpos, data.qvel, blend=blend)


def _free_joint_dofadr(model: mj.MjModel) -> int:
    """Dof address of freejoint (0 on Gate E plant; >0 when door hinges precede root on companion)."""
    for i in range(model.njnt):
        if int(model.jnt_type[i]) == int(mj.mjtJoint.mjJNT_FREE):
            return int(model.jnt_dofadr[i])
    return 0


def stance_plant(
    model: mj.MjModel,
    data: mj.MjData,
    phi: float,
    amp: float,
    bid_lf: int,
    bid_rf: int,
    gid_lfoot: int,
    gid_rfoot: int,
    gid_floor: int,
):
    """Oppose horizontal slip of the stance foot while it is in contact.

    This is a *plant* aid (no upright quat lock, no speed governor). Still
    external wrench — must be disclosed; does NOT alone make a clean walk PASS
    if stance vx remains high.
    """
    if amp <= 0.01:
        return
    # Which foot is stance (with lead matching lateral_com_target)
    lat = lateral_com_target(phi)
    # lat>0 → weight on L; lat<0 → weight on R
    # During transition |lat|<0.25 allow both light plant
    plants = []
    if lat > -0.25:
        plants.append(("L", bid_lf, gid_lfoot))
    if lat < 0.25:
        plants.append(("R", bid_rf, gid_rfoot))

    # Contact flags from MuJoCo contact list
    in_contact = {"L": False, "R": False}
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        for side, _bid, gid in (("L", bid_lf, gid_lfoot), ("R", bid_rf, gid_rfoot)):
            if (g1 == gid and g2 == gid_floor) or (g2 == gid and g1 == gid_floor):
                in_contact[side] = True
            # also toe spheres share floor contact — check by body
        # toe geoms: detect via body of either geom
        for side, bid, _gid in plants:
            b1 = int(model.geom_bodyid[g1])
            b2 = int(model.geom_bodyid[g2])
            if (b1 == bid or b2 == bid) and (g1 == gid_floor or g2 == gid_floor):
                in_contact[side] = True

    # Fallback height heuristic if contact buffer empty early
    for side, bid, _gid in (("L", bid_lf, gid_lfoot), ("R", bid_rf, gid_rfoot)):
        if data.xpos[bid, 2] < CONTACT_Z_THR:
            in_contact[side] = True

    fx_sum = fy_sum = 0.0
    for side, bid, _gid in plants:
        if not in_contact[side]:
            continue
        # cvel is [rot; lin] in world about com of body; lin part indices 3:6
        v = data.cvel[bid][3:6].copy()
        # weight by |lat| so primary stance gets more plant
        w = clamp(abs(lat), 0.35, 1.0) if ((side == "L" and lat > 0) or (side == "R" and lat < 0)) else 0.35
        fx = float(np.clip(-PLANT_KD * w * amp * v[0], -PLANT_MAX_F, PLANT_MAX_F))
        fy = float(np.clip(-PLANT_KD * w * amp * v[1], -PLANT_MAX_F, PLANT_MAX_F))
        fx_sum += fx
        fy_sum += fy
    # Root free-joint horizontal oppose keyed to stance-foot slip (disclosed plant aid).
    # Foot-body-only xfrc was tried and tipped; keep root form that achieved tip-free≥8s.
    # Use free-joint dofadr — on companion door hinges occupy dof 0/1.
    v0 = _free_joint_dofadr(model)
    data.qfrc_applied[v0 + 0] += fx_sum
    data.qfrc_applied[v0 + 1] += fy_sum






def apply_stance_vx_null(
    model: mj.MjModel,
    data: mj.MjData,
    act_idx: dict[str, int],
    phi: float,
    amp: float,
    bid_lf: int,
    bid_rf: int,
):
    """Null stance-foot world vx via hip/ankle joint correction (no free-joint force).

    If stance foot slides forward (+vx), command trailing hip (reduce fwd) to scrub speed.
    """
    if amp < 0.05 or not USE_STANCE_V_NULL:
        return
    lat = lateral_com_target(phi)
    targets = []
    if lat > 0.05:
        targets.append(("L", bid_lf, -1.0))  # L hip_j = -fwd
    if lat < -0.05:
        targets.append(("R", bid_rf, +1.0))
    for side, bid, jsign in targets:
        if data.xpos[bid, 2] > CONTACT_Z_THR + 0.01:
            continue
        vx = float(data.cvel[bid][3])
        # only oppose significant slip
        if abs(vx) < 0.02:
            continue
        # positive vx → need negative fwd correction
        d_fwd = float(np.clip(-STANCE_V_NULL_K * vx, -STANCE_V_NULL_CLIP, STANCE_V_NULL_CLIP)) * amp
        pref = "l_" if side == "L" else "r_"
        an = act_name(f"{pref}hip_pitch")
        if an in act_idx:
            data.ctrl[act_idx[an]] = float(np.clip(
                data.ctrl[act_idx[an]] + jsign * d_fwd, -1.5, 1.5))
        an_a = act_name(f"{pref}ank_pitch")
        if an_a in act_idx:
            data.ctrl[act_idx[an_a]] = float(np.clip(
                data.ctrl[act_idx[an_a]] + jsign * 0.6 * d_fwd, -1.2, 1.2))




def _leg_joint_dof_cols(model: mj.MjModel, side: str) -> tuple[list[str], list[int]]:
    """Return (joint_names, dof_column_indices) for hip/knee/ankle of one leg."""
    pref = "l_" if side == "L" else "r_"
    names = [
        f"{pref}hip_yaw", f"{pref}hip_roll", f"{pref}hip_pitch",
        f"{pref}knee", f"{pref}ank_pitch", f"{pref}ank_roll",
    ]
    cols: list[int] = []
    keep: list[str] = []
    for jn in names:
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
        if jid < 0:
            continue
        adr = int(model.jnt_dofadr[jid])
        cols.append(adr)
        keep.append(jn)
    return keep, cols



_stance_hold_q: dict[str, dict[str, float] | None] = {"L": None, "R": None}


def apply_stance_joint_hold(
    model: mj.MjModel,
    data: mj.MjData,
    act_idx: dict[str, int],
    phi: float,
    amp: float,
    bid_lf: int,
    bid_rf: int,
    gid_floor: int,
):
    """Hold stance-leg hip/knee position setpoints at touchdown (joint-space plant).

    Not world-frame freeze / xfrc. Releases on liftoff so swing can proceed.
    """
    if amp < 0.05 or not USE_STANCE_HOLD:
        return
    lat = lateral_com_target(phi)
    for side, bid, primary in (
        ("L", bid_lf, lat > 0.05),
        ("R", bid_rf, lat < -0.05),
    ):
        pref = "l_" if side == "L" else "r_"
        in_contact = foot_floor_contact(model, data, bid, gid_floor)
        if primary and in_contact:
            if _stance_hold_q[side] is None:
                held = {}
                for jn in (f"{pref}hip_pitch", f"{pref}knee", f"{pref}hip_roll"):
                    an = act_name(jn)
                    if an in act_idx:
                        held[an] = float(data.ctrl[act_idx[an]])
                _stance_hold_q[side] = held
            else:
                for an, val in _stance_hold_q[side].items():
                    data.ctrl[act_idx[an]] = val
        else:
            _stance_hold_q[side] = None



_foot_xy_hold: dict[str, np.ndarray | None] = {"L": None, "R": None}


def apply_foot_pos_ik(
    model: mj.MjModel,
    data: mj.MjData,
    act_idx: dict[str, int],
    phi: float,
    amp: float,
    bid_lf: int,
    bid_rf: int,
    gid_floor: int,
):
    """Phase D: hold stance foot world xy via Jacobian position IK (no xfrc)."""
    if amp < 0.05 or not USE_FOOT_POS_IK:
        return
    lat = lateral_com_target(phi)
    jacp = np.zeros((3, model.nv), dtype=np.float64)
    jacr = np.zeros((3, model.nv), dtype=np.float64)
    dt = 1.0 / CTRL_HZ
    for side, bid, primary in (
        ("L", bid_lf, lat > 0.08),
        ("R", bid_rf, lat < -0.08),
    ):
        in_c = foot_floor_contact(model, data, bid, gid_floor)
        if not (primary and in_c):
            _foot_xy_hold[side] = None
            continue
        if _foot_xy_hold[side] is None:
            _foot_xy_hold[side] = data.xpos[bid, :2].copy()
        hold = _foot_xy_hold[side]
        err = hold - np.asarray(data.xpos[bid, :2], dtype=np.float64)
        if float(np.linalg.norm(err)) < 0.001:
            continue
        jnames, cols = _leg_joint_dof_cols(model, side)
        if STANCE_VIK_ANKLE_ONLY:
            pair = [(jn, c) for jn, c in zip(jnames, cols) if "ank_" in jn]
            if len(pair) < 2:
                continue
            jnames = [p[0] for p in pair]
            cols = [p[1] for p in pair]
        if len(cols) < 2:
            continue
        mj.mj_jacBody(model, data, jacp, jacr, bid)
        J = jacp[0:2, cols].copy()
        v_des = FOOT_POS_IK_KP * err * amp
        n = J.shape[0]
        JJT = J @ J.T + STANCE_VIK_KDAMP * np.eye(n)
        try:
            dq = J.T @ np.linalg.solve(JJT, v_des)
        except np.linalg.LinAlgError:
            continue
        dq = np.clip(dq * dt, -FOOT_POS_IK_CLIP, FOOT_POS_IK_CLIP)
        for jn, dqi in zip(jnames, dq):
            an = act_name(jn)
            if an not in act_idx:
                continue
            lo, hi = (-1.2, 1.2) if "ank" in jn else (-1.5, 1.5)
            data.ctrl[act_idx[an]] = float(np.clip(data.ctrl[act_idx[an]] + float(dqi), lo, hi))


def apply_stance_jacobian_vik(
    model: mj.MjModel,
    data: mj.MjData,
    act_idx: dict[str, int],
    phi: float,
    amp: float,
    bid_lf: int,
    bid_rf: int,
    gid_floor: int,
):
    """Phase D: damped least-squares Jacobian velocity IK on stance foot.

    While foot in contact, solve J_xy dq = -v_xy (optionally ωz) using hip/knee/ankle
    DOFs only, then integrate dq into position setpoints. No root xfrc / freeze /
    speed governor. Torque stays within HX forcerange of position actuators.
    """
    if amp < 0.05 or not USE_STANCE_VIK:
        return
    lat = lateral_com_target(phi)
    # primary stance weight; allow light DS on both
    targets: list[tuple[str, int, float]] = []
    if lat > -0.35:
        wL = float(np.clip(0.55 + 0.45 * lat, 0.15, 1.0))
        targets.append(("L", bid_lf, wL))
    if lat < 0.35:
        wR = float(np.clip(0.55 - 0.45 * lat, 0.15, 1.0))
        targets.append(("R", bid_rf, wR))

    jacp = np.zeros((3, model.nv), dtype=np.float64)
    jacr = np.zeros((3, model.nv), dtype=np.float64)
    dt = 1.0 / CTRL_HZ

    for side, bid, w_stance in targets:
        if not foot_floor_contact(model, data, bid, gid_floor):
            continue
        if float(data.xpos[bid, 2]) > CONTACT_Z_THR + 0.012:
            continue
        # world linear vel of body COM; optional yaw rate
        vx = float(data.cvel[bid][3])
        vy = float(data.cvel[bid][4])
        wz = float(data.cvel[bid][2])
        speed = math.hypot(vx, vy)
        if speed < STANCE_VIK_DEADZONE and abs(wz) < 0.05:
            continue

        jnames, cols = _leg_joint_dof_cols(model, side)
        if STANCE_VIK_ANKLE_ONLY:
            keep_j, keep_c = [], []
            for jn, c in zip(jnames, cols):
                if "ank_" in jn:
                    keep_j.append(jn)
                    keep_c.append(c)
            jnames, cols = keep_j, keep_c
        if len(cols) < 2:
            continue
        mj.mj_jacBody(model, data, jacp, jacr, bid)
        J = jacp[0:2, cols].copy()  # 2 x n
        v = np.array([vx, vy], dtype=np.float64)
        if STANCE_VIK_W_OMEGA > 1e-6:
            Jw = jacr[2:3, cols]
            J = np.vstack([J, STANCE_VIK_W_OMEGA * Jw])
            v = np.concatenate([v, [STANCE_VIK_W_OMEGA * wz]])

        # damped LS: dq = J^T (J J^T + λ I)^{-1} v_des
        v_des = -STANCE_VIK_KP * v * w_stance * amp
        JJT = J @ J.T
        n_row = JJT.shape[0]
        JJT.flat[:: n_row + 1] += STANCE_VIK_KDAMP
        try:
            dq = J.T @ np.linalg.solve(JJT, v_des)
        except np.linalg.LinAlgError:
            continue
        # integrate to position setpoint delta
        dq = np.clip(dq * dt, -STANCE_VIK_CLIP, STANCE_VIK_CLIP)
        for jn, dqi in zip(jnames, dq):
            an = act_name(jn)
            if an not in act_idx:
                continue
            lo, hi = (-1.5, 1.5)
            if "ank" in jn:
                lo, hi = (-1.2, 1.2)
            data.ctrl[act_idx[an]] = float(np.clip(data.ctrl[act_idx[an]] + float(dqi), lo, hi))


def apply_zmp_qp_trim(
    model: mj.MjModel,
    data: mj.MjData,
    act_idx: dict[str, int],
    phi: float,
    amp: float,
    bid_lf: int,
    bid_rf: int,
):
    """Phase D tiny 50 Hz CP/ZMP trim: bias ankle/hip toward keeping CP in support.

    Not a full QP solver — clipped proportional on CP−support within HX setpoints.
    """
    if amp < 0.05 or not USE_ZMP_QP:
        return
    com_xy, com_vxy, com_z = estimate_com_state(model, data)
    omega = CP_OMEGA if CP_OMEGA > 0.1 else max(0.5, math.sqrt(9.81 / max(com_z, 0.12)))
    cp = capture_point_xy(com_xy, com_vxy, omega)
    lat = lateral_com_target(phi)
    # support = weighted foot xy
    wL = float(np.clip(0.5 + 0.5 * lat, 0.0, 1.0))
    wR = 1.0 - wL
    support = wL * data.xpos[bid_lf, :2] + wR * data.xpos[bid_rf, :2]
    err = cp - support  # want CP inside support → drive ankles
    dx = float(np.clip(ZMP_QP_KP * err[0], -ZMP_QP_CLIP, ZMP_QP_CLIP)) * amp
    dy = float(np.clip(ZMP_QP_KP * 1.1 * err[1], -ZMP_QP_CLIP, ZMP_QP_CLIP)) * amp
    # sagittal: plantarflex if CP ahead (same sign convention as CoP servo)
    for side, s_pitch in (("l", +1.0), ("r", -1.0)):
        an = act_name(f"{side}_ank_pitch")
        if an in act_idx:
            data.ctrl[act_idx[an]] = float(np.clip(
                data.ctrl[act_idx[an]] + s_pitch * dx, -1.2, 1.2))
        an_h = act_name(f"{side}_hip_pitch")
        if an_h in act_idx:
            # L hip_j = -fwd, R = +fwd; push hips to help contain CP
            s_fwd = -1.0 if side == "l" else 1.0
            data.ctrl[act_idx[an_h]] = float(np.clip(
                data.ctrl[act_idx[an_h]] + s_fwd * (-0.4 * dx), -1.5, 1.5))
    for jn, delta in (
        ("l_ank_roll", -dy),
        ("r_ank_roll", -dy),
        ("l_hip_roll", -0.5 * dy),
        ("r_hip_roll", -0.5 * dy),
    ):
        an = act_name(jn)
        if an in act_idx:
            data.ctrl[act_idx[an]] = float(np.clip(data.ctrl[act_idx[an]] + delta, -1.5, 1.5))

def estimate_com_state(model: mj.MjModel, data: mj.MjData) -> tuple[np.ndarray, np.ndarray, float]:
    """Freejoint / subtree COM xy and velocity; return (com_xy, com_vxy, com_z)."""
    # Prefer body_link COM when door bodies inflate world subtree_com[0] (companion).
    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    if bid >= 0:
        com = np.asarray(data.subtree_com[bid], dtype=np.float64).copy()
    else:
        com = np.asarray(data.subtree_com[0], dtype=np.float64).copy()
    v0 = _free_joint_dofadr(model)
    v = np.asarray(data.qvel[v0:v0 + 3], dtype=np.float64).copy()
    return com[:2], v[:2], float(com[2])


def capture_point_xy(com_xy: np.ndarray, com_vxy: np.ndarray, omega: float) -> np.ndarray:
    """DCM / capture point: xi = com + v / omega. omega ≈ √(g/z_com)."""
    w = max(float(omega), 0.5)
    return com_xy + com_vxy / w


def apply_cp_swing_placement(
    model: mj.MjModel,
    data: mj.MjData,
    act_idx: dict[str, int],
    phi: float,
    amp: float,
    bid_lf: int,
    bid_rf: int,
):
    """Phase B: bias swing hip pitch so foot lands under/beyond CP (ankle CoP remains trim).

    No horizontal free-joint force. Placement via joint-space fwd bias within HX clips.
    """
    if amp < 0.05 or not USE_CP_SWING:
        return
    com_xy, com_vxy, com_z = estimate_com_state(model, data)
    omega = CP_OMEGA if CP_OMEGA > 0.1 else max(0.5, math.sqrt(9.81 / max(com_z, 0.12)))
    cp = capture_point_xy(com_xy, com_vxy, omega)

    ds_frac = clamp(DS_S / max(GAIT_T, 1e-3), 0.08, 0.55)  # was 0.35; dual-T reopt needs DS duty room
    stance_end = 0.50
    ds_end = stance_end + ds_frac
    swing_len = max(0.18, 1.0 - ds_end)

    for side, bid in (("L", bid_lf), ("R", bid_rf)):
        p = phase_leg(phi, side)
        if p < ds_end:
            continue  # not swinging
        s = (p - ds_end) / swing_len
        if s < 0.15 or s > 0.95:
            continue
        # Sagittal CP placement (B02-winning): foot → under/at CP
        foot_x = float(data.xpos[bid, 0])
        alpha = float(np.clip(CP_STEP_GAIN, 0.4, 1.2))
        des_x = float(com_xy[0] + alpha * (cp[0] - com_xy[0]))
        err = des_x - foot_x
        d_fwd = float(np.clip(err / 0.20, -CP_MAX_STEP / 0.20, CP_MAX_STEP / 0.20)) * amp
        w = swing_blend(min(1.0, s / 0.7))
        d_fwd *= w
        d_lat = 0.0
        pref = "l_" if side == "L" else "r_"
        # L hip_j = -fwd, R hip_j = +fwd
        sign = -1.0 if side == "L" else 1.0
        an = act_name(f"{pref}hip_pitch")
        if an in act_idx:
            data.ctrl[act_idx[an]] = float(np.clip(
                data.ctrl[act_idx[an]] + sign * d_fwd, -1.5, 1.5))
        an_a = act_name(f"{pref}ank_pitch")
        if an_a in act_idx:
            data.ctrl[act_idx[an_a]] = float(np.clip(
                data.ctrl[act_idx[an_a]] + sign * 0.5 * d_fwd, -1.2, 1.2))


def apply_hip_strategy(
    model: mj.MjModel,
    data: mj.MjData,
    act_idx: dict[str, int],
    amp: float,
    prev_com_vxy: np.ndarray | None,
    dt: float,
) -> np.ndarray:
    """Phase C: hip pitch/roll share for residual COM accel within ankle budget leftover."""
    com_xy, com_vxy, _ = estimate_com_state(model, data)
    if prev_com_vxy is None or not USE_HIP_STRAT or amp < 0.05:
        return com_vxy
    acc = (com_vxy - prev_com_vxy) / max(dt, 1e-3)
    # oppose COM accel with hip (signed fwd / lat)
    d_fwd = float(np.clip(-HIP_STRAT_KP * acc[0], -HIP_STRAT_CLIP, HIP_STRAT_CLIP)) * amp
    d_lat = float(np.clip(-HIP_STRAT_KP * 1.2 * acc[1], -HIP_STRAT_CLIP, HIP_STRAT_CLIP)) * amp
    for side, s_fwd in (("l", -1.0), ("r", +1.0)):
        an = act_name(f"{side}_hip_pitch")
        if an in act_idx:
            data.ctrl[act_idx[an]] = float(np.clip(
                data.ctrl[act_idx[an]] + s_fwd * d_fwd, -1.5, 1.5))
    # common-mode roll (same convention as gait shift)
    for jn, delta in (
        ("l_hip_roll", -d_lat),
        ("r_hip_roll", -d_lat),
        ("l_ank_roll", -0.5 * d_lat),
        ("r_ank_roll", -0.5 * d_lat),
    ):
        an = act_name(jn)
        if an in act_idx:
            data.ctrl[act_idx[an]] = float(np.clip(data.ctrl[act_idx[an]] + delta, -1.5, 1.5))
    return com_vxy


def apply_capture_step(
    model: mj.MjModel,
    data: mj.MjData,
    act_idx: dict[str, int],
    amp: float,
    tip_latched: dict,
):
    """Phase C: on tip detect, force a larger step toward CP for one half-cycle."""
    if not USE_CAPTURE_STEP or amp < 0.05:
        return
    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    R = data.xmat[bid].reshape(3, 3)
    up_z = float(R[2, 2])
    up_x = float(R[0, 2])
    up_y = float(R[1, 2])
    tipping = up_z < CAPTURE_UP_Z or abs(up_x) > 0.28 or abs(up_y) > 0.28
    if tipping and not tip_latched.get("active"):
        tip_latched["active"] = True
        tip_latched["hold"] = 0.55  # s
        tip_latched["dir_x"] = 1.0 if up_x >= 0 else -1.0
        tip_latched["dir_y"] = 1.0 if up_y >= 0 else -1.0
        print(f"[ainex] CAPTURE STEP latch up_z={up_z:.2f} up_xy=({up_x:+.2f},{up_y:+.2f})")
    if not tip_latched.get("active"):
        return
    tip_latched["hold"] = float(tip_latched.get("hold", 0.0)) - 1.0 / CTRL_HZ
    if tip_latched["hold"] <= 0:
        tip_latched["active"] = False
        return
    # push both hips toward fall direction (step out)
    d_fwd = CAPTURE_STEP_M / 0.20 * float(tip_latched.get("dir_x", 1.0))
    d_lat = 0.12 * float(tip_latched.get("dir_y", 0.0))
    for side, s_fwd in (("l", -1.0), ("r", +1.0)):
        an = act_name(f"{side}_hip_pitch")
        if an in act_idx:
            data.ctrl[act_idx[an]] = float(np.clip(
                data.ctrl[act_idx[an]] + s_fwd * d_fwd * 0.5, -1.5, 1.5))
    for jn, delta in (
        ("l_hip_roll", -d_lat),
        ("r_hip_roll", -d_lat),
    ):
        an = act_name(jn)
        if an in act_idx:
            data.ctrl[act_idx[an]] = float(np.clip(data.ctrl[act_idx[an]] + delta, -1.5, 1.5))


def roll_servo(model: mj.MjModel, data: mj.MjData, act_idx: dict[str, int], amp: float):
    """Joint-space ankle/hip roll from body roll — recovery without free-joint quat lock.

    body roll ≈ atan2(R[2,1], R[2,2]) small-angle: R[1,2] of up vector y-component.
    Positive body roll (top toward +Y/L) → push ankles to recover toward center.
    """
    if amp <= 0.01:
        return
    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    R = data.xmat[bid].reshape(3, 3)
    # up vector world y component: lean toward +Y if up_y > 0
    up_y = float(R[1, 2])
    up_z = float(R[2, 2])
    if up_z < 0.5:
        return
    # recover: if leaning +Y, command hip/ank rolls that shift COM toward -Y
    # Using same common-mode convention as gait (s = -amp*lat with lat toward L=+Y)
    # leaning +Y → want lat negative → shift = -amp*(-1) = +amp in old? 
    # With corrected: shift = -COM * lat; lat_des = -k*up_y (lean +Y → lat toward R)
    k = 0.85
    lat_des = float(np.clip(-k * up_y / 0.10, -1.0, 1.0))
    s = -0.16 * amp * lat_des  # additive recovery on top of CPG
    for jn, delta in (
        ("l_hip_roll", -s),
        ("r_hip_roll", -s),
        ("l_ank_roll", -0.8 * s),
        ("r_ank_roll", -0.8 * s),
    ):
        an = act_name(jn)
        if an in act_idx:
            data.ctrl[act_idx[an]] = float(np.clip(data.ctrl[act_idx[an]] + delta, -1.5, 1.5))


def pitch_servo(model: mj.MjModel, data: mj.MjData, act_idx: dict[str, int], amp: float):
    """Joint-space ankle/hip pitch from body sagittal lean (no free-joint wrench)."""
    if amp <= 0.01:
        return
    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    R = data.xmat[bid].reshape(3, 3)
    up_x = float(R[0, 2])  # lean toward +X if >0
    up_z = float(R[2, 2])
    if up_z < 0.5:
        return
    # lean +X (forward) → plantarflex ankles / extend hips slightly (signed fwd space)
    k = 0.45
    corr = float(np.clip(-k * up_x / 0.12, -0.20, 0.20)) * amp  # signed "fwd" correction
    # L: hip = -fwd, R: hip = +fwd; ankles follow flat-ish
    for side, sign in (("l", -1.0), ("r", +1.0)):
        for jn, scale in ((f"{side}_hip_pitch", 1.0), (f"{side}_ank_pitch", 0.7)):
            an = act_name(jn)
            if an in act_idx:
                data.ctrl[act_idx[an]] = float(np.clip(
                    data.ctrl[act_idx[an]] + sign * scale * corr, -1.5, 1.5))

def _support_feet(
    model: mj.MjModel,
    data: mj.MjData,
    bid_lf: int,
    bid_rf: int,
    gid_floor: int,
    lat: float | None = None,
) -> tuple[np.ndarray, dict[str, float], bool, bool]:
    """Return (support_xy, side_weights, cL, cR) for CoP / freeze.

    lat: gait lateral target (+ → prefer L). None = pure contact weighting.
    """
    cL = foot_floor_contact(model, data, bid_lf, gid_floor)
    cR = foot_floor_contact(model, data, bid_rf, gid_floor)
    pL = data.xpos[bid_lf, :2].copy()
    pR = data.xpos[bid_rf, :2].copy()
    wL = wR = 0.0
    if cL and cR:
        wL = wR = 0.5
        if lat is not None:
            # shift weight toward primary stance
            if lat > 0.15:
                wL, wR = 0.75, 0.25
            elif lat < -0.15:
                wL, wR = 0.25, 0.75
    elif cL:
        wL = 1.0
    elif cR:
        wR = 1.0
    else:
        # airborne soft mid-feet reference
        wL = wR = 0.35
    support = (wL * pL + wR * pR) / max(wL + wR, 1e-6)
    return support, {"l": wL, "r": wR}, cL, cR


def ankle_cop_servo(
    model: mj.MjModel,
    data: mj.MjData,
    act_idx: dict[str, int],
    amp: float,
    bid_lf: int,
    bid_rf: int,
    gid_floor: int,
    lat: float | None = None,
    bias_xy: np.ndarray | None = None,
) -> dict[str, float]:
    """Ankle-strategy balance servo @ CTRL_HZ (assist OFF).

    Lean-primary (body up-vector + gyro) plus relative COM-xy vs support-foot
    xy after subtracting a settle bias. Absolute COM-foot without bias was
    observed to *destabilize* quiet stand under HX ±2.1 Nm position actuators.

    Sign convention (probed / matches prior lean servos):
      lean +X / COM ahead → L +ank_pitch, R -ank_pitch (plantarflex)
      lean +Y → both ank_roll negative (CoP toward +Y)
    HX forcerange clips torque; this only writes position setpoints.
    """
    diag = {
        "err_x": 0.0, "err_y": 0.0, "wL": 0.0, "wR": 0.0,
        "cL": 0.0, "cR": 0.0, "up_x": 0.0, "up_y": 0.0,
    }
    if amp <= 0.01:
        return diag

    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    R = data.xmat[bid].reshape(3, 3)
    up_x = float(R[0, 2])
    up_y = float(R[1, 2])
    up_z = float(R[2, 2])
    diag["up_x"] = up_x
    diag["up_y"] = up_y
    if up_z < 0.35:
        return diag

    v0 = _free_joint_dofadr(model)
    omega = np.asarray(data.qvel[v0 + 3:v0 + 6], dtype=np.float64)  # free-joint angular vel
    # Lean PD → plantarflex / roll restore
    pitch_lean = ANK_COP_KP_LEAN_X * up_x + ANK_COP_KD_LEAN_X * float(omega[1])
    roll_lean = -(ANK_COP_KP_LEAN_Y * up_y + ANK_COP_KD_LEAN_Y * float(omega[0]))

    com = np.asarray(data.subtree_com[0], dtype=np.float64)
    support, weights, cL, cR = _support_feet(model, data, bid_lf, bid_rf, gid_floor, lat)
    err = com[:2] - support
    if bias_xy is not None:
        err = err - np.asarray(bias_xy, dtype=np.float64)
    # deadzone
    err_dz = err.copy()
    for i in range(2):
        if abs(err_dz[i]) < ANK_COP_DEADZONE:
            err_dz[i] = 0.0

    pitch_cop = ANK_COP_KP_X * float(err_dz[0])
    roll_cop = -(ANK_COP_KP_Y * float(err_dz[1]))

    pitch = float(np.clip((pitch_lean + pitch_cop) * amp, -ANK_COP_CLIP, ANK_COP_CLIP))
    roll = float(np.clip((roll_lean + roll_cop) * amp, -ANK_COP_CLIP, ANK_COP_CLIP))

    diag.update({
        "err_x": float(err[0]), "err_y": float(err[1]),
        "wL": float(weights["l"]), "wR": float(weights["r"]),
        "cL": float(cL), "cR": float(cR),
        "sup_x": float(support[0]), "sup_y": float(support[1]),
        "com_x": float(com[0]), "com_y": float(com[1]),
        "pitch_cmd": pitch, "roll_cmd": roll,
    })

    for side, w in weights.items():
        if w <= 1e-6:
            continue
        d_pitch = (+pitch if side == "l" else -pitch) * w
        d_roll = roll * w
        for jn, delta in (
            (f"{side}_ank_pitch", d_pitch),
            (f"{side}_ank_roll", d_roll),
            (f"{side}_hip_roll", ANK_COP_HIP_SHARE * d_roll),
        ):
            an = act_name(jn)
            if an in act_idx:
                data.ctrl[act_idx[an]] = float(np.clip(
                    data.ctrl[act_idx[an]] + delta, -1.8, 1.8))
    return diag


def stance_foot_freeze(
    model: mj.MjModel,
    data: mj.MjData,
    hold: dict[str, np.ndarray | None],
    amp: float,
    bid_lf: int,
    bid_rf: int,
    gid_floor: int,
    lat: float,
    sticky: bool = False,
):
    """Ablation: spring-damper hold of primary stance foot xy in world.

    External xfrc on foot body — disclosed; separates balance from slip.
    Captures hold pose on first contact; clears on lift-off unless sticky
    (used for disturb-stand so a brief hop does not drop the plant).
    """
    if amp <= 0.01:
        return
    cL = foot_floor_contact(model, data, bid_lf, gid_floor)
    cR = foot_floor_contact(model, data, bid_rf, gid_floor)
    targets: list[tuple[str, int, bool]] = []
    if lat >= 0.0:
        targets.append(("L", bid_lf, cL))
    if lat <= 0.0:
        targets.append(("R", bid_rf, cR))
    if abs(lat) < 0.20:
        targets = [("L", bid_lf, cL), ("R", bid_rf, cR)]

    for side, bid, contact in targets:
        key = side
        if not contact and not sticky:
            hold[key] = None
            continue
        if hold.get(key) is None:
            # capture even if briefly airborne under sticky (use current xy)
            hold[key] = data.xpos[bid, :2].copy()
        if hold.get(key) is None:
            continue
        target = hold[key]
        pos = data.xpos[bid, :2]
        vel = data.cvel[bid][3:5]
        fx = float(np.clip(-FREEZE_KP * (pos[0] - target[0]) - FREEZE_KD * vel[0],
                           -FREEZE_FMAX, FREEZE_FMAX))
        fy = float(np.clip(-FREEZE_KP * (pos[1] - target[1]) - FREEZE_KD * vel[1],
                           -FREEZE_FMAX, FREEZE_FMAX))
        data.xfrc_applied[bid, 0] += fx * amp
        data.xfrc_applied[bid, 1] += fy * amp



def apply_auth_envelope(model: mj.MjModel, k: float) -> dict:
    """Scale leg HX forcerange/ctrlrange/jnt limits by k. Arms/head unchanged.
    Returns diag with base/scaled limits. Does not rewrite XML on disk.
    """
    global AUTH_K, AUTH_LEG_ACT_IDS
    AUTH_K = float(k)
    leg_tau0, leg_pos0 = 2.1, 2.09
    tau = leg_tau0 * AUTH_K
    pos = leg_pos0 * AUTH_K
    _residual_mod.AUTH_CTRL_LIM = pos
    AUTH_LEG_ACT_IDS = []
    for i in range(model.nu):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        nl = name.lower()
        is_leg = any(x in nl for x in ("hip_", "knee", "ank_"))
        if not is_leg:
            continue
        AUTH_LEG_ACT_IDS.append(i)
        # ctrlrange
        model.actuator_ctrlrange[i, 0] = -pos
        model.actuator_ctrlrange[i, 1] = pos
        # forcerange
        model.actuator_forcerange[i, 0] = -tau
        model.actuator_forcerange[i, 1] = tau
    for j in range(model.njnt):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, j) or ""
        nl = name.lower()
        if not any(x in nl for x in ("hip_", "knee", "ank_")):
            continue
        # joint range (position)
        if model.jnt_limited[j]:
            model.jnt_range[j, 0] = -pos
            model.jnt_range[j, 1] = pos
        # actuator force range on joint if present
        if hasattr(model, "jnt_actfrcrange"):
            model.jnt_actfrcrange[j, 0] = -tau
            model.jnt_actfrcrange[j, 1] = tau
        if hasattr(model, "jnt_actfrclimited"):
            model.jnt_actfrclimited[j] = 1
        # MuJoCo 3: actuatorfrcrange stored as jnt_actfrclimited / dof_frictionloss etc.
        # Also patch actuatorfrcrange via joint's actuatorfrcrange if available
    # joint actuatorfrcrange attribute (MJCF actuatorfrcrange on joint)
    if hasattr(model, "jnt_actfrcrange"):
        pass
    # Some builds expose as actuator_forcerange only — already done.
    # Also scale dof actuator force limits if present
    print(f"[ainex] AUTH envelope k={AUTH_K:.3f} leg_tau=±{tau:.3f} Nm leg_pos=±{pos:.3f} rad "
          f"(n_leg_act={len(AUTH_LEG_ACT_IDS)})")
    return {"k": AUTH_K, "leg_tau": tau, "leg_pos": pos, "n_leg_act": len(AUTH_LEG_ACT_IDS)}

def set_ctrl(model: mj.MjModel, data: mj.MjData, qdes: dict[str, float], act_idx: dict[str, int]):
    for jname, val in qdes.items():
        an = act_name(jname)
        if an in act_idx:
            data.ctrl[act_idx[an]] = val


def body_sagittal_pitch_x(data: mj.MjData, bid: int) -> float:
    R = data.xmat[bid].reshape(3, 3)
    down = -R[:, 2]
    return math.atan2(float(down[0]), float(-down[2]))


def _burn_overlay(img: np.ndarray, lines: list[str]) -> np.ndarray:
    from PIL import Image, ImageDraw, ImageFont

    out = Image.fromarray(img)
    draw = ImageDraw.Draw(out)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 13)
        font_sm = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 11)
    except Exception:
        font = ImageFont.load_default()
        font_sm = font
    y = 6
    for i, line in enumerate(lines):
        f = font if i == 0 else font_sm
        draw.text((7, y + 1), line, fill=(0, 0, 0), font=f)
        draw.text((6, y), line, fill=(255, 230, 80), font=f)
        y += 15 if i == 0 else 13
    return np.asarray(out, dtype=np.uint8)


def _encode_mp4_ffmpeg(frame_dir: Path, out_mp4: Path, fps: int = 25) -> None:
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    pattern = str(frame_dir / "frame_%05d.png")
    cmd = [
        "ffmpeg", "-y",
        "-framerate", str(fps),
        "-i", pattern,
        "-c:v", "libx264",
        "-profile:v", "baseline",
        "-level", "3.0",
        "-pix_fmt", "yuv420p",
        "-bf", "0",
        "-g", "15",
        "-keyint_min", "15",
        "-preset", "veryfast",
        "-crf", "20",
        "-movflags", "+faststart",
        str(out_mp4),
    ]
    print(f"[ainex] ffmpeg: {' '.join(cmd)}")
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stderr[-2000:], file=sys.stderr)
        raise RuntimeError(f"ffmpeg failed rc={r.returncode}")


def _sha256_first_1kb(data: bytes) -> str:
    return hashlib.sha256(data[:1024]).hexdigest()


def _zero_cross_cycles(sig: np.ndarray, level: float = 0.0) -> int:
    if len(sig) < 3:
        return 0
    d = sig - level
    zc = int(np.sum((d[1:] * d[:-1]) < 0))
    return zc // 2


def _count_lift_peaks(z: np.ndarray, thr: float = 0.012) -> tuple[int, float]:
    if len(z) < 5:
        return 0, 0.0
    z0 = float(np.percentile(z, 8))
    lift = z - z0
    n = 0
    for i in range(2, len(lift) - 2):
        if lift[i] > thr and lift[i] >= max(lift[i - 1], lift[i + 1], lift[i - 2], lift[i + 2]):
            n += 1
    return n, float(np.max(lift))


def foot_floor_contact(model, data, bid_foot, gid_floor) -> bool:
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        b1 = int(model.geom_bodyid[g1])
        b2 = int(model.geom_bodyid[g2])
        if (b1 == bid_foot or b2 == bid_foot) and (g1 == gid_floor or g2 == gid_floor):
            return True
    return float(data.xpos[bid_foot, 2]) < CONTACT_Z_THR


def main():
    ap = argparse.ArgumentParser(description="AiNex kit-matched open-loop walk @ 50 Hz (plant+COM+ankle CoP)")
    ap.add_argument("--model", type=str, default=str(XML_DEFAULT))
    ap.add_argument("--duration", type=float, default=7.0)
    ap.add_argument("--out", type=str, default=str(OUT_DIR / "ainex_walk.mp4"))
    ap.add_argument("--gait-off", action="store_true")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--assist", action="store_true",
                    help="Enable upright+height+speed balance assist (NOT clean walk)")
    ap.add_argument("--no-plant", action="store_true", help="Disable stance plant damper")
    ap.add_argument("--ankle-cop", dest="ankle_cop", action="store_true", default=True,
                    help="Enable ankle-strategy CoP servo (default ON)")
    ap.add_argument("--no-ankle-cop", dest="ankle_cop", action="store_false",
                    help="Disable ankle CoP servo")
    ap.add_argument("--stance-freeze", action="store_true",
                    help="Ablation: world-frame stance-foot xy freeze during support")
    ap.add_argument("--disturb-stand", action="store_true",
                    help="Stand + push disturbance; ankle CoP only (prove recovery)")
    ap.add_argument("--push-vy", type=float, default=0.0,
                    help="Lateral velocity nudge (m/s) at disturb time")
    ap.add_argument("--push-vx", type=float, default=0.0,
                    help="Sagittal velocity nudge (m/s) at disturb time")
    ap.add_argument("--push-fy", type=float, default=3.8,
                    help="Lateral force pulse (N) at free joint during disturb")
    ap.add_argument("--push-fx", type=float, default=0.0,
                    help="Sagittal force pulse (N) during disturb")
    ap.add_argument("--push-hold", type=float, default=0.60,
                    help="Force pulse duration (s)")
    ap.add_argument("--fx-bias", type=float, default=0.0)
    ap.add_argument("--v-des", type=float, default=V_DES_X, help="forward speed target m/s (assist only)")
    ap.add_argument("--tag", type=str, default="ankle_cop", help="label for note/stats")
    ap.add_argument("--gait-t", type=float, default=None, help="gait period T (s)")
    ap.add_argument("--hip-amp", type=float, default=None, help="hip pitch amp (rad)")
    ap.add_argument("--hip-bias", type=float, default=None, help="hip forward bias (rad)")
    ap.add_argument("--step-len", type=float, default=None, help="step length cmd (m); maps to hip amp ~ step/0.25")
    ap.add_argument("--ds", type=float, default=None, help="double-support hold (s)")
    ap.add_argument("--com-shift", type=float, default=None, help="lateral hip_roll amp (rad)")
    ap.add_argument("--com-shift-lead", type=float, default=None, help="COM shift phase lead (frac)")
    ap.add_argument("--knee-stance", type=float, default=None)
    ap.add_argument("--knee-swing", type=float, default=None)
    ap.add_argument("--swing-abduct", type=float, default=None, help="Phase F: swing hip_roll abduct amp (rad)")
    ap.add_argument("--swing-ank-df", type=float, default=None, help="Phase F: swing ankle dorsiflex amp (rad)")
    ap.add_argument("--com-z", type=float, default=None, help="stand / gait height (m)")
    ap.add_argument("--plant-kd", type=float, default=None)
    ap.add_argument("--plant-fmax", type=float, default=None)
    ap.add_argument("--stats-out", type=str, default=None, help="optional extra stats JSON path")
    ap.add_argument("--cp-swing", action="store_true", help="Phase B: CP/DCM swing placement")
    ap.add_argument("--hip-strat", action="store_true", help="Phase C: hip strategy for COM accel")
    ap.add_argument("--capture-step", action="store_true", help="Phase C: reactive capture step on tip")
    ap.add_argument("--cp-gain", type=float, default=None, help="CP placement alpha (0.4-1.2)")
    ap.add_argument("--cp-omega", type=float, default=None, help="DCM omega rad/s")
    ap.add_argument("--vnull", action="store_true", help="Enable stance vx null servo")
    ap.add_argument("--no-vnull", action="store_true", help="Disable stance vx null servo")
    ap.add_argument("--vnull-k", type=float, default=None)
    ap.add_argument("--stance-sweep", type=float, default=None,
                    help="stance hip travel frac of hip amp (default 0.55)")
    ap.add_argument("--stance-vik", action="store_true",
                    help="Phase D: Jacobian stance-foot velocity IK (vx,vy→0)")
    ap.add_argument("--vik-kp", type=float, default=None, help="stance VIK KP (1/s)")
    ap.add_argument("--vik-clip", type=float, default=None, help="stance VIK rad/step clip")
    ap.add_argument("--vik-damp", type=float, default=None, help="stance VIK damped-LS ridge")
    ap.add_argument("--vik-omega", type=float, default=None, help="stance VIK yaw-rate weight")
    ap.add_argument("--zmp-qp", action="store_true",
                    help="Phase D: tiny CP/ZMP ankle-hip trim @ 50 Hz")
    ap.add_argument("--zmp-kp", type=float, default=None)
    ap.add_argument("--wbc-stance", action="store_true",
                    help="Stance friction-cone wrench QP (WBC-lite); after CP/VIK")
    ap.add_argument("--wbc-mu", type=float, default=None,
                    help="Friction cone μ for WBC QP (default 0.8)")
    ap.add_argument("--wbc-scrub", type=float, default=None,
                    help="WBC skate-scrub gain N/(m/s)")
    ap.add_argument("--wbc-trim-clip", type=float, default=None,
                    help="WBC Δctrl clip rad")
    ap.add_argument("--wbc-tau-scale", type=float, default=None)
    ap.add_argument("--wbc-w-scrub", type=float, default=None)
    ap.add_argument("--hybrid-mpc", action="store_true",
                    help="Hybrid short-horizon MPC with T88 hard-reject in-loop")
    ap.add_argument("--mpc-n", type=int, default=None, help="MPC horizon steps")
    ap.add_argument("--mpc-u-max", type=float, default=None, help="MPC |Δctrl| max rad")
    ap.add_argument("--mpc-apply-thr", type=float, default=None,
                    help="Only apply when |vx| or rolling mean above thr")
    ap.add_argument("--mpc-disable-above-t", type=float, default=None,
                    help="Hard-disable MPC when gait_t >= this (CSF50 protect)")
    ap.add_argument("--fric", type=float, default=None,
                    help="override floor+foot sliding friction (both feet + floor)")
    ap.add_argument("--auth-k", type=float, default=None,
                    help="Scale HX leg torque ±2.1 and position ±2.09 by k (sim envelope)")
    ap.add_argument("--vik-ankle", action="store_true",
                    help="Restrict stance VIK to ankle DOFs only")
    ap.add_argument("--stance-hold", action="store_true",
                    help="Hold stance hip/knee setpoints at touchdown (joint-space)")
    ap.add_argument("--foot-pos-ik", action="store_true",
                    help="Phase D: Jacobian position IK hold stance foot world xy")
    ap.add_argument("--foot-ik-kp", type=float, default=None)
    ap.add_argument("--foot-ik-clip", type=float, default=None)
    ap.add_argument("--residual", type=str, default="none",
                    help="Residual stance-vx npz PATH, or none")
    ap.add_argument("--residual-gain", type=float, default=1.0,
                    help="Scalar gain on residual Δctrl")
    args = ap.parse_args()

    # Apply CLI overrides to module globals (gait_targets reads them)
    global GAIT_T, STEP_LEN, HIP_PITCH_AMP, HIP_BIAS_FWD, KNEE_STANCE, KNEE_SWING
    global COM_Z, COM_SHIFT_AMP, COM_SHIFT_LEAD, DS_S, PLANT_KD, PLANT_MAX_F
    global USE_CP_SWING, USE_HIP_STRAT, USE_CAPTURE_STEP
    global USE_RESIDUAL_STANCE_VX, RESIDUAL_FIT, RESIDUAL_GAIN, RESIDUAL_PATH
    if args.gait_t is not None:
        GAIT_T = float(args.gait_t)
    if args.step_len is not None:
        STEP_LEN = float(args.step_len)
        if args.hip_amp is None:
            # rough geometric map: hip amp ≈ step / (2 * effective leg lever ~0.12)
            HIP_PITCH_AMP = float(np.clip(STEP_LEN / 0.25, 0.06, 0.35))
    if args.hip_amp is not None:
        HIP_PITCH_AMP = float(args.hip_amp)
    if args.hip_bias is not None:
        HIP_BIAS_FWD = float(args.hip_bias)
    if args.ds is not None:
        DS_S = float(args.ds)
    if args.com_shift is not None:
        COM_SHIFT_AMP = float(args.com_shift)
    if args.com_shift_lead is not None:
        COM_SHIFT_LEAD = float(args.com_shift_lead)
    if args.knee_stance is not None:
        KNEE_STANCE = float(args.knee_stance)
    if args.knee_swing is not None:
        KNEE_SWING = float(args.knee_swing)
    global SWING_ABDUCT, SWING_ANK_DF
    if getattr(args, "swing_abduct", None) is not None:
        SWING_ABDUCT = float(args.swing_abduct)
    if getattr(args, "swing_ank_df", None) is not None:
        SWING_ANK_DF = float(args.swing_ank_df)
    if args.com_z is not None:
        COM_Z = float(args.com_z)
    if args.plant_kd is not None:
        PLANT_KD = float(args.plant_kd)
    if args.plant_fmax is not None:
        PLANT_MAX_F = float(args.plant_fmax)
    USE_CP_SWING = bool(args.cp_swing)
    USE_HIP_STRAT = bool(args.hip_strat)
    USE_CAPTURE_STEP = bool(args.capture_step)
    global CP_STEP_GAIN, CP_OMEGA
    if args.cp_gain is not None:
        CP_STEP_GAIN = float(args.cp_gain)
    if args.cp_omega is not None:
        CP_OMEGA = float(args.cp_omega)
    global USE_STANCE_V_NULL, STANCE_V_NULL_K
    if getattr(args, "vnull", False):
        USE_STANCE_V_NULL = True
    if args.vnull_k is not None:
        STANCE_V_NULL_K = float(args.vnull_k)
        USE_STANCE_V_NULL = True
    if args.no_vnull:
        USE_STANCE_V_NULL = False
    global STANCE_SWEEP_FRAC
    if args.stance_sweep is not None:
        STANCE_SWEEP_FRAC = float(args.stance_sweep)
    global USE_STANCE_VIK, STANCE_VIK_KP, STANCE_VIK_CLIP, STANCE_VIK_KDAMP, STANCE_VIK_W_OMEGA
    global USE_ZMP_QP, ZMP_QP_KP
    if args.stance_vik:
        USE_STANCE_VIK = True
    if args.vik_kp is not None:
        STANCE_VIK_KP = float(args.vik_kp)
    if args.vik_clip is not None:
        STANCE_VIK_CLIP = float(args.vik_clip)
    if args.vik_damp is not None:
        STANCE_VIK_KDAMP = float(args.vik_damp)
    if args.vik_omega is not None:
        STANCE_VIK_W_OMEGA = float(args.vik_omega)
    if args.zmp_qp:
        USE_ZMP_QP = True
    if args.zmp_kp is not None:
        ZMP_QP_KP = float(args.zmp_kp)
    global USE_WBC_STANCE, WBC_MU, WBC_SCRUB_K, WBC_TRIM_CLIP, WBC_TAU_SCALE, WBC_W_SCRUB
    if getattr(args, "wbc_stance", False):
        USE_WBC_STANCE = True
    if getattr(args, "wbc_mu", None) is not None:
        WBC_MU = float(args.wbc_mu)
        USE_WBC_STANCE = True
    if getattr(args, "wbc_scrub", None) is not None:
        WBC_SCRUB_K = float(args.wbc_scrub)
    if getattr(args, "wbc_trim_clip", None) is not None:
        WBC_TRIM_CLIP = float(args.wbc_trim_clip)
    if getattr(args, "wbc_tau_scale", None) is not None:
        WBC_TAU_SCALE = float(args.wbc_tau_scale)
    if getattr(args, "wbc_w_scrub", None) is not None:
        WBC_W_SCRUB = float(args.wbc_w_scrub)
    global USE_HYBRID_MPC, MPC_N, MPC_U_MAX, MPC_APPLY_THR, MPC_DISABLE_ABOVE_T, MPC_ANK_RATIO
    if getattr(args, "hybrid_mpc", False):
        USE_HYBRID_MPC = True
        reset_mpc_state()
    if getattr(args, "mpc_n", None) is not None:
        MPC_N = int(args.mpc_n)
        USE_HYBRID_MPC = True
    if getattr(args, "mpc_u_max", None) is not None:
        MPC_U_MAX = float(args.mpc_u_max)
        USE_HYBRID_MPC = True
    if getattr(args, "mpc_apply_thr", None) is not None:
        MPC_APPLY_THR = float(args.mpc_apply_thr)
    if getattr(args, "mpc_disable_above_t", None) is not None:
        MPC_DISABLE_ABOVE_T = float(args.mpc_disable_above_t)
    global STANCE_VIK_ANKLE_ONLY, USE_STANCE_HOLD
    if args.vik_ankle:
        STANCE_VIK_ANKLE_ONLY = True
    if args.stance_hold:
        USE_STANCE_HOLD = True
    global USE_FOOT_POS_IK, FOOT_POS_IK_KP, FOOT_POS_IK_CLIP
    if args.foot_pos_ik:
        USE_FOOT_POS_IK = True
    if args.foot_ik_kp is not None:
        FOOT_POS_IK_KP = float(args.foot_ik_kp)
    if args.foot_ik_clip is not None:
        FOOT_POS_IK_CLIP = float(args.foot_ik_clip)

    RESIDUAL_GAIN = float(args.residual_gain)
    RESIDUAL_PATH = (args.residual or "none").strip()
    if RESIDUAL_PATH.lower() in ("", "none", "off", "false", "0"):
        USE_RESIDUAL_STANCE_VX = False
        RESIDUAL_FIT = None
        RESIDUAL_PATH = "none"
    else:
        rpath = Path(RESIDUAL_PATH)
        if not rpath.is_file():
            print(f"ERROR: --residual not found: {rpath}", file=sys.stderr)
            return 2
        RESIDUAL_FIT = load_residual_npz(rpath)
        USE_RESIDUAL_STANCE_VX = True
        _rmode = str(RESIDUAL_FIT.get("mode", "v1_linear"))
        print(f"[ainex] residual ON mode={_rmode} path={rpath} gain={RESIDUAL_GAIN} "
              f"n_rows={int(RESIDUAL_FIT.get('n_rows', -1))}")

    xml = Path(args.model)
    if not xml.exists():
        print(f"ERROR: missing {xml}", file=sys.stderr)
        sys.exit(1)

    out_dir = Path(args.out).parent
    out_dir.mkdir(parents=True, exist_ok=True)
    HIWONDER_DIR.mkdir(parents=True, exist_ok=True)
    frame_seq = out_dir / "frame_seq"
    if frame_seq.exists():
        shutil.rmtree(frame_seq)
    frame_seq.mkdir(parents=True, exist_ok=True)

    model = mj.MjModel.from_xml_path(str(xml))
    data = mj.MjData(model)
    if args.fric is not None:
        mu = float(args.fric)
        for gname in ("floor", "l_foot_contact", "r_foot_contact"):
            gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, gname)
            if gid >= 0:
                model.geom_friction[gid, 0] = mu
        print(f"[ainex] friction override mu_slide={mu}")
    global AUTH_K, AUTH_SAT_COUNT, AUTH_SAT_STEPS, AUTH_LEG_ACT_IDS
    AUTH_SAT_COUNT = 0
    AUTH_SAT_STEPS = 0
    AUTH_LEG_ACT_IDS = []
    _residual_mod.AUTH_CTRL_LIM = 2.09
    AUTH_K = 1.0
    if getattr(args, "auth_k", None) is not None:
        apply_auth_envelope(model, float(args.auth_k))
    elif abs(AUTH_K - 1.0) > 1e-12:
        apply_auth_envelope(model, AUTH_K)
    # Even at k=1.0, populate leg act ids for saturation logging at kit HX
    if not AUTH_LEG_ACT_IDS:
        for i in range(model.nu):
            name = (mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or "").lower()
            if any(x in name for x in ("hip_", "knee", "ank_")):
                AUTH_LEG_ACT_IDS.append(i)
    act_idx = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(model.nu)}

    needed = [
        "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
        "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
        "l_sho_pitch", "l_sho_roll", "l_el_pitch", "r_sho_pitch", "r_sho_roll", "r_el_pitch",
        "head_pan", "head_tilt",
    ]
    missing = [act_name(n) for n in needed if act_name(n) not in act_idx]
    if missing:
        print(f"ERROR: missing actuators {missing}", file=sys.stderr)
        sys.exit(1)

    mass = float(mj.mj_getTotalmass(model))
    data.qpos[:] = 0
    data.qpos[2] = COM_Z
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[0] = -0.12
    q0 = gait_targets(0.0, False, 0.0)
    for jn, val in q0.items():
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = val
    set_ctrl(model, data, q0, act_idx)
    mj.mj_forward(model, data)

    ctrl_dt = 1.0 / CTRL_HZ
    sim_dt = model.opt.timestep
    steps_per_ctrl = max(1, int(round(ctrl_dt / sim_dt)))
    gait_on = not args.gait_off
    stand_hold = 0.80
    ramp_t = 1.40
    use_assist = bool(args.assist)
    use_plant = not args.no_plant
    use_ankle_cop = bool(args.ankle_cop)
    use_stance_freeze = bool(args.stance_freeze)
    disturb_stand = bool(args.disturb_stand)
    fx_bias = float(args.fx_bias)
    v_des_x = float(args.v_des)
    if disturb_stand:
        # Prove ankle CoP alone: gait off, plant off, assist off, freeze off
        gait_on = False
        use_assist = False
        use_plant = False
        # honor --no-ankle-cop / --stance-freeze for ablations; default CoP ON
        if args.tag in ("ankle_cop", "ankle_cop_stand"):
            args.tag = "ankle_cop_stand" if use_ankle_cop else "ankle_cop_stand_off"
        if args.duration < 4.5:
            args.duration = 4.5
        # default disturb: near-threshold lateral velocity that tips without CoP
        # (vy≈0.7 tips OFF; use 0.62 as the prove-recovery target)

    def gait_amp(t: float) -> float:
        if not gait_on:
            return 0.0
        if t < stand_hold:
            return 0.0
        u = t - stand_hold
        if u >= ramp_t:
            return 1.0
        s = u / ramp_t
        return s * s * (3.0 - 2.0 * s)

    def _qadr(name: str) -> int:
        return int(model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)])

    hip_l = _qadr("l_hip_pitch")
    hip_r = _qadr("r_hip_pitch")
    knee_l = _qadr("l_knee")
    knee_r = _qadr("r_knee")
    ank_l = _qadr("l_ank_pitch")
    ank_r = _qadr("r_ank_pitch")
    bid_body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    bid_lf = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "l_ank_roll_link")
    bid_rf = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "r_ank_roll_link")
    bid_l_thigh = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "l_hip_pitch_link")
    bid_r_thigh = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "r_hip_pitch_link")
    bid_l_shin = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "l_knee_link")
    bid_r_shin = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "r_knee_link")
    aid_l_hip = act_idx["l_hip_pitch_pos"]
    aid_r_hip = act_idx["r_hip_pitch_pos"]
    gid_lfoot = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "l_foot_contact")
    gid_rfoot = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "r_foot_contact")
    gid_floor = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "floor")

    print(f"[ainex] model={xml} nq={model.nq} nu={model.nu} nmesh={model.nmesh} dt={sim_dt}")
    print(f"[ainex] mass={mass:.4f} kg  (kit class ~2.45; URDF sum ~2.35)")
    print(f"[ainex] Controls: {CTRL_HZ} Hz T={GAIT_T}s hip_amp={HIP_PITCH_AMP} "
          f"COM_shift={COM_SHIFT_AMP} DS={DS_S}s step={STEP_LEN}")
    print(f"[ainex] COM_Z={COM_Z} assist={use_assist} plant={use_plant} "
          f"ankle_cop={use_ankle_cop} freeze={use_stance_freeze} disturb={disturb_stand}")
    print(f"[ainex] phaseB/C: cp_swing={USE_CP_SWING} hip_strat={USE_HIP_STRAT} "
          f"capture_step={USE_CAPTURE_STEP} plant_kd={PLANT_KD}")
    print(f"[ainex] phaseD: stance_vik={USE_STANCE_VIK} kp={STANCE_VIK_KP} "
          f"clip={STANCE_VIK_CLIP} zmp_qp={USE_ZMP_QP} wbc={int(USE_WBC_STANCE)} mu={WBC_MU} mpc={int(USE_HYBRID_MPC)} "
          f"foot_pos_ik={USE_FOOT_POS_IK} hold={USE_STANCE_HOLD} "
          f"residual={int(USE_RESIDUAL_STANCE_VX)} gain={RESIDUAL_GAIN}")
    print(f"[ainex] v_des={v_des_x} tag={args.tag} "
          f"ANK_COP lean=({ANK_COP_KP_LEAN_X},{ANK_COP_KP_LEAN_Y}) cop=({ANK_COP_KP_X},{ANK_COP_KP_Y}) clip={ANK_COP_CLIP}")
    print(f"[ainex] legs L=ORANGE R=GREEN | mesh=Hiwonder STL (NOT OEM STEP) | Path A FROZEN")
    print(f"[ainex] cam WORLD_FIXED lookat={CAM_FIXED_LOOKAT.tolist()} d={CAM_FIXED_DISTANCE}")

    renderer = None
    cam = mj.MjvCamera()
    mj.mjv_defaultCamera(cam)
    cam.azimuth = CAM_FIXED_AZIMUTH
    cam.elevation = CAM_FIXED_ELEVATION
    cam.distance = CAM_FIXED_DISTANCE
    cam.lookat[:] = CAM_FIXED_LOOKAT

    if not args.no_video:
        renderer = mj.Renderer(model, height=480, width=640)

    n_ctrl = int(args.duration * CTRL_HZ)
    exploded = False
    flipped = False
    x0 = float(data.qpos[0])
    csv_rows: list[list[float]] = []
    raw_frames: list[np.ndarray] = []
    frame_meta: list[dict] = []
    foot_lead_L: list[bool] = []
    upright_flags: list[bool] = []
    contact_L: list[bool] = []
    contact_R: list[bool] = []
    stance_vx_L: list[float] = []
    stance_vx_R: list[float] = []
    best_frame_idx = 0
    best_frame_score = -1e9
    freeze_hold: dict[str, np.ndarray | None] = {"L": None, "R": None}
    disturb_t = 1.00          # after CoP bias settle (0.5s); >1.1s drifts kill freeze RoA
    disturb_applied = False
    disturb_force_until = -1.0
    disturb_recovered = False
    disturb_recover_t = float("nan")
    max_lean_post = 0.0
    cop_err_x: list[float] = []
    cop_err_y: list[float] = []
    cop_bias_samples: list[np.ndarray] = []
    cop_bias_xy: np.ndarray | None = None
    recover_hold_s = 0.0
    tip_free_run = 0.0
    tip_free_max = 0.0
    tip_free_post_gait_max = 0.0
    gait_started = False
    prev_com_vxy: list = [None]  # mutable box for loop
    tip_latch: dict = {"active": False, "hold": 0.0}

    def _yaw_deg() -> float:
        w, x, y, z = [float(v) for v in data.qpos[3:7]]
        return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

    def _render_raw() -> np.ndarray:
        cam.lookat[:] = CAM_FIXED_LOOKAT
        cam.azimuth = CAM_FIXED_AZIMUTH
        cam.elevation = CAM_FIXED_ELEVATION
        cam.distance = CAM_FIXED_DISTANCE
        mj.mj_forward(model, data)
        renderer.update_scene(data, cam)
        return np.ascontiguousarray(renderer.render().copy(), dtype=np.uint8)

    for k in range(n_ctrl):
        t = data.time
        a = gait_amp(t)
        # In disturb-stand mode keep full stand targets; ankle CoP amp=1 always after t>0.2
        if disturb_stand:
            a_servo = 1.0 if t > 0.15 else 0.0
            a_gait = 0.0
        else:
            a_servo = a
            a_gait = a
        qdes = gait_targets(t, gait_on and not disturb_stand, a_gait)
        set_ctrl(model, data, qdes, act_idx)
        phi_now = (max(0.0, t - stand_hold) / GAIT_T) % 1.0 if a_gait > 0 else 0.0
        lat_now = lateral_com_target(phi_now) if a_gait > 0 else 0.0

        # legacy lean servos only when gait is on and ankle CoP is off (avoid double-count)
        if a_gait > 0 and not use_ankle_cop:
            roll_servo(model, data, act_idx, a_gait)
            pitch_servo(model, data, act_idx, a_gait)

        # Capture COM-support bias during quiet settle (before disturb / gait)
        if use_ankle_cop and cop_bias_xy is None and t < ANK_COP_BIAS_SETTLE_S:
            com = np.asarray(data.subtree_com[0, :2], dtype=np.float64)
            support, _, _, _ = _support_feet(model, data, bid_lf, bid_rf, gid_floor, None)
            cop_bias_samples.append(com - support)
            if t >= ANK_COP_BIAS_SETTLE_S - 1.5 / CTRL_HZ and cop_bias_samples:
                cop_bias_xy = np.mean(np.stack(cop_bias_samples, axis=0), axis=0)
                print(f"[ainex] CoP settle bias xy=({cop_bias_xy[0]:+.4f},{cop_bias_xy[1]:+.4f}) m")

        cop_diag = {"err_x": 0.0, "err_y": 0.0}
        if use_ankle_cop and a_servo > 0:
            lat_arg = lat_now if a_gait > 0.05 else None
            # during bias capture, still run lean terms but zero CoP via bias=err
            bias = cop_bias_xy
            if bias is None and cop_bias_samples:
                bias = np.mean(np.stack(cop_bias_samples, axis=0), axis=0)
            cop_diag = ankle_cop_servo(
                model, data, act_idx, a_servo,
                bid_lf, bid_rf, gid_floor, lat=lat_arg, bias_xy=bias,
            )
            cop_err_x.append(float(cop_diag.get("err_x", 0.0)))
            cop_err_y.append(float(cop_diag.get("err_y", 0.0)))

        # Phase B: CP/DCM swing placement (joint-space; no base force cheat)
        if USE_CP_SWING and a_gait > 0.05:
            apply_cp_swing_placement(
                model, data, act_idx, phi_now, a_gait, bid_lf, bid_rf,
            )
        # Stance foot vx null via hip/ankle (skate killer; joint-space only)
        if USE_STANCE_V_NULL and a_gait > 0.05:
            apply_stance_vx_null(
                model, data, act_idx, phi_now, a_gait, bid_lf, bid_rf,
            )
        # Phase D: stance joint hold + Jacobian velocity IK (no root xfrc)
        if USE_STANCE_HOLD and a_gait > 0.05:
            apply_stance_joint_hold(
                model, data, act_idx, phi_now, a_gait,
                bid_lf, bid_rf, gid_floor,
            )
        if USE_FOOT_POS_IK and a_gait > 0.05:
            apply_foot_pos_ik(
                model, data, act_idx, phi_now, a_gait,
                bid_lf, bid_rf, gid_floor,
            )
        if USE_STANCE_VIK and a_gait > 0.05:
            apply_stance_jacobian_vik(
                model, data, act_idx, phi_now, a_gait,
                bid_lf, bid_rf, gid_floor,
            )
        if USE_ZMP_QP and a_gait > 0.05:
            apply_zmp_qp_trim(
                model, data, act_idx, phi_now, a_gait, bid_lf, bid_rf,
            )
        # Stance friction-cone wrench QP (WBC-lite) — not residual K/MLP
        if USE_WBC_STANCE and a_gait > 0.05:
            apply_wbc_stance_qp(
                model, data, act_idx, phi_now, a_gait,
                bid_lf, bid_rf, gid_floor,
                mu=WBC_MU,
                mass=mass,
                lat_fn=lateral_com_target,
                omega=CP_OMEGA if CP_OMEGA > 0.1 else 3.2,
                scrub_k=WBC_SCRUB_K,
                trim_clip=WBC_TRIM_CLIP,
                tau_scale=WBC_TAU_SCALE,
                act_name_fn=act_name,
                w_scrub=WBC_W_SCRUB,
            )
        # Hybrid MPC + T88 hard-reject (post residual/WBC falsifiers)
        if USE_HYBRID_MPC and a_gait > 0.05:
            apply_hybrid_mpc_t88(
                model, data, act_idx, phi_now, a_gait,
                bid_lf, bid_rf, gid_floor,
                gait_t=GAIT_T,
                n_horizon=MPC_N,
                u_max=MPC_U_MAX,
                ank_ratio=MPC_ANK_RATIO,
                lat_fn=lateral_com_target,
                act_name_fn=act_name,
                apply_thr=MPC_APPLY_THR,
                disable_above_t=MPC_DISABLE_ABOVE_T,
            )
        # Residual stance-vx (additive AFTER gait+CP+VIK; HX clip inside)
        if USE_RESIDUAL_STANCE_VX and a_gait > 0.05:
            apply_residual_stance_vx(
                model, data, act_idx, phi_now, a_gait, bid_lf, bid_rf,
                RESIDUAL_FIT,
                gain=RESIDUAL_GAIN,
                lateral_com_target=lateral_com_target,
                contact_z_thr=CONTACT_Z_THR,
                act_name_fn=act_name,
                gid_floor=gid_floor,
            )
        # Phase C: hip strategy + reactive capture step
        if USE_HIP_STRAT and a_gait > 0.05:
            prev_com_vxy[0] = apply_hip_strategy(
                model, data, act_idx, a_gait, prev_com_vxy[0], 1.0 / CTRL_HZ,
            )
        elif a_gait > 0.05:
            _, vxy, _ = estimate_com_state(model, data)
            prev_com_vxy[0] = vxy
        if USE_CAPTURE_STEP and a_gait > 0.05:
            apply_capture_step(model, data, act_idx, a_gait, tip_latch)

        # Disturbance: velocity nudge and/or force pulse, assist OFF
        if disturb_stand and (not disturb_applied) and t >= disturb_t:
            data.qvel[0] += float(args.push_vx)
            data.qvel[1] += float(args.push_vy)
            disturb_applied = True
            disturb_force_until = disturb_t + float(args.push_hold)
            # lock current foot xy immediately (sticky freeze plant)
            freeze_hold["L"] = data.xpos[bid_lf, :2].copy()
            freeze_hold["R"] = data.xpos[bid_rf, :2].copy()
            print(
                f"[ainex] DISTURB at t={t:.3f}s  dv=({args.push_vx:+.3f},{args.push_vy:+.3f}) m/s  "
                f"F=({args.push_fx:+.2f},{args.push_fy:+.2f}) N hold={args.push_hold:.2f}s  "
                f"freeze={int(use_stance_freeze)}"
            )

        for _ in range(steps_per_ctrl):
            data.qfrc_applied[:] = 0
            data.xfrc_applied[:] = 0
            if use_assist:
                balance_assist(model, data, COM_Z, fx_bias=fx_bias * a_gait,
                               v_des_x=v_des_x, speed_gate=a_gait)
            if use_plant and a_gait > 0:
                stance_plant(
                    model, data, phi_now, a_gait,
                    bid_lf, bid_rf, gid_lfoot, gid_rfoot, gid_floor,
                )
            # Stance freeze: during walk (gait) OR during disturb-stand after push
            if use_stance_freeze and (a_gait > 0 or (disturb_stand and disturb_applied)):
                stance_foot_freeze(
                    model, data, freeze_hold,
                    1.0 if disturb_stand else a_gait,
                    bid_lf, bid_rf, gid_floor,
                    0.0 if disturb_stand else lat_now,
                    sticky=bool(disturb_stand),
                )
            if disturb_stand and disturb_applied and data.time < disturb_force_until:
                data.qfrc_applied[0] += float(args.push_fx)
                data.qfrc_applied[1] += float(args.push_fy)
            mj.mj_step(model, data)
            # HX authority saturation (legs): |τ| near forcerange
            if AUTH_LEG_ACT_IDS:
                AUTH_SAT_STEPS += 1
                for ai in AUTH_LEG_ACT_IDS:
                    lim = abs(float(model.actuator_forcerange[ai, 1]))
                    if lim > 1e-9 and abs(float(data.actuator_force[ai])) >= 0.98 * lim:
                        AUTH_SAT_COUNT += 1
            data.qfrc_applied[:] = 0
            data.xfrc_applied[:] = 0
            if (not np.isfinite(data.qpos).all()) or data.qpos[2] < 0.08 or data.qpos[2] > 0.55:
                exploded = True
                break
        if exploded:
            print(f"[ainex] STOP early t={data.time:.3f}s z={data.qpos[2]}")
            break

        # Disturb recovery tracking (must hold upright 0.4 s continuously)
        if disturb_stand and disturb_applied:
            Rtmp = data.xmat[bid_body].reshape(3, 3)
            lean = float(np.sqrt(Rtmp[0, 2] ** 2 + Rtmp[1, 2] ** 2))
            max_lean_post = max(max_lean_post, lean)
            upz_now = float(Rtmp[2, 2])
            ok_now = upz_now > 0.90 and abs(float(data.qpos[1])) < 0.05 and lean < 0.18 and float(data.qpos[2]) > 0.20
            if ok_now:
                recover_hold_s += 1.0 / CTRL_HZ
            else:
                recover_hold_s = 0.0
            if (not disturb_recovered) and recover_hold_s >= 0.40:
                disturb_recovered = True
                disturb_recover_t = float(data.time) - disturb_t

        up_z = float(data.xmat[bid_body].reshape(3, 3)[2, 2])
        if up_z < 0.3:
            flipped = True

        ctrl_l = float(data.ctrl[aid_l_hip])
        ctrl_r = float(data.ctrl[aid_r_hip])
        q_l = float(data.qpos[hip_l])
        q_r = float(data.qpos[hip_r])
        q_kl = float(data.qpos[knee_l])
        q_kr = float(data.qpos[knee_r])
        q_al = float(data.qpos[ank_l])
        q_ar = float(data.qpos[ank_r])
        z_l = float(data.xpos[bid_lf, 2])
        z_r = float(data.xpos[bid_rf, 2])
        x_l = float(data.xpos[bid_lf, 0])
        x_r = float(data.xpos[bid_rf, 0])
        pz = float(data.qpos[2])
        px = float(data.qpos[0])
        py = float(data.qpos[1])
        p_th_l = body_sagittal_pitch_x(data, bid_l_thigh)
        p_th_r = body_sagittal_pitch_x(data, bid_r_thigh)
        p_sh_l = body_sagittal_pitch_x(data, bid_l_shin)
        p_sh_r = body_sagittal_pitch_x(data, bid_r_shin)
        foot_lead_L.append(x_l > x_r)

        cL = foot_floor_contact(model, data, bid_lf, gid_floor)
        cR = foot_floor_contact(model, data, bid_rf, gid_floor)
        contact_L.append(cL)
        contact_R.append(cR)
        upright_now = up_z >= ACC["min_up_z"] and pz >= ACC["min_body_z"]
        upright_flags.append(upright_now)
        if a_gait > 0.15:
            gait_started = True
        if upright_now and not flipped and not exploded:
            tip_free_run += 1.0 / CTRL_HZ
            tip_free_max = max(tip_free_max, tip_free_run)
            if gait_started:
                tip_free_post_gait_max = max(tip_free_post_gait_max, tip_free_run)
        else:
            tip_free_run = 0.0

        # stance foot horizontal speed (use body cvel)
        vx_l = float(data.cvel[bid_lf][3])
        vx_r = float(data.cvel[bid_rf][3])
        lat = lat_now
        # record stance vx only when that side is weight-bearing + in contact
        if a_gait > 0.2 and lat > 0.0 and cL:
            stance_vx_L.append(abs(vx_l))
        if a_gait > 0.2 and lat < 0.0 and cR:
            stance_vx_R.append(abs(vx_r))

        csv_rows.append([
            float(data.time), ctrl_l, ctrl_r, q_l, q_r, z_l, z_r, pz, x_l, x_r, px,
            q_kl, q_kr, q_al, q_ar, p_th_l, p_th_r, p_sh_l, p_sh_r, py, up_z,
            float(cL), float(cR), vx_l, vx_r, lat, a,
        ])

        if renderer is not None and (k % 2 == 0):
            raw = _render_raw()
            raw_frames.append(raw)
            meta = {
                "t": float(data.time), "k": k, "z": pz, "x": px, "y": py,
                "yaw": _yaw_deg(), "q_l": q_l, "q_r": q_r,
                "z_l": z_l, "z_r": z_r, "shin_l": p_sh_l, "shin_r": p_sh_r,
                "up_z": up_z, "cL": cL, "cR": cR,
            }
            frame_meta.append(meta)
            # score for best still: upright + mid-corridor + both feet near ground or alt
            score = (
                10.0 * up_z
                + 5.0 * (1.0 if 0.05 < px < 0.55 else 0.0)
                + 2.0 * (1.0 if abs(py) < 0.08 else 0.0)
                + 1.0 * (1.0 if a_gait > 0.5 else 0.0)
                - 3.0 * abs(pz - COM_Z)
            )
            gate = True if disturb_stand else (a_gait > 0.3)
            if disturb_stand and disturb_applied:
                score += 4.0 * up_z + 2.0 * (1.0 if abs(py) < 0.04 else 0.0)
            if score > best_frame_score and gate:
                best_frame_score = score
                best_frame_idx = len(raw_frames) - 1

    x_final = float(data.qpos[0])
    dx = x_final - x0
    t_final = float(data.time)
    walk_t = max(1e-6, t_final - stand_hold - ramp_t)
    avg_speed = dx / walk_t if gait_on else 0.0

    arr = np.asarray(csv_rows, dtype=np.float64) if csv_rows else np.zeros((0, 27))
    i0 = int((stand_hold + ramp_t + 0.2) * CTRL_HZ)
    ss = arr[i0:] if len(arr) > i0 else arr

    hip_q_ptp = knee_q_ptp = ank_q_ptp = 0.0
    hip_q_corr = knee_q_corr = shin_corr = 0.0
    shin_ptp_l = shin_ptp_r = 0.0
    shin_cycles = foot_lead_cycles = 0
    if len(ss):
        hip_q_ptp = float(max(np.ptp(ss[:, 3]), np.ptp(ss[:, 4])))
        knee_q_ptp = float(max(np.ptp(ss[:, 11]), np.ptp(ss[:, 12])))
        ank_q_ptp = float(max(np.ptp(ss[:, 13]), np.ptp(ss[:, 14])))
        if len(ss) > 5:
            fwd_l = -ss[:, 3]
            fwd_r = +ss[:, 4]
            flex_l = +ss[:, 11]
            flex_r = -ss[:, 12]
            hip_q_corr = float(np.corrcoef(fwd_l, fwd_r)[0, 1])
            knee_q_corr = float(np.corrcoef(flex_l, flex_r)[0, 1])
            shin_corr = float(np.corrcoef(ss[:, 17], ss[:, 18])[0, 1])
        shin_ptp_l = float(np.ptp(ss[:, 17]))
        shin_ptp_r = float(np.ptp(ss[:, 18]))
        shin_cycles = _zero_cross_cycles(ss[:, 17] - ss[:, 18], 0.0)
        lead = np.asarray(foot_lead_L[i0:] if len(foot_lead_L) > i0 else foot_lead_L, dtype=bool)
        if len(lead) > 2:
            foot_lead_cycles = int(np.sum(lead[1:] != lead[:-1])) // 2

    n_l, lift_l = _count_lift_peaks(ss[:, 5]) if len(ss) else (0, 0.0)
    n_r, lift_r = _count_lift_peaks(ss[:, 6]) if len(ss) else (0, 0.0)

    # upright + contact duty over steady-state window
    uf = upright_flags[i0:] if len(upright_flags) > i0 else upright_flags
    cLf = contact_L[i0:] if len(contact_L) > i0 else contact_L
    cRf = contact_R[i0:] if len(contact_R) > i0 else contact_R
    upright_frac = float(np.mean(uf)) if uf else 0.0
    contact_duty_L = float(np.mean(cLf)) if cLf else 0.0
    contact_duty_R = float(np.mean(cRf)) if cRf else 0.0
    gait_s = max(0.0, t_final - stand_hold - ramp_t)

    def _vx_stats(xs: list[float]) -> tuple[float, float]:
        if not xs:
            return float("nan"), float("nan")
        a_ = np.asarray(xs, dtype=np.float64)
        return float(np.mean(a_)), float(np.percentile(a_, 95))

    stx_L_mean, stx_L_p95 = _vx_stats(stance_vx_L)
    stx_R_mean, stx_R_p95 = _vx_stats(stance_vx_R)

    skate = False
    skate_reasons: list[str] = []
    if foot_lead_cycles < 2 and abs(dx) > 0.05:
        skate = True
        skate_reasons.append("body translates without foot-lead alternation")
    for label, mean_v, p95 in (
        ("L", stx_L_mean, stx_L_p95),
        ("R", stx_R_mean, stx_R_p95),
    ):
        if mean_v == mean_v and mean_v > ACC["max_stance_vx_mean"]:
            skate = True
            skate_reasons.append(f"stance {label} mean |vx|={mean_v:.3f}>{ACC['max_stance_vx_mean']}")
        if p95 == p95 and p95 > ACC["max_stance_vx_p95"]:
            skate = True
            skate_reasons.append(f"stance {label} p95 |vx|={p95:.3f}>{ACC['max_stance_vx_p95']}")

    rel_l_amp = rel_r_amp = 0.0
    if len(ss):
        rel_l_amp = float(np.ptp(ss[:, 8] - ss[:, 10]))
        rel_r_amp = float(np.ptp(ss[:, 9] - ss[:, 10]))

    pz_ss = ss[:, 7] if len(ss) else np.array([])
    pz_min = float(pz_ss.min()) if len(pz_ss) else float("nan")
    pz_max = float(pz_ss.max()) if len(pz_ss) else float("nan")
    pz_mean = float(pz_ss.mean()) if len(pz_ss) else float("nan")

    leg_clip = (-2.1, 2.1)
    arm_clip = (-0.7, 0.7)

    print(
        f"[ainex] done t={t_final:.3f}s dx={dx:+.4f}m avg_speed={avg_speed*100:.1f} cm/s "
        f"exploded={exploded} flipped={flipped}"
    )
    print(
        f"[ainex] body_z ss: min={pz_min:.4f} max={pz_max:.4f} mean={pz_mean:.4f} "
        f"upright_frac={upright_frac:.3f}"
    )
    print(
        f"[ainex] joints: hip_ptp={hip_q_ptp:.3f} knee_ptp={knee_q_ptp:.3f} ank_ptp={ank_q_ptp:.3f} "
        f"hip_corr={hip_q_corr:.3f} knee_corr={knee_q_corr:.3f}"
    )
    print(
        f"[ainex] mesh: shin_ptp L/R={shin_ptp_l:.3f}/{shin_ptp_r:.3f} shin_corr={shin_corr:.3f} "
        f"shin_cycles={shin_cycles} foot_lead_cycles={foot_lead_cycles}"
    )
    print(
        f"[ainex] feet: lift L={lift_l:.4f}m({n_l}pk) R={lift_r:.4f}m({n_r}pk) "
        f"contact_duty L/R={contact_duty_L:.2f}/{contact_duty_R:.2f} "
        f"stance_vx_mean L/R={stx_L_mean:.3f}/{stx_R_mean:.3f} skate={skate}"
    )
    print(
        f"[ainex] tip_free_max={tip_free_max:.2f}s tip_free_post_gait_max={tip_free_post_gait_max:.2f}s "
        f"dx={dx:+.4f}m"
    )

    csv_path = out_dir / "ainex_walk_timeseries.csv"
    buf = io.StringIO()
    buf.write(
        "# ainex_walk mesh=Hiwonder_STL NOT_OEM_STEP "
        f"T={GAIT_T} hip_amp={HIP_PITCH_AMP} COM_shift={COM_SHIFT_AMP} "
        f"assist={int(use_assist)} plant={int(use_plant)} cam=WORLD_FIXED_d{CAM_FIXED_DISTANCE}\n"
    )
    w = csv.writer(buf)
    w.writerow([
        "t", "ctrl_l_hip", "ctrl_r_hip", "q_l_hip", "q_r_hip",
        "z_l_foot", "z_r_foot", "body_z", "x_l_foot", "x_r_foot", "body_x",
        "q_l_knee", "q_r_knee", "q_l_ank", "q_r_ank",
        "thigh_pitch_l", "thigh_pitch_r", "shin_pitch_l", "shin_pitch_r",
        "body_y", "up_z", "contact_L", "contact_R", "vx_L", "vx_R", "lat", "amp",
    ])
    for row in csv_rows:
        w.writerow([f"{v:.6f}" for v in row])
    csv_bytes = buf.getvalue().encode("utf-8")
    csv_path.write_bytes(csv_bytes)
    # Mirror timeseries next to --stats-out so ss_step scoring is not stale
    if args.stats_out:
        stats_csv = Path(args.stats_out).with_name(
            Path(args.stats_out).stem + "_timeseries.csv"
        )
        if stats_csv.resolve() != csv_path.resolve():
            stats_csv.write_bytes(csv_bytes)
            print(f"[ainex] stats timeseries → {stats_csv}")
        # also refresh iterate/ainex_walk_timeseries.csv if stats live under iterate/
        shared = Path(args.stats_out).parent / "ainex_walk_timeseries.csv"
        if shared.resolve() != csv_path.resolve():
            shared.write_bytes(csv_bytes)
    run_id = _sha256_first_1kb(csv_bytes)
    run_id_short = run_id[:12]
    print(f"[ainex] wrote {csv_path} run_id={run_id_short}")

    frame_paths: list[Path] = []
    best_png_bytes = None
    if renderer is not None and raw_frames:
        import imageio.v2 as imageio

        for fi, (raw, meta) in enumerate(zip(raw_frames, frame_meta)):
            lines = [
                f"run_id={run_id_short}  AiNex CoP+plant ({args.tag})",
                f"t={meta['t']:.2f}s  z={meta['z']:.3f}  x={meta['x']:.3f}  y={meta['y']:.3f}  up={meta['up_z']:.2f}",
                f"hipL={meta['q_l']:+.3f} hipR={meta['q_r']:+.3f}  cL={int(meta['cL'])} cR={int(meta['cR'])}",
                f"assist={int(use_assist)} plant={int(use_plant)} ankCoP={int(use_ankle_cop)} "
                f"freeze={int(use_stance_freeze)} WORLD_FIXED",
            ]
            img = _burn_overlay(raw, lines)
            path = frame_seq / f"frame_{fi:05d}.png"
            imageio.imwrite(path, img)
            frame_paths.append(path)
            if fi == best_frame_idx:
                best_png_bytes = img
        print(f"[ainex] wrote {len(frame_paths)} PNGs → {frame_seq}")
        out = Path(args.out)
        _encode_mp4_ffmpeg(frame_seq, out, fps=25)
        print(f"[ainex] wrote {out} all-intra ({len(frame_paths)} frames @25fps)")
        strip = out_dir / "debug_frames"
        strip.mkdir(exist_ok=True)
        for old in strip.glob("*.png"):
            old.unlink()
        idxs = np.linspace(0, len(frame_paths) - 1, min(12, len(frame_paths)), dtype=int)
        for si, fi in enumerate(idxs):
            shutil.copy2(frame_paths[fi], strip / f"strip_{si:02d}_t{frame_meta[fi]['t']:05.2f}.png")

        # Save best still to kit folder
        if best_png_bytes is not None:
            import imageio.v2 as imageio
            if disturb_stand and use_stance_freeze:
                attempt_path = HIWONDER_DIR / "ankle_cop_freeze.png"
            elif disturb_stand:
                attempt_path = HIWONDER_DIR / "ankle_cop_stand.png"
            elif use_ankle_cop:
                attempt_path = HIWONDER_DIR / "ankle_cop_walk.png"
            else:
                attempt_path = HIWONDER_DIR / "walk_attempt.png"
            imageio.imwrite(attempt_path, best_png_bytes)
            print(f"[ainex] best still → {attempt_path} (t={frame_meta[best_frame_idx]['t']:.2f}s)")
            # also keep a mid/late frame for disturb recovery proof
            if disturb_stand and frame_meta:
                # pre-push and post-push frames
                pre_i = max(0, min(len(frame_meta) - 1,
                                   int((disturb_t - 0.15) / (2.0 / CTRL_HZ))))
                # frames are every 2 ctrl steps → dt_frame = 2/CTRL_HZ
                dt_f = 2.0 / CTRL_HZ
                pre_i = int(max(0, (disturb_t - 0.2) / dt_f))
                post_i = int(min(len(raw_frames) - 1, (disturb_t + 1.2) / dt_f))
                prefix = "ankle_cop_freeze" if use_stance_freeze else "ankle_cop_stand"
                for label, idx in ((f"{prefix}_pre.png", pre_i),
                                   (f"{prefix}_post.png", post_i)):
                    idx = int(np.clip(idx, 0, len(raw_frames) - 1))
                    lines = [
                        f"run_id={run_id_short}  disturb-stand ({label})",
                        f"t={frame_meta[idx]['t']:.2f}s up={frame_meta[idx]['up_z']:.2f} "
                        f"y={frame_meta[idx]['y']:.3f}",
                        f"push_dv=({args.push_vx:+.2f},{args.push_vy:+.2f}) ankCoP=1 assist=0",
                    ]
                    img = _burn_overlay(raw_frames[idx], lines)
                    imageio.imwrite(HIWONDER_DIR / label, img)

    if renderer is not None:
        renderer.close()

    # --- WORLD_FIXED acceptance ---
    fails: list[str] = []
    if exploded and t_final < 6.0:
        fails.append(f"explode/fall before 6s (t={t_final:.2f})")
    if flipped:
        fails.append("flipped (body up_z < 0.3)")
    if gait_s < ACC["min_gait_s"]:
        fails.append(f"gait window {gait_s:.2f}s < {ACC['min_gait_s']}")
    if upright_frac < ACC["upright_frac"]:
        fails.append(f"upright_frac={upright_frac:.3f} < {ACC['upright_frac']}")
    if not (ACC["contact_duty_lo"] <= contact_duty_L <= ACC["contact_duty_hi"]):
        fails.append(f"contact_duty_L={contact_duty_L:.2f} outside [{ACC['contact_duty_lo']},{ACC['contact_duty_hi']}]")
    if not (ACC["contact_duty_lo"] <= contact_duty_R <= ACC["contact_duty_hi"]):
        fails.append(f"contact_duty_R={contact_duty_R:.2f} outside [{ACC['contact_duty_lo']},{ACC['contact_duty_hi']}]")
    if hip_q_ptp < ACC["min_hip_ptp"] and gait_on:
        fails.append(f"hip q ptp {hip_q_ptp:.3f}<{ACC['min_hip_ptp']}")
    if knee_q_ptp < ACC["min_knee_ptp"] and gait_on:
        fails.append(f"knee q ptp {knee_q_ptp:.3f}<{ACC['min_knee_ptp']}")
    if hip_q_corr > -0.45 and gait_on:
        fails.append(f"hips not opposite-phase corr={hip_q_corr:.3f}")
    if foot_lead_cycles < ACC["min_foot_lead_cycles"] and gait_on:
        fails.append(f"foot lead cycles={foot_lead_cycles}<{ACC['min_foot_lead_cycles']}")
    if dx < ACC["min_dx_m"] and gait_on:
        fails.append(f"dx={dx:.3f}m < {ACC['min_dx_m']} (no forward progress)")
    if abs(avg_speed) > ACC["max_avg_speed"] and gait_on:
        fails.append(f"avg speed {avg_speed*100:.1f} cm/s > {ACC['max_avg_speed']*100:.0f}")
    if skate:
        fails.append("skate: " + "; ".join(skate_reasons))
    if use_assist and ACC["require_assist_off"]:
        fails.append("balance_assist ON (upright lock + height + speed governor) — not clean walk")
    if lift_l < 0.006 and lift_r < 0.006 and gait_on:
        fails.append("no measurable foot lift — statue-slide risk")
    if gait_on and tip_free_post_gait_max < 5.0:
        fails.append(f"tip-free post-gait {tip_free_post_gait_max:.2f}s < 5.0")

    mesh_ok = (
        gait_on
        and not exploded
        and not flipped
        and hip_q_ptp >= ACC["min_hip_ptp"]
        and knee_q_ptp >= ACC["min_knee_ptp"]
        and hip_q_corr <= -0.45
        and foot_lead_cycles >= ACC["min_foot_lead_cycles"]
        and t_final >= 5.0
    )
    clean_walk = (
        mesh_ok and not fails and not use_assist and not skate
        and dx >= ACC["min_dx_m"]
        and tip_free_post_gait_max >= 5.0
        and not use_stance_freeze
    )

    # Disturb-stand has its own acceptance (not walk gates)
    disturb_pass = False
    if disturb_stand:
        post_uf = 0.0
        if upright_flags:
            i_push = int(disturb_t * CTRL_HZ)
            post = upright_flags[i_push:] if len(upright_flags) > i_push else upright_flags
            post_uf = float(np.mean(post)) if post else 0.0
        final_up = float(data.xmat[bid_body].reshape(3, 3)[2, 2])
        disturb_pass = (
            disturb_applied
            and not exploded
            and not flipped
            and post_uf >= 0.85
            and float(data.qpos[2]) >= 0.18
            and final_up >= 0.85
            and (disturb_recovered or post_uf >= 0.95)
        )
        fails = []
        if not disturb_applied:
            fails.append("disturbance never applied")
        if exploded:
            fails.append(f"fell/exploded t={t_final:.2f}")
        if flipped:
            fails.append("flipped")
        if post_uf < 0.85:
            fails.append(f"post-push upright_frac={post_uf:.3f}<0.85")
        if final_up < 0.85:
            fails.append(f"final up_z={final_up:.3f}<0.85")
        if not disturb_recovered and post_uf < 0.95:
            fails.append("no clear recover (up_z>0.92 & |y|<0.035)")
        if disturb_pass:
            mode = (
                "stance-freeze ablation"
                if use_stance_freeze and not use_ankle_cop else
                "ankle CoP + stance-freeze"
                if use_stance_freeze and use_ankle_cop else
                "ankle CoP only"
                if use_ankle_cop else
                "open-loop stand (no CoP/freeze)"
            )
            verdict = (
                f"PASS — stand disturbance recovery via {mode} "
                f"(recover_t={disturb_recover_t:.2f}s, post_uf={post_uf:.3f}, "
                f"max_lean={max_lean_post:.3f}, assist OFF, HX clips on)"
            )
        else:
            mode = (
                "stance-freeze" if use_stance_freeze else
                "ankle CoP only" if use_ankle_cop else "no CoP/freeze"
            )
            verdict = f"FAIL — stand disturbance recovery ({mode}): " + "; ".join(fails)
        clean_walk = False
        mesh_ok = False
    elif clean_walk:
        verdict = (
            "PASS — WORLD_FIXED gates met (upright/contact/dx/no-skate) with balance assist OFF. "
            "Still open-loop CPG + ankle CoP / plant; not full ZMP/capture-point/RL."
        )
    elif mesh_ok and use_assist and not flipped and not exploded:
        verdict = (
            "FAIL clean walk (assist ON) — mesh articulates but balance assist supplies upright/speed. "
            + ("Fails: " + "; ".join(fails) if fails else "")
        )
    elif mesh_ok and fails:
        verdict = "FAIL — articulation present but WORLD_FIXED gates failed: " + "; ".join(fails)
    else:
        verdict = "FAIL — " + ("; ".join(fails) if fails else "no credible walk")

    still_png = (
        HIWONDER_DIR / "ankle_cop_freeze.png" if (disturb_stand and use_stance_freeze)
        else HIWONDER_DIR / "ankle_cop_stand.png" if disturb_stand
        else (HIWONDER_DIR / "ankle_cop_walk.png" if use_ankle_cop
              else HIWONDER_DIR / "walk_attempt.png")
    )

    stats = {
        "run_id": run_id,
        "run_id_short": run_id_short,
        "tag": args.tag,
        "model": str(xml),
        "source_geometry": "Hiwonder URDF + 25 STL (NOT OEM STEP)",
        "mass_kg": mass,
        "nu": int(model.nu),
        "nmesh": int(model.nmesh),
        "ctrl_hz": CTRL_HZ,
        "gait_T": GAIT_T,
        "step_len_cmd": STEP_LEN,
        "hip_pitch_amp": HIP_PITCH_AMP,
        "hip_bias_fwd": HIP_BIAS_FWD,
        "ds_s": DS_S,
        "knee_stance": KNEE_STANCE,
        "knee_swing": KNEE_SWING,
        "com_z": COM_Z,
        "com_shift_amp": COM_SHIFT_AMP,
        "com_shift_lead": COM_SHIFT_LEAD,
        "balance_assist": use_assist,
        "stance_plant": use_plant,
        "ankle_cop": use_ankle_cop,
        "stance_freeze": use_stance_freeze,
        "residual_stance_vx": bool(USE_RESIDUAL_STANCE_VX),
        "residual_gain": float(RESIDUAL_GAIN) if USE_RESIDUAL_STANCE_VX else 0.0,
        "residual_path": str(RESIDUAL_PATH),
        "cp_swing": USE_CP_SWING,
        "hip_strat": USE_HIP_STRAT,
        "capture_step": USE_CAPTURE_STEP,
        "stance_vik": USE_STANCE_VIK,
        "stance_vik_kp": STANCE_VIK_KP,
        "stance_vik_clip": STANCE_VIK_CLIP,
        "stance_vik_ankle_only": STANCE_VIK_ANKLE_ONLY,
        "stance_hold": USE_STANCE_HOLD,
        "foot_pos_ik": USE_FOOT_POS_IK,
        "foot_pos_ik_kp": FOOT_POS_IK_KP,
        "zmp_qp": USE_ZMP_QP,
        "wbc_stance": bool(USE_WBC_STANCE),
        "wbc_mu": float(WBC_MU) if USE_WBC_STANCE else 0.0,
        "wbc_scrub_k": float(WBC_SCRUB_K) if USE_WBC_STANCE else 0.0,
        "hybrid_mpc": bool(USE_HYBRID_MPC),
        "mpc_n": int(MPC_N) if USE_HYBRID_MPC else 0,
        "mpc_u_max": float(MPC_U_MAX) if USE_HYBRID_MPC else 0.0,
        "mpc_apply_thr": float(MPC_APPLY_THR) if USE_HYBRID_MPC else 0.0,
        "mpc_disable_above_t": float(MPC_DISABLE_ABOVE_T) if USE_HYBRID_MPC else 0.0,
        "auth_k": float(AUTH_K),
        "auth_leg_tau": float(2.1 * AUTH_K),
        "auth_leg_pos": float(2.09 * AUTH_K),
        "auth_sat_count": int(AUTH_SAT_COUNT),
        "auth_sat_steps": int(AUTH_SAT_STEPS),
        "auth_sat_rate": (float(AUTH_SAT_COUNT) / float(max(1, AUTH_SAT_STEPS * max(1, len(AUTH_LEG_ACT_IDS))))) if AUTH_SAT_STEPS else 0.0,
        "tip_free_max_s": tip_free_max,
        "tip_free_post_gait_max_s": tip_free_post_gait_max,
        "disturb_stand": disturb_stand,
        "ank_cop_kp_lean_xy": [ANK_COP_KP_LEAN_X, ANK_COP_KP_LEAN_Y],
        "ank_cop_kd_lean_xy": [ANK_COP_KD_LEAN_X, ANK_COP_KD_LEAN_Y],
        "ank_cop_kp_xy": [ANK_COP_KP_X, ANK_COP_KP_Y],
        "ank_cop_clip": ANK_COP_CLIP,
        "plant_kd": PLANT_KD,
        "fx_bias": fx_bias,
        "v_des_x": v_des_x,
        "t_final": t_final,
        "gait_s": gait_s,
        "dx_m": dx,
        "avg_speed_m_s": avg_speed,
        "avg_speed_cm_s": avg_speed * 100,
        "exploded": exploded,
        "flipped": flipped,
        "upright_frac": upright_frac,
        "contact_duty_L": contact_duty_L,
        "contact_duty_R": contact_duty_R,
        "hip_q_ptp": hip_q_ptp,
        "knee_q_ptp": knee_q_ptp,
        "ank_q_ptp": ank_q_ptp,
        "hip_q_corr_LR": hip_q_corr,
        "knee_q_corr_LR": knee_q_corr,
        "shin_ptp_L": shin_ptp_l,
        "shin_ptp_R": shin_ptp_r,
        "shin_corr_LR": shin_corr,
        "shin_cycles": shin_cycles,
        "foot_lead_cycles": foot_lead_cycles,
        "lift_L_m": lift_l,
        "lift_R_m": lift_r,
        "lift_peaks_L": n_l,
        "lift_peaks_R": n_r,
        "rel_x_amp_L": rel_l_amp,
        "rel_x_amp_R": rel_r_amp,
        "stance_foot_vx_mean_L": stx_L_mean,
        "stance_foot_vx_mean_R": stx_R_mean,
        "stance_foot_vx_p95_L": stx_L_p95,
        "stance_foot_vx_p95_R": stx_R_p95,
        "skate": skate,
        "skate_reasons": skate_reasons,
        "body_z_min": pz_min,
        "body_z_max": pz_max,
        "body_z_mean": pz_mean,
        "torque_clip_leg_Nm": list(leg_clip),
        "torque_clip_arm_Nm": list(arm_clip),
        "acceptance": ACC,
        "n_frames": len(frame_paths),
        "mp4": str(args.out),
        "csv": str(csv_path),
        "still_png": str(still_png),
        "walk_attempt_png": str(still_png),
        "verdict": verdict,
        "fails": fails,
        "mesh_articulates": bool(mesh_ok),
        "clean_walk_claim": bool(clean_walk),
        "disturb_applied": bool(disturb_applied) if disturb_stand else False,
        "disturb_recovered": bool(disturb_recovered) if disturb_stand else False,
        "disturb_recover_t_s": (float(disturb_recover_t) if disturb_stand
                                and disturb_recover_t == disturb_recover_t else None),
        "disturb_max_lean": float(max_lean_post) if disturb_stand else None,
        "disturb_push_vx": float(args.push_vx) if disturb_stand else None,
        "disturb_push_vy": float(args.push_vy) if disturb_stand else None,
        "disturb_push_fx": float(args.push_fx) if disturb_stand else None,
        "disturb_push_fy": float(args.push_fy) if disturb_stand else None,
        "disturb_push_hold_s": float(args.push_hold) if disturb_stand else None,
        "disturb_pass": bool(disturb_pass) if disturb_stand else None,
        "cop_err_x_mean": float(np.mean(cop_err_x)) if cop_err_x else None,
        "cop_err_y_mean": float(np.mean(cop_err_y)) if cop_err_y else None,
        "cop_err_x_p95": float(np.percentile(cop_err_x, 95)) if len(cop_err_x) > 5 else None,
        "cop_err_y_p95": float(np.percentile(cop_err_y, 95)) if len(cop_err_y) > 5 else None,
    }
    stats_path = out_dir / "ainex_walk_stats.json"
    stats_path.write_text(json.dumps(stats, indent=2))
    (HIWONDER_DIR / "walk_gait_stats.json").write_text(json.dumps(stats, indent=2))
    if args.stats_out:
        sp = Path(args.stats_out)
        sp.parent.mkdir(parents=True, exist_ok=True)
        sp.write_text(json.dumps(stats, indent=2))
        print(f"[ainex] stats-out → {sp}")
    if use_ankle_cop or disturb_stand:
        if disturb_stand and use_stance_freeze:
            ankle_stats = HIWONDER_DIR / "ankle_cop_freeze_stats.json"
        elif disturb_stand:
            ankle_stats = HIWONDER_DIR / "ankle_cop_stand_stats.json"
        else:
            ankle_stats = HIWONDER_DIR / "ankle_cop_walk_stats.json"
        ankle_stats.write_text(json.dumps(stats, indent=2))
        print(f"[ainex] ankle stats → {ankle_stats}")
    print(f"[ainex] stats → {stats_path}")
    print(f"[ainex] VERDICT: {verdict}")

    note_path = HIWONDER_DIR / (
        "ankle_cop_note.txt" if (use_ankle_cop or disturb_stand) else "walk_gait_note.txt"
    )
    note_lines = [
        "AiNex ankle-CoP / walk_gait progress note (WORLD_FIXED acceptance)",
        "date: 2026-09-27 ~20:15+ BST",
        f"model: {xml}",
        "geometry: Hiwonder URDF+25 STL (NOT OEM STEP); Path A buy FROZEN",
        f"tag: {args.tag}",
        f"run_id: {run_id_short}",
        f"mode: {'disturb-stand' if disturb_stand else 'walk'}",
        "",
        "CONTROL LEVERS THIS RUN:",
        f"  + Ankle CoP servo: {'ON' if use_ankle_cop else 'OFF'} "
        f"lean=({ANK_COP_KP_LEAN_X},{ANK_COP_KP_LEAN_Y}) cop=({ANK_COP_KP_X},{ANK_COP_KP_Y}) clip={ANK_COP_CLIP}",
        f"  + Lateral COM shift: hip_roll amp={COM_SHIFT_AMP} rad, phase lead={COM_SHIFT_LEAD}",
        f"  + Stance plant damper: {'ON' if use_plant else 'OFF'} kd={PLANT_KD} N/(m/s) clip={PLANT_MAX_F} N",
        f"  + Stance-foot world freeze ablation: {'ON' if use_stance_freeze else 'OFF'}",
        f"  + Balance assist (upright/height/speed): {'ON' if use_assist else 'OFF'}",
        f"  + HX torque clips unchanged (±2.1 leg / ±0.7 arm-head)",
        f"  + Gait: T={GAIT_T}s hip_amp={HIP_PITCH_AMP} knee_swing={KNEE_SWING} COM_Z={COM_Z}",
        "",
        "METRICS:",
        f"  t_final={t_final:.2f}s  gait_s={gait_s:.2f}s  dx={dx:+.3f}m  avg_speed={avg_speed*100:.1f} cm/s",
        f"  upright_frac={upright_frac:.3f}  contact_duty L/R={contact_duty_L:.2f}/{contact_duty_R:.2f}",
        f"  hip_ptp={hip_q_ptp:.3f} knee_ptp={knee_q_ptp:.3f} hip_corr={hip_q_corr:.3f}",
        f"  foot_lead_cycles={foot_lead_cycles}  shin_cycles={shin_cycles}",
        f"  lift L/R={lift_l:.3f}/{lift_r:.3f} m",
        f"  stance |vx| mean L/R={stx_L_mean:.3f}/{stx_R_mean:.3f}  p95 L/R={stx_L_p95:.3f}/{stx_R_p95:.3f}",
        f"  skate={skate}  exploded={exploded}  flipped={flipped}",
        f"  cop_err mean x/y="
        f"{(float(np.mean(cop_err_x)) if cop_err_x else float('nan')):.4f}/"
        f"{(float(np.mean(cop_err_y)) if cop_err_y else float('nan')):.4f}",
        "",
    ]
    if disturb_stand:
        note_lines += [
            "DISTURB-STAND:",
            f"  push_dv=({args.push_vx:+.3f},{args.push_vy:+.3f}) m/s  F=({args.push_fx:+.2f},{args.push_fy:+.2f}) N hold={args.push_hold:.2f}s at t={disturb_t:.2f}s",
            f"  applied={disturb_applied} recovered={disturb_recovered} "
            f"recover_t={disturb_recover_t:.3f}s max_lean={max_lean_post:.3f}",
            f"  disturb_pass={disturb_pass}",
            "",
        ]
    note_lines += ["WORLD_FIXED GATES:"]
    for k_, v_ in ACC.items():
        note_lines.append(f"  {k_}: {v_}")
    note_lines += [
        "",
        f"VERDICT: {verdict}",
        f"clean_walk_claim: {clean_walk}",
        f"fails ({len(fails)}):",
    ]
    if fails:
        for f_ in fails:
            note_lines.append(f"  - {f_}")
    else:
        note_lines.append("  (none)")
    note_lines += [
        "",
        "WHAT STILL LACKS (if FAIL):",
        "  - Stronger capture-point / ZMP with swing placement under HX ±2.1 Nm",
        "  - True no-slip stance (plant/freeze are external; friction-limited contact preferred)",
        "  - Full mesh collision (boxes only); Path A OEM STEP still frozen",
        "",
        "NEXT CONTROL LEVER:",
        "  → If stand recovers but walk tips: raise CoP gains carefully / add hip strategy",
        "  → If skate: try --stance-freeze ablation, then raise foot friction/condim",
        "  → Do NOT claim walk while balance_assist supplies +X or quat lock",
        "",
        f"PNGs: {still_png}",
        f"MP4:  {args.out}",
        f"stats:{stats_path}",
        "script: scripts/walk_gait_ainex.py",
    ]
    note_path.write_text("\n".join(note_lines) + "\n")
    print(f"[ainex] note → {note_path}")
    if use_ankle_cop and not disturb_stand:
        (HIWONDER_DIR / "walk_gait_note.txt").write_text("\n".join(note_lines) + "\n")

    if disturb_stand:
        sys.exit(0 if disturb_pass else 1)
    sys.exit(0 if (not exploded or t_final >= 6.0) and not flipped else 1)



if __name__ == "__main__":
    main()
