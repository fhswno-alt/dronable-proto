#!/usr/bin/env python3
"""LIPM / ZMP outer loop, Raibert footholds, Bézier swing IK.

Replaces the open-loop CPG as the schedule. CSF50 numbers are not the clock
and are not swept here. Joint targets go to the existing position servos. The Bézier loop is
50 Hz (CTRL_DT 0.02 s) and approaches knee, hip pitch, and ankle pitch
over HIP_KNEE_MOVE_S (150 ms). The OP3 path plans at 8 ms and approaches
those three joints over ``gm_move_s`` (kit servo write 20 ms). Neither
path steps the command faster than the HX slew. Other joints stay
inside the linear band (|ctrl-q| <= 0.98 * tau / kp). Kit hip roll
uses the 2.33 Nm prediction budget while the bus yaw is away from 0.
Nothing in this file writes the plant:
no forcerange, kp, damping, or armature edits, and no free-joint wrench.
The actuator forcerange still clips force.

Weight moves onto the stance foot before the swing foot is allowed to rise.
The swing is a joint-space Bézier measured on this plant: knee flexion for
about 2 cm of level-sole clearance, hip pitch for a landing at most 2 cm
ahead. The ankle target is the hip and knee commands actually sent this
tick (the flat-foot sum), plus a measured-normal trim, so the box does not
ride a corner. Knee, hip pitch, and ankle pitch use the multi-tick
move. Other commands stay inside |ctrl-q| <= 0.98 * tau / kp.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Literal

import mujoco as mj
import numpy as np

import ds_split
import gait_manager_traj as gm
import op3_walk
import zmp_preview

CTRL_DT = 0.02
G = 9.81
LEG_TAU = 2.45
# Position-servo saturation boundary. Commanding past this does not add torque
# on joints that stay inside the linear band.
SAT_FRAC = 0.98
# HX-35H knee budget at about 10.5 V. The plant forcerange stays ±2.45.
# A knee past this is a Prefer FAIL. Not a forcerange edit.
KNEE_SAG_NM = 2.33
# Feedback on (q_ref − q) when the inverse-dynamics feedforward is on.
# This is not the plant actuator kp. The XML gain stays 45 / 40 / 35.
# 1 Nm/rad is small beside those gains. It is not a torque cap.
ID_FF_KP = 1.0
# File value on every joint. A load-time leg override replaces it on the
# walker with the compiled dof_armature. The strip has to use that number:
# mj_inverse already includes whatever armature the spec was compiled with.
# 0.01 is not a measured motor inertia. A planted hip pitch is about
# 0.2 kg·m², so 2.33/0.01 is not a legal q̈.
ID_FF_ARMATURE = 0.01
_LEG_JOINT_NAMES = (
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
)
# DC-motor model line, not a datasheet. |qvel| ≤ 5.82·(1 − |τ|/3.43).
DC_QVEL_LIM = 5.82
DC_STALL_NM = 3.43
# Discrete implicitfast residual.
# ID_RESID_EXACT_NM is the target. The measured nominal max is
# 1.476e-3 Nm, 4/2440 ticks, so that target is a Prefer FAIL.
# A tick is still bucketed while every leg joint is inside
# ID_RESID_BUCKET_NM (1e-2). The bucket does not relax the 2.33 Nm
# applied-ask bar, and it does not turn the 1e-3 miss into a pass.
ID_RESID_EXACT_NM = 1.0e-3
ID_RESID_BUCKET_NM = 1.0e-2
ID_RESID_NM = ID_RESID_BUCKET_NM
# Planned knee torque during the stop blend, including the 0.01 armature
# already in mj_inverse. The blend span grows until the knee reference
# acceleration keeps this budget. It is not a command clip.
PLAN_KNEE_TAU_NM = 2.0
# 0.01·q̈ at this cap is 0.25 Nm, inside the 2.0 Nm knee budget.
# At the 0.025 load-time armature the same cap is 0.625 Nm.
STOP_KNEE_QDD_MAX = 25.0
# Walk-reference knee acceleration. The double-support kink measured
# −109 rad/s². At armature 0.01 that is −1.66 Nm and the applied force
# stays inside 2.33. At 0.025 the same q̈ is −3.30 Nm. 40 rad/s² is
# 1.0 Nm at 0.025, which leaves room for the bare term.
WALK_KNEE_QDD_MAX = 40.0
STOP_BLEND_MIN_S = 0.5
# The position servo does not command the double-support load split.
# Each tick both feet are loaded, move the ZMP/CoM plan by a fraction of
# (α_qp − α_real) times the foot span, so the contact solver's normals
# are pulled toward the QP share. The bias is capped at 20 mm.
SPLIT_TRACK_STEP = 0.25
SPLIT_TRACK_STEP_M = 0.0015
SPLIT_TRACK_BIAS_M = 0.020
# Stop/silence command budget for every leg joint. Hip yaw, hip roll,
# hip pitch, knee, ankle pitch, and ankle roll share the HX-35H class.
# The sag bar stays 2.33. Sitting the prediction on that bar measured
# 2.330 Nm on the knees and 2.401 Nm on hip and ankle pitch. 0.05 Nm
# under the bar is the stop target. Not a forcerange edit.
LEG_STOP_HEADROOM_NM = 0.05
LEG_STOP_NM = KNEE_SAG_NM - LEG_STOP_HEADROOM_NM
# Older name. The stop budget is no longer knees-only.
KNEE_STOP_HEADROOM_NM = LEG_STOP_HEADROOM_NM
KNEE_STOP_NM = LEG_STOP_NM
# Locked kit row. Body speed is about this many (m/s) per meter of OP3
# x_amp, measured with the step scaled to the command. The bus forward
# clamp is KIT_BODY_PER_X * KIT_X_RAIL_M, so full stick is the speed the
# rail-safe step actually produces. KIT_X_RAIL_M is the longest step whose
# knees stayed at or under KNEE_SAG_NM. 0.020 m is the kit yaml step; it
# is not used when that step crosses the sag bar.
KIT_BODY_PER_X = 7.50
# Reverse is a shorter step for the same command. 0.032 m/s at the
# forward gain walked at about 0.023 m/s. This gain is the retreat
# per meter of step, so the −0.032 clamp is the body speed.
KIT_BODY_PER_X_REV = 5.90
KIT_X_RAIL_M = 0.020
# Step angle (rad per cycle) = yaw_rate * period * KIT_YAW_GAIN.
# +yaw_rate is a left turn. The walker already splits angle/2 across the feet.
# 0.50 keeps a full ±0.25 rad/s command off the hip rail. Straight body
# speed at x = 0.020 m is 0.150 m/s, so the bus cap is 7.50 * 0.020.
KIT_YAW_GAIN = 0.50
# Hiwonder no-load speed, https://www.hiwonder.com/products/hx-35h
# HX-35H (knee / leg) 0.18 s/60° at 11.1 V ≈ 5.8 rad/s.
# HX-35HM (hip) 0.19 s/60° ≈ 5.5 rad/s.
# Command tick is 50 Hz. A hard step of the whole gait target in one
# 20 ms tick rails kp·error while speed is still ~0. Clamping each tick
# to 0.050 rad (forcerange/kp) keeps the force near 2.0 Nm and the knee
# never reaches the mid-swing pose. Kit SERVO_MOVE_TIME is longer than
# one tick. Hip, knee, and ankle pitch therefore take HIP_KNEE_MOVE_S
# to approach the latest target, at the physics rate, and never faster
# than the HX slew below. Do not exceed ~5.8 rad/s.
HX35_SLEW_RAD_S = 5.5
# Middle of the 100–200 ms kit-style move. One 20 ms tick covers
# CTRL_DT/HIP_KNEE_MOVE_S of the remaining gap.
HIP_KNEE_MOVE_S = 0.150
# Hiwonder GaitManager look: step height about 2 cm, step length at most 2 cm.
# These are foothold clips, not CommandBus caps and not a torque change.
VENDOR_CLEAR_M = 0.020
VENDOR_STEP_M = 0.020
# Joint Bézier measured with the pelvis fixed, the swing sole level, and
# ank = hip + knee. flex 0.62 / hip 0.04 clears about 2.5 cm. Landing at
# flex 0.10 / hip 0.16 puts the sole on the floor about 2 cm ahead.
# A taller clear_m scales the same shape; the servo band may not finish it.
FLEX_PEAK = 0.62
FLEX_LAND = 0.10
HIP_RISE = 0.06
HIP_REACH = 0.24
HIP_LAND = 0.24
# Stance hip extension during swing (opposite the swing-flex sign) carries
# the pelvis forward over the planted foot. On the ±2.45 Nm / 135×76 plant,
# 0.070 rad stays upright for 24 s and Δx is positive. 0.075 rad tips near
# 12 s. 0.08 rad tipped immediately on the ±2.1 freeze. This is a few
# millimetres per step, not the 0.056 m/s command.
STANCE_PUSH = 0.070
# Rise is long enough for an unloaded knee (dampratio 1, about 1 rad/s)
# to clear ~2 cm. The reach holds that flexion and swings the hip through,
# because extending while the foot is still behind plants it early.
RISE_S = 0.42
REACH_S = 0.74
# Stable lateral-shift basin measured on this plant (stand pose + these
# deltas at |lat|=1). Larger ankle roll tipped. Not a CPG amplitude.
SHIFT_HIP_L = 0.15
SHIFT_HIP_R = 0.20
SHIFT_ANK = 0.15
# Rear-knee yield once the lean is on the forward foot. Two 14.5 cm soles
# a couple of centimetres apart both contain the COM. The exclusive toe of
# the front foot needs ~2.36 Nm and is not the target. Shortening the rear
# leg drops its normal force; the front heel/mid already covers the COM
# at a fraction of ±2.1 Nm. Not a hip extension.
YIELD_LAT = 0.80
YIELD_KNEE = 0.42
YIELD_RATE = 1.40
# Open-loop hip during the yield retracts the pelvis. The hold below only
# adds hip when the rear sole has already slid backward.
YIELD_HIP_MAX = 0.28
# Swing hip is the measured along-track error to a 2 cm lead, not a fixed
# pose. 0.24 rad from a side-by-side start lands about 2 cm up; a rear foot
# that yield pulled back needs more, and it is still capped.
HIP_PLACE_MAX = 0.48
COP_PITCH_SIGN = 1.0
# Single-support CoP this far behind the COM projection. x_ddot = (g/h)*(x-p),
# so a few millimetres of heel offset is the forward acceleration. Not the toe.
HEEL_LEAD_M = 0.006

Side = Literal["L", "R"]
PhaseName = Literal["stand", "shift", "swing"]


@dataclass(frozen=True)
class LipmConfig:
    """One A/B row. clear_m is the Bézier sole peak, not a CPG knee offset."""

    name: str
    clear_m: float
    arms: bool
    t_swing: float = 1.15
    t_shift_min: float = 0.24
    t_shift_max: float = 1.50
    ik_kp: float = 8.0
    shift_k: float = 2.2
    cop_k: float = 1.6
    arm_amp: float = 0.22
    unload_n: float = 5.0
    inside_margin_m: float = 0.008
    # "lipm" is the joint Bézier. "gait_manager" is the published Hiwonder
    # / ROBOTIS schedule (period, 2 cm, x amplitude) into the same servos.
    schedule: Literal["lipm", "gait_manager"] = "lipm"
    gm_period_s: float = gm.GM_PERIOD_S
    gm_dsp: float = gm.GM_DSP
    gm_y_swap_m: float = gm.GM_Y_SWAP_M
    gm_x_m: float = gm.GM_X_M
    gm_z_m: float = gm.GM_Z_M
    gm_z_swap_m: float = gm.GM_Z_SWAP_M
    gm_step_fb: float = gm.GM_STEP_FB
    gm_pelvis_deg: float = gm.GM_PELVIS_DEG
    # First swing after stand. "L" is the published clock (time 0). "R"
    # starts in the double support before the right swing. Not a clamp.
    gm_start_lead: str = "L"
    # After a nonzero yaw target returns to 0, the next double support
    # lines up the outside foot (right after +yaw, left after −yaw).
    # "keep" leaves the clock alone. "L" or "R" forces that swing.
    gm_resume_lead: str = "outside"
    gm_arm_deg: float = gm.GM_ARM_DEG
    # Kit hip_pitch_offset, on the stand pose and the walk. 15° takes
    # the 0.025 m crouch from hip pitch +0.504 rad to +0.766 rad.
    gm_hip_pitch_deg: float = gm.GM_HIP_PITCH_DEG
    # A loaded swing foot is not dragged along x. The z target still rises.
    gm_drag_gate: bool = False
    # Hold the published step height through the first half of single support
    # so the servo can arrive. The sine peaks for one sample and the knee
    # never gets there. Off is the raw wSin track.
    gm_z_hold: bool = False
    # Fraction of single support that holds the step height before descent.
    gm_hold_u: float = 0.55
    # Stance hip from the published x. On the 150 ms / 1.16 s row, 0.070
    # rad stays off the hip rail and 0.075 rad puts the right hip on
    # ±2.45 Nm while up_z is still 1. The default 0.48 leaves the clip
    # open so an unclipped run can show the tip.
    gm_stance_max: float = 0.48
    # Swing hip only. Read the clock this early so the 150 ms move is
    # already underway when the foot leaves the floor. 0.22 s is the
    # 1.16 s row that stayed off the hip rail. 0.20 s rails the right
    # hip. 0 keeps the live sample.
    gm_hip_lead_s: float = 0.22
    # Swing hip only. With the knee at 1.05 rad, 1.75 keeps both soles
    # near 2 cm and the hip pitch off ±2.45 Nm. 2.80 with that knee rails
    # the hip. The stance clip is not scaled.
    gm_swing_hip_gain: float = 1.75
    # Kit walking_param init_z_offset. 0.025 m. With the kit stance of
    # +0.005 m outward on each foot, a 2 s hold is 11.51 / 11.52 N per
    # foot. 0 leaves the geometric full extension.
    gm_crouch_m: float = 0.025
    # OP3 path only. Kit servo_control_cycle is 0.02 s; the planner is
    # already 8 ms, so a 150 ms approach is a second smoother. 0.008 and
    # 0.016 are the other measured rows. The Bézier loop ignores this
    # and keeps HIP_KNEE_MOVE_S.
    gm_move_s: float = 0.020
    # Unused by the IK path. Kept so older call sites still construct.
    gm_sway_max: float = 0.15
    # Lateral ZMP preview, metres. 0 leaves the OP3 path unchanged.
    # This is not y_swap. y_swap_cmd stays at gm_y_swap_m.
    preview_amp_m: float = 0.0
    # 0 measures CoM height above the sole on the first preview tick.
    preview_zc_m: float = 0.0
    # Seconds of double support used to shift onto the first stance foot
    # before the clock starts. Not a torque cap.
    preview_arm_s: float = 0.60
    # Cart-table jerk weight. 1e-4 is the kit-tight preview. A larger R
    # slows the sway so hip-roll rate stays under the unclamped bar.
    preview_r: float = 1.0e-4
    # 1 is the raised-cosine sway. Lower spends the double support closer
    # to a steady hip-roll rate. 0 is a straight ramp.
    preview_shape: float = 1.0
    # Rest-to-rest swing z and swing x. Off keeps z_flat / the sine.
    gm_z_quintic: bool = False
    # Start that quintic at the previous touchdown so the rise covers
    # double support plus the first half of single support.
    gm_z_lead: bool = False
    # Spring budget for the sagittal command, Nm. 0 leaves the IK target.
    # A positive value rate- and accel-limits hip pitch, knee, and ankle
    # pitch, and keeps |ctrl−q| inside budget/kp so kp·|e| cannot exceed
    # the budget. ω is not part of that projection: this is not a signed
    # torque clamp.
    gm_spring_nm: float = 0.0
    # Inverse-dynamics feedforward through the position servo. Off leaves
    # the IK target. On, every leg ctrl is
    # q + (τ_ff + kv·q̇ + K_fb·(q_ref−q)) / kp. τ_ff is the inverse of
    # the planned gait (q_ref, q̇_ref, q̈_ref) with the planned contact
    # wrench. It is not saturated into ±KNEE_SAG_NM. The plant kp stays
    # in the XML.
    gm_id_ff: bool = False


@dataclass
class LiftRecord:
    t: float
    stance: str
    cop_margin_m: float
    swing_fn_n: float
    com_inside: bool
    clear_cmd_m: float
    z_budget_m: float
    cop_x_m: float = 0.0
    rear_share: float = 1.0


@dataclass
class LipmTrace:
    t: list[float] = field(default_factory=list)
    sole_l: list[float] = field(default_factory=list)
    sole_r: list[float] = field(default_factory=list)
    fn_l: list[float] = field(default_factory=list)
    fn_r: list[float] = field(default_factory=list)
    contact_l: list[int] = field(default_factory=list)
    contact_r: list[int] = field(default_factory=list)
    phase: list[str] = field(default_factory=list)
    swing: list[str] = field(default_factory=list)
    stance: list[str] = field(default_factory=list)
    up_z: list[float] = field(default_factory=list)
    sat: list[int] = field(default_factory=list)
    slip_l: list[float] = field(default_factory=list)
    slip_r: list[float] = field(default_factory=list)
    z_cmd: list[float] = field(default_factory=list)
    z_bez: list[float] = field(default_factory=list)
    knee_band: list[float] = field(default_factory=list)
    sho_l: list[float] = field(default_factory=list)
    sho_r: list[float] = field(default_factory=list)


def _quintic_coeff(q0: float, q1: float, v0: float, span: float, a0: float = 0.0) -> tuple[float, float, float, float, float]:
    """Rest-to-rest quintic with a matched initial velocity. a(0) = a0, a(T) = 0."""
    span = max(float(span), 1e-6)
    b1 = float(v0) * span
    b2 = 0.5 * float(a0) * span * span
    remain = (float(q1) - float(q0)) - b1 - b2
    end_vel = -b1 - 2.0 * b2
    end_acc = -2.0 * b2
    d_coef = end_vel - 3.0 * remain
    e_coef = end_acc - 6.0 * remain
    c5 = 0.5 * (e_coef - 6.0 * d_coef)
    c4 = d_coef - 2.0 * c5
    c3 = remain - c4 - c5
    return b1, b2, c3, c4, c5


def _quintic_sample(
    q0: float, q1: float, v0: float, span: float, t_s: float, a0: float = 0.0,
) -> tuple[float, float, float]:
    """Position, velocity, and acceleration of the stop quintic at t_s."""
    span = max(float(span), 1e-6)
    u = min(1.0, max(0.0, float(t_s) / span))
    b1, b2, c3, c4, c5 = _quintic_coeff(q0, q1, v0, span, a0)
    q = q0 + u * (b1 + u * (b2 + u * (c3 + u * (c4 + c5 * u))))
    dqdu = b1 + u * (2.0 * b2 + u * (3.0 * c3 + u * (4.0 * c4 + 5.0 * c5 * u)))
    d2du = 2.0 * b2 + u * (6.0 * c3 + u * (12.0 * c4 + 20.0 * c5 * u))
    return q, dqdu / span, d2du / (span * span)


def _quintic_qdd_peak(q0: float, q1: float, v0: float, span: float) -> float:
    """Peak |q̈| of the stop quintic. Sampled, the analytic ends are zero."""
    peak = 0.0
    for i in range(65):
        _q, _qd, qdd = _quintic_sample(q0, q1, v0, span, (i / 64.0) * span)
        peak = max(peak, abs(qdd))
    return peak


def _leg_joint(name: str) -> bool:
    return name.endswith((
        "hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll",
    ))


def bezier_foot(s: float, p0: np.ndarray, p3: np.ndarray, clear_m: float) -> np.ndarray:
    """Cubic Bézier. Control height is clear/0.75 so the peak equals clear_m."""
    s = min(1.0, max(0.0, s))
    height = max(0.0, clear_m) / 0.75
    p1 = p0.copy()
    p1[2] = p0[2] + height
    p2 = p3.copy()
    p2[2] = p3[2] + height
    u = 1.0 - s
    return u**3 * p0 + 3.0 * u**2 * s * p1 + 3.0 * u * s**2 * p2 + s**3 * p3


def _compiled_leg_armature(model: mj.MjModel) -> float:
    """Armature on the twelve leg joints. They share one compiled value."""
    vals: list[float] = []
    for name in _LEG_JOINT_NAMES:
        jid = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name))
        if jid < 0:
            raise RuntimeError(f"missing joint {name}")
        vals.append(float(model.dof_armature[int(model.jnt_dofadr[jid])]))
    if max(vals) - min(vals) > 1e-9:
        raise RuntimeError(f"leg armatures differ: {min(vals)} .. {max(vals)}")
    return float(vals[0])


def bezier_peak_z(p0_z: float, clear_m: float) -> float:
    """Peak of bezier_foot when the landing z equals the liftoff z."""
    return p0_z + clear_m


class LipmWalker:
    """50 Hz outer loop. Writes position targets only."""

    def __init__(
        self,
        model: mj.MjModel,
        data: mj.MjData,
        act_idx: dict[str, int],
        cfg: LipmConfig,
        q_stand: dict[str, float],
    ) -> None:
        self.model = model
        self.leg_armature = _compiled_leg_armature(model)
        self._knee_ref: dict[str, list[float]] = {}
        self.data = data
        self.act_idx = act_idx
        self.cfg = cfg
        self.q_stand = dict(q_stand)
        self.op3: op3_walk.Op3Walker | None = None
        if cfg.schedule == "gait_manager":
            # Cartesian body drop through the OP3 IK. The old +0.34 rad knee
            # was not a 1.5 cm body-z change and is not applied on this path.
            self.op3 = op3_walk.Op3Walker.from_model(
                model,
                period_s=cfg.gm_period_s,
                dsp=cfg.gm_dsp,
                y_swap_m=cfg.gm_y_swap_m,
                z_move_m=cfg.gm_z_m,
                x_amp_m=cfg.gm_x_m,
                z_offset_m=cfg.gm_crouch_m,
                z_swap_m=cfg.gm_z_swap_m,
                step_fb=cfg.gm_step_fb,
                pelvis_deg=cfg.gm_pelvis_deg,
                hip_pitch_deg=cfg.gm_hip_pitch_deg,
            )
            # Lateral preview spawns on a level sole. Kit stays pitched.
            if cfg.preview_amp_m > 1e-6:
                self.op3.sole_level = 1.0
            for name, val in self.op3.stand_joints().items():
                if "sho" in name:
                    continue
                self.q_stand[name] = val
        self.phase: PhaseName = "stand"
        self.stance: Side = "L"
        lead = str(cfg.gm_start_lead)
        self.start_lead: Side = "R" if lead == "R" else "L"
        self.resume_lead = str(cfg.gm_resume_lead)
        self.yaw_target = 0.0
        self._move_entered = False
        self._yaw_sign = 0
        self._pending_lead: str | None = None
        self._resume_latched = False
        self.lat = 0.0
        self.phase_t = 0.0
        self.swing_s = 0.0
        self.p0 = np.zeros(3, dtype=np.float64)
        self.p3 = np.zeros(3, dtype=np.float64)
        self.hold_l = np.zeros(3, dtype=np.float64)
        self.hold_r = np.zeros(3, dtype=np.float64)
        self.z_cmd = 0.0
        self.z_bez = 0.0
        self.z_budget = 0.0
        self.z_limited = False
        self.cmd_vx = 0.0
        self.cmd_yaw = 0.0
        self.swing_q0: dict[str, float] = {}
        self.swing_held = False
        self.stance_q0: dict[str, float] = {}
        self.hold_sagittal: dict[Side, dict[str, float]] = {}
        self.sag_lean = 0.0
        self.yield_flex = 0.0
        self.yield_hip = 0.0
        self.yield_along = 0.0
        self.liftoff_along = 0.0
        self.reach_hip = 0.0
        self.x0 = float(data.qpos[0])
        self._same_misses = 0
        self.lifts: list[LiftRecord] = []
        self.trace = LipmTrace()
        self.missed_gates = 0
        self._prev_foot: dict[Side, np.ndarray] = {
            "L": np.zeros(2, dtype=np.float64),
            "R": np.zeros(2, dtype=np.float64),
        }
        self._have_prev = False
        self.bid_body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
        self.bid = {
            "L": mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "l_ank_roll_link"),
            "R": mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "r_ank_roll_link"),
        }
        self.gid = {
            "L": mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "l_foot_contact"),
            "R": mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "r_foot_contact"),
        }
        self.gid_floor = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "floor")
        # Extra ground geoms, empty on the walk plant. The entrance rug
        # is added here at runtime so a foot on the mat still counts as load.
        self.ground_extra: tuple[int, ...] = ()
        self.e_sat: dict[str, float] = {}
        self._kp: dict[str, float] = {}
        for name, idx in act_idx.items():
            kp = float(model.actuator_gainprm[idx, 0])
            tau = float(model.actuator_forcerange[idx, 1])
            self._kp[name] = kp
            self.e_sat[name] = SAT_FRAC * tau / kp if kp > 1e-6 else 0.0
        # Refuse a silent gain edit. These are the frozen position-servo kp.
        expected = {
            "l_knee_pos": 45.0,
            "l_hip_pitch_pos": 45.0,
            "l_ank_pitch_pos": 35.0,
            "l_hip_roll_pos": 40.0,
            "l_ank_roll_pos": 35.0,
        }
        for name, kp in expected.items():
            if abs(self._kp.get(name, -1.0) - kp) > 1e-6:
                raise RuntimeError(f"frozen kp changed for {name}")
        self._gm_swing: Side | None = None
        self.preview_stage = "stand"
        self.preview_com_y = 0.0
        self._preview: zmp_preview.ZmpPreview | None = None
        self._preview_clock = 0.0
        self._preview_zc = 0.0
        self._stop_planted = False
        self._stop_hold_s = 0.0
        self._return_t = 0.0
        self._return_zmp0 = 0.0
        self._return_hold = False
        self._return_done = False
        self._freeze_time = 0.0
        self._stand_q0: dict[str, float] | None = None
        self._stand_q1: dict[str, float] | None = None
        self._stand_v0: dict[str, float] | None = None
        self._stand_u = 0.0
        self._stop_span = 0.0
        self._com_v0 = 0.0
        self._stop_swing_end: float | None = None
        self.id_stop_span = 0.0
        self.id_stop_qdd = 0.0
        self.preview_ik_fail = 0
        self._gate_wait_s = 0.0
        self._gate_open = False
        self.preview_gate_holds = 0
        # A later vel, after the soft stop has planted, arms again at the
        # kit pitch. The spawn is the only place sole_level starts at 1.
        self._restart = False
        # Rate-limited CoP / DCM offsets. Not a torque cap.
        self._stab_preview_m = 0.0
        self._stab_hip = 0.0
        self._stab_roll = {"L": 0.0, "R": 0.0}
        self._stab_pitch = {"L": 0.0, "R": 0.0}
        # Sagittal command shaper. Position and velocity of ctrl, not a
        # torque projection.
        self._shape_pos: dict[str, float] = {}
        self._shape_vel: dict[str, float] = {}
        # Inverse-dynamics feedforward. Scratch data and the largest
        # inverse torque at this step's data.qacc.
        self._id_data: mj.MjData | None = None
        self._leg_dof: dict[str, int] = {}
        self.id_ff_peak_abs = 0.0
        self.id_ff_peak_joint = ""
        self.id_ff_peak_phase = ""
        self.id_ff_peak_term = ""
        self.id_ff_peak_t = 0.0
        self.id_ff_peak_tau = 0.0
        self.id_ff_over_n = 0
        self.id_ff_wall_n = 0
        self.id_ff_arm_n = 0
        self.id_ff_bucket = ""
        self.id_ff_arm = 0.0
        self.id_ff_stripped = 0.0
        self.id_ff_stripped_abs = 0.0
        self.id_ff_hold_abs = 0.0
        self.id_ff_hold_joint = ""
        self.id_ff_hold_phase = ""
        self.id_ff_hold_term = ""
        self.id_ff_hold_tau = 0.0
        self.id_ff_hold_t = 0.0
        self.id_ff_hold_over_n = 0
        self._id_split_residual = 0.0
        self.id_ff_impossible = 0
        self.id_ff_broke = 0
        # Consistent inverse from the last forward, before integration.
        self._consistent_tau: dict[str, float] = {}
        self.id_tick_tau: dict[str, float] = {}
        self.id_tick_arm: dict[str, float] = {}
        self.id_tick_stripped: dict[str, float] = {}
        self.id_tick_resid: dict[str, float] = {}
        self.id_tick_resid_pas: dict[str, float] = {}
        self.id_tick_ok: dict[str, bool] = {}
        self.id_phys_n = 0
        self.id_resid_fail_n = 0
        self.id_resid_max = 0.0
        self.id_resid_joint = ""
        self.id_resid_pas_max = 0.0
        self.id_root_max = 0.0
        self.id_root_t = 0.0
        self.id_root_dof = -1
        self.id_root_fail_n = 0
        self.id_root_bal_max = 0.0
        self.id_ident_max = 0.0
        self.id_inv_gap_max = 0.0
        self.id_inv_gap_joint = ""
        self.id_mj_abs = 0.0
        self.id_mj_tau = 0.0
        self.id_mj_joint = ""
        self.id_mj_t = 0.0
        self._plan_data: mj.MjData | None = None
        self._plan_q_hist: list[dict[str, float]] = []
        self._zmp_cmd = 0.0
        self.id_plan_tau = 0.0
        self.id_plan_abs = 0.0
        self.id_plan_ask = 0.0
        self.id_plan_joint = ""
        self.id_plan_phase = ""
        self.id_plan_term = ""
        self.id_plan_t = 0.0
        self.id_tick_term: dict[str, str] = {}
        self.id_tick_root = 0.0
        self.id_fwdinv0_max = 0.0
        self.id_fwdinv1_max = 0.0
        self.id_fwdinv_t = 0.0
        self.id_tick_fwdinv0 = 0.0
        self.id_tick_fwdinv1 = 0.0
        self.id_impl_max = 0.0
        self.id_impl_at_resid = 0.0
        self.id_band_at_resid = ID_RESID_NM
        self.id_resid_over_n = 0
        self.id_resid_exact_n = 0
        self.id_rail_n = 0
        self.id_req_with: dict[str, dict[str, float | str]] = {}
        self.id_req_bare: dict[str, dict[str, float | str]] = {}
        self.id_req_stop_with: dict[str, dict[str, float | str]] = {}
        self.id_req_stop_bare: dict[str, dict[str, float | str]] = {}
        self.id_req_arm = False
        self.id_req_arm_joint = ""
        self.id_req_arm_tau = 0.0
        self.id_req_arm_bare = 0.0
        self.id_req_arm_phase = ""
        self.id_req_arm_t = 0.0
        self.id_req_wall = False
        self.id_req_wall_n = 0
        self.id_req_wall_joint = ""
        self.id_req_wall_tau = 0.0
        self.id_req_wall_bare = 0.0
        self.id_req_wall_phase = ""
        self.id_req_wall_t = 0.0
        self.id_req_knee_abs = 0.0
        self.id_req_knee_tau = 0.0
        self.id_req_knee_joint = ""
        self.id_req_knee_phase = ""
        self.id_req_knee_t = 0.0
        self.id_req_knee_qdd = 0.0
        self.id_req_knee_bare = 0.0
        # Double-support split comparison. (c) is the feedforward share.
        # A wall is (c)'s bare torque, or the single-support wrench. (a)
        # and (b) are the spread, not a wall.
        self.id_split_ds_n = 0
        self.id_split_stop_n = 0
        self.id_split_stop_infeas = 0
        self.id_split_infeas = 0
        self.id_split_knee_spread = 0.0
        self.id_split_knee_joint = ""
        self.id_split_knee_phase = ""
        self.id_split_knee_t = 0.0
        self.id_split_knee_a = 0.0
        self.id_split_knee_b = 0.0
        self.id_split_knee_c = 0.0
        self.id_split_stop_knee_spread = 0.0
        self.id_split_stop_knee_joint = ""
        self.id_split_stop_knee_t = 0.0
        self.id_split_stop_knee_a = 0.0
        self.id_split_stop_knee_b = 0.0
        self.id_split_stop_knee_c = 0.0
        self.id_split_peak_spread = 0.0
        self.id_split_peak_phase = ""
        self.id_split_peak_t = 0.0
        self.id_split_peak_a = 0.0
        self.id_split_peak_b = 0.0
        self.id_split_peak_c = 0.0
        self.id_split_a_max = 0.0
        self.id_split_b_max = 0.0
        self.id_split_c_max = 0.0
        self.id_split_a_joint = ""
        self.id_split_b_joint = ""
        self.id_split_c_joint = ""
        self.id_split_a_resid_m = 0.0
        self.id_split_bare: dict[str, dict[str, dict[str, float | str]]] = {
            "a": {}, "b": {}, "c": {},
        }
        self.id_split_stop_bare: dict[str, dict[str, dict[str, float | str]]] = {
            "a": {}, "b": {}, "c": {},
        }
        # QP CoP margin to the declared 135×76 edge, per loaded foot.
        self.id_qp_margin_n = {"L": 0, "R": 0}
        self.id_qp_margin_sum = {"L": 0.0, "R": 0.0}
        self.id_qp_margin_min = {"L": 1.0, "R": 1.0}
        self.id_qp_margin_min_t = {"L": 0.0, "R": 0.0}
        # |α_qp − α_line| on solved QP ticks. The line share is the ZMP's
        # barycentric weight; the QP may leave it by using the CoP box.
        self.id_qp_line_n = 0
        self.id_qp_line_sum = 0.0
        self.id_qp_line_max = 0.0
        # Realised contact split against the QP, both feet above 5 N.
        self.id_real_n = 0
        self.id_real_split_sum = 0.0
        self.id_real_split_max = 0.0
        self.id_real_split_t = 0.0
        self.id_real_split_qp = 0.0
        self.id_real_split_fn = 0.0
        self.id_real_qp_sum = 0.0
        self.id_real_fn_sum = 0.0
        self.id_real_cop_n = {"L": 0, "R": 0}
        self.id_real_cop_sum = {"L": 0.0, "R": 0.0}
        self.id_real_cop_max = {"L": 0.0, "R": 0.0}
        self.id_real_cop_t = {"L": 0.0, "R": 0.0}
        self._qp_sample: dict | None = None
        self._split_zmp_bias = 0.0
        self._split_zmp_applied = 0.0
        self.id_split_bias_max = 0.0
        self.id_ff_resid = 0.0
        self._id_qvel0: np.ndarray | None = None
        self.id_tick_offset: dict[str, float] = {}
        self.id_tick_band: dict[str, float] = {}
        self._leg_kv: dict[str, float] = {}
        self.id_knee_ok_abs = 0.0
        self.id_knee_ok_tau = 0.0
        self.id_knee_ok_joint = ""
        self.id_knee_ok_t = 0.0
        self.id_knee_ok_phase = ""
        self.id_knee_act = 0.0
        self.id_knee_ok_n = 0
        self.id_knee_up_abs = 0.0
        self.id_knee_up_tau = 0.0
        self.id_knee_up_joint = ""
        self.id_knee_up_t = 0.0
        self.id_knee_up_phase = ""
        self.id_knee_up_act = 0.0
        self.id_knee_up_resid = 0.0
        self.id_hold_tau = 0.0
        self.id_hold_t = -1.0
        self.id_hold_act = 0.0
        self.id_hold_phase = ""
        self.id_hold_up = 0.0
        self.id_hold_dt = float("inf")
        self.id_rail_hits: list[tuple[float, str, float, float]] = []
        self.id_resid_vs_pas = 0.0
        self.id_wall_phys_n = 0
        self.id_arm_phys_n = 0
        self.ctrl_ticks = 0
        self.limit_ticks = 0
        self.limit_ctrl_n = 0
        self.limit_force_n = 0
        self.limit_slew_n = 0
        self.limit_band_n = 0
        self.limit_torque_n = 0
        self._limit_this_tick = False

    def other(self, side: Side) -> Side:
        return "R" if side == "L" else "L"

    def pref(self, side: Side) -> str:
        return "l_" if side == "L" else "r_"

    def q(self, jn: str) -> float:
        jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, jn)
        return float(self.data.qpos[self.model.jnt_qposadr[jid]])

    def write_clipped(self, jn: str, q_des: float) -> None:
        """Position target. Hip, knee, and ankle pitch keep the gait goal.

        Those three are approached over HIP_KNEE_MOVE_S by the physics
        loop, so this function stores the goal rather than a one-tick
        step. Ankle pitch is included so the flat-foot sum moves with
        the hip and knee. Other joints stay inside
        |ctrl-q| <= 0.98 * tau / kp and finish inside the 20 ms tick.
        The plant forcerange is unchanged and still clips force at
        ±2.45 Nm.
        """
        act = f"{jn}_pos"
        idx = self.act_idx.get(act)
        if idx is None:
            return
        # Feedforward writes a torque-sized ctrl on every leg joint.
        # The 0.98·τ/kp band would clip the spring term that cancels
        # kv·ω and the applied ask would no longer equal τ.
        leg_torque = bool(self.cfg.gm_id_ff) and _leg_joint(jn)
        if jn.endswith(("knee", "hip_pitch", "ank_pitch")) or leg_torque:
            cmd = q_des
        else:
            q = self.q(jn)
            band = self.e_sat.get(act, 0.0)
            cmd = min(q + band, max(q - band, q_des))
        lo_lim = float(self.model.actuator_ctrlrange[idx, 0])
        hi_lim = float(self.model.actuator_ctrlrange[idx, 1])
        # MuJoCo clips ctrl to ctrlrange before the force. The ask log
        # records q_des before this clip. A leg command outside ±2.09
        # is a fail even though the applied ctrl is the clipped value.
        applied = min(hi_lim, max(lo_lim, cmd))
        if _leg_joint(jn):
            if abs(cmd - float(q_des)) > 1e-12:
                self.limit_band_n += 1
                self._limit_this_tick = True
            if abs(applied - cmd) > 1e-12:
                self.limit_ctrl_n += 1
                self._limit_this_tick = True
        self.data.ctrl[idx] = applied

    def write_force_limited(self, jn: str, q_des: float, limit_nm: float | None = None) -> None:
        """Stand target whose predicted servo force stays inside the budget.

        Predicted force is kp·(ctrl−q) − kv·ω. The default budget is
        0.98·forcerange. A knee may pass a lower budget (the 10.5 V sag)
        without editing the plant forcerange.
        """
        act = f"{jn}_pos"
        idx = self.act_idx.get(act)
        if idx is None:
            return
        q = self.q(jn)
        jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, jn)
        omega = float(self.data.qvel[int(self.model.jnt_dofadr[jid])])
        kp = float(self.model.actuator_gainprm[idx, 0])
        kv = -float(self.model.actuator_biasprm[idx, 2])
        tau = abs(float(self.model.actuator_forcerange[idx, 1]))
        limit = SAT_FRAC * tau
        if limit_nm is not None:
            limit = min(limit, float(limit_nm))
        if kp < 1e-6:
            cmd = q_des
        else:
            # F = kp·(ctrl−q) − kv·ω. Keep that prediction inside ±limit.
            e_des = q_des - q
            e_lo = (-limit + kv * omega) / kp
            e_hi = (limit + kv * omega) / kp
            if e_lo > e_hi:
                e_lo, e_hi = e_hi, e_lo
            cmd = q + min(e_hi, max(e_lo, e_des))
        lo_lim = float(self.model.actuator_ctrlrange[idx, 0])
        hi_lim = float(self.model.actuator_ctrlrange[idx, 1])
        applied = min(hi_lim, max(lo_lim, cmd))
        if _leg_joint(jn) and abs(applied - float(q_des)) > 1e-12:
            self.limit_force_n += 1
            self._limit_this_tick = True
        self.data.ctrl[idx] = applied

    def hold_stand(self) -> None:
        self.phase = "stand"
        self.phase_t = 0.0
        self.swing_s = 0.0
        self.lat *= 0.8
        for jn, val in self.q_stand.items():
            # Stop and silence land here. The 20 ms walk slew is not used:
            # the caller applies this command on the same tick. Every leg
            # joint uses LEG_STOP_NM so the measured peak stays 0.05 Nm
            # under the 2.33 bar. Arms stay inside 0.98·τ.
            if any(tok in jn for tok in ("hip_", "knee", "ank_")):
                self.write_force_limited(jn, val, LEG_STOP_NM)
            else:
                self.write_force_limited(jn, val)

    def tick(self, vx: float, yaw_rate: float, walking: bool) -> None:
        self.cmd_vx = float(vx)
        self.cmd_yaw = float(yaw_rate)
        if self.cfg.schedule == "gait_manager":
            self._tick_gait_manager(walking)
            return
        if not walking:
            self.hold_stand()
            self.z_cmd = float(self.data.geom_xpos[self.gid["L"]][2])
            self.z_bez = self.z_cmd
            return
        self.phase_t += CTRL_DT
        if self.phase == "stand":
            self._begin_shift()
        if self.phase == "shift":
            self._tick_shift()
        elif self.phase == "swing":
            self._tick_swing(yaw_rate)
        self._write_arms()
        self._write_unused()

    def _arm_resume_lead(self, walker: op3_walk.Op3Walker) -> None:
        """After a yaw target returns to 0, swing the outside foot next.

        Applied only in double support, once per resume. The pose catches
        up across that double support and the following swing, instead of
        jumping in one tick. A foot already in single support waits.
        ``keep`` does not move the clock.
        """
        if self._pending_lead is not None:
            upcoming = walker.next_swing()
            if upcoming is None:
                return
            if upcoming != self._pending_lead:
                walker.arm_swing(self._pending_lead)
            self._pending_lead = None
            self._resume_latched = True
            return
        policy = self.resume_lead
        yaw = float(self.yaw_target)
        if abs(yaw) > 1e-3:
            self._yaw_sign = 1 if yaw > 0.0 else -1
            self._pending_lead = None
            self._resume_latched = False
            walker.cancel_lead_morph()
            return
        if policy == "keep" or self._resume_latched or self._yaw_sign == 0:
            return
        if policy in ("L", "R"):
            lead = policy
        elif policy == "outside":
            # +yaw is a left turn. The outside foot is the right foot.
            lead = "R" if self._yaw_sign > 0 else "L"
        else:
            return
        self._pending_lead = lead

    def _ensure_preview(self, walker: op3_walk.Op3Walker) -> zmp_preview.ZmpPreview:
        if self._preview is not None:
            return self._preview
        sole = min(self._sole("L"), self._sole("R"))
        com_z = float(self.data.subtree_com[self.bid_body, 2])
        measured = com_z - sole
        zc = float(self.cfg.preview_zc_m) if self.cfg.preview_zc_m > 0.05 else measured
        if zc < 0.08:
            zc = 0.22
        self._preview_zc = zc
        horizon = max(8, int(round(1.6 / op3_walk.OP3_CTRL_S)))
        self._preview = zmp_preview.ZmpPreview(
            zc, op3_walk.OP3_CTRL_S, horizon, float(self.cfg.preview_r),
        )
        self._preview_clock = -float(self.cfg.preview_arm_s)
        return self._preview

    def _cycle_zmp(self, t_s: float) -> float:
        walker = self.op3
        if walker is None:
            return 0.0
        return zmp_preview.cycle_zmp(
            t_s,
            walker.period,
            walker.l_ssp_start,
            walker.l_ssp_end,
            walker.r_ssp_start,
            walker.r_ssp_end,
            self.cfg.preview_amp_m,
            float(self.cfg.preview_shape),
        )

    def _step_preview(self, preview: zmp_preview.ZmpPreview, future: np.ndarray) -> float:
        self._apply_split_bias(future)
        self._zmp_cmd = float(future[0]) if len(future) else 0.0
        return preview.step(future)

    def _apply_split_bias(self, future: np.ndarray) -> None:
        """Shift the ZMP plan toward the foot the QP is under-loaded on.

        The same shift is stored so the stop's CoM quintic can move with
        the ZMP and the planned lateral acceleration stays put. Each
        sample is then clamped into the inset support of the feet that
        are actually loaded.
        """
        if len(future) == 0:
            self._split_zmp_applied = 0.0
            return
        raw0 = float(future[0])
        bias = float(self._split_zmp_bias)
        for i in range(len(future)):
            future[i] = self._clamp_support_y(float(future[i]) + bias)
        self._split_zmp_applied = float(future[0]) - raw0

    def _inset_y_limits(self, sides: tuple[str, ...]) -> tuple[float, float]:
        """World-y extent of the inset contact boxes on ``sides``."""
        ys: list[float] = []
        inset = float(ds_split.COP_INSET_M)
        for side in sides:
            gid = int(self.gid[side])
            center = np.array(self.data.geom_xpos[gid], dtype=np.float64)
            rot = np.array(self.data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
            half = np.array(self.model.geom_size[gid], dtype=np.float64)
            hx = max(float(half[0]) - inset, 1e-4)
            hy = max(float(half[1]) - inset, 1e-4)
            for sx in (-1.0, 1.0):
                for sy in (-1.0, 1.0):
                    local = np.array([sx * hx, sy * hy, -float(half[2])], dtype=np.float64)
                    ys.append(float((center + rot @ local)[1]))
        if not ys:
            return -0.070, 0.070
        return min(ys), max(ys)

    def _clamp_support_y(self, y: float) -> float:
        """Keep a planned ZMP inside the inset boxes of the loaded feet."""
        loaded = tuple(side for side in ("L", "R") if self.foot_normal(side) >= 1.0)
        sides = loaded if loaded else ("L", "R")
        lo, hi = self._inset_y_limits(sides)
        return min(hi, max(lo, float(y)))

    def _zmp_at(self, t_s: float) -> float:
        if t_s < 0.0:
            return zmp_preview.arm_zmp(t_s, self.cfg.preview_arm_s, self.cfg.preview_amp_m)
        return self._cycle_zmp(t_s)

    def _zmp_future(self, t0_s: float) -> np.ndarray:
        preview = self._preview
        if preview is None:
            raise RuntimeError("preview controller is not built")
        dt = op3_walk.OP3_CTRL_S
        out = np.zeros(preview.horizon, dtype=np.float64)
        for i in range(preview.horizon):
            out[i] = self._zmp_at(t0_s + i * dt)
        return out

    def _slew(self, current: float, target: float, rate: float) -> float:
        dt = op3_walk.OP3_CTRL_S
        step = max(-rate * dt, min(rate * dt, target - current))
        return current + step

    def _dcm_error_y(self) -> float:
        """Measured capture point minus the preview's planned CoM, metres."""
        zc = float(self._preview_zc) if self._preview_zc > 0.05 else 0.18
        omega = math.sqrt(G / max(zc, 1e-3))
        com = float(self.data.subtree_com[self.bid_body, 1])
        vy = float(self.data.subtree_linvel[self.bid_body, 1])
        return (com + vy / omega) - float(self.preview_com_y)

    def _preview_y_with_stab(self, com: float) -> float:
        """Foot-target y. The added term is a rate-limited DCM shift.

        Positive preview_y moves both foot targets toward +y, and the
        hip-roll IK moves the pelvis toward −y. A capture point left of
        the plan therefore adds a positive shift.
        """
        err = self._dcm_error_y()
        self.stab_err_peak = max(getattr(self, "stab_err_peak", 0.0), abs(err))
        # Quiet inside 2 cm. A larger capture-point miss, the kind a mass
        # or friction change leaves, shifts preview_y and therefore hip roll.
        dead = max(0.0, abs(err) - 0.020)
        target = 0.0 if self.preview_stage == "stop" else math.copysign(0.005 * math.tanh(dead / 0.025), err)
        self._stab_preview_m = self._slew(self._stab_preview_m, target, 0.020)
        return -float(com) + self._stab_preview_m

    def _sole_pitch_rad(self, side: Side) -> float:
        """Sole pitch. Positive is toe-down, the same sign as the trunk."""
        bid = int(self.bid[side])
        rot = np.asarray(self.data.xmat[bid], dtype=np.float64).reshape(3, 3)
        return math.atan2(-float(rot[2, 0]), math.hypot(float(rot[0, 0]), float(rot[1, 0])))

    def _swing_toe_bias(self, side: Side) -> float:
        """No toe-up through the swing. The stop blend nulls sole pitch.

        A clocked 0.040 rad toe-up held the lowest sole corner on the
        floor, so the foot slid instead of stepping. The swing ankle
        stays on the IK target. During the stop blend the just-landed
        sole rocks onto its heel. A fixed toe-up makes that worse. This
        asks the ankle to take that pitch back out, then fades as the
        upright stand arrives. +pitch raises the left toe. The right
        ankle is the opposite sign.
        """
        span = self._stop_span if self._stop_span > 1e-6 else 2.0
        if self._stand_q1 is None or self._stand_u <= 0.0 or self._stand_u >= span - 1e-9:
            return 0.0
        u = min(1.0, self._stand_u / span)
        if u < 0.80:
            fade = 1.0
        else:
            v = (u - 0.80) / 0.20
            fade = 1.0 - v * v * v * (v * (v * 6.0 - 15.0) + 10.0)
        sign = 1.0 if side == "L" else -1.0
        # Gain above 1. The blend keeps pitching the sole, so matching
        # the measured pitch once leaves about half of it. 4× leaves a
        # fifth. Cap at 0.12 rad, about 4 Nm if the ankle does not follow.
        pitch = min(0.12, max(-0.12, 4.0 * self._sole_pitch_rad(side)))
        return sign * pitch * fade

    def _accumulate_stab_hip(self) -> None:
        """Rate-limit the capture-point hip-roll offset. Does not write."""
        err = self._dcm_error_y()
        dead = max(0.0, abs(err) - 0.020)
        hip_t = 0.0 if self.preview_stage == "stop" else math.copysign(0.005 * math.tanh(dead / 0.025), err)
        self._stab_hip = self._slew(self._stab_hip, hip_t, 0.15)

    def _stabilize_contacts(self) -> None:
        """Hip roll from the capture point.

        Quiet inside 20 mm, and off during the stop. The swing-foot ankle
        is retargeted in the joint write, so this does not add a second
        ankle command. It does not solve q_des so the signed ask equals
        ±2.33 Nm.
        """
        self._accumulate_stab_hip()
        for name in ("l_hip_roll", "r_hip_roll"):
            idx = self.act_idx.get(name + "_pos")
            if idx is None:
                continue
            if abs(self._stab_hip) > 1e-6:
                self.write_clipped(name, float(self.data.ctrl[idx]) + self._stab_hip)

    def _ff_phase(self, joint: str) -> str:
        """Clock phase of one leg joint. Declared phase is not an input."""
        walker = self.op3
        stage = str(self.preview_stage)
        if stage != "walk" or walker is None or walker.period <= 1e-9:
            return stage or "unset"
        tt = float(walker.time) % float(walker.period)
        windows = (
            ("L", float(walker.l_ssp_start), float(walker.l_ssp_end)),
            ("R", float(walker.r_ssp_start), float(walker.r_ssp_end)),
        )
        swing = None
        frac = float("nan")
        for side, a, b in windows:
            if a < tt <= b and b > a:
                swing = side
                frac = (tt - a) / (b - a)
                break
        if swing is None:
            for _side, _a, b in windows:
                if 0.0 <= (tt - b) % walker.period <= 0.040:
                    return "touchdown impact"
            return "DS transfer"
        own = "L" if joint.startswith("l_") else "R"
        if own == swing:
            if frac <= 0.20:
                return "swing lift ramp"
            if frac >= 0.80:
                return "swing descent"
            return "held swing"
        if 0.20 <= frac <= 0.80:
            return "stance mid"
        return "stance edge"

    def _ankle_cop_mm(self, side: str) -> tuple[float, float, float] | None:
        """Contact CoP relative to the ankle-roll anchor, in that body frame.

        Returns x mm, y mm, and the horizontal distance in mm. None when
        that foot has no contact force.
        """
        gid = int(mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, f"{side}_foot_contact"))
        jid = int(mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, f"{side}_ank_roll"))
        bid = int(mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, f"{side}_ank_roll_link"))
        if gid < 0 or jid < 0 or bid < 0:
            return None
        fn = 0.0
        acc = np.zeros(3)
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            if int(con.geom1) != gid and int(con.geom2) != gid:
                continue
            force = np.zeros(6)
            mj.mj_contactForce(self.model, self.data, i, force)
            fn_i = abs(float(force[0]))
            if fn_i < 1e-4:
                continue
            fn += fn_i
            acc += np.array(con.pos, dtype=np.float64) * fn_i
        if fn < 1e-3:
            return None
        cop = acc / fn
        anchor = np.array(self.data.xanchor[jid], dtype=np.float64)
        rot = np.array(self.data.xmat[bid], dtype=np.float64).reshape(3, 3)
        rel = rot.T @ (cop - anchor)
        return float(rel[0]) * 1000.0, float(rel[1]) * 1000.0, float(math.hypot(rel[0], rel[1])) * 1000.0

    def _id_dominant(
        self,
        scratch: mj.MjData,
        adr: int,
        phase: str,
        cop_h: float | None,
    ) -> str:
        """Largest piece of qfrc_inverse. q̈ includes the velocity product.

        Armature is ID_FF_ARMATURE·q̈ and is not folded into q̈. A contact
        at touchdown is impact. A contact whose CoP is at least 15 mm
        from the ankle is CoP. Any other contact is counted with gravity.
        """
        nv = int(self.model.nv)
        con = np.array(scratch.qfrc_constraint, dtype=np.float64, copy=True)
        pas = np.array(scratch.qfrc_passive, dtype=np.float64, copy=True)
        qacc = np.array(scratch.qacc, dtype=np.float64, copy=True)
        qvel = np.array(scratch.qvel, dtype=np.float64, copy=True)
        inv = float(scratch.qfrc_inverse[adr])
        full = np.zeros(nv)
        bias = np.zeros(nv)
        mj.mj_rne(self.model, scratch, 1, full)
        scratch.qacc[:] = 0.0
        mj.mj_rne(self.model, scratch, 0, bias)
        scratch.qvel[:] = 0.0
        mj.mj_fwdVelocity(self.model, scratch)
        scratch.qacc[:] = 0.0
        grav_v = np.zeros(nv)
        mj.mj_rne(self.model, scratch, 0, grav_v)
        scratch.qvel[:] = qvel
        scratch.qacc[:] = qacc
        inertial = float(full[adr] - bias[adr])
        arm = self.leg_armature * float(qacc[adr])
        qdd = inertial - arm
        cor = float(bias[adr] - grav_v[adr])
        contact = -float(con[adr])
        ident = inertial + float(bias[adr]) - float(pas[adr]) - float(con[adr])
        self._id_split_residual = abs(ident - inv)
        scores = {
            "q̈": abs(qdd) + abs(cor),
            "armature": abs(arm),
            "gravity": abs(float(grav_v[adr])),
        }
        if phase == "touchdown impact":
            scores["impact"] = abs(contact)
        elif cop_h is not None and cop_h >= 15.0:
            scores["CoP"] = abs(contact)
        else:
            scores["gravity"] += abs(contact)
        # A tenth of a newton-metre is the RNE residual seen on this plant.
        # It does not pick the label when one term is clearly larger.
        if self._id_split_residual > 0.30 and self._id_split_residual > 0.15 * max(abs(inv), 1e-6):
            return "unsplit"
        return max(scores, key=lambda name: scores[name])

    def _note_id_peak(self, inv: np.ndarray, names: dict[str, float], hold: bool) -> None:
        """Classify mj_inverse at this step's data.qacc.

        ``hold`` is the unused zero-q̈ record. On the data.qacc path a
        tick with |mj_inverse| > 2.33 Nm is an unsourced-armature
        candidate when |mj_inverse − armature·q̈| is still inside ±2.33.
        Armature is the compiled leg value, 0.01 in the file or the
        load-time override. A wall is only a tick where that bare
        torque is still over 2.33.
        """
        sag = float(KNEE_SAG_NM)
        rot = np.array(self.data.xmat[self.bid_body], dtype=np.float64).reshape(3, 3)
        if float(rot[2, 2]) < 0.92:
            return
        if hold:
            worst_abs = 0.0
            worst_name = ""
            worst_tau = 0.0
            for name, adr in self._leg_dof.items():
                if name not in names:
                    continue
                tau = float(inv[adr])
                if abs(tau) > worst_abs:
                    worst_abs = abs(tau)
                    worst_name = name
                    worst_tau = tau
            if worst_abs > sag + 1e-9:
                self.id_ff_hold_over_n += 1
            if not worst_name or worst_abs <= self.id_ff_hold_abs:
                return
            self.id_ff_hold_abs = worst_abs
            self.id_ff_hold_joint = worst_name
            self.id_ff_hold_phase = self._ff_phase(worst_name)
            self.id_ff_hold_term = ""
            self.id_ff_hold_tau = worst_tau
            self.id_ff_hold_t = float(self.data.time)
            return
        qacc = None
        if self._id_data is not None:
            qacc = np.array(self._id_data.qacc, dtype=np.float64, copy=True)
        wall_abs = -1.0
        wall_name = ""
        wall_tau = 0.0
        wall_arm = 0.0
        wall_stripped = 0.0
        cand_abs = -1.0
        cand_name = ""
        cand_tau = 0.0
        cand_arm = 0.0
        cand_stripped = 0.0
        for name, adr in self._leg_dof.items():
            if name not in names:
                continue
            tau = float(inv[adr])
            arm = 0.0 if qacc is None else self.leg_armature * float(qacc[adr])
            stripped = tau - arm
            if abs(stripped) > sag + 1e-9:
                if abs(stripped) > wall_abs:
                    wall_abs = abs(stripped)
                    wall_name = name
                    wall_tau = tau
                    wall_arm = arm
                    wall_stripped = stripped
            elif abs(tau) > sag + 1e-9 and abs(tau) > cand_abs:
                cand_abs = abs(tau)
                cand_name = name
                cand_tau = tau
                cand_arm = arm
                cand_stripped = stripped
        if wall_name:
            self.id_ff_wall_n += 1
            self.id_ff_over_n += 1
            if self.id_ff_bucket == "wall" and wall_abs <= self.id_ff_stripped_abs:
                return
            phase = self._ff_phase(wall_name)
            term = ""
            if self._id_data is not None:
                side = "l" if wall_name.startswith("l_") else "r"
                other = "r" if side == "l" else "l"
                cop = self._ankle_cop_mm(side)
                if cop is None:
                    cop = self._ankle_cop_mm(other)
                cop_h = None if cop is None else cop[2]
                term = self._id_dominant(
                    self._id_data, self._leg_dof[wall_name], phase, cop_h,
                )
            self.id_ff_bucket = "wall"
            self.id_ff_peak_abs = abs(wall_tau)
            self.id_ff_peak_joint = wall_name
            self.id_ff_peak_phase = phase
            self.id_ff_peak_term = term
            self.id_ff_peak_tau = wall_tau
            self.id_ff_peak_t = float(self.data.time)
            self.id_ff_arm = wall_arm
            self.id_ff_stripped = wall_stripped
            self.id_ff_stripped_abs = wall_abs
            return
        if not cand_name:
            return
        self.id_ff_arm_n += 1
        self.id_ff_over_n += 1
        if self.id_ff_bucket == "wall" or cand_abs <= self.id_ff_peak_abs:
            return
        self.id_ff_bucket = "unsourced-armature candidate"
        self.id_ff_peak_abs = cand_abs
        self.id_ff_peak_joint = cand_name
        self.id_ff_peak_phase = self._ff_phase(cand_name)
        self.id_ff_peak_term = "unsourced-armature candidate"
        self.id_ff_peak_tau = cand_tau
        self.id_ff_peak_t = float(self.data.time)
        self.id_ff_arm = cand_arm
        self.id_ff_stripped = cand_stripped
        self.id_ff_stripped_abs = abs(cand_stripped)

    def _ensure_leg_dof(self) -> None:
        if self._leg_dof:
            return
        for name in (
            "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
            "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
        ):
            jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                continue
            self._leg_dof[name] = int(self.model.jnt_dofadr[jid])
            act = name + "_pos"
            idx = self.act_idx.get(act)
            kv = 0.0
            if idx is not None:
                kv = -float(self.model.actuator_biasprm[idx, 2])
            self._leg_kv[name] = kv

    def begin_id_tick(self) -> None:
        """Clear the per-command inverse record. Physics fills it."""
        self._ensure_leg_dof()
        self.id_tick_tau = {}
        self.id_tick_arm = {}
        self.id_tick_stripped = {}
        self.id_tick_resid = {}
        self.id_tick_resid_pas = {}
        self.id_tick_ok = {name: True for name in self._leg_dof}
        self.id_tick_term = {}
        self.id_tick_root = 0.0
        self.id_tick_fwdinv0 = 0.0
        self.id_tick_fwdinv1 = 0.0
        self.id_tick_offset = {}
        self.id_tick_band = {}
        self._limit_this_tick = False
        self._qp_sample = None

    def finish_id_tick(self) -> None:
        self.ctrl_ticks += 1
        if self._limit_this_tick:
            self.limit_ticks += 1

    def capture_pre_step(self) -> None:
        """Copy the state the live mj_step is about to integrate."""
        self._ensure_leg_dof()
        if self._id_data is None:
            self._id_data = mj.MjData(self.model)
        mj.mj_copyData(self._id_data, self.model, self.data)
        self._id_qvel0 = np.array(self.data.qvel, dtype=np.float64, copy=True)

    def _implicit_leg_force(self, name: str, adr: int, q: float, v0: float, v1: float) -> tuple[float, float]:
        """Force the implicitfast step applies at one leg joint.

        MuJoCo 3.14 implicitfast builds qH = M − dt·qDeriv with
        mjd_smooth_vel(flg_bias=0). qDeriv holds joint damping and the
        actuator velocity gain. A force-clamped actuator is omitted
        from qDeriv (actuatorDerivSkip), so its force stays at the
        value computed from qvel_t. Joint damping is not skipped: it
        acts on qvel_{t+1}. Returns (actuator force, actuator+damping).
        """
        act_name = name + "_pos"
        idx = self.act_idx.get(act_name)
        if idx is None:
            return 0.0, 0.0
        kp = float(self.model.actuator_gainprm[idx, 0])
        kv = float(self._leg_kv.get(name, 0.0))
        ctrl = float(self.data.ctrl[idx])
        lo_c = float(self.model.actuator_ctrlrange[idx, 0])
        hi_c = float(self.model.actuator_ctrlrange[idx, 1])
        if int(self.model.actuator_ctrllimited[idx]):
            ctrl = min(hi_c, max(lo_c, ctrl))
        spring = kp * (ctrl - q)
        raw_t = spring - kv * v0
        lo_f = float(self.model.actuator_forcerange[idx, 0])
        hi_f = float(self.model.actuator_forcerange[idx, 1])
        if int(self.model.actuator_forcelimited[idx]) and (raw_t >= hi_f or raw_t <= lo_f):
            actuator = min(hi_f, max(lo_f, raw_t))
        else:
            actuator = spring - kv * v1
        damp = float(self.model.dof_damping[adr]) * v1
        return actuator, actuator - damp

    def audit_forward_inverse(self) -> None:
        """Inverse of the realised implicitfast step.

        q̈ = (qvel_{t+1} − qvel_t) / dt on the pre-step copy. On that
        copy only, mjENBL_INVDISCRETE makes mj_inverse interpret q̈ as
        the discrete step, so qfrc_inverse matches (M − dt·qDeriv) q̈.
        The leg residual is

            |qfrc_inverse − qfrc_actuator − qfrc_applied|

        with both actuator fields taken from the forward pass on the
        copy. The flags are restored before return. The plant XML is
        not edited. A tick outside 1e-3 Nm is not bucketed. There is
        no wider band. The residual does not relax the 2.33 Nm bar.

        The Δqvel/dt identity (end-step damping, forward constraint)
        stays as id_ident_max. It is a cross-check, not the bucket.
        """
        self._ensure_leg_dof()
        scratch = self._id_data
        qvel0 = self._id_qvel0
        if scratch is None or qvel0 is None:
            return
        dt = float(self.model.opt.timestep)
        qvel1 = np.array(self.data.qvel, dtype=np.float64, copy=True)
        qacc_real = (qvel1 - qvel0) / dt
        saved = int(self.model.opt.enableflags)
        discrete_bit = getattr(mj.mjtEnableBit, "mjENBL_INVDISCRETE", None)
        try:
            mj.mj_forward(self.model, scratch)
            con = np.array(scratch.qfrc_constraint, dtype=np.float64, copy=True)
            app = np.array(scratch.qfrc_applied, dtype=np.float64, copy=True)
            act_fwd = np.array(scratch.qfrc_actuator, dtype=np.float64, copy=True)
            bias = np.zeros(int(self.model.nv), dtype=np.float64)
            mj.mj_rne(self.model, scratch, 0, bias)
            ma = np.zeros(int(self.model.nv), dtype=np.float64)
            mj.mj_mulM(self.model, scratch, ma, qacc_real)
            dyn = ma + bias
            scratch.qacc[:] = qacc_real
            flags = saved | int(mj.mjtEnableBit.mjENBL_FWDINV)
            if discrete_bit is not None:
                flags |= int(discrete_bit)
            self.model.opt.enableflags = flags
            if int(scratch.nefc) > 0:
                mj.mj_compareFwdInv(self.model, scratch)
            else:
                mj.mj_inverse(self.model, scratch)
            fwd0 = float(scratch.solver_fwdinv[0])
            fwd1 = float(scratch.solver_fwdinv[1])
            inv = np.array(scratch.qfrc_inverse, dtype=np.float64, copy=True)
        finally:
            self.model.opt.enableflags = saved
        qacc = qacc_real
        self.id_phys_n += 1
        if fwd0 > self.id_tick_fwdinv0:
            self.id_tick_fwdinv0 = fwd0
        if fwd1 > self.id_tick_fwdinv1:
            self.id_tick_fwdinv1 = fwd1
        if fwd0 > self.id_fwdinv0_max:
            self.id_fwdinv0_max = fwd0
        if fwd1 > self.id_fwdinv1_max:
            self.id_fwdinv1_max = fwd1
            self.id_fwdinv_t = float(self.data.time)
        root_abs = np.abs(inv[:6])
        root = float(np.max(root_abs)) if root_abs.size else 0.0
        if root > self.id_tick_root:
            self.id_tick_root = root
        if root > self.id_root_max:
            self.id_root_max = root
            self.id_root_t = float(self.data.time)
            self.id_root_dof = int(np.argmax(root_abs))
        bal = np.abs(dyn[:6] - con[:6] - app[:6])
        bal_max = float(np.max(bal)) if bal.size else 0.0
        if bal_max > self.id_root_bal_max:
            self.id_root_bal_max = bal_max
        if root > 0.05:
            self.id_root_fail_n += 1
        step_fail = False
        step_over = False
        step_exact = False
        rot = np.array(self.data.xmat[self.bid_body], dtype=np.float64).reshape(3, 3)
        upright = float(rot[2, 2]) >= 0.92
        sag = float(KNEE_SAG_NM)
        terms = self._leg_terms(scratch, qacc, con)
        for name, adr in self._leg_dof.items():
            mj_tau = float(inv[adr])
            arm = self.leg_armature * float(qacc[adr])
            jid = int(mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name))
            q = float(scratch.qpos[int(self.model.jnt_qposadr[jid])])
            actuator, applied = self._implicit_leg_force(
                name, adr, q, float(qvel0[adr]), float(qvel1[adr]),
            )
            # Cross-check: the end-step identity on Δqvel/dt. Not the bucket.
            ident = abs(float(dyn[adr]) - float(con[adr]) - float(app[adr]) - applied)
            if ident > self.id_ident_max:
                self.id_ident_max = ident
            # Inverse column is qfrc_inverse[dof]. mjENBL_INVDISCRETE
            # already undoes M → M − dt·D. Without that flag, subtract
            # the signed dt·(dof damping + kv)·q̈ on this joint.
            if discrete_bit is None:
                kv = float(self._leg_kv.get(name, 0.0))
                damp = float(self.model.dof_damping[adr])
                mj_tau = mj_tau - dt * (damp + kv) * float(qacc[adr])
            tau = mj_tau
            gap = abs(tau - float(act_fwd[adr]) - float(app[adr]))
            if gap > self.id_inv_gap_max:
                self.id_inv_gap_max = gap
                self.id_inv_gap_joint = name
            stripped = tau - arm
            resid = gap
            prev_resid = float(self.id_tick_resid.get(name, -1.0))
            if resid > prev_resid:
                self.id_tick_resid[name] = resid
            # 1e-3 is the missed target. 1e-2 is the bucket gate.
            if resid > ID_RESID_EXACT_NM:
                step_exact = True
            outside = resid > ID_RESID_BUCKET_NM
            if outside:
                step_over = True
                self.id_tick_ok[name] = False
                step_fail = True
            elif abs(tau) >= abs(float(self.id_tick_tau.get(name, 0.0))):
                self.id_tick_tau[name] = tau
                self.id_tick_arm[name] = arm
                self.id_tick_stripped[name] = stripped
                self.id_tick_term[name] = terms.get(name, "")
            if abs(mj_tau) > self.id_mj_abs:
                self.id_mj_abs = abs(mj_tau)
                self.id_mj_tau = mj_tau
                self.id_mj_joint = name
                self.id_mj_t = float(self.data.time)
            if abs(abs(actuator) - 2.45) <= 1.0e-6:
                self.id_rail_n += 1
                if len(self.id_rail_hits) < 64:
                    self.id_rail_hits.append((
                        float(self.data.time), name, actuator, mj_tau,
                    ))
            passed = not outside
            if passed:
                self._consistent_tau[name] = tau
                if name.endswith("knee"):
                    self.id_knee_ok_n += 1
                    if abs(tau) > self.id_knee_ok_abs:
                        self.id_knee_ok_abs = abs(tau)
                        self.id_knee_ok_tau = tau
                        self.id_knee_ok_joint = name
                        self.id_knee_ok_t = float(self.data.time)
                        self.id_knee_ok_phase = self._ff_phase(name)
                        self.id_knee_act = actuator
                    if upright and abs(tau) > self.id_knee_up_abs:
                        self.id_knee_up_abs = abs(tau)
                        self.id_knee_up_tau = tau
                        self.id_knee_up_joint = name
                        self.id_knee_up_t = float(self.data.time)
                        self.id_knee_up_phase = self._ff_phase(name)
                        self.id_knee_up_act = actuator
                        self.id_knee_up_resid = resid
                if name == "r_knee":
                    hold_dt = abs(float(self.data.time) - 2.680)
                    if hold_dt < self.id_hold_dt:
                        self.id_hold_dt = hold_dt
                        self.id_hold_t = float(self.data.time)
                        self.id_hold_tau = tau
                        self.id_hold_act = actuator
                        self.id_hold_phase = self._ff_phase(name)
                        self.id_hold_up = upright
            if resid > self.id_resid_max:
                self.id_resid_max = resid
                self.id_resid_joint = name
            if outside or not upright:
                continue
            # A realised inverse on a clamped actuator returns the clamp.
            # That tick cannot separate the controller from the physics.
            # The wall test is the planned trajectory, below.
            if abs(stripped) > sag + 1e-9:
                self.id_wall_phys_n += 1
                if abs(stripped) <= self.id_ff_stripped_abs and self.id_ff_bucket == "wall":
                    continue
                phase = self._ff_phase(name)
                term = terms.get(name, "")
                self.id_ff_bucket = "wall"
                self.id_ff_peak_abs = abs(tau)
                self.id_ff_peak_joint = name
                self.id_ff_peak_phase = phase
                self.id_ff_peak_term = term
                self.id_ff_peak_tau = tau
                self.id_ff_peak_t = float(self.data.time)
                self.id_ff_arm = arm
                self.id_ff_stripped = stripped
                self.id_ff_stripped_abs = abs(stripped)
                self.id_ff_resid = resid
            elif abs(tau) > sag + 1e-9:
                self.id_arm_phys_n += 1
                if self.id_ff_bucket == "wall" or abs(tau) <= self.id_ff_peak_abs:
                    continue
                self.id_ff_bucket = "unsourced-armature candidate"
                self.id_ff_peak_abs = abs(tau)
                self.id_ff_peak_joint = name
                self.id_ff_peak_phase = self._ff_phase(name)
                self.id_ff_peak_term = "unsourced-armature candidate"
                self.id_ff_peak_tau = tau
                self.id_ff_peak_t = float(self.data.time)
                self.id_ff_arm = arm
                self.id_ff_stripped = stripped
                self.id_ff_stripped_abs = abs(stripped)
        if step_exact:
            self.id_resid_exact_n += 1
        if step_over:
            self.id_resid_over_n += 1
        if step_fail:
            self.id_resid_fail_n += 1

    def _leg_terms(self, scratch: mj.MjData, qacc: np.ndarray, con: np.ndarray) -> dict[str, str]:
        """Name the largest piece of the exact step force at each leg joint.

        q̈ is the inertia excluding armature, plus the velocity product.
        Armature is the compiled leg value times q̈. Contact is the
        forward constraint. The label does not use qfrc_inverse, which
        re-solves that constraint.
        """
        nv = int(self.model.nv)
        qvel = np.array(scratch.qvel, dtype=np.float64, copy=True)
        qacc_saved = np.array(scratch.qacc, dtype=np.float64, copy=True)
        full = np.zeros(nv, dtype=np.float64)
        bias = np.zeros(nv, dtype=np.float64)
        mj.mj_rne(self.model, scratch, 1, full)
        scratch.qacc[:] = 0.0
        mj.mj_rne(self.model, scratch, 0, bias)
        scratch.qvel[:] = 0.0
        mj.mj_fwdVelocity(self.model, scratch)
        grav = np.zeros(nv, dtype=np.float64)
        scratch.qacc[:] = 0.0
        mj.mj_rne(self.model, scratch, 0, grav)
        scratch.qvel[:] = qvel
        scratch.qacc[:] = qacc_saved
        terms: dict[str, str] = {}
        for name, adr in self._leg_dof.items():
            inertial = float(full[adr] - bias[adr])
            arm = abs(self.leg_armature * float(qacc[adr]))
            qdd = abs(inertial - self.leg_armature * float(qacc[adr]))
            cor = abs(float(bias[adr] - grav[adr]))
            contact = abs(float(con[adr]))
            scores = {
                "q̈": qdd + cor,
                "armature": arm,
                "gravity": abs(float(grav[adr])),
                "contact": contact,
            }
            label = max(scores, key=lambda key: scores[key])
            if label == "contact":
                phase = self._ff_phase(name)
                cop = self._ankle_cop_mm("l" if name.startswith("l_") else "r")
                if phase == "touchdown impact":
                    label = "impact"
                elif cop is not None and cop[2] >= 15.0:
                    label = "CoP"
            terms[name] = label
        return terms

    def _id_feedforward(self, joints: dict[str, float]) -> None:
        """ctrl from the planned gait, not from the realised inverse.

        τ_ff = ID(q_ref, q̇_ref, q̈_ref) with the LIPM wrench inside the
        declared foot box. Then

            ctrl = q + (τ_ff + kv·q̇ + K_fb·(q_ref−q)) / kp

        q̇ is the measured joint velocity. K_fb is ID_FF_KP. The sum is
        not saturated and not clipped into ±2.33 Nm. ctrl is not pulled
        into ctrlrange here. A later limiter that changes it is a fail.
        """
        if not self.cfg.gm_id_ff:
            return
        self._ensure_leg_dof()
        tau_ff = self._planned_tau(joints)
        for name, adr in self._leg_dof.items():
            if name not in joints:
                continue
            act = name + "_pos"
            idx = self.act_idx.get(act)
            if idx is None:
                continue
            kp = float(self.model.actuator_gainprm[idx, 0])
            kv = -float(self.model.actuator_biasprm[idx, 2])
            if kp < 1e-6:
                self.id_ff_impossible += 1
                continue
            q = self.q(name)
            omega = float(self.data.qvel[adr])
            q_ref = float(joints[name])
            fb = ID_FF_KP * (q_ref - q)
            tau = float(tau_ff.get(name, 0.0)) + fb
            if abs(tau) > self.id_plan_ask:
                self.id_plan_ask = abs(tau)
                self.id_plan_tau = float(tau_ff.get(name, 0.0))
                self.id_plan_abs = abs(self.id_plan_tau)
                self.id_plan_joint = name
                self.id_plan_phase = self._ff_phase(name)
                self.id_plan_term = self._plan_term_name
                self.id_plan_t = float(self.data.time)
            joints[name] = q + (tau + kv * omega) / kp

    def _ref_derivatives(self, joints: dict[str, float]) -> tuple[dict[str, float], dict[str, float]]:
        """Central difference of the planned joint targets. dt is the 8 ms plan."""
        sample = {name: float(val) for name, val in joints.items()}
        self._plan_q_hist.append(sample)
        if len(self._plan_q_hist) > 3:
            self._plan_q_hist.pop(0)
        dt = float(op3_walk.OP3_CTRL_S)
        qd: dict[str, float] = {}
        qdd: dict[str, float] = {}
        hist = self._plan_q_hist
        for name in sample:
            if len(hist) >= 3 and name in hist[-2] and name in hist[-3]:
                q0 = float(hist[-3][name])
                q1 = float(hist[-2][name])
                q2 = float(sample[name])
                qd[name] = (q2 - q0) / (2.0 * dt)
                qdd[name] = (q2 - 2.0 * q1 + q0) / (dt * dt)
            elif len(hist) >= 2 and name in hist[-2]:
                qd[name] = (float(sample[name]) - float(hist[-2][name])) / dt
                qdd[name] = 0.0
            else:
                qd[name] = 0.0
                qdd[name] = 0.0
        return qd, qdd

    def _planned_tau(self, joints: dict[str, float]) -> dict[str, float]:
        """Inverse of the planned pose, with the LIPM wrench and no contacts.

        Contacts are disabled on the shared model only for this call and
        restored before return. The wrench is written on the scratch
        data, not on the live step. The plant XML is not edited.
        """
        self._plan_term_name = ""
        self._plan_term_phase = ""
        self._qp_sample = None
        qd, qdd = self._ref_derivatives(joints)
        if self._plan_data is None:
            self._plan_data = mj.MjData(self.model)
        scratch = self._plan_data
        mj.mj_resetData(self.model, scratch)
        scratch.qpos[:] = self.data.qpos
        scratch.qvel[:] = 0.0
        for name, val in joints.items():
            jid = int(mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name))
            if jid < 0:
                continue
            qadr = int(self.model.jnt_qposadr[jid])
            vadr = int(self.model.jnt_dofadr[jid])
            scratch.qpos[qadr] = float(val)
            scratch.qvel[vadr] = float(qd.get(name, 0.0))
            scratch.qacc[vadr] = float(qdd.get(name, 0.0))
        zc = float(self._preview_zc) if self._preview_zc > 0.05 else 0.18
        zmp_y = float(self._zmp_cmd)
        com_y = float(self.preview_com_y)
        ay = (com_y - zmp_y) * G / zc
        vy = 0.0
        if self._preview is not None:
            vy = float(self._preview.com_vel_m_s)
        free = self._free_dof()
        scratch.qvel[free + 0] = float(self.cmd_vx)
        scratch.qvel[free + 1] = vy
        scratch.qacc[free + 1] = ay
        saved_aff = np.array(self.model.geom_conaffinity, copy=True)
        saved_typ = np.array(self.model.geom_contype, copy=True)
        self.model.geom_conaffinity[:] = 0
        self.model.geom_contype[:] = 0
        self._ds_count_wall = True
        try:
            mj.mj_forward(self.model, scratch)
            for name, val in qdd.items():
                jid = int(mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name))
                if jid < 0:
                    continue
                vadr = int(self.model.jnt_dofadr[jid])
                scratch.qacc[vadr] = float(val)
            scratch.qacc[free + 0] = 0.0
            scratch.qacc[free + 1] = ay
            scratch.qacc[free + 2] = 0.0
            scratch.qacc[free + 3] = 0.0
            scratch.qacc[free + 4] = 0.0
            scratch.qacc[free + 5] = 0.0
            scratch.qfrc_applied[:] = 0.0
            scratch.xfrc_applied[:] = 0.0
            mj.mj_inverse(self.model, scratch)
            inv = np.array(scratch.qfrc_inverse, dtype=np.float64, copy=True)
            # Double support does not fix the foot-to-foot share. Split (c)
            # is the feedforward when it solves. One airborne foot is the
            # stance box, the same as single support.
            if self._planned_stance() is None and not self._return_hold:
                self._ds_apply_split(scratch, zmp_y, inv, qdd)
            else:
                forced = None
                if self._return_hold:
                    loaded = self._loaded_box_zmp()
                    if loaded is not None:
                        forced = "L" if loaded > 0.0 else "R"
                self._apply_planned_wrench(scratch, zmp_y, forced)
            applied = np.array(scratch.qfrc_applied, dtype=np.float64, copy=True)
        finally:
            self.model.geom_conaffinity[:] = saved_aff
            self.model.geom_contype[:] = saved_typ
        out: dict[str, float] = {}
        worst = 0.0
        worst_name = ""
        phase_of: dict[str, str] = {}
        for name, adr in self._leg_dof.items():
            # mj_inverse does not subtract qfrc_applied. With contacts
            # off, the actuator that realises the plan is inverse − wrench.
            # tau includes the compiled leg armature times q̈_ref.
            tau = float(inv[adr] - applied[adr])
            out[name] = tau
            qdd_i = float(qdd.get(name, 0.0))
            bare = tau - self.leg_armature * qdd_i
            phase = self._ff_phase(name)
            phase_of[name] = phase
            self._note_planned_req(
                name, tau, bare, qdd_i, phase, count_wall=self._ds_count_wall,
            )
            if abs(tau) > worst:
                worst = abs(tau)
                worst_name = name
        if worst_name:
            self._plan_term_name = self._planned_term(scratch, self._leg_dof[worst_name])
            self._plan_term_phase = phase_of.get(worst_name, "")
        return out

    def _note_planned_req(
        self, name: str, tau: float, bare: float, qdd: float, phase: str,
        count_wall: bool = True,
    ) -> None:
        """Per-joint peaks of the planned inverse, with and without armature.

        ``tau`` already includes the compiled leg armature times q̈_ref.
        ``bare`` is tau minus that term.
        On double support this tau is split (c), the min-max share. A wall
        is |bare| > 2.33 on that share, or on the single-support wrench.
        Splits (a) and (b) do not name a wall. |tau| > 2.33 with |bare| ≤
        2.33 is an unsourced-armature candidate. A pass is the applied
        signed force on the real plant, armature included, ≤ 2.33 on every
        tick, with limiter-active fraction 0 and clamp fraction 0.
        """
        t = float(self.data.time)
        sample = {
            "abs": abs(tau),
            "tau": float(tau),
            "bare": float(bare),
            "qdd": float(qdd),
            "t": t,
            "phase": phase,
        }
        with_rec = self.id_req_with.get(name)
        if with_rec is None or abs(tau) > float(with_rec["abs"]):
            self.id_req_with[name] = dict(sample)
        bare_rec = self.id_req_bare.get(name)
        if bare_rec is None or abs(bare) > float(bare_rec["abs"]):
            bare_sample = dict(sample)
            bare_sample["abs"] = abs(bare)
            self.id_req_bare[name] = bare_sample
        if phase == "stop":
            stop_with = self.id_req_stop_with.get(name)
            if stop_with is None or abs(tau) > float(stop_with["abs"]):
                self.id_req_stop_with[name] = dict(sample)
            stop_bare = self.id_req_stop_bare.get(name)
            if stop_bare is None or abs(bare) > float(stop_bare["abs"]):
                stop_sample = dict(sample)
                stop_sample["abs"] = abs(bare)
                self.id_req_stop_bare[name] = stop_sample
        if abs(tau) > KNEE_SAG_NM + 1e-9 and abs(bare) <= KNEE_SAG_NM + 1e-9:
            self.id_req_arm = True
            if abs(tau) > abs(self.id_req_arm_tau):
                self.id_req_arm_joint = name
                self.id_req_arm_tau = float(tau)
                self.id_req_arm_bare = float(bare)
                self.id_req_arm_phase = phase
                self.id_req_arm_t = t
        if count_wall and abs(bare) > KNEE_SAG_NM + 1e-9:
            self.id_req_wall = True
            self.id_req_wall_n += 1
            if abs(bare) > abs(self.id_req_wall_bare):
                self.id_req_wall_joint = name
                self.id_req_wall_tau = float(tau)
                self.id_req_wall_bare = float(bare)
                self.id_req_wall_phase = phase
                self.id_req_wall_t = t
        if name.endswith("knee") and abs(tau) >= self.id_req_knee_abs:
            self.id_req_knee_abs = abs(tau)
            self.id_req_knee_tau = float(tau)
            self.id_req_knee_joint = name
            self.id_req_knee_phase = phase
            self.id_req_knee_t = t
            self.id_req_knee_qdd = float(qdd)
            self.id_req_knee_bare = float(bare)

    def _free_dof(self) -> int:
        for jid in range(int(self.model.njnt)):
            if int(self.model.jnt_type[jid]) == int(mj.mjtJoint.mjJNT_FREE):
                return int(self.model.jnt_dofadr[jid])
        return 0

    def _ds_apply_split(
        self,
        scratch: mj.MjData,
        zmp_y: float,
        inv: np.ndarray,
        qdd: dict[str, float],
    ) -> None:
        """Write the double-support wrench. Feedforward is split (c) when it solves.

        (a) is the line split on the full declared box. (b) is minimum-norm
        ankle torque and (c) minimises the maximum |bare leg torque|. (b)
        and (c) keep each CoP inside the declared box shrunk by 5 mm on
        every side. All three sum to the same planned wrench. (c) is the
        one that can name a wall.
        """
        names = list(self._leg_dof)
        dofs = [int(self._leg_dof[name]) for name in names]
        free = self._free_dof()
        free_dofs = list(range(free, free + 6))
        mass = float(self.model.body_subtreemass[self.bid_body])
        if mass < 1e-6:
            mass = float(np.sum(self.model.body_mass))
        zc = float(self._preview_zc) if self._preview_zc > 0.05 else 0.18
        ay = (float(self.preview_com_y) - float(zmp_y)) * G / zc
        force = np.array([0.0, mass * ay, mass * G], dtype=np.float64)
        com = np.array(scratch.subtree_com[self.bid_body], dtype=np.float64)
        feet = []
        for side in ("L", "R"):
            center, rot, half = self._sole_frame(scratch, side)
            bottom = center + rot @ np.array([0.0, 0.0, -float(half[2])], dtype=np.float64)
            body = int(self.model.geom_bodyid[self.gid[side]])
            feet.append((bottom, rot, (float(half[0]), float(half[1])), body))
        p_zmp = np.array(
            [float(com[0]), float(zmp_y), 0.5 * (float(feet[0][0][2]) + float(feet[1][0][2]))],
            dtype=np.float64,
        )

        def probe(point: np.ndarray, body: int) -> np.ndarray:
            cols = []
            for axis in range(6):
                scratch.qfrc_applied[:] = 0.0
                f = np.zeros(3, dtype=np.float64)
                tq = np.zeros(3, dtype=np.float64)
                if axis < 3:
                    f[axis] = 1.0
                else:
                    tq[axis - 3] = 1.0
                mj.mj_applyFT(self.model, scratch, f, tq, point, body, scratch.qfrc_applied)
                cols.append(np.array(scratch.qfrc_applied, dtype=np.float64, copy=True))
            return np.column_stack(cols)

        maps = [probe(feet[i][0], feet[i][3]) for i in (0, 1)]
        full = [(float(feet[i][2][0]), float(feet[i][2][1])) for i in (0, 1)]
        inner = [ds_split.inset_half(hx, hy) for hx, hy in full]
        blocks = [
            ds_split.corner_wrench_map(feet[i][1], inner[i][0], inner[i][1]) for i in (0, 1)
        ]
        J = np.concatenate(
            [maps[i][np.ix_(dofs, range(6))] @ blocks[i] for i in (0, 1)], axis=1,
        )
        geq = np.concatenate(
            [maps[i][np.ix_(free_dofs, range(6))] @ blocks[i] for i in (0, 1)], axis=1,
        )
        scratch.qfrc_applied[:] = 0.0
        mj.mj_applyFT(
            self.model, scratch, force, np.zeros(3), p_zmp, int(self.bid_body), scratch.qfrc_applied,
        )
        target = np.array(scratch.qfrc_applied[free_dofs], dtype=np.float64, copy=True)
        bare0 = np.array(
            [
                float(inv[self._leg_dof[name]]) - self.leg_armature * float(qdd.get(name, 0.0))
                for name in names
            ],
            dtype=np.float64,
        )
        tau0 = np.array([float(inv[self._leg_dof[name]]) for name in names], dtype=np.float64)
        linear = ds_split.linear_split(
            [feet[0][0], feet[1][0]],
            [feet[0][1], feet[1][1]],
            [feet[0][2], feet[1][2]],
            p_zmp,
            force,
        )
        qfrc_a = (
            maps[0][np.ix_(dofs, range(6))] @ linear["wrenches"][0]
            + maps[1][np.ix_(dofs, range(6))] @ linear["wrenches"][1]
        )
        bare_a = bare0 - qfrc_a
        self.id_split_a_resid_m = max(
            self.id_split_a_resid_m, float(linear["residual_m"]),
        )
        x0 = ds_split.barycentric_forces(linear, [feet[0][1], feet[1][1]], inner)
        x_c, _t_c = ds_split.solve_minimax(J, bare0, geq, target, x0)
        ankle = [i for i, name in enumerate(names) if name.endswith(("ank_pitch", "ank_roll"))]
        x_b = ds_split.solve_ankle_norm(J[ankle, :], bare0[ankle], geq, target, x0)
        if x_b is not None and (
            float(np.linalg.norm(geq @ x_b - target)) > 1e-3
            or bool(np.any(ds_split._A_UB @ x_b > 1e-4))
        ):
            x_b = None
        if x_c is not None and float(np.linalg.norm(geq @ x_c - target)) > 1e-3:
            x_c = None
        bares = {"a": bare_a, "b": None, "c": None}
        taus = {"a": tau0 - qfrc_a, "b": None, "c": None}
        chosen = None
        if x_b is not None:
            bares["b"] = bare0 - J @ x_b
            taus["b"] = tau0 - J @ x_b
        if x_c is not None:
            bares["c"] = bare0 - J @ x_c
            taus["c"] = tau0 - J @ x_c
            chosen = x_c
            self._ds_count_wall = True
            self._note_qp_cop(x_c, inner, full, float(linear["alpha"]))
        else:
            self.id_split_infeas += 1
            self._ds_count_wall = False
        self._note_ds_spread(names, bares, taus, qdd)
        scratch.qfrc_applied[:] = 0.0
        scratch.xfrc_applied[:] = 0.0
        if chosen is None:
            for wrench, foot in zip(linear["wrenches"], feet):
                mj.mj_applyFT(
                    self.model, scratch, wrench[:3], wrench[3:], foot[0], foot[3], scratch.qfrc_applied,
                )
            return
        for foot_i, foot in enumerate(feet):
            rot = np.asarray(foot[1], dtype=np.float64).reshape(3, 3)
            for k, (sx, sy) in enumerate(((1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0))):
                f_s = chosen[12 * foot_i + 3 * k : 12 * foot_i + 3 * k + 3]
                if float(np.linalg.norm(f_s)) < 1e-10:
                    continue
                r = np.array(
                    [sx * inner[foot_i][0], sy * inner[foot_i][1], 0.0], dtype=np.float64,
                )
                mj.mj_applyFT(
                    self.model, scratch, rot @ f_s, rot @ np.cross(r, f_s),
                    foot[0], foot[3], scratch.qfrc_applied,
                )

    def _note_qp_cop(
        self,
        x: np.ndarray,
        inner: list[tuple[float, float]],
        full: list[tuple[float, float]],
        alpha_line: float,
    ) -> None:
        """QP CoP margin to the declared box, and the sample observe() compares."""
        cops: list[np.ndarray] = []
        fzs: list[float] = []
        for i, side in enumerate(("L", "R")):
            cop, fz = ds_split.foot_cop(x[12 * i : 12 * i + 12], inner[i][0], inner[i][1])
            cops.append(cop)
            fzs.append(float(fz))
            if fz <= 1.0:
                continue
            margin = ds_split.declared_margin(cop, full[i][0], full[i][1])
            self.id_qp_margin_n[side] += 1
            self.id_qp_margin_sum[side] += margin
            if margin < self.id_qp_margin_min[side]:
                self.id_qp_margin_min[side] = float(margin)
                self.id_qp_margin_min_t[side] = float(self.data.time)
        total = fzs[0] + fzs[1]
        alpha = fzs[0] / total if total > 1e-6 else 0.5
        gap_line = abs(alpha - float(alpha_line))
        self.id_qp_line_n += 1
        self.id_qp_line_sum += gap_line
        if gap_line > self.id_qp_line_max:
            self.id_qp_line_max = gap_line
        self._qp_sample = {"alpha": float(alpha), "fz": fzs, "cop": cops}

    def _note_ds_spread(
        self,
        names: list[str],
        bares: dict[str, np.ndarray | None],
        taus: dict[str, np.ndarray | None],
        qdd: dict[str, float],
    ) -> None:
        """Record the same-tick spread of the three double-support shares."""
        phase = self._ff_phase(names[0])
        t = float(self.data.time)
        if bares.get("c") is not None:
            self.id_split_ds_n += 1
            if phase == "stop":
                self.id_split_stop_n += 1
        elif phase == "stop":
            self.id_split_stop_infeas += 1
        present = [key for key in ("a", "b", "c") if bares.get(key) is not None]
        for key in present:
            bare = bares[key]
            tau = taus[key]
            assert bare is not None and tau is not None
            for i, name in enumerate(names):
                self._note_split_peak(
                    key, name, float(tau[i]), float(bare[i]), float(qdd.get(name, 0.0)), phase, t,
                    stop=False,
                )
                if phase == "stop":
                    self._note_split_peak(
                        key, name, float(tau[i]), float(bare[i]), float(qdd.get(name, 0.0)),
                        phase, t, stop=True,
                    )
            peak = float(np.max(np.abs(bare)))
            if peak > getattr(self, f"id_split_{key}_max"):
                setattr(self, f"id_split_{key}_max", peak)
                joint = names[int(np.argmax(np.abs(bare)))]
                setattr(self, f"id_split_{key}_joint", joint)
        peaks = []
        for key in present:
            bare = bares[key]
            assert bare is not None
            peaks.append(float(np.max(np.abs(bare))))
        if len(peaks) >= 2:
            spread = max(peaks) - min(peaks)
            if spread > self.id_split_peak_spread:
                self.id_split_peak_spread = spread
                self.id_split_peak_phase = phase
                self.id_split_peak_t = t
                self.id_split_peak_a = float(np.max(np.abs(bares["a"]))) if bares.get("a") is not None else 0.0
                self.id_split_peak_b = float(np.max(np.abs(bares["b"]))) if bares.get("b") is not None else 0.0
                self.id_split_peak_c = (
                    float(np.max(np.abs(bares["c"]))) if bares.get("c") is not None else 0.0
                )
        for i, name in enumerate(names):
            if not name.endswith("knee"):
                continue
            vals = []
            for key in ("a", "b", "c"):
                bare = bares.get(key)
                if bare is None:
                    continue
                vals.append(float(bare[i]))
            if len(vals) < 2:
                continue
            spread = max(vals) - min(vals)
            if spread > self.id_split_knee_spread:
                self.id_split_knee_spread = spread
                self.id_split_knee_joint = name
                self.id_split_knee_phase = self._ff_phase(name)
                self.id_split_knee_t = t
                self.id_split_knee_a = float(bares["a"][i]) if bares.get("a") is not None else 0.0
                self.id_split_knee_b = float(bares["b"][i]) if bares.get("b") is not None else 0.0
                self.id_split_knee_c = float(bares["c"][i]) if bares.get("c") is not None else 0.0
            if phase == "stop" and spread > self.id_split_stop_knee_spread:
                self.id_split_stop_knee_spread = spread
                self.id_split_stop_knee_joint = name
                self.id_split_stop_knee_t = t
                self.id_split_stop_knee_a = float(bares["a"][i]) if bares.get("a") is not None else 0.0
                self.id_split_stop_knee_b = float(bares["b"][i]) if bares.get("b") is not None else 0.0
                self.id_split_stop_knee_c = float(bares["c"][i]) if bares.get("c") is not None else 0.0

    def _note_split_peak(
        self, key: str, name: str, tau: float, bare: float, qdd: float, phase: str,
        t: float, stop: bool,
    ) -> None:
        store = self.id_split_stop_bare if stop else self.id_split_bare
        rec = store[key].get(name)
        if rec is not None and abs(bare) <= float(rec["abs"]):
            return
        store[key][name] = {
            "abs": abs(bare),
            "tau": float(tau),
            "bare": float(bare),
            "qdd": float(qdd),
            "t": float(t),
            "phase": phase,
        }

    def _apply_planned_wrench(
        self, scratch: mj.MjData, zmp_y: float, stance: str | None = None,
    ) -> None:
        """LIPM wrench on one foot. Double support uses ``_ds_apply_split``."""
        mass = float(self.model.body_subtreemass[self.bid_body])
        if mass < 1e-6:
            mass = float(np.sum(self.model.body_mass))
        zc = float(self._preview_zc) if self._preview_zc > 0.05 else 0.18
        ay = (float(self.preview_com_y) - float(zmp_y)) * G / zc
        force = np.array([0.0, mass * ay, mass * G], dtype=np.float64)
        torque = np.zeros(3, dtype=np.float64)
        scratch.qfrc_applied[:] = 0.0
        scratch.xfrc_applied[:] = 0.0
        if stance is None:
            stance = self._planned_stance()
        if stance in ("L", "R"):
            point = self._project_zmp(scratch, stance, zmp_y)
            body = int(self.model.geom_bodyid[self.gid[stance]])
            mj.mj_applyFT(self.model, scratch, force, torque, point, body, scratch.qfrc_applied)
            return
        left = self._sole_frame(scratch, "L")
        right = self._sole_frame(scratch, "R")
        y_l = float(left[0][1])
        y_r = float(right[0][1])
        span = y_l - y_r
        if abs(span) < 1e-6:
            alpha = 0.5
        else:
            alpha = (float(zmp_y) - y_r) / span
            alpha = min(1.0, max(0.0, alpha))
        # Lateral split uses the box centres, so the net CoP y is the ZMP.
        # Fore-aft, each share sits on the LIPM ZMP x (ax = 0, so that is
        # the planned CoM x) clamped inside that box. Both shares at the
        # geometric centre leave the pitch moment of mg times the CoM offset.
        com_x = float(scratch.subtree_com[self.bid_body][0])
        for side, share in (("L", alpha), ("R", 1.0 - alpha)):
            if share <= 1e-8:
                continue
            center, rot, half = self._sole_frame(scratch, side)
            world = np.array([com_x, float(center[1]), float(center[2])], dtype=np.float64)
            local = rot.T @ (world - center)
            local[0] = min(float(half[0]), max(-float(half[0]), float(local[0])))
            local[1] = 0.0
            local[2] = -float(half[2])
            point = center + rot @ local
            body = int(self.model.geom_bodyid[self.gid[side]])
            mj.mj_applyFT(
                self.model, scratch, force * share, torque, point, body, scratch.qfrc_applied,
            )

    def _planned_stance(self) -> str | None:
        if self.phase == "swing" and self.stance in ("L", "R"):
            return self.stance
        return None

    def _sole_frame(self, scratch: mj.MjData, side: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        gid = int(self.gid[side])
        center = np.array(scratch.geom_xpos[gid], dtype=np.float64)
        rot = np.array(scratch.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
        half = np.array(self.model.geom_size[gid], dtype=np.float64)
        return center, rot, half

    def _project_zmp(self, scratch: mj.MjData, side: str, zmp_y: float) -> np.ndarray:
        center, rot, half = self._sole_frame(scratch, side)
        com = np.array(scratch.subtree_com[self.bid_body], dtype=np.float64)
        world = np.array([float(com[0]), float(zmp_y), float(center[2])], dtype=np.float64)
        local = rot.T @ (world - center)
        local[0] = min(float(half[0]), max(-float(half[0]), float(local[0])))
        local[1] = min(float(half[1]), max(-float(half[1]), float(local[1])))
        local[2] = -float(half[2])
        return center + rot @ local

    def _planned_term(self, scratch: mj.MjData, adr: int) -> str:
        nv = int(self.model.nv)
        qvel = np.array(scratch.qvel, dtype=np.float64, copy=True)
        qacc = np.array(scratch.qacc, dtype=np.float64, copy=True)
        full = np.zeros(nv, dtype=np.float64)
        bias = np.zeros(nv, dtype=np.float64)
        mj.mj_rne(self.model, scratch, 1, full)
        scratch.qacc[:] = 0.0
        mj.mj_rne(self.model, scratch, 0, bias)
        scratch.qvel[:] = 0.0
        mj.mj_fwdVelocity(self.model, scratch)
        grav = np.zeros(nv, dtype=np.float64)
        scratch.qacc[:] = 0.0
        mj.mj_rne(self.model, scratch, 0, grav)
        scratch.qvel[:] = qvel
        scratch.qacc[:] = qacc
        inertial = float(full[adr] - bias[adr])
        arm = abs(self.leg_armature * float(qacc[adr]))
        qdd = abs(inertial - self.leg_armature * float(qacc[adr]))
        cor = abs(float(bias[adr] - grav[adr]))
        wrench = abs(float(scratch.qfrc_applied[adr]))
        scores = {
            "q̈": qdd + cor,
            "armature": arm,
            "gravity": abs(float(grav[adr])),
            "wrench": wrench,
        }
        return max(scores, key=lambda key: scores[key])

    def _shape_sagittal(self, joints: dict[str, float]) -> None:
        """Rate- and accel-limit hip pitch, knee, and ankle pitch.

        The published ctrl stays inside |ctrl−q| ≤ budget/kp, so the
        spring term kp·|e| cannot exceed ``gm_spring_nm``. The limit is
        that position band plus a velocity cap of budget/kv and an
        acceleration cap that reaches the velocity cap in four servo
        time constants (kv/kp). Measured ω only stops the command
        velocity from winding up against the band. The signed force
        kp·e − kv·ω is not projected onto ±budget.
        """
        budget = float(self.cfg.gm_spring_nm)
        if budget <= 0.0:
            return
        dt = float(op3_walk.OP3_CTRL_S)
        for name in list(joints):
            if not name.endswith(("hip_pitch", "knee", "ank_pitch")):
                continue
            act = name + "_pos"
            idx = self.act_idx.get(act)
            if idx is None:
                continue
            kp = float(self.model.actuator_gainprm[idx, 0])
            kv = -float(self.model.actuator_biasprm[idx, 2])
            if kp < 1e-6 or kv < 1e-6:
                continue
            q = self.q(name)
            jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name)
            omega = float(self.data.qvel[int(self.model.jnt_dofadr[jid])])
            e_max = budget / kp
            v_max = budget / kv
            a_max = v_max * kp / (4.0 * kv)
            if name not in self._shape_pos:
                self._shape_pos[name] = q
                self._shape_vel[name] = 0.0
            pos = float(self._shape_pos[name])
            vel = float(self._shape_vel[name])
            err = float(joints[name]) - pos
            wn = kp / (4.0 * kv)
            acc = wn * wn * err - 2.0 * wn * vel
            if acc > a_max:
                acc = a_max
            elif acc < -a_max:
                acc = -a_max
            vel = vel + acc * dt
            if vel > v_max:
                vel = v_max
            elif vel < -v_max:
                vel = -v_max
            pos = pos + vel * dt
            hi = q + e_max
            lo = q - e_max
            if pos > hi:
                pos = hi
                cap = max(0.0, omega)
                if vel > cap:
                    vel = cap
            elif pos < lo:
                pos = lo
                cap = min(0.0, omega)
                if vel < cap:
                    vel = cap
            self._shape_pos[name] = pos
            self._shape_vel[name] = vel
            joints[name] = pos

    def _limit_knee_qdd(self, joints: dict[str, float]) -> None:
        """Keep the knee reference inside WALK_KNEE_QDD_MAX.

        The cap is on the planned samples, before inverse dynamics. It is
        not a torque clip. A single-tick kink at −109 rad/s² fits in 2.33 Nm
        at armature 0.01 and does not fit at 0.025.
        """
        dt = float(op3_walk.OP3_CTRL_S)
        if dt <= 0.0:
            return
        cap = float(WALK_KNEE_QDD_MAX)
        for name in ("l_knee", "r_knee"):
            if name not in joints:
                continue
            q_new = float(joints[name])
            prev = self._knee_ref.setdefault(name, [])
            if len(prev) >= 2:
                q0 = float(prev[-2])
                q1 = float(prev[-1])
                qdd = (q_new - 2.0 * q1 + q0) / (dt * dt)
                if abs(qdd) > cap:
                    q_new = 2.0 * q1 - q0 + math.copysign(cap * dt * dt, qdd)
                    joints[name] = q_new
            prev.append(q_new)
            if len(prev) > 4:
                del prev[0]

    def _write_preview_joints(self, joints: dict[str, float] | None) -> None:
        for jn in (
            "l_sho_roll", "r_sho_roll", "l_el_pitch", "r_el_pitch",
            "l_el_yaw", "r_el_yaw", "l_gripper", "r_gripper",
        ):
            self.write_clipped(jn, self.q_stand.get(jn, 0.0))
        if joints is None:
            self.preview_ik_fail += 1
            self._write_unused()
            return
        # Swing-foot toe lift, scheduled from the clock so it is in place
        # before the landing. The gait ankle dives as the leg extends and
        # the toe meets the floor alone. The offset is slewed, then the
        # command is kept within 16 mrad of the measured angle so a late
        # gap does not become an ankle spike.
        for side in ("L", "R"):
            jn = self.pref(side) + "ank_pitch"
            if jn not in joints:
                continue
            gait = float(joints[jn])
            q = self.q(jn)
            # The stop blend used to slew this at 1.20 rad/s. The first
            # 8 ms sample is then a 150 rad/s² kink, and 0.01·q̈ is 1.5 Nm
            # on the ankle of an otherwise smooth quintic. 0.16 rad/s keeps
            # that corner near 20 rad/s². The walk slew is unchanged.
            if self.cfg.gm_id_ff and self._stand_u > 0.0:
                rate = 0.16
            else:
                rate = 1.20 if self._stand_u > 0.0 else 0.35
            self._stab_pitch[side] = self._slew(self._stab_pitch[side], self._swing_toe_bias(side), rate)
            want = gait + self._stab_pitch[side]
            # The 16 mrad band keeps a late swing gap off the ankle.
            # The stop blend's pitch null is already slewed; clamping it
            # leaves the light foot on the toe face.
            # The 16 mrad band keeps a late swing gap off the position
            # servo. Feedforward already builds the ankle torque inside
            # ±2.33 Nm, so the band would replace the gait reference.
            if self._stand_u <= 0.0 and not self.cfg.gm_id_ff:
                gap = want - q
                if abs(gap) > 0.016:
                    want = q + math.copysign(0.016, gap)
            if abs(want - gait) > 1e-6:
                joints[jn] = want
        if self.cfg.gm_id_ff:
            self._limit_knee_qdd(joints)
            # The hip-roll offset is part of the reference the torque
            # command tracks. A second write after the feedforward would
            # log the unshaped ask.
            self._accumulate_stab_hip()
            for name in ("l_hip_roll", "r_hip_roll"):
                if name in joints and abs(self._stab_hip) > 1e-6:
                    joints[name] = float(joints[name]) + self._stab_hip
            self._id_feedforward(joints)
        else:
            self._shape_sagittal(joints)
        for name, val in joints.items():
            # Straight preview walk. No yaw budget, no ±2.33 solve.
            # write_clipped still keeps non-sagittal joints inside the
            # existing 0.98·τ/kp band, except when the feedforward has
            # already built a torque command. It does not edit forcerange.
            self.write_clipped(name, float(val))
        if not self.cfg.gm_id_ff:
            self._stabilize_contacts()
        self._write_unused()

    def _preview_pose(self, walker: op3_walk.Op3Walker) -> dict[str, float] | None:
        """Joint targets at the current clock. Does not advance time."""
        joints, info = walker.joints_now()
        if info.phase == "L":
            self.phase = "swing"
            self.stance = "R"
        elif info.phase == "R":
            self.phase = "swing"
            self.stance = "L"
        else:
            self.phase = "shift"
        return joints

    def _voice_x_amp(self) -> float:
        """Hip-frame step for one swing of a full L+R cycle.

        A planted stance foot runs from +x_amp to −x_amp in the hip
        frame, so the body advances 4·x_amp per cycle. The step that
        matches the bus is x_amp = vx·T/4. kit_bus_step uses vx/7.50
        and ignores T, which only matches 4/T near the 0.50 s kit period.
        """
        period = float(self.cfg.gm_period_s)
        kin = abs(float(self.cmd_vx)) * period / 4.0
        return math.copysign(kin, float(self.cmd_vx))

    def _tick_preview_gait(self, walker: op3_walk.Op3Walker, walking: bool) -> None:
        """Shift CoM over the stance foot, then walk, then return in double support.

        y_swap_cmd is not written. The lateral channel is ``preview_y``.
        Stop does not call the ±2.33 stand solve.
        """
        if walker.y_swap_cmd != 0.0:
            raise RuntimeError("preview path refuses a non-zero y_swap")
        if not walking and self.preview_stage == "stand":
            walker.stop()
            self.phase = "stand"
            self.preview_stage = "stand"
            for jn, val in self.q_stand.items():
                self.write_clipped(jn, val)
            return
        if not walking and self.preview_stage == "start":
            # The lift has not started. Drop the shift back through the
            # same preview instead of snapping to the stand solve.
            self.preview_stage = "stop"
            self._stop_planted = True
            self._return_t = 0.0
            self._freeze_time = 0.0
            self._return_zmp0 = float(self.preview_com_y)
        if not walking:
            self.preview_stage = "stop"
            self._tick_preview_stop(walker)
            return
        if self.preview_stage == "stop":
            # A new vel after the soft stop has planted arms the shift
            # again. Until the return is done, the feet stay in that stop.
            if not self._return_done:
                self._tick_preview_stop(walker)
                return
            self.preview_stage = "stand"
            self._stop_planted = False
            self._stop_hold_s = 0.0
            self._return_t = 0.0
            self._return_hold = False
            self._return_done = False
            self._stand_q0 = None
            self._stand_q1 = None
            self._stand_v0 = None
            self._stand_u = 0.0
            self._stop_span = 0.0
            self._com_v0 = 0.0
            self._stop_swing_end = None
            self._gate_open = False
            self._gate_wait_s = 0.0
            self._restart = True
        preview = self._ensure_preview(walker)
        if self.preview_stage == "stand":
            self.preview_stage = "start"
            self._preview_clock = -float(self.cfg.preview_arm_s)
        if self.preview_stage == "start":
            walker.sole_level = self._sole_level_now()
            # The foot height grows with the arm. Dropping the full
            # swing in on the first tick is a knee step of several Nm.
            level = self._sole_level_now()
            walker.z_move_cmd = float(self.cfg.gm_z_m) * (1.0 - level)
            future = self._zmp_future(self._preview_clock)
            com = self._step_preview(preview, future)
            self.preview_com_y = com
            # Voice spreads the step during the arm. A full x_amp on the
            # first walk tick is a hip-pitch step of several Nm. The kit
            # path still starts from zero. z_flat holds the swing peak
            # across 20–80% of single support. z_quintic replaces that
            # hold with a minimum jerk over the whole single support.
            if self.cfg.gm_id_ff or (self.cfg.name == "voice056" and abs(self.cmd_vx) > 1e-4):
                if self.cfg.gm_id_ff:
                    walker.z_quintic = True
                    walker.z_lead = True
                    walker.z_flat = False
                else:
                    walker.z_quintic = bool(self.cfg.gm_z_quintic)
                    walker.z_lead = bool(self.cfg.gm_z_lead)
                    walker.z_flat = not walker.z_quintic
                x_full = self._voice_x_amp()
                walker.set_command(x_full * (1.0 - level), 0.0, 0.0)
                walker.previous_x = x_full
            else:
                walker.set_command(0.0, 0.0, 0.0)
                walker.previous_x = 0.0
            walker.time = 0.0
            walker.ctrl_running = False
            walker.update_movement()
            walker.preview_y = self._preview_y_with_stab(com)
            joints = self._preview_pose(walker)
            self.phase = "shift"
            self.stance = "R" if self.start_lead == "L" else "L"
            self._write_preview_joints(joints)
            self._preview_clock += op3_walk.OP3_CTRL_S
            if self._preview_clock >= -1e-9:
                self.preview_stage = "walk"
                walker.z_move_cmd = float(self.cfg.gm_z_m)
                walker.time = 0.0
                # Nonzero previous_x keeps the first swing at full amplitude.
                # Zero here is the OP3 half-step, which lands short of vx·T/2.
                if self.cfg.name == "voice056" and abs(self.cmd_vx) > 1e-4:
                    walker.previous_x = self._voice_x_amp()
                else:
                    walker.previous_x = 0.0
                walker.ctrl_running = True
                walker.update_movement()
            return
        walker.sole_level = 0.0
        x_amp, angle = kit_bus_step(self.cmd_vx, self.cmd_yaw, self.cfg.gm_period_s)
        # vx/7.50 ignores the period. One cycle advances the body by
        # 4·x_amp, so the step that matches vx is x_amp = vx·T/4.
        if self.cfg.gm_id_ff or (self.cfg.name == "voice056" and abs(self.cmd_vx) > 1e-4):
            if self.cfg.gm_id_ff:
                walker.z_quintic = True
                walker.z_lead = True
                walker.z_flat = False
            else:
                walker.z_quintic = bool(self.cfg.gm_z_quintic)
                walker.z_lead = bool(self.cfg.gm_z_lead)
                walker.z_flat = not walker.z_quintic
            x_amp = self._voice_x_amp()
        walker.set_command(x_amp, 0.0, angle)
        if self._hold_until_stance(walker, preview):
            return
        self._gate_wait_s = 0.0
        future = self._zmp_future(float(walker.time))
        com = self._step_preview(preview, future)
        self.preview_com_y = com
        walker.preview_y = self._preview_y_with_stab(com)
        joints, info = walker.step(op3_walk.OP3_CTRL_S)
        self._note_preview_phase(info)
        self._write_preview_joints(joints)

    def _sole_level_now(self) -> float:
        """1 on the quiet stand, 0 once the arm has finished.

        The stand spawns with the ankle matching the hip-pitch offset, so
        all four sole corners start on the floor. The arm returns that
        match to 0 on a smootherstep. The walk then uses the kit pitch.
        """
        if self._restart:
            return 0.0
        arm = float(self.cfg.preview_arm_s)
        if arm <= 1e-6:
            return 0.0
        u = (float(self._preview_clock) + arm) / arm
        if u <= 0.0:
            return 1.0
        if u >= 1.0:
            return 0.0
        s = u * u * u * (u * (u * 6.0 - 15.0) + 10.0)
        return 1.0 - s

    def _ssp_stance(self, t_s: float) -> str | None:
        """Stance foot during single support. None in double support."""
        walker = self.op3
        if walker is None or walker.period <= 1e-6:
            return None
        t = float(t_s) % walker.period
        if walker.l_ssp_start < t <= walker.l_ssp_end:
            return "R"
        if walker.r_ssp_start < t <= walker.r_ssp_end:
            return "L"
        return None

    def _ds_boundary(self, walker: op3_walk.Op3Walker) -> float | None:
        """Clock time of the double-support edge that opened this swing.

        ``t == r_ssp_start`` is still double support. Holding there puts
        both feet on the ground instead of commanding the swing up.
        """
        period = float(walker.period)
        if period <= 1e-6:
            return None
        t = float(walker.time)
        t_mod = t % period
        if walker.l_ssp_start < t_mod <= walker.l_ssp_end:
            edge = float(walker.l_ssp_start)
        elif walker.r_ssp_start < t_mod <= walker.r_ssp_end:
            edge = float(walker.r_ssp_start)
        else:
            return None
        cycles = math.floor(t / period)
        return cycles * period + edge

    def _on_stance_side(self, stance: str) -> bool:
        """True when CoM y is past the inner edge, onto ``stance``."""
        y = float(self.data.subtree_com[self.bid_body, 1])
        if stance == "L":
            return y >= zmp_preview.ENTRY_Y_M
        return y <= -zmp_preview.ENTRY_Y_M

    def _hold_until_stance(
        self, walker: op3_walk.Op3Walker, preview: zmp_preview.ZmpPreview,
    ) -> bool:
        """Keep both feet down until CoM is 8 mm onto the upcoming stance foot.

        The ZMP target on that hold is the box centre, ±0.043 m. The clock
        does not enter single support while the mass is still in the gap.
        After 1.5 s the clock is released and the miss is scored.
        """
        dt = op3_walk.OP3_CTRL_S
        t_now = float(walker.time)
        t_next = t_now + dt
        if t_next >= walker.period - 1e-12:
            t_next = 0.0
        stance_now = self._ssp_stance(t_now)
        stance_next = self._ssp_stance(t_next)
        if self._gate_open:
            if stance_now is not None:
                self._gate_open = False
                self._gate_wait_s = 0.0
            return False
        if stance_next is None or stance_next == stance_now:
            return False
        if self._on_stance_side(stance_next):
            return False
        self._gate_wait_s += dt
        if self._gate_wait_s >= 1.50:
            self._gate_open = True
            return False
        self.preview_gate_holds += 1
        target = zmp_preview.BOX_CENTER_Y_M if stance_next == "L" else -zmp_preview.BOX_CENTER_Y_M
        future = np.full(preview.horizon, target, dtype=np.float64)
        com = self._step_preview(preview, future)
        self.preview_com_y = com
        walker.preview_y = self._preview_y_with_stab(com)
        joints = self._preview_pose(walker)
        self.phase = "shift"
        self.stance = stance_next  # type: ignore[assignment]
        self._write_preview_joints(joints)
        return True

    def _feet_loaded(self) -> tuple[bool, bool]:
        """A foot at or above the 5 N unload is on the ground.

        The stop used to require 8 N. A stance foot at 7.8 N then kept
        the clock in swing while the swing foot still held about 15 N,
        and the contact CoP left the declared stance box.
        """
        unload = float(self.cfg.unload_n)
        return self.foot_normal("L") >= unload, self.foot_normal("R") >= unload

    def _loaded_box_zmp(self) -> float | None:
        """Box-centre ZMP when only one foot is carrying the weight.

        8 N, not the 5 N unload. A softer foot during the return is still
        double support. Treating 6 N as single support restarts the return
        and the contact CoP leaves the stance box.
        """
        fn_l = self.foot_normal("L")
        fn_r = self.foot_normal("R")
        if fn_l > 8.0 and fn_r <= 8.0:
            return zmp_preview.BOX_CENTER_Y_M
        if fn_r > 8.0 and fn_l <= 8.0:
            return -zmp_preview.BOX_CENTER_Y_M
        return None

    def _note_preview_phase(self, info: op3_walk.StepInfo) -> None:
        if info.phase == "L":
            self.phase = "swing"
            self.stance = "R"
            self._gm_swing = "L"
        elif info.phase == "R":
            self.phase = "swing"
            self.stance = "L"
            self._gm_swing = "R"
        else:
            self.phase = "shift"
            self._gm_swing = None
        self.lat = float(info.swap_y_m)

    def _plant_preview_return(self, walker: op3_walk.Op3Walker) -> None:
        """Both feet are down. The blend starts on this tick, at the walk velocity."""
        self._stop_planted = True
        self._freeze_time = float(walker.time)
        self._return_hold = False
        self._return_done = False
        self._stop_swing_end = None
        self._capture_stop_blend(walker)

    def _ref_velocity(self) -> dict[str, float]:
        """Joint velocity of the last two planned references. rad/s."""
        hist = self._plan_q_hist
        dt = float(op3_walk.OP3_CTRL_S)
        out: dict[str, float] = {}
        if len(hist) < 2 or dt <= 1e-9:
            return out
        for name, q1 in hist[-1].items():
            if name not in hist[-2]:
                continue
            out[name] = (float(q1) - float(hist[-2][name])) / dt
        return out

    def _capture_stop_blend(self, walker: op3_walk.Op3Walker) -> bool:
        """q0 and v0 from the last reference, before this tick appends another.

        q_stand is the spawn pose, sole_level 1, trunk upright.
        stand_joints() during the walk uses sole_level 0 and keeps the
        hip-pitch lean, so the blend target is q_stand.
        """
        hist = self._plan_q_hist
        if hist:
            q0 = {k: float(v) for k, v in hist[-1].items()}
        else:
            frozen = self._preview_pose(walker)
            if frozen is None:
                return False
            q0 = {k: float(v) for k, v in frozen.items()}
        self._stand_q0 = q0
        self._stand_v0 = self._ref_velocity()
        self._stand_q1 = {
            k: float(v) for k, v in self.q_stand.items() if k in q0
        }
        self._stand_u = 0.0
        self._return_t = 0.0
        self._return_zmp0 = float(self.preview_com_y)
        if self._preview is not None:
            self._com_v0 = float(self._preview.com_vel_m_s)
        else:
            self._com_v0 = 0.0
        self._stop_span = self._stop_blend_span()
        return True

    def _stop_blend_span(self) -> float:
        """At least 0.5 s, or one double-support interval, whichever is longer.

        The span then grows until both knees' quintic |q̈| stays inside
        STOP_KNEE_QDD_MAX. That cap is what keeps the planned knee torque,
        armature included, inside PLAN_KNEE_TAU_NM. The check is the
        measured τ_req, not this cap by itself.
        """
        period = float(self.cfg.gm_period_s)
        ds = max(0.0, float(self.cfg.gm_dsp)) * period
        span = max(STOP_BLEND_MIN_S, ds)
        q0 = self._stand_q0 or {}
        q1 = self._stand_q1 or {}
        v0 = self._stand_v0 or {}
        peak = 0.0
        for _ in range(16):
            peak = 0.0
            for name in ("l_knee", "r_knee"):
                if name not in q0 or name not in q1:
                    continue
                peak = max(peak, _quintic_qdd_peak(
                    float(q0[name]), float(q1[name]), float(v0.get(name, 0.0)), span,
                ))
            if peak <= STOP_KNEE_QDD_MAX or span >= 4.0:
                break
            span = min(4.0, span * 1.3)
        self.id_stop_qdd = float(peak)
        self.id_stop_span = float(span)
        return float(span)

    def _com_quintic(self, t_s: float, span: float) -> tuple[float, float, float]:
        """Planned CoM y, velocity, and acceleration into the DS centre."""
        return _quintic_sample(
            float(self._return_zmp0), 0.0, float(self._com_v0), max(span, 1e-6), t_s,
        )

    def _stance_y_limits(self) -> tuple[float, float]:
        """World-y extent of both contact boxes. The planned ZMP stays inside."""
        ys: list[float] = []
        for side in ("L", "R"):
            gid = int(self.gid[side])
            center = np.array(self.data.geom_xpos[gid], dtype=np.float64)
            rot = np.array(self.data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
            half = np.array(self.model.geom_size[gid], dtype=np.float64)
            for sx in (-1.0, 1.0):
                for sy in (-1.0, 1.0):
                    local = np.array([sx * half[0], sy * half[1], -half[2]], dtype=np.float64)
                    ys.append(float((center + rot @ local)[1]))
        if not ys:
            return -0.081, 0.081
        return min(ys), max(ys)

    def _stop_zmp_future(self, horizon: int, t_now: float, span: float) -> np.ndarray:
        """LIPM ZMP of the planned CoM deceleration, clamped into the stance boxes."""
        dt = float(op3_walk.OP3_CTRL_S)
        zc = float(self._preview_zc) if self._preview_zc > 0.05 else 0.18
        lo, hi = self._stance_y_limits()
        out = np.zeros(horizon, dtype=np.float64)
        for i in range(horizon):
            t = min(span, t_now + i * dt)
            com, _vel, acc = self._com_quintic(t, span)
            zmp = com - (zc / G) * acc
            if zmp < lo:
                zmp = lo
            elif zmp > hi:
                zmp = hi
            out[i] = zmp
        return out

    def _stand_blend_joints(self, walker: op3_walk.Op3Walker) -> dict[str, float] | None:
        """Quintic from the walk reference to stand, matching the incoming velocity.

        The first sample is one control step along the curve. A sample at
        u = 0 would repeat the last pose and drop q̇_ref to zero in 8 ms.
        End velocity and acceleration are zero, so the hold that follows
        does not add another step. Phase stays shift so this does not arm
        the kit stand hold.
        """
        if self._stand_q0 is None or self._stand_q1 is None:
            if not self._capture_stop_blend(walker):
                return None
        assert self._stand_q0 is not None and self._stand_q1 is not None
        span = self._stop_span if self._stop_span >= STOP_BLEND_MIN_S else STOP_BLEND_MIN_S
        dt = float(op3_walk.OP3_CTRL_S)
        t = min(span, self._stand_u + dt)
        self._stand_u = t
        v0 = self._stand_v0 or {}
        out: dict[str, float] = {}
        for key, dest in self._stand_q1.items():
            src = float(self._stand_q0.get(key, dest))
            q, _qd, _qdd = _quintic_sample(src, float(dest), float(v0.get(key, 0.0)), span, t)
            out[key] = q
        return out

    def _tick_preview_stop(self, walker: op3_walk.Op3Walker) -> None:
        preview = self._ensure_preview(walker)
        if not self._stop_planted:
            # Leave the step length alone. Zeroing it snaps hip pitch.
            # While one foot is up, the ZMP stays at that stance box centre
            # (±0.043 m). It does not follow the clock onto the airborne foot.
            # Finish the swing that is already in the air. Rewinding on
            # the first graze pulls the foot back up and the clock stalls.
            # Sole height, not the 5 N unload: a landed foot can sit near 1 N.
            if self._stop_swing_end is None:
                clock = self._ssp_stance(float(walker.time))
                if clock == "R":
                    self._stop_swing_end = float(walker.l_ssp_end)
                elif clock == "L":
                    self._stop_swing_end = float(walker.r_ssp_end)
                else:
                    self._stop_swing_end = float(walker.time)
            sole_l = sole_clearance(self.model, self.data, self.bid["L"], int(self.gid["L"]))
            sole_r = sole_clearance(self.model, self.data, self.bid["R"], int(self.gid["R"]))
            both_contact = sole_l < 0.003 and sole_r < 0.003
            loaded = self._loaded_box_zmp()
            clock_stance = self._ssp_stance(float(walker.time))
            period = float(walker.period)
            t_mod = float(walker.time) % period if period > 1e-6 else 0.0
            end = float(self._stop_swing_end) % period if period > 1e-6 else 0.0
            finishing = t_mod + 1e-9 < end
            if not finishing:
                # Plant on the first double-support tick. A held pose
                # drops the reference velocity to zero in one 8 ms sample.
                if both_contact and clock_stance is None:
                    self._plant_preview_return(walker)
                else:
                    future = np.full(preview.horizon, float(self.preview_com_y), dtype=np.float64)
                    com = self._step_preview(preview, future)
                    self.preview_com_y = com
                    walker.preview_y = self._preview_y_with_stab(com)
                    joints = self._preview_pose(walker)
                    self.phase = "shift"
                    self._write_preview_joints(joints)
                    self._stop_hold_s = 0.0
                    return
            else:
                self._stop_hold_s = 0.0
                # One foot is up. Finish this step on the declared stance
                # box. Do not steer the ZMP onto the foot the clock calls
                # the swing. Do not zero the step length: that snaps hip pitch.
                if loaded is None:
                    future = self._zmp_future(float(walker.time))
                else:
                    future = np.full(preview.horizon, loaded, dtype=np.float64)
                com = self._step_preview(preview, future)
                self.preview_com_y = com
                walker.preview_y = self._preview_y_with_stab(com)
                joints, info = walker.step(op3_walk.OP3_CTRL_S)
                self._note_preview_phase(info)
                # Contact still names the support. A clock swing whose swing
                # foot is the only loaded foot is that foot's stance, not the
                # clock's.
                if loaded is not None and info.phase in ("L", "R"):
                    heavy: Side = "L" if loaded > 0.0 else "R"
                    if self.stance != heavy:
                        self.phase = "swing"
                        self.stance = heavy
                        self._gm_swing = self.other(heavy)
                self._write_preview_joints(joints)
                return
        dt = op3_walk.OP3_CTRL_S
        if self._stand_q0 is None:
            if not self._capture_stop_blend(walker):
                self.phase = "shift"
                self._write_preview_joints(None)
                return
        span = self._stop_span if self._stop_span >= STOP_BLEND_MIN_S else STOP_BLEND_MIN_S
        # One light foot keeps the ZMP on that sole's box. The joint
        # quintic is not rewound: clearing it was the velocity step.
        airborne = self.foot_normal("L") < 1.0 or self.foot_normal("R") < 1.0
        loaded = self._loaded_box_zmp()
        t_now = min(span, self._return_t + dt)
        if airborne and loaded is not None and not self._return_done:
            future = np.full(preview.horizon, loaded, dtype=np.float64)
            self._return_hold = True
        else:
            self._return_hold = False
            future = self._stop_zmp_future(preview.horizon, t_now, span)
        self._return_t = t_now
        if self._return_t >= span - 1e-9:
            self._return_done = True
        self._step_preview(preview, future)
        if self._return_hold:
            com = float(self.preview_com_y)
        else:
            com, _vel, _acc = self._com_quintic(t_now, span)
            # Same shift the ZMP just took, so the stop acceleration stays
            # the quintic's and the weight moves with the plan.
            com = float(com) + float(self._split_zmp_applied)
        self.preview_com_y = com
        self._zmp_cmd = float(future[0])
        walker.time = self._freeze_time
        walker.preview_y = self._preview_y_with_stab(com)
        joints = self._stand_blend_joints(walker)
        # Stay in double support. Phase "stand" arms the kit stand hold,
        # which rewrites every leg toward the flat-floor pose. On the
        # entrance lip that yank is the ankle spike.
        self.phase = "shift"
        self._write_preview_joints(joints)

    def _tick_gait_manager(self, walking: bool) -> None:
        """OP3 cartesian foot targets through the plant-length IK.

        y_swap moves both feet sideways in the hip frame, so the pelvis
        sits over the stance foot. It is not a hip-roll lean of dy/0.22.
        The body drop is ``gm_crouch_m`` on the IK z offset. The walker
        advances one OP3 control cycle (8 ms), not the 50 Hz bus tick.
        """
        walker = self.op3
        if walker is None:
            self.hold_stand()
            return
        if self.cfg.preview_amp_m > 1e-6:
            self._tick_preview_gait(walker, walking)
            return
        if not walking:
            walker.stop()
            self._gm_swing = None
            self._move_entered = False
            self._yaw_sign = 0
            self._pending_lead = None
            self._resume_latched = False
            self.hold_stand()
            return
        x_amp, angle = kit_bus_step(self.cmd_vx, self.cmd_yaw, self.cfg.gm_period_s)
        # No vy. Cycle yaw is the bus yaw_rate; vx scales the step length.
        walker.set_command(x_amp, 0.0, angle)
        if not self._move_entered:
            # Stand has just released a clock at 0. Arm the cold lead once.
            self._move_entered = True
            walker.arm_swing(self.start_lead)
        self._arm_resume_lead(walker)
        joints, info = walker.step(op3_walk.OP3_CTRL_S)
        phase = info.phase
        if phase == "L":
            swing: Side = "L"
            self.phase = "swing"
            self.stance = "R"
        elif phase == "R":
            swing = "R"
            self.phase = "swing"
            self.stance = "L"
        else:
            swing = self.stance
            self.phase = "shift"
        if self._gm_swing != (swing if phase in ("L", "R") else None) and phase in ("L", "R"):
            self.lifts.append(
                LiftRecord(
                    t=float(self.data.time),
                    stance=self.stance,
                    cop_margin_m=self.cop_margin(self.stance),
                    swing_fn_n=self.foot_normal(swing),
                    com_inside=self.com_inside_foot(self.stance, self.cfg.inside_margin_m),
                    clear_cmd_m=self.cfg.gm_z_m,
                    z_budget_m=float(info.swap_y_m),
                )
            )
        self._gm_swing = swing if phase in ("L", "R") else None
        self.lat = float(info.swap_y_m)
        for jn in (
            "l_sho_roll", "r_sho_roll", "l_el_pitch", "r_el_pitch",
            "l_el_yaw", "r_el_yaw", "l_gripper", "r_gripper",
        ):
            self.write_clipped(jn, self.q_stand.get(jn, 0.0))
        if joints is None:
            self._write_unused()
            return
        for name, val in joints.items():
            # Shoulder targets already include arm_swing_gain. This path
            # writes them. The Bézier `arms` flag does not freeze the kit swing.
            # Hip roll on a yawed stance step was the joint over the 2.33 Nm
            # bar (left stance during the right swing, empty plant, −2.36 Nm
            # at 3.42 s). Damping adds to the spring while the hip is still
            # rolling the other way. The sag budget is the knee number, and
            # it applies only while the bus yaw is non-zero so a straight
            # walk is unchanged. Forcerange stays ±2.45 Nm.
            if name.endswith("hip_roll") and abs(self.cmd_yaw) > 1e-3:
                self.write_force_limited(name, val, KNEE_SAG_NM)
            else:
                self.write_clipped(name, val)
        if phase in ("L", "R"):
            self.z_bez = abs(info.ep_l[2] - info.ep_r[2])
        else:
            self.z_bez = 0.0
        self.z_cmd = self.z_bez
        self._write_unused()

    def _begin_shift(self) -> None:
        self.phase = "shift"
        self.phase_t = 0.0
        self.swing_s = 0.0
        self.z_limited = False
        self.hold_l = np.asarray(self.data.geom_xpos[self.gid["L"]], dtype=np.float64).copy()
        self.hold_r = np.asarray(self.data.geom_xpos[self.gid["R"]], dtype=np.float64).copy()
        # Hold the landing pose. Snapping both legs back to the stand
        # pitch/knee drags a placed foot back and the step disappears.
        self.hold_sagittal = {
            "L": self._sagittal_q("L"),
            "R": self._sagittal_q("R"),
        }
        self.yield_flex = 0.0
        self.yield_hip = 0.0
        self.yield_along = 0.0

    def _retry_shift(self) -> None:
        """More time on the same landing pose. Do not recapture the yield."""
        self.phase = "shift"
        self.phase_t = 0.0
        self.swing_s = 0.0
        self.hold_l = np.asarray(self.data.geom_xpos[self.gid["L"]], dtype=np.float64).copy()
        self.hold_r = np.asarray(self.data.geom_xpos[self.gid["R"]], dtype=np.float64).copy()

    def _tick_shift(self) -> None:
        com = self._com_xy()
        foot = self._foot_xy(self.stance)
        left = self._left_axis()
        err = float(np.dot(foot - com, left))
        # Ramp to the measured full-shift pose. The gate, not a bigger roll,
        # decides when the swing foot may leave the floor.
        if self.stance == "L":
            target = 0.0 if err < -0.012 else 1.0
        else:
            target = 0.0 if err > 0.012 else -1.0
        rate = self.cfg.shift_k * CTRL_DT
        self.lat += max(-rate, min(rate, target - self.lat))
        self.lat = max(-1.0, min(1.0, self.lat))
        self._write_shift()
        self._write_sagittal_hold()
        self._apply_forward_lean()
        self._yield_rear()
        if self.phase_t < self.cfg.t_shift_min:
            return
        swing = self.other(self.stance)
        fn = self.foot_normal(swing)
        fn_stance = self.foot_normal(self.stance)
        inside = self.com_inside_foot(self.stance, self.cfg.inside_margin_m)
        margin = self.cop_margin(self.stance)
        share = fn / max(fn + fn_stance, 1e-6)
        cop_x = self._cop_x(self.stance)
        if inside and fn <= self.cfg.unload_n and margin >= 0.0:
            self._begin_swing(share, cop_x)
            return
        if self.phase_t >= self.cfg.t_shift_max:
            self.missed_gates += 1
            self.lifts.append(
                LiftRecord(
                    t=float(self.data.time),
                    stance=self.stance,
                    cop_margin_m=margin,
                    swing_fn_n=fn,
                    com_inside=inside,
                    clear_cmd_m=0.0,
                    z_budget_m=0.0,
                    cop_x_m=cop_x,
                    rear_share=share,
                )
            )
            # The return lean is twice as far as the first one. One miss
            # keeps this stance and gives the load time to finish moving.
            # A second miss swaps, so a wrong-way lean cannot run forever.
            # Retry keeps the landing pose. Recapturing would stack the yield.
            self._same_misses += 1
            if self._same_misses >= 2:
                self.stance = self.other(self.stance)
                self._same_misses = 0
                self._begin_shift()
            else:
                self._retry_shift()

    def _begin_swing(self, rear_share: float | None = None, cop_x: float | None = None) -> None:
        swing = self.other(self.stance)
        fn = self.foot_normal(swing)
        fn_stance = self.foot_normal(self.stance)
        margin = self.cop_margin(self.stance)
        inside = self.com_inside_foot(self.stance, self.cfg.inside_margin_m)
        if rear_share is None:
            rear_share = fn / max(fn + fn_stance, 1e-6)
        if cop_x is None:
            cop_x = self._cop_x(self.stance)
        self.p0 = np.asarray(self.data.geom_xpos[self.gid[swing]], dtype=np.float64).copy()
        self.hold_l = np.asarray(self.data.geom_xpos[self.gid["L"]], dtype=np.float64).copy()
        self.hold_r = np.asarray(self.data.geom_xpos[self.gid["R"]], dtype=np.float64).copy()
        self.p3 = self._foothold(swing, self.cmd_vx, self.cmd_yaw)
        self.swing_q0 = self._sagittal_q(swing)
        self.stance_q0 = self._sagittal_q(self.stance)
        self.lifts.append(
            LiftRecord(
                t=float(self.data.time),
                stance=self.stance,
                cop_margin_m=margin,
                swing_fn_n=fn,
                com_inside=inside,
                clear_cmd_m=self.cfg.clear_m,
                z_budget_m=0.0,
                cop_x_m=float(cop_x),
                rear_share=float(rear_share),
            )
        )
        self.phase = "swing"
        self.phase_t = 0.0
        self.swing_s = 0.0
        self.z_limited = False
        self._same_misses = 0
        self.sag_lean = 0.0
        self.yield_flex = 0.0
        self.yield_hip = 0.0
        self.swing_held = False
        self.reach_hip = 0.0
        fwd = self._fwd_axis()
        self.liftoff_along = float(np.dot(self._foot_xy(swing) - self._foot_xy(self.stance), fwd))

    def _sagittal_q(self, side: Side) -> dict[str, float]:
        pref = self.pref(side)
        return {
            "hip": self.q(pref + "hip_pitch"),
            "knee": self.q(pref + "knee"),
            "yaw": self.q(pref + "hip_yaw"),
        }

    def _tick_swing(self, yaw_rate: float) -> None:
        swing = self.other(self.stance)
        self.swing_s = min(1.0, self.phase_t / max(self.cfg.t_swing, 1e-3))
        step = self._step_length()
        self._write_joint_bezier(swing, self.swing_s, step)
        self.z_bez = float(self.p0[2]) + self.cfg.clear_m * math.sin(math.pi * self.swing_s)
        self.z_cmd = self.z_bez
        self._cop_trim(self.stance)
        self._swing_yaw(swing, yaw_rate)
        landed = self.swing_s >= 0.999 and (
            self.foot_contact(swing) or self.phase_t > self.cfg.t_swing + 0.30
        )
        if landed:
            self.stance = swing
            self._begin_shift()

    def _step_length(self) -> float:
        """Raibert length from CommandBus vx, clipped to the vendor 2 cm step."""
        period = self.cfg.t_swing + 0.5 * (self.cfg.t_shift_min + 0.40)
        raw = abs(self.cmd_vx) * period
        mag = min(VENDOR_STEP_M, max(0.0, raw))
        if abs(self.cmd_vx) < 1e-4:
            mag = 0.008
        return math.copysign(mag, self.cmd_vx if abs(self.cmd_vx) > 1e-6 else 1.0)

    def _write_joint_bezier(self, swing: Side, s: float, step_m: float) -> None:
        """Flat-foot swing. Knee carries the sole height; hip carries the step.

        World-frame Jacobian IK left the knee almost idle: at this pose the
        knee's vertical lever is about 2 cm per radian, and the hip ate the
        update to chase a longer step. The joint curve below is the IK of a
        2 cm Bézier on this plant. Knee and hip pitch may lead by the HX
        slew. The plant forcerange still clips torque.
        """
        scale_h = max(0.0, self.cfg.clear_m) / VENDOR_CLEAR_M
        scale_x = abs(step_m) / VENDOR_STEP_M
        direction = 1.0 if step_m >= 0.0 else -1.0
        # Command the peak immediately. A sine spends the swing catching a
        # moving target, and the knee never reaches the pose that clears.
        # The position clip is the ramp. Descent starts early enough that
        # the same clip can extend onto the landing pose before touchdown.
        sole = self._sole(swing)
        # Hold the clearance flexion through the reach so the foot swings
        # forward in the air. Extending while it is still behind plants early.
        if s < RISE_S or (s < REACH_S and sole < 0.010):
            flex = scale_h * FLEX_PEAK
            dhip = direction * scale_x * HIP_RISE
        elif s < REACH_S:
            flex = scale_h * FLEX_PEAK
            dhip = direction * scale_x * HIP_REACH
        else:
            flex = scale_h * FLEX_LAND
            dhip = direction * scale_x * HIP_LAND
            if sole < 0.008:
                dhip += self._foothold_hip(swing, step_m)
            dhip = max(-0.36, min(0.36, dhip))
        self._write_shift()
        self._apply_sagittal(swing, self.swing_q0, flex, dhip, swing=True)
        self._level_swing_roll(swing)
        # Positive push extends the stance hip (pelvis forward). The swing
        # flex sign is the opposite, so this is minus the swing direction.
        push = -direction * STANCE_PUSH * min(1.0, scale_x)
        self._apply_sagittal(self.stance, self.stance_q0, 0.0, push, swing=False)

    def _apply_sagittal(
        self,
        side: Side,
        q0: dict[str, float],
        flex: float,
        dhip: float,
        swing: bool,
    ) -> None:
        if not q0:
            return
        pref = self.pref(side)
        # L hip joint decreases to swing the thigh forward. R increases.
        # L knee increases to flex. R knee decreases.
        hip_sign = -1.0 if side == "L" else 1.0
        knee_sign = 1.0 if side == "L" else -1.0
        if not swing:
            # Stance push is a forward thigh move, same hip sign, no extra flex.
            flex = 0.0
        hip = q0.get("hip", 0.0) + hip_sign * dhip
        knee = q0.get("knee", 0.0) + knee_sign * flex
        if swing:
            # Caps are relative to the stand pose. Adding the Bézier on top
            # of a leg that already reached forward folds the next step.
            stand_hip = self.q_stand.get(pref + "hip_pitch", 0.0)
            stand_knee = self.q_stand.get(pref + "knee", 0.0)
            hip_cap = abs(dhip) + 0.02
            knee_cap = abs(flex) + 0.02
            if side == "R":
                hip = min(hip, stand_hip + hip_cap)
                knee = max(knee, stand_knee - knee_cap)
            else:
                hip = max(hip, stand_hip - hip_cap)
                knee = min(knee, stand_knee + knee_cap)
        self.write_clipped(pref + "hip_pitch", hip)
        self.write_clipped(pref + "knee", knee)
        # Flat foot tracks the hip and knee commands sent this tick, not the
        # unclipped Bézier. Chasing the unclipped sum pitches the box onto
        # a corner and the lowest sole stays on the floor.
        hip_c = float(self.data.ctrl[self.act_idx[pref + "hip_pitch_pos"]])
        knee_c = float(self.data.ctrl[self.act_idx[pref + "knee_pos"]])
        ank = hip_c + knee_c
        if swing:
            up = self.data.xmat[self.bid[side]].reshape(3, 3)[:, 2]
            # +l_ank_pitch lowers up[0]; +r_ank_pitch raises it.
            sign = 1.0 if side == "L" else -1.0
            ank += sign * float(up[0])
        self.write_clipped(pref + "ank_pitch", ank)
        if "yaw" in q0 and not swing:
            self.write_clipped(pref + "hip_yaw", q0["yaw"])

    def _limit_z(self, swing: Side, p: np.ndarray) -> np.ndarray:
        """Keep the Bézier from commanding the foot back down early.

        The position servo already clips each joint to |ctrl-q| <= tau/kp.
        Capping cartesian z to one tick of that band pins the target on the
        floor, because the band is a lag, not the swing's total travel.
        What must not happen is a descending reference while the sole is
        still short: that spends the knee torque on extension.
        """
        out = p.copy()
        foot_z = float(self.data.geom_xpos[self.gid[swing]][2])
        self.z_budget = self._dz_budget(swing)
        peak = float(self.p0[2]) + self.cfg.clear_m
        if self.swing_s < 0.80 and foot_z + 0.006 < peak and out[2] < foot_z:
            out[2] = foot_z
            self.z_limited = True
        return out

    def _dz_budget(self, side: Side) -> float:
        """Vertical geom step if each sagittal joint moves one torque-band."""
        gid = self.gid[side]
        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        jacr = np.zeros((3, self.model.nv), dtype=np.float64)
        mj.mj_jacGeom(self.model, self.data, jacp, jacr, gid)
        dz = 0.0
        for jn in ("hip_pitch", "knee", "ank_pitch"):
            name = self.pref(side) + jn
            jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name)
            col = int(self.model.jnt_dofadr[jid])
            jz = float(jacp[2, col])
            band = self.e_sat.get(name + "_pos", 0.0)
            dz += abs(jz) * band
        return dz

    def _foothold(self, swing: Side, vx: float, yaw_rate: float) -> np.ndarray:
        fwd = self._fwd_axis()
        left = self._left_axis()
        com, v = self._com_state()
        omega = math.sqrt(G / max(self._com_z() - self._sole(self.stance), 0.12))
        period = self.cfg.t_swing + 0.5 * (self.cfg.t_shift_min + self.cfg.t_shift_max)
        step = float(vx) * period
        # Raibert: correct the kinematic step by the velocity error.
        vx_m = float(np.dot(v, fwd))
        step += 0.12 * (vx_m - float(vx)) * self.cfg.t_swing
        # Capture point at touchdown if the stance foot is the ZMP, clipped.
        cp = com + v / max(omega, 0.5)
        stance_xy = self._foot_xy(self.stance)
        cp_td = stance_xy + (cp - stance_xy) * math.exp(min(1.2, omega * self.cfg.t_swing))
        v_des = float(vx) * fwd
        p_cp = cp_td - v_des / max(omega, 0.5)
        along_cp = float(np.dot(p_cp - self.p0[:2], fwd))
        lat_cp = float(np.dot(p_cp - self.p0[:2], left))
        # Yaw: lengthen the outside of the turn. +yaw_rate is a left turn.
        yaw_lat = float(yaw_rate) * period * 0.15
        if swing == "R":
            yaw_lat = -yaw_lat
        along = 0.65 * step + 0.35 * along_cp
        lat = 0.35 * lat_cp + yaw_lat
        if vx >= 0.0:
            along = min(VENDOR_STEP_M, max(-0.005, along))
        else:
            along = min(0.005, max(-VENDOR_STEP_M, along))
        lat = min(0.015, max(-0.015, lat))
        p3 = self.p0.copy()
        p3[:2] = self.p0[:2] + fwd * along + left * lat
        p3[2] = float(self.p0[2])
        return p3

    def _write_shift(self) -> None:
        sl = self.q_stand.get("l_hip_roll", -0.05)
        sr = self.q_stand.get("r_hip_roll", 0.05)
        al = self.q_stand.get("l_ank_roll", 0.0)
        ar = self.q_stand.get("r_ank_roll", 0.0)
        self.write_clipped("l_hip_roll", sl + SHIFT_HIP_L * self.lat)
        self.write_clipped("r_hip_roll", sr + SHIFT_HIP_R * self.lat)
        self.write_clipped("l_ank_roll", al + SHIFT_ANK * self.lat)
        self.write_clipped("r_ank_roll", ar + SHIFT_ANK * self.lat)

    def _write_sagittal_hold(self) -> None:
        for side in ("L", "R"):
            q0 = self.hold_sagittal.get(side)
            if not q0:
                continue
            self._apply_sagittal(side, q0, 0.0, 0.0, swing=False)

    def _apply_forward_lean(self) -> None:
        """No sagittal push during double support.

        Extending the rear hip does move the pelvis toward the forward foot,
        and it also walks the CoP toward the toe. Full weight on that toe
        needs about 2.36 Nm. The freeze is ±2.1 Nm, so the torso pitches
        over (a 0.05 rad hold tipped near 13 s, a 0.08 rad swing push fell
        immediately). Leaving this at zero keeps the 2 cm step upright.
        """
        self.sag_lean = 0.0

    def _torso_up(self) -> float:
        return float(self.data.xmat[self.bid_body].reshape(3, 3)[2, 2])

    def _yield_rear(self) -> None:
        """Shorten the trailing leg once the lean and the COM are on the front foot.

        The overlap does not put the COM on the toe. It leaves the rear
        position servo still holding part of the weight, so the 5 N gate
        never opens and the same foot swings again. Flexing only the rear
        knee, with the hip left on the landing pose and the ankle the
        flat-foot sum, lets that normal force fall. The stance leg is what
        holds the pelvis up.
        """
        rear = self.other(self.stance)
        q0 = self.hold_sagittal.get(rear)
        if not q0:
            return
        leaned = abs(self.lat) >= YIELD_LAT
        inside = self.com_inside_foot(self.stance, self.cfg.inside_margin_m)
        up = self._torso_up()
        if leaned and inside and up >= 0.94:
            self.yield_flex = min(YIELD_KNEE, self.yield_flex + YIELD_RATE * CTRL_DT)
        elif up < 0.93:
            self.yield_flex = max(0.0, self.yield_flex - YIELD_RATE * CTRL_DT)
        if self.yield_flex <= 1e-4:
            return
        pref = self.pref(rear)
        knee_sign = 1.0 if rear == "L" else -1.0
        hip_sign = -1.0 if rear == "L" else 1.0
        along = float(np.dot(
            self._foot_xy(rear) - self._foot_xy(self.stance), self._fwd_axis(),
        ))
        if self.yield_flex < 0.04:
            self.yield_along = along
            self.yield_hip = 0.0
        else:
            slipped = self.yield_along - along
            self.yield_hip = min(YIELD_HIP_MAX, max(0.0, 12.0 * slipped))
        knee = q0.get("knee", 0.0) + knee_sign * self.yield_flex
        hip = q0.get("hip", 0.0) + hip_sign * self.yield_hip
        self.write_clipped(pref + "hip_pitch", hip)
        self.write_clipped(pref + "knee", knee)
        hip_i = self.act_idx[pref + "hip_pitch_pos"]
        knee_i = self.act_idx[pref + "knee_pos"]
        ank = float(self.data.ctrl[hip_i] + self.data.ctrl[knee_i])
        sole_up = self.data.xmat[self.bid[rear]].reshape(3, 3)[:, 2]
        sign = 1.0 if rear == "L" else -1.0
        ank += sign * float(sole_up[0])
        self.write_clipped(pref + "ank_pitch", ank)
        # The lean rolls the rear box onto a corner. Geom center can sit
        # almost 2 cm up while the lowest corner, and the normal force, stay
        # on the floor. Level this sole before the gate reads that force.
        self._level_swing_roll(rear)

    def _level_swing_roll(self, swing: Side) -> None:
        """Cancel the lean roll on the swing sole.

        The shift that unloads this foot also rolls it. With the box on a
        corner the geom center can be 2 cm up while the lowest sole is not.
        +ank_roll lowers up[1] on both feet. This write replaces the shift's
        ankle-roll target for the swing leg only.
        """
        up = self.data.xmat[self.bid[swing]].reshape(3, 3)[:, 2]
        jn = self.pref(swing) + "ank_roll"
        self.write_clipped(jn, self.q(jn) + float(up[1]))

    def _foothold_hip(self, swing: Side, step_m: float) -> float:
        """Extra hip so the sole lands about one vendor step ahead of stance.

        Knee flexion alone retracts the foot. A fixed hip from the liftoff
        pose only catches up to the stance foot. This trim is the Raibert
        placement, in joint space, and it is clipped so it cannot fold the
        leg past the landing pose measured on this plant.
        """
        fwd = self._fwd_axis()
        along = float(np.dot(self._foot_xy(swing) - self._foot_xy(self.stance), fwd))
        err = float(step_m) - along
        return max(-0.04, min(0.06, 3.0 * err))

    def _resolved_rate(
        self,
        side: Side,
        joints: tuple[str, ...],
        p_des: np.ndarray,
        orient: bool = False,
        z_scale: float = 1.0,
    ) -> None:
        gid = self.gid[side]
        bid = self.bid[side]
        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        jacr = np.zeros((3, self.model.nv), dtype=np.float64)
        mj.mj_jacGeom(self.model, self.data, jacp, jacr, gid)
        names: list[str] = []
        cols: list[int] = []
        for jn in joints:
            name = self.pref(side) + jn
            jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                continue
            names.append(name)
            cols.append(int(self.model.jnt_dofadr[jid]))
        if len(cols) < 2:
            return
        err = np.asarray(p_des, dtype=np.float64) - np.asarray(self.data.geom_xpos[gid], dtype=np.float64)
        err[2] *= z_scale
        v = np.clip(self.cfg.ik_kp * err, -0.70, 0.70)
        Jlin = jacp[:, cols]
        if orient:
            rot = self.data.xmat[bid].reshape(3, 3)
            up = rot[:, 2]
            w = np.cross(up, np.array([0.0, 0.0, 1.0], dtype=np.float64))
            v = np.concatenate([v, 5.0 * w[:2]])
            J = np.vstack([Jlin, jacr[:2, cols]])
        else:
            J = Jlin
        n = J.shape[0]
        a = J @ J.T + 2e-4 * np.eye(n)
        try:
            dq = J.T @ np.linalg.solve(a, v)
        except np.linalg.LinAlgError:
            return
        for name, rate in zip(names, dq):
            self.write_clipped(name, self.q(name) + float(rate) * CTRL_DT)

    def _flatten(self, side: Side, pitch: bool, roll: bool) -> None:
        """Keep the sole plane level with the floor.

        Pitch is the closed-form flat foot: ankle command tracks hip+knee in
        joint coordinates (the stand pose uses that sum). A measured-normal
        term trims the residual. Roll deadbeats up[1]. Both writes stay inside
        the torque band, so this cannot outrun ±2.1 Nm.
        """
        up = self.data.xmat[self.bid[side]].reshape(3, 3)[:, 2]
        pref = self.pref(side)
        if pitch:
            hip_i = self.act_idx[pref + "hip_pitch_pos"]
            knee_i = self.act_idx[pref + "knee_pos"]
            ank = float(self.data.ctrl[hip_i] + self.data.ctrl[knee_i])
            # +l_ank_pitch lowers up[0]; +r_ank_pitch raises it.
            sign = 1.0 if side == "L" else -1.0
            ank += sign * float(up[0])
            self.write_clipped(pref + "ank_pitch", ank)
        if roll:
            jn = pref + "ank_roll"
            # +ank_roll lowers up[1] on both feet, about 1 rad per rad.
            self.write_clipped(jn, self.q(jn) + float(up[1]))

    def _cop_trim(self, side: Side) -> None:
        """Stance CoP a few millimetres behind the COM, inside the box.

        x_ddot = (g/h)*(x − p). CoP behind the COM accelerates the body
        forward. CoP on the toe needs about 2.36 Nm at full weight and is
        not commanded. The target is clipped off both the heel edge and the
        toe, and it is never ahead of the COM.
        """
        local = self._cop_local(side)
        if local is None:
            return
        com_l = self._com_local_xy(side)
        center = np.asarray(self.model.geom_pos[self.gid[side], :2], dtype=np.float64)
        half = np.asarray(self.model.geom_size[self.gid[side], :2], dtype=np.float64)
        lo = float(center[0] - half[0] + 0.012)
        hi = float(min(center[0] + half[0] - 0.025, float(com_l[0]) - 0.001))
        if hi < lo:
            hi = lo
        target_x = min(hi, max(lo, float(com_l[0]) - HEEL_LEAD_M))
        err_x = float(local[0] - target_x)
        err_y = float(local[1] - center[1])
        sign = 1.0 if side == "L" else -1.0
        jn = self.pref(side) + "ank_pitch"
        pitch_i = self.act_idx[jn + "_pos"]
        # CoP ahead of the target → dorsiflex (positive on L, negative on R).
        self.write_clipped(
            jn,
            float(self.data.ctrl[pitch_i]) + COP_PITCH_SIGN * sign * self.cfg.cop_k * err_x,
        )
        jroll = self.pref(side) + "ank_roll"
        roll_i = self.act_idx[jroll + "_pos"]
        self.write_clipped(jroll, float(self.data.ctrl[roll_i]) + self.cfg.cop_k * 0.35 * err_y)

    def _swing_yaw(self, swing: Side, yaw_rate: float) -> None:
        # +hip yaw toes the foot right. A left command (+yaw_rate) toes left.
        stick = max(-1.0, min(1.0, yaw_rate / 0.25))
        hy = -0.10 * stick
        self.write_clipped(self.pref(swing) + "hip_yaw", hy)
        self.write_clipped(self.pref(self.stance) + "hip_yaw", 0.0)

    def _write_arms(self) -> None:
        for jn in (
            "l_sho_roll", "r_sho_roll", "l_el_pitch", "r_el_pitch",
            "l_el_yaw", "r_el_yaw", "l_gripper", "r_gripper",
            "l_sho_pitch", "r_sho_pitch",
        ):
            self.write_clipped(jn, self.q_stand.get(jn, 0.0))
        if not self.cfg.arms or self.phase != "swing":
            return
        amp = self.cfg.arm_amp * math.sin(math.pi * self.swing_s)
        swing = self.other(self.stance)
        # Contralateral shoulder, light. Same sign the old gait used for a
        # forward swing on that arm (negative pitch).
        if swing == "L":
            self.write_clipped("r_sho_pitch", self.q_stand.get("r_sho_pitch", 0.0) - amp)
        else:
            self.write_clipped("l_sho_pitch", self.q_stand.get("l_sho_pitch", 0.0) - amp)

    def _write_unused(self) -> None:
        for jn in ("head_pan", "head_tilt"):
            self.write_clipped(jn, self.q_stand.get(jn, 0.0))

    def _hold(self, side: Side) -> np.ndarray:
        return self.hold_l if side == "L" else self.hold_r

    def _com_local_xy(self, side: Side) -> np.ndarray:
        com = np.asarray(self.data.subtree_com[self.bid_body], dtype=np.float64)
        rot = self.data.xmat[self.bid[side]].reshape(3, 3)
        local = rot.T @ (com - np.asarray(self.data.xpos[self.bid[side]], dtype=np.float64))
        return local[:2]

    def _cop_x(self, side: Side) -> float:
        local = self._cop_local(side)
        if local is None:
            return float("nan")
        return float(local[0])

    def _com_xy(self) -> np.ndarray:
        return np.asarray(self.data.subtree_com[self.bid_body, :2], dtype=np.float64)

    def _com_z(self) -> float:
        return float(self.data.subtree_com[self.bid_body, 2])

    def _com_state(self) -> tuple[np.ndarray, np.ndarray]:
        com = self._com_xy()
        # cvel is [rot; lin] of the body, world frame. Finite-difference of
        # subtree_com is steadier for the capture point; use body lin vel.
        v = np.asarray(self.data.cvel[self.bid_body, 3:5], dtype=np.float64)
        return com, v

    def _yaw(self) -> float:
        w, x, y, z = (float(v) for v in self.data.qpos[3:7])
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _fwd_axis(self) -> np.ndarray:
        yaw = self._yaw()
        return np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float64)

    def _left_axis(self) -> np.ndarray:
        yaw = self._yaw()
        return np.array([-math.sin(yaw), math.cos(yaw)], dtype=np.float64)

    def _foot_xy(self, side: Side) -> np.ndarray:
        return np.asarray(self.data.geom_xpos[self.gid[side], :2], dtype=np.float64)

    def _on_ground(self, g1: int, g2: int, gid: int) -> bool:
        if g1 != gid and g2 != gid:
            return False
        other = g2 if g1 == gid else g1
        if other == self.gid_floor:
            return True
        return other in self.ground_extra

    def foot_contact(self, side: Side) -> bool:
        bid = self.bid[side]
        grounds = {int(self.gid_floor), *[int(g) for g in self.ground_extra if int(g) >= 0]}
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            b1 = int(self.model.geom_bodyid[g1])
            b2 = int(self.model.geom_bodyid[g2])
            if (b1 == bid or b2 == bid) and (g1 in grounds or g2 in grounds):
                return True
        return False

    def foot_normal(self, side: Side) -> float:
        """Floor normal, plus any runtime ground geom such as the entrance rug.

        Overlapping 135 mm soles collide with each other on a 2 cm step.
        That foot-foot force is not weight on the rear foot.
        """
        gid = self.gid[side]
        total = 0.0
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if not self._on_ground(g1, g2, gid):
                continue
            force = np.zeros(6, dtype=np.float64)
            mj.mj_contactForce(self.model, self.data, i, force)
            total += float(force[0])
        return total

    def _cop_sole(self, side: Side) -> np.ndarray | None:
        """Contact CoP in the sole geom frame, floor and any extra ground.

        The QP CoP is in this same frame: geom centre, geom axes. Rug
        contacts count. Foot-foot contacts do not.
        """
        gid = self.gid[side]
        num = np.zeros(3, dtype=np.float64)
        den = 0.0
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if not self._on_ground(g1, g2, gid):
                continue
            force = np.zeros(6, dtype=np.float64)
            mj.mj_contactForce(self.model, self.data, i, force)
            fn = float(force[0])
            if fn <= 1e-6:
                continue
            num += fn * np.asarray(c.pos, dtype=np.float64)
            den += fn
        if den <= 1e-6:
            return None
        world = num / den
        rot = np.asarray(self.data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
        center = np.asarray(self.data.geom_xpos[gid], dtype=np.float64)
        local = rot.T @ (world - center)
        return local[:2]

    def _note_realised_split(self) -> None:
        """Contact-solver split and CoP against the QP sample from this tick.

        Both feet have to be carrying weight. A light foot is not a
        share the position servo failed to command. The gap updates the
        ZMP bias used on the next tick.
        """
        pending = self._qp_sample
        if pending is None:
            return
        fn_l = self.foot_normal("L")
        fn_r = self.foot_normal("R")
        if fn_l < 5.0 or fn_r < 5.0:
            return
        alpha_real = fn_l / (fn_l + fn_r)
        alpha_qp = float(pending["alpha"])
        gap = abs(alpha_real - alpha_qp)
        self.id_real_n += 1
        self.id_real_split_sum += gap
        self.id_real_qp_sum += alpha_qp
        self.id_real_fn_sum += alpha_real
        if gap > self.id_real_split_max:
            self.id_real_split_max = gap
            self.id_real_split_t = float(self.data.time)
            self.id_real_split_qp = alpha_qp
            self.id_real_split_fn = alpha_real
        for i, side in enumerate(("L", "R")):
            if float(pending["fz"][i]) <= 1.0:
                continue
            real = self._cop_sole(side)
            if real is None:
                continue
            dist = float(np.linalg.norm(real - np.asarray(pending["cop"][i], dtype=np.float64)))
            self.id_real_cop_n[side] += 1
            self.id_real_cop_sum[side] += dist
            if dist > self.id_real_cop_max[side]:
                self.id_real_cop_max[side] = dist
                self.id_real_cop_t[side] = float(self.data.time)
        y_l = float(self.data.geom_xpos[int(self.gid["L"]), 1])
        y_r = float(self.data.geom_xpos[int(self.gid["R"]), 1])
        span = y_l - y_r
        if abs(span) < 1e-4:
            return
        err = alpha_qp - alpha_real
        step = SPLIT_TRACK_STEP * err * span
        step = min(SPLIT_TRACK_STEP_M, max(-SPLIT_TRACK_STEP_M, step))
        bias = float(self._split_zmp_bias) + step
        self._split_zmp_bias = min(SPLIT_TRACK_BIAS_M, max(-SPLIT_TRACK_BIAS_M, bias))
        self.id_split_bias_max = max(self.id_split_bias_max, abs(float(self._split_zmp_bias)))

    def _cop_local(self, side: Side) -> np.ndarray | None:
        gid = self.gid[side]
        bid = self.bid[side]
        floor = self.gid_floor
        num = np.zeros(3, dtype=np.float64)
        den = 0.0
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if not ((g1 == gid and g2 == floor) or (g2 == gid and g1 == floor)):
                continue
            force = np.zeros(6, dtype=np.float64)
            mj.mj_contactForce(self.model, self.data, i, force)
            fn = float(force[0])
            if fn <= 1e-6:
                continue
            num += fn * np.asarray(c.pos, dtype=np.float64)
            den += fn
        if den <= 1e-6:
            return None
        world = num / den
        rot = self.data.xmat[bid].reshape(3, 3)
        local = rot.T @ (world - np.asarray(self.data.xpos[bid], dtype=np.float64))
        return local[:2]

    def cop_margin(self, side: Side) -> float:
        """Positive when the contact CoP is inside the 145×86 box."""
        local = self._cop_local(side)
        if local is None:
            return -1.0
        center = np.asarray(self.model.geom_pos[self.gid[side], :2], dtype=np.float64)
        half = np.asarray(self.model.geom_size[self.gid[side], :2], dtype=np.float64)
        slack = half - np.abs(local - center)
        return float(min(slack[0], slack[1]))

    def com_inside_foot(self, side: Side, margin: float) -> bool:
        bid = self.bid[side]
        gid = self.gid[side]
        com = np.asarray(self.data.subtree_com[self.bid_body], dtype=np.float64)
        rot = self.data.xmat[bid].reshape(3, 3)
        local = rot.T @ (com - np.asarray(self.data.xpos[bid], dtype=np.float64))
        center = np.asarray(self.model.geom_pos[gid, :2], dtype=np.float64)
        half = np.asarray(self.model.geom_size[gid, :2], dtype=np.float64)
        delta = np.abs(local[:2] - center)
        return bool(delta[0] < half[0] - margin and delta[1] < half[1] - margin)

    def _sole(self, side: Side) -> float:
        return sole_clearance(self.model, self.data, self.bid[side], self.gid[side])

    def _slip(self, side: Side) -> float:
        xy = self._foot_xy(side)
        prev = self._prev_foot[side]
        if not self._have_prev:
            return 0.0
        return float(np.linalg.norm(xy - prev) / CTRL_DT)

    def knee_band_ratio(self) -> float:
        """Max |ctrl-q| / e_sat on the knees. Above 1 would be an overdrive."""
        worst = 0.0
        for jn in ("l_knee", "r_knee", "l_hip_pitch", "r_hip_pitch", "l_ank_pitch", "r_ank_pitch"):
            act = jn + "_pos"
            idx = self.act_idx.get(act)
            if idx is None:
                continue
            band = self.e_sat.get(act, 1.0)
            if band <= 1e-9:
                continue
            ratio = abs(float(self.data.ctrl[idx]) - self.q(jn)) / band
            worst = max(worst, ratio)
        return worst

    def sagittal_saturated(self) -> bool:
        for i in range(self.model.nu):
            name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
            if not any(tok in name for tok in ("hip_pitch", "knee", "ank_pitch", "hip_roll", "ank_roll")):
                continue
            if not name.startswith(("l_", "r_")):
                continue
            lim = abs(float(self.model.actuator_forcerange[i, 1]))
            if lim > 1e-9 and abs(float(self.data.actuator_force[i])) >= SAT_FRAC * lim:
                return True
        return False

    def _declare_stop_contact(self) -> None:
        """A swing label with both feet loaded is double support.

        The sample is taken after the physics step. A foot that was light
        when the clock advanced can be back on the floor, above 5 N, before
        the score reads the phase. The polygon follows that contact.
        """
        if self.preview_stage != "stop" or self.phase != "swing":
            return
        swing = self.other(self.stance)
        # 1 N is contact, not the 5 N weight gate. A swing foot at 4.9 N
        # is still on the floor. The score reads this phase after physics.
        if self.foot_normal(swing) > 1.0 and self.foot_normal(self.stance) >= float(self.cfg.unload_n):
            self.phase = "shift"

    def observe(self, up_z: float) -> None:
        self._declare_stop_contact()
        self._note_realised_split()
        swing = ""
        if self.phase == "swing":
            swing = self.other(self.stance)
        sl = self._slip("L")
        sr = self._slip("R")
        self.trace.t.append(float(self.data.time))
        self.trace.sole_l.append(self._sole("L"))
        self.trace.sole_r.append(self._sole("R"))
        self.trace.fn_l.append(self.foot_normal("L"))
        self.trace.fn_r.append(self.foot_normal("R"))
        self.trace.contact_l.append(1 if self.foot_contact("L") else 0)
        self.trace.contact_r.append(1 if self.foot_contact("R") else 0)
        self.trace.phase.append(self.phase)
        self.trace.swing.append(swing)
        self.trace.stance.append(self.stance)
        self.trace.up_z.append(float(up_z))
        self.trace.sat.append(1 if self.sagittal_saturated() else 0)
        self.trace.slip_l.append(sl)
        self.trace.slip_r.append(sr)
        self.trace.z_cmd.append(self.z_cmd)
        self.trace.z_bez.append(self.z_bez)
        self.trace.knee_band.append(self.knee_band_ratio())
        self.trace.sho_l.append(self.q("l_sho_pitch"))
        self.trace.sho_r.append(self.q("r_sho_pitch"))
        self._prev_foot["L"] = self._foot_xy("L").copy()
        self._prev_foot["R"] = self._foot_xy("R").copy()
        self._have_prev = True

    def score(self) -> dict[str, float | int | bool | str]:
        return score_lipm(self)


def sole_corners(model: mj.MjModel, data: mj.MjData, gid: int) -> np.ndarray:
    """Eight corners of the contact box, in the geom frame the collision uses.

    World z is measured from the floor plane at 0. ``bid`` is not used:
    a body-frame bottom face misses a geom whose quat is not the body's.
    """
    half = np.asarray(model.geom_size[gid], dtype=np.float64)
    pos = np.asarray(data.geom_xpos[gid], dtype=np.float64)
    rot = np.asarray(data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
    pts: list[np.ndarray] = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                local = np.array(
                    [sx * half[0], sy * half[1], sz * half[2]],
                    dtype=np.float64,
                )
                pts.append(pos + rot @ local)
    return np.stack(pts, axis=0)


def sole_clearance(model: mj.MjModel, data: mj.MjData, bid: int, gid: int) -> float:
    """Lowest of the eight contact-box corners above the floor plane z=0.

    ``bid`` stays in the signature so older callers compile. The corners
    come from ``geom_xpos`` and ``geom_xmat`` on this ``data``.
    """
    del bid
    return float(np.min(sole_corners(model, data, gid)[:, 2]))


def kit_bus_step(vx: float, yaw_rate: float, period_s: float) -> tuple[float, float]:
    """OP3 step length (m) and cycle yaw (rad) from a clamped bus command.

    ``vx`` is m/s and ``yaw_rate`` is rad/s, after Controls clamps them.
    There is no vy. ``x_amp`` tracks ``vx`` so the command is a speed, not
    a switch into the full 0.020 m step. The cycle angle is the heading
    change of one period. The walker applies half of it on each foot.
    """
    x_amp = 0.0
    per = KIT_BODY_PER_X if float(vx) >= 0.0 else KIT_BODY_PER_X_REV
    if abs(vx) > 1e-4 and per > 1e-9:
        mag = min(KIT_X_RAIL_M, abs(float(vx)) / per)
        x_amp = math.copysign(mag, float(vx))
    angle = float(yaw_rate) * float(period_s) * KIT_YAW_GAIN
    return x_amp, angle


def _median(values: list[float]) -> float:
    if not values:
        return 0.0
    arr = np.asarray(values, dtype=np.float64)
    return float(np.median(arr))


def score_lipm(walker: LipmWalker) -> dict[str, float | int | bool | str]:
    tr = walker.trace
    soles: list[float] = []
    contacts: list[float] = []
    slips: list[float] = []
    n_swing = 0
    n_sat_swing = 0
    for i, phase in enumerate(tr.phase):
        if phase != "swing":
            # Stance / shift slip of a foot that is down.
            if tr.contact_l[i]:
                slips.append(tr.slip_l[i])
            if tr.contact_r[i]:
                slips.append(tr.slip_r[i])
            continue
        n_swing += 1
        if tr.sat[i]:
            n_sat_swing += 1
        if tr.swing[i] == "L":
            soles.append(tr.sole_l[i])
            contacts.append(float(tr.contact_l[i]))
            if tr.contact_r[i]:
                slips.append(tr.slip_r[i])
        elif tr.swing[i] == "R":
            soles.append(tr.sole_r[i])
            contacts.append(float(tr.contact_r[i]))
            if tr.contact_l[i]:
                slips.append(tr.slip_l[i])
    opened = [rec for rec in walker.lifts if rec.clear_cmd_m > 0.0]
    missed = [rec for rec in walker.lifts if rec.clear_cmd_m <= 0.0]
    margins = [rec.cop_margin_m for rec in opened]
    cop_xs = [rec.cop_x_m for rec in opened if math.isfinite(rec.cop_x_m)]
    lift_shares = [rec.rear_share for rec in opened]
    shares: list[float] = []
    for i, phase in enumerate(tr.phase):
        if phase != "shift":
            continue
        if tr.stance[i] == "L":
            rear, front = tr.fn_r[i], tr.fn_l[i]
        else:
            rear, front = tr.fn_l[i], tr.fn_r[i]
        total = rear + front
        if total > 1.0:
            shares.append(rear / total)
    rear_share_min = min(shares) if shares else 1.0
    n_tick = max(1, len(tr.t))
    n_move = sum(1 for phase in tr.phase if phase != "stand")
    return {
        "name": walker.cfg.name,
        "schedule": walker.cfg.schedule,
        "clear_cmd_m": walker.cfg.clear_m,
        "gm_period_s": walker.cfg.gm_period_s,
        "gm_x_m": walker.cfg.gm_x_m,
        "gm_z_m": walker.cfg.gm_z_m,
        "gm_dsp": walker.cfg.gm_dsp,
        "arms": walker.cfg.arms,
        "n_ticks": len(tr.t),
        "n_swing_ticks": n_swing,
        "n_lifts": len(opened),
        "n_missed_gates": len(missed),
        "sole_median_m": _median(soles),
        "sole_p90_m": float(np.percentile(np.asarray(soles, dtype=np.float64), 90)) if soles else 0.0,
        "swing_contact_frac": float(np.mean(contacts)) if contacts else 1.0,
        "stance_slip_m_s": _median(slips),
        "sat_rate": (n_sat_swing / n_swing) if n_swing else 0.0,
        "sat_rate_all": float(sum(tr.sat) / n_tick),
        "cop_before_lift_median_m": _median(margins),
        "cop_before_lift_min_m": min(margins) if margins else -1.0,
        "cop_x_before_lift_m": _median(cop_xs),
        "rear_share_min": rear_share_min,
        "rear_unload_frac": 1.0 - rear_share_min,
        "rear_share_at_lift": _median(lift_shares),
        "dx_m": float(walker.data.qpos[0] - walker.x0),
        "lifts_com_inside_frac": (
            sum(1 for rec in opened if rec.com_inside) / len(opened) if opened else 0.0
        ),
        "min_up_z": min(tr.up_z) if tr.up_z else 1.0,
        "knee_band_max": max(tr.knee_band) if tr.knee_band else 0.0,
        "z_limited": walker.z_limited,
        "arm_ptp_rad": _arm_ptp(tr),
        "move_ticks": n_move,
    }


def _arm_ptp(tr: LipmTrace) -> float:
    if len(tr.sho_l) < 2:
        return 0.0
    left = max(tr.sho_l) - min(tr.sho_l)
    right = max(tr.sho_r) - min(tr.sho_r)
    return float(max(left, right))


def _check_bezier() -> None:
    p0 = np.array([0.0, 0.0, 0.01], dtype=np.float64)
    p3 = np.array([0.04, 0.0, 0.01], dtype=np.float64)
    peak = bezier_foot(0.5, p0, p3, 0.04)
    if abs(float(peak[2]) - bezier_peak_z(0.01, 0.04)) > 1e-9:
        raise SystemExit("bezier peak is not the commanded clearance")
    if abs(float(peak[2]) - 0.05) > 1e-9:
        raise SystemExit(f"bezier peak {peak[2]} != 0.05")


if __name__ == "__main__":
    _check_bezier()
    print("bezier peak ok")
