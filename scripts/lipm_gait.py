#!/usr/bin/env python3
"""LIPM / ZMP outer loop, Raibert footholds, Bézier swing IK.

Replaces the open-loop CPG as the schedule. CSF50 numbers are not the clock
and are not swept here. Joint targets go to the existing 50 Hz position
servos. Commands are clipped to the linear band of those servos
(|ctrl-q| <= 0.98 * tau / kp) so a knee target past ~0.046 rad does not
pretend to add torque. Nothing in this file writes the plant: no forcerange,
kp, damping, or armature edits, and no free-joint wrench.

Weight moves onto the stance foot before the swing foot is allowed to rise.
The swing is a joint-space Bézier measured on this plant: knee flexion for
about 2 cm of level-sole clearance, hip pitch for a landing at most 2 cm
ahead. The ankle target is the hip and knee commands actually sent this
tick (the flat-foot sum), plus a measured-normal trim, so the box does not
ride a corner. Commands stay inside |ctrl-q| <= 0.98 * tau / kp.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Literal

import mujoco as mj
import numpy as np

import gait_manager_traj as gm

CTRL_DT = 0.02
G = 9.81
LEG_TAU = 2.45
# Position-servo saturation boundary. Commanding past this does not add torque.
SAT_FRAC = 0.98
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
    gm_arm_deg: float = gm.GM_ARM_DEG
    # A loaded swing foot is not dragged along x. The z target still rises.
    gm_drag_gate: bool = False
    # Hold the published step height through the first half of single support
    # so the servo can arrive. The sine peaks for one sample and the knee
    # never gets there. Off is the raw wSin track.
    gm_z_hold: bool = False
    # Fraction of single support that holds the step height before descent.
    gm_hold_u: float = 0.55
    # Stance hip from the published x. 0.070 rad is the largest extension
    # that stayed upright for 24 s on this plant. A larger value is the
    # unclipped vendor x and it is allowed to tip so the number is visible.
    gm_stance_max: float = 0.48


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
        self.data = data
        self.act_idx = act_idx
        self.cfg = cfg
        self.q_stand = dict(q_stand)
        self.phase: PhaseName = "stand"
        self.stance: Side = "L"
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
        self.gm_clock = gm.GaitManagerClock(
            period_s=cfg.gm_period_s,
            dsp=cfg.gm_dsp,
            y_swap=cfg.gm_y_swap_m,
            z_move=cfg.gm_z_m,
            z_swap=cfg.gm_z_swap_m,
            step_fb=cfg.gm_step_fb,
            pelvis_deg=cfg.gm_pelvis_deg,
        )
        self._gm_swing: Side | None = None

    def other(self, side: Side) -> Side:
        return "R" if side == "L" else "L"

    def pref(self, side: Side) -> str:
        return "l_" if side == "L" else "r_"

    def q(self, jn: str) -> float:
        jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, jn)
        return float(self.data.qpos[self.model.jnt_qposadr[jid]])

    def write_clipped(self, jn: str, q_des: float) -> None:
        """Position target inside the servo's torque band. Never past it."""
        act = f"{jn}_pos"
        idx = self.act_idx.get(act)
        if idx is None:
            return
        band = self.e_sat.get(act, 0.0)
        q = self.q(jn)
        cmd = min(q + band, max(q - band, q_des))
        lo = float(self.model.actuator_ctrlrange[idx, 0])
        hi = float(self.model.actuator_ctrlrange[idx, 1])
        self.data.ctrl[idx] = min(hi, max(lo, cmd))

    def hold_stand(self) -> None:
        self.phase = "stand"
        self.phase_t = 0.0
        self.swing_s = 0.0
        self.lat *= 0.8
        for jn, val in self.q_stand.items():
            self.write_clipped(jn, val)

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

    def _tick_gait_manager(self, walking: bool) -> None:
        """Published GaitManager endpoints, same 50 Hz position servos.

        Foot height is the swing-minus-stance gap (about 2 cm on the demo
        preset), not a 2 cm cartesian lift of one foot from the floor. Lateral
        sway is hip roll. The 15 deg hip-pitch offset is their init pose and
        is not stacked on this stand.
        """
        clock = self.gm_clock
        if not walking:
            clock.set_command(0.0, 0.0, 0.0)
            clock.time = 0.0
            clock.previous_x = 0.0
            clock._update_movement()
            clock._capture_zero()
            self._gm_swing = None
            self.hold_stand()
            return
        x_amp = 0.0
        if abs(self.cmd_vx) > 1e-4:
            x_amp = math.copysign(self.cfg.gm_x_m, self.cmd_vx)
        yaw_deg = 0.0
        if abs(self.cmd_yaw) > 1e-4:
            yaw_deg = math.copysign(
                min(10.0, abs(math.degrees(self.cmd_yaw)) / 0.25 * 10.0),
                self.cmd_yaw,
            )
        clock.set_command(x_amp, 0.0, yaw_deg)
        left, right, phase = clock.advance(CTRL_DT)
        shifts = {"L": left, "R": right}
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
                    z_budget_m=0.0,
                )
            )
        self._gm_swing = swing if phase in ("L", "R") else None
        dy = 0.5 * (left.y + right.y)
        # +y puts both feet to the left of the pelvis, so the pelvis sits
        # over the right foot. lat > 0 is the measured lean onto the left.
        lat = max(-1.0, min(1.0, (-dy / 0.22) / SHIFT_HIP_L))
        self.lat = lat
        # The published z is a foot-to-foot gap. Map that gap through the
        # joint Bézier measured on this plant (0.62 rad knee clears ~2 cm)
        # instead of a cartesian target the stance foot cannot follow
        # through the floor. x maps through the same hip scale (0.24 rad
        # per 2 cm). A loaded swing foot keeps its current x.
        z_floor = min(left.z, right.z)
        for side in ("L", "R"):
            shift = shifts[side]
            lift = max(0.0, shift.z - z_floor)
            if self.cfg.gm_z_hold and phase == side:
                if phase == "L":
                    u = (clock.time - clock.l_ssp_start) / max(1e-3, clock.l_ssp_end - clock.l_ssp_start)
                else:
                    u = (clock.time - clock.r_ssp_start) / max(1e-3, clock.r_ssp_end - clock.r_ssp_start)
                hold_u = min(0.92, max(0.2, self.cfg.gm_hold_u))
                if u < hold_u:
                    lift = self.cfg.gm_z_m
                else:
                    lift = self.cfg.gm_z_m * max(0.0, (1.0 - u) / max(1e-3, 1.0 - hold_u))
            use_x = shift.x
            if (
                self.cfg.gm_drag_gate
                and phase == side
                and self.foot_normal(side) > self.cfg.unload_n
            ):
                use_x = 0.0
            scale_h = lift / VENDOR_CLEAR_M
            scale_x = use_x / VENDOR_STEP_M
            flex = max(0.0, scale_h) * FLEX_PEAK
            dhip = scale_x * HIP_LAND + max(0.0, scale_h) * HIP_RISE
            if phase != side:
                dhip = max(-self.cfg.gm_stance_max, min(self.cfg.gm_stance_max, dhip))
                flex = 0.0
            self._write_leg_delta(side, flex, dhip, shifts[side].yaw)
        sl = self.q_stand.get("l_hip_roll", -0.05)
        sr = self.q_stand.get("r_hip_roll", 0.05)
        al = self.q_stand.get("l_ank_roll", 0.0)
        ar = self.q_stand.get("r_ank_roll", 0.0)
        self.write_clipped("l_hip_roll", sl + SHIFT_HIP_L * lat + left.pelvis_roll)
        self.write_clipped("r_hip_roll", sr + SHIFT_HIP_R * lat + right.pelvis_roll)
        self.write_clipped("l_ank_roll", al + SHIFT_ANK * lat)
        self.write_clipped("r_ank_roll", ar + SHIFT_ANK * lat)
        if phase in ("L", "R"):
            self._level_swing_roll(swing)
        self.z_bez = float(shifts[swing].z - shifts[self.stance].z) if phase in ("L", "R") else 0.0
        self.z_cmd = self.z_bez
        self._write_gm_arms(x_amp)
        self._write_unused()

    def _write_leg_delta(self, side: Side, flex: float, dhip: float, yaw: float) -> None:
        """Stand pose plus a Bézier flex and a forward hip. Clipped to tau/kp."""
        pref = self.pref(side)
        hip_sign = -1.0 if side == "L" else 1.0
        knee_sign = 1.0 if side == "L" else -1.0
        hip0 = self.q_stand.get(pref + "hip_pitch", 0.0)
        knee0 = self.q_stand.get(pref + "knee", 0.0)
        self.write_clipped(pref + "hip_pitch", hip0 + hip_sign * dhip)
        self.write_clipped(pref + "knee", knee0 + knee_sign * flex)
        hip_c = float(self.data.ctrl[self.act_idx[pref + "hip_pitch_pos"]])
        knee_c = float(self.data.ctrl[self.act_idx[pref + "knee_pos"]])
        ank = hip_c + knee_c
        up = self.data.xmat[self.bid[side]].reshape(3, 3)[:, 2]
        sign = 1.0 if side == "L" else -1.0
        ank += sign * float(up[0])
        self.write_clipped(pref + "ank_pitch", ank)
        self.write_clipped(pref + "hip_yaw", self.q_stand.get(pref + "hip_yaw", 0.0) + yaw)

    def _write_gm_arms(self, x_amp: float) -> None:
        for jn in (
            "l_sho_roll", "r_sho_roll", "l_el_pitch", "r_el_pitch",
            "l_el_yaw", "r_el_yaw", "l_gripper", "r_gripper",
            "l_sho_pitch", "r_sho_pitch",
        ):
            self.write_clipped(jn, self.q_stand.get(jn, 0.0))
        if not self.cfg.arms or abs(x_amp) < 1e-6:
            return
        # set_step stores arm_swing_gain = radians(arm_swap). The OP2 arm
        # formula then scales by x_move * gain * 1000 * deg2rad.
        gain = math.radians(self.cfg.gm_arm_deg)
        period = max(self.gm_clock.period, 1e-3)
        mag = x_amp * gain * 1000.0 * (math.pi / 180.0)
        right = gm.wsin(self.gm_clock.time, period, math.pi * 1.5, -mag, 0.0)
        left = gm.wsin(self.gm_clock.time, period, math.pi * 1.5, mag, 0.0)
        self.write_clipped("r_sho_pitch", self.q_stand.get("r_sho_pitch", 0.0) + right)
        self.write_clipped("l_sho_pitch", self.q_stand.get("l_sho_pitch", 0.0) - left)

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
        2 cm Bézier on this plant. write_clipped keeps every command inside
        tau/kp, so a taller clear scales the shape without overdriving.
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

    def foot_contact(self, side: Side) -> bool:
        bid = self.bid[side]
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            b1 = int(self.model.geom_bodyid[g1])
            b2 = int(self.model.geom_bodyid[g2])
            if (b1 == bid or b2 == bid) and (g1 == self.gid_floor or g2 == self.gid_floor):
                return True
        return False

    def foot_normal(self, side: Side) -> float:
        """Floor normal only. Overlapping 135 mm soles collide with each other
        on a 2 cm step; that foot-foot force is not weight on the rear foot.
        """
        gid = self.gid[side]
        floor = self.gid_floor
        total = 0.0
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if not ((g1 == gid and g2 == floor) or (g2 == gid and g1 == floor)):
                continue
            force = np.zeros(6, dtype=np.float64)
            mj.mj_contactForce(self.model, self.data, i, force)
            total += float(force[0])
        return total

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

    def observe(self, up_z: float) -> None:
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


def sole_clearance(model: mj.MjModel, data: mj.MjData, bid: int, gid: int) -> float:
    """Lowest sole corner above the floor plane z=0."""
    pos = np.asarray(model.geom_pos[gid], dtype=np.float64)
    half = np.asarray(model.geom_size[gid], dtype=np.float64)
    rot = data.xmat[bid].reshape(3, 3)
    origin = np.asarray(data.xpos[bid], dtype=np.float64)
    zs: list[float] = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            local = np.array(
                [pos[0] + sx * half[0], pos[1] + sy * half[1], pos[2] - half[2]],
                dtype=np.float64,
            )
            zs.append(float((origin + rot @ local)[2]))
    return min(zs)


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
