#!/usr/bin/env python3
"""ROBOTIS OP3 walking module, retargeted to this plant's leg lengths.

The trajectory and the analytic leg IK are the Apache-2.0
``op3_walking_module`` / ``op3_kinematics_dynamics`` (ROBOTIS-GIT/ROBOTIS-OP3,
copyright 2017 ROBOTIS Co., Ltd.). Hiwonder's ``walking_module.so`` uses the
same field names (``x_swap_amplitude_``, ``dsp_ratio_``, ``pelvis_swing_``,
``hit_pitch_offset_``, ``l_ssp_start_time_``, ``phase1_time_``,
``arm_swing_gain_``). The aarch64 Cython extension is not loaded.

Limb lengths are read from the loaded plant (thigh, calf, sole drop). They
are not the OP3's 110 mm / 110 mm / 30.5 mm links. Kit ``init_z_offset`` is
0.025 m (the cartesian body drop). The 0.018 m sole-vs-hip stance is not
used. Kit ``init_y`` is outward-positive: +0.005 m on each foot. Kit
``hip_pitch_offset`` of 15° is on the stand and the walk.

The OP3 module steps at 8 ms (``control_cycle_msec_``). This port uses that
period. The plant timestep stays 0.002 s. Balance is off. No gyro feedback,
no gain schedule, no plant edit.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco as mj
import numpy as np

# Hiwonder walking_param / gait_control_demo, not the OP3 yaml defaults.
STEP_FB_RATIO = 0.028
Z_SWAP_M = 0.006
PELVIS_DEG = 5.0
# walking_param.yaml arm_swing_gain. OP3 multiplies x_move * gain * 1000
# and treats that product as degrees. 0.5 at x = 0.02 m is 10°.
ARM_SWING_GAIN = 0.5
# Hiwonder init_y, outward-positive. Each foot's hip-frame y moves
# 0.005 m outward. OP3 applies ±y_offset/2, so the stored offset is
# twice that. The 0.018 m sole-vs-hip hack is not the live stance.
KIT_Y_OUT_M = 0.005
# walking_param.yaml hip_pitch_offset. On the stand and the walk.
# 15° moves this plant's right hip pitch from +0.504 rad to +0.766 rad.
HIP_PITCH_OFFSET_DEG = 15.0
# WalkingModule::control_cycle_msec_. Four plant steps of 0.002 s.
OP3_CTRL_S = 0.008
# Frozen hip-roll position gain on this plant. Not written. The 1.29 Nm
# feed-forward (1.29/40 rad) was measured and does not move the COM; it is
# not applied. See the vendor note.
HIP_ROLL_KP = 40.0
# Published move(1..4): period s, dsp, y_swap m. x amp and step height are
# the same on every gear: 0.02 m and 0.02 m. Kit body drop is 0.025 m.
# Prefer FAIL floor for this draft is 400 ms and slower. 300 ms stays in
# the published table and is not the success bar. Armature is not a peel.
KIT_PRESETS: tuple[tuple[float, float, float], ...] = (
    (0.300, 0.20, 0.020),
    (0.400, 0.20, 0.020),
    (0.500, 0.20, 0.020),
    (0.600, 0.10, 0.040),
)

_LEG_R = ("r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll")
_LEG_L = ("l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll")


@dataclass(frozen=True)
class LegLengths:
    """Analytic chain. Metres and radians, from the plant bodies."""

    thigh_m: float
    calf_m: float
    ankle_m: float
    hip_pitch_offset_m: float
    hip_offset_angle_rad: float


@dataclass(frozen=True)
class StepInfo:
    phase: str
    swap_y_m: float
    ep_r: tuple[float, float, float]
    ep_l: tuple[float, float, float]
    ik_ok: bool


def _rot_x(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def _rot_y(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _rot_z(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def _sign(value: float) -> float:
    return 1.0 if value >= 0.0 else -1.0


def wsin(time: float, period: float, period_shift: float, mag: float, mag_shift: float) -> float:
    """``WalkingModule::wSin``. Period is seconds."""
    if period <= 1e-9:
        return mag_shift
    return mag * math.sin(2.0 * math.pi / period * time - period_shift) + mag_shift


def _axis_sum(model: mj.MjModel, joint: str) -> float:
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint)
    if jid < 0:
        raise RuntimeError(f"missing joint {joint}")
    axis = np.asarray(model.jnt_axis[jid], dtype=np.float64)
    return float(axis[0] + axis[1] + axis[2])


def y_offset_from_model(model: mj.MjModel) -> float:
    """Retired sole-vs-hip width. The live stance is ``2 * KIT_Y_OUT_M``.

    ``init_y_offset`` that puts each sole's inboard edge on the midline.

    Hip yaw sits at ±hip_y. The sole half-width is larger, so ``y_offset``
    of 0 places both contact patches across the center and the solver parks
    the whole weight on one foot. OP3 applies ±y_offset/2 in the hip frame,
    which is a full ``y_offset`` of extra stance width.
    """
    r_hip = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "r_hip_yaw_link")
    l_hip = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "l_hip_yaw_link")
    foot = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "r_foot_contact")
    if min(r_hip, l_hip, foot) < 0:
        raise RuntimeError("plant is missing hip yaw or the foot sole")
    hip_y = 0.5 * (float(model.body_pos[l_hip][1]) - float(model.body_pos[r_hip][1]))
    half_y = float(model.geom_size[foot][1])
    return 2.0 * (half_y - hip_y)


def lengths_from_model(model: mj.MjModel) -> LegLengths:
    """Thigh, calf, and sole drop from the plant bodies. Not the OP3 links.

    OP3 sets thigh from the knee-link xz distance, calf from the ankle-pitch
    z, and ankle length from the foot-end z. The same three are read here.
    The hip-roll drop above the hip pitch is outside that chain, as it is
    on the OP3 model.
    """
    knee = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "r_knee_link")
    ank = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "r_ank_pitch_link")
    foot = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "r_foot_contact")
    if min(knee, ank, foot) < 0:
        raise RuntimeError("plant is missing the right-leg chain")
    knee_pos = np.asarray(model.body_pos[knee], dtype=np.float64)
    calf = abs(float(model.body_pos[ank][2]))
    sole_z = float(model.geom_pos[foot][2] - model.geom_size[foot][2])
    ankle = abs(sole_z)
    thigh = float(math.hypot(float(knee_pos[0]), float(knee_pos[2])))
    hip_x = float(knee_pos[0])
    return LegLengths(
        thigh_m=thigh,
        calf_m=calf,
        ankle_m=ankle,
        hip_pitch_offset_m=hip_x,
        hip_offset_angle_rad=math.atan2(hip_x, abs(float(knee_pos[2]))),
    )


def ik_leg(
    lengths: LegLengths,
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> np.ndarray | None:
    """``calcInverseKinematicsForLeg``. Returns 6 angles before joint signs.

    Order is hip yaw, hip roll, hip pitch, knee, ankle pitch, ankle roll.
    """
    r06 = _rpy(roll, pitch, yaw)
    p06 = np.array([x, y, z], dtype=np.float64) + lengths.ankle_m * r06[:, 2]
    p60 = -r06.T @ p06
    out = np.zeros(6, dtype=np.float64)
    out[5] = math.atan2(float(p60[1]), float(p60[2]))
    r05 = r06 @ _rot_x(-float(out[5]))
    out[0] = math.atan2(-float(r05[0, 1]), float(r05[1, 1]))
    p03 = _rot_z(float(out[0])) @ np.array(
        [lengths.hip_pitch_offset_m, 0.0, 0.0], dtype=np.float64
    )
    p36 = p06 - p03
    dist = float(np.linalg.norm(p36))
    if dist < 1e-8:
        return None
    cos_knee = (
        lengths.thigh_m * lengths.thigh_m + lengths.calf_m * lengths.calf_m - dist * dist
    ) / (2.0 * lengths.thigh_m * lengths.calf_m)
    if abs(cos_knee) > 1.0 + 1e-6:
        return None
    cos_knee = max(-1.0, min(1.0, cos_knee))
    out[3] = -math.acos(cos_knee) + math.pi
    sin_arg = lengths.thigh_m * math.sin(math.pi - float(out[3])) / dist
    if abs(sin_arg) > 1.0 + 1e-6:
        return None
    sin_arg = max(-1.0, min(1.0, sin_arg))
    alpha = math.asin(sin_arg)
    p63 = -r06.T @ p36
    horiz = math.sqrt(float(p63[1]) * float(p63[1]) + float(p63[2]) * float(p63[2]))
    out[4] = -math.atan2(float(p63[0]), _sign(float(p63[2])) * horiz) - alpha
    r13 = _rot_z(-float(out[0])) @ r05 @ _rot_y(-(float(out[4]) + float(out[3])))
    out[1] = math.atan2(float(r13[2, 1]), float(r13[1, 1]))
    out[2] = math.atan2(float(r13[0, 2]), float(r13[0, 0]))
    out[2] += lengths.hip_offset_angle_rad
    out[3] -= lengths.hip_offset_angle_rad
    if not np.all(np.isfinite(out)):
        return None
    return out


class Op3Walker:
    """One walking cycle. ``step`` returns plant joint targets in radians."""

    def __init__(
        self,
        lengths: LegLengths,
        directions: dict[str, float],
        *,
        period_s: float,
        dsp: float,
        y_swap_m: float,
        z_move_m: float,
        x_amp_m: float,
        z_offset_m: float,
        z_swap_m: float = Z_SWAP_M,
        step_fb: float = STEP_FB_RATIO,
        pelvis_deg: float = PELVIS_DEG,
        arm_swing_gain: float = ARM_SWING_GAIN,
        hip_pitch_deg: float = HIP_PITCH_OFFSET_DEG,
    ) -> None:
        self.lengths = lengths
        self.directions = directions
        self.period_cmd = float(period_s)
        self.dsp_cmd = float(dsp)
        self.y_swap_cmd = float(y_swap_m)
        self.z_move_cmd = float(z_move_m)
        self.x_cmd = float(x_amp_m)
        self.y_cmd = 0.0
        self.angle_cmd = 0.0
        self.z_offset = float(z_offset_m)
        self.y_offset = 0.0
        self.x_offset = 0.0
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.yaw_offset = 0.0
        # Kit trim, stand and walk. See the module note.
        self.hit_pitch_offset = math.radians(hip_pitch_deg)
        self.z_swap_cmd = float(z_swap_m)
        self.step_fb = float(step_fb)
        self.pelvis_offset = math.radians(pelvis_deg)
        self.arm_swing_gain = float(arm_swing_gain)
        self.time = 0.0
        self.previous_x = 0.0
        self.ctrl_running = False
        # Remaining joint offset after a lead change. The clock may sit on
        # the other double support, but the published pose starts where it
        # was and catches up no faster than a normal walk tick.
        self._lead_err: dict[str, float] | None = None
        self._lead_prev_canon: dict[str, float] | None = None
        self._lead_cap = 0.0
        self._last_joints: dict[str, float] | None = None
        self.lead_blend_tick = False
        self.lead_carry_tick = False
        self.lead_blend_s = 0.0
        self._x_move = 0.0
        self._x_swap = 0.0
        self._y_move = 0.0
        self._y_move_shift = 0.0
        self._y_swap = 0.0
        self._z_move = 0.0
        self._z_move_shift = 0.0
        self._z_swap = 0.0
        self._z_swap_shift = 0.0
        self._a_move = 0.0
        self._a_move_shift = 0.0
        self.period = self.period_cmd
        self.pelvis_swing = 0.0
        self.l_ssp_start = 0.0
        self.l_ssp_end = 0.0
        self.r_ssp_start = 0.0
        self.r_ssp_end = 0.0
        self.phase1 = 0.0
        self.phase2 = 0.0
        self.phase3 = 0.0
        self.x_swap_period = 1.0
        self.x_move_period = 1.0
        self.y_swap_period = 1.0
        self.y_move_period = 1.0
        self.z_swap_period = 1.0
        self.z_move_period = 1.0
        self.a_move_period = 1.0
        self.update_time()
        self.update_movement()

    @classmethod
    def from_model(cls, model: mj.MjModel, **kwargs: float) -> Op3Walker:
        directions = {name: _axis_sum(model, name) for name in _LEG_R + _LEG_L}
        directions["r_sho_pitch"] = _axis_sum(model, "r_sho_pitch")
        directions["l_sho_pitch"] = _axis_sum(model, "l_sho_pitch")
        walker = cls(lengths_from_model(model), directions, **kwargs)
        # ±y_offset/2 in the hip frame. +KIT_Y_OUT_M on each foot.
        walker.y_offset = 2.0 * KIT_Y_OUT_M
        return walker

    def update_time(self) -> None:
        period = self.period_cmd
        dsp = min(0.95, max(0.0, self.dsp_cmd))
        ssp = 1.0 - dsp
        self.period = period
        self.x_swap_period = period / 2.0
        self.x_move_period = period * ssp
        self.y_swap_period = period
        self.y_move_period = period * ssp
        self.z_swap_period = period / 2.0
        self.z_move_period = period * ssp / 2.0
        self.a_move_period = period * ssp
        self.l_ssp_start = (1.0 - ssp) * period / 4.0
        self.l_ssp_end = (1.0 + ssp) * period / 4.0
        self.r_ssp_start = (3.0 - ssp) * period / 4.0
        self.r_ssp_end = (3.0 + ssp) * period / 4.0
        self.phase1 = (self.l_ssp_start + self.l_ssp_end) / 2.0
        self.phase2 = (self.l_ssp_end + self.r_ssp_start) / 2.0
        self.phase3 = (self.r_ssp_start + self.r_ssp_end) / 2.0
        self.pelvis_swing = self.pelvis_offset * 0.35

    def update_movement(self) -> None:
        x = self.x_cmd
        x_swap = x * self.step_fb
        if self.previous_x == 0.0:
            x *= 0.5
            x_swap *= 0.5
        self._x_move = x
        self._x_swap = x_swap
        y = self.y_cmd / 2.0
        self._y_move = y
        self._y_move_shift = y if y > 0.0 else -y
        self._y_swap = self.y_swap_cmd + self._y_move_shift * 0.04
        # OP3 updateMovementParam: z_move_amplitude_ = foot_height / 2, and
        # the shift is half of that again. wSin then runs from −amp to
        # +1.5·amp. The stance foot is frozen at the negative peak, so the
        # swing-to-stance gap is 2·amp, which is the yaml foot_height.
        # gait_manager 0.02 m therefore already commands a 2 cm sole.
        # Dropping the /2, or feeding 0.04 to "undo" it, commands ~4 cm.
        self._z_move = self.z_move_cmd / 2.0
        self._z_move_shift = self._z_move / 2.0
        self._z_swap = self.z_swap_cmd
        self._z_swap_shift = self.z_swap_cmd
        a = self.angle_cmd / 2.0
        self._a_move = a
        self._a_move_shift = a if a > 0.0 else -a

    def set_command(self, x_amp: float, y_amp: float = 0.0, angle_rad: float = 0.0) -> None:
        self.x_cmd = float(x_amp)
        self.y_cmd = float(y_amp)
        self.angle_cmd = float(angle_rad)

    def stop(self) -> None:
        self.ctrl_running = False
        self.x_cmd = 0.0
        self.y_cmd = 0.0
        self.angle_cmd = 0.0
        self.time = 0.0
        self.previous_x = 0.0
        self._lead_err = None
        self._lead_prev_canon = None
        self._lead_cap = 0.0
        self._last_joints = None
        self.lead_blend_tick = False
        self.lead_carry_tick = False
        self.lead_blend_s = 0.0
        self.update_movement()

    def next_swing(self) -> str | None:
        """``L`` or ``R`` when the clock is in the double support before that swing.

        ``None`` while a foot is already in single support. The cycle at
        time 0 is the double support before the left swing.
        """
        t = self.time
        if t <= self.l_ssp_start or t > self.r_ssp_end:
            return "L"
        if self.l_ssp_end < t <= self.r_ssp_start:
            return "R"
        return None

    def arm_swing(self, swing: str) -> None:
        """Line the next swing up with ``swing`` without a one-tick pose snap.

        ``L`` lands on time 0. ``R`` lands on the double support between
        the two single supports.         A small gap moves the clock now. A large gap still parks the
        clock on that double support, so the next swing is the requested
        foot, but the published joints start from the pose just sent and
        chase the live pose. The first tick closes at most one normal
        walk step. Later ticks follow the canonical step when that step
        is already larger, and do not add a second jump on top of it.
        Does not change a step length, a gain, or the plant.
        """
        if swing not in ("L", "R"):
            raise ValueError(f"swing lead must be L or R, got {swing}")
        self.update_movement()
        target = 0.0 if swing == "L" else float(self.phase2)
        q_here = self._last_joints if self._last_joints is not None else self._joints_at(self.time)
        q_there = self._joints_at(target)
        if q_here is None or q_there is None:
            self.time = target
            self._lead_err = None
            self._lead_prev_canon = None
            return
        gap = self._max_abs_joint_delta(q_here, q_there)
        if gap <= 1e-4:
            self.time = target
            self._lead_err = None
            self._lead_prev_canon = None
            return
        cap = self._normal_tick_p95(OP3_CTRL_S)
        if gap <= cap + 1e-4:
            self.time = target
            self._lead_err = None
            self._lead_prev_canon = None
            return
        err = {k: float(q_there[k]) - float(q_here[k]) for k in q_here if k in q_there}
        self._lead_cap = max(float(cap), 1e-4)
        self._lead_err = err
        self._lead_prev_canon = None
        self.time = target

    def cancel_lead_morph(self) -> None:
        """A new yaw leaves the offset in place. Clearing it would snap."""
        return

    def _joints_at(self, t: float) -> dict[str, float] | None:
        saved = self.time
        self.time = float(t)
        joints, _info = self.joints_now()
        self.time = saved
        if joints is None:
            return None
        return dict(joints)

    def _max_abs_joint_delta(self, a: dict[str, float], b: dict[str, float]) -> float:
        keys = set(a) & set(b)
        if not keys:
            return 0.0
        return max(abs(float(a[k]) - float(b[k])) for k in keys)

    def _normal_tick_p95(self, dt: float) -> float:
        """p95 of max |Δq| over one cycle at the current step command.

        The sample uses the same phase update as ``step``. Amplitudes and
        the clock are restored afterward.
        """
        saved_time = self.time
        saved_prev = self.previous_x
        saved_run = self.ctrl_running
        saved_amp = (
            self._x_move, self._x_swap, self._y_move, self._y_move_shift,
            self._y_swap, self._z_move, self._z_move_shift, self._z_swap,
            self._z_swap_shift, self._a_move, self._a_move_shift,
        )
        self.time = 0.0
        self.ctrl_running = True
        deltas: list[float] = []
        prev: dict[str, float] | None = None
        n = max(1, int(round(self.period / max(dt, 1e-6))))
        for _ in range(n):
            self.process_phase(dt)
            joints, _info = self.joints_now()
            self.time += dt
            if self.time >= self.period - 1e-12:
                self.time = 0.0
                self.previous_x = self.x_cmd * 0.5
                self.update_movement()
            if joints is not None and prev is not None:
                deltas.append(self._max_abs_joint_delta(prev, joints))
            prev = None if joints is None else dict(joints)
        self.time = saved_time
        self.previous_x = saved_prev
        self.ctrl_running = saved_run
        (
            self._x_move, self._x_swap, self._y_move, self._y_move_shift,
            self._y_swap, self._z_move, self._z_move_shift, self._z_swap,
            self._z_swap_shift, self._a_move, self._a_move_shift,
        ) = saved_amp
        self.update_time()
        if not deltas:
            return 0.0
        return float(np.percentile(np.asarray(deltas, dtype=np.float64), 95))

    def _bleed_lead_err(self, joints: dict[str, float], phase: str) -> dict[str, float]:
        """Chase the live pose after a lead change, without outrunning a walk tick.

        The first tick closes at most the walk's p95 step. Later ticks may
        also follow the canonical step when that step is already larger,
        so the swing is not slowed, and they do not add a second jump on
        top of it. ``phase`` is unused; both feet chase the same clock.
        """
        del phase
        err = self._lead_err
        if not err:
            return joints
        prev_pub = self._last_joints or {}
        prev_canon = self._lead_prev_canon
        cap = self._lead_cap
        out: dict[str, float] = {}
        canon_now: dict[str, float] = {}
        for name, canon in joints.items():
            canon_f = float(canon)
            before = float(prev_pub.get(name, canon_f - float(err.get(name, 0.0))))
            if prev_canon is None:
                cstep = 0.0
            else:
                cstep = canon_f - float(prev_canon.get(name, canon_f))
            limit = max(cap, abs(cstep))
            gap = canon_f - before
            step = max(-limit, min(limit, gap))
            pub = before + step
            out[name] = pub
            canon_now[name] = canon_f
            if name in err:
                err[name] = canon_f - pub
        self._lead_prev_canon = canon_now
        if all(abs(float(v)) < 1e-4 for v in err.values()):
            self._lead_err = None
            self._lead_prev_canon = None
        return out

    def process_phase(self, dt: float) -> None:
        half = dt / 2.0
        if self.time == 0.0:
            self.update_time()
            if not self.ctrl_running and self.x_cmd == 0.0 and self.y_cmd == 0.0 and self.angle_cmd == 0.0:
                self.previous_x = 0.0
            self.update_movement()
        elif abs(self.time - self.phase1) <= half:
            self.update_movement()
            self.update_time()
            self.time = self.phase1
        elif abs(self.time - self.phase2) <= half:
            self.update_time()
            self.time = self.phase2
        elif abs(self.time - self.phase3) <= half:
            self.update_movement()
            self.update_time()
            self.time = self.phase3

    def _leg_move(self, t_x: float, t_z: float, phase_t: float, sign: float, extra_pi: bool) -> tuple[float, float, float, float]:
        extra = math.pi if extra_pi else 0.0
        x_phase = math.pi / 2.0 + 2.0 * math.pi / max(self.x_move_period, 1e-6) * phase_t + extra
        y_phase = math.pi / 2.0 + 2.0 * math.pi / max(self.y_move_period, 1e-6) * phase_t + extra
        z_phase = math.pi / 2.0 + 2.0 * math.pi / max(self.z_move_period, 1e-6) * phase_t
        a_phase = math.pi / 2.0 + 2.0 * math.pi / max(self.a_move_period, 1e-6) * phase_t + extra
        # One negation on the right leg (sign = −1). Negating the shift
        # again puts the same yaw on both feet. Hip yaw, hip roll, and
        # ankle roll share an axis across the legs, so that common mode
        # cancels one turn. Pitch stays mirrored in the joint directions.
        # z is not signed. The stance foot freezes t_z at the other SSP start.
        x = wsin(t_x, self.x_move_period, x_phase, sign * self._x_move, 0.0)
        y = wsin(t_x, self.y_move_period, y_phase, sign * self._y_move, sign * self._y_move_shift)
        z = wsin(t_z, self.z_move_period, z_phase, self._z_move, self._z_move_shift)
        yaw = wsin(t_x, self.a_move_period, a_phase, sign * self._a_move, sign * self._a_move_shift)
        return x, y, z, yaw

    def endpoints(self) -> tuple[np.ndarray, np.ndarray, float, float, float]:
        """Right and left foot (x, y, z, roll, pitch, yaw), plus pelvis rolls and swap y.

        z is already ``swap + move + z_offset - leg_length``, the OP3 hip frame.
        """
        t = self.time
        swap_x = wsin(t, self.x_swap_period, math.pi, self._x_swap, 0.0)
        swap_y = self._swap_y_before_swing(t)
        swap_z = wsin(t, self.z_swap_period, 1.5 * math.pi, self._z_swap, self._z_swap_shift)
        if t <= self.l_ssp_start:
            left = self._leg_move(self.l_ssp_start, self.l_ssp_start, self.l_ssp_start, 1.0, False)
            right = self._leg_move(self.l_ssp_start, self.r_ssp_start, self.l_ssp_start, -1.0, False)
            pel_l = 0.0
            pel_r = 0.0
        elif t <= self.l_ssp_end:
            left = self._leg_move(t, t, self.l_ssp_start, 1.0, False)
            right = self._leg_move(t, self.r_ssp_start, self.l_ssp_start, -1.0, False)
            pel_l = wsin(
                t, self.z_move_period,
                math.pi / 2.0 + 2.0 * math.pi / self.z_move_period * self.l_ssp_start,
                self.pelvis_swing / 2.0, self.pelvis_swing / 2.0,
            )
            pel_r = wsin(
                t, self.z_move_period,
                math.pi / 2.0 + 2.0 * math.pi / self.z_move_period * self.l_ssp_start,
                -self.pelvis_offset / 2.0, -self.pelvis_offset / 2.0,
            )
        elif t <= self.r_ssp_start:
            left = self._leg_move(self.l_ssp_end, self.l_ssp_end, self.l_ssp_start, 1.0, False)
            right = self._leg_move(self.l_ssp_end, self.r_ssp_start, self.l_ssp_start, -1.0, False)
            pel_l = 0.0
            pel_r = 0.0
        elif t <= self.r_ssp_end:
            left = self._leg_move(t, self.l_ssp_end, self.r_ssp_start, 1.0, True)
            right = self._leg_move(t, t, self.r_ssp_start, -1.0, True)
            pel_l = wsin(
                t, self.z_move_period,
                math.pi / 2.0 + 2.0 * math.pi / self.z_move_period * self.r_ssp_start,
                self.pelvis_offset / 2.0, self.pelvis_offset / 2.0,
            )
            pel_r = wsin(
                t, self.z_move_period,
                math.pi / 2.0 + 2.0 * math.pi / self.z_move_period * self.r_ssp_start,
                -self.pelvis_swing / 2.0, -self.pelvis_swing / 2.0,
            )
        else:
            left = self._leg_move(self.r_ssp_end, self.l_ssp_end, self.r_ssp_start, 1.0, True)
            right = self._leg_move(self.r_ssp_end, self.r_ssp_end, self.r_ssp_start, -1.0, True)
            pel_l = 0.0
            pel_r = 0.0
        # Right-leg z freeze uses r_ssp_start as the z phase anchor. The
        # helper above used phase_t for z as well. Rebuild those z terms so
        # the frozen foot matches computeLegAngle, which passes r_ssp_start
        # as both the sample time and the phase for the right z channel.
        right_z = self._right_z(t)
        left_z = self._left_z(t)
        leg = self.lengths.thigh_m + self.lengths.calf_m + self.lengths.ankle_m
        er = np.array([
            swap_x + right[0] + self.x_offset,
            swap_y + right[1] - self.y_offset / 2.0,
            swap_z + right_z + self.z_offset - leg,
            0.0 - self.roll_offset / 2.0,
            0.0 + self.pitch_offset,
            right[3] - self.yaw_offset / 2.0,
        ], dtype=np.float64)
        el = np.array([
            swap_x + left[0] + self.x_offset,
            swap_y + left[1] + self.y_offset / 2.0,
            swap_z + left_z + self.z_offset - leg,
            0.0 + self.roll_offset / 2.0,
            0.0 + self.pitch_offset,
            left[3] + self.yaw_offset / 2.0,
        ], dtype=np.float64)
        return er, el, pel_r, pel_l, swap_y

    def _swap_y_before_swing(self, t: float) -> float:
        """Lateral shift that is already at its peak when the foot lifts.

        The OP3 sine is ``A sin(2π t / T)``. It is 0 at t = 0 and peaks at
        t = T/4, which is inside single support. The swing z has already
        started at ``l_ssp_start`` (dsp·T/4), when the sine is only
        sin(π·dsp/2) of the way there. This keeps that sine's ends: 0 at
        the half-period boundaries, and the peak at each single-support
        start, then falls through the swing so the next double support can
        cross. x and z are still the OP3 samples.
        """
        mag = self._y_swap
        if abs(mag) < 1e-12 or self.period <= 1e-9:
            return 0.0
        half = self.period / 2.0
        local = t % self.period
        sign = 1.0
        if local >= half:
            local -= half
            sign = -1.0
        rise = self.l_ssp_start
        if rise <= 1e-9 or rise >= half - 1e-9:
            phase = math.pi * local / half
        elif local <= rise:
            phase = (math.pi / 2.0) * (local / rise)
        else:
            phase = math.pi / 2.0 + (math.pi / 2.0) * ((local - rise) / (half - rise))
        return sign * mag * math.sin(phase)

    def _z_at(self, t_z: float, phase_t: float) -> float:
        z_phase = math.pi / 2.0 + 2.0 * math.pi / max(self.z_move_period, 1e-6) * phase_t
        return wsin(t_z, self.z_move_period, z_phase, self._z_move, self._z_move_shift)

    def _right_z(self, t: float) -> float:
        if t <= self.r_ssp_start:
            return self._z_at(self.r_ssp_start, self.r_ssp_start)
        if t <= self.r_ssp_end:
            return self._z_at(t, self.r_ssp_start)
        return self._z_at(self.r_ssp_end, self.r_ssp_start)

    def _left_z(self, t: float) -> float:
        if t <= self.l_ssp_start:
            return self._z_at(self.l_ssp_start, self.l_ssp_start)
        if t <= self.l_ssp_end:
            return self._z_at(t, self.l_ssp_start)
        return self._z_at(self.l_ssp_end, self.l_ssp_start)

    def _apply_direction(self, raw: np.ndarray, names: tuple[str, ...]) -> np.ndarray:
        out = np.zeros(6, dtype=np.float64)
        for i, name in enumerate(names):
            out[i] = float(raw[i]) * self.directions[name]
        return out

    def joints_now(self) -> tuple[dict[str, float] | None, StepInfo]:
        er, el, pel_r, pel_l, swap_y = self.endpoints()
        raw_r = ik_leg(self.lengths, float(er[0]), float(er[1]), float(er[2]), float(er[3]), float(er[4]), float(er[5]))
        raw_l = ik_leg(self.lengths, float(el[0]), float(el[1]), float(el[2]), float(el[3]), float(el[4]), float(el[5]))
        phase = "D"
        if self.l_ssp_start < self.time <= self.l_ssp_end:
            phase = "L"
        elif self.r_ssp_start < self.time <= self.r_ssp_end:
            phase = "R"
        info = StepInfo(
            phase=phase,
            swap_y_m=float(swap_y),
            ep_r=(float(er[0]), float(er[1]), float(er[2])),
            ep_l=(float(el[0]), float(el[1]), float(el[2])),
            ik_ok=raw_r is not None and raw_l is not None,
        )
        if raw_r is None or raw_l is None:
            return None, info
        jr = self._apply_direction(raw_r, _LEG_R)
        jl = self._apply_direction(raw_l, _LEG_L)
        jr[1] += self.directions["r_hip_roll"] * pel_r
        jl[1] += self.directions["l_hip_roll"] * pel_l
        # Kit hip_pitch_offset, including the stand pose. Right hip pitch
        # at the 0.025 m crouch is +0.766 rad; the knee stays −1.049 rad.
        jr[2] -= self.directions["r_hip_pitch"] * self.hit_pitch_offset
        jl[2] -= self.directions["l_hip_pitch"] * self.hit_pitch_offset
        out: dict[str, float] = {}
        for i, name in enumerate(_LEG_R):
            out[name] = float(jr[i])
        for i, name in enumerate(_LEG_L):
            out[name] = float(jl[i])
        if abs(self._x_move) < 1e-12:
            out["r_sho_pitch"] = 0.0
            out["l_sho_pitch"] = 0.0
        else:
            # OP3 treats x * gain * 1000 as degrees, then converts.
            mag = self._x_move * self.arm_swing_gain * 1000.0 * (math.pi / 180.0)
            right = wsin(self.time, self.period, math.pi * 1.5, -mag, 0.0)
            left = wsin(self.time, self.period, math.pi * 1.5, mag, 0.0)
            out["r_sho_pitch"] = right * self.directions["r_sho_pitch"]
            out["l_sho_pitch"] = left * self.directions["l_sho_pitch"]
        return out, info

    def stand_joints(self) -> dict[str, float]:
        """Offset-only pose. Every walking amplitude is zero.

        x/y/z move, y_swap, z_swap, and the turn are amplitudes. The
        cartesian crouch and the hip-width offset stay. Does not advance
        the clock, and restores the walking commands afterward.
        """
        saved = (
            self.x_cmd,
            self.y_cmd,
            self.angle_cmd,
            self.z_move_cmd,
            self.y_swap_cmd,
            self.z_swap_cmd,
            self.time,
            self.previous_x,
            self.ctrl_running,
        )
        self.x_cmd = 0.0
        self.y_cmd = 0.0
        self.angle_cmd = 0.0
        self.z_move_cmd = 0.0
        self.y_swap_cmd = 0.0
        self.z_swap_cmd = 0.0
        self.time = 0.0
        self.previous_x = 0.0
        self.ctrl_running = False
        self.update_time()
        self.update_movement()
        joints, info = self.joints_now()
        (
            self.x_cmd,
            self.y_cmd,
            self.angle_cmd,
            self.z_move_cmd,
            self.y_swap_cmd,
            self.z_swap_cmd,
            self.time,
            self.previous_x,
            self.ctrl_running,
        ) = saved
        self.update_time()
        self.update_movement()
        if joints is None or not info.ik_ok:
            raise RuntimeError("stand IK failed at the cartesian body drop")
        return joints

    def step(self, dt: float) -> tuple[dict[str, float] | None, StepInfo]:
        self.ctrl_running = True
        self.lead_blend_tick = False
        self.lead_carry_tick = False
        self.process_phase(dt)
        joints, info = self.joints_now()
        if joints is not None and self._lead_err is not None:
            carry = info.phase == "D"
            joints = self._bleed_lead_err(joints, info.phase)
            self.lead_blend_tick = True
            self.lead_carry_tick = carry
            self.lead_blend_s += dt
        self.time += dt
        if self.time >= self.period - 1e-12:
            self.time = 0.0
            self.previous_x = self.x_cmd * 0.5
            self.update_movement()
        if joints is not None:
            self._last_joints = joints
        return joints, info


def _sole_z(model: mj.MjModel, data: mj.MjData, side: str) -> float:
    gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, f"{side}_foot_contact")
    bid = int(model.geom_bodyid[gid])
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


def cartesian_body_drop(model: mj.MjModel, data: mj.MjData, walker: Op3Walker) -> float:
    """Hip-yaw z minus sole z, full extension minus the crouched IK stand.

    Positive means the crouched pose shortened the leg.
    """
    def height(joints: dict[str, float]) -> float:
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.qpos[2] = 0.30
        data.qpos[3] = 1.0
        for name, val in joints.items():
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                continue
            data.qpos[int(model.jnt_qposadr[jid])] = val
        mj.mj_forward(model, data)
        hip = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "r_hip_yaw_link")
        hip_z = float(data.xpos[hip][2])
        return hip_z - min(_sole_z(model, data, "r"), _sole_z(model, data, "l"))

    tall = walker.stand_joints()
    saved = walker.z_offset
    walker.z_offset = 0.0
    try:
        straight = walker.stand_joints()
    finally:
        walker.z_offset = saved
    return height(straight) - height(tall)


def kinematic_travel(walker: Op3Walker) -> tuple[float, float, float]:
    """Swing-stance z gap, swing-foot x travel, and peak |swap y|, cycle 2."""
    walker.previous_x = walker.x_cmd * 0.5
    walker.time = 0.0
    walker.update_movement()
    n = max(4, int(round(walker.period / 0.002)))
    xs: list[float] = []
    gaps: list[float] = []
    ys: list[float] = []
    for _ in range(n):
        er, el, _, _, swap_y = walker.endpoints()
        ys.append(abs(float(swap_y)))
        if walker.l_ssp_start < walker.time <= walker.l_ssp_end:
            gaps.append(float(el[2] - er[2]))
            xs.append(float(el[0]))
        walker.time += walker.period / n
        if walker.time >= walker.period:
            walker.time = 0.0
    walker.time = 0.0
    if not gaps:
        return 0.0, 0.0, max(ys) if ys else 0.0
    return max(gaps), max(xs) - min(xs), max(ys)
