#!/usr/bin/env python3
"""Draft Prefer-FAIL probes for the entrance straight rug bout.

Not the locked kit. locked_kit_config() stays gm_z_m 0.020. Plant file,
forcerange, kp, damping, and armature are not touched.

Two control-only copies, each scored on the same four bars:

- no airborne fault at 7.560 s (and none later on this bout)
- mid-swing toe min, middle 20-80% of each swing, above 0.002 m
- ankle pitch and roll |torque| under 2.33 Nm
- contact CoP inside the 145x86 sole box (excursion <= 0.001 m)

Swing-height copy: gm_z_m 0.034. At 20% of single support the OP3 z
sine is about 0.37 * gm_z_m above the frozen stance foot, so 0.034 m
adds about 5 mm of commanded lift at the phase of the -3.066 mm scuff.
That is the gap from that scuff to +2 mm, if the foot followed the
command. It does not raise a torque limit.

Ankle copy: while one sole has floor normal and rug normal both above
1 N, add atan(0.012 / 0.135) rad of toe-up on that ankle. Left positive,
right negative. The 12 mm step over the 135 mm sole. The flat
hip-plus-knee offset is otherwise unchanged.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path

import mujoco as mj
import numpy as np

import steer_walk as sw
import walk_gait_ainex as wg
from mono_toe_gate import _box_bottom_corners, body_forward_xy

SCENE = "/tmp/m62/mujoco/room_entrance.xml"
T_END = 8.2
T_BAR = 7.560
ANKLE_NM = 2.33
KNEE_NM = 2.33
PLANT_NM = 2.45
KNEE_KP = 45.0
TOE_BAR_M = 0.002
COP_BOX_M = 0.001
# 10.4 mm corner tilt over the 135 mm sole. MFG's 4.4° step-on.
STEP_TILT = math.atan(0.0104 / 0.135)
# 12 mm rug over the 135 mm contact box. Toe-up, not a torque limit.
EDGE_PITCH = math.atan(0.012 / 0.135)
# Commanded extra lift at ~20% of swing, sized to the 5 mm scuff gap.
Z_LIFT_M = 0.034
# 12 mm rug plus the 2 mm margin used on the arm line.
RUG_TOP_M = 0.012
CLEAR_MARGIN_M = 0.002
CMD_CLEAR_M = RUG_TOP_M + CLEAR_MARGIN_M


@dataclass
class Tick:
    t: float
    swing: str | None
    toe_z: float | None
    swing_fn: float
    fault: str


def _pair_fn(model: mj.MjModel, data: mj.MjData, ga: int, gb: int) -> float:
    total = 0.0
    for i in range(data.ncon):
        con = data.contact[i]
        g1, g2 = int(con.geom1), int(con.geom2)
        if (g1 == ga and g2 == gb) or (g1 == gb and g2 == ga):
            force = np.zeros(6, dtype=np.float64)
            mj.mj_contactForce(model, data, i, force)
            total += float(force[0])
    return total


def _leading_toe_z(session: sw.SteerSession, side: str) -> float:
    bid = session.bid_lf if side == "L" else session.bid_rf
    gid = session.gid_lfoot if side == "L" else session.gid_rfoot
    fwd = body_forward_xy(session.data, session.bid_body)
    origin = np.asarray(session.data.xpos[session.bid_body, :2], dtype=np.float64)
    best_z = 0.0
    best_off = -1e9
    for corner in _box_bottom_corners(session.model, session.data, bid, gid):
        off = float(np.dot(corner[:2] - origin, fwd))
        if off > best_off:
            best_off = off
            best_z = float(corner[2])
    return best_z


def _flex_sign(side: str) -> float:
    """L knee increases to flex. R knee decreases."""
    return 1.0 if side == "L" else -1.0


FOLLOW_TICKS = 0


def _hook_follow(session: sw.SteerSession, rug_gid: int) -> None:
    """Stance ankle tracks the contact pose on a split sole.

    The kit command is the flat hip-plus-knee offset. On the 12 mm step
    that offset never takes the 4.4° tilt, so the servo fights the sole.
    This copy writes the ankle target to the joint the contact is holding.
    It does not move kp or the ±2.45 Nm rail. The joint stays inside
    ±2.09; the nearest locked approach was about 0.7 rad short of that stop.
    """
    lipm = session.lipm
    if lipm is None:
        return
    orig = lipm._tick_gait_manager

    def wrapped(walking: bool) -> None:
        global FOLLOW_TICKS
        orig(walking)
        if not walking or session.bus.fault:
            return
        wrote = False
        for side, gid in (("L", session.gid_lfoot), ("R", session.gid_rfoot)):
            floor_n = _pair_fn(session.model, session.data, gid, session.gid_floor)
            rug_n = _pair_fn(session.model, session.data, gid, rug_gid)
            if floor_n <= 1.0 or rug_n <= 1.0:
                continue
            pref = "l_" if side == "L" else "r_"
            jn = pref + "ank_pitch"
            lipm.write_clipped(jn, lipm.q(jn))
            wrote = True
        if wrote:
            FOLLOW_TICKS += 1

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]


def _hook_clear(session: sw.SteerSession) -> None:
    """Mid-swing knee flex while the leading toe is under 2 mm.

    The added error is capped so kp * |goal − q| stays under 2.33 Nm.
    kp and forcerange are untouched.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        return
    orig = lipm._tick_gait_manager
    max_err = KNEE_NM / KNEE_KP

    def wrapped(walking: bool) -> None:
        orig(walking)
        if not walking or session.bus.fault:
            return
        swing = lipm._gm_swing
        walker = lipm.op3
        if swing not in ("L", "R") or walker is None:
            return
        if swing == "L":
            span = walker.l_ssp_end - walker.l_ssp_start
            frac = (walker.time - walker.l_ssp_start) / span if span > 1e-6 else 0.0
        else:
            span = walker.r_ssp_end - walker.r_ssp_start
            frac = (walker.time - walker.r_ssp_start) / span if span > 1e-6 else 0.0
        if frac < 0.20 or frac > 0.80:
            return
        if _leading_toe_z(session, swing) >= TOE_BAR_M:
            return
        pref = "l_" if swing == "L" else "r_"
        jn = pref + "knee"
        idx = lipm.act_idx[jn + "_pos"]
        q = lipm.q(jn)
        proposed = float(session.data.ctrl[idx]) + _flex_sign(swing) * 0.05
        err = proposed - q
        if abs(err) > max_err:
            proposed = q + math.copysign(max_err, err)
        lipm.write_clipped(jn, proposed)

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]


# Measured stance hip cmd-q at the peak of the scuff swing. Left stance
# -0.0566 rad, right stance +0.0566 rad. This is not the 1.29 Nm / 40
# feed-forward. That term stays off.
HIP_FF_L = -0.0566
HIP_FF_R = 0.0566


def _install_bezier_early(session: sw.SteerSession, peak_m: float) -> None:
    """Phase-local early bump. Zero at toe-off and at 40% of swing.

    The peak sits at 20% of single support. It is not a flat +12 or +20 mm
    held across the swing, and it does not change gm_z_m.
    """
    _install_phase_lift(session, peak_m, 0.20)


def _install_phase_lift(
    session: sw.SteerSession,
    peak_m: float,
    peak_frac: float,
    lead_frac: float = 0.0,
) -> None:
    """Move the bump peak earlier than the native z sine.

    The locked sine peaks at 50% of single support, so 30% is still low.
    This bump is zero at toe-off and at twice ``peak_frac``, and it peaks
    at ``peak_frac``. ``lead_frac`` starts that same bump before toe-off,
    during the end of double support, so the servo is already rising at
    20% of swing. Height stays near 8–12 mm. It is not a flat add, and it
    does not change gm_z_m. The 1.29 Nm term is not added.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        return
    walker = lipm.op3
    orig_r = walker._right_z
    orig_l = walker._left_z
    end_frac = min(0.80, 2.0 * peak_frac)

    def _bump(base: float, t: float, start: float, end: float) -> float:
        span = end - start
        if span <= 1e-6 or end_frac <= 1e-6:
            return base
        # Lead pulls the rise into the double support before this swing.
        if t <= start - lead_frac * span or t > end:
            return base
        frac = (t - start) / span + lead_frac
        if frac <= 0.0 or frac > end_frac:
            return base
        s = frac / end_frac
        return base + 4.0 * s * (1.0 - s) * peak_m

    def right_z(t: float) -> float:
        return _bump(orig_r(t), t, walker.r_ssp_start, walker.r_ssp_end)

    def left_z(t: float) -> float:
        return _bump(orig_l(t), t, walker.l_ssp_start, walker.l_ssp_end)

    walker._right_z = right_z  # type: ignore[method-assign]
    walker._left_z = left_z  # type: ignore[method-assign]


def _install_slow_rise(
    session: sw.SteerSession,
    peak_m: float,
    rise_frac: float,
    hold_frac: float,
    lead_s: float,
) -> None:
    """Rise to a modest peak over a longer window than the 20% parabola.

    The height stays at or under 12 mm. ``lead_s`` is seconds before
    toe-off. The smoothstep has zero slope at both ends of the rise, so
    the command is not a steep front. It does not change gm_z_m or kp.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        return
    walker = lipm.op3
    orig_r = walker._right_z
    orig_l = walker._left_z

    def _smooth(u: float) -> float:
        u = min(1.0, max(0.0, u))
        return u * u * (3.0 - 2.0 * u)

    def _bump(base: float, t: float, start: float, end: float) -> float:
        span = end - start
        if span <= 1e-6 or peak_m == 0.0:
            return base
        t0 = start - max(0.0, lead_s)
        rise_at = start + max(rise_frac, 1e-3) * span
        hold_at = start + max(hold_frac, rise_frac) * span
        fall_at = start + 0.80 * span
        if t <= t0 or t > fall_at:
            return base
        if t < rise_at:
            return base + _smooth((t - t0) / max(rise_at - t0, 1e-6)) * peak_m
        if t < hold_at:
            return base + peak_m
        return base + (1.0 - _smooth((t - hold_at) / max(fall_at - hold_at, 1e-6))) * peak_m

    def right_z(t: float) -> float:
        return _bump(orig_r(t), t, walker.r_ssp_start, walker.r_ssp_end)

    def left_z(t: float) -> float:
        return _bump(orig_l(t), t, walker.l_ssp_start, walker.l_ssp_end)

    walker._right_z = right_z  # type: ignore[method-assign]
    walker._left_z = left_z  # type: ignore[method-assign]


HIP_KIN: dict[str, float] = {}


def _stance_weights(walker: object) -> tuple[float, float]:
    """Left and right stance weights. Each double support ramps the switch.

    During a swing the stance weight is 1 and the swing weight is 0.
    Across the double support before the next swing, the outgoing stance
    fades out and the incoming stance fades in. No one-tick step.
    """
    period = float(walker.period)
    if period <= 1e-9:
        return 0.0, 0.0
    t = float(walker.time) % period
    a1 = float(walker.l_ssp_start)
    b0 = float(walker.l_ssp_end)
    b1 = float(walker.r_ssp_start)
    c0 = float(walker.r_ssp_end)
    if a1 < t <= b0:
        return 0.0, 1.0
    if b0 < t <= b1:
        span = b1 - b0
        u = 0.0 if span <= 1e-9 else (t - b0) / span
        return u, 1.0 - u
    if b1 < t <= c0:
        return 1.0, 0.0
    dsp = (period - c0) + a1
    elapsed = (t - c0) if t > c0 else (period - c0) + t
    u = 0.0 if dsp <= 1e-9 else elapsed / dsp
    return 1.0 - u, u


def _hook_hip_ramp(session: sw.SteerSession, gain: float) -> None:
    """Ramp the measured stance-hip error onto the current stance hip only.

    The swing hip is left on the kinematic target. The bias fades across
    double support instead of stepping on at the stance switch. The
    1.29/40 rad term is not added.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        return
    orig = lipm._tick_gait_manager
    walker = lipm.op3

    def wrapped(walking: bool) -> None:
        orig(walking)
        HIP_KIN.clear()
        if not walking or session.bus.fault:
            return
        w_l, w_r = _stance_weights(walker)
        for side, weight, bias in (("l", w_l, HIP_FF_L), ("r", w_r, HIP_FF_R)):
            name = side + "_hip_roll_pos"
            idx = lipm.act_idx[name]
            kin = float(session.data.ctrl[idx])
            HIP_KIN[side] = kin
            lo = float(session.model.actuator_ctrlrange[idx, 0])
            hi = float(session.model.actuator_ctrlrange[idx, 1])
            session.data.ctrl[idx] = min(hi, max(lo, kin + bias * gain * weight))

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]


def _hook_hip_ff(session: sw.SteerSession) -> None:
    """Shift the stance hip-roll target by the measured cmd-q error.

    The gait command is unchanged in intent. The target moves by the
    spring deflection the hip was already carrying, so the joint can sit
    on the old target. kp, damping, armature, and forcerange stay put.
    The 1.29/40 rad term is not added.
    """
    lipm = session.lipm
    if lipm is None:
        return
    orig = lipm._tick_gait_manager

    def wrapped(walking: bool) -> None:
        orig(walking)
        if not walking or session.bus.fault:
            return
        swing = lipm._gm_swing
        if swing not in ("L", "R"):
            return
        stance = "L" if swing == "R" else "R"
        bias = HIP_FF_L if stance == "L" else HIP_FF_R
        name = ("l_" if stance == "L" else "r_") + "hip_roll_pos"
        idx = lipm.act_idx[name]
        lo = float(session.model.actuator_ctrlrange[idx, 0])
        hi = float(session.model.actuator_ctrlrange[idx, 1])
        cmd = float(session.data.ctrl[idx]) + bias
        session.data.ctrl[idx] = min(hi, max(lo, cmd))

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]


def _install_early_z(session: sw.SteerSession, extra_m: float) -> None:
    """Add hip-frame z from toe-off through 40% of single support.

    The locked sine is still on the floor at 30%. This copy puts the extra
    on before the 20% mark so the 20–30% command can clear 0.014 m. It does
    not change kp, damping, armature, or forcerange.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        return
    walker = lipm.op3
    orig_r = walker._right_z
    orig_l = walker._left_z

    def _lift(base: float, t: float, start: float, end: float) -> float:
        span = end - start
        if span <= 1e-6 or not (start < t <= end):
            return base
        frac = (t - start) / span
        if frac <= 0.40 + 1e-12:
            return base + extra_m
        return base

    def right_z(t: float) -> float:
        return _lift(orig_r(t), t, walker.r_ssp_start, walker.r_ssp_end)

    def left_z(t: float) -> float:
        return _lift(orig_l(t), t, walker.l_ssp_start, walker.l_ssp_end)

    walker._right_z = right_z  # type: ignore[method-assign]
    walker._left_z = left_z  # type: ignore[method-assign]


def _install_mid_z(session: sw.SteerSession, extra_m: float) -> None:
    """Add hip-frame z on the swing foot during the middle 20–80% of SSP.

    Patches the OP3 z sample only. kp, damping, armature, and forcerange
    stay on the plant file.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        return
    walker = lipm.op3
    orig_r = walker._right_z
    orig_l = walker._left_z

    def _lift(base: float, t: float, start: float, end: float) -> float:
        span = end - start
        if span <= 1e-6 or not (start < t <= end):
            return base
        frac = (t - start) / span
        if 0.20 - 1e-12 <= frac <= 0.80 + 1e-12:
            return base + extra_m
        return base

    def right_z(t: float) -> float:
        return _lift(orig_r(t), t, walker.r_ssp_start, walker.r_ssp_end)

    def left_z(t: float) -> float:
        return _lift(orig_l(t), t, walker.l_ssp_start, walker.l_ssp_end)

    walker._right_z = right_z  # type: ignore[method-assign]
    walker._left_z = left_z  # type: ignore[method-assign]


def _hook_drop_flat(session: sw.SteerSession, rug_gid: int) -> None:
    """On a split sole, replace the flat hip+knee offset with the 4.4° tilt.

    The kit ankle is hip+knee ± 0.2618. This copy writes hip+knee ± the
    step tilt and leaves the ±2.09 ctrlrange clamp in write_clipped.
    It does not change kp or forcerange.
    """
    lipm = session.lipm
    if lipm is None:
        return
    orig = lipm._tick_gait_manager

    def wrapped(walking: bool) -> None:
        orig(walking)
        if not walking or session.bus.fault:
            return
        for side, gid in (("L", session.gid_lfoot), ("R", session.gid_rfoot)):
            floor_n = _pair_fn(session.model, session.data, gid, session.gid_floor)
            rug_n = _pair_fn(session.model, session.data, gid, rug_gid)
            if floor_n <= 1.0 or rug_n <= 1.0:
                continue
            pref = "l_" if side == "L" else "r_"
            sign = 1.0 if side == "L" else -1.0
            hip = float(session.data.ctrl[lipm.act_idx[pref + "hip_pitch_pos"]])
            knee = float(session.data.ctrl[lipm.act_idx[pref + "knee_pos"]])
            lipm.write_clipped(pref + "ank_pitch", hip + knee + sign * STEP_TILT)
            DROP_RESIDUAL.append(sign * (
                float(session.data.ctrl[lipm.act_idx[pref + "ank_pitch_pos"]]) - hip - knee
            ))

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]


DROP_RESIDUAL: list[float] = []
ANK_KP = 35.0
ANK_ERR_CAP = KNEE_NM / ANK_KP
ANK_GOAL: dict[str, float] = {}
ANK_WRITES: list[str] = []


def _sole_tilt(session: sw.SteerSession, side: str) -> float:
    """Toe-up angle of the sole. Positive means the forward corner is higher."""
    bid = session.bid_lf if side == "L" else session.bid_rf
    gid = session.gid_lfoot if side == "L" else session.gid_rfoot
    fwd = body_forward_xy(session.data, session.bid_body)
    origin = np.asarray(session.data.xpos[session.bid_body, :2], dtype=np.float64)
    best_off = -1e9
    worst_off = 1e9
    toe_z = heel_z = 0.0
    toe_xy = origin.copy()
    heel_xy = origin.copy()
    for corner in _box_bottom_corners(session.model, session.data, bid, gid):
        off = float(np.dot(corner[:2] - origin, fwd))
        if off > best_off:
            best_off = off
            toe_z = float(corner[2])
            toe_xy = np.asarray(corner[:2], dtype=np.float64)
        if off < worst_off:
            worst_off = off
            heel_z = float(corner[2])
            heel_xy = np.asarray(corner[:2], dtype=np.float64)
    dx = float(np.dot(toe_xy - heel_xy, fwd))
    return math.atan2(toe_z - heel_z, max(dx, 1e-6))


def _hook_swing_ff(session: sw.SteerSession, knee_rad: float, hip_rad: float) -> None:
    """Lead the swing knee and hip through the same 0–40% window as the bump.

    The 12 mm command is already above +2 mm. The knee at that tick is
    still behind it, and the applied ctrl is on the 5.5 rad/s slew. This
    adds a smooth extra in the flex direction so the slew keeps pushing.
    It peaks at 20% of single support and is zero at the ends. kp is
    unchanged. The 1.29 Nm hip-roll term is not added.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None or (knee_rad == 0.0 and hip_rad == 0.0):
        return
    orig = lipm._tick_gait_manager
    walker = lipm.op3

    def wrapped(walking: bool) -> None:
        orig(walking)
        if not walking or session.bus.fault:
            return
        swing = lipm._gm_swing
        if swing not in ("L", "R"):
            return
        if swing == "L":
            start, end = walker.l_ssp_start, walker.l_ssp_end
        else:
            start, end = walker.r_ssp_start, walker.r_ssp_end
        span = end - start
        if span <= 1e-6:
            return
        frac = (float(walker.time) - start) / span
        if frac <= 0.0 or frac > 0.40:
            return
        s = frac / 0.40
        weight = 4.0 * s * (1.0 - s)
        pref = "l_" if swing == "L" else "r_"
        hip_sign = -1.0 if swing == "L" else 1.0
        knee_sign = 1.0 if swing == "L" else -1.0
        for jn, sign, amp in (
            ("hip_pitch", hip_sign, hip_rad),
            ("knee", knee_sign, knee_rad),
        ):
            if amp == 0.0:
                continue
            idx = lipm.act_idx[pref + jn + "_pos"]
            lo = float(session.model.actuator_ctrlrange[idx, 0])
            hi = float(session.model.actuator_ctrlrange[idx, 1])
            cmd = float(session.data.ctrl[idx]) + sign * amp * weight
            session.data.ctrl[idx] = min(hi, max(lo, cmd))

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]


SOLE_LEN_M = 0.135
# 0.012 rad/tick at ankle kp 35 is about 0.42 Nm. The one-tick step that
# railed the knee was 0.067 rad, about 2.3 Nm.
ANK_RATE_RAD = 0.012


def _sole_ends(session: sw.SteerSession, side: str) -> tuple[np.ndarray, np.ndarray]:
    """Heel and toe of the sole bottom, world frame. Heel is the −x end."""
    bid = session.bid_lf if side == "L" else session.bid_rf
    gid = session.gid_lfoot if side == "L" else session.gid_rfoot
    origin = np.array(session.data.xpos[bid], dtype=np.float64)
    rot = np.array(session.data.xmat[bid], dtype=np.float64).reshape(3, 3)
    pos = np.array(session.model.geom_pos[gid], dtype=np.float64)
    half = np.array(session.model.geom_size[gid], dtype=np.float64)
    heel = origin + rot @ (pos + np.array([-half[0], 0.0, -half[2]]))
    toe = origin + rot @ (pos + np.array([half[0], 0.0, -half[2]]))
    return heel, toe


def _rug_edge(session: sw.SteerSession, side: str, rug_gid: int) -> dict[str, float | str] | None:
    """Where the rug face crosses the sole, as a heel→toe fraction.

    Tilt is atan(plant step / lever). The lever is the sole length on the
    rug side of that face. It changes as the foot walks onto the rug.
    """
    if rug_gid < 0:
        return None
    heel, toe = _sole_ends(session, side)
    center = np.array(session.data.geom_xpos[rug_gid], dtype=np.float64)
    rot = np.array(session.data.geom_xmat[rug_gid], dtype=np.float64).reshape(3, 3)
    half = np.array(session.model.geom_size[rug_gid], dtype=np.float64)
    seg = toe - heel
    best: dict[str, float | str] | None = None
    for axis in (0, 1):
        axis_xy = rot[:, axis].copy()
        axis_xy[2] = 0.0
        norm = float(np.linalg.norm(axis_xy))
        if norm < 1e-8:
            continue
        axis_xy /= norm
        for sgn in (-1.0, 1.0):
            normal = sgn * axis_xy
            point = center + sgn * rot[:, axis] * float(half[axis])
            den = float(np.dot(seg[:2], normal[:2]))
            if abs(den) < 1e-8:
                continue
            frac = float(np.dot((point - heel)[:2], normal[:2]) / den)
            if frac < -0.05 or frac > 1.05:
                continue
            heel_out = float(np.dot((heel - point)[:2], normal[:2]))
            toe_out = float(np.dot((toe - point)[:2], normal[:2]))
            if heel_out * toe_out > 0.0:
                continue
            # The walk hits the face with the heel still outside.
            if heel_out <= 0.0 or toe_out >= 0.0:
                continue
            frac = min(1.0, max(0.0, frac))
            lever = (1.0 - frac) * SOLE_LEN_M
            row: dict[str, float | str] = {
                "u": frac,
                "lever_m": lever,
                "high": "toe",
                "face_x": float(point[0]),
                "heel_x": float(heel[0]),
                "toe_x": float(toe[0]),
            }
            if best is None or float(row["u"]) < float(best["u"]):
                best = row
    return best


def _heel_z(session: sw.SteerSession, side: str) -> float:
    """Lower of the two rear sole corners, world z."""
    bid = session.bid_lf if side == "L" else session.bid_rf
    gid = session.gid_lfoot if side == "L" else session.gid_rfoot
    fwd = body_forward_xy(session.data, session.bid_body)
    origin = np.asarray(session.data.xpos[session.bid_body, :2], dtype=np.float64)
    ranked: list[tuple[float, float]] = []
    for corner in _box_bottom_corners(session.model, session.data, bid, gid):
        off = float(np.dot(corner[:2] - origin, fwd))
        ranked.append((off, float(corner[2])))
    ranked.sort(key=lambda item: item[0])
    if not ranked:
        return float("nan")
    return min(z for _off, z in ranked[:2])


def _contact_geom(session: sw.SteerSession, side: str, rug_gid: int) -> dict[str, float]:
    """Tilt from the floor contacts and the rug contacts on this sole.

    Absolute sole tilt includes body pitch. The contact tilt is the height
    between those two contact clouds over their separation along the sole.
    The edge tilt is atan(12 mm / heel-to-edge), which is about 5.5° when
    the edge is at heel→toe fraction 0.915.
    """
    gid = session.gid_lfoot if side == "L" else session.gid_rfoot
    floor_pts: list[np.ndarray] = []
    rug_pts: list[np.ndarray] = []
    for i in range(session.data.ncon):
        con = session.data.contact[i]
        g1, g2 = int(con.geom1), int(con.geom2)
        pos = np.array(con.pos, dtype=np.float64)
        if (g1 == gid and g2 == session.gid_floor) or (g2 == gid and g1 == session.gid_floor):
            floor_pts.append(pos)
        if rug_gid >= 0 and ((g1 == gid and g2 == rug_gid) or (g2 == gid and g1 == rug_gid)):
            rug_pts.append(pos)
    heel, toe = _sole_ends(session, side)
    out: dict[str, float] = {
        "heel_z": _heel_z(session, side),
        "sole": _sole_tilt(session, side),
        "contact_tilt": float("nan"),
        "geom_tilt": float("nan"),
        "u": float("nan"),
        "sep": float("nan"),
    }
    edge = _rug_edge(session, side, rug_gid)
    if edge is not None:
        heel_to_edge = float(edge["u"]) * SOLE_LEN_M
        out["u"] = float(edge["u"])
        out["geom_tilt"] = math.atan(RUG_TOP_M / max(heel_to_edge, 0.020))
    if floor_pts and rug_pts:
        pf = np.mean(np.stack(floor_pts, axis=0), axis=0)
        pr = np.mean(np.stack(rug_pts, axis=0), axis=0)
        along = toe - heel
        span = float(np.linalg.norm(along[:2]))
        dist = 0.0
        if span > 1e-6:
            dist = float(np.dot((pr - pf)[:2], along[:2]) / span)
        out["sep"] = dist
        out["contact_tilt"] = math.atan2(float(pr[2] - pf[2]), max(abs(dist), 1e-4))
    return out


def _edge_line(session: sw.SteerSession, rug_gid: int) -> str:
    parts: list[str] = []
    for side in ("L", "R"):
        gid = session.gid_lfoot if side == "L" else session.gid_rfoot
        floor_n = _pair_fn(session.model, session.data, gid, session.gid_floor)
        rug_n = _pair_fn(session.model, session.data, gid, rug_gid)
        geo = _contact_geom(session, side, rug_gid)
        if not math.isfinite(geo["u"]):
            parts.append(
                f"{side} floor {floor_n:.1f} rug {rug_n:.1f} "
                f"heel_z {geo['heel_z'] * 1000:+.1f} mm sole {geo['sole']:+.4f} edge none"
            )
            continue
        parts.append(
            f"{side} floor {floor_n:.1f} rug {rug_n:.1f} "
            f"heel_z {geo['heel_z'] * 1000:+.1f} mm "
            f"sole {geo['sole']:+.4f} rad ({math.degrees(geo['sole']):.2f} deg) "
            f"edge {geo['u']:.3f} "
            f"geom {geo['geom_tilt']:+.4f} rad ({math.degrees(geo['geom_tilt']):.2f} deg) "
            f"contact {geo['contact_tilt']:+.4f} rad "
            f"({math.degrees(geo['contact_tilt']):.2f} deg) sep {geo['sep'] * 1000:.1f} mm"
        )
    return " | ".join(parts)


def _hook_ank_follow(session: sw.SteerSession, rug_gid: int, gain: float = 1.0, rate: float = 0.0) -> None:
    """Stance ankle follows the floor-to-rug contact tilt, capped per tick.

    The add is gain times the tilt between the floor contact cloud and the
    rug contact cloud. Absolute sole tilt is logged and is not the command.
    The swing foot is not written. kp stays 35.
    """
    lipm = session.lipm
    if lipm is None:
        return
    orig = lipm._tick_gait_manager
    ANK_WRITES.clear()

    def wrapped(walking: bool) -> None:
        orig(walking)
        ANK_GOAL.clear()
        if not walking or session.bus.fault:
            return
        swing = lipm._gm_swing
        for side, gid in (("L", session.gid_lfoot), ("R", session.gid_rfoot)):
            if swing == side:
                continue
            floor_n = _pair_fn(session.model, session.data, gid, session.gid_floor)
            rug_n = _pair_fn(session.model, session.data, gid, rug_gid)
            if floor_n <= 1.0 or rug_n <= 1.0:
                continue
            geo = _contact_geom(session, side, rug_gid)
            tilt = geo["contact_tilt"]
            if not math.isfinite(tilt) or tilt <= 0.0:
                tilt = geo["geom_tilt"]
            if not math.isfinite(tilt) or tilt <= 0.0:
                continue
            pref = "l_" if side == "L" else "r_"
            sign = 1.0 if side == "L" else -1.0
            idx = lipm.act_idx[pref + "ank_pitch_pos"]
            flat = float(session.data.ctrl[idx])
            q = lipm.q(pref + "ank_pitch")
            target = flat + sign * gain * tilt
            cap = rate if rate > 0.0 else ANK_RATE_RAD
            step = target - flat
            if abs(step) > cap:
                target = flat + math.copysign(cap, step)
            lipm.write_clipped(pref + "ank_pitch", target)
            ANK_GOAL[side] = float(session.data.ctrl[idx])
            if len(ANK_WRITES) < 8:
                ANK_WRITES.append(
                    f"t {float(session.data.time):.3f} {side} "
                    f"heel_z {geo['heel_z'] * 1000:+.1f} mm "
                    f"sole {geo['sole']:+.4f} contact {geo['contact_tilt']:+.4f} "
                    f"geom {geo['geom_tilt']:+.4f} edge {geo['u']:.3f} "
                    f"q {q:+.4f} flat {flat:+.4f} goal {ANK_GOAL[side]:+.4f} "
                    f"dctrl {ANK_GOAL[side] - flat:+.4f} "
                    f"floor {floor_n:.1f} rug {rug_n:.1f}"
                )

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]


def _hook_edge(session: sw.SteerSession, rug_gid: int) -> None:
    """Toe-up only on a sole that is already split across floor and rug."""
    lipm = session.lipm
    if lipm is None:
        return
    orig = lipm._tick_gait_manager

    def wrapped(walking: bool) -> None:
        orig(walking)
        if not walking or session.bus.fault:
            return
        for side, gid in (("L", session.gid_lfoot), ("R", session.gid_rfoot)):
            floor_n = _pair_fn(session.model, session.data, gid, session.gid_floor)
            rug_n = _pair_fn(session.model, session.data, gid, rug_gid)
            if floor_n <= 1.0 or rug_n <= 1.0:
                continue
            sign = 1.0 if side == "L" else -1.0
            pref = "l_" if side == "L" else "r_"
            idx = lipm.act_idx[pref + "ank_pitch_pos"]
            lipm.write_clipped(pref + "ank_pitch", float(session.data.ctrl[idx]) + sign * EDGE_PITCH)

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]


def _mid_swing(rows: list[Tick]) -> tuple[float, float, float, int, int, float, str]:
    """Min, time, p90, mid count, cycle count, fraction, swing side. t < 7.560 s."""
    cycles: list[list[Tick]] = []
    cur: list[Tick] = []
    for row in rows:
        if row.t >= T_BAR - 1e-9:
            break
        if row.swing in ("L", "R"):
            if cur and cur[-1].swing != row.swing:
                cycles.append(cur)
                cur = []
            cur.append(row)
        elif cur:
            cycles.append(cur)
            cur = []
    if cur:
        cycles.append(cur)
    mids: list[tuple[float, float, float, str]] = []
    for cyc in cycles:
        n = len(cyc)
        if n < 2:
            continue
        for i, row in enumerate(cyc):
            frac = i / (n - 1)
            if row.toe_z is None or row.swing is None:
                continue
            if 0.20 - 1e-12 <= frac <= 0.80 + 1e-12:
                mids.append((row.t, row.toe_z, frac, row.swing))
    if not mids:
        return (float("nan"), float("nan"), float("nan"), 0, len(cycles), float("nan"), "")
    zs = np.asarray([z for _t, z, _f, _s in mids], dtype=np.float64)
    k = int(np.argmin(zs))
    return (
        float(zs[k]),
        float(mids[k][0]),
        float(np.percentile(zs, 90)),
        len(mids),
        len(cycles),
        float(mids[k][2]),
        mids[k][3],
    )


def _ankle_peak(session: sw.SteerSession) -> tuple[float, str, float]:
    peak = 0.0
    name = ""
    t_peak = 0.0
    for i in range(session.model.nu):
        act = mj.mj_id2name(session.model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        if "ank_pitch" not in act and "ank_roll" not in act:
            continue
        force = float(session.data.actuator_force[i])
        if abs(force) >= abs(peak):
            peak = force
            name = act
            t_peak = float(session.data.time)
    return peak, name, t_peak


def _named_force(session: sw.SteerSession, tokens: tuple[str, ...]) -> tuple[float, str, float]:
    peak = 0.0
    name = ""
    t_peak = 0.0
    for i in range(session.model.nu):
        act = mj.mj_id2name(session.model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        if not any(tok in act for tok in tokens):
            continue
        force = float(session.data.actuator_force[i])
        if abs(force) >= abs(peak):
            peak = force
            name = act
            t_peak = float(session.data.time)
    return peak, name, t_peak


def _cmd_time(walker: object) -> float:
    import op3_walk
    t_cmd = float(walker.time) - op3_walk.OP3_CTRL_S
    if t_cmd < -1e-9:
        t_cmd += float(walker.period)
    return t_cmd


def _mid_frac(walker: object, side: str, t_cmd: float) -> float | None:
    if side == "L":
        start = float(walker.l_ssp_start)
        end = float(walker.l_ssp_end)
    else:
        start = float(walker.r_ssp_start)
        end = float(walker.r_ssp_end)
    span = end - start
    if span <= 1e-6 or not (start < t_cmd <= end):
        return None
    return (t_cmd - start) / span


def _pelvis_roll(session: sw.SteerSession) -> float:
    rot = np.asarray(session.data.xmat[session.bid_body], dtype=np.float64).reshape(3, 3)
    return math.atan2(float(rot[2, 1]), float(rot[2, 2]))


def _hip_kin_sample(session: sw.SteerSession, side: str) -> tuple[float, float, float, float]:
    """Actual q, un-offset kinematic target, applied ctrl, torque."""
    q, ctrl, tau = _joint_qcf(session, side + "_hip_roll")
    kin = float(HIP_KIN.get(side, ctrl))
    return q, kin, ctrl, tau


def _roll_sample(session: sw.SteerSession, t: float, swing: str) -> dict[str, float | str]:
    stance = "L" if swing == "R" else "R"
    pref = "l_" if stance == "L" else "r_"
    lq, lk, lc, lt = _hip_kin_sample(session, "l")
    rq, rk, rc, rt = _hip_kin_sample(session, "r")
    aq, ac, af = _joint_qcf(session, pref + "ank_roll")
    _, _, lp = _joint_qcf(session, "l_hip_pitch")
    _, _, rp = _joint_qcf(session, "r_hip_pitch")
    return {
        "t": t,
        "roll": _pelvis_roll(session),
        "ldq": lq - lk,
        "rdq": rq - rk,
        "lt": lt,
        "rt": rt,
        "lp": lp,
        "rp": rp,
        "at": af,
        "aerr": abs(ac - aq),
        "lq": lq,
        "lk": lk,
        "rq": rq,
        "rk": rk,
        "lc": lc,
        "rc": rc,
    }


def _roll_note(rows: list[dict[str, float | str]]) -> str:
    if not rows:
        return ""
    window = [row for row in rows if 3.344 - 1e-3 <= float(row["t"]) <= 3.536 + 1e-3]
    use = window or rows
    at = min(use, key=lambda row: abs(float(row["t"]) - 3.384))
    roll_peak = max(use, key=lambda row: abs(float(row["roll"])))
    ldq = max(use, key=lambda row: abs(float(row["ldq"])))
    rdq = max(use, key=lambda row: abs(float(row["rdq"])))
    return (
        f"window {float(use[0]['t']):.3f}-{float(use[-1]['t']):.3f} "
        f"roll_at_3.384 {float(at['roll']):.4f} "
        f"ldq_at_3.384 {float(at['ldq']):.4f} rdq_at_3.384 {float(at['rdq']):.4f} "
        f"roll_peak {float(roll_peak['roll']):.4f} t_roll {float(roll_peak['t']):.3f} "
        f"ldq_peak {float(ldq['ldq']):.4f} t_ldq {float(ldq['t']):.3f} "
        f"rdq_peak {float(rdq['rdq']):.4f} t_rdq {float(rdq['t']):.3f} "
        f"win_l_hip_roll {float(max(use, key=lambda row: abs(float(row['lt'])))['lt']):.4f} "
        f"win_r_hip_roll {float(max(use, key=lambda row: abs(float(row['rt'])))['rt']):.4f} "
        f"win_l_hip_pitch {float(max(use, key=lambda row: abs(float(row['lp'])))['lp']):.4f} "
        f"win_r_hip_pitch {float(max(use, key=lambda row: abs(float(row['rp'])))['rp']):.4f} "
        f"win_ank {float(max(use, key=lambda row: abs(float(row['at'])))['at']):.4f} "
        f"win_ank_abs_err {float(max(use, key=lambda row: float(row['aerr']))['aerr']):.4f}"
    )


def _rug_sample(session: sw.SteerSession, rug: int, t: float, reason: str) -> dict[str, float | str]:
    """One tick of the rug window: contacts, ankle pitch, sole tilt, CoP."""
    row: dict[str, float | str] = {
        "t": t,
        "up": session._up_z(),
        "cop": float(session.cop_excursion),
        "fault": reason,
    }
    for side, gid in (("L", session.gid_lfoot), ("R", session.gid_rfoot)):
        pref = "l_" if side == "L" else "r_"
        q, ctrl, tau = _joint_qcf(session, pref + "ank_pitch")
        goal = float(ANK_GOAL.get(side, ctrl))
        row[side + "_floor"] = _pair_fn(session.model, session.data, gid, session.gid_floor)
        row[side + "_rug"] = _pair_fn(session.model, session.data, gid, rug)
        row[side + "_q"] = q
        row[side + "_ctrl"] = ctrl
        row[side + "_goal"] = goal
        row[side + "_tau"] = tau
        row[side + "_tilt"] = _sole_tilt(session, side)
        row[side + "_lead"] = _leading_toe_z(session, side)
    floor_l = float(row["L_floor"]) > 1.0
    floor_r = float(row["R_floor"]) > 1.0
    if floor_l and floor_r:
        row["air_foot"] = "none"
    elif floor_l:
        row["air_foot"] = "R"
    elif floor_r:
        row["air_foot"] = "L"
    else:
        row["air_foot"] = "both"
    return row


def _rug_note(rows: list[dict[str, float | str]]) -> str:
    if not rows:
        return ""
    picks = []
    for label, target in (("t7016", 7.016), ("t7560", 7.560)):
        row = min(rows, key=lambda item: abs(float(item["t"]) - target))
        picks.append(_rug_line(label, row))
    fault = next((row for row in rows if "airborne" in str(row["fault"])), None)
    if fault is not None:
        picks.append(_rug_line("fault", fault))
    return " | ".join(picks)


def _rug_line(label: str, row: dict[str, float | str]) -> str:
    return (
        f"{label} t {float(row['t']):.3f} up {float(row['up']):.3f} "
        f"cop {float(row['cop']):.6f} air {row['air_foot']} fault {row['fault'] or '-'} "
        f"L floor {float(row['L_floor']):.1f} rug {float(row['L_rug']):.1f} "
        f"q {float(row['L_q']):+.4f} ctrl {float(row['L_ctrl']):+.4f} "
        f"goal {float(row['L_goal']):+.4f} tilt {float(row['L_tilt']):+.4f} "
        f"lead {float(row['L_lead']):+.4f} tau {float(row['L_tau']):+.3f} "
        f"R floor {float(row['R_floor']):.1f} rug {float(row['R_rug']):.1f} "
        f"q {float(row['R_q']):+.4f} ctrl {float(row['R_ctrl']):+.4f} "
        f"goal {float(row['R_goal']):+.4f} tilt {float(row['R_tilt']):+.4f} "
        f"lead {float(row['R_lead']):+.4f} tau {float(row['R_tau']):+.3f}"
    )


def _swing_joint_snap(
    session: sw.SteerSession,
    side: str,
    t: float,
    frac: float,
    goal_sole: float,
    ctrl_sole: float,
    actual_sole: float,
    joints: dict[str, float],
) -> dict[str, float | str]:
    """Swing hip, knee, and ankle at one tick: gait goal, applied ctrl, q."""
    pref = "l_" if side == "L" else "r_"
    row: dict[str, float | str] = {
        "t": t,
        "side": side,
        "frac": frac,
        "goal_sole": goal_sole,
        "ctrl_sole": ctrl_sole,
        "actual_sole": actual_sole,
        "cmd_short": TOE_BAR_M - goal_sole,
        "sag": goal_sole - actual_sole,
    }
    for jn in ("hip_roll", "hip_pitch", "knee", "ank_pitch"):
        q, ctrl, tau = _joint_qcf(session, pref + jn)
        goal = float(joints.get(pref + jn, ctrl))
        idx = session.act_idx[pref + jn + "_pos"]
        kp = float(session.model.actuator_gainprm[idx, 0])
        row[jn + "_q"] = q
        row[jn + "_ctrl"] = ctrl
        row[jn + "_goal"] = goal
        row[jn + "_tau"] = tau
        row[jn + "_goal_q"] = goal - q
        row[jn + "_ctrl_q"] = ctrl - q
        # kp*(goal−q) is the position-loop ask. actuator_force is the output.
        row[jn + "_ask"] = kp * (goal - q)
        row[jn + "_kp"] = kp
    return row


def _snap_note(snaps: list[dict[str, float | str]], toe_t: float) -> str:
    if not snaps or not math.isfinite(toe_t):
        return ""
    row = min(snaps, key=lambda item: abs(float(item["t"]) - toe_t))
    parts = [
        f"t {float(row['t']):.3f} {row['side']} frac {float(row['frac']):.3f}",
        f"goal_sole {float(row['goal_sole']):+.6f}",
        f"ctrl_sole {float(row['ctrl_sole']):+.6f}",
        f"actual_sole {float(row['actual_sole']):+.6f}",
        f"cmd_short_of_0.002 {float(row['cmd_short']):+.6f}",
        f"sag_cmd_minus_actual {float(row['sag']):+.6f}",
    ]
    for jn in ("hip_roll", "hip_pitch", "knee", "ank_pitch"):
        force = float(row[jn + "_tau"])
        clip = " CLIP" if abs(force) >= PLANT_NM - 0.01 else ""
        parts.append(
            f"{jn} goal-q {float(row[jn + '_goal_q']):+.4f} "
            f"ask {float(row[jn + '_ask']):+.2f}Nm "
            f"actuator {force:+.3f}{clip}"
        )
    return " ".join(parts)


LEG8 = (
    "l_hip_roll_pos", "r_hip_roll_pos",
    "l_hip_pitch_pos", "r_hip_pitch_pos",
    "l_knee_pos", "r_knee_pos",
    "l_ank_pitch_pos", "r_ank_pitch_pos",
)


def run_one(
    name: str,
    *,
    z_m: float | None,
    edge: bool,
    mode: str = "",
    z_extra: float = 0.0,
    t_end: float = T_END,
    gain: float = 1.0,
    peak_frac: float = 0.20,
    lead_frac: float = 0.0,
    rise_frac: float = 0.20,
    hold_frac: float = 0.40,
    lead_s: float = 0.0,
    knee_ff: float = 0.0,
    hip_ff: float = 0.0,
    ank_rate: float = 0.0,
) -> dict[str, object]:
    cfg = sw.locked_kit_config()
    if z_m is not None:
        cfg = replace(cfg, name=name, gm_z_m=z_m)
    session = sw.SteerSession(video=False, scene_xml=Path(SCENE), lipm=cfg)
    rug = mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_GEOM, "col_mat_rug")
    if rug < 0:
        raise SystemExit("missing col_mat_rug")
    if mode == "follow":
        _hook_follow(session, rug)
    elif mode == "clear":
        _hook_clear(session)
    elif mode == "zmid":
        _install_mid_z(session, z_extra)
    elif mode == "zearly":
        _install_early_z(session, z_extra)
    elif mode == "hipff":
        _hook_hip_ff(session)
    elif mode == "hipbez":
        _hook_hip_ff(session)
        _install_bezier_early(session, z_extra)
    elif mode == "hipramp":
        _hook_hip_ramp(session, gain)
    elif mode == "bez":
        _install_bezier_early(session, z_extra)
    elif mode == "phase":
        _install_phase_lift(session, z_extra, peak_frac, lead_frac)
    elif mode == "slow":
        _install_slow_rise(session, z_extra, rise_frac, hold_frac, lead_s)
    elif mode == "phaseank":
        _install_phase_lift(session, z_extra, peak_frac, lead_frac)
        _hook_ank_follow(session, rug, gain, ank_rate)
    elif mode == "swingff":
        _install_phase_lift(session, z_extra, peak_frac, lead_frac)
        _hook_swing_ff(session, knee_ff, hip_ff)
    elif mode == "phaseramp":
        _hook_hip_ramp(session, gain)
        _install_phase_lift(session, z_extra, peak_frac, lead_frac)
    elif mode == "hiprampbez":
        _hook_hip_ramp(session, gain)
        _install_bezier_early(session, z_extra)
    elif mode == "ankfollow":
        _hook_ank_follow(session, rug, gain, ank_rate)
    elif mode == "dropflat":
        DROP_RESIDUAL.clear()
        _hook_drop_flat(session, rug)
    elif edge:
        _hook_edge(session, rug)
    driver = sw.ScriptedDriver((
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(t_end, "vel", sw.VX_FWD_CAP, 0.0, "forward"),
    ))
    cmd_leads: list[tuple[float, float]] = []
    swing_snaps: list[dict[str, float | str]] = []
    rows: list[Tick] = []
    ank_peak = 0.0
    ank_name = ""
    ank_t = 0.0
    walk_peak = 0.0
    walk_name = ""
    walk_t = 0.0
    knee_peak = 0.0
    knee_name = ""
    knee_t = 0.0
    walk_knee = 0.0
    walk_knee_name = ""
    walk_knee_t = 0.0
    first_fault_t: float | None = None
    first_fault = ""
    air_at_bar = False
    edge_ticks = 0
    real_step = mj.mj_step
    seen_fault = False
    leg_peak = 0.0
    leg_name = ""
    leg_t = 0.0
    walk_leg = 0.0
    walk_leg_name = ""
    walk_leg_t = 0.0
    roll_rows: list[dict[str, float | str]] = []
    hip4 = {name: (0.0, 0.0) for name in (
        "l_hip_roll_pos", "r_hip_roll_pos", "l_hip_pitch_pos", "r_hip_pitch_pos",
    )}
    leg8 = {name: (0.0, 0.0) for name in LEG8}
    rug_rows: list[dict[str, float | str]] = []
    old_hold = 0.0
    old_air_t: float | None = None
    old_air_up = float("nan")
    old_air_margin = float("nan")
    edge_7016 = ""
    up_7560 = float("nan")
    margin_7560 = float("nan")

    def _hook(model: mj.MjModel, data: mj.MjData) -> None:
        nonlocal ank_peak, ank_name, ank_t, walk_peak, walk_name, walk_t
        nonlocal knee_peak, knee_name, knee_t, walk_knee, walk_knee_name, walk_knee_t
        nonlocal leg_peak, leg_name, leg_t, walk_leg, walk_leg_name, walk_leg_t
        real_step(model, data)
        lforce, lact, lwhen = _named_force(
            session, ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll"),
        )
        if abs(lforce) > abs(leg_peak):
            leg_peak = lforce
            leg_name = lact
            leg_t = lwhen
        if not seen_fault and abs(lforce) > abs(walk_leg):
            walk_leg = lforce
            walk_leg_name = lact
            walk_leg_t = lwhen
        force, act, when = _ankle_peak(session)
        if abs(force) > abs(ank_peak):
            ank_peak = force
            ank_name = act
            ank_t = when
        if not seen_fault and abs(force) > abs(walk_peak):
            walk_peak = force
            walk_name = act
            walk_t = when
        kforce, kact, kwhen = _named_force(session, ("knee_pos",))
        if abs(kforce) > abs(knee_peak):
            knee_peak = kforce
            knee_name = kact
            knee_t = kwhen
        if not seen_fault and abs(kforce) > abs(walk_knee):
            walk_knee = kforce
            walk_knee_name = kact
            walk_knee_t = kwhen
        if not seen_fault:
            for name in leg8:
                idx = session.act_idx[name]
                tau = float(data.actuator_force[idx])
                prev, _when = leg8[name]
                if abs(tau) > abs(prev):
                    leg8[name] = (tau, float(data.time))
                if name in hip4 and abs(tau) > abs(hip4[name][0]):
                    hip4[name] = (tau, float(data.time))

    mj.mj_step = _hook
    try:
        while float(session.data.time) < t_end - 1e-9:
            driver.publish(session.bus, float(session.data.time))
            session.step()
            t = float(session.data.time)
            swing = session.lipm._gm_swing if session.lipm is not None else None
            toe_z = _leading_toe_z(session, swing) if swing in ("L", "R") else None
            if mode in ("hipff", "hipbez", "hipramp", "hiprampbez", "bez", "phase", "phaseramp") and swing in ("L", "R") and t < T_BAR:
                roll_rows.append(_roll_sample(session, t, swing))
            if mode in ("zmid", "zearly", "hipbez", "bez", "hiprampbez", "phase", "phaseramp", "swingff", "slow", "phaseank") and swing in ("L", "R") and session.lipm is not None and session.lipm.op3 is not None and not (mode in ("phase", "slow", "phaseank") and t >= 6.0):
                walker = session.lipm.op3
                frac = _mid_frac(walker, swing, _cmd_time(walker))
                lo, hi = (0.0, 1.0) if mode in ("phase", "slow", "phaseank") else (
                    (0.20, 0.30) if mode in ("zearly", "hipbez", "bez", "hiprampbez", "phaseramp", "swingff") else (0.20, 0.80)
                )
                record_all = mode in ("phase", "slow", "phaseank") and t < 5.0
                if frac is not None and lo - 1e-12 <= frac <= hi + 1e-12 and t < T_BAR:
                    # ctrl has been slewed. The gait goal was the pre-slew write.
                    # Re-read the live OP3 pose at the command time and FK that.
                    saved_t = walker.time
                    walker.time = _cmd_time(walker)
                    joints, _info = walker.joints_now()
                    walker.time = saved_t
                    if joints is not None:
                        goal = np.array(session.data.ctrl, dtype=np.float64, copy=True)
                        for jn, val in joints.items():
                            act = jn + "_pos"
                            if act in session.act_idx:
                                goal[session.act_idx[act]] = val
                        lead, _low = _fk_leading_toe(session, swing, goal)
                        cmd_leads.append((t, lead))
                        if record_all or abs(frac - 0.208) <= 0.03:
                            applied = np.array(session.data.ctrl, dtype=np.float64, copy=True)
                            app_lead, _app_low = _fk_leading_toe(session, swing, applied)
                            swing_snaps.append(_swing_joint_snap(
                                session, swing, t, frac, lead, app_lead, toe_z if toe_z is not None else float("nan"), joints,
                            ))
            swing_fn = 0.0
            if swing in ("L", "R"):
                gid = session.gid_lfoot if swing == "L" else session.gid_rfoot
                swing_fn = _pair_fn(session.model, session.data, gid, session.gid_floor)
            if edge and session.lipm is not None and not session.bus.fault:
                for gid in (session.gid_lfoot, session.gid_rfoot):
                    if (
                        _pair_fn(session.model, session.data, gid, session.gid_floor) > 1.0
                        and _pair_fn(session.model, session.data, gid, rug) > 1.0
                    ):
                        edge_ticks += 1
                        break
            reason = session.bus.fault_reason if session.bus.fault else ""
            floor_l = wg.foot_floor_contact(
                session.model, session.data, session.bid_lf, session.gid_floor,
            )
            floor_r = wg.foot_floor_contact(
                session.model, session.data, session.bid_rf, session.gid_floor,
            )
            if not floor_l and not floor_r:
                old_hold += session.ctrl_dt
            else:
                old_hold = 0.0
            if old_air_t is None and old_hold >= 0.20 - 1e-9:
                old_air_t = t
                old_air_up = float(session._up_z())
                old_air_margin = float(session._support_margin())
            if not edge_7016 and abs(t - 7.016) <= session.ctrl_dt * 0.51:
                edge_7016 = _edge_line(session, rug)
            if abs(t - T_BAR) <= session.ctrl_dt * 0.51:
                up_7560 = float(session._up_z())
                margin_7560 = float(session._support_margin())
            rows.append(Tick(t, swing if swing in ("L", "R") else None, toe_z, swing_fn, reason))
            if mode in ("ankfollow", "phaseank") and 6.85 <= t <= 7.70:
                rug_rows.append(_rug_sample(session, rug, t, reason))
            if session.bus.fault and first_fault_t is None:
                first_fault_t = t
                first_fault = reason
                seen_fault = True
            if (
                abs(t - T_BAR) <= session.ctrl_dt * 0.51
                and session.bus.fault
                and "airborne" in (session.bus.fault_reason or "")
            ):
                air_at_bar = True
    finally:
        mj.mj_step = real_step
    toe_min, toe_t, toe_p90, n_mid, n_cyc, toe_frac, toe_side = _mid_swing(rows)
    toe_fn = 0.0
    for row in rows:
        if row.toe_z is not None and abs(row.t - toe_t) <= 1e-9:
            toe_fn = row.swing_fn
            break
    air_later = first_fault_t is not None and "airborne" in first_fault
    cop = float(session.max_cop_excursion)
    bars = {
        "no_airborne_at_7_560": not air_at_bar,
        "no_airborne_on_bout": not air_later,
        "mid_toe_above_0_002": bool(toe_min > TOE_BAR_M),
        "ankles_under_2_33": bool(abs(ank_peak) < ANKLE_NM),
        "knees_under_2_33": bool(abs(knee_peak) < KNEE_NM),
        "cop_in_box": bool(cop <= COP_BOX_M),
    }
    # Soft-pass is off. A fault that only moves past 7.560 s is still a miss.
    clear = all(bars[k] for k in (
        "no_airborne_on_bout", "mid_toe_above_0_002",
        "ankles_under_2_33", "knees_under_2_33", "cop_in_box",
    ))
    session.assert_plant_unchanged()
    return {
        "name": name,
        "gm_z_m": float(session.lipm.cfg.gm_z_m) if session.lipm is not None else None,
        "edge_pitch_rad": EDGE_PITCH if edge else 0.0,
        "toe_min_m": toe_min,
        "toe_min_t": toe_t,
        "toe_frac": toe_frac,
        "toe_side": toe_side,
        "toe_p90_m": toe_p90,
        "n_mid": n_mid,
        "n_cyc": n_cyc,
        "ank_peak_nm": ank_peak,
        "ank_name": ank_name,
        "ank_t": ank_t,
        "walk_ank_nm": walk_peak,
        "walk_ank_name": walk_name,
        "walk_ank_t": walk_t,
        "knee_peak_nm": knee_peak,
        "knee_name": knee_name,
        "knee_t": knee_t,
        "walk_knee_nm": walk_knee,
        "walk_knee_name": walk_knee_name,
        "walk_knee_t": walk_knee_t,
        "toe_fn_n": toe_fn,
        "edge_ticks": edge_ticks,
        "cop_m": cop,
        "min_up_z": float(session.min_up_z),
        "first_fault_t": first_fault_t,
        "first_fault": first_fault,
        "air_at_7_560": air_at_bar,
        "bars": bars,
        "clear": clear,
        "leg_peak_nm": leg_peak,
        "leg_name": leg_name,
        "leg_t": leg_t,
        "walk_leg_nm": walk_leg,
        "walk_leg_name": walk_leg_name,
        "walk_leg_t": walk_leg_t,
        "legs_under_2_33": bool(abs(walk_leg) < KNEE_NM),
        "hit_plant_rail": bool(abs(leg_peak) >= PLANT_NM - 1e-3),
        "roll_note": _roll_note(roll_rows),
        "hip4": {name: (tau, when) for name, (tau, when) in hip4.items()},
        "leg8": {name: (tau, when) for name, (tau, when) in leg8.items()},
        "leg8_under_2_33": all(abs(tau) < KNEE_NM for tau, _when in leg8.values()),
        "track1_clear": bool(
            toe_min > TOE_BAR_M and all(abs(tau) < KNEE_NM for tau, _when in leg8.values())
            and abs(walk_leg) < KNEE_NM
        ),
        "rug_note": _rug_note(rug_rows),
        "peak_frac": peak_frac,
        "lead_frac": lead_frac,
        "ank_writes": list(ANK_WRITES) if mode in ("ankfollow", "phaseank") else [],
        "snap_note": _snap_note(swing_snaps, toe_t),
        "gain": gain,
        "x": float(session.data.qpos[0]),
        "z_extra_m": z_extra,
        "cmd_toe_min_m": min((z for _t, z in cmd_leads), default=float("nan")),
        "cmd_toe_min_t": min(cmd_leads, key=lambda row: row[1])[0] if cmd_leads else float("nan"),
        "cmd_n": len(cmd_leads),
        "cmd_clear": bool(cmd_leads) and min(z for _t, z in cmd_leads) > CMD_CLEAR_M,
        "cmd_actual_m": next(
            (row.toe_z for row in rows if cmd_leads and abs(row.t - min(cmd_leads, key=lambda item: item[1])[0]) <= 1e-9 and row.toe_z is not None),
            float("nan"),
        ),
        "cmd_actual_fn": next(
            (row.swing_fn for row in rows if cmd_leads and abs(row.t - min(cmd_leads, key=lambda item: item[1])[0]) <= 1e-9),
            float("nan"),
        ),
        "drop_n": len(DROP_RESIDUAL) if mode == "dropflat" else 0,
        "drop_res_min": min(DROP_RESIDUAL) if mode == "dropflat" and DROP_RESIDUAL else float("nan"),
        "drop_res_max": max(DROP_RESIDUAL) if mode == "dropflat" and DROP_RESIDUAL else float("nan"),
        "old_air_t": old_air_t,
        "old_air_up": old_air_up,
        "old_air_margin": old_air_margin,
        "edge_7016": edge_7016,
        "up_7560": up_7560,
        "margin_7560": margin_7560,
    }


def measure_lag_split() -> None:
    """Split goal-sole minus actual-sole at the 12 mm / 20% mid-swing toe.

    The scored tick is the same 20–80% index the toe bar uses. Goal FK and
    actual FK share the live freejoint, so the pelvis drop, pitch, and roll
    of this gap are zero. Each swing joint then contributes (goal − q) times
    the sole-z slope. A path from actual to goal, one joint at a time, closes
    the remainder the linear slope leaves.
    """
    session = sw.SteerSession(
        video=False, scene_xml=Path(SCENE), lipm=sw.locked_kit_config(),
    )
    _install_phase_lift(session, 0.012, 0.20, 0.0)
    driver = sw.ScriptedDriver((
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(7.6, "vel", sw.VX_FWD_CAP, 0.0, "forward"),
    ))
    captured: list[dict[str, object]] = []
    while float(session.data.time) < 7.56:
        driver.publish(session.bus, float(session.data.time))
        session.step()
        lipm = session.lipm
        if lipm is None or lipm.op3 is None or lipm._gm_swing not in ("L", "R"):
            continue
        if session.bus.fault:
            break
        side = str(lipm._gm_swing)
        walker = lipm.op3
        saved_t = walker.time
        walker.time = _cmd_time(walker)
        joints, _info = walker.joints_now()
        walker.time = saved_t
        if joints is None:
            continue
        captured.append({
            "t": float(session.data.time),
            "side": side,
            "toe": _leading_toe_z(session, side),
            "qpos": np.array(session.data.qpos, dtype=np.float64, copy=True),
            "ctrl": np.array(session.data.ctrl, dtype=np.float64, copy=True),
            "joints": dict(joints),
        })
    cycles: list[list[dict[str, object]]] = []
    cur: list[dict[str, object]] = []
    for row in captured:
        if cur and cur[-1]["side"] != row["side"]:
            cycles.append(cur)
            cur = []
        cur.append(row)
    if cur:
        cycles.append(cur)
    mids: list[tuple[dict[str, object], float]] = []
    for cyc in cycles:
        n = len(cyc)
        if n < 2:
            continue
        for i, row in enumerate(cyc):
            frac = i / (n - 1)
            if 0.20 - 1e-12 <= frac <= 0.80 + 1e-12:
                mids.append((row, frac))
    if not mids:
        print("lag split: no mid-swing tick")
        return
    best, frac = min(mids, key=lambda item: float(item[0]["toe"]))
    side = str(best["side"])
    session.data.qpos[:] = np.array(best["qpos"], dtype=np.float64, copy=True)
    mj.mj_forward(session.model, session.data)
    names = tuple(
        f"{'r' if side == 'R' else 'l'}_{jn}"
        for jn in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll")
    )
    joints = best["joints"]
    assert isinstance(joints, dict)
    goal_ctrl = np.array(best["ctrl"], dtype=np.float64, copy=True)
    actual_ctrl = np.array(best["ctrl"], dtype=np.float64, copy=True)
    for name in names:
        jid = mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_JOINT, name)
        adr = int(session.model.jnt_qposadr[jid])
        actual_ctrl[session.act_idx[name + "_pos"]] = float(session.data.qpos[adr])
        goal_ctrl[session.act_idx[name + "_pos"]] = float(joints[name])
    live_actual = _fk_sole(session, side, actual_ctrl, None)
    live_goal = _fk_sole(session, side, goal_ctrl, None)
    drop = float(live_goal["body_z"] - live_actual["body_z"])
    pitch_term = _pitch_about_body_y(live_goal, live_actual)
    roll_term = _roll_about_body_x(live_goal, live_actual)
    gap = float(live_goal["lead"] - live_actual["lead"])
    pelvis = drop + pitch_term + roll_term
    session.data.qpos[:] = best["qpos"]
    mj.mj_forward(session.model, session.data)
    base = _leading_toe_z(session, side)
    eps = 1e-4
    rows: list[tuple[str, float, float, float, float, float]] = []
    linear = 0.0
    for name in names:
        jid = mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_JOINT, name)
        adr = int(session.model.jnt_qposadr[jid])
        q = float(session.data.qpos[adr])
        goal = float(joints[name])
        session.data.qpos[adr] = q + eps
        mj.mj_forward(session.model, session.data)
        slope = (_leading_toe_z(session, side) - base) / eps
        session.data.qpos[adr] = q
        mj.mj_forward(session.model, session.data)
        ctrl = float(np.asarray(best["ctrl"])[session.act_idx[name + "_pos"]])
        part = slope * (goal - q)
        linear += part
        rows.append((name, q, goal, ctrl, slope, part))
    session.data.qpos[:] = best["qpos"]
    mj.mj_forward(session.model, session.data)
    path_sum = 0.0
    path_rows: list[tuple[str, float]] = []
    z_prev = _leading_toe_z(session, side)
    for name in names:
        jid = mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_JOINT, name)
        adr = int(session.model.jnt_qposadr[jid])
        session.data.qpos[adr] = float(joints[name])
        mj.mj_forward(session.model, session.data)
        z_now = _leading_toe_z(session, side)
        path_rows.append((name, z_now - z_prev))
        path_sum += z_now - z_prev
        z_prev = z_now
    scored = float(best["toe"])
    matched = float(live_actual["lead"])
    goal_z = float(live_goal["lead"])
    print(
        f"lag split t {float(best['t']):.3f} side {side} mid-frac {frac:.3f} "
        f"scored toe {scored * 1000:+.2f} mm "
        f"qpos toe {matched * 1000:+.2f} mm "
        f"goal {goal_z * 1000:+.2f} mm"
    )
    print(
        f"geom lag (qpos toe - scored toe) {(matched - scored) * 1000:+.2f} mm  "
        f"joint gap (goal - qpos toe) {(goal_z - matched) * 1000:+.2f} mm  "
        f"scored gap (goal - scored toe) {(goal_z - scored) * 1000:+.2f} mm"
    )
    print(
        f"pelvis drop {drop * 1000:+.2f} mm  pitch {pitch_term * 1000:+.2f} mm  "
        f"roll {roll_term * 1000:+.2f} mm  pelvis sum {pelvis * 1000:+.2f} mm  "
        f"same freejoint body_z {float(live_actual['body_z']):.4f}"
    )
    for name, q, goal, ctrl, slope, part in rows:
        act = session.act_idx[name + "_pos"]
        kp = float(session.model.actuator_gainprm[act, 0])
        stall = 2.33 / kp
        hold = abs(goal - q) * kp
        print(
            f"  {name} q {q:+.4f} goal {goal:+.4f} ctrl {ctrl:+.4f} "
            f"dq {goal - q:+.4f} dz/dq {slope:+.4f} "
            f"part {part * 1000:+.2f} mm  hold {hold:.2f} Nm  stall {stall:.4f} rad"
        )
    print(
        f"linear sum {linear * 1000:+.2f} mm  pelvis {pelvis * 1000:+.2f} mm  "
        f"linear residual {(gap - pelvis - linear) * 1000:+.2f} mm"
    )
    for name, part in path_rows:
        print(f"  path {name} {part * 1000:+.2f} mm")
    print(f"path sum {path_sum * 1000:+.2f} mm  exact gap {gap * 1000:+.2f} mm")
    session.assert_plant_unchanged()


def measure_scuff() -> None:
    """Hip pitch and knee over the swing that holds t = 3.384 s."""
    session = sw.SteerSession(
        video=False, scene_xml=Path(SCENE), lipm=sw.locked_kit_config(),
    )
    lipm = session.lipm
    assert lipm is not None
    driver = sw.ScriptedDriver((
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(4.2, "vel", sw.VX_FWD_CAP, 0.0, "forward"),
    ))
    goal: dict[str, float] = {}
    orig = lipm._tick_gait_manager

    def wrapped(walking: bool) -> None:
        orig(walking)
        swing = lipm._gm_swing
        if swing not in ("L", "R"):
            goal.clear()
            return
        pref = "l_" if swing == "L" else "r_"
        goal["side"] = 1.0 if swing == "L" else 0.0
        goal["knee"] = float(session.data.ctrl[session.act_idx[pref + "knee_pos"]])
        goal["hip"] = float(session.data.ctrl[session.act_idx[pref + "hip_pitch_pos"]])

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]
    phys: list[tuple[float, str, float, float, float, float]] = []
    ticks: list[tuple[float, str, float, float, float, float, float]] = []
    real_step = mj.mj_step

    def hook(model: mj.MjModel, data: mj.MjData) -> None:
        real_step(model, data)
        side = lipm._gm_swing
        if side not in ("L", "R"):
            return
        pref = "l_" if side == "L" else "r_"
        hip = float(data.actuator_force[session.act_idx[pref + "hip_pitch_pos"]])
        knee = float(data.actuator_force[session.act_idx[pref + "knee_pos"]])
        knee_q = lipm.q(pref + "knee")
        knee_ctrl = float(data.ctrl[session.act_idx[pref + "knee_pos"]])
        phys.append((float(data.time), side, hip, knee, knee_q, knee_ctrl))

    mj.mj_step = hook
    try:
        while float(session.data.time) < 4.2 - 1e-9:
            driver.publish(session.bus, float(session.data.time))
            session.step()
            t = float(session.data.time)
            swing = lipm._gm_swing
            if swing not in ("L", "R") or "knee" not in goal:
                continue
            pref = "l_" if swing == "L" else "r_"
            ticks.append((
                t,
                swing,
                _leading_toe_z(session, swing),
                lipm.q(pref + "knee"),
                float(goal["knee"]),
                lipm.q(pref + "hip_pitch"),
                float(goal["hip"]),
            ))
    finally:
        mj.mj_step = real_step
    cycles: list[list[tuple[float, str, float, float, float, float, float]]] = []
    cur: list[tuple[float, str, float, float, float, float, float]] = []
    for row in ticks:
        if cur and cur[-1][1] != row[1]:
            cycles.append(cur)
            cur = []
        cur.append(row)
    if cur:
        cycles.append(cur)
    hit = None
    for cyc in cycles:
        if any(abs(row[0] - 3.384) <= session.ctrl_dt * 0.51 for row in cyc):
            hit = cyc
            break
    if hit is None:
        print("SCUFF cycle missing")
        return
    t0 = hit[0][0]
    t1 = hit[-1][0]
    mark = min(hit, key=lambda row: abs(row[0] - 3.384))
    in_phase = [p for p in phys if t0 - 1e-9 <= p[0] <= t1 + 1e-6 and p[1] == hit[0][1]]
    hip_peak = max(in_phase, key=lambda p: abs(p[2]))
    knee_peak = max(in_phase, key=lambda p: abs(p[3]))
    near = min(in_phase, key=lambda p: abs(p[0] - mark[0]))
    applied = [p[5] - p[4] for p in in_phase]
    knee_err = [row[4] - row[3] for row in hit]
    print("SCUFF_FOOT", hit[0][1])
    print(f"SCUFF_WINDOW {t0:.3f} {t1:.3f} n {len(hit)}")
    print(f"SCUFF_MARK t {mark[0]:.3f} toe {mark[2]:.6f} knee_q {mark[3]:.4f} knee_cmd {mark[4]:.4f} hip_q {mark[5]:.4f} hip_cmd {mark[6]:.4f}")
    print(f"SCUFF_AT_MARK hip_tau {near[2]:.3f} knee_tau {near[3]:.3f} t {near[0]:.3f}")
    print(f"SCUFF_HIP_PEAK {hip_peak[2]:.3f} t {hip_peak[0]:.3f}")
    print(f"SCUFF_KNEE_PEAK {knee_peak[3]:.3f} t {knee_peak[0]:.3f}")
    print(f"SCUFF_KNEE_GOAL_ERR max {max(knee_err):.4f} min {min(knee_err):.4f} at_mark {mark[4] - mark[3]:.4f}")
    print(f"SCUFF_KNEE_APPLIED_ERR max {max(applied):.4f} min {min(applied):.4f} at_mark {near[5] - near[4]:.4f}")
    print(f"SCUFF_APPLIED_AT_MARK knee_q {near[4]:.4f} knee_ctrl {near[5]:.4f}")
    print(f"SCUFF_OVER_2_33 hip {abs(hip_peak[2]) >= KNEE_NM} knee {abs(knee_peak[3]) >= KNEE_NM}")
    print(f"SCUFF_ON_2_45 hip {abs(hip_peak[2]) >= PLANT_NM - 0.002} knee {abs(knee_peak[3]) >= PLANT_NM - 0.002}")
    session.assert_plant_unchanged()


def _fk_leading_toe(session: sw.SteerSession, side: str, ctrl: np.ndarray) -> tuple[float, float]:
    """World z of the leading bottom corner, and the lowest corner, at ``ctrl``."""
    data = session.data
    model = session.model
    saved = np.array(data.qpos, dtype=np.float64, copy=True)
    pref = "r_" if side == "R" else "l_"
    for jn in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll"):
        name = pref + jn
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        adr = int(model.jnt_qposadr[jid])
        data.qpos[adr] = float(ctrl[session.act_idx[name + "_pos"]])
    mj.mj_forward(model, data)
    lead = _leading_toe_z(session, side)
    bid = session.bid_rf if side == "R" else session.bid_lf
    gid = session.gid_rfoot if side == "R" else session.gid_lfoot
    lows = [float(c[2]) for c in _box_bottom_corners(model, data, bid, gid)]
    data.qpos[:] = saved
    mj.mj_forward(model, data)
    return lead, min(lows)


def _fk_sole(
    session: sw.SteerSession,
    side: str,
    ctrl: np.ndarray,
    freejoint: np.ndarray | None,
) -> dict[str, float]:
    """Leading-bottom z for ``ctrl``. ``freejoint`` replaces the live root when set."""
    from mono_toe_gate import torso_pitch

    data = session.data
    model = session.model
    saved = np.array(data.qpos, dtype=np.float64, copy=True)
    if freejoint is not None:
        data.qpos[0:7] = freejoint
    pref = "r_" if side == "R" else "l_"
    for jn in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll"):
        name = pref + jn
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        adr = int(model.jnt_qposadr[jid])
        data.qpos[adr] = float(ctrl[session.act_idx[name + "_pos"]])
    mj.mj_forward(model, data)
    bid = session.bid_rf if side == "R" else session.bid_lf
    gid = session.gid_rfoot if side == "R" else session.gid_lfoot
    lead = _leading_toe_z(session, side)
    origin = np.asarray(data.xpos[bid], dtype=np.float64)
    rot = np.asarray(data.xmat[bid], dtype=np.float64).reshape(3, 3)
    fwd = body_forward_xy(data, session.bid_body)
    body = np.asarray(data.xpos[session.bid_body], dtype=np.float64).copy()
    body_r = np.asarray(data.xmat[session.bid_body], dtype=np.float64).reshape(3, 3).copy()
    best_off = -1e9
    best_local_z = 0.0
    best_corner = np.zeros(3, dtype=np.float64)
    for corner in _box_bottom_corners(model, data, bid, gid):
        off = float(np.dot(corner[:2] - body[:2], fwd))
        if off > best_off:
            best_off = off
            best_corner = np.asarray(corner, dtype=np.float64).copy()
            best_local_z = float((rot.T @ (best_corner - origin))[2])
    joints = []
    for jn in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll"):
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, pref + jn)
        joints.append(float(data.qpos[int(model.jnt_qposadr[jid])]))
    out = {
        "lead": lead,
        "ank_z": float(origin[2]),
        "local_z": best_local_z,
        "body_z": float(body[2]),
        "pitch": torso_pitch(data, session.bid_body),
        "corner": best_corner,
        "body": body,
        "body_r": body_r,
        "p_body": body_r.T @ (best_corner - body),
        "joints": np.asarray(joints, dtype=np.float64),
    }
    data.qpos[:] = saved
    mj.mj_forward(model, data)
    return out


def measure_phase_cmd() -> None:
    """Commanded sole at 30/50/70% of the 3.344–3.536 s right swing, two pelvis frames."""
    session = sw.SteerSession(
        video=False, scene_xml=Path(SCENE), lipm=sw.locked_kit_config(),
    )
    lipm = session.lipm
    assert lipm is not None
    for side, gid in (("L", session.gid_lfoot), ("R", session.gid_rfoot)):
        pos = np.asarray(session.model.geom_pos[gid], dtype=np.float64)
        half = np.asarray(session.model.geom_size[gid], dtype=np.float64)
        print(
            f"PLANT {side} pos_z {pos[2]:.4f} half_z {half[2]:.4f} "
            f"bottom {pos[2] - half[2]:.4f} half_xy {half[0]:.4f} {half[1]:.4f} "
            f"match {abs(half[0] - sw.FOOT_HALF_X) < 1e-6 and abs(half[1] - sw.FOOT_HALF_Y) < 1e-6}"
        )
    driver = sw.ScriptedDriver((
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(3.7, "vel", sw.VX_FWD_CAP, 0.0, "forward"),
    ))
    goal = np.zeros(session.model.nu, dtype=np.float64)
    have = {"ok": False}
    orig = lipm._tick_gait_manager
    stand_free: np.ndarray | None = None

    def wrapped(walking: bool) -> None:
        orig(walking)
        goal[:] = np.array(session.data.ctrl, dtype=np.float64, copy=True)
        have["ok"] = lipm._gm_swing == "R"

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]
    rows: list[tuple[float, np.ndarray, np.ndarray]] = []
    stance_roll: list[tuple[float, float, float, float, float, float, float]] = []
    while float(session.data.time) < 3.7 - 1e-9:
        driver.publish(session.bus, float(session.data.time))
        session.step()
        t = float(session.data.time)
        if t < 1.0 - 1e-9:
            from mono_toe_gate import torso_pitch
            stand_free = np.array(session.data.qpos[0:7], dtype=np.float64, copy=True)
            stand_body_z = float(session.data.xpos[session.bid_body][2])
            stand_pitch = torso_pitch(session.data, session.bid_body)
        if have["ok"] and 3.30 < t < 3.60:
            rows.append((t, goal.copy(), np.array(session.data.qpos[0:7], dtype=np.float64, copy=True)))
            stance_roll.append(_stance_roll_tick(session, t))
    assert stand_free is not None
    # The published window is the contiguous right swing around 3.384 s.
    hit = [row for row in rows if 3.344 - 1e-3 <= row[0] <= 3.536 + 1e-3]
    if len(hit) < 3:
        print("PHASE rows", len(hit))
        return
    t0 = hit[0][0]
    t1 = hit[-1][0]
    span = t1 - t0
    print(
        f"PHASE_WINDOW {t0:.3f} {t1:.3f} n {len(hit)} "
        f"stand_body_z {stand_body_z:.6f} stand_pitch {stand_pitch:.4f}"
    )
    samples: list[tuple[float, float, dict[str, float], dict[str, float]]] = []
    for t, ctrl, live_free in hit:
        frac = 0.0 if span <= 1e-9 else (t - t0) / span
        live = _fk_sole(session, "R", ctrl, live_free)
        nominal = _fk_sole(session, "R", ctrl, stand_free)
        samples.append((t, frac, live, nominal))
        print(
            f"PHASE t {t:.3f} frac {frac:.3f} live {live['lead']:.6f} "
            f"nominal {nominal['lead']:.6f} ank {live['ank_z']:.6f} "
            f"local_z {live['local_z']:.6f} body_z {live['body_z']:.6f} "
            f"pitch {live['pitch']:.4f}"
        )
    for target in (0.30, 0.50, 0.70):
        t, frac, live, nominal = min(samples, key=lambda row: abs(row[1] - target))
        print(
            f"PHASE_PICK {target:.2f} t {t:.3f} frac {frac:.3f} "
            f"live {live['lead']:.6f} nominal {nominal['lead']:.6f} "
            f"delta {live['lead'] - nominal['lead']:.6f} "
            f"body_z {live['body_z']:.6f} pitch {live['pitch']:.4f} "
            f"local_z {live['local_z']:.6f} ank_minus_lead {live['ank_z'] - live['lead']:.6f}"
        )
    _print_gap_split(min(samples, key=lambda row: abs(row[0] - 3.384)))
    _print_stance_roll([row for row in stance_roll if 3.344 - 1e-3 <= row[0] <= 3.536 + 1e-3])
    session.assert_plant_unchanged()


def _stance_roll_tick(session: sw.SteerSession, t: float) -> tuple[float, float, float, float, float, float, float]:
    """Left hip-roll and ankle-roll: q, ctrl, force. Left is the stance leg."""
    hip_q, hip_c, hip_f = _joint_qcf(session, "l_hip_roll")
    ank_q, ank_c, ank_f = _joint_qcf(session, "l_ank_roll")
    return (t, hip_q, hip_c, hip_f, ank_q, ank_c, ank_f)


def _joint_qcf(session: sw.SteerSession, name: str) -> tuple[float, float, float]:
    model = session.model
    data = session.data
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
    q = float(data.qpos[int(model.jnt_qposadr[jid])])
    idx = session.act_idx[name + "_pos"]
    return q, float(data.ctrl[idx]), float(data.actuator_force[idx])


# Hardware's stated position-loop gains. The print reads the compiled
# model and only uses these as a comparison. They are not written back.
_STATED_KP_KV: dict[str, tuple[float, float]] = {
    "hip_roll": (40.0, 1.538),
    "hip_pitch": (45.0, 1.551),
}
_CTRL_EQ_RAD = 1e-4
_DAMP_EQ_RAD_S = 0.05


def _install_ik_track(session: sw.SteerSession, *, hips_only: bool = False) -> None:
    """Write the kinematic target straight into ``data.ctrl``.

    On the gait-manager clock the active approach is ``gm_move_s`` (20 ms)
    plus the 5.5 rad/s cap, on hip pitch, knee, and ankle pitch.
    ``HIP_KNEE_MOVE_S`` (150 ms) is the Bézier clock and is not this path.
    Hip roll is outside that approach. ``write_clipped`` holds it inside
    ``0.98 * forcerange / kp`` of q. This copy stores the kinematic target
    and holds ctrl there for the physics steps. ``hips_only`` does that
    for hip roll and hip pitch, and leaves the knee and ankle approach.
    kp, kv, dampratio, and forcerange stay.
    """
    lipm = session.lipm
    if lipm is None:
        return
    full_suffix = ("hip_roll", "hip_pitch") if hips_only else (
        "hip_roll", "hip_pitch", "knee", "ank_pitch",
    )
    orig_write = lipm.write_clipped

    def write_full(jn: str, q_des: float) -> None:
        if jn.endswith(full_suffix):
            act = f"{jn}_pos"
            idx = lipm.act_idx.get(act)
            if idx is None:
                return
            lo = float(session.model.actuator_ctrlrange[idx, 0])
            hi = float(session.model.actuator_ctrlrange[idx, 1])
            session.data.ctrl[idx] = min(hi, max(lo, float(q_des)))
            return
        orig_write(jn, q_des)

    lipm.write_clipped = write_full  # type: ignore[method-assign]
    if hips_only:
        session._move_ctrl_idx = [
            idx
            for name, idx in session.act_idx.items()
            if name.endswith(("knee_pos", "ank_pitch_pos"))
        ]
        return
    orig_sub = session._lipm_substep

    def substep(ctrl_from: np.ndarray | None = None) -> None:
        walking = lipm.phase != "stand"
        if not walking:
            orig_sub(ctrl_from)
            return
        n = session.steps_per_ctrl
        ctrl_to = np.array(session.data.ctrl, dtype=np.float64, copy=True)
        for _i in range(n):
            session.data.ctrl[:] = ctrl_to
            session.data.qfrc_applied[:] = 0.0
            session.data.xfrc_applied[:] = 0.0
            mj.mj_step(session.model, session.data)

    session._lipm_substep = substep  # type: ignore[method-assign]


def _prior_lead_lines(span_s: float) -> list[str]:
    """What the scored ±lead / 8 ms / 12 ms copies actually moved.

    Both are a time shift of the foot-z bump. Neither adds kv/kp seconds
    onto the swing-hip command. ``lead_s`` is seconds. ``lead_frac`` is
    a fraction of this swing's SSP window, which is 0.200 s here.
    """
    ms_per = span_s * 1000.0
    return [
        (
            "prior ±lead / 8 ms / 12 ms copies move the foot-z bump earlier. "
            "They do not advance hip_roll or hip_pitch ctrl. "
            f"This walker's single-support window is {span_s:.3f} s "
            f"(r_ssp_end - r_ssp_start). Both swings in the 0.500 s period "
            "add to 0.400 s; the bump does not use that sum. "
            "lead_s is clamped at 0, so there is no later lead."
        ),
        (
            "20 ms and 30 ms copies: _install_phase_lift lead_frac on the 12 mm "
            f"@ 20% parabola. Seconds early = lead_frac * {span_s:.3f} s. "
            f"30 ms on this window is lead_frac {0.030 / span_s:.4f}. "
            f"20 ms is lead_frac {0.020 / span_s:.4f}. "
            "A lead_frac taken from a 0.400 s span (0.075 for 30 ms, 0.050 for 20 ms) "
            f"is only {0.075 * ms_per:.1f} ms and {0.050 * ms_per:.1f} ms here. "
            "The call argument is not stored. The scored writeup called them "
            "30 ms and 20 ms before toe-off: toe +0.508 mm and x 0.820 m at 30 ms; "
            "20 ms railed left hip pitch -2.45 Nm, right hip pitch +2.45 Nm, "
            "right knee +2.45 Nm, min up_z 0.631."
        ),
        (
            "8 ms and 12 ms copies: _install_slow_rise lead_s 0.008 and 0.012 "
            "on the 8 mm smoothstep. Those arguments are seconds before toe-off, "
            "not a fraction of the swing. Toes -2.10 mm and -2.04 mm, x 1.031 m. "
            "Not a kv/kp advance of the swing hips."
        ),
    ]


def _servo_filter(
    ik: float,
    published: float,
    written: float,
    ctrl: float,
) -> str:
    parts: list[str] = []
    if abs(published - ik) > _CTRL_EQ_RAD:
        parts.append("lead_morph")
    if abs(written - published) > _CTRL_EQ_RAD:
        parts.append("write_clipped_band")
    if abs(ctrl - written) > _CTRL_EQ_RAD:
        parts.append("move_slew")
    if not parts:
        return "none" if abs(ctrl - ik) <= _CTRL_EQ_RAD else "unattributed"
    return "+".join(parts)


def measure_hip_servo(
    *,
    track_ik: bool = False,
    hips_only: bool = False,
    t_end: float = 3.05,
) -> None:
    """Print swing-hip IK, ctrl, q, qd, and force at 2.872 s on 12 mm @ 20%.

    Force is the value MuJoCo computed at the start of the last physics
    step, so the damping check uses that same q and qd. The post-step
    q at data.time 2.872 is printed beside it. ``track_ik`` drops the
    gait-manager approach and the hip-roll band so ctrl can equal IK.
    """
    import op3_walk

    cfg = sw.locked_kit_config()
    session = sw.SteerSession(video=False, scene_xml=Path(SCENE), lipm=cfg)
    _install_phase_lift(session, 0.012, 0.20, 0.0)
    if track_ik:
        _install_ik_track(session, hips_only=hips_only)
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        raise RuntimeError("gait manager walker missing")
    walker = lipm.op3
    span = float(walker.r_ssp_end - walker.r_ssp_start)
    import lipm_gait
    if lipm.cfg.schedule == "gait_manager":
        move_s = max(float(lipm.cfg.gm_move_s), float(session.ctrl_dt))
    else:
        move_s = max(float(lipm_gait.HIP_KNEE_MOVE_S), float(session.ctrl_dt))
    driver = sw.ScriptedDriver((
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(t_end, "vel", sw.VX_FWD_CAP, 0.0, "forward"),
    ))
    joints_watch = ("r_hip_roll", "r_hip_pitch", "l_hip_roll", "l_hip_pitch")
    published: dict[str, float] = {}
    lead_bleed = False
    lead_err = 0.0
    written = np.zeros(session.model.nu, dtype=np.float64)
    ctrl_from = np.zeros(session.model.nu, dtype=np.float64)
    have_write = False

    orig_walker_step = walker.step

    def _step(dt: float) -> tuple[dict[str, float] | None, op3_walk.StepInfo]:
        nonlocal lead_bleed, lead_err
        joints, info = orig_walker_step(dt)
        lead_bleed = bool(walker.lead_blend_tick)
        err = walker._lead_err
        lead_err = 0.0 if err is None else max(abs(float(v)) for v in err.values())
        published.clear()
        if joints is not None:
            for name, val in joints.items():
                published[name] = float(val)
        return joints, info

    walker.step = _step  # type: ignore[method-assign]
    orig_sub = session._lipm_substep

    def _sub(ctrl_src: np.ndarray | None = None) -> None:
        nonlocal have_write
        written[:] = np.array(session.data.ctrl, dtype=np.float64, copy=True)
        if ctrl_src is None:
            ctrl_from[:] = written
        else:
            ctrl_from[:] = np.array(ctrl_src, dtype=np.float64, copy=True)
        have_write = True
        orig_sub(ctrl_src)

    session._lipm_substep = _sub  # type: ignore[method-assign]

    @dataclass
    class _Pre:
        t_pre: float
        q: float
        qd: float
        ctrl: float

    pre: dict[str, _Pre] = {}
    real_step = mj.mj_step

    def _hook(model: mj.MjModel, data: mj.MjData) -> None:
        for jn in joints_watch:
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
            idx = session.act_idx[jn + "_pos"]
            pre[jn] = _Pre(
                t_pre=float(data.time),
                q=float(data.qpos[int(model.jnt_qposadr[jid])]),
                qd=float(data.qvel[int(model.jnt_dofadr[jid])]),
                ctrl=float(data.ctrl[idx]),
            )
        real_step(model, data)

    mj.mj_step = _hook
    rows: list[Tick] = []
    leg8 = {name: (0.0, 0.0) for name in LEG8}
    up_7560 = float("nan")
    margin_7560 = float("nan")
    first_fault_t: float | None = None
    first_fault = ""
    servo_at: dict[float, str] = {}
    try:
        while float(session.data.time) < t_end - 1e-9:
            driver.publish(session.bus, float(session.data.time))
            session.step()
            t = float(session.data.time)
            swing = lipm._gm_swing if lipm._gm_swing in ("L", "R") else None
            toe_z = _leading_toe_z(session, swing) if swing in ("L", "R") else None
            if not session.bus.fault:
                for name in leg8:
                    idx = session.act_idx[name]
                    tau = float(session.data.actuator_force[idx])
                    prev, _when = leg8[name]
                    if abs(tau) > abs(prev):
                        leg8[name] = (tau, t)
            if abs(t - T_BAR) <= session.ctrl_dt * 0.51:
                up_7560 = float(session._up_z())
                margin_7560 = float(session._support_margin())
            if session.bus.fault and first_fault_t is None:
                first_fault_t = t
                first_fault = session.bus.fault_reason or ""
            rows.append(Tick(
                t, swing if swing in ("L", "R") else None, toe_z, 0.0, first_fault,
            ))
            want = abs(t - 2.872) <= session.ctrl_dt * 0.51 or t_end >= T_BAR
            if want:
                servo_at[t] = _format_servo_tick(
                    session, walker, t, swing, toe_z, published, written, ctrl_from,
                    have_write, pre, lead_bleed, lead_err, move_s, track_ik,
                )
    finally:
        mj.mj_step = real_step
    print(
        f"HIP_SERVO track_ik {int(track_ik)} hips_only {int(hips_only)} "
        f"schedule {lipm.cfg.schedule}"
    )
    print(
        f"HIP_SERVO move_s {move_s:.3f} bezier_move_s {lipm_gait.HIP_KNEE_MOVE_S:.3f} "
        f"hx_slew {lipm_gait.HX35_SLEW_RAD_S:.1f} ctrl_dt {session.ctrl_dt:.3f} "
        f"gm_move_s {lipm.cfg.gm_move_s:.3f}"
    )
    for line in _prior_lead_lines(span):
        print(f"HIP_SERVO {line}")
    at_2872 = min(servo_at, key=lambda key: abs(key - 2.872), default=None)
    if at_2872 is None or abs(at_2872 - 2.872) > session.ctrl_dt:
        print("HIP_SERVO missed 2.872")
    elif t_end < T_BAR:
        print(servo_at[at_2872])
    if t_end >= T_BAR:
        toe_min, toe_t, _p90, n_mid, n_cyc, toe_frac, toe_side = _mid_swing(rows)
        over = [name for name, (tau, _when) in leg8.items() if abs(tau) >= KNEE_NM]
        rail = [name for name, (tau, _when) in leg8.items() if abs(tau) >= PLANT_NM - 1e-3]
        print(
            f"HIP_SERVO score toe {toe_min * 1000:.3f} mm t {toe_t:.3f} "
            f"frac {toe_frac:.3f} side {toe_side} n_mid {n_mid} n_cyc {n_cyc} "
            f"x {float(session.data.qpos[0]):.3f} fault {first_fault_t} {first_fault} "
            f"up7560 {up_7560:.4f} margin7560 {margin_7560:.4f} "
            f"min_up {float(session.min_up_z):.4f}"
        )
        for name, (tau, when) in leg8.items():
            print(f"HIP_SERVO leg {name} {tau:+.4f} t {when:.3f}")
        print(f"HIP_SERVO over_2_33 {over} rail {rail}")
        if at_2872 is not None and abs(at_2872 - 2.872) <= session.ctrl_dt:
            print("HIP_SERVO sample 2.872")
            print(servo_at[at_2872])
        if servo_at:
            at_toe = min(servo_at, key=lambda key: abs(key - toe_t))
            if abs(at_toe - toe_t) <= session.ctrl_dt and abs(at_toe - 2.872) > session.ctrl_dt:
                print(f"HIP_SERVO sample min-toe {at_toe:.3f}")
                print(servo_at[at_toe])
    session.assert_plant_unchanged()


def _format_servo_tick(
    session: sw.SteerSession,
    walker: object,
    t: float,
    swing: str | None,
    toe_z: float | None,
    published: dict[str, float],
    written: np.ndarray,
    ctrl_from: np.ndarray,
    have_write: bool,
    pre: dict[str, object],
    lead_bleed: bool,
    lead_err: float,
    move_s: float,
    track_ik: bool,
) -> str:
    import lipm_gait

    saved_t = float(walker.time)  # type: ignore[attr-defined]
    walker.time = _cmd_time(walker)  # type: ignore[attr-defined]
    joints, _info = walker.joints_now()  # type: ignore[attr-defined]
    walker.time = saved_t  # type: ignore[attr-defined]
    frac = min(1.0, float(session.ctrl_dt) / move_s)
    slew = float(lipm_gait.HX35_SLEW_RAD_S) * float(session.ctrl_dt)
    toe_mm = float("nan") if toe_z is None else toe_z * 1000.0
    lines = [
        (
            f"HIP_SERVO t {t:.3f} swing {swing} toe {toe_mm:.3f} mm "
            f"track_ik {int(track_ik)} lead_bleed {int(lead_bleed)} lead_err {lead_err:.5f} "
            f"have_write {int(have_write)}"
        ),
    ]
    if joints is None:
        lines.append("HIP_SERVO ik missing")
        return "\n".join(lines)
    for jn in ("r_hip_roll", "r_hip_pitch", "l_hip_roll", "l_hip_pitch"):
        kind = "hip_roll" if jn.endswith("hip_roll") else "hip_pitch"
        idx = session.act_idx[jn + "_pos"]
        ik = float(joints[jn])
        pub = float(published.get(jn, float("nan")))
        wrote = float(written[idx]) if have_write else float("nan")
        src = float(ctrl_from[idx]) if have_write else float("nan")
        q_post, ctrl, force = _joint_qcf(session, jn)
        jid = mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_JOINT, jn)
        qd_post = float(session.data.qvel[int(session.model.jnt_dofadr[jid])])
        sample = pre.get(jn)
        t_pre = q_force = qd_force = ctrl_force = float("nan")
        if sample is not None and hasattr(sample, "q"):
            t_pre = float(sample.t_pre)  # type: ignore[attr-defined]
            q_force = float(sample.q)  # type: ignore[attr-defined]
            qd_force = float(sample.qd)  # type: ignore[attr-defined]
            ctrl_force = float(sample.ctrl)  # type: ignore[attr-defined]
        kp = float(session.model.actuator_gainprm[idx, 0])
        bias0 = float(session.model.actuator_biasprm[idx, 0])
        bias1 = float(session.model.actuator_biasprm[idx, 1])
        kv = -float(session.model.actuator_biasprm[idx, 2])
        stated_kp, stated_kv = _STATED_KP_KV[kind]
        in_slew = idx in session._move_ctrl_idx
        e_sat = float(session.lipm.e_sat.get(jn + "_pos", float("nan"))) if session.lipm is not None else float("nan")
        delta = wrote - src
        step = delta * frac
        if step > slew:
            step = slew
        elif step < -slew:
            step = -slew
        expected = src + step if in_slew else wrote
        ident = (kp * (ctrl_force - q_force) - force) / kv if abs(kv) > 1e-9 else float("nan")
        ident_post = (kp * (ctrl - q_post) - force) / kv if abs(kv) > 1e-9 else float("nan")
        ctrl_eq = abs(ctrl - ik) <= _CTRL_EQ_RAD
        ident_ok = abs(ident - qd_force) <= _DAMP_EQ_RAD_S
        filt = _servo_filter(ik, pub, wrote, ctrl)
        clip = " CLIP" if abs(force) >= PLANT_NM - 0.01 else ""
        lines.append(
            f"HIP_SERVO {jn} ik {ik:.6f} published {pub:.6f} written {wrote:.6f} "
            f"ctrl {ctrl:.6f} ctrl_at_force {ctrl_force:.6f} "
            f"q_at_force {q_force:.6f} qd_at_force {qd_force:.6f} "
            f"q {q_post:.6f} qd {qd_post:.6f} force {force:.6f}{clip} "
            f"t_force {t_pre:.3f}"
        )
        lines.append(
            f"HIP_SERVO {jn} kp {kp:.4f} kv {kv:.4f} bias0 {bias0:.4f} bias1 {bias1:.4f} "
            f"stated_kp {stated_kp:.3f} stated_kv {stated_kv:.3f} "
            f"kp_match {int(abs(kp - stated_kp) <= 1e-3)} kv_match {int(abs(kv - stated_kv) <= 1e-3)} "
            f"in_move_slew {int(in_slew)} e_sat {e_sat:.6f} "
            f"ctrl_from {src:.6f} slew_expected {expected:.6f} "
            f"ctrl_eq_ik {int(ctrl_eq)} filter {filt} "
            f"ctrl_minus_ik {ctrl - ik:.6f} "
            f"ident_qd {ident:.6f} qd_at_force {qd_force:.6f} ident_ok {int(ident_ok)} "
            f"ident_post {ident_post:.6f} qd_post {qd_post:.6f}"
        )
    return "\n".join(lines)


def _print_stance_roll(rows: list[tuple[float, float, float, float, float, float, float]]) -> None:
    if not rows:
        print("STANCE_ROLL empty")
        return
    hip_f = max(rows, key=lambda row: abs(row[3]))
    ank_f = max(rows, key=lambda row: abs(row[6]))
    hip_err = max(rows, key=lambda row: abs(row[2] - row[1]))
    ank_err = max(rows, key=lambda row: abs(row[5] - row[4]))
    at = min(rows, key=lambda row: abs(row[0] - 3.384))
    print(
        f"STANCE_ROLL n {len(rows)} t0 {rows[0][0]:.3f} t1 {rows[-1][0]:.3f}"
    )
    print(
        f"STANCE_HIP peak_tau {hip_f[3]:.4f} t {hip_f[0]:.3f} "
        f"q {hip_f[1]:.4f} ctrl {hip_f[2]:.4f} "
        f"max_ctrl_minus_q {hip_err[2] - hip_err[1]:.4f} t_err {hip_err[0]:.3f}"
    )
    print(
        f"STANCE_ANK peak_tau {ank_f[6]:.4f} t {ank_f[0]:.3f} "
        f"q {ank_f[4]:.4f} ctrl {ank_f[5]:.4f} "
        f"max_ctrl_minus_q {ank_err[5] - ank_err[4]:.4f} t_err {ank_err[0]:.3f}"
    )
    print(
        f"STANCE_AT t {at[0]:.3f} hip_q {at[1]:.4f} hip_ctrl {at[2]:.4f} "
        f"hip_tau {at[3]:.4f} ank_q {at[4]:.4f} ank_ctrl {at[5]:.4f} ank_tau {at[6]:.4f}"
    )
    for row in rows:
        print(
            f"STANCE_TICK t {row[0]:.3f} hip_q {row[1]:.4f} hip_ctrl {row[2]:.4f} "
            f"hip_tau {row[3]:.4f} ank_q {row[4]:.4f} ank_ctrl {row[5]:.4f} ank_tau {row[6]:.4f}"
        )


def _print_gap_split(
    sample: tuple[float, float, dict[str, float], dict[str, float]],
) -> None:
    """Split live-minus-stand sole z at the scuff tick. Same leg qdes, two pelvises."""
    t, frac, live, nominal = sample
    gap = float(live["lead"] - nominal["lead"])
    drop = float(live["body_z"] - nominal["body_z"])
    dpitch = float(live["pitch"] - nominal["pitch"])
    live_off = _pelvis_corner_offset(live)
    stand_off = _pelvis_corner_offset(nominal)
    # IMU pitch is negative nose-down, so a negative dpitch times a forward
    # offset lowers the corner. This is the linear term, not the full rotation.
    pitch_x_fwd = dpitch * live_off["fwd_horiz"]
    pitch_x_horiz = dpitch * live_off["horiz"]
    pitch_x_body_x = dpitch * float(live["p_body"][0])
    joints = np.asarray(live["joints"] - nominal["joints"], dtype=np.float64)
    p_delta = np.asarray(live["p_body"] - nominal["p_body"], dtype=np.float64)
    r_live = np.asarray(live["body_r"], dtype=np.float64)
    r_stand = np.asarray(nominal["body_r"], dtype=np.float64)
    p_stand = np.asarray(nominal["p_body"], dtype=np.float64)
    rot_parts = (r_live[2] - r_stand[2]) * p_stand
    rot = float(np.sum(rot_parts))
    pitch_exact = _pitch_about_body_y(nominal, live)
    roll_exact = _roll_about_body_x(nominal, live)
    leftover_fwd = gap - drop - pitch_x_fwd
    leftover_horiz = gap - drop - pitch_x_horiz
    leftover_exact = gap - drop - pitch_exact
    print(
        f"GAP_SPLIT t {t:.3f} frac {frac:.3f} "
        f"live {live['lead']:.6f} stand {nominal['lead']:.6f} gap {gap:.6f}"
    )
    print(
        f"GAP_DROP {drop:.6f} body_z_live {live['body_z']:.6f} "
        f"body_z_stand {nominal['body_z']:.6f}"
    )
    print(
        f"GAP_OFFSET live_fwd_horiz {live_off['fwd_horiz']:.6f} "
        f"live_horiz {live_off['horiz']:.6f} "
        f"live_body_xyz {live['p_body'][0]:.6f} {live['p_body'][1]:.6f} {live['p_body'][2]:.6f}"
    )
    print(
        f"GAP_OFFSET stand_fwd_horiz {stand_off['fwd_horiz']:.6f} "
        f"stand_horiz {stand_off['horiz']:.6f} "
        f"stand_body_xyz {nominal['p_body'][0]:.6f} {nominal['p_body'][1]:.6f} "
        f"{nominal['p_body'][2]:.6f}"
    )
    print(
        f"GAP_PITCH dpitch {dpitch:.6f} "
        f"pitch_x_live_fwd {pitch_x_fwd:.6f} "
        f"pitch_x_live_horiz {pitch_x_horiz:.6f} "
        f"pitch_x_body_x {pitch_x_body_x:.6f} "
        f"pitch_about_y {pitch_exact:.6f} roll_about_x {roll_exact:.6f} "
        f"roll_live {_body_roll(live):.6f} roll_stand {_body_roll(nominal):.6f}"
    )
    print(
        f"GAP_LEFTOVER linear_fwd {leftover_fwd:.6f} "
        f"linear_horiz {leftover_horiz:.6f} "
        f"after_pitch_about_y {leftover_exact:.6f} "
        f"rotation_on_stand_point {rot:.6f} "
        f"rot_xyz {rot_parts[0]:.6f} {rot_parts[1]:.6f} {rot_parts[2]:.6f} "
        f"drop_plus_rotation {drop + rot:.6f}"
    )
    _print_direct_roll(live, nominal, gap, drop, rot_parts)
    print(
        f"GAP_JOINTS max_abs_dq {float(np.max(np.abs(joints))):.3e} "
        f"dq {' '.join(f'{v:.3e}' for v in joints)} "
        f"p_body_delta {' '.join(f'{v:.3e}' for v in p_delta)}"
    )


def _print_direct_roll(
    live: dict[str, float],
    nominal: dict[str, float],
    gap: float,
    drop: float,
    rot_parts: np.ndarray,
) -> None:
    """Roll term from (R_live - R_stand) on the pelvis-to-corner vector.

    The three axis pieces are that one matrix difference. They are not a
    leftover assigned to roll after the fact.
    """
    r_live = np.asarray(live["body_r"], dtype=np.float64)
    r_stand = np.asarray(nominal["body_r"], dtype=np.float64)
    point = np.asarray(nominal["p_body"], dtype=np.float64)
    pitch_term = float(rot_parts[0])
    roll_term = float(rot_parts[1])
    up_term = float(rot_parts[2])
    residual = gap - drop - pitch_term - roll_term
    droll = _body_roll(live) - _body_roll(nominal)
    # Scalar roll on the lateral lever, the -7.4 to -8.0 mm estimate.
    roll_times_y = droll * float(point[1])
    roll_sin_y = math.sin(droll) * float(point[1])
    # Roll-only factor of the relative rotation, then the stand pelvis
    # carries that vector into world z. Yaw and pitch stay at the stand.
    rel = r_stand.T @ r_live
    roll_only = _rx(_body_roll_of(rel) - _body_roll_of(np.eye(3)))
    rolled = r_stand @ roll_only @ point
    stood = r_stand @ point
    isolated = float(rolled[2] - stood[2])
    print(
        f"ROLL_DIRECT pitch_axis {pitch_term:.6f} roll_axis {roll_term:.6f} "
        f"up_axis {up_term:.6f}"
    )
    print(
        f"ROLL_MATH droll {droll:.6f} y {point[1]:.6f} "
        f"droll_times_y {roll_times_y:.6f} sin_droll_times_y {roll_sin_y:.6f} "
        f"isolated_rx {isolated:.6f}"
    )
    print(
        f"ROLL_RESIDUAL after_drop_pitch_roll {residual:.6f} "
        f"equals_up_axis {abs(residual - up_term) < 1e-9} "
        f"sum {drop + pitch_term + roll_term + up_term:.6f} gap {gap:.6f}"
    )


def _rx(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def _body_roll_of(rot: np.ndarray) -> float:
    return math.atan2(float(rot[2, 1]), float(rot[2, 2]))


def _pelvis_corner_offset(row: dict[str, float]) -> dict[str, float]:
    delta = np.asarray(row["corner"] - row["body"], dtype=np.float64)
    fwd = np.asarray(row["body_r"], dtype=np.float64)[:2, 0]
    norm = float(np.linalg.norm(fwd))
    fwd = fwd / norm if norm > 1e-9 else np.array([1.0, 0.0])
    return {
        "horiz": float(math.hypot(delta[0], delta[1])),
        "fwd_horiz": float(np.dot(delta[:2], fwd)),
    }


def _pitch_about_body_y(stand: dict[str, float], live: dict[str, float]) -> float:
    """Sole-z change from rotating the stand pelvis about its +Y by the pitch delta."""
    delta = float(stand["pitch"] - live["pitch"])
    c, s = math.cos(delta), math.sin(delta)
    ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)
    pitched = np.asarray(stand["body_r"], dtype=np.float64) @ ry
    return float((pitched[2] - np.asarray(stand["body_r"])[2]) @ np.asarray(stand["p_body"]))


def _body_roll(row: dict[str, float]) -> float:
    rot = np.asarray(row["body_r"], dtype=np.float64)
    return math.atan2(float(rot[2, 1]), float(rot[2, 2]))


def _roll_about_body_x(stand: dict[str, float], live: dict[str, float]) -> float:
    """Sole-z change from rotating the stand pelvis about its +X by the roll delta."""
    delta = _body_roll(stand) - _body_roll(live)
    c, s = math.cos(delta), math.sin(delta)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)
    rolled = np.asarray(stand["body_r"], dtype=np.float64) @ rx
    return float((rolled[2] - np.asarray(stand["body_r"])[2]) @ np.asarray(stand["p_body"]))


def measure_cmd_toe() -> None:
    """Commanded right-swing toe height at the 3.384 s scuff, from the gait target."""
    session = sw.SteerSession(
        video=False, scene_xml=Path(SCENE), lipm=sw.locked_kit_config(),
    )
    lipm = session.lipm
    assert lipm is not None and lipm.op3 is not None
    driver = sw.ScriptedDriver((
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(3.6, "vel", sw.VX_FWD_CAP, 0.0, "forward"),
    ))
    goal = np.zeros(session.model.nu, dtype=np.float64)
    have = {"ok": False}
    orig = lipm._tick_gait_manager

    def wrapped(walking: bool) -> None:
        orig(walking)
        goal[:] = np.array(session.data.ctrl, dtype=np.float64, copy=True)
        have["ok"] = lipm._gm_swing == "R"

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]
    while float(session.data.time) < 3.6 - 1e-9:
        driver.publish(session.bus, float(session.data.time))
        session.step()
        t = float(session.data.time)
        if abs(t - 3.384) > session.ctrl_dt * 0.51 or not have["ok"]:
            continue
        actual = _leading_toe_z(session, "R")
        cmd_lead, cmd_min = _fk_leading_toe(session, "R", goal)
        applied = np.array(session.data.ctrl, dtype=np.float64, copy=True)
        app_lead, app_min = _fk_leading_toe(session, "R", applied)
        er, el, _, _, _ = lipm.op3.endpoints()
        print(f"CMD_TOE t {t:.3f}")
        print(f"CMD_TOE actual {actual:.6f}")
        print(f"CMD_TOE goal_lead {cmd_lead:.6f} goal_min {cmd_min:.6f}")
        print(f"CMD_TOE applied_lead {app_lead:.6f} applied_min {app_min:.6f}")
        print(f"CMD_TOE ep_r {er[2]:.6f} ep_l {el[2]:.6f} gap {er[2] - el[2]:.6f}")
        print(f"CMD_TOE actual_minus_goal {actual - cmd_lead:.6f}")
        break
    else:
        print("CMD_TOE missed 3.384")
    session.assert_plant_unchanged()


def main() -> None:
    print(f"STEP_TILT {STEP_TILT:.6f} rad {math.degrees(STEP_TILT):.2f} deg")
    measure_scuff()
    runs = [
        run_one("follow", z_m=None, edge=False, mode="follow"),
        run_one("clear", z_m=None, edge=False, mode="clear"),
    ]
    print(f"FOLLOW_TICKS {FOLLOW_TICKS}")
    for row in runs:
        print("---")
        for key, val in row.items():
            print(f"{key} {val}")


if __name__ == "__main__":
    main()
