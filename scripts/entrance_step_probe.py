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

import op3_walk as ow
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
# Track 1 torques are the flat-floor window. The rug-window knee at
# 7.232 s is Track 2 and does not fail this bar.
T_TRACK1 = 7.0
# Inside-edge drop below this is sole roll ≈ 0. 0.5 mm on the 38 mm
# half-width is about 0.013 rad.
SOLE_FLAT_M = 0.0005
# Swing ankle toe-up peak. Sine over the swing, zero at toe-off and touchdown.
# Stays under 0.03 rad.
TOE_UP_PEAK = 0.020
TOE_PITCHOFF_MM = -0.247
TOE_BAND_MM = -0.81
TOE_CLIP20_MM = -1.002
PITCH_GAP_CLIP20 = 0.038044
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


def _xy_in_rug(session: sw.SteerSession, rug_gid: int, point: np.ndarray, margin: float = 0.001) -> bool:
    """True when a world point's xy sits inside the rug box, plus a 1 mm margin."""
    if rug_gid < 0:
        return False
    center = np.array(session.data.geom_xpos[rug_gid], dtype=np.float64)
    rot = np.array(session.data.geom_xmat[rug_gid], dtype=np.float64).reshape(3, 3)
    half = np.array(session.model.geom_size[rug_gid], dtype=np.float64)
    local = rot.T @ (np.array([point[0], point[1], center[2]], dtype=np.float64) - center)
    return abs(float(local[0])) <= float(half[0]) + margin and abs(float(local[1])) <= float(half[1]) + margin


def _foot_surface(session: sw.SteerSession, side: str, rug_gid: int) -> dict[str, float | str | int]:
    """Contact and the surface under the scored leading corner.

    A foot is on the rug when any contact-box corner lies over ``col_mat_rug``
    or the rug normal is above 0.5 N. Clearance uses the rug top only when
    that leading corner itself is over the rug. Otherwise it uses z = 0.
    """
    gid = session.gid_lfoot if side == "L" else session.gid_rfoot
    bid = session.bid_lf if side == "L" else session.bid_rf
    floor_n = _pair_fn(session.model, session.data, gid, session.gid_floor)
    rug_n = _pair_fn(session.model, session.data, gid, rug_gid) if rug_gid >= 0 else 0.0
    names: list[str] = []
    for i in range(session.data.ncon):
        con = session.data.contact[i]
        g1, g2 = int(con.geom1), int(con.geom2)
        if g1 != gid and g2 != gid:
            continue
        other = g2 if g1 == gid else g1
        nm = mj.mj_id2name(session.model, mj.mjtObj.mjOBJ_GEOM, other) or str(other)
        if nm not in names:
            names.append(nm)
    corners = _box_bottom_corners(session.model, session.data, bid, gid)
    fwd = body_forward_xy(session.data, session.bid_body)
    origin = np.asarray(session.data.xpos[session.bid_body, :2], dtype=np.float64)
    lead = max(corners, key=lambda corner: float(np.dot(corner[:2] - origin, fwd)))
    n_in = sum(1 for corner in corners if _xy_in_rug(session, rug_gid, corner))
    lead_in = _xy_in_rug(session, rug_gid, lead)
    on_rug = n_in > 0 or rug_n > 0.5
    if lead_in:
        surface = "rug"
        clear = float(lead[2]) - RUG_TOP_M
    else:
        surface = "floor"
        clear = float(lead[2])
    center_z = float(sum(float(corner[2]) for corner in corners) / len(corners))
    return {
        "floor_n": floor_n,
        "rug_n": rug_n,
        "geoms": ",".join(names) if names else "-",
        "corners_in": n_in,
        "lead_in": int(lead_in),
        "lead_z": float(lead[2]),
        "center_z": center_z,
        "sole_pitch": _sole_pitch(session, side),
        "on_rug": int(on_rug),
        "surface": surface,
        "clear": clear,
    }


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


def _install_swing_z_add(session: sw.SteerSession, extra_m: float) -> None:
    """Add a constant to the swing-foot z command during that foot's single support.

    The 12 mm phase bump stays underneath. Stance z is unchanged. The log
    stores the z component and the add at the walker time the IK used.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        return
    walker = lipm.op3
    orig_r = walker._right_z
    orig_l = walker._left_z
    log: dict[tuple[float, str], tuple[float, float]] = {}
    session._z_cmd_log = log

    def _wrap(orig, side: str):
        def z_fn(t: float) -> float:
            z = float(orig(t))
            start = walker.r_ssp_start if side == "R" else walker.l_ssp_start
            end = walker.r_ssp_end if side == "R" else walker.l_ssp_end
            added = extra_m if start < t <= end else 0.0
            z_out = z + added
            log[(round(float(t), 5), side)] = (z_out, added)
            return z_out
        return z_fn

    walker._right_z = _wrap(orig_r, "R")  # type: ignore[method-assign]
    walker._left_z = _wrap(orig_l, "L")  # type: ignore[method-assign]


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


def _sole_roll(session: sw.SteerSession, side: str) -> tuple[float, float, float, str]:
    """Contact-box roll about its long axis, and the inside-edge drop.

    The box frame is the ankle-roll body (the foot geom quat is identity).
    Local x is the long axis. ``roll`` is atan2(R[2,1], R[2,2]): positive
    lifts the local +y edge. ``inside_drop_m`` is the box-center height
    minus the inside-edge height, so positive means the inside edge is
    low. The lever is the 0.038 m half-width, not ``_sole_tilt``.
    """
    bid = session.bid_lf if side == "L" else session.bid_rf
    gid = session.gid_lfoot if side == "L" else session.gid_rfoot
    rot = np.asarray(session.data.xmat[bid], dtype=np.float64).reshape(3, 3)
    roll = math.atan2(float(rot[2, 1]), float(rot[2, 2]))
    half = np.asarray(session.model.geom_size[gid], dtype=np.float64)
    pos = np.asarray(session.model.geom_pos[gid], dtype=np.float64)
    origin = np.asarray(session.data.xpos[bid], dtype=np.float64)
    body_y = float(session.data.xpos[session.bid_body, 1])

    def _edge(sign: float) -> tuple[float, float]:
        local = np.array(
            [pos[0], pos[1] + sign * float(half[1]), pos[2] - float(half[2])],
            dtype=np.float64,
        )
        world = origin + rot @ local
        return float(world[1]), float(world[2])

    y_pos, z_pos = _edge(1.0)
    y_neg, z_neg = _edge(-1.0)
    if abs(y_pos - body_y) <= abs(y_neg - body_y):
        inside = "+y"
        z_in = z_pos
    else:
        inside = "-y"
        z_in = z_neg
    z_mid = 0.5 * (z_pos + z_neg)
    return roll, z_mid - z_in, float(half[1]), inside


def _decompose_rpy(rot: np.ndarray) -> tuple[float, float, float]:
    """``Rz(yaw) @ Ry(pitch) @ Rx(roll)``. Pitch here is toe-down when applied to the sole."""
    pitch = math.atan2(-float(rot[2, 0]), math.hypot(float(rot[0, 0]), float(rot[1, 0])))
    roll = math.atan2(float(rot[2, 1]), float(rot[2, 2]))
    yaw = math.atan2(float(rot[1, 0]), float(rot[0, 0]))
    return roll, pitch, yaw


def _sole_pitch(session: sw.SteerSession, side: str) -> float:
    """Contact-box pitch about its short axis. Positive is toe-down.

    Local x is forward. ``atan2(-R[2,0], R[2,2])`` is positive when the
    forward axis points down. This is not ``_sole_tilt`` (that one is toe-up).
    """
    bid = session.bid_lf if side == "L" else session.bid_rf
    rot = np.asarray(session.data.xmat[bid], dtype=np.float64).reshape(3, 3)
    return math.atan2(-float(rot[2, 0]), float(rot[2, 2]))


def _print_foot_geoms(session: sw.SteerSession) -> None:
    """Contact box versus the viz sphere. The scorer reads the box."""
    model = session.model
    for gname in ("l_foot_contact", "r_foot_contact", "l_toe_viz", "r_toe_viz"):
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, gname)
        pos = np.asarray(model.geom_pos[gid], dtype=np.float64)
        size = np.asarray(model.geom_size[gid], dtype=np.float64)
        sphere = int(model.geom_type[gid]) == int(mj.mjtGeom.mjGEOM_SPHERE)
        half_down = float(size[0] if sphere else size[2])
        bottom = float(pos[2] - half_down)
        front = float(pos[0] + size[0])
        print(
            f"PRED geom {gname} type {int(model.geom_type[gid])} "
            f"pos {pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} "
            f"size {size[0]:.4f} {size[1]:.4f} {size[2]:.4f} "
            f"bottom {bottom * 1000:.3f} mm front {front * 1000:.1f} mm"
        )
    left = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, session.gid_lfoot)
    right = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, session.gid_rfoot)
    print(
        f"PRED scorer_geom {left} {right} via _leading_toe_z. "
        "Prior toes -0.81 / -1.002 / -0.247 / -2.066 mm used this contact box, "
        "not *_toe_viz. No rescore."
    )


def _box_corner_table(
    session: sw.SteerSession, side: str,
) -> list[tuple[str, float, float]]:
    """Bottom corners of the contact box: label, world z, forward offset.

    Labels are front/heel × inside/outside. Front is local +x. Inside is
    the edge closer to the body in world y.
    """
    bid = session.bid_lf if side == "L" else session.bid_rf
    gid = session.gid_lfoot if side == "L" else session.gid_rfoot
    rot = np.asarray(session.data.xmat[bid], dtype=np.float64).reshape(3, 3)
    origin = np.asarray(session.data.xpos[bid], dtype=np.float64)
    pos = np.asarray(session.model.geom_pos[gid], dtype=np.float64)
    half = np.asarray(session.model.geom_size[gid], dtype=np.float64)
    fwd = body_forward_xy(session.data, session.bid_body)
    body_xy = np.asarray(session.data.xpos[session.bid_body, :2], dtype=np.float64)
    body_y = float(session.data.xpos[session.bid_body, 1])
    y_at: dict[float, float] = {}
    raw: list[tuple[float, float, float, float]] = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            local = np.array(
                [pos[0] + sx * half[0], pos[1] + sy * half[1], pos[2] - half[2]],
                dtype=np.float64,
            )
            world = origin + rot @ local
            y_at[sy] = float(world[1])
            off = float(np.dot(world[:2] - body_xy, fwd))
            raw.append((sx, sy, float(world[2]), off))
    inside_sy = 1.0 if abs(y_at[1.0] - body_y) <= abs(y_at[-1.0] - body_y) else -1.0
    rows: list[tuple[str, float, float]] = []
    for sx, sy, z, off in raw:
        along = "front" if sx > 0.0 else "heel"
        across = "inside" if sy == inside_sy else "outside"
        rows.append((f"{along}-{across}", z, off))
    return rows


def _viz_bottom_z(session: sw.SteerSession, side: str) -> float:
    """Lowest point of the viz sphere. Not the scored toe."""
    gname = "l_toe_viz" if side == "L" else "r_toe_viz"
    gid = mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_GEOM, gname)
    bid = int(session.model.geom_bodyid[gid])
    rot = np.asarray(session.data.xmat[bid], dtype=np.float64).reshape(3, 3)
    origin = np.asarray(session.data.xpos[bid], dtype=np.float64)
    pos = np.asarray(session.model.geom_pos[gid], dtype=np.float64)
    center = origin + rot @ pos
    return float(center[2] - session.model.geom_size[gid, 0])


def _front_mean_z(model: mj.MjModel, data: mj.MjData, session: sw.SteerSession, side: str) -> float:
    bid = session.bid_lf if side == "L" else session.bid_rf
    gid = session.gid_lfoot if side == "L" else session.gid_rfoot
    rot = np.asarray(data.xmat[bid], dtype=np.float64).reshape(3, 3)
    origin = np.asarray(data.xpos[bid], dtype=np.float64)
    pos = np.asarray(model.geom_pos[gid], dtype=np.float64)
    half = np.asarray(model.geom_size[gid], dtype=np.float64)
    zs = []
    for sy in (-1.0, 1.0):
        local = np.array(
            [pos[0] + half[0], pos[1] + sy * half[1], pos[2] - half[2]],
            dtype=np.float64,
        )
        zs.append(float((origin + rot @ local)[2]))
    return 0.5 * (zs[0] + zs[1])


def _ank_pitch_raises_front(session: sw.SteerSession, side: str) -> float:
    """+1 if a positive ankle-pitch joint raises the contact-box front."""
    model = session.model
    data = mj.MjData(model)
    data.qpos[:] = session.data.qpos
    data.qvel[:] = 0.0
    mj.mj_forward(model, data)
    z0 = _front_mean_z(model, data, session, side)
    jn = ("l_" if side == "L" else "r_") + "ank_pitch"
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
    adr = int(model.jnt_qposadr[jid])
    data.qpos[adr] += 0.02
    mj.mj_forward(model, data)
    z1 = _front_mean_z(model, data, session, side)
    return 1.0 if z1 > z0 + 1e-6 else -1.0


def _world_level_ep(ep: np.ndarray, r_body: np.ndarray) -> np.ndarray:
    """Foot RPY in the pelvis frame whose sole is level in the world.

    Heading stays the gait yaw. Roll and pitch cancel the body orientation.
    """
    out = np.array(ep, dtype=np.float64, copy=True)
    heading = r_body @ ow._rot_z(float(ep[5]))
    yaw_w = math.atan2(float(heading[1, 0]), float(heading[0, 0]))
    r_des = ow._rpy(0.0, 0.0, yaw_w)
    r06 = r_body.T @ r_des
    roll, pitch, yaw = _decompose_rpy(r06)
    out[3], out[4], out[5] = roll, pitch, yaw
    return out


def _install_swing_sole_target(session: sw.SteerSession, *, world_level: bool) -> None:
    """Log the swing foot target. Optionally make that sole world-level.

    The OP3 foot roll and pitch are in the pelvis frame. Zeros mean the
    sole is level with the pelvis. ``world_level`` rewrites the swing
    foot so its normal is world up, and cancels the post-IK hip-roll
    add on that swing ankle so the add does not roll the sole again.
    Stance feet stay on the gait target.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        return
    walker = lipm.op3
    orig_ep = walker.endpoints

    def endpoints():
        er, el, pel_r, pel_l, swap_y = orig_ep()
        r_body = np.asarray(session.data.xmat[session.bid_body], dtype=np.float64).reshape(3, 3)
        body_roll, body_pitch, _yaw = _decompose_rpy(r_body)
        t = float(walker.time)
        side = ""
        ep: np.ndarray | None = None
        pel = 0.0
        if walker.l_ssp_start < t <= walker.l_ssp_end:
            side, ep, pel = "L", el, float(pel_l)
        elif walker.r_ssp_start < t <= walker.r_ssp_end:
            side, ep, pel = "R", er, float(pel_r)
        raw_roll = float(ep[3]) if ep is not None else 0.0
        raw_pitch = float(ep[4]) if ep is not None else 0.0
        if world_level and ep is not None:
            leveled = _world_level_ep(np.asarray(ep, dtype=np.float64), r_body)
            if side == "L":
                el = leveled
            else:
                er = leveled
            ep = leveled
        session._sole_cmd = None if ep is None else {
            "side": side,
            "raw_roll": raw_roll,
            "raw_pitch": raw_pitch,
            "roll": float(ep[3]),
            "pitch": float(ep[4]),
            "yaw": float(ep[5]),
            "pel": pel,
            "body_roll": body_roll,
            "body_pitch": body_pitch,
            "world_level": world_level,
            "ank_cancel": 0.0,
        }
        return er, el, pel_r, pel_l, swap_y

    walker.endpoints = endpoints  # type: ignore[method-assign]
    if not world_level:
        return
    orig_step = walker.step

    def step(dt: float):
        joints, info = orig_step(dt)
        cmd = session._sole_cmd
        if (
            joints is not None
            and cmd is not None
            and info.phase in ("L", "R")
            and cmd["side"] == info.phase
        ):
            hip = ("l_" if info.phase == "L" else "r_") + "hip_roll"
            ank = ("l_" if info.phase == "L" else "r_") + "ank_roll"
            delta = float(walker.directions[hip]) * float(cmd["pel"])
            joints[ank] = float(joints[ank]) + delta
            cmd["ank_cancel"] = delta
        return joints, info

    walker.step = step  # type: ignore[method-assign]


def _install_swing_toe_up(session: sw.SteerSession, peak: float) -> None:
    """Add a swing ankle toe-up that is zero at toe-off and at touchdown.

    The peak is under 0.03 rad. Knee and ankle pitch keep the 20 ms
    approach. This does not world-level the sole.
    """
    if not 0.0 < peak < 0.03 - 1e-12:
        raise SystemExit(f"toe-up peak {peak} rad is not under 0.03")
    signs = {side: _ank_pitch_raises_front(session, side) for side in ("L", "R")}
    print(
        f"PRED toe_up_sign L {signs['L']:+.0f} R {signs['R']:+.0f} "
        f"peak {peak:.3f} rad sine"
    )
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        return
    walker = lipm.op3
    orig = lipm._tick_gait_manager

    def wrapped(walking: bool) -> None:
        orig(walking)
        if not walking or session.bus.fault:
            return
        swing = lipm._gm_swing
        if swing not in ("L", "R"):
            return
        frac = _mid_frac(walker, swing, _cmd_time(walker))
        if frac is None or frac <= 0.0 or frac >= 1.0:
            return
        ramp = math.sin(math.pi * float(frac))
        delta = signs[swing] * peak * ramp
        act = ("l_" if swing == "L" else "r_") + "ank_pitch_pos"
        idx = session.act_idx[act]
        session.data.ctrl[idx] = float(session.data.ctrl[idx]) + delta
        cmd = getattr(session, "_sole_cmd", None)
        if cmd is not None and cmd.get("side") == swing:
            cmd["toe_up"] = delta
            cmd["toe_up_frac"] = float(frac)

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]


def _capture_sole_tick(session: sw.SteerSession, t: float, side: str) -> dict[str, object]:
    corners = _box_corner_table(session, side)
    scored = max(corners, key=lambda row: row[2])
    lowest = min(corners, key=lambda row: row[1])
    cmd = getattr(session, "_sole_cmd", None) or {}
    if cmd.get("side") not in (None, side):
        cmd = {}
    roll, drop, _half, inside = _sole_roll(session, side)
    center_z = float(sum(z for _label, z, _off in corners) / len(corners))
    return {
        "t": t,
        "side": side,
        "pitch": _sole_pitch(session, side),
        "roll": roll,
        "inside_drop": drop,
        "inside": inside,
        "corners": corners,
        "center_z": center_z,
        "scored_label": scored[0],
        "scored_z": scored[1],
        "low_label": lowest[0],
        "low_z": lowest[1],
        "viz_z": _viz_bottom_z(session, side),
        "cmd": dict(cmd) if cmd else {},
    }


def _print_sole_diag(
    name: str,
    log: list[dict[str, object]],
    snaps: list[PredSnap],
    t_want: float,
) -> None:
    near = [row for row in log if abs(float(row["t"]) - t_want) <= 0.008 * 0.51]
    if not near:
        print(f"PRED {name} sole_diag missed {t_want:.3f}")
        return
    row = near[0]
    cmd = row["cmd"] if isinstance(row["cmd"], dict) else {}
    print(
        f"PRED {name} sole_diag t {float(row['t']):.3f} side {row['side']} "
        f"sole_pitch_toe_down {float(row['pitch']):.5f} rad"
    )
    for label, z, off in row["corners"]:  # type: ignore[misc]
        mark = ""
        if label == row["scored_label"]:
            mark += " SCORED"
        if label == row["low_label"]:
            mark += " LOWEST"
        print(
            f"PRED {name} corner {label} z {float(z) * 1000:.3f} mm "
            f"fwd {float(off) * 1000:.1f} mm{mark}"
        )
    print(
        f"PRED {name} scored_corner {row['scored_label']} "
        f"z {float(row['scored_z']) * 1000:.3f} mm "
        f"lowest_corner {row['low_label']} z {float(row['low_z']) * 1000:.3f} mm "
        f"box_center {float(row.get('center_z', float('nan'))) * 1000:.3f} mm "
        f"viz_bottom {float(row['viz_z']) * 1000:.3f} mm"
    )
    raw_roll = float(cmd.get("raw_roll", float("nan")))
    raw_pitch = float(cmd.get("raw_pitch", float("nan")))
    frame = (
        "pelvis"
        if abs(raw_roll) < 1e-6 and abs(raw_pitch) < 1e-6
        else "offset"
    )
    print(
        f"PRED {name} ik_foot_frame {frame} "
        f"raw_roll {raw_roll:.5f} raw_pitch {raw_pitch:.5f} "
        f"cmd_roll {float(cmd.get('roll', float('nan'))):.5f} "
        f"cmd_pitch {float(cmd.get('pitch', float('nan'))):.5f} "
        f"body_roll {float(cmd.get('body_roll', float('nan'))):.5f} "
        f"body_pitch {float(cmd.get('body_pitch', float('nan'))):.5f} "
        f"pel {float(cmd.get('pel', float('nan'))):.5f} "
        f"ank_cancel {float(cmd.get('ank_cancel', 0.0)):.5f} "
        f"toe_up {float(cmd.get('toe_up', 0.0)):.5f} "
        f"toe_up_frac {float(cmd.get('toe_up_frac', float('nan'))):.3f}"
    )
    if cmd.get("world_level"):
        print(
            f"PRED {name} ik_sole world-level on the swing foot. "
            "The gait target before that rewrite is pelvis-level. Stance foot unchanged."
        )
    elif frame == "pelvis":
        print(
            f"PRED {name} ik_sole pelvis-level. Foot roll and pitch targets are 0 "
            "in the pelvis frame. The achieved sole is not a copy of body roll."
        )
    ank = [
        s for s in snaps
        if s.joint.endswith("ank_pitch") and abs(s.t - float(row["t"])) <= 0.008 * 0.51
    ]
    if not ank:
        print(f"PRED {name} ank_pitch sample missed {float(row['t']):.3f}")
        return
    for s in ank:
        print(
            f"PRED {name} ank_pitch {s.joint} ik {s.ik:.6f} ctrl {s.ctrl:.6f} "
            f"q {s.q:.6f} ctrl_minus_ik {s.ctrl - s.ik:.6f}"
        )


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


def _swing_cycles(rows: list[Tick], t_cut: float) -> list[list[Tick]]:
    """Contiguous swing ticks with t < t_cut, split when the swing foot changes."""
    cycles: list[list[Tick]] = []
    cur: list[Tick] = []
    for row in rows:
        if row.t >= t_cut - 1e-9:
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
    return cycles


def _mid_swing_step_mins(
    rows: list[Tick], t_cut: float,
) -> list[tuple[int, str, float, float, float]]:
    """Per swing, the min contact-box toe inside the 20–80% window before t_cut.

    The fraction is the cycle index i/(n-1), the same index `_mid_swing` uses.
    Each row is (step, side, t, toe_z, frac).
    """
    out: list[tuple[int, str, float, float, float]] = []
    for step, cyc in enumerate(_swing_cycles(rows, t_cut)):
        n = len(cyc)
        if n < 2:
            continue
        best: tuple[float, float, float, str] | None = None
        for i, row in enumerate(cyc):
            frac = i / (n - 1)
            if row.toe_z is None or row.swing is None:
                continue
            if 0.20 - 1e-12 <= frac <= 0.80 + 1e-12 and (
                best is None or row.toe_z < best[0]
            ):
                best = (row.toe_z, row.t, frac, row.swing)
        if best is None:
            continue
        out.append((step, best[3], best[1], best[0], best[2]))
    return out


def _mid_swing_limited(
    rows: list[Tick], t_limit: float,
) -> tuple[float, float, float, str]:
    """Min contact-box toe in the 20–80% window with t < t_limit.

    Cycles are built through 7.560 s, so the index matches ``_mid_swing``.
    Samples at or after ``t_limit`` are left out of this minimum.
    """
    best: tuple[float, float, float, str] | None = None
    for cyc in _swing_cycles(rows, T_BAR):
        n = len(cyc)
        if n < 2:
            continue
        for i, row in enumerate(cyc):
            if row.toe_z is None or row.swing is None or row.t >= t_limit - 1e-9:
                continue
            frac = i / (n - 1)
            if not (0.20 - 1e-12 <= frac <= 0.80 + 1e-12):
                continue
            if best is None or row.toe_z < best[0]:
                best = (row.toe_z, row.t, frac, row.swing)
    if best is None:
        return (float("nan"), float("nan"), float("nan"), "")
    return best


def _worst_mid_swing(
    rows: list[Tick],
) -> tuple[float, float, float, str, int, int, int]:
    """Worst 20–80% contact-box toe on the whole bout.

    A swing still open at the end keeps the median full-swing length, so
    an early sample is not relabeled into the window.
    """
    cycles = _swing_cycles(rows, T_END + 1.0)
    earlier = [len(cyc) for cyc in cycles[:-1] if len(cyc) >= 2] if len(cycles) >= 2 else []
    if earlier:
        ordered = sorted(earlier)
        med_n = ordered[len(ordered) // 2]
    elif cycles:
        med_n = len(cycles[0])
    else:
        med_n = 0
    last_n = len(cycles[-1]) if cycles else 0
    best: tuple[float, float, float, str] | None = None
    for i, cyc in enumerate(cycles):
        n = len(cyc)
        if n < 2 or med_n < 2:
            continue
        n_full = med_n if i == len(cycles) - 1 and n < med_n - 1 else n
        for j, row in enumerate(cyc):
            if row.toe_z is None or row.swing is None:
                continue
            frac = j / (n_full - 1)
            if not (0.20 - 1e-12 <= frac <= 0.80 + 1e-12):
                continue
            if best is None or row.toe_z < best[0]:
                best = (row.toe_z, row.t, frac, row.swing)
    if best is None:
        return (float("nan"), float("nan"), float("nan"), "", len(cycles), med_n, last_n)
    return (best[0], best[1], best[2], best[3], len(cycles), med_n, last_n)


KNEE_TICK_S = 7.432


def _cycle_index(cycles: list[list[Tick]]) -> tuple[int, int]:
    earlier = [len(cyc) for cyc in cycles[:-1] if len(cyc) >= 2] if len(cycles) >= 2 else []
    if earlier:
        ordered = sorted(earlier)
        med_n = ordered[len(ordered) // 2]
    elif cycles:
        med_n = len(cycles[0])
    else:
        med_n = 0
    return med_n, (len(cycles[-1]) if cycles else 0)


def _surface_summary(
    name: str,
    rows: list[Tick],
    surface_rows: list[dict[str, object]],
    hx_flat: dict[str, tuple[float, float]],
    hx_rug: dict[str, tuple[float, float]],
    ka_hits: list[str] | None = None,
    ka_checked: int = 0,
) -> dict[str, object]:
    """Knee at 7.432 s, and flat-floor mid-swing separate from rug mid-swing."""
    knee = min(surface_rows, key=lambda row: abs(float(row["t"]) - KNEE_TICK_S))
    kt = float(knee["t"])
    lk = float(knee["l_knee"])
    rk = float(knee["r_knee"])
    print(
        f"PRED {name} knee_tick t {kt:.3f} "
        f"l_knee {lk:+.4f} abs {abs(lk):.4f} ge_2.33 {int(abs(lk) >= KNEE_NM)} "
        f"ge_2.45 {int(abs(lk) >= PLANT_NM - 1e-3)} "
        f"r_knee {rk:+.4f} abs {abs(rk):.4f} ge_2.33 {int(abs(rk) >= KNEE_NM)} "
        f"ge_2.45 {int(abs(rk) >= PLANT_NM - 1e-3)}"
    )
    rug_bits: list[str] = []
    for side in ("L", "R"):
        plane = knee[side]
        assert isinstance(plane, dict)
        rug_bits.append(
            f"{side} geoms {plane['geoms']} floor {float(plane['floor_n']):.2f} N "
            f"rug {float(plane['rug_n']):.2f} N corners_in {plane['corners_in']} "
            f"lead_in {plane['lead_in']} lead_z {float(plane['lead_z']) * 1000:.3f} mm "
            f"rug_top {RUG_TOP_M * 1000:.1f} mm on_rug {plane['on_rug']} "
            f"surface {plane['surface']}"
        )
    knee_rug = " ".join(rug_bits)
    knee_on = int(any(
        int(knee[side]["on_rug"]) for side in ("L", "R")  # type: ignore[index]
    ))
    print(f"PRED {name} rug_at {kt:.3f} either_on_rug {knee_on} {knee_rug}")
    by_t = {round(float(row["t"]), 5): row for row in surface_rows}
    cycles = _swing_cycles(rows, T_END + 1.0)
    med_n, _last_n = _cycle_index(cycles)
    flat_best: tuple[float, float, float, str, float, float, float, float] | None = None
    rug_best: tuple[float, float, float, str, float] | None = None
    mid_samples: list[tuple[float, str, float, float, float, float, float, str]] = []
    n_flat = 0
    n_rug = 0
    for step, cyc in enumerate(cycles):
        n = len(cyc)
        if n < 2 or med_n < 2:
            continue
        n_full = med_n if step == len(cycles) - 1 and n < med_n - 1 else n
        mids: list[tuple[float, Tick, dict[str, object]]] = []
        for j, row in enumerate(cyc):
            if row.toe_z is None or row.swing is None:
                continue
            frac = j / (n_full - 1)
            if not (0.20 - 1e-12 <= frac <= 0.80 + 1e-12):
                continue
            surf = by_t.get(round(row.t, 5))
            if surf is None:
                continue
            plane = surf[row.swing]
            if not isinstance(plane, dict):
                continue
            mids.append((frac, row, plane))
        if not mids:
            continue
        on_rug = any(int(plane["on_rug"]) for _frac, _row, plane in mids)
        tag = "rug" if on_rug else "flat"
        if tag == "rug":
            n_rug += 1
        else:
            n_flat += 1
        worst = min(mids, key=lambda item: float(item[1].toe_z or 0.0))
        for _frac, row_i, plane_i in mids:
            surf_i = by_t.get(round(row_i.t, 5), {})
            body_i = (
                float(surf_i["body_z"])
                if isinstance(surf_i, dict) and "body_z" in surf_i else float("nan")
            )
            zadd_i = (
                float(surf_i["z_add"])
                if isinstance(surf_i, dict) and "z_add" in surf_i else float("nan")
            )
            mid_samples.append((
                row_i.t, row_i.swing or "", float(row_i.toe_z or 0.0) * 1000.0,
                float(plane_i["center_z"]) * 1000.0, body_i * 1000.0,
                zadd_i * 1000.0, float(plane_i["sole_pitch"]), tag,
            ))
        frac, row, plane = worst
        toe = float(row.toe_z or 0.0)
        clear = float(plane["clear"])
        surf = by_t.get(round(row.t, 5), {})
        body_z = float(surf["body_z"]) if isinstance(surf, dict) and "body_z" in surf else float("nan")
        z_add = float(surf["z_add"]) if isinstance(surf, dict) and "z_add" in surf else float("nan")
        center_z = float(plane["center_z"])
        sole_pitch = float(plane["sole_pitch"])
        print(
            f"PRED {name} swing {step} side {row.swing} t {row.t:.3f} "
            f"frac {frac:.3f} tag {tag} toe {toe * 1000:.3f} mm "
            f"clear {clear * 1000:.3f} mm surface {plane['surface']} "
            f"corners_in {plane['corners_in']} "
            f"floor {float(plane['floor_n']):.1f} rug {float(plane['rug_n']):.1f}"
        )
        if tag == "flat":
            if flat_best is None or toe < flat_best[0]:
                flat_best = (
                    toe, row.t, frac, row.swing or "",
                    center_z, sole_pitch, body_z, z_add,
                )
        elif rug_best is None or clear < rug_best[0]:
            rug_best = (clear, row.t, frac, row.swing or "", toe)
    flat_toe = flat_best[0] * 1000.0 if flat_best else float("nan")
    flat_t = flat_best[1] if flat_best else float("nan")
    flat_side = flat_best[3] if flat_best else ""
    rug_clear = rug_best[0] * 1000.0 if rug_best else float("nan")
    rug_t = rug_best[1] if rug_best else float("nan")
    rug_side = rug_best[3] if rug_best else ""
    rug_toe = rug_best[4] * 1000.0 if rug_best else float("nan")
    flat_center = flat_best[4] * 1000.0 if flat_best else float("nan")
    flat_pitch = flat_best[5] if flat_best else float("nan")
    flat_body = flat_best[6] * 1000.0 if flat_best else float("nan")
    flat_z_add = flat_best[7] * 1000.0 if flat_best else float("nan")
    print(
        f"PRED {name} flat_swings {n_flat} rug_swings {n_rug} "
        f"flat_worst {flat_toe:.3f} mm t {flat_t:.3f} side {flat_side} "
        f"box_centre {flat_center:.3f} mm sole_pitch {flat_pitch:.5f} rad "
        f"body_z {flat_body:.3f} mm z_add {flat_z_add:.3f} mm "
        f"rug_worst_clear {rug_clear:.3f} mm t {rug_t:.3f} side {rug_side} "
        f"rug_toe {rug_toe:.3f} mm. Track 1 uses the flat swings only."
    )
    flat_hits: list[str] = []
    for act, (tau, when) in hx_flat.items():
        over = abs(tau) >= KNEE_NM
        if over:
            flat_hits.append(f"{act} {tau:+.4f} t {when:.3f}")
        print(
            f"PRED {name} hx_flat {act} {tau:+.4f} t {when:.3f} "
            f"abs {abs(tau):.4f} ge_2.33 {int(over)} "
            f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
        )
    for act, (tau, when) in hx_rug.items():
        print(
            f"PRED {name} hx_rug {act} {tau:+.4f} t {when:.3f} "
            f"abs {abs(tau):.4f} ge_2.33 {int(abs(tau) >= KNEE_NM)} "
            f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
        )
    if ka_hits is not None:
        print(f"PRED {name} ka_checked {ka_checked} ka_over_2.33 {len(ka_hits)}")
        for line in ka_hits:
            print(f"PRED {name} ka_tick {line}")
    return {
        "flat_toe_mm": flat_toe,
        "flat_toe_t": flat_t,
        "flat_toe_side": flat_side,
        "flat_center_mm": flat_center,
        "flat_sole_pitch": flat_pitch,
        "flat_body_mm": flat_body,
        "flat_z_add_mm": flat_z_add,
        "rug_clear_mm": rug_clear,
        "rug_clear_t": rug_t,
        "rug_clear_side": rug_side,
        "knee_l": lk,
        "knee_r": rk,
        "knee_rug": knee_rug,
        "knee_on_rug": knee_on,
        "flat_hx_over": ", ".join(flat_hits),
        "ka_over_n": 0 if ka_hits is None else len(ka_hits),
        "ka_checked": ka_checked,
        "mid_samples": tuple(mid_samples),
    }


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
LEG_ALL = LEG8 + (
    "l_hip_yaw_pos", "r_hip_yaw_pos",
    "l_ank_roll_pos", "r_ank_roll_pos",
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


# Compiled kv from dampratio 1 on this plant. The clip reads the model.
# These are the check, not a second source written into the actuator.
_COMPILED_KV: dict[str, float] = {
    "hip_roll": 1.7027,
    "hip_pitch": 1.8102,
}


def _kv_kind(jn: str) -> str:
    if jn.endswith("hip_roll"):
        return "hip_roll"
    if jn.endswith("hip_pitch"):
        return "hip_pitch"
    raise ValueError(f"swing-hip clip is only hip_roll and hip_pitch, got {jn}")


def _joint_q_qd(session: sw.SteerSession, jn: str) -> tuple[float, float, int]:
    model = session.model
    data = session.data
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
    q = float(data.qpos[int(model.jnt_qposadr[jid])])
    qd = float(data.qvel[int(model.jnt_dofadr[jid])])
    return q, qd, session.act_idx[jn + "_pos"]


def _clamp_ctrl_predicted(session: sw.SteerSession, jn: str, limit_nm: float) -> float:
    """Pull ``data.ctrl`` in so kp·(ctrl−q) − kv·q̇ stays inside ±limit.

    q̇ is the live joint velocity. The desired command is whatever is
    already in ctrl, so this only tightens. kp and kv come from the
    compiled actuator. The plant file is not edited.
    """
    q, qd, idx = _joint_q_qd(session, jn)
    model = session.model
    kp = float(model.actuator_gainprm[idx, 0])
    kv = -float(model.actuator_biasprm[idx, 2])
    ctrl = float(session.data.ctrl[idx])
    if kp < 1e-6:
        return 0.0
    e_lo = (-limit_nm + kv * qd) / kp
    e_hi = (limit_nm + kv * qd) / kp
    if e_lo > e_hi:
        e_lo, e_hi = e_hi, e_lo
    cmd = q + min(e_hi, max(e_lo, ctrl - q))
    lo = float(model.actuator_ctrlrange[idx, 0])
    hi = float(model.actuator_ctrlrange[idx, 1])
    cmd = min(hi, max(lo, cmd))
    session.data.ctrl[idx] = cmd
    return kp * (cmd - q) - kv * qd


def _install_swing_pred_clip(session: sw.SteerSession, *, ankle_roll: bool = False) -> None:
    """Sim-only predicted-force clip on the swing hips.

    Replaces the kp·(ctrl−q) band for swing hip roll and hip pitch.
    ``ankle_roll`` adds the same law on swing ankle roll only. Stance
    joints and ankle pitch keep the existing write. The kit position
    servo does not have this limiter. kp, kv, and dampratio stay.
    """
    lipm = session.lipm
    if lipm is None:
        return
    orig_write = lipm.write_clipped
    suffixes = ("hip_roll", "hip_pitch", "ank_roll") if ankle_roll else ("hip_roll", "hip_pitch")

    def write(jn: str, q_des: float) -> None:
        swing = lipm._gm_swing
        side = "L" if jn.startswith("l_") else "R" if jn.startswith("r_") else ""
        if swing in ("L", "R") and side == swing and jn.endswith(suffixes):
            lipm.write_force_limited(jn, float(q_des), KNEE_NM)
            return
        orig_write(jn, q_des)

    lipm.write_clipped = write  # type: ignore[method-assign]


def _install_pitch_move_off(session: sw.SteerSession) -> None:
    """Take hip pitch out of the 20 ms approach.

    Knee and ankle pitch stay on ``gm_move_s``. ``gm_move_s`` itself stays
    0.020, and the period stays 0.500. Hip pitch then finishes the tick
    the way hip roll already does, with no 20 ms fraction and no HX cap.
    """
    session._move_ctrl_idx = [
        idx
        for name, idx in session.act_idx.items()
        if name.endswith(("knee_pos", "ank_pitch_pos"))
    ]


def _joints_at(walker: object, t: float):
    """IK at a clock time. Restores the walker clock."""
    saved = float(walker.time)
    walker.time = float(t)
    try:
        return walker.joints_now()
    finally:
        walker.time = saved


def _wrap_period(t: float, period: float) -> float:
    if period <= 1e-9:
        return t
    while t >= period - 1e-12:
        t -= period
    while t < -1e-12:
        t += period
    return t


def _install_swing_hip_lead(
    session: sw.SteerSession,
    *,
    roll_scale: float = 1.0,
    pitch_scale: float = 1.0,
) -> tuple[float, float]:
    """Advance swing-hip ctrl by a scale of the compiled kv/kp lag.

    Scale 0 leaves that joint on the baseline write. Foot z is not
    shifted. Ankle roll is not written here. The predicted-force clip
    still limits any joint this function does write. Stance hips stay
    on the current IK.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        raise RuntimeError("gait manager walker missing")
    walker = lipm.op3
    scales = {"hip_roll": float(roll_scale), "hip_pitch": float(pitch_scale)}
    compiled: dict[str, float] = {}
    applied: dict[str, float] = {}
    for kind, scale in scales.items():
        _q, _qd, idx = _joint_q_qd(session, "r_" + kind)
        kv = -float(session.model.actuator_biasprm[idx, 2])
        kp = float(session.model.actuator_gainprm[idx, 0])
        if kp < 1e-6:
            raise SystemExit(f"compiled kp for {kind} is {kp}")
        compiled[kind] = kv / kp
        applied[kind] = compiled[kind] * scale
    roll_note = (
        "Hip roll ctrl is not rewritten."
        if applied["hip_roll"] <= 0.0
        else "Hip roll ctrl is advanced."
    )
    print(
        f"PRED hip_lead roll applied {applied['hip_roll'] * 1000:.2f} ms "
        f"compiled {compiled['hip_roll'] * 1000:.2f} ms "
        f"pitch applied {applied['hip_pitch'] * 1000:.2f} ms "
        f"compiled {compiled['hip_pitch'] * 1000:.2f} ms. "
        f"{roll_note} Ankle roll is not led. Foot z is not advanced."
    )
    session._hip_lead_log = []
    orig = lipm._tick_gait_manager

    def wrapped(walking: bool) -> None:
        orig(walking)
        if not walking or session.bus.fault:
            return
        swing = lipm._gm_swing
        if swing not in ("L", "R"):
            return
        saved_cmd = getattr(session, "_sole_cmd", None)
        t_cmd = _cmd_time(walker)
        period = float(walker.period)
        now_joints, _now_info = _joints_at(walker, t_cmd)
        if now_joints is None:
            session._sole_cmd = saved_cmd
            return
        t_mark = float(session.data.time) + float(session.ctrl_dt)
        pref = "l_" if swing == "L" else "r_"
        for kind in ("hip_roll", "hip_pitch"):
            if applied[kind] <= 0.0:
                continue
            jn = pref + kind
            fut_joints, _fut_info = _joints_at(
                walker, _wrap_period(t_cmd + applied[kind], period),
            )
            if fut_joints is None or jn not in fut_joints:
                continue
            lipm.write_force_limited(jn, float(fut_joints[jn]), KNEE_NM)
            idx = session.act_idx[jn + "_pos"]
            session._hip_lead_log.append({
                "t": t_mark,
                "joint": jn,
                "ik": float(now_joints[jn]),
                "future": float(fut_joints[jn]),
                "ctrl": float(session.data.ctrl[idx]),
                "lead_s": applied[kind],
            })
        session._sole_cmd = saved_cmd

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]
    return applied["hip_roll"], applied["hip_pitch"]


@dataclass
class PredSnap:
    t: float
    joint: str
    ik: float
    ctrl: float
    q: float
    qd: float
    predicted: float
    force: float


@dataclass
class PredScore:
    name: str
    clip: bool
    move_s: float
    toe_mm: float
    toe_t: float
    toe_frac: float
    toe_side: str
    x_m: float
    fault: str
    legs_under: bool
    pred_under: bool
    toe_clear: bool
    holds_bar: bool
    snaps: list[PredSnap]
    trace: list[PredSnap]
    sole_roll_rad: float = 0.0
    sole_inside_drop_mm: float = 0.0
    sole_inside: str = ""
    sole_half_m: float = 0.038
    sole_flat: bool = True
    pitch_gap_rad: float = 0.0
    pitch_on_ik: bool = False
    track2_over: str = ""
    flat_toe_mm: float = float("nan")
    flat_toe_t: float = float("nan")
    flat_toe_side: str = ""
    rug_clear_mm: float = float("nan")
    rug_clear_t: float = float("nan")
    rug_clear_side: str = ""
    knee_l: float = float("nan")
    knee_r: float = float("nan")
    knee_rug: str = ""
    knee_on_rug: int = 0
    flat_hx_over: str = ""
    flat_center_mm: float = float("nan")
    flat_sole_pitch: float = float("nan")
    flat_body_mm: float = float("nan")
    flat_z_add_mm: float = float("nan")
    ka_over_n: int = 0
    ka_checked: int = 0
    mid_samples: tuple = ()


def _check_compiled_kv(session: sw.SteerSession) -> None:
    for jn, kind in (
        ("r_hip_roll", "hip_roll"),
        ("l_hip_roll", "hip_roll"),
        ("r_hip_pitch", "hip_pitch"),
        ("l_hip_pitch", "hip_pitch"),
    ):
        _q, _qd, idx = _joint_q_qd(session, jn)
        kv = -float(session.model.actuator_biasprm[idx, 2])
        kp = float(session.model.actuator_gainprm[idx, 0])
        want = _COMPILED_KV[kind]
        if abs(kv - want) > 1e-3:
            raise SystemExit(f"compiled kv for {jn} is {kv:.4f}, expected {want:.4f}")
        want_kp = 40.0 if kind == "hip_roll" else 45.0
        if abs(kp - want_kp) > 1e-3:
            raise SystemExit(f"compiled kp for {jn} is {kp:.4f}, expected {want_kp:.1f}")


def _check_ank_roll_kv(session: sw.SteerSession) -> float:
    """Compiled ankle-roll kv. The clip reads the actuator. This only checks."""
    seen: list[float] = []
    for jn in ("r_ank_roll", "l_ank_roll"):
        _q, _qd, idx = _joint_q_qd(session, jn)
        kv = -float(session.model.actuator_biasprm[idx, 2])
        kp = float(session.model.actuator_gainprm[idx, 0])
        if abs(kp - 35.0) > 1e-3:
            raise SystemExit(f"compiled kp for {jn} is {kp:.4f}, expected 35")
        if kv <= 0.0:
            raise SystemExit(f"compiled kv for {jn} is {kv:.4f}")
        seen.append(kv)
        print(f"PRED ank_roll {jn} kp {kp:.4f} kv {kv:.4f}")
    if abs(seen[0] - seen[1]) > 1e-3:
        raise SystemExit(f"ankle-roll kv differs L {seen[1]:.4f} R {seen[0]:.4f}")
    return seen[0]


def measure_pred_clip(
    *,
    clip: bool,
    move_s: float | None,
    pitch_move_off: bool = False,
    ankle_roll: bool = False,
    period_s: float | None = None,
    bar_before_s: float | None = None,
    world_level: bool = False,
    toe_up_rad: float = 0.0,
    sole_report: bool = False,
    geometry_diag: bool = False,
    hip_lead: bool = False,
    lag_report: bool = False,
    lead_roll_scale: float = 1.0,
    lead_pitch_scale: float = 1.0,
    whole_toe: bool = False,
    surface_tag: bool = False,
    z_extra_m: float = 0.0,
    ka_log: bool = False,
) -> PredScore:
    """12 mm @ 20% with an optional swing-hip predicted-force clip.

    ``move_s`` None keeps the locked 20 ms approach. ``pitch_move_off``
    drops hip pitch from that approach and leaves knee and ankle pitch
    on it.     ``period_s`` None keeps 0.500 s. ``bar_before_s`` scores leg
    torques only before that time. ``world_level`` and ``toe_up_rad``
    are separate swing-sole copies. The 0.98 spring band is not widened.
    """
    if world_level and toe_up_rad > 0.0:
        raise SystemExit("world-level and toe-up are separate copies")
    cfg = sw.locked_kit_config()
    if move_s is not None:
        cfg = replace(cfg, gm_move_s=float(move_s))
    if period_s is not None:
        cfg = replace(cfg, gm_period_s=float(period_s))
    session = sw.SteerSession(video=False, scene_xml=Path(SCENE), lipm=cfg)
    _check_compiled_kv(session)
    ank_kv = _check_ank_roll_kv(session) if ankle_roll else 0.0
    _install_phase_lift(session, 0.012, 0.20, 0.0)
    if ka_log or abs(z_extra_m) > 0.0:
        _install_swing_z_add(session, z_extra_m)
    if pitch_move_off:
        _install_pitch_move_off(session)
    if clip:
        _install_swing_pred_clip(session, ankle_roll=ankle_roll)
    if sole_report or world_level:
        _print_foot_geoms(session)
        _install_swing_sole_target(session, world_level=world_level)
    if toe_up_rad > 0.0:
        _install_swing_toe_up(session, toe_up_rad)
    if hip_lead:
        _install_swing_hip_lead(
            session,
            roll_scale=lead_roll_scale,
            pitch_scale=lead_pitch_scale,
        )
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        raise RuntimeError("gait manager walker missing")
    walker = lipm.op3
    driver = sw.ScriptedDriver((
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(T_END, "vel", sw.VX_FWD_CAP, 0.0, "forward"),
    ))
    rows: list[Tick] = []
    surface_rows: list[dict[str, object]] = []
    hx_chain = (
        "l_hip_pitch_pos", "r_hip_pitch_pos",
        "l_knee_pos", "r_knee_pos",
        "l_ank_pitch_pos", "r_ank_pitch_pos",
    )
    hx_flat = {act: (0.0, 0.0) for act in hx_chain}
    hx_rug = {act: (0.0, 0.0) for act in hx_chain}
    ka_names = (
        "l_knee_pos", "r_knee_pos",
        "l_ank_pitch_pos", "r_ank_pitch_pos",
    )
    ka_hits: list[str] = []
    ka_checked = 0
    suffixes = ("hip_roll", "hip_pitch", "ank_roll") if ankle_roll else ("hip_roll", "hip_pitch")
    record_suffixes = suffixes + (("ank_pitch",) if sole_report or toe_up_rad > 0.0 else ())
    sole_log: list[dict[str, object]] = []
    leg8 = {name: (0.0, 0.0) for name in LEG8}
    leg_bout = {name: (0.0, 0.0) for name in LEG_ALL}
    leg_pre = {name: (0.0, 0.0) for name in LEG_ALL}
    leg_post = {name: (0.0, 0.0) for name in LEG_ALL}
    swing_names = tuple(pref + suffix for pref in ("l_", "r_") for suffix in suffixes)
    swing_force: dict[str, tuple[float, float]] = {name: (0.0, 0.0) for name in swing_names}
    swing_pred: dict[str, tuple[float, float]] = {name: (0.0, 0.0) for name in swing_names}
    swing_pred_pre: dict[str, tuple[float, float]] = {name: (0.0, 0.0) for name in swing_names}
    last: dict[str, tuple[float, float, float, float]] = {}
    sole_rows: list[tuple[float, str, float, float, float, str]] = []
    clip_n = 0
    real_step = mj.mj_step
    bar = float(bar_before_s) if bar_before_s is not None else None

    def _hook(model: mj.MjModel, data: mj.MjData) -> None:
        nonlocal clip_n
        swing = lipm._gm_swing
        if clip and swing in ("L", "R"):
            pref = "l_" if swing == "L" else "r_"
            for suffix in suffixes:
                jn = pref + suffix
                idx = session.act_idx[jn + "_pos"]
                before = float(data.ctrl[idx])
                _clamp_ctrl_predicted(session, jn, KNEE_NM)
                if abs(float(data.ctrl[idx]) - before) > 1e-5:
                    clip_n += 1
        pre_t = float(data.time)
        pre: dict[str, tuple[float, float, float, float]] = {}
        if swing in ("L", "R"):
            pref = "l_" if swing == "L" else "r_"
            for suffix in record_suffixes:
                jn = pref + suffix
                q, qd, idx = _joint_q_qd(session, jn)
                kp = float(model.actuator_gainprm[idx, 0])
                kv = -float(model.actuator_biasprm[idx, 2])
                ctrl = float(data.ctrl[idx])
                pre[jn] = (q, qd, ctrl, kp * (ctrl - q) - kv * qd)
        real_step(model, data)
        if swing in ("L", "R"):
            for jn, (q, qd, ctrl, pred) in pre.items():
                idx = session.act_idx[jn + "_pos"]
                force = float(data.actuator_force[idx])
                last[jn] = (q, qd, ctrl, force)
                if jn not in swing_force:
                    continue
                when = float(data.time)
                prev_f, _when = swing_force[jn]
                if abs(force) > abs(prev_f):
                    swing_force[jn] = (force, when)
                prev_p, _tp = swing_pred[jn]
                if abs(pred) > abs(prev_p):
                    swing_pred[jn] = (pred, pre_t)
                if bar is not None and pre_t < bar - 1e-9:
                    prev_pp, _tpp = swing_pred_pre[jn]
                    if abs(pred) > abs(prev_pp):
                        swing_pred_pre[jn] = (pred, pre_t)

    mj.mj_step = _hook
    snaps: list[PredSnap] = []
    trace: list[PredSnap] = []
    first_fault = ""
    fault_t = float("nan")
    x_at: dict[str, float] = {}
    try:
        while float(session.data.time) < T_END - 1e-9:
            driver.publish(session.bus, float(session.data.time))
            session.step()
            t = float(session.data.time)
            swing = lipm._gm_swing if lipm._gm_swing in ("L", "R") else None
            toe_z = _leading_toe_z(session, swing) if swing in ("L", "R") else None
            if swing in ("L", "R"):
                roll, drop, half_y, inside = _sole_roll(session, swing)
                sole_rows.append((t, swing, roll, drop, half_y, inside))
                if sole_report:
                    sole_log.append(_capture_sole_tick(session, t, swing))
            if not session.bus.fault:
                for name in LEG_ALL:
                    idx = session.act_idx[name]
                    tau = float(session.data.actuator_force[idx])
                    if name in leg8:
                        prev, _when = leg8[name]
                        if abs(tau) > abs(prev):
                            leg8[name] = (tau, t)
                    prev_w, _ww = leg_bout[name]
                    if abs(tau) > abs(prev_w):
                        leg_bout[name] = (tau, t)
                    if bar is not None and t < bar - 1e-9:
                        prev_b, _wb = leg_pre[name]
                        if abs(tau) > abs(prev_b):
                            leg_pre[name] = (tau, t)
                    elif bar is not None:
                        prev_a, _wa = leg_post[name]
                        if abs(tau) > abs(prev_a):
                            leg_post[name] = (tau, t)
            if session.bus.fault and not first_fault:
                first_fault = session.bus.fault_reason or ""
                fault_t = t
            x_now = float(session.data.qpos[0])
            for mark in (2.872, 7.0):
                key = f"{mark:.3f}"
                if key not in x_at and t + 1e-9 >= mark:
                    x_at[key] = x_now
            if surface_tag:
                rug_gid = int(session.gid_rug)
                planes = {
                    side: _foot_surface(session, side, rug_gid) for side in ("L", "R")
                }
                z_comp = float("nan")
                z_add = 0.0
                if swing in ("L", "R"):
                    zlog = getattr(session, "_z_cmd_log", {})
                    rec = None
                    for back in (0.0, float(session.ctrl_dt), 2.0 * float(session.ctrl_dt)):
                        rec = zlog.get((round(t - back, 5), swing))
                        if rec is not None:
                            break
                    if rec is not None:
                        z_comp, z_add = rec
                surface_rows.append({
                    "t": t,
                    "swing": swing if swing in ("L", "R") else None,
                    "toe_z": toe_z,
                    "body_z": float(session.data.xpos[session.bid_body][2]),
                    "z_comp": z_comp,
                    "z_add": z_add,
                    "L": planes["L"],
                    "R": planes["R"],
                    "l_knee": float(session.data.actuator_force[session.act_idx["l_knee_pos"]]),
                    "r_knee": float(session.data.actuator_force[session.act_idx["r_knee_pos"]]),
                })
                if ka_log:
                    ka_checked += 1
                    for act in ka_names:
                        tau = float(session.data.actuator_force[session.act_idx[act]])
                        foot = planes["L" if act.startswith("l_") else "R"]
                        if abs(tau) >= KNEE_NM:
                            tag = "rug" if int(foot["on_rug"]) else "flat"
                            ka_hits.append(
                                f"{act} {tau:+.4f} t {t:.3f} {tag} "
                                f"ge_2.33 1 ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
                            )
                for act in hx_chain:
                    tau = float(session.data.actuator_force[session.act_idx[act]])
                    foot = planes["L" if act.startswith("l_") else "R"]
                    bucket = hx_rug if int(foot["on_rug"]) else hx_flat
                    prev, _when = bucket[act]
                    if abs(tau) > abs(prev):
                        bucket[act] = (tau, t)
            rows.append(Tick(
                t, swing if swing in ("L", "R") else None, toe_z, 0.0, first_fault,
            ))
            if swing in ("L", "R") and last:
                saved_t = float(walker.time)
                walker.time = _cmd_time(walker)
                joints, _info = walker.joints_now()
                walker.time = saved_t
                if joints is not None:
                    pref = "l_" if swing == "L" else "r_"
                    for suffix in record_suffixes:
                        jn = pref + suffix
                        if jn not in last:
                            continue
                        q, qd, ctrl, force = last[jn]
                        ik = float(joints[jn])
                        _qq, _qd, idx = _joint_q_qd(session, jn)
                        kp = float(session.model.actuator_gainprm[idx, 0])
                        kv = -float(session.model.actuator_biasprm[idx, 2])
                        pred = kp * (ctrl - q) - kv * qd
                        snap = PredSnap(t, jn, ik, ctrl, q, qd, pred, force)
                        snaps.append(snap)
                        trace.append(snap)
            last.clear()
    finally:
        mj.mj_step = real_step
    toe_min, toe_t, _p90, _n_mid, _n_cyc, toe_frac, toe_side = _mid_swing(rows)
    if whole_toe:
        bout_z, bout_t, bout_frac, bout_side, bout_n, med_n, last_n = _worst_mid_swing(rows)
        if bout_t == bout_t and (
            toe_t != toe_t or abs(bout_t - toe_t) > 1e-6 or abs(bout_z - toe_min) > 1e-9
        ):
            print(
                f"PRED toe_window_7.560 {toe_min * 1000:.3f} mm t {toe_t:.3f} "
                f"side {toe_side} is not the whole-walk toe."
            )
        toe_min, toe_t, toe_frac, toe_side = bout_z, bout_t, bout_frac, bout_side
        print(
            f"PRED whole_walk cycles {bout_n} median_n {med_n} last_n {last_n} "
            f"toe {toe_min * 1000:.3f} mm t {toe_t:.3f} frac {toe_frac:.3f} side {bout_side}"
        )
    if bar is not None and not whole_toe:
        lim_z, lim_t, lim_frac, lim_side = _mid_swing_limited(rows, bar)
        if lim_t == lim_t:
            if abs(lim_t - toe_t) > 1e-6:
                print(
                    f"PRED toe_after_{bar:.1f} {toe_min * 1000:.3f} mm "
                    f"t {toe_t:.3f} side {toe_side} is not the Track 1 toe."
                )
            toe_min, toe_t, toe_frac, toe_side = lim_z, lim_t, lim_frac, lim_side
    whole_over = [name for name, (tau, _when) in leg8.items() if abs(tau) >= KNEE_NM]
    whole_pred = [name for name, (tau, _when) in swing_pred.items() if abs(tau) > KNEE_NM + 1e-3]
    if bar is None:
        over = whole_over
        pred_over = whole_pred
        track2 = ""
    else:
        over = [name for name, (tau, _when) in leg_pre.items() if abs(tau) >= KNEE_NM]
        pred_over = [
            name for name, (tau, _when) in swing_pred_pre.items() if abs(tau) > KNEE_NM + 1e-3
        ]
        track2_hits = [
            f"{name} {tau:+.4f} t {when:.3f}"
            for name, (tau, when) in leg_post.items()
            if abs(tau) >= KNEE_NM
        ]
        track2 = ", ".join(track2_hits)
    if whole_toe:
        over = [name for name, (tau, _when) in leg_bout.items() if abs(tau) >= KNEE_NM]
    legs_under = not over
    pred_under = not pred_over
    toe_clear = bool(toe_min > TOE_BAR_M)
    used_move = float(cfg.gm_move_s)
    if (
        pitch_move_off or ankle_roll or period_s is not None
        or world_level or toe_up_rad > 0.0 or hip_lead
    ):
        parts = ["clip" if clip else "base"]
        parts.append("pitchoff" if pitch_move_off else f"move{used_move * 1000:.0f}")
        if ankle_roll:
            parts.append("ankroll")
        if world_level:
            parts.append("worldlevel")
        if toe_up_rad > 0.0:
            parts.append(f"toeup{toe_up_rad * 1000:.0f}mrad")
        if hip_lead:
            if abs(lead_roll_scale) <= 1e-12 and lead_pitch_scale > 0.0:
                pitch_ms = lead_pitch_scale * (_COMPILED_KV["hip_pitch"] / 45.0) * 1000.0
                parts.append("pitchlead" if abs(pitch_ms - 40.23) < 0.05 else f"pitch{pitch_ms:.0f}ms")
            elif abs(lead_roll_scale - 1.0) <= 1e-12 and abs(lead_pitch_scale - 1.0) <= 1e-12:
                parts.append("hiplead")
            else:
                parts.append(f"r{lead_roll_scale:.2f}p{lead_pitch_scale:.2f}")
        if period_s is not None:
            parts.append(f"T{period_s:.2f}")
        if abs(z_extra_m) > 0.0:
            parts.append(f"z{z_extra_m * 1000.0:.3f}mm")
        name = "_".join(parts)
    else:
        name = f"{'clip' if clip else 'base'}_move{used_move * 1000:.0f}"
    pitch_slew = any(
        name.endswith("hip_pitch_pos") and idx in session._move_ctrl_idx
        for name, idx in session.act_idx.items()
    )
    knee_slew = any(
        name.endswith("knee_pos") and idx in session._move_ctrl_idx
        for name, idx in session.act_idx.items()
    )
    sole_roll = 0.0
    sole_drop = 0.0
    sole_inside = ""
    sole_half = 0.038
    sole_match = [
        row for row in sole_rows
        if row[1] == toe_side and abs(row[0] - toe_t) <= session.ctrl_dt * 0.51
    ]
    if sole_match:
        _st, _ss, sole_roll, sole_drop, sole_half, sole_inside = sole_match[0]
    sole_flat = abs(sole_drop) < SOLE_FLAT_M
    pitch_rows = [
        row for row in snaps
        if row.joint.endswith("hip_pitch") and abs(row.t - 2.872) <= 0.004
    ]
    if not pitch_rows:
        pitch_rows = [
            row for row in snaps
            if row.joint.endswith("hip_pitch") and abs(row.t - toe_t) <= session.ctrl_dt * 0.51
        ]
    pitch_gap = abs(pitch_rows[0].ctrl - pitch_rows[0].ik) if pitch_rows else float("nan")
    pitch_on_ik = bool(pitch_rows) and pitch_gap <= 1e-4
    x_m = float(session.data.qpos[0])
    print(
        f"PRED {name} toe {toe_min * 1000:.3f} mm t {toe_t:.3f} "
        f"frac {toe_frac:.3f} side {toe_side} x {x_m:.3f} "
        f"x_2.872 {x_at.get('2.872', float('nan')):.3f} "
        f"x_7.000 {x_at.get('7.000', float('nan')):.3f} "
        f"fault {first_fault or 'none'} fault_t {fault_t:.3f} "
        f"min_up_z {float(session.min_up_z):.3f} clip_binds {clip_n} "
        f"period {cfg.gm_period_s:.3f} dsp {cfg.gm_dsp:.2f} "
        f"gm_move_s {used_move:.3f} pitch_slew {int(pitch_slew)} knee_slew {int(knee_slew)}"
    )
    if bar is None:
        for act, (tau, when) in leg8.items():
            print(f"PRED {name} leg {act} {tau:+.4f} t {when:.3f}")
    else:
        for act, (tau, when) in leg_pre.items():
            print(f"PRED {name} track1 {act} {tau:+.4f} t {when:.3f}")
        for act, (tau, when) in leg_post.items():
            if abs(tau) <= 1e-9:
                continue
            print(f"PRED {name} track2 {act} {tau:+.4f} t {when:.3f}")
    for jn, (tau, when) in swing_force.items():
        pred, pred_t = swing_pred[jn]
        print(
            f"PRED {name} swing {jn} force {tau:+.4f} t {when:.3f} "
            f"predicted {pred:+.4f} t {pred_t:.3f}"
        )
    print(
        f"PRED {name} over_2_33 {over} pred_over {pred_over} "
        f"track2_over {track2 or 'none'} bar_before {bar if bar is not None else 'bout'}"
    )
    if whole_toe:
        hx_names = (
            "l_hip_pitch_pos", "r_hip_pitch_pos",
            "l_knee_pos", "r_knee_pos",
            "l_ank_pitch_pos", "r_ank_pitch_pos",
        )
        for act in hx_names:
            tau, when = leg_bout[act]
            print(
                f"PRED {name} hx {act} {tau:+.4f} t {when:.3f} "
                f"abs {abs(tau):.4f} "
                f"ge_2.33 {int(abs(tau) >= KNEE_NM)} "
                f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
            )
        for act in ("l_hip_roll_pos", "r_hip_roll_pos", "l_ank_roll_pos", "r_ank_roll_pos"):
            tau, when = leg_bout[act]
            print(
                f"PRED {name} also {act} {tau:+.4f} t {when:.3f} "
                f"abs {abs(tau):.4f} "
                f"ge_2.33 {int(abs(tau) >= KNEE_NM)} "
                f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
            )
    if sole_match:
        print(
            f"PRED {name} sole_roll {sole_roll:.5f} rad "
            f"inside {toe_side} {sole_inside} drop {sole_drop * 1000:.3f} mm "
            f"half {sole_half * 1000:.1f} mm "
            f"flat {int(sole_flat)} threshold {SOLE_FLAT_M * 1000:.1f} mm "
            f"t {toe_t:.3f}"
        )
    else:
        print(f"PRED {name} sole_roll missed t {toe_t:.3f} side {toe_side}")
        raise SystemExit(f"sole roll missed at t {toe_t} side {toe_side}")
    if sole_report:
        _print_sole_diag(name, sole_log, snaps, 2.872)
        if abs(toe_t - 2.872) > session.ctrl_dt:
            _print_sole_diag(name, sole_log, snaps, toe_t)
        if whole_toe:
            near = [
                row for row in sole_log
                if abs(float(row["t"]) - toe_t) <= session.ctrl_dt * 0.51
            ]
            if near:
                row = near[0]
                print(
                    f"PRED {name} worst_midswing corner {row['scored_label']} "
                    f"z {float(row['scored_z']) * 1000:.3f} mm "
                    f"lowest {row['low_label']} {float(row['low_z']) * 1000:.3f} mm "
                    f"t {float(row['t']):.3f} side {row['side']}"
                )
    if geometry_diag:
        _print_toeup_geometry(name, sole_log, snaps, rows, toe_up_rad, toe_t)
    if lag_report:
        _print_peak_lag(
            name,
            snaps,
            list(getattr(session, "_hip_lead_log", [])),
            T_END + 1.0 if whole_toe else T_TRACK1,
        )
    _print_pred_sample(name, snaps, 2.872)
    if abs(toe_t - 2.872) > session.ctrl_dt:
        _print_pred_sample(name, snaps, toe_t)
    if ankle_roll:
        print(f"PRED {name} ank_kv {ank_kv:.4f}")
    surf = {
        "flat_toe_mm": float("nan"),
        "flat_toe_t": float("nan"),
        "flat_toe_side": "",
        "rug_clear_mm": float("nan"),
        "rug_clear_t": float("nan"),
        "rug_clear_side": "",
        "knee_l": float("nan"),
        "knee_r": float("nan"),
        "knee_rug": "",
        "knee_on_rug": 0,
        "flat_hx_over": "",
        "flat_center_mm": float("nan"),
        "flat_sole_pitch": float("nan"),
        "flat_body_mm": float("nan"),
        "flat_z_add_mm": float("nan"),
        "ka_over_n": 0,
        "ka_checked": 0,
        "mid_samples": (),
    }
    if surface_tag and surface_rows:
        surf = _surface_summary(
            name, rows, surface_rows, hx_flat, hx_rug,
            ka_hits if ka_log else None, ka_checked,
        )
    session.assert_plant_unchanged()
    return PredScore(
        name=name,
        clip=clip,
        move_s=used_move,
        toe_mm=toe_min * 1000.0,
        toe_t=toe_t,
        toe_frac=toe_frac,
        toe_side=toe_side,
        x_m=x_m,
        fault=first_fault,
        legs_under=legs_under,
        pred_under=pred_under,
        toe_clear=toe_clear,
        holds_bar=bool(legs_under and pred_under),
        snaps=snaps,
        trace=trace,
        sole_roll_rad=sole_roll,
        sole_inside_drop_mm=sole_drop * 1000.0,
        sole_inside=sole_inside,
        sole_half_m=sole_half,
        sole_flat=sole_flat,
        pitch_gap_rad=pitch_gap,
        pitch_on_ik=pitch_on_ik,
        track2_over=track2,
        flat_toe_mm=float(surf["flat_toe_mm"]),
        flat_toe_t=float(surf["flat_toe_t"]),
        flat_toe_side=str(surf["flat_toe_side"]),
        rug_clear_mm=float(surf["rug_clear_mm"]),
        rug_clear_t=float(surf["rug_clear_t"]),
        rug_clear_side=str(surf["rug_clear_side"]),
        knee_l=float(surf["knee_l"]),
        knee_r=float(surf["knee_r"]),
        knee_rug=str(surf["knee_rug"]),
        knee_on_rug=int(surf["knee_on_rug"]),
        flat_hx_over=str(surf["flat_hx_over"]),
        flat_center_mm=float(surf["flat_center_mm"]),
        flat_sole_pitch=float(surf["flat_sole_pitch"]),
        flat_body_mm=float(surf["flat_body_mm"]),
        flat_z_add_mm=float(surf["flat_z_add_mm"]),
        ka_over_n=int(surf["ka_over_n"]),
        ka_checked=int(surf["ka_checked"]),
        mid_samples=tuple(surf["mid_samples"]),  # type: ignore[arg-type]
    )


def _print_pred_sample(name: str, snaps: list[PredSnap], t_want: float) -> None:
    near = [row for row in snaps if abs(row.t - t_want) <= 0.008 * 0.51]
    if not near:
        print(f"PRED {name} sample missed {t_want:.3f}")
        return
    print(f"PRED {name} sample t {near[0].t:.3f}")
    for row in near:
        toward = (row.ik - row.q) * row.qd > 0.0
        print(
            f"PRED {name} {row.joint} ik {row.ik:.6f} ctrl {row.ctrl:.6f} "
            f"q {row.q:.6f} qd {row.qd:.6f} predicted {row.predicted:.4f} "
            f"force {row.force:.4f} ctrl_minus_ik {row.ctrl - row.ik:.6f} "
            f"ctrl_minus_q {row.ctrl - row.q:.6f} toward {int(toward)}"
        )


def _lead_at(
    lead_log: list[dict[str, float | str]],
    joint: str,
    t_want: float,
) -> dict[str, float | str] | None:
    near = [
        row for row in lead_log
        if row["joint"] == joint and abs(float(row["t"]) - t_want) <= 0.008 * 0.51
    ]
    return near[0] if near else None


def _print_peak_lag(
    name: str,
    snaps: list[PredSnap],
    lead_log: list[dict[str, float | str]],
    t_cut: float,
) -> None:
    """IK, ctrl, and q where |IK−q| peaks on each swing hip before t_cut."""
    for suffix in ("hip_roll", "hip_pitch"):
        rows = [
            row for row in snaps
            if row.joint.endswith(suffix) and row.t < t_cut - 1e-9
        ]
        if not rows:
            print(f"PRED {name} peak_lag {suffix} missed before {t_cut:.1f}")
            continue
        row = max(rows, key=lambda item: abs(item.ik - item.q))
        print(
            f"PRED {name} peak_lag {row.joint} t {row.t:.3f} "
            f"ik {row.ik:.6f} ctrl {row.ctrl:.6f} q {row.q:.6f} "
            f"qd {row.qd:.6f} ik_minus_q {row.ik - row.q:.6f} "
            f"ctrl_minus_ik {row.ctrl - row.ik:.6f} "
            f"predicted {row.predicted:.4f} force {row.force:.4f}"
        )
        led = _lead_at(lead_log, row.joint, row.t)
        if led is None:
            continue
        future = float(led["future"])
        written = float(led["ctrl"])
        same = (future - row.ik) * (written - row.ik) > 0.0
        print(
            f"PRED {name} peak_lag_lead {row.joint} "
            f"future_ik {future:.6f} written_ctrl {written:.6f} "
            f"lead_s {float(led['lead_s']) * 1000:.2f} ms "
            f"future_minus_ik {future - row.ik:.6f} "
            f"written_minus_ik {written - row.ik:.6f} "
            f"applied_minus_ik {row.ctrl - row.ik:.6f} "
            f"ctrl_ahead {int(same)} "
            f"write_clipped {int(abs(written - future) > 1e-4)}"
        )
    if not lead_log:
        return
    binds = sum(
        1 for row in lead_log
        if abs(float(row["ctrl"]) - float(row["future"])) > 1e-4
    )
    print(
        f"PRED {name} lead_writes {len(lead_log)} "
        f"write_clip_binds {binds} "
        "clip keeps predicted force inside ±2.33. Plant forcerange stays ±2.45."
    )


def _write_pred_trace(score: PredScore) -> None:
    """Per-tick swing-hip ctrl, q, and force for a before-7.0 s clear.

    The clip is sim-only. A closed position servo does not apply it.
    """
    path = Path(__file__).resolve().parents[1] / "previews" / "swing_hip_pred_clip_trace.txt"
    lines = [
        f"# {score.name} sim-only predicted clip at ±{KNEE_NM:.2f} Nm",
        "# The kit HX-35H position servo has no force clip. Do not rely on this limiter.",
        "# t joint ctrl_minus_q qd ik ctrl q predicted force",
    ]
    for row in score.trace:
        lines.append(
            f"{row.t:.3f} {row.joint} {row.ctrl - row.q:.6f} {row.qd:.6f} "
            f"{row.ik:.6f} {row.ctrl:.6f} {row.q:.6f} {row.predicted:.4f} {row.force:.4f}"
        )
    path.write_text("\n".join(lines) + "\n")
    print(f"PRED trace {path} rows {len(score.trace)}")


def score_swing_pred_clip() -> None:
    """Baseline, then the swing-hip clip at 20 ms, then move off if the bar holds."""
    base = measure_pred_clip(clip=False, move_s=None)
    held = measure_pred_clip(clip=True, move_s=None)
    opened: PredScore | None = None
    if held.holds_bar:
        opened = measure_pred_clip(clip=True, move_s=0.0)
    else:
        print("PRED move_off skipped; clip at 20 ms did not hold ±2.33")
    _compare_pred_gaps(base, held)
    if opened is not None:
        _compare_pred_gaps(base, opened)
    for score in (held, opened):
        if score is not None and score.toe_clear and score.holds_bar:
            _write_pred_trace(score)
    best = base
    for score in (held, opened):
        if score is None or not score.holds_bar:
            continue
        if score.toe_mm > best.toe_mm:
            best = score
    print(
        f"PRED best_under_2_33 {best.name} toe {best.toe_mm:.3f} mm "
        f"t {best.toe_t:.3f} side {best.toe_side} "
        f"ctrl_tracks_ik {int(best.clip and best.holds_bar and best.toe_clear)}"
    )


def _compare_pred_gaps(base: PredScore, other: PredScore) -> None:
    base_at = [row for row in base.snaps if abs(row.t - 2.872) <= 0.004]
    new_at = [row for row in other.snaps if abs(row.t - 2.872) <= 0.004]
    by_base = {row.joint: row for row in base_at}
    for row in new_at:
        prev = by_base.get(row.joint)
        if prev is None:
            continue
        prev_gap = abs(prev.ctrl - prev.ik)
        new_gap = abs(row.ctrl - row.ik)
        toward = (row.ik - row.q) * row.qd > 0.0
        print(
            f"PRED gap {other.name} {row.joint} |ctrl-ik| {new_gap:.6f} "
            f"base {prev_gap:.6f} closer {int(new_gap < prev_gap - 1e-4)} "
            f"toward {int(toward)}"
        )


def _print_pitch_verdict(score: PredScore) -> None:
    gap = score.pitch_gap_rad
    gap_s = "nan" if math.isnan(gap) else f"{gap:.6f}"
    closer = (not math.isnan(gap)) and gap < PITCH_GAP_CLIP20 - 1e-4
    print(
        f"PRED verdict {score.name} pitch_ctrl_eq_ik {int(score.pitch_on_ik)} "
        f"pitch_gap {gap_s} closer_than_clip20 {int(closer)} "
        f"toe {score.toe_mm:.3f} vs_m081 {score.toe_mm - TOE_BAND_MM:+.3f} "
        f"vs_m1002 {score.toe_mm - TOE_CLIP20_MM:+.3f} "
        f"sole_roll {score.sole_roll_rad:.5f} inside_drop_mm {score.sole_inside_drop_mm:.3f} "
        f"inside {score.sole_inside} sole_flat {int(score.sole_flat)} "
        f"track1_under {int(score.holds_bar)} toe_clear {int(score.toe_clear)} "
        f"x {score.x_m:.3f} track2 {score.track2_over or 'none'}"
    )


def score_pitch_move_off() -> None:
    """Hip-pitch approach off. Swing-hip clip stays. Period stays 0.500.

    Leg torques before 7.0 s are Track 1. A rail there stops the chain.
    If the sole is rolled and the bar holds, the next copy clips swing
    ankle roll only. If the sole is flat and the toe is still short, the
    next copy lengthens the period.
    """
    row = measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        bar_before_s=T_TRACK1,
    )
    _print_pitch_verdict(row)
    if row.toe_clear and row.holds_bar:
        _write_pred_trace(row)
    if not row.holds_bar:
        print("PRED next neither: pitch-off railed before 7.0 s. No ankle clip. No period.")
        return
    if not row.sole_flat:
        print("PRED next ank_roll clip only. Period stays 0.500. Hip-pitch approach stays off.")
        ank = measure_pred_clip(
            clip=True,
            move_s=None,
            pitch_move_off=True,
            ankle_roll=True,
            bar_before_s=T_TRACK1,
        )
        _print_pitch_verdict(ank)
        if ank.toe_clear and ank.holds_bar:
            _write_pred_trace(ank)
        if not ank.holds_bar:
            print("PRED ank_roll clip railed before 7.0 s. Stop.")
        return
    if row.toe_clear:
        print("PRED sole flat and toe cleared. No period. No ankle clip.")
        return
    print(
        "PRED next period slowdown. Sole roll is flat, the toe is still under +2 mm, "
        "and forces before 7.0 s stay under 2.33. Hip-pitch approach stays off. "
        "0.60 s is a published preset. 0.70 s is not."
    )
    for period in (0.60, 0.70):
        slow = measure_pred_clip(
            clip=True,
            move_s=None,
            pitch_move_off=True,
            period_s=period,
            bar_before_s=T_TRACK1,
        )
        scaled = slow.x_m * (period / 0.50)
        print(
            f"PRED period {period:.2f} x {slow.x_m:.3f} "
            f"x_scaled_to_0.50 {scaled:.3f} ref 1.028"
        )
        _print_pitch_verdict(slow)
        if slow.toe_clear and slow.holds_bar:
            _write_pred_trace(slow)
            print(f"PRED period {period:.2f} cleared the toe under 2.33 before 7.0 s.")
            return
        if not slow.holds_bar:
            print(f"PRED period {period:.2f} railed before 7.0 s. Stop.")
            return
    print("PRED period slowdown held 2.33 before 7.0 s and did not clear +2 mm.")


def _print_toeup_geometry(
    name: str,
    log: list[dict[str, object]],
    snaps: list[PredSnap],
    rows: list[Tick],
    peak: float,
    toe_t: float,
) -> None:
    """Contact-box print for the 0.020 rad toe-up copy. Not a clear attempt."""
    print(
        f"PRED {name} geometry_diag peak {peak:.3f} rad. "
        "Commanded rad is sign * peak * sin(pi * gait_frac), added to the "
        "ankle-pitch target before the 20 ms approach. Track 1 stays parked."
    )

    def _row_at(t_want: float) -> dict[str, object] | None:
        near = [row for row in log if abs(float(row["t"]) - t_want) <= 0.008 * 0.51]
        return near[0] if near else None

    def _cycle_frac(t_want: float) -> float:
        for cyc in _swing_cycles(rows, T_BAR):
            n = len(cyc)
            if n < 2:
                continue
            for i, row in enumerate(cyc):
                if abs(row.t - t_want) <= 0.008 * 0.51:
                    return i / (n - 1)
        return float("nan")

    def _dump(tag: str, t_want: float) -> None:
        row = _row_at(t_want)
        if row is None:
            print(f"PRED {name} {tag} missed {t_want:.3f}")
            return
        cmd = row["cmd"] if isinstance(row["cmd"], dict) else {}
        gait_frac = float(cmd.get("toe_up_frac", float("nan")))
        commanded = float(cmd.get("toe_up", float("nan")))
        sine = math.sin(math.pi * gait_frac) if gait_frac == gait_frac else float("nan")
        print(
            f"PRED {name} {tag} t {float(row['t']):.3f} side {row['side']} "
            f"scored {row['scored_label']} {float(row['scored_z']) * 1000:.3f} mm "
            f"lowest {row['low_label']} {float(row['low_z']) * 1000:.3f} mm "
            f"box_center {float(row['center_z']) * 1000:.3f} mm "
            f"sole_pitch_toe_down {float(row['pitch']):.5f} rad "
            f"sole_roll {float(row['roll']):.5f} rad "
            f"inside {row['inside']} drop {float(row['inside_drop']) * 1000:.3f} mm "
            f"cycle_frac {_cycle_frac(float(row['t'])):.3f}"
        )
        for label, z, off in row["corners"]:  # type: ignore[misc]
            mark = ""
            if label == row["low_label"]:
                mark += " LOW"
            if label == row["scored_label"]:
                mark += " SCORED"
            print(
                f"PRED {name} {tag} corner {label} z {float(z) * 1000:.3f} mm "
                f"fwd {float(off) * 1000:.1f} mm{mark}"
            )
        print(
            f"PRED {name} {tag} commanded_rad {commanded:.5f} "
            f"gait_frac {gait_frac:.3f} sine {sine:.3f} "
            f"peak {peak:.3f} of_peak {sine:.3f}"
        )
        ank = [
            s for s in snaps
            if s.joint.endswith("ank_pitch") and abs(s.t - float(row["t"])) <= 0.008 * 0.51
        ]
        for s in ank:
            print(
                f"PRED {name} {tag} ank_pitch {s.joint} ik {s.ik:.6f} "
                f"ctrl {s.ctrl:.6f} q {s.q:.6f} "
                f"ctrl_minus_ik {s.ctrl - s.ik:.6f} "
                f"ctrl_minus_ik_minus_commanded {s.ctrl - s.ik - commanded:.6f}"
            )

    _dump("at_2.872", 2.872)
    _dump("at_min", toe_t)
    # Same cycles as _mid_swing (ticks before 7.560 s). A cut at 7.0 s
    # shortens the last swing and pulls an earlier tick into the window.
    steps = [
        item for item in _mid_swing_step_mins(rows, T_BAR)
        if item[2] < T_TRACK1 - 1e-9
    ]
    print(
        f"PRED {name} step_mins n {len(steps)} "
        f"cycle_cut {T_BAR:.3f} min_before {T_TRACK1:.1f}"
    )
    for step, side, t, z, frac in steps:
        print(
            f"PRED {name} step {step} side {side} t {t:.3f} "
            f"toe {z * 1000:.3f} mm cycle_frac {frac:.3f}"
        )
    if steps:
        worst = min(steps, key=lambda item: item[3])
        print(
            f"PRED {name} step_min_before_7s step {worst[0]} side {worst[1]} "
            f"t {worst[2]:.3f} toe {worst[3] * 1000:.3f} mm "
            f"cycle_frac {worst[4]:.3f}"
        )
    print(
        f"PRED {name} not_a_clear. Track 1 stays parked. "
        "Next gate is the kit HX-35H kp/kv bench. "
        "A 0.030 rad clear was not run."
    )


TOE_TOEUP_MM = -0.013


def _unpark_kwargs(
    *,
    hip_lead: bool = False,
    period_s: float | None = None,
) -> dict[str, object]:
    return {
        "clip": True,
        "move_s": None,
        "pitch_move_off": True,
        "bar_before_s": T_TRACK1,
        "toe_up_rad": TOE_UP_PEAK,
        "sole_report": True,
        "lag_report": True,
        "hip_lead": hip_lead,
        "period_s": period_s,
    }


def _print_unpark(score: PredScore) -> None:
    print(
        f"PRED unpark {score.name} toe {score.toe_mm:.3f} mm "
        f"vs_m0013 {score.toe_mm - TOE_TOEUP_MM:+.3f} mm "
        f"holds {int(score.holds_bar)} clear {int(score.toe_clear)} "
        f"fault {score.fault or 'none'} "
        f"track2 {score.track2_over or 'none'}"
    )


def _helps_under_torque(score: PredScore) -> bool:
    """Toe rose versus −0.013 mm and every leg actuator stays under 2.33 before 7.0 s."""
    return score.legs_under and score.toe_mm > TOE_TOEUP_MM + 1e-6


def _pitch_lead_scale(ms: float) -> float:
    """Scale of the compiled hip-pitch kv/kp lag that lands on ``ms``."""
    compiled = _COMPILED_KV["hip_pitch"] / 45.0
    return (ms / 1000.0) / compiled


def _pitch20_copy(z_extra_m: float) -> PredScore:
    """Pitch lead 20 ms, toe-up 0.020, optional swing foot-z add. Roll lead 0."""
    return measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=TOE_UP_PEAK,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        whole_toe=True,
        surface_tag=True,
        z_extra_m=z_extra_m,
        ka_log=True,
    )


def _mid_at(
    samples: tuple,
    t_want: float,
    side: str,
) -> tuple | None:
    near = [
        row for row in samples
        if row[1] == side and abs(float(row[0]) - t_want) <= 0.004
    ]
    return near[0] if near else None


def score_pitch20_zlift() -> None:
    """20 ms pitch lead. Foot-z sweep starts at +2 mm minus that copy's box centre.

    Toe-up stays 0.020 rad. The 40.23 ms centre is not the source of the
    minimum. Knee and ankle pitch are checked on every tick.
    """
    base = _pitch20_copy(0.0)
    centre = base.flat_center_mm
    zmin = 2.0 - centre
    print(
        f"PRED zlift_min pitch20 box_centre {centre:.3f} mm "
        f"t {base.flat_toe_t:.3f} side {base.flat_toe_side} "
        f"toe {base.flat_toe_mm:.3f} mm "
        f"sole_pitch {base.flat_sole_pitch:.5f} rad "
        f"front_below_centre {centre - base.flat_toe_mm:.3f} mm "
        f"foot_z_min {zmin:.3f} mm"
    )
    start = zmin
    if zmin < 0.0:
        print(
            "PRED zlift_min box centre is already at or above +2 mm. "
            "The sweep starts at 0 mm."
        )
        start = 0.0
    lifts = [start + 0.4 * i for i in range(5)]
    if abs(lifts[0]) <= 1e-9:
        lifts = lifts[1:]
    print(
        "PRED zlift_plan "
        + " ".join(f"{mm:.3f}" for mm in lifts)
        + " mm above the 12 mm phase bump. Spacing 0.4 mm."
    )
    cleared: list[PredScore] = []
    for mm in lifts:
        row = _pitch20_copy(mm / 1000.0)
        same = _mid_at(base.mid_samples, row.flat_toe_t, row.flat_toe_side)
        if same is None:
            d_toe = float("nan")
            d_centre = float("nan")
            d_body = float("nan")
            base_toe = float("nan")
            base_centre = float("nan")
            base_body = float("nan")
        else:
            base_toe = float(same[2])
            base_centre = float(same[3])
            base_body = float(same[4])
            d_toe = row.flat_toe_mm - base_toe
            d_centre = row.flat_center_mm - base_centre
            d_body = row.flat_body_mm - base_body
        holds = row.flat_hx_over == ""
        clear = holds and row.flat_toe_mm >= 2.0
        print(
            f"PRED zlift {mm:.3f} mm {row.name} "
            f"flat_toe {row.flat_toe_mm:.3f} mm t {row.flat_toe_t:.3f} "
            f"side {row.flat_toe_side} "
            f"box_centre {row.flat_center_mm:.3f} mm "
            f"sole_pitch {row.flat_sole_pitch:.5f} rad "
            f"z_add {row.flat_z_add_mm:.3f} mm "
            f"d_toe {d_toe:.3f} mm d_centre {d_centre:.3f} mm "
            f"d_body {d_body:.3f} mm "
            f"base_toe {base_toe:.3f} base_centre {base_centre:.3f} "
            f"base_body {base_body:.3f} "
            f"flat_hx {row.flat_hx_over or 'under'} "
            f"holds_flat {int(holds)} clear {int(clear)} "
            f"ka_over {row.ka_over_n} ka_checked {row.ka_checked}"
        )
        if mm > 1e-6:
            if d_centre == d_centre and d_centre >= 0.7 * mm:
                split = "cmd"
            elif d_body == d_body and d_body <= -0.3 * mm and (
                d_centre != d_centre or d_centre < 0.7 * mm
            ):
                split = "crouch"
            else:
                split = "sag"
        else:
            split = "no_add"
        print(
            f"PRED zlift_split {mm:.3f} mm {split} "
            f"cmd_add {mm:.3f} mm world_centre_delta {d_centre:.3f} mm "
            f"body_delta {d_body:.3f} mm toe_delta {d_toe:.3f} mm"
        )
        if clear:
            cleared.append(row)
    if not cleared:
        print(
            "PRED zlift Prefer FAIL. No foot-z add cleared a flat mid-swing "
            "contact-box toe of +2 mm with hip pitch, knee, and ankle pitch "
            "at or under 2.33 Nm on flat samples. Pitch lead stays 20 ms. "
            "Toe-up stays 0.020 rad. Period stays 0.500 s."
        )
        return
    best = max(cleared, key=lambda row: row.flat_toe_mm)
    print(
        f"PRED zlift clear {best.name} flat_toe {best.flat_toe_mm:.3f} mm "
        f"t {best.flat_toe_t:.3f} side {best.flat_toe_side}"
    )


def _surface_copy(pitch_scale: float | None) -> PredScore:
    """Baseline, or a pitch-only lead. Roll lead stays 0. Period stays 0.500 s."""
    return measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=TOE_UP_PEAK,
        hip_lead=pitch_scale is not None,
        lead_roll_scale=0.0,
        lead_pitch_scale=1.0 if pitch_scale is None else pitch_scale,
        whole_toe=True,
        surface_tag=True,
    )


def score_surface_pitch() -> None:
    """Knee at 7.432 s, then flat-floor vs rug toes, then shorter pitch leads.

    The 40.23 ms lead stays when the 7.432 s right knee is over 2.33 Nm
    and the feet are over the rug in plan. Track 1 toe is the flat-floor
    mid-swing only. Rug swings are reported on their own clearance.
    """
    base = _surface_copy(None)
    full = _surface_copy(1.0)
    rug_knee = bool(full.knee_on_rug) and abs(full.knee_r) >= KNEE_NM
    base_over = abs(base.knee_r) >= KNEE_NM or abs(base.knee_l) >= KNEE_NM
    if rug_knee and base_over:
        print(
            "PRED pitchlead_stays. The 7.432 s knee is on the rug and the "
            f"no-lead knee is also over 2.33 Nm "
            f"(L {base.knee_l:+.3f} R {base.knee_r:+.3f}). Track 2. "
            "Cutting the 40.23 ms pitch lead is not the knee fix."
        )
        stays = True
    elif rug_knee:
        print(
            "PRED pitchlead_rug_knee. The 7.432 s right knee is over 2.33 Nm "
            "while the feet are over the rug in plan. "
            f"No-lead knees at that tick are L {base.knee_l:+.3f} "
            f"R {base.knee_r:+.3f}."
        )
        stays = True
    else:
        print(
            "PRED pitchlead_knee_not_rug. The 7.432 s sample is not on the rug. "
            f"No-lead L {base.knee_l:+.3f} R {base.knee_r:+.3f}. "
            f"Pitch-lead L {full.knee_l:+.3f} R {full.knee_r:+.3f}."
        )
        stays = False
    print(
        f"PRED track1_flat {full.name} toe {full.flat_toe_mm:.3f} mm "
        f"t {full.flat_toe_t:.3f} side {full.flat_toe_side} "
        f"rug_clear {full.rug_clear_mm:.3f} mm t {full.rug_clear_t:.3f} "
        f"side {full.rug_clear_side} flat_hx_over {full.flat_hx_over or 'none'} "
        f"stays {int(stays)}"
    )
    held: list[PredScore] = []
    for ms in (30.0, 20.0, 10.0):
        row = _surface_copy(_pitch_lead_scale(ms))
        holds = row.flat_hx_over == ""
        print(
            f"PRED pitchsweep {ms:.0f} ms {row.name} "
            f"flat_toe {row.flat_toe_mm:.3f} mm t {row.flat_toe_t:.3f} "
            f"side {row.flat_toe_side} "
            f"rug_clear {row.rug_clear_mm:.3f} mm "
            f"flat_hx {row.flat_hx_over or 'under'} "
            f"holds_flat {int(holds)} "
            f"whole_toe {row.toe_mm:.3f} mm"
        )
        if holds:
            held.append(row)
    thirty = next((row for row in held if "pitch30" in row.name), None)
    if thirty is None:
        print("PRED pitchsweep 35 ms skipped. 30 ms did not hold the flat pitch chain.")
    else:
        row = _surface_copy(_pitch_lead_scale(35.0))
        holds = row.flat_hx_over == ""
        print(
            f"PRED pitchsweep 35 ms {row.name} "
            f"flat_toe {row.flat_toe_mm:.3f} mm t {row.flat_toe_t:.3f} "
            f"side {row.flat_toe_side} "
            f"rug_clear {row.rug_clear_mm:.3f} mm "
            f"flat_hx {row.flat_hx_over or 'under'} "
            f"holds_flat {int(holds)} "
            f"whole_toe {row.toe_mm:.3f} mm"
        )
        if holds:
            held.append(row)
    if not held:
        print(
            "PRED pitchsweep none held hip pitch, knee, and ankle pitch "
            "at or under 2.33 Nm while that foot was off the rug. "
            f"40.23 ms stays {int(stays)}."
        )
        return
    best = max(held, key=lambda row: row.flat_toe_mm)
    print(
        f"PRED pitchsweep best_flat {best.name} toe {best.flat_toe_mm:.3f} mm "
        f"t {best.flat_toe_t:.3f} side {best.flat_toe_side} "
        f"clear {int(best.flat_toe_mm > TOE_BAR_M * 1000.0)} "
        f"40.23_stays {int(stays)}. Rug swings are not this toe."
    )


def score_pitch_only_lead() -> None:
    """Pitch-only kv/kp lead at the locked 0.500 s period.

    Roll lead is zero. Hip-roll timing and the ankle-roll pair stay on
    the baseline write. The clear is the worst 20–80% contact-box toe
    on the whole bout. Half-roll lead and the extra foot-z lift wait
    until this copy tips.
    """
    row = measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=TOE_UP_PEAK,
        sole_report=True,
        lag_report=True,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=1.0,
        whole_toe=True,
    )
    print(
        f"PRED pitchlead {row.name} toe {row.toe_mm:.3f} mm "
        f"t {row.toe_t:.3f} frac {row.toe_frac:.3f} side {row.toe_side} "
        f"holds {int(row.holds_bar)} clear {int(row.toe_clear)} "
        f"fault {row.fault or 'none'} period 0.500"
    )
    if row.toe_clear and row.legs_under:
        print(f"PRED pitchlead clear {row.name} toe {row.toe_mm:.3f} mm")
        return
    print(
        f"PRED pitchlead Prefer FAIL {row.name} toe {row.toe_mm:.3f} mm "
        f"legs_under {int(row.legs_under)} clear {int(row.toe_clear)}. "
        "No half-roll lead. No foot-z add. Period stays 0.500 s."
    )


def score_unpark_track1() -> None:
    """Hip kv/kp lead, then a longer period, then both if one holds 2.33.

    Base is the swing-hip clip, hip pitch off the 20 ms approach, and the
    0.020 rad toe-up. Period copies change only the period. dsp stays 0.20
    and y_swap stays 0.020. The hip lead is kv/kp on hip roll and hip pitch.
    """
    hip = measure_pred_clip(**_unpark_kwargs(hip_lead=True))
    _print_unpark(hip)
    if hip.toe_clear and hip.legs_under:
        _write_pred_trace(hip)
        print(f"PRED unpark clear {hip.name} toe {hip.toe_mm:.3f} mm")
    periods: list[tuple[float, PredScore]] = []
    for period in (0.60, 0.65):
        row = measure_pred_clip(**_unpark_kwargs(period_s=period))
        swing = (1.0 - 0.20) * period / 2.0
        print(
            f"PRED unpark period {period:.2f} swing {swing:.3f} s "
            f"vs_0.200 {swing / 0.200:.3f} dsp 0.20 y_swap 0.020 no_hip_lead"
        )
        _print_unpark(row)
        periods.append((period, row))
        if row.toe_clear and row.legs_under:
            _write_pred_trace(row)
            print(f"PRED unpark clear {row.name} toe {row.toe_mm:.3f} mm")
    helped = [(None, hip)] if _helps_under_torque(hip) else []
    helped.extend((period, row) for period, row in periods if _helps_under_torque(row))
    if not helped:
        print(
            "PRED unpark no_combine. Neither the hip lead nor the longer period "
            "raised the toe while every leg stayed under 2.33 before 7.0 s."
        )
        return
    period_help = [(period, row) for period, row in periods if _helps_under_torque(row)]
    if period_help:
        period_s, _best = max(period_help, key=lambda item: item[1].toe_mm)
    else:
        period_s = 0.60
    both = measure_pred_clip(**_unpark_kwargs(hip_lead=True, period_s=period_s))
    swing = (1.0 - 0.20) * period_s / 2.0
    print(
        f"PRED unpark combine period {period_s:.2f} swing {swing:.3f} s "
        f"because {[row.name for _period, row in helped]}"
    )
    _print_unpark(both)
    if both.toe_clear and both.legs_under:
        _write_pred_trace(both)
        print(f"PRED unpark clear {both.name} toe {both.toe_mm:.3f} mm")
        return
    print(
        f"PRED unpark combine_short {both.name} toe {both.toe_mm:.3f} mm "
        f"legs_under {int(both.legs_under)}."
    )


def score_toeup_diag() -> None:
    """Print the 0.020 rad toe-up copy. Do not try to clear +2 mm."""
    measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        bar_before_s=T_TRACK1,
        toe_up_rad=TOE_UP_PEAK,
        sole_report=True,
        geometry_diag=True,
    )


def score_sole_frame() -> None:
    """Contact-box print, then world-level and toe-up as separate copies.

    Both copies keep the swing-hip clip and hip-pitch approach off.
    Period stays 0.500 s. The ankle-roll clip stays off. Torques before
    7.0 s are Track 1.
    """
    base = measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        bar_before_s=T_TRACK1,
        sole_report=True,
    )
    _print_pitch_verdict(base)
    leveled = measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        bar_before_s=T_TRACK1,
        world_level=True,
        sole_report=True,
    )
    _print_pitch_verdict(leveled)
    print(
        f"PRED worldlevel_vs_pitchoff toe {leveled.toe_mm - TOE_PITCHOFF_MM:+.3f} mm "
        f"holds {int(leveled.holds_bar)}"
    )
    raised = measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        bar_before_s=T_TRACK1,
        toe_up_rad=TOE_UP_PEAK,
        sole_report=True,
    )
    _print_pitch_verdict(raised)
    print(
        f"PRED toeup_base pitchoff toe {raised.toe_mm - TOE_PITCHOFF_MM:+.3f} mm "
        f"holds {int(raised.holds_bar)} peak {TOE_UP_PEAK:.3f} rad"
    )
    best = base
    for score in (leveled, raised):
        if score.holds_bar and score.toe_mm > best.toe_mm:
            best = score
        if score.toe_clear and score.holds_bar:
            _write_pred_trace(score)
    if best.toe_clear and best.holds_bar:
        print(f"PRED clear {best.name} toe {best.toe_mm:.3f} mm")
        return
    print(
        f"PRED park_track1 best_under_2.33_before_7s {best.name} "
        f"toe {best.toe_mm:.3f} mm. Next gate is the kit kp/kv bench. "
        "Do not thaw dampratio or kp."
    )


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
