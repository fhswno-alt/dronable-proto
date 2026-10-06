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
from types import SimpleNamespace

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
# Continuous-walk swing-z rate. Knee kv is 1.4573, so 1.55 rad/s keeps
# that velocity term near 2.26 Nm. The parked stop stretch stays at
# 1.90 rad/s and 0.25 / 1.060×. This rate is not that knob.
_WALK_Z_RATE = 1.55
# Loaded-leg stance rates on the continuous walk. Ankle pitch stays at
# the 1.90 rad/s rate that already cleared it. The knee is slower on
# its own when that rate still leaves mid-SS over 2.33 Nm. Hip pitch
# stays at the rate that cleared it and is not dragged down with the
# knee. The stop bout does not read these.
_WALK_STANCE_RATE = 1.90
_WALK_KNEE_RATE = 1.70
_WALK_HIP_PITCH_RATE = 1.90
# Locked 1.55 copy, no hip slew, CoM tick 2.200 s: stance corner −2.564 mm.
# A mid-stance tick more than 1 mm past that is a new dig.
_STANCE_DIG_M = -0.003564
# Swing ankle trim cap. This is not a world-level sole and not body roll.
ANK_TRIM_CAP = 0.025
# Trim-lead schedule. Full through 80% of swing, command back to 0 by 92%
# so the lookahead is off before touchdown. Not the toe-up ramp.
TRIM_HOLD_FRAC = 0.80
TRIM_OFF_FRAC = 0.92
# "About 0.6 mm" for the centre minus front-outside drop. Outside this, no trim.
SOLE_FO_LO_MM = 0.50
SOLE_FO_HI_MM = 0.70
# Hardware: pitch axis ~11 mm behind the box centre, roll axis ~14 mm inboard.
PITCH_AXIS_MM = 11.0
ROLL_AXIS_MM = 14.0
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


def _foot_surface(session: sw.SteerSession, side: str, rug_gid: int) -> dict[str, object]:
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
    dist = float("nan")
    for i in range(session.data.ncon):
        con = session.data.contact[i]
        g1, g2 = int(con.geom1), int(con.geom2)
        if g1 != gid and g2 != gid:
            continue
        other = g2 if g1 == gid else g1
        nm = mj.mj_id2name(session.model, mj.mjtObj.mjOBJ_GEOM, other) or str(other)
        if nm not in names:
            names.append(nm)
        # MuJoCo dist is the gap. Negative is penetration.
        gap = float(con.dist)
        if dist != dist or gap < dist:
            dist = gap
    corners = _box_bottom_corners(session.model, session.data, bid, gid)
    fwd = body_forward_xy(session.data, session.bid_body)
    origin = np.asarray(session.data.xpos[session.bid_body, :2], dtype=np.float64)
    lead = max(corners, key=lambda corner: float(np.dot(corner[:2] - origin, fwd)))
    n_in = sum(1 for corner in corners if _xy_in_rug(session, rug_gid, corner))
    lead_in = _xy_in_rug(session, rug_gid, lead)
    on_rug = n_in > 0 or rug_n > 0.5
    contact = _contact_kind(names)
    if lead_in:
        surface = "rug"
        clear = float(lead[2]) - RUG_TOP_M
    else:
        surface = "floor"
        clear = float(lead[2])
    center_z = float(sum(float(corner[2]) for corner in corners) / len(corners))
    labeled = tuple(
        (label, float(z)) for label, z, _off in _box_corner_table(session, side)
    )
    return {
        "floor_n": floor_n,
        "rug_n": rug_n,
        "fn": floor_n + rug_n,
        "dist": dist,
        "geoms": ",".join(names) if names else "-",
        "corners_in": n_in,
        "lead_in": int(lead_in),
        "lead_z": float(lead[2]),
        "center_z": center_z,
        "sole_pitch": _sole_pitch(session, side),
        "on_rug": int(on_rug),
        "contact": contact,
        "surface": surface,
        "clear": clear,
        "corners": labeled,
    }


def _contact_kind(names: list[str]) -> str:
    """Geom the foot is touching. Plan position over the rug is not contact."""
    floor = any(nm == "floor" for nm in names)
    rug = any("rug" in nm for nm in names)
    if floor and rug:
        return "floor+rug"
    if floor:
        return "floor"
    if rug:
        return "rug"
    return "none"


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
    print(
        f"PRED swing_z_add {extra_m * 1000.0:.3f} mm on the swing foot "
        "during single support. Stance z is unchanged."
    )

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


def _walk_z_ik(
    walker: ow.Op3Walker, side: str, t: float, z_cmd: float,
) -> tuple[float, float] | None:
    """Signed knee and ankle pitch if this swing z replaces the nominal one."""
    saved = walker.time
    walker.time = float(t)
    er, el, *_rest = walker.endpoints()
    ep = np.array(el if side == "L" else er, dtype=np.float64)
    cur = float(walker._left_z(t) if side == "L" else walker._right_z(t))
    ep[2] = float(ep[2]) + (float(z_cmd) - cur)
    raw = ow.ik_leg(
        walker.lengths,
        float(ep[0]), float(ep[1]), float(ep[2]),
        float(ep[3]), float(ep[4]), float(ep[5]),
    )
    walker.time = saved
    if raw is None:
        return None
    names = ow._LEG_L if side == "L" else ow._LEG_R
    signed = walker._apply_direction(raw, names)
    return float(signed[3]), float(signed[4])


def _fit_walk_z_side(
    walker: ow.Op3Walker, side: str, rate: float,
) -> dict[str, object]:
    """Highest swing z whose knee and ankle pitch stay at ``rate`` and still land.

    The clock stays the real SSP. z is the only warped channel. A schedule
    that cannot reach the landing z at this rate is refused.
    """
    z_fn = walker._left_z if side == "L" else walker._right_z
    start = float(walker.l_ssp_start if side == "L" else walker.r_ssp_start)
    end = float(walker.l_ssp_end if side == "L" else walker.r_ssp_end)
    dt = float(ow.OP3_CTRL_S)
    ts: list[float] = []
    nom: list[float] = []
    t = start
    while True:
        ts.append(t)
        nom.append(float(z_fn(t)))
        if t >= end - 1e-12:
            break
        t = min(end, t + dt)
    n = len(ts)
    span = end - start
    fracs = [(ts[i] - start) / span for i in range(n)]
    zend = float(nom[-1])
    # 161 samples is the grid that still connects knee and ankle pitch
    # back to the landing z at 1.40 rad/s. A finer grid misses that chain.
    grid = np.linspace(min(nom) - 0.004, max(nom) + 0.002, 161)
    cache: list[list[tuple[float, float] | None]] = []
    for i in range(n):
        cache.append([_walk_z_ik(walker, side, ts[i], float(z)) for z in grid])
    step = float(rate) * dt
    term = int(np.argmin(np.abs(grid - zend)))
    if cache[-1][term] is None:
        raise SystemExit(f"walk z stretch {side} has no landing IK")
    reachable: list[set[int]] = [set() for _ in range(n)]
    reachable[-1].add(term)
    for i in range(n - 2, -1, -1):
        for j, ik in enumerate(cache[i]):
            if ik is None:
                continue
            for k in reachable[i + 1]:
                nxt = cache[i + 1][k]
                if nxt is None:
                    continue
                if (
                    abs(ik[0] - nxt[0]) <= step + 1e-8
                    and abs(ik[1] - nxt[1]) <= step + 1e-8
                ):
                    reachable[i].add(j)
                    break
    if not reachable[0]:
        raise SystemExit(
            f"walk z stretch {side} cannot land at {rate:.2f} rad/s"
        )
    j0 = min(reachable[0], key=lambda j: abs(float(grid[j]) - nom[0]))
    path = [j0]
    for i in range(n - 1):
        ik = cache[i][path[-1]]
        if ik is None:
            raise SystemExit(f"walk z stretch {side} lost the IK")
        best: int | None = None
        best_key = -1e9
        for k in reachable[i + 1]:
            nxt = cache[i + 1][k]
            if nxt is None:
                continue
            if (
                abs(ik[0] - nxt[0]) > step + 1e-8
                or abs(ik[1] - nxt[1]) > step + 1e-8
            ):
                continue
            # Climb while the 0.208 toe is still ahead, then come back to
            # the landing z. Both stay inside the rate step.
            if fracs[i + 1] <= 0.45:
                key = float(grid[k])
            else:
                key = -abs(float(grid[k]) - zend)
            if best is None or key > best_key:
                best = k
                best_key = key
        if best is None:
            raise SystemExit(f"walk z stretch {side} broke at frac {fracs[i]:.3f}")
        path.append(best)
    table = [(float(fracs[i]), float(grid[path[i]])) for i in range(n)]
    idx208 = min(range(n), key=lambda i: abs(fracs[i] - 0.208))
    peak_k = 0.0
    peak_a = 0.0
    peak_kf = 0.0
    peak_af = 0.0
    prev = cache[0][path[0]]
    for i in range(1, n):
        ik = cache[i][path[i]]
        if ik is None or prev is None:
            prev = ik
            continue
        dk = abs(ik[0] - prev[0]) / dt
        da = abs(ik[1] - prev[1]) / dt
        if 0.20 - 1e-9 <= fracs[i] <= 0.80 + 1e-9:
            if dk > peak_k:
                peak_k = dk
                peak_kf = fracs[i]
            if da > peak_a:
                peak_a = da
                peak_af = fracs[i]
        prev = ik
    return {
        "table": table,
        "z208": float(grid[path[idx208]]),
        "frac208": float(fracs[idx208]),
        "zend": zend,
        "z_end": float(grid[path[-1]]),
        "z_max": float(max(grid[j] for j in path)),
        "peak_knee": peak_k,
        "peak_knee_frac": peak_kf,
        "peak_ank": peak_a,
        "peak_ank_frac": peak_af,
    }


def _install_walk_z_stretch(session: sw.SteerSession, rate: float = _WALK_Z_RATE) -> None:
    """Time-shape continuous-walk swing z so knee and ankle pitch can track.

    Installed only on the walk score, after the 12 mm phase lift and the
    1.170 mm foot-z add. The stop bout does not call this. Stance z is the
    original freeze. The joint slew that tracks this z lives in the
    sagittal write and stays off when the stop is installed.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        raise RuntimeError("gait manager walker missing")
    if float(rate) <= 0.0:
        raise SystemExit(f"walk z stretch rate {rate} is not positive")
    walker = lipm.op3
    saved_time = float(walker.time)
    saved_prev = float(walker.previous_x)
    saved_x = float(walker.x_cmd)
    saved_y = float(walker.y_cmd)
    saved_a = float(walker.angle_cmd)
    saved_run = bool(walker.ctrl_running)
    saved_amp = (
        walker._x_move, walker._x_swap, walker._y_move, walker._y_move_shift,
        walker._y_swap, walker._z_move, walker._z_move_shift, walker._z_swap,
        walker._z_swap_shift, walker._a_move, walker._a_move_shift,
    )
    try:
        # Steady Day-1 step. The first half-step is not the fit.
        walker.x_cmd = 0.020
        walker.y_cmd = 0.0
        walker.angle_cmd = 0.0
        walker.previous_x = 0.020
        walker.ctrl_running = True
        walker.update_movement()
        left = _fit_walk_z_side(walker, "L", float(rate))
        right = _fit_walk_z_side(walker, "R", float(rate))
    finally:
        walker.time = saved_time
        walker.previous_x = saved_prev
        walker.x_cmd = saved_x
        walker.y_cmd = saved_y
        walker.angle_cmd = saved_a
        walker.ctrl_running = saved_run
        (
            walker._x_move, walker._x_swap, walker._y_move, walker._y_move_shift,
            walker._y_swap, walker._z_move, walker._z_move_shift, walker._z_swap,
            walker._z_swap_shift, walker._a_move, walker._a_move_shift,
        ) = saved_amp
    session._walk_z_stretch = True  # type: ignore[attr-defined]
    session._walk_z_rate = float(rate)  # type: ignore[attr-defined]
    session._walk_z_fit = {"L": left, "R": right}  # type: ignore[attr-defined]

    def _warp(orig, fit: dict[str, object], start: float, end: float):
        table = fit["table"]
        if not isinstance(table, list) or len(table) < 2:
            raise SystemExit("walk z stretch table is empty")
        fracs = [float(row[0]) for row in table]
        zs = [float(row[1]) for row in table]

        def z_fn(t: float) -> float:
            tt = float(t)
            if tt <= start or tt > end:
                return float(orig(tt))
            frac = (tt - start) / (end - start)
            if frac <= fracs[0]:
                return zs[0]
            if frac >= fracs[-1]:
                return zs[-1]
            hi = 1
            while hi < len(fracs) - 1 and fracs[hi] < frac:
                hi += 1
            lo = hi - 1
            span = fracs[hi] - fracs[lo]
            w = 0.0 if span <= 1e-9 else (frac - fracs[lo]) / span
            return zs[lo] * (1.0 - w) + zs[hi] * w

        return z_fn

    orig_l = walker._left_z
    orig_r = walker._right_z
    walker._left_z = _warp(orig_l, left, float(walker.l_ssp_start), float(walker.l_ssp_end))  # type: ignore[method-assign]
    walker._right_z = _warp(orig_r, right, float(walker.r_ssp_start), float(walker.r_ssp_end))  # type: ignore[method-assign]
    print(
        "PRED walk_z_stretch "
        f"rate {float(rate):.3f} rad/s. "
        f"L z208 {float(left['z208']) * 1000.0:+.2f} mm "
        f"peak {float(left['z_max']) * 1000.0:+.2f} "
        f"end {float(left['z_end']) * 1000.0:+.2f} "
        f"land {float(left['zend']) * 1000.0:+.2f} "
        f"open knee {float(left['peak_knee']):.2f} rad/s "
        f"at {float(left['peak_knee_frac']):.3f} "
        f"ank {float(left['peak_ank']):.2f} "
        f"at {float(left['peak_ank_frac']):.3f}. "
        f"R z208 {float(right['z208']) * 1000.0:+.2f} mm "
        f"end {float(right['z_end']) * 1000.0:+.2f}. "
        "Swing knee and ankle pitch slew at this rate. "
        "The stop stretch stays 0.25 / 1.060× and is not armed here. "
        "Foot-z stays 1.170 mm. y_swap stays 0."
    )


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


def _toe_up_ramp(frac: float, shape: str, full_frac: float = 0.15) -> float:
    """Unit toe-up vs single-support fraction. Peak stays the caller's rad."""
    if shape == "sine":
        return math.sin(math.pi * frac)
    if shape != "front":
        raise SystemExit(f"toe-up shape {shape} is not sine or front")
    if not 0.0 < full_frac < 0.80:
        raise SystemExit(f"toe-up full fraction {full_frac} is not inside 0-80%")
    # Full by full_frac of single support, held through 80%, back to 0 at touchdown.
    if frac < full_frac:
        u = frac / full_frac
        return u * u * (3.0 - 2.0 * u)
    if frac <= 0.80:
        return 1.0
    u = (frac - 0.80) / 0.20
    u = min(1.0, max(0.0, u))
    return 1.0 - u * u * (3.0 - 2.0 * u)


def _install_swing_toe_up(
    session: sw.SteerSession,
    peak: float,
    shape: str = "sine",
    full_frac: float = 0.15,
) -> None:
    """Add a swing ankle toe-up that is zero at toe-off and at touchdown.

    The peak is under 0.03 rad. ``sine`` rises from 0 at toe-off.
    ``front`` is already at that peak by ``full_frac`` of single support
    and holds it through 80%. The landed copy is full by 15%. Knee and
    ankle pitch keep the 20 ms approach. This does not world-level the sole.
    """
    if not 0.0 < peak < 0.03 - 1e-12:
        raise SystemExit(f"toe-up peak {peak} rad is not under 0.03")
    if shape not in ("sine", "front"):
        raise SystemExit(f"toe-up shape {shape} is not sine or front")
    if shape == "front" and not 0.0 < full_frac < 0.80:
        raise SystemExit(f"toe-up full fraction {full_frac} is not inside 0-80%")
    signs = {side: _ank_pitch_raises_front(session, side) for side in ("L", "R")}
    label = (
        f"front full by {full_frac * 100:.0f}% hold through 80%"
        if shape == "front"
        else "sine"
    )
    print(
        f"PRED toe_up_sign L {signs['L']:+.0f} R {signs['R']:+.0f} "
        f"peak {peak:.3f} rad {label}"
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
        ramp = _toe_up_ramp(float(frac), shape, full_frac)
        delta = signs[swing] * peak * ramp
        act = ("l_" if swing == "L" else "r_") + "ank_pitch_pos"
        idx = session.act_idx[act]
        session.data.ctrl[idx] = float(session.data.ctrl[idx]) + delta
        cmd = getattr(session, "_sole_cmd", None)
        if cmd is not None and cmd.get("side") == swing:
            cmd["toe_up"] = delta
            cmd["toe_up_frac"] = float(frac)

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]


def _install_yswap_lead(session: sw.SteerSession, lead_s: float) -> None:
    """Sample the existing lateral shift a few milliseconds earlier.

    Amplitude stays the walker's y_swap command. Both ankle rolls still
    take the same shift times lat. This does not add a one-sided ankle offset.
    """
    if lead_s <= 0.0:
        raise SystemExit(f"yswap lead {lead_s} s is not positive")
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        raise RuntimeError("gait manager walker missing")
    walker = lipm.op3
    orig = walker._swap_y_before_swing
    print(
        f"PRED yswap_lead {lead_s * 1000:.1f} ms. "
        f"y_swap amplitude stays {walker.y_swap_cmd:.3f} m. "
        "The lateral shift is sampled that much earlier. "
        "Ankle-roll pair stays the same lat on both feet."
    )

    def led(t: float) -> float:
        return float(orig(float(t) + float(lead_s)))

    walker._swap_y_before_swing = led  # type: ignore[method-assign]


def _trim_lead_shape(frac: float, on_frac: float) -> float:
    """Unit trim vs swing fraction. 0 before lift-off, 0 again before touchdown."""
    if frac < on_frac:
        return 0.0
    if frac <= TRIM_HOLD_FRAC:
        return 1.0
    if frac >= TRIM_OFF_FRAC:
        return 0.0
    u = (frac - TRIM_HOLD_FRAC) / (TRIM_OFF_FRAC - TRIM_HOLD_FRAC)
    return 1.0 - u * u * (3.0 - 2.0 * u)


def _compiled_ank_lag(session: sw.SteerSession, jn: str) -> tuple[float, float, float]:
    """Compiled kv, kp, and kv/kp for one ankle joint. kv is not invented."""
    _q, _qd, idx = _joint_q_qd(session, jn)
    kv = -float(session.model.actuator_biasprm[idx, 2])
    kp = float(session.model.actuator_gainprm[idx, 0])
    if abs(kp - 35.0) > 1e-3:
        raise SystemExit(f"compiled kp for {jn} is {kp:.4f}, expected 35")
    if kv <= 0.0:
        raise SystemExit(f"compiled kv for {jn} is {kv:.4f}")
    return kv, kp, kv / kp


def _install_swing_ank_roll_trim(
    session: sw.SteerSession,
    add_l: float,
    add_r: float,
    pitch_l: float = 0.0,
    pitch_r: float = 0.0,
    lead: bool = False,
    on_frac: float = 0.208,
) -> None:
    """Add the leftover-sole trim on the swing ankle. Stance is not rewritten.

    Roll and pitch adds are the leftover sole at the first scored tick,
    in joint radians, each capped at ±0.025. Pitch stays 0 on a roll-only
    copy. This is not a world-level sole and it does not cancel body roll.
    ``lead`` advances only that add by the ankle's own kv/kp. The toe-up,
    the IK ankle, and the hip stay on their own clocks. The add stays 0
    before ``on_frac``, so the lookahead cannot push a loaded foot.
    """
    if abs(add_l) > ANK_TRIM_CAP + 1e-12 or abs(add_r) > ANK_TRIM_CAP + 1e-12:
        raise SystemExit(
            f"ankle-roll trim L {add_l} R {add_r} exceeds ±{ANK_TRIM_CAP:.3f}"
        )
    if abs(pitch_l) > ANK_TRIM_CAP + 1e-12 or abs(pitch_r) > ANK_TRIM_CAP + 1e-12:
        raise SystemExit(
            f"ankle-pitch trim L {pitch_l} R {pitch_r} exceeds ±{ANK_TRIM_CAP:.3f}"
        )
    if lead and not (0.0 < on_frac < TRIM_HOLD_FRAC):
        raise SystemExit(f"ankle trim on fraction {on_frac} is not inside 0-80%")
    pitch_on = abs(pitch_l) > 1e-12 or abs(pitch_r) > 1e-12
    if pitch_on:
        print(
            f"PRED level_trim roll L {add_l:+.5f} R {add_r:+.5f} "
            f"pitch L {pitch_l:+.5f} R {pitch_r:+.5f} rad "
            f"cap ±{ANK_TRIM_CAP:.3f} rad each. Swing ankle only. "
            "Leftover sole, not body roll. Not a world-level sole. "
            "Stance ankles are not rewritten."
        )
    else:
        print(
            f"PRED ank_roll_trim L {add_l:+.5f} rad R {add_r:+.5f} rad "
            f"cap ±{ANK_TRIM_CAP:.3f} rad. Swing ankle roll only. "
            "Not a world-level sole. Stance ankle roll is not rewritten. "
            "Body roll is not cancelled by this add."
        )
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        raise RuntimeError("gait manager walker missing")
    lags: dict[str, float] = {}
    if lead:
        for jn in ("l_ank_roll", "r_ank_roll", "l_ank_pitch", "r_ank_pitch"):
            kv, kp, lag = _compiled_ank_lag(session, jn)
            lags[jn] = lag
            note = ""
            if jn.endswith("ank_roll") and abs(kv - 1.1876) > 1e-3:
                note = " kv_note compiled_not_1.1876"
            print(
                f"PRED trim_lead {jn} kp {kp:.4f} kv {kv:.4f} "
                f"lag {lag * 1000:.2f} ms{note}"
            )
        for kind in ("ank_roll", "ank_pitch"):
            split_ms = (lags["l_" + kind] - lags["r_" + kind]) * 1000.0
            if abs(split_ms) > 1.0:
                print(
                    f"PRED trim_lead {kind} lag_split {split_ms:+.2f} ms. "
                    "Each foot uses its own compiled lag."
                )
        print(
            f"PRED trim_lead on_frac {on_frac:.6f} "
            f"hold_through {TRIM_HOLD_FRAC:.2f} off_by {TRIM_OFF_FRAC:.2f}. "
            "Trim weight is 0 before on_frac. "
            "Lookahead does not start the trim on a loaded foot. "
            "The add is led. The toe-up, the IK ankle, and the hip are not. "
            f"Cap ±{ANK_TRIM_CAP:.3f} rad."
        )
    roll_adds = {"L": float(add_l), "R": float(add_r)}
    pitch_adds = {"L": float(pitch_l), "R": float(pitch_r)}
    walker = lipm.op3
    orig = lipm._tick_gait_manager

    def wrapped(walking: bool) -> None:
        orig(walking)
        if not walking or session.bus.fault:
            return
        swing = lipm._gm_swing
        if swing not in ("L", "R"):
            return
        if lead:
            frac = _mid_frac(walker, swing, _cmd_time(walker))
            if frac is None or frac < on_frac - 1e-6:
                return
            if swing == "L":
                span = float(walker.l_ssp_end) - float(walker.l_ssp_start)
            else:
                span = float(walker.r_ssp_end) - float(walker.r_ssp_start)
            if span <= 1e-6:
                return
            pref = "l_" if swing == "L" else "r_"
            w_roll = _trim_lead_shape(frac + lags[pref + "ank_roll"] / span, on_frac)
            w_pitch = _trim_lead_shape(frac + lags[pref + "ank_pitch"] / span, on_frac)
        else:
            w_roll = 1.0
            w_pitch = 1.0
        delta = roll_adds[swing] * w_roll
        if abs(delta) > 1e-12:
            act = ("l_" if swing == "L" else "r_") + "ank_roll_pos"
            idx = session.act_idx[act]
            session.data.ctrl[idx] = float(session.data.ctrl[idx]) + delta
        pdelta = pitch_adds[swing] * w_pitch
        if abs(pdelta) > 1e-12:
            act = ("l_" if swing == "L" else "r_") + "ank_pitch_pos"
            idx = session.act_idx[act]
            session.data.ctrl[idx] = float(session.data.ctrl[idx]) + pdelta

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
    t_cut: float | None = None,
) -> tuple[float, float, float, str, int, int, int]:
    """Worst 20–80% contact-box toe on the whole bout.

    A swing still open at the end keeps the median full-swing length, so
    an early sample is not relabeled into the window.
    """
    cycles = _swing_cycles(rows, (T_END + 1.0) if t_cut is None else t_cut)
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


def _hx_parts(packed: tuple) -> tuple[float, float, str]:
    tau = float(packed[0])
    when = float(packed[1])
    contact = str(packed[2]) if len(packed) > 2 else ""
    return tau, when, contact


def _surface_summary(
    name: str,
    rows: list[Tick],
    surface_rows: list[dict[str, object]],
    hx_flat: dict[str, tuple[float, float]],
    hx_rug: dict[str, tuple[float, float]],
    ka_hits: list[str] | None = None,
    ka_checked: int = 0,
    t_cut: float | None = None,
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
            f"contact {plane.get('contact', '')} "
            f"surface {plane['surface']}"
        )
    knee_rug = " ".join(rug_bits)
    knee_on = int(any(
        int(knee[side]["on_rug"]) for side in ("L", "R")  # type: ignore[index]
    ))
    print(f"PRED {name} rug_at {kt:.3f} either_on_rug {knee_on} {knee_rug}")
    by_t = {round(float(row["t"]), 5): row for row in surface_rows}
    cycles = _swing_cycles(rows, (T_END + 1.0) if t_cut is None else t_cut)
    med_n, _last_n = _cycle_index(cycles)
    flat_best: tuple[float, float, float, str, float, float, float, float] | None = None
    rug_best: tuple[float, float, float, str, float] | None = None
    mid_samples: list[tuple[float, str, float, float, float, float, float, str]] = []
    n_flat = 0
    n_rug = 0
    flat_fracs: list[float] = []
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
            flat_fracs.append(frac)
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
    n_at = sum(1 for frac in flat_fracs if abs(frac - (5.0 / 24.0)) <= 0.01)
    shown = " ".join(f"{frac:.3f}" for frac in flat_fracs)
    print(
        f"PRED {name} flat_min_frac n {len(flat_fracs)} at_0.208 {n_at}"
        + (f" fracs {shown}" if shown else "")
    )
    flat_hits: list[str] = []
    for act, packed in hx_flat.items():
        tau, when, contact = _hx_parts(packed)
        over = abs(tau) >= KNEE_NM
        if over:
            flat_hits.append(f"{act} {tau:+.4f} t {when:.3f} contact {contact}")
        print(
            f"PRED {name} hx_flat {act} {tau:+.4f} t {when:.3f} "
            f"contact {contact or '-'} "
            f"abs {abs(tau):.4f} ge_2.33 {int(over)} "
            f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
        )
    for act, packed in hx_rug.items():
        tau, when, contact = _hx_parts(packed)
        print(
            f"PRED {name} hx_rug {act} {tau:+.4f} t {when:.3f} "
            f"contact {contact or '-'} "
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
        "flat_min_fracs": tuple(flat_fracs),
    }


def _print_early_flat(
    name: str,
    rows: list[Tick],
    surface_rows: list[dict[str, object]],
) -> None:
    """Toe on every 0–30% tick of each flat swing. The 20–80% window stays."""
    by_t = {round(float(row["t"]), 5): row for row in surface_rows}
    cycles = _swing_cycles(rows, T_END + 1.0)
    med_n, _last_n = _cycle_index(cycles)
    n_lines = 0
    for step, cyc in enumerate(cycles):
        n = len(cyc)
        if n < 2 or med_n < 2:
            continue
        n_full = med_n if step == len(cycles) - 1 and n < med_n - 1 else n
        tagged: list[dict[str, object]] = []
        early: list[tuple[float, Tick, dict[str, object]]] = []
        for j, row in enumerate(cyc):
            if row.toe_z is None or row.swing is None:
                continue
            frac = j / (n_full - 1)
            surf = by_t.get(round(row.t, 5))
            if surf is None:
                continue
            plane = surf[row.swing]
            if not isinstance(plane, dict):
                continue
            if 0.20 - 1e-12 <= frac <= 0.80 + 1e-12:
                tagged.append(plane)
            if frac <= 0.30 + 1e-12:
                early.append((frac, row, plane))
        if not tagged or any(int(plane["on_rug"]) for plane in tagged):
            continue
        for frac, row, plane in early:
            n_lines += 1
            toe_mm = float(row.toe_z) * 1000.0
            fn = float(plane.get("fn", float(plane["floor_n"]) + float(plane["rug_n"])))
            dist = float(plane.get("dist", float("nan")))
            if dist == dist:
                dist_txt = f"{dist * 1000.0:.3f} mm"
                pen_txt = f"{max(0.0, -dist) * 1000.0:.3f} mm"
            else:
                dist_txt = "none"
                pen_txt = "none"
            print(
                f"PRED {name} early_toe swing {step} side {row.swing} "
                f"t {row.t:.3f} frac {frac:.3f} toe {toe_mm:.3f} mm "
                f"vs_plus2 {toe_mm - 2.0:+.3f} mm "
                f"fn {fn:.3f} N dist {dist_txt} pen {pen_txt} "
                f"contact {plane.get('contact', '')}"
            )
    print(
        f"PRED {name} early_toe_n {n_lines} "
        "window 0-30% of flat swings. Scoring window stays 20-80%. "
        "fn is floor plus rug normal on the swing foot. "
        "MuJoCo dist is the contact gap; negative is penetration. "
        "fn near 0 with no penetration is unload. "
        "fn above 0 with negative dist is soft-contact spring-back."
    )


def _print_chain_bar(name: str, surface_rows: list[dict[str, object]]) -> None:
    """Whole-walk knees and the swing ankle pitch against 2.33 Nm."""

    def _best(pick) -> tuple[float, float, str, str] | None:
        best: tuple[float, float, str, str] | None = None
        for row in surface_rows:
            got = pick(row)
            if got is None:
                continue
            tau, contact, act = got
            if best is None or abs(tau) > abs(best[0]):
                best = (tau, float(row["t"]), contact, act)
        return best

    def _knee(key: str):
        def pick(row: dict[str, object]):
            if key not in row:
                return None
            side = "L" if key.startswith("l_") else "R"
            plane = row[side]
            if not isinstance(plane, dict):
                return None
            return float(row[key]), str(plane.get("contact", "")), key
        return pick

    def _swing_ank(row: dict[str, object]):
        swing = row.get("swing")
        if swing not in ("L", "R"):
            return None
        key = "l_ank_pitch" if swing == "L" else "r_ank_pitch"
        if key not in row:
            return None
        plane = row[swing]
        if not isinstance(plane, dict):
            return None
        return float(row[key]), str(plane.get("contact", "")), key

    for key in ("l_knee", "r_knee"):
        hit = _best(_knee(key))
        if hit is None:
            print(f"PRED {name} chain_vs_2.33 {key} missed")
            continue
        tau, when, contact, act = hit
        print(
            f"PRED {name} chain_vs_2.33 {act} {tau:+.4f} t {when:.3f} "
            f"contact {contact or '-'} abs {abs(tau):.4f} "
            f"ge_2.33 {int(abs(tau) >= KNEE_NM)} "
            f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
        )
    hit = _best(_swing_ank)
    if hit is None:
        print(f"PRED {name} chain_vs_2.33 swing_ank_pitch missed")
        return
    tau, when, contact, act = hit
    print(
        f"PRED {name} chain_vs_2.33 swing_ank_pitch {act} {tau:+.4f} "
        f"t {when:.3f} contact {contact or '-'} abs {abs(tau):.4f} "
        f"ge_2.33 {int(abs(tau) >= KNEE_NM)} "
        f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
    )
    for key in ("l_hip_roll", "r_hip_roll", "l_ank_roll", "r_ank_roll"):
        if not any(key in row for row in surface_rows):
            continue
        hit_r = _best(_knee(key))
        if hit_r is None:
            print(f"PRED {name} chain_vs_2.33 {key} missed")
            continue
        tau, when, contact, act = hit_r
        print(
            f"PRED {name} chain_vs_2.33 {act} {tau:+.4f} t {when:.3f} "
            f"contact {contact or '-'} abs {abs(tau):.4f} "
            f"ge_2.33 {int(abs(tau) >= KNEE_NM)} "
            f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
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


def _install_swing_knee_lead(session: sw.SteerSession, lead_s: float) -> None:
    """Write the swing knee from IK a few control ticks ahead.

    Stance knees stay on the gait target. The write uses the predicted
    force limit at 2.33 Nm. This is not a stance-load offset.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        raise RuntimeError("gait manager walker missing")
    if lead_s <= 0.0:
        raise SystemExit("swing knee lead is not positive")
    walker = lipm.op3
    dt = float(session.ctrl_dt)
    ticks = lead_s / dt if dt > 1e-9 else float("nan")
    print(
        f"PRED knee_lead swing only {lead_s * 1000.0:.1f} ms "
        f"({ticks:.2f} ctrl ticks). Stance knees stay on the gait write. "
        "Predicted force is clamped to ±2.33 Nm."
    )
    session._knee_lead_log = []
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
        jn = "l_knee" if swing == "L" else "r_knee"
        fut_joints, _fut_info = _joints_at(
            walker, _wrap_period(t_cmd + lead_s, period),
        )
        session._sole_cmd = saved_cmd
        if fut_joints is None or jn not in fut_joints:
            return
        future = float(fut_joints[jn])
        lipm.write_force_limited(jn, future, KNEE_NM)
        idx = session.act_idx[jn + "_pos"]
        written = float(session.data.ctrl[idx])
        session._knee_lead_log.append((
            float(session.data.time) + float(session.ctrl_dt),
            jn,
            future,
            written,
            int(abs(written - future) > 1e-4),
        ))

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]


def _stance_knees(lipm: object) -> tuple[str, ...]:
    """Knees carrying the body this tick. Double support writes both."""
    swing = getattr(lipm, "_gm_swing", None)
    if swing == "L":
        return ("r_knee",)
    if swing == "R":
        return ("l_knee",)
    return ("l_knee", "r_knee")


def _knee_load(session: sw.SteerSession, jn: str) -> tuple[float, float, float, float, float]:
    """Previous-tick hold offset, compiled kp, bias, constraint, and foot dz.

    ``(qfrc_bias - qfrc_constraint) / kp`` is the position offset that
    holds this pose. It is not the measured actuator force. ``dz`` is
    the pelvis-fixed vertical jacobian of the foot contact geom, metres
    per radian.
    """
    model = session.model
    data = session.data
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
    dof = int(model.jnt_dofadr[jid])
    _q, _qd, idx = _joint_q_qd(session, jn)
    kp = float(model.actuator_gainprm[idx, 0])
    if abs(kp - KNEE_KP) > 1e-3:
        raise SystemExit(f"compiled kp for {jn} is {kp:.4f}, expected {KNEE_KP:.1f}")
    bias = float(data.qfrc_bias[dof])
    con = float(data.qfrc_constraint[dof])
    offset = 0.0 if kp < 1e-6 else (bias - con) / kp
    gid = session.gid_lfoot if jn.startswith("l_") else session.gid_rfoot
    jacp = np.zeros((3, model.nv), dtype=np.float64)
    jacr = np.zeros((3, model.nv), dtype=np.float64)
    mj.mj_jacGeom(model, data, jacp, jacr, gid)
    dz = float(jacp[2, dof])
    return offset, kp, bias, con, dz


def _install_stance_knee_sag(session: sw.SteerSession) -> None:
    """Add the previous tick's stance-knee load offset, then clamp.

    Offset is ``(qfrc_bias - qfrc_constraint) / compiled kp`` from the
    physics step that already finished. Predicted force
    ``kp·(ctrl−q) − kv·q̇`` is then clamped to ±2.33 Nm. Measured
    actuator_force is not an input. The plant file is not edited.
    """
    lipm = session.lipm
    if lipm is None:
        raise RuntimeError("gait manager missing")
    session._sag_cancel = True
    session._sag_log = []
    for jn in ("l_knee", "r_knee"):
        _offset, kp, _bias, _con, _dz = _knee_load(session, jn)
        _q, _qd, idx = _joint_q_qd(session, jn)
        kv = -float(session.model.actuator_biasprm[idx, 2])
        print(f"PRED sag_cancel {jn} kp {kp:.4f} kv {kv:.4f}")
    print(
        "PRED sag_cancel stance knee offset is "
        "(qfrc_bias - qfrc_constraint) / compiled kp from the previous tick. "
        "Predicted force is clamped to ±2.33 Nm. "
        "Measured actuator_force is not the offset."
    )
    orig = lipm._tick_gait_manager

    def wrapped(walking: bool) -> None:
        loads = {
            jn: _knee_load(session, jn) for jn in ("l_knee", "r_knee")
        }
        orig(walking)
        if not walking or session.bus.fault:
            return
        t_mark = float(session.data.time) + float(session.ctrl_dt)
        for jn in _stance_knees(lipm):
            raw, kp, bias, con, dz = loads[jn]
            _q, _qd, idx = _joint_q_qd(session, jn)
            before = float(session.data.ctrl[idx])
            session.data.ctrl[idx] = before + raw
            pred = _clamp_ctrl_predicted(session, jn, KNEE_NM)
            applied = float(session.data.ctrl[idx]) - before
            session._sag_log.append((
                t_mark, jn, raw, applied, dz,
                applied * dz * 1000.0, raw * dz * 1000.0,
                float(pred), bias, con, kp,
                int(abs(applied - raw) > 1e-6),
            ))

    lipm._tick_gait_manager = wrapped  # type: ignore[method-assign]


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
    sag_rows: tuple = ()
    flat_min_fracs: tuple = ()
    roll_hx_over: str = ""
    sole_ticks: tuple = ()
    ank_hx_over: str = ""
    floor_chain: tuple = ()


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


def _install_unclamp_log(lipm: object, bucket: list[tuple]) -> None:
    """Record kp*(q_des-q) - kv*omega before the yaw budget or the stop clamp."""
    orig_fl = lipm.write_force_limited
    orig_wc = lipm.write_clipped

    def _note(jn: str, q_des: float, kind: str, limit: float | None) -> None:
        if not jn.endswith((
            "hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll",
        )):
            return
        act = jn + "_pos"
        idx = lipm.act_idx.get(act)
        if idx is None:
            return
        q = float(lipm.q(jn))
        jid = mj.mj_name2id(lipm.model, mj.mjtObj.mjOBJ_JOINT, jn)
        omega = float(lipm.data.qvel[int(lipm.model.jnt_dofadr[jid])])
        kp = float(lipm.model.actuator_gainprm[idx, 0])
        kv = -float(lipm.model.actuator_biasprm[idx, 2])
        kp_term = kp * (float(q_des) - q)
        kv_term = -kv * omega
        ask = kp_term + kv_term
        bucket.append((
            float(lipm.data.time), jn, float(q_des), float(ask),
            float(lipm.cmd_yaw), str(lipm.phase), kind,
            None if limit is None else float(limit),
            float(kp_term), float(kv_term), float(omega), float(kp), float(kv),
        ))

    def wrapped_fl(jn: str, q_des: float, limit_nm: float | None = None) -> None:
        _note(jn, q_des, "force_limited", limit_nm)
        orig_fl(jn, q_des, limit_nm)

    def wrapped_wc(jn: str, q_des: float) -> None:
        _note(jn, q_des, "clipped", None)
        orig_wc(jn, q_des)

    lipm.write_force_limited = wrapped_fl  # type: ignore[method-assign]
    lipm.write_clipped = wrapped_wc  # type: ignore[method-assign]


def _movement_boundary(walker: object) -> bool:
    """True on the OP3 ticks that call ``update_movement``.

    Those ticks are gait time 0, phase1, and phase3. phase2 updates the
    clock only. A command written on this tick is the one the step latches.
    """
    t = float(walker.time)
    half = float(ow.OP3_CTRL_S) / 2.0
    if abs(t) <= 1e-12:
        return True
    if abs(t - float(walker.phase1)) <= half + 1e-12:
        return True
    if abs(t - float(walker.phase3)) <= half + 1e-12:
        return True
    return False


def _ramp_goal(label: str, yaw_cap: float) -> tuple[float, float] | None:
    """Gait (vx, yaw) after a boundary ramp. None keeps the bus command."""
    cap = sw.VX_FWD_CAP
    yaw = float(yaw_cap)
    if label in ("stand", "forward"):
        return None
    if label in ("stop", "settle"):
        return (0.0, 0.0)
    if label in ("yaw_left_on", "yaw_left"):
        return (cap, yaw)
    if label in ("yaw_right_on", "yaw_right"):
        return (cap, -yaw)
    raise SystemExit(f"ramp label {label} has no gait goal")


def _install_command_ramp(
    lipm: object,
    driver: sw.ScriptedDriver,
    latches: list[tuple],
    yaw_cap: float,
) -> None:
    """Latch vx and yaw at OP3 movement boundaries. Do not write the stand pose.

    The bus keeps a non-zero vel so the 20 ms approach stays on. The gait
    sees the latched command, not that bus slew. A new goal is anchored at
    the first boundary, then spread over two periods. Settle keeps the
    zero step. It does not call hold_stand.
    """
    period = float(lipm.cfg.gm_period_s)
    ramp_s = 2.0 * period
    ramp_labels = {"stop", "yaw_left_on", "yaw_right_on"}
    orig = lipm.tick
    state = {
        "label": "",
        "vx": 0.0,
        "yaw": 0.0,
        "vx0": 0.0,
        "yaw0": 0.0,
        "armed": False,
        "t0": 0.0,
    }
    print(
        "PRED steer_ramp schedule "
        f"period {period:.3f} s ramp {ramp_s:.3f} s "
        f"phase1 {float(lipm.op3.phase1):.3f} phase3 {float(lipm.op3.phase3):.3f}. "
        "Step length and yaw change at those boundaries. "
        "Settle stays on the zero step. hold_stand stays off."
    )

    def wrapped(vx: float, yaw_rate: float, walking: bool) -> None:
        t = float(lipm.data.time)
        label = driver.segment(t).label
        goal = _ramp_goal(label, yaw_cap)
        if goal is None:
            state["label"] = label
            state["vx"] = float(vx)
            state["yaw"] = float(yaw_rate)
            state["armed"] = False
            orig(vx, yaw_rate, walking)
            return
        if state["label"] != label:
            state["label"] = label
            state["vx0"] = float(state["vx"])
            state["yaw0"] = float(state["yaw"])
            state["armed"] = False
            state["t0"] = t
        if _movement_boundary(lipm.op3):
            if not state["armed"]:
                state["armed"] = True
                state["t0"] = t
                frac = 0.0
            elif label in ramp_labels:
                frac = min(1.0, (t - float(state["t0"])) / ramp_s)
            else:
                frac = 1.0
            gvx, gyaw = goal
            state["vx"] = float(state["vx0"]) + (gvx - float(state["vx0"])) * frac
            state["yaw"] = float(state["yaw0"]) + (gyaw - float(state["yaw0"])) * frac
            latches.append((
                t, float(lipm.op3.time), label, float(state["vx"]),
                float(state["yaw"]), float(frac),
            ))
            print(
                f"PRED steer_ramp latch t {t:.3f} gait_t {float(lipm.op3.time):.3f} "
                f"seg {label} vx {float(state['vx']):+.4f} "
                f"yaw {float(state['yaw']):+.4f} frac {frac:.3f}"
            )
        lipm.yaw_target = float(state["yaw"])
        orig(float(state["vx"]), float(state["yaw"]), True)

    lipm.tick = wrapped  # type: ignore[method-assign]


# Inside 4–6 periods. Five periods of 0.500 s is 2.500 s.
SOFT_STOP_PERIODS = 5.0
# Body speed under this, held this long, is vel ≈ 0. Same gates as the toe-gap settle.
SOFT_STOP_SPEED_EPS = 0.02
SOFT_STOP_HOLD_S = 0.20


def _gait_phase_name(walker: object) -> str:
    """``L`` / ``R`` in single support, ``D`` in either double support."""
    t = float(walker.time)
    if float(walker.l_ssp_start) < t <= float(walker.l_ssp_end):
        return "L"
    if float(walker.r_ssp_start) < t <= float(walker.r_ssp_end):
        return "R"
    return "D"


def _sole_load_n(session: sw.SteerSession, side: str) -> float:
    """Floor normal plus rug normal. A foot on the rug still counts as loaded."""
    gid = session.gid_lfoot if side == "L" else session.gid_rfoot
    rug = int(session.gid_rug)
    load = _pair_fn(session.model, session.data, gid, session.gid_floor)
    if rug >= 0:
        load += _pair_fn(session.model, session.data, gid, rug)
    return load


def _print_vendor_stop() -> None:
    """What the published clocks do when the walk ends. No decay is in that set."""
    print(
        "PRED vendor_stop OP3 Op3Walker.stop sets ctrl_running false, "
        "zeros x_cmd y_cmd and angle_cmd, sets time to 0, sets previous_x to 0, "
        "and calls update_movement on that same call. "
        "Time 0 is the double support before the left swing, not a wait for "
        "the next double support. Amplitudes go to 0 on that call. "
        "There is no step-length decay and no both-feet load check."
    )
    print(
        "PRED vendor_stop gait_manager_traj.GaitManagerClock has no stop. "
        "set_command stores x, y, and angle. _phase_update applies them at "
        "time 0, phase1, and phase3, including a single-support boundary. "
        "The forward demo is set_step rot 0 with a constant x. "
        "No dsp-wait and no multi-period decay are in that copy. "
        "The live not-walking path calls walker.stop then hold_stand, "
        "which writes the stand pose on that tick."
    )


def _write_frozen_dsp(
    session: sw.SteerSession,
    x_amp: float,
    gait_t: float,
    x_prev: float,
) -> dict[str, float]:
    """DSP pose at a frozen clock. Step length is ``x_amp``. Toe-up stays off.

    Level-trim roll and pitch are added on both ankles. The landing
    toe-down schedule is the swing toe-up returning to 0, and this path
    does not run it.
    """
    lipm = session.lipm
    walker = lipm.op3
    walker.time = float(gait_t)
    walker.x_cmd = float(x_amp)
    walker.y_cmd = 0.0
    walker.angle_cmd = 0.0
    walker.previous_x = float(x_prev) if abs(float(x_prev)) > 1e-9 else 1.0
    walker.ctrl_running = True
    walker.update_movement()
    joints, info = walker.joints_now()
    lipm.cmd_vx = 0.0
    lipm.cmd_yaw = 0.0
    lipm.yaw_target = 0.0
    lipm.phase = "shift"
    lipm.phase_t = 0.0
    lipm._gm_swing = None
    out = {
        "x_move": float(walker._x_move),
        "ik": 0.0 if joints is None else 1.0,
        "phase_d": 1.0 if info.phase == "D" else 0.0,
    }
    if joints is None:
        return out
    for jn in (
        "l_sho_roll", "r_sho_roll", "l_el_pitch", "r_el_pitch",
        "l_el_yaw", "r_el_yaw", "l_gripper", "r_gripper",
    ):
        lipm.write_clipped(jn, lipm.q_stand.get(jn, 0.0))
    for name, val in joints.items():
        if name.endswith("hip_roll") and abs(lipm.cmd_yaw) > 1e-3:
            lipm.write_force_limited(name, float(val), KNEE_NM)
        else:
            lipm.write_clipped(name, float(val))
    for act, delta in (
        ("l_ank_roll_pos", _RESTORED_ROLL_L),
        ("r_ank_roll_pos", _RESTORED_ROLL_R),
        ("l_ank_pitch_pos", _RESTORED_PITCH_L),
        ("r_ank_pitch_pos", _RESTORED_PITCH_R),
    ):
        idx = session.act_idx[act]
        session.data.ctrl[idx] = float(session.data.ctrl[idx]) + float(delta)
    for side, hip_name, knee_name in (
        ("l", "l_hip_pitch", "l_knee"),
        ("r", "r_hip_pitch", "r_knee"),
    ):
        if hip_name in joints and knee_name in joints:
            out[f"{side}_knee_des"] = float(joints[knee_name])
            out[f"{side}_knee_q"] = float(lipm.q(knee_name))
    lipm._write_unused()
    return out


def _soft_snapshot(
    session: sw.SteerSession,
    mode: str,
    x_move: float,
    x_cmd: float,
    knees: dict[str, float],
) -> dict[str, object]:
    lipm = session.lipm
    walker = lipm.op3
    feet: dict[str, object] = {}
    for side in ("L", "R"):
        corners = _box_corner_table(session, side)
        centre = float(sum(z for _label, z, _off in corners) / len(corners))
        feet[side] = {
            "load": _sole_load_n(session, side),
            "centre": centre,
            "roll": _sole_roll(session, side)[0],
            "pitch": _sole_pitch(session, side),
            "corners": [(label, z) for label, z, _off in corners],
        }
    ssp = None
    gt = float(walker.time)
    for start_name, end_name in (
        ("l_ssp_start", "l_ssp_end"),
        ("r_ssp_start", "r_ssp_end"),
    ):
        a = float(getattr(walker, start_name))
        b = float(getattr(walker, end_name))
        if a < gt <= b and b > a:
            ssp = (gt - a) / (b - a)
            break
    return {
        "t": float(session.data.time),
        "mode": mode,
        "gait_t": gt,
        "phase": str(lipm.phase),
        "clock": _gait_phase_name(walker),
        "ssp_frac": ssp,
        "x_move": float(x_move),
        "x_cmd": float(x_cmd),
        "body_vx": float(session._body_forward_speed()),
        "knees": knees,
        "feet": feet,
        "yaw": float(lipm.cmd_yaw),
    }


def _install_soft_stop(
    session: sw.SteerSession,
    driver: sw.ScriptedDriver,
    trace: list[dict[str, object]],
) -> None:
    """Finish the stride, freeze in double support, then decay the step.

    Yaw stays 0. hold_stand is not called. The vendor stop is not this
    path: that one zeros the clock on the stop call.
    """
    lipm = session.lipm
    walker = lipm.op3
    period = float(lipm.cfg.gm_period_s)
    decay_s = SOFT_STOP_PERIODS * period
    load_n = float(lipm.cfg.unload_n)
    orig = lipm.tick
    state = {
        "stop": False,
        "frozen": False,
        "gait_t": 0.0,
        "x_full": 0.0,
        "x_prev": 0.0,
        "anchor": None,
        "walk_vx": sw.VX_FWD_CAP,
    }
    _print_vendor_stop()
    print(
        "PRED soft_stop schedule "
        f"period {period:.3f} s decay {SOFT_STOP_PERIODS:.0f} periods "
        f"({decay_s:.3f} s) load_n {load_n:.1f} N floor+rug. "
        "Finish the stride to the next double support. "
        "Freeze the clock there. Decay step length to 0 linearly. "
        "Level trim stays on both ankles. The landing toe-down schedule stays off "
        "while the step length is falling. Yaw stays 0. hold_stand stays off."
    )

    def wrapped(vx: float, yaw_rate: float, walking: bool) -> None:
        del yaw_rate
        t = float(lipm.data.time)
        label = driver.segment(t).label
        if not state["stop"]:
            if label != "stop":
                state["walk_vx"] = float(vx)
                orig(float(vx), 0.0, walking)
                return
            state["stop"] = True
            state["x_full"] = float(walker.x_cmd)
            state["x_prev"] = float(walker.previous_x)
            print(
                f"PRED soft_stop command t {t:.3f} gait_t {float(walker.time):.3f} "
                f"phase {_gait_phase_name(walker)} "
                f"x_cmd {float(walker.x_cmd):+.5f} "
                f"x_move {float(walker._x_move):+.5f} yaw 0"
            )
        if not state["frozen"]:
            if _gait_phase_name(walker) != "D":
                orig(float(state["walk_vx"]), 0.0, True)

                def _knee_ctrl(jn: str) -> float:
                    idx = lipm.act_idx.get(f"{jn}_pos")
                    if idx is None:
                        return float("nan")
                    return float(lipm.data.ctrl[idx])

                knees = {
                    "l_des": _knee_ctrl("l_knee"),
                    "l_q": float(lipm.q("l_knee")),
                    "r_des": _knee_ctrl("r_knee"),
                    "r_q": float(lipm.q("r_knee")),
                }
                trace.append(_soft_snapshot(
                    session, "finish", float(walker._x_move), float(walker.x_cmd), knees,
                ))
                return
            state["frozen"] = True
            state["gait_t"] = float(walker.time)
            if abs(float(walker.x_cmd)) > 1e-9:
                state["x_full"] = float(walker.x_cmd)
            if abs(float(walker.previous_x)) > 1e-9:
                state["x_prev"] = float(walker.previous_x)
            print(
                f"PRED soft_stop freeze t {t:.3f} gait_t {state['gait_t']:.3f} "
                f"x_full {float(state['x_full']):+.5f} "
                f"load L {_sole_load_n(session, 'L'):.2f} "
                f"R {_sole_load_n(session, 'R'):.2f}"
            )
        load_l = _sole_load_n(session, "L")
        load_r = _sole_load_n(session, "R")
        both = load_l > load_n and load_r > load_n
        if state["anchor"] is None:
            if not both:
                mode = "wait"
                x_amp = float(state["x_full"])
            else:
                state["anchor"] = t
                mode = "decay"
                x_amp = float(state["x_full"])
                print(
                    f"PRED soft_stop decay_start t {t:.3f} "
                    f"gait_t {float(state['gait_t']):.3f} "
                    f"load L {load_l:.2f} R {load_r:.2f} "
                    f"x {x_amp:+.5f} over {decay_s:.3f} s"
                )
        else:
            frac = min(1.0, (t - float(state["anchor"])) / decay_s)
            x_amp = float(state["x_full"]) * (1.0 - frac)
            mode = "hold" if frac >= 1.0 - 1e-9 else "decay"
        written = _write_frozen_dsp(
            session, x_amp, float(state["gait_t"]), float(state["x_prev"]),
        )
        knees = {
            "l_des": float(written.get("l_knee_des", float("nan"))),
            "l_q": float(written.get("l_knee_q", float("nan"))),
            "r_des": float(written.get("r_des", written.get("r_knee_des", float("nan")))),
            "r_q": float(written.get("r_knee_q", float("nan"))),
        }
        # The key above used a typo guard. Read the names the writer stores.
        knees["r_des"] = float(written.get("r_knee_des", float("nan")))
        snap = _soft_snapshot(
            session, mode, float(written["x_move"]), float(x_amp), knees,
        )
        snap["phase_d"] = float(written["phase_d"])
        snap["load_l"] = load_l
        snap["load_r"] = load_r
        trace.append(snap)

    lipm.tick = wrapped  # type: ignore[method-assign]


def _clock_at(walker: object, gait_t: float) -> str:
    t = float(gait_t)
    if float(walker.l_ssp_start) < t <= float(walker.l_ssp_end):
        return "L"
    if float(walker.r_ssp_start) < t <= float(walker.r_ssp_end):
        return "R"
    return "D"


def _ssp_frac_at(walker: object, gait_t: float) -> float | None:
    t = float(gait_t)
    for start_name, end_name in (
        ("l_ssp_start", "l_ssp_end"),
        ("r_ssp_start", "r_ssp_end"),
    ):
        a = float(getattr(walker, start_name))
        b = float(getattr(walker, end_name))
        if a < t <= b and b > a:
            return (t - a) / (b - a)
    return None


_STANCE_SUFFIX = (
    "hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll",
)
_FULL_GOAL_SUFFIX = ("hip_pitch", "knee", "ank_pitch")


def _install_inflight_stop(
    session: sw.SteerSession,
    driver: sw.ScriptedDriver,
    trace: list[dict[str, object]],
    edge: dict[str, object],
    *,
    next_stride_decay: bool = False,
    stance_slew: bool = False,
    sag_stop_cap: bool = False,
) -> None:
    """Freeze the loaded stance chain. Only the airborne swing-z schedule moves.

    The stance leg's six pre-stop targets stay put, and so do the stance
    foot and the pelvis rolls. The airborne foot keeps its pre-stop x/y
    and sole roll/pitch. Its z follows the live schedule until both feet
    are over 5 N. Step length is not zeroed on the stop tick. The snap
    zeros the amplitude on the first real double-support tick.
    """
    lipm = session.lipm
    walker = lipm.op3
    load_n = float(lipm.cfg.unload_n)
    # The parked stop was scored with the stance slew remembering only
    # stance writes. A pure walk remembers the swing command too.
    session._sag_stop_installed = True  # type: ignore[attr-defined]
    orig_tick = lipm.tick
    orig_endpoints = walker.endpoints
    state: dict[str, object] = {
        "stop": False,
        "pin_swing": False,
        "pin_stance": False,
        "pin_chain": False,
        "pin_z": False,
        "snapped": False,
        "swing": "",
        "stance_sides": [],
        "swing_pin": (0.0, 0.0, 0.0, 0.0, 0.0),
        "stance_xy": (0.0, 0.0),
        "stance_q": {},
        "stance_ep": {},
        "pel": (0.0, 0.0),
        "swap_y": 0.0,
        "z_pin": {"L": 0.0, "R": 0.0},
        "pre": None,
        "pre_row": None,
        "pose_gait": 0.0,
        "edge_done": False,
        "held_time": False,
        "toe_hold": 0.0,
        "toe_sign": {},
        "ik_knee": float("nan"),
        "pin_knee": float("nan"),
        "slew": bool(stance_slew),
        "slew_started": False,
        "slew_t0": float("nan"),
        "slew_from": {},
        "slew_s": 1.0,
        "sag_stop_cap": bool(sag_stop_cap),
        "saw_air": False,
        "told_hold": False,
        "z_stretch_on": False,
        "z_stretch": 1.0,
        "z_stretch_end": 0.0,
        "z_stretch_hold": False,
        "z_wait_n": 0,
        "touch_logged": False,
    }
    print(
        "PRED inflight_stop schedule "
        f"load_n {load_n:.1f} N floor+rug. "
        "Yaw stays 0. On the stop tick the loaded stance leg's six "
        "pre-stop targets stay put, including level trim already in them. "
        "The stance foot and the pelvis rolls stay at that pre-stop pose. "
        "The airborne foot keeps its pre-stop x/y and roll/pitch. "
        "Only that foot's swing-z schedule advances until both feet are "
        "over the load gate. Step length is not zeroed on the stop tick. "
        "At the first real double support the amplitude snaps to 0 and "
        "the stance planar target stays put. Time is not parked at 0."
    )
    if stance_slew:
        print(
            "PRED inflight_slew once both feet are over the load gate, "
            "pinned stance q_des slews toward the stand pose over 1.000 s "
            "(2 periods). A 1-period rate from the full gap stays over "
            "2.33 Nm, so each tick also keeps |kp*(q_des-q)| + |kv*omega| "
            "at or under 2.33 Nm. The airborne window before that double "
            "support still holds the pin."
        )
    if next_stride_decay:
        print(
            "PRED inflight_fallback next-stride decay was requested and "
            "is not used. Zeroing the next stride made the knee ask worse. "
            "x_move stays at the walking value until the double-support snap."
        )
    edge["fallback"] = 0

    def _ctrl(jn: str) -> float:
        idx = lipm.act_idx.get(f"{jn}_pos")
        if idx is None:
            return float("nan")
        return float(lipm.data.ctrl[idx])

    def _knees_now() -> dict[str, float]:
        return {
            "l_des": _ctrl("l_knee"),
            "l_q": float(lipm.q("l_knee")),
            "r_des": _ctrl("r_knee"),
            "r_q": float(lipm.q("r_knee")),
        }

    def _side_des(side: str, joints: dict[str, float]) -> dict[str, float]:
        """Pre-stop q_des. Full-goal joints use ctrl, which already has trim."""
        pref = "l_" if side == "L" else "r_"
        out: dict[str, float] = {}
        for suf in _STANCE_SUFFIX:
            jn = pref + suf
            if suf in _FULL_GOAL_SUFFIX:
                out[suf] = _ctrl(jn)
            else:
                out[suf] = float(joints[jn])
        return out

    def _stamp(snap: dict[str, object], mode_freeze: str) -> dict[str, object]:
        gait_t = float(state["pose_gait"])
        snap["gait_t"] = gait_t
        snap["clock"] = _clock_at(walker, gait_t)
        snap["ssp_frac"] = _ssp_frac_at(walker, gait_t)
        snap["freeze"] = mode_freeze
        return snap

    def _pose_clock() -> float:
        """Gait time the IK just used. ``step`` has already advanced the clock."""
        pose = float(walker.time) - float(ow.OP3_CTRL_S)
        if pose < -1e-12:
            pose += float(walker.period)
        return pose

    def _swing_toe(swing: str, gait_t: float) -> float:
        """Toe-up add at ``gait_t``. Zero outside that foot's single support."""
        spec = getattr(session, "_inflight_toe", None)
        if not isinstance(spec, tuple) or swing not in ("L", "R"):
            return 0.0
        peak, shape, full = float(spec[0]), str(spec[1]), float(spec[2])
        if peak <= 0.0:
            return 0.0
        frac = _ssp_frac_at(walker, gait_t)
        if frac is None or frac <= 0.0 or frac >= 1.0:
            return 0.0
        signs = state["toe_sign"]
        assert isinstance(signs, dict)
        if swing not in signs:
            signs[swing] = float(_ank_pitch_raises_front(session, swing))
        ramp = _toe_up_ramp(float(frac), shape, full)
        return float(signs[swing]) * peak * float(ramp)

    def _add_pitch(swing: str, delta: float) -> None:
        if swing not in ("L", "R") or abs(float(delta)) < 1e-12:
            return
        act = ("l_" if swing == "L" else "r_") + "ank_pitch_pos"
        idx = session.act_idx[act]
        session.data.ctrl[idx] = float(session.data.ctrl[idx]) + float(delta)

    def _add_swing_trim(swing: str) -> None:
        """Constant level trim on the swing ankle. Trim-lead stays off."""
        if swing not in ("L", "R"):
            return
        roll = _RESTORED_ROLL_L if swing == "L" else _RESTORED_ROLL_R
        pitch = _RESTORED_PITCH_L if swing == "L" else _RESTORED_PITCH_R
        pref = "l_" if swing == "L" else "r_"
        for act, delta in (
            (f"{pref}ank_roll_pos", roll),
            (f"{pref}ank_pitch_pos", pitch),
        ):
            if abs(float(delta)) < 1e-12:
                continue
            idx = session.act_idx[act]
            session.data.ctrl[idx] = float(session.data.ctrl[idx]) + float(delta)

    def _hold_toe() -> None:
        """Replace the live toe-up with the pre-stop toe-up. IK ankles stay."""
        swing = str(state["swing"])
        if swing not in ("L", "R") or not state["pin_swing"]:
            return
        now = _swing_toe(swing, float(state["pose_gait"]))
        _add_pitch(swing, float(state["toe_hold"]) - now)

    def _remember_pre(t: float) -> None:
        gait_t = _pose_clock()
        saved = float(walker.time)
        walker.time = gait_t
        try:
            er, el, pel_r, pel_l, swap_y = orig_endpoints()
            ik_joints, _ik_info = walker.joints_now()
        finally:
            walker.time = saved
        er = np.array(er, dtype=np.float64, copy=True)
        el = np.array(el, dtype=np.float64, copy=True)
        knees = _knees_now()
        des = {"L": {}, "R": {}}
        if ik_joints is not None:
            des = {
                "L": _side_des("L", ik_joints),
                "R": _side_des("R", ik_joints),
            }
        state["pre"] = {
            "t": float(t),
            "gait": float(gait_t),
            "er": er,
            "el": el,
            "knees": knees,
            "des": des,
            "pel_r": float(pel_r),
            "pel_l": float(pel_l),
            "swap_y": float(swap_y),
            "x_cmd": float(walker.x_cmd),
            "x_move": float(walker._x_move),
        }
        row = _soft_snapshot(
            session, "pre", float(walker._x_move), float(walker.x_cmd), knees,
        )
        row["t"] = float(t)
        row["gait_t"] = float(gait_t)
        row["clock"] = _clock_at(walker, gait_t)
        row["ssp_frac"] = _ssp_frac_at(walker, gait_t)
        row["freeze"] = "none"
        row["des"] = des
        row["pel_r"] = float(pel_r)
        row["pel_l"] = float(pel_l)
        row["pelvis_frozen"] = 0
        row["ik_stance_knee"] = float("nan")
        row["pin_knee"] = float("nan")
        state["pre_row"] = row

    def _write_pinned() -> None:
        lipm.cmd_vx = 0.0
        lipm.cmd_yaw = 0.0
        lipm.yaw_target = 0.0
        lipm.phase = "shift"
        lipm.phase_t = 0.0
        lipm._gm_swing = None
        for jn in (
            "l_sho_roll", "r_sho_roll", "l_el_pitch", "r_el_pitch",
            "l_el_yaw", "r_el_yaw", "l_gripper", "r_gripper",
        ):
            lipm.write_clipped(jn, lipm.q_stand.get(jn, 0.0))
        joints, _info = walker.joints_now()
        if joints is None:
            return
        for name, val in joints.items():
            lipm.write_clipped(name, float(val))
        swing = str(state["swing"])
        _add_swing_trim(swing)
        _add_pitch(swing, float(state["toe_hold"]))
        lipm._write_unused()

    def _past_this_swing(swing: str) -> bool:
        t = float(walker.time)
        if swing == "L":
            return t >= float(walker.r_ssp_start) - 1e-9
        return t <= float(walker.l_ssp_start) + 1e-9

    def wrapped_endpoints():
        er, el, pel_r, pel_l, swap_y = orig_endpoints()
        state["last_raw"] = (er.copy(), el.copy(), float(walker.time), float(lipm.data.time))
        state["pose_gait"] = float(walker.time)
        state["live_z"] = {"L": float(el[2]), "R": float(er[2])}
        def _hold_swing_xy(ep: np.ndarray, side: str) -> None:
            """Hold x/y and the sole angles. Sagittal stop leaves z live.

            Knee and ankle pitch are the IK of that pose, not a frozen
            q_des, so the foot can still reach the floor. Hip roll, hip
            yaw, hip pitch, and ankle roll are pinned on the joint write.
            """
            x, y, roll, pitch, yaw = state["swing_pin"]
            ep[0] = float(x)
            ep[1] = float(y)
            if not state.get("xy_only"):
                ep[3] = float(roll)
                ep[4] = float(pitch)
                ep[5] = float(yaw)
            if state["pin_z"]:
                z_pin = state["z_pin"]
                assert isinstance(z_pin, dict)
                ep[2] = float(z_pin[side])
            elif state.get("z_stretch_on") and side == str(state.get("swing", "")):
                # The live write uses the softened schedule. The catch
                # check reads the same z, so the clock cannot run a
                # different descent than the knee is tracking.
                z_cmd = _soft_z_at(float(walker.time))
                if z_cmd is not None:
                    ep[2] = float(z_cmd)

        if state["pin_chain"]:
            swing = str(state["swing"])
            stance_ep = state["stance_ep"]
            assert isinstance(stance_ep, dict)
            for side, ep in (("L", el), ("R", er)):
                held = stance_ep.get(side)
                if isinstance(held, np.ndarray):
                    ep[:] = held
                    continue
                if state["pin_swing"] and side == swing:
                    _hold_swing_xy(ep, side)
            pel_r, pel_l = state["pel"]
            swap_y = float(state["swap_y"])
            return er, el, float(pel_r), float(pel_l), swap_y
        if state["pin_swing"]:
            swing = str(state["swing"])
            ep = el if swing == "L" else er
            _hold_swing_xy(ep, swing)
        if state["pin_stance"]:
            swing = str(state["swing"])
            side = "R" if swing == "L" else "L"
            ep = er if side == "R" else el
            sx, sy = state["stance_xy"]
            ep[0] = float(sx)
            ep[1] = float(sy)
            if state["pin_z"]:
                z_pin = state["z_pin"]
                assert isinstance(z_pin, dict)
                ep[2] = float(z_pin[side])
        return er, el, pel_r, pel_l, swap_y

    walker.endpoints = wrapped_endpoints  # type: ignore[method-assign]

    def _ep_of(side: str, raw: tuple) -> object:
        er, el = raw[0], raw[1]
        return el if side == "L" else er

    def _arm_chain(pre: object, swing: str) -> None:
        """Pin the loaded leg and the pelvis at the pre-stop totals."""
        if not isinstance(pre, dict):
            return
        des = pre.get("des")
        if not isinstance(des, dict) or "L" not in des or "R" not in des:
            return
        if swing == "L":
            sides = ["R"]
        elif swing == "R":
            sides = ["L"]
        else:
            sides = ["L", "R"]
        qs: dict[str, dict[str, float]] = {}
        eps: dict[str, np.ndarray] = {}
        for side in sides:
            side_des = des[side]
            if not isinstance(side_des, dict):
                continue
            qs[side] = {suf: float(side_des[suf]) for suf in _STANCE_SUFFIX}
            src = pre["el"] if side == "L" else pre["er"]
            eps[side] = np.array(src, dtype=np.float64, copy=True)
        state["stance_sides"] = sides
        state["stance_q"] = qs
        state["stance_ep"] = eps
        state["pel"] = (float(pre["pel_r"]), float(pre["pel_l"]))
        state["swap_y"] = float(pre["swap_y"])
        state["pin_chain"] = True
        state["pin_stance"] = True
        bits = []
        for side in sides:
            q = qs[side]
            bits.append(
                f"{side} knee {q['knee']:+.5f} hip_roll {q['hip_roll']:+.5f} "
                f"hip_pitch {q['hip_pitch']:+.5f} hip_yaw {q['hip_yaw']:+.5f} "
                f"ank_pitch {q['ank_pitch']:+.5f} ank_roll {q['ank_roll']:+.5f}"
            )
        print(
            "PRED inflight_stance pin "
            + " | ".join(bits)
            + f" pel_r {float(pre['pel_r']):+.5f} pel_l {float(pre['pel_l']):+.5f} "
            "pelvis/CoM planar frozen. x_move is not zeroed on this tick."
        )

    def _arm_stand_sides(load_l: float, load_r: float) -> None:
        """Stand slew only after real double support, and only on a loaded leg."""
        if not state["sag_stop_cap"]:
            return
        session._sag_stand_armed = True  # type: ignore[attr-defined]
        sides: list[str] = []
        if load_l > load_n:
            sides.append("L")
        if load_r > load_n:
            sides.append("R")
        session._sag_stand_sides = tuple(sides)  # type: ignore[attr-defined]

    def _swing_leg_names(swing: str) -> tuple[str, ...]:
        if swing == "L":
            return (
                "l_hip_yaw", "l_hip_roll", "l_hip_pitch",
                "l_knee", "l_ank_pitch", "l_ank_roll",
            )
        return (
            "r_hip_yaw", "r_hip_roll", "r_hip_pitch",
            "r_knee", "r_ank_pitch", "r_ank_roll",
        )

    def _soft_z_at(gait_t: float) -> float | None:
        sched = state.get("z_soft_sched")
        if not isinstance(sched, list) or not sched:
            return None
        if float(gait_t) <= float(sched[0][0]):
            return float(sched[0][1])
        if float(gait_t) >= float(sched[-1][0]):
            return float(sched[-1][1])
        for a, b in zip(sched, sched[1:]):
            if float(a[0]) <= float(gait_t) <= float(b[0]):
                span = float(b[0]) - float(a[0])
                u = 0.0 if span < 1e-9 else (float(gait_t) - float(a[0])) / span
                return float(a[1]) + u * (float(b[1]) - float(a[1]))
        return float(sched[-1][1])

    def _swing_ik_at(
        swing: str, gait_t: float, z_override: float | None = None,
    ) -> tuple[float, float, float] | None:
        """Knee, ankle pitch, and endpoint z of the frozen swing pose at ``gait_t``.

        Planar x/y and the sole angles stay on the pre-stop pin. z is the
        live schedule at that gait time, including the phase bump.
        """
        if swing not in ("L", "R"):
            return None
        pin = state.get("swing_pin")
        if not isinstance(pin, tuple) or len(pin) < 5:
            return None
        saved = float(walker.time)
        try:
            walker.time = float(gait_t)
            er, el, _pel_r, _pel_l, _swap_y = orig_endpoints()
        finally:
            walker.time = saved
        ep = np.array(el if swing == "L" else er, dtype=np.float64, copy=True)
        ep[0] = float(pin[0])
        ep[1] = float(pin[1])
        ep[3] = float(pin[2])
        ep[4] = float(pin[3])
        ep[5] = float(pin[4])
        if z_override is not None:
            ep[2] = float(z_override)
        elif state.get("z_soft_sched") is not None:
            z_cmd = _soft_z_at(float(gait_t))
            if z_cmd is not None:
                ep[2] = float(z_cmd)
        raw = ow.ik_leg(
            walker.lengths,
            float(ep[0]), float(ep[1]), float(ep[2]),
            float(ep[3]), float(ep[4]), float(ep[5]),
        )
        if raw is None:
            return None
        signed = walker._apply_direction(raw, _swing_leg_names(swing))
        return float(signed[3]), float(signed[4]), float(ep[2])

    def _arm_z_stretch(swing: str, t0: float) -> None:
        """Stretch the leftover swing-z so knee and ankle pitch can follow at 1.90 rad/s.

        The clock then advances only while those two commands are already
        within one step of the IK at the current time. z does not run ahead
        of the slew.
        """
        if swing not in ("L", "R"):
            return
        end = float(walker.l_ssp_end if swing == "L" else walker.r_ssp_end)
        dt = float(ow.OP3_CTRL_S)
        rows: list[tuple[float, float, float, float]] = []
        t = float(t0)
        guard = 0
        while t <= end + 1e-9 and guard < 500:
            sample = _swing_ik_at(swing, t)
            if sample is not None:
                rows.append((t, sample[0], sample[1], sample[2]))
            if t >= end - 1e-9:
                break
            nxt = min(end, t + dt)
            if nxt <= t + 1e-12:
                break
            t = nxt
            guard += 1
        def _schedule_of(
            samples: list[tuple[float, float, float, float]],
        ) -> dict[str, float]:
            knee_rate = 0.0
            ank_rate = 0.0
            for a, b in zip(samples, samples[1:]):
                span = max(float(b[0]) - float(a[0]), 1e-6)
                knee_rate = max(knee_rate, abs(float(b[1]) - float(a[1])) / span)
                ank_rate = max(ank_rate, abs(float(b[2]) - float(a[2])) / span)
            peak_rate = max(knee_rate, ank_rate)
            stretch_s = 1.0 if peak_rate <= 1.90 + 1e-9 else peak_rate / 1.90
            z_start = float(samples[0][3]) if samples else float("nan")
            z_stop = float(samples[-1][3]) if samples else float("nan")
            gait_at_peak = float(t0)
            z_at_peak = z_start
            for row in samples:
                if float(row[3]) > z_at_peak:
                    z_at_peak = float(row[3])
                    gait_at_peak = float(row[0])
            rise = z_at_peak - min(z_start, z_stop) if samples else 0.0
            if rise < 0.002:
                gait_land = float(t0)
            else:
                # Halfway down the leftover descent. A reload before that
                # is the liftoff brush, not double support.
                gait_land = gait_at_peak + 0.5 * (float(end) - gait_at_peak)
            wall_s = max(0.0, gait_land - float(t0)) * stretch_s
            return {
                "knee_rate": knee_rate,
                "ank_rate": ank_rate,
                "peak": peak_rate,
                "stretch": stretch_s,
                "z0": z_start,
                "zend": z_stop,
                "peak_gait": gait_at_peak,
                "peak_z": z_at_peak,
                "land_gait": gait_land,
                "wall": wall_s,
            }

        z0 = float(rows[0][3]) if rows else float("nan")
        zend = float(rows[-1][3]) if rows else float("nan")
        pref = "l_" if swing == "L" else "r_"
        seed: dict[str, float] = {}
        pre = state.get("pre")
        if isinstance(pre, dict):
            des = pre.get("des")
            if isinstance(des, dict):
                side_des = des.get(swing)
                if isinstance(side_des, dict):
                    for suf in ("knee", "ank_pitch"):
                        if suf in side_des:
                            seed[pref + suf] = float(side_des[suf])
        seed_knee = seed.get(pref + "knee")
        seed_ank = seed.get(pref + "ank_pitch")
        # The full rise needs ~0.49 s to the descent midpoint and longer
        # to the end. The tip on this stop is ~0.31 s after the command.
        # Scale the rise above the end height, largest scale first, until
        # the stretched schedule can finish, plus the first slew catch-up.
        end_budget = 0.28
        scales = (1.0, 0.85, 0.70, 0.55, 0.40, 0.25, 0.15, 0.0)
        chosen_rows = rows
        chosen = _schedule_of(rows) if rows else {
            "knee_rate": 0.0,
            "ank_rate": 0.0,
            "peak": 0.0,
            "stretch": 1.0,
            "z0": z0,
            "zend": zend,
            "peak_gait": float(t0),
            "peak_z": z0,
            "land_gait": float(t0),
            "wall": 0.0,
        }
        chosen_soft = 1.0
        chosen_catch = 0.0
        fitted = False
        if rows:
            for soft in scales:
                soft_rows: list[tuple[float, float, float, float]] = []
                for sample_t, _knee, _ank, z in rows:
                    z_s = float(zend) + float(soft) * (float(z) - float(zend))
                    ik = _swing_ik_at(swing, float(sample_t), z_override=z_s)
                    if ik is None:
                        continue
                    soft_rows.append((float(sample_t), float(ik[0]), float(ik[1]), float(ik[2])))
                if len(soft_rows) < 2:
                    continue
                info = _schedule_of(soft_rows)
                catch = 0.0
                if seed_knee is not None:
                    catch = max(catch, abs(float(soft_rows[0][1]) - float(seed_knee)) / 1.90)
                if seed_ank is not None:
                    catch = max(catch, abs(float(soft_rows[0][2]) - float(seed_ank)) / 1.90)
                end_wall = max(0.0, float(end) - float(t0)) * float(info["stretch"])
                info["end_wall"] = float(end_wall)
                fits = catch + end_wall <= end_budget + 1e-9
                if fits:
                    chosen_rows = soft_rows
                    chosen = info
                    chosen_soft = float(soft)
                    chosen_catch = float(catch)
                    fitted = True
                    break
                chosen_rows = soft_rows
                chosen = info
                chosen_soft = float(soft)
                chosen_catch = float(catch)
        knee_rate = float(chosen["knee_rate"])
        ank_rate = float(chosen["ank_rate"])
        peak = float(chosen["peak"])
        stretch = float(chosen["stretch"])
        peak_gait = float(chosen["peak_gait"])
        peak_z = float(chosen["peak_z"])
        land_gait = float(chosen["land_gait"])
        z0 = float(chosen["z0"])
        zend = float(chosen["zend"])
        if chosen_rows:
            state["z_soft_sched"] = [
                (float(row[0]), float(row[3])) for row in chosen_rows
            ]
        state["z_stretch"] = float(stretch)
        state["z_stretch_end"] = float(end)
        state["z_stretch_on"] = True
        state["z_sched_n"] = len(chosen_rows)
        state["z_peak_gait"] = float(peak_gait)
        state["z_land_gait"] = float(land_gait)
        edge["z_stretch"] = float(stretch)
        edge["z_peak_rate"] = float(peak)
        edge["z_knee_rate"] = float(knee_rate)
        edge["z_ank_rate"] = float(ank_rate)
        edge["z_from"] = float(t0)
        edge["z_end"] = float(end)
        edge["z_peak_gait"] = float(peak_gait)
        edge["z_land_gait"] = float(land_gait)
        edge["z_peak_z"] = float(peak_z)
        edge["z_soft"] = float(chosen_soft)
        edge["z_wall"] = float(chosen["wall"])
        edge["z_end_wall"] = float(chosen.get("end_wall", (float(end) - float(t0)) * stretch))
        edge["z_catch"] = float(chosen_catch)
        edge["z_fit"] = int(fitted)
        session._sag_swing_track = True  # type: ignore[attr-defined]
        session._sag_swing_joints = (pref + "knee", pref + "ank_pitch")  # type: ignore[attr-defined]
        session._sag_swing_seed = seed  # type: ignore[attr-defined]
        session._sag_swing_cmd = dict(seed)  # type: ignore[attr-defined]
        print(
            "PRED sag_stop_cap z stretch "
            f"swing {swing} factor {stretch:.3f} "
            f"soft {chosen_soft:.2f} fit {int(fitted)} "
            f"peak_rate {peak:.3f} rad/s "
            f"knee {knee_rate:.3f} ank {ank_rate:.3f} "
            f"gait {float(t0):.3f} to {end:.3f} "
            f"z {z0:+.5f} peak {peak_z:+.5f} at {peak_gait:.3f} "
            f"land_gate {land_gait:.3f} end_z {zend:+.5f} "
            f"wall {float(chosen['wall']):.3f} s "
            f"end_wall {float(chosen.get('end_wall', 0.0)):.3f} s "
            f"catch {chosen_catch:.3f} s "
            f"samples {len(chosen_rows)} "
            f"open_loop {(end - float(t0)) * stretch:.3f} s. "
            "The rise above the end height is scaled so the stretched "
            "schedule can finish before the tip. Knee and ankle pitch "
            "slew toward that IK at or under 1.90 rad/s and inside 2.33 Nm. "
            "The clock waits while either command is still more than one "
            "step behind."
        )

    def _swing_cmd(jn: str) -> float | None:
        cmd = getattr(session, "_sag_swing_cmd", None)
        if isinstance(cmd, dict) and jn in cmd:
            return float(cmd[jn])
        seed = getattr(session, "_sag_swing_seed", None)
        if isinstance(seed, dict) and jn in seed:
            return float(seed[jn])
        return None

    def _advance_z_clock() -> None:
        if state.get("z_stretch_on"):
            if state.get("z_stretch_hold"):
                return
            stretch = max(float(state.get("z_stretch", 1.0)), 1.0)
            end = float(state.get("z_stretch_end", float(walker.period)))
            now = float(walker.time)
            if now >= end - 1e-9:
                walker.time = end
                return
            step = float(ow.OP3_CTRL_S) / stretch
            proposed = min(end, now + step)
            swing = str(state.get("swing", ""))
            pref = "l_" if swing == "L" else "r_"
            cur = _swing_ik_at(swing, now)
            cap = 1.90 * float(ow.OP3_CTRL_S)
            knee_cmd = _swing_cmd(pref + "knee")
            ank_cmd = _swing_cmd(pref + "ank_pitch")

            def _near(sample: tuple[float, float, float] | None) -> bool:
                if cur is None or sample is None:
                    return False
                return (
                    abs(float(sample[0]) - float(cur[0])) <= cap + 1e-6
                    and abs(float(sample[1]) - float(cur[1])) <= cap + 1e-6
                )

            cmd_ok = (
                cur is not None
                and knee_cmd is not None
                and ank_cmd is not None
                and abs(float(knee_cmd) - float(cur[0])) <= cap + 1e-6
                and abs(float(ank_cmd) - float(cur[1])) <= cap + 1e-6
            )
            nxt = _swing_ik_at(swing, proposed) if cmd_ok else None
            if cmd_ok and not _near(nxt) and proposed > now + 1e-9:
                # The nominal stretch step still outruns the slew here.
                # Shorten it. Holding the clock forever leaves the foot up.
                lo = 0.0
                hi = float(proposed - now)
                for _ in range(12):
                    mid = 0.5 * (lo + hi)
                    trial = _swing_ik_at(swing, now + mid)
                    if _near(trial):
                        lo = mid
                    else:
                        hi = mid
                if lo >= 1e-5:
                    proposed = now + lo
                    nxt = _swing_ik_at(swing, proposed)
                else:
                    nxt = None
            if cmd_ok and _near(nxt):
                walker.time = float(proposed)
            else:
                state["z_wait_n"] = int(state.get("z_wait_n", 0)) + 1
                edge["z_miss_knee_cmd"] = (
                    float("nan") if knee_cmd is None else float(knee_cmd)
                )
                edge["z_miss_ank_cmd"] = (
                    float("nan") if ank_cmd is None else float(ank_cmd)
                )
                edge["z_miss_knee_ik"] = (
                    float("nan") if cur is None else float(cur[0])
                )
                edge["z_miss_ank_ik"] = (
                    float("nan") if cur is None else float(cur[1])
                )
                edge["z_miss_cmd_ok"] = int(cmd_ok)
            edge["z_waits"] = int(state["z_wait_n"])
            edge["z_gait_now"] = float(walker.time)
            return
        edge["z_waits"] = int(state.get("z_wait_n", 0))
        edge["z_gait_now"] = float(walker.time)
        walker.time = float(walker.time) + float(ow.OP3_CTRL_S)
        if walker.time >= float(walker.period) - 1e-12:
            walker.time = 0.0

    def _apply_slew(t: float, joints: dict[str, float]) -> None:
        """Move pinned q_des toward measured q without holding the 0.16 rad miss.

        The time blend covers 2 periods. The torque cap is applied on the
        same tick, because that blend alone leaves the first loaded tick
        over 2.33 Nm.
        """
        if not state["slew"] or not state["snapped"]:
            return
        stance_q = state["stance_q"]
        assert isinstance(stance_q, dict)
        if not state["slew_started"]:
            seeded: dict[str, dict[str, float]] = {}
            goals: dict[str, dict[str, float]] = {}
            for side in ("L", "R"):
                pref = "l_" if side == "L" else "r_"
                src = stance_q.get(side)
                if not isinstance(src, dict):
                    src = {
                        suf: float(joints[pref + suf]) for suf in _STANCE_SUFFIX
                    }
                    stance_q[side] = src
                seeded[side] = {suf: float(src[suf]) for suf in _STANCE_SUFFIX}
                goals[side] = {}
                for suf in _STANCE_SUFFIX:
                    jn = pref + suf
                    # Stand is the hold. Measured q at this tick is the
                    # other candidate; the roll miss is what stand removes.
                    q_now = float(lipm.q(jn))
                    q_stand = float(lipm.q_stand.get(jn, q_now))
                    goals[side][suf] = q_stand
            state["slew_from"] = seeded
            state["slew_goal"] = goals
            state["slew_t0"] = float(t)
            state["slew_started"] = True
            state["stance_sides"] = ["L", "R"]
            print(
                "PRED inflight_slew start "
                f"t {t:.3f} duration {float(state['slew_s']):.3f} s "
                "toward the stand pose, held after the window. "
                "Both feet are loaded."
            )
        t0 = float(state["slew_t0"])
        duration = float(state["slew_s"])
        u = 0.0 if duration <= 1e-9 else min(1.0, max(0.0, (float(t) - t0) / duration))
        state["slew_u"] = u
        slew_from = state["slew_from"]
        slew_goal = state["slew_goal"]
        assert isinstance(slew_from, dict) and isinstance(slew_goal, dict)
        parts: list[dict[str, float | str | int]] = []
        for side in ("L", "R"):
            pref = "l_" if side == "L" else "r_"
            held = stance_q.get(side)
            src = slew_from.get(side)
            goal_side = slew_goal.get(side)
            if (
                not isinstance(held, dict)
                or not isinstance(src, dict)
                or not isinstance(goal_side, dict)
            ):
                continue
            for suf in _STANCE_SUFFIX:
                jn = pref + suf
                pin = float(src[suf])
                goal = float(goal_side[suf])
                q = float(lipm.q(jn))
                jid = mj.mj_name2id(lipm.model, mj.mjtObj.mjOBJ_JOINT, jn)
                omega = float(lipm.data.qvel[int(lipm.model.jnt_dofadr[jid])])
                idx = lipm.act_idx[jn + "_pos"]
                kp = float(lipm.model.actuator_gainprm[idx, 0])
                kv = -float(lipm.model.actuator_biasprm[idx, 2])
                proposed = pin + u * (goal - pin)
                kv_abs = abs(kv * omega)
                budget = KNEE_NM - kv_abs
                capped = 0
                if budget <= 0.0 or kp < 1e-6:
                    proposed = q
                    capped = 1
                else:
                    max_err = budget / kp
                    err = proposed - q
                    if abs(err) > max_err:
                        proposed = q + math.copysign(max_err, goal - q if abs(goal - q) > 1e-9 else err)
                        capped = 1
                held[suf] = float(proposed)
                kp_abs = abs(kp * (proposed - q))
                parts.append({
                    "jn": jn,
                    "q": q,
                    "q_des": float(proposed),
                    "pin": pin,
                    "goal": goal,
                    "kp_abs": kp_abs,
                    "kv_abs": kv_abs,
                    "parts": kp_abs + kv_abs,
                    "capped": capped,
                    "omega": omega,
                })
        state["slew_parts"] = parts

    def _write_chain(mode: str, freeze: str) -> None:
        """Write pinned stance q_des. Airborne joints come from the z-only IK."""
        lipm.cmd_vx = 0.0
        lipm.cmd_yaw = 0.0
        lipm.yaw_target = 0.0
        lipm.phase = "shift" if state["snapped"] else "swing"
        joints, _info = walker.joints_now()
        state["pose_gait"] = float(walker.time)
        if joints is None:
            return
        _apply_slew(float(lipm.data.time), joints)
        stance_q = state["stance_q"]
        assert isinstance(stance_q, dict)
        sides = state["stance_sides"]
        assert isinstance(sides, list)
        written: dict[str, dict[str, float]] = {"L": {}, "R": {}}
        ik_knee = float("nan")
        pin_knee = float("nan")
        swing_hold = state.get("swing_hold")
        if not isinstance(swing_hold, dict):
            swing_hold = {}
        # These four stay at the pre-stop q_des until that foot is on the
        # stand slew. Knee and ankle pitch stay on the live IK, which is
        # the remaining swing-z schedule.
        freeze_suf = ("hip_yaw", "hip_roll", "hip_pitch", "ank_roll")
        for side in ("L", "R"):
            pref = "l_" if side == "L" else "r_"
            held = stance_q.get(side)
            if isinstance(held, dict):
                ik_name = pref + "knee"
                if side == (sides[0] if sides else ""):
                    ik_knee = float(joints[ik_name])
                    pin_knee = float(held["knee"])
                for suf in _STANCE_SUFFIX:
                    val = float(held[suf])
                    lipm.write_clipped(pref + suf, val)
                    written[side][suf] = val
                continue
            for suf in _STANCE_SUFFIX:
                jn = pref + suf
                if suf in freeze_suf and jn in swing_hold:
                    val = float(swing_hold[jn])
                else:
                    val = float(joints[jn])
                lipm.write_clipped(jn, val)
                written[side][suf] = val
        for jn in (
            "l_sho_pitch", "r_sho_pitch",
            "l_sho_roll", "r_sho_roll", "l_el_pitch", "r_el_pitch",
            "l_el_yaw", "r_el_yaw", "l_gripper", "r_gripper",
        ):
            if jn in joints:
                lipm.write_clipped(jn, float(joints[jn]))
        lipm._write_unused()
        state["ik_knee"] = ik_knee
        state["pin_knee"] = pin_knee
        delta = ik_knee - pin_knee
        if abs(delta) >= 1e-3 and not state["slew_started"]:
            print(
                "PRED inflight_ik_climb "
                f"t {float(lipm.data.time):.3f} "
                f"ik_knee {ik_knee:+.5f} pin_knee {pin_knee:+.5f} "
                f"delta {delta:+.5f}. Written q_des stays on the pin."
            )
        knees = _knees_now()
        row = _stamp(_soft_snapshot(
            session, mode, float(walker._x_move), float(walker.x_cmd), knees,
        ), freeze)
        row["des"] = written
        row["pel_r"] = float(state["pel"][0])
        row["pel_l"] = float(state["pel"][1])
        row["pelvis_frozen"] = 1
        row["ik_stance_knee"] = ik_knee
        row["pin_knee"] = pin_knee
        if state["slew"] and state["slew_started"]:
            row["slew_u"] = float(state.get("slew_u", 0.0))
            row["slew_parts"] = state.get("slew_parts", [])
            row["corners"] = {
                "L": _box_corner_table(session, "L"),
                "R": _box_corner_table(session, "R"),
            }
            row["com_l"] = _com_box_slack(session, "L")
            row["com_r"] = _com_box_slack(session, "R")
        trace.append(row)
        if not state["snapped"]:
            _advance_z_clock()

    def wrapped(vx: float, yaw_rate: float, walking: bool) -> None:
        del vx, yaw_rate
        t = float(lipm.data.time)
        label = driver.segment(t).label
        entered_now = False
        if not state["stop"]:
            if label != "stop":
                orig_tick(float(sw.VX_FWD_CAP), 0.0, walking)
                _remember_pre(t)
                return
            state["stop"] = True
            entered_now = True
            if state["sag_stop_cap"]:
                session._sag_torque_cap = True  # type: ignore[attr-defined]
                session._sag_stand_armed = False  # type: ignore[attr-defined]
                session._sag_stand_sides = ()  # type: ignore[attr-defined]
                print(
                    "PRED sag_stop_cap on. Stance knee and ankle pitch "
                    "keep the 1.90 rad/s slew and step toward the stand pose. "
                    "Each of those commands stays inside "
                    "|kp*(q_des-q)| + |kv*omega| <= 2.33 Nm. "
                    "Swing hip roll, hip yaw, hip pitch, and ankle roll hold "
                    "the pre-stop q_des until that foot is over 5 N. "
                    "Swing knee and ankle pitch track the time-stretched swing-z IK, "
                    "not a frozen q_des. "
                    "Planar x/y and sole roll/pitch hold. z is not pinned. "
                    "Loaded hips slew toward stand only after both feet are "
                    "over 5 N, inside the same 2.33 Nm part cap. "
                    "The airborne leg stays on that freeze."
                )
            pre = state["pre"]
            phase = _gait_phase_name(walker)
            load_l = _sole_load_n(session, "L")
            load_r = _sole_load_n(session, "R")
            # The clock swing can still be loaded. The in-flight foot is the
            # one at or under the load gate. A 0.52 N brush stays airborne.
            airborne_sides = [
                side for side, load in (("L", load_l), ("R", load_r)) if load <= load_n
            ]
            if len(airborne_sides) == 1:
                swing = airborne_sides[0]
            elif len(airborne_sides) == 2 and phase in ("L", "R"):
                swing = phase
            elif len(airborne_sides) == 2:
                swing = "L" if load_l <= load_r else "R"
            else:
                swing = ""
            if state["sag_stop_cap"] and swing == "" and phase in ("L", "R"):
                # Both feet are still over the gate at the start of this
                # single support. The clock swing has not left. That is
                # not a settled double support, and x_move stays.
                swing = phase
            state["swing"] = swing
            foot_up = (
                (swing == "L" and load_l <= load_n)
                or (swing == "R" and load_r <= load_n)
            )
            if foot_up:
                state["saw_air"] = True
            airborne = bool(foot_up)
            if state["sag_stop_cap"] and swing in ("L", "R") and isinstance(pre, dict):
                # From the stop command. Planar x/y and the four non-sagittal
                # swing joints hold. Knee and ankle pitch are not in this pin.
                ep = pre["el"] if swing == "L" else pre["er"]
                state["swing_pin"] = (
                    float(ep[0]), float(ep[1]), float(ep[3]), float(ep[4]), float(ep[5]),
                )
                state["pin_swing"] = True
                state["xy_only"] = False
                state["toe_hold"] = _swing_toe(swing, float(pre["gait"]))
                des = pre.get("des")
                hold: dict[str, float] = {}
                if isinstance(des, dict):
                    for side_name in ("L", "R"):
                        side_des = des.get(side_name)
                        if not isinstance(side_des, dict):
                            continue
                        pref = "l_" if side_name == "L" else "r_"
                        for suf in ("hip_yaw", "hip_roll", "hip_pitch", "ank_roll"):
                            if suf in side_des:
                                hold[pref + suf] = float(side_des[suf])
                state["swing_hold"] = hold
                session._sag_joint_pin = hold  # type: ignore[attr-defined]
                bits = []
                pref = "l_" if swing == "L" else "r_"
                for suf in ("hip_yaw", "hip_roll", "hip_pitch", "ank_roll"):
                    jn = pref + suf
                    if jn in hold:
                        bits.append(f"{suf} {hold[jn]:+.5f}")
                print(
                    "PRED sag_stop_cap swing freeze "
                    f"t {t:.3f} swing {swing} "
                    + " ".join(bits)
                    + f" x {float(ep[0]):+.5f} y {float(ep[1]):+.5f}. "
                    "Sole roll and pitch hold. z advances on the stretched clock. "
                    "Knee and ankle pitch track that IK. Ankle roll stays on the pin "
                    "until the foot is loaded, then sole-flat."
                )
                _arm_z_stretch(swing, float(walker.time))
            if airborne and isinstance(pre, dict) and not state["pin_swing"]:
                ep = pre["el"] if swing == "L" else pre["er"]
                state["swing_pin"] = (
                    float(ep[0]), float(ep[1]), float(ep[3]), float(ep[4]), float(ep[5]),
                )
                state["pin_swing"] = True
                state["toe_hold"] = _swing_toe(swing, float(pre["gait"]))
            state["pose_gait"] = float(walker.time)
            pre_row = state.get("pre_row")
            if isinstance(pre_row, dict):
                trace.append(pre_row)
            else:
                knees_pre = _knees_now()
                pre_snap = _stamp(_soft_snapshot(
                    session, "pre",
                    float(pre["x_move"]) if isinstance(pre, dict) else float("nan"),
                    float(pre["x_cmd"]) if isinstance(pre, dict) else float("nan"),
                    pre["knees"] if isinstance(pre, dict) else knees_pre,
                ), "none")
                trace.append(pre_snap)
            # Unfrozen plan at this tick, before the pin is allowed to hide it.
            saved_pin = bool(state["pin_swing"])
            state["pin_swing"] = False
            raw_joints, _raw_info = walker.joints_now()
            state["pin_swing"] = saved_pin
            raw = state.get("last_raw")
            plan_ep = None
            if isinstance(raw, tuple) and swing in ("L", "R"):
                plan_ep = _ep_of(swing, raw)
            pin_ep = None
            if isinstance(pre, dict) and swing in ("L", "R"):
                pin_ep = pre["el"] if swing == "L" else pre["er"]
            jump = False
            if plan_ep is not None and pin_ep is not None and raw_joints is not None:
                knee_name = "l_knee" if swing == "L" else "r_knee"
                pre_des = float(pre["knees"][f"{swing.lower()}_des"]) if isinstance(pre, dict) else float("nan")
                plan_des = float(raw_joints[knee_name])
                dx = float(plan_ep[0]) - float(pin_ep[0])
                dy = float(plan_ep[1]) - float(pin_ep[1])
                dz = float(plan_ep[2]) - float(pin_ep[2])
                droll = float(plan_ep[3]) - float(pin_ep[3])
                dpitch = float(plan_ep[4]) - float(pin_ep[4])
                ddes = plan_des - pre_des
                planar_jump = (
                    abs(dx) >= 0.001 or abs(dy) >= 0.001
                    or abs(droll) >= 0.005 or abs(dpitch) >= 0.005
                )
                qdes_jump = abs(ddes) >= 0.01
                jump = planar_jump or qdes_jump
                edge["jump"] = jump
                edge["planar_jump"] = planar_jump
                edge["qdes_jump"] = qdes_jump
                edge["dx"] = dx
                edge["dy"] = dy
                edge["dz"] = dz
                edge["droll"] = droll
                edge["dpitch"] = dpitch
                edge["ddes"] = ddes
                edge["pre_t"] = float(pre["t"]) if isinstance(pre, dict) else float("nan")
                edge["pre_gait"] = float(pre["gait"]) if isinstance(pre, dict) else float("nan")
                edge["post_t"] = t
                edge["post_gait"] = float(walker.time)
                edge["swing"] = swing
                edge["pre_des"] = pre_des
                edge["plan_des"] = plan_des
                edge["pre_x"] = float(pin_ep[0])
                edge["pre_y"] = float(pin_ep[1])
                edge["pre_z"] = float(pin_ep[2])
                edge["pre_roll"] = float(pin_ep[3])
                edge["pre_pitch"] = float(pin_ep[4])
                edge["plan_x"] = float(plan_ep[0])
                edge["plan_y"] = float(plan_ep[1])
                edge["plan_z"] = float(plan_ep[2])
                edge["plan_roll"] = float(plan_ep[3])
                edge["plan_pitch"] = float(plan_ep[4])
                print(
                    "PRED inflight_edge pre "
                    f"t {edge['pre_t']:.3f} gait_t {edge['pre_gait']:.3f} "
                    f"swing {swing} "
                    f"x {edge['pre_x']:+.5f} y {edge['pre_y']:+.5f} z {edge['pre_z']:+.5f} "
                    f"roll {edge['pre_roll']:+.5f} pitch {edge['pre_pitch']:+.5f} "
                    f"knee_des {pre_des:+.5f}"
                )
                print(
                    "PRED inflight_edge plan "
                    f"t {t:.3f} gait_t {float(walker.time):.3f} "
                    f"swing {swing} "
                    f"x {edge['plan_x']:+.5f} y {edge['plan_y']:+.5f} z {edge['plan_z']:+.5f} "
                    f"roll {edge['plan_roll']:+.5f} pitch {edge['plan_pitch']:+.5f} "
                    f"knee_des {plan_des:+.5f}"
                )
                print(
                    "PRED inflight_edge jump "
                    f"{int(jump)} planar {int(planar_jump)} qdes {int(qdes_jump)} "
                    f"dx {dx:+.5f} dy {dy:+.5f} dz {dz:+.5f} "
                    f"droll {droll:+.5f} dpitch {dpitch:+.5f} dknee_des {ddes:+.5f}. "
                    "Pin is the pre-stop x/y/roll/pitch. z is the live plan."
                )
            state["edge_done"] = True
            print(
                f"PRED inflight_stop command t {t:.3f} "
                f"phase {phase} air {swing or 'none'} "
                f"load L {load_l:.2f} R {load_r:.2f} "
                f"pin {int(bool(state['pin_swing']))} "
                f"x_move {float(walker._x_move):+.5f} "
                "x_move not zeroed"
            )
            _arm_chain(pre, swing)
        if state["snapped"]:
            load_l = _sole_load_n(session, "L")
            load_r = _sole_load_n(session, "R")
            _arm_stand_sides(load_l, load_r)
            _write_chain("snap", "swing_xy+swing_z")
            return
        load_l = _sole_load_n(session, "L")
        load_r = _sole_load_n(session, "R")
        both = load_l > load_n and load_r > load_n
        swing = str(state["swing"])
        if state["sag_stop_cap"] and (load_l <= load_n or load_r <= load_n):
            state["saw_air"] = True
        if (
            state["sag_stop_cap"]
            and swing in ("L", "R")
            and not state["pin_swing"]
            and (
                (swing == "L" and load_l <= load_n)
                or (swing == "R" and load_r <= load_n)
            )
        ):
            er_now, el_now, _pel_r, _pel_l, _swap_y = orig_endpoints()
            ep_now = el_now if swing == "L" else er_now
            state["swing_pin"] = (
                float(ep_now[0]), float(ep_now[1]),
                float(ep_now[3]), float(ep_now[4]), float(ep_now[5]),
            )
            state["pin_swing"] = True
            state["toe_hold"] = _swing_toe(swing, float(walker.time))
            up = load_l if swing == "L" else load_r
            print(
                "PRED sag_stop_cap swing foot unloaded "
                f"t {t:.3f} swing {swing} load {up:.2f} N. "
                "x/y/roll/pitch pin at this pose. z still advances. "
                "x_move is not zeroed."
            )
        if state["sag_stop_cap"]:
            edge["z_gait_last"] = float(walker.time)
            edge["z_waits"] = int(state.get("z_wait_n", 0))
            edge["load_l_last"] = float(load_l)
            edge["load_r_last"] = float(load_r)
            edge["saw_air"] = int(bool(state.get("saw_air")))
            sw_load = load_l if str(state.get("swing")) == "L" else load_r
            prev_min = edge.get("swing_load_min")
            if not isinstance(prev_min, float) or float(sw_load) < float(prev_min):
                edge["swing_load_min"] = float(sw_load)
                edge["swing_load_min_t"] = float(t)
        land_gate = state.get("z_land_gait")
        past_land = (
            land_gate is None
            or float(walker.time) + 1e-9 >= float(land_gate)
        )
        schedule_done = (
            bool(state.get("z_stretch_on"))
            and float(walker.time) + 1e-9 >= float(state.get("z_stretch_end", 1e9))
        )
        if (
            state["sag_stop_cap"]
            and swing in ("L", "R")
            and past_land
            and (state["saw_air"] or schedule_done)
            and not state.get("touch_logged")
        ):
            load_sw = load_l if swing == "L" else load_r
            if load_sw > load_n:
                state["touch_logged"] = True
                state["z_stretch_hold"] = True
                session._sag_swing_track = False  # type: ignore[attr-defined]
                pref = "l_" if swing == "L" else "r_"
                corners = _box_corner_table(session, swing)
                low = min(corners, key=lambda row: float(row[1]))
                bits = " ".join(
                    f"{label} {float(z) * 1000.0:+.3f}" for label, z, _off in corners
                )
                hold = state.get("swing_hold")
                roll_pin = float("nan")
                if isinstance(hold, dict) and (pref + "ank_roll") in hold:
                    roll_pin = float(hold[pref + "ank_roll"])
                edge["touchdown"] = {
                    "t": float(t),
                    "swing": swing,
                    "load": float(load_sw),
                    "lowest": str(low[0]),
                    "lowest_mm": float(low[1]) * 1000.0,
                    "waits": int(state.get("z_wait_n", 0)),
                    "gait": float(walker.time),
                }
                print(
                    "PRED sag_stop_cap touchdown "
                    f"t {t:.3f} swing {swing} load {load_sw:.2f} N "
                    f"gait {float(walker.time):.3f} "
                    f"waits {int(state.get('z_wait_n', 0))} "
                    f"lowest {low[0]} {float(low[1]) * 1000.0:+.3f} mm "
                    + bits
                    + f" ank_roll q {float(lipm.q(pref + 'ank_roll')):+.5f} "
                    f"pin {roll_pin:+.5f} "
                    f"ank_pitch q {float(lipm.q(pref + 'ank_pitch')):+.5f}. "
                    "Ankle roll sole-flat applies on this write. "
                    "The four corners are the contact box at the first load over 5 N."
                )
        clock = _gait_phase_name(walker)
        real_dsp = bool(both and state["pin_chain"])
        if state["sag_stop_cap"]:
            # A loaded clock-swing is not double support. The snap also
            # waits one tick so the stop command itself does not zero x_move.
            if clock in ("L", "R") and not state["saw_air"]:
                real_dsp = False
            if entered_now:
                real_dsp = False
            # The liftoff brush reloads both feet before the descent.
            # That is not the landing.
            if state.get("z_stretch_on") and not past_land:
                real_dsp = False
            if schedule_done and both and past_land and not entered_now:
                real_dsp = True
            if (
                both
                and not real_dsp
                and not state["saw_air"]
                and clock in ("L", "R")
                and not state["told_hold"]
            ):
                state["told_hold"] = True
                print(
                    "PRED sag_stop_cap both feet loaded in single support "
                    f"t {t:.3f} clock {clock} gait_t {float(walker.time):.3f} "
                    f"load L {load_l:.2f} R {load_r:.2f}. "
                    "This is not a settled double support. "
                    "x_move stays. The snap waits until a foot has been "
                    "airborne or the clock is in double support, and not "
                    "on the stop tick."
                )
        if real_dsp:
            raw_now = orig_endpoints()
            live_l = float(raw_now[1][2])
            live_r = float(raw_now[0][2])
            stance_ep = state["stance_ep"]
            assert isinstance(stance_ep, dict)
            z_pin = {"L": live_l, "R": live_r}
            for side, held in stance_ep.items():
                if isinstance(held, np.ndarray):
                    z_pin[str(side)] = float(held[2])
            if swing in ("L", "R"):
                z_pin[swing] = live_l if swing == "L" else live_r
            state["z_pin"] = z_pin
            state["pin_z"] = True
            state["pin_stance"] = True
            walker.x_cmd = 0.0
            walker.y_cmd = 0.0
            walker.angle_cmd = 0.0
            walker.update_movement()
            state["snapped"] = True
            if state["sag_stop_cap"]:
                session._sag_both_stance = True  # type: ignore[attr-defined]
                _arm_stand_sides(load_l, load_r)
                print(
                    "PRED sag_stop_cap both feet loaded. "
                    "Knee and ankle pitch on a loaded leg keep the 1.90 rad/s slew. "
                    "Loaded hips step toward stand inside the 2.33 Nm part cap. "
                    "A foot at or under 5 N stays on the swing freeze."
                )
            print(
                "PRED inflight_snap "
                f"t {t:.3f} gait_t {float(walker.time):.3f} "
                f"load L {load_l:.2f} R {load_r:.2f} "
                f"x_cmd {float(walker.x_cmd):+.5f} "
                f"x_move {float(walker._x_move):+.5f} "
                "amplitudes 0. "
                + (
                    "Stance q_des starts the slew toward the stand pose. "
                    if state["slew"]
                    else "Stance q_des and planar held. "
                )
                + "Clock stays here."
            )
            _write_chain("snap", "swing_xy+swing_z")
            return
        if not state["pin_chain"]:
            orig_tick(float(sw.VX_FWD_CAP), 0.0, True)
            state["pose_gait"] = _pose_clock()
            knees = _knees_now()
            row = _stamp(_soft_snapshot(
                session, "finish", float(walker._x_move), float(walker.x_cmd), knees,
            ), "none")
            trace.append(row)
            return
        _write_chain(
            "air",
            "swing_z+stance_q" if state["pin_swing"] else "stance_q",
        )

    lipm.tick = wrapped  # type: ignore[method-assign]


def _joint_damping(session: sw.SteerSession, jn: str) -> tuple[float, float, float, float]:
    """Joint damping force ``-damping * ω``, plus ω, the coefficient, and qfrc_passive."""
    jid = mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_JOINT, jn)
    dof = int(session.model.jnt_dofadr[jid])
    damp = float(session.model.dof_damping[dof])
    omega = float(session.data.qvel[dof])
    return -damp * omega, omega, damp, float(session.data.qfrc_passive[dof])


def _com_box_slack(session: sw.SteerSession, side: str) -> dict[str, float]:
    """CoM slack inside the foot contact box. Positive is inside. Margin is 0.

    Lateral slack is the box y half-width (38 mm on this plant) minus the
    distance from the geom center. This is the contact box, not the 8 mm
    inside-margin inset.
    """
    lipm = session.lipm
    if lipm is None:
        raise RuntimeError("gait manager missing")
    bid = lipm.bid[side]
    gid = lipm.gid[side]
    com = np.asarray(lipm.data.subtree_com[lipm.bid_body], dtype=np.float64)
    rot = lipm.data.xmat[bid].reshape(3, 3)
    local = rot.T @ (com - np.asarray(lipm.data.xpos[bid], dtype=np.float64))
    center = np.asarray(lipm.model.geom_pos[gid, :2], dtype=np.float64)
    half = np.asarray(lipm.model.geom_size[gid, :2], dtype=np.float64)
    delta = local[:2] - center
    slack = half - np.abs(delta)
    return {
        "slack_x": float(slack[0]),
        "slack_y": float(slack[1]),
        "local_x": float(local[0]),
        "local_y": float(local[1]),
        "center_x": float(center[0]),
        "center_y": float(center[1]),
        "half_x": float(half[0]),
        "half_y": float(half[1]),
    }


_SOLE_FLAT_LOAD_N = 5.0


def _install_sole_flat_stance(session: sw.SteerSession, mode: str = "q") -> None:
    """Loaded ankle roll stays with the sole. Hip roll keeps the y_swap IK.

    ``q`` sets q_des to the measured angle when floor+rug load is over 5 N,
    so the kp term is zero and the ankle does not fight the ground.
    ``zero`` sets that q_des to 0. Level trim is swing-only and is not
    added on this write. Hip yaw stays the gait IK. Plant kp is not changed.
    """
    if mode not in ("q", "zero"):
        raise SystemExit(f"sole-flat mode {mode} is not q or zero")
    lipm = session.lipm
    if lipm is None:
        raise RuntimeError("gait manager missing")
    rug = int(session.gid_rug)
    orig = lipm.write_clipped
    log: dict[str, dict[str, float]] = {}
    session._sole_flat_log = log  # type: ignore[attr-defined]
    session._sole_flat_mode = mode  # type: ignore[attr-defined]

    def write(jn: str, q_des: float) -> None:
        if jn in ("l_ank_roll", "r_ank_roll"):
            side = "L" if jn.startswith("l_") else "R"
            fn = float(_foot_surface(session, side, rug)["fn"])
            q = float(lipm.q(jn))
            gait = float(q_des)
            flat = 0.0
            if fn > _SOLE_FLAT_LOAD_N:
                flat = 1.0
                q_des = q if mode == "q" else 0.0
            log[jn] = {
                "gait": gait,
                "des": float(q_des),
                "q": q,
                "fn": fn,
                "flat": flat,
            }
        orig(jn, float(q_des))

    lipm.write_clipped = write  # type: ignore[method-assign]
    target = "measured q" if mode == "q" else "0 rad"
    print(
        f"PRED sole_flat mode {mode}. "
        f"A foot over {_SOLE_FLAT_LOAD_N:.1f} N sets ankle-roll q_des to {target}. "
        "Level trim stays off that write. Hip roll keeps the y_swap IK. "
        "Hip yaw is not added. The loaded ankle is not tipped to make the lateral."
    )


def _note_walk_lateral(
    session: sw.SteerSession,
    walker: object,
    t_post: float,
    bucket: list[dict[str, float | str]],
) -> None:
    """One row per control tick: gait command, trim add, and CoM box slack.

    The pose clock is the time ``joints_now`` just used. Level trim is the
    swing-ankle add (lead off). Double support adds none. The unclamped
    ask log is separate and is joined on ``t_ask``.
    """
    lipm = session.lipm
    if lipm is None:
        return
    dt = float(session.ctrl_dt)
    t_ask = float(t_post) - dt
    if str(lipm.phase) == "stand":
        pose = float("nan")
        phase = "stand"
        joints = None
        nox = None
        pel_r = 0.0
        pel_l = 0.0
        swap_y = 0.0
        ep_roll_r = 0.0
        ep_roll_l = 0.0
        ep_x_r = float("nan")
        ep_z_r = float("nan")
        ep_x_l = float("nan")
        ep_z_l = float("nan")
    else:
        pose = _cmd_time(walker)
        saved = float(walker.time)
        walker.time = pose
        joints, info = walker.joints_now()
        er, el, pel_r, pel_l, swap_y = walker.endpoints()
        nox = None
        if getattr(session, "_sag_split", False) and joints is not None:
            saved_move = float(walker._x_move)
            saved_swap = float(walker._x_swap)
            walker._x_move = 0.0
            walker._x_swap = 0.0
            nox, _nox_info = walker.joints_now()
            walker._x_move = saved_move
            walker._x_swap = saved_swap
        walker.time = saved
        phase = str(info.phase) if joints is not None else "?"
        ep_roll_r = float(er[3])
        ep_roll_l = float(el[3])
        ep_x_r = float(er[0])
        ep_z_r = float(er[2])
        ep_x_l = float(el[0])
        ep_z_l = float(el[2])
    rug_gid = int(session.gid_rug)

    def _jn(name: str) -> float:
        if joints is None or name not in joints:
            return float("nan")
        return float(joints[name])

    def _nox_q(name: str) -> float:
        if nox is None or name not in nox:
            return float("nan")
        return float(nox[name])

    def _stand_q(name: str) -> float:
        table = lipm.q_stand
        if name not in table:
            return float("nan")
        return float(table[name])

    r_body = np.asarray(session.data.xmat[session.bid_body], dtype=np.float64).reshape(3, 3)
    _body_roll, body_pitch, _body_yaw = _decompose_rpy(r_body)

    dir_l = float(walker.directions["l_hip_roll"])
    dir_r = float(walker.directions["r_hip_roll"])
    pel_add_l = dir_l * float(pel_l)
    pel_add_r = dir_r * float(pel_r)
    trim_l = _RESTORED_ROLL_L if phase == "L" else 0.0
    trim_r = _RESTORED_ROLL_R if phase == "R" else 0.0
    slack_l = _com_box_slack(session, "L")
    slack_r = _com_box_slack(session, "R")
    fn_l = float(_foot_surface(session, "L", rug_gid)["fn"])
    fn_r = float(_foot_surface(session, "R", rug_gid)["fn"])
    flat_log = getattr(session, "_sole_flat_log", {}) or {}

    def _flat(jn: str, key: str) -> float:
        packed = flat_log.get(jn) if isinstance(flat_log, dict) else None
        if not isinstance(packed, dict) or key not in packed:
            return float("nan")
        return float(packed[key])

    def _corner_min(side: str) -> float:
        rows = _box_corner_table(session, side)
        if not rows:
            return float("nan")
        return float(min(z for _label, z, _off in rows))
    if phase == "L":
        stance = "R"
    elif phase == "R":
        stance = "L"
    else:
        stance = "-"
    bucket.append({
        "t_ask": t_ask,
        "t_post": float(t_post),
        "pose": float(pose),
        "phase": phase,
        "stance": stance,
        "swap_y": float(swap_y),
        "y_swap_cmd": float(walker.y_swap_cmd),
        "y_swap_amp": float(walker._y_swap),
        "pel_l": float(pel_l),
        "pel_r": float(pel_r),
        "pel_add_l": pel_add_l,
        "pel_add_r": pel_add_r,
        "ep_roll_l": ep_roll_l,
        "ep_roll_r": ep_roll_r,
        "gait_l_ank_roll": _jn("l_ank_roll"),
        "gait_r_ank_roll": _jn("r_ank_roll"),
        "gait_l_hip_roll": _jn("l_hip_roll"),
        "gait_r_hip_roll": _jn("r_hip_roll"),
        "gait_l_knee": _jn("l_knee"),
        "gait_r_knee": _jn("r_knee"),
        "gait_l_ank_pitch": _jn("l_ank_pitch"),
        "gait_r_ank_pitch": _jn("r_ank_pitch"),
        "gait_l_hip_pitch": _jn("l_hip_pitch"),
        "gait_r_hip_pitch": _jn("r_hip_pitch"),
        "nox_l_knee": _nox_q("l_knee"),
        "nox_r_knee": _nox_q("r_knee"),
        "nox_l_ank_pitch": _nox_q("l_ank_pitch"),
        "nox_r_ank_pitch": _nox_q("r_ank_pitch"),
        "nox_l_hip_pitch": _nox_q("l_hip_pitch"),
        "nox_r_hip_pitch": _nox_q("r_hip_pitch"),
        "stand_l_knee": _stand_q("l_knee"),
        "stand_r_knee": _stand_q("r_knee"),
        "stand_l_ank_pitch": _stand_q("l_ank_pitch"),
        "stand_r_ank_pitch": _stand_q("r_ank_pitch"),
        "stand_l_hip_pitch": _stand_q("l_hip_pitch"),
        "stand_r_hip_pitch": _stand_q("r_hip_pitch"),
        "ep_x_l": ep_x_l,
        "ep_z_l": ep_z_l,
        "ep_x_r": ep_x_r,
        "ep_z_r": ep_z_r,
        "x_cmd": float(walker.x_cmd),
        "x_move": float(walker._x_move),
        "body_pitch": float(body_pitch),
        "sole_pitch_l": float(_sole_pitch(session, "L")),
        "sole_pitch_r": float(_sole_pitch(session, "R")),
        "trim_l": trim_l,
        "trim_r": trim_r,
        "fn_l": fn_l,
        "fn_r": fn_r,
        "slack_x_l": slack_l["slack_x"],
        "slack_y_l": slack_l["slack_y"],
        "slack_x_r": slack_r["slack_x"],
        "slack_y_r": slack_r["slack_y"],
        "half_x_l": slack_l["half_x"],
        "half_y_l": slack_l["half_y"],
        "local_y_l": slack_l["local_y"],
        "local_y_r": slack_r["local_y"],
        "center_y_l": slack_l["center_y"],
        "center_y_r": slack_r["center_y"],
        "y_out": float(walker.y_offset) / 2.0,
        "flat_mode": str(getattr(session, "_sole_flat_mode", "")),
        "flat_l": _flat("l_ank_roll", "flat"),
        "flat_r": _flat("r_ank_roll", "flat"),
        "flat_des_l": _flat("l_ank_roll", "des"),
        "flat_des_r": _flat("r_ank_roll", "des"),
        "flat_q_l": _flat("l_ank_roll", "q"),
        "flat_q_r": _flat("r_ank_roll", "q"),
        "flat_gait_l": _flat("l_ank_roll", "gait"),
        "flat_gait_r": _flat("r_ank_roll", "gait"),
        "flat_fn_l": _flat("l_ank_roll", "fn"),
        "flat_fn_r": _flat("r_ank_roll", "fn"),
        "sole_roll_l": float(_sole_roll(session, "L")[0]),
        "sole_roll_r": float(_sole_roll(session, "R")[0]),
        "corner_l": _corner_min("L"),
        "corner_r": _corner_min("R"),
    })


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
    sag_cancel: bool = False,
    toe_up_shape: str = "sine",
    z_profile: str = "phase",
    early_peak_m: float = 0.0,
    knee_lead_s: float = 0.0,
    early_log: bool = False,
    toe_full_frac: float = 0.15,
    yswap_lead_s: float = 0.0,
    roll_log: bool = False,
    sole_corner_log: bool = False,
    ank_trim_l: float = 0.0,
    ank_trim_r: float = 0.0,
    ank_pitch_trim_l: float = 0.0,
    ank_pitch_trim_r: float = 0.0,
    ank_log: bool = False,
    ank_trim_lead: bool = False,
    ank_trim_on_frac: float = 0.208,
    t_end: float | None = None,
    segments: tuple[sw.DemoSegment, ...] | None = None,
    steer_out: dict[str, object] | None = None,
    command_ramp: bool = False,
    yaw_cap: float = sw.YAW_RATE_CAP,
    soft_stop: bool = False,
    inflight_stop: bool = False,
    next_stride_decay: bool = False,
    y_swap_m: float | None = None,
    lateral_log: bool = False,
    stance_slew: bool = False,
    sag_stop_cap: bool = False,
    sole_flat: bool = False,
    sole_flat_mode: str = "q",
    y_out_m: float | None = None,
    x_scale: float = 1.0,
    sag_slew_rad_s: float | None = None,
    sag_slew_joints: tuple[str, ...] = ("knee", "ank_pitch"),
    sag_split: bool = False,
    walk_z_stretch: bool = False,
) -> PredScore:
    """12 mm @ 20% with an optional swing-hip predicted-force clip.

    ``move_s`` None keeps the locked 20 ms approach. ``pitch_move_off``
    drops hip pitch from that approach and leaves knee and ankle pitch
    on it.     ``period_s`` None keeps 0.500 s. ``bar_before_s`` scores leg
    torques only before that time. ``world_level`` and ``toe_up_rad``
    are separate swing-sole copies. The 0.98 spring band is not widened.
    ``z_profile`` ``front`` and ``early`` replace the 12 mm parabola.
    They are not stacked on it. Sag cancel stays off unless asked.
    """
    if world_level and toe_up_rad > 0.0:
        raise SystemExit("world-level and toe-up are separate copies")
    if toe_up_shape not in ("sine", "front"):
        raise SystemExit(f"toe-up shape {toe_up_shape} is not sine or front")
    if z_profile not in ("phase", "front", "early"):
        raise SystemExit(f"z profile {z_profile} is not phase, front, or early")
    if z_profile == "early" and not (0.0 < early_peak_m <= 0.012 + 1e-9):
        raise SystemExit(f"early rise peak {early_peak_m} m is outside 0–12 mm")
    if sag_cancel and knee_lead_s > 0.0:
        raise SystemExit("swing knee lead is not stacked on stance sag cancel")
    if toe_up_shape == "front" and not 0.0 < toe_full_frac < 0.80:
        raise SystemExit(f"toe-up full fraction {toe_full_frac} is not inside 0-80%")
    if yswap_lead_s < 0.0:
        raise SystemExit(f"yswap lead {yswap_lead_s} s is negative")
    if sag_cancel and yswap_lead_s > 0.0:
        raise SystemExit("yswap lead is not stacked on stance sag cancel")
    if abs(ank_trim_l) > ANK_TRIM_CAP + 1e-12 or abs(ank_trim_r) > ANK_TRIM_CAP + 1e-12:
        raise SystemExit(
            f"ankle-roll trim L {ank_trim_l} R {ank_trim_r} exceeds ±{ANK_TRIM_CAP:.3f}"
        )
    if (
        abs(ank_pitch_trim_l) > ANK_TRIM_CAP + 1e-12
        or abs(ank_pitch_trim_r) > ANK_TRIM_CAP + 1e-12
    ):
        raise SystemExit(
            f"ankle-pitch trim L {ank_pitch_trim_l} R {ank_pitch_trim_r} "
            f"exceeds ±{ANK_TRIM_CAP:.3f}"
        )
    trim_on = (
        abs(ank_trim_l) > 1e-12 or abs(ank_trim_r) > 1e-12
        or abs(ank_pitch_trim_l) > 1e-12 or abs(ank_pitch_trim_r) > 1e-12
    )
    if world_level and trim_on:
        raise SystemExit("ankle trim is not a world-level sole")
    if ank_trim_lead and not trim_on:
        raise SystemExit("ankle trim lead needs a trim add")
    if ank_trim_lead and not (0.0 < ank_trim_on_frac < TRIM_HOLD_FRAC):
        raise SystemExit(
            f"ankle trim on fraction {ank_trim_on_frac} is not inside 0-80%"
        )
    cfg = sw.locked_kit_config()
    if y_swap_m is not None:
        cfg = replace(cfg, gm_y_swap_m=float(y_swap_m))
        print(
            f"PRED y_swap_cmd {float(y_swap_m):.4f} m "
            "replaces the locked 0.020 m on this copy. "
            "Pelvis stays 5 deg. HIP_FF is not on this stack. "
            "Plant, kp, and forcerange stay put."
        )
    if move_s is not None:
        cfg = replace(cfg, gm_move_s=float(move_s))
    if period_s is not None:
        cfg = replace(cfg, gm_period_s=float(period_s))
    session = sw.SteerSession(video=False, scene_xml=Path(SCENE), lipm=cfg)
    _check_compiled_kv(session)
    ank_kv = _check_ank_roll_kv(session) if ankle_roll else 0.0
    if z_profile == "phase":
        _install_phase_lift(session, 0.012, 0.20, 0.0)
    elif z_profile == "front":
        print(
            "PRED z_front 12.000 mm smoothstep from toe-off, "
            "full by 10% of single support, hold through 40%, down by 80%. "
            "Same peak as the 12 mm parabola. Not stacked on it."
        )
        _install_slow_rise(session, 0.012, 0.10, 0.40, 0.0)
    else:
        print(
            f"PRED early_rise {early_peak_m * 1000.0:.3f} mm smoothstep "
            "from toe-off, full by 10% of single support, hold through 40%, "
            "down by 80%. Replaces the 12 mm parabola. Not stacked on it."
        )
        _install_slow_rise(session, early_peak_m, 0.10, 0.40, 0.0)
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
        _install_swing_toe_up(
            session, toe_up_rad, shape=toe_up_shape, full_frac=toe_full_frac,
        )
    if hip_lead:
        _install_swing_hip_lead(
            session,
            roll_scale=lead_roll_scale,
            pitch_scale=lead_pitch_scale,
        )
    if yswap_lead_s > 0.0:
        _install_yswap_lead(session, yswap_lead_s)
    if knee_lead_s > 0.0:
        _install_swing_knee_lead(session, knee_lead_s)
    if sag_cancel:
        _install_stance_knee_sag(session)
    if (
        abs(ank_trim_l) > 1e-12 or abs(ank_trim_r) > 1e-12
        or abs(ank_pitch_trim_l) > 1e-12 or abs(ank_pitch_trim_r) > 1e-12
    ):
        _install_swing_ank_roll_trim(
            session, ank_trim_l, ank_trim_r, ank_pitch_trim_l, ank_pitch_trim_r,
            lead=ank_trim_lead, on_frac=ank_trim_on_frac,
        )
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        raise RuntimeError("gait manager walker missing")
    walker = lipm.op3
    if y_out_m is not None:
        walker.y_offset = 2.0 * float(y_out_m)
        stood = walker.stand_joints()
        for name, val in stood.items():
            if "sho" in name:
                continue
            lipm.q_stand[name] = float(val)
            session.q_stand[name] = float(val)
        session._reset_stand()
        print(
            f"PRED init_y {float(y_out_m):.4f} m outward on each foot. "
            f"y_offset {float(walker.y_offset):.4f} m. "
            "Foot placement only. The contact mesh is unchanged."
        )
    if not (0.0 - 1e-12 <= float(x_scale) <= 1.0 + 1e-12):
        raise SystemExit(f"x_scale {x_scale} is outside 0–1")
    if abs(float(x_scale) - 1.0) > 1e-12:
        orig_set = walker.set_command
        scale = float(x_scale)

        def set_command(x_amp: float, y_amp: float = 0.0, angle_rad: float = 0.0) -> None:
            orig_set(float(x_amp) * scale, y_amp, angle_rad)

        walker.set_command = set_command  # type: ignore[method-assign]
        print(
            f"PRED x_scale {scale:.3f}. Bus vx stays Day-1. "
            "The step length is that scale times the bus x_amp. "
            "Foot-z and the crouch stay put."
        )
    if walk_z_stretch and inflight_stop:
        raise SystemExit("walk swing-z stretch is not on the stop bout")
    if walk_z_stretch:
        _install_walk_z_stretch(session, _WALK_Z_RATE)
    if sag_split:
        session._sag_split = True  # type: ignore[attr-defined]
    end = T_END if t_end is None else float(t_end)
    if segments is None:
        segments = (
            sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
            sw.DemoSegment(end, "vel", sw.VX_FWD_CAP, 0.0, "forward"),
        )
    driver = sw.ScriptedDriver(segments)
    ramp_latches: list[tuple] = []
    soft_trace: list[dict[str, object]] = []
    if command_ramp and soft_stop:
        raise SystemExit("soft stop is not stacked on the yaw ramp")
    if inflight_stop and (soft_stop or command_ramp):
        raise SystemExit("inflight stop is not stacked on the soft stop or the yaw ramp")
    if command_ramp:
        if steer_out is None:
            raise SystemExit("command ramp needs the steer log")
        _install_command_ramp(lipm, driver, ramp_latches, float(yaw_cap))
    if soft_stop:
        if steer_out is None:
            raise SystemExit("soft stop needs the steer log")
        _install_soft_stop(session, driver, soft_trace)
    inflight_trace: list[dict[str, object]] = []
    inflight_edge: dict[str, object] = {}
    if inflight_stop:
        if steer_out is None:
            raise SystemExit("inflight stop needs the steer log")
        session._inflight_toe = (
            float(toe_up_rad), str(toe_up_shape), float(toe_full_frac),
        )
        _install_inflight_stop(
            session, driver, inflight_trace, inflight_edge,
            next_stride_decay=next_stride_decay,
            stance_slew=stance_slew,
            sag_stop_cap=sag_stop_cap,
        )
    rows: list[Tick] = []
    surface_rows: list[dict[str, object]] = []
    hx_chain = (
        "l_hip_pitch_pos", "r_hip_pitch_pos",
        "l_knee_pos", "r_knee_pos",
        "l_ank_pitch_pos", "r_ank_pitch_pos",
    )
    hx_flat = {act: (0.0, 0.0, "") for act in hx_chain}
    hx_rug = {act: (0.0, 0.0, "") for act in hx_chain}
    roll_chain = ("l_hip_roll_pos", "r_hip_roll_pos") if roll_log else ()
    roll_flat = {act: (0.0, 0.0, "") for act in roll_chain}
    roll_rug = {act: (0.0, 0.0, "") for act in roll_chain}
    ank_chain = ("l_ank_roll_pos", "r_ank_roll_pos") if ank_log else ()
    ank_flat = {act: (0.0, 0.0, "") for act in ank_chain}
    ank_rug = {act: (0.0, 0.0, "") for act in ank_chain}
    floor_acts = (
        "l_knee_pos", "r_knee_pos",
        "l_ank_pitch_pos", "r_ank_pitch_pos",
        "l_ank_roll_pos", "r_ank_roll_pos",
    )
    floor_peak = {act: (0.0, 0.0, "") for act in floor_acts}
    ka_names = hx_chain + roll_chain + ank_chain
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
    ank_split = 0.0
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
        if getattr(session, "_sag_cancel", False) and lipm.phase != "stand":
            for jn in _stance_knees(lipm):
                _clamp_ctrl_predicted(session, jn, KNEE_NM)
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
    ask_log: list[tuple] = []
    lateral_rows: list[dict[str, float | str]] = []
    if steer_out is not None:
        _install_unclamp_log(lipm, ask_log)
    if sole_flat:
        # After the ask log, so the logged q_des is the sole-flat command.
        _install_sole_flat_stance(session, sole_flat_mode)
    if sag_slew_rad_s is not None:
        # After the ask log, so the logged q_des is the slowed stance command.
        _install_sagittal_slew(session, float(sag_slew_rad_s), sag_slew_joints)
    try:
        while float(session.data.time) < end - 1e-9:
            driver.publish(session.bus, float(session.data.time))
            session.step()
            t = float(session.data.time)
            if lateral_log:
                _note_walk_lateral(session, walker, t, lateral_rows)
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
                if swing in ("L", "R") and session.lipm is not None and session.lipm.op3 is not None:
                    zlog = getattr(session, "_z_cmd_log", {})
                    period = float(session.lipm.op3.period)
                    phase = t % period if period > 1e-9 else t
                    best_dt = 1e9
                    for (ts, sd), (z_val, added) in zlog.items():
                        if sd != swing:
                            continue
                        dt = abs(float(ts) - phase)
                        if period > 1e-9:
                            dt = min(dt, period - dt)
                        if dt < best_dt - 1e-9 or (
                            abs(dt - best_dt) <= 1e-9 and float(added) > z_add
                        ):
                            best_dt = dt
                            z_comp = float(z_val)
                            z_add = float(added)
                surf_row: dict[str, object] = {
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
                    "l_ank_pitch": float(
                        session.data.actuator_force[session.act_idx["l_ank_pitch_pos"]]
                    ),
                    "r_ank_pitch": float(
                        session.data.actuator_force[session.act_idx["r_ank_pitch_pos"]]
                    ),
                }
                if roll_log:
                    surf_row["l_hip_roll"] = float(
                        session.data.actuator_force[session.act_idx["l_hip_roll_pos"]]
                    )
                    surf_row["r_hip_roll"] = float(
                        session.data.actuator_force[session.act_idx["r_hip_roll_pos"]]
                    )
                if ank_log:
                    surf_row["l_ank_roll"] = float(
                        session.data.actuator_force[session.act_idx["l_ank_roll_pos"]]
                    )
                    surf_row["r_ank_roll"] = float(
                        session.data.actuator_force[session.act_idx["r_ank_roll_pos"]]
                    )
                if sole_corner_log and swing in ("L", "R"):
                    surf_row["sole_tick"] = _sole_tick_capture(session, swing)
                if steer_out is not None:
                    surf_row["seg"] = driver.segment(t).label
                    surf_row["yaw"] = float(lipm.cmd_yaw)
                    surf_row["phase"] = str(lipm.phase)
                    for jn in ("l_knee", "r_knee", "l_hip_roll", "r_hip_roll"):
                        damp_f, omega, damp_c, passive = _joint_damping(session, jn)
                        surf_row[jn + "_damp"] = damp_f
                        surf_row[jn + "_omega"] = omega
                        surf_row[jn + "_damp_c"] = damp_c
                        surf_row[jn + "_passive"] = passive
                if yswap_lead_s > 0.0:
                    stand_l = float(lipm.q_stand.get("l_ank_roll", 0.0))
                    stand_r = float(lipm.q_stand.get("r_ank_roll", 0.0))
                    ctrl_l = float(session.data.ctrl[session.act_idx["l_ank_roll_pos"]])
                    ctrl_r = float(session.data.ctrl[session.act_idx["r_ank_roll_pos"]])
                    split = abs((ctrl_l - stand_l) - (ctrl_r - stand_r))
                    if split > ank_split:
                        ank_split = split
                surface_rows.append(surf_row)
                if ka_log:
                    ka_checked += 1
                    for act in ka_names:
                        tau = float(session.data.actuator_force[session.act_idx[act]])
                        foot = planes["L" if act.startswith("l_") else "R"]
                        contact = str(foot["contact"])
                        if abs(tau) >= KNEE_NM:
                            ka_hits.append(
                                f"{act} {tau:+.4f} t {t:.3f} contact {contact} "
                                f"geoms {foot['geoms']} "
                                f"ge_2.33 1 ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
                            )
                for act in hx_chain:
                    tau = float(session.data.actuator_force[session.act_idx[act]])
                    foot = planes["L" if act.startswith("l_") else "R"]
                    contact = str(foot["contact"])
                    # Floor contact, and a foot touching nothing, are Track 1.
                    # A rug geom in the contact list is not called floor.
                    bucket = hx_flat if contact in ("floor", "none") else hx_rug
                    prev, _when, _contact = _hx_parts(bucket[act])
                    if abs(tau) > abs(prev):
                        bucket[act] = (tau, t, contact)
                for act in roll_chain:
                    tau = float(session.data.actuator_force[session.act_idx[act]])
                    foot = planes["L" if act.startswith("l_") else "R"]
                    contact = str(foot["contact"])
                    bucket = roll_flat if contact in ("floor", "none") else roll_rug
                    prev, _when, _contact = _hx_parts(bucket[act])
                    if abs(tau) > abs(prev):
                        bucket[act] = (tau, t, contact)
                for act in ank_chain:
                    tau = float(session.data.actuator_force[session.act_idx[act]])
                    foot = planes["L" if act.startswith("l_") else "R"]
                    contact = str(foot["contact"])
                    bucket = ank_flat if contact in ("floor", "none") else ank_rug
                    prev, _when, _contact = _hx_parts(bucket[act])
                    if abs(tau) > abs(prev):
                        bucket[act] = (tau, t, contact)
                for act in floor_acts:
                    tau = float(session.data.actuator_force[session.act_idx[act]])
                    foot = planes["L" if act.startswith("l_") else "R"]
                    contact = str(foot["contact"])
                    if contact not in ("floor", "floor+rug"):
                        continue
                    prev, _when, _contact = _hx_parts(floor_peak[act])
                    if abs(tau) > abs(prev):
                        floor_peak[act] = (tau, t, contact)
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
        bout_z, bout_t, bout_frac, bout_side, bout_n, med_n, last_n = _worst_mid_swing(
            rows, None if t_end is None else end + 1.0,
        )
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
        or world_level or toe_up_rad > 0.0 or hip_lead or sag_cancel
        or z_profile != "phase" or knee_lead_s > 0.0 or toe_up_shape != "sine"
        or yswap_lead_s > 0.0 or abs(toe_full_frac - 0.15) > 1e-9
        or abs(ank_trim_l) > 1e-12 or abs(ank_trim_r) > 1e-12
        or abs(ank_pitch_trim_l) > 1e-12 or abs(ank_pitch_trim_r) > 1e-12
        or ank_trim_lead
    ):
        parts = ["clip" if clip else "base"]
        parts.append("pitchoff" if pitch_move_off else f"move{used_move * 1000:.0f}")
        if ankle_roll:
            parts.append("ankroll")
        if world_level:
            parts.append("worldlevel")
        if toe_up_rad > 0.0:
            landed_front = (
                toe_up_shape == "front"
                and abs(toe_full_frac - 0.15) <= 1e-9
                and abs(toe_up_rad - 0.020) <= 1e-9
            )
            if landed_front:
                parts.append("toeupfront")
            elif toe_up_shape == "front":
                parts.append(
                    f"toeup{toe_up_rad * 1000:.0f}by{toe_full_frac * 100:02.0f}"
                )
            else:
                parts.append(f"toeup{toe_up_rad * 1000:.0f}mrad")
        if z_profile == "front":
            parts.append("zfront")
        elif z_profile == "early":
            parts.append(f"early{early_peak_m * 1000.0:.0f}mm")
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
        if knee_lead_s > 0.0:
            parts.append(f"kneelead{knee_lead_s * 1000.0:.0f}ms")
        if sag_cancel:
            parts.append("sag")
        if yswap_lead_s > 0.0:
            parts.append(f"yswap{yswap_lead_s * 1000:.0f}ms")
        if abs(ank_pitch_trim_l) > 1e-12 or abs(ank_pitch_trim_r) > 1e-12:
            parts.append("leveltrim")
        elif abs(ank_trim_l) > 1e-12 or abs(ank_trim_r) > 1e-12:
            parts.append("anktrim")
        if ank_trim_lead:
            parts.append("trimlead")
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
        "flat_min_fracs": (),
    }
    if surface_tag and surface_rows:
        surf = _surface_summary(
            name, rows, surface_rows, hx_flat, hx_rug,
            ka_hits if ka_log else None, ka_checked,
            None if t_end is None else end + 1.0,
        )
        if early_log:
            _print_early_flat(name, rows, surface_rows)
            _print_chain_bar(name, surface_rows)
    roll_hits: list[str] = []
    if roll_log:
        for act, packed in roll_flat.items():
            tau, when, contact = _hx_parts(packed)
            over = abs(tau) >= KNEE_NM
            if over:
                roll_hits.append(f"{act} {tau:+.4f} t {when:.3f} contact {contact}")
            print(
                f"PRED {name} hx_flat {act} {tau:+.4f} t {when:.3f} "
                f"contact {contact or '-'} "
                f"abs {abs(tau):.4f} ge_2.33 {int(over)} "
                f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
            )
        for act, packed in roll_rug.items():
            tau, when, contact = _hx_parts(packed)
            print(
                f"PRED {name} hx_rug {act} {tau:+.4f} t {when:.3f} "
                f"contact {contact or '-'} "
                f"abs {abs(tau):.4f} ge_2.33 {int(abs(tau) >= KNEE_NM)} "
                f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
            )
    if yswap_lead_s > 0.0:
        print(
            f"PRED {name} ank_pair_split {ank_split:.6f} rad "
            f"matched {int(ank_split <= 1e-4)} "
            "shift lat is the same on both ankle rolls"
        )
    ank_hits: list[str] = []
    if ank_log:
        for act, packed in ank_flat.items():
            tau, when, contact = _hx_parts(packed)
            over = abs(tau) >= KNEE_NM
            if over:
                ank_hits.append(f"{act} {tau:+.4f} t {when:.3f} contact {contact}")
            print(
                f"PRED {name} hx_flat {act} {tau:+.4f} t {when:.3f} "
                f"contact {contact or '-'} "
                f"abs {abs(tau):.4f} ge_2.33 {int(over)} "
                f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
            )
        for act, packed in ank_rug.items():
            tau, when, contact = _hx_parts(packed)
            print(
                f"PRED {name} hx_rug {act} {tau:+.4f} t {when:.3f} "
                f"contact {contact or '-'} "
                f"abs {abs(tau):.4f} ge_2.33 {int(abs(tau) >= KNEE_NM)} "
                f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)}"
            )
    sole_ticks: tuple = ()
    if sole_corner_log and surface_rows:
        sole_ticks = _print_sole_208(name, rows, surface_rows)
    klog = getattr(session, "_knee_lead_log", None)
    if klog:
        clamps = sum(int(row[4]) for row in klog)
        print(
            f"PRED {name} knee_lead_n {len(klog)} clamp {clamps} "
            "swing knee only. Stance sag cancel is off."
        )
    session.assert_plant_unchanged()
    if steer_out is not None:
        steer_out["asks"] = ask_log
        steer_out["rows"] = rows
        steer_out["surface"] = surface_rows
        steer_out["fault"] = first_fault
        steer_out["fault_t"] = fault_t
        steer_out["end"] = end
        steer_out["latches"] = ramp_latches
        steer_out["soft"] = soft_trace
        steer_out["inflight"] = inflight_trace
        steer_out["inflight_edge"] = inflight_edge
        steer_out["lateral"] = lateral_rows
        steer_out["sag_cap"] = list(getattr(session, "_sag_cap_rows", []))
        steer_out["walk_slew"] = dict(getattr(session, "_walk_slew_stats", {}))
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
        sag_rows=tuple(getattr(session, "_sag_log", ())),
        flat_min_fracs=tuple(surf.get("flat_min_fracs", ())),  # type: ignore[arg-type]
        roll_hx_over=", ".join(roll_hits),
        sole_ticks=sole_ticks,
        ank_hx_over=", ".join(ank_hits),
        floor_chain=tuple(
            (act, float(tau), float(when), contact)
            for act, (tau, when, contact) in floor_peak.items()
        ),
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


def _sag_stats(rows: tuple, t_want: float, swing_side: str) -> dict[str, float | str | int]:
    """Peak knee-z offset, and the stance knee at the flat toe tick.

    ``knee_z_mm`` is the applied offset times the pelvis-fixed foot
    jacobian. Positive raises that foot relative to the pelvis.
    """
    empty: dict[str, float | str | int] = {
        "n": 0,
        "clamp_n": 0,
        "peak_mm": float("nan"),
        "peak_raw_mm": float("nan"),
        "peak_t": float("nan"),
        "peak_joint": "",
        "toe_mm": float("nan"),
        "toe_raw_mm": float("nan"),
        "toe_rad": float("nan"),
        "toe_joint": "",
        "toe_bias": float("nan"),
        "toe_con": float("nan"),
        "toe_pred": float("nan"),
        "p50_mm": float("nan"),
        "p90_mm": float("nan"),
    }
    if not rows:
        return empty
    stance = "l_knee" if swing_side == "R" else "r_knee" if swing_side == "L" else ""
    abs_mm = [abs(float(row[6])) for row in rows]
    abs_mm.sort()
    peak = max(rows, key=lambda row: abs(float(row[6])))
    near = [
        row for row in rows
        if abs(float(row[0]) - t_want) <= 0.004 and (not stance or row[1] == stance)
    ]
    if not near:
        near = [row for row in rows if abs(float(row[0]) - t_want) <= 0.004]
    toe = near[0] if near else None

    def _pct(frac: float) -> float:
        if not abs_mm:
            return float("nan")
        i = min(len(abs_mm) - 1, max(0, int(round(frac * (len(abs_mm) - 1)))))
        return abs_mm[i]

    out = dict(empty)
    out.update({
        "n": len(rows),
        "clamp_n": sum(int(row[11]) for row in rows),
        "peak_mm": float(peak[6]),
        "peak_raw_mm": float(peak[5]),
        "peak_t": float(peak[0]),
        "peak_joint": str(peak[1]),
        "p50_mm": _pct(0.50),
        "p90_mm": _pct(0.90),
    })
    if toe is not None:
        out.update({
            "toe_mm": float(toe[5]),
            "toe_raw_mm": float(toe[6]),
            "toe_rad": float(toe[3]),
            "toe_joint": str(toe[1]),
            "toe_bias": float(toe[8]),
            "toe_con": float(toe[9]),
            "toe_pred": float(toe[7]),
        })
    return out


def _print_sag(score: PredScore, sag_ref_mm: float = 0.927) -> None:
    stats = _sag_stats(score.sag_rows, score.flat_toe_t, score.flat_toe_side)
    print(
        f"PRED sag_mm {score.name} n {stats['n']} clamp {stats['clamp_n']} "
        f"formula_peak {float(stats['peak_mm']):+.3f} mm "
        f"applied_at_peak {float(stats['peak_raw_mm']):+.3f} mm "
        f"t {float(stats['peak_t']):.3f} {stats['peak_joint']} "
        f"formula_p50 {float(stats['p50_mm']):.3f} mm "
        f"formula_p90 {float(stats['p90_mm']):.3f} mm "
        f"at_flat_toe formula {float(stats['toe_raw_mm']):+.3f} mm "
        f"applied {float(stats['toe_mm']):+.3f} mm "
        f"applied_rad {float(stats['toe_rad']):+.5f} "
        f"{stats['toe_joint']} "
        f"bias {float(stats['toe_bias']):+.4f} "
        f"constraint {float(stats['toe_con']):+.4f} "
        f"pred {float(stats['toe_pred']):+.4f} "
        f"beside_sag_share {sag_ref_mm:.3f} mm"
    )


def score_stance_sag() -> None:
    """Stance-knee sag cancel on the 20 ms base.

    Over-2.33 ticks are tagged by the contact geom. Floor contact is
    Track 1, so the foot-z chase stops at +0.849 mm. This copy adds the
    previous tick's ``(qfrc_bias − qfrc_constraint) / kp`` on the stance
    knee and clamps the predicted force to ±2.33 Nm. A +0.849 mm foot-z
    add runs only when the zero-add copy holds that bar and the flat toe
    is still under +2 mm.
    """
    print(
        "PRED sag_plan Contact geom tags every over-2.33 tick. "
        "Floor contact is Track 1. "
        "The +1.249 mm add put the right knee on +2.450 Nm at 7.432 s "
        "with contact geom floor, so the foot-z chase stops at +0.849 mm. "
        "Sag cancel reads the previous tick. "
        "Offset is (qfrc_bias - qfrc_constraint) / compiled kp. "
        "Predicted force stays inside ±2.33 Nm."
    )
    base = _pitch20_copy(0.0)
    print(
        f"PRED sag_base {base.name} flat_toe {base.flat_toe_mm:.3f} mm "
        f"t {base.flat_toe_t:.3f} side {base.flat_toe_side} "
        f"box_centre {base.flat_center_mm:.3f} mm "
        f"sole_pitch {base.flat_sole_pitch:.5f} rad "
        f"flat_hx {base.flat_hx_over or 'under'} "
        f"ka_over {base.ka_over_n} ka_checked {base.ka_checked}"
    )
    row = _pitch20_sag(0.0)
    _report_sag_copy(base, row, 0.0)
    holds = row.flat_hx_over == ""
    if not holds:
        print(
            "PRED sag Prefer FAIL. Floor-contact hip pitch, knee, or ankle "
            "pitch is over 2.33 Nm. The +0.849 mm foot-z add was not run. "
            "Pitch lead stays 20 ms. Toe-up stays 0.020 rad. Period stays 0.500 s."
        )
        return
    if row.flat_toe_mm >= 2.0:
        print(
            f"PRED sag clear {row.name} flat_toe {row.flat_toe_mm:.3f} mm "
            f"t {row.flat_toe_t:.3f} side {row.flat_toe_side}. "
            "Pitch lead stays 20 ms. Toe-up stays 0.020 rad. Period stays 0.500 s."
        )
        return
    extra = _pitch20_sag(0.849 / 1000.0)
    _report_sag_copy(base, extra, 0.849)
    holds_z = extra.flat_hx_over == ""
    if holds_z and extra.flat_toe_mm >= 2.0:
        print(
            f"PRED sag clear {extra.name} flat_toe {extra.flat_toe_mm:.3f} mm "
            f"t {extra.flat_toe_t:.3f} side {extra.flat_toe_side}. "
            "Pitch lead stays 20 ms. Toe-up stays 0.020 rad. Period stays 0.500 s."
        )
        return
    print(
        "PRED sag Prefer FAIL. Stance-knee sag cancel did not clear a flat "
        "mid-swing contact-box toe of +2 mm with hip pitch, knee, and ankle "
        "pitch at or under 2.33 Nm on floor contact. "
        "Pitch lead stays 20 ms. Toe-up stays 0.020 rad. Period stays 0.500 s."
    )


def _pitch20_sag(z_extra_m: float) -> PredScore:
    """20 ms pitch lead plus stance-knee sag cancel. Roll lead stays 0."""
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
        sag_cancel=True,
    )


def _report_sag_copy(base: PredScore, row: PredScore, mm: float) -> None:
    same = _mid_at(base.mid_samples, row.flat_toe_t, row.flat_toe_side)
    if same is None:
        d_toe = float("nan")
        d_centre = float("nan")
        d_body = float("nan")
    else:
        d_toe = row.flat_toe_mm - float(same[2])
        d_centre = row.flat_center_mm - float(same[3])
        d_body = row.flat_body_mm - float(same[4])
    holds = row.flat_hx_over == ""
    clear = holds and row.flat_toe_mm >= 2.0
    print(
        f"PRED sag_copy {mm:.3f} mm {row.name} "
        f"flat_toe {row.flat_toe_mm:.3f} mm t {row.flat_toe_t:.3f} "
        f"side {row.flat_toe_side} "
        f"box_centre {row.flat_center_mm:.3f} mm "
        f"sole_pitch {row.flat_sole_pitch:.5f} rad "
        f"d_toe {d_toe:.3f} mm d_centre {d_centre:.3f} mm "
        f"d_body {d_body:.3f} mm "
        f"flat_hx {row.flat_hx_over or 'under'} "
        f"holds_flat {int(holds)} clear {int(clear)} "
        f"ka_over {row.ka_over_n} ka_checked {row.ka_checked}"
    )
    _print_sag(row)


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
        shortfall = mm - d_centre
        crouch = -d_body if d_body == d_body and d_body < 0.0 else 0.0
        sag = shortfall - crouch
        if shortfall <= 0.3 * mm:
            split = "cmd"
        elif crouch >= sag:
            split = "crouch"
        else:
            split = "sag"
        print(
            f"PRED zlift_split {mm:.3f} mm {split} "
            f"cmd_add {mm:.3f} mm world_centre_delta {d_centre:.3f} mm "
            f"body_delta {d_body:.3f} mm toe_delta {d_toe:.3f} mm "
            f"shortfall {shortfall:.3f} mm crouch {crouch:.3f} mm sag {sag:.3f} mm"
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


def _liftoff_copy(
    *,
    toe_shape: str = "sine",
    z_profile: str = "phase",
    early_peak_m: float = 0.0,
    knee_lead_s: float = 0.0,
) -> PredScore:
    """20 ms pitch lead, roll lead 0, clip on, sag cancel off."""
    return measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=TOE_UP_PEAK,
        toe_up_shape=toe_shape,
        z_profile=z_profile,
        early_peak_m=early_peak_m,
        knee_lead_s=knee_lead_s,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        whole_toe=True,
        surface_tag=True,
        ka_log=True,
        early_log=True,
        sag_cancel=False,
    )


def _liftoff_clears(row: PredScore) -> bool:
    return row.flat_hx_over == "" and row.flat_toe_mm >= 2.0


def _report_liftoff(label: str, row: PredScore) -> None:
    knee_rail = "knee" in row.flat_hx_over
    print(
        f"PRED liftoff {label} {row.name} "
        f"flat_worst {row.flat_toe_mm:.3f} mm t {row.flat_toe_t:.3f} "
        f"side {row.flat_toe_side} vs_plus2 {row.flat_toe_mm - 2.0:+.3f} mm "
        f"short {2.0 - row.flat_toe_mm:.3f} mm "
        f"box_centre {row.flat_center_mm:.3f} mm "
        f"sole_pitch {row.flat_sole_pitch:.5f} rad "
        f"flat_hx {row.flat_hx_over or 'under'} "
        f"knee_rail {int(knee_rail)} "
        f"clear {int(_liftoff_clears(row))} "
        f"ka_over {row.ka_over_n} ka_checked {row.ka_checked}"
    )


def score_liftoff() -> None:
    """Front-load lift-off on the 20 ms base. Sag cancel stays off.

    The 20–80% window is unchanged. Floor-contact overs are Track 1.
    """
    print(
        "PRED liftoff_plan Sag cancel stays off. "
        "Worst toe has been the first 20-80% tick. "
        "A ramps toe-up to 0.020 rad by 15% and holds through 80%. "
        "B brings the 12 mm foot-z to full by 10% of single support and holds through 40%. "
        "C is A and B. D, if A-C miss +2 mm, leads the swing knee by 1 and 2 ticks. "
        "E replaces the 12 mm parabola with an 8 mm and a 10 mm rise, full by 10%, "
        "on the current swing-hip clip and 20 ms pitch lead. "
        "Scoring window stays 20-80%. Roll lead stays 0. Period stays 0.500 s."
    )
    cleared: list[tuple[str, PredScore]] = []
    ran: list[tuple[str, PredScore]] = []
    for label, kwargs in (
        ("A", {"toe_shape": "front"}),
        ("B", {"z_profile": "front"}),
        ("C", {"toe_shape": "front", "z_profile": "front"}),
    ):
        row = _liftoff_copy(**kwargs)
        _report_liftoff(label, row)
        ran.append((label, row))
        if _liftoff_clears(row):
            cleared.append((label, row))
    if cleared:
        print(
            "PRED liftoff D skipped. A, B, or C already cleared +2 mm "
            "with floor-contact hip pitch, knee, and ankle pitch at or under 2.33 Nm."
        )
    else:
        for label, lead in (("D8", 0.008), ("D16", 0.016)):
            row = _liftoff_copy(knee_lead_s=lead)
            _report_liftoff(label, row)
            ran.append((label, row))
            if _liftoff_clears(row):
                cleared.append((label, row))
    for label, peak in (("E8", 0.008), ("E10", 0.010)):
        row = _liftoff_copy(z_profile="early", early_peak_m=peak)
        _report_liftoff(label, row)
        ran.append((label, row))
        if _liftoff_clears(row):
            cleared.append((label, row))
        knee_hits = [
            bit for bit in row.flat_hx_over.split(", ") if "knee" in bit
        ] if row.flat_hx_over else []
        if knee_hits:
            print(
                f"PRED liftoff {label} knee_rail_floor {' ; '.join(knee_hits)}. "
                "That rail is floor contact, Track 1. "
                "The earlier -1.60 mm / +2.45 Nm copy was before this clip and this 20 ms lead."
            )
    digest = sw._md5(sw.PLANT_XML)
    print(f"PRED liftoff plant_md5 {digest}")
    if digest != sw.PLANT_MD5:
        raise SystemExit(f"plant md5 {digest} != {sw.PLANT_MD5}")
    bits = " ".join(
        f"{label} {row.flat_toe_mm:+.3f} mm"
        for label, row in ran
    )
    if cleared:
        best = max(cleared, key=lambda item: item[1].flat_toe_mm)
        print(
            f"PRED liftoff clear {best[0]} {best[1].name} "
            f"flat_worst {best[1].flat_toe_mm:.3f} mm "
            f"t {best[1].flat_toe_t:.3f} side {best[1].flat_toe_side}. "
            f"copies {bits}. "
            "Pitch lead stays 20 ms. Period stays 0.500 s."
        )
        return
    print(
        "PRED liftoff Prefer FAIL. Front-loaded toe-up, early foot-z, "
        "the swing-knee lead, and the 8-10 mm rise did not clear a flat "
        "mid-swing contact-box toe of +2 mm with hip pitch, knee, and "
        "ankle pitch at or under 2.33 Nm on floor contact. "
        f"copies {bits}. "
        "The 20-80% window was not moved. Sag cancel stayed off. "
        "Pitch lead stays 20 ms. Toe-up peak stays 0.020 rad. Period stays 0.500 s."
    )


def _toe_sooner_copy(
    peak: float,
    full_frac: float,
    yswap_lead_s: float = 0.0,
) -> PredScore:
    """20 ms pitch lead, roll lead 0, clip on, sag cancel off, phase lift."""
    return measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=peak,
        toe_up_shape="front",
        toe_full_frac=full_frac,
        yswap_lead_s=yswap_lead_s,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        whole_toe=True,
        surface_tag=True,
        ka_log=True,
        early_log=True,
        roll_log=yswap_lead_s > 0.0,
        sag_cancel=False,
        z_profile="phase",
    )


def _mins_at_208(row: PredScore) -> bool:
    fracs = tuple(row.flat_min_fracs)
    if not fracs:
        return False
    return all(abs(float(frac) - (5.0 / 24.0)) <= 0.01 for frac in fracs)


def _toe_chain_clear(row: PredScore) -> bool:
    return row.flat_hx_over == "" and row.flat_toe_mm >= 2.0


def _yswap_clear(row: PredScore) -> bool:
    return _toe_chain_clear(row) and row.roll_hx_over == ""


def _report_toe_sooner(label: str, row: PredScore) -> None:
    knee_rail = "knee" in row.flat_hx_over
    print(
        f"PRED toe_sooner {label} {row.name} "
        f"flat_worst {row.flat_toe_mm:.3f} mm t {row.flat_toe_t:.3f} "
        f"side {row.flat_toe_side} vs_plus2 {row.flat_toe_mm - 2.0:+.3f} mm "
        f"short {2.0 - row.flat_toe_mm:.3f} mm "
        f"box_centre {row.flat_center_mm:.3f} mm "
        f"sole_pitch {row.flat_sole_pitch:.5f} rad "
        f"flat_hx {row.flat_hx_over or 'under'} "
        f"roll_hx {row.roll_hx_over or 'under'} "
        f"mins_at_0.208 {int(_mins_at_208(row))} "
        f"flat_min_n {len(row.flat_min_fracs)} "
        f"knee_rail {int(knee_rail)} "
        f"clear {int(_yswap_clear(row) if row.roll_hx_over or 'yswap' in row.name else _toe_chain_clear(row))} "
        f"ka_over {row.ka_over_n} ka_checked {row.ka_checked}"
    )


def score_toe_sooner() -> None:
    """Earlier front toe-up, then earlier y_swap if the mins stay at 0.208.

    The 20–80% window is unchanged. Sag cancel stays off. Soft-pass stays off.
    Contact settings and the 2.33 Nm bar stay put. Plant stays cold.
    """
    print(
        "PRED toe_sooner_plan Sag cancel stays off. "
        "Base is the 20 ms pitch lead, roll lead 0, period 0.500 s, "
        "the 12 mm phase lift, and the swing-hip clip. "
        "A10 is toe-up 0.020 rad full by 10% and held through 80%. "
        "A05 is the same peak full by 5%. "
        "A15+025 is 0.025 rad full by 15%. "
        "A10+025 is 0.025 rad full by 10%. "
        "Each 0-30% tick of a flat swing logs the swing-foot normal and contact dist. "
        "Scoring window stays 20-80%. "
        "If those copies stay short of +2 mm under 2.33 Nm and every flat minimum "
        "is still fraction 0.208, y_swap is sampled 8 ms and 16 ms earlier "
        "on the best under-bar shape. Amplitude stays 0.020 m. "
        "Both ankle rolls keep the same shift lat."
    )
    copies = (
        ("A10", 0.020, 0.10),
        ("A05", 0.020, 0.05),
        ("A15+025", 0.025, 0.15),
        ("A10+025", 0.025, 0.10),
    )
    ran: list[tuple[str, float, float, PredScore]] = []
    cleared: list[tuple[str, PredScore]] = []
    for label, peak, full in copies:
        row = _toe_sooner_copy(peak, full)
        _report_toe_sooner(label, row)
        ran.append((label, peak, full, row))
        if _toe_chain_clear(row):
            cleared.append((label, row))
    digest = sw._md5(sw.PLANT_XML)
    print(f"PRED toe_sooner plant_md5 {digest}")
    if digest != sw.PLANT_MD5:
        raise SystemExit(f"plant md5 {digest} != {sw.PLANT_MD5}")
    bits = " ".join(
        f"{label} {row.flat_toe_mm:+.3f} mm" for label, _peak, _full, row in ran
    )
    if cleared:
        best = max(cleared, key=lambda item: item[1].flat_toe_mm)
        print(
            f"PRED toe_sooner clear {best[0]} {best[1].name} "
            f"flat_worst {best[1].flat_toe_mm:.3f} mm "
            f"t {best[1].flat_toe_t:.3f} side {best[1].flat_toe_side}. "
            f"copies {bits}. "
            "y_swap lead was not run. Pitch lead stays 20 ms. Period stays 0.500 s."
        )
        return
    under = [item for item in ran if item[3].flat_hx_over == ""]
    if not under:
        print(
            "PRED toe_sooner Prefer FAIL. Every earlier toe-up copy puts a "
            "floor-contact pitch-chain actuator over 2.33 Nm. "
            f"copies {bits}. "
            "y_swap lead was not run. The 20-80% window was not moved. "
            "Pitch lead stays 20 ms. Period stays 0.500 s."
        )
        return
    best_label, best_peak, best_full, best_row = max(under, key=lambda item: item[3].flat_toe_mm)
    stalled = best_row.flat_toe_mm < 2.0 and all(_mins_at_208(item[3]) for item in under)
    if not stalled:
        print(
            "PRED toe_sooner Prefer FAIL. Earlier toe-up is still short of +2 mm "
            "under 2.33 Nm, and a flat minimum left fraction 0.208. "
            f"best {best_label} {best_row.flat_toe_mm:+.3f} mm. copies {bits}. "
            "y_swap lead is not the next copy. The 20-80% window was not moved. "
            "Pitch lead stays 20 ms. Period stays 0.500 s."
        )
        return
    print(
        f"PRED toe_sooner stall mins_at_0.208 1 best {best_label} "
        f"{best_row.flat_toe_mm:+.3f} mm under 2.33 Nm. "
        f"Running yswap 8 ms and 16 ms on peak {best_peak:.3f} rad "
        f"full by {best_full * 100:.0f}%. Amplitude stays 0.020 m."
    )
    y_ran: list[tuple[str, PredScore]] = []
    y_cleared: list[tuple[str, PredScore]] = []
    for label, lead in ((f"{best_label}+Y8", 0.008), (f"{best_label}+Y16", 0.016)):
        row = _toe_sooner_copy(best_peak, best_full, yswap_lead_s=lead)
        _report_toe_sooner(label, row)
        y_ran.append((label, row))
        if _yswap_clear(row):
            y_cleared.append((label, row))
    digest = sw._md5(sw.PLANT_XML)
    print(f"PRED toe_sooner plant_md5 {digest}")
    if digest != sw.PLANT_MD5:
        raise SystemExit(f"plant md5 {digest} != {sw.PLANT_MD5}")
    ybits = " ".join(f"{label} {row.flat_toe_mm:+.3f} mm" for label, row in y_ran)
    if y_cleared:
        ybest = max(y_cleared, key=lambda item: item[1].flat_toe_mm)
        print(
            f"PRED toe_sooner clear {ybest[0]} {ybest[1].name} "
            f"flat_worst {ybest[1].flat_toe_mm:.3f} mm "
            f"t {ybest[1].flat_toe_t:.3f} side {ybest[1].flat_toe_side}. "
            f"toe copies {bits}. yswap {ybits}. "
            "Hip rolls on floor contact stay at or under 2.33 Nm. "
            "Ankle-roll shift lat stays matched. Pitch lead stays 20 ms."
        )
        return
    print(
        "PRED toe_sooner Prefer FAIL. Earlier toe-up and an earlier y_swap "
        "did not clear a flat mid-swing contact-box toe of +2 mm with the "
        "pitch chain and both hip rolls at or under 2.33 Nm on floor contact. "
        f"toe copies {bits}. yswap {ybits}. "
        "The 20-80% window was not moved. Sag cancel stayed off. "
        "y_swap amplitude stayed 0.020 m. Pitch lead stays 20 ms. Period stays 0.500 s."
    )


_CORNER_ORDER = ("front-outside", "front-inside", "heel-outside", "heel-inside")


def _sole_tick_capture(session: sw.SteerSession, side: str) -> dict[str, object]:
    """Corners, sole roll and pitch, ankle ctrl, and the pose at this swing tick."""
    corners = _box_corner_table(session, side)
    roll, drop, half, inside = _sole_roll(session, side)
    center_z = float(sum(z for _label, z, _off in corners) / len(corners))
    scored = max(corners, key=lambda row: row[2])
    lowest = min(corners, key=lambda row: row[1])
    fo = next((float(z) for label, z, _off in corners if label == "front-outside"), None)
    if fo is None:
        raise SystemExit("front-outside corner missing")
    pref = "l_" if side == "L" else "r_"
    gait_frac = float("nan")
    lipm = session.lipm
    if lipm is not None and lipm.op3 is not None:
        got = _mid_frac(lipm.op3, side, _cmd_time(lipm.op3))
        if got is not None:
            gait_frac = float(got)
    return {
        "roll": float(roll),
        "pitch": float(_sole_pitch(session, side)),
        "inside_drop": float(drop),
        "inside": inside,
        "half": float(half),
        "center_z": center_z,
        "corners": tuple((str(label), float(z), float(off)) for label, z, off in corners),
        "scored_label": str(scored[0]),
        "scored_z": float(scored[1]),
        "low_label": str(lowest[0]),
        "low_z": float(lowest[1]),
        "front_outside_z": float(fo),
        "drop_fo_m": center_z - float(fo),
        "ank_ctrl": float(session.data.ctrl[session.act_idx[pref + "ank_roll_pos"]]),
        "ank_pitch_ctrl": float(session.data.ctrl[session.act_idx[pref + "ank_pitch_pos"]]),
        "qpos": np.array(session.data.qpos, dtype=np.float64, copy=True),
        "gait_frac": gait_frac,
    }


def _emit_sole_208(name: str, tag: str, pack: dict[str, object]) -> None:
    corners = _corner_map(pack["corners"])
    centre = float(pack["center_z"])
    print(
        f"PRED {name} {tag} side {pack['side']} t {float(pack['t']):.3f} "
        f"frac {float(pack['frac']):.3f} "
        f"gait_frac {float(pack.get('gait_frac', float('nan'))):.6f} "
        f"sole_roll {float(pack['roll']):+.5f} rad "
        f"sole_pitch {float(pack['pitch']):+.5f} rad "
        f"inside {pack['inside']} "
        f"inside_drop {float(pack['inside_drop']) * 1000:+.3f} mm "
        f"centre {centre * 1000:+.3f} mm "
        f"tick_toe {float(pack['toe_z']) * 1000:+.3f} mm "
        f"swing_min_toe {float(pack['swing_min_toe']) * 1000:+.3f} mm "
        f"swing_min_frac {float(pack['swing_min_frac']):.3f} "
        f"ank_ctrl {float(pack['ank_ctrl']):+.5f} rad "
        f"ank_pitch_ctrl {float(pack['ank_pitch_ctrl']):+.5f} rad"
    )
    for label in _CORNER_ORDER:
        z, off = corners[label]
        drop_mm = (centre - z) * 1000.0
        print(
            f"PRED {name} {tag} corner {label} z {z * 1000:+.3f} mm "
            f"centre_minus_corner {drop_mm:+.3f} mm "
            f"forward {off * 1000:.1f} mm "
            f"scored {int(label == pack['scored_label'])} "
            f"lowest {int(label == pack['low_label'])}"
        )
    drop_mm = float(pack["drop_fo_m"]) * 1000.0
    about = int(SOLE_FO_LO_MM - 1e-9 <= drop_mm <= SOLE_FO_HI_MM + 1e-9)
    print(
        f"PRED {name} {tag} scored {pack['scored_label']} "
        f"lowest {pack['low_label']} "
        f"centre_minus_front_outside {drop_mm:+.3f} mm "
        f"about_0.6 {about} window_mm {SOLE_FO_LO_MM:.2f} {SOLE_FO_HI_MM:.2f}"
    )


def _corner_map(corners: object) -> dict[str, tuple[float, float]]:
    out: dict[str, tuple[float, float]] = {}
    if not isinstance(corners, tuple):
        raise SystemExit("sole corners missing")
    for row in corners:
        label, z, off = row  # type: ignore[misc]
        out[str(label)] = (float(z), float(off))
    for label in _CORNER_ORDER:
        if label not in out:
            raise SystemExit(f"sole corner {label} missing")
    return out


def _print_sole_208(
    name: str,
    rows: list[Tick],
    surface_rows: list[dict[str, object]],
) -> tuple[dict[str, object], ...]:
    """First scored tick of the worst flat swing, and each foot's own worst."""
    by_t = {round(float(row["t"]), 5): row for row in surface_rows}
    cycles = _swing_cycles(rows, T_END + 1.0)
    med_n, _last_n = _cycle_index(cycles)
    target = 5.0 / 24.0
    foot_best: dict[str, dict[str, object]] = {}
    for _step, cyc in enumerate(cycles):
        n = len(cyc)
        if n < 2 or med_n < 2:
            continue
        n_full = med_n if _step == len(cycles) - 1 and n < med_n - 1 else n
        window: list[tuple[float, Tick, dict[str, object]]] = []
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
            sole = surf.get("sole_tick")
            if not isinstance(plane, dict) or not isinstance(sole, dict):
                continue
            window.append((frac, row, sole))
        if not window:
            continue
        tagged = []
        rug = False
        for frac, row, _sole in window:
            surf = by_t.get(round(row.t, 5))
            if surf is None or row.swing is None:
                continue
            plane = surf[row.swing]
            if isinstance(plane, dict) and int(plane["on_rug"]):
                rug = True
            tagged.append((frac, row))
        if rug or not tagged:
            continue
        worst = min(window, key=lambda item: float(item[1].toe_z or 0.0))
        tick = min(window, key=lambda item: (abs(item[0] - target), item[0]))
        frac, row, sole = tick
        side = str(row.swing or "")
        pack: dict[str, object] = {
            "side": side,
            "t": float(row.t),
            "frac": float(frac),
            "toe_z": float(row.toe_z or 0.0),
            "swing_min_toe": float(worst[1].toe_z or 0.0),
            "swing_min_frac": float(worst[0]),
            "roll": float(sole["roll"]),
            "pitch": float(sole["pitch"]),
            "inside_drop": float(sole["inside_drop"]),
            "inside": str(sole["inside"]),
            "center_z": float(sole["center_z"]),
            "corners": sole["corners"],
            "scored_label": str(sole["scored_label"]),
            "low_label": str(sole["low_label"]),
            "front_outside_z": float(sole["front_outside_z"]),
            "drop_fo_m": float(sole["drop_fo_m"]),
            "ank_ctrl": float(sole["ank_ctrl"]),
            "ank_pitch_ctrl": float(sole["ank_pitch_ctrl"]),
            "qpos": sole["qpos"],
            "gait_frac": float(sole.get("gait_frac", float("nan"))),
            "is_worst": 0,
        }
        prev = foot_best.get(side)
        if prev is None or float(pack["swing_min_toe"]) < float(prev["swing_min_toe"]):
            foot_best[side] = pack
    if not foot_best:
        raise SystemExit(f"{name} sole_208 missed every flat swing")
    worst_pack = min(foot_best.values(), key=lambda item: float(item["swing_min_toe"]))
    worst_pack["is_worst"] = 1
    _emit_sole_208(name, "sole_208", worst_pack)
    for side in ("L", "R"):
        pack = foot_best.get(side)
        if pack is None:
            print(f"PRED {name} sole_208_foot side {side} missed")
            continue
        _emit_sole_208(name, "sole_208_foot", pack)
    return tuple(foot_best[side] for side in ("L", "R") if side in foot_best)


def _sole_view(model: mj.MjModel, data: mj.MjData) -> SimpleNamespace:
    return SimpleNamespace(
        model=model,
        data=data,
        bid_body=mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link"),
        bid_lf=mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "l_ank_roll_link"),
        bid_rf=mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "r_ank_roll_link"),
        gid_lfoot=mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "l_foot_contact"),
        gid_rfoot=mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "r_foot_contact"),
    )


def _ank_roll_adr(model: mj.MjModel, side: str) -> int:
    jn = ("l_" if side == "L" else "r_") + "ank_roll"
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
    if jid < 0:
        raise SystemExit(f"missing joint {jn}")
    return int(model.jnt_qposadr[jid])


def _pose_sole(
    model: mj.MjModel,
    data: mj.MjData,
    view: SimpleNamespace,
    qpos: np.ndarray,
    adr: int,
    side: str,
    add: float,
) -> dict[str, object]:
    data.qpos[:] = qpos
    data.qvel[:] = 0.0
    data.qpos[adr] = float(qpos[adr]) + float(add)
    mj.mj_forward(model, data)
    roll, inside_drop, _half, inside = _sole_roll(view, side)
    corners = tuple(
        (str(label), float(z), float(off))
        for label, z, off in _box_corner_table(view, side)
    )
    centre = float(sum(z for _label, z, _off in corners) / len(corners))
    fo = next(z for label, z, _off in corners if label == "front-outside")
    return {
        "roll": float(roll),
        "inside_drop": float(inside_drop),
        "inside": inside,
        "center_z": centre,
        "front_outside_z": float(fo),
        "drop_fo_m": centre - float(fo),
        "corners": corners,
    }


def _sign_ok(base: dict[str, object], after: dict[str, object]) -> bool:
    """|sole roll| must fall, and a low front-outside corner must not drop further."""
    if abs(float(after["roll"])) > abs(float(base["roll"])) + 1e-5:
        return False
    if (
        float(base["drop_fo_m"]) > 0.0
        and float(after["drop_fo_m"]) > float(base["drop_fo_m"]) + 5e-5
    ):
        return False
    return True


def _print_sign(
    tag: str,
    side: str,
    t: float,
    bout_roll: float,
    base: dict[str, object],
    after: dict[str, object],
    d_per: float,
    raw: float,
    add: float,
    flipped: int,
    ank_ctrl: float,
) -> None:
    mismatch = abs(float(base["roll"]) - bout_roll)
    print(
        f"PRED sole_sign {tag} side {side} t {t:.3f} "
        f"bout_roll {bout_roll:+.5f} rad replay_roll {float(base['roll']):+.5f} rad "
        f"replay_mismatch {mismatch:.5f} rad "
        f"droll_per_rad {d_per:+.5f} raw {raw:+.5f} add {add:+.5f} rad "
        f"flipped {flipped} cap {ANK_TRIM_CAP:.3f} "
        f"roll_after {float(after['roll']):+.5f} rad "
        f"inside {base['inside']} "
        f"inside_drop {float(base['inside_drop']) * 1000:+.3f} "
        f"to {float(after['inside_drop']) * 1000:+.3f} mm "
        f"inside_lowered {int(float(after['inside_drop']) > float(base['inside_drop']) + 1e-6)} "
        f"centre_minus_front_outside {float(base['drop_fo_m']) * 1000:+.3f} "
        f"to {float(after['drop_fo_m']) * 1000:+.3f} mm "
        f"ank_ctrl {ank_ctrl:+.5f} target {ank_ctrl + add:+.5f} "
        "not_world_level 1"
    )
    corners = _corner_map(after["corners"])
    centre = float(after["center_z"])
    for label in _CORNER_ORDER:
        z, _off = corners[label]
        print(
            f"PRED sole_sign {tag} corner {side} {label} "
            f"z {z * 1000:+.3f} mm "
            f"centre_minus_corner {(centre - z) * 1000:+.3f} mm"
        )


def _clip_trim(add: float) -> float:
    return max(-ANK_TRIM_CAP, min(ANK_TRIM_CAP, float(add)))


def _choose_add(
    model: mj.MjModel,
    data: mj.MjData,
    view: SimpleNamespace,
    tick: dict[str, object],
) -> dict[str, object] | None:
    """Joint add that cancels leftover sole roll. None means do not run the trim."""
    side = str(tick["side"])
    qpos = np.asarray(tick["qpos"], dtype=np.float64)
    adr = _ank_roll_adr(model, side)
    base = _pose_sole(model, data, view, qpos, adr, side, 0.0)
    nudged = _pose_sole(model, data, view, qpos, adr, side, 0.01)
    d_per = (float(nudged["roll"]) - float(base["roll"])) / 0.01
    bout_roll = float(tick["roll"])
    mismatch = abs(float(base["roll"]) - bout_roll)
    if mismatch > 5e-3:
        print(
            f"PRED sole_sign side {side} replay_mismatch {mismatch:.5f} rad "
            "exceeds 0.005. Trim copy was not run."
        )
        return None
    if abs(d_per) < 1e-4:
        print(
            f"PRED sole_sign side {side} droll_per_rad {d_per:+.5f} "
            "does not move sole roll. Trim copy was not run."
        )
        return None
    raw = -float(base["roll"]) / d_per
    add = _clip_trim(raw)
    after = _pose_sole(model, data, view, qpos, adr, side, add)
    flipped = 0
    if not _sign_ok(base, after):
        add = _clip_trim(-add)
        after = _pose_sole(model, data, view, qpos, adr, side, add)
        flipped = 1
        if not _sign_ok(base, after):
            print(
                f"PRED sole_sign side {side} both signs raise |sole roll| "
                "or the front-outside drop. Trim copy was not run."
            )
            _print_sign(
                "reject", side, float(tick["t"]), bout_roll, base, after,
                d_per, raw, add, flipped, float(tick["ank_ctrl"]),
            )
            return None
    _print_sign(
        "tick", side, float(tick["t"]), bout_roll, base, after,
        d_per, raw, add, flipped, float(tick["ank_ctrl"]),
    )
    return {
        "side": side,
        "t": float(tick["t"]),
        "bout_roll": bout_roll,
        "ank_ctrl": float(tick["ank_ctrl"]),
        "qpos": qpos,
        "adr": adr,
        "base": base,
        "after": after,
        "d_per": d_per,
        "raw": raw,
        "add": add,
        "flipped": flipped,
    }


def _match_mirror(
    model: mj.MjModel,
    data: mj.MjData,
    view: SimpleNamespace,
    chosen: dict[str, dict[str, object]],
    worst_side: str,
) -> dict[str, float]:
    """Opposite signs and the worst foot's magnitude, unless that raises |sole roll|."""
    left = float(chosen["L"]["add"])
    right = float(chosen["R"]["add"])
    if left * right >= 0.0:
        print(
            f"PRED sole_sign pair_mirror 0 L {left:+.5f} R {right:+.5f}. "
            "A mirrored sign raises |sole roll| on one foot. "
            "Adds stay the probed signs."
        )
        return {"L": left, "R": right}
    mag = abs(float(chosen[worst_side]["add"]))
    out: dict[str, float] = {}
    for side in ("L", "R"):
        own = float(chosen[side]["add"])
        trial = _clip_trim(math.copysign(mag, own))
        if abs(trial - own) <= 1e-9:
            out[side] = own
            continue
        qpos = np.asarray(chosen[side]["qpos"], dtype=np.float64)
        after = _pose_sole(
            model, data, view, qpos, int(chosen[side]["adr"]), side, trial,
        )
        base = chosen[side]["base"]
        if not isinstance(base, dict) or not _sign_ok(base, after):
            out[side] = own
            print(
                f"PRED sole_sign pair_mag_keep side {side} "
                f"trial {trial:+.5f} own {own:+.5f}. "
                "Matching the worst-foot magnitude raises |sole roll| "
                "or the front-outside drop."
            )
            continue
        out[side] = trial
        _print_sign(
            "mirror", side, float(chosen[side]["t"]), float(chosen[side]["bout_roll"]),
            base, after, float(chosen[side]["d_per"]), float(chosen[side]["raw"]),
            trial, int(chosen[side]["flipped"]), float(chosen[side]["ank_ctrl"]),
        )
    matched = int(abs(abs(out["L"]) - abs(out["R"])) <= 1e-9)
    print(
        f"PRED sole_sign pair_mirror 1 mag {mag:.5f} "
        f"L {out['L']:+.5f} R {out['R']:+.5f} "
        f"magnitudes_matched {matched} worst_side {worst_side}"
    )
    return out


def _require_plant(tag: str, label: str = "sole_level") -> None:
    digest = sw._md5(sw.PLANT_XML)
    print(f"PRED {label} plant_md5 {digest} {tag}")
    if digest != sw.PLANT_MD5:
        raise SystemExit(f"plant md5 {digest} != {sw.PLANT_MD5}")


def _sole_level_copy(ank_trim_l: float = 0.0, ank_trim_r: float = 0.0) -> PredScore:
    """A10+025. Pitch lead 20 ms, roll lead 0, sag cancel off. No y_swap."""
    return measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=0.025,
        toe_up_shape="front",
        toe_full_frac=0.10,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        whole_toe=True,
        surface_tag=True,
        ka_log=True,
        early_log=True,
        sag_cancel=False,
        z_profile="phase",
        sole_corner_log=True,
        ank_trim_l=ank_trim_l,
        ank_trim_r=ank_trim_r,
        ank_log=True,
    )


def _foot_tick(row: PredScore, side: str) -> dict[str, object] | None:
    for tick in row.sole_ticks:
        if isinstance(tick, dict) and tick.get("side") == side:
            return tick
    return None


def _worst_tick(row: PredScore) -> dict[str, object]:
    found = [
        tick for tick in row.sole_ticks
        if isinstance(tick, dict) and int(tick.get("is_worst", 0)) == 1
    ]
    if not found:
        raise SystemExit(f"{row.name} sole_208 worst tick missing")
    return found[0]


def _roll_bits(row: PredScore) -> str:
    bits: list[str] = []
    for side in ("L", "R"):
        tick = _foot_tick(row, side)
        if tick is None:
            bits.append(f"{side} missed")
            continue
        bits.append(
            f"{side} sole_roll {float(tick['roll']):+.5f} rad "
            f"centre_minus_front_outside {float(tick['drop_fo_m']) * 1000:+.3f} mm "
            f"scored {tick['scored_label']} lowest {tick['low_label']} "
            f"t {float(tick['t']):.3f}"
        )
    return " ".join(bits)


def _report_sole(label: str, row: PredScore, tick: dict[str, object]) -> None:
    drop_mm = float(tick["drop_fo_m"]) * 1000.0
    print(
        f"PRED sole_level {label} {row.name} "
        f"flat_worst {row.flat_toe_mm:.3f} mm t {row.flat_toe_t:.3f} "
        f"side {row.flat_toe_side} "
        f"vs_plus2 {row.flat_toe_mm - 2.0:+.3f} mm "
        f"sole_roll {float(tick['roll']):+.5f} rad "
        f"centre_minus_front_outside {drop_mm:+.3f} mm "
        f"scored {tick['scored_label']} lowest {tick['low_label']} "
        f"flat_hx {row.flat_hx_over or 'under'} "
        f"ank_hx {row.ank_hx_over or 'under'} "
        f"ka_over {row.ka_over_n} ka_checked {row.ka_checked}"
    )


def score_sole_level() -> None:
    """Measure sole roll on A10+025, then trim swing ankle roll only if the drop is ~0.6 mm.

    The trim is not a world-level sole. It does not ask the ankle for body roll.
    A10+030 and the y_swap copies are not run. Sag cancel stays off.
    """
    print(
        "PRED sole_level_plan A10+030 was not run. A05+025 was not run. "
        "A00+025 was not run. y_swap was not re-run. Sag cancel stays off. "
        "Base is A10+025: toe-up 0.025 rad full by 10%, held through 80%, "
        "20 ms pitch lead, roll lead 0, period 0.500 s, swing-hip clip, "
        "hip pitch off the 20 ms approach. "
        "Step 1 logs sole roll and the four corners at fraction 0.208 "
        "on the worst flat swing. "
        f"About 0.6 mm means centre minus front-outside between "
        f"{SOLE_FO_LO_MM:.2f} and {SOLE_FO_HI_MM:.2f} mm. "
        "Outside that window the trim copy is not run. "
        f"The trim adds at most ±{ANK_TRIM_CAP:.3f} rad on the swing ankle roll. "
        "It is not a world-level sole. Body roll is not the ankle target. "
        "One kinematic tick per foot is printed before that sweep."
    )
    _require_plant("before")
    measured = _sole_level_copy()
    _require_plant("after_measure")
    tick = _worst_tick(measured)
    drop_mm = float(tick["drop_fo_m"]) * 1000.0
    about = SOLE_FO_LO_MM - 1e-9 <= drop_mm <= SOLE_FO_HI_MM + 1e-9
    _report_sole("measure", measured, tick)
    print(f"PRED sole_level measure_feet {_roll_bits(measured)}")
    if not about:
        print(
            f"PRED sole_level STOP centre_minus_front_outside {drop_mm:+.3f} mm "
            f"is outside {SOLE_FO_LO_MM:.2f}-{SOLE_FO_HI_MM:.2f} mm. "
            "Trim copy was not run."
        )
        print(
            "PRED sole_level Prefer FAIL. "
            f"A10+025 flat_worst {measured.flat_toe_mm:+.3f} mm "
            f"t {measured.flat_toe_t:.3f} side {measured.flat_toe_side} "
            f"sole_roll {float(tick['roll']):+.5f} rad "
            f"centre_minus_front_outside {drop_mm:+.3f} mm "
            f"scored {tick['scored_label']} lowest {tick['low_label']} "
            f"flat_hx {measured.flat_hx_over or 'under'} "
            f"ank_hx {measured.ank_hx_over or 'under'}. "
            f"feet {_roll_bits(measured)}. "
            "The ankle-roll trim was not run. A10+030 was not run. "
            "y_swap was not re-run. Less-crouch was not run. "
            "Pitch lead stays 20 ms. Period stays 0.500 s. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    model = mj.MjModel.from_xml_path(SCENE)
    data = mj.MjData(model)
    view = _sole_view(model, data)
    chosen: dict[str, dict[str, object]] = {}
    for side in ("L", "R"):
        foot = _foot_tick(measured, side)
        if foot is None:
            print(
                f"PRED sole_level STOP side {side} sole_208_foot missed. "
                "Trim copy was not run."
            )
            print(
                "PRED sole_level Prefer FAIL. "
                "A foot had no flat 0.208 tick, so the ankle-roll trim was not run. "
                "Less-crouch was not run. Not kit-safe. Not go-anywhere."
            )
            return
        picked = _choose_add(model, data, view, foot)
        if picked is None:
            print(
                "PRED sole_level Prefer FAIL. "
                "The sign check refused the ankle-roll trim before the sweep. "
                f"measure flat_worst {measured.flat_toe_mm:+.3f} mm "
                f"sole_roll {float(tick['roll']):+.5f} rad "
                f"centre_minus_front_outside {drop_mm:+.3f} mm "
                f"scored {tick['scored_label']}. "
                "Less-crouch was not run. Not kit-safe. Not go-anywhere."
            )
            return
        chosen[side] = picked
    adds = _match_mirror(model, data, view, chosen, str(tick["side"]))
    trimmed = _sole_level_copy(adds["L"], adds["R"])
    _require_plant("after_trim")
    tick_b = _worst_tick(trimmed)
    drop_b = float(tick_b["drop_fo_m"]) * 1000.0
    _report_sole("trim", trimmed, tick_b)
    print(f"PRED sole_level trim_feet {_roll_bits(trimmed)}")
    cleared = (
        trimmed.flat_toe_mm >= 2.0
        and trimmed.flat_hx_over == ""
        and trimmed.ank_hx_over == ""
    )
    if cleared:
        print(
            "PRED sole_level clear "
            f"{trimmed.name} flat_worst {trimmed.flat_toe_mm:.3f} mm "
            f"t {trimmed.flat_toe_t:.3f} side {trimmed.flat_toe_side}. "
            f"before sole_roll {float(tick['roll']):+.5f} rad "
            f"centre_minus_front_outside {drop_mm:+.3f} mm "
            f"scored {tick['scored_label']}. "
            f"after sole_roll {float(tick_b['roll']):+.5f} rad "
            f"centre_minus_front_outside {drop_b:+.3f} mm "
            f"scored {tick_b['scored_label']} "
            f"flat_hx under ank_hx under. "
            f"adds L {adds['L']:+.5f} R {adds['R']:+.5f}. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    print(
        "PRED sole_level Prefer FAIL. "
        "The swing ankle-roll trim did not clear a flat mid-swing contact-box "
        "toe of +2 mm with both knees and both ankle rolls at or under 2.33 Nm "
        "on floor contact. "
        f"measure flat_worst {measured.flat_toe_mm:+.3f} mm "
        f"sole_roll {float(tick['roll']):+.5f} rad "
        f"centre_minus_front_outside {drop_mm:+.3f} mm "
        f"scored {tick['scored_label']} lowest {tick['low_label']} "
        f"flat_hx {measured.flat_hx_over or 'under'} "
        f"ank_hx {measured.ank_hx_over or 'under'}. "
        f"trim flat_worst {trimmed.flat_toe_mm:+.3f} mm "
        f"t {trimmed.flat_toe_t:.3f} side {trimmed.flat_toe_side} "
        f"sole_roll {float(tick_b['roll']):+.5f} rad "
        f"centre_minus_front_outside {drop_b:+.3f} mm "
        f"scored {tick_b['scored_label']} lowest {tick_b['low_label']} "
        f"flat_hx {trimmed.flat_hx_over or 'under'} "
        f"ank_hx {trimmed.ank_hx_over or 'under'}. "
        f"adds L {adds['L']:+.5f} R {adds['R']:+.5f}. "
        f"measure_feet {_roll_bits(measured)}. "
        f"trim_feet {_roll_bits(trimmed)}. "
        "Less-crouch was not run. A10+030 was not run. y_swap was not re-run. "
        "Pitch lead stays 20 ms. Period stays 0.500 s. "
        "Not kit-safe. Not go-anywhere."
    )


def _ank_pitch_adr(model: mj.MjModel, side: str) -> int:
    jn = ("l_" if side == "L" else "r_") + "ank_pitch"
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
    if jid < 0:
        raise SystemExit(f"missing joint {jn}")
    return int(model.jnt_qposadr[jid])


def _expect_rise_mm(pitch: float, roll: float) -> float:
    """11 mm times toe-down pitch plus 14 mm times |roll|. An expectation."""
    return PITCH_AXIS_MM * max(float(pitch), 0.0) + ROLL_AXIS_MM * abs(float(roll))


def _pose_level(
    model: mj.MjModel,
    data: mj.MjData,
    view: SimpleNamespace,
    qpos: np.ndarray,
    roll_adr: int,
    pitch_adr: int,
    side: str,
    roll_add: float,
    pitch_add: float,
) -> dict[str, object]:
    data.qpos[:] = qpos
    data.qvel[:] = 0.0
    data.qpos[roll_adr] = float(qpos[roll_adr]) + float(roll_add)
    data.qpos[pitch_adr] = float(qpos[pitch_adr]) + float(pitch_add)
    mj.mj_forward(model, data)
    roll, inside_drop, _half, inside = _sole_roll(view, side)
    corners = tuple(
        (str(label), float(z), float(off))
        for label, z, off in _box_corner_table(view, side)
    )
    centre = float(sum(z for _label, z, _off in corners) / len(corners))
    fo = next(z for label, z, _off in corners if label == "front-outside")
    front = [z for label, z, _off in corners if str(label).startswith("front")]
    scored = max(corners, key=lambda row: row[2])
    return {
        "roll": float(roll),
        "pitch": float(_sole_pitch(view, side)),
        "inside_drop": float(inside_drop),
        "inside": inside,
        "center_z": centre,
        "front_z": float(sum(front) / len(front)),
        "front_outside_z": float(fo),
        "drop_fo_m": centre - float(fo),
        "corners": corners,
        "scored_label": str(scored[0]),
    }


def _level_ok(base: dict[str, object], after: dict[str, object]) -> bool:
    """Leftover sole only. A sign error that doubles the drop does not pass.

    |roll| and |pitch| must not rise. A high inside edge must not go higher.
    Toe-down pitch must not lower the front. A low front-outside corner
    must not drop further.
    """
    if abs(float(after["roll"])) > abs(float(base["roll"])) + 2e-4:
        return False
    if abs(float(after["pitch"])) > abs(float(base["pitch"])) + 2e-4:
        return False
    if (
        float(base["inside_drop"]) < 0.0
        and float(after["inside_drop"]) < float(base["inside_drop"]) - 5e-5
    ):
        return False
    if (
        float(base["pitch"]) > 1e-4
        and float(after["front_z"]) < float(base["front_z"]) - 5e-5
    ):
        return False
    if (
        float(base["drop_fo_m"]) > 0.0
        and float(after["drop_fo_m"]) > float(base["drop_fo_m"]) + 5e-5
    ):
        return False
    return True


def _print_level_corners(tag: str, side: str, phase: str, pack: dict[str, object]) -> None:
    corners = _corner_map(pack["corners"])
    centre = float(pack["center_z"])
    for label in _CORNER_ORDER:
        z, off = corners[label]
        print(
            f"PRED level_sign {tag} corner {side} {phase} {label} "
            f"z {z * 1000:+.3f} mm "
            f"centre_minus_corner {(centre - z) * 1000:+.3f} mm "
            f"forward {off * 1000:.1f} mm "
            f"scored {int(label == pack.get('scored_label', ''))}"
        )


def _print_level_sign(
    tag: str,
    side: str,
    t: float,
    bout_roll: float,
    bout_pitch: float,
    base: dict[str, object],
    after: dict[str, object],
    droll: float,
    dpitch: float,
    raw_roll: float,
    raw_pitch: float,
    add_roll: float,
    add_pitch: float,
    flip_roll: int,
    flip_pitch: int,
    ank_roll: float,
    ank_pitch: float,
) -> None:
    expect = _expect_rise_mm(bout_pitch, bout_roll)
    rise = (float(after["center_z"]) - float(base["center_z"])) * 1000.0
    print(
        f"PRED level_sign {tag} side {side} t {t:.3f} "
        f"bout_roll {bout_roll:+.5f} replay_roll {float(base['roll']):+.5f} "
        f"bout_pitch {bout_pitch:+.5f} replay_pitch {float(base['pitch']):+.5f} rad "
        f"roll_mismatch {abs(float(base['roll']) - bout_roll):.5f} "
        f"pitch_mismatch {abs(float(base['pitch']) - bout_pitch):.5f} rad "
        f"droll {droll:+.5f} dpitch {dpitch:+.5f} "
        f"raw_roll {raw_roll:+.5f} raw_pitch {raw_pitch:+.5f} "
        f"add_roll {add_roll:+.5f} add_pitch {add_pitch:+.5f} rad "
        f"flip_roll {flip_roll} flip_pitch {flip_pitch} cap {ANK_TRIM_CAP:.3f} "
        f"roll_after {float(after['roll']):+.5f} pitch_after {float(after['pitch']):+.5f} rad "
        f"inside {base['inside']} "
        f"inside_drop {float(base['inside_drop']) * 1000:+.3f} "
        f"to {float(after['inside_drop']) * 1000:+.3f} mm "
        f"inside_lowered {int(float(after['inside_drop']) > float(base['inside_drop']) + 1e-6)} "
        f"centre {float(base['center_z']) * 1000:+.3f} "
        f"to {float(after['center_z']) * 1000:+.3f} mm "
        f"centre_rise {rise:+.3f} mm expect_centre_rise {expect:+.3f} mm "
        f"lever_mm {PITCH_AXIS_MM:.0f} {ROLL_AXIS_MM:.0f} "
        f"front {float(base['front_z']) * 1000:+.3f} "
        f"to {float(after['front_z']) * 1000:+.3f} mm "
        f"centre_minus_front_outside {float(base['drop_fo_m']) * 1000:+.3f} "
        f"to {float(after['drop_fo_m']) * 1000:+.3f} mm "
        f"ank_roll_ctrl {ank_roll:+.5f} target {ank_roll + add_roll:+.5f} "
        f"ank_pitch_ctrl {ank_pitch:+.5f} target {ank_pitch + add_pitch:+.5f} "
        "not_world_level 1 leftover_sole 1"
    )
    _print_level_corners(tag, side, "before", base)
    _print_level_corners(tag, side, "after", after)


def _choose_level_add(
    model: mj.MjModel,
    data: mj.MjData,
    view: SimpleNamespace,
    tick: dict[str, object],
) -> dict[str, object] | None:
    """Joint adds that cancel leftover sole roll and sole pitch together."""
    side = str(tick["side"])
    qpos = np.asarray(tick["qpos"], dtype=np.float64)
    roll_adr = _ank_roll_adr(model, side)
    pitch_adr = _ank_pitch_adr(model, side)
    base = _pose_level(model, data, view, qpos, roll_adr, pitch_adr, side, 0.0, 0.0)
    nudged_r = _pose_level(model, data, view, qpos, roll_adr, pitch_adr, side, 0.01, 0.0)
    nudged_p = _pose_level(model, data, view, qpos, roll_adr, pitch_adr, side, 0.0, 0.01)
    droll = (float(nudged_r["roll"]) - float(base["roll"])) / 0.01
    dpitch = (float(nudged_p["pitch"]) - float(base["pitch"])) / 0.01
    bout_roll = float(tick["roll"])
    bout_pitch = float(tick["pitch"])
    ank_roll = float(tick["ank_ctrl"])
    ank_pitch = float(tick["ank_pitch_ctrl"])
    roll_mis = abs(float(base["roll"]) - bout_roll)
    pitch_mis = abs(float(base["pitch"]) - bout_pitch)
    if roll_mis > 5e-3 or pitch_mis > 5e-3:
        print(
            f"PRED level_sign side {side} replay_mismatch "
            f"roll {roll_mis:.5f} pitch {pitch_mis:.5f} rad exceeds 0.005. "
            "Trim copy was not run."
        )
        return None
    if abs(droll) < 1e-4 or abs(dpitch) < 1e-4:
        print(
            f"PRED level_sign side {side} droll {droll:+.5f} dpitch {dpitch:+.5f} "
            "does not move the sole. Trim copy was not run."
        )
        return None
    raw_roll = -float(base["roll"]) / droll
    raw_pitch = -float(base["pitch"]) / dpitch
    add_roll = _clip_trim(raw_roll)
    add_pitch = _clip_trim(raw_pitch)
    trials = (
        (0, 0, add_roll, add_pitch),
        (1, 0, _clip_trim(-add_roll), add_pitch),
        (0, 1, add_roll, _clip_trim(-add_pitch)),
        (1, 1, _clip_trim(-add_roll), _clip_trim(-add_pitch)),
    )
    seen: set[tuple[float, float]] = set()
    last: tuple[int, int, float, float, dict[str, object]] | None = None
    for flip_roll, flip_pitch, trial_roll, trial_pitch in trials:
        key = (round(trial_roll, 8), round(trial_pitch, 8))
        if key in seen:
            continue
        seen.add(key)
        after = _pose_level(
            model, data, view, qpos, roll_adr, pitch_adr, side, trial_roll, trial_pitch,
        )
        last = (flip_roll, flip_pitch, trial_roll, trial_pitch, after)
        if not _level_ok(base, after):
            continue
        _print_level_sign(
            "tick", side, float(tick["t"]), bout_roll, bout_pitch, base, after,
            droll, dpitch, raw_roll, raw_pitch, trial_roll, trial_pitch,
            flip_roll, flip_pitch, ank_roll, ank_pitch,
        )
        return {
            "side": side,
            "t": float(tick["t"]),
            "bout_roll": bout_roll,
            "bout_pitch": bout_pitch,
            "ank_ctrl": ank_roll,
            "ank_pitch_ctrl": ank_pitch,
            "qpos": qpos,
            "roll_adr": roll_adr,
            "pitch_adr": pitch_adr,
            "base": base,
            "after": after,
            "droll": droll,
            "dpitch": dpitch,
            "raw_roll": raw_roll,
            "raw_pitch": raw_pitch,
            "add_roll": trial_roll,
            "add_pitch": trial_pitch,
            "flip_roll": flip_roll,
            "flip_pitch": flip_pitch,
        }
    print(
        f"PRED level_sign side {side} no sign lowers the inside edge "
        "and raises a toe-down front without growing |sole roll| or |sole pitch|. "
        "Trim copy was not run."
    )
    if last is not None:
        flip_roll, flip_pitch, trial_roll, trial_pitch, after = last
        _print_level_sign(
            "reject", side, float(tick["t"]), bout_roll, bout_pitch, base, after,
            droll, dpitch, raw_roll, raw_pitch, trial_roll, trial_pitch,
            flip_roll, flip_pitch, ank_roll, ank_pitch,
        )
    return None


def _match_level_mirror(
    model: mj.MjModel,
    data: mj.MjData,
    view: SimpleNamespace,
    chosen: dict[str, dict[str, object]],
    worst_side: str,
) -> dict[str, dict[str, float]]:
    """Match ankle-roll magnitudes. Pitch adds stay per foot."""
    left = float(chosen["L"]["add_roll"])
    right = float(chosen["R"]["add_roll"])
    pitches = {side: float(chosen[side]["add_pitch"]) for side in ("L", "R")}
    if left * right >= 0.0:
        print(
            f"PRED level_sign pair_mirror 0 L {left:+.5f} R {right:+.5f}. "
            "A mirrored roll sign raises |sole roll| on one foot. "
            "Roll adds stay the probed signs. Pitch adds stay per foot."
        )
        return {
            side: {"roll": float(chosen[side]["add_roll"]), "pitch": pitches[side]}
            for side in ("L", "R")
        }
    mag = abs(float(chosen[worst_side]["add_roll"]))
    out_roll: dict[str, float] = {}
    for side in ("L", "R"):
        own = float(chosen[side]["add_roll"])
        trial = _clip_trim(math.copysign(mag, own))
        if abs(trial - own) <= 1e-9:
            out_roll[side] = own
            continue
        qpos = np.asarray(chosen[side]["qpos"], dtype=np.float64)
        after = _pose_level(
            model, data, view, qpos,
            int(chosen[side]["roll_adr"]), int(chosen[side]["pitch_adr"]),
            side, trial, pitches[side],
        )
        base = chosen[side]["base"]
        if not isinstance(base, dict) or not _level_ok(base, after):
            out_roll[side] = own
            print(
                f"PRED level_sign pair_mag_keep side {side} "
                f"trial {trial:+.5f} own {own:+.5f}. "
                "Matching the worst-foot roll magnitude raises |sole roll| "
                "or the front-outside drop. Pitch add stays on this foot."
            )
            continue
        out_roll[side] = trial
        _print_level_sign(
            "mirror", side, float(chosen[side]["t"]),
            float(chosen[side]["bout_roll"]), float(chosen[side]["bout_pitch"]),
            base, after, float(chosen[side]["droll"]), float(chosen[side]["dpitch"]),
            float(chosen[side]["raw_roll"]), float(chosen[side]["raw_pitch"]),
            trial, pitches[side], int(chosen[side]["flip_roll"]),
            int(chosen[side]["flip_pitch"]), float(chosen[side]["ank_ctrl"]),
            float(chosen[side]["ank_pitch_ctrl"]),
        )
    matched = int(abs(abs(out_roll["L"]) - abs(out_roll["R"])) <= 1e-9)
    print(
        f"PRED level_sign pair_mirror 1 mag {mag:.5f} "
        f"L {out_roll['L']:+.5f} R {out_roll['R']:+.5f} "
        f"magnitudes_matched {matched} worst_side {worst_side} "
        f"pitch L {pitches['L']:+.5f} R {pitches['R']:+.5f}"
    )
    return {
        side: {"roll": out_roll[side], "pitch": pitches[side]}
        for side in ("L", "R")
    }


def _level_trim_copy(
    ank_trim_l: float = 0.0,
    ank_trim_r: float = 0.0,
    ank_pitch_trim_l: float = 0.0,
    ank_pitch_trim_r: float = 0.0,
    z_extra_m: float = 0.0,
) -> PredScore:
    """A10+025 plus an optional leftover-sole trim and an optional foot-z."""
    return measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=0.025,
        toe_up_shape="front",
        toe_full_frac=0.10,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        whole_toe=True,
        surface_tag=True,
        ka_log=True,
        sag_cancel=False,
        z_profile="phase",
        sole_corner_log=True,
        ank_trim_l=ank_trim_l,
        ank_trim_r=ank_trim_r,
        ank_pitch_trim_l=ank_pitch_trim_l,
        ank_pitch_trim_r=ank_pitch_trim_r,
        ank_log=True,
        z_extra_m=z_extra_m,
    )


def _knee_over(row: PredScore) -> str:
    hits = [
        part.strip()
        for part in row.flat_hx_over.split(",")
        if "knee" in part
    ]
    return ", ".join(hits)


def _bar_clear(row: PredScore) -> bool:
    return row.flat_toe_mm >= 2.0 and row.flat_hx_over == "" and row.ank_hx_over == ""


def _geom_bits(tick: dict[str, object]) -> str:
    return (
        f"sole_roll {float(tick['roll']):+.5f} rad "
        f"sole_pitch {float(tick['pitch']):+.5f} rad "
        f"centre {float(tick['center_z']) * 1000:+.3f} mm "
        f"scored {tick['scored_label']} lowest {tick['low_label']} "
        f"centre_minus_front_outside {float(tick['drop_fo_m']) * 1000:+.3f} mm "
        f"t {float(tick['t']):.3f}"
    )


def score_level_trim() -> None:
    """Swing-only leftover sole roll and pitch on A10+025, then sized foot-z.

    The 0.6 mm roll-only gate is not this copy. Foot-z is the trim bout's
    shortfall to +2 mm. A floor knee over 2.33 stops the batch. Less-crouch
    is named and not implemented.
    """
    print(
        "PRED level_trim_plan A10+030 was not run. A05+025 was not run. "
        "A00+025 was not run. y_swap was not re-run. "
        "The roll-only 0.6 mm trim was not re-run and its gate was not loosened. "
        "Sag cancel stays off. Less-crouch is not this copy. "
        "Base is A10+025: toe-up 0.025 rad full by 10%, held through 80%, "
        "20 ms pitch lead, roll lead 0, period 0.500 s, swing-hip clip, "
        "hip pitch off the 20 ms approach. "
        "Step 1 adds leftover sole roll and leftover sole pitch on the swing "
        f"ankle, each capped at ±{ANK_TRIM_CAP:.3f} rad. Not a world-level sole. "
        "Body roll is not the ankle target. "
        f"Pitch axis {PITCH_AXIS_MM:.0f} mm behind the box centre. "
        f"Roll axis {ROLL_AXIS_MM:.0f} mm inboard of the box. "
        "The centre-rise expectation is those levers times the trimmed angles. "
        "The measured centre rise is the result. "
        "One kinematic tick per foot is printed before the trim sweep. "
        "Step 2 sizes foot-z from the trim bout's shortfall to +2 mm. "
        "The old +0.4 mm add is not carried in."
    )
    _require_plant("before", "level_trim")
    measured = _level_trim_copy()
    _require_plant("after_measure", "level_trim")
    tick = _worst_tick(measured)
    print(
        f"PRED level_trim measure {measured.name} "
        f"flat_worst {measured.flat_toe_mm:+.3f} mm "
        f"t {measured.flat_toe_t:.3f} side {measured.flat_toe_side} "
        f"flat_hx {measured.flat_hx_over or 'under'} "
        f"ank_hx {measured.ank_hx_over or 'under'} "
        f"{_geom_bits(tick)}"
    )
    model = mj.MjModel.from_xml_path(SCENE)
    data = mj.MjData(model)
    view = _sole_view(model, data)
    print("PRED level_trim sign_ticks_before_trim 1")
    chosen: dict[str, dict[str, object]] = {}
    for side in ("L", "R"):
        foot = _foot_tick(measured, side)
        if foot is None:
            print(
                f"PRED level_trim STOP side {side} sole_208_foot missed. "
                "Trim copy was not run. Foot-z was not run. "
                "Less-crouch was not run. Not kit-safe. Not go-anywhere."
            )
            return
        picked = _choose_level_add(model, data, view, foot)
        if picked is None:
            print(
                "PRED level_trim Prefer FAIL. "
                "The sign check refused the level trim before the sweep. "
                f"measure flat_worst {measured.flat_toe_mm:+.3f} mm "
                f"{_geom_bits(tick)}. "
                "Foot-z was not run. Less-crouch was not run. "
                "Not kit-safe. Not go-anywhere."
            )
            return
        chosen[side] = picked
    adds = _match_level_mirror(model, data, view, chosen, str(tick["side"]))
    for side in ("L", "R"):
        row = chosen[side]
        qpos = np.asarray(row["qpos"], dtype=np.float64)
        after = _pose_level(
            model, data, view, qpos,
            int(row["roll_adr"]), int(row["pitch_adr"]),
            side, adds[side]["roll"], adds[side]["pitch"],
        )
        base = row["base"]
        if not isinstance(base, dict):
            raise SystemExit(f"level trim base pose missing for {side}")
        _print_level_sign(
            "apply", side, float(row["t"]),
            float(row["bout_roll"]), float(row["bout_pitch"]),
            base, after, float(row["droll"]), float(row["dpitch"]),
            float(row["raw_roll"]), float(row["raw_pitch"]),
            adds[side]["roll"], adds[side]["pitch"],
            int(row["flip_roll"]), int(row["flip_pitch"]),
            float(row["ank_ctrl"]), float(row["ank_pitch_ctrl"]),
        )
    trimmed = _level_trim_copy(
        adds["L"]["roll"], adds["R"]["roll"],
        adds["L"]["pitch"], adds["R"]["pitch"],
    )
    _require_plant("after_trim", "level_trim")
    tick_b = _worst_tick(trimmed)
    knee = _knee_over(trimmed)
    print(
        f"PRED level_trim step1 {trimmed.name} "
        f"flat_worst {trimmed.flat_toe_mm:+.3f} mm "
        f"t {trimmed.flat_toe_t:.3f} side {trimmed.flat_toe_side} "
        f"vs_plus2 {trimmed.flat_toe_mm - 2.0:+.3f} mm "
        f"flat_hx {trimmed.flat_hx_over or 'under'} "
        f"ank_hx {trimmed.ank_hx_over or 'under'} "
        f"adds_roll L {adds['L']['roll']:+.5f} R {adds['R']['roll']:+.5f} "
        f"adds_pitch L {adds['L']['pitch']:+.5f} R {adds['R']['pitch']:+.5f} "
        f"{_geom_bits(tick_b)}"
    )
    for side in ("L", "R"):
        before = _foot_tick(measured, side)
        after = _foot_tick(trimmed, side)
        if before is None or after is None:
            print(f"PRED level_trim centre_rise side {side} missed")
            continue
        dyn = (float(after["center_z"]) - float(before["center_z"])) * 1000.0
        expect = _expect_rise_mm(float(before["pitch"]), float(before["roll"]))
        print(
            f"PRED level_trim centre_rise side {side} "
            f"dynamic {dyn:+.3f} mm expect {expect:+.3f} mm "
            f"before {_geom_bits(before)} "
            f"after {_geom_bits(after)}"
        )
    if knee:
        print(
            "PRED level_trim STOP floor knee over 2.33 on the level trim: "
            f"{knee}. Foot-z copy was not run. "
            "Less-crouch is next. init_z_offset was not invented. "
            f"step1 flat_worst {trimmed.flat_toe_mm:+.3f} mm "
            f"t {trimmed.flat_toe_t:.3f} side {trimmed.flat_toe_side} "
            f"flat_hx {trimmed.flat_hx_over or 'under'} "
            f"ank_hx {trimmed.ank_hx_over or 'under'}. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    if _bar_clear(trimmed):
        print(
            "PRED level_trim clear "
            f"{trimmed.name} flat_worst {trimmed.flat_toe_mm:.3f} mm "
            f"t {trimmed.flat_toe_t:.3f} side {trimmed.flat_toe_side}. "
            "Plus 2 mm cleared with knees, ankle pitch, and ankle roll "
            "at or under 2.33 Nm on floor contact. Foot-z was not added. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    short_mm = 2.0 - trimmed.flat_toe_mm
    if short_mm <= 1e-9:
        print(
            "PRED level_trim Prefer FAIL. "
            "The level trim is not short of +2 mm, and a floor joint is over 2.33 Nm. "
            "The over is not a floor knee, so less-crouch was not started. "
            "Foot-z was not added. No further copy was invented. "
            f"step1 flat_worst {trimmed.flat_toe_mm:+.3f} mm "
            f"flat_hx {trimmed.flat_hx_over or 'under'} "
            f"ank_hx {trimmed.ank_hx_over or 'under'}. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    z_extra = short_mm / 1000.0
    print(
        f"PRED level_trim z_size shortfall {short_mm:.3f} mm "
        f"command {short_mm:.3f} mm. "
        "Sized from the trim bout toe to +2 mm. "
        "The old +0.4 mm add was not carried in. "
        "Not +2 mm minus the box centre."
    )
    lifted = _level_trim_copy(
        adds["L"]["roll"], adds["R"]["roll"],
        adds["L"]["pitch"], adds["R"]["pitch"],
        z_extra,
    )
    _require_plant("after_z", "level_trim")
    tick_z = _worst_tick(lifted)
    knee_z = _knee_over(lifted)
    cleared = _bar_clear(lifted)
    print(
        f"PRED level_trim step2 {lifted.name} "
        f"flat_worst {lifted.flat_toe_mm:+.3f} mm "
        f"t {lifted.flat_toe_t:.3f} side {lifted.flat_toe_side} "
        f"vs_plus2 {lifted.flat_toe_mm - 2.0:+.3f} mm "
        f"z_command {short_mm:.3f} mm "
        f"plus2_under_2.33 {int(cleared)} "
        f"flat_hx {lifted.flat_hx_over or 'under'} "
        f"ank_hx {lifted.ank_hx_over or 'under'} "
        f"{_geom_bits(tick_z)}"
    )
    if knee_z:
        print(
            "PRED level_trim STOP floor knee over 2.33 on the sized foot-z: "
            f"{knee_z}. Less-crouch is next. init_z_offset was not invented. "
            "No third copy. "
            f"step1 flat_worst {trimmed.flat_toe_mm:+.3f} mm "
            f"step2 flat_worst {lifted.flat_toe_mm:+.3f} mm "
            f"t {lifted.flat_toe_t:.3f} side {lifted.flat_toe_side} "
            f"flat_hx {lifted.flat_hx_over or 'under'} "
            f"ank_hx {lifted.ank_hx_over or 'under'}. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    if cleared:
        print(
            "PRED level_trim clear "
            f"{lifted.name} flat_worst {lifted.flat_toe_mm:.3f} mm "
            f"t {lifted.flat_toe_t:.3f} side {lifted.flat_toe_side}. "
            f"Foot-z command {short_mm:.3f} mm on the level trim. "
            "Plus 2 mm cleared with knees, ankle pitch, and ankle roll "
            "at or under 2.33 Nm on floor contact. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    print(
        "PRED level_trim Prefer FAIL. "
        "The sized foot-z on the level trim did not clear +2 mm "
        "with knees, ankle pitch, and ankle roll at or under 2.33 Nm "
        "on floor contact. The floor knee stayed at or under 2.33 Nm, "
        "so less-crouch was not started. No third copy. "
        f"step1 flat_worst {trimmed.flat_toe_mm:+.3f} mm "
        f"t {trimmed.flat_toe_t:.3f} side {trimmed.flat_toe_side} "
        f"{_geom_bits(tick_b)} "
        f"flat_hx {trimmed.flat_hx_over or 'under'} "
        f"ank_hx {trimmed.ank_hx_over or 'under'}. "
        f"step2 flat_worst {lifted.flat_toe_mm:+.3f} mm "
        f"t {lifted.flat_toe_t:.3f} side {lifted.flat_toe_side} "
        f"z_command {short_mm:.3f} mm "
        f"{_geom_bits(tick_z)} "
        f"flat_hx {lifted.flat_hx_over or 'under'} "
        f"ank_hx {lifted.ank_hx_over or 'under'}. "
        "Pitch lead stays 20 ms. Period stays 0.500 s. "
        "Not kit-safe. Not go-anywhere."
    )


_FLOOR_ACTS = (
    "l_knee_pos", "r_knee_pos",
    "l_ank_pitch_pos", "r_ank_pitch_pos",
    "l_ank_roll_pos", "r_ank_roll_pos",
)
# Foot-z sweep cap. 0.170 mm plus twenty 0.1 mm steps.
_Z_SWEEP_CAP_UM = 2170


def _margin_hundredths(toe_mm: float) -> tuple[float, float]:
    """Raw +2 gap, and that gap floored to a hundredth of a millimetre."""
    margin = 2.0 - float(toe_mm)
    hundredths = math.floor(margin * 100.0 + 1e-9) / 100.0
    return margin, hundredths


def _floor_over(row: PredScore) -> str:
    """Floor or floor+rug knee and ankle peaks at or over 2.33 Nm."""
    hits: list[str] = []
    for item in row.floor_chain:
        act, tau, when, contact = item
        if str(contact) not in ("floor", "floor+rug"):
            continue
        if act not in _FLOOR_ACTS:
            continue
        if abs(float(tau)) < KNEE_NM:
            continue
        hits.append(
            f"{act} {float(tau):+.4f} t {float(when):.3f} contact {contact}"
        )
    return ", ".join(hits)


def _print_floor_chain(tag: str, row: PredScore) -> str:
    over = _floor_over(row)
    if not row.floor_chain:
        print(f"PRED trim_lead {tag} floor missing")
        return over
    for item in row.floor_chain:
        act, tau, when, contact = item
        ge = int(
            str(contact) in ("floor", "floor+rug") and abs(float(tau)) >= KNEE_NM
        )
        print(
            f"PRED trim_lead {tag} floor {act} {float(tau):+.4f} "
            f"t {float(when):.3f} contact {contact or '-'} ge_2.33 {ge}"
        )
    return over


def _print_exact_208(tag: str, tick: dict[str, object], flat_toe_mm: float) -> None:
    """0.208 sample in raw millimetres. The margin hundredth is floored."""
    roll = float(tick["roll"])
    pitch = float(tick["pitch"])
    centre = float(tick["center_z"]) * 1000.0
    toe = float(tick["toe_z"]) * 1000.0
    margin, hundredths = _margin_hundredths(toe)
    flat_margin, flat_h = _margin_hundredths(flat_toe_mm)
    under = int(abs(roll) < 5e-4 and abs(pitch) < 5e-4)
    print(
        f"PRED trim_lead exact {tag} side {tick['side']} "
        f"t {float(tick['t']):.3f} "
        f"cycle_frac {float(tick['frac']):.6f} "
        f"gait_frac {float(tick.get('gait_frac', float('nan'))):.6f} "
        f"sole_roll {roll:+.8f} rad sole_pitch {pitch:+.8f} rad "
        f"under_0.0005 {under} "
        f"centre_mm {centre:.6f} "
        f"tick_toe_mm {toe:.6f} "
        f"flat_toe_mm {flat_toe_mm:.6f} "
        f"vs_plus2_mm {margin:.6f} "
        f"margin_hundredths {hundredths:.2f} "
        f"flat_vs_plus2_mm {flat_margin:.6f} "
        f"flat_margin_hundredths {flat_h:.2f} "
        f"scored {tick['scored_label']} lowest {tick['low_label']} "
        f"ank_ctrl {float(tick['ank_ctrl']):+.5f} "
        f"ank_pitch_ctrl {float(tick['ank_pitch_ctrl']):+.5f}"
    )
    corners = _corner_map(tick["corners"])
    for label in _CORNER_ORDER:
        z, off = corners[label]
        z_mm = z * 1000.0
        print(
            f"PRED trim_lead exact {tag} corner {label} "
            f"z_mm {z_mm:.6f} centre_minus_corner_mm {centre - z_mm:.6f} "
            f"forward_mm {off * 1000.0:.1f} "
            f"scored {int(label == tick['scored_label'])} "
            f"lowest {int(label == tick['low_label'])}"
        )


def _trim_lead_copy(
    ank_trim_l: float = 0.0,
    ank_trim_r: float = 0.0,
    ank_pitch_trim_l: float = 0.0,
    ank_pitch_trim_r: float = 0.0,
    z_extra_m: float = 0.000170,
    lead: bool = False,
    on_frac: float = 0.208,
) -> PredScore:
    """A10+025, level trim, foot-z, and an optional swing-only trim lead."""
    return measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=0.025,
        toe_up_shape="front",
        toe_full_frac=0.10,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        whole_toe=True,
        surface_tag=True,
        ka_log=True,
        sag_cancel=False,
        z_profile="phase",
        sole_corner_log=True,
        ank_trim_l=ank_trim_l,
        ank_trim_r=ank_trim_r,
        ank_pitch_trim_l=ank_pitch_trim_l,
        ank_pitch_trim_r=ank_pitch_trim_r,
        ank_log=True,
        z_extra_m=z_extra_m,
        ank_trim_lead=lead,
        ank_trim_on_frac=on_frac,
    )


def _gate_on_frac(row: PredScore) -> float | None:
    """Gait fraction of the cycle-0.208 sample. The smaller foot opens both."""
    fracs: list[float] = []
    for side in ("L", "R"):
        foot = _foot_tick(row, side)
        if foot is None:
            print(f"PRED trim_lead gait_frac side {side} missed")
            continue
        gait = float(foot.get("gait_frac", float("nan")))
        print(
            f"PRED trim_lead gait_frac side {side} {gait:.6f} "
            f"cycle_frac {float(foot['frac']):.6f}"
        )
        if gait == gait:
            fracs.append(gait)
    if len(fracs) < 2:
        return None
    on_frac = min(fracs)
    print(
        f"PRED trim_lead gate_on_gait {on_frac:.6f} cycle_frac 0.208 "
        "trim weight 0 before gate_on_gait"
    )
    return on_frac


def _print_ctrl_delta(
    measured: PredScore,
    led: PredScore,
    adds: dict[str, dict[str, float]],
) -> None:
    for side in ("L", "R"):
        before = _foot_tick(measured, side)
        after = _foot_tick(led, side)
        if before is None or after is None:
            print(f"PRED trim_lead ctrl_delta side {side} missed")
            continue
        droll = float(after["ank_ctrl"]) - float(before["ank_ctrl"])
        dpitch = float(after["ank_pitch_ctrl"]) - float(before["ank_pitch_ctrl"])
        print(
            f"PRED trim_lead ctrl_delta side {side} "
            f"roll {droll:+.5f} add {adds[side]['roll']:+.5f} "
            f"pitch {dpitch:+.5f} add {adds[side]['pitch']:+.5f} "
            f"gait_frac {float(after.get('gait_frac', float('nan'))):.6f}"
        )


def _less_crouch_next(why: str) -> None:
    print(
        "PRED trim_lead less_crouch_next 1. "
        f"{why} init_z_offset was not invented. "
        "Less-crouch was not run. Not kit-safe. Not go-anywhere."
    )


def score_trim_lead() -> None:
    """Lead the leftover-sole trim from lift-off, then a 0.1 mm foot-z sweep.

    The lead is the trim add only. Hip pitch stays at 20 ms and hip roll
    lead stays 0. The add is off before the lift-off fraction and ramps
    out before touchdown. Less-crouch is named only if both steps stall.
    """
    print(
        "PRED trim_lead_plan A10+030 was not run. y_swap was not re-run. "
        "The roll-only 0.6 mm trim was not re-run. Sag cancel stays off. "
        "Less-crouch is not this copy unless the lead and the foot-z sweep "
        "both stall. "
        "Base is level trim plus 0.170 mm foot-z on A10+025: "
        "toe-up 0.025 rad full by 10%, held through 80%, "
        "20 ms hip pitch lead, roll lead 0, period 0.500 s, swing-hip clip, "
        "hip pitch off the 20 ms approach. "
        "Step 1 leads only the leftover sole roll and pitch adds by each "
        "ankle joint's compiled kv/kp. The toe-up, the IK ankle, and the hip "
        "are not led. Trim weight is 0 before the lift-off gait fraction and "
        f"the schedule is 0 by {TRIM_OFF_FRAC:.2f}, before touchdown. "
        f"Each add stays inside ±{ANK_TRIM_CAP:.3f} rad. "
        "Step 2, if the lead is still short of +2 mm and no floor knee or "
        "ankle is over 2.33 Nm, continues foot-z from 0.170 mm in 0.1 mm "
        f"steps and stops at the first floor knee or ankle over 2.33 Nm "
        f"or at {_Z_SWEEP_CAP_UM / 1000.0:.3f} mm."
    )
    _require_plant("before", "trim_lead")
    measured = _trim_lead_copy()
    _require_plant("after_measure", "trim_lead")
    tick = _worst_tick(measured)
    print(
        f"PRED trim_lead measure {measured.name} "
        f"flat_worst {measured.flat_toe_mm:+.6f} mm "
        f"t {measured.flat_toe_t:.3f} side {measured.flat_toe_side} "
        f"flat_hx {measured.flat_hx_over or 'under'} "
        f"ank_hx {measured.ank_hx_over or 'under'} "
        f"{_geom_bits(tick)}"
    )
    on_frac = _gate_on_frac(measured)
    if on_frac is None:
        print(
            "PRED trim_lead STOP gait fraction at cycle 0.208 was missing. "
            "The lead was not run. Foot-z sweep was not run. "
            "Less-crouch was not run. Not kit-safe. Not go-anywhere."
        )
        return
    model = mj.MjModel.from_xml_path(SCENE)
    data = mj.MjData(model)
    view = _sole_view(model, data)
    print("PRED trim_lead sign_ticks_before_lead 1 z_mm 0.170")
    chosen: dict[str, dict[str, object]] = {}
    for side in ("L", "R"):
        foot = _foot_tick(measured, side)
        if foot is None:
            print(
                f"PRED trim_lead STOP side {side} sole_208_foot missed. "
                "The lead was not run. Foot-z sweep was not run. "
                "Less-crouch was not run. Not kit-safe. Not go-anywhere."
            )
            return
        picked = _choose_level_add(model, data, view, foot)
        if picked is None:
            print(
                "PRED trim_lead Prefer FAIL. "
                "The sign check refused the trim before the lead. "
                f"measure flat_worst {measured.flat_toe_mm:+.6f} mm "
                f"{_geom_bits(tick)}. "
                "The lead was not run. Foot-z sweep was not run. "
                "Less-crouch was not run. Not kit-safe. Not go-anywhere."
            )
            return
        chosen[side] = picked
    adds = _match_level_mirror(model, data, view, chosen, str(tick["side"]))
    for side in ("L", "R"):
        row = chosen[side]
        qpos = np.asarray(row["qpos"], dtype=np.float64)
        after = _pose_level(
            model, data, view, qpos,
            int(row["roll_adr"]), int(row["pitch_adr"]),
            side, adds[side]["roll"], adds[side]["pitch"],
        )
        base = row["base"]
        if not isinstance(base, dict):
            raise SystemExit(f"trim lead base pose missing for {side}")
        _print_level_sign(
            "apply", side, float(row["t"]),
            float(row["bout_roll"]), float(row["bout_pitch"]),
            base, after, float(row["droll"]), float(row["dpitch"]),
            float(row["raw_roll"]), float(row["raw_pitch"]),
            adds[side]["roll"], adds[side]["pitch"],
            int(row["flip_roll"]), int(row["flip_pitch"]),
            float(row["ank_ctrl"]), float(row["ank_pitch_ctrl"]),
        )
    led = _trim_lead_copy(
        adds["L"]["roll"], adds["R"]["roll"],
        adds["L"]["pitch"], adds["R"]["pitch"],
        0.000170, True, on_frac,
    )
    _require_plant("after_lead", "trim_lead")
    tick_b = _worst_tick(led)
    _print_ctrl_delta(measured, led, adds)
    _print_exact_208("step1", tick_b, led.flat_toe_mm)
    for side in ("L", "R"):
        foot = _foot_tick(led, side)
        if foot is None:
            print(f"PRED trim_lead exact step1_foot side {side} missed")
            continue
        if foot is tick_b:
            continue
        _print_exact_208(f"step1_foot_{side}", foot, float(foot["toe_z"]) * 1000.0)
    over = _print_floor_chain("step1", led)
    print(
        f"PRED trim_lead step1 {led.name} "
        f"flat_worst {led.flat_toe_mm:+.6f} mm "
        f"t {led.flat_toe_t:.3f} side {led.flat_toe_side} "
        f"flat_hx {led.flat_hx_over or 'under'} "
        f"ank_hx {led.ank_hx_over or 'under'} "
        f"floor_over {over or 'under'} "
        f"adds_roll L {adds['L']['roll']:+.5f} R {adds['R']['roll']:+.5f} "
        f"adds_pitch L {adds['L']['pitch']:+.5f} R {adds['R']['pitch']:+.5f} "
        f"on_frac {on_frac:.6f} "
        f"{_geom_bits(tick_b)}"
    )
    if over:
        print(
            "PRED trim_lead STOP floor knee or ankle over 2.33 on the lead: "
            f"{over}. Foot-z sweep was not run. "
            "Less-crouch was not opened. Step 2 did not stall. "
            f"step1 flat_worst {led.flat_toe_mm:+.6f} mm "
            f"t {led.flat_toe_t:.3f} side {led.flat_toe_side} "
            f"flat_hx {led.flat_hx_over or 'under'} "
            f"ank_hx {led.ank_hx_over or 'under'}. "
            "Not kit-safe. Not go-anywhere."
        )
        _require_plant("after_stop", "trim_lead")
        return
    if _bar_clear(led):
        print(
            "PRED trim_lead clear "
            f"{led.name} flat_worst {led.flat_toe_mm:.6f} mm "
            f"t {led.flat_toe_t:.3f} side {led.flat_toe_side}. "
            "Plus 2 mm cleared with knees, ankle pitch, and ankle roll "
            "at or under 2.33 Nm on floor contact. Foot-z sweep was not run. "
            "Not kit-safe. Not go-anywhere."
        )
        _require_plant("after_clear", "trim_lead")
        return
    if led.flat_toe_mm >= 2.0:
        print(
            "PRED trim_lead STOP. The lead toe is not short of +2 mm, "
            "and a joint is over 2.33 Nm. That over is not a floor knee "
            "or ankle, so the foot-z sweep was not run. "
            "Less-crouch was not opened. "
            f"step1 flat_worst {led.flat_toe_mm:+.6f} mm "
            f"flat_hx {led.flat_hx_over or 'under'} "
            f"ank_hx {led.ank_hx_over or 'under'}. "
            "Not kit-safe. Not go-anywhere."
        )
        _require_plant("after_stop", "trim_lead")
        return
    print(
        "PRED trim_lead sweep_from 0.270 mm step 0.100 mm "
        f"cap {_Z_SWEEP_CAP_UM / 1000.0:.3f} mm. "
        "Same trim lead. Same adds. "
        "Stop at the first floor-contact knee or ankle over 2.33 Nm."
    )
    last = led
    last_um = 170
    stopped = ""
    for um in range(270, _Z_SWEEP_CAP_UM + 1, 100):
        z_m = um / 1_000_000.0
        row = _trim_lead_copy(
            adds["L"]["roll"], adds["R"]["roll"],
            adds["L"]["pitch"], adds["R"]["pitch"],
            z_m, True, on_frac,
        )
        step_tick = _worst_tick(row)
        tag = f"z{um / 1000.0:.3f}"
        _print_exact_208(tag, step_tick, row.flat_toe_mm)
        step_over = _print_floor_chain(tag, row)
        print(
            f"PRED trim_lead sweep {row.name} "
            f"z_mm {um / 1000.0:.3f} "
            f"flat_worst {row.flat_toe_mm:+.6f} mm "
            f"t {row.flat_toe_t:.3f} side {row.flat_toe_side} "
            f"centre_mm {float(step_tick['center_z']) * 1000.0:.6f} "
            f"flat_hx {row.flat_hx_over or 'under'} "
            f"ank_hx {row.ank_hx_over or 'under'} "
            f"floor_over {step_over or 'under'} "
            f"plus2_under_2.33 {int(_bar_clear(row))}"
        )
        last = row
        last_um = um
        if step_over:
            stopped = "torque"
            break
        if _bar_clear(row):
            stopped = "clear"
            break
        if row.flat_toe_mm >= 2.0:
            stopped = "toe"
            break
    else:
        stopped = "cap"
    _require_plant("after_sweep", "trim_lead")
    if stopped == "clear":
        print(
            "PRED trim_lead clear "
            f"{last.name} flat_worst {last.flat_toe_mm:.6f} mm "
            f"t {last.flat_toe_t:.3f} side {last.flat_toe_side} "
            f"z_mm {last_um / 1000.0:.3f}. "
            "Plus 2 mm cleared with knees, ankle pitch, and ankle roll "
            "at or under 2.33 Nm on floor contact. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    if stopped == "torque":
        short = int(last.flat_toe_mm < 2.0)
        print(
            "PRED trim_lead STOP floor knee or ankle over 2.33 "
            f"on foot-z {last_um / 1000.0:.3f} mm: {_floor_over(last)}. "
            f"flat_worst {last.flat_toe_mm:+.6f} mm "
            f"t {last.flat_toe_t:.3f} side {last.flat_toe_side} "
            f"still_short {short} "
            f"flat_hx {last.flat_hx_over or 'under'} "
            f"ank_hx {last.ank_hx_over or 'under'}."
        )
        if short:
            _less_crouch_next(
                "The trim lead and the foot-z sweep both stalled short of +2 mm."
            )
        else:
            print(
                "PRED trim_lead less_crouch_next 0. "
                "The toe is not short. The bar did not clear. "
                "init_z_offset was not invented. Not kit-safe. Not go-anywhere."
            )
        return
    if stopped == "toe":
        print(
            "PRED trim_lead STOP. Foot-z reached +2 mm and a joint is over "
            "2.33 Nm. That over is not a floor knee or ankle, so no further "
            "foot-z was added. Less-crouch was not opened. "
            f"z_mm {last_um / 1000.0:.3f} "
            f"flat_worst {last.flat_toe_mm:+.6f} mm "
            f"flat_hx {last.flat_hx_over or 'under'} "
            f"ank_hx {last.ank_hx_over or 'under'}. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    _less_crouch_next(
        "The trim lead and the foot-z sweep both stalled short of +2 mm "
        f"under 2.33 Nm. Last foot-z was {last_um / 1000.0:.3f} mm, "
        f"flat worst {last.flat_toe_mm:+.6f} mm."
    )


# Restored constant level trim from the landed A10+025 sign check.
# Roll pair matched. Pitch stays per foot. The trim lead is not reapplied.
_RESTORED_ROLL_L = 0.00662
_RESTORED_ROLL_R = -0.00662
_RESTORED_PITCH_L = 0.00210
_RESTORED_PITCH_R = -0.00069
_LEVEL_FOOTZ_UM = 1170


def _print_ank_axes() -> None:
    """Plant joint axes. Read only. The file is not written."""
    model = mj.MjModel.from_xml_path(SCENE)
    for jn in ("l_ank_roll", "r_ank_roll", "l_ank_pitch", "r_ank_pitch"):
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
        if jid < 0:
            raise SystemExit(f"missing joint {jn}")
        axis = model.jnt_axis[jid]
        act = jn + "_pos"
        idx = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, act)
        if idx < 0:
            raise SystemExit(f"missing actuator {act}")
        lo = float(model.actuator_ctrlrange[idx, 0])
        hi = float(model.actuator_ctrlrange[idx, 1])
        print(
            f"PRED level_footz axis {jn} "
            f"{float(axis[0]):+.3f} {float(axis[1]):+.3f} {float(axis[2]):+.3f} "
            f"ctrlrange {lo:+.3f} {hi:+.3f}"
        )


def _tilt_check_mm(tick: dict[str, object]) -> float:
    """67.5 mm times |pitch| plus 38 mm times |roll|. A check, not the toe."""
    return 67.5 * abs(float(tick["pitch"])) + 38.0 * abs(float(tick["roll"]))


def score_level_footz() -> None:
    """Constant level trim plus 1.170 mm foot-z. The trim lead stays off.

    Less-crouch is not opened here. A derivative lead is the next copy only
    if this one is still short of +2 mm with floor knees and ankles under
    2.33 Nm.
    """
    z_m = _LEVEL_FOOTZ_UM / 1_000_000.0
    print(
        "PRED level_footz_plan trim-lead is off. "
        "Restored constant level trim from the landed sign check: "
        f"roll L {_RESTORED_ROLL_L:+.5f} R {_RESTORED_ROLL_R:+.5f} "
        f"pitch L {_RESTORED_PITCH_L:+.5f} R {_RESTORED_PITCH_R:+.5f} rad. "
        "Cap stays ±0.025 rad. Not a world-level sole. "
        f"Foot-z command {_LEVEL_FOOTZ_UM / 1000.0:.3f} mm on A10+025. "
        "Hip pitch lead stays 20 ms. Roll lead stays 0. "
        "Sag cancel stays off. Period stays 0.500 s. "
        "MFG check, not a result: toe about +2.15 mm if the sole were level. "
        "Less-crouch is not this copy."
    )
    _require_plant("before", "level_footz")
    _print_ank_axes()
    row = _level_trim_copy(
        _RESTORED_ROLL_L, _RESTORED_ROLL_R,
        _RESTORED_PITCH_L, _RESTORED_PITCH_R,
        z_m,
    )
    _require_plant("after", "level_footz")
    if "trimlead" in row.name:
        raise SystemExit(f"trim lead is still in the name {row.name}")
    tick = _worst_tick(row)
    _print_exact_208("step1", tick, row.flat_toe_mm)
    for side in ("L", "R"):
        foot = _foot_tick(row, side)
        if foot is None:
            print(f"PRED level_footz exact side {side} missed")
            continue
        if foot is tick:
            continue
        _print_exact_208(f"step1_foot_{side}", foot, float(foot["toe_z"]) * 1000.0)
    over = _print_floor_chain("step1", row)
    check = _tilt_check_mm(tick)
    drop = float(tick["drop_fo_m"]) * 1000.0
    margin, hundredths = _margin_hundredths(row.flat_toe_mm)
    print(
        f"PRED level_footz tilt_check side {tick['side']} "
        f"67.5*|pitch|+38*|roll| {check:.6f} mm "
        f"measured_centre_minus_front_outside {drop:.6f} mm "
        f"sole_roll {float(tick['roll']):+.8f} "
        f"sole_pitch {float(tick['pitch']):+.8f} "
        "check_not_toe 1"
    )
    print(
        f"PRED level_footz mfg_check measured_toe_mm {row.flat_toe_mm:.6f} "
        f"guess_mm 2.15 delta_mm {row.flat_toe_mm - 2.15:+.6f} "
        "guess_is_not_the_result 1"
    )
    print(
        f"PRED level_footz step1 {row.name} "
        f"flat_worst {row.flat_toe_mm:+.6f} mm "
        f"t {row.flat_toe_t:.3f} side {row.flat_toe_side} "
        f"vs_plus2_mm {margin:.6f} margin_hundredths {hundredths:.2f} "
        f"flat_hx {row.flat_hx_over or 'under'} "
        f"ank_hx {row.ank_hx_over or 'under'} "
        f"floor_over {over or 'under'} "
        f"z_mm {_LEVEL_FOOTZ_UM / 1000.0:.3f} "
        f"lead 0 "
        f"{_geom_bits(tick)}"
    )
    if over:
        print(
            "PRED level_footz STOP floor knee or ankle over 2.33: "
            f"{over}. Derivative lead was not run. "
            "Less-crouch was not opened. "
            f"flat_worst {row.flat_toe_mm:+.6f} mm "
            f"t {row.flat_toe_t:.3f} side {row.flat_toe_side} "
            f"flat_hx {row.flat_hx_over or 'under'} "
            f"ank_hx {row.ank_hx_over or 'under'}. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    if _bar_clear(row):
        print(
            "PRED level_footz clear "
            f"{row.name} flat_worst {row.flat_toe_mm:.6f} mm "
            f"t {row.flat_toe_t:.3f} side {row.flat_toe_side} "
            f"vs_plus2_mm {margin:.6f} margin_hundredths {hundredths:.2f}. "
            "Plus 2 mm cleared with knees, ankle pitch, and ankle roll "
            "at or under 2.33 Nm on floor contact. "
            "Derivative lead was not run. Less-crouch was not opened. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    if row.flat_toe_mm >= 2.0:
        print(
            "PRED level_footz STOP. The toe is not short of +2 mm, "
            "and a joint is over 2.33 Nm. That over is not a floor knee "
            "or ankle. Derivative lead was not run. "
            "Less-crouch was not opened. "
            f"flat_worst {row.flat_toe_mm:+.6f} mm "
            f"flat_hx {row.flat_hx_over or 'under'} "
            f"ank_hx {row.ank_hx_over or 'under'}. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    print(
        "PRED level_footz short_under_2.33 1. "
        f"flat_worst {row.flat_toe_mm:+.6f} mm "
        f"vs_plus2_mm {margin:.6f} margin_hundredths {hundredths:.2f} "
        f"floor_over under. "
        "Derivative trim lead is next. It was not run in this copy. "
        "Less-crouch was not opened. Not kit-safe. Not go-anywhere."
    )


def _steer_turn_script() -> tuple[sw.DemoSegment, ...]:
    """Straight entrance walk, then a stop, then ±0.25 yaw, then stops.

    The straight piece ends at the same 8.2 s as the cleared level-trim bout,
    so the entrance rug is still in the score. Yaw is the bus cap, 0.25 rad/s.
    """
    cap = sw.VX_FWD_CAP
    yaw = sw.YAW_RATE_CAP
    return (
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(8.2, "vel", cap, 0.0, "forward"),
        sw.DemoSegment(9.4, "stop", 0.0, 0.0, "stop"),
        sw.DemoSegment(14.4, "vel", cap, yaw, "yaw_left"),
        sw.DemoSegment(15.6, "stop", 0.0, 0.0, "stop"),
        sw.DemoSegment(20.6, "vel", cap, -yaw, "yaw_right"),
        sw.DemoSegment(21.6, "stop", 0.0, 0.0, "stop"),
    )


def _steer_corners(
    tag: str,
    sole: dict[str, object],
    centre_mm: float,
    name: str = "steer_turn",
) -> None:
    corners = _corner_map(sole["corners"])
    for label in _CORNER_ORDER:
        z, off = corners[label]
        z_mm = z * 1000.0
        print(
            f"PRED {name} corner {tag} {label} "
            f"z_mm {z_mm:.6f} centre_minus_corner_mm {centre_mm - z_mm:.6f} "
            f"forward_mm {off * 1000.0:.1f}"
        )


def _steer_window(
    rows: list[Tick],
    surface: list[dict[str, object]],
    t_cut: float,
) -> list[dict[str, object]]:
    """One record per swing: its 20-80% toe minimum, plus the lift-off tick."""
    by_t = {round(float(row["t"]), 5): row for row in surface}
    cycles = _swing_cycles(rows, t_cut)
    med_n, _last = _cycle_index(cycles)
    out: list[dict[str, object]] = []
    for step, cyc in enumerate(cycles):
        n = len(cyc)
        if n < 2 or med_n < 2:
            continue
        n_full = med_n if step == len(cycles) - 1 and n < med_n - 1 else n
        mids: list[tuple[float, Tick, dict[str, object]]] = []
        lift: tuple[float, Tick, dict[str, object]] | None = None
        for j, row in enumerate(cyc):
            if row.toe_z is None or row.swing is None:
                continue
            frac = j / (n_full - 1)
            surf = by_t.get(round(row.t, 5))
            if surf is None:
                continue
            plane = surf.get(row.swing)
            sole = surf.get("sole_tick")
            if not isinstance(plane, dict) or not isinstance(sole, dict):
                continue
            packed = (frac, row, surf)
            if lift is None:
                lift = packed
            if 0.20 - 1e-12 <= frac <= 0.80 + 1e-12:
                mids.append(packed)
        if not mids or lift is None:
            continue
        on_rug = any(int(item[2][item[1].swing]["on_rug"]) for item in mids)  # type: ignore[index]
        worst = min(mids, key=lambda item: float(item[1].toe_z or 0.0))
        out.append({
            "step": step,
            "tag": "rug" if on_rug else "flat",
            "worst": worst,
            "lift": lift,
        })
    return out


def _print_steer_tick(
    tag: str,
    packed: tuple[float, Tick, dict[str, object]],
    name: str = "steer_turn",
) -> float:
    frac, row, surf = packed
    side = str(row.swing)
    plane = surf[side]
    sole = surf["sole_tick"]
    assert isinstance(plane, dict) and isinstance(sole, dict)
    toe = float(row.toe_z or 0.0) * 1000.0
    centre = float(sole["center_z"]) * 1000.0
    clear = float(plane["clear"]) * 1000.0
    margin, hundredths = _margin_hundredths(toe)
    print(
        f"PRED {name} {tag} side {side} t {row.t:.3f} "
        f"frac {frac:.6f} seg {surf.get('seg', '')} "
        f"yaw {float(surf.get('yaw', 0.0)):+.5f} phase {surf.get('phase', '')} "
        f"toe_mm {toe:.6f} clear_mm {clear:.6f} "
        f"vs_plus2_mm {margin:.6f} margin_hundredths {hundredths:.2f} "
        f"centre_mm {centre:.6f} "
        f"sole_roll {float(sole['roll']):+.5f} sole_pitch {float(sole['pitch']):+.5f} "
        f"scored {sole['scored_label']} lowest {sole['low_label']} "
        f"contact {plane['contact']} geoms {plane['geoms']} "
        f"surface {plane['surface']} on_rug {plane['on_rug']}"
    )
    _steer_corners(tag, sole, centre, name)
    return toe


def _steer_ramp_script(yaw_cap: float = sw.YAW_RATE_CAP) -> tuple[tuple[sw.DemoSegment, ...], float]:
    """Same straight walk, then yaw on and off across two periods.

    The bus vel stays non-zero so the walk approach stays on. The gait
    command is the boundary latch, not this bus value. Stop and settle
    publish forward so the bus does not snap to stand.
    """
    cap = sw.VX_FWD_CAP
    yaw = float(yaw_cap)
    t = 8.2
    segs = [
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(t, "vel", cap, 0.0, "forward"),
    ]

    def add(dt: float, label: str, yaw_rate: float) -> None:
        nonlocal t
        t = round(t + dt, 3)
        segs.append(sw.DemoSegment(t, "vel", cap, yaw_rate, label))

    add(2.0, "stop", 0.0)
    add(0.5, "settle", 0.0)
    add(2.0, "yaw_left_on", yaw)
    add(4.0, "yaw_left", yaw)
    add(2.0, "stop", 0.0)
    add(0.5, "settle", 0.0)
    add(2.0, "yaw_right_on", -yaw)
    add(4.0, "yaw_right", -yaw)
    add(2.0, "stop", 0.0)
    add(0.5, "settle", 0.0)
    return tuple(segs), t


def _seg_label(script: tuple[sw.DemoSegment, ...], t: float) -> str:
    for seg in script:
        if t < seg.t_end - 1e-9:
            return seg.label
    return script[-1].label


def score_steer_turn() -> None:
    """Snap-yaw bout. Logs the tilt, the damping split, and the kp/kv split."""
    _score_steer(ramp=False)


def score_steer_ramp(yaw_cap: float = sw.YAW_RATE_CAP) -> None:
    """Boundary ramp of yaw and step length. The stand pose is not written."""
    _score_steer(ramp=True, yaw_cap=float(yaw_cap))


def _score_steer(*, ramp: bool, yaw_cap: float = sw.YAW_RATE_CAP) -> None:
    """Level trim plus foot-z 1.170, with ±0.25 yaw and starts/stops.

    Trim lead stays off. Less-crouch stays closed. The bar fails if a
    whole-walk 20-80% toe is under +2 mm, a rug clearance is under +2 mm,
    a floor knee, ankle, or hip roll is over 2.33 Nm, or the unclamped
    hip-roll or knee ask on a turning step or a stop is over 2.33 Nm.
    """
    name = "steer_turn" if not ramp else f"steer_ramp_{int(round(yaw_cap * 100)):03d}"
    if ramp:
        script, end = _steer_ramp_script(yaw_cap)
        plan = (
            f"PRED {name}_plan trim-lead is off. Less-crouch is not this copy. "
            "Stack is the restored constant level trim plus foot-z 1.170 mm on A10+025. "
            "Hip pitch lead stays 20 ms. Roll lead stays 0. Sag cancel stays off. "
            "Period stays 0.500 s. Yaw on and off is latched at OP3 movement "
            "boundaries and spread over 2 periods (1.000 s). "
            "Stop ramps step length and yaw to 0 over those 2 periods, "
            "then settles on the zero step for 1 period. hold_stand is not called. "
            f"Cruise yaw stays ±{yaw_cap:.2f} rad/s. "
            "Vendor gait_manager publishes rot 0 on the forward demo and no yaw "
            "column in the speed table. The phase-boundary latch is the published "
            "smoothing. No tighter rad/s cap is in that set. "
            "Unclamped ask is kp*(q_des-q)-kv*omega."
        )
    else:
        script = _steer_turn_script()
        end = 21.6
        plan = (
            "PRED steer_turn_plan trim-lead is off. Less-crouch is not this copy. "
            "Stack is the restored constant level trim plus foot-z 1.170 mm on A10+025. "
            "Hip pitch lead stays 20 ms. Roll lead stays 0. Sag cancel stays off. "
            "Period stays 0.500 s. "
            "Script is stand, straight through 8.2 s, stop, "
            f"yaw_left +{sw.YAW_RATE_CAP:.2f} rad/s, stop, "
            f"yaw_right -{sw.YAW_RATE_CAP:.2f} rad/s, stop. "
            "Unclamped ask is kp*(q_des-q)-kv*omega before the yaw budget and the stop clamp."
        )
    print(plan)
    _require_plant("before", name)
    held: dict[str, object] = {}
    row = measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=0.025,
        toe_up_shape="front",
        toe_full_frac=0.10,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        whole_toe=True,
        surface_tag=True,
        ka_log=True,
        sag_cancel=False,
        z_profile="phase",
        sole_corner_log=True,
        ank_trim_l=_RESTORED_ROLL_L,
        ank_trim_r=_RESTORED_ROLL_R,
        ank_pitch_trim_l=_RESTORED_PITCH_L,
        ank_pitch_trim_r=_RESTORED_PITCH_R,
        ank_log=True,
        roll_log=True,
        z_extra_m=_LEVEL_FOOTZ_UM / 1_000_000.0,
        t_end=end,
        segments=script,
        steer_out=held,
        command_ramp=ramp,
        yaw_cap=yaw_cap,
    )
    _require_plant("after", name)
    if "trimlead" in row.name:
        raise SystemExit(f"trim lead is still in the name {row.name}")
    surface = held["surface"]
    asks = held["asks"]
    assert isinstance(surface, list) and isinstance(asks, list)
    swings = _steer_window(held["rows"], surface, end + 1.0)  # type: ignore[arg-type]
    yaw_max = {"yaw_left": 0.0, "yaw_right": 0.0}
    for surf in surface:
        seg = str(surf.get("seg", ""))
        if seg in yaw_max:
            yaw_max[seg] = max(yaw_max[seg], abs(float(surf.get("yaw", 0.0))))
    print(
        f"PRED {name} yaw_seen left {yaw_max['yaw_left']:+.5f} "
        f"right {yaw_max['yaw_right']:+.5f} "
        f"fault {held['fault'] or 'none'} t {float(held['fault_t']):.3f}"
    )
    fails: list[str] = []
    foot_best: dict[str, dict[str, object]] = {}
    rug_best: dict[str, object] | None = None
    for item in swings:
        worst = item["worst"]
        assert isinstance(worst, tuple)
        side = str(worst[1].swing)
        toe = float(worst[1].toe_z or 0.0)
        prev = foot_best.get(side)
        if prev is None or toe < float(prev["toe"]):
            foot_best[side] = {"toe": toe, "item": item}
        if item["tag"] == "rug":
            plane = worst[2][side]
            clear = float(plane["clear"])
            if rug_best is None or clear < float(rug_best["clear"]):
                rug_best = {"clear": clear, "item": item}
    for side in ("L", "R"):
        packed = foot_best.get(side)
        if packed is None:
            print(f"PRED {name} foot {side} missed")
            fails.append(f"{side} swing missed")
            continue
        item = packed["item"]
        assert isinstance(item, dict)
        toe = _print_steer_tick(f"worst_{side}", item["worst"], name)
        _print_steer_tick(f"liftoff_{side}", item["lift"], name)
        margin, hundredths = _margin_hundredths(toe)
        if toe < 2.0:
            fails.append(
                f"{side} toe {toe:.6f} mm t {item['worst'][1].t:.3f} "
                f"vs_plus2_mm {margin:.6f} margin_hundredths {hundredths:.2f}"
            )
    if rug_best is None:
        print(f"PRED {name} rug missed")
        fails.append("rug swing missed")
    else:
        item = rug_best["item"]
        assert isinstance(item, dict)
        _print_steer_tick("rug_worst", item["worst"], name)
        _print_steer_tick("rug_liftoff", item["lift"], name)
        clear = float(rug_best["clear"]) * 1000.0
        margin, hundredths = _margin_hundredths(clear)
        print(
            f"PRED {name} rug_clear_mm {clear:.6f} "
            f"vs_plus2_mm {margin:.6f} margin_hundredths {hundredths:.2f}"
        )
        if clear < 2.0:
            fails.append(
                f"rug clear {clear:.6f} mm vs_plus2_mm {margin:.6f} "
                f"margin_hundredths {hundredths:.2f}"
            )
    floor_acts = (
        "l_knee", "r_knee", "l_ank_pitch", "r_ank_pitch",
        "l_ank_roll", "r_ank_roll", "l_hip_roll", "r_hip_roll",
    )
    floor_peak: dict[str, tuple[float, float, str, str]] = {}
    for surf in surface:
        for act in floor_acts:
            if act not in surf:
                continue
            side = "L" if act.startswith("l_") else "R"
            plane = surf[side]
            if not isinstance(plane, dict):
                continue
            contact = str(plane["contact"])
            if contact not in ("floor", "floor+rug"):
                continue
            tau = float(surf[act])
            prev = floor_peak.get(act)
            if prev is None or abs(tau) > abs(prev[0]):
                floor_peak[act] = (tau, float(surf["t"]), contact, str(surf.get("seg", "")))
    for act, (tau, when, contact, seg) in floor_peak.items():
        head = KNEE_NM - abs(tau)
        head_h = math.floor(head * 100.0 + 1e-9) / 100.0
        ge = int(abs(tau) >= KNEE_NM)
        print(
            f"PRED {name} floor {act} {tau:+.4f} t {when:.3f} "
            f"contact {contact} seg {seg} ge_2.33 {ge} "
            f"headroom_nm {head:.6f} headroom_hundredths {head_h:.2f}"
        )
        if ge:
            fails.append(f"floor {act} {tau:+.4f} t {when:.3f} contact {contact} seg {seg}")
    turn_labels = {"yaw_left", "yaw_right", "yaw_left_on", "yaw_right_on"}
    stop_labels = {"stop", "settle"}

    def _ask_bucket(item: tuple) -> str:
        t = float(item[0])
        yaw = float(item[4])
        phase = str(item[5])
        if ramp:
            seg = _seg_label(script, t)
            if seg in turn_labels:
                return "turn"
            if seg in stop_labels:
                return "stop"
            return ""
        turn = abs(yaw) > 1e-3 and phase != "stand"
        if turn:
            return "turn"
        if phase == "stand" and t >= 1.2:
            return "stop"
        return ""

    def _ask_peak(bucket: str) -> dict[str, tuple]:
        best: dict[str, tuple] = {}
        for item in asks:
            if _ask_bucket(item) != bucket:
                continue
            prev = best.get(str(item[1]))
            if prev is None or abs(float(item[3])) > abs(float(prev[3])):
                best[str(item[1])] = item
        return best
    def _near_surf(when: float) -> dict[str, object] | None:
        best: dict[str, object] | None = None
        best_dt = 1e9
        for surf in surface:
            dt = abs(float(surf["t"]) - when)
            if dt < best_dt:
                best_dt = dt
                best = surf
        if best is None or best_dt > 0.012:
            return None
        return best

    for act, (tau, when, _contact, seg) in floor_peak.items():
        if act not in ("l_knee", "r_knee", "l_hip_roll", "r_hip_roll"):
            continue
        near_f = _near_surf(when)
        if near_f is None or f"{act}_damp" not in near_f:
            continue
        damp_f = float(near_f[f"{act}_damp"])
        omega = float(near_f[f"{act}_omega"])
        damp_c = float(near_f[f"{act}_damp_c"])
        passive = float(near_f[f"{act}_passive"])
        total = tau + damp_f
        print(
            f"PRED {name} split {act} t {float(near_f['t']):.3f} "
            f"actuator {tau:+.4f} joint_damping {damp_f:+.4f} "
            f"sum {total:+.4f} omega {omega:+.4f} "
            f"dof_damping {damp_c:.4f} qfrc_passive {passive:+.4f} "
            f"ge_2.33_actuator {int(abs(tau) >= KNEE_NM)} "
            f"ge_2.33_sum {int(abs(total) >= KNEE_NM)} "
            f"seg {seg}"
        )

    for label in ("turn", "stop"):
        for jn, item in _ask_peak(label).items():
            t = float(item[0])
            qdes = float(item[2])
            ask = float(item[3])
            yaw = float(item[4])
            phase = item[5]
            kind = item[6]
            limit = item[7]
            kp_term = float(item[8]) if len(item) > 8 else float("nan")
            kv_term = float(item[9]) if len(item) > 9 else float("nan")
            omega = float(item[10]) if len(item) > 10 else float("nan")
            dom = "kp" if abs(kp_term) >= abs(kv_term) else "kv"
            lim = "none" if limit is None else f"{float(limit):.2f}"
            ge = int(abs(ask) >= KNEE_NM)
            head = KNEE_NM - abs(ask)
            head_h = math.floor(head * 100.0 + 1e-9) / 100.0
            near = _near_surf(t)
            measured = float("nan")
            contact = "-"
            if near is not None and jn in near:
                measured = float(near[jn])
                side = "L" if str(jn).startswith("l_") else "R"
                plane = near.get(side)
                if isinstance(plane, dict):
                    contact = str(plane.get("contact", "-"))
            print(
                f"PRED {name} unclamped {label} {jn} {ask:+.4f} "
                f"t {t:.3f} q_des {qdes:+.5f} "
                f"kp_term {kp_term:+.4f} kv_term {kv_term:+.4f} "
                f"dominant {dom} omega {omega:+.4f} "
                f"yaw {yaw:+.5f} phase {phase} kind {kind} limit {lim} "
                f"seg {_seg_label(script, t)} "
                f"measured {measured:+.4f} contact {contact} "
                f"ge_2.33 {ge} headroom_nm {head:.6f} "
                f"headroom_hundredths {head_h:.2f}"
            )
            if ge:
                fails.append(
                    f"unclamped {label} {jn} {ask:+.4f} t {t:.3f}"
                )
    if yaw_max["yaw_left"] < yaw_cap - 0.005 or yaw_max["yaw_right"] < yaw_cap - 0.005:
        fails.append(
            f"yaw short left {yaw_max['yaw_left']:.5f} right {yaw_max['yaw_right']:.5f}"
        )
    if held["fault"]:
        fails.append(f"fault {held['fault']} t {float(held['fault_t']):.3f}")
    if fails:
        print(
            f"PRED {name} Prefer FAIL. "
            + " | ".join(fails)
            + ". Trim lead stayed off. Less-crouch was not opened. "
            "Foot-z was not raised. Not kit-safe. Not go-anywhere."
        )
        return
    print(
        f"PRED {name} bar_held. Whole-walk 20-80% toes and the rug clearance "
        "are at or over +2 mm. Floor knee, ankle, and hip roll stay under 2.33 Nm. "
        "Unclamped hip-roll and knee asks on turning steps and stops "
        "stay under 2.33 Nm. "
        "Trim lead stayed off. Less-crouch was not opened. "
        "Foot-z was not raised. Not kit-safe. Not go-anywhere."
    )


def _soft_stop_script() -> tuple[tuple[sw.DemoSegment, ...], float]:
    """Straight Day-1 walk, then the soft stop. Yaw is 0 on every segment."""
    t_stop = 8.2
    t_end = t_stop + 0.50 + SOFT_STOP_PERIODS * 0.500 + 2.00
    cap = sw.VX_FWD_CAP
    script = (
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(t_stop, "vel", cap, 0.0, "forward"),
        sw.DemoSegment(t_end, "vel", cap, 0.0, "stop"),
    )
    return script, t_end


def _print_soft_feet(name: str, snap: dict[str, object]) -> None:
    feet = snap["feet"]
    assert isinstance(feet, dict)
    ssp = snap.get("ssp_frac")
    ssp_txt = "DSP" if ssp is None else f"{float(ssp):.3f}"
    print(
        f"PRED {name} tick t {float(snap['t']):.3f} mode {snap['mode']} "
        f"phase {snap['phase']} clock {snap.get('clock', '-')} "
        f"ssp_frac {ssp_txt} gait_t {float(snap['gait_t']):.3f} "
        f"x_cmd {float(snap['x_cmd']):+.5f} x_move {float(snap['x_move']):+.5f} "
        f"body_vx {float(snap['body_vx']):+.4f} yaw {float(snap['yaw']):+.5f}"
    )
    knees = snap["knees"]
    assert isinstance(knees, dict)
    print(
        f"PRED {name} knees "
        f"L q {float(knees['l_q']):+.5f} des {float(knees['l_des']):+.5f} "
        f"R q {float(knees['r_q']):+.5f} des {float(knees['r_des']):+.5f}"
    )
    for side in ("L", "R"):
        foot = feet[side]
        assert isinstance(foot, dict)
        corners = foot["corners"]
        assert isinstance(corners, list)
        centre = float(foot["centre"]) * 1000.0
        bits = []
        for label, z in corners:
            z_mm = float(z) * 1000.0
            bits.append(f"{label} {z_mm:+.6f} centre_minus {centre - z_mm:+.6f}")
        print(
            f"PRED {name} sole {side} centre {centre:+.6f} mm "
            f"roll {float(foot['roll']):+.5f} pitch {float(foot['pitch']):+.5f} "
            f"load {float(foot['load']):.2f} N " + " ".join(bits)
        )


def _soft_t_stop(trace: list[dict[str, object]], stop_t: float) -> float | None:
    """Seconds from the stop command until body speed stays under 0.02 m/s for 0.20 s."""
    after = [row for row in trace if float(row["t"]) + 1e-9 >= stop_t]
    if len(after) < 2:
        return None
    quiet = 0.0
    prev = after[0]
    for row in after[1:]:
        dt = float(row["t"]) - float(prev["t"])
        if abs(float(row["body_vx"])) < SOFT_STOP_SPEED_EPS:
            quiet += dt
            if quiet >= SOFT_STOP_HOLD_S:
                return float(row["t"]) - stop_t
        else:
            quiet = 0.0
        prev = row
    return None


def score_soft_stop() -> None:
    """Straight walk, yaw 0, then the double-support step decay.

    Pass is every unclamped knee or hip-roll ask from the stop command
    onward at or under 2.33 Nm, including the finishing stride, and no
    leg actuator on the plant rail. Dropping the finishing swing is a
    soft-pass. Trim lead stays off.
    """
    name = "soft_stop"
    script, end = _soft_stop_script()
    print(
        f"PRED {name}_plan trim-lead is off. Less-crouch is not this copy. "
        "Stack is the restored constant level trim plus foot-z 1.170 mm on A10+025. "
        "Hip pitch lead stays 20 ms. Roll lead stays 0. Sag cancel stays off. "
        "Period stays 0.500 s. Yaw stays 0 for the whole bout. "
        f"Day-1 vx is {sw.VX_FWD_CAP:.3f} m/s. "
        "Stop finishes the stride, freezes in double support, "
        f"and decays step length over {SOFT_STOP_PERIODS:.0f} periods. "
        "Foot-z was not raised. Turns are not this bout."
    )
    _require_plant("before", name)
    held: dict[str, object] = {}
    measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=0.025,
        toe_up_shape="front",
        toe_full_frac=0.10,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        whole_toe=True,
        surface_tag=True,
        ka_log=True,
        sag_cancel=False,
        z_profile="phase",
        sole_corner_log=True,
        ank_trim_l=_RESTORED_ROLL_L,
        ank_trim_r=_RESTORED_ROLL_R,
        ank_pitch_trim_l=_RESTORED_PITCH_L,
        ank_pitch_trim_r=_RESTORED_PITCH_R,
        ank_log=True,
        roll_log=True,
        z_extra_m=_LEVEL_FOOTZ_UM / 1_000_000.0,
        t_end=end,
        segments=script,
        steer_out=held,
        soft_stop=True,
    )
    _require_plant("after", name)
    trace = held["soft"]
    asks = held["asks"]
    surface = held["surface"]
    assert isinstance(trace, list) and isinstance(asks, list) and isinstance(surface, list)
    fails: list[str] = []
    if not trace:
        print(f"PRED {name} Prefer FAIL. stop trace empty")
        return
    stop_t = float(trace[0]["t"])
    modes = {str(row["mode"]) for row in trace}
    print(
        f"PRED {name} stop_t {stop_t:.3f} end {end:.3f} "
        f"modes {' '.join(sorted(modes))} "
        f"fault {held['fault'] or 'none'} t {float(held['fault_t']):.3f}"
    )
    if "decay" not in modes:
        fails.append("decay did not start")
    if any(abs(float(row["yaw"])) > 1e-3 for row in trace):
        fails.append("yaw left 0")

    def _near_trace(when: float) -> dict[str, object] | None:
        best: dict[str, object] | None = None
        best_dt = 1e9
        for row in trace:
            dt = abs(float(row["t"]) - when)
            if dt < best_dt:
                best_dt = dt
                best = row
        if best is None or best_dt > 0.012:
            return None
        return best

    morph_modes = {"wait", "decay", "hold"}
    morph_best: tuple | None = None
    finish_best: tuple | None = None
    pre_best: tuple | None = None
    post_best: tuple | None = None
    for item in asks:
        if not (str(item[1]).endswith("knee") or str(item[1]).endswith("hip_roll")):
            continue
        t = float(item[0])
        if t + 1e-9 < stop_t:
            if pre_best is None or abs(float(item[3])) > abs(float(pre_best[3])):
                pre_best = item
            continue
        if post_best is None or abs(float(item[3])) > abs(float(post_best[3])):
            post_best = item
        near = _near_trace(t)
        mode = str(near["mode"]) if near is not None else ""
        if mode in morph_modes:
            if morph_best is None or abs(float(item[3])) > abs(float(morph_best[3])):
                morph_best = item
        elif mode == "finish":
            if finish_best is None or abs(float(item[3])) > abs(float(finish_best[3])):
                finish_best = item

    def _print_ask(label: str, item: tuple, *, feet: bool = True) -> None:
        t = float(item[0])
        qdes = float(item[2])
        ask = float(item[3])
        phase = item[5]
        kp_term = float(item[8])
        kv_term = float(item[9])
        omega = float(item[10])
        kp = float(item[11])
        q = qdes - (kp_term / kp if abs(kp) > 1e-9 else 0.0)
        dom = "kp" if abs(kp_term) >= abs(kv_term) else "kv"
        near = _near_trace(t) if feet else None
        mode = str(near["mode"]) if near is not None else "-"
        clock = str(near["clock"]) if near is not None else "-"
        gait_phase = "DSP" if mode in morph_modes else mode
        print(
            f"PRED {name} unclamped {label} {item[1]} {ask:+.4f} "
            f"t {t:.3f} q {q:+.5f} q_des {qdes:+.5f} "
            f"kp_term {kp_term:+.4f} kv_term {kv_term:+.4f} "
            f"dominant {dom} omega {omega:+.4f} kp {kp:.2f} "
            f"phase {phase} clock {clock} gait {gait_phase} mode {mode} "
            f"ge_2.33 {int(abs(ask) >= KNEE_NM)}"
        )
        if near is not None:
            _print_soft_feet(name, near)

    if pre_best is not None:
        _print_ask("walk_before_stop", pre_best, feet=False)
    if morph_best is not None:
        _print_ask("dsp_morph", morph_best)
    else:
        fails.append("no DSP stop ask")
    if finish_best is not None:
        _print_ask("finish_stride", finish_best)
    if post_best is not None and abs(float(post_best[3])) > KNEE_NM + 1e-9:
        fails.append(
            f"unclamped stop {post_best[1]} {float(post_best[3]):+.4f} "
            f"t {float(post_best[0]):.3f}"
        )

    old = _near_trace(8.224)
    if old is not None:
        print(f"PRED {name} old_fail_tick")
        _print_soft_feet(name, old)

    rail_best: tuple[str, float, float] | None = None
    for surf in surface:
        t = float(surf["t"])
        if t + 1e-9 < stop_t:
            continue
        for act in (
            "l_knee", "r_knee", "l_hip_roll", "r_hip_roll",
            "l_ank_pitch", "r_ank_pitch", "l_ank_roll", "r_ank_roll",
        ):
            if act not in surf:
                continue
            tau = float(surf[act])
            if rail_best is None or abs(tau) > abs(rail_best[1]):
                rail_best = (act, tau, t)
    if rail_best is not None:
        act, tau, t = rail_best
        head = PLANT_NM - abs(tau)
        print(
            f"PRED {name} actuator_peak {act} {tau:+.4f} t {t:.3f} "
            f"plant_rail {PLANT_NM:.2f} headroom_nm {head:.6f} "
            f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)} "
            f"ge_2.33 {int(abs(tau) >= KNEE_NM)}"
        )
        if abs(tau) >= PLANT_NM - 1e-3:
            fails.append(f"actuator {act} {tau:+.4f} t {t:.3f}")
    if held["fault"]:
        fails.append(f"fault {held['fault']} t {float(held['fault_t']):.3f}")
    t_stop = _soft_t_stop(trace, stop_t)
    t_txt = "none" if t_stop is None else f"{t_stop:.3f}"
    if fails:
        print(
            f"PRED {name} Prefer FAIL. "
            + " | ".join(fails)
            + f". T_stop {t_txt} s is the body-speed settle and is not a pass. "
            "d_min is not rebuilt. "
            "Yaw stayed 0. Foot-z was not raised. Less-crouch stayed closed. "
            "Trim lead stayed off. Not kit-safe. Not go-anywhere."
        )
        return
    d_note = "none"
    if t_stop is not None:
        d_note = f"{0.150 * t_stop:.4f}"
    print(
        f"PRED {name} CLEAR. Unclamped stop asks from the stop command onward "
        "stay at or under 2.33 Nm. "
        "Leg actuators stay under the ±2.45 Nm plant rail. "
        f"T_stop {t_txt} s from the stop command to body speed under "
        f"{SOFT_STOP_SPEED_EPS:.2f} m/s for {SOFT_STOP_HOLD_S:.2f} s. "
        f"At T_detect 0 the rebuilt d_min is 0.150 * T_stop = {d_note} m. "
        "The old d_min 0.1263 m used T_stop about 0.83 s and is not this stop. "
        "No wall-stop pass is claimed. "
        "Yaw stayed 0. Foot-z was not raised. Less-crouch stayed closed. "
        "Trim lead stayed off. Not kit-safe. Not go-anywhere."
    )


def _inflight_stop_script() -> tuple[tuple[sw.DemoSegment, ...], float]:
    """Straight Day-1 walk, then the inflight freeze. Yaw is 0 on every segment."""
    t_stop = 8.2
    t_end = t_stop + 3.5
    cap = sw.VX_FWD_CAP
    script = (
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(t_stop, "vel", cap, 0.0, "forward"),
        sw.DemoSegment(t_end, "vel", cap, 0.0, "stop"),
    )
    return script, t_end


def score_inflight_stop(y_swap_m: float | None = None) -> None:
    """Freeze the loaded stance chain. Only the airborne swing-z schedule moves.

    Pass is every unclamped leg ask from the stop command onward at or
    under 2.33 Nm, the stance knee and hip targets flat across the stop
    edge, and no leg actuator on the plant rail. Step length stays at the
    walking value until the double-support snap.
    """
    name = "inflight"
    script, end = _inflight_stop_script()
    print(
        f"PRED {name}_plan trim-lead is off. Less-crouch is not this copy. "
        "Stack is the restored constant level trim plus foot-z 1.170 mm on A10+025. "
        "Hip pitch lead stays 20 ms. Roll lead stays 0. Sag cancel stays off. "
        "Period stays 0.500 s. Yaw stays 0 for the whole bout. "
        f"Day-1 vx is {sw.VX_FWD_CAP:.3f} m/s. "
        "On the stop tick the loaded stance leg's six pre-stop targets stay put. "
        "The stance foot and the pelvis rolls stay at that pose. "
        "The airborne foot keeps its pre-stop x/y and roll/pitch. "
        "Only that foot's swing-z schedule advances. "
        "x_move is not zeroed on the stop tick. "
        "At the first tick both feet are over 5 N, the amplitude snaps to 0 "
        "and the stance targets stay put. "
        "The next-stride decay is not this bout. "
        "Foot-z was not raised. Turns are not this bout."
    )
    _require_plant("before", name)
    held: dict[str, object] = {}
    measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=0.025,
        toe_up_shape="front",
        toe_full_frac=0.10,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        whole_toe=True,
        surface_tag=True,
        ka_log=True,
        sag_cancel=False,
        z_profile="phase",
        sole_corner_log=True,
        ank_trim_l=_RESTORED_ROLL_L,
        ank_trim_r=_RESTORED_ROLL_R,
        ank_pitch_trim_l=_RESTORED_PITCH_L,
        ank_pitch_trim_r=_RESTORED_PITCH_R,
        ank_log=True,
        roll_log=True,
        z_extra_m=_LEVEL_FOOTZ_UM / 1_000_000.0,
        t_end=end,
        segments=script,
        steer_out=held,
        inflight_stop=True,
        next_stride_decay=False,
        y_swap_m=y_swap_m,
    )
    _require_plant("after", name)
    trace = held["inflight"]
    edge = held["inflight_edge"]
    asks = held["asks"]
    surface = held["surface"]
    assert isinstance(trace, list) and isinstance(asks, list) and isinstance(surface, list)
    assert isinstance(edge, dict)
    fails: list[str] = []
    if not trace:
        print(f"PRED {name} Prefer FAIL. stop trace empty")
        return
    stop_rows = [row for row in trace if str(row["mode"]) != "pre"]
    if not stop_rows:
        print(f"PRED {name} Prefer FAIL. no stop tick")
        return
    stop_t = float(stop_rows[0]["t"])
    modes = {str(row["mode"]) for row in trace}
    frozen = any(str(row.get("freeze", "")).startswith("swing_xy_rp") for row in trace)
    print(
        f"PRED {name} stop_t {stop_t:.3f} end {end:.3f} "
        f"modes {' '.join(sorted(modes))} "
        f"swing_frozen {int(frozen)} "
        f"fault {held['fault'] or 'none'} t {float(held['fault_t']):.3f}"
    )
    if "snap" not in modes:
        fails.append("DSP snap did not fire")
    if not frozen:
        fails.append("swing foot was not frozen")
    if any(abs(float(row["yaw"])) > 1e-3 for row in trace):
        fails.append("yaw left 0")
    if edge:
        jump = bool(edge.get("jump"))
        ddes = float(edge.get("ddes", float("nan")))
        print(
            "PRED inflight_edge bracket "
            f"pre_t {float(edge.get('pre_t', float('nan'))):.3f} "
            f"post_t {float(edge.get('post_t', float('nan'))):.3f} "
            f"swing {edge.get('swing', '-')} "
            f"jump {int(jump)} "
            f"dx {float(edge.get('dx', float('nan'))):+.5f} "
            f"dy {float(edge.get('dy', float('nan'))):+.5f} "
            f"dz {float(edge.get('dz', float('nan'))):+.5f} "
            f"droll {float(edge.get('droll', float('nan'))):+.5f} "
            f"dpitch {float(edge.get('dpitch', float('nan'))):+.5f} "
            f"dknee_des {ddes:+.5f}"
        )
        print(
            "PRED inflight_edge targets "
            f"pre x {float(edge.get('pre_x', float('nan'))):+.5f} "
            f"y {float(edge.get('pre_y', float('nan'))):+.5f} "
            f"z {float(edge.get('pre_z', float('nan'))):+.5f} "
            f"roll {float(edge.get('pre_roll', float('nan'))):+.5f} "
            f"pitch {float(edge.get('pre_pitch', float('nan'))):+.5f} "
            f"knee_des {float(edge.get('pre_des', float('nan'))):+.5f} "
            f"plan x {float(edge.get('plan_x', float('nan'))):+.5f} "
            f"y {float(edge.get('plan_y', float('nan'))):+.5f} "
            f"z {float(edge.get('plan_z', float('nan'))):+.5f} "
            f"roll {float(edge.get('plan_roll', float('nan'))):+.5f} "
            f"pitch {float(edge.get('plan_pitch', float('nan'))):+.5f} "
            f"knee_des {float(edge.get('plan_des', float('nan'))):+.5f}"
        )
        disc = abs(ddes) >= 0.01
        print(
            "PRED inflight_edge q_des_discontinuity "
            + ("yes" if disc else "no")
            + f" dknee_des {ddes:+.5f}. "
            "Pin is the pre-stop x/y/roll/pitch. z stays on the live plan."
        )
    else:
        fails.append("edge log missing")

    def _near_trace(when: float) -> dict[str, object] | None:
        best: dict[str, object] | None = None
        best_dt = 1e9
        for row in trace:
            dt = abs(float(row["t"]) - when)
            if dt < best_dt:
                best_dt = dt
                best = row
        if best is None or best_dt > 0.012:
            return None
        return best

    leg_suffix = (
        "hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll",
    )
    post_best: tuple | None = None
    worst_by: dict[str, tuple] = {}
    for item in asks:
        if not str(item[1]).endswith(leg_suffix):
            continue
        t = float(item[0])
        if t + 1e-9 < stop_t:
            continue
        if post_best is None or abs(float(item[3])) > abs(float(post_best[3])):
            post_best = item
        prev = worst_by.get(str(item[1]))
        if prev is None or abs(float(item[3])) > abs(float(prev[3])):
            worst_by[str(item[1])] = item

    def _print_ask(label: str, item: tuple) -> None:
        t = float(item[0])
        qdes = float(item[2])
        ask = float(item[3])
        phase = item[5]
        kp_term = float(item[8])
        kv_term = float(item[9])
        omega = float(item[10])
        kp = float(item[11])
        kv = float(item[12])
        q = qdes - (kp_term / kp if abs(kp) > 1e-9 else 0.0)
        dom = "kp" if abs(kp_term) >= abs(kv_term) else "kv"
        near = _near_trace(t)
        mode = str(near["mode"]) if near is not None else "-"
        clock = str(near["clock"]) if near is not None else "-"
        freeze = str(near.get("freeze", "-")) if near is not None else "-"
        print(
            f"PRED {name} unclamped {label} {item[1]} {ask:+.4f} "
            f"t {t:.3f} q {q:+.5f} q_des {qdes:+.5f} "
            f"kp_term {kp_term:+.4f} kv_term {kv_term:+.4f} "
            f"dominant {dom} omega {omega:+.4f} kp {kp:.2f} kv {kv:.4f} "
            f"phase {phase} clock {clock} mode {mode} freeze {freeze} "
            f"ge_2.33 {int(abs(ask) >= KNEE_NM)}"
        )
        if near is not None:
            _print_soft_feet(name, near)
            print(f"PRED {name} freeze {freeze}")

    def _print_bracket(tag: str, when: float) -> None:
        near = _near_trace(when)
        if near is None:
            print(f"PRED {name} bracket {tag} t {when:.3f} missing")
            fails.append(f"bracket {tag} missing")
            return
        print(f"PRED {name} bracket {tag}")
        _print_soft_feet(name, near)
        print(f"PRED {name} freeze {near.get('freeze', '-')}")
        got = False
        for item in asks:
            if abs(float(item[0]) - float(near["t"])) > 1e-6:
                continue
            if not (str(item[1]).endswith("knee") or str(item[1]).endswith("hip_roll")):
                continue
            got = True
            _print_ask(tag, item)
        if not got:
            print(f"PRED {name} bracket {tag} no knee or hip-roll ask")

    if edge:
        _print_bracket("pre_stop", float(edge.get("pre_t", stop_t - 0.008)))
        _print_bracket("stop_tick", float(edge.get("post_t", stop_t)))
    else:
        _print_bracket("pre_stop", stop_t - 0.008)
        _print_bracket("stop_tick", stop_t)
    def _des_of(row: dict[str, object] | None, side: str, suf: str) -> float:
        if row is None:
            return float("nan")
        des = row.get("des")
        if not isinstance(des, dict):
            return float("nan")
        side_des = des.get(side)
        if not isinstance(side_des, dict) or suf not in side_des:
            return float("nan")
        return float(side_des[suf])

    lever_times = (8.192, 8.200, 8.224)
    lever_rows = [(_near_trace(when), when) for when in lever_times]
    base = lever_rows[0][0]
    flat = True
    for suf in ("knee", "hip_roll", "hip_pitch"):
        b = _des_of(base, "L", suf)
        for row, when in lever_rows:
            val = _des_of(row, "L", suf)
            print(
                f"PRED {name} lever L {suf} t {when:.3f} "
                f"q_des {val:+.5f} "
                f"row_t {float(row['t']) if row is not None else float('nan'):.3f} "
                f"mode {row.get('mode') if row is not None else '-'} "
                f"freeze {row.get('freeze') if row is not None else '-'} "
                f"x_move {float(row['x_move']) if row is not None else float('nan'):+.5f} "
                f"pelvis_frozen {int(row.get('pelvis_frozen', 0)) if row is not None else 0} "
                f"pel_r {float(row.get('pel_r', float('nan'))) if row is not None else float('nan'):+.5f} "
                f"pel_l {float(row.get('pel_l', float('nan'))) if row is not None else float('nan'):+.5f} "
                f"ik_knee {float(row.get('ik_stance_knee', float('nan'))) if row is not None else float('nan'):+.5f} "
                f"pin_knee {float(row.get('pin_knee', float('nan'))) if row is not None else float('nan'):+.5f}"
            )
            if not math.isfinite(val) or not math.isfinite(b) or abs(val - b) > 1e-4:
                flat = False
    print(f"PRED {name} lever_flat {int(flat)}")
    if not flat:
        fails.append("stance q_des climbed across the stop edge")
    ik_worst = 0.0
    ik_t = float("nan")
    for row in trace:
        if str(row.get("mode")) == "pre":
            continue
        ik = float(row.get("ik_stance_knee", float("nan")))
        pin = float(row.get("pin_knee", float("nan")))
        if not (math.isfinite(ik) and math.isfinite(pin)):
            continue
        if not math.isfinite(ik_t) or abs(ik - pin) > abs(ik_worst):
            ik_worst = ik - pin
            ik_t = float(row["t"])
    print(
        f"PRED {name} ik_vs_pin delta {ik_worst:+.5f} t {ik_t:.3f} "
        "Written stance q_des is the pin."
    )
    if abs(ik_worst) >= 1e-3:
        fails.append(
            f"stance IK climbed under the pin {ik_worst:+.5f} t {ik_t:.3f}"
        )

    if post_best is not None:
        _print_ask("peak", post_best)
    else:
        fails.append("no stop ask")
    snap_row = _near_trace(8.224)
    if snap_row is not None:
        print(f"PRED {name} bracket snap_tick")
        _print_soft_feet(name, snap_row)
        print(f"PRED {name} freeze {snap_row.get('freeze', '-')}")
    for when in (8.192, 8.200, 8.224):
        for item in asks:
            if abs(float(item[0]) - when) > 1e-6:
                continue
            if not str(item[1]).endswith(leg_suffix):
                continue
            qdes = float(item[2])
            ask = float(item[3])
            kp_term = float(item[8])
            kv_term = float(item[9])
            omega = float(item[10])
            kp = float(item[11])
            kv = float(item[12])
            q = qdes - (kp_term / kp if abs(kp) > 1e-9 else 0.0)
            dom = "kp" if abs(kp_term) >= abs(kv_term) else "kv"
            print(
                f"PRED {name} chain {item[1]} t {when:.3f} "
                f"q {q:+.5f} q_des {qdes:+.5f} ask {ask:+.4f} "
                f"kp_term {kp_term:+.4f} kv_term {kv_term:+.4f} "
                f"dominant {dom} omega {omega:+.4f} kp {kp:.2f} kv {kv:.4f} "
                f"ge_2.33 {int(abs(ask) >= KNEE_NM)}"
            )
    for jn in sorted(worst_by):
        item = worst_by[jn]
        ask = float(item[3])
        over = abs(ask) > KNEE_NM + 1e-9
        qdes = float(item[2])
        kp_term = float(item[8])
        kp = float(item[11])
        q = qdes - (kp_term / kp if abs(kp) > 1e-9 else 0.0)
        print(
            f"PRED {name} leg_worst {jn} {ask:+.4f} t {float(item[0]):.3f} "
            f"q {q:+.5f} q_des {qdes:+.5f} "
            f"kp_term {kp_term:+.4f} kv_term {float(item[9]):+.4f} "
            f"ge_2.33 {int(over)}"
        )
        if over:
            fails.append(
                f"unclamped stop {jn} {ask:+.4f} t {float(item[0]):.3f}"
            )

    rail_best: tuple[str, float, float] | None = None
    for surf in surface:
        t = float(surf["t"])
        if t + 1e-9 < stop_t:
            continue
        for act in (
            "l_knee", "r_knee", "l_hip_roll", "r_hip_roll",
            "l_ank_pitch", "r_ank_pitch", "l_ank_roll", "r_ank_roll",
        ):
            if act not in surf:
                continue
            tau = float(surf[act])
            if rail_best is None or abs(tau) > abs(rail_best[1]):
                rail_best = (act, tau, t)
    if rail_best is not None:
        act, tau, t = rail_best
        head = PLANT_NM - abs(tau)
        print(
            f"PRED {name} actuator_peak {act} {tau:+.4f} t {t:.3f} "
            f"plant_rail {PLANT_NM:.2f} headroom_nm {head:.6f} "
            f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)} "
            f"ge_2.33 {int(abs(tau) >= KNEE_NM)}"
        )
        if abs(tau) >= PLANT_NM - 1e-3:
            fails.append(f"actuator {act} {tau:+.4f} t {t:.3f}")
    if held["fault"]:
        fails.append(f"fault {held['fault']} t {float(held['fault_t']):.3f}")
    t_stop = _soft_t_stop(stop_rows, stop_t)
    t_txt = "none" if t_stop is None else f"{t_stop:.3f}"
    if fails:
        print(
            f"PRED {name} Prefer FAIL. "
            + " | ".join(fails)
            + f". T_stop {t_txt} s is the body-speed settle and is not a pass. "
            "d_min is not rebuilt. "
            "x_move was not zeroed on the stop tick. "
            "The next-stride decay was not taken. "
            "The loaded stance chain stayed pinned. "
            "The airborne foot kept its swing-z schedule. "
            "Yaw stayed 0. Foot-z was not raised. Less-crouch stayed closed. "
            "Trim lead stayed off. Not kit-safe. Not go-anywhere."
        )
        return
    d_note = "none"
    if t_stop is not None:
        d_note = f"{0.150 * t_stop:.4f}"
    print(
        f"PRED {name} CLEAR. Unclamped stop asks from the stop command onward "
        "stay at or under 2.33 Nm on every leg joint. "
        "Stance knee and hip targets stayed flat across the stop edge. "
        "Leg actuators stay under the ±2.45 Nm plant rail. "
        f"T_stop {t_txt} s from the stop command to body speed under "
        f"{SOFT_STOP_SPEED_EPS:.2f} m/s for {SOFT_STOP_HOLD_S:.2f} s. "
        f"At T_detect 0 the rebuilt d_min is 0.150 * T_stop = {d_note} m. "
        "The old d_min 0.1263 m used T_stop about 0.83 s and is not this stop. "
        "No wall-stop pass is claimed. "
        "x_move was not zeroed on the stop tick. "
        "The next-stride decay was not taken. "
        "Yaw stayed 0. Foot-z was not raised. Less-crouch stayed closed. "
        "Trim lead stayed off. Not kit-safe. Not go-anywhere."
    )


_WALK_LEG = (
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
)
_WALK_ROLL = ("l_ank_roll", "r_ank_roll", "l_hip_roll", "r_hip_roll")
_WALK_STEADY_S = 2.0


def _ask_q(item: tuple) -> float:
    qdes = float(item[2])
    kp_term = float(item[8])
    kp = float(item[11])
    if abs(kp) < 1e-9:
        return float("nan")
    return qdes - kp_term / kp


_FLAT_SUF = ("hip_roll", "knee", "ank_roll", "ank_pitch")


def _install_sagittal_slew(
    session: sw.SteerSession,
    rad_s: float,
    joints: tuple[str, ...] = ("knee", "ank_pitch"),
) -> None:
    """Slow stance q_des. The airborne knee keeps the swing-z schedule.

    The cap is radians per second. Double support limits both legs.
    Single support limits the loaded leg only. ``joints`` chooses knee,
    ankle pitch, and, on the continuous walk, hip pitch. The logged ask
    uses this slowed q_des. The stop bout does not put hip pitch here.
    """
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        raise RuntimeError("gait manager missing")
    if rad_s < 0.0:
        raise SystemExit(f"sagittal slew {rad_s} is negative")
    suffixes = tuple(joints)
    allowed = ("knee", "ank_pitch", "hip_pitch")
    if not suffixes or any(name not in allowed for name in suffixes):
        raise SystemExit(
            f"sagittal slew joints {joints} are not knee, ank_pitch, or hip_pitch"
        )
    walker = lipm.op3
    orig = lipm.write_clipped
    orig_fl = lipm.write_force_limited
    prev: dict[str, float] = {}
    cap = float(rad_s) * float(ow.OP3_CTRL_S)
    hip_cap = float(_WALK_HIP_PITCH_RATE) * float(ow.OP3_CTRL_S)
    knee_cap = float(_WALK_KNEE_RATE) * float(ow.OP3_CTRL_S)
    stance_cap = float(_WALK_STANCE_RATE) * float(ow.OP3_CTRL_S)
    slew_stats: dict[str, float] = {
        "hip_clip": 0.0,
        "hip_max_abs_dq": 0.0,
        "knee_clip": 0.0,
        "knee_max_abs_dq": 0.0,
    }
    session._walk_slew_stats = slew_stats  # type: ignore[attr-defined]
    session._sag_cap_rows = []  # type: ignore[attr-defined]
    step_at: dict[str, float] = {}
    stepped_cmd: dict[str, float] = {}
    ctrl_period = float(session.ctrl_dt)

    def _stance_prefix(t_cmd: float) -> str:
        if getattr(session, "_sag_both_stance", False):
            return "both"
        if float(walker.l_ssp_start) < t_cmd <= float(walker.l_ssp_end):
            return "r_"
        if float(walker.r_ssp_start) < t_cmd <= float(walker.r_ssp_end):
            return "l_"
        return "both"

    def _hold_inside(jn: str, proposed: float) -> tuple[float, float, float, int]:
        """Pull q_des onto |kp*err| + |kv*omega| <= 2.33. Direction stays put."""
        q = float(lipm.q(jn))
        jid = mj.mj_name2id(lipm.model, mj.mjtObj.mjOBJ_JOINT, jn)
        omega = float(lipm.data.qvel[int(lipm.model.jnt_dofadr[jid])])
        idx = lipm.act_idx[jn + "_pos"]
        kp = float(lipm.model.actuator_gainprm[idx, 0])
        kv = -float(lipm.model.actuator_biasprm[idx, 2])
        budget = KNEE_NM - abs(kv * omega)
        capped = 0
        if budget <= 0.0 or kp < 1e-6:
            return q, q, omega, 1
        max_err = budget / kp
        err = float(proposed) - q
        held = float(proposed)
        if abs(err) > max_err:
            held = q + math.copysign(max_err, err)
            capped = 1
        return held, q, omega, capped

    def _cap_row(
        jn: str,
        q_now: float,
        q_des: float,
        q_stand: float,
        stepped: float,
        old: float,
        capped: int,
        omega: float,
    ) -> None:
        session._sag_cap_rows.append({  # type: ignore[attr-defined]
            "t": float(lipm.data.time),
            "jn": jn,
            "q": q_now,
            "q_des": float(q_des),
            "q_stand": q_stand,
            "stepped": float(stepped),
            "from": float(old),
            "capped": capped,
            "omega": omega,
        })

    _HIP_STAND_SUFFIX = ("hip_yaw", "hip_roll", "hip_pitch")

    def _stand_side(jn: str) -> bool:
        """True when this leg may leave the freeze and step toward stand.

        The flag is set only after both feet have been over 5 N. A foot
        that is airborne again is left out of the tuple.
        """
        if not getattr(session, "_sag_stand_armed", False):
            return False
        sides = getattr(session, "_sag_stand_sides", ())
        if not isinstance(sides, tuple):
            return False
        if jn.startswith("l_"):
            return "L" in sides
        if jn.startswith("r_"):
            return "R" in sides
        return False

    def _hold_pin(jn: str, q_des: float) -> float:
        pins = getattr(session, "_sag_joint_pin", {})
        if isinstance(pins, dict) and jn in pins:
            return float(pins[jn])
        return float(q_des)

    def _hold_last(jn: str, q_des: float) -> float:
        """Keep the last capped command when a foot leaves the loaded set.

        Falling back to the pre-stop pin retargets the knee. The swing-z
        schedule is a different write and does not come through here.
        """
        held = prev.get(jn)
        if held is not None:
            return float(held)
        return _hold_pin(jn, q_des)

    def _slew_loaded_hip(jn: str, q_des: float) -> float:
        """One 1.90 rad/s step toward stand, then the 2.33 Nm hold.

        The first step starts at the pre-stop pin. A stand write that
        passes q_stand on this tick does not replace that pin.
        """
        pins = getattr(session, "_sag_joint_pin", {})
        if jn not in prev and isinstance(pins, dict) and jn in pins:
            prev[jn] = float(pins[jn])
        held, _q_stand = _toward_stand(jn, float(q_des))
        return held

    def _toward_stand(jn: str, q_des: float) -> tuple[float, float]:
        """One slew step per control tick, then the live torque hold.

        Physics substeps reuse that step and re-hold against the live
        omega. A stand write after a tip otherwise requests q_stand with
        the cap skipped.
        """
        t_now = float(lipm.data.time)
        q_stand = float(lipm.q_stand.get(jn, float(lipm.q(jn))))
        last = step_at.get(jn)
        if last is None or t_now - last >= ctrl_period - 1e-4:
            old = prev.get(jn)
            if old is None:
                old = float(q_des)
            dq = q_stand - old
            if dq > cap:
                stepped = old + cap
            elif dq < -cap:
                stepped = old - cap
            else:
                stepped = q_stand
            stepped_cmd[jn] = float(stepped)
            step_at[jn] = t_now
            origin = float(old)
        else:
            stepped = float(stepped_cmd[jn])
            origin = float(prev.get(jn, stepped))
        held, q_now, omega, capped = _hold_inside(jn, stepped)
        prev[jn] = float(held)
        _cap_row(jn, q_now, held, q_stand, stepped, origin, capped, omega)
        return float(held), q_stand

    def _toward_target(jn: str, target: float) -> float:
        """One 1.90 rad/s step toward the stretched swing-z IK, then the 2.33 Nm hold.

        The first step starts at the pre-stop command. This is not the stance
        slew and it does not aim at the stand pose.
        """
        t_now = float(lipm.data.time)
        last = step_at.get(jn)
        if last is None or t_now - last >= ctrl_period - 1e-4:
            old = prev.get(jn)
            started = getattr(session, "_sag_swing_started", None)
            if not isinstance(started, set):
                started = set()
                session._sag_swing_started = started  # type: ignore[attr-defined]
            if jn not in started:
                # Walk DSP may have left a stale prev. The stop starts at
                # the pre-stop command, then steps toward the stretched IK.
                started.add(jn)
                seed = getattr(session, "_sag_swing_seed", None)
                if isinstance(seed, dict) and jn in seed:
                    old = float(seed[jn])
            if old is None:
                old = float(target)
            dq = float(target) - float(old)
            if dq > cap:
                stepped = float(old) + cap
            elif dq < -cap:
                stepped = float(old) - cap
            else:
                stepped = float(target)
            stepped_cmd[jn] = float(stepped)
            step_at[jn] = t_now
            origin = float(old)
        else:
            stepped = float(stepped_cmd[jn])
            origin = float(prev.get(jn, stepped))
        held, q_now, omega, capped = _hold_inside(jn, stepped)
        prev[jn] = float(held)
        cmd = getattr(session, "_sag_swing_cmd", None)
        if isinstance(cmd, dict):
            cmd[jn] = float(held)
        _cap_row(jn, q_now, held, float(target), stepped, origin, capped, omega)
        return float(held)

    def _toward_walk(jn: str, target: float) -> float:
        """One walk-z step toward the warped swing IK. The stop seed stays unused.

        The step is the walk rate, not the 1.90 stance cap. Physics substeps
        reuse it. There is no torque hold here: the logged ask is the track
        of the stretched z.
        """
        rate = float(getattr(session, "_walk_z_rate", _WALK_Z_RATE))
        walk_cap = rate * float(ow.OP3_CTRL_S)
        t_now = float(lipm.data.time)
        last = step_at.get(jn)
        if last is None or t_now - last >= ctrl_period - 1e-4:
            old = prev.get(jn)
            if old is None:
                old = float(target)
            dq = float(target) - float(old)
            if dq > walk_cap:
                stepped = float(old) + walk_cap
            elif dq < -walk_cap:
                stepped = float(old) - walk_cap
            else:
                stepped = float(target)
            stepped_cmd[jn] = float(stepped)
            step_at[jn] = t_now
        else:
            stepped = float(stepped_cmd[jn])
        prev[jn] = float(stepped)
        return float(stepped)

    def _apply_stop_hip(jn: str, q_des: float) -> float:
        """Loaded hips step toward stand. An airborne hip stays on the pin.

        Knee and ankle pitch are not this path. Their 1.90 rad/s stance
        slew is unchanged. Ankle roll stays on sole-flat when the foot is
        loaded, and on the pin while that foot is airborne.
        """
        if not getattr(session, "_sag_torque_cap", False):
            return float(q_des)
        if not jn.endswith(_HIP_STAND_SUFFIX):
            return float(q_des)
        if _stand_side(jn):
            return _slew_loaded_hip(jn, float(q_des))
        return _hold_last(jn, float(q_des))

    last_cmd: dict[str, float] = {}

    def write(jn: str, q_des: float) -> None:
        if jn.endswith(suffixes):
            stance = _stance_prefix(_cmd_time(walker))
            limited = stance == "both" or jn.startswith(stance)
            # After real double support, an airborne leg is not stance.
            if getattr(session, "_sag_stand_armed", False) and not _stand_side(jn):
                limited = False
            swing_joints = getattr(session, "_sag_swing_joints", ())
            tracking = (
                getattr(session, "_sag_swing_track", False)
                and isinstance(swing_joints, tuple)
                and jn in swing_joints
                and not _stand_side(jn)
            )
            if tracking:
                # The stretched swing-z IK owns these two joints until the
                # foot loads. The stance 1.90 path stays on the loaded leg.
                limited = False
            if limited and getattr(session, "_sag_torque_cap", False):
                q_des, _q_stand = _toward_stand(jn, float(q_des))
            elif (
                getattr(session, "_sag_torque_cap", False)
                and getattr(session, "_sag_stand_armed", False)
                and not _stand_side(jn)
                and jn in prev
            ):
                # Foot left the loaded set. Hold the last capped command.
                # The pre-stop pin is not written again.
                q_des = float(prev[jn])
            elif tracking:
                q_des = _toward_target(jn, float(q_des))
            elif (
                getattr(session, "_sag_torque_cap", False)
                and isinstance(swing_joints, tuple)
                and jn in swing_joints
                and not _stand_side(jn)
                and jn in prev
            ):
                # Foot has landed and stand is not armed on this leg yet.
                q_des = float(prev[jn])
            elif limited:
                # The stop walk still uses the caller's cap. The continuous
                # walk can slow hip pitch without dragging the knee.
                walk_stance = (
                    getattr(session, "_walk_z_stretch", False)
                    and not getattr(session, "_sag_stop_installed", False)
                    and not getattr(session, "_sag_torque_cap", False)
                )
                if walk_stance and jn.endswith("hip_pitch"):
                    step = hip_cap
                elif walk_stance and jn.endswith("knee"):
                    step = knee_cap
                elif walk_stance:
                    step = stance_cap
                else:
                    step = cap
                old = prev.get(jn)
                if old is not None:
                    dq = float(q_des) - float(old)
                    if walk_stance and jn.endswith("hip_pitch"):
                        slew_stats["hip_max_abs_dq"] = max(
                            float(slew_stats["hip_max_abs_dq"]), abs(dq)
                        )
                    elif walk_stance and jn.endswith("knee"):
                        slew_stats["knee_max_abs_dq"] = max(
                            float(slew_stats["knee_max_abs_dq"]), abs(dq)
                        )
                    if dq > step:
                        q_des = float(old) + step
                        if walk_stance and jn.endswith("hip_pitch"):
                            slew_stats["hip_clip"] = float(slew_stats["hip_clip"]) + 1.0
                        elif walk_stance and jn.endswith("knee"):
                            slew_stats["knee_clip"] = float(slew_stats["knee_clip"]) + 1.0
                    elif dq < -step:
                        q_des = float(old) - step
                        if walk_stance and jn.endswith("hip_pitch"):
                            slew_stats["hip_clip"] = float(slew_stats["hip_clip"]) + 1.0
                        elif walk_stance and jn.endswith("knee"):
                            slew_stats["knee_clip"] = float(slew_stats["knee_clip"]) + 1.0
                prev[jn] = float(q_des)
            elif (
                getattr(session, "_walk_z_stretch", False)
                and not getattr(session, "_sag_stop_installed", False)
                and not getattr(session, "_sag_torque_cap", False)
                and not limited
                and not jn.endswith("hip_pitch")
            ):
                # Continuous walk only. Swing knee and ankle pitch follow
                # the warped z. The parked stop does not set this flag.
                q_des = _toward_walk(jn, float(q_des))
            elif (
                not getattr(session, "_sag_torque_cap", False)
                and not getattr(session, "_sag_stop_installed", False)
            ):
                # Pure walk. The swing command is not rate limited, and the
                # next stance step has to start from it. The parked stop
                # keeps the previous memory so that landing stays put.
                prev[jn] = float(q_des)
            last_cmd[jn] = float(q_des)
        else:
            q_des = _apply_stop_hip(jn, float(q_des))
        orig(jn, float(q_des))

    def write_limited(jn: str, q_des: float, limit_nm: float | None = None) -> None:
        # hold_stand and the stand substep call this, not write_clipped.
        # Knee and ankle pitch on a loaded leg keep the 1.90 rad/s cap.
        # An airborne knee keeps the last swing-z command. Hips do not
        # take a one-tick stand write. Loaded ankle roll stays at q,
        # which is the sole-flat rule this path would otherwise skip.
        if jn.endswith(suffixes) and getattr(session, "_sag_torque_cap", False):
            if not getattr(session, "_sag_stand_armed", False) or _stand_side(jn):
                q_des, _q_stand = _toward_stand(jn, float(q_des))
            elif jn in last_cmd:
                q_des = float(last_cmd[jn])
        elif getattr(session, "_sag_torque_cap", False) and jn.endswith("ank_roll"):
            if _stand_side(jn):
                q_des = float(lipm.q(jn))
            else:
                q_des = _hold_pin(jn, float(q_des))
        else:
            q_des = _apply_stop_hip(jn, float(q_des))
        orig_fl(jn, float(q_des), limit_nm)

    lipm.write_clipped = write  # type: ignore[method-assign]
    lipm.write_force_limited = write_limited  # type: ignore[method-assign]
    names = "+".join(name for name in suffixes if name != "hip_pitch")
    walk_split = (
        getattr(session, "_walk_z_stretch", False)
        and not getattr(session, "_sag_stop_installed", False)
        and abs(float(_WALK_KNEE_RATE) - float(_WALK_STANCE_RATE)) > 1e-9
    )
    if walk_split:
        print(
            f"PRED sag_slew {_WALK_STANCE_RATE:.3f} rad/s on stance ank_pitch. "
            f"Per tick {stance_cap:.5f} rad. The swing knee keeps the swing-z schedule. "
            "Foot-z and less-crouch stay put."
        )
        print(
            f"PRED sag_slew knee {_WALK_KNEE_RATE:.3f} rad/s on the loaded leg. "
            f"Per tick {knee_cap:.5f} rad."
        )
    else:
        print(
            f"PRED sag_slew {float(rad_s):.3f} rad/s on stance {names}. "
            f"Per tick {cap:.5f} rad. The swing knee keeps the swing-z schedule. "
            "Foot-z and less-crouch stay put."
        )
    if "hip_pitch" in suffixes:
        print(
            f"PRED sag_slew hip_pitch {_WALK_HIP_PITCH_RATE:.3f} rad/s "
            "on the loaded leg. "
            f"Per tick {hip_cap:.5f} rad. "
            f"Walk stance knee stays at {_WALK_KNEE_RATE:.3f} rad/s. "
            f"Walk stance ankle pitch stays at {_WALK_STANCE_RATE:.3f} rad/s. "
            "The stop bout does not take hip pitch. "
            "Swing hip pitch stays on the gait command."
        )


def _quiet_flat_toe(held: dict[str, object]) -> tuple[float, float, str, float]:
    """Worst flat mid-swing toe from 2 s on. Rug-tagged swings stay out."""
    rows = held.get("rows")
    surface = held.get("surface")
    if not isinstance(rows, list) or not rows:
        return float("nan"), float("nan"), "", float("nan")
    by_t: dict[float, dict[str, object]] = {}
    if isinstance(surface, list):
        for row in surface:
            if isinstance(row, dict):
                by_t[round(float(row["t"]), 5)] = row
    cycles = _swing_cycles(rows, 1.0e9)
    med_n, _last_n = _cycle_index(cycles)
    best: tuple[float, float, str, float] | None = None
    for step, cyc in enumerate(cycles):
        n = len(cyc)
        if n < 2 or med_n < 2:
            continue
        n_full = med_n if step == len(cycles) - 1 and n < med_n - 1 else n
        mids: list[tuple[float, Tick, dict[str, object] | None]] = []
        for j, row in enumerate(cyc):
            if row.toe_z is None or row.swing is None or row.t < _WALK_STEADY_S:
                continue
            frac = j / (n_full - 1)
            if not (0.20 - 1e-12 <= frac <= 0.80 + 1e-12):
                continue
            plane = None
            surf = by_t.get(round(row.t, 5))
            if isinstance(surf, dict):
                packed = surf.get(row.swing)
                if isinstance(packed, dict):
                    plane = packed
            mids.append((frac, row, plane))
        if not mids:
            continue
        if any(plane is not None and int(plane["on_rug"]) for _frac, _row, plane in mids):
            continue
        frac, row, _plane = min(mids, key=lambda item: float(item[1].toe_z or 0.0))
        toe = float(row.toe_z or 0.0)
        if best is None or toe < best[0]:
            best = (toe, row.t, row.swing or "", frac)
    if best is None:
        return float("nan"), float("nan"), "", float("nan")
    return best[0] * 1000.0, best[1], best[2], best[3]


def _steady_x(lateral: list[dict[str, float | str]]) -> float:
    """Largest step length after 2 s. The first steady tick is still ramping."""
    best = float("nan")
    for row in lateral:
        if not isinstance(row, dict) or "x_cmd" not in row:
            continue
        if float(row["t_ask"]) < _WALK_STEADY_S:
            continue
        x_cmd = float(row["x_cmd"])
        if best != best or abs(x_cmd) > abs(best):
            best = x_cmd
    return best


def _run_continuous_walk(
    y_swap_m: float,
    t_end: float = 6.5,
    *,
    y_out_m: float | None = None,
    sole_flat: bool = False,
    sole_flat_mode: str = "q",
    x_scale: float = 1.0,
    sag_slew_rad_s: float | None = None,
    sag_slew_joints: tuple[str, ...] = ("knee", "ank_pitch"),
    sag_split: bool = False,
    surface_tag: bool = False,
) -> dict[str, object]:
    """Day-1 straight walk, yaw 0, no stop. Same stack as the stance-freeze bout."""
    held: dict[str, object] = {}
    cap = sw.VX_FWD_CAP
    segments = (
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(t_end, "vel", cap, 0.0, "forward"),
    )
    measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=0.025,
        toe_up_shape="front",
        toe_full_frac=0.10,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        sag_cancel=False,
        z_profile="phase",
        ank_trim_l=_RESTORED_ROLL_L,
        ank_trim_r=_RESTORED_ROLL_R,
        ank_pitch_trim_l=_RESTORED_PITCH_L,
        ank_pitch_trim_r=_RESTORED_PITCH_R,
        z_extra_m=_LEVEL_FOOTZ_UM / 1_000_000.0,
        t_end=t_end,
        segments=segments,
        steer_out=held,
        y_swap_m=y_swap_m,
        lateral_log=True,
        sole_flat=sole_flat,
        sole_flat_mode=sole_flat_mode,
        y_out_m=y_out_m,
        x_scale=x_scale,
        sag_slew_rad_s=sag_slew_rad_s,
        sag_slew_joints=sag_slew_joints,
        sag_split=sag_split,
        surface_tag=surface_tag,
    )
    summary = _walk_lateral_summary(held, y_swap_m)
    summary["y_out"] = 0.005 if y_out_m is None else float(y_out_m)
    summary["sole_flat"] = int(sole_flat)
    summary["sole_mode"] = sole_flat_mode if sole_flat else ""
    summary["x_scale"] = float(x_scale)
    summary["sag_slew"] = float("nan") if sag_slew_rad_s is None else float(sag_slew_rad_s)
    summary["slew_joints"] = "" if sag_slew_rad_s is None else "+".join(sag_slew_joints)
    toe_mm, toe_t, toe_side, toe_frac = _quiet_flat_toe(held)
    summary["flat_toe_mm"] = toe_mm
    summary["flat_toe_t"] = toe_t
    summary["flat_toe_side"] = toe_side
    summary["flat_toe_frac"] = toe_frac
    return summary


def _walk_lateral_summary(held: dict[str, object], y_swap_m: float) -> dict[str, object]:
    """DSP / mid-walk unclamped peaks, gait-vs-trim split, CoM box margin."""
    asks = held.get("asks")
    lateral = held.get("lateral")
    if not isinstance(asks, list) or not isinstance(lateral, list):
        raise RuntimeError("walk lateral log missing")
    by_t: dict[float, dict[str, float | str]] = {}
    for row in lateral:
        if not isinstance(row, dict):
            continue
        by_t[round(float(row["t_ask"]), 5)] = row
    joined = 0
    dsp: dict[str, tuple] = {}
    dsp_late: dict[str, tuple] = {}
    ssp: dict[str, tuple] = {}
    startup: dict[str, tuple] = {}
    mid_stance: dict[str, tuple] = {}
    dsp_loaded: dict[str, tuple] = {}

    def _keep(bucket: dict[str, tuple], item: tuple) -> None:
        prev = bucket.get(str(item[1]))
        if prev is None or abs(float(item[3])) > abs(float(prev[3])):
            bucket[str(item[1])] = item

    for item in asks:
        if str(item[1]) not in _WALK_LEG:
            continue
        t = float(item[0])
        row = by_t.get(round(t, 5))
        if row is None:
            continue
        joined += 1
        phase = str(row["phase"])
        if 1.0 - 1e-9 <= t < _WALK_STEADY_S:
            _keep(startup, item)
        if t < _WALK_STEADY_S - 1e-9:
            continue
        if phase == "D":
            _keep(dsp, item)
            if t >= 3.0 - 1e-9:
                _keep(dsp_late, item)
            jn_d = str(item[1])
            if jn_d.endswith("ank_roll") or jn_d.endswith("hip_roll"):
                side_d = "l" if jn_d.startswith("l_") else "r"
                if float(row[f"fn_{side_d}"]) > _SOLE_FLAT_LOAD_N:
                    _keep(dsp_loaded, item)
        elif phase in ("L", "R"):
            _keep(ssp, item)
            pose = float(row["pose"])
            if phase == "L":
                frac = (pose - 0.025) / 0.200
            else:
                frac = (pose - 0.275) / 0.200
            if 0.25 - 1e-9 <= frac <= 0.75 + 1e-9:
                if phase == "L":
                    fn_st = float(row["fn_r"])
                    fn_sw = float(row["fn_l"])
                    stance_side = "R"
                else:
                    fn_st = float(row["fn_l"])
                    fn_sw = float(row["fn_r"])
                    stance_side = "L"
                jn = str(item[1])
                pref = "l_" if stance_side == "L" else "r_"
                if (
                    fn_st > _SOLE_FLAT_LOAD_N
                    and fn_sw <= _SOLE_FLAT_LOAD_N
                    and jn.startswith(pref)
                    and jn[len(pref):] in _FLAT_SUF
                ):
                    _keep(mid_stance, item)

    def _edge(lo: float, hi: float) -> dict[str, float | str] | None:
        best: dict[str, float | str] | None = None
        best_pose = -1.0
        for row in lateral:
            if not isinstance(row, dict):
                continue
            if float(row["t_ask"]) < _WALK_STEADY_S:
                continue
            if str(row["phase"]) != "D":
                continue
            pose = float(row["pose"])
            if not (lo < pose <= hi + 1e-9):
                continue
            if pose > best_pose:
                best_pose = pose
                best = row
        return best

    def _des_at(row: dict[str, float | str] | None, jn: str) -> tuple | None:
        if row is None:
            return None
        want = round(float(row["t_ask"]), 5)
        found: tuple | None = None
        for item in asks:
            if str(item[1]) != jn:
                continue
            if round(float(item[0]), 5) != want:
                continue
            if found is None or abs(float(item[3])) > abs(float(found[3])):
                found = item
        return found

    split_l = _edge(0.016, 0.025)
    split_r = _edge(0.266, 0.275)

    def _ssp_frac(phase: str, pose: float) -> float:
        if phase == "L":
            start, end = 0.025, 0.225
        else:
            start, end = 0.275, 0.475
        span = end - start
        if span <= 1e-9:
            return float("nan")
        return (pose - start) / span

    def _ss_stats(
        airborne_only: bool,
        frac_lo: float = 0.0,
        frac_hi: float = 1.0,
    ) -> tuple[float, float, float, int, int, dict[str, float | str] | None, float]:
        """Min stance-box slack. Airborne means the swing foot is at or under 5 N."""
        slack_x = float("inf")
        slack_y = float("inf")
        margin = float("inf")
        n = 0
        n_out = 0
        half = float("nan")
        worst: dict[str, float | str] | None = None
        for row in lateral:
            if not isinstance(row, dict):
                continue
            if float(row["t_ask"]) < _WALK_STEADY_S:
                continue
            phase = str(row["phase"])
            if phase == "L":
                fn_stance = float(row["fn_r"])
                fn_swing = float(row["fn_l"])
                sx = float(row["slack_x_r"])
                sy = float(row["slack_y_r"])
                stance = "R"
            elif phase == "R":
                fn_stance = float(row["fn_l"])
                fn_swing = float(row["fn_r"])
                sx = float(row["slack_x_l"])
                sy = float(row["slack_y_l"])
                stance = "L"
            else:
                continue
            if fn_stance <= 5.0:
                continue
            if airborne_only and fn_swing > 5.0:
                continue
            frac = _ssp_frac(phase, float(row["pose"]))
            if not (frac_lo - 1e-9 <= frac <= frac_hi + 1e-9):
                continue
            n += 1
            half = float(row["half_y_l"])
            box = min(sx, sy)
            if box < -1e-4:
                n_out += 1
            if sy < slack_y:
                slack_y = sy
            if sx < slack_x:
                slack_x = sx
            if box < margin:
                margin = box
                worst = {
                    "t": float(row["t_ask"]),
                    "pose": float(row["pose"]),
                    "phase": phase,
                    "stance": stance,
                    "slack_x": sx,
                    "slack_y": sy,
                    "fn": fn_stance,
                    "fn_swing": fn_swing,
                    "swap_y": float(row["swap_y"]),
                    "frac": frac,
                }
        if n == 0:
            return float("nan"), float("nan"), float("nan"), 0, 0, None, float("nan")
        return slack_x, slack_y, margin, n, n_out, worst, half

    ss_slack_x, ss_slack_y, ss_margin, n_ss, n_ss_out, worst_ss, half_y = _ss_stats(False)
    air_x, air_y, air_margin, n_air, n_air_out, worst_air, _air_half = _ss_stats(True)
    # Middle of single support. The first airborne ticks are the transfer,
    # while the body is still moving onto the stance foot.
    mid_x, mid_y, mid_margin, n_mid, n_mid_out, worst_mid, _mid_half = _ss_stats(
        True, 0.25, 0.75,
    )
    corner_z = float("inf")
    corner_side = ""
    corner_t = float("nan")
    center_y_l = float("nan")
    center_y_r = float("nan")
    for row in lateral:
        if not isinstance(row, dict):
            continue
        if center_y_l != center_y_l and "center_y_l" in row:
            center_y_l = float(row["center_y_l"])
            center_y_r = float(row["center_y_r"])
        if float(row["t_ask"]) < _WALK_STEADY_S:
            continue
        for side, fn_key, z_key in (
            ("L", "fn_l", "corner_l"),
            ("R", "fn_r", "corner_r"),
        ):
            if z_key not in row:
                continue
            if float(row[fn_key]) <= _SOLE_FLAT_LOAD_N:
                continue
            z = float(row[z_key])
            if z < corner_z:
                corner_z = z
                corner_side = side
                corner_t = float(row["t_ask"])
    if corner_z == float("inf"):
        corner_z = float("nan")
    return {
        "y": float(y_swap_m),
        "fault": held.get("fault") or "",
        "joined": joined,
        "n_ask": sum(1 for item in asks if str(item[1]) in _WALK_LEG),
        "dsp": dsp,
        "dsp_late": dsp_late,
        "ssp": ssp,
        "startup": startup,
        "split_l": split_l,
        "split_r": split_r,
        "des_l": {jn: _des_at(split_l, jn) for jn in _WALK_ROLL},
        "des_r": {jn: _des_at(split_r, jn) for jn in _WALK_ROLL},
        "ss_slack_x": ss_slack_x,
        "ss_slack_y": ss_slack_y,
        "ss_margin": ss_margin,
        "n_ss": n_ss,
        "n_ss_out": n_ss_out,
        "half_y": half_y,
        "worst_ss": worst_ss,
        "air_slack_x": air_x,
        "air_slack_y": air_y,
        "air_margin": air_margin,
        "n_air": n_air,
        "n_air_out": n_air_out,
        "worst_air": worst_air,
        "mid_slack_x": mid_x,
        "mid_slack_y": mid_y,
        "mid_margin": mid_margin,
        "n_mid": n_mid,
        "n_mid_out": n_mid_out,
        "worst_mid": worst_mid,
        "inside": int(n_mid > 0 and mid_margin >= -1e-4),
        "mid_stance": mid_stance,
        "dsp_loaded": dsp_loaded,
        "corner_z": corner_z,
        "corner_side": corner_side,
        "corner_t": corner_t,
        "dig_in": int(corner_z == corner_z and corner_z < -0.0005),
        "center_y_l": center_y_l,
        "center_y_r": center_y_r,
        "x_cmd": _steady_x(lateral),
        "lateral": lateral,
    }


def _peak_abs(bucket: object, jn: str) -> float:
    if not isinstance(bucket, dict):
        return float("nan")
    item = bucket.get(jn)
    if item is None:
        return 0.0
    return abs(float(item[3]))


def _dsp_roll_ok(summary: dict[str, object]) -> bool:
    dsp = summary["dsp"]
    return all(_peak_abs(dsp, jn) <= KNEE_NM + 1e-9 for jn in _WALK_ROLL)


def _dsp_all_ok(summary: dict[str, object]) -> bool:
    dsp = summary["dsp"]
    return all(_peak_abs(dsp, jn) <= KNEE_NM + 1e-9 for jn in _WALK_LEG)


def _ssp_roll_ok(summary: dict[str, object]) -> bool:
    ssp = summary["ssp"]
    return all(_peak_abs(ssp, jn) <= KNEE_NM + 1e-9 for jn in _WALK_ROLL)


def _print_ask_item(tag: str, item: tuple, phase: str, pose: float) -> None:
    qdes = float(item[2])
    ask = float(item[3])
    kp_term = float(item[8])
    kv_term = float(item[9])
    omega = float(item[10])
    kp = float(item[11])
    kv = float(item[12])
    q = _ask_q(item)
    print(
        f"PRED walk_lat {tag} {item[1]} {ask:+.4f} "
        f"t {float(item[0]):.3f} pose {pose:.3f} phase {phase} "
        f"q {q:+.5f} q_des {qdes:+.5f} "
        f"kp_term {kp_term:+.4f} kv_term {kv_term:+.4f} "
        f"omega {omega:+.4f} kp {kp:.2f} kv {kv:.4f} "
        f"ge_2.33 {int(abs(ask) > KNEE_NM + 1e-9)}"
    )


def _print_split(tag: str, row: object, des: object) -> None:
    if not isinstance(row, dict) or not isinstance(des, dict):
        print(f"PRED walk_lat split {tag} missing")
        return
    print(
        f"PRED walk_lat split {tag} t {float(row['t_ask']):.3f} "
        f"pose {float(row['pose']):.5f} phase {row['phase']} "
        f"swap_y {float(row['swap_y']):+.5f} "
        f"y_swap_cmd {float(row['y_swap_cmd']):.4f} "
        f"y_swap_amp {float(row['y_swap_amp']):.5f} "
        f"ep_roll_l {float(row['ep_roll_l']):+.5f} "
        f"ep_roll_r {float(row['ep_roll_r']):+.5f} "
        f"pel_l {float(row['pel_l']):+.5f} pel_r {float(row['pel_r']):+.5f} "
        f"pel_add_l {float(row['pel_add_l']):+.5f} "
        f"pel_add_r {float(row['pel_add_r']):+.5f} "
        f"trim_l {float(row['trim_l']):+.5f} trim_r {float(row['trim_r']):+.5f}"
    )
    gait_key = {
        "l_ank_roll": "gait_l_ank_roll",
        "r_ank_roll": "gait_r_ank_roll",
        "l_hip_roll": "gait_l_hip_roll",
        "r_hip_roll": "gait_r_hip_roll",
    }
    trim_key = {
        "l_ank_roll": "trim_l",
        "r_ank_roll": "trim_r",
        "l_hip_roll": "pel_add_l",
        "r_hip_roll": "pel_add_r",
    }
    for jn, gait_name in gait_key.items():
        item = des.get(jn)
        gait = float(row[gait_name])
        extra = float(row[trim_key[jn]])
        if item is None:
            print(
                f"PRED walk_lat split {tag} {jn} gait {gait:+.5f} "
                f"extra {extra:+.5f} logged_des missing"
            )
            continue
        qdes = float(item[2])
        print(
            f"PRED walk_lat split {tag} {jn} gait {gait:+.5f} "
            f"extra {extra:+.5f} logged_des {qdes:+.5f} "
            f"des_minus_gait {qdes - gait:+.5f} "
            f"q {_ask_q(item):+.5f} ask {float(item[3]):+.4f}"
        )


def _print_walk_detail(summary: dict[str, object]) -> None:
    y = float(summary["y"])
    print(
        f"PRED walk_lat detail y {y:.4f} "
        f"joined {int(summary['joined'])}/{int(summary['n_ask'])} "
        f"fault {summary['fault'] or 'none'} "
        f"half_y {float(summary['half_y']):.4f} "
        f"n_ss {int(summary['n_ss'])} n_ss_out {int(summary['n_ss_out'])} "
        f"ss_slack_y {float(summary['ss_slack_y']):+.5f} "
        f"ss_slack_x {float(summary['ss_slack_x']):+.5f} "
        f"ss_margin {float(summary['ss_margin']):+.5f} "
        f"air_slack_y {float(summary['air_slack_y']):+.5f} "
        f"air_slack_x {float(summary['air_slack_x']):+.5f} "
        f"air_margin {float(summary['air_margin']):+.5f} "
        f"n_air {int(summary['n_air'])} n_air_out {int(summary['n_air_out'])} "
        f"mid_slack_y {float(summary['mid_slack_y']):+.5f} "
        f"mid_slack_x {float(summary['mid_slack_x']):+.5f} "
        f"mid_margin {float(summary['mid_margin']):+.5f} "
        f"n_mid {int(summary['n_mid'])} n_mid_out {int(summary['n_mid_out'])} "
        f"inside {int(summary['inside'])}"
    )
    worst = summary["worst_ss"]
    if isinstance(worst, dict):
        print(
            f"PRED walk_lat com_clock t {float(worst['t']):.3f} "
            f"pose {float(worst['pose']):.5f} phase {worst['phase']} "
            f"stance {worst['stance']} "
            f"slack_x {float(worst['slack_x']):+.5f} "
            f"slack_y {float(worst['slack_y']):+.5f} "
            f"fn {float(worst['fn']):.2f} "
            f"fn_swing {float(worst['fn_swing']):.2f} "
            f"swap_y {float(worst['swap_y']):+.5f}"
        )
    worst_air = summary["worst_air"]
    if isinstance(worst_air, dict):
        print(
            f"PRED walk_lat com_air t {float(worst_air['t']):.3f} "
            f"pose {float(worst_air['pose']):.5f} phase {worst_air['phase']} "
            f"stance {worst_air['stance']} "
            f"slack_x {float(worst_air['slack_x']):+.5f} "
            f"slack_y {float(worst_air['slack_y']):+.5f} "
            f"fn {float(worst_air['fn']):.2f} "
            f"fn_swing {float(worst_air['fn_swing']):.2f} "
            f"swap_y {float(worst_air['swap_y']):+.5f} "
            f"frac {float(worst_air['frac']):.3f}"
        )
    worst_mid = summary["worst_mid"]
    if isinstance(worst_mid, dict):
        print(
            f"PRED walk_lat com_mid t {float(worst_mid['t']):.3f} "
            f"pose {float(worst_mid['pose']):.5f} phase {worst_mid['phase']} "
            f"stance {worst_mid['stance']} "
            f"slack_x {float(worst_mid['slack_x']):+.5f} "
            f"slack_y {float(worst_mid['slack_y']):+.5f} "
            f"fn {float(worst_mid['fn']):.2f} "
            f"fn_swing {float(worst_mid['fn_swing']):.2f} "
            f"swap_y {float(worst_mid['swap_y']):+.5f} "
            f"frac {float(worst_mid['frac']):.3f}"
        )
    _print_split("dsp_L_edge", summary["split_l"], summary["des_l"])
    _print_split("dsp_R_edge", summary["split_r"], summary["des_r"])
    for label, bucket in (("dsp", summary["dsp"]), ("ssp", summary["ssp"]), ("startup", summary["startup"])):
        if not isinstance(bucket, dict):
            continue
        for jn in _WALK_LEG:
            item = bucket.get(jn)
            if item is None:
                continue
            _print_ask_item(label, item, label, float("nan"))


def _print_walk_grid(summary: dict[str, object]) -> None:
    dsp = summary["dsp"] if isinstance(summary["dsp"], dict) else {}
    ssp = summary["ssp"] if isinstance(summary["ssp"], dict) else {}

    def _one(bucket: dict, jn: str) -> str:
        item = bucket.get(jn)
        if item is None:
            return f"{jn} none"
        return f"{jn} {float(item[3]):+.4f} t {float(item[0]):.3f}"

    dsp_worst = None
    for jn in _WALK_LEG:
        item = dsp.get(jn)
        if item is None:
            continue
        if dsp_worst is None or abs(float(item[3])) > abs(float(dsp_worst[3])):
            dsp_worst = item
    ssp_worst = None
    for jn in _WALK_LEG:
        item = ssp.get(jn)
        if item is None:
            continue
        if ssp_worst is None or abs(float(item[3])) > abs(float(ssp_worst[3])):
            ssp_worst = item
    dsp_txt = "none" if dsp_worst is None else (
        f"{dsp_worst[1]} {float(dsp_worst[3]):+.4f} t {float(dsp_worst[0]):.3f}"
    )
    ssp_txt = "none" if ssp_worst is None else (
        f"{ssp_worst[1]} {float(ssp_worst[3]):+.4f} t {float(ssp_worst[0]):.3f}"
    )
    print(
        f"PRED walk_lat grid y {float(summary['y']):.4f} "
        f"dsp_worst {dsp_txt} ssp_worst {ssp_txt} "
        f"dsp_roll {_one(dsp, 'l_ank_roll')} | {_one(dsp, 'r_ank_roll')} | "
        f"{_one(dsp, 'l_hip_roll')} | {_one(dsp, 'r_hip_roll')} "
        f"ssp_roll {_one(ssp, 'l_ank_roll')} | {_one(ssp, 'r_ank_roll')} | "
        f"{_one(ssp, 'l_hip_roll')} | {_one(ssp, 'r_hip_roll')} "
        f"ss_slack_y {float(summary['ss_slack_y']):+.5f} "
        f"ss_slack_x {float(summary['ss_slack_x']):+.5f} "
        f"air_slack_y {float(summary['air_slack_y']):+.5f} "
        f"air_slack_x {float(summary['air_slack_x']):+.5f} "
        f"air_margin {float(summary['air_margin']):+.5f} "
        f"mid_slack_y {float(summary['mid_slack_y']):+.5f} "
        f"mid_slack_x {float(summary['mid_slack_x']):+.5f} "
        f"mid_margin {float(summary['mid_margin']):+.5f} "
        f"half_y {float(summary['half_y']):.4f} "
        f"n_ss {int(summary['n_ss'])} n_air {int(summary['n_air'])} "
        f"n_mid {int(summary['n_mid'])} n_mid_out {int(summary['n_mid_out'])} "
        f"inside {int(summary['inside'])} "
        f"dsp_late_roll "
        f"{_one(summary['dsp_late'] if isinstance(summary.get('dsp_late'), dict) else {}, 'l_ank_roll')} | "
        f"{_one(summary['dsp_late'] if isinstance(summary.get('dsp_late'), dict) else {}, 'r_ank_roll')} | "
        f"{_one(summary['dsp_late'] if isinstance(summary.get('dsp_late'), dict) else {}, 'l_hip_roll')} | "
        f"{_one(summary['dsp_late'] if isinstance(summary.get('dsp_late'), dict) else {}, 'r_hip_roll')} "
        f"dsp_roll_ok {int(_dsp_roll_ok(summary))} "
        f"dsp_all_ok {int(_dsp_all_ok(summary))} "
        f"ssp_roll_ok {int(_ssp_roll_ok(summary))}"
    )


def _steady_over(summary: dict[str, object]) -> bool:
    for bucket in (summary["dsp"], summary["ssp"]):
        if not isinstance(bucket, dict):
            continue
        for item in bucket.values():
            if abs(float(item[3])) > KNEE_NM + 1e-9:
                return True
    return False


def score_stance_slew(y_swap_m: float = 0.020) -> None:
    """Slew pinned stance q_des toward the stand pose once both feet are loaded.

    The walk lever is already over, so this does not rebuild d_min. The
    air window before double support still holds the pin.
    """
    name = "slew"
    script, end = _inflight_stop_script()
    print(
        f"PRED {name} plan y_swap {y_swap_m:.4f} m. "
        "Stop keeps the stance-chain freeze until both feet are over 5 N. "
            "Then pinned q_des slews toward the stand pose over 1.000 s and holds it. "
        "The bar on that window is |kp*(q_des-q)| + |kv*omega| <= 2.33 Nm. "
        "Foot-z stays 1.170 mm. Less-crouch stays closed. Yaw stays 0."
    )
    _require_plant("before", name)
    held: dict[str, object] = {}
    measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=0.025,
        toe_up_shape="front",
        toe_full_frac=0.10,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        sag_cancel=False,
        z_profile="phase",
        ank_trim_l=_RESTORED_ROLL_L,
        ank_trim_r=_RESTORED_ROLL_R,
        ank_pitch_trim_l=_RESTORED_PITCH_L,
        ank_pitch_trim_r=_RESTORED_PITCH_R,
        z_extra_m=_LEVEL_FOOTZ_UM / 1_000_000.0,
        t_end=end,
        segments=script,
        steer_out=held,
        inflight_stop=True,
        next_stride_decay=False,
        y_swap_m=y_swap_m,
        stance_slew=True,
        surface_tag=True,
    )
    _require_plant("after", name)
    trace = held["inflight"]
    asks = held["asks"]
    assert isinstance(trace, list) and isinstance(asks, list)
    slew_rows = [row for row in trace if isinstance(row, dict) and "slew_u" in row]
    if not slew_rows:
        print(f"PRED {name} Prefer FAIL. slew did not start")
        return
    slew_t = float(slew_rows[0]["t"])
    slew_end = slew_t + 1.0
    stop_rows = [row for row in trace if isinstance(row, dict) and str(row.get("mode")) != "pre"]
    stop_t = float(stop_rows[0]["t"]) if stop_rows else float("nan")
    print(
        f"PRED {name} stop_t {stop_t:.3f} slew_t {slew_t:.3f} "
        f"slew_end {slew_end:.3f} "
        f"slew_end_u {float(slew_rows[-1]['slew_u']):.3f} "
        f"n_slew {len(slew_rows)} "
        f"fault {held.get('fault') or 'none'} t {float(held.get('fault_t', float('nan'))):.3f}"
    )
    window_rows = [
        row for row in slew_rows if float(row["t"]) <= slew_end + 1e-9
    ]
    leg_suffix = (
        "hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll",
    )

    def _window(t0: float, t1: float | None) -> tuple | None:
        best = None
        for item in asks:
            if not str(item[1]).endswith(leg_suffix):
                continue
            t = float(item[0])
            if t + 1e-9 < t0:
                continue
            if t1 is not None and t >= t1 - 1e-9:
                continue
            if best is None or abs(float(item[3])) > abs(float(best[3])):
                best = item
        return best

    air_best = _window(stop_t, slew_t)
    slew_best = _window(slew_t, slew_end + 1e-9)
    hold_best = _window(slew_end, None)
    if air_best is not None:
        print(
            f"PRED {name} air_window {air_best[1]} {float(air_best[3]):+.4f} "
            f"t {float(air_best[0]):.3f} q_des {float(air_best[2]):+.5f} "
            f"q {_ask_q(air_best):+.5f} "
            f"kp_term {float(air_best[8]):+.4f} kv_term {float(air_best[9]):+.4f} "
            "pin still held. Both feet were not loaded yet."
        )
    if slew_best is not None:
        print(
            f"PRED {name} slew_ask {slew_best[1]} {float(slew_best[3]):+.4f} "
            f"t {float(slew_best[0]):.3f} q_des {float(slew_best[2]):+.5f} "
            f"q {_ask_q(slew_best):+.5f} "
            f"kp_term {float(slew_best[8]):+.4f} kv_term {float(slew_best[9]):+.4f} "
            f"parts {abs(float(slew_best[8])) + abs(float(slew_best[9])):.4f}"
        )
    if hold_best is not None:
        print(
            f"PRED {name} hold_ask {hold_best[1]} {float(hold_best[3]):+.4f} "
            f"t {float(hold_best[0]):.3f} q_des {float(hold_best[2]):+.5f} "
            f"q {_ask_q(hold_best):+.5f} "
            f"kp_term {float(hold_best[8]):+.4f} kv_term {float(hold_best[9]):+.4f}"
        )
    part_best = None
    n_cap = 0
    n_part = 0
    for row in window_rows:
        parts = row.get("slew_parts")
        if not isinstance(parts, list):
            continue
        for part in parts:
            if not isinstance(part, dict):
                continue
            n_part += 1
            n_cap += int(part.get("capped", 0))
            if part_best is None or float(part["parts"]) > float(part_best["parts"]):
                part_best = part
                part_best = dict(part)
                part_best["t"] = float(row["t"])
    if part_best is not None:
        print(
            f"PRED {name} parts {part_best['jn']} {float(part_best['parts']):.4f} "
            f"t {float(part_best['t']):.3f} "
            f"kp_abs {float(part_best['kp_abs']):.4f} "
            f"kv_abs {float(part_best['kv_abs']):.4f} "
            f"q {float(part_best['q']):+.5f} q_des {float(part_best['q_des']):+.5f} "
            f"pin {float(part_best['pin']):+.5f} "
            f"capped_ticks {n_cap}/{n_part} "
            f"ge_2.33 {int(float(part_best['parts']) > KNEE_NM + 1e-9)}"
        )
    # Contact-box corners through the slew. Dig-in is a corner below the floor.
    low_z = float("inf")
    low_row = None
    low_label = ""
    low_side = ""
    for row in window_rows:
        corners = row.get("corners")
        if not isinstance(corners, dict):
            continue
        for side in ("L", "R"):
            packed = corners.get(side)
            if not isinstance(packed, list):
                continue
            for label, z, _off in packed:
                if float(z) < low_z:
                    low_z = float(z)
                    low_row = row
                    low_label = str(label)
                    low_side = side
    if low_row is not None:
        dig = int(low_z < -0.0005)
        print(
            f"PRED {name} corner_min {low_side} {low_label} z {low_z * 1000.0:+.3f} mm "
            f"t {float(low_row['t']):.3f} slew_u {float(low_row['slew_u']):.3f} "
            f"dig_in {dig}"
        )
        corners = low_row.get("corners")
        if isinstance(corners, dict):
            for side in ("L", "R"):
                packed = corners.get(side)
                if not isinstance(packed, list):
                    continue
                text = " ".join(
                    f"{label} {float(z) * 1000.0:+.3f}" for label, z, _off in packed
                )
                print(f"PRED {name} corners {side} t {float(low_row['t']):.3f} {text}")
        for mark in (0.0, 0.5, 1.0):
            want = slew_t + mark
            near = min(slew_rows, key=lambda row: abs(float(row["t"]) - want))
            if abs(float(near["t"]) - want) > 0.02:
                continue
            packed_corners = near.get("corners")
            if not isinstance(packed_corners, dict):
                continue
            bits = []
            for side in ("L", "R"):
                packed = packed_corners.get(side)
                if not isinstance(packed, list):
                    continue
                bits.append(
                    side + " " + " ".join(
                        f"{label} {float(z) * 1000.0:+.3f}" for label, z, _off in packed
                    )
                )
            print(
                f"PRED {name} corners_at u_target {mark:.1f} "
                f"t {float(near['t']):.3f} slew_u {float(near['slew_u']):.3f} "
                + " | ".join(bits)
            )
    parts_ok = part_best is not None and float(part_best["parts"]) <= KNEE_NM + 1e-9
    ask_ok = slew_best is not None and abs(float(slew_best[3])) <= KNEE_NM + 1e-9
    air_over = air_best is not None and abs(float(air_best[3])) > KNEE_NM + 1e-9
    print(
        f"PRED {name} Prefer FAIL. "
        f"slew_parts_ok {int(parts_ok)} slew_ask_ok {int(ask_ok)} "
        f"air_window_over {int(air_over)}. "
        "The double-support slew stays at the 2.33 Nm cap and the corners stay up. "
        "The airborne tick before that window still holds the 0.16 rad pin. "
        "Walk DSP at y_swap 0.020 is still over 2.33 Nm, so the stop is not a pass. "
        "d_min is not rebuilt. Foot-z was not raised. Less-crouch stayed closed."
    )


def score_walk_lateral() -> None:
    """Continuous Day-1 walk, then a downward y_swap sweep if the ask is over.

    No stop on the walk bouts. The stance-freeze stop is re-scored only
    when a y_swap keeps DSP asks at or under 2.33 Nm and the CoM inside
    the stance contact box. The slew branch is not this function.
    """
    print(
        "PRED walk_lat plan continuous straight walk. No stop command. "
        f"Day-1 vx {sw.VX_FWD_CAP:.3f} m/s. Yaw 0. Period 0.500 s. "
        "Foot-z 1.170 mm. Less-crouch closed. Trim-lead off. "
        "Steady window starts at 2.000 s so the first step is not the DSP score. "
        "Unclamped ask is kp*(q_des-q)-kv*omega. "
        "Level trim is logged separate from the gait ankle-roll command. "
        "CoM margin is the 38 mm contact box on the stance foot while the other foot is airborne (floor+rug at or under 5 N). "
        "Plant, kp, and forcerange are not raised."
    )
    _require_plant("before", "walk_lat")
    base = _run_continuous_walk(0.020)
    _print_walk_detail(base)
    _print_walk_grid(base)
    if not _steady_over(base):
        print(
            "PRED walk_lat CLEAR for the continuous walk. "
            "Steady DSP and mid-walk unclamped asks stay at or under 2.33 Nm. "
            "The stop bout at 8.192 is a different schedule. "
            "y_swap stays 0.020. Slew branch not taken. "
            "d_min is not rebuilt. Foot-z was not raised."
        )
        _require_plant("after", "walk_lat")
        return
    print(
        "PRED walk_lat walk-tracking Prefer FAIL at y_swap 0.020. "
        "A steady unclamped leg ask is over 2.33 Nm with no stop command. "
        "Sweeping y_swap downward. Pelvis stays 5 deg. HIP_FF stays off."
    )
    grid = (0.016, 0.015, 0.014, 0.012, 0.011, 0.008, 0.004, 0.000)
    rows = [base]
    for y in grid:
        summary = _run_continuous_walk(y)
        _print_walk_grid(summary)
        rows.append(summary)
    # Largest y_swap that keeps DSP ankle/hip-roll at or under 2.33 and
    # the CoM inside the stance box. Smaller values are reported too.
    clearers = [
        row for row in rows
        if _dsp_roll_ok(row) and int(row["inside"]) == 1
    ]
    torque_only = [
        row for row in rows
        if _dsp_roll_ok(row) and int(row["inside"]) != 1
    ]
    adopted: dict[str, object] | None = None
    if clearers:
        adopted = max(clearers, key=lambda row: float(row["y"]))
        # Step down from just under the next larger failure, 0.001 m at
        # a time. The first value that still clears is the least cut.
        ys = sorted(float(row["y"]) for row in rows)
        y_lo = float(adopted["y"])
        higher = [y for y in ys if y > y_lo + 1e-9]
        if higher:
            y_hi = min(higher)
            y_try = round(y_hi - 0.001, 4)
            while y_try > y_lo + 5e-4:
                summary = _run_continuous_walk(y_try)
                _print_walk_grid(summary)
                rows.append(summary)
                if _dsp_roll_ok(summary) and int(summary["inside"]) == 1:
                    adopted = summary
                    break
                y_try = round(y_try - 0.001, 4)
        _print_walk_detail(adopted)
        print(
            f"PRED walk_lat adopted y_swap {float(adopted['y']):.4f} m. "
            "Largest swept value with DSP ankle-roll and hip-roll "
            "unclamped asks at or under 2.33 Nm and CoM inside both "
            "stance contact boxes during single support. "
            f"dsp_all_ok {int(_dsp_all_ok(adopted))} "
            f"ssp_roll_ok {int(_ssp_roll_ok(adopted))} "
            f"ss_margin {float(adopted['ss_margin']):+.5f} m."
        )
        if _dsp_all_ok(adopted):
            print(
                f"PRED walk_lat walk lever CLEAR at y_swap {float(adopted['y']):.4f}. "
                "Re-scoring the stance-freeze stop at this y_swap. "
                "Slew branch not taken."
            )
            _require_plant("after", "walk_lat")
            score_inflight_stop(y_swap_m=float(adopted["y"]))
            return
        print(
            "PRED walk_lat walk lever Prefer FAIL. "
            "DSP ankle/hip-roll cleared and the CoM stayed inside, "
            "and another DSP joint is still over 2.33 Nm. "
            "Stop is not re-scored. Slew branch not taken. "
            "Foot-z and less-crouch stay closed. d_min is not rebuilt."
        )
        _require_plant("after", "walk_lat")
        return
    insiders = [row for row in rows if int(row["inside"]) == 1]
    if insiders:
        best_in = max(insiders, key=lambda row: float(row["y"]))
        dsp_in = best_in["dsp"] if isinstance(best_in["dsp"], dict) else {}
        print(
            f"PRED walk_lat inside y_swap {float(best_in['y']):.4f} "
            f"mid_margin {float(best_in['mid_margin']):+.5f} "
            f"dsp_l_ank {_peak_abs(dsp_in, 'l_ank_roll'):.4f} "
            f"dsp_l_hip {_peak_abs(dsp_in, 'l_hip_roll'):.4f} "
            "CoM stays inside the stance box and DSP roll asks stay over 2.33 Nm."
        )
    if torque_only:
        best_torque = max(torque_only, key=lambda row: float(row["y"]))
        print(
            f"PRED walk_lat torque cleared at y_swap {float(best_torque['y']):.4f} "
            f"mid_margin {float(best_torque['mid_margin']):+.5f} "
            f"n_mid_out {int(best_torque['n_mid_out'])}/{int(best_torque['n_mid'])}. "
            "The CoM left the stance contact box. "
            "Lateral has to stay. Slew branch is the stop, at locked y_swap 0.020. "
            "The tipping y_swap is not the stop bout."
        )
        _require_plant("after", "walk_lat")
        score_stance_slew(0.020)
        return
    else:
        def _roll_peak(row: dict[str, object]) -> float:
            return max(_peak_abs(row["dsp"], jn) for jn in _WALK_ROLL)

        best = min(rows, key=_roll_peak)
        print(
            f"PRED walk_lat no y_swap cleared DSP ankle/hip-roll. "
            f"Best under-rail candidate y_swap {float(best['y']):.4f} "
            f"ss_margin {float(best['ss_margin']):+.5f} "
            f"inside {int(best['inside'])}. "
            "Slew branch not taken. Lateral cut did not have to stay "
            "for a torque pass that never arrived. "
            "Stop stays the stance-freeze Prefer FAIL. "
            "d_min is not rebuilt. Foot-z was not raised."
        )
    _require_plant("after", "walk_lat")


def _flat_names() -> tuple[str, ...]:
    return tuple(f"{side}_{suf}" for side in ("l", "r") for suf in _FLAT_SUF)


def _chain_clear(bucket: object) -> bool:
    if not isinstance(bucket, dict):
        return False
    for jn in _flat_names():
        item = bucket.get(jn)
        if item is None or abs(float(item[3])) > KNEE_NM + 1e-9:
            return False
    return True


def _body_bad(summary: dict[str, object]) -> bool:
    """A fallen bout. A dug corner stays in the log and blocks a pass."""
    return bool(str(summary.get("fault") or ""))


def _sole_pass(summary: dict[str, object]) -> bool:
    return (
        _chain_clear(summary.get("dsp"))
        and _chain_clear(summary.get("mid_stance"))
        and int(summary.get("inside") or 0) == 1
        and int(summary.get("dig_in") or 0) == 0
        and not _body_bad(summary)
    )


def _ask_brief(bucket: object, jn: str) -> str:
    if not isinstance(bucket, dict):
        return f"{jn} none"
    item = bucket.get(jn)
    if item is None:
        return f"{jn} none"
    return f"{jn} {float(item[3]):+.4f}@{float(item[0]):.3f}"


def _print_flat_edge(tag: str, row: object) -> None:
    if not isinstance(row, dict) or "flat_des_l" not in row:
        print(f"PRED sole_flat edge {tag} missing")
        return
    side = "l" if tag.startswith("L") else "r"
    print(
        f"PRED sole_flat edge {tag} t {float(row['t_ask']):.3f} "
        f"pose {float(row['pose']):.5f} phase {row['phase']} "
        f"gait_ank {float(row[f'gait_{side}_ank_roll']):+.5f} "
        f"trim {float(row[f'trim_{side}']):+.5f} "
        f"flat {float(row[f'flat_{side}']):.0f} "
        f"des {float(row[f'flat_des_{side}']):+.5f} "
        f"q {float(row[f'flat_q_{side}']):+.5f} "
        f"fn_write {float(row[f'flat_fn_{side}']):.2f} "
        f"fn_l {float(row['fn_l']):.2f} fn_r {float(row['fn_r']):.2f} "
        f"gait_hip {float(row[f'gait_{side}_hip_roll']):+.5f} "
        f"sole_roll {float(row[f'sole_roll_{side}']):+.5f} "
        f"center_y {float(row[f'center_y_{side}']) * 1000.0:+.2f} mm "
        f"swap_y {float(row['swap_y']):+.5f}"
    )


def _print_sole_row(summary: dict[str, object]) -> None:
    dsp = summary.get("dsp")
    mid = summary.get("mid_stance")
    ssp = summary.get("ssp")
    print(
        f"PRED sole_flat row y {float(summary['y']):.4f} "
        f"y_out {float(summary.get('y_out', float('nan'))):.4f} "
        f"mode {summary.get('sole_mode') or '-'} "
        f"dsp_hip {_ask_brief(dsp, 'l_hip_roll')} | {_ask_brief(dsp, 'r_hip_roll')} "
        f"dsp_ank {_ask_brief(dsp, 'l_ank_roll')} | {_ask_brief(dsp, 'r_ank_roll')} "
        f"dsp_knee {_ask_brief(dsp, 'l_knee')} | {_ask_brief(dsp, 'r_knee')} "
        f"dsp_ap {_ask_brief(dsp, 'l_ank_pitch')} | {_ask_brief(dsp, 'r_ank_pitch')} "
        f"loaded_ank {_ask_brief(summary.get('dsp_loaded'), 'l_ank_roll')} | "
        f"{_ask_brief(summary.get('dsp_loaded'), 'r_ank_roll')} "
        f"loaded_hip {_ask_brief(summary.get('dsp_loaded'), 'l_hip_roll')} | "
        f"{_ask_brief(summary.get('dsp_loaded'), 'r_hip_roll')} "
        f"mid_hip {_ask_brief(mid, 'l_hip_roll')} | {_ask_brief(mid, 'r_hip_roll')} "
        f"mid_ank {_ask_brief(mid, 'l_ank_roll')} | {_ask_brief(mid, 'r_ank_roll')} "
        f"mid_knee {_ask_brief(mid, 'l_knee')} | {_ask_brief(mid, 'r_knee')} "
        f"mid_ap {_ask_brief(mid, 'l_ank_pitch')} | {_ask_brief(mid, 'r_ank_pitch')} "
        f"ssp_knee {_ask_brief(ssp, 'r_knee')} | {_ask_brief(ssp, 'l_knee')} "
        f"mid_margin {float(summary['mid_margin']):+.5f} "
        f"mid_slack_x {float(summary['mid_slack_x']):+.5f} "
        f"n_mid {int(summary['n_mid'])} n_mid_out {int(summary['n_mid_out'])} "
        f"inside {int(summary['inside'])} "
        f"half_y {float(summary['half_y']):.4f} "
        f"center_y_l {float(summary.get('center_y_l', float('nan'))) * 1000.0:+.2f} "
        f"center_y_r {float(summary.get('center_y_r', float('nan'))) * 1000.0:+.2f} "
        f"corner {summary.get('corner_side') or '-'} "
        f"z {float(summary.get('corner_z', float('nan'))) * 1000.0:+.3f} mm "
        f"t {float(summary.get('corner_t', float('nan'))):.3f} "
        f"dig_in {int(summary.get('dig_in') or 0)} "
        f"chain {int(_chain_clear(dsp) and _chain_clear(mid))} "
        f"pass {int(_sole_pass(summary))} "
        f"fault {summary.get('fault') or 'none'}"
    )


def _print_sole_pareto(rows: list[dict[str, object]]) -> None:
    print("PRED sole_flat pareto y_swap init_y mode vs CoM margin vs unclamped hip/ankle/knee")
    for summary in rows:
        _print_sole_row(summary)


def score_sole_flat() -> None:
    """Sole-flat loaded ankle. Hip roll keeps y_swap. No stop. No plant edit."""
    print(
        "PRED sole_flat plan continuous straight walk. No stop. "
        f"Day-1 vx {sw.VX_FWD_CAP:.3f} m/s. Yaw 0. Period 0.500 s. "
        "Foot-z 1.170 mm. Less-crouch closed. Trim-lead off. "
        "Loaded ankle roll (floor+rug over 5 N) does not take the y_swap roll. "
        "Hip roll keeps that IK. Hip yaw is not added. "
        "Pass is DSP and mid-SS hip roll, knee, and ankle at or under 2.33 Nm, "
        "with mid-SS CoM inside the stance contact box (half-width 38 mm). "
        "The swing-z knee stays parked. Plant, kp, and forcerange stay put."
    )
    _require_plant("before", "sole_flat")
    rows: list[dict[str, object]] = []

    def run(y: float, y_out: float, mode: str, *, detail: bool = False) -> dict[str, object]:
        summary = _run_continuous_walk(
            y, y_out_m=y_out, sole_flat=True, sole_flat_mode=mode,
        )
        rows.append(summary)
        if detail:
            _print_walk_detail(summary)
            _print_flat_edge("L", summary.get("split_l"))
            _print_flat_edge("R", summary.get("split_r"))
        _print_sole_row(summary)
        return summary

    mode = "q"
    base = run(0.020, 0.005, mode, detail=True)
    if _body_bad(base):
        print(
            "PRED sole_flat measured-q left the body. "
            "Trying ankle-roll q_des = 0 on the loaded foot. Trim on that write stays 0."
        )
        alt = run(0.020, 0.005, "zero", detail=True)
        if not _body_bad(alt):
            mode = "zero"
            base = alt
        elif _body_bad(base):
            _print_sole_pareto(rows)
            print(
                "PRED sole_flat Prefer FAIL. "
                "Both loaded-ankle commands left the body. "
                "y_swap 0.020 m, init_y 0.005 m. "
                "No wider contact box. d_min is not rebuilt. "
                "Foot-z stays 1.170 mm. Less-crouch stayed closed. "
                "The swing-z knee stayed parked."
            )
            _require_plant("after", "sole_flat")
            return
    if _sole_pass(base):
        print(
            f"PRED sole_flat CLEAR y_swap {float(base['y']):.4f} "
            f"init_y {float(base['y_out']):.4f} mode {base['sole_mode']}. "
            "DSP and mid-SS hip roll, knee, and ankle stay at or under 2.33 Nm. "
            "Mid-SS CoM stays inside the stance box. "
            "Stop is not this bout. d_min is not rebuilt. "
            "Foot-z stays 1.170 mm. The swing-z knee stayed parked."
        )
        _require_plant("after", "sole_flat")
        return

    y_out = 0.005
    widen = (0.008, 0.010, 0.012, 0.015, 0.020, 0.025)
    if int(base["inside"]) != 1:
        print(
            "PRED sole_flat CoM is outside the stance box at y_swap 0.020. "
            "Widening init_y before cutting y_swap. Placement only."
        )
        landed = False
        for yo in widen:
            trial = run(0.020, yo, mode)
            if _sole_pass(trial):
                print(
                    f"PRED sole_flat CLEAR y_swap 0.0200 init_y {yo:.4f} mode {mode}. "
                    "Stop is not this bout. d_min is not rebuilt."
                )
                _require_plant("after", "sole_flat")
                return
            if int(trial["inside"]) == 1 and not _body_bad(trial):
                y_out = yo
                base = trial
                landed = True
                break
        if not landed:
            print(
                "PRED sole_flat init_y through 0.025 m still leaves the CoM outside "
                "at y_swap 0.020. Wider placement is not the CoM lever. "
                "Cutting y_swap at the kit init_y, then at 0.025 m. "
                "The loaded ankle stays sole-flat."
            )
    cut_outs: list[float] = []
    if rows:
        best_margin = max(float(row["mid_margin"]) for row in rows)
        for row in rows:
            if abs(float(row["mid_margin"]) - best_margin) <= 1e-9:
                cut_outs.append(float(row["y_out"]))
                break
    if 0.025 not in cut_outs and any(
        abs(float(row["y"]) - 0.020) < 1e-9 and abs(float(row["y_out"]) - 0.025) < 1e-9
        for row in rows
    ):
        cut_outs.append(0.025)
    if not cut_outs:
        cut_outs.append(y_out)
    for cut_out in cut_outs:
        anchor = next(
            (
                row for row in rows
                if abs(float(row["y"]) - 0.020) < 1e-9
                and abs(float(row["y_out"]) - cut_out) < 1e-9
            ),
            base,
        )
        if _chain_clear(anchor.get("dsp")) and _chain_clear(anchor.get("mid_stance")):
            continue
        print(
            f"PRED sole_flat hip or ankle or knee still over 2.33 Nm "
            f"at y_swap 0.0200 init_y {cut_out:.4f}. "
            "Cutting y_swap. The loaded ankle stays sole-flat."
        )
        for y in (0.016, 0.014, 0.012, 0.011, 0.008, 0.004, 0.000):
            trial = run(y, cut_out, mode)
            if _sole_pass(trial):
                print(
                    f"PRED sole_flat CLEAR y_swap {y:.4f} init_y {cut_out:.4f} mode {mode}. "
                    "Stop is not this bout. d_min is not rebuilt. "
                    "Foot-z stays 1.170 mm. The swing-z knee stayed parked."
                )
                _require_plant("after", "sole_flat")
                return
    _print_sole_pareto(rows)
    best = None
    for summary in rows:
        if _body_bad(summary):
            continue
        if best is None:
            best = summary
            continue
        # Prefer inside, then the least-out CoM, then a flat sole, then a lower ask.
        def _rank(row: dict[str, object]) -> tuple:
            dsp = row.get("dsp")
            mid = row.get("mid_stance")
            peak = 0.0
            for bucket in (dsp, mid):
                if not isinstance(bucket, dict):
                    continue
                for jn in _flat_names():
                    item = bucket.get(jn)
                    if item is not None:
                        peak = max(peak, abs(float(item[3])))
            return (
                int(row.get("inside") or 0),
                float(row["mid_margin"]),
                -int(row.get("dig_in") or 0),
                -peak,
                float(row["y"]),
                -float(row.get("y_out") or 0.0),
            )
        if _rank(summary) > _rank(best):
            best = summary
    if best is None:
        best = rows[-1]
    dsp = best.get("dsp") if isinstance(best.get("dsp"), dict) else {}
    mid = best.get("mid_stance") if isinstance(best.get("mid_stance"), dict) else {}
    print(
        "PRED sole_flat Prefer FAIL. "
        f"best y_swap {float(best['y']):.4f} init_y {float(best.get('y_out', float('nan'))):.4f} "
        f"mode {best.get('sole_mode') or '-'} "
        f"inside {int(best.get('inside') or 0)} "
        f"mid_margin {float(best['mid_margin']):+.5f} m "
        f"n_mid_out {int(best['n_mid_out'])}/{int(best['n_mid'])} "
        f"dig_in {int(best.get('dig_in') or 0)} "
        f"corner {best.get('corner_side') or '-'} "
        f"z {float(best.get('corner_z', float('nan'))) * 1000.0:+.3f} mm "
        f"dsp {_ask_brief(dsp, 'l_hip_roll')} {_ask_brief(dsp, 'l_ank_roll')} "
        f"{_ask_brief(dsp, 'l_knee')} "
        f"mid {_ask_brief(mid, 'l_hip_roll')} {_ask_brief(mid, 'l_ank_roll')} "
        f"{_ask_brief(mid, 'r_hip_roll')} {_ask_brief(mid, 'r_ank_roll')}. "
        "No value kept DSP and mid-SS hip roll, knee, and ankle at or under 2.33 Nm "
        "with the CoM inside the 38 mm stance box. "
        "The contact box was not widened. "
        "d_min is not rebuilt. Foot-z stays 1.170 mm. Less-crouch stayed closed. "
        "The swing-z knee stayed parked. "
        "Plant md5 stays 207f3d5e9c6a72e16f7aa0c8d224f75e."
    )
    _require_plant("after", "sole_flat")


_SAG_NAMES = ("l_knee", "r_knee", "l_ank_pitch", "r_ank_pitch")


def _sag_clear(bucket: object) -> bool:
    if not isinstance(bucket, dict):
        return False
    for jn in _SAG_NAMES:
        item = bucket.get(jn)
        if item is None or abs(float(item[3])) > KNEE_NM + 1e-9:
            return False
    return True


def _sag_pass(summary: dict[str, object]) -> bool:
    return (
        _sag_clear(summary.get("dsp"))
        and _sag_clear(summary.get("mid_stance"))
        and not _body_bad(summary)
    )


def _row_at(summary: dict[str, object], t: float) -> dict[str, float | str] | None:
    lateral = summary.get("lateral")
    if not isinstance(lateral, list):
        return None
    want = round(float(t), 5)
    for row in lateral:
        if isinstance(row, dict) and round(float(row["t_ask"]), 5) == want:
            return row
    return None


def _print_sag_split(tag: str, item: object, summary: dict[str, object]) -> None:
    if not isinstance(item, tuple):
        print(f"PRED sag split {tag} missing")
        return
    row = _row_at(summary, float(item[0]))
    if row is None:
        print(f"PRED sag split {tag} row missing t {float(item[0]):.3f}")
        return
    jn = str(item[1])
    side = "l" if jn.startswith("l_") else "r"
    if jn.endswith("knee"):
        kind = "knee"
    elif jn.endswith("ank_pitch"):
        kind = "ank_pitch"
    else:
        kind = "hip_pitch"
    phase = str(row["phase"])
    if phase == "L":
        stance = "r"
    elif phase == "R":
        stance = "l"
    else:
        stance = "both"
    role = "stance" if stance in (side, "both") else "swing"
    pose = float(row["pose"])
    if phase == "L":
        frac = (pose - 0.025) / 0.200
    elif phase == "R":
        frac = (pose - 0.275) / 0.200
    else:
        frac = float("nan")
    q = _ask_q(item)
    qdes = float(item[2])
    q_stand = float(row.get(f"stand_{side}_{kind}", float("nan")))
    q_nox = float(row.get(f"nox_{side}_{kind}", float("nan")))
    gait = float(row.get(f"gait_{side}_{kind}", float("nan")))
    fn = float(row[f"fn_{side}"])
    other = "r" if side == "l" else "l"
    fn_other = float(row[f"fn_{other}"])
    print(
        f"PRED sag split {tag} {jn} ask {float(item[3]):+.4f} "
        f"t {float(item[0]):.3f} pose {pose:.3f} phase {phase} "
        f"frac {frac:.3f} role {role} "
        f"q {q:+.5f} q_des {qdes:+.5f} gait {gait:+.5f} "
        f"q_stand {q_stand:+.5f} q_nox {q_nox:+.5f} "
        f"shape {qdes - q_stand:+.5f} step {qdes - q_nox:+.5f} "
        f"crouch {q_nox - q_stand:+.5f} sag {q - qdes:+.5f} "
        f"kp_term {float(item[8]):+.4f} kv_term {float(item[9]):+.4f} "
        f"omega {float(item[10]):+.4f} kp {float(item[11]):.2f} kv {float(item[12]):.4f} "
        f"fn {fn:.2f} fn_other {fn_other:.2f} "
        f"ep_x {float(row.get(f'ep_x_{side}', float('nan'))):+.5f} "
        f"ep_z {float(row.get(f'ep_z_{side}', float('nan'))):+.5f} "
        f"body_pitch {float(row.get('body_pitch', float('nan'))):+.5f} "
        f"sole_pitch {float(row.get(f'sole_pitch_{side}', float('nan'))):+.5f} "
        f"hip_gait {float(row.get(f'gait_{side}_hip_pitch', float('nan'))):+.5f} "
        f"hip_nox {float(row.get(f'nox_{side}_hip_pitch', float('nan'))):+.5f} "
        f"hip_stand {float(row.get(f'stand_{side}_hip_pitch', float('nan'))):+.5f} "
        f"x_cmd {float(row.get('x_cmd', float('nan'))):+.5f} "
        f"x_move {float(row.get('x_move', float('nan'))):+.5f}"
    )


def _print_sag_row(summary: dict[str, object]) -> None:
    dsp = summary.get("dsp")
    mid = summary.get("mid_stance")
    ssp = summary.get("ssp")
    slew = float(summary.get("sag_slew", float("nan")))
    slew_txt = "off" if slew != slew else f"{slew:.3f}"
    toe = float(summary.get("flat_toe_mm", float("nan")))
    print(
        f"PRED sag row x_scale {float(summary.get('x_scale', float('nan'))):.3f} "
        f"x_cmd {float(summary.get('x_cmd', float('nan'))):+.5f} "
        f"slew {slew_txt} joints {summary.get('slew_joints') or '-'} "
        f"dsp_knee {_ask_brief(dsp, 'l_knee')} | {_ask_brief(dsp, 'r_knee')} "
        f"dsp_ap {_ask_brief(dsp, 'l_ank_pitch')} | {_ask_brief(dsp, 'r_ank_pitch')} "
        f"mid_knee {_ask_brief(mid, 'l_knee')} | {_ask_brief(mid, 'r_knee')} "
        f"mid_ap {_ask_brief(mid, 'l_ank_pitch')} | {_ask_brief(mid, 'r_ank_pitch')} "
        f"ssp_knee {_ask_brief(ssp, 'r_knee')} | {_ask_brief(ssp, 'l_knee')} "
        f"mid_margin {float(summary['mid_margin']):+.5f} "
        f"n_mid_out {int(summary['n_mid_out'])}/{int(summary['n_mid'])} "
        f"flat_toe {toe:+.3f} mm t {float(summary.get('flat_toe_t', float('nan'))):.3f} "
        f"side {summary.get('flat_toe_side') or '-'} "
        f"frac {float(summary.get('flat_toe_frac', float('nan'))):.3f} "
        f"toe_clear {int(toe > TOE_BAR_M * 1000.0)} "
        f"sag_pass {int(_sag_pass(summary))} "
        f"fault {summary.get('fault') or 'none'}"
    )


def _sag_worst(bucket: object, suffix: str) -> tuple | None:
    if not isinstance(bucket, dict):
        return None
    best = None
    for side in ("l", "r"):
        item = bucket.get(f"{side}_{suffix}")
        if item is None:
            continue
        if best is None or abs(float(item[3])) > abs(float(best[3])):
            best = item
    return best


def score_sagittal() -> None:
    """Mid-SS knee and ankle pitch at y_swap 0. Sole-flat stays on. Plant stays cold."""
    print(
        "PRED sag plan continuous straight walk. No stop. "
        f"Day-1 vx {sw.VX_FWD_CAP:.3f} m/s. Yaw 0. Period 0.500 s. "
        "y_swap 0. init_y 0.005 m. Foot-z 1.170 mm. Less-crouch closed. "
        "Trim-lead off. Sole-flat stance ankle stays on. "
        "Pass is DSP and mid-SS knee and ankle pitch at or under 2.33 Nm. "
        "Hip roll and the CoM box are logged and are not this pass. "
        "The swing-z knee stays parked. Plant, kp, and forcerange stay put."
    )
    _require_plant("before", "sag")
    rows: list[dict[str, object]] = []

    def run(
        x_scale: float,
        slew: float | None,
        *,
        detail: bool = False,
        joints: tuple[str, ...] = ("knee", "ank_pitch"),
    ) -> dict[str, object]:
        summary = _run_continuous_walk(
            0.0,
            y_out_m=0.005,
            sole_flat=True,
            sole_flat_mode="q",
            x_scale=x_scale,
            sag_slew_rad_s=slew,
            sag_slew_joints=joints,
            sag_split=True,
            surface_tag=True,
        )
        rows.append(summary)
        _print_sag_row(summary)
        if detail and not _body_bad(summary):
            mid = summary.get("mid_stance")
            dsp = summary.get("dsp")
            ssp = summary.get("ssp")
            _print_sag_split("mid_knee", _sag_worst(mid, "knee"), summary)
            _print_sag_split("mid_ap", _sag_worst(mid, "ank_pitch"), summary)
            _print_sag_split("dsp_knee", _sag_worst(dsp, "knee"), summary)
            _print_sag_split("dsp_ap", _sag_worst(dsp, "ank_pitch"), summary)
            _print_sag_split("ssp_knee", _sag_worst(ssp, "knee"), summary)
        return summary

    base = run(1.0, None, detail=True)
    if _sag_pass(base):
        toe = float(base.get("flat_toe_mm", float("nan")))
        print(
            "PRED sag CLEAR at y_swap 0, x_scale 1.000, slew off. "
            "DSP and mid-SS knee and ankle pitch stay at or under 2.33 Nm. "
            f"Flat mid-swing toe {toe:+.3f} mm. "
            f"toe_clear {int(toe > TOE_BAR_M * 1000.0)}. "
            "Stop is not this bout. d_min is not rebuilt. "
            "Foot-z stays 1.170 mm. Less-crouch stayed closed."
        )
        _require_plant("after", "sag")
        return

    print(
        "PRED sag full step is over 2.33 Nm. "
        "Cutting step length before slowing the stance pitch. "
        "Bus vx stays 0.150 m/s."
    )
    for scale in (0.75, 0.50, 0.35, 0.25, 0.15, 0.00):
        trial = run(scale, None)
        if _body_bad(trial):
            print(f"PRED sag x_scale {scale:.3f} left the body. Stopping the step cut.")
            break
    print(
        "PRED sag pitch-rate cut at the full step. "
        "Stance knee and stance ankle pitch q_des are rate limited. "
        "The swing knee is not."
    )
    for rate in (2.50, 2.00, 1.95, 1.90, 1.75, 1.50, 1.00, 0.60):
        trial = run(1.0, rate)
        if _body_bad(trial):
            print(f"PRED sag slew {rate:.2f} rad/s left the body. Stopping the rate cut.")
            break
    full_step_pass = [
        row for row in rows
        if _sag_pass(row) and abs(float(row.get("x_scale") or 0.0) - 1.0) < 1e-9
        and str(row.get("slew_joints") or "") == "knee+ank_pitch"
    ]
    if full_step_pass:
        mild_rate = max(float(row["sag_slew"]) for row in full_step_pass)
        print(
            f"PRED sag fastest combined slew that holds the bar is {mild_rate:.2f} rad/s. "
            "Trying that cap on the knee alone, then on ankle pitch alone."
        )
        run(1.0, mild_rate, joints=("knee",), detail=True)
        run(1.0, mild_rate, joints=("ank_pitch",), detail=True)

    passing = [row for row in rows if _sag_pass(row)]
    if passing:
        def _mild(row: dict[str, object]) -> tuple:
            slew = row.get("sag_slew")
            slew_v = float(slew) if isinstance(slew, float) else float("nan")
            # Full step outranks a cut. A faster slew outranks a slower one.
            return (
                float(row.get("x_scale") or 0.0),
                0.0 if slew_v != slew_v else slew_v,
            )
        best = max(passing, key=_mild)
        _print_sag_split("mid_knee", _sag_worst(best.get("mid_stance"), "knee"), best)
        _print_sag_split("mid_ap", _sag_worst(best.get("mid_stance"), "ank_pitch"), best)
        _print_sag_split("dsp_knee", _sag_worst(best.get("dsp"), "knee"), best)
        _print_sag_split("dsp_ap", _sag_worst(best.get("dsp"), "ank_pitch"), best)
        _print_sag_split("ssp_knee", _sag_worst(best.get("ssp"), "knee"), best)
        toe = float(best.get("flat_toe_mm", float("nan")))
        slew = float(best.get("sag_slew", float("nan")))
        slew_txt = "off" if slew != slew else f"{slew:.3f} rad/s"
        print(
            "PRED sag CLEAR. "
            f"y_swap 0. x_scale {float(best.get('x_scale', float('nan'))):.3f} "
            f"x_cmd {float(best.get('x_cmd', float('nan'))):+.5f} m "
            f"slew {slew_txt} joints {best.get('slew_joints') or '-'}. "
            "DSP and mid-SS knee and ankle pitch stay at or under 2.33 Nm. "
            f"Flat mid-swing toe {toe:+.3f} mm "
            f"t {float(best.get('flat_toe_t', float('nan'))):.3f} "
            f"side {best.get('flat_toe_side') or '-'}. "
            f"toe_clear {int(toe > TOE_BAR_M * 1000.0)}. "
            "Hip roll and the CoM box are not this pass. "
            "The swing-z knee stayed parked. "
            "Foot-z stays 1.170 mm. Less-crouch stayed closed. "
            "d_min is not rebuilt. "
            "Plant md5 stays 207f3d5e9c6a72e16f7aa0c8d224f75e."
        )
        _require_plant("after", "sag")
        return

    print("PRED sag pareto x_scale slew vs knee and ankle pitch")
    for summary in rows:
        _print_sag_row(summary)
    print(
        "PRED sag Prefer FAIL. "
        "No step-length cut and no stance pitch-rate cut kept DSP and mid-SS "
        "knee and ankle pitch at or under 2.33 Nm at y_swap 0. "
        "The contact box was not widened. Foot-z stayed 1.170 mm. "
        "Less-crouch stayed closed. The swing-z knee stayed parked. "
        "d_min is not rebuilt. "
        "Plant md5 stays 207f3d5e9c6a72e16f7aa0c8d224f75e."
    )
    _require_plant("after", "sag")


def _print_box_pair(name: str, tag: str, row: dict[str, float | str] | None) -> None:
    """CoM slack in both contact boxes. Outside is logged and is not the torque bar."""
    if row is None:
        print(f"PRED {name} com {tag} missing")
        return
    def _out(sx: float, sy: float) -> int:
        return int(min(sx, sy) < -1e-4)

    sx_l = float(row["slack_x_l"])
    sy_l = float(row["slack_y_l"])
    sx_r = float(row["slack_x_r"])
    sy_r = float(row["slack_y_r"])
    print(
        f"PRED {name} com {tag} t {float(row['t_ask']):.3f} "
        f"phase {row['phase']} stance {row['stance']} "
        f"L slack_x {sx_l * 1000.0:+.2f} slack_y {sy_l * 1000.0:+.2f} mm "
        f"out {_out(sx_l, sy_l)} "
        f"R slack_x {sx_r * 1000.0:+.2f} slack_y {sy_r * 1000.0:+.2f} mm "
        f"out {_out(sx_r, sy_r)} "
        f"half_y {float(row['half_y_l']) * 1000.0:.2f} mm "
        f"centre_y L {float(row['center_y_l']) * 1000.0:+.2f} "
        f"R {float(row['center_y_r']) * 1000.0:+.2f} "
        f"fn L {float(row['fn_l']):.2f} R {float(row['fn_r']):.2f} "
        f"corner L {float(row['corner_l']) * 1000.0:+.3f} "
        f"R {float(row['corner_r']) * 1000.0:+.3f} mm"
    )


def score_sag_stop() -> None:
    """Soft stop on the sagittal CLEAR locks. Yaw stays 0.

    Pass is every unclamped leg ask from the stop command onward at or
    under 2.33 Nm, and no leg actuator on the plant rail. Hip roll and
    the CoM box are logged and are not this pass. The y_swap 0 flat toe
    is not the earlier y_swap 0.020 clear.
    """
    name = "sag_stop"
    y_swap = 0.0
    slew = 1.90
    script, end = _inflight_stop_script()
    print(
        f"PRED {name} plan y_swap {y_swap:.3f}. "
        "Sole-flat stance ankle stays on. "
        f"Stance knee and ankle pitch stay on the {slew:.2f} rad/s slew "
        "through the walk and the stop. "
        "On the stop that slew steps toward the stand pose and stays inside "
        "the 2.33 Nm part cap. "
        "From the stop command the swing hip roll, hip yaw, hip pitch, and "
        "ankle roll hold the pre-stop q_des. Planar x/y and sole roll/pitch hold. "
        "The leftover swing-z is time-stretched so the knee and ankle-pitch "
        "IK move at or under 1.90 rad/s. The rise above the end height is "
        "scaled down when the full schedule cannot reach the descent "
        "midpoint before the tip. Those two joints slew toward that IK "
        "inside the 2.33 Nm part cap. The clock waits while either "
        "command is more than one step behind, so z does not run ahead. "
        "Ankle roll stays on the pin until that foot loads, then sole-flat. "
        "Stand hips start only after both feet are over 5 N, and only on "
        "a loaded leg, inside the same part cap. "
        "x_move is not zeroed on the stop tick. "
        "The first real double support snaps the amplitude to 0. "
        "Full step. Foot-z 1.170 mm. Less-crouch closed. Trim-lead off. Yaw 0. "
        "Torque is scored before any tip. "
        "Hip roll and CoM-in-box are logged and are not this pass. "
        "The y_swap 0 toe is not the y_swap 0.020 clear."
    )
    _require_plant("before", name)
    held: dict[str, object] = {}
    measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=0.025,
        toe_up_shape="front",
        toe_full_frac=0.10,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        sag_cancel=False,
        z_profile="phase",
        ank_trim_l=_RESTORED_ROLL_L,
        ank_trim_r=_RESTORED_ROLL_R,
        ank_pitch_trim_l=_RESTORED_PITCH_L,
        ank_pitch_trim_r=_RESTORED_PITCH_R,
        z_extra_m=_LEVEL_FOOTZ_UM / 1_000_000.0,
        t_end=end,
        segments=script,
        steer_out=held,
        inflight_stop=True,
        next_stride_decay=False,
        y_swap_m=y_swap,
        sole_flat=True,
        sole_flat_mode="q",
        sag_slew_rad_s=slew,
        sag_slew_joints=("knee", "ank_pitch"),
        sag_stop_cap=True,
        lateral_log=True,
        surface_tag=True,
    )
    _require_plant("after", name)
    trace = held.get("inflight")
    asks = held.get("asks")
    lateral = held.get("lateral")
    surface = held.get("surface")
    edge = held.get("inflight_edge")
    assert isinstance(trace, list) and isinstance(asks, list)
    assert isinstance(lateral, list) and isinstance(surface, list)
    assert isinstance(edge, dict)
    summary = _walk_lateral_summary(held, y_swap)
    by_t: dict[float, dict[str, float | str]] = {}
    for row in lateral:
        if isinstance(row, dict):
            by_t[round(float(row["t_ask"]), 5)] = row
    cap_rows = held.get("sag_cap")
    # The cap log lives on the session. measure_pred_clip keeps it only if
    # we copied it. Pull it from the steer bag when present.
    if not isinstance(cap_rows, list):
        cap_rows = []

    stop_rows = [row for row in trace if isinstance(row, dict) and str(row.get("mode")) != "pre"]
    if not stop_rows:
        print(f"PRED {name} Prefer FAIL. no stop tick")
        _require_plant("after", name)
        return
    stop_t = float(stop_rows[0]["t"])
    stop_move = float(stop_rows[0].get("x_move", float("nan")))
    print(
        f"PRED {name} stop_t {stop_t:.3f} end {end:.3f} "
        f"x_move {stop_move:+.5f} "
        f"fault {held.get('fault') or 'none'} "
        f"t {float(held.get('fault_t', float('nan'))):.3f}"
    )
    air_zero = abs(stop_move) < 1e-4 and str(stop_rows[0].get("mode")) != "snap"
    if air_zero:
        print(f"PRED {name} x_move was zero on the stop tick before the snap")

    leg_suffix = (
        "hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll",
    )
    roll_suffix = ("hip_roll", "ank_roll")

    def _row_at(t: float) -> dict[str, float | str] | None:
        exact = by_t.get(round(t, 5))
        if exact is not None:
            return exact
        best: dict[str, float | str] | None = None
        best_dt = 1e9
        for row in lateral:
            if not isinstance(row, dict):
                continue
            dt = min(
                abs(float(row["t_ask"]) - t),
                abs(float(row["t_post"]) - t),
            )
            if dt < best_dt:
                best_dt = dt
                best = row
        if best is not None and best_dt <= 0.02 + 1e-6:
            return best
        return None

    def _immediate_pre(jn: str) -> tuple | None:
        """Last sample of this joint before the stop. Not the walk-wide peak."""
        best: tuple | None = None
        for item in asks:
            if str(item[1]) != jn:
                continue
            t_item = float(item[0])
            if t_item >= stop_t - 1e-9:
                continue
            if best is None or t_item > float(best[0]) - 1e-12:
                best = item
        return best

    def _print_one(label: str, item: tuple) -> None:
        t = float(item[0])
        q = _ask_q(item)
        ask = float(item[3])
        print(
            f"PRED {name} {label} {item[1]} {ask:+.4f} t {t:.3f} "
            f"q {q:+.5f} q_des {float(item[2]):+.5f} "
            f"kp_term {float(item[8]):+.4f} kv_term {float(item[9]):+.4f} "
            f"omega {float(item[10]):+.4f} kp {float(item[11]):.2f} kv {float(item[12]):.4f} "
            f"phase {item[5]} ge_2.33 {int(abs(ask) > KNEE_NM + 1e-9)}"
        )
        _print_box_pair(name, label, _row_at(t))

    def _worst(items: list[tuple], suffixes: tuple[str, ...]) -> dict[str, tuple]:
        out: dict[str, tuple] = {}
        for item in items:
            if not str(item[1]).endswith(suffixes):
                continue
            prev = out.get(str(item[1]))
            if prev is None or abs(float(item[3])) > abs(float(prev[3])):
                out[str(item[1])] = item
        return out

    pre_items = [
        item for item in asks
        if str(item[1]).endswith(leg_suffix) and _WALK_STEADY_S - 1e-9 <= float(item[0]) < stop_t - 1e-9
    ]
    fault_t = float(held.get("fault_t", float("nan")))
    tipped = bool(held.get("fault")) and math.isfinite(fault_t)

    def _before_tip(t_item: float) -> bool:
        if not tipped:
            return True
        return t_item < fault_t - 1e-9

    post_items = [
        item for item in asks
        if str(item[1]).endswith(leg_suffix)
        and float(item[0]) + 1e-9 >= stop_t
        and _before_tip(float(item[0]))
    ]
    after_tip_items = [
        item for item in asks
        if str(item[1]).endswith(leg_suffix)
        and float(item[0]) + 1e-9 >= stop_t
        and not _before_tip(float(item[0]))
    ]
    print(
        f"PRED {name} walk_mid_margin {float(summary.get('mid_margin', float('nan'))) * 1000.0:+.2f} mm "
        f"n_mid_out {summary.get('n_mid_out')}/{summary.get('n_mid')} "
        "CoM outside is expected at y_swap 0 and is not the torque bar."
    )
    for bucket, label in (
        ("dsp", "dsp"),
        ("mid_stance", "mid"),
    ):
        packed = summary.get(bucket)
        if not isinstance(packed, dict):
            continue
        for suf in roll_suffix:
            for side in ("l_", "r_"):
                item = packed.get(side + suf)
                if isinstance(item, tuple):
                    _print_one(label, item)
    pre_worst = _worst(pre_items, leg_suffix)
    post_worst = _worst(post_items, leg_suffix)
    peak = None
    for item in post_worst.values():
        if peak is None or abs(float(item[3])) > abs(float(peak[3])):
            peak = item
    fails: list[str] = []
    if peak is not None:
        _print_one("peak", peak)
        jn = str(peak[1])
        pre = _immediate_pre(jn)
        walk_max = pre_worst.get(jn)
        pre_ask = float(pre[3]) if pre is not None else float("nan")
        pre_over = pre is not None and abs(pre_ask) > KNEE_NM + 1e-9
        kind = "pre-stop lag" if pre_over else "stop transition"
        walk_ask = float(walk_max[3]) if walk_max is not None else float("nan")
        print(
            f"PRED {name} peak_kind {kind} {jn} "
            f"stop {float(peak[3]):+.4f} t {float(peak[0]):.3f} "
            f"pre {pre_ask:+.4f} "
            f"pre_t {float(pre[0]) if pre is not None else float('nan'):.3f} "
            f"walk_max {walk_ask:+.4f} "
            f"walk_max_t {float(walk_max[0]) if walk_max is not None else float('nan'):.3f} "
            "walk_max is the parked swing-z when that time is near 2 s. "
            "The kind uses the tick immediately before the stop."
        )
        cap_hit = None
        for row in cap_rows:
            if not isinstance(row, dict):
                continue
            if str(row.get("jn")) != jn:
                continue
            if abs(float(row["t"]) - float(peak[0])) > 1e-6:
                continue
            cap_hit = row
        if isinstance(cap_hit, dict):
            print(
                f"PRED {name} slew {jn} t {float(cap_hit['t']):.3f} "
                f"q {float(cap_hit['q']):+.5f} q_des {float(cap_hit['q_des']):+.5f} "
                f"q_stand {float(cap_hit['q_stand']):+.5f} "
                f"from {float(cap_hit['from']):+.5f} "
                f"stepped {float(cap_hit['stepped']):+.5f} "
                f"capped {int(cap_hit['capped'])} "
                f"omega {float(cap_hit['omega']):+.4f}"
            )
        else:
            print(
                f"PRED {name} slew {jn} t {float(peak[0]):.3f} "
                "not on the stance knee and ankle-pitch cap. "
                "Swing-z stays on the gait schedule."
            )
    else:
        fails.append("no stop ask")
        kind = "stop transition"
    for jn in sorted(post_worst):
        item = post_worst[jn]
        ask = float(item[3])
        over = abs(ask) > KNEE_NM + 1e-9
        pre = _immediate_pre(jn)
        pre_ask = float(pre[3]) if pre is not None else float("nan")
        pre_over = pre is not None and abs(pre_ask) > KNEE_NM + 1e-9
        print(
            f"PRED {name} leg_worst {jn} {ask:+.4f} t {float(item[0]):.3f} "
            f"q {_ask_q(item):+.5f} q_des {float(item[2]):+.5f} "
            f"pre {pre_ask:+.4f} "
            f"pre_t {float(pre[0]) if pre is not None else float('nan'):.3f} "
            f"pre_over {int(pre_over)} "
            f"ge_2.33 {int(over)}"
        )
        if over:
            why = "pre-stop lag" if pre_over else "stop transition"
            fails.append(f"unclamped stop {jn} {ask:+.4f} t {float(item[0]):.3f} {why}")
    after_worst = _worst(after_tip_items, leg_suffix)
    after_peak = None
    for item in after_worst.values():
        if after_peak is None or abs(float(item[3])) > abs(float(after_peak[3])):
            after_peak = item
    if after_peak is not None:
        _print_one("post_tip", after_peak)
        print(
            f"PRED {name} post_tip_note {after_peak[1]} {float(after_peak[3]):+.4f} "
            f"t {float(after_peak[0]):.3f} is at or after the tip. "
            "It is not the torque bar."
        )
    near = None
    if peak is not None:
        best_dt = 1e9
        for row in trace:
            if not isinstance(row, dict):
                continue
            dt = abs(float(row["t"]) - float(peak[0]))
            if dt < best_dt:
                best_dt = dt
                near = row
    if isinstance(near, dict):
        print(f"PRED {name} bracket peak")
        _print_soft_feet(name, near)
        print(
            f"PRED {name} freeze {near.get('freeze', '-')} "
            f"mode {near.get('mode', '-')}"
        )
    if edge:
        print(
            "PRED sag_stop edge "
            f"swing {edge.get('swing', '-')} "
            f"dx {float(edge.get('dx', float('nan'))):+.5f} "
            f"dknee_des {float(edge.get('ddes', float('nan'))):+.5f} "
            f"z_stretch {float(edge.get('z_stretch', float('nan'))):.3f} "
            f"soft {float(edge.get('z_soft', float('nan'))):.2f} "
            f"fit {int(edge.get('z_fit', 0))} "
            f"wall {float(edge.get('z_wall', float('nan'))):.3f} "
            f"end_wall {float(edge.get('z_end_wall', float('nan'))):.3f} "
            f"catch {float(edge.get('z_catch', float('nan'))):.3f} "
            f"gait_last {float(edge.get('z_gait_last', float('nan'))):.3f} "
            f"waits {int(edge.get('z_waits', 0))} "
            f"saw_air {int(edge.get('saw_air', 0))} "
            f"load_last L {float(edge.get('load_l_last', float('nan'))):.2f} "
            f"R {float(edge.get('load_r_last', float('nan'))):.2f} "
            f"swing_min {float(edge.get('swing_load_min', float('nan'))):.2f} "
            f"t {float(edge.get('swing_load_min_t', float('nan'))):.3f} "
            f"miss_cmd_ok {int(edge.get('z_miss_cmd_ok', 0))} "
            f"knee_cmd {float(edge.get('z_miss_knee_cmd', float('nan'))):+.5f} "
            f"knee_ik {float(edge.get('z_miss_knee_ik', float('nan'))):+.5f} "
            f"ank_cmd {float(edge.get('z_miss_ank_cmd', float('nan'))):+.5f} "
            f"ank_ik {float(edge.get('z_miss_ank_ik', float('nan'))):+.5f} "
            f"peak_rate {float(edge.get('z_peak_rate', float('nan'))):.3f} "
            f"knee_rate {float(edge.get('z_knee_rate', float('nan'))):.3f} "
            f"ank_rate {float(edge.get('z_ank_rate', float('nan'))):.3f} "
            f"x_move_note pin keeps the walking step until the snap"
        )
        touch = edge.get("touchdown")
        if isinstance(touch, dict):
            print(
                "PRED sag_stop touchdown "
                f"t {float(touch.get('t', float('nan'))):.3f} "
                f"swing {touch.get('swing', '-')} "
                f"load {float(touch.get('load', float('nan'))):.2f} N "
                f"gait {float(touch.get('gait', float('nan'))):.3f} "
                f"waits {int(touch.get('waits', 0))} "
                f"lowest {touch.get('lowest', '-')} "
                f"{float(touch.get('lowest_mm', float('nan'))):+.3f} mm"
            )
        else:
            print(
                "PRED sag_stop touchdown none. "
                "The swing foot did not reload over 5 N."
            )
    toe_mm, toe_t, toe_side, toe_frac = _quiet_flat_toe(held)
    print(
        f"PRED {name} flat_toe {toe_mm:+.3f} mm t {toe_t:.3f} "
        f"side {toe_side or '-'} frac {toe_frac:.3f} "
        "This toe is the y_swap 0 copy. "
        "It is not the +2.221 mm clear at y_swap 0.020. "
        "When lateral returns, the 20-80% flat toe and the parked SSP swing-z knee "
        "are re-scored. Track 1 is not done."
    )
    rail_best: tuple[str, float, float] | None = None
    rail_after: tuple[str, float, float] | None = None
    for surf in surface:
        if not isinstance(surf, dict):
            continue
        t = float(surf["t"])
        if t + 1e-9 < stop_t:
            continue
        before = _before_tip(t)
        for act in (
            "l_knee", "r_knee", "l_hip_roll", "r_hip_roll",
            "l_hip_pitch", "r_hip_pitch", "l_hip_yaw", "r_hip_yaw",
            "l_ank_pitch", "r_ank_pitch", "l_ank_roll", "r_ank_roll",
        ):
            if act not in surf:
                continue
            tau = float(surf[act])
            if before:
                if rail_best is None or abs(tau) > abs(rail_best[1]):
                    rail_best = (act, tau, t)
            elif rail_after is None or abs(tau) > abs(rail_after[1]):
                rail_after = (act, tau, t)
    if rail_best is not None:
        act, tau, t = rail_best
        print(
            f"PRED {name} actuator_peak {act} {tau:+.4f} t {t:.3f} "
            f"plant_rail {PLANT_NM:.2f} "
            f"ge_2.45 {int(abs(tau) >= PLANT_NM - 1e-3)} "
            f"ge_2.33 {int(abs(tau) >= KNEE_NM)} "
            "before any tip"
        )
        if abs(tau) >= PLANT_NM - 1e-3:
            fails.append(f"actuator {act} {tau:+.4f} t {t:.3f}")
    if rail_after is not None:
        act, tau, t = rail_after
        print(
            f"PRED {name} actuator_post_tip {act} {tau:+.4f} t {t:.3f} "
            "is at or after the tip and is not the torque bar"
        )
    if held.get("fault"):
        fails.append(f"fault {held.get('fault')} t {float(held.get('fault_t', float('nan'))):.3f}")
    if air_zero:
        fails.append("x_move zeroed on the stop tick")
    t_stop = _soft_t_stop(
        [row for row in trace if isinstance(row, dict)],
        stop_t,
    )
    t_txt = "none" if t_stop is None else f"{t_stop:.3f}"
    torque_fails = [
        item for item in fails
        if item.startswith("unclamped") or item.startswith("actuator") or item.startswith("no stop")
    ]
    if not torque_fails:
        print(
            f"PRED {name} pre_tip_torque every leg ask from the stop command "
            "until the tip stays at or under 2.33 Nm."
        )
    if fails:
        print(
            f"PRED {name} Prefer FAIL. "
            + " | ".join(fails)
            + f". T_stop {t_txt} s is the body-speed settle and is not a pass. "
            "d_min is not rebuilt. The old d_min 0.1263 m is not claimed. "
            "CoM outside the 38 mm box is not this bar and is not kit-safe. "
            "Yaw stayed 0. Foot-z stayed 1.170 mm. Less-crouch stayed closed. "
            "Trim lead stayed off. y_swap stayed 0. "
            "Plant md5 stays 207f3d5e9c6a72e16f7aa0c8d224f75e. "
            "Not kit-safe. Not go-anywhere."
        )
        return
    d_note = "none" if t_stop is None else f"{0.150 * t_stop:.4f}"
    print(
        f"PRED {name} CLEAR. Unclamped stop asks from the stop command "
        "until any tip stay at or under 2.33 Nm on every leg joint. "
        "Leg actuators stay under the +/-2.45 Nm plant rail. "
        f"T_stop {t_txt} s from the stop command to body speed under "
        f"{SOFT_STOP_SPEED_EPS:.2f} m/s for {SOFT_STOP_HOLD_S:.2f} s. "
        f"At T_detect 0 the rebuilt d_min is 0.150 * T_stop = {d_note} m. "
        "The old d_min 0.1263 m is not this stop. No wall-stop pass is claimed. "
        "CoM outside the 38 mm box is not this bar and is not kit-safe. "
        "The y_swap 0 toe is not the y_swap 0.020 clear. Track 1 is not done. "
        "x_move was not zeroed on the stop tick. "
        "Yaw stayed 0. Foot-z stayed 1.170 mm. Less-crouch stayed closed. "
        "Trim lead stayed off. "
        "Plant md5 stays 207f3d5e9c6a72e16f7aa0c8d224f75e. "
        "Not kit-safe. Not go-anywhere."
    )


def _swing_frac(phase: str, pose: float) -> float:
    """Single-support fraction. Left SSP is 0.025–0.225. Right is 0.275–0.475."""
    if phase == "L":
        start = 0.025
    elif phase == "R":
        start = 0.275
    else:
        return float("nan")
    return (pose - start) / 0.200


def _keep_ask(best: tuple | None, item: tuple) -> tuple:
    if best is None or abs(float(item[3])) > abs(float(best[3])):
        return item
    return best


def score_mid_swing() -> None:
    """Continuous straight walk on the locked sagittal copy. No stop.

    20–80% toe on both feet, the parked SSP swing-z knee, and the
    entrance rug when a swing is tagged. Mid-SS and DSP asks are the
    stance chain. The stop stretch does not arm on this bout.
    """
    name = "mid_swing"
    print(
        f"PRED {name} plan continuous straight walk. No stop. "
        f"Day-1 vx {sw.VX_FWD_CAP:.3f} m/s. Yaw 0. Period 0.500 s. "
        "y_swap 0. init_y 0.005 m. Foot-z 1.170 mm. Less-crouch closed. "
        "Trim-lead off. Sole-flat stance ankle stays on. "
        f"Loaded knee stays on the {_WALK_KNEE_RATE:.2f} rad/s slew. "
        f"Loaded ankle pitch stays on the {_WALK_STANCE_RATE:.2f} rad/s slew. "
        f"Loaded hip pitch is on its own {_WALK_HIP_PITCH_RATE:.2f} rad/s slew. "
        "Walk swing-z stays at "
        f"{_WALK_Z_RATE:.2f} rad/s. The stop stretch stays off at "
        "0.25 / 1.060×. "
        "Pass is mid-stance peaks at or under 2.33 Nm, and the flat toe "
        "still at or above +2 mm, with four contact-box corners at or "
        "above 0. CoM slack and the four corners are logged at the "
        "mid-stance knee and hip-pitch peaks. "
        "The parked SSP swing knee is logged and is not the bar unless "
        "that tick moves into 20-80%. "
        "CoM outside the 38 mm box is logged and is not a torque pass. "
        "Plant, kp, and forcerange stay put."
    )
    _require_plant("before", name)
    held: dict[str, object] = {}
    t_end = 6.5
    measure_pred_clip(
        clip=True,
        move_s=None,
        pitch_move_off=True,
        toe_up_rad=0.025,
        toe_up_shape="front",
        toe_full_frac=0.10,
        hip_lead=True,
        lead_roll_scale=0.0,
        lead_pitch_scale=_pitch_lead_scale(20.0),
        sag_cancel=False,
        z_profile="phase",
        ank_trim_l=_RESTORED_ROLL_L,
        ank_trim_r=_RESTORED_ROLL_R,
        ank_pitch_trim_l=_RESTORED_PITCH_L,
        ank_pitch_trim_r=_RESTORED_PITCH_R,
        z_extra_m=_LEVEL_FOOTZ_UM / 1_000_000.0,
        t_end=t_end,
        segments=(
            sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
            sw.DemoSegment(t_end, "vel", sw.VX_FWD_CAP, 0.0, "forward"),
        ),
        steer_out=held,
        y_swap_m=0.0,
        lateral_log=True,
        sole_flat=True,
        sole_flat_mode="q",
        y_out_m=0.005,
        x_scale=1.0,
        sag_slew_rad_s=1.90,
        sag_slew_joints=("knee", "ank_pitch", "hip_pitch"),
        sag_split=True,
        surface_tag=True,
        walk_z_stretch=True,
    )
    summary = _walk_lateral_summary(held, 0.0)
    summary["x_scale"] = 1.0
    summary["sag_slew"] = _WALK_KNEE_RATE
    summary["hip_slew"] = _WALK_HIP_PITCH_RATE
    summary["slew_joints"] = (
        f"knee@{_WALK_KNEE_RATE:.2f}+ank@{_WALK_STANCE_RATE:.2f}"
        f"+hip@{_WALK_HIP_PITCH_RATE:.2f}"
    )
    toe_mm, toe_t, toe_side, toe_frac = _quiet_flat_toe(held)
    summary["flat_toe_mm"] = toe_mm
    summary["flat_toe_t"] = toe_t
    summary["flat_toe_side"] = toe_side
    summary["flat_toe_frac"] = toe_frac
    _print_sag_row(summary)

    rows = held.get("rows")
    surface = held.get("surface")
    by_surf: dict[float, dict[str, object]] = {}
    if isinstance(surface, list):
        for row in surface:
            if isinstance(row, dict):
                by_surf[round(float(row["t"]), 5)] = row
    flat_best: dict[str, tuple[float, float, float, dict[str, object] | None]] = {}
    corner_low: dict[str, tuple[float, str, float, float, dict[str, object]]] = {}
    rug_best: dict[str, tuple[float, float, float, float]] = {}
    n_flat = {"L": 0, "R": 0}
    n_rug = {"L": 0, "R": 0}
    if isinstance(rows, list):
        cycles = _swing_cycles(rows, 1.0e9)
        med_n, _last_n = _cycle_index(cycles)
        for step, cyc in enumerate(cycles):
            n = len(cyc)
            if n < 2 or med_n < 2:
                continue
            n_full = med_n if step == len(cycles) - 1 and n < med_n - 1 else n
            mids: list[tuple[float, Tick, dict[str, object] | None]] = []
            for j, row in enumerate(cyc):
                if row.toe_z is None or row.swing not in ("L", "R") or row.t < _WALK_STEADY_S:
                    continue
                frac = j / (n_full - 1)
                if not (0.20 - 1e-12 <= frac <= 0.80 + 1e-12):
                    continue
                plane = None
                surf = by_surf.get(round(row.t, 5))
                if isinstance(surf, dict):
                    packed = surf.get(row.swing or "")
                    if isinstance(packed, dict):
                        plane = packed
                mids.append((frac, row, plane))
            if not mids:
                continue
            side = str(mids[0][1].swing)
            on_rug = any(
                plane is not None and int(plane.get("on_rug") or 0)
                for _frac, _row, plane in mids
            )
            if on_rug:
                n_rug[side] = n_rug.get(side, 0) + 1
                frac, row, plane = min(
                    mids,
                    key=lambda item: (
                        float(item[2]["clear"]) if isinstance(item[2], dict) else float("inf")
                    ),
                )
                clear = float(plane["clear"]) if isinstance(plane, dict) else float("nan")
                toe = float(row.toe_z or 0.0)
                prev = rug_best.get(side)
                if prev is None or clear < prev[0]:
                    rug_best[side] = (clear, toe, row.t, frac)
            else:
                n_flat[side] = n_flat.get(side, 0) + 1
                for frac_i, row_i, plane_i in mids:
                    if not isinstance(plane_i, dict):
                        continue
                    corners_i = plane_i.get("corners")
                    if not isinstance(corners_i, tuple):
                        continue
                    for label_i, z_i in corners_i:
                        prev_c = corner_low.get(side)
                        if prev_c is None or float(z_i) < prev_c[0]:
                            corner_low[side] = (
                                float(z_i), str(label_i), row_i.t, frac_i, plane_i,
                            )
                frac, row, plane = min(mids, key=lambda item: float(item[1].toe_z or 0.0))
                toe = float(row.toe_z or 0.0)
                prev_f = flat_best.get(side)
                if prev_f is None or toe < prev_f[0]:
                    flat_best[side] = (toe, row.t, frac, plane)

    def _print_corners(
        tag: str, side: str, t_c: float, frac_c: float, plane_c: dict[str, object] | None,
    ) -> None:
        corners = plane_c.get("corners") if isinstance(plane_c, dict) else None
        if not isinstance(corners, tuple) or not corners:
            print(f"PRED {name} corners side {side} {tag} missing")
            return
        parts = " ".join(
            f"{label} {float(z) * 1000.0:+.3f}" for label, z in corners
        )
        low = min(corners, key=lambda item: float(item[1]))
        print(
            f"PRED {name} corners side {side} {tag} "
            f"t {t_c:.3f} frac {frac_c:.3f} {parts} "
            f"low {low[0]} {float(low[1]) * 1000.0:+.3f} mm "
            f"toe_down {int(float(low[1]) < 0.0)}"
        )

    for side in ("L", "R"):
        packed_f = flat_best.get(side)
        if packed_f is None:
            print(f"PRED {name} flat_toe side {side} missing n_swings {n_flat.get(side, 0)}")
        else:
            toe_s, t_s, frac_s, plane_s = packed_f
            print(
                f"PRED {name} flat_toe side {side} {toe_s * 1000.0:+.3f} mm "
                f"t {t_s:.3f} frac {frac_s:.3f} "
                f"n_swings {n_flat.get(side, 0)} "
                f"scuff {int(toe_s < 0.0)} "
                f"above_2mm {int(toe_s >= TOE_BAR_M)}"
            )
            _print_corners("at_toe", side, t_s, frac_s, plane_s)
        packed_c = corner_low.get(side)
        if packed_c is None:
            print(f"PRED {name} corners side {side} window missing")
        else:
            _z_c, _label_c, t_c, frac_c, plane_c = packed_c
            _print_corners("window_low", side, t_c, frac_c, plane_c)
        packed_r = rug_best.get(side)
        if packed_r is None:
            print(f"PRED {name} rug side {side} none n_swings {n_rug.get(side, 0)}")
        else:
            clear_s, toe_s, t_s, frac_s = packed_r
            print(
                f"PRED {name} rug side {side} clear {clear_s * 1000.0:+.3f} mm "
                f"toe {toe_s * 1000.0:+.3f} mm t {t_s:.3f} frac {frac_s:.3f} "
                f"n_swings {n_rug.get(side, 0)} "
                f"scuff {int(clear_s < 0.0)}"
            )

    asks = held.get("asks")
    lateral = held.get("lateral")
    by_lat: dict[float, dict[str, float | str]] = {}
    if isinstance(lateral, list):
        for row in lateral:
            if isinstance(row, dict):
                by_lat[round(float(row["t_ask"]), 5)] = row
    swing_ssp: dict[str, tuple] = {}
    swing_mid: dict[str, tuple] = {}
    stance_mid: dict[str, tuple] = {}
    stance_dsp: dict[str, tuple] = {}
    if isinstance(asks, list):
        for item in asks:
            jn = str(item[1])
            if jn not in _WALK_LEG:
                continue
            t = float(item[0])
            if t < _WALK_STEADY_S - 1e-9:
                continue
            row = by_lat.get(round(t, 5))
            if row is None:
                continue
            phase = str(row["phase"])
            if phase == "D":
                stance_dsp[jn] = _keep_ask(stance_dsp.get(jn), item)
                continue
            if phase not in ("L", "R"):
                continue
            frac = _swing_frac(phase, float(row["pose"]))
            swing_pref = "l_" if phase == "L" else "r_"
            stance_pref = "r_" if phase == "L" else "l_"
            if jn.startswith(swing_pref):
                if jn.endswith("knee"):
                    swing_ssp[jn] = _keep_ask(swing_ssp.get(jn), item)
                if 0.20 - 1e-12 <= frac <= 0.80 + 1e-12:
                    swing_mid[jn] = _keep_ask(swing_mid.get(jn), item)
            elif (
                jn.startswith(stance_pref)
                and 0.25 - 1e-9 <= frac <= 0.75 + 1e-9
            ):
                fn_key = "fn_r" if phase == "L" else "fn_l"
                sw_key = "fn_l" if phase == "L" else "fn_r"
                if (
                    float(row[fn_key]) > _SOLE_FLAT_LOAD_N
                    and float(row[sw_key]) <= _SOLE_FLAT_LOAD_N
                ):
                    stance_mid[jn] = _keep_ask(stance_mid.get(jn), item)

    def _dump(tag: str, bucket: dict[str, tuple]) -> tuple | None:
        worst = None
        for jn in _WALK_LEG:
            item = bucket.get(jn)
            if item is None:
                continue
            row = by_lat.get(round(float(item[0]), 5))
            frac = float("nan")
            phase = "-"
            if row is not None:
                phase = str(row["phase"])
                frac = _swing_frac(phase, float(row["pose"]))
            over = abs(float(item[3])) > KNEE_NM + 1e-9
            print(
                f"PRED {name} {tag} {jn} {float(item[3]):+.4f} "
                f"t {float(item[0]):.3f} phase {phase} frac {frac:.3f} "
                f"q {_ask_q(item):+.5f} q_des {float(item[2]):+.5f} "
                f"kp_term {float(item[8]):+.4f} kv_term {float(item[9]):+.4f} "
                f"omega {float(item[10]):+.4f} "
                f"ge_2.33 {int(over)}"
            )
            if worst is None or abs(float(item[3])) > abs(float(worst[3])):
                worst = item
        if worst is None:
            print(f"PRED {name} {tag} missing")
        return worst

    knee_worst = _dump("ssp_swing_knee", swing_ssp)
    mid_j = _dump("swing_20_80", swing_mid)
    mid_ss = _dump("mid_ss", stance_mid)
    dsp_w = _dump("dsp", stance_dsp)
    _print_sag_split("mid_knee", _sag_worst(summary.get("mid_stance"), "knee"), summary)
    _print_sag_split("mid_ap", _sag_worst(summary.get("mid_stance"), "ank_pitch"), summary)
    _print_sag_split("dsp_knee", _sag_worst(summary.get("dsp"), "knee"), summary)
    _print_sag_split("ssp_knee", _sag_worst(summary.get("ssp"), "knee"), summary)

    worst_mid = summary.get("worst_mid")
    if isinstance(worst_mid, dict):
        com_row = by_lat.get(round(float(worst_mid["t"]), 5))
        _print_box_pair(name, "mid", com_row)
    print(
        f"PRED {name} com mid_margin {float(summary['mid_margin']) * 1000.0:+.2f} mm "
        f"n_mid_out {int(summary['n_mid_out'])}/{int(summary['n_mid'])} "
        f"inside {int(summary['inside'])} "
        f"half_y {float(summary['half_y']) * 1000.0:.2f} mm "
        f"fault {summary.get('fault') or 'none'}"
    )

    scored = ("l_knee", "r_knee", "l_ank_pitch", "r_ank_pitch")
    swing_ok = all(
        jn in swing_mid and abs(float(swing_mid[jn][3])) <= KNEE_NM + 1e-9
        for jn in scored
    )
    toe_bar = all(side in flat_best for side in ("L", "R")) and all(
        flat_best[side][0] >= TOE_BAR_M for side in ("L", "R")
    )
    rug_ok = all(packed[0] >= 0.0 for packed in rug_best.values())
    corners_ok = all(side in corner_low for side in ("L", "R")) and all(
        corner_low[side][0] >= 0.0 for side in ("L", "R")
    )
    support_ok = (
        mid_ss is not None
        and dsp_w is not None
        and abs(float(mid_ss[3])) <= KNEE_NM + 1e-9
        and abs(float(dsp_w[3])) <= KNEE_NM + 1e-9
        and not _body_bad(summary)
    )
    slew_stats = held.get("walk_slew")
    if isinstance(slew_stats, dict):
        dt_s = float(ow.OP3_CTRL_S)
        hip_dq = float(slew_stats.get("hip_max_abs_dq", 0.0))
        knee_dq = float(slew_stats.get("knee_max_abs_dq", 0.0))
        hip_cmd = hip_dq / dt_s if dt_s > 0.0 else float("nan")
        knee_cmd = knee_dq / dt_s if dt_s > 0.0 else float("nan")
        print(
            f"PRED {name} stance_slew hip {_WALK_HIP_PITCH_RATE:.2f} rad/s "
            f"clip {int(float(slew_stats.get('hip_clip', 0.0)))} "
            f"max_cmd {hip_cmd:.3f} rad/s "
            f"knee {_WALK_KNEE_RATE:.2f} rad/s "
            f"clip {int(float(slew_stats.get('knee_clip', 0.0)))} "
            f"max_cmd {knee_cmd:.3f} rad/s "
            f"ank {_WALK_STANCE_RATE:.2f} rad/s"
        )
    mid_plant_ok = True
    peak_times: dict[float, list[str]] = {}
    for jn, item in stance_mid.items():
        if not (str(jn).endswith("knee") or str(jn).endswith("hip_pitch")):
            continue
        peak_times.setdefault(round(float(item[0]), 5), []).append(str(jn))
    for t_key, joints_at in sorted(peak_times.items()):
        row = by_lat.get(t_key)
        tag = "mid_ss_" + "+".join(joints_at)
        _print_box_pair(name, tag, row)
        surf = by_surf.get(t_key)
        frac_m = float("nan")
        if row is not None:
            frac_m = _swing_frac(str(row["phase"]), float(row["pose"]))
        stance_side = str(row["stance"]) if row is not None else ""
        for side in ("L", "R"):
            plane = surf.get(side) if isinstance(surf, dict) else None
            plane_d = plane if isinstance(plane, dict) else None
            _print_corners(tag, side, t_key, frac_m, plane_d)
            corners = plane_d.get("corners") if isinstance(plane_d, dict) else None
            if not isinstance(corners, tuple) or not corners:
                mid_plant_ok = False
                continue
            low_z = min(float(z) for _label, z in corners)
            if side == stance_side:
                if low_z < _STANCE_DIG_M:
                    mid_plant_ok = False
            elif low_z < 0.0:
                mid_plant_ok = False
    ssp_frac = float("nan")
    if knee_worst is not None:
        ssp_row = by_lat.get(round(float(knee_worst[0]), 5))
        if ssp_row is not None:
            ssp_frac = _swing_frac(str(ssp_row["phase"]), float(ssp_row["pose"]))
    ssp_in_window = ssp_frac == ssp_frac and 0.20 - 1e-12 <= ssp_frac <= 0.80 + 1e-12
    if knee_worst is None:
        print(f"PRED {name} ssp_knee_peak missing")
    else:
        print(
            f"PRED {name} ssp_knee_peak {knee_worst[1]} {float(knee_worst[3]):+.4f} "
            f"t {float(knee_worst[0]):.3f} frac {ssp_frac:.3f} "
            f"in_20_80 {int(ssp_in_window)} "
            f"ge_2.33 {int(abs(float(knee_worst[3])) > KNEE_NM + 1e-9)}"
        )
    if swing_ok and toe_bar and rug_ok and corners_ok and support_ok and mid_plant_ok:
        print(
            f"PRED {name} CLEAR. 20-80% knee and ankle pitch stay at or under "
            "2.33 Nm, and both flat toes stay at or above +2 mm near frac 0.208. "
            "Four corners stay at or above 0 through that window and at the "
            "mid-stance knee and hip-pitch peaks. "
            "Mid-SS and DSP stay at or under 2.33 Nm. "
            "The parked SSP swing knee stays outside 20-80%. "
            f"CoM mid_margin {float(summary['mid_margin']) * 1000.0:+.2f} mm "
            "is outside the 38 mm box and is not a torque pass. "
            "y_swap stayed 0. Foot-z stayed 1.170 mm. Less-crouch stayed closed. "
            "The stop stretch stayed 0.25 / 1.060×. d_min is not rebuilt. "
            "Not kit-safe. "
            "Plant md5 stays 207f3d5e9c6a72e16f7aa0c8d224f75e."
        )
    else:
        why = []
        if not swing_ok:
            if mid_j is None:
                why.append("20-80% knee and ankle pitch missing")
            else:
                why.append(
                    f"20-80% {mid_j[1]} {float(mid_j[3]):+.4f} Nm "
                    f"t {float(mid_j[0]):.3f}"
                )
        if not toe_bar:
            why.append("flat toe under +2 mm")
        if not corners_ok:
            why.append("a contact-box corner went under 0")
        if not rug_ok:
            why.append("rug clearance under 0")
        if not mid_plant_ok:
            why.append(
                "a mid-stance tick dug the stance sole past the locked "
                "−2.564 mm class, or put the swing foot under 0"
            )
        if not support_ok:
            overs = []
            for jn, item in stance_mid.items():
                if abs(float(item[3])) > KNEE_NM + 1e-9:
                    overs.append(
                        f"mid-SS {jn} {float(item[3]):+.4f} t {float(item[0]):.3f}"
                    )
            for jn, item in stance_dsp.items():
                if abs(float(item[3])) > KNEE_NM + 1e-9:
                    overs.append(
                        f"DSP {jn} {float(item[3]):+.4f} t {float(item[0]):.3f}"
                    )
            why.append("; ".join(overs) if overs else "mid-SS or DSP body fault")
        next_lever = (
            "Foot-z, crouch, y_swap, the rail, and the plant stay closed. "
            "The stop stretch stays 0.25 / 1.060×."
        )
        if swing_ok and toe_bar and not corners_ok:
            next_lever = (
                "20-80% knee and ankle pitch are inside 2.33 Nm and the flat "
                "toe stays at or above +2 mm. A contact-box corner is under 0. "
                "Foot-z, crouch, y_swap, the rail, and the plant stay closed."
            )
        elif swing_ok and toe_bar and corners_ok and mid_plant_ok and not support_ok:
            next_lever = (
                "20-80% knee and ankle pitch are inside 2.33 Nm, both flat "
                "toes stay at or above +2 mm, and every contact-box corner "
                "stays at or above 0. The miss is the support chain named above. "
                f"Loaded hip pitch is at {_WALK_HIP_PITCH_RATE:.2f} rad/s, "
                f"the stance knee is at {_WALK_KNEE_RATE:.2f} rad/s, and "
                f"ankle pitch stays at {_WALK_STANCE_RATE:.2f} rad/s. "
                "The walk swing stays at 1.55 rad/s. "
                "Foot-z, crouch, y_swap, the rail, and the plant stay closed."
            )
        elif swing_ok and toe_bar and not mid_plant_ok:
            next_lever = (
                "The stance slew put a contact-box corner under 0 at a "
                "mid-stance knee or hip-pitch peak. That is a dig, not a "
                "torque pass. The walk swing stays at 1.55 rad/s. "
                "Foot-z, crouch, y_swap, the rail, and the plant stay closed."
            )
        elif swing_ok and not toe_bar:
            next_lever = (
                "Knee and ankle pitch in 20-80% are inside 2.33 Nm, and the "
                "landing schedule spent the +12.9 mm toe under +2 mm. "
                "A faster early rise puts that ask back over 2.33. "
                "Foot-z, crouch, y_swap, the rail, and the plant stay closed."
            )
        elif toe_bar and not swing_ok:
            next_lever = (
                "The toe is still at or above +2 mm, and the 20-80% ask is "
                "still over 2.33 Nm. The next stretch has to slow that tick "
                "without dropping the 0.208 toe under +2 mm."
            )
        print(
            f"PRED {name} Prefer FAIL. {'; '.join(why)}. {next_lever} "
            "y_swap stayed 0. Foot-z stayed 1.170 mm. Less-crouch stayed closed. "
            "The stop stretch stayed 0.25 / 1.060×. d_min is not rebuilt. "
            "CoM outside is not a torque pass. Not kit-safe. "
            "Plant md5 stays 207f3d5e9c6a72e16f7aa0c8d224f75e."
        )
    _require_plant("after", name)


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
