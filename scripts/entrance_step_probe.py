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


def _mid_swing(rows: list[Tick]) -> tuple[float, float, float, int, int]:
    """Min, time of min, p90, mid count, cycle count. Window is t < 7.560 s."""
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
    mids: list[tuple[float, float]] = []
    for cyc in cycles:
        n = len(cyc)
        if n < 2:
            continue
        for i, row in enumerate(cyc):
            frac = i / (n - 1)
            if row.toe_z is None:
                continue
            if 0.20 - 1e-12 <= frac <= 0.80 + 1e-12:
                mids.append((row.t, row.toe_z))
    if not mids:
        return (float("nan"), float("nan"), float("nan"), 0, len(cycles))
    zs = np.asarray([z for _t, z in mids], dtype=np.float64)
    k = int(np.argmin(zs))
    return (float(zs[k]), float(mids[k][0]), float(np.percentile(zs, 90)), len(mids), len(cycles))


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


def run_one(name: str, *, z_m: float | None, edge: bool, mode: str = "") -> dict[str, object]:
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
    elif edge:
        _hook_edge(session, rug)
    driver = sw.ScriptedDriver((
        sw.DemoSegment(1.0, "stand", 0.0, 0.0, "stand"),
        sw.DemoSegment(T_END, "vel", sw.VX_FWD_CAP, 0.0, "forward"),
    ))
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

    def _hook(model: mj.MjModel, data: mj.MjData) -> None:
        nonlocal ank_peak, ank_name, ank_t, walk_peak, walk_name, walk_t
        nonlocal knee_peak, knee_name, knee_t, walk_knee, walk_knee_name, walk_knee_t
        real_step(model, data)
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

    mj.mj_step = _hook
    try:
        while float(session.data.time) < T_END - 1e-9:
            driver.publish(session.bus, float(session.data.time))
            session.step()
            t = float(session.data.time)
            swing = session.lipm._gm_swing if session.lipm is not None else None
            toe_z = _leading_toe_z(session, swing) if swing in ("L", "R") else None
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
            rows.append(Tick(t, swing if swing in ("L", "R") else None, toe_z, swing_fn, reason))
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
    toe_min, toe_t, toe_p90, n_mid, n_cyc = _mid_swing(rows)
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
        "x": float(session.data.qpos[0]),
    }


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
