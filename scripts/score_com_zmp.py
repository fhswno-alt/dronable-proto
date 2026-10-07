#!/usr/bin/env python3
"""Score CoM and ZMP inside the stance sole, and the unclamped leg ask.

Stand, shift onto the first stance foot, walk, soft-stop. y_swap stays 0.
The lateral channel is the cart-table preview. Nothing here solves q_des
onto ±2.33 Nm, edits the plant, or raises a torque rail.

The ask is |kp*(q_des−q)| + |kv*ω| on the gait target passed to the
position writer. A later slew of ctrl is reported beside it and is not
the bar.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco as mj
import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import lipm_gait
import steer_walk as sw

G = 9.81
LOAD_N = 5.0
ASK_NM = 2.33
# Tighter voice-speed target. CLEAR stays at ASK_NM. This is the headroom bar.
HEADROOM_NM = 2.20
LEG_JOINTS: tuple[str, ...] = (
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
)


@dataclass
class AskRow:
    t: float
    joint: str
    q_des: float
    q: float
    omega: float
    kp: float
    kv: float
    signed_nm: float
    sum_nm: float
    stage: str


@dataclass
class MarginRow:
    t: float
    stage: str
    support: str
    com_mm: float
    cop_mm: float
    zmp_mm: float
    fn_l: float
    fn_r: float


def _plant_md5() -> str:
    digest = hashlib.md5()
    digest.update(sw.PLANT_XML.read_bytes())
    return digest.hexdigest()


def _install_ask_log(lipm: lipm_gait.LipmWalker, bucket: list[AskRow]) -> None:
    orig = lipm.write_clipped

    def wrapped(jn: str, q_des: float) -> None:
        if jn in LEG_JOINTS:
            act = jn + "_pos"
            idx = lipm.act_idx.get(act)
            if idx is not None:
                q = float(lipm.q(jn))
                jid = mj.mj_name2id(lipm.model, mj.mjtObj.mjOBJ_JOINT, jn)
                omega = float(lipm.data.qvel[int(lipm.model.jnt_dofadr[jid])])
                kp = float(lipm.model.actuator_gainprm[idx, 0])
                kv = -float(lipm.model.actuator_biasprm[idx, 2])
                kp_term = kp * (float(q_des) - q)
                signed = kp_term - kv * omega
                total = abs(kp_term) + abs(kv * omega)
                bucket.append(AskRow(
                    t=float(lipm.data.time),
                    joint=jn,
                    q_des=float(q_des),
                    q=q,
                    omega=omega,
                    kp=kp,
                    kv=kv,
                    signed_nm=float(signed),
                    sum_nm=float(total),
                    stage=str(lipm.preview_stage),
                ))
        orig(jn, q_des)

    lipm.write_clipped = wrapped  # type: ignore[method-assign]


def _support(
    session: sw.SteerSession,
) -> tuple[np.ndarray, str, float, float]:
    lipm = session.lipm
    if lipm is None:
        raise RuntimeError("preview walker missing")
    fn_l = float(lipm.foot_normal("L"))
    fn_r = float(lipm.foot_normal("R"))
    chunks: list[np.ndarray] = []
    if fn_l > LOAD_N:
        chunks.append(session._foot_corners(session.bid_lf, session.gid_lfoot))
    if fn_r > LOAD_N:
        chunks.append(session._foot_corners(session.bid_rf, session.gid_rfoot))
    if not chunks:
        return np.zeros((0, 2), dtype=np.float64), "air", fn_l, fn_r
    hull = sw.convex_hull_xy(np.concatenate(chunks, axis=0))
    if len(chunks) == 2:
        kind = "double"
    elif fn_l > LOAD_N:
        kind = "L"
    else:
        kind = "R"
    return hull, kind, fn_l, fn_r


def _cop_xy(session: sw.SteerSession, kind: str) -> np.ndarray | None:
    sides: tuple[str, ...]
    if kind == "double":
        sides = ("L", "R")
    elif kind in ("L", "R"):
        sides = (kind,)
    else:
        return None
    num = np.zeros(2, dtype=np.float64)
    den = 0.0
    floor = int(session.gid_floor)
    for side in sides:
        gid = int(session.lipm.gid[side]) if session.lipm is not None else -1
        for i in range(session.data.ncon):
            con = session.data.contact[i]
            g1 = int(con.geom1)
            g2 = int(con.geom2)
            if not ((g1 == gid and g2 == floor) or (g2 == gid and g1 == floor)):
                continue
            force = np.zeros(6, dtype=np.float64)
            mj.mj_contactForce(session.model, session.data, i, force)
            fn = float(force[0])
            if fn <= 1e-6:
                continue
            num += fn * np.asarray(con.pos[:2], dtype=np.float64)
            den += fn
    if den <= 1e-6:
        return None
    return num / den


def _zmp_xy(
    com: np.ndarray,
    vel_prev: np.ndarray | None,
    vel_now: np.ndarray,
    dt: float,
) -> np.ndarray | None:
    """Cart-table ZMP from subtree CoM velocity. CoP is logged separately."""
    if vel_prev is None or dt <= 0.0:
        return None
    acc = (vel_now - vel_prev) / dt
    denom = G + float(acc[2])
    if denom < 1.0:
        return None
    return com[:2] - (float(com[2]) * acc[:2] / denom)


def _worst(rows: list[MarginRow], stage: str, field: str) -> MarginRow | None:
    picked = [row for row in rows if row.stage == stage and math_finite(getattr(row, field))]
    if not picked:
        return None
    return min(picked, key=lambda row: float(getattr(row, field)))


def math_finite(value: float) -> bool:
    return bool(np.isfinite(value))


def _ask_peak(rows: list[AskRow], stage: str | None) -> AskRow | None:
    picked = rows if stage is None else [row for row in rows if row.stage == stage]
    if not picked:
        return None
    return max(picked, key=lambda row: row.sum_nm)


def _fmt_ask(row: AskRow | None) -> str:
    if row is None:
        return "none"
    return (
        f"{row.joint} sum {row.sum_nm:.4f} Nm signed {row.signed_nm:+.4f} Nm "
        f"t {row.t:.3f} s stage {row.stage} q {row.q:+.5f} q_des {row.q_des:+.5f} "
        f"kp {row.kp:.1f} kv {row.kv:.4f} omega {row.omega:+.4f}"
    )


def _fmt_margin(row: MarginRow | None, field: str) -> str:
    if row is None:
        return "none"
    value = float(getattr(row, field))
    return (
        f"{value:+.2f} mm t {row.t:.3f} s stage {row.stage} "
        f"support {row.support} fn {row.fn_l:.2f}/{row.fn_r:.2f}"
    )


# Day-1 kit bout the #102 tip holds this scorer to. Soft-pass stays off.
BASE_ZMP_MIN_M = -0.095164
BASE_ZMP_OUT = 0.213
BASE_COM_MIN_M = -0.028823
BASE_COM_OUT = 0.335
BASE_COM_JERK_PEAK = 1048.991
BASE_COM_JERK_RMS = 132.473
BASE_JOINT_JERK_PEAK = 39843.0
BASE_JOINT_JERK_RMS = 4111.0
BASE_UP_Z = 0.934
BASE_TAU_NM = 2.280


@dataclass
class SideFrac:
    n: int = 0
    n_out: int = 0
    min_mm: float = float("inf")


def _bucket(kind: str) -> str:
    if kind == "L":
        return "ss_L"
    if kind == "R":
        return "ss_R"
    if kind == "double":
        return "ds"
    return "air"


def _note_margin(stats: dict[str, SideFrac], kind: str, mm: float) -> None:
    if not math_finite(mm):
        return
    for key in ("all", _bucket(kind)):
        row = stats[key]
        row.n += 1
        if mm < 0.0:
            row.n_out += 1
        if mm < row.min_mm:
            row.min_mm = mm


def _empty_fracs() -> dict[str, SideFrac]:
    return {key: SideFrac() for key in ("all", "ss_L", "ss_R", "ds", "air")}


def _frac_of(row: SideFrac) -> float:
    if row.n <= 0:
        return float("nan")
    return float(row.n_out) / float(row.n)


def _rms(sum_sq: float, n: int) -> float:
    if n <= 0:
        return float("nan")
    return float(math.sqrt(sum_sq / float(n)))


def _leg_actuator_nm(session: sw.SteerSession) -> tuple[str, float]:
    lipm = session.lipm
    if lipm is None:
        return "", 0.0
    name = ""
    peak = 0.0
    for jn in LEG_JOINTS:
        idx = lipm.act_idx.get(jn + "_pos")
        if idx is None:
            continue
        force = abs(float(session.data.actuator_force[int(idx)]))
        if force > peak:
            peak = force
            name = jn
    return name, peak


def _applied_peak(session: sw.SteerSession) -> float:
    if not session.samples:
        return 0.0
    return max(float(sample.applied_vx) for sample in session.samples)


def _applied_settle(session: sw.SteerSession, t_stop: float) -> float:
    """Last applied_vx in the second before the stop command."""
    window = [
        sample for sample in session.samples
        if (t_stop - 1.0) <= float(sample.t) < t_stop - 1e-9
    ]
    if not window:
        return 0.0
    return float(window[-1].applied_vx)


def run_attempt(
    *,
    period_s: float,
    dsp: float,
    amp_m: float,
    z_m: float,
    arm_s: float,
    stand_s: float,
    walk_s: float,
    stop_s: float,
    kit_baseline: bool = False,
    vx_m_s: float = 0.0,
    preview_r: float = 1.0e-4,
    preview_shape: float = 1.0,
    select_gait: bool = False,
) -> dict[str, object]:
    md5_before = _plant_md5()
    if md5_before != sw.PLANT_MD5:
        raise SystemExit(f"plant md5 {md5_before} != {sw.PLANT_MD5}")
    script = None
    if kit_baseline:
        cfg = sw.locked_kit_config()
        stand_s = sw.BUS_KIT_STAND_S
        walk_s = sw.BUS_KIT_VEL_S - sw.BUS_KIT_STAND_S
        stop_s = sw.BUS_KIT_STOP_S - sw.BUS_KIT_VEL_S
        period_s = cfg.gm_period_s
        dsp = cfg.gm_dsp
        amp_m = 0.0
        z_m = cfg.gm_z_m
        arm_s = 0.0
    elif select_gait:
        cfg = sw.gait_for_command(sw.VOICE_VX_M_S, 0.0)
        period_s = float(cfg.gm_period_s)
        dsp = float(cfg.gm_dsp)
        amp_m = float(cfg.preview_amp_m)
        z_m = float(cfg.gm_z_m)
        arm_s = float(cfg.preview_arm_s)
        preview_r = float(cfg.preview_r)
        preview_shape = float(cfg.preview_shape)
        script = sw.voice_bus_script(stand_s, walk_s, stop_s, 0.0)
    else:
        cfg = lipm_gait.LipmConfig(
            name="preview",
            clear_m=z_m,
            arms=True,
            schedule="gait_manager",
            gm_period_s=period_s,
            gm_dsp=dsp,
            gm_y_swap_m=0.0,
            gm_x_m=0.020,
            gm_z_m=z_m,
            gm_z_swap_m=0.0,
            gm_pelvis_deg=0.0,
            gm_hip_pitch_deg=15.0,
            gm_start_lead="L",
            gm_crouch_m=0.025,
            gm_move_s=0.020,
            preview_amp_m=amp_m,
            preview_arm_s=arm_s,
            preview_r=preview_r,
            preview_shape=preview_shape,
        )
    session = sw.SteerSession(video=False, lipm=cfg)
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        raise SystemExit("walker did not build")
    if not kit_baseline and lipm.op3.y_swap_cmd != 0.0:
        raise SystemExit("y_swap_cmd is not 0")
    if kit_baseline:
        driver = sw.ScriptedDriver(sw.BUS_KIT_SCRIPT)
    elif script is not None:
        driver = sw.ScriptedDriver(script)
    else:
        driver = None
    asks: list[AskRow] = []
    _install_ask_log(lipm, asks)
    margins: list[MarginRow] = []
    com_hist: list[np.ndarray] = []
    vel_prev: np.ndarray | None = None
    q_hist: dict[str, list[float]] = {jn: [] for jn in LEG_JOINTS}
    joint_sq: dict[str, float] = {jn: 0.0 for jn in LEG_JOINTS}
    joint_n: dict[str, int] = {jn: 0 for jn in LEG_JOINTS}
    joint_peak: dict[str, float] = {jn: 0.0 for jn in LEG_JOINTS}
    joint_peak_t: dict[str, float] = {jn: 0.0 for jn in LEG_JOINTS}
    jerk_peak = 0.0
    jerk_joint = ""
    jerk_t = 0.0
    com_jerk_sq = 0.0
    com_jerk_n = 0
    torso_jerk_peak = 0.0
    torso_jerk_t = 0.0
    walk_jerk_sq = 0.0
    walk_jerk_n = 0
    walk_jerk_peak = 0.0
    com_stats = _empty_fracs()
    zmp_stats = _empty_fracs()
    zmp_pos_stats = _empty_fracs()
    tau_peak = 0.0
    tau_joint = ""
    tau_t = 0.0
    t_move0 = stand_s
    t_stop = stand_s + walk_s
    if select_gait:
        vx_cmd = float(sw.VOICE_VX_M_S)
    else:
        vx_cmd = float(sw.VX_FWD_CAP if vx_m_s <= 0.0 else vx_m_s)
    t_end = t_stop + stop_s
    last_send = -1.0
    stop_sent = False
    fault = ""
    dt = float(session.ctrl_dt)
    x0 = float(session.data.qpos[0])
    preview_peak = 0.0
    diag: list[str] = []
    worst_diag = ""
    worst_com = float("inf")
    while float(session.data.time) < t_end - 1e-12:
        now = float(session.data.time)
        if driver is not None:
            driver.publish(session.bus, now)
        elif now + 1e-12 < stand_s:
            pass
        elif now + 1e-12 < t_stop:
            if last_send < 0.0 or (now - last_send) >= (sw.VEL_RESEND_S - 1e-12):
                session.bus.vel(vx_cmd, 0.0, now)
                last_send = now
        elif not stop_sent:
            session.bus.stop(now)
            stop_sent = True
        n_ask = len(asks)
        session.step()
        if not kit_baseline and lipm.op3.y_swap_cmd != 0.0:
            raise SystemExit("y_swap_cmd changed")
        if kit_baseline:
            if now + 1e-12 < sw.BUS_KIT_STAND_S:
                stage = "stand"
            elif now + 1e-12 < sw.BUS_KIT_VEL_S:
                stage = "walk"
            else:
                stage = "stop"
        else:
            stage = str(lipm.preview_stage)
        for row in asks[n_ask:]:
            row.stage = stage
        hull, kind, fn_l, fn_r = _support(session)
        com = np.asarray(session.data.subtree_com[session.bid_body], dtype=np.float64).copy()
        com_hist.append(com)
        com_mm = -1000.0
        if len(hull) >= 3:
            com_mm = 1000.0 * sw.support_margin(com[:2], hull)
        cop = _cop_xy(session, kind)
        cop_mm = float("nan")
        if cop is not None and len(hull) >= 3:
            cop_mm = 1000.0 * sw.support_margin(cop, hull)
        vel_now = np.asarray(
            session.data.subtree_linvel[session.bid_body], dtype=np.float64,
        ).copy()
        zmp = _zmp_xy(com, vel_prev, vel_now, dt)
        vel_prev = vel_now
        zmp_mm = float("nan")
        if zmp is not None and len(hull) >= 3:
            zmp_mm = 1000.0 * sw.support_margin(zmp, hull)
        zmp_pos_mm = float("nan")
        if len(com_hist) >= 3 and len(hull) >= 3:
            c0p, c1p, c2p = com_hist[-3:]
            acc = (c2p - 2.0 * c1p + c0p) / (dt ** 2)
            denom = G + float(acc[2])
            if denom >= 1.0:
                zmp_pos = c2p[:2] - (float(c2p[2]) * acc[:2] / denom)
                zmp_pos_mm = 1000.0 * sw.support_margin(zmp_pos, hull)
        margins.append(MarginRow(
            t=now,
            stage=stage,
            support=kind,
            com_mm=com_mm,
            cop_mm=cop_mm,
            zmp_mm=zmp_mm,
            fn_l=fn_l,
            fn_r=fn_r,
        ))
        _note_margin(com_stats, kind, com_mm)
        _note_margin(zmp_stats, kind, zmp_mm)
        _note_margin(zmp_pos_stats, kind, zmp_pos_mm)
        for jn in LEG_JOINTS:
            hist = q_hist[jn]
            hist.append(float(lipm.q(jn)))
            if len(hist) < 4:
                continue
            q0, q1, q2, q3 = hist[-4:]
            jerk = (q3 - 3.0 * q2 + 3.0 * q1 - q0) / (dt ** 3)
            aj = abs(jerk)
            joint_sq[jn] += jerk * jerk
            joint_n[jn] += 1
            if aj > joint_peak[jn]:
                joint_peak[jn] = aj
                joint_peak_t[jn] = now
            if aj > jerk_peak:
                jerk_peak = aj
                jerk_joint = jn
                jerk_t = now
            del hist[0]
        if len(com_hist) >= 4:
            c0, c1, c2, c3 = com_hist[-4:]
            jerk_v = (c3 - 3.0 * c2 + 3.0 * c1 - c0) / (dt ** 3)
            mag = float(np.linalg.norm(jerk_v))
            com_jerk_sq += mag * mag
            com_jerk_n += 1
            if mag > torso_jerk_peak:
                torso_jerk_peak = mag
                torso_jerk_t = now
            if now + 1e-12 >= stand_s:
                walk_jerk_sq += mag * mag
                walk_jerk_n += 1
                if mag > walk_jerk_peak:
                    walk_jerk_peak = mag
        tau_name, tau_now = _leg_actuator_nm(session)
        if tau_now > tau_peak:
            tau_peak = tau_now
            tau_joint = tau_name
            tau_t = now
        preview_peak = max(preview_peak, abs(float(lipm.preview_com_y)))
        if kind in ("L", "R") and com_mm < worst_com:
            worst_com = com_mm
            foot_y = float(session.data.geom_xpos[session.lipm.gid[kind], 1])
            worst_diag = (
                f"t {now:.3f} {stage} support {kind} com_y {float(com[1]):+.4f} "
                f"foot_y {foot_y:+.4f} preview_com {float(lipm.preview_com_y):+.4f} "
                f"preview_y {float(lipm.op3.preview_y):+.4f} margin {com_mm:+.2f} mm"
            )
        if com_mm < 0.0 and len(diag) < 12:
            foot_y = 0.0
            if kind in ("L", "R"):
                foot_y = float(session.data.geom_xpos[session.lipm.gid[kind], 1])
            diag.append(
                f"OUT t {now:.3f} {stage} support {kind} com_y {float(com[1]):+.4f} "
                f"foot_y {foot_y:+.4f} preview_com {float(lipm.preview_com_y):+.4f} "
                f"fn {fn_l:.2f}/{fn_r:.2f} margin {com_mm:+.2f} mm"
            )
        if kind in ("L", "R") and len(diag) < 4:
            foot_y = float(session.data.geom_xpos[session.lipm.gid[kind], 1])
            diag.append(
                f"t {now:.3f} {stage} support {kind} com_y {float(com[1]):+.4f} "
                f"foot_y {foot_y:+.4f} preview_com {float(lipm.preview_com_y):+.4f} "
                f"preview_y {float(lipm.op3.preview_y):+.4f} margin {com_mm:+.2f} mm"
            )
        if session.bus.fault and not fault:
            fault = session.bus.fault_reason
            break
    md5_after = _plant_md5()
    session.assert_plant_unchanged()
    stages = ("stand", "start", "walk", "stop")
    by_stage: dict[str, dict[str, object]] = {}
    for stage in stages:
        com_w = _worst(margins, stage, "com_mm")
        cop_w = _worst(
            [row for row in margins if math_finite(row.cop_mm)],
            stage,
            "cop_mm",
        )
        zmp_w = _worst(
            [row for row in margins if math_finite(row.zmp_mm)],
            stage,
            "zmp_mm",
        )
        ask_w = _ask_peak(asks, stage)
        by_stage[stage] = {
            "com": _fmt_margin(com_w, "com_mm"),
            "cop": _fmt_margin(cop_w, "cop_mm"),
            "zmp": _fmt_margin(zmp_w, "zmp_mm"),
            "ask": _fmt_ask(ask_w),
            "com_mm": None if com_w is None else com_w.com_mm,
            "cop_mm": None if cop_w is None else cop_w.cop_mm,
            "zmp_mm": None if zmp_w is None else zmp_w.zmp_mm,
            "ask_nm": None if ask_w is None else ask_w.sum_nm,
        }
    ask_all = _ask_peak(asks, None)
    com_all = min(margins, key=lambda row: row.com_mm) if margins else None
    finite_cop = [row for row in margins if math_finite(row.cop_mm)]
    finite_zmp = [row for row in margins if math_finite(row.zmp_mm)]
    cop_all = min(finite_cop, key=lambda row: row.cop_mm) if finite_cop else None
    zmp_all = min(finite_zmp, key=lambda row: row.zmp_mm) if finite_zmp else None
    hard_cap = 0
    for row in asks:
        if abs(abs(row.signed_nm) - ASK_NM) <= 1e-3 and row.sum_nm > ASK_NM + 1e-3:
            hard_cap += 1
    over = [row for row in asks if row.sum_nm > ASK_NM + 1e-9]
    com_out = [row for row in margins if row.com_mm <= 0.0]
    cop_out = [row for row in finite_cop if row.cop_mm <= 0.0]
    zmp_out = [row for row in finite_zmp if row.zmp_mm <= 0.0]
    com_frac = _frac_of(com_stats["all"])
    zmp_frac = _frac_of(zmp_stats["all"])
    jerk_rms = _rms(joint_sq.get(jerk_joint, 0.0), joint_n.get(jerk_joint, 0))
    knee_rms = _rms(joint_sq["r_knee"], joint_n["r_knee"])
    torso_rms = _rms(com_jerk_sq, com_jerk_n)
    walk_rms = _rms(walk_jerk_sq, walk_jerk_n)
    ask_nm = 0.0 if ask_all is None else float(ask_all.sum_nm)
    com_min_m = -1.0 if com_all is None else float(com_all.com_mm) / 1000.0
    zmp_min_m = -1.0 if zmp_all is None else float(zmp_all.zmp_mm) / 1000.0
    up_z = float(session.min_up_z)
    margins_ok = (
        com_stats["all"].n > 0
        and com_stats["all"].n_out == 0
        and com_min_m >= 0.0
        and zmp_stats["all"].n > 0
        and zmp_stats["all"].n_out == 0
        and zmp_min_m >= 0.0
    )
    ask_ok = ask_nm <= ASK_NM + 1e-9 and hard_cap == 0 and not over
    jerk_ok = (
        torso_jerk_peak < BASE_COM_JERK_PEAK - 1e-3
        and torso_rms < BASE_COM_JERK_RMS - 1e-3
        and jerk_peak < BASE_JOINT_JERK_PEAK - 0.5
        and jerk_rms < BASE_JOINT_JERK_RMS - 0.5
        and joint_peak["r_knee"] < BASE_JOINT_JERK_PEAK - 0.5
        and knee_rms < BASE_JOINT_JERK_RMS - 0.5
    )
    tip_ok = up_z >= 0.90
    cleared = (
        margins_ok
        and ask_ok
        and jerk_ok
        and tip_ok
        and fault == ""
        and md5_after == md5_before
        and int(lipm.preview_ik_fail) == 0
    )
    result: dict[str, object] = {
        "cleared": cleared,
        "margins_ok": margins_ok,
        "ask_ok": ask_ok,
        "jerk_ok": jerk_ok,
        "tip_ok": tip_ok,
        "period_s": period_s,
        "dsp": dsp,
        "amp_m": amp_m,
        "z_m": z_m,
        "arm_s": arm_s,
        "preview_r": float(preview_r),
        "preview_shape": float(preview_shape),
        "vx_m_s": vx_cmd,
        "kit_baseline": kit_baseline,
        "y_swap_m": float(cfg.gm_y_swap_m),
        "pelvis_deg": float(cfg.gm_pelvis_deg),
        "zc_m": lipm._preview_zc,
        "plant_md5": md5_after,
        "soft_pass": 0,
        "hard_cap_ticks": hard_cap,
        "fault": fault,
        "dx_m": float(session.data.qpos[0]) - x0,
        "min_up_z": up_z,
        "ik_fail": int(lipm.preview_ik_fail),
        "gate_holds": int(lipm.preview_gate_holds),
        "n_ticks": len(margins),
        "com_out": len(com_out),
        "cop_out": len(cop_out),
        "zmp_out": len(zmp_out),
        "ask_over": len(over),
        "com_min_m": com_min_m,
        "zmp_min_m": zmp_min_m,
        "com_out_frac": com_frac,
        "zmp_out_frac": zmp_frac,
        "com_frac": _frac_dict(com_stats),
        "zmp_frac": _frac_dict(zmp_stats),
        "zmp_pos_frac": _frac_dict(zmp_pos_stats),
        "com_worst": _fmt_margin(com_all, "com_mm"),
        "cop_worst": _fmt_margin(cop_all, "cop_mm"),
        "zmp_worst": _fmt_margin(zmp_all, "zmp_mm"),
        "ask_worst": _fmt_ask(ask_all),
        "ask_nm": ask_nm,
        "jerk_joint": jerk_joint,
        "jerk_rad_s3": jerk_peak,
        "jerk_rms": jerk_rms,
        "jerk_t": jerk_t,
        "r_knee_jerk": joint_peak["r_knee"],
        "r_knee_jerk_rms": knee_rms,
        "r_knee_jerk_t": joint_peak_t["r_knee"],
        "torso_jerk_m_s3": torso_jerk_peak,
        "torso_jerk_rms": torso_rms,
        "torso_jerk_t": torso_jerk_t,
        "torso_jerk_walk_peak": walk_jerk_peak,
        "torso_jerk_walk_rms": walk_rms,
        "tau_joint": tau_joint,
        "tau_nm": tau_peak,
        "tau_t": tau_t,
        "preview_peak_m": preview_peak,
        "applied_vx_peak": _applied_peak(session),
        "applied_vx_settle": _applied_settle(session, t_stop),
        "select_gait": bool(select_gait),
        "diag": diag,
        "worst_diag": worst_diag,
        "stages": by_stage,
    }
    return result


def _frac_dict(stats: dict[str, SideFrac]) -> dict[str, dict[str, float]]:
    out: dict[str, dict[str, float]] = {}
    for key, row in stats.items():
        out[key] = {
            "n": float(row.n),
            "n_out": float(row.n_out),
            "frac": _frac_of(row),
            "min_mm": row.min_mm if row.n > 0 else float("nan"),
        }
    return out


def _fmt_frac(blob: object, key: str) -> str:
    if not isinstance(blob, dict):
        return "none"
    row = blob.get(key)
    if not isinstance(row, dict):
        return "none"
    n = int(row.get("n", 0))
    n_out = int(row.get("n_out", 0))
    frac = float(row.get("frac", float("nan")))
    min_mm = float(row.get("min_mm", float("nan")))
    return f"{frac:.3f} ({n_out}/{n}) min {min_mm:+.2f} mm"


def _print_result(result: dict[str, object]) -> None:
    cleared = bool(result["cleared"])
    label = "KIT" if bool(result.get("kit_baseline")) else "PREVIEW"
    print(
        f"{label} {'CLEAR' if cleared else 'Prefer FAIL'} "
        f"period {float(result['period_s']):.3f} s dsp {float(result['dsp']):.2f} "
        f"amp {float(result['amp_m']) * 1000.0:.1f} mm "
        f"z {float(result['z_m']) * 1000.0:.1f} mm "
        f"vx {float(result['vx_m_s']):.3f} m/s "
        f"y_swap {float(result['y_swap_m']):.3f} pelvis {float(result['pelvis_deg']):.1f} "
        f"zc {float(result['zc_m']):.3f} m"
    )
    print(
        f"  plant {result['plant_md5']} soft-pass OFF hard_cap {result['hard_cap_ticks']} "
        f"fault {result['fault'] or 'none'} dx {float(result['dx_m']):+.3f} m "
        f"min_up_z {float(result['min_up_z']):.3f} (bar {BASE_UP_Z:.3f}) "
        f"ik_fail {result['ik_fail']} gate {result['gate_holds']} "
        f"ticks {result['n_ticks']}"
    )
    print(
        f"  bars margins {result['margins_ok']} ask {result['ask_ok']} "
        f"jerk {result['jerk_ok']} tip {result['tip_ok']}"
    )
    print(
        f"  CoM out {result['com_out']} frac {float(result['com_out_frac']):.3f} "
        f"min {float(result['com_min_m']):+.6f} m  "
        f"#102 {BASE_COM_MIN_M:+.6f} m frac {BASE_COM_OUT:.3f}"
    )
    print(
        f"  CoM ss_L {_fmt_frac(result['com_frac'], 'ss_L')}  "
        f"ss_R {_fmt_frac(result['com_frac'], 'ss_R')}  "
        f"ds {_fmt_frac(result['com_frac'], 'ds')}"
    )
    print(
        f"  ZMP out {result['zmp_out']} frac {float(result['zmp_out_frac']):.3f} "
        f"min {float(result['zmp_min_m']):+.6f} m  "
        f"#102 {BASE_ZMP_MIN_M:+.6f} m frac {BASE_ZMP_OUT:.3f}"
    )
    print(
        f"  ZMP ss_L {_fmt_frac(result['zmp_frac'], 'ss_L')}  "
        f"ss_R {_fmt_frac(result['zmp_frac'], 'ss_R')}  "
        f"ds {_fmt_frac(result['zmp_frac'], 'ds')}"
    )
    print(
        f"  ZMP pos-diff ss_L {_fmt_frac(result['zmp_pos_frac'], 'ss_L')}  "
        f"all {_fmt_frac(result['zmp_pos_frac'], 'all')}"
    )
    print(f"  CoM worst {result['com_worst']}")
    print(f"  CoP worst {result['cop_worst']}  CoP out {result['cop_out']}")
    print(f"  ZMP worst {result['zmp_worst']}")
    print(
        f"  ask worst {result['ask_worst']}  ask over {result['ask_over']}  "
        f"headroom to {HEADROOM_NM:.2f} "
        f"{HEADROOM_NM - float(result['ask_nm']):+.4f} Nm  "
        f"to {ASK_NM:.2f} {ASK_NM - float(result['ask_nm']):+.4f} Nm"
    )
    print(
        f"  applied_vx peak {float(result.get('applied_vx_peak', 0.0)):+.4f} "
        f"settle {float(result.get('applied_vx_settle', 0.0)):+.4f} m/s  "
        f"select_gait {result.get('select_gait')}"
    )
    print(
        f"  actuator {result['tau_joint']} {float(result['tau_nm']):.4f} Nm "
        f"t {float(result['tau_t']):.3f} s  #102 {BASE_TAU_NM:.3f} Nm"
    )
    print(f"  preview |com| peak {float(result['preview_peak_m']) * 1000.0:.2f} mm")
    print(f"  worst {result['worst_diag']}")
    diag_rows = result["diag"]
    if isinstance(diag_rows, list):
        for line in diag_rows:
            print(f"  diag {line}")
    print(
        f"  joint jerk {result['jerk_joint']} peak {float(result['jerk_rad_s3']):.3f} "
        f"rms {float(result['jerk_rms']):.3f} rad/s^3 t {float(result['jerk_t']):.3f} s  "
        f"#102 {BASE_JOINT_JERK_PEAK:.3f} / {BASE_JOINT_JERK_RMS:.3f} r_knee"
    )
    print(
        f"  r_knee jerk peak {float(result['r_knee_jerk']):.3f} "
        f"rms {float(result['r_knee_jerk_rms']):.3f} "
        f"t {float(result['r_knee_jerk_t']):.3f} s"
    )
    print(
        f"  CoM jerk peak {float(result['torso_jerk_m_s3']):.3f} "
        f"rms {float(result['torso_jerk_rms']):.3f} m/s^3 "
        f"t {float(result['torso_jerk_t']):.3f} s  "
        f"#102 {BASE_COM_JERK_PEAK:.3f} / {BASE_COM_JERK_RMS:.3f}"
    )
    print(
        f"  CoM jerk after stand peak {float(result['torso_jerk_walk_peak']):.3f} "
        f"rms {float(result['torso_jerk_walk_rms']):.3f} m/s^3"
    )
    stages = result["stages"]
    if isinstance(stages, dict):
        for name in ("stand", "start", "walk", "stop"):
            row = stages.get(name)
            if not isinstance(row, dict):
                continue
            print(f"  [{name}] CoM {row['com']}")
            print(f"  [{name}] CoP {row['cop']}")
            print(f"  [{name}] ZMP {row['zmp']}")
            print(f"  [{name}] ask {row['ask']}")


def render_side_front(
    out_mp4: Path,
    stand_s: float = 3.0,
    walk_s: float = 11.0,
    stop_s: float = 6.0,
) -> None:
    """Side and front of the voice bout. 30 fps. No second mj_forward."""
    import tempfile

    import imageio.v2 as imageio

    import walk_gait_ainex as wg

    cfg = sw.gait_for_command(sw.VOICE_VX_M_S, 0.0)
    if cfg.preview_amp_m <= 1e-6 or cfg.gm_y_swap_m != 0.0:
        raise SystemExit("voice command did not select the preview gait")
    session = sw.SteerSession(
        video=True,
        lipm=cfg,
        cam_distance=1.70,
        cam_elevation=-8.0,
        cam_azimuth=90.0,
    )
    if session.lipm is None or session.lipm.op3 is None or session.lipm.op3.y_swap_cmd != 0.0:
        raise SystemExit("preview walker is not the voice gait")
    script = sw.voice_bus_script(stand_s, walk_s, stop_s, 0.0)
    driver = sw.ScriptedDriver(script)
    t_end = float(script[-1].t_end)
    next_frame = 0.0
    frame_dt = 1.0 / 30.0
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="voice_frames_") as tmp:
        folder = Path(tmp)
        n = 0
        while float(session.data.time) < t_end - 1e-12:
            now = float(session.data.time)
            driver.publish(session.bus, now)
            session.step()
            if session.lipm.op3.y_swap_cmd != 0.0:
                raise SystemExit("y_swap_cmd changed")
            if now + 1e-9 < next_frame:
                continue
            session.cam.distance = 1.70
            session.cam.elevation = -8.0
            session.cam.azimuth = 90.0
            side = session.render([f"side  t {now:5.2f}s"])
            session.cam.azimuth = 0.0
            front = session.render([
                f"front  t {now:5.2f}s  vx {session.bus.applied_vx:+.3f}",
            ])
            imageio.imwrite(folder / f"frame_{n:05d}.png", np.concatenate([side, front], axis=1))
            n += 1
            next_frame += frame_dt
        if n < 2:
            raise SystemExit("render produced no frames")
        wg._encode_mp4_ffmpeg(folder, out_mp4, fps=30)
    print(
        f"  wrote {out_mp4}  frames {n}  video {n / 30.0:.2f} s  "
        f"sim {t_end:.2f} s  fault {session.bus.fault_reason or 'none'}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Score preview CoM/ZMP and unclamped ask")
    parser.add_argument("--period", type=float, default=1.40)
    parser.add_argument("--dsp", type=float, default=0.50)
    parser.add_argument("--amp", type=float, default=0.016)
    parser.add_argument("--z", type=float, default=0.012)
    parser.add_argument("--arm", type=float, default=0.60)
    parser.add_argument("--stand", type=float, default=0.40)
    parser.add_argument("--walk", type=float, default=5.00)
    parser.add_argument("--stop", type=float, default=1.60)
    parser.add_argument("--tag", type=str, default="a")
    parser.add_argument("--baseline", action="store_true")
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--preview-r", type=float, default=1.0e-4)
    parser.add_argument("--preview-shape", type=float, default=1.0)
    parser.add_argument("--select-gait", action="store_true")
    parser.add_argument("--render", type=str, default="")
    args = parser.parse_args()
    if args.render:
        render_side_front(Path(args.render), stand_s=float(args.stand), walk_s=float(args.walk), stop_s=float(args.stop))
        return
    result = run_attempt(
        period_s=float(args.period),
        dsp=float(args.dsp),
        amp_m=float(args.amp),
        z_m=float(args.z),
        arm_s=float(args.arm),
        stand_s=float(args.stand),
        walk_s=float(args.walk),
        stop_s=float(args.stop),
        kit_baseline=bool(args.baseline),
        vx_m_s=float(args.vx),
        preview_r=float(args.preview_r),
        preview_shape=float(args.preview_shape),
        select_gait=bool(args.select_gait),
    )
    _print_result(result)
    out = Path("previews") / f"com_zmp_preview_{args.tag}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
