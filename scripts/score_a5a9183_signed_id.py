#!/usr/bin/env python3
"""Independent score of Controls' cadence tip a5a9183 on the #102 bar set.

The gait and the plant are loaded from that tip. This file does not edit
either. Soft-pass stays off. q̈ is ``data.qacc`` from ``mj_step``, not a
finite difference of qvel.

Each control tick integrates four 2 ms substeps. An over-bar write is paired
with the later substep of that tick whose ``|qfrc_inverse|`` on that joint is
largest. Inverse dynamics runs at the q, q̇ that entered that ``mj_step``,
with the contacts ``mj_inverse`` builds at that state, and with that
substep's ``data.qacc``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("DISPLAY", ":1")
os.environ["MUJOCO_GL"] = "glfw"
TIP_SCRIPTS = Path(os.environ.get("A5_TIP_SCRIPTS", "/tmp/tip-a5a9183/scripts"))
os.environ["WALK_GAIT_SCRIPTS"] = str(TIP_SCRIPTS)

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import mujoco as mj
import numpy as np

import score_walk_smoothness as sws
import step_bars
import zmp_preview

TIP_SHA = "a5a9183461105016211bebbf110051c2bb8f9df2"
PLANT_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
ASK_BAR = 2.33
G = 9.81
HAND_M_KG = 2.2
HAND_L_M = 0.093
HAND_LABEL = "hand estimate"
QACC_SOURCE = (
    "data.qacc from mj_step of the physics substep after the write with the "
    "largest |qfrc_inverse| on that joint; not a finite difference of qvel"
)
PERIODS = (0.80, 0.75, 0.70, 0.65, 0.60, 0.55, 0.50, 1.00, 1.20)
SPEEDS = (0.016, 0.024, 0.032, 0.040, 0.048, 0.056)
TERMS = ("armature·q̈", "impact/contact", "link inertia", "gravity")

_ORIG_MJ_STEP = mj.mj_step
_CAPTURE: list[dict[str, np.ndarray]] = []
_CAPTURE_ON = False


def _capturing_step(model: mj.MjModel, data: mj.MjData) -> None:
    if _CAPTURE_ON:
        _CAPTURE.append({
            "qpos": np.array(data.qpos, dtype=np.float64, copy=True),
            "qvel": np.array(data.qvel, dtype=np.float64, copy=True),
            "ctrl": np.array(data.ctrl, dtype=np.float64, copy=True),
        })
    _ORIG_MJ_STEP(model, data)
    if _CAPTURE_ON and _CAPTURE:
        _CAPTURE[-1]["qacc"] = np.array(data.qacc, dtype=np.float64, copy=True)
        _CAPTURE[-1]["actuator"] = np.array(data.qfrc_actuator, dtype=np.float64, copy=True)


mj.mj_step = _capturing_step


@dataclass
class Write:
    t_s: float
    joint: str
    q_des: float
    q: float
    omega: float
    kp: float
    kv: float
    ctrl: float
    signed_nm: float
    sum_nm: float
    ask_nm: float
    kp_e_ctrl: float
    kv_qdot: float
    steps_before: int
    phase: str
    stance: str
    stage: str
    swing_side: str
    swing_frac: float | None
    tick: int


def _plant_md5() -> str:
    digest = hashlib.md5()
    digest.update(sws.steer_walk.PLANT_XML.read_bytes())
    return digest.hexdigest()


def _forcerange_audit(model: mj.MjModel, walker: sws.lipm_gait.LipmWalker) -> dict[str, object]:
    rows = []
    ok = True
    for name in sws.LEG_JOINTS:
        idx = walker.act_idx[name + "_pos"]
        jid = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name))
        act = [float(model.actuator_forcerange[idx, 0]), float(model.actuator_forcerange[idx, 1])]
        jnt = [float(model.jnt_actfrcrange[jid, 0]), float(model.jnt_actfrcrange[jid, 1])]
        arm = float(model.dof_armature[int(model.jnt_dofadr[jid])])
        match = (
            abs(act[0] + 2.45) < 1e-9 and abs(act[1] - 2.45) < 1e-9
            and abs(jnt[0] + 2.45) < 1e-9 and abs(jnt[1] - 2.45) < 1e-9
        )
        ok = ok and match
        rows.append({
            "joint": name,
            "actuator_forcerange": act,
            "joint_actfrcrange": jnt,
            "dof_armature": arm,
            "pm_2_45": match,
        })
    return {"n": len(rows), "all_pm_2_45": ok, "joints": rows}


def _amp(period_s: float, cache: dict[float, float]) -> float:
    key = float(period_s)
    if key not in cache:
        cache[key] = float(zmp_preview.sway_zmp_amp(
            key, 0.25, 0.18, 0.016, 0.043, 0.0, 1.0e-4,
        ))
    return cache[key]


def _config(period_s: float, vx: float, amp: float) -> sws.lipm_gait.LipmConfig:
    return sws.lipm_gait.LipmConfig(
        name="voice056",
        clear_m=0.008,
        arms=True,
        schedule="gait_manager",
        gm_period_s=float(period_s),
        gm_dsp=0.25,
        gm_y_swap_m=0.0,
        gm_x_m=abs(float(vx)) * float(period_s) / 4.0,
        gm_z_m=0.008,
        gm_z_swap_m=0.0,
        gm_pelvis_deg=0.0,
        gm_hip_pitch_deg=15.0,
        gm_start_lead="L",
        gm_crouch_m=0.025,
        gm_move_s=0.020,
        preview_amp_m=float(amp),
        preview_arm_s=1.0,
        preview_r=1.0e-4,
        preview_shape=0.0,
    )


def _swing(op3: object) -> tuple[str, float | None]:
    if op3 is None:
        return "", None
    period = float(getattr(op3, "period", 0.0))
    if period <= 1e-9:
        return "", None
    t = float(getattr(op3, "time")) % period
    spans = (
        ("L", float(op3.l_ssp_start), float(op3.l_ssp_end)),
        ("R", float(op3.r_ssp_start), float(op3.r_ssp_end)),
    )
    for side, start, end in spans:
        if start < t <= end and end > start + 1e-12:
            return side, (t - start) / (end - start)
    return "", None


def _install(walker: sws.lipm_gait.LipmWalker, bucket: list[Write], asks: list[sws.AskSample], tick_box: list[int]) -> None:
    def _log(jn: str, q_des: float) -> None:
        if jn not in sws.LEG_JOINTS:
            return
        idx = walker.act_idx.get(jn + "_pos")
        if idx is None:
            return
        jid = int(mj.mj_name2id(walker.model, mj.mjtObj.mjOBJ_JOINT, jn))
        q = float(walker.q(jn))
        omega = float(walker.data.qvel[int(walker.model.jnt_dofadr[jid])])
        kp = float(walker.model.actuator_gainprm[idx, 0])
        kv = -float(walker.model.actuator_biasprm[idx, 2])
        ctrl = float(walker.data.ctrl[idx])
        kp_e = kp * (float(q_des) - q)
        kv_qdot = kv * omega
        signed = kp_e - kv_qdot
        ask = kp * (ctrl - q) - kv_qdot
        side, frac = _swing(walker.op3)
        rec = Write(
            t_s=float(walker.data.time),
            joint=jn,
            q_des=float(q_des),
            q=q,
            omega=omega,
            kp=kp,
            kv=kv,
            ctrl=ctrl,
            signed_nm=float(signed),
            sum_nm=float(abs(kp_e) + abs(kv_qdot)),
            ask_nm=float(ask),
            kp_e_ctrl=float(kp * (ctrl - q)),
            kv_qdot=float(kv_qdot),
            steps_before=len(_CAPTURE),
            phase=str(walker.phase),
            stance=str(walker.stance),
            stage=str(getattr(walker, "preview_stage", "")),
            swing_side=side,
            swing_frac=None if frac is None else float(frac),
            tick=int(tick_box[0]),
        )
        bucket.append(rec)
        asks.append(sws.AskSample(
            t_s=rec.t_s,
            joint=jn,
            signed_nm=rec.signed_nm,
            sum_nm=rec.sum_nm,
            ask_nm=rec.ask_nm,
            stage=rec.stage,
            qvel_abs=abs(omega),
        ))

    orig = walker.write_clipped

    def wrapped(jn: str, q_des: float) -> None:
        orig(jn, q_des)
        _log(jn, q_des)

    walker.write_clipped = wrapped  # type: ignore[method-assign]
    limited = walker.write_force_limited

    def wrapped_limited(jn: str, q_des: float, limit_nm: float | None = None) -> None:
        limited(jn, q_des, limit_nm)
        _log(jn, q_des)

    walker.write_force_limited = wrapped_limited  # type: ignore[method-assign]


def _round(value: float | None, digits: int = 6) -> float | None:
    if value is None or not math.isfinite(float(value)):
        return None
    return round(float(value), digits)


def _inverse_split(
    model: mj.MjModel,
    scratch: mj.MjData,
    grav_data: mj.MjData,
    shot: dict[str, np.ndarray],
) -> dict[str, np.ndarray]:
    scratch.qpos[:] = shot["qpos"]
    scratch.qvel[:] = shot["qvel"]
    scratch.qacc[:] = shot["qacc"]
    scratch.ctrl[:] = shot["ctrl"]
    scratch.qfrc_applied[:] = 0.0
    scratch.xfrc_applied[:] = 0.0
    mj.mj_inverse(model, scratch)
    qacc = np.array(shot["qacc"], dtype=np.float64, copy=True)
    # mj_inverse keeps the supplied qacc. Re-read in case a build replaces it.
    if float(np.max(np.abs(scratch.qacc - qacc))) > 1e-8:
        qacc = np.array(scratch.qacc, dtype=np.float64, copy=True)
    mqacc = np.zeros(model.nv, dtype=np.float64)
    mj.mj_mulM(model, scratch, mqacc, qacc)
    arm = np.asarray(model.dof_armature, dtype=np.float64) * qacc
    grav_data.qpos[:] = shot["qpos"]
    grav_data.qvel[:] = 0.0
    grav_data.qacc[:] = 0.0
    mj.mj_fwdPosition(model, grav_data)
    mj.mj_fwdVelocity(model, grav_data)
    gravity = np.array(grav_data.qfrc_bias, dtype=np.float64, copy=True)
    coriolis = np.array(scratch.qfrc_bias, dtype=np.float64, copy=True) - gravity
    contact = -np.array(scratch.qfrc_constraint, dtype=np.float64, copy=True)
    passive = -np.array(scratch.qfrc_passive, dtype=np.float64, copy=True)
    link = (mqacc - arm) + coriolis
    ident = arm + link + gravity + contact + passive
    return {
        "id": np.array(scratch.qfrc_inverse, dtype=np.float64, copy=True),
        "arm_model": arm,
        "arm_001": 0.01 * qacc,
        "link": link,
        "gravity": gravity,
        "contact": contact,
        "passive": passive,
        "ident": ident,
        "qacc": qacc,
        "actuator": np.array(shot["actuator"], dtype=np.float64, copy=True),
    }


def _cop_mm(
    model: mj.MjModel,
    data: mj.MjData,
    foot_gid: int,
    floor_gid: int,
    ankle_bid: int,
) -> tuple[float | None, float | None]:
    got = sws.foot_floor_cop(model, data, foot_gid, floor_gid, (floor_gid,))
    if got is None:
        return None, None
    world, _fn = got
    origin = np.asarray(data.xpos[ankle_bid], dtype=np.float64)
    rot = np.asarray(data.xmat[ankle_bid], dtype=np.float64).reshape(3, 3)
    local = rot.T @ (np.asarray(world, dtype=np.float64) - origin)
    return float(local[0] * 1000.0), float(local[1] * 1000.0)


def _phase_label(
    mode: str,
    t: float,
    touchdowns: list[float],
    n_l: int,
    n_r: int,
    swing_side: str,
    swing_frac: float | None,
) -> str:
    for event in touchdowns:
        if abs(t - event) <= 0.020 + 1e-9:
            return "touchdown"
    if mode == "stop":
        return "stop"
    if mode == "stand":
        return "stand"
    if (n_l == 0) ^ (n_r == 0):
        side = "L" if n_l == 0 else "R"
        if swing_side in ("L", "R"):
            side = swing_side
        pct = 0.0 if swing_frac is None else 100.0 * float(swing_frac)
        return f"ss {side} {pct:.0f}%"
    return "ds"


def _dominant(parts: dict[str, float]) -> str:
    return max(TERMS, key=lambda name: abs(parts[name]))


def _classify(ask_abs: float, id_abs: float, parts: dict[str, float]) -> tuple[str, str]:
    if id_abs > ASK_BAR + 1e-9:
        return "physics candidate", _dominant(parts)
    if ask_abs > ASK_BAR + 1e-9:
        return "controller fail", ""
    return "under bar", ""


def _jsonable(value: object) -> object:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        return value
    if isinstance(value, (np.floating, np.integer)):
        return _jsonable(float(value) if isinstance(value, np.floating) else int(value))
    return value


def score_cell(period_s: float, vx: float, amp: float) -> dict[str, object]:
    global _CAPTURE_ON
    digest_before = _plant_md5()
    if digest_before != PLANT_MD5:
        raise SystemExit(f"plant md5 {digest_before} != {PLANT_MD5}")
    stand_s = 0.25
    walk_s = 1.00 + 2.05 * float(period_s)
    stop_s = 2.40
    t_stop = stand_s + walk_s
    t_end = t_stop + stop_s
    cfg = _config(period_s, vx, amp)
    session = sws.steer_walk.SteerSession(video=False, lipm=cfg)
    walker = session.lipm
    if walker is None or walker.op3 is None:
        raise RuntimeError("walker did not build")
    if walker.op3.y_swap_cmd != 0.0 or cfg.gm_y_swap_m != 0.0:
        raise RuntimeError("y_swap is not 0")
    if cfg.name != "voice056":
        raise RuntimeError("voice path is not voice056")
    audit = _forcerange_audit(session.model, walker)
    if not audit["all_pm_2_45"]:
        raise RuntimeError("forcerange is not ±2.45 on every leg joint")
    writes: list[Write] = []
    asks: list[sws.AskSample] = []
    tick_box = [0]
    _install(walker, writes, asks, tick_box)
    scratch = mj.MjData(session.model)
    grav_data = mj.MjData(session.model)
    dof = {
        name: int(session.model.jnt_dofadr[mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_JOINT, name)])
        for name in sws.LEG_JOINTS
    }
    ankle = {
        "l": int(mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_BODY, "l_ank_pitch_link")),
        "r": int(mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_BODY, "r_ank_pitch_link")),
    }
    foot = {"l": int(walker.gid["L"]), "r": int(walker.gid["R"])}
    floor_gid = int(walker.gid_floor)
    times: list[float] = []
    pre_times: list[float] = []
    com_rows: list[np.ndarray] = []
    q_rows: list[np.ndarray] = []
    joint_names: list[str] = []
    reasons: list[str] = []
    step_cols: dict[str, list[object]] = {key: [] for key in (
        "t", "x_l", "x_r", "y_l", "y_r", "n_l", "n_r", "z_l", "z_r",
        "declared", "mode", "roll", "pitch", "roll_l", "pitch_l", "roll_r", "pitch_r",
        "up_z", "zmp_act", "com_act", "cop_l", "cop_r",
    )}
    q_stand_rows: list[list[float]] = []
    qvel_abs_rows: list[list[float]] = []
    pair_signed: list[list[float | None]] = []
    pair_sum: list[list[float | None]] = []
    pair_qvel: list[list[float | None]] = []
    pair_qvel_sum: list[list[float | None]] = []
    pair_ask: list[list[float | None]] = []
    pair_qvel_ask: list[list[float | None]] = []
    stage_ticks: list[str] = []
    trunk_x: list[float] = []
    trunk_y: list[float] = []
    trunk_yaw: list[float] = []
    tick_contact: list[dict[str, float | int | str]] = []
    body_vx_sum = 0.0
    body_vx_n = 0
    phases: dict[str, sws.PhaseScore] = {
        name: sws.PhaseScore(phase=name) for name in ("stand", "ds", "ss_L", "ss_R", "unknown")
    }
    last_send = -1.0
    stop_sent = False
    ctrl_dt = float(session.ctrl_dt)
    vx_cmd = float(vx)
    while float(session.data.time) < t_end - 1e-12:
        now = float(session.data.time)
        if now + 1e-12 < stand_s:
            mode = "stand"
        elif now + 1e-12 < t_stop:
            mode = "move"
            if last_send < 0.0 or (now - last_send) >= (sws.steer_walk.VEL_RESEND_S - 1e-12):
                session.bus.vel(vx_cmd, 0.0, now)
                last_send = now
        else:
            mode = "stop"
            if not stop_sent:
                session.bus.stop(now)
                stop_sent = True
        n_ask = len(asks)
        _CAPTURE.clear()
        _CAPTURE_ON = True
        try:
            session.step()
        finally:
            _CAPTURE_ON = False
        shots = list(_CAPTURE)
        new_writes = writes[n_ask:]
        new_asks = asks[n_ask:]
        stage_name = str(getattr(walker, "preview_stage", "unknown"))
        for sample in new_asks:
            sample.stage = stage_name
        signed_tick, sum_tick, qvel_tick, qvel_sum_tick, ask_tick, qvel_ask_tick = sws._tick_pair(
            new_asks, sws.LEG_JOINTS,
        )
        pair_signed.append(signed_tick)
        pair_sum.append(sum_tick)
        pair_qvel.append(qvel_tick)
        pair_qvel_sum.append(qvel_sum_tick)
        pair_ask.append(ask_tick)
        pair_qvel_ask.append(qvel_ask_tick)
        if walker.op3.y_swap_cmd != 0.0:
            raise RuntimeError("y_swap_cmd changed")
        if session.bus.fault and session.bus.fault_reason and session.bus.fault_reason not in reasons:
            reasons.append(session.bus.fault_reason)
        tick = sws.sample_zmp(session, walker)
        t_post = float(session.data.time)
        phases[tick.phase].add(tick, t_post)
        names, q = sws.actuated_q(session.model, session.data)
        if not joint_names:
            joint_names = names
        com_rows.append(np.asarray(session.data.subtree_com[walker.bid_body], dtype=np.float64).copy())
        q_rows.append(q)
        times.append(t_post)
        pre_times.append(now)
        grounds = sws._ground_gids(session, walker)
        n_l = sws._contact_count(session.data, int(walker.gid["L"]), grounds)
        n_r = sws._contact_count(session.data, int(walker.gid["R"]), grounds)
        x_l, y_l, z_l, roll_l, pitch_l = sws._foot_box_sample(
            session.model, session.data, int(walker.gid["L"]), None,
        )
        x_r, y_r, z_r, roll_r, pitch_r = sws._foot_box_sample(
            session.model, session.data, int(walker.gid["R"]), None,
        )
        roll, pitch, up_z = sws._trunk_angles(session.data, int(walker.bid_body))
        rot = np.asarray(session.data.xmat[int(walker.bid_body)], dtype=np.float64).reshape(3, 3)
        origin = np.asarray(session.data.xpos[int(walker.bid_body)], dtype=np.float64)
        qv = session.data.qvel
        qvel_abs_rows.append([abs(float(qv[dof[name]])) for name in sws.LEG_JOINTS])
        stage_ticks.append(stage_name)
        trunk_x.append(float(origin[0]))
        trunk_y.append(float(origin[1]))
        trunk_yaw.append(math.atan2(float(rot[1, 0]), float(rot[0, 0])))
        zmp_act, com_act = sws._actual_margins(session, walker, grounds, n_l, n_r)
        step_cols["t"].append(t_post)
        step_cols["x_l"].append(x_l)
        step_cols["x_r"].append(x_r)
        step_cols["y_l"].append(y_l)
        step_cols["y_r"].append(y_r)
        step_cols["n_l"].append(n_l)
        step_cols["n_r"].append(n_r)
        step_cols["z_l"].append(z_l)
        step_cols["z_r"].append(z_r)
        step_cols["declared"].append(tick.phase)
        step_cols["mode"].append(mode)
        step_cols["roll"].append(roll)
        step_cols["pitch"].append(pitch)
        step_cols["roll_l"].append(roll_l)
        step_cols["pitch_l"].append(pitch_l)
        step_cols["roll_r"].append(roll_r)
        step_cols["pitch_r"].append(pitch_r)
        step_cols["up_z"].append(up_z)
        step_cols["zmp_act"].append(zmp_act)
        step_cols["com_act"].append(com_act)
        step_cols["cop_l"].append(sws._foot_cop_margin(session, walker, "L", grounds))
        step_cols["cop_r"].append(sws._foot_cop_margin(session, walker, "R", grounds))
        q_stand_rows.append(sws._leg_q(session.model, session.data))
        tick_contact.append({
            "t": t_post,
            "t_pre": now,
            "mode": mode,
            "n_l": n_l,
            "n_r": n_r,
        })
        if stand_s - 1e-12 <= now < t_stop - 1e-12:
            body_vx_sum += float(session._body_forward_speed())
            body_vx_n += 1
        over_idx = [
            i for i, rec in enumerate(new_writes)
            if abs(rec.ask_nm) > ASK_BAR + 1e-9 or abs(rec.signed_nm) > ASK_BAR + 1e-9
        ]
        if over_idx and shots:
            splits = [
                _inverse_split(session.model, scratch, grav_data, shot)
                for shot in shots
            ]
            cop_cache: dict[tuple[int, str], tuple[float | None, float | None]] = {}
            for i in over_idx:
                rec = new_writes[i]
                later = [
                    j for j, other in enumerate(new_writes)
                    if other.joint == rec.joint and other.steps_before > rec.steps_before
                ]
                end = min(item.steps_before for item in (new_writes[j] for j in later)) if later else len(shots)
                start = min(rec.steps_before, len(shots) - 1)
                end = max(start + 1, min(end, len(shots)))
                best_j = start
                best_abs = -1.0
                adr = dof[rec.joint]
                for j in range(start, end):
                    mag = abs(float(splits[j]["id"][adr]))
                    if mag > best_abs:
                        best_abs = mag
                        best_j = j
                split = splits[best_j]
                # Rebuild contacts at the chosen pre-step state for the CoP.
                scratch.qpos[:] = shots[best_j]["qpos"]
                scratch.qvel[:] = shots[best_j]["qvel"]
                scratch.qacc[:] = split["qacc"]
                mj.mj_fwdPosition(session.model, scratch)
                side = "l" if rec.joint.startswith("l_") else "r"
                key = (best_j, side)
                if key not in cop_cache:
                    cop_cache[key] = _cop_mm(
                        session.model, scratch, foot[side], floor_gid, ankle[side],
                    )
                cop_x, cop_y = cop_cache[key]
                parts = {
                    "armature·q̈": float(split["arm_001"][adr]),
                    "impact/contact": float(split["contact"][adr]),
                    "link inertia": float(split["link"][adr]),
                    "gravity": float(split["gravity"][adr]),
                }
                ident = float(split["ident"][adr])
                id_nm = float(split["id"][adr])
                rec_extra = getattr(rec, "id", None)
                del rec_extra
                setattr(rec, "id_blob", {
                    "id_nm": id_nm,
                    "id_abs_nm": abs(id_nm),
                    "parts": parts,
                    "passive_nm": float(split["passive"][adr]),
                    "ident_nm": ident,
                    "ident_residual_nm": id_nm - ident,
                    "armature_model_qacc_nm": float(split["arm_model"][adr]),
                    "qacc_rad_s2": float(split["qacc"][adr]),
                    "actuator_nm": float(split["actuator"][adr]),
                    "substep": best_j,
                    "n_substeps": len(shots),
                    "cop_x_mm": cop_x,
                    "cop_y_mm": cop_y,
                })
        tick_box[0] += 1
        if session.bus.fault:
            break
    session.assert_plant_unchanged()
    digest_after = _plant_md5()
    n = len(times)
    dt = sws._uniform_dt(np.asarray(times, dtype=np.float64)) if n else None
    com_arr = np.stack(com_rows, axis=0) if com_rows else np.zeros((0, 3))
    q_arr = np.stack(q_rows, axis=0) if q_rows else np.zeros((0, 0))
    com_jerk = sws.jerk_stats(com_arr, dt, ["x", "y", "z"], "body_link subtree_com world", "m/s^3") if dt is not None else None
    joint_jerk = sws.jerk_stats(q_arr, dt, joint_names, "actuated hinge qpos", "rad/s^3") if dt is not None and joint_names else None
    zmp_out = sum(row.n_zmp_outside for row in phases.values())
    zmp_min: float | None = None
    com_out = 0
    com_known = 0
    com_min: float | None = None
    for row in phases.values():
        if row.zmp_min_m is not None and (zmp_min is None or row.zmp_min_m < zmp_min):
            zmp_min = row.zmp_min_m
        com_known += row.n_com
        com_out += row.n_com_outside
        if row.com_min_m is not None and (com_min is None or row.com_min_m < com_min):
            com_min = row.com_min_m
    zmp_frac = (zmp_out / n) if n else 1.0
    com_frac = (com_out / com_known) if com_known else 1.0
    q_mat = np.asarray(q_stand_rows, dtype=np.float64) if q_stand_rows else np.zeros((0, 12))
    q_err = np.zeros(q_mat.shape[0], dtype=np.float64)
    if q_mat.shape[0] > 0:
        t_arr = np.asarray(step_cols["t"], dtype=np.float64)
        stand_mask = t_arr <= stand_s + 1e-12
        if not np.any(stand_mask):
            stand_mask = np.zeros(q_mat.shape[0], dtype=bool)
            stand_mask[0] = True
        q_ref = np.median(q_mat[stand_mask], axis=0)
        q_err = np.max(np.abs(q_mat - q_ref), axis=1)
    stepping = step_bars.summarize_trace(
        step_bars.as_arrays(step_cols),
        vx_m_s=vx_cmd,
        period_s=float(period_s),
        q_stand_err=q_err,
    )
    half_x = float(session.model.geom_size[int(walker.gid["L"]), 0])
    if abs(half_x - 0.0675) > 1e-6:
        stepping["fail_reasons"].append(
            f"contact box half-length {half_x * 1000.0:.2f} mm is not 67.5 mm"
        )
        stepping["passes"] = False
    pair_ask_m = sws._pair_matrix(pair_ask, len(sws.LEG_JOINTS))
    pair_sum_m = sws._pair_matrix(pair_sum, len(sws.LEG_JOINTS))
    torque_bar = step_bars.signed_torque_bar(sws.LEG_JOINTS, pair_ask_m, pair_sum_m)
    line_t, line_tau, line_w, line_stage = sws._writes_for_line(asks, sws.LEG_JOINTS)
    dc = step_bars.speed_torque_check(
        sws.LEG_JOINTS, line_t, line_tau, line_w, line_stage,
        no_load_speed=step_bars.DC_MOTOR_NO_LOAD_RAD_S,
        stall_torque=step_bars.DC_MOTOR_STALL_NM,
        voltage=step_bars.DC_MOTOR_VOLTAGE_V,
        label=step_bars.DC_MOTOR_LABEL,
    )
    clamp = step_bars.signed_ask_clamp_bar(sws.LEG_JOINTS, pair_ask_m)
    qvel_report = step_bars.hinge_speed_report(
        sws.LEG_JOINTS,
        np.asarray(times, dtype=np.float64),
        np.asarray(qvel_abs_rows, dtype=np.float64) if qvel_abs_rows else np.zeros((0, 12)),
        np.asarray(stage_ticks, dtype=object),
    )
    speed = step_bars.trunk_speed_line(
        np.asarray(times, dtype=np.float64),
        np.asarray(trunk_x, dtype=np.float64),
        np.asarray(trunk_y, dtype=np.float64),
        np.asarray(trunk_yaw, dtype=np.float64),
        np.asarray(step_cols["mode"], dtype=object),
        period_s=float(period_s),
        vx_cmd_m_s=vx_cmd,
    )
    tipped = bool(session.min_up_z < sws.TIP_UP_Z or reasons)
    measured = (
        n > 0 and zmp_min is not None and com_min is not None
        and com_jerk is not None and joint_jerk is not None
        and digest_after == PLANT_MD5
    )
    signed_reasons = torque_bar.get("fail_reasons")
    fail_reasons = sws._mfg_reasons(
        measured=measured,
        zmp_min_m=zmp_min,
        zmp_frac=zmp_frac,
        com_min_m=com_min,
        com_frac=com_frac,
        tipped=tipped,
        ask_nm=0.0,
        ask_over_ticks=0,
        com_peak=None if com_jerk is None else com_jerk.peak_l2,
        com_rms=None if com_jerk is None else com_jerk.rms_l2,
        joint_peak=None if joint_jerk is None else joint_jerk.peak_l2,
        joint_rms=None if joint_jerk is None else joint_jerk.rms_l2,
        torque_reasons=[str(item) for item in signed_reasons] if isinstance(signed_reasons, list) else [],
    )
    for blob in (stepping, qvel_report, dc, clamp):
        extra = blob.get("fail_reasons") if isinstance(blob, dict) else None
        if isinstance(extra, list):
            fail_reasons.extend(str(item) for item in extra)
    touchdowns: list[float] = []
    prev_l = 1
    prev_r = 1
    for row in tick_contact:
        n_l = int(row["n_l"])
        n_r = int(row["n_r"])
        if prev_l == 0 and n_l > 0:
            touchdowns.append(float(row["t"]))
        if prev_r == 0 and n_r > 0:
            touchdowns.append(float(row["t"]))
        prev_l, prev_r = n_l, n_r
    contact_at = {int(i): row for i, row in enumerate(tick_contact)}
    ctrl_hist: dict[str, list[float]] = {name: [] for name in sws.LEG_JOINTS}
    events: list[dict[str, object]] = []
    max_resid = 0.0
    for rec in writes:
        ctrl_hist[rec.joint].append(rec.ctrl)
        history = ctrl_hist[rec.joint]
        d2 = None
        if len(history) >= 3:
            d2 = history[-1] - 2.0 * history[-2] + history[-3]
        ask_over = abs(rec.ask_nm) > ASK_BAR + 1e-9
        qdes_over = abs(rec.signed_nm) > ASK_BAR + 1e-9
        if not (ask_over or qdes_over):
            continue
        blob = getattr(rec, "id_blob", None)
        tick_row = contact_at.get(rec.tick, {})
        phase = _phase_label(
            str(tick_row.get("mode", "")),
            float(tick_row.get("t", rec.t_s)),
            touchdowns,
            int(tick_row.get("n_l", 0)),
            int(tick_row.get("n_r", 0)),
            rec.swing_side,
            rec.swing_frac,
        )
        hand = HAND_M_KG * G * HAND_L_M * math.sin(abs(rec.q) / 2.0)
        if not isinstance(blob, dict):
            kind, term = ("unmeasured", "")
            id_abs = float("nan")
            parts = {name: float("nan") for name in TERMS}
            resid = float("nan")
        else:
            parts = {name: float(blob["parts"][name]) for name in TERMS}
            id_abs = float(blob["id_abs_nm"])
            kind, term = _classify(abs(rec.ask_nm), id_abs, parts)
            resid = abs(float(blob["ident_residual_nm"]))
            max_resid = max(max_resid, resid)
        events.append({
            "t_s": rec.t_s,
            "t_post_s": tick_row.get("t"),
            "joint": rec.joint,
            "over_on": "both" if ask_over and qdes_over else ("ask" if ask_over else "qdes"),
            "ask_nm": rec.ask_nm,
            "qdes_signed_nm": rec.signed_nm,
            "sum_nm": rec.sum_nm,
            "kp_e_nm": rec.kp_e_ctrl,
            "kv_qdot_nm": rec.kv_qdot,
            "kp_e_qdes_nm": rec.kp * (rec.q_des - rec.q),
            "q_rad": rec.q,
            "q_des_rad": rec.q_des,
            "qvel_rad_s": rec.omega,
            "ctrl_rad": rec.ctrl,
            "ctrl_second_diff_rad": d2,
            "id_nm": None if not isinstance(blob, dict) else blob["id_nm"],
            "armature_qacc_nm": None if not isinstance(blob, dict) else parts["armature·q̈"],
            "armature_model_qacc_nm": None if not isinstance(blob, dict) else blob["armature_model_qacc_nm"],
            "contact_nm": None if not isinstance(blob, dict) else parts["impact/contact"],
            "link_inertia_nm": None if not isinstance(blob, dict) else parts["link inertia"],
            "gravity_nm": None if not isinstance(blob, dict) else parts["gravity"],
            "passive_nm": None if not isinstance(blob, dict) else blob["passive_nm"],
            "ident_residual_nm": None if not isinstance(blob, dict) else blob["ident_residual_nm"],
            "qacc_rad_s2": None if not isinstance(blob, dict) else blob["qacc_rad_s2"],
            "qacc_source": QACC_SOURCE,
            "actuator_nm": None if not isinstance(blob, dict) else blob["actuator_nm"],
            "substep": None if not isinstance(blob, dict) else blob["substep"],
            "cop_x_mm": None if not isinstance(blob, dict) else blob["cop_x_mm"],
            "cop_y_mm": None if not isinstance(blob, dict) else blob["cop_y_mm"],
            "phase": phase,
            "clock_phase": rec.phase,
            "clock_stance": rec.stance,
            "preview_stage": rec.stage,
            "swing_frac": rec.swing_frac,
            "hand_estimate_nm": hand,
            "hand_estimate_label": HAND_LABEL,
            "hand_m_kg": HAND_M_KG,
            "hand_L_m": HAND_L_M,
            "class": kind,
            "dominant_term": term,
        })
    ask_events = [row for row in events if row["over_on"] in ("ask", "both")]
    n_ask_over = len(ask_events)
    n_qdes_over = sum(1 for rec in writes if abs(rec.signed_nm) > ASK_BAR + 1e-9)
    ask_times = {round(float(row["t_s"]), 6) for row in ask_events}
    counts = {name: 0 for name in ("controller fail", "physics candidate", "unmeasured")}
    term_counts = {name: 0 for name in TERMS}
    for row in ask_events:
        counts[str(row["class"])] = counts.get(str(row["class"]), 0) + 1
        if row["class"] == "physics candidate" and row["dominant_term"]:
            term_counts[str(row["dominant_term"])] += 1
    worst = None
    if ask_events:
        measured_events = [row for row in ask_events if isinstance(row.get("id_nm"), float)]
        pool = measured_events or ask_events
        worst_row = max(pool, key=lambda row: abs(float(row["id_nm"] or 0.0)))
        worst = {
            "joint": worst_row["joint"],
            "t_s": worst_row["t_s"],
            "phase": worst_row["phase"],
            "class": worst_row["class"],
            "dominant_term": worst_row["dominant_term"],
            "id_nm": worst_row["id_nm"],
            "ask_nm": worst_row["ask_nm"],
        }
    def _frac(count: int) -> float | None:
        if n_ask_over <= 0:
            return None
        return count / n_ask_over

    qdes_peak = max(writes, key=lambda rec: abs(rec.signed_nm)) if writes else None
    ask_peak = max(writes, key=lambda rec: abs(rec.ask_nm)) if writes else None
    sum_peak = max(writes, key=lambda rec: rec.sum_nm) if writes else None
    verdict = "CLEAR" if not fail_reasons else "Prefer FAIL"
    gait = "STEPS" if int(stepping["n_scored_swings"]) > 0 and float(stepping["step_fraction"]) >= 0.50 else "SKATES"
    body_mean = (body_vx_sum / body_vx_n) if body_vx_n else None
    body_ratio = (body_mean / vx_cmd) if body_mean is not None and abs(vx_cmd) > 1e-9 else None
    stop = stepping.get("stop") if isinstance(stepping.get("stop"), dict) else {}
    ss = stepping.get("ss") if isinstance(stepping.get("ss"), dict) else {}
    row = {
        "T_s": float(period_s),
        "vx_m_s": float(vx),
        "x_amp_m": abs(float(vx)) * float(period_s) / 4.0,
        "preview_amp_m": float(amp),
        "stand_s": stand_s,
        "walk_s": walk_s,
        "stop_s": stop_s,
        "n_ticks": n,
        "soft_pass": False,
        "plant_md5_before": digest_before,
        "plant_md5_after": digest_after,
        "verdict": verdict,
        "gait": gait,
        "fail_reasons": fail_reasons,
        "signed_pass": bool(torque_bar["passes"]),
        "sum_pass": bool(torque_bar["sum_passes"]),
        "signed_sum_note": torque_bar["note"],
        "torque_bar": torque_bar,
        "clamp": clamp,
        "dc": {k: v for k, v in dc.items() if k != "corners"},
        "qvel": qvel_report,
        "speed_ratio": speed,
        "body_forward_mean_m_s": body_mean,
        "body_forward_ratio": body_ratio,
        "declared_zmp_min_m": zmp_min,
        "declared_zmp_outside": zmp_frac,
        "declared_com_min_m": com_min,
        "declared_com_outside": com_frac,
        "com_jerk_peak": None if com_jerk is None else com_jerk.peak_l2,
        "com_jerk_rms": None if com_jerk is None else com_jerk.rms_l2,
        "joint_jerk_peak": None if joint_jerk is None else joint_jerk.peak_l2,
        "joint_jerk_rms": None if joint_jerk is None else joint_jerk.rms_l2,
        "step_fraction": stepping.get("step_fraction"),
        "worst_slip_m": stepping.get("worst_slip_m"),
        "min_clear_m": stepping.get("min_clear_m"),
        "airborne_min_m": stepping.get("airborne_min_m"),
        "ss_min_contacts": ss.get("min_contacts"),
        "ss_n": ss.get("n"),
        "stop_pitch_rad": stop.get("pitch_rad") if isinstance(stop, dict) else None,
        "stop_ok": bool(stepping.get("passes")) and not any("stop" in str(item) or "upright" in str(item) or "pitch" in str(item) for item in (stepping.get("fail_reasons") or [])),
        "qdes_peak_nm": None if qdes_peak is None else abs(qdes_peak.signed_nm),
        "qdes_peak_signed_nm": None if qdes_peak is None else qdes_peak.signed_nm,
        "qdes_peak_joint": None if qdes_peak is None else qdes_peak.joint,
        "qdes_peak_t_s": None if qdes_peak is None else qdes_peak.t_s,
        "ask_peak_nm": None if ask_peak is None else abs(ask_peak.ask_nm),
        "ask_peak_joint": None if ask_peak is None else ask_peak.joint,
        "ask_peak_t_s": None if ask_peak is None else ask_peak.t_s,
        "sum_peak_nm": None if sum_peak is None else sum_peak.sum_nm,
        "sum_peak_joint": None if sum_peak is None else sum_peak.joint,
        "sum_peak_t_s": None if sum_peak is None else sum_peak.t_s,
        "over_writes_qdes": n_qdes_over,
        "over_writes_ask": n_ask_over,
        "over_ticks_ask": len(ask_times),
        "class_counts": counts,
        "class_fraction": {name: _frac(counts[name]) for name in counts},
        "physics_term_counts": term_counts,
        "physics_term_fraction": {
            name: (term_counts[name] / n_ask_over) if n_ask_over else None for name in TERMS
        },
        "worst_id": worst,
        "ident_residual_max_nm": max_resid,
        "min_up_z": float(session.min_up_z),
        "fault": list(reasons),
        "over_bar": events,
    }
    # Drop the bulky stop_ok guess if it disagrees with the stepping reasons.
    # The stepping fail_reasons already name the stop. Keep stop fields from summarize.
    row["stop"] = stop
    row["stepping_passes"] = bool(stepping.get("passes"))
    row["n_scored_swings"] = stepping.get("n_scored_swings")
    print(
        f"T {period_s:.2f} vx {vx:.3f} {verdict} {gait} "
        f"ask {row['ask_peak_nm']:.3f} {row['ask_peak_joint']} "
        f"qdes {row['qdes_peak_nm']:.3f} {row['qdes_peak_joint']} "
        f"over ask/qdes {n_ask_over}/{n_qdes_over} "
        f"note {torque_bar['note'] or '-'} resid {max_resid:.3e} "
        f"class {counts}",
        flush=True,
    )
    return row


def _parse_cells(text: str) -> list[tuple[float, float]]:
    cells: list[tuple[float, float]] = []
    for chunk in text.split(","):
        if not chunk.strip():
            continue
        period_s, vx = chunk.split(":")
        cells.append((float(period_s), float(vx)))
    return cells


def main() -> None:
    parser = argparse.ArgumentParser(description="Score a5a9183 on the #102 bars plus inverse dynamics")
    parser.add_argument("--cells", default="", help="Optional T:vx pairs, comma separated. Empty runs the 54.")
    parser.add_argument("--out", default="", help="JSON path. Empty prints only.")
    args = parser.parse_args()
    if sws.SOFT_PASS:
        raise SystemExit("soft-pass is on")
    if _plant_md5() != PLANT_MD5:
        raise SystemExit(f"plant md5 {_plant_md5()} != {PLANT_MD5}")
    cells = _parse_cells(args.cells) if args.cells else [
        (period, vx) for period in PERIODS for vx in SPEEDS
    ]
    cache: dict[float, float] = {}
    rows = []
    for period_s, vx in cells:
        rows.append(score_cell(period_s, vx, _amp(period_s, cache)))
        if _plant_md5() != PLANT_MD5:
            raise SystemExit("plant md5 changed")
    payload = {
        "tip": TIP_SHA,
        "soft_pass": False,
        "plant_md5": PLANT_MD5,
        "runtime_only": True,
        "qacc_source": QACC_SOURCE,
        "ask_bar_nm": ASK_BAR,
        "ask_definition": "|kp*(ctrl-q) - kv*qvel|",
        "qdes_definition": "kp*(q_des-q) - kv*qvel",
        "kv": "-actuator_biasprm[i,2]",
        "sum_definition": "|kp*(q_des-q)| + |kv*qvel|",
        "hand_estimate": {
            "label": HAND_LABEL,
            "formula": "m*g*L*sin(abs(q)/2)",
            "m_kg": HAND_M_KG,
            "L_m": HAND_L_M,
            "g": G,
        },
        "id_split": {
            "id": "qfrc_inverse on the realised q, qvel, qacc with contacts",
            "armature_qacc": "0.01 * qacc",
            "impact_contact": "-qfrc_constraint",
            "link_inertia": "(M*qacc - dof_armature*qacc) + (qfrc_bias(q,v) - qfrc_bias(q,0))",
            "gravity": "qfrc_bias at qvel 0",
            "passive_not_a_class": "-qfrc_passive",
            "class_rule": "ID abs <= 2.33 and |ask| > 2.33 is controller fail; ID abs > 2.33 is a physics candidate",
        },
        "preview_amp_m": {f"{key:.2f}": value for key, value in sorted(cache.items())},
        "rows": rows,
    }
    if args.out:
        path = Path(args.out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(_jsonable(payload), indent=2) + "\n", encoding="utf-8")
        print(f"wrote {path}", flush=True)


if __name__ == "__main__":
    main()
