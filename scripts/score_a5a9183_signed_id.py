#!/usr/bin/env python3
"""Independent score of Controls' cadence tip a5a9183 on the #102 bar set.

The gait and the plant are loaded from that tip. This file does not edit
either. Soft-pass stays off. q̈ is ``data.qacc`` from the ``mj_step`` that
applied this ctrl, not a finite difference of qvel.

The 2.33 Nm pass uses the ctrl the plant applies. When ``actuator_ctrllimited``
is set, that ctrl is ``data.ctrl`` clipped to ``ctrlrange`` (±2.09). The
control-tick sample is the max |ask| across that tick's four 2 ms substeps.
A tick with ``|ctrl_raw| > 2.09`` fails the clip bar. ``ctrl_raw`` is the
writer command before the ctrlrange clip when that command reproduces the
stored ctrl; otherwise the clip fraction is ``raw ctrl unavailable``.

Inverse dynamics runs on every substep whose plant |ask| exceeds 2.33, at
the q and q̇ that entered that ``mj_step``, with the plant-clipped ctrl and
that substep's ``data.qacc``.
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
CTRL_ABS = 2.09
RAW_UNAVAILABLE = "raw ctrl unavailable"
G = 9.81
HAND_M_KG = 2.2
HAND_L_M = 0.093
HAND_LABEL = "hand estimate"
QACC_SOURCE = (
    "data.qacc from the mj_step that applied this ctrl; not a finite difference of qvel"
)
PERIODS = (0.80, 0.75, 0.70, 0.65, 0.60, 0.55, 0.50, 1.00, 1.20)
SPEEDS = (0.016, 0.024, 0.032, 0.040, 0.048, 0.056)
TERMS = ("armature·q̈", "impact/contact", "link inertia", "gravity")
REMAINING = ("impact/contact", "link inertia", "gravity")
CLASS_NAMES = (
    "controller fail",
    "unsourced-armature candidate",
    "physics candidate",
)
CLASS_RULE = (
    "Comparisons use the absolute value of the signed torque. "
    "Exactly 2.33 stays on the low side of each greater-than test. "
    "controller fail: |qfrc_inverse| <= 2.33 and |ask| > 2.33. "
    "unsourced-armature candidate: |qfrc_inverse| > 2.33 and "
    "|qfrc_inverse - 0.01*qacc| <= 2.33. Armature 0.01 has no Hiwonder source; "
    "this bucket is not a physics candidate. "
    "physics candidate: |qfrc_inverse - 0.01*qacc| > 2.33, named by the largest "
    "of impact/contact, link inertia, and gravity."
)

_ORIG_MJ_STEP = mj.mj_step
_CAPTURE: list[dict[str, np.ndarray]] = []
_CAPTURE_ON = False


def _capturing_step(model: mj.MjModel, data: mj.MjData) -> None:
    if _CAPTURE_ON:
        _CAPTURE.append({
            "time": float(data.time),
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
    ctrl_raw: float | None
    raw_ok: bool
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


@dataclass
class PlantOver:
    t_s: float
    joint: str
    ask_nm: float
    kp_e: float
    kv_qdot: float
    q: float
    omega: float
    ctrl_raw: float
    ctrl_plant: float
    clipped: bool
    d2: float | None
    tick: int
    id_blob: dict[str, object] | None = None


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
        lo = float(model.actuator_ctrlrange[idx, 0])
        hi = float(model.actuator_ctrlrange[idx, 1])
        limited = int(model.actuator_ctrllimited[idx]) != 0
        match = (
            abs(act[0] + 2.45) < 1e-9 and abs(act[1] - 2.45) < 1e-9
            and abs(jnt[0] + 2.45) < 1e-9 and abs(jnt[1] - 2.45) < 1e-9
        )
        range_ok = abs(lo + CTRL_ABS) < 1e-9 and abs(hi - CTRL_ABS) < 1e-9
        ok = ok and match
        rows.append({
            "joint": name,
            "actuator_forcerange": act,
            "joint_actfrcrange": jnt,
            "dof_armature": arm,
            "pm_2_45": match,
            "ctrllimited": limited,
            "ctrlrange": [lo, hi],
            "ctrlrange_pm_2_09": range_ok,
        })
    return {
        "n": len(rows),
        "all_pm_2_45": ok,
        "ctrllimited_all": all(bool(row["ctrllimited"]) for row in rows),
        "ctrlrange_all_pm_2_09": all(bool(row["ctrlrange_pm_2_09"]) for row in rows),
        "joints": rows,
    }


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


def _range_clip(model: mj.MjModel, idx: int, cmd: float) -> float:
    lo = float(model.actuator_ctrlrange[idx, 0])
    hi = float(model.actuator_ctrlrange[idx, 1])
    return min(hi, max(lo, float(cmd)))


def _plant_ctrl(model: mj.MjModel, idx: int, raw: float) -> float:
    """Ctrl MuJoCo uses for force when ``ctrllimited`` is set."""
    if int(model.actuator_ctrllimited[idx]) == 0:
        return float(raw)
    return _range_clip(model, idx, raw)


def _raw_outside(value: float) -> bool:
    return abs(float(value)) > CTRL_ABS + 1e-9


def _leg_actuators(model: mj.MjModel, walker: sws.lipm_gait.LipmWalker) -> list[dict[str, object]]:
    rows = []
    for name in sws.LEG_JOINTS:
        idx = walker.act_idx.get(name + "_pos")
        if idx is None:
            continue
        jid = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name))
        rows.append({
            "joint": name,
            "idx": int(idx),
            "dof": int(model.jnt_dofadr[jid]),
            "qadr": int(model.jnt_qposadr[jid]),
            "kp": float(model.actuator_gainprm[idx, 0]),
            "kv": -float(model.actuator_biasprm[idx, 2]),
        })
    return rows


def _install(walker: sws.lipm_gait.LipmWalker, bucket: list[Write], asks: list[sws.AskSample], tick_box: list[int]) -> None:
    def _clipped_cmd(jn: str, q_des: float) -> float | None:
        act = f"{jn}_pos"
        if walker.act_idx.get(act) is None:
            return None
        if jn.endswith(("knee", "hip_pitch", "ank_pitch")):
            return float(q_des)
        q = float(walker.q(jn))
        band = float(walker.e_sat.get(act, 0.0))
        return min(q + band, max(q - band, float(q_des)))

    def _limited_cmd(jn: str, q_des: float, limit_nm: float | None) -> float | None:
        act = f"{jn}_pos"
        idx = walker.act_idx.get(act)
        if idx is None:
            return None
        q = float(walker.q(jn))
        jid = int(mj.mj_name2id(walker.model, mj.mjtObj.mjOBJ_JOINT, jn))
        omega = float(walker.data.qvel[int(walker.model.jnt_dofadr[jid])])
        kp = float(walker.model.actuator_gainprm[idx, 0])
        kv = -float(walker.model.actuator_biasprm[idx, 2])
        tau = abs(float(walker.model.actuator_forcerange[idx, 1]))
        limit = float(sws.lipm_gait.SAT_FRAC) * tau
        if limit_nm is not None:
            limit = min(limit, float(limit_nm))
        if kp < 1e-6:
            return float(q_des)
        e_des = float(q_des) - q
        e_lo = (-limit + kv * omega) / kp
        e_hi = (limit + kv * omega) / kp
        if e_lo > e_hi:
            e_lo, e_hi = e_hi, e_lo
        return q + min(e_hi, max(e_lo, e_des))

    def _log(jn: str, q_des: float, raw: float | None) -> None:
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
        raw_ok = raw is not None and abs(_range_clip(walker.model, int(idx), raw) - ctrl) <= 1e-8
        kp_e = kp * (float(q_des) - q)
        kv_qdot = kv * omega
        signed = kp_e - kv_qdot
        # Goal command after the writer's own range clip. The torque pass uses
        # the ctrl at mj_step, which can be the slewed value rather than this.
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
            ctrl_raw=None if not raw_ok else float(raw),
            raw_ok=bool(raw_ok),
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
        raw = _clipped_cmd(jn, q_des)
        orig(jn, q_des)
        _log(jn, q_des, raw)

    walker.write_clipped = wrapped  # type: ignore[method-assign]
    limited = walker.write_force_limited

    def wrapped_limited(jn: str, q_des: float, limit_nm: float | None = None) -> None:
        raw = _limited_cmd(jn, q_des, limit_nm)
        limited(jn, q_des, limit_nm)
        _log(jn, q_des, raw)

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


def _remaining_dominant(parts: dict[str, float]) -> str:
    return max(REMAINING, key=lambda name: abs(parts[name]))


def _worst_tick(events: list[dict[str, object]], kind: str) -> dict[str, object] | None:
    """Largest excess inside one bucket.

    Controller fail ranks on ``|ask|``. The unsourced-armature bucket ranks
    on ``|armature·q̈|``. A physics candidate ranks on ``|ID − armature·q̈|``.
    """
    pool = [row for row in events if row.get("class") == kind and isinstance(row.get("id_nm"), float)]
    if not pool:
        return None

    def _key(row: dict[str, object]) -> tuple[float, float, float]:
        ask = abs(float(row["ask_nm"]))
        ident = abs(float(row["id_nm"]))
        arm = abs(float(row["armature_qacc_nm"]))
        rest = abs(float(row["id_nm"]) - float(row["armature_qacc_nm"]))
        if kind == "controller fail":
            return (ask, ident, arm)
        if kind == "unsourced-armature candidate":
            return (arm, ident, ask)
        return (rest, ident, ask)

    row = max(pool, key=_key)
    ident = float(row["id_nm"])
    arm = float(row["armature_qacc_nm"])
    return {
        "joint": row["joint"],
        "t_s": row["t_s"],
        "phase": row["phase"],
        "class": kind,
        "dominant_term": row["dominant_term"],
        "ask_nm": row["ask_nm"],
        "id_nm": ident,
        "armature_qacc_nm": arm,
        "id_minus_armature_nm": ident - arm,
        "contact_nm": row["contact_nm"],
        "link_inertia_nm": row["link_inertia_nm"],
        "gravity_nm": row["gravity_nm"],
    }


def _classify(ask_abs: float, id_nm: float, armature_nm: float, parts: dict[str, float]) -> tuple[str, str]:
    """Three buckets on the absolute value of the signed torques.

    Armature stays 0.01. An unsourced-armature candidate is the case where
    removing that term brings ``|qfrc_inverse|`` back to 2.33 or under.
    """
    if ask_abs <= ASK_BAR + 1e-9:
        return "under bar", ""
    if abs(id_nm) <= ASK_BAR + 1e-9:
        return "controller fail", ""
    if abs(id_nm - armature_nm) <= ASK_BAR + 1e-9:
        return "unsourced-armature candidate", "armature·q̈"
    return "physics candidate", _remaining_dominant(parts)


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
    plant_asks: list[sws.AskSample] = []
    plant_overs: list[PlantOver] = []
    tick_box = [0]
    _install(walker, writes, asks, tick_box)
    legs = _leg_actuators(session.model, walker)
    ctrl_hist: dict[str, list[float]] = {name: [] for name in sws.LEG_JOINTS}
    raw_unavailable = False
    clip_ticks = 0
    clip_substeps = 0
    plant_substeps = 0
    max_step_abs = 0.0
    max_writer_abs = 0.0
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
        signed_tick, sum_tick, qvel_tick, qvel_sum_tick, _, _ = sws._tick_pair(
            new_asks, sws.LEG_JOINTS,
        )
        plant_new: list[sws.AskSample] = []
        tick_clipped = False
        over_from = len(plant_overs)
        over_shots: dict[int, list[tuple[dict[str, object], float]]] = {}
        for si, shot in enumerate(shots):
            raw_ctrl = shot["ctrl"]
            qpos = shot["qpos"]
            qvel = shot["qvel"]
            t_step = float(shot["time"])
            for leg in legs:
                idx = int(leg["idx"])
                raw = float(raw_ctrl[idx])
                used = _plant_ctrl(session.model, idx, raw)
                q = float(qpos[int(leg["qadr"])])
                omega = float(qvel[int(leg["dof"])])
                kp = float(leg["kp"])
                kv = float(leg["kv"])
                kp_e = kp * (used - q)
                kv_qdot = kv * omega
                ask = kp_e - kv_qdot
                joint = str(leg["joint"])
                clipped = _raw_outside(raw)
                max_step_abs = max(max_step_abs, abs(raw))
                plant_substeps += 1
                if clipped:
                    tick_clipped = True
                    clip_substeps += 1
                plant_new.append(sws.AskSample(
                    t_s=t_step,
                    joint=joint,
                    signed_nm=float(ask),
                    sum_nm=float(abs(kp_e) + abs(kv_qdot)),
                    ask_nm=float(ask),
                    stage=stage_name,
                    qvel_abs=abs(omega),
                ))
                history = ctrl_hist[joint]
                history.append(used)
                if abs(ask) > ASK_BAR + 1e-9:
                    d2 = None
                    if len(history) >= 3:
                        d2 = history[-1] - 2.0 * history[-2] + history[-3]
                    over = PlantOver(
                        t_s=t_step,
                        joint=joint,
                        ask_nm=float(ask),
                        kp_e=float(kp_e),
                        kv_qdot=float(kv_qdot),
                        q=q,
                        omega=omega,
                        ctrl_raw=raw,
                        ctrl_plant=float(used),
                        clipped=clipped,
                        d2=d2,
                        tick=int(tick_box[0]),
                    )
                    plant_overs.append(over)
                    over_shots.setdefault(si, []).append((leg, ask))
                    setattr(over, "_shot", si)
        for rec in new_writes:
            if not rec.raw_ok or rec.ctrl_raw is None:
                raw_unavailable = True
            elif _raw_outside(rec.ctrl_raw):
                tick_clipped = True
            if rec.raw_ok and rec.ctrl_raw is not None:
                max_writer_abs = max(max_writer_abs, abs(float(rec.ctrl_raw)))
        if tick_clipped:
            clip_ticks += 1
        _, _, _, _, ask_tick, qvel_ask_tick = sws._tick_pair(plant_new, sws.LEG_JOINTS)
        plant_asks.extend(plant_new)
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
        if over_shots:
            split_cache: dict[int, dict[str, np.ndarray]] = {}
            cop_cache: dict[tuple[int, str], tuple[float | None, float | None]] = {}
            for over in plant_overs[over_from:]:
                si = getattr(over, "_shot", None)
                if not isinstance(si, int) or si not in over_shots:
                    continue
                if si not in split_cache:
                    shot = dict(shots[si])
                    clipped_ctrl = np.array(shot["ctrl"], dtype=np.float64, copy=True)
                    for leg in legs:
                        idx = int(leg["idx"])
                        clipped_ctrl[idx] = _plant_ctrl(session.model, idx, float(clipped_ctrl[idx]))
                    shot["ctrl"] = clipped_ctrl
                    split_cache[si] = _inverse_split(session.model, scratch, grav_data, shot)
                split = split_cache[si]
                adr = dof[over.joint]
                scratch.qpos[:] = shots[si]["qpos"]
                scratch.qvel[:] = shots[si]["qvel"]
                scratch.qacc[:] = split["qacc"]
                mj.mj_fwdPosition(session.model, scratch)
                side = "l" if over.joint.startswith("l_") else "r"
                key = (si, side)
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
                over.id_blob = {
                    "id_nm": id_nm,
                    "id_abs_nm": abs(id_nm),
                    "parts": parts,
                    "passive_nm": float(split["passive"][adr]),
                    "ident_nm": ident,
                    "ident_residual_nm": id_nm - ident,
                    "armature_model_qacc_nm": float(split["arm_model"][adr]),
                    "qacc_rad_s2": float(split["qacc"][adr]),
                    "actuator_nm": float(split["actuator"][adr]),
                    "substep": si,
                    "n_substeps": len(shots),
                    "cop_x_mm": cop_x,
                    "cop_y_mm": cop_y,
                }
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
    line_t, line_tau, line_w, line_stage = sws._writes_for_line(plant_asks, sws.LEG_JOINTS)
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
    if raw_unavailable:
        clip_fraction: float | str = RAW_UNAVAILABLE
        fail_reasons.append(RAW_UNAVAILABLE)
    else:
        clip_fraction = (clip_ticks / n) if n else 0.0
        if clip_ticks:
            fail_reasons.append(
                f"ctrl clipped to ±{CTRL_ABS:.2f} on {clip_ticks}/{n} control ticks"
            )
    if not audit["ctrllimited_all"]:
        fail_reasons.append("ctrllimited is off on a leg actuator")
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
    write_at: dict[tuple[int, str], Write] = {}
    for rec in writes:
        write_at[(rec.tick, rec.joint)] = rec
    events: list[dict[str, object]] = []
    max_resid = 0.0
    for rec in plant_overs:
        blob = rec.id_blob
        src = write_at.get((rec.tick, rec.joint))
        tick_row = contact_at.get(rec.tick, {})
        phase = _phase_label(
            str(tick_row.get("mode", "")),
            float(tick_row.get("t", rec.t_s)),
            touchdowns,
            int(tick_row.get("n_l", 0)),
            int(tick_row.get("n_r", 0)),
            "" if src is None else src.swing_side,
            None if src is None else src.swing_frac,
        )
        hand = HAND_M_KG * G * HAND_L_M * math.sin(abs(rec.q) / 2.0)
        if not isinstance(blob, dict):
            kind, term = ("unmeasured", "")
            parts = {name: float("nan") for name in TERMS}
        else:
            parts = {name: float(blob["parts"][name]) for name in TERMS}
            kind, term = _classify(
                abs(rec.ask_nm), float(blob["id_nm"]), parts["armature·q̈"], parts,
            )
            resid = abs(float(blob["ident_residual_nm"]))
            max_resid = max(max_resid, resid)
        events.append({
            "t_s": rec.t_s,
            "t_post_s": tick_row.get("t"),
            "joint": rec.joint,
            "over_on": "ask",
            "ask_nm": rec.ask_nm,
            "qdes_signed_nm": None if src is None else src.signed_nm,
            "sum_nm": None if src is None else src.sum_nm,
            "kp_e_nm": rec.kp_e,
            "kv_qdot_nm": rec.kv_qdot,
            "kp_e_qdes_nm": None if src is None else src.kp * (src.q_des - src.q),
            "q_rad": rec.q,
            "q_des_rad": None if src is None else src.q_des,
            "qvel_rad_s": rec.omega,
            "ctrl_raw_rad": rec.ctrl_raw,
            "ctrl_plant_rad": rec.ctrl_plant,
            "ctrl_clipped": rec.clipped,
            "ctrl_rad": rec.ctrl_plant,
            "ctrl_second_diff_rad": rec.d2,
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
            "clock_phase": "" if src is None else src.phase,
            "clock_stance": "" if src is None else src.stance,
            "preview_stage": "" if src is None else src.stage,
            "swing_frac": None if src is None else src.swing_frac,
            "hand_estimate_nm": hand,
            "hand_estimate_label": HAND_LABEL,
            "hand_m_kg": HAND_M_KG,
            "hand_L_m": HAND_L_M,
            "class": kind,
            "dominant_term": term,
        })
    ask_events = events
    n_ask_over = len(ask_events)
    n_qdes_over = sum(1 for rec in writes if abs(rec.signed_nm) > ASK_BAR + 1e-9)
    ask_times = {round(float(row["t_s"]), 6) for row in ask_events}
    counts = {name: 0 for name in CLASS_NAMES}
    term_counts = {name: 0 for name in REMAINING}
    for row in ask_events:
        counts[str(row["class"])] = counts.get(str(row["class"]), 0) + 1
        if row["class"] == "physics candidate" and row["dominant_term"] in term_counts:
            term_counts[str(row["dominant_term"])] += 1
    bucket_worst = {name: _worst_tick(ask_events, name) for name in CLASS_NAMES}
    worst = bucket_worst["unsourced-armature candidate"] or bucket_worst["physics candidate"] or bucket_worst["controller fail"]
    def _frac(count: int) -> float | None:
        if n_ask_over <= 0:
            return None
        return count / n_ask_over

    qdes_peak = max(writes, key=lambda rec: abs(rec.signed_nm)) if writes else None
    goal_peak = max(writes, key=lambda rec: abs(rec.ask_nm)) if writes else None
    ask_peak = max(plant_asks, key=lambda rec: abs(rec.ask_nm)) if plant_asks else None
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
        "goal_ask_peak_nm": None if goal_peak is None else abs(goal_peak.ask_nm),
        "goal_ask_peak_joint": None if goal_peak is None else goal_peak.joint,
        "goal_ask_peak_t_s": None if goal_peak is None else goal_peak.t_s,
        "ctrl_clip_fraction": clip_fraction,
        "ctrl_clip_ticks": clip_ticks,
        "ctrl_clip_substeps": clip_substeps,
        "max_abs_ctrl_at_step": max_step_abs,
        "max_abs_writer_raw": None if raw_unavailable else max_writer_abs,
        "plant_substeps": plant_substeps,
        "ctrllimited_all": bool(audit["ctrllimited_all"]),
        "ctrlrange_all_pm_2_09": bool(audit["ctrlrange_all_pm_2_09"]),
        "raw_ctrl": RAW_UNAVAILABLE if raw_unavailable else "writer cmd before ctrlrange clip, and data.ctrl entering mj_step",
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
            name: (term_counts[name] / n_ask_over) if n_ask_over else None for name in REMAINING
        },
        "bucket_worst": bucket_worst,
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
        f"goal {row['goal_ask_peak_nm']:.3f} "
        f"clip {clip_fraction} "
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
        "ask_definition": "|kp*(ctrl_plant-q) - kv*qvel|, ctrl_plant = clip(data.ctrl, ctrlrange) when actuator_ctrllimited",
        "ctrl_clip": "|ctrl_raw| > 2.09 on a control tick is a fail. ctrl_raw is the writer command before the ctrlrange clip when that command reproduces the stored ctrl, and data.ctrl entering mj_step.",
        "ctrllimited_leg": all(bool(row.get("ctrllimited_all")) for row in rows),
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
            "class_rule": CLASS_RULE,
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
