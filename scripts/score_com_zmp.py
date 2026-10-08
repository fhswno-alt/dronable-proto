#!/usr/bin/env python3
"""Score CoM and ZMP inside the stance sole, and the unclamped leg ask.

Stand, shift onto the first stance foot, walk, soft-stop. y_swap stays 0.
The lateral channel is the cart-table preview. Nothing here solves q_des
onto ±2.33 Nm, edits the plant, or raises a torque rail.

The historical 2.33 Nm bar is |kp*(q_des−q)| + |kv*ω| on the command
before the ctrlrange clip. The signed force is the ctrl MuJoCo applies
after that clip: kp*(clip(ctrl, ±2.09)−q) − kv*ω. kv is
−actuator_biasprm[i, 2] from dampratio=1, a different number on every
joint. Max |ctrl| and the clip fraction are logged per row against
ctrlrange ±2.09. ctrllimited is required on every leg actuator, so the
clip is the one the force uses. The historical mfg bar does not include
the clip fraction.
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
# A step is a foot that leaves the floor, moves forward in the air, and lands.
# Clearance is the lowest of the eight corners of the 135×76 mm contact
# box, in the geom frame. Not the box centre, not a toe sphere, and not
# the declared phase. Half-length 67.5 mm: 1° of sole pitch drops a toe
# or heel by about 1.2 mm. The bar is the minimum of that corner over
# 20–80% of the swing. Airborne is zero floor contacts and every corner
# above the floor plane.
STEP_CLEAR_M = 0.008
STEP_SLIP_M = 0.002
STEP_FRAC_MIN = 0.90
STEP_PITCH_DEG = 3.0
STEP_LEN_TOL = 0.20
STEP_SWING_LO = 0.20
STEP_SWING_HI = 0.80
# Single support is a flat stance sole: three or more floor contacts,
# and the swing foot airborne, over the same 20–80% window.
STANCE_CONTACT_N = 3
# Upright stop. Both soles on the floor, corner spread under 3 mm.
FLAT_SPREAD_M = 0.003
FLAT_SOLE_Z_M = 0.003
# Feet at double support, each side of the pelvis, for the 3.60 s example
# (vx·T/2 = 0.10 m, so each foot is vx·T/4 = 0.05 m from the pelvis).
DS_FOOT_M = 0.050
FOOT_HALF_X_M = 0.0675
COP_P5_MM = 5.0
AIR_N = 1.0
# HX-35H no-load speed. The plant has no velocity rail, so this is a score.
QVEL_LIM = 5.82
# DC-motor model, not datasheet. Stall 3.43 Nm, no-load 5.82 rad/s.
# Same-tick bar: |qvel| <= 5.82 * (1 - |τ| / 3.43). τ is the pre-clamp
# signed force on that tick, not the conservative sum.
DC_STALL_NM = 3.43
DC_LABEL = "DC-motor model, not datasheet"
# Leg actuator forcerange and joint actuatorfrcrange are both ±2.45 Nm.
# A tick is clamp-active when the pre-clamp |signed| meets either rail.
CLAMP_NM = 2.45
# Position ctrlrange on every leg actuator. MuJoCo clips ctrl to this
# before computing the force. A commanded value outside it is a fail
# of the feedforward bar. The plant check raises if a leg differs.
CTRL_RANGE = 2.09
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


def preclamp_torque(
    kp: float, q_des: float, q: float, kv: float, omega: float,
) -> tuple[float, float]:
    """Pre-clamp torque from one command, before any position or force clip.

    ``kv`` is ``-actuator_biasprm[i, 2]``. dampratio=1 compiles a
    different kv on every joint. It is not a shared constant.

    The signed force is ``kp·(ctrl−q) − kv·ω`` on the ctrl passed in.
    The logger passes the command after the ±2.09 clip, which is the
    ctrl MuJoCo uses when ctrllimited is set. The conservative bar is
    ``|kp·(q_des−q)| + |kv·ω|`` on the pre-clip command. The historical
    2.33 Nm figure is the conservative one.
    """
    kp_term = float(kp) * (float(q_des) - float(q))
    signed = kp_term - float(kv) * float(omega)
    conservative = abs(kp_term) + abs(float(kv) * float(omega))
    return float(signed), float(conservative)


# Printed components from the ask log at those commits. The published
# peak is sum_nm. signed_nm on the same tick is the other formula.
# kv is the hip-roll value of -actuator_biasprm[:, 2], printed to 4 decimals.
# d6e8b5e rounded kv to 1.703 and omega to 0.588 in the diary; the
# reconstruction from those rounded digits sits 0.0003 Nm above 2.2567.
_HISTORICAL_ASKS: tuple[tuple[str, float, float | None, float, float, float, float, float], ...] = (
    # commit, published sum, signed on that tick, q, q_des, kp, kv, omega.
    # d6e8b5e printed the components in the diary and not the signed field.
    # ac81435 and 58ce1d8 store both in ask_worst.
    ("d6e8b5e", 2.2567, None, -0.11738, -0.08599, 40.0, 1.703, 0.588),
    ("ac81435", 2.3135, 0.2607, -0.00845, 0.02372, 40.0, 1.7027, 0.6028),
    ("58ce1d8", 1.9792, 0.2486, -0.14153, -0.11368, 40.0, 1.7027, 0.5082),
)


def historical_torque_identity(model: mj.MjModel | None = None) -> list[dict[str, float | str]]:
    """Which formula produced the three published peaks.

    All three commits log in ``_install_ask_log``. ``ask_nm`` is
    ``sum_nm``, the conservative ``|kp·e|+|kv·ω|``. ``kv`` is
    ``-actuator_biasprm[i, 2]`` with dampratio=1, so hip roll is 1.7027
    and the knee is 1.4573. The signed force on those ticks is about
    +0.25 Nm and was not the published number.

    ac81435 and 58ce1d8 store the full line in ``ask_worst``
    (``previews/com_zmp_preview_s.json`` and ``_j.json``). d6e8b5e's
    2.2567 is the same field, with the components in the diary.
    """
    rows: list[dict[str, float | str]] = []
    for commit, published, signed_pub, q, q_des, kp, kv, omega in _HISTORICAL_ASKS:
        signed, conservative = preclamp_torque(kp, q_des, q, kv, omega)
        if abs(conservative - published) > 5e-4:
            raise RuntimeError(
                f"{commit} conservative {conservative:.4f} != published {published:.4f}"
            )
        if signed_pub is not None and abs(signed - signed_pub) > 5e-3:
            raise RuntimeError(
                f"{commit} signed {signed:.4f} != stored {signed_pub:.4f}"
            )
        compared = signed if signed_pub is None else signed_pub
        if abs(published - compared) < 1.0:
            raise RuntimeError(f"{commit} published peak matches the signed force")
        rows.append({
            "commit": commit,
            "formula": "|kp*(q_des-q)|+|kv*omega|",
            "kv_source": "-actuator_biasprm[i,2]",
            "published": published,
            "conservative": conservative,
            "signed": signed,
        })
    if model is not None:
        for jn, expect in (("r_hip_roll", 1.7027), ("r_knee", 1.4573)):
            aid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, jn + "_pos")
            kv = -float(model.actuator_biasprm[aid, 2])
            if abs(kv - expect) > 5e-4:
                raise RuntimeError(f"{jn} kv {kv:.4f} != {expect:.4f}")
            if abs(kv - 1.7027) < 1e-4 and jn == "r_knee":
                raise RuntimeError("knee kv collapsed onto the hip-roll constant")
    return rows


def _log_preclamp(
    lipm: lipm_gait.LipmWalker, bucket: list[AskRow], jn: str, q_des: float,
) -> None:
    """Record the pre-clip command and the signed ask of the applied ctrl.

    ``q_des`` is the command before the ctrlrange clip, so a clip stays
    visible. The signed force uses the ctrl after that clip. ctrllimited
    on the actuator is what makes MuJoCo apply the same clip before the
    force. The conservative sum stays on the pre-clip command.
    """
    if jn not in LEG_JOINTS:
        return
    idx = lipm.act_idx.get(jn + "_pos")
    if idx is None:
        return
    q = float(lipm.q(jn))
    jid = mj.mj_name2id(lipm.model, mj.mjtObj.mjOBJ_JOINT, jn)
    omega = float(lipm.data.qvel[int(lipm.model.jnt_dofadr[jid])])
    kp = float(lipm.model.actuator_gainprm[idx, 0])
    kv = -float(lipm.model.actuator_biasprm[idx, 2])
    lo = float(lipm.model.actuator_ctrlrange[idx, 0])
    hi = float(lipm.model.actuator_ctrlrange[idx, 1])
    applied = min(hi, max(lo, float(q_des)))
    signed, _ = preclamp_torque(kp, applied, q, kv, omega)
    _, total = preclamp_torque(kp, float(q_des), q, kv, omega)
    bucket.append(AskRow(
        t=float(lipm.data.time),
        joint=jn,
        q_des=float(q_des),
        q=q,
        omega=omega,
        kp=kp,
        kv=kv,
        signed_nm=signed,
        sum_nm=total,
        stage=str(lipm.preview_stage),
    ))


def _install_ask_log(lipm: lipm_gait.LipmWalker, bucket: list[AskRow]) -> None:
    orig = lipm.write_clipped
    orig_lim = lipm.write_force_limited

    def wrapped(jn: str, q_des: float) -> None:
        _log_preclamp(lipm, bucket, jn, q_des)
        orig(jn, q_des)

    def wrapped_lim(jn: str, q_des: float, limit_nm: float | None = None) -> None:
        _log_preclamp(lipm, bucket, jn, q_des)
        orig_lim(jn, q_des, limit_nm)

    lipm.write_clipped = wrapped  # type: ignore[method-assign]
    lipm.write_force_limited = wrapped_lim  # type: ignore[method-assign]


def _dc_speed_limit(tau_nm: float) -> float:
    """Allowed |qvel| on the DC-motor model line. Not a datasheet."""
    return QVEL_LIM * (1.0 - abs(float(tau_nm)) / DC_STALL_NM)


def _clamp_limits(lipm: lipm_gait.LipmWalker) -> dict[str, tuple[float, float]]:
    """Actuator forcerange and joint actuatorfrcrange, both absolute."""
    out: dict[str, tuple[float, float]] = {}
    for jn in LEG_JOINTS:
        idx = lipm.act_idx.get(jn + "_pos")
        jid = mj.mj_name2id(lipm.model, mj.mjtObj.mjOBJ_JOINT, jn)
        if idx is None or jid < 0:
            raise RuntimeError(f"missing actuator or joint {jn}")
        act = abs(float(lipm.model.actuator_forcerange[idx, 1]))
        jnt = abs(float(lipm.model.jnt_actfrcrange[jid, 1]))
        out[jn] = (act, jnt)
    return out


def _ctrl_limits(lipm: lipm_gait.LipmWalker) -> dict[str, tuple[float, float]]:
    """Leg ctrlrange. Raises when a joint is not ±CTRL_RANGE."""
    out: dict[str, tuple[float, float]] = {}
    for jn in LEG_JOINTS:
        idx = lipm.act_idx.get(jn + "_pos")
        if idx is None:
            raise RuntimeError(f"missing actuator {jn}")
        lo = float(lipm.model.actuator_ctrlrange[idx, 0])
        hi = float(lipm.model.actuator_ctrlrange[idx, 1])
        if abs(lo + CTRL_RANGE) > 1e-9 or abs(hi - CTRL_RANGE) > 1e-9:
            raise RuntimeError(f"{jn} ctrlrange {lo} {hi} != ±{CTRL_RANGE}")
        if int(lipm.model.actuator_ctrllimited[idx]) == 0:
            raise RuntimeError(f"{jn} actuator_ctrllimited is off")
        out[jn] = (lo, hi)
    return out


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
            grounds = {floor}
            if session.lipm is not None:
                grounds.update(int(g) for g in session.lipm.ground_extra if int(g) >= 0)
            if not ((g1 == gid and g2 in grounds) or (g2 == gid and g1 in grounds)):
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


def _note_declared(stats: dict[str, SideFrac], label: str, mm: float | None, outside: bool) -> None:
    """Declared-phase polygon. A missing distance still counts as outside."""
    for key in ("all", label):
        row = stats.get(key)
        if row is None:
            continue
        row.n += 1
        if outside:
            row.n_out += 1
        if mm is not None and math.isfinite(mm) and mm < row.min_mm:
            row.min_mm = mm


def _declared_polygon(
    session: sw.SteerSession,
) -> tuple[str, float | None, float | None, bool, bool]:
    """Contact CoP and CoM against the walker's declared stance box.

    Single support is that foot only. A loaded swing foot is not added.
    Double support and stand use both boxes. Margin is millimetres,
    positive inside. Outside is a negative margin, no floor contact, or a
    declared stance foot under 5 N.
    """
    lipm = session.lipm
    if lipm is None:
        return "unknown", None, None, True, True
    phase = str(lipm.phase)
    stance = str(lipm.stance)
    if phase == "swing" and stance == "L":
        label, sides = "ss_L", ("L",)
    elif phase == "swing" and stance == "R":
        label, sides = "ss_R", ("R",)
    elif phase == "shift":
        label, sides = "ds", ("L", "R")
    elif phase == "stand":
        label, sides = "stand", ("L", "R")
    else:
        return "unknown", None, None, True, True
    num = np.zeros(3, dtype=np.float64)
    den = 0.0
    for side in ("L", "R"):
        gid = int(lipm.gid[side])
        for i in range(session.data.ncon):
            con = session.data.contact[i]
            g1 = int(con.geom1)
            g2 = int(con.geom2)
            if not lipm._on_ground(g1, g2, gid):
                continue
            force = np.zeros(6, dtype=np.float64)
            mj.mj_contactForce(session.model, session.data, i, force)
            fn = float(force[0])
            if fn <= 1e-6:
                continue
            num += fn * np.asarray(con.pos, dtype=np.float64)
            den += fn
    chunks = [session._foot_corners(lipm.bid[side], lipm.gid[side]) for side in sides]
    hull = sw.convex_hull_xy(np.concatenate(chunks, axis=0))
    com = np.asarray(session.data.subtree_com[lipm.bid_body, :2], dtype=np.float64)
    com_mm = 1000.0 * sw.support_margin(com, hull)
    cop_mm: float | None = None
    margins: list[float] = []
    if den > 1e-6:
        cop_mm = 1000.0 * sw.support_margin((num / den)[:2], hull)
        margins.append(cop_mm)
    unload = float(lipm.cfg.unload_n)
    stance_unloaded = False
    box_missing = False
    for side in ("L", "R"):
        fn = float(lipm.foot_normal(side))
        if fn < unload:
            if side in sides and label.startswith("ss"):
                stance_unloaded = True
            continue
        slack = float(lipm.cop_margin(side))
        if slack <= -0.5:
            box_missing = True
        else:
            margins.append(1000.0 * slack)
    if margins:
        cop_mm = min(margins)
    cop_out = den <= 1e-6 or stance_unloaded or box_missing or cop_mm is None or cop_mm < 0.0
    com_out = com_mm < 0.0
    return label, cop_mm, com_mm, cop_out, com_out


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


SEED_Q_STD = 0.002
SEED_QD_STD = 0.01
RUG_EDGE_AHEAD_M = 0.015


@dataclass
class Perturb:
    """Runtime-only. The plant file is not written."""

    label: str = "nominal"
    seed: int | None = None
    mass_scale: float = 1.0
    friction: float | None = None
    latency_ticks: int = 0
    rug: bool = False
    cycles: int = 0


def _scale_mass(model: mj.MjModel, data: mj.MjData, scale: float) -> None:
    model.body_mass[1:] *= float(scale)
    model.body_inertia[1:] *= float(scale)
    mj.mj_setConst(model, data)


def _set_sliding_friction(model: mj.MjModel, mu: float) -> None:
    for name in ("floor", "l_foot_contact", "r_foot_contact"):
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        if gid < 0:
            raise RuntimeError(f"missing geom {name}")
        model.geom_friction[gid, 0] = float(mu)


def _seed_hinges(session: sw.SteerSession, seed: int) -> None:
    """Hinge q and qd noise. The free joint is left on the seated stand."""
    rng = np.random.default_rng(int(seed))
    model = session.model
    data = session.data
    for jid in range(model.njnt):
        if int(model.jnt_type[jid]) != int(mj.mjtJoint.mjJNT_HINGE):
            continue
        qadr = int(model.jnt_qposadr[jid])
        vadr = int(model.jnt_dofadr[jid])
        q = float(data.qpos[qadr]) + float(rng.normal(0.0, SEED_Q_STD))
        lo = float(model.jnt_range[jid, 0])
        hi = float(model.jnt_range[jid, 1])
        data.qpos[qadr] = min(hi, max(lo, q))
        data.qvel[vadr] = float(data.qvel[vadr]) + float(rng.normal(0.0, SEED_QD_STD))
    mj.mj_forward(model, data)


def _place_entrance_rug(session: sw.SteerSession) -> dict[str, float]:
    """Slide the entrance mat so its near edge is just ahead of the toes.

    The authored height stays. The geom is ground for load and for the
    airborne check. The plant file is not written.
    """
    gid = int(session.gid_rug)
    if gid < 0:
        raise RuntimeError("entrance rug geom missing")
    bid = int(session.model.geom_bodyid[gid])
    half_x = float(session.model.geom_size[gid, 0])
    half_z = float(session.model.geom_size[gid, 2])
    front = -1.0e9
    for body, geom in (
        (session.bid_lf, session.gid_lfoot),
        (session.bid_rf, session.gid_rfoot),
    ):
        corners = session._foot_corners(body, geom)
        front = max(front, float(np.max(corners[:, 0])))
    near = front + RUG_EDGE_AHEAD_M
    session.model.body_pos[bid, 0] = near + half_x
    mj.mj_forward(session.model, session.data)
    if session.lipm is not None:
        session.lipm.ground_extra = (gid,)
    top = float(session.data.geom_xpos[gid, 2]) + half_z
    return {"near_x_m": near, "top_z_m": top, "half_x_m": half_x}


def _install_latency(session: sw.SteerSession, ticks: int) -> None:
    """+1 holds the previous ctrl for one planner tick. -1 leads by one tick."""
    if ticks not in (-1, 1):
        raise ValueError(f"latency ticks {ticks}")
    orig = session._lipm_substep
    prev: list[np.ndarray | None] = [None]

    def wrapped(ctrl_from: np.ndarray | None = None) -> None:
        commanded = np.array(session.data.ctrl, dtype=np.float64, copy=True)
        if prev[0] is None:
            applied = commanded
        elif ticks > 0:
            applied = prev[0]
        else:
            applied = commanded + (commanded - prev[0])
        prev[0] = commanded.copy()
        session.data.ctrl[:] = applied
        orig(ctrl_from)

    session._lipm_substep = wrapped  # type: ignore[method-assign]


def _cycle_script(
    n_cycles: int,
    stand_s: float = 0.40,
    walk_s: float = 11.0,
    stop_s: float = 3.0,
) -> tuple[sw.DemoSegment, ...]:
    """Repeat the cleared vel window. Each stop is the same 11 s phase."""
    t = float(stand_s)
    segments = [sw.DemoSegment(t, "stand", 0.0, 0.0, "stand")]
    for i in range(int(n_cycles)):
        t += float(walk_s)
        segments.append(sw.DemoSegment(t, "vel", sw.VOICE_VX_M_S, 0.0, f"forward{i}"))
        t += float(stop_s)
        segments.append(sw.DemoSegment(t, "stop", 0.0, 0.0, f"stop{i}"))
    return tuple(segments)


def _rug_normal(session: sw.SteerSession) -> float:
    gid = int(session.gid_rug)
    if gid < 0:
        return 0.0
    total = 0.0
    for i in range(session.data.ncon):
        con = session.data.contact[i]
        if int(con.geom1) != gid and int(con.geom2) != gid:
            continue
        force = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(session.model, session.data, i, force)
        total += float(force[0])
    return total


def _apply_perturb(session: sw.SteerSession, perturb: Perturb) -> dict[str, object]:
    info: dict[str, object] = {"label": perturb.label}
    if abs(float(perturb.mass_scale) - 1.0) > 1e-12:
        _scale_mass(session.model, session.data, float(perturb.mass_scale))
        session._reset_stand()
        info["mass_scale"] = float(perturb.mass_scale)
        info["mass_kg"] = float(np.sum(session.model.body_mass[1:]))
    if perturb.friction is not None:
        _set_sliding_friction(session.model, float(perturb.friction))
        info["friction"] = float(perturb.friction)
    if perturb.rug:
        info["rug"] = _place_entrance_rug(session)
    if perturb.seed is not None:
        _seed_hinges(session, int(perturb.seed))
        info["seed"] = int(perturb.seed)
        info["q_std_rad"] = SEED_Q_STD
        info["qd_std_rad_s"] = SEED_QD_STD
    if perturb.latency_ticks:
        _install_latency(session, int(perturb.latency_ticks))
        info["latency_ticks"] = int(perturb.latency_ticks)
    if perturb.cycles:
        info["cycles"] = int(perturb.cycles)
    return info


def _sole_spread_pitch(session: sw.SteerSession, side: str) -> tuple[float, float]:
    """Corner spread (m) and sole pitch (deg). Positive pitch is nose-down."""
    lipm = session.lipm
    if lipm is None:
        return 0.0, 0.0
    gid = int(lipm.gid[side])
    bid = int(lipm.bid[side])
    pos = np.asarray(session.model.geom_pos[gid], dtype=np.float64)
    half = np.asarray(session.model.geom_size[gid], dtype=np.float64)
    rot = np.asarray(session.data.xmat[bid], dtype=np.float64).reshape(3, 3)
    origin = np.asarray(session.data.xpos[bid], dtype=np.float64)
    zs: list[float] = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            local = np.array(
                [pos[0] + sx * half[0], pos[1] + sy * half[1], pos[2] - half[2]],
                dtype=np.float64,
            )
            zs.append(float((origin + rot @ local)[2]))
    pitch = math.degrees(
        math.atan2(-float(rot[2, 0]), math.hypot(float(rot[0, 0]), float(rot[1, 0])))
    )
    return max(zs) - min(zs), pitch


def _n_ground(session: sw.SteerSession, side: str) -> int:
    lipm = session.lipm
    if lipm is None:
        return 0
    gid = int(lipm.gid[side])
    grounds = {int(lipm.gid_floor)}
    for g in lipm.ground_extra:
        if int(g) >= 0:
            grounds.add(int(g))
    n = 0
    for i in range(session.data.ncon):
        c = session.data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        if g1 != gid and g2 != gid:
            continue
        other = g2 if g1 == gid else g1
        if other in grounds:
            n += 1
    return n


def _trunk_pitch(session: sw.SteerSession) -> float:
    lipm = session.lipm
    if lipm is None:
        return 0.0
    R = np.asarray(session.data.xmat[int(lipm.bid_body)], dtype=np.float64).reshape(3, 3)
    return math.atan2(-float(R[2, 0]), math.hypot(float(R[0, 0]), float(R[1, 0])))


def _within(value: float, target: float) -> bool:
    return abs(value - target) <= STEP_LEN_TOL * abs(target)


def _airborne(row: dict[str, float | str], side: str) -> bool:
    """Zero floor contacts and every contact-box corner above z=0.

    The declared phase is not an input. A foot the clock calls swinging
    stays down when this is false.
    """
    return int(row[side + "n"]) == 0 and float(row[side + "z"]) > 0.0


def _swing_window_z(iv: list[dict[str, float | str]], side: str) -> tuple[float, float]:
    """Min and max of the lowest sole corner over 20–80% of the swing."""
    t0 = float(iv[0]["t"])
    t1 = float(iv[-1]["t"])
    span = t1 - t0
    lo = t0 + STEP_SWING_LO * span
    hi = t0 + STEP_SWING_HI * span
    zs = [
        float(r[side + "z"])
        for r in iv
        if lo - 1e-12 <= float(r["t"]) <= hi + 1e-12
    ]
    if not zs:
        zs = [float(r[side + "z"]) for r in iv]
    return min(zs), max(zs)


def _step_honesty(
    rows: list[dict[str, float | str]],
    x_amp: float,
    vx: float,
    period_s: float,
) -> dict[str, object]:
    """Per swing: clearance, slip, step length, and world airborne advance.

    ``period`` is one full L+R cycle. ``update_time`` puts the left
    single support in the first half and the right single support in
    the second, and the clock wraps at ``period``. One step is T/2.

    Step length is the fore/aft world placement of the landing foot
    relative to the stance foot, and the bar is vx·T/2. World airborne
    advance is the swing foot's world-x change while it is in the air,
    and the bar is the stride vx·T. At double support the two feet sit
    at ±vx·T/4 from the pelvis. All three are ±20%.

    Clearance is the lowest of the eight corners of the 135×76 mm
    contact box, minimum over 20–80% of the swing. A completed step
    lands, is airborne for more than 10 ticks, and keeps that corner at
    least 8 mm up through the window. Airborne is zero floor contacts
    and every corner above the plane. The declared phase is not an
    input. A truncated swing that never lands is not scored. Zero
    completed steps fails.
    """
    step_cmd = abs(float(vx)) * float(period_s) / 2.0
    stride_cmd = abs(float(vx)) * float(period_s)
    ds_cmd = step_cmd / 2.0

    def intervals(side: str) -> list[list[dict[str, float | str]]]:
        out: list[list[dict[str, float | str]]] = []
        cur: list[dict[str, float | str]] = []
        for row in rows:
            swinging = row["phase"] == "swing" and row["stance"] != side and row["stage"] in ("walk", "stop")
            if swinging:
                cur.append(row)
            elif cur:
                out.append(cur)
                cur = []
        if cur:
            out.append(cur)
        return out

    swings: list[dict[str, float | str | bool]] = []
    sep_air = 0.0
    for side in ("L", "R"):
        other = "R" if side == "L" else "L"
        for iv in intervals(side):
            if len(iv) < 20:
                continue
            clear_z, peak_z = _swing_window_z(iv, side)
            air = 0.0
            con = 0.0
            off = 0.0
            on = 0.0
            n_air = 0
            n_off = 0
            air_x0 = 0.0
            air_x1 = 0.0
            for a, b in zip(iv, iv[1:]):
                d = float(b[side + "x"]) - float(a[side + "x"])
                airborne = _airborne(b, side)
                contact_off = int(b[side + "n"]) == 0
                if contact_off:
                    n_off += 1
                if airborne:
                    if n_air == 0:
                        air_x0 = float(b[side + "x"])
                    n_air += 1
                    air_x1 = float(b[side + "x"])
                    sep_air = max(sep_air, abs(float(b["lx"]) - float(b["rx"])))
                if d > 0.0:
                    if airborne:
                        air += d
                    else:
                        con += d
                    if contact_off:
                        off += d
                    else:
                        on += d
            landed = int(iv[-1][side + "n"]) > 0 or float(iv[-1][side + "fn"]) >= AIR_N
            if not landed:
                continue
            air_adv = (air_x1 - air_x0) if n_air > 0 else 0.0
            place = float(iv[-1][side + "x"]) - float(iv[-1][other + "x"])
            slip = math.hypot(
                float(iv[-1][other + "x"]) - float(iv[0][other + "x"]),
                float(iv[-1][other + "y"]) - float(iv[0][other + "y"]),
            )
            frac = air / (air + con) if (air + con) > 1e-6 else 0.0
            # Contact-off window: forward travel while the swing foot has
            # zero floor contacts. Clearance is not an input. The AI
            # tiebreak scores the step on this window.
            off_frac = off / (off + on) if (off + on) > 1e-6 else 0.0
            off_time = n_off / max(1, len(iv) - 1)
            done = n_air > 10 and clear_z >= STEP_CLEAR_M
            t0 = float(iv[0]["t"])
            t1 = float(iv[-1]["t"])
            span = t1 - t0
            lo = t0 + STEP_SWING_LO * span
            hi = t0 + STEP_SWING_HI * span
            window = [r for r in iv if lo - 1e-12 <= float(r["t"]) <= hi + 1e-12]
            n_win = 0
            n_mis = 0
            stance_n_min = 99.0
            for r in window:
                n_win += 1
                swing_up = _airborne(r, side)
                stance_n = float(r[other + "n"])
                stance_n_min = min(stance_n_min, stance_n)
                stance_down = float(r[other + "fn"]) >= LOAD_N and stance_n + 1e-9 >= STANCE_CONTACT_N
                if not swing_up or not stance_down:
                    n_mis += 1
            bouts: list[list[dict[str, float | str]]] = []
            cur_air: list[dict[str, float | str]] = []
            for r in iv:
                if _airborne(r, side):
                    cur_air.append(r)
                elif cur_air:
                    bouts.append(cur_air)
                    cur_air = []
            if cur_air:
                bouts.append(cur_air)
            bout = max(bouts, key=len) if bouts else []
            t_lift = float(bout[0]["t"]) if bout else float("nan")
            t_down = float("nan")
            if bout:
                after = [r for r in iv if float(r["t"]) > float(bout[-1]["t"]) + 1e-12]
                touched = [r for r in after if int(r[side + "n"]) > 0 or float(r[side + "fn"]) >= AIR_N]
                t_down = float(touched[0]["t"]) if touched else float(bout[-1]["t"])
            air_peak = max((float(r[side + "z"]) for r in bout), default=float("nan"))
            air_dx = (
                float(bout[-1][side + "x"]) - float(bout[0][side + "x"])
                if len(bout) >= 2 else 0.0
            )
            swings.append({
                "side": side,
                "t0": float(iv[0]["t"]),
                "t1": float(iv[-1]["t"]),
                "t_lift": t_lift,
                "t_down": t_down,
                "air_peak_mm": air_peak * 1000.0 if math.isfinite(air_peak) else float("nan"),
                "air_dx_mm": air_dx * 1000.0,
                "done": done,
                "n_air": float(n_air),
                "clear_mm": clear_z * 1000.0,
                "peak_mm": peak_z * 1000.0,
                "slip_mm": slip * 1000.0,
                "frac": frac,
                "off_frac": off_frac,
                "off_time": off_time,
                "air_mm": air_adv * 1000.0,
                "place_mm": place * 1000.0,
                "fore_mm": float(iv[-1][side + "fore"]) * 1000.0,
                "fore_other_mm": float(iv[-1][other + "fore"]) * 1000.0,
                "crouch_mm": float(iv[-1][side + "cz"]) * 1000.0,
                "crouch_other_mm": float(iv[-1][other + "cz"]) * 1000.0,
                "air_fwd_mm": air * 1000.0,
                "con_mm": con * 1000.0,
                "net_mm": (float(iv[-1][side + "x"]) - float(iv[0][side + "x"])) * 1000.0,
                "bx0": float(iv[0]["bx"]),
                "bx1": float(iv[-1]["bx"]),
                "n_win": float(n_win),
                "n_mis": float(n_mis),
                "stance_n": stance_n_min if n_win else 0.0,
            })
    done_rows = [s for s in swings if bool(s["done"])]
    scored = done_rows if done_rows else swings
    stand = [float(r["pitch"]) for r in rows if r["stage"] == "stand" and float(r["t"]) > 0.20]
    pitch0 = float(np.median(stand)) if stand else (float(rows[0]["pitch"]) if rows else 0.0)
    pitch1 = float(rows[-1]["pitch"]) if rows else pitch0
    v = 0.0
    if done_rows:
        ordered = sorted(done_rows, key=lambda s: float(s["t0"]))
        dt = float(ordered[-1]["t1"]) - float(ordered[0]["t0"])
        if dt > 1e-6:
            v = (float(ordered[-1]["bx1"]) - float(ordered[0]["bx0"])) / dt
    n = len(done_rows)
    n_win = sum(int(s["n_win"]) for s in swings)
    n_mis = sum(int(s["n_mis"]) for s in swings)
    stance_ns = [float(s["stance_n"]) for s in swings if int(s["n_win"]) > 0]
    end = rows[-1] if rows else None
    flat = False
    flat_spread = float("nan")
    flat_pitch = float("nan")
    if end is not None and "Lsp" in end and "Rsp" in end:
        spreads = (float(end["Lsp"]), float(end["Rsp"]))
        pitches = (float(end["Lpit"]), float(end["Rpit"]))
        zs = (float(end["Lz"]), float(end["Rz"]))
        flat_spread = max(spreads)
        flat_pitch = max(abs(p) for p in pitches)
        flat = (
            flat_spread <= FLAT_SPREAD_M
            and all(-0.004 <= z <= FLAT_SOLE_Z_M for z in zs)
            and flat_pitch <= 1.5
        )
    airs = [float(s["air_mm"]) for s in done_rows]
    air_report = airs if airs else [float(s["air_mm"]) for s in swings]
    place_ok = bool(swings) and step_cmd > 1e-6 and all(
        _within(float(s["place_mm"]) / 1000.0, step_cmd) for s in swings
    )
    stride_ok = bool(swings) and stride_cmd > 1e-6 and all(
        _within(float(s["air_mm"]) / 1000.0, stride_cmd) for s in swings
    )
    ds_ok = bool(swings) and ds_cmd > 1e-6 and all(
        _within(float(s["fore_mm"]) / 1000.0, ds_cmd)
        and _within(float(s["fore_other_mm"]) / 1000.0, -ds_cmd)
        for s in swings
    )
    return {
        "n_steps": float(n),
        "n_swings": float(len(swings)),
        "clear_mm": (min(float(s["clear_mm"]) for s in scored)) if scored else float("nan"),
        "clear_max_mm": (max(float(s["peak_mm"]) for s in scored)) if scored else float("nan"),
        "slip_mm": (max(float(s["slip_mm"]) for s in scored)) if scored else float("nan"),
        "slip_sum_mm": (sum(float(s["slip_mm"]) for s in scored)) if scored else float("nan"),
        "sep_mm": sep_air * 1000.0,
        "step_frac": (min(float(s["frac"]) for s in scored)) if scored else float("nan"),
        "off_frac": (min(float(s["off_frac"]) for s in scored)) if scored else float("nan"),
        "off_time": (min(float(s["off_time"]) for s in scored)) if scored else float("nan"),
        "adv_mm": (min(float(s["net_mm"]) for s in scored)) if scored else float("nan"),
        "air_mm": (min(air_report)) if air_report else float("nan"),
        "place_mm": (min(float(s["place_mm"]) for s in swings)) if swings else float("nan"),
        "cmd_mm": step_cmd * 1000.0,
        "stride_mm": stride_cmd * 1000.0,
        "ds_mm": ds_cmd * 1000.0,
        "place_ok": place_ok,
        "stride_ok": stride_ok,
        "ds_ok": ds_ok,
        "len_ok": place_ok and stride_ok and ds_ok,
        "x_amp_mm": abs(x_amp) * 1000.0,
        "pitch_deg": math.degrees(pitch1 - pitch0),
        "v_m_s": v,
        "steps": swings,
        "phase_mis": (n_mis / n_win) if n_win else 1.0,
        "stance_n": min(stance_ns) if stance_ns else 0.0,
        "flat": flat,
        "flat_spread_mm": flat_spread * 1000.0 if math.isfinite(flat_spread) else float("nan"),
        "flat_pitch_deg": flat_pitch,
    }


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
    perturb: Perturb | None = None,
    crouch_m: float = 0.025,
    hip_pitch_deg: float = 15.0,
    gait_name: str = "preview",
    z_quintic: bool = False,
    z_lead: bool = False,
    spring_nm: float = 0.0,
    honor_vx: bool = False,
    id_ff: bool = False,
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
            name=gait_name,
            clear_m=z_m,
            arms=True,
            schedule="gait_manager",
            gm_period_s=period_s,
            gm_dsp=dsp,
            gm_y_swap_m=0.0,
            gm_x_m=abs(float(vx_m_s)) * float(period_s) / 4.0,
            gm_z_m=z_m,
            gm_z_swap_m=0.0,
            gm_pelvis_deg=0.0,
            gm_hip_pitch_deg=hip_pitch_deg,
            gm_start_lead="L",
            gm_crouch_m=crouch_m,
            gm_move_s=0.020,
            preview_amp_m=amp_m,
            preview_arm_s=arm_s,
            preview_r=preview_r,
            preview_shape=preview_shape,
            gm_z_quintic=bool(z_quintic),
            gm_z_lead=bool(z_lead),
            gm_spring_nm=float(spring_nm),
            gm_id_ff=bool(id_ff),
        )
    scene = None
    if perturb is not None and perturb.rug:
        scene = sw.ROOT / "mujoco" / "room_entrance.xml"
    session = sw.SteerSession(video=False, lipm=cfg, scene_xml=scene)
    perturb_info: dict[str, object] = {}
    if perturb is not None:
        perturb_info = _apply_perturb(session, perturb)
    lipm = session.lipm
    if lipm is None or lipm.op3 is None:
        raise SystemExit("walker did not build")
    if not kit_baseline and lipm.op3.y_swap_cmd != 0.0:
        raise SystemExit("y_swap_cmd is not 0")
    if kit_baseline:
        driver = sw.ScriptedDriver(sw.BUS_KIT_SCRIPT)
    elif perturb is not None and perturb.cycles:
        driver = sw.ScriptedDriver(_cycle_script(int(perturb.cycles)))
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
    decl_cop_stats = _empty_fracs()
    decl_com_stats = _empty_fracs()
    decl_cop_mm: list[float] = []
    decl_ss_mm: list[float] = []
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
    if perturb is not None and perturb.cycles:
        t_end = float(_cycle_script(int(perturb.cycles))[-1].t_end)
    last_send = -1.0
    stop_sent = False
    fault = ""
    dt = float(session.ctrl_dt)
    x0 = float(session.data.qpos[0])
    preview_peak = 0.0
    diag: list[str] = []
    worst_diag = ""
    worst_com = float("inf")
    rug_peak = 0.0
    foot_rows: list[dict[str, float | str]] = []
    x_amp_seen = 0.0
    qvel_adr = {
        jn: int(session.model.jnt_dofadr[mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_JOINT, jn)])
        for jn in LEG_JOINTS
    }
    qvel_peak = 0.0
    qvel_joint = ""
    qvel_t = 0.0
    clamp_lim = _clamp_limits(lipm)
    clamp_n = {jn: 0 for jn in LEG_JOINTS}
    clamp_hit = {jn: 0 for jn in LEG_JOINTS}
    ctrl_lim = _ctrl_limits(lipm)
    ctrl_abs = {jn: 0.0 for jn in LEG_JOINTS}
    ctrl_clip = {jn: 0 for jn in LEG_JOINTS}
    # A large negative sentinel so a row that stays inside the line
    # still records a joint. -1 would ignore every tick more than 1 rad/s
    # inside the line and then fail the bar for an empty joint.
    dc_excess = -1.0e9
    dc_joint = ""
    signed_over = 0
    sum_over = 0
    id_wall_n = 0
    id_arm_n = 0
    id_ctrl_n = 0
    id_skip_n = 0
    id_in_n = 0
    id_over_signed = 0
    stop_up_z = float("nan")
    stop_up_n = 0
    dc_tau = 0.0
    dc_spd = 0.0
    dc_lim = 0.0
    dc_t = 0.0
    vx_sum = 0.0
    vx_n = 0
    bind_abs = -1.0
    bind_joint = ""
    bind_phase = ""
    bind_term = ""
    bind_tau = 0.0
    bind_t = 0.0
    bind_resid = float("nan")
    bind_inside = False
    historical_torque_identity(session.model)

    def _body_id(name: str) -> int:
        bid = int(mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_BODY, name))
        if bid < 0:
            raise SystemExit(f"missing body {name}")
        return bid

    bid_pelvis = _body_id("body_link")
    bid_hip = {"L": _body_id("l_hip_pitch_link"), "R": _body_id("r_hip_pitch_link")}
    bid_ank = {"L": _body_id("l_ank_pitch_link"), "R": _body_id("r_ank_pitch_link")}
    leg_m = float(lipm.op3.lengths.thigh_m + lipm.op3.lengths.calf_m)
    while float(session.data.time) < t_end - 1e-12:
        now = float(session.data.time)
        if driver is not None:
            driver.publish(session.bus, now)
        elif now + 1e-12 < stand_s:
            pass
        elif now + 1e-12 < t_stop:
            if last_send < 0.0 or (now - last_send) >= (sw.VEL_RESEND_S - 1e-12):
                session.bus.vel(vx_cmd, 0.0, now)
                # The stick deadband is 0.012 m/s. A frontier row below
                # that is a gait command, not a joystick zero.
                if honor_vx and abs(vx_cmd) >= 1e-6 and abs(session.bus.target_vx) < 1e-12:
                    session.bus.target_vx = float(vx_cmd)
                    session.bus._snapped = False
                    session.bus.mode = "move"
                    session.bus.last_cmd_time = now
                last_send = now
        elif not stop_sent:
            session.bus.stop(now)
            stop_sent = True
        n_ask = len(asks)
        session.step()
        if perturb is not None and perturb.rug:
            rug_peak = max(rug_peak, _rug_normal(session))
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
            clamp_n[row.joint] += 1
            act_lim, jnt_lim = clamp_lim[row.joint]
            # Either rail. On this plant both are 2.45 Nm.
            if abs(row.signed_nm) >= act_lim - 1e-9 or abs(row.signed_nm) >= jnt_lim - 1e-9:
                clamp_hit[row.joint] += 1
            # MFG pass is the signed force on every leg write. The
            # conservative sum stays logged and is not this bar.
            if abs(row.signed_nm) > bind_abs:
                bind_abs = abs(row.signed_nm)
                bind_joint = row.joint
                bind_t = float(row.t)
                bind_phase = lipm._ff_phase(row.joint)
                bind_term = str(lipm.id_tick_term.get(row.joint, ""))
                raw_tau = lipm.id_tick_tau.get(row.joint)
                bind_tau = float("nan") if raw_tau is None else float(raw_tau)
                bind_resid = float(lipm.id_tick_resid.get(row.joint, float("inf")))
                bind_inside = bool(lipm.id_tick_ok.get(row.joint, False)) and bind_resid <= lipm_gait.ID_RESID_BUCKET_NM
            if abs(row.signed_nm) > ASK_NM + 1e-9:
                signed_over += 1
            if row.sum_nm > ASK_NM + 1e-9:
                sum_over += 1
            # q_des is the command before write_clipped clips ctrlrange.
            # MuJoCo would clip the same range before the force.
            if abs(row.q_des) > ctrl_abs[row.joint]:
                ctrl_abs[row.joint] = abs(row.q_des)
            clo, chi = ctrl_lim[row.joint]
            if row.q_des < clo - 1e-9 or row.q_des > chi + 1e-9:
                ctrl_clip[row.joint] += 1
            # The residual chooses a bucket. It does not relax the
            # 2.33 Nm applied-ask bar, which is signed_over above.
            # 1e-3 Nm is a Prefer FAIL. The bucket gate is 1e-2 Nm.
            tau_id = lipm.id_tick_tau.get(row.joint)
            resid_id = float(lipm.id_tick_resid.get(row.joint, float("inf")))
            ok_id = bool(lipm.id_tick_ok.get(row.joint, False)) and tau_id is not None
            if abs(row.signed_nm) > ASK_NM + 1e-9:
                id_over_signed += 1
            if not ok_id or resid_id > lipm_gait.ID_RESID_BUCKET_NM:
                id_skip_n += 1
            else:
                stripped_id = float(lipm.id_tick_stripped.get(row.joint, float(tau_id)))
                if abs(stripped_id) > ASK_NM + 1e-9:
                    id_wall_n += 1
                elif abs(float(tau_id)) > ASK_NM + 1e-9:
                    id_arm_n += 1
                elif abs(row.signed_nm) > ASK_NM + 1e-9:
                    id_ctrl_n += 1
                else:
                    id_in_n += 1
            # τ and ω are this write. ω is the qvel the torque used,
            # before mj_step replaces it.
            tau = abs(row.signed_nm)
            spd = abs(row.omega)
            limit = _dc_speed_limit(tau)
            excess = spd - limit
            if excess > dc_excess:
                dc_excess = excess
                dc_joint = row.joint
                dc_tau = tau
                dc_spd = spd
                dc_lim = limit
                dc_t = row.t
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
        if not kit_baseline:
            d_label, d_cop_mm, d_com_mm, d_cop_out, d_com_out = _declared_polygon(session)
            _note_declared(decl_cop_stats, d_label, d_cop_mm, d_cop_out)
            _note_declared(decl_com_stats, d_label, d_com_mm, d_com_out)
            if d_cop_mm is not None and math.isfinite(d_cop_mm):
                decl_cop_mm.append(float(d_cop_mm))
                if d_label in ("ss_L", "ss_R"):
                    decl_ss_mm.append(float(d_cop_mm))
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
        for jn, adr in qvel_adr.items():
            spd = abs(float(session.data.qvel[adr]))
            if spd > qvel_peak:
                qvel_peak = spd
                qvel_joint = jn
                qvel_t = now
        if stand_s - 1e-12 <= now < t_stop - 1e-12:
            vx_sum += float(session._body_forward_speed())
            vx_n += 1
        if stage == "stop":
            stop_uz = float(session._up_z())
            stop_up_z = stop_uz if stop_up_n == 0 else min(stop_up_z, stop_uz)
            stop_up_n += 1
        preview_peak = max(preview_peak, abs(float(lipm.preview_com_y)))
        if not kit_baseline and lipm.op3 is not None:
            if stage == "walk":
                x_amp_seen = max(x_amp_seen, abs(float(lipm.op3.x_cmd)))
            rec: dict[str, float | str] = {
                "t": now,
                "stage": stage,
                "phase": str(lipm.phase),
                "stance": str(lipm.stance),
                "bx": float(session.data.qpos[0]),
                "pitch": _trunk_pitch(session),
                "lx": 0.0,
                "rx": 0.0,
            }
            for side in ("L", "R"):
                gid = int(lipm.gid[side])
                half_x = float(session.model.geom_size[gid, 0])
                if abs(half_x - FOOT_HALF_X_M) > 1e-4:
                    raise SystemExit(
                        f"{side} contact box half-length {half_x:.4f} m "
                        f"is not {FOOT_HALF_X_M:.4f} m"
                    )
                key = side.lower()
                rec[side + "x"] = float(session.data.geom_xpos[gid][0])
                rec[side + "y"] = float(session.data.geom_xpos[gid][1])
                rec[key + "x"] = rec[side + "x"]
                rec[side + "z"] = lipm_gait.sole_clearance(
                    session.model, session.data, lipm.bid[side], gid,
                )
                spread_m, sole_pitch = _sole_spread_pitch(session, side)
                rec[side + "sp"] = spread_m
                rec[side + "pit"] = sole_pitch
                rec[side + "fn"] = float(lipm.foot_normal(side))
                rec[side + "n"] = float(_n_ground(session, side))
                pel = np.asarray(session.data.xpos[bid_pelvis], dtype=np.float64)
                forward = np.asarray(session.data.xmat[bid_pelvis], dtype=np.float64).reshape(3, 3)[:, 0].copy()
                forward[2] = 0.0
                norm = float(np.linalg.norm(forward))
                if norm > 1e-9:
                    forward /= norm
                foot = np.asarray(session.data.geom_xpos[gid], dtype=np.float64)
                delta = foot - pel
                delta[2] = 0.0
                rec[side + "fore"] = float(np.dot(delta, forward))
                rec[side + "cz"] = float(
                    session.data.xpos[bid_hip[side]][2] - session.data.xpos[bid_ank[side]][2]
                )
            foot_rows.append(rec)
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
    if not kit_baseline and decl_cop_stats["all"].n > 0:
        cop_row = decl_cop_stats["all"]
        com_row = decl_com_stats["all"]
        com_min_m = float(com_row.min_mm) / 1000.0 if math.isfinite(com_row.min_mm) else -1.0
        zmp_min_m = float(cop_row.min_mm) / 1000.0 if math.isfinite(cop_row.min_mm) else -1.0
        com_frac = _frac_of(com_row)
        zmp_frac = _frac_of(cop_row)
        margins_ok = (
            cop_row.n_out == 0
            and com_row.n_out == 0
            and zmp_min_m >= 0.0
            and com_min_m >= 0.0
        )
    else:
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
    honesty = _step_honesty(foot_rows, x_amp_seen, vx_cmd, period_s)
    signed_all = max(asks, key=lambda row: abs(row.signed_nm)) if asks else None
    clamp_frac = {
        jn: (float(clamp_hit[jn]) / float(clamp_n[jn])) if clamp_n[jn] else float("nan")
        for jn in LEG_JOINTS
    }
    clamp_max = max(clamp_frac.values()) if clamp_frac else float("nan")
    clamp_joint = max(clamp_frac, key=lambda jn: clamp_frac[jn]) if clamp_frac else ""
    clamp_ok = all(clamp_n[jn] > 0 and clamp_hit[jn] == 0 for jn in LEG_JOINTS)
    dc_ok = dc_excess > -1.0e8 and dc_excess <= 1e-6 and dc_joint != ""
    # Signed bar: |kp*(applied ctrl−q) − kv*ω| <= 2.33. The applied
    # ctrl is q_des after the ±2.09 clip. Sum bar: the conservative
    # |kp*e|+|kv*ω| on the pre-clip command.
    signed_ok = bool(asks) and signed_over == 0
    sum_ok = bool(asks) and sum_over == 0
    signed_pass_sum_fail = bool(signed_ok and not sum_ok)
    mfg_ok = bool(signed_ok and clamp_ok and dc_ok)
    ctrl_clip_n = int(sum(ctrl_clip.values()))
    ctrl_abs_joint = max(ctrl_abs, key=lambda jn: ctrl_abs[jn]) if asks else ""
    ctrl_abs_max = float(ctrl_abs[ctrl_abs_joint]) if ctrl_abs_joint else 0.0
    ctrl_clip_frac = (
        float(ctrl_clip_n) / float(len(asks)) if asks else float("nan")
    )
    # Clip-free is part of the feedforward bar. It stays out of mfg_ok
    # so a historical row is not rewritten when its IK target sits
    # outside ctrlrange and write_clipped then clips it.
    ctrl_ok = bool(asks) and ctrl_clip_n == 0
    vx_mean = vx_sum / float(vx_n) if vx_n else float("nan")
    vx_ratio = vx_mean / vx_cmd if vx_n and abs(vx_cmd) > 1e-9 else float("nan")
    cop_p5 = float(np.percentile(decl_cop_mm, 5)) if decl_cop_mm else float("nan")
    if kit_baseline:
        step_ok = True
        cop_p5_ok = True
    else:
        step_ok = (
            honesty["n_steps"] >= 1.0
            and honesty["clear_mm"] >= STEP_CLEAR_M * 1000.0
            and honesty["slip_mm"] <= STEP_SLIP_M * 1000.0
            and honesty["step_frac"] >= STEP_FRAC_MIN
            and honesty["sep_mm"] + 1e-6 >= honesty["x_amp_mm"]
            and abs(float(honesty["pitch_deg"])) <= STEP_PITCH_DEG
            and bool(honesty["len_ok"])
            and float(honesty["phase_mis"]) <= 1e-9
            and float(honesty["stance_n"]) + 1e-9 >= STANCE_CONTACT_N
            and bool(honesty["flat"])
        )
        cop_p5_ok = math.isfinite(cop_p5) and cop_p5 >= COP_P5_MM
    cleared = (
        margins_ok
        and ask_ok
        and jerk_ok
        and tip_ok
        and step_ok
        and cop_p5_ok
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
        "step_ok": step_ok,
        "cop_p5_ok": cop_p5_ok,
        "n_steps": honesty["n_steps"],
        "clear_mm": honesty["clear_mm"],
        "clear_max_mm": honesty["clear_max_mm"],
        "slip_mm": honesty["slip_mm"],
        "slip_sum_mm": honesty["slip_sum_mm"],
        "sep_mm": honesty["sep_mm"],
        "step_frac": honesty["step_frac"],
        "off_frac": honesty["off_frac"],
        "off_time": honesty["off_time"],
        "qvel_peak": qvel_peak,
        "qvel_joint": qvel_joint,
        "qvel_t": qvel_t,
        "qvel_ok": qvel_peak <= QVEL_LIM + 1e-9,
        "ask_joint": "" if ask_all is None else str(ask_all.joint),
        "ask_t": 0.0 if ask_all is None else float(ask_all.t),
        "ask_formula": "|kp*(q_des-q)|+|kv*omega|",
        "ask_kv": "-actuator_biasprm[i,2]",
        "signed_nm": 0.0 if signed_all is None else abs(float(signed_all.signed_nm)),
        "signed_value": 0.0 if signed_all is None else float(signed_all.signed_nm),
        "signed_joint": "" if signed_all is None else str(signed_all.joint),
        "signed_t": 0.0 if signed_all is None else float(signed_all.t),
        "signed_sum": 0.0 if signed_all is None else float(signed_all.sum_nm),
        "ask_signed": 0.0 if ask_all is None else float(ask_all.signed_nm),
        "signed_formula": "kp*(clip(ctrl,±2.09)-q)-kv*omega",
        "signed_over": signed_over,
        "signed_ok": signed_ok,
        "sum_over": sum_over,
        "sum_ok": sum_ok,
        "signed_pass_sum_fail": signed_pass_sum_fail,
        "mfg_ok": mfg_ok,
        "ctrl_abs": ctrl_abs,
        "ctrl_abs_max": ctrl_abs_max,
        "ctrl_abs_joint": ctrl_abs_joint,
        "ctrl_clip": ctrl_clip,
        "ctrl_clip_n": ctrl_clip_n,
        "ctrl_clip_frac": ctrl_clip_frac,
        "ctrl_ok": ctrl_ok,
        "ctrl_range": CTRL_RANGE,
        "ctrl_limited": True,
        "ctrl_limited_n": len(ctrl_lim),
        "id_ff": bool(getattr(cfg, "gm_id_ff", False)),
        "id_ff_peak": float(getattr(lipm, "id_ff_peak_abs", 0.0)),
        "id_ff_tau": float(getattr(lipm, "id_ff_peak_tau", 0.0)),
        "id_ff_joint": str(getattr(lipm, "id_ff_peak_joint", "")),
        "id_ff_phase": str(getattr(lipm, "id_ff_peak_phase", "")),
        "id_ff_term": str(getattr(lipm, "id_ff_peak_term", "")),
        "id_ff_t": float(getattr(lipm, "id_ff_peak_t", 0.0)),
        "id_ff_over_n": int(getattr(lipm, "id_ff_over_n", 0)),
        "id_ff_wall_n": int(getattr(lipm, "id_ff_wall_n", 0)),
        "id_ff_arm_n": int(getattr(lipm, "id_ff_arm_n", 0)),
        "id_ff_bucket": str(getattr(lipm, "id_ff_bucket", "")),
        "id_ff_arm": float(getattr(lipm, "id_ff_arm", 0.0)),
        "id_ff_stripped": float(getattr(lipm, "id_ff_stripped", 0.0)),
        "id_ff_stripped_abs": float(getattr(lipm, "id_ff_stripped_abs", 0.0)),
        "id_ff_hold": float(getattr(lipm, "id_ff_hold_abs", 0.0)),
        "id_ff_hold_tau": float(getattr(lipm, "id_ff_hold_tau", 0.0)),
        "id_ff_hold_joint": str(getattr(lipm, "id_ff_hold_joint", "")),
        "id_ff_hold_phase": str(getattr(lipm, "id_ff_hold_phase", "")),
        "id_ff_hold_term": str(getattr(lipm, "id_ff_hold_term", "")),
        "id_ff_hold_t": float(getattr(lipm, "id_ff_hold_t", 0.0)),
        "id_ff_hold_over_n": int(getattr(lipm, "id_ff_hold_over_n", 0)),
        "id_ff_impossible": int(getattr(lipm, "id_ff_impossible", 0)),
        "id_ff_broke": int(getattr(lipm, "id_ff_broke", 0)),
        "id_root_max": float(getattr(lipm, "id_root_max", 0.0)),
        "id_root_t": float(getattr(lipm, "id_root_t", 0.0)),
        "id_root_dof": int(getattr(lipm, "id_root_dof", -1)),
        "id_root_fail_n": int(getattr(lipm, "id_root_fail_n", 0)),
        "id_root_bal_max": float(getattr(lipm, "id_root_bal_max", 0.0)),
        "id_ident_max": float(getattr(lipm, "id_ident_max", 0.0)),
        "id_inv_gap_max": float(getattr(lipm, "id_inv_gap_max", 0.0)),
        "id_inv_gap_joint": str(getattr(lipm, "id_inv_gap_joint", "")),
        "id_mj_abs": float(getattr(lipm, "id_mj_abs", 0.0)),
        "id_mj_tau": float(getattr(lipm, "id_mj_tau", 0.0)),
        "id_mj_joint": str(getattr(lipm, "id_mj_joint", "")),
        "id_mj_t": float(getattr(lipm, "id_mj_t", 0.0)),
        "id_plan_tau": float(getattr(lipm, "id_plan_tau", 0.0)),
        "id_plan_abs": float(getattr(lipm, "id_plan_abs", 0.0)),
        "id_plan_ask": float(getattr(lipm, "id_plan_ask", 0.0)),
        "id_plan_joint": str(getattr(lipm, "id_plan_joint", "")),
        "id_plan_phase": str(getattr(lipm, "id_plan_phase", "")),
        "id_plan_term": str(getattr(lipm, "id_plan_term", "")),
        "id_plan_t": float(getattr(lipm, "id_plan_t", 0.0)),
        "id_resid_exact_n": int(getattr(lipm, "id_resid_exact_n", 0)),
        "id_resid_bucket_nm": float(lipm_gait.ID_RESID_BUCKET_NM),
        "id_req_with": dict(getattr(lipm, "id_req_with", {})),
        "id_req_bare": dict(getattr(lipm, "id_req_bare", {})),
        "id_req_wall": bool(getattr(lipm, "id_req_wall", False)),
        "id_req_wall_n": int(getattr(lipm, "id_req_wall_n", 0)),
        "id_req_wall_joint": str(getattr(lipm, "id_req_wall_joint", "")),
        "id_req_wall_tau": float(getattr(lipm, "id_req_wall_tau", 0.0)),
        "id_req_wall_bare": float(getattr(lipm, "id_req_wall_bare", 0.0)),
        "id_req_wall_phase": str(getattr(lipm, "id_req_wall_phase", "")),
        "id_req_wall_t": float(getattr(lipm, "id_req_wall_t", 0.0)),
        "id_req_knee_abs": float(getattr(lipm, "id_req_knee_abs", 0.0)),
        "id_req_knee_tau": float(getattr(lipm, "id_req_knee_tau", 0.0)),
        "id_req_knee_joint": str(getattr(lipm, "id_req_knee_joint", "")),
        "id_req_knee_phase": str(getattr(lipm, "id_req_knee_phase", "")),
        "id_req_knee_t": float(getattr(lipm, "id_req_knee_t", 0.0)),
        "id_req_knee_qdd": float(getattr(lipm, "id_req_knee_qdd", 0.0)),
        "id_req_knee_bare": float(getattr(lipm, "id_req_knee_bare", 0.0)),
        "id_stop_span": float(getattr(lipm, "id_stop_span", 0.0)),
        "id_stop_qdd": float(getattr(lipm, "id_stop_qdd", 0.0)),
        "id_bind_joint": bind_joint,
        "id_bind_phase": bind_phase,
        "id_bind_term": bind_term,
        "id_bind_tau": bind_tau,
        "id_bind_t": bind_t,
        "id_bind_resid": bind_resid,
        "id_bind_inside": bind_inside,
        "id_fwdinv0_max": float(getattr(lipm, "id_fwdinv0_max", 0.0)),
        "id_fwdinv1_max": float(getattr(lipm, "id_fwdinv1_max", 0.0)),
        "id_fwdinv_t": float(getattr(lipm, "id_fwdinv_t", 0.0)),
        "id_impl_max": float(getattr(lipm, "id_impl_max", 0.0)),
        "id_resid_max": float(getattr(lipm, "id_resid_max", 0.0)),
        "id_resid_over_n": int(getattr(lipm, "id_resid_over_n", 0)),
        "id_resid_over_frac": (
            float(getattr(lipm, "id_resid_over_n", 0)) / float(lipm.id_phys_n)
            if int(getattr(lipm, "id_phys_n", 0)) else float("nan")
        ),
        "id_rail_n": int(getattr(lipm, "id_rail_n", 0)),
        "id_knee_act": float(getattr(lipm, "id_knee_act", 0.0)),
        "id_knee_ok_phase": str(getattr(lipm, "id_knee_ok_phase", "")),
        "id_knee_up_abs": float(getattr(lipm, "id_knee_up_abs", 0.0)),
        "id_knee_up_tau": float(getattr(lipm, "id_knee_up_tau", 0.0)),
        "id_knee_up_joint": str(getattr(lipm, "id_knee_up_joint", "")),
        "id_knee_up_t": float(getattr(lipm, "id_knee_up_t", 0.0)),
        "id_knee_up_phase": str(getattr(lipm, "id_knee_up_phase", "")),
        "id_knee_up_act": float(getattr(lipm, "id_knee_up_act", 0.0)),
        "id_knee_up_resid": float(getattr(lipm, "id_knee_up_resid", 0.0)),
        "mujoco_version": mj.__version__,
        "invdiscrete": hasattr(mj.mjtEnableBit, "mjENBL_INVDISCRETE"),
        "id_hold_tau": float(getattr(lipm, "id_hold_tau", 0.0)),
        "id_hold_t": float(getattr(lipm, "id_hold_t", -1.0)),
        "id_hold_act": float(getattr(lipm, "id_hold_act", 0.0)),
        "id_hold_phase": str(getattr(lipm, "id_hold_phase", "")),
        "id_hold_up": bool(getattr(lipm, "id_hold_up", False)),
        "id_hold_dt": float(getattr(lipm, "id_hold_dt", float("inf"))),
        "id_rail_hits": [
            {"t": float(t), "joint": str(j), "actuator": float(a), "inverse": float(inv)}
            for t, j, a, inv in getattr(lipm, "id_rail_hits", [])
        ],
        "id_ff_resid": float(getattr(lipm, "id_ff_resid", 0.0)),
        "id_resid_joint": str(getattr(lipm, "id_resid_joint", "")),
        "id_resid_pas_max": float(getattr(lipm, "id_resid_pas_max", 0.0)),
        "id_resid_fail_n": int(getattr(lipm, "id_resid_fail_n", 0)),
        "id_resid_vs_pas": float(getattr(lipm, "id_resid_vs_pas", 0.0)),
        "id_phys_n": int(getattr(lipm, "id_phys_n", 0)),
        "id_knee_ok_abs": float(getattr(lipm, "id_knee_ok_abs", 0.0)),
        "id_knee_ok_tau": float(getattr(lipm, "id_knee_ok_tau", 0.0)),
        "id_knee_ok_joint": str(getattr(lipm, "id_knee_ok_joint", "")),
        "id_knee_ok_t": float(getattr(lipm, "id_knee_ok_t", 0.0)),
        "id_knee_ok_n": int(getattr(lipm, "id_knee_ok_n", 0)),
        "id_wall_n": id_wall_n,
        "id_arm_n": id_arm_n,
        "id_ctrl_n": id_ctrl_n,
        "id_skip_n": id_skip_n,
        "id_in_n": id_in_n,
        "id_over_signed": id_over_signed,
        "id_wall_phys_n": int(getattr(lipm, "id_wall_phys_n", 0)),
        "id_arm_phys_n": int(getattr(lipm, "id_arm_phys_n", 0)),
        "limit_ticks": int(getattr(lipm, "limit_ticks", 0)),
        "ctrl_ticks": int(getattr(lipm, "ctrl_ticks", 0)),
        "limit_frac": (
            float(getattr(lipm, "limit_ticks", 0)) / float(lipm.ctrl_ticks)
            if int(getattr(lipm, "ctrl_ticks", 0)) else float("nan")
        ),
        "limit_ok": int(getattr(lipm, "limit_ticks", 0)) == 0 and int(getattr(lipm, "ctrl_ticks", 0)) > 0,
        "limit_ctrl_n": int(getattr(lipm, "limit_ctrl_n", 0)),
        "limit_force_n": int(getattr(lipm, "limit_force_n", 0)),
        "limit_slew_n": int(getattr(lipm, "limit_slew_n", 0)),
        "limit_band_n": int(getattr(lipm, "limit_band_n", 0)),
        "limit_torque_n": int(getattr(lipm, "limit_torque_n", 0)),
        "stop_up_z": stop_up_z,
        "stop_up_n": stop_up_n,
        "clamp_frac": clamp_frac,
        "clamp_max": clamp_max,
        "clamp_joint": clamp_joint,
        "clamp_ok": clamp_ok,
        "dc_excess": dc_excess,
        "dc_joint": dc_joint,
        "dc_tau": dc_tau,
        "dc_qvel": dc_spd,
        "dc_limit": dc_lim,
        "dc_t": dc_t,
        "dc_ok": dc_ok,
        "dc_label": DC_LABEL,
        "vx_mean": vx_mean,
        "vx_ratio": vx_ratio,
        "adv_mm": honesty["adv_mm"],
        "air_mm": honesty["air_mm"],
        "place_mm": honesty["place_mm"],
        "cmd_mm": honesty["cmd_mm"],
        "stride_mm": honesty["stride_mm"],
        "ds_mm": honesty["ds_mm"],
        "place_ok": honesty["place_ok"],
        "stride_ok": honesty["stride_ok"],
        "ds_ok": honesty["ds_ok"],
        "len_ok": honesty["len_ok"],
        "phase_mis": honesty["phase_mis"],
        "stance_n": honesty["stance_n"],
        "flat": honesty["flat"],
        "flat_spread_mm": honesty["flat_spread_mm"],
        "flat_pitch_deg": honesty["flat_pitch_deg"],
        "n_swings": honesty["n_swings"],
        "steps": honesty["steps"],
        "leg_m": leg_m,
        "reach_mm": math.sqrt(max(0.0, leg_m * leg_m - DS_FOOT_M * DS_FOOT_M)) * 1000.0,
        "crouch_mm": (
            min(float(r[side + "cz"]) for r in foot_rows for side in ("L", "R")) * 1000.0
            if foot_rows else float("nan")
        ),
        "crouch_ask_mm": (
            min(float(nearest[side + "cz"]) for side in ("L", "R")) * 1000.0
            if foot_rows and ask_all is not None and (
                nearest := min(foot_rows, key=lambda r: abs(float(r["t"]) - ask_all.t))
            ) else float("nan")
        ),
        "x_amp_mm": honesty["x_amp_mm"],
        "pitch_deg": honesty["pitch_deg"],
        "v_step_m_s": honesty["v_m_s"],
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
        "com_out": int(decl_com_stats["all"].n_out) if not kit_baseline else len(com_out),
        "cop_out": int(decl_cop_stats["all"].n_out) if not kit_baseline else len(cop_out),
        "zmp_out": int(decl_cop_stats["all"].n_out) if not kit_baseline else len(zmp_out),
        "ask_over": len(over),
        "com_min_m": com_min_m,
        "zmp_min_m": zmp_min_m,
        "cop_p1_mm": float(np.percentile(decl_cop_mm, 1)) if decl_cop_mm else float("nan"),
        "cop_p5_mm": float(np.percentile(decl_cop_mm, 5)) if decl_cop_mm else float("nan"),
        "cop_p50_mm": float(np.percentile(decl_cop_mm, 50)) if decl_cop_mm else float("nan"),
        "ss_cop_p5_mm": float(np.percentile(decl_ss_mm, 5)) if decl_ss_mm else float("nan"),
        "ss_cop_p50_mm": float(np.percentile(decl_ss_mm, 50)) if decl_ss_mm else float("nan"),
        "ss_cop_min_mm": float(min(decl_ss_mm)) if decl_ss_mm else float("nan"),
        "com_out_frac": com_frac,
        "zmp_out_frac": zmp_frac,
        "com_frac": _frac_dict(decl_com_stats if not kit_baseline else com_stats),
        "zmp_frac": _frac_dict(decl_cop_stats if not kit_baseline else zmp_stats),
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
        "perturb": perturb_info,
        "rug_peak_n": rug_peak,
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
        f"jerk {result['jerk_ok']} tip {result['tip_ok']} "
        f"step {result['step_ok']} cop_p5 {result['cop_p5_ok']}"
    )
    print(
        f"  step n {int(result['n_steps'])} "
        f"clear {float(result['clear_mm']):.2f} mm "
        f"(max {float(result['clear_max_mm']):.2f}, bar {STEP_CLEAR_M * 1000.0:.0f}) "
        f"slip {float(result['slip_mm']):.2f} mm "
        f"(sum {float(result['slip_sum_mm']):.2f}, bar {STEP_SLIP_M * 1000.0:.0f}) "
        f"sep {float(result['sep_mm']):.2f} mm "
        f"(x {float(result['x_amp_mm']):.2f}) "
        f"frac {float(result['step_frac']):.3f} "
        f"place {float(result['place_mm']):.1f} mm "
        f"(step {float(result['cmd_mm']):.1f}, {result['place_ok']}) "
        f"air {float(result['air_mm']):.1f} mm "
        f"(stride {float(result['stride_mm']):.1f}, {result['stride_ok']}) "
        f"ds ±{float(result['ds_mm']):.1f} mm ({result['ds_ok']}) "
        f"pitch {float(result['pitch_deg']):+.2f} deg "
        f"v {float(result['v_step_m_s']):.4f} m/s "
        f"phase_mis {float(result['phase_mis']):.3f} "
        f"stance_n {float(result['stance_n']):.0f} "
        f"flat {result['flat']} "
        f"spread {float(result['flat_spread_mm']):.2f} mm "
        f"sole {float(result['flat_pitch_deg']):.2f} deg"
    )
    print(
        f"  crouch min {float(result['crouch_mm']):.1f} mm "
        f"at ask {float(result['crouch_ask_mm']):.1f} mm "
        f"leg {float(result['leg_m']) * 1000.0:.1f} mm "
        f"reach at ±{DS_FOOT_M * 1000.0:.0f} mm is {float(result['reach_mm']):.1f} mm "
        f"ask {float(result['ask_nm']):.4f} Nm (bar {ASK_NM:.2f})"
    )
    steps = result.get("steps")
    if isinstance(steps, list) and steps:
        for step in steps:
            if not isinstance(step, dict):
                continue
            mark = "step" if bool(step.get("done")) else "down"
            print(
                f"  {mark} {step['side']} "
                f"t {float(step['t0']):.3f}-{float(step['t1']):.3f} "
                f"place {float(step['place_mm']):+.1f} mm "
                f"(step {float(result['cmd_mm']):.1f}) "
                f"air {float(step['air_mm']):+.1f} mm "
                f"(stride {float(result['stride_mm']):.1f}) "
                f"pel {float(step['fore_mm']):+.1f}/{float(step['fore_other_mm']):+.1f} mm "
                f"(DS ±{float(result['ds_mm']):.1f}) "
                f"crouch {float(step['crouch_mm']):.1f}/{float(step['crouch_other_mm']):.1f} mm "
                f"clear {float(step['clear_mm']):.2f} mm "
                f"(peak {float(step['peak_mm']):.2f}) "
                f"slip {float(step['slip_mm']):.2f} mm "
                f"frac {float(step['frac']):.3f} "
                f"n_air {int(step['n_air'])} "
                f"lift {float(step['t_lift']):.3f} "
                f"down {float(step['t_down']):.3f} "
                f"air_peak {float(step['air_peak_mm']):.2f} mm "
                f"air_dx {float(step['air_dx_mm']):+.1f} mm"
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


# Close-up is 0.15 m off the sole, side on. A 45° vertical fov puts an
# 8 mm gap on about 30 pixels. The wide camera at 1.7 m puts that same
# gap on about 3 pixels, which is why a review of the wide shot calls
# an 8 mm lift a slide.
#
# Elevation 0 sits the camera in the sole plane, so the sole and the
# horizon are the same row and the gap has no pixels. A slight look
# down keeps the camera low (about 6 cm) and opens the gap. The plant's
# default stereo separation is 68 mm. At 0.15 m that offset walks the
# foot out of the frame, so the render draws with ipd 0. Both are
# visual. Neither touches mjData.
FOOT_CAM_M = 0.15
FOOT_CAM_ELEV_DEG = -18.0
# Look just above the contact-box centre so the sole and the floor are
# both in the 480-row frame. The box centre alone, at this distance,
# clips the ankle off the top.
FOOT_CAM_LOOK_ABOVE_M = 0.010


def _foot_readout(session: sw.SteerSession) -> tuple[list[str], np.ndarray]:
    """Clearance, contact count, and normal from this mjData. No mj_forward."""
    lipm = session.lipm
    if lipm is None:
        raise SystemExit("walker did not build")
    lines = [f"t {float(session.data.time):5.2f}s"]
    look = np.zeros(3, dtype=np.float64)
    best = -1.0
    for side, name in (("L", "L"), ("R", "R")):
        gid = int(lipm.gid[side])
        clear = lipm_gait.sole_clearance(session.model, session.data, lipm.bid[side], gid)
        n_con = _n_ground(session, side)
        fn = float(lipm.foot_normal(side))
        lines.append(f"{name} {clear * 1000.0:7.2f} mm  n {n_con:d}  {fn:5.1f} N")
        if clear >= best:
            best = clear
            pos = np.asarray(session.data.geom_xpos[gid], dtype=np.float64)
            look[:] = (
                float(pos[0]),
                float(pos[1]),
                float(pos[2]) + FOOT_CAM_LOOK_ABOVE_M,
            )
    return lines, look


def render_side_front(
    out_mp4: Path,
    stand_s: float = 3.0,
    walk_s: float = 11.0,
    stop_s: float = 6.0,
    period_s: float | None = None,
    vx_m_s: float | None = None,
) -> None:
    """Side, front, and a sole-height close-up of the voice bout.

    30 fps. Each frame is the mjData just stepped, the same objects the
    scorer reads. The render does not call mj_forward and does not
    interpolate a pose. ``period_s`` renders that cadence instead of the
    wired voice period. The step is still x_amp = vx·T/4.
    """
    import tempfile

    import imageio.v2 as imageio

    import walk_gait_ainex as wg
    import zmp_preview

    vx = sw.VOICE_VX_M_S if vx_m_s is None else float(vx_m_s)
    if period_s is None:
        cfg = sw.gait_for_command(sw.VOICE_VX_M_S, 0.0)
    else:
        amp = zmp_preview.sway_zmp_amp(float(period_s), 0.25, 0.18, 0.016, 0.043, 0.0, 1.0e-4)
        cfg = lipm_gait.LipmConfig(
            name="voice056",
            clear_m=0.008,
            arms=True,
            schedule="gait_manager",
            gm_period_s=float(period_s),
            gm_dsp=0.25,
            gm_y_swap_m=0.0,
            gm_x_m=abs(vx) * float(period_s) / 4.0,
            gm_z_m=0.008,
            gm_z_swap_m=0.0,
            gm_pelvis_deg=0.0,
            gm_hip_pitch_deg=15.0,
            gm_start_lead="L",
            gm_crouch_m=0.025,
            gm_move_s=0.020,
            preview_amp_m=amp,
            preview_arm_s=1.0,
            preview_r=1.0e-4,
            preview_shape=0.0,
        )
    if cfg.preview_amp_m <= 1e-6 or cfg.gm_y_swap_m != 0.0:
        raise SystemExit("voice command did not select the preview gait")
    session = sw.SteerSession(
        video=True,
        lipm=cfg,
        cam_distance=1.70,
        cam_elevation=-8.0,
        cam_azimuth=90.0,
    )
    session.foot_trace = []
    if session.lipm is None or session.lipm.op3 is None or session.lipm.op3.y_swap_cmd != 0.0:
        raise SystemExit("preview walker is not the voice gait")
    script = sw.voice_bus_script(stand_s, walk_s, stop_s, 0.0, vx_m_s=vx)
    driver = sw.ScriptedDriver(script)
    t_end = float(script[-1].t_end)
    next_frame = 0.0
    frame_dt = 1.0 / 30.0
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="voice_frames_") as tmp:
        folder = Path(tmp)
        n = _write_frames(session, driver, t_end, folder, next_frame, frame_dt, imageio)
        if n < 2:
            raise SystemExit("render produced no frames")
        wg._encode_mp4_ffmpeg(folder, out_mp4, fps=30)
    print(
        f"  wrote {out_mp4}  frames {n}  video {n / 30.0:.2f} s  "
        f"sim {t_end:.2f} s  fault {session.bus.fault_reason or 'none'}"
    )


def _write_frames(session, driver, t_end, folder, next_frame, frame_dt, imageio) -> int:
    # Visual only. Restored before return. mj_step is the only physics call.
    vis = session.model.vis
    saved_ipd = float(vis.global_.ipd)
    saved_znear = float(vis.map.znear)
    vis.global_.ipd = 0.0
    vis.map.znear = 0.0002
    n = 0
    try:
        n = _write_frames_body(session, driver, t_end, folder, next_frame, frame_dt, imageio)
    finally:
        vis.global_.ipd = saved_ipd
        vis.map.znear = saved_znear
    return n


def _write_frames_body(session, driver, t_end, folder, next_frame, frame_dt, imageio) -> int:
    n = 0
    while float(session.data.time) < t_end - 1e-12:
        now = float(session.data.time)
        driver.publish(session.bus, now)
        session.step()
        if session.lipm is None or session.lipm.op3 is None:
            raise SystemExit("walker dropped")
        if session.lipm.op3.y_swap_cmd != 0.0:
            raise SystemExit("y_swap_cmd changed")
        if now + 1e-9 < next_frame:
            continue
        session.foot_trace.append((
            np.array(session.data.geom_xpos[int(session.lipm.gid["L"])], dtype=np.float64).copy(),
            np.array(session.data.geom_xpos[int(session.lipm.gid["R"])], dtype=np.float64).copy(),
        ))
        q_before = np.array(session.data.qpos, dtype=np.float64).copy()
        t_before = float(session.data.time)
        lines, look = _foot_readout(session)
        side = session.render(
            ["side  " + lines[0], lines[1], lines[2]],
            distance=1.70, azimuth=90.0, elevation=-8.0,
        )
        front = session.render(
            [f"front  {lines[0]}  vx {session.bus.applied_vx:+.3f}", lines[1], lines[2]],
            distance=1.70, azimuth=0.0, elevation=-8.0,
        )
        close = session.render(
            ["sole  " + lines[0], lines[1], lines[2]],
            lookat=look, distance=FOOT_CAM_M, azimuth=90.0, elevation=FOOT_CAM_ELEV_DEG,
            sole_box=True,
        )
        if float(session.data.time) != t_before or not np.allclose(session.data.qpos, q_before):
            raise SystemExit("render changed mjData")
        imageio.imwrite(folder / f"frame_{n:05d}.png", np.concatenate([side, front, close], axis=1))
        n += 1
        next_frame += frame_dt
    return n


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
