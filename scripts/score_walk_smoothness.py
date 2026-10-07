#!/usr/bin/env python3
"""Jerk and ZMP-margin score for the locked Day-1 kit walk.

Uses the existing harness in ``scripts/steer_walk.py``: ``SteerSession`` with
``locked_kit_config()`` and ``BUS_KIT_SCRIPT`` (stand, forward, stop). It does
not retune the gait, edit the plant, raise the torque rail, or open tip-yaw,
reach, voice, or d_min.

Signals
-------
CoM jerk
    ``body_link`` subtree CoM in world metres. Sampled once per kit control
    tick (8 ms on the gait-manager schedule). Three backward differences.
    Unfiltered. Peak and RMS are of the 3-vector Euclidean norm, plus each
    axis.
Joint jerk
    Hinge ``qpos`` of every position actuator, radians, same differences.
    Peak and RMS are of that vector's Euclidean norm. The joint with the
    largest peak |jerk| is named.
ZMP
    Normal-force-weighted MuJoCo contact position on ``l_foot_contact`` and
    ``r_foot_contact`` against the floor geom. ``condim`` is 3, so each
    contact's CoP is the contact point. This is the same sum as
    ``LipmWalker._cop_local``, taken in world and over both feet. It is the
    plant contact CoP, not a second estimator and not a preview controller.
Support
    The gait phase already on the walker. ``stand`` and ``shift`` (double
    support) use the convex hull of both foot boxes. ``swing`` uses only the
    stance foot box. A dragging swing foot is not added to the polygon.
    Margin is the signed distance to that polygon (positive inside), from
    ``support_margin``. Any foot whose floor normal is at least the locked
    ``unload_n`` (5 N) must also keep its own CoP inside its 135×76 mm box
    (``LipmWalker.cop_margin``). The sample margin is the worse of those.
    A declared stance foot with no load is outside. Samples are not dropped.
Soft-pass
    Off. One negative margin, a tip, or a bus fault is Prefer FAIL. There is
    no millimetre allowance.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco as mj
import numpy as np
import numpy.typing as npt

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import lipm_gait  # noqa: E402
import steer_walk  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PLANT_XML = steer_walk.PLANT_XML
FROZEN_MD5 = "207f3d5e9c6a72e16f7aa0c8d224f75e"
# Stated hardware figures. Checked against the frozen file. Not substitutes.
STATED_MASS_KG = 2.347
STATED_TRUNK_MASS_KG = 0.743
STATED_TRUNK_COM_M = (0.00394, -0.00008, 0.05045)
STATED_ROOT_Z_M = 0.268
STATED_HIP_YAW_M = 0.029
STATED_FOOT_HALF_M = (0.0675, 0.0380, 0.008)
STATED_FOOT_POS_M = {
    "l_foot_contact": (0.030, 0.014, -0.018),
    "r_foot_contact": (0.030, -0.014, -0.018),
}
SAG_BAR_NM = lipm_gait.KNEE_SAG_NM
TIP_UP_Z = 0.85
SOFT_PASS = False
OUT_JSON = ROOT / "previews" / "walk_smoothness_summary.json"
OUT_MD = ROOT / "docs" / "WALK_SMOOTHNESS_SCORE.md"

Vec = npt.NDArray[np.float64]
PhaseLabel = Literal["stand", "ds", "ss_L", "ss_R", "unknown"]
Verdict = Literal["Prefer FAIL", "inside"]


@dataclass(frozen=True)
class JerkStats:
    signal: str
    unit: str
    dt_s: float
    n: int
    peak_l2: float
    rms_l2: float
    axis_peak: dict[str, float]
    axis_rms: dict[str, float]
    worst_name: str
    worst_peak: float


@dataclass
class PhaseBucket:
    phase: str
    n: int = 0
    n_outside: int = 0
    n_no_contact: int = 0
    n_stance_unloaded: int = 0
    n_margin: int = 0
    min_margin_m: float = math.inf
    sum_margin_m: float = 0.0

    def add(self, tick: ZmpTick) -> None:
        self.n += 1
        if tick.outside:
            self.n_outside += 1
        if tick.no_contact:
            self.n_no_contact += 1
        if tick.stance_unloaded:
            self.n_stance_unloaded += 1
        if tick.margin_m is not None:
            self.n_margin += 1
            self.sum_margin_m += tick.margin_m
            self.min_margin_m = min(self.min_margin_m, tick.margin_m)

    def outside_fraction(self) -> float:
        if self.n == 0:
            return 1.0
        return self.n_outside / self.n

    def mean_margin_m(self) -> float | None:
        if self.n_margin == 0:
            return None
        return self.sum_margin_m / self.n_margin

    def min_or_none(self) -> float | None:
        if self.n_margin == 0:
            return None
        return self.min_margin_m


@dataclass(frozen=True)
class WorstZmp:
    t_s: float
    phase: str
    margin_m: float
    zmp_x_m: float
    zmp_y_m: float
    com_margin_m: float


@dataclass
class SmoothnessScore:
    plant_md5: str
    plant_md5_after: str
    soft_pass: bool
    verdict: Verdict
    honesty: str
    bout_s: float
    n_samples: int
    ctrl_dt_s: float
    min_up_z: float
    tipped: bool
    fault_reasons: list[str]
    zmp_min_margin_m: float | None
    zmp_outside_fraction: float
    zmp_no_contact_n: int
    ss_stance_unloaded_n: int
    com_min_margin_m: float | None
    com_outside_fraction: float
    harness_com_min_margin_m: float
    max_leg_tau_nm: float
    sag_bar_nm: float
    phases: list[PhaseBucket]
    worst_zmp: WorstZmp | None
    com_jerk: JerkStats | None
    joint_jerk: JerkStats | None
    joint_names: list[str]
    plant_notes: list[str] = field(default_factory=list)


def finite_jerk(samples: Vec, dt: float) -> Vec:
    """Three backward differences. ``samples`` is (N, D). Returns (N-3, D)."""
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    if samples.ndim != 2 or samples.shape[0] < 4:
        return np.zeros((0, samples.shape[1] if samples.ndim == 2 else 0), dtype=np.float64)
    vel = np.diff(samples, axis=0) / dt
    acc = np.diff(vel, axis=0) / dt
    return np.diff(acc, axis=0) / dt


def _l2_rows(values: Vec) -> Vec:
    return np.linalg.norm(values, axis=1)


def _peak_rms(series: Vec) -> tuple[float, float]:
    if series.size == 0:
        return 0.0, 0.0
    peak = float(np.max(np.abs(series)))
    rms = float(np.sqrt(np.mean(np.square(series))))
    return peak, rms


def jerk_stats(samples: Vec, dt: float, names: list[str], signal: str, unit: str) -> JerkStats | None:
    jerk = finite_jerk(samples, dt)
    if jerk.shape[0] == 0 or len(names) != jerk.shape[1]:
        return None
    norms = _l2_rows(jerk)
    peak_l2 = float(np.max(norms))
    rms_l2 = float(np.sqrt(np.mean(np.square(norms))))
    axis_peak: dict[str, float] = {}
    axis_rms: dict[str, float] = {}
    worst_i = 0
    worst_peak = -1.0
    for i, name in enumerate(names):
        peak, rms = _peak_rms(jerk[:, i])
        axis_peak[name] = peak
        axis_rms[name] = rms
        if peak > worst_peak:
            worst_peak = peak
            worst_i = i
    return JerkStats(
        signal=signal,
        unit=unit,
        dt_s=dt,
        n=int(jerk.shape[0]),
        peak_l2=peak_l2,
        rms_l2=rms_l2,
        axis_peak=axis_peak,
        axis_rms=axis_rms,
        worst_name=names[worst_i],
        worst_peak=worst_peak,
    )


def decide(min_margin_m: float, outside_fraction: float, tipped: bool, measured: bool) -> Verdict:
    """Soft-pass is not a parameter. A miss, a tip, or a missing bout fails."""
    if not measured or tipped or outside_fraction > 0.0 or min_margin_m < 0.0:
        return "Prefer FAIL"
    return "inside"


def phase_label(phase: str, stance: str) -> PhaseLabel:
    if phase == "swing" and stance == "L":
        return "ss_L"
    if phase == "swing" and stance == "R":
        return "ss_R"
    if phase == "shift":
        return "ds"
    if phase == "stand":
        return "stand"
    return "unknown"


def stance_sides(label: PhaseLabel, stance: str) -> tuple[str, ...]:
    if label == "ss_L":
        return ("L",)
    if label == "ss_R":
        return ("R",)
    if label in ("ds", "stand"):
        return ("L", "R")
    if stance in ("L", "R"):
        return (stance,)
    return ()


def _plant_md5() -> str:
    return hashlib.md5(PLANT_XML.read_bytes()).hexdigest()


def _close(got: float, want: float, tol: float) -> bool:
    return abs(got - want) <= tol


def plant_notes(model: mj.MjModel) -> list[str]:
    """Read the frozen file. Do not write it and do not replace a mismatch."""
    notes: list[str] = []
    digest = _plant_md5()
    if digest != FROZEN_MD5:
        notes.append(f"plant md5 {digest} != {FROZEN_MD5}")
    mass = float(np.sum(model.body_mass))
    if not _close(mass, STATED_MASS_KG, 0.002):
        notes.append(f"total mass {mass:.4f} kg != stated {STATED_MASS_KG:.3f}")
    bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    trunk = float(model.body_mass[bid])
    if not _close(trunk, STATED_TRUNK_MASS_KG, 0.001):
        notes.append(f"trunk mass {trunk:.4f} kg != stated {STATED_TRUNK_MASS_KG:.3f}")
    ipos = np.asarray(model.body_ipos[bid], dtype=np.float64)
    for i, stated in enumerate(STATED_TRUNK_COM_M):
        if not _close(float(ipos[i]), stated, 5e-5):
            notes.append(f"trunk CoM[{i}] {float(ipos[i]):.6f} != stated {stated:.5f}")
    root = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "body_link")
    root_z = float(model.body_pos[root, 2])
    if not _close(root_z, STATED_ROOT_Z_M, 1e-6):
        notes.append(f"root z {root_z:.4f} != stated {STATED_ROOT_Z_M:.3f}")
    for side, sign in (("l", 1.0), ("r", -1.0)):
        hip = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, f"{side}_hip_yaw_link")
        y = float(model.body_pos[hip, 1])
        if not _close(y, sign * STATED_HIP_YAW_M, 1e-6):
            notes.append(f"{side} hip yaw y {y:.4f} != {sign * STATED_HIP_YAW_M:.3f}")
    for name, pos in STATED_FOOT_POS_M.items():
        gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
        size = np.asarray(model.geom_size[gid], dtype=np.float64)
        centre = np.asarray(model.geom_pos[gid], dtype=np.float64)
        for i, stated in enumerate(STATED_FOOT_HALF_M):
            if not _close(float(size[i]), stated, 1e-9):
                notes.append(f"{name} half[{i}] {float(size[i]):.4f} != {stated:.4f}")
        for i, stated in enumerate(pos):
            if not _close(float(centre[i]), stated, 1e-9):
                notes.append(f"{name} pos[{i}] {float(centre[i]):.4f} != {stated:.4f}")
    cam = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "kit_cam")
    cam_pos = np.asarray(model.cam_pos[cam], dtype=np.float64)
    if not np.allclose(cam_pos, (0.050, 0.019, 0.007), atol=1e-9):
        notes.append(f"kit_cam pos {cam_pos.tolist()}")
    for i in range(model.nu):
        aname = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        if not any(tok in aname for tok in ("hip_", "knee", "ank_")):
            continue
        fr = np.asarray(model.actuator_forcerange[i], dtype=np.float64)
        if not np.allclose(fr, (-2.45, 2.45), atol=1e-9):
            notes.append(f"{aname} forcerange {fr.tolist()} != ±2.45")
    if not notes:
        notes.append(
            f"plant md5 {digest} matches the cleared geometry; "
            f"mass {mass:.3f} kg; trunk {trunk:.3f} kg; "
            f"foot half {STATED_FOOT_HALF_M[0]:.4f}×{STATED_FOOT_HALF_M[1]:.4f}×{STATED_FOOT_HALF_M[2]:.3f} m; "
            "centres unchanged; forcerange ±2.45; kit_cam frozen"
        )
    return notes


def actuated_q(model: mj.MjModel, data: mj.MjData) -> tuple[list[str], Vec]:
    names: list[str] = []
    vals: list[float] = []
    for i in range(model.nu):
        jid = int(model.actuator_trnid[i, 0])
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, jid)
        if name is None:
            continue
        adr = int(model.jnt_qposadr[jid])
        names.append(name)
        vals.append(float(data.qpos[adr]))
    return names, np.asarray(vals, dtype=np.float64)


def foot_floor_cop(
    model: mj.MjModel,
    data: mj.MjData,
    gid: int,
    floor_gid: int,
) -> tuple[Vec, float] | None:
    """World CoP and normal load. Same contact sum as ``LipmWalker._cop_local``."""
    num = np.zeros(3, dtype=np.float64)
    den = 0.0
    for i in range(data.ncon):
        con = data.contact[i]
        g1 = int(con.geom1)
        g2 = int(con.geom2)
        if not ((g1 == gid and g2 == floor_gid) or (g2 == gid and g1 == floor_gid)):
            continue
        force = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(model, data, i, force)
        fn = float(force[0])
        if fn <= 1e-6:
            continue
        num += fn * np.asarray(con.pos, dtype=np.float64)
        den += fn
    if den <= 1e-6:
        return None
    return num / den, den


@dataclass(frozen=True)
class ZmpTick:
    phase: PhaseLabel
    zmp_x_m: float
    zmp_y_m: float
    # Signed distance of the contact CoP to the phase polygon, in metres.
    # None when this tick has no floor contact, so a sentinel is not reported as a distance.
    margin_m: float | None
    com_margin_m: float | None
    outside: bool
    no_contact: bool
    stance_unloaded: bool


def sample_zmp(
    session: steer_walk.SteerSession,
    walker: lipm_gait.LipmWalker,
) -> ZmpTick:
    """Contact CoP against the phase stance polygon.

    The margin is the geometric signed distance. A missing contact is outside,
    and it does not invent a −1 m edge distance. A single-support tick whose
    declared stance foot is under ``unload_n`` is outside even if the CoP of
    the other foot happens to land in the stance box.
    """
    label = phase_label(walker.phase, walker.stance)
    sides = stance_sides(label, walker.stance)
    num = np.zeros(3, dtype=np.float64)
    den = 0.0
    for side in ("L", "R"):
        got = foot_floor_cop(session.model, session.data, walker.gid[side], walker.gid_floor)
        if got is None:
            continue
        world, fn = got
        num += fn * world
        den += fn
    no_contact = den <= 1e-6 or not sides or label == "unknown"
    margins: list[float] = []
    zmp_xy = np.zeros(2, dtype=np.float64)
    if not no_contact:
        zmp = num / den
        zmp_xy = zmp[:2].copy()
        chunks = [
            session._foot_corners(walker.bid[side], walker.gid[side])
            for side in sides
        ]
        hull = steer_walk.convex_hull_xy(np.concatenate(chunks, axis=0))
        margins.append(steer_walk.support_margin(zmp_xy, hull))
    unload = float(walker.cfg.unload_n)
    stance_unloaded = False
    box_missing = False
    for side in ("L", "R"):
        fn = walker.foot_normal(side)
        if fn < unload:
            if side in sides and label.startswith("ss"):
                stance_unloaded = True
            continue
        slack = walker.cop_margin(side)
        # cop_margin returns −1 when that foot has no CoP. A loaded foot
        # without a CoP is outside; the −1 is not an edge distance.
        if slack <= -0.5:
            box_missing = True
        else:
            margins.append(slack)
    margin: float | None = min(margins) if margins else None
    com = np.asarray(session.data.subtree_com[walker.bid_body, :2], dtype=np.float64)
    com_margin: float | None
    if sides:
        chunks = [
            session._foot_corners(walker.bid[side], walker.gid[side])
            for side in sides
        ]
        hull = steer_walk.convex_hull_xy(np.concatenate(chunks, axis=0))
        com_margin = float(steer_walk.support_margin(com, hull))
    else:
        com_margin = None
    outside = (
        no_contact
        or stance_unloaded
        or box_missing
        or margin is None
        or margin < 0.0
    )
    return ZmpTick(
        phase=label,
        zmp_x_m=float(zmp_xy[0]),
        zmp_y_m=float(zmp_xy[1]),
        margin_m=None if margin is None else float(margin),
        com_margin_m=com_margin,
        outside=outside,
        no_contact=no_contact,
        stance_unloaded=stance_unloaded,
    )


def _uniform_dt(times: Vec) -> float | None:
    if times.size < 2:
        return None
    dts = np.diff(times)
    dt = float(np.median(dts))
    if dt <= 0.0 or float(np.max(np.abs(dts - dt))) > 1e-6:
        return None
    return dt


def run_bout() -> SmoothnessScore:
    digest_before = _plant_md5()
    session = steer_walk.SteerSession(video=False, lipm=steer_walk.locked_kit_config())
    if session.lipm is None:
        raise RuntimeError("kit session has no walker")
    walker = session.lipm
    notes = plant_notes(session.model)
    driver = steer_walk.ScriptedDriver(steer_walk.BUS_KIT_SCRIPT)
    times: list[float] = []
    com_rows: list[Vec] = []
    q_rows: list[Vec] = []
    joint_names: list[str] = []
    buckets: dict[str, PhaseBucket] = {
        name: PhaseBucket(phase=name) for name in ("stand", "ds", "ss_L", "ss_R", "unknown")
    }
    worst: WorstZmp | None = None
    com_out = 0
    com_known = 0
    zmp_out = 0
    n = 0
    n_no_contact = 0
    n_unloaded = 0
    min_zmp: float | None = None
    min_com: float | None = None
    reasons: list[str] = []
    while float(session.data.time) < steer_walk.BUS_KIT_STOP_S - 1e-9:
        now = float(session.data.time)
        driver.publish(session.bus, now)
        session.step()
        if session.bus.fault and session.bus.fault_reason and session.bus.fault_reason not in reasons:
            reasons.append(session.bus.fault_reason)
        tick = sample_zmp(session, walker)
        buckets[tick.phase].add(tick)
        n += 1
        if tick.outside:
            zmp_out += 1
        if tick.no_contact:
            n_no_contact += 1
        if tick.stance_unloaded:
            n_unloaded += 1
        if tick.com_margin_m is not None:
            com_known += 1
            if tick.com_margin_m < 0.0:
                com_out += 1
            if min_com is None or tick.com_margin_m < min_com:
                min_com = tick.com_margin_m
        if tick.margin_m is not None and (min_zmp is None or tick.margin_m < min_zmp):
            min_zmp = tick.margin_m
        if tick.margin_m is not None and tick.com_margin_m is not None:
            if worst is None or tick.margin_m < worst.margin_m:
                worst = WorstZmp(
                    t_s=float(session.data.time),
                    phase=tick.phase,
                    margin_m=tick.margin_m,
                    zmp_x_m=tick.zmp_x_m,
                    zmp_y_m=tick.zmp_y_m,
                    com_margin_m=tick.com_margin_m,
                )
        names, q = actuated_q(session.model, session.data)
        if not joint_names:
            joint_names = names
        com_rows.append(np.asarray(session.data.subtree_com[walker.bid_body], dtype=np.float64).copy())
        q_rows.append(q)
        times.append(float(session.data.time))
    session.assert_plant_unchanged()
    digest_after = _plant_md5()
    if digest_after != digest_before:
        notes.append(f"plant md5 changed during the bout to {digest_after}")
    dt = _uniform_dt(np.asarray(times, dtype=np.float64))
    com_jerk: JerkStats | None = None
    joint_jerk: JerkStats | None = None
    if dt is not None and com_rows and q_rows:
        com_jerk = jerk_stats(
            np.stack(com_rows, axis=0),
            dt,
            ["x", "y", "z"],
            "body_link subtree_com world",
            "m/s^3",
        )
        joint_jerk = jerk_stats(
            np.stack(q_rows, axis=0),
            dt,
            joint_names,
            "actuated hinge qpos",
            "rad/s^3",
        )
    tipped = bool(session.min_up_z < TIP_UP_Z or reasons)
    outside_fraction = (zmp_out / n) if n else 1.0
    com_fraction = (com_out / com_known) if com_known else 1.0
    measured = (
        n > 0
        and min_zmp is not None
        and com_jerk is not None
        and joint_jerk is not None
        and digest_after == FROZEN_MD5
    )
    if any(note.startswith("plant md5") and "!=" in note for note in notes):
        measured = False
    verdict = decide(
        min_zmp if min_zmp is not None else -1.0,
        outside_fraction,
        tipped,
        measured,
    )
    score = SmoothnessScore(
        plant_md5=digest_before,
        plant_md5_after=digest_after,
        soft_pass=SOFT_PASS,
        verdict=verdict,
        honesty="",
        bout_s=steer_walk.BUS_KIT_STOP_S,
        n_samples=n,
        ctrl_dt_s=float(session.ctrl_dt),
        min_up_z=float(session.min_up_z),
        tipped=tipped,
        fault_reasons=reasons,
        zmp_min_margin_m=min_zmp,
        zmp_outside_fraction=float(outside_fraction),
        zmp_no_contact_n=n_no_contact,
        ss_stance_unloaded_n=n_unloaded,
        com_min_margin_m=min_com,
        com_outside_fraction=float(com_fraction),
        harness_com_min_margin_m=float(session.min_margin) if math.isfinite(session.min_margin) else -1.0,
        max_leg_tau_nm=float(session.max_leg_tau),
        sag_bar_nm=SAG_BAR_NM,
        phases=[buckets[name] for name in ("stand", "ds", "ss_L", "ss_R")],
        worst_zmp=worst,
        com_jerk=com_jerk,
        joint_jerk=joint_jerk,
        joint_names=joint_names,
        plant_notes=notes,
    )
    score.honesty = honesty_line(score)
    return score


def _metres(value: float | None) -> str:
    if value is None:
        return "unmeasured"
    return f"{value:+.6f} m"


def honesty_line(score: SmoothnessScore) -> str:
    zmp = (
        f"ZMP/CoP min margin {_metres(score.zmp_min_margin_m)}, "
        f"outside fraction {score.zmp_outside_fraction:.3f}, "
        f"no-contact samples {score.zmp_no_contact_n}, "
        f"single-support stance under 5 N {score.ss_stance_unloaded_n}"
    )
    com = (
        f"CoM projection on the same polygon min margin {_metres(score.com_min_margin_m)}, "
        f"outside fraction {score.com_outside_fraction:.3f}"
    )
    tip = f"min up_z {score.min_up_z:.3f}"
    if score.fault_reasons:
        tip += " fault " + "; ".join(score.fault_reasons)
    plant = f"plant md5 {score.plant_md5_after}"
    if score.plant_md5_after != FROZEN_MD5:
        plant += " MOVED"
    else:
        plant += " unchanged"
    sag = f"peak leg torque {score.max_leg_tau_nm:.2f} Nm (sag bar {score.sag_bar_nm:.2f}, rail ±2.45, not raised)"
    com_note = ""
    if score.com_min_margin_m is not None and score.com_min_margin_m < 0.0:
        com_note = " CoM outside the stance polygon is a gait miss and is not cleared here."
    if score.verdict == "Prefer FAIL":
        return (
            f"Prefer FAIL. Soft-pass off. {zmp}. {com}. {tip}. {sag}. {plant}.{com_note}"
        )
    return (
        f"ZMP stayed inside the stance polygon. Soft-pass off. {zmp}. {com}. {tip}. {sag}. {plant}.{com_note}"
    )


def _jerk_dict(stats: JerkStats | None) -> dict[str, object] | None:
    if stats is None:
        return None
    return {
        "signal": stats.signal,
        "unit": stats.unit,
        "dt_s": stats.dt_s,
        "n": stats.n,
        "peak_l2": stats.peak_l2,
        "rms_l2": stats.rms_l2,
        "axis_peak": stats.axis_peak,
        "axis_rms": stats.axis_rms,
        "worst_name": stats.worst_name,
        "worst_peak": stats.worst_peak,
    }


def score_dict(score: SmoothnessScore) -> dict[str, object]:
    worst: dict[str, object] | None = None
    if score.worst_zmp is not None:
        worst = {
            "t_s": score.worst_zmp.t_s,
            "phase": score.worst_zmp.phase,
            "margin_m": score.worst_zmp.margin_m,
            "zmp_x_m": score.worst_zmp.zmp_x_m,
            "zmp_y_m": score.worst_zmp.zmp_y_m,
            "com_margin_m": score.worst_zmp.com_margin_m,
        }
    phases: list[dict[str, object]] = []
    for bucket in score.phases:
        phases.append({
            "phase": bucket.phase,
            "n": bucket.n,
            "n_outside": bucket.n_outside,
            "n_no_contact": bucket.n_no_contact,
            "n_stance_unloaded": bucket.n_stance_unloaded,
            "min_margin_m": bucket.min_or_none(),
            "mean_margin_m": bucket.mean_margin_m(),
            "outside_fraction": None if bucket.n == 0 else bucket.outside_fraction(),
        })
    return {
        "bout": "Day-1 kit BUS_KIT_SCRIPT stand/forward/stop on locked_kit_config",
        "plant": str(PLANT_XML.relative_to(ROOT)),
        "plant_md5": score.plant_md5,
        "plant_md5_after": score.plant_md5_after,
        "frozen_md5": FROZEN_MD5,
        "soft_pass": score.soft_pass,
        "verdict": score.verdict,
        "honesty": score.honesty,
        "bout_s": score.bout_s,
        "n_samples": score.n_samples,
        "ctrl_dt_s": score.ctrl_dt_s,
        "min_up_z": score.min_up_z,
        "tipped": score.tipped,
        "tip_bar_up_z": TIP_UP_Z,
        "fault_reasons": score.fault_reasons,
        "zmp_signal": "floor contact CoP on l_foot_contact and r_foot_contact (LipmWalker._cop_local sum)",
        "zmp_min_margin_m": score.zmp_min_margin_m,
        "zmp_outside_fraction": score.zmp_outside_fraction,
        "zmp_no_contact_n": score.zmp_no_contact_n,
        "ss_stance_unloaded_n": score.ss_stance_unloaded_n,
        "com_min_margin_m": score.com_min_margin_m,
        "com_outside_fraction": score.com_outside_fraction,
        "harness_com_min_margin_m": score.harness_com_min_margin_m,
        "max_leg_tau_nm": score.max_leg_tau_nm,
        "sag_bar_nm": score.sag_bar_nm,
        "forcerange_nm": 2.45,
        "phases": phases,
        "worst_zmp": worst,
        "com_jerk": _jerk_dict(score.com_jerk),
        "joint_jerk": _jerk_dict(score.joint_jerk),
        "joint_names": score.joint_names,
        "plant_notes": score.plant_notes,
        "tip_sha_101": "7adccd1248cd432f5ab083e8c39489e3bbf4b065",
        "tip_101_note": "comment-only plant hash 7f5fc0b; contact polygon identical to this md5",
    }


def render_md(score: SmoothnessScore) -> str:
    def cell(value: float | None) -> str:
        if value is None:
            return "—"
        return f"{value:+.6f}"

    phase_rows: list[str] = []
    for bucket in score.phases:
        if bucket.n == 0:
            phase_rows.append(f"| {bucket.phase} | 0 | — | — | — | 0 | 0 |")
            continue
        phase_rows.append(
            f"| {bucket.phase} | {bucket.n} | {cell(bucket.min_or_none())} | "
            f"{cell(bucket.mean_margin_m())} | {bucket.outside_fraction():.3f} | "
            f"{bucket.n_no_contact} | {bucket.n_stance_unloaded} |"
        )
    com = score.com_jerk
    joint = score.joint_jerk
    com_line = "not measured"
    joint_line = "not measured"
    if com is not None:
        com_line = (
            f"peak {com.peak_l2:.3f} {com.unit}, RMS {com.rms_l2:.3f} {com.unit} "
            f"(x {com.axis_peak['x']:.3f}/{com.axis_rms['x']:.3f}, "
            f"y {com.axis_peak['y']:.3f}/{com.axis_rms['y']:.3f}, "
            f"z {com.axis_peak['z']:.3f}/{com.axis_rms['z']:.3f} peak/RMS)"
        )
    if joint is not None:
        joint_line = (
            f"peak {joint.peak_l2:.3f} {joint.unit}, RMS {joint.rms_l2:.3f} {joint.unit}; "
            f"worst joint {joint.worst_name} peak |jerk| {joint.worst_peak:.3f} {joint.unit}"
        )
    worst = "—"
    if score.worst_zmp is not None:
        w = score.worst_zmp
        worst = (
            f"t={w.t_s:.3f} s phase={w.phase} margin={w.margin_m:+.6f} m "
            f"ZMP=({w.zmp_x_m:+.4f}, {w.zmp_y_m:+.4f}) m"
        )
    faults = ", ".join(score.fault_reasons) if score.fault_reasons else "none"
    return "\n".join([
        "# Day-1 kit walk smoothness score",
        "",
        "Harness: `SteerSession(lipm=locked_kit_config())` and `BUS_KIT_SCRIPT` in `scripts/steer_walk.py`.",
        "Stand 0.50 s, forward until 6.00 s, stop until 7.60 s. Yaw, reach, voice, and d_min are not run.",
        "Soft-pass is off. The plant file is not written.",
        "",
        f"**{score.honesty}**",
        "",
        "| | |",
        "| --- | --- |",
        f"| Verdict | {score.verdict} |",
        f"| Soft-pass | {str(score.soft_pass).lower()} |",
        f"| Plant md5 | `{score.plant_md5_after}` |",
        f"| Frozen md5 | `{FROZEN_MD5}` |",
        f"| #101 tip | `7adccd1` (comment-only; contact polygon unchanged) |",
        f"| Samples | {score.n_samples} at {score.ctrl_dt_s * 1000:.0f} ms |",
        f"| min up_z | {score.min_up_z:.3f} (tip bar {TIP_UP_Z:.2f}) |",
        f"| Faults | {faults} |",
        f"| ZMP min margin | {_metres(score.zmp_min_margin_m)} |",
        f"| ZMP outside fraction | {score.zmp_outside_fraction:.3f} |",
        f"| ZMP no-contact samples | {score.zmp_no_contact_n} |",
        f"| SS stance under 5 N | {score.ss_stance_unloaded_n} |",
        f"| CoM min margin (same polygon) | {_metres(score.com_min_margin_m)} |",
        f"| CoM outside fraction | {score.com_outside_fraction:.3f} |",
        f"| Harness CoM min margin | {score.harness_com_min_margin_m:+.6f} m |",
        f"| Peak leg torque | {score.max_leg_tau_nm:.3f} Nm |",
        f"| Sag bar / forcerange | {score.sag_bar_nm:.2f} Nm / ±2.45 Nm (not raised) |",
        f"| Worst ZMP sample | {worst} |",
        "",
        "## Jerk",
        "",
        "Unfiltered third difference at the 8 ms control tick. A contact impact sets the peak. RMS is the bout figure. Nothing is low-passed.",
        "",
        f"- CoM (`body_link` subtree_com): {com_line}",
        f"- Actuated joints (hinge qpos, same differences): {joint_line}",
        "",
        "## ZMP margin by phase",
        "",
        "Margin is metres to the stance polygon edge, on ticks that have a contact CoP. Negative is outside that polygon or outside a loaded foot's own box. Outside fraction also counts ticks with no floor contact and single-support ticks whose stance foot is under 5 N. Those ticks are not given a fake −1 m margin. No sample is dropped.",
        "",
        "| Phase | n | min margin (m) | mean margin (m) | outside fraction | no contact | stance unloaded |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        *phase_rows,
        "",
        "## Signals",
        "",
        "- ZMP is the floor-contact CoP already in the plant (`LipmWalker._cop_local` normal-force sum on both foot boxes). Not a preview law.",
        "- Single support polygon is the stance foot box (135×76 mm, half 0.0675×0.0380 m, centres read from the frozen geom). Double support and stand use the convex hull of both boxes.",
        "- A foot loaded at or above `unload_n` (5 N) must keep its own CoP inside its box. That slack is included in the sample margin.",
        "- CoM margin is the body subtree CoM against the same phase polygon. It is reported beside ZMP. It does not soften a ZMP miss.",
        "",
        "## Plant check",
        "",
        *[f"- {note}" for note in score.plant_notes],
        "",
    ])


def write_outputs(score: SmoothnessScore, json_path: Path, md_path: Path) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    md_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(score_dict(score), indent=2) + "\n")
    md_path.write_text(render_md(score))


def self_test() -> int:
    failures: list[str] = []

    def expect(cond: bool, msg: str) -> None:
        if not cond:
            failures.append(msg)

    dt = 0.008
    t = np.arange(12, dtype=np.float64) * dt
    jerk_true = np.array([1.5, -0.4, 0.2], dtype=np.float64)
    # p = (1/6) j t^3  → third derivative is j.
    pos = (jerk_true[None, :] * (t[:, None] ** 3)) / 6.0
    got = finite_jerk(pos, dt)
    expect(got.shape[0] == 9, f"jerk length {got.shape[0]}")
    expect(bool(np.allclose(got, jerk_true, atol=1e-9)), f"jerk {got[0].tolist()}")
    stats = jerk_stats(pos, dt, ["x", "y", "z"], "probe", "m/s^3")
    expect(stats is not None and abs(stats.peak_l2 - float(np.linalg.norm(jerk_true))) < 1e-9, "peak l2")
    expect(stats is not None and abs(stats.rms_l2 - float(np.linalg.norm(jerk_true))) < 1e-9, "rms l2")
    expect(stats is not None and stats.worst_name == "x", f"worst {None if stats is None else stats.worst_name}")

    # 135×76 mm box, centre at the stated left-foot local centre, axis-aligned.
    hx, hy = 0.0675, 0.0380
    cx, cy = 0.030, 0.014
    corners = np.array(
        [
            [cx - hx, cy - hy],
            [cx + hx, cy - hy],
            [cx + hx, cy + hy],
            [cx - hx, cy + hy],
        ],
        dtype=np.float64,
    )
    hull = steer_walk.convex_hull_xy(corners)
    centre_margin = steer_walk.support_margin(np.array([cx, cy]), hull)
    expect(abs(centre_margin - hy) < 1e-9, f"centre margin {centre_margin}")
    outside_pt = np.array([cx, cy + hy + 0.004])
    outside_margin = steer_walk.support_margin(outside_pt, hull)
    expect(outside_margin < -0.0039, f"outside margin {outside_margin}")
    expect(decide(outside_margin, 1.0 / 950.0, False, True) == "Prefer FAIL", "one miss must fail")
    expect(decide(0.01, 0.0, False, True) == "inside", "clean bout is not forced to fail")
    expect(decide(0.01, 0.0, True, True) == "Prefer FAIL", "tip fails")
    expect(decide(0.01, 0.0, False, False) == "Prefer FAIL", "unmeasured fails")
    expect(SOFT_PASS is False, "soft-pass flag")
    expect(decide(-1e-4, 0.0, False, True) == "Prefer FAIL", "negative margin fails even if fraction was dropped")

    # A 999-inside / 1-outside fraction is still Prefer FAIL. No soft threshold.
    expect(decide(-0.006, 0.001, False, True) == "Prefer FAIL", "millimetre miss is not a pass")

    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    print("[smoothness] self-test ok; soft-pass off")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description="Score Day-1 kit walk jerk and ZMP margin")
    parser.add_argument("--self-test", action="store_true", help="Finite-difference and fail-rule checks")
    parser.add_argument("--json", type=Path, default=OUT_JSON)
    parser.add_argument("--md", type=Path, default=OUT_MD)
    args = parser.parse_args()
    if args.self_test:
        raise SystemExit(self_test())
    score = run_bout()
    write_outputs(score, args.json, args.md)
    print(score.honesty)
    print(f"[smoothness] wrote {args.json}")
    print(f"[smoothness] wrote {args.md}")
    if score.verdict == "Prefer FAIL":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
