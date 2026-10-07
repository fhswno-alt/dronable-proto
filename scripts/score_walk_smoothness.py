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
import importlib.util
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
# A tip worktree's scripts/ wins for the gait modules. step_bars stays on this branch.
_GAIT_SCRIPTS = os.environ.get("WALK_GAIT_SCRIPTS", "").strip()
if _GAIT_SCRIPTS:
    if _GAIT_SCRIPTS in sys.path:
        sys.path.remove(_GAIT_SCRIPTS)
    sys.path.insert(0, _GAIT_SCRIPTS)

import lipm_gait  # noqa: E402
import steer_walk  # noqa: E402
import step_bars  # noqa: E402

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


def _ground_gids(session: steer_walk.SteerSession, walker: lipm_gait.LipmWalker) -> tuple[int, ...]:
    """Floor geom, plus any runtime ground the walker already treats as floor."""
    gids = [int(walker.gid_floor)]
    for gid in getattr(walker, "ground_extra", ()):
        gi = int(gid)
        if gi >= 0 and gi not in gids:
            gids.append(gi)
    return tuple(gids)


def foot_floor_cop(
    model: mj.MjModel,
    data: mj.MjData,
    gid: int,
    floor_gid: int,
    grounds: tuple[int, ...] | None = None,
) -> tuple[Vec, float] | None:
    """World CoP and normal load. Same contact sum as ``LipmWalker._cop_local``."""
    allowed = {int(floor_gid)} if grounds is None else {int(g) for g in grounds if int(g) >= 0}
    num = np.zeros(3, dtype=np.float64)
    den = 0.0
    for i in range(data.ncon):
        con = data.contact[i]
        g1 = int(con.geom1)
        g2 = int(con.geom2)
        pair = (g1 == gid and g2 in allowed) or (g2 == gid and g1 in allowed)
        if not pair:
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
    grounds = _ground_gids(session, walker)
    num = np.zeros(3, dtype=np.float64)
    den = 0.0
    for side in ("L", "R"):
        got = foot_floor_cop(
            session.model, session.data, walker.gid[side], walker.gid_floor, grounds,
        )
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
    if step_bars.self_test() != 0:
        return 1
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
    stand_impact = 1048.9908045256434
    expect(
        not strictly_below(stand_impact, BASE_COM_JERK_PEAK, COM_JERK_TOL),
        "stand-impact CoM peak is not below the published baseline",
    )
    expect(strictly_below(32.662, BASE_COM_JERK_RMS, COM_JERK_TOL), "lower CoM RMS is below")
    expect(strictly_below(5857.0, BASE_JOINT_JERK_PEAK, JOINT_JERK_TOL), "lower joint peak is below")
    expect(not strictly_below(39843.130, BASE_JOINT_JERK_PEAK, JOINT_JERK_TOL), "baseline joint peak is not below")
    expect(abs(command_time(0.40, 1, 0.008) - 0.392) < 1e-12, "+1 tick latency is one tick late")
    expect(abs(command_time(0.392, -1, 0.008) - 0.400) < 1e-12, "-1 tick latency is one tick early")

    if failures:
        for msg in failures:
            print(f"FAIL {msg}")
        return 1
    print("[smoothness] self-test ok; soft-pass off")
    return 0


# Published #102 kit-bout figures. A rerun is below the baseline only when it
# clears the last reported digit. The stand impact that prints as 1048.991 is
# not below 1048.991.
BASE_COM_JERK_PEAK = 1048.991
BASE_COM_JERK_RMS = 132.473
BASE_JOINT_JERK_PEAK = 39843.0
BASE_JOINT_JERK_RMS = 4111.0
COM_JERK_TOL = 1e-3
JOINT_JERK_TOL = 0.5
TIP_SHA = "58ce1d861b85a2af3f4ca09f0eeead629baa40d1"
TIP_SHA_AC81435 = "ac814356f484735c442f825a2afb567ed65785d6"
ROW_JSON = ROOT / "previews" / "walk_smoothness_rerun.json"
ROW_MD = ROOT / "docs" / "WALK_SMOOTHNESS_RERUN.md"
AC_JSON = ROOT / "previews" / "walk_smoothness_ac81435.json"
AC_MD = ROOT / "docs" / "WALK_SMOOTHNESS_AC81435.md"
SEED_SIGMA_RAD = 0.002
STAGE_NAMES: tuple[str, ...] = ("stand", "start", "walk", "stop")
LEG_JOINTS: tuple[str, ...] = (
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
)


def strictly_below(value: float, published: float, tol: float) -> bool:
    """True when ``value`` is lower than the published baseline by more than ``tol``."""
    return value < published - tol


@dataclass(frozen=True)
class PreviewRowSpec:
    """One bout Controls actually ran in ``scripts/score_com_zmp.py``.

    Times are that script's ``--stand`` / ``--walk`` / ``--stop``. They were
    recovered by matching ``previews/com_zmp_preview_j.json`` and
    ``previews/com_zmp_preview_g.json`` on tip 58ce1d8, not by picking new ones.
    """

    name: str
    period_s: float
    dsp: float
    amp_m: float
    z_m: float
    arm_s: float
    vx_m_s: float
    stand_s: float
    walk_s: float
    stop_s: float
    source: str
    preview_shape: float = 1.0


# No vx 0.056 preview row is on this tip. ``voice_caller.py`` still publishes
# the old bus phrase vel(+0.056, 0). It is not a period/dsp/swing preview bout,
# and ``score_com_zmp.py`` has no committed 0.056 tag.
PREVIEW_ROWS: tuple[PreviewRowSpec, ...] = (
    PreviewRowSpec(
        name="slow",
        period_s=6.40,
        dsp=0.70,
        amp_m=0.043,
        z_m=0.004,
        arm_s=2.40,
        vx_m_s=0.040,
        stand_s=0.40,
        walk_s=11.00,
        stop_s=3.00,
        source=(
            "scripts/score_com_zmp.py --period 6.40 --dsp 0.70 --amp 0.043 "
            "--z 0.004 --arm 2.40 --stand 0.40 --walk 11.00 --stop 3.00 "
            "--vx 0.040 --tag j"
        ),
    ),
    PreviewRowSpec(
        name="kit-vx",
        period_s=2.80,
        dsp=0.55,
        amp_m=0.043,
        z_m=0.008,
        arm_s=1.20,
        vx_m_s=0.150,
        stand_s=0.40,
        walk_s=5.50,
        stop_s=2.20,
        source=(
            "scripts/score_com_zmp.py --period 2.80 --dsp 0.55 --amp 0.043 "
            "--z 0.008 --arm 1.20 --stand 0.40 --walk 5.50 --stop 2.20 "
            "--vx 0.150 --tag g"
        ),
    ),
)


@dataclass(frozen=True)
class PostedRow:
    com_min_mm: float
    zmp_min_mm: float
    ask_nm: float
    ask_joint: str
    ask_t_s: float
    com_jerk_peak: float
    com_jerk_rms: float
    com_jerk_post_peak: float
    joint_name: str
    joint_peak: float
    joint_rms: float
    knee_peak: float
    knee_rms: float


POSTED: dict[str, PostedRow] = {
    "slow": PostedRow(
        com_min_mm=19.91223536343605,
        zmp_min_mm=19.955007818544413,
        ask_nm=1.9791926809382394,
        ask_joint="r_hip_roll",
        ask_t_s=2.816,
        com_jerk_peak=1048.9908045256434,
        com_jerk_rms=32.662423300976606,
        com_jerk_post_peak=538.6054460859632,
        joint_name="r_ank_pitch",
        joint_peak=5857.085122019196,
        joint_rms=239.0842692062063,
        knee_peak=2477.5711205517537,
        knee_rms=127.99933277827915,
    ),
    "kit-vx": PostedRow(
        com_min_mm=18.231587903077374,
        zmp_min_mm=18.231587903077374,
        ask_nm=4.446196517997931,
        ask_joint="l_hip_pitch",
        ask_t_s=4.424,
        com_jerk_peak=1048.9908045256434,
        com_jerk_rms=48.195127551922646,
        com_jerk_post_peak=520.3025034247216,
        joint_name="r_ank_pitch",
        joint_peak=5857.085122019196,
        joint_rms=399.7898894858721,
        knee_peak=2653.9141831771053,
        knee_rms=311.2173820136163,
    ),
}


# Voice rows on tip ac81435. Timings are the vendor log's stand 0.40 / walk
# 11.00 / stop 3.00 (1800 ticks), the same clock as tag ``l`` / ``p`` / ``s``.
# The 58ce1d8 ``PREVIEW_ROWS`` above stay the older tip.
AC81435_ROWS: tuple[PreviewRowSpec, ...] = (
    PreviewRowSpec(
        name="voice-3.60",
        period_s=3.60,
        dsp=0.70,
        amp_m=0.043,
        z_m=0.004,
        arm_s=2.40,
        vx_m_s=0.056,
        stand_s=0.40,
        walk_s=11.00,
        stop_s=3.00,
        source=(
            "previews/com_zmp_preview_s.json period 3.60 dsp 0.70 amp 0.043 "
            "z 0.004 arm 2.40 vx 0.056 stand 0.40 walk 11.00 stop 3.00"
        ),
    ),
    PreviewRowSpec(
        name="voice-3.70",
        period_s=3.70,
        dsp=0.70,
        amp_m=0.043,
        z_m=0.004,
        arm_s=2.40,
        vx_m_s=0.056,
        stand_s=0.40,
        walk_s=11.00,
        stop_s=3.00,
        source=(
            "previews/com_zmp_preview_p.json period 3.70 dsp 0.70 amp 0.043 "
            "z 0.004 arm 2.40 vx 0.056 stand 0.40 walk 11.00 stop 3.00"
        ),
    ),
    PreviewRowSpec(
        name="slow",
        period_s=6.40,
        dsp=0.70,
        amp_m=0.043,
        z_m=0.004,
        arm_s=2.40,
        vx_m_s=0.040,
        stand_s=0.40,
        walk_s=11.00,
        stop_s=3.00,
        source=(
            "previews/com_zmp_preview_l.json period 6.40 dsp 0.70 amp 0.043 "
            "z 0.004 arm 2.40 vx 0.040 stand 0.40 walk 11.00 stop 3.00"
        ),
    ),
)


@dataclass(frozen=True)
class AcClaim:
    """Controls' posted figures for one ac81435 row. Their margin is cart-table."""

    margin_mm: float
    ask_nm: float
    ask_joint: str
    ask_t_s: float
    com_jerk_peak: float
    com_jerk_rms: float
    knee_peak: float
    knee_rms: float
    stop_margin_mm: float
    stop_ask_nm: float
    stop_ask_joint: str
    stop_ask_t_s: float
    joint_name: str
    joint_peak: float
    joint_rms: float


AC81435_CLAIMS: dict[str, AcClaim] = {
    "voice-3.60": AcClaim(
        margin_mm=21.380818889018192,
        ask_nm=2.313474049378506,
        ask_joint="r_hip_roll",
        ask_t_s=8.176,
        com_jerk_peak=395.30944711258593,
        com_jerk_rms=12.014751454517079,
        knee_peak=1250.4078238076595,
        knee_rms=73.14132832050701,
        stop_margin_mm=24.824043203363395,
        stop_ask_nm=2.2712777994990185,
        stop_ask_joint="r_ank_pitch",
        stop_ask_t_s=14.392,
        joint_name="r_ank_pitch",
        joint_peak=8367.011450050986,
        joint_rms=222.3453170100869,
    ),
    "voice-3.70": AcClaim(
        margin_mm=21.417031134190564,
        ask_nm=2.261872523867808,
        ask_joint="r_hip_roll",
        ask_t_s=8.352,
        com_jerk_peak=88.85277150719413,
        com_jerk_rms=5.353261716101873,
        knee_peak=1250.4078238076595,
        knee_rms=71.36159608040991,
        stop_margin_mm=47.71525680297464,
        stop_ask_nm=1.5982087191771084,
        stop_ask_joint="l_knee",
        stop_ask_t_s=11.400,
        joint_name="r_knee",
        joint_peak=1250.4078238076595,
        joint_rms=71.36159608040991,
    ),
    "slow": AcClaim(
        margin_mm=21.637693658216682,
        ask_nm=1.995631587118884,
        ask_joint="r_hip_roll",
        ask_t_s=2.816,
        com_jerk_peak=88.85277150719413,
        com_jerk_rms=3.7223634276158113,
        knee_peak=1250.4078238076595,
        knee_rms=55.487486327744406,
        stop_margin_mm=26.858296619237212,
        stop_ask_nm=0.82001705351437,
        stop_ask_joint="r_ank_roll",
        stop_ask_t_s=14.392,
        joint_name="r_knee",
        joint_peak=1250.4078238076595,
        joint_rms=55.487486327744406,
    ),
}


@dataclass(frozen=True)
class Perturb:
    """Runtime-only change on the loaded MjModel. The plant XML is not written."""

    label: str
    seed: int | None = None
    mass_scale: float = 1.0
    friction: float | None = None
    latency_ticks: int = 0
    rug: bool = False
    cycles: int = 0


@dataclass
class AskSample:
    t_s: float
    joint: str
    signed_nm: float
    sum_nm: float
    stage: str = ""


@dataclass
class PhaseScore:
    phase: str
    n: int = 0
    n_zmp_outside: int = 0
    n_zmp_margin: int = 0
    zmp_min_m: float | None = None
    zmp_min_t_s: float | None = None
    n_com: int = 0
    n_com_outside: int = 0
    com_min_m: float | None = None
    com_min_t_s: float | None = None

    def add(self, tick: ZmpTick, t_s: float) -> None:
        self.n += 1
        if tick.outside:
            self.n_zmp_outside += 1
        if tick.margin_m is not None:
            self.n_zmp_margin += 1
            if self.zmp_min_m is None or tick.margin_m < self.zmp_min_m:
                self.zmp_min_m = tick.margin_m
                self.zmp_min_t_s = t_s
        if tick.com_margin_m is not None:
            self.n_com += 1
            if tick.com_margin_m < 0.0:
                self.n_com_outside += 1
            if self.com_min_m is None or tick.com_margin_m < self.com_min_m:
                self.com_min_m = tick.com_margin_m
                self.com_min_t_s = t_s

    def zmp_frac(self) -> float:
        if self.n <= 0:
            return 1.0
        return self.n_zmp_outside / self.n

    def com_frac(self) -> float:
        if self.n_com <= 0:
            return 1.0
        return self.n_com_outside / self.n_com


def _install_ask_log(walker: lipm_gait.LipmWalker, bucket: list[AskSample]) -> None:
    """Log |kp*(q_des−q)| + |kv*ω| before ``write_clipped`` slews ctrl.

    This is the unclamped ask Controls posted. The hook does not change the
    command that is written.
    """
    orig = walker.write_clipped

    def wrapped(jn: str, q_des: float) -> None:
        if jn in LEG_JOINTS:
            act = jn + "_pos"
            idx = walker.act_idx.get(act)
            if idx is not None:
                q = float(walker.q(jn))
                jid = mj.mj_name2id(walker.model, mj.mjtObj.mjOBJ_JOINT, jn)
                omega = float(walker.data.qvel[int(walker.model.jnt_dofadr[jid])])
                kp = float(walker.model.actuator_gainprm[idx, 0])
                kv = -float(walker.model.actuator_biasprm[idx, 2])
                kp_term = kp * (float(q_des) - q)
                signed = kp_term - kv * omega
                total = abs(kp_term) + abs(kv * omega)
                bucket.append(AskSample(
                    t_s=float(walker.data.time),
                    joint=jn,
                    signed_nm=float(signed),
                    sum_nm=float(total),
                ))
        orig(jn, q_des)

    walker.write_clipped = wrapped  # type: ignore[method-assign]


def _preview_config(spec: PreviewRowSpec) -> lipm_gait.LipmConfig:
    kwargs: dict[str, object] = {
        "name": "preview",
        "clear_m": spec.z_m,
        "arms": True,
        "schedule": "gait_manager",
        "gm_period_s": spec.period_s,
        "gm_dsp": spec.dsp,
        "gm_y_swap_m": 0.0,
        "gm_x_m": 0.020,
        "gm_z_m": spec.z_m,
        "gm_z_swap_m": 0.0,
        "gm_pelvis_deg": 0.0,
        "gm_hip_pitch_deg": 15.0,
        "gm_start_lead": "L",
        "gm_crouch_m": 0.025,
        "gm_move_s": 0.020,
        "preview_amp_m": spec.amp_m,
        "preview_arm_s": spec.arm_s,
    }
    fields = getattr(lipm_gait.LipmConfig, "__dataclass_fields__", {})
    if "preview_shape" in fields:
        kwargs["preview_shape"] = float(spec.preview_shape)
    if "preview_r" in fields:
        kwargs["preview_r"] = 1.0e-4
    return lipm_gait.LipmConfig(**kwargs)  # type: ignore[arg-type]


def command_time(now: float, latency_ticks: int, dt: float) -> float:
    """Clock the bus command is chosen on. The bus itself still sees ``now``."""
    return now - latency_ticks * dt


def _masked_jerk(samples: Vec, dt: float, keep: Vec) -> tuple[float, float] | None:
    jerk = finite_jerk(samples, dt)
    if jerk.shape[0] == 0 or keep.shape[0] != samples.shape[0]:
        return None
    chosen = jerk[keep[3:]]
    if chosen.shape[0] == 0:
        return None
    norms = _l2_rows(chosen)
    return float(np.max(norms)), float(np.sqrt(np.mean(np.square(norms))))


def _mfg_reasons(
    *,
    measured: bool,
    zmp_min_m: float | None,
    zmp_frac: float,
    com_min_m: float | None,
    com_frac: float,
    tipped: bool,
    ask_nm: float,
    ask_over_ticks: int,
    com_peak: float | None,
    com_rms: float | None,
    joint_peak: float | None,
    joint_rms: float | None,
) -> list[str]:
    reasons: list[str] = []
    if not measured:
        reasons.append("bout unmeasured or plant md5 moved")
    if zmp_min_m is None or zmp_min_m < 0.0:
        reasons.append(f"ZMP margin { _metres(zmp_min_m) } is not ≥ 0")
    if zmp_frac > 0.0:
        reasons.append(f"ZMP outside fraction {zmp_frac:.3f} is not 0")
    if com_min_m is None or com_min_m < 0.0:
        reasons.append(f"CoM margin {_metres(com_min_m)} is not ≥ 0")
    if com_frac > 0.0:
        reasons.append(f"CoM outside fraction {com_frac:.3f} is not 0")
    if tipped:
        reasons.append("tipped or bus fault")
    if ask_nm > SAG_BAR_NM + 1e-9 or ask_over_ticks > 0:
        reasons.append(
            f"unclamped ask {ask_nm:.3f} Nm, {ask_over_ticks} ticks over {SAG_BAR_NM:.2f}"
        )
    if com_peak is None or not strictly_below(com_peak, BASE_COM_JERK_PEAK, COM_JERK_TOL):
        reasons.append(
            f"whole-bout CoM jerk peak {com_peak} is not below {BASE_COM_JERK_PEAK} "
            "(same stand impact; post-stand is not the gate)"
        )
    if com_rms is None or not strictly_below(com_rms, BASE_COM_JERK_RMS, COM_JERK_TOL):
        reasons.append(
            f"whole-bout CoM jerk RMS {com_rms} is not below {BASE_COM_JERK_RMS}"
        )
    if joint_peak is None or not strictly_below(joint_peak, BASE_JOINT_JERK_PEAK, JOINT_JERK_TOL):
        reasons.append(
            f"whole-bout joint jerk peak {joint_peak} is not below {BASE_JOINT_JERK_PEAK}"
        )
    if joint_rms is None or not strictly_below(joint_rms, BASE_JOINT_JERK_RMS, JOINT_JERK_TOL):
        reasons.append(
            f"whole-bout joint jerk RMS {joint_rms} is not below {BASE_JOINT_JERK_RMS}"
        )
    return reasons


def _apply_perturb(session: steer_walk.SteerSession, perturb: Perturb) -> None:
    """Scale mass, friction, or the spawn pose on the loaded model.

    Mass and inertia are scaled together so a rigid body stays consistent.
    Sliding friction is geom_friction column 0 on the floor and both foot
    boxes. A seed adds Gaussian noise to the twelve leg hinges only.
    ``mj_setConst`` refreshes derived mass fields. The XML file is not opened
    for writing.
    """
    model = session.model
    data = session.data
    touched = False
    if abs(perturb.mass_scale - 1.0) > 1e-12:
        model.body_mass[:] *= float(perturb.mass_scale)
        model.body_inertia[:] *= float(perturb.mass_scale)
        # mj_setConst refreshes invweight from the new mass, and it also
        # writes qpos back to qpos0. Keep the seated stand pose.
        qpos = data.qpos.copy()
        qvel = data.qvel.copy()
        ctrl = data.ctrl.copy()
        act = data.act.copy()
        time_s = float(data.time)
        mj.mj_setConst(model, data)
        data.qpos[:] = qpos
        data.qvel[:] = qvel
        data.ctrl[:] = ctrl
        data.act[:] = act
        data.time = time_s
        touched = True
    if perturb.friction is not None:
        for gid in (int(session.gid_floor), int(session.gid_lfoot), int(session.gid_rfoot)):
            if gid < 0:
                raise RuntimeError("floor or foot contact geom is missing")
            model.geom_friction[gid, 0] = float(perturb.friction)
        touched = True
    if perturb.seed is not None:
        rng = np.random.Generator(np.random.PCG64(int(perturb.seed)))
        for jn in LEG_JOINTS:
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)
            if jid < 0:
                raise RuntimeError(f"missing joint {jn}")
            adr = int(model.jnt_qposadr[jid])
            data.qpos[adr] += float(rng.normal(0.0, SEED_SIGMA_RAD))
        touched = True
    if touched:
        mj.mj_forward(model, data)


def _asks_by_joint(asks: list[AskSample]) -> list[dict[str, object]]:
    best: dict[str, AskSample] = {}
    for sample in asks:
        prev = best.get(sample.joint)
        if prev is None or sample.sum_nm > prev.sum_nm:
            best[sample.joint] = sample
    rows: list[dict[str, object]] = []
    for sample in best.values():
        rows.append({
            "joint": sample.joint,
            "sum_nm": sample.sum_nm,
            "t_s": sample.t_s,
            "signed_nm": sample.signed_nm,
            "headroom_nm": SAG_BAR_NM - sample.sum_nm,
            "stage": sample.stage,
        })
    rows.sort(key=lambda row: float(row["sum_nm"]), reverse=True)
    return rows


def _stage_blob(score: PhaseScore, asks: list[AskSample]) -> dict[str, object]:
    picked = [sample for sample in asks if sample.stage == score.phase]
    peak = max(picked, key=lambda sample: sample.sum_nm) if picked else None
    return {
        "stage": score.phase,
        "n": score.n,
        "zmp_min_margin_m": score.zmp_min_m,
        "zmp_min_t_s": score.zmp_min_t_s,
        "zmp_outside_fraction": score.zmp_frac() if score.n else None,
        "zmp_outside_n": score.n_zmp_outside,
        "com_min_margin_m": score.com_min_m,
        "com_min_t_s": score.com_min_t_s,
        "com_outside_fraction": score.com_frac() if score.n_com else None,
        "com_n": score.n_com,
        "com_outside_n": score.n_com_outside,
        "ask_joint": None if peak is None else peak.joint,
        "ask_nm": None if peak is None else peak.sum_nm,
        "ask_t_s": None if peak is None else peak.t_s,
        "ask_signed_nm": None if peak is None else peak.signed_nm,
        "ask_headroom_nm": None if peak is None else SAG_BAR_NM - peak.sum_nm,
    }


RUG_EDGE_AHEAD_M = 0.015
STEP_MD = ROOT / "docs" / "WALK_STEPPING_BARS.md"
TIP_SHA_D7 = "d7b06e79757a6394784ab3582e9c34c1b8c2ab2b"


def _contact_count(
    data: mj.MjData,
    foot_gid: int,
    grounds: tuple[int, ...],
) -> int:
    allowed = {int(g) for g in grounds}
    n = 0
    for i in range(data.ncon):
        con = data.contact[i]
        g1 = int(con.geom1)
        g2 = int(con.geom2)
        if (g1 == foot_gid and g2 in allowed) or (g2 == foot_gid and g1 in allowed):
            n += 1
    return n


def _foot_box_sample(
    model: mj.MjModel,
    data: mj.MjData,
    gid: int,
    rug: tuple[float, float, float, float, float] | None,
) -> tuple[float, float, float, float, float]:
    """Box-centre x/y, lowest-corner clearance, and sole roll/pitch."""
    half = np.asarray(model.geom_size[gid], dtype=np.float64)
    if hasattr(data, "geom_xmat"):
        rot = np.asarray(data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
        origin = np.asarray(data.geom_xpos[gid], dtype=np.float64)
    else:
        bid = int(model.geom_bodyid[gid])
        rot = np.asarray(data.xmat[bid], dtype=np.float64).reshape(3, 3)
        origin = np.asarray(data.xpos[bid], dtype=np.float64) + rot @ np.asarray(
            model.geom_pos[gid], dtype=np.float64,
        )
    clear = 1.0e9
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                local = np.array([sx * half[0], sy * half[1], sz * half[2]], dtype=np.float64)
                world = origin + rot @ local
                ground = 0.0
                if rug is not None:
                    x0, x1, y0, y1, top = rug
                    if x0 <= float(world[0]) <= x1 and y0 <= float(world[1]) <= y1:
                        ground = top
                clear = min(clear, float(world[2]) - ground)
    normal = rot[:, 2]
    roll = math.atan2(float(normal[1]), float(normal[2]))
    pitch = math.atan2(-float(normal[0]), math.hypot(float(normal[1]), float(normal[2])))
    return float(origin[0]), float(origin[1]), float(clear), roll, pitch


def _foot_cop_margin(
    session: steer_walk.SteerSession,
    walker: lipm_gait.LipmWalker,
    side: str,
    grounds: tuple[int, ...],
) -> float:
    got = foot_floor_cop(session.model, session.data, walker.gid[side], walker.gid_floor, grounds)
    if got is None:
        return float("nan")
    world, _fn = got
    corners = session._foot_corners(walker.bid[side], walker.gid[side])
    hull = steer_walk.convex_hull_xy(corners)
    return float(steer_walk.support_margin(world[:2], hull))


def _actual_margins(
    session: steer_walk.SteerSession,
    walker: lipm_gait.LipmWalker,
    grounds: tuple[int, ...],
    n_l: int,
    n_r: int,
) -> tuple[float, float]:
    """Contact CoP and CoM against the boxes of feet that have a floor contact."""
    sides = [side for side, n in (("L", n_l), ("R", n_r)) if n > 0]
    if not sides:
        return float("nan"), float("nan")
    num = np.zeros(3, dtype=np.float64)
    den = 0.0
    for side in sides:
        got = foot_floor_cop(
            session.model, session.data, walker.gid[side], walker.gid_floor, grounds,
        )
        if got is None:
            continue
        world, fn = got
        num += fn * world
        den += fn
    chunks = [
        session._foot_corners(walker.bid[side], walker.gid[side])
        for side in sides
    ]
    hull = steer_walk.convex_hull_xy(np.concatenate(chunks, axis=0))
    com = np.asarray(session.data.subtree_com[walker.bid_body, :2], dtype=np.float64)
    com_m = float(steer_walk.support_margin(com, hull))
    if den <= 1e-6:
        return float("nan"), com_m
    zmp = (num / den)[:2]
    return float(steer_walk.support_margin(zmp, hull)), com_m


def _trunk_angles(data: mj.MjData, bid: int) -> tuple[float, float, float]:
    up = np.asarray(data.xmat[bid], dtype=np.float64).reshape(3, 3)[:, 2]
    roll = math.atan2(float(up[1]), float(up[2]))
    pitch = math.atan2(-float(up[0]), math.hypot(float(up[1]), float(up[2])))
    return roll, pitch, float(up[2])


def _leg_q(model: mj.MjModel, data: mj.MjData) -> list[float]:
    out: list[float] = []
    for name in LEG_JOINTS:
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f"missing joint {name}")
        out.append(float(data.qpos[int(model.jnt_qposadr[jid])]))
    return out


def _place_entrance_rug(session: steer_walk.SteerSession) -> tuple[float, float, float, float, float]:
    """Slide the entrance mat so its near edge is 15 mm ahead of the toes.

    Body position only. The plant file is not written. The geom stays ground
    for contact once the walker is told about it.
    """
    gid = int(getattr(session, "gid_rug", -1))
    if gid < 0:
        gid = mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_GEOM, "mat_rug")
    if gid < 0:
        raise RuntimeError("entrance rug geom is not in this scene")
    session.gid_rug = gid
    bid = int(session.model.geom_bodyid[gid])
    half_x = float(session.model.geom_size[gid, 0])
    half_y = float(session.model.geom_size[gid, 1])
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
    walker = session.lipm
    if walker is not None and hasattr(walker, "ground_extra"):
        walker.ground_extra = (gid,)
    center = np.asarray(session.data.geom_xpos[gid], dtype=np.float64)
    top = float(center[2]) + half_z
    return (
        float(center[0]) - half_x,
        float(center[0]) + half_x,
        float(center[1]) - half_y,
        float(center[1]) + half_y,
        top,
    )


def _cycle_script(spec: PreviewRowSpec, n_cycles: int) -> tuple[steer_walk.DemoSegment, ...]:
    """Repeat the row's vel window. Each cycle is the same walk and the same stop."""
    t = float(spec.stand_s)
    segments = [steer_walk.DemoSegment(t, "stand", 0.0, 0.0, "stand")]
    vx = float(spec.vx_m_s)
    for i in range(int(n_cycles)):
        t += float(spec.walk_s)
        segments.append(steer_walk.DemoSegment(t, "vel", vx, 0.0, f"forward{i}"))
        t += float(spec.stop_s)
        segments.append(steer_walk.DemoSegment(t, "stop", 0.0, 0.0, f"stop{i}"))
    return tuple(segments)


def _command_mode(now: float, spec: PreviewRowSpec, driver: steer_walk.ScriptedDriver | None) -> str:
    if driver is not None:
        kind = driver.segment(now).kind
        return "move" if kind == "vel" else str(kind)
    if now + 1e-12 < spec.stand_s:
        return "stand"
    if now + 1e-12 < spec.stand_s + spec.walk_s:
        return "move"
    return "stop"


def run_preview_row(
    spec: PreviewRowSpec,
    perturb: Perturb | None = None,
    tip_sha: str = TIP_SHA,
) -> dict[str, object]:
    digest_before = _plant_md5()
    if digest_before != FROZEN_MD5:
        raise SystemExit(f"plant md5 {digest_before} != {FROZEN_MD5}")
    scene = None
    if perturb is not None and perturb.rug:
        scene = steer_walk.ROOT / "mujoco" / "room_entrance.xml"
        if not scene.is_file():
            raise RuntimeError(f"{spec.name}: entrance rug scene is not on this tip")
    session = steer_walk.SteerSession(video=False, lipm=_preview_config(spec), scene_xml=scene)
    walker = session.lipm
    if walker is None or walker.op3 is None:
        raise RuntimeError(f"{spec.name}: walker did not build")
    if walker.op3.y_swap_cmd != 0.0:
        raise RuntimeError(f"{spec.name}: y_swap_cmd is not 0")
    notes = plant_notes(session.model)
    asks: list[AskSample] = []
    _install_ask_log(walker, asks)
    if perturb is not None:
        _apply_perturb(session, perturb)
    rug_box = _place_entrance_rug(session) if perturb is not None and perturb.rug else None
    if _plant_md5() != FROZEN_MD5:
        raise SystemExit("plant XML md5 changed while applying a runtime perturb")
    phases: dict[str, PhaseScore] = {
        name: PhaseScore(phase=name) for name in ("stand", "ds", "ss_L", "ss_R", "unknown")
    }
    stages: dict[str, PhaseScore] = {
        name: PhaseScore(phase=name) for name in STAGE_NAMES
    }
    times: list[float] = []
    pre_times: list[float] = []
    com_rows: list[Vec] = []
    q_rows: list[Vec] = []
    joint_names: list[str] = []
    reasons: list[str] = []
    step_cols: dict[str, list[object]] = {key: [] for key in (
        "t", "x_l", "x_r", "y_l", "y_r", "n_l", "n_r", "z_l", "z_r",
        "declared", "mode", "roll", "pitch", "roll_l", "pitch_l", "roll_r", "pitch_r",
        "up_z", "zmp_act", "com_act", "cop_l", "cop_r",
    )}
    q_stand_rows: list[list[float]] = []
    t_stop = spec.stand_s + spec.walk_s
    t_end = t_stop + spec.stop_s
    driver: steer_walk.ScriptedDriver | None = None
    if perturb is not None and int(perturb.cycles) > 0:
        driver = steer_walk.ScriptedDriver(_cycle_script(spec, int(perturb.cycles)))
        t_end = float(driver.segments[-1].t_end)
    vx_cmd = spec.vx_m_s if spec.vx_m_s > 0.0 else float(steer_walk.VX_FWD_CAP)
    latency_ticks = 0 if perturb is None else int(perturb.latency_ticks)
    ctrl_dt = float(session.ctrl_dt)
    last_send = -1.0
    stop_sent = False
    while float(session.data.time) < t_end - 1e-12:
        now = float(session.data.time)
        # Positive latency delivers the command one tick late. Negative
        # latency delivers it one tick early. The bus timeout still sees the
        # real clock.
        cmd_t = command_time(now, latency_ticks, ctrl_dt)
        mode = _command_mode(now, spec, driver)
        if driver is not None:
            driver.publish(session.bus, now)
        elif cmd_t + 1e-12 < spec.stand_s:
            pass
        elif cmd_t + 1e-12 < t_stop:
            if last_send < 0.0 or (now - last_send) >= (steer_walk.VEL_RESEND_S - 1e-12):
                session.bus.vel(vx_cmd, 0.0, now)
                last_send = now
        elif not stop_sent:
            session.bus.stop(now)
            stop_sent = True
        n_ask = len(asks)
        session.step()
        stage_name = str(getattr(walker, "preview_stage", "unknown"))
        for sample in asks[n_ask:]:
            sample.stage = stage_name
        if walker.op3.y_swap_cmd != 0.0:
            raise RuntimeError(f"{spec.name}: y_swap_cmd changed")
        if session.bus.fault and session.bus.fault_reason and session.bus.fault_reason not in reasons:
            reasons.append(session.bus.fault_reason)
        tick = sample_zmp(session, walker)
        t_post = float(session.data.time)
        phases[tick.phase].add(tick, t_post)
        if stage_name in stages:
            stages[stage_name].add(tick, t_post)
        names, q = actuated_q(session.model, session.data)
        if not joint_names:
            joint_names = names
        com_rows.append(np.asarray(session.data.subtree_com[walker.bid_body], dtype=np.float64).copy())
        q_rows.append(q)
        times.append(t_post)
        pre_times.append(now)
        grounds = _ground_gids(session, walker)
        n_l = _contact_count(session.data, int(walker.gid["L"]), grounds)
        n_r = _contact_count(session.data, int(walker.gid["R"]), grounds)
        x_l, y_l, z_l, roll_l, pitch_l = _foot_box_sample(
            session.model, session.data, int(walker.gid["L"]), rug_box,
        )
        x_r, y_r, z_r, roll_r, pitch_r = _foot_box_sample(
            session.model, session.data, int(walker.gid["R"]), rug_box,
        )
        roll, pitch, up_z = _trunk_angles(session.data, int(walker.bid_body))
        zmp_act, com_act = _actual_margins(session, walker, grounds, n_l, n_r)
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
        step_cols["cop_l"].append(_foot_cop_margin(session, walker, "L", grounds))
        step_cols["cop_r"].append(_foot_cop_margin(session, walker, "R", grounds))
        q_stand_rows.append(_leg_q(session.model, session.data))
        if session.bus.fault:
            break
    session.assert_plant_unchanged()
    digest_after = _plant_md5()
    if digest_after != digest_before:
        notes.append(f"plant md5 changed during the bout to {digest_after}")
    n = len(times)
    dt = _uniform_dt(np.asarray(times, dtype=np.float64)) if n else None
    com_arr = np.stack(com_rows, axis=0) if com_rows else np.zeros((0, 3), dtype=np.float64)
    q_arr = np.stack(q_rows, axis=0) if q_rows else np.zeros((0, 0), dtype=np.float64)
    com_jerk = jerk_stats(com_arr, dt, ["x", "y", "z"], "body_link subtree_com world", "m/s^3") if dt is not None else None
    joint_jerk = jerk_stats(q_arr, dt, joint_names, "actuated hinge qpos", "rad/s^3") if dt is not None and joint_names else None
    post_peak: float | None = None
    post_rms: float | None = None
    if dt is not None and n:
        keep = np.asarray(pre_times, dtype=np.float64) + 1e-12 >= spec.stand_s
        masked = _masked_jerk(com_arr, dt, keep)
        if masked is not None:
            post_peak, post_rms = masked
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
    ask_peak = max(asks, key=lambda row: row.sum_nm) if asks else None
    over_times = {row.t_s for row in asks if row.sum_nm > SAG_BAR_NM + 1e-9}
    ask_nm = 0.0 if ask_peak is None else ask_peak.sum_nm
    tipped = bool(session.min_up_z < TIP_UP_Z or reasons)
    measured = (
        n > 0
        and zmp_min is not None
        and com_min is not None
        and com_jerk is not None
        and joint_jerk is not None
        and digest_after == FROZEN_MD5
        and not any("!=" in note and note.startswith("plant md5") for note in notes)
    )
    q_mat = np.asarray(q_stand_rows, dtype=np.float64) if q_stand_rows else np.zeros((0, 12))
    q_err = np.zeros(q_mat.shape[0], dtype=np.float64)
    if q_mat.shape[0] > 0:
        t_arr = np.asarray(step_cols["t"], dtype=np.float64)
        stand_mask = t_arr <= spec.stand_s + 1e-12
        if not np.any(stand_mask):
            stand_mask = np.zeros(q_mat.shape[0], dtype=bool)
            stand_mask[0] = True
        q_ref = np.median(q_mat[stand_mask], axis=0)
        q_err = np.max(np.abs(q_mat - q_ref), axis=1)
    stepping = step_bars.summarize_trace(
        step_bars.as_arrays(step_cols),
        vx_m_s=vx_cmd,
        period_s=spec.period_s,
        q_stand_err=q_err,
    )
    half_x = float(session.model.geom_size[int(walker.gid["L"]), 0])
    stepping["box_half_x_m"] = half_x
    if abs(half_x - 0.0675) > 1e-6:
        stepping["fail_reasons"].append(
            f"contact box half-length {half_x * 1000.0:.2f} mm is not 67.5 mm"
        )
        stepping["passes"] = False
    fail_reasons = _mfg_reasons(
        measured=measured,
        zmp_min_m=zmp_min,
        zmp_frac=zmp_frac,
        com_min_m=com_min,
        com_frac=com_frac,
        tipped=tipped,
        ask_nm=ask_nm,
        ask_over_ticks=len(over_times),
        com_peak=None if com_jerk is None else com_jerk.peak_l2,
        com_rms=None if com_jerk is None else com_jerk.rms_l2,
        joint_peak=None if joint_jerk is None else joint_jerk.peak_l2,
        joint_rms=None if joint_jerk is None else joint_jerk.rms_l2,
    )
    step_reasons = stepping.get("fail_reasons")
    if isinstance(step_reasons, list):
        fail_reasons.extend(str(reason) for reason in step_reasons)
    verdict: Literal["CLEAR", "Prefer FAIL"] = "CLEAR" if not fail_reasons else "Prefer FAIL"
    knee_peak = None if joint_jerk is None else joint_jerk.axis_peak.get("r_knee")
    knee_rms = None if joint_jerk is None else joint_jerk.axis_rms.get("r_knee")
    phase_rows = [phases[name] for name in ("stand", "ds", "ss_L", "ss_R")]
    joint_asks = _asks_by_joint(asks)
    stage_rows = [_stage_blob(stages[name], asks) for name in STAGE_NAMES]
    floor_mu = float(session.model.geom_friction[int(session.gid_floor), 0])
    return {
        "name": spec.name,
        "source": spec.source,
        "tip_sha": tip_sha,
        "perturb_label": None if perturb is None else perturb.label,
        "seed": None if perturb is None else perturb.seed,
        "mass_scale": 1.0 if perturb is None else perturb.mass_scale,
        "latency_ticks": latency_ticks,
        "friction_sliding": floor_mu,
        "runtime_mass_kg": float(np.sum(session.model.body_mass)),
        "plant_md5_before": digest_before,
        "plant_md5": digest_after,
        "soft_pass": SOFT_PASS,
        "verdict": verdict,
        "fail_reasons": fail_reasons,
        "plant_notes": notes,
        "period_s": spec.period_s,
        "dsp": spec.dsp,
        "z_m": spec.z_m,
        "arm_s": spec.arm_s,
        "amp_m": spec.amp_m,
        "vx_m_s": vx_cmd,
        "stand_s": spec.stand_s,
        "walk_s": spec.walk_s,
        "stop_s": spec.stop_s,
        "n_samples": n,
        "ctrl_dt_s": float(session.ctrl_dt),
        "min_up_z": float(session.min_up_z),
        "tipped": tipped,
        "fault_reasons": reasons,
        "zmp_min_margin_m": zmp_min,
        "zmp_outside_fraction": zmp_frac,
        "com_min_margin_m": com_min,
        "com_outside_fraction": com_frac,
        "phases": [
            {
                "phase": row.phase,
                "n": row.n,
                "zmp_min_margin_m": row.zmp_min_m,
                "zmp_min_t_s": row.zmp_min_t_s,
                "zmp_outside_fraction": row.zmp_frac() if row.n else None,
                "zmp_outside_n": row.n_zmp_outside,
                "com_min_margin_m": row.com_min_m,
                "com_min_t_s": row.com_min_t_s,
                "com_outside_fraction": row.com_frac() if row.n_com else None,
                "com_n": row.n_com,
                "com_outside_n": row.n_com_outside,
            }
            for row in phase_rows
        ],
        "com_jerk_whole_peak": None if com_jerk is None else com_jerk.peak_l2,
        "com_jerk_whole_rms": None if com_jerk is None else com_jerk.rms_l2,
        "com_jerk_post_stand_peak": post_peak,
        "com_jerk_post_stand_rms": post_rms,
        "post_stand_is_gate": False,
        "joint_jerk_peak": None if joint_jerk is None else joint_jerk.peak_l2,
        "joint_jerk_rms": None if joint_jerk is None else joint_jerk.rms_l2,
        "worst_joint": None if joint_jerk is None else joint_jerk.worst_name,
        "worst_joint_peak": None if joint_jerk is None else joint_jerk.worst_peak,
        "worst_joint_rms": None if joint_jerk is None else joint_jerk.axis_rms.get(joint_jerk.worst_name),
        "r_knee_peak": knee_peak,
        "r_knee_rms": knee_rms,
        "ask_nm": ask_nm,
        "ask_joint": None if ask_peak is None else ask_peak.joint,
        "ask_t_s": None if ask_peak is None else ask_peak.t_s,
        "ask_signed_nm": None if ask_peak is None else ask_peak.signed_nm,
        "ask_over_ticks": len(over_times),
        "ask_over_writes": sum(1 for row in asks if row.sum_nm > SAG_BAR_NM + 1e-9),
        "asks_by_joint": joint_asks,
        "preview_stages": stage_rows,
        "max_leg_actuator_nm": float(session.max_leg_tau),
        "sag_bar_nm": SAG_BAR_NM,
        "preview_shape": float(spec.preview_shape),
        "stepping": stepping,
    }


def _f(row: dict[str, object], key: str) -> float:
    value = row[key]
    if not isinstance(value, (int, float)):
        return float("nan")
    return float(value)


def _opt(row: dict[str, object], key: str) -> float | None:
    value = row[key]
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _mm(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return "unmeasured"
    return f"{value * 1000.0:+.2f} mm"


def _phase_cell(blob: object, field: str) -> str:
    if not isinstance(blob, dict):
        return "unmeasured"
    value = blob.get(field)
    if not isinstance(value, (int, float)):
        return "unmeasured"
    return f"{float(value) * 1000.0:+.2f} mm"


def _frac_cell(blob: object, field: str) -> str:
    if not isinstance(blob, dict):
        return "unmeasured"
    value = blob.get(field)
    n = blob.get("n")
    if not isinstance(value, (int, float)) or not isinstance(n, int):
        return "unmeasured"
    return f"{float(value):.3f} (n {n})"


def _phase_map(row: dict[str, object]) -> dict[str, dict[str, object]]:
    phases = row.get("phases")
    out: dict[str, dict[str, object]] = {}
    if not isinstance(phases, list):
        return out
    for blob in phases:
        if isinstance(blob, dict) and isinstance(blob.get("phase"), str):
            out[str(blob["phase"])] = blob
    return out


def _disagreement(row: dict[str, object]) -> list[str]:
    name = str(row["name"])
    posted = POSTED.get(name)
    if posted is None:
        return ["no posted Controls numbers for this row"]
    notes: list[str] = []
    phases = _phase_map(row)
    stand = phases.get("stand", {})
    stand_com = stand.get("com_min_margin_m")
    stand_com_t = stand.get("com_min_t_s")
    if isinstance(stand_com, float) and abs(stand_com * 1000.0 - posted.com_min_mm) <= 0.05:
        notes.append(
            f"Controls' CoM minimum {posted.com_min_mm:+.2f} mm matches this scorer's "
            f"stand bucket {_mm(stand_com)} at {stand_com_t} s (post-step clock; their "
            "label is one tick earlier). It is not this scorer's whole-bout minimum. "
            f"Whole-bout CoM min is {_mm(_opt(row, 'com_min_margin_m'))}. "
            "This polygon is the declared stance foot during swing, including the stop "
            "while that phase is still swing. Their posted minimum is the hull of feet "
            "with floor normal above 5 N, which stayed positive."
        )
    else:
        ss_l = phases.get("ss_L", {}).get("com_min_margin_m")
        ss_r = phases.get("ss_R", {}).get("com_min_margin_m")
        ss_r_t = phases.get("ss_R", {}).get("com_min_t_s")
        ss_r_t_s = f"{ss_r_t:.3f}" if isinstance(ss_r_t, float) else "unmeasured"
        notes.append(
            f"Controls' CoM minimum {posted.com_min_mm:+.2f} mm is not this scorer's "
            f"whole-bout minimum {_mm(_opt(row, 'com_min_margin_m'))}. "
            f"Stand is {_mm(stand_com if isinstance(stand_com, float) else None)}, "
            f"ss_L is {_mm(ss_l if isinstance(ss_l, float) else None)}, "
            f"ss_R is {_mm(ss_r if isinstance(ss_r, float) else None)} at {ss_r_t_s} s. "
            "Their +18.23 mm is the loaded-foot hull on right single support at 5.144 s. "
            "This whole-bout miss is the declared stance box while the walker is still "
            "in swing during the stop. A loaded swing foot is not added to that box."
        )
    stand_zmp = stand.get("zmp_min_margin_m")
    notes.append(
        f"Controls' ZMP minimum {posted.zmp_min_mm:+.2f} mm is the cart-table ZMP on the "
        f"loaded-foot hull. This scorer's contact CoP on the stand polygon is "
        f"{_mm(stand_zmp if isinstance(stand_zmp, float) else None)}. "
        f"Whole-bout contact-CoP min is {_mm(_opt(row, 'zmp_min_margin_m'))}."
    )
    ask = _opt(row, "ask_nm")
    ask_joint = str(row.get("ask_joint"))
    ask_t = _opt(row, "ask_t_s")
    if (
        ask is not None
        and ask_joint == posted.ask_joint
        and ask_t is not None
        and abs(ask - posted.ask_nm) <= 1e-3
        and abs(ask_t - posted.ask_t_s) <= 1e-6
    ):
        notes.append(
            f"Unclamped ask matches: {ask_joint} {ask:.4f} Nm at {ask_t:.3f} s, "
            f"|kp*(q_des−q)|+|kv*ω| before the clip. "
            f"Writes over 2.33: {row['ask_over_writes']}. "
            f"Control ticks over 2.33: {row['ask_over_ticks']}."
        )
    else:
        notes.append(
            f"Unclamped ask {ask_joint} {ask} Nm at {ask_t} s vs Controls "
            f"{posted.ask_joint} {posted.ask_nm:.4f} Nm at {posted.ask_t_s:.3f} s."
        )
    peak = _opt(row, "com_jerk_whole_peak")
    rms = _opt(row, "com_jerk_whole_rms")
    post = _opt(row, "com_jerk_post_stand_peak")
    if (
        peak is not None
        and rms is not None
        and post is not None
        and abs(peak - posted.com_jerk_peak) <= 1e-3
        and abs(rms - posted.com_jerk_rms) <= 1e-3
        and abs(post - posted.com_jerk_post_peak) <= 1e-2
    ):
        notes.append(
            f"CoM jerk matches: whole-bout peak {peak:.3f} (the stand impact, not below "
            f"{BASE_COM_JERK_PEAK}), RMS {rms:.3f}, post-stand peak {post:.3f}. "
            "Post-stand is diagnostic only."
        )
    else:
        notes.append(
            f"CoM jerk whole {peak} / {rms}, post-stand {post} vs Controls "
            f"{posted.com_jerk_peak:.3f} / {posted.com_jerk_rms:.3f}, "
            f"post {posted.com_jerk_post_peak:.3f}."
        )
    worst = str(row.get("worst_joint"))
    worst_peak = _opt(row, "worst_joint_peak")
    worst_rms = _opt(row, "worst_joint_rms")
    if (
        worst == posted.joint_name
        and worst_peak is not None
        and worst_rms is not None
        and abs(worst_peak - posted.joint_peak) <= 1e-2
        and abs(worst_rms - posted.joint_rms) <= 1e-2
    ):
        notes.append(
            f"Scalar joint jerk matches Controls: {worst} {worst_peak:.3f} / {worst_rms:.3f}. "
            f"The gate uses the vector L2 {_opt(row, 'joint_jerk_peak'):.3f} / "
            f"{_opt(row, 'joint_jerk_rms'):.3f}, which is below {BASE_JOINT_JERK_PEAK:.0f}/"
            f"{BASE_JOINT_JERK_RMS:.0f}."
        )
    else:
        notes.append(
            f"Worst-joint scalar {worst} {worst_peak} / {worst_rms} vs Controls "
            f"{posted.joint_name} {posted.joint_peak:.3f} / {posted.joint_rms:.3f}."
        )
    knee_peak = _opt(row, "r_knee_peak")
    knee_rms = _opt(row, "r_knee_rms")
    if (
        knee_peak is not None
        and knee_rms is not None
        and abs(knee_peak - posted.knee_peak) <= 1e-2
        and abs(knee_rms - posted.knee_rms) <= 1e-2
    ):
        notes.append(f"r_knee jerk matches: {knee_peak:.3f} / {knee_rms:.3f}.")
    else:
        notes.append(
            f"r_knee jerk {knee_peak} / {knee_rms} vs Controls "
            f"{posted.knee_peak:.3f} / {posted.knee_rms:.3f}."
        )
    return notes


def _row_md(row: dict[str, object]) -> list[str]:
    phases = row["phases"]
    phase_list = phases if isinstance(phases, list) else []
    lines = [
        f"## {row['name']}: {row['verdict']}",
        "",
        f"Source: `{row['source']}`",
        "",
        f"Plant md5 `{row['plant_md5']}`. Samples {row['n_samples']}. "
        f"vx { _f(row, 'vx_m_s'):.3f} m/s. Soft-pass off.",
        "",
        "| Signal | Value |",
        "| --- | --- |",
        f"| ZMP min margin | {_mm(_opt(row, 'zmp_min_margin_m'))} |",
        f"| ZMP outside fraction | {_f(row, 'zmp_outside_fraction'):.3f} |",
        f"| CoM min margin | {_mm(_opt(row, 'com_min_margin_m'))} |",
        f"| CoM outside fraction | {_f(row, 'com_outside_fraction'):.3f} |",
        (
            f"| CoM jerk whole bout (gate) | "
            f"{_f(row, 'com_jerk_whole_peak'):.3f} / {_f(row, 'com_jerk_whole_rms'):.3f} m/s³ |"
        ),
        (
            f"| CoM jerk post-stand (diagnostic) | "
            f"{_f(row, 'com_jerk_post_stand_peak'):.3f} / {_f(row, 'com_jerk_post_stand_rms'):.3f} m/s³ |"
        ),
        (
            f"| Joint jerk vector L2 (gate) | "
            f"{_f(row, 'joint_jerk_peak'):.3f} / {_f(row, 'joint_jerk_rms'):.3f} rad/s³ |"
        ),
        (
            f"| Worst joint scalar | {row['worst_joint']} "
            f"{_f(row, 'worst_joint_peak'):.3f} / {_f(row, 'worst_joint_rms'):.3f} rad/s³ |"
        ),
        (
            f"| r_knee scalar | "
            f"{_f(row, 'r_knee_peak'):.3f} / {_f(row, 'r_knee_rms'):.3f} rad/s³ |"
        ),
        f"| min up_z | {_f(row, 'min_up_z'):.3f} |",
        (
            f"| Unclamped leg ask | {row['ask_joint']} {_f(row, 'ask_nm'):.4f} Nm "
            f"at {_f(row, 'ask_t_s'):.3f} s, signed {_f(row, 'ask_signed_nm'):+.4f} Nm, "
            f"{row['ask_over_ticks']} ticks / {row['ask_over_writes']} writes over {SAG_BAR_NM:.2f} |"
        ),
        "",
        "| Phase | ZMP min | ZMP outside | CoM min | CoM outside |",
        "| --- | --- | --- | --- | --- |",
    ]
    for blob in phase_list:
        if not isinstance(blob, dict):
            continue
        lines.append(
            f"| {blob.get('phase')} | {_phase_cell(blob, 'zmp_min_margin_m')} | "
            f"{_frac_cell(blob, 'zmp_outside_fraction')} | "
            f"{_phase_cell(blob, 'com_min_margin_m')} | "
            f"{_frac_cell(blob, 'com_outside_fraction')} |"
        )
    lines.append("")
    fails = row["fail_reasons"]
    if isinstance(fails, list) and fails:
        lines.append("Gate:")
        lines.append("")
        for reason in fails:
            lines.append(f"- {reason}")
        lines.append("")
    else:
        lines.append("Gate: every MFG bar holds. CLEAR.")
        lines.append("")
    lines.append("Against Controls' posted numbers:")
    lines.append("")
    for note in _disagreement(row):
        lines.append(f"- {note}")
    lines.append("")
    return lines


def render_rows_md(rows: list[dict[str, object]]) -> str:
    lines = [
        "# Walk smoothness rerun, tip 58ce1d8",
        "",
        "Scorer rules are the #102 contact-CoP rules. Soft-pass is off. "
        "The plant file was not edited. Torque limits were not raised. "
        "Controls' gait code was not edited. The bouts are the preview rows "
        "committed on that tip.",
        "",
        f"Tip `{TIP_SHA}`. Frozen plant md5 `{FROZEN_MD5}`.",
        "",
        "Whole-bout jerk is the gate. Post-stand jerk is diagnostic. "
        "Trimming the stand impact out of the window is not a pass.",
        "",
        "MFG bar: ZMP and CoM margin ≥ 0 on the whole bout, outside fraction 0, "
        f"unclamped ask ≤ {SAG_BAR_NM:.2f} Nm, whole-bout CoM jerk below "
        f"{BASE_COM_JERK_PEAK}/{BASE_COM_JERK_RMS}, whole-bout joint-jerk vector below "
        f"{BASE_JOINT_JERK_PEAK:.0f}/{BASE_JOINT_JERK_RMS:.0f}. "
        "Below means lower by more than the last reported digit "
        f"({COM_JERK_TOL} on CoM jerk, {JOINT_JERK_TOL} on joint jerk). "
        "The stand impact that prints as 1048.991 is not below the baseline.",
        "",
        "Voice-speed row (vx 0.056): none. `scripts/voice_caller.py` still publishes "
        "`vel(+0.056, 0)` as the old bus phrase. `scripts/score_com_zmp.py` has no "
        "committed preview row at 0.056. The committed tags are `j` (vx 0.040), "
        "`g` (vx 0.150), and `kit102` (locked kit, not a preview row).",
        "",
        "| Row | Verdict | Samples | ZMP min | ZMP out | CoM min | CoM out | "
        "CoM jerk whole | CoM jerk post-stand | Joint jerk | Unclamped | min up_z |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        lines.append(
            f"| {row['name']} | {row['verdict']} | {row['n_samples']} | "
            f"{_mm(_opt(row, 'zmp_min_margin_m'))} | {_f(row, 'zmp_outside_fraction'):.3f} | "
            f"{_mm(_opt(row, 'com_min_margin_m'))} | {_f(row, 'com_outside_fraction'):.3f} | "
            f"{_f(row, 'com_jerk_whole_peak'):.3f}/{_f(row, 'com_jerk_whole_rms'):.3f} | "
            f"{_f(row, 'com_jerk_post_stand_peak'):.3f}/{_f(row, 'com_jerk_post_stand_rms'):.3f} | "
            f"{_f(row, 'joint_jerk_peak'):.3f}/{_f(row, 'joint_jerk_rms'):.3f} | "
            f"{row['ask_joint']} {_f(row, 'ask_nm'):.3f} Nm | {_f(row, 'min_up_z'):.3f} |"
        )
    lines.append("")
    for row in rows:
        lines.extend(_row_md(row))
    return "\n".join(lines) + "\n"


def _near(value: float | None, posted: float, tol: float) -> bool:
    return value is not None and math.isfinite(value) and abs(value - posted) <= tol


def _named_blobs(row: dict[str, object], key: str, name_key: str) -> dict[str, dict[str, object]]:
    blobs = row.get(key)
    out: dict[str, dict[str, object]] = {}
    if not isinstance(blobs, list):
        return out
    for blob in blobs:
        if isinstance(blob, dict) and isinstance(blob.get(name_key), str):
            out[str(blob[name_key])] = blob
    return out


def _time_cell(blob: object, field: str) -> str:
    if not isinstance(blob, dict):
        return "unmeasured"
    value = blob.get(field)
    if not isinstance(value, float):
        return "unmeasured"
    return f"{value:.3f} s"


def _nm(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return "unmeasured"
    return f"{value:.4f} Nm"


def _ac_notes(row: dict[str, object]) -> list[str]:
    claim = AC81435_CLAIMS.get(str(row["name"]))
    if claim is None:
        return ["no posted Controls numbers for this row"]
    notes: list[str] = []
    zmp = _opt(row, "zmp_min_margin_m")
    com = _opt(row, "com_min_margin_m")
    z_out = _f(row, "zmp_outside_fraction")
    c_out = _f(row, "com_outside_fraction")
    same_margin = (
        _near(None if zmp is None else zmp * 1000.0, claim.margin_mm, 0.05)
        and _near(None if com is None else com * 1000.0, claim.margin_mm, 0.05)
        and z_out == 0.0
        and c_out == 0.0
    )
    if same_margin:
        notes.append(
            f"Whole-bout contact-CoP and CoM margins match Controls' "
            f"+{claim.margin_mm:.2f} mm with outside fraction 0."
        )
    else:
        notes.append(
            f"Measured whole-bout contact CoP minimum is {_mm(zmp)}, outside fraction "
            f"{z_out:.3f}. CoM minimum is {_mm(com)}, outside fraction {c_out:.3f}. "
            f"Controls posted +{claim.margin_mm:.2f} mm and outside fraction 0. "
            "Their margin is the cart-table ZMP on the hull of feet with floor normal "
            "above 5 N. This margin is the contact CoP on the declared phase polygon. "
            "Swing uses the stance foot only, including while the stop is still in swing."
        )
        stages_now = _named_blobs(row, "preview_stages", "stage")
        for stage_name in ("walk", "stand"):
            blob = stages_now.get(stage_name, {})
            stage_com = blob.get("com_min_margin_m")
            stage_com_f = stage_com if isinstance(stage_com, float) else None
            if _near(None if stage_com_f is None else stage_com_f * 1000.0, claim.margin_mm, 0.05):
                notes.append(
                    f"The {stage_name}-stage CoM minimum is {_mm(stage_com_f)}. "
                    f"That is the +{claim.margin_mm:.2f} mm Controls posted as the "
                    "whole-bout minimum."
                )
                break
    ask = _opt(row, "ask_nm")
    ask_t = _opt(row, "ask_t_s")
    ask_joint = str(row.get("ask_joint"))
    if (
        ask_joint == claim.ask_joint
        and _near(ask, claim.ask_nm, 1e-3)
        and _near(ask_t, claim.ask_t_s, 1e-4)
    ):
        notes.append(
            f"Unclamped peak matches: {ask_joint} {_nm(ask)} at {ask_t:.3f} s "
            f"(posted {claim.ask_nm:.4f} Nm at {claim.ask_t_s:.3f} s)."
        )
    else:
        notes.append(
            f"Unclamped peak is {ask_joint} {_nm(ask)} at {ask_t} s. "
            f"Controls posted {claim.ask_joint} {claim.ask_nm:.4f} Nm at {claim.ask_t_s:.3f} s."
        )
    peak = _opt(row, "com_jerk_whole_peak")
    rms = _opt(row, "com_jerk_whole_rms")
    if _near(peak, claim.com_jerk_peak, 1e-2) and _near(rms, claim.com_jerk_rms, 1e-2):
        notes.append(
            f"CoM jerk matches: peak {peak:.3f}, RMS {rms:.3f} "
            f"(posted {claim.com_jerk_peak:.3f} / {claim.com_jerk_rms:.3f})."
        )
    else:
        notes.append(
            f"CoM jerk is {peak} / {rms}. Controls posted "
            f"{claim.com_jerk_peak:.3f} / {claim.com_jerk_rms:.3f}."
        )
    knee_peak = _opt(row, "r_knee_peak")
    knee_rms = _opt(row, "r_knee_rms")
    if _near(knee_peak, claim.knee_peak, 1e-2) and _near(knee_rms, claim.knee_rms, 1e-2):
        notes.append(
            f"r_knee jerk matches: {knee_peak:.3f} / {knee_rms:.3f}."
        )
    else:
        notes.append(
            f"r_knee jerk is {knee_peak} / {knee_rms}. Controls posted "
            f"{claim.knee_peak:.3f} / {claim.knee_rms:.3f}."
        )
    worst = str(row.get("worst_joint"))
    worst_peak = _opt(row, "worst_joint_peak")
    worst_rms = _opt(row, "worst_joint_rms")
    if (
        worst == claim.joint_name
        and _near(worst_peak, claim.joint_peak, 1e-2)
        and _near(worst_rms, claim.joint_rms, 1e-2)
    ):
        notes.append(
            f"Worst-joint scalar matches: {worst} {worst_peak:.3f} / {worst_rms:.3f}."
        )
    else:
        worst_txt = (
            f"{worst_peak:.3f} / {worst_rms:.3f}"
            if worst_peak is not None and worst_rms is not None
            else f"{worst_peak} / {worst_rms}"
        )
        notes.append(
            f"Worst-joint scalar is {worst} {worst_txt}. "
            f"Controls posted {claim.joint_name} {claim.joint_peak:.3f} / {claim.joint_rms:.3f}, "
            "the largest leg hinge. This scorer names the largest actuated hinge, arms included."
        )
    stages = _named_blobs(row, "preview_stages", "stage")
    stop = stages.get("stop", {})
    stop_com = stop.get("com_min_margin_m")
    stop_zmp = stop.get("zmp_min_margin_m")
    stop_com_f = stop_com if isinstance(stop_com, float) else None
    stop_zmp_f = stop_zmp if isinstance(stop_zmp, float) else None
    stop_same = (
        _near(None if stop_com_f is None else stop_com_f * 1000.0, claim.stop_margin_mm, 0.05)
        and _near(None if stop_zmp_f is None else stop_zmp_f * 1000.0, claim.stop_margin_mm, 0.05)
    )
    if stop_same:
        notes.append(
            f"Stop-stage contact CoP and CoM minima match Controls' "
            f"+{claim.stop_margin_mm:.2f} mm."
        )
    else:
        notes.append(
            f"Stop-stage contact CoP minimum is {_mm(stop_zmp_f)}, CoM minimum is "
            f"{_mm(stop_com_f)}. Controls posted stop margin +{claim.stop_margin_mm:.2f} mm "
            "on the loaded-foot hull."
        )
    stop_ask = stop.get("ask_nm")
    stop_ask_f = stop_ask if isinstance(stop_ask, float) else None
    stop_t = stop.get("ask_t_s")
    stop_t_f = stop_t if isinstance(stop_t, float) else None
    stop_joint = str(stop.get("ask_joint"))
    if (
        stop_joint == claim.stop_ask_joint
        and _near(stop_ask_f, claim.stop_ask_nm, 1e-3)
        and _near(stop_t_f, claim.stop_ask_t_s, 1e-4)
    ):
        notes.append(
            f"Stop ask matches: {stop_joint} {_nm(stop_ask_f)} at {stop_t_f:.3f} s."
        )
    else:
        notes.append(
            f"Stop ask is {stop_joint} {_nm(stop_ask_f)} at {stop_t_f} s. "
            f"Controls posted {claim.stop_ask_joint} {claim.stop_ask_nm:.4f} Nm "
            f"at {claim.stop_ask_t_s:.3f} s."
        )
    return notes


def _case_margin(row: dict[str, object]) -> tuple[float, str]:
    pairs: list[tuple[float, str]] = []
    zmp = _opt(row, "zmp_min_margin_m")
    com = _opt(row, "com_min_margin_m")
    if zmp is not None:
        pairs.append((zmp, "ZMP"))
    if com is not None:
        pairs.append((com, "CoM"))
    if not pairs:
        return float("-inf"), "unmeasured"
    return min(pairs, key=lambda item: item[0])


def _worst_axis(cases: list[dict[str, object]]) -> dict[str, object]:
    margin_case = min(cases, key=lambda row: _case_margin(row)[0])
    margin_m, margin_signal = _case_margin(margin_case)
    torque_case = max(cases, key=lambda row: _opt(row, "ask_nm") if _opt(row, "ask_nm") is not None else -1.0)
    tips = [row for row in cases if bool(row.get("tipped"))]
    ask_nm = _opt(torque_case, "ask_nm")

    def label_of(row: dict[str, object]) -> str:
        return str(row.get("perturb_label") or row.get("name"))

    return {
        "n": len(cases),
        "worst_margin_m": margin_m,
        "worst_margin_signal": margin_signal,
        "worst_margin_label": label_of(margin_case),
        "worst_margin_zmp_m": margin_case.get("zmp_min_margin_m"),
        "worst_margin_com_m": margin_case.get("com_min_margin_m"),
        "worst_margin_zmp_out": margin_case.get("zmp_outside_fraction"),
        "worst_margin_com_out": margin_case.get("com_outside_fraction"),
        "worst_ask_nm": ask_nm,
        "worst_ask_joint": torque_case.get("ask_joint"),
        "worst_ask_t_s": torque_case.get("ask_t_s"),
        "worst_ask_label": label_of(torque_case),
        "worst_ask_headroom_nm": None if ask_nm is None else SAG_BAR_NM - ask_nm,
        "tip": bool(tips),
        "tip_labels": [label_of(row) for row in tips],
        "plant_md5s": sorted({str(row.get("plant_md5")) for row in cases}),
    }


def _compact_case(row: dict[str, object]) -> dict[str, object]:
    keys = (
        "perturb_label", "verdict", "n_samples", "zmp_min_margin_m", "zmp_outside_fraction",
        "com_min_margin_m", "com_outside_fraction", "ask_nm", "ask_joint", "ask_t_s",
        "min_up_z", "tipped", "com_jerk_whole_peak", "com_jerk_whole_rms",
        "joint_jerk_peak", "joint_jerk_rms", "plant_md5", "runtime_mass_kg",
        "friction_sliding", "latency_ticks", "seed", "mass_scale", "fault_reasons",
    )
    return {key: row.get(key) for key in keys}


def _joint_lines(row: dict[str, object]) -> list[str]:
    asks = row.get("asks_by_joint")
    lines = [
        "| Joint | Unclamped peak | Time | Headroom to 2.33 | Stage |",
        "| --- | --- | --- | --- | --- |",
    ]
    if not isinstance(asks, list):
        return lines
    for blob in asks:
        if not isinstance(blob, dict):
            continue
        t_s = blob.get("t_s")
        t_txt = f"{float(t_s):.3f} s" if isinstance(t_s, float) else "unmeasured"
        head = blob.get("headroom_nm")
        head_txt = f"{float(head):+.4f} Nm" if isinstance(head, float) else "unmeasured"
        summed = blob.get("sum_nm")
        lines.append(
            f"| {blob.get('joint')} | {_nm(summed if isinstance(summed, float) else None)} | "
            f"{t_txt} | {head_txt} | {blob.get('stage')} |"
        )
    return lines


def _axis_line(name: str, worst: dict[str, object]) -> str:
    margin = worst.get("worst_margin_m")
    margin_f = margin if isinstance(margin, float) else None
    ask = worst.get("worst_ask_nm")
    ask_f = ask if isinstance(ask, float) else None
    tip = "yes" if worst.get("tip") else "no"
    tip_labels = worst.get("tip_labels")
    tip_txt = tip if not tip_labels else f"yes ({', '.join(str(item) for item in tip_labels)})"
    return (
        f"| {name} | {_mm(margin_f)} {worst.get('worst_margin_signal')} | "
        f"{worst.get('worst_margin_label')} | "
        f"{worst.get('worst_ask_joint')} {_nm(ask_f)} | {worst.get('worst_ask_label')} | {tip_txt} |"
    )


def _case_table(cases: list[dict[str, object]]) -> list[str]:
    lines = [
        "| Case | ZMP min | ZMP out | CoM min | CoM out | Unclamped | min up_z | Tip | Plant md5 |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in cases:
        lines.append(
            f"| {row.get('perturb_label')} | {_mm(_opt(row, 'zmp_min_margin_m'))} | "
            f"{_f(row, 'zmp_outside_fraction'):.3f} | {_mm(_opt(row, 'com_min_margin_m'))} | "
            f"{_f(row, 'com_outside_fraction'):.3f} | {row.get('ask_joint')} {_nm(_opt(row, 'ask_nm'))} | "
            f"{_f(row, 'min_up_z'):.3f} | {'yes' if row.get('tipped') else 'no'} | `{row.get('plant_md5')}` |"
        )
    return lines


def render_ac81435_md(payload: dict[str, object]) -> str:
    rows = payload["rows"]
    row_list = rows if isinstance(rows, list) else []
    perturb = payload["perturbation"]
    perturb_map = perturb if isinstance(perturb, dict) else {}
    lines = [
        "# Walk smoothness, tip ac81435",
        "",
        "The 58ce1d8 rerun is already scored in `docs/WALK_SMOOTHNESS_RERUN.md`. "
        "Both the slow row and the kit-vx row on that tip are Prefer FAIL. "
        "This file scores tip ac81435, whole bout, with the same #102 rules. "
        "Soft-pass is off. The plant file was not edited. The gait was not edited.",
        "",
        f"Tip `{payload['tip_sha']}`. Frozen plant md5 `{FROZEN_MD5}`. "
        f"File hash after the runs: `{payload['plant_md5']}`.",
        "",
        "The gate is the whole bout. Margin ≥ 0, outside fraction 0, unclamped ask "
        f"≤ {SAG_BAR_NM:.2f} Nm, CoM jerk below {BASE_COM_JERK_PEAK}/{BASE_COM_JERK_RMS}, "
        f"joint-jerk vector below {BASE_JOINT_JERK_PEAK:.0f}/{BASE_JOINT_JERK_RMS:.0f}. "
        "Below means lower by more than the last reported digit. "
        "Preview stages (stand / start / walk / stop) are diagnostic. "
        "They do not replace the phase polygon.",
        "",
        "| Row | Verdict | Samples | ZMP min | ZMP out | CoM min | CoM out | "
        "CoM jerk | Joint jerk | Unclamped | min up_z |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in row_list:
        if not isinstance(row, dict):
            continue
        lines.append(
            f"| {row['name']} | {row['verdict']} | {row['n_samples']} | "
            f"{_mm(_opt(row, 'zmp_min_margin_m'))} | {_f(row, 'zmp_outside_fraction'):.3f} | "
            f"{_mm(_opt(row, 'com_min_margin_m'))} | {_f(row, 'com_outside_fraction'):.3f} | "
            f"{_f(row, 'com_jerk_whole_peak'):.3f}/{_f(row, 'com_jerk_whole_rms'):.3f} | "
            f"{_f(row, 'joint_jerk_peak'):.3f}/{_f(row, 'joint_jerk_rms'):.3f} | "
            f"{row['ask_joint']} {_f(row, 'ask_nm'):.4f} Nm | {_f(row, 'min_up_z'):.3f} |"
        )
    lines.append("")
    for row in row_list:
        if not isinstance(row, dict):
            continue
        lines.extend(_ac_row_md(row))
    lines.extend([
        "## Perturbation, voice-3.60",
        "",
        "Scorer-side only, on the loaded MjModel. The plant XML is not written. "
        f"Seeds 0–9 add Gaussian noise, sigma {SEED_SIGMA_RAD} rad, to the twelve leg "
        "hinges after the stand pose. The nominal bout is not one of those seeds. "
        "Mass ±5% scales `body_mass` and `body_inertia` together, then `mj_setConst`. "
        "`mj_setConst` writes qpos back to qpos0, so the seated stand pose is restored "
        "before the bout. "
        "Friction sets sliding friction on the floor and both foot boxes to 1.2, 1.4, "
        "and 1.6. Latency ±1 tick shifts the command clock. The bus timeout still uses "
        "the real clock. Axes are separate, not a full factorial.",
        "",
        str(perturb_map.get("rug", "")),
        "",
        str(perturb_map.get("commandbus_repeats", "")),
        "",
        "| Axis | Worst margin | Case | Worst unclamped | Case | Tip |",
        "| --- | --- | --- | --- | --- | --- |",
    ])
    axes = perturb_map.get("axes")
    if isinstance(axes, dict):
        for name in ("seed", "mass", "friction", "latency"):
            worst = axes.get(name)
            if isinstance(worst, dict):
                lines.append(_axis_line(name, worst))
    lines.append("")
    overall = perturb_map.get("overall")
    if isinstance(overall, dict):
        lines.append("Overall worst case, nominal plus every axis:")
        lines.append("")
        lines.append("| Scope | Worst margin | Case | Worst unclamped | Case | Tip |")
        lines.append("| --- | --- | --- | --- | --- | --- |")
        lines.append(_axis_line("overall", overall))
        lines.append("")
        md5s = overall.get("plant_md5s")
        lines.append(f"Plant file hashes in that set: `{md5s}`.")
        lines.append("")
    cases = perturb_map.get("cases")
    if isinstance(cases, dict):
        for name in ("seed", "mass", "friction", "latency"):
            group = cases.get(name)
            if not isinstance(group, list):
                continue
            lines.append(f"### {name}")
            lines.append("")
            lines.extend(_case_table([item for item in group if isinstance(item, dict)]))
            lines.append("")
    return "\n".join(lines) + "\n"


def _ac_row_md(row: dict[str, object]) -> list[str]:
    lines = [
        f"## {row['name']}: {row['verdict']}",
        "",
        f"Source: `{row['source']}`",
        "",
        f"Plant md5 `{row['plant_md5']}`. Samples {row['n_samples']}. "
        f"vx {_f(row, 'vx_m_s'):.3f} m/s. Runtime mass {_f(row, 'runtime_mass_kg'):.4f} kg. "
        f"Sliding friction {_f(row, 'friction_sliding'):.2f}. Soft-pass off.",
        "",
        "| Signal | Value |",
        "| --- | --- |",
        f"| ZMP min margin | {_mm(_opt(row, 'zmp_min_margin_m'))} |",
        f"| ZMP outside fraction | {_f(row, 'zmp_outside_fraction'):.3f} |",
        f"| CoM min margin | {_mm(_opt(row, 'com_min_margin_m'))} |",
        f"| CoM outside fraction | {_f(row, 'com_outside_fraction'):.3f} |",
        (
            f"| CoM jerk whole bout (gate) | "
            f"{_f(row, 'com_jerk_whole_peak'):.3f} / {_f(row, 'com_jerk_whole_rms'):.3f} m/s³ |"
        ),
        (
            f"| Joint jerk vector L2 (gate) | "
            f"{_f(row, 'joint_jerk_peak'):.3f} / {_f(row, 'joint_jerk_rms'):.3f} rad/s³ |"
        ),
        (
            f"| Worst joint scalar | {row['worst_joint']} "
            f"{_f(row, 'worst_joint_peak'):.3f} / {_f(row, 'worst_joint_rms'):.3f} rad/s³ |"
        ),
        (
            f"| r_knee scalar | "
            f"{_f(row, 'r_knee_peak'):.3f} / {_f(row, 'r_knee_rms'):.3f} rad/s³ |"
        ),
        f"| min up_z | {_f(row, 'min_up_z'):.3f} |",
        (
            f"| Unclamped leg ask | {row['ask_joint']} {_nm(_opt(row, 'ask_nm'))} "
            f"at {_f(row, 'ask_t_s'):.3f} s, signed {_f(row, 'ask_signed_nm'):+.4f} Nm, "
            f"headroom {SAG_BAR_NM - _f(row, 'ask_nm'):+.4f} Nm, "
            f"{row['ask_over_ticks']} ticks over {SAG_BAR_NM:.2f} |"
        ),
        "",
        "| Phase | ZMP min | ZMP at | ZMP outside | CoM min | CoM at | CoM outside |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    phases = row.get("phases")
    if isinstance(phases, list):
        for blob in phases:
            if not isinstance(blob, dict):
                continue
            lines.append(
                f"| {blob.get('phase')} | {_phase_cell(blob, 'zmp_min_margin_m')} | "
                f"{_time_cell(blob, 'zmp_min_t_s')} | "
                f"{_frac_cell(blob, 'zmp_outside_fraction')} | "
                f"{_phase_cell(blob, 'com_min_margin_m')} | "
                f"{_time_cell(blob, 'com_min_t_s')} | "
                f"{_frac_cell(blob, 'com_outside_fraction')} |"
            )
    lines.extend([
        "",
        "| Preview stage | ZMP min | ZMP outside | CoM min | CoM outside | Unclamped ask |",
        "| --- | --- | --- | --- | --- | --- |",
    ])
    stages = row.get("preview_stages")
    if isinstance(stages, list):
        for blob in stages:
            if not isinstance(blob, dict):
                continue
            ask_nm = blob.get("ask_nm")
            ask_t = blob.get("ask_t_s")
            ask_txt = "none"
            if isinstance(ask_nm, float):
                t_txt = f"{ask_t:.3f} s" if isinstance(ask_t, float) else "unmeasured"
                head = blob.get("ask_headroom_nm")
                head_txt = f"{head:+.4f} Nm" if isinstance(head, float) else "unmeasured"
                ask_txt = f"{blob.get('ask_joint')} {_nm(ask_nm)} at {t_txt}, headroom {head_txt}"
            lines.append(
                f"| {blob.get('stage')} | {_phase_cell(blob, 'zmp_min_margin_m')} | "
                f"{_frac_cell(blob, 'zmp_outside_fraction')} | "
                f"{_phase_cell(blob, 'com_min_margin_m')} | "
                f"{_frac_cell(blob, 'com_outside_fraction')} | {ask_txt} |"
            )
    lines.extend(["", "Unclamped peak per joint:", ""])
    lines.extend(_joint_lines(row))
    lines.append("")
    fails = row.get("fail_reasons")
    if isinstance(fails, list) and fails:
        lines.append("Gate:")
        lines.append("")
        for reason in fails:
            lines.append(f"- {reason}")
        lines.append("")
    else:
        lines.append("Gate: every MFG bar holds. CLEAR.")
        lines.append("")
    lines.append("Against Controls' posted numbers:")
    lines.append("")
    for note in _ac_notes(row):
        lines.append(f"- {note}")
    lines.append("")
    return lines


def run_ac81435() -> dict[str, object]:
    nominal: list[dict[str, object]] = []
    for spec in AC81435_ROWS:
        print(f"[ac81435] {spec.name}", flush=True)
        row = run_preview_row(spec, tip_sha=TIP_SHA_AC81435)
        nominal.append(row)
        print(
            f"[ac81435] {spec.name} n={row['n_samples']} "
            f"ask={row['ask_joint']} {row['ask_nm']:.4f} at {row['ask_t_s']} "
            f"jerk={row['com_jerk_whole_peak']:.3f}/{row['com_jerk_whole_rms']:.3f} "
            f"zmp={row['zmp_min_margin_m']} com={row['com_min_margin_m']} "
            f"{row['verdict']}",
            flush=True,
        )
    voice = AC81435_ROWS[0]
    grouped: dict[str, list[dict[str, object]]] = {
        "seed": [],
        "mass": [],
        "friction": [],
        "latency": [],
    }
    for seed in range(10):
        label = f"seed-{seed}"
        print(f"[ac81435] {label}", flush=True)
        grouped["seed"].append(run_preview_row(
            voice, Perturb(label=label, seed=seed), TIP_SHA_AC81435,
        ))
    for scale in (0.95, 1.05):
        label = f"mass-{scale:.2f}"
        print(f"[ac81435] {label}", flush=True)
        grouped["mass"].append(run_preview_row(
            voice, Perturb(label=label, mass_scale=scale), TIP_SHA_AC81435,
        ))
    for mu in (1.2, 1.4, 1.6):
        label = f"friction-{mu:.1f}"
        print(f"[ac81435] {label}", flush=True)
        grouped["friction"].append(run_preview_row(
            voice, Perturb(label=label, friction=mu), TIP_SHA_AC81435,
        ))
    for ticks in (-1, 1):
        label = f"latency-{ticks:+d}"
        print(f"[ac81435] {label}", flush=True)
        grouped["latency"].append(run_preview_row(
            voice, Perturb(label=label, latency_ticks=ticks), TIP_SHA_AC81435,
        ))
    nominal_voice = dict(nominal[0])
    nominal_voice["perturb_label"] = "nominal"
    overall_rows = [nominal_voice]
    for group in grouped.values():
        overall_rows.extend(group)
    file_md5 = _plant_md5()
    return {
        "tip_sha": TIP_SHA_AC81435,
        "plant_md5": file_md5,
        "soft_pass": SOFT_PASS,
        "rows": nominal,
        "perturbation": {
            "row": "voice-3.60",
            "seed_sigma_rad": SEED_SIGMA_RAD,
            "seeds": list(range(10)),
            "mass_scales": [0.95, 1.05],
            "inertia_scaled_with_mass": True,
            "frictions": [1.2, 1.4, 1.6],
            "latency_ticks": [-1, 1],
            "rug": (
                "Rug was not run. The plant can name `col_mat_rug`, and `steer_walk` "
                "has entrance CommandBus scripts. Neither is a preview scene for "
                "vx 0.056 at period 3.60 s, so a rug bout was not invented."
            ),
            "commandbus_repeats": (
                "Extra CommandBus start/stop repeats were not added. This preview row "
                "already sends one `vel` and one `stop`."
            ),
            "axes": {name: _worst_axis(group) for name, group in grouped.items()},
            "overall": _worst_axis(overall_rows),
            "cases": {name: [_compact_case(row) for row in group] for name, group in grouped.items()},
        },
    }


def _voice_spec(name: str, period_s: float, shape: float, source: str, vx_m_s: float = 0.056) -> PreviewRowSpec:
    return PreviewRowSpec(
        name=name,
        period_s=period_s,
        dsp=0.70,
        amp_m=0.043,
        z_m=0.004,
        arm_s=2.40,
        vx_m_s=vx_m_s,
        stand_s=0.40,
        walk_s=11.00,
        stop_s=3.00,
        source=source,
        preview_shape=shape,
    )


def _d7_nominals() -> tuple[PreviewRowSpec, ...]:
    return (
        _voice_spec(
            "shape0-3.57", 3.57, 0.0,
            "previews/com_zmp_declared.json voice-3.57 nominal, preview_shape 0, 1800 ticks",
        ),
        _voice_spec(
            "shape0-3.60", 3.60, 0.0,
            "previews/com_zmp_declared.json voice-3.60 nominal, preview_shape 0, 1800 ticks",
        ),
        _voice_spec(
            "shape1-3.60", 3.60, 1.0,
            "previews/com_zmp_declared.json s1-3.60 nominal, preview_shape 1, 1800 ticks",
        ),
        _voice_spec(
            "slow-6.40", 6.40, 1.0,
            "previews/com_zmp_declared.json slow-6.40-s1, preview_shape 1, vx 0.040",
            vx_m_s=0.040,
        ),
    )


def _assert_gait(tip: str) -> None:
    gait = Path(lipm_gait.__file__).resolve()
    op3 = Path(__import__("op3_walk").__file__).resolve()
    text = op3.read_text(encoding="utf-8")
    fields = getattr(lipm_gait.LipmConfig, "__dataclass_fields__", {})
    if tip == "d7b06e7":
        if "preview_shape" not in fields:
            raise SystemExit(f"d7b06e7 gait is missing preview_shape ({gait})")
        if _GAIT_SCRIPTS and not str(gait).startswith(str(Path(_GAIT_SCRIPTS).resolve())):
            raise SystemExit(f"d7b06e7 loaded {gait}, not {_GAIT_SCRIPTS}")
    elif tip == "58ce1d8":
        if "sole_level" in text:
            raise SystemExit(f"58ce1d8 loaded a sole_level walker from {op3}")
        if "preview_shape" in fields:
            raise SystemExit(f"58ce1d8 loaded a preview_shape config from {gait}")
    elif tip == "ac81435":
        if "sole_level" not in text:
            raise SystemExit(f"ac81435 walker has no sole_level ({op3})")
        if "preview_shape" in fields:
            raise SystemExit(f"ac81435 loaded preview_shape from {gait}")
    else:
        raise SystemExit(f"unknown tip {tip}")


def _step_jobs(tip: str) -> list[tuple[PreviewRowSpec, Perturb | None]]:
    if tip == "58ce1d8":
        slow = next(spec for spec in PREVIEW_ROWS if spec.name == "slow")
        return [(slow, None)]
    if tip == "ac81435":
        return [(spec, None) for spec in AC81435_ROWS]
    if tip != "d7b06e7":
        raise SystemExit(f"unknown tip {tip}")
    jobs: list[tuple[PreviewRowSpec, Perturb | None]] = [(spec, None) for spec in _d7_nominals()]
    shape0 = [spec for spec in _d7_nominals() if spec.preview_shape == 0.0]
    for spec in shape0:
        jobs.append((spec, Perturb(label="rug", rug=True)))
        jobs.append((spec, Perturb(label="cycles-5", cycles=5)))
    voice = [spec for spec in _d7_nominals() if spec.vx_m_s == 0.056]
    for spec in voice:
        jobs.append((spec, Perturb(label="mass-0.95", mass_scale=0.95)))
        jobs.append((spec, Perturb(label="friction-1.2", friction=1.2)))
        jobs.append((spec, Perturb(label="latency--1", latency_ticks=-1)))
    return jobs


def _tip_sha(tip: str) -> str:
    if tip == "d7b06e7":
        return TIP_SHA_D7
    if tip == "ac81435":
        return TIP_SHA_AC81435
    if tip == "58ce1d8":
        return TIP_SHA
    raise SystemExit(f"unknown tip {tip}")


def run_stepping(tip: str) -> dict[str, object]:
    _assert_gait(tip)
    sha = _tip_sha(tip)
    rows: list[dict[str, object]] = []
    for spec, perturb in _step_jobs(tip):
        label = "nominal" if perturb is None else perturb.label
        print(f"[stepping] {tip} {spec.name} {label}", flush=True)
        rows.append(run_preview_row(spec, perturb, sha))
        row = rows[-1]
        print(
            f"[stepping] {row['verdict']} plant {row['plant_md5_before']} -> {row['plant_md5']} "
            f"n {row['n_samples']}",
            flush=True,
        )
    return {
        "tip": tip,
        "tip_sha": sha,
        "gait_file": str(Path(lipm_gait.__file__).resolve()),
        "plant_md5": _plant_md5(),
        "soft_pass": SOFT_PASS,
        "period_definition": (
            "op3_walk.update_time puts both single-support windows inside one period, "
            "so period is one left-plus-right cycle. Each foot swings once per period. "
            "The no-slip per-swing foot travel is vx·T. vx·T/2 is the stance-to-stance spacing."
        ),
        "rows": rows,
    }


def _step_get(row: dict[str, object], *path: str) -> object:
    cur: object = row
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur


def _num(value: object) -> float | None:
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return None


def _cell(value: object, scale: float = 1.0, digits: int = 3, signed: bool = False) -> str:
    got = _num(value)
    if got is None:
        return "—"
    shown = got * scale
    if signed:
        return f"{shown:+.{digits}f}"
    return f"{shown:.{digits}f}"


def _compact_line(row: dict[str, object]) -> str:
    step = row.get("stepping")
    if not isinstance(step, dict):
        step = {}
    ss = step.get("ss")
    if not isinstance(ss, dict):
        ss = {}
    stop = step.get("stop")
    if not isinstance(stop, dict):
        stop = {}
    label = str(row.get("perturb_label") or "nominal")
    pose = str(stop.get("pose") or "—")
    return " | ".join([
        str(row.get("name")),
        label,
        str(row.get("verdict")),
        _cell(step.get("step_fraction"), digits=3),
        _cell(step.get("worst_slip_m"), 1000.0, 2),
        _cell(step.get("min_clear_m"), 1000.0, 3),
        _cell(step.get("honest_max_clear_m"), 1000.0, 3),
        _cell(step.get("airborne_min_m"), digits=3),
        _cell(_step_get(step, "expected", "per_swing_m"), digits=3),
        _cell(_step_get(step, "expected", "stance_to_stance_m"), digits=3),
        _cell(step.get("sep_move_m") if step.get("sep_move_m") is not None else step.get("sep_peak_m"), 1000.0, 1),
        _cell(None if not ss.get("n") else ss.get("min_contacts"), digits=0),
        _cell(ss.get("cop_min_m"), 1000.0, 2, signed=True),
        _cell(ss.get("edge_fraction"), digits=3),
        _cell(ss.get("tilt_max_rad"), 180.0 / math.pi, 2),
        pose,
        _cell(row.get("zmp_min_margin_m"), 1000.0, 2, signed=True),
        _cell(step.get("actual",) if False else _step_get(step, "actual", "zmp_min_m"), 1000.0, 2, signed=True),
        _cell(row.get("ask_nm"), digits=3),
        _cell(step.get("mismatch_fraction"), digits=3),
        str(row.get("n_samples")),
    ])


def _worst_perturb(rows: list[dict[str, object]]) -> dict[str, object] | None:
    cells = [
        row for row in rows
        if str(row.get("perturb_label") or "") in ("mass-0.95", "friction-1.2", "latency--1")
    ]
    if not cells:
        return None

    def key(row: dict[str, object]) -> tuple[float, float, float, float]:
        step = row.get("stepping")
        if not isinstance(step, dict):
            step = {}
        fraction = _num(step.get("step_fraction"))
        slip = _num(step.get("worst_slip_m"))
        clear = _num(step.get("min_clear_m"))
        ask = _num(row.get("ask_nm"))
        return (
            0.0 if row.get("tipped") else 1.0,
            1.0 if fraction is None else fraction,
            0.0 if slip is None else -slip,
            1.0 if clear is None else clear,
            0.0 if ask is None else -ask,
        )

    return min(cells, key=key)


def render_stepping_md(payloads: list[dict[str, object]]) -> str:
    header = (
        "| Tip | Row | Cell | Verdict | Step frac | Slip mm | Clear min mm | Clear honest mm | "
        "Air min m | vx·T m | vx·T/2 m | Sep mm | SS min contacts | CoP min mm | Edge <5 mm | "
        "Tilt deg | Stop | Declared ZMP mm | Actual ZMP mm | Ask Nm | Mismatch | n |"
    )
    rule = (
        "| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | "
        "---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: |"
    )
    header = header.replace("Sep mm", "Sep move mm")
    lines = [
        "# Walk stepping bars",
        "",
        "Soft-pass is off. A row is CLEAR only when the existing jerk, declared-stance",
        "ZMP/CoM, and unclamped-torque bars pass and the stepping bars pass.",
        "Plant file md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is checked before and after every bout.",
        "Mass, friction, latency, and the entrance mat are applied on the loaded model.",
        "The plant file is not written.",
        "",
        "## Period",
        "",
        "`op3_walk.update_time` places the left single-support window and the right",
        "single-support window inside one `period`. `gait_manager_traj` uses the same",
        "split. Period is one left-plus-right cycle, not one step. Each foot swings",
        "once per period.",
        "",
        "A no-slip walk at speed `vx` puts the next plant of that same foot `vx·T`",
        "further along x. That is the per-swing foot travel, and it is the stride bar",
        "(±20% on the airborne x change of each swing that starts while the bus is in",
        "`vel`). `vx·T/2` is the spacing between consecutive opposite footfalls, the",
        "stance-to-stance step. At 0.056 m/s and 3.60 s that spacing is 0.1008 m.",
        "The ~0.10 m figure is that stance-to-stance step. It is not the distance one",
        "swing foot travels.",
        "",
        "`kit_bus_step` does not command either distance. `x_amp = min(0.020, vx/7.50)`",
        "and the swing sine runs about ±`x_amp` in the hip frame, so the commanded",
        "foot travel is about `2·x_amp` (14.9 mm at 0.056 m/s). The bar stays on `vx·T`.",
        "",
        "## Bars",
        "",
        "Swing events are runs of ticks where that foot has zero contacts with the",
        "floor (or the entrance mat, when that geom is ground). The walker's declared",
        "phase is not the detector. Declared-versus-actual mismatch is reported.",
        "",
        "- Step fraction: forward x travelled with zero floor contacts, divided by the",
        "  forward x of both feet. Bar ≥ 90%.",
        "- Stance slip: path length of the loaded stance foot while the other foot is",
        "  airborne. Bar ≤ 2 mm on the worst move-window step.",
        "- Sole clearance: lowest of the eight contact-box corners (half-length 67.5 mm)",
        "  over 20–80% of each actual swing. Bar ≥ 8 mm. A miss reports the honest max",
        "  of those per-step minima.",
        "- Airborne advance of each move-window swing within ±20% of `vx·T`.",
        "- Single support, from contact: stance contact count ≥ 3 on every tick.",
        "- Declared-stance and actual-stance contact CoP and CoM margins ≥ 0, outside",
        "  fraction 0. Jerk strictly below the kit baseline. Unclamped ask ≤ 2.33 Nm.",
        "- Final 1 s: both feet have at least 3 contacts on every tick, trunk pitch and",
        "  roll stay within 5° of the stand median, and `up_z` stays at least 0.90.",
        "  The pose line says whether that window returns to the stand joints or",
        "  freezes in a lean.",
        "",
        "Fore/aft separation, CoP percentiles, edge dwell, and sole tilt are reported.",
        "They do not add a second cutoff.",
        "",
        header,
        rule,
    ]
    notes: list[str] = []
    for payload in payloads:
        tip = str(payload.get("tip"))
        sha = str(payload.get("tip_sha"))
        rows = payload.get("rows")
        if not isinstance(rows, list):
            continue
        typed = [row for row in rows if isinstance(row, dict)]
        for row in typed:
            lines.append("| " + tip + " | " + _compact_line(row) + " |")
        worst = _worst_perturb(typed)
        if worst is not None:
            notes.append(
                f"Worst perturbation cell on `{tip}` `{sha}`: "
                f"`{worst.get('name')}` / `{worst.get('perturb_label')}` "
                f"is {worst.get('verdict')}."
            )
    lines.extend(["", *notes, "", "Nominal rows, from the sim state:", ""])
    for payload in payloads:
        rows = payload.get("rows")
        if not isinstance(rows, list):
            continue
        tip = str(payload.get("tip"))
        for row in rows:
            if not isinstance(row, dict) or row.get("perturb_label") not in (None, "nominal"):
                continue
            step = row.get("stepping")
            if not isinstance(step, dict):
                continue
            stop = step.get("stop")
            if not isinstance(stop, dict):
                stop = {}
            pitch = _num(stop.get("pitch_off_rad"))
            final_p = _num(stop.get("final_pitch_rad"))
            stand_p = _num(stop.get("stand_pitch_rad"))
            lines.append(
                f"- `{tip}` `{row.get('name')}`: contact advance "
                f"{_cell(step.get('contact_forward_m'), digits=3)} m, "
                f"airborne advance {_cell(step.get('airborne_forward_m'), digits=3)} m, "
                f"move separation {_cell(step.get('sep_move_m'), 1000.0, 1)} mm, "
                f"declared/actual mismatch {_cell(step.get('mismatch_fraction'), digits=3)}, "
                f"final 1 s pitch { _cell(final_p, 180.0 / math.pi, 2, signed=True) } deg "
                f"against stand { _cell(stand_p, 180.0 / math.pi, 2, signed=True) } deg "
                f"(off by {_cell(pitch, 180.0 / math.pi, 2)} deg), "
                f"roll off {_cell(_num(stop.get('roll_off_rad')), 180.0 / math.pi, 2)} deg, "
                f"up_z {_cell(stop.get('min_up_z'), digits=3)}, "
                f"contacts L/R {stop.get('min_contacts_l')}/{stop.get('min_contacts_r')}, "
                f"joint error {_cell(stop.get('q_stand_err_rad'), 180.0 / math.pi, 1)} deg. "
                f"{stop.get('pose')}."
            )
    lines.append("")
    lines.extend([
        "`Sep move mm` is the peak `|x_L − x_R|` while the bus is in `vel`.",
        "With no swing, that peak is the fore/aft gap of the two feet as they move.",
        "",
        "58ce1d8 has no vx 0.056 preview row. The slow row is the one that tip published.",
        "",
        "Controls marked the ac81435 voice rows and the slow row CLEAR, and marked",
        "the d7b06e7 declared-stance cells CLEAR. A step fraction of 0, with both feet",
        "keeping floor contact and the forward motion happening in contact, is a skate.",
        "The #102 margin and torque bars already Prefer-FAIL'd ac81435 and 58ce1d8.",
        "They did not say the feet were skating. On d7b06e7 those older bars pass:",
        "declared contact CoP sits on the polygon edge, outside fraction is 0, unclamped",
        "ask stays ≤ 2.33 Nm, and jerk is under the kit baseline. The stepping bars",
        "are what fail that tip. The stop freezes pitched forward of the stand.",
        "μ 1.2 is the only #103 cell with airborne ticks. Those runs last a few",
        "control ticks, the lowest sole corner stays under 0.02 mm, and the airborne",
        "advance is a few millimetres against `vx·T` of about 0.20 m. Mass −5% and",
        "latency −1 stay at step fraction 0. Latency here is the #102 command clock,",
        "one tick early on the bus phrase. It does not create a step.",
        "",
    ])
    text = "\n".join(lines) + "\n"
    retro_path = Path(__file__).resolve().parent / "score_retro_voice.py"
    if retro_path.is_file():
        spec = importlib.util.spec_from_file_location("score_retro_voice_doc", retro_path)
        if spec is not None and spec.loader is not None:
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            section = mod.retro_markdown()
            if section:
                text = text.rstrip() + "\n\n" + section
                if not text.endswith("\n"):
                    text += "\n"
    return text


def stepping_json_path(tip: str) -> Path:
    return ROOT / "previews" / f"walk_stepping_{tip}.json"


def run_preview_rows() -> list[dict[str, object]]:
    return [run_preview_row(spec) for spec in PREVIEW_ROWS]


def _json_ready(value: object) -> object:
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, np.generic):
        return _json_ready(value.item())
    return value


def _write_stepping(tip: str) -> int:
    payload = run_stepping(tip)
    path = stepping_json_path(tip)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_json_ready(payload), indent=2) + "\n", encoding="utf-8")
    print(f"[stepping] wrote {path}")
    rows = payload.get("rows")
    if isinstance(rows, list) and any(isinstance(row, dict) and row.get("verdict") != "CLEAR" for row in rows):
        return 1
    return 0


def _write_stepping_doc() -> int:
    payloads: list[dict[str, object]] = []
    for tip in ("d7b06e7", "ac81435", "58ce1d8"):
        path = stepping_json_path(tip)
        if not path.is_file():
            raise SystemExit(f"missing {path}")
        loaded = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(loaded, dict):
            raise SystemExit(f"{path} is not an object")
        payloads.append(loaded)
    STEP_MD.parent.mkdir(parents=True, exist_ok=True)
    STEP_MD.write_text(render_stepping_md(payloads), encoding="utf-8")
    print(f"[stepping] wrote {STEP_MD}")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description="Score Day-1 kit walk jerk and ZMP margin")
    parser.add_argument("--self-test", action="store_true", help="Finite-difference and fail-rule checks")
    parser.add_argument("--rows", action="store_true", help="Score the 58ce1d8 preview rows")
    parser.add_argument("--ac81435", action="store_true", help="Score the ac81435 preview rows and perturbations")
    parser.add_argument(
        "--stepping",
        choices=("d7b06e7", "ac81435", "58ce1d8"),
        help="Score stepping bars for one tip. Set WALK_GAIT_SCRIPTS to that tip's scripts/.",
    )
    parser.add_argument("--stepping-doc", action="store_true", help="Write docs/WALK_STEPPING_BARS.md")
    parser.add_argument("--json", type=Path, default=OUT_JSON)
    parser.add_argument("--md", type=Path, default=OUT_MD)
    args = parser.parse_args()
    if args.stepping_doc:
        raise SystemExit(_write_stepping_doc())
    if args.stepping:
        raise SystemExit(_write_stepping(args.stepping))
    if args.self_test:
        raise SystemExit(self_test())
    if args.ac81435:
        payload = run_ac81435()
        json_path = args.json if args.json != OUT_JSON else AC_JSON
        md_path = args.md if args.md != OUT_MD else AC_MD
        json_path.parent.mkdir(parents=True, exist_ok=True)
        md_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(payload, indent=2) + "\n")
        md_path.write_text(render_ac81435_md(payload))
        rows = payload["rows"]
        if isinstance(rows, list):
            for row in rows:
                if isinstance(row, dict):
                    print(
                        f"{row['name']}: {row['verdict']} plant {row['plant_md5']} "
                        f"n {row['n_samples']} ask {row['ask_joint']} {row['ask_nm']}"
                    )
        print(f"[smoothness] wrote {json_path}")
        print(f"[smoothness] wrote {md_path}")
        if isinstance(rows, list) and any(
            isinstance(row, dict) and row["verdict"] != "CLEAR" for row in rows
        ):
            raise SystemExit(1)
        return
    if args.rows:
        rows = run_preview_rows()
        json_path = args.json if args.json != OUT_JSON else ROW_JSON
        md_path = args.md if args.md != OUT_MD else ROW_MD
        json_path.parent.mkdir(parents=True, exist_ok=True)
        md_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps({"tip_sha": TIP_SHA, "rows": rows}, indent=2) + "\n")
        md_path.write_text(render_rows_md(rows))
        for row in rows:
            print(f"{row['name']}: {row['verdict']} plant {row['plant_md5']} n {row['n_samples']}")
        print(f"[smoothness] wrote {json_path}")
        print(f"[smoothness] wrote {md_path}")
        if any(row["verdict"] != "CLEAR" for row in rows):
            raise SystemExit(1)
        return
    score = run_bout()
    write_outputs(score, args.json, args.md)
    print(score.honesty)
    print(f"[smoothness] wrote {args.json}")
    print(f"[smoothness] wrote {args.md}")
    if score.verdict == "Prefer FAIL":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
