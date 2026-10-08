#!/usr/bin/env python3
"""Independent score of Controls' cadence tip a5a9183 on the #102 bar set.

The gait and the plant are loaded from that tip. This file does not edit
either. Soft-pass stays off. q̈ is the acceleration ``mj_forward`` writes on
a copy of the pre-step state. It is not ``data.qacc`` after ``mj_step``, and
it is not a finite difference of qvel.

The 2.33 Nm pass uses the ctrl the plant applies. When ``actuator_ctrllimited``
is set, that ctrl is ``data.ctrl`` clipped to ``ctrlrange`` (±2.09). The
control-tick sample is the max |ask| across that tick's four 2 ms substeps.
A tick with ``|ctrl_raw| > 2.09`` fails the clip bar. ``ctrl_raw`` is the
writer command before the ctrlrange clip when that command reproduces the
stored ctrl; otherwise the clip fraction is ``raw ctrl unavailable``.

Inverse dynamics runs on every physics substep. A binary copy of the model
carries ``mjENBL_FWDINV`` and, when this MuJoCo build has it,
``mjENBL_INVDISCRETE``. The live model does not, so the forward rollout is
unchanged. Under implicitfast the step solves ``(M + dt·D)·qacc = f``. The
qacc that belongs to that discrete step is ``(qvel_next − qvel) / dt``, not
the continuous ``mj_forward`` acceleration. The copy takes ``mj_forward`` on
the pre-step state, the live ``mj_step`` advances, and ``mj_inverse`` with
the discrete flag runs on that finite-difference qacc. The inverse column
is ``data_copy.qfrc_inverse`` only.

The match residual is ``|qfrc_inverse − qfrc_actuator|`` on each leg joint
and each root dof. That is the inverse-versus-applied identity under
``mjENBL_INVDISCRETE``. 1e-3 Nm is the Prefer FAIL target. A tick is bucketed
when that residual is ≤ 1e-2 Nm on every leg joint and the root. Counts over
each threshold are both reported. The passive-inclusive residual and the
0.15 Nm band stay in the table and do not decide the bucket. The band does
not relax the signed applied bar of 2.33 Nm.

A joint with ``|applied| ≥ 2.45`` Nm is ``clamped, unclassifiable``. The
realised-motion inverse returns the clamp there, so it is not a controller,
armature, or physics class. The pass stays the real plant: armature 0.01,
signed applied ≤ 2.33 Nm on every tick, limiter fraction 0.

An independent planned-motion inverse reads the walker's own reference
``(q_ref, q̇_ref, q̈_ref)`` and replaces contacts with the planned ZMP wrench.
Double support is split three ways: ZMP position along the foot-to-foot
line, a min-norm ankle-torque split, and a minimax split (called QP below)
that minimises the largest |τ| over the leg joints with armature removed.
The QP keeps each foot's centre of pressure inside its 135×76 mm box
shrunk by ``step_bars.EDGE_DWELL_M`` (the #102 edge-dwell margin) and
each corner force inside a friction cone of μ 1.2. The three splits are
reported. A planned-motion wall is only a double-support tick where even
that QP still needs more than 2.33 Nm without armature. The realised
double-support contact split is compared with the QP split. Neither
comparison changes the pass. The per-foot QP CoP margin is logged, and a
tick whose reported CoP lies on that shrunk edge is flagged. The flag
does not change the pass. ``τ_req`` is printed with and without
``armature·q̈_ref``. A joint that is over 2.33 only before that subtraction
is an unsourced-armature candidate. It is not a pass.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
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
from scipy.optimize import Bounds, linprog, minimize

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
    "finite-difference qacc (qvel_next - qvel) / dt on the pre-step state; "
    "mj_inverse with mjENBL_INVDISCRETE uses that qacc. "
    "Not the continuous mj_forward qacc, and not a post-step q paired with it."
)
RESIDUAL_FLOOR = 0.05
JOINT_DAMPING = 0.08
BAND_CAP_NM = 0.15
DISCRETE_RESIDUAL_NM = 1e-3
BUCKET_RESIDUAL_NM = 1e-2
RAIL_NM = 2.45
RAIL_ATOL = 5e-4
RESIDUAL_FAIL = "residual-fail, not bucketed"
DISCRETE_FAIL = "discrete-residual, not bucketed"
OVER_BUCKET = "residual over 1e-2, not bucketed"
CLAMPED = "clamped, unclassifiable"
UNBUCKETED = "unbucketed"
PLAN_PHASES = ("start", "walk", "stop")
QP_MU = 1.2
QP_HALF_X_M = 0.0675
QP_HALF_Y_M = 0.038
QP_DS_FZ_N = 1.0
# Inner margin the #102 scorer already uses for edge dwell. A single-support
# CoP closer to the sole edge than this is not clearly inside. The QP box
# is the 135×76 mm sole shrunk by this value on every side. Do not replace
# it with a new literal.
QP_COP_INSET_M = float(step_bars.EDGE_DWELL_M)
# A reported CoP is on the shrunk edge when its margin to the sole matches
# the inset within this tolerance. Same 0.1 mm the full-box check already uses.
QP_COP_BOUND_TOL_M = 1e-4
# Plant file value. The sensitivity compile must not replace this literal
# with a hand-picked inertia, and it must not write the XML.
PLANT_ARMATURE = 0.01
SENSITIVITY_ARMATURE = 0.025
SENSITIVITY_LABEL = (
    "sensitivity, HW cited bound 0.0012–0.045 "
    "(STS3215 SysID 0.022–0.026), not plant"
)
# Committed 54-cell grid: fewest fail reasons, signed ask passes, and the
# match residual stays under 1e-3. The stop is this bout's final 1 s.
BEST_ROW = (0.50, 0.016)
KNEE_ID_CONTROLS_NM = 2.045
KNEE_ID_CONTROLS_DISCRETE_NM = (0.934, 1.087)
ROOT_DOFS = 6
# A planned wrench is consistent when these six root residuals are at the
# noise floor. 1e-2 Nm is the same edge the live match already treats as a
# real miss. A large hip-roll τ is not this flag.
ROOT_CONSIST_NM = 1e-2
SUPPORT_OUT_M = 1e-4
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
    "MuJoCo implicitfast solves (M + dt*D)*qacc = f. The discrete qacc is "
    "(qvel_next - qvel) / dt. mjENBL_INVDISCRETE is set on the inverse copy "
    "when mjtEnableBit has that flag, so mj_inverse matches that step. "
    "The match residual is |qfrc_inverse - qfrc_actuator| on each leg joint and "
    "each root dof. Exactly 1e-3 stays inside the Prefer FAIL target, and "
    "exactly 1e-2 stays inside the bucket. A tick over 1e-3 Nm is a Prefer FAIL "
    "count. A tick is bucketed only when every one of those residuals is "
    "<= 1e-2 Nm. A tick over 1e-2 Nm is residual over 1e-2, not bucketed. "
    "The passive-inclusive residual and the band 0.05 + dt*(0.08 + kv)*|qacc| "
    "with its 0.15 Nm cap stay as comparison columns. "
    "The band does not relax the signed applied bar: |ask| > 2.33 fails that "
    "bar on every control tick. Limiter fraction must be 0. Armature stays 0.01. "
    "A joint with |applied| >= 2.45 Nm is clamped, unclassifiable. The "
    "realised-motion inverse returns the clamp, so that sample is not put in "
    "the three buckets. "
    "On bucketed, unclamped substeps: "
    "controller fail: |qfrc_inverse| <= 2.33 and |ask| > 2.33. "
    "unsourced-armature candidate: |qfrc_inverse| > 2.33 and "
    "|qfrc_inverse - 0.01*qacc| <= 2.33. Armature 0.01 has no Hiwonder source; "
    "this bucket is not a physics candidate. "
    "physics candidate: |qfrc_inverse - 0.01*qacc| > 2.33, named by the largest "
    "of impact/contact, link inertia, and gravity. "
    "An inverse sample whose absolute value is exactly 2.450 Nm is a column bug. "
    "The planned-motion τ_req is a separate inverse of the walker's reference. "
    "A joint that clears 2.33 only after subtracting armature·q̈_ref is an "
    "unsourced-armature candidate, not a pass."
)
INTEGRATOR_NAME = {
    int(mj.mjtIntegrator.mjINT_EULER): "Euler",
    int(mj.mjtIntegrator.mjINT_RK4): "RK4",
    int(mj.mjtIntegrator.mjINT_IMPLICIT): "implicit",
    int(mj.mjtIntegrator.mjINT_IMPLICITFAST): "implicitfast",
    int(mj.mjtIntegrator.mjINT_DISCRETE): "discrete",
}

_ORIG_MJ_STEP = mj.mj_step
_CAPTURE: list[dict[str, np.ndarray]] = []
_CAPTURE_ON = False
_ID_MODEL: mj.MjModel | None = None
_ID_COPY: mj.MjData | None = None
_FOOT_SINK: list | None = None
_FOOT_SPEC: dict[str, object] | None = None


def _invdiscrete_bit() -> int | None:
    """``mjtEnableBit.mjENBL_INVDISCRETE`` when this build has the flag."""
    enum = getattr(mj, "mjtEnableBit", None)
    if enum is None:
        return None
    bit = getattr(enum, "mjENBL_INVDISCRETE", None)
    if bit is None:
        return None
    return int(bit)


def _isolated_inverse_model(model: mj.MjModel) -> mj.MjModel:
    """Binary copy with the inverse flags set. The source model's flags stay put."""
    flags = int(model.opt.enableflags)
    fd, path = tempfile.mkstemp(prefix="a5-inv-", suffix=".mjb")
    os.close(fd)
    try:
        mj.mj_saveModel(model, path)
        copied = mj.MjModel.from_binary_path(path)
    finally:
        os.unlink(path)
    if int(model.opt.enableflags) != flags:
        raise RuntimeError("saving the inverse model changed the live enableflags")
    copied.opt.enableflags |= int(mj.mjtEnableBit.mjENBL_FWDINV)
    bit = _invdiscrete_bit()
    if bit is not None:
        copied.opt.enableflags |= bit
    if int(model.opt.enableflags) != flags:
        raise RuntimeError("the inverse copy shares enableflags with the live model")
    return copied


def _qfrc_inverse_column(data_copy: mj.MjData) -> np.ndarray:
    """Inverse column for every dof. This reads ``data_copy.qfrc_inverse`` only."""
    return np.array(data_copy.qfrc_inverse, dtype=np.float64, copy=True)


def _rail_exact(value: float) -> bool:
    """True when an inverse sample prints as exactly ±2.450 Nm."""
    return abs(abs(float(value)) - RAIL_NM) <= RAIL_ATOL


def _capturing_step(model: mj.MjModel, data: mj.MjData) -> None:
    """Step the live state, then inverse the discrete acceleration.

    ``mj_forward`` on the copy snapshots the pre-step forces. The live model
    does not carry ``mjENBL_FWDINV`` or ``mjENBL_INVDISCRETE``. After the
    live step, qacc is ``(qvel_next − qvel) / dt`` on that pre-step copy.
    ``mj_inverse`` reads it. The stored inverse column is
    ``data_copy.qfrc_inverse`` only.
    """
    if not _CAPTURE_ON:
        _ORIG_MJ_STEP(model, data)
        return
    if _ID_COPY is None or _ID_MODEL is None:
        raise RuntimeError("inverse copy is not allocated")
    copy_model = _ID_MODEL
    copy = _ID_COPY
    t_pre = float(data.time)
    qpos = np.array(data.qpos, dtype=np.float64, copy=True)
    qvel = np.array(data.qvel, dtype=np.float64, copy=True)
    ctrl = np.array(data.ctrl, dtype=np.float64, copy=True)
    mj.mj_copyData(copy, copy_model, data)
    mj.mj_forward(copy_model, copy)
    qacc_fwd = np.array(copy.qacc, dtype=np.float64, copy=True)
    actuator = np.array(copy.qfrc_actuator, dtype=np.float64, copy=True)
    passive = np.array(copy.qfrc_passive, dtype=np.float64, copy=True)
    _ORIG_MJ_STEP(model, data)
    if _FOOT_SINK is not None and _FOOT_SPEC is not None:
        _FOOT_SINK.append(_sum_foot_forces(model, data, _FOOT_SPEC))
    dt = float(copy_model.opt.timestep)
    qacc = (np.array(data.qvel, dtype=np.float64) - qvel) / dt
    copy.qacc[:] = qacc
    # Constraint forces from the forward pass are still on the copy.
    # compareFwdInv saves them, runs inverse, and restores them.
    mj.mj_compareFwdInv(copy_model, copy)
    fwdinv = np.array(copy.solver_fwdinv[:2], dtype=np.float64, copy=True)
    copy.qacc[:] = qacc
    mj.mj_inverse(copy_model, copy)
    if float(np.max(np.abs(np.array(copy.qacc) - qacc))) > 1e-8:
        copy.qacc[:] = qacc
        mj.mj_inverse(copy_model, copy)
    _CAPTURE.append({
        "time": t_pre,
        "qpos": qpos,
        "qvel": qvel,
        "ctrl": ctrl,
        "qacc": qacc,
        "qacc_fwd": qacc_fwd,
        "actuator": actuator,
        "passive": passive,
        "id": _qfrc_inverse_column(copy),
        "constraint": np.array(copy.qfrc_constraint, dtype=np.float64, copy=True),
        "bias": np.array(copy.qfrc_bias, dtype=np.float64, copy=True),
        "fwdinv": fwdinv,
    })


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
    limiter_bound: bool


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


def _open_session(
    cfg: sws.lipm_gait.LipmConfig,
    scene: Path | None,
    compiled: mj.MjModel | None,
) -> sws.steer_walk.SteerSession:
    """Open a session. A compiled model replaces the plant load.

    The gait session loads with ``MjModel.from_xml_path``. When ``compiled``
    is set, that call returns the MjSpec model for the plant path. The
    compiled ``dof_armature`` is left as ``compile`` wrote it.
    """
    if compiled is None:
        return sws.steer_walk.SteerSession(video=False, lipm=cfg, scene_xml=scene)
    real = mj.MjModel.from_xml_path
    plant = sws.steer_walk.PLANT_XML.resolve()

    def _load(path: str) -> mj.MjModel:
        if Path(path).resolve() == plant:
            return compiled
        return real(path)

    mj.MjModel.from_xml_path = _load  # type: ignore[method-assign]
    try:
        return sws.steer_walk.SteerSession(video=False, lipm=cfg, scene_xml=scene)
    finally:
        mj.MjModel.from_xml_path = real  # type: ignore[method-assign]


def compile_leg_armature(armature: float) -> mj.MjModel:
    """Compile the plant with a leg-armature override set before kv.

    The file md5 is checked first. Armature is written on the MjSpec joints,
    then ``compile`` runs. A position actuator with ``dampratio=1`` recomputes
    ``actuator_biasprm`` kv from that inertia. ``model.dof_armature`` is not
    assigned after compile. The XML file is not written.
    """
    path = sws.steer_walk.PLANT_XML
    digest = _plant_md5()
    if digest != PLANT_MD5:
        raise SystemExit(f"plant md5 {digest} != {PLANT_MD5}")
    spec = mj.MjSpec.from_file(str(path.resolve()))
    found: list[str] = []
    for joint in spec.joints:
        if joint.name in sws.LEG_JOINTS:
            joint.armature = float(armature)
            found.append(str(joint.name))
    if set(found) != set(sws.LEG_JOINTS):
        raise SystemExit(f"spec leg joints {sorted(found)} != the twelve leg joints")
    model = spec.compile()
    for name in sws.LEG_JOINTS:
        jid = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name))
        got = float(model.dof_armature[int(model.jnt_dofadr[jid])])
        if abs(got - float(armature)) > 1e-12:
            raise SystemExit(f"{name} compiled armature {got} != {armature}")
    if _plant_md5() != PLANT_MD5:
        raise SystemExit("plant file changed during the armature compile")
    return model


def _leg_kv(model: mj.MjModel) -> dict[str, float]:
    rows: dict[str, float] = {}
    for name in sws.LEG_JOINTS:
        idx = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name + "_pos"))
        if idx < 0:
            raise SystemExit(f"missing actuator {name}_pos")
        rows[name] = -float(model.actuator_biasprm[idx, 2])
    return rows


def print_kv_side_by_side() -> list[dict[str, float | str]]:
    """Print plant kv beside the sensitivity kv and refuse a joint that matches."""
    if _plant_md5() != PLANT_MD5:
        raise SystemExit(f"plant md5 {_plant_md5()} != {PLANT_MD5}")
    plant = mj.MjModel.from_xml_path(str(sws.steer_walk.PLANT_XML))
    sens = compile_leg_armature(SENSITIVITY_ARMATURE)
    kv_plant = _leg_kv(plant)
    kv_sens = _leg_kv(sens)
    print(
        f"leg kv, dampratio=1, armature {PLANT_ARMATURE:.3f} beside {SENSITIVITY_ARMATURE:.3f}",
        flush=True,
    )
    print(f"{'joint':16} {'kv_0.01':>12} {'kv_0.025':>12}", flush=True)
    rows: list[dict[str, float | str]] = []
    for name in sws.LEG_JOINTS:
        left = kv_plant[name]
        right = kv_sens[name]
        if abs(left - right) <= 1e-9:
            raise SystemExit(
                f"{name} kv {left} at armature {PLANT_ARMATURE} "
                f"equals kv at {SENSITIVITY_ARMATURE}"
            )
        print(f"{name:16} {left:12.6f} {right:12.6f}", flush=True)
        rows.append({"joint": name, "kv_0_01": left, "kv_0_025": right})
    return rows


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

    def _limiter_bound(q_des: float, raw: float | None) -> bool:
        return raw is not None and abs(float(q_des) - float(raw)) > 1e-9

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
            limiter_bound=_limiter_bound(q_des, raw),
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
    """Split the pre-step inverse already stored on ``shot``.

    Gravity is ``qfrc_bias`` at the same q with qvel 0. The inverse itself
    is not rerun: ``shot`` came from ``mj_forward`` then ``mj_inverse`` on
    the pre-step copy.
    """
    qacc = np.array(shot["qacc"], dtype=np.float64, copy=True)
    ident_vec = np.array(shot["id"], dtype=np.float64, copy=True)
    scratch.qpos[:] = shot["qpos"]
    scratch.qvel[:] = shot["qvel"]
    scratch.qacc[:] = qacc
    mj.mj_fwdPosition(model, scratch)
    mqacc = np.zeros(model.nv, dtype=np.float64)
    mj.mj_mulM(model, scratch, mqacc, qacc)
    arm = np.asarray(model.dof_armature, dtype=np.float64) * qacc
    grav_data.qpos[:] = shot["qpos"]
    grav_data.qvel[:] = 0.0
    grav_data.qacc[:] = 0.0
    mj.mj_fwdPosition(model, grav_data)
    mj.mj_fwdVelocity(model, grav_data)
    gravity = np.array(grav_data.qfrc_bias, dtype=np.float64, copy=True)
    bias = np.array(shot["bias"], dtype=np.float64, copy=True)
    coriolis = bias - gravity
    contact = -np.array(shot["constraint"], dtype=np.float64, copy=True)
    passive = -np.array(shot["passive"], dtype=np.float64, copy=True)
    link = (mqacc - arm) + coriolis
    ident = arm + link + gravity + contact + passive
    return {
        "id": ident_vec,
        "arm_model": arm,
        "arm_001": 0.01 * qacc,
        "link": link,
        "gravity": gravity,
        "contact": contact,
        "passive": passive,
        "ident": ident,
        "qacc": qacc,
        "actuator": np.array(shot["actuator"], dtype=np.float64, copy=True),
        "passive_raw": np.array(shot["passive"], dtype=np.float64, copy=True),
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


def _integrator_name(model: mj.MjModel) -> str:
    code = int(model.opt.integrator)
    return INTEGRATOR_NAME.get(code, f"unknown-{code}")


def _implicit_terms(dt: float, kv: float, qacc: float) -> tuple[float, float]:
    """Signed implicitfast offset and the residual band, both in Nm."""
    scale = float(dt) * (JOINT_DAMPING + float(kv))
    offset = scale * float(qacc)
    band = RESIDUAL_FLOOR + scale * abs(float(qacc))
    return offset, band


def _solver_report(model: mj.MjModel) -> dict[str, object]:
    """Plant solver settings. Cone, impratio, and noslip are the MuJoCo defaults."""
    opt = model.opt
    cone = int(opt.cone)
    solver = int(opt.solver)
    jacobian = int(opt.jacobian)
    noslip_iterations = int(opt.noslip_iterations)
    impratio = float(opt.impratio)
    report = {
        "timestep": float(opt.timestep),
        "integrator": _integrator_name(model),
        "cone": "pyramidal" if cone == int(mj.mjtCone.mjCONE_PYRAMIDAL) else f"code-{cone}",
        "impratio": impratio,
        "noslip_iterations": noslip_iterations,
        "noslip_tolerance": float(opt.noslip_tolerance),
        "solver": "Newton" if solver == int(mj.mjtSolver.mjSOL_NEWTON) else f"code-{solver}",
        "iterations": int(opt.iterations),
        "tolerance": float(opt.tolerance),
        "jacobian": "auto" if jacobian == int(mj.mjtJacobian.mjJAC_AUTO) else f"code-{jacobian}",
        "enableflags": int(opt.enableflags),
        "disableflags": int(opt.disableflags),
        "cone_override": cone != int(mj.mjtCone.mjCONE_PYRAMIDAL),
        "impratio_override": abs(impratio - 1.0) > 1e-15,
        "noslip_override": noslip_iterations != 0,
    }
    if abs(float(opt.timestep) - 0.002) > 1e-15:
        raise RuntimeError(f"timestep is {float(opt.timestep)}, expected 0.002")
    if report["integrator"] != "implicitfast":
        raise RuntimeError(f"integrator is {report['integrator']}")
    if report["cone_override"] or report["impratio_override"] or report["noslip_override"]:
        raise RuntimeError(f"solver override on the plant: {report}")
    if solver != int(mj.mjtSolver.mjSOL_NEWTON) or int(opt.iterations) != 100:
        raise RuntimeError(f"solver is not the Newton default: {report}")
    if abs(float(opt.tolerance) - 1e-8) > 1e-15:
        raise RuntimeError(f"solver tolerance is {float(opt.tolerance)}")
    if jacobian != int(mj.mjtJacobian.mjJAC_AUTO):
        raise RuntimeError(f"jacobian is {jacobian}")
    if int(opt.enableflags) != 0 or int(opt.disableflags) != 0:
        raise RuntimeError("live model option flags are not clear")
    return report


def _plan_phase(stage: str) -> str:
    """Map the walker's preview stage onto start, walk, or stop."""
    if stage == "walk":
        return "walk"
    if stage == "stop":
        return "stop"
    return "start"


def _plan_support(swing: str, stance: str) -> tuple[str, str]:
    """Single support when a swing side is set. Stance is ``l`` or ``r``."""
    if swing in ("L", "R"):
        side = stance.lower() if stance in ("L", "R") else ("r" if swing == "L" else "l")
        return "ss", side
    side = stance.lower() if stance in ("L", "R") else ""
    return "ds", side


def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = (float(v[0]), float(v[1]), float(v[2]))
    return np.array(
        [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]],
        dtype=np.float64,
    )


def _foot_box(model: mj.MjModel, data: mj.MjData, side: str) -> dict[str, object]:
    """Bottom face of the contact box. Centre matches ``_sole_point``."""
    gid = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, f"{side}_foot_contact"))
    if gid < 0:
        raise RuntimeError(f"missing {side}_foot_contact")
    centre = np.array(data.geom_xpos[gid], dtype=np.float64)
    rot = np.array(data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
    half = np.array(model.geom_size[gid], dtype=np.float64)
    sole = centre - rot[:, 2] * float(half[2])
    corners = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            corners.append(sole + rot[:, 0] * sx * float(half[0]) + rot[:, 1] * sy * float(half[1]))
    pts = np.stack(corners, axis=0)
    return {
        "centre": sole,
        "corners": pts,
        "y_m": float(sole[1]),
        "y_lo_m": float(np.min(pts[:, 1])),
        "y_hi_m": float(np.max(pts[:, 1])),
    }


def _support_margin(point_xy: np.ndarray, corners: np.ndarray) -> float:
    hull = sws.steer_walk.convex_hull_xy(np.asarray(corners, dtype=np.float64)[:, :2])
    return float(sws.steer_walk.support_margin(np.asarray(point_xy, dtype=np.float64), hull))


def _sole_point(model: mj.MjModel, data: mj.MjData, side: str) -> np.ndarray:
    """World point at the bottom centre of the foot contact box."""
    gid = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, f"{side}_foot_contact"))
    if gid < 0:
        raise RuntimeError(f"missing {side}_foot_contact")
    centre = np.array(data.geom_xpos[gid], dtype=np.float64)
    rot = np.array(data.geom_xmat[gid], dtype=np.float64).reshape(3, 3)
    half_z = float(model.geom_size[gid, 2])
    return centre - rot[:, 2] * half_z


def _body_point(model: mj.MjModel, data: mj.MjData, name: str) -> tuple[int, np.ndarray]:
    bid = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name))
    if bid < 0:
        raise RuntimeError(f"missing body {name}")
    return bid, np.array(data.xpos[bid], dtype=np.float64)


def _jac_wrench(
    model: mj.MjModel,
    data: mj.MjData,
    point: np.ndarray,
    body_id: int,
    force: np.ndarray,
    torque: np.ndarray,
) -> np.ndarray:
    """Generalized force of a world wrench applied to ``body_id`` at ``point``."""
    jacp = np.zeros((3, model.nv), dtype=np.float64)
    jacr = np.zeros((3, model.nv), dtype=np.float64)
    mj.mj_jac(model, data, jacp, jacr, np.asarray(point, dtype=np.float64), int(body_id))
    return jacp.T @ np.asarray(force, dtype=np.float64) + jacr.T @ np.asarray(torque, dtype=np.float64)


def _linear_foot_split(
    p_l: np.ndarray,
    p_r: np.ndarray,
    p_zmp: np.ndarray,
    force: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Share ``force`` by the ZMP's position along the foot-to-foot line.

    The share is clamped to the segment. A ZMP past either foot puts the
    whole wrench on that foot. The return is ``(f_l, f_r, unclamped s on R)``.
    """
    line = p_r[:2] - p_l[:2]
    denom = float(line @ line)
    if denom < 1e-12:
        s = 0.5
    else:
        s = float((p_zmp[:2] - p_l[:2]) @ line) / denom
    s_clip = min(1.0, max(0.0, s))
    f_r = s_clip * force
    f_l = (1.0 - s_clip) * force
    return f_l, f_r, s


def _min_norm_ankle_split(
    p_l: np.ndarray,
    p_r: np.ndarray,
    a_l: np.ndarray,
    a_r: np.ndarray,
    p_zmp: np.ndarray,
    force: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Split a ZMP wrench by minimum ankle-torque norm.

    Variables are the two foot forces and the two foot moments. The wrench
    sum equals ``force`` acting at ``p_zmp`` with zero moment about that
    point. Among those splits, ankle moments
    ``m + (c − a) × f`` have minimum norm. A tiny wrench penalty keeps the
    solution unique when several splits share that minimum.
    """
    # x = [f_l, m_l, f_r, m_r], 12
    eye3 = np.eye(3, dtype=np.float64)
    z_l = _skew(p_l - p_zmp)
    z_r = _skew(p_r - p_zmp)
    # Moment about the ZMP: m + (c - p_zmp) × f
    c_mat = np.zeros((6, 12), dtype=np.float64)
    c_mat[0:3, 0:3] = eye3
    c_mat[0:3, 6:9] = eye3
    c_mat[3:6, 0:3] = z_l
    c_mat[3:6, 3:6] = eye3
    c_mat[3:6, 6:9] = z_r
    c_mat[3:6, 9:12] = eye3
    b = np.zeros(6, dtype=np.float64)
    b[0:3] = force
    # Ankle moment rows.
    al = _skew(p_l - a_l)
    ar = _skew(p_r - a_r)
    a_mat = np.zeros((6, 12), dtype=np.float64)
    a_mat[0:3, 0:3] = al
    a_mat[0:3, 3:6] = eye3
    a_mat[3:6, 6:9] = ar
    a_mat[3:6, 9:12] = eye3
    eps = 1e-8
    hess = 2.0 * (a_mat.T @ a_mat + eps * np.eye(12, dtype=np.float64))
    kkt = np.zeros((18, 18), dtype=np.float64)
    kkt[0:12, 0:12] = hess
    kkt[0:12, 12:18] = c_mat.T
    kkt[12:18, 0:12] = c_mat
    rhs = np.zeros(18, dtype=np.float64)
    rhs[12:18] = b
    try:
        sol = np.linalg.solve(kkt, rhs)
    except np.linalg.LinAlgError:
        sol = np.linalg.lstsq(kkt, rhs, rcond=None)[0]
    return sol[0:3], sol[3:6], sol[6:9], sol[9:12]


def _sum_foot_forces(model: mj.MjModel, data: mj.MjData, spec: dict[str, object]) -> dict[str, list[float]]:
    """World force on each foot from floor contacts. Positive z is upward."""
    grounds = spec["grounds"]
    gids = {"l": int(spec["l"]), "r": int(spec["r"])}
    total = {"l": np.zeros(3, dtype=np.float64), "r": np.zeros(3, dtype=np.float64)}
    if not isinstance(grounds, set):
        return {"l": [0.0, 0.0, 0.0], "r": [0.0, 0.0, 0.0]}
    for i in range(int(data.ncon)):
        con = data.contact[i]
        g1 = int(con.geom1)
        g2 = int(con.geom2)
        side = ""
        for name, gid in gids.items():
            if g1 == gid and g2 in grounds:
                side = name
                break
            if g2 == gid and g1 in grounds:
                side = name
                break
        if not side:
            continue
        fr = np.zeros(6, dtype=np.float64)
        mj.mj_contactForce(model, data, i, fr)
        frame = np.array(con.frame, dtype=np.float64).reshape(3, 3)
        total[side] += frame.T @ fr[:3]
    return {side: [float(v) for v in total[side]] for side in ("l", "r")}


def _foot_corners(model: mj.MjModel, data: mj.MjData, side: str) -> tuple[np.ndarray, list[np.ndarray]]:
    """Sole rotation and the four bottom corners of the 135×76 mm box."""
    gid = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, f"{side}_foot_contact"))
    if gid < 0:
        raise RuntimeError(f"missing {side}_foot_contact")
    hx = float(model.geom_size[gid, 0])
    hy = float(model.geom_size[gid, 1])
    hz = float(model.geom_size[gid, 2])
    if abs(hx - QP_HALF_X_M) > 1e-6 or abs(hy - QP_HALF_Y_M) > 1e-6:
        raise RuntimeError(
            f"{side} foot box {2 * hx * 1000:.1f}×{2 * hy * 1000:.1f} mm is not 135×76"
        )
    centre = np.array(data.geom_xpos[gid], dtype=np.float64)
    rot = np.array(data.geom_xmat[gid], dtype=np.float64).reshape(3, 3).copy()
    points = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            local = np.array([sx * hx, sy * hy, -hz], dtype=np.float64)
            points.append(centre + rot @ local)
    return rot, points


def _corner_locals() -> list[tuple[float, float]]:
    """Local xy of the eight sole corners, left foot then right.

    Order matches ``_foot_corners``: sx outer, sy inner.
    """
    hx = QP_HALF_X_M
    hy = QP_HALF_Y_M
    one = ((-hx, -hy), (-hx, hy), (hx, -hy), (hx, hy))
    return list(one) + list(one)


def _cop_inset_limits() -> tuple[float, float, float]:
    """Inset and the shrunk half-extents, both in metres.

    The inset is ``step_bars.EDGE_DWELL_M``. A non-positive shrunk box is
    a scorer error, not a silent fallback to the full sole.
    """
    inset = float(step_bars.EDGE_DWELL_M)
    if inset != QP_COP_INSET_M:
        raise RuntimeError(
            f"QP CoP inset {QP_COP_INSET_M} m drifted from step_bars.EDGE_DWELL_M {inset} m"
        )
    hx_in = QP_HALF_X_M - inset
    hy_in = QP_HALF_Y_M - inset
    if hx_in <= 1e-6 or hy_in <= 1e-6:
        raise RuntimeError(
            f"CoP inset {inset} m does not fit inside the 135×76 mm sole"
        )
    return inset, hx_in, hy_in


def _add_cop_inset_rows(n_var: int, add) -> None:
    """Linear inequalities that keep each foot's CoP inside the shrunk box."""
    _inset, hx_in, hy_in = _cop_inset_limits()
    xy = _corner_locals()
    for start in (0, 4):
        for sign, axis, limit in (
            (1.0, 0, hx_in),
            (-1.0, 0, hx_in),
            (1.0, 1, hy_in),
            (-1.0, 1, hy_in),
        ):
            row = np.zeros(n_var, dtype=np.float64)
            for c in range(start, start + 4):
                row[3 * c + 2] = sign * xy[c][axis] - limit
            add(row, 0.0)


def _qp_solve_minmax(
    rotations: list[np.ndarray],
    points: list[np.ndarray],
    coef: np.ndarray,
    b_leg: np.ndarray,
    force: np.ndarray,
    p_zmp: np.ndarray,
) -> dict[str, object]:
    """Minimise the largest |leg torque| with armature already removed.

    Corner forces of both feet are the variables. The summed wrench equals
    ``force`` at ``p_zmp`` with zero moment about that point. Each corner
    stays in the friction cone μ = 1.2. Each foot's centre of pressure stays
    inside the 135×76 mm sole shrunk by ``step_bars.EDGE_DWELL_M`` on every
    side. The program is the linear epigraph of that maximum on the
    containing pyramid. A solution that leaves the disk is resolved on the
    disk. Among those optima, a further solve minimises the sum of tangential
    magnitudes so the reported split is one torque-optimal wrench.
    """
    n_c = 8
    n_leg = int(b_leg.shape[0])
    n_f = n_c * 3
    n_u = n_c * 2
    t_idx = n_f + n_u
    n_var = t_idx + 1
    a_eq = np.zeros((6, n_var), dtype=np.float64)
    for c in range(n_c):
        cols = slice(3 * c, 3 * c + 3)
        rot = rotations[c]
        a_eq[0:3, cols] = rot
        a_eq[3:6, cols] = _skew(points[c] - p_zmp) @ rot
    b_eq = np.zeros(6, dtype=np.float64)
    b_eq[0:3] = force
    rows: list[np.ndarray] = []
    rhs: list[float] = []

    def _add(row: np.ndarray, limit: float) -> None:
        rows.append(row)
        rhs.append(float(limit))

    mu = QP_MU
    for c in range(n_c):
        base = 3 * c
        u_fx = n_f + 2 * c
        u_fy = u_fx + 1
        for sign, axis, scale_z in (
            (1.0, 0, -mu),
            (-1.0, 0, -mu),
            (1.0, 1, -mu),
            (-1.0, 1, -mu),
        ):
            row = np.zeros(n_var, dtype=np.float64)
            row[base + axis] = sign
            row[base + 2] = scale_z
            _add(row, 0.0)
        row = np.zeros(n_var, dtype=np.float64)
        row[base + 2] = -1.0
        _add(row, 0.0)
        for axis, u_idx in ((0, u_fx), (1, u_fy)):
            pos = np.zeros(n_var, dtype=np.float64)
            pos[base + axis] = 1.0
            pos[u_idx] = -1.0
            _add(pos, 0.0)
            neg = np.zeros(n_var, dtype=np.float64)
            neg[base + axis] = -1.0
            neg[u_idx] = -1.0
            _add(neg, 0.0)
    _add_cop_inset_rows(n_var, _add)
    for j in range(n_leg):
        pos = np.zeros(n_var, dtype=np.float64)
        neg = np.zeros(n_var, dtype=np.float64)
        for c in range(n_c):
            block = coef[j, c]
            pos[3 * c:3 * c + 3] = block
            neg[3 * c:3 * c + 3] = -block
        pos[t_idx] = -1.0
        neg[t_idx] = -1.0
        _add(pos, float(b_leg[j]))
        _add(neg, -float(b_leg[j]))
    a_ub = np.vstack(rows)
    b_ub = np.asarray(rhs, dtype=np.float64)
    bounds = [(None, None)] * n_f + [(0.0, None)] * n_u + [(0.0, None)]
    c_obj = np.zeros(n_var, dtype=np.float64)
    c_obj[t_idx] = 1.0

    def _run(obj: np.ndarray, ub: np.ndarray, bb: np.ndarray) -> np.ndarray | None:
        try:
            res = linprog(obj, A_ub=ub, b_ub=bb, A_eq=a_eq, b_eq=b_eq, bounds=bounds, method="highs")
        except (ValueError, np.linalg.LinAlgError):
            return None
        if not res.success or res.x is None:
            return None
        return np.asarray(res.x, dtype=np.float64)

    x1 = _run(c_obj, a_ub, b_ub)
    if x1 is None:
        # Distinguish an infeasible wrench from a solver failure.
        try:
            probe = linprog(
                c_obj, A_ub=a_ub, b_ub=b_ub, A_eq=a_eq, b_eq=b_eq, bounds=bounds, method="highs",
            )
            status = "infeasible" if int(probe.status) == 2 else "unsolved"
        except (ValueError, np.linalg.LinAlgError):
            status = "unsolved"
        return {"status": status}
    t_star = float(x1[t_idx])
    cap_row = np.zeros(n_var, dtype=np.float64)
    cap_row[t_idx] = 1.0
    a2 = np.vstack([a_ub, cap_row])
    b2 = np.concatenate([b_ub, np.array([t_star + 1e-7])])
    c2 = np.zeros(n_var, dtype=np.float64)
    c2[n_f:n_f + n_u] = 1.0
    x2 = _run(c2, a2, b2)
    x = x1 if x2 is None else x2
    forces1 = x1[:n_f].reshape(n_c, 3)
    forces = x[:n_f].reshape(n_c, 3)
    eq = a_eq[:, :n_f] @ forces.reshape(-1) - b_eq
    if float(np.max(np.abs(eq))) > 1e-4:
        return {"status": "unsolved"}

    def _disk_excess(arr: np.ndarray) -> float:
        excess = 0.0
        for c in range(n_c):
            fx, fy, fz = (float(arr[c, 0]), float(arr[c, 1]), float(arr[c, 2]))
            if fz < -1e-6 or abs(fx) > mu * max(fz, 0.0) + 1e-5 or abs(fy) > mu * max(fz, 0.0) + 1e-5:
                return -1.0
            excess = max(excess, math.hypot(fx, fy) - mu * max(fz, 0.0))
        return excess

    excess1 = _disk_excess(forces1)
    if excess1 < 0.0:
        return {"status": "unsolved"}
    excess = _disk_excess(forces)
    if excess < 0.0:
        forces = forces1
        excess = excess1
        x2 = None
    # Stage 1 already lies in the disk, so its t is the cone minimax.
    # Keep stage 2 only when that wrench stayed in the disk too.
    if excess1 <= 1e-6:
        if excess > 1e-6:
            forces = forces1
            excess = excess1
            x2 = None
        return {
            "status": "feasible",
            "forces": forces,
            "t_star": t_star,
            "from_stage2": x2 is not None and excess <= 1e-6,
            "cone_excess_n": excess,
            "cone_resolved": False,
        }
    refined = _cone_refine(
        forces1, t_star, rotations, points, coef, b_leg, force, p_zmp,
    )
    if refined is None:
        return {"status": "unsolved"}
    refined["cone_resolved"] = True
    return refined


def _cone_refine(
    forces_lp: np.ndarray,
    t_lp: float,
    rotations: list[np.ndarray],
    points: list[np.ndarray],
    coef: np.ndarray,
    b_leg: np.ndarray,
    force: np.ndarray,
    p_zmp: np.ndarray,
) -> dict[str, object] | None:
    """Re-solve on the friction disk when the pyramid solution leaves it.

    The disk is a subset of the pyramid, so the minimax can only rise.
    The returned ``t_star`` is that disk minimum. The forces minimise the
    sum of tangential magnitudes among disk solutions within 1e-7 Nm of it.
    The shrunk CoP box stays in the constraint set.
    """
    n_c = 8
    n_leg = int(b_leg.shape[0])
    n_f = n_c * 3
    n = n_f + 1
    mu = QP_MU
    a_eq = np.zeros((6, n), dtype=np.float64)
    for c in range(n_c):
        cols = slice(3 * c, 3 * c + 3)
        a_eq[0:3, cols] = rotations[c]
        a_eq[3:6, cols] = _skew(points[c] - p_zmp) @ rotations[c]
    b_eq = np.zeros(6, dtype=np.float64)
    b_eq[0:3] = force
    flat = np.asarray(coef, dtype=np.float64).reshape(n_leg, n_f)
    x0 = np.zeros(n, dtype=np.float64)
    x0[:n_f] = np.asarray(forces_lp, dtype=np.float64).reshape(-1)
    for c in range(n_c):
        fx, fy, fz = (float(x0[3 * c]), float(x0[3 * c + 1]), float(x0[3 * c + 2]))
        if fz < 0.0:
            x0[3 * c:3 * c + 3] = 0.0
            continue
        hyp = math.hypot(fx, fy)
        limit = mu * fz
        if hyp > limit and hyp > 0.0:
            scale = limit / hyp
            x0[3 * c] *= scale
            x0[3 * c + 1] *= scale
    acc0 = flat @ x0[:n_f]
    x0[-1] = max(float(t_lp), float(np.max(np.abs(b_leg - acc0)))) + 1e-6
    lb = []
    ub = []
    for _c in range(n_c):
        lb.extend([-np.inf, -np.inf, 0.0])
        ub.extend([np.inf, np.inf, np.inf])
    lb.append(0.0)
    ub.append(np.inf)
    bounds = Bounds(lb, ub)

    def _torque(x: np.ndarray) -> np.ndarray:
        delta = b_leg - flat @ x[:n_f]
        return np.concatenate([x[-1] - delta, x[-1] + delta])

    def _torque_jac(x: np.ndarray) -> np.ndarray:
        jac = np.zeros((2 * n_leg, n), dtype=np.float64)
        jac[:n_leg, :n_f] = flat
        jac[n_leg:, :n_f] = -flat
        jac[:, -1] = 1.0
        return jac

    def _friction(x: np.ndarray) -> np.ndarray:
        out = np.empty(n_c, dtype=np.float64)
        for c in range(n_c):
            fx, fy, fz = (float(x[3 * c]), float(x[3 * c + 1]), float(x[3 * c + 2]))
            out[c] = mu * fz - math.hypot(fx, fy)
        return out

    def _friction_jac(x: np.ndarray) -> np.ndarray:
        jac = np.zeros((n_c, n), dtype=np.float64)
        for c in range(n_c):
            fx, fy = (float(x[3 * c]), float(x[3 * c + 1]))
            hyp = math.hypot(fx, fy)
            if hyp > 1e-10:
                jac[c, 3 * c] = -fx / hyp
                jac[c, 3 * c + 1] = -fy / hyp
            jac[c, 3 * c + 2] = mu
        return jac

    def _eq(x: np.ndarray) -> np.ndarray:
        return a_eq @ x - b_eq

    def _eq_jac(x: np.ndarray) -> np.ndarray:
        return a_eq

    _inset, hx_in, hy_in = _cop_inset_limits()
    xy = _corner_locals()
    cop_coef = np.zeros((8, n_c), dtype=np.float64)
    cop_row = 0
    for start in (0, 4):
        for sign, axis, limit in (
            (1.0, 0, hx_in),
            (-1.0, 0, hx_in),
            (1.0, 1, hy_in),
            (-1.0, 1, hy_in),
        ):
            for c in range(start, start + 4):
                cop_coef[cop_row, c] = limit - sign * xy[c][axis]
            cop_row += 1

    def _cop(x: np.ndarray) -> np.ndarray:
        out = np.zeros(8, dtype=np.float64)
        for row in range(8):
            for c in range(n_c):
                out[row] += cop_coef[row, c] * float(x[3 * c + 2])
        return out

    def _cop_jac(x: np.ndarray) -> np.ndarray:
        jac = np.zeros((8, n), dtype=np.float64)
        for row in range(8):
            for c in range(n_c):
                jac[row, 3 * c + 2] = cop_coef[row, c]
        return jac

    cons = (
        {"type": "eq", "fun": _eq, "jac": _eq_jac},
        {"type": "ineq", "fun": _torque, "jac": _torque_jac},
        {"type": "ineq", "fun": _friction, "jac": _friction_jac},
        {"type": "ineq", "fun": _cop, "jac": _cop_jac},
    )

    def _obj(x: np.ndarray) -> float:
        return float(x[-1])

    def _obj_jac(x: np.ndarray) -> np.ndarray:
        grad = np.zeros(n, dtype=np.float64)
        grad[-1] = 1.0
        return grad

    def _accept(res: object, t_cap: float | None) -> np.ndarray | None:
        if getattr(res, "x", None) is None:
            return None
        x = np.asarray(res.x, dtype=np.float64)
        if float(np.max(np.abs(_eq(x)))) > 1e-4:
            return None
        if float(np.min(_friction(x))) < -5e-4:
            return None
        if float(np.min(_torque(x))) < -1e-5:
            return None
        if float(np.min(_cop(x))) < -1e-4:
            return None
        if float(x[-1]) + 1e-5 < float(t_lp):
            return None
        if t_cap is not None and float(x[-1]) > t_cap + 1e-5:
            return None
        return x

    try:
        res1 = minimize(
            _obj, x0, jac=_obj_jac, method="SLSQP", bounds=bounds, constraints=cons,
            options={"maxiter": 400, "ftol": 1e-10},
        )
    except (ValueError, np.linalg.LinAlgError):
        return None
    x1 = _accept(res1, None)
    if x1 is None:
        return None
    t_star = float(x1[-1])
    ub[-1] = t_star + 1e-7
    bounds2 = Bounds(lb, ub)

    def _tangential(x: np.ndarray) -> float:
        total = 0.0
        for c in range(n_c):
            total += math.hypot(float(x[3 * c]), float(x[3 * c + 1]))
        return total

    def _tangential_jac(x: np.ndarray) -> np.ndarray:
        grad = np.zeros(n, dtype=np.float64)
        for c in range(n_c):
            fx, fy = (float(x[3 * c]), float(x[3 * c + 1]))
            hyp = math.hypot(fx, fy)
            if hyp > 1e-10:
                grad[3 * c] = fx / hyp
                grad[3 * c + 1] = fy / hyp
        return grad

    try:
        res2 = minimize(
            _tangential, x1, jac=_tangential_jac, method="SLSQP", bounds=bounds2,
            constraints=cons, options={"maxiter": 80, "ftol": 1e-12},
        )
    except (ValueError, np.linalg.LinAlgError):
        res2 = None
    x2 = None if res2 is None else _accept(res2, t_star + 1e-7)
    x = x1 if x2 is None else x2
    for c in range(n_c):
        fx, fy, fz = (float(x[3 * c]), float(x[3 * c + 1]), float(x[3 * c + 2]))
        limit = mu * max(fz, 0.0)
        hyp = math.hypot(fx, fy)
        if hyp > limit and hyp > 0.0:
            scale = limit / hyp
            x[3 * c] *= scale
            x[3 * c + 1] *= scale
    if (
        float(np.max(np.abs(_eq(x)))) > 1e-4
        or float(np.min(_torque(x))) < -1e-4
        or float(np.min(_cop(x))) < -1e-4
    ):
        return None
    forces = x[:n_f].reshape(n_c, 3)
    excess = 0.0
    for c in range(n_c):
        fx, fy, fz = (float(forces[c, 0]), float(forces[c, 1]), float(forces[c, 2]))
        excess = max(excess, math.hypot(fx, fy) - mu * max(fz, 0.0))
    return {
        "status": "feasible",
        "forces": forces,
        "t_star": t_star,
        "from_stage2": x2 is not None,
        "cone_excess_n": excess,
    }


def _qp_foot_split(
    model: mj.MjModel,
    data: mj.MjData,
    foot_ids: dict[str, int],
    p_zmp: np.ndarray,
    force: np.ndarray,
    b_no_arm: np.ndarray,
    leg_order: list[tuple[str, int]],
) -> dict[str, object]:
    """QP split for one double-support tick. ``b_no_arm`` is per dof."""
    rotations: list[np.ndarray] = []
    points: list[np.ndarray] = []
    jacps: list[np.ndarray] = []
    locals_xy: list[tuple[float, float]] = []
    for side in ("l", "r"):
        rot, corners = _foot_corners(model, data, side)
        hx = QP_HALF_X_M
        hy = QP_HALF_Y_M
        xy = ((-hx, -hy), (-hx, hy), (hx, -hy), (hx, hy))
        # _foot_corners walks sx outer, sy inner: (-hx,-hy), (-hx,hy), (hx,-hy), (hx,hy).
        for point, local in zip(corners, xy):
            jacp = np.zeros((3, model.nv), dtype=np.float64)
            jacr = np.zeros((3, model.nv), dtype=np.float64)
            mj.mj_jac(model, data, jacp, jacr, np.asarray(point, dtype=np.float64), int(foot_ids[side]))
            rotations.append(rot)
            points.append(point)
            jacps.append(jacp)
            locals_xy.append(local)
    n_leg = len(leg_order)
    coef = np.zeros((n_leg, 8, 3), dtype=np.float64)
    b_leg = np.zeros(n_leg, dtype=np.float64)
    for j, (_name, adr) in enumerate(leg_order):
        b_leg[j] = float(b_no_arm[adr])
        for c, jacp in enumerate(jacps):
            coef[j, c, :] = rotations[c].T @ jacp[:, adr]
    solved = _qp_solve_minmax(rotations, points, coef, b_leg, force, p_zmp)
    if solved["status"] != "feasible":
        return {"status": solved["status"]}
    forces = np.asarray(solved["forces"], dtype=np.float64)
    gen = np.zeros(model.nv, dtype=np.float64)
    world = {"l": np.zeros(3, dtype=np.float64), "r": np.zeros(3, dtype=np.float64)}
    inset, hx_in, hy_in = _cop_inset_limits()
    cop_ok = True
    cop_margin: dict[str, float | None] = {}
    cop_xy: dict[str, list[float] | None] = {}
    cop_on: dict[str, bool] = {}
    for side, start in (("l", 0), ("r", 4)):
        fz = 0.0
        cop_x = 0.0
        cop_y = 0.0
        for c in range(start, start + 4):
            f_world = rotations[c] @ forces[c]
            world[side] += f_world
            gen += jacps[c].T @ f_world
            weight = float(forces[c, 2])
            fz += weight
            cop_x += locals_xy[c][0] * weight
            cop_y += locals_xy[c][1] * weight
        if fz > 1e-6:
            cx = cop_x / fz
            cy = cop_y / fz
            margin = min(QP_HALF_X_M - abs(cx), QP_HALF_Y_M - abs(cy))
            cop_margin[side] = float(margin)
            cop_xy[side] = [float(cx), float(cy)]
            cop_on[side] = abs(float(margin) - inset) <= QP_COP_BOUND_TOL_M
            if abs(cx) > hx_in + 1e-4 or abs(cy) > hy_in + 1e-4:
                cop_ok = False
        else:
            cop_margin[side] = None
            cop_xy[side] = None
            cop_on[side] = False
    if not cop_ok:
        return {"status": "unsolved"}
    return {
        "status": "feasible",
        "gen": gen,
        "f": world,
        "t_star": float(solved["t_star"]),
        "cone_excess_n": float(solved.get("cone_excess_n", 0.0)),
        "cone_resolved": bool(solved.get("cone_resolved")),
        "cop": {
            "margin_m": cop_margin,
            "xy_m": cop_xy,
            "on_boundary": cop_on,
            "inset_m": inset,
        },
    }


def _contact_copy(model: mj.MjModel) -> mj.MjModel:
    """Binary copy with contacts and inverse flags off. The source stays put."""
    flags = int(model.opt.enableflags)
    fd, path = tempfile.mkstemp(prefix="a5-plan-", suffix=".mjb")
    os.close(fd)
    try:
        mj.mj_saveModel(model, path)
        copied = mj.MjModel.from_binary_path(path)
    finally:
        os.unlink(path)
    if int(model.opt.enableflags) != flags:
        raise RuntimeError("the planned-motion copy changed the live enableflags")
    copied.opt.enableflags = 0
    copied.geom_contype[:] = 0
    copied.geom_conaffinity[:] = 0
    if int(model.opt.enableflags) != flags:
        raise RuntimeError("the planned-motion copy shares enableflags with the live model")
    return copied


def _clone_model(model: mj.MjModel) -> mj.MjModel:
    fd, path = tempfile.mkstemp(prefix="a5-swing-", suffix=".mjb")
    os.close(fd)
    try:
        mj.mj_saveModel(model, path)
        return mj.MjModel.from_binary_path(path)
    finally:
        os.unlink(path)


def _qfrc_at(
    model: mj.MjModel,
    data: mj.MjData,
    pose: np.ndarray,
    qvel: np.ndarray,
    qacc: np.ndarray,
) -> np.ndarray:
    data.qpos[:] = pose
    data.qvel[:] = qvel
    data.qacc[:] = qacc
    mj.mj_inverse(model, data)
    return np.array(data.qfrc_inverse, dtype=np.float64, copy=True)


def _zero_swing_links(model: mj.MjModel, swing: str) -> list[str]:
    """Drop the swing leg's link mass and inertia. Armature stays on the dof."""
    names: list[str] = []
    prefix = f"{swing}_"
    for bid in range(int(model.nbody)):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, bid) or ""
        if not name.startswith(prefix):
            continue
        if not any(tok in name for tok in ("hip", "knee", "ank")):
            continue
        model.body_mass[bid] = 0.0
        model.body_inertia[bid, :] = 0.0
        names.append(name)
    if not names:
        raise RuntimeError(f"swing leg {swing} has no links to drop")
    mj.mj_setConst(model, mj.MjData(model))
    return names


def _best_force_point(
    model: mj.MjModel,
    data: mj.MjData,
    body_id: int,
    force: np.ndarray,
    ident: np.ndarray,
    centre: np.ndarray,
) -> dict[str, object]:
    """Point on z = 0 whose pure force leaves the smallest root residual."""
    mj.mj_fwdPosition(model, data)
    zero = np.zeros(3, dtype=np.float64)
    best = None
    # Coarse, then a 1 mm patch. The map is the moment arm of one force.
    for step, span in ((0.004, 0.08),):
        xs = np.arange(float(centre[0]) - span, float(centre[0]) + span + 0.5 * step, step)
        ys = np.arange(float(centre[1]) - span, float(centre[1]) + span + 0.5 * step, step)
        for x in xs:
            for y in ys:
                point = np.array([float(x), float(y), 0.0], dtype=np.float64)
                gen = _jac_wrench(model, data, point, body_id, force, zero)
                resid = ident - gen
                score = float(np.max(np.abs(resid[:ROOT_DOFS])))
                if best is None or score < float(best["root_max_nm"]):
                    best = {
                        "point_m": [float(x), float(y), 0.0],
                        "root_max_nm": score,
                        "resid_nm": [float(v) for v in resid[:ROOT_DOFS]],
                        "gen": gen,
                    }
    assert best is not None
    px, py, _ = best["point_m"]
    for x in np.arange(px - 0.004, px + 0.004 + 1e-9, 0.001):
        for y in np.arange(py - 0.004, py + 0.004 + 1e-9, 0.001):
            point = np.array([float(x), float(y), 0.0], dtype=np.float64)
            gen = _jac_wrench(model, data, point, body_id, force, zero)
            resid = ident - gen
            score = float(np.max(np.abs(resid[:ROOT_DOFS])))
            if score < float(best["root_max_nm"]):
                best = {
                    "point_m": [float(x), float(y), 0.0],
                    "root_max_nm": score,
                    "resid_nm": [float(v) for v in resid[:ROOT_DOFS]],
                    "gen": gen,
                }
    return best


def _hip_roll_parts(
    model: mj.MjModel,
    pose: np.ndarray,
    qvel: np.ndarray,
    qacc: np.ndarray,
    stance: str,
    force: np.ndarray,
    gen_planned: np.ndarray,
    gen_best: np.ndarray | None,
    dofadr: dict[str, int],
    com: np.ndarray,
    zmp_y: float,
    box: dict[str, object],
) -> dict[str, object]:
    """Stance hip-roll τ = gravity + M·q̈ + velocity + contact.

    Gravity is ``qfrc_inverse`` at q̇ = 0, q̈ = 0. Inertial is the same
    call with the planned q̈, minus gravity, so it is M·q̈ including
    armature. Velocity is the remainder of the full inverse. Contact is
    minus the generalized force of the planned wrench. Swing-leg is the
    change in those three when the swing links' mass and inertia are
    zero. τ magnitude is not a discard.
    """
    joint = f"{stance}_hip_roll"
    adr = int(dofadr[joint])
    data = mj.MjData(model)
    zeros = np.zeros_like(qvel)
    full = _qfrc_at(model, data, pose, qvel, qacc)
    grav = _qfrc_at(model, data, pose, zeros, zeros)
    inert_and_grav = _qfrc_at(model, data, pose, zeros, qacc)
    inert = inert_and_grav - grav
    vel = full - inert_and_grav
    contact = -np.asarray(gen_planned, dtype=np.float64)
    tau = full + contact
    arm = float(model.dof_armature[adr]) * float(qacc[adr])
    swing = "l" if stance == "r" else "r"
    swung = _clone_model(model)
    dropped = _zero_swing_links(swung, swing)
    sdata = mj.MjData(swung)
    s_full = _qfrc_at(swung, sdata, pose, qvel, qacc)
    s_grav = _qfrc_at(swung, sdata, pose, zeros, zeros)
    s_ig = _qfrc_at(swung, sdata, pose, zeros, qacc)
    s_inert = s_ig - s_grav
    s_vel = s_full - s_ig
    # Vertical support anywhere on the sole. The joint axis at this pose
    # dots the moment, so the range is what a pure fz can do.
    mj.mj_fwdPosition(model, data)
    jid = int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint))
    hip = np.array(data.xanchor[jid], dtype=np.float64)
    foot_id = int(model.geom_bodyid[int(mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, f"{stance}_foot_contact"))])
    fz = float(force[2])
    vertical = np.array([0.0, 0.0, fz], dtype=np.float64)
    zero_m = np.zeros(3, dtype=np.float64)
    static_tau = []
    corners = np.asarray(box["corners"], dtype=np.float64)
    for corner in corners:
        gen_c = _jac_wrench(model, data, corner, foot_id, vertical, zero_m)
        static_tau.append(float(grav[adr] - gen_c[adr]))
    best_tau = None
    best_bare = None
    if gen_best is not None:
        best_tau = float(full[adr] - gen_best[adr])
        best_bare = best_tau - arm
    # The swing leg is on the pelvis side of this hip, so it does not
    # appear in qfrc_bias. It changes the root force the foot has to carry.
    p_zmp = np.array([float(com[0]), float(zmp_y), 0.0], dtype=np.float64)
    gen_swingless = _jac_wrench(model, data, p_zmp, foot_id, s_full[0:3], zero_m)
    tau_swingless = float(s_full[adr] - gen_swingless[adr])
    swing_wrench = float(tau[adr] - tau_swingless)
    parts = {
        "joint": joint,
        "gravity_nm": float(grav[adr]),
        "inertial_nm": float(inert[adr]),
        "inertial_bare_nm": float(inert[adr] - arm),
        "velocity_nm": float(vel[adr]),
        "contact_nm": float(contact[adr]),
        "swing_gravity_nm": float(grav[adr] - s_grav[adr]),
        "swing_inertial_nm": float(inert[adr] - s_inert[adr]),
        "swing_velocity_nm": float(vel[adr] - s_vel[adr]),
        "swing_wrench_nm": swing_wrench,
        "body_gravity_nm": float(s_grav[adr]),
        "body_inertial_nm": float(s_inert[adr]),
        "body_velocity_nm": float(s_vel[adr]),
        "tau_nm": float(tau[adr]),
        "tau_bare_nm": float(tau[adr] - arm),
        "armature_qdd_nm": arm,
        "sum_nm": float(grav[adr] + inert[adr] + vel[adr] + contact[adr]),
        "hip_axis_y_m": float(hip[1]),
        "box_centre_y_m": float(box["y_m"]),
        "box_y_lo_m": float(box["y_lo_m"]),
        "box_y_hi_m": float(box["y_hi_m"]),
        "planned_com_y_m": float(com[1]),
        "planned_zmp_y_m": float(zmp_y),
        "static_vertical_nm": [float(min(static_tau)), float(max(static_tau))],
        "tau_at_best_point_nm": best_tau,
        "tau_bare_at_best_point_nm": best_bare,
        "swing_links_dropped": dropped,
        "q_hip_roll_rad": float(pose[int(model.jnt_qposadr[jid])]),
    }
    parts["closes_nm"] = float(parts["sum_nm"] - parts["tau_nm"])
    return parts


def _cycle_rows(rows: list[dict[str, object]], period_s: float) -> list[dict[str, object]]:
    """Second walk cycle when the bout has one. Otherwise the first period of walk."""
    walk = [row for row in rows if row.get("phase") == "walk"]
    if len(walk) < 4:
        return rows
    t0 = float(walk[0]["t_s"])
    period = float(period_s)
    second = [
        row for row in walk
        if t0 + period - 1e-9 <= float(row["t_s"]) < t0 + 2.0 * period - 1e-9
    ]
    if len(second) >= 8:
        return second
    return [
        row for row in walk
        if t0 - 1e-9 <= float(row["t_s"]) < t0 + period - 1e-9
    ]


def _print_plan_y(trace: dict[str, object]) -> None:
    """Root rows for every tick, then one cycle of the y channels."""
    ticks = trace.get("ticks")
    if not isinstance(ticks, list):
        return
    print(
        "planned root rows: qfrc is mj_inverse on the reference; "
        "resid is qfrc_inverse - J^T f at the planned ZMP. resid should be ~0",
        flush=True,
    )
    print(
        "t phase support stance "
        "qfrc_fx qfrc_fy qfrc_fz qfrc_tx qfrc_ty qfrc_tz "
        "resid_fx resid_fy resid_fz resid_tx resid_ty resid_tz",
        flush=True,
    )
    for row in ticks:
        qf = row["qfrc_root_nm"]
        rs = row["resid_root_nm"]
        if not isinstance(qf, list) or not isinstance(rs, list):
            continue
        print(
            f"{float(row['t_s']):.3f} {row['phase']} {row['support']} {row['stance'] or '-'} "
            + " ".join(f"{float(v):+.4f}" for v in qf)
            + " "
            + " ".join(f"{float(v):+.4f}" for v in rs),
            flush=True,
        )
    cycle = trace.get("cycle")
    print(
        "one cycle: preview_com_y planned_com_y planned_zmp_y "
        "stance_box_y hip_roll_axis_y live_com_y live_cop_y",
        flush=True,
    )
    if isinstance(cycle, list):
        for row in cycle:
            box_y = row.get("stance_box_y_m")
            hip_y = row.get("stance_hip_y_m")
            live_cop = row.get("live_cop_y_m")
            preview = row.get("preview_com_y_m")
            print(
                f"{float(row['t_s']):.3f} {row['support']} {row['stance'] or '-'} "
                f"preview {float(preview) if preview is not None else float('nan'):+.4f} "
                f"com {float(row['planned_com_y_m']):+.4f} "
                f"zmp {float(row['planned_zmp_y_m']):+.4f} "
                f"box {float(box_y) if box_y is not None else float('nan'):+.4f} "
                f"hip {float(hip_y) if hip_y is not None else float('nan'):+.4f} "
                f"live_com {float(row['live_com_y_m']) if row.get('live_com_y_m') is not None else float('nan'):+.4f} "
                f"live_cop {float(live_cop) if live_cop is not None else float('nan'):+.4f} "
                f"out {int(bool(row['com_outside']))}/{int(bool(row['zmp_outside']))}",
                flush=True,
            )
    summary = trace.get("summary")
    if isinstance(summary, dict):
        print(
            "plan-y "
            f"root_max {summary.get('root_resid_max_nm')} "
            f"root_inconsistent {summary.get('root_inconsistent')} "
            f"support_inconsistent {summary.get('support_inconsistent')} "
            f"inconsistent {summary.get('inconsistent')} "
            f"tau_not_a_discard {summary.get('tau_is_not_a_discard')}",
            flush=True,
        )
    mid = trace.get("stance_mid")
    if isinstance(mid, dict):
        parts = mid.get("parts")
        print(
            "stance mid "
            f"t {mid.get('t_s')} joint {mid.get('joint')} "
            f"bare {mid.get('tau_bare_nm')} tau {mid.get('tau_nm')}",
            flush=True,
        )
        if isinstance(parts, dict):
            print(
                "decompose "
                f"grav {parts.get('gravity_nm')} "
                f"inert {parts.get('inertial_nm')} "
                f"vel {parts.get('velocity_nm')} "
                f"contact {parts.get('contact_nm')} "
                f"swing_grav {parts.get('swing_gravity_nm')} "
                f"swing_inert {parts.get('swing_inertial_nm')} "
                f"swing_vel {parts.get('swing_velocity_nm')} "
                f"swing_wrench {parts.get('swing_wrench_nm')} "
                f"static {parts.get('static_vertical_nm')} "
                f"bare_at_best_point {parts.get('tau_bare_at_best_point_nm')} "
                f"closes {parts.get('closes_nm')}",
                flush=True,
            )


def _planned_tau(
    model: mj.MjModel,
    samples: list[dict[str, object]],
    qadr: dict[str, int],
    dofadr: dict[str, int],
    trace: bool = False,
    trace_period_s: float | None = None,
) -> dict[str, object]:
    """τ_req from the walker's reference, contacts replaced by the ZMP wrench.

    ``samples`` are one control tick each. q_ref is the walker's q_des.
    q̇_ref and q̈_ref are backward differences at 8 ms. The floating base is
    placed so the planned stance foot stays on its foothold and the sole
    is on z = 0. Double support reports the linear foot-line split, the
    min-norm ankle split, and the QP minimax split. Single support has one
    wrench, so the three columns match and the spread is 0.
    """
    empty = {
        "phases": {name: {} for name in PLAN_PHASES},
        "candidates": [],
        "root_residual_max_nm": None,
        "n_ticks": 0,
    }
    if len(samples) < 3:
        return empty
    plan = _contact_copy(model)
    data = mj.MjData(plan)
    nv = int(plan.nv)
    dt = 0.008
    mass = float(np.sum(plan.body_mass))
    anchor: dict[str, np.ndarray] = {}
    held = {"support": "", "stance": ""}
    shift_log = {"n_ok": 0, "n_reject": 0, "max_attempt_m": 0.0}
    poses: list[np.ndarray] = []
    soles: list[dict[str, np.ndarray]] = []
    ankles: list[dict[str, np.ndarray]] = []
    ankle_ids: dict[str, int] = {}
    foot_ids: dict[str, int] = {}
    trace_rows: list[dict[str, object]] = []
    trace_aux: list[dict[str, object]] = []
    hip_jnt = {
        side: int(mj.mj_name2id(plan, mj.mjtObj.mjOBJ_JOINT, f"{side}_hip_roll"))
        for side in ("l", "r")
    }
    for side, body in (("l", "l_ank_roll_link"), ("r", "r_ank_roll_link")):
        ankle_ids[side], _ = _body_point(plan, data, body)
        gid = int(mj.mj_name2id(plan, mj.mjtObj.mjOBJ_GEOM, f"{side}_foot_contact"))
        foot_ids[side] = int(plan.geom_bodyid[gid])

    def _place(q_map: dict[str, float], support: str, stance: str) -> None:
        data.qpos[:] = plan.qpos0
        data.qvel[:] = 0.0
        data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
        for name, value in q_map.items():
            data.qpos[qadr[name]] = float(value)
        data.qpos[2] = 0.30
        mj.mj_forward(plan, data)
        sole = {side: _sole_point(plan, data, side) for side in ("l", "r")}
        if support == "ss" and stance in sole:
            drop = float(sole[stance][2])
        else:
            drop = 0.5 * (float(sole["l"][2]) + float(sole["r"][2]))
        data.qpos[2] -= drop
        mj.mj_forward(plan, data)
        sole = {side: _sole_point(plan, data, side) for side in ("l", "r")}

        def _shift(delta_xy: np.ndarray) -> bool:
            attempt = float(np.hypot(delta_xy[0], delta_xy[1]))
            shift_log["max_attempt_m"] = max(float(shift_log["max_attempt_m"]), attempt)
            shift_log["n_ok"] = int(shift_log["n_ok"]) + 1
            data.qpos[0] += float(delta_xy[0])
            data.qpos[1] += float(delta_xy[1])
            mj.mj_forward(plan, data)
            return True

        def _soles() -> dict[str, np.ndarray]:
            return {side: _sole_point(plan, data, side) for side in ("l", "r")}

        # A foot that was swinging keeps its old foothold until it lands.
        # The landing tick aligns the stance foot, then records the new
        # foothold once. Later double-support ticks do not move either anchor.
        prev_sup = held["support"]
        prev_st = held["stance"]
        if support == "ss" and stance in ("l", "r"):
            if stance not in anchor:
                anchor[stance] = sole[stance][:2].copy()
            if not _shift(anchor[stance] - sole[stance][:2]):
                anchor[stance] = sole[stance][:2].copy()
        else:
            landing = ""
            if prev_sup == "ss" and prev_st in ("l", "r"):
                landing = "r" if prev_st == "l" else "l"
            if "l" not in anchor and "r" not in anchor:
                anchor["l"] = sole["l"][:2].copy()
                anchor["r"] = sole["r"][:2].copy()
            elif landing and prev_st in anchor:
                if _shift(anchor[prev_st] - sole[prev_st][:2]):
                    sole = _soles()
                anchor[landing] = sole[landing][:2].copy()
            elif "l" in anchor and "r" not in anchor:
                if _shift(anchor["l"] - sole["l"][:2]):
                    sole = _soles()
                anchor["r"] = sole["r"][:2].copy()
            elif "r" in anchor and "l" not in anchor:
                if _shift(anchor["r"] - sole["r"][:2]):
                    sole = _soles()
                anchor["l"] = sole["l"][:2].copy()
            else:
                mid_now = 0.5 * (sole["l"][:2] + sole["r"][:2])
                mid_anc = 0.5 * (anchor["l"] + anchor["r"])
                if not _shift(mid_anc - mid_now):
                    sole = _soles()
                    anchor["l"] = sole["l"][:2].copy()
                    anchor["r"] = sole["r"][:2].copy()
        sole = _soles()
        held["support"] = support
        held["stance"] = stance if stance in ("l", "r") else ""
        poses.append(np.array(data.qpos, dtype=np.float64, copy=True))
        soles.append(sole)
        ankles.append({
            side: _body_point(plan, data, f"{side}_ank_roll_link")[1] for side in ("l", "r")
        })

    used: list[dict[str, object]] = []
    for sample in samples:
        q_map = sample["q"]
        if not isinstance(q_map, dict):
            continue
        _place(q_map, str(sample["support"]), str(sample["stance"]))
        used.append(sample)
    samples = used
    n = len(poses)
    if n < 3 or n != len(samples):
        return empty
    qvel = np.zeros((n, nv), dtype=np.float64)
    qacc = np.zeros((n, nv), dtype=np.float64)

    def _dt(k: int) -> float:
        span = float(samples[k]["t"]) - float(samples[k - 1]["t"])
        return span if span > 1e-9 else dt

    for k in range(1, n):
        step = _dt(k)
        delta = poses[k][0:3] - poses[k - 1][0:3]
        # A foothold relabel can move the root by a centimetre in one tick.
        # That is not a body acceleration. Keep the previous root velocity
        # across the jump. Joint differences stay the raw reference.
        jump = float(np.hypot(delta[0], delta[1])) > 0.005 or abs(float(delta[2])) > 0.002
        if jump and k > 1:
            qvel[k, 0:3] = qvel[k - 1, 0:3]
            shift_log["n_reject"] = int(shift_log["n_reject"]) + 1
        else:
            qvel[k, 0:3] = delta / step
        for name, adr in dofadr.items():
            qvel[k, adr] = (float(poses[k][qadr[name]]) - float(poses[k - 1][qadr[name]])) / step
    for k in range(2, n):
        qacc[k] = (qvel[k] - qvel[k - 1]) / _dt(k)
    # Worst |τ| per phase, per joint. All three splits are stored at that tick.
    worst: dict[str, dict[str, dict[str, float | str]]] = {name: {} for name in PLAN_PHASES}
    root_max = 0.0
    force_max = 0.0
    qacc_max = 0.0
    spike: dict[str, object] = {"qacc": 0.0}
    phase_peaks: dict[str, list[float]] = {name: [] for name in PLAN_PHASES}
    phase_qp: dict[str, list[float]] = {name: [] for name in PLAN_PHASES}
    dof_name = {int(adr): name for name, adr in dofadr.items()}
    candidate_best: dict[tuple[str, str, str], dict[str, object]] = {}
    candidate_n = 0
    arm = np.asarray(plan.dof_armature, dtype=np.float64)
    leg_order = [(name, int(adr)) for name, adr in dofadr.items()]
    qp_ds = 0
    qp_feasible = 0
    qp_infeasible = 0
    qp_unsolved = 0
    qp_wall_n = 0
    qp_worst: dict[str, object] | None = None
    cone_excess_max = 0.0
    cone_resolved_n = 0
    ss_n = 0
    ss_over_n = 0
    ss_worst: dict[str, object] | None = None
    gap_share: list[float] = []
    gap_force: list[float] = []
    gap_worst: dict[str, object] | None = None
    n_compared = 0
    cop_ticks: list[dict[str, object]] = []
    cop_margins: list[float] = []
    n_boundary_ticks = 0
    n_boundary_feet = 0
    n_cop_feet = 0
    cop_inset = float(step_bars.EDGE_DWELL_M)
    for k in range(2, n):
        sample = samples[k]
        data.qpos[:] = poses[k]
        data.qvel[:] = qvel[k]
        data.qacc[:] = qacc[k]
        mj.mj_inverse(plan, data)
        ident = np.array(data.qfrc_inverse, dtype=np.float64, copy=True)
        # The inverse already carries the reference's required root force.
        # Aim that force through the planned ZMP so gravity is not applied twice.
        force = np.array(ident[0:3], dtype=np.float64, copy=True)
        mj.mj_fwdPosition(plan, data)
        p_l = soles[k]["l"]
        p_r = soles[k]["r"]
        com = np.array(data.subtree_com[int(mj.mj_name2id(plan, mj.mjtObj.mjOBJ_BODY, "body_link"))], dtype=np.float64)
        zmp_y = float(sample["zmp_y"])
        p_zmp = np.array([float(com[0]), zmp_y, 0.0], dtype=np.float64)
        support = str(sample["support"])
        stance = str(sample["stance"])
        zero = np.zeros(3, dtype=np.float64)
        if support == "ss" and stance in ("l", "r"):
            # One foot. Both labels carry the same wrench, so the spread is 0.
            f_lin = {stance: force, "l" if stance == "r" else "r": zero}
            m_lin = {"l": zero, "r": zero}
            # Apply the stance force at the planned ZMP, on the stance foot body.
            f_min = f_lin
            m_min = m_lin
            point = {"l": p_l, "r": p_r}
            point[stance] = p_zmp
            s_right = 1.0 if stance == "r" else 0.0
        else:
            f_l, f_r, s_right = _linear_foot_split(p_l, p_r, p_zmp, force)
            f_lin = {"l": f_l, "r": f_r}
            m_lin = {"l": zero, "r": zero}
            a_l = ankles[k]["l"]
            a_r = ankles[k]["r"]
            mf_l, mm_l, mf_r, mm_r = _min_norm_ankle_split(p_l, p_r, a_l, a_r, p_zmp, force)
            f_min = {"l": mf_l, "r": mf_r}
            m_min = {"l": mm_l, "r": mm_r}
            point = {"l": p_l, "r": p_r}
        gen_lin = np.zeros(nv, dtype=np.float64)
        gen_min = np.zeros(nv, dtype=np.float64)
        for side in ("l", "r"):
            gen_lin += _jac_wrench(plan, data, point[side] if support == "ss" and side == stance else soles[k][side], foot_ids[side], f_lin[side], m_lin[side])
            if support == "ss":
                gen_min += _jac_wrench(
                    plan, data,
                    p_zmp if side == stance else soles[k][side],
                    foot_ids[side], f_min[side], m_min[side],
                )
            else:
                gen_min += _jac_wrench(plan, data, soles[k][side], foot_ids[side], f_min[side], m_min[side])
        tau_lin = ident - gen_lin
        if trace:
            boxes = {side: _foot_box(plan, data, side) for side in ("l", "r")}
            if support == "ss" and stance in ("l", "r"):
                corners = np.asarray(boxes[stance]["corners"], dtype=np.float64)
                stance_box_y = float(boxes[stance]["y_m"])
                stance_hip_y = float(data.xanchor[hip_jnt[stance], 1])
            else:
                corners = np.vstack([
                    np.asarray(boxes["l"]["corners"], dtype=np.float64),
                    np.asarray(boxes["r"]["corners"], dtype=np.float64),
                ])
                stance_box_y = None
                stance_hip_y = None
            com_m = _support_margin(com[:2], corners)
            zmp_m = _support_margin(p_zmp[:2], corners)
            hip_tau = {}
            for side in ("l", "r"):
                hadr = int(dofadr[f"{side}_hip_roll"])
                hip_tau[side] = float(tau_lin[hadr])
                hip_tau[side + "_bare"] = float(tau_lin[hadr] - float(arm[hadr]) * float(qacc[k][hadr]))
            preview_y = sample.get("preview_com_y")
            if preview_y is None:
                preview_y = sample.get("com_y")
            tick_row = {
                "k": k,
                "t_s": float(sample["t"]),
                "phase": str(sample["phase"]),
                "support": support,
                "stance": stance if stance in ("l", "r") else "",
                "qfrc_root_nm": [float(v) for v in ident[:ROOT_DOFS]],
                "resid_root_nm": [float(v) for v in tau_lin[:ROOT_DOFS]],
                "planned_com_y_m": float(com[1]),
                "preview_com_y_m": None if preview_y is None else float(preview_y),
                "planned_zmp_y_m": float(zmp_y),
                "planned_zmp_x_m": float(p_zmp[0]),
                "root_y_m": float(poses[k][1]),
                "box_l_y_m": float(boxes["l"]["y_m"]),
                "box_r_y_m": float(boxes["r"]["y_m"]),
                "hip_l_y_m": float(data.xanchor[hip_jnt["l"], 1]),
                "hip_r_y_m": float(data.xanchor[hip_jnt["r"], 1]),
                "stance_box_y_m": stance_box_y,
                "stance_hip_y_m": stance_hip_y,
                "live_com_y_m": sample.get("live_com_y"),
                "live_cop_y_m": sample.get("live_cop_y"),
                "preview_y_m": sample.get("preview_y"),
                "com_margin_m": com_m,
                "zmp_margin_m": zmp_m,
                "com_outside": com_m < -SUPPORT_OUT_M,
                "zmp_outside": zmp_m < -SUPPORT_OUT_M,
                "r_hip_roll_nm": hip_tau["r"],
                "r_hip_roll_bare_nm": hip_tau["r_bare"],
                "l_hip_roll_nm": hip_tau["l"],
                "l_hip_roll_bare_nm": hip_tau["l_bare"],
            }
            trace_rows.append(tick_row)
            trace_aux.append({
                "pose": np.array(poses[k], dtype=np.float64, copy=True),
                "qvel": np.array(qvel[k], dtype=np.float64, copy=True),
                "qacc": np.array(qacc[k], dtype=np.float64, copy=True),
                "force": np.array(force, dtype=np.float64, copy=True),
                "gen": np.array(gen_lin, dtype=np.float64, copy=True),
                "com": np.array(com, dtype=np.float64, copy=True),
                "box": boxes[stance] if stance in boxes else boxes["r"],
                "stance": stance if stance in ("l", "r") else "",
            })
        tau_min = ident - gen_min
        b_no_arm = ident - arm * qacc[k]
        qp_status = "single-support"
        tau_qp = tau_lin
        f_qp: dict[str, np.ndarray] | None = f_lin
        t_star = None
        if support == "ss" and stance in ("l", "r"):
            gen_qp = gen_lin
        else:
            qp_ds += 1
            solved = _qp_foot_split(plan, data, foot_ids, p_zmp, force, b_no_arm, leg_order)
            qp_status = str(solved["status"])
            if qp_status == "feasible" and "gen" in solved:
                qp_feasible += 1
                gen_qp = np.asarray(solved["gen"], dtype=np.float64)
                tau_qp = ident - gen_qp
                f_qp = solved["f"] if isinstance(solved["f"], dict) else None
                t_star = float(solved["t_star"])
                cone_excess_max = max(cone_excess_max, float(solved.get("cone_excess_n", 0.0)))
                if solved.get("cone_resolved"):
                    cone_resolved_n += 1
                cop = solved.get("cop")
                if isinstance(cop, dict):
                    margins = cop.get("margin_m")
                    xys = cop.get("xy_m")
                    ons = cop.get("on_boundary")
                    if isinstance(margins, dict) and isinstance(xys, dict) and isinstance(ons, dict):
                        flagged = False
                        tick_row: dict[str, object] = {
                            "t_s": float(sample["t"]),
                            "phase": str(sample["phase"]),
                        }
                        for side in ("l", "r"):
                            margin = margins.get(side)
                            tick_row[f"margin_{side}_m"] = (
                                None if margin is None else float(margin)
                            )
                            tick_row[f"cop_{side}_m"] = xys.get(side)
                            on = bool(ons.get(side))
                            tick_row[f"boundary_{side}"] = on
                            if margin is not None:
                                n_cop_feet += 1
                                cop_margins.append(float(margin))
                                if on:
                                    n_boundary_feet += 1
                                    flagged = True
                        if flagged:
                            n_boundary_ticks += 1
                        cop_ticks.append(tick_row)
            else:
                if qp_status == "infeasible":
                    qp_infeasible += 1
                else:
                    qp_unsolved += 1
                    qp_status = "unsolved"
                gen_qp = None
                tau_qp = None
                f_qp = None
        if gen_qp is not None:
            root_here = float(np.max(np.abs((ident - gen_qp)[:ROOT_DOFS])))
        else:
            root_here = 0.0
        force_max = max(force_max, float(np.max(np.abs(force))))
        peak_i = int(np.argmax(np.abs(qacc[k])))
        peak = abs(float(qacc[k][peak_i]))
        qacc_max = max(qacc_max, peak)
        if peak > float(spike["qacc"]):
            dq = {
                name: float(poses[k][qadr[name]] - poses[k - 1][qadr[name]])
                for name in dofadr
            }
            spike = {
                "qacc": peak,
                "dof": "root" if peak_i < ROOT_DOFS else dof_name.get(peak_i, str(peak_i)),
                "t_s": float(sample["t"]),
                "phase": str(sample["phase"]),
                "support": support,
                "root_dxy_m": [
                    float(poses[k][0] - poses[k - 1][0]),
                    float(poses[k][1] - poses[k - 1][1]),
                ],
                "root_dz_m": float(poses[k][2] - poses[k - 1][2]),
                "joint_dq_rad": dq,
                "force_n": [float(v) for v in force],
            }
        root_max = max(
            root_max,
            float(np.max(np.abs(tau_lin[:ROOT_DOFS]))),
            float(np.max(np.abs(tau_min[:ROOT_DOFS]))),
            root_here,
        )
        phase = str(sample["phase"])
        if phase in phase_peaks:
            phase_peaks[phase].append(max(
                max(abs(float(tau_lin[adr])), abs(float(tau_min[adr])))
                for adr in dofadr.values()
            ))
        no_qp_legs: dict[str, float] = {}
        with_qp_legs: dict[str, float] = {}
        if tau_qp is not None:
            for name, adr in leg_order:
                with_qp_legs[name] = float(tau_qp[adr])
                no_qp_legs[name] = float(tau_qp[adr]) - float(arm[adr]) * float(qacc[k][adr])
            qp_peak = max(abs(v) for v in no_qp_legs.values())
            qp_joint = max(no_qp_legs, key=lambda name: abs(no_qp_legs[name]))
            if phase in phase_qp:
                phase_qp[phase].append(qp_peak)
            sample_rec = {
                "joint": qp_joint,
                "t_s": float(sample["t"]),
                "phase": phase,
                "support": support,
                "tau_no_armature_nm": no_qp_legs[qp_joint],
                "tau_nm": with_qp_legs[qp_joint],
                "max_no_armature_nm": qp_peak,
                "t_star_nm": t_star if t_star is not None else qp_peak,
            }
            if support == "ss":
                ss_n += 1
                if qp_peak > ASK_BAR + 1e-9:
                    ss_over_n += 1
                if ss_worst is None or qp_peak > float(ss_worst["max_no_armature_nm"]):
                    ss_worst = sample_rec
            elif qp_status == "feasible":
                # Wall number is the pure minimax, before the tangential tie-break.
                wall_peak = float(t_star) if t_star is not None else qp_peak
                if wall_peak > ASK_BAR + 1e-9:
                    qp_wall_n += 1
                if qp_worst is None or wall_peak > float(qp_worst["max_no_armature_nm"]):
                    qp_worst = dict(sample_rec)
                    qp_worst["max_no_armature_nm"] = wall_peak
            if (
                support != "ss"
                and qp_status == "feasible"
                and f_qp is not None
                and "foot_f_l" in sample
                and "foot_f_r" in sample
            ):
                real_l = np.asarray(sample["foot_f_l"], dtype=np.float64)
                real_r = np.asarray(sample["foot_f_r"], dtype=np.float64)
                fql = np.asarray(f_qp["l"], dtype=np.float64)
                fqr = np.asarray(f_qp["r"], dtype=np.float64)
                den_q = float(fql[2] + fqr[2])
                if (
                    float(real_l[2]) > QP_DS_FZ_N
                    and float(real_r[2]) > QP_DS_FZ_N
                    and den_q > QP_DS_FZ_N
                ):
                    n_compared += 1
                    den_r = float(real_l[2] + real_r[2])
                    share = abs(float(real_r[2]) / den_r - float(fqr[2]) / den_q)
                    gap_share.append(share)
                    gap = float(np.linalg.norm(np.concatenate([real_l - fql, real_r - fqr])))
                    gap_force.append(gap)
                    if gap_worst is None or gap > float(gap_worst["force_gap_n"]):
                        gap_worst = {
                            "t_s": float(sample["t"]),
                            "phase": phase,
                            "force_gap_n": gap,
                            "share_abs": share,
                            "share_real": float(real_r[2]) / den_r,
                            "share_qp": float(fqr[2]) / den_q,
                            "f_l_real_n": [float(v) for v in real_l],
                            "f_r_real_n": [float(v) for v in real_r],
                            "f_l_qp_n": [float(v) for v in fql],
                            "f_r_qp_n": [float(v) for v in fqr],
                        }
        if phase not in worst:
            continue
        qdd = qacc[k]
        for name, adr in dofadr.items():
            with_lin = float(tau_lin[adr])
            with_min = float(tau_min[adr])
            strip = float(arm[adr]) * float(qdd[adr])
            no_lin = with_lin - strip
            no_min = with_min - strip
            spread = abs(with_lin - with_min)
            rank = max(abs(with_lin), abs(with_min))
            for label, with_nm, no_nm in (
                ("linear", with_lin, no_lin),
                ("min-norm", with_min, no_min),
            ):
                if abs(with_nm) > ASK_BAR + 1e-9 and abs(no_nm) <= ASK_BAR + 1e-9:
                    candidate_n += 1
                    key = (name, phase, label)
                    prev_c = candidate_best.get(key)
                    if prev_c is None or abs(with_nm) > abs(float(prev_c["tau_nm"])):
                        candidate_best[key] = {
                            "joint": name,
                            "t_s": float(sample["t"]),
                            "phase": phase,
                            "split": label,
                            "tau_nm": with_nm,
                            "tau_no_armature_nm": no_nm,
                            "class": "unsourced-armature candidate",
                        }
            prev = worst[phase].get(name)
            if prev is not None and float(prev["rank"]) >= rank:
                continue
            row = {
                "joint": name,
                "t_s": float(sample["t"]),
                "phase": phase,
                "support": support,
                "tau_linear_nm": with_lin,
                "tau_linear_no_armature_nm": no_lin,
                "tau_minnorm_nm": with_min,
                "tau_minnorm_no_armature_nm": no_min,
                "tau_qp_nm": with_qp_legs.get(name),
                "tau_qp_no_armature_nm": no_qp_legs.get(name),
                "qp_status": qp_status,
                "spread_nm": spread,
                "armature_qdd_nm": strip,
                "zmp_y_m": zmp_y,
                "right_share": float(s_right),
                "rank": rank,
            }
            worst[phase][name] = row
    for phase in PLAN_PHASES:
        for row in worst[phase].values():
            row.pop("rank", None)

    def _median(vals: list[float]) -> float | None:
        if not vals:
            return None
        return float(np.median(np.asarray(vals, dtype=np.float64)))

    y_trace: dict[str, object] | None = None
    if trace and trace_rows:
        resid = np.asarray([row["resid_root_nm"] for row in trace_rows], dtype=np.float64)
        root_abs = np.max(np.abs(resid), axis=1)
        root_max_here = float(np.max(root_abs)) if root_abs.size else 0.0
        period = float(trace_period_s) if trace_period_s is not None else 1.0
        cycle = _cycle_rows(trace_rows, period)
        right = [
            row for row in cycle
            if row["support"] == "ss" and row["stance"] == "r"
        ]
        mid_row = right[len(right) // 2] if right else None
        nearest = None
        nearest_gap = None
        for row in trace_rows:
            if row["phase"] != "walk" or row["support"] != "ss" or row["stance"] != "r":
                continue
            gap = abs(float(row["r_hip_roll_bare_nm"]) - 2.75)
            if nearest_gap is None or gap < nearest_gap:
                nearest_gap = gap
                nearest = row

        # index identity: trace_aux[i] belongs to trace_rows[i], and k = i + 2.
        for i, row in enumerate(trace_rows):
            if int(row["k"]) != i + 2:
                raise RuntimeError(f"plan-y k {row['k']} != {i + 2}")

        def _parts_at(row: dict[str, object] | None) -> dict[str, object] | None:
            if row is None or row.get("stance") not in ("l", "r"):
                return None
            aux = trace_aux[int(row["k"]) - 2]
            stance_side = str(row["stance"])
            pose = np.asarray(aux["pose"], dtype=np.float64)
            qvel_k = np.asarray(aux["qvel"], dtype=np.float64)
            qacc_k = np.asarray(aux["qacc"], dtype=np.float64)
            force_k = np.asarray(aux["force"], dtype=np.float64)
            data.qpos[:] = pose
            mj.mj_fwdPosition(plan, data)
            ident_k = _qfrc_at(plan, data, pose, qvel_k, qacc_k)
            box = aux["box"] if isinstance(aux["box"], dict) else {}
            best = _best_force_point(
                plan, data, foot_ids[stance_side], force_k, ident_k,
                np.asarray(box["centre"], dtype=np.float64),
            )
            parts = _hip_roll_parts(
                plan, pose, qvel_k, qacc_k, stance_side, force_k,
                np.asarray(aux["gen"], dtype=np.float64),
                np.asarray(best["gen"], dtype=np.float64),
                dofadr,
                np.asarray(aux["com"], dtype=np.float64),
                float(row["planned_zmp_y_m"]),
                box,
            )
            corners = np.asarray(box["corners"], dtype=np.float64)
            best_xy = np.asarray(best["point_m"], dtype=np.float64)[:2]
            return {
                "t_s": float(row["t_s"]),
                "phase": row["phase"],
                "support": row["support"],
                "stance": stance_side,
                "joint": parts["joint"],
                "tau_nm": parts["tau_nm"],
                "tau_bare_nm": parts["tau_bare_nm"],
                "parts": parts,
                "best_point_m": best["point_m"],
                "best_root_max_nm": best["root_max_nm"],
                "best_resid_nm": best["resid_nm"],
                "best_inside_box": _support_margin(best_xy, corners) >= -SUPPORT_OUT_M,
                "planned_resid_nm": row["resid_root_nm"],
                "planned_zmp_y_m": row["planned_zmp_y_m"],
                "planned_com_y_m": row["planned_com_y_m"],
                "preview_com_y_m": row["preview_com_y_m"],
                "live_com_y_m": row["live_com_y_m"],
                "live_cop_y_m": row["live_cop_y_m"],
            }

        stance_mid = _parts_at(mid_row)
        nearest_parts = None
        if nearest is not None and (mid_row is None or int(nearest["k"]) != int(mid_row["k"])):
            nearest_parts = _parts_at(nearest)
        support_bad = any(bool(row["com_outside"]) or bool(row["zmp_outside"]) for row in trace_rows)
        root_bad = root_max_here > ROOT_CONSIST_NM
        comp_max = [float(v) for v in np.max(np.abs(resid), axis=0)] if resid.size else []
        y_trace = {
            "period_s": period,
            "root_consist_nm": ROOT_CONSIST_NM,
            "support_out_m": SUPPORT_OUT_M,
            "summary": {
                "n_ticks": len(trace_rows),
                "root_resid_max_nm": root_max_here,
                "root_resid_component_max_nm": comp_max,
                "n_root_over_1e-3": int(np.sum(root_abs > 1e-3)),
                "n_root_over_1e-2": int(np.sum(root_abs > ROOT_CONSIST_NM)),
                "root_inconsistent": root_bad,
                "n_com_outside": int(sum(bool(row["com_outside"]) for row in trace_rows)),
                "n_zmp_outside": int(sum(bool(row["zmp_outside"]) for row in trace_rows)),
                "support_inconsistent": support_bad,
                "inconsistent": bool(root_bad or support_bad),
                "tau_is_not_a_discard": True,
                "cycle_t0_s": float(cycle[0]["t_s"]) if cycle else None,
                "cycle_t1_s": float(cycle[-1]["t_s"]) if cycle else None,
                "right_stance_n": len(right),
            },
            "ticks": [
                {key: row[key] for key in row if key != "k"}
                for row in trace_rows
            ],
            "cycle": [
                {key: row[key] for key in row if key != "k"}
                for row in cycle
            ],
            "stance_mid": stance_mid,
            "nearest_bare_2_75": nearest_parts,
        }
        _print_plan_y(y_trace)

    out = {
        "phases": worst,
        "candidates": list(candidate_best.values()),
        "n_candidate_samples": candidate_n,
        "root_residual_max_nm": root_max,
        "force_abs_max_n": force_max,
        "qacc_abs_max": qacc_max,
        "qacc_spike": spike,
        "phase_median_abs_nm": {name: _median(vals) for name, vals in phase_peaks.items()},
        "phase_median_qp_no_armature_nm": {name: _median(vals) for name, vals in phase_qp.items()},
        "qp_wall": {
            "n_ds": qp_ds,
            "n_feasible": qp_feasible,
            "n_infeasible": qp_infeasible,
            "n_unsolved": qp_unsolved,
            "n_wall": qp_wall_n,
            "cone_excess_max_n": cone_excess_max,
            "n_cone_resolved": cone_resolved_n,
            "bar_nm": ASK_BAR,
            "mu": QP_MU,
            "box_mm": [135.0, 76.0],
            "cop_inset_m": cop_inset,
            "cop_inset_name": "step_bars.EDGE_DWELL_M",
            "worst": qp_worst,
        },
        "qp_cop": {
            "inset_m": cop_inset,
            "inset_name": "step_bars.EDGE_DWELL_M",
            "bound_tol_m": QP_COP_BOUND_TOL_M,
            "n_ticks": len(cop_ticks),
            "n_feet": n_cop_feet,
            "n_boundary_ticks": n_boundary_ticks,
            "n_boundary_feet": n_boundary_feet,
            "margin_min_m": (min(cop_margins) if cop_margins else None),
            "margin_median_m": _median(cop_margins),
            "ticks": cop_ticks,
        },
        "ss_over_no_armature": {
            "n_ss": ss_n,
            "n_over": ss_over_n,
            "worst": ss_worst,
        },
        "realised_vs_qp": {
            "n_planned_ds": qp_ds,
            "n_qp_feasible": qp_feasible,
            "n_compared": n_compared,
            "fz_min_n": QP_DS_FZ_N,
            "share_abs_median": _median(gap_share),
            "share_abs_max": (max(gap_share) if gap_share else None),
            "force_gap_median_n": _median(gap_force),
            "force_gap_max_n": (max(gap_force) if gap_force else None),
            "worst": gap_worst,
        },
        "root_velocity_holds": int(shift_log["n_reject"]),
        "max_root_step_m": float(shift_log["max_attempt_m"]),
        "root_x_m": (
            [min(float(p[0]) for p in poses), max(float(p[0]) for p in poses)]
            if poses else [None, None]
        ),
        "n_ticks": n,
        "mass_kg": mass,
        "note": (
            "τ_req is mj_inverse of the walker's q_des, with qvel and qacc "
            "the 8 ms differences, contacts off, and the root force applied "
            "through the planned ZMP. Double support reports the foot-line "
            "split, the min-norm ankle split, and a linear program that "
            "minimises the largest |τ| over the leg joints with armature "
            "removed. That program keeps each foot's centre of pressure "
            "inside the 135×76 mm box shrunk on every side by "
            "step_bars.EDGE_DWELL_M and each corner force inside a friction "
            "cone of μ 1.2. The per-foot CoP margin is logged, and a tick "
            "whose reported CoP lies on that shrunk edge is flagged. A wall "
            "is only a double-support tick where this split still needs more "
            "than 2.33 Nm without armature. Linear and min-norm exceeding "
            "2.33 are not a wall. Single support above 2.33 without armature "
            "is required single-foot torque. The realised contact split is "
            "compared with this split and does not change the pass. The "
            "boundary flag does not change the pass. "
            "clears-only-without-armature is an unsourced-armature candidate, "
            "not a pass."
        ),
    }
    if y_trace is not None:
        out["y_trace"] = y_trace
    return out


def score_cell(
    period_s: float,
    vx: float,
    amp: float,
    perturb: sws.Perturb | None = None,
    leg_armature: float | None = None,
    plan_y: bool = False,
) -> dict[str, object]:
    global _CAPTURE_ON, _ID_COPY, _ID_MODEL, _FOOT_SINK, _FOOT_SPEC
    digest_before = _plant_md5()
    if digest_before != PLANT_MD5:
        raise SystemExit(f"plant md5 {digest_before} != {PLANT_MD5}")
    armature = PLANT_ARMATURE if leg_armature is None else float(leg_armature)
    override = abs(armature - PLANT_ARMATURE) > 1e-12
    stand_s = 0.25
    walk_s = 1.00 + 2.05 * float(period_s)
    stop_s = 2.40
    t_stop = stand_s + walk_s
    t_end = t_stop + stop_s
    cfg = _config(period_s, vx, amp)
    scene = None
    if perturb is not None and perturb.rug:
        scene = sws.steer_walk.ROOT / "mujoco" / "room_entrance.xml"
    if override and scene is not None:
        raise SystemExit("armature override does not load a scene include")
    compiled = compile_leg_armature(armature) if override else None
    session = _open_session(cfg, scene, compiled)
    if perturb is not None:
        sws._apply_perturb(session, perturb)
        if perturb.rug:
            sws._place_entrance_rug(session)
        if _plant_md5() != PLANT_MD5:
            raise SystemExit("plant md5 changed while applying a runtime perturb")
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
    integrator = _integrator_name(session.model)
    solver = _solver_report(session.model)
    physics_dt = float(session.model.opt.timestep)
    # Flags live on a binary copy taken after any runtime perturb. The file stays clear.
    _ID_MODEL = _isolated_inverse_model(session.model)
    _ID_COPY = mj.MjData(_ID_MODEL)
    if int(session.model.opt.enableflags) != 0:
        raise RuntimeError("live enableflags changed while building the inverse copy")
    inv_bit = _invdiscrete_bit()
    use_invdiscrete = inv_bit is not None
    if use_invdiscrete and int(_ID_MODEL.opt.enableflags) & inv_bit == 0:
        raise RuntimeError("inverse copy is missing mjENBL_INVDISCRETE")
    root_jid = int(mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_JOINT, "root"))
    if root_jid < 0 or int(session.model.jnt_type[root_jid]) != int(mj.mjtJoint.mjJNT_FREE):
        raise RuntimeError("root is not a free joint")
    root_adr = int(session.model.jnt_dofadr[root_jid])
    if root_adr != 0:
        raise RuntimeError(f"root dof address is {root_adr}, expected 0")
    writes: list[Write] = []
    asks: list[sws.AskSample] = []
    plant_asks: list[sws.AskSample] = []
    plant_overs: list[PlantOver] = []
    tick_box = [0]
    _install(walker, writes, asks, tick_box)
    legs = _leg_actuators(session.model, walker)
    for leg in legs:
        damp = float(session.model.dof_damping[int(leg["dof"])])
        if abs(damp - JOINT_DAMPING) > 1e-12:
            raise RuntimeError(f"{leg['joint']} damping {damp} is not {JOINT_DAMPING}")
        arm = float(session.model.dof_armature[int(leg["dof"])])
        if abs(arm - armature) > 1e-12:
            raise RuntimeError(f"{leg['joint']} armature {arm} is not {armature}")
    kv_of = {str(leg["joint"]): float(leg["kv"]) for leg in legs}
    ctrl_hist: dict[str, list[float]] = {name: [] for name in sws.LEG_JOINTS}
    raw_unavailable = False
    clip_ticks = 0
    clip_substeps = 0
    plant_substeps = 0
    max_step_abs = 0.0
    max_writer_abs = 0.0
    limiter_ticks = 0
    residual_fail_ticks = 0
    residual_fail_substeps = 0
    discrete_fail_ticks = 0
    discrete_fail_substeps = 0
    match_over_fail_ticks = 0
    match_over_fail_substeps = 0
    leg_over_fail_ticks = 0
    root_over_fail_ticks = 0
    match_over_bucket_ticks = 0
    match_over_bucket_substeps = 0
    match_residual_max = 0.0
    leg_match_max = 0.0
    root_match_max = 0.0
    plan_samples: list[dict[str, object]] = []
    zmp_calls: list[float] = []
    preview_step = zmp_preview.ZmpPreview.step

    def _preview_step(self: object, future: object) -> float:
        arr = np.asarray(future, dtype=np.float64).reshape(-1)
        if arr.size:
            zmp_calls.append(float(arr[0]))
        return preview_step(self, future)

    zmp_preview.ZmpPreview.step = _preview_step  # type: ignore[method-assign]
    band_max = 0.0
    band_cap_ticks = 0
    band_and_residual_ticks = 0
    knee_rail_count = 0
    root_resid_max = 0.0
    fwdinv_ticks: list[list[float]] = []
    root_id_ticks: list[float] = []
    root_id_max = 0.0
    knee_id_peak = 0.0
    knee_id_signed = 0.0
    knee_id_joint: str | None = None
    knee_id_t: float | None = None
    knee_id_pass_peak = 0.0
    knee_id_pass_signed = 0.0
    knee_id_pass_joint: str | None = None
    knee_id_pass_t: float | None = None
    max_id_minus_act = 0.0
    max_leg_resid = 0.0
    max_leg_resid_joint: str | None = None
    max_leg_resid_qvel = 0.0
    max_leg_resid_passive = 0.0
    slope_num = 0.0
    slope_den = 0.0
    raw_peak_abs = 0.0
    raw_peak_signed = 0.0
    raw_peak_offset = 0.0
    raw_peak_adjusted = 0.0
    raw_peak_band = 0.0
    raw_peak_joint: str | None = None
    raw_peak_t: float | None = None
    adj_max_abs = 0.0
    offset_max_abs = 0.0
    tick_residual_fail: list[bool] = []
    move_s = max(float(cfg.gm_move_s), float(session.ctrl_dt))
    slew_frac = min(1.0, float(session.ctrl_dt) / move_s)
    slew_rad = float(sws.lipm_gait.HX35_SLEW_RAD_S) * float(session.ctrl_dt)
    move_idx = {
        str(mj.mj_id2name(session.model, mj.mjtObj.mjOBJ_ACTUATOR, int(idx)) or "").removesuffix("_pos"): int(idx)
        for idx in session._move_ctrl_idx
    }
    scratch = mj.MjData(session.model)
    grav_data = mj.MjData(session.model)
    dof = {
        name: int(session.model.jnt_dofadr[mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_JOINT, name)])
        for name in sws.LEG_JOINTS
    }
    qadr = {
        name: int(session.model.jnt_qposadr[mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_JOINT, name)])
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
    last_q: dict[str, float] | None = None
    _FOOT_SPEC = {
        "l": int(walker.gid["L"]),
        "r": int(walker.gid["R"]),
        "grounds": {int(gid) for gid in sws._ground_gids(session, walker)},
    }
    _FOOT_SINK = None
    try:
        while float(session.data.time) < t_end - 1e-12:
            now = float(session.data.time)
            zmp_mark = len(zmp_calls)
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
            ctrl_before = np.array(session.data.ctrl, dtype=np.float64, copy=True)
            n_ask = len(asks)
            _CAPTURE.clear()
            _CAPTURE_ON = True
            foot_buf: list[dict[str, list[float]]] = []
            _FOOT_SINK = foot_buf
            try:
                session.step()
            finally:
                _CAPTURE_ON = False
                _FOOT_SINK = None
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
            tick_fail = False
            tick_discrete = False
            tick_over_fail = False
            tick_leg_over = False
            tick_root_over = False
            tick_over_bucket = False
            tick_band_max = 0.0
            fwd0 = 0.0
            fwd1 = 0.0
            root_tick = 0.0
            substep_fail: list[bool] = []
            substep_discrete: list[bool] = []
            substep_over_bucket: list[bool] = []
            for shot in shots:
                ident = shot["id"]
                actuator = shot["actuator"]
                passive = shot["passive"]
                qvel = shot["qvel"]
                qacc_fwd = shot["qacc_fwd"]
                fwd = shot["fwdinv"]
                fwd0 = max(fwd0, abs(float(fwd[0])))
                fwd1 = max(fwd1, abs(float(fwd[1])))
                root_slice = slice(root_adr, root_adr + ROOT_DOFS)
                root_raw = ident[root_slice] - (actuator[root_slice] + passive[root_slice])
                root_resid = float(np.max(np.abs(root_raw)))
                root_resid_max = max(root_resid_max, root_resid)
                root_abs = float(np.max(np.abs(ident[root_slice])))
                root_tick = max(root_tick, root_abs)
                root_id_max = max(root_id_max, root_abs)
                this_fail = False
                root_match = ident[root_slice] - actuator[root_slice]
                root_match_abs = float(np.max(np.abs(root_match)))
                match_residual_max = max(match_residual_max, root_match_abs)
                root_match_max = max(root_match_max, root_match_abs)
                this_over_fail = root_match_abs > DISCRETE_RESIDUAL_NM + 1e-12
                this_root_over = this_over_fail
                this_leg_over = False
                this_over_bucket = root_match_abs > BUCKET_RESIDUAL_NM + 1e-12
                this_discrete = root_resid > DISCRETE_RESIDUAL_NM + 1e-12
                for leg in legs:
                    adr = int(leg["dof"])
                    joint = str(leg["joint"])
                    raw = float(ident[adr] - (actuator[adr] + passive[adr]))
                    resid = abs(raw)
                    # Band stays on the forward qacc so the comparison column matches
                    # the previous score. The inverse itself used the discrete qacc.
                    offset, band = _implicit_terms(physics_dt, float(leg["kv"]), float(qacc_fwd[adr]))
                    adjusted = raw - offset
                    act_gap = abs(float(ident[adr] - actuator[adr]))
                    speed = abs(float(qvel[adr]))
                    max_id_minus_act = max(max_id_minus_act, act_gap)
                    match_residual_max = max(match_residual_max, act_gap)
                    leg_match_max = max(leg_match_max, act_gap)
                    if act_gap > DISCRETE_RESIDUAL_NM + 1e-12:
                        this_over_fail = True
                        this_leg_over = True
                    if act_gap > BUCKET_RESIDUAL_NM + 1e-12:
                        this_over_bucket = True
                    slope_num += resid * speed
                    slope_den += speed * speed
                    if resid > max_leg_resid:
                        max_leg_resid = resid
                        max_leg_resid_joint = joint
                        max_leg_resid_qvel = speed
                        max_leg_resid_passive = abs(float(passive[adr]))
                    if resid > raw_peak_abs:
                        raw_peak_abs = resid
                        raw_peak_signed = raw
                        raw_peak_offset = offset
                        raw_peak_adjusted = adjusted
                        raw_peak_band = band
                        raw_peak_joint = joint
                        raw_peak_t = float(shot["time"])
                    adj_max_abs = max(adj_max_abs, abs(adjusted))
                    offset_max_abs = max(offset_max_abs, abs(offset))
                    tick_band_max = max(tick_band_max, band)
                    band_max = max(band_max, band)
                    if resid > band + 1e-9:
                        this_fail = True
                        tick_fail = True
                    gate = resid if use_invdiscrete else abs(adjusted)
                    if gate > DISCRETE_RESIDUAL_NM + 1e-12:
                        this_discrete = True
                    signed_id = float(ident[adr])
                    if joint.endswith("knee"):
                        if _rail_exact(signed_id):
                            knee_rail_count += 1
                        if abs(signed_id) > knee_id_peak:
                            knee_id_peak = abs(signed_id)
                            knee_id_signed = signed_id
                            knee_id_joint = joint
                            knee_id_t = float(shot["time"])
                if this_fail:
                    residual_fail_substeps += 1
                if this_discrete:
                    discrete_fail_substeps += 1
                    tick_discrete = True
                if this_over_fail:
                    match_over_fail_substeps += 1
                    tick_over_fail = True
                if this_leg_over:
                    tick_leg_over = True
                if this_root_over:
                    tick_root_over = True
                if this_over_bucket:
                    match_over_bucket_substeps += 1
                    tick_over_bucket = True
                substep_fail.append(this_fail)
                substep_discrete.append(this_discrete)
                substep_over_bucket.append(this_over_bucket)
            # The 0.15 Nm cap is a comparison count. It does not unbucket the tick.
            tick_capped = tick_band_max > BAND_CAP_NM + 1e-9
            if tick_capped:
                band_cap_ticks += 1
            if tick_capped and tick_fail:
                band_and_residual_ticks += 1
            if not tick_discrete:
                for shot, failed in zip(shots, substep_discrete):
                    if failed:
                        continue
                    ident_pass = shot["id"]
                    for leg in legs:
                        joint = str(leg["joint"])
                        if not joint.endswith("knee"):
                            continue
                        signed_id = float(ident_pass[int(leg["dof"])])
                        if abs(signed_id) > knee_id_pass_peak:
                            knee_id_pass_peak = abs(signed_id)
                            knee_id_pass_signed = signed_id
                            knee_id_pass_joint = joint
                            knee_id_pass_t = float(shot["time"])
            for over in plant_overs[over_from:]:
                si = getattr(over, "_shot", None)
                setattr(over, "_sub_fail", True if not isinstance(si, int) else substep_fail[si])
                setattr(over, "_discrete_fail", tick_discrete)
                setattr(over, "_over_fail", tick_over_fail)
                setattr(over, "_over_bucket", tick_over_bucket)
                setattr(over, "_band_cap", tick_capped)
                setattr(over, "_tick_band_max", tick_band_max)
            if tick_fail:
                residual_fail_ticks += 1
            if tick_discrete:
                discrete_fail_ticks += 1
            if tick_over_fail:
                match_over_fail_ticks += 1
            if tick_leg_over:
                leg_over_fail_ticks += 1
            if tick_root_over:
                root_over_fail_ticks += 1
            if tick_over_bucket:
                match_over_bucket_ticks += 1
            tick_residual_fail.append(tick_fail)
            fwdinv_ticks.append([fwd0, fwd1])
            root_id_ticks.append(root_tick)
            limited = any(
                rec.limiter_bound and rec.joint in sws.LEG_JOINTS for rec in new_writes
            )
            if mode == "move":
                goal_ctrl = {rec.joint: float(rec.ctrl) for rec in new_writes if rec.joint in move_idx}
                for joint, idx in move_idx.items():
                    if joint not in goal_ctrl:
                        continue
                    if abs((goal_ctrl[joint] - float(ctrl_before[idx])) * slew_frac) > slew_rad + 1e-9:
                        limited = True
                        break
            if limited:
                limiter_ticks += 1
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
                        split_cache[si] = _inverse_split(session.model, scratch, grav_data, shots[si])
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
                        "armature·q̈": float(
                            split["arm_model" if override else "arm_001"][adr]
                        ),
                        "impact/contact": float(split["contact"][adr]),
                        "link inertia": float(split["link"][adr]),
                        "gravity": float(split["gravity"][adr]),
                    }
                    ident = float(split["ident"][adr])
                    id_nm = float(split["id"][adr])
                    actuator_nm = float(split["actuator"][adr])
                    qfrc_passive = float(split["passive_raw"][adr])
                    qacc_i = float(split["qacc"][adr])
                    raw_i = id_nm - (actuator_nm + qfrc_passive)
                    offset_i, band_i = _implicit_terms(physics_dt, kv_of[over.joint], qacc_i)
                    force_resid = abs(raw_i)
                    fwd = shots[si]["fwdinv"]
                    over.id_blob = {
                        "id_nm": id_nm,
                        "id_abs_nm": abs(id_nm),
                        "parts": parts,
                        "passive_nm": float(split["passive"][adr]),
                        "qfrc_passive_nm": qfrc_passive,
                        "ident_nm": ident,
                        "ident_residual_nm": id_nm - ident,
                        "force_residual_nm": force_resid,
                        "residual_raw_nm": raw_i,
                        "implicitfast_offset_nm": offset_i,
                        "residual_adjusted_nm": raw_i - offset_i,
                        "residual_band_nm": band_i,
                        "armature_model_qacc_nm": float(split["arm_model"][adr]),
                        "qacc_rad_s2": float(split["qacc"][adr]),
                        "actuator_nm": actuator_nm,
                        "solver_fwdinv": [float(fwd[0]), float(fwd[1])],
                        "substep": si,
                        "n_substeps": len(shots),
                        "cop_x_mm": cop_x,
                        "cop_y_mm": cop_y,
                    }
            q_map = {
                rec.joint: float(rec.q_des)
                for rec in new_writes
                if rec.joint in sws.LEG_JOINTS
            }
            swing = ""
            stance = ""
            if new_writes:
                swing = new_writes[-1].swing_side
                stance = new_writes[-1].stance
            if len(q_map) == len(sws.LEG_JOINTS):
                last_q = q_map
            elif last_q is not None:
                filled = dict(last_q)
                filled.update(q_map)
                q_map = filled
            if len(q_map) == len(sws.LEG_JOINTS):
                support, stance_side = _plan_support(swing, stance)
                zmp_y = float(zmp_calls[-1]) if len(zmp_calls) > zmp_mark else 0.0
                if foot_buf:
                    mean_l = np.mean(
                        np.asarray([item["l"] for item in foot_buf], dtype=np.float64), axis=0,
                    )
                    mean_r = np.mean(
                        np.asarray([item["r"] for item in foot_buf], dtype=np.float64), axis=0,
                    )
                else:
                    mean_l = np.zeros(3, dtype=np.float64)
                    mean_r = np.zeros(3, dtype=np.float64)
                sample_row = {
                    "t": now,
                    "phase": _plan_phase(stage_name),
                    "support": support,
                    "stance": stance_side,
                    "q": q_map,
                    "zmp_y": zmp_y,
                    "com_y": float(getattr(walker, "preview_com_y", 0.0)),
                    "foot_f_l": [float(v) for v in mean_l],
                    "foot_f_r": [float(v) for v in mean_r],
                }
                if plan_y:
                    body_id = int(mj.mj_name2id(session.model, mj.mjtObj.mjOBJ_BODY, "body_link"))
                    sample_row["live_com_y"] = float(session.data.subtree_com[body_id, 1])
                    sample_row["preview_y"] = float(walker.op3.preview_y)
                    num = np.zeros(3, dtype=np.float64)
                    den = 0.0
                    grounds = tuple(int(gid) for gid in _FOOT_SPEC["grounds"])
                    for side_key in ("L", "R"):
                        got = sws.foot_floor_cop(
                            session.model, session.data,
                            int(walker.gid[side_key]), int(walker.gid_floor), grounds,
                        )
                        if got is None:
                            continue
                        world, fn = got
                        num += float(fn) * np.asarray(world, dtype=np.float64)
                        den += float(fn)
                    sample_row["live_cop_y"] = float(num[1] / den) if den > 1e-6 else None
                plan_samples.append(sample_row)
            tick_box[0] += 1
            if session.bus.fault:
                break
    finally:
        zmp_preview.ZmpPreview.step = preview_step  # type: ignore[method-assign]
        _FOOT_SINK = None
        _FOOT_SPEC = None
    session.assert_plant_unchanged()
    if int(session.model.opt.enableflags) != 0:
        raise RuntimeError("live enableflags were set during the bout")
    planned = _planned_tau(
        session.model, plan_samples, qadr, dof,
        trace=plan_y, trace_period_s=float(period_s),
    )
    if int(session.model.opt.enableflags) != 0:
        raise RuntimeError("planned-motion copy changed the live enableflags")
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
    # The residual band never drops a tick from this bar.
    if pair_ask_m.shape[0] != n:
        raise RuntimeError(
            f"signed bar saw {pair_ask_m.shape[0]} ticks, bout has {n}"
        )
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
    if match_over_fail_ticks:
        fail_reasons.append(
            f"match residual |qfrc_inverse - qfrc_actuator| over 1e-3 Nm "
            f"on {match_over_fail_ticks}/{n} control ticks"
        )
    if limiter_ticks:
        fail_reasons.append(
            f"limiter active on {limiter_ticks}/{n} control ticks"
        )
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
        gated = bool(getattr(rec, "_sub_fail", True))
        capped = bool(getattr(rec, "_band_cap", False))
        discrete_gated = bool(getattr(rec, "_discrete_fail", True))
        tick_over_bucket = bool(getattr(rec, "_over_bucket", False))
        tick_over_fail = bool(getattr(rec, "_over_fail", False))
        stripped: float | None = None
        joint_match: float | None = None
        clamped = False
        if not isinstance(blob, dict):
            kind, term = ("unmeasured", "")
            parts = {name: float("nan") for name in TERMS}
        else:
            parts = {name: float(blob["parts"][name]) for name in TERMS}
            id_nm = float(blob["id_nm"])
            actuator_nm = float(blob["actuator_nm"])
            stripped = id_nm - parts["armature·q̈"]
            joint_match = abs(id_nm - actuator_nm)
            # The realised inverse returns the actuator rail, so a clamped
            # joint cannot be sorted into the three buckets.
            clamped = (
                abs(rec.ask_nm) >= RAIL_NM - 1e-9
                or abs(actuator_nm) >= RAIL_NM - 1e-6
            )
            resid = abs(float(blob["ident_residual_nm"]))
            max_resid = max(max_resid, resid)
            if clamped:
                kind, term = CLAMPED, ""
            elif tick_over_bucket:
                kind, term = OVER_BUCKET, ""
            else:
                kind, term = _classify(
                    abs(rec.ask_nm), id_nm, parts["armature·q̈"], parts,
                )
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
            "qfrc_passive_nm": None if not isinstance(blob, dict) else blob["qfrc_passive_nm"],
            "force_residual_nm": None if not isinstance(blob, dict) else blob["force_residual_nm"],
            "residual_raw_nm": None if not isinstance(blob, dict) else blob["residual_raw_nm"],
            "implicitfast_offset_nm": None if not isinstance(blob, dict) else blob["implicitfast_offset_nm"],
            "residual_adjusted_nm": None if not isinstance(blob, dict) else blob["residual_adjusted_nm"],
            "residual_band_nm": None if not isinstance(blob, dict) else blob["residual_band_nm"],
            "ident_residual_nm": None if not isinstance(blob, dict) else blob["ident_residual_nm"],
            "solver_fwdinv": None if not isinstance(blob, dict) else blob["solver_fwdinv"],
            "residual_gated": gated,
            "discrete_gated": discrete_gated,
            "match_over_1e-3": tick_over_fail,
            "match_over_1e-2": tick_over_bucket,
            "joint_match_residual_nm": joint_match,
            "id_minus_armature_nm": stripped,
            "clamped": clamped,
            "band_capped": capped,
            "tick_band_max_nm": float(getattr(rec, "_tick_band_max", float("nan"))),
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
    n_residual_over = 0
    n_unbucketed = 0
    n_discrete_over = 0
    n_clamped = 0
    n_over_bucket = 0
    for row in ask_events:
        if row["class"] == UNBUCKETED:
            n_unbucketed += 1
            continue
        if row["class"] == DISCRETE_FAIL:
            n_discrete_over += 1
            continue
        if row["class"] == CLAMPED:
            n_clamped += 1
            continue
        if row["class"] == OVER_BUCKET:
            n_over_bucket += 1
            continue
        if row["class"] == RESIDUAL_FAIL:
            n_residual_over += 1
            continue
        if row["class"] in counts:
            counts[str(row["class"])] += 1
        if row["class"] == "physics candidate" and row["dominant_term"] in term_counts:
            term_counts[str(row["dominant_term"])] += 1
    n_bucketed = sum(counts.values())
    bucket_worst = {name: _worst_tick(ask_events, name) for name in CLASS_NAMES}
    worst = bucket_worst["unsourced-armature candidate"] or bucket_worst["physics candidate"] or bucket_worst["controller fail"]
    def _frac(count: int) -> float | None:
        if n_bucketed <= 0:
            return None
        return count / n_bucketed

    qdes_peak = max(writes, key=lambda rec: abs(rec.signed_nm)) if writes else None
    goal_peak = max(writes, key=lambda rec: abs(rec.ask_nm)) if writes else None
    ask_peak = max(plant_asks, key=lambda rec: abs(rec.ask_nm)) if plant_asks else None
    sum_peak = max(writes, key=lambda rec: rec.sum_nm) if writes else None
    signed_peak, sum_peak = step_bars.torque_column_peaks(torque_bar)
    gate = step_bars.clear_gate(
        full_bars_pass=not fail_reasons,
        step_pass=bool(stepping.get("passes")),
        signed_peak_nm=signed_peak,
        sum_peak_nm=sum_peak,
    )
    for reason in gate["block_reasons"]:
        text = str(reason)
        if text not in fail_reasons:
            fail_reasons.append(text)
    verdict = "CLEAR" if gate["may_clear"] else "Prefer FAIL"
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
        "leg_armature_kgm2": armature,
        "armature_label": SENSITIVITY_LABEL if override else "plant",
        "leg_kv": kv_of,
        "verdict": verdict,
        "gait": gait,
        "signed_peak_nm": gate["signed_peak_nm"],
        "sum_peak_nm": gate["sum_peak_nm"],
        "sum_signed_ratio": gate["sum_signed_ratio"],
        "flag": gate["flag"],
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
        "integrator": integrator,
        "physics_dt_s": physics_dt,
        "solver": solver,
        "joint_damping": JOINT_DAMPING,
        "limiter_active_fraction": (limiter_ticks / n) if n else 0.0,
        "limiter_active_ticks": limiter_ticks,
        "limiter": (
            "controller-side clip: write_clipped band on non-sagittal joints, "
            "write_force_limited predicted-force clip, and the hip/knee/ankle "
            "pitch slew in _lipm_substep"
        ),
        "residual_fail_label": RESIDUAL_FAIL,
        "residual_fail_ticks": residual_fail_ticks,
        "residual_fail_substeps": residual_fail_substeps,
        "residual_fail_over_bar": n_residual_over,
        "perturb": None if perturb is None else {
            "label": perturb.label,
            "seed": perturb.seed,
            "mass_scale": perturb.mass_scale,
            "rug": perturb.rug,
        },
        "mujoco_version": mj.__version__,
        "invdiscrete": use_invdiscrete,
        "discrete_residual_nm": DISCRETE_RESIDUAL_NM,
        "discrete_fail_label": DISCRETE_FAIL,
        "discrete_fail_ticks": discrete_fail_ticks,
        "discrete_fail_substeps": discrete_fail_substeps,
        "discrete_fail_over_bar": n_discrete_over,
        "match_residual_nm": DISCRETE_RESIDUAL_NM,
        "bucket_residual_nm": BUCKET_RESIDUAL_NM,
        "match_over_1e-3_ticks": match_over_fail_ticks,
        "match_over_1e-3_substeps": match_over_fail_substeps,
        "leg_over_1e-3_ticks": leg_over_fail_ticks,
        "root_over_1e-3_ticks": root_over_fail_ticks,
        "match_over_1e-2_ticks": match_over_bucket_ticks,
        "match_over_1e-2_substeps": match_over_bucket_substeps,
        "match_residual_max_nm": match_residual_max,
        "leg_match_residual_max_nm": leg_match_max,
        "root_match_residual_max_nm": root_match_max,
        "clamped_label": CLAMPED,
        "clamped_over_bar": n_clamped,
        "over_bucket_label": OVER_BUCKET,
        "over_bucket_over_bar": n_over_bucket,
        "planned": planned,
        "root_residual_max_nm": root_resid_max,
        "knee_id_rail_nm": RAIL_NM,
        "knee_id_rail_bug": knee_rail_count > 0 or _rail_exact(knee_id_peak),
        "knee_id_rail_count": knee_rail_count,
        "unbucketed_label": UNBUCKETED,
        "unbucketed_band_ticks": band_cap_ticks,
        "unbucketed_residual_ticks": residual_fail_ticks,
        "unbucketed_both_ticks": band_and_residual_ticks,
        "unbucketed_over_bar": n_unbucketed,
        "residual_band_max_nm": band_max,
        "band_cap_nm": BAND_CAP_NM,
        "residual_floor_nm": RESIDUAL_FLOOR,
        "residual_raw_peak_nm": raw_peak_signed,
        "residual_raw_peak_abs_nm": raw_peak_abs,
        "residual_raw_peak_joint": raw_peak_joint,
        "residual_raw_peak_t_s": raw_peak_t,
        "implicitfast_offset_nm": raw_peak_offset,
        "residual_adjusted_nm": raw_peak_adjusted,
        "residual_band_at_raw_peak_nm": raw_peak_band,
        "residual_adjusted_max_abs_nm": adj_max_abs,
        "implicitfast_offset_max_abs_nm": offset_max_abs,
        "n_bucketed": n_bucketed,
        "root_id_max_nm": root_id_max,
        "root_id_per_tick_nm": root_id_ticks,
        "solver_fwdinv_per_tick": fwdinv_ticks,
        "solver_fwdinv_max": [
            max((row[0] for row in fwdinv_ticks), default=0.0),
            max((row[1] for row in fwdinv_ticks), default=0.0),
        ],
        "knee_id_peak_nm": knee_id_peak,
        "knee_id_peak_signed_nm": knee_id_signed,
        "knee_id_peak_joint": knee_id_joint,
        "knee_id_peak_t_s": knee_id_t,
        "knee_id_controls_nm": KNEE_ID_CONTROLS_NM,
        "knee_id_matches_controls": abs(knee_id_peak - KNEE_ID_CONTROLS_NM) <= 5e-4,
        "knee_id_pass_peak_nm": knee_id_pass_peak,
        "knee_id_pass_signed_nm": knee_id_pass_signed,
        "knee_id_pass_joint": knee_id_pass_joint,
        "knee_id_pass_t_s": knee_id_pass_t,
        "knee_id_pass_matches_controls": abs(knee_id_pass_peak - KNEE_ID_CONTROLS_NM) <= 5e-4,
        "implicit_offset": {
            "integrator": integrator,
            "damping_implicit": integrator in ("implicit", "implicitfast"),
            "formula_nm": "dt*(0.08 + kv)*qacc",
            "joint_damping": JOINT_DAMPING,
            "timestep_s": physics_dt,
            "max_abs_id_minus_actuator_nm": max_id_minus_act,
            "max_leg_residual_nm": max_leg_resid,
            "max_leg_residual_joint": max_leg_resid_joint,
            "max_leg_residual_qvel_rad_s": max_leg_resid_qvel,
            "max_leg_residual_passive_nm": max_leg_resid_passive,
            "residual_per_speed_nm_per_rad_s": (slope_num / slope_den) if slope_den > 0.0 else None,
            "offset_at_raw_peak_nm": raw_peak_offset,
            "adjusted_at_raw_peak_nm": raw_peak_adjusted,
            "adjusted_max_abs_nm": adj_max_abs,
            "offset_max_abs_nm": offset_max_abs,
        },
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
            name: (term_counts[name] / n_bucketed) if n_bucketed else None for name in REMAINING
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
    plan_abs = 0.0
    plan_name = "-"
    plan_root = float(planned.get("root_residual_max_nm") or 0.0) if isinstance(planned, dict) else 0.0
    phases_plan = planned.get("phases") if isinstance(planned, dict) else None
    if isinstance(phases_plan, dict):
        for block in phases_plan.values():
            if not isinstance(block, dict):
                continue
            for item in block.values():
                if not isinstance(item, dict):
                    continue
                for key in ("tau_linear_nm", "tau_minnorm_nm"):
                    val = abs(float(item[key]))
                    if val > plan_abs:
                        plan_abs = val
                        plan_name = f"{item['phase']}:{item['joint']}"
    clamp_pick: dict[str, object] | None = None
    for ev in events:
        if ev.get("class") != CLAMPED or not str(ev.get("joint", "")).endswith("knee"):
            continue
        if clamp_pick is None or abs(float(ev["t_s"]) - 2.69) < abs(float(clamp_pick["t_s"]) - 2.69):
            clamp_pick = ev
    if clamp_pick is None:
        clamp_note = "-"
    else:
        clamp_note = (
            f"{clamp_pick['joint']} {float(clamp_pick['id_nm']):+.3f} "
            f"at {float(clamp_pick['t_s']):.3f}s "
            f"strip {float(clamp_pick['id_minus_armature_nm']):+.3f}"
        )
    qp_block = planned.get("qp_wall") if isinstance(planned, dict) else None
    real_block = planned.get("realised_vs_qp") if isinstance(planned, dict) else None
    if not isinstance(qp_block, dict):
        qp_block = {}
    if not isinstance(real_block, dict):
        real_block = {}
    ss_block = planned.get("ss_over_no_armature") if isinstance(planned, dict) else None
    if not isinstance(ss_block, dict):
        ss_block = {}
    cop_block = planned.get("qp_cop") if isinstance(planned, dict) else None
    if not isinstance(cop_block, dict):
        cop_block = {}

    def _opt(block: dict[str, object], key: str, spec: str) -> str:
        val = block.get(key)
        if not isinstance(val, (int, float)):
            return "-"
        return format(float(val), spec)

    print(
        f"T {period_s:.2f} vx {vx:.3f} arm {armature:.3f} {verdict} {gait} "
        f"ask {row['ask_peak_nm']:.3f} {row['ask_peak_joint']} "
        f"goal {row['goal_ask_peak_nm']:.3f} "
        f"clip {clip_fraction} lim {row['limiter_active_fraction']:.3f} "
        f"kneeID {knee_id_peak:.3f} {knee_id_joint} "
        f"raw {raw_peak_signed:+.4f} off {raw_peak_offset:+.4f} adj {raw_peak_adjusted:+.4f} "
        f"adjmax {adj_max_abs:.4f} "
        f"root {root_id_max:.3e} rootR {root_resid_max:.3e} "
        f"dresid {max_leg_resid:.4f} dfail {discrete_fail_ticks}/{n} "
        f"match {match_residual_max:.3e} leg {leg_match_max:.3e} rootM {root_match_max:.3e} "
        f"over1e-3 {match_over_fail_ticks}/{n} over1e-2 {match_over_bucket_ticks}/{n} "
        f"rail {knee_rail_count} "
        f"bandmax {band_max:.4f} "
        f"cap {band_cap_ticks}/{n} resfail {residual_fail_ticks}/{n} "
        f"qdes {row['qdes_peak_nm']:.3f} {row['qdes_peak_joint']} "
        f"over ask/qdes {n_ask_over}/{n_qdes_over} "
        f"clamped {n_clamped} unbucketed {n_over_bucket} bucketed {n_bucketed} "
        f"plan {plan_abs:.3f} {plan_name} rootP {plan_root:.3e} "
        f"qpW {_opt(qp_block, 'n_wall', '.0f')}/{_opt(qp_block, 'n_feasible', '.0f')} "
        f"qpInf {_opt(qp_block, 'n_infeasible', '.0f')} "
        f"ssOver {_opt(ss_block, 'n_over', '.0f')} "
        f"copInset {_opt(cop_block, 'inset_m', '.6f')} "
        f"copB {_opt(cop_block, 'n_boundary_ticks', '.0f')}/{_opt(cop_block, 'n_ticks', '.0f')} "
        f"copM {_opt(cop_block, 'margin_min_m', '.4f')}/{_opt(cop_block, 'margin_median_m', '.4f')} "
        f"share {_opt(real_block, 'share_abs_median', '.3f')}/{_opt(real_block, 'share_abs_max', '.3f')} "
        f"fgap {_opt(real_block, 'force_gap_median_n', '.2f')}/{_opt(real_block, 'force_gap_max_n', '.2f')} "
        f"note {torque_bar['note'] or '-'} resid {max_resid:.3e} "
        f"clampEx {clamp_note} "
        f"class {counts}",
        flush=True,
    )
    return row


def _slim_row(row: dict[str, object]) -> dict[str, object]:
    """Drop per-tick traces. The bar results and the planned τ table stay."""
    out = dict(row)
    for key in ("over_bar", "root_id_per_tick_nm", "solver_fwdinv_per_tick"):
        out.pop(key, None)
    planned = out.get("planned")
    if isinstance(planned, dict):
        planned = dict(planned)
        cop = planned.get("qp_cop")
        if isinstance(cop, dict):
            cop = dict(cop)
            cop.pop("ticks", None)
            planned["qp_cop"] = cop
        out["planned"] = planned
    return out


def _passes(blob: object) -> bool:
    return isinstance(blob, dict) and bool(blob.get("passes"))


def _bar_map(row: dict[str, object]) -> dict[str, bool]:
    reasons = [str(item) for item in row.get("fail_reasons", [])]
    return {
        "verdict_clear": row.get("verdict") == "CLEAR",
        "signed": bool(row.get("signed_pass")),
        "sum": bool(row.get("sum_pass")),
        "stop": not any(item.startswith("stop does not") for item in reasons),
        "stepping": bool(row.get("stepping_passes")),
        "limiter": float(row.get("limiter_active_fraction") or 0.0) == 0.0,
        "match": int(row.get("match_over_1e-3_ticks") or 0) == 0,
        "clip": row.get("ctrl_clip_fraction") == 0,
        "qvel": _passes(row.get("qvel")),
        "clamp": _passes(row.get("clamp")),
        "dc": _passes(row.get("dc")),
        "zmp": not any(item.startswith("ZMP") for item in reasons),
        "com": not any(item.startswith("CoM") for item in reasons),
        "jerk": not any("jerk" in item for item in reasons),
    }


def _sensitivity_comparison(plant: dict[str, object], sens: dict[str, object]) -> dict[str, object]:
    """0.025 is informational on an intermediate CLEAR and a hard gate before locked."""
    clears_plant = plant.get("verdict") == "CLEAR"
    clears_sens = sens.get("verdict") == "CLEAR"
    transfer = bool(clears_plant and not clears_sens)
    return {
        "label": "transfer risk" if transfer else None,
        "transfer_risk": transfer,
        "sensitivity_role": (
            "informational for an intermediate CLEAR"
            if clears_plant
            else "informational"
        ),
        "lock_gate": "0.025 is a hard gate before locked",
        "locked": bool(clears_plant and clears_sens),
        "bars_0_01": _bar_map(plant),
        "bars_0_025": _bar_map(sens),
    }


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
    parser.add_argument("--mass-scale", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=-1)
    parser.add_argument("--rug", action="store_true")
    parser.add_argument("--compact", action="store_true", help="Drop per-tick traces from the JSON.")
    parser.add_argument(
        "--armature", type=float, default=PLANT_ARMATURE,
        help="Leg armature. 0.01 loads the plant file. Any other value compiles an MjSpec.",
    )
    parser.add_argument(
        "--sensitivity", action="store_true",
        help="Score the best row and its stop at armature 0.01 and 0.025.",
    )
    parser.add_argument(
        "--plan-y", action="store_true",
        help="voice056 T 1.0 s vx 0.016: root residual and the stance hip-roll split.",
    )
    args = parser.parse_args()
    if sws.SOFT_PASS:
        raise SystemExit("soft-pass is on")
    if float(step_bars.EDGE_DWELL_M) != QP_COP_INSET_M:
        raise SystemExit(
            f"QP CoP inset {QP_COP_INSET_M} != step_bars.EDGE_DWELL_M {step_bars.EDGE_DWELL_M}"
        )
    print(
        f"QP CoP inset {step_bars.EDGE_DWELL_M!r} m from step_bars.EDGE_DWELL_M",
        flush=True,
    )
    if _plant_md5() != PLANT_MD5:
        raise SystemExit(f"plant md5 {_plant_md5()} != {PLANT_MD5}")
    if args.sensitivity or abs(float(args.armature) - PLANT_ARMATURE) > 1e-12:
        kv_rows = print_kv_side_by_side()
    else:
        kv_rows = None
    if args.sensitivity:
        if args.rug or args.seed >= 0 or abs(args.mass_scale - 1.0) > 1e-12:
            raise SystemExit("the armature sensitivity is the nominal plant")
        cache: dict[float, float] = {}
        period_s, vx = BEST_ROW
        amp = _amp(period_s, cache)
        scored = [
            score_cell(period_s, vx, amp, None, leg_armature=arm)
            for arm in (PLANT_ARMATURE, SENSITIVITY_ARMATURE)
        ]
        if _plant_md5() != PLANT_MD5:
            raise SystemExit("plant md5 changed")
        plant_row, sens_row = scored
        comparison = _sensitivity_comparison(plant_row, sens_row)
        label = comparison["label"] or "-"
        print(
            f"sensitivity {SENSITIVITY_LABEL} "
            f"best T {period_s:.2f} vx {vx:.3f} "
            f"verdict {plant_row['verdict']} / {sens_row['verdict']} "
            f"label {label} locked {comparison['locked']}",
            flush=True,
        )
        payload = {
            "tip": TIP_SHA,
            "soft_pass": False,
            "plant_md5": PLANT_MD5,
            "plant_armature_kgm2": PLANT_ARMATURE,
            "sensitivity_armature_kgm2": SENSITIVITY_ARMATURE,
            "sensitivity_label": SENSITIVITY_LABEL,
            "best_row": {
                "T_s": period_s,
                "vx_m_s": vx,
                "why": (
                    "Fewest fail reasons on the committed 54-cell grid, "
                    "signed ask passes, match residual under 1e-3. "
                    "The stop is this bout's final 1 s."
                ),
            },
            "kv": kv_rows,
            "lock_gate": comparison["lock_gate"],
            "comparison": comparison,
            "rows": [_slim_row(plant_row), _slim_row(sens_row)],
        }
        if args.out:
            path = Path(args.out)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(_jsonable(payload), indent=2) + "\n", encoding="utf-8")
            print(f"wrote {path}", flush=True)
        return
    if args.plan_y:
        if args.sensitivity or args.rug or args.seed >= 0 or abs(args.mass_scale - 1.0) > 1e-12:
            raise SystemExit("plan-y is the nominal voice056 cell")
        if abs(float(args.armature) - PLANT_ARMATURE) > 1e-12:
            raise SystemExit("plan-y stays on the plant armature 0.01")
        cache = {}
        period_s, vx = 1.0, 0.016
        row = score_cell(period_s, vx, _amp(period_s, cache), None, plan_y=True)
        if _plant_md5() != PLANT_MD5:
            raise SystemExit("plant md5 changed")
        planned = row.get("planned") if isinstance(row.get("planned"), dict) else {}
        trace = planned.get("y_trace") if isinstance(planned, dict) else None
        joints = row.get("torque_bar")
        hip = None
        if isinstance(joints, dict):
            for item in joints.get("joints", []):
                if isinstance(item, dict) and item.get("joint") == "r_hip_roll":
                    hip = item
        dc = row.get("dc")
        print(
            f"T {period_s:.2f} vx {vx:.3f} {row.get('verdict')} {row.get('gait')} "
            f"r_hip_roll signed {None if hip is None else hip.get('peak_ask_nm')} "
            f"dc {dc.get('passes') if isinstance(dc, dict) else None}",
            flush=True,
        )
        payload = {
            "tip": TIP_SHA,
            "soft_pass": False,
            "plant_md5": _plant_md5(),
            "plant_armature_kgm2": PLANT_ARMATURE,
            "T_s": period_s,
            "vx_m_s": vx,
            "verdict": row.get("verdict"),
            "gait": row.get("gait"),
            "fail_reasons": row.get("fail_reasons"),
            "r_hip_roll_signed": hip,
            "dc": row.get("dc"),
            "stop": row.get("stop"),
            "y_trace": trace,
            "note": (
                "Root residual is qfrc_inverse minus the planned ZMP wrench. "
                "A hip-roll torque is not a discard. Inconsistency is the root "
                "residual or CoM/ZMP outside the support polygon."
            ),
        }
        if args.out:
            path = Path(args.out)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(_jsonable(payload), indent=2) + "\n", encoding="utf-8")
            print(f"wrote {path}", flush=True)
        return
    cells = _parse_cells(args.cells) if args.cells else [
        (period, vx) for period in PERIODS for vx in SPEEDS
    ]
    perturb = None
    if args.rug or args.seed >= 0 or abs(args.mass_scale - 1.0) > 1e-12:
        label = "nominal"
        if args.rug:
            label = "rug"
        elif args.seed >= 0:
            label = f"seed-{args.seed}"
        elif abs(args.mass_scale - 1.0) > 1e-12:
            label = f"mass-{args.mass_scale:.2f}"
        perturb = sws.Perturb(
            label=label,
            seed=None if args.seed < 0 else int(args.seed),
            mass_scale=float(args.mass_scale),
            rug=bool(args.rug),
        )
    cache: dict[float, float] = {}
    rows = []
    for period_s, vx in cells:
        row = score_cell(
            period_s, vx, _amp(period_s, cache), perturb,
            leg_armature=float(args.armature),
        )
        if args.compact:
            for key in (
                "over_bar", "root_id_per_tick_nm", "solver_fwdinv_per_tick",
                "torque_bar", "qvel", "stop",
            ):
                row.pop(key, None)
        rows.append(row)
        if _plant_md5() != PLANT_MD5:
            raise SystemExit("plant md5 changed")
    payload = {
        "tip": TIP_SHA,
        "soft_pass": False,
        "plant_md5": PLANT_MD5,
        "runtime_only": True,
        "qacc_source": QACC_SOURCE,
        "integrator_note": (
            "The plant timestep is 0.002 s and the integrator is implicitfast. "
            "Cone, impratio, and noslip are the MuJoCo defaults: pyramidal cone, "
            "impratio 1, noslip iterations 0. Under implicitfast, damping is "
            "implicit, so the raw residual can carry an offset. The logged "
            "offset is dt*(0.08 + kv)*qacc, signed. The adjusted residual is "
            "the raw residual minus that offset."
        ),
        "solver": None if not rows else rows[0]["solver"],
        "residual_floor_nm": RESIDUAL_FLOOR,
        "mujoco_version": mj.__version__,
        "invdiscrete": _invdiscrete_bit() is not None,
        "invdiscrete_flag": "mjtEnableBit.mjENBL_INVDISCRETE",
        "discrete_residual_nm": DISCRETE_RESIDUAL_NM,
        "match_residual_nm": DISCRETE_RESIDUAL_NM,
        "bucket_residual_nm": BUCKET_RESIDUAL_NM,
        "match_residual_definition": (
            "|qfrc_inverse - qfrc_actuator| on each leg joint and each root dof. "
            "Exactly 1e-3 stays inside the Prefer FAIL target. Exactly 1e-2 stays "
            "inside the bucket. Counts over each threshold are both reported."
        ),
        "discrete_rule": (
            "mujoco.__version__ is checked for mjtEnableBit.mjENBL_INVDISCRETE. "
            "On this build the flag exists and is set on the inverse copy only. "
            "qacc is (qvel_next - qvel) / dt. The match residual is "
            "|qfrc_inverse - qfrc_actuator| on each leg joint and each root dof. "
            "A tick over 1e-3 Nm is a Prefer FAIL count. A tick is bucketed when "
            "every one of those residuals is <= 1e-2 Nm. A tick over 1e-2 Nm is "
            "residual over 1e-2, not bucketed. A joint with |applied| >= 2.45 Nm "
            "is clamped, unclassifiable, because the realised inverse returns "
            "the clamp. The passive-inclusive residual stays a comparison column. "
            "The inverse column is data_copy.qfrc_inverse only. "
            "A knee inverse value exactly ±2.450 Nm is a column bug. "
            "The planned-motion τ_req does not change the pass, including a "
            "double-support tick where the QP split still needs more than "
            "2.33 Nm without armature. The realised contact split versus "
            "that QP does not change the pass. A QP CoP that sits on the "
            "box shrunk by step_bars.EDGE_DWELL_M is logged and does not "
            "change the pass. A joint that clears 2.33 only "
            "after subtracting armature·q̈_ref is an unsourced-armature candidate."
        ),
        "planned_motion": (
            "τ_req is mj_inverse on the walker's q_des with 8 ms qvel and qacc, "
            "contacts off, and the root force applied through the planned ZMP. "
            "Double support reports the foot-line split, the min-norm ankle "
            "split, and a linear program that minimises the largest |τ| over "
            "the leg joints with armature removed. The program keeps each "
            "foot's centre of pressure inside the 135×76 mm box shrunk on "
            "every side by step_bars.EDGE_DWELL_M and each corner force inside "
            "a friction cone of μ 1.2. The per-foot CoP margin is logged on "
            "every feasible double-support tick. A tick whose reported CoP "
            "lies on that shrunk edge is flagged. The flag does not change "
            "the pass. A wall is only a double-support tick where that "
            "program still needs more than 2.33 Nm without armature. Linear "
            "and min-norm exceeding 2.33 are not a wall. Single support above "
            "2.33 without armature is required single-foot torque. The "
            "realised double-support contact split is compared with the QP "
            "split. A position servo does not command the split. Neither "
            "comparison changes the pass."
        ),
        "band_cap_nm": BAND_CAP_NM,
        "band_cap_rule": (
            "The band and its 0.15 Nm cap are comparison columns. They do not "
            "decide the bucket once the discrete residual is available. "
            "A tick can still be counted in Cap or ResFail. "
            "The band does not relax the signed applied bar of 2.33 Nm."
        ),
        "residual_band": "0.05 + dt*(0.08 + kv)*|qacc| per leg joint per 0.002 s step",
        "residual_definition": "signed qfrc_inverse - (qfrc_actuator + qfrc_passive) per leg joint; comparison column, not the bucket",
        "implicitfast_offset": "signed dt*(0.08 + kv)*qacc; kv = -actuator_biasprm[i,2]; 0.08 is leg joint damping",
        "residual_adjusted": "signed raw residual minus the implicitfast offset",
        "fwdinv": (
            "mjENBL_FWDINV is set on a binary copy of the model. mj_forward, "
            "mj_compareFwdInv, and mj_inverse run on that copy. The live model "
            "enableflags stay 0, so the forward rollout does not see the flag. "
            "solver_fwdinv[0] is the constraint-force norm and [1] is the applied-force norm."
        ),
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
            "id": "data_copy.qfrc_inverse from mj_inverse on the pre-step copy, qacc = (qvel_next - qvel) / dt, mjENBL_INVDISCRETE set on that copy only",
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
