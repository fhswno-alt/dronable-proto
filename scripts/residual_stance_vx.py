#!/usr/bin/env python3
"""Residual stance-vx helpers (sim only).

v1: linear K from open-loop −k·vx teacher — FALSIFIED (RES00–03).
v2a: phase-gated linear K_ss, SS-stance-only apply, CMA-ES black-box fit
     (NOT imitate vnull). See docs/RESIDUAL_V2_COSPEC.md.
v2b: ≤2×64 MLP (separate path if v2a fails Gate E).

No spend / Path language. Assist/freeze/xfrc forbidden at apply (caller).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WALK = ROOT / "scripts" / "walk_gait_ainex.py"
PLANT_M145 = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
ITER = ROOT / "previews" / "ainex_walk" / "iterate"
PY = ROOT / ".venv" / "bin" / "python"

# Obs: [vx, vy, com_vx, yaw_rate, phi, sinφ, cosφ, q_leg(6), dq_leg(6), cL, cR]
OBS_DIM = 21
LEG_JOINTS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll")

# CSV columns from walk_gait_ainex timeseries (v1 fit legacy)
COLS = [
    "t", "ctrl_l_hip", "ctrl_r_hip", "q_l_hip", "q_r_hip",
    "z_l_foot", "z_r_foot", "body_z", "x_l_foot", "x_r_foot", "body_x",
    "q_l_knee", "q_r_knee", "q_l_ank", "q_r_ank",
    "thigh_pitch_l", "thigh_pitch_r", "shin_pitch_l", "shin_pitch_r",
    "body_y", "up_z", "contact_L", "contact_R", "vx_L", "vx_R", "lat", "amp",
]

HX_LEG = 2.1
AUTH_CTRL_LIM = 2.09  # scaled by --auth-k in walk_gait
HX_ARM = 0.7
CONTACT_Z_THR_DEFAULT = 0.025


def foot_floor_contact(model, data, bid_foot, gid_floor, z_thr: float = CONTACT_Z_THR_DEFAULT) -> bool:
    """True if foot body contacts floor geom; z fallback matches walker."""
    if gid_floor is not None and gid_floor >= 0:
        for i in range(data.ncon):
            c = data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            b1 = int(model.geom_bodyid[g1])
            b2 = int(model.geom_bodyid[g2])
            if (b1 == bid_foot or b2 == bid_foot) and (g1 == gid_floor or g2 == gid_floor):
                return True
    return float(data.xpos[bid_foot, 2]) < z_thr



def _load_csv(path: Path) -> np.ndarray:
    rows = []
    with path.open() as f:
        for line in f:
            if line.startswith("#") or line.startswith("t,"):
                continue
            parts = line.strip().split(",")
            if len(parts) < 27:
                continue
            try:
                rows.append([float(x) for x in parts[:27]])
            except ValueError:
                continue
    return np.asarray(rows, dtype=np.float64) if rows else np.zeros((0, 27))


def _obs_row_v1(r: np.ndarray, side: str) -> np.ndarray:
    """Legacy 9-dim obs for v1."""
    if side == "L":
        vx = r[23]
        qh, qk, qa = r[3], r[11], r[13]
        c_self = r[21]
    else:
        vx = r[24]
        qh, qk, qa = r[4], r[12], r[14]
        c_self = r[22]
    return np.array(
        [vx, r[25], r[26], qh, qk, qa, r[21], r[22], c_self],
        dtype=np.float64,
    )


def fit_linear(
    csv_paths: list[Path],
    vx_deadband: float = 0.02,
    ridge: float = 1e-3,
) -> dict:
    """v1 LS teacher (open-loop −k·vx) — kept for replay; do NOT use for Gate E."""
    X_list, y_hip, y_ank = [], [], []
    for p in csv_paths:
        a = _load_csv(p)
        if a.size == 0:
            continue
        for r in a:
            for side, jsign in (("L", -1.0), ("R", +1.0)):
                c = r[21] if side == "L" else r[22]
                z = r[5] if side == "L" else r[6]
                vx = r[23] if side == "L" else r[24]
                if c < 0.5 or z > 0.035:
                    continue
                if abs(vx) < vx_deadband:
                    continue
                obs = _obs_row_v1(r, side)
                d_hip = jsign * float(np.clip(-0.55 * vx, -0.12, 0.12))
                d_ank = 0.6 * d_hip
                X_list.append(obs)
                y_hip.append(d_hip)
                y_ank.append(d_ank)
    if not X_list:
        raise SystemExit("no stance rows for fit — need timeseries with contact+vx")
    X = np.asarray(X_list)
    mean = X.mean(axis=0)
    std = X.std(axis=0) + 1e-6
    Xn = (X - mean) / std
    d = Xn.shape[1]
    A = Xn.T @ Xn + ridge * np.eye(d)
    K_hip = np.linalg.solve(A, Xn.T @ np.asarray(y_hip))
    K_ank = np.linalg.solve(A, Xn.T @ np.asarray(y_ank))
    return {
        "mode": "v1_linear",
        "K_hip": K_hip,
        "K_ank": K_ank,
        "obs_mean": mean,
        "obs_std": std,
        "n_rows": int(len(X_list)),
        "obs_dim": int(d),
    }


def save_npz(path: Path, fit: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {k: v for k, v in fit.items() if k != "mode"}
    mode = str(fit.get("mode", "v1_linear"))
    np.savez(path, mode=np.array(mode), **{
        k: (np.asarray(v) if not isinstance(v, (str, bytes)) else np.array(v))
        for k, v in payload.items()
        if not isinstance(v, dict)
    })


def load_npz(path: Path) -> dict:
    z = np.load(path, allow_pickle=True)
    out = {}
    for k in z.files:
        v = z[k]
        if k == "mode":
            out["mode"] = str(v.item() if v.shape == () else v)
        elif v.shape == ():
            out[k] = v.item()
        else:
            out[k] = np.asarray(v, dtype=np.float64)
    if "mode" not in out:
        # legacy v1 npz
        out["mode"] = "v1_linear"
    return out


def residual_deltas(
    fit: dict,
    vx: float,
    lat: float,
    amp: float,
    q_hip: float,
    q_knee: float,
    q_ank: float,
    contact_L: float,
    contact_R: float,
    side: str,
    gain: float = 1.0,
) -> tuple[float, float]:
    """v1 path only."""
    c_self = contact_L if side == "L" else contact_R
    obs = np.array(
        [vx, lat, amp, q_hip, q_knee, q_ank, contact_L, contact_R, c_self],
        dtype=np.float64,
    )
    xn = (obs - fit["obs_mean"]) / fit["obs_std"]
    d_hip = float(gain * fit["K_hip"] @ xn)
    d_ank = float(gain * fit["K_ank"] @ xn)
    return d_hip, d_ank


def clip_total_hx(ctrl, act_idx: dict, *, joints: list[str] | None = None) -> None:
    """Clip position setpoints we touched to leg ctrlrange (~±2.09).

    HX ±2.1/±0.7 is actuator *forcerange* (torque), enforced by MuJoCo — do NOT
    crush arm position targets (e.g. sho_roll ±1.4) to ±0.7; that alone tips CSF50.
    """
    CTRL_LIM = float(AUTH_CTRL_LIM)  # matches plant ctrlrange (±auth-k)
    if joints is None:
        # only legs — never rewrite arms/head here
        keys = [n for n in act_idx if any(
            k in n.lower() for k in ("hip_", "knee", "ank_")
        )]
    else:
        keys = joints
    for name in keys:
        if name in act_idx:
            i = act_idx[name]
            ctrl[i] = float(np.clip(ctrl[i], -CTRL_LIM, CTRL_LIM))


def _q_dq(model, data, jname: str, jnt_qpos=None) -> tuple[float, float]:
    import mujoco as mj
    if jnt_qpos is not None and jname in jnt_qpos:
        adr = jnt_qpos[jname]
        # qpos adr only — need dof for dq
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jname)
        if jid < 0:
            return 0.0, 0.0
        return float(data.qpos[adr]), float(data.qvel[model.jnt_dofadr[jid]])
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jname)
    if jid < 0:
        return 0.0, 0.0
    return float(data.qpos[model.jnt_qposadr[jid]]), float(data.qvel[model.jnt_dofadr[jid]])


def build_obs_v2(
    model,
    data,
    *,
    side: str,
    bid: int,
    phi: float,
    cL: float,
    cR: float,
    jnt_qpos=None,
) -> np.ndarray:
    """Co-spec obs (21,). Stance-leg q/dq."""
    vx = float(data.cvel[bid][3])
    vy = float(data.cvel[bid][4])
    com_vx = float(data.qvel[0])
    yaw_rate = float(data.qvel[5]) if data.qvel.shape[0] > 5 else 0.0
    sphi, cphi = math.sin(2.0 * math.pi * phi), math.cos(2.0 * math.pi * phi)
    pref = "l_" if side == "L" else "r_"
    qdq = []
    for jn in LEG_JOINTS:
        q, dq = _q_dq(model, data, f"{pref}{jn}", jnt_qpos=jnt_qpos)
        qdq.extend([q, dq])
    # q_leg(6) then dq_leg(6)
    qs = qdq[0::2]
    dqs = qdq[1::2]
    return np.array(
        [vx, vy, com_vx, yaw_rate, phi, sphi, cphi, *qs, *dqs, cL, cR],
        dtype=np.float64,
    )


def is_ss_stance(side: str, cL: float, cR: float, lat: float) -> bool:
    """SS stance only: contact + peer swing; ZERO in DS / swing.

    Exclusive contact is the hard gate (co-spec). Lat soft-confirms weight shift
    but is not required — CPG lat can lag true SS and would starve the residual.
    """
    if side == "L":
        return (cL >= 0.5) and (cR < 0.5)
    return (cR >= 0.5) and (cL < 0.5)


def pack_v2a(K_hip: np.ndarray, K_ank: np.ndarray, K_roll: np.ndarray | None,
             obs_mean: np.ndarray, obs_std: np.ndarray, meta: dict | None = None) -> dict:
    d = {
        "mode": "v2a",
        "K_hip": np.asarray(K_hip, dtype=np.float64).ravel(),
        "K_ank": np.asarray(K_ank, dtype=np.float64).ravel(),
        "obs_mean": np.asarray(obs_mean, dtype=np.float64).ravel(),
        "obs_std": np.asarray(obs_std, dtype=np.float64).ravel(),
        "obs_dim": OBS_DIM,
        "n_rows": int((meta or {}).get("n_rows", 0)),
        "use_roll": 1.0 if K_roll is not None else 0.0,
    }
    if K_roll is not None:
        d["K_roll"] = np.asarray(K_roll, dtype=np.float64).ravel()
    return d


def make_zero_v2a(obs_mean: np.ndarray | None = None, obs_std: np.ndarray | None = None,
                  use_roll: bool = False) -> dict:
    mean = obs_mean if obs_mean is not None else np.zeros(OBS_DIM)
    std = obs_std if obs_std is not None else np.ones(OBS_DIM)
    std = np.maximum(std, 1e-3)
    K_roll = np.zeros(OBS_DIM) if use_roll else None
    return pack_v2a(np.zeros(OBS_DIM), np.zeros(OBS_DIM), K_roll, mean, std)


def apply_residual_stance_vx(
    model,
    data,
    act_idx: dict,
    phi: float,
    amp: float,
    bid_lf: int,
    bid_rf: int,
    fit: dict | None,
    *,
    gain: float = 1.0,
    lateral_com_target=None,
    contact_z_thr: float = CONTACT_Z_THR_DEFAULT,
    act_name_fn=None,
    jnt_qpos=None,
    gid_floor: int | None = None,
) -> dict:
    """Additive residual after gait/CP/VIK. HX clip inside.

    v1_linear: legacy open-loop-teacher K (falsified).
    v2a: SS-stance-only phase-gated linear K_ss on co-spec obs.
    """
    diag = {"applied": False, "mode": None}
    if fit is None or amp < 0.05:
        return diag
    mode = str(fit.get("mode", "v1_linear"))
    diag["mode"] = mode
    if lateral_com_target is None:
        lat = math.cos(2.0 * math.pi * phi)
    else:
        lat = float(lateral_com_target(phi))
    if act_name_fn is None:
        act_name_fn = lambda j: f"{j}_pos"

    cL = 1.0 if foot_floor_contact(model, data, bid_lf, gid_floor, contact_z_thr) else 0.0
    cR = 1.0 if foot_floor_contact(model, data, bid_rf, gid_floor, contact_z_thr) else 0.0

    if mode in ("v2a", "v2b"):
        return _apply_v2a(
            model, data, act_idx, phi, amp, bid_lf, bid_rf, fit,
            gain=gain, lat=lat, cL=cL, cR=cR, contact_z_thr=contact_z_thr,
            act_name_fn=act_name_fn, jnt_qpos=jnt_qpos, diag=diag,
        )

    # ---- v1 path (legacy) ----
    def _q(jname: str) -> float:
        q, _ = _q_dq(model, data, jname, jnt_qpos=jnt_qpos)
        return q

    targets = []
    if lat > 0.05:
        targets.append(("L", bid_lf))
    if lat < -0.05:
        targets.append(("R", bid_rf))

    applied = False
    for side, bid in targets:
        if float(data.xpos[bid, 2]) > contact_z_thr + 0.01:
            continue
        vx = float(data.cvel[bid][3])
        if abs(vx) < 0.015:
            continue
        pref = "l_" if side == "L" else "r_"
        d_hip, d_ank = residual_deltas(
            fit, vx, lat, amp, _q(f"{pref}hip_pitch"), _q(f"{pref}knee"),
            _q(f"{pref}ank_pitch"), cL, cR, side, gain=gain,
        )
        d_hip = float(np.clip(d_hip, -0.15, 0.15))
        d_ank = float(np.clip(d_ank, -0.10, 0.10))
        for jn, dd in ((f"{pref}hip_pitch", d_hip), (f"{pref}ank_pitch", d_ank)):
            an = act_name_fn(jn)
            if an in act_idx:
                data.ctrl[act_idx[an]] = float(data.ctrl[act_idx[an]] + dd)
        applied = True
        diag[f"delta_{side}"] = [d_hip, d_ank]

    clip_total_hx(data.ctrl, act_idx)
    diag["applied"] = applied
    return diag


def _apply_v2a(
    model, data, act_idx, phi, amp, bid_lf, bid_rf, fit, *,
    gain, lat, cL, cR, contact_z_thr, act_name_fn, jnt_qpos, diag,
) -> dict:
    mean = fit["obs_mean"]
    std = np.maximum(fit["obs_std"], 1e-6)
    K_hip = fit.get("K_hip", np.zeros(OBS_DIM))
    K_ank = fit.get("K_ank", np.zeros(OBS_DIM))
    use_roll = float(fit.get("use_roll", 0.0)) > 0.5 and "K_roll" in fit
    K_roll = fit.get("K_roll") if use_roll else None

    # soft clips from fit or defaults (still under HX total)
    clip_hip = float(fit.get("clip_hip", 0.18))
    clip_ank = float(fit.get("clip_ank", 0.12))
    clip_roll = float(fit.get("clip_roll", 0.08))

    targets = []
    if is_ss_stance("L", cL, cR, lat):
        targets.append(("L", bid_lf))
    if is_ss_stance("R", cL, cR, lat):
        targets.append(("R", bid_rf))

    applied = False
    for side, bid in targets:
        obs = build_obs_v2(
            model, data, side=side, bid=bid, phi=phi, cL=cL, cR=cR, jnt_qpos=jnt_qpos,
        )
        xn = (obs - mean) / std
        jsign = -1.0 if side == "L" else +1.0
        mode_r = str(fit.get("mode", "v2a"))
        if mode_r == "v2b" and "W1" in fit and "W2" in fit:
            # ≤2×64 MLP: tanh hidden, readout → [d_fwd_hip, d_fwd_ank]
            h = np.tanh(fit["W1"] @ xn + fit.get("b1", 0.0))
            y = fit["W2"] @ h + fit.get("b2", 0.0)
            d_fwd_hip = float(gain * y[0])
            d_fwd_ank = float(gain * y[1])
        else:
            d_fwd_hip = float(gain * K_hip @ xn)
            d_fwd_ank = float(gain * K_ank @ xn)
        d_fwd_hip = float(np.clip(d_fwd_hip, -clip_hip, clip_hip))
        d_fwd_ank = float(np.clip(d_fwd_ank, -clip_ank, clip_ank))
        d_hip = jsign * d_fwd_hip
        d_ank = jsign * d_fwd_ank
        pref = "l_" if side == "L" else "r_"
        for jn, dd in ((f"{pref}hip_pitch", d_hip), (f"{pref}ank_pitch", d_ank)):
            an = act_name_fn(jn)
            if an in act_idx:
                data.ctrl[act_idx[an]] = float(data.ctrl[act_idx[an]] + dd)
        if K_roll is not None:
            # roll: + toward midline for stance — use raw K (side asymmetries in obs)
            d_roll = float(np.clip(gain * K_roll @ xn, -clip_roll, clip_roll))
            an = act_name_fn(f"{pref}hip_roll")
            if an in act_idx:
                data.ctrl[act_idx[an]] = float(data.ctrl[act_idx[an]] + d_roll)
            diag[f"delta_{side}"] = [d_hip, d_ank, d_roll]
        else:
            diag[f"delta_{side}"] = [d_hip, d_ank]
        applied = True

    clip_total_hx(data.ctrl, act_idx)
    diag["applied"] = applied
    return diag


# ---------------------------------------------------------------------------
# CMA-ES black-box fit for v2a
# ---------------------------------------------------------------------------

CSF50 = dict(
    gait_t=0.88, hip_amp=0.116, ds=0.31, step_len=0.021,
    com_shift=0.275, com_shift_lead=0.23, knee_stance=0.4, knee_swing=0.8,
    com_z=0.225, hip_bias=0.06, plant_kd=115, cp=True, vik=True,
    vik_kp=0.45, vik_clip=0.03,
)

T75 = dict(CSF50, gait_t=0.75, ds=0.26)
T80 = dict(CSF50, gait_t=0.80, ds=0.28)


def _default_obs_norm() -> tuple[np.ndarray, np.ndarray]:
    """Conservative fixed scale so CMA is scale-aware without teacher."""
    # vx, vy, com_vx, yaw_rate, phi, sin, cos, q×6, dq×6, cL, cR
    mean = np.zeros(OBS_DIM)
    std = np.array([
        0.25, 0.08, 0.15, 0.8,   # vx vy com_vx yaw
        0.5, 0.7, 0.7,            # phi sin cos
        0.3, 0.2, 0.3, 0.5, 0.4, 0.2,  # q leg
        1.0, 1.0, 1.5, 2.0, 2.0, 1.5,  # dq leg
        0.5, 0.5,                 # contacts
    ], dtype=np.float64)
    return mean, std


def _theta_to_fit(theta: np.ndarray, obs_mean, obs_std, use_roll: bool = False) -> dict:
    """theta = [K_hip (21), K_ank (21), optional K_roll (21)]."""
    n = OBS_DIM
    K_hip = theta[0:n]
    K_ank = theta[n:2 * n]
    K_roll = theta[2 * n:3 * n] if use_roll and len(theta) >= 3 * n else None
    fit = pack_v2a(K_hip, K_ank, K_roll, obs_mean, obs_std)
    fit["clip_hip"] = 0.28
    fit["clip_ank"] = 0.18
    fit["clip_roll"] = 0.08
    return fit


def run_rollout(
    *,
    residual_path: Path | None,
    residual_gain: float,
    params: dict,
    tag: str,
    duration: float = 9.0,
    video: bool = False,
    work: Path | None = None,
) -> dict:
    """Run walk_gait_ainex once; return stats + light SS bout from csv."""
    work = work or (ITER / "_v2a_cma")
    work.mkdir(parents=True, exist_ok=True)
    stats_out = work / f"{tag}.json"
    out_mp4 = work / f"{tag}.mp4"
    cmd = [
        str(PY), str(WALK),
        "--model", str(PLANT_M145),
        "--duration", str(duration),
        "--tag", tag,
        "--gait-t", str(params["gait_t"]),
        "--hip-amp", str(params["hip_amp"]),
        "--ds", str(params["ds"]),
        "--step-len", str(params["step_len"]),
        "--com-shift", str(params["com_shift"]),
        "--com-shift-lead", str(params["com_shift_lead"]),
        "--knee-stance", str(params["knee_stance"]),
        "--knee-swing", str(params["knee_swing"]),
        "--com-z", str(params["com_z"]),
        "--hip-bias", str(params["hip_bias"]),
        "--plant-kd", str(params["plant_kd"]),
        "--out", str(out_mp4),
        "--stats-out", str(stats_out),
    ]
    if not video:
        cmd.append("--no-video")
    if params.get("cp"):
        cmd.append("--cp-swing")
    if params.get("vik"):
        cmd += ["--stance-vik", "--vik-ankle",
                "--vik-kp", str(params["vik_kp"]),
                "--vik-clip", str(params["vik_clip"])]
    if residual_path is not None:
        cmd += ["--residual", str(residual_path), "--residual-gain", str(residual_gain)]
    env = dict(os.environ)
    env["MUJOCO_GL"] = "egl"
    r = subprocess.run(cmd, cwd=str(ROOT), env=env, capture_output=True, text=True)
    if not stats_out.exists():
        return {
            "tag": tag, "error": f"rc={r.returncode}",
            "stderr": (r.stderr or "")[-500:],
            "tip": 0.0, "dx": 0.0, "stx_mean": 9.0, "stx_p95": 9.0,
            "bout_min": 0.0, "hip_corr": 0.0, "skate": True, "flipped": True,
        }
    stats = json.loads(stats_out.read_text())
    # bout from timeseries (SS = exclusive contact)
    csv = work / f"{tag}_timeseries.csv"
    # walk_gait writes next to stats-out
    candidates = [
        Path(str(stats_out).replace(".json", "_timeseries.csv")),
        work / "ainex_walk_timeseries.csv",
        ITER / "ainex_walk_timeseries.csv",
        ROOT / "previews" / "ainex_walk" / "ainex_walk_timeseries.csv",
    ]
    existing = [c for c in candidates if c.exists()]
    bout_L = bout_R = 0.0
    n_bouts_L = n_bouts_R = 0
    peak_clear_L = peak_clear_R = 0.0
    if existing:
        src = max(existing, key=lambda p: p.stat().st_mtime)
        if src.resolve() != csv.resolve():
            shutil.copy2(src, csv)
        a = _load_csv(csv)
        if len(a) > 50:
            i0 = int(2.4 * 50)  # skip stand
            ss = a[i0:] if len(a) > i0 else a
            amp = ss[:, 26]
            mask = amp > 0.2
            if not np.any(mask):
                mask = np.ones(len(ss), dtype=bool)
            cL = ss[mask, 21] > 0.5
            cR = ss[mask, 22] > 0.5
            ssL = cL & ~cR
            ssR = cR & ~cL
            zl = ss[mask, 5]
            zr = ss[mask, 6]
            sole = 0.026

            def _runs(m):
                out = []
                i, n = 0, len(m)
                while i < n:
                    if m[i]:
                        j = i
                        while j < n and m[j]:
                            j += 1
                        out.append((j - i) / 50.0)
                        i = j
                    else:
                        i += 1
                return out

            rl, rr = _runs(ssL), _runs(ssR)
            bout_L = float(np.mean(rl)) if rl else 0.0
            bout_R = float(np.mean(rr)) if rr else 0.0
            n_bouts_L, n_bouts_R = len(rl), len(rr)
            peak_clear_L = float(np.max(zl[ssL] - sole)) if np.any(ssL) else 0.0
            peak_clear_R = float(np.max(zr[ssR] - sole)) if np.any(ssR) else 0.0

    stx_L = float(stats.get("stance_foot_vx_mean_L") or stats.get("stance_vx_mean_L") or 9.0)
    stx_R = float(stats.get("stance_foot_vx_mean_R") or stats.get("stance_vx_mean_R") or 9.0)
    p95_L = float(stats.get("stance_foot_vx_p95_L") or 9.0)
    p95_R = float(stats.get("stance_foot_vx_p95_R") or 9.0)
    tip = float(stats.get("tip_free_post_gait_max_s") or stats.get("tip_free_max_s") or 0.0)
    dx = float(stats.get("dx_m") or 0.0)
    hip_corr = float(stats.get("hip_q_corr_LR") or 0.0)
    lead = int(stats.get("foot_lead_cycles") or 0)
    return {
        "tag": tag,
        "tip": tip,
        "dx": dx,
        "stx_mean": 0.5 * (stx_L + stx_R),
        "stx_p95": 0.5 * (p95_L + p95_R),
        "stx_L": stx_L, "stx_R": stx_R,
        "p95_L": p95_L, "p95_R": p95_R,
        "bout_L": bout_L, "bout_R": bout_R,
        "bout_min": min(bout_L, bout_R) if (bout_L > 0 or bout_R > 0) else 0.0,
        "n_bouts_L": n_bouts_L, "n_bouts_R": n_bouts_R,
        "peak_clear_L": peak_clear_L, "peak_clear_R": peak_clear_R,
        "hip_corr": hip_corr,
        "foot_lead": lead,
        "skate": bool(stats.get("skate")),
        "flipped": bool(stats.get("flipped")),
        "exploded": bool(stats.get("exploded")),
        "assist": bool(stats.get("balance_assist")),
        "freeze": bool(stats.get("stance_freeze")),
        "stats_path": str(stats_out),
        "csv_path": str(csv) if csv.exists() else None,
        "mp4": str(out_mp4) if out_mp4.exists() else None,
        "rc": r.returncode,
    }


def cost_from_metrics(m: dict, *, duration: float = 9.0) -> float:
    """Co-spec objective + constraint penalties. Lower is better."""
    if m.get("error") or m.get("flipped") or m.get("exploded"):
        return 80.0
    tip_need = min(8.0, duration - 0.05)
    cost = 0.5 * float(m["stx_mean"]) + 0.5 * float(m["stx_p95"])
    if m["tip"] < tip_need:
        cost += 10.0 + 2.0 * (tip_need - m["tip"])
    if m["dx"] < 0.05:
        cost += 5.0 + 40.0 * (0.05 - m["dx"])
    if m["bout_min"] < 0.10:
        cost += 5.0 + 30.0 * (0.10 - m["bout_min"])
    if m["hip_corr"] > -0.45:
        cost += 3.0 + 5.0 * (m["hip_corr"] + 0.45)
    return float(cost)


def t88_reject(m: dict) -> bool:
    """Hard reject if RES00-class T=0.88 clean_ss regresses."""
    p95 = max(m.get("p95_L", 9), m.get("p95_R", 9))
    bout = m.get("bout_min", 0.0)
    if p95 > 0.18:
        return True
    if bout < 0.10:  # ≪ 0.12 knife-edge
        return True
    if m.get("flipped") or m.get("exploded"):
        return True
    if m.get("tip", 0) < 7.5:
        return True
    return False


def cma_fit_v2a(
    *,
    out_path: Path,
    maxiter: int = 18,
    popsize: int = 10,
    sigma0: float = 0.08,
    use_roll: bool = False,
    duration: float = 9.0,
    seed: int = 27,
) -> dict:
    import cma

    obs_mean, obs_std = _default_obs_norm()
    n = OBS_DIM * (3 if use_roll else 2)
    x0 = np.zeros(n)
    work = ITER / "_v2a_cma"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True, exist_ok=True)

    eval_i = {"n": 0}
    best = {"cost": 1e9, "theta": x0.copy(), "m75": None, "m88": None}

    def _write_tmp(theta: np.ndarray, name: str) -> Path:
        fit = _theta_to_fit(theta, obs_mean, obs_std, use_roll=use_roll)
        p = work / f"{name}.npz"
        save_npz(p, fit)
        return p

    def fitness(theta: np.ndarray) -> float:
        eval_i["n"] += 1
        eid = eval_i["n"]
        npz = _write_tmp(theta, f"cand_{eid}")
        # Primary: T=0.75
        m75 = run_rollout(
            residual_path=npz, residual_gain=1.0, params=T75,
            tag=f"c75_{eid}", duration=duration, video=False, work=work,
        )
        c = cost_from_metrics(m75, duration=duration)
        # Periodic / early T=0.88 reject on every 4th eval + always if looking good
        do88 = (eid % 3 == 0) or (c < 0.45) or (eid <= 5)
        m88 = None
        if do88:
            m88 = run_rollout(
                residual_path=npz, residual_gain=1.0, params=CSF50,
                tag=f"c88_{eid}", duration=duration, video=False, work=work,
            )
            if t88_reject(m88):
                c += 25.0 + 10.0 * max(0.0, max(m88["p95_L"], m88["p95_R"]) - 0.18)
                c += 10.0 * max(0.0, 0.10 - m88["bout_min"])
        # Also lightly include T=0.80 every 5th
        if eid % 5 == 0:
            m80 = run_rollout(
                residual_path=npz, residual_gain=1.0, params=T80,
                tag=f"c80_{eid}", duration=duration, video=False, work=work,
            )
            c = 0.7 * c + 0.3 * cost_from_metrics(m80, duration=duration)

        if c < best["cost"]:
            best["cost"] = c
            best["theta"] = np.asarray(theta, dtype=np.float64).copy()
            best["m75"] = m75
            best["m88"] = m88
            print(
                f"[cma] NEW best #{eid} cost={c:.4f} "
                f"stx={m75['stx_mean']:.3f}/{m75['stx_p95']:.3f} "
                f"bout={m75['bout_min']:.3f} tip={m75['tip']:.2f} dx={m75['dx']:+.3f}"
                + (f" | T88 p95={max(m88['p95_L'], m88['p95_R']):.3f} bout={m88['bout_min']:.3f}"
                   if m88 else ""),
                flush=True,
            )
        elif eid % 10 == 0:
            print(
                f"[cma] eval #{eid} cost={c:.4f} stx={m75['stx_mean']:.3f}/{m75['stx_p95']:.3f}",
                flush=True,
            )
        return c

    opts = cma.CMAOptions()
    opts.set("seed", seed)
    opts.set("maxiter", maxiter)
    opts.set("popsize", popsize)
    opts.set("bounds", [-1.0, 1.0])
    opts.set("verbose", -9)
    print(
        f"[cma] start n={n} pop={popsize} maxiter={maxiter} sigma0={sigma0} "
        f"duration={duration}s use_roll={use_roll}",
        flush=True,
    )
    t0 = time.time()
    es = cma.CMAEvolutionStrategy(x0, sigma0, opts)
    es.optimize(fitness)
    elapsed = time.time() - t0
    print(f"[cma] done in {elapsed:.1f}s evals={eval_i['n']} best_cost={best['cost']:.4f}", flush=True)

    # Final T=0.88 reject on best
    final_theta = best["theta"]
    npz = _write_tmp(final_theta, "best_final")
    m88 = run_rollout(
        residual_path=npz, residual_gain=1.0, params=CSF50,
        tag="best_T88", duration=duration, video=False, work=work,
    )
    m75 = run_rollout(
        residual_path=npz, residual_gain=1.0, params=T75,
        tag="best_T75", duration=duration, video=False, work=work,
    )
    rejected = t88_reject(m88)
    fit = _theta_to_fit(final_theta, obs_mean, obs_std, use_roll=use_roll)
    fit["n_rows"] = eval_i["n"]
    fit["cma_best_cost"] = best["cost"]
    fit["cma_t88_reject"] = 1.0 if rejected else 0.0
    save_npz(out_path, fit)
    # also save json sidecar
    side = {
        "out": str(out_path),
        "evals": eval_i["n"],
        "best_cost": best["cost"],
        "t88_reject": rejected,
        "m75": {k: m75[k] for k in (
            "tip", "dx", "stx_mean", "stx_p95", "stx_L", "stx_R", "p95_L", "p95_R",
            "bout_L", "bout_R", "bout_min", "hip_corr", "skate", "n_bouts_L", "n_bouts_R",
            "peak_clear_L", "peak_clear_R", "foot_lead",
        )},
        "m88": {k: m88[k] for k in (
            "tip", "dx", "stx_mean", "stx_p95", "stx_L", "stx_R", "p95_L", "p95_R",
            "bout_L", "bout_R", "bout_min", "hip_corr", "skate",
        )},
        "elapsed_s": elapsed,
        "theta_norm": float(np.linalg.norm(final_theta)),
    }
    (ITER / "residual_stance_vx_v2a_cma.json").write_text(json.dumps(side, indent=2))
    print(json.dumps(side, indent=2), flush=True)
    return side


def main() -> None:
    ap = argparse.ArgumentParser(description="Residual stance-vx fit / CMA v2a")
    ap.add_argument("csv", nargs="*", type=Path, help="timeseries CSV(s) for v1 LS")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--cma-v2a", action="store_true", help="CMA-ES black-box fit for v2a")
    ap.add_argument("--maxiter", type=int, default=18)
    ap.add_argument("--popsize", type=int, default=10)
    ap.add_argument("--sigma0", type=float, default=0.08)
    ap.add_argument("--use-roll", action="store_true")
    ap.add_argument("--duration", type=float, default=9.0)
    ap.add_argument("--seed", type=int, default=27)
    args = ap.parse_args()

    if args.cma_v2a:
        out = args.out or (ITER / "residual_stance_vx_v2a.npz")
        cma_fit_v2a(
            out_path=out, maxiter=args.maxiter, popsize=args.popsize,
            sigma0=args.sigma0, use_roll=args.use_roll, duration=args.duration,
            seed=args.seed,
        )
        return

    if not args.csv:
        ap.error("csv paths required unless --cma-v2a")
    out = args.out or (ITER / "residual_stance_vx_v1.npz")
    fit = fit_linear(args.csv)
    save_npz(out, fit)
    print(
        f"wrote {out} n_rows={fit['n_rows']} "
        f"|K_hip|={np.linalg.norm(fit['K_hip']):.4f} |K_ank|={np.linalg.norm(fit['K_ank']):.4f}"
    )


if __name__ == "__main__":
    main()
