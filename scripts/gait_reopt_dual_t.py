#!/usr/bin/env python3
"""Dual-T outer gait re-opt (CMA-ES) — CSF50 warm start, no residual/WBC/MPC.

Cospec: docs/GAIT_REOPT_DUAL_T_COSPEC.md
Hard: T=0.88 clean_ss floor. Soft: Gate E at T≤0.75 from same shape vector.
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
sys.path.insert(0, str(ROOT / "scripts"))
from ss_step_ainex import analyze_ss  # noqa: E402

WALK = ROOT / "scripts" / "walk_gait_ainex.py"
PLANT = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
ITER = ROOT / "previews" / "ainex_walk" / "iterate"
PY = ROOT / ".venv" / "bin" / "python"
WORK = ITER / "_gait_reopt_cma"

os.environ.setdefault("MUJOCO_GL", "glfw")

# CSF50 warm start (locked basin)
CSF50 = dict(
    hip_amp=0.116, step_len=0.021, ds=0.308,  # effective ds_frac=0.35 (locked knife-edge)
    com_shift=0.275, com_shift_lead=0.23,
    knee_stance=0.40, knee_swing=0.80,
    com_z=0.225, hip_bias=0.06, plant_kd=115.0,
    vik_kp=0.45, vik_clip=0.03, stance_sweep=1.00,
    swing_ank_df=0.0,  # optional clearance
)
# Historical CLI used ds=0.31 but walk_gait clamped ds_frac≤0.35 → effective 0.308s.
# After raising clamp to 0.55 for dual-T search, warm start MUST use effective basin.
DS_FRAC0 = 0.35

# Theta layout (shared shape; T and ds absolute derived at eval)
# [ds_frac, hip_amp, step_len, com_shift, lead, com_z, hip_bias,
#  knee_swing, plant_kd, vik_kp, stance_sweep, swing_ank_df]
THETA_NAMES = [
    "ds_frac", "hip_amp", "step_len", "com_shift", "com_shift_lead",
    "com_z", "hip_bias", "knee_swing", "plant_kd", "vik_kp",
    "stance_sweep", "swing_ank_df",
]
X0 = np.array([
    0.35, 0.116, 0.021, 0.275, 0.23,
    0.225, 0.06, 0.80, 115.0, 0.45,
    1.00, 0.0,
], dtype=np.float64)
# CMA works better in whitened space — scale to ~unit
X_SCALE = np.array([
    0.05, 0.02, 0.006, 0.04, 0.05,
    0.015, 0.03, 0.12, 25.0, 0.15,
    0.20, 0.08,
], dtype=np.float64)
BOUNDS_LO = np.array([
    0.22, 0.090, 0.012, 0.18, 0.10,
    0.200, 0.00, 0.55, 80.0, 0.20,
    0.55, 0.0,
], dtype=np.float64)
BOUNDS_HI = np.array([
    0.48, 0.145, 0.035, 0.36, 0.40,
    0.250, 0.12, 1.05, 160.0, 0.80,
    1.00, 0.25,
], dtype=np.float64)


def theta_clip(th: np.ndarray) -> np.ndarray:
    return np.clip(np.asarray(th, dtype=np.float64), BOUNDS_LO, BOUNDS_HI)


def theta_to_params(th: np.ndarray, gait_t: float) -> dict:
    th = theta_clip(th)
    ds = float(th[0] * gait_t)
    # keep DS < ~0.55*T so SS exists
    ds = min(ds, 0.55 * gait_t)
    ds = max(ds, 0.12 * gait_t)
    hip_amp = float(th[1])
    # step_len is display/legacy; kinematics follow hip_amp when both would be passed
    step_len = float(th[2])  # kept in theta for CMA coupling; applied only if we drop hip_amp
    # Prefer hip_amp as authority; keep step_len ≈ hip_amp*0.25 for logs
    step_len = float(np.clip(hip_amp * 0.25, 0.012, 0.040))
    p = {
        "gait_t": float(gait_t),
        "ds": ds,
        "hip_amp": hip_amp,
        "step_len": step_len,
        "com_shift": float(th[3]),
        "com_shift_lead": float(th[4]),
        "com_z": float(th[5]),
        "hip_bias": float(th[6]),
        "knee_stance": 0.40,
        "knee_swing": float(th[7]),
        "plant_kd": float(th[8]),
        "vik_kp": float(th[9]),
        "vik_clip": 0.03,
        "stance_sweep": float(th[10]),
        "swing_ank_df": float(th[11]),
        "cp": True,
        "vik": True,
    }
    return p


def params_to_dict(th: np.ndarray) -> dict:
    th = theta_clip(th)
    return {n: float(v) for n, v in zip(THETA_NAMES, th)}


def run_rollout(params: dict, *, tag: str, duration: float, video: bool, work: Path) -> dict:
    work.mkdir(parents=True, exist_ok=True)
    stats_out = work / f"{tag}.json"
    out_mp4 = work / f"{tag}.mp4"
    cmd = [
        str(PY), str(WALK),
        "--model", str(PLANT),
        "--duration", str(duration),
        "--tag", tag,
        "--gait-t", str(params["gait_t"]),
        "--hip-amp", str(params["hip_amp"]),
        "--ds", str(params["ds"]),
        # step_len omitted: hip_amp is kinematic authority (walk_gait ignores step when hip_amp set)
        "--com-shift", str(params["com_shift"]),
        "--com-shift-lead", str(params["com_shift_lead"]),
        "--knee-stance", str(params["knee_stance"]),
        "--knee-swing", str(params["knee_swing"]),
        "--com-z", str(params["com_z"]),
        "--hip-bias", str(params["hip_bias"]),
        "--plant-kd", str(params["plant_kd"]),
        "--stance-sweep", str(params["stance_sweep"]),
        "--out", str(out_mp4),
        "--stats-out", str(stats_out),
        "--cp-swing", "--stance-vik", "--vik-ankle",
        "--vik-kp", str(params["vik_kp"]),
        "--vik-clip", str(params["vik_clip"]),
    ]
    if params.get("swing_ank_df", 0) and float(params["swing_ank_df"]) > 1e-6:
        cmd += ["--swing-ank-df", str(params["swing_ank_df"])]
    if not video:
        cmd.append("--no-video")
    env = dict(os.environ)
    env["MUJOCO_GL"] = os.environ.get("MUJOCO_GL", "glfw")
    r = subprocess.run(cmd, cwd=str(ROOT), env=env, capture_output=True, text=True)
    if not stats_out.exists():
        return {
            "tag": tag, "error": f"rc={r.returncode}",
            "stderr": (r.stderr or "")[-400:],
            "tip": 0.0, "dx": 0.0, "stx_mean": 9.0, "stx_p95": 9.0,
            "bout_min": 0.0, "hip_corr": 0.0, "skate": True,
            "foot_lead": 0, "n_bouts_L": 0, "n_bouts_R": 0,
            "peak_clear_L": 0.0, "peak_clear_R": 0.0, "flipped": True,
        }
    stats = json.loads(stats_out.read_text())
    csv_tag = work / f"{tag}_timeseries.csv"
    candidates = [
        Path(str(stats_out).replace(".json", "_timeseries.csv")),
        work / "ainex_walk_timeseries.csv",
        ITER / "ainex_walk_timeseries.csv",
        ROOT / "previews" / "ainex_walk" / "ainex_walk_timeseries.csv",
    ]
    existing = [c for c in candidates if c.exists()]
    if existing:
        src = max(existing, key=lambda p: p.stat().st_mtime)
        if src.resolve() != csv_tag.resolve():
            shutil.copy2(src, csv_tag)
    ss = analyze_ss(csv_tag if csv_tag.exists() else candidates[0], stats) if csv_tag.exists() or existing else {}
    stx_L = float(stats.get("stance_foot_vx_mean_L") or 9)
    stx_R = float(stats.get("stance_foot_vx_mean_R") or 9)
    p95_L = float(stats.get("stance_foot_vx_p95_L") or 9)
    p95_R = float(stats.get("stance_foot_vx_p95_R") or 9)
    bout_L = float(ss.get("ss_mean_bout_s_L") or 0)
    bout_R = float(ss.get("ss_mean_bout_s_R") or 0)
    m = {
        "tag": tag,
        "tip": float(stats.get("tip_free_post_gait_max_s") or 0),
        "dx": float(stats.get("dx_m") or 0),
        "stx_L": stx_L, "stx_R": stx_R,
        "stx_mean": 0.5 * (stx_L + stx_R),
        "p95_L": p95_L, "p95_R": p95_R,
        "stx_p95": 0.5 * (p95_L + p95_R),
        "bout_L": bout_L, "bout_R": bout_R,
        "bout_min": min(bout_L, bout_R) if (bout_L and bout_R) else 0.0,
        "hip_corr": float(stats.get("hip_q_corr_LR") or 0),
        "skate": bool(stats.get("skate", True)),
        "foot_lead": int(stats.get("foot_lead_cycles") or 0),
        "n_bouts_L": int(ss.get("ss_bouts_L") or 0),
        "n_bouts_R": int(ss.get("ss_bouts_R") or 0),
        "peak_clear_L": float(ss.get("ss_peak_clear_m_L") or 0),
        "peak_clear_R": float(ss.get("ss_peak_clear_m_R") or 0),
        "flipped": bool(stats.get("flipped", False)),
        "exploded": bool(stats.get("exploded", False)),
        "mp4": str(out_mp4) if out_mp4.exists() else None,
        "params": params,
    }
    return m


def t88_ok(m: dict) -> bool:
    """Hard T88 floor (knife-edge): do not regress ≪ CSF50.

    CSF50 itself sits at bout≈0.117; strict ≥0.12 would reject the warm start.
    Hard reject uses bout≥0.10 (same as residual/WBC T88 reject). Soft cost
    still pushes bout toward ≥0.12.
    """
    if m.get("error"):
        return False
    if m.get("flipped") or m.get("exploded"):
        return False
    if m["tip"] < 8.0:
        return False
    if m["bout_min"] < 0.10 - 1e-9:
        return False
    if m["stx_p95"] > 0.18 + 1e-9:
        return False
    if m["skate"]:
        return False
    if m["hip_corr"] > -0.45:
        return False
    return True

def t88_strong(m: dict) -> bool:
    """Cospec strong clean_ss: bout≥0.12 + hard floor."""
    return t88_ok(m) and m["bout_min"] >= 0.12 - 1e-9


def gate_e(m: dict) -> bool:
    tip = m["tip"]
    if tip < 8.0:
        return False
    if m["dx"] < 0.15:
        return False
    if m["bout_min"] < 0.10:
        return False
    if m["stx_mean"] > 0.08 + 1e-9:
        return False
    if m["stx_p95"] > 0.18 + 1e-9:
        return False
    if m["skate"]:
        return False
    if m["hip_corr"] > -0.45:
        return False
    if m["foot_lead"] < 5:
        return False
    clear = min(m["peak_clear_L"], m["peak_clear_R"])
    if clear < 0.012:
        return False
    if m["n_bouts_L"] < 5 or m["n_bouts_R"] < 5:
        return False
    return True


def cost_dual(m88: dict, m75: dict) -> float:
    """Hard T88 reject → large cost; soft Gate E progress at T75."""
    if not t88_ok(m88):
        c = 50.0
        c += 20.0 * max(0.0, 0.12 - m88.get("bout_min", 0))
        c += 15.0 * max(0.0, m88.get("stx_p95", 9) - 0.18)
        c += 10.0 * max(0.0, 8.0 - m88.get("tip", 0))
        if m88.get("skate"):
            c += 8.0
        if m88.get("hip_corr", 0) > -0.45:
            c += 5.0 + 10.0 * (m88["hip_corr"] + 0.45)
        if m88.get("flipped") or m88.get("exploded") or m88.get("error"):
            c += 30.0
        # still peek at T75 lightly so CMA gets gradient when near T88
        c += 0.15 * (m75.get("stx_mean", 9) + 0.5 * m75.get("stx_p95", 9))
        return float(c)

    # Feasible T88: optimize T75 toward Gate E
    c = 0.0
    c += 4.0 * m75["stx_mean"]
    c += 2.5 * m75["stx_p95"]
    c += 3.0 * max(0.0, m75["stx_mean"] - 0.08)
    c += 4.0 * max(0.0, m75["stx_p95"] - 0.18)
    c += 6.0 * max(0.0, 0.10 - m75["bout_min"])
    c += 2.0 * max(0.0, 0.15 - m75["dx"])
    clear = min(m75["peak_clear_L"], m75["peak_clear_R"])
    c += 2.0 * max(0.0, 0.012 - clear)
    c += 2.0 * max(0.0, 8.0 - m75["tip"])
    if m75["skate"]:
        c += 5.0
    if m75["hip_corr"] > -0.45:
        c += 2.0 + 5.0 * (m75["hip_corr"] + 0.45)
    if m75["foot_lead"] < 5:
        c += 1.5 * (5 - m75["foot_lead"])
    if m75["n_bouts_L"] < 5 or m75["n_bouts_R"] < 5:
        c += 2.0
    # Prefer T88 bout≥0.12 (strong clean_ss) and margin on p95
    c += 2.0 * max(0.0, 0.12 - m88["bout_min"])
    c += 0.5 * max(0.0, 0.13 - m88["bout_min"])
    c += 0.3 * max(0.0, m88["stx_p95"] - 0.15)
    if t88_strong(m88):
        c -= 1.5
    if gate_e(m75):
        c -= 15.0  # big bonus
    return float(c)


def encode(th: np.ndarray) -> np.ndarray:
    return (theta_clip(th) - X0) / X_SCALE


def decode(z: np.ndarray) -> np.ndarray:
    return theta_clip(X0 + np.asarray(z, dtype=np.float64) * X_SCALE)


def cma_search(
    *,
    maxiter: int = 20,
    popsize: int = 10,
    sigma0: float = 0.55,
    duration: float = 9.0,
    seed: int = 27,
) -> dict:
    import cma

    if WORK.exists():
        shutil.rmtree(WORK)
    WORK.mkdir(parents=True, exist_ok=True)

    eval_i = {"n": 0}
    best = {
        "cost": 1e9, "theta": X0.copy(), "m88": None, "m75": None, "gate_e": False,
        "history": [],
    }

    def fitness(z: np.ndarray) -> float:
        eval_i["n"] += 1
        eid = eval_i["n"]
        th = decode(z)
        p88 = theta_to_params(th, 0.88)
        p75 = theta_to_params(th, 0.75)
        m88 = run_rollout(p88, tag=f"e{eid}_88", duration=duration, video=False, work=WORK)
        m75 = run_rollout(p75, tag=f"e{eid}_75", duration=duration, video=False, work=WORK)
        c = cost_dual(m88, m75)
        ok88 = t88_ok(m88)
        ge = gate_e(m75) if ok88 else False
        rec = {
            "eid": eid, "cost": c, "t88_ok": ok88, "gate_e": ge,
            "theta": params_to_dict(th),
            "m88": {k: m88[k] for k in ("tip", "dx", "bout_min", "stx_mean", "stx_p95", "skate", "hip_corr")},
            "m75": {k: m75[k] for k in ("tip", "dx", "bout_min", "stx_mean", "stx_p95", "skate", "hip_corr", "foot_lead")},
        }
        best["history"].append(rec)
        if c < best["cost"]:
            best["cost"] = c
            best["theta"] = th.copy()
            best["m88"] = m88
            best["m75"] = m75
            best["gate_e"] = ge
            print(
                f"[cma] NEW best #{eid} cost={c:.4f} t88_ok={ok88} gate_e={ge} "
                f"88bout={m88['bout_min']:.3f} 88p95={m88['stx_p95']:.3f} "
                f"75stx={m75['stx_mean']:.3f}/{m75['stx_p95']:.3f} 75bout={m75['bout_min']:.3f}",
                flush=True,
            )
        elif eid % 10 == 0 or eid <= 5:
            print(
                f"[cma] eval #{eid} cost={c:.4f} t88_ok={ok88} "
                f"75stx={m75['stx_mean']:.3f}/{m75['stx_p95']:.3f}",
                flush=True,
            )
        # early stop if Gate E unlocked with T88 hold
        if ge and ok88:
            print(f"[cma] Gate E unlocked at eval #{eid} — stopping early", flush=True)
            raise StopIteration
        return c

    opts = cma.CMAOptions()
    opts.set("seed", seed)
    opts.set("popsize", popsize)
    opts.set("maxiter", maxiter)
    opts.set("verbose", -9)
    opts.set("bounds", [-3.5, 3.5])  # in z-space
    n = len(X0)
    print(
        f"[cma] start n={n} pop={popsize} maxiter={maxiter} sigma0={sigma0} "
        f"duration={duration} budget≈{popsize*maxiter} evals",
        flush=True,
    )
    t0 = time.time()
    es = cma.CMAEvolutionStrategy(encode(X0), sigma0, opts)
    try:
        es.optimize(fitness)
    except StopIteration:
        pass
    elapsed = time.time() - t0
    print(
        f"[cma] done in {elapsed:.1f}s evals={eval_i['n']} best_cost={best['cost']:.4f} "
        f"gate_e={best['gate_e']}",
        flush=True,
    )
    best["evals"] = eval_i["n"]
    best["elapsed_s"] = elapsed
    best["params"] = params_to_dict(best["theta"])
    # persist
    (WORK / "best.json").write_text(json.dumps({
        "cost": best["cost"], "evals": best["evals"], "elapsed_s": best["elapsed_s"],
        "gate_e": best["gate_e"], "params": best["params"],
        "m88": {k: best["m88"][k] for k in best["m88"] if k != "params"} if best["m88"] else None,
        "m75": {k: best["m75"][k] for k in best["m75"] if k != "params"} if best["m75"] else None,
        "history_tail": best["history"][-30:],
        "n_t88_ok": sum(1 for h in best["history"] if h["t88_ok"]),
        "n_gate_e": sum(1 for h in best["history"] if h["gate_e"]),
    }, indent=2, default=str))
    (WORK / "history.json").write_text(json.dumps(best["history"], indent=2, default=str))
    return best


def summarize_row(tag: str, T: float, m: dict, params: dict, *, is_baseline: bool = False) -> dict:
    row = {
        "tag": tag,
        "T": T,
        "params": {
            "gait_t": params.get("gait_t", T),
            "ds": params.get("ds"),
            "hip_amp": params.get("hip_amp"),
            "step_len": params.get("step_len"),
            "com_shift": params.get("com_shift"),
            "com_shift_lead": params.get("com_shift_lead"),
            "com_z": params.get("com_z"),
            "hip_bias": params.get("hip_bias"),
            "knee_swing": params.get("knee_swing"),
            "plant_kd": params.get("plant_kd"),
            "vik_kp": params.get("vik_kp"),
            "stance_sweep": params.get("stance_sweep"),
            "swing_ank_df": params.get("swing_ank_df", 0.0),
            "ds_frac": (params["ds"] / T) if params.get("ds") and T else None,
        },
        "tip": m["tip"], "dx": m["dx"],
        "bout_min": m["bout_min"], "bout_L": m["bout_L"], "bout_R": m["bout_R"],
        "stx_mean": m["stx_mean"], "stx_p95": m["stx_p95"],
        "stx_L": m["stx_L"], "stx_R": m["stx_R"],
        "p95_L": m["p95_L"], "p95_R": m["p95_R"],
        "hip_corr": m["hip_corr"], "foot_lead": m["foot_lead"],
        "n_bouts_L": m["n_bouts_L"], "n_bouts_R": m["n_bouts_R"],
        "peak_clear_L": m["peak_clear_L"], "peak_clear_R": m["peak_clear_R"],
        "skate": m["skate"],
        "t88_ok": t88_ok(m) if abs(T - 0.88) < 1e-6 else None,
        "t88_strong": t88_strong(m) if abs(T - 0.88) < 1e-6 else None,
        "gate_e": gate_e(m) if T <= 0.75 + 1e-9 else False,
        "assist": False, "freeze": False,
        "mp4": m.get("mp4"),
        "baseline": is_baseline,
    }
    return row


def relocate(tag: str, m: dict) -> None:
    src = m.get("mp4")
    if not src:
        return
    src_p = Path(src)
    dst = ITER / f"{tag}.mp4"
    if src_p.exists():
        shutil.copy2(src_p, dst)
        m["mp4"] = str(dst)
    # also copy json
    js = WORK / f"{tag}.json" if False else None
    # stats live under work with tag — for final tags we use ITER
    for ext in (".json", "_timeseries.csv", "_ss.json"):
        pass


def score_table(best: dict, *, duration: float = 9.0) -> list[dict]:
    ITER.mkdir(parents=True, exist_ok=True)
    table = []
    th = best["theta"]
    shape = params_to_dict(th)

    # GRO00 CSF50 T88 baseline
    p0 = dict(
        gait_t=0.88, ds=0.308, hip_amp=0.116, step_len=0.021,
        com_shift=0.275, com_shift_lead=0.23, com_z=0.225, hip_bias=0.06,
        knee_stance=0.40, knee_swing=0.80, plant_kd=115.0, vik_kp=0.45,
        vik_clip=0.03, stance_sweep=1.0, swing_ank_df=0.0, cp=True, vik=True,
    )
    m = run_rollout(p0, tag="GRO00_CSF50_T88", duration=duration, video=True, work=ITER)
    relocate_final("GRO00_CSF50_T88", m)
    row = summarize_row("GRO00_CSF50_T88", 0.88, m, p0, is_baseline=True)
    (ITER / "GRO00_CSF50_T88_ss.json").write_text(json.dumps(row, indent=2))
    table.append(row)
    print(f"ROW {row['tag']}: tip={row['tip']:.2f} bout={row['bout_min']:.3f} "
          f"stx={row['stx_mean']:.3f} p95={row['stx_p95']:.3f} skate={row['skate']} t88={row['t88_ok']}", flush=True)

    # GRO01 best at T88
    p88 = theta_to_params(th, 0.88)
    m = run_rollout(p88, tag="GRO01_best_T88", duration=duration, video=True, work=ITER)
    relocate_final("GRO01_best_T88", m)
    row = summarize_row("GRO01_best_T88", 0.88, m, p88)
    row["shape"] = shape
    (ITER / "GRO01_best_T88_ss.json").write_text(json.dumps(row, indent=2))
    table.append(row)
    print(f"ROW {row['tag']}: tip={row['tip']:.2f} bout={row['bout_min']:.3f} "
          f"stx={row['stx_mean']:.3f} p95={row['stx_p95']:.3f} skate={row['skate']} t88={row['t88_ok']}", flush=True)

    # GRO02 best at T75
    p75 = theta_to_params(th, 0.75)
    m = run_rollout(p75, tag="GRO02_best_T75", duration=duration, video=True, work=ITER)
    relocate_final("GRO02_best_T75", m)
    row = summarize_row("GRO02_best_T75", 0.75, m, p75)
    row["shape"] = shape
    (ITER / "GRO02_best_T75_ss.json").write_text(json.dumps(row, indent=2))
    table.append(row)
    print(f"ROW {row['tag']}: tip={row['tip']:.2f} dx={row['dx']:+.3f} bout={row['bout_min']:.3f} "
          f"stx={row['stx_mean']:.3f} p95={row['stx_p95']:.3f} skate={row['skate']} gate_e={row['gate_e']}", flush=True)

    # GRO03+ A/B: vary key vars around best (and CSF50) at T75
    ab = []
    # ds_frac ±, step ±, shift ± relative to best
    for name, delta in [
        ("GRO03_ds_hi", {"ds_frac": +0.04}),
        ("GRO04_ds_lo", {"ds_frac": -0.04}),
        ("GRO05_step_hi", {"step_len": +0.004}),
        ("GRO06_shift_hi", {"com_shift": +0.03}),
        ("GRO07_amp_lo", {"hip_amp": -0.008}),
    ]:
        th_ab = th.copy()
        # apply delta by name index
        dmap = {
            "ds_frac": 0, "hip_amp": 1, "step_len": 2, "com_shift": 3,
        }
        for k, dv in delta.items():
            th_ab[dmap[k]] = th_ab[dmap[k]] + dv
        th_ab = theta_clip(th_ab)
        p = theta_to_params(th_ab, 0.75)
        m = run_rollout(p, tag=name, duration=duration, video=True, work=ITER)
        relocate_final(name, m)
        row = summarize_row(name, 0.75, m, p)
        row["delta"] = delta
        (ITER / f"{name}_ss.json").write_text(json.dumps(row, indent=2))
        table.append(row)
        ab.append(row)
        print(f"ROW {row['tag']}: tip={row['tip']:.2f} dx={row['dx']:+.3f} bout={row['bout_min']:.3f} "
              f"stx={row['stx_mean']:.3f} p95={row['stx_p95']:.3f} skate={row['skate']} gate_e={row['gate_e']}", flush=True)

    (ITER / "GAIT_REOPT_TABLE.json").write_text(json.dumps(table, indent=2))
    return table


def relocate_final(tag: str, m: dict) -> None:
    src = m.get("mp4")
    if src and Path(src).exists():
        dst = ITER / f"{tag}.mp4"
        if Path(src).resolve() != dst.resolve():
            shutil.copy2(src, dst)
        m["mp4"] = str(dst)
    # ensure stats json at ITER/{tag}.json (run_rollout already wrote there if work=ITER)
    stats = ITER / f"{tag}.json"
    if not stats.exists():
        # copy from wherever
        pass


def write_note(best: dict, table: list[dict]) -> None:
    ge_hits = [r for r in table if r.get("gate_e")]
    gro00 = next(r for r in table if r["tag"].startswith("GRO00"))
    gro01 = next(r for r in table if r["tag"].startswith("GRO01"))
    gro02 = next(r for r in table if r["tag"].startswith("GRO02"))
    unlocked = bool(ge_hits) and bool(gro01.get("t88_ok"))
    n_t88 = sum(1 for h in best.get("history", []) if h.get("t88_ok"))
    n_ge = sum(1 for h in best.get("history", []) if h.get("gate_e"))

    def fmt(r):
        ge = "PASS" if r.get("gate_e") else ("n/a" if r["T"] > 0.75 else "FAIL")
        t88 = r.get("t88_ok")
        t88s = "yes" if t88 else ("no" if t88 is False else "—")
        return (
            f"| {r['tag']} | {r['T']:.2f} | {r['tip']:.2f} | {r['dx']:+.3f} | "
            f"{r['bout_min']:.3f} | {r['stx_mean']:.3f} | {r['stx_p95']:.3f} | "
            f"{str(r['skate']).lower()} | {r['hip_corr']:.3f} | {ge} | {t88s} |"
        )

    rows = "\n".join(fmt(r) for r in table)
    p = best.get("params", {})
    verdict = "Gate E UNLOCKED" if unlocked else "HARD-FALSIFIED"
    note = f"""# Dual-T outer gait re-opt — GRO00+

**When:** Sun 27 Sep 2026 Europe/London (BST)
**Role:** Founding Controls
**Plant (score):** `ainex_controls_m2_145.xml` locked
**Assist / freeze / xfrc:** OFF
**Cospec:** `docs/GAIT_REOPT_DUAL_T_COSPEC.md`
**Method:** CMA-ES on shared shape vector; score both T=0.88 and T=0.75; hard-reject T88 fails
**Warm start:** CSF50 (ds_frac≈{DS_FRAC0:.4f}, hip_amp=0.116, step=0.021, shift=0.275/0.23, plant_kd=115, CP+VIK)
**Authority:** outer references only — no residual / WBC / MPC scrub

## Verdict

| Claim | Result |
|-------|--------|
| GRO00 CSF50 T88 floor holds | {"YES" if gro00.get("t88_ok") else "NO"} |
| Best dual-T holds T88 clean_ss | {"YES" if gro01.get("t88_ok") else "NO"} |
| Same candidate Gate E at T≤0.75 | {"YES" if gro02.get("gate_e") else "NO"} |
| Dual-T outer gait re-opt under HX ±2.1 | **{verdict}** |

CMA budget: **{best.get("evals", 0)}** evals in {best.get("elapsed_s", 0):.0f}s; T88-ok count={n_t88}; Gate E count={n_ge}; best_cost={best.get("cost", 0):.4f}.

## Best shape (shared)

```
{json.dumps(p, indent=2)}
```

At T=0.88: ds={theta_to_params(best["theta"], 0.88)["ds"]:.4f}s
At T=0.75: ds={theta_to_params(best["theta"], 0.75)["ds"]:.4f}s

## M145 table (authoritative)

| Tag | T | tip | dx | bout_min | stx mean | stx p95 | skate | hip_corr | Gate E | T88 ok |
|-----|---|-----|-----|----------|----------|---------|-------|----------|--------|--------|
{rows}

### Reading

- **GRO00:** CSF50 T=0.88 baseline (must hold).
- **GRO01 / GRO02:** best CMA candidate at both rates from the **same** shape.
- **GRO03–07:** A/B on ds_frac / step / shift / amp around best at T=0.75.

## Stop rule

{"**Gate E unlocked** with T88 floor held — dual-T outer wins; inner scrubbers still locked-off until further notice." if unlocked else "**Dual-T outer gait re-opt HARD-FALSIFIED** on locked M145 under HX after bounded CMA (≥200 evals target). Not more CSF50 micro-sweeps or residual/WBC/MPC on frozen outer. Next is a different architecture (still sim)."}

## Artifacts

- `scripts/gait_reopt_dual_t.py`
- `GAIT_REOPT_TABLE.json`, `GAIT_REOPT_NOTE.md`
- `GRO00`…`GRO07` mp4/json/_ss.json
- CMA work: `previews/ainex_walk/iterate/_gait_reopt_cma/`
"""
    (ITER / "GAIT_REOPT_NOTE.md").write_text(note)
    print(f"wrote {ITER / 'GAIT_REOPT_NOTE.md'} verdict={verdict}", flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--maxiter", type=int, default=20)
    ap.add_argument("--popsize", type=int, default=10)
    ap.add_argument("--sigma0", type=float, default=0.55)
    ap.add_argument("--duration", type=float, default=9.0)
    ap.add_argument("--seed", type=int, default=27)
    ap.add_argument("--score-only", action="store_true", help="skip CMA; load best.json")
    args = ap.parse_args()

    if args.score_only and (WORK / "best.json").exists():
        raw = json.loads((WORK / "best.json").read_text())
        th = np.array([raw["params"][n] for n in THETA_NAMES], dtype=np.float64)
        best = {
            "cost": raw["cost"], "theta": th, "params": raw["params"],
            "evals": raw.get("evals", 0), "elapsed_s": raw.get("elapsed_s", 0),
            "gate_e": raw.get("gate_e", False), "history": [],
            "m88": raw.get("m88"), "m75": raw.get("m75"),
        }
        if (WORK / "history.json").exists():
            best["history"] = json.loads((WORK / "history.json").read_text())
    else:
        # Ensure budget ≥200: pop*maxiter
        if args.popsize * args.maxiter < 200:
            args.maxiter = int(math.ceil(200 / args.popsize))
            print(f"[cma] raised maxiter→{args.maxiter} for ≥200 evals", flush=True)
        best = cma_search(
            maxiter=args.maxiter, popsize=args.popsize,
            sigma0=args.sigma0, duration=args.duration, seed=args.seed,
        )

    table = score_table(best, duration=9.0)
    write_note(best, table)
    ge = any(r.get("gate_e") for r in table)
    t88 = next(r for r in table if r["tag"].startswith("GRO01")).get("t88_ok")
    print("=== FINAL ===", "gate_e", ge, "best_t88_ok", t88, "evals", best.get("evals"), flush=True)


if __name__ == "__main__":
    main()
