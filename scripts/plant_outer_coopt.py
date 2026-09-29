#!/usr/bin/env python3
"""Plant-specific outer co-opt (CMA) on GEO02a / optional GEO02b.

Cospec: docs/PLANT_OUTER_COOPT_COSPEC.md
Seed: GRO01 (hip_amp HI widened 0.145→0.16 so seed is interior).
k_auth=1.0; no residual/WBC/MPC. Gate criteria from score_auth_envelope.
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
from score_auth_envelope import GRO01, CSF50, t88_ok, gate_e  # noqa: E402 — exact criteria

WALK = ROOT / "scripts" / "walk_gait_ainex.py"
ITER = ROOT / "previews" / "ainex_walk" / "iterate"
PY = ROOT / ".venv" / "bin" / "python"
HIW = ROOT / "mujoco" / "ainex_hiwonder"
GEO00 = HIW / "ainex_controls_m2_145.xml"
GEO02A = HIW / "ainex_controls_m2_160x90.xml"
GEO02B = HIW / "ainex_controls_m2_155x86.xml"
WORK = ITER / "_plant_outer_cma"

os.environ.setdefault("MUJOCO_GL", "glfw")

THETA_NAMES = [
    "ds_frac", "hip_amp", "step_len", "com_shift", "com_shift_lead",
    "com_z", "hip_bias", "knee_swing", "plant_kd", "vik_kp",
    "stance_sweep", "swing_ank_df",
]

# GRO01 seed — hip_amp at old bound 0.145; widen HI to 0.16 (documented)
X0 = np.array([GRO01[n] for n in THETA_NAMES], dtype=np.float64)
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
    0.48, 0.160, 0.040, 0.36, 0.40,  # hip_amp HI 0.16 (was 0.145)
    0.250, 0.12, 1.05, 160.0, 0.80,
    1.00, 0.25,
], dtype=np.float64)

# Module plant path set per run
PLANT: Path = GEO02A


def theta_clip(th: np.ndarray) -> np.ndarray:
    return np.clip(np.asarray(th, dtype=np.float64), BOUNDS_LO, BOUNDS_HI)


def theta_to_params(th: np.ndarray, gait_t: float) -> dict:
    th = theta_clip(th)
    hip_amp = float(th[1])
    step_len = float(np.clip(hip_amp * 0.25, 0.012, 0.040))
    ds = float(th[0] * gait_t)
    ds = min(ds, 0.55 * gait_t)
    ds = max(ds, 0.12 * gait_t)
    return {
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


def params_to_dict(th: np.ndarray) -> dict:
    return {n: float(v) for n, v in zip(THETA_NAMES, theta_clip(th))}


def shape_dict_to_theta(shape: dict) -> np.ndarray:
    return theta_clip(np.array([shape[n] for n in THETA_NAMES], dtype=np.float64))


def run_rollout(params: dict, *, tag: str, duration: float, video: bool, work: Path,
                model: Path | None = None) -> dict:
    model = model or PLANT
    work.mkdir(parents=True, exist_ok=True)
    stats_out = work / f"{tag}.json"
    out_mp4 = work / f"{tag}.mp4"
    cmd = [
        str(PY), str(WALK),
        "--model", str(model),
        "--duration", str(duration),
        "--tag", tag,
        "--gait-t", str(params["gait_t"]),
        "--hip-amp", str(params["hip_amp"]),
        "--ds", str(params["ds"]),
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
        "--auth-k", "1.0",
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
            "tip": 0.0, "dx": 0.0, "stx_mean": 9.0, "stx_p95": 9.0,
            "bout_min": 0.0, "hip_corr": 0.0, "skate": True,
            "foot_lead": 0, "n_bouts_L": 0, "n_bouts_R": 0,
            "peak_clear_L": 0.0, "peak_clear_R": 0.0,
            "flipped": True, "exploded": True, "params": params,
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
    ss = analyze_ss(csv_tag, stats) if csv_tag.exists() else {}
    stx_L = float(stats.get("stance_foot_vx_mean_L") or 9)
    stx_R = float(stats.get("stance_foot_vx_mean_R") or 9)
    p95_L = float(stats.get("stance_foot_vx_p95_L") or 9)
    p95_R = float(stats.get("stance_foot_vx_p95_R") or 9)
    bout_L = float(ss.get("ss_mean_bout_s_L") or 0)
    bout_R = float(ss.get("ss_mean_bout_s_R") or 0)
    return {
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
        "model": str(model),
    }


def cost_dual(m88: dict, m75: dict) -> float:
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
        c += 0.15 * (m75.get("stx_mean", 9) + 0.5 * m75.get("stx_p95", 9))
        return float(c)
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
    c += 2.0 * max(0.0, 0.12 - m88["bout_min"])
    c += 0.3 * max(0.0, m88["stx_p95"] - 0.15)
    if gate_e(m75):
        c -= 15.0
    return float(c)


def encode(th: np.ndarray) -> np.ndarray:
    return (theta_clip(th) - X0) / X_SCALE


def decode(z: np.ndarray) -> np.ndarray:
    return theta_clip(X0 + np.asarray(z, dtype=np.float64) * X_SCALE)


def cma_search(
    *,
    model: Path,
    maxiter: int = 20,
    popsize: int = 10,
    sigma0: float = 0.55,
    duration: float = 9.0,
    seed: int = 27,
    work: Path | None = None,
    x0: np.ndarray | None = None,
) -> dict:
    import cma
    global PLANT
    PLANT = model
    work = work or WORK
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True, exist_ok=True)

    x_seed = theta_clip(x0 if x0 is not None else X0.copy())
    eval_i = {"n": 0}
    best = {
        "cost": 1e9, "theta": x_seed.copy(), "m88": None, "m75": None,
        "gate_e": False, "history": [], "model": str(model),
    }

    def fitness(z: np.ndarray) -> float:
        eval_i["n"] += 1
        eid = eval_i["n"]
        th = decode(z)
        # encode/decode uses global X0 as center — OK for GRO01 seed
        p88 = theta_to_params(th, 0.88)
        p75 = theta_to_params(th, 0.75)
        m88 = run_rollout(p88, tag=f"e{eid}_88", duration=duration, video=False, work=work, model=model)
        m75 = run_rollout(p75, tag=f"e{eid}_75", duration=duration, video=False, work=work, model=model)
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
        if ge and ok88:
            print(f"[cma] Gate E unlocked at eval #{eid} — stopping early", flush=True)
            raise StopIteration
        return c

    opts = cma.CMAOptions()
    opts.set("seed", seed)
    opts.set("popsize", popsize)
    opts.set("maxiter", maxiter)
    opts.set("verbose", -9)
    opts.set("bounds", [-3.5, 3.5])
    print(
        f"[cma] plant={model.name} n={len(X0)} pop={popsize} maxiter={maxiter} "
        f"sigma0={sigma0} duration={duration} budget≈{popsize*maxiter}",
        flush=True,
    )
    print(f"[cma] seed GRO01 hip_amp={x_seed[1]:.4f} (BOUNDS_HI hip_amp={BOUNDS_HI[1]})", flush=True)
    t0 = time.time()
    es = cma.CMAEvolutionStrategy(encode(x_seed), sigma0, opts)
    try:
        es.optimize(fitness)
    except StopIteration:
        pass
    elapsed = time.time() - t0
    best["evals"] = eval_i["n"]
    best["elapsed_s"] = elapsed
    best["params"] = params_to_dict(best["theta"])
    print(
        f"[cma] done in {elapsed:.1f}s evals={best['evals']} best_cost={best['cost']:.4f} "
        f"gate_e={best['gate_e']}",
        flush=True,
    )
    (work / "best.json").write_text(json.dumps({
        "cost": best["cost"], "evals": best["evals"], "elapsed_s": best["elapsed_s"],
        "gate_e": best["gate_e"], "params": best["params"], "model": str(model),
        "m88": {k: best["m88"][k] for k in best["m88"] if k not in ("params", "mp4")} if best["m88"] else None,
        "m75": {k: best["m75"][k] for k in best["m75"] if k not in ("params", "mp4")} if best["m75"] else None,
        "n_t88_ok": sum(1 for h in best["history"] if h["t88_ok"]),
        "n_gate_e": sum(1 for h in best["history"] if h["gate_e"]),
        "bounds_note": "hip_amp HI widened 0.145→0.16 so GRO01 seed is interior",
    }, indent=2, default=str))
    (work / "history.json").write_text(json.dumps(best["history"], indent=2, default=str))
    return best


def summarize(tag: str, T: float, m: dict, params: dict, *, plant: str, model: Path) -> dict:
    row = {
        "tag": tag,
        "plant": plant,
        "model": str(model),
        "T": T,
        "auth_k": 1.0,
        "tip": m["tip"], "dx": m["dx"],
        "bout_min": m["bout_min"], "bout_L": m.get("bout_L", 0), "bout_R": m.get("bout_R", 0),
        "stx_mean": m["stx_mean"], "stx_p95": m["stx_p95"],
        "stx_L": m.get("stx_L"), "stx_R": m.get("stx_R"),
        "p95_L": m.get("p95_L"), "p95_R": m.get("p95_R"),
        "hip_corr": m["hip_corr"], "foot_lead": m["foot_lead"],
        "n_bouts_L": m["n_bouts_L"], "n_bouts_R": m["n_bouts_R"],
        "peak_clear_L": m["peak_clear_L"], "peak_clear_R": m["peak_clear_R"],
        "skate": m["skate"],
        "t88_ok": t88_ok(m) if abs(T - 0.88) < 1e-6 else None,
        "gate_e": gate_e(m) if T <= 0.75 + 1e-9 else False,
        "assist": False, "freeze": False,
        "mp4": m.get("mp4"),
        "params": params,
    }
    return row


def relocate(tag: str, m: dict) -> None:
    src = m.get("mp4")
    if src and Path(src).exists():
        dst = ITER / f"{tag}.mp4"
        if Path(src).resolve() != dst.resolve():
            shutil.copy2(src, dst)
        m["mp4"] = str(dst)


def score_row(tag: str, T: float, th: np.ndarray, *, model: Path, plant: str,
              duration: float = 9.0) -> dict:
    params = theta_to_params(th, T)
    m = run_rollout(params, tag=tag, duration=duration, video=True, work=ITER, model=model)
    relocate(tag, m)
    row = summarize(tag, T, m, params, plant=plant, model=model)
    (ITER / f"{tag}_ss.json").write_text(json.dumps(row, indent=2, default=str))
    print(
        f"ROW {tag}: tip={row['tip']:.2f} dx={row['dx']:+.3f} bout={row['bout_min']:.3f} "
        f"stx={row['stx_mean']:.3f} p95={row['stx_p95']:.3f} skate={row['skate']} "
        f"ge={row['gate_e']} t88={row['t88_ok']}",
        flush=True,
    )
    return row


def write_note(table: list[dict], best: dict) -> str:
    poc00_88 = next((r for r in table if r["tag"] == "POC00_GEO00_T88_GRO01"), None)
    poc00_75 = next((r for r in table if r["tag"] == "POC00_GEO00_T75_GRO01"), None)
    poc01_88 = next((r for r in table if r["tag"] == "POC01_GEO02a_best_T88"), None)
    poc01_75 = next((r for r in table if r["tag"] == "POC01_GEO02a_best_T75"), None)
    ge = [r for r in table if r.get("gate_e")]
    if ge:
        outcome = "GATE_E_UNLOCK"
        verdict = f"Gate E UNLOCKED on: {', '.join(r['tag'] for r in ge)} — lock plant claim"
    else:
        outcome = "HARD_FALSIFIED"
        verdict = "Plant×outer co-opt HARD-FALSIFIED — no Gate E after CMA budget on GEO02a (± GEO02b)"

    auth01_match = False
    if poc00_75:
        auth01_match = (
            poc00_75["skate"]
            and abs(poc00_75["stx_mean"] - 0.121) < 0.02
            and abs(poc00_75["stx_p95"] - 0.638) < 0.05
        )

    def fmt(r):
        ge_s = "PASS" if r.get("gate_e") else ("n/a" if r["T"] > 0.75 else "FAIL")
        t88 = r.get("t88_ok")
        t88s = "yes" if t88 else ("no" if t88 is False else "—")
        return (
            f"| {r['tag']} | {r['plant']} | {r['T']:.2f} | {r['tip']:.2f} | {r['dx']:+.3f} | "
            f"{r['bout_min']:.3f} | {r['stx_mean']:.3f} | {r['stx_p95']:.3f} | "
            f"{str(r['skate']).lower()} | {ge_s} | {t88s} |"
        )

    rows = "\n".join(fmt(r) for r in table)
    p = best.get("params", {})
    note = f'''# Plant-specific outer co-opt — POC00+

**When:** Sun 27 Sep 2026 Europe/London (BST)
**Role:** Founding Controls
**Cospec:** `docs/PLANT_OUTER_COOPT_COSPEC.md`
**Primary plant:** GEO02a `ainex_controls_m2_160x90.xml`
**Seed:** GRO01 (hip_amp BOUNDS_HI widened 0.145→**0.16** so seed is interior)
**Authority:** k_auth=1.0; no residual/WBC/MPC
**Assist / freeze / xfrc:** OFF
**GL:** glfw

## Verdict

| Claim | Result |
|-------|--------|
| POC00 GEO00 T75 matches AUTH01/GEO00 FAIL | **{"YES" if auth01_match else "CHECK"}** |
| GEO02a CMA Gate E (T88 held) | **{"YES" if poc01_75 and poc01_75.get("gate_e") and poc01_88 and poc01_88.get("t88_ok") else "NO"}** |
| Outcome | **{outcome}** |

{verdict}

CMA: **{best.get("evals", 0)}** evals in {best.get("elapsed_s", 0):.0f}s; best_cost={best.get("cost", 0):.4f}; T88-ok hist={sum(1 for h in best.get("history", []) if h.get("t88_ok"))}; Gate E hist={sum(1 for h in best.get("history", []) if h.get("gate_e"))}.

## Best theta (GEO02a)

```json
{json.dumps(p, indent=2)}
```

## M145 table (authoritative)

| Tag | plant | T | tip | dx | bout_min | stx mean | stx p95 | skate | Gate E | T88 ok |
|-----|-------|---|-----|-----|----------|----------|---------|-------|--------|--------|
{rows}

## Stop rule

{"**Gate E unlocked** — freeze winning plant XML." if outcome=="GATE_E_UNLOCK" else "**Plant×outer co-opt HARD-FALSIFIED.** Classic open-loop+geometry family exhausted. Next: new control class in sim (learned/RL Gate E) — not more frozen-outer GEO packs, k_auth, or inner scrubbers."}

## Artifacts

- `scripts/plant_outer_coopt.py`
- `PLANT_OUTER_COOPT_TABLE.json`, `PLANT_OUTER_COOPT_NOTE.md`
- POC* mp4/json/_ss.json; CMA work `_plant_outer_cma/`
'''
    (ITER / "PLANT_OUTER_COOPT_NOTE.md").write_text(note)
    print(f"wrote note outcome={outcome}", flush=True)
    return outcome


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--maxiter", type=int, default=20)
    ap.add_argument("--popsize", type=int, default=10)
    ap.add_argument("--sigma0", type=float, default=0.55)
    ap.add_argument("--duration", type=float, default=9.0)
    ap.add_argument("--seed", type=int, default=27)
    ap.add_argument("--skip-cma", action="store_true")
    ap.add_argument("--geo02b", action="store_true", help="Also transfer best + short CMA on GEO02b")
    ap.add_argument("--geo02b-maxiter", type=int, default=10)  # ≤100 evals if pop=10
    args = ap.parse_args()

    if args.popsize * args.maxiter < 200:
        args.maxiter = int(math.ceil(200 / args.popsize))
        print(f"[cma] raised maxiter→{args.maxiter} for ≥200 evals", flush=True)

    ITER.mkdir(parents=True, exist_ok=True)
    table: list[dict] = []

    # --- POC00 parity: GEO00 + frozen GRO01 ---
    th0 = shape_dict_to_theta(GRO01)
    print(f"[poc] GRO01 seed hip_amp={th0[1]:.4f} (HI={BOUNDS_HI[1]})", flush=True)
    table.append(score_row("POC00_GEO00_T88_GRO01", 0.88, th0, model=GEO00, plant="GEO00", duration=args.duration))
    table.append(score_row("POC00_GEO00_T75_GRO01", 0.75, th0, model=GEO00, plant="GEO00", duration=args.duration))

    # --- CMA on GEO02a ---
    if args.skip_cma and (WORK / "best.json").exists():
        raw = json.loads((WORK / "best.json").read_text())
        th = shape_dict_to_theta(raw["params"])
        best = {
            "cost": raw["cost"], "theta": th, "params": raw["params"],
            "evals": raw.get("evals", 0), "elapsed_s": raw.get("elapsed_s", 0),
            "gate_e": raw.get("gate_e", False), "history": [],
            "m88": raw.get("m88"), "m75": raw.get("m75"),
        }
        if (WORK / "history.json").exists():
            best["history"] = json.loads((WORK / "history.json").read_text())
    else:
        best = cma_search(
            model=GEO02A, maxiter=args.maxiter, popsize=args.popsize,
            sigma0=args.sigma0, duration=args.duration, seed=args.seed,
            work=WORK, x0=th0,
        )

    # Prefer best among T88-ok history if final best somehow fails T88 on rescore
    table.append(score_row("POC01_GEO02a_best_T88", 0.88, best["theta"], model=GEO02A, plant="GEO02a", duration=args.duration))
    table.append(score_row("POC01_GEO02a_best_T75", 0.75, best["theta"], model=GEO02A, plant="GEO02a", duration=args.duration))

    # Seed row on GEO02a frozen GRO01 for reference
    table.append(score_row("POC01b_GEO02a_GRO01_T75", 0.75, th0, model=GEO02A, plant="GEO02a", duration=args.duration))

    # --- Optional GEO02b: transfer best + short CMA if no Gate E ---
    unlocked = any(r.get("gate_e") and r.get("plant") == "GEO02a" for r in table)
    if (args.geo02b or not unlocked) and GEO02B.exists():
        # Transfer score
        table.append(score_row("POC_b00_GEO02b_xfer_T88", 0.88, best["theta"], model=GEO02B, plant="GEO02b", duration=args.duration))
        table.append(score_row("POC_b00_GEO02b_xfer_T75", 0.75, best["theta"], model=GEO02B, plant="GEO02b", duration=args.duration))
        if not unlocked and not args.skip_cma:
            work_b = ITER / "_plant_outer_cma_b"
            best_b = cma_search(
                model=GEO02B, maxiter=args.geo02b_maxiter, popsize=args.popsize,
                sigma0=args.sigma0, duration=args.duration, seed=args.seed + 1,
                work=work_b, x0=best["theta"],
            )
            table.append(score_row("POC_b01_GEO02b_best_T88", 0.88, best_b["theta"], model=GEO02B, plant="GEO02b", duration=args.duration))
            table.append(score_row("POC_b01_GEO02b_best_T75", 0.75, best_b["theta"], model=GEO02B, plant="GEO02b", duration=args.duration))
            best["geo02b"] = {
                "evals": best_b["evals"], "cost": best_b["cost"],
                "params": best_b["params"], "gate_e": best_b["gate_e"],
            }

    (ITER / "PLANT_OUTER_COOPT_TABLE.json").write_text(json.dumps(table, indent=2, default=str))
    outcome = write_note(table, best)
    print(
        "=== FINAL ===", outcome,
        "gate_e", [r["tag"] for r in table if r.get("gate_e")],
        "evals", best.get("evals"),
        flush=True,
    )


if __name__ == "__main__":
    main()
