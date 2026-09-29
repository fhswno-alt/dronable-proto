#!/usr/bin/env python3
"""HX authority envelope A/B — scale kit ±2.1/±2.09 by k_auth on locked M145.

Cospec: docs/AUTH_ENVELOPE_HX_COSPEC.md
Outer: GRO01 best (primary) + CSF50 control. No residual/WBC/MPC.
"""
from __future__ import annotations

import json
import os
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from ss_step_ainex import analyze_ss  # noqa: E402

WALK = ROOT / "scripts" / "walk_gait_ainex.py"
PLANT = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
ITER = ROOT / "previews" / "ainex_walk" / "iterate"
PY = ROOT / ".venv" / "bin" / "python"
os.environ.setdefault("MUJOCO_GL", "glfw")

GRO01 = {
    "ds_frac": 0.3245536345821725,
    "hip_amp": 0.145,
    "step_len": 0.029239725926324792,
    "com_shift": 0.31133025663712655,
    "com_shift_lead": 0.21792882001887864,
    "com_z": 0.24144458843778405,
    "hip_bias": 0.0864943788801559,
    "knee_swing": 0.7869671613937524,
    "plant_kd": 105.97541086071345,
    "vik_kp": 0.5240709042821181,
    "stance_sweep": 0.743996716733196,
    "swing_ank_df": 0.015401656420420469,
}

CSF50 = {
    "ds_frac": 0.35,
    "hip_amp": 0.116,
    "step_len": 0.021,
    "com_shift": 0.275,
    "com_shift_lead": 0.23,
    "com_z": 0.225,
    "hip_bias": 0.06,
    "knee_swing": 0.80,
    "plant_kd": 115.0,
    "vik_kp": 0.45,
    "stance_sweep": 1.0,
    "swing_ank_df": 0.0,
}


def shape_to_params(shape: dict, gait_t: float) -> dict:
    ds = float(shape["ds_frac"]) * float(gait_t)
    ds = min(ds, 0.55 * gait_t)
    return {
        "gait_t": float(gait_t),
        "ds": ds,
        "hip_amp": float(shape["hip_amp"]),
        "com_shift": float(shape["com_shift"]),
        "com_shift_lead": float(shape["com_shift_lead"]),
        "com_z": float(shape["com_z"]),
        "hip_bias": float(shape["hip_bias"]),
        "knee_stance": 0.40,
        "knee_swing": float(shape["knee_swing"]),
        "plant_kd": float(shape["plant_kd"]),
        "vik_kp": float(shape["vik_kp"]),
        "vik_clip": 0.03,
        "stance_sweep": float(shape["stance_sweep"]),
        "swing_ank_df": float(shape.get("swing_ank_df") or 0.0),
    }


def run_one(params: dict, *, tag: str, auth_k: float, fric: float | None,
            duration: float, video: bool) -> dict:
    import subprocess
    stats_out = ITER / f"{tag}.json"
    out_mp4 = ITER / f"{tag}.mp4"
    cmd = [
        str(PY), str(WALK),
        "--model", str(PLANT),
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
        "--auth-k", str(auth_k),
    ]
    if params.get("swing_ank_df", 0) and float(params["swing_ank_df"]) > 1e-6:
        cmd += ["--swing-ank-df", str(params["swing_ank_df"])]
    if fric is not None:
        cmd += ["--fric", str(fric)]
    if not video:
        cmd.append("--no-video")
    env = dict(os.environ)
    env["MUJOCO_GL"] = os.environ.get("MUJOCO_GL", "glfw")
    print(f"\n=== RUN {tag} auth_k={auth_k} fric={fric} video={video} ===", flush=True)
    r = subprocess.run(cmd, cwd=str(ROOT), env=env, capture_output=True, text=True)
    if r.returncode != 0:
        print((r.stdout or "")[-800:], flush=True)
        print((r.stderr or "")[-400:], flush=True)
    if not stats_out.exists():
        return {"tag": tag, "error": f"rc={r.returncode}", "skate": True, "tip": 0,
                "dx": 0, "stx_mean": 9, "stx_p95": 9, "bout_min": 0, "gate_e": False,
                "t88_ok": False, "auth_k": auth_k}
    stats = json.loads(stats_out.read_text())
    csv = ITER / f"{tag}_timeseries.csv"
    candidates = [
        Path(str(stats_out).replace(".json", "_timeseries.csv")),
        ITER / "ainex_walk_timeseries.csv",
        ROOT / "previews" / "ainex_walk" / "ainex_walk_timeseries.csv",
    ]
    existing = [c for c in candidates if c.exists()]
    if existing:
        src = max(existing, key=lambda p: p.stat().st_mtime)
        if src.resolve() != csv.resolve():
            shutil.copy2(src, csv)
    ss = analyze_ss(csv, stats) if csv.exists() else {}
    stx_L = float(stats.get("stance_foot_vx_mean_L") or 9)
    stx_R = float(stats.get("stance_foot_vx_mean_R") or 9)
    p95_L = float(stats.get("stance_foot_vx_p95_L") or 9)
    p95_R = float(stats.get("stance_foot_vx_p95_R") or 9)
    bout_L = float(ss.get("ss_mean_bout_s_L") or 0)
    bout_R = float(ss.get("ss_mean_bout_s_R") or 0)
    m = {
        "tag": tag,
        "T": float(params["gait_t"]),
        "shape": "GRO01" if abs(params["hip_amp"] - 0.145) < 1e-6 else "CSF50",
        "auth_k": float(stats.get("auth_k", auth_k)),
        "auth_leg_tau": float(stats.get("auth_leg_tau", 2.1 * auth_k)),
        "auth_leg_pos": float(stats.get("auth_leg_pos", 2.09 * auth_k)),
        "auth_sat_rate": float(stats.get("auth_sat_rate") or 0),
        "auth_sat_count": int(stats.get("auth_sat_count") or 0),
        "auth_sat_steps": int(stats.get("auth_sat_steps") or 0),
        "fric": fric,
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
        "assist": False, "freeze": False,
        "mp4": str(out_mp4) if out_mp4.exists() else None,
        "params": params,
    }
    m["t88_ok"] = t88_ok(m) if abs(m["T"] - 0.88) < 1e-6 else None
    m["gate_e"] = gate_e(m) if m["T"] <= 0.75 + 1e-9 else False
    (ITER / f"{tag}_ss.json").write_text(json.dumps(m, indent=2, default=str))
    print(
        f"ROW {tag}: tip={m['tip']:.2f} dx={m['dx']:+.3f} bout={m['bout_min']:.3f} "
        f"stx={m['stx_mean']:.3f} p95={m['stx_p95']:.3f} skate={m['skate']} "
        f"sat={m['auth_sat_rate']:.4f} ge={m['gate_e']} t88={m['t88_ok']}",
        flush=True,
    )
    return m


def t88_ok(m: dict) -> bool:
    if m.get("flipped") or m.get("exploded"):
        return False
    if m["tip"] < 8.0:
        return False
    if m["bout_min"] < 0.10:
        return False
    if m["stx_p95"] > 0.18:
        return False
    if m["skate"]:
        return False
    if m["hip_corr"] > -0.45:
        return False
    return True


def gate_e(m: dict) -> bool:
    if m["tip"] < 8.0:
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
    if min(m["peak_clear_L"], m["peak_clear_R"]) < 0.012:
        return False
    if m["n_bouts_L"] < 5 or m["n_bouts_R"] < 5:
        return False
    return True


def write_note(table: list[dict]) -> None:
    ge = [r for r in table if r.get("gate_e")]
    k_gt1_ge = [r for r in ge if r.get("auth_k", 1) > 1.0 + 1e-9]
    any_ge = bool(ge)
    only_gt1 = bool(k_gt1_ge) and not any(r.get("auth_k", 1) <= 1.0 + 1e-9 for r in ge)
    auth00 = next(r for r in table if r["tag"] == "AUTH00_T88_GRO01_k1")
    if any_ge and only_gt1:
        outcome = "KIT_HX_INSUFFICIENT"
        verdict = "Gate E only for k_auth>1 — kit HX ±2.1 insufficient for Gate E cadence (sim)"
    elif any_ge:
        outcome = "GATE_E_AT_1X_OR_MIXED"
        verdict = "Gate E unlocked (incl. possible 1.0×) — unexpected vs prior falsifiers; see table"
    else:
        outcome = "HARD_FALSIFIED"
        verdict = "HX-authority-upsized open-loop HARD-FALSIFIED — no Gate E for any k incl. uncapped"

    def fmt(r):
        ge_s = "PASS" if r.get("gate_e") else ("n/a" if r["T"] > 0.75 else "FAIL")
        t88 = r.get("t88_ok")
        t88s = "yes" if t88 else ("no" if t88 is False else "—")
        fr = r.get("fric")
        frs = f"{fr:.1f}" if fr is not None else "—"
        return (
            f"| {r['tag']} | {r['T']:.2f} | {r.get('shape','?')} | {r['auth_k']:.2f} | {frs} | "
            f"{r['tip']:.2f} | {r['dx']:+.3f} | {r['bout_min']:.3f} | {r['stx_mean']:.3f} | "
            f"{r['stx_p95']:.3f} | {str(r['skate']).lower()} | {r['auth_sat_rate']:.4f} | "
            f"{ge_s} | {t88s} |"
        )
    rows = "\n".join(fmt(r) for r in table)
    note = f'''# HX authority envelope A/B — AUTH00+

**When:** Sun 27 Sep 2026 Europe/London (BST)
**Role:** Founding Controls
**Plant (geometry):** `ainex_controls_m2_145.xml` locked — no cadsole/size swap
**Assist / freeze / xfrc:** OFF
**Cospec:** `docs/AUTH_ENVELOPE_HX_COSPEC.md`
**Outer:** GRO01 best shape (primary); CSF50 control
**Authority:** scale leg HX torque ±2.1 and position ±2.09 by `k_auth` at runtime (XML on disk unchanged)
**Inner scrub:** none (no residual / WBC / MPC)

## Verdict

| Claim | Result |
|-------|--------|
| AUTH00 GRO01 T88 @ 1.0× floor holds | **{"YES" if auth00.get("t88_ok") else "NO"}** |
| Gate E at any k_auth (incl. uncapped) | **{"YES" if any_ge else "NO"}** |
| Gate E only for k_auth>1 | **{"YES" if only_gt1 else "NO"}** |
| Outcome | **{outcome}** |

{verdict}

## M145 table (authoritative)

| Tag | T | shape | k | μ | tip | dx | bout_min | stx mean | stx p95 | skate | sat_rate | Gate E | T88 ok |
|-----|---|-------|---|---|-----|-----|----------|----------|---------|-------|----------|--------|--------|
{rows}

sat_rate = fraction of (leg-actuator × physics-step) samples with |τ| ≥ 0.98 × forcerange.

### Reading

- **AUTH00:** GRO01 @ T=0.88, k=1.0 — must hold T88 floor.
- **AUTH00b:** CSF50 @ T=0.88, k=1.0 — control.
- **AUTH01–05:** GRO01 @ T=0.75 with k∈{{1.0, 1.25, 1.5, 2.0, 10.0}}.
- **AUTH_mu*:** GRO01 @ T=0.75, k=1.0, μ∈{{0.8, 1.0, 1.2}}.

## Stop rule

{"**Kit HX ±2.1 insufficient (sim):** Gate E needs k_auth>1. Document saturation; stop more gait search under 1.0×." if only_gt1 else ("**HX-authority-upsized open-loop HARD-FALSIFIED** for Gate E on M145. Next lever is contact/geometry honesty with Hardware (still sim) — not more k_auth or residual/WBC/MPC." if outcome=="HARD_FALSIFIED" else "See table — Gate E unlocked; lock sim claim carefully.")}

Locked falsifiers remain: CPG+VIK, residual, WBC, MPC, dual-T. This class adds the authority-envelope outcome above.

## Artifacts

- `--auth-k` hook in `scripts/walk_gait_ainex.py` (+ sat logging)
- `scripts/score_auth_envelope.py`
- `AUTH_ENVELOPE_TABLE.json`, `AUTH_ENVELOPE_NOTE.md`
- AUTH00+ mp4/json/_ss.json
'''
    (ITER / "AUTH_ENVELOPE_NOTE.md").write_text(note)
    print(f"wrote note outcome={outcome}", flush=True)
    return outcome


def main() -> None:
    ITER.mkdir(parents=True, exist_ok=True)
    table: list[dict] = []
    dur = 9.0

    # AUTH00 T88 GRO01 k=1
    table.append(run_one(shape_to_params(GRO01, 0.88), tag="AUTH00_T88_GRO01_k1",
                         auth_k=1.0, fric=None, duration=dur, video=True))
    # AUTH00b CSF50 T88 k=1
    table.append(run_one(shape_to_params(CSF50, 0.88), tag="AUTH00b_T88_CSF50_k1",
                         auth_k=1.0, fric=None, duration=dur, video=True))

    # T75 k grid on GRO01
    for tag, k in [
        ("AUTH01_T75_k1", 1.0),
        ("AUTH02_T75_k125", 1.25),
        ("AUTH03_T75_k15", 1.5),
        ("AUTH04_T75_k2", 2.0),
        ("AUTH05_T75_k10", 10.0),  # uncapped-or-large
    ]:
        table.append(run_one(shape_to_params(GRO01, 0.75), tag=tag,
                             auth_k=k, fric=None, duration=dur, video=True))

    # optional μ at 1.0×
    for tag, mu in [
        ("AUTH_mu08_T75_k1", 0.8),
        ("AUTH_mu10_T75_k1", 1.0),
        ("AUTH_mu12_T75_k1", 1.2),
    ]:
        table.append(run_one(shape_to_params(GRO01, 0.75), tag=tag,
                             auth_k=1.0, fric=mu, duration=dur, video=True))

    # CSF50 T75 @ 1.0 and @ 10 as extra control
    table.append(run_one(shape_to_params(CSF50, 0.75), tag="AUTH_CSF_T75_k1",
                         auth_k=1.0, fric=None, duration=dur, video=True))
    table.append(run_one(shape_to_params(CSF50, 0.75), tag="AUTH_CSF_T75_k10",
                         auth_k=10.0, fric=None, duration=dur, video=True))

    (ITER / "AUTH_ENVELOPE_TABLE.json").write_text(json.dumps(table, indent=2, default=str))
    outcome = write_note(table)
    print("=== FINAL ===", outcome, "gate_e_tags", [r["tag"] for r in table if r.get("gate_e")], flush=True)


if __name__ == "__main__":
    main()
