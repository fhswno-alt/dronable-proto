#!/usr/bin/env python3
"""Contact / geometry honesty A/B — GEO00+GEO01 (GEO02/03 when Hardware ships).

Cospec: docs/CONTACT_GEOMETRY_HONESTY_COSPEC.md
Plant doc: docs/PLANT_CADSOLE_AB.md
Outer: GRO01 primary + CSF50 control. k_auth=1.0. No residual/WBC/MPC.
SOLE_OFFSET stays 0.026 (cadsole preserves sole bottom).
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from ss_step_ainex import analyze_ss  # noqa: E402
from score_auth_envelope import (  # noqa: E402 — identical Gate E / T88 criteria
    GRO01,
    CSF50,
    shape_to_params,
    t88_ok,
    gate_e,
)

WALK = ROOT / "scripts" / "walk_gait_ainex.py"
ITER = ROOT / "previews" / "ainex_walk" / "iterate"
PY = ROOT / ".venv" / "bin" / "python"
HIW = ROOT / "mujoco" / "ainex_hiwonder"

# Prefer EGL; fall back to glfw (box EGL often broken)
def _pick_gl() -> str:
    if os.environ.get("MUJOCO_GL"):
        return os.environ["MUJOCO_GL"]
    # probe egl quickly
    env = dict(os.environ)
    env["MUJOCO_GL"] = "egl"
    r = subprocess.run(
        [str(PY), "-c", "import mujoco"],
        env=env, capture_output=True, text=True,
    )
    return "egl" if r.returncode == 0 else "glfw"


PLANTS = {
    "GEO00": {
        "path": HIW / "ainex_controls_m2_145.xml",
        "label": "locked M145 16 mm contact box (control)",
        "delta": "145×86; half_z=0.008 (16 mm); pos_z=-0.018; sole bottom=-0.026",
    },
    "GEO01": {
        "path": HIW / "ainex_controls_m2_145_cadsole.xml",
        "label": "cadsole ~4.5 mm contact height, 145×86 planform",
        "delta": "145×86; half_z=0.00225 (4.5 mm); pos_z=-0.02375; sole bottom=-0.026",
    },
    "GEO02a": {
        "path": HIW / "ainex_controls_m2_160x90.xml",
        "label": "planform 160×90, 16 mm box",
        "delta": "160×90 (+15 L, +4 W); half 0.080×0.045×0.008; pos_z=-0.018; sole=-0.026",
    },
    "GEO02b": {
        "path": HIW / "ainex_controls_m2_155x86.xml",
        "label": "planform 155×86 (+10 mm length), 16 mm box",
        "delta": "155×86 (+10 L); half 0.0775×0.043×0.008; pos_z=-0.018; sole=-0.026",
    },
    "GEO02c": {
        "path": HIW / "ainex_controls_m2_160x90_cadsole.xml",
        "label": "160×90 planform + 4.5 mm cadsole z",
        "delta": "160×90; half_z=0.00225; pos_z=-0.02375; sole=-0.026",
    },
    "GEO03": {
        "path": HIW / "ainex_controls_m2_145_softsole.xml",
        "label": "145×86 soft solref/solimp (compliance only)",
        "delta": "same 16 mm box; solref=0.05 0.8; solimp=0.85 0.90 0.01 0.5 2",
    },
}

# Aliases / reserved names (Hardware pack)
GEO02_PRIMARY = PLANTS["GEO02a"]["path"]      # ainex_controls_m2_160x90.xml
GEO02_SECONDARY = PLANTS["GEO02b"]["path"]    # ainex_controls_m2_155x86.xml
GEO02_CADSOLE = PLANTS["GEO02c"]["path"]      # ainex_controls_m2_160x90_cadsole.xml
GEO03_SOFT = PLANTS["GEO03"]["path"]          # ainex_controls_m2_145_softsole.xml
def run_one(
    params: dict,
    *,
    tag: str,
    model: Path,
    plant_tag: str,
    duration: float,
    video: bool,
    gl: str,
) -> dict:
    stats_out = ITER / f"{tag}.json"
    out_mp4 = ITER / f"{tag}.mp4"
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
    env["MUJOCO_GL"] = gl
    print(f"\n=== RUN {tag} plant={plant_tag} model={model.name} gl={gl} video={video} ===", flush=True)
    r = subprocess.run(cmd, cwd=str(ROOT), env=env, capture_output=True, text=True)
    if r.returncode != 0:
        print((r.stdout or "")[-800:], flush=True)
        print((r.stderr or "")[-400:], flush=True)
    if not stats_out.exists():
        return {
            "tag": tag, "plant": plant_tag, "model": str(model),
            "error": f"rc={r.returncode}", "skate": True, "tip": 0.0,
            "dx": 0.0, "stx_mean": 9.0, "stx_p95": 9.0, "bout_min": 0.0,
            "gate_e": False, "t88_ok": False, "auth_k": 1.0,
        }
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
    shape = "GRO01" if abs(params["hip_amp"] - 0.145) < 1e-6 else "CSF50"
    m = {
        "tag": tag,
        "plant": plant_tag,
        "model": str(model),
        "T": float(params["gait_t"]),
        "shape": shape,
        "auth_k": 1.0,
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
        f"ge={m['gate_e']} t88={m['t88_ok']}",
        flush=True,
    )
    return m


def write_note(table: list[dict], *, gl: str) -> str:
    geo00_t75 = next((r for r in table if r["tag"] == "GEO00_T75_GRO01"), None)
    geo01_t75 = next((r for r in table if r["tag"] == "GEO01_T75_GRO01"), None)
    geo00_t88 = next((r for r in table if r["tag"] == "GEO00_T88_GRO01"), None)
    geo01_t88 = next((r for r in table if r["tag"] == "GEO01_T88_GRO01"), None)
    ge_hits = [r for r in table if r.get("gate_e")]
    # AUTH01 reference: stx≈0.121 p95≈0.638 skate=true tip=9 dx≈0.053
    auth01_match = False
    if geo00_t75 and not geo00_t75.get("error"):
        auth01_match = (
            bool(geo00_t75["skate"])
            and geo00_t75["stx_mean"] > 0.08
            and geo00_t75["stx_p95"] > 0.18
            and not geo00_t75.get("gate_e")
        )

    if ge_hits:
        verdict = f"Gate E UNLOCKED on: {', '.join(r['tag'] for r in ge_hits)}"
        outcome = "GATE_E_UNLOCK"
    else:
        # Full pack scored in CONTACT_GEOMETRY_NOTE merge path; this branch is GEO00+01-only runs
        plants_seen = {r.get("plant") for r in table}
        full = {"GEO00","GEO01","GEO02a","GEO02b","GEO02c","GEO03"}.issubset(plants_seen)
        if full:
            verdict = "Contact/geometry honesty HARD-FALSIFIED — all scored GEO plants fail Gate E."
            outcome = "HARD_FALSIFIED"
        else:
            verdict = (
                "GEO00+GEO01 fail Gate E (partial until full pack). "
                "Use --full-pack when GEO02/03 XML present."
            )
            outcome = "PARTIAL_FAIL_AWAIT_GEO02_03"

    def fmt(r):
        ge_s = "PASS" if r.get("gate_e") else ("n/a" if r["T"] > 0.75 else "FAIL")
        t88 = r.get("t88_ok")
        t88s = "yes" if t88 else ("no" if t88 is False else "—")
        return (
            f"| {r['tag']} | {r['plant']} | {r['T']:.2f} | {r.get('shape','?')} | "
            f"{r['tip']:.2f} | {r['dx']:+.3f} | {r['bout_min']:.3f} | {r['stx_mean']:.3f} | "
            f"{r['stx_p95']:.3f} | {str(r['skate']).lower()} | {ge_s} | {t88s} |"
        )

    rows = "\n".join(fmt(r) for r in table)
    pending = []
    for name, path in [
        ("GEO02 primary", GEO02_PRIMARY),
        ("GEO02 secondary", GEO02_SECONDARY),
        ("GEO03 softsole", GEO03_SOFT),
    ]:
        pending.append(f"- {name}: `{path.name}` — {'PRESENT' if path.exists() else 'not shipped yet'}")

    note = f'''# Contact / geometry honesty — GEO00+GEO01

**When:** Sun 27 Sep 2026 Europe/London (BST)
**Role:** Founding Controls
**Cospec:** `docs/CONTACT_GEOMETRY_HONESTY_COSPEC.md`
**Plant doc:** `docs/PLANT_CADSOLE_AB.md`
**Assist / freeze / xfrc:** OFF
**Authority:** k_auth=1.0 only (HX envelope closed)
**Inner scrub:** none (no residual / WBC / MPC)
**SOLE_OFFSET:** 0.026 (unchanged; cadsole preserves sole bottom)
**GL:** `{gl}`

## Plants scored

| Tag | Path | Geometry |
|-----|------|----------|
| GEO00 | `ainex_controls_m2_145.xml` | {PLANTS["GEO00"]["delta"]} |
| GEO01 | `ainex_controls_m2_145_cadsole.xml` | {PLANTS["GEO01"]["delta"]} |

CAD stack reference: 3 mm plate + 1.5 mm tread = **4.5 mm** (`cad/m2_outsole/`).

## Verdict

| Claim | Result |
|-------|--------|
| GEO00 T88 GRO01 floor | **{"YES" if geo00_t88 and geo00_t88.get("t88_ok") else "NO"}** |
| GEO00 T75 GRO01 matches AUTH01 FAIL pattern | **{"YES" if auth01_match else "NO"}** |
| GEO01 T88 GRO01 floor | **{"YES" if geo01_t88 and geo01_t88.get("t88_ok") else "NO / broke"}** |
| GEO01 T75 GRO01 Gate E | **{"YES" if geo01_t75 and geo01_t75.get("gate_e") else "NO"}** |
| Outcome (GEO00+01 only) | **{outcome}** |

{verdict}

## M145 table (authoritative)

| Tag | plant | T | shape | tip | dx | bout_min | stx mean | stx p95 | skate | Gate E | T88 ok |
|-----|-------|---|-------|-----|-----|----------|----------|---------|-------|--------|--------|
{rows}

### Reading

- **GEO00 T75 GRO01** must echo AUTH01 FAIL (skate, stx mean≳0.12, p95≳0.6).
- **GEO01** is the vertical-honesty claim (~4.5 mm vs 16 mm box). Gate E here would lock the cadsole plant.
- CSF50 rows are control outer on each plant.

## Hardware pending (not scored)

{chr(10).join(pending)}

Hooks in `scripts/score_contact_geometry.py` (`GEO02_PRIMARY`, `GEO02_SECONDARY`, `GEO03_SOFT` + commented `PLANTS` entries). Fold in when Hardware confirms XML up — do not invent plants.

## Stop rule (partial)

Full **contact/geometry honesty HARD-FALSIFIED** only after GEO02/03 also fail Gate E with honest T88. GEO00+01 alone → {"Gate E unlock on cadsole — freeze GEO01 XML for further work." if outcome=="GATE_E_UNLOCK" else "partial fail; await GEO02/03."}

## Artifacts

- `scripts/score_contact_geometry.py`
- `CONTACT_GEOMETRY_TABLE.json`, `CONTACT_GEOMETRY_NOTE.md`
- GEO00/01 mp4/json/_ss.json under `previews/ainex_walk/iterate/`
'''
    (ITER / "CONTACT_GEOMETRY_NOTE.md").write_text(note)
    print(f"wrote note outcome={outcome}", flush=True)
    return outcome


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=9.0)
    ap.add_argument("--plants", type=str, default="GEO00,GEO01",
                    help="Comma list. Default GEO00,GEO01.")
    ap.add_argument("--full-pack", action="store_true",
                    help="Run GEO00–GEO03 pack (02a/02b/02c/03) for plants whose XML exists")
    ap.add_argument("--include-pending", action="store_true",
                    help="Alias for --full-pack")
    args = ap.parse_args()

    gl = _pick_gl()
    os.environ["MUJOCO_GL"] = gl
    print(f"[geo] MUJOCO_GL={gl}", flush=True)

    ITER.mkdir(parents=True, exist_ok=True)
    if args.full_pack or args.include_pending:
        wanted = ["GEO00", "GEO01", "GEO02a", "GEO02b", "GEO02c", "GEO03"]
    else:
        wanted = [x.strip() for x in args.plants.split(",") if x.strip()]

    table: list[dict] = []
    for plant_tag in wanted:
        info = PLANTS.get(plant_tag)
        if info is None:
            print(f"[geo] skip unknown plant {plant_tag}", flush=True)
            continue
        model = info["path"]
        if not model.exists():
            print(f"[geo] SKIP {plant_tag}: missing {model}", flush=True)
            continue

        # GRO01 T88 + T75
        table.append(run_one(
            shape_to_params(GRO01, 0.88), tag=f"{plant_tag}_T88_GRO01",
            model=model, plant_tag=plant_tag, duration=args.duration, video=True, gl=gl,
        ))
        table.append(run_one(
            shape_to_params(GRO01, 0.75), tag=f"{plant_tag}_T75_GRO01",
            model=model, plant_tag=plant_tag, duration=args.duration, video=True, gl=gl,
        ))
        # CSF50 T75 (+ T88 on GEO00/01)
        table.append(run_one(
            shape_to_params(CSF50, 0.75), tag=f"{plant_tag}_T75_CSF50",
            model=model, plant_tag=plant_tag, duration=args.duration, video=True, gl=gl,
        ))
        if plant_tag in ("GEO00", "GEO01"):
            table.append(run_one(
                shape_to_params(CSF50, 0.88), tag=f"{plant_tag}_T88_CSF50",
                model=model, plant_tag=plant_tag, duration=args.duration, video=True, gl=gl,
            ))

    (ITER / "CONTACT_GEOMETRY_TABLE.json").write_text(json.dumps(table, indent=2, default=str))
    outcome = write_note(table, gl=gl)
    print(
        "=== FINAL ===", outcome,
        "gate_e", [r["tag"] for r in table if r.get("gate_e")],
        flush=True,
    )


if __name__ == "__main__":
    main()
