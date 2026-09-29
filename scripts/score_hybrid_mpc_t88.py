#!/usr/bin/env python3
"""Score hybrid MPC + T88 hard-reject (MPC00+ table + mild T88 scan)."""
from __future__ import annotations

import json
import os
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from ss_step_ainex import run_trial, _csf  # noqa: E402

ITER = ROOT / "previews" / "ainex_walk" / "iterate"
os.environ.setdefault("MUJOCO_GL", "glfw")

# CSF50 basin (locked)
CSF50 = dict(
    hip_amp=0.116, step_len=0.021, ds=0.31, knee_swing=0.80,
    com_shift=0.275, com_shift_lead=0.23, knee_stance=0.40,
    com_z=0.225, hip_bias=0.06, plant_kd=115, cp=True, vik=True,
    vik_kp=0.45, vik_clip=0.03,
)


def gate_e(m: dict) -> bool:
    tip = float(m.get("tip", 0) or 0)
    dx = float(m.get("dx", 0) or 0)
    bout = float(m.get("bout_min", 0) or 0)
    stx = float(m.get("stx_mean", 9) or 9)
    p95 = float(m.get("stx_p95", 9) or 9)
    hip = float(m.get("hip_corr", 0) or 0)
    lead = int(m.get("foot_lead", 0) or 0)
    clear = min(float(m.get("peak_clear_L", 0) or 0), float(m.get("peak_clear_R", 0) or 0))
    nL = int(m.get("n_bouts_L", 0) or 0)
    nR = int(m.get("n_bouts_R", 0) or 0)
    skate = bool(m.get("skate", True))
    if tip < 8.0:
        return False
    if dx < 0.15:
        return False
    if bout < 0.10:
        return False
    if stx > 0.08 + 1e-9:
        return False
    if p95 > 0.18 + 1e-9:
        return False
    if skate:
        return False
    if hip > -0.45:
        return False
    if lead < 5:
        return False
    if clear < 0.012:
        return False
    if nL < 5 or nR < 5:
        return False
    return True


def t88_reject(m: dict) -> bool:
    p95 = float(m.get("stx_p95", 9) or 9)
    bout = float(m.get("bout_min", 0) or 0)
    tip = float(m.get("tip", 0) or 0)
    if p95 > 0.18:
        return True
    if bout < 0.10:
        return True
    if tip < 7.5:
        return True
    if m.get("skate"):
        return True
    return False


def summarize(raw: dict, trial: dict) -> dict:
    tip = float(raw.get("tip_free_post_gait_max_s") or 0)
    dx = float(raw.get("dx_m") or 0)
    bout_L = float(raw.get("ss_mean_bout_s_L") or 0)
    bout_R = float(raw.get("ss_mean_bout_s_R") or 0)
    stx_L = float(raw.get("stance_foot_vx_mean_L") or raw.get("stance_vx_mean_L") or 9)
    stx_R = float(raw.get("stance_foot_vx_mean_R") or raw.get("stance_vx_mean_R") or 9)
    p95_L = float(raw.get("stance_foot_vx_p95_L") or 9)
    p95_R = float(raw.get("stance_foot_vx_p95_R") or 9)
    hip = float(raw.get("hip_q_corr_LR") or 0)
    lead = int(raw.get("foot_lead_cycles") or 0)
    nL = int(raw.get("ss_bouts_L") or 0)
    nR = int(raw.get("ss_bouts_R") or 0)
    clear_L = float(raw.get("ss_peak_clear_m_L") or 0)
    clear_R = float(raw.get("ss_peak_clear_m_R") or 0)
    skate = bool(raw.get("skate", True))
    m = {
        "tag": trial["tag"],
        "T": float(trial["gait_t"]),
        "mpc": bool(trial.get("hybrid_mpc", False)),
        "mpc_n": trial.get("mpc_n"),
        "mpc_u_max": trial.get("mpc_u_max"),
        "mpc_apply_thr": trial.get("mpc_apply_thr"),
        "mpc_disable_above_t": trial.get("mpc_disable_above_t"),
        "tip": tip,
        "dx": dx,
        "bout_L": bout_L,
        "bout_R": bout_R,
        "bout_min": min(bout_L, bout_R) if (bout_L and bout_R) else 0.0,
        "stx_L": stx_L,
        "stx_R": stx_R,
        "stx_mean": 0.5 * (stx_L + stx_R),
        "p95_L": p95_L,
        "p95_R": p95_R,
        "stx_p95": 0.5 * (p95_L + p95_R),
        "hip_corr": hip,
        "foot_lead": lead,
        "n_bouts_L": nL,
        "n_bouts_R": nR,
        "peak_clear_L": clear_L,
        "peak_clear_R": clear_R,
        "skate": skate,
        "assist": False,
        "freeze": False,
        "mp4": raw.get("mp4"),
        "error": raw.get("error"),
    }
    m["t88_reject"] = t88_reject(m) if abs(m["T"] - 0.88) < 1e-6 else False
    m["gate_e"] = gate_e(m) if m["T"] <= 0.75 + 1e-9 else False
    return m


def relocate_mp4(tag: str, raw: dict) -> None:
    src = raw.get("mp4")
    if not src:
        return
    src_p = Path(src)
    dst = ITER / f"{tag}.mp4"
    if src_p.exists() and src_p.resolve() != dst.resolve():
        shutil.copy2(src_p, dst)
        raw["mp4"] = str(dst)


def run_one(trial: dict, *, video: bool, duration: float = 9.0) -> dict:
    raw = run_trial(trial, video=video, duration=duration)
    relocate_mp4(trial["tag"], raw)
    m = summarize(raw, trial)
    # write compact row next to full ss
    (ITER / f"{trial['tag']}_ss.json").write_text(json.dumps({**m, **{k: raw.get(k) for k in (
        'ss_duty_total','ss_duty_L','ss_duty_R','ds_duty','flight_duty',
        'ss_bouts_L','ss_bouts_R','ss_mean_bout_s_L','ss_mean_bout_s_R',
        'ss_max_bout_s_L','ss_max_bout_s_R','ss_peak_clear_m_L','ss_peak_clear_m_R',
        'ss_mean_clear_m_L','ss_mean_clear_m_R','ss_side_flips','ss_verified_step',
        'clean_walk_ss','ss_fail_reasons','tip_free_post_gait_max_s','dx_m',
        'stance_vx_mean_L','stance_vx_mean_R','hip_q_corr_LR','foot_lead_cycles',
        'clean_walk_base','stance_foot_vx_mean_L','stance_foot_vx_mean_R',
        'stance_foot_vx_p95_L','stance_foot_vx_p95_R','skate',
    ) if k in raw}}, indent=2, default=str))
    print(
        f"ROW {m['tag']}: tip={m['tip']:.2f} dx={m['dx']:+.3f} bout={m['bout_min']:.3f} "
        f"stx={m['stx_mean']:.3f} p95={m['stx_p95']:.3f} skate={m['skate']} "
        f"gate_e={m['gate_e']} t88_rej={m['t88_reject']}",
        flush=True,
    )
    return m


def main() -> None:
    ITER.mkdir(parents=True, exist_ok=True)
    table: list[dict] = []

    # --- Main A/B table ---
    trials = [
        # MPC00: T88 OFF baseline
        {**CSF50, "tag": "MPC00_T88_off", "gait_t": 0.88, "ds": 0.31, "hybrid_mpc": False},
        # MPC01: T88 ON with schedule disable (should match OFF / not reject)
        {**CSF50, "tag": "MPC01_T88_on_sched", "gait_t": 0.88, "ds": 0.31,
         "hybrid_mpc": True, "mpc_n": 12, "mpc_u_max": 0.10, "mpc_apply_thr": 0.11,
         "mpc_disable_above_t": 0.86},
        # MPC02: T88 ON forced apply (true nonzero controller test)
        {**CSF50, "tag": "MPC02_T88_on_force", "gait_t": 0.88, "ds": 0.31,
         "hybrid_mpc": True, "mpc_n": 12, "mpc_u_max": 0.10, "mpc_apply_thr": 0.08,
         "mpc_disable_above_t": 1.0},
        # MPC03: T75 OFF
        {**CSF50, "tag": "MPC03_T75_off", "gait_t": 0.75, "ds": 0.26, "hybrid_mpc": False},
        # MPC04: T75 ON A (default)
        {**CSF50, "tag": "MPC04_T75_on_A", "gait_t": 0.75, "ds": 0.26,
         "hybrid_mpc": True, "mpc_n": 12, "mpc_u_max": 0.10, "mpc_apply_thr": 0.11,
         "mpc_disable_above_t": 0.86},
        # MPC05: T75 ON B (stronger / longer horizon)
        {**CSF50, "tag": "MPC05_T75_on_B", "gait_t": 0.75, "ds": 0.26,
         "hybrid_mpc": True, "mpc_n": 16, "mpc_u_max": 0.14, "mpc_apply_thr": 0.08,
         "mpc_disable_above_t": 0.86},
        # MPC06: T75 ON C (milder)
        {**CSF50, "tag": "MPC06_T75_on_C", "gait_t": 0.75, "ds": 0.26,
         "hybrid_mpc": True, "mpc_n": 8, "mpc_u_max": 0.06, "mpc_apply_thr": 0.14,
         "mpc_disable_above_t": 0.86},
    ]

    for tr in trials:
        table.append(run_one(tr, video=True, duration=9.0))

    (ITER / "MPC_T88_TABLE.json").write_text(json.dumps(table, indent=2))

    # --- Mild T88 scan: nonzero MPC must not reject CSF50 ---
    mild_rows = []
    u_max_list = [0.04, 0.06, 0.08, 0.10, 0.12, 0.14]
    n_list = [8, 12, 16]
    thr_list = [0.08, 0.11]  # 6*3*2=36 matches WBC mild size
    for u in u_max_list:
        for n in n_list:
            for thr in thr_list:
                tag = f"MPC_mild_u{u:.2f}_n{n}_thr{thr:.2f}".replace(".", "")
                tr = {
                    **CSF50, "tag": tag, "gait_t": 0.88, "ds": 0.31,
                    "hybrid_mpc": True, "mpc_n": n, "mpc_u_max": u,
                    "mpc_apply_thr": thr, "mpc_disable_above_t": 1.0,  # force apply at T88
                }
                # no-video for scan speed
                m = run_one(tr, video=False, duration=9.0)
                mild_rows.append(m)

    safe = [r for r in mild_rows if not r["t88_reject"] and r.get("error") is None]
    mild = {
        "n_total": len(mild_rows),
        "n_safe": len(safe),
        "best": min(safe, key=lambda r: r["stx_mean"]) if safe else None,
        "base88": next((r for r in table if r["tag"] == "MPC00_T88_off"), None),
        "rows": mild_rows,
    }
    (ITER / "MPC_MILD_SCAN.json").write_text(json.dumps(mild, indent=2))
    print(f"\nMILD SCAN n_safe={mild['n_safe']}/{mild['n_total']}", flush=True)

    # Gate E any?
    ge = [r for r in table if r.get("gate_e")]
    print(f"GATE_E hits: {[r['tag'] for r in ge]}", flush=True)
    print(f"Wrote {ITER/'MPC_T88_TABLE.json'} and mild scan", flush=True)


if __name__ == "__main__":
    main()
