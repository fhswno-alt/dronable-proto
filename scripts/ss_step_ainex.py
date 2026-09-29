#!/usr/bin/env python3
"""Hardware-supporting Controls — single-support (SS) step trials on AiNex M2 145×86.

Additive to walk_gait_ainex.py (does not mutate Controls basins). Goal: verify a
proper bipedal step — swing foot fully clear (contact≈0 / sole clearance>0) while
stance planted; alternate L/R; assist OFF; freeze OFF; HX ±2.1/±0.7.

Plant default: mujoco/ainex_hiwonder/ainex_controls_m2_145.xml

Usage:
  MUJOCO_GL=egl .venv/bin/python scripts/ss_step_ainex.py --sweep --video-best
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WALK = ROOT / "scripts" / "walk_gait_ainex.py"
PLANT = ROOT / "mujoco" / "ainex_hiwonder" / "ainex_controls_m2_145.xml"
ITER = ROOT / "previews" / "ainex_walk" / "iterate"
NOTE = ITER / "SS_STEP_HARDWARE_NOTE.md"
PY = ROOT / ".venv" / "bin" / "python"

SOLE_OFFSET = 0.026  # foot-body z − sole bottom (box pos −0.018 + half-h 0.008)
CTRL_HZ = 50.0

# Gate D bar above E80 brief unload (~0.08 s mean bout)
SS_MIN_BOUT_S = 0.12
SS_MIN_PEAK_CLEAR_M = 0.012
SS_MIN_CYCLES = 3
SS_MIN_DUTY = 0.15
SS_MIN_TIP = 5.0
SS_MIN_DX = 0.05


def _load_csv(path: Path) -> np.ndarray:
    rows = []
    with path.open() as f:
        for line in f:
            if line.startswith("#") or line.startswith("t,"):
                continue
            rows.append([float(x) for x in line.strip().split(",")])
    return np.asarray(rows, dtype=np.float64) if rows else np.zeros((0, 27))


def _runs(mask: np.ndarray) -> list[int]:
    out: list[int] = []
    i, n = 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            out.append(j - i)
            i = j
        else:
            i += 1
    return out


def analyze_ss(csv_path: Path, stats: dict) -> dict:
    a = _load_csv(csv_path)
    if len(a) < 10:
        return {"ss_error": "empty_csv"}

    i0 = int((0.80 + 1.40 + 0.2) * CTRL_HZ)
    ss = a[i0:] if len(a) > i0 else a
    amp = ss[:, 26]
    mask = amp > 0.2
    if not np.any(mask):
        mask = np.ones(len(ss), dtype=bool)
    cL = ss[mask, 21] > 0.5
    cR = ss[mask, 22] > 0.5
    zl, zr = ss[mask, 5], ss[mask, 6]

    both = cL & cR
    ssL = (~cL) & cR
    ssR = (~cR) & cL
    flight = (~cL) & (~cR)
    xor = ssL | ssR

    clear_L = zl - SOLE_OFFSET
    clear_R = zr - SOLE_OFFSET
    rl, rr = _runs(ssL), _runs(ssR)
    rl_s = [x / CTRL_HZ for x in rl]
    rr_s = [x / CTRL_HZ for x in rr]

    side_seq = []
    i, n = 0, len(xor)
    while i < n:
        if ssL[i]:
            j = i
            while j < n and ssL[j]:
                j += 1
            side_seq.append("L")
            i = j
        elif ssR[i]:
            j = i
            while j < n and ssR[j]:
                j += 1
            side_seq.append("R")
            i = j
        else:
            i += 1
    flips = sum(1 for a_, b_ in zip(side_seq, side_seq[1:]) if a_ != b_)

    peak_clear_L = float(np.max(clear_L[ssL])) if np.any(ssL) else 0.0
    peak_clear_R = float(np.max(clear_R[ssR])) if np.any(ssR) else 0.0
    mean_clear_L = float(np.mean(clear_L[ssL])) if np.any(ssL) else 0.0
    mean_clear_R = float(np.mean(clear_R[ssR])) if np.any(ssR) else 0.0
    mean_bout_L = float(np.mean(rl_s)) if rl_s else 0.0
    mean_bout_R = float(np.mean(rr_s)) if rr_s else 0.0
    max_bout_L = float(max(rl_s)) if rl_s else 0.0
    max_bout_R = float(max(rr_s)) if rr_s else 0.0

    tip = float(stats.get("tip_free_post_gait_max_s") or 0.0)
    dx = float(stats.get("dx_m") or 0.0)
    stx_L = float(stats.get("stance_foot_vx_mean_L") or float("nan"))
    stx_R = float(stats.get("stance_foot_vx_mean_R") or float("nan"))
    hip_corr = float(stats.get("hip_q_corr_LR") or 0.0)
    lead = int(stats.get("foot_lead_cycles") or 0)
    assist = bool(stats.get("balance_assist"))
    freeze = bool(stats.get("stance_freeze"))
    skate = bool(stats.get("skate"))
    clean_base = bool(stats.get("clean_walk_claim"))

    ss_duty = float(np.mean(xor)) if len(xor) else 0.0
    n_bouts_L, n_bouts_R = len(rl), len(rr)

    # Mean bout both sides (E80 ~0.08s fails — not a soft-pass via max spike)
    sustained = (mean_bout_L >= SS_MIN_BOUT_S) and (mean_bout_R >= SS_MIN_BOUT_S)
    clearance_ok = peak_clear_L >= SS_MIN_PEAK_CLEAR_M and peak_clear_R >= SS_MIN_PEAK_CLEAR_M
    alt_ok = n_bouts_L >= SS_MIN_CYCLES and n_bouts_R >= SS_MIN_CYCLES and flips >= 4
    tip_ok = tip >= SS_MIN_TIP and not bool(stats.get("exploded")) and not bool(stats.get("flipped"))
    dx_ok = dx >= SS_MIN_DX
    duty_ok = ss_duty >= SS_MIN_DUTY
    no_cheat = (not assist) and (not freeze)

    ss_verified = bool(
        sustained and clearance_ok and alt_ok and tip_ok and dx_ok and duty_ok and no_cheat
        and hip_corr <= -0.45 and lead >= 3
    )
    clean_ss = bool(ss_verified and not skate and clean_base)

    reasons = []
    if not sustained:
        reasons.append(
            f"SS mean bout L/R={mean_bout_L:.3f}/{mean_bout_R:.3f}s < {SS_MIN_BOUT_S}s"
        )
    if not clearance_ok:
        reasons.append(
            f"peak sole clear L/R={peak_clear_L:.4f}/{peak_clear_R:.4f} < {SS_MIN_PEAK_CLEAR_M}"
        )
    if not alt_ok:
        reasons.append(f"alt weak bouts L/R={n_bouts_L}/{n_bouts_R} flips={flips}")
    if not tip_ok:
        reasons.append(f"tip/fall tip={tip:.2f}s exp={stats.get('exploded')} flip={stats.get('flipped')}")
    if not dx_ok:
        reasons.append(f"dx={dx:.3f}<{SS_MIN_DX}")
    if not duty_ok:
        reasons.append(f"ss_duty={ss_duty:.3f}<{SS_MIN_DUTY}")
    if assist:
        reasons.append("assist ON")
    if freeze:
        reasons.append("freeze ON")
    if skate:
        reasons.append(
            f"skate stx={stx_L:.3f}/{stx_R:.3f} "
            f"p95={stats.get('stance_foot_vx_p95_L')}/{stats.get('stance_foot_vx_p95_R')}"
        )
    if hip_corr > -0.45:
        reasons.append(f"hip_corr={hip_corr:.3f}")
    if lead < 3:
        reasons.append(f"lead={lead}")

    return {
        "ss_duty_total": ss_duty,
        "ss_duty_L": float(np.mean(ssL)) if len(ssL) else 0.0,
        "ss_duty_R": float(np.mean(ssR)) if len(ssR) else 0.0,
        "ds_duty": float(np.mean(both)) if len(both) else 0.0,
        "flight_duty": float(np.mean(flight)) if len(flight) else 0.0,
        "ss_bouts_L": n_bouts_L,
        "ss_bouts_R": n_bouts_R,
        "ss_mean_bout_s_L": mean_bout_L,
        "ss_mean_bout_s_R": mean_bout_R,
        "ss_max_bout_s_L": max_bout_L,
        "ss_max_bout_s_R": max_bout_R,
        "ss_peak_clear_m_L": peak_clear_L,
        "ss_peak_clear_m_R": peak_clear_R,
        "ss_mean_clear_m_L": mean_clear_L,
        "ss_mean_clear_m_R": mean_clear_R,
        "ss_side_flips": flips,
        "ss_verified_step": ss_verified,
        "clean_walk_ss": clean_ss,
        "ss_fail_reasons": reasons,
        "tip_free_post_gait_max_s": tip,
        "dx_m": dx,
        "stance_vx_mean_L": stx_L,
        "stance_vx_mean_R": stx_R,
        "hip_q_corr_LR": hip_corr,
        "foot_lead_cycles": lead,
        "skate": skate,
        "assist": assist,
        "freeze": freeze,
        "clean_walk_base": clean_base,
    }


TRIALS = [
    dict(tag="SS00_E80_baseline", gait_t=0.88, hip_amp=0.118, ds=0.34, step_len=0.022,
         com_shift=0.27, com_shift_lead=0.23, knee_stance=0.40, knee_swing=0.75,
         com_z=0.225, hip_bias=0.06, plant_kd=95, cp=True, vik=True, vik_kp=0.35, vik_clip=0.03),
    dict(tag="SS11_ds24", gait_t=0.88, hip_amp=0.120, ds=0.24, step_len=0.024,
         com_shift=0.29, com_shift_lead=0.24, knee_stance=0.40, knee_swing=0.90,
         com_z=0.225, hip_bias=0.06, plant_kd=95, cp=True, vik=True, vik_kp=0.35, vik_clip=0.03),
    dict(tag="SS60_ds32_k80", gait_t=0.88, hip_amp=0.118, ds=0.32, step_len=0.022,
         com_shift=0.28, com_shift_lead=0.24, knee_stance=0.40, knee_swing=0.80,
         com_z=0.225, hip_bias=0.06, plant_kd=95, cp=True, vik=True, vik_kp=0.35, vik_clip=0.03),
    dict(tag="SS61_ds30_k82", gait_t=0.88, hip_amp=0.118, ds=0.30, step_len=0.022,
         com_shift=0.28, com_shift_lead=0.24, knee_stance=0.40, knee_swing=0.82,
         com_z=0.225, hip_bias=0.06, plant_kd=100, cp=True, vik=True, vik_kp=0.35, vik_clip=0.03),
    dict(tag="SS62_ds28_k85", gait_t=0.88, hip_amp=0.118, ds=0.28, step_len=0.022,
         com_shift=0.28, com_shift_lead=0.24, knee_stance=0.40, knee_swing=0.85,
         com_z=0.225, hip_bias=0.06, plant_kd=100, cp=True, vik=True, vik_kp=0.35, vik_clip=0.03),
    dict(tag="SS63_ds28_k85_pk110", gait_t=0.88, hip_amp=0.118, ds=0.28, step_len=0.022,
         com_shift=0.28, com_shift_lead=0.24, knee_stance=0.40, knee_swing=0.85,
         com_z=0.225, hip_bias=0.06, plant_kd=110, cp=True, vik=True, vik_kp=0.35, vik_clip=0.03),
    dict(tag="SS64_ds26_k88", gait_t=0.88, hip_amp=0.119, ds=0.26, step_len=0.023,
         com_shift=0.29, com_shift_lead=0.24, knee_stance=0.40, knee_swing=0.88,
         com_z=0.225, hip_bias=0.06, plant_kd=105, cp=True, vik=True, vik_kp=0.35, vik_clip=0.03),
    dict(tag="SS65_ds26_k88_shift30", gait_t=0.88, hip_amp=0.118, ds=0.26, step_len=0.022,
         com_shift=0.30, com_shift_lead=0.25, knee_stance=0.40, knee_swing=0.88,
         com_z=0.225, hip_bias=0.06, plant_kd=105, cp=True, vik=True, vik_kp=0.30, vik_clip=0.03),
    dict(tag="SS66_ds29_k84_T90", gait_t=0.90, hip_amp=0.117, ds=0.29, step_len=0.022,
         com_shift=0.285, com_shift_lead=0.24, knee_stance=0.40, knee_swing=0.84,
         com_z=0.225, hip_bias=0.06, plant_kd=100, cp=True, vik=True, vik_kp=0.35, vik_clip=0.03),
    dict(tag="SS67_ds27_k86_amp116", gait_t=0.88, hip_amp=0.116, ds=0.27, step_len=0.021,
         com_shift=0.285, com_shift_lead=0.24, knee_stance=0.40, knee_swing=0.86,
         com_z=0.225, hip_bias=0.055, plant_kd=100, cp=True, vik=True, vik_kp=0.35, vik_clip=0.03),
    dict(tag="SS68_ds28_sw70", gait_t=0.88, hip_amp=0.118, ds=0.28, step_len=0.022,
         com_shift=0.28, com_shift_lead=0.24, knee_stance=0.40, knee_swing=0.85,
         com_z=0.225, hip_bias=0.06, plant_kd=100, cp=True, vik=True, vik_kp=0.35, vik_clip=0.03,
         stance_sweep=0.70),
    dict(tag="SS69_ds25_k90_pk120", gait_t=0.88, hip_amp=0.118, ds=0.25, step_len=0.022,
         com_shift=0.29, com_shift_lead=0.25, knee_stance=0.40, knee_swing=0.90,
         com_z=0.225, hip_bias=0.06, plant_kd=120, cp=True, vik=True, vik_kp=0.30, vik_clip=0.025),
    # Extra plant / milder amp at ds28
    dict(tag="SS70_ds28_amp114_pk115", gait_t=0.88, hip_amp=0.114, ds=0.28, step_len=0.020,
         com_shift=0.28, com_shift_lead=0.24, knee_stance=0.40, knee_swing=0.85,
         com_z=0.225, hip_bias=0.055, plant_kd=115, cp=True, vik=True, vik_kp=0.30, vik_clip=0.025),
    dict(tag="SS71_ds31_k78", gait_t=0.88, hip_amp=0.118, ds=0.31, step_len=0.022,
         com_shift=0.275, com_shift_lead=0.23, knee_stance=0.40, knee_swing=0.78,
         com_z=0.225, hip_bias=0.06, plant_kd=100, cp=True, vik=True, vik_kp=0.35, vik_clip=0.03),
]



# Controls skate-fix around SS71 (bout≥0.12 verified; kill p95≤0.18)
SS71 = dict(gait_t=0.88, hip_amp=0.118, ds=0.31, step_len=0.022,
            com_shift=0.275, com_shift_lead=0.23, knee_stance=0.40, knee_swing=0.78,
            com_z=0.225, hip_bias=0.06, plant_kd=100, cp=True, vik=True, vik_kp=0.35, vik_clip=0.03)

def _csf(tag, **kw):
    d = dict(SS71)
    d.update(kw)
    d["tag"] = tag
    return d

CSF_TRIALS = [
    _csf("CSF00_SS71_baseline"),
    # Stronger plant
    _csf("CSF01_pk110", plant_kd=110),
    _csf("CSF02_pk120", plant_kd=120),
    _csf("CSF03_pk130", plant_kd=130),
    _csf("CSF04_pk140", plant_kd=140),
    # Stronger VIK ankle
    _csf("CSF05_vik45", vik_kp=0.45, plant_kd=110),
    _csf("CSF06_vik55", vik_kp=0.55, plant_kd=110),
    _csf("CSF07_vik65", vik_kp=0.65, plant_kd=120),
    _csf("CSF08_vik45_c025", vik_kp=0.45, vik_clip=0.025, plant_kd=110),
    _csf("CSF09_vik55_c04", vik_kp=0.55, vik_clip=0.04, plant_kd=120),
    # Stance sweep reduce (less stance travel → less p95 spike)
    _csf("CSF10_sw90", stance_sweep=0.90, plant_kd=110),
    _csf("CSF11_sw80", stance_sweep=0.80, plant_kd=110),
    _csf("CSF12_sw70", stance_sweep=0.70, plant_kd=110),
    _csf("CSF13_sw65_vik45", stance_sweep=0.65, vik_kp=0.45, plant_kd=120),
    # Vnull ON
    _csf("CSF14_vnull55", vnull=True, vnull_k=0.55, plant_kd=110),
    _csf("CSF15_vnull70", vnull=True, vnull_k=0.70, plant_kd=110),
    _csf("CSF16_vnull40", vnull=True, vnull_k=0.40, plant_kd=110),
    _csf("CSF17_vnull55_vik45", vnull=True, vnull_k=0.55, vik_kp=0.45, plant_kd=120),
    # Longer T / milder amp (quieter stance)
    _csf("CSF18_T92", gait_t=0.92, plant_kd=110),
    _csf("CSF19_T96", gait_t=0.96, plant_kd=110),
    _csf("CSF20_T100", gait_t=1.00, plant_kd=110),
    _csf("CSF21_amp114", hip_amp=0.114, step_len=0.020, plant_kd=110),
    _csf("CSF22_amp112", hip_amp=0.112, step_len=0.020, plant_kd=110),
    _csf("CSF23_amp116_T92", hip_amp=0.116, step_len=0.021, gait_t=0.92, plant_kd=110),
    # COM shift / lead
    _csf("CSF24_sh28_ld24", com_shift=0.28, com_shift_lead=0.24, plant_kd=110),
    _csf("CSF25_sh29_ld25", com_shift=0.29, com_shift_lead=0.25, plant_kd=110),
    _csf("CSF26_sh27_ld22", com_shift=0.27, com_shift_lead=0.22, plant_kd=110),
    _csf("CSF27_sh285_vik45", com_shift=0.285, com_shift_lead=0.24, vik_kp=0.45, plant_kd=120),
    # Combined quiet-stance recipes targeting p95
    _csf("CSF28_quietA", plant_kd=120, vik_kp=0.50, vik_clip=0.03, stance_sweep=0.80,
         hip_amp=0.114, step_len=0.020, com_shift=0.28, com_shift_lead=0.24),
    _csf("CSF29_quietB", plant_kd=125, vik_kp=0.55, stance_sweep=0.75, vnull=True, vnull_k=0.50,
         hip_amp=0.114, step_len=0.020, gait_t=0.92, com_shift=0.28, com_shift_lead=0.24),
    _csf("CSF30_quietC", plant_kd=130, vik_kp=0.45, stance_sweep=0.85, gait_t=0.94,
         hip_amp=0.115, step_len=0.021, com_shift=0.28, com_shift_lead=0.235, knee_swing=0.80),
    _csf("CSF31_ds305_pk120", ds=0.305, plant_kd=120, vik_kp=0.45, hip_amp=0.116, step_len=0.021),
    _csf("CSF32_ds315_pk120", ds=0.315, plant_kd=120, vik_kp=0.45, stance_sweep=0.85),
    _csf("CSF33_ds31_k76_pk120", knee_swing=0.76, plant_kd=120, vik_kp=0.50, stance_sweep=0.80),
    _csf("CSF34_ds31_k80_pk115", knee_swing=0.80, plant_kd=115, vik_kp=0.45, hip_amp=0.116, step_len=0.021),
    _csf("CSF35_fmax25", plant_kd=120, plant_fmax=25.0, vik_kp=0.50),
    _csf("CSF36_fmax30_vik55", plant_kd=130, plant_fmax=30.0, vik_kp=0.55, stance_sweep=0.80),
    _csf("CSF37_vnull_sw75_T94", vnull=True, vnull_k=0.60, stance_sweep=0.75, gait_t=0.94,
         plant_kd=120, vik_kp=0.45, hip_amp=0.114, step_len=0.020, com_shift=0.28),
    _csf("CSF38_amp110_pk130", hip_amp=0.110, step_len=0.019, plant_kd=130, vik_kp=0.55,
         stance_sweep=0.80, com_shift=0.28, com_shift_lead=0.24),
    _csf("CSF39_ds30_pk125_sw80", ds=0.30, plant_kd=125, stance_sweep=0.80, vik_kp=0.50,
         knee_swing=0.80, com_shift=0.28, com_shift_lead=0.24),
    _csf("CSF40_ds312_amp114_pk125", ds=0.312, hip_amp=0.114, step_len=0.020, plant_kd=125,
         vik_kp=0.48, stance_sweep=0.82, com_shift=0.278, com_shift_lead=0.235),
    # Micro frontier around CSF34 (bout 0.123/0.119 no-skate)
    _csf("CSF50_k80_pk115", hip_amp=0.116, step_len=0.021, knee_swing=0.80, plant_kd=115, vik_kp=0.45),
    _csf("CSF51_k81_pk115", hip_amp=0.116, step_len=0.021, knee_swing=0.81, plant_kd=115, vik_kp=0.45),
    _csf("CSF52_k82_pk115", hip_amp=0.116, step_len=0.021, knee_swing=0.82, plant_kd=115, vik_kp=0.45),
    _csf("CSF53_k80_ds305", hip_amp=0.116, step_len=0.021, knee_swing=0.80, ds=0.305, plant_kd=115, vik_kp=0.45),
    _csf("CSF54_k80_ds308", hip_amp=0.116, step_len=0.021, knee_swing=0.80, ds=0.308, plant_kd=115, vik_kp=0.45),
    _csf("CSF55_k81_ds308", hip_amp=0.116, step_len=0.021, knee_swing=0.81, ds=0.308, plant_kd=115, vik_kp=0.45),
    _csf("CSF56_k80_pk112", hip_amp=0.116, step_len=0.021, knee_swing=0.80, plant_kd=112, vik_kp=0.45),
    _csf("CSF57_k80_pk118", hip_amp=0.116, step_len=0.021, knee_swing=0.80, plant_kd=118, vik_kp=0.48),
    _csf("CSF58_k81_vik48", hip_amp=0.116, step_len=0.021, knee_swing=0.81, plant_kd=115, vik_kp=0.48),
    _csf("CSF59_k82_vik50_pk118", hip_amp=0.116, step_len=0.021, knee_swing=0.82, plant_kd=118, vik_kp=0.50),
    _csf("CSF60_k80_amp117", hip_amp=0.117, step_len=0.021, knee_swing=0.80, plant_kd=115, vik_kp=0.45),
    _csf("CSF61_k81_sh278", hip_amp=0.116, step_len=0.021, knee_swing=0.81, plant_kd=115, vik_kp=0.45,
         com_shift=0.278, com_shift_lead=0.235),
    _csf("CSF62_k82_ds31_sw90", hip_amp=0.116, step_len=0.021, knee_swing=0.82, plant_kd=115, vik_kp=0.45,
         stance_sweep=0.90),
    _csf("CSF63_k83_pk115", hip_amp=0.116, step_len=0.021, knee_swing=0.83, plant_kd=115, vik_kp=0.45),
    _csf("CSF64_k80_ds302_pk115", hip_amp=0.116, step_len=0.021, knee_swing=0.80, ds=0.302, plant_kd=115, vik_kp=0.45),
]



def build_cmd(trial: dict, *, video: bool, duration: float, out_mp4: Path, stats_out: Path) -> list[str]:
    cmd = [
        str(PY), str(WALK),
        "--model", str(PLANT),
        "--duration", str(duration),
        "--tag", trial["tag"],
        "--gait-t", str(trial["gait_t"]),
        "--hip-amp", str(trial["hip_amp"]),
        "--ds", str(trial["ds"]),
        "--step-len", str(trial["step_len"]),
        "--com-shift", str(trial["com_shift"]),
        "--com-shift-lead", str(trial["com_shift_lead"]),
        "--knee-stance", str(trial["knee_stance"]),
        "--knee-swing", str(trial["knee_swing"]),
        "--com-z", str(trial["com_z"]),
        "--hip-bias", str(trial["hip_bias"]),
        "--plant-kd", str(trial["plant_kd"]),
        "--out", str(out_mp4),
        "--stats-out", str(stats_out),
    ]
    if not video:
        cmd.append("--no-video")
    if trial.get("cp"):
        cmd.append("--cp-swing")
    if trial.get("vik"):
        cmd += ["--stance-vik", "--vik-ankle"]
        if trial.get("vik_kp") is not None:
            cmd += ["--vik-kp", str(trial["vik_kp"])]
        if trial.get("vik_clip") is not None:
            cmd += ["--vik-clip", str(trial["vik_clip"])]
    if trial.get("stance_sweep") is not None:
        cmd += ["--stance-sweep", str(trial["stance_sweep"])]
    if trial.get("vnull"):
        cmd.append("--vnull")
        if trial.get("vnull_k") is not None:
            cmd += ["--vnull-k", str(trial["vnull_k"])]
    if trial.get("plant_fmax") is not None:
        cmd += ["--plant-fmax", str(trial["plant_fmax"])]
    if trial.get("residual") and str(trial["residual"]).lower() not in ("none", "off", ""):
        cmd += ["--residual", str(trial["residual"])]
        if trial.get("residual_gain") is not None:
            cmd += ["--residual-gain", str(trial["residual_gain"])]
    if trial.get("wbc_stance"):
        cmd.append("--wbc-stance")
        if trial.get("wbc_mu") is not None:
            cmd += ["--wbc-mu", str(trial["wbc_mu"])]
        if trial.get("wbc_scrub") is not None:
            cmd += ["--wbc-scrub", str(trial["wbc_scrub"])]
        if trial.get("wbc_trim_clip") is not None:
            cmd += ["--wbc-trim-clip", str(trial["wbc_trim_clip"])]
        if trial.get("wbc_tau_scale") is not None:
            cmd += ["--wbc-tau-scale", str(trial["wbc_tau_scale"])]
        if trial.get("wbc_w_scrub") is not None:
            cmd += ["--wbc-w-scrub", str(trial["wbc_w_scrub"])]
    if trial.get("fric") is not None:
        cmd += ["--fric", str(trial["fric"])]
    if trial.get("hybrid_mpc"):
        cmd.append("--hybrid-mpc")
        if trial.get("mpc_n") is not None:
            cmd += ["--mpc-n", str(trial["mpc_n"])]
        if trial.get("mpc_u_max") is not None:
            cmd += ["--mpc-u-max", str(trial["mpc_u_max"])]
        if trial.get("mpc_apply_thr") is not None:
            cmd += ["--mpc-apply-thr", str(trial["mpc_apply_thr"])]
        if trial.get("mpc_disable_above_t") is not None:
            cmd += ["--mpc-disable-above-t", str(trial["mpc_disable_above_t"])]
    return cmd


def run_trial(trial: dict, *, video: bool, duration: float = 9.0) -> dict:
    ITER.mkdir(parents=True, exist_ok=True)
    tag = trial["tag"]
    stats_out = ITER / f"{tag}.json"
    vid_dir = ITER / "ss_runs"
    vid_dir.mkdir(parents=True, exist_ok=True)
    out_mp4 = (vid_dir / f"{tag}.mp4") if video else (ITER / f"{tag}_novid.mp4")
    csv_shared = ITER / "ainex_walk_timeseries.csv"

    cmd = build_cmd(trial, video=video, duration=duration, out_mp4=out_mp4, stats_out=stats_out)
    env = dict(os.environ)
    env["MUJOCO_GL"] = os.environ.get("MUJOCO_GL", "glfw")
    print(f"\n=== RUN {tag} video={video} ===", flush=True)
    print(" ".join(cmd), flush=True)
    r = subprocess.run(cmd, cwd=str(ROOT), env=env, capture_output=True, text=True)
    if r.returncode != 0:
        print((r.stdout or "")[-1200:])
        print((r.stderr or "")[-600:])

    csv_tag = ITER / f"{tag}_timeseries.csv"
    # Prefer per-stats timeseries (written next to --stats-out), then out_dir, then shared
    candidates = [
        ITER / f"{tag}_timeseries.csv",
        out_mp4.parent / "ainex_walk_timeseries.csv",
        csv_shared,
    ]
    src = next((c for c in candidates if c.exists() and c.stat().st_mtime >= (stats_out.stat().st_mtime - 5 if stats_out.exists() else 0)), None)
    # Prefer newest among existing
    existing = [c for c in candidates if c.exists()]
    if existing:
        src = max(existing, key=lambda p: p.stat().st_mtime)
        if src.resolve() != csv_tag.resolve():
            shutil.copy2(src, csv_tag)
        # refresh shared
        if src.resolve() != csv_shared.resolve():
            shutil.copy2(src, csv_shared)

    if not stats_out.exists():
        return {"tag": tag, "error": f"rc={r.returncode} no_stats", "params": trial,
                "stderr": (r.stderr or "")[-400:]}

    stats = json.loads(stats_out.read_text())
    ss = analyze_ss(csv_tag if csv_tag.exists() else csv_shared, stats)
    out = {
        "tag": tag, "params": trial, "stats_path": str(stats_out),
        "csv_path": str(csv_tag), "mp4": str(out_mp4) if out_mp4.exists() else None,
        "rc": r.returncode, **ss,
    }
    for k in (
        "tip_free_post_gait_max_s", "dx_m", "stance_foot_vx_mean_L", "stance_foot_vx_mean_R",
        "stance_foot_vx_p95_L", "stance_foot_vx_p95_R", "hip_q_ptp", "hip_q_corr_LR",
        "foot_lead_cycles", "contact_duty_L", "contact_duty_R", "lift_L_m", "lift_R_m",
        "skate", "clean_walk_claim", "balance_assist", "stance_freeze", "exploded", "flipped",
    ):
        if k in stats:
            out[k] = stats[k]
    (ITER / f"{tag}_ss.json").write_text(json.dumps(out, indent=2))
    print(
        f"[{tag}] tip={out.get('tip_free_post_gait_max_s')} dx={out.get('dx_m'):+.3f} "
        f"stx={out.get('stance_foot_vx_mean_L')}/{out.get('stance_foot_vx_mean_R')} "
        f"p95={out.get('stance_foot_vx_p95_L')}/{out.get('stance_foot_vx_p95_R')} "
        f"ss_duty={out.get('ss_duty_total', 0):.3f} "
        f"bout={out.get('ss_mean_bout_s_L', 0):.3f}/{out.get('ss_mean_bout_s_R', 0):.3f} "
        f"clear={out.get('ss_peak_clear_m_L', 0):.3f}/{out.get('ss_peak_clear_m_R', 0):.3f} "
        f"ss_ok={out.get('ss_verified_step')} clean_ss={out.get('clean_walk_ss')} "
        f"skate={out.get('skate')}",
        flush=True,
    )
    if out.get("ss_fail_reasons"):
        print(f"  fails: {out['ss_fail_reasons']}", flush=True)
    return out


def write_note(results: list[dict], best: dict | None) -> None:
    lines = [
        "# SS Step Hardware Note — M2 145×86",
        "",
        "**When:** 2026-09-27 evening Europe/London (BST)",
        "**Role:** Hardware-supporting Controls (additive `scripts/ss_step_ainex.py`)",
        "**Plant:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (HX ±2.1/±0.7)",
        "**Constraint:** assist OFF, freeze OFF, Path A spend FROZEN. No soft-pass.",
        "",
        "## Goal",
        "",
        "Verified **single-support** bipedal step: swing foot contact≈0 / sole clearance>0",
        "while stance planted; L/R alternate; **not** E80 plant-and-shift shuffle alone.",
        "",
        "### SS verification gates",
        "",
        "| Gate | Threshold |",
        "|------|-----------|",
        f"| SS **mean** bout duration L **and** R | ≥ {SS_MIN_BOUT_S} s |",
        f"| Peak sole clearance during SS L **and** R | ≥ {SS_MIN_PEAK_CLEAR_M} m |",
        f"| SS XOR duty | ≥ {SS_MIN_DUTY} |",
        f"| Alternating SS bouts | ≥ {SS_MIN_CYCLES} each side |",
        f"| tip_free_post_gait | ≥ {SS_MIN_TIP} s |",
        f"| dx | ≥ {SS_MIN_DX} m |",
        "| hip_corr / foot_lead | ≤ −0.45 / ≥ 3 |",
        "| assist / freeze | OFF |",
        "| `clean_walk_ss` | SS verified **and** no skate **and** base clean_walk |",
        "",
        "Sole clearance = foot-body z − 0.026 m (M2 box).",
        "",
    ]

    clean_ss = [r for r in results if r.get("clean_walk_ss")]
    verified = [r for r in results if r.get("ss_verified_step")]

    lines.append("## Verdict")
    lines.append("")
    if clean_ss:
        b = best if best and best.get("clean_walk_ss") else clean_ss[0]
        lines.append(f"- **`clean_walk` for single-support: TRUE** — tag `{b['tag']}`")
        lines.append(f"- MP4: `{b.get('mp4')}`")
    elif verified:
        b = verified[0]
        lines.append(
            f"- SS metrics can clear tip/dx/alt/clear (e.g. `{b['tag']}`) but **skate remains** "
            "→ `clean_walk_ss` FALSE"
        )
        lines.append("- **`clean_walk` for single-support: FALSE**")
        lines.append(
            "- **Hard falsifier:** under HX ±2.1 on M2 145×86, open-loop CPG + plant/CoP/VIK "
            "that achieve sustained SS mean bout ≥0.12 s trade into stance skate (p95>|vx| or "
            "mean>|vx|), tip, or dx/alt loss. E80 QS keeps no-skate only with brief SS (~0.08 s)."
        )
    else:
        lines.append("- **`clean_walk` for single-support: FALSE**")
        lines.append(
            "- **Hard falsifier:** no trial jointly hit sustained SS mean bout ≥0.12 s + "
            "clearance + tip≥5 + dx≥0.05 + L/R alt + no-skate under HX clips / assist OFF / freeze OFF."
        )
    lines.append("")

    lines.append("## Metrics table")
    lines.append("")
    lines.append(
        "| Tag | tip | dx | stx L/R | p95 L/R | ss_duty | bout L/R s | "
        "peak clear | bouts | skate | ss_ok | clean_ss |"
    )
    lines.append(
        "|-----|-----|----|---------|---------|---------|------------|"
        "------------|-------|-------|-------|----------|"
    )
    for r in results:
        if r.get("error") and r.get("dx_m") is None:
            lines.append(f"| `{r['tag']}` | ERR | | | | | | | | | | |")
            continue
        tip = r.get("tip_free_post_gait_max_s")
        dx = r.get("dx_m")
        tip_s = f"{tip:.2f}" if isinstance(tip, (int, float)) else "?"
        dx_s = f"{dx:+.3f}" if isinstance(dx, (int, float)) else "?"
        stx = f"{r.get('stance_foot_vx_mean_L', float('nan')):.3f}/{r.get('stance_foot_vx_mean_R', float('nan')):.3f}"
        p95 = f"{r.get('stance_foot_vx_p95_L', float('nan')):.3f}/{r.get('stance_foot_vx_p95_R', float('nan')):.3f}"
        lines.append(
            f"| `{r['tag']}` | {tip_s} | {dx_s} | {stx} | {p95} | "
            f"{r.get('ss_duty_total', 0):.3f} | "
            f"{r.get('ss_mean_bout_s_L', 0):.3f}/{r.get('ss_mean_bout_s_R', 0):.3f} | "
            f"{r.get('ss_peak_clear_m_L', 0):.3f}/{r.get('ss_peak_clear_m_R', 0):.3f} | "
            f"{r.get('ss_bouts_L', 0)}/{r.get('ss_bouts_R', 0)} | "
            f"{r.get('skate')} | {r.get('ss_verified_step')} | {r.get('clean_walk_ss')} |"
        )
    lines.append("")

    lines.append("## Best candidate")
    lines.append("")
    if best and not best.get("error"):
        lines.append(f"- Tag: `{best['tag']}`")
        lines.append(f"- SS verified: {best.get('ss_verified_step')}")
        lines.append(f"- clean_walk_ss: {best.get('clean_walk_ss')}")
        tip = best.get("tip_free_post_gait_max_s")
        dx = best.get("dx_m")
        lines.append(
            f"- tip / dx / stx: {tip} / "
            f"{(dx if isinstance(dx, (int, float)) else float('nan')):+.3f} / "
            f"{best.get('stance_foot_vx_mean_L')}/{best.get('stance_foot_vx_mean_R')}"
        )
        lines.append(
            f"- SS duty / mean bout L/R / peak clear: "
            f"{best.get('ss_duty_total', 0):.3f} / "
            f"{best.get('ss_mean_bout_s_L', 0):.3f}/{best.get('ss_mean_bout_s_R', 0):.3f} / "
            f"{best.get('ss_peak_clear_m_L', 0):.3f}/{best.get('ss_peak_clear_m_R', 0):.3f}"
        )
        lines.append(f"- Fail reasons: {best.get('ss_fail_reasons')}")
        lines.append(
            f"- Artifacts: `{best.get('stats_path')}`, `{best.get('csv_path')}`, `{best.get('mp4')}`"
        )
        if best.get("mp4_canonical"):
            lines.append(f"- Canonical MP4: `{best['mp4_canonical']}`")
    else:
        lines.append("- None.")
    lines.append("")
    lines.append("## Explicit non-claims")
    lines.append("")
    lines.append("- Does **not** thaw Path A spend.")
    lines.append("- Does **not** claim E80 QS shuffle is Gate D/E Dave unlock.")
    lines.append("- Stance-freeze / assist were **not** used as product claim.")
    lines.append("- Additive script only — Controls `walk_gait_ainex.py` basins untouched.")
    lines.append("")
    lines.append("*Generated by `scripts/ss_step_ainex.py`.*")
    NOTE.write_text("\n".join(lines) + "\n")
    print(f"Wrote {NOTE}")



def write_csf_note(results: list[dict], best: dict | None) -> None:
    """Controls skate-kill note — does not clobber Hardware SS_STEP_HARDWARE_NOTE.md."""
    csf_note = ITER / "CSF_SKATE_KILL_NOTE.md"
    lines = [
        "# Controls skate-kill (CSF) — Phase F / Gate D",
        "",
        "**When:** Sun 27 Sep 2026 evening Europe/London (BST)",
        "**Role:** Founding Controls — kill skate on Hardware SS71 neighborhood",
        "**Plant:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (locked)",
        "**Baseline:** `BEST_SS_step` / `SS71_ds31_k78` — ss_verified TRUE; clean_walk_ss FALSE (p95 R=0.183)",
        "**Constraint:** assist OFF, freeze OFF, Path A FROZEN. CoP not used as SS proof.",
        "",
        "## Goal",
        "",
        "Hold SS mean bout L/R ≥0.12 s + sole clear ≥0.012 + L/R alt + tip≥8 + dx≥0.05,",
        "while stance |vx| mean≤0.08 **and** p95≤0.18 both feet → `clean_walk_ss` TRUE.",
        "",
        "## Verdict",
        "",
    ]
    clean = [r for r in results if r.get("clean_walk_ss")]
    verified = [r for r in results if r.get("ss_verified_step")]
    tip_dx_alt = [
        r for r in results
        if (r.get("tip_free_post_gait_max_s") or 0) >= 8
        and (r.get("dx_m") or 0) >= 0.05
        and (r.get("hip_q_corr_LR") or 0) <= -0.45
        and (r.get("foot_lead_cycles") or 0) >= 3
    ]
    bout_ok = [
        r for r in results
        if (r.get("ss_mean_bout_s_L") or 0) >= 0.12 and (r.get("ss_mean_bout_s_R") or 0) >= 0.12
    ]
    no_skate = [r for r in results if r.get("skate") is False]
    joint = [
        r for r in results
        if r.get("ss_verified_step") and not r.get("skate")
        and (r.get("ss_mean_bout_s_L") or 0) >= 0.12
        and (r.get("ss_mean_bout_s_R") or 0) >= 0.12
        and (r.get("tip_free_post_gait_max_s") or 0) >= 8
        and (r.get("dx_m") or 0) >= 0.05
    ]
    if clean:
        b = clean[0]
        lines.append(f"- **`clean_walk_ss`: TRUE** — `{b['tag']}`")
        lines.append(f"- MP4: `{b.get('mp4')}`")
    else:
        lines.append("- **`clean_walk_ss`: FALSE**")
        lines.append(
            f"- Counts: tip≥8+dx+alt={len(tip_dx_alt)}, bout≥0.12 both={len(bout_ok)}, "
            f"no-skate={len(no_skate)}, ss_verified={len(verified)}, "
            f"joint(ss+bout+no-skate+tip+dx)={len(joint)}"
        )
        if not joint:
            lines.append(
                "- **Hard block (Controls):** no CSF trial jointly holds mean bout≥0.12 + "
                "no-skate (mean≤0.08 and p95≤0.18) + tip/dx/alt under HX ±2.1 on M2 145×86. "
                "Next lever: Hardware contact geom / friction A/B (or Path A thaw / residual)."
            )
        else:
            lines.append(
                "- Joint physical gates met but `clean_walk_ss` still false "
                "(likely base clean_walk_claim false) — inspect `ss_fail_reasons`."
            )
    lines += [
        "",
        "## Results (ranked)",
        "",
        "| Tag | tip | dx | stx L/R | p95 L/R | bout L/R | clear | skate | ss_ok | clean_ss |",
        "|-----|-----|----|---------|---------|----------|-------|-------|-------|----------|",
    ]
    for r in sorted(results, key=score, reverse=True):
        if r.get("dx_m") is None:
            continue
        tip = r.get("tip_free_post_gait_max_s")
        tip_s = f"{tip:.2f}" if tip is not None else "—"
        dx = r.get("dx_m") or 0.0
        lines.append(
            f"| `{r.get('tag')}` | {tip_s} | {dx:+.3f} | "
            f"{r.get('stance_foot_vx_mean_L', float('nan')):.3f}/{r.get('stance_foot_vx_mean_R', float('nan')):.3f} | "
            f"{r.get('stance_foot_vx_p95_L', float('nan')):.3f}/{r.get('stance_foot_vx_p95_R', float('nan')):.3f} | "
            f"{r.get('ss_mean_bout_s_L', 0):.3f}/{r.get('ss_mean_bout_s_R', 0):.3f} | "
            f"{r.get('ss_peak_clear_m_L', 0):.3f}/{r.get('ss_peak_clear_m_R', 0):.3f} | "
            f"{r.get('skate')} | {r.get('ss_verified_step')} | {r.get('clean_walk_ss')} |"
        )
    lines += [
        "",
        f"Best by score: `{best.get('tag') if best else None}`",
        "",
        "*Generated by Founding Controls via `scripts/ss_step_ainex.py --csf-sweep`.*",
        "",
    ]
    csf_note.write_text("\n".join(lines))
    print(f"Wrote {csf_note}")


def score(r: dict) -> float:
    if r.get("error") and r.get("dx_m") is None:
        return -1e9
    s = 0.0
    if r.get("clean_walk_ss"):
        s += 1000
    if r.get("ss_verified_step"):
        s += 500
    s += 200 * min(r.get("ss_mean_bout_s_L", 0), r.get("ss_mean_bout_s_R", 0))
    s += 80 * min(r.get("ss_peak_clear_m_L", 0), r.get("ss_peak_clear_m_R", 0))
    tip = r.get("tip_free_post_gait_max_s") or 0
    dx = r.get("dx_m") or 0
    s += 3 * min(tip, 9)
    s += 30 * max(0.0, min(dx, 0.3))
    if r.get("skate"):
        s -= 80
    else:
        s += 40
    if tip < 5:
        s -= 120
    if r.get("exploded") or r.get("flipped"):
        s -= 200
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--csf-sweep", action="store_true", help="Controls skate-fix around SS71")
    ap.add_argument("--tag", type=str, default=None)
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--duration", type=float, default=9.0)
    ap.add_argument("--video-best", action="store_true")
    args = ap.parse_args()

    pool = CSF_TRIALS if args.csf_sweep else TRIALS
    trials = pool
    if args.tag:
        trials = [t for t in (TRIALS + CSF_TRIALS) if t["tag"] == args.tag]
        if not trials:
            print(f"Unknown tag {args.tag}", file=sys.stderr)
            sys.exit(2)
    if not args.sweep and not args.csf_sweep and not args.tag:
        args.sweep = True

    results = []
    video = bool(args.video) and not args.sweep
    for t in trials:
        results.append(run_trial(t, video=video, duration=args.duration))

    ranked = sorted(results, key=score, reverse=True)
    best = ranked[0] if ranked else None

    if args.video_best and best and best.get("params"):
        print("\n=== VIDEO BEST ===", flush=True)
        best_v = run_trial(best["params"], video=True, duration=args.duration)
        results = [best_v if r.get("tag") == best_v.get("tag") else r for r in results]
        best = best_v
        if best.get("mp4") and Path(best["mp4"]).exists():
            canon = ITER / "BEST_SS_step.mp4"
            shutil.copy2(best["mp4"], canon)
            best["mp4_canonical"] = str(canon)
            shutil.copy2(ITER / f"{best['tag']}_ss.json", ITER / "BEST_SS_step.json")

    if not args.csf_sweep:
        write_note(results, best)
    else:
        write_csf_note(results, best)
    summary = {
        "best_tag": best.get("tag") if best else None,
        "clean_walk_ss": bool(best.get("clean_walk_ss")) if best else False,
        "ss_verified_step": bool(best.get("ss_verified_step")) if best else False,
        "note": str(NOTE),
        "mp4": best.get("mp4_canonical") or best.get("mp4") if best else None,
        "results": [
            {k: r.get(k) for k in (
                "tag", "tip_free_post_gait_max_s", "dx_m",
                "stance_foot_vx_mean_L", "stance_foot_vx_mean_R",
                "stance_foot_vx_p95_L", "stance_foot_vx_p95_R",
                "ss_duty_total", "ss_mean_bout_s_L", "ss_mean_bout_s_R",
                "ss_peak_clear_m_L", "ss_peak_clear_m_R",
                "ss_verified_step", "clean_walk_ss", "skate", "ss_fail_reasons", "mp4",
            )}
            for r in results
        ],
    }
    sum_name = "CSF_SKATE_KILL_SUMMARY.json" if args.csf_sweep else "SS_STEP_SUMMARY.json"
    (ITER / sum_name).write_text(json.dumps(summary, indent=2))
    # Promote clean_walk_ss video to canonical BEST_SS_clean_walk
    if best and best.get("clean_walk_ss") and best.get("mp4") and Path(best["mp4"]).exists():
        shutil.copy2(best["mp4"], ITER / "BEST_SS_clean_walk.mp4")
        shutil.copy2(ITER / f"{best['tag']}_ss.json", ITER / "BEST_SS_clean_walk.json")
        print(f"Promoted BEST_SS_clean_walk from {best['tag']}")
    print(json.dumps({k: summary[k] for k in (
        "best_tag", "clean_walk_ss", "ss_verified_step", "note", "mp4"
    )}, indent=2))
    print(f"Wrote {ITER / sum_name}")


if __name__ == "__main__":
    main()
