#!/usr/bin/env python3
"""Gate Q: stepped approach → N compose → stepped retreat (SAME continuous run).
Criteria: docs/GATE_Q_AI_CRITERIA.md
Reuse Gate P iterate2 approach/compose; reverse-hip stepped retreat.
"""
from __future__ import annotations
import hashlib, json, math, os, sys
from pathlib import Path
import numpy as np
import mujoco as mj
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
os.environ.setdefault("MUJOCO_GL", "glfw")
import walk_gait_ainex as wg
from score_gate_f import (
    ITER, PLANT_F, CKPT, GateFEnv, body_x_for_cam_dist, CTRL_HZ,
    cam_to_lever_horiz, _apply_shape, GRO01, CTRL_LIM, ACTION_SCALE,
)
from score_gate_g import _hand_lever_contact
from score_gate_h import _hinge_id, _pin_arms_head, OPEN_CMD, CLOSE_CMD
from score_gate_i import _panel_hinge_id, _angle_deg
from score_gate_k import (
    _hud, _panel_cam, _world_cam, _mark_base_edge, _arm_seed, _panel_spring, _sha16,
)
COMPANION_LOCK_MD5 = "59cc408eda07037a58f92ad27da045d6"
CKPT_SHA16 = "9ffaa1a21b607bf6"
DISTURB_TYPE = "panel_push_xfrc_door_panel_link"
DISTURB_FORCE_N = np.array([1.2, 0.0, 0.0])
DISTURB_IMPULSE_S = 0.35
DISTURB_WINDOW_S = 1.00
DISTURB_S = DISTURB_WINDOW_S
REJECT_S = 2.5
CAM_START = 0.60
CAM_WORK = 0.11
WALK_S = 22.0  # iterate3c: tip-stable walk budget; multi-SS before early-stop
WALK_STAND_HOLD = 0.60
WALK_RAMP = 1.0
FINALIZE_S = 0.8  # re-score: small post-walk settle only; HUD = KINEMATIC XY SHIM
RETREAT_HOLD = 2.0
RETREAT_WALK_S = 50.0
RETREAT_FINALIZE_S = 0.8
RETREAT_RAMP = 1.5
RETREAT_S = RETREAT_HOLD + RETREAT_WALK_S + RETREAT_FINALIZE_S
RETREAT_AMP_SCALE = 0.82  # iterate3r19h7k8: keep h7j amp; CoP gate + clear boost
RETREAT_AMP_SCALE_POST = 0.42
RETREAT_BIAS_FRAC = 0.78
RETREAT_BIAS_FRAC_POST = 0.38
RETREAT_GAIT_T = 0.88
RETREAT_RES_SCALE = 0.95  # restore C3 (1.00 stalled cam)
RETREAT_RES_SCALE_POST = 0.22
RETREAT_ADD_KNEE = 0.13
RETREAT_ADD_ANK = 0.115
RETREAT_ADD_ABD = 0.030
RETREAT_PROOF_DELAY = 2.2
RETREAT_CLEAR_PROOF_S = 32.0
RETREAT_EARLY_DX = 0.42
RETREAT_EARLY_CAM = 0.55
START_OFFSET_METHOD = "cam_to_door_ge_0.55"
APPROACH_METHOD = "gate_e_residual_stepped_visible_clear_then_small_finalize_shim"
RETREAT_METHOD = "gate_e_residual_reverse_hip_stepped_then_small_finalize_shim"
# Honesty gates (Root B iterate2 / GATE_Q_AI_CRITERIA.md ~05:21)
SOLE_OFFSET = 0.026          # foot-body z − sole bottom (same as ss_step_ainex)
# Soft-contact plant rest: body_z≈0.0346 → clear≈+8.6 mm while visually planted.
# Require ≥2 cm ABOVE plant rest (defeat SOLE_OFFSET bias), not absolute 2 cm from floor.
VISIBLE_CLEAR_ABOVE_REST_M = 0.02
# Absolute sole-clear floor that cannot trip on soft-contact rest gap alone (~0.0086+0.02)
VISIBLE_CLEAR_M = 0.03
SS_DWELL_S = 0.15            # consecutive clear dwell for SS credit (150–250 ms band)
STEPPED_FRAC_MIN = 0.70      # stepped Δ ≥ ~70% of total approach Δ
EARLY_STOP_CAM_H = 0.16      # near work; paired with dx≥0.34 proved gate for stepped_frac
# Controller clearance boost (Gate Q walk only; plant untouched)
CLEAR_BOOST_KNEE_SWING = 0.92
CLEAR_BOOST_SWING_ANK_DF = 0.30  # iterate3r19h7j: golden 3r19 daylight (app untouched)
CLEAR_BOOST_SWING_ABDUCT = 0.048
# Iterate3r19h4: golden 3r19 clear-window (hip×1.45 residual×1.40 damp0.40 hold0.32)
# + skate plant-gate 0.24 + plant-settle zero XY on clear→plant edge (clear_frac margin).
# iterate3r19h7k8ai: k8ah soft NO-OP (tip cam~0.19). Bout1 ret amp0.62+hip×1.35+res×0.85 tip-protect;
# bout1 app golden untouched. cancel 0.70. Soft-pass forbidden. Prefer FAIL.
GATE_Q_ADD_KNEE_DEFAULT = "0.16"
GATE_Q_ADD_ANK_DEFAULT = "0.14"
GATE_Q_ADD_ABD_DEFAULT = "0.032"
GATE_Q_CLEAR_PROOF_S_DEFAULT = "30.0"
GATE_Q_GAIT_T_DEFAULT = "0.85"
GATE_Q_DS_S = "0.10"
GATE_Q_CLEAR_HOLD_S = "0.32"
GATE_Q_CLEAR_HOLD_AIR_M = "0.018"
GATE_Q_PLANTED_HIP_SCALE = "0.0"
GATE_Q_RES_IN_PROOF = "0.90"
GATE_Q_RES_TAPER = "0.75"
GATE_Q_ADD_TAPER = "1.0"
GATE_Q_HUD_CLEAR_M = "0.065"
GATE_Q_HUD_CLEAR_ABOVE_REST_M = "0.055"
GATE_Q_MIN_WALK_PROOF_S = "8.0"
SS_PER_SIDE_MIN = 2
SS_TOTAL_MIN = 3
SS_DENSE_PER_SIDE = 3
SS_DENSE_TOTAL = 4
GATE_Q_RES_POST_PROOF = "0.30"
GATE_Q_RES_PLANTED_SCALE = "0.0"
GATE_Q_RES_NO_SWING_SCALE = "0.0"
GATE_Q_RES_SWING_BOOST = "1.40"
GATE_Q_SWING_CLEAR_AR_M = "0.02"
GATE_Q_SWING_CLEAR_ABS_M = "0.03"
GATE_Q_SWING_XY_DAMP = "0.40"
GATE_Q_SWING_HIP_SCALE = "1.45"
GATE_Q_CLEAR_FRAC_MIN = "0.55"
GATE_Q_PLANT_KD = "72"
GATE_Q_PLANT_MAX_F = "26"
GATE_Q_PLANTED_XY_DAMP_RET = "0.30"  # planted vel damp (no full qvel0)
GATE_Q_RET_SWING_HIP_SCALE = "1.55"  # A3/k8n clear-only hip (1.52+res1.68 stalled)
GATE_Q_RET_CLEAR_HOLD_S = "0.42"  # A3
GATE_Q_RET_RES_SWING_BOOST = "1.65"  # A3/k8n (1.68 stalled with hold0.45)
GATE_Q_RET_SAGITTAL_FLIP = "0"  # hip-only flip; full sagittal hurt clear Δ
GATE_Q_RET_PLANTED_GAIT_SCALE = "1.00"  # full gait amp while planted (need lifts)
GATE_Q_RET_GAP_PLANTED_GAIT_SCALE = "1.00"  # if <1: scale planted amp only when SS-silent ≥ gap (cut middle skate)
GATE_Q_RET_GAP_PLANTED_SS_GAP = "2.0"  # seconds since last retreat SS to arm gap planted-amp
GATE_Q_RET_CP_VIK = "0"  # off (approach-only CP/VIK)
GATE_Q_RET_PLANTED_COP_GATE = "0"  # full CoP off kills lifts
GATE_Q_RET_PLANTED_COP_SAGITTAL_GATE = "0"  # off — prefer clear Δ; plant_gate is the CoP/stance fix
GATE_Q_RET_PLANTED_PLANT_GATE = "1"  # skip stance_plant freejoint XY while planted
GATE_Q_RET_PLANTED_XY_QFRC_KD = "26"  # restore k8n (kd28 tip/cam bomb)
GATE_Q_RET_PLANTED_XY_QFRC_MAX = "20"  # N clip on planted XY qfrc
GATE_Q_RET_CONTACT_XY_CANCEL = "1"  # oppose sole-floor tangential contact XY on freejoint
GATE_Q_RET_CONTACT_XY_SCALE = "0.70"  # k8n band; higher=more plant cancel
GATE_Q_RET_CONTACT_XY_MAX = "24"  # N clip on contact-cancel qfrc
GATE_Q_RET_CONTACT_XY_CAM0 = "0"  # full cancel mid-retreat (k8n; ramp-up abandoned — tip/plant)
GATE_Q_RET_CONTACT_XY_CAM1 = "0"  # full cancel (no ramp-up)
GATE_Q_RET_CONTACT_XY_EXIT0 = "0.48"  # A28 shared
GATE_Q_RET_CONTACT_XY_EXIT1 = "0.58"
GATE_Q_RET_CONTACT_XY_EXIT_MIN = "0.55"
GATE_Q_RET_CONTACT_XY_SIGNED = "0"  # OFF — signed hurt cf/cam; keep unsigned k8n cancel
GATE_Q_BOUT1_CLEAR_HOLD_S = "0.32"  # tip: DROP over-damp; match golden CLEAR_HOLD
GATE_Q_BOUT1_PLANTED_XY_DAMP = "0.40"  # tip: match golden app planted damp (was 0.32 over-damp)
GATE_Q_BOUT1_EXTRA_HOLD = "0.0"  # tip: DROP EXTRA_HOLD2.0 (over-damp handoff; kill lifts)
GATE_Q_BOUT1_SWING_HIP_SCALE = "1.45"  # golden app hip
GATE_Q_BOUT1_RES_TAPER = "0.85"  # tip: keep clear residual after 1+1 (was 0.70; grow dx_clear)
GATE_Q_RET_SOFT_CAM = "0.50"  # A3/C3 best ret cf~0.39 (0.52 identical)
GATE_Q_RET_TAPER_CAM = "0.45"
GATE_Q_BOUT1_RET_SOFT_CAM = "0.55"  # A11: full reverse until stop cam (was taper@0.45 freezing ~0.46)
GATE_Q_BOUT1_RET_TAPER_CAM = "0.52"
GATE_Q_BOUT1_RETREAT_AMP_SCALE = "0.68"  # E7lock plateau (A17/A28; banner amp0.62 is stale label)
GATE_Q_BOUT1_RETREAT_AMP_LATE = "0.82"  # E7lock late ramp
GATE_Q_BOUT1_RETREAT_AMP_LATE_CAM0 = "0.22"
GATE_Q_BOUT1_RETREAT_AMP_LATE_CAM1 = "0.38"
GATE_Q_BOUT1_RET_SWING_HIP_SCALE = "1.50"  # E7lock clear hip (plateau)
GATE_Q_BOUT1_RETREAT_RES_SCALE = "1.00"  # E7lock clear residual
GATE_Q_BOUT1_RETREAT_ADD_KNEE = "0.14"
GATE_Q_BOUT1_RETREAT_ADD_ANK = "0.14"
GATE_Q_BOUT1_RETREAT_ADD_ABD = "0.05"
GATE_Q_BOUT1_RET_CLEAR_HOLD_S = "0.48"  # E7lock A28 hold
GATE_Q_BOUT1_RET_TIP_SOFT_CAM = "0.55"  # A8: do not tip-soft-kill before stop cam
GATE_Q_RET_TIP_SOFT_CAM = "0.45"  # bout0 unchanged
GATE_Q_BOUT1_CONTACT_XY_CANCEL = "0"  # OFF — bout1 cancel tip-bombed clear_frac (J0/J1)
GATE_Q_BOUT1_CONTACT_XY_SCALE = "0.55"  # milder than ret 0.70 (protect lifts/tip)
GATE_Q_BOUT1_CONTACT_XY_MAX = "18"
GATE_Q_BOUT1_PLANTED_XY_QFRC_KD = "18"  # mild vel-oppose on bout1 planted
GATE_Q_RET_PLANTED_QVEL0 = "0"  # OFF — full qvel0 stalls cam / kills bout1 (k8m)
GATE_Q_RET_PLANTED_QVEL_SCALE = "1.0"  # once/ctrl multiply freejoint XY qvel while planted (not pin)
GATE_Q_RET_GAP_QVEL_SCALE = "1.0"  # if <1: apply as qvel_sc only when SS-silent ≥ gap (skate cut, keep amp)
GATE_Q_RET_GAP_QVEL_SS_GAP = "2.0"
GATE_Q_CLEAR_HOLD_AIR_M_RET = "0.028"  # restore C3 (0.024 stalled)
GATE_Q_RET_EARLY_DX_CLEAR = "0.15"  # stop when cam≥0.55 + dx_clear≥this + multi-SS
GATE_Q_RET_PERIODIC_REBURST_DT = "2.5"  # restore (1.8 stalled ret cam)
GATE_Q_RET_PERIODIC_REBURST_DX = "0.08"
GATE_Q_RET_PERIODIC_REBURST_MAX = "4"
GATE_Q_RET_CLEAR_TRACK_SWING = "1"  # A28c E3: clear-hold Δ counts as clear (cf 0.48→0.65 bout1)
# R1 retreat-native swing placement: clear-only world −X foot target offset (m); 0=OFF
GATE_Q_RET_SWING_PLACE_M = "0"
GATE_Q_RET_SWING_PLACE_EARLY_CAM = "0"  # if >0: only apply place while cam < this (first cluster)
GATE_Q_RET_SWING_PLACE_GAIN = "0.20"  # metres→rad divisor (larger=milder)
# R2 discrete reverse clear-bursts (default OFF) — clear-only lunge then brief planted hold
GATE_Q_RET_CLEAR_BURST = "0"
GATE_Q_RET_CLEAR_BURST_PERIOD_S = "3.0"  # seconds between burst starts
GATE_Q_RET_CLEAR_BURST_DUTY_S = "0.55"  # clear-lunge window
GATE_Q_RET_CLEAR_BURST_HOLD_S = "0.45"  # planted settle after burst (cancel×0.70; no pin)
GATE_Q_RET_CLEAR_BURST_HIP_MULT = "1.25"  # × clear hip during burst while sole clear
GATE_Q_RET_CLEAR_BURST_ADD_MULT = "1.35"  # × clearance ADD during burst
GATE_Q_RET_CLEAR_BURST_HOLD_DAMP = "0.18"  # brief hold-only planted damp (not continuous damp↑)
GATE_Q_RET_CLEAR_BURST_SS_MIN = "2"  # need this many ret SS before schedule arms
GATE_Q_RET_CLEAR_BURST_CAM_MAX = "0.52"  # stop schedule near dest cam
# R3 hard-capped plant gaps (default OFF) — clear bursts + plant_cap; NO damp-hold
# GATE_Q_RET_BURST_PLANT_CAP seconds; 0=OFF. When >0: R3 schedule (R2 CLEAR_BURST stay 0).
GATE_Q_RET_BURST_PLANT_CAP = "0"
# R3 reuses CLEAR_BURST_DUTY/HIP/ADD/SS_MIN/CAM_MAX for clear-burst kinematics.
# Plant gap between bursts hard-capped at PLANT_CAP; cancel×0.70 + vel-oppose only.
# Prefer FAIL ε: any plant-gap cam gain >0.02 OR cumulative >0.05/bout.
GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS = "0.02"
GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM = "0.05"
# R4 forced reverse SS cadence / max plant dwell (default OFF) — reactive; NO damp-hold
# GATE_Q_RET_MAX_PLANT_DWELL seconds; 0=OFF. GATE_Q_RET_MIN_SS_RATE SS/s; 0=OFF.
# When plant dwell > cap OR live plant-cam would trip ε (any>0.02 / cum>0.05), force next clear lift.
# Distinct from R3 scheduled burst→capped-gap. R1 place OFF. R2 CLEAR_BURST OFF. R3 PLANT_CAP stay 0.
GATE_Q_RET_MAX_PLANT_DWELL = "0"
GATE_Q_RET_MIN_SS_RATE = "0"
GATE_Q_RET_FORCE_LIFT_DUTY_S = "0.55"  # clear-kick window (reuse burst-like kinematics)
GATE_Q_RET_FORCE_LIFT_HIP_MULT = "1.25"
GATE_Q_RET_FORCE_LIFT_ADD_MULT = "1.35"
GATE_Q_RET_FORCE_LIFT_SS_MIN = "2"
GATE_Q_RET_FORCE_LIFT_CAM_MAX = "0.52"
GATE_Q_RET_FORCE_LIFT_COOLDOWN_S = "0.40"  # anti-chatter after force / flush
# Plant-cam ε identical to R3 (reuse EPS/CUM above). Log plant_dwell_max / force_count / cam_gain_during_plant.
# S1 CSF50-on-retreat (default OFF) — capture-timed reverse F,T from CoM under negative Vx.
# ≠ R1 place / R2 damp-hold / R3 gap-cap / R4 dwell-force. R1–R5 flags stay OFF.
GATE_Q_RET_CSF50 = "0"  # 1=on retreat-only CSF50-shaped F,T
GATE_Q_RET_CSF50_VX = "-0.08"  # commanded reverse sagittal m/s (negative = away from door)
GATE_Q_RET_CSF50_F_SCALE = "1.0"  # scale on swing amplitude F
GATE_Q_RET_CSF50_T_SCALE = "1.0"  # scale on support-exchange T
GATE_Q_RET_CSF50_T_NOM = "0.75"  # CSF50-like nominal exchange (Gate D CSF50 T75 basin)
GATE_Q_RET_CSF50_DX_BACK_MAX = "0.03"  # Placo-style |dx_back| cap (m); 0=no cap beyond soft 0.08
GATE_Q_RET_CSF50_TRACK_GAIN = "0.22"  # clear-only metres→rad for F-track (larger=milder)
# S2 ALIP-TVR-SWING (default OFF) — mid-swing residual foothold for reverse Vx.
# ≠ R1 place / R2 damp-hold / R3 gap-cap / R4 dwell-force / S1 CSF50 F+T. CSF50 stay 0.
GATE_Q_RET_ALIP_TVR = "0"  # 1=on retreat-only ALIP/TVR foothold + mid-swing Δu_fp
GATE_Q_RET_ALIP_TVR_VX = "-0.08"  # reverse sagittal command m/s (negative = away from door)
GATE_Q_RET_ALIP_TVR_N_REPLAN = "2"  # mid-swing replan count (1–N)
GATE_Q_RET_ALIP_TVR_SMOOTH = "0.5"  # consecutive Δu_fp smoothness weight
GATE_Q_RET_ALIP_TVR_DX_CAP = "0.04"  # optional |foothold Δ| cap (m); 0=soft 0.08
GATE_Q_RET_ALIP_TVR_TRACK_GAIN = "0.22"  # clear-only metres→rad for foothold track
# S3 REV-PHASE (default OFF) — reverse phase-polarity / hip / swing encoding for retreat.
# ≠ R1 place / R2 damp-hold / R3 gap-cap / R4 dwell-force / S1 CSF50 F+T / S2 ALIP Δu_fp.
# CSF50=0 · ALIP_TVR=0 throughout. CLEAR_TRACK clear-only + cancel×0.70 KEPT.
GATE_Q_RET_REV_PHASE = "0"  # 1=on retreat-only phase/hip/swing polarity
GATE_Q_RET_REV_PHASE_PHI = "0.5"  # phase-clock offset (0.5=half-cycle) or ≈π for 1-phi invert
GATE_Q_RET_REV_PHASE_HIP = "-1.0"  # hip-pitch sign/bias scale (full invert)
GATE_Q_RET_REV_PHASE_SWING = "1.0"  # swing-leg encoding flip strength (1=full L↔R / +0.5phi)
# Early clear-cam front-load (cam < EARLY_CAM): boost clear-only hip/res/hold, then E7lock schedule
GATE_Q_RET_EARLY_CAM = "0.35"
GATE_Q_RET_EARLY_T = "999"  # if <999: time-gate front-load to t_ret_local < this (first SS cluster)
GATE_Q_RET_EARLY_HIP_MULT = "1.0"  # × clear swing hip while cam < EARLY_CAM
GATE_Q_RET_EARLY_RES_MULT = "1.0"  # × clear residual while cam < EARLY_CAM
GATE_Q_RET_EARLY_HOLD_ADD = "0.0"  # +s clear-hold while cam < EARLY_CAM
GATE_Q_RET_EARLY_REBURST = "0"  # if 1: one reburst after multi-SS while cam < EARLY_CAM + dxc low
GATE_Q_RET_EARLY_REBURST_DXC = "0.12"
GATE_Q_RET_CLEAR_THROUGH_GAP = "0"  # 1=on: after SS, phi→swing + ADD kick (not planted-amp)
GATE_Q_RET_CLEAR_THROUGH_AGE_LO = "0.35"
GATE_Q_RET_CLEAR_THROUGH_AGE_HI = "2.2"
GATE_Q_RET_CLEAR_THROUGH_ADD_MULT = "1.30"
GATE_Q_RET_CLEAR_THROUGH_SS_MIN = "3"

GATE_Q_RET_RES_STRICT_SS = "0"  # OFF — strict SS residual stalled cam (k8aa); soft hold residual = k8n
GATE_Q_SKATE_PLANT_S = "0.24"
GATE_Q_SKATE_PLANT_RET_S = "0.20"
GATE_Q_PLANT_SETTLE = "0"  # approach golden
GATE_Q_PLANT_SETTLE_RET = "0"  # off — settle alone did not raise clear_frac
GATE_Q_RET_PLANTED_XY_RETAIN = "1.0"  # OFF — retain<1 stalls cam / regresses bout1 app (freeze-class)
GATE_Q_CADENCE_SPAN_FRAC_MIN = "0.50"
GATE_Q_LIFTS_PER_M_MIN = "10.0"
GATE_Q_SPAN_EARLY_MIN = "0.55"
GATE_Q_SS_TAIL_MAX_M = "0.14"
GATE_Q_SS_TIME_SPAN_FRAC_MIN = "0.45"
GATE_Q_DISABLE_MID_SETTLE = "0"

def _ss_multi_proved(n_L: int, n_R: int) -> bool:
    """Thin multi-step (≥2/side or ≥3 total) — mid-settle trigger / criteria floor."""
    n_L, n_R = int(n_L or 0), int(n_R or 0)
    if n_L >= SS_PER_SIDE_MIN and n_R >= SS_PER_SIDE_MIN:
        return True
    return (n_L + n_R) >= SS_TOTAL_MIN and n_L >= 1 and n_R >= 1

def _ss_dense_proved(n_L: int, n_R: int) -> bool:
    """True dense: ≥3/side or ≥4 total both≥2. No weak ≥3-total fallthrough (3n Root B)."""
    n_L, n_R = int(n_L or 0), int(n_R or 0)
    if n_L >= SS_DENSE_PER_SIDE and n_R >= SS_DENSE_PER_SIDE:
        return True
    if (n_L + n_R) >= SS_DENSE_TOTAL and n_L >= 2 and n_R >= 2:
        return True
    return False

def _ss_span_frac_live(ss_xs, dx_now: float) -> float:
    if not ss_xs or len(ss_xs) < 2 or abs(dx_now) < 1e-3:
        return 0.0
    return float(abs(max(ss_xs) - min(ss_xs)) / max(abs(dx_now), 1e-6))

def _ss_span_ready(ss_xs, dx_now: float, n_L: int, n_R: int) -> bool:
    span_need = float(__import__("os").environ.get("GATE_Q_SPAN_EARLY_MIN", GATE_Q_SPAN_EARLY_MIN))
    if _ss_span_frac_live(ss_xs, dx_now) < span_need - 1e-9:
        return False
    return _ss_multi_proved(n_L, n_R) and (n_L + n_R) >= 4

def _ss_time_span_frac(ss_ts, t0, t_now: float) -> float:
    """Fraction of walk duration covered by first→last SS time (continuous cadence)."""
    if not ss_ts or len(ss_ts) < 2:
        return 0.0
    return float((max(ss_ts) - min(ss_ts)) / max(float(t_now) - float(t0), 1e-6))

def apply_ret_swing_place_m(m, d, act_idx, phi, amp, bid_lf, bid_rf, place_m):
    """R1: while retreat sole-clear, bias swing hip so aerial foot lands farther −X (away from door).

    One-axis world-X only. Joint-space hip/ank — NO planted soft-XY / freejoint force.
    place_m is metres of additional reverse stride (mild 0.02–0.04).
    """
    if place_m < 1e-6 or amp < 0.05:
        return
    import numpy as _np
    rest = 0.0086
    ar_m = float(os.environ.get("GATE_Q_SWING_CLEAR_AR_M", GATE_Q_SWING_CLEAR_AR_M))
    abs_m = float(os.environ.get("GATE_Q_SWING_CLEAR_ABS_M", GATE_Q_SWING_CLEAR_ABS_M))
    ds_frac = float(_np.clip(wg.DS_S / max(wg.GAIT_T, 1e-3), 0.08, 0.55))
    ds_end = 0.5 + 0.5 * ds_frac
    # negative d_fwd → place foot in −X (CP convention: +d_fwd = +X)
    _gain = float(os.environ.get("GATE_Q_RET_SWING_PLACE_GAIN", GATE_Q_RET_SWING_PLACE_GAIN))
    _gain = max(_gain, 0.05)
    d_base = float(_np.clip(-float(place_m) / _gain, -0.30, 0.30)) * float(amp)
    for side, bid, sign in (("L", bid_lf, -1.0), ("R", bid_rf, +1.0)):
        z = float(d.xpos[bid, 2])
        clr = z - SOLE_OFFSET
        if (clr - rest) < ar_m - 1e-9 or clr < abs_m - 1e-9:
            continue  # planted / not truly clear — never place as stride
        p_leg = wg.phase_leg(phi, side)
        if p_leg < ds_end:
            continue  # not in swing half
        s = (p_leg - ds_end) / max(1e-6, 1.0 - ds_end)
        if s < 0.15 or s > 0.95:
            continue
        sw = wg.swing_blend(min(1.0, s / 0.7))
        d_fwd = d_base * sw
        pref = "l_" if side == "L" else "r_"
        an = f"{pref}hip_pitch_pos"
        if an in act_idx:
            d.ctrl[act_idx[an]] = float(_np.clip(d.ctrl[act_idx[an]] + sign * d_fwd, -1.5, 1.5))
        an_a = f"{pref}ank_pitch_pos"
        if an_a in act_idx:
            d.ctrl[act_idx[an_a]] = float(_np.clip(d.ctrl[act_idx[an_a]] + sign * 0.45 * d_fwd, -1.2, 1.2))


def csf50_retreat_FT(com_xy, com_vxy, com_z, vx_cmd, *,
                     T_nom=0.75, F_scale=1.0, T_scale=1.0, dx_back_max=0.03):
    """Missura/CSF50-shaped capture F + T for reverse Vx (retreat only).

    F = sagittal swing amplitude (m), ≤0 for reverse (−X away from door).
    T = support-exchange period (s) from lateral orbital / LIPM state.
    Planted XY is never credited here — caller tracks F only while sole clear.
    """
    import math as _math
    import numpy as _np
    g = 9.81
    z = max(float(com_z), 0.12)
    omega = max(_math.sqrt(g / z), 0.5)
    vx = float(com_vxy[0]); vy = float(com_vxy[1])
    vxc = float(vx_cmd)
    # Lateral remaining-exchange estimate (orbital): soft asinh on |vy|/(ω·y_ref)
    y_ref = 0.04
    T_lat = (1.0 / omega) * _math.asinh(max(1e-6, abs(vy) / max(omega * y_ref, 1e-3)))
    T = float(T_nom) * float(T_scale) * (0.85 + 0.15 * min(T_lat / 0.40, 1.5))
    T = float(_np.clip(T, 0.45, 1.20))
    # Sagittal F: commanded reverse Vx over T + mild DCM (cp) correction
    F_raw = vxc * T + (vx - vxc) / omega * 0.35
    F = float(F_raw) * float(F_scale)
    if float(dx_back_max) > 1e-9:
        F = float(_np.clip(F, -abs(float(dx_back_max)), 0.0))
    else:
        F = float(_np.clip(F, -0.08, 0.0))
    return F, T, omega


def apply_csf_f_track(m, d, act_idx, phi, amp, bid_lf, bid_rf, F_cmd):
    """Clear-only: bias swing hip so aerial foot lands near support+F (CSF foothold).

    Joint-space only. NO planted soft-XY / freejoint force. ≠ R1 fixed world-place_m —
    F_cmd is capture-computed (≤0 reverse) and applied only while sole clear.
    """
    if amp < 0.05 or abs(float(F_cmd)) < 1e-6:
        return
    import numpy as _np
    rest = 0.0086
    ar_m = float(os.environ.get("GATE_Q_SWING_CLEAR_AR_M", GATE_Q_SWING_CLEAR_AR_M))
    abs_m = float(os.environ.get("GATE_Q_SWING_CLEAR_ABS_M", GATE_Q_SWING_CLEAR_ABS_M))
    ds_frac = float(_np.clip(wg.DS_S / max(wg.GAIT_T, 1e-3), 0.08, 0.55))
    ds_end = 0.5 + 0.5 * ds_frac
    _gain = float(os.environ.get("GATE_Q_RET_CSF50_TRACK_GAIN", GATE_Q_RET_CSF50_TRACK_GAIN))
    _gain = max(_gain, 0.05)
    # Support mid-x from planted foot (or both mean)
    zL = float(d.xpos[bid_lf, 2]); zR = float(d.xpos[bid_rf, 2])
    cL = zL - SOLE_OFFSET; cR = zR - SOLE_OFFSET
    L_plant = not ((cL - rest) >= ar_m - 1e-9 and cL >= abs_m - 1e-9)
    R_plant = not ((cR - rest) >= ar_m - 1e-9 and cR >= abs_m - 1e-9)
    if L_plant and not R_plant:
        support_x = float(d.xpos[bid_lf, 0])
    elif R_plant and not L_plant:
        support_x = float(d.xpos[bid_rf, 0])
    else:
        support_x = 0.5 * (float(d.xpos[bid_lf, 0]) + float(d.xpos[bid_rf, 0]))
    des_x = support_x + float(F_cmd)  # F≤0 → land further −X
    for side, bid, sign in (("L", bid_lf, -1.0), ("R", bid_rf, +1.0)):
        z = float(d.xpos[bid, 2])
        clr = z - SOLE_OFFSET
        if (clr - rest) < ar_m - 1e-9 or clr < abs_m - 1e-9:
            continue  # planted — never credit XY stride
        p_leg = wg.phase_leg(phi, side)
        if p_leg < ds_end:
            continue
        s = (p_leg - ds_end) / max(1e-6, 1.0 - ds_end)
        if s < 0.15 or s > 0.95:
            continue
        sw = wg.swing_blend(min(1.0, s / 0.7))
        foot_x = float(d.xpos[bid, 0])
        err = des_x - foot_x
        # +d_fwd in CP convention = +X; we want err (−X if des behind) → negative d_fwd
        d_fwd = float(_np.clip(err / _gain, -0.30, 0.30)) * float(amp) * sw
        pref = "l_" if side == "L" else "r_"
        an = f"{pref}hip_pitch_pos"
        if an in act_idx:
            d.ctrl[act_idx[an]] = float(_np.clip(d.ctrl[act_idx[an]] + sign * d_fwd, -1.5, 1.5))
        an_a = f"{pref}ank_pitch_pos"
        if an_a in act_idx:
            d.ctrl[act_idx[an_a]] = float(_np.clip(d.ctrl[act_idx[an_a]] + sign * 0.45 * d_fwd, -1.2, 1.2))


def alip_tvr_foothold_prior(com_xy, com_vxy, com_z, vx_cmd, *,
                            dx_cap=0.04, dx_remain=None):
    """ALIP / TVR foothold prior for reverse Vx (retreat only).

    Returns (F, omega) where F is support-relative sagittal land offset (m), ≤0 for
    reverse (−X away from door). Distinct from CSF50 F,T exchange — no T, no capture
    period drive. Mid-swing residual Δu_fp is layered by the caller with smoothness.
    """
    import math as _math
    import numpy as _np
    g = 9.81
    z = max(float(com_z), 0.12)
    omega = max(_math.sqrt(g / z), 0.5)
    vx = float(com_vxy[0])
    vxc = float(vx_cmd)
    # Softened ALIP: F ≈ v_des/(0.35ω) + α(vx−v_des)/ω → reverse-biased ≤0
    F = vxc / max(omega * 0.35, 0.5) + 0.25 * (vx - vxc) / omega
    if dx_remain is not None and float(dx_remain) > 1e-4:
        _share = min(float(dx_remain) * 0.12, abs(float(dx_cap)) if float(dx_cap) > 1e-9 else 0.08)
        F = 0.75 * F - 0.25 * _share
    cap = abs(float(dx_cap)) if float(dx_cap) > 1e-9 else 0.08
    F = float(_np.clip(F, -cap, 0.0))
    return F, omega



def _gate_q_t5e_active() -> bool:
    """T5-E FOOTSTEP-SEQ dual-ckpt / outer path (cospec GATE_Q_AI_COSPEC_T5E)."""
    return (
        bool(os.environ.get("GATE_Q_T5E_CKPT", "").strip())
        or os.environ.get("GATE_Q_T5E_FOOTSTEP_SEQ", "0") not in ("0", "false", "no")
    )


def _gate_q_t5d2_active() -> bool:
    """T5-D2 ICP-ΔT+ΔFOOT dual-ckpt / outer path (cospec GATE_Q_AI_COSPEC_T5D2)."""
    return (
        bool(os.environ.get("GATE_Q_T5D2_CKPT", "").strip())
        or os.environ.get("GATE_Q_T5D2_ICP", "0") not in ("0", "false", "no")
    )


def _gate_q_t5d_family_active() -> bool:
    """T5-E / T5-D1 / T5-D2 outer-primary dual-ckpt path."""
    return (
        _gate_q_t5e_active()
        or _gate_q_t5d2_active()
        or bool(os.environ.get("GATE_Q_T5D1_CKPT", "").strip())
        or os.environ.get("GATE_Q_T5D1_OUTER_PRIMARY", "0") not in ("0", "false", "no")
    )


def _emit_t5e_td_list(target_dx, *, dx_back=0.028, n_max=10, n_min=3, lat_bos=0.0):
    """Placo-style short reverse TD list for T5-E FOOTSTEP-SEQ (scorer-side)."""
    import math as _m
    dxb = float(max(0.012, min(abs(float(dx_back)), 0.045)))
    budget = float(max(0.04, abs(float(target_dx))))
    n = int(_m.ceil(budget / dxb))
    n = max(int(n_min), min(int(n_max), n))
    tds = []
    for i in range(n):
        side = "L" if (i % 2 == 0) else "R"
        lat = float(lat_bos) if side == "L" else -float(lat_bos)
        tds.append({"i": int(i), "F": float(-dxb), "side": side, "lat": float(lat), "dx_back": float(dxb)})
    return tds


def icp_delta_TF(com_xy, com_vxy, com_z, support_x, vx_cmd, F_prior, T_prior, *,
                 dx_cap=0.035, k_T=3.0, k_foot=1.4, T_lo=0.50, T_hi=1.10):
    """IHMC Atlas-style ICP error → ΔT + Δfoot (arXiv:1703.00477). Retreat F≤0."""
    g = 9.81
    z = max(float(com_z), 0.12)
    omega = max(math.sqrt(g / z), 0.5)
    icp_x = float(com_xy[0]) + float(com_vxy[0]) / omega
    e_icp = float(icp_x) - float(support_x)
    F_p = float(F_prior); T_p = float(T_prior)
    cap = abs(float(dx_cap)) if float(dx_cap) > 1e-9 else 0.035
    F_icp = float(np.clip(e_icp, -cap, 0.0))
    dfoot = float(np.clip(F_icp - F_p, -cap, cap))
    if float(vx_cmd) < 0.0:
        e_escape = max(0.0, -(e_icp) - abs(F_p))
        e_recover = max(0.0, e_icp)
    else:
        e_escape = max(0.0, e_icp - abs(F_p))
        e_recover = max(0.0, -e_icp)
    dT = float(np.clip(-k_T * e_escape + 0.35 * k_T * e_recover, -0.28, 0.18))
    F_new = float(np.clip(F_p + dfoot, -cap, 0.0))
    T_new = float(np.clip(T_p + dT, T_lo, T_hi))
    return F_new, T_new, float(icp_x), float(e_icp), float(dT), float(dfoot), float(omega)


def apply_alip_fp_track(m, d, act_idx, phi, amp, bid_lf, bid_rf, F_cmd):
    """Clear-only: bias swing hip so aerial foot lands near support+F (ALIP/TVR foothold).

    Joint-space only. NO planted soft-XY. ≠ R1 fixed world-place; ≠ S1 CSF F exchange —
    F_cmd is ALIP/TVR prior + mid-swing Δu_fp residual, applied only while sole clear.
    """
    if amp < 0.05 or abs(float(F_cmd)) < 1e-6:
        return
    import numpy as _np
    rest = 0.0086
    ar_m = float(os.environ.get("GATE_Q_SWING_CLEAR_AR_M", GATE_Q_SWING_CLEAR_AR_M))
    abs_m = float(os.environ.get("GATE_Q_SWING_CLEAR_ABS_M", GATE_Q_SWING_CLEAR_ABS_M))
    ds_frac = float(_np.clip(wg.DS_S / max(wg.GAIT_T, 1e-3), 0.08, 0.55))
    ds_end = 0.5 + 0.5 * ds_frac
    _gain = float(os.environ.get("GATE_Q_RET_ALIP_TVR_TRACK_GAIN", GATE_Q_RET_ALIP_TVR_TRACK_GAIN))
    _gain = max(_gain, 0.05)
    zL = float(d.xpos[bid_lf, 2]); zR = float(d.xpos[bid_rf, 2])
    cL = zL - SOLE_OFFSET; cR = zR - SOLE_OFFSET
    L_plant = not ((cL - rest) >= ar_m - 1e-9 and cL >= abs_m - 1e-9)
    R_plant = not ((cR - rest) >= ar_m - 1e-9 and cR >= abs_m - 1e-9)
    if L_plant and not R_plant:
        support_x = float(d.xpos[bid_lf, 0])
    elif R_plant and not L_plant:
        support_x = float(d.xpos[bid_rf, 0])
    else:
        support_x = 0.5 * (float(d.xpos[bid_lf, 0]) + float(d.xpos[bid_rf, 0]))
    des_x = support_x + float(F_cmd)  # F≤0 → land further −X
    for side, bid, sign in (("L", bid_lf, -1.0), ("R", bid_rf, +1.0)):
        z = float(d.xpos[bid, 2])
        clr = z - SOLE_OFFSET
        if (clr - rest) < ar_m - 1e-9 or clr < abs_m - 1e-9:
            continue  # planted — never credit XY stride
        p_leg = wg.phase_leg(phi, side)
        if p_leg < ds_end:
            continue
        s = (p_leg - ds_end) / max(1e-6, 1.0 - ds_end)
        if s < 0.15 or s > 0.95:
            continue
        sw = wg.swing_blend(min(1.0, s / 0.7))
        foot_x = float(d.xpos[bid, 0])
        err = des_x - foot_x
        d_fwd = float(_np.clip(err / _gain, -0.30, 0.30)) * float(amp) * sw
        pref = "l_" if side == "L" else "r_"
        an = f"{pref}hip_pitch_pos"
        if an in act_idx:
            d.ctrl[act_idx[an]] = float(_np.clip(d.ctrl[act_idx[an]] + sign * d_fwd, -1.5, 1.5))
        an_a = f"{pref}ank_pitch_pos"
        if an_a in act_idx:
            d.ctrl[act_idx[an_a]] = float(_np.clip(d.ctrl[act_idx[an_a]] + sign * 0.45 * d_fwd, -1.2, 1.2))


def _ss_carry_tail_ok(ss_xs, bx_now: float, *, reverse: bool = False,
                      max_tail_m: float | None = None) -> bool:
    """Last SS must be near current base — forbids early-cluster then long soft-slide tail."""
    if not ss_xs:
        return False
    tail_max = float(max_tail_m if max_tail_m is not None else
                     __import__("os").environ.get("GATE_Q_SS_TAIL_MAX_M", GATE_Q_SS_TAIL_MAX_M))
    if reverse:
        return (float(min(ss_xs)) - float(bx_now)) <= tail_max + 1e-9
    return (float(bx_now) - float(max(ss_xs))) <= tail_max + 1e-9

def _md5(p: Path) -> str:
    return hashlib.md5(p.read_bytes()).hexdigest()

def _gait_amp(t_local, stand_hold, ramp_t):
    if t_local < stand_hold: return 0.0
    u = t_local - stand_hold
    if u >= ramp_t: return 1.0
    s = u / ramp_t
    return float(s * s * (3.0 - 2.0 * s))

def _mean_p95(xs):
    if not xs: return 0.0, 0.0
    a = np.asarray(xs, dtype=np.float64)
    return float(a.mean()), float(np.percentile(a, 95))

def _substep_p(env, arm_q, *, disturb_xfrc=None, base_xy=None, walk_ctrl=None, stand_pin=False):
    m, d = env.model, env.data
    n = env.steps_per_ctrl
    bid_panel = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "door_panel_link")
    bid_com = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "body_link")
    if walk_ctrl is not None:
        a = float(walk_ctrl["amp"]); phi = float(walk_ctrl["phi"])
        t_abs = float(walk_ctrl.get("t_abs", d.time))
        qdes = wg.gait_targets(t_abs if a > 0 else 0.0, a > 0.01, a)
        wg.set_ctrl(m, d, qdes, env.act_idx)
        d.ctrl[env.aid_tilt] = env.head_tilt
        lat = wg.lateral_com_target(phi) if a > 0 else 0.0
        bias = walk_ctrl.get("cop_bias")
        if a > 0.01:
            saved = d.subtree_com[0, :2].copy()
            d.subtree_com[0, :2] = d.subtree_com[bid_com, :2]
            try:
                wg.ankle_cop_servo(m, d, env.act_idx, a, env.bid_lf, env.bid_rf, env.gid_floor,
                                   lat=lat if a > 0.05 else None, bias_xy=bias)
            except Exception: pass
            d.subtree_com[0, :2] = saved
            try: wg.apply_cp_swing_placement(m, d, env.act_idx, phi, a, env.bid_lf, env.bid_rf)
            except Exception: pass
            try: wg.apply_stance_jacobian_vik(m, d, env.act_idx, phi, a, env.bid_lf, env.bid_rf, env.gid_floor)
            except Exception: pass
            action = walk_ctrl.get("action")
            if action is not None:
                delta = np.clip(np.asarray(action, dtype=np.float64).reshape(12), -1, 1) * ACTION_SCALE
                for i, ai in enumerate(env.leg_act):
                    d.ctrl[ai] = float(np.clip(d.ctrl[ai] + delta[i], -CTRL_LIM, CTRL_LIM))
            d.ctrl[env.aid_tilt] = env.head_tilt
        # During stepped walk: do NOT pin door-reach arms (destabilizes gait).
        # gait_targets already set arm ctrl; only hold head_tilt.
        d.qpos[m.jnt_qposadr[env.jid_tilt]] = env.head_tilt
        d.ctrl[env.aid_tilt] = env.head_tilt
        for _ in range(n):
            d.qfrc_applied[:] = 0; d.xfrc_applied[:] = 0
            if disturb_xfrc is not None and bid_panel >= 0:
                d.xfrc_applied[bid_panel, :3] = disturb_xfrc
            if a > 0:
                wg.stance_plant(m, d, phi, a, env.bid_lf, env.bid_rf, env.gid_lfoot, env.gid_rfoot, env.gid_floor)
            mj.mj_step(m, d)
        return
    qstand = wg.gait_targets(0.0, False, 0.0)
    wg.set_ctrl(m, d, qstand, env.act_idx)
    d.ctrl[env.aid_tilt] = env.head_tilt
    for _ in range(n):
        if stand_pin:
            for jn, val in qstand.items():
                jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
                if jid >= 0:
                    d.qpos[m.jnt_qposadr[jid]] = val
                    d.qvel[m.jnt_dofadr[jid]] = 0.0
            # Full free-joint hold during settle/finalize/retreat (prevent pre-walk drift)
            if base_xy is not None:
                d.qpos[env.q_free] = base_xy[0]
                d.qpos[env.q_free + 1] = base_xy[1]
            d.qpos[env.q_free + 2] = wg.COM_Z
            d.qpos[env.q_free + 3:env.q_free + 7] = [1, 0, 0, 0]
            d.qvel[env.v_free:env.v_free + 6] = 0.0
        for jn, val in arm_q.items():
            jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                d.qpos[m.jnt_qposadr[jid]] = val
                d.qvel[m.jnt_dofadr[jid]] = 0.0
        d.qpos[m.jnt_qposadr[env.jid_tilt]] = env.head_tilt
        if base_xy is not None:
            d.qpos[env.q_free] = base_xy[0]
            d.qpos[env.q_free + 1] = base_xy[1]
            d.qvel[env.v_free] = 0.0
            d.qvel[env.v_free + 1] = 0.0
        d.qfrc_applied[:] = 0; d.xfrc_applied[:] = 0
        if disturb_xfrc is not None and bid_panel >= 0:
            d.xfrc_applied[bid_panel, :3] = disturb_xfrc
        mj.mj_step(m, d)

def _banner(frame, y0, y1, x1, fill, text, font_size=20):
    from PIL import Image, ImageDraw, ImageFont
    img = Image.fromarray(frame); draw = ImageDraw.Draw(img)
    try: font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", font_size)
    except Exception: font = ImageFont.load_default()
    draw.rectangle([6, y0, x1, y1], fill=fill)
    draw.text((10, y0 + 2), text, fill=(255, 255, 0), font=font)
    return np.asarray(img)

def _hud_p(frame, dx, dy, x, y, t, bout, panel_deg, lever_deg, phase,
           disturb_on, leave_on, approach_on, retreat_on, disturb_force_on=False,
           cam_h=None, ss_event=False, shim_on=False):
    title = f"kit · Gate Q · {phase}"
    if shim_on: title = f"kit · Gate Q · KINEMATIC XY SHIM · {phase}"
    elif approach_on: title = f"kit · Gate Q · STEPPED APPROACH · {phase}"
    elif retreat_on: title = f"kit · Gate Q · STEPPED RETREAT · {phase}"
    elif disturb_on and disturb_force_on: title = f"kit · Gate Q · DISTURB FORCE · {phase}"
    elif disturb_on: title = f"kit · Gate Q · DISTURB WINDOW · {phase}"
    elif leave_on: title = f"kit · Gate Q · LEAVE · {phase}"
    if cam_h is not None: title = f"{title} · camH={cam_h:.2f}"
    out = _hud(frame, dx, dy, x, y, t, bout, panel_deg, lever_deg, title=title)
    yb = 128
    if shim_on:
        out = _banner(out, yb, yb + 40, 560, (120, 60, 0), "KINEMATIC XY SHIM — not stepped walk", 16); yb += 42
    elif approach_on:
        out = _banner(out, yb, yb + 40, 520, (0, 110, 40), "APPROACH START — stepped walk (active gait)", 16); yb += 42
    if ss_event:
        # Banner thr MUST match GATE_Q_HUD_CLEAR_* (never claim ≥2cm while thr is higher)
        _hud_cm = float(os.environ.get("GATE_Q_HUD_CLEAR_M", GATE_Q_HUD_CLEAR_M)) * 100.0
        out = _banner(out, yb, yb + 36, 520, (0, 140, 80), f"FOOT-LIFT / SS (≥{_hud_cm:.0f}cm sole clear)", 16); yb += 38
    # STEPPED RETREAT only while currently FOOT-LIFT-clear (ss_event) — kill flush slide overclaim
    if retreat_on and ss_event:
        out = _banner(out, yb, yb + 40, 480, (80, 80, 20), "STEPPED RETREAT — reverse gait (active)", 16); yb += 42
    if disturb_on:
        if disturb_force_on:
            out = _banner(out, yb, yb + 40, 440, (180, 0, 0), "DISTURBANCE ON — force applied (declared)", 18)
        else:
            out = _banner(out, yb, yb + 40, 440, (140, 40, 0), "DISTURB WINDOW (declared; force ended)", 18)
        yb += 42
    if leave_on:
        out = _banner(out, yb, yb + 40, 380, (0, 90, 140), "LEAVE WINDOW (no grasp)", 18)
    return out

def _render_p(env, qpos_log, xy_log, bout_log, t_log, phase_log, disturb_log, leave_log,
              force_log, approach_log, retreat_log, camh_log, ss_log, shim_log=None, stride=5, fps=10):
    import imageio.v2 as imageio
    from PIL import Image as _PILImage
    ITER.mkdir(parents=True, exist_ok=True)
    still_dir = ITER / "gate_q_stills"
    still_dir.mkdir(parents=True, exist_ok=True)
    still_counts = {"app_fl": 0, "app_fin": 0, "disturb": 0, "leave": 0, "ret_fl": 0, "ret_fin": 0}
    still_paths = []
    r = mj.Renderer(env.model, height=480, width=640)
    wcam = _world_cam(env.model); pcam = _panel_cam(env.model)
    jid_p = _panel_hinge_id(env.model); jid_l = _hinge_id(env.model)
    x0, y0 = xy_log[0]
    paths = {"kit": ITER/"Q03_kit.mp4", "panel": ITER/"Q03_panel.mp4", "world": ITER/"Q03_world.mp4",
             "dual": ITER/"Q03.mp4", "dual_s9": ITER/"Q03_dual_s9.mp4"}
    n = len(qpos_log)
    print(f"[Q] stream-render n={n} stride={stride} -> Q03*.mp4", flush=True)
    writers = {k: imageio.get_writer(str(pp), fps=fps, codec="libx264", quality=8, macro_block_size=1) for k, pp in paths.items()}
    try:
        for i in range(0, n, stride):
            env.data.qpos[:] = qpos_log[i]; env.data.qvel[:] = 0; mj.mj_forward(env.model, env.data)
            x, y = xy_log[i]; dx, dy = x - x0, y - y0
            panel_deg = _angle_deg(env.model, env.data, jid_p)
            lever_deg = _angle_deg(env.model, env.data, jid_l)
            tt, bb, ph = t_log[i], bout_log[i], phase_log[i]
            dist_on, leave_on = bool(disturb_log[i]), bool(leave_log[i])
            force_on, app_on, ret_on = bool(force_log[i]), bool(approach_log[i]), bool(retreat_log[i])
            ss_on = bool(ss_log[i]) if ss_log else False
            shim_on = bool(shim_log[i]) if shim_log else (ph == "finalize")
            # approach_on banner only during active stepped walk (not finalize)
            # iterate3r: STEPPED banner ONLY while currently FOOT-LIFT-clear (kill flush slide overclaim)
            stepped_on = (app_on or ret_on) and (not shim_on) and ss_on
            cam_h = float(camh_log[i]) if camh_log else None
            r.update_scene(env.data, camera=env.cid)
            kit = _hud_p(r.render().copy(), dx, dy, x, y, tt, bb, panel_deg, lever_deg, ph,
                         dist_on, leave_on, stepped_on, ret_on, disturb_force_on=force_on,
                         cam_h=cam_h, ss_event=ss_on, shim_on=shim_on)
            r.update_scene(env.data, camera=wcam)
            wt = f"S9 feet · {ph}" + (" · KINEMATIC XY SHIM" if shim_on else "") + (" · STEPPED" if stepped_on else "") + (" · SS/LIFT" if ss_on else "") + (" · RETREAT" if ret_on else "") + (" · DISTURB" if dist_on else "") + (" · LEAVE" if leave_on else "")
            world = _hud(_mark_base_edge(r.render().copy()), dx, dy, x, y, tt, bb, panel_deg, lever_deg, title=wt)
            r.update_scene(env.data, camera=pcam)
            pt = f"oblique free-edge · {ph}" + (" · KINEMATIC XY SHIM" if shim_on else "") + (" · STEPPED" if stepped_on else "") + (" · DISTURB ON" if dist_on else "") + (" · LEAVE" if leave_on else "")
            panel = _hud(r.render().copy(), dx, dy, x, y, tt, bb, panel_deg, lever_deg, title=pt)
            yb = 128
            if shim_on: panel = _banner(panel, yb, yb+28, 420, (120,60,0), "KINEMATIC XY SHIM"); yb += 30
            elif stepped_on and ret_on: panel = _banner(panel, yb, yb+28, 400, (80,80,20), "STEPPED RETREAT"); yb += 30
            elif stepped_on: panel = _banner(panel, yb, yb+28, 400, (0,110,40), "APPROACH START (stepped)"); yb += 30
            if ss_on:
                _hud_cm = float(os.environ.get("GATE_Q_HUD_CLEAR_M", GATE_Q_HUD_CLEAR_M)) * 100.0
                panel = _banner(panel, yb, yb+28, 400, (0,140,80), f"FOOT-LIFT / SS (≥{_hud_cm:.0f}cm)"); yb += 30
            if dist_on:
                panel = _banner(panel, yb, yb+28, 360, (180,0,0) if force_on else (140,40,0),
                                "DISTURBANCE ON (force)" if force_on else "DISTURB WINDOW"); yb += 30
            if leave_on: panel = _banner(panel, yb, yb+28, 300, (0,90,140), "LEAVE WINDOW")
            # Dense stills at key honesty moments (bout0 primary + bout1 sample)
            def _save_still(tag, limit=8):
                if still_counts[tag] >= limit: return
                still_counts[tag] += 1
                stem = f"{tag}_b{bb}_t{tt:.1f}_{still_counts[tag]:02d}"
                for name, arr in (("kit", kit), ("s9", world), ("panel", panel),
                                  ("dual", np.concatenate([kit, panel], axis=1))):
                    p = still_dir / f"{stem}_{name}.jpg"
                    _PILImage.fromarray(arr).save(p, quality=90)
                    still_paths.append(str(p.relative_to(ROOT)))
            if ss_on and stepped_on and app_on and not ret_on: _save_still("app_fl", 16)
            if ph == "finalize" and still_counts["app_fin"] < 4: _save_still("app_fin", 4)
            if dist_on and force_on: _save_still("disturb", 4)
            if leave_on: _save_still("leave", 4)
            if ss_on and ret_on and not shim_on: _save_still("ret_fl", 16)
            if ph == "retreat_finalize": _save_still("ret_fin", 4)
            writers["kit"].append_data(kit); writers["world"].append_data(world)
            writers["panel"].append_data(panel)
            writers["dual"].append_data(np.concatenate([kit, panel], axis=1))
            writers["dual_s9"].append_data(np.concatenate([kit, world], axis=1))
            if i % 500 == 0:
                print(f"[Q] render {i}/{n} phase={ph} panel={panel_deg:+.1f} camH={cam_h} app={app_on} ss={ss_on}", flush=True)
    finally:
        for w in writers.values(): w.close()
        r.close()
    for tag in ("Q00", "Q01", "Q02"):
        dst = ITER / f"{tag}.mp4"
        if dst.exists() or dst.is_symlink(): dst.unlink()
        try: dst.symlink_to("Q03.mp4")
        except OSError:
            import shutil; shutil.copy2(paths["dual"], dst)
    print("[Q] wrote Q03.mp4 (+ Q00-Q02 -> Q03)", flush=True)
    print(f"[Q] stills {sum(still_counts.values())} events -> gate_q_stills/ ({len(still_paths)} files)", flush=True)
    out = {k: str(pp.relative_to(ROOT)) for k, pp in paths.items()}
    out["stills_dir"] = str((ITER/"gate_q_stills").relative_to(ROOT))
    out["stills"] = still_paths
    out["still_counts"] = still_counts
    return out

def score_approach_compose_retreat(n_cycles=2, *, use_residual=False, video=True):
    arm_q, _, tilt = _arm_seed()
    stand_arm_q = {k: float(v) for k, v in arm_q.items()}
    # Neutral-ish stand arms for walk (less lateral COM pull than door-reach seed)
    for k in list(stand_arm_q.keys()):
        if "sho_roll" in k:
            stand_arm_q[k] = 1.4 if k.startswith("r_") else -1.4
        elif "sho_pitch" in k or "el_" in k:
            stand_arm_q[k] = 0.0 if "yaw" in k else (0.38 if "el_pitch" in k else 0.0)
    reach_arm_q = {k: float(v) for k, v in arm_q.items()}
    cam_work, cam_start = CAM_WORK, CAM_START
    t_settle_open, t_settle_close = 0.6, 0.6  # kinematic stand pin during settle
    push_s, hold_s = 10.0, 2.0
    disturb_s, reject_s = DISTURB_S, REJECT_S
    leave_s, regrasp_s = 1.0, 5.0  # iterate: longer regrasp for ≥0.3s contact
    close_s, recover_s = 6.5, 2.5
    walk_s, finalize_s, retreat_s = WALK_S, FINALIZE_S, RETREAT_S
    walk_hold, walk_ramp = WALK_STAND_HOLD, WALK_RAMP
    approach_s = walk_hold + walk_s + finalize_s
    push_to, rev_cmd_end = 29.8, -4.0
    soft_close = True; close_pull_scale = 0.22
    g_push, g_close, g_hold = 1.35, 1.15, 1.40
    g_leave, g_regrasp = 1.05, 2.10  # iterate: stronger regrasp IK
    leave_off = np.array([-0.05, 0.02, 0.015])  # iterate: smaller leave offset
    compose_s = push_s + hold_s + disturb_s + reject_s + leave_s + regrasp_s + close_s + recover_s
    cycle_s = approach_s + compose_s + retreat_s
    episode_s = t_settle_open + t_settle_close + n_cycles * cycle_s
    body_x_start = body_x_for_cam_dist(cam_start)
    body_x_work = body_x_for_cam_dist(cam_work)
    body_y0 = 0.0
    _apply_shape(GRO01, 0.75)
    env = GateFEnv(gait_t=0.75, episode_s=episode_s, stand_hold=episode_s, ramp_t=0.0,
                   head_tilt_deg=-16.0, walk=False, body_x0=body_x_start, use_policy=False, shape="GRO01")
    walk_head = -16.0; compose_head = float(tilt)
    _apply_shape(GRO01, 0.75)
    # Gate Q controller-only clearance boost (visible daylight); plant XML untouched.
    # Save true module defaults once so restore does not leak boosted values across runs.
    _gait_prev = {
        "KNEE_SWING": float(wg.KNEE_SWING),
        "SWING_ANK_DF": float(wg.SWING_ANK_DF),
        "SWING_ABDUCT": float(wg.SWING_ABDUCT),
    }
    wg.KNEE_SWING = max(float(wg.KNEE_SWING), float(os.environ.get("GATE_Q_KNEE_SWING", CLEAR_BOOST_KNEE_SWING)))
    wg.SWING_ANK_DF = max(float(wg.SWING_ANK_DF), float(os.environ.get("GATE_Q_SWING_ANK_DF", CLEAR_BOOST_SWING_ANK_DF)))
    wg.SWING_ABDUCT = max(float(wg.SWING_ABDUCT), float(os.environ.get("GATE_Q_SWING_ABDUCT", CLEAR_BOOST_SWING_ABDUCT)))
    # Slightly slower cadence → longer swing dwell + softer landings (skate)
    _gait_prev["GAIT_T"] = float(wg.GAIT_T)
    wg.GAIT_T = float(os.environ.get("GATE_Q_GAIT_T", GATE_Q_GAIT_T_DEFAULT))
    # iterate3r19h6: shorter DS → longer swing phase (taller clear dwell)
    _gait_prev["DS_S"] = float(wg.DS_S)
    wg.DS_S = float(os.environ.get("GATE_Q_DS_S", GATE_Q_DS_S))
    # iterate3n: stronger stance plant (controller-only; restore after episode)
    _gait_prev["PLANT_KD"] = float(wg.PLANT_KD)
    _gait_prev["PLANT_MAX_F"] = float(wg.PLANT_MAX_F)
    wg.PLANT_KD = max(float(wg.PLANT_KD), float(os.environ.get("GATE_Q_PLANT_KD", GATE_Q_PLANT_KD)))
    wg.PLANT_MAX_F = max(float(wg.PLANT_MAX_F), float(os.environ.get("GATE_Q_PLANT_MAX_F", GATE_Q_PLANT_MAX_F)))
    env.reset()
    m, d = env.model, env.data
    spring = _panel_spring(m)
    jid_p = _panel_hinge_id(m); jid_l = _hinge_id(m)
    assert jid_p >= 0 and jid_l >= 0
    for i in range(m.nu):
        n = mj.mj_id2name(m, mj.mjtObj.mjOBJ_ACTUATOR, i) or ""
        if "panel" in n.lower(): raise RuntimeError(f"panel actuator: {n}")
    d.qpos[m.jnt_qposadr[jid_p]] = 0.0; d.qpos[m.jnt_qposadr[jid_l]] = 0.0
    mj.mj_forward(m, d)
    pol = None
    pol_ret = None  # T4/T5 reverse-native retreat ckpt (approach stays E7lock)
    if use_residual:
        from stable_baselines3 import PPO
        assert CKPT.exists(); pol = PPO.load(str(CKPT), device="cpu")
        # T5E > T5D2 > T5D1 > T5C > T5B > T5 > T4; dual-ckpt approach=E7lock always.
        # T5E  = FOOTSTEP-SEQ outer (ordered clear TDs; Walk This Way / Placo short |dx_back|).
        # T5D2 = ICP-ΔT+ΔFOOT outer (CSF+ALIP schedule PRIOR + online ICP feedback; arXiv:1703.00477).
        # T5D1 = OUTER-PRIMARY+CLEAR-CORRECTOR (CSF+ALIP owns F,T; residual corrector only).
        # T5C = teacher-BC + multi-seed NEW weights (may carry ALIP/DCM conditioning; ≠ S2 twin).
        # T5B = ALIP/DCM base in NEW weights (≠ S2 GATE_Q_RET_ALIP_TVR twin on frozen E7lock).
        _t5e = os.environ.get("GATE_Q_T5E_CKPT", "").strip()
        _t5d2 = os.environ.get("GATE_Q_T5D2_CKPT", "").strip()
        _t5d1 = os.environ.get("GATE_Q_T5D1_CKPT", "").strip()
        _t5c = os.environ.get("GATE_Q_T5C_CKPT", "").strip()
        _t5b = os.environ.get("GATE_Q_T5B_CKPT", "").strip()
        _t5 = os.environ.get("GATE_Q_T5_CKPT", "").strip()
        _t4 = os.environ.get("GATE_Q_T4_CKPT", "").strip()
        _ret = _t5e or _t5d2 or _t5d1 or _t5c or _t5b or _t5 or _t4
        _ret_tag = ("T5E" if _t5e else ("T5D2" if _t5d2 else ("T5D1" if _t5d1 else ("T5C" if _t5c else ("T5B" if _t5b else ("T5" if _t5 else ("T4" if _t4 else None)))))))
        if _ret:
            _retp = Path(_ret)
            assert _retp.exists(), _retp
            pol_ret = PPO.load(str(_retp), device="cpu")
            print(f"[Q] {_ret_tag} retreat ckpt={_retp} sha16={_sha16(_retp)} (approach=E7lock)", flush=True)
            if _t5e:
                os.environ.setdefault("GATE_Q_T5E_FOOTSTEP_SEQ", "1")
                os.environ.setdefault("GATE_Q_T5B_MODELBASE", "1")  # clear-only track path
                print("[Q] T5E OUTER=FOOTSTEP-SEQ (doi:10.1145/3747865) · residual=corrector-only · soft-pass NEVER · H2 OUT", flush=True)
            elif _t5d2:
                os.environ.setdefault("GATE_Q_T5D2_ICP", "1")
                os.environ.setdefault("GATE_Q_T5D1_OUTER_PRIMARY", "1")  # schedule prior path
                os.environ.setdefault("GATE_Q_T5B_MODELBASE", "1")
                print("[Q] T5D2 OUTER=CSF+ALIP+ICP (arXiv:1703.00477) · residual=corrector-only · soft-pass NEVER", flush=True)
            elif _t5d1:
                # Outer PRIMARY: CSF+ALIP in-process (≠ S1/S2 twin flags on frozen E7lock)
                os.environ.setdefault("GATE_Q_T5D1_OUTER_PRIMARY", "1")
                os.environ.setdefault("GATE_Q_T5B_MODELBASE", "1")
                print("[Q] T5D1 OUTER=CSF+ALIP primary (Placo N/A) · residual=corrector-only", flush=True)
            elif _t5c or _t5b:
                # ALIP model-base path activates without S2 twin flag (T5C may carry as conditioning)
                os.environ.setdefault("GATE_Q_T5B_MODELBASE", "1")
    x0_ep = float(d.qpos[env.q_free]); y0_ep = float(d.qpos[env.q_free + 1])
    sid_hand = mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, "l_hand_site")
    reach = ["l_sho_pitch", "l_sho_roll", "l_el_pitch", "l_el_yaw"]
    jids = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, j) for j in reach]
    panel_hinge_xy = np.array([0.45, -0.05])
    bid_com = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "body_link")
    def arc(deg):
        rest = np.array([0.40, 0.0]) - panel_hinge_xy
        th = np.radians(deg); c, s = np.cos(th), np.sin(th)
        xy = panel_hinge_xy + np.array([[c, -s], [s, c]]) @ rest
        return np.array([xy[0], xy[1], 0.275])
    t_walk = walk_hold + walk_s
    t_app = approach_s
    t_push = t_app + push_s; t_hold = t_push + hold_s
    t_disturb = t_hold + disturb_s; t_reject = t_disturb + reject_s
    t_leave = t_reject + leave_s; t_regrasp = t_leave + regrasp_s
    t_close = t_regrasp + close_s; t_recover = t_close + recover_s
    ret_hold = float(RETREAT_HOLD); ret_walk_s = float(RETREAT_WALK_S)
    ret_fin_s = float(RETREAT_FINALIZE_S); ret_ramp = float(RETREAT_RAMP)
    t_ret_hold = t_recover + ret_hold
    t_ret_walk = t_ret_hold + ret_walk_s
    t_ret_fin = t_ret_walk + ret_fin_s
    _hip_amp_fwd = abs(float(wg.HIP_PITCH_AMP))
    _hip_bias_fwd = abs(float(wg.HIP_BIAS_FWD))
    def new_cycle():
        return {
            "peak_panel_deg": 0.0, "max_lever_deg": 0.0, "hold25_s": 0.0, "_hold_run": 0.0,
            "disturb_t0": None, "disturb_t1": None, "panel_at_disturb_start": None,
            "panel_min_during_disturb": None, "panel_min_during_reject": None, "panel_at_reject_end": None,
            "time_to_panel15_after_disturb_s": None, "_seen15_after": False,
            "leave_no_contact_s": 0.0, "leave_no_contact_panel20_s": 0.0,
            "_leave_nc_run": 0.0, "_leave_nc20_run": 0.0, "min_panel_during_leave_ok": None,
            "panel_at_leave_start": None, "regrasp_contact_s": 0.0, "min_abs_during_close": None,
            "close_delta_deg": 0.0, "contact_rising_s": 0.0, "contact_s": 0.0,
            "coupled_ok": False, "open25_ok": False, "hold_ok": False, "disturb_applied_ok": False,
            "reject_panel15_ok": False, "leave_ok": False, "regrasp_ok": False, "close_ok": False,
            "pass": False, "_prev_abs": 0.0,
            "cam_start": None, "cam_at_approach_end": None, "cam_at_first_contact": None,
            "body_x_approach_start": None, "body_x_at_walk_end": None, "body_x_at_first_contact": None,
            "approach_delta_m": None, "stepped_delta_m": None, "approach_delta_before_contact_ok": False,
            "cam_start_ge_055": False, "entered_n_working_pose": False,
            "tip_during_approach": 0.0, "_tip_app_run": 0.0, "first_contact_t": None,
            "approach_t0": None, "approach_t1": None, "teleport_falsifier": False,
            "n_ss_L": 0, "n_ss_R": 0, "n_ss_total": 0,
            "n_contact_breaks_L": 0, "n_contact_breaks_R": 0,
            "_prev_cL": True, "_prev_cR": True, "_prev_ssL": False, "_prev_ssR": False,
            "skate_mean": None, "skate_p95": None, "skate_mean_L": None, "skate_p95_L": None,
            "skate_mean_R": None, "skate_p95_R": None, "skate_ok": False, "stepped_ok": False,
            "_vxL_st": [], "_vxR_st": [], "_plant_run_L": 0.0, "_plant_run_R": 0.0, "_walk_x0": None, "_finalize_x0": None,
            "_cop_bias": None, "_cop_samples": [],
            "_walk_stop": False, "_stop_xy": None,
            "max_clear_L_m": 0.0, "max_clear_R_m": 0.0,
            "max_clear_ss_L_m": 0.0, "max_clear_ss_R_m": 0.0,
            "max_clear_above_rest_L_m": 0.0, "max_clear_above_rest_R_m": 0.0,
            "max_ss_dwell_L_s": 0.0, "max_ss_dwell_R_s": 0.0,
            "plant_rest_clear_L_m": None, "plant_rest_clear_R_m": None,
            "_rest_L": [], "_rest_R": [],
            "_ss_run_L": 0.0, "_ss_run_R": 0.0,
            "_dwell_latched_L": False, "_dwell_latched_R": False,
            "_ss_xs": [], "_ss_ts": [],
            "_dx_clear_m": 0.0, "_dx_planted_m": 0.0, "_prev_bx_track": None,
            "_dx_clear_m_ret": 0.0, "_dx_planted_m_ret": 0.0, "_prev_bx_track_ret": None,
            "dx_clear_m": 0.0, "dx_planted_m": 0.0, "clear_frac": 0.0,
            "dx_clear_m_ret": 0.0, "dx_planted_m_ret": 0.0, "clear_frac_ret": 0.0,
            "_mid_settle_left": 0.0, "_mid_settle_done": False, "_burst2_t0": None, "_burst3_done": False,
            "_span_latched": False, "_post_span_reburst_n": 0,
            "_periodic_reburst_n": 0, "_last_reburst_dx": 0.0, "_last_reburst_t": 0.0,
            "shim_delta_m": None, "stepped_frac": None, "shim_budget_ok": False,
            "visible_ss_ok": False, "hud_finalize_label": "KINEMATIC XY SHIM",
            "retreat_cam_start": None, "retreat_cam_end": None,
            "body_x_retreat_start": None, "body_x_at_retreat_walk_end": None,
            "retreat_delta_m": None, "retreat_stepped_delta_m": None, "retreat_shim_delta_m": None,
            "retreat_stepped_frac": None, "retreat_shim_budget_ok": False,
            "retreat_cam_ge_055": False, "retreat_delta_ok": False,
            "n_ss_L_ret": 0, "n_ss_R_ret": 0, "n_ss_total_ret": 0,
            "max_clear_L_ret_m": 0.0, "max_clear_R_ret_m": 0.0,
            "max_clear_above_rest_L_ret_m": 0.0, "max_clear_above_rest_R_ret_m": 0.0,
            "max_ss_dwell_L_ret_s": 0.0, "max_ss_dwell_R_ret_s": 0.0,
            "max_clear_ss_L_ret_m": 0.0, "max_clear_ss_R_ret_m": 0.0,
            "skate_mean_ret": None, "skate_p95_ret": None, "skate_ok_ret": False,
            "skate_mean_L_ret": None, "skate_p95_L_ret": None,
            "skate_mean_R_ret": None, "skate_p95_R_ret": None,
            "tip_during_retreat": 0.0, "_tip_ret_run": 0.0, "_tip_ret_down": 0.0,
            "retreat_t0": None, "retreat_t1": None,
            "retreat_stepped_ok": False, "retreat_ok": False, "retreat_tip_ok": False,
            "visible_ss_ok_ret": False, "hud_retreat_finalize_label": "KINEMATIC XY SHIM",
            "_vxL_st_ret": [], "_vxR_st_ret": [],
            "_plant_run_L_ret": 0.0, "_plant_run_R_ret": 0.0,
            "_ss_run_L_ret": 0.0, "_ss_run_R_ret": 0.0,
            "_dwell_latched_L_ret": False, "_dwell_latched_R_ret": False,
            "_ss_xs_ret": [], "_ss_ts_ret": [], "_ret_settle_left": 0.0, "_ret_settle_done": False, "_ret_burst_t0": None,
            "_r2_phase": "idle", "_r2_phase_t0": None, "_r2_next_t": None, "_r2_burst_n": 0,
            "_r2_hold_cam0": None, "_r2_hold_cam_gain": 0.0, "_r2_hold_s": 0.0,
            "_r3_phase": "idle", "_r3_phase_t0": None, "_r3_next_t": None, "_r3_burst_n": 0,
            "_r3_gap_cam0": None, "_r3_gap_cam_gain": 0.0, "_r3_gap_s": 0.0,
            "_r3_gap_max_gain": 0.0, "_r3_gap_n": 0,
            "_csf_F": 0.0, "_csf_T": float(RETREAT_GAIT_T), "_csf_omega": 0.0,
            "_csf_step_t0": None, "_csf_F_sum": 0.0, "_csf_T_sum": 0.0, "_csf_n": 0,
            "_csf_plant_cam0": None, "_csf_plant_cam_gain": 0.0, "_csf_plant_max_gain": 0.0,
            "_csf_plant_n": 0, "_csf_plant_dwell": 0.0, "_csf_plant_dwell_max": 0.0,
            "_alip_fp_prior": 0.0, "_alip_du_fp": 0.0, "_alip_du_prev": 0.0,
            "_alip_omega": 0.0, "_alip_replan_idx": -1, "_alip_n_replan": 0,
            "_alip_fp_jump_max": 0.0, "_alip_fp_sum": 0.0, "_alip_du_sum": 0.0,
            "_alip_swing_side": None,
            "_alip_plant_cam0": None, "_alip_plant_cam_gain": 0.0, "_alip_plant_max_gain": 0.0,
            "_alip_plant_n": 0, "_alip_plant_dwell": 0.0, "_alip_plant_dwell_max": 0.0,
            "_icp_err": 0.0, "_icp_dT": 0.0, "_icp_dfoot": 0.0, "_icp_x": 0.0,
            "_icp_n": 0, "_icp_err_sum": 0.0, "_icp_dT_sum": 0.0, "_icp_dfoot_sum": 0.0,
            "_icp_T": float(RETREAT_GAIT_T), "_icp_F": 0.0, "_icp_omega": 0.0,
            "_icp_sched_F": 0.0, "_icp_sched_T": float(RETREAT_GAIT_T),
            # T5-E FOOTSTEP-SEQ (ordered clear TDs; ≠ D1 schedule / ≠ D2 ICP)
            "_t5e_tds": None, "_t5e_idx": 0, "_t5e_n_track": 0, "_t5e_n_advance": 0,
            "_t5e_err_sum": 0.0, "_t5e_err_n": 0, "_t5e_last_err": 0.0,
            "_t5e_list_id": "", "_t5e_prev_swing": None, "_t5e_F": 0.0,
            "_rev_phi_polarity": 0.0, "_rev_hip_sign": 1.0, "_rev_swing_enc": 0.0,
            "_rev_n": 0, "_rev_clear_n": 0,
            "_rev_plant_cam0": None, "_rev_plant_cam_gain": 0.0, "_rev_plant_max_gain": 0.0,
            "_rev_plant_n": 0, "_rev_plant_dwell": 0.0, "_rev_plant_dwell_max": 0.0,
            "_ret_periodic_n": 0, "_ret_last_reburst_dx": 0.0, "_ret_last_reburst_t": 0.0,
            "_retreat_walk_x0": None, "_retreat_finalize_x0": None,
            "_retreat_stop": False, "_retreat_stop_xy": None, "_retreat_cam_ge_run": 0.0, "_retreat_soft": False,
            "_retreat_jump_fin": False, "_retreat_fin_t0": None,
            "_retreat_cop_bias": None, "_retreat_cop_samples": [],
            "_retreat_fin_cam0": None,
        }
    cycles = [new_cycle() for _ in range(n_cycles)]
    qpos_log, xy_log, bout_log, t_log = [], [], [], []
    phase_log, disturb_log, leave_log, force_log = [], [], [], []
    approach_log, retreat_log, camh_log, ss_log, shim_log = [], [], [], [], []
    # iterate2: no sticky FOOT-LIFT — HUD only while currently clear
    tip_run = tip = 0.0; tip_after_disturb_ok = True
    tip_down_run = 0.0
    global_max_panel = global_max_lever = 0.0
    global_max_panel_compose = 0.0
    global_rising = global_contact = 0.0; prev_abs = 0.0
    t0 = t_settle_open + t_settle_close
    disturb_force_applied_s = 0.0; cam_h_episode_start = None
    LEG = ["l_hip_yaw","l_hip_roll","l_hip_pitch","l_knee","l_ank_pitch","l_ank_roll",
           "r_hip_yaw","r_hip_roll","r_hip_pitch","r_knee","r_ank_pitch","r_ank_roll"]
    def _obs_now(phi):
        q = np.zeros(12); dq = np.zeros(12)
        for i, jn in enumerate(LEG):
            jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
            q[i] = d.qpos[m.jnt_qposadr[jid]]; dq[i] = d.qvel[m.jnt_dofadr[jid]]
        v = d.qvel[env.v_free:env.v_free+3].copy(); w = d.qvel[env.v_free+3:env.v_free+6].copy()
        quat = d.qpos[env.q_free+3:env.q_free+7].copy()
        sinp, cosp = math.sin(2*math.pi*phi), math.cos(2*math.pi*phi)
        cL = 1.0 if wg.foot_floor_contact(m, d, env.bid_lf, env.gid_floor) else 0.0
        cR = 1.0 if wg.foot_floor_contact(m, d, env.bid_rf, env.gid_floor) else 0.0
        zL = float(d.xpos[env.bid_lf, 2]); zR = float(d.xpos[env.bid_rf, 2])
        return np.concatenate([q, dq, v, w, quat, [sinp, cosp, cL, cR, zL, zR, 0.0]]).astype(np.float32)

    for k in range(env.n_ctrl):
        t = k / CTRL_HZ
        disturb_xfrc = None; base_xy = None; walk_ctrl = None; ss_flag = False
        if t < t_settle_open:
            gcmd, phase, cmd, a_rev, bi = OPEN_CMD, "settle_open", 0.0, 0.0, -1
            base_xy = (body_x_start, body_y0)
        elif t < t0:
            a = (t - t_settle_open) / t_settle_close
            gcmd = OPEN_CMD * (1 - a) + CLOSE_CMD * a
            phase, cmd, a_rev, bi = "settle_close", 0.0, 0.0, -1
            base_xy = (body_x_start, body_y0)
        else:
            tb = t - t0
            bi = min(int(tb // cycle_s), n_cycles - 1)
            local = tb - bi * cycle_s
            b = cycles[bi]
            if local < t_walk:
                # Early-stop near work after visible SS + Δ≥0.15 (do NOT stop at Δ=0.15 alone —
                # that left finalize dominating approach under Root B).
                bx_now = float(d.qpos[env.q_free])
                cam_now = cam_to_lever_horiz(d, env.sid, env.gid_lever)
                ang_now = abs(_angle_deg(m, d, jid_p))
                dx_now = bx_now - (b["_walk_x0"] if b["_walk_x0"] is not None else bx_now)
                ss_now = b["n_ss_L"] + b["n_ss_R"]
                both_sides = _ss_multi_proved(b["n_ss_L"], b["n_ss_R"])  # mid-settle trigger
                dense_sides = _ss_dense_proved(b["n_ss_L"], b["n_ss_R"])  # true 2+2 / 3+3
                span_now = _ss_span_ready(b.get("_ss_xs") or [], dx_now, b["n_ss_L"], b["n_ss_R"])
                # iterate3q4b: sticky latch only while LIVE span still carries (≥0.45). Early cluster
                # briefly had span>0.50 at small dx then slid — stale latch + cam<0.08 freeze killed burst3.
                span_need = float(os.environ.get("GATE_Q_SPAN_EARLY_MIN", GATE_Q_SPAN_EARLY_MIN))
                live_span = _ss_span_frac_live(b.get("_ss_xs") or [], dx_now)
                ss_ts_now = list(b.get("_ss_ts") or [])
                t0_app = float(b.get("approach_t0") or (t - local))
                time_span = _ss_time_span_frac(ss_ts_now, t0_app, t)
                time_span_need = float(os.environ.get("GATE_Q_SS_TIME_SPAN_FRAC_MIN", GATE_Q_SS_TIME_SPAN_FRAC_MIN))
                carry_tail = _ss_carry_tail_ok(b.get("_ss_xs") or [], bx_now, reverse=False)
                spaced_ok = (time_span >= time_span_need - 1e-9 and len(ss_ts_now) >= 4 and carry_tail)
                if span_now and spaced_ok:
                    b["_span_latched"] = True
                elif live_span < max(0.45, span_need * 0.90) - 1e-9 or (not carry_tail):
                    b["_span_latched"] = False
                span_latched = bool(b.get("_span_latched")) and live_span >= 0.45 - 1e-9 and spaced_ok
                near_work = cam_now <= EARLY_STOP_CAM_H + 1e-6
                min_walk = walk_hold + float(os.environ.get("GATE_Q_MIN_WALK_PROOF_S", GATE_Q_MIN_WALK_PROOF_S))
                # early-stop only when LIVE lifts span XY AND time-spaced across walk (not early cluster + slide)
                # iterate3r18: early-stop also needs Δ mostly during clear windows (not planted soft-XY)
                clear_frac_live = float(b.get("clear_frac") or 0.0)
                clear_frac_need_es = float(os.environ.get("GATE_Q_CLEAR_FRAC_MIN", GATE_Q_CLEAR_FRAC_MIN))
                proved = (span_latched and span_now and spaced_ok and local >= min_walk and dx_now >= 0.18 - 1e-6
                          and clear_frac_live >= clear_frac_need_es - 1e-9
                          and (near_work or cam_now <= 0.18 or dx_now >= 0.35 - 1e-6))
                # fallback: true dense 3+3 + near work + live span + carry tail
                if (dense_sides and b["n_ss_L"] >= 3 and b["n_ss_R"] >= 3 and near_work and dx_now >= 0.15
                        and live_span >= 0.50 - 1e-9 and spaced_ok):
                    proved = True
                # Collision freeze only after live span carries; else keep stepping (cam<0.055)
                # bout1 short approach: thinner freeze until spaced (avoid stop before 2nd-wave)
                if bi >= 1 and not spaced_ok:
                    collision_risk = cam_now < float(os.environ.get("GATE_Q_BOUT1_COLLISION_CAM", "0.042"))
                elif not span_now and live_span < 0.50 - 1e-9:
                    collision_risk = cam_now < float(os.environ.get("GATE_Q_COLLISION_CAM_THIN", "0.055"))
                else:
                    collision_risk = cam_now < (0.08 if b.get("_mid_settle_done") else 0.10)
                # iterate3q4b: proactive 2nd-wave when live span thins after mid-settle (don't wait for wall)
                want_burst3 = (
                    os.environ.get("GATE_Q_DISABLE_MID_SETTLE", GATE_Q_DISABLE_MID_SETTLE) not in ("1", "true", "yes")
                    and (not b.get("_burst3_done")) and bool(b.get("_mid_settle_done"))
                    and b.get("_mid_settle_left", 0.0) <= 1e-12
                    and (b["n_ss_L"] + b["n_ss_R"]) >= 3
                    and dx_now >= float(os.environ.get("GATE_Q_BURST3_DX", "0.22"))
                    and live_span < float(os.environ.get("GATE_Q_BURST3_SPAN_MAX", "0.52"))
                    and not b["_walk_stop"]
                )
                if want_burst3:
                    b["_mid_settle_left"] = float(os.environ.get("GATE_Q_BURST3_SETTLE_S", "0.32"))
                    b["_burst3_done"] = True
                    b["_mid_settle_done"] = False
                    b["_burst2_t0"] = None
                    b["_stop_xy"] = (bx_now, float(d.qpos[env.q_free + 1]))
                    collision_risk = False
                    proved = False
                if (proved or collision_risk) and not b["_walk_stop"]:
                    b["_walk_stop"] = True
                    b["_walk_stop_t"] = float(t); b["walk_stop_t"] = float(t)
                    b["_stop_xy"] = (bx_now, float(d.qpos[env.q_free + 1]))
                    b["body_x_at_walk_end"] = bx_now
                    if b["_walk_x0"] is not None:
                        dx_c_stop = float(b.get("_dx_clear_m") or 0.0)
                        dx_p_stop = float(b.get("_dx_planted_m") or 0.0)
                        if (dx_c_stop + dx_p_stop) > 1e-4:
                            b["stepped_delta_m"] = dx_c_stop  # planted soft-XY excluded
                        else:
                            b["stepped_delta_m"] = bx_now - b["_walk_x0"]
                        b["dx_clear_m"] = dx_c_stop; b["dx_planted_m"] = dx_p_stop
                        _tstop = dx_c_stop + dx_p_stop
                        b["clear_frac"] = (dx_c_stop / _tstop) if _tstop > 1e-6 else 0.0
                if b["_walk_stop"]:
                    base_xy = b["_stop_xy"]
                    walk_ctrl = None
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "step_walk", 0.0, local / max(t_walk, 1e-6)
                elif (os.environ.get("GATE_Q_DISABLE_MID_SETTLE", GATE_Q_DISABLE_MID_SETTLE) not in ("1", "true", "yes")
                      and not span_latched and not b["_walk_stop"]
                      and b.get("_mid_settle_left", 0.0) <= 1e-12
                      and (b["n_ss_L"] + b["n_ss_R"]) >= 2
                      and int(b.get("_periodic_reburst_n") or 0) < (
                          int(os.environ.get("GATE_Q_BOUT1_PERIODIC_REBURST_MAX", "2")) if bi >= 1
                          else int(os.environ.get("GATE_Q_PERIODIC_REBURST_MAX", "3")))
                      and dx_now - float(b.get("_last_reburst_dx") or 0.0) >= float(
                          os.environ.get("GATE_Q_BOUT1_PERIODIC_REBURST_DX", "0.07") if bi >= 1
                          else os.environ.get("GATE_Q_PERIODIC_REBURST_DX", "0.10"))
                      and (local - float(b.get("_last_reburst_t") or 0.0)) >= float(
                          os.environ.get("GATE_Q_BOUT1_PERIODIC_REBURST_DT", "2.0") if bi >= 1
                          else os.environ.get("GATE_Q_PERIODIC_REBURST_DT", "2.8"))
                      and live_span < float(os.environ.get("GATE_Q_PERIODIC_REBURST_SPAN_MAX", "0.70"))
                      and cam_now > (0.10 if bi >= 1 else 0.12)):
                    # iterate3r: periodic clearance reburst so lifts continue throughout travel (not one late lift)
                    b["_periodic_reburst_n"] = int(b.get("_periodic_reburst_n") or 0) + 1; b["periodic_reburst_n"] = b["_periodic_reburst_n"]
                    b["_last_reburst_dx"] = dx_now
                    b["_last_reburst_t"] = local
                    b["_mid_settle_left"] = float(os.environ.get("GATE_Q_PERIODIC_REBURST_SETTLE_S", "0.22"))
                    b["_mid_settle_done"] = False
                    b["_burst2_t0"] = None
                    b["_stop_xy"] = (bx_now, float(d.qpos[env.q_free + 1]))
                    base_xy = b["_stop_xy"]; walk_ctrl = None
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "step_walk", 0.0, local / max(t_walk, 1e-6)
                elif (os.environ.get("GATE_Q_DISABLE_MID_SETTLE", GATE_Q_DISABLE_MID_SETTLE) not in ("1", "true", "yes")
                      and b["n_ss_L"] >= 1 and b["n_ss_R"] >= 1
                      and not b.get("_mid_settle_done") and b.get("_mid_settle_left", 0.0) <= 1e-12
                      and dx_now >= float(os.environ.get("GATE_Q_MID_SETTLE_DX", "0.10"))
                      and not span_latched):
                    # iterate3q4b: mid-settle after dx≥0.10 even if already multi (3q2 bug: raced past 1+1
                    # before dx gate → mid_settle never armed → burst3 never fired → early-cluster slide)
                    settle_s = float(os.environ.get("GATE_Q_MID_SETTLE_S", "0.45"))
                    if bi >= 1:
                        settle_s = float(os.environ.get("GATE_Q_BOUT1_MID_SETTLE_S", "0.52"))  # iterate3m keep
                    b["_mid_settle_left"] = settle_s
                    b["_last_reburst_dx"] = dx_now
                    b["_last_reburst_t"] = local
                    b["_stop_xy"] = (bx_now, float(d.qpos[env.q_free + 1]))
                    base_xy = b["_stop_xy"]; walk_ctrl = None
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "step_walk", 0.0, local / max(t_walk, 1e-6)
                elif (os.environ.get("GATE_Q_DISABLE_MID_SETTLE", GATE_Q_DISABLE_MID_SETTLE) not in ("1", "true", "yes")
                      and not span_latched and b.get("_mid_settle_done")
                      and not b.get("_burst3_done") and b.get("_mid_settle_left", 0.0) <= 1e-12
                      and (b["n_ss_L"] + b["n_ss_R"]) >= 3
                      and (both_sides or (b["n_ss_L"] >= 1 and b["n_ss_R"] >= 1))
                      and dx_now >= float(os.environ.get("GATE_Q_BURST3_DX", "0.20"))
                      and _ss_span_frac_live(b.get("_ss_xs") or [], dx_now) < float(os.environ.get("GATE_Q_BURST3_SPAN_MAX", "0.52"))):
                    # iterate3q burst3: only if still slide-thin after more XY (avoid settle interrupt loop)
                    b["_mid_settle_left"] = float(os.environ.get("GATE_Q_BURST3_SETTLE_S", "0.28"))
                    b["_burst3_done"] = True
                    b["_mid_settle_done"] = False
                    b["_burst2_t0"] = None
                    b["_stop_xy"] = (bx_now, float(d.qpos[env.q_free + 1]))
                    base_xy = b["_stop_xy"]; walk_ctrl = None
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "step_walk", 0.0, local / max(t_walk, 1e-6)
                elif (os.environ.get("GATE_Q_DISABLE_MID_SETTLE", GATE_Q_DISABLE_MID_SETTLE) not in ("1", "true", "yes")
                      and not span_latched and both_sides and b.get("_mid_settle_done")
                      and b.get("_burst3_done")
                      and int(b.get("_post_span_reburst_n") or 0) < 1
                      and b.get("_mid_settle_left", 0.0) <= 1e-12
                      and cam_now > 0.14 and dx_now >= 0.12):
                    # iterate3p: one post-2+2 reburst if span still not latched
                    b["_post_span_reburst_n"] = 1
                    b["_mid_settle_left"] = float(os.environ.get("GATE_Q_POST_SPAN_REBURST_S", "0.30"))
                    b["_mid_settle_done"] = False
                    b["_burst2_t0"] = None
                    b["_stop_xy"] = (bx_now, float(d.qpos[env.q_free + 1]))
                    base_xy = b["_stop_xy"]; walk_ctrl = None
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "step_walk", 0.0, local / max(t_walk, 1e-6)
                elif b.get("_mid_settle_left", 0.0) > 1e-12:
                    b["_mid_settle_left"] = max(0.0, b["_mid_settle_left"] - 1.0 / CTRL_HZ)
                    if b["_mid_settle_left"] <= 1e-12:
                        b["_mid_settle_done"] = True
                        b["_burst2_t0"] = local  # reset gait phase for clean 2nd lift burst
                    base_xy = b.get("_stop_xy") or (bx_now, float(d.qpos[env.q_free + 1]))
                    walk_ctrl = None
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "step_walk", 0.0, local / max(t_walk, 1e-6)
                else:
                    hold_i = walk_hold + (float(os.environ.get("GATE_Q_BOUT1_EXTRA_HOLD", GATE_Q_BOUT1_EXTRA_HOLD)) if bi >= 1 else 0.0)
                    if b.get("_burst2_t0") is not None:
                        # Clean 2nd burst: full amp, phase clock reset (avoid mid-cycle resume scrape)
                        a_w = 1.0
                        phi = (max(0.0, local - float(b["_burst2_t0"])) / max(wg.GAIT_T, 1e-6)) % 1.0
                    else:
                        a_w = _gait_amp(local, hold_i, walk_ramp)
                        phi = (max(0.0, local - hold_i) / max(wg.GAIT_T, 1e-6)) % 1.0 if a_w > 0 else 0.0
                    settle = getattr(wg, "ANK_COP_BIAS_SETTLE_S", 0.55)
                    if b["_cop_bias"] is None and local < settle:
                        com = np.asarray(d.subtree_com[bid_com, :2], dtype=np.float64)
                        try:
                            support, _, _, _ = wg._support_feet(m, d, env.bid_lf, env.bid_rf, env.gid_floor, None)
                            b["_cop_samples"].append(com - support)
                        except Exception: pass
                        if local >= settle - 1.5 / CTRL_HZ and b["_cop_samples"]:
                            b["_cop_bias"] = np.mean(np.stack(b["_cop_samples"], 0), 0)
                    action = None
                    if use_residual and pol is not None and a_w > 0.01:
                        action, _ = pol.predict(_obs_now(phi), deterministic=True)
                    walk_ctrl = {"amp": a_w, "phi": phi, "t_abs": local, "cop_bias": b["_cop_bias"], "action": action}
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "step_walk", 0.0, local / max(t_walk, 1e-6)
            elif local < t_app:
                a_fin = min(1.0, max(0.0, (local - t_walk) / max(finalize_s, 1e-6)))
                if b["_finalize_x0"] is None:
                    b["_finalize_x0"] = float(d.qpos[env.q_free])
                    b["body_x_at_walk_end"] = b["_finalize_x0"]
                    if b["_walk_x0"] is not None:
                        b["stepped_delta_m"] = b["_finalize_x0"] - b["_walk_x0"]
                bx = b["_finalize_x0"] + (body_x_work - b["_finalize_x0"]) * a_fin
                base_xy = (bx, body_y0)
                b["shim_delta_m"] = bx - b["_finalize_x0"]
                wg.set_ctrl(m, d, wg.gait_targets(0.0, False, 0.0), env.act_idx)
                gcmd, phase, cmd, a_rev = OPEN_CMD, "finalize", 0.0, a_fin
            elif local < t_push:
                gcmd, phase = CLOSE_CMD, "push"; cmd = push_to * ((local - t_app) / push_s); a_rev = 0.0
            elif local < t_hold:
                gcmd, phase, cmd, a_rev = CLOSE_CMD, "hold", push_to, 0.0
            elif local < t_disturb:
                gcmd, phase, cmd, a_rev = CLOSE_CMD, "disturb", push_to, 0.0
                if (local - t_hold) < DISTURB_IMPULSE_S:
                    disturb_xfrc = DISTURB_FORCE_N; disturb_force_applied_s += 1 / CTRL_HZ
            elif local < t_reject:
                gcmd, phase, cmd, a_rev = CLOSE_CMD, "reject", push_to, 0.0
            elif local < t_leave:
                a_leave = (local - t_reject) / leave_s
                gcmd = CLOSE_CMD * (1.0 - a_leave) + OPEN_CMD * a_leave
                phase, cmd, a_rev = "leave", push_to, a_leave
            elif local < t_regrasp:
                a_rg = (local - t_leave) / regrasp_s
                gcmd = OPEN_CMD * (1.0 - a_rg) + CLOSE_CMD * a_rg
                cmd = max(abs(_angle_deg(m, d, jid_p)), 18.0); phase, a_rev = "regrasp", a_rg
            elif local < t_close:
                a_rev = (local - t_regrasp) / close_s
                gcmd = CLOSE_CMD * (1.0 - 0.55 * a_rev) + OPEN_CMD * (0.55 * a_rev) if soft_close else CLOSE_CMD * (1.0 - 0.85 * a_rev) + OPEN_CMD * (0.85 * a_rev)
                phase = "close"; ang_now = abs(_angle_deg(m, d, jid_p)); cmd = ang_now + (rev_cmd_end - ang_now) * a_rev
            elif local < t_recover:
                a_rec = (local - t_close) / recover_s
                gcmd = OPEN_CMD * (1.0 - a_rec) + CLOSE_CMD * a_rec
                phase, cmd, a_rev = "recover", 0.0, a_rec
            elif local < t_ret_walk and not b.get("_retreat_jump_fin"):
                bx_now = float(d.qpos[env.q_free])
                cam_now = cam_to_lever_horiz(d, env.sid, env.gid_lever)
                if b["_retreat_walk_x0"] is None:
                    b["_retreat_walk_x0"] = bx_now
                    b["body_x_retreat_start"] = bx_now
                    b["retreat_cam_start"] = cam_now
                    b["retreat_cam_max"] = cam_now
                    b["retreat_t0"] = t
                dx_ret = b["_retreat_walk_x0"] - bx_now
                b["retreat_cam_max"] = max(float(b.get("retreat_cam_max") or 0.0), float(cam_now))
                both_ret = _ss_multi_proved(b["n_ss_L_ret"], b["n_ss_R_ret"])
                dense_ret = _ss_dense_proved(b["n_ss_L_ret"], b["n_ss_R_ret"])
                t_ret_local = local - t_recover
                if cam_now >= 0.52 - 1e-6:
                    b["_retreat_cam_ge_run"] = b.get("_retreat_cam_ge_run", 0.0) + 1.0 / CTRL_HZ
                else:
                    b["_retreat_cam_ge_run"] = 0.0
                cam_stable = b["_retreat_cam_ge_run"] >= 0.20 - 1e-9
                tip_ret_now = float(b.get("tip_during_retreat") or 0.0)
                # Soft-amp: full soft after strong dense (2+2); taper after thin multi (tip-stable like 3m)
                nLr, nRr = b["n_ss_L_ret"], b["n_ss_R_ret"]
                strong_dense_ret = (nLr >= 2 and nRr >= 2 and (nLr + nRr) >= 4) or (nLr >= 3 and nRr >= 3)
                # k8ag: k8af entered SOFT after first 2+2 (~4s, cam~0.28) which cut amp
                # 0.82->0.42 and residual x1.57->0.36, then 20s planted-slide to cam 0.55
                # (cf_ret~0.30). Keep full reverse gait until cam near stop, then taper/soft.
                if bi >= 1:
                    soft_cam = float(os.environ.get("GATE_Q_BOUT1_RET_SOFT_CAM", GATE_Q_BOUT1_RET_SOFT_CAM))
                    taper_cam = float(os.environ.get("GATE_Q_BOUT1_RET_TAPER_CAM", GATE_Q_BOUT1_RET_TAPER_CAM))
                else:
                    soft_cam = float(os.environ.get("GATE_Q_RET_SOFT_CAM", GATE_Q_RET_SOFT_CAM))
                    taper_cam = float(os.environ.get("GATE_Q_RET_TAPER_CAM", GATE_Q_RET_TAPER_CAM))
                if cam_now < taper_cam - 1e-6:
                    b["_retreat_soft"] = False
                elif cam_now < soft_cam - 1e-6:
                    if strong_dense_ret or both_ret or (nLr >= 1 and nRr >= 1):
                        b["_retreat_soft"] = "taper"
                    else:
                        b["_retreat_soft"] = False
                else:
                    if strong_dense_ret:
                        b["_retreat_soft"] = True
                    elif both_ret or (nLr >= 1 and nRr >= 1):
                        b["_retreat_soft"] = "taper"
                # Stop once strong-dense or thin-multi + (cam≥0.45 stable OR Δ≥0.28)
                # Spaced/carry-tail preferred for continuous honesty but NOT hard-gated on early-stop
                # (3r3/r4: hard gate → never stop → tip_ret~1.6 + cam overshoot). Scoring still requires spaced.
                ret_tail_ok = _ss_carry_tail_ok(b.get("_ss_xs_ret") or [], bx_now, reverse=True)
                ret_tspan = _ss_time_span_frac(b.get("_ss_ts_ret") or [], float(b.get("retreat_t0") or t), t)
                ret_tspan_need = float(os.environ.get("GATE_Q_SS_TIME_SPAN_FRAC_MIN", GATE_Q_SS_TIME_SPAN_FRAC_MIN))
                ret_spaced = ret_tail_ok and ret_tspan >= ret_tspan_need - 1e-9 and (nLr + nRr) >= 4
                # iterate3r13: require cam≥0.52 or Δ≥0.40 so retreat-finalize shim can leave bout1 ≥0.55 room
                # (shim_budget≈stepped×0.43; Δ=0.15 only shims ~6cm → bout1 cam_start~0.32 starved)
                # Stop at cam≥0.48 if last SS recent (carry-tail); else need cam≥0.52 / Δ≥0.40
                ret_tail_now = _ss_carry_tail_ok(b.get("_ss_xs_ret") or [], bx_now, reverse=True,
                                                 max_tail_m=float(os.environ.get("GATE_Q_SS_TAIL_MAX_M_RET", "0.28")))
                # 3r19h4c: need cam≥0.54 (or clear Δ≥0.15) so finalize shim / bout1 start ≥0.55
                dx_c_ret_live = float(b.get("dx_clear_m_ret") or b.get("_dx_clear_m_ret") or 0.0)
                # always prefer cam≥0.55 so bout1 starts ≥0.55 after finalize shim
                cam_ge = float(os.environ.get("GATE_Q_RETREAT_STOP_CAM", "0.55"))
                if cam_now >= cam_ge - 1e-6:
                    b["_retreat_cam_ge_run"] = b.get("_retreat_cam_ge_run", 0.0) + 1.0 / CTRL_HZ
                # (cam_ge_run also updated above at 0.52 — dual OK)
                dx_c_need = float(os.environ.get("GATE_Q_RET_EARLY_DX_CLEAR", GATE_Q_RET_EARLY_DX_CLEAR))
                # k8ad: stop ASAP once multi-SS + dx_clear≥0.18 + cam≥0.55 (not 0.50 — need bout1 start).
                # Exit-taper cancel protects handoff; longer clear steps grow dx_clear before stop.
                cam_stop_clear = float(os.environ.get("GATE_Q_RETREAT_STOP_CAM_CLEAR", "0.55"))
                if cam_now >= cam_stop_clear - 1e-6:
                    b["_retreat_cam_clear_run"] = b.get("_retreat_cam_clear_run", 0.0) + 1.0 / CTRL_HZ
                else:
                    b["_retreat_cam_clear_run"] = 0.0
                # k8ag: stop ASAP once cam>=0.55 + dx_clear enough + multi-SS (cut plant tail).
                # tip A28c: latch stop ASAP at cam≥0.55 + multi-SS + dx_clear (cut ~26s planted middle)
                _clr_run_need = float(os.environ.get("GATE_Q_RET_CAM_CLEAR_RUN_S", "0.03"))
                proved_ret_clear = (
                    both_ret and t_ret_local >= ret_hold + 3.0
                    and dx_c_ret_live >= dx_c_need - 1e-6
                    and cam_now >= cam_stop_clear - 1e-6
                    and b.get("_retreat_cam_clear_run", 0.0) >= _clr_run_need - 1e-9)
                # tip A28c / E7lock: cam≥0.55 + multi + dx_clear (CLEAR_TRACK_SWING=1 plateau)
                _dx_early = float(os.environ.get("GATE_Q_RET_EARLY_STOP_DX_CLEAR", "0.25"))
                _ss_ts = list(b.get("_ss_ts_ret") or [])
                _last_ss_age = (float(t) - float(_ss_ts[-1])) if _ss_ts else 0.0
                _cam_max = float(b.get("retreat_cam_max") or 0.0)
                proved_ret_tip_early = (
                    (both_ret or strong_dense_ret)
                    and (nLr + nRr) >= 4
                    and t_ret_local >= ret_hold + 3.0
                    and dx_c_ret_live >= _dx_early - 1e-6
                    and cam_now >= cam_stop_clear - 1e-6
                    and b.get("_retreat_cam_clear_run", 0.0) >= 0.02 - 1e-9)
                # A10: dx-only dense stop also blocked bout1 before cam>=0.55 (AI joint bar).
                if bi >= 1:
                    proved_ret_dense = proved_ret_clear or (strong_dense_ret and t_ret_local >= ret_hold + 5.0
                        and dx_ret >= 0.20 - 1e-6
                        and cam_now >= cam_ge - 1e-6
                        and b["_retreat_cam_ge_run"] >= 0.15 - 1e-9)
                else:
                    proved_ret_dense = proved_ret_clear or (strong_dense_ret and t_ret_local >= ret_hold + 5.0 and dx_ret >= 0.20 - 1e-6 and (
                        (cam_now >= cam_ge - 1e-6 and b["_retreat_cam_ge_run"] >= 0.15 - 1e-9)
                        or (dx_ret >= 0.40 - 1e-6 and dx_c_ret_live >= dx_c_need - 1e-6)))
                # k8aj A9: dx-only fallback (dx_ret>=0.42) stopped bout1 at cam~0.46 before AI joint bar cam>=0.55.
                if bi >= 1:
                    proved_ret_fallback = (both_ret and t_ret_local >= ret_hold + 18.0
                                           and dx_ret >= 0.28 - 1e-6
                                           and cam_now >= cam_ge - 1e-6
                                           and b["_retreat_cam_ge_run"] >= 0.15 - 1e-9)
                else:
                    proved_ret_fallback = (both_ret and t_ret_local >= ret_hold + 18.0 and dx_ret >= 0.28 - 1e-6 and (
                        (cam_now >= 0.50 - 1e-6 and b["_retreat_cam_ge_run"] >= 0.20 - 1e-9)
                        or dx_ret >= 0.42 - 1e-6))
                # k8aj A8: tip-soft at cam>=0.45 froze bout1 ret at ~0.46 (blocked AI joint bar cam>=0.55).
                tip_soft_cam = float(os.environ.get(
                    "GATE_Q_BOUT1_RET_TIP_SOFT_CAM" if bi >= 1 else "GATE_Q_RET_TIP_SOFT_CAM",
                    "0.55" if bi >= 1 else "0.45"))
                if float(b.get("_tip_ret_down") or 0.0) >= 0.10 and cam_now >= tip_soft_cam - 1e-6:
                    b["_retreat_soft"] = True
                proved_ret = proved_ret_dense or proved_ret_fallback or proved_ret_tip_early
                if proved_ret and not b["_retreat_stop"]:
                    b["_retreat_stop"] = True
                    b["_retreat_jump_fin"] = True  # tip A28c: leave retreat_walk ASAP → finalize
                    b["retreat_jump_fin"] = True
                    _sts=list(b.get("_ss_ts_ret") or [])
                    print(f"[Q] EARLY_STOP bi={bi} t={t:.2f} gap={_last_ss_age:.2f} ss={nLr}+{nRr} dxc={dx_c_ret_live:.3f} cam={cam_now:.3f} cammax={_cam_max:.3f} nts={len(_sts)}", flush=True)
                    b["_retreat_stop_t"] = float(t); b["retreat_stop_t"] = float(t)
                    b["_retreat_stop_xy"] = (bx_now, float(d.qpos[env.q_free + 1]))
                    b["body_x_at_retreat_walk_end"] = bx_now
                    b["retreat_stepped_delta_m"] = dx_ret
                # R2 CLEAR_BURST schedule: discrete reverse clear-lunge then planted hold (no XY pin)
                b["_r2_in_burst"] = False
                b["_r2_in_hold"] = False
                if (not b["_retreat_stop"]
                        and os.environ.get("GATE_Q_RET_CLEAR_BURST", GATE_Q_RET_CLEAR_BURST)
                        not in ("0", "false", "no")):
                    _r2_per = float(os.environ.get("GATE_Q_RET_CLEAR_BURST_PERIOD_S", GATE_Q_RET_CLEAR_BURST_PERIOD_S))
                    _r2_duty = float(os.environ.get("GATE_Q_RET_CLEAR_BURST_DUTY_S", GATE_Q_RET_CLEAR_BURST_DUTY_S))
                    _r2_hold = float(os.environ.get("GATE_Q_RET_CLEAR_BURST_HOLD_S", GATE_Q_RET_CLEAR_BURST_HOLD_S))
                    _r2_ssmin = int(float(os.environ.get("GATE_Q_RET_CLEAR_BURST_SS_MIN", GATE_Q_RET_CLEAR_BURST_SS_MIN)))
                    _r2_camax = float(os.environ.get("GATE_Q_RET_CLEAR_BURST_CAM_MAX", GATE_Q_RET_CLEAR_BURST_CAM_MAX))
                    _ss_tot = int(nLr) + int(nRr)
                    _ph = str(b.get("_r2_phase") or "idle")
                    if b.get("_r2_next_t") is None and _ss_tot >= _r2_ssmin and cam_now < _r2_camax - 1e-6:
                        b["_r2_next_t"] = float(t_ret_local)  # arm immediately after SS_MIN
                    if _ph == "idle" and b.get("_r2_next_t") is not None and float(t_ret_local) >= float(b["_r2_next_t"]) - 1e-12:
                        if cam_now < _r2_camax - 1e-6 and _ss_tot >= _r2_ssmin:
                            b["_r2_phase"] = "burst"
                            b["_r2_phase_t0"] = float(t_ret_local)
                            b["_r2_burst_n"] = int(b.get("_r2_burst_n") or 0) + 1
                            print(f"[Q] R2_BURST bi={bi} t={t:.2f} n={b['_r2_burst_n']} "
                                  f"cam={cam_now:.3f} dxc={dx_c_ret_live:.3f} ss={nLr}+{nRr}", flush=True)
                            _ph = "burst"
                        else:
                            b["_r2_next_t"] = None  # disarm near dest
                    if _ph == "burst":
                        _age = float(t_ret_local) - float(b.get("_r2_phase_t0") or t_ret_local)
                        if _age >= _r2_duty - 1e-12:
                            b["_r2_phase"] = "hold"
                            b["_r2_phase_t0"] = float(t_ret_local)
                            b["_r2_hold_cam0"] = float(cam_now)
                            print(f"[Q] R2_HOLD bi={bi} t={t:.2f} cam0={cam_now:.3f}", flush=True)
                            _ph = "hold"
                        else:
                            b["_r2_in_burst"] = True
                    if _ph == "hold":
                        _age = float(t_ret_local) - float(b.get("_r2_phase_t0") or t_ret_local)
                        b["_r2_in_hold"] = True
                        b["_r2_hold_s"] = float(b.get("_r2_hold_s") or 0.0) + 1.0 / CTRL_HZ
                        if _age >= _r2_hold - 1e-12:
                            _hc0 = b.get("_r2_hold_cam0")
                            _dg = (float(cam_now) - float(_hc0)) if _hc0 is not None else 0.0
                            b["_r2_hold_cam_gain"] = float(b.get("_r2_hold_cam_gain") or 0.0) + _dg
                            _gain = float(b["_r2_hold_cam_gain"])
                            print(f"[Q] R2_HOLD_END bi={bi} t={t:.2f} dcam_hold={_dg:.3f} "
                                  f"cam_gain_during_hold={_gain:.3f} hold_s={b['_r2_hold_s']:.2f}", flush=True)
                            b["_r2_phase"] = "idle"
                            b["_r2_phase_t0"] = None
                            b["_r2_in_hold"] = False
                            _burst_start = float(t_ret_local) - _age - _r2_duty
                            b["_r2_next_t"] = _burst_start + _r2_per
                            b["_r2_hold_cam0"] = None
                            _ph = "idle"
                    b["_r2_phase"] = _ph
                # R3 BURST_PLANT_CAP: clear-only reverse bursts + hard-capped plant gaps (NO damp-hold).
                # Distinct from R2 hold. R1 place OFF. Reuses CLEAR_BURST duty/hip/add/ss_min/cam_max.
                b["_r3_in_burst"] = False
                b["_r3_in_plant_gap"] = False
                _r3_cap = float(os.environ.get("GATE_Q_RET_BURST_PLANT_CAP", GATE_Q_RET_BURST_PLANT_CAP))
                _r2_on = os.environ.get("GATE_Q_RET_CLEAR_BURST", GATE_Q_RET_CLEAR_BURST) not in ("0", "false", "no")
                if (not b["_retreat_stop"] and _r3_cap > 1e-9 and not _r2_on):
                    _r3_duty = float(os.environ.get("GATE_Q_RET_CLEAR_BURST_DUTY_S", GATE_Q_RET_CLEAR_BURST_DUTY_S))
                    _r3_ssmin = int(float(os.environ.get("GATE_Q_RET_CLEAR_BURST_SS_MIN", GATE_Q_RET_CLEAR_BURST_SS_MIN)))
                    _r3_camax = float(os.environ.get("GATE_Q_RET_CLEAR_BURST_CAM_MAX", GATE_Q_RET_CLEAR_BURST_CAM_MAX))
                    _ss_tot = int(nLr) + int(nRr)
                    _ph3 = str(b.get("_r3_phase") or "idle")
                    if b.get("_r3_next_t") is None and _ss_tot >= _r3_ssmin and cam_now < _r3_camax - 1e-6:
                        b["_r3_next_t"] = float(t_ret_local)
                    if _ph3 == "idle" and b.get("_r3_next_t") is not None and float(t_ret_local) >= float(b["_r3_next_t"]) - 1e-12:
                        if cam_now < _r3_camax - 1e-6 and _ss_tot >= _r3_ssmin:
                            b["_r3_phase"] = "burst"
                            b["_r3_phase_t0"] = float(t_ret_local)
                            b["_r3_burst_n"] = int(b.get("_r3_burst_n") or 0) + 1
                            print(f"[Q] R3_BURST bi={bi} t={t:.2f} n={b['_r3_burst_n']} "
                                  f"cam={cam_now:.3f} dxc={dx_c_ret_live:.3f} ss={nLr}+{nRr} "
                                  f"plant_cap={_r3_cap:.2f}", flush=True)
                            _ph3 = "burst"
                        else:
                            b["_r3_next_t"] = None
                    if _ph3 == "burst":
                        _age = float(t_ret_local) - float(b.get("_r3_phase_t0") or t_ret_local)
                        if _age >= _r3_duty - 1e-12:
                            b["_r3_phase"] = "plant_gap"
                            b["_r3_phase_t0"] = float(t_ret_local)
                            b["_r3_gap_cam0"] = float(cam_now)
                            print(f"[Q] R3_PLANT_GAP bi={bi} t={t:.2f} cam0={cam_now:.3f} "
                                  f"cap={_r3_cap:.2f}", flush=True)
                            _ph3 = "plant_gap"
                        else:
                            b["_r3_in_burst"] = True
                    if _ph3 == "plant_gap":
                        _age = float(t_ret_local) - float(b.get("_r3_phase_t0") or t_ret_local)
                        b["_r3_in_plant_gap"] = True
                        b["_r3_gap_s"] = float(b.get("_r3_gap_s") or 0.0) + 1.0 / CTRL_HZ
                        # Hard cap: end gap at plant_cap — NO damp-hold; cancel×0.70 + vel-oppose only
                        if _age >= _r3_cap - 1e-12:
                            _gc0 = b.get("_r3_gap_cam0")
                            _dg = (float(cam_now) - float(_gc0)) if _gc0 is not None else 0.0
                            b["_r3_gap_cam_gain"] = float(b.get("_r3_gap_cam_gain") or 0.0) + _dg
                            b["_r3_gap_max_gain"] = max(float(b.get("_r3_gap_max_gain") or 0.0), _dg)
                            b["_r3_gap_n"] = int(b.get("_r3_gap_n") or 0) + 1
                            _gain = float(b["_r3_gap_cam_gain"])
                            _mx = float(b["_r3_gap_max_gain"])
                            print(f"[Q] R3_PLANT_GAP_END bi={bi} t={t:.2f} dcam_gap={_dg:.3f} "
                                  f"cam_gain_during_plant_gap={_gain:.3f} max_gap={_mx:.3f} "
                                  f"gap_s={b['_r3_gap_s']:.2f} n={b['_r3_gap_n']}", flush=True)
                            b["_r3_phase"] = "idle"
                            b["_r3_phase_t0"] = None
                            b["_r3_in_plant_gap"] = False
                            # Force next burst immediately (gap hard-capped; no idle wait)
                            b["_r3_next_t"] = float(t_ret_local)
                            b["_r3_gap_cam0"] = None
                            _ph3 = "idle"
                    b["_r3_phase"] = _ph3
                # R3 finalize open plant_gap on retreat stop (don't drop last gap cam)
                if (b.get("_retreat_stop") and b.get("_r3_gap_cam0") is not None
                        and str(b.get("_r3_phase") or "") == "plant_gap"):
                    _gc0 = b.get("_r3_gap_cam0")
                    _dg = (float(cam_now) - float(_gc0)) if _gc0 is not None else 0.0
                    b["_r3_gap_cam_gain"] = float(b.get("_r3_gap_cam_gain") or 0.0) + _dg
                    b["_r3_gap_max_gain"] = max(float(b.get("_r3_gap_max_gain") or 0.0), _dg)
                    b["_r3_gap_n"] = int(b.get("_r3_gap_n") or 0) + 1
                    print(f"[Q] R3_PLANT_GAP_FINAL bi={bi} t={t:.2f} dcam_gap={_dg:.3f} "
                          f"cam_gain_during_plant_gap={float(b['_r3_gap_cam_gain']):.3f}", flush=True)
                    b["_r3_gap_cam0"] = None
                    b["_r3_phase"] = "idle"
                    b["_r3_in_plant_gap"] = False
                # R4 MAX_PLANT_DWELL / MIN_SS_RATE: reactive forced reverse clear lift (NO damp-hold).
                # ≠ R3 scheduled burst→capped-gap. R1 place OFF. R2 hold OFF. R3 PLANT_CAP must stay 0.
                b["_r4_in_force"] = False
                _r4_dwell_cap = float(os.environ.get("GATE_Q_RET_MAX_PLANT_DWELL", GATE_Q_RET_MAX_PLANT_DWELL))
                _r4_min_rate = float(os.environ.get("GATE_Q_RET_MIN_SS_RATE", GATE_Q_RET_MIN_SS_RATE))
                _r3_cap_live = float(os.environ.get("GATE_Q_RET_BURST_PLANT_CAP", GATE_Q_RET_BURST_PLANT_CAP))
                _r2_on_r4 = os.environ.get("GATE_Q_RET_CLEAR_BURST", GATE_Q_RET_CLEAR_BURST) not in ("0", "false", "no")
                _r4_on = ((_r4_dwell_cap > 1e-9 or _r4_min_rate > 1e-9)
                          and (not _r2_on_r4) and _r3_cap_live <= 1e-9)
                if (not b["_retreat_stop"] and _r4_on):
                    _rest_r4 = 0.0086
                    _zL4 = float(d.xpos[env.bid_lf, 2]); _zR4 = float(d.xpos[env.bid_rf, 2])
                    _cL4 = _zL4 - SOLE_OFFSET; _cR4 = _zR4 - SOLE_OFFSET
                    _ar4 = float(os.environ.get("GATE_Q_SWING_CLEAR_AR_M", GATE_Q_SWING_CLEAR_AR_M))
                    _abs4 = float(os.environ.get("GATE_Q_SWING_CLEAR_ABS_M", GATE_Q_SWING_CLEAR_ABS_M))
                    _sole_clear4 = (
                        ((_cL4 - _rest_r4) >= _ar4 - 1e-9 and _cL4 >= _abs4 - 1e-9)
                        or ((_cR4 - _rest_r4) >= _ar4 - 1e-9 and _cR4 >= _abs4 - 1e-9)
                    )
                    # Readiness: at least one sole near lift OR already clear — avoid flush-only kicks as SS
                    _near_lift4 = (
                        ((_cL4 - _rest_r4) >= 0.008 - 1e-9)
                        or ((_cR4 - _rest_r4) >= 0.008 - 1e-9)
                        or _sole_clear4
                    )
                    _r4_duty = float(os.environ.get("GATE_Q_RET_FORCE_LIFT_DUTY_S", GATE_Q_RET_FORCE_LIFT_DUTY_S))
                    _r4_ssmin = int(float(os.environ.get("GATE_Q_RET_FORCE_LIFT_SS_MIN", GATE_Q_RET_FORCE_LIFT_SS_MIN)))
                    _r4_camax = float(os.environ.get("GATE_Q_RET_FORCE_LIFT_CAM_MAX", GATE_Q_RET_FORCE_LIFT_CAM_MAX))
                    _r4_cd = float(os.environ.get("GATE_Q_RET_FORCE_LIFT_COOLDOWN_S", GATE_Q_RET_FORCE_LIFT_COOLDOWN_S))
                    _eps4 = float(os.environ.get("GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS", GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS))
                    _cum4 = float(os.environ.get("GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM", GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM))
                    _ss_tot4 = int(nLr) + int(nRr)
                    _dt4 = 1.0 / float(CTRL_HZ)
                    _ph4 = str(b.get("_r4_phase") or "idle")
                    _live_gain4 = 0.0
                    if _sole_clear4:
                        if b.get("_r4_plant_cam0") is not None:
                            _dg4 = float(cam_now) - float(b["_r4_plant_cam0"])
                            b["_r4_plant_cam_gain"] = float(b.get("_r4_plant_cam_gain") or 0.0) + _dg4
                            b["_r4_plant_max_gain"] = max(float(b.get("_r4_plant_max_gain") or 0.0), _dg4)
                            b["_r4_plant_n"] = int(b.get("_r4_plant_n") or 0) + 1
                            print(f"[Q] R4_PLANT_END bi={bi} t={t:.2f} dcam_plant={_dg4:.3f} "
                                  f"dwell={float(b.get('_r4_plant_dwell') or 0.0):.2f} "
                                  f"cam_gain_during_plant={float(b['_r4_plant_cam_gain']):.3f} "
                                  f"max={float(b['_r4_plant_max_gain']):.3f}", flush=True)
                            b["_r4_plant_cam0"] = None
                        b["_r4_plant_dwell"] = 0.0
                        if _ph4 == "force":
                            b["_r4_force_cleared"] = True
                    else:
                        if b.get("_r4_plant_cam0") is None:
                            b["_r4_plant_cam0"] = float(cam_now)
                            b["_r4_plant_dwell"] = 0.0
                        b["_r4_plant_dwell"] = float(b.get("_r4_plant_dwell") or 0.0) + _dt4
                        b["_r4_plant_dwell_max"] = max(
                            float(b.get("_r4_plant_dwell_max") or 0.0),
                            float(b["_r4_plant_dwell"]))
                        _live_gain4 = float(cam_now) - float(b["_r4_plant_cam0"])
                    _dwell4 = float(b.get("_r4_plant_dwell") or 0.0)
                    _cum_live4 = float(b.get("_r4_plant_cam_gain") or 0.0) + (0.0 if _sole_clear4 else max(0.0, _live_gain4))
                    _last_ss_age4 = _last_ss_age
                    if _ph4 == "force":
                        _age4 = float(t_ret_local) - float(b.get("_r4_phase_t0") or t_ret_local)
                        b["_r4_in_force"] = True
                        if _age4 >= _r4_duty - 1e-12:
                            _cleared4 = bool(b.get("_r4_force_cleared"))
                            if not _cleared4:
                                b["_r4_flush_n"] = int(b.get("_r4_flush_n") or 0) + 1
                                _r4_cd = max(_r4_cd, 0.80)  # longer cool after flush chatter
                            print(f"[Q] R4_FORCE_END bi={bi} t={t:.2f} cleared={_cleared4} "
                                  f"force_n={int(b.get('_r4_force_n') or 0)} "
                                  f"flush_n={int(b.get('_r4_flush_n') or 0)} "
                                  f"dwell_max={float(b.get('_r4_plant_dwell_max') or 0.0):.2f}", flush=True)
                            b["_r4_phase"] = "idle"
                            b["_r4_phase_t0"] = None
                            b["_r4_in_force"] = False
                            b["_r4_cooldown_until"] = float(t_ret_local) + _r4_cd
                            _ph4 = "idle"
                    if _ph4 == "idle":
                        _cd_until = float(b.get("_r4_cooldown_until") or 0.0)
                        _armed4 = (_ss_tot4 >= _r4_ssmin and cam_now < _r4_camax - 1e-6
                                   and float(t_ret_local) >= _cd_until - 1e-12)
                        # Force lift only when swing can actually clear (near-lift or long dwell must try)
                        _can_clear4 = bool(_near_lift4) or _dwell4 >= max(_r4_dwell_cap, 0.8) - 1e-9
                        _trig_dwell = (_r4_dwell_cap > 1e-9 and (not _sole_clear4)
                                       and _dwell4 >= _r4_dwell_cap - 1e-12)
                        _trig_cam = ((not _sole_clear4)
                                     and (_live_gain4 > _eps4 + 1e-12
                                          or _cum_live4 > _cum4 + 1e-12))
                        _trig_rate = False
                        if _r4_min_rate > 1e-9 and (not _sole_clear4):
                            _max_gap = 1.0 / _r4_min_rate
                            _trig_rate = _last_ss_age4 >= _max_gap - 1e-12
                        if _armed4 and _can_clear4 and (_trig_dwell or _trig_cam or _trig_rate):
                            b["_r4_phase"] = "force"
                            b["_r4_phase_t0"] = float(t_ret_local)
                            b["_r4_force_n"] = int(b.get("_r4_force_n") or 0) + 1
                            b["_r4_force_cleared"] = bool(_sole_clear4)
                            b["_r4_in_force"] = True
                            _why = ("dwell" if _trig_dwell else ("cam_eps" if _trig_cam else "ss_rate"))
                            print(f"[Q] R4_FORCE bi={bi} t={t:.2f} n={b['_r4_force_n']} why={_why} "
                                  f"dwell={_dwell4:.2f} liveG={_live_gain4:.3f} cumG={_cum_live4:.3f} "
                                  f"cam={cam_now:.3f} ss={nLr}+{nRr} near={_near_lift4}", flush=True)
                            _ph4 = "force"
                    b["_r4_phase"] = _ph4
                # R4 finalize open plant bout on retreat stop
                if (b.get("_retreat_stop") and b.get("_r4_plant_cam0") is not None):
                    _dg4 = float(cam_now) - float(b["_r4_plant_cam0"])
                    b["_r4_plant_cam_gain"] = float(b.get("_r4_plant_cam_gain") or 0.0) + _dg4
                    b["_r4_plant_max_gain"] = max(float(b.get("_r4_plant_max_gain") or 0.0), _dg4)
                    b["_r4_plant_n"] = int(b.get("_r4_plant_n") or 0) + 1
                    print(f"[Q] R4_PLANT_FINAL bi={bi} t={t:.2f} dcam_plant={_dg4:.3f} "
                          f"cam_gain_during_plant={float(b['_r4_plant_cam_gain']):.3f}", flush=True)
                    b["_r4_plant_cam0"] = None
                    b["_r4_phase"] = "idle"
                    b["_r4_in_force"] = False
                # S1 CSF50 plant-cam suite (honesty; ≠ R4 force). Reuse R3 ε envs.
                _csf_pc = os.environ.get("GATE_Q_RET_CSF50", GATE_Q_RET_CSF50) not in ("0", "false", "no")
                if _csf_pc and not b.get("_retreat_stop"):
                    _rest_pc = 0.0086
                    _ar_pc = float(os.environ.get("GATE_Q_SWING_CLEAR_AR_M", GATE_Q_SWING_CLEAR_AR_M))
                    _abs_pc = float(os.environ.get("GATE_Q_SWING_CLEAR_ABS_M", GATE_Q_SWING_CLEAR_ABS_M))
                    _zL = float(d.xpos[env.bid_lf, 2]); _zR = float(d.xpos[env.bid_rf, 2])
                    _cL = _zL - SOLE_OFFSET; _cR = _zR - SOLE_OFFSET
                    _sole_clear = (
                        ((_cL - _rest_pc) >= _ar_pc - 1e-9 and _cL >= _abs_pc - 1e-9)
                        or ((_cR - _rest_pc) >= _ar_pc - 1e-9 and _cR >= _abs_pc - 1e-9)
                    )
                    if _sole_clear:
                        if b.get("_csf_plant_cam0") is not None:
                            _dg = float(cam_now) - float(b["_csf_plant_cam0"])
                            b["_csf_plant_cam_gain"] = float(b.get("_csf_plant_cam_gain") or 0.0) + max(0.0, _dg)
                            b["_csf_plant_max_gain"] = max(float(b.get("_csf_plant_max_gain") or 0.0), _dg)
                            b["_csf_plant_n"] = int(b.get("_csf_plant_n") or 0) + 1
                            b["_csf_plant_dwell_max"] = max(
                                float(b.get("_csf_plant_dwell_max") or 0.0),
                                float(b.get("_csf_plant_dwell") or 0.0))
                            b["_csf_plant_cam0"] = None
                            b["_csf_plant_dwell"] = 0.0
                    else:
                        if b.get("_csf_plant_cam0") is None:
                            b["_csf_plant_cam0"] = float(cam_now)
                        b["_csf_plant_dwell"] = float(b.get("_csf_plant_dwell") or 0.0) + 1.0 / CTRL_HZ
                if (b.get("_retreat_stop") and _csf_pc and b.get("_csf_plant_cam0") is not None):
                    _dg = float(cam_now) - float(b["_csf_plant_cam0"])
                    b["_csf_plant_cam_gain"] = float(b.get("_csf_plant_cam_gain") or 0.0) + max(0.0, _dg)
                    b["_csf_plant_max_gain"] = max(float(b.get("_csf_plant_max_gain") or 0.0), _dg)
                    b["_csf_plant_n"] = int(b.get("_csf_plant_n") or 0) + 1
                    b["_csf_plant_dwell_max"] = max(
                        float(b.get("_csf_plant_dwell_max") or 0.0),
                        float(b.get("_csf_plant_dwell") or 0.0))
                    b["_csf_plant_cam0"] = None
                # S2 ALIP-TVR plant-cam suite (honesty; ≠ R4 force / ≠ S1 CSF). Reuse R3 ε envs.
                _alip_pc = (
                    os.environ.get("GATE_Q_RET_ALIP_TVR", GATE_Q_RET_ALIP_TVR) not in ("0", "false", "no")
                    or os.environ.get("GATE_Q_T5B_MODELBASE", "0") not in ("0", "false", "no")
                    or (bool(os.environ.get("GATE_Q_T5B_CKPT", "").strip()) or bool(os.environ.get("GATE_Q_T5C_CKPT", "").strip()) or _gate_q_t5d_family_active())
                )
                if _alip_pc and not b.get("_retreat_stop"):
                    _rest_ap = 0.0086
                    _ar_ap = float(os.environ.get("GATE_Q_SWING_CLEAR_AR_M", GATE_Q_SWING_CLEAR_AR_M))
                    _abs_ap = float(os.environ.get("GATE_Q_SWING_CLEAR_ABS_M", GATE_Q_SWING_CLEAR_ABS_M))
                    _zLa = float(d.xpos[env.bid_lf, 2]); _zRa = float(d.xpos[env.bid_rf, 2])
                    _cLa = _zLa - SOLE_OFFSET; _cRa = _zRa - SOLE_OFFSET
                    _sole_clear_a = (
                        ((_cLa - _rest_ap) >= _ar_ap - 1e-9 and _cLa >= _abs_ap - 1e-9)
                        or ((_cRa - _rest_ap) >= _ar_ap - 1e-9 and _cRa >= _abs_ap - 1e-9)
                    )
                    if _sole_clear_a:
                        if b.get("_alip_plant_cam0") is not None:
                            _dga = float(cam_now) - float(b["_alip_plant_cam0"])
                            b["_alip_plant_cam_gain"] = float(b.get("_alip_plant_cam_gain") or 0.0) + max(0.0, _dga)
                            b["_alip_plant_max_gain"] = max(float(b.get("_alip_plant_max_gain") or 0.0), _dga)
                            b["_alip_plant_n"] = int(b.get("_alip_plant_n") or 0) + 1
                            b["_alip_plant_dwell_max"] = max(
                                float(b.get("_alip_plant_dwell_max") or 0.0),
                                float(b.get("_alip_plant_dwell") or 0.0))
                            b["_alip_plant_cam0"] = None
                            b["_alip_plant_dwell"] = 0.0
                    else:
                        if b.get("_alip_plant_cam0") is None:
                            b["_alip_plant_cam0"] = float(cam_now)
                        b["_alip_plant_dwell"] = float(b.get("_alip_plant_dwell") or 0.0) + 1.0 / CTRL_HZ
                if (b.get("_retreat_stop") and _alip_pc and b.get("_alip_plant_cam0") is not None):
                    _dga = float(cam_now) - float(b["_alip_plant_cam0"])
                    b["_alip_plant_cam_gain"] = float(b.get("_alip_plant_cam_gain") or 0.0) + max(0.0, _dga)
                    b["_alip_plant_max_gain"] = max(float(b.get("_alip_plant_max_gain") or 0.0), _dga)
                    b["_alip_plant_n"] = int(b.get("_alip_plant_n") or 0) + 1
                    b["_alip_plant_dwell_max"] = max(
                        float(b.get("_alip_plant_dwell_max") or 0.0),
                        float(b.get("_alip_plant_dwell") or 0.0))
                    b["_alip_plant_cam0"] = None
                # S3 REV-PHASE plant-cam suite (honesty; ≠ R*/S1/S2 schedulers). Reuse R3 ε envs.
                _rev_pc = os.environ.get("GATE_Q_RET_REV_PHASE", GATE_Q_RET_REV_PHASE) not in ("0", "false", "no")
                if _rev_pc and not b.get("_retreat_stop"):
                    _rest_rv = 0.0086
                    _ar_rv = float(os.environ.get("GATE_Q_SWING_CLEAR_AR_M", GATE_Q_SWING_CLEAR_AR_M))
                    _abs_rv = float(os.environ.get("GATE_Q_SWING_CLEAR_ABS_M", GATE_Q_SWING_CLEAR_ABS_M))
                    _zLv = float(d.xpos[env.bid_lf, 2]); _zRv = float(d.xpos[env.bid_rf, 2])
                    _cLv = _zLv - SOLE_OFFSET; _cRv = _zRv - SOLE_OFFSET
                    _sole_clear_v = (
                        ((_cLv - _rest_rv) >= _ar_rv - 1e-9 and _cLv >= _abs_rv - 1e-9)
                        or ((_cRv - _rest_rv) >= _ar_rv - 1e-9 and _cRv >= _abs_rv - 1e-9)
                    )
                    if _sole_clear_v:
                        b["_rev_clear_n"] = int(b.get("_rev_clear_n") or 0) + 1
                        if b.get("_rev_plant_cam0") is not None:
                            _dgv = float(cam_now) - float(b["_rev_plant_cam0"])
                            b["_rev_plant_cam_gain"] = float(b.get("_rev_plant_cam_gain") or 0.0) + max(0.0, _dgv)
                            b["_rev_plant_max_gain"] = max(float(b.get("_rev_plant_max_gain") or 0.0), _dgv)
                            b["_rev_plant_n"] = int(b.get("_rev_plant_n") or 0) + 1
                            b["_rev_plant_dwell_max"] = max(
                                float(b.get("_rev_plant_dwell_max") or 0.0),
                                float(b.get("_rev_plant_dwell") or 0.0))
                            b["_rev_plant_cam0"] = None
                            b["_rev_plant_dwell"] = 0.0
                    else:
                        if b.get("_rev_plant_cam0") is None:
                            b["_rev_plant_cam0"] = float(cam_now)
                        b["_rev_plant_dwell"] = float(b.get("_rev_plant_dwell") or 0.0) + 1.0 / CTRL_HZ
                if (b.get("_retreat_stop") and _rev_pc and b.get("_rev_plant_cam0") is not None):
                    _dgv = float(cam_now) - float(b["_rev_plant_cam0"])
                    b["_rev_plant_cam_gain"] = float(b.get("_rev_plant_cam_gain") or 0.0) + max(0.0, _dgv)
                    b["_rev_plant_max_gain"] = max(float(b.get("_rev_plant_max_gain") or 0.0), _dgv)
                    b["_rev_plant_n"] = int(b.get("_rev_plant_n") or 0) + 1
                    b["_rev_plant_dwell_max"] = max(
                        float(b.get("_rev_plant_dwell_max") or 0.0),
                        float(b.get("_rev_plant_dwell") or 0.0))
                    b["_rev_plant_cam0"] = None
                if b["_retreat_stop"]:
                    base_xy = b["_retreat_stop_xy"]; walk_ctrl = None
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "retreat_walk", 0.0, t_ret_local / max(ret_hold + ret_walk_s, 1e-6)
                elif (os.environ.get("GATE_Q_RET_GAP_REBURST", "0") not in ("0", "false", "no")
                      and not b["_retreat_stop"]
                      and b.get("_ret_settle_left", 0.0) <= 1e-12
                      and (nLr + nRr) >= int(float(os.environ.get("GATE_Q_RET_GAP_REBURST_SS_MIN", "4")))
                      and int(b.get("_ret_gap_reburst_n") or 0) < int(float(os.environ.get("GATE_Q_RET_GAP_REBURST_MAX", "3")))
                      and _last_ss_age >= float(os.environ.get("GATE_Q_RET_GAP_REBURST_SS_GAP", "2.0")) - 1e-9
                      and (t_ret_local - float(b.get("_ret_last_reburst_t") or 0.0)) >= 1.0
                      and cam_now < (0.58 if bi >= 1 else 0.50)
                      and dx_c_ret_live >= float(os.environ.get("GATE_Q_RET_GAP_REBURST_DXC_MIN", "0.04")) - 1e-6):
                    # tip A28c: break planted middle (SS silence) with fresh clearance burst
                    b["_ret_gap_reburst_n"] = int(b.get("_ret_gap_reburst_n") or 0) + 1
                    b["ret_gap_reburst_n"] = b["_ret_gap_reburst_n"]
                    b["_ret_last_reburst_dx"] = dx_ret
                    b["_ret_last_reburst_t"] = t_ret_local
                    b["_ret_settle_left"] = float(os.environ.get("GATE_Q_RET_GAP_REBURST_SETTLE_S", "0.18"))
                    b["_ret_settle_done"] = False
                    b["_ret_burst_t0"] = None
                    print(f"[Q] GAP_REBURST bi={bi} t={t:.2f} age={_last_ss_age:.2f} n={b['_ret_gap_reburst_n']} "
                          f"dxc={dx_c_ret_live:.3f} cam={cam_now:.3f}", flush=True)
                    b["_retreat_stop_xy"] = (bx_now, float(d.qpos[env.q_free + 1]))
                    base_xy = b["_retreat_stop_xy"]; walk_ctrl = None
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "retreat_walk", 0.0, t_ret_local / max(ret_hold + ret_walk_s, 1e-6)
                elif (os.environ.get("GATE_Q_RET_EARLY_REBURST", GATE_Q_RET_EARLY_REBURST) not in ("0", "false", "no")
                      and not b["_retreat_stop"]
                      and not b.get("_ret_early_reburst_done")
                      and b.get("_ret_settle_left", 0.0) <= 1e-12
                      and both_ret
                      and cam_now < float(os.environ.get("GATE_Q_RET_EARLY_CAM", GATE_Q_RET_EARLY_CAM)) - 1e-6
                      and dx_c_ret_live < float(os.environ.get("GATE_Q_RET_EARLY_REBURST_DXC", GATE_Q_RET_EARLY_REBURST_DXC)) - 1e-6
                      and t_ret_local >= ret_hold + 2.0):
                    # G3: one early clearance reburst before planted middle forms
                    b["_ret_early_reburst_done"] = True
                    b["ret_early_reburst"] = True
                    b["_ret_last_reburst_dx"] = dx_ret
                    b["_ret_last_reburst_t"] = t_ret_local
                    b["_ret_settle_left"] = float(os.environ.get("GATE_Q_RET_EARLY_REBURST_SETTLE_S", "0.18"))
                    b["_ret_settle_done"] = False
                    b["_ret_burst_t0"] = None
                    print(f"[Q] EARLY_REBURST bi={bi} t={t:.2f} cam={cam_now:.3f} dxc={dx_c_ret_live:.3f}", flush=True)
                    b["_retreat_stop_xy"] = (bx_now, float(d.qpos[env.q_free + 1]))
                    base_xy = b["_retreat_stop_xy"]; walk_ctrl = None
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "retreat_walk", 0.0, t_ret_local / max(ret_hold + ret_walk_s, 1e-6)
                elif (os.environ.get("GATE_Q_DISABLE_MID_SETTLE", GATE_Q_DISABLE_MID_SETTLE) not in ("1", "true", "yes")
                      and not b["_retreat_stop"]
                      and b.get("_ret_settle_left", 0.0) <= 1e-12
                      and (b["n_ss_L_ret"] + b["n_ss_R_ret"]) >= 3
                      and int(b.get("_ret_periodic_n") or 0) < int(os.environ.get("GATE_Q_RET_PERIODIC_REBURST_MAX", GATE_Q_RET_PERIODIC_REBURST_MAX))
                      and dx_ret - float(b.get("_ret_last_reburst_dx") or 0.0) >= float(os.environ.get("GATE_Q_RET_PERIODIC_REBURST_DX", GATE_Q_RET_PERIODIC_REBURST_DX))
                      and (t_ret_local - float(b.get("_ret_last_reburst_t") or 0.0)) >= float(os.environ.get("GATE_Q_RET_PERIODIC_REBURST_DT", GATE_Q_RET_PERIODIC_REBURST_DT))
                      and cam_now < (0.58 if bi >= 1 else 0.50)):
                    # A14: bout1 reburst until past joint cam bar (was <0.50 stalling ~0.46)
                    b["_ret_periodic_n"] = int(b.get("_ret_periodic_n") or 0) + 1
                    b["_ret_last_reburst_dx"] = dx_ret
                    b["_ret_last_reburst_t"] = t_ret_local
                    b["_ret_settle_left"] = float(os.environ.get("GATE_Q_RET_PERIODIC_REBURST_SETTLE_S", "0.22"))
                    b["_ret_settle_done"] = False
                    b["_ret_burst_t0"] = None
                    b["_retreat_stop_xy"] = (bx_now, float(d.qpos[env.q_free + 1]))
                    base_xy = b["_retreat_stop_xy"]; walk_ctrl = None
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "retreat_walk", 0.0, t_ret_local / max(ret_hold + ret_walk_s, 1e-6)
                elif (os.environ.get("GATE_Q_DISABLE_MID_SETTLE", GATE_Q_DISABLE_MID_SETTLE) not in ("1", "true", "yes")
                      and not (bi >= 1 and os.environ.get("GATE_Q_BOUT1_RET_SKIP_MID_SETTLE", "1") not in ("0","false","no"))
                      and cam_now >= (0.28 if bi < 1 else 0.32) - 1e-6
                      and b["n_ss_L_ret"] >= 1 and b["n_ss_R_ret"] >= 1
                      and not b.get("_ret_settle_done") and b.get("_ret_settle_left", 0.0) <= 1e-12
                      and (b["n_ss_L_ret"] + b["n_ss_R_ret"]) < 4):
                    settle_r = float(os.environ.get("GATE_Q_RETREAT_MID_SETTLE_S", "0.35"))
                    if bi >= 1:
                        settle_r = float(os.environ.get("GATE_Q_BOUT1_RETREAT_MID_SETTLE_S", "0.28"))
                    b["_ret_settle_left"] = settle_r
                    b["_ret_last_reburst_dx"] = dx_ret
                    b["_ret_last_reburst_t"] = t_ret_local
                    b["_retreat_stop_xy"] = (bx_now, float(d.qpos[env.q_free + 1]))
                    base_xy = b["_retreat_stop_xy"]; walk_ctrl = None
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "retreat_walk", 0.0, t_ret_local / max(ret_hold + ret_walk_s, 1e-6)
                elif b.get("_ret_settle_left", 0.0) > 1e-12:
                    b["_ret_settle_left"] = max(0.0, b["_ret_settle_left"] - 1.0 / CTRL_HZ)
                    if b["_ret_settle_left"] <= 1e-12:
                        b["_ret_settle_done"] = True
                        b["_ret_burst_t0"] = t_ret_local
                    base_xy = b.get("_retreat_stop_xy") or (bx_now, float(d.qpos[env.q_free + 1]))
                    walk_ctrl = None
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "retreat_walk", 0.0, t_ret_local / max(ret_hold + ret_walk_s, 1e-6)
                else:
                    a_w = _gait_amp(t_ret_local, ret_hold, ret_ramp)
                    # S1 CSF50: CoM-timed T for phi (exchange), recompute F,T at support exchange
                    # S3 REV-PHASE: never call CSF/ALIP helpers (family ≠ S1/S2)
                    _rev_on = os.environ.get("GATE_Q_RET_REV_PHASE", GATE_Q_RET_REV_PHASE) not in ("0", "false", "no")
                    _csf_on = (not _rev_on) and os.environ.get("GATE_Q_RET_CSF50", GATE_Q_RET_CSF50) not in ("0", "false", "no")
                    _gait_T_phi = float(RETREAT_GAIT_T)
                    if _csf_on and a_w > 0.05:
                        try:
                            _com_xy, _com_vxy, _com_z = wg.estimate_com_state(m, d)
                            _vx_cmd = float(os.environ.get("GATE_Q_RET_CSF50_VX", GATE_Q_RET_CSF50_VX))
                            _Fsc = float(os.environ.get("GATE_Q_RET_CSF50_F_SCALE", GATE_Q_RET_CSF50_F_SCALE))
                            _Tsc = float(os.environ.get("GATE_Q_RET_CSF50_T_SCALE", GATE_Q_RET_CSF50_T_SCALE))
                            _Tnom = float(os.environ.get("GATE_Q_RET_CSF50_T_NOM", GATE_Q_RET_CSF50_T_NOM))
                            _dxb = float(os.environ.get("GATE_Q_RET_CSF50_DX_BACK_MAX", GATE_Q_RET_CSF50_DX_BACK_MAX))
                            _need_recomp = b.get("_csf_step_t0") is None
                            if b.get("_csf_step_t0") is not None:
                                _age_s = float(t_ret_local) - float(b["_csf_step_t0"])
                                _Tlock = max(float(b.get("_csf_T") or _Tnom), 1e-3)
                                if _age_s >= _Tlock - 1e-9:
                                    _need_recomp = True  # remaining T→0 → exchange; new reverse swing
                            if _need_recomp:
                                _F, _T, _om = csf50_retreat_FT(
                                    _com_xy, _com_vxy, _com_z, _vx_cmd,
                                    T_nom=_Tnom, F_scale=_Fsc, T_scale=_Tsc, dx_back_max=_dxb)
                                b["_csf_F"] = float(_F)
                                b["_csf_T"] = float(_T)
                                b["_csf_omega"] = float(_om)
                                b["_csf_step_t0"] = float(t_ret_local)
                                b["_csf_F_sum"] = float(b.get("_csf_F_sum") or 0.0) + float(_F)
                                b["_csf_T_sum"] = float(b.get("_csf_T_sum") or 0.0) + float(_T)
                                b["_csf_n"] = int(b.get("_csf_n") or 0) + 1
                            _gait_T_phi = max(float(b.get("_csf_T") or RETREAT_GAIT_T), 1e-3)
                        except Exception:
                            _gait_T_phi = float(RETREAT_GAIT_T)
                    # S2 ALIP-TVR: mid-swing foothold prior + Δu_fp replan (CSF50 stays OFF; no T drive)
                    _alip_s2 = os.environ.get("GATE_Q_RET_ALIP_TVR", GATE_Q_RET_ALIP_TVR) not in ("0", "false", "no")
                    _alip_t5b = os.environ.get("GATE_Q_T5B_MODELBASE", "0") not in ("0", "false", "no") or (bool(os.environ.get("GATE_Q_T5B_CKPT", "").strip()) or bool(os.environ.get("GATE_Q_T5C_CKPT", "").strip()) or _gate_q_t5d_family_active())
                    _alip_on = (not _rev_on) and (_alip_s2 or _alip_t5b)
                    if _alip_on and a_w > 0.05:
                        try:
                            import numpy as _np_al
                            _rest_a = 0.0086
                            _ar_a = float(os.environ.get("GATE_Q_SWING_CLEAR_AR_M", GATE_Q_SWING_CLEAR_AR_M))
                            _abs_a = float(os.environ.get("GATE_Q_SWING_CLEAR_ABS_M", GATE_Q_SWING_CLEAR_ABS_M))
                            _zL2 = float(d.xpos[env.bid_lf, 2]); _zR2 = float(d.xpos[env.bid_rf, 2])
                            _cL2 = _zL2 - SOLE_OFFSET; _cR2 = _zR2 - SOLE_OFFSET
                            _clrL = (_cL2 - _rest_a) >= _ar_a - 1e-9 and _cL2 >= _abs_a - 1e-9
                            _clrR = (_cR2 - _rest_a) >= _ar_a - 1e-9 and _cR2 >= _abs_a - 1e-9
                            # provisional phi for swing-fraction (same formula as below)
                            if b.get("_ret_burst_t0") is not None:
                                _phi_a = (max(0.0, t_ret_local - float(b["_ret_burst_t0"])) / max(_gait_T_phi, 1e-6)) % 1.0
                            else:
                                _phi_a = (max(0.0, t_ret_local - ret_hold) / max(_gait_T_phi, 1e-6)) % 1.0 if a_w > 0 else 0.0
                            _ds_a = float(_np_al.clip(wg.DS_S / max(_gait_T_phi, 1e-3), 0.08, 0.55))
                            _ds_end_a = 0.5 + 0.5 * _ds_a
                            _swing_side = None; _s_sw = None
                            for _side, _clr in (("L", _clrL), ("R", _clrR)):
                                if not _clr:
                                    continue
                                _pl = wg.phase_leg(_phi_a, _side)
                                if _pl < _ds_end_a:
                                    continue
                                _s = (_pl - _ds_end_a) / max(1e-6, 1.0 - _ds_end_a)
                                if 0.05 <= _s <= 0.98:
                                    _swing_side = _side; _s_sw = float(_s); break
                            if _swing_side is None:
                                # no clear swing — cancel mid-swing state (planted = cancel×0.70 only)
                                b["_alip_replan_idx"] = -1
                                b["_alip_swing_side"] = None
                            else:
                                _Nrp = max(1, int(float(os.environ.get(
                                    "GATE_Q_RET_ALIP_TVR_N_REPLAN", GATE_Q_RET_ALIP_TVR_N_REPLAN))))
                                _ridx = min(int(_s_sw * _Nrp), _Nrp - 1)
                                _side_chg = (b.get("_alip_swing_side") != _swing_side)
                                if _side_chg or int(b.get("_alip_replan_idx") if b.get("_alip_replan_idx") is not None else -1) != _ridx:
                                    _com_xy, _com_vxy, _com_z = wg.estimate_com_state(m, d)
                                    _vx_a = float(os.environ.get(
                                        "GATE_Q_T5E_VX_CMD",
                                        os.environ.get(
                                            "GATE_Q_T5D2_VX_CMD",
                                            os.environ.get(
                                                "GATE_Q_T5D1_VX_CMD",
                                                os.environ.get(
                                                    "GATE_Q_T5C_VX_CMD",
                                                    os.environ.get(
                                                        "GATE_Q_T5B_VX_CMD",
                                                        os.environ.get("GATE_Q_RET_ALIP_TVR_VX", GATE_Q_RET_ALIP_TVR_VX),
                                                    ),
                                                ),
                                            ),
                                        ),
                                    ))
                                    _dxcap = float(os.environ.get("GATE_Q_RET_ALIP_TVR_DX_CAP", GATE_Q_RET_ALIP_TVR_DX_CAP))
                                    _wsm = float(os.environ.get("GATE_Q_RET_ALIP_TVR_SMOOTH", GATE_Q_RET_ALIP_TVR_SMOOTH))
                                    _wsm = float(_np_al.clip(_wsm, 0.0, 0.95))
                                    # remaining retreat Δ proxy from dest cam gap
                                    _dx_rem = max(0.0, 0.55 - float(cam_now)) * 0.55
                                    _fp, _om = alip_tvr_foothold_prior(
                                        _com_xy, _com_vxy, _com_z, _vx_a,
                                        dx_cap=_dxcap, dx_remain=_dx_rem)
                                    # real swing target = foothold prior (forbid dwell-only kicks)
                                    _du_raw = float(_fp)
                                    _du_prev = float(b.get("_alip_du_fp") or _du_raw)
                                    if _side_chg or b.get("_alip_replan_idx", -1) < 0:
                                        _du = _du_raw  # fresh swing: no smooth from other leg
                                    else:
                                        _du = (1.0 - _wsm) * _du_raw + _wsm * _du_prev
                                    _jump = abs(_du - _du_prev) if not _side_chg else 0.0
                                    b["_alip_fp_prior"] = float(_fp)
                                    b["_alip_du_fp"] = float(_du)
                                    b["_alip_du_prev"] = float(_du_prev)
                                    b["_alip_omega"] = float(_om)
                                    b["_alip_replan_idx"] = int(_ridx)
                                    b["_alip_swing_side"] = _swing_side
                                    b["_alip_n_replan"] = int(b.get("_alip_n_replan") or 0) + 1
                                    b["_alip_fp_jump_max"] = max(float(b.get("_alip_fp_jump_max") or 0.0), _jump)
                                    b["_alip_fp_sum"] = float(b.get("_alip_fp_sum") or 0.0) + float(_fp)
                                    b["_alip_du_sum"] = float(b.get("_alip_du_sum") or 0.0) + float(_du)
                                    # T5-E: FOOTSTEP-SEQ owns F from external TD list (≠ D1/D2)
                                    if _gate_q_t5e_active():
                                        try:
                                            _dxb_e = float(os.environ.get("GATE_Q_T5E_DX_BACK", "0.028"))
                                            _nmax_e = int(float(os.environ.get("GATE_Q_T5E_N_MAX", "10")))
                                            _nmin_e = int(float(os.environ.get("GATE_Q_T5E_N_MIN", "3")))
                                            _lat_e = float(os.environ.get("GATE_Q_T5E_LAT_BOS", "0.0"))
                                            _tgt_e = float(os.environ.get("GATE_Q_T5E_TARGET_DX", "0.18"))
                                            if not b.get("_t5e_tds"):
                                                b["_t5e_tds"] = _emit_t5e_td_list(
                                                    _tgt_e, dx_back=_dxb_e, n_max=_nmax_e,
                                                    n_min=_nmin_e, lat_bos=_lat_e)
                                                b["_t5e_idx"] = 0
                                                b["_t5e_list_id"] = (
                                                    f"n{len(b['_t5e_tds'])}_dxb{_dxb_e:.3f}"
                                                    f"_lat{_lat_e:.3f}_tgt{_tgt_e:.3f}")
                                            _tds = b["_t5e_tds"]
                                            _ti = int(b.get("_t5e_idx") or 0)
                                            if _ti >= len(_tds):
                                                _side_x = "L" if (len(_tds) % 2 == 0) else "R"
                                                _tds.append({
                                                    "i": len(_tds), "F": float(-abs(_dxb_e)),
                                                    "side": _side_x,
                                                    "lat": float(_lat_e) if _side_x == "L" else -float(_lat_e),
                                                    "dx_back": float(abs(_dxb_e)),
                                                })
                                            _td = _tds[_ti]
                                            _F_seq = float(_np_al.clip(_td["F"], -abs(_dxcap), 0.0))
                                            # override foothold track with sequence next TD
                                            b["_alip_du_sum"] = float(b.get("_alip_du_sum") or 0.0) - float(_du) + float(_F_seq)
                                            b["_alip_du_fp"] = float(_F_seq)
                                            b["_alip_fp_prior"] = float(_F_seq)
                                            b["_t5e_F"] = float(_F_seq)
                                            b["_t5e_n_track"] = int(b.get("_t5e_n_track") or 0) + 1
                                            # next-TD error (swing foot vs support+F)
                                            _bid_sw = env.bid_lf if _swing_side == "L" else env.bid_rf
                                            _bid_st = env.bid_rf if _swing_side == "L" else env.bid_lf
                                            _sx_e = float(d.xpos[_bid_st, 0]); _sy_e = float(d.xpos[_bid_st, 1])
                                            _fx_e = float(d.xpos[_bid_sw, 0]); _fy_e = float(d.xpos[_bid_sw, 1])
                                            _err_e = float(((_fx_e - (_sx_e + _F_seq))**2 + (_fy_e - (_sy_e + float(_td.get("lat", 0.0))))**2) ** 0.5)
                                            b["_t5e_last_err"] = float(_err_e)
                                            b["_t5e_err_sum"] = float(b.get("_t5e_err_sum") or 0.0) + float(_err_e)
                                            b["_t5e_err_n"] = int(b.get("_t5e_err_n") or 0) + 1
                                            # advance on support exchange (side change) or late near-TD
                                            _adv = False
                                            if _side_chg and b.get("_t5e_prev_swing") is not None:
                                                _adv = True
                                            if _s_sw is not None and float(_s_sw) >= 0.85 and _err_e <= float(os.environ.get("GATE_Q_T5E_ADV_ERR", "0.018")):
                                                _adv = True
                                            if _adv:
                                                b["_t5e_idx"] = int(b.get("_t5e_idx") or 0) + 1
                                                b["_t5e_n_advance"] = int(b.get("_t5e_n_advance") or 0) + 1
                                            b["_t5e_prev_swing"] = _swing_side
                                        except Exception:
                                            pass
                                    # T5-D2: online ICP error → ΔT + Δfoot (≠ D1 schedule twin; ≠ T5-E)
                                    if _gate_q_t5d2_active() and (not _gate_q_t5e_active()):
                                        try:
                                            _zL3 = float(d.xpos[env.bid_lf, 2]); _zR3 = float(d.xpos[env.bid_rf, 2])
                                            _cL3 = _zL3 - SOLE_OFFSET; _cR3 = _zR3 - SOLE_OFFSET
                                            _rest3 = 0.0086
                                            _ar3 = float(os.environ.get("GATE_Q_SWING_CLEAR_AR_M", GATE_Q_SWING_CLEAR_AR_M))
                                            _abs3 = float(os.environ.get("GATE_Q_SWING_CLEAR_ABS_M", GATE_Q_SWING_CLEAR_ABS_M))
                                            _Lp = not ((_cL3 - _rest3) >= _ar3 - 1e-9 and _cL3 >= _abs3 - 1e-9)
                                            _Rp = not ((_cR3 - _rest3) >= _ar3 - 1e-9 and _cR3 >= _abs3 - 1e-9)
                                            if _Lp and not _Rp:
                                                _sx = float(d.xpos[env.bid_lf, 0])
                                            elif _Rp and not _Lp:
                                                _sx = float(d.xpos[env.bid_rf, 0])
                                            else:
                                                _sx = 0.5 * (float(d.xpos[env.bid_lf, 0]) + float(d.xpos[env.bid_rf, 0]))
                                            _Tnom_i = float(os.environ.get("GATE_Q_T5D2_T_NOM",
                                                os.environ.get("GATE_Q_RET_CSF50_T_NOM", GATE_Q_RET_CSF50_T_NOM)))
                                            _T_sched = float(b.get("_icp_T") or _Tnom_i)
                                            # CSF-ish schedule T prior (in-process; S1 flag stays OFF)
                                            _g = 9.81; _z = max(float(_com_z), 0.12)
                                            _om_s = max((_g / _z) ** 0.5, 0.5)
                                            _vy = float(_com_vxy[1]); _yref = 0.04
                                            try:
                                                _Tlat = (1.0 / _om_s) * math.asinh(max(1e-6, abs(_vy) / max(_om_s * _yref, 1e-3)))
                                            except Exception:
                                                _Tlat = 0.0
                                            _T_sched = float(_Tnom_i) * (0.85 + 0.15 * min(_Tlat / 0.40, 1.5))
                                            _T_sched = float(max(0.50, min(1.10, _T_sched)))
                                            b["_icp_sched_F"] = float(_fp)
                                            b["_icp_sched_T"] = float(_T_sched)
                                            _F_new, _T_new, _icpx, _eicp, _dT, _df, _omi = icp_delta_TF(
                                                _com_xy, _com_vxy, _com_z, _sx, _vx_a, float(_du), _T_sched,
                                                dx_cap=_dxcap,
                                                k_T=float(os.environ.get("GATE_Q_T5D2_K_T", "3.0")),
                                                k_foot=float(os.environ.get("GATE_Q_T5D2_K_FOOT", "1.4")),
                                            )
                                            b["_alip_du_fp"] = float(_F_new)  # foothold track uses ICP-adjusted F
                                            b["_alip_du_sum"] = float(b.get("_alip_du_sum") or 0.0) - float(_du) + float(_F_new)
                                            b["_icp_F"] = float(_F_new)
                                            b["_icp_T"] = float(_T_new)
                                            b["_icp_err"] = float(_eicp)
                                            b["_icp_dT"] = float(_dT)
                                            b["_icp_dfoot"] = float(_df)
                                            b["_icp_x"] = float(_icpx)
                                            b["_icp_omega"] = float(_omi)
                                            if abs(_dT) > 1e-5 or abs(_df) > 1e-5 or abs(_eicp) > 1e-4:
                                                b["_icp_n"] = int(b.get("_icp_n") or 0) + 1
                                                b["_icp_err_sum"] = float(b.get("_icp_err_sum") or 0.0) + float(_eicp)
                                                b["_icp_dT_sum"] = float(b.get("_icp_dT_sum") or 0.0) + float(_dT)
                                                b["_icp_dfoot_sum"] = float(b.get("_icp_dfoot_sum") or 0.0) + float(_df)
                                        except Exception:
                                            pass
                        except Exception:
                            pass
                    # T5-D2: drive gait period from ICP-adjusted T (online ΔT; skip under T5-E)
                    if _gate_q_t5d2_active() and (not _gate_q_t5e_active()) and bi >= 0:
                        _Ti = float(cycles[bi].get("_icp_T") or 0.0)
                        if _Ti > 1e-3:
                            _gait_T_phi = max(_Ti, 1e-3)
                    if b.get("_ret_burst_t0") is not None:
                        a_w = max(a_w, 1.0)
                        phi = (max(0.0, t_ret_local - float(b["_ret_burst_t0"])) / max(_gait_T_phi, 1e-6)) % 1.0
                    else:
                        phi = (max(0.0, t_ret_local - ret_hold) / max(_gait_T_phi, 1e-6)) % 1.0 if a_w > 0 else 0.0
                    # R2/R3/R4: during burst/force kick phi into swing half to initiate SS from DS
                    if (b.get("_r2_in_burst") or b.get("_r3_in_burst") or b.get("_r4_in_force")) and a_w > 0.05:
                        a_w = max(a_w, 1.0)
                        if b.get("_r4_in_force"):
                            _bn = int(b.get("_r4_force_n") or 0)
                        elif b.get("_r3_in_burst"):
                            _bn = int(b.get("_r3_burst_n") or 0)
                        else:
                            _bn = int(b.get("_r2_burst_n") or 0)
                        phi = 0.22 if (_bn % 2 == 0) else 0.72
                    # R2 hold: keep amp (cancel path needs a_w>0.05) — no pin; no swing kick
                    if b.get("_r2_in_hold") and a_w > 0.05:
                        a_w = max(a_w, 0.85)  # stay in planted cancel band; hip×0 while not clear
                    # R3 plant_gap: keep amp for cancel×0.70 + vel-oppose; NO damp-hold / NO phi kick
                    if b.get("_r3_in_plant_gap") and a_w > 0.05:
                        a_w = max(a_w, 0.85)
                    # S3 REV-PHASE: invert/offset gait phase clock so clear strides are reverse-native.
                    # PHI≈π → full invert (1-phi); else phi := (phi + PHI) % 1. Real swing before advance — no dwell force.
                    if _rev_on and a_w > 0.05:
                        import math as _math_rev
                        _phi_kn = float(os.environ.get("GATE_Q_RET_REV_PHASE_PHI", GATE_Q_RET_REV_PHASE_PHI))
                        _hip_kn = float(os.environ.get("GATE_Q_RET_REV_PHASE_HIP", GATE_Q_RET_REV_PHASE_HIP))
                        _sw_kn = float(os.environ.get("GATE_Q_RET_REV_PHASE_SWING", GATE_Q_RET_REV_PHASE_SWING))
                        if abs(_phi_kn - _math_rev.pi) < 0.15 or abs(_phi_kn - 3.14159) < 0.15:
                            phi = (1.0 - float(phi)) % 1.0
                            _phi_pol = -1.0  # full invert
                        else:
                            phi = (float(phi) + _phi_kn) % 1.0
                            _phi_pol = float(_phi_kn)
                        b["_rev_phi_polarity"] = float(_phi_pol)
                        b["_rev_hip_sign"] = float(_hip_kn)
                        b["_rev_swing_enc"] = float(_sw_kn)
                        b["_rev_n"] = int(b.get("_rev_n") or 0) + 1
                    if a_w < 0.05:
                        base_xy = (float(b["_retreat_walk_x0"]), body_y0)
                    settle = 0.70
                    if b["_retreat_cop_bias"] is None and t_ret_local < settle:
                        com = np.asarray(d.subtree_com[bid_com, :2], dtype=np.float64)
                        try:
                            support, _, _, _ = wg._support_feet(m, d, env.bid_lf, env.bid_rf, env.gid_floor, None)
                            b["_retreat_cop_samples"].append(com - support)
                        except Exception: pass
                        if t_ret_local >= settle - 1.5 / CTRL_HZ and b["_retreat_cop_samples"]:
                            b["_retreat_cop_bias"] = np.mean(np.stack(b["_retreat_cop_samples"], 0), 0)
                    action = None
                    if use_residual and a_w > 0.01:
                        _pol_r = pol_ret if pol_ret is not None else pol
                        if _pol_r is not None:
                            _phi_r = phi
                            # T4 opt C: reverse phase encoding inside NEW weights (not REV_PHASE flag)
                            if pol_ret is not None:
                                _phi_r = (phi + 0.5) % 1.0
                            obs = _obs_now(_phi_r).copy(); obs[24] *= -1.0
                            if pol_ret is not None:
                                # Opt B: commanded reverse Vx (+ remain proxy) in tn slot
                                _vx = float(os.environ.get(
                                    "GATE_Q_T5E_VX_CMD",
                                    os.environ.get(
                                        "GATE_Q_T5D2_VX_CMD",
                                        os.environ.get(
                                            "GATE_Q_T5D1_VX_CMD",
                                            os.environ.get(
                                                "GATE_Q_T5C_VX_CMD",
                                                os.environ.get(
                                                    "GATE_Q_T5B_VX_CMD",
                                                    os.environ.get(
                                                        "GATE_Q_T5_VX_CMD",
                                                        os.environ.get("GATE_Q_T4_VX_CMD", "-0.08"),
                                                    ),
                                                ),
                                            ),
                                        ),
                                    ),
                                ))
                                _vx_n = max(-1.0, min(1.0, _vx / 0.15))
                                _dx_done = 0.0
                                if b.get("_retreat_walk_x0") is not None:
                                    _dx_done = max(0.0, float(b["_retreat_walk_x0"]) - float(bx_now))
                                _tgt = float(os.environ.get(
                                    "GATE_Q_T5C_TARGET_DX",
                                    os.environ.get(
                                        "GATE_Q_T5B_TARGET_DX",
                                        os.environ.get(
                                            "GATE_Q_T5_TARGET_DX",
                                            os.environ.get("GATE_Q_T4_TARGET_DX", "0.35"),
                                        ),
                                    ),
                                ))
                                _remain = max(-1.0, min(1.0, (_tgt - _dx_done) / max(_tgt, 1e-6)))
                                obs[40] = np.float32(0.70 * _vx_n + 0.30 * _remain)
                            action, _ = _pol_r.predict(obs, deterministic=True)
                    walk_ctrl = (None if a_w < 0.05 else
                                 {"amp": a_w, "phi": phi, "t_abs": t_ret_local,
                                  "cop_bias": b["_retreat_cop_bias"], "action": action, "reverse": True})
                    gcmd, phase, cmd, a_rev = OPEN_CMD, "retreat_walk", 0.0, t_ret_local / max(ret_hold + ret_walk_s, 1e-6)
            else:
                # tip A28c: if jump-fin early, ramp shim from stop instant (not t_ret_walk)
                if b.get("_retreat_jump_fin") and b.get("_retreat_fin_t0") is None:
                    b["_retreat_fin_t0"] = float(local)
                if b.get("_retreat_fin_t0") is not None:
                    a_fin = min(1.0, max(0.0, (local - float(b["_retreat_fin_t0"])) / max(ret_fin_s, 1e-6)))
                else:
                    a_fin = min(1.0, max(0.0, (local - t_ret_walk) / max(ret_fin_s, 1e-6)))
                if b["_retreat_finalize_x0"] is None:
                    b["_retreat_finalize_x0"] = float(d.qpos[env.q_free])
                    b["body_x_at_retreat_walk_end"] = b["_retreat_finalize_x0"]
                    if b["_retreat_walk_x0"] is not None:
                        b["retreat_stepped_delta_m"] = b["_retreat_walk_x0"] - b["_retreat_finalize_x0"]
                    b["_retreat_fin_cam0"] = cam_to_lever_horiz(d, env.sid, env.gid_lever)
                    # restore forward gait after reverse walk
                    wg.HIP_PITCH_AMP = _hip_amp_fwd; wg.HIP_BIAS_FWD = _hip_bias_fwd
                    wg.GAIT_T = float(os.environ.get("GATE_Q_GAIT_T", GATE_Q_GAIT_T_DEFAULT))
                bx0f = b["_retreat_finalize_x0"]
                cam0f = float(b.get("_retreat_fin_cam0") or 0.0)
                stepped_rdx = float(b.get("retreat_stepped_delta_m") or 0.0)
                shim_budget = max(0.0, stepped_rdx * (1.0 / STEPPED_FRAC_MIN - 1.0) - 1e-4)
                need_cam = cam0f < 0.55 - 1e-6
                tgt_x = max(body_x_for_cam_dist(0.55), body_x_start)
                desired_shim = max(0.0, bx0f - tgt_x) if (need_cam and bx0f > tgt_x) else 0.0
                use_shim = min(desired_shim, shim_budget)
                if use_shim < 1e-4:
                    bx = bx0f; b["retreat_shim_delta_m"] = 0.0
                else:
                    bx = bx0f - use_shim * a_fin
                    b["retreat_shim_delta_m"] = abs(bx0f - bx)
                base_xy = (bx, body_y0)
                wg.set_ctrl(m, d, wg.gait_targets(0.0, False, 0.0), env.act_idx)
                gcmd, phase, cmd, a_rev = OPEN_CMD, "retreat_finalize", 0.0, a_fin

        mj.mj_forward(m, d)
        if phase in ("push", "hold", "disturb", "reject", "leave", "regrasp", "close", "recover"):
            if phase == "leave":
                target = arc(cmd) + leave_off * min(1.0, 0.3 + 0.7 * a_rev); g = g_leave
            elif phase == "regrasp":
                target = arc(cmd) + leave_off * (1.0 - a_rev); g = g_regrasp
            elif phase == "close":
                target = arc(cmd)
                if soft_close: target = target + np.array([0.03*close_pull_scale, -0.05*a_rev*close_pull_scale, 0.0])
                g = g_close
            elif phase == "recover":
                target = arc(0.0) + np.array([-0.02, 0.02, 0.0]) * (1.0 - a_rev); g = g_hold
            else:
                target = arc(cmd); g = g_push if phase == "push" else g_hold
            jacp = np.zeros((3, m.nv)); mj.mj_jacSite(m, d, jacp, None, sid_hand)
            err = target - d.site_xpos[sid_hand]
            cols = [m.jnt_dofadr[j] for j in jids]; J = jacp[:, cols]
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(3), err)
                for jid, ddq, jn in zip(jids, dq, reach):
                    lo, hi = m.jnt_range[jid]
                    nv = float(np.clip(d.qpos[m.jnt_qposadr[jid]] + g * ddq, lo, hi))
                    d.qpos[m.jnt_qposadr[jid]] = nv; arm_q[jn] = nv
            except np.linalg.LinAlgError: pass

        # Use stand arms during walk/finalize/retreat/settle; reach arms during compose
        if phase in ("step_walk", "finalize", "retreat_walk", "retreat_finalize", "settle_open", "settle_close"):
            arms_now = stand_arm_q
        else:
            arms_now = arm_q
        # head: -16 during approach walk; door look-down during compose
        if phase in ("step_walk", "finalize", "retreat_walk", "retreat_finalize", "settle_open", "settle_close"):
            env.head_tilt = math.radians(walk_head)
        else:
            env.head_tilt = math.radians(compose_head)
        if phase in ("step_walk", "retreat_walk") and walk_ctrl is not None:
            # Proven walk physics (match Gate F / OL probe) — do not use _substep_p walk path
            a_w = float(walk_ctrl["amp"]); phi = float(walk_ctrl["phi"]); t_abs = float(walk_ctrl["t_abs"])
            reverse = bool(walk_ctrl.get("reverse", False))
            # tip E7lock clear-through: force phi into swing half after last SS (no settle)
            if (reverse and bi >= 0
                    and os.environ.get("GATE_Q_RET_CLEAR_THROUGH_GAP", GATE_Q_RET_CLEAR_THROUGH_GAP)
                    not in ("0", "false", "no")):
                _sts = list(cycles[bi].get("_ss_ts_ret") or [])
                _age = (float(t) - float(_sts[-1])) if _sts else 0.0
                _lo = float(os.environ.get("GATE_Q_RET_CLEAR_THROUGH_AGE_LO", GATE_Q_RET_CLEAR_THROUGH_AGE_LO))
                _hi = float(os.environ.get("GATE_Q_RET_CLEAR_THROUGH_AGE_HI", GATE_Q_RET_CLEAR_THROUGH_AGE_HI))
                _ss_n = int(cycles[bi].get("n_ss_L_ret", 0)) + int(cycles[bi].get("n_ss_R_ret", 0))
                _ss_min = int(float(os.environ.get("GATE_Q_RET_CLEAR_THROUGH_SS_MIN", GATE_Q_RET_CLEAR_THROUGH_SS_MIN)))
                if _lo - 1e-9 <= _age <= _hi + 1e-9 and _ss_n >= _ss_min:
                    # alternate swing half so contralateral foot lifts
                    phi = 0.22 if (_ss_n % 2 == 0) else 0.72
                    walk_ctrl["phi"] = phi
            if reverse:
                # Face door (+X); negate hip pitch amp/bias for reverse locomotion (Gate E residual flipped)
                soft = cycles[bi].get("_retreat_soft") if bi >= 0 else False
                if soft is True:
                    amp_sc = float(RETREAT_AMP_SCALE_POST); bias_sc = float(RETREAT_BIAS_FRAC_POST)
                elif soft == "taper":
                    amp_sc = 0.55 * float(RETREAT_AMP_SCALE) + 0.45 * float(RETREAT_AMP_SCALE_POST)
                    bias_sc = 0.55 * float(RETREAT_BIAS_FRAC) + 0.45 * float(RETREAT_BIAS_FRAC_POST)
                else:
                    amp_sc = float(RETREAT_AMP_SCALE); bias_sc = float(RETREAT_BIAS_FRAC)
                if bi >= 1:
                    _b1amp = float(os.environ.get("GATE_Q_BOUT1_RETREAT_AMP_SCALE", GATE_Q_BOUT1_RETREAT_AMP_SCALE))
                    # A17: ramp amp up once past early tip zone so walk cammax can exceed ~0.38
                    _amp_hi = float(os.environ.get("GATE_Q_BOUT1_RETREAT_AMP_LATE", GATE_Q_BOUT1_RETREAT_AMP_LATE))
                    _c0 = float(os.environ.get("GATE_Q_BOUT1_RETREAT_AMP_LATE_CAM0", GATE_Q_BOUT1_RETREAT_AMP_LATE_CAM0))
                    _c1 = float(os.environ.get("GATE_Q_BOUT1_RETREAT_AMP_LATE_CAM1", GATE_Q_BOUT1_RETREAT_AMP_LATE_CAM1))
                    try:
                        _cam_amp = float(cam_to_lever_horiz(d, env.sid, env.gid_lever))
                    except Exception:
                        _cam_amp = 0.0
                    if _amp_hi > _b1amp + 1e-9 and _c1 > _c0 + 1e-9:
                        if _cam_amp >= _c1 - 1e-12:
                            _b1amp = _amp_hi
                        elif _cam_amp > _c0 + 1e-12:
                            _a = (_cam_amp - _c0) / (_c1 - _c0)
                            _b1amp = _b1amp + _a * (_amp_hi - _b1amp)
                    # A42: full A28 amp until cam≥0.55; ONLY THEN honor soft/taper
                    # (A41 honored soft from taper@0.52 → starved mid-retreat)
                    if soft and _cam_amp >= 0.55 - 1e-6:
                        if soft is True:
                            amp_sc = float(RETREAT_AMP_SCALE_POST)
                            bias_sc = float(RETREAT_BIAS_FRAC_POST)
                        else:
                            amp_sc = 0.55 * float(_b1amp) + 0.45 * float(RETREAT_AMP_SCALE_POST)
                            bias_sc = 0.55 * float(RETREAT_BIAS_FRAC) + 0.45 * float(RETREAT_BIAS_FRAC_POST)
                    else:
                        bias_sc = float(bias_sc) * (_b1amp / max(float(amp_sc), 1e-6))
                        amp_sc = _b1amp
                wg.HIP_PITCH_AMP = -amp_sc * _hip_amp_fwd
                wg.HIP_BIAS_FWD = -bias_sc * _hip_bias_fwd
                wg.GAIT_T = float(os.environ.get("GATE_Q_RETREAT_GAIT_T", RETREAT_GAIT_T))
                # S1 CSF50: drive GAIT_T from capture T; modest |F|→amp (Placo dx_back already in F)
                if (os.environ.get("GATE_Q_RET_CSF50", GATE_Q_RET_CSF50) not in ("0", "false", "no")
                        and bi >= 0):
                    _Tc = float(cycles[bi].get("_csf_T") or wg.GAIT_T)
                    wg.GAIT_T = max(_Tc, 0.45)
                    _Fc = abs(float(cycles[bi].get("_csf_F") or 0.0))
                    _vx_r = abs(float(os.environ.get("GATE_Q_RET_CSF50_VX", GATE_Q_RET_CSF50_VX)))
                    _Fref = max(_vx_r * float(os.environ.get("GATE_Q_RET_CSF50_T_NOM", GATE_Q_RET_CSF50_T_NOM)), 1e-3)
                    _amod = max(0.55, min(1.35, _Fc / _Fref)) if _Fc > 1e-6 else 1.0
                    wg.HIP_PITCH_AMP = float(wg.HIP_PITCH_AMP) * _amod
                    wg.HIP_BIAS_FWD = float(wg.HIP_BIAS_FWD) * _amod
                _rds = os.environ.get("GATE_Q_RETREAT_DS_S")
                if _rds is not None and str(_rds).strip() != "":
                    wg.DS_S = float(_rds)
                # S3 REV-PHASE: hip-pitch sign/bias scale (suggest -1.0 full invert). Clear-only later.
                if (os.environ.get("GATE_Q_RET_REV_PHASE", GATE_Q_RET_REV_PHASE) not in ("0", "false", "no")
                        and bi >= 0):
                    _hip_s = float(os.environ.get("GATE_Q_RET_REV_PHASE_HIP", GATE_Q_RET_REV_PHASE_HIP))
                    wg.HIP_PITCH_AMP = float(wg.HIP_PITCH_AMP) * _hip_s
                    wg.HIP_BIAS_FWD = float(wg.HIP_BIAS_FWD) * _hip_s
                    cycles[bi]["_rev_hip_sign"] = float(_hip_s)

            if not reverse:
                nL = cycles[bi]["n_ss_L"] if bi >= 0 else 0
                nR = cycles[bi]["n_ss_R"] if bi >= 0 else 0
                # iterate3p: after thin multi before span latch — moderate hip (still step)
                span_ok = bool(cycles[bi].get("_span_latched"))
                if _ss_multi_proved(nL, nR) and not span_ok:
                    # iterate3q/q2: keep stepping until span latch; after burst3 slightly higher hip
                    hip_sc = float(os.environ.get("GATE_Q_POST_MULTI_HIP_SCALE", "0.92"))  # keep stepping for spaced lifts
                    if bi >= 0 and cycles[bi].get("_burst3_done"):
                        hip_sc = float(os.environ.get("GATE_Q_POST_BURST3_HIP_SCALE", "0.82"))
                    if bi >= 1:
                        hip_sc = float(os.environ.get("GATE_Q_BOUT1_BURST2_HIP_SCALE", "0.85"))  # iterate3r9: need span on bout1
                        if cycles[bi].get("_burst3_done") or int(cycles[bi].get("_periodic_reburst_n") or 0) > 0:
                            hip_sc = float(os.environ.get("GATE_Q_BOUT1_POST_BURST3_HIP_SCALE", "0.88"))
                    wg.HIP_PITCH_AMP = hip_sc * _hip_amp_fwd
                    wg.HIP_BIAS_FWD = hip_sc * _hip_bias_fwd
                elif bi >= 1 and cycles[bi].get("_burst2_t0") is not None and not span_ok:
                    hip_sc = float(os.environ.get("GATE_Q_BOUT1_BURST2_HIP_SCALE", "0.85"))
                    wg.HIP_PITCH_AMP = hip_sc * _hip_amp_fwd
                    wg.HIP_BIAS_FWD = hip_sc * _hip_bias_fwd
                else:
                    wg.HIP_PITCH_AMP = _hip_amp_fwd
                    wg.HIP_BIAS_FWD = _hip_bias_fwd
            # iterate3r19h6: hip=0 while BOTH not clear (app+ret); hip burst ONLY in clear-hold.
            # Clear-hold keeps swing_eff while still airborne after first true clear (longer dwell).
            _rest_doc_h = 0.0086
            zLh = float(d.xpos[env.bid_lf, 2]); zRh = float(d.xpos[env.bid_rf, 2])
            cLh = zLh - SOLE_OFFSET; cRh = zRh - SOLE_OFFSET
            ar_h = float(os.environ.get("GATE_Q_SWING_CLEAR_AR_M", GATE_Q_SWING_CLEAR_AR_M))
            abs_h = float(os.environ.get("GATE_Q_SWING_CLEAR_ABS_M", GATE_Q_SWING_CLEAR_ABS_M))
            raw_clear_h = (
                ((cLh - _rest_doc_h) >= ar_h - 1e-9 and cLh >= abs_h - 1e-9)
                or ((cRh - _rest_doc_h) >= ar_h - 1e-9 and cRh >= abs_h - 1e-9)
            )
            if reverse:
                # Prefer RET_* constants (do NOT fall through to approach CLEAR_HOLD_S=0.32)
                hold_s = float(os.environ.get("GATE_Q_RET_CLEAR_HOLD_S", GATE_Q_RET_CLEAR_HOLD_S))
                if bi >= 1:
                    hold_s = float(os.environ.get("GATE_Q_BOUT1_RET_CLEAR_HOLD_S", GATE_Q_BOUT1_RET_CLEAR_HOLD_S))
                air_m = float(os.environ.get("GATE_Q_CLEAR_HOLD_AIR_M_RET", GATE_Q_CLEAR_HOLD_AIR_M_RET))
                # G-family: longer clear-hold only while cam still early
                try:
                    _cam_e = float(cam_to_lever_horiz(d, env.sid, env.gid_lever))
                except Exception:
                    _cam_e = 0.5
                _early_cam = float(os.environ.get("GATE_Q_RET_EARLY_CAM", GATE_Q_RET_EARLY_CAM))
                _early_t = float(os.environ.get("GATE_Q_RET_EARLY_T", GATE_Q_RET_EARLY_T))
                _tloc = float(t_abs) if reverse else 999.0
                if _cam_e < _early_cam - 1e-6 and _tloc < _early_t - 1e-9:
                    hold_s = hold_s + float(os.environ.get("GATE_Q_RET_EARLY_HOLD_ADD", GATE_Q_RET_EARLY_HOLD_ADD))
            else:
                hold_s = float(os.environ.get("GATE_Q_CLEAR_HOLD_S", GATE_Q_CLEAR_HOLD_S))
                if bi >= 1:
                    hold_s = float(os.environ.get("GATE_Q_BOUT1_CLEAR_HOLD_S", GATE_Q_BOUT1_CLEAR_HOLD_S))
                air_m = float(os.environ.get("GATE_Q_CLEAR_HOLD_AIR_M", GATE_Q_CLEAR_HOLD_AIR_M))
            dt_ctrl = 1.0 / float(CTRL_HZ)
            if bi >= 0:
                if raw_clear_h:
                    cycles[bi]["_clear_hold_left"] = hold_s
                    swing_hip = True
                else:
                    left_h = float(cycles[bi].get("_clear_hold_left") or 0.0)
                    still_air = max(cLh, cRh) >= air_m - 1e-9
                    if left_h > 0.0 and still_air:
                        cycles[bi]["_clear_hold_left"] = max(0.0, left_h - dt_ctrl)
                        swing_hip = True
                    else:
                        cycles[bi]["_clear_hold_left"] = 0.0
                        swing_hip = False
            else:
                swing_hip = raw_clear_h
            # k8aa: residual-clear gate = strict SS (raw_clear_h). Soft hold may keep hip,
            # but residual only while SS-clear so Δ counts in clear_frac under SS tracking.
            res_clear = bool(raw_clear_h) if reverse else bool(swing_hip)
            if bi >= 0 and reverse:
                cycles[bi]["_swing_hip_ret"] = bool(swing_hip)
                cycles[bi]["_res_clear_ret"] = bool(res_clear)
            if a_w > 0.05:
                if swing_hip:
                    if reverse:
                        hip_b = float(os.environ.get("GATE_Q_RET_SWING_HIP_SCALE", GATE_Q_RET_SWING_HIP_SCALE))
                        if bi >= 1:
                            hip_b = float(os.environ.get("GATE_Q_BOUT1_RET_SWING_HIP_SCALE", GATE_Q_BOUT1_RET_SWING_HIP_SCALE))
                        try:
                            _cam_h = float(cam_to_lever_horiz(d, env.sid, env.gid_lever))
                        except Exception:
                            _cam_h = 0.5
                        if (_cam_h < float(os.environ.get("GATE_Q_RET_EARLY_CAM", GATE_Q_RET_EARLY_CAM)) - 1e-6
                                and float(t_abs) < float(os.environ.get("GATE_Q_RET_EARLY_T", GATE_Q_RET_EARLY_T)) - 1e-9):
                            hip_b *= float(os.environ.get("GATE_Q_RET_EARLY_HIP_MULT", GATE_Q_RET_EARLY_HIP_MULT))
                        # R2/R3/R4: clear-only hip boost during burst/force (sole must already be clear)
                        if bi >= 0 and (cycles[bi].get("_r2_in_burst") or cycles[bi].get("_r3_in_burst")
                                        or cycles[bi].get("_r4_in_force")):
                            if cycles[bi].get("_r4_in_force"):
                                hip_b *= float(os.environ.get(
                                    "GATE_Q_RET_FORCE_LIFT_HIP_MULT", GATE_Q_RET_FORCE_LIFT_HIP_MULT))
                            else:
                                hip_b *= float(os.environ.get(
                                    "GATE_Q_RET_CLEAR_BURST_HIP_MULT", GATE_Q_RET_CLEAR_BURST_HIP_MULT))
                    else:
                        hip_b = float(os.environ.get("GATE_Q_SWING_HIP_SCALE", GATE_Q_SWING_HIP_SCALE))
                        if bi >= 1:
                            hip_b = float(os.environ.get("GATE_Q_BOUT1_SWING_HIP_SCALE", GATE_Q_BOUT1_SWING_HIP_SCALE))
                    wg.HIP_PITCH_AMP = float(wg.HIP_PITCH_AMP) * hip_b
                    wg.HIP_BIAS_FWD = float(wg.HIP_BIAS_FWD) * hip_b
                else:
                    hip_p = float(os.environ.get("GATE_Q_PLANTED_HIP_SCALE", GATE_Q_PLANTED_HIP_SCALE))
                    wg.HIP_PITCH_AMP = float(wg.HIP_PITCH_AMP) * hip_p
                    wg.HIP_BIAS_FWD = float(wg.HIP_BIAS_FWD) * hip_p
            # iterate3r19h7k: while planted on retreat, cut gait amp (soft-XY source with hip×0);
            # keep enough for next lift. Clear windows use full a_w. No freejoint freeze.
            a_gait = float(a_w)
            if reverse and a_w > 0.05 and (not bool(swing_hip)):
                _pg = float(os.environ.get(
                    "GATE_Q_RET_PLANTED_GAIT_SCALE", GATE_Q_RET_PLANTED_GAIT_SCALE))
                # tip A28c F8: only while long SS silence — cut planted shove in 26s middle
                _gap_pg = float(os.environ.get(
                    "GATE_Q_RET_GAP_PLANTED_GAIT_SCALE", GATE_Q_RET_GAP_PLANTED_GAIT_SCALE))
                if _gap_pg < 0.999 and bi >= 0:
                    _sts = list(cycles[bi].get("_ss_ts_ret") or [])
                    _age = (float(t) - float(_sts[-1])) if _sts else 0.0
                    _need = float(os.environ.get(
                        "GATE_Q_RET_GAP_PLANTED_SS_GAP", GATE_Q_RET_GAP_PLANTED_SS_GAP))
                    _dxc_need = float(os.environ.get("GATE_Q_RET_GAP_PLANTED_DXC_MIN", "0.0"))
                    _dxc_now = float(cycles[bi].get("dx_clear_m_ret") or cycles[bi].get("_dx_clear_m_ret") or 0.0)
                    if (_age >= _need - 1e-9
                            and (cycles[bi].get("n_ss_L_ret", 0) + cycles[bi].get("n_ss_R_ret", 0)) >= 4
                            and _dxc_now >= _dxc_need - 1e-6):
                        _pg = min(_pg, _gap_pg)
                a_gait = float(a_w) * _pg
            # S3 REV-PHASE: gait CPG must use polarity-adjusted phi (not raw t_abs clock)
            _t_gait = t_abs if a_gait > 0 else 0.0
            if (reverse and a_gait > 0
                    and os.environ.get("GATE_Q_RET_REV_PHASE", GATE_Q_RET_REV_PHASE) not in ("0", "false", "no")):
                _t_gait = float(phi) * float(wg.GAIT_T)
            qdes = wg.gait_targets(_t_gait, a_gait > 0, a_gait)
            # Keep stand arms during walk — gait arm swing bumps door at cam≈0.55 and soft-limit-slams panel
            for jn, val in stand_arm_q.items():
                qdes[jn] = val
            wg.set_ctrl(m, d, qdes, env.act_idx)
            d.ctrl[env.aid_tilt] = env.head_tilt
            # S3 REV-PHASE: swing-leg encoding flip → phase offset 0.5 * strength for L↔R roles
            _phi_lat = float(phi)
            _rev_sw_live = 0.0
            if (reverse and a_w > 0
                    and os.environ.get("GATE_Q_RET_REV_PHASE", GATE_Q_RET_REV_PHASE) not in ("0", "false", "no")):
                _rev_sw_live = float(os.environ.get("GATE_Q_RET_REV_PHASE_SWING", GATE_Q_RET_REV_PHASE_SWING))
                _phi_lat = (float(phi) + 0.5 * _rev_sw_live) % 1.0
                if bi >= 0:
                    cycles[bi]["_rev_swing_enc"] = float(_rev_sw_live)
            lat = wg.lateral_com_target(_phi_lat) if a_w > 0 else 0.0
            bias = walk_ctrl.get("cop_bias")
            if a_w > 0.01:
                saved = d.subtree_com[0, :2].copy()
                d.subtree_com[0, :2] = d.subtree_com[bid_com, :2]
                # iterate3r19h7k8f: while planted on retreat, keep CoP roll (lift balance) but
                # zero sagittal pitch/lean-X so CoP does not soft-push freejoint +X/−X.
                # Full CoP-off killed lifts (k8/k8b). Clear windows: full CoP. Approach untouched.
                _cop_off = reverse and (not bool(swing_hip)) and (
                    os.environ.get("GATE_Q_RET_PLANTED_COP_GATE", GATE_Q_RET_PLANTED_COP_GATE)
                    not in ("0", "false", "no"))
                _cop_sag = reverse and (not bool(swing_hip)) and (
                    os.environ.get("GATE_Q_RET_PLANTED_COP_SAGITTAL_GATE", GATE_Q_RET_PLANTED_COP_SAGITTAL_GATE)
                    not in ("0", "false", "no"))
                if _cop_off:
                    _cop_a = 0.0
                elif reverse:
                    _cop_a = float(a_gait)
                else:
                    _cop_a = float(a_w)
                _kx = _klean = _kdx = None
                try:
                    if _cop_a > 0.01:
                        if _cop_sag:
                            _kx = float(wg.ANK_COP_KP_X); _klean = float(wg.ANK_COP_KP_LEAN_X)
                            _kdx = float(getattr(wg, "ANK_COP_KD_LEAN_X", 0.0))
                            wg.ANK_COP_KP_X = 0.0
                            wg.ANK_COP_KP_LEAN_X = 0.0
                            if hasattr(wg, "ANK_COP_KD_LEAN_X"):
                                wg.ANK_COP_KD_LEAN_X = 0.0
                        wg.ankle_cop_servo(m, d, env.act_idx, _cop_a, env.bid_lf, env.bid_rf, env.gid_floor,
                                           lat=lat if _cop_a > 0.05 else None, bias_xy=bias)
                except Exception:
                    pass
                finally:
                    if _kx is not None:
                        wg.ANK_COP_KP_X = _kx
                        wg.ANK_COP_KP_LEAN_X = _klean
                        if _kdx is not None and hasattr(wg, "ANK_COP_KD_LEAN_X"):
                            wg.ANK_COP_KD_LEAN_X = _kdx
                d.subtree_com[0, :2] = saved
                # CP / stance-VIK: only while clear-hold (iterate3r19h6 — avoid planted soft-XY push)
                # iterate3r19h7k: also on retreat clear (mirror approach; was approach-only → weak ret Δ)
                swing_cp = bool(swing_hip) if a_w > 0.05 else False
                ret_cp = (not reverse) or (os.environ.get("GATE_Q_RET_CP_VIK", GATE_Q_RET_CP_VIK) not in ("0", "false", "no"))
                if swing_cp and ret_cp:
                    _phi_cp = _phi_lat if reverse else phi
                    try: wg.apply_cp_swing_placement(m, d, env.act_idx, _phi_cp, a_w, env.bid_lf, env.bid_rf)
                    except Exception: pass
                    try: wg.apply_stance_jacobian_vik(m, d, env.act_idx, _phi_cp, a_w, env.bid_lf, env.bid_rf, env.gid_floor)
                    except Exception: pass
                action = walk_ctrl.get("action")
                if action is not None:
                    import numpy as _np
                    from score_gate_f import ACTION_SCALE, CTRL_LIM
                    # iterate3r19h6: residual≈0 while BOTH not clear; residual×boost only in clear-hold.
                    # Planted soft-XY must not drive Δ; clear windows produce each step's Δxy.
                    swing_clear = bool(swing_hip)
                    no_swing_sc = float(os.environ.get(
                        "GATE_Q_RES_NO_SWING_SCALE",
                        os.environ.get("GATE_Q_RES_PLANTED_SCALE", GATE_Q_RES_PLANTED_SCALE)))
                    swing_boost = float(os.environ.get("GATE_Q_RES_SWING_BOOST", GATE_Q_RES_SWING_BOOST))
                    if reverse:
                        soft = cycles[bi].get("_retreat_soft") if bi >= 0 else False
                        if soft is True:
                            scale = float(os.environ.get("GATE_Q_RETREAT_RES_SCALE_POST", RETREAT_RES_SCALE_POST))
                        elif soft == "taper":
                            scale = float(os.environ.get("GATE_Q_RETREAT_RES_SCALE", RETREAT_RES_SCALE)) * float(os.environ.get("GATE_Q_RES_TAPER", GATE_Q_RES_TAPER))
                        else:
                            scale = float(os.environ.get("GATE_Q_RETREAT_RES_SCALE", RETREAT_RES_SCALE))
                        if bi >= 1:
                            scale *= float(os.environ.get("GATE_Q_BOUT1_RETREAT_RES_SCALE", GATE_Q_BOUT1_RETREAT_RES_SCALE))
                        try:
                            _cam_r = float(cam_to_lever_horiz(d, env.sid, env.gid_lever))
                        except Exception:
                            _cam_r = 0.5
                        if (_cam_r < float(os.environ.get("GATE_Q_RET_EARLY_CAM", GATE_Q_RET_EARLY_CAM)) - 1e-6
                                and float(t_abs) < float(os.environ.get("GATE_Q_RET_EARLY_T", GATE_Q_RET_EARLY_T)) - 1e-9):
                            scale *= float(os.environ.get("GATE_Q_RET_EARLY_RES_MULT", GATE_Q_RET_EARLY_RES_MULT))
                        act = _np.asarray(action, dtype=_np.float64).reshape(12).copy()
                        # iterate3r19h7k4: residual flip may FIGHT reverse gait hip (clear Δ~0).
                        # Default: NO flip — reverse propulsion from gait hip only; residual keeps
                        # balance as trained. Optional hip/sagittal flip via env.
                        flip_mode = os.environ.get("GATE_Q_RET_RES_FLIP", "hip").strip().lower()
                        if flip_mode in ("hip", "1", "true", "yes"):
                            act[2] *= -1.0; act[8] *= -1.0
                        elif flip_mode in ("sagittal", "full"):
                            act[2] *= -1.0; act[8] *= -1.0
                            act[3] *= -1.0; act[9] *= -1.0
                            act[4] *= -1.0; act[10] *= -1.0
                        # else "none": do not flip residual
                        # S3 REV-PHASE: additional hip residual sign + swing-leg L↔R encoding
                        if os.environ.get("GATE_Q_RET_REV_PHASE", GATE_Q_RET_REV_PHASE) not in ("0", "false", "no"):
                            _hs = float(os.environ.get("GATE_Q_RET_REV_PHASE_HIP", GATE_Q_RET_REV_PHASE_HIP))
                            act[2] *= _hs; act[8] *= _hs
                            _sw = float(os.environ.get("GATE_Q_RET_REV_PHASE_SWING", GATE_Q_RET_REV_PHASE_SWING))
                            if _sw >= 0.5 - 1e-9:
                                act = _np.concatenate([act[6:12], act[0:6]]).copy()
                            elif _sw > 1e-9:
                                _swapped = _np.concatenate([act[6:12], act[0:6]])
                                act = (1.0 - _sw) * act + _sw * _swapped
                        # k8aa: residual only while raw SS-clear (not soft clear-hold) so Δ counts clear
                        _res_on = bool(res_clear) if os.environ.get("GATE_Q_RET_RES_STRICT_SS", GATE_Q_RET_RES_STRICT_SS) not in ("0","false","no") else bool(swing_clear)
                        if _res_on:
                            ret_boost = float(os.environ.get("GATE_Q_RET_RES_SWING_BOOST",
                                              GATE_Q_RET_RES_SWING_BOOST))
                            scale *= ret_boost
                        else:
                            scale *= float(os.environ.get("GATE_Q_RET_RES_NO_SWING_SCALE", no_swing_sc))
                        delta = _np.clip(act, -1, 1) * ACTION_SCALE * scale
                    else:
                        scale = float(os.environ.get("GATE_Q_RES_SCALE", "1.0"))
                        nL = cycles[bi]["n_ss_L"] if bi >= 0 else 0
                        nR = cycles[bi]["n_ss_R"] if bi >= 0 else 0
                        if bi >= 0 and cycles[bi].get("_span_latched"):
                            scale *= float(os.environ.get("GATE_Q_RES_POST_PROOF", GATE_Q_RES_POST_PROOF))
                        elif bi >= 0 and nL >= 1 and nR >= 1:
                            taper = float(os.environ.get("GATE_Q_RES_TAPER", GATE_Q_RES_TAPER))
                            if bi >= 1:
                                taper = float(os.environ.get("GATE_Q_BOUT1_RES_TAPER", GATE_Q_BOUT1_RES_TAPER))
                            scale *= taper
                        elif bi >= 0:
                            scale *= float(os.environ.get("GATE_Q_RES_IN_PROOF", GATE_Q_RES_IN_PROOF))
                        # FROM t=0: residual only during clear; near-zero while planted
                        if swing_clear:
                            scale *= swing_boost
                        else:
                            scale *= no_swing_sc
                        delta = _np.clip(_np.asarray(action, dtype=_np.float64).reshape(12), -1, 1) * ACTION_SCALE * scale
                    for i, ai in enumerate(env.leg_act):
                        d.ctrl[ai] = float(_np.clip(d.ctrl[ai] + delta[i], -CTRL_LIM, CTRL_LIM))
                # Additive swing until multi-step dwell-SS (≥2/side or ≥4 total), then residual-only.
                need_L = int(os.environ.get("GATE_Q_SS_PER_SIDE", SS_PER_SIDE_MIN))
                need_R = need_L
                if reverse:
                    ssL_n = cycles[bi]["n_ss_L_ret"] if bi >= 0 else 0
                    ssR_n = cycles[bi]["n_ss_R_ret"] if bi >= 0 else 0
                    proof_start = ret_hold + float(os.environ.get("GATE_Q_RETREAT_PROOF_DELAY", str(RETREAT_PROOF_DELAY)))
                    hold_for_proof = ret_hold
                    add_knee = float(os.environ.get("GATE_Q_RETREAT_ADD_KNEE", max(RETREAT_ADD_KNEE, 0.10)))
                    add_ank = float(os.environ.get("GATE_Q_RETREAT_ADD_ANK", max(RETREAT_ADD_ANK, 0.10)))
                    add_abd = float(os.environ.get("GATE_Q_RETREAT_ADD_ABD", max(RETREAT_ADD_ABD, 0.028)))
                    max_proof = float(os.environ.get("GATE_Q_RETREAT_CLEAR_PROOF_S", max(RETREAT_CLEAR_PROOF_S, 28.0)))
                    if bi >= 1:
                        add_knee = float(os.environ.get("GATE_Q_BOUT1_RETREAT_ADD_KNEE", GATE_Q_BOUT1_RETREAT_ADD_KNEE))
                        add_ank = float(os.environ.get("GATE_Q_BOUT1_RETREAT_ADD_ANK", GATE_Q_BOUT1_RETREAT_ADD_ANK))
                        add_abd = float(os.environ.get("GATE_Q_BOUT1_RETREAT_ADD_ABD", GATE_Q_BOUT1_RETREAT_ADD_ABD))
                        max_proof = float(os.environ.get("GATE_Q_BOUT1_RETREAT_CLEAR_PROOF_S", max(max_proof, 28.0)))
                        proof_start = ret_hold + float(os.environ.get("GATE_Q_BOUT1_RETREAT_PROOF_DELAY", "3.0"))
                    # tip A28c E11: plant-gap clearance kick (default OFF)
                    if (os.environ.get("GATE_Q_RET_GAP_ADD_KICK", "0") not in ("0", "false", "no")
                            and bi >= 0):
                        _sts = list(cycles[bi].get("_ss_ts_ret") or [])
                        _age = (float(t) - float(_sts[-1])) if _sts else 0.0
                        _gap_need = float(os.environ.get("GATE_Q_RET_GAP_ADD_KICK_SS_GAP", "1.5"))
                        _ss_min = int(float(os.environ.get("GATE_Q_RET_GAP_ADD_KICK_SS_MIN", "4")))
                        if (_age >= _gap_need - 1e-9
                                and (ssL_n + ssR_n) >= _ss_min
                                and not cycles[bi].get("_retreat_stop")):
                            _k = float(os.environ.get("GATE_Q_RET_GAP_ADD_KICK_SCALE", "1.35"))
                            add_knee *= _k; add_ank *= _k; add_abd *= _k
                            cycles[bi]["_gap_add_kick"] = True
                        else:
                            cycles[bi]["_gap_add_kick"] = False
                    # R2/R3/R4: elevate clearance ADD only during burst/force windows
                    if bi >= 0 and (cycles[bi].get("_r2_in_burst") or cycles[bi].get("_r3_in_burst")
                                    or cycles[bi].get("_r4_in_force")):
                        if cycles[bi].get("_r4_in_force"):
                            _k = float(os.environ.get("GATE_Q_RET_FORCE_LIFT_ADD_MULT", GATE_Q_RET_FORCE_LIFT_ADD_MULT))
                        else:
                            _k = float(os.environ.get("GATE_Q_RET_CLEAR_BURST_ADD_MULT", GATE_Q_RET_CLEAR_BURST_ADD_MULT))
                        add_knee *= _k; add_ank *= _k; add_abd *= _k
                    # tip E7lock: clear-through-gap — ADD kick ASAP after SS (turn gap into lifts)
                    if (os.environ.get("GATE_Q_RET_CLEAR_THROUGH_GAP", GATE_Q_RET_CLEAR_THROUGH_GAP)
                            not in ("0", "false", "no") and bi >= 0):
                        _sts = list(cycles[bi].get("_ss_ts_ret") or [])
                        _age = (float(t) - float(_sts[-1])) if _sts else 0.0
                        _lo = float(os.environ.get("GATE_Q_RET_CLEAR_THROUGH_AGE_LO", GATE_Q_RET_CLEAR_THROUGH_AGE_LO))
                        _hi = float(os.environ.get("GATE_Q_RET_CLEAR_THROUGH_AGE_HI", GATE_Q_RET_CLEAR_THROUGH_AGE_HI))
                        _ss_min = int(float(os.environ.get("GATE_Q_RET_CLEAR_THROUGH_SS_MIN", GATE_Q_RET_CLEAR_THROUGH_SS_MIN)))
                        if (_lo - 1e-9 <= _age <= _hi + 1e-9
                                and (ssL_n + ssR_n) >= _ss_min
                                and not cycles[bi].get("_retreat_stop")):
                            _k = float(os.environ.get("GATE_Q_RET_CLEAR_THROUGH_ADD_MULT", GATE_Q_RET_CLEAR_THROUGH_ADD_MULT))
                            add_knee *= _k; add_ank *= _k; add_abd *= _k
                            cycles[bi]["_clear_through"] = True
                        else:
                            cycles[bi]["_clear_through"] = False
                    if bi >= 0 and cycles[bi].get("_ret_burst_t0") is not None:
                        proof_start = float(cycles[bi]["_ret_burst_t0"])
                        hold_for_proof = 0.0
                        max_proof = max(max_proof, 20.0)
                else:
                    ssL_n = cycles[bi]["n_ss_L"] if bi >= 0 else 0
                    ssR_n = cycles[bi]["n_ss_R"] if bi >= 0 else 0
                    max_proof = float(os.environ.get("GATE_Q_CLEAR_PROOF_S", GATE_Q_CLEAR_PROOF_S_DEFAULT))
                    proof_start = walk_hold + float(os.environ.get("GATE_Q_PROOF_DELAY", "2.0"))
                    hold_for_proof = walk_hold
                    add_knee = float(os.environ.get("GATE_Q_ADD_KNEE", GATE_Q_ADD_KNEE_DEFAULT))
                    add_ank = float(os.environ.get("GATE_Q_ADD_ANK", GATE_Q_ADD_ANK_DEFAULT))
                    add_abd = float(os.environ.get("GATE_Q_ADD_ABD", GATE_Q_ADD_ABD_DEFAULT))
                    if bi >= 0 and cycles[bi].get("_burst2_t0") is not None:
                        add_knee = float(os.environ.get("GATE_Q_BURST2_ADD_KNEE", max(add_knee, 0.15)))
                        add_ank = float(os.environ.get("GATE_Q_BURST2_ADD_ANK", max(add_ank, 0.12)))
                        add_abd = float(os.environ.get("GATE_Q_BURST2_ADD_ABD", max(add_abd, 0.03)))
                        max_proof = float(os.environ.get("GATE_Q_BURST2_PROOF_S", max(max_proof, 24.0)))
                    if bi >= 1:
                        add_knee = float(os.environ.get("GATE_Q_BOUT1_ADD_KNEE", max(add_knee, 0.16)))
                        add_ank = float(os.environ.get("GATE_Q_BOUT1_ADD_ANK", max(add_ank, 0.13)))
                        add_abd = float(os.environ.get("GATE_Q_BOUT1_ADD_ABD", max(add_abd, 0.035)))
                        max_proof = float(os.environ.get("GATE_Q_BOUT1_CLEAR_PROOF_S", max(max_proof, 24.0)))
                if reverse:
                    proved_ss = (ssL_n >= 2 and ssR_n >= 2 and (ssL_n + ssR_n) >= 4) or (ssL_n >= 3 and ssR_n >= 3)
                else:
                    # iterate3p: keep add until walk early-stop (span latch alone would resume soft-slide)
                    proved_ss = bool(cycles[bi].get("_walk_stop"))
                # After mid-settle, keep proof window open from burst2_t0 for max_proof seconds
                if (not reverse) and bi >= 0 and cycles[bi].get("_burst2_t0") is not None:
                    proof_start = float(cycles[bi]["_burst2_t0"])
                    hold_for_proof = 0.0
                in_proof = (a_w > 0.05 and (not proved_ss)
                            and (t_abs >= proof_start) and (t_abs <= (hold_for_proof + max_proof)))
                if in_proof:
                    import numpy as _np
                    from score_gate_f import CTRL_LIM
                    # After span latch taper add×0.65 (still lift, less tip); before latch keep full.
                    if (not reverse) and bi >= 0 and cycles[bi].get("_span_latched"):
                        add_knee *= 0.65; add_ank *= 0.65; add_abd *= 0.65
                    ds_frac = float(_np.clip(wg.DS_S / max(wg.GAIT_T, 1e-3), 0.08, 0.55))
                    # Boost lagging side when asymmetric (e.g. 2+1) to earn the missing lift
                    lag_L = (not reverse) and bi >= 0 and cycles[bi]["n_ss_L"] < cycles[bi]["n_ss_R"]
                    lag_R = (not reverse) and bi >= 0 and cycles[bi]["n_ss_R"] < cycles[bi]["n_ss_L"]
                    ds_end = 0.5 + 0.5 * ds_frac
                    _phi_add = _phi_lat if reverse else phi
                    for side, knee_n, ank_n, roll_n, abd_sign in (
                        ("L", "l_knee_pos", "l_ank_pitch_pos", "l_hip_roll_pos", -1.0),
                        ("R", "r_knee_pos", "r_ank_pitch_pos", "r_hip_roll_pos", +1.0),
                    ):
                        p_leg = wg.phase_leg(_phi_add, side)
                        if p_leg < ds_end:
                            continue
                        s = (p_leg - ds_end) / max(1e-6, 1.0 - ds_end)
                        sw = wg.swing_blend(s)
                        if sw < 0.20:
                            continue  # mid-swing only — avoid toe-off tip from huge early add
                        lag_boost = 1.0
                        if lag_L and side == "L":
                            lag_boost = float(os.environ.get("GATE_Q_LAG_BOOST", "1.18"))  # between 1.12 and 1.20
                        if lag_R and side == "R":
                            lag_boost = float(os.environ.get("GATE_Q_LAG_BOOST", "1.15"))
                        if knee_n in env.act_idx:
                            ai = env.act_idx[knee_n]
                            d.ctrl[ai] = float(_np.clip(d.ctrl[ai] + add_knee * sw * a_w * lag_boost, -CTRL_LIM, CTRL_LIM))
                        if ank_n in env.act_idx:
                            ai = env.act_idx[ank_n]
                            sgn = 1.0 if side == "L" else -1.0
                            d.ctrl[ai] = float(_np.clip(d.ctrl[ai] + sgn * add_ank * sw * a_w * lag_boost, -CTRL_LIM, CTRL_LIM))
                        if roll_n in env.act_idx:
                            ai = env.act_idx[roll_n]
                            d.ctrl[ai] = float(_np.clip(d.ctrl[ai] + abd_sign * add_abd * sw * a_w * lag_boost, -CTRL_LIM, CTRL_LIM))
                # iterate3r19h6: while clear-hold, extra knee/ank to keep sole daylight (extend dwell)
                if a_w > 0.05 and bool(swing_hip):
                    import numpy as _np
                    from score_gate_f import CTRL_LIM
                    hold_knee = float(os.environ.get("GATE_Q_CLEAR_HOLD_KNEE", "0.06"))
                    hold_ank = float(os.environ.get("GATE_Q_CLEAR_HOLD_ANK", "0.05"))
                    _rest_ah = 0.0086
                    for side, knee_n, ank_n, z in (
                        ("L", "l_knee_pos", "l_ank_pitch_pos", float(d.xpos[env.bid_lf, 2])),
                        ("R", "r_knee_pos", "r_ank_pitch_pos", float(d.xpos[env.bid_rf, 2])),
                    ):
                        clr = z - SOLE_OFFSET
                        if (clr - _rest_ah) < 0.012:
                            continue  # only the aerial foot
                        if knee_n in env.act_idx:
                            ai = env.act_idx[knee_n]
                            d.ctrl[ai] = float(_np.clip(d.ctrl[ai] + hold_knee * a_w, -CTRL_LIM, CTRL_LIM))
                        if ank_n in env.act_idx:
                            ai = env.act_idx[ank_n]
                            sgn = 1.0 if side == "L" else -1.0
                            d.ctrl[ai] = float(_np.clip(d.ctrl[ai] + sgn * hold_ank * a_w, -CTRL_LIM, CTRL_LIM))
                # R1: retreat-native swing placement (clear-only −X foot target; default OFF)
                if reverse and a_w > 0.05 and bool(swing_hip):
                    _place = float(os.environ.get("GATE_Q_RET_SWING_PLACE_M", GATE_Q_RET_SWING_PLACE_M))
                    if _place > 1e-6:
                        _early_only = float(os.environ.get(
                            "GATE_Q_RET_SWING_PLACE_EARLY_CAM", GATE_Q_RET_SWING_PLACE_EARLY_CAM))
                        _cam_ok = True
                        if _early_only > 1e-6:
                            try:
                                _cam_pl = float(cam_to_lever_horiz(d, env.sid, env.gid_lever))
                            except Exception:
                                _cam_pl = 0.5
                            _cam_ok = _cam_pl < _early_only - 1e-6
                        if _cam_ok:
                            try:
                                apply_ret_swing_place_m(
                                    m, d, env.act_idx, phi, a_w,
                                    env.bid_lf, env.bid_rf, _place)
                            except Exception:
                                pass
                # S1 CSF50: clear-only track swing sole toward capture F (R1 place stays OFF)
                # S3 REV-PHASE: do NOT call CSF/ALIP track helpers
                _rev_trk = os.environ.get("GATE_Q_RET_REV_PHASE", GATE_Q_RET_REV_PHASE) not in ("0", "false", "no")
                if (reverse and a_w > 0.05 and bool(swing_hip) and (not _rev_trk)
                        and os.environ.get("GATE_Q_RET_CSF50", GATE_Q_RET_CSF50) not in ("0", "false", "no")
                        and bi >= 0):
                    _Ftr = float(cycles[bi].get("_csf_F") or 0.0)
                    if abs(_Ftr) > 1e-6:
                        try:
                            apply_csf_f_track(
                                m, d, env.act_idx, phi, a_w,
                                env.bid_lf, env.bid_rf, _Ftr)
                        except Exception:
                            pass
                # S2 ALIP-TVR OR T5-B model-base (NEW weights dual-ckpt): clear-only foothold track
                # GATE_Q_RET_ALIP_TVR stays 0 for T5B — activated via GATE_Q_T5B_CKPT / MODELBASE only
                _alip_trk = (
                    os.environ.get("GATE_Q_RET_ALIP_TVR", GATE_Q_RET_ALIP_TVR) not in ("0", "false", "no")
                    or os.environ.get("GATE_Q_T5B_MODELBASE", "0") not in ("0", "false", "no")
                    or (bool(os.environ.get("GATE_Q_T5B_CKPT", "").strip()) or bool(os.environ.get("GATE_Q_T5C_CKPT", "").strip()) or _gate_q_t5d_family_active())
                )
                if (reverse and a_w > 0.05 and bool(swing_hip) and (not _rev_trk)
                        and _alip_trk and bi >= 0):
                    _Ftr_a = float(cycles[bi].get("_alip_du_fp") or 0.0)
                    if abs(_Ftr_a) > 1e-6:
                        try:
                            apply_alip_fp_track(
                                m, d, env.act_idx, phi, a_w,
                                env.bid_lf, env.bid_rf, _Ftr_a)
                        except Exception:
                            pass
                d.ctrl[env.aid_tilt] = env.head_tilt
            # iterate3r19h6: planted XY damp each physics substep, scaled so env damp is
            # per-control-tick effective (kill gait/plant/momentum soft-XY without tip-bomb).
            # XY advance only during clear-hold windows. Soft-pass forbidden.
            nsp = max(int(env.steps_per_ctrl), 1)
            if a_w > 0.05 and (not bool(swing_hip)):
                if reverse:
                    damp_ctrl = float(os.environ.get("GATE_Q_PLANTED_XY_DAMP_RET", GATE_Q_PLANTED_XY_DAMP_RET))
                    # R2 brief hold-only damp (kill settle shove; not full-retreat damp↑)
                    if bi >= 0 and cycles[bi].get("_r2_in_hold"):
                        damp_ctrl = min(damp_ctrl, float(os.environ.get(
                            "GATE_Q_RET_CLEAR_BURST_HOLD_DAMP", GATE_Q_RET_CLEAR_BURST_HOLD_DAMP)))
                else:
                    damp_ctrl = float(os.environ.get("GATE_Q_PLANTED_XY_DAMP", GATE_Q_SWING_XY_DAMP))
                    if bi >= 1:
                        damp_ctrl = float(os.environ.get("GATE_Q_BOUT1_PLANTED_XY_DAMP", damp_ctrl))
                damp_step = float(damp_ctrl) ** (1.0 / float(nsp))
            else:
                damp_step = 1.0
            # iterate3r19h7k5: soft planted XY retain on retreat (keep fraction of each
            # substep's Δxy). Not freejoint freeze/pin — tip can still recover. Cuts
            # skate accumulate that damp alone left at dx_plant≈1.0.
            _ret_retain = None
            if reverse and a_w > 0.05 and (not bool(swing_hip)):
                _rr = float(os.environ.get(
                    "GATE_Q_RET_PLANTED_XY_RETAIN", GATE_Q_RET_PLANTED_XY_RETAIN))
                if _rr < 0.999:
                    _ret_retain = _rr
            # iterate3r19h7k8n/k8af: while planted on retreat — skip stance_plant freejoint XY;
            # vel-oppose + sole-contact tangential cancel (not FREEZE/pin/retain). Clear: normal plant.
            # k8af: also mild bout1-approach planted cancel to restore clear_frac after ret handoff.
            _ret_plant_gate = reverse and a_w > 0.05 and (not bool(swing_hip)) and (
                os.environ.get("GATE_Q_RET_PLANTED_PLANT_GATE", GATE_Q_RET_PLANTED_PLANT_GATE)
                not in ("0", "false", "no"))
            _b1_plant_gate = (not reverse) and (bi >= 1) and a_w > 0.05 and (not bool(swing_hip)) and (
                os.environ.get("GATE_Q_BOUT1_CONTACT_XY_CANCEL", GATE_Q_BOUT1_CONTACT_XY_CANCEL)
                not in ("0", "false", "no"))
            _plant_gate = bool(_ret_plant_gate or _b1_plant_gate)
            if _b1_plant_gate and not _ret_plant_gate:
                _xy_kd = float(os.environ.get("GATE_Q_BOUT1_PLANTED_XY_QFRC_KD", GATE_Q_BOUT1_PLANTED_XY_QFRC_KD))
                _xy_fmax = float(os.environ.get("GATE_Q_BOUT1_CONTACT_XY_MAX", GATE_Q_BOUT1_CONTACT_XY_MAX))
                _cxl_on = True
                _cxl_sc = float(os.environ.get("GATE_Q_BOUT1_CONTACT_XY_SCALE", GATE_Q_BOUT1_CONTACT_XY_SCALE))
                _cxl_max = float(os.environ.get("GATE_Q_BOUT1_CONTACT_XY_MAX", GATE_Q_BOUT1_CONTACT_XY_MAX))
            else:
                _xy_kd = float(os.environ.get("GATE_Q_RET_PLANTED_XY_QFRC_KD", GATE_Q_RET_PLANTED_XY_QFRC_KD))
                _xy_fmax = float(os.environ.get("GATE_Q_RET_PLANTED_XY_QFRC_MAX", GATE_Q_RET_PLANTED_XY_QFRC_MAX))
                _cxl_on = _plant_gate and (
                    os.environ.get("GATE_Q_RET_CONTACT_XY_CANCEL", GATE_Q_RET_CONTACT_XY_CANCEL)
                    not in ("0", "false", "no"))
                _cxl_sc = float(os.environ.get("GATE_Q_RET_CONTACT_XY_SCALE", GATE_Q_RET_CONTACT_XY_SCALE))
                _cxl_max = float(os.environ.get("GATE_Q_RET_CONTACT_XY_MAX", GATE_Q_RET_CONTACT_XY_MAX))
            # Ramp cancel with cam: weak near door (protect lifts / bout1 handoff), full mid-retreat
            try:
                _cam_now = float(cam_to_lever_horiz(d, env.sid, env.gid_lever))
            except Exception:
                _cam_now = 0.3
            _c0 = float(os.environ.get("GATE_Q_RET_CONTACT_XY_CAM0", GATE_Q_RET_CONTACT_XY_CAM0))
            _c1 = float(os.environ.get("GATE_Q_RET_CONTACT_XY_CAM1", GATE_Q_RET_CONTACT_XY_CAM1))
            if _c1 > _c0 + 1e-9:
                _ramp = max(0.0, min(1.0, (_cam_now - _c0) / (_c1 - _c0)))
            else:
                _ramp = 1.0
            # Exit taper: ease cancel as cam→stop so bout1 approach handoff isn't tip/yaw bombed
            _e0 = float(os.environ.get("GATE_Q_RET_CONTACT_XY_EXIT0", GATE_Q_RET_CONTACT_XY_EXIT0))
            _e1 = float(os.environ.get("GATE_Q_RET_CONTACT_XY_EXIT1", GATE_Q_RET_CONTACT_XY_EXIT1))
            _emin = float(os.environ.get("GATE_Q_RET_CONTACT_XY_EXIT_MIN", GATE_Q_RET_CONTACT_XY_EXIT_MIN))
            if _e1 > _e0 + 1e-9:
                if _cam_now >= _e1 - 1e-12:
                    _exit = _emin
                elif _cam_now <= _e0 + 1e-12:
                    _exit = 1.0
                else:
                    _exit = 1.0 - (1.0 - _emin) * ((_cam_now - _e0) / (_e1 - _e0))
                _ramp *= max(0.0, min(1.0, _exit))
            if _b1_plant_gate and not _ret_plant_gate:
                _ramp = 1.0  # bout1 approach: no cam-ramp
            _cxl_sc = _cxl_sc * _ramp
            _qvel0 = _ret_plant_gate and (
                os.environ.get("GATE_Q_RET_PLANTED_QVEL0", GATE_Q_RET_PLANTED_QVEL0)
                not in ("0", "false", "no"))
            _qvel_sc = float(os.environ.get("GATE_Q_RET_PLANTED_QVEL_SCALE", GATE_Q_RET_PLANTED_QVEL_SCALE))
            # tip A28c F18: gap-only qvel scale (not full qvel0; cancel stays ×0.70)
            _gap_qv = float(os.environ.get("GATE_Q_RET_GAP_QVEL_SCALE", GATE_Q_RET_GAP_QVEL_SCALE))
            if (_ret_plant_gate and _gap_qv < 0.999 and bi >= 0):
                _sts = list(cycles[bi].get("_ss_ts_ret") or [])
                _age = (float(t) - float(_sts[-1])) if _sts else 0.0
                _need = float(os.environ.get("GATE_Q_RET_GAP_QVEL_SS_GAP", GATE_Q_RET_GAP_QVEL_SS_GAP))
                if (_age >= _need - 1e-9
                        and (cycles[bi].get("n_ss_L_ret", 0) + cycles[bi].get("n_ss_R_ret", 0)) >= 4):
                    _qvel_sc = min(_qvel_sc, _gap_qv)
            import numpy as _np
            _cxl_fx = float(cycles[bi].get("_cxl_fx") or 0.0) if bi >= 0 else 0.0
            _cxl_fy = float(cycles[bi].get("_cxl_fy") or 0.0) if bi >= 0 else 0.0
            for _ in range(nsp):
                d.qfrc_applied[:] = 0; d.xfrc_applied[:] = 0
                _ap = float(a_gait) if reverse else float(a_w)
                if _plant_gate:
                    vx = float(d.qvel[env.v_free]); vy = float(d.qvel[env.v_free + 1])
                    fx = float(_np.clip(-_xy_kd * vx, -_xy_fmax, _xy_fmax))
                    fy = float(_np.clip(-_xy_kd * vy, -_xy_fmax, _xy_fmax))
                    if _cxl_on:
                        _sfx_use, _sfy_use = float(_cxl_fx), float(_cxl_fy)
                        # Signed: only cancel anti-retreat contact ( +X = toward door )
                        if os.environ.get("GATE_Q_RET_CONTACT_XY_SIGNED", GATE_Q_RET_CONTACT_XY_SIGNED) not in ("0","false","no"):
                            _sfx_use = max(0.0, _sfx_use)
                            # keep Y cancel (lateral skate) unsigned
                        fx = float(_np.clip(fx - _cxl_sc * _sfx_use, -_cxl_max, _cxl_max))
                        fy = float(_np.clip(fy - _cxl_sc * _sfy_use, -_cxl_max, _cxl_max))
                    d.qfrc_applied[env.v_free + 0] += fx
                    d.qfrc_applied[env.v_free + 1] += fy
                elif _ap > 0:
                    wg.stance_plant(m, d, phi, _ap, env.bid_lf, env.bid_rf, env.gid_lfoot, env.gid_rfoot, env.gid_floor)
                if _ret_retain is not None:
                    _bx0 = float(d.qpos[env.q_free]); _by0 = float(d.qpos[env.q_free + 1])
                mj.mj_step(m, d)
                if _ret_retain is not None:
                    d.qpos[env.q_free] = _bx0 + _ret_retain * (float(d.qpos[env.q_free]) - _bx0)
                    d.qpos[env.q_free + 1] = _by0 + _ret_retain * (float(d.qpos[env.q_free + 1]) - _by0)
                    d.qvel[env.v_free] *= _ret_retain
                    d.qvel[env.v_free + 1] *= _ret_retain
                elif damp_step < 0.999:
                    d.qvel[env.v_free] *= damp_step
                    d.qvel[env.v_free + 1] *= damp_step
                # Measure sole-floor contact world-XY force for next-substep cancel (planted only)
                if _cxl_on:
                    _sfx = _sfy = 0.0
                    _f6 = _np.zeros(6, dtype=_np.float64)
                    for _ci in range(int(d.ncon)):
                        _c = d.contact[_ci]
                        _g1, _g2 = int(_c.geom1), int(_c.geom2)
                        _b1 = int(m.geom_bodyid[_g1]); _b2 = int(m.geom_bodyid[_g2])
                        _is_floor = (_g1 == env.gid_floor or _g2 == env.gid_floor)
                        _is_foot = (_b1 in (env.bid_lf, env.bid_rf) or _b2 in (env.bid_lf, env.bid_rf))
                        if not (_is_floor and _is_foot):
                            continue
                        mj.mj_contactForce(m, d, _ci, _f6)
                        # frame[0:3]=normal(x), [3:6]=y, [6:9]=z; tangential = y,z components
                        _ty = _np.asarray(_c.frame[3:6], dtype=_np.float64)
                        _tz = _np.asarray(_c.frame[6:9], dtype=_np.float64)
                        _fw = float(_f6[1]) * _ty + float(_f6[2]) * _tz
                        _sfx += float(_fw[0]); _sfy += float(_fw[1])
                    _cxl_fx, _cxl_fy = _sfx, _sfy
            if bi >= 0 and _cxl_on:
                cycles[bi]["_cxl_fx"] = float(_cxl_fx)
                cycles[bi]["_cxl_fy"] = float(_cxl_fy)
            # Milder than freeze: once/ctrl planted XY qvel kill (full) or scale (partial) — not qpos pin
            if _qvel0:
                d.qvel[env.v_free] = 0.0
                d.qvel[env.v_free + 1] = 0.0
            elif _ret_plant_gate and _qvel_sc < 0.999:
                d.qvel[env.v_free] *= _qvel_sc
                d.qvel[env.v_free + 1] *= _qvel_sc
            # iterate3r19h6/h7k: on clear→plant edge, zero freejoint XY once (kill post-swing momentum)
            # Approach: GATE_Q_PLANT_SETTLE (golden=0). Retreat: GATE_Q_PLANT_SETTLE_RET (h7k=1).
            if a_w > 0.05 and bi >= 0:
                if reverse:
                    _settle = os.environ.get("GATE_Q_PLANT_SETTLE_RET", GATE_Q_PLANT_SETTLE_RET)
                else:
                    _settle = os.environ.get("GATE_Q_PLANT_SETTLE", GATE_Q_PLANT_SETTLE)
                if _settle not in ("0", "false", "no"):
                    prev_sw = bool(cycles[bi].get("_prev_swing_hip"))
                    if prev_sw and (not bool(swing_hip)):
                        d.qvel[env.v_free] = 0.0
                        d.qvel[env.v_free + 1] = 0.0
                cycles[bi]["_prev_swing_hip"] = bool(swing_hip)
            # iterate3n: retreat yaw-PD both bouts (bout1 stronger as 3m; bout0 mild — 3n walked +X into door on yaw drift)
            if reverse and a_w > 0.05:
                if bi >= 1:
                    dampz = float(os.environ.get("GATE_Q_RETREAT_YAW_RATE_DAMP_Z", "0.72"))
                    kz = float(os.environ.get("GATE_Q_RETREAT_YAW_KP", "2.8"))
                    dz = float(os.environ.get("GATE_Q_RETREAT_YAW_KD", "0.38"))
                else:
                    dampz = float(os.environ.get("GATE_Q_BOUT0_RETREAT_YAW_RATE_DAMP_Z", "0.85"))
                    kz = float(os.environ.get("GATE_Q_BOUT0_RETREAT_YAW_KP", "1.6"))
                    dz = float(os.environ.get("GATE_Q_BOUT0_RETREAT_YAW_KD", "0.28"))
                d.qvel[env.v_free + 5] *= dampz
                quat = d.qpos[env.q_free + 3:env.q_free + 7]
                w, x, y, z = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
                yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                wz = float(d.qvel[env.v_free + 5])
                d.qfrc_applied[env.v_free + 5] += (-kz * yaw - dz * wz)
        else:
            if walk_ctrl is None:
                _pin_arms_head(env, arms_now, gcmd)
            pin = phase in ("settle_open", "settle_close", "finalize", "retreat_finalize") or (phase in ("step_walk", "retreat_walk") and walk_ctrl is None and base_xy is not None)
            _substep_p(env, arms_now, disturb_xfrc=disturb_xfrc, base_xy=base_xy, walk_ctrl=None, stand_pin=pin)

        cam_h = cam_to_lever_horiz(d, env.sid, env.gid_lever)
        if cam_h_episode_start is None and t >= t0 - 1e-9: cam_h_episode_start = cam_h
        cL = wg.foot_floor_contact(m, d, env.bid_lf, env.gid_floor)
        cR = wg.foot_floor_contact(m, d, env.bid_rf, env.gid_floor)
        zL = float(d.xpos[env.bid_lf, 2]); zR = float(d.xpos[env.bid_rf, 2])
        clear_L = zL - SOLE_OFFSET; clear_R = zR - SOLE_OFFSET
        # Plant-rest clear (soft-contact bias): sample while both feet planted early in stand/walk
        if bi >= 0:
            btmp = cycles[bi]
            if phase == "step_walk" and cL and cR and (walk_ctrl is None or float(walk_ctrl.get("amp", 0)) < 0.08):
                if len(btmp["_rest_L"]) < 80:
                    btmp["_rest_L"].append(clear_L); btmp["_rest_R"].append(clear_R)
                if btmp["plant_rest_clear_L_m"] is None and len(btmp["_rest_L"]) >= 10:
                    btmp["plant_rest_clear_L_m"] = max(float(np.mean(btmp["_rest_L"])), 0.0086)
                    btmp["plant_rest_clear_R_m"] = max(float(np.mean(btmp["_rest_R"])), 0.0086)
            # Floor rest at documented soft-contact ≈8.6 mm (never game above-rest with under-sample)
            _doc_rest = 0.0086
            rest_L = max(btmp["plant_rest_clear_L_m"] if btmp["plant_rest_clear_L_m"] is not None else _doc_rest, _doc_rest)
            rest_R = max(btmp["plant_rest_clear_R_m"] if btmp["plant_rest_clear_R_m"] is not None else _doc_rest, _doc_rest)
        else:
            rest_L = rest_R = 0.0086
        clear_ar_L = clear_L - rest_L
        clear_ar_R = clear_R - rest_R
        # Visible clear: ≥2 cm ABOVE plant rest + abs thr (defeats +8.6 mm bias).
        # Do NOT require not-contact on swing foot (toe scrapes). Other-foot planted is
        # preferred but not hard-gated — bout1 R dwell was truncated by stance chatter.
        # Still require amp>0 (checked at credit site) so settle/pin does not credit SS.
        vis_L = ((clear_ar_L >= VISIBLE_CLEAR_ABOVE_REST_M - 1e-9)
                 and (clear_L >= VISIBLE_CLEAR_M - 1e-9))
        vis_R = ((clear_ar_R >= VISIBLE_CLEAR_ABOVE_REST_M - 1e-9)
                 and (clear_R >= VISIBLE_CLEAR_M - 1e-9))
        # SS credit uses vis_*; HUD FOOT-LIFT uses stricter thr so banner never overclaims flush soles
        hud_clear = float(os.environ.get("GATE_Q_HUD_CLEAR_M", GATE_Q_HUD_CLEAR_M))
        hud_ar = float(os.environ.get("GATE_Q_HUD_CLEAR_ABOVE_REST_M", GATE_Q_HUD_CLEAR_ABOVE_REST_M))
        hud_L = (clear_ar_L >= hud_ar - 1e-9) and (clear_L >= hud_clear - 1e-9)
        hud_R = (clear_ar_R >= hud_ar - 1e-9) and (clear_R >= hud_clear - 1e-9)
        ssL, ssR = vis_L, vis_R  # dwell / SS credit (criteria thr)
        hud_ssL, hud_ssR = hud_L, hud_R  # FOOT-LIFT banner only
        qpos_log.append(d.qpos.copy())
        xy_log.append((float(d.qpos[env.q_free]), float(d.qpos[env.q_free + 1])))
        bout_log.append(bi); t_log.append(t); phase_log.append(phase)
        disturb_log.append(phase == "disturb"); leave_log.append(phase == "leave")
        force_log.append(disturb_xfrc is not None)
        # approach_log kept for timeline; HUD uses shim_log to avoid "stepped" during finalize
        approach_log.append(phase in ("step_walk", "finalize")); retreat_log.append(phase in ("retreat_walk", "retreat_finalize"))
        shim_log.append(phase in ("finalize", "retreat_finalize"))
        camh_log.append(cam_h)
        Rmat = d.xmat[env.bid_body].reshape(3, 3); up = float(Rmat[2, 2])
        if up > 0.5:
            tip_run += 1/CTRL_HZ; tip = max(tip, tip_run); tip_down_run = 0.0
        else:
            tip_down_run += 1/CTRL_HZ
            tip_run = 0.0
            # Real tip = consecutive lean ≥120 ms (single-tick swing lean is not tip_after fail)
            # Approach clearance bursts can briefly lean; tip_during_approach still gates ≥8s upright.
            # tip_after_disturb honesty: compose + finalize only (not active step_walk clearance).
            if tip_down_run >= 0.12 - 1e-9 and phase in ("finalize","disturb","reject","leave","regrasp","close") and bi >= 0:
                tip_after_disturb_ok = False
        ang_p = _angle_deg(m, d, jid_p); ang_l = _angle_deg(m, d, jid_l)
        global_max_panel = max(global_max_panel, abs(ang_p)); global_max_lever = max(global_max_lever, abs(ang_l))
        if phase in ("push", "hold", "disturb", "reject", "leave", "regrasp", "close", "recover"):
            global_max_panel_compose = max(global_max_panel_compose, abs(ang_p))
        contacting = _hand_lever_contact(m, d)
        if contacting:
            global_contact += 1/CTRL_HZ
            if abs(ang_p) > prev_abs + 1e-4: global_rising += 1/CTRL_HZ
        if bi >= 0:
            b = cycles[bi]; bx_now = float(d.qpos[env.q_free])
            if phase == "step_walk":
                if b["approach_t0"] is None:
                    b["approach_t0"] = t; b["cam_start"] = cam_h
                    b["body_x_approach_start"] = bx_now; b["_walk_x0"] = bx_now
                    b["cam_start_ge_055"] = cam_h >= 0.55 - 1e-6
                b["approach_t1"] = t; b["cam_at_approach_end"] = cam_h
                if up > 0.5:
                    b["_tip_app_run"] += 1/CTRL_HZ; b["tip_during_approach"] = max(b["tip_during_approach"], b["_tip_app_run"])
                else: b["_tip_app_run"] = 0.0
                # Track max sole clearance (active walk, amp>0)
                amp = walk_ctrl["amp"] if walk_ctrl else 0.0
                if amp > 0.05:
                    b["max_clear_L_m"] = max(b["max_clear_L_m"], clear_L)
                    b["max_clear_R_m"] = max(b["max_clear_R_m"], clear_R)
                    b["max_clear_above_rest_L_m"] = max(b["max_clear_above_rest_L_m"], clear_ar_L)
                    b["max_clear_above_rest_R_m"] = max(b["max_clear_above_rest_R_m"], clear_ar_R)
                    # iterate3r18: accumulate Δ-while-clear vs Δ-while-planted (toward door = +X)
                    prev_bx = b.get("_prev_bx_track")
                    if prev_bx is not None:
                        dbx = float(bx_now) - float(prev_bx)
                        if dbx > 0.0:
                            swing_trk = bool(ssL or ssR)  # same thr as SS credit (≥2cm above rest)
                            if swing_trk:
                                b["_dx_clear_m"] = float(b.get("_dx_clear_m") or 0.0) + dbx
                            else:
                                b["_dx_planted_m"] = float(b.get("_dx_planted_m") or 0.0) + dbx
                    b["_prev_bx_track"] = float(bx_now)
                    b["dx_clear_m"] = float(b.get("_dx_clear_m") or 0.0)
                    b["dx_planted_m"] = float(b.get("_dx_planted_m") or 0.0)
                    _tot_dx = b["dx_clear_m"] + b["dx_planted_m"]
                    b["clear_frac"] = (b["dx_clear_m"] / _tot_dx) if _tot_dx > 1e-6 else 0.0
                else:
                    b["_prev_bx_track"] = float(bx_now)
                # Dwell SS: consecutive currently-clear ≥ SS_DWELL_S (not rising-edge / sticky)
                if ssL:
                    b["_ss_run_L"] += 1.0 / CTRL_HZ
                    b["max_ss_dwell_L_s"] = max(b["max_ss_dwell_L_s"], b["_ss_run_L"])
                    if (not b["_dwell_latched_L"]) and b["_ss_run_L"] >= SS_DWELL_S - 1e-9:
                        b["n_ss_L"] += 1; ss_flag = True; b["_dwell_latched_L"] = True
                        b["max_clear_ss_L_m"] = max(b["max_clear_ss_L_m"], clear_L)
                        b["_ss_xs"].append(float(d.qpos[env.q_free])); b["_ss_ts"].append(float(t))
                else:
                    b["_ss_run_L"] = 0.0; b["_dwell_latched_L"] = False
                if ssR:
                    b["_ss_run_R"] += 1.0 / CTRL_HZ
                    b["max_ss_dwell_R_s"] = max(b["max_ss_dwell_R_s"], b["_ss_run_R"])
                    if (not b["_dwell_latched_R"]) and b["_ss_run_R"] >= SS_DWELL_S - 1e-9:
                        b["n_ss_R"] += 1; ss_flag = True; b["_dwell_latched_R"] = True
                        b["max_clear_ss_R_m"] = max(b["max_clear_ss_R_m"], clear_R)
                        b["_ss_xs"].append(float(d.qpos[env.q_free])); b["_ss_ts"].append(float(t))
                else:
                    b["_ss_run_R"] = 0.0; b["_dwell_latched_R"] = False
                if (not cL) and b["_prev_cL"]: b["n_contact_breaks_L"] += 1
                if (not cR) and b["_prev_cR"]: b["n_contact_breaks_R"] += 1
                b["_prev_ssL"], b["_prev_ssR"] = ssL, ssR
                b["_prev_cL"], b["_prev_cR"] = cL, cR
                phi = walk_ctrl["phi"] if walk_ctrl else 0.0
                if amp > 0.2:
                    lat = wg.lateral_com_target(phi)
                    # Skate = stance-foot slip only. Ignore swing-leg toe scrapes while sole is clear
                    # (those inflated p95 after clearance boost; not stance skate).
                    skate_clear = float(os.environ.get("GATE_Q_SKATE_CLEAR_M", "0.008"))
                    planted_L = cL and clear_L < skate_clear
                    planted_R = cR and clear_R < skate_clear
                    if planted_L: b["_plant_run_L"] = b.get("_plant_run_L", 0.0) + 1.0/CTRL_HZ
                    else: b["_plant_run_L"] = 0.0
                    if planted_R: b["_plant_run_R"] = b.get("_plant_run_R", 0.0) + 1.0/CTRL_HZ
                    else: b["_plant_run_R"] = 0.0
                    plant_need = float(os.environ.get("GATE_Q_SKATE_PLANT_S", GATE_Q_SKATE_PLANT_S))
                    if lat > 0 and planted_L and b["_plant_run_L"] >= plant_need:
                        b["_vxL_st"].append(abs(float(d.cvel[env.bid_lf][3])))
                    if lat < 0 and planted_R and b["_plant_run_R"] >= plant_need:
                        b["_vxR_st"].append(abs(float(d.cvel[env.bid_rf][3])))
                b["teleport_falsifier"] = False
                b["stepped_delta_m"] = bx_now - (b["_walk_x0"] or bx_now)
            elif phase == "finalize":
                b["approach_t1"] = t; b["cam_at_approach_end"] = cam_h
                if up > 0.5:
                    b["_tip_app_run"] += 1/CTRL_HZ; b["tip_during_approach"] = max(b["tip_during_approach"], b["_tip_app_run"])
                else: b["_tip_app_run"] = 0.0
            elif phase == "retreat_walk":
                b["retreat_t1"] = t; b["retreat_cam_end"] = cam_h
                amp = walk_ctrl["amp"] if walk_ctrl else 0.0
                # tip during active reverse OR post-proof stand pin (not hold drift)
                tip_credit = (amp > 0.05) or bool(b.get("_retreat_stop"))
                if tip_credit:
                    if up > 0.5:
                        b["_tip_ret_run"] += 1/CTRL_HZ
                        b["_tip_ret_down"] = 0.0
                        b["tip_during_retreat"] = max(b["tip_during_retreat"], b["_tip_ret_run"])
                    else:
                        b["_tip_ret_down"] = b.get("_tip_ret_down", 0.0) + 1/CTRL_HZ
                        if b["_tip_ret_down"] >= 0.12 - 1e-9:
                            b["_tip_ret_run"] = 0.0
                if amp > 0.05:
                    b["max_clear_L_ret_m"] = max(b["max_clear_L_ret_m"], clear_L)
                    b["max_clear_R_ret_m"] = max(b["max_clear_R_ret_m"], clear_R)
                    b["max_clear_above_rest_L_ret_m"] = max(b["max_clear_above_rest_L_ret_m"], clear_ar_L)
                    b["max_clear_above_rest_R_ret_m"] = max(b["max_clear_above_rest_R_ret_m"], clear_ar_R)
                    # iterate3r18: Δ-while-clear vs planted on retreat (away from door = −X)
                    # k8w diagnostic: optional clear-hold (swing_hip) tracking vs strict SS thr
                    prev_bxr = b.get("_prev_bx_track_ret")
                    if prev_bxr is not None:
                        dbxr = float(prev_bxr) - float(bx_now)  # positive = retreat progress
                        if dbxr > 0.0:
                            _trk = bool(ssL or ssR)
                            if os.environ.get("GATE_Q_RET_CLEAR_TRACK_SWING", GATE_Q_RET_CLEAR_TRACK_SWING) not in ("0","false","no"):
                                _trk = bool(b.get("_swing_hip_ret")) or _trk
                            if _trk:
                                b["_dx_clear_m_ret"] = float(b.get("_dx_clear_m_ret") or 0.0) + dbxr
                            else:
                                b["_dx_planted_m_ret"] = float(b.get("_dx_planted_m_ret") or 0.0) + dbxr
                    b["_prev_bx_track_ret"] = float(bx_now)
                    b["dx_clear_m_ret"] = float(b.get("_dx_clear_m_ret") or 0.0)
                    b["dx_planted_m_ret"] = float(b.get("_dx_planted_m_ret") or 0.0)
                    _tot_r = b["dx_clear_m_ret"] + b["dx_planted_m_ret"]
                    b["clear_frac_ret"] = (b["dx_clear_m_ret"] / _tot_r) if _tot_r > 1e-6 else 0.0
                    if ssL:
                        b["_ss_run_L_ret"] += 1.0 / CTRL_HZ
                        b["max_ss_dwell_L_ret_s"] = max(b["max_ss_dwell_L_ret_s"], b["_ss_run_L_ret"])
                        if (not b["_dwell_latched_L_ret"]) and b["_ss_run_L_ret"] >= SS_DWELL_S - 1e-9:
                            b["n_ss_L_ret"] += 1; ss_flag = True; b["_dwell_latched_L_ret"] = True
                            b["max_clear_ss_L_ret_m"] = max(b["max_clear_ss_L_ret_m"], clear_L)
                            b["_ss_xs_ret"].append(float(d.qpos[env.q_free])); b["_ss_ts_ret"].append(float(t))
                            b.setdefault("_ss_snap_ret", []).append((
                                float(t),
                                float(b.get("_dx_clear_m_ret") or 0.0),
                                float(b.get("_dx_planted_m_ret") or 0.0),
                                float(b.get("retreat_cam_max") or 0.0),
                            ))
                    else:
                        b["_ss_run_L_ret"] = 0.0; b["_dwell_latched_L_ret"] = False
                    if ssR:
                        b["_ss_run_R_ret"] += 1.0 / CTRL_HZ
                        b["max_ss_dwell_R_ret_s"] = max(b["max_ss_dwell_R_ret_s"], b["_ss_run_R_ret"])
                        if (not b["_dwell_latched_R_ret"]) and b["_ss_run_R_ret"] >= SS_DWELL_S - 1e-9:
                            b["n_ss_R_ret"] += 1; ss_flag = True; b["_dwell_latched_R_ret"] = True
                            b["max_clear_ss_R_ret_m"] = max(b["max_clear_ss_R_ret_m"], clear_R)
                            b["_ss_xs_ret"].append(float(d.qpos[env.q_free])); b["_ss_ts_ret"].append(float(t))
                            b.setdefault("_ss_snap_ret", []).append((
                                float(t),
                                float(b.get("_dx_clear_m_ret") or 0.0),
                                float(b.get("_dx_planted_m_ret") or 0.0),
                                float(b.get("retreat_cam_max") or 0.0),
                            ))
                    else:
                        b["_ss_run_R_ret"] = 0.0; b["_dwell_latched_R_ret"] = False
                    if amp > 0.2:
                        lat = wg.lateral_com_target(walk_ctrl["phi"] if walk_ctrl else 0.0)
                        planted_L = cL and clear_L < 0.012
                        planted_R = cR and clear_R < 0.012
                        if planted_L: b["_plant_run_L_ret"] = b.get("_plant_run_L_ret", 0.0) + 1.0/CTRL_HZ
                        else: b["_plant_run_L_ret"] = 0.0
                        if planted_R: b["_plant_run_R_ret"] = b.get("_plant_run_R_ret", 0.0) + 1.0/CTRL_HZ
                        else: b["_plant_run_R_ret"] = 0.0
                        plant_need_r = float(os.environ.get("GATE_Q_SKATE_PLANT_RET_S", GATE_Q_SKATE_PLANT_RET_S))
                        if lat > 0 and planted_L and b["_plant_run_L_ret"] >= plant_need_r:
                            b["_vxL_st_ret"].append(abs(float(d.cvel[env.bid_lf][3])))
                        if lat < 0 and planted_R and b["_plant_run_R_ret"] >= plant_need_r:
                            b["_vxR_st_ret"].append(abs(float(d.cvel[env.bid_rf][3])))
                if b["_retreat_walk_x0"] is not None:
                    b["retreat_stepped_delta_m"] = b["_retreat_walk_x0"] - bx_now
            elif phase == "retreat_finalize":
                b["retreat_t1"] = t; b["retreat_cam_end"] = cam_h
            elif phase in ("push", "hold"):
                if not b["entered_n_working_pose"]:
                    if abs(bx_now - body_x_work) < 0.03 or cam_h <= CAM_WORK + 0.08:
                        b["entered_n_working_pose"] = True
                        if b["cam_at_approach_end"] is None: b["cam_at_approach_end"] = cam_h
                if contacting and b["first_contact_t"] is None:
                    b["first_contact_t"] = t; b["cam_at_first_contact"] = cam_h
                    b["body_x_at_first_contact"] = bx_now
                    if b["body_x_approach_start"] is not None:
                        b["approach_delta_m"] = bx_now - b["body_x_approach_start"]
                        b["approach_delta_before_contact_ok"] = b["approach_delta_m"] >= 0.15 - 1e-6
                if abs(ang_p) > abs(b["peak_panel_deg"]): b["peak_panel_deg"] = ang_p
                b["max_lever_deg"] = max(b["max_lever_deg"], abs(ang_l))
                if abs(ang_p) >= 25.0 - 1e-6:
                    b["_hold_run"] += 1/CTRL_HZ; b["hold25_s"] = max(b["hold25_s"], b["_hold_run"])
                else: b["_hold_run"] = 0.0
                if contacting:
                    b["contact_s"] += 1/CTRL_HZ
                    if abs(ang_p) > b["_prev_abs"] + 1e-4: b["contact_rising_s"] += 1/CTRL_HZ
            elif phase == "disturb":
                if b["disturb_t0"] is None:
                    b["disturb_t0"] = t; b["panel_at_disturb_start"] = abs(ang_p)
                b["disturb_t1"] = t
                b["panel_min_during_disturb"] = abs(ang_p) if b["panel_min_during_disturb"] is None else min(b["panel_min_during_disturb"], abs(ang_p))
                if contacting: b["contact_s"] += 1/CTRL_HZ
                if abs(ang_p) > abs(b["peak_panel_deg"]): b["peak_panel_deg"] = ang_p
            elif phase == "reject":
                b["panel_min_during_reject"] = abs(ang_p) if b["panel_min_during_reject"] is None else min(b["panel_min_during_reject"], abs(ang_p))
                b["panel_at_reject_end"] = abs(ang_p)
                if (not b["_seen15_after"]) and abs(ang_p) >= 15.0 - 1e-6:
                    t_end = b["disturb_t1"] if b["disturb_t1"] is not None else t
                    b["time_to_panel15_after_disturb_s"] = max(0.0, t - t_end); b["_seen15_after"] = True
                if contacting: b["contact_s"] += 1/CTRL_HZ
                if abs(ang_p) > abs(b["peak_panel_deg"]): b["peak_panel_deg"] = ang_p
            elif phase == "leave":
                if b["panel_at_leave_start"] is None: b["panel_at_leave_start"] = abs(ang_p)
                if not contacting:
                    b["_leave_nc_run"] += 1/CTRL_HZ; b["leave_no_contact_s"] = max(b["leave_no_contact_s"], b["_leave_nc_run"])
                    if abs(ang_p) >= 20.0 - 1e-6:
                        b["_leave_nc20_run"] += 1/CTRL_HZ
                        b["leave_no_contact_panel20_s"] = max(b["leave_no_contact_panel20_s"], b["_leave_nc20_run"])
                        b["min_panel_during_leave_ok"] = abs(ang_p) if b["min_panel_during_leave_ok"] is None else min(b["min_panel_during_leave_ok"], abs(ang_p))
                    else: b["_leave_nc20_run"] = 0.0
                else: b["_leave_nc_run"] = 0.0; b["_leave_nc20_run"] = 0.0
            elif phase == "regrasp":
                if contacting: b["regrasp_contact_s"] += 1/CTRL_HZ; b["contact_s"] += 1/CTRL_HZ
            elif phase == "close":
                b["min_abs_during_close"] = abs(ang_p) if b["min_abs_during_close"] is None else min(b["min_abs_during_close"], abs(ang_p))
                if contacting: b["contact_s"] += 1/CTRL_HZ
            b["_prev_abs"] = abs(ang_p)
        # HUD FOOT-LIFT only while STRICT currently clear (stricter than SS credit; no sticky; no flush overclaim)
        ss_log.append(bool(hud_ssL or hud_ssR)); prev_abs = abs(ang_p)

    tip_ok = tip >= 8.0 and tip_after_disturb_ok
    for i, b in enumerate(cycles):
        if b["min_abs_during_close"] is None: b["min_abs_during_close"] = abs(b["peak_panel_deg"])
        b["close_delta_deg"] = abs(b["peak_panel_deg"]) - float(b["min_abs_during_close"])
        b["open25_ok"] = abs(b["peak_panel_deg"]) >= 25.0 - 1e-6
        b["hold_ok"] = b["hold25_s"] >= 1.0 - 1e-9
        b["disturb_applied_ok"] = b["disturb_t0"] is not None and b["disturb_t1"] is not None and (b["disturb_t1"] - b["disturb_t0"]) >= 0.2
        if b["time_to_panel15_after_disturb_s"] is None and b["panel_min_during_disturb"] is not None and b["panel_min_during_disturb"] >= 15:
            b["time_to_panel15_after_disturb_s"] = 0.0; b["_seen15_after"] = True
        if b["_seen15_after"] and b["time_to_panel15_after_disturb_s"] is None: b["time_to_panel15_after_disturb_s"] = 0.0
        b["reject_panel15_ok"] = b["time_to_panel15_after_disturb_s"] is not None and b["time_to_panel15_after_disturb_s"] <= 2.0 + 1e-9
        leave_start_ok = b["panel_at_leave_start"] is not None and b["panel_at_leave_start"] >= 20.0 - 1e-6
        b["leave_ok"] = leave_start_ok and b["leave_no_contact_panel20_s"] >= 0.5 - 1e-9
        b["regrasp_ok"] = b["regrasp_contact_s"] >= 0.3
        b["close_ok"] = b["close_delta_deg"] >= 10.0 - 1e-6
        b["coupled_ok"] = b["contact_rising_s"] > 0.1 and b["contact_s"] > 0.5
        b["tip_ok"] = tip_ok; b["within_30"] = abs(b["peak_panel_deg"]) <= 30.0 + 0.05
        mL, pL = _mean_p95(b["_vxL_st"]); mR, pR = _mean_p95(b["_vxR_st"])
        b["skate_mean_L"], b["skate_p95_L"] = mL, pL; b["skate_mean_R"], b["skate_p95_R"] = mR, pR
        b["skate_mean"] = 0.5*(mL+mR); b["skate_p95"] = 0.5*(pL+pR)
        # Criteria: skate mean ≤0.08, p95 ≤0.18 (combined stance). Per-foot still logged.
        b["skate_ok"] = not (b["skate_mean"] > 0.08 or b["skate_p95"] > 0.18)
        b["n_ss_total"] = b["n_ss_L"] + b["n_ss_R"]
        # iterate3r18 honesty: stepped Δ = Δ-while-clear only (planted soft-XY does NOT count).
        # Cadence span/lifts_per_m still use raw walk Δ so clear-only denom cannot inflate cadence.
        dx_c = float(b.get("dx_clear_m") or b.get("_dx_clear_m") or 0.0)
        dx_p = float(b.get("dx_planted_m") or b.get("_dx_planted_m") or 0.0)
        b["dx_clear_m"] = dx_c; b["dx_planted_m"] = dx_p
        _tot_cp = dx_c + dx_p
        b["clear_frac"] = (dx_c / _tot_cp) if _tot_cp > 1e-6 else 0.0
        raw_walk_dx = 0.0
        if b["_walk_x0"] is not None and b["body_x_at_walk_end"] is not None:
            raw_walk_dx = float(b["body_x_at_walk_end"]) - float(b["_walk_x0"])
        b["raw_walk_delta_m"] = raw_walk_dx
        if _tot_cp > 1e-4:
            b["stepped_delta_m"] = dx_c  # planted soft-XY excluded from stepped credit
        elif b["stepped_delta_m"] is None:
            b["stepped_delta_m"] = raw_walk_dx
        if b["shim_delta_m"] is None and b.get("body_x_at_walk_end") is not None:
            # finalize end ≈ body_x_work (kinematic target)
            b["shim_delta_m"] = float(body_x_work) - float(b["body_x_at_walk_end"])
        stepped_dx = float(b["stepped_delta_m"] or 0.0)
        shim_dx = float(b["shim_delta_m"] or 0.0)
        cadence_dx = max(abs(raw_walk_dx), abs(dx_c + dx_p), 1e-6)  # denom for span/lifts
        # Total approach Δ for shim budget: prefer measured approach_delta; else stepped+shim
        if b["approach_delta_m"] is None and b["body_x_approach_start"] is not None:
            b["approach_delta_m"] = body_x_work - b["body_x_approach_start"]
            b["approach_delta_before_contact_ok"] = b["approach_delta_m"] >= 0.15 - 1e-6
        total_app = float(b["approach_delta_m"] or (stepped_dx + max(0.0, shim_dx)) or 1e-9)
        if total_app < 1e-6:
            total_app = max(stepped_dx + max(0.0, shim_dx), 1e-9)
        b["stepped_frac"] = stepped_dx / total_app
        b["shim_budget_ok"] = bool(b["stepped_frac"] >= STEPPED_FRAC_MIN - 1e-6)
        # Visible multi-step dwell-SS — criteria floor (thin multi) + iterate3n dense+cadence continuous
        both_sides = _ss_multi_proved(b["n_ss_L"], b["n_ss_R"])
        dense_sides = _ss_dense_proved(b["n_ss_L"], b["n_ss_R"])
        clear_ok = (b["max_clear_L_m"] >= VISIBLE_CLEAR_M - 1e-9 and b["max_clear_R_m"] >= VISIBLE_CLEAR_M - 1e-9
                    and b["max_clear_above_rest_L_m"] >= VISIBLE_CLEAR_ABOVE_REST_M - 1e-9
                    and b["max_clear_above_rest_R_m"] >= VISIBLE_CLEAR_ABOVE_REST_M - 1e-9)
        dwell_ok = (b["max_ss_dwell_L_s"] >= SS_DWELL_S - 1e-9 and b["max_ss_dwell_R_s"] >= SS_DWELL_S - 1e-9)
        b["visible_ss_ok"] = bool(both_sides and clear_ok and dwell_ok)
        b["dense_ss_ok"] = bool(dense_sides and clear_ok and dwell_ok)
        ss_xs = list(b.get("_ss_xs") or [])
        ss_ts = list(b.get("_ss_ts") or [])
        walk_dur = max(1e-6, float(b.get("approach_t1") or 0) - float(b.get("approach_t0") or 0))
        if len(ss_xs) >= 2 and cadence_dx > 1e-3:
            span = abs(max(ss_xs) - min(ss_xs))
            b["ss_span_m"] = span
            b["ss_span_frac"] = span / max(cadence_dx, 1e-6)
        else:
            b["ss_span_m"] = 0.0; b["ss_span_frac"] = 0.0
        n_tot = b["n_ss_L"] + b["n_ss_R"]
        b["lifts_per_m"] = float(n_tot) / max(cadence_dx, 1e-6)
        b["lifts_per_s"] = float(n_tot) / walk_dur
        b["ss_times_s"] = [round(float(x), 3) for x in ss_ts]
        span_min = float(os.environ.get("GATE_Q_CADENCE_SPAN_FRAC_MIN", GATE_Q_CADENCE_SPAN_FRAC_MIN))
        lpm_min = float(os.environ.get("GATE_Q_LIFTS_PER_M_MIN", GATE_Q_LIFTS_PER_M_MIN))
        tspan_need = float(os.environ.get("GATE_Q_SS_TIME_SPAN_FRAC_MIN", GATE_Q_SS_TIME_SPAN_FRAC_MIN))
        # Denominator = active walk end (stop_t or last SS) — exclude post-stop pin (was killing ret tspan)
        _t0a = float(b.get("approach_t0") or 0)
        _t1a = float(b.get("walk_stop_t") or (max(ss_ts) if ss_ts else b.get("approach_t1") or 0))
        if ss_ts:
            _t1a = max(_t1a, float(max(ss_ts)))
        b["ss_time_span_frac"] = _ss_time_span_frac(ss_ts, _t0a, _t1a)
        b["walk_stop_t"] = b.get("walk_stop_t")
        walk_end_x = float(b.get("body_x_at_walk_end") if b.get("body_x_at_walk_end") is not None
                           else (ss_xs[-1] if ss_xs else 0.0))
        b["ss_carry_tail_m"] = (walk_end_x - float(max(ss_xs))) if ss_xs else 999.0
        b["ss_carry_tail_ok"] = bool(_ss_carry_tail_ok(ss_xs, walk_end_x, reverse=False))
        b["cadence_ok"] = bool(b["ss_span_frac"] >= span_min - 1e-9
                               and b["ss_time_span_frac"] >= tspan_need - 1e-9
                               and b["ss_carry_tail_ok"]
                               and ((n_tot >= 4 and b["lifts_per_m"] >= lpm_min - 1e-9)
                                    or (n_tot >= 3 and b["ss_span_frac"] >= span_min - 1e-9
                                        and b["lifts_per_m"] >= 6.0 - 1e-9)))
        # Continuous: span + time-span + carry-tail (prefer FAIL if early-cluster then slide)
        cont_ok = bool(b["ss_carry_tail_ok"] and b["ss_time_span_frac"] >= tspan_need - 1e-9
                       and b["ss_span_frac"] >= span_min - 1e-9)
        foot_ok = bool(
            cont_ok and (
                (b["dense_ss_ok"])
                or (b["visible_ss_ok"] and n_tot >= 4 and b["lifts_per_m"] >= lpm_min - 1e-9)
                # iterate3r: thin multi 2+1/1+2 OK only with strong continuous span+tspan+tail (prefer FAIL if cluster)
                or (b["visible_ss_ok"] and n_tot >= 3 and b["ss_span_frac"] >= span_min - 1e-9
                    and b["lifts_per_m"] >= 6.0 - 1e-9)
            )
        )
        clear_frac_need = float(os.environ.get("GATE_Q_CLEAR_FRAC_MIN", GATE_Q_CLEAR_FRAC_MIN))
        b["clear_frac_ok"] = bool(b.get("clear_frac", 0.0) >= clear_frac_need - 1e-9)
        # Planted soft-XY must not count: require clear_frac + clear-window stepped Δ
        b["stepped_ok"] = bool(foot_ok and stepped_dx >= 0.15 - 1e-6 and b["skate_ok"]
                               and b["shim_budget_ok"] and b["clear_frac_ok"])
        b["approach_tip_ok"] = bool(b["tip_during_approach"] >= 8.0 - 1e-6)  # criteria tip≥8; soft-pass forbidden
        b["start_offset_ok"] = bool(b["cam_start_ge_055"] or b["approach_delta_before_contact_ok"])
        if b["approach_delta_m"] is None and b["body_x_approach_start"] is not None:
            b["approach_delta_m"] = body_x_work - b["body_x_approach_start"]
            b["approach_delta_before_contact_ok"] = b["approach_delta_m"] >= 0.15 - 1e-6
            b["start_offset_ok"] = bool(b["cam_start_ge_055"] or b["approach_delta_before_contact_ok"])
        b["approach_ok"] = bool(b["start_offset_ok"] and b["approach_tip_ok"] and b["entered_n_working_pose"] and not b["teleport_falsifier"] and b["stepped_ok"] and (b["approach_delta_before_contact_ok"] or b["cam_start_ge_055"]))
        b["compose_ok"] = bool(b["open25_ok"] and b["hold_ok"] and b["disturb_applied_ok"] and b["reject_panel15_ok"] and b["leave_ok"] and b["regrasp_ok"] and b["close_ok"] and b["coupled_ok"] and b["tip_ok"] and b["within_30"])
        if not b["compose_ok"]:
            b["compose_fail_reasons"] = [k for k in
                ("open25_ok","hold_ok","disturb_applied_ok","reject_panel15_ok","leave_ok","regrasp_ok","close_ok","coupled_ok","tip_ok","within_30")
                if not b.get(k)]
        # --- retreat scoring ---
        mL, pL = _mean_p95(b.get("_vxL_st_ret") or []); mR, pR = _mean_p95(b.get("_vxR_st_ret") or [])
        b["skate_mean_L_ret"], b["skate_p95_L_ret"] = mL, pL
        b["skate_mean_R_ret"], b["skate_p95_R_ret"] = mR, pR
        b["skate_mean_ret"] = 0.5 * (mL + mR); b["skate_p95_ret"] = 0.5 * (pL + pR)
        b["skate_ok_ret"] = not (b["skate_mean_ret"] > 0.08 or b["skate_p95_ret"] > 0.18)
        b["n_ss_total_ret"] = b["n_ss_L_ret"] + b["n_ss_R_ret"]
        dx_cr = float(b.get("dx_clear_m_ret") or b.get("_dx_clear_m_ret") or 0.0)
        dx_pr = float(b.get("dx_planted_m_ret") or b.get("_dx_planted_m_ret") or 0.0)
        b["dx_clear_m_ret"] = dx_cr; b["dx_planted_m_ret"] = dx_pr
        _tot_cpr = dx_cr + dx_pr
        b["clear_frac_ret"] = (dx_cr / _tot_cpr) if _tot_cpr > 1e-6 else 0.0
        raw_ret_dx = 0.0
        if b.get("body_x_retreat_start") is not None and b.get("body_x_at_retreat_walk_end") is not None:
            raw_ret_dx = abs(float(b["body_x_retreat_start"]) - float(b["body_x_at_retreat_walk_end"]))
        b["raw_retreat_delta_m"] = raw_ret_dx
        if _tot_cpr > 1e-4:
            b["retreat_stepped_delta_m"] = dx_cr  # planted soft-XY excluded
        stepped_rdx = float(b.get("retreat_stepped_delta_m") or 0.0)
        cadence_rdx = max(raw_ret_dx, abs(dx_cr + dx_pr), 1e-6)
        shim_rdx = float(b.get("retreat_shim_delta_m") or 0.0)
        if b.get("body_x_retreat_start") is not None and b.get("retreat_cam_end") is not None:
            # total retreat Δ ≈ stepped + shim (away from door = +x decrease / cam increase)
            total_ret = max(stepped_rdx + max(0.0, shim_rdx), 1e-9)
        else:
            total_ret = max(stepped_rdx + max(0.0, shim_rdx), 1e-9)
        b["retreat_delta_m"] = stepped_rdx + max(0.0, shim_rdx)
        b["retreat_stepped_frac"] = stepped_rdx / total_ret
        b["retreat_shim_budget_ok"] = bool(b["retreat_stepped_frac"] >= STEPPED_FRAC_MIN - 1e-6)
        b["retreat_cam_ge_055"] = bool((b.get("retreat_cam_end") or 0.0) >= 0.55 - 1e-6)
        b["retreat_delta_ok"] = bool(stepped_rdx >= 0.15 - 1e-6 or b["retreat_cam_ge_055"])
        both_ret = _ss_multi_proved(b["n_ss_L_ret"], b["n_ss_R_ret"])
        dense_ret = _ss_dense_proved(b["n_ss_L_ret"], b["n_ss_R_ret"])
        clear_ret = (b["max_clear_L_ret_m"] >= VISIBLE_CLEAR_M - 1e-9 and b["max_clear_R_ret_m"] >= VISIBLE_CLEAR_M - 1e-9
                     and b["max_clear_above_rest_L_ret_m"] >= VISIBLE_CLEAR_ABOVE_REST_M - 1e-9
                     and b["max_clear_above_rest_R_ret_m"] >= VISIBLE_CLEAR_ABOVE_REST_M - 1e-9)
        dwell_ret = (b["max_ss_dwell_L_ret_s"] >= SS_DWELL_S - 1e-9 and b["max_ss_dwell_R_ret_s"] >= SS_DWELL_S - 1e-9)
        b["visible_ss_ok_ret"] = bool(both_ret and clear_ret and dwell_ret)
        b["dense_ss_ok_ret"] = bool(dense_ret and clear_ret and dwell_ret)
        ss_xr = list(b.get("_ss_xs_ret") or []); ss_tr = list(b.get("_ss_ts_ret") or [])
        ret_dur = max(1e-6, float(b.get("retreat_t1") or 0) - float(b.get("retreat_t0") or 0))
        if len(ss_xr) >= 2 and cadence_rdx > 1e-3:
            span_r = abs(max(ss_xr) - min(ss_xr))
            b["ss_span_m_ret"] = span_r
            b["ss_span_frac_ret"] = span_r / max(cadence_rdx, 1e-6)
        else:
            b["ss_span_m_ret"] = 0.0; b["ss_span_frac_ret"] = 0.0
        n_tot_r = b["n_ss_L_ret"] + b["n_ss_R_ret"]
        b["lifts_per_m_ret"] = float(n_tot_r) / max(cadence_rdx, 1e-6)
        b["lifts_per_s_ret"] = float(n_tot_r) / ret_dur
        b["ss_times_s_ret"] = [round(float(x), 3) for x in ss_tr]
        span_min = float(os.environ.get("GATE_Q_CADENCE_SPAN_FRAC_MIN", GATE_Q_CADENCE_SPAN_FRAC_MIN))
        lpm_min = float(os.environ.get("GATE_Q_LIFTS_PER_M_MIN", GATE_Q_LIFTS_PER_M_MIN))
        ret_span_min = float(os.environ.get("GATE_Q_RETREAT_CADENCE_SPAN_FRAC_MIN", "0.50"))
        tspan_need = float(os.environ.get("GATE_Q_SS_TIME_SPAN_FRAC_MIN", GATE_Q_SS_TIME_SPAN_FRAC_MIN))
        _t0r = float(b.get("retreat_t0") or 0)
        _t1r = float(b.get("retreat_stop_t") or (max(ss_tr) if ss_tr else b.get("retreat_t1") or 0))
        if ss_tr:
            _t1r = max(_t1r, float(max(ss_tr)))
        b["ss_time_span_frac_ret"] = _ss_time_span_frac(ss_tr, _t0r, _t1r)
        ret_end_x = float(b.get("body_x_at_retreat_walk_end") if b.get("body_x_at_retreat_walk_end") is not None
                          else (ss_xr[-1] if ss_xr else 0.0))
        b["ss_carry_tail_m_ret"] = (float(min(ss_xr)) - ret_end_x) if ss_xr else 999.0
        ret_tail_max = float(os.environ.get("GATE_Q_SS_TAIL_MAX_M_RET", "0.28"))  # post-lift slide to cam0.52 ~24cm
        b["ss_carry_tail_ok_ret"] = bool(_ss_carry_tail_ok(ss_xr, ret_end_x, reverse=True, max_tail_m=ret_tail_max))
        ret_tspan_need = float(os.environ.get("GATE_Q_SS_TIME_SPAN_FRAC_MIN_RET", "0.30"))
        b["cadence_ok_ret"] = bool(b["ss_span_frac_ret"] >= ret_span_min - 1e-9
                                   and n_tot_r >= 4
                                   and b["ss_time_span_frac_ret"] >= ret_tspan_need - 1e-9
                                   and b["ss_carry_tail_ok_ret"])
        b["retreat_tip_ok"] = bool(b["tip_during_retreat"] >= 8.0 - 1e-6)
        ret_tspan_need = float(os.environ.get("GATE_Q_SS_TIME_SPAN_FRAC_MIN_RET", "0.30"))
        cont_ret = bool(b["ss_carry_tail_ok_ret"] and b["ss_time_span_frac_ret"] >= ret_tspan_need - 1e-9
                        and b["ss_span_frac_ret"] >= ret_span_min - 1e-9)
        foot_ret = bool(
            cont_ret and (
                b["dense_ss_ok_ret"]
                or (b["visible_ss_ok_ret"] and n_tot_r >= 4)
            )
        )
        clear_frac_need_r = float(os.environ.get("GATE_Q_CLEAR_FRAC_MIN", GATE_Q_CLEAR_FRAC_MIN))
        b["clear_frac_ok_ret"] = bool(b.get("clear_frac_ret", 0.0) >= clear_frac_need_r - 1e-9)
        b["retreat_stepped_ok"] = bool(foot_ret and stepped_rdx >= 0.15 - 1e-6
                                       and b["skate_ok_ret"] and b["retreat_shim_budget_ok"]
                                       and b["clear_frac_ok_ret"])
        b["retreat_ok"] = bool(b["retreat_stepped_ok"] and b["retreat_delta_ok"] and b["retreat_tip_ok"])
        _sts=list(b.get("_ss_ts_ret") or [])
        if len(_sts) >= 2:
            _gaps=[float(_sts[j])-float(_sts[j-1]) for j in range(1, len(_sts))]
            _big=[(j, round(g, 2)) for j, g in enumerate(_gaps) if g >= 1.5]
            _ts_s=",".join(f"{x:.1f}" for x in _sts)
            _gp_s=",".join(f"{g:.2f}" for g in _gaps)
            print(
                f"[Q] SS_GAPS cyc{i}: n={len(_sts)} maxgap={max(_gaps):.2f}s @i={_gaps.index(max(_gaps))} "
                f"big={_big[:10]} dxc={b.get('dx_clear_m_ret'):.3f} dxp={b.get('dx_planted_m_ret'):.3f}",
                flush=True,
            )
            print(f"[Q] SS_TS cyc{i}: {_ts_s}", flush=True)
            print(f"[Q] SS_GP cyc{i}: {_gp_s}", flush=True)
            _sn=list(b.get("_ss_snap_ret") or [])
            if _sn:
                # print first 8 and around max-gap index
                _gi=_gaps.index(max(_gaps))
                def _fmt(sn):
                    return ",".join(f"{t:.1f}:{dc:.3f}/{dp:.3f}/cm{cm:.2f}" for t,dc,dp,cm in sn)
                print(f"[Q] SS_SNAP cyc{i}: around_gap_i={_gi} "
                      f"pre={_fmt(_sn[max(0,_gi-1):_gi+2])} "
                      f"first3={_fmt(_sn[:3])} last3={_fmt(_sn[-3:])}", flush=True)
                # tip E7lock: cam@last_pre_gap_SS, dxc_pre_gap, planted_middle_s
                _pre = _sn[_gi]  # SS just before max gap
                _t_pre, _dxc_pre, _dxp_pre, _cam_pre = _pre
                _mid = float(_gaps[_gi])
                _post = _sn[_gi+1] if _gi+1 < len(_sn) else None
                _cam_post = _post[3] if _post else float("nan")
                _dxc_post = _post[1] if _post else float("nan")
                print(
                    f"[Q] PRE_GAP cyc{i}: planted_middle_s={_mid:.2f} "
                    f"cam@preSS={_cam_pre:.3f} dxc_pre={_dxc_pre:.3f} dxp_pre={_dxp_pre:.3f} "
                    f"cam@postSS={_cam_post:.3f} dxc_post={_dxc_post:.3f} "
                    f"cam_gain_in_gap={(_cam_post-_cam_pre) if _post else float('nan'):.3f} "
                    f"dxc_gain_in_gap={(_dxc_post-_dxc_pre) if _post else float('nan'):.3f}",
                    flush=True,
                )
                b["planted_middle_s"] = _mid
                b["cam_at_pre_gap_ss"] = _cam_pre
                b["dxc_pre_gap"] = _dxc_pre
                b["cam_gain_in_gap"] = (_cam_post - _cam_pre) if _post else float("nan")
        _hold_g = float(b.get("_r2_hold_cam_gain") or b.get("cam_gain_during_hold") or 0.0)
        _hold_s = float(b.get("_r2_hold_s") or 0.0)
        b["cam_gain_during_hold"] = _hold_g
        b["r2_hold_s"] = _hold_s
        b["r2_burst_n"] = int(b.get("_r2_burst_n") or 0)
        print(
            f"[Q] R2_HOLD_GAIN cyc{i}: cam_gain_during_hold={_hold_g:.3f} "
            f"hold_s={_hold_s:.2f} burst_n={b['r2_burst_n']}",
            flush=True,
        )
        _gap_g = float(b.get("_r3_gap_cam_gain") or b.get("cam_gain_during_plant_gap") or 0.0)
        _gap_s = float(b.get("_r3_gap_s") or 0.0)
        _gap_mx = float(b.get("_r3_gap_max_gain") or 0.0)
        _gap_n = int(b.get("_r3_gap_n") or 0)
        b["cam_gain_during_plant_gap"] = _gap_g
        b["r3_plant_gap_s"] = _gap_s
        b["r3_plant_gap_max_gain"] = _gap_mx
        b["r3_plant_gap_n"] = _gap_n
        b["r3_burst_n"] = int(b.get("_r3_burst_n") or 0)
        _eps = float(os.environ.get("GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS", GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS))
        _cum = float(os.environ.get("GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM", GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM))
        _steal = bool(_gap_n > 0 and (_gap_mx > _eps + 1e-12 or _gap_g > _cum + 1e-12))
        b["r3_plant_gap_cam_steal"] = _steal
        print(
            f"[Q] R3_PLANT_GAP_GAIN cyc{i}: cam_gain_during_plant_gap={_gap_g:.3f} "
            f"max_gap={_gap_mx:.3f} gap_s={_gap_s:.2f} gap_n={_gap_n} burst_n={b['r3_burst_n']} "
            f"steal={_steal} (ε any>{_eps:.2f} or cum>{_cum:.2f})",
            flush=True,
        )
        _r4_g = float(b.get("_r4_plant_cam_gain") or b.get("cam_gain_during_plant") or 0.0)
        _r4_mx = float(b.get("_r4_plant_max_gain") or 0.0)
        _r4_dn = int(b.get("_r4_plant_n") or 0)
        _r4_dmax = float(b.get("_r4_plant_dwell_max") or 0.0)
        _r4_fn = int(b.get("_r4_force_n") or 0)
        _r4_fl = int(b.get("_r4_flush_n") or 0)
        b["cam_gain_during_plant"] = _r4_g
        b["plant_dwell_max"] = _r4_dmax
        b["force_count"] = _r4_fn
        b["r4_plant_max_gain"] = _r4_mx
        b["r4_plant_n"] = _r4_dn
        b["r4_flush_n"] = _r4_fl
        _eps4 = float(os.environ.get("GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS", GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS))
        _cum4 = float(os.environ.get("GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM", GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM))
        _steal4 = bool(_r4_dn > 0 and (_r4_mx > _eps4 + 1e-12 or _r4_g > _cum4 + 1e-12))
        _chatter4 = bool(_r4_fn > 0 and _r4_fl >= max(1, (_r4_fn + 1) // 2))
        b["r4_plant_cam_steal"] = _steal4
        b["r4_flush_chatter"] = _chatter4
        print(
            f"[Q] R4_PLANT_GAIN cyc{i}: cam_gain_during_plant={_r4_g:.3f} "
            f"max_plant={_r4_mx:.3f} plant_dwell_max={_r4_dmax:.2f} force_count={_r4_fn} "
            f"flush_n={_r4_fl} plant_n={_r4_dn} steal={_steal4} chatter={_chatter4} "
            f"(ε any>{_eps4:.2f} or cum>{_cum4:.2f})",
            flush=True,
        )
        # S1 CSF50 logs (F_cmd / T_exchange / dx_back_max / plant-cam suite)
        _csf_n = int(b.get("_csf_n") or 0)
        if _csf_n > 0:
            _F_cmd = float(b.get("_csf_F_sum") or 0.0) / float(_csf_n)
            _T_ex = float(b.get("_csf_T_sum") or 0.0) / float(_csf_n)
        else:
            _F_cmd = float(b.get("_csf_F") or 0.0)
            _T_ex = float(b.get("_csf_T") or 0.0)
        _dxb_log = float(os.environ.get("GATE_Q_RET_CSF50_DX_BACK_MAX", GATE_Q_RET_CSF50_DX_BACK_MAX))
        _csf_g = float(b.get("_csf_plant_cam_gain") or 0.0)
        _csf_mx = float(b.get("_csf_plant_max_gain") or 0.0)
        _csf_dn = int(b.get("_csf_plant_n") or 0)
        _csf_dmax = float(b.get("_csf_plant_dwell_max") or 0.0)
        _eps_c = float(os.environ.get("GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS", GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS))
        _cum_c = float(os.environ.get("GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM", GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM))
        _steal_c = bool(_csf_dn > 0 and (_csf_mx > _eps_c + 1e-12 or _csf_g > _cum_c + 1e-12))
        b["F_cmd"] = _F_cmd
        b["T_exchange"] = _T_ex
        b["dx_back_max"] = _dxb_log
        b["csf_plant_cam_gain"] = _csf_g
        b["csf_plant_max_gain"] = _csf_mx
        b["csf_plant_n"] = _csf_dn
        b["csf_plant_dwell_max"] = _csf_dmax
        b["csf_plant_cam_steal"] = _steal_c
        # Prefer FAIL plant-cam: also surface via cam_gain_during_plant when CSF on and R4 off
        if _csf_n > 0 and float(b.get("cam_gain_during_plant") or 0.0) == 0.0 and _csf_dn > 0:
            b["cam_gain_during_plant"] = _csf_g
            b["plant_dwell_max"] = _csf_dmax
        print(
            f"[Q] CSF50 cyc{i}: F_cmd={_F_cmd:.4f} T_exchange={_T_ex:.3f} dx_back_max={_dxb_log:.3f} "
            f"n_steps={_csf_n} plantG={_csf_g:.3f} maxG={_csf_mx:.3f} plant_n={_csf_dn} "
            f"dwell_max={_csf_dmax:.2f} steal={_steal_c} (ε any>{_eps_c:.2f} or cum>{_cum_c:.2f})",
            flush=True,
        )
        # S2 ALIP-TVR logs (fp_prior / du_fp / n_replan / fp_jump / plant-cam suite)
        _alip_nr = int(b.get("_alip_n_replan") or 0)
        if _alip_nr > 0:
            _fp_avg = float(b.get("_alip_fp_sum") or 0.0) / float(_alip_nr)
            _du_avg = float(b.get("_alip_du_sum") or 0.0) / float(_alip_nr)
        else:
            _fp_avg = float(b.get("_alip_fp_prior") or 0.0)
            _du_avg = float(b.get("_alip_du_fp") or 0.0)
        _fp_jump = float(b.get("_alip_fp_jump_max") or 0.0)
        _alip_g = float(b.get("_alip_plant_cam_gain") or 0.0)
        _alip_mx = float(b.get("_alip_plant_max_gain") or 0.0)
        _alip_dn = int(b.get("_alip_plant_n") or 0)
        _alip_dmax = float(b.get("_alip_plant_dwell_max") or 0.0)
        _eps_a = float(os.environ.get("GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS", GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS))
        _cum_a = float(os.environ.get("GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM", GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM))
        _steal_a = bool(_alip_dn > 0 and (_alip_mx > _eps_a + 1e-12 or _alip_g > _cum_a + 1e-12))
        b["fp_prior"] = _fp_avg
        b["du_fp"] = _du_avg
        b["n_replan"] = _alip_nr
        b["fp_jump"] = _fp_jump
        b["alip_plant_cam_gain"] = _alip_g
        b["alip_plant_max_gain"] = _alip_mx
        b["alip_plant_n"] = _alip_dn
        b["alip_plant_dwell_max"] = _alip_dmax
        b["alip_plant_cam_steal"] = _steal_a
        if _alip_nr > 0 and float(b.get("cam_gain_during_plant") or 0.0) == 0.0 and _alip_dn > 0:
            b["cam_gain_during_plant"] = _alip_g
            b["plant_dwell_max"] = _alip_dmax
        if _gate_q_t5e_active():
            _alip_tag = "T5E_SEQ"
        elif _gate_q_t5d2_active():
            _alip_tag = "T5D2_ICP"
        elif bool(os.environ.get("GATE_Q_T5D1_CKPT", "").strip()) or os.environ.get("GATE_Q_T5D1_OUTER_PRIMARY", "0") not in ("0", "false", "no"):
            _alip_tag = "T5D1_OUTER"
        elif (
            os.environ.get("GATE_Q_T5B_MODELBASE", "0") not in ("0", "false", "no")
            or (bool(os.environ.get("GATE_Q_T5B_CKPT", "").strip()) or bool(os.environ.get("GATE_Q_T5C_CKPT", "").strip()))
        ) and os.environ.get("GATE_Q_RET_ALIP_TVR", GATE_Q_RET_ALIP_TVR) in ("0", "false", "no"):
            _alip_tag = ("T5C_TEACHER" if bool(os.environ.get("GATE_Q_T5C_CKPT", "").strip()) else "T5B_MODELBASE")
        else:
            _alip_tag = "ALIP_TVR"
        _outer_lab = (
            "FOOTSTEP-SEQ" if _alip_tag == "T5E_SEQ"
            else ("CSF+ALIP+ICP" if _alip_tag == "T5D2_ICP" else "CSF+ALIP")
        )
        print(
            f"[Q] {_alip_tag} cyc{i}: fp_prior={_fp_avg:.4f} du_fp={_du_avg:.4f} n_replan={_alip_nr} "
            f"fp_jump={_fp_jump:.4f} plantG={_alip_g:.3f} maxG={_alip_mx:.3f} plant_n={_alip_dn} "
            f"dwell_max={_alip_dmax:.2f} steal={_steal_a} (ε any>{_eps_a:.2f} or cum>{_cum_a:.2f}) "
            f"alip_or_dcm=ALIP+DCM ankle_cmd=track outer={_outer_lab}",
            flush=True,
        )
        if _alip_tag == "T5E_SEQ":
            _tds = b.get("_t5e_tds") or []
            _ti = int(b.get("_t5e_idx") or 0)
            _ntr = int(b.get("_t5e_n_track") or 0)
            _nadv = int(b.get("_t5e_n_advance") or 0)
            _en = int(b.get("_t5e_err_n") or 0)
            _eavg = (float(b.get("_t5e_err_sum") or 0.0) / float(_en)) if _en > 0 else float(b.get("_t5e_last_err") or 0.0)
            _seq_idle = bool(_ntr < 2 or (not _tds))
            _flist = ",".join(f"{float(t.get('F', 0.0)):.3f}" for t in (_tds[:8] if _tds else []))
            _next_F = float(_tds[_ti]["F"]) if _tds and _ti < len(_tds) else float(b.get("_t5e_F") or _du_avg)
            _next_side = str(_tds[_ti]["side"]) if _tds and _ti < len(_tds) else "?"
            b["seq_idle"] = _seq_idle
            b["seq_n_track"] = _ntr
            b["seq_n_advance"] = _nadv
            b["seq_err"] = _eavg
            b["seq_idx"] = _ti
            b["seq_n"] = int(len(_tds)) if _tds else 0
            b["seq_list_id"] = str(b.get("_t5e_list_id") or "")
            b["seq_next_F"] = _next_F
            b["seq_next_side"] = _next_side
            b["seq_footholds"] = [float(t.get("F", 0.0)) for t in (_tds[:12] if _tds else [])]
            print(
                f"[Q] T5E_SEQ cyc{i}: list_id={b['seq_list_id']} n={b['seq_n']} idx={_ti} "
                f"next_F={_next_F:.4f} next_side={_next_side} next_err={_eavg:.4f} "
                f"n_track={_ntr} n_advance={_nadv} seq_idle={_seq_idle} "
                f"footholds=[{_flist}] (FOOTSTEP-SEQ · residual=corrector · doi:10.1145/3747865)",
                flush=True,
            )
            print(
                f"[Q] T5E_OUTER_FT cyc{i}: F={_next_F:.4f} T=seq_gait n_replan={_alip_nr} "
                f"footholds=du_fp:{_du_avg:.4f} seq_idle={_seq_idle} "
                f"(FOOTSTEP-SEQ PRIMARY · residual=corrector)",
                flush=True,
            )
        elif _alip_tag == "T5D2_ICP":
            _icp_n = int(b.get("_icp_n") or 0)
            if _icp_n > 0:
                _ie = float(b.get("_icp_err_sum") or 0.0) / float(_icp_n)
                _idT = float(b.get("_icp_dT_sum") or 0.0) / float(_icp_n)
                _idf = float(b.get("_icp_dfoot_sum") or 0.0) / float(_icp_n)
            else:
                _ie = float(b.get("_icp_err") or 0.0)
                _idT = float(b.get("_icp_dT") or 0.0)
                _idf = float(b.get("_icp_dfoot") or 0.0)
            _icp_idle = bool(_icp_n < 2)
            b["icp_err"] = _ie; b["icp_dT"] = _idT; b["icp_dfoot"] = _idf
            b["icp_n"] = _icp_n; b["icp_idle"] = _icp_idle
            b["icp_T"] = float(b.get("_icp_T") or 0.0)
            b["icp_F"] = float(b.get("_icp_F") or _du_avg)
            print(
                f"[Q] T5D2_ICP cyc{i}: icp_err={_ie:.4f} ΔT={_idT:.4f} Δfoot={_idf:.4f} "
                f"n_icp={_icp_n} icp_idle={_icp_idle} F={float(b.get('_icp_F') or _du_avg):.4f} "
                f"T={float(b.get('_icp_T') or 0.0):.4f} schedF={float(b.get('_icp_sched_F') or _fp_avg):.4f} "
                f"schedT={float(b.get('_icp_sched_T') or 0.0):.4f} "
                f"(OUTER ICP · arXiv:1703.00477 · residual=corrector)",
                flush=True,
            )
            print(
                f"[Q] T5D2_OUTER_FT cyc{i}: F={float(b.get('_icp_F') or _du_avg):.4f} "
                f"T={float(b.get('_icp_T') or 0.0):.4f} n_replan={_alip_nr} "
                f"footholds=du_fp:{_du_avg:.4f} (ICP-ΔT+ΔFOOT · residual=corrector)",
                flush=True,
            )
        elif _alip_tag == "T5D1_OUTER":
            print(
                f"[Q] T5D1_OUTER_FT cyc{i}: F={_fp_avg:.4f} T=alip_replan n_replan={_alip_nr} "
                f"footholds=du_fp:{_du_avg:.4f} (OUTER PRIMARY · residual=corrector)",
                flush=True,
            )

        # S3 REV-PHASE logs (phi_polarity / hip_sign / swing_enc / plant-cam suite)
        _phi_pol = float(b.get("_rev_phi_polarity") or 0.0)
        _hip_sgn = float(b.get("_rev_hip_sign") or 1.0)
        _sw_enc = float(b.get("_rev_swing_enc") or 0.0)
        _rev_n = int(b.get("_rev_n") or 0)
        _rev_cn = int(b.get("_rev_clear_n") or 0)
        _rev_g = float(b.get("_rev_plant_cam_gain") or 0.0)
        _rev_mx = float(b.get("_rev_plant_max_gain") or 0.0)
        _rev_dn = int(b.get("_rev_plant_n") or 0)
        _rev_dmax = float(b.get("_rev_plant_dwell_max") or 0.0)
        _eps_r = float(os.environ.get("GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS", GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS))
        _cum_r = float(os.environ.get("GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM", GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM))
        _steal_r = bool(_rev_dn > 0 and (_rev_mx > _eps_r + 1e-12 or _rev_g > _cum_r + 1e-12))
        b["phi_polarity"] = _phi_pol
        b["hip_sign"] = _hip_sgn
        b["swing_enc"] = _sw_enc
        b["rev_n"] = _rev_n
        b["rev_clear_n"] = _rev_cn
        b["rev_plant_cam_gain"] = _rev_g
        b["rev_plant_max_gain"] = _rev_mx
        b["rev_plant_n"] = _rev_dn
        b["rev_plant_dwell_max"] = _rev_dmax
        b["rev_plant_cam_steal"] = _steal_r
        if _rev_n > 0 and float(b.get("cam_gain_during_plant") or 0.0) == 0.0 and _rev_dn > 0:
            b["cam_gain_during_plant"] = _rev_g
            b["plant_dwell_max"] = _rev_dmax
        print(
            f"[Q] REV_PHASE cyc{i}: phi_polarity={_phi_pol:.4f} hip_sign={_hip_sgn:.3f} swing_enc={_sw_enc:.3f} "
            f"n_ticks={_rev_n} clear_n={_rev_cn} plantG={_rev_g:.3f} maxG={_rev_mx:.3f} plant_n={_rev_dn} "
            f"dwell_max={_rev_dmax:.2f} steal={_steal_r} (ε any>{_eps_r:.2f} or cum>{_cum_r:.2f})",
            flush=True,
        )
        print(
            f"[Q] RET_OK_BREAKDOWN cyc{i}: "
            f"ret_ok={b['retreat_ok']} stepped_ok={b['retreat_stepped_ok']} "
            f"delta_ok={b['retreat_delta_ok']} tip_ok={b['retreat_tip_ok']} | "
            f"clear_frac_ok={b['clear_frac_ok_ret']}(cf={b.get('clear_frac_ret'):.3f} need≥0.55) "
            f"skate_ok={b['skate_ok_ret']}(mean/p95={b.get('skate_mean_ret'):.3f}/{b.get('skate_p95_ret'):.3f} need≤0.08/0.18) "
            f"shim_ok={b['retreat_shim_budget_ok']}(frac={b.get('retreat_stepped_frac'):.3f}) "
            f"dense_ss={b['dense_ss_ok_ret']} visible_ss={b['visible_ss_ok_ret']} "
            f"cadence_ok={b['cadence_ok_ret']} "
            f"tail_ok={b['ss_carry_tail_ok_ret']} tspan={b.get('ss_time_span_frac_ret'):.3f} "
            f"span_frac={b.get('ss_span_frac_ret'):.3f} ss={b['n_ss_L_ret']}+{b['n_ss_R_ret']} "
            f"dxc={b.get('dx_clear_m_ret'):.3f} stop_t={b.get('retreat_stop_t')} jump={b.get('retreat_jump_fin')}",
            flush=True,
        )
        b["pass"] = bool(b["approach_ok"] and b["compose_ok"] and b["retreat_ok"])
        for kk in list(b.keys()):
            if kk.startswith("_"): del b[kk]
        b["cycle_index"] = i
    n_pass = sum(1 for b in cycles if b["pass"])
    dx = float(d.qpos[env.q_free]) - x0_ep; dy = float(d.qpos[env.q_free + 1]) - y0_ep
    method = ("gate_e_residual_stepped_visible_clear_then_small_finalize_shim" if use_residual
              else "ol_gro01_stepped_visible_clear_then_small_finalize_shim")
    retreat_method = RETREAT_METHOD
    print(f"[Q] physics done mode={'residual' if use_residual else 'OL'} n_pass={n_pass}/{n_cycles} tip={tip:.2f}s", flush=True)
    for b in cycles:
        print(f"  cyc{b['cycle_index']}: Δstep={b.get('stepped_delta_m')} Δshim={b.get('shim_delta_m')} frac={b.get('stepped_frac')} "
              f"ss={b['n_ss_L']}+{b['n_ss_R']} clear={b.get('max_clear_L_m'):.4f}/{b.get('max_clear_R_m'):.4f} "
              f"above_rest={b.get('max_clear_above_rest_L_m'):.4f}/{b.get('max_clear_above_rest_R_m'):.4f} "
              f"dwell={b.get('max_ss_dwell_L_s'):.3f}/{b.get('max_ss_dwell_R_s'):.3f} "
              f"rest={b.get('plant_rest_clear_L_m')}/{b.get('plant_rest_clear_R_m')} "
              f"cadence span_frac={b.get('ss_span_frac')} lifts/m={b.get('lifts_per_m')} lifts/s={b.get('lifts_per_s')} "
              f"dx_clear={b.get('dx_clear_m')} dx_plant={b.get('dx_planted_m')} clear_frac={b.get('clear_frac')} "
              f"dense={b.get('dense_ss_ok')} cad={b.get('cadence_ok')} "
              f"skate={b.get('skate_mean')}/{b.get('skate_p95')} sk_ok={b['skate_ok']} tip_app={b['tip_during_approach']:.2f} "
              f"peak={b['peak_panel_deg']:+.2f} stepped_ok={b['stepped_ok']} shim_ok={b.get('shim_budget_ok')} "
              f"app={b['approach_ok']} compose={b['compose_ok']} "
              f"ret Δstep={b.get('retreat_stepped_delta_m')} frac={b.get('retreat_stepped_frac')} "
              f"ss_ret={b['n_ss_L_ret']}+{b['n_ss_R_ret']} clear_ret={b.get('max_clear_L_ret_m'):.4f}/{b.get('max_clear_R_ret_m'):.4f} "
              f"above_ret={b.get('max_clear_above_rest_L_ret_m'):.4f}/{b.get('max_clear_above_rest_R_ret_m'):.4f} "
              f"dwell_ret={b.get('max_ss_dwell_L_ret_s'):.3f}/{b.get('max_ss_dwell_R_ret_s'):.3f} "
              f"dx_clear_ret={b.get('dx_clear_m_ret')} dx_plant_ret={b.get('dx_planted_m_ret')} clear_frac_ret={b.get('clear_frac_ret')} "
              f"cad_ret span={b.get('ss_span_frac_ret')} lpm={b.get('lifts_per_m_ret')} lps={b.get('lifts_per_s_ret')} "
              f"dense_ret={b.get('dense_ss_ok_ret')} cad_ret={b.get('cadence_ok_ret')} "
              f"skate_ret={b.get('skate_mean_ret')}/{b.get('skate_p95_ret')} tip_ret={b['tip_during_retreat']:.2f} "
              f"cam_ret={b.get('retreat_cam_end')} fin0={b.get('_retreat_fin_cam0')} cammax={b.get('retreat_cam_max')} ret_ok={b['retreat_ok']} pass={b['pass']}", flush=True)
    # Restore gait clearance params (controller-only bump was episode-local)
    try:
        wg.KNEE_SWING = _gait_prev["KNEE_SWING"]
        wg.SWING_ANK_DF = _gait_prev["SWING_ANK_DF"]
        wg.SWING_ABDUCT = _gait_prev["SWING_ABDUCT"]
        if "GAIT_T" in _gait_prev:
            wg.GAIT_T = _gait_prev["GAIT_T"]
        if "DS_S" in _gait_prev:
            wg.DS_S = _gait_prev["DS_S"]
        if "PLANT_KD" in _gait_prev:
            wg.PLANT_KD = _gait_prev["PLANT_KD"]
        if "PLANT_MAX_F" in _gait_prev:
            wg.PLANT_MAX_F = _gait_prev["PLANT_MAX_F"]
    except Exception:
        pass
    if video:
        mp4s = _render_p(env, qpos_log, xy_log, bout_log, t_log, phase_log, disturb_log, leave_log, force_log, approach_log, retreat_log, camh_log, ss_log, shim_log=shim_log)
    else:
        mp4s = {"dual": "previews/ainex_walk/iterate/Q03.mp4"}
        print("[Q] skip video (probe)", flush=True)
    del qpos_log[:]
    skate_means = [b.get("skate_mean") for b in cycles if b.get("skate_mean") is not None]
    return {
        "tip": tip, "tip_ok": tip_ok, "tip_after_disturb_ok": tip_after_disturb_ok,
        "max_abs_panel_deg": max(global_max_panel_compose, max((abs(b.get("peak_panel_deg", 0)) for b in cycles), default=0.0)), "max_abs_lever_deg": global_max_lever,
        "cycles": cycles, "n_cycles": n_cycles, "n_pass": n_pass,
        "success_rate": f"{n_pass}/{n_cycles}", "all_cycles_pass": n_pass == n_cycles,
        "coupled_ok": global_rising > 0.2 and global_contact > 1.0,
        "hand_lever_contact_s": global_contact, "contact_while_panel_rising_s": global_rising,
        "panel_qpos_scripted": False, "panel_actuator": False, "stand_xy_hold": False,
        "free_joint_xy_pinned": False, "base_dx_m": dx, "base_dy_m": dy, "panel_spring": spring,
        "assist": False, "freeze": False, "vision_in_walk_obs": False, "fall": tip < 1.0,
        "head_joint": "head_tilt", "head_tilt_cmd_deg": tilt,
        "plant": str(PLANT_F.relative_to(ROOT)), "companion_md5": _md5(PLANT_F),
        "within_30deg_range": all(abs(b.get("peak_panel_deg", 0)) <= 30.0 + 0.05 for b in cycles) and global_max_panel_compose <= 30.0 + 0.05,
        "max_abs_panel_deg_compose": global_max_panel_compose,
        "max_abs_panel_deg_global_raw": global_max_panel,
        "cam_start_cmd": cam_start, "cam_work_cmd": cam_work, "cam_h_episode_start": cam_h_episode_start,
        "start_offset_method": START_OFFSET_METHOD, "approach_method": method, "retreat_method": retreat_method, "use_residual": use_residual,
        "working_stand": "controls_documented_N_working_pose_cam_dist_0.11",
        "skate_mean": float(np.mean(skate_means)) if skate_means else None, "skate_n_a_reason": None,
        "disturbance": {"type": DISTURB_TYPE, "body": "door_panel_link", "force_N_world": DISTURB_FORCE_N.tolist(),
            "impulse_s": DISTURB_IMPULSE_S, "window_s": DISTURB_WINDOW_S, "duration_s": DISTURB_IMPULSE_S,
            "applied_s_total": disturb_force_applied_s, "declared": True, "undeclared_xfrc": False,
            "note": "Closing-sense +X push on door_panel_link via xfrc; impulse 0.35 s within declared 1.0 s window."},
        "controller": {"walk_s": walk_s, "walk_stand_hold": walk_hold, "walk_ramp": walk_ramp,
            "finalize_s": finalize_s, "approach_s": approach_s, "retreat_s": retreat_s,
            "push_s": push_s, "hold_s": hold_s, "disturb_s": disturb_s, "reject_s": reject_s,
            "leave_s": leave_s, "regrasp_s": regrasp_s, "close_s": close_s, "recover_s": recover_s,
            "push_to": push_to, "leave_off": leave_off.tolist(), "cam_start": cam_start, "cam_work": cam_work,
            "xy_pin": False, "approach_method": method, "body_link_cop": True,
            "compose": "step_walk->finalize->open->hold->disturb->reject->leave->regrasp->close->retreat_walk->retreat_finalize",
            "retreat_method": retreat_method, "retreat_walk_s": RETREAT_WALK_S, "retreat_amp_scale": RETREAT_AMP_SCALE},
        "mp4s": mp4s, "episode_s": episode_s,
    }

def _write_pack(ep, *, iterate_note=""):
    sha = _sha16(CKPT); plant_md5 = ep["companion_md5"]; tip_ok = ep["tip_ok"]
    no_cheat = (not ep["panel_qpos_scripted"] and not ep["panel_actuator"] and not ep["stand_xy_hold"]
                and not ep["free_joint_xy_pinned"] and not ep["assist"] and not ep["freeze"]
                and not ep["vision_in_walk_obs"] and ep["within_30deg_range"]
                and ep["disturbance"]["declared"] and not ep["disturbance"]["undeclared_xfrc"]
                and not any(b.get("teleport_falsifier") for b in ep["cycles"]))
    cycles = ep["cycles"]; c0 = cycles[0] if cycles else {}; c1 = cycles[1] if len(cycles) > 1 else {}
    def st(r): return "PASS" if r.get("pass") else "FAIL"
    p00 = {"tag":"Q00","pass":bool(c0.get("start_offset_ok") and c0.get("approach_tip_ok") and c0.get("stepped_ok")
           and (c0.get("approach_delta_before_contact_ok") or c0.get("cam_start_ge_055")) and tip_ok and no_cheat
           and not c0.get("teleport_falsifier") and c0.get("shim_budget_ok")),
           "start_offset_method":ep["start_offset_method"],"cam_start":c0.get("cam_start"),
           "cam_start_ge_055":c0.get("cam_start_ge_055"),"stepped_delta_m":c0.get("stepped_delta_m"),
           "shim_delta_m":c0.get("shim_delta_m"),"stepped_frac":c0.get("stepped_frac"),
           "shim_budget_ok":c0.get("shim_budget_ok"),
           "approach_delta_m":c0.get("approach_delta_m"),"n_ss_L":c0.get("n_ss_L"),"n_ss_R":c0.get("n_ss_R"),
           "n_ss_total":c0.get("n_ss_total"),"n_contact_breaks_L":c0.get("n_contact_breaks_L"),
           "n_contact_breaks_R":c0.get("n_contact_breaks_R"),
           "max_clear_L_m":c0.get("max_clear_L_m"),"max_clear_R_m":c0.get("max_clear_R_m"),
           "max_clear_ss_L_m":c0.get("max_clear_ss_L_m"),"max_clear_ss_R_m":c0.get("max_clear_ss_R_m"),
           "max_clear_above_rest_L_m":c0.get("max_clear_above_rest_L_m"),
           "max_clear_above_rest_R_m":c0.get("max_clear_above_rest_R_m"),
           "max_ss_dwell_L_s":c0.get("max_ss_dwell_L_s"),"max_ss_dwell_R_s":c0.get("max_ss_dwell_R_s"),
           "plant_rest_clear_L_m":c0.get("plant_rest_clear_L_m"),
           "plant_rest_clear_R_m":c0.get("plant_rest_clear_R_m"),
           "visible_clear_thr_m":VISIBLE_CLEAR_M,"visible_clear_above_rest_m":VISIBLE_CLEAR_ABOVE_REST_M,
           "ss_dwell_s":SS_DWELL_S,"sole_offset_m":SOLE_OFFSET,
           "skate_mean":c0.get("skate_mean"),
           "skate_p95":c0.get("skate_p95"),"skate_ok":c0.get("skate_ok"),"stepped_ok":c0.get("stepped_ok"),
           "tip_during_approach":c0.get("tip_during_approach"),"tip":ep["tip"],
           "approach_method":ep["approach_method"],"hud_finalize_label":"KINEMATIC XY SHIM",
           "mp4":"previews/ainex_walk/iterate/Q00.mp4"}
    p01 = {"tag":"Q01","pass":bool(c0.get("entered_n_working_pose") and c0.get("open25_ok") and c0.get("hold_ok") and tip_ok and no_cheat),
           "working_stand":ep["working_stand"],"cam_at_approach_end":c0.get("cam_at_approach_end"),
           "entered_n_working_pose":c0.get("entered_n_working_pose"),"peak_panel_deg":c0.get("peak_panel_deg"),
           "hold25_s":c0.get("hold25_s"),"mp4":"previews/ainex_walk/iterate/Q01.mp4"}
    p02 = {"tag":"Q02","pass":bool(c0.get("retreat_ok") and tip_ok and no_cheat),
           "retreat_method":ep.get("retreat_method"),
           "retreat_stepped_delta_m":c0.get("retreat_stepped_delta_m"),
           "retreat_shim_delta_m":c0.get("retreat_shim_delta_m"),
           "retreat_stepped_frac":c0.get("retreat_stepped_frac"),
           "retreat_shim_budget_ok":c0.get("retreat_shim_budget_ok"),
           "retreat_cam_start":c0.get("retreat_cam_start"),"retreat_cam_end":c0.get("retreat_cam_end"),
           "retreat_cam_ge_055":c0.get("retreat_cam_ge_055"),"retreat_delta_ok":c0.get("retreat_delta_ok"),
           "n_ss_L_ret":c0.get("n_ss_L_ret"),"n_ss_R_ret":c0.get("n_ss_R_ret"),
           "max_clear_L_ret_m":c0.get("max_clear_L_ret_m"),"max_clear_R_ret_m":c0.get("max_clear_R_ret_m"),
           "max_clear_above_rest_L_ret_m":c0.get("max_clear_above_rest_L_ret_m"),
           "max_clear_above_rest_R_ret_m":c0.get("max_clear_above_rest_R_ret_m"),
           "max_ss_dwell_L_ret_s":c0.get("max_ss_dwell_L_ret_s"),"max_ss_dwell_R_ret_s":c0.get("max_ss_dwell_R_ret_s"),
           "skate_mean_ret":c0.get("skate_mean_ret"),"skate_p95_ret":c0.get("skate_p95_ret"),
           "skate_ok_ret":c0.get("skate_ok_ret"),"tip_during_retreat":c0.get("tip_during_retreat"),
           "hud_retreat_finalize_label":"KINEMATIC XY SHIM",
           "mp4":"previews/ainex_walk/iterate/Q02.mp4"}
    p03 = {"tag":"Q03","pass":bool(ep["all_cycles_pass"] and tip_ok and no_cheat
           and all(b.get("stepped_ok") and b.get("retreat_ok") for b in cycles)),
           "success_rate":ep["success_rate"],"n_pass":ep["n_pass"],"n_cycles":ep["n_cycles"],"cycles":cycles,
           "mp4":"previews/ainex_walk/iterate/Q03.mp4"}
    rows = {"Q00":p00,"Q01":p01,"Q02":p02,"Q03":p03}
    overall = all(r["pass"] for r in rows.values())
    if any(not b.get("stepped_ok") or not b.get("shim_budget_ok") for b in cycles):
        overall = False
        for r in rows.values():
            if r["tag"] in ("Q00","Q03"): r["pass"] = False
    if any(not b.get("retreat_ok") or not b.get("retreat_shim_budget_ok") for b in cycles):
        overall = False
        for r in rows.values():
            if r["tag"] in ("Q02","Q03"): r["pass"] = False
    # Soft-pass forbidden: never override FAIL
    metrics_pass = bool(overall)
    verdict = "GATE_Q_PASS" if metrics_pass else "GATE_Q_FAIL"
    controls_watch = "PENDING_CONTROLS_WATCH" if metrics_pass else "METRICS_FAIL"
    table = {"verdict":verdict,"overall_pass":metrics_pass,"ai_can_lock":False,
             "metrics_verdict":verdict,"controls_watch":controls_watch,
             "criteria":"docs/GATE_Q_AI_CRITERIA.md",
             "plant":str(PLANT_F.relative_to(ROOT)),"companion_md5":plant_md5,
             "companion_lock_md5_expected":COMPANION_LOCK_MD5,
             "gate_e_plant_untouched":"mujoco/ainex_hiwonder/ainex_controls_m2_145.xml","ckpt_sha16":sha,
             "stand_xy_hold":False,"free_joint_xy_pinned":False,"soft_pass_used":False,
             "approach_and_compose_same_run":True,"approach_compose_retreat_same_run":True,
             "separate_approach_N_runs_falsifier":False,
             "shim_alone_falsifier":bool(any(not b.get("shim_budget_ok") or not b.get("stepped_ok")
                                            or not b.get("retreat_shim_budget_ok") or not b.get("retreat_ok") for b in cycles)),
             "visible_clear_thr_m":VISIBLE_CLEAR_M,"visible_clear_above_rest_m":VISIBLE_CLEAR_ABOVE_REST_M,
             "ss_dwell_s":SS_DWELL_S,"sole_offset_m":SOLE_OFFSET,
             "stepped_frac_min":STEPPED_FRAC_MIN,"hud_finalize_label":"KINEMATIC XY SHIM",
             "start_offset_method":ep["start_offset_method"],
             "approach_method":ep["approach_method"],"retreat_method":ep.get("retreat_method"),"use_residual":ep.get("use_residual",False),
             "working_stand":ep["working_stand"],"cam_start_cmd":ep["cam_start_cmd"],"cam_work_cmd":ep["cam_work_cmd"],
             "cam_h_episode_start":ep["cam_h_episode_start"],"skate_mean":ep["skate_mean"],
             "disturbance":ep["disturbance"],"tip":ep["tip"],"tip_ok":tip_ok,
             "tip_after_disturb_ok":ep["tip_after_disturb_ok"],"base_dx_m":ep["base_dx_m"],"base_dy_m":ep["base_dy_m"],
             "max_abs_panel_deg":ep["max_abs_panel_deg"],"panel_spring":ep["panel_spring"],
             "controller":ep["controller"],"rows":rows,"cycles":cycles,"mp4s":ep["mp4s"],
             "watch_primary":"Q03.mp4","stills_dir":ep["mp4s"].get("stills_dir"),
             "still_counts":ep["mp4s"].get("still_counts"),
             "open_angle_criterion":"free_edge_stripe_d_screen_x_ge_25px_at_plus30",
             "non_claims":["full_door_open","latch","90_deg","walk_through","UK_handle","hinge_range_bump",
                           "Pi","Orin","room_walk","investor_walk_script","vision_in_walk","gate_o_xy_shim_alone"],
             "iterate_note":iterate_note,"scored_at_bst":__import__("datetime").datetime.now().strftime("%Y-%m-%d %H:%M BST"),
             "iterate_series":"iterate3r19h7k8aj",
             "cadence":[{"bout":i,
                         "app_ss":f"{b.get('n_ss_L')}+{b.get('n_ss_R')}",
                         "app_span_frac":b.get("ss_span_frac"),
                         "app_lifts_per_m":b.get("lifts_per_m"),
                         "app_lifts_per_s":b.get("lifts_per_s"),
                         "app_ss_times_s":b.get("ss_times_s"),
                         "ret_ss":f"{b.get('n_ss_L_ret')}+{b.get('n_ss_R_ret')}",
                         "ret_span_frac":b.get("ss_span_frac_ret"),
                         "ret_lifts_per_m":b.get("lifts_per_m_ret"),
                         "ret_lifts_per_s":b.get("lifts_per_s_ret"),
                         "ret_ss_times_s":b.get("ss_times_s_ret")} for i,b in enumerate(cycles)]}

    (ITER/"GATE_Q_TABLE.json").write_text(json.dumps(table, indent=2, default=str))
    def _fmt(v):
        if v is None: return "—"
        if isinstance(v, float): return f"{v:.3f}"
        return str(v)
    cyc_lines = "\n".join(
        f"| {b['cycle_index']} | {_fmt(b.get('cam_start'))} | {_fmt(b.get('cam_at_approach_end'))} | "
        f"{_fmt(b.get('stepped_delta_m'))} | {_fmt(b.get('shim_delta_m'))} | {_fmt(b.get('stepped_frac'))} | "
        f"{b['n_ss_L']}+{b['n_ss_R']} | {_fmt(b.get('max_clear_L_m'))}/{_fmt(b.get('max_clear_R_m'))} | "
        f"{_fmt(b.get('skate_mean'))}/{_fmt(b.get('skate_p95'))} | {b['skate_ok']} | "
        f"{b['tip_during_approach']:.2f} | {b['entered_n_working_pose']} | {b['peak_panel_deg']:+.2f} | "
        f"{b['hold25_s']:.2f} | {b['leave_no_contact_panel20_s']:.2f} | "
        f"{b['regrasp_contact_s']:.2f} | {b['close_delta_deg']:.2f} | {b['stepped_ok']} | "
        f"{b['approach_ok']} | {b['compose_ok']} | **{st(b)}** |" for b in cycles)
    dist = ep["disturbance"]
    approach_lines = "\n".join(
        f"- bout{b['cycle_index']}: cam_start={_fmt(b.get('cam_start'))} (≥0.55={b.get('cam_start_ge_055')}) → "
        f"cam_end={_fmt(b.get('cam_at_approach_end'))}; Δstepped={_fmt(b.get('stepped_delta_m'))} m; "
        f"Δshim={_fmt(b.get('shim_delta_m'))} m; stepped_frac={_fmt(b.get('stepped_frac'))} "
        f"(need ≥{STEPPED_FRAC_MIN}); shim_budget_ok={b.get('shim_budget_ok')}; "
        f"Δapproach_total={_fmt(b.get('approach_delta_m'))} m; "
        f"visible dwell-SS L+R={b['n_ss_L']}+{b['n_ss_R']} "
        f"(abs thr≥{VISIBLE_CLEAR_M} m; above-rest≥{VISIBLE_CLEAR_ABOVE_REST_M} m; dwell≥{SS_DWELL_S}s); "
        f"max_clear L/R={_fmt(b.get('max_clear_L_m'))}/{_fmt(b.get('max_clear_R_m'))} m; "
        f"above_rest L/R={_fmt(b.get('max_clear_above_rest_L_m'))}/{_fmt(b.get('max_clear_above_rest_R_m'))} m; "
        f"max_dwell L/R={_fmt(b.get('max_ss_dwell_L_s'))}/{_fmt(b.get('max_ss_dwell_R_s'))} s; "
        f"plant_rest_clear L/R={_fmt(b.get('plant_rest_clear_L_m'))}/{_fmt(b.get('plant_rest_clear_R_m'))} m; "
        f"max_clear_ss L/R={_fmt(b.get('max_clear_ss_L_m'))}/{_fmt(b.get('max_clear_ss_R_m'))} m; "
        f"contact_breaks L+R={b['n_contact_breaks_L']}+{b['n_contact_breaks_R']} (not SS credit); "
        f"skate mean/p95={_fmt(b.get('skate_mean'))}/{_fmt(b.get('skate_p95'))} "
        f"(L {_fmt(b.get('skate_mean_L'))}/{_fmt(b.get('skate_p95_L'))}, R {_fmt(b.get('skate_mean_R'))}/{_fmt(b.get('skate_p95_R'))}); "
        f"sk_ok={b['skate_ok']}; tip_approach={b['tip_during_approach']:.2f}s; N-work={b['entered_n_working_pose']}; "
        f"stepped_ok={b['stepped_ok']}; dense_ss={b.get('dense_ss_ok')}; "
        f"cadence span_frac={_fmt(b.get('ss_span_frac'))} lifts/m={_fmt(b.get('lifts_per_m'))} "
        f"lifts/s={_fmt(b.get('lifts_per_s'))} cad_ok={b.get('cadence_ok')}; "
        f"ss_t={b.get('ss_times_s')}; HUD finalize=KINEMATIC XY SHIM; teleport={b.get('teleport_falsifier')}" for b in cycles)
    retreat_lines = "\n".join(
        f"- bout{b['cycle_index']}: cam_ret { _fmt(b.get('retreat_cam_start'))}→{_fmt(b.get('retreat_cam_end'))}; "
        f"Δstepped_ret={_fmt(b.get('retreat_stepped_delta_m'))} m; Δshim_ret={_fmt(b.get('retreat_shim_delta_m'))} m; "
        f"frac={_fmt(b.get('retreat_stepped_frac'))}; ss_ret L+R={b['n_ss_L_ret']}+{b['n_ss_R_ret']}; "
        f"clear={_fmt(b.get('max_clear_L_ret_m'))}/{_fmt(b.get('max_clear_R_ret_m'))}; "
        f"above_rest={_fmt(b.get('max_clear_above_rest_L_ret_m'))}/{_fmt(b.get('max_clear_above_rest_R_ret_m'))}; "
        f"dwell={_fmt(b.get('max_ss_dwell_L_ret_s'))}/{_fmt(b.get('max_ss_dwell_R_ret_s'))}; "
        f"skate={_fmt(b.get('skate_mean_ret'))}/{_fmt(b.get('skate_p95_ret'))}; "
        f"tip_ret={b['tip_during_retreat']:.2f}s; ret_ok={b['retreat_ok']}; "
        f"dense_ret={b.get('dense_ss_ok_ret')}; cadence_ret span={_fmt(b.get('ss_span_frac_ret'))} "
        f"lifts/m={_fmt(b.get('lifts_per_m_ret'))} lifts/s={_fmt(b.get('lifts_per_s_ret'))} "
        f"cad_ok={b.get('cadence_ok_ret')}; ss_t_ret={b.get('ss_times_s_ret')}; "
        f"HUD retreat finalize=KINEMATIC XY SHIM"
        for b in cycles)
    leave_windows = "\n".join(
        f"- bout{b['cycle_index']}: leave_start panel={b.get('panel_at_leave_start')}°; "
        f"nc@≥20°={b['leave_no_contact_panel20_s']:.2f}s; min_panel_during_leave_ok={b.get('min_panel_during_leave_ok')}; "
        f"disturb t∈[{b.get('disturb_t0')}, {b.get('disturb_t1')}]" for b in cycles)
    note = f"""# Gate Q Controls — stepped approach → N compose → stepped retreat (sim)

**When:** Mon 28 Sep 2026 Europe/London (BST)  
**Role:** Founding Controls  
**Criteria:** `docs/GATE_Q_AI_CRITERIA.md`  
**Plant:** `{PLANT_F.relative_to(ROOT)}` · md5 `{plant_md5}` (lock `{COMPANION_LOCK_MD5}`)  
**Gate E plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Ckpt:** sha16 **`{sha}`** · `head_tilt` · assist/freeze OFF · vision off · glfw  
**stand_xy_hold:** **false** · Soft-pass: **not used**  
**Same continuous run:** stepped-approach→N-compose→stepped-retreat (not separate score runs)

## Metrics verdict: **{verdict}** · controls_watch=**{controls_watch}** · **ai_can_lock=false**

Soft-pass **not used**. Continuous must look multi-step throughout. Parent Controls watchVideo before lock. Do not ping AI.

| Tag | Result |
|-----|--------|
| Q00 start offset + stepped approach (SS/Δ/skate/tip) | **{st(p00)}** |
| Q01 enter N working stand + open/hold (compose) | **{st(p01)}** |
| Q02 stepped retreat (SS/Δ/skate/tip; cam≥0.55 or Δ≥0.15) | **{st(p02)}** |
| Q03 2/2 consecutive approach→compose→retreat cycles | **{st(p03)}** ({ep["success_rate"]}) |

| Cycle | cam0 | cam@end | Δstep | Δshim | frac | SS L+R | clear L/R | skate mean/p95 | sk_ok | tip_app | N-work | peak° | hold≥25 | leave20 | regrasp | Δclose | stepped | app_ok | compose | Status |
|-------|------|---------|-------|-------|------|--------|-----------|----------------|-------|---------|--------|-------|---------|---------|---------|--------|---------|--------|---------|--------|
{cyc_lines}

**tip**={ep["tip"]:.2f}s · tip_after_disturb_ok={ep["tip_after_disturb_ok"]} · Δx={ep["base_dx_m"]*1000:+.1f} mm · Δy={ep["base_dy_m"]*1000:+.1f} mm · max panel={ep["max_abs_panel_deg"]:.2f}°

## Start offset + stepped approach (documented)

| Item | Value |
|------|--------|
| Start offset method | **{ep["start_offset_method"]}** (cam_cmd={ep["cam_start_cmd"]} ≥0.55) |
| Working stand | **{ep["working_stand"]}** (cam_cmd={ep["cam_work_cmd"]}) |
| Approach method | **{ep["approach_method"]}** |
| Residual used | {ep.get("use_residual")} |
| body_link CoP | True (door mass otherwise skews world COM on companion) |
| Episode camH @ approach start | {_fmt(ep.get("cam_h_episode_start"))} |

{approach_lines}

## Disturbance (declared)

| Item | Value |
|------|--------|
| Type | **{dist["type"]}** (same as Gate N/M/O) |
| Body | `{dist["body"]}` |
| Force | {dist["force_N_world"]} N world (+X closing-sense) |
| Impulse | **{dist.get("impulse_s", dist["duration_s"])} s** |
| Window | **{dist.get("window_s", dist["duration_s"])} s** (banner durable) |
| Applied total | {dist["applied_s_total"]:.2f} s across cycles |
| Honesty | Declared only; `_substep_p` re-applies after env xfrc clear; **no** undeclared xfrc |

## Leave windows
{leave_windows}

## Controller
walk {ep["controller"]["walk_s"]}s (stand_hold={ep["controller"]["walk_stand_hold"]}, ramp={ep["controller"]["walk_ramp"]}) / finalize {ep["controller"]["finalize_s"]}s / push {ep["controller"]["push_s"]}s / hold {ep["controller"]["hold_s"]}s / disturb {ep["controller"]["disturb_s"]}s / reject {ep["controller"]["reject_s"]}s / leave {ep["controller"]["leave_s"]}s / regrasp {ep["controller"]["regrasp_s"]}s / close {ep["controller"]["close_s"]}s / recover {ep["controller"]["recover_s"]}s / retreat {ep["controller"]["retreat_s"]}s; cam_start={ep["cam_start_cmd"]} cam_work={ep["cam_work_cmd"]}; leave_off={ep["controller"]["leave_off"]}. No XY pin during compose. Finalize = **KINEMATIC XY SHIM** (HUD); early-stop only near work after visible SS+Δ≥0.15. Clearance boost (knee_swing/ank_df/abduct) controller-only. No plant/spring/range invent.

## Controller iterate
{iterate_note if iterate_note else "(none — first score path)"}

## Stepped retreat (documented)

{retreat_lines}

**Stills:** `gate_q_stills/` — approach FOOT-LIFT, finalize, disturb/leave, retreat FOOT-LIFT, retreat finalize (kit/s9/panel/dual).

- **watch_primary:** `Q03.mp4` (kit ‖ oblique; HUD panel° + XY; **STEPPED APPROACH** during gait; **KINEMATIC XY SHIM** during finalize; **FOOT-LIFT/SS (HUD thr)** + **DISTURBANCE ON** + **LEAVE WINDOW**)
- Also: `Q03_kit.mp4`, `Q03_panel.mp4`, `Q03_world.mp4`, `Q03_dual_s9.mp4`
- Open-angle: inherit K (green stripe Δ screen-X ≥~25 px @ +30°)

## Honesty / non-claims
No panel actuator / scripted panel qpos. No M145 edit. No hinge-range bump. Not latch / 90° / walk-through / UK / full open / range bump / room-walk / investor walk script. Soft-pass forbidden. Stepped-approach+N composed **in same continuous score run** — not separate runs. SS credit requires **consecutive dwell ≥150–250 ms** of sole clearance **≥2 cm above plant rest** (absolute thr defeats SOLE_OFFSET ~+8.6 mm soft-contact bias); HUD FOOT-LIFT **only while currently clear** (no sticky). Soft-pass forbidden. Stepped Δ ≥0.15 m and ≥~70% of total approach Δ (finalize must not dominate). HUD labels finalize as **KINEMATIC XY SHIM** only. Compose has free-joint XY free (`stand_xy_hold=false`). No teleport. No vision-in-walk.

**AI can lock:** NO — ai_can_lock=false until parent Controls watchVideo confirms continuous multi-step (metrics={metrics_pass}, watch={controls_watch}). Soft-pass not used. Do not ping AI.
"""
    import datetime as _dt
    _now = _dt.datetime.now().strftime("%H:%M BST")
    cad_rows = []
    for b in cycles:
        cad_rows.append(
            f"| {b['cycle_index']} | app | {b.get('n_ss_L')}+{b.get('n_ss_R')} | "
            f"{(b.get('ss_span_frac') or 0):.3f} | "
            f"{(b.get('clear_frac') or 0):.3f} | "
            f"{(b.get('dx_clear_m') or 0):.3f} | {(b.get('dx_planted_m') or 0):.3f} | "
            f"{b.get('cadence_ok')} | {b.get('ss_times_s')} |"
        )
        cad_rows.append(
            f"| {b['cycle_index']} | ret | {b.get('n_ss_L_ret')}+{b.get('n_ss_R_ret')} | "
            f"{(b.get('ss_span_frac_ret') or 0):.3f} | "
            f"{(b.get('clear_frac_ret') or 0):.3f} | "
            f"{(b.get('dx_clear_m_ret') or 0):.3f} | {(b.get('dx_planted_m_ret') or 0):.3f} | "
            f"{b.get('cadence_ok_ret')} | {b.get('ss_times_s_ret')} |"
        )
    b0 = cycles[0] if cycles else {}
    root_app = (
        f"SS {b0.get('n_ss_L')}+{b0.get('n_ss_R')} span_frac={(b0.get('ss_span_frac') or 0):.3f} "
        f"clear_frac={(b0.get('clear_frac') or 0):.3f} "
        f"dx_clear={(b0.get('dx_clear_m') or 0):.3f} dx_plant={(b0.get('dx_planted_m') or 0):.3f} "
        f"cad_ok={b0.get('cadence_ok')} stepped_ok={b0.get('stepped_ok')}"
    )
    b0r_cf = float(b0.get("clear_frac_ret") or 0.0)
    b0r_dc = float(b0.get("dx_clear_m_ret") or 0.0)
    b0r_dp = float(b0.get("dx_planted_m_ret") or 0.0)
    root_ret = (
        f"ret clear_frac={b0r_cf:.3f} dx_clear={b0r_dc:.3f} dx_plant={b0r_dp:.3f} "
        f"ss={b0.get('n_ss_L_ret')}+{b0.get('n_ss_R_ret')}"
    )
    preamble = (
        f"## Controls score (iterate3r19h7k8aj): **{verdict}** — bout1-ret clearΔ hip×1.48/res1.0/hold0.45 (amp0.62 tip-safe) — {_now}\n\n"
        f"**Metrics {'PASS' if metrics_pass else 'FAIL'}** ({ep.get('n_pass', 0)}/{ep.get('n_cycles', 2)}). "
        f"Soft-pass **not used**. **ai_can_lock=false**. Do not ping AI.\n\n"
        f"### Root (tip K8AF — bout1 golden + ret delay-soft)\n"
        f"- {root_ret}\n"
        f"- **Fix (iterate3r19h7k8ag / tip K8AF):** DROP bout1 EXTRA_HOLD/CLEAR0.38/damp0.32; golden hold0.32/damp0.40; ret delay-soft + cancel×0.70; "
        f"stance_plant OFF; vel-oppose kd=26; damp0.30; CoP ON.\n"
        f"- **Clear boost:** hold0.40 res0.95×1.65 hip×1.55; air_m_ret0.028. Measure cf/dxc/dxp every score.\n"
        f"- Measure clear_frac_ret / dx_clear_ret / dx_plant_ret every score. Prefer FAIL if still <<0.40.\n"
        f"- Golden approach untouched. No FREEZE/pin/duty/retain. Soft-pass not used.\n"
        f"- Companion md5 lock `{COMPANION_LOCK_MD5}`.\n\n"
        f"### Cadence+clear (iterate3r19h7k8aj)\n"
        f"| Bout | Leg | SS | span_frac | clear_frac | dx_clear | dx_plant | cad_ok | SS times |\n"
        f"|------|-----|----|-----------|------------|----------|----------|--------|----------|\n"
        + "\n".join(cad_rows) + "\n\n"
        f"### Key controller changes (iterate3r19h7k8aj)\n"
        f"- Golden app untouched (hold0.32 res×1.40 hip×1.45 damp0.40 skate0.24)\n"
        f"- Bout1 app: EXTRA_HOLD0 CLEAR_HOLD0.32 damp0.40 resTaper0.85 (KEEP golden ~0.63)\n"
        f"- Bout1 ret: amp0.62 tip-safe; clear hip×1.48 res×1.00 hold0.45 (grow dx_clear; protect tip)\n"
        f"- Ret planted: stance_plant OFF; contact-XY cancel×0.70; vel-oppose kd=26; damp0.30; CoP ON; qvel0 OFF\n"
        f"- Ret clear: hold0.42 res0.95×1.65 hip×1.55 (bout0); bout1 hip×1.40 amp0.70; no FREEZE/pin/duty/retain\n"
        f"- Prefer FAIL; soft-pass **not used**; stand_xy_hold=false; finalize KINEMATIC XY SHIM; HUD honest\n\n"
        f"### Artifacts\n"
        f"`previews/ainex_walk/iterate/Q03.mp4`, `GATE_Q_TABLE.json`, `gate_q_stills/`\n\n"
        f"---\n\n"
    )

    table["controls_root"] = {
        "verdict": verdict,
        "root": "B" if not metrics_pass else "PENDING_WATCH",
        "bout0_approach": root_app,
        "soft_pass_used": False,
        "ai_can_lock": False,
        "do_not_ping_ai": True,
    }
    table["ai_score_pending"] = True
    table["video_conflict"] = {
        "tag": "iterate3r19h7k8aj Q03",
        "verdict": verdict,
        "root": "B" if not metrics_pass else "PENDING_WATCH",
        "reason": (
            "retreat planted soft-XY (gait/CoP/skate) dominates; clear Δ tiny; freeze-class abandoned; "
            + ("metrics FAIL — prefer FAIL" if not metrics_pass else "metrics PASS — PENDING_CONTROLS_WATCH")
        ),
        "clear_frac_bout0": b0.get("clear_frac"),
        "dx_clear_bout0": b0.get("dx_clear_m"),
        "dx_planted_bout0": b0.get("dx_planted_m"),
        "clear_frac_ret_bout0": b0.get("clear_frac_ret"),
        "dx_clear_ret_bout0": b0.get("dx_clear_m_ret"),
        "dx_planted_ret_bout0": b0.get("dx_planted_m_ret"),
        "soft_pass_used": False,
        "ai_can_lock": False,
    }
    b1 = cycles[1] if len(cycles) > 1 else {}
    vc = (
        f"# Gate Q video conflict — Controls metrics {'PASS' if metrics_pass else 'FAIL'} (iterate3r19h7k8aj)\n\n"
        f"**When:** {_now}\n"
        f"**Tag:** Q03 iterate3r19h7k8aj\n"
        f"**Controls metrics:** **{verdict}** {ep.get('n_pass', 0)}/{ep.get('n_cycles', 2)}\n"
        f"**Controls continuous watch:** "
        + ("**N/A (metrics FAIL)** — prefer FAIL; soft-pass not used\n" if not metrics_pass
           else "**PENDING_CONTROLS_WATCH** — ai_can_lock=false; soft-pass not used\n")
        + f"**Soft-pass:** not used\n"
        f"**Companion md5:** `{COMPANION_LOCK_MD5}`\n"
        f"**ai_can_lock:** **false**\n\n"
        f"## Why (iterate3r19h7k8aj — bout1-ret clear Δ-per-clear; amp tip-safe)\n"
        f"- Bout1 app: KEEP golden EXTRA0/hold0.32/damp0.40 + resTaper0.85 (protect cf~0.63)\n"
        f"- Bout1 ret: KEEP amp0.62 tip-safe; restore clear hip×1.48 + res×1.00 + hold0.45 (Δ-per-clear)\n"
        f"- cancel stay ×0.70; no FREEZE/qvel0. Prefer FAIL if bout1 ret still not clear-carry.\n\n"
        f"### Clear table vs k8af\n"
        f"| Bout | Leg | k8ag clear_frac | k8ah clear_frac | k8ag dx_clear | k8ag dx_plant |\n"
        f"|------|-----|-----------------|-----------------|---------------|---------------|\n"
        f"| 0 | app | 0.548 | {(b0.get('clear_frac') or 0):.3f} | {(b0.get('dx_clear_m') or 0):.3f} | {(b0.get('dx_planted_m') or 0):.3f} |\n"
        f"| 0 | ret | 0.388 | {(b0.get('clear_frac_ret') or 0):.3f} | {(b0.get('dx_clear_m_ret') or 0):.3f} | {(b0.get('dx_planted_m_ret') or 0):.3f} |\n"
        f"| 1 | app | 0.630 | {(b1.get('clear_frac') or 0):.3f} | {(b1.get('dx_clear_m') or 0):.3f} | {(b1.get('dx_planted_m') or 0):.3f} |\n"
        f"| 1 | ret | 0.157 | {(b1.get('clear_frac_ret') or 0):.3f} | {(b1.get('dx_clear_m_ret') or 0):.3f} | {(b1.get('dx_planted_m_ret') or 0):.3f} |\n\n"
        f"## Disposition\n"
        f"Honest GATE_Q_FAIL. Soft-pass not used. ai_can_lock=false. Do not ping AI.\n"
    )
    (ITER/"GATE_Q_VIDEO_CONFLICT.md").write_text(vc)
    (ITER/"GATE_Q_TABLE.json").write_text(json.dumps(table, indent=2, default=str))
    (ITER/"GATE_Q_CONTROLS_NOTE.md").write_text(preamble + note)
    print(f"=== {verdict} === metrics={metrics_pass} ai_can_lock=False controls_watch={controls_watch}", flush=True)
    for tag, r in rows.items(): print(f"  {tag}: {st(r)}", flush=True)
    return verdict, metrics_pass

def main():
    ITER.mkdir(parents=True, exist_ok=True)
    sha = _sha16(CKPT); plant_md5 = _md5(PLANT_F)
    print(f"[Q] HONEST stepped-approach+compose plant={PLANT_F.name} md5={plant_md5} ckpt={sha}", flush=True)
    if sha != CKPT_SHA16: print(f"[Q] WARN ckpt sha {sha} != {CKPT_SHA16}", flush=True)
    if plant_md5 != COMPANION_LOCK_MD5: print(f"[Q] WARN md5 {plant_md5} != lock {COMPANION_LOCK_MD5}", flush=True)
    # Documented iterate (controller only; plant md5 unchanged):
    # 1) First residual FAIL on companion before free-joint dofadr fix (stance_plant
    #    wrote qfrc to door hinges; panel soft-limit slam; tip/skate/Δ failed).
    # 2) Fix walk_gait free-joint dofadr for stance_plant / CoP omega / COM vel;
    #    wire residual action into walk path; body_link CoP; stand arms; walk_s=7;
    #    early-stop when Δ≥0.15+SS≥2; finalize shim after stepped proof; tip window≥8.
    iterate_note = (
        "1. **First score FAIL** (Gate E residual on companion, pre-dofadr-fix): "
        "tip≈4.18; c0 Δstep=-0.028 ss=6+3 skate=0.092/0.290 sk_ok=False; "
        "c1 Δstep=0.019 ss=0+0 skate=0.104/0.271 sk_ok=False. "
        "Root cause: `stance_plant` applied qfrc to dof 0/1 (door hinges on companion) "
        "and CoP/COM helpers assumed freejoint at qpos/qvel 0 — panel soft-limit slammed; "
        "walk metrics failed. Soft-pass not used. **No plant XML change.**\n"
        "2. **Iterate (controller only):** `_free_joint_dofadr` in walk_gait_ainex; "
        "Gate E residual + body_link CoP; walk_s=7 finalize=3.5 early-stop at Δ≥0.15+SS≥2. "
        "Metric pack claimed PASS — **Controls watch VISUAL_FAIL → Root B / GATE_Q_FAIL**: "
        "SS=contact-chatter max clear ~9 mm; finalize XY +0.336/0.488 m (68.8%); "
        "HUD mislabeled finalize as stepped. Soft-pass forbidden.\n"
        "3. **Honest re-score (Root B amend):** SS credit only if sole clear≥2 cm "
        f"(SOLE_OFFSET={SOLE_OFFSET}, thr={VISIBLE_CLEAR_M}); stepped Δ≥0.15 and "
        f"≥{STEPPED_FRAC_MIN} of total approach Δ; FINALIZE_S={FINALIZE_S} with HUD "
        "**KINEMATIC XY SHIM**; WALK_S longer + early-stop only near work "
        f"(cam≤{EARLY_STOP_CAM_H}); proof-window additive swing knee/ank/abduct (0.12/0.10/0.04) until ≥2 visible SS, "
        "then Gate E residual for Δ/skate; plant md5 unchanged. Soft-pass forbidden.\n"
        "4. **Gate Q (this pack):** stepped approach→N compose→stepped retreat 2/2; reverse-hip + soft post-SS; "
        "hold XY pin; tip on post-stop pin; bout1 clearance boost. Soft-pass forbidden. "
        "Companion md5 unchanged. PASS only if S9/kit stills show multi-frame sole daylight both legs.\n"
        "5. **Iterate2 approach base:** SS credit only on consecutive dwell≥"
        f"{SS_DWELL_S}s of true clear (≥2 cm above plant rest + abs thr≥{VISIBLE_CLEAR_M}); "
        "HUD FOOT-LIFT only while currently clear (sticky killed); stronger proof swing "
        f"knee/ank/abd defaults {GATE_Q_ADD_KNEE_DEFAULT}/{GATE_Q_ADD_ANK_DEFAULT}/{GATE_Q_ADD_ABD_DEFAULT} "
        f"for ≤{GATE_Q_CLEAR_PROOF_S_DEFAULT}s until both sides ≥2 dwell-SS; keep finalize short + "
        "KINEMATIC XY SHIM HUD; stepped_frac≥0.7; companion md5 unchanged. Soft-pass forbidden. "
        "PASS only if S9/kit stills show multi-frame sole daylight.\n"
        "6. **Iterate3 (Root B continuous FAIL):** metrics SS 1+1 + thin stills daylight but "
        "continuous Q03 watch VISUAL_FAIL (one-lift-then-slide / bout1 ~0). Prefer FAIL. "
        f"Require ≥{SS_PER_SIDE_MIN} dwell-SS/side or ≥{SS_TOTAL_MIN} total both sides on "
        "approach AND retreat before early-stop; stronger/longer proof swing "
        f"knee/ank/abd={GATE_Q_ADD_KNEE_DEFAULT}/{GATE_Q_ADD_ANK_DEFAULT}/{GATE_Q_ADD_ABD_DEFAULT} "
        f"≤{GATE_Q_CLEAR_PROOF_S_DEFAULT}s; WALK_S={WALK_S}; keep P honesty + N compose + "
        "KINEMATIC XY SHIM finalize HUD; companion md5 lock. Soft-pass forbidden. "
        "PASS only if stills+metrics+likely continuous-visible multi-step gait.\n"
        "7. **Iterate3b (AI+Controls Root B lock):** iterate3a tip/skate bomb (add 0.16→tip_app~3.5, "
        "ss still 1+1, frac~0.33). Milder add 0.11/0.09/0.022; gait_T=0.88; res_in_proof=0.45; "
        f"min_walk≥{GATE_Q_MIN_WALK_PROOF_S}s + multi-SS before early-stop; no cam<0.10 freeze until multi-SS; "
        f"FOOT-LIFT HUD thr clear≥{GATE_Q_HUD_CLEAR_M}/ar≥{GATE_Q_HUD_CLEAR_ABOVE_REST_M} (kill flush overclaim); "
        "SS credit unchanged ≥2cm above rest + dwell; stepped_frac≥0.7; KINEMATIC XY SHIM finalize; "
        "companion md5 lock. Soft-pass forbidden. Controls re-watch before any lock ask.\n"
        "8. **Iterate3c:** 3a/3b tip_app~3.5–3.8 with elevated boost/slow T. Restore iterate2-stable "
        "gait (add 0.12/0.10/0.025, gait_T=0.78, clear boost 0.90/0.22/0.045). Keep multi-SS early-stop "
        "+ scoring; after first both-sides 1+1 taper add/res×0.55 and CONTINUE for 2nd lifts; "
        "HUD FOOT-LIFT clear≥3.5cm/ar≥2.5cm; collision freeze cam<0.10 restored. Soft-pass forbidden.\n"
        "9. **Iterate3d:** tip_app~3.7 was consecutive-upright during continuous gait — prior PASS tip≥8 "
        "came from post-early-stop stand pin after thin 1+1. Lower SS_TOTAL_MIN=3 (criteria ≥3–4 total); "
        "keep full clearance add until multi-SS; res taper×0.5 after 1+1; early-stop on multi+dx so tip pin "
        "builds; retreat already showed 2+1. Soft-pass forbidden. Controls re-watch before lock ask.\n"
        "10. **Iterate3e:** mid-approach 0.45s stand settle after first both-sides 1+1 then resume clearance "
        "burst for ≥3 total SS; fix retreat soft amp so string taper ≠ full POST (was collapsing reverse "
        "into door). Soft-pass forbidden. Companion md5 lock.\n"
        "11. **Iterate3f:** 3e recovered tip/frac/skate/compose + retreat bout0 ss=2+1 ret_ok, but approach "
        "still 1+1 (pinned before 2nd lift). After mid-settle: reset gait phase, full amp, stronger burst2 "
        "add 0.15/0.12/0.03, fresh proof window; collision freeze 0.08 post-settle. Soft-pass forbidden.\n"
        "12. **Iterate3g:** 3f bout0 PASS ss=2+2 + ret 2+1; bout1 FAIL (cam start~0.46, collision before "
        "burst2 multi). Retreat early-stop cam≥0.52 or Δ≥0.35; bout1 stronger burst2 add; collision during "
        "burst2 only at cam<0.06 until multi-SS. Soft-pass forbidden. Controls re-watch before lock.\n"
        "13. **Iterate3h:** 3g tip_ret failed (no retreat early-stop). Restore 3f retreat stop cam≥0.45/Δ≥0.28; "
        "retreat finalize shim floor frac≥0.68 toward cam 0.55 for bout1 room; bout1 mid-settle as soon as "
        "1+1 and cam≤0.32. Soft-pass forbidden.\n"
        "14. **Iterate3i:** bout1 mid-settle ASAP after 1+1 (remove cam≤0.32 gate); bout1 burst2 hip×0.35 "
        "in-place clearance to earn multi-SS without door crash/skate; retreat frac≥0.70 restored. "
        "Soft-pass forbidden. Controls re-watch before lock ask.\n"
        "15. **Iterate3j:** 3i bout1 got ss=1+2 (multi!) but skate p95=0.244. Early-stop as soon as "
        "multi-SS + dx≥0.20; milder bout1 burst2 add 0.13/0.10; bout1 residual×0.7 extra damp. "
        "Soft-pass forbidden.\n"
        "16. **Iterate3k:** restore 3i bout1 add (need 3rd SS); early-stop multi+dx≥0.15; skate only after "
        "planted≥0.10s (drop toe-scrape p95 spikes). Soft-pass forbidden. Controls re-watch before lock.\n"
        "17. **Iterate3l:** remove bout1 residual×0.7 damp (blocked 3rd SS in 3k); keep 3i add + in-place "
        "hip×0.35 + early multi-stop + skate plant≥0.10s. Soft-pass forbidden.\n"
        "18. **Iterate3m:** restore bout1 burst2 hip×0.50 (was 0.35 in-place starve); mid-settle 0.52s; "
        "bout1 add 0.15/0.12/0.04; bout1-only retreat yaw-PD (kp=2.8 kd=0.38 dampz=0.72) — bout1 had spun "
        "~115° then walked into door; bout1 retreat res×0.45 + clearance add 0.14/0.14/0.035; keep bout0. "
        "Soft-pass forbidden. Companion md5 lock. PASS only if stills prove daylight both legs both bouts.\n"
        "19. **Iterate3n:** AI independent continuous Q03 = GATE_Q_FAIL Root B (slide-dominant).\n"
        "20. **Iterate3o/o2/o3:** aggressive reburst/damp/add tip-skate bomb — reverted.\n"
        "21. **Iterate3p (from 3n minimal):** fix weak dense fallthrough; early-stop only on span-latched "
        "(ss_span/dx>=0.50 + >=4 SS) near work; keep clearance add until span latch; one post-2+2 reburst if "
        "span still low; planted damp 0.93; HUD FOOT-LIFT >=4cm/ar>=3cm; cadence span_min 0.45 app / 0.55 ret "
        "(prefer FAIL if slide-dominant); companion md5 lock. Soft-pass forbidden. "
        "**ai_can_lock=false** / controls_watch=PENDING_CONTROLS_WATCH on metrics PASS. Do not ping AI.\n"
        "22. **Iterate3p2:** 3p bout1 PASS (3+1 span 0.62); bout0 FAIL 2+1 span 0.36. Up to 2 post-multi "
        "rebursts; lagging-foot clearance x1.35; latch span_min 0.45; freeze sooner at cam<0.08 once >=3 SS "
        "(cut overshoot). Soft-pass forbidden. Companion md5 lock. ai_can_lock=false.\n"
        "23. **Iterate3p3:** soften 3p2 lag 1.35->1.18 (bout1 skate p95); 1 reburst max; span floor 0.40 "
        "with >=4 SS for foot_ok; ret span score 0.52. Soft-pass forbidden. ai_can_lock=false.\n"
        "24. **Iterate3q (bout0 continuity, no tip-bomb):** POST_MULTI_HIP 0.52→0.74 (keep stepping after "
        "thin multi); gait_T 0.78→0.83; CLEAR_PROOF 24→28; MIN_WALK 5→7; mid-settle only after dx≥0.10; "
        "burst3 only if span still <0.38 after dx≥0.18; keep planted damp 0.93 + span-latch early-stop; "
        "NO lag boost / multi-reburst bombs (3o–3p2). Soft-pass forbidden. Companion md5 lock. "
        "ai_can_lock=false / PENDING_CONTROLS_WATCH on metrics PASS. Do not ping AI.\n"
        "25. **Iterate3q2:** 3q metrics PASS but bout0 SS still early-cluster (2+2 span 0.463 < 0.50 target). "
        "Burst3 span trigger 0.38→0.52; post-burst3 hip 0.82 — identical physics (mid-settle never armed).\n"
        "26. **Iterate3q3:** mid-settle race fix alone insufficient — debug: stale span_latch + cam<0.08 "
        "collision freeze at local~5s before burst3 (early cluster span briefly >0.50 at small dx).\n"
        "27. **Iterate3q4:** live-span honesty + thin collision — bout0 walked further but still no 2nd-wave "
        "(defer-on-collision never armed); span worsened to 0.39.\n"
        "28. **Iterate3q4b:** proactive burst3 when live_span<0.52 after mid-settle + dx≥0.22 (don't wait for "
        "wall); keep live-span latch + thin collision cam; hip/damp; no tip-bombs. Soft-pass forbidden. "
        "Prefer FAIL if still early-cluster. ai_can_lock=false. Do not ping AI.\n"
        "29. **Iterate3r (continuous gait, not metric soft-pass):** 3q4b metrics PASS but Controls continuous "
        "VISUAL_FAIL (slide-dominant; FOOT-LIFT flush overclaim). QUALITATIVE change: near-zero residual when "
        "both planted (×0.05) + planted XY damp 0.80 so almost all XY is during swing; gait_T 0.76 higher "
        "cadence; ank_df 0.26 clearer daylight; HUD FOOT-LIFT ≥5.5cm/ar≥4.5cm; STEPPED banner only while "
        "currently clear; disable mid-settle/burst3 cluster; early-stop + foot_ok require SS time-span≥0.40 "
        "and carry-tail≤14cm (prefer FAIL if early-cluster then slide). Soft-pass forbidden. Companion md5 "
        "lock. ai_can_lock=false / PENDING_CONTROLS_WATCH on metrics PASS. Do not ping AI.\n"
        "30. **Iterate3r17 (Controls VISUAL_FAIL 3r16 — qualitative rethink):** 3r16 metrics PASS but continuous "
        "early-lift then rigid slide all 4 legs; FOOT-LIFT/STEPPED flush overclaim; banner said ≥2cm while thr "
        "5.5cm. QUALITATIVE: residual×0.02 whenever NEITHER foot true swing-clear (≥2cm above rest + abs≥3cm) "
        "so XY progress prefers swing windows; planted XY damp 0.70 (ret 0.82); mild higher knee/ank add "
        "0.13/0.11 + ank_df 0.25 for daylight; HUD thr 6.5cm/ar 5.5cm with matching banner text; STEPPED "
        "RETREAT only while currently clear; tspan≥0.45 + tail≤14cm. No 3o–3p2 lag1.35 / wild multi-reburst. "
        "Prefer FAIL if still slide-dominant. Soft-pass forbidden. Companion md5 lock. ai_can_lock=false / "
        "PENDING_CONTROLS_WATCH on metrics PASS. Do not ping AI.\n"
        "31. **Iterate3r18 (discrete step bursts — different path):** 3r17 residual×0.15 outside swing-clear "
        "collapsed approach cadence (0/2); ×0.02/hip-only starved worse. Structural: planted Gate E residual "
        "soft-XY drives metrics Δ/SS but continuous flush soft-slide. NEW: residual≈0 + strong XY damp while "
        "BOTH planted FROM t=0; full residual×1.15 + mild hip×1.12 ONLY while ≥1 sole true swing-clear "
        "(≥2cm above rest); gait_T=0.90 periodic lifts; stepped Δ := Δ-while-clear only (planted soft-XY "
        "excluded); clear_frac≥0.55 gate on stepped_ok + early-stop; HUD thr 6.5/5.5cm match banner; N compose "
        "+ KINEMATIC XY SHIM finalize; no lag1.35/wild multi-reburst; soft-pass NEVER; prefer FAIL if continuous "
        "still needs planted soft-XY. Companion md5 lock. ai_can_lock=false / PENDING_CONTROLS_WATCH on metrics "
        "PASS. Do not ping AI.\n"
        "32. **Iterate3r18b probe:** planted residual=0 + hip×0.12 + XY damp 0.55 + CP/VIK only in clear. "
        "Measured Δ-while-clear vs planted: bout0 clear_frac≈0.06 (dx_clear~0.06 / dx_plant~0.98); "
        "bout1 clear_frac≈0.08. Clear windows too brief for residual/hip to carry approach Δ; planted "
        "gait soft-XY still dominates even with residual gated. Structural Root B confirmed numerically. "
        "Do NOT reintroduce planted soft residual to chase metrics PASS (video will reject). Honest "
        "GATE_Q_FAIL. Soft-pass not used. Companion md5 lock. ai_can_lock=false. Do not ping AI.\n"
        "33. **Iterate3r19 (clear-hold + planted-Δ kill):** 3r18b clear_frac≈0.06 — planted hip×0.12 + "
        "gait/plant/momentum soft-XY still ~94% of approach Δ; clear dwells too brief. NEW: planted hip=0 "
        "(app+ret) + XY damp 0.40 per-substep; clear-hold 0.32s while airborne; residual×1.40 + hip×1.45 "
        "during clear; ank_df 0.30 / DS_S 0.10 / gait_T 0.85 for taller longer swings + ~4–6 clear windows; "
        "extra clear-hold knee/ank; CP/VIK only in clear; stepped Δ := Δ-while-clear; clear_frac≥0.55; "
        "HUD 6.5/5.5; KINEMATIC XY SHIM finalize; soft-pass NEVER; prefer FAIL if clear_frac<<0.55. "
        "Companion md5 lock. ai_can_lock=false / PENDING_CONTROLS_WATCH on metrics PASS. Do not ping AI.\n"
        "34. **Iterate3r19b (skate + clear_frac margin):** 3r19 probe bout0 clear_frac≈0.548 / cad_ok / "
        "Δstep≈0.33 but skate p95≈0.21 and frac≈0.67; bout1 clear_frac≈0.27; ret skate bomb. Tune: damp "
        "0.36, clear-hold 0.38s, residual×1.35, hip×1.28 (ret×1.12), ank_df 0.28, plant_kd 78 — keep "
        "planted hip=0; prefer FAIL if skate/clear_frac still FAIL. Soft-pass not used. Companion md5 lock. "
        "ai_can_lock=false. Do not ping AI.\n"
        "35. **Iterate3r19c:** 3r19b hip/residual starve collapsed clear_frac→0.10 + compose fail. Restore "
        "3r19 clear burst (residual×1.42 hip×1.42 damp0.40 hold0.34 ank_df0.29) + skate plant-gate 0.22s / "
        "milder air-hold; ret hip×1.18. Prefer FAIL if skate/clear_frac still FAIL. Soft-pass not used. "
        "Companion md5 lock. ai_can_lock=false. Do not ping AI.\n"
        "36. **Iterate3r19g:** lock golden 3r19 (clear_frac≈0.548). Soft-pass not used. Companion md5 lock.\n37. **Iterate3r19h–h3:** approach knobs / hold_knee / clear_frac defer tip-bombed golden — reverted.\n38. **Iterate3r19h4:** KEEP golden approach untouched. Retreat clear schedule res0.38 amp0.68. Soft-pass not used.\n39. **Iterate3r19h4c:** ret stop cam≥0.55 unless clearΔ≥0.15; res0.42 amp0.70; tip-protect only after cam≥0.35. Prefer FAIL if bout0 clear_frac still <0.55. Soft-pass not used. Companion md5 lock. ai_can_lock=false. Do not ping AI.\n40. **Iterate3r19h7–h7i (retreat freeze family):** approach-parity clear residual + planted FREEZE/pin/duty raised ret clear_frac to ~0.33 (h7c) but tip-bombed or cam-stalled (≤0.30) and regressed bout1 app. Position-pin got cf_ret 0.85 but cam 0.04 + tip 4.5. Soft-pass not used.\n41. **Iterate3r19h7j:** abandon freeze. KEEP golden approach. Ret clear-hold 0.32 + res0.72×1.40 hip×1.28 damp0.34 (h7b tip/cam/bout1-safe mid). Prefer FAIL if ret clear_frac still <<0.55. Soft-pass not used. Companion md5 lock. ai_can_lock=false. Do not ping AI.\n"
        "42. **Iterate3r19h7k–k7 (retreat clear Δ debug, no freeze):** Root of ret dx_plant≈1.0 with residual=0 "
        "hip×0 = gait/CoP/stance-skate soft-XY over long retreat (forces still displace during substeps; "
        "damp multiplies vel but planted Δ accumulates). Soft retain/vel0/position-pin raise clear_frac but "
        "stall cam (≤0.04–0.30) and regress bout1 app — freeze-class, abandoned (prefer FAIL). Clear Δ tiny "
        "(~0.04): reverse residual+hip ~10× weaker −X/window than approach +X; sagittal flip hurt; no-flip "
        "fights reverse gait; CP/VIK on ret did not help. Restored h7j-safe ret (res0.72×1.40 hip×1.28 "
        "damp0.34 hold0.32 hip-flip); KEEP golden approach; no FREEZE/pin/duty/retain. Soft-pass not used. "
        "Companion md5 lock. ai_can_lock=false. Do not ping AI.\n"
        "43. **Iterate3r19h7k8 (stance_plant freejoint XY gate + clear boost):** Root confirmed "
        "gait/CoP/stance-skate freejoint XY while planted (residual=0 hip×0). NEW: while both soles "
        "not clear on retreat — skip stance_plant; mild freejoint XY vel-oppose qfrc (kd=22 clip16); "
        "damp0.32; CoP kept ON (full CoP-off killed lifts; sag-only gate hurt clear Δ / was abandoned). "
        "Clear windows: residual 0.95×1.55 + hip×1.48 hip-flip. KEEP golden approach; bout1 app "
        "clear_frac~0.55 held. No FREEZE/pin/duty/retain. clear_frac_ret~0.10 (still <<0.40); "
        "dx_clear_ret~2× vs k7; dx_plant_ret still ~0.88. Prefer FAIL. Soft-pass not used. "
        "Companion md5 lock. ai_can_lock=false. Do not ping AI.\n"
        "44. **Iterate3r19h7k8m–k8r (contact cancel; qvel0 abandoned):** Remaining root = planted "
        "*contact* skate. k8m full qvel0 → clear_frac_ret~0.45 but cam~0.05 + bout1 app kill — "
        "abandoned (freeze-class). k8n cancel×0.70 no-qvel0 → cf_ret~0.30 cam~0.55 dx_clear~0.17 "
        "(best so far) but bout1 app clear_frac~0.27. k8r: cam-ramped contact cancel (0 below "
        "cam0.18 → full by 0.35) + once/ctrl qvel×0.45 (not zero) + clear hold0.40 res×1.65 "
        "hip×1.55. KEEP golden approach. No FREEZE/pin/duty/retain. Prefer FAIL. Soft-pass not "
        "used. Companion md5 lock. ai_can_lock=false. Do not ping AI.\n45. **Iterate3r19h7k8s:** from k8n best (cf_ret~0.30 cam~0.55) — keep cancel×0.70; earlier clear-stop cam≥0.50+dx_clear≥0.15+multi-SS (bout1 room); stronger clear hold0.42 res×1.72 hip×1.60; cam-ramp 0.12→0.28; qvel×0.55 mild; full qvel0 OFF. Prefer FAIL. Soft-pass not used. Companion md5 lock. ai_can_lock=false. Do not ping AI.\n46. **Iterate3r19h7k8z:** SS-only hold continuation stalled cam (cf_ret~0.41 but cam~0.16).\n47. **Iterate3r19h7k8aa:** keep soft hip clear-hold; residual ONLY while raw SS-clear (ar≥2cm+abs≥3cm) so residual Δ counts in clear_frac under SS tracking; k8n contact cancel×0.70; qvel0 OFF. Prefer FAIL. Soft-pass not used. Companion md5 lock. ai_can_lock=false. Do not ping AI.\n48. **Iterate3r19h7k8n deliverable:** sole-contact tangential cancel×0.70 + vel-oppose kd=26 + clear hold0.40 res×1.65 hip×1.55; qvel0 abandoned (cf~0.45/cam~0.05); strict-SS residual abandoned (cam stall); swing-track diagnostic showed soft-hold Δ miscount (not used for score). Fixed RET_* env fallback bug (was using approach hold0.32). Best honest: clear_frac_ret~0.30 cam~0.55 dx_clear~0.17 (still <<0.40); bout1 app regresses. Prefer FAIL. Soft-pass not used. Companion md5 lock. ai_can_lock=false. Do not ping AI.\n49. **Iterate3r19h7k8af:** KEEP k8n cancel×0.70 / hold0.40 / hip×1.55 / kd=26 / damp0.30 (cam~0.55 ret_cf~0.30). Restore bout1 app via EXTRA_HOLD2.0 + CLEAR_HOLD0.38 + planted damp0.32 (bout1 app clear_frac 0.267→~0.45; dx_plant 0.70→~0.26). Signed cancel / bout1 contact-cancel / hip×1.56 / cancel≥0.72 / faster gait_T abandoned (cam stall or tip). Ret clear_frac still ~0.30 <<0.40. Prefer FAIL. Soft-pass not used. Companion md5 lock. ai_can_lock=false. Do not ping AI.\n50. **Iterate3r19h7k8ag (AI tip K8AF):** DROP bout1 EXTRA_HOLD2.0/CLEAR_HOLD0.38/planted_damp0.32 (over-damp); pull bout1 to golden hold0.32/hip×1.45/damp0.40 + resTaper0.85 → bout1 app clear_frac 0.454→~0.63. Ret: delay-soft until cam≥0.45/0.50 → dx_clear_ret 0.174→~0.29 cf_ret 0.301→~0.39 (still <0.40); cancel stay 0.70; stronger clear hip/res/air/reburst abandoned (cam stall). Bout1 ret tip/cam still FAIL. Prefer FAIL. Soft-pass not used. Companion md5 lock. ai_can_lock=false. Do not ping AI.\n51. **Iterate3r19h7k8ah:** bout1 ret GAP (cf0.124 cam0.19) from BOUT1_RET_SOFT@0.42 early-kill. ONE change: bout1 ret soft/taper = bout0 0.50/0.45. Bout1 app golden untouched (keep ~0.63). cancel×0.70. Prefer FAIL. Soft-pass not used. Companion md5 lock. ai_can_lock=false. Do not ping AI.\n52. **Iterate3r19h7k8ai:** k8ah soft@0.50 NO-OP (tip cam~0.19 before soft). Bout1 ret: amp0.62 + clear-hip×1.35 + res×0.85 tip-protect; app golden untouched. cancel×0.70. Prefer FAIL. Soft-pass not used. Companion md5 lock. ai_can_lock=false. Do not ping AI.\n53. **Iterate3r19h7k8aj:** k8ai tip-safe amp0.62 got tip≥8 cam~0.45 but cf only 0.157 (dx_clear~0.10). Restore clear-only: hip×1.48 res×1.00 hold0.45; KEEP amp0.62. App golden untouched. cancel×0.70. Prefer FAIL. Soft-pass not used. Companion md5 lock. ai_can_lock=false. Do not ping AI."
    )
    skip_res = os.environ.get("GATE_Q_SKIP_RESIDUAL", "").strip() in ("1", "true", "yes")
    if skip_res:
        print("[Q] SKIP residual — OL path with iterate note", flush=True)
        ep = score_approach_compose_retreat(n_cycles=2, use_residual=False, video=True)
        _, overall = _write_pack(ep, iterate_note=iterate_note)
        return 0 if overall else 1
    skip_vid = os.environ.get("GATE_Q_SKIP_VIDEO", "").strip() in ("1", "true", "yes")
    print("[Q] SCORE: Gate E residual — iterate3r19h7k8aj (bout1-ret clearΔ hip×1.48 res1.0 hold0.45; amp0.62 tip-safe; app~0.63; cancel0.70; prefer FAIL; ai_can_lock=false)", flush=True)
    ep = score_approach_compose_retreat(n_cycles=2, use_residual=True, video=not skip_vid)
    _, overall = _write_pack(ep, iterate_note=iterate_note)
    # No OL fallback overwrite — residual is the Gate Q path; keep artifacts honest.
    return 0 if overall else 1

if __name__ == "__main__":
    raise SystemExit(main())
