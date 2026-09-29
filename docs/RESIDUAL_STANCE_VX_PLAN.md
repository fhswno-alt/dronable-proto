**Status: v1 linear residual FALSIFIED** (RES01/RES03). See `RESIDUAL_AB_NOTE.md` + `RESIDUAL_V2_COSPEC.md`.

# Residual / learned stance-vx — sim plan (post Gate E falsifier)

**When:** Sun 27 Sep 2026 ~21:36 BST  
**Owner:** AI (spec) + Controls (integrate)  
**Plant:** M2 145×86 locked. Path A spend FROZEN.  
**Why:** Open-loop CPG+plant+VIK/ZMP hard-falsified for Gate E cadence (T≤0.75 + clean_ss) and dynamic skate-kill.

## Goal
Kill stance skate at T≤0.75 while holding Gate D clear-swing / alt / tip / dx — **sim only**.

## Scope (in)
1. **Residual stance-vx null** on planted foot: estimate stance foot vx from sim state (or joint Jacobian); add small ankle/hip correction **inside HX ±2.1**.
2. Train or fit residual on CSF50→T∈[0.75,0.88] rollouts (privileged sim OK).
3. Policy input: IMU + q + contact flags (no vision; CoP MARGINAL = optional binary only).
4. Deploy as additive term on Controls walker — assist OFF, freeze OFF.
5. Score vs `GATE_E_AI_CRITERIA.md` (same gates). Pass → Gate E reopen; fail → hard falsifier on residual class too.

## Scope (out)
- Path A buy / OEM sole / Orin / NN-first on Pi claim
- Soft-pass MAD/HUD
- More open-loop μ/VIK/T sweeps

## 24–48h deliverable
- Spec + hook point in `walk_gait_ainex.py` / `ss_step_ainex.py` for residual torque/bias
- First residual A/B: CSF50 replay + T=0.75 with residual on/off metrics table
- Dave: no spend ask unless residual also falsifies and we need Path A thaw

## Success
Gate E criteria TRUE at T≤0.75 on M145, continuous video PASS, assist OFF.

## Hook (Controls integrate — concrete)

**File:** `scripts/walk_gait_ainex.py` (ss_step imports same path)

**Do not** retune open-loop `apply_stance_vx_null` / VIK gains as the residual. Those are falsified as a class. Residual is a **separate additive** after gait targets + plant + CP + optional VIK:

```
# after apply_stance_jacobian_vik / apply_stance_vx_null (if any):
apply_residual_stance_vx(model, data, act_idx, phi, bid_lf, bid_rf, obs)
```

**New CLI:** `--residual PATH.npz|none` and `--residual-gain FLOAT` (default 1.0).  
**Flag:** `USE_RESIDUAL_STANCE_VX` set when PATH given.

### `apply_residual_stance_vx` contract
- **When:** foot in contact (`xpos[bid,2] ≤ CONTACT_Z_THR`) and that side is stance (same lat rule as `apply_stance_vx_null`).
- **Obs (sim-privileged OK for A/B):**  
  `[vx_stance, vy_stance, yaw_rate, q_leg(6), dq_leg(6), contact_L, contact_R]`  
  No vision. CoP optional binary only — not required for v1.
- **Out:** Δctrl on `hip_pitch` + `ank_pitch` (and optionally `hip_roll`) **clipped so total ctrl stays in HX ±2.1 / ±0.7**.
- **v1 fit (fast A/B):** linear map `Δ = -K @ obs` fit by least squares on CSF50 rollouts where target is `vx_stance→0` during stance windows; save `residual_stance_vx_v1.npz` (`K`, obs_mean, obs_std).
- **v1 forbidden:** root `xfrc` cheats, assist, freeze, speed governor.

### First A/B (score vs Gate E)
| Tag | T | residual | Expect |
|-----|---|----------|--------|
| RES00 | 0.88 CSF50 | OFF | Gate D floor |
| RES01 | 0.88 CSF50 | ON | must not break clean_ss |
| RES02 | 0.75 | OFF | skate FAIL (baseline) |
| RES03 | 0.75 | ON | Gate E score — pass or honest fail |

Artifacts: `iterate/RES0{0,1,2,3}_*.{json,mp4}` + table in `ITERATE_REPORT.md`.

AI delivers: `scripts/residual_stance_vx.py` (fit + apply helpers) + first `K` from CSF50 logs if Controls dumps a short privileged CSV; else Controls calls fit on existing timeseries.
