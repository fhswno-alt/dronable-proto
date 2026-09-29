# Residual v2 co-spec — after linear v1 falsifier

**When:** Sun 27 Sep 2026 ~21:47 BST  
**Evidence:** `previews/ainex_walk/iterate/RESIDUAL_AB_NOTE.md`  
**Locked:** linear K v1 (`residual_stance_vx_v1.npz`) **FALSE** for Gate E; also breaks CSF50 p95 (RES01). Cadsole honesty: no unlock, no plant change.

## Why v1 failed
Teacher was open-loop `−k·vx` (same class as falsified `apply_stance_vx_null`). Fit reproduced a known-dead lever; at T=0.75 stx got **worse**.

## v2 goal (sim only)
Unlock Gate E on M145 **or** hard-falsify richer residual under HX ±2.1. No spend talk.

## v2 design (AI + Controls)

| Item | Spec |
|------|------|
| Apply window | **SS stance only** (contact + peer swing); **zero** in early DS / swing |
| Plant | Locked M145 first; cadsole honesty only after a M145 win |
| Authority | Additive hip_pitch + ank_pitch (+ optional hip_roll), HX clip; **no** xfrc / assist / freeze |
| Obs | `[vx, vy, com_vx, yaw_rate, phi, sinφ, cosφ, q_leg(6), dq_leg(6), cL, cR]` — vision off; CoP optional binary |
| Policy | **v2a** phase-gated linear K (separate K_ss); **v2b** if v2a fails: small MLP (≤2×64) residual |
| Fit / train | **Not** imitate open-loop vnull. Black-box / CMA-ES or short REINFORCE on T∈{0.75,0.80,0.88} rollouts |
| Objective | Minimize `0.5·stx_mean + 0.5·stx_p95` subject to tip≥8, dx≥0.05, bout≥0.10, hip_corr≤−0.45; **hard reject** if RES00-class T=0.88 clean_ss regresses (p95>0.18 or bout≪0.12) |
| Score | Same RES00–03 table + Gate E criteria; continuous video on any claim |

## Stop rule
If v2a+v2b both fail Gate E with honest metrics → residual class under HX clips **hard-falsified** for cadence; next is different control architecture (still sim), not more linear K.

## Artifacts
`residual_stance_vx_v2a.npz` / `v2b.pt` + `RES_V2_*` table in iterate/.

## Verdict LOCKED 2026-09-27 ~22:12 BST (AI score)

**Residual class under HX ±2.1 HARD-FALSIFIED** for Gate E cadence on M145.

Evidence: `previews/ainex_walk/iterate/RESIDUAL_V2_NOTE.md`, `RES_V2_TABLE.json`.
- v2a T88-safe (RES11/13): floor OK; T75 stx~0.20 / p95~0.74 → Gate E FAIL
- v2a aggressive (RES14/15): tip collapse + T88 hard-reject
- v2b ELM (RES16/17): Gate E FAIL + T88 hard-reject
- Cadsole skipped (no M145 win) — correct

Stop rule fired. **No more residual K / ELM search.**

## Next architecture (sim only)
Whole-body / friction-cone **stance QP** (or WBC) that sets foot wrenches inside μ cone and solves joint cmds under HX — not additive residual on open-loop CPG. Spec: `docs/WBC_STANCE_QP_PLAN.md` (AI next).
