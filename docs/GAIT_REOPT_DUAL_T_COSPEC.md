# Next architecture — dual-T outer gait re-opt (sim)

**When:** Sun 27 Sep 2026 ~22:53 BST  
**Why:** Hybrid MPC+T88 hard-falsified (`MPC_T88_NOTE.md`). Residual, WBC-lite, and inner MPC all show the same knife-edge: **nonzero authority on frozen CSF50 that might cut T=0.75 skate rejects the T=0.88 floor; T88-safe authority does not reach Gate E.**  
**Sim only. No spend talk. No plant change.**

## Locked falsifiers (do not reopen)
| Class | Status |
|-------|--------|
| Open-loop CPG + VIK + μ/T micro-sweep | Gate E FALSE |
| Residual stance-vx (v1 / v2a / v2b) | HARD-FALSIFIED |
| Stance friction-cone QP / WBC-lite | HARD-FALSIFIED |
| Hybrid short-horizon MPC + T88 in-loop | HARD-FALSIFIED |

## Diagnosis (locked pattern)
CSF50 Gate D is a **knife-edge basin** (bout≈0.117). Inner scrubbers / residuals / WBC / MPC on a **frozen** outer schedule cannot expand that basin while speeding. Next lever must **change the outer contact schedule and CoM reference**, not add another Δctrl on CSF50.

## Goal
Find one outer gait (foot timing + placement + CoM/CP reference) that:
1. **Preserves** clean_ss at T=0.88 (or better — bout≥0.12, stx p95≤0.18, no-skate), **and**
2. **Unlocks** Gate E at T≤0.75  
— or hard-falsify this class under HX ±2.1. Assist / freeze / xfrc OFF.

## Design (AI formulation + Controls integrate)

| Item | Spec |
|------|------|
| Plant | Locked `ainex_controls_m2_145.xml`; cadsole honesty only after M145 win |
| Warm start | CSF50 params (T=0.88 ds/hip_amp/step/shift/plant_kd/CP+VIK) — **not** frozen outer |
| Decision vars | Contact schedule (T, ds fraction, SS/DS duty), step length / lateral shift, CoM/CP height & lateral offset, optional swing clearance target — **jointly** optimized |
| Method | Black-box / CMA-ES (or short TO) on dual-rate rollouts: score **both** T=0.88 and T=0.75 from the **same** parameter vector (or a continuous T-schedule with shared shape) |
| Hard constraints | T88: tip≥8, bout_min≥0.12, stx p95≤0.18, skate=false, hip_corr≤−0.45; reject any candidate that fails T88 |
| Soft / Gate E | At T≤0.75: minimize stx mean+p95; require bout≥0.10, ≥5 SS/side, dx≥0.15, clear≥0.012, tip≥8 |
| Authority | Open-loop / scheduled references only for this class — **no** additive residual / WBC / MPC scrub on top until a dual-T outer wins |
| Out | More N/u_max/μ on frozen CSF50; plant resize; Path/spend; NN-first on Pi |

## Score table (authoritative)
| Tag | What |
|-----|------|
| GRO00 | CSF50 T88 baseline (must still hold) |
| GRO01 | Best dual-T candidate at T=0.88 |
| GRO02 | Same candidate at T=0.75 |
| GRO03+ | A/B on key vars (ds, step, shift) |

Gate E unlock **or** hard-falsify. Continuous video on any claim.

## Stop rule
If dual-T outer re-opt cannot hold T88 **and** reach Gate E after a bounded search (e.g. ≥200 CMA evals or equivalent TO budget) → **outer gait re-opt class HARD-FALSIFIED** on locked M145 under HX. Next is a different architecture again (still sim) — not more CSF50 micro-sweeps.

## Artifacts expected
`GAIT_REOPT_NOTE.md`, `GAIT_REOPT_TABLE.json`, GRO00+ under `previews/ainex_walk/iterate/`.

## Owner
AI (cospec + score/lock) + Controls (search + table).

## Verdict LOCKED 2026-09-27 ~23:16 BST (AI score)

**Dual-T outer gait re-opt (shared shape) under HX ±2.1 HARD-FALSIFIED** for Gate E on locked M145.

Evidence: `previews/ainex_walk/iterate/GAIT_REOPT_NOTE.md`, `GAIT_REOPT_TABLE.json`, `GAIT_REOPT_CMA_META.json`.
- 200 CMA evals; n_gate_e=0; n_t75_noskate ∩ T88-ok = **0**
- GRO00 CSF50 T88 floor holds; GRO01 best T88 **strong** (bout=0.120, p95=0.173)
- GRO02 same shape T75: stx mean=0.121 / p95=0.638 skate → Gate E FAIL
- Quietest T75 among T88-ok ≈0.109 mean (still >0.08) with skate

Stop rule fired. **Not more CSF50 / shared-shape CMA micro-sweeps.** Inner scrubbers stay closed.

## Next architecture (sim only)
HX **authority envelope** A/B — falsify whether Gate E is unreachable under kit HX ±2.1 vs needs more torque/position authority. Spec: `docs/AUTH_ENVELOPE_HX_COSPEC.md`.
