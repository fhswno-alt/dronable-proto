# Next architecture — stance WBC / friction-cone QP (sim)

**When:** Sun 27 Sep 2026 ~22:12 BST  
**Why:** Residual class (v1 linear + v2a CMA + v2b ELM) hard-falsified for Gate E on M145 under HX.  
**Sim only. No spend talk.**

## Goal
Replace additive residual-on-CPG with a **stance wrench QP**:
1. Estimate desired CoM / CP from current gait phase (or keep CP swing).
2. Solve foot wrenches in friction cone (μ from plant; start 0.6–1.0 A/B).
3. Map wrenches → joint torques/position trim via Jacobian under HX ±2.1 / ±0.7.
4. Score vs Gate E; hard-falsify if still unreachable.

## Out of scope
More residual K/MLP search; plant resize; spend/Path labels.

## Owner
AI (QP formulation + metrics) + Controls (integrate in walker, A/B table).

## Verdict LOCKED 2026-09-27 ~22:44 BST (AI score)

**Stance friction-cone QP / WBC-lite under HX ±2.1 HARD-FALSIFIED** for Gate E cadence on locked M145.

Evidence: `previews/ainex_walk/iterate/WBC_STANCE_QP_NOTE.md`, `WBC_STANCE_QP_TABLE.json`, `WBC_MILD_SCAN.json` (n_safe=0).
- WBC00 T88 OFF: floor holds (knife-edge bout≈0.117, no-skate)
- WBC01/08 T88 ON: **hard-reject** (p95≫0.18, bout≪0.12)
- WBC03–07 T75 ON μ/scrub/fric A/B: tip OK, dx up, stx mean ≳0.176 / p95 ≳0.80 → Gate E FAIL

Stop rule fired. **Not more μ/scrub/gain retune.** Open-loop CPG+VIK, residual, and WBC-lite all falsified for Gate E under HX.

## Next architecture (sim only)
Hybrid short-horizon MPC with **T88 clean_ss hard constraint in-loop** (optional learned tracker). Spec: `docs/HYBRID_MPC_T88_COSPEC.md`.
