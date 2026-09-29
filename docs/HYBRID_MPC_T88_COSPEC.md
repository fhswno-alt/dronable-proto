# Next architecture — hybrid MPC with T88 in-loop (sim)

**When:** Sun 27 Sep 2026 ~22:44 BST  
**Why:** WBC-lite / stance friction-cone QP hard-falsified for Gate E on M145 (`WBC_STANCE_QP_NOTE.md`). Same T88-safe vs skate pattern as residual.  
**Sim only. No spend talk. No plant change.**

## Locked falsifiers (do not reopen)
| Class | Status |
|-------|--------|
| Open-loop CPG + VIK + μ/T micro-sweep | Gate E FALSE |
| Residual stance-vx (v1 linear, v2a CMA, v2b ELM) | HARD-FALSIFIED |
| Stance friction-cone QP / WBC-lite under HX | HARD-FALSIFIED |

## Goal
Unlock Gate E on locked M145 **or** hard-falsify this control class under HX ±2.1 / ±0.7. Assist / freeze / xfrc OFF for claims.

## Design (AI formulation + Controls integrate)

| Item | Spec |
|------|------|
| Plant | Locked `ainex_controls_m2_145.xml` only; cadsole honesty only after a M145 win |
| Outer | Keep phase / foot timing from CSF50 gait (or short schedule); **do not** reopen open-loop cadence alone |
| Inner | Short-horizon **hybrid MPC** (N≈8–20 @ control rate): CoM/CP (or COM + foot wrench) as decision vars |
| Hard constraint | **T88 CSF50 floor in-loop**: any candidate that would regress T=0.88 clean_ss (stx p95>0.18 **or** bout_min≪0.12 **or** skate) is **rejected** before apply — same hard-reject as residual/WBC tables |
| Soft cost | Minimize predicted stance \|vx\| mean+p95 at T∈{0.75,0.80}; tip / dx / hip_corr / clear as soft barriers |
| Authority | Joint position/torque trim under HX; friction cone optional as soft constraint only (WBC class already dead as primary) |
| Learned track (optional sibling) | If open reference needed: small tracker (≤2×64) on tracking error **inside** MPC cost — not free residual on CPG. Train only with T88 hard-reject in the rollout filter |
| Score | WBC00-style table: T88 OFF baseline, T88 ON (must not reject), T75 OFF, T75 ON A/B; Gate E criteria; continuous video on any claim |
| Mild scan | Same stop pattern: if n_safe=0 at T88 for nonzero controller → class hard-falsify |

## Stop rule
If hybrid MPC (± optional learned tracker) cannot hold T88 floor **and** reach Gate E at T≤0.75 with honest metrics → **this class HARD-FALSIFIED**. Next is a different architecture again (still sim) — not more horizon/μ retune of the same QP/WBC pattern.

## Out of scope
More residual K/ELM; more WBC μ/scrub sweeps; plant resize; Path/spend framing; NN-first on Pi / Orin claims.

## Artifacts expected
`MPC_T88_NOTE.md`, `MPC_T88_TABLE.json`, mild scan JSON, MPC00+ mp4/json under `previews/ainex_walk/iterate/`.

## Owner
AI (cospec + score/lock) + Controls (implement + A/B table).

## Verdict LOCKED 2026-09-27 ~22:53 BST (AI score)

**Hybrid short-horizon MPC (+ T88 hard constraint) under HX ±2.1 HARD-FALSIFIED** for Gate E cadence on locked M145.

Evidence: `previews/ainex_walk/iterate/MPC_T88_NOTE.md`, `MPC_T88_TABLE.json`, `MPC_MILD_SCAN.json` (n_safe=0/36).
- MPC00 T88 OFF: floor holds
- MPC01 schedule-disable: identical to OFF (authority zero at T88)
- MPC02 forced ON at T88: **hard-reject**
- MPC04–06 T75 ON N/u A/B/C: stx mean ≳0.197 / p95 ≳0.77 → Gate E FAIL
- Optional ≤2×64 tracker not required — stop rule already met

Stop rule fired. **Not more horizon / scrub / u_max retune of the same inner loop on frozen CSF50.**

## Next architecture (sim only)
Joint **outer** footstep + CoM / contact-schedule replan (CSF50 = warm start only). Spec: `docs/GAIT_REOPT_DUAL_T_COSPEC.md`.
