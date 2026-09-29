# Gate E — falsifier stack + unlock (sim)

**When:** Mon 28 Sep 2026 ~00:58 BST  
**Plant:** locked `ainex_controls_m2_145.xml` (M2 145×86)  
**Sim only. No spend talk. No Pi/Orin NN-first claim.**

## Unlock (locked)
| Claim | Result | Evidence |
|-------|--------|----------|
| Gate E @ T≤0.75 | **TRUE** | `LEARNED_GATE_E_NOTE.md` RL04_locked_T75 |
| Same weights T88-ok | **TRUE** | RL04_locked_T88 |
| Class | PPO MLP [64,64] residual Δctrl on GRO01+CoP+CP+VIK stack | `ppo_gate_e_best.zip` |

## Hard-falsified classes (do not reopen for Gate E)
| Class | When (BST 27 Sep) | Note |
|-------|-------------------|------|
| Open-loop CPG + VIK + μ/T | ~21:34 | Gate E FALSE |
| Linear residual stance-vx | ~22:12 | HARD-FALSIFIED |
| WBC / friction-cone QP | ~22:44 | HARD-FALSIFIED |
| Hybrid MPC + T88 | ~22:53 | HARD-FALSIFIED |
| Dual-T outer (shared M145) | ~23:16 | HARD-FALSIFIED |
| HX authority envelope | ~23:24 | HARD-FALSIFIED |
| Contact/geometry honesty | ~23:36 | HARD-FALSIFIED |
| Plant×outer co-opt (GEO02a/b) | ~23:48 | HARD-FALSIFIED |

## Reading
Classic open-loop + geometry cannot hit kit cadence without skate. A **state-dependent** residual under kit HX 1.0× can. Unlock does **not** thaw spend, sole fab, or Pi deploy.

## Next
Gate F / lever sensing checklist. Reproduce RL04. Continuous video on file for claim.
