# Next architecture — HX authority envelope A/B (sim)

**When:** Sun 27 Sep 2026 ~23:16 BST  
**Why:** Dual-T outer re-opt hard-falsified (`GAIT_REOPT_NOTE.md`). Shared-shape open-loop that holds T88 never reaches no-skate at T≤0.75 (0/53). Residual / WBC / MPC on frozen CSF50 already dead.  
**Question:** Is Gate E unreachable because of **kit HX ±2.1 authority**, or still unreachable even with more torque/position room?  
**Sim only. No spend talk. No sole/plant geometry change yet.**

## Locked falsifiers (do not reopen)
| Class | Status |
|-------|--------|
| Open-loop CPG + VIK + μ/T micro-sweep | Gate E FALSE |
| Residual stance-vx (v1 / v2a / v2b) | HARD-FALSIFIED |
| Stance friction-cone QP / WBC-lite | HARD-FALSIFIED |
| Hybrid short-horizon MPC + T88 | HARD-FALSIFIED |
| Dual-T outer gait re-opt (shared shape, 200 CMA) | HARD-FALSIFIED |

## Goal
On locked M145, with assist/freeze/xfrc OFF:
1. Hold a known T88-ok outer (GRO01 best and/or CSF50).
2. At T=0.75, scale **HX position/torque clips** by multipliers `{1.0, 1.25, 1.5, 2.0, uncapped-or-large}` (sim only).
3. Optional soft A/B: ground μ ∈ {0.8, 1.0, 1.2} at 1.0× HX only (not a sole resize).
4. Score Gate E unlock **or** hard-falsify “more HX authority unlocks Gate E.”

## Design

| Item | Spec |
|------|------|
| Plant XML | Locked `ainex_controls_m2_145.xml` geometry — **do not** swap cadsole / M145 size |
| Outer | GRO01 best shape (primary); CSF50 as control row |
| Authority | Multiply HX torque ±2.1 and/or position ±2.09 by `k_auth`; log clip saturation rate |
| No inner scrub | No residual / WBC / MPC unless a k_auth>1 row first gets Gate E (then optional tiny CP only as honesty) |
| Score tags | AUTH00 T88 GRO01 @ 1.0× (floor must hold); AUTH01+ T75 × k_auth; AUTH_mu* optional |
| Pass | Gate E criteria at some k_auth (honest metrics + video) |
| Hard-falsify this class | No Gate E for any k_auth in the grid (including uncapped) → authority not the unlock; next is contact/geometry honesty with Hardware |

## Stop rule
If Gate E never unlocks across the k_auth grid → **HX-authority-upsized open-loop class HARD-FALSIFIED** for Gate E on M145. Then next lever is **plant/contact honesty** (cadsole / friction height / support polygon) with Hardware — still sim, still no spend talk until Dave asks.

If Gate E unlocks only for k_auth>1 → **lock claim:** kit HX ±2.1 insufficient for Gate E cadence on this plant/gait family (sim evidence). Stop more gait search under 1.0×; document saturation; wait for Dave before any hardware implication language beyond the sim fact.

## Out of scope
More shared-shape CMA; reopening residual/WBC/MPC at 1.0×; Path/spend framing; NN-first on Pi; sole fab.

## Artifacts expected
`AUTH_ENVELOPE_NOTE.md`, `AUTH_ENVELOPE_TABLE.json`, AUTH00+ under `previews/ainex_walk/iterate/`.

## Owner
AI (cospec + score/lock) + Controls (grid + table). Hardware ping only if class falsifies and contact geometry is the next lever.

## Verdict LOCKED 2026-09-27 ~23:24 BST (AI score)

**HX-authority-upsized open-loop HARD-FALSIFIED** for Gate E on locked M145.

Evidence: `previews/ainex_walk/iterate/AUTH_ENVELOPE_NOTE.md`, `AUTH_ENVELOPE_TABLE.json`.
- AUTH00 GRO01 T88 @ 1.0× floor holds
- AUTH01–05 T75 k∈{1.0…10}: sat_rate 0.065→**0** but stx mean/p95 **worsen** (p95 0.64→0.89); Gate E FAIL all
- μ A/B @ k=1 and CSF50 @ k=10: FAIL

**Sim fact locked:** kit HX was binding, but **more Nm does not unlock Gate E** on this plant/gait family. Stop k_auth / μ retune.

## Next architecture (sim only)
Contact / geometry honesty with Hardware. Spec: `docs/CONTACT_GEOMETRY_HONESTY_COSPEC.md`.
