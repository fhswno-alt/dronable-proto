# Next architecture — plant-specific outer co-opt (sim)

**When:** Sun 27 Sep 2026 ~23:36 BST  
**Why:** Contact/geometry honesty hard-falsified (`CONTACT_GEOMETRY_NOTE.md`). All GEO00–03 fail Gate E under **frozen** GRO01/CSF50 + kit HX. GEO02a (160×90) showed the mildest T75 stx (mean≃0.093 / p95≃0.41 vs GEO00 ≃0.12 / 0.64) but still skate. Dual-T CMA was **shared shape on locked M145 only** — not a plant-fit outer.  
**Question:** Does a **bounded outer CMA on GEO02a** (and optional GEO02b) unlock Gate E while holding T88, or is plant×outer co-search also dead?  
**Sim only. No spend talk.**

## Locked falsifiers (do not reopen)
| Class | Status |
|-------|--------|
| Open-loop CPG + VIK + μ/T micro-sweep | Gate E FALSE |
| Residual / WBC / MPC | HARD-FALSIFIED |
| Dual-T outer re-opt (**shared shape, locked M145**) | HARD-FALSIFIED |
| HX-authority-upsized open-loop | HARD-FALSIFIED |
| Contact/geometry honesty (**frozen outer**) | HARD-FALSIFIED |

## Goal
On **GEO02a** primary (`ainex_controls_m2_160x90.xml`), assist/freeze/xfrc OFF, **k_auth=1.0 only**:
1. Hold T88 `clean_ss` / T88-ok (bout≥0.10, no skate, tip≥8).
2. Unlock Gate E at T≤0.75 (same AI criteria) **or** hard-falsify plant×outer co-opt.
3. Optional parallel: GEO02b (155×86) same budget if GEO02a fails early; GEO00 control row only if needed to prove script parity.

## Design

| Item | Spec |
|------|------|
| Plant | **GEO02a** primary; optional GEO02b; **not** cadsole/soft (those broke T88) |
| Authority | k_auth=1.0 only — authority class closed |
| Inner scrub | **None** — residual/WBC/MPC stay closed |
| Decision vars | Same dual-T family: T, ds, hip_amp, com_shift(+lead), com_z, hip_bias, knee_*, plant_kd, vik_*, stance_sweep, swing_ank_df — **jointly** on this plant |
| Seed | GRO01 params; optional CSF50 seed row |
| Budget | ≤**200** CMA evals (or equal) on GEO02a; stop early on Gate E win |
| Hard constraint | T88-ok must hold for any T75 claim |
| Soft / Gate E | T≤0.75: minimize stx mean+p95; bout≥0.10; ≥5 SS/side; dx≥0.15; clear≥0.012; tip≥8; skate false |
| Score tags | POC00 GEO00 GRO01 T88/T75 (parity); POC01+ GEO02a CMA best T88 + T75; optional POC_b* GEO02b |
| Pass | Gate E TRUE on GEO02a (or GEO02b) with continuous video |
| Hard-falsify | No Gate E after budget → **plant×outer co-opt HARD-FALSIFIED**; classic open-loop+geometry family exhausted |

## Stop rule
If GEO02a (and optional GEO02b) CMA cannot hold T88 **and** clear Gate E → **plant-specific outer co-opt HARD-FALSIFIED**. Then next architecture is a **new control class in sim** (learned / RL Gate E policy — still no NN-first claim on Pi) — not more frozen-outer GEO packs, k_auth, or inner scrubbers on CSF50.

If Gate E unlocks on GEO02a/b → **lock plant claim** (which XML); freeze that plant for further work; Manufacturing STEP refresh only after Dave asks; still no spend talk.

## Out of scope
Reopening residual/WBC/MPC/authority; more GEO01/03 vertical/soft; Path/spend framing; Pi NN gait; sole fab.

## Owner
- **Controls:** CMA + score table + videos
- **AI:** cospec + score/lock
- **Hardware:** silent unless a winning plant needs another honesty variant
- **Manufacturing:** STEP still frozen at 145×86 until a GEO/POC plant wins **and** Dave asks

## Artifacts expected
`PLANT_OUTER_COOPT_NOTE.md`, `PLANT_OUTER_COOPT_TABLE.json`, POC00+ under `previews/ainex_walk/iterate/`.

## Verdict LOCKED 2026-09-27 ~23:48 BST (AI score)

**Plant×outer co-opt HARD-FALSIFIED** for Gate E.

Evidence: `previews/ainex_walk/iterate/PLANT_OUTER_COOPT_NOTE.md`, `PLANT_OUTER_COOPT_TABLE.json`.
- POC00 GEO00 ≡ AUTH01 FAIL
- GEO02a 200 CMA: T88-ok hist=93, **Gate E hist=0**; best T75 still skate (stx≃0.112 / p95≃0.38) despite dx≥0.15
- GEO02b transfer + short CMA: no unlock
- Classic open-loop + geometry family **exhausted**

Do not reopen frozen-outer GEO packs, k_auth, or residual/WBC/MPC.

## Next architecture (sim only)
Learned / RL Gate E policy — new control class. Spec: `docs/LEARNED_GATE_E_SIM_COSPEC.md`. Still **no** NN-first claim on Pi / no Orin.
