# Next architecture — learned / RL Gate E (sim only)

**When:** Sun 27 Sep 2026 ~23:48 BST  
**Why:** Plant×outer co-opt hard-falsified (`PLANT_OUTER_COOPT_NOTE.md`). Classic open-loop CPG family + fixers + authority + contact honesty + plant-fit outer are all dead for Gate E under kit HX.  
**Question:** Can a **state-dependent learned policy** (sim) unlock Gate E at T≤0.75 while holding a T88 floor — where open-loop + linear residual could not?  
**Sim only. No spend talk. No NN-first claim on Pi. No Orin.**

## Locked falsifiers (do not reopen)
| Class | Status |
|-------|--------|
| Open-loop CPG + VIK + μ/T micro-sweep | Gate E FALSE |
| Linear residual / WBC / MPC | HARD-FALSIFIED |
| Dual-T outer (shared M145) + plant×outer (GEO02a/b) | HARD-FALSIFIED |
| HX authority envelope | HARD-FALSIFIED |
| Contact/geometry honesty (frozen outer) | HARD-FALSIFIED |

## Goal
On **locked M145** (`ainex_controls_m2_145.xml`), assist/freeze/xfrc OFF, **k_auth=1.0**:
1. Train a small closed-loop policy that **augments or replaces** open-loop outer outputs.
2. Hold T88-ok floor (periodic eval).
3. Unlock Gate E at T≤0.75 **or** hard-falsify learned class under kit HX on M145.

## Design

| Item | Spec |
|------|------|
| Plant | Locked M145 only (GEO02a co-opt already dead; no plant hop) |
| Authority | HX ±2.1 / kit position clips — **1.0× only** |
| Policy class | Small MLP (or equivalent) → Δqdot / Δτ / residual joint targets; **state-dependent** every control tick |
| Obs | Proprio q, qd; IMU/base ω,v; phase or clock; stance contact flags / foot height — **no vision** |
| Action limits | Strictly inside kit HX clips; log sat_rate |
| Bootstrap | CSF50 or GRO01 open-loop as prior / residual base (not a frozen-only scrub) |
| Algorithm | PPO or SAC (or equal on-policy/off-policy RL) in MuJoCo; optional brief BC warmup from open-loop rollouts |
| Reward (shaped) | Primary: Gate E terms — skate mean/p95, bout, tip, dx, alt, clear; heavy penalty tip/flip/explode; **T88 retention** bonus or hard gate in curriculum |
| Curriculum | Prefer start near T=0.88 clean_ss → anneal T→0.75; do not cold-start T75 only |
| Eval tags | RL00 open-loop baseline T88/T75; RL01+ checkpoints: T88 floor + T75 Gate E; continuous video on any claim |
| Budget | Bounded wall: **≤4 h** box train **or** ≤2e6 env steps (first hit wins stop); early stop on Gate E |
| Pass | Gate E TRUE (`GATE_E_AI_CRITERIA.md`) + continuous video; T88-ok still holds on same weights |
| Hard-falsify | No Gate E after budget with T88 held at least once mid-train → **learned Gate E class HARD-FALSIFIED** under kit HX on M145 |

## Explicit non-claims
- **Not** a Pi deploy path. Not Orin. Not “NN-first gait on kit.”
- **Not** reopening linear residual K-search, WBC QP, or MPC horizon grids.
- **Not** Path/spend / sole fab language.

## Stop rule
If learned policy never clears Gate E after budget → **learned / RL Gate E HARD-FALSIFIED** on M145 under kit HX. Then AI surfaces the **full classic+learned falsifier stack** to Dave (still sim facts only) — next lever needs Dave direction (architecture or plant), not another open-loop GEO pack.

If Gate E unlocks → lock claim **sim learned policy unlocks Gate E**; freeze weights + seed; still no Pi claim until Dave asks for an on-device plan.

## Owner
- **Controls:** train loop, eval table, videos
- **AI:** cospec, Gate E lock criteria, score/lock; reward shape review if Controls asks
- **Hardware / Manufacturing:** silent (no plant/STEP ask)

## Artifacts expected
`LEARNED_GATE_E_NOTE.md`, `LEARNED_GATE_E_TABLE.json`, RL00+ under `previews/ainex_walk/iterate/`; checkpoint path noted in the note.


## Verdict LOCKED 2026-09-28 ~00:58 BST (AI score)

**Gate E UNLOCKED (sim learned residual)** on locked M145 @ k_auth=1.0.

Evidence: `previews/ainex_walk/iterate/LEARNED_GATE_E_NOTE.md`, `LEARNED_GATE_E_TABLE.json`, ckpt `previews/ainex_walk/iterate/learned_gate_e/ppo_gate_e_best.zip`.
- RL04_locked_T75: tip=9.0, dx=+0.360, bout_min=0.120, stx 0.021/0.059, skate=false, hip=-0.510, foot_lead=21, clear L/R 0.0123/0.0124, n_bouts 9/9, assist/freeze OFF → `gate_e` TRUE
- RL04_locked_T88 same weights: t88_ok TRUE (bout≥0.10, no skate); Gate D `clean_walk_ss` not required at T88 for this cospec floor
- RL00 open-loop T75 still FAIL (parity with AUTH01)
- Budget ~1.75e6 steps / ~1.0 h wall

**Explicit non-claim:** sim only — **not** Pi / Orin / NN-first on kit.

**AI continuous-video confirm (28 Sep ~01:00 BST):** `RL04_locked_T75.mp4` PASS — upright, clear alternating SS, continuous articulation, no visible skate/teleport/assist.

## Next (sim / sensing)
Freeze weights. Gate F / Monday perception-compute checklist. No plant/STEP change. No spend talk until Dave asks.
