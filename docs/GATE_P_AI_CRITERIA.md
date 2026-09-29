# Gate P — stepped approach then leave+disturb compose (AI criteria, sim)

**When:** Mon 28 Sep 2026 ~04:12 BST (honesty amends ~04:56 + ~05:21 BST after Root B FAILs)  
**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (confirm plant-of-record)  
**Prereq:** Gate D–O TRUE (sim); Gate O approach→N compose locked on ±30° companion  
**Plant (P rows):** companion `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` (md5 lock `59cc408eda07037a58f92ad27da045d6`)  
**Walk plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Sim only. No spend. Vision off walk. Not Pi NN-first / not Orin.**

## Goal
On the **current ±30°** plant, prove the kit can **approach with real stepped locomotion** (not base-XY shim alone), enter working stand, then run a **full Gate N compose** in the **same continuous score run**. Escalates Gate O’s documented XY ramp. No hinge-range bump, latch, 90°, walk-through, or room-walk claim.

## Pass (all, assist OFF, freeze OFF, k_auth=1.0)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Start offset | Begin with cam→door **≥0.55 m** **or** approach Δ toward door ≥**0.15 m** before first lever contact (document which) |
| 2 | Stepped approach | During **active walk** (not finalize): ≥**2** visible foot lifts **per bout** with sole clearance ≥**2 cm above plant rest** (or absolute clear thr that cannot trip on soft-contact rest gap alone — **not** contact rising-edge / sticky banner / SOLE_OFFSET bias); each lift **dwells ≥150–250 ms** consecutive clear; S9/kit stills must show multi-frame sole daylight (≥3–5 frames) both sides; stepped base Δ during active walk ≥**0.15 m**; tip≥8 |
| 3 | Skate / honesty / shim budget | Skate mean ≤**0.08**, p95 ≤**0.18**; **no** vision-in-walk; **no** teleport. Finalize/retreat kinematic XY shim allowed only after stepped Δ proved, and stepped Δ must be **≥~70%** of total approach cam/base Δ (finalize must **not** dominate). HUD finalize = **KINEMATIC XY SHIM** only. HUD **FOOT-LIFT/SS only while currently clear** (no sticky hold that outlives re-plant) |
| 4 | Working stand | Enter N working stand (cam→door ∈ **[0.35, 0.45] m** or Controls-documented N pose); tip≥8 |
| 5 | N compose | After approach, one bout satisfies Gate N: open ≥25°; hold ≥1.0 s @ ≥25°; declared disturb; reject panel ≥15° in 2 s; leave ≥0.5 s @ panel ≥20°; re-grasp; close Δ ≥10°; tip≥8 |
| 6 | Multi-bout | **2/2** consecutive **stepped-approach→N-compose** cycles in one continuous run (retreat/recover OK; each cycle must re-approach with steps, not shim-only) |
| 7 | Plant honesty | ±30°; Gate J spring; companion md5 unchanged; **no** hinge-range bump |
| 8 | Base DOF | Free-joint **x,y free** during compose (no Stand XY pin). Same §9 honesty as K |
| 9 | Open-angle visual | Inherit K: free-edge **Δ screen-X ≥~25 px** @ +30°; soft-pass forbidden |
| 10 | Video | Continuous MP4 both cycles; HUD panel hinge + XY; mark approach start, foot-lift/SS events, disturb, leave |

**Hard falsifier:** vision-in-walk; tip&lt;8; M145 edit; panel scripted / panel actuator; claiming latch / 90° / walk-through / UK / full open / room-walk / investor walk script; hinge range bump as “Gate P”; undeclared assist/freeze/xfrc; Stand XY pin on compose; soft-pass; spend; **Gate O–style XY shim alone** / finalize-dominated approach; SS from contact-edge / sticky HUD / SOLE_OFFSET plant-rest bias without multi-frame visible daylight; HUD FOOT-LIFT while soles flush; HUD claiming stepped during finalize; separate approach vs N score runs called P.

**Not Gate P:** Hardware range bump; latch; walk-through doorway; UK handle; Pi detector; sole/STEP; spend; full-room cadence T≤0.75 demo claim.

## Prove tags (Controls)

| Tag | Intent |
|-----|--------|
| **P00** | Offset start + stepped approach (SS/foot-lift count + Δ + skate) |
| **P01** | Enter working stand; begin N open/hold |
| **P02** | Full N compose (disturb + leave + regrasp + close) |
| **P03** | **2/2** consecutive stepped-approach→compose; continuous MP4 |

Artifacts: `GATE_P_CONTROLS_NOTE.md`, `GATE_P_TABLE.json`, P00+ mp4 under `previews/ainex_walk/iterate/`.

## Plant / Hardware
- Companion only. Locked M145 untouched. **No plant change** for Gate P.
- Range bump / latch / walk-through: out of scope (not P; no PO).
- Manufacturing: idle; no spend.

## Owner
- **AI:** this criteria + score/lock
- **Controls:** P00–P03 on companion + frozen ckpt sha16 `9ffaa1a21b607bf6` (prefer Gate E residual for steps; CSF50 OK if tip/skate hold)
- **Hardware:** confirm plant-of-record (no change)
- **Manufacturing:** silent

## Explicit non-claims
Not Pi/Orin/NN-first; not vision-in-walk; **not full door open / latch / walk-through / 90°**; not UK height; no spend; **not** a hinge-range bump; **not** a full-room walk / investor walk script. Soft-pass forbidden. Gate O lock stays shim-approach — P requires **steps**.

## Score
**LOCKED TRUE** (sim) ~05:45 BST — Controls iterate2 + AI stills/`P03.mp4` watch. Soft-pass forbidden.  
Prior packs: (1) Root B FAIL (contact-SS + finalize-dom); (2) Root B FAIL (flush soles / SOLE_OFFSET sticky HUD) — `GATE_P_VIDEO_CONFLICT.md`. Iterate2 resolved. Artifacts: `GATE_P_AI_LOCK.md/.json`. Curriculum D–P TRUE.
