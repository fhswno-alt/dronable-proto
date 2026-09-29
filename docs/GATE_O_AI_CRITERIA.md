# Gate O — approach then leave+disturb compose (AI criteria, sim)

**When:** Mon 28 Sep 2026 ~04:02 BST  
**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (confirm plant-of-record)  
**Prereq:** Gate D–N TRUE (sim); Gate N leave+disturb compose locked on ±30° companion  
**Plant (O rows):** companion `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` (md5 lock `59cc408eda07037a58f92ad27da045d6`)  
**Walk plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Sim only. No spend. Vision off walk. Not Pi NN-first / not Orin.**

## Goal
On the **current ±30°** plant, prove the kit can **start outside the working stand**, **approach into lever workspace**, then run a **full Gate N compose** (open→hold→disturb→reject→leave→re-grasp→close) in the **same continuous score run**. No hinge-range bump, latch, 90°, room walk-through, or plant invent.

## Pass (all, assist OFF, freeze OFF, k_auth=1.0)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Start offset | Begin with cam→door **≥0.55 m** **or** base start such that approach Δ toward door ≥**0.15 m** before first lever contact (document which) |
| 2 | Approach | Tip≥8 during approach; skate mean ≤**0.08** if stepping; enter working stand (cam→door ∈ **[0.35, 0.45] m** or Controls-documented N working pose) without assist / freeze / XY pin |
| 3 | Look (optional retain) | `head_tilt` look-down allowed; **no** vision feature in walk residual / PPO obs |
| 4 | N compose | After approach, one bout must satisfy Gate N thresholds: open ≥25°; hold ≥1.0 s @ ≥25°; declared disturb; reject panel ≥15° in 2 s; leave ≥0.5 s @ panel ≥20°; re-grasp; close Δ ≥10°; tip≥8 |
| 5 | Multi-bout | **2/2** consecutive **approach→N-compose** cycles in one continuous score run (reset/recover between cycles OK; each cycle must re-approach from offset, not teleport into grasp) |
| 6 | Plant honesty | ±30°; Gate J spring; companion md5 unchanged; **no** hinge-range bump |
| 7 | Base DOF | Free-joint **x,y free** (no Stand XY pin). Same §9 honesty as K |
| 8 | Open-angle visual | Inherit K: free-edge **Δ screen-X ≥~25 px** @ +30°; soft-pass forbidden |
| 9 | Video | Continuous MP4 covering approach + full compose both cycles; HUD panel hinge + XY; mark approach start, disturb, leave |

**Hard falsifier:** vision-in-walk; tip&lt;8; M145 edit; panel scripted / panel actuator; claiming latch / 90° / walk-through / UK / full open / room walk demo; hinge range bump as “Gate O”; undeclared assist/freeze/xfrc; Stand XY pin; soft-pass; spend; teleport into working pose without measured approach Δ; proving approach and N only in **separate** score runs and calling that O.

**Not Gate O:** Hardware range bump; latch; walk-through doorway; UK handle height; Pi detector; sole/STEP; spend; Gate E cadence T≤0.75 room walk claim.

## Prove tags (Controls)

| Tag | Intent |
|-----|--------|
| **O00** | Start offset + approach Δ ≥0.15 m (or cam 0.55→working); tip≥8 |
| **O01** | Enter working stand; begin N open/hold |
| **O02** | Full N compose (disturb + leave + regrasp + close) |
| **O03** | **2/2** consecutive approach→compose cycles; continuous MP4 |

Artifacts: `GATE_O_CONTROLS_NOTE.md`, `GATE_O_TABLE.json`, O00+ mp4 under `previews/ainex_walk/iterate/`.

## Plant / Hardware
- Companion only. Locked M145 untouched. **No plant change** for Gate O.
- Range bump / latch / walk-through: out of scope (not O; no PO).
- Manufacturing: idle; no spend.

## Owner
- **AI:** this criteria + score/lock
- **Controls:** O00–O03 on companion + frozen ckpt sha16 `9ffaa1a21b607bf6` (approach may use Gate E residual or documented shim; no vision-in-walk)
- **Hardware:** confirm plant-of-record (no change)
- **Manufacturing:** silent

## Explicit non-claims
Not Pi/Orin/NN-first; not vision-in-walk; **not full door open / latch / walk-through / 90°**; not UK height; no spend; **not** a hinge-range bump; **not** a full-room walk demo / investor walk script. Soft-pass forbidden. N lock stays compose-at-stand — O adds approach stitch only.

## Score — LOCKED TRUE (~04:10 BST)
Controls metrics PASS 2/2; AI stills confirm approach + disturb + leave both bouts (`gate_o_rewatch/`). Soft-pass not used. See `GATE_O_AI_LOCK.md`.
