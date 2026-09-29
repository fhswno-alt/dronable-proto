# Gate N — leave + disturb compose (AI criteria, sim)

**When:** Mon 28 Sep 2026 ~03:51 BST  
**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (confirm plant-of-record)  
**Prereq:** Gate D–M TRUE (sim); Gate L leave/re-grasp + Gate M open-hold disturb reject locked on ±30° companion  
**Plant (N rows):** companion `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` (md5 lock `59cc408eda07037a58f92ad27da045d6`)  
**Walk plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Sim only. No spend. Vision off walk. Not Pi NN-first / not Orin.**

## Goal
On the **current ±30°** plant, prove **one continuous controller** can run **both** Gate M disturb-reject **and** Gate L leave/re-grasp **in the same bout** — not as separate scripts — then close. No hinge-range bump, latch, 90°, or walk-through.

## Pass (all, assist OFF, freeze OFF, k_auth=1.0)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Inherit open | Panel abs ≥**25°** within ±30°; tip≥8; coupling honesty (hand→lever→panel→`door_panel_hinge`; no panel actuator / no scripted panel qpos) |
| 2 | Hold + disturb | Hold ≥**1.0 s** @ panel ≥**25°**; one declared disturbance (same honesty as M: document type; ≤0.5 s impulse / ≤1.0 s window) |
| 3 | Reject | After disturb ends: tip≥8; within **2.0 s** panel abs ≥**15°**; no undeclared xfrc / assist / freeze |
| 4 | Leave | After reject (panel still ≥**20°**), **break grasp / leave lever** for ≥**0.5 s** while panel stays ≥**20°**; tip≥8 |
| 5 | Re-grasp + close | Re-establish grasp; close Δ ≥**10°** (or near-closed); tip≥8 |
| 6 | Multi-bout | **2/2** consecutive full sequences in one continuous score run: open→hold→disturb→reject→leave→re-grasp→close |
| 7 | Plant honesty | Stay within ±30°; Gate J spring plant-of-record; **no** hinge-range bump; companion md5 unchanged |
| 8 | Base DOF | Free-joint **x,y free** (no Stand XY pin). Same §9 honesty as K |
| 9 | Open-angle visual | Inherit K: free-edge **Δ screen-X ≥~25 px** @ +30°; soft-pass forbidden |
| 10 | Video | Continuous MP4 both cycles (dual or world+kit); HUD panel hinge + XY; mark disturb onset + leave window |

**Hard falsifier:** vision-in-walk; tip&lt;8; M145 edit; panel scripted / panel actuator; claiming latch / 90° / walk-through / UK / full open; hinge range bump as “Gate N”; undeclared assist/freeze/xfrc; Stand XY pin; soft-pass; spend; proving L and M only in **separate** score runs and calling that N.

**Not Gate N:** Hardware range bump toward 90°; latch; walk-through; UK handle height; Pi detector; sole/STEP; spend; walk→door stitch (later gate).

## Prove tags (Controls)

| Tag | Intent |
|-----|--------|
| **N00** | Open ≥25° + tip≥8 + coupling |
| **N01** | Hold + declared disturb + reject (M thresholds) |
| **N02** | Leave ≥0.5 s @ panel ≥20° + re-grasp + close Δ ≥10° (L thresholds) |
| **N03** | **2/2** consecutive full compose cycles; continuous MP4 |

Artifacts: `GATE_N_CONTROLS_NOTE.md`, `GATE_N_TABLE.json`, N00+ mp4 under `previews/ainex_walk/iterate/`.

## Plant / Hardware
- Companion only. Locked M145 untouched. **No plant change** for Gate N.
- Range bump / latch / walk-through: out of scope (not N; no PO).
- Manufacturing: idle; no spend.

## Owner
- **AI:** this criteria + score/lock
- **Controls:** N00–N03 on companion + frozen ckpt sha16 `9ffaa1a21b607bf6`
- **Hardware:** confirm plant-of-record (no change)
- **Manufacturing:** silent

## Explicit non-claims
Not Pi/Orin/NN-first; not vision-in-walk; **not full door open / latch / walk-through / 90°**; not UK height; no spend; **not** a hinge-range bump; **not** walk→door composition. Soft-pass forbidden. L and M locks stay; N does not reopen them as full-open.

## Score — LOCKED TRUE (~04:00 BST)
Controls metrics PASS 2/2; AI stills confirm disturb + leave both bouts (`gate_n_rewatch/`). Soft-pass not used. See `GATE_N_AI_LOCK.md`.
