# Gate L — leave / re-grasp while holding panel (AI criteria, sim)

**When:** Mon 28 Sep 2026 ~03:25 BST  
**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (confirm plant-of-record)  
**Prereq:** Gate D–K TRUE (sim); Gate K multi-bout open/close locked on ±30° companion  
**Plant (L rows):** companion `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` (edge-stripe visual OK; md5 may match K striped plant)  
**Walk plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Sim only. No spend. Vision off walk. Not Pi NN-first / not Orin.**

## Goal
On the **current ±30°** plant, prove the kit can **open**, **release the lever**, **return and re-grasp**, then **finish a controlled close** — without bumping hinge range, latch, 90°, or walk-through. This is the leave/re-grasp reliability step deferred out of Gate K.

## Pass (all, assist OFF, freeze OFF, k_auth=1.0)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Inherit K | Retain K-class open: panel abs ≥**25°** within ±30°; tip≥8; coupling honesty (hand→lever→panel→`door_panel_hinge`; no panel actuator / no scripted panel qpos) |
| 2 | Leave | After open hold ≥**0.5 s**, **break grasp / leave lever contact** for ≥**0.5 s** while panel stays ≥**20°** (spring may drift; no tip) |
| 3 | Re-grasp | Return hand to lever and **re-establish grasp** (same honesty as H/K); tip≥8 |
| 4 | Close | From re-grasp, close Δ ≥**10°** (or to near-closed) without tip / assist |
| 5 | Multi-cycle | **2** consecutive leave→re-grasp→close cycles in one continuous score run (**2/2**) |
| 6 | Plant honesty | Stay within ±30°; Gate J spring plant-of-record; **no** hinge-range bump |
| 7 | Base DOF | Free-joint **x,y free** (no Stand XY pin). Same §9 honesty as K |
| 8 | Open-angle visual | Same K criterion: free-edge **Δ screen-X ≥~25 px** @ +30° (stills and/or continuous); soft-pass forbidden; do not judge “lip lean” |
| 9 | Video | Continuous MP4 covering both cycles (dual or world+kit); HUD panel hinge + XY |

**Hard falsifier:** vision-in-walk; tip&lt;8; M145 edit; panel scripted / panel actuator; claiming latch / 90° / walk-through / UK / full open; hinge range bump as “Gate L”; assist/freeze/xfrc; Stand XY pin; soft-pass of open-angle FAIL; spend.

**Not Gate L:** Hardware range bump toward 90°; latch; walk-through; UK handle height; Pi detector; sole/STEP; spend.

## Prove tags (Controls)

| Tag | Intent |
|-----|--------|
| **L00** | Open ≥25° + tip≥8 + coupling (K retain) |
| **L01** | Leave contact ≥0.5 s with panel ≥20°; no tip |
| **L02** | Re-grasp + close Δ ≥10°; tip≥8 |
| **L03** | **2/2** consecutive leave→re-grasp→close cycles; continuous MP4 |

Artifacts: `GATE_L_CONTROLS_NOTE.md`, `GATE_L_TABLE.json`, L00+ mp4 under `previews/ainex_walk/iterate/`.

## Plant / Hardware
- Companion only. Locked M145 untouched.
- **No plant change** for Gate L — confirm ±30° + Gate J spring + optional free-edge stripe remain plant-of-record.
- **Range bump:** out of scope until AI explicitly opens a later gate (not L; no PO).
- Manufacturing: idle; no spend.

## Owner
- **AI:** this criteria + score/lock
- **Controls:** L00–L03 prove on companion + frozen ckpt sha16 `9ffaa1a21b607bf6`
- **Hardware:** confirm plant-of-record (no change)
- **Manufacturing:** silent

## Explicit non-claims
Not Pi/Orin/NN-first; not vision-in-walk; **not full door open / latch / walk-through / 90°**; not UK height; no spend; **not** a hinge-range bump. Gate K lock stays multi-bout on ±30° — L does not reopen K as full-open.

## Score — LOCKED TRUE (~03:36 BST)
Reconcile root **A** (watch false FAIL). L00–L03 PASS 2/2. Soft-pass not used. See `GATE_L_AI_LOCK.md`, `GATE_L_VIDEO_CONFLICT.md`.
