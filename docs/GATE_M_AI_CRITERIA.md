# Gate M — open-hold disturbance reject (AI criteria, sim)

**When:** Mon 28 Sep 2026 ~03:37 BST  
**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (confirm plant-of-record)  
**Prereq:** Gate D–L TRUE (sim); Gate L leave/re-grasp locked on ±30° companion  
**Plant (M rows):** companion `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` (md5 may match K/L striped plant `59cc408e…`)  
**Walk plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Sim only. No spend. Vision off walk. Not Pi NN-first / not Orin.**

## Goal
On the **current ±30°** plant, prove the kit can **hold an open panel**, take a **brief disturbance**, **stay upright and keep the panel usefully open**, then **close** — without hinge-range bump, latch, 90°, or walk-through.

## Pass (all, assist OFF, freeze OFF, k_auth=1.0)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Inherit L/K open | Open panel abs ≥**25°** within ±30°; tip≥8; coupling honesty (hand→lever→panel→`door_panel_hinge`; no panel actuator / no scripted panel qpos) |
| 2 | Hold open | Hold ≥**1.0 s** with panel ≥**25°** before disturbance |
| 3 | Disturbance | Apply one brief disturbance during hold (Controls choose: small panel push **or** free-base nudge; document which). Duration ≤**0.5 s** impulse / ≤**1.0 s** applied window |
| 4 | Reject | After disturbance ends: tip≥8 continuous; within **2.0 s** panel abs ≥**15°** (may dip then recover); no assist / freeze / xfrc cheat beyond the declared disturbance |
| 5 | Close | Controlled close Δ ≥**10°** (or to near-closed) after reject; tip≥8 |
| 6 | Multi-bout | **2/2** consecutive open→hold→disturb→reject→close cycles in one continuous score run |
| 7 | Plant honesty | Stay within ±30°; Gate J spring plant-of-record; **no** hinge-range bump |
| 8 | Base DOF | Free-joint **x,y free** (no Stand XY pin). Same §9 honesty as K |
| 9 | Open-angle visual | Inherit K: free-edge **Δ screen-X ≥~25 px** @ +30° when judging open; soft-pass forbidden |
| 10 | Video | Continuous MP4 of both cycles (dual or world+kit); HUD panel hinge + XY; mark disturbance onset |

**Hard falsifier:** vision-in-walk; tip&lt;8; M145 edit; panel scripted / panel actuator; claiming latch / 90° / walk-through / UK / full open; hinge range bump as “Gate M”; undeclared assist/freeze; Stand XY pin; soft-pass of open-angle FAIL; spend.

**Not Gate M:** Hardware range bump toward 90°; latch; walk-through; UK handle height; Pi detector; sole/STEP; spend.

## Prove tags (Controls)

| Tag | Intent |
|-----|--------|
| **M00** | Open ≥25° + tip≥8 + coupling (K/L retain) |
| **M01** | Hold ≥1.0 s @ ≥25°; apply documented disturbance |
| **M02** | Reject: tip≥8; panel ≥15° within 2.0 s after disturb ends; then close Δ ≥10° |
| **M03** | **2/2** consecutive cycles; continuous MP4 |

Artifacts: `GATE_M_CONTROLS_NOTE.md`, `GATE_M_TABLE.json`, M00+ mp4 under `previews/ainex_walk/iterate/`.

## Plant / Hardware
- Companion only. Locked M145 untouched.
- **No plant change** for Gate M — confirm ±30° + Gate J spring + optional free-edge stripe remain plant-of-record.
- **Range bump:** out of scope until AI explicitly opens a later gate (not M; no PO).
- Manufacturing: idle; no spend.

## Owner
- **AI:** this criteria + score/lock
- **Controls:** M00–M03 prove on companion + frozen ckpt sha16 `9ffaa1a21b607bf6`
- **Hardware:** confirm plant-of-record (no change)
- **Manufacturing:** silent

## Explicit non-claims
Not Pi/Orin/NN-first; not vision-in-walk; **not full door open / latch / walk-through / 90°**; not UK height; no spend; **not** a hinge-range bump. Gate L lock stays leave/re-grasp on ±30° — M does not reopen L as full-open.

## Score — LOCKED TRUE (~03:49 BST)
Controls metrics PASS 2/2; AI re-watch after root **A** reconcile: both cycles show `disturb` + DISTURBANCE ON + panel dip (`gate_m_disturb_rewatch/`). Soft-pass not used. See `GATE_M_AI_LOCK.md`.
