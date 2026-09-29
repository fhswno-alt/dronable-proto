# Gate J — controlled larger panel swing (AI criteria, sim)

**When:** Mon 28 Sep 2026 ~01:58 BST  
**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (hinge range honesty)  
**Prereq:** Gate D–I TRUE (sim); Gate I limited panel ≥10° locked  
**Plant (J rows):** companion `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml`  
**Walk plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Sim only. No spend. Vision off walk. Not Pi NN-first / not Orin.**  
**Controls stub:** `docs/CONTROLS_POST_I_MUST_PROVE_STUB.md` (R00–R03 → J00–J03)

## Goal
After Gate I’s ~12° coupled swing, prove a **controlled larger panel swing** (≥**25°**, still inside Hardware ±30°) with tip≥8 and honest lever/hand coupling, plus intentional hold+close — **not** latch / UK handle / 90° / walk-through.

## Pass (all, assist OFF, freeze OFF, k_auth=1.0)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Inherit I | J00 retains I00/I01 (lever ≥15° + coupled panel ≥10°; tip≥8) or re-prove equivalent |
| 2 | Larger swing | Panel hinge abs ≥**25°** from rest; tip≥8; stay within plant ±30° |
| 3 | Coupled | Same honesty as I: hand→lever contact→lever child of panel→`door_panel_hinge`; **no** panel actuator / **no** scripted panel qpos |
| 4 | Hold + close | Sustain ≥25° for ≥**1.0 s**, then intentional close toward rest with Δ ≥**10°**; tip≥8 |
| 5 | Upright | tip≥8 on all J rows; skate mean ≤0.08 if stepping |
| 6 | Walk/vision split | No vision in Gate E / walk residual; ckpt sha16 `9ffaa1a21b607bf6` frozen |
| 7 | Video | Continuous MP4: larger panel arc + hold/close; no assist geom |

**Hard falsifier:** vision-in-walk; tip&lt;8; plant hop / M145 edit; panel scripted / panel actuator; claiming full open / latch / walk-through / UK height; assist/freeze/xfrc; soft-pass without this doc.

**Not Gate J:** full 90° door open; latch release; walk-through; UK door-handle height product; Pi detector; sole/STEP refresh; spend.

## Prove tags (Controls)

| Tag | Intent (was stub Rxx) |
|-----|------------------------|
| **J00** | Retain I00/I01 — lever ≥15° + coupled panel ≥10°; tip≥8; vision off |
| **J01** | Larger coupled swing — panel abs ≥**25°** (within ±30°); tip≥8; coupling honesty |
| **J02** | Hold ≥25° ≥**1.0 s**, then close Δ ≥**10°**; tip≥8 |
| **J03** (optional) | Repeatability — reverse toward rest (panel ≤5°), then second swing ≥**15°**; tip≥8; still not latch |

Artifacts: `GATE_J_CONTROLS_NOTE.md`, `GATE_J_TABLE.json`, J00+ mp4 under `previews/ainex_walk/iterate/`.

## Plant / Hardware
- Companion only. Locked M145 untouched.
- Panel/lever hinges as Gate I: `docs/GATE_I_HARDWARE_PANEL_HINGE.md` (range ±30°; lever child of panel).
- **Hardware confirm:** soft centering spring still allows ≥25° under hand force through lever; light retune of damping/stiffness OK if needed — **no** M145 hop; Controls does not invent plant edits.
- Manufacturing: foot STEP frozen; panel STEP still N/A for J unless Hardware freezes dims.

## Owner
- **AI:** this criteria + score/lock
- **Controls:** J00–J03 prove on companion + frozen ckpt
- **Hardware:** confirm ±30° + spring allow ≥25° coupled; no M145 hop
- **Manufacturing:** silent unless dims freeze for QC

## Explicit non-claims
Not Pi/Orin/NN-first; not vision-in-walk; **not full door open / latch / walk-through**; not UK height; no spend. Gate I lock stays limited-swing context — J does not reopen I as full-open.

## Lock
**GATE_J_LOCKED_TRUE** (2026-09-28 ~02:04 BST) — `previews/ainex_walk/iterate/GATE_J_AI_LOCK.md` (soft-spring plant-of-record).
