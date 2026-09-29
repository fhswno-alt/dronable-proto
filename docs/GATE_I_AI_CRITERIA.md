# Gate I — limited door-panel swing (AI criteria, sim)

**When:** Mon 28 Sep 2026 ~01:48 BST  
**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (panel hinge honesty)  
**Prereq:** Gate D–H TRUE (sim); Gate H grasp+lever rotate locked  
**Plant (I rows):** companion `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` (+ panel hinge if needed)  
**Walk plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Sim only. No spend. Vision off walk. Not Pi NN-first / not Orin.**

## Goal
After Gate H lever rotate, prove a **limited door-panel swing** driven by the hinged lever / hand — enough to show the panel **moves**, **not** a full open / latch-clear / walk-through claim.

## Pass (all, assist OFF, freeze OFF, k_auth=1.0)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Inherit H | I00 retains H02/H03 (lever rotate ≥15°; tip≥8) or re-prove equivalent |
| 2 | Panel swing | Door panel hinge rotates ≥**10°** (absolute) from rest; tip≥8 |
| 3 | Coupled motion | Panel motion coincident with lever rotate / hand contact (not free-fall / scripted panel-only cheat) |
| 4 | Upright | tip≥8 on all I rows; skate mean ≤0.08 if stepping |
| 5 | Walk/vision split | No vision in Gate E / walk residual; ckpt sha16 `9ffaa1a21b607bf6` frozen |
| 6 | Video | Continuous MP4: lever + panel move; no assist geom |

**Hard falsifier:** vision-in-walk; tip&lt;8; plant hop / M145 edit; panel scripted without lever coupling; claiming full open / latch clear / walk-through; assist/freeze/xfrc.

**Not Gate I:** full 90° door open; latch release; UK door-handle height product; Pi detector; sole/STEP refresh; spend.

## Prove tags (Controls)

| Tag | Intent |
|-----|--------|
| I00 | Retain H02/H03 lever rotate ≥15° |
| I01 | Panel swing ≥10° with lever/hand coupling |
| I02 | Optional: hold panel ≥10° for ≥1 s without tip |
| I03 | Optional: reverse / close ≥5° (shows control, not latch) |

Artifacts: `GATE_I_CONTROLS_NOTE.md`, `GATE_I_TABLE.json`, I00+ mp4 under `previews/ainex_walk/iterate/`.

## Plant / Hardware
- Companion only. Locked M145 untouched.
- Panel hinge shipped: `docs/GATE_I_HARDWARE_PANEL_HINGE.md` (lever child of panel; no panel actuator).
- Manufacturing: foot STEP frozen; hinged lever STEP deferred stays OK; panel STEP not required for I.

## Owner
- **AI:** this criteria + score/lock
- **Controls:** I00–I03 prove (I01 after panel hinge)
- **Hardware:** panel hinge on companion for I01; no M145 hop
- **Manufacturing:** silent unless dims freeze for QC

## Explicit non-claims
Not Pi/Orin/NN-first; not vision-in-walk; **not full door open**; not latch/UK height; no spend.

## Lock
**GATE_I_LOCKED_TRUE** (2026-09-28 ~01:57 BST) — `previews/ainex_walk/iterate/GATE_I_AI_LOCK.md`
