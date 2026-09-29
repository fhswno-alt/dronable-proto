# Gate H — lever grasp / rotate (AI criteria, sim)

**When:** Mon 28 Sep 2026 ~01:42 BST  
**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (hinged lever honesty)  
**Prereq:** Gate D/E/F/G TRUE (sim); Gate G light contact locked  
**Plant (H rows):** companion `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` (+ Hardware hinge if needed)  
**Walk plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Sim only. No spend. Vision off walk. Not Pi NN-first / not Orin.**

## Goal
After Gate G reach + light contact, prove a **scripted / open-loop grasp hold** and **small lever rotate** on the demo prop @ **275 mm AFF** — still **no** full latch release / full door swing / UK handle-height product claim.

## Pass (all, assist OFF, freeze OFF, k_auth=1.0)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Inherit G | H00 retains G02 reach (min_d ≤0.05 m; hand Z ∈[0.25,0.30]; tip≥8) or re-prove equivalent |
| 2 | Grasp hold | Hand–lever contact sustained ≥**1.0 s** while finger/gripper cmd moves toward close (HX-12H class); tip≥8 |
| 3 | Lever rotate | Lever hinge rotates ≥**15°** about its shaft (Hardware hinged DOF) **or** equivalent measured site arc ≥**15°** without tip/skate fail |
| 4 | Upright | tip≥8 on all H rows; skate mean ≤0.08 if stepping |
| 5 | Walk/vision split | No vision in Gate E / walk residual obs; ckpt sha16 `9ffaa1a21b607bf6` frozen |
| 6 | Video | Continuous MP4: grasp + rotate; no assist geom |

**Hard falsifier:** vision-in-walk; tip&lt;8; plant hop / M145 edit; GEO reopen; assist/freeze/xfrc; claiming full door open / latch without criteria; cartoon hinge not on companion.

**Not Gate H:** full door panel swing to open; latch release; UK door-handle height; Pi detector; sole/STEP refresh; spend.

## Prove tags (Controls)

| Tag | Intent |
|-----|--------|
| H00 | Retain G02 reach pose (or re-prove) |
| H01 | Grasp hold ≥1 s with close cmd |
| H02 | Lever rotate ≥15° (needs Hardware hinge if lever still welded) |
| H03 | Optional: hold rotate ≥1 s without tip |

Artifacts: `GATE_H_CONTROLS_NOTE.md`, `GATE_H_TABLE.json`, H00+ mp4 under `previews/ainex_walk/iterate/`.

## Plant / Hardware
- Companion only. Locked M145 contact/HX untouched.
- Hinge shipped: `docs/GATE_H_HARDWARE_LEVER_HINGE.md` (`door_lever_hinge` ±30° on companion).
- Finger DOF: use kit Standard hand joints if present; else gripper close proxy documented by Controls.

## Owner
- **AI:** this criteria + score/lock
- **Controls:** H00–H03 prove when ready (H02 after hinge)
- **Hardware:** hinged lever on companion for H02; no M145 hop
- **Manufacturing:** silent (foot STEP frozen; lever STEP already QC PASS)

## Explicit non-claims
Not Pi/Orin/NN-first; not vision-in-walk; not full door open; not UK handle height; no spend.

## Lock
**GATE_H_LOCKED_TRUE** (2026-09-28 ~01:47 BST) — `previews/ainex_walk/iterate/GATE_H_AI_LOCK.md`
