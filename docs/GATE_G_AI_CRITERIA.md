# Gate G — lever reach / light contact (AI criteria, sim)

**When:** Mon 28 Sep 2026 ~01:34 BST  
**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (lever contact honesty)  
**Prereq:** Gate D TRUE; Gate E TRUE (frozen ckpt); Gate F TRUE (look+approach)  
**Plant (G rows):** companion `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml`  
**Walk plant (untouched):** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Sim only. No spend. Vision off walk loop. Not Pi NN-first / not Orin.**

Controls stub accepted: `docs/CONTROLS_NEXT_MUST_PROVE_STUB.md` → named **Gate G**. Tags **G00–G03** (was provisional N00–N03).

## Goal
After Gate F look+approach, prove **hand/wrist reach to the demo lever** at **275 mm AFF** (band 250–300), with optional **light contact** — **no** grasp-force closure and **no** door-open / latch claim.

## Pass (all, assist OFF, freeze OFF, k_auth=1.0)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Inherit F look+approach | G00 retains F00; G01 retains F01 (tip≥8; ego≥2 s; head_tilt ≈ −16° @ ~0.4 m; F01 dx≥0.15 or cam∈[0.35,0.45]; skate mean ≤0.08 if stepping) |
| 2 | Head joint | Command **`head_tilt`** only (not `neck_pitch`); deepen ≈ −19°/−20° only on grasp-distance rows |
| 3 | Hands reach | ≥1 gripper/wrist site within **≤0.05 m** of `door_lever` geom center (or Hardware contact site); hand Z in **0.25–0.30 m** AFF |
| 4 | Upright | tip≥8 on reach/contact rows; no fall |
| 5 | Light contact (G03) | Brief hand–lever geom contact **or** ≤**5 mm** lever site depression on a **contactable** lever geom; tip/skate still OK if stepping |
| 6 | Walk/vision split | **No** vision in Gate E / walk residual obs; ckpt bytes frozen (`sha16 9ffaa1a21b607bf6`) |
| 7 | Video | Continuous MP4: reach (± contact); no assist geom |

**Hard falsifier:** vision-in-walk; tip&lt;8 or skate&gt;0.08 on stepping rows; wrong head joint; plant hop / Gate E rescore on companion; GEO reopen; assist/freeze/xfrc cheat; claiming grasp wrench or door swing.

**Not Gate G:** force grasp closure; door panel open; UK door-handle height product; Pi detector; sole/STEP refresh; spend.

## Prove tags (Controls)

| Tag | Intent |
|-----|--------|
| G00 | Retain F00 — stand look-down @ ~0.4 m |
| G01 | Retain F01 — approach w/ frozen Gate E ckpt |
| G02 | Hands reach ≤5 cm to lever; Z in 0.25–0.30 m AFF (kinematic OK) |
| G03 | Optional light contact (needs contactable lever geom — Hardware) |

Artifacts: `GATE_G_CONTROLS_NOTE.md`, `GATE_G_TABLE.json`, G00+ mp4 under `previews/ainex_walk/iterate/`.

## Plant / Hardware
- Companion only for G rows. Locked M145 contact/HX untouched.
- Lever Z **0.275 m** AFF (STEP stub `docs/GATE_F_LEVER_STEP_STUB.md`).
- For **G03**: Hardware must enable a **contactable** lever geom/site if current `door_lever` is visual-only (`contype=0`). G02 can pass kinematic without contact geom.
- Mass/COM + hand-reach honesty (Hardware in flight) informs G02/G03 scoring — do not invent kit reach beyond Hardware note.

## Owner
- **AI:** this criteria + score/lock
- **Controls:** G00–G03 prove when ready
- **Hardware:** contactable lever if G03; mass/COM + hand-reach honesty
- **Manufacturing:** silent (foot STEP frozen; lever prep only)

## Explicit non-claims
Not Pi/Orin/NN-first; not vision-in-walk; not UK handle height; not force door open; no spend.

## Lock
**GATE_G_LOCKED_TRUE** (2026-09-28 ~01:40 BST) — `previews/ainex_walk/iterate/GATE_G_AI_LOCK.md`
