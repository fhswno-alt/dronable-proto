# Gate F — lever approach (AI criteria, sim)

**When:** Mon 28 Sep 2026 ~01:02 BST  
**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (plant cam / lever prop honesty)  
**Prereq:** Gate D TRUE; Gate E TRUE (sim learned residual, ckpt frozen)  
**Plant:** locked `ainex_controls_m2_145.xml` — **no plant hop**  
**Sim only. No spend talk. Vision off walk loop. Not Pi NN-first / not Orin.**

## Goal
Prove the kit can **look at and approach** a prop lever at **250–300 mm AFF** (working center **275 mm**) from ~**0.4 m** cam→door, with `head_tilt` depression locked to FOV re-bind, **without** putting vision in the walk loop.

## Pass (all, assist OFF, freeze OFF)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Inherit walk | Gate E ckpt (or open-loop CSF50 for stand-only rows) still upright; tip≥8 on approach script |
| 2 | Look-down | Command **`head_tilt`** so optical axis centers lever @ **0.4 m** → depression **≈ −15° to −16°** (FOV re-bind `docs/FOV_REBIND_AINEX_HIWONDER.md`); deepen ≈ −19°/−20° only for optional 0.3 m grasp row |
| 3 | Lever in ego | With kit HFOV claim **120°** (or Hardware `kit_cam` site when present): lever bbox / marker stays in frame ≥**2 s** continuous at 0.4 m stand or slow approach |
| 4 | Approach | Base/dx toward door ≥**0.15 m** or final cam→door ∈ **[0.35, 0.45] m** without tip; skate mean ≤**0.08** if stepping |
| 5 | Walk/vision split | **No** vision feature in walk residual / PPO obs — Gate E stack unchanged |
| 6 | Hands (sim) | Standard hand frames reach lever Z band **0.25–0.30 m** AFF without floor collision (kinematic OK; grasp force optional later) |
| 7 | Video | Continuous MP4: look-down + approach; no assist geom |

**Hard falsifier:** lever only visible via cartoon cam Z, wrong joint (`neck_pitch` not `head_tilt`), vision injected into Gate E policy, tip/skate fail, or plant change required.

**Not Gate F:** full UK door-handle height; Pi on-device detector claim; sole/STEP refresh; spend / cart.

## Plant / cam honesty
- Prefer measured `kit_cam` site when Hardware adds it; until then use FOV re-bind cam Z ≈**0.380 m**.
- Prop lever Z = **0.275 m** AFF in MJCF (or Hardware note).
- 16 mm sole box vs CAD 4.5 mm: **irrelevant** to Gate F unless tip geometry blocks reach — do not reopen GEO for F.

## Prove tags (Controls)
| Tag | Intent |
|-----|--------|
| F00 | Stand @ 0.4 m, head_tilt −16°, lever in frame ≥2 s |
| F01 | Slow approach 0.5→0.4 m with Gate E ckpt or CSF50; look-down held |
| F02 | Optional 0.3 m grasp look-down ≈ −19° (kinematic only) |

Artifacts: `GATE_F_CONTROLS_NOTE.md`, `GATE_F_TABLE.json`, F00+ mp4 under `previews/ainex_walk/iterate/`.

## Owner
- **AI:** this criteria + score/lock
- **Controls:** F00–F02 prove (idle until ready)
- **Hardware:** confirm lever prop + cam site / optical Z if missing
- **Manufacturing:** silent (STEP frozen)

## Monday
Use with `docs/MONDAY_PERCEPTION_COMPUTE_CHECKLIST.md`. Standard lock meeting: FOV bind + Gate F criteria — still no spend ask.

## Lock
**GATE_F_LOCKED_TRUE** (2026-09-28 ~01:15 BST) — `previews/ainex_walk/iterate/GATE_F_AI_LOCK.md`
