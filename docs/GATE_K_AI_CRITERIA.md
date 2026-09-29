# Gate K — multi-bout panel open/close reliability (AI criteria, sim)

**When:** Mon 28 Sep 2026 ~02:06 BST  
**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (confirm plant-of-record)  
**Prereq:** Gate D–J TRUE (sim); Gate J larger panel ≥25° locked on soft-spring ±30° companion  
**Plant (K rows):** companion `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml`  
**Walk plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Sim only. No spend. Vision off walk. Not Pi NN-first / not Orin.**  
**Controls stub:** `docs/CONTROLS_POST_J_MUST_PROVE_STUB.md` (S00–S03 → K00–K03)  
**Selected path:** **Primary** (multi-bout on current ±30°). **Alt A** (range bump) and **Alt B** (leave/re-grasp) deferred — **not** Gate K.

## Goal
Package Gate J into a **demo-grade repeatable multi-bout open/close** on the **current ±30°** companion (N=**3** successful coupled swing+close bouts, tip≥8) — **without** more degrees, latch, 90°, or walk-through.

## Pass (all, assist OFF, freeze OFF, k_auth=1.0)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Inherit J | K00 retains J00/J01 (lever ≥15° + coupled panel ≥25° within ±30°; tip≥8) or re-prove equivalent |
| 2 | Multi-bout | **3** consecutive successful bouts in one continuous score run |
| 3 | Per-bout | Each bout: open panel abs ≥**25°**, hold ≥**1.0 s**, close Δ ≥**10°**; tip≥8; coupling honesty |
| 4 | Success rate | **3/3** bouts pass (no tip fail / loss of coupling / assist mid-bout) |
| 5 | Coupled | Same honesty as J: hand→lever→panel child→`door_panel_hinge`; **no** panel actuator / **no** scripted panel qpos |
| 6 | Plant honesty | Stay within ±30°; **do not** claim >30° on this hinge; Gate J spring (damp 0.05 / stiff 0.015) plant-of-record |
| 7 | Walk/vision split | No vision in Gate E / walk residual; ckpt sha16 `9ffaa1a21b607bf6` frozen |
| 8 | Video | Continuous MP4 of the 3 bouts (or K00 + K01 continuous); no assist geom |
| 9 | Base DOF honesty | Free-joint **x,y free** (no Stand XY pin / footprint lock). Tip from upright physics only; base may react to lever/panel forces. For **lock TRUE**, K03 must include **XY HUD** and/or **world-fixed** side cam (body-mounted `kit_cam` alone can false-positive “pinned” — frozen horizon is attitude, not XY) |

**Hard falsifier:** vision-in-walk; tip&lt;8; plant hop / M145 edit; panel scripted / panel actuator; claiming full open / latch / walk-through / UK height / **90°**; pretending ±30° supports >30°; assist/freeze/xfrc; **Stand XY hold / free-joint x,y pin** (soft-pass — K03 watch FAIL if base unnaturally rigid); soft-pass without this doc; demanding range-bump PO as Gate K.

**Not Gate K:** Hardware range bump (Alt A); leave-and-return / re-grasp (Alt B); full 90°; latch; walk-through; UK height; Pi detector; sole/STEP; spend.

## Prove tags (Controls)

| Tag | Intent (was stub Sxx) |
|-----|------------------------|
| **K00** | Retain J00/J01 — lever ≥15° + coupled panel ≥25° (within ±30°); tip≥8; Gate J spring |
| **K01** | Multi-bout open/close — **3** consecutive bouts; each open ≥25°, hold ≥1.0 s, close Δ ≥10°; tip≥8; coupling |
| **K02** | Bout success rate — **3/3** pass without tip fail / coupling loss |
| **K03** (optional) | Single continuous MP4 of the 3 bouts for room review; still not latch / 90° / walk-through |

Artifacts: `GATE_K_CONTROLS_NOTE.md`, `GATE_K_TABLE.json`, K00+ mp4 under `previews/ainex_walk/iterate/`.

## Plant / Hardware
- Companion only. Locked M145 untouched.
- **No plant change** for Gate K — confirm ±30° + Gate J spring remain plant-of-record (`docs/GATE_I_HARDWARE_PANEL_HINGE.md`, `docs/GATE_J_HARDWARE_PANEL_SPRING.md`).
- **Alt A / range bump:** out of scope until AI explicitly asks (not this gate; no PO).
- Manufacturing: idle; foot STEP frozen; no spend.

## Owner
- **AI:** this criteria + score/lock
- **Controls:** K00–K03 prove on companion + frozen ckpt
- **Hardware:** confirm plant-of-record (no change); idle unless AI later asks Alt A
- **Manufacturing:** silent

## Explicit non-claims
Not Pi/Orin/NN-first; not vision-in-walk; **not full door open / latch / walk-through / 90°**; not UK height; no spend; **not** a range bump. Gate J lock stays larger-but-limited on ±30° — K does not reopen J as full-open.

## Hold / soft-pass note (2026-09-28 ~02:23 BST)
Controls scored 3/3 with Stand XY hold; **K03 watch = FAIL visual**. Soft-pass **rejected**. AI will **not lock** until K00–K03 are redelivered with **XY free**. Prior `GATE_K_*` artifacts with `stand_xy_hold=true` are **superseded / void**.

## Score (this prove)
**GATE_K_FAIL** (2026-09-28 ~02:29 BST) — `previews/ainex_walk/iterate/GATE_K_AI_LOCK.md`. Honest XY-free rescore: K00 PASS; K01/K02 FAIL (2/3). **Not locked TRUE.** Criteria remain live for re-attempt (§9 still required).

## §9 visual hold (2026-09-28 ~02:29 BST)
XY-free rescore metrics reported `stand_xy_hold=false` and small base drift (Δx≈−9.5 mm, Δy≈−4.4 mm), but **K03 watch = FAIL** on visual base honesty (grid/horizon appear rigidly fixed under lever force). **§9 is not visually closed.** Do **not** lock TRUE / do **not** treat metric-only §9 as sufficient until Controls clarifies camera framing vs residual pin. GATE_K_FAIL (2/3) still stands; re-attempt only after that clarity.

## §9 diagnostic disposition (2026-09-28 ~02:31 BST)
Controls `GATE_K_XY_HONESTY_DIAG.md` = **Verdict A**: code/plant freeflyer XY free (no residual pin); prior K03 rigid grid/horizon read was **body-mounted `kit_cam` framing**, not a soft-pass. Soft-pass **not** claimed. **GATE_K_FAIL (2/3) still stands.** §9 **code/plant honesty accepted**; §9 **watch-closed for lock TRUE** only after next pack ships **XY HUD and/or world-fixed cam** and AI watch agrees — then one clean XY-free 3/3 re-attempt.

## §9 AI dual watch (2026-09-28 ~02:44 BST)
AI watched `K03_dual.mp4`. **§9 visual = FAIL (looks pinned).**
- HUD present and digits move (~dx≈13 mm claimed).
- **World-fixed panel:** green base appears welded to floor grid — no visible reaction under lever force.
- **kit_cam:** horizon + foreground post/shadow show **zero** parallax despite HUD Δ — contradicts claimed freeflyer translation.
- Soft-pass / residual pin / HUD-vs-render mismatch **not closed**. **Do not lock TRUE.** GATE_K_FAIL (2/3) stands. Controls must reconcile Verdict A code claim vs this visual FAIL before any re-attempt counts.

## §9 AI second look (2026-09-28 ~02:47 BST)
Re-watched `K03_dual.mp4` with tighter brief. **§9 visual FAIL confirmed** (not overturned).
- World-fixed: green base anti-aliasing staircase **identical** across frames; contact shadow under base **0 px** shift; arm shadow moves (do not confuse with base).
- HUD: x/y/dx/dy digits move (~1.3 cm claimed) — **contradicts** world-cam pixels.
- kit_cam: upper-body tilt dominates; cannot prove ~cm XY from ego alone.
- Likely Controls PASS mistook arm-floor shadow or HUD for base slide.
- **Not soft-closed.** Root cause still required: residual pin and/or HUD-vs-render mismatch. Metrics FAIL 2/3; **not locked TRUE**.

## §9 reconcile disposition (2026-09-28 ~02:49 BST)
Controls `GATE_K_XY_HONESTY_DIAG.md` RECONCILE = **SCALE** (PIN/CAM/HUD-as-lie rejected). Docs/JSON accepted: edge NCC (0,0) on shipped world cam under push; HUD matches q_free; geometric ~1–5 px at distance 1.35. AI dual-watch FAIL was **correct as a pixel watch** of that FOV, **not** proof of residual pin. Arm-shadow PASS = false-positive. Proof reel `K03_xy_proof.mp4` (+5 cm → ~−40 px) under AI watch. Future K03 must use **zoomed** world cam / base-edge-only. **GATE_K_FAIL 2/3 unchanged; not locked TRUE.**

AI proof watch `K03_xy_proof.mp4`: **SCALE_PROOF_PASS** (intentional +5 cm visible; live mm freeflyer). Confirms §9 SCALE disposition.

## Zoomed XY-free rescore (~02:52 BST)
Controls delivered zoomed §9 pack (d≈0.55, base-edge). **GATE_K_FAIL** unchanged: K00 PASS; K01/K02 **2/3** (B2 peak **21.75°**). §9 evidence path accepted; metrics block lock. **Next:** controller iterate on **same primary** (clear B2 under freeflyer). Alt A/B **still deferred**. Not locked TRUE.

## VISUAL_HOLD (~03:01 BST)
Controller-iterate metrics 3/3 recorded but **not lock-eligible**. Bout0 + §9 visual HOLD: pin rejected; §9 edge NCC real (~11–20 px); Bout0 qpos honest +30° (root cause A — framing/HUD). Soft-pass forbidden. Next: evidence re-encode (panel° HUD + panel-facing cam); then AI re-watch both.

## Open-angle visual (~03:08 BST)
Face-on thin slab foreshortens honest +30° (`docs/GATE_K_HARDWARE_PANEL_VISUAL.md`, `GATE_K_PANEL_VISUAL_MISMATCH.md`). Lock needs **oblique / free-edge** open-angle evidence. Soft-pass of face-on FAIL forbidden. §9 marked world may PASS independently.

## Open-angle watch criterion (~03:17 BST)
Oblique free-edge pack required. Judge **free-edge (far extremity) only** — not hinge-side silhouette (≈0 px at +30°). Bout0 free-edge may be only ~11 px without a tell. For lock TRUE: companion **edge-stripe** (visual-only, Hardware) on free edge + AI re-watch PASS. Soft-pass forbidden. Cite `GATE_K_OPEN_ANGLE_WATCH_CONFLICT.md`.

## Score — LOCKED TRUE (~03:24 BST)
Metrics 3/3 + §9 PASS + open-angle PASS under free-edge Δ screen-X ≥~25 px @ +30° (stills + measure; B2 watch false-FAIL accepted). Soft-pass not used. See `GATE_K_AI_LOCK.md`.
