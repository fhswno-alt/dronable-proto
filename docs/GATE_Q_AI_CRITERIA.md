# Gate Q — stepped approach then N compose then stepped retreat (AI criteria, sim)

**When:** Mon 28 Sep 2026 ~05:50 BST  
**Owner:** Founding AI Scientist (criteria) + Controls (prove) + Hardware (confirm plant-of-record)  
**Prereq:** Gate D–P TRUE (sim); Gate P stepped approach→N compose locked on ±30° companion  
**Plant (Q rows):** companion `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` (md5 lock `59cc408eda07037a58f92ad27da045d6`)  
**Walk plant untouched:** `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`  
**Sim only. No spend. Vision off walk. Not Pi NN-first / not Orin.**

## Goal
On the **current ±30°** plant, prove the kit can **approach with real stepped locomotion**, run a **full Gate N compose**, then **retreat with real stepped locomotion** back toward the start offset — all in the **same continuous score run**. Escalates Gate P by requiring stepped honesty on **exit**, not shim-only / teleport retreat. No hinge-range bump, latch, 90°, walk-through, or room-walk claim.

## Pass (all, assist OFF, freeze OFF, k_auth=1.0)

| # | Criterion | Threshold |
|---|-----------|-----------|
| 1 | Start offset | Begin with cam→door **≥0.55 m** **or** approach Δ toward door ≥**0.15 m** before first lever contact (document which) |
| 2 | Stepped approach | Inherit Gate P §2 honesty: during **active walk** (not finalize): ≥**2** visible foot lifts **per bout** with sole clearance ≥**2 cm above plant rest** (or abs thr that cannot trip on soft-contact rest gap alone); each lift **dwells ≥150–250 ms** consecutive clear; S9/kit stills multi-frame sole daylight (≥3–5 frames) both sides; stepped base Δ during active walk ≥**0.15 m**; tip≥8; HUD **FOOT-LIFT only while currently clear** |
| 3 | Shim budget (approach) | Skate mean ≤**0.08**, p95 ≤**0.18**; finalize/retreat kinematic XY shim allowed only after stepped Δ proved; stepped Δ ≥**~70%** of total approach cam/base Δ; HUD finalize = **KINEMATIC XY SHIM** only |
| 4 | Working stand + N compose | Enter N working stand; one bout satisfies Gate N: open ≥25°; hold ≥1.0 s @ ≥25°; declared disturb; reject panel ≥15° in 2 s; leave ≥0.5 s @ panel ≥20°; re-grasp; close Δ ≥10°; tip≥8 |
| 5 | Stepped retreat | After close (panel near rest), **active retreat walk** (not retreat-finalize shim alone): ≥**2** visible foot lifts with same sole-clear / dwell / multi-frame daylight honesty as §2; stepped retreat base Δ ≥**0.15 m** away from door **or** cam→door returns to ≥**0.55 m**; tip≥8; stepped retreat Δ ≥**~70%** of total retreat cam/base Δ; HUD FOOT-LIFT only while currently clear; retreat finalize (if any) = **KINEMATIC XY SHIM** only |
| 6 | Multi-bout | **2/2** consecutive **stepped-approach→N-compose→stepped-retreat** cycles in one continuous run (recover OK; each cycle must re-approach and re-retreat with steps, not shim-only either leg) |
| 7 | Plant honesty | ±30°; Gate J spring; companion md5 unchanged; **no** hinge-range bump |
| 8 | Base DOF | Free-joint **x,y free** during compose (no Stand XY pin). Same §9 honesty as K |
| 9 | Open-angle visual | Inherit K: free-edge **Δ screen-X ≥~25 px** @ +30°; soft-pass forbidden |
| 10 | Video | Continuous MP4 both cycles; HUD panel hinge + XY; mark approach start, foot-lift/SS, disturb, leave, retreat start, retreat foot-lift |

**Hard falsifier:** vision-in-walk; tip&lt;8; M145 edit; panel scripted / panel actuator; claiming latch / 90° / walk-through / UK / full open / room-walk / investor walk script; hinge range bump as “Gate Q”; undeclared assist/freeze/xfrc; Stand XY pin on compose; soft-pass; spend; shim-only approach **or** shim-only retreat; planted Gate E residual soft-XY while soles flush credited as stepped Δ (continuous Root B); finalize-dominated either leg; SS from contact-edge / sticky HUD / SOLE_OFFSET plant-rest bias without multi-frame visible daylight; HUD FOOT-LIFT while soles flush; HUD claiming stepped during finalize; separate approach / N / retreat score runs called Q; plant invent.

**Not Gate Q:** Hardware range bump; latch; walk-through doorway; UK handle; Pi detector; sole/STEP; spend; full-room cadence T≤0.75 demo claim; investor walk script.

## Prove tags (Controls)

| Tag | Intent |
|-----|--------|
| **Q00** | Offset start + stepped approach (SS/foot-lift + Δ + skate) |
| **Q01** | Enter working stand; full N compose |
| **Q02** | Stepped retreat to offset (SS/foot-lift + Δ + skate; cam≥0.55 or retreat Δ≥0.15) |
| **Q03** | **2/2** consecutive approach→compose→retreat cycles; continuous MP4 |

Artifacts: `GATE_Q_CONTROLS_NOTE.md`, `GATE_Q_TABLE.json`, Q00+ mp4 under `previews/ainex_walk/iterate/`.

## Plant / Hardware
- Companion only. Locked M145 untouched. **No plant change** for Gate Q.
- Range bump / latch / walk-through: out of scope (not Q; no PO).
- Manufacturing: idle; no spend.

## Owner
- **AI:** this criteria + score/lock
- **Controls:** Q00–Q03 on companion + frozen ckpt sha16 `9ffaa1a21b607bf6` (prefer Gate E residual for steps both directions; CSF50 OK if tip/skate hold)
- **Hardware:** confirm plant-of-record (no change)
- **Manufacturing:** silent

## Explicit non-claims
Not Pi/Orin/NN-first; not vision-in-walk; **not full door open / latch / walk-through / 90°**; not UK height; no spend; **not** a hinge-range bump; **not** a full-room walk / investor walk script. Soft-pass forbidden. Gate P lock stays stepped-approach→compose — Q adds **stepped retreat**.

## Score
Criteria **live** and **KEPT** (~05:50 BST; reaffirmed through A28c / E7lock). Soft-pass forbidden. Skate ≤0.08/0.18 and clear_frac≥0.55 **not** softened. Formal §5 still wants stepped retreat Δ ≥~70%.

**Disposition ~14:22 BST: Prefer FAIL structural** — fair tip set H1–H10 (front-load + clear-through-gap) did not put destination cam into clear under cancel×0.70 while holding apps + bout1 cf_ok+joint. Residual cam advance is planted-middle Root B. See `docs/GATE_Q_AI_STRUCTURAL_PREFER_FAIL.md`. Gate Q **OPEN** (not lock TRUE, not LOCKED FALSE). Curriculum D–P TRUE. Sync: `docs/AI_GATE_Q_STATUS_MONDAY.md`.
