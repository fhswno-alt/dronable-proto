# Gate Q AI cospec — T4 reverse-native ckpt (~16:07 BST)

**To:** Controls · **From:** AI · Soft-pass **off** · Bars **KEPT** · Dave **GREENLIT** · Prefer FAIL train

## Veto?

**No veto — fair new family.** Dave greenlit T4 reverse-native ckpt (+ creative Prefer-FAIL options). Aligns Controls research T4 / AI research Q-U3. S1–S3 closed under frozen ckpt `9ffaa1a21b607bf6` with Root B (plant≫clear) persisting — new weights are the honest next residual, not another scheduler on Gate E forward residual.

**≠ closed families:** R1–R5 / Q-U1 CSF F+T / S2 ALIP Δu_fp / S3 REV-PHASE as **flags on frozen E7lock**. T4 may *reuse ideas* (negative Vx, reverse phase, clear-only track) only **inside the new reverse-trained policy**, not as twin env schedulers on the old ckpt.

Controls proposal `docs/GATE_Q_AI_COSPEC_T4_PROPOSAL.md` **accepted** on must-holds + stack note.

**Stack (Controls owns):** `scripts/learned_gate_e/{train_ppo,finetune_clear,env_ainex,eval_policy}.py` · artifacts `previews/ainex_walk/iterate/learned_gate_e/` · scorer `scripts/score_gate_q.py` loads new `CKPT_SHA16` only after Prefer FAIL fair set beats E7lock honestly. Keep E7lock as rollback until then.

## Family

**New reverse-native residual / policy ckpt** for Gate Q retreat (sim only). Condition on commanded sagittal velocity spanning **negatives** (and remaining retreat Δ) so reverse is a first-class command, not mirrored forward residual + place targets. Train / thin-finetune under Prefer FAIL until dest cam banks in **clear** SS (FOOT-LIFT honesty) with skate/cf bars, or Prefer FAIL the train budget.

Approach + N compose stay on locked E7lock / frozen Gate E path unless a single joint ckpt proves both without trading apps — default: **retreat-only new weights**, approach package held.

## Baseline (held)

| Item | Value |
|------|--------|
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` (plant XML untouched) |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` |
| Prior best eval package | E7lock (apps ~0.548/0.630; bout1 rcf 0.649; skate ~0.098/0.221) — **baseline to beat honestly** |
| CLEAR_TRACK_SWING | 1 on clear sole only |
| cancel | ×0.70 planted (do not reopen ↑ / FREEZE / qvel0 / damp↑) |
| R*/S1–S3 flags as twins on frozen ckpt | **OFF — do not reopen** |
| Vision-in-walk | **off** |
| Soft-pass | **off** |

## Must-holds (every train + eval)

1. Soft-pass **forbidden**. Skate ≤0.08/0.18 · clear_frac≥0.55 — **no soften**. Formal §5 ~70% stepped Δ north star.
2. Clear authority only while sole clear (FOOT-LIFT ≥2 cm). **Zero** planted soft-XY stride credit.
3. Plant-cam ε: Prefer FAIL / hard train penalty if plant-window cam gain >**0.02** or cum >**0.05**/bout.
4. tip≥8 · apps hold ~0.548/0.630 on approach bouts or Prefer FAIL if traded for retreat.
5. No plant invent · no GEO/friction/STEP · no Path A · no HW spend · no Pi/Orin NN-first claim.
6. Logs every score: `ckpt_id`, `vx_cmd`, `cam@preSS`, `dxc_pre`, `planted_middle_s`, `cam_gain_in_gap`, plant-cam suite, PRE_GAP, clear_frac, skate, tip.
7. Ping AI full Q03 only on `ret_ok` both + Controls continuous watch PASS.

## Train plan (Controls owns loop; AI owns score/lock)

| Item | Spec |
|------|------|
| Policy | Small MLP residual (or equal) → Δq / Δqdot / joint targets; **state-dependent**; action inside kit HX clips |
| Obs | Proprio q, qd; base ω,v; phase/clock; contact / foot height; commanded Vx / remaining retreat Δ — **no vision** |
| Bootstrap | Thin finetune from frozen `9ffaa1a21b607bf6` / Gate E best zip **or** BC warmup from reverse teacher (CSF50 / oracle reverse schedule) via `finetune_clear.py` / `train_ppo.py` — Controls picks one; log which |
| Tasks | Retreat-only stepped retreat + optional full Q compose (approach→N→retreat) once retreat clear_frac approaches bar |
| Reward | Primary: clear_frac↑, skate↓, tip≥8 floor, dest cam/dxc from **clear** SS, plant-cam ε **heavy penalty**, Root B (plant≫clear cam) penalty; retain approach apps |
| Curriculum | Start short reverse steps → full Q retreat Δ; do not cold-start full Q03 only |
| Budget (Prefer FAIL wall) | **≤4 h** box train **or** ≤2e6 env steps (first hit wins stop); early stop on `ret_ok` both under bars |
| Eval tags | T4_R0 = E7lock frozen baseline; T4_01+ under `previews/ainex_walk/iterate/learned_gate_e/`; fair Prefer FAIL via `score_gate_q.py`; new sha16 only after honest beat |
| Pass | `ret_ok` both bouts + skate/cf/tip/apps + plant-cam ≤ε + Controls continuous watch PASS → AI independent stills/Q03 → lock path |
| Prefer FAIL train | Budget exhausted without `ret_ok` both under bars → **T4 class Prefer FAIL / hard-falsify reverse-ckpt under current plant+cancel** — park; do not soften |

## Creative Prefer-FAIL options (allowed; still no soft-pass)

Dave asked for creative Prefer-FAIL options to get walk-back world-class. **In scope** if they stay sim-only, bars KEPT, no plant invent, and Prefer FAIL scored:

| Opt | Idea | Guard |
|-----|------|-------|
| A | Reverse teacher / oracle CSF reverse BC → residual RL | Teacher never credits plant XY as stepped |
| B | Negative-Vx library spanning vx∈[−v,0] (Cassie-style conditioning) | tip≥8; dx_back co-cap OK |
| C | Joint ckpt: reverse residual **plus** reverse phase encoding in one train (S3 idea inside new weights) | Not S3 flag on frozen E7lock |
| D | Mid-swing foothold residual smoothness inside new policy (S2 idea inside new weights) | Not ALIP flag on frozen E7lock |
| E | Multi-seed Prefer FAIL wall (2–3 seeds) — best seed must still clear bars | No cherry-pick soft metrics |
| F | Asymmetric reward: clear-SS cam bank ≫ plant-cam; explicit Root B falsifier term | ε still hard Prefer FAIL |

**Out:** bar soften · soft-pass · FREEZE/cancel↑ · plant/STEP/µ invent · vision-memory backward · Path A · claiming lock from skip-video alone · reopening R1–R5 / S1–S3 twin schedulers on frozen ckpt.

## Success / Prefer FAIL

- **Progress / pass path:** dest cam/dxc from **clear** reverse-native SS; skate→bars; tip/cam/joint + apps held; plant-cam ≤ε; PRE_GAP shrinks; then continuous watch + AI lock procedure.
- **Prefer FAIL:** fair eval dies, Root B persists, budget wall hit, apps traded, or creative option collapses into soft metrics / plant credit.
- After Prefer FAIL wall: **park** — next lever needs Dave (architecture or plant), not another silent twin.

## Spin

Controls: start train **only under this cospec**. Fair Prefer FAIL eval table each meaningful land. Same-turn Prefer FAIL score or `ret_ok` pack-ping when ready. HW/MFG: freeze holds — no plant ask / no spend.
