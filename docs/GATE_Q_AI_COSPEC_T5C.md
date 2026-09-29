# Gate Q AI cospec — T5-C BC teacher + multi-seed (~18:55 BST)

**To:** Controls · **From:** AI · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen  
**Dave:** Gate Q **stay open** — escalate after T5-B Prefer FAIL (do not park Q)

## Veto?

**No veto — fair new family.** Aligns AI `docs/GATE_Q_AI_RESEARCH_T5.md` T5-C + T5/T5B escalation. T5-B Prefer FAIL agreed (`docs/GATE_Q_AI_SCORE_T5B.md`): bout1 skate under + ret_ok; bout0 p95 **0.183** (0.003 over) + plant-cam ε Prefer FAIL.

**≠ closed:** R*/S1–S3 · T5-A reward-only twins · T5-B model-base-only twins · T4 opts A+E left OFF — T5-C **is** finishing teacher BC + multi-seed under skate-honest bars.

## Family

**T5-C** — reverse **teacher BC** (oracle CSF50 / Placo short-reverse / documented reverse schedule / optional Walk-This-Way footstep timing) → residual RL, with **multi-seed Prefer FAIL wall** (opt E). Skate mean/p95 co-primary; cf≥0.55 floor; plant-cam ε hard Prefer FAIL; tip≥8; apps hold. Clear-only residual; planted cancel×**0.70** only — **zero** planted soft-XY. Teacher **never** credits plant XY as stepped. Dual-ckpt: approach=E7lock · retreat=T5-C via `GATE_Q_T5C_CKPT`. Soft-pass **never**.

May keep T5-B model-base / T5-A SLR-DXB-ANK as **conditioning inside new weights** — not twin env flags on frozen E7lock.

Bootstrap: **T5B_02 near-miss** zip if present, else T5_06, else T4_09, else E7lock — log which.

## Baseline (held)

| Item | Value |
|------|--------|
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` |
| Rollback | E7lock `9ffaa1a21b607bf6` until Prefer-FAIL beat |
| cancel | ×0.70 — no FREEZE / qvel0 / damp↑ / cancel↑ |
| R*/S1–S3 / T5-A/B twin flags on frozen | **OFF** |
| Vision / plant invent / Path A | **off** |

## Must-holds

1. Soft-pass **off**. Skate ≤0.08/0.18 · clear_frac≥0.55 · tip≥8 — **no soften**.
2. Teacher BC phase documented (source + steps); Prefer FAIL if teacher banks plant XY.
3. Multi-seed: **≥2 seeds** (prefer 3); report each seed fair Prefer FAIL; best seed must still clear bars honestly — no cherry-pick soft metrics.
4. Clear authority only while sole clear. Plant-cam ε: Prefer FAIL if plant-window cam >**0.02** or cum >**0.05**/bout (T5B_02 ε steal is a live target).
5. Apps hold ~0.548/0.630 or Prefer FAIL if traded.
6. Attack: close bout0 skate p95 (0.183→≤0.18) **and** kill ε steal **without** trading bout1 package / apps.
7. Logs every score: `ckpt_id`, `seed`, `bc_teacher`, skate mean/p95, clear_frac, tip, apps, plant-cam suite, `cam@preSS`, `dxc_pre`, `planted_middle_s`, optional footstep timing fields.
8. New sha16 **only** after Prefer FAIL fair set beats E7lock honestly (note vs T5B_02 / T5_06).
9. No HW/MFG plant ask / no spend.

## Stack (Controls owns)

- Train: `train_t5c_teacher.py` (or documented T5-C mode: BC warmup + PPO/SAC)  
- Artifacts: `previews/ainex_walk/iterate/learned_gate_e/` (`ppo_t5c_seed*_*.zip`, logs, history)  
- Scorer: `score_gate_q.py` + T5-C ckpt env  
- Budget: ≤4 h **or** ≤2e6 steps **per seed** (first hit per seed); early stop on `ret_ok` both; aggregate Prefer FAIL if no seed clears

## Success / Prefer FAIL

- **Pass path:** `ret_ok` both under bars + ε + apps + tip + cf → continuous watch → AI Q03 lock path.
- **Prefer FAIL:** all seeds budget-wall; apps traded; Root B / ε; soft bars; twin collapse; plant invent.
- **Escalation if Prefer FAIL:** **T5-D** only with Dave (architecture / plant). Soft-pass off. **Do not park Gate Q** without Dave — ask him for T5-D vs park.

## Spin

Controls: train **only under this cospec**. Fair Prefer FAIL per seed vs E7lock (+ note vs T5B_02). Same-turn Prefer FAIL score or `ret_ok` pack when ready. HW/MFG: freeze holds — no plant ask.
