# Gate Q AI cospec — T5-B ALIP/DCM+ankle reverse base + residual (~18:06 BST)

**To:** Controls · **From:** AI · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen  
**Dave:** Gate Q **stay open** — escalate after T5-A Prefer FAIL (do not park Q)

## Veto?

**No veto — fair new family.** Aligns AI `docs/GATE_Q_AI_RESEARCH_T5.md` T5-B + Controls research escalation + Dave keep-moving. T5-A Prefer FAIL agreed (`docs/GATE_Q_AI_SCORE_T5.md`): cf superb (0.900/0.894), bout0 skate under bars once, **bout1 skate p95** still Prefer FAIL.

**≠ closed:**
| Closed | Why T5-B differs |
|--------|------------------|
| S2 ALIP-TVR | Env-flag Δu_fp on **frozen** E7lock — Prefer FAIL |
| T5-A | Skate-primary residual **only** (no reduced-order reverse footstep/ankle **base**) |
| R*/S1/S3/T4 twins | Stay OFF on frozen E7lock |

## Family

**T5-B** — model-based **reverse** footstep + **ankle** prior (ALIP S2S / DCM / Placo-style short reverse steps) inside a **new trained stack**, plus residual that closes full-body gap. Mid-swing foothold replan OK **in new weights**. Skate mean/p95 stay co-primary with cf floor ≥0.55 (T5-A lesson held). Clear-only residual authority; planted cancel×**0.70** only — **zero** planted soft-XY. Dual-ckpt: approach=E7lock · retreat=T5-B via `GATE_Q_T5B_CKPT` (or documented name). Soft-pass **never**.

Bootstrap: **T5_06 near-miss** zip if present (cf 0.900/0.894 · bout0 skate✓), else T4_09, else E7lock — log which.

## Baseline (held)

| Item | Value |
|------|--------|
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` |
| Rollback | E7lock `9ffaa1a21b607bf6` until Prefer-FAIL beat |
| cancel | ×0.70 — no FREEZE / qvel0 / damp↑ / cancel↑ |
| R*/S1–S3 / T5-A twin flags on frozen | **OFF** |
| Vision / plant invent / Path A | **off** |

## Must-holds

1. Soft-pass **off**. Skate ≤0.08/0.18 · clear_frac≥0.55 · tip≥8 — **no soften**.
2. Reduced-order prior + residual live in **new weights** — Prefer FAIL if implementation is only S2-style env flags on frozen E7lock.
3. Clear authority only while sole clear (FOOT-LIFT ≥2 cm). Ankle DF / foothold Δ only clear-phase or model-base commanded — not planted soft-XY credit.
4. Plant-cam ε: Prefer FAIL if plant-window cam >**0.02** or cum >**0.05**/bout.
5. Apps hold ~0.548/0.630 or Prefer FAIL if traded.
6. Prefer FAIL if bout1 (or either) skate p95 still >0.18 while claiming progress — T5-A wall target.
7. Logs every score: `ckpt_id`, `fp_prior` / `alip_or_dcm`, `ankle_cmd`, `du_fp`/`n_replan`, skate mean/p95, clear_frac, tip, apps, `cam@preSS`, `dxc_pre`, `planted_middle_s`, plant-cam suite, optional `dx_back_max`.
8. New sha16 **only** after Prefer FAIL fair set beats E7lock honestly (and notes vs T5_06 / T4_09).
9. No HW/MFG plant ask / no spend.

## Stack (Controls owns)

- Train: `train_t5b_alip.py` (or documented T5-B mode); may compose ALIP/DCM/Placo reverse prior + residual PPO  
- Artifacts: `previews/ainex_walk/iterate/learned_gate_e/` (`ppo_t5b_*.zip`, logs, history)  
- Scorer: `score_gate_q.py` + T5-B ckpt env  
- Budget: ≤4 h **or** ≤2e6 steps (first hit); early stop on `ret_ok` both

## Success / Prefer FAIL

- **Pass path:** `ret_ok` both under bars (skate p95≤0.18 **both**) + ε + apps + tip + cf → continuous watch → AI Q03 lock path.
- **Prefer FAIL:** budget wall; apps traded; Root B / ε; soft bars; S2 twin collapse; plant invent.
- **Escalation if Prefer FAIL:** **T5-C** (BC teacher + multi-seed) → **T5-D** only with Dave. Soft-pass off. AI cospec each step. **Do not park Gate Q** without Dave.

## Spin

Controls: train **only under this cospec**. Fair Prefer FAIL vs E7lock (+ note vs T5_06 / T4_09). Same-turn Prefer FAIL score or `ret_ok` pack when ready. HW/MFG: freeze holds — no plant ask.
