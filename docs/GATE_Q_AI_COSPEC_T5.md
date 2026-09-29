# Gate Q AI cospec — T5-A skate-primary reverse residual (~17:21 BST)

**To:** Controls · **From:** AI · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen  
**Dave:** deeper reverse research **GREENLIT**

## Veto?

**No veto — fair new family.** Aligns AI `docs/GATE_Q_AI_RESEARCH_T5.md` **T5-A** + Controls `docs/GATE_Q_AI_COSPEC_T5_PROPOSAL.md` + `docs/GATE_Q_RETREAT_RESEARCH_T5.md`. Proposal **accepted** on must-holds + stack note.

**≠ closed families:** R1–R5 · S1 CSF50 · S2 ALIP Δu_fp · S3 REV-PHASE as twins on frozen E7lock · T4 clear/Root-B-primary residual (T5 **inverts** to skate-primary while constraining cf).

## Family

**T5-A SKATE-PRIMARY** (tag `T5`). New reverse residual / thin finetune — bootstrap **T4_09 near-miss zip if present**, else E7lock `ppo_gate_e_locked.zip` sha16 `9ffaa1a21b607bf6`. Reward dominated by **skate mean + heavy p95**; clear_frac≥0.55 as **floor/constraint** (already near on T4_09); plant-cam ε + Root B heavy Prefer FAIL penalties; tip≥8 floor. Clear-only residual (FOOT-LIFT ≥2 cm): optional late-swing **TDVM/SLR** + optional ankle DF. Planted = cancel×**0.70** + vel-oppose only — **zero** planted soft-XY credit. Dual-ckpt eval: approach=E7lock · retreat=T5 via `GATE_Q_T5_CKPT`. Soft-pass **never**.

## Baseline (held)

| Item | Value |
|------|--------|
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` |
| Rollback ckpt | E7lock `9ffaa1a21b607bf6` until Prefer-FAIL beat |
| Near-miss bootstrap | T4_09 (cf 0.687/0.821 · skate p95~0.205/0.217) |
| CLEAR_TRACK_SWING | 1 on clear sole only |
| cancel | ×0.70 (do not reopen ↑ / FREEZE / qvel0 / damp↑) |
| R*/S1–S3 twins | **OFF** |

## Must-holds

1. Soft-pass **off**. Skate ≤0.08/0.18 · clear_frac≥0.55 · tip≥8 — **no soften**.
2. Clear authority only while sole clear. No planted soft-XY stride credit.
3. Plant-cam ε: Prefer FAIL if plant-window cam >**0.02** or cum >**0.05**/bout.
4. Apps hold ~0.548/0.630 or Prefer FAIL if traded.
5. Skate-primary reward — Prefer FAIL if cf/apps traded for fake skate.
6. SLR / DXB / ANK add-ons clear-only only; log `v_sw_tang_preTD`, `dx_back_max` when used.
7. No plant invent · no Path A · no vision-in-walk · no HW/MFG plant ask.
8. Logs every score: `ckpt_id`, skate mean/p95, clear_frac, tip, apps, `cam@preSS`, `dxc_pre`, `planted_middle_s`, plant-cam suite, optional SLR/DXB fields.
9. New sha16 **only** after Prefer FAIL fair set beats E7lock honestly.

## Stack (Controls owns)

- Train: `train_t5_skate.py` (or documented T5 mode on `train_t4_reverse.py`) · `env_ainex.py` reverse residual  
- Artifacts: `previews/ainex_walk/iterate/learned_gate_e/` (`ppo_t5_*.zip`, `t5_train.log`, history)  
- Scorer: `score_gate_q.py` + `GATE_Q_T5_CKPT`  
- Budget: ≤4 h **or** ≤2e6 steps (first hit); early stop on `ret_ok` both

## Creative Prefer-FAIL opts (in-family)

| Opt | Idea | Guard |
|-----|------|-------|
| SLR | Late-swing tangential match / swing-backward | Clear-only |
| DXB | `|dx_back|` cap / short-step curriculum | tip≥8; cf floor |
| ANK | Ankle DF residual clear-only soft TD | Not planted soft-XY |

## Success / Prefer FAIL

- **Pass path:** `ret_ok` both under bars (esp. skate p95≤0.18 **both**) + ε + apps + tip + cf → Controls continuous watch → AI Q03 lock path.
- **Prefer FAIL:** budget wall; apps traded; Root B / ε; soft bars; R*/S1–S3 twin collapse; plant invent.
- **Escalation:** T5-B (ALIP/DCM+ankle new weights) → T5-C (BC teacher + multi-seed) → T5-D only with Dave. Soft-pass off. AI cospec each step.

## Spin

Controls: train **only under this cospec**. Fair Prefer FAIL vs E7lock (+ note vs T4_09). Same-turn Prefer FAIL score or `ret_ok` pack when ready. HW/MFG: freeze holds — no plant ask.
