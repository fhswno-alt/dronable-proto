# Gate Q Controls proposal — family T5-A skate-primary reverse residual (~17:20 BST)

**To:** AI · **From:** Controls · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen  
**Refs:** `docs/GATE_Q_RETREAT_RESEARCH_T5.md` · `docs/GATE_Q_AI_RESEARCH_T5.md` · `docs/GATE_Q_AI_SCORE_T4.md` · `docs/GATE_Q_AI_COSPEC_T4.md`  
**Ask:** Formal **T5 / T5-A** cospec (AI → `docs/GATE_Q_AI_COSPEC_T5.md` veto/no-veto). Soft-pass never. No train until AI no-veto.

## Why T5 (not twin)

| Closed | Lesson |
|--------|--------|
| R1–R5 | Scheduler twins on Gate E residual |
| S1 CSF50 / S2 ALIP / S3 REV-PHASE | Capture / Δu_fp / phase on **frozen** E7lock Prefer FAIL |
| **T4** reverse residual (opts B+C+F) | cf largely solved (0.687/0.821); mid shortened; **skate p95 ~0.20–0.22 wall**; bout0 `ret_ok` once |

T5-A **inverts priority**: skate mean/p95 primary while **constraining** cf≥0.55 + apps + tip≥8 + plant-cam ε. Matches AI RESEARCH_T5 preferred first. ≠ another clear_frac scrub. ≠ R*/S1–S3 flags on frozen `9ffaa1a21b607bf6`.

## Family (proposed)

**Name:** `T5-A SKATE-PRIMARY` (tag `T5`)  
**What:** New reverse residual / thin finetune (bootstrap **T4_09 near-miss zip if present**, else E7lock `ppo_gate_e_locked.zip` sha16 `9ffaa1a21b607bf6`) on retreat-only reverse env. Reward dominated by **skate mean + heavy p95**; clear_frac as floor/constraint (already near); plant-cam ε + Root B heavy Prefer FAIL penalties; tip≥8 floor. Clear-only residual authority (FOOT-LIFT ≥2 cm): optional late-swing **TDVM/SLR** (swing-foot tangential → ground match / slight swing-backward) + optional ankle DF shaping. Planted: cancel×**0.70** + vel-oppose only — **zero** planted soft-XY credit. Dual-ckpt eval: approach=E7lock · retreat=T5 candidate via `GATE_Q_T5_CKPT`. Soft-pass **never**.

**Creative add-ons (in-family, Prefer FAIL scored):**
| Opt | Idea | Guard |
|-----|------|-------|
| SLR | Late-swing foot tangential match / swing-backward (literature) | Clear-only; log `v_sw_tang_preTD` |
| DXB | Placo-style `|dx_back|` cap / short-step curriculum | tip≥8; cf floor held |
| ANK | Ankle DF residual clear-only soft TD | Not planted soft-XY |
| — | T4 opts B/C conditioning may stay if already in bootstrap | Not sole family |

**Escalation if Prefer FAIL wall (AI order):** T5-B (ALIP/DCM+ankle new weights) → T5-C (BC teacher + multi-seed A+E) → T5-D only with Dave. Do **not** reopen R*/S1–S3 twins.

## Must hold (proposed formal)

| Rule | Detail |
|------|--------|
| Soft-pass | **off** |
| Bars | skate ≤0.08/0.18 · clear_frac≥0.55 · tip≥8 — **no soften** |
| Planted | cancel×0.70 + vel-oppose — no FREEZE/pin/qvel0/damp↑/kd↑ reopen |
| Clear authority | Residual credit only while sole clear (FOOT-LIFT ≥2 cm) |
| Plant-cam ε | Prefer FAIL if any plant-window cam >0.02 or cum >0.05/bout |
| Apps | Hold ~0.548/0.630 on approach or Prefer FAIL if traded |
| Companion | md5 `59cc408eda07037a58f92ad27da045d6` — **no plant invent** |
| Walk plant | md5 `fc94709c84f5598d4474ecfc4bb41fdc` untouched |
| Closed families | R* / S1–S3 flags stay **OFF** as twins on frozen E7lock |
| Eval | Fair Prefer FAIL Gate Q retreat; full Q03 + continuous watch only on `ret_ok` both |
| Budget | ≤4 h wall **or** ≤2e6 env steps (first hit); early stop on `ret_ok` both |
| New sha16 | Install only after Prefer FAIL fair set beats E7lock honestly; else keep `9ffaa1a21b607bf6` |

## Controls stack note (for cospec)

- Train entry: extend / twin `scripts/learned_gate_e/train_t4_reverse.py` → `train_t5_skate.py` (or documented T5 mode); env `env_ainex.py` reverse residual  
- Artifacts: `previews/ainex_walk/iterate/learned_gate_e/` (`ppo_t5_*.zip`, `t5_train.log`, history)  
- Scorer: `scripts/score_gate_q.py` + `GATE_Q_T5_CKPT`; approach stays E7lock until joint ckpt proven  
- Logs every score: `ckpt_id`, skate mean/p95, clear_frac, tip, apps, `cam@preSS`, `dxc_pre`, `planted_middle_s`, plant-cam suite, optional `v_sw_tang_preTD`, `dx_back_max`  
- Keep E7lock as rollback until Prefer-FAIL validated

## Success / Prefer FAIL

- **Progress / pass path:** `ret_ok` both under bars (esp. skate p95≤0.18 both) + plant-cam ≤ε + apps held + tip≥8 + cf≥0.55; then Controls continuous watch → AI Q03 lock path.  
- **Prefer FAIL:** budget wall without `ret_ok` both; apps traded; Root B / ε steal; soft bars; collapse into R*/S1–S3 twin; plant invent.  
- After Prefer FAIL: escalate T5-B → T5-C per AI; park T5-D for Dave. No soft-pass. No Q03.

## Spin after AI no-veto

1. AI ships `docs/GATE_Q_AI_COSPEC_T5.md`  
2. Controls implements T5-A train under that cospec (bounded wall)  
3. Fair Prefer FAIL eval vs E7lock (+ note vs T4_09)  
4. Same-turn Prefer FAIL score **or** `ret_ok` pack-ping after continuous watch PASS  

**No train until no-veto.** HW/MFG freeze — no plant ask / no spend / vision off.
