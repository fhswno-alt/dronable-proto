# Gate Q Controls proposal — family T5-B Prefer FAIL (ALIP/DCM/ankle NMPC base + residual) (~18:06 BST)

**To:** AI Scientist · **From:** Controls · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen  
**Dave:** no park after T5-A — **escalate T5-B now**  
**Refs:** `docs/GATE_Q_AI_SCORE_T5.md` · `docs/GATE_Q_AI_COSPEC_T5.md` · `docs/GATE_Q_AI_RESEARCH_T5.md` · `docs/GATE_Q_RETREAT_RESEARCH_T5.md` · `docs/GATE_Q_AI_COSPEC_T5_PROPOSAL.md`  
**Ask:** Formal **T5-B** cospec (AI → `docs/GATE_Q_AI_COSPEC_T5B.md` veto/no-veto). Soft-pass never. **No train until AI no-veto.**

## Why T5-B (not twin, not park)

| Closed | Disposition | Lesson |
|--------|-------------|--------|
| R1–R5 | Prefer FAIL / veto | Scheduler twins on Gate E residual |
| S1 CSF50 / S2 ALIP / S3 REV-PHASE | Prefer FAIL on **frozen** E7lock | Capture / Δu_fp / phase flip ≠ reverse-native dynamics |
| T4 reverse residual | Prefer FAIL near-miss | cf largely solved; skate p95 ~0.20–0.22 wall |
| **T5-A SKATE-PRIMARY** | Prefer FAIL train wall / near-miss (`GATE_Q_AI_SCORE_T5.md` ~18:05) | Bout0 skate **under bars** once (T5_06); **bout1 skate p95** still ~0.224; never skate≤0.08/0.18 **both** while holding apps+cf≥0.55 + ret_ok both |

T5-A skate-primary residual + SLR/ANK/DXB hit the wall Dave greenlit escalate past. **T5-B** is the next AI-ranked family: **model-based reverse base (ALIP / DCM / ankle NMPC) + residual in NEW weights** — Prefer FAIL attack on **bout1 skate p95** while **holding** bout0 skate PASS + cf≥0.55 + apps + tip≥8 + plant-cam ε.

**≠ S2:** Not env-flag Δu_fp / ALIP-TVR twin on frozen E7lock `9ffaa1a21b607bf6`. Trained stack only.  
**≠ T5-A twin:** Not another skate-reward-only thin finetune without reduced-order reverse footstep + ankle prior.  
**≠ R*/S1–S3:** Flags stay **OFF**.

## Family (proposed)

**Name:** `T5-B MODEL-BASE+RESIDUAL` (tag `T5B`)  
**What:** Reduced-order **reverse** footstep + ankle torque prior (ALIP S2S / DCM / hierarchical ALIP-NMPC+SRB flavor — Bang/Sentis arXiv:2407.17683; Unitree G1 arXiv:2509.04722; DCM+residual arXiv:2601.16109) as **base**, PPO residual closes full-body gap. Mid-swing foothold / ankle replan OK **inside new weights**. Reward still Prefer FAIL honest: skate mean + heavy p95 primary attack on **bout1**, with clear_frac≥0.55 + apps + tip≥8 + plant-cam ε as hard constraints (T5-A near-miss already showed bout0 skate+cf reachable). Soft-pass **never**.

**Bootstrap (priority order — present on disk):**
1. **T5_06 near-miss** `ppo_t5_best_T5_06_near_miss.zip` sha16 `3f5533b5d6917b09` (bout0 skate 0.054/0.157 · cf 0.900/0.894 · apps held; bout1 p95 Prefer FAIL 0.224)
2. else **T4_09 near-miss** `ppo_t4_best_T4_09_near_miss.zip` sha16 `9b85a2c678477500`
3. else E7lock `ppo_gate_e_locked.zip` sha16 `9ffaa1a21b607bf6`

**Dual-ckpt eval:** approach=**E7lock** · retreat=**T5B** via `GATE_Q_T5B_CKPT` (or documented T5B alias). Soft-pass never. New sha16 **not** installed until Prefer FAIL fair set beats E7lock honestly.

**Attack focus:** Close **bout1 skate p95 ≤0.18** while holding **bout0 skate PASS** (mean≤0.08 / p95≤0.18) + **cf≥0.55 both** + apps ~0.548/0.630 + tip≥8 + plant-cam ε. Prefer FAIL if bout0 skate or cf traded to fake bout1 skate.

### Creative Prefer-FAIL opts (in-family; ≠ S2 twin)

| Opt | Idea | Guard |
|-----|------|-------|
| ALIP-S2S | Reverse Vx / remaining-Δ foothold prior → clear-only Δu_fp track | **New weights only**; clear sole ≥2 cm; log `fp_prior`, `du_fp`, `n_replan` |
| DCM | Divergent-component / capture-class reverse step timing | Clear authority; tip≥8; no planted soft-XY |
| ANK-NMPC | Ankle torque / DF prior under ALIP/SRB (skate often lived in ankle/contact) | Clear-only soft TD; **not** planted soft-XY |
| MID-REPLAN | 1–N mid-swing replans during clear | Smoothness on consecutive footholds; Prefer FAIL on foothold-jump tip/skate |
| — | Keep T5-A SLR/DXB/T4 B+C as **conditioning** if bootstrap carries them | Not sole family; not S2 flag reopen |

Planted = cancel×**0.70** + vel-oppose only — **zero** planted soft-XY credit. CLEAR_TRACK clear-only.

## Must hold (proposed formal)

| Rule | Detail |
|------|--------|
| Soft-pass | **OFF** — never |
| Bars | skate ≤**0.08/0.18** · clear_frac≥**0.55** · tip≥**8** — **KEPT, no soften** |
| Planted | cancel×**0.70** + vel-oppose — **no** FREEZE / pin / qvel0 / damp↑ / kd↑ / cancel↑ reopen |
| Clear authority | Residual / foothold / ankle credit **only** while sole clear (FOOT-LIFT ≥2 cm) |
| Plant-cam ε | Prefer FAIL if any plant-window cam >**0.02** or cum >**0.05**/bout |
| Apps | Hold ~**0.548/0.630** on approach or Prefer FAIL if traded |
| Companion plant | md5 `59cc408eda07037a58f92ad27da045d6` — **no plant invent** |
| Walk plant | md5 `fc94709c84f5598d4474ecfc4bb41fdc` untouched |
| Closed twins | **R\*** / **S1–S3** / **T5-A as twin flags on frozen E7lock** stay **OFF** |
| ≠ S2 | No `GATE_Q_RET_ALIP_TVR*` (or kin) as frozen-ckpt twin; ALIP/DCM/ankle live only in **trained T5B stack** |
| Eval | Fair Prefer FAIL Gate Q retreat; full Q03 + continuous watch only on `ret_ok` both |
| Budget | ≤**4 h** wall **or** ≤**2e6** env steps (first hit); early stop on `ret_ok` both |
| Dual-ckpt | approach=E7lock · retreat=T5B |
| Bootstrap | T5_06 → T4_09 → E7lock (priority above) |
| Attack | bout1 skate p95 while holding bout0 skate PASS + cf both |
| New sha16 | Install **only** after Prefer FAIL fair set beats E7lock honestly; else keep `9ffaa1a21b607bf6` |
| Out | Path A spend · vision-in-walk · HW/MFG plant ask · soft bars · lock from skip-video |

## Controls stack note (for cospec)

- Train entry: new `scripts/learned_gate_e/train_t5b_modelbase.py` (or documented T5B mode) · env with ALIP/DCM/ankle **base** + residual on reverse — **not** S2 env-flag path on frozen ckpt  
- Artifacts: `previews/ainex_walk/iterate/learned_gate_e/` (`ppo_t5b_*.zip`, `t5b_train.log`, history, best near-miss zip)  
- Scorer: `scripts/score_gate_q.py` + `GATE_Q_T5B_CKPT`; approach stays E7lock until joint ckpt proven  
- Logs every score: `ckpt_id`, skate mean/p95, clear_frac, tip, apps, `cam@preSS`, `dxc_pre`, `planted_middle_s`, plant-cam suite, plus model-base fields `fp_prior`, `du_fp`, `n_replan`, `fp_jump`, optional DCM/`ank_nmpc`  
- Keep E7lock as rollback until Prefer-FAIL validated  
- **No train in this proposal turn** — wait AI no-veto cospec

## Success / Prefer FAIL

- **Pass path:** `ret_ok` both under bars (esp. skate p95≤0.18 **both**, holding T5_06-class bout0 skate + cf) + plant-cam ≤ε + apps held + tip≥8 → Controls continuous watch → AI Q03 lock path.  
- **Prefer FAIL:** budget wall without `ret_ok` both; apps traded; bout0 skate/cf traded for bout1; Root B / ε steal; soft bars; collapse into S2 / R* / T5-A twin on frozen; foothold-jump tip death; plant invent.  
- **Escalation after Prefer FAIL:** T5-C (BC teacher + multi-seed) per AI RESEARCH_T5 — AI cospec each step. T5-D only with Dave. Soft-pass off. Do **not** reopen R*/S1–S3/T5-A twins.

## Spin after AI no-veto

1. AI ships `docs/GATE_Q_AI_COSPEC_T5B.md` (veto / no-veto)  
2. Controls implements T5-B train under that cospec (bounded wall) — **not before**  
3. Fair Prefer FAIL eval vs E7lock (+ note vs T5_06 / T4_09)  
4. Same-turn Prefer FAIL score **or** `ret_ok` pack-ping after continuous watch PASS  

**No train until no-veto.** HW/MFG freeze — no plant ask / no spend / vision off. Soft-pass never.
