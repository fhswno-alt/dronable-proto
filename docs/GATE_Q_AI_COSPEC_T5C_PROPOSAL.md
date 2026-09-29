# Gate Q Controls proposal — family T5-C Prefer FAIL (reverse teacher BC + multi-seed) (~18:56 BST)

**To:** AI Scientist · **From:** Controls · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen  
**Dave:** no park after T5-B — **escalate T5-C now**  
**Refs:** `docs/GATE_Q_AI_SCORE_T5B.md` · `docs/GATE_Q_AI_COSPEC_T5B.md` · `docs/GATE_Q_AI_COSPEC_T5.md` · `docs/GATE_Q_AI_RESEARCH_T5.md` · `docs/GATE_Q_RETREAT_RESEARCH_T5.md` · `docs/GATE_Q_AI_COSPEC_T5B_PROPOSAL.md`  
**Ask:** Formal **T5-C** cospec (AI → `docs/GATE_Q_AI_COSPEC_T5C.md` veto/no-veto). Soft-pass never. **No train until AI no-veto.**

## Why T5-C (not twin, not park)

| Closed | Disposition | Lesson |
|--------|-------------|--------|
| R1–R5 | Prefer FAIL / veto | Scheduler twins on Gate E residual |
| S1 CSF50 / S2 ALIP / S3 REV-PHASE | Prefer FAIL on **frozen** E7lock | Capture / Δu_fp / phase flip ≠ reverse-native dynamics |
| T4 reverse residual | Prefer FAIL near-miss | cf largely solved; skate p95 wall; **opts A (BC) + E (multi-seed) left OFF** |
| **T5-A SKATE-PRIMARY** | Prefer FAIL (`GATE_Q_AI_SCORE_T5.md`) | Bout0 skate under once; bout1 skate p95 Prefer FAIL |
| **T5-B MODEL-BASE+RESIDUAL** | Prefer FAIL train wall / near-miss (`GATE_Q_AI_SCORE_T5B.md` ~18:54) | Bout1 skate **under bars** + ret_ok repeatedly (T5B_02 ★ 0.049/0.106); **bout0 skate p95 0.183** (0.003 over 0.18) + plant-cam ε steal Prefer FAIL; never skate≤0.08/0.18 **both** + apps+cf≥0.55 + ret_ok both + ε |

T5-B ALIP/DCM+ANK-NMPC base + residual hit the wall Dave greenlit escalate past. **T5-C** is the next AI-ranked family: **finish T4 creative opts A+E** — reverse **teacher BC** + **multi-seed** Prefer FAIL wall — framed **skate-honest** (hold T5B_02 bout1 skate PASS + cf, attack remaining **bout0 skate p95** + **plant-cam ε**).

**≠ T5-B twin:** Not another ALIP/DCM/ankle model-base fine-tune without teacher BC + multi-seed curriculum.  
**≠ T5-A twin:** Not skate-reward-only thin finetune flags on frozen.  
**≠ S2 / R* / S1 / S3:** Flags stay **OFF** on frozen E7lock.

## Family (proposed)

**Name:** `T5-C TEACHER-BC+MULTI-SEED` (tag `T5C`)  
**What:** Reverse **teacher / oracle** behavioral cloning (CSF / Placo-style short reverse / sim teleop oracle — T4 opt **A**) → residual RL under Prefer FAIL, with **multi-seed** Prefer FAIL wall (T4 opt **E**, 2–3 seeds; best seed must still clear bars — no cherry-pick soft metrics). Optional Walk-This-Way-style footstep / timing constraints for retreat Δ (vision off). Reward Prefer FAIL honest: skate mean + heavy p95 co-primary with cf floor ≥0.55; plant-cam ε hard Prefer FAIL. Soft-pass **never**.

**Bootstrap (priority order — present on disk):**
1. **T5B_02 near-miss** `ppo_t5b_best_T5B_02_near_miss.zip` sha16 `a18be16ca662454f` (bout1 skate **0.049/0.106** PASS + ret_ok · cf 0.856/0.847 · apps 0.548/0.630; bout0 p95 Prefer FAIL **0.183**; plant-cam ε steal Prefer FAIL)
2. else **T5_06 near-miss** `ppo_t5_best_T5_06_near_miss.zip` sha16 `3f5533b5d6917b09`
3. else **T4_09 near-miss** `ppo_t4_best_T4_09_near_miss.zip` sha16 `9b85a2c678477500`
4. else E7lock `ppo_gate_e_locked.zip` sha16 `9ffaa1a21b607bf6`

**Dual-ckpt eval:** approach=**E7lock** · retreat=**T5C** via `GATE_Q_T5C_CKPT` (or documented T5C alias). Soft-pass never. New sha16 **not** installed until Prefer FAIL fair set beats E7lock honestly.

**Attack focus:** Close remaining **bout0 skate p95 ≤0.18** + **plant-cam ε** (plant-window cam ≤0.02 · cum ≤0.05/bout) while **holding** **bout1 skate PASS** (mean≤0.08 / p95≤0.18) + **cf≥0.55 both** + apps ~0.548/0.630 + tip≥8. Prefer FAIL if bout1 skate or cf traded to fake bout0 skate / ε.

### Creative Prefer-FAIL opts (in-family; finish T4 A+E)

| Opt | Idea | Guard |
|-----|------|-------|
| **A — BC teacher** | Oracle CSF / Placo short-reverse / sim-teleop reverse trajectories → BC warm-start → residual RL | Teacher **never** credits plant XY as stepped; clear sole ≥2 cm for stride credit; log `bc_loss`, `teacher_src`, `n_demo` |
| **E — multi-seed** | 2–3 independent seeds under same cospec; Prefer FAIL wall per seed; report all | Best seed must still clear **full** bars; no cherry-pick soft metrics / soft-pass |
| FOOTSTEP | Optional Walk-This-Way-style footstep location / timing constraints for retreat Δ | tip≥8; cf floor; vision off |
| — | May carry T5-B ALIP/DCM/ANK or T5-A SLR/DXB as **conditioning** if bootstrap carries them | Not sole family; not S2 / T5-A / T5-B twin flags on frozen |

Planted = cancel×**0.70** + vel-oppose only — **zero** planted soft-XY credit. CLEAR_TRACK clear-only.

## Must hold (proposed formal)

| Rule | Detail |
|------|--------|
| Soft-pass | **OFF** — never |
| Bars | skate ≤**0.08/0.18** · clear_frac≥**0.55** · tip≥**8** — **KEPT, no soften** |
| Planted | cancel×**0.70** + vel-oppose — **no** FREEZE / pin / qvel0 / damp↑ / kd↑ / cancel↑ reopen |
| Clear authority | Residual / BC stride credit **only** while sole clear (FOOT-LIFT ≥2 cm) |
| Teacher honesty | Prefer FAIL if teacher / BC demo banks plant-window cam or credits planted soft-XY |
| Plant-cam ε | Prefer FAIL if any plant-window cam >**0.02** or cum >**0.05**/bout |
| Apps | Hold ~**0.548/0.630** on approach or Prefer FAIL if traded |
| Companion plant | md5 `59cc408eda07037a58f92ad27da045d6` — **no plant invent** |
| Walk plant | md5 `fc94709c84f5598d4474ecfc4bb41fdc` untouched |
| Closed twins | **R\*** / **S1–S3** / **T5-A** / **T5-B** as twin flags on frozen E7lock stay **OFF** |
| ≠ S2 / T5-B twin | No `GATE_Q_RET_ALIP_TVR*` (or kin) as frozen-ckpt twin; ALIP/DCM live only if carried inside **trained T5C stack**, not as sole reopen |
| Eval | Fair Prefer FAIL Gate Q retreat; full Q03 + continuous watch only on `ret_ok` both |
| Budget | ≤**4 h** wall **or** ≤**2e6** env steps (first hit); early stop on `ret_ok` both; multi-seed shares wall (log allocate) |
| Dual-ckpt | approach=E7lock · retreat=T5C |
| Bootstrap | T5B_02 → T5_06 → T4_09 → E7lock (priority above) |
| Attack | bout0 skate p95 + plant-cam ε while holding bout1 skate PASS + cf both |
| New sha16 | Install **only** after Prefer FAIL fair set beats E7lock honestly; else keep `9ffaa1a21b607bf6` |
| Out | Path A spend · vision-in-walk · HW/MFG plant ask · soft bars · lock from skip-video · plant invent |

## Controls stack note (for cospec)

- Train entry: new `scripts/learned_gate_e/train_t5c_teacher.py` (or documented T5C mode) · BC warm-start from reverse teacher demos → residual PPO; multi-seed Prefer FAIL wall — **not** S2 / T5-A / T5-B twin flags on frozen ckpt  
- Artifacts: `previews/ainex_walk/iterate/learned_gate_e/` (`ppo_t5c_*.zip`, `t5c_train.log`, history, per-seed best, best near-miss zip)  
- Scorer: `scripts/score_gate_q.py` + `GATE_Q_T5C_CKPT`; approach stays E7lock until joint ckpt proven  
- Logs every score: `ckpt_id`, `seed`, `teacher_src` / `bc_loss`, skate mean/p95, clear_frac, tip, apps, `cam@preSS`, `dxc_pre`, `planted_middle_s`, plant-cam suite, optional footstep / conditioning fields  
- Keep E7lock as rollback until Prefer-FAIL validated  
- **No train in this proposal turn** — wait AI no-veto cospec

## Success / Prefer FAIL

- **Pass path:** `ret_ok` both under bars (esp. skate p95≤0.18 **both**, holding T5B_02-class bout1 skate + cf) + plant-cam ≤ε + apps held + tip≥8 → Controls continuous watch → AI Q03 lock path.  
- **Prefer FAIL:** budget wall without `ret_ok` both; apps traded; bout1 skate/cf traded for bout0; Root B / ε steal; soft bars; collapse into S2 / R* / T5-A / T5-B twin on frozen; teacher plant-XY credit; cherry-pick soft multi-seed; plant invent.  
- **Escalation after Prefer FAIL:** **T5-D** only with Dave (architecture / plant). Soft-pass off. Do **not** reopen R*/S1–S3/T5-A/T5-B twins. Do **not** park Gate Q without Dave.

## Spin after AI no-veto

1. AI ships `docs/GATE_Q_AI_COSPEC_T5C.md` (veto / no-veto)  
2. Controls implements T5-C train under that cospec (bounded wall) — **not before**  
3. Fair Prefer FAIL eval vs E7lock (+ note vs T5B_02 / T5_06 / T4_09)  
4. Same-turn Prefer FAIL score **or** `ret_ok` pack-ping after continuous watch PASS  

**No train until no-veto.** HW/MFG freeze — no plant ask / no spend / vision off. Soft-pass never.
