# Gate Q AI score packet — T5-A SKATE-PRIMARY Prefer FAIL / near-miss (~18:05 BST)

**From:** Controls · Soft-pass **never** · Bars **KEPT** · `ai_can_lock=false` · No Q03
**Cospec:** AI no veto — `docs/GATE_Q_AI_COSPEC_T5.md` · proposal `docs/GATE_Q_AI_COSPEC_T5_PROPOSAL.md` · Dave GREENLIT deeper reverse

## Family

**T5-A SKATE-PRIMARY** (tag `T5`). New reverse residual / thin finetune. Reward dominated by skate mean + heavy p95; clear_frac≥0.55 as floor/constraint; plant-cam ε + Root B Prefer FAIL; tip≥8; clear-only residual. In-family opts **SLR/TDVM + ANK DF + DXB**. Keep T4 B+C conditioning. ≠ R1–R5 / S1 CSF50 / S2 ALIP / S3 REV-PHASE / T4 clear-primary twins. Flags **OFF**. Dual-ckpt: approach=E7lock · retreat=T5 via `GATE_Q_T5_CKPT`.

## Bootstrap + stack

| Item | Value |
|------|--------|
| Bootstrap | **T4_09 near-miss** `ppo_t4_best_T4_09_near_miss.zip` sha16 `9b85a2c678477500` (present) |
| Trainer | `scripts/learned_gate_e/train_t5_skate.py` |
| Env | `AinexT5SkateEnv` (skate-primary · SLR/ANK/DXB · clear-only) |
| Opts ON | **SLR + ANK + DXB** (+ T4 B neg-Vx · C rev-phase conditioning) |
| cancel | ×0.70 planted · CLEAR_TRACK clear-only · zero planted soft-XY |
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` (untouched) |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` (untouched) |
| Scorer | `score_gate_q.py` + `GATE_Q_T5_CKPT` |

## Budget

| Cap | Used |
|-----|------|
| ≤4 h wall OR ≤2e6 steps | **2 002 944 steps** first (wall **2579 s / 0.72 h**) |
| Early stop ret_ok both | **not hit** |

## Baseline T5_R0 ≡ E7lock

**YES.** apps **0.548/0.630**; bout0 rcf **0.468** skate **0.096/0.240** mid **34.88**; bout1 rcf **0.649** skate **0.098/0.221** mid **26.70**; REV/CSF/ALIP n=0; plant-cam steal=False.

## T4_09 note (dual-ckpt via GATE_Q_T5_CKPT)

apps✓; rcf **0.687/0.821**; skate 0.071/**0.205** · 0.069/**0.217**; tip 27.6/21.5; mid 4.8/2.1 — Prefer FAIL skate p95 (matches SCORE_T4).

## Fair Prefer FAIL probes (T5_01…T5_13 + FINAL)

All Prefer FAIL for `ret_ok` both. Soft-pass NEVER. New sha16 **not** installed — E7lock `9ffaa1a21b607bf6` remains frozen in `score_gate_q.CKPT_SHA16` + `ppo_gate_e_locked.zip`.

### Near-miss package (best rank) — T5_06 @ 0.90e6

| | bout0 | bout1 |
|--|-------|-------|
| apps | **0.548** | **0.630** |
| ret cf | **0.900** | **0.894** |
| skate mean/p95 | **0.054/0.157** (bars✓) | 0.072/**0.224** |
| tip_ret | 23.8 | 15.0 |
| mid (PRE_GAP) | 5.68 | 3.08 |
| cam@preSS / dxc_pre | 0.298 / 0.314 | 0.270 / 0.173 |
| plant-cam ε steal | False | False |
| ret_ok | False (cadence/stepped) | False (skate p95) |

cf≫0.55 both · apps held · tip≥8 · bout0 **skate under bars** (first T5) · mid shortened vs E7lock — **bout1 skate p95** still >0.18. Soft-pass off. vs T4_09: cf improved (0.90/0.89 vs 0.69/0.82); bout0 skate newly under bars; bout1 p95 still Prefer FAIL (~0.224 vs T4_09 0.217).

### Closest single-bout ret_ok

| probe | bout0 | bout1 | notes |
|-------|-------|-------|-------|
| **T5_08** @ 1.20e6 | **ret_ok** (cf 0.829 skate **0.059/0.172**) | FAIL cf 0.479 + skate 0.103/0.205 | apps✓; bout1 cf traded below floor |
| **T5_09** @ 1.35e6 | FAIL p95 0.229 | **ret_ok** (cf 0.830 skate **0.047/0.158**) | bout1 **apps traded 0.0** Prefer FAIL |

### Other notables

- **T5_02:** skate mean under (0.062/0.063) but p95 0.238/0.221 + cf below floor
- **T5_03/05/09:** apps traded Prefer FAIL (bout1 approach collapse after bad retreat)
- **T5_FINAL:** apps✓; cf 0.807/0.486; skate 0.060/0.202 · 0.057/0.212 — bout1 cf floor Prefer FAIL; p95 both Prefer FAIL
- Never skate≤0.08/0.18 **both** bouts while holding E7lock apps + cf≥0.55 both + full ret_ok both

## Disposition

**Prefer FAIL train wall / near-miss.** Soft-pass off. Bars KEPT. `ai_can_lock=false`. No Q03 video. New sha16 **not** installed. E7lock best-known rollback `9ffaa1a21b607bf6`. Park T5-A — escalate path per cospec is T5-B (ALIP/DCM+ankle new weights) → T5-C (BC+multi-seed) only with AI cospec; T5-D only with Dave. Do **not** reopen R*/S1–S3. No continuous watch (ret_ok both not hit).

Artifacts: `GATE_Q_T5_PROGRESS.md` · `GATE_Q_T5_*_PROBE.log` · `learned_gate_e/{t5_train.log,t5_train_history.json,T5_DONE.json,ppo_t5_*.zip,ppo_t5_best_T5_06_near_miss.zip}` · this packet · CONTROLS_NOTE / A28_RETOK_DIAG / TABLE / VIDEO_CONFLICT.

## AI disposition (~18:06 BST)

**Agrees Prefer FAIL / near-miss of T5-A only.** Soft-pass never. Bars KEPT. `ai_can_lock=false`. No Q03. New sha16 correctly **not** installed — E7lock kept.

Honest progress: T5_06 cf **0.900/0.894**; bout0 skate under bars (0.054/0.157); bout1 p95 still Prefer FAIL. **Gate Q stays OPEN** per Dave — escalate **T5-B** (`docs/GATE_Q_AI_COSPEC_T5B.md` no veto). Do not reopen R*/S1–S3.
