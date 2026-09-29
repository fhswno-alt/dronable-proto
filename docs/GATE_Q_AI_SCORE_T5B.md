# Gate Q AI score packet — T5-B ALIP/DCM+ankle Prefer FAIL / near-miss (~18:54 BST)

**From:** Controls · Soft-pass **never** · Bars **KEPT** · `ai_can_lock=false` · No Q03
**Cospec:** AI no veto — `docs/GATE_Q_AI_COSPEC_T5B.md` · proposal `docs/GATE_Q_AI_COSPEC_T5B_PROPOSAL.md` · Dave no-park escalate after T5-A

## Family

**T5-B MODEL-BASE+RESIDUAL** (tag `T5B`). ALIP S2S / DCM reverse foothold + ankle NMPC prior as **base** inside **NEW trained weights**, PPO residual closes full-body gap. Mid-swing replan OK in new weights. Skate mean/p95 co-primary with cf floor ≥0.55. Clear-only residual authority; planted cancel×**0.70**. ≠ S2 twin flags on frozen E7lock (`GATE_Q_RET_ALIP_TVR=0`). ≠ reopen R*/S1–S3/T5-A as twins. Dual-ckpt: approach=E7lock · retreat=T5B via `GATE_Q_T5B_CKPT`. Soft-pass **never**.

## Bootstrap + stack

| Item | Value |
|------|--------|
| Bootstrap | **T5_06 near-miss** `ppo_t5_best_T5_06_near_miss.zip` sha16 `3f5533b5d6917b09` (present) |
| Trainer | `scripts/learned_gate_e/train_t5b_alip.py` |
| Env | `AinexT5BModelBaseEnv` (ALIP+DCM+ANK-NMPC base · residual · SLR/DXB/ANK conditioning) |
| Opts ON | **ALIP + DCM + ANK-NMPC** (+ SLR/DXB/ANK · T4 B+C conditioning) |
| cancel | ×0.70 planted · CLEAR_TRACK clear-only · zero planted soft-XY |
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` (untouched) |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` (untouched) |
| Scorer | `score_gate_q.py` + `GATE_Q_T5B_CKPT` (modelbase path ≠ S2 flag) |

## Budget

| Cap | Used |
|-----|------|
| ≤4 h wall OR ≤2e6 steps | **2 002 944 steps** first (wall **2590 s / 0.72 h**) |
| Early stop ret_ok both | **not hit** |

## Baseline T5B_R0 ≡ E7lock

**YES.** apps **0.548/0.630**; bout0 rcf **0.468** skate **0.096/0.240** mid **34.88**; bout1 rcf **0.649** skate **0.098/0.221** mid **26.70**; CSF/REV n=0; ALIP S2 flag OFF.

## T5_06 note (dual-ckpt via GATE_Q_T5B_CKPT — ALIP modelbase ON)

apps✓; rcf **0.562/0.891**; skate 0.060/**0.224** · 0.062/**0.192** — Prefer FAIL (ALIP base on frozen T5_06 weights without fine-tune; original T5_06 SCORE_T5 was cf 0.900/0.894 skate 0.054/0.157 · 0.072/0.224).

## Fair Prefer FAIL probes (T5B_01…T5B_13 + FINAL)

All Prefer FAIL for `ret_ok` both. Soft-pass NEVER. New sha16 **not** installed — E7lock `9ffaa1a21b607bf6` remains frozen in `score_gate_q.CKPT_SHA16` + `ppo_gate_e_locked.zip`.

### Near-miss package (best rank) — T5B_02 @ 0.30e6 · sha16 `a18be16ca662454f`

| | bout0 | bout1 |
|--|-------|-------|
| apps | **0.548** | **0.630** |
| ret cf | **0.856** | **0.847** |
| skate mean/p95 | 0.064/**0.183** | **0.049/0.106** (bars✓) |
| tip_ret | 29.1 | 12.7 |
| mid (PRE_GAP) | 4.88 | 2.62 |
| cam@preSS / dxc_pre | 0.319 / 0.267 | 0.399 / 0.264 |
| T5B_MODELBASE | fp≈−0.028 · n_replan 30/23 · fp_jump≤0.0013 | |
| plant-cam ε (T5B suite) | steal=True (maxG 0.033) | steal=True (maxG 0.022) |
| ret_ok | False (skate p95 0.183 + stepped) | **True** |

**Attack hit:** bout1 skate **under bars** + ret_ok while holding apps + cf≫0.55 + tip≥8 + mid shortened — **bout0 skate p95 0.183** still Prefer FAIL (0.003 over 0.18). Soft-pass off. Prefer FAIL also on T5B plant-cam ε steal (cospec ≤0.02/0.05). vs T5_06: bout1 skate newly under bars (0.106 vs 0.224); bout0 p95 still Prefer FAIL (~0.183 vs T5_06 0.157 under); cf held high both. vs E7lock: cf + skate mean/p95 improved both bouts; not honest beat (no ret_ok both).

### Closest single-bout ret_ok

| probe | bout0 | bout1 | notes |
|-------|-------|-------|-------|
| **T5B_02** ★ | FAIL p95 0.183 | **ret_ok** (cf 0.847 skate **0.049/0.106**) | best rank; ε steal Prefer FAIL |
| **T5B_09** | **ret_ok** (cf 0.853 skate **0.050/0.161**) | FAIL cf 0.344 + skate 0.075/0.240 | bout1 cf traded |
| **T5B_08/10/12/13** | FAIL | **ret_ok** (bout1 skate under) | bout0 skate/cf Prefer FAIL |

### Other notables

- **T5B_01:** bout0 skate under (0.067/0.176) but apps bout1 traded 0.477 Prefer FAIL
- **T5B_05:** apps bout1 **0.0** Prefer FAIL
- **T5B_FINAL:** apps✓; cf 0.533/0.874; skate 0.065/0.221 · 0.061/0.225 — cf floor + p95 Prefer FAIL
- Never skate≤0.08/0.18 **both** bouts while holding E7lock apps + cf≥0.55 both + full ret_ok both + ε

## Disposition

**Prefer FAIL train wall / near-miss.** Soft-pass off. Bars KEPT. `ai_can_lock=false`. No Q03 video. New sha16 **not** installed. E7lock best-known rollback `9ffaa1a21b607bf6`. Park T5-B — escalate path per cospec is **T5-C** (BC teacher + multi-seed) with AI cospec; T5-D only with Dave. Do **not** reopen R*/S1–S3/T5-A twins. No continuous watch (ret_ok both not hit). Soft-pass never.

Artifacts: `GATE_Q_T5B_PROGRESS.md` · `GATE_Q_T5B_*_PROBE.log` · `learned_gate_e/{t5b_train.log,t5b_train_history.json,T5B_DONE.json,ppo_t5b_*.zip,ppo_t5b_best_T5B_02_near_miss.zip}` · this packet · CONTROLS_NOTE / A28_RETOK_DIAG / TABLE / VIDEO_CONFLICT.

## AI disposition (~18:55 BST)

**Agrees Prefer FAIL / near-miss of T5-B only.** Soft-pass never. Bars KEPT. `ai_can_lock=false`. No Q03. New sha16 correctly **not** installed — E7lock kept.

Honest progress: T5B_02 bout1 skate under + ret_ok; bout0 p95 **0.183** Prefer FAIL + ε steal. **Gate Q stays OPEN** — escalate **T5-C** (`docs/GATE_Q_AI_COSPEC_T5C.md` no veto). Do not reopen R*/S1–S3/T5-A twins.
