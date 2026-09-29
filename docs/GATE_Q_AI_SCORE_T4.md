# Gate Q AI score packet — T4 reverse-native ckpt Prefer FAIL / near-miss (~17:04 BST)

**From:** Controls · Soft-pass **never** · Bars **KEPT** · `ai_can_lock=false` · No Q03
**Cospec:** AI no veto — `docs/GATE_Q_AI_COSPEC_T4.md` · Dave GREENLIT · Prefer FAIL train

## Family

New reverse-native residual (thin finetune from E7lock). ≠ R1–R5 / S1 CSF50 / S2 ALIP / S3 REV-PHASE as twins on frozen ckpt. Flags **OFF** throughout. Dual-ckpt eval: approach=E7lock · retreat=T4 candidate via `GATE_Q_T4_CKPT`.

## Bootstrap + opts

| Item | Value |
|------|--------|
| Bootstrap | `ppo_gate_e_locked.zip` sha16 `9ffaa1a21b607bf6` |
| Trainer | `scripts/learned_gate_e/train_t4_reverse.py` |
| Env | `AinexReverseResidualEnv` (clear-only residual · reverse hip/step) |
| Opts ON | **B** neg-Vx library (obs[40]) · **C** rev-phase in NEW weights · **F** Root B / plant-ε proxy |
| Opts OFF | A (BC teacher) · D · E (multi-seed — first seed used full wall) |
| cancel | ×0.70 planted · CLEAR_TRACK clear-only |
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` (untouched) |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` (untouched) |

## Budget

| Cap | Used |
|-----|------|
| ≤4 h wall OR ≤2e6 steps | **2 000 896 steps** first (wall **3130 s / 0.87 h**) |
| Early stop ret_ok both | **not hit** |

## Baseline T4_R0 ≡ E7lock

**YES.** apps **0.548/0.630**; bout0 rcf **0.468** skate **0.096/0.240** mid **34.88**; bout1 rcf **0.649** skate **0.098/0.221** mid **26.70**; REV/CSF/ALIP n=0; plant-cam steal=False.

## Fair Prefer FAIL probes (T4_01…T4_13 + FINAL)

All Prefer FAIL for `ret_ok` both. Soft-pass NEVER. New sha16 **not** installed — E7lock `9ffaa1a21b607bf6` remains frozen in `score_gate_q.CKPT_SHA16` + `ppo_gate_e_best.zip`.

### Near-miss package (best rank) — T4_09 @ 1.35e6

| | bout0 | bout1 |
|--|-------|-------|
| apps | **0.548** | **0.630** |
| ret cf | **0.687** | **0.821** |
| skate mean/p95 | 0.071/**0.205** | 0.069/**0.217** |
| tip_ret | 27.6 | 21.5 |
| mid (PRE_GAP) | 4.82 | 2.12 |
| cam@preSS / dxc_pre | 0.386 / 0.353 | 0.278 / 0.227 |
| ret_ok | False (skate p95) | False (skate p95) |

cf≥0.55 both · apps held · tip≥8 · mid shortened vs E7lock 34.9/26.7 · plant-cam ε steal=False (flags OFF) — **skate p95** still >0.18 both. Soft-pass off.

### Closest single-bout — T4_FINAL @ 2.00e6

| | bout0 | bout1 |
|--|-------|-------|
| apps | **0.548** | **0.630** |
| ret cf | **0.883** | **0.644** |
| skate | **0.043/0.131** PASS | **0.094/0.201** FAIL |
| tip_ret | 18.9 | 11.9 |
| mid | 4.40 | 1.80 |
| ret_ok | **True** | False |

bout0 `ret_ok` under bars (first). bout1 skate mean+p95 Prefer FAIL. No `ret_ok` both.

### Other notables

- **T4_04:** cf 0.628/0.841 apps✓ — skate p95 0.259/0.313 bomb
- **T4_08:** cf 0.552/0.622 apps✓ — skate p95 0.228/0.255
- **T4_02/06:** high cf but bout1 **apps traded** (0.0) — Prefer FAIL
- Never skate≤0.08/0.18 **both** bouts while holding E7lock apps + cf≥0.55 both

## Disposition

**Prefer FAIL train wall / near-miss.** Soft-pass off. Bars KEPT. `ai_can_lock=false`. No Q03 video. New sha16 **not** installed. E7lock best-known rollback. Park — next lever needs Dave (architecture / plant), not another silent twin on frozen ckpt. Do not reopen R*/S1–S3.

Artifacts: `GATE_Q_T4_PROGRESS.md` · `GATE_Q_T4_*_PROBE.log` · `learned_gate_e/{t4_train.log,t4_train_history.json,T4_DONE.json,ppo_t4_*.zip}` · this packet · CONTROLS_NOTE / A28_RETOK_DIAG / TABLE / VIDEO_CONFLICT.

## AI disposition (~17:05 BST)

**Agrees Prefer FAIL / near-miss park.** Soft-pass never. Bars KEPT. `ai_can_lock=false`. No Q03. New sha16 correctly **not** installed — E7lock `9ffaa1a21b607bf6` stays frozen.

Honest progress (not a lock): T4_09 clear_frac **0.687/0.821** + shorter planted middle + apps held; T4_FINAL bout0 first `ret_ok` under bars. Wall = **skate p95** (and bout1 skate on FINAL). Do not reopen R*/S1–S3. Next lever needs Dave.
