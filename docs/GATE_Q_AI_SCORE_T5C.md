# Gate Q AI score packet — T5-C teacher-BC + multi-seed Prefer FAIL

**Status:** **Prefer FAIL / near-miss** · Soft-pass **off** · Bars **KEPT** · `ai_can_lock=false` · No Q03

**Cospec:** `docs/GATE_Q_AI_COSPEC_T5C.md`  
**Progress:** `previews/ainex_walk/iterate/GATE_Q_T5C_PROGRESS.md`  
**Done:** `previews/ainex_walk/iterate/learned_gate_e/T5C_DONE.json`

## Family and controls

T5-C reverse teacher BC (`CSF50_oracle+Placo_dx0.03+footstep_timing`) → residual RL, trained over seeds **41, 43, 47**. Clear-only residual; planted cancel ×0.70; zero planted soft-XY. Soft-pass was never used. Skate mean/p95, clear fraction, tip, apps, and plant-cam ε bars remain hard Prefer-FAIL bars.

Bootstrap was T5B_02 near-miss `ppo_t5b_best_T5B_02_near_miss.zip`, sha16 `a18be16ca662454f`. All three seeds completed at 2,002,944 steps; no early `ret_ok_both` stop.

| Item | Value |
|---|---|
| Seeds | 41, 43, 47 |
| Teacher | `CSF50_oracle+Placo_dx0.03+footstep_timing` |
| Budget | 2,002,944 steps/seed; wall ≤4h/seed |
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` |
| Rollback kept | E7lock sha16 `9ffaa1a21b607bf6` |
| New install | **None** (`installed_sha16=null`) |

## Held baselines

**E7lock / T5C_R0:** apps `0.548/0.630`; clear fraction `0.468/0.649`; skate `0.096/0.240 · 0.098/0.221`; `ret_ok=[False,False]`; plant-cam ε clean.

**T5B_02:** apps `0.548/0.630`; clear fraction `0.856/0.847`; skate `0.064/0.183 · 0.049/0.106`; `ret_ok=[False,True]`; `eps_steal=True` (maxG `0.033`).

## Per-seed best fair results

Values are `[bout0, bout1]`; skate is `mean/p95`.

| Seed | Best ckpt / sha16 | apps | clear fraction | skate mean/p95 | tip | mid | ret_ok | plant-cam ε |
|---|---|---|---|---|---|---|---|---|
| 41 | `T5C_s41_13` / `6b77f67dbe9a7319` | `0.548/0.630` | `0.542/0.580` | `0.065/0.228 · 0.033/0.093` | `13.48/49.78` | `22.08/29.52` | `[False,False]` | Prefer FAIL; steal=True |
| 43 | `T5C_s43_BC` / `5e0c0726953840ed` | `0.548/0.630` | `0.821/0.792` | `0.043/0.124 · 0.044/0.153` | `20.70/23.26` | `3.58/2.66` | `[True,False]` | Prefer FAIL; steal=True, maxG `0.020` |
| 47 | `T5C_s47_03` / `19d0807cfb0421b2` | `0.548/0.630` | `0.862/0.750` | `0.037/0.092 · 0.082/0.271` | `18.00/48.82` | `2.30/2.60` | `[True,False]` | Prefer FAIL; steal=True, maxG `0.014` |

**Best overall rank package:** seed 47 `T5C_s47_03`, sha16 `19d0807cfb0421b2`: apps `0.548/0.630`, clear fraction `0.862/0.750`, skate `0.037/0.092 · 0.082/0.271`, tip `18.00/48.82`, mid `2.30/2.60`, `ret_ok=[True,False]`, `ret_ok_both=false`, `eps_steal=true`. Its bout1 p95 and plant-cam ε prevent an honest pass.

## Disposition

**Prefer FAIL.** `ret_ok_both=false` for every seed; no seed cleared the complete skate + clear-fraction + tip + apps + ε package. The best seed is a useful near-miss (bout0 ret_ok only), but does not beat the gate honestly across both bouts. Soft-pass remains off and bars remain unchanged. E7lock sha16 `9ffaa1a21b607bf6` remains frozen; do not install any T5-C sha16.

Per cospec escalation, **T5-D needs Dave**. Do not reopen R*/S1–S3/T5-A/T5-B twins. No continuous Q03 watch because `ret_ok_both` was not reached.

## AI disposition (~21:22 BST)

**Agrees Prefer FAIL / near-miss of T5-C.** Soft-pass never. Bars KEPT. `ai_can_lock=false`. No Q03. New sha16 correctly **not** installed — E7lock `9ffaa1a21b607bf6` kept.

Honest progress across seeds: s43_BC skate under both bars with bout0 ret_ok; s47_03 bout0 ret_ok + strong cf — never `ret_ok` both + ε clean. **T5-A/B/C residual ladder exhausted.** Escalation: **T5-D needs Dave** (architecture / plant) — do not auto-park, do not reopen R*/S1–S3/T5 twins, do not soft-pass.
