# Gate Q AI score packet — T5-D1 OUTER-PRIMARY+CLEAR-CORRECTOR Prefer FAIL

**Status:** **Prefer FAIL / near-miss** · Soft-pass **off** · Bars **KEPT** · `ai_can_lock=false` · No Q03  
**Outcome:** `PREFER_FAIL_SCORED`

**Cospec:** `docs/GATE_Q_AI_COSPEC_T5D.md` (AI no-veto)  
**Progress:** `previews/ainex_walk/iterate/GATE_Q_T5D1_PROGRESS.md`  
**Done:** `previews/ainex_walk/iterate/learned_gate_e/T5D1_DONE.json`

## Family and controls

T5-D1 reverse-native **outer PRIMARY** (`OUTER=CSF+ALIP`; Placo not in stack) owns support-exchange **F,T** and foothold targets from commanded −Vx / capture. Residual demoted to **clear corrector only** (mid-swing track + late-swing TDVM/SLR; authority only while sole clear ≥2 cm). Planted: cancel×0.70 + vel-oppose — **zero** planted soft-XY. Soft-pass never. Skate mean/p95, clear fraction, tip, apps, and plant-cam ε remain hard Prefer-FAIL bars.

Bootstrap **corrector warm-start only** (outer is NEW — not a T5-C retrain): T5C_s43_BC `ppo_t5c_seed43_bc.zip` sha16 `5e0c0726953840ed`. Dual-ckpt: approach=E7lock · retreat=`GATE_Q_T5D1_CKPT`. Train completed 2,002,944 steps / ~0.81 h wall (≤4 h / ≤2e6 first-hit budget); no early honest `ret_ok_both`+ε stop.

| Item | Value |
|---|---|
| Seed | 43 |
| Outer | `CSF+ALIP` (in-process; Placo N/A) |
| Corrector scale | 0.028 (residual-primary forbid) |
| Budget | 2,002,944 steps; wall ~0.81 h |
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` |
| Rollback kept | E7lock sha16 `9ffaa1a21b607bf6` |
| New install | **None** (`installed_sha16=null`) |

## Held baselines

**E7lock / T5D1_R0:** apps `0.548/0.630`; clear fraction `0.468/0.649`; skate `0.096/0.240 · 0.098/0.221`; `ret_ok=[False,False]`; plant-cam ε clean (outer idle expected — no T5D1 ckpt).

**T5C_s43_BC note (via GATE_Q_T5D1_CKPT):** apps `0.548/0.630`; clear fraction `0.821/0.792`; skate `0.043/0.124 · 0.044/0.153`; `ret_ok=[True,False]`; outer logged F≈−0.028/−0.029 n_replan 21/29; `eps_steal=True`.

## Best fair result

**Best overall:** `T5D1_13` / sha16 `45799fc897157a3a` (`ppo_t5d1_step1950000.zip` / `ppo_t5d1_best_T5D1_13_near_miss.zip`).

Values are `[bout0, bout1]`; skate is `mean/p95`.

| Metric | Value |
|---|---|
| apps | `0.548/0.630` (held) |
| clear fraction | `0.814/0.849` |
| skate mean/p95 | `0.051/0.150 · 0.042/0.144` (**under** ≤0.08/0.18 both) |
| tip_ret | `23.66/21.92` (≥8) |
| planted_middle_s | `3.50/11.44` |
| ret_ok | `[True, True]` · `ret_ok_both=true` |
| outer F / n_replan | `−0.0294/−0.0296` · `18/12` · `outer_idle=false` · OUTER logged every score |
| plant-cam ε | **Prefer FAIL** · steal=True · outer plant maxG `0.055/0.020` (ε any>0.02) · plantG `0.198/0.138` |

Honest beat blocked solely by **plant-cam ε / Root B plant-window steal** (outer plant suite maxG 0.055 bout0; PRE_GAP cam_gain_in_gap 0.057/0.249). Soft-pass never used to waive ε.

## Disposition

**Prefer FAIL / near-miss.** Outer PRIMARY engaged (F,T,footholds logged every score; not residual-idle). Skate + clear_frac + tip + apps + `ret_ok` both cleared on best ckpt — but plant-cam ε Prefer FAIL forbids honest E7lock beat / new sha16 install. E7lock sha16 `9ffaa1a21b607bf6` remains frozen. Soft-pass remains off; bars unchanged. No continuous Q03 (ε package incomplete).

**Next lever (cospec escalation):** **T5-D2** ICP step timing+location **or** Dave **T5-D4**/H2 named unlock — **not** residual twin; **not** auto H2; do not park Q without Dave. Do not reopen R*/S1–S3/T5-A/B/C twins.

## AI disposition (~22:39 BST)

**Agrees Prefer FAIL / near-miss of T5-D1.** Soft-pass never. Bars KEPT. `ai_can_lock=false`. No Q03. New sha16 correctly **not** installed — E7lock `9ffaa1a21b607bf6` kept.

Honest progress: outer PRIMARY engaged (`outer_idle=false`); skate under both bars; `ret_ok_both=true`; cf/tip/apps held. **Blocked solely by plant-cam ε / Root B** (maxG 0.055). Architecture switch worked for locomotor package; planted-window cam steal remains.

**Next:** T5-D2 ICP (prep `GATE_Q_AI_RESEARCH_T5D2_PREP.md`) **or** Dave named H2 unlock (pack ready-not-installed) — not residual twin; not auto H2; do not park without Dave.
