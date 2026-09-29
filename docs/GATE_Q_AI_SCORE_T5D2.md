# Gate Q AI score packet — T5-D2 ICP-ΔT+ΔFOOT Prefer FAIL

**Status:** **Prefer FAIL / train wall** · Soft-pass **off** · Bars **KEPT** · `ai_can_lock=false` · No Q03  
**Outcome:** `PREFER_FAIL_SCORED`

**Cospec:** `docs/GATE_Q_AI_COSPEC_T5D2.md` (AI no-veto)  
**Progress:** `previews/ainex_walk/iterate/GATE_Q_T5D2_PROGRESS.md`  
**Done:** `previews/ainex_walk/iterate/learned_gate_e/T5D2_DONE.json`  
**Cite:** arXiv:1703.00477 (Atlas ICP step timing + location)

## Family and controls

T5-D2 reverse-native **outer** keeps CSF+ALIP as **schedule PRIOR** and adds **online ICP error → ΔT + Δfoot** feedback on retreat (plus CMP→ankle under stance honesty). Residual demoted to **clear corrector only** (mid-swing track + late-swing TDVM/SLR; authority only while sole clear ≥2 cm). Planted: cancel×0.70 + vel-oppose — **zero** planted soft-XY. Soft-pass never. Skate mean/p95, clear fraction, tip, apps, and plant-cam ε remain hard Prefer-FAIL bars. **H2 OUT** (not unlocked).

Bootstrap **corrector warm-start only** (outer ICP loop is NEW): T5D1_13 near-miss `ppo_t5d1_best_T5D1_13_near_miss.zip` sha16 `45799fc897157a3a`. Dual-ckpt: approach=E7lock · retreat=`GATE_Q_T5D2_CKPT`. Train completed 2,002,944 steps / ~0.83 h wall (≤4 h / ≤2e6 first-hit budget); no early honest `ret_ok_both`+ε stop.

| Item | Value |
|---|---|
| Seed | 43 |
| Outer | `CSF+ALIP+ICP` (schedule prior + online ICP ΔT+Δfoot) |
| Corrector scale | 0.028 (residual-primary forbid) |
| Budget | 2,002,944 steps; wall ~0.83 h |
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` (LIVE FREEZE) |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` (LIVE FREEZE) |
| Rollback kept | E7lock sha16 `9ffaa1a21b607bf6` |
| New install | **None** (`installed_sha16=null`) |
| H2 | **OUT** / not unlocked |

## Held baselines

**E7lock / T5D2_R0:** apps `0.548/0.630`; clear fraction `0.468/0.649`; skate `0.096/0.240 · 0.098/0.221`; `ret_ok=[False,False]`; ICP idle expected (no T5D2 ckpt).

**T5D2_T5D1_13 note (D1 near-miss under ICP outer):** apps `0.548/0.399`; ICP engaged (`icp_idle=false`, n_icp 9/17, ΔT/Δfoot logged); `eps_steal=True`; skate/cf/ret_ok regress vs D1 SCORE under new ICP outer (expected — outer is NEW).

**T5D2_T5C_s43_BC note:** apps `0.548/0.630`; ICP engaged; skate over; `eps_steal=True`.

## Best fair result

**Best overall:** `T5D2_13` / sha16 `71aae561854ff1a6` (`ppo_t5d2_step1950000.zip` / `ppo_t5d2_best_T5D2_13_near_miss.zip`).

Values are `[bout0, bout1]`; skate is `mean/p95`.

| Metric | Value |
|---|---|
| apps | `0.548/0.630` (held) |
| clear fraction | `0.631/0.694` (≥0.55 both) |
| skate mean/p95 | `0.079/0.237 · 0.068/0.212` (**p95 over** ≤0.18 both; mean under) |
| tip_ret | `14.74/21.58` (≥8) |
| planted_middle_s | `18.34/12.04` |
| ret_ok | `[False, False]` · `ret_ok_both=false` |
| outer F / T / n_replan | ICP-adjusted · T≈`0.716/0.731` · n_replan `16/25` · `outer_idle=false` |
| ICP suite | `icp_idle=false` · err≈`0.074/0.085` · ΔT≈`0.075/0.088` · Δfoot≈`0.028/0.027` · n_icp `16/25` · **ICP logged every score** |
| plant-cam ε | **Prefer FAIL** · steal=True · outer plant maxG **`0.177/0.033`** (ε any>0.02) · plantG `0.285/0.226` · PRE_GAP cam_gain `0.116/0.057` |

Vs T5D1_13 near-miss (sha16 `45799fc897157a3a`): D1 held skate under both + `ret_ok_both=true` with maxG 0.055 ε wall. **D2 did not hold those locomotor wins** — skate p95 over both; `ret_ok_both=false`; plant-cam maxG **worse** (0.177). Soft-pass never used to waive bars/ε.

## Disposition

**Prefer FAIL / train wall.** ICP loop **engaged** (`icp_idle=false`, ΔT/Δfoot/F/T logged every score — **not** D1-schedule twin without ICP; **not** residual-primary idle). Apps + cf + tip held on best ckpt — but skate p95 Prefer FAIL + plant-cam ε Prefer FAIL + no `ret_ok` both forbid honest E7lock beat / new sha16 install. E7lock sha16 `9ffaa1a21b607bf6` remains frozen. Soft-pass remains off; bars unchanged. No continuous Q03. H2 not auto-unlocked.

**Next lever (cospec escalation):** ask Dave **T5-D4** / **H2** named unlock **or** next genuinely different family — **not** residual twin; **not** auto H2; do not park Q without Dave. Do not reopen R*/S1–S3/T5-A/B/C/T5-D1-twin.

## Controls notes (~01:38 BST Tue 29 Sep)

- Train: `scripts/learned_gate_e/train_t5d2_icp.py` · env `AinexT5D2ICPEnv` · log `t5d2_train.log` · pid recorded
- Scorer: `GATE_Q_T5D2_CKPT` dual-ckpt + `T5D2_ICP` / `T5D2_OUTER_FT` logs
- Fair Prefer FAIL set completed under freeze; `installed_sha16=null`; soft_pass=false

## AI disposition (~01:40 BST Tue 29 Sep)

**Agrees Prefer FAIL / train wall of T5-D2.** Soft-pass never. Bars KEPT. `ai_can_lock=false`. No Q03. New sha16 correctly **not** installed — E7lock `9ffaa1a21b607bf6` kept. H2 not unlocked.

ICP loop was engaged (not D1 twin without ICP). Honest regression vs T5D1_13: skate p95 over, `ret_ok_both=false`, plant-cam maxG **worse** (0.177 vs 0.055). Do **not** reopen D1/D2 twins. Best locomotor rollback note remains T5D1_13 near-miss (ε wall only).

**Preferred next families for overnight research / morning Dave pick** (genuinely ≠ D1/D2):
1. **T5-E FOOTSTEP-SEQ** — external retreat foothold sequence (Walk-This-Way / planned clear touchdowns) as PRIMARY; residual clear corrector only. Attacks Root B by *planning* clear Δ, not ICP ΔT on planted middle.
2. **H2 Prefer FAIL A/B** — Dave named unlock on ready-not-installed pack (not auto). Plant honesty experiment only.
3. Out: residual twin · D1/D2 reshape · soft-pass · bar soften · auto H2.
