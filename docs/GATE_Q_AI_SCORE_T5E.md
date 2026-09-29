# Gate Q AI score packet — T5-E FOOTSTEP-SEQ Prefer FAIL

**Status:** **Prefer FAIL / near-miss** · Soft-pass **off** · Bars **KEPT** · `ai_can_lock=false` · No Q03  
**Outcome:** `PREFER_FAIL_SCORED`

**Cospec:** `docs/GATE_Q_AI_COSPEC_T5E.md` (AI no-veto · Dave greenlit ~20:29 BST)  
**Progress:** `previews/ainex_walk/iterate/GATE_Q_T5E_PROGRESS.md`  
**Done:** `previews/ainex_walk/iterate/learned_gate_e/T5E_DONE.json`  
**Cite:** Walk This Way doi:10.1145/3747865 · arXiv:2203.07589 · Placo OpenDuck short `|dx_back|`

## Family and controls

T5-E reverse-native **FOOTSTEP-SEQ outer PRIMARY** emits an ordered clear touchdown list (Placo-style short `|dx_back|=0.028`, N≈7 toward dest-cam budget) and tracks next-TD every score (`list_id` / `next_F` / `next_err` / `n_track` / `n_advance` / `seq_idle`). Residual demoted to **clear corrector only** (mid-swing track + late-swing TDVM/SLR; authority only while sole clear ≥2 cm). Planted: cancel×**0.70** + vel-oppose — **zero** planted soft-XY. Soft-pass never. Skate mean/p95, clear fraction, tip, apps, and plant-cam ε remain hard Prefer-FAIL bars. **H2 OUT**.

**≠ T5-D1:** not CSF+ALIP schedule alone — TD **list** is first-class logged primary.  
**≠ T5-D2:** no online ICP→ΔT+Δfoot loop.

Bootstrap **corrector warm-start only** (sequence outer is NEW): T5D1_13 near-miss `ppo_t5d1_best_T5D1_13_near_miss.zip` sha16 `45799fc897157a3a`. Dual-ckpt: approach=E7lock · retreat=`GATE_Q_T5E_CKPT`. Train completed 2,002,944 steps / ~0.68 h wall (≤4 h / ≤2e6 first-hit budget); no early honest `ret_ok_both`+ε stop.

| Item | Value |
|---|---|
| Seed | 43 |
| Outer | `FOOTSTEP-SEQ` (Placo-style short `|dx_back|` TD list) |
| Corrector scale | 0.028 (residual-primary forbid) |
| Budget | 2,002,944 steps; wall ~0.68 h (~2460 s) |
| Companion md5 | `59cc408eda07037a58f92ad27da045d6` (LIVE FREEZE) |
| Walk plant md5 | `fc94709c84f5598d4474ecfc4bb41fdc` (LIVE FREEZE) |
| Rollback kept | E7lock sha16 `9ffaa1a21b607bf6` |
| New install | **None** (`installed_sha16=null`) |
| H2 | **OUT** / not unlocked |
| Train | `scripts/learned_gate_e/train_t5e_footstep.py` · pid `347076` · log `t5e_train.log` |

## Held baselines

**E7lock / T5E_R0:** apps `0.548/0.630`; clear fraction `0.468/0.649`; skate `0.096/0.240 · 0.098/0.221`; `ret_ok=[False,False]`; seq idle expected (no T5E ckpt).

**T5E_T5D1_13 note (D1 near-miss under SEQ outer):** apps `0.548/0.630`; seq engaged (`seq_idle=false`, n_track 26/21); skate bout0 p95 over; `ret_ok=[False,True]`; `eps_steal=True` maxG 0.156.

**T5E_T5D2_13 note:** apps `0.548/0.723`; seq engaged; skate over; cf bout0 under; `ret_ok=[False,False]`; maxG 0.109.

## Best fair result

**Best overall:** `T5E_07` / sha16 `6023518b83da5b18` (`ppo_t5e_step1050000.zip` / `ppo_t5e_best_T5E_07_near_miss.zip`).

Values are `[bout0, bout1]`; skate is `mean/p95`.

| Metric | Value |
|---|---|
| apps | `0.548/0.630` (held) |
| clear fraction | `0.810/0.836` (≥0.55 both) |
| skate mean/p95 | `0.057/0.157 · 0.047/0.151` (**under** ≤0.08/0.18 both) |
| tip_ret | `27.06/17.04` (≥8) |
| planted_middle_s | `4.44/2.18` |
| ret_ok | `[True, False]` · `ret_ok_both=false` (bout1 cadence/stepped FAIL — skate/cf/tip held) |
| SEQ suite | `seq_idle=false` · list_id `n7_dxb0.028_lat0.000_tgt0.180` · next_F `−0.028/−0.028` · err≈`0.092/0.095` · n_track `26/17` · n_advance `24/16` · **SEQ logged every score** |
| plant-cam ε | **Prefer FAIL** · steal=True · outer plant maxG **`0.031/0.022`** (ε any>0.02) · plantG `0.284/0.153` · PRE_GAP cam_gain `0.052/0.086` |

**Vs T5D1_13** (sha16 `45799fc897157a3a`, maxG **0.055**, `ret_ok_both=true`): T5-E **beat plant maxG** (0.031 < 0.055) and **held skate under both** + apps/cf/tip — but **did not hold `ret_ok_both`** (bout1 cadence). Soft-pass never used to waive ε or bars.  
**Vs T5D2_13** (maxG 0.177, skate p95 FAIL): T5-E **did not trade skate for ε cosmetics** (D2 lesson held on best).

Also-ran note: `T5E_09` maxG **0.027** (further ε cut) but skate bout1 p95 over + no `ret_ok` both — Prefer FAIL (would be D2-style trade). `T5E_10` skate under + maxG 0.032, still `ret_ok=[True,False]`.

## Disposition

**Prefer FAIL / near-miss.** FOOTSTEP-SEQ **engaged** (`seq_idle=false`, foothold list / next TD / n_track logged every score — **not** D1-schedule twin without TD-list; **not** D2-ICP twin; **not** residual-primary idle). Apps + cf + tip + skate under both held on best ckpt; plant-cam maxG **improved vs D1** (0.031 vs 0.055) — but ε Prefer FAIL (steal any>0.02) + no `ret_ok` both forbid honest E7lock beat / new sha16 install. E7lock sha16 `9ffaa1a21b607bf6` remains frozen. Soft-pass remains off; bars unchanged. No continuous Q03. H2 not auto-unlocked.

**Next lever (cospec escalation):** ask Dave **H2** named Prefer FAIL A/B **or** alt **DCM-VRP-DS** **or** park — **not** residual twin; **not** auto H2; do not park Q without Dave. Do not reopen R*/S1–S3/T5-A/B/C/T5-D1/D2 twins.

## Controls notes (~21:16 BST Tue 29 Sep)

- Train: `scripts/learned_gate_e/train_t5e_footstep.py` · env `AinexT5EFootstepSeqEnv` · log `t5e_train.log` · pid `347076`
- Scorer: `GATE_Q_T5E_CKPT` dual-ckpt + `T5E_SEQ` / `T5E_OUTER_FT` logs (list / next TD / `seq_idle`)
- Fair Prefer FAIL set completed under freeze; `installed_sha16=null`; soft_pass=false; E7lock kept
- Companion / walk plant md5 LIVE FREEZE untouched; no mid-wall plant swap; H2 OUT
