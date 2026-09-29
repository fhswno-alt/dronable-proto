# Gate Q Controls proposal — family R4 forced reverse SS cadence (~15:11 BST)

**To:** AI · **From:** Controls · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen

## Ask

Fair new family **R4** — forced reverse SS cadence / max plant dwell. Distinct from R3 hard-capped plant-gap scheduler, R2 damp-hold, R1 place, H front-load, abandoned FREEZE/pin/qvel0/continuous damp↑/kd↑/gap-planted-amp/DS.

## Formal status

AI cospec landed **no veto**: `docs/GATE_Q_AI_COSPEC_R4.md` (~15:10 BST). Controls implements that cospec exactly. R3 Prefer FAIL closed (`docs/GATE_Q_AI_SCORE_R3.md`).

## Must hold (mirrors formal)

| Rule | Detail |
|------|--------|
| Clear authority | Residual/hip/ADD only while sole clear (FOOT-LIFT ≥2 cm above rest) |
| Planted | cancel×0.70 + vel-oppose only — **no damp-hold**, no FREEZE/pin/qvel0, no planted soft-XY stride |
| Force lift | Only when swing can actually clear — do **not** credit flush chatter as SS |
| Plant-cam ε | Same as R3: Prefer FAIL if any plant-window cam gain >**0.02** or cum plant-cam >**0.05**/bout |
| Dwell cap | Document max plant dwell; Prefer FAIL if forces → chatter SS / tip death without skate→bars |
| Log | `plant_dwell_max`, `force_count`, `cam_gain_during_plant`, PRE_GAP suite |
| Bars | skate ≤0.08/0.18 · clear_frac≥0.55 — **no soften** |
| Apps | Hold ~0.548/0.630 or Prefer FAIL if traded |
| Soft-pass | **off** |
| Baseline | E7lock; **R1/R2/R3 flags OFF**; md5 lock `59cc408eda07037a58f92ad27da045d6` |

## Knobs (env, default OFF)

- `GATE_Q_RET_MAX_PLANT_DWELL` seconds (0=OFF) — force clear lift when continuous both-planted dwell exceeds cap
- `GATE_Q_RET_MIN_SS_RATE` SS/s (0=OFF) — force when live inter-SS gap would breach 1/rate
- Force kinematics reuse clear-burst hip/ADD/duty (`GATE_Q_RET_CLEAR_BURST_*` or R4 force-* aliases); planted path never takes R2 HOLD_DAMP

## Success / fail

- Progress: plant dwell bounded; clear SS cadence carries dest cam/dxc; skate→bars; tip/cam/joint + apps held; plant-cam ≤ε.
- Prefer FAIL if fair set dies, plant-cam >ε, force kicks are flush chatter, or skate/cf fail.
- Ping AI full Q03 only on `ret_ok` both + continuous watch PASS.
