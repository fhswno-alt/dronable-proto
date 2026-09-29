# Gate Q Controls proposal — family R3 hard-capped plant gaps (~15:05 BST)

**To:** AI · Dave · room · **From:** Controls  
**Soft-pass:** **off** · Bars **KEPT** (skate ≤0.08/0.18 · clear_frac≥0.55) · Prefer FAIL default  
**Companion md5 lock:** `59cc408eda07037a58f92ad27da045d6` · Ckpt sha16 `9ffaa1a21b607bf6`  
**Baseline:** E7lock (A28c + CLEAR_TRACK_SWING=1, cancel×0.70, end-fire)  
**AI formal cospec:** `docs/GATE_Q_AI_COSPEC_R3.md` — **no veto** (landed)

## Family

**R3: reverse clear-bursts with hard-capped plant gaps (NO damp-hold)**

| vs | Distinct because |
|----|------------------|
| R1 place | No swing-foot world −X place target |
| R2 damp-hold | Planted damp-hold phase **OFF**; no `HOLD_DAMP` |
| H front-load | Not cam/time-gated hip·res·hold bump of first cluster |
| gap-planted amp/qvel/DS | No planted gait shove / qvel scale / DS cut |
| continuous damp↑ | No full-retreat damp↑ / kd↑ |
| FREEZE | No pin / qvel0 / retain / duty |

## Mechanism

1. **Clear-only reverse lunge bursts** while sole clear (FOOT-LIFT honesty ≥2 cm above rest).
   - Reuses R2 clear-burst kinematics: phi kick into swing half + clear hip/ADD mult (`GATE_Q_RET_CLEAR_BURST_*` duty/hip/add/ss_min/cam_max).
2. **Between bursts: plant gap hard-capped** at `GATE_Q_RET_BURST_PLANT_CAP` (default try ≤0.40–0.50 s).
   - cancel×0.70 + existing vel-oppose only; settle≈0 XY intent (amp kept in cancel band).
   - **No intentional damp-hold** that can advance dest cam (`_r2_in_hold` / HOLD_DAMP never armed).
3. After plant_cap ends → **force next burst** (no idle wait that re-opens long DS middle).

## Flag (default OFF)

| Env | Meaning |
|-----|---------|
| `GATE_Q_RET_BURST_PLANT_CAP` | seconds; **0=OFF**. When >0 enables R3 |
| `GATE_Q_RET_CLEAR_BURST` | stay **0** (R2 hold path OFF) |
| `GATE_Q_RET_SWING_PLACE_M` | stay **0** (R1 OFF) |
| `GATE_Q_RET_CLEAR_BURST_DUTY_S` etc. | shared burst knobs (duty/hip/add/ss_min/cam_max) |
| `GATE_Q_RET_BURST_PLANT_GAP_CAM_EPS` | default `0.02` — Prefer FAIL if **any** gap cam > this |
| `GATE_Q_RET_BURST_PLANT_GAP_CAM_CUM` | default `0.05` — Prefer FAIL if **cumulative**/bout > this |

## Logging (every score)

- `cam_gain_during_plant_gap` (cumulative/bout)
- `r3_plant_gap_max_gain`, `r3_plant_gap_s`, `r3_plant_gap_n`, `r3_burst_n`
- `r3_plant_gap_cam_steal` (ε any>0.02 OR cum>0.05)
- Existing PRE_GAP fields kept

## Success / fail

- **Progress:** more SS clusters / shorter single middle; dest cam from **clear bursts**; skate→bars; tip/cam/joint held; plant-gap cam under ε.
- **Prefer FAIL** if fair set dies, plant gaps steal cam (ε), or apps/bout1 package traded.
- Soft-pass **never**. Ping full Q03 only on `ret_ok` both + continuous watch PASS.
- On Prefer FAIL: `ai_can_lock=false`, no video, flag default OFF.

## Fair probe plan (skip-video)

~8–12 off E7lock: plant_cap ∈ {0.40,0.45,0.50} × duty/hip/add/ss_min mild variants. R1/R2 flags OFF.
