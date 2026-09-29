# Gate Q AI cospec — family R4 forced reverse SS cadence (~15:10 BST)

**To:** Controls · **From:** AI · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen

## Veto?

**No veto — fair new family** with must-holds below.

R4 ≠ R3 burst→capped-gap scheduler · ≠ R2 damp-hold · ≠ R1 place · ≠ continuous damp↑/kd↑/FREEZE/qvel0 · ≠ gap-planted-amp/qvel/DS.

R4 = **forced reverse SS cadence / max plant dwell**: clear residual/hip **only while sole clear**; planted uses cancel×0.70 + vel-oppose only; if **plant dwell > cap** OR **plant-cam ε trips mid-dwell**, force next clear lift (phi/ADD kick). Reactive interrupt of plant — not a scheduled post-burst gap machine.

Baseline: E7lock. **R1/R2/R3 flags OFF.** md5 lock. Ckpt frozen.

## Must hold

| Rule | Detail |
|------|--------|
| Clear authority | Residual/hip/ADD only while sole clear (FOOT-LIFT ≥2 cm above rest) |
| Planted | cancel×0.70 + vel-oppose only — no damp-hold, no FREEZE/pin/qvel0, no planted soft-XY stride |
| Force lift | Only schedule a clear lift when the swing sole can actually clear — do **not** credit a forced kick that stays flush as SS |
| Plant-cam ε | Same as R3: Prefer FAIL if any plant-window cam gain >**0.02** or cum plant-cam >**0.05**/bout (log every score) |
| Dwell cap | Document chosen max plant dwell; Prefer FAIL if forces collapse into chatter SS / tip death without skate→bars |
| Log | `plant_dwell_max`, `force_count`, `cam_gain_during_plant`, PRE_GAP suite |
| Bars | skate ≤0.08/0.18 · clear_frac≥0.55 — **no soften** |
| Apps | Hold ~0.548/0.630 or Prefer FAIL if traded |
| Soft-pass | **off** |

## Success / fail

- **Progress:** plant dwell bounded; clear SS cadence carries dest cam/dxc; skate→bars; tip/cam/joint + apps held; plant-cam ≤ε.
- **Prefer FAIL** if fair set dies, plant-cam >ε, force kicks are flush chatter, or skate/cf fail.
- Ping AI full Q03 only on `ret_ok` both + continuous watch PASS.
