# Gate Q AI cospec — family R3 hard-capped plant gaps (~15:02 BST)

**To:** Controls · **From:** AI · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen

## Veto?

**No veto — fair new family.**

R3 ≠ R1 (stride place) ≠ R2 (damp-hold between bursts) ≠ H front-load ≠ gap-planted-amp/qvel/DS ≠ continuous damp↑/cancel↑/FREEZE.

R3 = reverse **clear-only** bursts with **hard-capped plant gaps** (≤0.4–0.5s) between bursts using cancel×0.70 + vel-oppose only — **no intentional damp-hold**.

Baseline: E7lock (CLEAR_TRACK=1, cancel×0.70, end-fire, R1 place OFF, R2 hold OFF). md5 lock. Ckpt frozen.

## Must hold

| Rule | Detail |
|------|--------|
| Clear bursts | Swing / place / residual only while sole clear (FOOT-LIFT honesty ≥2 cm above rest) |
| Plant gaps | Hard-capped ≤0.4–0.5s; cancel×0.70 + vel-oppose only; settle≈0 XY; **no intentional damp-hold** |
| Cam steal ε | Log `cam_gain_during_plant_gap`. Prefer FAIL if **any** gap >0.02 **or** cumulative plant-gap cam >0.05/bout |
| Not R2 hold | R2 damp-hold path OFF |
| Not R1 place | R1 place OFF |
| Not abandoned | No FREEZE/pin/qvel0/continuous damp↑/kd↑/gap-planted-amp/DS |
| Bars | skate ≤0.08/0.18 · clear_frac≥0.55 — **no soften** |
| Apps | Hold ~0.548/0.630 or Prefer FAIL if traded |
| Soft-pass | **off** |

## Success / fail

- **Progress:** more SS clusters / shorter single middle; dest cam from **clear bursts**; skate→bars; tip/cam/joint held; plant-gap cam under ε.
- **Prefer FAIL** if fair set dies or plant gaps steal cam (ε above).
- Ping AI full Q03 only on `ret_ok` both + continuous watch PASS.
