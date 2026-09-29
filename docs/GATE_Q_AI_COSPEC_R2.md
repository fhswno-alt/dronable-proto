# Gate Q AI cospec — family R2 discrete reverse clear-bursts (~14:54 BST)

**To:** Controls · **From:** AI · Soft-pass **off** · Bars **KEPT** · Prefer FAIL reopen

## Veto?

**No veto — fair new family** with hold constraints below.

R2 ≠ R1 (stride place) ≠ H front-load ≠ gap-planted-amp/qvel/DS ≠ continuous damp↑/cancel↑/FREEZE.

R2 = **periodic reverse lunge**: short **clear-only** reverse stride burst, then brief **planted settle hold** — aim to break the ~27–35s DS plant into multiple SS windows without planted soft-XY credited as stepped Δ.

Baseline: E7lock (CLEAR_TRACK=1, cancel×0.70, end-fire, R1 place flag OFF). md5 lock. Ckpt frozen.

## Must hold (especially the “damp hold”)

| Rule | Detail |
|------|--------|
| Clear bursts | Swing / place / residual only while sole clear (FOOT-LIFT honesty ≥2 cm above rest) |
| Planted hold | **Settle ≈ zero XY travel** — damp to kill residual shove, **not** a cam-advance window. Log `cam_gain_during_hold` separately every score. Prefer FAIL if hold windows do the dest-cam work |
| Not continuous damp↑ | Brief hold between bursts only; do **not** reopen full-retreat damp↑ / kd↑ abandoned path |
| Not gap-planted-amp/qvel/DS | Hold ≠ scaled plant gait shove through the old middle |
| Bars | skate ≤0.08/0.18 · clear_frac≥0.55 — **no soften** |
| Apps | Hold ~0.548/0.630 or Prefer FAIL if traded |
| PRE_GAP | `cam@preSS`, `dxc_pre`, `planted_middle_s`, `cam_gain_in_gap`, plus hold-window cam gain |
| Soft-pass | **off** |

## Success / fail

- **Progress:** more SS clusters / shorter single middle; dxc_pre↑ / cam from **clear bursts**; skate→bars; tip/cam/joint held.
- **Prefer FAIL** if fair set dies or hold windows steal cam.
- Ping AI full Q03 only on `ret_ok` both + continuous watch PASS.
