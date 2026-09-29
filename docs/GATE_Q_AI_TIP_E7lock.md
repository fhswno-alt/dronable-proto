# Gate Q AI tip — E7lock skate-only plateau (14:15 BST) Prefer FAIL

**To:** Controls · **From:** AI · Soft-pass **off** · Criteria **KEEP** (no skate amend)

## (1) KEEP skate bars — get cam from clear Δ *before* the gap

| Bar | Verdict |
|-----|---------|
| skate mean≤0.08 / p95≤0.18 | **KEEP** — planted-middle cam advance is exactly Root B; softening skate credits that slide |
| clear_frac≥0.55 | **KEEP** (formal §5 still ~70% stepped Δ) |
| tip / cam≥0.55 joint | necessary; not sufficient alone |

**How to get cam advance from clear Δ early** (so the 26.7s planted middle never forms / never needs to do work):

**Family: front-load clear Δ + clear-through-gap** — not mid-gap planted-amp/qvel/DS cuts (those are correctly abandoned).

1. **Front-load the first SS cluster (bout1 ~166–169s).** That window already has dense lifts. Raise *clear-only* hip/res/hold / CLEAR_TRACK authority **there** so `dx_clear` and cam climb while soles are clear — target **cam≳0.45–0.55 and dxc≥0.15 before t≈169**. Then end-fire early-stop can fire without needing plant travel 169→195.7.
2. **Clear-through-gap (not planted-scale).** Abandoned gap planted-amp/qvel/DS. Still open: force the **next swing phase immediately after last clear SS** (clear reburst / phase reset / CLEAR_TRACK scheduled lift) so the gap becomes more SS, not 26s DS plant. Goal: no long planted middle at all.
3. **Do not rely on post-cam≥0.55 early-stop to fix cf** — you already showed it cuts clear more than plant. End-fire early-stop KEEP only after front-load already put cam+dxc into clear.
4. **Same family for bout0** (SS ~59–62 → jump ~97): front-load clear in the first cluster; clear-through that gap; raise ret cf 0.468→≥0.55 without skate from plant middle.
5. Keep: CLEAR_TRACK_SWING=1, cancel×0.70, no damp↑/kd↑/FREEZE, apps golden.

Measure every score: `cam@last_pre_gap_SS`, `dxc_pre_gap`, `planted_middle_s`, skate. Prefer FAIL if cam still climbs mostly in plant≫clear.

## (2) Prefer FAIL / structural Root B?

**Yes — Prefer FAIL is the honest disposition until front-load (or clear-through-gap) lands.** That is **not** a soft-pass and **not** a criteria amend. It is also **not** a plant hard-falsifier / lock-FALSE yet: E7lock already proves bout1 clear_frac_ok + joint under CLEAR_TRACK; the remaining falsifier is continuous Root B (cam-from-plant middle + skate). Hold Prefer FAIL; do not reopen abandoned cancel/damp/FREEZE/gap-planted families; do not ping full Q03 until `ret_ok` both bouts + Controls continuous watch PASS.

If front-load + clear-through-gap also couple tip/cam death under cancel×0.70 after a fair probe set, ping AI for a **structural Prefer FAIL note** (still no skate soften) — we can then name the residual family wall without pretending metrics PASS.

Companion md5 lock `59cc408eda07037a58f92ad27da045d6`. Soft-pass off.
