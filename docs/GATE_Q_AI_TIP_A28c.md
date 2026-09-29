# Gate Q AI tip — A28c plateau (13:54 BST) Prefer FAIL

**To:** Controls · **From:** AI · Soft-pass **off** · Criteria **KEEP** (no amend)

## (1) Scorer bars — KEEP (do not amend)

| Bar | Verdict | Why |
|-----|---------|-----|
| `clear_frac≥0.55` | **KEEP** | Formal Gate Q §5 wants stepped retreat Δ ≥**~70%** of total retreat Δ. Scorer 0.55 is already a floor under that; lowering to 0.40 because “joint HIT” would credit planted soft-XY again → continuous **Root B**. The earlier AI `cf≥0.40 ∧ cam≥0.55` was a **progress checkpoint**, not a criteria amend. |
| skate mean≤0.08 / p95≤0.18 | **KEEP** | Inherited Gate P / Q §3 honesty. Softening skate while planted-cancel is weak just hides contact skate. Prefer FAIL. |
| tip_ok + cam≥0.55 / Δ≥0.15 | KEEP as already true | Q02 destination honesty — necessary but not sufficient without clear-carry Δ. |

**Do not soft-pass.** Prefer FAIL if continuous would still show planted slide-middle.

A28c read: bout1 ret cf~0.483 / cam~0.573 is past the progress joint but **below** scorer 0.55 and below criteria ~0.70; bout0 ret cf~0.377 still short; skate~0.098/0.221 fails. Gap is still Δ-per-clear + skate honesty, not destination.

## (2) Controller family next — clear-window length (your current path is right)

Keep probing **clear-window length / air latch / clear-only hip·res**. Do **not** reopen planted-cancel strength (cancel≥0.71 / damp↑ / kd↑ / FREEZE / qvel0 abandoned — agree).

Concrete next levers (controller-only; companion md5 lock):
1. **Longer clear dwell** (hold/air latch) so each swing deposits more `dx_clear` before plant — raise cf without cancel.
2. **Early-stop retreat** once `cam≥0.55` **and** multi-SS **and** `dx_clear` on track — cuts the planted mid-retreat gap that inflates `dx_plant` (A28c/bout1 SS jump ~169→195s ≈ 26 s planted middle = Root B tell).
3. **Clear-only hip/res** already; bias **more Δ into clear**, not more lifts (bout1 already 14+7 SS).
4. Protect bout0 ret (cf~0.377) with same clear-window family — don’t only tune bout1.
5. Skate: prefer less planted travel (early-stop / clearer swings) over harder planted force cancel.

Ping AI with full continuous Q03 only when metrics `ret_ok` both bouts + Controls continuous watch PASS. Soft-pass off.

Companion md5 lock `59cc408eda07037a58f92ad27da045d6`.
