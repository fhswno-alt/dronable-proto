# Gate Q AI tip — iterate3r19h7k8af (13:03 BST) Prefer FAIL

**To:** Controls · **From:** AI · Soft-pass **off** · Criteria **KEEP**

## What the numbers say (this pack)
| Bout | Leg | SS | clear_frac | dx_clear | dx_plant | Read |
|------|-----|----|------------|----------|----------|------|
| 0 | app | 4+2 | **0.548** | 0.326 | 0.269 | Golden — leave alone |
| 0 | ret | 5+6 | **0.234** | 0.134 | 0.440 | Lifts exist; planted Δ still wins |
| 1 | app | **0+0** | **0.000** | 0.000 | 0.037 | Handoff killed lifts |
| 1 | ret | 2+2 | 0.344 | 0.218 | 0.416 | Thin; still plant-heavy |

## Tips (controller-only; no freeze/qvel0/pin)
1. **Bout1 app first.** Latest score shows bout1 app SS=0+0 (earlier restore to ~0.45 did not hold). EXTRA_HOLD2.0 / CLEAR_HOLD0.38 / planted_damp0.32 is over-damping the handoff into no clear windows. Pull bout1 app **toward golden-app** params (hold0.32 / res×1.40 / hip×1.45 / damp0.40), not more hold. Target bout1 clear_frac ≥0.55 without inventing plant soft-XY.
2. **Retreat is Δ-per-clear, not lift count.** Ret0 already has 5+6 SS but clear_frac 0.234 because dx_plant (0.44) ≫ dx_clear (0.13). Raising cancel further → freeze-class (already abandoned). Prefer **stronger residual/hip impulse only while true clear**, longer clear dwell, measure `dx_clear` growth; do **not** credit planted contact cancel as stepped.
3. **Continuous Root B tell:** ret0 SS times cluster ~59–65s then jump to ~103–105s — ~40 s planted travel between lift clusters. Prefer FAIL until clear lifts **span** retreat Δ (not two bursts with a slide middle).
4. Soft-pass stays off. Ping AI only when approach **and** retreat both clear-carry on one continuous run (bout0+bout1).

Companion md5 lock `59cc408eda07037a58f92ad27da045d6` unchanged.
