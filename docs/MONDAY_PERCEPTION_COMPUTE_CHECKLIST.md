# Monday — perception / compute checklist (draft)

**When:** drafted 28 Sep 2026 ~01:00 BST (post Gate E unlock)  
**Trigger:** Gate E moved (sim learned residual TRUE). Review with Hardware Mon 29 Sep 11:00 BST Standard lock if still scheduled.  
**Sim / kit honesty only. No spend talk in this doc.**

## Walk stack (locked facts)
- Gate D: CSF50 clean_walk_ss on M145 — TRUE
- Gate E: PPO residual on M145 @ k=1.0 — TRUE (sim); **not** Pi NN claim
- Gate F: lever look+approach on companion plant — TRUE (sim); vision off walk
- Vision: **off** walk loop (Gate F / lever only)
- CoP HX: MARGINAL — not SS/cadence prove
- Contact: 16 mm XML box vs CAD ~4.5 mm — honesty pack falsified for Gate E; plant stays M145

## Checklist for Monday
| # | Item | Owner | Status |
|---|------|-------|--------|
| 1 | Confirm FOV / look-down still bound to AiNex Hiwonder mesh (locked M145 + gate_f) | AI + Hardware | **CLOSED PASS** — `docs/MONDAY_FOV_RECONFIRM.md` (optical Z 0.380 m; `head_tilt` −16° @ 0.4 m; no spend) |
| 2 | Gate F lever approach: head_tilt −16° @ 0.4 m; vision off-board plan | AI | **LOCKED TRUE** — `GATE_F_AI_LOCK.md` (F00–F02) |
| 3 | On-Pi budget: no NN-first gait; residual MLP size vs Pi 8–10 W (informational only) | AI | **DONE (info)** — `docs/MONDAY_PI_BUDGET_NOTE.md` |
| 4 | Reproduce RL04 eval from frozen ckpt on box | Controls | **PASS EXACT** — `LEARNED_GATE_E_REPRO.md` |
| 5 | Plant↔CAD honesty one-pager acknowledged | Hardware | `PLANT_TO_CAD_ASSEMBLY_HONESTY.md` |
| 6 | STEP/fab pack still frozen 145×86 until Dave asks | Manufacturing | frozen |

## Explicit non-asks
No cart, no Orin, no sole fab refresh, no Path framing until Dave asks.
