# Monday #3 — on-Pi residual budget (informational only)

**When:** Mon 28 Sep 2026 ~01:34 BST  
**Owner:** Founding AI Scientist  
**Checklist:** `docs/MONDAY_PERCEPTION_COMPUTE_CHECKLIST.md` item #3  
**Scope:** Honesty note only. **Not** a deploy plan. **Not** NN-first gait on Pi. **No Orin.** **No spend.**

## Locked sim fact
Gate E unlock used **PPO MLP [64, 64]** residual Δctrl on 12 leg channels, proprio-only obs (~41-D), frozen ckpt  
`previews/ainex_walk/iterate/learned_gate_e/ppo_gate_e_best.zip` (sha16 `9ffaa1a21b607bf6`).

Rough param count (order-of-magnitude):  
obs≈41 → 64 → 64 → act≈12 ⇒ ~**7–8k** weights — tiny vs any vision net.

## Pi 8–10 W class (kit compute)
| Claim | Status |
|-------|--------|
| MLP [64,64] residual **fits** Pi-class CPU/NPU envelope as a **control tick add-on** | **Plausible** (informational) |
| Same stack as **NN-first kit gait** or vision-in-walk on Pi | **Not claimed** — forbidden until Dave asks |
| Orin / Jetson path | **Out of scope** |
| On-device detector for lever | **Out of scope** (Gate F/G keep vision off walk) |

## Monday close
Item #3 = **DONE (informational)**. No hardware change. No deploy ticket. Revisit only if Dave asks for an on-device residual plan after assembly greenlight.
