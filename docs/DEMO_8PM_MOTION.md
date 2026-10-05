# 8 PM motion demo (CommandBus, frozen plant)

Clip for Dave: upright stand → walk → left turn → walk → stop on the empty walk plant. Voice later calls the same `CommandBus` (`stand` | `stop` | `vel(vx, yaw_rate)`). No kitchen script, no door, no map.

```bash
MUJOCO_GL=osmesa python scripts/demo_8pm_motion.py
```

Watch `previews/demo_8pm_motion.mp4`. Stills are `previews/demo_8pm_*.png`. Numbers are `previews/demo_8pm_motion_summary.json` (plant md5, caps, bus commands, Δx/Δyaw, min up_z, peak torque, end mode, upright).

The sequence is the claimed nav-left window already on main: 15 s forward, 12.5 s `vel(0.056, +0.25)`, 6 s forward, then `stop`. This run matches that envelope: approach Δx **+0.598 m**, turn **+75.7 deg**, resume **+0.317 m** along the new heading, min up_z **0.954**, peak leg torque **2.10 Nm**, CoP in the box, end mode **stand**, upright. End heading is **+72.0 deg**. Caps stay +0.056 / −0.032 m/s and yaw ±0.25 rad/s. Plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` stays md5 `71b2c86d133ebc603f58b99c53e496f3`. Soft-pass is off. Tip and up_z bars are unchanged.

A second file, `previews/demo_8pm_reverse.mp4`, is a short reverse (`vel(-0.032, 0)` for 5.5 s). It stayed upright: Δx **−0.376 m** in that window, min up_z **0.903** (same floor as the longer retreat on main, bar still 0.90), end mode stand. It is not chained after the turn.

Prefer FAIL, and not this clip: a 14 s left hold from t=15 s tips on the resume; a second 11 s right hold tips; a second 12.5 s left hold does not yaw and then tips; right-then-left stays upright but the left hold does not yaw left. The longer `nav-multi` clip keeps both claimed holds, and its chained right arc is about −54.9 deg, not the single-arc −77.3 deg.
