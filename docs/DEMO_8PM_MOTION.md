# 8 PM motion demo (CommandBus, frozen plant)

Clip for Dave: upright stand → walk → left turn → walk → stop on the empty walk plant. Voice later calls the same `CommandBus` (`stand` | `stop` | `vel(vx, yaw_rate)`). No kitchen script, no door, no map.

```bash
MUJOCO_GL=osmesa python scripts/demo_8pm_motion.py
```

Watch `previews/demo_8pm_motion.mp4`. Stills are `previews/demo_8pm_*.png`. Numbers are `previews/demo_8pm_motion_summary.json` (plant md5, caps, bus commands, Δx/Δyaw, min up_z, peak torque, end mode, upright).

The sequence is the same nav-left windows: 15 s forward, 12.5 s `vel(0.056, +0.25)`, 6 s forward, then `stop`. This step-cycle basin does **not** match the previous +0.598 m / +75.7 deg envelope. Measured here: approach Δx **+0.742 m**, turn **+58.7 deg**, resume **+0.311 m** along the new heading, min up_z **0.960**, peak leg torque **2.10 Nm**, peak arm torque **0.62 Nm** (clip stays ±0.7), CoP in the box, end mode **stand**, upright. End heading is **+52.1 deg**. Caps stay +0.056 / −0.032 m/s and yaw ±0.25 rad/s. Plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` stays md5 `71b2c86d133ebc603f58b99c53e496f3`. Soft-pass is off. Tip and up_z bars are unchanged (upright floor 0.90).

The forward step is what changed. Lateral COM shift is 0.16 rad instead of 0.275, so the torso rocks less. Mid-swing adds 0.38 rad of knee flexion (forward only) and the sole median clearance on that arc is about 2.5 cm, with the swing foot in contact about 1% of swing instead of about a third. Contralateral shoulder pitch is scaled by 3.4 (about ±0.11 rad of travel, peak arm torque 0.62 Nm of the ±0.7 clip). A side camera still reads that as nearly frozen. Scale 6.0 moves the shoulder about ±0.19 rad and stays inside ±0.7 Nm, but a cold left turn then tips (min up_z 0.42). That scale is not shipped. Yaw is on the airborne foot: stance hip yaw stays 0, and full stick toes that foot 0.10 rad left or 0.12 rad right. A right command uses outside-step scale 0.10 so the post-straight right arc does not spin out. Reverse does not take the knee lift or the arm scale. `vx=0` yaw still does not change heading.

A side close-up of stand → forward → stop is `previews/demo_step_cycle.mp4` (Δx **+0.257 m**, min up_z **0.973**). On that clip the swing foot does leave the floor and the knee bends, but the step is still short (sole median about 2.5 cm) and the arms do not read as a swing. Torso roll is smaller than the 0.275 rad shift and is not gone.

A second file, `previews/demo_8pm_reverse.mp4`, is a short reverse (`vel(-0.032, 0)` for 5.5 s). It stayed upright: Δx **−0.415 m** in that window, min up_z **0.905** (bar still 0.90), end mode stand. It is not chained after the turn.

Other orders (a 14 s left that starts at 15 s, a second same-sign arc, right-then-left) are not this clip and were not re-qualified on this basin. `nav-multi` on this basin keeps both claimed holds upright: left **+58.7 deg**, chained right **−68.9 deg**, min up_z **0.959**.
