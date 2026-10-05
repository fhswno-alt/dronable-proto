# 8 PM motion demo (CommandBus, frozen plant)

Clip for Dave: upright stand → walk → left turn → walk → stop on the empty walk plant. Voice later calls the same `CommandBus` (`stand` | `stop` | `vel(vx, yaw_rate)`). No kitchen script, no door, no map.

```bash
MUJOCO_GL=osmesa python scripts/demo_8pm_motion.py
```

Watch `previews/demo_8pm_motion.mp4`. Stills are `previews/demo_8pm_*.png`. Numbers are `previews/demo_8pm_motion_summary.json` (plant md5, caps, bus commands, Δx/Δyaw, min up_z, peak torque, end mode, upright).

The sequence is the same nav-left windows: 15 s forward, 12.5 s `vel(0.056, +0.25)`, 6 s forward, then `stop`. This step-cycle basin does **not** match the previous +0.598 m / +75.7 deg envelope, and it is a small shift from the first draft of this PR (+0.742 m / +58.7 deg / min up_z 0.960). Measured here: approach Δx **+0.695 m**, turn **+61.1 deg**, resume **+0.340 m** along the new heading, min up_z **0.940**, peak leg torque **2.10 Nm**, CoP in the box, end mode **stand**, upright. End heading is **+60.7 deg**. Caps stay +0.056 / −0.032 m/s and yaw ±0.25 rad/s. Plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` stays md5 `71b2c86d133ebc603f58b99c53e496f3`. Soft-pass is off. Tip and up_z bars are unchanged (upright floor 0.90).

The forward step is what changed. Lateral COM shift is 0.16 rad instead of 0.275. Mid-swing adds 0.38 rad of knee flexion (forward only). Sole median clearance is still about 2.5 cm. Extra knee (+0.12 rad) and a shorter double-support (0.08–0.10 s, forward only) were tried: the extra knee tips, and the shorter double-support yaws the left arc past +100 deg. Neither is shipped. Contralateral shoulder pitch slews to scale **11** while going straight (measured peak-to-peak about **0.67 rad**) and back to **3.4** while yaw is commanded. Arm torque stays on the existing ±0.7 Nm clip. Holding scale 6 through a cold left turn tips (min up_z 0.42). Scale 9 tips this left arc. Scale 11.4 tips nav-multi. Yaw is on the airborne foot: stance hip yaw stays 0, and full stick toes that foot 0.10 rad left or 0.12 rad right. A right command uses outside-step scale 0.10. Reverse does not take the knee lift or the arm scale. `vx=0` yaw still does not change heading.

A side close-up of stand → forward → stop is `previews/demo_step_cycle.mp4` (Δx **+0.280 m**, min up_z **0.976**). The swing foot leaves the floor and the knee bends, but the step is still a short shuffle (sole median about 2.5 cm). On that close-up the arms alternate with the steps. Torso roll is smaller than the 0.275 rad shift and is not gone.

A second file, `previews/demo_8pm_reverse.mp4`, is a short reverse (`vel(-0.032, 0)` for 5.5 s). It stayed upright: Δx **−0.415 m** in that window, min up_z **0.905** (bar still 0.90), end mode stand. It is not chained after the turn.

Other orders (a 14 s left that starts at 15 s, a second same-sign arc, right-then-left) are not this clip and were not re-qualified on this basin. `nav-multi` on this basin keeps both claimed holds upright: left **+61.1 deg**, chained right **−89.6 deg**, min up_z **0.940**. The chained right arc sits close to the 90 deg test edge.
