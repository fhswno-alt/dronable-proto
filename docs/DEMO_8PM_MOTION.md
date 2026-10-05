# 8 PM motion demo (CommandBus, frozen plant)

Clip for Dave: upright stand → walk → left turn → walk → stop on the empty walk plant. Voice later calls the same `CommandBus` (`stand` | `stop` | `vel(vx, yaw_rate)`). No kitchen script, no door, no map.

```bash
MUJOCO_GL=osmesa python scripts/demo_8pm_motion.py
```

Watch `previews/demo_8pm_motion.mp4`. Stills are `previews/demo_8pm_*.png`. Numbers are `previews/demo_8pm_motion_summary.json` (plant md5, caps, bus commands, Δx/Δyaw, min up_z, peak torque, end mode, upright).

The sequence is the same nav-left windows: 15 s forward, 12.5 s `vel(0.056, +0.25)`, 6 s forward, then `stop`. This step-cycle basin does **not** match the previous +0.598 m / +75.7 deg envelope, and it is a small shift from the first draft of this PR (+0.742 m / +58.7 deg / min up_z 0.960). Measured here: approach Δx **+0.695 m**, turn **+61.1 deg**, resume **+0.340 m** along the new heading, min up_z **0.940**, peak leg torque **2.10 Nm**, CoP in the box, end mode **stand**, upright. End heading is **+60.7 deg**. Caps stay +0.056 / −0.032 m/s and yaw ±0.25 rad/s. Plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` stays md5 `71b2c86d133ebc603f58b99c53e496f3`. Soft-pass is off. Tip and up_z bars are unchanged (upright floor 0.90).

The forward step is what changed. Lateral COM shift is 0.16 rad instead of 0.275. Mid-swing adds 0.38 rad of knee flexion (forward only). Sole median clearance is still about **2.4 cm** on this nav-left window (re-measured **2.38 cm**, p90 **3.2 cm**). A second pass looked for a deeper step that stays inside about ±10 deg of the shipped **+61.1 deg** arc and an approach still near **0.7 m**. Nothing in that box raised the median enough to see, so the clips were not re-rendered.

Not shipped, on top of the existing +0.38 rad knee lift, forward only:

- Knee **+0.03 rad** tips the left arc (min up_z −1). **+0.05 rad** stays up (min up_z 0.935, arc **+64.8 deg**, approach **+0.855 m**) but the median only moves to **2.44 cm** and the straight section drifts **−15 deg**. **+0.08 rad** tips, including the cold left turn.
- Hip flexion **+0.03 rad** stays up (arc **+67.4 deg**) with median **2.39 cm**. **+0.05 rad** pushes the arc to **+71.5 deg** and the approach to **+1.04 m**.
- Ankle dorsiflex **+0.04 rad** drops the arc to **+47 deg**. **+0.06 rad** tips the cold left turn. **+0.08 rad** keeps nav-left up (arc **+56.4 deg**, approach **+0.792 m**, min up_z 0.945) and the median only reaches **2.53 cm**, while the cold left arc falls to about **+18 deg**.
- Knee **+0.03** with hip **+0.03** holds the arc (**+58.8 deg**, approach **+0.712 m**) and the median falls to **2.37 cm**. Knee **+0.05** with ankle **+0.04** tips.
- Double-support **0.125 s** or **0.120 s** (from 0.1375 s) yaws the arc to **+88.7 deg** or **+91.6 deg**. The earlier **0.10 s** / **0.08 s** cuts were already rejected (**+125 deg** / **+104 deg**).
- Lengthening the period to **0.58 s** or **0.60 s** tips. Holding the same 0.38 rad knee lift flat through mid-swing tips (peak sole about 8 cm). A ±0.02 rad torso bob tips the left arc. Extra swing abduction **+0.03 rad** stays up (arc **+69 deg**) but the approach runs to **+1.32 m**.

Extra knee **+0.12 rad** still tips (min up_z −1) even when the peak sole is about 6 cm. Contralateral shoulder pitch slews to scale **11** while going straight (measured peak-to-peak about **0.67 rad**) and back to **3.4** while yaw is commanded. Arm torque stays on the existing ±0.7 Nm clip. Holding scale 6 through a cold left turn tips (min up_z 0.42). Scale 9 tips this left arc. Scale 11.4 tips nav-multi. Yaw is on the airborne foot: stance hip yaw stays 0, and full stick toes that foot 0.10 rad left or 0.12 rad right. A right command uses outside-step scale 0.10. Reverse does not take the knee lift or the arm scale. `vx=0` yaw still does not change heading.

A side close-up of stand → forward → stop is `previews/demo_step_cycle.mp4` (Δx **+0.280 m**, min up_z **0.976**). The swing foot leaves the floor and the knee bends, but the step is still a short shuffle (sole median about 2.5 cm). On that close-up the arms alternate with the steps. Torso roll is smaller than the 0.275 rad shift and is not gone.

A second file, `previews/demo_8pm_reverse.mp4`, is a short reverse (`vel(-0.032, 0)` for 5.5 s). It stayed upright: Δx **−0.415 m** in that window, min up_z **0.905** (bar still 0.90), end mode stand. It is not chained after the turn.

Other orders (a 14 s left that starts at 15 s, a second same-sign arc, right-then-left) are not this clip and were not re-qualified on this basin. `nav-multi` on this basin keeps both claimed holds upright: left **+61.1 deg**, chained right **−89.6 deg**, min up_z **0.940**. The chained right arc sits close to the 90 deg test edge.
