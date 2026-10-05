# 8 PM motion demo (CommandBus, frozen plant)

Clip for Dave: upright stand → walk → left turn → walk → stop on the empty walk plant. Voice later calls the same `CommandBus` (`stand` | `stop` | `vel(vx, yaw_rate)`). No kitchen script, no door, no map.

```bash
MUJOCO_GL=osmesa python scripts/demo_8pm_motion.py
```

Watch `previews/demo_8pm_motion.mp4`. Stills are `previews/demo_8pm_*.png`. Numbers are `previews/demo_8pm_motion_summary.json` (plant md5, caps, bus commands, Δx/Δyaw, min up_z, peak torque, end mode, upright).

The sequence is the same nav-left windows: 15 s forward, 12.5 s `vel(0.056, +0.25)`, 6 s forward, then `stop`. This step-cycle basin does **not** match the previous +0.598 m / +75.7 deg envelope. Measured here: approach Δx **+0.556 m**, turn **+57.5 deg**, resume **+0.285 m** along the new heading, min up_z **0.938**, peak leg torque **2.10 Nm**, CoP in the box, end mode **stand**, upright. End heading is **+63.2 deg**. Straight-walk yaw drift on the approach is **+6.0 deg**. Caps stay +0.056 / −0.032 m/s and yaw ±0.25 rad/s. Plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` stays md5 `71b2c86d133ebc603f58b99c53e496f3`. Soft-pass is off. Tip and up_z bars are unchanged (upright floor 0.90).

The forward step is what changed. Lateral COM shift is **0.10 rad** on straight and left (was 0.16, and 0.275 before that). A right yaw in the first 12 s, and reverse, stay at **0.16 rad**: 0.10 on those steps tips the cold right turn and drops the long retreat to min up_z **0.890**. Mid-swing still adds 0.38 rad of knee flexion (forward only). Straight-walk torso roll is about **11 deg** peak-to-peak (was about **18 deg**). Sole median on that straight walk is about **2.0 cm** (was about 2.3 cm). Stance slip median is about **0.024 m/s** (was 0.016). This is not a higher step.

Not shipped, on top of the existing +0.38 rad knee lift, forward only:

- Knee **+0.03 rad** tips the left arc (min up_z −1). **+0.05 rad** stays up (min up_z 0.935, arc **+64.8 deg**, approach **+0.855 m**) but the median only moves to **2.44 cm** and the straight section drifts **−15 deg**. **+0.08 rad** tips, including the cold left turn.
- Hip flexion **+0.03 rad** stays up (arc **+67.4 deg**) with median **2.39 cm**. **+0.05 rad** pushes the arc to **+71.5 deg** and the approach to **+1.04 m**.
- Ankle dorsiflex **+0.04 rad** drops the arc to **+47 deg**. **+0.06 rad** tips the cold left turn. **+0.08 rad** keeps nav-left up (arc **+56.4 deg**, approach **+0.792 m**, min up_z 0.945) and the median only reaches **2.53 cm**, while the cold left arc falls to about **+18 deg**.
- Knee **+0.03** with hip **+0.03** holds the arc (**+58.8 deg**, approach **+0.712 m**) and the median falls to **2.37 cm**. Knee **+0.05** with ankle **+0.04** tips.
- Double-support **0.125 s** or **0.120 s** (from 0.1375 s) yaws the arc to **+88.7 deg** or **+91.6 deg**. The earlier **0.10 s** / **0.08 s** cuts were already rejected (**+125 deg** / **+104 deg**).
- Lengthening the period to **0.58 s** or **0.60 s** tips. Holding the same 0.38 rad knee lift flat through mid-swing tips (peak sole about 8 cm). A ±0.02 rad torso bob tips the left arc. Extra swing abduction **+0.03 rad** stays up (arc **+69 deg**) but the approach runs to **+1.32 m**.

A residual on top of that CPG was measured and left off. On a straight walk the swing sole median is **2.31 cm** and the foot is already off the floor for the whole swing (contact fraction **0**), so a contact-triggered swing kick never fires. Knee, hip pitch, and ankle pitch sit on the **±2.1 Nm** clip during swing, with the knee several tenths of a radian behind the command, so more swing flexion does not add torque. Holding the knee so that clip does not reverse into extension leaves the median at **2.31 cm**. Preloading **0.20 rad** of flexion while the foot is still down (that phase is not on the clip) reaches **2.45 cm**; **0.45 rad** and above lowers the sole because the hip crouches. Extending the planted stance leg by **0.10 rad** reaches **2.59 cm** and yaws **+5.6 deg** in 3 s of straight walking. **0.25 rad** reaches **2.84 cm** and yaws **+9.4 deg**. **0.40 rad** tips (min up_z −1). None of the upright residuals is a visible step, and a longer position-policy train cannot exceed the same torque clip. The flags in `scripts/steer_walk.py` stay off.

A cosine shoulder and a partial hip-timing blend were measured for a less robotic cadence and left off. Shoulder mix **0.5** keeps a single left arc upright (cold **+61.5 deg**, nav-left **+75.5 deg**) and then `nav-multi`'s chained right goes to about **−144 deg** and tips. Hip blend **0.15** does not cut the roll (still about 18 deg) and moves the cold left arc to **+48 deg**.

Arms on the straight walk are still the slewed scale **11** (shoulder peak-to-peak about **0.67 rad**).

Extra knee **+0.12 rad** still tips (min up_z −1) even when the peak sole is about 6 cm. Contralateral shoulder pitch slews to scale **11** while going straight (measured peak-to-peak about **0.67 rad**) and back to **3.4** while yaw is commanded. Arm torque stays on the existing ±0.7 Nm clip. Holding scale 6 through a cold left turn tips (min up_z 0.42). Scale 9 tips this left arc. Scale 11.4 tips nav-multi. Yaw is on the airborne foot: stance hip yaw stays 0, and full stick toes that foot 0.10 rad left or 0.12 rad right. A right command uses outside-step scale 0.10. Reverse does not take the knee lift or the arm scale. `vx=0` yaw still does not change heading.

A side close-up of stand → forward → stop is `previews/demo_step_cycle.mp4` (Δx **+0.220 m**, min up_z **0.976**, yaw drift about **+10.5 deg**). The swing foot still only clears a couple of centimeters and it still slides when it lands. The arms still travel about **0.67 rad**. From the side the torso looks upright with a small bob; the roll cut is mostly out of that camera. Realism on that clip is about **4/10**. The worst remaining issue is the short skatey step.

A second file, `previews/demo_8pm_reverse.mp4`, is a short reverse (`vel(-0.032, 0)` for 5.5 s). It stayed upright: Δx **−0.415 m** in that window, min up_z **0.905** (bar still 0.90), end mode stand. It is not chained after the turn.

Other orders (a 14 s left that starts at 15 s, a second same-sign arc, right-then-left) are not this clip and were not re-qualified on this basin. `nav-multi` on this basin keeps both claimed holds upright: left **+57.5 deg**, chained right **−78.0 deg**, min up_z **0.938**. Cold left is **+61.4 deg** (Δx **+0.284 m**, upright). Cold right stays on the 0.16 rad shift: **−85.1 deg**, upright.
