# 8 PM motion demo (CommandBus, frozen plant)

Clip for Dave: upright stand → walk → left turn → walk → stop on the empty walk plant. Voice later calls the same `CommandBus` (`stand` | `stop` | `vel(vx, yaw_rate)`). No kitchen script, no door, no map.

```bash
MUJOCO_GL=osmesa python scripts/demo_8pm_motion.py
```

Watch `previews/demo_8pm_motion.mp4`. Stills are `previews/demo_8pm_*.png`. Numbers are `previews/demo_8pm_motion_summary.json` (plant md5, caps, bus commands, Δx/Δyaw, min up_z, peak torque, end mode, upright).

## Current Prefer FAIL (kit bus)

The clip is the same nav-left times on the kit gait: 15 s `vel(+0.150, 0)`, 12.5 s `vel(+0.150, +0.25)`, 6 s `vel(+0.150, 0)`, then `stop`. Plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` stays md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. Caps stay **+0.150 / −0.032** m/s and yaw **±0.25** rad/s. This pack does not raise them. Soft-pass is off. `pelvis_offset` stays **+5°**. The knee bar stays **2.33 Nm**.

| Window | Old claim (plant `71b2c86d…`) | Newly measured |
| --- | --- | --- |
| Approach 15 s | +0.598 m | heading **+2.210 m** (Δyaw +1.4 deg, mean body vx +0.139 m/s) |
| Left 12.5 s | +75.7 deg | **+145.9 deg** |
| Resume 6 s | +0.317 m | heading **+0.944 m**, drift **+12.4 deg** |
| Right 11 s (nav-right) | about −77.3 deg | **−136.2 deg**, resume heading +0.950 m |
| Chained right 11 s (nav-multi) | about −54.9 deg | **−138.6 deg** |
| Run | min up_z 0.954 | nav-left **0.934**, nav-multi stop dip **0.932**, no tip, no fault, end stand |

Controls' settled rates at `±0.25` rad/s stay **+0.217 / −0.221** rad/s (2.0% apart). Straight settled yaw is **−0.003** rad/s. Window-mean rates include slew and are not a new L/R mismatch. Forward settled body speed stays **0.150 m/s**. Reverse clamp **−0.032** realizes about **−0.031** (ratio 0.98). This is not go-anywhere, not arrival, and not a demo-ready human walk.

The sections below are the previous CPG basin (`+0.695 m / +61.1 deg`, forward clamp `+0.056`). They are not the current claim.

The sequence on that older basin was 15 s forward, 12.5 s `vel(0.056, +0.25)`, 6 s forward, then `stop`. It did not match the earlier +0.598 m / +75.7 deg envelope, and it was a small shift from the first draft of that PR (+0.742 m / +58.7 deg / min up_z 0.960). Measured there: approach Δx **+0.695 m**, turn **+61.1 deg**, resume **+0.340 m** along the new heading, min up_z **0.940**, peak leg torque **2.10 Nm**, CoP in the box, end mode **stand**, upright. End heading was **+60.7 deg**. Caps on that basin were +0.056 / −0.032 m/s and yaw ±0.25 rad/s. Plant md5 on that writeup was `71b2c86d133ebc603f58b99c53e496f3`.

The forward step is what changed. Lateral COM shift is 0.16 rad instead of 0.275, on every step. Mid-swing adds 0.38 rad of knee flexion (forward only). Sole median clearance is still about **2.4 cm** on this nav-left window (re-measured **2.38 cm**, p90 **3.2 cm**). A second pass looked for a deeper step that stays inside about ±10 deg of the shipped **+61.1 deg** arc and an approach still near **0.7 m**. Nothing in that box raised the median enough to see.

A later smoothness pass dropped the shift to **0.10 rad** on straight and left. It made the skate worse: sole median about **2.0 cm**, stance slip about **0.024 m/s** (was about **0.016**), side-close-up realism about **4/10**. That gait is fully reverted, including the cosine shoulder and hip-timing helpers from the same pass. COM shift is **0.16 rad** everywhere again. This is still a stiff short shuffle. It is not a realistic human walk, and it is not claimed to work.

Not shipped, on top of the existing +0.38 rad knee lift, forward only:

- Knee **+0.03 rad** tips the left arc (min up_z −1). **+0.05 rad** stays up (min up_z 0.935, arc **+64.8 deg**, approach **+0.855 m**) but the median only moves to **2.44 cm** and the straight section drifts **−15 deg**. **+0.08 rad** tips, including the cold left turn.
- Hip flexion **+0.03 rad** stays up (arc **+67.4 deg**) with median **2.39 cm**. **+0.05 rad** pushes the arc to **+71.5 deg** and the approach to **+1.04 m**.
- Ankle dorsiflex **+0.04 rad** drops the arc to **+47 deg**. **+0.06 rad** tips the cold left turn. **+0.08 rad** keeps nav-left up (arc **+56.4 deg**, approach **+0.792 m**, min up_z 0.945) and the median only reaches **2.53 cm**, while the cold left arc falls to about **+18 deg**.
- Knee **+0.03** with hip **+0.03** holds the arc (**+58.8 deg**, approach **+0.712 m**) and the median falls to **2.37 cm**. Knee **+0.05** with ankle **+0.04** tips.
- Double-support **0.125 s** or **0.120 s** (from 0.1375 s) yaws the arc to **+88.7 deg** or **+91.6 deg**. The earlier **0.10 s** / **0.08 s** cuts were already rejected (**+125 deg** / **+104 deg**).
- Lengthening the period to **0.58 s** or **0.60 s** tips. Holding the same 0.38 rad knee lift flat through mid-swing tips (peak sole about 8 cm). A ±0.02 rad torso bob tips the left arc. Extra swing abduction **+0.03 rad** stays up (arc **+69 deg**) but the approach runs to **+1.32 m**.

A residual on top of that CPG was measured and left off. On a straight walk the swing sole median is **2.31 cm** and the foot is already off the floor for the whole swing (contact fraction **0**), so a contact-triggered swing kick never fires. Knee, hip pitch, and ankle pitch sit on the **±2.1 Nm** clip during swing, with the knee several tenths of a radian behind the command, so more swing flexion does not add torque. Holding the knee so that clip does not reverse into extension leaves the median at **2.31 cm**. Preloading **0.20 rad** of flexion while the foot is still down (that phase is not on the clip) reaches **2.45 cm**; **0.45 rad** and above lowers the sole because the hip crouches. Extending the planted stance leg by **0.10 rad** reaches **2.59 cm** and yaws **+5.6 deg** in 3 s of straight walking. **0.25 rad** reaches **2.84 cm** and yaws **+9.4 deg**. **0.40 rad** tips (min up_z −1). None of the upright residuals is a visible step, and a longer position-policy train cannot exceed the same torque clip. The flags in `scripts/steer_walk.py` stay off. The clips on this branch are that same open-loop baseline, re-rendered after the 0.10 rad shift was removed: arms about **0.67 rad**, straight-walk sole median **2.31 cm**, left arc **+61.1 deg**.

Extra knee **+0.12 rad** still tips (min up_z −1) even when the peak sole is about 6 cm. Contralateral shoulder pitch slews to scale **11** while going straight (measured peak-to-peak about **0.67 rad**) and back to **3.4** while yaw is commanded. Arm torque stays on the existing ±0.7 Nm clip. Holding scale 6 through a cold left turn tips (min up_z 0.42). Scale 9 tips this left arc. Scale 11.4 tips nav-multi. Yaw is on the airborne foot: stance hip yaw stays 0, and full stick toes that foot 0.10 rad left or 0.12 rad right. A right command uses outside-step scale 0.10. Reverse does not take the knee lift or the arm scale. `vx=0` yaw still does not change heading.

A side close-up of stand → forward → stop is `previews/demo_step_cycle.mp4` (Δx **+0.280 m**, min up_z **0.976**). It is still a stiff short shuffle. Sole median is about **2.4 cm**. The arms still travel about **0.67 rad**. This restore does not claim a realistic walk.

A second file, `previews/demo_8pm_reverse.mp4`, is a short reverse (`vel(-0.032, 0)` for 5.5 s). It stayed upright: Δx **−0.415 m** in that window, min up_z **0.905** (bar still 0.90), end mode stand. It is not chained after the turn.

Other orders (a 14 s left that starts at 15 s, a second same-sign arc, right-then-left) are not this clip and were not re-qualified on this basin. `nav-multi` on this basin keeps both claimed holds upright: left **+61.1 deg**, chained right **−89.6 deg**, min up_z **0.940**. The chained right arc sits close to the 90 deg test edge.
