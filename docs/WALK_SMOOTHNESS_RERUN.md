# Walk smoothness rerun, tip 58ce1d8

Scorer rules are the #102 contact-CoP rules. Soft-pass is off. The plant file was not edited. Torque limits were not raised. Controls' gait code was not edited. The bouts are the preview rows committed on that tip.

Tip `58ce1d861b85a2af3f4ca09f0eeead629baa40d1`. Frozen plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`.

Whole-bout jerk is the gate. Post-stand jerk is diagnostic. Trimming the stand impact out of the window is not a pass.

MFG bar: ZMP and CoM margin ≥ 0 on the whole bout, outside fraction 0, unclamped ask ≤ 2.33 Nm, whole-bout CoM jerk below 1048.991/132.473, whole-bout joint-jerk vector below 39843/4111. Below means lower by more than the last reported digit (0.001 on CoM jerk, 0.5 on joint jerk). The stand impact that prints as 1048.991 is not below the baseline.

Voice-speed row (vx 0.056): none. `scripts/voice_caller.py` still publishes `vel(+0.056, 0)` as the old bus phrase. `scripts/score_com_zmp.py` has no committed preview row at 0.056. The committed tags are `j` (vx 0.040), `g` (vx 0.150), and `kit102` (locked kit, not a preview row).

| Row | Verdict | Samples | ZMP min | ZMP out | CoM min | CoM out | CoM jerk whole | CoM jerk post-stand | Joint jerk | Unclamped | min up_z |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| slow | Prefer FAIL | 1800 | -48.83 mm | 0.012 | -45.86 mm | 0.012 | 1048.991/32.662 | 538.605/16.659 | 8725.806/402.455 | r_hip_roll 1.979 Nm | 0.934 |
| kit-vx | Prefer FAIL | 1013 | -49.51 mm | 0.027 | -45.68 mm | 0.011 | 1048.991/48.195 | 520.303/30.817 | 8725.806/816.661 | l_hip_pitch 4.446 Nm | 0.934 |

## slow: Prefer FAIL

Source: `scripts/score_com_zmp.py --period 6.40 --dsp 0.70 --amp 0.043 --z 0.004 --arm 2.40 --stand 0.40 --walk 11.00 --stop 3.00 --vx 0.040 --tag j`

Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. Samples 1800. vx 0.040 m/s. Soft-pass off.

| Signal | Value |
| --- | --- |
| ZMP min margin | -48.83 mm |
| ZMP outside fraction | 0.012 |
| CoM min margin | -45.86 mm |
| CoM outside fraction | 0.012 |
| CoM jerk whole bout (gate) | 1048.991 / 32.662 m/s³ |
| CoM jerk post-stand (diagnostic) | 538.605 / 16.659 m/s³ |
| Joint jerk vector L2 (gate) | 8725.806 / 402.455 rad/s³ |
| Worst joint scalar | r_ank_pitch 5857.085 / 239.084 rad/s³ |
| r_knee scalar | 2477.571 / 127.999 rad/s³ |
| min up_z | 0.934 |
| Unclamped leg ask | r_hip_roll 1.9792 Nm at 2.816 s, signed +0.2486 Nm, 0 ticks / 0 writes over 2.33 |

| Phase | ZMP min | ZMP outside | CoM min | CoM outside |
| --- | --- | --- | --- | --- |
| stand | +0.01 mm | 0.000 (n 50) | +19.91 mm | 0.000 (n 50) |
| ds | +0.01 mm | 0.000 (n 1368) | +48.90 mm | 0.000 (n 1368) |
| ss_L | -48.83 mm | 0.155 (n 142) | -45.86 mm | 0.155 (n 142) |
| ss_R | +0.00 mm | 0.000 (n 240) | +22.07 mm | 0.000 (n 240) |

Gate:

- ZMP margin -0.048835 m is not ≥ 0
- ZMP outside fraction 0.012 is not 0
- CoM margin -0.045858 m is not ≥ 0
- CoM outside fraction 0.012 is not 0
- whole-bout CoM jerk peak 1048.9908045256923 is not below 1048.991 (same stand impact; post-stand is not the gate)

Against Controls' posted numbers:

- Controls' CoM minimum +19.91 mm matches this scorer's stand bucket +19.91 mm at 0.008 s (post-step clock; their label is one tick earlier). It is not this scorer's whole-bout minimum. Whole-bout CoM min is -45.86 mm. This polygon is the declared stance foot during swing, including the stop while that phase is still swing. Their posted minimum is the hull of feet with floor normal above 5 N, which stayed positive.
- Controls' ZMP minimum +19.96 mm is the cart-table ZMP on the loaded-foot hull. This scorer's contact CoP on the stand polygon is +0.01 mm. Whole-bout contact-CoP min is -48.83 mm.
- Unclamped ask matches: r_hip_roll 1.9792 Nm at 2.816 s, |kp*(q_des−q)|+|kv*ω| before the clip. Writes over 2.33: 0. Control ticks over 2.33: 0.
- CoM jerk matches: whole-bout peak 1048.991 (the stand impact, not below 1048.991), RMS 32.662, post-stand peak 538.605. Post-stand is diagnostic only.
- Scalar joint jerk matches Controls: r_ank_pitch 5857.085 / 239.084. The gate uses the vector L2 8725.806 / 402.455, which is below 39843/4111.
- r_knee jerk matches: 2477.571 / 127.999.

## kit-vx: Prefer FAIL

Source: `scripts/score_com_zmp.py --period 2.80 --dsp 0.55 --amp 0.043 --z 0.008 --arm 1.20 --stand 0.40 --walk 5.50 --stop 2.20 --vx 0.150 --tag g`

Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. Samples 1013. vx 0.150 m/s. Soft-pass off.

| Signal | Value |
| --- | --- |
| ZMP min margin | -49.51 mm |
| ZMP outside fraction | 0.027 |
| CoM min margin | -45.68 mm |
| CoM outside fraction | 0.011 |
| CoM jerk whole bout (gate) | 1048.991 / 48.195 m/s³ |
| CoM jerk post-stand (diagnostic) | 520.303 / 30.817 m/s³ |
| Joint jerk vector L2 (gate) | 8725.806 / 816.661 rad/s³ |
| Worst joint scalar | r_ank_pitch 5857.085 / 399.790 rad/s³ |
| r_knee scalar | 2653.914 / 311.217 rad/s³ |
| min up_z | 0.934 |
| Unclamped leg ask | l_hip_pitch 4.4462 Nm at 4.424 s, signed +2.0615 Nm, 352 ticks / 689 writes over 2.33 |

| Phase | ZMP min | ZMP outside | CoM min | CoM outside |
| --- | --- | --- | --- | --- |
| stand | +0.01 mm | 0.000 (n 50) | +19.91 mm | 0.000 (n 50) |
| ds | +0.00 mm | 0.000 (n 640) | +39.67 mm | 0.000 (n 640) |
| ss_L | -16.77 mm | 0.032 (n 156) | +18.37 mm | 0.000 (n 156) |
| ss_R | -49.51 mm | 0.132 (n 167) | -45.68 mm | 0.066 (n 167) |

Gate:

- ZMP margin -0.049508 m is not ≥ 0
- ZMP outside fraction 0.027 is not 0
- CoM margin -0.045678 m is not ≥ 0
- CoM outside fraction 0.011 is not 0
- unclamped ask 4.446 Nm, 352 ticks over 2.33
- whole-bout CoM jerk peak 1048.990804526041 is not below 1048.991 (same stand impact; post-stand is not the gate)

Against Controls' posted numbers:

- Controls' CoM minimum +18.23 mm is not this scorer's whole-bout minimum -45.68 mm. Stand is +19.91 mm, ss_L is +18.37 mm, ss_R is -45.68 mm at 7.968 s. Their +18.23 mm is the loaded-foot hull on right single support at 5.144 s. This whole-bout miss is the declared stance box while the walker is still in swing during the stop. A loaded swing foot is not added to that box.
- Controls' ZMP minimum +18.23 mm is the cart-table ZMP on the loaded-foot hull. This scorer's contact CoP on the stand polygon is +0.01 mm. Whole-bout contact-CoP min is -49.51 mm.
- Unclamped ask matches: l_hip_pitch 4.4462 Nm at 4.424 s, |kp*(q_des−q)|+|kv*ω| before the clip. Writes over 2.33: 689. Control ticks over 2.33: 352.
- CoM jerk matches: whole-bout peak 1048.991 (the stand impact, not below 1048.991), RMS 48.195, post-stand peak 520.303. Post-stand is diagnostic only.
- Scalar joint jerk matches Controls: r_ank_pitch 5857.085 / 399.790. The gate uses the vector L2 8725.806 / 816.661, which is below 39843/4111.
- r_knee jerk matches: 2653.914 / 311.217.

