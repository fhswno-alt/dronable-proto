# Day-1 kit walk smoothness score

Harness: `SteerSession(lipm=locked_kit_config())` and `BUS_KIT_SCRIPT` in `scripts/steer_walk.py`.
Stand 0.50 s, forward until 6.00 s, stop until 7.60 s. Yaw, reach, voice, and d_min are not run.
Soft-pass is off. The plant file is not written.

**Prefer FAIL. Soft-pass off. ZMP/CoP min margin -0.095164 m, outside fraction 0.213, no-contact samples 0, single-support stance under 5 N 95. CoM projection on the same polygon min margin -0.028823 m, outside fraction 0.335. min up_z 0.934. peak leg torque 2.28 Nm (sag bar 2.33, rail ±2.45, not raised). plant md5 207f3d5e9c6a72e16f7aa0c8d224f75e unchanged. CoM outside the stance polygon is a gait miss and is not cleared here.**

| | |
| --- | --- |
| Verdict | Prefer FAIL |
| Soft-pass | false |
| Plant md5 | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| Frozen md5 | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| #101 tip | `7adccd1` (comment-only; contact polygon unchanged) |
| Samples | 950 at 8 ms |
| min up_z | 0.934 (tip bar 0.85) |
| Faults | none |
| ZMP min margin | -0.095164 m |
| ZMP outside fraction | 0.213 |
| ZMP no-contact samples | 0 |
| SS stance under 5 N | 95 |
| CoM min margin (same polygon) | -0.028823 m |
| CoM outside fraction | 0.335 |
| Harness CoM min margin | -0.009935 m |
| Peak leg torque | 2.280 Nm |
| Sag bar / forcerange | 2.33 Nm / ±2.45 Nm (not raised) |
| Worst ZMP sample | t=2.080 s phase=ss_R margin=-0.095164 m ZMP=(+0.1510, +0.0862) m |

## Jerk

Unfiltered third difference at the 8 ms control tick. A contact impact sets the peak. RMS is the bout figure. Nothing is low-passed.

- CoM (`body_link` subtree_com): peak 1048.991 m/s^3, RMS 132.473 m/s^3 (x 614.428/49.388, y 263.951/50.368, z 850.211/112.129 peak/RMS)
- Actuated joints (hinge qpos, same differences): peak 39843.130 rad/s^3, RMS 4111.013 rad/s^3; worst joint r_knee peak |jerk| 17489.560 rad/s^3

## ZMP margin by phase

Margin is metres to the stance polygon edge, on ticks that have a contact CoP. Negative is outside that polygon or outside a loaded foot's own box. Outside fraction also counts ticks with no floor contact and single-support ticks whose stance foot is under 5 N. Those ticks are not given a fake −1 m margin. No sample is dropped.

| Phase | n | min margin (m) | mean margin (m) | outside fraction | no contact | stance unloaded |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| stand | 263 | -0.000020 | +0.020277 | 0.008 | 0 | 0 |
| ds | 151 | +0.000002 | +0.000938 | 0.000 | 0 | 0 |
| ss_L | 261 | -0.094937 | -0.014584 | 0.375 | 0 | 48 |
| ss_R | 275 | -0.095164 | -0.013402 | 0.371 | 0 | 47 |

## Signals

- ZMP is the floor-contact CoP already in the plant (`LipmWalker._cop_local` normal-force sum on both foot boxes). Not a preview law.
- Single support polygon is the stance foot box (135×76 mm, half 0.0675×0.0380 m, centres read from the frozen geom). Double support and stand use the convex hull of both boxes.
- A foot loaded at or above `unload_n` (5 N) must keep its own CoP inside its box. That slack is included in the sample margin.
- CoM margin is the body subtree CoM against the same phase polygon. It is reported beside ZMP. It does not soften a ZMP miss.

## Plant check

- plant md5 207f3d5e9c6a72e16f7aa0c8d224f75e matches the cleared geometry; mass 2.347 kg; trunk 0.743 kg; foot half 0.0675×0.0380×0.008 m; centres unchanged; forcerange ±2.45; kit_cam frozen
