# Walk smoothness, tip ac81435

The 58ce1d8 rerun is already scored in `docs/WALK_SMOOTHNESS_RERUN.md`. Both the slow row and the kit-vx row on that tip are Prefer FAIL. This file scores tip ac81435, whole bout, with the same #102 rules. Soft-pass is off. The plant file was not edited. The gait was not edited.

Tip `ac814356f484735c442f825a2afb567ed65785d6`. Frozen plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. File hash after the runs: `207f3d5e9c6a72e16f7aa0c8d224f75e`.

The gate is the whole bout. Margin ≥ 0, outside fraction 0, unclamped ask ≤ 2.33 Nm, CoM jerk below 1048.991/132.473, joint-jerk vector below 39843/4111. Below means lower by more than the last reported digit. Preview stages (stand / start / walk / stop) are diagnostic. They do not replace the phase polygon.

| Row | Verdict | Samples | ZMP min | ZMP out | CoM min | CoM out | CoM jerk | Joint jerk | Unclamped | min up_z |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| voice-3.60 | Prefer FAIL | 1800 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | 395.309/12.015 | 11316.318/357.432 | r_hip_roll 2.3135 Nm | 0.960 |
| voice-3.70 | Prefer FAIL | 1800 | -49.05 mm | 0.008 | -45.67 mm | 0.008 | 88.853/5.353 | 2761.741/192.192 | r_hip_roll 2.2619 Nm | 0.960 |
| slow | Prefer FAIL | 1800 | -48.83 mm | 0.012 | -45.86 mm | 0.012 | 88.853/3.722 | 2114.692/134.977 | r_hip_roll 1.9956 Nm | 0.965 |

## voice-3.60: Prefer FAIL

Source: `previews/com_zmp_preview_s.json period 3.60 dsp 0.70 amp 0.043 z 0.004 arm 2.40 vx 0.056 stand 0.40 walk 11.00 stop 3.00`

Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. Samples 1800. vx 0.056 m/s. Runtime mass 2.3475 kg. Sliding friction 1.60. Soft-pass off.

| Signal | Value |
| --- | --- |
| ZMP min margin | -46.58 mm |
| ZMP outside fraction | 0.007 |
| CoM min margin | -45.15 mm |
| CoM outside fraction | 0.007 |
| CoM jerk whole bout (gate) | 395.309 / 12.015 m/s³ |
| Joint jerk vector L2 (gate) | 11316.318 / 357.432 rad/s³ |
| Worst joint scalar | r_ank_pitch 8367.011 / 222.345 rad/s³ |
| r_knee scalar | 1250.408 / 73.141 rad/s³ |
| min up_z | 0.960 |
| Unclamped leg ask | r_hip_roll 2.3135 Nm at 8.176 s, signed +0.2607 Nm, headroom +0.0165 Nm, 0 ticks over 2.33 |

| Phase | ZMP min | ZMP at | ZMP outside | CoM min | CoM at | CoM outside |
| --- | --- | --- | --- | --- | --- | --- |
| stand | +0.00 mm | 14.384 s | 0.000 (n 53) | +21.64 mm | 0.176 s | 0.000 (n 53) |
| ds | +0.00 mm | 4.040 s | 0.000 (n 1400) | +22.41 mm | 0.552 s | 0.000 (n 1400) |
| ss_L | -46.58 mm | 12.672 s | 0.082 (n 146) | -45.15 mm | 12.680 s | 0.082 (n 146) |
| ss_R | +0.00 mm | 3.888 s | 0.000 (n 201) | +21.46 mm | 3.744 s | 0.000 (n 201) |

| Preview stage | ZMP min | ZMP outside | CoM min | CoM outside | Unclamped ask |
| --- | --- | --- | --- | --- | --- |
| stand | +8.65 mm | 0.000 (n 50) | +21.64 mm | 0.000 (n 50) | l_knee 0.6993 Nm at 0.128 s, headroom +1.6307 Nm |
| start | +7.10 mm | 0.000 (n 299) | +22.41 mm | 0.000 (n 299) | l_knee 1.8291 Nm at 0.424 s, headroom +0.5009 Nm |
| walk | +0.00 mm | 0.000 (n 1076) | +21.38 mm | 0.000 (n 1076) | r_hip_roll 2.3135 Nm at 8.176 s, headroom +0.0165 Nm |
| stop | -46.58 mm | 0.032 (n 375) | -45.15 mm | 0.032 (n 375) | r_ank_pitch 2.2713 Nm at 14.392 s, headroom +0.0587 Nm |

Unclamped peak per joint:

| Joint | Unclamped peak | Time | Headroom to 2.33 | Stage |
| --- | --- | --- | --- | --- |
| r_hip_roll | 2.3135 Nm | 8.176 s | +0.0165 Nm | walk |
| l_hip_roll | 2.3127 Nm | 9.976 s | +0.0173 Nm | walk |
| r_ank_pitch | 2.2713 Nm | 14.392 s | +0.0587 Nm | stop |
| r_knee | 2.2410 Nm | 9.256 s | +0.0890 Nm | walk |
| l_knee | 2.2400 Nm | 11.056 s | +0.0900 Nm | walk |
| r_hip_pitch | 2.1263 Nm | 9.032 s | +0.2037 Nm | walk |
| l_hip_pitch | 2.1243 Nm | 10.832 s | +0.2057 Nm | walk |
| r_ank_roll | 1.7689 Nm | 4.568 s | +0.5611 Nm | walk |
| l_ank_roll | 1.7685 Nm | 6.360 s | +0.5615 Nm | walk |
| l_ank_pitch | 1.6845 Nm | 11.056 s | +0.6455 Nm | walk |
| l_hip_yaw | 0.4571 Nm | 9.288 s | +1.8729 Nm | walk |
| r_hip_yaw | 0.4563 Nm | 11.088 s | +1.8737 Nm | walk |

Gate:

- ZMP margin -0.046584 m is not ≥ 0
- ZMP outside fraction 0.007 is not 0
- CoM margin -0.045146 m is not ≥ 0
- CoM outside fraction 0.007 is not 0

Against Controls' posted numbers:

- Measured whole-bout contact CoP minimum is -46.58 mm, outside fraction 0.007. CoM minimum is -45.15 mm, outside fraction 0.007. Controls posted +21.38 mm and outside fraction 0. Their margin is the cart-table ZMP on the hull of feet with floor normal above 5 N. This margin is the contact CoP on the declared phase polygon. Swing uses the stance foot only, including while the stop is still in swing.
- The walk-stage CoM minimum is +21.38 mm. That is the +21.38 mm Controls posted as the whole-bout minimum.
- Unclamped peak matches: r_hip_roll 2.3135 Nm at 8.176 s (posted 2.3135 Nm at 8.176 s).
- CoM jerk matches: peak 395.309, RMS 12.015 (posted 395.309 / 12.015).
- r_knee jerk matches: 1250.408 / 73.141.
- Worst-joint scalar matches: r_ank_pitch 8367.011 / 222.345.
- Stop-stage contact CoP minimum is -46.58 mm, CoM minimum is -45.15 mm. Controls posted stop margin +24.82 mm on the loaded-foot hull.
- Stop ask matches: r_ank_pitch 2.2713 Nm at 14.392 s.

## voice-3.70: Prefer FAIL

Source: `previews/com_zmp_preview_p.json period 3.70 dsp 0.70 amp 0.043 z 0.004 arm 2.40 vx 0.056 stand 0.40 walk 11.00 stop 3.00`

Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. Samples 1800. vx 0.056 m/s. Runtime mass 2.3475 kg. Sliding friction 1.60. Soft-pass off.

| Signal | Value |
| --- | --- |
| ZMP min margin | -49.05 mm |
| ZMP outside fraction | 0.008 |
| CoM min margin | -45.67 mm |
| CoM outside fraction | 0.008 |
| CoM jerk whole bout (gate) | 88.853 / 5.353 m/s³ |
| Joint jerk vector L2 (gate) | 2761.741 / 192.192 rad/s³ |
| Worst joint scalar | r_sho_pitch 1844.123 / 90.686 rad/s³ |
| r_knee scalar | 1250.408 / 71.362 rad/s³ |
| min up_z | 0.960 |
| Unclamped leg ask | r_hip_roll 2.2619 Nm at 8.352 s, signed +0.2534 Nm, headroom +0.0681 Nm, 0 ticks over 2.33 |

| Phase | ZMP min | ZMP at | ZMP outside | CoM min | CoM at | CoM outside |
| --- | --- | --- | --- | --- | --- | --- |
| stand | +8.65 mm | 0.136 s | 0.000 (n 50) | +21.64 mm | 0.176 s | 0.000 (n 50) |
| ds | +0.00 mm | 4.072 s | 0.000 (n 1386) | +22.41 mm | 0.552 s | 0.000 (n 1386) |
| ss_L | -49.05 mm | 12.944 s | 0.091 (n 154) | -45.67 mm | 12.952 s | 0.091 (n 154) |
| ss_R | +0.00 mm | 3.928 s | 0.000 (n 210) | +21.50 mm | 3.768 s | 0.000 (n 210) |

| Preview stage | ZMP min | ZMP outside | CoM min | CoM outside | Unclamped ask |
| --- | --- | --- | --- | --- | --- |
| stand | +8.65 mm | 0.000 (n 50) | +21.64 mm | 0.000 (n 50) | l_knee 0.6993 Nm at 0.128 s, headroom +1.6307 Nm |
| start | +7.06 mm | 0.000 (n 299) | +22.41 mm | 0.000 (n 299) | l_knee 1.8291 Nm at 0.424 s, headroom +0.5009 Nm |
| walk | +0.00 mm | 0.000 (n 1076) | +21.42 mm | 0.000 (n 1076) | r_hip_roll 2.2619 Nm at 8.352 s, headroom +0.0681 Nm |
| stop | -49.05 mm | 0.037 (n 375) | -45.67 mm | 0.037 (n 375) | l_knee 1.5982 Nm at 11.400 s, headroom +0.7318 Nm |

Unclamped peak per joint:

| Joint | Unclamped peak | Time | Headroom to 2.33 | Stage |
| --- | --- | --- | --- | --- |
| r_hip_roll | 2.2619 Nm | 8.352 s | +0.0681 Nm | walk |
| l_hip_roll | 2.2611 Nm | 10.208 s | +0.0689 Nm | walk |
| r_knee | 2.1964 Nm | 9.480 s | +0.1336 Nm | walk |
| l_knee | 2.1942 Nm | 11.336 s | +0.1358 Nm | walk |
| l_hip_pitch | 2.0866 Nm | 11.096 s | +0.2434 Nm | walk |
| r_hip_pitch | 2.0865 Nm | 9.240 s | +0.2435 Nm | walk |
| r_ank_roll | 1.7322 Nm | 4.624 s | +0.5978 Nm | walk |
| l_ank_roll | 1.7317 Nm | 6.480 s | +0.5983 Nm | walk |
| r_ank_pitch | 1.6527 Nm | 9.480 s | +0.6773 Nm | walk |
| l_ank_pitch | 1.6507 Nm | 11.336 s | +0.6793 Nm | walk |
| l_hip_yaw | 0.4447 Nm | 9.464 s | +1.8853 Nm | walk |
| r_hip_yaw | 0.4446 Nm | 11.320 s | +1.8854 Nm | walk |

Gate:

- ZMP margin -0.049046 m is not ≥ 0
- ZMP outside fraction 0.008 is not 0
- CoM margin -0.045674 m is not ≥ 0
- CoM outside fraction 0.008 is not 0

Against Controls' posted numbers:

- Measured whole-bout contact CoP minimum is -49.05 mm, outside fraction 0.008. CoM minimum is -45.67 mm, outside fraction 0.008. Controls posted +21.42 mm and outside fraction 0. Their margin is the cart-table ZMP on the hull of feet with floor normal above 5 N. This margin is the contact CoP on the declared phase polygon. Swing uses the stance foot only, including while the stop is still in swing.
- The walk-stage CoM minimum is +21.42 mm. That is the +21.42 mm Controls posted as the whole-bout minimum.
- Unclamped peak matches: r_hip_roll 2.2619 Nm at 8.352 s (posted 2.2619 Nm at 8.352 s).
- CoM jerk matches: peak 88.853, RMS 5.353 (posted 88.853 / 5.353).
- r_knee jerk matches: 1250.408 / 71.362.
- Worst-joint scalar is r_sho_pitch 1844.123 / 90.686. Controls posted r_knee 1250.408 / 71.362, the largest leg hinge. This scorer names the largest actuated hinge, arms included.
- Stop-stage contact CoP minimum is -49.05 mm, CoM minimum is -45.67 mm. Controls posted stop margin +47.72 mm on the loaded-foot hull.
- Stop ask matches: l_knee 1.5982 Nm at 11.400 s.

## slow: Prefer FAIL

Source: `previews/com_zmp_preview_l.json period 6.40 dsp 0.70 amp 0.043 z 0.004 arm 2.40 vx 0.040 stand 0.40 walk 11.00 stop 3.00`

Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. Samples 1800. vx 0.040 m/s. Runtime mass 2.3475 kg. Sliding friction 1.60. Soft-pass off.

| Signal | Value |
| --- | --- |
| ZMP min margin | -48.83 mm |
| ZMP outside fraction | 0.012 |
| CoM min margin | -45.86 mm |
| CoM outside fraction | 0.012 |
| CoM jerk whole bout (gate) | 88.853 / 3.722 m/s³ |
| Joint jerk vector L2 (gate) | 2114.692 / 134.977 rad/s³ |
| Worst joint scalar | r_sho_pitch 1365.439 / 60.220 rad/s³ |
| r_knee scalar | 1250.408 / 55.487 rad/s³ |
| min up_z | 0.965 |
| Unclamped leg ask | r_hip_roll 1.9956 Nm at 2.816 s, signed +0.2465 Nm, headroom +0.3344 Nm, 0 ticks over 2.33 |

| Phase | ZMP min | ZMP at | ZMP outside | CoM min | CoM at | CoM outside |
| --- | --- | --- | --- | --- | --- | --- |
| stand | +8.65 mm | 0.136 s | 0.000 (n 50) | +21.64 mm | 0.176 s | 0.000 (n 50) |
| ds | +0.41 mm | 14.136 s | 0.000 (n 1368) | +22.41 mm | 0.552 s | 0.000 (n 1368) |
| ss_L | -48.83 mm | 13.800 s | 0.155 (n 142) | -45.86 mm | 13.800 s | 0.155 (n 142) |
| ss_R | +0.00 mm | 11.256 s | 0.000 (n 240) | +22.07 mm | 4.416 s | 0.000 (n 240) |

| Preview stage | ZMP min | ZMP outside | CoM min | CoM outside | Unclamped ask |
| --- | --- | --- | --- | --- | --- |
| stand | +8.65 mm | 0.000 (n 50) | +21.64 mm | 0.000 (n 50) | l_knee 0.6993 Nm at 0.128 s, headroom +1.6307 Nm |
| start | +6.50 mm | 0.000 (n 299) | +22.41 mm | 0.000 (n 299) | r_hip_roll 1.9069 Nm at 2.784 s, headroom +0.4231 Nm |
| walk | +0.00 mm | 0.000 (n 1076) | +22.02 mm | 0.000 (n 1076) | r_hip_roll 1.9956 Nm at 2.816 s, headroom +0.3344 Nm |
| stop | -48.83 mm | 0.059 (n 375) | -45.86 mm | 0.059 (n 375) | r_ank_roll 0.8200 Nm at 14.392 s, headroom +1.5100 Nm |

Unclamped peak per joint:

| Joint | Unclamped peak | Time | Headroom to 2.33 | Stage |
| --- | --- | --- | --- | --- |
| r_hip_roll | 1.9956 Nm | 2.816 s | +0.3344 Nm | walk |
| l_hip_roll | 1.9021 Nm | 2.816 s | +0.4279 Nm | walk |
| l_knee | 1.8291 Nm | 0.424 s | +0.5009 Nm | start |
| r_knee | 1.8278 Nm | 0.424 s | +0.5022 Nm | start |
| r_ank_roll | 1.6872 Nm | 2.808 s | +0.6428 Nm | walk |
| l_ank_roll | 1.4134 Nm | 2.816 s | +0.9166 Nm | walk |
| l_hip_pitch | 1.1876 Nm | 10.656 s | +1.1424 Nm | walk |
| r_hip_pitch | 1.1488 Nm | 2.816 s | +1.1812 Nm | walk |
| l_ank_pitch | 1.1384 Nm | 2.816 s | +1.1916 Nm | walk |
| r_ank_pitch | 0.8370 Nm | 9.224 s | +1.4930 Nm | walk |
| r_hip_yaw | 0.2466 Nm | 11.272 s | +2.0834 Nm | walk |
| l_hip_yaw | 0.1922 Nm | 11.000 s | +2.1378 Nm | walk |

Gate:

- ZMP margin -0.048835 m is not ≥ 0
- ZMP outside fraction 0.012 is not 0
- CoM margin -0.045858 m is not ≥ 0
- CoM outside fraction 0.012 is not 0

Against Controls' posted numbers:

- Measured whole-bout contact CoP minimum is -48.83 mm, outside fraction 0.012. CoM minimum is -45.86 mm, outside fraction 0.012. Controls posted +21.64 mm and outside fraction 0. Their margin is the cart-table ZMP on the hull of feet with floor normal above 5 N. This margin is the contact CoP on the declared phase polygon. Swing uses the stance foot only, including while the stop is still in swing.
- The stand-stage CoM minimum is +21.64 mm. That is the +21.64 mm Controls posted as the whole-bout minimum.
- Unclamped peak matches: r_hip_roll 1.9956 Nm at 2.816 s (posted 1.9956 Nm at 2.816 s).
- CoM jerk matches: peak 88.853, RMS 3.722 (posted 88.853 / 3.722).
- r_knee jerk matches: 1250.408 / 55.487.
- Worst-joint scalar is r_sho_pitch 1365.439 / 60.220. Controls posted r_knee 1250.408 / 55.487, the largest leg hinge. This scorer names the largest actuated hinge, arms included.
- Stop-stage contact CoP minimum is -48.83 mm, CoM minimum is -45.86 mm. Controls posted stop margin +26.86 mm on the loaded-foot hull.
- Stop ask matches: r_ank_roll 0.8200 Nm at 14.392 s.

## Perturbation, voice-3.60

Scorer-side only, on the loaded MjModel. The plant XML is not written. Seeds 0–9 add Gaussian noise, sigma 0.002 rad, to the twelve leg hinges after the stand pose. The nominal bout is not one of those seeds. Mass ±5% scales `body_mass` and `body_inertia` together, then `mj_setConst`. `mj_setConst` writes qpos back to qpos0, so the seated stand pose is restored before the bout. Friction sets sliding friction on the floor and both foot boxes to 1.2, 1.4, and 1.6. Latency ±1 tick shifts the command clock. The bus timeout still uses the real clock. Axes are separate, not a full factorial.

Rug was not run. The plant can name `col_mat_rug`, and `steer_walk` has entrance CommandBus scripts. Neither is a preview scene for vx 0.056 at period 3.60 s, so a rug bout was not invented.

Extra CommandBus start/stop repeats were not added. This preview row already sends one `vel` and one `stop`.

| Axis | Worst margin | Case | Worst unclamped | Case | Tip |
| --- | --- | --- | --- | --- | --- |
| seed | -46.58 mm ZMP | seed-8 | r_hip_roll 2.3136 Nm | seed-8 | no |
| mass | -50.33 mm ZMP | mass-0.95 | l_hip_roll 4.0576 Nm | mass-0.95 | no |
| friction | -46.58 mm ZMP | friction-1.6 | r_ank_pitch 8.0757 Nm | friction-1.2 | no |
| latency | -46.58 mm ZMP | latency--1 | r_ank_pitch 3.5580 Nm | latency--1 | no |

Overall worst case, nominal plus every axis:

| Scope | Worst margin | Case | Worst unclamped | Case | Tip |
| --- | --- | --- | --- | --- | --- |
| overall | -50.33 mm ZMP | mass-0.95 | r_ank_pitch 8.0757 Nm | friction-1.2 | no |

Plant file hashes in that set: `['207f3d5e9c6a72e16f7aa0c8d224f75e']`.

### seed

| Case | ZMP min | ZMP out | CoM min | CoM out | Unclamped | min up_z | Tip | Plant md5 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| seed-0 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_hip_roll 2.3134 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| seed-1 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_hip_roll 2.3135 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| seed-2 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_hip_roll 2.3133 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| seed-3 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_hip_roll 2.3135 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| seed-4 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_hip_roll 2.3136 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| seed-5 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_hip_roll 2.3136 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| seed-6 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_hip_roll 2.3133 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| seed-7 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_hip_roll 2.3134 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| seed-8 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_hip_roll 2.3136 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| seed-9 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_hip_roll 2.3134 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |

### mass

| Case | ZMP min | ZMP out | CoM min | CoM out | Unclamped | min up_z | Tip | Plant md5 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| mass-0.95 | -50.33 mm | 0.008 | -46.22 mm | 0.008 | l_hip_roll 4.0576 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| mass-1.05 | +0.00 mm | 0.000 | +21.40 mm | 0.000 | r_hip_roll 2.3238 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |

### friction

| Case | ZMP min | ZMP out | CoM min | CoM out | Unclamped | min up_z | Tip | Plant md5 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| friction-1.2 | -46.28 mm | 0.007 | -44.58 mm | 0.006 | r_ank_pitch 8.0757 Nm | 0.963 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| friction-1.4 | -45.71 mm | 0.006 | -44.70 mm | 0.006 | r_ank_pitch 8.0576 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| friction-1.6 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_hip_roll 2.3135 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |

### latency

| Case | ZMP min | ZMP out | CoM min | CoM out | Unclamped | min up_z | Tip | Plant md5 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| latency--1 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_ank_pitch 3.5580 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |
| latency-+1 | -46.58 mm | 0.007 | -45.15 mm | 0.007 | r_hip_roll 2.3135 Nm | 0.960 | no | `207f3d5e9c6a72e16f7aa0c8d224f75e` |

