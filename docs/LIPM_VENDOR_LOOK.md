# LIPM vendor-look Prefer FAIL

Soft-pass is off. The plant file was not edited.
`mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` md5
`71b2c86d133ebc603f58b99c53e496f3`. Leg forcerange stays ±2.1 Nm.

The bar is the Hiwonder GaitManager look: step height about 2 cm, step
length at most 2 cm, a real push, no skate, upright, with a light arm
swing. A 4–5 cm sole is not the bar. This controller does not pass.

## What replaced the CPG schedule

`scripts/lipm_gait.py` is the schedule. The open-loop CPG in
`walk_gait_ainex.py` is unchanged and is still what `SteerSession` runs
when no `LipmConfig` is passed (LIPM00).

The outer loop shifts weight onto one foot, and it does not lift until
that foot's CoP is inside the 145×86 mm box and the swing foot is light.
The swing is a joint-space Bézier into the existing 50 Hz position
servos: knee flexion for the sole, hip pitch for the reach, ankle equal
to the hip and knee commands actually sent on that tick, plus a
measured-normal trim so the box is not on a corner. Every command is
clipped to `|ctrl-q| ≤ 0.98 τ/kp`. There is no root wrench and no foot
`xfrc`.

## Same 8.4 s forward window, `vel(+0.056, 0)`

Numbers are in `previews/lipm_ab.json`.

| Row | Sole median | Sole p90 | Stance slip | sat_rate | min up_z | Δx | Arms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| LIPM00 CPG | 0.91 cm | 1.50 cm | 1.85 cm/s | 0.65 | 0.970 | +36.8 cm | 0.67 rad |
| LIPM01 clear 2 cm | 1.23 cm | 2.33 cm | 0.07 cm/s | 0.00 | 0.995 | −2.0 cm | — |
| LIPM02 clear 4 cm | 2.01 cm | 4.76 cm | 0.21 cm/s | 0.00 | 0.976 | +0.1 cm | — |
| LIPM03 clear 2 cm + arms | 1.27 cm | 2.36 cm | 0.07 cm/s | 0.00 | 0.995 | −1.9 cm | 0.21 rad |

LIPM00 is the old gait, including its root wrench. It is the one that
moves, and it is the one that skates. LIPM03 is the vendor-height row:
the swing sole's 90th percentile is 2.4 cm, contact on that foot is
about 5% of the swing, stance slip is under 1 mm/s, the torso stays at
up_z 0.995, and the shoulder moves 0.21 rad. CoP margin at the lifts
that do happen is about 1.9 cm inside the box. Two lifts, two missed
return gates. The body does not go forward.

LIPM02 is not the bar. Commanding 4 cm does raise the swing peak (p90
4.8 cm) with sat_rate still 0, and the body still does not advance.
An unloaded, level swing can clear that height. A step that pushes
cannot.

## Why the push is the miss

The first swing does leave the floor. With the sole held level, a knee
flexion of about half a radian clears ~2 cm, and the hip then reaches
so the foot lands a couple of centimetres ahead. The 145 mm soles still
overlap. The return lean puts more load on the forward foot and never
gets the rear foot under the 5 N gate, so the same foot is the one that
swings again. Extending the rear hip to carry the pelvis onto the
forward toe pitches the torso over: a 0.08 rad push during swing fell
immediately, and a 0.05 rad hold in double support tipped near 13 s.
Both are off. The toe of this box sits 10.25 cm ahead of the ankle.
Full weight there is about 2.36 Nm. The freeze is ±2.1 Nm.

Swing sat_rate on LIPM01 and LIPM03 is 0. The knee is not sitting on
the ±2.1 clip during the 2 cm clear (knee command stays inside the
position band; peak band ratio about 0.96). The 0.35 Nm gap from ±2.1
to the HX running torque of ±2.45 Nm is not what this swing is short of.
±2.45 Nm is only about 0.09 Nm above the 2.36 Nm toe moment, so a thaw
to running torque is the smallest change that could allow a toe CoP,
and it is marginal. Stall torque 3.43 Nm is not the request. A 135×76
mesh sole is not this change, and it is not useful to ask for until
that torque thaw has been tried. This PR does not edit the plant.

## Nav-left, same 37 s script

Approach (stand 1 s, forward through 16 s): Δx = −1.6 cm, no tip,
segment min up_z 0.976. The 12.5 s left arc `vel(+0.056, +0.25)` was
not shortened. It tips at **t = 18.7 s** (`tip up_z=0.43`), about 2.7 s
into the arc. Turn-window Δyaw on that fallen clip is +13.6 deg, not a
walk. Peak leg torque on the clip reaches the 2.10 Nm clip. End mode is
fault.

## Clips

- `previews/lipm_step_closeup.mp4` — side view of the upright 2 cm swing. Not a passing walk.
- `previews/lipm_nav_left.mp4` — the full nav-left script, including the tip at 18.7 s.
