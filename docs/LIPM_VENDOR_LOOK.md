# LIPM vendor-look Prefer FAIL

Soft-pass is off. This does not walk. Do not read the clips as a pass.

The plant is the Hardware thaw in PR #43
(`cursor/plant-thaw-legs-foot-6f10`, tip `b711472`), and nothing past that
file. `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` md5
`17dc4ff37491c8e61900fd83b5d31f0c` (was `71b2c86d…`). Legs are ±2.45 Nm
running torque. Arms and head stay ±0.7 Nm. Foot contact boxes are
135×76 mm (half-size 0.0675 × 0.038 × 0.008 m), position and friction
unchanged, toe spheres still `contype=0`. kp is unchanged.

The bar is the Hiwonder GaitManager look: step height about 2 cm, step
length at most 2 cm, step period 300–600 ms, a real push, forward
progress, no skate, upright (`min up_z` ≥ 0.92), arms not frozen. A 5 cm
sole is not the bar. An in-place march is not a walk.

## What the return gate was missing

Two 135 mm soles a couple of centimetres apart overlap. After the rear
knee shortens and the sole is about 1 cm up, the rear box hits the
stance box. That foot-foot force is about 9 N. The gate was treating it
as floor load, so the swing never started and the same foot stepped
again. Floor normal only is what the gate reads now. The rear knee
yield (0.42 rad once the lean is on the forward foot, ankle the flat-foot
sum, swing roll leveled) is what gets the sole off the floor. There is
no root wrench and no added toe torque.

The toe of this box is 9.75 cm ahead of the ankle. Full weight there is
about 2.24 Nm. ±2.45 Nm covers that on paper, with about 0.20 Nm to
spare. Commanding the hip far enough to hold a lead out in front of the
body either threw the foot to ~7 cm and tipped, or left the pelvis
behind the feet. The shipped swing is the joint Bézier that clears
about 2 cm. It does not chase the toe.

## Same 8.4 s forward window, `vel(+0.056, 0)`

Numbers are in `previews/lipm_ab.json`.

| Row | Sole median | Sole p90 | Stance slip | sat_rate | min up_z | Δx | Rear unload | Arms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| LIPM00 CPG | 0.98 cm | 1.92 cm | 1.56 cm/s | 0.51 | 0.976 | +67.8 cm | — | 0.67 rad |
| LIPM01 clear 2 cm | 1.02 cm | 2.00 cm | 0.23 cm/s | 0.00 | 0.994 | −1.2 cm | 0.98 | — |
| LIPM02 clear 4 cm | 1.58 cm | 3.86 cm | 0.22 cm/s | 0.00 | 0.995 | −1.3 cm | 0.98 | — |
| LIPM03 clear 2 cm + arms | 0.98 cm | 2.02 cm | 0.24 cm/s | 0.00 | 0.993 | −1.1 cm | 0.97 | 0.21 rad |

LIPM00 is the old gait, including its root wrench. It is the one that
moves, and it is the one that skates. LIPM03 is the vendor-height row.
Four lifts, zero missed gates, sides alternate. Rear-load share gets
down to 0.03 (unload fraction 0.97). At the lifts that open, the rear
foot is still about 12% of the contact force and the stance CoP is
2.2 cm ahead of the ankle in the foot frame (box center is 3.0 cm, so
this is heel-side of center, not the toe). CoP margin inside the box
is about 0.9 cm. Swing contact is about 4%. Stance slip is 2.4 mm/s.
sat_rate on the swing is 0. The shoulder moves 0.21 rad.

The body does not go forward. Δx is −1.1 cm. Four steps in 8 s is a
step period of about 2 s, not 300–600 ms. Shortening the swing to
0.55–0.70 s dropped the sole p90 to 1.1–1.3 cm and the body still did
not advance. This is an upright in-place march. It is a Prefer FAIL.

## Thaw, and what it did not buy

Swing sat_rate is 0, so ±2.45 Nm is not what the 2 cm clear is short of.
The gap that is still open is forward progress: the alternating feet
land beside each other, and the pelvis does not travel onto the new
lead. A toe CoP at full weight is now inside the thawed clip by a small
margin, and it was not enough, in this controller, to make Δx positive
without a tip. No further plant change is in this PR. Stall torque
3.43 Nm is not the request. The mesh sole is already the 135×76 box
from #43.
