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

Numbers are in `previews/lipm_ab.json`. Stance hip extension during
the swing is 0.070 rad. That is the largest value that stayed upright
for 24 s on this plant. 0.075 rad tips near 12 s.

| Row | Sole median | Sole p90 | Stance slip | sat_rate | min up_z | Δx | Rear unload | Arms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| LIPM00 CPG | 0.98 cm | 1.92 cm | 1.56 cm/s | 0.51 | 0.976 | +67.8 cm | — | 0.67 rad |
| LIPM01 clear 2 cm | 1.16 cm | 1.91 cm | 0.23 cm/s | 0.00 | 0.994 | +3.3 cm | 0.98 | — |
| LIPM02 clear 4 cm | 1.86 cm | 3.96 cm | 0.24 cm/s | 0.008 | 0.995 | +4.1 cm | 0.93 | — |
| LIPM03 clear 2 cm + arms | 1.15 cm | 1.91 cm | 0.24 cm/s | 0.00 | 0.994 | +3.4 cm | 0.98 | 0.21 rad |

LIPM00 is the old gait, including its root wrench. It is the one that
moves, and it is the one that skates. LIPM02 is not the bar. LIPM03 is
the vendor-height row. Four lifts, zero missed gates, sides alternate.
Rear-load share gets down to 0.02 (unload fraction 0.98). At the lifts
that open, the rear foot is still about 11% of the contact force and
the stance CoP is 2.9 cm ahead of the ankle (box center is 3.0 cm, so
this is just heel-side of center, not the toe). CoP margin inside the
box is about 1.1 cm. Swing contact is about 4%. Stance slip is 2.4 mm/s.
sat_rate on the swing is 0. Peak leg torque on the forward clip is
2.35 Nm of 2.45. The shoulder moves 0.21 rad.

Δx is positive. It is also tiny. Mean body speed on the 8.4 s window is
+0.4 cm/s against a 5.6 cm/s command. A 24 s continuation of the same
command stays upright (min up_z 0.994, no fault) and travels +9.2 cm in
12 lifts. The step period is 2.02 s, not 300–600 ms.

The clips match that. The side view shows one sole a couple of
centimetres up and the other planted. The body barely leaves the spot
it started. Arms are off the hips, not a walking swing. It reads as a
slow upright shuffle.

## Why it is still not a walk

The swing does place the free foot a couple of centimetres ahead. During
the weight shift the pelvis rocks back about a centimetre, so most of
the step is given back. Speeding the lateral shift (`shift_k` 2.6 and
up) tips by about 5 s. Cutting the swing to 0.90 s at this push tips by
about 10 s. Cutting it to 0.80 s stays upright and drops sole p90 to
1.3 cm. The sole needs about 0.30 s to reach 1.5 cm and about 0.45 s to
reach 2 cm, so a 300–600 ms step cannot also reach forward and move the
weight. Swing sat_rate is 0, so ±2.45 Nm is not what the clear or the
creep is short of.

No further plant change is in this PR. Stall torque 3.43 Nm is not the
request. The foot box is already the 135×76 box from #43. This is a
Prefer FAIL.

## Published GaitManager clock on the same plant

The trajectory is the Hiwonder / ROBOTIS `wSin` schedule
(`scripts/gait_manager_traj.py`), not a new set of amplitudes.
`walking_param.yaml` and `gait_control_demo.py` use period 400 ms,
`dsp_ratio` 0.2, `step_fb_ratio` 0.028, step height 0.02 m, x amplitude
0.02 m, y swap 0.02 m, z swap 0.006 m, pelvis offset 5 deg.
`gait_manager.py` also lists 300 / 400 / 500 ms at dsp 0.2 and y swap
0.02 m, and 600 ms at dsp 0.1 and y swap 0.04 m. The straight-walk gears
in `ainex_controller.py` use x = 0.012 m at 300 ms (z = 0.015 m),
0.013 m at 400 ms, and 0.015 m at 500 and 600 ms. One period is a left
step plus a right step. The 2 cm figure is the swing-foot height minus
the stance-foot height. `hip_pitch_offset` 15 deg is their offset from
a different init pose and is not added on this stand.

The knee that clears 2 cm needs about 0.62 rad from this stand. The
position servo clips at about 0.053 rad per 20 ms tick, so the rise
alone is about 0.23 s before the leg can come back down. A 400 ms
cycle with dsp 0.2 gives each foot 0.16 s in the air. That is shorter
than the rise.

8.4 s, `vel(+0.056, 0)`, published amplitudes, pelvis 5 deg left on,
stance hip left at the full x map (about 0.24 rad per 2 cm):

| Clock | min up_z | Sole p90 | Sole max | Slip | Δx | Note |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 300 ms, x 0.012 m | 0.984 | 0.11 cm | 0.27 cm | 2.34 cm/s | −3.3 cm | no lift, skate |
| 400 ms demo, x 0.02 m | 0.896 | 0.48 cm | 1.47 cm | 1.49 cm/s | −1.1 cm | tips at 2.6 s |
| 400 ms gear, x 0.013 m | 0.972 | 0.17 cm | 0.33 cm | 2.07 cm/s | −9.3 cm | no lift |
| 600 ms gear, x 0.015 m | 0.892 | 1.00 cm | 2.02 cm | 1.34 cm/s | +1.0 cm | tips at 3.1 s |
| 600 ms table, dsp 0.1 | 0.896 | 0.17 cm | 0.31 cm | 0.72 cm/s | +2.7 cm | tips at 1.6 s |

300–600 ms does not clear about 2 cm and stay upright. The 5 deg pelvis
offset, added on top of the y-swap roll in this joint convention, is
what tips the longer clocks. Turning that offset off and clipping the
stance hip to the 0.070 rad that already survived 24 s is the closest
stable row. The waveform is otherwise the published sine. Period is
1.00 s, which is the shortest raw sine that still reaches a 2 cm sole.

That row, 24 s, `vel(+0.056, 0)`: min up_z 0.992, no fault, sole p90
1.98 cm, stance slip 0.31 cm/s, sat_rate 0, shoulder 0.21 rad, Δx
+46.6 cm. Mean speed is about 1.9 cm/s. The kit range is 7–21 cm/s.
The period is still about 1.7× the slowest published preset and about
2.5× the 400 ms demo. A 0.90 s cycle reaches the same sole only if the
2 cm target is held through 70% of single support instead of tracking
the sine peak. That hold also stays upright for 24 s (Δx +52.4 cm,
slip 0.41 cm/s) and is not the published shape.

The delta-map clips of this 1.00 s row were `previews/gm_forward.mp4`
and `previews/gm_step_closeup.mp4`. Forward window Δx +15.5 cm, mean
body vx +1.9 cm/s, pooled sole p90 2.0 cm (the right foot), min up_z
0.992, peak torque 2.10 Nm. Close-up forward Δx +11.3 cm, then stop.
Those files are re-rendered below from the absolute-x clock. It is
still a Prefer FAIL against a 300–600 ms kit walk. Soft-pass is off.

The pooled sole p90 hides which foot moves. On the same 8.4 s window
the right swing sole is p90 2.03 cm (max 2.19 cm, contact fraction
0.14) and the left swing sole is p90 0.51 cm (max 0.64 cm, contact
fraction 0.48). The left knee does flex, and the foot is briefly
unloaded, but the hip reach is already large at the z peak and the
sole stays on the floor. One foot is stepping. The other is not.

Raising the stance-hip cap from 0.07 rad does not buy the missing
speed. 0.10 rad stays up (min up_z 0.977) and is slower. 0.14 rad
tips at 7.2 s. 0.18 rad tips at 2.6 s. Zeroing swing x while that
foot is loaded, and slewing the landing hip back to 0.07 rad instead
of snapping it, both drop the sole below 1 cm and do not raise Δx.

A 0.15 s lead on y-swap, so the lean arrives before the knee flex,
does get both soles near 1.8 cm and then tips at 2.8 s. Clamping that
lead so it cannot reverse onto the swing foot keeps the robot up, and
at 0.32 s the left sole p90 reaches 2.30 cm, but min up_z falls to
0.955, the body drifts 8.7 cm to the side, and forward speed falls to
about 0.6 cm/s. Holding the full y-swap for the whole single-support
interval never unloads the left foot.

Shorter periods with the 0.07 rad cap stay upright and lose both
clearance and forward speed. 0.80 s raw is about 1.2 cm/s with sole
p90 1.10 cm. The 1.00 s one-sided step is still the fastest upright
row, and it is still about 4× short of 7 cm/s. Soft-pass is off.

## Joint sign map against the plant axes

The plant file was not edited. Axes below are from
`ainex_controls_m2_145.xml` (md5 `17dc4ff37491c8e61900fd83b5d31f0c`).
Pitch is mirrored. Ankle pitch is flipped the other way from hip and
knee. Roll is not mirrored.

| Joint | Left axis | Right axis | Gait sign for a positive command |
| --- | --- | --- | --- |
| Hip roll | `-1 0 0` | `-1 0 0` | same `lat` added to both (not mirrored) |
| Hip pitch | `0 1 0` | `0 -1 0` | `dhip > 0` decreases left, increases right |
| Knee | `0 1 0` | `0 -1 0` | `flex > 0` increases left, decreases right |
| Ankle pitch | `0 -1 0` | `0 1 0` | command is hip+knee, plus a level term |
| Ankle roll | `1 0 0` | `1 0 0` | same `lat` on both; `+ank_roll` lowers `up[1]` on both |

Pelvis fixed at the stand, ankle held on hip+knee: `flex = 0.62` raises
the left sole by 2.31 cm and the right sole by 2.31 cm. `dhip = +0.24`
moves both feet +4.4 cm in x. The same-sign hip-roll step moves both
feet in −y. `+l_ank_pitch` lowers foot `up[0]`; `+r_ank_pitch` raises
it, which is what the level term assumes. No pitch sign is backwards,
and roll is not being mirrored.

On the delta-from-t=0 map, the live 1.00 s row did not use those
lifts the same way. At the left sole peak, commanded `dhip` was +0.20
(thigh forward) and `flex` was +0.31, and the sole was 0.48 cm. At
the right sole peak, commanded `dhip` was −0.13 (thigh back) and
`flex` was +0.30, and the sole was 1.96 cm. No swing joint sat on
±2.45 Nm. That split is the t=0 delta, not a wrong pitch sign.

## Lift phase against the published clock

`wSin` phases in `gait_manager_traj.py` match the ROBOTIS OP2 walker
that Hiwonder's `walking_module.so` embeds:

| Channel | Phase constant | Period |
| --- | --- | --- |
| x move | π/2 | `period * ssp` |
| z move | π/2 | `period * ssp / 2` |
| x swap | π | `period / 2` |
| z swap | 3π/2 | `period / 2` |

x and z use the same phase constant. z's period is half of x's, so the
height sine finishes one full cycle during a swing while fore-aft
finishes half a cycle. The height peak is mid-swing. It is not a
quarter-cycle ahead of fore-aft, and it is not a cosine against a sine.

The right swing adds π to x, y, and yaw only. z does not get that
offset. That is the OP2 `computeLegAngle` branch: the extra π is on
the fore-aft channels, and z stays anchored to the start of that
foot's single support.

At period 1.00 s, dsp 0.2, x 0.02 m, z 0.02 m, with the steady-cycle
half-step already in `previous_x`:

| | t | phase | absolute x (both feet) | delta x from t=0 | foot gap |
| --- | ---: | --- | --- | --- | ---: |
| Left height peak | 0.25 s | L | 0.00 cm | L +2.00 cm, R −2.00 cm | +2.00 cm |
| Right height peak | 0.75 s | R | 0.00 cm | L +2.00 cm, R −2.00 cm | −2.00 cm |

t=0 is already split by ±x_move (left −2 cm, right +2 cm). The delta
from that split is still ±2 cm when the published endpoint is under
the hip. Mapping the delta commanded the left thigh forward and the
right thigh back at the height peak. The half-step offset was not
missing on z.

The hip command now tracks that absolute x. 0 is under the hip.
`HIP_RISE` is the LIPM Bézier's extra rise hip and is not added on
this clock. Both flex commands peak at 0.616 rad with `|dhip| ≤ 0.02`.

24 s, `vel(+0.056, 0)`, same 1.00 s row, pelvis 0, stance hip cap
0.070 rad: min up_z 0.970, no tip, sat_rate 0, peak torque 2.14 Nm,
knee band 0.94 (inside the clip, not past it). Left swing sole p90
0.78 cm (max 1.08 cm, contact 0.14). Right swing sole p90 0.88 cm
(max 1.81 cm, contact 0.18). Δx +14.1 cm, mean body vx +0.56 cm/s.
The 8.4 s clip window is min up_z 0.989, Δx +12.9 cm, mean vx
+1.43 cm/s, then the body almost stops: the next 15.6 s add about
1.2 cm.

At the flex-command peak the hip command is under the hip. The sole
there stays about 0.9 cm on the left and 1.6 cm on the right. The
knee command is held to about 0.053 rad per tick, so the joint does
not reach the 0.62 rad pose before the sine falls. The highest sole
samples are still just before the hip crossing (left command dhip
−0.13 at 1.08 cm, right −0.10 at 1.81 cm). No joint sat on ±2.45 Nm.

This is a Prefer FAIL against both feet at about 2 cm and 7–21 cm/s.
The previous one-sided row was faster (about 1.9 cm/s) and cleared
only the right foot, by commanding that thigh back at mid-swing. That
pose is the t=0 delta, not the published endpoint. Soft-pass is off.

`previews/gm_forward.mp4` and `previews/gm_step_closeup.mp4` were that
absolute-x row before the HX slew below. Forward clip then: Δx +12.7 cm,
mean body vx +1.4 cm/s, pooled sole p90 0.8 cm, min up_z 0.989, peak
torque 2.14 Nm, no tip. Those files are re-rendered from the slew.

## Command tick and the HX slew

The gait command is written at **50 Hz** (`CTRL_DT` 0.02 s). The plant
timestep stays 0.002 s (10 substeps per command). It is not a 100 Hz tick.

`0.98 * 2.45 / 45 = 0.0534` rad. At 50 Hz that is 2.67 rad/s. At 100 Hz
it would be 5.34 rad/s. The knee and hip pitch share that linear band
(kp 45).

Published no-load speed, [HX-35H](https://www.hiwonder.com/products/hx-35h):
0.18 s/60° at 11.1 V is 5.8 rad/s. The hip HX-35HM is 0.19 s/60°, 5.5 rad/s.
Knee and hip pitch now lead the joint by `5.5 rad/s * 0.02 s = 0.110 rad`
per command. That is under the 5.8 rad/s ceiling. Ankle, hip roll, and
hip yaw stay on the linear band. Plant `forcerange`, kp, and ±2.45 Nm
are not changed. md5 `17dc4ff37491c8e61900fd83b5d31f0c`.

The joint does not reach 5.5 rad/s. Knee speed peaks around 2.4–2.6 rad/s.
Hip pitch force hits ±2.45 Nm from the first walking command. The split
of that force is in the next section.

Same absolute-x map, pelvis 0, stance hip cap 0.070 rad, dsp 0.2, x = z = 0.02 m:

| Period | min up_z | Tip | Sole p90 L / R | Mean vx | sat_rate | Peak torque |
| --- | ---: | --- | --- | ---: | ---: | ---: |
| 1.00 s | 0.570 | 4.44 s, COM outside support | 2.0 / 1.0 cm before 3 s, then the fall | — | 0.24 | 2.45 Nm |
| 0.60 s, 24 s | 0.977 | no | 0.83 / 0.69 cm | +4.0 cm/s | 0.20 | 2.45 Nm |
| 0.50 s, 8.4 s | 0.988 | no | 0.61 / 0.47 cm | +4.5 cm/s | 0.20 | 2.45 Nm |
| 0.40 s, 24 s | 0.986 | no | 0.20 / 0.62 cm | +4.5 cm/s | 0.36 | 2.45 Nm |
| 0.30 s, 8.4 s | 0.961 | no | 0.30 / 0.04 cm | −0.8 cm/s | 0.39 | 2.45 Nm |

On the 1.00 s row the knee joint reaches about 0.50 rad, not 0.62, and
the body tips at 4.44 s (Δy about +15 cm). On 0.60 s the flex command
still peaks under the hip (`|dhip| ≤ 0.02`) but the knee joint only
reaches about 0.36 rad before the sine falls. Δx over 24 s is +95 cm.
That is still short of 7–21 cm/s and short of 2 cm on both feet.
Soft-pass is off.

The re-rendered clips are the 1.00 s row with this slew, so the tip is
in the picture. Forward faults at 4.42 s. Close-up faults at 4.60 s.
After the fault the body reaches min up_z −1. Constrained Baseline,
yuv420p, `+faststart`.

## Actuator force split

Same 1.00 s row, logged every physics step (0.002 s). The position
actuator is `kp·(ctrl−q) − kv·qvel`, then clipped to ±2.45 Nm.
`kv` is the compiled `dampratio="1"` coefficient (`−biasprm[2]`).
The unconstrained sum matches `actuator_force` to 1e-15 Nm inside the
rail. Plant XML was not edited. md5 stays
`17dc4ff37491c8e61900fd83b5d31f0c`.

| Actuator | kp | kv (N·m·s/rad) | speed where \|kv·v\| = 2.45 |
| --- | ---: | ---: | ---: |
| Hip pitch | 45 | 1.810 | 1.35 rad/s |
| Knee | 45 | 1.457 | 1.68 rad/s |

`2·sqrt(kp·armature)` with armature 0.01 is 1.34. The compiled kv is
higher because dampratio uses the reflected joint inertia. Passive
joint damping is 0.08 and is not part of `actuator_force` (0.16 Nm at
2 rad/s).

On the ±2.45 Nm rail the position term is the large one. Right hip
pitch first hits the rail at 0.402 s with velocity −0.001 rad/s,
`kp·error` +3.15 Nm, damping 0.00 Nm, applied +2.45 Nm. Across 239
saturated hip-pitch samples, `|kp·error|` is larger than `|damping|`
on every sample (median 3.28 Nm vs 0.57 Nm, median speed 0.31 rad/s).
The left hip is the same shape at the same instant. The right knee
first hits the rail at 0.982 s: velocity −0.39 rad/s, position −3.02 Nm,
damping +0.56 Nm. Position is larger on 96% of saturated knee samples.

| t (s) | Joint | vel (rad/s) | kp·error | damping | raw | applied |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 0.402 | R hip pitch | −0.00 | +3.15 | 0.00 | +3.15 | +2.45 |
| 0.82 | R hip pitch | +0.42 | −1.77 | −0.76 | −2.53 | −2.45 |
| 0.94 | R hip pitch | +0.47 | −3.42 | −0.86 | −4.28 | −2.45 |
| 0.982 | R knee | −0.39 | −3.02 | +0.56 | −2.45 | −2.45 |
| 2.386 | R hip pitch, peak \|v\| | −2.06 | −3.79 | +3.72 | −0.07 | −0.07 |
| 1.268 | R knee, peak \|v\| | +2.76 | +4.23 | −4.02 | +0.21 | +0.21 |

At the peak speeds the two terms are both about 4 Nm and they cancel,
so the applied force is near zero. Hip damping magnitude equals the
whole ±2.45 Nm at 1.35 rad/s, and knee damping does that at 1.68 rad/s.
That cancellation is why the joint stops accelerating near 2–2.7 rad/s.
It is not the sample that puts `actuator_force` on the rail. The rail
is the position error while the joint is still slow. Dampratio stays
as it is. Forcerange stays ±2.45 Nm.

## Move-time ramp

`write_clipped` runs once per 50 Hz tick, before any `mj_step`, and
stores the end-of-tick target in `data.ctrl`. The plant then takes 10
steps of 0.002 s. Before this change those 10 steps held that target
as a hard step, so the first substep saw the whole position error at
nearly zero speed.

The physics loop now ramps `ctrl` from the previous command to the new
target across those 10 steps. The last substep lands on the target.
Move time is the control tick, 20 ms. Plant kp, dampratio, forcerange,
and armature are unchanged.

On the same 1.00 s row the right hip still hits ±2.45 Nm. The first
hit moves from 0.402 s to 0.420 s, which is the last substep of the
first walking tick. Velocity there is +0.27 rad/s, `kp·error` is
+3.06 Nm (0.068 rad), damping is −0.48 Nm, applied is +2.45 Nm.
Saturated hip samples fall from 239 to 81. Position is still the
larger term on every one of them. Both knees stay off the rail
(peak force 2.23 Nm and 2.32 Nm).

The 0.07 rad hip step does not fit in 20 ms under ±2.45 Nm, so the
end of the move still has about 0.068 rad of error. 8.4 s of
`vel(+0.056, 0)` stays up: min up_z 0.938, no tip, sole p90 1.19 cm
left and 1.07 cm right, mean vx +2.4 cm/s, sat_rate 0.25, peak torque
2.45 Nm. The same row run to 24 s tips at 10.52 s, COM outside
support. 0.60 s for 24 s stays up, mean vx +3.8 cm/s, sole p90
0.42 / 0.62 cm, sat_rate 0.34. Both feet are still short of 2 cm and
the speed is still short of 7 cm/s. Soft-pass is off.
