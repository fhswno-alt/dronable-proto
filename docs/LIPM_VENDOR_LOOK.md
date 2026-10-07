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

Those clip files were that 20 ms ramp. The capped row below replaces them.

## Torque-budget command step

Hip and knee pitch no longer take the 0.110 rad HX lead. Each 50 Hz
tick moves those commands by at most 0.050 rad, and the stored target
stays within 0.050 rad of the measured joint. `2.45/45 ≈ 0.054` rad is
the error that spends the whole rail when damping is about zero, so
0.050 rad leaves `kp·error` at 2.25 Nm. A gait target past that cap
continues on later ticks. The physics loop still ramps `ctrl` across
the 20 ms tick. Plant kp, dampratio, forcerange, and armature are
unchanged. md5 stays `17dc4ff37491c8e61900fd83b5d31f0c`.

On the 1.00 s row the hip and knee rails are clear. Logged every
0.002 s for 8.4 s, inside the rail the position-minus-damping sum
matches `actuator_force` to 1e-14 Nm. No hip or knee sample reaches
±2.45 Nm. The largest right-hip force is +2.03 Nm at 0.580 s, with
velocity −0.017 rad/s, `kp·error` +2.00 Nm, damping +0.03 Nm. That is
the same slow, position-dominated shape as the old rail hit, and it
stops under the clip because the lead is capped. Max `|ctrl−q|` on the
right hip is 0.050 rad. Peak hip speed is about 0.9 rad/s and peak
knee speed about 1.2 rad/s. At those peaks the two terms still cancel
and the applied force is a few tenths of a newton-metre. sat_rate is 0.
Peak leg torque on the 8.4 s run is 2.03 Nm.

The vendor mid-swing pose does not fit in that budget. Knee travel off
the stand pose is about 0.26 rad, not 0.62. Soles stay near half a
centimetre. Speed stays near zero.

| Run | min up_z | Tip | Sole p90 L / R | Mean vx | sat_rate | Peak torque |
| --- | ---: | --- | --- | ---: | ---: | ---: |
| 1.00 s, 8.4 s | 0.988 | no | 0.39 / 0.57 cm | +0.7 cm/s | 0 | 2.03 Nm |
| 1.00 s, 24 s | 0.979 | no | 0.51 / 0.62 cm | −0.2 cm/s | 0 | 2.04 Nm |
| 0.60 s, 24 s | 0.987 | no | 0.10 / 0.25 cm | +1.2 cm/s | 0 | 2.34 Nm |

Δx on the 1.00 s, 8.4 s run is +7.3 cm. Over 24 s it is −0.9 cm. The
0.60 s row moves +26 cm in 24 s. Both feet are short of 2 cm. Speed is
short of 7 cm/s. The body stays upright. Soft-pass is off. This is not
a kit walk. ±2.45 Nm can keep the position term off the rail when the
command lead is 0.050 rad, and it does not track the published
mid-swing knee pose in the swing time.

The clips are this capped 1.00 s row. Forward 8.400 s stays up
(Δx +7.1 cm, mean vx +0.7 cm/s, pooled sole p90 0.5 cm, min up_z
0.988, peak torque 2.03 Nm, sat_rate 0). Close-up 7.600 s stays up
through the stop (forward Δx +1.6 cm, stop Δx −1.8 cm, min up_z 0.969,
peak torque 2.38 Nm). Constrained Baseline, yuv420p, `+faststart`.

## Slower cadence

The 0.050 rad tick and the 20 ms physics ramp stay. The only change in
this section is the GaitManager period. dsp 0.20, x = z = y_swap 0.02 m,
pelvis 0, stance hip cap 0.070 rad. Plant md5 stays
`17dc4ff37491c8e61900fd83b5d31f0c`.

Published swing-foot x travel is 4.07 cm at every period. Two swings
per cycle, so the foot-travel ceiling is `8.14 cm / period`. Kit speed
of 7 cm/s needs that ceiling at or above 7, which is a period of
1.16 s or faster. A period slow enough for the capped knee to finish
0.62 rad is slower than that.

Knee flex below is the peak off the stand pose while `up_z` is still
at least 0.95. Sole p90 is the swing foot in that same upright window.

| Period | Upright? | Knee flex L / R | Sole p90 L / R | Mean vx | Foot-travel ceiling | Peak torque |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1.00 s, 24 s | yes, min up_z 0.979 | 0.26 / 0.25 | 0.51 / 0.62 cm | −0.2 cm/s | 8.1 cm/s | 2.04 Nm |
| 1.20 s, 24 s | yes, min up_z 0.986 | 0.30 / 0.29 | 1.17 / 1.27 cm | +1.9 cm/s | 6.8 cm/s | 2.01 Nm |
| 1.35 s, 24 s | yes, min up_z 0.984 | 0.33 / 0.32 | 1.06 / 1.27 cm | +1.7 cm/s | 6.0 cm/s | 1.98 Nm |
| 1.50 s | tips at 6.46 s | 0.35 / 0.38 | 0.80 / 0.98 cm | +1.2 cm/s | 5.4 cm/s | 2.45 Nm in the fall |
| 2.00 s | tips at 5.40 s | 0.42 / 0.44 | 1.21 / 1.44 cm | +1.8 cm/s | 4.1 cm/s | 2.45 Nm in the fall |
| 3.00 s | tips at 4.66 s | 0.56 / 0.56 | 1.56 / 2.35 cm | +1.4 cm/s | 2.7 cm/s | 2.26 Nm |
| 4.00 s | collapses at 5.50 s | 0.60 / 0.61 | 2.32 / 2.97 cm | +1.0 cm/s | 2.0 cm/s | 2.37 Nm |

At 4.00 s the capped knee does reach about 0.61 rad and both swing
soles clear about 2 cm while the torso is still upright. The body then
collapses at 5.50 s. At 3.00 s the right sole p90 is 2.35 cm and the
knee is 0.56 rad, and the body tips at 4.66 s. The rows that stay up
for 24 s stop at 1.35 s: knee about 0.33 rad, sole p90 about 1.2 cm.

Swing sat_rate stays 0 on these rows. Hip and knee force stays under
±2.45 Nm while the body is upright (right hip peak about 2.03 Nm at
1.50 s before the fall, about 2.36 Nm at 4.00 s). The ±2.45 Nm samples
on the 1.50 s and 2.00 s rows are ankle roll as the torso is already
dropping.

Measured speed on every row is about 1–2 cm/s. The 4.00 s row that
reaches the pose has a foot-travel ceiling of 2.0 cm/s, under the
7 cm/s kit floor, and it does not stay up. Soft-pass is off. This is
not a kit walk. The 1.00 s clips were not re-rendered.

## Longer swing-foot x

The 0.050 rad tick and the 20 ms ramp stay. Stance hip stays clipped
at 0.070 rad. `gm_x_m` is raised so the clock's swing-foot travel makes
`2·Δx / period` at least 7 cm/s. Plant md5 stays
`17dc4ff37491c8e61900fd83b5d31f0c`.

`kin Δx` is that clock travel. `ach Δx` is the median distance the
swing foot actually moves along heading while `up_z` is still at least
0.95. Knee flex and sole p90 use the same upright window.

| Period | x command | kin Δx | kin ceiling | ach Δx | Knee L / R | Sole p90 L / R | Mean vx | Result |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1.35 s, 24 s | 2.32 cm | 4.7 cm | 7.0 cm/s | 4.2 cm | 0.33 / 0.32 | 1.15 / 1.31 cm | +1.6 cm/s | stays up, min up_z 0.982 |
| 1.35 s, 24 s | 3.32 cm | 6.8 cm | 10.0 cm/s | 4.2 cm | 0.32 / 0.33 | 1.17 / 1.44 cm | +1.3 cm/s | stays up, step did not grow |
| 2.00 s | 3.44 cm | 7.0 cm | 7.0 cm/s | 3.3 cm | 0.42 / 0.43 | 1.13 / 1.40 cm | +1.6 cm/s | tips at 3.40 s |
| 3.00 s | 5.16 cm | 10.5 cm | 7.0 cm/s | 6.1 cm | 0.53 / 0.56 | 1.48 / 2.25 cm | +1.2 cm/s | collapses at 4.28 s |
| 4.00 s | 6.88 cm | 14.0 cm | 7.0 cm/s | 9.1 cm | 0.60 / 0.61 | 2.97 / 3.34 cm | +1.4 cm/s | tips at 5.46 s |
| 4.00 s | 13.8 cm | 28.0 cm | 14.0 cm/s | 9.8 cm | 0.60 / 0.61 | 3.33 / 4.85 cm | +1.1 cm/s | tips at 5.14 s |

At 1.35 s the body stays up for 24 s. Raising the clock from a 7 cm/s
ceiling to a 10 cm/s ceiling leaves the median step at 4.2 cm, the knee
at 0.33 rad, and the sole p90 near 1.3 cm. Mean vx stays about
+1.5 cm/s. Swing sat_rate is 0. Upright hip-pitch peak is 2.00 Nm,
knee 1.53 Nm, ankle roll 0.90 Nm.

At 4.00 s the knee still reaches about 0.61 rad and the soles are up
while `up_z` is at least 0.95, then the body tips around 5.1–5.5 s.
The median realized step stops near 10 cm even when the clock asks for
28 cm. `2·10 cm / 4 s` is 5 cm/s. Measured vx is about +1.4 cm/s.
Upright peaks on that row: hip pitch 2.33 Nm, knee 1.56 Nm, ankle roll
2.09 Nm. The ±2.45 Nm samples are ankle roll on the 2.00 s rows once
`up_z` is already about 0.6.

A longer commanded step does not produce 7 cm/s, and it does not keep
the 0.62 rad / 2 cm pose upright. The 1.00 s clips were not re-rendered.
Soft-pass is off. This is not a kit walk.

## Multi-tick move time

The 0.050 rad starve-cap is off. Hip pitch, knee, and ankle pitch store
the gait target. The physics loop approaches that target over a kit-style
move time, at the 0.002 s rate, and not faster than 5.5 rad/s. One 20 ms
tick covers `0.02 / move_time` of the remaining gap. Plant md5 stays
`17dc4ff37491c8e61900fd83b5d31f0c`. x stays 0.02 m, so the foot-travel
ceiling is `8.14 cm / period`. A period of 1.16 s is the fastest that
ceiling still puts at 7 cm/s.

Knee flex and sole p90 are while `up_z` is at least 0.95. Upright force
peaks use the same window. The first rail, when there is one, is the
pre-step split `kp·(ctrl−q) − kv·qvel`.

| Move | Period | Upright 24 s? | Knee L / R | Sole p90 L / R | Mean vx | First rail while upright | Upright hip peak |
| --- | ---: | --- | ---: | ---: | ---: | --- | ---: |
| 100 ms | 1.16 s | tips at 5.20 s | 0.49 / 0.49 | 1.90 / 2.00 cm | +0.6 cm/s | R hip 0.592 s, ω −0.06, kp·e +2.35, damp +0.11, F +2.45, up 1.00 | 2.45 Nm |
| 100 ms | 1.00 s | tips at 2.42 s | 0.46 / 0.46 | 1.48 / 1.56 cm | +1.8 cm/s | R hip 0.574 s, same shape, F +2.45, up 1.00 | 2.45 Nm |
| 150 ms | 1.16 s | yes, min up_z 0.995 | 0.42 / 0.42 | 1.62 / 2.05 cm | +1.7 cm/s | none, sat_rate 0 | 2.43 Nm |
| 150 ms | 1.00 s | tips at 11.76 s | 0.39 / 0.39 | 1.27 / 1.58 cm | +0.2 cm/s | R hip 1.864 s, ω +0.23, kp·e −2.04, damp −0.42, F −2.45, up 1.00 | 2.45 Nm |
| 200 ms | 1.16 s | yes, min up_z 0.995 | 0.37 / 0.37 | 1.29 / 1.90 cm | +0.8 cm/s | none, sat_rate 0 | 2.25 Nm |
| 200 ms | 1.00 s | yes, min up_z 0.994 | 0.34 / 0.34 | 1.35 / 1.82 cm | +1.1 cm/s | none, sat_rate 0, peak torque 2.13 Nm | 2.13 Nm |

At 100 ms the right hip rails while the body is still upright, with
speed about zero and the position term on the clip. Shorter periods at
100 ms do the same, except 0.60 s stays up for 24 s with the hip already
on the rail, knee 0.37 rad, sole p90 about 1.4 cm, and mean vx +2.4 cm/s.

At 150 ms and period 1.16 s the body stays up for 24 s and the hip does
not rail. Upright hip peak is 2.43 Nm, knee 1.21 Nm, ankle pitch 1.97 Nm,
ankle roll 1.44 Nm. The knee stops at 0.42 rad. The right sole p90 is
2.05 cm and the left is 1.62 cm. Mean vx is +1.7 cm/s. The same move at
1.00 s rails at 1.86 s and tips at 11.76 s.

At 200 ms and period 1.16 s there is again no rail, and the knee only
reaches 0.37 rad. Mean vx is +0.8 cm/s.

The shipped move is 150 ms. None of these rows reaches 0.62 rad, both
soles at about 2 cm, and 7 cm/s together. The 100 ms move is the one
that spends the ±2.45 Nm rail while upright. Soft-pass is off. This is
not a kit walk.

The clips are the 1.00 s row with this 150 ms move. Forward 8.400 s
stays up (Δx −0.1 cm, mean vx −0.3 cm/s, pooled sole p90 1.4 cm,
min up_z 0.953, peak torque 2.45 Nm, sat_rate 0.08). A run past the
clip tips at 11.76 s. Close-up 7.600 s stays up through the stop
(forward Δx +12.1 cm, mean vx +2.2 cm/s, stop Δx −1.7 cm, min up_z
0.993, peak torque 2.34 Nm). Constrained Baseline, yuv420p,
`+faststart`.

## Deeper swing on the 1.16 s baseline

The 150 ms move and the 1.16 s period stay. Hip pitch is not given a
larger step. The swing knee command on this clock is 0.75 rad instead
of 0.62 rad, so the lagged knee lands higher. The LIPM Bézier is still
0.62 rad. Plant md5 stays `17dc4ff37491c8e61900fd83b5d31f0c`.

At 24 s, upright, min up_z 0.996: knee 0.51 / 0.51 rad, sole p90
2.11 / 2.69 cm, mean vx +1.71 cm/s, sat_rate 0.001. Upright peaks: hip
2.44 / 2.32 Nm, knee 1.14 / 1.34 Nm, ankle pitch 1.44 / 2.04 Nm, ankle
roll 1.72 / 1.92 Nm. No sample reached ±2.45 Nm. The 0.001 sat_rate is
the 98% flag (2.40 Nm). Both soles clear 2 cm. The knee does not reach
0.62 rad.

A 0.78 rad command puts the right hip on the rail while `up_z` is still
1.00 (0.620 s, ω −0.06, kp·e +2.34, F +2.45). A 0.90 rad command reaches
knee 0.61 rad and sole p90 2.83 / 3.31 cm, and the same hip rail at
0.604 s. Holding the 0.62 rad target through single support rails the
hip at 0.52 s and tips by 1.8 s. A faster knee move, hip still at 150 ms,
does the same: 120 ms reaches knee 0.46 rad and tips at 6.10 s after a
hip rail at 2.10 s; 100 ms rails the hip at 0.62 s.

## Why 1.7 cm/s against a 7.0 cm/s ceiling

Clock swing-foot travel at x = 0.02 m is 4.07 cm, so
`2 · 4.07 cm / 1.16 s = 7.0 cm/s`. On the 24 s row the clock still
runs about −1.94 cm to +2.01 cm in the swing (3.95 cm). The body does
not.

Per swing, while `up_z` is at least 0.95 (41 swings):

| Quantity | Value |
| --- | ---: |
| Clock Δx | 3.95 cm |
| Swing-foot world Δx | +0.21 cm |
| Of which, while the foot is unloaded | +0.79 cm |
| Foot still in contact | 0.16 of the swing |
| Swing foot relative to the body | +0.20 cm |
| Body Δx during the swing | +0.01 cm |
| Stance-foot slip | +0.16 cm |
| Swing hip command, peak to peak | 0.474 rad |
| Swing hip achieved, peak to peak | 0.212 rad |
| Stance hip command (clipped) | 0.070 rad |
| Stance hip achieved | 0.089 rad |

The foot is mostly in the air, and it does not travel the clock. The
hip is asked for 0.47 rad and delivers 0.21 rad, which is about 0.8 cm
of airborne travel instead of 4 cm. Stance slip is +0.16 cm, forward,
not a backward skate. The stance hip stays on the 0.070 rad cap, and
the body barely moves during the swing. Mean vx stays +1.7 cm/s.
Soft-pass is off. This is not a kit walk.

Those clips were replaced by the hip-lead row below.

## Stance clip and swing-hip lead

The 150 ms move, the 5.5 rad/s cap, and the 1.16 s period stay. Plant
md5 stays `17dc4ff37491c8e61900fd83b5d31f0c`.

Raising the stance-hip clip rails the right hip while `up_z` is still
1, before the body moves. At 0.075 rad the right hip hits the rail at
0.622 s (ω −0.06, kp·e +2.35, damp +0.10, F +2.45). Mean vx is
+1.74 cm/s against +1.67 cm/s at 0.070 rad. Stance hip achieved is
0.086 rad. At 0.110 rad the same rail is immediate and the body tips
at 7.96 s. The 0.070 rad clip stays.

The swing hip is read 0.22 s ahead of the live clock, and the swing
knee command is 1.00 rad. The 150 ms move is unchanged. At 24 s,
upright, min up_z 0.997, sat_rate 0, no sample at ±2.45 Nm:

| | |
| --- | ---: |
| Knee L / R | 0.68 / 0.68 rad |
| Sole p90 L / R | 2.18 / 2.65 cm |
| Mean vx | +2.79 cm/s |
| Swing hip command / achieved, inside the swing | 0.255 / 0.242 rad |
| Stance hip command / achieved | 0.070 / 0.134 rad |
| Clock Δx | 4.07 cm |
| Swing-foot world Δx | +1.96 cm |
| Airborne foot Δx | +1.96 cm |
| Body Δx during the swing | +0.94 cm |
| Stance slip | +0.32 cm |
| Upright hip peak | 1.76 / 1.61 Nm |
| Upright knee peak | 1.12 / 1.43 Nm |
| Upright ankle pitch | 1.15 / 1.97 Nm |
| Upright ankle roll | 1.80 / 2.13 Nm |

The lead does not make the joint track the live 0.47 rad clock. Inside
the swing the led command is 0.26 rad peak to peak and the joint
matches that. Airborne travel is 2.0 cm, not 4 cm. Mean vx is
+2.8 cm/s, not 7 cm/s. A 0.20 s lead puts the right hip on the rail at
7.89 s while `up_z` is 1. Both soles stay above 2 cm, and the knee
clears 0.62 rad, because the hip peak fell to about 1.8 Nm and the
taller knee command fits under the rail. Soft-pass is off.

Those clips were replaced by the swing-hip gain row below.

## Swing-hip gain on the 0.22 s lead

The 150 ms move, the 5.5 rad/s cap, the 1.16 s period, and the 0.070 rad
stance clip stay. Plant md5 stays `17dc4ff37491c8e61900fd83b5d31f0c`.
The swing hip command, and only that command, is multiplied by 2.80.
The stance clip is not scaled. The swing knee command is 1.90 rad so
the sole stays up while the hip reaches. The peak of that knee target
is past the ±2.09 rad ctrlrange and that one sample is clipped; the
rise is not. 1.69 rad, which stays inside the range, puts the left hip
on the rail at this gain.

Clock swing-foot travel is still 4.07 cm (`2 · 4.07 cm / 1.16 s = 7.0
cm/s`). At 24 s, upright, min up_z 0.997, sat_rate 0, no hip-pitch
sample at ±2.45 Nm (41 swings):

| | |
| --- | ---: |
| Knee L / R | 1.18 / 1.18 rad |
| Sole p90 L / R | 4.51 / 4.85 cm |
| Mean vx | +7.20 cm/s |
| Swing hip command / achieved, inside the swing | 0.713 / 0.632 rad |
| Stance hip command / achieved | 0.070 / 0.369 rad |
| Clock Δx | 4.07 cm |
| Swing-foot world Δx | +7.72 cm |
| Airborne foot Δx | +7.59 cm |
| Body Δx during the swing | +3.17 cm |
| Stance slip | −0.64 cm |
| Retract into the next swing | +0.59 cm |
| Upright hip peak R / L | 1.79 / 2.18 Nm |
| Upright knee peak R / L | 1.38 / 1.80 Nm |
| Upright ankle pitch R / L | 1.29 / 1.32 Nm |
| Upright ankle roll R / L | 1.66 / 2.09 Nm |

The scaled command inside the swing is 0.71 rad peak to peak and the
joint reaches 0.63 rad, past the live clock's 0.47 rad. Airborne
travel is 7.6 cm, past the 4 cm clock, because the hip command was
scaled past that clock. Mean vx is +7.20 cm/s. The body during one
swing moves +3.2 cm, which is not the whole airborne foot travel.
Stance slip is a modest −0.64 cm. The stance command stays on the
0.070 rad clip; the stance joint at 0.37 rad is coupling, not a raised
clip.

The next step hits the rail before it adds a usable margin. Gain 3.20
at the same 1.90 rad knee, on an 8.4 s window, reaches vx +7.48 cm/s
and then the left hip rails at 2.872 s while `up_z` is 1 (ω −0.83,
kp·e −3.95, damp +1.50, F −2.45). sat_rate is 0.030. A 0.26 s lead at
gain 2.80 shrinks the in-swing command to 0.50 rad, rails the left hip
at 2.848 s while `up_z` is 1 (ω −0.88, kp·e −4.07, damp +1.59, F −2.45),
and mean vx on that same 8.4 s window is +5.48 cm/s (sat_rate 0.060).
The 0.22 s / gain 2.80 row on that 8.4 s window is +6.47 cm/s with no
rail and the same 2.18 Nm left-hip peak. Soft-pass is off. This is
not a kit walk beyond these bars.

Those clips were replaced by the sole-lift row below.

## Sole lift toward 2 cm, then cadence

The 150 ms move, the 5.5 rad/s cap, the 1.16 s period, and the 0.070 rad
stance clip stay for this row. Plant md5 stays
`17dc4ff37491c8e61900fd83b5d31f0c`. The swing knee command is 1.05 rad.
Stand knee is 0.40 rad, so the peak target is 1.45 rad, inside the
±2.09 ctrlrange. No knee sample is clipped. The swing-hip gain is 1.75.
Gain 2.80 with a knee near this height rails the hip: at flex 1.15 and
gain 2.80, on an 8.4 s window, sole p90 is 2.27 / 2.65 cm and vx is
+8.18 cm/s, and the left hip is on ±2.45 Nm.

24 s, min up_z 0.997, no hip-pitch sample at ±2.45 Nm:

| | |
| --- | ---: |
| Period | 1.16 s |
| Knee command peak | 1.45 rad |
| Knee achieved L / R | 0.71 / 0.71 rad |
| Sole p90 L / R | 2.08 / 2.01 cm |
| Mean vx | +5.02 cm/s |
| Swing hip command / achieved | 0.446 / 0.403 rad |
| Stance hip command / achieved | 0.070 / 0.233 rad |
| Airborne foot Δx | +5.25 cm |
| Body Δx during the swing | +2.27 cm |
| Stance slip | −0.26 cm |
| Upright hip pitch peak R / L | 1.74 / 1.95 Nm |
| Upright knee peak R / L | 0.99 / 1.88 Nm |
| Upright ankle pitch R / L | 1.01 / 1.63 Nm |
| Upright ankle roll R / L | 1.50 / 2.36 Nm |
| sat_rate | 0.005 |

sat_rate 0.005 is five swing samples on the left hip roll (2.41 to
2.45 Nm), one of them on the roll rail. Hip pitch stays at 1.95 Nm.
Mean vx is +5.02 cm/s. The sole is at the kit height and the knee
command is inside the ctrlrange. Soft-pass is off.

Shortening the period from here does not reach a kit step. With the
0.22 s lead left in place, on 8.4 s windows:

| Period | Mean vx | min up_z | Sole p90 L / R | Hip pitch peak | Note |
| --- | ---: | ---: | ---: | ---: | --- |
| 1.00 s | +4.11 cm/s | 0.987 | 1.89 / 1.74 cm | 2.16 Nm | sole under 2 cm |
| 0.60 s | +1.11 cm/s | 0.996 | 0.67 / 0.87 cm | 1.93 Nm | sole collapses |
| 0.40 s | −2.43 cm/s | 0.992 | 0.51 / 0.12 cm | rail | left hip pitch at 0.528 s, ω −0.32, kp·e +1.89, damp +0.58, F +2.45, up_z 1 |

Scaling the lead with the period (0.114 s at 0.60 s) reaches vx
+6.50 cm/s with sat_rate 0 and hip pitch peak 1.48 Nm, and the sole
p90 falls to 0.87 / 0.99 cm. A faster knee-only move (80 ms, hip still
150 ms) at 1.00 s rails the hip pitch. Holding the clock height through
half of single support drops min up_z to 0.673 at 1.16 s. Leading the
knee as well as the hip, at 0.60 s, either rails the hip pitch or
leaves the sole near 1 cm. Kit cadence, 300–600 ms, still fails.
Soft-pass is off.

Those clips were replaced by the kit-preset row below.

## Kit speed presets, with a 1.5 cm crouch

`gait_manager.py` move(1..4) is the clock. Plant md5 stays
`17dc4ff37491c8e61900fd83b5d31f0c`. Soft-pass is off.

| Move | Period | dsp | y_swap | x amp | z |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 300 ms | 0.20 | 0.02 m | 0.02 m | 0.02 m |
| 2 | 400 ms | 0.20 | 0.02 m | 0.02 m | 0.02 m |
| 3 | 500 ms | 0.20 | 0.02 m | 0.02 m | 0.02 m |
| 4 | 600 ms | 0.10 | 0.04 m | 0.02 m | 0.02 m |

The body is lowered 0.015 m from the full stand by +0.34 rad of knee,
sole level, hip left at the stand angle. That is the measured drop on
this nearly straight leg (stand knee 0.40 rad). +0.36 rad falls over.
It is not the kit's cartesian body-z offset. z_swap stays the published
0.006 m. Changing it to 0.020 m does not move these numbers.

Three knobs do not map 1:1. Clock y is a hip-roll lean of `dy / 0.22`
rad, not a sideways foot step. 0.02 m of y_swap is 0.091 rad and fits
in the 0.15 rad basin. 0.04 m wants 0.182 rad, so the 600 ms gear
raises that clamp. The published wSin at x = 0.02 m still travels about
4.0 cm fore-aft; the amplitude cap is 0.02 m, and the swing-hip gain
on this row is 1.0 so that clock is not enlarged. The 150 ms move, the
1.05 rad knee command, the 0.070 rad stance clip, and pelvis 0 stay.
The kit's 5 deg pelvis offset tipped this plant before and is not added.
The 0.22 s hip lead is not in the preset and is off.

8.4 s, gain 1.0, lead 0, crouch on:

| Period | Mean vx | min up_z | Sole p90 L / R | Hip pitch R / L | Hip roll R / L | Rail while up_z is 1 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 300 ms | +0.84 cm/s | 0.992 | 0.83 / 0.69 cm | 1.90 / 1.89 Nm | 1.04 / 1.74 Nm | none on the hips; left knee later |
| 400 ms | +7.24 cm/s | 0.744 | 1.83 / 0.13 cm | 2.45 / 2.45 Nm | 2.45 / 2.45 Nm | right hip pitch at 0.692 s, ω +0.39, kp·e −1.78, damp −0.70, F −2.45, then the body tips |
| 500 ms | +8.05 cm/s | 0.765 | 2.23 / 1.10 cm | 2.45 / 2.45 Nm | 2.45 / 2.45 Nm | right hip pitch at 0.760 s, ω +0.31, kp·e −1.95, damp −0.56, F −2.45, then the body tips |
| 600 ms | +10.42 cm/s | 0.668 | 2.97 / −0.19 cm | 2.45 / 2.45 Nm | 1.89 / 2.45 Nm | right hip pitch at 0.806 s, ω +0.35, kp·e −1.86, damp −0.63, F −2.45, then the body tips |

The 300 ms row is the one that stays up. At 24 s, min up_z 0.992,
sat_rate 0.001, no hip-pitch or hip-roll sample at ±2.45 Nm:

| | |
| --- | ---: |
| Period | 300 ms |
| Sole p90 L / R | 0.88 / 0.63 cm |
| Mean vx | +1.29 cm/s |
| Knee achieved L / R | 0.66 / 0.66 rad |
| Swing hip command / achieved | 0.466 / 0.027 rad |
| Airborne foot Δx | −0.82 cm |
| Body Δx during the swing | +0.15 cm |
| Hip pitch peak R / L | 1.90 / 1.89 Nm |
| Hip roll peak R / L | 1.04 / 1.74 Nm |
| Knee peak R / L | 2.26 / 2.45 Nm |

The left knee touches ±2.45 Nm at 0.800 s while `up_z` is 1 (ω +0.37,
kp·e +1.59, damp −0.54, F +2.45). The swing is 120 ms and the move is
150 ms, so the hip is asked for 0.47 rad and delivers 0.03 rad. The
foot's airborne travel is backward. Mean vx is +1.29 cm/s. Sole p90
stays under 1 cm. Raising the knee command to the ctrlrange limit,
1.35 rad, still leaves the sole near 1 cm and rails the right hip pitch
at 1.260 s. Soft-pass is off. This is not a kit walk.

The clips are the 300 ms row. Forward 8.400 s stays up (Δx +6.9 cm,
mean vx +0.84 cm/s, pooled sole p90 0.7 cm, min up_z 0.992, peak
torque 2.45 Nm, sat_rate 0.003). Close-up 7.600 s stays up through
the stop (forward Δx +4.8 cm, mean vx +0.84 cm/s, stop Δx −2.5 cm,
pooled sole p90 0.7 cm, min up_z 0.991, peak torque 2.41 Nm, sat_rate
0.005). The forward peak is the left knee on ±2.45 Nm. Constrained
Baseline, yuv420p, `+faststart`.

That joint-space row is not a kit gait test. The crouch was a knee
angle, the sway was a hip-roll lean, and the fore-aft travel was about
4 cm. The right-only hip-pitch rail on the mirrored legs was a missing
weight shift, not a plant conclusion.

## OP3 IK, AiNex lengths, kit move(1..4)

Hiwonder's `kinematics.so` / `walking_module.so` are aarch64 Cython
extensions for Python 3.8. They are not loaded. The walker is the
Apache-2.0 ROBOTIS OP3 `WalkingModule` plus
`calcInverseKinematicsForLeg` (`ROBOTIS-GIT/ROBOTIS-OP3`), with this
plant's link lengths in place of the OP3's 110 / 110 / 30.5 mm chain.
`scripts/op3_walk.py` reads them from the plant: thigh 9.69 cm, calf
8.91 cm, sole drop 2.60 cm. Joint-axis signs match the plant
(`getJointDirection` is the sum of the axis). Balance is off.
`hit_pitch_offset_` stays 0. Hiwonder's 15 deg is their offset from a
different init pose, and the IK already places the feet.

`init_z_offset` is 0.015 m. On this plant that shortens hip-yaw to sole
by 1.48 cm, and the soles stay level (`n_z` = 1). It is not a knee-angle
add. The stand the IK returns is knee ±0.99 rad and hip pitch ±0.48 rad.
The spawn seats those soles on the floor. `COM_Z` 0.225 m left them
9 mm in the air.

The clock is the OP3 `wSin` with the Hiwonder move(1..4) parameters:
x amplitude 0.02 m, step height 0.02 m, dsp and y_swap as in the table
above, z_swap 0.006 m, `step_fb_ratio` 0.028, pelvis offset 5 deg.
Kinematically the swing-stance height gap is 2.00 cm and the y swap
peaks at 2.00 cm (4 cm is not what this row commands; 600 ms y_swap is
0.04 m, so that peak is 4 cm). The foot x travel on the same formulas is
4.07 cm. The x parameter is 0.02 m. The wSin endpoint is not.

8.4 s, soft-pass off, plant md5 `17dc4ff37491c8e61900fd83b5d31f0c`,
legs still ±2.45 Nm. No XML, forcerange, dampratio, or armature edit.

| Period | Mean vx | min up_z | Sole p90 L / R | Hip pitch R / L | Hip roll R / L | sat_rate | First rail, up_z still ~1 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 300 ms | +0.68 cm/s | 0.985 | −0.04 / 0.65 cm | 2.23 / 1.99 Nm | 2.45 / 2.45 Nm | 0.171 | right knee at 0.420 s, ω +0.13, kp·e +2.67, damp −0.19, F +2.45 |
| 400 ms | +0.65 cm/s | 0.987 | 0.03 / 0.41 cm | 2.45 / 2.28 Nm | 2.45 / 2.45 Nm | 0.197 | right knee at 0.420 s, ω +0.02, kp·e +2.59, damp −0.02, F +2.45 |
| 500 ms | +0.92 cm/s | 0.985 | 0.03 / 0.41 cm | 2.45 / 2.45 Nm | 2.45 / 2.45 Nm | 0.332 | right knee at 0.420 s, ω −0.03, kp·e +2.55, damp +0.05, F +2.45 |
| 600 ms | +0.40 cm/s | 0.977 | −0.17 / 0.40 cm | 2.45 / 2.45 Nm | 2.45 / 2.45 Nm | 0.415 | right knee at 0.500 s, ω +1.29, kp·e +4.15, damp −1.87, F +2.45 |

The first hip rail is the right hip roll, after the knee, still upright:

| Period | Right hip roll |
| --- | --- |
| 300 ms | 0.460 s, ω −1.11, kp·e +2.02, damp +1.88, F +2.45, up_z 0.99. Left hip roll at 0.520 s. Hip pitch stays under the rail (2.23 / 1.99 Nm). |
| 400 ms | 0.460 s, ω −0.91, kp·e +1.06, damp +1.55, F +2.45, up_z 1. Right hip pitch follows at 1.020 s. |
| 500 ms | 0.480 s, ω −0.84, kp·e +1.62, damp +1.43, F +2.45, up_z 0.99. Both hip pitches follow near 1.15 s. |
| 600 ms | 0.520 s, ω −1.14, kp·e +1.18, damp +1.94, F +2.45, up_z 0.99. Both hip pitches follow near 1.27 s. |

Weight shift does not happen before the first swing. The IK y target
moves. The body does not. At the first swing (sole above 5 mm or normal
under 5 N) the COM is 0.17, 0.01, 0.09, and 0.57 cm off the mid-foot
line, against a 2 cm y_swap (4 cm at 600 ms). On 300, 400, and 600 ms
there is no tick after the walk starts, and before that swing, where
both feet are still above 5 N. The 500 ms row does reach an 80/20
normal split while both feet are above 5 N, with the COM 0.12 cm off
center. That is not the 2 cm shift.

All four rows stay upright and do not walk. Mean vx stays under 1 cm/s
and both soles stay under 1 cm. The rails are inside ±2.45 Nm with the
IK gait actually running. This is the Hardware armature / inertia
comparison. No plant edit from this row.

The clips are the 400 ms row, the highest min up_z. It still rails.
Forward 8.400 s stays up (Δx +4.9 cm, mean vx +0.6 cm/s, pooled sole
p90 0.3 cm, min up_z 0.987, peak torque 2.45 Nm). Close-up 7.600 s
stays up through the stop (forward Δx +1.9 cm, mean vx +0.4 cm/s,
stop Δx +1.1 cm, min up_z 0.987 on the forward window and 0.998 after
the stop, peak 2.45 Nm while walking and 1.90 Nm after the stop).
The picture is an upright shuffle: the feet barely leave the floor and
the body drifts a few centimetres. Constrained Baseline, yuv420p,
`+faststart`.

## Standing load, 8 ms cycle, hip-roll feed-forward

Three checks before another gait retune. Plant md5
`17dc4ff37491c8e61900fd83b5d31f0c`. Mass 2.3475 kg, weight 23.03 N,
11.51 N on each foot if the stand is even. Soft-pass is off. No kp,
forcerange, dampratio, armature, or XML edit.

The pose is the OP3 initial pose with every walking amplitude at zero
(x, y, z move, y_swap, z_swap, turn). The cartesian crouch stays
`init_z_offset` 0.015 m. Hold is 2.0 s.

At `init_y_offset` 0 the soles stack. Hip yaw is at ±2.90 cm and the
sole half-width is 3.80 cm, so each patch crosses the midline. At 2.0 s
the floor normal is 23.20 N on the left and 0.00 N on the right.
up_z is 0.992. Knee torque is +1.25 / +2.09 Nm, hip roll +2.42 / +1.46 Nm,
ankle roll −2.15 / −2.40 Nm. Foot centers are +2.88 / −3.63 cm and the
COM is +0.97 cm. That is a Prefer FAIL of the hip-width offset.

The controller offset is `2 * (sole_half_y − hip_yaw_y) = 0.018 m`.
OP3 adds ±y_offset/2 in the hip frame, so each sole center lands on
±3.80 cm and the inboard edges meet on the midline. At 2.0 s the
normals are 11.51 / 11.52 N. Knee torque is −0.31 / +0.31 Nm (joint
±0.814 rad). Hip roll is +0.05 / −0.05 Nm. Ankle roll is +0.03 / −0.03 Nm.
up_z is 1.000. COM y is 0. The standing crouch is far from ±2.45 Nm,
so the body stays at the 0.015 m drop.

The plant timestep is 0.002 s. The OP3 `WalkingModule` cycle is 8 ms.
The other command bus in this repo is 20 ms, and 20 is not an integer
multiple of 8, so a 20 ms tick cannot be labeled as that cycle. The
gait-manager path now steps the walker and the position targets every
0.008 s (4 physics steps). The Bézier loop stays at 0.020 s. The 150 ms
hip/knee/ankle move is still a fraction of wall time, `dt / 0.150`.

Hip-roll kp stays 40. A static hold at the y_swap pose (command 0.020 m,
sampled lateral offset +0.015 m, both feet down) has hip-roll torque
+0.08 / +0.03 Nm and a lag of −0.002 / −0.001 rad. COM is 1.26 cm off
the mid-foot line. Adding the expected 1.29 Nm as `1.29/40 = 0.03225 rad`
on the hip-roll target does not recover ~0.7 cm:

| Offset | Hip-roll force | Joint lag (achieved − command) | COM off mid-foot |
| --- | --- | --- | --- |
| 0 | +0.08 / +0.03 Nm | −0.002 / −0.001 rad | 1.26 cm |
| 0.032 rad with the sway | −0.00 / −0.17 Nm | +0.000 / +0.004 rad | 1.38 cm |
| 0.032 rad opposing, one sign per hip | +1.25 / −1.16 Nm | −0.031 / +0.029 rad | 1.27 cm |

The opposing offset is the 1.29 Nm case. The joint sags back by F/kp
and the COM stays on 1.27 cm. The sway-direction offset moves the COM
0.12 cm. The feed-forward is not left in the controller. kp stays a
plant question only after this measurement.

## Weight shift before the first swing

Hardware and MFG locked the plant. Armature is not a fix: no armature
peel, the HX-35H rotor and gear ratio are unpublished, and an OP3
Menagerie armature of 0.045 would make the 300 ms knee worse. The
Prefer FAIL step-time floor is **400 ms and slower**. 300 ms stays in
the published table and is not the success bar.

The OP3 sine `A sin(2π t / T)` peaks at T/4, inside single support.
At `l_ssp_start` (dsp·T/4) it has only reached `sin(π·dsp/2)` of the
amplitude (0.62 cm on a 2 cm swap). The foot-lift z is still the OP3
sample. The lateral channel is warped so the peak is at each
single-support start, then falls through the swing and is 0 at the
half-period. x and z are unchanged. The stand pose is still the
zero-amplitude pose above.

8.4 s, walk starts at 0.40 s, soft-pass off. "First swing" here is the
first tick after the walk starts with a sole above 5 mm or a normal
under 5 N. On these rows that tick is an unload with both soles still
on the floor. A both-feet-above-5 N tick does happen first.

| Period | Mean vx | Δx | min up_z | Sole p90 L / R | Max sole | Hip roll R / L | Knee R / L | First unload |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 400 ms | +8.06 cm/s | +64.2 cm | 0.995 | 0.28 / 0.22 cm | 0.62 cm | 2.44 / 2.08 Nm | 1.28 / 1.18 Nm | 0.496 s, Fn 2.16 / 25.03 N, COM 0.91 cm off mid-foot, 3.01 cm from the right sole center |
| 500 ms | +7.76 cm/s | +61.9 cm | 0.991 | 0.74 / 0.71 cm | 1.23 cm | 2.28 / 2.26 Nm | 1.10 / 1.41 Nm | 0.496 s, Fn 4.63 / 23.07 N, COM 0.91 cm off mid-foot, 2.95 cm from the right sole center |
| 600 ms | +3.36 cm/s | +25.6 cm | 0.936 | 1.01 / 1.52 cm | 2.04 cm | 1.96 / 1.88 Nm | 1.64 / 1.54 Nm | 0.496 s, Fn 4.82 / 17.67 N, COM 0.99 cm off mid-foot, 2.82 cm from the right sole center |

400 ms peaks at 2.44 Nm and does not cross 2.449 Nm. 500 and 600 ms
stay under the hip-roll rail. sat_rate on the swing samples is 0.001,
0.000, 0.000. Stance slip on the 400 and 500 ms rows is 0.61 and
0.59 cm/s.

The body does advance. On 400 and 500 ms the sole stays under 2 cm
(max 0.62 and 1.23 cm). The 600 ms row does touch 2.04 cm, with
min up_z 0.936. The COM at the first unload is inside the stance sole
(half-width 3.80 cm) and about 3 cm from that sole's center, after a
double-support tick with both feet above 5 N. Knee torque on that tick
is under 1 Nm. This is still a low shuffle on ±2.45 Nm, not a kit walk.

300 ms is below the floor. It is upright (min up_z 0.998, Δx +55.2 cm,
mean vx +6.93 cm/s) and the soles stay at 0.40 cm. It rails the right
hip roll at 0.472 s while up_z is 1: ω −0.09, kp·e +2.28, damp +0.15,
F +2.45. That rail is not a reason to edit armature.

The clips are the 400 ms row. Constrained Baseline, yuv420p,
`+faststart`. Forward 8.400 s stays up (Δx +63.1 cm from 0.50 s,
mean body vx +8.1 cm/s, min up_z 0.995, peak torque 2.44 Nm, yaw drift
+7.4 deg). Close-up 7.600 s stays up through the stop (forward Δx
+42.5 cm, mean vx +7.8 cm/s, stop Δx +0.5 cm, min up_z 0.995 while
walking and 0.999 after the stop, peak 2.44 Nm while walking and
0.98 Nm after the stop). The feet stay close to the floor.

## Foot-height half, target sole vs actual

The port does the OP3 half. `updateMovementParam` sets
`z_move_amplitude_ = walking_param_.z_move_amplitude / 2`, and
`loadWalkingParam` stores yaml `foot_height` in `z_move_amplitude`.
`update_movement` does the same: `_z_move = z_move_cmd / 2`,
`_z_move_shift = _z_move / 2`. With gait_manager `foot_height` 0.020 m
the internal amplitude is 0.010 m and the shift is 0.005 m.

The commanded sole is the swing-to-stance gap, which is the full
yaml value. `wSin` is
`amp·sin + shift`, so z runs from −0.005 m to +0.015 m. The stance
foot stays at the negative peak while the swing foot reaches the
positive peak, and the gap is 0.020 m. Posing those IK joints with
`mj_forward` (no dynamics) gives a swing-minus-stance sole of 2.25 cm.
Feeding 0.040 m instead makes the endpoint gap 4.00 cm and the
kinematic sole 4.23 cm. The yaml value stays 0.020 m. The half stays.

On the live 8.4 s walk the IK target, written into `ctrl` before the
150 ms slew, is still about 2 cm. The actual sole stays under that.

| Period | Endpoint p90 | IK sole p90 / max | Actual sole p90 L / R | Actual max | Hip roll R / L | Knee R / L |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 400 ms | 1.95 cm | 1.88 / 2.28 cm | 0.28 / 0.22 cm | 0.62 cm | 2.45 / 2.08 Nm | 1.28 / 1.18 Nm |
| 500 ms | 1.97 cm | 2.01 / 2.36 cm | 0.74 / 0.71 cm | 1.23 cm | 2.28 / 2.27 Nm | 1.10 / 1.42 Nm |

Torque is the peak `|actuator_force|` over every 2 ms physics step.
Knee is reported on the same row as hip roll. 500 ms is the row with
hip-roll headroom (2.28 / 2.27 Nm against ±2.45). 400 ms reaches
2.450 Nm on the right hip roll, so this pass leaves that row's lift
alone.

The knee peaks land with the sole on the floor, beside the hip-roll
peaks. At 500 ms the left knee reaches
−1.42 Nm at 0.772 s on the same tick as the left hip roll (−2.27 Nm),
with both soles on the floor (−0.15 / −0.04 cm). The right knee reaches
+1.10 Nm at 5.666 s with the right sole at −0.24 cm and the right hip
roll at +1.58 Nm. The highest actual sole, 1.23 cm at 0.888 s, has
knee +0.08 / −0.20 Nm and hip roll −0.04 / −0.37 Nm.

At the tick where the posed IK sole is 2.36 cm (endpoint gap 2.00 cm),
the actual swing sole is 0.20 cm. The 150 ms slew has moved the knee
0.060 / 0.034 rad of an IK error of 0.256 / 0.218 rad. Knee force on
that tick is −1.12 / −0.38 Nm, inside ±2.45, and the swing sole stays
at 0.20 cm. Hip roll on that tick is −1.37 / +0.36 Nm. The 2.28 Nm
hip-roll peaks are the floor-contact samples above.

Mean vx on these two rows is still +8.06 and +7.76 cm/s, min up_z
0.995 and 0.991. First unload is still 0.504 s, after both feet have
been above 5 N. Plant md5 is `17dc4ff37491c8e61900fd83b5d31f0c`.

This is a tracking Prefer FAIL. The command is already the kit 2 cm
sole. The 1.42 Nm knee peak is the hip-roll load with the sole on the
floor, so a linear scale from the 1.23 cm sole does not turn that peak
into the 2 cm knee risk. Knee torque at the missed 2 cm command is
about 1.1 Nm. The extra inertia of a sole that actually reached 2 cm
was not measured. Foot height stays 0.020 m. Plant md5 stays
`17dc4ff37491c8e61900fd83b5d31f0c`.

## Kit offsets, then a 20 ms servo move at 500 ms

Kit `walking_param.yaml` plans at `trajectory_step_s` 0.008 s and
writes servos at `servo_control_cycle` 0.02 s. The 150 ms approach on
top of the 8 ms planner was a second smoother. The OP3 path now
approaches hip pitch, knee, and ankle pitch over `gm_move_s`. The
shipped value is 0.020 s. The Bézier loop still uses 150 ms. The HX
command rate stays 5.5 rad/s. Foot-height half stays. Plant md5 stays
`17dc4ff37491c8e61900fd83b5d31f0c`.

Kit `init_z_offset` 0.025 m with the 0.018 m stance width, held 2.0 s:
normals 11.51 / 11.52 N, knee ±0.39 Nm, hip roll ±0.05 Nm, up_z 1.000.
That crouch is what the 500 ms rows use.

Kit `init_y_offset` −0.005 m does not share the load. At the same
0.025 m drop and a 2.0 s hold the normals are 23.03 / 0.00 N, the right
sole is off the floor, up_z 0.997, and the right knee is +2.45 Nm.
The 0.018 m width stays. It is the sole-versus-hip gap on this plant,
and the kit lateral offset is a stand Prefer FAIL here.

Kit `hip_pitch_offset` 15°, on top of this IK, pitches the body. With
the even stance and the 0.025 m drop the normals stay 11.51 / 11.52 N
and up_z is 0.966, which is cos(15°). That row left the offset at 0.
Hardware #49's stand bar is hip pitch about 0.77 rad and knee about
−1.05 rad, so the offset is on the stand in the section below.
`pelvis_offset` stays 5°. `arm_swing_gain` is the yaml 0.5.

500 ms period, dsp 0.2, y_swap 0.020 m, z 0.020 m, walk from 0.40 s
for 8.4 s. Torque is the peak `|actuator_force|` on every 2 ms step.
Knee is on the same row as hip roll.

| Servo move | Mean vx | Δx | min up_z | Sole p90 L / R | Max sole | IK sole p90 / max | Hip roll R / L | Knee R / L |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 20 ms | +14.84 cm/s | +120.5 cm | 0.992 | 1.52 / 1.54 cm | 1.97 cm | 2.08 / 2.35 cm | 2.15 / 2.16 Nm | 2.22 / 2.35 Nm |
| 8 ms | +15.20 cm/s | +123.6 cm | 0.992 | 1.58 / 1.59 cm | 1.98 cm | 2.12 / 2.34 cm | 2.08 / 2.18 Nm | 2.45 / 2.45 Nm |
| 16 ms | +14.97 cm/s | +121.6 cm | 0.992 | 1.55 / 1.55 cm | 1.99 cm | 2.09 / 2.34 cm | 2.13 / 2.16 Nm | 2.31 / 2.45 Nm |

Endpoint p90 is 1.97 cm on all three. At the 20 ms tick where the posed
IK sole is 2.35 cm (endpoint gap 2.00 cm), the actual swing sole is
1.10 cm and the knee error is 0.088 / 0.084 rad (it was ~0.25 rad under
the 150 ms move). The highest actual sole on that row, 1.97 cm at
0.848 s, has knee +0.86 / +0.31 Nm and hip roll −0.52 / +0.54 Nm.

20 ms knee peaks are not the hip-roll peaks. Right hip roll +2.15 Nm at
3.080 s with both soles down and knee +0.40 / +0.25 Nm. Left hip roll
−2.16 Nm at 2.312 s, soles down, knee −0.24 / −0.39 Nm. Right knee
+2.22 Nm at 0.686 s, soles down. Left knee −2.35 Nm at 0.942 s with the
right sole at 0.31 cm. None of those cross 2.449 Nm. Hip pitch peaks
2.01 / 1.74 Nm. sat_rate on swing samples is 0.

8 ms rails both knees at 2.45 Nm and the right hip pitch at 2.45 Nm.
16 ms rails the left knee at 2.45 Nm (0.938 s, right sole 0.29 cm).
Those two rows are the torque answer for a faster write. They are not
the shipped move time. 400 ms was not re-run. The sole amplitude was
not raised.

MFG and Hardware locked this sole. The kit swing runs from −0.5 cm to
+1.5 cm, and the floor clip leaves about 1.5 cm of clearance. The
measured p90 of 1.52 / 1.54 cm already matches. Max sole 1.97 cm is
the peak of that swing, not a shortfall. Knees on this row sit at
2.22 / 2.35 Nm, so the headroom to ±2.45 is 0.23 / 0.10 Nm. The lift
was not raised. Mean vx stays +14.84 cm/s and min up_z 0.992. Body
speed is the measured speed, not the 5.6 cm/s stick command.

## Stop, without a hip-roll clip

The first close-up stop wrote the stand pose in one tick. Hip roll and
ankle roll are not on the 20 ms move, so `write_clipped` stepped them
by the full 0.98·τ/kp band. At 6.000 s the right hip was at +0.145 rad
with ω +0.95 rad/s, and the stand target is +0.056 rad. That step was
kp·e = −2.40 Nm plus damping −1.62 Nm, and the actuator clipped at
−2.450 Nm on the next sample (6.008 s, up_z 0.999).

The stand write for every joint that is not knee, hip pitch, or ankle
pitch now keeps the predicted force kp·(ctrl−q) − kv·ω inside
0.98·±2.45 Nm. Knee, hip pitch, and ankle pitch still approach the
stand goal over the 20 ms move. The walk tick is unchanged.

Same 500 ms / 20 ms row. Physics-step peaks, knee beside hip roll:

| Window | Hip roll R / L | Knee R / L | Ankle roll R / L | min up_z |
| --- | ---: | ---: | ---: | ---: |
| Walk, 8.4 s | 2.15 / 2.16 Nm | 2.22 / 2.35 Nm | 1.66 / 1.67 Nm | 0.992 |
| Stop from 6.00 s | 1.85 / 1.98 Nm | 2.38 / 1.21 Nm | 1.81 / 1.62 Nm | 0.999 |

The walk peaks are the same samples as before (right hip +2.15 Nm at
3.080 s, left knee −2.35 Nm at 0.942 s). Sole p90 stays 1.52 / 1.54 cm,
mean vx +14.84 cm/s. Nothing on the stop crosses 2.449 Nm. The right
hip-roll stop peak is −1.85 Nm at 6.008 s. The right knee stop peak is
+2.38 Nm at 6.018 s. Plant md5 stays
`17dc4ff37491c8e61900fd83b5d31f0c`.

Clips are this row. Constrained Baseline, yuv420p, `+faststart`.
Forward 8.400 s stays up (Δx +118.5 cm from 0.50 s, mean body vx
+14.8 cm/s, min up_z 0.992, peak torque 2.32 Nm on the control
samples, yaw drift −1.8 deg). Close-up 7.600 s stays up through the
stop (forward Δx +81.2 cm, mean vx +14.5 cm/s, stop Δx +1.1 cm,
min up_z 0.992 while walking and 0.999 after the stop, stop peak
2.36 Nm). The feet leave the floor. The robot stays standing after
the stop.

## Kit stance on the #43 foot boxes

The plant file is #43 tip `921f5941`. Foot contact local y is +0.014 m
on the left and −0.014 m on the right. Size, friction, torque, kp,
dampratio, and kit_cam are the #43 file. This tree does not edit it.
md5 is `207f3d5e9c6a72e16f7aa0c8d224f75e`.

`init_z_offset` is 0.025 m. The 0.018 m sole-vs-hip stance is not the
live offset. Kit `init_y` is outward-positive: +0.005 m on each foot.
OP3 applies ±`y_offset`/2, so the stored offset is 0.010 m. At a posed
stand the sole centres sit at −0.0477 / +0.0477 m. Against zero offset
(−0.0430 / +0.0430 m) that is 4.7 mm outward on each foot. The inner-edge
gap is 19.4 mm. The feet do not overlap.

Stand targets at that pose: right hip pitch +0.768 rad, right knee
−1.052 rad. Left hip pitch −0.768 rad and left knee +1.052 rad, which
is the same crouch on the opposite joint axes. Hardware #49's bar is
hip pitch about 0.77 rad and knee about −1.05 rad. A 2.0 s hold
settles at hip pitch +0.770 / −0.770 rad and knee −1.059 / +1.059 rad.
Normals 11.51 / 11.52 N. Settled up_z is 0.966. Knee torque on the
hold peaks at ±0.84 Nm.

`pelvis_offset` 5° is the single-support hip-roll correction.
`z_swap_amplitude` 0.006 m is the body bob. `arm_swing_gain` 0.5
commands ±10° of shoulder at x = 0.02 m. The gait-manager tick writes
that command. After 1.5 s the shoulder joints move 6.0 / 5.9° peak to
peak. The arm actuators stay at ±0.7 Nm, so the joint does not reach
the command. `hip_pitch_offset` 15° is on the stand and the walk.
Lateral y_swap stays 0.020 m.

500 ms period, 20 ms servo move. Foot height stays 0.020 m. Torque is
the peak `|actuator_force|` on every 2 ms step. The 8.4 s walk starts
at 0.40 s. The close-up starts at 0.50 s and stops at 6.00 s.

| Window | Mean vx | Δx | min up_z | Sole p90 L / R | Body z ptp | Hip roll R / L | Knee R / L |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Walk 8.4 s | +14.18 cm/s | +118.8 cm | 0.932 | 1.75 / 1.75 cm | 11.7 mm | 2.13 / 2.16 Nm | 2.03 / 1.97 Nm |
| Stop from 6.00 s | +13.8 cm/s while walking | walk +80.1 cm, stop +1.0 cm | 0.952 after the stop | 1.75 / 1.74 cm | 11.0 mm | 1.79 / 1.86 Nm | 2.27 / 1.24 Nm |

Nothing on either window crosses 2.449 Nm. The close-up right knee
peaks at +2.423 Nm at 0.788 s. Forward yaw drift is −5.8 deg. Close-up
yaw at the stop is −4.0 deg and +1.3 deg during the stand after the
stop. Loaded-foot skate on the 8.4 s row, normal above 5 N, is
1.72 / 1.75 cm/s mean. That skate is not peeled.

Clips are this row. Constrained Baseline, yuv420p, `+faststart`.
Forward 8.400 s and close-up 7.600 s. Settled up_z after the stop is
0.966.

## Skate check, no peel

Same kit row. Plant md5 stays `207f3d5e9c6a72e16f7aa0c8d224f75e`. No
friction, forcerange, or kp edit. x_move stays 0.020 m. The 20 ms
servo move stays.

`2 · x_move / period` is 8.00 cm/s. While a foot is above 5 N, its
hip-frame x retreat averages 9.44 / 9.68 cm/s. Steady body vx after
1.5 s is +14.94 cm/s. The body is faster than that retreat by 3.3 to
5.5 cm/s, not slower by 1.7. Along-track speed of the loaded box is
0.19 / 0.18 cm/s. The 1.67 / 1.62 cm/s loaded-foot speed is lateral,
1.38 / −1.32 cm/s. That is not a stride deficit, so x_move is not trimmed.

Contact-point tangential speed, from the COM velocity plus the
cross product out to the contact, is 1.27 cm/s on both feet. The box
has at most four corners. Of the loaded samples, 287 / 442 and
308 / 475 have one or two contacts. The mean count is 2.52. Ankle
roll over a loaded bout is 7.1 / 7.9° peak to peak, and the foot
angular rate averages 0.79 / 0.78 rad/s. The loaded foot is rocking
on an edge. It is not a flat slide at μ = 1.6.

min up_z 0.932 is at 0.480 s, 80 ms after the walk command. Pitch
there is 21.17° and roll is −1.58°. acos(0.932) is 21.22°, so the
extra beyond the 15° lean is pitch. After 1.5 s the mean pitch is
15.18°. Roll runs from −5.5° to +6.7° and is not what makes up_z
0.932. The steady minimum, 0.948 at 1.792 s, is pitch 18.11° and
roll 4.02°.

Hip-pitch `|ctrl − q|` on a loaded foot has p90 5.62 / 5.64°. The
signed mean is +2.23 / −2.25°, which is the same lag on opposite
joint axes. 5.6° at the 5.5 rad/s command cap is about 18 ms, the
locked 20 ms write. Mean body pitch is already the 15° trim. A faster
hip-pitch approach is the 8 ms and 16 ms rows that railed the knees.
It is not applied.

No peel. Sole p90, vx, and the rail numbers above are unchanged.

## Day-1 CommandBus on the locked kit row

Plant md5 stays `207f3d5e9c6a72e16f7aa0c8d224f75e`. Soft-pass is off.
The kit numbers are unchanged: period 500 ms, servo move 20 ms, stance
+0.005 m outward on each foot, init_z 0.025 m, arm swing gain 0.5,
pelvis 5°, hip pitch 15° on stand and walk, z_swap 0.006 m, y_swap
0.020 m, step x and z 0.020 m.

`CommandBus` is what stand, stop, and vel go through. `vel` takes
`vx` in m/s and `yaw_rate` in rad/s. A `vy` argument is refused and
does not change the targets. A later `vel` replaces the earlier one
on that same call. Controls clamps live on the bus: forward 0.056 m/s,
back 0.032 m/s, yaw ±0.25 rad/s. `vel(3.0, -4.0)` becomes
+0.056 m/s and −0.25 rad/s. Those three limits are the Controls clamps.

The AI resend is 100 ms. On the kit clock (8 ms) a gap of 200.0 ms
with no command returns `mode=stand` and `applied_vx=0`. `stop` does
that on the same tick. Each tick returns `applied_vx`,
`applied_yaw_rate`, and `mode` in {stand, move, fault}. This run did
not enter fault.

`applied_yaw_rate` is the clamped, slewed yaw command. One tick of a
full left command returned `applied_vx=+0.0006`,
`applied_yaw_rate=+0.0032`, `mode=move`. The kit step angle stayed
0 rad, so that command is reported and the feet stay on the forward
pattern. Body yaw in the table is drift.

The Prefer FAIL is a cold start: stand until 0.50 s, `vel(+0.056, 0)`
resent at 10 Hz until 6.00 s, then `stop` until 7.60 s. Torque is the
signed peak `|actuator_force|` on every 2 ms step. The forward command
stays in `mode=move`, so the 200 ms watchdog does not trip while the
resend is running. `applied_vx` reaches the +0.056 m/s clamp and does
not go past it. The kit step length stays 0.020 m for any non-zero vx.
Measured body speed on this window is +13.79 cm/s.

| Window | Δx | Mean body vx | min up_z | Δyaw | Hip roll R / L | Knee R / L | Worst |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Forward | +80.0 cm | +13.79 cm/s | 0.941 | −3.9 deg | +2.114 / −2.154 Nm | +2.423 / −1.797 Nm | right knee +2.423 Nm at 0.788 s |
| Stop | +1.0 cm | — | 0.953 | +1.3 deg | −1.840 / −1.940 Nm | +2.319 / +1.262 Nm | right knee +2.319 Nm at 6.018 s |

The stop tick is `applied_vx=+0.0000`, `applied_yaw_rate=+0.0000`,
`mode=stand`. Swing sole p90 on the forward window is 1.74 / 1.72 cm.
Nothing in stand, forward, or stop crosses 2.449 Nm. The stand peak
is the right ankle pitch at +1.262 Nm. The clip is
`previews/bus_kit_forward_stop.mp4`.

## Honest vx, sag bar, kit turn

Plant md5 stays `207f3d5e9c6a72e16f7aa0c8d224f75e`. Soft-pass is off.
The forward clamp is no longer 0.056 m/s. That command was a switch
into the full 0.020 m step, and the body then walked at about 0.138 m/s.

`applied_vx` is still the clamped command. The OP3 step length is
`applied_vx / 7.50`. At the 0.020 m step the settled body speed is
0.150 m/s, so the bus forward clamp is 0.150 m/s. Yaw stays ±0.25 rad/s.
A request of `vel(3.0, −4.0)` clamps to +0.150 m/s and −0.25 rad/s.

Once the slew has reached that clamp, settled body vx is +0.150 m/s.
The ratio is 1.00. Half of the clamp, 0.075 m/s, settles at +0.078 m/s.
That residual gain is 1.05. The window mean on the 0.50–6.00 s forward
row is +0.119 m/s because the slew up to 0.150 m/s takes 1.875 s.
Reverse stays clamped at 0.032 m/s. The forward gain of 7.50 made that
command a 4.3 mm step and the body retreated at 0.023 m/s (ratio 0.71).
Retreat uses 5.90 (m/s) per meter of step, so the same clamp is a 5.4 mm
step. Settled body vx is −0.031 m/s (ratio 0.98). Half of that clamp,
−0.016 m/s, retreats at −0.012 m/s (ratio 0.73).

Knees are reported against 2.33 Nm, the HX-35H budget at about 10.5 V.
The plant forcerange stays ±2.45. Stop and the 200 ms silence apply the
stand command through the force limit on that same tick. Every leg
joint on that hold — hip yaw, hip roll, hip pitch, knee, ankle pitch,
and ankle roll — uses a 2.28 Nm prediction, rewritten from the live
joint and its rate before each physics step. The measured stop peak
sits on 2.280 Nm, 0.05 under the 2.33 bar. A command written once per
tick left `l_hip_pitch` at 2.309 Nm on the straight stop at gait clock
0.322 s. They do not slew through the walking target.

| Window | Settled body vx | Knee peak | Other peak |
| --- | ---: | ---: | ---: |
| Forward, command +0.150 | +0.150 m/s | right +2.173 Nm | hip roll −2.128 Nm |
| Stop from that walk | — | stop legs 2.280 Nm | stop legs 2.280 Nm |

Nothing crosses 2.33 Nm on a knee. Nothing crosses 2.449 Nm on a leg.
Swing sole p90 on this forward row is 1.75 / 1.73 cm. min up_z is 0.941
while walking and 0.954 after the stop. Δx on the forward command is
+68.9 cm. The stop adds +1.0 cm. 200.0 ms of silence still returns
`mode=stand` and `applied_vx=0`.

Cycle yaw is `yaw_rate * period * 0.50`. Full stick is 0.062 rad of
step angle. The walker splits that across the two feet. +yaw_rate is left.

| Command | Step angle | Steady Δyaw | Steady yaw rate | Knee | Worst leg |
| --- | ---: | ---: | ---: | ---: | ---: |
| vel(+0.150, +0.25) | 0.062 rad | +49.6 deg | +0.217 rad/s | 2.330 Nm | hip pitch −2.401 Nm |
| vel(+0.150, −0.25) | 0.062 rad | −50.6 deg | −0.221 rad/s | 2.330 Nm | hip pitch −2.401 Nm |
| vel(0, +0.25) | 0.062 rad | +48.6 deg | +0.213 rad/s | 2.330 Nm | hip pitch −2.401 Nm |

Hip yaw, hip roll, and ankle roll share an axis on the two legs
(`0 0 -1`, `-1 0 0`, `1 0 0`). Only pitch is mirrored. The right-leg
yaw shift was negated twice, so both feet took the same step angle.
That common mode made a left command turn at 0.78× the right command.
One negation, the OP3 right-leg sign, puts opposite yaw on the two
feet. `pelvis_offset` stays +5°. Flipping that pair matches the yaw
rates and drops the straight walk from 0.150 m/s to 0.110 m/s, so the
5° sign stays.

From 4 s to 8 s, `vel(+0.150, +0.25)` yaws at +0.217 rad/s and
`vel(+0.150, −0.25)` yaws at −0.221 rad/s. The magnitudes differ by
2.0%. Straight walk over that same span is −0.003 rad/s. Knees sit on
2.330 Nm. The worst other leg sample is hip pitch at 2.401 Nm.
`vel(0, +0.25)` yaws at +0.213 rad/s. Those steady rates are from
before the hip-roll budget at the end of this note.

Nav-left resume heading (Prefer FAIL). Scenario: 1 s stand, 15 s
forward, 12.5 s `vel(+0.150, +0.25)`, then 6 s `vel(+0.150, 0)`.
Resume Δyaw ≈ +12.41°. Plant md5
`207f3d5e9c6a72e16f7aa0c8d224f75e` unchanged. Split: (1) 0–0.68 s
`applied_yaw` still slewing +0.25→0 at 0.40 rad/s² after the 100 ms
resend clears the target → body +6.81° (command integral ~+5.22°);
(2) 0.68–6.0 s `applied_yaw` and the step angle are already 0 →
leftover left curve +5.60° (body yaw rate +0.034→+0.009 rad/s). That
is ~half ramp-out, ~half steady leftover curve under `vel(+0.150, 0)`
— not “turn still commanded.” Soft-pass is off.

Left versus right unload, same plant. The 6 s `vel(+0.150, 0)` after
the left turn is +12.3°: +6.7° while `applied_yaw` slews +0.25→0 in
0.69 s (command integral +5.3°), then +5.6° with the yaw command and
the step angle already 0. The 6 s `vel(+0.150, 0)` after an 11 s
`vel(+0.150, −0.25)` is −2.2° on the chained walk and −0.6° when that
right turn follows a straight approach. The slew is 0.40 rad/s² both
ways. On a matching step phase the right ramp moves the body −5.3°
(command integral −4.5°), and the rest of the window then curves left
+4.7°, so the ramp and the curve cancel. Straight `vel(+0.150, 0)`
with no turn already curves +4.2° over 27–33 s and +3.5° over
28.5–34.5 s. The left ramp and that curve add. Hip-yaw targets are 0
once the step angle is 0. `a_move` follows the yaw sign and the shift
stays +|a|. Steady rates are +0.217 and −0.233 rad/s. A pelvis-0
probe still reads +11.3° versus −0.4°. Stop snaps yaw to 0 in one
tick, and the heading kick follows the step phase: after a right turn
it runs about +11° to −10° across 0.37 s of phase.

Straight walk, five cold starts, same plant. Each trial is 1 s stand,
then `vel(+0.150, 0)` for 30 s, then stop. The five trajectories match
(std 0). Every one starts in double support at gait time 0, both feet
loaded (7.1 N / 7.1 N), and the first swing foot is the left foot.
Stand resets the step clock, and this cycle swings left first. From
the first move tick, Δyaw is −1.46° at 6 s (0/5 left), +1.39° at 15 s
(5/5), and +0.50° at 30 s (5/5). Mean body yaw rate is +0.0003 rad/s.
Two-second slices rock from about −2.9° to +1.9°. The +3° to +4°
straight windows above are that rock, not a steady left bias. An extra
0.25 s of stand leaves the lead foot on the left and Δyaw at 30 s
+0.51°. Starting the clock half a period later, probe only, swings the
right foot first and the 30 s net is −2.55°. A pelvis-0 probe still
nets +0.22° at 30 s with the left foot leading. Soft-pass is off.

Lead foot is a choice, not a permanent lock. Cold stand→forward stays
left-first: right-first over the same 30 s is −0.14° / −3.33° / −2.60°
at 6 / 15 / 30 s, and |Δ30| is larger (2.60° versus 0.50°). First-step
knees after stand, left-lead L 1.152 / R 1.749 Nm, right-lead L 1.749 /
R 1.152 Nm. Both are under 2.33 Nm. A yawed walk peaks at 2.294 Nm.
The stop peaks at 2.280 Nm. After a yaw target returns to 0, the next double support
swings the outside foot: right after +yaw, left after −yaw. Doing
that by parking the clock on the other double support in one tick
stepped the joint targets 0.342 rad (both shoulders). A normal
published walk tick is mean 0.023 rad, p95 0.047 rad, max 0.063 rad.
The pose now chases the live gait over that double support, 0.056 s,
7 ticks. The largest tick in the chase is 0.062 rad, the same size as
the walk's own largest tick.

Hip roll on a yawed walk crossed the 2.33 Nm bar with the plant cold.
Empty plant, 1.0 s stand, then 8 s `vel(+0.150, −0.25)`: `l_hip_roll_pos`
peaked −2.363 Nm at t=3.42 s. Knees were −2.060 and +1.966 Nm.
`r_hip_roll_pos` was +2.207 Nm at 4.70 s. Gait time 0.37 s is the right
single support, so the left leg is stance. Joint velocity was still
positive (ω +0.20 rad/s) while the servo torque was negative, so
damping added onto the spring. That peak is not an arm swing and not a
prop. While applied yaw is away from 0, the hip-roll command uses the
same 2.33 Nm prediction budget as a stop knee. Forcerange stays ±2.45 Nm.
Straight forward and reverse do not take that budget. After it, the same
window peaks at −2.271 Nm on the left hip roll (3.92 s) and +2.159 Nm on
the right. Knees are −2.083 / +2.043 Nm. min up_z is 0.934. Heading at
9 s is −101.4°. The left-turn mirror peaks at +2.278 Nm on the right hip
roll. `--bus-kit` steady rates are +0.235 and −0.236 rad/s (mismatch
0.6%). In-place is +0.220 rad/s. Straight-forward hip roll stays
−2.128 / +2.122 Nm.

The commanded turns on the nav windows are not the same length. Left is
12.5 s (16→28.5 s) at +0.25 rad/s and finishes at +156.4°, mean body
rate +0.218 rad/s. Right is 11.0 s (16→27.0 s) at −0.25 rad/s and
finishes at −148.6°, mean body rate −0.236 rad/s. The right arc is
7.8° shorter. The right rate is the higher of the two. The gap is the
1.5 s shorter right hold, not a weak right gain and not an outsole
peel. After the straight resume the leftover is +1.4° on the left and
−2.0° on the right. Forward knees peak at 2.173 Nm. A yawed walk peaks
at 2.294 Nm. The stop peaks at 2.280 Nm. First-step knees stay under
2.33 Nm. Soft-pass is off.

Kitchen, #62 group-3 `col_*` boxes, same 8 s right turn, colliders not
edited. With the hip-roll budget off, the left foot hits
`col_chair_stool_b_leg_2` at 23.9 N and 6.85 s, the right foot hits
`col_chair_stool_b_rail_yp` at 41 N and 9.08 s, and the COM leaves
support (tipping margin −0.889, min up_z −0.94). With the budget on,
the same script still clips that stool leg at 6.82 s and 20.4 N, and
hip pitch reaches ±2.45 Nm during the clip. It does not tip (min up_z
0.921). A Day-1 `stop` at 6.80 s, before that contact, finishes with
0 prop contacts, min up_z 0.934, xy +0.555, −0.517, yaw −74.4°, and
every stop-window leg joint at 2.280 Nm. A stop at 6.20 s is also
clear, with the same 2.280 Nm stop peak. A stop at 6.82 s is the
contact. Commanding yaw 0 at 5.0 s or 6.0 s still hits the stool and
tips. Reversing to `vel(+0.150, +0.25)` at 4.0 s misses the stool,
stays at min up_z 0.934, and stays under 2.33 Nm. The same reversal
at 5.0 s does not. Bath, bed, living, and entrance on this 8 s turn
match the empty plant (heading −101.4°, 0 prop contacts).
`col_entrance_sill` is a wall and was not touched. Not go-anywhere.

Stop distance. `applied_vx` is 0 and `mode` is stand on the next 8 ms
tick (`T_bus` = 0.008 s). The body is not stood then. `T_stop` is the
time from the stop command until the horizontal COM speed stays under
0.02 m/s for 0.20 s. Sampled on the empty plant after the walk is up,
for `vel(+0.150, 0)` and `vel(+0.150, ±0.25)`, across one gait period
of issue times, with every leg joint on the 2.28 Nm per-step hold.
`T_stop` runs from 0.538 s to 0.832 s. The long one is the straight
walk stopped at 4.40 s, gait clock 0.322 s. COM path on that settle
is 8.2 cm. Net COM displacement on the grid is at most 3.4 cm.
min up_z stays 0.930–0.934. Every leg joint on those stops measures
2.280 Nm. Nothing on the hold crosses 2.33 Nm. Settled
`|q − q_stand|` is 0.006 rad on the right knee and body z is 0.211.
The larger error at the stop instant, up to 0.231 rad, is the walk
pose being left. On that empty-plant grid, `v × 0.832` = 0.1248 m,
longer than the 0.082 m COM path, because the body is already slowing
down. 0.832 s replaces 0.830 s on the empty plant. The old 0.830 s is
not kept.

The kitchen stop is timed on `subtree_com` of `body_link`. The world
subtree includes the room and reads a false ~0.20 s. The live-height
toe-gap stop, issued at 6.200 s, settled in 0.842 s. The empty-plant
grid max stays 0.832 s. The worst measured settle is 0.842 s. That is
the `T_stop` in the formula below.

Clear distance, with `v` the commanded 0.150 m/s:

`d_min = v × (T_detect + T_stop)`

`T_stop` = 0.842 s. `T_detect` is the time from the first-visible frame
to an RGB-only detector firing on `kit_cam`, on the detector the kit
would run, including Pi and camera frame latency. It is a parameter.
It is not filled in here. A numeric `d_min` is not locked. 0.125 m and
0.129 m are not the margin.

`t_cue` is separate. On #63 the leg/rail band of the stool mesh is
`visible_enough` at t = 1.90 s, and the whole stool is `visible_enough`
at t = 1.00 s. That frustum uses sim ground-truth masks the real kit
will not have. First contact on that replay is t = 6.85 s, so the cue
is available for 6.85 − 1.90 = 4.95 s before contact. That 4.95 s is
cue availability, not `T_detect`.

#65 times an RGB wood-fraction rule at 0.342 ms from the `kit_cam`
buffer to `CommandBus.stop`. That compute is not kit `T_detect`.
Hardware still wants the Pi and the camera frame, about +33 ms at
30 fps, and that add is not a measured Pi latency either. The 28.8 ms
mesh read is sim ground truth only. None of those three numbers is
plugged into `d_min`.

Mono range. The body-side estimate uses the pixel row where a stool
leg meets the floor, the live `kit_cam` height and position at that
frame, and the camera world pitch. World pitch is IMU torso pitch
plus the `head_tilt` joint, not `head_tilt` alone. Height is the
camera's world z. The floor plane is z = 0, so the range is
horizontal from the point under the eye. The head pivot moves that
point, so 0.335 m is only the stand measurement. At the end of stand
(t = 1.00 s) the sum is −14.84°: IMU −14.82°, `head_tilt` −0.02°.
The optical axis is −14.84° and the eye is at 0.335 m. At t = 1.90 s
on this walk the sum is −15.58°: IMU −15.56°, `head_tilt` −0.02°.
The optical axis matches the sum. The −0.74° change from stand is
torso rock. `head_tilt` does not move. Using `head_tilt` alone
(−0.02°) on the cue row estimates 1.210 m. The horizontal gap from
the camera to that floor point is 0.767 m.

The row is the sim projection of the group-3 leg bottom. It is not
an RGB contact finder. The formula ignores the image column. At the
cue the contact sits at column 456, and the live estimate is 0.557 m
against 0.767 m. Stand pitch on the same row is 0.574 m. Most of that
0.21 m shortfall is the center-column model. The pitch choice is
1.7 cm.

Near the middle of the frame the pitch choice is the error. At
t = 5.46 s, `col_chair_stool_b_leg_2` is at row 381, column 312,
ground gap 0.227 m. Live pitch −18.29° estimates 0.229 m. Stand
pitch −14.84° estimates 0.260 m, 3.3 cm long. `head_tilt` alone
estimates 0.439 m. Those two pitch comparisons used a frozen 0.335 m
height. The gate below uses the live camera z.

The floor leaves the frame before an eye-only `d_min`. At about
−15.6° world pitch the bottom row is an eye range of about 0.14 m.
`d_min` at `T_detect` = 0 is 0.1263 m, so the in-frame compare only
has a sliver. A kit `T_detect` of 0.1 s or more puts that `d_min` at
or past the bottom row, and an in-frame-only gate cannot fire.
Soft-pass is off. With the gate off, the left ankle still hits
`col_chair_stool_b_leg_2` at 6.82 s and 20.4 N.

The foot is not ahead of the eye on this plant. The forward offset is
the leading bottom corner of `l_foot_contact` / `r_foot_contact`
minus `kit_cam`, dotted with body forward. Positive means the toe is
ahead of the camera. At stand (t = 1.00 s) both soles are −0.016 m
and `cam_z` is 0.335 m: the lean puts the eye 1.6 cm ahead of the
sole. The near edge, from under the eye, is 0.141 m, and the toe gap
of that edge is 0.158 m. At t = 5.90 s `cam_z` is 0.332 m, the left
sole (not swinging) is +0.006 m, and the right swing sole is −0.037 m.
The near edge is 0.141 m and the toe gap of that edge is 0.135 m.
That single-frame sole is not the gate offset. The unstopped contact
is the left ankle on `col_chair_stool_b_leg_2`. The offset is the
furthest either toe reaches over a gait period.

Over the 0.400 s gait period, both feet, on this walk with no stop:
the high water is **+0.017 m** on the right foot at t = 3.056 s.
Once the gait is up, a period peaks near +0.015 m on the right and
+0.013 m on the left. The commanded step length is 0.020 m. A stop
that used the sole at one frame and no buffer, issued at 6.200 s,
left the closest foot 0.133 m from a leg floor against a 0.1263 m
floor. That is about 7 mm. It is not kit-safe.

    cam_z = kit_cam world z at the frame
    eye_range = cam_z / tan(depression(row, IMU + head_tilt))
    step_off = max forward toe offset of either foot since the walk
    toe_gap = eye_range − step_off
    compare (toe_gap − buffer) with d_min
    d_min = 0.150 × (T_detect + 0.842)

`buffer` is 0.03 m or 0.05 m, a fixed slip allowance. 0.03 m is a bit
more than the 0.020 m commanded step. 0.1263 m stays the floor at
`T_detect` = 0. It is not a safe gap. `T_detect` of 0.033 s is one
30 fps frame, and 0.100 s is the blind-zone warning. Neither is a
kit measurement. The 0.342 ms RGB compute is not plugged in.

All six of those stops have 0 prop contacts. min up_z is 0.934. Every
stop-window leg joint peaks at 2.280 Nm. None crosses 2.33 Nm.

| `T_detect` | buffer | stop | `d_min` | compare | post-stop foot–leg | settle |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 0.03 m | 5.680 s | 0.1263 m | 0.123 m | 0.207 m | 0.788 s |
| 0 | 0.05 m | 5.640 s | 0.1263 m | 0.123 m | 0.207 m | 0.668 s |
| 0.033 s | 0.03 m | 5.664 s | 0.1313 m | 0.131 m | 0.206 m | 0.764 s |
| 0.033 s | 0.05 m | 5.624 s | 0.1313 m | 0.129 m | 0.209 m | 0.668 s |
| 0.100 s | 0.03 m | 5.648 s | 0.1413 m | 0.139 m | 0.206 m | 0.666 s |
| 0.100 s | 0.05 m | 5.176 s | 0.1413 m | 0.139 m | 0.284 m | 0.836 s |

These settles run 0.666–0.836 s. They do not raise the 0.842 s worst,
so `T_stop` in the formula stays 0.842 s.

Every one of those stops is on `col_chair_stool_b_leg_0`, not the leg
the ankle hits. The row model ignores the column. At the 0.03 m /
`T_detect` = 0 stop the eye range is 0.170 m and the true
camera-to-floor gap is 0.351 m, short by 0.181 m. The compare is
0.123 m. The closest toe to that leg's floor is 0.316 m. The same
pattern holds for the other five: the eye is 0.17–0.20 m short. The
early stop is that off-axis read, plus the buffer and the 0.017 m
offset. It is not a calibrated toe gap. On the unarmed walk,
`col_chair_stool_b_leg_2` at t = 5.824 s (row 449) has eye 0.182 m
against a true gap of 0.197 m, short by 0.015 m. The buffered stops
happen before that frame. The post-stop clearance above is real, and
it is not a measured 3 cm of kit margin. Not kit-safe.

The corridor half-widths are the outer edges of `l_foot_contact` and
`r_foot_contact` at the stand pose, in the body frame. The plant box
half-width is 0.0380 m and the geom pos is already 0.014 m outboard,
so the ankle-frame outer face is at ±0.052 m. Posed, the body-frame
outer edges are **+0.0867 m** and **−0.0867 m**. The inboard edges are
+0.0096 m and −0.0096 m. On `vel(+0.150, ±0.25)` the outside foot's
outer edge goes **0.021 m** past that stand edge, so that side's
half-width is 0.108 m. Left and right match. The kitchen command is
yaw −0.25, so the outside foot is the left foot.

The Day-1 latch calls `ray_corridor.estimate_hazard` and sends `stop`
when the hit is inside that corridor and `toe_gap_m` is at or under
`d_min`. `toe_gap_m` already subtracts the frozen 20 mm pad and the
live step offset (+0.017 m on this walk). No 3–5 cm buffer is added.
The pixel is still the sim projection of a stool-leg floor point, not
an RGB finder.

On the kitchen yaw −0.25 walk the latch is `col_chair_stool_b_leg_2`,
the leg the ankle hits. `leg_0` still reaches a forward `toe_gap` near
`d_min`, at a sideways offset of about −0.24 m, and the corridor
leaves it out. Eye range matches the true camera-to-floor gap at the
reported precision. At `T_detect` = 0 the stop is 5.904 s, `toe_gap`
0.125 m, eye 0.180 m against a true gap of 0.180 m. Prop contacts are
0. The closest foot-to-leg-floor after the stop is 0.158 m. The stop
peak is +2.280 Nm on the left hip pitch. Settle is 0.668 s. The same
latch at `T_detect` 0.033 s stops at 5.888 s (foot–leg 0.160 m, settle
0.788 s, min up_z 0.932) and at 0.100 s stops at 5.856 s (foot–leg
0.165 m, settle 0.642 s). No stop-window leg joint crosses 2.33 Nm.
These settles do not raise the 0.842 s `T_stop`. The forward gap
without the 20 mm pad is still about 0.145 m, above the 0.1263 m
floor, so the stop is the pad. That pad is a stand-in, not a measured
leg. Not kit-safe. The 3–5 cm buffer is not sized from this clear.

A stress froze the pad at 0.035 m for the run. The module constant
stays 0.020 m. The buffer stays off. The head stays level. The same
kitchen yaw −0.25 walk still latches `col_chair_stool_b_leg_2`.
`leg_0` stays outside the corridor. Its closest sideways is −0.243 m
at `T_detect` = 0, and the pad-inflated edge of that interval is still
about 0.12 m outside the right corridor edge. `leg_3` is inside the
corridor on both pads, at a sideways offset near +0.05 m and a
`toe_gap` near 0.47 m, so it is not the latch. Prop contacts are 0.
No stop-window leg joint crosses 2.33 Nm. min up_z is 0.934. These
settles do not raise the 0.842 s `T_stop`.

| `T_detect` | stop | `toe_gap` | eye / true | foot–leg after stop | settle | stop peak |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 5.856 s | 0.126 m | 0.193 / 0.193 m | 0.165 m | 0.642 s | right knee +2.280 Nm |
| 0.033 s | 5.840 s | 0.130 m | 0.196 / 0.196 m | 0.170 m | 0.610 s | right knee +2.280 Nm |
| 0.100 s | 5.712 s | 0.141 m | 0.194 / 0.194 m | 0.208 m | 0.804 s | left knee −2.280 Nm |

Against the 20 mm stops (5.904 s, 5.888 s, 5.856 s) the clock moves
earlier by 0.048 s, 0.048 s, and 0.144 s. At `T_detect` = 0 the eye
at the stop is 0.193 m against 0.180 m on the 20 mm run. The gap
without this pad, at that frame, is 0.161 m, still above the 0.1263 m
floor, so the stop is still the pad. Not kit-safe. The 3–5 cm buffer
is not sized from this clear.

The Day-1 stop now reads `hazard_finder.find_hazard_cues` on the
kit_cam RGB frame and sends `stop` when a cue is inside the corridor
and `toe_gap_m` is at or under `d_min`, or when a cue sets
`too_close`. The pad stays 0.020. The buffer stays off. The head stays
level. The leg name below is a match within 48 px of the sim-projected
floor point. That projection is not the stop input.

| `T_detect` | stop | path | latch | eye / sim eye | foot–leg | settle | stop peak |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | 5.864 s | cues | `leg_2` | 0.181 / 0.191 m | 0.163 m | 0.680 s | left hip pitch +2.280 Nm |
| 0.033 s | 5.848 s | cues | `leg_2` | 0.184 / 0.194 m | 0.167 m | 0.624 s | left hip pitch +2.280 Nm |
| 0.100 s | 5.736 s | pixel | `leg_2` | 0.180 / 0.193 m | 0.198 m | 0.540 s | left knee −2.280 Nm |

`too_close` does not fire on these stops. Prop contacts are 0. min
up_z is 0.934. No stop-window leg joint crosses 2.33 Nm. These settles
do not raise the 0.842 s `T_stop`. `leg_0` stays outside the corridor.
On the first two runs its forward gap is already under `d_min` at a
sideways offset of −0.270 m, and the corridor leaves it out.

The stops are earlier than the sim-projection latch (5.904 s, 5.888 s,
5.856 s) by 0.040 s, 0.040 s, and 0.120 s. At the `T_detect` = 0 frame
the finder eye is 0.181 m and the sim eye is 0.191 m. The sim
`toe_gap` at that frame is 0.139 m, still above 0.1263 m, so the lead
is the short RGB read. The finder gap without the 20 mm pad is 0.146 m,
still above that floor, so the stop is still the pad. Not kit-safe.
The 3–5 cm buffer is not sized from this clear. On the first two runs
the winning cue is not the lowest blob, so `find_hazard_pixel` alone
would not have stopped on that frame.

On the frame that fires each stop, finder eye minus the true
camera-to-floor range is short. The same frame's `toe_gap` is short
of the sim-ray gap by about the same amount. True range is the
horizontal distance from `kit_cam` to that leg's floor point.

| `T_detect` | stop | finder eye | true | eye bias | finder `toe_gap` | GT `toe_gap` | gap bias |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | 5.864 s | 0.181 m | 0.191 m | −0.011 m | 0.126 m | 0.139 m | −0.013 m |
| 0.033 s | 5.848 s | 0.184 m | 0.194 m | −0.011 m | 0.130 m | 0.143 m | −0.013 m |
| 0.100 s | 5.736 s | 0.180 m | 0.193 m | −0.013 m | 0.141 m | 0.154 m | −0.013 m |

The bias does not stay short on the walk up to that stop. Of 1492
matched cue samples, 503 read long by more than 1 mm. The longest is
+0.164 m at t = 4.544 s on `leg_0`, sideways −0.523 m, outside the
corridor (finder eye 0.659 m, true 0.495 m). Inside the corridor, 29
of 250 samples read long. The longest of those is +0.116 m at
t = 5.312 s on `leg_2` (finder eye 0.384 m, true 0.268 m, finder
`toe_gap` 0.345 m, GT `toe_gap` 0.231 m). That true gap is still above
0.1263 m. No in-corridor sample has a true `toe_gap` already at or
under 0.1263 m while the finder `toe_gap` is still above it. The
frames that fire the stop are the short ones. Not kit-safe.

The Day-1 stop keeps the shortest in-corridor floor point. A later
frame can replace that point only when the gap gets shorter. Each
tick re-reads the point in the current body x, y, and yaw, and drops
it when the point leaves the corridor. A new cue joins a saved point
only when the floor positions, after that re-read, are within
0.035 m. That radius is frozen before the run. Sim leg names are not
a key. The kit has no leg ids. The name in the table is the nearest
stool-leg floor point after the stop, and it is not an input.

On this kitchen yaw the fire does not move versus the cue-only latch.
The point that crosses `d_min` is born on the fire tick, except at
`T_detect` = 0.100 s, where it is born one tick earlier and still
crosses at 5.736 s. The report name is `leg_2`. That saved point sits
0.014 m, 0.014 m, and 0.013 m from that leg's floor. The 0.035 m
radius leaves other hits of the same leg on their own tracks: 3, 3,
and 4 tracks at the three stops. The farthest of those is 0.100 m,
0.100 m, and 0.184 m from the same floor, with gaps 0.231 m, 0.235 m,
and 0.327 m, all above `d_min`. None of them is `leg_0`. `leg_0`
stays outside the corridor. `too_close` does not fire. Prop contacts
are 0. The stop peaks stay at 2.280 Nm. At the fire the finder eye
is still short of the true camera-to-floor range by 0.011 m, 0.011 m,
and 0.013 m. A join of 0.20 m on the same walk stopped at these same
times. Not kit-safe.

| `T_detect` | stop | born | tracks | report | world | eye bias | gap bias | foot–leg | settle | stop peak |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | 5.864 s | 5.864 s | 3 | `leg_2` | 0.014 m | −0.011 m | −0.013 m | 0.163 m | 0.680 s | left hip pitch +2.280 Nm |
| 0.033 s | 5.848 s | 5.848 s | 3 | `leg_2` | 0.014 m | −0.011 m | −0.013 m | 0.167 m | 0.624 s | left hip pitch +2.280 Nm |
| 0.100 s | 5.736 s | 5.728 s | 4 | `leg_2` | 0.013 m | −0.013 m | −0.013 m | 0.198 m | 0.540 s | left knee −2.280 Nm |

A one-frame long read does not cancel that stop. The stress arms on the first frame whose shortest in-corridor gap is at or under `d_min + 0.05`, then adds 0.10 m to the toe gap of the next frame's cue when that cue's floor point is within 0.035 m of the armed point. The finder is normal again after that frame. Sim leg names are not used to pick the cue. On this kitchen yaw the offered gaps are 0.276 m, 0.279 m, and 0.286 m. The latch keeps 0.176 m, 0.178 m, and 0.186 m, the gap already on the track after the odometry re-read. The join distances are 0.004 m, 0.002 m, and 0.004 m. The stop stays at the cue-only times, 5.864 s, 5.848 s, and 5.736 s. Prop contacts are 0. The stop peaks stay at 2.280 Nm. The report name is still `leg_2`. Not kit-safe.

| `T_detect` | arm | inject | real | offered | kept | join | stop | contacts | stop peak |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | 5.480 s, 0.176 m | 5.488 s | 0.176 m | 0.276 m | 0.176 m | 0.004 m | 5.864 s | 0 | left hip pitch +2.280 Nm |
| 0.033 s | 5.448 s, 0.180 m | 5.456 s | 0.179 m | 0.279 m | 0.178 m | 0.002 m | 5.848 s | 0 | left hip pitch +2.280 Nm |
| 0.100 s | 5.416 s, 0.189 m | 5.424 s | 0.186 m | 0.286 m | 0.186 m | 0.004 m | 5.736 s | 0 | left knee −2.280 Nm |

The same chain at `T_detect` = 0 does not clear the five furnished rooms. Every scene has furniture colliders. None is an empty plant. Bedroom furniture sits near x 2.4 m, and a 9 s walk ends near x 1.16 m on a straight approach, so that room's unstopped walks have no prop contact. A stop there is not needed. Two bouts latch the prop the unstopped walk hits, with 0 contacts and a stop peak of 2.280 Nm: kitchen left on `col_chair_stool_b_leg_2` at 5.864 s, and living straight on `col_coffee_leg_0` at 4.376 s. Entrance straight does not stop, and the left foot hits `col_mat_rug` at 6.892 s, 11.3 N. Bathroom straight stops at 8.952 s on `col_toilet_seat` with the saved point 0.313 m from that seat and the eye 0.312 m short of the true range; the unstopped walk has no contact by 9.0 s. Living right stops at 4.000 s on an unmatched last-row `too_close`, not on `col_coffee_leg_2`, which the unstopped walk hits at 8.104 s. The other false stops are unmatched `too_close` on the last row, with no prop contact in the unstopped walk. Stop-window peaks stay at 2.280 Nm. Walks that do not stop stay under 2.33 Nm, worst hip roll −2.128 Nm or +2.278 Nm. Not kit-safe. Not go-anywhere.

| room | approach | stop | latch | contacts | peak |
| --- | --- | --- | --- | --- | --- |
| kitchen | straight | none | | 0 | hip roll −2.128 Nm |
| kitchen | left −0.25 | 5.864 s | `stool_b_leg_2` | 0 | +2.280 Nm |
| kitchen | right +0.25 | none | | 0 | hip roll +2.278 Nm |
| bathroom | straight | 8.952 s | `toilet_seat`, 0.313 m off | 0, unstopped also 0 | +2.280 Nm |
| bathroom | left −0.25 | 5.416 s `too_close` | none | 0, unstopped also 0 | +2.280 Nm |
| bathroom | right +0.25 | none | | 0 | hip roll +2.278 Nm |
| living | straight | 4.376 s | `coffee_leg_0` | 0 | +2.280 Nm |
| living | left −0.25 | 3.992 s `too_close` | none | 0, unstopped also 0 | −2.280 Nm |
| living | right +0.25 | 4.000 s `too_close` | none | 0; unstopped hits `coffee_leg_2` at 8.104 s | −2.280 Nm |
| bedroom | straight | 4.560 s `too_close` | none | 0, unstopped also 0 | −2.280 Nm |
| bedroom | left −0.25 | 1.976 s `too_close` | none | 0, unstopped also 0 | +2.280 Nm |
| bedroom | right +0.25 | 8.104 s `too_close` | none | 0, unstopped also 0 | −2.280 Nm |
| entrance | straight | none | | rug at 6.892 s, 11.3 N | hip roll −2.128 Nm |
| entrance | left −0.25 | 5.408 s `too_close` | none | 0, unstopped also 0 | +2.280 Nm |
| entrance | right +0.25 | none | | 0 | hip roll +2.278 Nm |

Existing furniture legs were checked on the open ±0.25 walks, with no room XML moved. A leg center enters the strip between the stand outer edge, 0.0867 m, and the yaw-outside edge, 0.108 m, on kitchen left, living left, living right, bathroom right, and entrance right. Kitchen right, bathroom left, both bedroom turns, and entrance left do not. The close crossings are kitchen-left `col_chair_stool_b_leg_2` at 6.280 s (sideways +0.088 m, 0.193 m ahead) and `col_chair_seat_leg_2` down to 0.234 m ahead at +0.090 m, and living-right `col_coffee_leg_2` at sideways −0.091 m with 0.093 m ahead. The other crossings are still 0.57 m to 1.76 m ahead.

The 0.020 m pad already counts a floor point as inside the stand corridor until |sideways| passes 0.1067 m. Widening the outside edge to 0.108 m changes that call only for |sideways| in (0.1067, 0.128]. Inside the named strip that leaves (0.1067, 0.108]. The only centers there are far: living-right `col_coffee_leg_0` at 0.567 m (side −0.107 m) and entrance-right console legs at 1.49 m and beyond (side −0.107 m to −0.108 m). Those gaps stay above `d_min`. They do not stop the walk. The close crossings are inside the stand corridor once the pad is applied. Kitchen left stops at 5.864 s with the saved point at sideways +0.069 m, before `stool_b_leg_2` enters the strip. Living right's close crossing is on the open walk, after the unmatched `too_close` at 4.000 s, so the latch never sees it.

No existing leg sits in (0.1067, 0.108] at a forward gap near `d_min`. A leg placed there, so the widened edge is what makes the in/out call, is a room-collider move. The plant file was not edited. That placement is open for MFG/Hardware. The five-room bar is unchanged. Not kit-safe. Not go-anywhere.

On the same entrance straight walk, `col_mat_rug` is a box. The `mat` body is at z 0, the geom pos z is 0.006 m, and the half-height is 0.006 m. The top face is at world z 0.012 m. The sim center z is 0.006 m. The left sole `l_foot_contact` hits that geom at 6.892 s, 11.28 N, in double support. No swing-phase sole corner is over the rug. The first sole corner over the rug is at 6.878 s, still double support, z 0.0218 m, clearance +9.8 mm. The lowest sole corner over the rug is the contact frame, z 0.01198 m, clearance −0.02 mm. The leading bottom corner at that frame is at z 0.0194 m, clearance +7.4 mm.

That contact is a step-on, not a Day-1 stop. While the gait is still walking on the rug, before 7.55 s, the ankles stay under the 2.33 Nm sag bar: left pitch +1.282 Nm, left roll +1.171 Nm, right roll +1.135 Nm, right pitch −1.083 Nm. The stance sole does span the edge. At 7.016 s the left heel is off the rug at z −0.0012 m with 15.5 N on the floor, and the left toe is inside the rug at z 0.0092 m with 15.6 N on the rug. The corner tilt is 10.4 mm. The toe is 2.8 mm under the 0.012 m top, so it is on the edge, not seated on the top face. Ankles during those split ticks stay at or under 1.282 Nm. Contact CoP margin stays at or above 0. The sole box is not left (max outside 0.000005 m). Firm stance slip on the rug, rug normal above 15 N, median 0.019 m/s. min up_z on the bout is 0.934. The body stays upright.

At 7.560 s the bus flags airborne and the command goes to 0. up_z is 0.979. The left sole is on the rug at 13.4 N and the right foot is up. Neither foot is on the floor geom, and both ankle bodies are above 0.025 m, so the floor-only check counts the rug as air. That is not a tip.

On this same entrance straight walk, 640 swing ticks before the 7.560 s fault: `SWING_TOE_MIN_M` = −0.003066. The swing-toe p90 is 0.025967 and is report-only. The arm line is frozen before the run at `SWING_TOE_MIN_M − 0.002` = −0.005066. The sole minimum on those ticks is −0.003115 and the sole p90 is 0.017508, also report-only. The minimum is at or under 2 mm, so the gait scuffs. Raise that clearance. Do not silently disarm the arm line. Until the clearance is honest, this floor-edge arm is not a useful stop. The 12 mm rug stays a step-on.

The negative toe is not a command through the floor. At the minimum, t = 3.384 s, right swing, the hip-frame foot z is +3.01 mm above the stand command, and the foot-box center is +7.93 mm, +0.65 mm above its stand height. A level sole at that center would bottom near 0. The sole is toe-down 3.2°: heel corner +2.96 mm, toe corner −3.066 mm. That toe corner is the floor contact. The contact distance is −3.066 mm, the same number, at 12.9 N. Across the 154 negative-toe ticks the median penetration is −2.6 mm at 7.3 N, and 141 of them are commanded above the stand height. Ankle torque at the minimum is +1.07 Nm, and the joint is 0.025 rad off its command. The plane contact holds the pitched corner about 3 mm through z = 0. Forcerange, kp, damping, and armature were not raised. Not kit-safe. Not go-anywhere.

`MID_SWING_TOE_MIN_M` = −0.003066 on the middle 20–80% of each swing, 384 ticks, at t = 3.384 s. The mid-swing p90 is 0.024671 and is report-only. That minimum is inside the phase window, not only at lift-off or touchdown. 97 of the 101 negative mid-swing ticks still have floor normal above 1 N. The other 287 mid-swing ticks, floor normal at or under 1 N, reach −0.001748. The −3.066 mm reading is a loaded contact tick that sits in the middle of the phase clock.

From 7.016 s to 7.560 s both ankle pitch joints stay inside ±2.09 rad. The closest approach is 1.375 rad on the right and 1.381 rad on the left. At 7.016 s the left ankle is the loaded one, q +0.4815 against command +0.4620, 0.020 rad apart, with 15.5 N on the floor and 15.6 N on the rug. The right ankle is in the air, q −0.7152 against command −0.7064. At 7.560 s the left ankle is q +0.4613 against command +0.5462, and the right, still in the air, is q −0.6974 against command −0.5462, 0.151 rad apart. Over the window the largest command gap is 0.106 rad on the left and 0.151 rad on the right. Ankle command stays a fixed ±0.2618 rad off hip plus knee at both ends of the window. Torso pitch is −0.2477 rad (−14.2°) at 7.016 s and −0.1996 rad (−11.4°) at 7.560 s. The left sole normal’s forward component at 7.016 s is −0.083 rad, the 10.4 mm edge tilt, while the torso is at −14.2°. The ankle is not on its range stop. The command holds the flat hip-plus-knee offset, and the body is pitched over the edge. Not kit-safe. Not go-anywhere.

Two control-only copies of that same entrance straight walk were scored and left out of `locked_kit_config`. `gm_z_m` stays 0.020. Forcerange, kp, damping, and armature were not raised. The plant file is unchanged. Soft-pass is off. A copy clears only if the bout does not flag airborne, the mid-swing toe minimum is above 0.002 m, both ankle pitch and roll stay under 2.33 Nm, and contact CoP stays in the sole box.

The swing-height copy sets `gm_z_m` to 0.034. At about 20% of single support the OP3 z sine sits about 0.37 × `gm_z_m` above the frozen stance foot, so 0.034 m is the command that would add about 5 mm at the phase of the −3.066 mm scuff if the foot followed it. On 384 mid-swing ticks the toe minimum is −0.002859 at t = 3.128 s, with floor normal 5.3 N. The mid-swing p90 is 0.036256 and is report-only. That minimum is still under 0.002 m, and it is still a loaded floor contact. The bus flags airborne at 7.568 s. The 7.560 s sample is the control tick before that flag. The bout is still airborne, so the 8 ms move is not a pass. Walking ankle peak before the fault is −1.741 Nm on the left roll at 1.016 s. After the fault the stop rail is +2.280 Nm on the right ankle pitch at 7.570 s, under 2.33 Nm. CoP max outside the sole is 0.000003 m. Prefer FAIL. Not a clear.

The ankle copy keeps `gm_z_m` at 0.020. On a sole that already has floor normal and rug normal both above 1 N, it adds atan(0.012 / 0.135) = 0.088656 rad of toe-up, left positive and right negative. That offset is on for 49 ticks. The mid-swing toe minimum stays −0.003066 at t = 3.384 s, floor normal 12.9 N, 382 mid ticks. The mid-swing p90 is 0.024628 and is report-only. The bus flags airborne at 7.512 s, so 7.560 s is already inside that fault. Walking ankle peak before the fault is +1.939 Nm on the right pitch at 7.424 s, under 2.33 Nm. After the fault the stop rail is +2.280 Nm at 7.526 s. CoP max outside the sole is 0.000005 m. Prefer FAIL. Not a clear. Not kit-safe. Not go-anywhere.

The rug step-on is a flat-foot fight. The ankle command holds the fixed hip-plus-knee offset, so it does not take the 4.4° sole tilt (10.4 mm over the 135 mm sole, 0.0769 rad). That fight is what pitches the body over the edge. The ankle joint itself still has about 0.7 rad of range before ±2.09.

On the swing that contains the mid-swing minimum, the swing toe is the right foot. The phase runs 3.344 s to 3.536 s, 25 ticks. At t = 3.384 s the toe is −0.003066 m. Over that phase the swing hip pitch peaks at +1.331 Nm (3.424 s) and the swing knee at +1.582 Nm (3.344 s). At the toe tick the hip pitch is +0.348 Nm and the knee is −0.907 Nm. Both stay under 2.33 Nm and under the plant rail of 2.45 Nm. The knee at that tick is q −1.1335 rad. The applied command is −1.1484 rad, 0.0149 rad off q. The unslewed gait target is −1.1774 rad, 0.0439 rad off q. Across the phase the gait target sits between −0.1160 rad and +0.1024 rad off q, and the applied command between −0.0813 rad and +0.0707 rad off q, while the knee force stays at or under 1.582 Nm. The swing joints do not saturate. The commanded swing-toe height at that tick is the gait-target forward kinematics of the leading bottom corner: −0.003205 m. The four commanded bottom corners share that height, so the target sole is level. The actual toe is −0.003066 m, 0.000139 m above the command. The command is on the floor. That is trajectory shape. The sag case would be a command clear of the floor with the actual toe below it. This tick is the command itself.

That −0.003205 m command is the gait-target forward kinematics on the live pelvis at t = 3.384 s, not on the stand pelvis. The same leg target on the stand pelvis, body z 0.210932 m and pitch −0.2587 rad, puts the leading corner at +0.007686 m. Live minus stand at that tick is −0.010890 m. The pelvis drop is −0.001742 m (body z 0.209190 m against 0.210932 m). The pelvis-to-leading-corner offset is the same body-frame point in both forward kinematics: +0.109801 m forward, −0.098448 m lateral, −0.181185 m down. The live world horizontal offset is 0.097288 m, and 0.050028 m of that is forward along the heading. Pitch is −0.2960 rad against −0.2587 rad, Δ −0.037282 rad. Pitch times the 0.050028 m forward horizontal offset is −0.001865 m. Pitch times the 0.097288 m horizontal offset is −0.003627 m. The exact forward-axis term is −0.003937 m. That term is the pitch piece of (R_live − R_stand) on the corner. The roll piece of the same matrix difference is −0.007631 m. Δroll is 0.081123 rad and the lateral offset is −0.098448 m, so Δroll × y is −0.007986 m and sin(Δroll) × y is −0.007978 m. The direct roll term sits next to that −8.0 mm roll math. It is not the −0.005212 m lump, which was the roll piece plus the +0.002419 m up-axis piece. After the drop, the exact pitch piece, and this exact roll piece, the residual is +0.002419 m. That residual is the body-up axis of the same rotation. Drop + pitch + roll + residual equals the −0.010890 m gap. The six right-leg targets still match, max |Δq| = 0. Nothing else is in the gap.

Over the right swing 3.344–3.536 s the left hip roll peaks at −2.121 Nm at 3.416 s, under 2.33 Nm and under 2.45 Nm. At that tick q is +0.0362 rad and the command is −0.0054 rad. The largest command-minus-q is −0.0566 rad at 3.432 s. Hip-roll kp is 40, so 2.33 Nm is 0.0583 rad, and that error is inside it. The left ankle roll peaks at −1.007 Nm at 3.536 s. At that tick q is +0.0396 rad and the command is −0.0063 rad. The largest command-minus-q is +0.0558 rad at 3.344 s. Ankle-roll kp is 35, so 2.33 Nm is 0.0666 rad. At t = 3.384 s the hip roll is −0.662 Nm (q +0.0185, command +0.0466) and the ankle roll is −0.236 Nm (q +0.0476, command +0.0861). Neither joint is at 2.33 Nm or at the plant rail. The 0.081 rad pelvis roll is the walk pose, not an HX-35H limit.

A stance-hip copy, left out of the locked kit, adds the measured cmd−q error to the stance hip-roll target: −0.0566 rad on the left and +0.0566 rad on the right. The 1.29 Nm term, 1.29/40 rad, stays off. On the right swing 3.344–3.536 s the pelvis roll at 3.384 s is 0.0475 rad, and the window peak is −0.1134 rad at 3.536 s. The left hip roll peaks at −1.980 Nm. Peak |cmd−q| on that hip is 0.0608 rad, so the angle error does not go away. The left ankle roll peaks at −1.001 Nm, and its peak |cmd−q| is 0.0568 rad. The right hip roll hits the plant rail, +2.45 Nm, at 1.272 s. The mid-swing toe minimum is −0.003022 m at 2.872 s, floor normal 10.3 N. Airborne is at 7.560 s. min up_z is 0.934, so the rug window does not tip. Prefer FAIL.

The same offset plus an 8 mm Bézier bump, zero at toe-off and at 40% of single support, leaves the 20–30% command at 0.002932 m. The mid-swing toe minimum is −0.002143 m at 1.848 s. The right hip is still on +2.45 Nm at 1.272 s. Pelvis roll at 3.384 s is 0.0390 rad, and the window peak is −0.0882 rad. Airborne is at 7.552 s, so 7.560 s is inside that fault. min up_z is 0.934. Prefer FAIL. Neither copy clears. `gm_z_m` stays 0.020. Not kit-safe. Not go-anywhere.

A position servo keeps |cmd−q| near τ/kp. At about 2 Nm and hip-roll kp 40 that gap is about 0.05 rad. That gap is not a fail. The score is the actual hip roll against the un-offset kinematic target, Δq_unoffset = q − kin, and the pelvis roll. The constant ±0.0566 rad offset above steps on in one tick at the stance switch. That step put the right hip on +2.45 Nm at 1.272 s and took the pelvis to −0.1134 rad at 3.536 s. The pelvis move at 3.384 s, from 0.081 rad to 0.0475 rad, still counts. The 0.0608 rad |cmd−q| on that copy is the servo gap.

A ramped copy, left out of the locked kit, adds the same ±0.0566 rad only on the current stance hip and fades it across double support when the stance foot switches. The swing hip stays on the kinematic target. The 1.29 Nm term stays off. Gains are 0.25×, 0.50×, and 1.0×. The hardware bar is the peak |τ| on all four hips, left and right roll and left and right pitch, against 2.33 Nm. A 2.45 Nm plant rail is Prefer FAIL. A copy clears only if the mid-swing toe minimum is above +0.002 m, the bout does not flag airborne at 7.560 s, every leg stays under 2.33 Nm, and the rug window does not tip.

At 0.25× the mid-swing toe minimum is −0.003002 m at 3.128 s, floor normal 11.2 N. Airborne is at 7.560 s. Every leg stays under 2.33 Nm. The four hip peaks are left roll −1.988 Nm at 4.438 s, right roll +2.055 Nm at 3.160 s, left pitch +1.667 Nm at 5.456 s, and right pitch −1.682 Nm at 3.160 s. The walking knee is +2.013 Nm at 1.286 s. min up_z is 0.934. CoP max outside the sole is 0.000003 m. Pelvis roll at 3.384 s is 0.0686 rad, and the window peak is +0.0842 rad at 3.344 s. Δq_unoffset at 3.384 s is −0.0476 rad on the left and −0.0506 rad on the right. The window peaks of that gap are −0.0524 rad at 3.368 s and −0.0521 rad at 3.416 s. On that window the left hip roll is −1.983 Nm, the right hip roll is −0.488 Nm, the left hip pitch is +1.663 Nm, the right hip pitch is +1.336 Nm, and the stance ankle roll is −1.033 Nm. The right hip stays off the 2.45 Nm rail. The actual hips stay about 0.05 rad off the un-offset kinematic target. Prefer FAIL on the toe and on airborne at 7.560 s.

At 0.50× the mid-swing toe minimum is −0.002668 m at 1.848 s, floor normal 15.9 N. The walking knee is +2.395 Nm at 7.426 s, over 2.33 Nm and under 2.45 Nm. Airborne is at 7.568 s, so 7.560 s is the tick before that flag and the bout is still airborne. min up_z is 0.934. Pelvis roll at 3.384 s is 0.0576 rad, and the window peak is −0.0796 rad at 3.536 s. Δq_unoffset at 3.384 s is −0.0530 rad on the left and −0.0510 rad on the right. The four hip peaks are left roll −2.134 Nm at 1.528 s, right roll +2.197 Nm at 1.272 s, left pitch +1.573 Nm at 4.426 s, and right pitch −1.625 Nm at 3.148 s. Prefer FAIL.

At 1.0× the mid-swing toe minimum is −0.001981 m at 2.872 s, floor normal 3.4 N. The right knee reaches the plant rail, +2.45 Nm, at 7.424 s. The left hip roll is −2.339 Nm at 1.016 s, over 2.33 Nm. Airborne is at 7.824 s. 7.560 s is not the declaring tick, and the bout still goes airborne. min up_z is 0.934. Pelvis roll at 3.384 s is 0.0347 rad, and the window peak is −0.0713 rad at 3.536 s. Δq_unoffset at 3.384 s is −0.0550 rad on the left and −0.0524 rad on the right. The other three hip peaks are right roll +2.305 Nm at 7.416 s, left pitch +1.601 Nm at 7.010 s, and right pitch +1.523 Nm at 7.492 s. Prefer FAIL.

A phase-local Bézier alone, with no hip offset, is zero at toe-off and at 40% of single support and peaks at 20%. At 8 mm the mid-swing toe minimum is −0.001785 m at 2.616 s with no floor force. The 20–30% command minimum is 0.004459 m. The walking knee is +2.101 Nm at 7.232 s. Airborne is at 7.560 s. The four hip peaks are left roll −2.052 Nm at 2.378 s, right roll +2.081 Nm at 2.636 s, left pitch +1.321 Nm at 6.986 s, and right pitch −1.301 Nm at 3.652 s. min up_z is 0.934. Prefer FAIL. At 10 mm the mid-swing toe minimum is −0.001293 m at 2.872 s with no floor force. The 20–30% command minimum is 0.006454 m. The walking knee is +2.221 Nm at 7.232 s. Airborne is at 7.560 s. The four hip peaks are left roll −2.051 Nm at 2.890 s, right roll +2.077 Nm at 2.634 s, left pitch +1.336 Nm at 6.986 s, and right pitch −1.291 Nm at 5.186 s. Legs stay under 2.33 Nm. min up_z is 0.934. Prefer FAIL. That 10 mm copy is the early lift that stays under 2.33 Nm, and the toe is still under +0.002 m.

The 0.25× ramp plus that 10 mm Bézier leaves the mid-swing toe at −0.001352 m at 2.360 s with no floor force. The 20–30% command minimum is 0.005978 m. The walking knee is +2.433 Nm at 7.228 s, over 2.33 Nm and under 2.45 Nm. Airborne is at 7.560 s. min up_z is 0.934. CoP max outside the sole is 0.000005 m. Pelvis roll at 3.384 s is 0.0457 rad, and the window peak is −0.0834 rad at 3.528 s. Δq_unoffset at 3.384 s is −0.0426 rad on the left and −0.0504 rad on the right. The four hip peaks are left roll −1.986 Nm at 7.508 s, right roll +2.014 Nm at 2.126 s, left pitch +1.316 Nm at 6.986 s, and right pitch −1.286 Nm at 3.644 s. On the scuff window the left hip roll is −1.936 Nm, the right hip roll is −0.488 Nm, the left hip pitch is +1.282 Nm, the right hip pitch is +0.643 Nm, and the stance ankle roll is −0.997 Nm. Prefer FAIL.

None of the ramped gains, the Bézier-alone copies, or the combination clears. `gm_z_m` stays 0.020. The 1.29 Nm term stays off. Not kit-safe. Not go-anywhere.

The flat-floor scuff and the rug step-on are separate bars. The scuff bar is the mid-swing toe above +0.002 m and every leg under 2.33 Nm, including both knees and both ankle pitches. Airborne at 7.560 s is not that bar. The rug bar is the step-on itself.

The scuff copies start from the 10 mm Bézier that peaks at 20% of single support. That toe is −0.001293 m at 2.872 s, fraction 0.208, right foot, no floor force. The right knee is +2.221 Nm. The eight peaks stay under 2.33 Nm: left hip roll −2.051 Nm, right hip roll +2.077 Nm, left hip pitch +1.336 Nm, right hip pitch −1.291 Nm, left knee −1.997 Nm, right knee +2.221 Nm, left ankle pitch +1.443 Nm, right ankle pitch +1.276 Nm. Moving the peak to 25%, 30%, or 35% of single support, at 8, 10, or 12 mm, lowers the toe at fraction 0.208. Every copy’s minimum is that first mid-swing tick.

The best copy that still finishes at the same x, 0.942 m, keeps the peak at 20% and raises the bump to 12 mm. The mid-swing toe is −0.000807 m at 2.872 s, fraction 0.208, right foot, no floor force. The 20–30% command at that tick is +0.008407 m. The command is above the floor and the toe is still under +0.002 m. Peaks against 2.33 Nm: left hip roll −2.052 Nm at 7.504 s, right hip roll +2.073 Nm at 2.632 s, left hip pitch +1.347 Nm at 6.982 s, right hip pitch −1.282 Nm at 6.722 s, left knee −2.017 Nm at 7.176 s, right knee +2.316 Nm at 7.234 s, left ankle pitch +1.461 Nm at 7.068 s, right ankle pitch −1.300 Nm at 7.228 s. Pelvis roll at 3.384 s is 0.0314 rad. This copy adds no stance-hip offset. Airborne stays at 7.560 s. Prefer FAIL.

The 0.25× ramp on the 10 mm bump puts the right knee at +2.433 Nm. On the 12 mm bump the right knee reaches +2.45 Nm. That gain is not used.

Starting the 12 mm bump 30 ms before toe-off leaves the mid-swing toe at +0.000508 m at 1.080 s, still under +0.002 m. Every leg stays under 2.33 Nm. The body ends at x 0.820 m instead of 0.942 m. A 20 ms lead hits −2.45 Nm on the left hip pitch and +2.45 Nm on the right hip pitch and the right knee, and min up_z is 0.631. Prefer FAIL.

The rug copy, with no swing bump and no hip offset, walks the stance ankle on a split sole from the flat hip-plus-knee offset toward hip plus knee plus the 4.4° step, 0.076885 rad. Each tick stays inside 2.33 Nm at ankle kp 35. The first write is at 7.016 s on the left foot: floor 15.5 N, rug 15.6 N, measured sole tilt +0.1533 rad, q +0.4815 rad, goal +0.4149 rad. The right foot is already off the floor, 0 N on the floor and 0 N on the rug, leading corner +0.0025 m. CoP is 0. The left knee reaches −2.45 Nm at 7.176 s and the right knee reaches +2.45 Nm at 7.424 s. The bus flags a tip at 7.528 s, support margin −1.016, and min up_z is −1. At 7.560 s up_z is 0.789. The swing Bézier does not move this fault. Prefer FAIL.

Neither track clears. `gm_z_m` stays 0.020. The 1.29 Nm term stays off. Not kit-safe. Not go-anywhere.

The 9.21 mm between that 12 mm bump’s commanded toe (+8.41 mm) and the scored toe (−0.81 mm) at 2.872 s, fraction 0.208, is not a pelvis term. Goal forward kinematics and the actual leg share the live freejoint, so pelvis drop, pitch, and roll of this gap are 0.00 mm. The contact geom is one integration behind `qpos`, and that accounts for 0.32 mm. The other 8.89 mm is the swing leg. Stepping the joints from the actual pose to the goal, hip to ankle, closes it: hip yaw +0.04 mm, hip roll +6.24 mm, hip pitch +6.29 mm, knee −2.71 mm, ankle pitch +4.58 mm, ankle roll −5.55 mm. The linear pieces, (goal − q) times the sole-z slope, sum to +7.81 mm and leave +1.08 mm of curvature. Holding the hip-roll error of 0.100 rad takes 4.01 Nm at kp 40. Holding the hip-pitch error of 0.097 rad takes 4.37 Nm at kp 45. Both are over 2.33 Nm. The knee error is 0.160 rad (7.19 Nm) and it raises the toe relative to the goal, so it is not this miss. A taller bump asks those hips for more. Taller height is a dead end at these kp values and 2.33 Nm. The toe stays −0.81 mm. The right knee on that bout is +2.316 Nm. Prefer FAIL.

The floor-only airborne flag at 7.560 s is the detector. `col_mat_rug` now counts as ground for that flag, and the support margin uses the body-link COM, not the world subtree that includes the room. The same baseline then does not flag. up_z at 7.560 s is 0.979, the body-COM margin is +1.0 mm, min up_z stays 0.934, and x finishes at 1.011 m. With the floor-only check the same walk flags airborne at 7.560 s and x stops at 0.920 m. The 12 mm swing bump on the rug-aware check finishes at x 1.028 m, up_z 0.991, margin −1.2 mm. The 0.25× stance-hip ramp finishes at x 1.015 m, up_z 0.981, margin −2.0 mm. Neither tips. Hip and swing do move the rug score. They do not clear the toe.

At 7.016 s the rug face crosses the left sole at heel→toe fraction 0.915 (heel x 0.791 m, toe x 0.926 m, face x 0.915 m). The toe-side lever is 11.5 mm, not 68 mm. The measured sole is +0.1533 rad (8.78°). The stance ankle adds gain times that live tilt, and only while that foot has floor and rug contact. It is not a fixed 4.4° or 8.8°. Each tick is capped. At 0.012 rad/tick, 0.25× and 0.50× both saturate. The first write is +0.012 rad (q +0.4815, flat +0.4835, goal +0.4955). up_z at 7.560 s is 0.975 and the margin is +1.8 mm. No leg reaches 2.33 Nm. At a 0.050 rad cap the 0.25× first write is +0.038 rad and up_z is 0.968. The 0.50× first write is +0.050 rad and up_z is 0.962. The margin stays positive. min up_z stays 0.934. No tip. The one-tick 0.067 rad step that railed the knees is not this copy. The toe is still −3.07 mm. Not kit-safe. Not go-anywhere.

The 4.01 Nm and 4.37 Nm figures at the 12 mm tick are kp times the lag, the position-loop ask at sim kp 40 and 45. They are not the actuator output. At that same tick the swing hip-roll actuator_force is −0.027 Nm and the swing hip-pitch actuator_force is +0.366 Nm, both inside the ±2.45 Nm force range. The actuators are not clipped. The lag is the sim position loop settling. Prefer FAIL at sim kp 40/45. A taller bump stays a dead end because the ask is already past 2.33 Nm. The real limit on raising kp is the Hardware stiffness bench and the thaw PR.

A slower rise at 6, 8, and 10 mm, still peaking at 20%, keeps every actuator_force under 2.33 Nm and keeps x at 1.029–1.031 m. The command–actual gap shrinks (4.76 mm, 6.40 mm, 7.75 mm) and the actual toe goes the other way: −2.28 mm, −1.78 mm, −1.29 mm. Stretching the same 8 and 10 mm peaks out to 35% of swing leaves the toe at −2.74 mm and −2.72 mm, with x at 1.032 m and 1.033 m. Reaching an 8 mm peak by 10% and holding it leaves the toe at −1.60 mm, x 1.029 m, under 2.33 Nm. The 10 mm hold reaches −1.06 mm and puts the right knee on +2.45 Nm. An 8 ms lead and a 12 ms lead on the 8 mm rise leave the toe at −2.10 mm and −2.04 mm, and x stays 1.031 m. None of these copies puts the mid-swing toe above +0.002 m. The best toe that stays under 2.33 Nm is still the 12 mm peak at 20%, −0.81 mm.

At 7.016 s on the baseline the left heel corner is −1.2 mm, so the heel is on the floor. Absolute sole tilt is +0.1533 rad (8.78°). The rug edge is at heel→toe fraction 0.915, and atan(12 mm / 123.5 mm) is +0.0968 rad (5.55°). The floor-contact cloud to the rug-contact cloud is +0.0866 rad (4.96°) over 124.8 mm. The ankle follow now adds gain times that contact tilt, on top of the 12 mm / 20% bump. At 0.25× and 0.012 rad/tick the first write is +0.012 rad, the toe stays −0.81 mm, up_z at 7.560 s is 0.988, and the body-COM margin is −0.5 mm. At 0.50× and 0.050 rad/tick the first write is +0.045 rad, the toe stays −0.81 mm, up_z is 0.972, and the margin is +1.1 mm. No tip. No leg reaches 2.33 Nm. Not kit-safe. Not go-anywhere.

At 2.872 s on that 12 mm / 20% right swing, `data.ctrl` is not the IK target. Right hip roll IK is 0.182332 rad and ctrl is 0.132275 rad. Hip roll is not in the move approach. `write_clipped` holds it inside 0.060025 rad of q, which is 0.98 × 2.45 / 40. Right hip pitch IK is 0.829440 rad, the write stores that full target, and ctrl is 0.799644 rad. This schedule approaches hip pitch, knee, and ankle pitch over `gm_move_s` 0.020 s, so the 8 ms tick covers 40% of the gap. The 5.5 rad/s cap does not bind here (the step is 0.020 rad, the cap is 0.044 rad). The 0.150 s Bézier move is not this clock. The arm-swing lead morph is idle. The force MuJoCo applied, from the state at 2.870 s, is +0.030 Nm on hip roll and +0.595 Nm on hip pitch, both inside ±2.45 Nm. q and q̇ there are 0.079693 rad and 1.218 rad/s, and 0.729404 rad and 1.417 rad/s. Compiled kp is 40 and 45. Compiled kv is 1.7027 and 1.8102 with dampratio 1. The stated 1.538 and 1.551 are not the compiled bias. (kp×(ctrl−q) − force) / kv equals q̇ on that sample. The earlier −0.027 Nm and +0.366 Nm figures are that same law one 2 ms step later. They are not a second actuator output. The 8 ms and 12 ms copies are a foot-z smoothstep via `lead_s` 0.008 and 0.012. The 20 ms and 30 ms copies are `lead_frac` on the 12 mm parabola. Seconds early are `lead_frac` times the single-support window, which is 0.200 s on this walker, not the 0.400 s of both swings. None of them advances hip ctrl. A kv/kp lead was not run, because ctrl is not the IK command.

Passing the kinematic target straight through, on hip roll and on the 20 ms approach, makes ctrl equal the IK. The mid-swing toe minimum is +2.510 mm at 1.080 s, fraction 0.208, left foot, and x is 1.028 m. No fault, min up_z 0.934. Both knees are on ±2.45 Nm at 1.008 s. Prefer FAIL. Passing it through on the hips only, and leaving the knee and ankle approach, also makes hip ctrl equal the IK. The toe minimum is +0.305 mm at 3.384 s, fraction 0.208, right foot, under +2 mm, and x is 0.994 m. Both hip rolls are on ±2.45 Nm. Prefer FAIL. The walking path still uses the band and the 20 ms approach. ScuffGait stays locked. The best toe under 2.33 Nm is still −0.81 mm. Not kit-safe. Not go-anywhere.

A sim-only clip on the swing hip roll and hip pitch keeps kp·(ctrl−q) − kv·q̇ inside ±2.33 Nm. kv is the compiled 1.7027 and 1.8102, and q̇ is the live joint velocity. The 0.98 spring band is not raised. With the 20 ms approach left on, the right hip roll at 2.872 s sits on the IK: ctrl 0.182332 rad, q̇ +1.859 rad/s toward the target, predicted −0.219 Nm, actuator_force −0.219 Nm. q is 0.109 rad. The right hip pitch is still short of the IK, ctrl 0.791 rad against 0.829 rad, a 0.038 rad gap. The live re-clamp never moved a command; the write-time prediction was already inside the budget. The mid-swing toe is −1.002 mm. The right knee reaches +2.45 Nm at 7.232 s, so the copy does not hold 2.33 Nm and gm_move_s stays at 20 ms. Nothing cleared +2 mm under 2.33 Nm, so there is no per-tick bench dump. The kit position servo does not have this clip. Best toe under 2.33 Nm is still −0.81 mm. Not kit-safe. Not go-anywhere.

Track 1 torques are every leg actuator_force before 7.0 s. The right knee at 7.232 s on the clip with the 20 ms approach left on is Track 2, and it does not block this bar. Hip pitch is taken out of that approach. Knee and ankle pitch stay on `gm_move_s` 0.020. The period stays 0.500 s. The swing-hip clip stays. At 2.872 s the right hip pitch ctrl equals the IK, 0.829440 rad, q 0.742 rad, q̇ +1.795 rad/s toward the target, predicted and actuator_force +0.694 Nm. Hip-roll ctrl is still the IK, predicted and force −0.220 Nm. The mid-swing toe is −0.247 mm at 2.872 s, fraction 0.208, right foot. That is 0.56 mm above the −0.81 mm banded toe and 0.76 mm above the −1.002 mm clip toe, and it is still under +2 mm. Every leg stays under 2.33 Nm before 7.0 s. The largest are left hip roll −2.169 Nm at 2.888 s and right hip roll +2.151 Nm at 2.632 s. After 7.0 s the left knee is −2.358 Nm at 7.488 s and the right knee is +2.367 Nm at 7.944 s. The body tips at 8.024 s, margin −0.061, min up_z 0.218. x at 7.0 s is 0.828 m. The end x of 0.610 m is that fall. Sole roll about the contact-box long axis at the toe tick is +0.01317 rad. The right inside edge, local +y, is 0.50 mm above the box center, not dropped. The half-width is 38 mm, so 38 mm times that roll is 0.50 mm, on the flat line used here, and the sole is not called flat. The next copy adds the same predicted clip on swing ankle roll only. Compiled ankle-roll kv is 1.1876 and kp is 35. Ankle-roll ctrl equals the IK. The mid-swing toe moves to −2.066 mm at 2.360 s, fraction 0.208, right foot, worse than −0.81 mm and −1.002 mm. Sole roll at that tick is +0.06043 rad and the inside edge is 2.30 mm above the center. Ankle-roll lag is not dropping the inside edge. No leg reaches 2.33 Nm on the whole bout. No fault. min up_z is 0.934. x finishes at 0.983 m. Prefer FAIL. Period stays 0.500 s because the sole was not flat. Nothing cleared +2 mm, so there is no bench dump. The walking path still uses the band and the 20 ms approach. Not kit-safe. Not go-anywhere.

The scored toe is the contact box. `l_foot_contact` and `r_foot_contact` sit at local bottom −26.000 mm with the front edge 97.5 mm ahead of the ankle-roll origin, a 135×76 mm footprint. `*_toe_viz` is a sphere at bottom −24 mm and front 87 mm. `_leading_toe_z` reads the box. The −0.81 mm, −1.002 mm, −0.247 mm, and −2.066 mm toes are that box, so they are not rescored. At 2.872 s on the hip-pitch-off copy the scored corner and the lowest corner are the same one: front-outside, −0.247 mm. Front-inside is +0.753 mm, heel-outside +1.618 mm, heel-inside +2.619 mm. The viz sphere bottom at that tick is +2.746 mm. Sole pitch about the short axis is +0.01382 rad, toe-down positive. Sole roll is +0.01317 rad and the inside edge is 0.50 mm high. Right ankle pitch IK is −0.819224 rad, ctrl −0.804369 rad, q −0.755253 rad. The foot roll and pitch targets are 0 in the pelvis frame. Body roll is 0.057 rad and body pitch is 0.283 rad. The achieved sole does not copy that body pitch. Rewriting the swing foot so the sole is world-level sets the swing pitch target to −0.293 rad at 2.872 s. Before 7.0 s both hip pitches, both ankle pitches, and the right hip yaw hit ±2.45 Nm, and both knees pass 2.33 Nm. The scored toe at 7.464 s is +0.054 mm on the front-inside corner while the front-outside corner is −2.803 mm. That copy is not kept. A 0.020 rad swing toe-up, a sine from toe-off to touchdown, on the −0.247 mm baseline and not on the world-level copy, leaves every leg under 2.33 Nm before 7.0 s. The mid-swing toe is −0.013 mm at 6.968 s, fraction 0.208, right foot, front-outside. At 2.872 s that corner is +0.025 mm. Pitch ctrl still equals the IK. The left knee reaches −2.45 Nm at 8.200 s. That is Track 2. No fault. min up_z is 0.934. x at 7.0 s is 0.828 m. Prefer FAIL. Nothing cleared +2 mm under 2.33 Nm before 7.0 s. The best toe on that bar is −0.013 mm. Track 1 parks. The next gate is the kit kp/kv bench. Period stays 0.500 s. The ankle-roll clip stays off. Not kit-safe. Not go-anywhere.

The same 0.020 rad toe-up copy was printed again and not changed. Track 1 stays parked. A 0.030 rad clear was not run. At 2.872 s the contact-box bottom center is +1.215 mm. The corners are front-outside +0.025 mm (scored and lowest), front-inside +0.922 mm, heel-outside +1.508 mm, and heel-inside +2.404 mm. The low corner is 1.190 mm under the center, so a rotation about that center can raise it only to +1.215 mm, which is 0.785 mm short of +2 mm. The sine uses the gait-phase fraction, 0.235, not the cycle-index fraction 0.208. sin(π × 0.235) is 0.673. The commanded right-ankle add is −0.01346 rad at 2.872 s and the same −0.01346 rad at 6.968 s. The 20 ms approach is still on ankle pitch. At 2.872 s the gait IK is −0.819224 rad and ctrl is −0.814782 rad, so ctrl − IK is +0.004442 rad. On the hip-pitch-off copy without this add, that ctrl was −0.804369 rad. The difference is −0.010413 rad of the −0.01346 rad command. At 6.968 s the IK is −0.829264 rad, ctrl is −0.825472 rad, and ctrl − IK is +0.003792 rad. The per-step 20–80% contact-box minima use the same cycles as the scorer, ticks before 7.560 s, and the list keeps those minima that fall before 7.0 s. All 24 are at cycle fraction 0.208: +1.695 mm at 1.080 s, +4.658 mm at 1.336 s, +4.763 mm at 1.592 s, +4.011 mm at 1.848 s, +3.113 mm at 2.104 s, +1.896 mm at 2.360 s, +0.679 mm at 2.616 s, +0.025 mm at 2.872 s, +0.481 mm at 3.128 s, +0.988 mm at 3.384 s, +1.196 mm at 3.640 s, +1.122 mm at 3.896 s, +0.893 mm at 4.152 s, +0.798 mm at 4.408 s, +0.839 mm at 4.664 s, +0.887 mm at 4.920 s, +1.017 mm at 5.176 s, +0.982 mm at 5.432 s, +0.877 mm at 5.688 s, +0.890 mm at 5.944 s, +0.898 mm at 6.200 s, +0.906 mm at 6.456 s, +0.920 mm at 6.712 s, and −0.013 mm at 6.968 s. Sides alternate left, right, from the left foot at 1.080 s. The bout minimum is the last one. At 6.968 s the box center is +1.251 mm. The corners are front-outside −0.013 mm (scored and lowest), front-inside +1.176 mm, heel-outside +1.327 mm, and heel-inside +2.516 mm. Sole pitch is +0.00993 rad, toe-down. Sole roll is +0.01565 rad and the inside edge is 0.595 mm high. Not a clear. The next gate remains the kit HX-35H kp/kv bench. Not kit-safe. Not go-anywhere.

The swing-hip pre-command is the compiled dampratio-1 lag, kv/kp, on that same toe-up copy. Hip roll leads 42.57 ms (kv 1.7027, kp 40). Hip pitch leads 40.23 ms (kv 1.8102, kp 45). Foot z is not shifted. Period stays 0.500 s. dsp stays 0.20. y_swap stays 0.020. The contact-box toe minimum is +1.696 mm at 1.080 s, fraction 0.208, left foot, front-outside. The lowest corner there is heel-outside, +1.223 mm. The box center is +2.349 mm. The viz bottom is +4.830 mm and is not the score. +1.696 mm is still under +2 mm. Both hip rolls hit the plant rail before 7.0 s: left −2.450 Nm at 1.240 s, right +2.450 Nm at 1.496 s. The 2.33 Nm bar does not hold, so this is not a better under-bar toe than −0.013 mm. The physics re-clamp never moved ctrl. The write-time clip shortened 249 of 1400 swing-hip targets. Swing predicted peaks stay under 2.33 Nm. At 2.872 s the right hip pitch ctrl is 0.861765 rad against IK 0.829440 rad, 0.032 rad ahead, predicted and force +0.982 Nm. The right hip roll ctrl is 0.173751 rad against IK 0.182332 rad, behind the current IK, predicted and force −0.271 Nm. At the peak |IK−q| sample, left hip roll at 1.552 s is IK −0.163516 rad, ctrl −0.121860 rad, q −0.027146 rad. The future IK is −0.182875 rad and the clip writes ctrl the other way. Left hip pitch at 3.128 s is IK −0.816631 rad, ctrl −0.850885 rad, future IK −0.858631 rad, so that ctrl is ahead and the clip trims it. No fault. min up_z is 0.934. x at 7.0 s is 0.820 m. After 7.0 s both hip rolls are on ±2.45 Nm and the right knee is +2.45 Nm at 7.424 s. That is Track 2. Prefer FAIL. The 400 ms kit period was not run. A slower period was not run. Not kit-safe. Not go-anywhere.

The 0.600 s and 0.650 s periods were then scored on that same toe-up copy, with no hip lead. dsp stays 0.20. y_swap stays 0.020. Single support at 0.600 s is 0.240 s. The contact-box toe minimum before 7.0 s is +1.411 mm at 1.088 s, fraction 0.207, left foot, front-outside. The lowest corner there is heel-outside, +0.640 mm. The box center is +2.046 mm. Every leg actuator stays under 2.33 Nm before 7.0 s. The largest is the right hip pitch, +1.994 Nm at 1.008 s, and the left hip pitch is −1.971 Nm at the same time. At the toe tick both left hip roll and left hip pitch ctrl equal the current IK. Peak |IK−q| before 7.0 s is the left hip roll at 1.648 s: IK −0.160671, ctrl −0.127677, q −0.029523, force −1.727 Nm. The right hip pitch at 3.224 s has ctrl equal to IK 0.819438 and q 0.740313, force +0.337 Nm. x at 7.0 s is 0.676 m. The 2.872 s sole print on this copy is left, front-outside −0.601 mm, and that tick sits outside the 20–80% window, so it is not this minimum. +1.411 mm holds 2.33 Nm and stays under +2 mm. No fault. min up_z is 0.934.

Single support at 0.650 s is 0.260 s. The contact-box toe minimum before 7.0 s is +3.880 mm at 1.104 s, fraction 0.226, left foot, front-outside. The lowest corner there is heel-outside, +2.682 mm. The box center is +4.444 mm. Sole pitch at that tick is −0.00888 rad. Every leg actuator stays under 2.33 Nm before 7.0 s. The largest is again the right hip pitch, +1.994 Nm at 1.008 s. Peak |IK−q| is the right hip roll at 1.368 s: IK 0.162851, ctrl 0.140966, q 0.039680, force +1.689 Nm. Left hip pitch at 3.696 s has ctrl equal to IK −0.820488 and q −0.745793, force −0.292 Nm. The mid-swing toe after 7.0 s is −0.619 mm at 7.536 s, left foot, and that sample is outside the Track 1 window. x at 7.0 s is 0.621 m. No fault. min up_z is 0.934.

Both period copies raised the toe while every leg stayed under 2.33 Nm before 7.0 s, so the same hip lead was added on the 0.650 s period. The contact-box toe minimum before 7.0 s is +4.328 mm at 1.104 s, fraction 0.226, left foot, front-outside. The lowest corner there is heel-outside, +3.198 mm. The box center is +4.781 mm. Sole pitch is −0.00837 rad. Every leg actuator stays under 2.33 Nm before 7.0 s. The largest is still the right hip pitch, +1.994 Nm at 1.008 s. Peak |IK−q| is the right hip roll at 2.016 s: IK 0.162851, ctrl 0.137449, q 0.036637, force +1.672 Nm. The write clip holds that roll ctrl behind the current IK, 149 of 1416 writes. Left hip pitch at 1.080 s is ahead of the current IK: IK −0.907782, ctrl −0.926119, q −0.853356, and the written ctrl matches the future IK. The mid-swing toe after 7.0 s is −0.987 mm at 7.536 s, left foot, outside the Track 1 window. x at 7.0 s is 0.620 m. No fault. min up_z is 0.934. The predicted clip stays sim-only. The locked walk period stays 0.500 s. Not kit-safe. Not go-anywhere.

Pitch-only hip lead, on that same toe-up copy, at period 0.500 s. Hip pitch is advanced by the compiled lag, 40.23 ms (kv 1.8102, kp 45). Hip roll applied lead is 0.00 ms. The compiled roll lag 42.57 ms is not written. Ankle roll is not led. Foot z is not shifted. dsp stays 0.20. y_swap stays 0.020. The clear is the worst 20–80% contact-box toe on the whole bout, 28 swings, median length 25 ticks, last swing also 25 ticks. That minimum is +0.756 mm at 3.128 s, fraction 0.208, left foot, front-outside. That corner is also the lowest. The box center there is +1.643 mm. The viz bottom is +3.437 mm and is not the score. +0.756 mm is under +2 mm. The 7.560 s window has the same minimum. At that tick the left hip pitch ctrl is −0.858631 rad, equal to the future IK, 0.042 rad ahead of the current IK −0.816631 rad. q is −0.740871 rad. Force is −1.171 Nm. The write clip does not bind on that sample. Peak |IK−q| on hip pitch is at 7.480 s: IK 0.816631, ctrl 0.827316, future IK 0.858631, q 0.723336, force +1.525 Nm. The clip trims that write. 139 of 700 pitch writes are shortened. Hip roll ctrl at 3.128 s equals the current IK −0.182332 rad. There is no roll-lead log.

Whole-bout peaks, hip pitch, knee, and ankle pitch, both legs: left hip pitch +2.035 Nm at 4.400 s, right hip pitch +2.040 Nm at 1.296 s, left knee −2.007 Nm at 7.680 s, right knee +2.363 Nm at 7.432 s, left ankle pitch −1.579 Nm at 7.992 s, right ankle pitch −1.509 Nm at 7.320 s. The right knee is over 2.33 Nm and under the ±2.45 Nm plant rail. The other five stay under 2.33 Nm. Hip rolls are −2.128 Nm at 2.888 s and +2.121 Nm at 3.144 s, under 2.33 Nm. Ankle rolls are −1.780 Nm and −1.596 Nm at 1.016 s. No fault. min up_z is 0.934. x at 7.0 s is 0.812 m. Prefer FAIL. The toe is short of +2 mm and the right knee is over 2.33 Nm. The half-roll lead was not run. The extra foot-z lift was not run. Period stays 0.500 s. Not kit-safe. Not go-anywhere.

The same tick on the no-lead toe-up copy, 7.432 s, is left knee −1.348 Nm and right knee +1.015 Nm, both under 2.33 Nm and under ±2.45 Nm. The 40.23 ms pitch-lead copy at that tick is left knee −0.544 Nm and right knee +2.363 Nm. Both feet have two contact-box corners inside the rug box, and the leading corner is inside. Contact geoms are floor. Rug normal is 0.00 N. Leading-corner height against the 12.0 mm rug top is 14.029 mm left and 35.231 mm right on the no-lead copy, and 29.676 mm left and 24.354 mm right on the pitch-lead copy. The feet are over the rug in plan and the load is on the floor geom. The +2.363 Nm is the pitch lead at that sample. The no-lead left knee reaches −2.450 Nm at 8.200 s on rug-tagged ticks, on the plant rail. That is a later sample.

Each of the 28 mid-swings is tagged flat or rug. A swing is rug when any 20–80% sample has a contact-box corner inside the rug box or rug normal above 0.5 N. Rug clearance is the leading-corner z minus the 12 mm rug top. Track 1 uses the flat-tagged swings only. On the no-lead copy, 23 swings are flat and 5 are rug. The flat worst is +0.025 mm at 2.872 s, right foot, on the floor. The rug worst clearance is −8.517 mm at 7.224 s, left foot, absolute toe +3.483 mm. The whole-walk minimum −0.013 mm at 6.968 s sits on swing 23, which is tagged rug. That tick's own surface is floor, corners inside 0, floor normal 0.1 N, rug normal 0 N. On the 40.23 ms copy, 24 swings are flat and 4 are rug. The flat worst is +0.756 mm at 3.128 s, left foot, on the floor, corners inside 0. The rug worst clearance is +2.536 mm at 7.992 s, right foot, absolute toe +14.536 mm. Flat-floor hip pitch, knee, and ankle pitch stay under 2.33 Nm. The +2.363 Nm right knee is the 7.432 s sample over the rug in plan.

Shorter pitch leads at period 0.500 s, roll lead 0, on the same toe-up copy. 30 ms flat toe +0.750 mm at 3.128 s, left, rug clearance +2.959 mm, right knee at 7.432 s +2.349 Nm. 20 ms flat toe +0.764 mm at 2.872 s, right, rug clearance +3.988 mm, right knee at 7.432 s +2.263 Nm, under 2.33 Nm. 10 ms flat toe +0.662 mm at 2.872 s, right, rug clearance +4.285 mm, right knee at 7.432 s +2.178 Nm, under 2.33 Nm. 35 ms flat toe +0.745 mm at 3.128 s, left, rug clearance +2.522 mm, right knee at 7.432 s +2.354 Nm. Every one of these holds hip pitch, knee, and ankle pitch at or under 2.33 Nm while that foot is off the rug. The best flat toe is the 20 ms copy at +0.764 mm, 0.008 mm above the 40.23 ms flat toe, still under +2 mm. The 10 ms copy leaves the flat toe at +0.662 mm. Dropping the lead leaves the flat toe at +0.025 mm. Prefer FAIL on the flat toe. Contact geoms at 7.432 s are floor, and the right knee there is +2.363 Nm, so that sample retires the 40.23 ms lead. The base is the 20 ms pitch lead. Rug swings stay out of the Track 1 toe. The half-roll lead was not run. Period stays 0.500 s. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

The 20 ms pitch lead is the base. Toe-up stays 0.020 rad. Roll lead stays 0. Period stays 0.500 s. On that copy the worst flat mid-swing is +0.764 mm at 2.872 s, right foot. The box centre there is +1.551 mm. Sole pitch is +0.00600 rad, toe-down. The leading corner sits 0.788 mm under the centre. The foot-z minimum is +2.000 − 1.551 = +0.449 mm. The sweep adds that on the swing foot for the whole single support, on top of the 12 mm phase bump, then steps up by 0.4 mm: +0.449, +0.849, +1.249, +1.649, and +2.049 mm. At 2.872 s the logged z add matches each request.

Every copy holds hip pitch, knee, and ankle pitch under 2.33 Nm on flat-foot ticks. Both knees and both ankle pitches were checked on all 1025 ticks. The samples at or over 2.33 Nm were tagged from the plan box. Contact geoms at those ticks are floor, and the rug normal is 0.00 N, so they are Track 1. The right knee is +2.450 Nm at 7.432 s on the +1.249 mm copy, +2.443 Nm at 7.432 s on the +1.649 mm copy, and +2.336 Nm at 7.424 s plus +2.450 Nm at 7.432 s on the +2.049 mm copy. The +0.449 mm and +0.849 mm copies have none. The foot-z chase stops at +0.849 mm.

The world box centre rises by less than the command, and the body drops. At +0.449 mm the toe is +0.957 mm (centre +0.137 mm, body −0.144 mm, sag 0.169 mm). At +0.849 mm the toe is +1.191 mm (centre +0.357 mm, body −0.184 mm, sag 0.308 mm). At +1.249 mm the toe is +1.280 mm (centre +0.439 mm, body −0.275 mm, sag 0.534 mm). At +1.649 mm the toe is +1.432 mm (centre +0.551 mm, body −0.365 mm, sag 0.733 mm). At +2.049 mm the toe is +1.582 mm, the centre is +2.201 mm, the body is 0.472 mm lower, sole pitch is +0.00550 rad, and the split of the +2.049 mm command is +0.650 mm in the world centre, 0.472 mm of body drop, and 0.927 mm of sag. Flat left knee on that copy is −2.033 Nm at 1.544 s. Flat ankle pitch stays at −1.260 Nm and +1.260 Nm. Prefer FAIL. The flat toe stays under +2 mm. Pitch lead stays 20 ms. Toe-up stays 0.020 rad. Period stays 0.500 s. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Contact geoms tag every tick at or over 2.33 Nm. A foot over the rug in plan whose contact geom is floor counts as Track 1. On the 20 ms base that retag leaves the right knee at +2.263 Nm at 7.432 s, contact floor, under 2.33 Nm. Hip pitch, knee, and ankle pitch on both legs were checked on 1025 ticks. None is over 2.33 Nm on floor contact.

Stance-knee sag cancel is sim-only. Each walking tick adds (qfrc_bias − qfrc_constraint) / compiled kp from the previous tick to the stance knee, then clamps predicted force to ±2.33 Nm. Compiled knee kp is 45, kv 1.4573. Measured actuator_force is not the offset. The plant file is not edited. Knee drop in millimetres is that offset times the pelvis-fixed vertical jacobian of the foot contact geom, set beside the 0.927 mm sag share.

With no extra foot-z the flat toe is +0.099 mm at 2.616 s, left foot. Box centre +0.833 mm. Sole pitch +0.00309 rad, toe-down. That toe is 1.360 mm under the base sample at the same tick, the centre is 0.899 mm lower, and the body is 0.158 mm lower. Floor-contact peaks: left hip pitch +2.123 Nm at 3.888 s, right hip pitch −2.125 Nm at 5.168 s, left knee −1.781 Nm at 1.536 s, right knee +1.991 Nm at 7.928 s, left ankle pitch −1.583 Nm at 2.864 s, right ankle pitch +1.570 Nm at 4.144 s. All under 2.33 Nm. 1025 ticks, no over. The formula's knee drop peaks at −0.962 mm at 1.792 s on the right knee. Median absolute drop is 0.393 mm and the 90th is 0.827 mm. At the flat toe tick the formula is −0.393 mm (bias −0.113 Nm, constraint −0.657 Nm) and the predicted force is on the +2.330 Nm clamp, so the ctrl change is not that offset. 345 of 1100 stance writes hit the clamp.

The last under-bar foot-z add, +0.849 mm, on the same cancel leaves the flat toe at +0.440 mm at 2.616 s, left foot. Box centre +1.115 mm. Sole pitch +0.00297 rad. Floor-contact peaks stay under 2.33 Nm: hip pitch +2.071 Nm and −2.074 Nm, knees −1.773 Nm and +2.130 Nm, ankle pitch −1.587 Nm and +1.602 Nm. Formula peak −0.954 mm. At that toe tick the formula is −0.415 mm and predicted force is +2.330 Nm.

Prefer FAIL. The flat toe stays under +2 mm. The load offset is about the size of the 0.927 mm sag share, and the clamp leaves no room to apply it. Pitch lead stays 20 ms. Toe-up stays 0.020 rad. Period stays 0.500 s. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Sag cancel stays off. The miss is lift-off timing. On every copy below, all 28 swings put their 20–80% contact-box minimum at cycle fraction 0.208, the first tick inside that window. The window was not moved. Base is the 20 ms pitch lead, roll lead 0, period 0.500 s, swing-hip predicted clip, hip pitch off the 20 ms approach, soft-pass off, plant cold. Toe at every 0–30% tick of each flat swing was logged (192 ticks, 24 flat swings). Both knees and the swing ankle pitch were scored against 2.33 Nm on the whole walk, tagged by contact geom.

A ramps the same 0.020 rad toe-up to full by 15% of single support and holds it through 80%, instead of a sine that is still rising at toe-off. Flat worst is +1.128 mm at 3.128 s, left, fraction 0.208. Box centre +1.772 mm. Sole pitch +0.00470 rad. On that swing the 0–30% toes are −2.291 mm at fraction 0.083 (floor), −1.875 mm at 0.125 (floor), −0.722 mm at 0.167 (floor), +1.128 mm at 0.208, and +3.496 mm at 0.250. Floor-contact peaks stay under 2.33 Nm: hip pitch +2.015 Nm and +2.034 Nm, knees −1.968 Nm and +2.277 Nm, ankle pitch −1.260 Nm and −1.528 Nm. The swing ankle pitch peaks at −1.528 Nm. 1025 ticks, no over. Short of +2 mm by 0.872 mm.

B replaces the 12 mm parabola with a smoothstep of the same 12 mm peak, full by 10% of single support, held through 40%, down by 80%, from toe-off. Flat worst is +0.673 mm at 2.872 s, right, fraction 0.208. Box centre +1.399 mm. Sole pitch +0.00390 rad. That is under the 20 ms base at +0.764 mm. On that swing the toe is still on the floor at fraction 0.167 (−1.069 mm) and +0.673 mm at 0.208. Floor-contact peaks stay under 2.33 Nm: hip pitch −1.971 Nm and +2.048 Nm, knees −2.001 Nm and +2.171 Nm, ankle pitch −1.260 Nm and −1.593 Nm. Swing ankle pitch −1.593 Nm.

C is A and B together. Flat worst is +1.026 mm at 2.872 s, right, fraction 0.208. Box centre +1.565 mm. Sole pitch +0.00175 rad. Floor-contact peaks stay under 2.33 Nm: hip pitch −1.971 Nm and +2.042 Nm, knees −2.002 Nm and +2.158 Nm, ankle pitch −1.260 Nm and −1.692 Nm. Swing ankle pitch −1.692 Nm.

D leads the swing knee only, because A–C missed +2 mm. One control tick (8 ms) leaves the flat toe at +0.624 mm at 3.128 s, left, fraction 0.208, and puts the right knee on the plant rail, +2.450 Nm at 7.432 s, contact geom floor. 398 of 700 swing-knee writes were clamped to the predicted ±2.33 Nm. Two ticks (16 ms) leave the flat toe at +0.680 mm and the right knee at +2.449 Nm at 7.432 s, contact floor. 378 of 700 writes clamped. Swing ankle pitch stays at −1.326 Nm and −1.323 Nm. Stance knees were not offset.

E replaces the 12 mm parabola with an 8 mm and a 10 mm smoothstep, full by 10% and held through 40%, on the current clip and the 20 ms lead. The earlier −1.60 mm and the right-knee +2.45 Nm were before that clip and that lead. At 8 mm the flat toe is −0.710 mm at 2.872 s, right, fraction 0.208, still on the floor (1.9 N). Box centre +0.377 mm. Floor knees are −1.995 Nm and +1.960 Nm, both under 2.33 Nm. The knee does not rail. Swing ankle pitch peaks at −1.078 Nm. A left hip pitch of +2.347 Nm at 7.512 s is contact geom rug, not Track 1. At 10 mm the flat toe is +0.092 mm at 2.872 s, right, fraction 0.208. Box centre +1.012 mm. The right knee is +2.434 Nm at 7.432 s, contact geom floor, over 2.33 Nm and under ±2.45 Nm. That rail is Track 1. Swing ankle pitch peaks at −1.519 Nm.

Prefer FAIL. The best flat toe that holds hip pitch, knee, and ankle pitch under 2.33 Nm on floor contact is A at +1.128 mm, 0.872 mm short of +2 mm. Every scored minimum is still the first 20–80% tick. Sag cancel stayed off. Pitch lead stays 20 ms. Toe-up peak stays 0.020 rad. Period stays 0.500 s. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Earlier toe-up on that same 20 ms base, sag cancel off. The 20–80% window stays. Each 0–30% tick of each flat swing logs the swing-foot normal, floor plus rug, and the MuJoCo contact dist. Negative dist is penetration. A10 is 0.020 rad full by 10% and held through 80%. A05 is that peak full by 5%. A15+025 is 0.025 rad full by 15%. A10+025 is 0.025 rad full by 10%. The 12 mm phase lift stays. All 24 flat swings on every copy still put the 20–80% minimum at fraction 0.208. 1025 ticks, and no pitch-chain sample is over 2.33 Nm on floor contact.

A10 flat worst is +1.199 mm at 3.128 s, left. Box centre +1.780 mm. Sole pitch +0.00391 rad. Floor knees −1.969 Nm and +2.175 Nm. Swing ankle pitch −1.508 Nm. A05 is +1.311 mm at the same tick, box centre +1.857 mm, sole pitch +0.00370 rad, floor knees −1.966 Nm and +2.155 Nm, swing ankle pitch −1.455 Nm. A15+025 is +1.239 mm, box centre +1.788 mm, sole pitch +0.00344 rad, floor knees −1.969 Nm and +2.222 Nm, swing ankle pitch −1.568 Nm. A10+025 is +1.385 mm, box centre +1.865 mm, sole pitch +0.00295 rad, floor knees −1.965 Nm and +2.192 Nm, swing ankle pitch −1.570 Nm. Short of +2 mm by 0.615 mm. On that swing fraction 0.167 is still a loaded contact: toe −0.547 mm, normal 0.807 N, dist −0.547 mm, contact floor. At 0.208 the toe is +1.385 mm, normal 0 N, dist none, contact none. At 0.250 it is +3.847 mm and still unloaded. The scored tick is unload. The soft-contact spring-back is the tick before the window.

Those four stay short of +2 mm under 2.33 Nm with every flat minimum at 0.208, so y_swap is sampled 8 ms and 16 ms earlier on A10+025. Amplitude stays 0.020 m. The shift still adds the same lat to both ankle rolls. The measured |(left ctrl − stand) − (right ctrl − stand)| is 0.0495 rad at 8 ms and 0.0493 rad at 16 ms. At 8 ms the flat toe is −0.183 mm at 2.360 s, right, box centre +0.612 mm, contact floor, normal 0.000 N, dist −0.191 mm. The right knee is +2.443 Nm at 7.424 s, contact floor, over 2.33 Nm and under ±2.45 Nm. Floor hip rolls are −2.230 Nm at 2.376 s and +2.159 Nm at 3.648 s, both under 2.33 Nm. At 16 ms the flat toe is −0.068 mm at 2.360 s, right, box centre +0.760 mm, contact floor, normal 0.021 N, dist −0.141 mm. The right knee is +2.439 Nm at 7.424 s, contact floor. Floor hip rolls are −2.203 Nm at 3.392 s and +2.220 Nm at 2.120 s, both under 2.33 Nm.

Prefer FAIL. The best flat toe that holds the pitch chain under 2.33 Nm on floor contact is A10+025 at +1.385 mm, 0.615 mm short of +2 mm. Earlier y_swap drops the toe below 0 and puts the right knee over 2.33 Nm on floor contact. Hip rolls stay under 2.33 Nm. Every scored minimum is still fraction 0.208. Sag cancel stayed off. Pitch lead stays 20 ms. Period stays 0.500 s. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Sole-roll level on that same A10+025 copy, before any less-crouch stance. A10+030, A05+025, and A00+025 were not run. y_swap was not re-run. Sag cancel stayed off. Roll lead stays 0. The trim gate is a centre minus front-outside drop between 0.50 and 0.70 mm at fraction 0.208 on the worst flat swing. Outside that window the swing ankle-roll trim is not run.

On the worst flat swing, left at 3.128 s, fraction 0.208, the toe is +1.385 mm. Box centre +1.865 mm. Sole pitch +0.00295 rad. Sole roll about the long axis is −0.00740 rad. The inside edge is local −y and 0.281 mm high (38 mm × 0.00740 rad). The scored corner and the lowest corner are both front-outside at +1.385 mm. Centre minus that corner is +0.480 mm. Front-inside is +1.947 mm (centre minus corner −0.082 mm). Heel-outside is +1.784 mm (+0.082 mm). Heel-inside is +2.346 mm (−0.480 mm). +0.480 mm is outside 0.50–0.70 mm.

The right foot's own worst flat tick is 2.872 s, fraction 0.208. Sole roll +0.00644 rad. Centre +1.861 mm. Front-outside +1.506 mm, centre minus corner +0.354 mm, and that corner is both the scored corner and the lowest. Inside edge 0.245 mm high. That drop is outside the window too.

Floor-contact knees are −1.965 Nm at 2.056 s and +2.192 Nm at 7.432 s. Floor-contact ankle rolls are −1.780 Nm and −1.596 Nm at 1.016 s. All four are under 2.33 Nm. 1025 ticks, and no pitch-chain or ankle-roll sample is over 2.33 Nm. All 24 flat minima stay at fraction 0.208.

Prefer FAIL. The centre minus front-outside drop is +0.480 mm, not about 0.6 mm, so the swing ankle-roll trim was not run. Less-crouch was not run. Pitch lead stays 20 ms. Period stays 0.500 s. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Combined level trim on that same A10+025 copy, swing ankle only, before any extra foot-z. The add is the leftover sole at fraction 0.208. It is not a world-level sole and it does not cancel body roll. Each add is capped at ±0.025 rad. The sign check did not flip either axis. Ankle-roll adds are opposite and matched to the left foot, +0.00662 rad and −0.00662 rad. Pitch stays per foot, +0.00210 rad left and −0.00069 rad right. One kinematic tick per foot was printed before the sweep.

On the qpos replay the left centre rises from +2.352 mm to +2.467 mm, +0.115 mm, against an expectation of +0.136 mm (11 mm times the toe-down pitch plus 14 mm times |roll|). After the roll magnitudes match, the right centre rises from +2.347 mm to +2.447 mm, +0.100 mm, against +0.108 mm. That replay sets the joint and is not the walking toe. The walking sole is not driven to zero.

On the walk the level trim alone leaves the flat worst at +1.830 mm at 2.872 s, right, 0.170 mm short of +2 mm. Box centre +2.001 mm. Sole roll +0.00185 rad. Sole pitch +0.00150 rad. The scored corner and the lowest corner are both front-outside at +1.830 mm. Centre minus that corner is +0.171 mm. Front-inside is +1.971 mm (+0.030 mm). Heel-outside is +2.032 mm (−0.030 mm). Heel-inside is +2.172 mm (−0.171 mm). The right centre rose +0.140 mm from +1.861 mm. The expectation at the measured right angles was +0.108 mm. The left foot at 3.128 s rose from centre +1.865 mm to +2.096 mm, +0.231 mm, against an expectation of +0.136 mm. Its toe is +1.879 mm. Sole roll −0.00238 rad. Sole pitch +0.00188 rad. Front-outside +1.879 mm, centre minus corner +0.217 mm. Front-inside +2.060 mm (+0.036 mm). Heel-outside +2.133 mm (−0.036 mm). Heel-inside +2.314 mm (−0.217 mm).

Floor-contact knees are −1.973 Nm at 2.056 s and +2.103 Nm at 7.432 s. Right ankle pitch is −1.477 Nm at 7.456 s, contact floor. The flat-bucket left ankle pitch is −1.271 Nm at 7.768 s, contact none. Floor-contact ankle rolls are −1.780 Nm and −1.596 Nm at 1.016 s. All under 2.33 Nm. 1025 ticks, and no pitch-chain or ankle-roll sample is over 2.33 Nm.

The shortfall to +2 mm is 0.170 mm, so that command was added on the swing foot on top of the level trim. The old +0.4 mm was not carried in. Flat worst +1.859 mm at 2.872 s, right, still 0.141 mm short. Box centre +2.036 mm. Sole roll +0.00193 rad. Sole pitch +0.00155 rad. Front-outside +1.859 mm, centre minus corner +0.178 mm, scored and lowest. Front-inside +2.005 mm (+0.031 mm). Heel-outside +2.067 mm (−0.031 mm). Heel-inside +2.214 mm (−0.178 mm). The left foot at 3.128 s is +1.930 mm, centre +2.152 mm, front-outside drop +0.222 mm. Floor knees are −1.970 Nm at 2.056 s and +2.093 Nm at 7.432 s. Ankle pitch is −1.273 Nm at 7.768 s, contact none, and −1.475 Nm at 7.456 s, contact floor. Ankle rolls stay −1.780 Nm and −1.596 Nm. All under 2.33 Nm. 1025 ticks, no over. The floor knee did not cross 2.33 Nm, so less-crouch was not started and init_z_offset was not invented.

Prefer FAIL. +2 mm did not clear under 2.33 Nm. Pitch lead stays 20 ms. Period stays 0.500 s. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Trim lead on that same A10+025 stack, swing ankle only, with the 0.170 mm foot-z still on. The lead is only the leftover sole add. The toe-up, the IK ankle, and the hip are not led. Hip pitch lead stays 20 ms. Roll lead stays 0. Compiled ankle roll is kp 35, kv 1.1876, lag 33.93 ms. Compiled ankle pitch is kp 35, kv 1.1980, lag 34.23 ms. The gate opens at gait fraction 0.235000, the cycle-0.208333 sample on both feet. Trim weight is 0 before that fraction, so the lookahead does not start the add on a loaded foot. The schedule is 0 by 0.92 of swing, before touchdown. Cap stays ±0.025 rad. The sign check did not flip. Roll adds match at +0.00641 rad and −0.00641 rad. Pitch stays per foot, +0.00217 rad and −0.00040 rad.

At cycle fraction 0.208 the leftover sole is not under 0.0005 rad. Worst flat tick, left at 3.128 s: sole roll −0.00703847 rad, sole pitch +0.00301729 rad, box centre +1.923767 mm. Scored and lowest corner is front-outside at +1.452646 mm. Centre minus that corner is +0.471120 mm. Front-inside +1.987563 mm (centre minus corner −0.063797 mm). Heel-outside +1.859970 mm (+0.063797 mm). Heel-inside +2.394887 mm (−0.471120 mm). Flat toe +1.452646 mm. Raw gap to +2 mm is 0.547354 mm. The floored hundredth is 0.54 mm. Against the untrimmed 0.170 mm copy, left ankle-roll ctrl moved +0.00641 rad, equal to the add, and left ankle-pitch ctrl moved +0.00087 rad against an add of +0.00217 rad. The right foot at 2.872 s is sole roll +0.00594882 rad, sole pitch +0.00137190 rad, centre +1.939363 mm, front-outside +1.620707 mm, raw gap 0.379293 mm, floored hundredth 0.37 mm. Right ctrl moved −0.00645 rad in roll against an add of −0.00641 rad, and −0.00016 rad in pitch against an add of −0.00040 rad. Floor knees are −1.9641 Nm at 7.688 s, contact floor+rug, and +2.1725 Nm at 7.432 s, contact floor. Ankle pitch is +1.3133 Nm at 7.568 s, contact floor+rug, and −1.5540 Nm at 7.456 s, contact floor. Ankle rolls are −1.7796 Nm and −1.5959 Nm at 1.016 s, contact floor. All under 2.33 Nm.

The lead is still short of +2 mm and no floor knee or ankle is over 2.33 Nm, so foot-z continues from 0.270 mm in 0.100 mm steps with the same lead. Scored corner is front-outside on every step. Ankle rolls stay −1.7796 Nm and −1.5959 Nm.

| Foot-z | Side | Toe mm | Centre mm | Front-outside | Front-inside | Heel-outside | Heel-inside | Left knee | Right knee | Left ankle pitch | Right ankle pitch |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.270 mm | L 3.128 s | +1.618127 | +2.033155 | +1.618127 | +2.138614 | +1.927697 | +2.448184 | −2.0371 floor+rug | +2.1998 floor | +1.3041 floor+rug | −1.5918 floor |
| 0.370 mm | L 3.128 s | +1.646342 | +2.057778 | +1.646342 | +2.160446 | +1.955110 | +2.469214 | −2.0420 floor+rug | +2.1987 floor | +1.3038 floor+rug | −1.5919 floor |
| 0.470 mm | L 3.128 s | +1.688142 | +2.104227 | +1.688142 | +2.218757 | +1.989698 | +2.520313 | −2.0473 floor+rug | +2.1873 floor | +1.3123 floor+rug | −1.5811 floor |
| 0.570 mm | L 3.128 s | +1.733332 | +2.137409 | +1.733332 | +2.240781 | +2.034036 | +2.541485 | −2.0590 floor+rug | +2.1901 floor | +1.3032 floor+rug | −1.5820 floor |
| 0.670 mm | L 3.128 s | +1.756146 | +2.158100 | +1.756146 | +2.261837 | +2.054363 | +2.560054 | −2.0934 floor+rug | +2.2042 floor | +1.3090 floor+rug | −1.5969 floor |
| 0.770 mm | L 3.128 s | +1.775716 | +2.176663 | +1.775716 | +2.282680 | +2.070646 | +2.577610 | −2.1021 floor+rug | +2.2018 floor | +1.3029 floor+rug | −1.5908 floor |
| 0.870 mm | R 2.872 s | +1.778944 | +2.095062 | +1.778944 | +2.232206 | +1.957917 | +2.411179 | −2.0981 floor+rug | +2.1918 floor | +1.3085 floor+rug | −1.5959 floor |
| 0.970 mm | R 2.872 s | +1.822531 | +2.123833 | +1.822531 | +2.254146 | +1.993520 | +2.425134 | −2.1403 floor+rug | +2.1943 floor | +1.3029 floor+rug | −1.5930 floor |
| 1.070 mm | L 3.128 s | +1.816930 | +2.204340 | +1.816930 | +2.316062 | +2.092619 | +2.591750 | −2.1411 floor+rug | +2.2341 floor | +1.3013 floor+rug | −1.5937 floor |
| 1.170 mm | L 3.128 s | +1.853298 | +2.228273 | +1.853298 | +2.336712 | +2.119833 | +2.603248 | −2.1469 floor+rug | +2.2336 floor | +1.3049 floor+rug | −1.5909 floor |
| 1.270 mm | L 3.128 s | +1.884446 | +2.241661 | +1.884446 | +2.367823 | +2.115499 | +2.598875 | −2.2676 floor+rug | +2.4300 floor | +1.2885 floor+rug | −1.6257 floor |

The 1.270 mm step is the first floor knee or ankle over 2.33 Nm: right knee +2.4300 Nm at 7.432 s, contact floor. The sweep stops there. Flat toe +1.884446 mm, raw gap 0.115554 mm, floored hundredth 0.11 mm. Ankle pitch and both ankle rolls stay under 2.33 Nm. The last step under 2.33 Nm is 1.170 mm, flat toe +1.853298 mm, raw gap 0.146702 mm, floored hundredth 0.14 mm. Both the lead and the foot-z sweep stalled short of +2 mm. Less-crouch is next. init_z_offset was not invented. Less-crouch was not run.

Prefer FAIL. +2 mm did not clear under 2.33 Nm. Pitch lead stays 20 ms. Period stays 0.500 s. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

The gated trim lead stays off. This copy is the restored constant level trim plus foot-z 1.170 mm on A10+025. Roll adds are +0.00662 rad left and −0.00662 rad right. Pitch adds are +0.00210 rad left and −0.00069 rad right. Both ankle-roll axes are +1 0 0, ctrlrange ±2.090 rad. Left ankle pitch axis is 0 −1 0 and right is 0 +1 0, same ctrlrange. At cycle fraction 0.208333 the worst flat tick is the left foot at 3.128 s. Sole roll −0.00167885 rad. Sole pitch +0.00151348 rad. Box centre +2.387138 mm. Front-outside +2.221182 mm is the scored corner and the lowest. Centre minus that corner is +0.165956 mm. Front-inside +2.348775 mm, centre minus corner +0.038364 mm. Heel-outside +2.425502 mm, centre minus corner −0.038364 mm. Heel-inside +2.553094 mm, centre minus corner −0.165956 mm. Flat toe +2.221182 mm. Raw gap 2 − toe is −0.221182 mm. The scorer floors that signed gap to −0.23. The surplus to a hundredth, without raising the toe, is 0.22 mm. The MFG +2.15 mm figure is a check. Measured toe minus that guess is +0.071182 mm. The tilt check 67.5*|pitch| + 38*|roll| is 0.165956 mm and equals the centre-minus-front-outside drop. The other flat 0.208 tick is the right foot at 2.872 s: sole roll +0.00037182 rad, sole pitch +0.00031265 rad, centre +2.417606 mm, front-outside +2.382373 mm (centre minus corner +0.035233 mm), front-inside +2.410631 mm (+0.006975 mm), heel-outside +2.424581 mm (−0.006975 mm), heel-inside +2.452839 mm (−0.035233 mm). Floor knees on the flat bar are −2.0549 Nm at 7.680 s and +2.1404 Nm at 7.432 s, both contact floor. The floor+rug left knee is −2.1106 Nm at 7.688 s. Right ankle pitch −1.5574 Nm at 7.456 s, contact floor. Left ankle pitch on floor or none is −1.2889 Nm at 7.768 s, contact none. The floor+rug left ankle pitch is +1.3146 Nm at 7.576 s. Ankle rolls −1.7796 Nm and −1.5959 Nm at 1.016 s, contact floor. All of those stay under 2.33 Nm. The flat toe clears +2 mm with the pitch chain and both ankle rolls at or under 2.33 Nm on floor or none. Derivative trim lead was not run. Less-crouch was not opened. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Steer whole-walk on that same stack, trim lead off, less-crouch closed. Stand, straight through 8.2 s, stop, yaw left at +0.25 rad/s, stop, yaw right at −0.25 rad/s, stop, end 21.6 s. The left segment's applied yaw reaches +0.25000 rad/s. The right segment's applied yaw reaches −0.25000 rad/s. The yaw_seen line stores the absolute peak, so both print +0.25000. No bus fault. The whole-walk 20–80% left toe is +1.608609 mm at 17.728 s, cycle fraction 0.208333, yaw_right, yaw −0.25000, phase swing, contact none. Scored corner and lowest corner are front-outside. Centre +2.102492 mm. Front-outside +1.608609 mm, centre minus corner +0.493883 mm. Front-inside +2.106890 mm, centre minus corner −0.004398 mm. Heel-outside +2.098093 mm, centre minus corner +0.004398 mm. Heel-inside +2.596375 mm, centre minus corner −0.493883 mm. Raw gap 2 − toe is +0.391391 mm. The scorer floors that gap to 0.39 mm. Lift-off of that swing is 17.688 s, fraction 0.000000, toe −0.754636 mm, contact floor, geoms floor. Scored front-outside, lowest front-inside. Centre −1.252968 mm. Front-outside −0.754636 mm, centre minus corner −0.498332 mm. Front-inside −1.917529 mm, centre minus corner +0.664561 mm. Heel-outside −0.588408 mm, centre minus corner −0.664561 mm. Heel-inside −1.751301 mm, centre minus corner +0.498332 mm. The whole-walk 20–80% right toe is −1.162939 mm at 14.328 s, fraction 0.250000, yaw_left, yaw +0.25000, phase swing, contact floor, geoms floor. Scored corner and lowest corner are front-inside. Centre −0.708646 mm. Front-outside −1.158787 mm, centre minus corner +0.450140 mm. Front-inside −1.162939 mm, centre minus corner +0.454293 mm. Heel-outside −0.254354 mm, centre minus corner −0.454293 mm. Heel-inside −0.258506 mm, centre minus corner −0.450140 mm. Raw gap 2 − toe is +3.162939 mm. Floored hundredth 3.16 mm. Lift-off of that swing is 14.304 s, fraction 0.000000, toe −0.854463 mm, contact floor. Scored front-inside, lowest front-outside. Centre −1.247810 mm. Front-outside −1.886326 mm, centre minus corner +0.638516 mm. Front-inside −0.854463 mm, centre minus corner −0.393347 mm. Heel-outside −1.641157 mm, centre minus corner +0.393347 mm. Heel-inside −0.609294 mm, centre minus corner −0.638516 mm. Rug clearance at the toe-min tick is +1.767310 mm at 11.784 s, right, fraction 0.208333, yaw_left, yaw +0.25000. Absolute toe +13.767310 mm. Surface rug, contact none, on_rug 1. Scored and lowest front-outside. Centre +14.178412 mm. Front-outside +13.767310 mm, centre minus corner +0.411102 mm. Front-inside +14.236270 mm, centre minus corner −0.057858 mm. Heel-outside +14.120554 mm, centre minus corner +0.057858 mm. Heel-inside +14.589514 mm, centre minus corner −0.411102 mm. Raw gap 2 − clearance is +0.232690 mm. Floored hundredth 0.23 mm. Lift-off of that rug swing is 11.744 s, toe +11.160744 mm, clearance −0.839256 mm, contact rug, geoms col_mat_rug. Scored front-outside, lowest front-inside. Centre +10.754460 mm. Front-outside +11.160744 mm, centre minus corner −0.406284 mm. Front-inside +10.064808 mm, centre minus corner +0.689652 mm. Heel-outside +11.444112 mm, centre minus corner −0.689652 mm. Heel-inside +10.348176 mm, centre minus corner +0.406284 mm. Floor or floor+rug peaks: left knee +2.2800 Nm at 20.608 s, contact floor, stop, raw headroom 0.050000 Nm, hundredths 0.05. Right knee +2.3375 Nm at 12.952 s, contact floor, yaw_left, raw headroom −0.007515 Nm, hundredths −0.01. Left ankle pitch −2.1722 Nm at 12.504 s, floor+rug, yaw_left, raw headroom 0.157840 Nm, hundredths 0.15. Right ankle pitch +2.2013 Nm at 12.560 s, floor, yaw_left, raw headroom 0.128670 Nm, hundredths 0.12. Left ankle roll −2.2800 Nm at 20.608 s, floor, stop, raw headroom 0.050000 Nm, hundredths 0.05. Right ankle roll +1.9738 Nm at 12.968 s, floor, yaw_left, raw headroom 0.356178 Nm, hundredths 0.35. Left hip roll −2.2364 Nm at 14.336 s, floor, yaw_left, raw headroom 0.093640 Nm, hundredths 0.09. Right hip roll +2.2430 Nm at 17.720 s, floor, yaw_right, raw headroom 0.087013 Nm, hundredths 0.08. The 7.432 s right knee is still +2.1404 Nm. It is not the floor peak on this bout. Unclamped turning asks, kp*(q_des−q)−kv*ω, before the yaw budget: right hip roll +4.2372 Nm at 15.872 s, q_des +0.14078 rad, yaw −0.11200 rad/s, phase shift, force_limited, limit 2.33 Nm, measured +1.7631 Nm, contact floor, raw headroom −1.907176 Nm, hundredths −1.91. Right knee −6.3249 Nm at 13.296 s, q_des −1.35402 rad, yaw +0.25000, phase swing, clipped, no force limit, measured −1.8075 Nm, contact floor, raw headroom −3.994887 Nm, hundredths −4.00. Left hip roll −4.3120 Nm at 16.128 s, q_des −0.14244 rad, yaw −0.21440, phase shift, force_limited, limit 2.33 Nm, measured −1.7914 Nm, contact floor, raw headroom −1.982033 Nm, hundredths −1.99. Left knee +6.2326 Nm at 13.040 s, q_des +1.35408 rad, yaw +0.25000, phase swing, clipped, measured +1.7398 Nm, contact rug, raw headroom −3.902578 Nm, hundredths −3.91. Later-stop unclamped asks, limit 2.28 Nm: left hip roll +1.2543 Nm at 8.200 s, measured −0.6646 Nm, contact rug, raw headroom 1.075736 Nm, hundredths 1.07. Right hip roll −5.6472 Nm at 20.600 s, measured −0.1155 Nm, contact none, raw headroom −3.317244 Nm, hundredths −3.32. Left knee −4.7885 Nm at 8.200 s, measured −1.4143 Nm, contact rug, raw headroom −2.458509 Nm, hundredths −2.46. Right knee +13.0915 Nm at 14.400 s, measured +0.2788 Nm, contact none, raw headroom −10.761549 Nm, hundredths −10.77. The measured 2.2800 Nm peaks are that stop clamp. Prefer FAIL. Both whole-walk toes are under +2 mm, the rug clearance is under +2 mm, the floor right knee is over 2.33 Nm, and the unclamped hip-roll and knee asks on the turns and on the later stops are over 2.33 Nm. Trim lead stayed off. Less-crouch was not opened. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Command ramp on that same stack. Soft-pass stays off. Trim lead stays off. Less-crouch stays closed. Foot-z stays 1.170 mm. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. On the snap bout the right toe at 14.328 s is -1.162939 mm, fraction 0.250000, yaw +0.25000. Centre -0.708646 mm. Sole roll -0.00005 rad. Sole pitch +0.00670 rad, toe-down. Front-outside -1.158787 mm, centre minus corner +0.450140 mm. Front-inside -1.162939 mm, +0.454293 mm, scored and lowest. Heel-outside -0.254354 mm. Heel-inside -0.258506 mm. The front edge is the low edge. Roll is about zero, so the tilt is pitch, and the centre is already under the floor. Right knee at 12.952 s, contact floor, yaw_left: actuator force +2.3375 Nm, joint damping force -0.1003 Nm (plant damping 0.0800 times omega +1.2534 rad/s, sign -damping*omega), sum +2.2372 Nm, qfrc_passive -0.0996 Nm. The actuator is over 2.33 Nm, raw headroom -0.007515 Nm, hundredths -0.01. The sum is under 2.33 Nm. The actuator still counts. This tick is not a 2.280 Nm clamp plus a 0.056 Nm damper. Left hip roll at 14.336 s is actuator -2.2364 Nm and damping -0.1084 Nm, sum -2.3448 Nm. Right hip roll at 17.720 s is actuator +2.2430 Nm and damping +0.1063 Nm, sum +2.3493 Nm. Those sums are over 2.33 Nm. Unclamped snap peaks split into kp*(q_des-q) and -kv*omega. Turn right hip roll +4.2372 Nm at 15.872 s, kp +5.3271, kv -1.0899, kp dominates. Turn right knee -6.3249 Nm at 13.296 s, kp -8.4632, kv +2.1383, kp dominates. Turn left hip roll -4.3120 Nm at 16.128 s, kp -5.3265, kv +1.0145, kp dominates. Turn left knee +6.2326 Nm at 13.040 s, kp +9.3436, kv -3.1110, kp dominates. Stop right knee +13.0915 Nm at 14.400 s, kp +13.1089, kv -0.0174, kp dominates. The overs are the position term.

The ramp latches step length and yaw at the OP3 movement boundaries, gait time 0, phase1 0.125 s, and phase3 0.375 s, and spreads the change over 2 periods. From the anchor to frac 1.000 is 1.024 s, 2.05 periods of 0.500 s. The straight stop goes from vx +0.1500 at 8.296 s to vx 0 at 9.320 s. Yaw left goes from 0 at 10.728 s to +0.25000 at 11.752 s. Yaw right goes from 0 at 19.280 s to -0.25000 at 20.304 s. Settle holds that zero step for 0.500 s. hold_stand is not the stop. Vendor gait_manager, as copied in gait_manager_traj.py, publishes rot 0 on the forward demo and no yaw column in the dsp table or the speed gears. That phase-boundary update is the published smoothing. No rad/s cap under +/-0.25 is in that set, so the lower caps are 0.20 and then 0.15.

At +/-0.25 the ramp still Prefer FAILs. It tips at 21.096 s, margin -0.083. Unclamped asks, all kp-dominant and all over 2.33 Nm: turn left hip roll -5.5251 Nm at 20.192 s, yaw -0.22400, kp -6.5207, kv +0.9956; turn right knee +12.6156 Nm at 21.096 s on the fault stand write, kp +13.0298; stop left knee +6.1409 Nm at 8.224 s, yaw 0, kp +8.5374, kv -2.3966; stop right knee -5.9742 Nm at 8.480 s, kp -8.1355. Measured floor right hip roll +2.3723 Nm at 17.016 s, contact floor, stop, raw headroom -0.042280 Nm, hundredths -0.05. Whole-walk left toe +1.074939 mm at 17.464 s, fraction 0.208333, centre +2.266390 mm, sole roll -0.00903 rad, sole pitch +0.01257 rad, scored front-inside. Corners +1.761212 / +1.074939 / +3.457840 / +2.771568 mm. Raw gap +0.925061 mm, floored hundredth 0.92 mm. Right toe -1.229813 mm at 20.000 s, centre +0.477040 mm, sole roll +0.05382 rad, sole pitch -0.00500 rad. Corners -1.229813 / +2.858482 / -1.904403 / +2.183893 mm. Raw gap +3.229813 mm, floored hundredth 3.22 mm. Rug clearance +1.763119 mm at 12.600 s, right, absolute toe +13.763119 mm, surface rug. Corners +14.076663 / +13.763119 / +14.511272 / +14.197729 mm. Raw gap +0.236881 mm, floored hundredth 0.23 mm.

At +/-0.20 the cruise reaches +/-0.20000 and the bout tips at 21.144 s, margin -0.083. Turn left knee +6.1554 Nm at 13.344 s, yaw +0.20000, kp +8.2737, kv -2.1183. Stop left knee stays +6.1409 Nm at 8.224 s. Floor right knee +2.4500 Nm at 21.056 s. Left toe +2.221182 mm at 3.128 s, the straight tick, surplus floored hundredth -0.23. Right toe +0.814970 mm at 17.720 s, floored hundredth 1.18 short. Centre +1.078090 mm. Corners +0.814970 / +1.706283 / +0.449898 / +1.341211 mm. Rug clearance +1.794184 mm at 12.600 s, floored hundredth 0.20 short.

At +/-0.15 the cruise reaches +/-0.15000 and the bout tips at 25.160 s, margin -0.066. Turn left knee +6.1302 Nm at 24.328 s, yaw -0.15000, kp +8.8534, kv -2.7233. The fault writes a right-knee ask of +15.3724 Nm at 25.160 s. Stop left knee stays +6.1409 Nm at 8.224 s. Floor knees are -2.4500 Nm at 20.512 s and +2.4500 Nm at 21.296 s. Left toe -0.291185 mm at 23.840 s, floored hundredth 2.29. Right toe -2.616074 mm at 22.048 s, floored hundredth 4.61. Rug-tagged clearance +0.956466 mm at 14.392 s, left, the tick surface is floor and on_rug is 1, floored hundredth 1.04.

Turning asks stay over 2.33 Nm at +/-0.25, +/-0.20, and +/-0.15. kp dominates each of those overs. A doorway re-point is a finite heading change, separate from this cruise. At 0.15 rad/s a quarter turn is 10.47 s. At 0.20 rad/s it is 7.85 s. That lower cap can carry the re-point. It does not put the turning ask under 2.33 Nm. Foot-z was not raised. Less-crouch stayed closed. Not kit-safe. Not go-anywhere.

Soft stop, yaw held at 0, on that same stack. Vendor walk-to-stand was read before this copy. OP3 Op3Walker.stop sets ctrl_running false, zeros x_cmd, y_cmd, and angle_cmd, sets time to 0, sets previous_x to 0, and calls update_movement on that same call. Time 0 is the double support before the left swing. It is not a wait for the next double support. Amplitudes go to 0 on that call. There is no step-length decay and no both-feet load check. gait_manager_traj.GaitManagerClock has no stop. set_command stores x, y, and angle. _phase_update applies them at time 0, phase1, and phase3, including a single-support boundary. The forward demo is set_step with rot 0 and a constant x. The live not-walking path calls walker.stop then hold_stand, which writes the stand pose on that tick. That snap is not the scored stop.

This bout is straight Day-1 vx +0.150 m/s, then a stop command at 8.200 s. Yaw is 0 for the whole bout. The stop arrives in left single support, gait time 0.032 s, step length +0.02000 m. The clock keeps that full step until the next double support. Freeze is 8.400 s, gait time 0.229 s, just after the left swing ends at 0.225 s. Left load at the freeze is 0.00 N and right load is 16.77 N, so the step length stays +0.02000 m until both feet exceed 5 N. Decay starts at 8.456 s, gait time still 0.229 s, left load 6.14 N, right load 22.08 N, and the step length falls linearly to 0 over 5 periods, 2.500 s. The clock stays frozen. Level-trim roll and pitch stay on both ankles through that decay. The landing toe-down schedule stays off while the step length is falling. hold_stand is not called. Foot-z stays 1.170 mm. Less-crouch stays closed. Trim lead stays off. Period stays 0.500 s. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged.

Prefer FAIL. The unclamped stop ask is left knee +6.1409 Nm at 8.224 s. q is +1.16437 rad. q_des is +1.35409 rad. kp_term is +8.5374 Nm. kv_term is -2.3966 Nm. kp dominates. omega is +1.6445 rad/s. kp is 45. Phase is swing. The gait clock is left single support, fraction 0.195, gait time 0.064 s. Mode is finish. Step length is still +0.02000 m. This tick is not double support. Left sole centre +12.185736 mm, roll +0.00093 rad, pitch +0.00600 rad, load 0.52 N. Corners heel-inside +12.555583 mm, heel-outside +12.626543 mm, front-inside +11.744928 mm, front-outside +11.815889 mm. Right sole centre +10.684042 mm, roll -0.00904 rad, pitch +0.00835 rad, load 30.01 N. Corners heel-outside +11.591102 mm, heel-inside +10.904336 mm, front-outside +10.463748 mm, front-inside +9.776981 mm. The double-support morph peak is right knee +1.2773 Nm at 8.440 s, mode wait, clock D, q -1.14239 rad, q_des -1.15619 rad, kp_term -0.6211 Nm, kv_term +1.8984 Nm, kv dominates, step length still +0.02000 m. Left sole at that tick is still unloaded, centre +18.351963 mm, roll +0.04482 rad, pitch -0.06312 rad, load 0.00 N. Corners heel-inside +12.399129 mm, heel-outside +15.797638 mm, front-inside +20.906288 mm, front-outside +24.304797 mm. Right sole centre +10.907164 mm, roll +0.01455 rad, pitch -0.00512 rad, load 26.95 N. Corners heel-outside +10.008480 mm, heel-inside +11.114107 mm, front-outside +10.700222 mm, front-inside +11.805849 mm. That morph ask is under 2.33 Nm. It is not the stop peak. The straight-walk peak before the stop command is left knee +6.0744 Nm at 1.000 s, phase shift, kp-dominant. It is the first-step boundary, not this stop. Actuator peak from the stop command onward is left ankle pitch -1.6713 Nm at 8.200 s, under the +/-2.45 Nm plant rail and under 2.33 Nm. Soft-pass stays off. The actuator does not clear the unclamped ask. Body speed falls under 0.02 m/s for 0.20 s at 0.816 s after the command. That T_stop is not a pass. d_min is not rebuilt. The old d_min 0.1263 m is not claimed. Yaw ramp and +/-0.15 stay closed. Turns are not scored. Not kit-safe. Not go-anywhere.

Inflight stop, yaw held at 0, on that same stack. The stop command is 8.200 s. The bracketing ticks are 8.192 s and 8.200 s. At 8.200 s the clock is left single support, gait time 0.032 s, fraction 0.035. The airborne foot is the right one. Left load is 26.47 N. Right load is 0.00 N. The right foot's pre-stop target at 8.192 s, gait time 0.024 s, is x +0.01968 m, y +0.01496 m, z -0.18102 m, roll 0, pitch 0, knee q_des -1.13565 rad. The unfrozen plan at 8.200 s is x +0.01948 m, y +0.01498 m, z -0.18180 m, roll 0, pitch 0, knee q_des -1.12059 rad. Planar x/y and roll/pitch do not jump. dx is -0.00021 m. dy is +0.00002 m. droll is 0. dpitch is 0. q_des does move. dknee_des is +0.01506 rad, with the one-tick z change of -0.00078 m. The pin is that pre-stop x/y and roll/pitch. z stays on the live schedule. The phase clock keeps running. Time is not parked at 0. Vendor stop is not this path.

At 8.192 s the mode is pre, the clock is double support, step length is +0.02000 m, and the freeze is off. Left knee q is +1.15005 rad, q_des +1.11022 rad, centre +11.294726 mm, load 26.44 N. Right knee q is -1.16797 rad, q_des -1.13565 rad, centre +19.431059 mm, load 0.00 N. At 8.200 s the mode is air, the freeze is swing_xy_rp, and step length is still +0.02000 m. Left knee q is +1.15468 rad, q_des +1.19688 rad, centre +11.077936 mm, load 26.47 N. The left knee ask is +1.3910 Nm. The left hip roll ask is -3.8823 Nm. Right knee q is -1.16451 rad, q_des -1.12009 rad, centre +17.930135 mm, load 0.00 N. The right hip roll ask is -3.2762 Nm.

With that pin and no step-length change, the DSP snap fires at 8.432 s, gait time 0.258 s, left load 7.50 N, right load 23.96 N. Amplitudes go to 0 on that tick and the stance planar target stays put. The unclamped stop ask is still left knee +5.8728 Nm at 8.224 s. q is +1.17284 rad. q_des is +1.35409 rad. kp_term is +8.1562 Nm. kv_term is -2.2834 Nm. kp dominates. The right foot is frozen. The left foot is the clock swing and is not frozen. Mode is air. Step length is still +0.02000 m. Clock fraction is 0.155, gait time 0.056 s. Left sole centre +10.747896 mm, roll -0.01474 rad, pitch +0.00467 rad, load 2.30 N. Corners heel-inside +11.623137 mm, heel-outside +10.503017 mm, front-inside +10.992775 mm, front-outside +9.872656 mm. Right sole centre +13.361409 mm, roll -0.06112 rad, pitch +0.00542 rad, load 36.15 N. That right load is real. Prefer FAIL. The next-stride decay was then taken.

The decay runs from the stop tick over one period, 0.500 s. The right foot stays pinned. At 8.224 s the step length is +0.01904 m. The unclamped ask is left knee +5.9483 Nm. q is +1.17286 rad. q_des is +1.35589 rad. kp_term is +8.2364 Nm. kv_term is -2.2881 Nm. kp dominates. omega is +1.5701 rad/s. kp is 45. kv is 1.4573. Phase is swing. Clock is left single support, fraction 0.155, gait time 0.056 s. Mode is air. Freeze is swing_xy_rp. Left sole centre +10.748332 mm, roll -0.01474 rad, pitch +0.00473 rad, load 2.38 N. Corners heel-inside +11.627850 mm, heel-outside +10.507475 mm, front-inside +10.989189 mm, front-outside +9.868814 mm. Right sole centre +13.362124 mm, roll -0.06112 rad, pitch +0.00540 rad, load 36.09 N. Corners heel-outside +16.047303 mm, heel-inside +11.405249 mm, front-outside +15.318999 mm, front-inside +10.676945 mm. The snap is 8.432 s, gait time 0.258 s, left load 8.71 N, right load 23.48 N. Actuator peak from the stop command is right hip roll +1.8977 Nm at 8.256 s, under the +/-2.45 Nm plant rail and under 2.33 Nm. Setting that next stride's step length to 0 on the stop tick, with the right foot still pinned, leaves left knee q_des at +1.33362 rad and the ask at +6.3681 Nm at 8.216 s, with x_move 0. The knee target is the swing-z schedule. Step length is not this miss. T_stop on the one-period decay is 0.744 s and is not a pass. d_min is not rebuilt. The old d_min 0.1263 m is not claimed. Yaw ramp and +/-0.15 stay closed. Foot-z was not raised. Less-crouch stayed closed. Trim lead stayed off. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Stance-chain freeze, yaw held at 0, on that same stack. On the stop tick the loaded stance leg is the left one, 26.47 N, and the airborne foot is the right one, 0.00 N. The clock is left single support, gait time 0.032 s, fraction 0.035. All six left q_des are pinned at the pre-stop totals, including level trim already in them: knee +1.11022 rad, hip roll -0.15964 rad, hip pitch -0.66655 rad, hip yaw +0.00000 rad, ankle pitch +0.70547 rad, ankle roll -0.15964 rad. The stance foot pose and the pelvis rolls stay at that pre-stop pose. pel_r is 0 and pel_l is 0. The airborne right foot keeps its pre-stop x/y and roll/pitch. Only that foot's swing-z schedule advances. x_move stays +0.02000 m on the stop tick. The next-stride decay is not this bout. Time is not parked at 0.

The lever is flat. Left knee_des is +1.11022 rad at 8.192 s, 8.200 s, and 8.224 s. Left hip_roll_des is -0.15964 rad at all three. Left hip_pitch_des is -0.66655 rad at all three. The IK stance knee equals the pin. The delta is +0.00000 rad at 8.200 s. Written q_des is the pin. The 1.110 to 1.356 climb is gone. IK and the CoM planar target were frozen, not live. The DSP snap fires at 8.224 s, gait time 0.056 s, left load 15.72 N, right load 18.10 N. Amplitudes go to 0 on that tick. Stance q_des and the planar target stay put. x_move is 0 on the snap tick only.

Prefer FAIL. The unclamped stop ask peaks at left ankle roll -4.5042 Nm at 8.200 s. q is -0.00129 rad. q_des is -0.15964 rad. kp_term is -5.5422 Nm. kv_term is +1.0380 Nm. kp dominates. omega is -0.8740 rad/s. kp is 35. kv is 1.1876. Phase is swing. Mode is air. Freeze is swing_z+stance_q. Pelvis frozen is 1. Body vx is +0.2255 m/s. Yaw is 0. Left sole centre +11.077936 mm, roll -0.03036 rad, pitch -0.00207 rad, load 26.47 N. Corners heel-inside +12.091474 mm, heel-outside +9.784738 mm, front-inside +12.371134 mm, front-outside +10.064399 mm. Right sole centre +17.930135 mm, roll -0.06498 rad, pitch -0.02841 rad, load 0.00 N. Corners heel-outside +18.483483 mm, heel-inside +13.550578 mm, front-outside +22.309692 mm, front-inside +17.376786 mm. The six left targets at that tick are the pin: hip yaw q +0.00531 rad against q_des 0, ask -0.2973 Nm; hip roll q -0.02575 rad against q_des -0.15964 rad, ask -3.6776 Nm; hip pitch q -0.68644 rad against q_des -0.66655 rad, ask +0.4020 Nm; knee q +1.15468 rad against q_des +1.11022 rad, ask -2.5088 Nm; ankle pitch q +0.71475 rad against q_des +0.70547 rad, ask -1.6065 Nm; ankle roll q -0.00129 rad against q_des -0.15964 rad, ask -4.5042 Nm.

The other stop-window overs, all kp-dominant, are left hip roll -3.6776 Nm at 8.200 s (q -0.02575 rad, q_des -0.15964 rad, kp_term -5.3555 Nm, kv_term +1.6779 Nm, omega -0.9854 rad/s, kp 40, kv 1.7027), right hip roll -3.3182 Nm at 8.200 s (q +0.04089 rad, q_des -0.09573 rad, kp_term -5.4649 Nm, kv_term +2.1467 Nm, omega -1.2608 rad/s), right knee +2.6025 Nm at 8.224 s (q -1.14589 rad, q_des -1.05431 rad, kp_term +4.1212 Nm, kv_term -1.5187 Nm, omega +1.0421 rad/s, kp 45, kv 1.4573), left knee -2.5088 Nm at 8.200 s (q +1.15468 rad, q_des +1.11022 rad, kp_term -2.0005 Nm, kv_term -0.5083 Nm, omega +0.3488 rad/s), and right ankle roll -2.4033 Nm at 8.200 s (q +0.02622 rad, q_des -0.09573 rad, kp_term -4.2684 Nm, kv_term +1.8650 Nm). The right knee is the airborne swing-z schedule. It is not a stance climb. The left knee is down from +5.9483 Nm and its q_des did not climb. Under the rail from the stop command: left ankle pitch -1.6065 Nm at 8.200 s, left hip pitch +0.4020 Nm at 8.200 s, left hip yaw -0.3421 Nm at 8.216 s, right ankle pitch +1.2640 Nm at 8.232 s, right hip pitch -1.0018 Nm at 8.224 s, right hip yaw +0.3916 Nm at 8.288 s.

At 8.224 s the mode is snap, freeze is swing_xy_rp+stance_q, x_move is 0, body vx is +0.1842 m/s, and yaw is 0. Left knee q is +1.14761 rad, q_des +1.11022 rad. Right knee q is -1.14589 rad, q_des -1.05431 rad. Left sole centre +10.657740 mm, roll -0.01525 rad, pitch +0.00492 rad, load 15.72 N. Corners heel-inside +11.569690 mm, heel-outside +10.410405 mm, front-inside +10.905074 mm, front-outside +9.745789 mm. Right sole centre +13.812213 mm, roll -0.05624 rad, pitch -0.00019 rad, load 18.10 N. Corners heel-outside +15.935282 mm, heel-inside +11.663155 mm, front-outside +15.961271 mm, front-inside +11.689144 mm. The six left q_des values are still the pin. At 8.192 s, before the bar, the same left q_des is already over: left ankle roll -4.7555 Nm, left hip roll -4.1902 Nm, left knee -3.1850 Nm. Freezing q_des does not close a gap that already existed. Actuator peak from the stop command is left knee -1.6866 Nm at 8.200 s, headroom 0.763383 Nm, under the +/-2.45 Nm plant rail and under 2.33 Nm. T_stop 0.616 s is the body-speed settle and is not a pass. d_min is not rebuilt. The old d_min 0.1263 m is not claimed. Yaw ramp and +/-0.15 stay closed. Turns are not scored. Foot-z was not raised. Less-crouch stayed closed. Trim lead stayed off. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Continuous straight walk, no stop, same Day-1 vx, yaw 0. The steady window starts at 2.000 s so the first step is not the score. Prefer FAIL. This is a walk-tracking miss. DSP left ankle roll is -4.7852 Nm at 2.560 s. q is +0.00633 rad. q_des is -0.15964 rad. kp_term is -5.8090 Nm. kv_term is +1.0238 Nm. kp is 35. kv is 1.1876. DSP left hip roll is -4.3545 Nm at 2.048 s. DSP left knee is -3.3719 Nm at 2.048 s. Mid-walk right knee is -5.8710 Nm at 3.872 s, q -1.17294 rad, q_des -1.35409 rad. That knee is the swing-z schedule. Foot-z stays 1.170 mm.

At the DSP edge, pose 0.024 s, t 2.048 s, the gait ankle-roll command is -0.15964 rad, the level-trim add is +0.00000 rad, endpoint roll is 0, pelvis add is 0, and logged q_des minus gait is 0. The 0.16 rad is y_swap. HIP_FF is not on this stack. Pelvis stays 5 deg and is 0 in double support.

y_swap swept downward from the locked 0.020 m. DSP ankle and hip roll stay over 2.33 Nm at 0.016, 0.015, 0.014, and 0.012 m. They clear at 0.011 m. The worst DSP ask there is right hip roll -2.1974 Nm at 3.064 s, and every DSP leg ask is at or under 2.33 Nm. Mid-single-support CoM margin at 0.011 m is -4.81 mm, and all 227 mid ticks sit outside the 38 mm stance box. At 0.015 m the mid margin is +0.04 mm and every mid tick is inside, but DSP left hip roll is still -3.3170 Nm at 2.040 s and DSP left ankle roll is -2.6370 Nm at 4.600 s. Lateral has to stay for the box. The stop stays at locked y_swap 0.020 m.

Slew branch taken. Once both feet are over 5 N, at 8.224 s, pinned q_des slews toward the stand pose over 1.000 s and then holds that pose. Each tick keeps |kp*(q_des-q)| + |kv*omega| at or under 2.33 Nm. During that window the unclamped ask peaks at right ankle pitch -2.3300 Nm at 8.224 s, on the cap. The parts peak is right knee 2.3300 Nm at 8.280 s, also on the cap. Corners stay up. The lowest in the window is right front-outside +8.645 mm at 8.296 s. No dig-in. At 8.224 s, 8.720 s, and 9.224 s every corner is above +9.7 mm. The airborne tick before that window still holds the pin: left ankle roll -4.5042 Nm at 8.200 s, q -0.00129 rad, q_des -0.15964 rad. No fault. Prefer FAIL. The walk at 0.020 m is still over, so the stop is not a pass. d_min is not rebuilt. The old d_min 0.1263 m is not claimed. Foot-z was not raised. Less-crouch stayed closed. Trim lead stayed off. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Sole-flat stance ankle on the same continuous walk, yaw 0, no stop. When floor+rug load is over 5 N, ankle-roll q_des is the measured angle. Hip roll keeps the y_swap IK. Hip yaw is not added. Level trim on that write is 0. Prefer FAIL. At y_swap 0.020 m and kit init_y 0.005 m, DSP left hip roll is +4.4847 Nm at 3.328 s and right hip roll is -4.4513 Nm at 6.144 s. Both peaks are on the loaded foot. The loaded ankle peaks are left -1.1671 Nm at 2.272 s and right +1.2021 Nm at 2.520 s, under 2.33 Nm, because q_des equals q. The DSP ankle peaks that stay over are the unloading foot: left -3.1296 Nm at 2.040 s (q -0.01307 rad, q_des -0.14078 rad) and right +3.0824 Nm at 2.808 s. At the DSP edge, pose 0.024, t 2.048 s, the left foot is at 0.00 N, flat 0, gait ankle roll -0.15964 rad, trim 0, des -0.15964 rad, q -0.02412 rad. The right foot is at 23.12 N, gait -0.09621 rad, des +0.03865 rad equals q, ask +0.8602 Nm. Sole roll there is +0.08611 rad. The lowest loaded corner is the right foot at -4.162 mm at 3.064 s. Mid-SS CoM margin is -9.39 mm, 227 of 227 ticks outside the 38 mm box. Centres stay +14.00 mm and -14.00 mm. Longitudinal slack is +53.89 mm. Mid-SS hips and ankle rolls are under 2.33 Nm. Mid-SS knees are -3.2265 Nm and +3.3813 Nm. Mid-SS ankle pitch is +2.9384 Nm and -3.0131 Nm. Widening init_y at y_swap 0.020 m pushes the CoM further out: -12.60 mm at 0.008 m, -15.19 mm at 0.010 m, -18.01 mm at 0.012 m, -20.93 mm at 0.015 m, -26.97 mm at 0.020 m, and -32.50 mm at 0.025 m. Every mid tick stays outside. Cutting y_swap at kit init_y 0.005 m never puts the CoM inside. The closest margin is -6.27 mm at y_swap 0.000 m, still 227 of 227 outside, corner -3.846 mm. DSP hip roll is under 2.33 Nm by y_swap 0.008 m (+1.7551 Nm) and the loaded ankle stays near 1.1 Nm, while the mid-SS knee stays about -3.4 Nm and the mid-SS ankle pitch about +2.8 Nm. The same cut at init_y 0.025 m leaves the margin between -30.34 mm and -24.56 mm. The contact box was not widened. The swing-z knee stays about -6.16 Nm and is parked. Foot-z stays 1.170 mm. Less-crouch stayed closed. Trim lead stayed off. d_min is not rebuilt. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Sagittal mid-SS at y_swap 0 on that same continuous walk, yaw 0, no stop. Sole-flat stance ankle stays on. Kit init_y stays 0.005 m. Day-1 vx stays 0.150 m/s. Hip roll and CoM-in-box are not this pass. CLEAR. The mid-SS knee and the parked swing-z knee are different joints and phases. On the unlimited full step the mid-SS stance knee is right knee +3.6558 Nm at 2.104 s, phase L, fraction 0.275, role stance, load 25.70 N. q is -1.15467 rad. q_des is -1.01469 rad and equals the gait IK. q_stand is -1.05222 rad. q_nox is -1.01826 rad. shape is +0.03753 rad. step is +0.00357 rad. crouch is +0.03396 rad. sag is -0.13998 rad. kp_term is +6.2993 Nm. kv_term is -2.6435 Nm. omega is +1.8139 rad/s. kp is 45. kv is 1.4573. The measured knee is 0.140 rad more flexed than the command, and the step barely moves that command. Left mid knee is -3.4840 Nm at 2.360 s. DSP knees stay under: left -1.7178 Nm at 2.304 s and right +1.8104 Nm at 2.560 s. The SSP swing knee is right knee -6.3934 Nm at 3.352 s, phase R, fraction 0.115, role swing, load 0.00 N. q is -1.16005 rad. q_des is -1.34742 rad. shape is -0.29520 rad. step is +0.02109 rad. crouch is -0.31629 rad. That peak is the swing-z crouch. It stays parked. Foot-z stays 1.170 mm. Less-crouch stays closed. The unlimited mid-SS ankle pitch is late-stance tracking, and part of it is the step. Right ankle pitch is -2.7823 Nm at 3.224 s, phase L, fraction 0.740, role stance, load 15.54 N. q is -0.48707 rad. q_des is -0.60826 rad and equals the gait IK. q_stand is -0.54616 rad. q_nox is -0.53310 rad. shape is -0.06210 rad. step is -0.07515 rad. crouch is +0.01306 rad. sag is +0.12118 rad. kp_term is -4.2414 Nm. kv_term is +1.4591 Nm. omega is -1.2179 rad/s. kp is 35. kv is 1.1980. ep_x is -0.01317 m. Sole pitch is -0.01166 rad. Left mid ankle pitch is +2.7616 Nm at 3.992 s. DSP ankle pitch stays under: left +1.4533 Nm at 4.048 s and right -1.4576 Nm at 3.280 s. Cutting the step length, bus vx still 0.150 m/s, does not clear. The knee gets worse as the step shrinks. Ankle pitch is under 2.33 Nm by x_scale 0.50. At x_scale 1.00, x_cmd +0.020 m, mid knee is -3.4840 / +3.6558 Nm and mid ankle pitch is +2.7616 / -2.7823 Nm, flat toe +12.856 mm. At 0.75, x_cmd +0.015 m, mid knee is -3.6275 / +3.7531 Nm. At 0.50, x_cmd +0.010 m, mid knee is -3.7622 / +3.8595 Nm and mid ankle pitch is +1.9306 / -1.9482 Nm. At x_scale 0.00, x_cmd 0, mid knee is -4.0110 Nm at 5.432 s and +4.0185 Nm at 5.176 s. Zero step does not remove the stance-knee sag. It is the locked crouch under single-support load. A stance slew on both the knee and the ankle pitch, full step, does clear. The cap is radians per second on stance q_des. Double support limits both legs. Single support limits the loaded leg only. The swing knee is not limited. 2.50 rad/s fails: mid knee -2.8918 / +3.0295 Nm, mid ankle pitch +2.5285 / -2.5670 Nm. 2.00 rad/s fails: mid knee -2.3106 / +2.4013 Nm. 1.95 rad/s fails by 0.0005 Nm: right mid knee +2.3305 Nm at 2.160 s, left mid knee -2.2349 Nm at 3.944 s, mid ankle pitch +2.1173 / -2.2212 Nm. The mildest rate that clears is 1.90 rad/s, 0.0152 rad per 8 ms tick. Mid knee is left -2.2276 Nm at 2.928 s and right +2.3070 Nm at 2.672 s. At that right knee, q is -1.05034 rad, q_des is -0.94377 rad, gait is -0.94168 rad, sag is -0.10657 rad, kp_term is +4.7958 Nm, kv_term is -2.4888 Nm, omega is +1.7077 rad/s, load 20.88 N. Mid ankle pitch is left +2.0750 Nm at 3.992 s and right -2.1889 Nm at 2.200 s. At that right ankle, q is -0.49815 rad, q_des is -0.57085 rad, gait is -0.58113 rad, sag is +0.07271 rad, kp_term is -2.5448 Nm, kv_term is +0.3559 Nm. DSP knee is left -1.6490 Nm at 2.816 s and right +1.7365 Nm at 2.560 s. DSP ankle pitch is left +2.0037 Nm at 2.000 s and right -2.0968 Nm at 2.256 s. Either joint alone at 1.90 rad/s fails. Knee-only leaves right mid ankle pitch at -2.9708 Nm, with q_des equal to the gait -0.60826 rad. Ankle-pitch-only leaves right mid knee at +3.6717 Nm, with q_des equal to the gait -1.01469 rad. The SSP swing knee stays over: right -6.3947 Nm at 2.328 s and left +6.4171 Nm at 2.072 s, and stays parked. Flat mid-swing toe is +12.957 mm at 2.872 s, right, fraction 0.208, above the +2 mm bar. Mid margin is -5.95 mm, 227 of 227 outside. Foot-z stays 1.170 mm. Less-crouch stayed closed. Trim lead stayed off. d_min is not rebuilt. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Soft stop on those sagittal locks, yaw 0, same plant. Period 0.500 s. Foot-z 1.170 mm. Less-crouch closed. Trim lead off. y_swap 0. Sole-flat stance ankle stays on. The 1.90 rad/s stance slew stays on through the walk and the stop. After the stop it steps toward the stand pose and keeps |kp*(q_des-q)| + |kv*omega| at or under 2.33 Nm. The airborne swing-z knee is not on that slew. The stop command is 8.200 s, clock L, gait time 0.032 s, just after left single support starts. Both feet are still loaded, left 7.41 N and right 19.75 N. That is not a settled double support. x_move stays +0.02000 m on that tick. The left foot unloads at 8.208 s, 4.97 N. Its x/y/roll/pitch pin there and only z advances. The snap is 8.336 s, gait time 0.168 s, left 14.75 N and right 16.31 N, and x_move goes to 0 only then. Prefer FAIL. The unclamped stop ask is left knee +7.7458 Nm at 8.224 s, phase swing, the airborne swing-z schedule, mode air. q is +1.15745 rad. q_des is +1.38448 rad. kp_term is +10.2163 Nm. kv_term is -2.4704 Nm. omega is +1.6952 rad/s. kp is 45. kv is 1.4573. The tick immediately before the stop, 8.192 s, is left knee -0.8591 Nm, under 2.33 Nm. This is a stop transition, not a pre-stop lag. The walk-wide left knee +6.3756 Nm at 2.072 s is the parked SSP swing-z and is not this label. On the stop tick the stance right knee is +2.3300 Nm, on the cap, q_des -1.14024 rad. The tick before that is +2.0244 Nm. The body then tips at 8.504 s, margin -0.060. After the tip the knee and ankle-pitch cap still holds q_des, and the velocity term alone stays over: right knee -2.6723 Nm at 8.794 s with q_des equal to q, left ankle pitch +5.5117 Nm at 8.854 s with q_des equal to q and omega -4.6007 rad/s, right ankle pitch -4.9381 Nm at 8.852 s with q_des equal to q. Right hip pitch is not on that slew. The post-tip stand write is -7.4126 Nm at 8.504 s, phase stand, q +0.93212 rad, q_des +0.76786 rad, kp_term -7.3917 Nm, kv_term -0.0209 Nm. The tick before the stop is -0.2102 Nm. Hip roll and ankle roll stay under 2.33 Nm on this bout. DSP hip roll is left -0.8958 Nm at 7.664 s and right +0.8312 Nm at 2.272 s. DSP ankle roll is left -0.8679 Nm at 7.928 s and right +0.9855 Nm at 2.040 s, q_des equal to q. Mid hip roll is left -1.6618 Nm at 8.000 s and right +1.5791 Nm at 7.232 s. At the peak tick both contact boxes are outside: left slack_y -11.14 mm, right slack_y -6.83 mm, half_y 38.00 mm, centres +14.00 mm and -14.00 mm, load left 0.09 N and right 18.87 N. That outside CoM is the y_swap 0 class (the continuous-walk margin stays -5.95 mm). It is not the torque bar and it does not make the bout kit-safe. T_stop is none. d_min is not rebuilt. The old d_min 0.1263 m is not claimed. Actuator peak from the stop is left knee +2.2800 Nm at 8.776 s, under the +/-2.45 Nm plant rail. Flat toe on this copy is +11.199 mm at 2.616 s, left, fraction 0.208. That toe is this y_swap 0 copy. It is not the +2.221 mm clear at y_swap 0.020. When lateral returns, the 20-80% flat toe and the parked SSP swing-z knee are re-scored. Track 1 is not done. Foot-z stayed 1.170 mm. Less-crouch stayed closed. Trim lead stayed off. y_swap stayed 0. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Swing freeze on that same stop, yaw 0, same plant. From the stop command the swing hip roll, hip yaw, hip pitch, and ankle roll hold the pre-stop q_des. Planar x/y and sole roll/pitch hold. z is not pinned. Knee and ankle pitch are the swing-z IK, not a frozen q_des. Stand hips start only after both feet are over 5 N, and only on a loaded leg, inside the 2.33 Nm part cap. The 1.90 rad/s stance knee and ankle-pitch slew is unchanged. The stop command is still 8.200 s, clock L, gait time 0.032 s. The freeze is left hip yaw 0, hip roll −0.03224 rad, hip pitch −0.68308 rad, ankle roll −0.03224 rad, x −0.02032 m, y +0.00500 m. Both feet are still loaded, left 7.41 N and right 19.75 N. That is not settled double support. x_move stays +0.02000 m. The snap is 8.336 s, gait time 0.168 s, left 17.36 N and right 14.97 N, and x_move goes to 0 only then. Prefer FAIL. Torque is scored before the tip. The unclamped stop ask is left knee +8.0985 Nm at 8.224 s, phase swing, the airborne swing-z IK, mode air. q is +1.15466 rad. q_des is +1.38355 rad. kp_term is +10.3004 Nm. kv_term is −2.2019 Nm. omega is +1.5109 rad/s. kp is 45. kv is 1.4573. The tick immediately before the stop, 8.192 s, is left knee −0.8591 Nm. This is a stop transition, not a pre-stop lag. The walk-wide left knee +6.3756 Nm at 2.072 s is the parked SSP swing-z and is not this label. Left ankle pitch is +3.6375 Nm at 8.216 s, q +0.69318 rad, q_des +0.84108 rad, the tick before the stop +0.1603 Nm. That joint is the same swing-z IK. It is not a hard pin. On the stop tick the stance right knee is +2.3300 Nm, on the 1.90 rad/s cap, q_des −1.14024 rad. The tick before that is +2.0244 Nm. Right hip pitch before the tip is −1.1582 Nm at 8.344 s, q +0.94338 rad, q_des +0.90902 rad, stepping from the pin +0.93942 rad toward stand inside the part cap. It is not the one-tick stand write. Hip roll and ankle roll stay under 2.33 Nm before the tip. Left hip roll is −0.4246 Nm at 8.512 s. Right hip roll is +0.5111 Nm at 8.328 s. Left ankle roll is +0.4666 Nm at 8.216 s, q_des equal to the pin −0.03224 rad. Right ankle roll is +0.7755 Nm at 8.408 s. The body tips at 8.528 s, margin −0.061. After the tip the held ankle-pitch command and the velocity term stay over: right ankle pitch −6.7360 Nm at 8.886 s, q_des −0.54616 rad, phase stand. That tick is not the torque bar. At the peak tick both contact boxes are outside: left slack_y −11.41 mm, right slack_y −6.88 mm, half_y 38.00 mm, centres +14.00 mm and −14.00 mm, load left 3.09 N and right 14.73 N. That outside CoM is the y_swap 0 class. It is not the torque bar and it does not make the bout kit-safe. T_stop is none. d_min is not rebuilt. The old d_min 0.1263 m is not claimed. Actuator peak from the stop before the tip is left knee +1.8478 Nm at 8.232 s, under the +/−2.45 Nm plant rail. The post-tip actuator peak is left knee +2.2800 Nm at 8.840 s and is not this bar. Flat toe on this copy is +11.199 mm at 2.616 s, left, fraction 0.208. That toe is this y_swap 0 copy. It is not the +2.221 mm clear at y_swap 0.020. Track 1 is not done. The next lever is the swing-z knee and ankle-pitch IK during the lift, which still asks over 2.33 Nm before that foot loads. Approaching those two targets inside the 2.33 Nm budget, without freezing z, is the lever. A full pose freeze is not, because holding z up never loads both feet. Foot-z stayed 1.170 mm. Less-crouch stayed closed. Trim lead stayed off. y_swap stayed 0. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Stretched swing-z on that same stop, yaw 0, same plant. The leftover rise above the end height is scaled to 0.25 and the clock is stretched by 1.060 so the knee and ankle-pitch IK stay at or under 1.90 rad/s. Peak open-loop rate is 2.014 rad/s on the knee and 1.157 rad/s on the ankle pitch. The clock waits while either command is more than one step behind, and a stretch step that still outruns the slew is shortened. The 1.90 rad/s stance slew is unchanged. Planar x/y, sole angles, and the swing hip and ankle-roll pins stay. The stop command is still 8.200 s, clock L, gait time 0.032 s, left 7.41 N and right 19.75 N. That is not settled double support. x_move stays +0.02000 m. The left foot unloads to 3.91 N at 8.216 s, then reloads. Touchdown is 8.384 s, gait 0.153, left 17.48 N, after 7 clock waits. The four contact-box corners are heel-inside −1.953 mm, heel-outside +1.271 mm, front-inside +68.474 mm, front-outside +71.698 mm. The lowest corner is the heel. Ankle roll q is −0.03714 rad against the pin −0.03224 rad, and ankle pitch q is +0.75687 rad. Sole-flat applies on that write. The snap is 8.392 s, gait 0.153, left 17.48 N and right 6.27 N, and x_move goes to 0 only then. Prefer FAIL on the tip. Torque before the tip is clear: every leg ask from the stop until the tip stays at or under 2.33 Nm. The unclamped peak is the stance right knee +2.3300 Nm at 8.200 s, on the 1.90 rad/s bar, q −1.18702 rad, q_des −1.14024 rad. The tick before that is +2.0244 Nm. Left knee before the tip is −2.0824 Nm at 8.400 s, q +1.22542 rad, q_des +1.17735 rad. The tick before the stop is −0.8591 Nm. Left ankle pitch is −1.6938 Nm at 8.200 s, q +0.67241 rad, q_des +0.68149 rad. The tick before the stop is +0.1603 Nm. Right hip pitch before the tip is −1.3108 Nm at 8.424 s, q_des +0.88195 rad, stepping from the pin +0.93942 rad. It is not a stand dump. The body tips at 8.512 s, margin −0.068. After the tip, right ankle pitch −8.5621 Nm at 8.878 s is not the torque bar. At the stop tick both contact boxes are outside: left slack_y −11.54 mm, right slack_y −7.84 mm, half_y 38.00 mm. The continuous-walk margin stays −7.99 mm, 305/312 outside. That outside CoM is the y_swap 0 class. It is not the torque bar and it does not make the bout kit-safe. T_stop is none. d_min is not rebuilt. The old d_min 0.1263 m is not claimed. Actuator peak from the stop before the tip is right knee +1.6682 Nm at 8.208 s, under the +/−2.45 Nm plant rail. The post-tip actuator peak is left knee +2.2800 Nm at 9.224 s and is not this bar. Flat toe on this copy is +11.199 mm at 2.616 s, left, fraction 0.208. That toe is this y_swap 0 copy. It is not the +2.221 mm clear at y_swap 0.020. Track 1 is not done. Foot-z stayed 1.170 mm. Less-crouch stayed closed. Trim lead stayed off. y_swap stayed 0. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Continuous straight walk on that same copy, no stop, yaw 0. The 1.90 rad/s stance slew remembers the swing command so the next stance step starts there. The parked stop does not use that memory, and its landing is unchanged: touchdown 8.384 s, snap 8.392 s, tip 8.512 s, margin −0.068, right knee +2.3300 Nm. Prefer FAIL on the walk. The 20–80% flat toes stay up. Left is +12.983 mm at 2.104 s, fraction 0.208. Right is +12.957 mm at 2.872 s, fraction 0.208. Neither is a scuff, and both are above +2 mm. The entrance-rug tag is on and this window has no rug swing. Mid-SS and DSP stay at or under 2.33 Nm. Mid knee is left −2.2276 Nm at 2.928 s and right +2.3070 Nm at 2.672 s. Mid ankle pitch is left +2.0750 Nm at 3.992 s and right −2.1889 Nm at 2.200 s. DSP knee is left −1.6490 Nm at 2.816 s and right +1.7365 Nm at 2.560 s. DSP ankle pitch is left +2.0037 Nm at 2.000 s and right −2.0968 Nm at 2.256 s. The 20–80% swing joints are over. Left knee is +4.1946 Nm at 3.632 s, fraction 0.235. Left ankle pitch is −4.3410 Nm at 3.152 s, fraction 0.395. Right knee is −4.1951 Nm at 3.376 s, fraction 0.235. Right ankle pitch is +4.2864 Nm at 4.944 s, fraction 0.395. The parked SSP swing-z knee is left +6.4171 Nm at 2.072 s, fraction 0.115, and right −6.3947 Nm at 2.328 s, fraction 0.115. At 2.200 s both contact boxes are outside: left slack_y −9.56 mm, right slack_y −5.95 mm, half_y 38.00 mm, centres +14.00 mm and −14.00 mm. The mid margin is −5.95 mm, 227 of 227 outside. That outside CoM is the y_swap 0 class. It is not a torque pass and it does not make the bout kit-safe. The next lever is the walk swing-z knee and ankle pitch through 20–80%, stretched so those asks stay at or under 2.33 Nm without putting a toe under 0. A rate cap on the old clock is not that lever. The stop stretch stays 0.25 / 1.060. d_min is not rebuilt. Foot-z stayed 1.170 mm. Less-crouch stayed closed. Trim lead stayed off. y_swap stayed 0. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Walk swing-z on that continuous copy is its own knob. The schedule is the highest swing z whose knee and ankle pitch stay at or under 1.55 rad/s and that still ends on the landing z. The stop stretch stays 0.25 / 1.060× and is not installed on this bout. The stop bout was scored again and did not move: touchdown 8.384 s, snap 8.392 s, tip 8.512 s, margin −0.068, pre-tip torque clear, right knee +2.3300 Nm at 8.200 s, stretch factor 1.060, soft 0.25. Prefer FAIL on the walk. Both named bars hold. Left flat toe is +6.788 mm at 2.104 s, fraction 0.208. Right is +6.991 mm at 2.360 s, fraction 0.208. Both stay above +2 mm. At those ticks the four corners stay up. Left is heel-inside +0.858 mm, heel-outside +5.642 mm, front-inside +2.005 mm, front-outside +6.788 mm. Right is heel-inside +1.313 mm, heel-outside +6.590 mm, front-inside +1.714 mm, front-outside +6.991 mm. The lowest corner through the window is the heel-inside, left +0.030 mm at 3.176 s, fraction 0.458, and right +0.114 mm at 3.440 s, fraction 0.500. No front corner goes under 0. The 20–80% swing knee is left +1.4740 Nm at 2.176 s, fraction 0.620, and right −1.4372 Nm at 2.432 s, fraction 0.620. Swing ankle pitch is left −2.1981 Nm at 3.648 s, fraction 0.315, and right +2.1455 Nm at 6.464 s, fraction 0.315. The parked SSP knee at fraction 0.115 is gone. The swing-knee peak is that same +1.4740 Nm, inside 20–80% and under 2.33 Nm. DSP stays under 2.33 Nm. Mid-SS does not. Right hip pitch is −2.3441 Nm at 3.128 s, fraction 0.275. Right knee is +2.3824 Nm at 2.672 s, fraction 0.540. A 1.40 rad/s schedule kept that right knee at +2.2885 Nm and put the heel-inside under 0, −0.093 mm left and −0.083 mm right, with the stance hip pitch further over. The next lever is that mid-SS stance right knee and right hip pitch. It is not another cut of the walk swing, and it is not foot-z, crouch, y_swap, the rail, or the plant. CoM mid margin is −5.82 mm, 227 of 227 outside. That is the y_swap 0 class and is not a torque pass. d_min is not rebuilt. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not kit-safe. Not go-anywhere.

Loaded-leg hip pitch on that continuous copy is its own slew, 1.90 rad/s, and the loaded knee is 1.75 rad/s. Ankle pitch stays at 1.90 rad/s. The walk swing stays at 1.55 rad/s. The stop stretch stays 0.25 / 1.060× and was scored again: touchdown 8.384 s, snap 8.392 s, tip 8.512 s, margin −0.068, pre-tip torque clear, right knee +2.3300 Nm at 8.200 s, stretch factor 1.060, soft 0.25. CLEAR on the mid-SS window. Right hip pitch is −1.8358 Nm at 3.128 s, fraction 0.275. Left hip pitch is +1.8290 Nm at 3.384 s, fraction 0.275. Right knee is +2.3004 Nm at 3.184 s, fraction 0.540. Left knee is −2.2883 Nm at 3.952 s, fraction 0.540. A 1.80 rad/s knee left that right knee at +2.3720 Nm. A 1.50 rad/s knee cleared the mid knees and put DSP ankle pitch at −2.4063 Nm. DSP ankle pitch on this copy is left +2.1982 Nm at 2.000 s and right −2.2316 Nm at 2.256 s. The 20–80% swing knee is left +1.4593 Nm at 2.176 s and right −1.4132 Nm at 2.432 s. Swing ankle pitch is left −2.2041 Nm at 3.136 s and right +2.1504 Nm at 5.952 s. Left flat toe is +6.711 mm at 2.104 s, fraction 0.208. Right is +6.908 mm at 2.360 s, fraction 0.208. Window heel-inside is left +0.384 mm at 3.176 s and right +0.676 mm at 3.944 s. At the right-knee peak the swing foot stays up and the loaded sole is front-inside −1.858 mm. At the right-hip peak the loaded sole is front-outside −2.825 mm. That is the locked stance penetration class, not a new dig. The early stance knee at fraction 0.115 is right +2.4591 Nm at 3.096 s and is outside this mid window. CoM mid margin is −5.70 mm, 227 of 227 outside. That is the y_swap 0 class and is not a torque pass. The tip still awaits Dave. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

The loaded knee on that continuous copy is 1.55 rad/s through fraction 0.13, then returns to 1.75 rad/s. Hip pitch stays at 1.90 rad/s. Ankle pitch stays at 1.90 rad/s. The walk swing stays at 1.55 rad/s. The stop stretch stays 0.25 / 1.060× and was scored again: touchdown 8.384 s, snap 8.392 s, tip 8.512 s, margin −0.068, pre-tip torque clear, right knee +2.3300 Nm at 8.200 s, stretch factor 1.060, soft 0.25. CLEAR on the early-stance knee. Fraction 0.115 is a loaded knee, not the airborne swing channel. Right knee there is +2.2956 Nm at 3.096 s, load 29.23 N, the other foot 0.02 N. Left knee there is −2.2594 Nm at 2.840 s, load 28.97 N, the other foot 0.09 N. The loaded sole at that right tick is front-outside −3.217 mm. The swing-foot heel-inside is −1.429 mm and is toe-off, outside 20–80%. The early-window peak is right knee +2.2963 Nm at 3.088 s, fraction 0.075, load 30.93 N, swing foot 0.50 N, loaded sole front-outside −2.604 mm. Left early knee is −2.2682 Nm at 3.344 s, fraction 0.075, load 30.73 N. A 1.60 rad/s early rate put the right knee back to +2.3405 Nm at 3.096 s, fraction 0.115, load 29.25 N. Mid-SS stays under 2.33 Nm. Right knee is +2.3253 Nm at 2.680 s, fraction 0.580. Left knee is −2.2899 Nm at 3.440 s, fraction 0.540. Right hip pitch is −1.8625 Nm at 3.128 s, fraction 0.275. Left hip pitch is +1.8549 Nm at 6.456 s, fraction 0.275. DSP ankle pitch is left +2.2331 Nm at 2.000 s and right −2.2639 Nm at 2.256 s. The 20–80% swing knee is left +1.4605 Nm at 2.176 s and right −1.4143 Nm at 2.432 s. Swing ankle pitch is left −2.2043 Nm at 3.136 s and right +2.1509 Nm at 6.464 s. Left flat toe is +6.742 mm at 2.104 s, fraction 0.208. Right is +6.762 mm at 2.360 s, fraction 0.208. Window heel-inside is left +0.206 mm at 3.176 s and right +0.367 mm at 3.952 s. The swing-knee peak inside 20–80% is +1.4605 Nm, not the parked ±6.39 Nm channel. CoM mid margin is −5.61 mm, 227 of 227 outside. That is the y_swap 0 class and is not a torque pass. The tip still awaits Dave. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

Whole-gait unclamped scan on that same locked copy, continuous walk and the soft-stop bout. Every leg joint reports its max |ask| before the tip. Post-tip is logged and is not the bar. Prefer FAIL. On the continuous walk there is no tip. The largest pre-tip ask is left knee +6.0744 Nm at 1.000 s, double support, left 6.27 N and right 6.21 N. That same tick is right knee −6.0740 Nm, right hip pitch +2.9451 Nm, left hip pitch −2.9204 Nm, left ankle pitch +2.5388 Nm, and right ankle pitch −2.5193 Nm. From 2.0 s the only joint over 2.33 Nm is right ankle pitch −2.3690 Nm at 2.224 s, phase L, fraction 0.860, right foot 17.31 N, left foot 0.00 N. That fraction is past the mid-stance window. The locked early, mid, 20–80%, toe, and DSP bars still hold on this copy. On the soft-stop bout the largest pre-tip ask is right ankle pitch −7.8537 Nm at 8.144 s, double support, left foot 22.37 N, right foot 0.00 N. That tick is before the stop command at 8.200 s. From the stop command until the tip every leg ask stays at or under 2.33 Nm. The stop stretch stays 1.060 and soft 0.25. Touchdown 8.384 s, snap 8.392 s, tip 8.512 s, margin −0.068, right knee +2.3300 Nm at 8.200 s. Left ankle pitch on that bout is +7.8031 Nm at 7.888 s, right foot 20.64 N and left foot 0.00 N. Right knee is −6.3617 Nm at 2.328 s, fraction 0.115, left foot 23.88 N and right foot 0.00 N. Left knee is +6.3756 Nm at 2.072 s, fraction 0.115, airborne. Left hip pitch is −4.4760 Nm at 7.696 s, fraction 0.075. Post-tip right ankle pitch −8.5621 Nm at 8.878 s is not the bar. CoM mid margin on the walk is −5.61 mm, 227 of 227 outside. On the stop bout it is −7.99 mm, 305 of 312 outside. The next lever is that soft-stop right ankle pitch −7.8537 Nm at 8.144 s. The plant stays cold. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

Airborne ankle pitch on the stop walk, before the stop command, is CLEAR. The ±7.8 Nm ticks at 0 N were a clock-swing slam, not a loaded double-support ankle. While the clock calls that foot swing, ankle pitch steps at 1.90 rad/s and then stays inside 2.33 Nm, including after that foot has already loaded. A loaded double-support ankle stays on the 1.90 rad/s stance cap. Ankle roll stays on its freeze and sole-flat stays on. The worst clock-swing ask is right ankle pitch +2.3300 Nm at 1.416 s, phase R, fraction 0.660, right foot 0.00 N, and left ankle pitch −2.3300 Nm at 4.720 s, phase L, fraction 0.540, left foot 0.00 N. The worst ask with that foot at or under 5 N is right ankle pitch +2.3300 Nm at 1.240 s, double support, right foot 2.81 N. Joint speed alone was +2.4334 Nm there, and a −0.1034 Nm position term puts the signed ask on the bar. The first load after that 0 N right channel is 1.496 s, right foot 14.74 N and left foot 16.79 N. The right corners there are heel-outside +2.356 mm, heel-inside +2.113 mm, front-outside +15.404 mm, and front-inside +15.161 mm. No front corner on that foot is under 0. The first load after the 0 N left channel is 4.808 s, left foot 22.61 N. The left corners there are heel-inside +1.723 mm, heel-outside +2.817 mm, front-inside +24.014 mm, and front-outside +25.108 mm. No front corner on that foot is under 0. After the stop the swing left foot loads at 5.53 N. Touchdown is 8.384 s, swing left 5.53 N, front-inside +21.761 mm, front-outside +21.292 mm, heel-inside +0.246 mm, heel-outside −0.223 mm. That heel is not a toe-down. The stop stretch stays 1.060 and soft 0.25, catch 0.020 s, peak rate 2.014 rad/s. From the stop command until any tip every leg ask stays at or under 2.33 Nm. This copy does not record the parked tip at 8.512 s, margin −0.068. CoM mid margin on the stop bout is −8.11 mm, 319 of 321 outside. That is not a tip pass and is not kit-safe. The continuous-walk bars are unchanged: early right knee +2.2956 Nm, mid right knee +2.3253 Nm, flat toes +6.742 mm and +6.762 mm, entrance left knee +6.0744 Nm at 1.000 s, and steady right ankle pitch −2.3690 Nm at 2.224 s, fraction 0.860. The next lever is that stand-to-walk tick at 1.000 s, both feet near 6 N. It is a loaded transition, a freeze or delayed write, not a swing stretch. On the stop bout the same entrance is left ankle pitch +4.6909 Nm at 1.000 s with both feet loaded. The approach knees at fraction 0.115 stay a later lever. They are not the loaded early-stance slew. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

The continuous-walk stand-to-walk tick at 1.000 s is CLEAR. The first gait command starts from the live joint and steps at the locked stance rate. It is not a swing stretch. A torque hold on that catch-up put the mid-stance right knee at +2.3309 Nm, so the hold stays off. On this copy no knee, hip-pitch, or ankle-pitch ask from 0.98 s through 1.000 s is over 1.5 Nm. At 1.024 s both ankle pitches are ±1.5767 Nm with both feet near 10 N. The locked bars still hold. Early right knee is +2.2957 Nm at 3.096 s, load 29.23 N. Left is −2.2594 Nm at 2.840 s, load 28.96 N. Mid right knee is +2.3242 Nm at 2.680 s, fraction 0.580. Flat toes are +6.722 mm and +6.764 mm. CoM mid margin is −5.62 mm, 227 of 227 outside. The stop bout does not take this seed. Seeding it there drove the approach left knee to +9.1968 Nm at 1.048 s, fraction 0.115. On the stop bout the entrance is still left ankle pitch +4.6909 Nm at 1.000 s, both feet loaded, and right hip pitch +5.7075 Nm at that same tick. Airborne ankle pitch on that bout stays at ±2.3300 Nm. The stop stretch stays 1.060 and soft 0.25. From the stop command until any tip every leg ask stays at or under 2.33 Nm. The walk still has two loaded stance ankles over 2.33 Nm in the first cycle: left −2.3628 Nm at 1.336 s, fraction 0.275, load 26.28 N, and right −2.4471 Nm at 1.720 s, fraction 0.900, load 18.38 N. From 2.0 s the steady right ankle pitch is −2.3634 Nm at 2.224 s, fraction 0.860, right foot 17.31 N. The next lever is the stop-bout approach left knee +6.6915 Nm at 1.560 s, fraction 0.115, left foot 1.34 N. That foot is airborne. It is not the loaded early-stance slew. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

The stop-bout approach knee, slewed like the airborne ankle, is Prefer FAIL. Holding every clock-swing knee inside 2.33 Nm brings the fraction-0.115 left knee to +0.8790 Nm at 7.192 s, left foot 4.99 N, and the worst airborne left knee to −1.3992 Nm at 1.200 s with that foot at 0 N. The first later load of that left foot is 1.208 s at 11.94 N. Its four corners stay at or above +0.446 mm, and no front corner on that foot is under 0. The right airborne knee peaks at +1.5736 Nm and then loads at 8.24 N with every right corner at or above +0.779 mm. The stop stretch stays 1.060 and soft 0.25, catch 0.020 s. From the stop command until any tip every leg ask stays at or under 2.33 Nm. The post-stop touchdown moves to 8.448 s, swing left 7.50 N, and front-outside is −1.243 mm. That is a toe-down. Holding the knee only while the foot is at or under 5 N still digs front-outside to −1.155 mm at 8.432 s, swing left 7.59 N, and the stop-command right ankle roll reaches −2.3839 Nm at 8.968 s. That copy also moves the stretch catch from 0.020 s to 0.029 s. The factor stays 1.060 and soft stays 0.25. Neither hold is kept. The continuous-walk bars do not move. This copy does not record the parked tip at 8.512 s. That is not a tip pass. CoM on the whole-swing hold is −8.16 mm, 316 of 316 outside. The approach knee is still the next lever. The stop-bout entrance at 1.000 s stays behind it. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

Stop-bout approach swing-z is its own schedule. Before the stop command, swing z is the path closest to the live z whose knee and ankle pitch stay at or under 1.55 rad/s and that still ends on the landing z. Peak departure from the live z is 17.51 mm. The delta is off from the stop command, so the 0.25 / 1.060× stretch still samples the unwarped z. The swing knee slews toward that IK at 1.55 rad/s with no torque hold, and that memory is not the stance slew. Ankle roll stays on its freeze and sole-flat stays on. The continuous-walk install stays at 1.55 rad/s and was scored again on the walk bout: early right knee +2.2957 Nm at 3.096 s, load 29.23 N, left early knee −2.2594 Nm at 2.840 s, load 28.96 N, mid right knee +2.3242 Nm at 2.680 s, fraction 0.580, flat toes +6.722 mm and +6.764 mm, steady right ankle pitch −2.3634 Nm at 2.224 s, fraction 0.860, CoM mid margin −5.62 mm, 227 of 227 outside. CLEAR on the approach knee. The fraction-0.115 left knee is −0.1395 Nm at 1.560 s, left foot 0.00 N. The airborne left knee peak is +1.4070 Nm at 1.152 s, fraction 0.620, left foot 0.00 N. The airborne right knee is +1.5428 Nm at 1.832 s, fraction 0.195, right foot 2.78 N. The fraction-0.115 right knee is +0.3707 Nm at 1.304 s, right foot 1.12 N. Post-stop touchdown stays 8.384 s, gait 0.153, after 7 clock waits, swing left 9.28 N. The four corners are heel-inside +2.659 mm, heel-outside +3.096 mm, front-inside +10.850 mm, and front-outside +11.288 mm. All four are at or above 0. Ankle roll q is −0.03129 rad against the pin −0.03224 rad. Sole-flat applies on that write. Stop ankle roll from the command until any tip is left +1.2968 Nm at 8.256 s and right −0.3903 Nm at 8.376 s. The stop stretch stays factor 1.060, soft 0.25, catch 0.020 s, peak rate 2.014 rad/s. Clock-swing ankle pitch stays under 2.33 Nm. Right is +2.1345 Nm at 1.856 s, right foot 0.00 N, and the first reload of that foot is 1.976 s at 5.64 N with that foot's corners at or above +0.416 mm. Left is −2.1462 Nm at 2.104 s, left foot 0.00 N, and the first reload of that foot is 2.200 s at 5.21 N with that foot's corners at or above +0.117 mm. The stop-bout steady left ankle pitch is −3.4240 Nm at 7.968 s, fraction 0.155, left foot 28.43 N. That tick is outside this lever. CoM mid margin on the stop bout is −11.90 mm, 317 of 317 outside. That is not a tip pass and is not kit-safe. The entrance at 1.000 s is still left ankle pitch +4.6909 Nm and right hip pitch +5.7075 Nm, both feet loaded. That is the next lever. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

The stop-bout entrance at 1.000 s is CLEAR. It is the same pattern as the continuous stand-to-walk catch, and it is not that seed. The engage tick freezes planar x/y and sole roll/pitch/yaw on the stand pose and writes the live joint. z stays live. Loaded hip pitch, knee, and ankle pitch then step from that joint at the stop-bout stance rate, 1.90 rad/s. The walk ladder stays off this bout. At 1.000 s the asks are about 0 Nm. Left foot is 11.37 N and right foot is 11.06 N. Left corners are heel-inside −0.840 mm, heel-outside −0.632 mm, front-inside −0.790 mm, and front-outside −0.582 mm. Right corners are heel-outside −0.633 mm, heel-inside −0.840 mm, front-outside −0.582 mm, and front-inside −0.790 mm. Through 1.200 s the worst asks are right knee +2.3170 Nm at 1.080 s, both feet loaded, and right ankle pitch −2.2863 Nm at 1.192 s. Right hip pitch is +1.8817 Nm at 1.040 s. Left hip pitch is −1.6857 Nm at 1.120 s. Left knee is +1.7022 Nm at 1.048 s. Left ankle pitch is −1.5115 Nm at 1.200 s. Approach stays CLEAR. The airborne left knee is +1.4270 Nm at 1.544 s, fraction 0.035, left foot 4.85 N. Fraction 0.115 is −0.1826 Nm at 1.560 s, left foot 0.00 N. The airborne right knee is +1.6386 Nm at 1.832 s, fraction 0.195, right foot 3.29 N. Fraction 0.115 is +0.3969 Nm at 1.304 s, right foot 1.17 N. Post-stop touchdown stays 8.384 s, gait 0.153, after 7 clock waits, swing left 9.11 N. The four corners are heel-inside +2.913 mm, heel-outside +3.283 mm, front-inside +10.576 mm, and front-outside +10.946 mm. All four are at or above 0. Ankle roll q is −0.03201 rad against the pin −0.03224 rad. Sole-flat applies. Stop ankle roll is left +1.2573 Nm at 8.256 s and right −0.3933 Nm at 8.376 s. The stop stretch stays factor 1.060, soft 0.25, catch 0.020 s, peak rate 2.014 rad/s. Clock-swing ankle pitch stays under 2.33 Nm. Right is +2.1930 Nm at 1.856 s, right foot 0.00 N, and the first reload of that foot is 1.976 s at 5.65 N with that foot's corners at or above +0.577 mm. Left is −2.1512 Nm at 2.104 s, left foot 0.00 N, and the first reload of that foot is 2.208 s at 5.26 N with that foot's corners at or above +0.017 mm. The continuous walk was scored again and did not move: walk-z 1.55 rad/s, early right knee +2.2957 Nm at 3.096 s, load 29.23 N, left early knee −2.2594 Nm at 2.840 s, load 28.96 N, mid right knee +2.3242 Nm at 2.680 s, fraction 0.580, flat toes +6.722 mm and +6.764 mm, steady right ankle pitch −2.3634 Nm at 2.224 s, fraction 0.860, right foot 17.31 N, CoM mid margin −5.62 mm, 227 of 227 outside. The first landing double support still has left knee −4.3910 Nm at 1.232 s, both feet loaded. That tick is outside the engage window. The stop-bout steady left ankle pitch is −3.5677 Nm at 7.968 s, fraction 0.155, left foot 21.06 N. That is the next lever, with the continuous steady right ankle if it is still over. CoM mid margin on the stop bout is −11.73 mm, 318 of 318 outside. From the stop command until any tip the scorer prints a torque clear and T_stop 1.064 s. That is not a tip pass. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

The first landing double support at 1.232 s is CLEAR. It is the live-joint hold on the landing knee and hip pitch, and it is not a swing stretch. The landing tick freezes planar x/y and sole roll/pitch/yaw on the last swing pose and writes the live joint. z stays live. The knee and hip pitch then stay within one step of the live joint through that double support. Hip pitch continues on the engage catch from that command. Ankle pitch stays on its slew. At 1.232 s the left foot is 10.95 N and the right foot is 13.35 N. Left corners are heel-inside +0.293 mm, heel-outside −1.278 mm, front-inside +11.103 mm, and front-outside +9.532 mm. Right corners are heel-outside −0.064 mm, heel-inside −3.148 mm, front-outside +2.403 mm, and front-inside −0.681 mm. Engage corners stay on this copy. The lowest corner at 1.000 s is still −0.840 mm on both feet, so this catch does not lift that penetration. Through the catch the worst asks are left knee −2.2947 Nm at 1.400 s, left foot 21.20 N and right foot 0.00 N, and left ankle pitch −2.3292 Nm at 1.320 s, left foot 25.30 N. Left hip pitch is +1.9196 Nm at 1.336 s. Right hip pitch is +1.4751 Nm at 1.360 s. Right knee is −1.4565 Nm at 1.408 s. Right ankle pitch is −2.1640 Nm at 1.232 s. Entrance stays CLEAR, with the same peaks through 1.200 s. Approach stays CLEAR. The airborne left knee is +1.3822 Nm at 2.168 s, fraction 0.580, left foot 0.00 N. Fraction 0.115 is −0.1264 Nm at 1.560 s, left foot 0.00 N. The airborne right knee is −1.4565 Nm at 1.408 s, fraction 0.620, right foot 0.00 N. Fraction 0.115 is +0.4925 Nm at 1.304 s, right foot 0.96 N. Post-stop touchdown stays 8.384 s, gait 0.153, after 7 clock waits, swing left 9.45 N. The four corners are heel-inside +2.826 mm, heel-outside +3.257 mm, front-inside +10.553 mm, and front-outside +10.985 mm. All four are at or above 0. Stop ankle roll is left +1.3034 Nm at 8.256 s. The stop stretch stays factor 1.060, soft 0.25, catch 0.020 s, peak rate 2.014 rad/s. Clock-swing ankle pitch stays under 2.33 Nm, and the first reload of that foot stays at or above 0 on that foot's corners. The continuous walk was scored again and did not move: walk-z 1.55 rad/s, early right knee +2.2957 Nm at 3.096 s, load 29.23 N, left early knee −2.2594 Nm at 2.840 s, load 28.96 N, mid right knee +2.3242 Nm at 2.680 s, fraction 0.580, flat toes +6.722 mm and +6.764 mm, steady right ankle pitch −2.3634 Nm at 2.224 s, fraction 0.860, right foot 17.31 N, CoM mid margin −5.62 mm, 227 of 227 outside. The stop-bout steady left ankle pitch is −3.5603 Nm at 7.968 s, fraction 0.155, left foot 20.17 N. That is the next lever, with the continuous steady right ankle. CoM mid margin on the stop bout is −11.93 mm, 317 of 317 outside. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

The stop-bout steady left ankle pitch at 7.968 s is CLEAR. It is a loaded stance ankle, phase R, fraction 0.155, and it is not a swing stretch. From 2 s until the stop command, a left-ankle step that would put the ask over 2.33 Nm is not taken. The command stays on the nearer side of the live joint. Planar x/y and sole attitude stay on the live step. z stays live. The walk ladder stays at 1.90 rad/s on that ankle. At 7.968 s the ask is −2.3070 Nm, q +0.58121 rad, q_des +0.49727 rad. The left foot is 26.05 N and the right foot is 0.00 N. Left corners are heel-inside −0.402 mm, heel-outside −0.134 mm, front-inside +8.891 mm, and front-outside +9.159 mm. Right corners are heel-outside +6.025 mm, heel-inside +3.097 mm, front-outside +5.173 mm, and front-inside +2.245 mm. Engage corners stay on this copy. The lowest corner at 1.000 s is still −0.840 mm on both feet. Entrance, the first landing, and the approach stay CLEAR, with the same landing knee −2.2947 Nm at 1.400 s. Post-stop touchdown is 8.400 s, gait 0.153, after 9 clock waits, swing left 8.38 N. The four corners are heel-inside +4.549 mm, heel-outside +4.657 mm, front-inside +11.023 mm, and front-outside +11.131 mm. All four are at or above 0. Stop ankle roll is left +1.0434 Nm at 8.232 s. The stop stretch stays factor 1.060, soft 0.25, catch 0.020 s, peak rate 2.014 rad/s. From the stop command until any tip every leg ask stays at or under 2.33 Nm. The continuous walk was scored again and did not move: walk-z 1.55 rad/s, early right knee +2.2957 Nm at 3.096 s, load 29.23 N, left early knee −2.2594 Nm at 2.840 s, load 28.96 N, mid right knee +2.3242 Nm at 2.680 s, fraction 0.580, flat toes +6.722 mm and +6.764 mm, steady right ankle pitch −2.3634 Nm at 2.224 s, fraction 0.860, right foot 17.31 N, CoM mid margin −5.62 mm, 227 of 227 outside. That continuous steady right ankle is the next lever. CoM mid margin on the stop bout is −11.22 mm, 317 of 317 outside. T_stop 1.088 s is the body-speed settle and is not a tip pass. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

The continuous steady right ankle pitch at 2.224 s is CLEAR. It is a loaded stance ankle, phase L, fraction 0.860, and it is not a swing stretch. From 2 s on the continuous walk, an ankle-pitch step that would put the ask over 2.33 Nm is not taken. The command stays on the nearer side of the live joint. z stays live. The walk ladder stays at 1.90 rad/s on that ankle. Walk swing-z stays at 1.55 rad/s. At 2.224 s the ask is −1.8892 Nm, q −0.51628 rad, q_des −0.60125 rad. The right foot is 17.44 N and the left foot is 0.00 N. Left corners are heel-inside +2.248 mm, heel-outside +6.840 mm, front-inside +18.674 mm, and front-outside +23.266 mm. Right corners are heel-outside +0.070 mm, heel-inside −2.825 mm, front-outside +1.618 mm, and front-inside −1.277 mm. The steady peak on that ankle is −2.3166 Nm at 2.208 s, phase L, fraction 0.780, right foot 16.64 N. Early right knee is +2.2950 Nm at 3.096 s, load 29.24 N. Left early knee is −2.2695 Nm at 2.840 s, load 28.93 N. Mid right knee is +2.3248 Nm at 2.680 s, fraction 0.580. Flat toes are +6.722 mm and +6.952 mm. The stop bout was scored again and did not move. Steady left ankle pitch stays −2.3070 Nm at 7.968 s, left foot 26.05 N. The lowest engage corner at 1.000 s is still −0.840 mm on both feet. The landing knee stays −2.2947 Nm at 1.400 s. Post-stop touchdown stays 8.400 s, and the four corners stay at or above +4.549 mm. Stop ankle roll stays left +1.0434 Nm at 8.232 s. The stop stretch stays factor 1.060, soft 0.25, catch 0.020 s, peak rate 2.014 rad/s. The whole-gait scan is not under 2.33 Nm on either bout. On the continuous walk before 2 s, right ankle pitch is −2.4471 Nm at 1.720 s, fraction 0.900, right foot 18.38 N, and left ankle pitch is −2.3628 Nm at 1.336 s. From 2 s that walk stays at or under 2.33 Nm. On the stop bout the pre-steady worst is left hip pitch +3.2259 Nm at 1.512 s, left foot 14.43 N and right foot 9.53 N. From 2 s the worst is right knee +2.8579 Nm at 2.592 s, fraction 0.155, right foot 34.88 N. Also over on that bout: left knee −2.8478 Nm at 2.336 s, right hip pitch −2.6501 Nm at 2.112 s, left hip pitch +2.5880 Nm at 2.880 s, left ankle pitch +2.4275 Nm at 1.456 s, and right knee +2.9649 Nm at 1.552 s. Torque stays open on that scan. Tip and CoM await Dave. CoM mid margin is −5.62 mm on the walk, 227 of 227 outside, and −11.22 mm on the stop, 317 of 317 outside. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

The stop-bout loaded hips are under 2.33 Nm. Both feet are loaded at 1.512 s, so this is a loaded chain and not a swing stretch. After the engage catch meets the gait, the next hip write was the raw gait command. That jump is cut to one stance step from the last command that was actually written, and a step that would put the ask over 2.33 Nm is not taken. An airborne hip stays on the gait command. At 1.512 s the left hip pitch ask is +0.6348 Nm, q −0.74291 rad, q_des −0.75193 rad. The left foot is 11.48 N and the right foot is 11.79 N. Left corners are heel-inside −3.412 mm, heel-outside +1.454 mm, front-inside −0.053 mm, and front-outside +4.814 mm. Right corners are heel-outside −2.035 mm, heel-inside −0.004 mm, front-outside +6.031 mm, and front-inside +8.062 mm. The left hip peak on the bout is +2.3076 Nm at 1.840 s, phase R, fraction 0.235, left foot 29.33 N. The right hip peak is −2.2695 Nm at 1.648 s, phase L, fraction 0.540, right foot 23.38 N. At 2.112 s the right hip is −2.1740 Nm, right foot 23.81 N. At 2.880 s the left hip is +2.0965 Nm, left foot 23.82 N. Engage lowest corner at 1.000 s is still −0.840 mm on both feet. The landing knee stays −2.2947 Nm at 1.400 s. Steady left ankle pitch stays at or under 2.33 Nm; the peak on this copy is +2.3288 Nm at 8.080 s, left foot 16.74 N. Post-stop touchdown is 8.392 s, swing left 6.83 N, and the four corners are heel-inside +5.354 mm, heel-outside +5.816 mm, front-inside +10.969 mm, and front-outside +11.431 mm. Stop ankle roll is left +1.1055 Nm at 8.232 s. The stop stretch stays factor 1.060, soft 0.25, catch 0.020 s, peak rate 2.014 rad/s. Walk swing-z stays 1.55 rad/s. The continuous walk was scored again and did not move. A loaded-knee hold across the whole walk put the named knees on 2.33 Nm and dug the post-stop front-outside under 0, and it reopened the steady left ankle, so that hold is not in this copy. The next overs are loaded knees: right knee +2.8384 Nm at 1.552 s, left foot 5.28 N and right foot 27.12 N; left knee −2.8646 Nm at 2.336 s, left foot 33.65 N; right knee +2.8533 Nm at 2.592 s, right foot 36.08 N. Loaded left ankle pitch is +2.4275 Nm at 1.456 s, left foot 17.09 N. Continuous pre-2 s ankles stay right −2.4471 Nm at 1.720 s, right foot 18.38 N, and left −2.3628 Nm at 1.336 s, left foot 26.28 N. Torque stays open. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

The ±2.3300 Nm writes are Prefer FAIL. Each one solves q_des so the signed ask kp*(q_des−q)−kv*ω equals ±2.33 Nm. That is a hard cap. The unclamped sum |kp*(q_des−q)|+|kv*ω| on that write is over 2.33 Nm. Torque stays open. Tip and CoM await Dave.

Stop right knee at 1.552 s is signed +2.3300 Nm, kp term +3.0361 Nm, kv term −0.7061 Nm, unclamped 3.7423 Nm, hard cap 1. Left foot 5.17 N, right foot 26.82 N. Left corners are heel-inside −2.213 mm, heel-outside +3.382 mm, front-inside +0.288 mm, front-outside +5.883 mm. Right corners are heel-outside −1.964 mm, heel-inside +0.366 mm, front-outside −2.780 mm, front-inside −0.450 mm. Left knee at 2.336 s is signed −2.3300 Nm, kp term −3.6465 Nm, kv term +1.3165 Nm, unclamped 4.9629 Nm, hard cap 1. Left foot 31.94 N, right foot 1.76 N. Left corners are heel-inside −0.861 mm, heel-outside −1.329 mm, front-inside −1.813 mm, front-outside −2.281 mm. Right knee at 2.592 s is signed +2.3300 Nm, kp term +3.5689 Nm, kv term −1.2389 Nm, unclamped 4.8078 Nm, hard cap 1. Right foot 34.75 N, left foot 1.76 N. Left ankle pitch at 1.456 s is signed +2.3300 Nm, kp term +3.3234 Nm, kv term −0.9934 Nm, unclamped 4.3169 Nm, hard cap 1. Left foot 17.03 N. Continuous right ankle pitch at 1.720 s is signed −2.3300 Nm, kp term −3.3537 Nm, kv term +1.0237 Nm, unclamped 4.3774 Nm, hard cap 1. Right foot 18.33 N. Continuous left ankle pitch at 1.336 s is signed −2.3300 Nm, kp term −3.2145 Nm, kv term +0.8845 Nm, unclamped 4.0991 Nm, hard cap 1. Left foot 26.25 N.

Left hip at 1.512 s is signed +0.6309 Nm, unclamped 1.4354 Nm, hard cap 0, both feet loaded (12.74 N and 11.45 N). Right hip at 2.112 s is signed −1.8301 Nm, unclamped 5.1298 Nm, hard cap 0, right foot 23.79 N. Left hip at 2.880 s is signed +2.1872 Nm, unclamped 5.9113 Nm, hard cap 0, left foot 24.35 N. The landing knee stays −2.2947 Nm at 1.400 s, kp term −4.5914 Nm, kv term +2.2967 Nm, unclamped 6.8881 Nm. Steady left ankle pitch is −2.3300 Nm at 2.344 s, the same signed solve, kp term −2.9264 Nm, kv term +0.5964 Nm, unclamped 3.5228 Nm, left foot 28.82 N. Continuous steady right ankle peak is −2.3208 Nm at 2.208 s, kp term −2.8569 Nm, kv term +0.5361 Nm, unclamped 3.3930 Nm, right foot 16.65 N. The 2.224 s tick is −1.8928 Nm.

Walking the command toward the live joint in stance steps until that unclamped sum is at or under 2.33 Nm, with the slew memory kept on the 1.90 rad/s path, reopened the stop catch to 0.044 s, the landing knee to −3.3162 Nm at 1.336 s, and the steady left ankle to −2.6414 Nm at 6.960 s. The first knee tick outside that window asked +5.7090 Nm. That walk is not in this copy. Engage lowest corner at 1.000 s is still −0.840 mm on both feet. Post-stop touchdown is 8.392 s, swing left 7.13 N, gait 0.153, after 8 clock waits. The four corners are heel-inside +5.753 mm, heel-outside +6.628 mm, front-inside +10.722 mm, and front-outside +11.597 mm. Stop ankle roll is left +1.0874 Nm at 8.232 s. The stop stretch stays factor 1.060, soft 0.25, catch 0.020 s, peak rate 2.014 rad/s. Early right knee is +2.2954 Nm at 3.096 s, load 29.24 N. Left early knee is −2.2695 Nm at 2.840 s, load 28.92 N. Mid right knee is +2.3258 Nm at 2.680 s. Flat toes are +6.683 mm and +6.984 mm. Walk swing-z stays 1.55 rad/s. CoM mid margin is −5.66 mm on the walk, 227 of 227 outside, and −11.46 mm on the stop, 318 of 318 outside. d_min is not rebuilt. Not kit-safe. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. Not go-anywhere.

On this same right swing the commanded leading corner is −0.000501 m at 30% (t = 3.400 s, fraction 0.292), +0.011144 m at 50% (t = 3.440 s, fraction 0.500), and +0.019190 m at 70% (t = 3.480 s, fraction 0.708). The live command is still on the floor at 30% and is up at 50% and 70%. That is a late lift. Each `*_foot_contact` is centre z −0.018 m and half-height 0.008 m, so the sole bottom is 0.026 m below the ankle-roll origin. The forward-kinematics local z of that leading corner is −0.026000 m on every tick of the swing. The footprint half is 0.0675 m by 0.0380 m and matches `FOOT_HALF_X` / `FOOT_HALF_Y`. The 1.7 mm drop plus 0.037 rad times a 70 mm arm does not add up to the 10.9 mm. The gap is the live pelvis pose on this same leg target. It is not an ankle-to-sole offset and not a different commanded leg. MFG lock: the IK ankle-to-sole is 26 mm on both the sim and the kit. The outsole is the plant box bottom, so the stack is that 26 mm and not 26 mm plus an outsole. The −3.2 mm is not an outsole stack. The Prefer FAIL lead is the live pelvis against the nominal stand pelvis. The plant file stays cold.

A compliant stance copy, left out of the locked kit, sets the ankle target to the joint the split sole is holding, on 23 ticks. The mid-swing toe stays −0.003066. At 7.424 s the right knee reaches the plant rail, +2.45 Nm, over 2.33 Nm. At 7.520 s the bus flags a tip, support margin −1.064, and min up_z is −1.000. There is no airborne flag at 7.560 s because the body is already over. Walking left ankle pitch peaks at +2.153 Nm at 7.222 s, under 2.33 Nm. CoP max outside the sole is 0.000005 m. Prefer FAIL.

A mid-swing clearance copy adds swing-knee flex while the leading toe is under 2 mm and the phase is inside 20–80%, capped so the commanded knee error stays inside 2.33 Nm at kp 45. The mid-swing toe minimum is −0.003069 at t = 3.384 s, floor normal 13.3 N. The mid-swing p90 is 0.025005 and is report-only. The bus still flags airborne at 7.560 s. Walking knee peak is −2.149 Nm at 7.182 s, under 2.33 Nm. The stop rail after the fault is +2.280 Nm. Walking ankle peak is −1.781 Nm. CoP max outside the sole is 0.000002 m. Prefer FAIL. `gm_z_m` stays 0.020. Forcerange, kp, damping, and armature were not raised. Not kit-safe. Not go-anywhere.

A mid-swing copy adds 0.018 m of hip-frame z on the swing foot during the middle 20–80% of single support, and only there. The commanded leading corner, from that gait target, has a mid-swing minimum of 0.014538 m at t = 2.872 s, 386 ticks. That is above 0.012 m plus the 0.002 m margin. The actual toe at that same tick is −0.003014 m, floor normal 4.6 N, 0.0176 m below the command. The command is clear of the floor and the toe is below it. Walking knee peak is +2.147 Nm at 7.428 s, under 2.33 Nm. Walking ankle peak is −1.781 Nm. After the fault the stop rail is +2.280 Nm. The bus flags airborne at 7.544 s, so 7.560 s is inside that fault. CoP max outside the sole is 0.000005 m. Prefer FAIL.

A split-sole copy drops the flat hip-plus-knee offset on 15 ticks. The ankle command becomes hip plus knee plus the 4.4° tilt, residual 0.076885 rad, inside ±2.09. The mid-swing toe stays −0.003066. The left knee reaches the plant rail, −2.45 Nm, at 7.170 s. The bus flags a tip at 7.448 s, support margin −1.014, and min up_z is −1.000. There is no airborne flag at 7.560 s because the body is already over. Walking left ankle pitch peaks at −2.189 Nm at 7.032 s, under 2.33 Nm. CoP max outside the sole is 0.000005 m. Prefer FAIL. Neither copy is in the locked kit. Not kit-safe. Not go-anywhere.

An earlier-lift copy adds hip-frame z from toe-off through 40% of single support, so the extra is already on at 20–30%. With +0.012 m the 20–30% commanded leading corner minimum is 0.009511 m at t = 3.128 s, 52 ticks, under 0.012 m plus the 0.002 m margin. The actual toe at that tick is +0.000781 m with no floor force. The mid-swing toe minimum is that same 0.000781 m, under 0.002 m. The walking knee reaches +2.350 Nm at 7.426 s, over 2.33 Nm and under the 2.45 Nm rail. Walking ankle peak is −1.781 Nm. The bus flags airborne at 7.552 s, so 7.560 s is inside that fault. CoP max outside the sole is 0.000012 m. Prefer FAIL. Adding 0.013 m already puts the left knee on the plant rail, −2.45 Nm, at 1.128 s, while the 20–30% command is still 0.010495 m. The extra that does clear the command is +0.020 m: the 20–30% commanded minimum is 0.016742 m at t = 2.872 s. The actual toe at that tick is +0.003149 m with no floor force, and the mid-swing toe minimum is that same 0.003149 m, above 0.002 m. The left knee is on −2.45 Nm at 1.124 s. Walking ankle peak is +2.159 Nm at 7.192 s. Airborne is at 7.536 s, so 7.560 s is inside that fault. CoP max outside the sole is 0.000002 m. Prefer FAIL. The 20–30% command does not clear 0.014 m without the knee on the rail. `gm_z_m` stays 0.020. The split-sole drop scored again on this same freeze and the numbers above are unchanged. Not kit-safe. Not go-anywhere.

A −10° `head_tilt` walk is not enabled. On that pose the near edge is
0.077 m at `cam_z` 0.326 m. #67 checked the wood rule at that session
tilt: the foot contact boxes project to rows 643–749, below the
480-row frame, and `foot_wood_px` is 0. The kitchen fires in that
check are the stool only. That does not turn the tilt on for this
gate. This gate does not command `head_tilt`. Putting the head back
to +0.25 rad for a room ask remains a session joint, not a Day-1 bus
key.

A Day-1 stop at the cue (issued at 1.904 s) has 0 prop contacts,
min up_z 0.934, end xy +0.088, −0.005, yaw −5.9°, and a stop peak of
+2.280 Nm on the right knee. That stop is first-sight. It is not this
gate. Soft-pass is off. Not go-anywhere.

Lateral sway is sized off the cold plant, not a fit. Foot box centres are world y ±0.0430 m, x +0.011 m from the trunk root. Each box runs y from ±0.0050 m to ±0.0810 m and x from −0.0565 m to +0.0785 m. Single support needs the ZMP and the CoM at least 5 mm off the midline onto the stance foot before the margin is 0. The box centre is the 43 mm sway. Trunk CoM x +0.0039 m is already inside that fore-aft span, so the lever is lateral. y_swap stays 0. The channel is `preview_y`, the negative of a cart-table preview whose single-support hold is ±0.043 m. The first lift waits until the measured CoM is 8 mm onto that foot. A stop freezes in double support and returns the ZMP to 0 only while both feet stay down. One light foot sends the reference back to that sole's box centre. Soft-pass is off. There is no ±2.33 solve. Forcerange, kp, damping, and armature were not raised. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged.

A fresh Day-1 kit bout, `locked_kit_config` and `BUS_KIT_SCRIPT`, 950 ticks at 8 ms, reproduces the #102 CoM jerk exactly: peak 1048.991 m/s³ and RMS 132.473 m/s³ at t 0.312 s. Joint third-difference on that same bout is right knee 17489.560 / 1795.049 rad/s³, not the published 39843 / 4111. Linvel ZMP on that bout is −0.015238 m, outside fraction 0.234, not −0.095164 / 0.213. The bars stay the published ones. CoM jerk is the comparison that matches.

Kit vx 0.150 m/s, period 2.80 s, dsp 0.55, swing z 8 mm, arm 1.20 s, amplitude 43 mm, 1013 ticks. The box clears and the ask does not. CoM and ZMP outside fraction is 0 on ss_L, ss_R, and double support. Minimum CoM margin is +18.23 mm at 5.144 s, right single support, com_y −0.0275 m against a foot at −0.0475 m. The unclamped peak is left hip pitch 4.4462 Nm at 4.424 s. hard_cap is 0. min up_z is 0.934. Prefer FAIL on the ask.

The slow row is period 6.40 s, dsp 0.70, swing z 4 mm, arm 2.40 s, vx 0.040 m/s, 1800 ticks, dx +0.065 m. CoM minimum margin is +19.91 mm. ZMP minimum is +19.96 mm. Outside fraction is 0. Single-support minima are +22.02 mm left and +22.07 mm right. At the first right single support, com_y is −0.0346 m and the preview command is −0.0426 m. Unclamped peak is right hip roll 1.9792 Nm at 2.816 s, q −0.14153 rad, q_des −0.11368 rad, under 2.33 Nm, 0 ticks over, hard_cap 0. Actuator peak is right ankle pitch 1.2602 Nm. Joint jerk peak/RMS is 5857.085 / 239.084 rad/s³ on the right ankle pitch, under 39843 / 4111. Right knee jerk is 2477.571 / 127.999. CoM jerk RMS is 32.662, under 132.473. min up_z is 0.934. Prefer FAIL. CoM jerk peak is 1048.991 m/s³ at 0.312 s, the same stand impact as #102, so the whole-bout peak is not below the baseline. After the stand the peak is 538.605. Not kit-safe. Not go-anywhere.

The 1048.991 m/s³ peak is the toe landing, not a sample to drop. At spawn the 15° hip-pitch offset leans the trunk and the ankle does not match it, so two sole corners sit on the floor and the other two are 35.0 mm up. Controls is already holding the stand: ctrl equals q_stand, and the knee error stays near 0.015 rad, under the 2.33 Nm bar. From t 0 to 0.296 s those high corners descend. At t 0.304 s contact count goes from 4 to 8 and the normal jumps from about 10 N to 21 N a foot. The third difference of subtree CoM at t 0.312 s is 1048.991 m/s³, of which 850 m/s³ is vertical. The same peak is on the kit bout. The scoring window is the whole bout.

Preview only, kit sole_level stays 0. The stand sets sole_level to 1, which adds the hip-pitch offset onto the ankle with the joint sign, and the spawn sole span is 0. The arm smoothersteps sole_level from 1 to 0 before the first lift, so the walk pitch is the kit pitch again. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. No keyframe edit. No rail raise. y_swap stays 0. Soft-pass is off. There is no ±2.33 solve on the preview write.

Slow row again, same period 6.40 s, dsp 0.70, swing z 4 mm, arm 2.40 s, vx 0.040 m/s, 1800 ticks, dx +0.056 m. CLEAR. CoM and ZMP minimum margin is +21.64 mm at t 0.168 s, double support. Outside fraction is 0. Single support is +22.02 mm left (0/99) and +22.07 mm right (0/190). Unclamped peak is right hip roll 1.9956 Nm at 2.816 s, q −0.14133 rad, q_des −0.11330 rad, 0 ticks over, hard_cap 0. Joint jerk is right knee 1250.408 / 55.487 rad/s³ at 0.408 s, under 39843 / 4111. CoM jerk peak is 88.853 m/s³ at t 0.024 s, RMS 3.722, under 1048.991 / 132.473. After the stand the peak is 56.011. min up_z is 0.965. Soft stop keeps the box: CoM and ZMP minimum on the stop is +26.86 mm at 11.696 s, right single support, and the stop ask is right ankle roll 0.8200 Nm at 14.392 s.

Voice vx +0.056 m/s, same amplitude 43 mm, dsp 0.70, arm 2.40 s, swing z 4 mm, stand 0.40 s, walk 11.00 s, stop 3.00 s. Each row is 1800 ticks. Margins stay positive and the jerk stays under #102 on every row below. The ask is the cut.

- Period 6.40 s. CLEAR. CoM/ZMP minimum +21.64 mm, outside fraction 0. Single support +22.23 / +22.28 mm. Unclamped right hip roll 2.0013 Nm at 2.816 s. CoM jerk 88.853 at 0.024 s. Stop margin +26.80 mm, stop ask right ankle roll 0.8413 Nm. dx +0.061 m.
- Period 4.60 s. CLEAR. CoM/ZMP minimum +21.64 mm, outside fraction 0. Single support +21.83 / +21.89 mm. Unclamped right hip roll 1.8699 Nm at 9.672 s. CoM jerk 88.853. Stop margin +25.83 mm, stop ask left ankle roll 1.0881 Nm. dx +0.064 m.
- Period 3.70 s. CLEAR. CoM/ZMP minimum +21.42 mm, outside fraction 0. Single support +21.42 / +21.50 mm. Unclamped right hip roll 2.2619 Nm at 8.352 s. CoM jerk 88.853. Stop margin +47.72 mm, stop ask left knee 1.5982 Nm. dx +0.067 m.
- Period 3.60 s. CLEAR. This is the fastest row that holds every bar. CoM/ZMP minimum +21.38 mm, outside fraction 0. Single support +21.38 / +21.46 mm. Unclamped right hip roll 2.3135 Nm at 8.176 s, q −0.00845 rad, q_des +0.02372 rad, omega +0.6028 rad/s, 0 ticks over, hard_cap 0. Joint jerk right ankle pitch 8367.011 / 222.345 rad/s³ at 14.376 s, under 39843 / 4111. Right knee jerk 1250.408 / 73.141. CoM jerk peak 395.309 m/s³ at 14.384 s, RMS 12.015, under 1048.991 / 132.473. Stop CoM/ZMP minimum +24.82 mm at 11.728 s. Stop ask right ankle pitch 2.2713 Nm at 14.392 s. The measured actuator on that settle is left ankle pitch 2.2800 Nm at 14.376 s. dx +0.065 m.
- Period 3.50 s. Prefer FAIL. Margins stay inside, minimum +21.31 mm, outside fraction 0. CoM jerk 88.853. The walk ask is right hip roll 2.3682 Nm at 8.040 s. The stop ask is left knee 3.2970 Nm at 11.960 s, q +0.87098 rad, q_des +0.83144 rad, omega −1.0415 rad/s, 112 ticks over. hard_cap 0.
- Period 2.80 s, swing z still 4 mm. Prefer FAIL. Margins stay inside, minimum +20.76 mm, outside fraction 0. CoM jerk 88.853. Unclamped right hip roll 2.8272 Nm at 7.000 s, omega +0.7353 rad/s, 571 ticks over. Stop ask left hip pitch 2.1967 Nm stays under 2.33 Nm.
- Period 4.60 s, swing z 6 mm. Prefer FAIL. Margins stay inside, minimum +20.40 mm, outside fraction 0. CoM jerk 88.853. Unclamped right knee 2.5348 Nm at 11.088 s, 34 ticks over. The 4 mm swing is the one that holds.

Kit vx 0.150 m/s remains the earlier Prefer FAIL, left hip pitch 4.4462 Nm. Not kit-safe. Not go-anywhere.

The 3.60 s right-hip-roll peak of 2.3135 Nm was the cosine. A raised cosine spends the double support at pi/2 times the mean sway rate. `preview_shape` 1 is that cosine. `preview_shape` 0 is a straight ramp over the same double support, which is the lowest peak rate for a monotone transfer. Preview R from 1e-4 to 1e-2 only moved the 3.60 s shape-1 ask from 2.3135 Nm to 2.2924 Nm. Dropping the sway from 43 mm to 40 mm at shape 1 left the 3.70 s ask at 2.2027 Nm, 0.0027 Nm over 2.20. Raising dsp from 0.70 to 0.80 shortened the swing and put the 3.60 s ask at 3.0913 Nm. None of those is the lever. The ramp is.

Period 3.57 s, dsp 0.70, amplitude 43 mm, swing z 4 mm, arm 2.40 s, preview R 1e-4, preview_shape 0, vx 0.056 m/s. `gait_for_command` selects this walk for vel(vx ≤ 0.056, yaw_rate). yaw_rate stays on that gait as cycle yaw. A faster forward command stays on the locked kit. Stand before the step is the level sole. Stop zeros the bus speed and returns through the preview soft stop. y_swap stays 0. Soft-pass is off. There is no ±2.20 solve and no ±2.33 solve. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged.

The 10 Hz bus bout is stand 0.40 s, vel(0.056, 0) for 11.00 s, stop 3.00 s, 1800 ticks. ScriptedDriver resends vel at 10 Hz. applied_vx peaks at +0.0560 m/s and is still +0.0560 m/s in the last second of the walk. CLEAR against #102 and against 2.20 Nm. CoM and ZMP minimum is +17.93 mm at 11.784 s, stop, right single support, outside fraction 0. Single support is +20.82 mm left (0/91) and +17.93 mm right (0/183). Double support is +21.64 mm. Unclamped peak is right knee 2.0695 Nm at 9.248 s, q −0.93895 rad, q_des −0.90731 rad, omega +0.4433 rad/s, 0 ticks over, hard_cap 0. Headroom to 2.20 Nm is +0.1305 Nm. Headroom to 2.33 Nm is +0.2605 Nm. Stop ask is left hip roll 1.7921 Nm at 11.728 s. Joint jerk is right knee 1250.408 / 60.598 rad/s³ at 0.408 s, under 39843 / 4111. CoM jerk peak is 88.853 m/s³ at 0.024 s, RMS 5.747, under 1048.991 / 132.473. After the stand the peak is 56.011. min up_z is 0.960. dx +0.068 m. Gate holds 0. ik_fail 0.

- Period 3.60 s, same ramp. CLEAR. CoM/ZMP minimum +19.00 mm at 11.784 s, stop, right single support. Single support +20.87 / +19.00 mm. Unclamped right knee 2.0572 Nm at 9.256 s. Headroom to 2.20 Nm is +0.1428 Nm. Headroom to 2.33 Nm is +0.2728 Nm. Stop ask left hip roll 1.8342 Nm at 11.736 s. The shape-1 row at this period was right hip roll 2.3135 Nm, which is 0.1135 Nm over 2.20 and 0.0165 Nm under 2.33.
- Period 3.70 s, same ramp. CLEAR. CoM/ZMP minimum +20.93 mm at 5.616 s, walk, left single support. Single support +20.93 / +21.00 mm. Unclamped right knee 2.0212 Nm at 9.472 s. Headroom to 2.20 Nm is +0.1788 Nm. Headroom to 2.33 Nm is +0.3088 Nm. Stop ask right hip pitch 1.3957 Nm at 11.400 s. The shape-1 row at this period was right hip roll 2.2619 Nm, which is 0.0619 Nm over 2.20 and 0.0681 Nm under 2.33.
- Period 3.56 s, shape 0. Prefer FAIL. The walk knee is 2.0707 Nm. The stop left knee is 2.4627 Nm at 11.936 s, over 2.20 and over 2.33. 3.57 s is the shortest period that holds.

Robustness on that 3.57 s row and on the 3.60 s row, same ramp, vx 0.056 m/s, whole bout, soft-pass off. A cell holds only when CoM and ZMP margins stay ≥0, the outside fraction is 0, the unclamped ask stays ≤2.33 Nm, hard_cap is 0, and the bout does not tip. Mass, friction, the mat, and the hinge seed are applied to the loaded MjModel. The plant file is not written. md5 stays `207f3d5e9c6a72e16f7aa0c8d224f75e`. Ten seeds draw hinge q at σ 0.002 rad and qd at σ 0.01 rad/s. The free joint is left on the seated stand. Friction sets the sliding coefficient of the floor and both foot boxes to 1.2, 1.4, or 1.6. Latency +1 holds the previous ctrl for one planner tick. Latency −1 leads by one tick. The entrance mat is `mat_rug`, slid so its near edge is 15 mm ahead of the toes, top left at 20 mm, and counted as ground. Start/stop is three bus cycles of the 11 s vel and the 3 s soft stop. Numbers are in `previews/com_zmp_robust.json`.

| Period | Cell | Ask Nm | Joint | t | CoM mm | ZMP mm | CoM/ZMP out | Overs | hard_cap | up_z | Result |
| ---: | --- | ---: | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- |
| 3.57 | seed 0 | 2.0693 | r_knee | 9.248 | +17.93 | +17.93 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.57 | seed 1 | 2.0694 | r_knee | 9.248 | +17.93 | +17.93 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.57 | seed 2 | 2.0692 | r_knee | 9.248 | −9.65 | +17.93 | 1/0 | 0 | 0 | 0.960 | FAIL |
| 3.57 | seed 3 | 2.0697 | r_knee | 9.248 | +17.92 | +17.92 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.57 | seed 4 | 2.0698 | r_knee | 9.248 | −10.33 | +17.92 | 1/0 | 0 | 0 | 0.960 | FAIL |
| 3.57 | seed 5 | 2.0693 | r_knee | 9.248 | +17.93 | +17.93 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.57 | seed 6 | 2.0692 | r_knee | 9.248 | +17.93 | +17.93 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.57 | seed 7 | 2.0698 | r_knee | 9.248 | +17.92 | +17.92 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.57 | seed 8 | 2.0698 | r_knee | 9.248 | +17.92 | +17.92 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.57 | seed 9 | 2.0693 | r_knee | 9.248 | +17.93 | +17.93 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.57 | mass −5% | 4.0224 | l_hip_roll | 14.272 | −2.11 | −2.11 | 2/2 | 84 | 0 | 0.960 | FAIL |
| 3.57 | mass +5% | 2.0801 | r_knee | 9.248 | +20.85 | +20.85 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.57 | friction 1.2 | 2.0546 | r_hip_pitch | 9.032 | +17.35 | +17.35 | 0/0 | 0 | 0 | 0.963 | CLEAR |
| 3.57 | friction 1.4 | 2.0345 | l_hip_pitch | 10.824 | +17.33 | +17.33 | 0/0 | 0 | 0 | 0.961 | CLEAR |
| 3.57 | friction 1.6 | 2.0695 | r_knee | 9.248 | +17.93 | +17.93 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.57 | latency +1 | 2.2164 | r_knee | 9.248 | +18.36 | +18.36 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.57 | latency −1 | 1.9354 | l_knee | 0.416 | +20.81 | +20.81 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.57 | entrance rug | 8.0444 | r_ank_pitch | 13.144 | no support | +20.82 | 5/0 | 22 | 0 | 0.957 | FAIL |
| 3.57 | start/stop ×3 | 2.0695 | r_knee | 9.248 | +17.93 | +17.93 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | seed 0 | 2.0570 | r_knee | 9.256 | +19.00 | +19.00 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | seed 1 | 2.0571 | r_knee | 9.256 | +19.00 | +19.00 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | seed 2 | 2.0569 | r_knee | 9.256 | −9.65 | +19.00 | 1/0 | 0 | 0 | 0.960 | FAIL |
| 3.60 | seed 3 | 2.0574 | r_knee | 9.256 | +19.00 | +19.00 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | seed 4 | 2.0576 | r_knee | 9.256 | −10.33 | +18.99 | 1/0 | 0 | 0 | 0.960 | FAIL |
| 3.60 | seed 5 | 2.0570 | r_knee | 9.256 | +19.00 | +19.00 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | seed 6 | 2.0569 | r_knee | 9.256 | +19.00 | +19.00 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | seed 7 | 2.0575 | r_knee | 9.256 | +18.99 | +18.99 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | seed 8 | 2.0575 | r_knee | 9.256 | +18.99 | +18.99 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | seed 9 | 2.0570 | r_knee | 9.256 | +19.00 | +19.00 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | mass −5% | 2.0443 | r_hip_pitch | 9.040 | +18.66 | +18.66 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | mass +5% | 2.0674 | r_knee | 9.256 | +20.90 | +20.90 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | friction 1.2 | 2.0548 | r_hip_pitch | 9.064 | +17.99 | +17.99 | 0/0 | 0 | 0 | 0.963 | CLEAR |
| 3.60 | friction 1.4 | 2.0240 | r_hip_pitch | 9.040 | +17.95 | +17.95 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | friction 1.6 | 2.0572 | r_knee | 9.256 | +19.00 | +19.00 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | latency +1 | 2.2021 | r_knee | 9.256 | +18.95 | +18.95 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | latency −1 | 1.9354 | l_knee | 0.416 | +20.86 | +20.86 | 0/0 | 0 | 0 | 0.960 | CLEAR |
| 3.60 | entrance rug | 8.0479 | r_ank_pitch | 13.144 | no support | +20.87 | 5/0 | 22 | 0 | 0.957 | FAIL |
| 3.60 | start/stop ×3 | 2.0572 | r_knee | 9.256 | +19.00 | +19.00 | 0/0 | 0 | 0 | 0.960 | CLEAR |

The worst cell is the 3.60 s entrance rug. Right ankle pitch 8.0479 Nm at 13.144 s, stop, q −0.47657 rad, q_des −0.57790 rad, omega +3.7575 rad/s, 22 ticks over, hard_cap 0. At that tick the support is air, foot normals 2.79 N and 0.15 N. Rug normal peaks at 5.46 N. min up_z is 0.957, so it does not tip. The 3.57 s rug is the same miss, 8.0444 Nm at the same time. Seeds 2 and 4 miss on the spawn tick only: CoM −9.65 mm and −10.33 mm at t 0.000 s, one foot under 5 N, and the walk ask stays at the nominal knee. Mass −5% (2.230 kg) holds on 3.60 s at right hip pitch 2.0443 Nm and fails on 3.57 s: left hip roll 4.0224 Nm at 14.272 s, CoM and ZMP −2.11 mm at 14.304 s, 84 ticks over. Mass +5%, friction 1.2/1.4/1.6, both latencies, and three start/stop cycles hold ≤2.33 Nm with margins ≥0. The +1 tick delay is the thin clear: 2.2164 Nm at 3.57 s and 2.2021 Nm at 3.60 s, under 2.33 Nm and over the 2.20 Nm headroom. No cell raises hard_cap. Not kit-safe. Not go-anywhere.

The same gait on a 20.00 s bus bout, stand 3.00 s, walk 11.00 s, stop 6.00 s, 2500 ticks, keeps the walk phase at the stop. applied_vx is +0.0560 / +0.0560 m/s. Unclamped peak is the same right knee 2.0695 Nm, now at 11.848 s. CoM/ZMP minimum is the same +17.93 mm, now at 14.384 s. CoM jerk stays 88.853. The clip is `docs/media/voice_vx_056_side_front.mp4`, side and front, 30 fps. Not kit-safe. Not go-anywhere. Kit vx 0.150 m/s is still left hip pitch 4.4462 Nm.

