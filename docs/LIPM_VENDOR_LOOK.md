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
stand command through the force limit on that same tick, with the knee
budget at 2.33 Nm. They do not slew through the walking target.

| Window | Settled body vx | Knee peak | Other peak |
| --- | ---: | ---: | ---: |
| Forward, command +0.150 | +0.150 m/s | right +2.173 Nm | hip roll −2.128 Nm |
| Stop from that walk | — | both knees +2.330 Nm | ankle pitch +2.401 Nm |

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
`vel(0, +0.25)` yaws at +0.213 rad/s.

Nav-left resume heading on tip `d00efbf` (Prefer FAIL). Scenario: 1 s
stand, 15 s forward, 12.5 s `vel(+0.150, +0.25)`, then 6 s
`vel(+0.150, 0)`. Resume heading **+0.904 m** / **+2.24 deg**. On
`08731c0` this resume was **+2.57 deg**. The pre-outside-lead resume
was **+12.41 deg**. The no-ask chain mid is **+2.24 deg** and the
chained resume is **−3.66 deg**. Plant md5
`207f3d5e9c6a72e16f7aa0c8d224f75e` unchanged. Soft-pass is off. The
pre-outside-lead split (slew then leftover curve) is not this row.

Left versus right unload, same plant, before the outside-foot resume.
The 6 s `vel(+0.150, 0)` after the left turn was +12.3°: +6.7° while
`applied_yaw` slews +0.25→0 in 0.69 s (command integral +5.3°), then
+5.6° with the yaw command and the step angle already 0. The 6 s
`vel(+0.150, 0)` after an 11 s `vel(+0.150, −0.25)` was −2.2° on that
chain and −0.6° when that right turn follows a straight approach. The
live chain is the README table. The slew is 0.40 rad/s² both
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
R 1.152 Nm. Both are under 2.33 Nm. The later walk still sits on
2.330 Nm. After a yaw target returns to 0, the next double support
swings the outside foot: right after +yaw, left after −yaw. Doing
that by parking the clock on the other double support in one tick
stepped the joint targets 0.342 rad (both shoulders). A normal
published walk tick is mean 0.023 rad, p95 0.047 rad, max 0.063 rad.
The pose now chases the live gait over that double support, 0.056 s,
7 ticks. The largest tick in the chase is 0.062 rad, the same size as
the walk's own largest tick. The post-left 6 s with that chase is
+2.1°. The post-right 6 s does not move the clock and is −0.5°.

The commanded turns on those windows are not the same length. Left is
12.5 s (16→28.5 s) at +0.25 rad/s and finishes at +145.0°, mean body
rate +0.202 rad/s. Right is 11.0 s (16→27.0 s) at −0.25 rad/s and
finishes at −136.5°, mean body rate −0.217 rad/s. The right arc is
8.5° shorter. The right rate is the higher of the two. The gap is the
1.5 s shorter right hold, not a weak right gain and not an outsole
peel. After the straight resume the leftover is +2.1° on the left and
−0.5° on the right. Knees on the walks stay on 2.330 Nm. First-step
knees stay under 2.33 Nm. Soft-pass is off.

Head-tilt ask-pose is not adopted. Joint `head_tilt` / actuator `head_tilt_pos` exists. A 0.20 s stand hold at **+0.25 rad** (near-level look, camera pitch −0.99 deg versus −14.84 deg at the quiet stand) still answers kitchen, bathroom, and bedroom as living. The bus was not given a joint command. The plant file was not edited. Controls does not own an ask-pose from this probe.

Mesh-room remeasure, same pin, same prompt. Straight locked-kit walk, `head_tilt` 0: pitch **−14.86 deg**, near floor edge **0.142 m**. `head_tilt` **−10 deg**: pitch **−24.83 deg**, near edge **0.077 m**. `head_tilt` **−15 deg**: pitch **−29.79 deg**, near edge **0.049 m**. Head torque about **0.010 Nm**. Min up_z **0.934**. Kit-stand asks: `head_tilt` 0, −10 deg, and restore +0.25 rad (pitch **−0.66 deg**) each label kitchen, bathroom, bedroom, and living as themselves. Not a go-to. Soft-pass is off. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`.

Scene frustum at that quiet stand, for a stop-pose Prefer FAIL. The ask stand is xy **+0.057 m, +0.000 m**, yaw **+0.01 deg**, eye **0.335 m**, pitch **−14.84 deg**. The current toilet is already in frame (29450 px, a box, bearing **−41.2 deg**, clipped at the right edge). The current bed is already in frame (114667 px, a box, bearing **0.0 deg**). The stove is not in the current kitchen file. Heading **−41.2 deg** at this xy would center the box toilet. The bed needs no yaw. No heading adds a stove. The mesh rooms behind the four SmolVLM-correct kit stills (`bb94512`), at this same eye, already show stove 10430 px at bearing **+34.2 deg**, toilet 3818 px at **+2.7 deg**, and bed 20901 px at **+5.2 deg**. Controls owns the stop. This probe does not command a new one, and it does not edit the plant or the bus.

Kitchen stool legs are in `kit_cam` before the foot hits. On the locked-kit 8 s `vel(+0.150, −0.25)` the leg band of `chair_stool_b` is `tiny` at the end of the stand (2012 px, short side 96, t = 1.00 s) and `visible_enough` at t = 1.90 s, **4.94 s** before `l_ank_roll_link` meets `col_chair_stool_b_leg_2` at t = 6.85 s. The whole mesh is `visible_enough` from t = 1.00 s (**5.85 s** of lead). Controls owns stop and steer. This probe does not send one, and it does not edit the plant.

The cue is now wired to Day1 `stop` in `scripts/stool_leg_stop.py`. `t_cue` is **1.90 s**. The **28.8 ms** span is the visual-mesh segmentation and depth read, not kit `T_detect`. No Moondream. The sim clock does not move in that span, so `t_stop` is **1.90 s**. The 4.94 s cue-to-contact lead is not detection time. `T_stop` is not measured here. Held to t = 9.0 s on plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`: 0 prop contacts, min up_z **0.934**, knees **2.060 Nm** and **1.960 Nm**. Not a go-to. Soft-pass is off.

Kit `T_detect_rgb` is the RGB-only wood rule in `scripts/stool_rgb_detect.py`: **0.342 ms** on the `t_cue` frame through `CommandBus.stop`. The rule is already positive at t = 1.60 s while the leg band is still tiny (1968 px), 3/9 frames before `t_cue`, and 0/50 misses from `t_cue` to contact. `d_min` 0.129 m against `v × (T_detect_rgb + T_stop)` = 0.125 m uses Controls' `T_stop` 0.830 s and is not a cue pass. Not a go-to. Soft-pass is off.

