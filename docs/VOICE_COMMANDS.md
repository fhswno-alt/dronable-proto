# Voice commands (Day1 bus)

Typed phrases become `stand`, `stop`, or `vel(vx, yaw_rate)` on the `CommandBus` in `scripts/steer_walk.py`. The caller does not own a second gait. `vel` is resent at 10 Hz. If nothing arrives for 200 ms, the bus stands. The clip runs the kit gait (`locked_kit_config`). This caller does not raise Controls' clamps.

The clip types the phrases. A speech front-end would hand the same text over. No speech API, microphone, or GPU is required.

## Phrases Dave can say

| Phrase | Bus command |
| --- | --- |
| stand, stand up, stand still | `stand` |
| stop, halt, freeze, stop walking | `stop` |
| walk forward, go forward, move forward, walk | `vel(+0.150, +0.000)` |
| back up, reverse, walk backward | `vel(-0.032, +0.000)` |
| turn left, go left, yaw left | `vel(+0.150, +0.250)` |
| turn right, go right, yaw right | `vel(+0.150, -0.250)` |
| turn left in place, spin left | `vel(+0.000, +0.250)` |
| turn right in place, spin right | `vel(+0.000, -0.250)` |

`please` may sit on either end. A polite prefix does not change the bus command.

A turn while walking is walk-yaw at the forward cap: `vel(+0.150, ±0.250)`. A signed turn in place is `vel(0, ±0.250)`. On this kit row that command does change heading (about **+42.6 deg** in 4 s at `+0.25` rad/s, min up_z **0.958**, no tip). The command is not the realized heading rate. Controls' settled rates at `±0.25` rad/s with forward stick stay **+0.217 / −0.221** rad/s.

## Refused (Prefer FAIL)

| Phrase | Line |
| --- | --- |
| go to the kitchen, bathroom, any room | voice does not go to a room; kitchen and bathroom finders are separate, and this caller has no map |
| go anywhere, explore, map, waypoint, SLAM | no map, no SLAM, and no go-anywhere |
| strafe, sidestep, move left, walk right | no vy and no strafe on this bus |
| spin, turn around, turn in place | say turn left in place or turn right in place |
| walk faster, sprint | forward cap is +0.150 m/s; this caller does not raise it |
| door, joint, knee | not a day-1 velocity command |

Caps stay `vx` **+0.150 / −0.032** m/s and yaw **±0.25** rad/s. `0.080` m/s is under the forward clamp. It is not a phrase and it is not a tip claim. This caller does not raise the clamps. The command is not the realized speed.

## Clip

`MUJOCO_GL=osmesa python scripts/voice_caller.py --clip` plays the nav-left window as phrases: **stand → walk forward → turn left → walk forward → stop**. Times are 1 s, 15 s, 12.5 s, 6 s, then a 2.5 s stop. The bus commands are `stand`, `vel(+0.150, +0.000)`, `vel(+0.150, +0.250)`, `vel(+0.150, +0.000)`, `stop`. Each frame captions the phrase and that bus command. The view is the third-person camera so the body is visible. `kit_cam` stays at `0.050 0.019 0.007` and is not the demo camera.

Prefer FAIL on plant `207f3d5e9c6a72e16f7aa0c8d224f75e`, kit row, video off. The stale envelope on plant `71b2c86d…` was approach **+0.598 m**, left arc **+75.7 deg**, resume **+0.317 m**, min up_z **0.954**. The later CPG-basin **+0.695 m / +61.1 deg** is also not this clip.

| Window | Command | New measure | Stale claim |
| --- | --- | --- | --- |
| Approach 15 s | `vel(+0.150, 0)` | heading **+2.210 m**, Δyaw **+1.4 deg**, mean body vx **+0.139 m/s** | +0.598 m |
| Left 12.5 s | `vel(+0.150, +0.25)` | Δyaw **+145.9 deg** | +75.7 deg |
| Resume 6 s | `vel(+0.150, 0)` | heading **+0.944 m**, drift **+12.4 deg** | +0.317 m, treated as straight |
| Whole clip | stand at the end | min up_z **0.934**, no tip, no fault | min up_z 0.954 |

The resume is not a pure straight. Soft-pass is off. This is not nav-multi, not a kitchen arrival, not go-anywhere, and not a demo-ready human walk.

```bash
python scripts/voice_caller.py "turn left"
python scripts/voice_caller.py --self-test
MUJOCO_GL=osmesa python scripts/voice_caller.py --measure
MUJOCO_GL=osmesa python scripts/voice_caller.py --clip
```
