# Voice commands (8 PM demo)

Typed phrases become `stand`, `stop`, or `vel(vx, yaw_rate)` on the `CommandBus` in `scripts/steer_walk.py`. The caller does not own a second gait. `vel` is resent at 10 Hz. If nothing arrives for 200 ms, the bus stands.

The clip types the phrases. A speech front-end would hand the same text over. No speech API, microphone, or GPU is required.

## Phrases Dave can say

| Phrase | Bus command |
| --- | --- |
| stand, stand up, stand still | `stand` |
| stop, halt, freeze, stop walking | `stop` |
| walk forward, go forward, move forward, walk | `vel(+0.056, +0.000)` |
| back up, reverse, walk backward | `vel(-0.032, +0.000)` |
| turn left, go left, yaw left | `vel(+0.028, +0.250)` |
| turn right, go right, yaw right | `vel(+0.028, -0.250)` |

`please` may sit on either end. A polite prefix does not change the bus command.

A turn is soft walk-yaw: half the forward cap, plus yaw at the cap. That is the envelope that changes heading on this plant. `vx = 0` with a yaw command does not change heading, so it is not published.

## Refused (Prefer FAIL)

| Phrase | Line |
| --- | --- |
| go to the kitchen, bathroom, any room | voice does not go to a room; kitchen and bathroom finders are separate, and this caller has no map |
| go anywhere, explore, map, waypoint, SLAM | no map, no SLAM, and no go-anywhere |
| strafe, sidestep, move left, walk right | no vy and no strafe on this bus |
| turn left in place, spin, turn around | vx=0 yaw does not change heading; say turn left or turn right |
| walk faster, 0.080 | forward cap is +0.056 m/s; 0.080 tips |
| door, joint, knee | not a day-1 velocity command |

Caps stay `vx` **+0.056 / −0.032** m/s and yaw **±0.25** rad/s. This caller does not raise them. The command is not the realized speed or the heading rate.

## Clip

`MUJOCO_GL=osmesa python scripts/voice_caller.py --clip` plays one steer on the frozen plant: **stand → walk forward → turn left → stop**. The frame is the third-person view so the body is visible. Each frame captions the phrase and the bus command. `kit_cam` stays at `0.050 0.019 0.007` and is not the demo camera.

Measured approach, heading, tip, and fault are written to `previews/voice_commands_summary.json` by that run. Those numbers are the clip. They are not a kitchen arrival and not a go-anywhere claim.

```bash
python scripts/voice_caller.py "turn left"
python scripts/voice_caller.py --self-test
MUJOCO_GL=osmesa python scripts/voice_caller.py --measure
MUJOCO_GL=osmesa python scripts/voice_caller.py --clip
```
