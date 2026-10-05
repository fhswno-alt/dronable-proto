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
| turn left, go left, yaw left | `vel(+0.056, +0.250)` |
| turn right, go right, yaw right | `vel(+0.056, -0.250)` |

`please` may sit on either end. A polite prefix does not change the bus command.

A turn is walk-yaw at the forward cap: `vel(+0.056, ±0.250)`. `vx = 0` with a yaw command does not change heading, so it is not published. `vel(+0.028, yaw)` is the finder trim on the kitchen script. It is not this clip. The command is not the realized heading rate.

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

`MUJOCO_GL=osmesa python scripts/voice_caller.py --clip` plays the claimed nav-left window as phrases: **stand → walk forward → turn left → walk forward → stop**. Times are 1 s, 15 s, 12.5 s, 6 s, then a 2.5 s stop. The bus commands are `stand`, `vel(+0.056, +0.000)`, `vel(+0.056, +0.250)`, `vel(+0.056, +0.000)`, `stop`. Each frame captions the phrase and that bus command. The view is the third-person camera so the body is visible. `kit_cam` stays at `0.050 0.019 0.007` and is not the demo camera.

The voice clip on main matched the previous gait basin: approach **+0.598 m**, left arc **+75.7 deg**, resume **+0.317 m**, min up_z **0.954**. This step-cycle basin uses the same phrases and the same bus, and it does not match that envelope. The remeasured walk is in `docs/DEMO_8PM_MOTION.md` (approach **+0.695 m**, left arc **+61.1 deg**, resume **+0.340 m**, min up_z **0.940**). `vx = 0` yaw still does not change heading.

Locked-kit nav-left resume heading (Prefer FAIL). The phrases above still publish `vel(+0.056, …)`. On the locked kit CommandBus the same window shape is `vel(+0.150, …)`. Scenario: 1 s stand, 15 s forward, 12.5 s `vel(+0.150, +0.25)`, then 6 s `vel(+0.150, 0)`. Resume Δyaw ≈ +12.41°. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` unchanged. Split: (1) 0–0.68 s `applied_yaw` still slewing +0.25→0 at 0.40 rad/s² after the 100 ms resend clears the target → body +6.81° (command integral ~+5.22°); (2) 0.68–6.0 s `applied_yaw` and the step angle are already 0 → leftover left curve +5.60° (body yaw rate +0.034→+0.009 rad/s). That is ~half ramp-out, ~half steady leftover curve under `vel(+0.150, 0)` — not “turn still commanded.” Soft-pass is off.

This is not nav-multi, not a 14 s left hold from t=15 s, and not a kitchen arrival. Those other orders were not re-qualified with this basin.

```bash
python scripts/voice_caller.py "turn left"
python scripts/voice_caller.py --self-test
MUJOCO_GL=osmesa python scripts/voice_caller.py --measure
MUJOCO_GL=osmesa python scripts/voice_caller.py --clip
```
