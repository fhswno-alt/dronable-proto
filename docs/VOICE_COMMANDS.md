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

## Five-room go-to (Prefer FAIL)

Reach was written into `previews/voice_goto_rooms_summary.json` before the five stands. A reach is body COM inside that room's floor box, the same COM having started outside it, min up_z at least 0.90, zero prop contacts, and the bout ending by 53.0 s. The floor box is the axis-aligned XY of every loaded plane named `floor` or `room_floor`. Voice vx stays **+0.056** m/s and yaw **±0.25**. `d_min = 0.150 × (T_detect + T_stop)` with `T_detect = 0` and `T_stop = 0.842` s, so `d_min = 0.1263` m. 0.150 is that hardware bound, not the voice cap. Soft-pass is off. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged.

The tree has five room files and no joined scene. Each file's only floor plane is the plant floor, x and y from −3 m to +3 m. The documented 0.60 s stand settles at COM about **−0.003, −0.000** m, inside that box. No doorway spawn exists. A start inside the box is not a reach, so no `vel` was published and the #71 latch was not applied.

| Room | Voice parse | Spawn COM | Started outside | Commanded vel | d_min | Reached |
| --- | --- | --- | --- | --- | --- | --- |
| kitchen | refuse | −0.003, −0.000 | false | none | 0.1263 m | false |
| bathroom | refuse | −0.003, −0.000 | false | none | 0.1263 m | false |
| living | refuse | −0.003, −0.000 | false | none | 0.1263 m | false |
| bedroom | refuse | −0.003, −0.000 | false | none | 0.1263 m | false |
| entrance | refuse | −0.003, −0.000 | false | none | 0.1263 m | false |

Stand contact is none. Stand peak is +0.362 Nm on `l_ank_roll_pos`. min up_z is 1.000. The recogniser was not asked. Not kit-safe. Not go-anywhere.

Next gap: a joined multi-room scene, or a doorway spawn that starts outside the named room's floor box. None is in this tree, and a new scene was not authored.

```bash
MUJOCO_GL=osmesa python scripts/voice_goto_rooms.py
```

Exit status is 1. The summary is `previews/voice_goto_rooms_summary.json`.

## Clip

`MUJOCO_GL=osmesa python scripts/voice_caller.py --clip` plays the claimed nav-left window as phrases: **stand → walk forward → turn left → walk forward → stop**. Times are 1 s, 15 s, 12.5 s, 6 s, then a 2.5 s stop. The bus commands are `stand`, `vel(+0.056, +0.000)`, `vel(+0.056, +0.250)`, `vel(+0.056, +0.000)`, `stop`. Each frame captions the phrase and that bus command. The view is the third-person camera so the body is visible. `kit_cam` stays at `0.050 0.019 0.007` and is not the demo camera.

The voice clip on main matched the previous gait basin: approach **+0.598 m**, left arc **+75.7 deg**, resume **+0.317 m**, min up_z **0.954**. This step-cycle basin uses the same phrases and the same bus, and it does not match that envelope. The remeasured walk is in `docs/DEMO_8PM_MOTION.md` (approach **+0.695 m**, left arc **+61.1 deg**, resume **+0.340 m**, min up_z **0.940**). `vx = 0` yaw still does not change heading.

Locked-kit nav-left resume heading (Prefer FAIL). The phrases above still publish `vel(+0.056, …)`. On the locked kit CommandBus the same window shape is `vel(+0.150, …)`. Scenario: 1 s stand, 15 s forward, 12.5 s `vel(+0.150, +0.25)`, then 6 s `vel(+0.150, 0)`. Resume Δyaw ≈ +12.41°. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` unchanged. Split: (1) 0–0.68 s `applied_yaw` still slewing +0.25→0 at 0.40 rad/s² after the 100 ms resend clears the target → body +6.81° (command integral ~+5.22°); (2) 0.68–6.0 s `applied_yaw` and the step angle are already 0 → leftover left curve +5.60° (body yaw rate +0.034→+0.009 rad/s). That is ~half ramp-out, ~half steady leftover curve under `vel(+0.150, 0)` — not “turn still commanded.” Soft-pass is off.

Left versus right unload, same plant. The 6 s `vel(+0.150, 0)` after the left turn is +12.3°. The 6 s `vel(+0.150, 0)` after an 11 s `vel(+0.150, −0.25)` is −2.2° on the chained walk and −0.6° from a straight approach. The slew is 0.40 rad/s² both ways. On a matching step phase the right ramp moves the body −5.3° (command integral −4.5°) and the rest of the window then curves left +4.7°, so they cancel. The left side adds: +6.7° of ramp, then +5.6° with the yaw command and the step angle already 0. Straight `vel(+0.150, 0)` with no turn already curves +4.2° over 27–33 s and +3.5° over 28.5–34.5 s. Hip-yaw targets are 0 once the step angle is 0. Stop snaps yaw to 0 in one tick, and after a right turn that heading kick runs about +11° to −10° across 0.37 s of step phase. Five cold straight walks on the kit row net +0.50° over 30 s and the cold start still swings the left foot first. After a turn, the straight resume swings the outside foot and the pose chases the live gait over that double support instead of stepping 0.342 rad in one tick. The post-left 6 s is +1.4°, and the post-right 6 s stays −2.0°. The left turn finishes at +156.4° in 12.5 s; the right turn finishes at −148.6° in 11.0 s (7.8° shorter because the hold is shorter). Hip roll while yawing stays at or under 2.33 Nm. First right-lead step knees after stand are 1.749 / 1.152 Nm, under 2.33.

This is not nav-multi, not a 14 s left hold from t=15 s, and not a kitchen arrival. Those other orders were not re-qualified with this basin.

```bash
python scripts/voice_caller.py "turn left"
python scripts/voice_caller.py --self-test
MUJOCO_GL=osmesa python scripts/voice_caller.py --measure
MUJOCO_GL=osmesa python scripts/voice_caller.py --clip
```
