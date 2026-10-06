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

## Five-room go-to on the joined apartment (Prefer FAIL)

Reach was written into `previews/voice_goto_rooms_summary.json` before the bouts. A reach is body COM inside that room's `floor_box` from `mujoco/room_apartment.json`, the same COM having started outside it, min up_z at least 0.90, zero prop contacts, the bout ending by 53.0 s, and a correct per-target yes having committed the go-to. The box is the named room. The plant floor from −3 m to +3 m is not the box. Voice vx stays **+0.056** m/s and yaw **±0.25**. `d_min = 0.150 × (T_detect + T_stop)` with `T_detect = 0` and `T_stop = 0.842` s, so `d_min = 0.1263` m. 0.150 is that hardware bound, not the voice cap. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` is unchanged. `kit_cam` is not moved.

The open-set question ("which room is this?") is not this measure. From the 0.70 m doorway the hall fills most of the frame, so "entrance" is a fair answer and is not pushed off the label set. The question is per target: "Is there a kitchen through the doorway ahead?" (living is asked as "living room"). A yes counts only when that room's furniture covers at least 1% of `kit_cam`. Any other yes is a wrong yes: no forward velocity, and that bout fails.

Scored spawns are the doorway xy turned **±90°** from the door-facing yaw, still outside the room box. A geometric census, taken before the model runs, logs the doorway fraction at 0°, ±60°, ±75°, and ±90°. ±60° and ±75° still see the opening, so they are not the search spawns. ±90° is the heading in that band where the opening fraction is 0. After the 1 s stand the body turns in place at `vel(0, +0.25)`. The sign is not taken from the door bearing. Every 20° the turn stops (`yaw 0`) and the body settles for 0.70 s, which covers the 0.40 rad/s² slew from 0.25 rad/s. The yes/no is asked only after that settle. The logged heading is the heading at capture, not the heading when the answer returns. The first yes does not lock that heading. A correct yes publishes `vel(+0.056, yaw)` with yaw ±0.25 from the doorway pixel and arms the #71 latch. The straight 12.592 s walk with yaw 0 is not a bout here.

While yaw is commanded, both hip yaw joints and both hip roll joints are logged against 2.33 Nm. The bout peak names its joint. A #71 stop on the speckled kitchen or bathroom shadow with no prop in frame is a false stop and is not a #71 pass.

Moondream detects a doorway before the yes/no. No doorway keeps the search. A correct yes does not lock the capture heading. The walk is `vel(+0.056, yaw)` with yaw ±0.25 from the doorway pixel (32 px deadzone around the frame centre), and each later stop-settle-capture re-points. The #71 latch stays armed on that forward motion. A wall or prop contact with the latch armed fails the bout.

The floor/wall row is ranged with the live kit_cam height and the IMU pitch plus `head_tilt`, not a standing table. The logged range is that floor hit in front of the toe that is ahead at capture. The latch compares `toe_gap` (that distance, minus the 20 mm pad and the period high-water) with `d_min + latch_extra`. The 1 cm bar was frozen before the bouts. The pre-bout probe, with no Moondream, measured a worst error of 0.0101 m on a walking bob (`cam_z` 0.338 m, IMU pitch −0.267 rad, camera pitch −0.267 rad), so `latch_extra` is 0.0101 m. The stand sample was 0.18 mm (`cam_z` 0.335 m, IMU pitch −0.259 rad, camera pitch −0.259 rad). A floor-edge stop is a wall-stop pass only when there is no contact and the range error is at most 1 cm, or the pre-measured extra already covers it and the true gap is still at least `d_min`. A wall-stop pass does not make the bout a reach.

The typed caller still refuses a room phrase. The scored run is **Prefer FAIL**. `reached_all` is false. `wrong_yes_any` is true. `false_stop_any` is false. `latch_contact_any` is false. `wall_range_fail_any` is true. `gait_limit_any` is true. `go_anywhere` is false. `kit_safe` is false. All 10 bouts started outside the named room box. None finished inside. Min up_z is 0.934. No #71 latch pass. Every capture has `applied_yaw_at_capture` 0. There were no prop or wall contacts, so the finder cue list at a contact is empty.

Seven yeses fired with the asked room under 1% of `kit_cam`. Those bouts published no forward velocity. Three correct yeses re-pointed while walking. Kitchen −90° stopped on a floor-edge reading of `wall_hall_e_1` (`toe_gap` 0.134 m, latch 0.136 m) with camera pitch −0.253 rad and `cam_z` 0.341 m. The logged range was 0.159 m. The toe's forward ray at that frame hits `table_leg` at 1.113 m, so the error is 0.954 m. Living −90° stopped on `wall_hall_w_2` (`toe_gap` 0.133 m) with camera pitch −0.248 rad and `cam_z` 0.333 m: ranged 0.156 m, true 0.169 m, error 0.0132 m. That is past both the 1 cm bar and the 0.0101 m extra, and the true gap is still above `d_min`. Neither stop is a wall-stop pass. Bedroom −90° walked to the 53 s limit with yaw still ±0.25, no contact, and the COM still outside the box. On those three walks the hip-roll write sat on ±2.33 Nm while the unclamped prediction was about ±4.16 Nm. That is a gait limit. The 1% bar, the 0.70 s settle, the prompt, the latch, and the turn sign stay as they were.

| Room | Offset | First yes t / tick / heading | Door u / frac | Asked frac / visible | Result | Peak joint | Range vs true |
| --- | --- | --- | --- | --- | --- | --- | --- |
| kitchen | +90° | 1.000 / 125 / +1.571 | 198 / 0.041 | 0 / living | wrong yes, no vel | +1.262 `r_ank_pitch` | — |
| kitchen | −90° | 3.800 / 475 / −1.208 | 333 / 0.010 | 0.046 / kitchen | floor-edge, not inside, range fail, gait limit | −2.280 `l_knee` | 0.159 m vs `table_leg` 1.113 m, cam pitch −0.253 |
| bathroom | +90° | 6.600 / 825 / +2.297 | 49 / 0.010 | 0 / living | wrong yes, no vel | +2.280 `r_knee` | — |
| bathroom | −90° | 1.000 / 125 / −1.571 | 255 / 0.012 | 0 / none | wrong yes, no vel | +1.262 `r_ank_pitch` | — |
| living | +90° | 1.000 / 125 / −1.571 | 51 / 0.090 | 0 / none | wrong yes, no vel | +1.262 `r_ank_pitch` | — |
| living | −90° | 3.800 / 475 / +1.933 | 184 / 0.225 | 0.055 / living | floor-edge, not inside, range fail, gait limit | +2.280 `r_knee` | 0.156 m vs `wall_hall_w_2` 0.169 m, cam pitch −0.248 |
| bedroom | +90° | 1.000 / 125 / +1.571 | 255 / 0.005 | 0 / none | wrong yes, no vel | +1.262 `r_ank_pitch` | — |
| bedroom | −90° | 3.800 / 475 / −1.208 | 55 / 0.073 | 0.059 / bedroom | time 53 s, not inside, gait limit | −2.280 `l_knee` | no wall contact |
| entrance | +90° | 1.000 / 125 / −1.571 | 22 / 0.018 | 0 / none | wrong yes, no vel | +1.262 `r_ank_pitch` | — |
| entrance | −90° | 1.000 / 125 / +1.571 | 278 / 0.005 | 0 / kitchen | wrong yes, no vel | +1.262 `r_ank_pitch` | — |

On the three yaw-while-walk bouts the approach hip rolls are unclamped −4.106 / +4.165 Nm (kitchen) and −4.105 / +4.164 Nm (living and bedroom) against a clamped write of −2.33 / +2.33 Nm. Measured hip rolls and the named knee peaks sit at ±2.280 Nm, which is the write rail. Knee commands during the walk are not force-limited in `write_clipped`, so their unclamped and clamped predictions both read about ±6.10 Nm while the measured knee force is ±2.280 Nm. Bathroom +90° turned in place only: unclamped hip rolls 4.789 / 5.749 Nm against a 2.280 Nm write, and `gait_limit` stays false because forward velocity was never published. Speckled patches were sighted and the finder did not emit; no stop landed on a patch. Not kit-safe. Not go-anywhere.

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
