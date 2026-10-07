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

The open-set question ("which room is this?") is not this measure. From the 0.70 m doorway the hall fills most of the frame, so "entrance" is a fair answer and is not pushed off the label set. The question is per target: "Is there a kitchen through the doorway ahead?" (living is asked as "living room"). Moondream yes is logged against the 1% `kit_cam` gate. A yes under that fraction is refused and the search continues. It does not publish forward velocity. A yes under that fraction that still commits forward velocity is a wrong yes, and that bout fails.

Scored spawns are the doorway xy turned **±90°** from the door-facing yaw, still outside the room box. A geometric census, taken before the model runs, logs the doorway fraction at 0°, ±60°, ±75°, and ±90°. ±60° and ±75° still see the opening, so they are not the search spawns. ±90° is the heading in that band where the opening fraction is 0. After the 1 s stand the body turns in place at `vel(0, +0.25)`. The sign is not taken from the door bearing. Every 20° the turn stops (`yaw 0`) and the body settles for 0.70 s, which covers the 0.40 rad/s² slew from 0.25 rad/s. The yes/no is asked only after that settle. The logged heading is the heading at capture, not the heading when the answer returns. The room question is asked only when that opening is actually in the frame. Otherwise the search applies yaw +0.25 in place. The picture is taken after that yaw has settled to 0. A correct yes yaws in place toward the doorway pixel, settles, then walks `vel(+0.056, 0)`. Yaw is not commanded while walking. The straight 12.592 s walk with yaw 0 from a facing spawn is not a bout here.

While yaw is commanded, both hip yaw joints and both hip roll joints are logged against 2.33 Nm. The bout peak names its joint. A #71 stop on the speckled kitchen or bathroom shadow with no prop in frame is a false stop and is not a #71 pass.

The room question runs only when the doorway opening is in the frame. Search applies yaw +0.25 until then. A correct yes re-points in place, settles, and walks `vel(+0.056, 0)`. Doorway pixel and size are logged. That re-point is not a free turn: in-place yaw that asks a hip roll over 2.33 Nm unclamped fails the bout. The floor-edge latch fires only when the forward ray hits the same wall the row ranged. A prop, or a different wall, is logged and is not added to the latch. `latch_extra` 0.04087 m is the recorded median of the 17 close `wall_hall_w_2` errors (sim gap under 0.40 m). The live right-toe pad on that wall is the median of 49 close right samples, 0.04937 m, with both committing stops left out. The left-toe pad on `wall_hall_w_2` is the median of 19 close left samples, 0.01063 m. An east pad is not applied to that wall. `wall_hall_e_1` left samples median 0.04542 m (N=45). Median plus MAD is 0.06507 m and is not the live latch. A same-bout median of every close left error, pad 0.05613 m, stopped at residual −0.0284 m. The 75th percentile, 0.05944 m, stopped at residual −0.0318 m. Neither is the latch. The live left latch is the same-bout median of left rays that are as close as any earlier left ray on `wall_hall_e_1`. The sample that crosses the gate is left out. Right stays 0.07279 m (9 samples) and does not stop the walk. An unknown toe gets pad 0. Contact-box centres sit 14 mm outboard of each ankle-roll axis, so the ray origin flips side with the lead foot. A stop clears only when the absolute gap between the measured error and that wall×toe pad is at most 1 cm. A large negative residual does not clear. Once the left-toe ray on `wall_hall_e_1` is at or under 0.60 m, further in-place re-points are dropped and the walk stays at yaw 0. No pitch, camera-height, or toe bias is fit across pooled samples. The 0.40 m bound is 2.39 times the old living right-toe latch distance 0.1672 m (`d_min` 0.1263 m plus 0.04087 m), so it is the walk-up in front of the latch. A ray past 0.40 m stays a far reject. `wall_hall_e_0` had five close samples, under the living set of 17, so its pad stays 0. The settled-stand error on `wall_hall_e_1` is 0.18 mm, so the old 0.0101 m bob pad is not stacked. Room reach is a separate bar. `d_min` stays 0.1263 m. Living right latches at 0.1757 m. Living left latches at 0.1369 m. East right latches at 0.1991 m and cannot CLEAR on nine samples. Every other close wall latches at 0.1263 m. A wall or prop contact with the latch armed fails the bout.

The typed caller still refuses a room phrase. The scored run is **Prefer FAIL**. Reach CLEAR is 0 of 10. Wall-stop CLEAR is 1 of 10. `wrong_yes_any` is false. `frac_refuse_count` is 10. `false_stop_any` is false. `latch_contact_any` is false. `wall_stop_fail_any` is true. `gait_limit_any` is false. `search_yaw_fail_any` is false. `inplace_hip_fail_any` is true. `class_reject_count` is 691. `surface_reject_count` is 374. `go_anywhere` is false. `kit_safe` is false. Soft-pass is off. All 10 bouts started outside the named room box. None finished inside. Min up_z is 0.934. No prop or wall contact. Every bout's search applied yaw peaked at +0.250. Every capture still has `applied_yaw_at_capture` 0, because the picture is taken after the settle. Walk yaw on every committed bout is 0. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`.

Ten Moondream yeses had the asked room under 1% of `kit_cam` and were refused: bathroom +90° four, bathroom −90° three, entrance +90° three. Each returned to search and did not commit. Later pictures on those bouts cleared the gate. Wrong-yes count stays 0.

Living −90° stops on `wall_hall_w_2` with the right toe leading. The pad is 0.04937 m (N=49). Ray 0.239 m, toe gap 0.165 m, gate 0.176 m, error 0.04006 m, residual −0.0093 m. Heading +2.672 rad, `cam_z` 0.337 m, camera pitch −0.267, lead −0.0165 m. The absolute miss is 0.93 cm, so it CLEARs. The left-toe pad 0.01063 m (N=19) was not used. The body is still outside the living box, so reach stays open.

Kitchen −90° holds the walk at yaw 0 after the left toe's ray on `wall_hall_e_1` falls under 0.60 m. The live pad is the same-bout median of 6 left rays that set a new closest ray, 0.02857 m, gate 0.155 m. The stop is left toe, ray 0.239 m, toe gap 0.154 m, error 0.06308 m, residual +0.0345 m. Heading −0.477 rad, `cam_z` 0.333 m, camera pitch −0.249, lead −0.0039 m. The absolute miss is 3.5 cm, so it does not CLEAR. The approach-wide median 0.05613 m and the 75th percentile 0.05944 m stay off. The right pad 0.07279 m (N=9) is not used. The body is still outside the kitchen box, so reach stays open.

In-place yaw asks hip roll unclamped from about −4.04 Nm to +4.16 Nm against a clamped write of ±2.33 Nm on every bout. Bedroom +90° left roll is −4.01 Nm. That saturation is a fail. `gait_limit` stays false because yaw was not commanded while walking. Not kit-safe. Not go-anywhere.

| Room | Offset | Refused yes | Door u / size | Wall stop | Residual at stop | Leading toe | Reach |
| --- | --- | --- | --- | --- | --- | --- | --- |
| kitchen | +90° | 0 | 355 / 0.198 | none, 53 s | — | — | outside |
| kitchen | −90° | 0 | 424 / 0.536 | `wall_hall_e_1` | +0.0345 m | L | outside |
| bathroom | +90° | 4 | 394 / 0.024 | none, 53 s | — | — | outside |
| bathroom | −90° | 3 | 484 / 0.005 | none, 53 s | — | — | outside |
| living | +90° | 0 | 223 / 0.016 | none, 53 s | — | — | outside |
| living | −90° | 0 | 157 / 0.028 | `wall_hall_w_2` | −0.0093 m | R | outside |
| bedroom | +90° | 0 | 320 / 1.000 | none, 53 s | — | — | outside |
| bedroom | −90° | 0 | 143 / 0.331 | none, 53 s | — | — | outside |
| entrance | +90° | 3 | 243 / 0.036 | none, 53 s | — | — | outside |
| entrance | −90° | 0 | 294 / 0.044 | none, 53 s | — | — | outside |

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
