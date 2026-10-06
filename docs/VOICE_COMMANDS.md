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

The room question runs only when the doorway opening is in the frame. Search applies yaw +0.25 until then. A correct yes re-points in place, settles, and walks `vel(+0.056, 0)`. Doorway pixel and size are logged. That re-point is not a free turn: in-place yaw that asks a hip roll over 2.33 Nm unclamped fails the bout. The floor-edge latch fires only when the forward ray hits the same wall the row ranged. A prop, or a different wall, is logged and is not added to the latch. `latch_extra` 0.04087 m is the median of the 17 close `wall_hall_w_2` errors from the previous tip (0.0301 m to 0.0543 m, sim gap under 0.40 m). It is not the peak, and it is not added on any other wall or on a far look at the same name. The settled-stand error on `wall_hall_e_1` is 0.18 mm, so the old 0.0101 m bob pad is not stacked on that median. A stop commits only in the close class (sim gap under 0.40 m): `wall_hall_w_2` uses the median, any other wall uses `d_min` alone. The residual past that wall's own pad has to be within 1 cm or the wall stop fails. A same name seen from farther than 0.40 m is a class reject and stays out of the latch. Room reach is a separate bar. `d_min` stays 0.1263 m, so the living close latch distance is 0.1672 m and every other close wall latches at 0.1263 m. A wall or prop contact with the latch armed fails the bout.

The typed caller still refuses a room phrase. The scored run is **Prefer FAIL**. `reached_all` is false (0 of 10). `wrong_yes_any` is false. `frac_refuse_count` is 10. `false_stop_any` is false. `latch_contact_any` is false. `wall_range_fail_any` is true. `wall_stop_pass_any` is true. `gait_limit_any` is false. `search_yaw_fail_any` is false. `inplace_hip_fail_any` is true. `go_anywhere` is false. `kit_safe` is false. All 10 bouts started outside the named room box. None finished inside. Min up_z is 0.934. No prop or wall contact. Every bout's search applied yaw peaked at +0.250. Every capture still has `applied_yaw_at_capture` 0, because the picture is taken after the settle. Walk yaw on every committed bout is 0.

Ten Moondream yeses had the asked room under 1% of `kit_cam` and were refused: bathroom +90° four, bathroom −90° three, entrance +90° three. Each returned to search and did not commit. Later pictures on those bouts cleared the gate.

Living −90° counted three `tv_stand` rays and logged one (the ranged row was `wall_hall_w_2`, the ray was the stand at 1.358 m error, camera pitch −0.260, `cam_z` 0.332 m, leading toe −0.016 m). That prop was not latched. The bout then stopped on the same wall `wall_hall_w_2`: ranged 0.199 m, ray 0.239 m, error 0.0401 m, residual after the median −0.0008 m, camera pitch −0.267, `cam_z` 0.337 m, leading toe −0.017 m. That wall stop passes the 1 cm residual bar. The body was still outside the living box, so reach fails. Other same-wall samples in that bout have residuals up to 0.242 m, so the bout's wall-range flag stays true. Kitchen −90° also stopped, on `wall_hall_e_1`: ranged 0.199 m, ray 0.414 m, residual 0.174 m. That wall stop fails. Different-wall rays were rejected 330 times and were not soaked into the latch. The largest same-wall residual is 0.790 m (entrance +90°).

In-place yaw asks hip roll unclamped from −4.04 Nm to +4.16 Nm against a clamped write of ±2.33 Nm on every bout. That saturation is a fail. `gait_limit` stays false because yaw was not commanded while walking. Not kit-safe. Not go-anywhere.

| Room | Offset | Refused yes | Door u / size | Wall stop | Residual at stop | Reach |
| --- | --- | --- | --- | --- | --- | --- |
| kitchen | +90° | 0 | 356 / 0.010 | none, 53 s | — | outside |
| kitchen | −90° | 0 | 333 / 0.010 | `wall_hall_e_1` | 0.174 m | outside |
| bathroom | +90° | 4 | 522 / 0.022 | none, 53 s | — | outside |
| bathroom | −90° | 3 | 479 / 0.016 | none, 53 s | — | outside |
| living | +90° | 0 | 210 / 0.184 | none, 53 s | — | outside |
| living | −90° | 0 | 184 / 0.225 | `wall_hall_w_2` | −0.0008 m | outside |
| bedroom | +90° | 0 | 80 / 0.103 | none, 53 s | — | outside |
| bedroom | −90° | 0 | 55 / 0.073 | none, 53 s | — | outside |
| entrance | +90° | 3 | 524 / 0.023 | none, 53 s | — | outside |
| entrance | −90° | 0 | 484 / 0.015 | none, 53 s | — | outside |

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
