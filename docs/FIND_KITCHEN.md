# Find kitchen (Prefer FAIL)

`scripts/find_kitchen.py` is a vision-reactive steer on the frozen walk plant. It is not SLAM, not a map, and not an arrival.

The scene is `mujoco/room_kitchen.xml`, which includes `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (md5 `71b2c86d133ebc603f58b99c53e496f3`). The finder does not edit that file. `kit_cam` stays on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. Feet stay 145×86. Leg actuators stay ±2.1 Nm. No second camera, no lidar, no door.

## What the pixels are

From the stand pose the backsplash is the signal that separates the kitchen from the empty checkerboard. Measured on `kit_cam` at 640×480:

| Frame | Yellow fraction | Blob bias |
|-------|-----------------|-----------|
| Room, stand | ~0.039 | ~−0.014 (center) |
| Room, base yaw −18° | ~0.051 | ~−0.35 (left) |
| Room, base yaw +18° | ~0.046 | ~+0.31 (right) |
| Empty plant, or kitchen behind the camera | 0 | no blob |

The wood counter's lit face does not separate from the blue cabinet, and the kettle renders brown, so those colors are not the blob. The blue cabinet is not the blob either: the empty-floor sky already passes a loose blue test.

A named-body projection of `kitchen`, `table`, and `chair` checks that the yellow centroid sits on the kitchen. That check can only refuse. It is not a waypoint, and it is not passed to a planner. The `vel` sign comes from which side of the image the centroid is on.

## Phrase path

The bus is `CommandBus` in `scripts/steer_walk.py`: `stand`, `stop`, and `vel(vx, yaw_rate)` only. Resend is 10 Hz. 200 ms of silence stands. The bus caps stay +0.056 / −0.032 m/s and yaw ±0.25. The phrase sends half the forward cap. The bus clamps again.

From the stand pose the blob is already centered (`|bias| ≤ 0.08`, measured bias −0.014). The phrase walks in 0.40 s slices, leaves the gait in move, and re-scores `kit_cam` between them:

- centered: `vel(+0.028, 0)`
- off center: `vel(+0.028, yaw)` with `yaw = clamp(−bias / 0.35 × 0.25, ±0.25)`

While the torso-to-kitchen gap is under 0.40 m and yellow is still at least 0.015, the same half-cap command uses shorter slices and a stronger trim. The gait stays in move. There is no stand between slices.

- slice length: 0.20 s
- yaw: `clamp(−bias / 0.20 × 0.25, ±0.25)` so the bus yaw cap is reached at `|bias| = 0.20`

The trim is not a heading setpoint. Farther out, full yaw cap is used only when `|bias|` reaches 0.35. `vx = 0` yaw does not change heading on this plant, so a correction is a walking turn at the soft forward speed.

The `up_z` Prefer FAIL bar stays **0.90**. It is not lowered. A full-cap slice run on the same bar stopped at end x +0.394 m when `up_z` hit 0.894. Shorter slices with a stand between them were tried and did not get past that distance:

| Trial | Result |
|-------|--------|
| 0.25 s vel, 0.40 s stand | Δx −0.052 m at the time limit, min up_z 0.999. The velocity slew takes 0.70 s to reach a cap, so the slice never walks and the stand drifts backward. |
| 0.25 s vel, 0.15 s stand | Δx −0.029 m, min up_z 0.999. Same stall. |
| 0.20 s vel, 0.30 s stand | Kitchen left the frame at Δx +0.082 m (yaw +0.79 rad), min up_z 0.956. |

Half the forward cap, with no stand between slices, is the duty that stayed upright. At a 0.60 m world-x budget that duty stopped on the budget: end x +0.610 m, Δx +0.608 m, min up_z 0.989, remaining 0.705 m, yellow fraction 0.131, arrival false.

The world-x budget on this draft is **1.10 m**. That is still a stop, not a counter pose. The `up_z` bar, the half-cap command, and both arrival bars are unchanged.

Without a reacquire, the same half-cap walk stopped at end x +0.938 m when the yellow fraction fell to 0.004 with the kitchen body still in frame (remaining 0.377 m, min up_z 0.981). `vx = 0` yaw does not change heading on this plant, so a yaw-only reacquire is not used. A dim yellow with the kitchen still in frame keeps half-cap `vel(+0.028, yaw trim)` toward the last bias. Each try is 2.0 s. The normal approach resumes only if yellow returns to at least 0.015 and the score is usable (`forward`, `yaw_left`, or `yaw_right`). While the gap is 0.40 m or more, two tries are allowed. Inside 0.40 m, four tries are allowed. A blob that stays lost, a kitchen that leaves the frame, `up_z` under 0.90, or the end of that budget is Prefer FAIL and sends no further `vel`.

On main, two walk-yaw tries resumed once and then lost the blob again: end x +1.024 m, Δx +1.022 m, remaining 0.291 m, min up_z 0.980, settled yellow 0.000, arrival false. This draft tries to keep yellow usable across that last part of the gap. The 0.25 m bar is not lowered. A stop inside 0.25 m with yellow still under 0.50 is Prefer FAIL `close`, not arrival.

Baseline on main, before the close-range protect, "go to the kitchen", room scene, after stand:

| | |
|--|--|
| Stop | Prefer FAIL, `stop_kind` blob. Two soft walk-yaw tries. Yellow came back above 0.015 and the normal half-cap loop resumed, then the blob was lost again. The two-try budget blocked a third try. No further `vel`. |
| End x | **+1.024 m** (start x +0.002 m, Δx **+1.022 m**) |
| End y, yaw | +0.052 m, +0.229 rad |
| min up_z | **0.980** (bar 0.90, not crossed) |
| Remaining | **0.291 m** from the torso to the kitchen geom (near face x = 1.315 m) |
| Final blob | yellow fraction **0.000** on the settled frame. Kitchen body still in frame. |
| Reacquires | 2. Every phrase command was `vx = +0.028` (yaw trim up to +0.250). `vx = 0` was not sent. |
| Arrival | **false** |

111 centered slices and 50 walking yaw trims were sent at `vx = +0.028`, including the slices after yellow briefly returned. The mid still is the first frame after Δx crossed 0.55 m (yellow 0.105, bias −0.078). 0.000 of the frame is not half, and 0.291 m is still outside the 0.25 m gap. Soft walk-yaw did not keep the backsplash.

## What would count as arrival

Both bars, on the settled stop frame:

1. Yellow fraction ≥ 0.50. Half the frame is "most of the frame". Stand is 0.039 and this stop is 0.000, so the counter does not fill the frame.
2. Torso-to-kitchen horizontal gap ≤ 0.25 m. That is contact range for this torso, not a room crossing. 0.291 m is still outside that bar.

Either bar alone is not arrival. A stop inside 0.25 m with the backsplash still small would be Prefer FAIL (`close`), not arrival. The gap is read from the kitchen geom boxes so the summary can state the remaining distance. It does not choose left versus right and it is not a waypoint.

## 1.2 s hop (self-test)

`--self-test` still runs one short hop. The sequence is twelve resends of `vel(+0.056, 0)` from t=0.60 s to t=1.70 s, then stop. Measured end x **+0.036 m**, yaw about −0.1 deg, min up_z 0.981, mode stand. Stopping there is not arrival.

If the blob is off to one side, that self-test uses one `vx = 0` yaw hold (1.0 s, or sooner if the blob reaches the center). Forward comes next only when the new frame is a clear center. A base yaw of −18° (the freejoint, not a camera move) puts the backsplash on the left, bias about −0.35. The self-test sends ten resends of `vel(0, +0.25)` and then stops with no forward.

## Prefer FAIL

No further `vel` is sent when:

- the phrase is bathroom, or any other room
- the loaded scene is the empty walk plant
- the kitchen body is outside the frame (including turned away)
- the backsplash fraction is below 0.015 and the soft walk-yaw reacquire does not keep it usable (two tries of `vel(+0.028, yaw trim)` while the gap is 0.40 m or more, four tries inside that gap, then stop)
- the yellow is split across the left and right of the frame
- the centroid does not sit on the kitchen body
- `up_z` drops below 0.90
- the 1.10 m budget is reached, or the slice caps are exhausted
- the stop frame does not meet both arrival bars

```bash
MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to the kitchen"
MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to kitchen"
MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to the bathroom"
MUJOCO_GL=osmesa python scripts/find_kitchen.py --scene plant "go to the kitchen"
python scripts/find_kitchen.py --self-test
```

Stills from the room phrase: `previews/find_kitchen_before.png`, `previews/find_kitchen_mid.png`, `previews/find_kitchen_after.png`, and `previews/find_kitchen_prefer_fail.png` (empty plant). `scripts/steer_walk.py` with no scene argument still loads the empty plant.
