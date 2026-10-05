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

The bus is `CommandBus` in `scripts/steer_walk.py`: `stand`, `stop`, and `vel(vx, yaw_rate)` only. Resend is 10 Hz. 200 ms of silence stands. The finder sends the forward cap and a yaw inside the yaw cap. The bus clamps again.

From the stand pose the blob is already centered (`|bias| ≤ 0.08`, measured bias −0.014). The phrase then walks in 0.40 s slices and re-scores `kit_cam` between them:

- centered: `vel(+0.056, 0)`
- off center: `vel(+0.056, yaw)` with `yaw = clamp(−bias / 0.35 × 0.25, ±0.25)`

The trim is not a heading setpoint. Full yaw cap is used only when `|bias|` reaches 0.35. `vx = 0` yaw does not change heading on this plant, so the correction is a walking turn. A held full-cap turn of about 2 s pitched through `up_z` 0.90 and the bus then faulted, so the phrase does not do that. It stops on the first sample with `up_z` under 0.90.

There is also a 0.60 m world-x budget. That budget is a stop inside the 0.3–0.8 m band, not a counter pose. This room run did not reach it.

Measured phrase, "go to the kitchen", room scene, after stand:

| | |
|--|--|
| Stop | `up_z` 0.894 (bar 0.90), min up_z 0.890, mode stand |
| End x | **+0.394 m** (start x +0.002 m, Δx **+0.392 m**) |
| End y, yaw | +0.061 m, +0.095 rad |
| Remaining | **0.921 m** from the torso to the kitchen geom (near face x = 1.315 m) |
| Final blob | yellow fraction 0.076, bias +0.102, still in frame, still right of center |
| Arrival | **false** |

Six centered slices and thirteen yaw trims were sent. The mid still is the first frame after Δx crossed 0.30 m (yellow 0.075, bias +0.49). The counter is larger than at the start and still a small part of the frame.

## What would count as arrival

Both bars, on the settled stop frame:

1. Yellow fraction ≥ 0.50. Half the frame is "most of the frame". Stand is 0.039 and this stop is 0.076, so the counter does not fill the frame.
2. Torso-to-kitchen horizontal gap ≤ 0.25 m. That is contact range for this torso, not a room crossing. 0.921 m is more than three times that bar.

Either bar alone is not arrival. A stop inside 0.25 m with the backsplash still small would be Prefer FAIL (`close`), not arrival. The gap is read from the kitchen geom boxes so the summary can state the remaining distance. It does not choose left versus right and it is not a waypoint.

## 1.2 s hop (self-test)

`--self-test` still runs one short hop. The sequence is twelve resends of `vel(+0.056, 0)` from t=0.60 s to t=1.70 s, then stop. Measured end x **+0.036 m**, yaw about −0.1 deg, min up_z 0.981, mode stand. Stopping there is not arrival.

If the blob is off to one side, that self-test uses one `vx = 0` yaw hold (1.0 s, or sooner if the blob reaches the center). Forward comes next only when the new frame is a clear center. A base yaw of −18° (the freejoint, not a camera move) puts the backsplash on the left, bias about −0.35. The self-test sends ten resends of `vel(0, +0.25)` and then stops with no forward.

## Prefer FAIL

No further `vel` is sent when:

- the phrase is bathroom, or any other room
- the loaded scene is the empty walk plant
- the kitchen body is outside the frame (including turned away)
- the backsplash fraction is below 0.015
- the yellow is split across the left and right of the frame
- the centroid does not sit on the kitchen body
- `up_z` drops below 0.90
- the 0.60 m budget is reached, or the slice caps are exhausted
- the stop frame does not meet both arrival bars

```bash
MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to the kitchen"
MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to kitchen"
MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to the bathroom"
MUJOCO_GL=osmesa python scripts/find_kitchen.py --scene plant "go to the kitchen"
python scripts/find_kitchen.py --self-test
```

Stills from the room phrase: `previews/find_kitchen_before.png`, `previews/find_kitchen_mid.png`, `previews/find_kitchen_after.png`, and `previews/find_kitchen_prefer_fail.png` (empty plant). `scripts/steer_walk.py` with no scene argument still loads the empty plant.
