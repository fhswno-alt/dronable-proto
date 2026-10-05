# Find kitchen (Prefer FAIL)

`scripts/find_kitchen.py` is a vision-reactive steer on the frozen walk plant. It is not SLAM, not a map, and not an arrival guarantee.

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

## Commands

The bus is `CommandBus` in `scripts/steer_walk.py`: `stand`, `stop`, and `vel(vx, yaw_rate)` only. Resend is 10 Hz. 200 ms of silence stands. The finder sends the bus caps and the bus clamps again:

- forward `vx = +0.056`, `yaw_rate = 0`
- yaw toward a left blob `vx = 0`, `yaw_rate = +0.25`
- yaw toward a right blob `vx = 0`, `yaw_rate = -0.25`

No `vy`, no strafe, no waypoint, no door. Pure yaw still uses the plant's reduced forward step, so a little +X creep during a yaw is the existing gait, not a path.

From the stand pose the blob is already in the center band (`|bias| ≤ 0.08`, measured bias −0.014). The sequence is twelve resends of `vel(+0.056, 0)` from t=0.60 s to t=1.70 s, then stop. Measured end pose: x **+0.036 m**, yaw **−0.1 deg**, min up_z 0.981, mode stand. The counter is about 1.5 m ahead. Stopping there is not arrival.

If the blob is off to one side, one yaw hold (1.0 s, or sooner if the blob reaches the center) is the honest try. Forward comes next only when that new frame is a clear center. Still off to the side, or a blob that crossed to the other side, stops. The finder does not reverse and does not keep turning until a kitchen appears.

A base yaw of −18° (the freejoint, not a camera move) puts the backsplash on the left, bias about −0.35. The self-test sends ten resends of `vel(0, +0.25)` and then stops with no forward, because that one second does not put the blob in the center.

## Prefer FAIL

No `vel` is sent when:

- the phrase is bathroom, or any other room
- the loaded scene is the empty walk plant
- the kitchen body is outside the frame (including turned away)
- the backsplash fraction is below 0.015
- the yellow is split across the left and right of the frame
- the centroid does not sit on the kitchen body
- a short yaw did not put the blob in the center

```bash
MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to the kitchen"
MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to kitchen"
MUJOCO_GL=osmesa python scripts/find_kitchen.py "go to the bathroom"
MUJOCO_GL=osmesa python scripts/find_kitchen.py --scene plant "go to the kitchen"
python scripts/find_kitchen.py --self-test
```

Stills from the room phrase: `previews/find_kitchen_before.png`, `previews/find_kitchen_after.png`, and `previews/find_kitchen_prefer_fail.png` (empty plant). `scripts/steer_walk.py` with no scene argument still loads the empty plant.
