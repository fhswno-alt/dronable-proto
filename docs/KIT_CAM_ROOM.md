# kit_cam room

`mujoco/room_kitchen.xml` is a vision scene for the existing `kit_cam`. It includes the frozen walk plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (md5 `71b2c86d133ebc603f58b99c53e496f3`) and adds three static bodies:

| Body | What it is |
|------|------------|
| `kitchen` | Blue cabinet, wood counter, yellow backsplash, red kettle |
| `table` | Brown table in front and to the camera's right |
| `chair` | Teal chair in front and to the camera's left |

Sites with those same names exist for a later vision step. They are labels on the geometry. They are not a map, and nothing here is passed to a planner.

The floor is still the plant's plane. `kit_cam` is still the only camera, still on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. Feet stay 145×86. Leg actuators stay ±2.1 Nm. There is no door, no lidar, and no second camera.

```bash
MUJOCO_GL=osmesa python scripts/render_kit_cam_room.py
```

That stands the robot with the same quiet pose as `scripts/steer_walk.py`, renders `kit_cam`, and writes `previews/kit_cam_room.png`. It exits non-zero if that frame is still the empty checkerboard, or if `kitchen` / `table` / `chair` do not fall inside the image.

`scripts/steer_walk.py` still loads only the frozen plant by default. Voice can already steer stand / forward / reverse / left / right / left-then-right on that empty checkerboard.

`scripts/find_kitchen.py` is the Prefer FAIL finder for this room. "Go to the kitchen" and "go to kitchen" call it only when `room_kitchen.xml` is loaded. It scores the current `kit_cam` frame (yellow backsplash blob, plus a named-body check that the blob sits on `kitchen` and not on `table` or `chair`). A centered blob gets a 0.40 s slice at half the forward cap (`vx = +0.028`). If the blob drifts, the next slice keeps that vx and adds a yaw trim scaled by the bias. Bathroom, the empty plant, a kitchen that is not in frame, a split blob, or `up_z` under 0.90 send no further `vel`. The torso-to-kitchen distance is reported and is one arrival bar, not a path. The world-x budget is 1.10 m. If yellow is still at least 0.015 but has fallen from its peak, forward-only stops at `|bias|` 0.04 and the next slice is soft walk-yaw so the blob is recentered before it hits zero. If yellow then falls below 0.015 while the kitchen body is still in frame, a soft walk-yaw reacquire (`vel(+0.028, yaw trim)`, at most two tries of 2 s) may run. `vx = 0` yaw does not change heading on this plant, so it is not the reacquire. On main that walk stopped at remaining 0.291 m with settled yellow 0.000. This run stopped at remaining 0.277 m with settled yellow 0.006, still under 0.015, min up_z 0.980. It did not claim arrival. See `docs/FIND_KITCHEN.md`.
