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

`scripts/steer_walk.py` still loads only the frozen plant. Voice can already steer stand / forward / reverse / left / right / left-then-right on that empty checkerboard. "Go to the kitchen" is still refused. This room does not add a goal command, a map, or an autonomous go-to-kitchen claim.
