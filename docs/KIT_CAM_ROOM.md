# kit_cam rooms

These files are vision scenes for the existing `kit_cam`. Each one includes the frozen walk plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (md5 `71b2c86d133ebc603f58b99c53e496f3`) and adds static bodies. Sites with the same names are labels on that geometry. They are not a map, and nothing here is passed to a planner.

The floor is still the plant's plane. `kit_cam` is still the only camera, still on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. Feet stay 145×86. Leg actuators stay ±2.1 Nm. There is no lidar and no second camera. No room adds a joint. The entrance frame is visual geometry only: jambs, a lintel, side walls, and a mat. There is no hinge, no lever, and no latch.

| Scene | Bodies | What kit_cam is meant to see |
|-------|--------|------------------------------|
| `mujoco/room_kitchen.xml` | `kitchen`, `table`, `chair` | Blue cabinet, wood counter, yellow backsplash, red kettle; brown table; teal chair |
| `mujoco/room_bathroom.xml` | `bathroom`, `sink`, `toilet` | Cyan tile and a white vanity (not the kitchen yellow); white pedestal sink with a violet faucet; white toilet with a magenta seat |
| `mujoco/room_living.xml` | `living`, `tv` | Rust sofa (`living`); charcoal television with a pale blue screen |
| `mujoco/room_bedroom.xml` | `bedroom`, `nightstand` | The bed: purple headboard, indigo duvet band, pink pillows; gray nightstand with a green lamp |
| `mujoco/room_entrance.xml` | `entrance`, `mat` | Visual doorframe (dark jambs, white trim, teal side walls) and a violet floor mat |

Furniture sits in front of `kit_cam` look (+X), the same idea as the kitchen counter, so a quiet stand sees the named room body.

```bash
MUJOCO_GL=osmesa python scripts/render_kit_cam_room.py
MUJOCO_GL=osmesa python scripts/render_kit_cam_room.py --room bathroom
MUJOCO_GL=osmesa python scripts/render_kit_cam_room.py --all
```

That stands the robot with the same quiet pose as `scripts/steer_walk.py`, renders `kit_cam`, and writes a still. The default command writes `previews/kit_cam_room.png`. `--room bathroom` (or `living`, `bedroom`, `entrance`) writes `previews/kit_cam_room_<name>.png`. `--all` renders the kitchen and the four other rooms. The script exits non-zero if a frame is still the empty checkerboard, or if that room's named bodies do not fall inside the image. It also checks the plant md5, `kit_cam` pose, the 145×86 feet, and leg torque ±2.1 Nm.

`scripts/steer_walk.py` still loads only the frozen plant by default. Voice can already steer stand / forward / reverse / left / right / left-then-right on that empty checkerboard.

`scripts/find_kitchen.py` is the Prefer FAIL finder for the kitchen scene. "Go to the kitchen" and "go to kitchen" call it only when `room_kitchen.xml` is loaded. The stand frame is painted into the explore map. No logged yellow means no vel. Logged yellow means half-cap `vel(+0.028, yaw)` from `query_kitchen_like_yellow()` and `frontier_cells()`, not from the live blob alone. The torso-to-kitchen distance is one arrival bar, not a path. The world-x budget is 1.10 m. `vx = 0` yaw is not used. This run stopped Prefer FAIL `close` at remaining 0.239 m with settled yellow 0.000, min up_z 0.979. The gap bar alone is not arrival. See `docs/FIND_KITCHEN.md`.

The living, bedroom, and entrance scenes have no finder. "Go to the living room", "go to the bedroom", and "go to the entrance" stay refused. Prefer FAIL until an AI finder for that room lands. These scenes do not add a goal command, a map, or an arrival claim. The entrance frame stays visual geometry only.

`scripts/explore_map.py` can load one of these files as vision input. It paints a partial grid from `kit_cam` and does not read the body names. See `docs/EXPLORE_MAP.md`. That is not a finder and not an arrival.

The ask-stand frustum is a separate measurement. `scripts/frustum_stop_ask.py` holds the 1.0 s kit stand and counts `kit_cam` pixels. It does not edit these XML files or the plant. On plant `207f3d5e9c6a72e16f7aa0c8d224f75e` the eye is 0.335 m and the pitch is −14.84 deg. The stop is xy +0.057 m, +0.000 m, yaw +0.01 deg. The current toilet covers 29450 px as a box on the right edge (bearing −41.2 deg). The current bed covers 114667 px as a box on the look axis. `kitchen_stove` is not in this kitchen file. The kettle, the towel, and the mattress have 0 pixels (occluded). The mesh rooms at commit `bb94512`, rendered at this same eye, do contain a stove (10430 px), a toilet (3818 px), and a bed (20901 px). Counts and stills are in `previews/frustum_stop_ask/`. Not a go-to.
