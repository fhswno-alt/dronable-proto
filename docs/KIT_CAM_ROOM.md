# kit_cam rooms

These files are vision scenes for the existing `kit_cam`. Each one includes the frozen walk plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (md5 `71b2c86d133ebc603f58b99c53e496f3`) and adds static bodies. Sites with the same names are labels on that geometry. They are not a map, and nothing here is passed to a planner.

The floor is still the plant's plane. `kit_cam` is still the only camera, still on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. Feet stay 145×86. Leg actuators stay ±2.1 Nm. There is no lidar and no second camera. No room adds a joint. The entrance frame is visual geometry only: jambs, a lintel, side walls, and a mat. There is no hinge, no lever, and no latch.

| Scene | Bodies | What kit_cam is meant to see |
|-------|--------|------------------------------|
| `mujoco/room_kitchen.xml` | `kitchen`, `table`, `chair` | Mesh stove, basin, stools, pendants, and the white cabinet run from `bb94512` |
| `mujoco/room_bathroom.xml` | `bathroom`, `sink`, `toilet`, `bathtub` | Mesh vanity, sink, toilet, and tub |
| `mujoco/room_living.xml` | `living`, `tv`, `coffee` | Mesh sofa, coffee table, and television |
| `mujoco/room_bedroom.xml` | `bedroom`, `nightstand`, `dresser` | Platform-bed mesh, nightstand, lamp, and dresser |
| `mujoco/room_entrance.xml` | `entrance`, `mat`, `shoes`, `console` | Visual frame (no hinge, no lever, no latch), mat, boots, and console |

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

The ask-stand frustum is a separate measurement. `scripts/frustum_stop_ask.py` holds the 1.0 s kit stand and counts `kit_cam` pixels. It does not edit these XML files or the plant. On plant `207f3d5e9c6a72e16f7aa0c8d224f75e` the eye is 0.335 m and the pitch is −14.84 deg. The stop is xy +0.057 m, +0.000 m, yaw +0.01 deg.

The colored-box rooms that this file used to load put the toilet at 29450 px on the right edge (bearing −41.2 deg) and the bed at 114667 px on the look axis, and they had no `kitchen_stove`. Those box files are replaced by the mesh rooms from commit `bb94512`: textured meshes, a floor, walls, and a ceiling, including `kitchen_stove`. Positions, scales, and colors are the `bb94512` values. Room geoms stay `contype="0"` `conaffinity="0"`, the same flags as that commit, so a mesh convex hull is not a collider. The plant plane is still the walking surface.

At this same eye the mesh kitchen stove is visible enough, **10430 px** (3.40%), short side 114, bearing **+34.2 deg**. The mesh toilet is visible enough, **3818 px** (1.24%), short side 59, bearing **+2.7 deg**. The mesh bed (`bedroom_headboard`, the platform-bed mesh) is visible enough, **20901 px** (6.80%), short side 161, bearing **+5.2 deg**. Counts and stills are in `previews/frustum_stop_ask/`. Not a go-to.

The locked-kit 8 s `vel(+0.150, −0.25)` matches the empty plant in every mesh room: min up_z **0.934**, no fault, no tip, end xy **+0.596 m, −0.837 m**, yaw **−94.6 deg**, and no scene contact. In the kitchen that arc's torso passes through `chair_stool_b` (mesh z 0.23–0.39 m). The stool does not collide. It was not moved. The island top's footprint is about 4 cm from a foot in XY and the top itself is at 0.86–0.92 m. `explore_map.py --demo` uses the default CPG, not that locked kit, and it tips on the empty plant as well as on the kitchen and bathroom. Soft-pass is off. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` unchanged.
