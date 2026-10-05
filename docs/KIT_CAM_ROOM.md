# kit_cam rooms

These files are vision scenes for the existing `kit_cam`. Each one includes the frozen walk plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (md5 `71b2c86d133ebc603f58b99c53e496f3`) and adds static bodies. Sites with the same names are labels on that geometry. They are not a map, and nothing here is passed to a planner.

The plant's checker plane is still in the included file. Each room adds a non-colliding floor texture plane plus a closed shell (back wall, side walls, ceiling) as extra worldbody geoms. The shell uses the CC0 wall and plaster textures already in the repo. That is not an edit of the plant. `kit_cam` is still the only camera, still on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. Feet stay 145×86. Leg actuators stay ±2.1 Nm. There is no lidar and no second camera. No room adds a joint. The entrance body is a static visual frame (Poly Haven's castle-door mesh, loaded as `entrance_panel`, scaled to fit under the ceiling). There is no hinge, no lever, and no latch.

Furniture is CC0 textured meshes under `mujoco/assets/rooms/`. Licenses, authors, and source URLs are in `docs/ROOM_ASSETS.md`. These stills are a visual domain-gap fix for evaluation. They are not go-anywhere.

| Scene | Bodies | What kit_cam is meant to see |
|-------|--------|------------------------------|
| `mujoco/room_kitchen.xml` | `kitchen`, `table`, `chair` | White slab base run and oak uppers on the back wall, sink and faucet on the sightline, freestanding stove at the image-left end. Island and two stools sit image-right so they do not block the run. Faces +X. `kit_cam` stays at standing height about 0.38 m and looks level, so the countertop surface stays above the lens. Cabinets and the island are CC0 textures on boxes. The stove is not a stacked black double oven. The basin is a CC0 bathroom bowl. Closed white room, wood floor. No flat yellow backsplash |
| `mujoco/room_bathroom.xml` | `bathroom`, `sink`, `toilet`, `bathtub` | Vanity, basin, and mirror against the back-left wall, toilet beside them, stylized tub in the back-right corner. Closed tile room, marble floor |
| `mujoco/room_living.xml` | `living`, `tv`, `coffee` | Larger leather sofa (`living`) against the back wall, cushions toward +X; coffee table in front; CRT beside the sofa at the same depth, screen toward +X. Closed beige room |
| `mujoco/room_bedroom.xml` | `bedroom`, `nightstand` | Closed shell 4.40 m deep by 5.40 m wide. York platform bed (CC BY 4.0) at product scale on the long back wall, long side toward +X, headboard clear of both corners. About 1.48 m of floor in front. Stark nightstand and desk lamp beside the head. Jonathan dresser on the right wall with a clear front |
| `mujoco/room_entrance.xml` | `entrance`, `mat`, `shoes`, `console` | Static panel in thicker wood trim against the back wall. Hessian mat runs up to the leaf. Boots sit image-right of the mat. Console is image-left, clear of the boots. No hinge, no lever, no latch |

Furniture sits in front of `kit_cam` look (+X), so a quiet stand sees the named room body.

```bash
MUJOCO_GL=osmesa python scripts/render_kit_cam_room.py
MUJOCO_GL=osmesa python scripts/render_kit_cam_room.py --room bathroom
MUJOCO_GL=osmesa python scripts/render_kit_cam_room.py --all
```

That stands the robot with the same quiet pose as `scripts/steer_walk.py`, renders `kit_cam`, and writes a still. The default command writes `previews/kit_cam_room.png`. `--room bathroom` (or `living`, `bedroom`, `entrance`) writes `previews/kit_cam_room_<name>.png`. `--all` renders the kitchen and the four other rooms. The script exits non-zero if a frame is still the empty checkerboard, or if that room's named bodies do not fall inside the image. It also checks the plant md5, `kit_cam` pose, the 145×86 feet, and leg torque ±2.1 Nm.

`scripts/steer_walk.py` still loads only the frozen plant by default. Voice can already steer stand / forward / reverse / left / right / left-then-right on that empty checkerboard.

`scripts/find_kitchen.py` is the Prefer FAIL finder for the kitchen scene. "Go to the kitchen" and "go to kitchen" call it only when `room_kitchen.xml` is loaded. The steer cue is the warm white slab from `kit_cam` RGB, not the old flat-yellow backsplash. Arrival still needs both a cue fraction ≥ 0.50 and a torso-to-kitchen gap ≤ 0.25 m. On the merged kitchen the walked stop is `arrival` (remaining 0.230 m, settled slab 0.793, old yellow 0.009, min up_z 0.980). The bars were not lowered. The torso-to-kitchen distance is one arrival bar, not a path. The world-x budget is 1.32 m and did not fire. `vx = 0` yaw is not used.

The living, bedroom, and entrance scenes have no finder. "Go to the living room", "go to the bedroom", and "go to the entrance" stay refused. Prefer FAIL until an AI finder for that room lands. These scenes do not add a goal command, a map, or an arrival claim. The entrance frame stays visual geometry only.

`mujoco/room_hallway_front.xml` is a separate test scene, not a sixth room in `--all` and not a retune of `room_entrance`. It is a narrow hall ending at a static front leaf, for a Moondream2 / find_room entrance still. Prefer FAIL. It is not an arrival and not a go-anywhere claim. See `docs/KIT_CAM_HALLWAY_FRONT.md`.

`scripts/explore_map.py` can load one of these files as vision input. It paints a partial grid from `kit_cam` and does not read the body names. See `docs/EXPLORE_MAP.md`. That is not a finder and not an arrival.
