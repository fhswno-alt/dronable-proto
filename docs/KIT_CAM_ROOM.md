# kit_cam rooms

These files are vision scenes for the existing `kit_cam`. Each one includes the frozen walk plant `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (md5 `71b2c86d133ebc603f58b99c53e496f3`) and adds static bodies. Sites with the same names are labels on that geometry. They are not a map, and nothing here is passed to a planner.

The plant's checker plane is still in the included file. Each room adds a non-colliding floor texture plane and a textured back wall as extra worldbody geoms. That is not an edit of the plant. `kit_cam` is still the only camera, still on `head_tilt_link` at `0.050 0.019 0.007`, fovy 104.82. Feet stay 145×86. Leg actuators stay ±2.1 Nm. There is no lidar and no second camera. No room adds a joint. The entrance body is a static visual frame (Poly Haven's castle-door mesh, loaded as `entrance_panel`). There is no hinge, no lever, and no latch.

Furniture is CC0 textured meshes under `mujoco/assets/rooms/`. Licenses, authors, and source URLs are in `docs/ROOM_ASSETS.md`. These stills are a visual domain-gap fix for evaluation. They are not go-anywhere.

| Scene | Bodies | What kit_cam is meant to see |
|-------|--------|------------------------------|
| `mujoco/room_kitchen.xml` | `kitchen`, `table`, `chair` | Wood counter and tall pantry, stove, kettle; dining table; dining chair. Wood floor, beige wall. No flat yellow backsplash |
| `mujoco/room_bathroom.xml` | `bathroom`, `sink`, `toilet` | Painted cabinet and ornate mirror on tile; farmhouse sink; round toilet. Marble floor |
| `mujoco/room_living.xml` | `living`, `tv` | Leather sofa (`living`); CRT television |
| `mujoco/room_bedroom.xml` | `bedroom`, `nightstand` | Gothic bed; nightstand with an arm lamp. Plaster wall |
| `mujoco/room_entrance.xml` | `entrance`, `mat` | Static castle-door panel as a visual frame, and a hessian floor mat |

Furniture sits in front of `kit_cam` look (+X), so a quiet stand sees the named room body.

```bash
MUJOCO_GL=osmesa python scripts/render_kit_cam_room.py
MUJOCO_GL=osmesa python scripts/render_kit_cam_room.py --room bathroom
MUJOCO_GL=osmesa python scripts/render_kit_cam_room.py --all
```

That stands the robot with the same quiet pose as `scripts/steer_walk.py`, renders `kit_cam`, and writes a still. The default command writes `previews/kit_cam_room.png`. `--room bathroom` (or `living`, `bedroom`, `entrance`) writes `previews/kit_cam_room_<name>.png`. `--all` renders the kitchen and the four other rooms. The script exits non-zero if a frame is still the empty checkerboard, or if that room's named bodies do not fall inside the image. It also checks the plant md5, `kit_cam` pose, the 145×86 feet, and leg torque ±2.1 Nm.

`scripts/steer_walk.py` still loads only the frozen plant by default. Voice can already steer stand / forward / reverse / left / right / left-then-right on that empty checkerboard.

`scripts/find_kitchen.py` is the Prefer FAIL finder for the kitchen scene. "Go to the kitchen" and "go to kitchen" call it only when `room_kitchen.xml` is loaded. It still keys on kitchen-like yellow (the old flat backsplash). That cue is gone. Thresholds were not retuned and the bars were not lowered. Until an AI retargets the finder, the mesh kitchen Prefer FAILs: no logged yellow means no vel. The numbers in `docs/FIND_KITCHEN.md` are the previous box-scene run (Prefer FAIL `close` at remaining 0.239 m, settled yellow 0.000, min up_z 0.979). The torso-to-kitchen distance is one arrival bar, not a path. The world-x budget stays 1.10 m. `vx = 0` yaw is not used.

The living, bedroom, and entrance scenes have no finder. "Go to the living room", "go to the bedroom", and "go to the entrance" stay refused. Prefer FAIL until an AI finder for that room lands. These scenes do not add a goal command, a map, or an arrival claim. The entrance frame stays visual geometry only.

`scripts/explore_map.py` can load one of these files as vision input. It paints a partial grid from `kit_cam` and does not read the body names. See `docs/EXPLORE_MAP.md`. That is not a finder and not an arrival.
