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

The colored-box rooms that this file used to load put the toilet at 29450 px on the right edge (bearing −41.2 deg) and the bed at 114667 px on the look axis, and they had no `kitchen_stove`. Those box files are replaced by the mesh rooms from commit `bb94512`: textured meshes, a floor, walls, and a ceiling, including `kitchen_stove`. Positions, scales, and colors are the `bb94512` values. Visual room geoms stay `contype="0"` `conaffinity="0"`, so a mesh convex hull is not a collider. Each prop also has a hidden group-3 box proxy (`contype="1"`, friction `1.0 0.1 0.01`) at the mesh footprint. Open stools, the coffee table, the console, the nightstand, the vanity, and the sofa use legs or rails plus a top, and the space under the top stays open. The decorative floor plane and the ceiling stay non-colliding. The plant plane is still the walking surface, and its friction stays 1.6. Group 3 is off in the default renderer, so `kit_cam` does not see the proxies.

At this same eye the mesh kitchen stove is visible enough, **10430 px** (3.40%), short side 114, bearing **+34.2 deg**. The mesh toilet is visible enough, **3818 px** (1.24%), short side 59, bearing **+2.7 deg**. The mesh bed (`bedroom_headboard`, the platform-bed mesh) is visible enough, **20901 px** (6.80%), short side 161, bearing **+5.2 deg**. Counts and stills are in `previews/frustum_stop_ask/`. Not a go-to.

The locked-kit 8 s `vel(+0.150, −0.25)` is measured after the 1.0 s stand. Times below are seconds from spawn; the vel window is t = 1.0 s to 9.0 s. The plant md5 is the include MuJoCo resolved at load, not a separate hash of a path that was never opened. On `207f3d5e9c6a72e16f7aa0c8d224f75e` every leg actuator forcerange in that loaded model is ±2.45 Nm. The shared bar is 2.33 Nm.

Empty plant, bathroom, bedroom, living, and entrance match: runtime md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`, min up_z **0.934**, no fault, no tip, end xy **+0.596 m, −0.837 m**, yaw **−94.6 deg**, no robot-prop contact. Left knee peaks at **2.06 Nm**, right knee at **1.96 Nm**. The left-side peak is `l_hip_roll_pos` at **2.36 Nm** (t = 3.42 s), over 2.33 Nm, with no prop contact. The right-side peak is `r_hip_roll_pos` at **2.21 Nm**.

The kitchen arc hits `chair_stool_b` and tips. Runtime md5 is the same `207f3d5e9c6a72e16f7aa0c8d224f75e`. Min up_z **−0.732**, tip, fault `COM outside support and tipping margin=-0.889` at t = 7.95 s, end xy **+0.709 m, −0.675 m**, yaw **+128.1 deg**. Four contact episodes (141 pair samples on 135 ticks): `l_ank_roll_link` on `col_chair_stool_b_leg_2` peaks **23.9 N** at t = 6.85 s, **21.3 N** at t = 7.32 s, and **17.8 N** at t = 7.83 s; `r_ank_roll_link` on `col_chair_stool_b_rail_yp` peaks **31.6 N** at t = 8.84 s. Left knee peaks at **2.44 Nm** (t = 7.18 s), over 2.33 Nm, on a tick with no prop contact. `l_hip_pitch_pos`, `r_hip_pitch_pos`, and `r_ank_pitch_pos` reach the ±2.45 Nm forcerange during a prop contact. `l_hip_roll_pos` still peaks at **2.36 Nm** before the hit, the same empty-plant sample. The stool was not moved. The island top stays a separate slab from the pedestal.

`scripts/stool_leg_frustum.py` replays that same walk and counts `chair_stool_b` in `kit_cam`. It does not move the stool. Group 3 stays hidden, so the counts are the visual mesh. A mesh pixel is in the leg/rail band when its reconstructed world z is below the seat box (z < 0.2562), the split authored on `col_chair_stool_b`. The size bar is the earlier frustum bar: under 3072 px, or a short side under 36 px, is `tiny`. A mesh region that clears both is `visible_enough`.

The replay lands on the same contacts. First contact is t = 6.85 s, `l_ank_roll_link` on `col_chair_stool_b_leg_2`, peak **23.9 N**. The later peaks are **21.3 N** at t = 7.32 s, **17.8 N** at t = 7.83 s, and **31.6 N** at t = 8.84 s on `r_ank_roll_link` / `col_chair_stool_b_rail_yp`. Min up_z is **−0.732**. The fault string is unchanged.

At the end of the stand (t = 1.00 s, eye 0.335 m, pitch −14.84 deg) the stool mesh is `visible_enough`, **6370 px** (2.07%), short side 154, bearing **−53.9 deg**, bbox x 486–639 (clipped on the right). The seat band is `visible_enough`, **4358 px** (1.42%), short side 104. The leg/rail band is in frame and `tiny`, **2012 px** (0.65%), short side **96**. The short side clears 36 px; the pixel count is what stays under 3072. `col_chair_stool_b_leg_2` and `col_chair_stool_b_rail_yp` both project into the image. Pixel world z runs 0.006–0.477 m, with a leg-band mean of 0.123 m and a seat-band mean of 0.425 m.

The leg band first clears the bar at t = **1.90 s**: **3849 px** (1.25%), short side 141, bearing **−47.3 deg**. That is **4.94 s** before the first contact. The whole stool is `visible_enough` from t = 1.00 s, **5.85 s** before the hit. The largest pre-contact leg count is t = 5.90 s, **17836 px** (5.81%), short side 258, bearing **−18.7 deg**. The sample nearest the hit (t = 6.80 s) still has the leg band at **5970 px**, short side 257, bearing **−12.0 deg**. Every pre-contact sample has the stool mesh `visible_enough`, and both hit boxes project. `kit_cam` can see the stool legs before the foot hits them. This probe does not command a stop. Soft-pass is off. Stills and the table are in `previews/stool_leg_frustum/`.

`scripts/stool_leg_stop.py` writes a Day1 `stop` when that leg/rail band first reaches `visible_enough`. `t_cue` is that sim time, **1.90 s** (3849 px, short side 141, bearing −47.3 deg). `T_detect` is **28.8 ms** of wall-clock on that frame only: the clock starts when the kit_cam RGB buffer is in hand and ends when `CommandBus.stop` returns. The read inside the span is the same visual-mesh leg/rail check (segmentation and depth). Group-3 boxes are not the mask. No Moondream. The sim clock does not advance during `T_detect`, so `t_stop` is **1.90 s** and `t_stop − t_cue` is 0. The **4.94 s** from `t_cue` to the old 6.85 s contact is not `T_detect`. `T_stop`, the gait stop distance and time, is not measured here.

After that write, `vel` is not sent again. The stand holds through t = 9.0 s on plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`. Prop contacts **0**. Min up_z **0.934**. No fault. End mode stand, xy **+0.086 m, −0.007 m**, yaw **−5.6 deg**. Left knee peaks at **2.060 Nm** at t = 1.54 s. Right knee peaks at **1.960 Nm** at t = 1.29 s. No leg actuator crosses 2.33 Nm. This walk stays clear of the stool. It is not a go-to. Soft-pass is off. The cue frame is `previews/stool_leg_stop/kitchen_cue_stop.png`.

`explore_map.py --demo` uses the default CPG, not that locked kit. Those demo numbers are from before these proxies: it tips on the empty plant as well as on the kitchen and bathroom. Soft-pass is off. Plant file md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` unchanged.
