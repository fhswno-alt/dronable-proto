# Dronable v0 — Visual prototype (CAD + MuJoCo gait)

Small biped (~415 mm AiNex-class envelope) for founder review.  
**Not a purchase lock.** See `ASSUMPTIONS.md`.

## Layout

```
dronable-proto/
  cad/           STEP + STL + GLB
  mujoco/        MJCF model
  scripts/       CAD builder + gait runner
  previews/      walk MP4, CAD PNG, sim log
  ASSUMPTIONS.md
  README.md
  .venv/         local Python env
```

## Setup

```bash
cd /workspace/dronable-proto
source .venv/bin/activate
# If recreating env:
# uv venv .venv && source .venv/bin/activate
# uv pip install mujoco cadquery trimesh numpy-stl pillow imageio imageio-ffmpeg numpy
# Offscreen GL (box): sudo apt install libosmesa6 libosmesa6-dev
```

## Open CAD

| File | Use |
|------|-----|
| `cad/dronable_v0_body.step` | Preferred — FreeCAD, Fusion, SolidWorks, OnShape |
| `cad/dronable_v0_assembly.step` | Body + door/lever prop @ ~275 mm |
| `cad/dronable_v0_body.stl` | Mesh viewers / slicers |
| `cad/dronable_v0_body.glb` | Quick web/3D view (drag into a GLB viewer) |
| `previews/cad_static.png` | Static screenshot |

Rebuild CAD:

```bash
source .venv/bin/activate
python scripts/build_cad.py
```

## Run MuJoCo sim

Headless record (~8 s MP4, Controls 50 Hz PD gait):

```bash
cd /workspace/dronable-proto
source .venv/bin/activate
MUJOCO_GL=osmesa python scripts/walk_gait.py --duration 8 --out previews/walk_gait.mp4
```

Interactive viewer (needs display):

```bash
MUJOCO_GL=glfw python scripts/walk_gait.py --view --duration 20
```

Stand only (gait off):

```bash
MUJOCO_GL=osmesa python scripts/walk_gait.py --gait-off --duration 5 --out previews/stand.mp4
```

## Day-1 steerable walk (frozen M145, no door)

Velocity only (`stand` / `stop` / `vel`). Voice later calls the same `CommandBus` in `scripts/steer_walk.py`. Plant is `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (md5 `71b2c86d…`) with Hardware `kit_cam` on `head_tilt_link` at `0.050 0.019 0.007` (zero-mass site, fovy 104.82). Foot box, friction, kp, and ±2.1 Nm are unchanged. No OptC / door plant.

```bash
MUJOCO_GL=osmesa python scripts/steer_walk.py
MUJOCO_GL=osmesa python scripts/steer_walk.py --no-video
MUJOCO_GL=glfw  python scripts/steer_walk.py --view
python scripts/steer_walk.py --self-test
```

Headless clips:

```bash
MUJOCO_GL=osmesa python scripts/steer_walk.py --clip forward
MUJOCO_GL=osmesa python scripts/steer_walk.py --clip stop
MUJOCO_GL=osmesa python scripts/steer_walk.py --clip reverse
MUJOCO_GL=osmesa python scripts/steer_walk.py --clip turn
MUJOCO_GL=osmesa python scripts/steer_walk.py --clip turn-right
MUJOCO_GL=osmesa python scripts/steer_walk.py --clip nav-left
MUJOCO_GL=osmesa python scripts/steer_walk.py --clip nav-right
MUJOCO_GL=osmesa python scripts/steer_walk.py --clip nav-multi
```

8 PM pack (same bus, empty plant, claimed left arc): `docs/DEMO_8PM_MOTION.md`.

```bash
MUJOCO_GL=osmesa python scripts/demo_8pm_motion.py
```

That writes `previews/demo_8pm_motion.mp4` and `previews/demo_8pm_motion_summary.json`. Voice uses this `CommandBus` (`stand` | `stop` | `vel(vx, yaw_rate)`), not a kitchen script. Measured on this pack: approach **+0.598 m**, left arc **+75.7 deg**, resume **+0.317 m**, min up_z **0.954**, peak torque **2.10 Nm**, end mode stand, upright, plant md5 unchanged. A separate short reverse (`previews/demo_8pm_reverse.mp4`) is **−0.376 m** with min up_z **0.903** and is not chained after the turn.

`forward` is stand → forward → stop, long enough for several body lengths. `stop` is the same bus on a gait phase that used to pitch after the stop (support margin about −0.08 m) and latch `mode=fault`. `reverse` is stand → retreat → stop. `turn` is stand → walk while yawing left → stop. `turn-right` is the same 11 s window with yaw_rate −0.25. `nav-left` and `nav-right` are the furniture pattern: walk about 0.6 m, arc with `vel(vx, yaw_rate)` held together, walk a bit further along the new heading, then stop. `nav-multi` chains both claimed holds on one continuous walk: forward, left arc (12.5 s), forward, right arc (11 s), forward, then stop. Each writes `previews/steer_walk_<clip>.mp4` and a JSON summary (Δx, forward yaw drift, Δyaw, tip, CoP in box, peak torque, end mode; `nav-multi` also has per-segment Δx, Δyaw, mean body vx, mean yaw rate, and peak torque). Keys, when a display exists: **W/S** ±vx, **A/D** ±yaw (A = left), **Space** stop. Keys latch until Space. AI resends `vel` at 10 Hz; 200 ms of silence stands.

Forward clamp is **+0.056 m/s**, reverse **−0.032 m/s**, yaw **±0.25 rad/s**. The forward clamp was lowered from 0.080 because that command yaws off and tips near 1.2 m. CPG amplitude stays `|applied_vx| / 0.080`, so full forward stick is amplitude 0.70 and reverse stays amplitude 0.40. On the forward clip the body covers **+2.161 m**, stays upright (min up_z 0.967), CoP in the box, peak leg torque 2.10 Nm, and ends in stand (support margin +0.063 m). Mean body vx over that window is **+0.039 m/s**, not the 0.056 command. Net yaw from the first forward sample to the last moving sample is **−0.71 deg**. During the walk yaw rocks between about **−15 deg and +11 deg** and does not run away the way 0.080 does (that reached about +40 deg and tipped near 1.2 m). Heading at the end of the clip is about **−3.1 deg** and holds in stand. Reverse raises the stance-slip damper to 100 N/(m/s) while backing up (clip Δx −0.967 m). After stop that damper stays for 1.20 s (and a reverse cut in a pitching phase finishes at most one step first), then it is off. Turning lengthens the outside step. On the shared 11 s window the left clip is **+61.8 deg** (Δx +0.418 m). The right clip used to stall near **−16 deg** because the full hip-yaw clip toed the swing foot the wrong way; the swing leg now keeps 0.60 of that command and the same window measures **−83.4 deg**, Δx **+0.459 m**, mean body vx **+0.052 m/s**, mean yaw rate **−0.133 rad/s** (cap ±0.25), min up_z 0.955, CoP in the box, peak leg torque 2.10 Nm, end margin +0.063 m, no tip. After the gait has already been walking for 12 s, a left yaw command drops the 0.06 rad bias (that drift has settled) and uses outside-step scale 0.24. Cold-start turns finish before that gate. The nav clips use the same caps: approach **+0.60 m**, then `vel(+0.056, ±0.25)` together, then another short forward, then stop. Left arc **+75.7 deg** in 12.5 s (mean body vx +0.044 m/s during the arc, resume +0.32 m). Right arc **−77.3 deg** in 11 s (mean body vx +0.053 m/s, resume +0.32 m). Both end in stand with margin +0.063 m, CoP in the box, peak leg torque 2.10 Nm, no tip. A left arc started one second earlier and held for 14 s tips on the resume; that window is not claimed. `nav-multi` stays inside those same phase lengths and the same caps, with no plant change and no extra settle. Approach **+0.598 m** (mean body vx +0.038 m/s), left arc **+75.7 deg** in 12.5 s (mean body vx +0.044 m/s, mean yaw rate +0.106 rad/s), straight **+0.317 m** (Δyaw −3.0 deg), right arc **−54.9 deg** in 11 s (mean body vx +0.039 m/s, mean yaw rate −0.087 rad/s), then another straight **+0.227 m**. That last straight still walks, and heading drifts back **+30.7 deg** before the stop. End heading is **+52.9 deg**. Min up_z 0.954, CoP in the box, peak leg torque 2.10 Nm, end mode stand, end margin +0.063 m, no tip. The chained right arc is shorter than the single-arc **−77.3 deg**; both holds are the claimed durations, and vx and yaw are applied together. Prefer FAIL, not claimed: a second 11 s right after a right tips; a second 12.5 s left after a left does not yaw and then tips; right-then-left stays upright but the left hold does not yaw left. Not a door or Gate Q claim.

## Voice commands (same CommandBus)

`scripts/voice_caller.py` turns a typed phrase into `stand`, `stop`, or `vel(vx, yaw_rate)` on the bus above. It resends `vel` at 10 Hz. 200 ms of silence stands. Caps stay **+0.056 / −0.032** m/s and yaw **±0.25** rad/s. Walk forward is `vel(+0.056, 0)`. Turn left / turn right is walk-yaw `vel(+0.056, ±0.25)`. `vx = 0` yaw does not change heading and is refused. The clip speaks the claimed nav-left window: stand, walk forward, turn left, walk forward, stop. Kitchen, bathroom, SLAM, maps, waypoints, and strafe are refused. The finders are separate scripts. This is not go-anywhere. Phrases and the refusal lines are in `docs/VOICE_COMMANDS.md`.

```bash
python scripts/voice_caller.py "turn left"
python scripts/voice_caller.py --self-test
MUJOCO_GL=osmesa python scripts/voice_caller.py --clip
```

The clip is that claimed left arc with the phrase and the bus command on each frame. `kit_cam` is not moved. This run matched the envelope: approach **+0.598 m**, left arc **+75.7 deg**, resume **+0.317 m**, min up_z **0.954**, end mode stand, no tip. Numbers are in `previews/voice_commands_summary.json`.

## kit_cam rooms (vision only, not a map)

Each scene includes the frozen walk plant and adds static CC0 furniture meshes in front of `kit_cam` (+X). `kit_cam` is unchanged: one camera, `head_tilt_link`, `0.050 0.019 0.007`. The plant checker plane is still in the included file. Each room adds a floor texture and a closed shell (walls and a ceiling) as extra worldbody geoms. Body names are labels for a later vision step. They are not a navigation map, and these scenes are not handed to a planner. `scripts/steer_walk.py` still loads only the frozen plant by default, so stand / forward / reverse / left / right / left-then-right stay on the empty checkerboard. Licenses are in `docs/ROOM_ASSETS.md`. See `docs/KIT_CAM_ROOM.md`. This is a visual domain-gap fix for evaluation, not go-anywhere.

| Scene | Named bodies |
|-------|----------------|
| `mujoco/room_kitchen.xml` | `kitchen`, `table`, `chair` |
| `mujoco/room_bathroom.xml` | `bathroom`, `sink`, `toilet`, `bathtub` |
| `mujoco/room_living.xml` | `living` (sofa), `tv`, `coffee` |
| `mujoco/room_bedroom.xml` | `bedroom` (bed), `nightstand` |
| `mujoco/room_entrance.xml` | `entrance` (visual frame only; no hinge, no lever, no latch), `mat`, `shoes`, `console` |

"Go to the kitchen" runs only when `room_kitchen.xml` is loaded. The steer cue is the warm white slab (luminance ≥ 185, saturation ≤ 48, red at least 8 above blue), not the old flat-yellow HSV test. The stand frame is painted into the explore map. If `query_kitchen_like_yellow()` is false, it sends no vel. Until the slab fills a fifth of the frame, each slice sends half-cap `vel(+0.028, yaw)` toward the frontier nearest the logged camera ray. After that, yaw is a tighter trim on the live centroid and vx stays at 0.028. Arrival still needs both cue fraction ≥ 0.50 and torso-to-kitchen ≤ 0.25 m. The `up_z` 0.90 stop is unchanged. The world-x budget is 1.32 m. `vx = 0` yaw is not used. On the merged kitchen the walked stop is `arrival`: end x **+1.270 m** (Δx **+1.268 m**), min up_z **0.980**, remaining **0.230 m**, settled slab **0.793**. Both bars were met. Bathroom slab **0.0007**. The empty plant is **0**. See `docs/FIND_KITCHEN.md`.

`scripts/find_room.py` scores that same frozen kit_cam with local OWL-ViT `google/owlvit-base-patch32` (`cbc355fb`). Objects map to rooms under one cutoff, 0.06. The path vote sums each room's objects over the last 3 poses and commits only when the lead is at least 0.15. Bedroom evidence also includes pillow, nightstand, wardrobe, lamp, and rug. An undecided bedroom pose yaws the stand and logs a re-look. The measured path is a scripted stand at x = 0.0, 0.3, 0.6, 0.9, and 1.2 m. It does not step the gait, it does not publish vel, and it does not change the kitchen slab path. Kitchen stays 5/5 with mean-bearing step 0.307 rad. Bathroom stays 5/5. Living falls to 4/5. Bedroom commits 3/5, and two poses stay undecided after the re-look. The entrance commits bedroom on 4 poses because wardrobe scores up to 0.150. The empty plant peaks at 0.010. This is not an arrival. See `docs/FIND_ROOM.md`.

The living, bedroom, and entrance scenes are vision stills only for the kitchen finder. The entrance body is a visual frame: no hinge, no lever, no latch.

## Explore / map (partial, Prefer FAIL)

`scripts/explore_map.py` is not a room finder. It resends `vel` at 10 Hz on the same `CommandBus` and paints a 0.10 m grid from `kit_cam` only. Floor rays out to 2.60 m become free cells, and one-cell holes in that fan are filled. A saturated floor hit becomes a feature cell. On the previous box entrance that hit was the violet mat. The mesh entrance mat is hessian, and `explore_map.py` was not retuned. The previous yellow backsplash did not meet the floor plane in range, so it was a camera-ray bearing, not a waypoint. The mesh kitchen cue is cabinet grain, stored on the same yellow log. `find_kitchen.py` still uses the 1.80 m paint and the 4-connected 1.60 m frontier ring, and it does not read a soft XY. The empty plant holds the claimed left prefix (stand 1.0 s, `vel(+0.056, 0)` for 15 s, `vel(+0.056, +0.25)` for 12.5 s), then a 4 s forward gap and an 8 s left window when the dense rim is still left, then one `vel(+0.056, −0.25)` for 34 s. A 6 s gap swallows that left window. About 38 s of the right hold crosses `up_z` 0.90. Furnished scenes use `vel(+0.056, −0.25)` for 8 s, because the claimed yaw windows cross `up_z` 0.90 there. `vx = 0` yaw is not sent. Empty plant free cells **883** (open-loop chain 821), frontiers **41** (39), walked cells **57** (52), min up_z **0.939** (0.949), no tip. A separate soft-XY probe is Prefer FAIL and is not this path and not arrival. Those yellow and mat numbers were measured on the previous colored-box rooms. Voice still refuses "explore" and "go anywhere". The rooms stay separate XML files. See `docs/EXPLORE_MAP.md`.

```bash
MUJOCO_GL=osmesa python scripts/explore_map.py --self-test
MUJOCO_GL=osmesa python scripts/explore_map.py --demo
```

```bash
MUJOCO_GL=osmesa python scripts/render_kit_cam_room.py
MUJOCO_GL=osmesa python scripts/render_kit_cam_room.py --all
```

The default command writes `previews/kit_cam_room.png` from `kit_cam` after the same quiet stand as the steer script. `--all` also writes `previews/kit_cam_room_bathroom.png`, `previews/kit_cam_room_living.png`, `previews/kit_cam_room_bedroom.png`, and `previews/kit_cam_room_entrance.png`. Exits non-zero if a still is still the empty checkerboard.

Model: `mujoco/dronable_v0.xml`  
Gait params & teleop stubs: top of `scripts/walk_gait.py` (wired to Controls/AI brief).

## Preview outputs

- `previews/walk_gait.mp4` — stepping gait
- `previews/cad_static.png` — CAD silhouette
- `previews/sim_run_log.txt` — last run proof
- `previews/ego_cam/` + `EGO_CAM_REPORT.md` — kit-cam FOV / neck-pitch approach stills (Mon review)
- `previews/kit_cam_room.png` — `kit_cam` still of the kitchen room (not a navigation map)
- `previews/kit_cam_room_bathroom.png` — bathroom vision still (not a go-to)
- `previews/kit_cam_room_living.png` — living-room vision still (not a go-to)
- `previews/kit_cam_room_bedroom.png` — bedroom vision still (not a go-to)
- `previews/kit_cam_room_entrance.png` — entrance vision still (visual frame only, not a go-to)
- `previews/find_kitchen_before.png` / `find_kitchen_mid.png` / `find_kitchen_after.png` / `find_kitchen_prefer_fail.png` — kit_cam before the bursts, between them, after the stop, and an empty-plant Prefer FAIL
- `previews/explore_map_summary.json` and `previews/explore_map_*_sheet.png` — partial kit_cam map while walking (not go-anywhere)
- `previews/voice_commands_demo.mp4` plus stand / walk / turn / resume / stop stills — voice phrase → bus command on the claimed left arc

## Model summary

~407 mm tall CAD, **21 actuated DOF** (incl. neck pitch) (+ 6-DoF freejoint base), ~**2.66 kg** sim mass, Pi-first (no Orin).  
v0 includes temporary balance assist for visual stability — see ASSUMPTIONS.md.
