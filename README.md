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

That writes `previews/demo_8pm_motion.mp4`, a side close-up `previews/demo_step_cycle.mp4`, and `previews/demo_8pm_motion_summary.json`. Voice uses this `CommandBus` (`stand` | `stop` | `vel(vx, yaw_rate)`), not a kitchen script.

Step-cycle basin (same caps, same plant): lateral COM shift **0.16 rad** on every step (a 0.10 rad straight/left shift was reverted after it raised slip; was 0.275 before that), forward mid-swing knee lift **+0.38 rad**, contralateral arm scale **11** while going straight and **3.4** while yaw is commanded, slewed, inside ±0.7 Nm. Straight shoulder travel is about 0.67 rad peak-to-peak. Scale 6 held through a cold left turn tips (min up_z 0.42) and is not shipped. Yaw is only on the airborne foot (**0.10 rad** left / **0.12 rad** right toe; stance hip yaw 0). Reverse does not take the knee lift or the arm scale. This pack measures approach **+0.695 m**, left arc **+61.1 deg**, resume **+0.340 m**, min up_z **0.940**, peak leg torque **2.10 Nm**, end mode stand, CoP in the box. It does not match the previous +0.598 m / +75.7 deg envelope. Sole clearance is still about 2.4 cm median. Micro knee, hip, and ankle bumps of 0.03–0.08 rad, a slightly shorter double-support, and a slightly longer period were measured on this arm basin and not shipped: they tip, or they move the left arc well away from +61 deg, or the median sole barely changes. A swing residual (hold, contact preload, stance push) was measured on the same gait and left off: the swing knee is already on the ±2.1 Nm clip, contact during swing is zero, and the upright residuals only move the straight-walk median from 2.31 cm to at most 2.84 cm while yawing the heading. Details are in `docs/DEMO_8PM_MOTION.md`. Short reverse is **−0.415 m**, min up_z **0.905**. Soft-pass is off. See `docs/DEMO_8PM_MOTION.md`.

Older clip stats in the next paragraph (+0.598 m, +75.7 deg, and the matching voice numbers) are the previous basin.

`forward` is stand → forward → stop, long enough for several body lengths. `stop` is the same bus on a gait phase that used to pitch after the stop (support margin about −0.08 m) and latch `mode=fault`. `reverse` is stand → retreat → stop. `turn` is stand → walk while yawing left → stop. `turn-right` is the same 11 s window with yaw_rate −0.25. `nav-left` and `nav-right` are the furniture pattern: walk about 0.6 m, arc with `vel(vx, yaw_rate)` held together, walk a bit further along the new heading, then stop. `nav-multi` chains both claimed holds on one continuous walk: forward, left arc (12.5 s), forward, right arc (11 s), forward, then stop. Each writes `previews/steer_walk_<clip>.mp4` and a JSON summary (Δx, forward yaw drift, Δyaw, tip, CoP in box, peak torque, end mode; `nav-multi` also has per-segment Δx, Δyaw, mean body vx, mean yaw rate, and peak torque). Keys, when a display exists: **W/S** ±vx, **A/D** ±yaw (A = left), **Space** stop. Keys latch until Space. AI resends `vel` at 10 Hz; 200 ms of silence stands.

Forward clamp is **+0.056 m/s**, reverse **−0.032 m/s**, yaw **±0.25 rad/s**. The forward clamp was lowered from 0.080 because that command yaws off and tips near 1.2 m. CPG amplitude stays `|applied_vx| / 0.080`, so full forward stick is amplitude 0.70 and reverse stays amplitude 0.40. On the forward clip the body covers **+2.161 m**, stays upright (min up_z 0.967), CoP in the box, peak leg torque 2.10 Nm, and ends in stand (support margin +0.063 m). Mean body vx over that window is **+0.039 m/s**, not the 0.056 command. Net yaw from the first forward sample to the last moving sample is **−0.71 deg**. During the walk yaw rocks between about **−15 deg and +11 deg** and does not run away the way 0.080 does (that reached about +40 deg and tipped near 1.2 m). Heading at the end of the clip is about **−3.1 deg** and holds in stand. Reverse raises the stance-slip damper to 100 N/(m/s) while backing up (clip Δx −0.967 m). After stop that damper stays for 1.20 s (and a reverse cut in a pitching phase finishes at most one step first), then it is off. Turning lengthens the outside step. On the shared 11 s window the left clip is **+61.8 deg** (Δx +0.418 m). The right clip used to stall near **−16 deg** because the full hip-yaw clip toed the swing foot the wrong way; the swing leg now keeps 0.60 of that command and the same window measures **−83.4 deg**, Δx **+0.459 m**, mean body vx **+0.052 m/s**, mean yaw rate **−0.133 rad/s** (cap ±0.25), min up_z 0.955, CoP in the box, peak leg torque 2.10 Nm, end margin +0.063 m, no tip. After the gait has already been walking for 12 s, a left yaw command drops the 0.06 rad bias (that drift has settled) and uses outside-step scale 0.24. Cold-start turns finish before that gate. The nav clips use the same caps: approach **+0.60 m**, then `vel(+0.056, ±0.25)` together, then another short forward, then stop. Left arc **+75.7 deg** in 12.5 s (mean body vx +0.044 m/s during the arc, resume +0.32 m). Right arc **−77.3 deg** in 11 s (mean body vx +0.053 m/s, resume +0.32 m). Both end in stand with margin +0.063 m, CoP in the box, peak leg torque 2.10 Nm, no tip. A left arc started one second earlier and held for 14 s tips on the resume; that window is not claimed. `nav-multi` stays inside those same phase lengths and the same caps, with no plant change and no extra settle. Approach **+0.598 m** (mean body vx +0.038 m/s), left arc **+75.7 deg** in 12.5 s (mean body vx +0.044 m/s, mean yaw rate +0.106 rad/s), straight **+0.317 m** (Δyaw −3.0 deg), right arc **−54.9 deg** in 11 s (mean body vx +0.039 m/s, mean yaw rate −0.087 rad/s), then another straight **+0.227 m**. That last straight still walks, and heading drifts back **+30.7 deg** before the stop. End heading is **+52.9 deg**. Min up_z 0.954, CoP in the box, peak leg torque 2.10 Nm, end mode stand, end margin +0.063 m, no tip. The chained right arc is shorter than the single-arc **−77.3 deg**; both holds are the claimed durations, and vx and yaw are applied together. Prefer FAIL, not claimed: a second 11 s right after a right tips; a second 12.5 s left after a left does not yaw and then tips; right-then-left stays upright but the left hold does not yaw left. Not a door or Gate Q claim.

## Voice commands (same CommandBus)

`scripts/voice_caller.py` turns a typed phrase into `stand`, `stop`, or `vel(vx, yaw_rate)` on the bus above. It resends `vel` at 10 Hz. 200 ms of silence stands. Caps stay **+0.056 / −0.032** m/s and yaw **±0.25** rad/s. Walk forward is `vel(+0.056, 0)`. Turn left / turn right is walk-yaw `vel(+0.056, ±0.25)`. `vx = 0` yaw does not change heading and is refused. The clip speaks the claimed nav-left window: stand, walk forward, turn left, walk forward, stop. Kitchen, bathroom, SLAM, maps, waypoints, and strafe are refused. The finders are separate scripts. This is not go-anywhere. Phrases and the refusal lines are in `docs/VOICE_COMMANDS.md`.

```bash
python scripts/voice_caller.py "turn left"
python scripts/voice_caller.py --self-test
MUJOCO_GL=osmesa python scripts/voice_caller.py --clip
```

The clip is that left arc with the phrase and the bus command on each frame. `kit_cam` is not moved. The voice mp4 on main matched the previous basin (approach **+0.598 m**, left arc **+75.7 deg**). This step-cycle basin changes those walk numbers; see `docs/DEMO_8PM_MOTION.md`. The bus API is unchanged.

## kit_cam rooms (vision only, not a map)

Each scene includes the frozen walk plant and adds static furniture in front of `kit_cam` (+X). `kit_cam` is unchanged: one camera, `head_tilt_link`, `0.050 0.019 0.007`. The floor is still the plant plane. Body names are labels for a later vision step. They are not a navigation map, and these scenes are not handed to a planner. `scripts/steer_walk.py` still loads only the frozen plant by default, so stand / forward / reverse / left / right / left-then-right stay on the empty checkerboard. See `docs/KIT_CAM_ROOM.md`.

| Scene | Named bodies |
|-------|----------------|
| `mujoco/room_kitchen.xml` | `kitchen`, `table`, `chair` |
| `mujoco/room_bathroom.xml` | `bathroom`, `sink`, `toilet` |
| `mujoco/room_living.xml` | `living` (sofa), `tv` |
| `mujoco/room_bedroom.xml` | `bedroom` (bed), `nightstand` |
| `mujoco/room_entrance.xml` | `entrance` (visual frame only; no hinge, no lever, no latch), `mat` |

"Go to the kitchen" is a Prefer FAIL finder in `scripts/find_kitchen.py`. It runs only when `room_kitchen.xml` is loaded. The stand frame is painted into the explore map. If `query_kitchen_like_yellow()` is false, it sends no vel. If yellow was logged, each slice calls that query and `frontier_cells()` and sends half-cap `vel(+0.028, yaw)` toward the frontier nearest the logged camera ray. The live blob is the arrival fraction, not the command. The `up_z` 0.90 stop and the 1.10 m world-x budget are unchanged. `vx = 0` yaw is not used. This run logged yellow (max fraction **0.148**) and stopped Prefer FAIL `close`: end x **+1.076 m** (Δx **+1.074 m**), min up_z **0.979**, remaining **0.239 m**, settled yellow **0.000**. The gap is inside 0.25 m and the frame is not half backsplash, so it is not arrival. The 1.2 s hop remains a self-test at the full cap (end x +0.036 m). The empty plant still sends no velocity. See `docs/FIND_KITCHEN.md`.

The living, bedroom, and entrance scenes are vision stills only. They do not add a go-to. "Go to the living room", "go to the bedroom", and "go to the entrance" stay refused. Prefer FAIL until an AI finder for that room lands. The entrance body is a visual frame: no hinge, no lever, no latch.

## Explore / map (partial, Prefer FAIL)

`scripts/explore_map.py` is not a room finder. It resends `vel` at 10 Hz on the same `CommandBus` and paints a 0.10 m grid from `kit_cam` only. Floor rays out to 2.60 m become free cells, and one-cell holes in that fan are filled. A saturated floor hit (the entrance mat) becomes a feature cell. The yellow backsplash does not meet the floor plane in range, so it is a camera-ray bearing, not a waypoint. `find_kitchen.py` is unchanged: it still uses the 1.80 m paint and the 4-connected 1.60 m frontier ring, and it does not read a soft XY. The empty plant holds the claimed left prefix (stand 1.0 s, `vel(+0.056, 0)` for 15 s, `vel(+0.056, +0.25)` for 12.5 s), then a 4 s forward gap and an 8 s left window when the dense rim is still left, then one `vel(+0.056, −0.25)` for 34 s. A 6 s gap swallows that left window. About 38 s of the right hold crosses `up_z` 0.90. Furnished scenes use `vel(+0.056, −0.25)` for 8 s, because the claimed yaw windows cross `up_z` 0.90 there. `vx = 0` yaw is not sent. Empty plant free cells **883** (open-loop chain 821), frontiers **41** (39), walked cells **57** (52), min up_z **0.939** (0.949), no tip. A separate soft-XY probe is Prefer FAIL and is not this path and not arrival. Voice still refuses "explore" and "go anywhere". The rooms stay separate XML files. See `docs/EXPLORE_MAP.md`.

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
