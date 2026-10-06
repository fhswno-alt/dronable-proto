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

Locked-kit nav-left on Controls tip `d00efbf` (`gm_resume_lead="outside"`, double-support chase). Cold start stays left-first. Same window: 1 s stand, 15 s forward, 12.5 s `vel(+0.150, +0.25)`, 6 s `vel(+0.150, 0)`. Approach heading **+2.210 m** / **+1.39 deg**. Left arc heading **+0.422 m** / **+144.99 deg**. Resume heading **+0.904 m** / **+2.24 deg**. min up_z **0.928**. No fault. End stand. On `08731c0` this resume was **+0.907 m** / **+2.57 deg**. The pre-outside-lead resume on this window was **+12.41 deg**. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` unchanged. Soft-pass is off. Not go-anywhere.

Left versus right unload on that kit row. The 6 s `vel(+0.150, 0)` after the left turn is +12.3°: +6.7° while `applied_yaw` slews +0.25→0 in 0.69 s (command integral +5.3°), then +5.6° with the yaw command and the step angle already 0. The 6 s `vel(+0.150, 0)` after an 11 s `vel(+0.150, −0.25)` is −2.2° on the chained walk and −0.6° when that right turn follows a straight approach. The slew is 0.40 rad/s² both ways. On a matching step phase the right ramp moves the body −5.3° (command integral −4.5°), and the rest of the window then curves left +4.7°, so the ramp and the curve cancel. Straight `vel(+0.150, 0)` with no turn already curves +4.2° over 27–33 s and +3.5° over 28.5–34.5 s. The left ramp and that curve add. Hip-yaw targets are 0 once the step angle is 0. `a_move` follows the yaw sign and the shift stays +|a|. Steady rates are +0.217 and −0.233 rad/s. A pelvis-0 probe still reads +11.3° versus −0.4°. Stop snaps yaw to 0 in one tick, and the heading kick follows the step phase: after a right turn it runs about +11° to −10° across 0.37 s of phase.

That straight curve is not a steady left bias. Five cold starts of 1 s stand then 30 s `vel(+0.150, 0)` are the same trajectory (std 0). Every one starts in double support at gait time 0 and the first swing foot is the left foot, because stand resets the step clock and the cycle swings left first. Δyaw is −1.46° at 6 s, +1.39° at 15 s, and +0.50° at 30 s. Mean body yaw rate is +0.0003 rad/s. Two-second slices rock about −2.9° to +1.9°. An extra 0.25 s of stand does not change the lead foot. Starting the clock half a period later (probe only) swings the right foot first and the 30 s net is −2.55°. Soft-pass is off.

Cold stand→forward stays left-first. Right-first on the same 30 s is −2.60° at 30 s, a larger heading error, so it is not the default. First-step knees after a 1 s stand: left-lead 1.152 / 1.749 Nm, right-lead 1.749 / 1.152 Nm, both under 2.33. After a yaw target returns to 0 the next double support swings the outside foot. A one-tick clock park stepped the joint targets 0.342 rad; the pose now chases the live gait over that double support (0.056 s) and the largest tick is 0.062 rad, the same as the walk's own largest tick. The post-left 6 s is +2.1°. The post-right 6 s does not move the clock and is −0.5°. The left turn (12.5 s) finishes at +145.0° and the right turn (11.0 s) at −136.5°. The right arc is 8.5° shorter because that hold is shorter; the right body rate is −0.217 rad/s against the left's +0.202 rad/s.

No-ask chain on this tip, kit gait, `vel(+0.150, yaw)`, no scene question. Heading is travel along the heading at the start of the window. The pose chases the live gait across double support (0.056 s, 7 ticks, largest tick 0.062 rad) instead of parking that clock in one tick (0.342 rad). Straight walks rock about −2.9° to +1.9° in two-second slices. That is the gait account. It is not a plant change.

| Phase | `d00efbf` | #55 on `08731c0` | Controls isolated |
|-------|-----------|------------------|-------------------|
| approach 15 s | +2.210 m / +1.39 deg | +2.210 m / +1.39 deg | — |
| left 12.5 s | +0.422 m / +144.99 deg / +0.203 rad/s | +0.422 m / +144.99 deg | +145.0 deg / +0.202 rad/s |
| mid 6 s | +0.904 m / **+2.24 deg** | +0.907 m / **+2.57 deg** | **+2.1 deg** |
| right 11 s | +0.550 m / −128.01 deg / −0.203 rad/s | +0.551 m / −128.02 deg | −136.5 deg / −0.217 rad/s |
| resume 6 s | +0.900 m / **−3.66 deg** | +0.899 m / **−4.14 deg** | **−0.5 deg** |

Approach and the cold left arc stay with the `08731c0` row. The mid is the post-left 6 s: **+2.24 deg** here, **+2.57 deg** on `08731c0`, Controls' isolated **+2.1 deg**. The chained resume is **−3.66 deg**, against **−4.14 deg** on `08731c0`. That is not the isolated post-right **−0.5 deg**. On this chain the left rate is **+0.203 rad/s** and the right rate is **−0.203 rad/s**. The angle ratio 128.01/144.99 is 0.883 and the hold ratio 11.0/12.5 is 0.880, so the shorter right arc is the shorter hold. Controls' isolated right is the straight-approach window, −136.5 deg at −0.217 rad/s. Whole chain min up_z **0.923**, no fault, end stand. World at the end of the 2.5 s stop **+2.873 m, +3.411 m**, yaw **+13.50 deg**. Nav-left resume matches the chain mid. Kitchen and bathroom 8 s `vel(+0.150, −0.25)` stay heading **+0.540 m** / **−94.61 deg**, min up_z **0.929**, settled yaw **−114.36 deg**, same trace in both rooms.

Stand `kit_cam` height and pitch, plant files read only. The mount string is the same on both: `head_tilt_link`, `0.050 0.019 0.007`, xyaxes `0 -1 0 0 0 1`, fovy 104.82. Pitch is the look direction above the horizontal. The living-collapse trio is a 1.0 s kit-gait stand. Kitchen, bathroom, and bedroom match the empty stand: height **0.335 m**, pitch **−14.84 deg**. The main-freeze plant `71b2c86d133ebc603f58b99c53e496f3` on that same kit stand is height **0.338 m**, pitch **−14.81 deg**, look yaw **−1.22 deg** against **+0.01 deg** here. The plant file moves the kit-stand camera by **2.6 mm** and **0.03 deg** of pitch. The empty #47 `kit/plant.png` matches the CPG quiet stand (0.60 s, `gait_targets`, COM_Z 0.225): mean absolute pixel difference **0.74** on this plant and **1.59** on `71b2c86d`, against about **20** for the kit stand. That CPG stand is height **0.374 m** and pitch **−0.56 deg** on this plant (**0.376 m**, **−0.52 deg** on the old plant). The trio viewpoint sits **39 mm** lower and **14.3 deg** more nose-down than that #47 empty still. The kitchen #47 still does not match today's room XML under either stand. This comparison does not assign the living-collapse to Moondream.

Head-tilt ask-pose during the 0.20 s stop, Prefer FAIL. Joint `head_tilt` has actuator `head_tilt_pos` (axis `0 -1 0`, range ±2.09 rad, force ±0.7 Nm). Day1 `CommandBus.vel` still refuses a `head_tilt` key. The probe wrote `q_stand["head_tilt"]` inside the stand hold only. It did not touch the plant or the bus. The command was frozen from the pitch map before any label: **+0.25 rad**. Positive raises the look. +0.15 rad still looks down (−6.5 deg). +0.35 rad looks up (+4.4 deg). +0.25 rad is the near-level pose. After 0.20 s the joint reads **+0.245 rad**.

| Pose | kit_cam height | pitch | empty | kitchen | bathroom | living | bedroom |
|------|----------------|-------|-------|---------|----------|--------|---------|
| Quiet 1.0 s stand | 0.335 m | −14.84 deg | none | living | living | living | living |
| `head_tilt` +0.25 rad | 0.347 m | −0.99 deg | none | living | living | living | living |

Kitchen, bathroom, and bedroom stay **living** under the raised look. True labels on that trio stay **0/3**. Empty stays **none**. Moondream2 pin `5d6c926f44e26b07957b0dd315bbedcb4c17a5fe` stays the primary. This pose does not separate the rooms, so Controls does not take an ask-pose from this probe. Not go-anywhere. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` unchanged.

Scene and frustum at that same 1.0 s kit stand, Prefer FAIL. Top-of-frame crop was not scored. The size classes were frozen before the pixel counts: missing, out of frustum, occluded, tiny (under **3072** px, 1% of 640×480, or a mask shorter than **36** px), in-frame box, or visible enough (a mesh that clears that bar). The stand is the ask viewpoint. Bus mode stays `stand`. Every room stops at **+0.057 m, +0.000 m**, yaw **+0.01 deg**. `kit_cam` is **(0.146, 0.000, 0.335) m**, pitch **−14.84 deg**, look yaw **+0.01 deg**.

The four furnished stills SmolVLM named correctly are the kit frames in commit `bb94512` (`kitchen`, `bathroom`, `living`, `bedroom`). Those room files are textured meshes, a floor, and walls, including `kitchen_stove`. This branch's room files are colored boxes on the plant checkerboard. There is no stove geom. A shared geom name is not the same shape. The mesh rooms were loaded read-only with this plant's `kit_cam`, so the eye matches. The plant file was not edited.

| Prop | This branch, this stand | `bb94512` meshes, this same eye |
|------|-------------------------|--------------------------------|
| stove | **missing** | visible enough, **10430 px** (3.40%), short side 114, bearing **+34.2 deg** |
| toilet | in-frame box, **29450 px** (9.59%), short side 211, bearing **−41.2 deg**, bbox x 427–639 (clipped on the right edge) | visible enough, **3818 px** (1.24%), short side 59, bearing **+2.7 deg** |
| bed | in-frame box, **114667 px** (37.33%), short side 261, bearing **0.0 deg** | visible enough, **20901 px** (6.80%), short side 161, bearing **+5.2 deg** |

The crouch ask already contains the current toilet and the current bed. They are boxes. The mattress, the kettle, and the towel have **0** pixels and still project into the image, so they are occluded, not outside the frustum. The stove is not in `room_kitchen.xml`, so no stop heading on this scene can put one in frame. On the mesh kitchen that same stand already shows the stove (center **1.870, 1.235, 0.467 m**). A heading of **−41.2 deg** at this xy would center the current box toilet. The bed is already on the look axis (yaw offset **−0.01 deg**). Controls owns whether a stop uses those headings. This probe does not command one. The mesh kitchen and bathroom ceilings are already out of the crouch frame (0 px). Not go-anywhere. Plant md5 `207f3d5e9c6a72e16f7aa0c8d224f75e` unchanged. Stills and the geom counts are in `previews/frustum_stop_ask/`.

Forward clamp is **+0.056 m/s**, reverse **−0.032 m/s**, yaw **±0.25 rad/s**. The forward clamp was lowered from 0.080 because that command yaws off and tips near 1.2 m. CPG amplitude stays `|applied_vx| / 0.080`, so full forward stick is amplitude 0.70 and reverse stays amplitude 0.40. On the forward clip the body covers **+2.161 m**, stays upright (min up_z 0.967), CoP in the box, peak leg torque 2.10 Nm, and ends in stand (support margin +0.063 m). Mean body vx over that window is **+0.039 m/s**, not the 0.056 command. Net yaw from the first forward sample to the last moving sample is **−0.71 deg**. During the walk yaw rocks between about **−15 deg and +11 deg** and does not run away the way 0.080 does (that reached about +40 deg and tipped near 1.2 m). Heading at the end of the clip is about **−3.1 deg** and holds in stand. Reverse raises the stance-slip damper to 100 N/(m/s) while backing up (clip Δx −0.967 m). After stop that damper stays for 1.20 s (and a reverse cut in a pitching phase finishes at most one step first), then it is off. Turning lengthens the outside step. On the shared 11 s window the left clip is **+61.8 deg** (Δx +0.418 m). The right clip used to stall near **−16 deg** because the full hip-yaw clip toed the swing foot the wrong way; the swing leg now keeps 0.60 of that command and the same window measures **−83.4 deg**, Δx **+0.459 m**, mean body vx **+0.052 m/s**, mean yaw rate **−0.133 rad/s** (cap ±0.25), min up_z 0.955, CoP in the box, peak leg torque 2.10 Nm, end margin +0.063 m, no tip. After the gait has already been walking for 12 s, a left yaw command drops the 0.06 rad bias (that drift has settled) and uses outside-step scale 0.24. Cold-start turns finish before that gate. The nav clips use the same caps: approach **+0.60 m**, then `vel(+0.056, ±0.25)` together, then another short forward, then stop. Left arc **+75.7 deg** in 12.5 s (mean body vx +0.044 m/s during the arc, resume +0.32 m). Right arc **−77.3 deg** in 11 s (mean body vx +0.053 m/s, resume +0.32 m). Both end in stand with margin +0.063 m, CoP in the box, peak leg torque 2.10 Nm, no tip. A left arc started one second earlier and held for 14 s tips on the resume; that window is not claimed. `nav-multi` stays inside those same phase lengths and the same caps, with no plant change and no extra settle. Approach **+0.598 m** (mean body vx +0.038 m/s), left arc **+75.7 deg** in 12.5 s (mean body vx +0.044 m/s, mean yaw rate +0.106 rad/s), straight **+0.317 m** (Δyaw −3.0 deg), right arc **−54.9 deg** in 11 s (mean body vx +0.039 m/s, mean yaw rate −0.087 rad/s), then another straight **+0.227 m**. That last straight still walks, and heading drifts back **+30.7 deg** before the stop. End heading is **+52.9 deg**. Min up_z 0.954, CoP in the box, peak leg torque 2.10 Nm, end mode stand, end margin +0.063 m, no tip. The chained right arc is shorter than the single-arc **−77.3 deg**; both holds are the claimed durations, and vx and yaw are applied together. Prefer FAIL, not claimed: a second 11 s right after a right tips; a second 12.5 s left after a left does not yaw and then tips; right-then-left stays upright but the left hold does not yaw left. Not a door or Gate Q claim.

## Voice commands (same CommandBus)

`scripts/voice_caller.py` turns a typed phrase into `stand`, `stop`, or `vel(vx, yaw_rate)` on the bus above. It resends `vel` at 10 Hz. 200 ms of silence stands. Caps stay **+0.056 / −0.032** m/s and yaw **±0.25** rad/s. Walk forward is `vel(+0.056, 0)`. Turn left / turn right is walk-yaw `vel(+0.056, ±0.25)`. `vx = 0` yaw does not change heading and is refused. The clip speaks the claimed nav-left window: stand, walk forward, turn left, walk forward, stop. Kitchen, bathroom, SLAM, maps, waypoints, and strafe are refused. The finders are separate scripts. This is not go-anywhere. Phrases and the refusal lines are in `docs/VOICE_COMMANDS.md`.

On the locked kit CommandBus the same window, commanded as `vel(+0.150, …)`, is the nav-left row in the table above: resume heading **+0.904 m** / **+2.24 deg** on tip `d00efbf`. The phrase table in this section still publishes the 0.056 m/s voice clip.

Before the outside-foot resume, the same 6 s `vel(+0.150, 0)` after an 11 s right turn was −2.2° chained and −0.6° from a straight approach, not a mirror of the +12.3°. The live chain is the table above. The slew is 0.40 rad/s² both ways. On a matching step phase the right ramp is −5.3° of body yaw (command integral −4.5°), then the walk curves left +4.7° with `applied_yaw` and the step angle already 0, and those cancel. Straight walking on those clocks already curves +4.2° (27–33 s) and +3.5° (28.5–34.5 s). Hip-yaw targets are 0 in that stretch. Stop snaps yaw to 0 in one tick; after a right turn the heading kick runs about +11° to −10° across 0.37 s of step phase. Five cold straight walks net +0.50° over 30 s (mean rate +0.0003 rad/s) and the cold start still swings the left foot first. After a turn, the straight resume swings the outside foot and the pose chases the live gait over that double support instead of stepping 0.342 rad in one tick. The post-left 6 s is +2.1°, and the post-right 6 s stays −0.5°. The left turn finishes at +145.0° in 12.5 s; the right turn finishes at −136.5° in 11.0 s. The right arc is 8.5° shorter because the hold is shorter, not because the right rate is low.

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
