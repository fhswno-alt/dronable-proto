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

Velocity only (`stand` / `stop` / `vel`). `scripts/voice_caller.py` calls the same `CommandBus` in `scripts/steer_walk.py`. Plant is `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (md5 `71b2c86d…`) with Hardware `kit_cam` on `head_tilt_link` at pos `0.050 0.019 0.007` (zero-mass site, same aim, fovy 104.82). Foot box, friction, kp, and ±2.1 Nm are unchanged. No OptC / door plant.

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

`forward` is stand → forward → stop, long enough for several body lengths. `stop` is the same bus on a gait phase that used to pitch after the stop (support margin about −0.08 m) and latch `mode=fault`. `reverse` is stand → retreat → stop. `turn` is stand → walk while yawing left → stop. `turn-right` is the same 11 s window with yaw_rate −0.25. `nav-left` and `nav-right` are the furniture pattern: walk about 0.6 m, arc with `vel(vx, yaw_rate)` held together, walk a bit further along the new heading, then stop. `nav-multi` chains both claimed holds on one continuous walk: forward, left arc (12.5 s), forward, right arc (11 s), forward, then stop. Each writes `previews/steer_walk_<clip>.mp4` and a JSON summary (Δx, forward yaw drift, Δyaw, tip, CoP in box, peak torque, end mode; `nav-multi` also has per-segment Δx, Δyaw, mean body vx, mean yaw rate, and peak torque). Keys, when a display exists: **W/S** ±vx, **A/D** ±yaw (A = left), **Space** stop. Keys latch until Space. AI resends `vel` at 10 Hz; 200 ms of silence stands.

Forward clamp is **+0.056 m/s**, reverse **−0.032 m/s**, yaw **±0.25 rad/s**. The forward clamp was lowered from 0.080 because that command yaws off and tips near 1.2 m. CPG amplitude stays `|applied_vx| / 0.080`, so full forward stick is amplitude 0.70 and reverse stays amplitude 0.40. On the forward clip the body covers **+2.161 m**, stays upright (min up_z 0.967), CoP in the box, peak leg torque 2.10 Nm, and ends in stand (support margin +0.063 m). Mean body vx over that window is **+0.039 m/s**, not the 0.056 command. Net yaw from the first forward sample to the last moving sample is **−0.71 deg**. During the walk yaw rocks between about **−15 deg and +11 deg** and does not run away the way 0.080 does (that reached about +40 deg and tipped near 1.2 m). Heading at the end of the clip is about **−3.1 deg** and holds in stand. Reverse raises the stance-slip damper to 100 N/(m/s) while backing up (clip Δx −0.967 m). After stop that damper stays for 1.20 s (and a reverse cut in a pitching phase finishes at most one step first), then it is off. Turning lengthens the outside step. On the shared 11 s window the left clip is **+61.8 deg** (Δx +0.418 m). The right clip used to stall near **−16 deg** because the full hip-yaw clip toed the swing foot the wrong way; the swing leg now keeps 0.60 of that command and the same window measures **−83.4 deg**, Δx **+0.459 m**, mean body vx **+0.052 m/s**, mean yaw rate **−0.133 rad/s** (cap ±0.25), min up_z 0.955, CoP in the box, peak leg torque 2.10 Nm, end margin +0.063 m, no tip. After the gait has already been walking for 12 s, a left yaw command drops the 0.06 rad bias (that drift has settled) and uses outside-step scale 0.24. Cold-start turns finish before that gate. The nav clips use the same caps: approach **+0.60 m**, then `vel(+0.056, ±0.25)` together, then another short forward, then stop. Left arc **+75.7 deg** in 12.5 s (mean body vx +0.044 m/s during the arc, resume +0.32 m). Right arc **−77.3 deg** in 11 s (mean body vx +0.053 m/s, resume +0.32 m). Both end in stand with margin +0.063 m, CoP in the box, peak leg torque 2.10 Nm, no tip. A left arc started one second earlier and held for 14 s tips on the resume; that window is not claimed. `nav-multi` stays inside those same phase lengths and the same caps, with no plant change and no extra settle. Approach **+0.598 m** (mean body vx +0.038 m/s), left arc **+75.7 deg** in 12.5 s (mean body vx +0.044 m/s, mean yaw rate +0.106 rad/s), straight **+0.317 m** (Δyaw −3.0 deg), right arc **−54.9 deg** in 11 s (mean body vx +0.039 m/s, mean yaw rate −0.087 rad/s), then another straight **+0.227 m**. That last straight still walks, and heading drifts back **+30.7 deg** before the stop. End heading is **+52.9 deg**. Min up_z 0.954, CoP in the box, peak leg torque 2.10 Nm, end mode stand, end margin +0.063 m, no tip. The chained right arc is shorter than the single-arc **−77.3 deg**; both holds are the claimed durations, and vx and yaw are applied together. Prefer FAIL, not claimed: a second 11 s right after a right tips; a second 12.5 s left after a left does not yaw and then tips; right-then-left stays upright but the left hold does not yaw left. Not a door or Gate Q claim.

## Day-1 voice caller (same bus)

Short phrases become `stand`, `stop`, or `vel` on that bus. `vel` is resent at 10 Hz. Silence longer than 200 ms stands inside the bus. Walk forward publishes **+0.056 m/s**, the bus forward cap, and resends it at 10 Hz. It does not send 0.080. 0.080 yaws and tips near 1.2 m. Controls' clip at +0.056 moved about **+2.16 m**, mean body speed about **+0.039 m/s**, then stood, end margin about **+0.063 m**. The command 0.056 is not the odometry. The caller does not claim faster or farther than that clip. Back up sends the bus clamp **−0.032 m/s** and resends it at 10 Hz. Controls' merged reverse clip moved about **−0.97 m** at about **−0.053 m/s**, upright, no tip. The caller does not send a more negative vx and does not claim faster than that clip. A turn while already walking is the nav window, not a cold start from the first step. It publishes forward **+0.056 m/s** together with yaw **±0.25 rad/s** and resends that at 10 Hz. It does not send 0.080 or a yaw past ±0.25. The command is not the heading rate. Right nav walks forward from t=1 to t=16 (approach about **+0.60 m**), holds `vel(+0.056, −0.25)` until t=27 (heading about **−77 deg** while still walking), walks forward until t=33 (resume about **+0.32 m**), and stops by t=35.5. End stand, margin about **+0.063 m**. Left uses the same approach, then `vel(+0.056, +0.25)` until t=28.5 (about **+76 deg**), forward until t=34.5, stop by t=37. The kit_cam clip renders that left window. A left yaw started at 15 s and held for 14 s tips on the resume and is not claimed. Cold-start from stand stays about **+62 deg** left and about **−83 deg** right. Those are not this arc. Yaw with zero forward is only a turn in place. Neither arc is a spin. Go to the kitchen or the bathroom is refused: the camera can see, but there is no room and no map. The tip predicate is still COM outside support with `up_z < 0.85`. It was not loosened.

```bash
python scripts/voice_caller.py "walk forward"
python scripts/voice_caller.py "back up"
python scripts/voice_caller.py "go to the kitchen"
python scripts/voice_caller.py --self-test
MUJOCO_GL=osmesa python scripts/voice_caller.py --clip
```

`--clip` renders plant `kit_cam` (pos `0.050 0.019 0.007`, not moved) during back up, then walk forward, then stop. Stills and `previews/voice_caller_kit_cam.mp4` are what that camera records. The floor is empty. Not autonomous navigation.

Model: `mujoco/dronable_v0.xml`  
Gait params & teleop stubs: top of `scripts/walk_gait.py` (wired to Controls/AI brief).

## Preview outputs

- `previews/walk_gait.mp4` — stepping gait
- `previews/cad_static.png` — CAD silhouette
- `previews/sim_run_log.txt` — last run proof
- `previews/ego_cam/` + `EGO_CAM_REPORT.md` — kit-cam FOV / neck-pitch approach stills (Mon review)

## Model summary

~407 mm tall CAD, **21 actuated DOF** (incl. neck pitch) (+ 6-DoF freejoint base), ~**2.66 kg** sim mass, Pi-first (no Orin).  
v0 includes temporary balance assist for visual stability — see ASSUMPTIONS.md.
