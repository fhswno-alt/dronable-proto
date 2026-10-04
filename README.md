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

Velocity only (`stand` / `stop` / `vel`). Voice later calls the same `CommandBus` in `scripts/steer_walk.py`. Plant is `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (md5 `e3feef97…`) with Hardware `kit_cam` on `head_tilt_link` (zero-mass site, fovy 104.82). Foot box, friction, kp, and ±2.1 Nm are unchanged. No OptC / door plant.

```bash
MUJOCO_GL=osmesa python scripts/steer_walk.py
MUJOCO_GL=osmesa python scripts/steer_walk.py --no-video
MUJOCO_GL=glfw  python scripts/steer_walk.py --view
python scripts/steer_walk.py --self-test
```

Headless demo writes `previews/steer_walk_forward.mp4` (stand → forward → stop) and `previews/steer_walk_forward_summary.json`. Keys, when a display exists: **W/S** ±vx, **A/D** ±yaw (A = left), **Space** stop. Keys latch until Space. AI resends `vel` at 10 Hz; 200 ms of silence stands. Full-stick forward is a 0.08 m/s command; the summary records realized Δx and mean body vx. Reverse is a best-effort sagittal mirror and can tip. Yaw is still a hip-yaw bias, not a verified spin. Not a door or Gate Q claim.

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
