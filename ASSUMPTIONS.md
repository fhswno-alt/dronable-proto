# Dronable v0 — Assumptions

**Prototype date:** Sun 27 Sep 2026 (London / BST)  
**Design review:** Mon 29 Sep 2026  
**Path A (buy AiNex-class) is NOT locked** — geometry here is a build/source envelope under ~$1.5k, not a purchase decision.

---

## Locked product goals (do not invent conflicting numbers)

| Item | Value | Owner |
|------|-------|-------|
| Class | Small biped ~415 mm (knee-to-waist / AiNex Standard class) | Product |
| Budget ceiling | ≤ ~$1.5k low-volume source/build | Product |
| Actuators | Hiwonder-class bus servos (HX-35H hip/leg, HX-12H smaller) | Hardware |
| Compute | Pi 5 class onboard; **no Orin / no Jetson mass** | AI |
| Face | Eyes-only thin display later (≤1–2 W); no mouth | AI / Product |
| Lever target | Door lever height **0.25–0.30 m** | Product |
| First walk | Controls **50 Hz PD gait**, not on-Pi NN; vision/voice off-board | Controls / AI |

### Controls (wired into MJCF + `walk_gait.py`)

- **PD gains:** kp **40** hips/knees, **25** ankles, **20** arms/hands; kd **1.0 / 0.6 / 0.4** (same groups; `dampratio` on actuators + joint damping).
- **Torque clip:** stall × **0.6** → HX-35H ≈ **±2.1 Nm**, HX-12H ≈ **±1.1 Nm** (`forcerange` on actuators). Fall should trip torque limit, not explode.
- **Control rate:** **50 Hz** joint command.
- **Gait:** T=**0.5 s**, double support ~**0.1 s**; step length **0.06 m**; foot clearance **0.025 m**; hip pitch amp **±0.25 rad**; knee stance ~**0.35 rad**, swing peak ~**0.9 rad**; COM height **~0.22–0.24 m**.
- **Teleop fallback (stub wired):** Xbox → joint Δq, rate-limit **1.5 rad/s**, deadband **0.08**; gait on/off; soft stop = hold last q.

### AI envelope (documented; not simulated as NN)

- Pi 5 class; perception stubs ≤ ~**2 cores / 1–1.5 GB**.
- Cam stub: ~**640×480 @ ≤30 Hz**, **50–80 ms** latency; kit HFOV **120°** / VFOV ≈**104.8°** (Path A AiNex-class 2DOF pan-tilt). Stand mount **cam_z ≈ 385 mm**.
- IMU **100–200 Hz** for balance (sites/sensors present in MJCF).
- Eyes-only LCD ≤1–2 W; first walk = Controls gait, vision/voice off-board.

---

## v0 geometry / model guesses (open to revision Mon 29 Sep)

- CAD height ≈ **407 mm** (target ~415 mm); pelvis/hip height ≈ **219 mm**; COM in sim held ~**0.23 m**.
- **Mass estimate ~2.66 kg** (MuJoCo summed inertias) — plausible for plastic + bus servos; not weighed hardware.
- Link sizes / densities chosen for Hiwonder-class envelope; inertias are approximate boxes/capsules.
- **Frame convention:** Z-up, **+Y forward** (toes / long foot axis). Sagittal leg hinges (`hip_pitch`, `knee`, `ankle_pitch`) rotate about **+X**; frontal (`hip_roll`, `ankle_roll`) about **+Y** — matches arm pitch about +X. (An earlier MJCF draft had leg pitch/roll axes swapped, which made the CPG drive lateral mush instead of steps.)
- Joint ranges set to typical bus-servo travel, not measured on a specific SKU.
- Optional door panel + lever geom at **0.275 m** in CAD assembly and MJCF (visual target only).
- **Kit cam site** on head (forward **+Y**): stand **cam_z ≈ 0.385 m**; HFOV **120°** (MJCF `kit_cam` fovy≈104.82° @ 4:3). Depression to center lever ≈ **12° @ 0.5 m → 15° @ 0.4 m → 20° @ 0.3 m**. Default approach **neck_pitch = −16°** (no fab shim for unit one). See `previews/EGO_CAM_REPORT.md` / `previews/ego_cam/` (run_id in `ego_cam/run_id.txt`).

---

## DOF cuts (target class ~24 with hands → v0 walking core)

| Included in v0 (21 actuated) | Cut / deferred |
|------------------------------|----------------|
| Legs 6+6 (yaw/roll/pitch hip, knee, ankle pitch/roll) | Finger / gripper DOF |
| Waist yaw | Waist pitch/roll |
| Head yaw + **neck pitch** (2DOF pan-tilt; eyes fixed geom) | Actuated eye gimbals |
| Arms 3+3 (shoulder pitch/roll, elbow) | Wrist DOF; hand articulations |

Full 24-DOF hands-class remains a product goal; v0 prioritizes gait + reach silhouette.

---

## Still open

- Path A buy vs custom frame / servo brand lock.
- Exact HX-35H / HX-12H stall torque datasheet confirmation (clip uses 3.5 Nm / 1.8 Nm × 0.6).
- Real balance (LIPM / WBC / RL) — **v0 uses a temporary pelvis upright + height assist with gravity feedforward** (`v0_balance_assist` in `walk_gait.py`) so open-loop CPG can show stepping 5–10 s without saturating leg torque just to hold weight. Assist writes freejoint `qfrc_applied` plus a hard upright+**yaw=0 lock** (v4: unconstrained yaw spun ~230° and made the fixed “side” camera look like wobble/collapse while CSV foot-z still passed). v7: **fy_bias=0** (v6 0.10 still slid); hip stance/swing polarity flipped so stance push yields +Y; primary +Y from real step kinematics. L/R legs painted orange/green so mesh alternation is obvious. Preview video is PNG `frame_seq/` → ffmpeg all-intra (`-g 1 -bf 0`) so continuous playback cannot lie via B-frames. Assist does **not** overwrite joint `ctrl`. This is **not** production balance.
- Onboard vs offboard perception split; LCD driver. **Camera mount:** neck pitch restored in MJCF; Monday choose commanded look-down (recommend **−16°** approach, deepen to **≈−20°** at grasp) — not a fixed downward fab shim for unit one.
- Path A AiNex kit FOV/pan-tilt assumed (HFOV 120°); **Path A buy decision still not locked.**
- Structural materials, battery placement, harness mass.
- Design review **Mon 29 Sep** may change envelope.

---

## What “success” means for this prototype

CAD openable (STEP/STL/GLB) + MuJoCo MJCF with Controls-wired PD + periodic gait preview video + this note. Not a hardware commit.
