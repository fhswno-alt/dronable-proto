# AiNex kit-matched MuJoCo (from Hiwonder URDF)

**Date:** Sun 27 Sep 2026 (Europe/London / BST)  
**Path A buy:** frozen — this is simulation only; no spend.  
**Design review:** Mon 29 Sep 2026.

## Source

| Field | Value |
|---|---|
| Requested | `thorobotics/AiNex` — **404** (see vendor `SOURCE.txt`) |
| Actual | [`Hiwonder/ainex`](https://github.com/Hiwonder/ainex) commit **`e8fe2a816797cf83054135160df5a82ec3596a69`** |
| Vendor drop | `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/` |
| Geometry | 25 binary STL link meshes (already **metres**) + URDF/xacro kinematics |
| No STEP | meshes-only; Onshape import skipped per Dave |

## Working MJCF

| Asset | Path |
|---|---|
| **Load this (geometry)** | `/workspace/dronable-proto/mujoco/ainex_hiwonder/ainex.xml` |
| **Load this (dynamics / stand / walk)** | `/workspace/dronable-proto/mujoco/ainex_hiwonder/ainex_controls.xml` |
| Flat URDF (resolved xacro) | `/workspace/dronable-proto/mujoco/ainex_hiwonder/ainex.urdf` |
| Raw MuJoCo compile dump | `/workspace/dronable-proto/mujoco/ainex_hiwonder/ainex_raw.xml` |
| Mesh dir (symlinks → vendor) | `/workspace/dronable-proto/mujoco/ainex_hiwonder/meshes/*.STL` |
| Smoke stats | `/workspace/dronable-proto/mujoco/ainex_hiwonder/smoke_stats.json` |
| Smoke / stand PNGs | `smoke_preview.png`, `stand_first_frame.png`, `stand_side.png`, `stand_held.png`, `stand_walk_progress.png` |
| Stand→step script | `/workspace/dronable-proto/scripts/ainex_stand_then_step.py` |
| Converter | `/workspace/dronable-proto/scripts/convert_ainex_urdf_to_mjcf.py` |

**Not replaced:** `/workspace/dronable-proto/mujoco/dronable_v0.xml` (earlier approx capsule/box walking core) remains for gait experiments. Prefer `ainex_hiwonder/ainex.xml` for kit-matched geometry.

## What matched (kit)

- **25 Hiwonder link meshes** referenced 1:1 (body, head pan/tilt, L/R legs 6+6, L/R arms sho/el/gripper).
- **Kinematics** from `ainex.urdf.xacro` (joint origins, axes, ±2.09 rad limits).
- **Per-link mass + full inertia tensors** from the same URDF (CAD-derived numbers in the Hiwonder package).
- **24 revolute DOF** + floating base:
  - Legs: `l/r_hip_yaw|roll|pitch`, `l/r_knee`, `l/r_ank_pitch|roll` (12)
  - Arms: `l/r_sho_pitch|roll`, `l/r_el_pitch|yaw`, `l/r_gripper` (10)
  - Head: `head_pan`, `head_tilt` (2)
- **24 position actuators** (`*_pos`) with URDF effort band ±6 Nm and ctrlrange ±2.09 rad.
- Units: **metres / kg / rad**; STL bbox confirms meshes are already SI (e.g. body ~106×100×99 mm).
- Floating base height `root_z ≈ 0.268 m` so zero-pose foot mesh sits ~2 mm above floor.

## Smoke-test results (2026-09-27 evening BST)

```
nq=31  nv=30  nu=24  njnt=25 (1 free + 24 hinge)
nbody=26  ngeom=26  nmesh=25
total_mass ≈ 2.347 kg   body_link = 0.743 kg
100× mj_step: OK, no NaNs
```

## How to launch / view

```bash
cd /workspace/dronable-proto
source .venv/bin/activate

# Headless load check
python -c "import mujoco; m=mujoco.MjModel.from_xml_path('mujoco/ainex_hiwonder/ainex.xml'); print(m.nq,m.nu,m.nmesh)"

# Interactive viewer (needs display)
python -m mujoco.viewer --mjcf mujoco/ainex_hiwonder/ainex.xml

# Regenerate from vendor xacro (idempotent)
python scripts/convert_ainex_urdf_to_mjcf.py
```

Actuator ctrl indices follow `smoke_stats.json` → `act_names` (same order as `REVOLUTE_JOINTS` in the converter: L-leg, R-leg, L-arm, R-arm, head).

## ASSUMPTIONS / remaining gaps vs AiNex Standard kit

1. **SKU:** Hiwonder public URDF is the sim package shipping with their ROS stack; marketed as AiNex-class (~415 mm, ~2.45 kg, 24 DOF + hands). **Standard vs Starter** differences (servo grade, battery, extras) are **not** encoded in the meshes — treat as Standard-class envelope until a kit BOM is locked.
2. **Mass:** URDF sum **2.35 kg** vs marketing **~2.45 kg**. Likely missing battery / harness / Pi mass; not a weighed unit.
3. **Inertias:** Present and non-diagonal in URDF (converted to MuJoCo `diaginertia` + quat). Still **CAD estimates**, not measured.
4. **Actuator limits:** URDF `effort=6` Nm / `velocity=100` are Gazebo placeholders. Real HX-35H / HX-12H stall is lower (~3.5 / ~1.8 Nm class); v0 Controls clip used stall×0.6. **Do not trust ±6 Nm for fall/torque studies** without datasheet confirmation.
5. **PD / home pose:** Converter adds generic `kp=30` position actuators; not the Controls 50 Hz gait PD from `dronable_v0`. Zero pose is URDF zero (arms out), not a standing ready pose.
6. **Collision:** Each link uses the **full STL as the collision mesh** (heavy; no convex decomposition). Self-collision may be noisy.
7. **Sensors:** `camera_link` / `imu_link` exist as fixed frames in URDF; not yet exposed as MuJoCo cameras/sites in `ainex.xml` (unlike `dronable_v0` kit_cam).
8. **No official STEP** — geometry is kit-matched **meshes**, not editable B-rep. Path A purchase still frozen.
9. **License:** Hiwonder README “educational/research”; no SPDX — commercial reuse unclear.

## Related

- Vendor inventory: `cad/vendor/ainex-thorobotics/INVENTORY.md`
- Product assumptions (gait PD, budget): `/workspace/dronable-proto/ASSUMPTIONS.md`
- Approx walking core (unchanged): `mujoco/dronable_v0.xml`

## Stand pose + first walking progress (2026-09-27 evening BST)

Prefer **`ainex_controls.xml`** over bare `ainex.xml` for dynamics work:

| Asset | Path |
|---|---|
| **Controls model (preferred)** | `/workspace/dronable-proto/mujoco/ainex_hiwonder/ainex_controls.xml` |
| Stand / step script | `/workspace/dronable-proto/scripts/ainex_stand_then_step.py` |
| Full CPG attempt (same model) | `/workspace/dronable-proto/scripts/walk_gait_ainex.py` |
| Progress note | `/workspace/dronable-proto/mujoco/ainex_hiwonder/stand_then_step_note.txt` |

`ainex_controls.xml` = Hardware `ainex.xml` + HX torque clips (±2.1 Nm legs / ±0.7 arm-head), mesh collision off, foot contact boxes, L=orange / R=green shin-foot rgba. Path A buy still frozen; NOT OEM STEP.

### Home / stand joint targets

Defined in `scripts/ainex_stand_then_step.py` → `stand_targets()`:

- Slight knee bend: flex `0.32` rad (L knee `+`, R knee `-` — axes mirrored)
- Hip forward bias `0.10` rad in signed-fwd space → L hip_pitch `-0.10`, R `+0.10`
- Ankle flat-foot identity: `ank = hip + knee`
- Arms down: `|sho_roll|=1.45`, elbow pitch `0.40`
- Slight hip abduct: L roll `-0.05`, R `+0.05`
- Free-joint height `COM_Z ≈ 0.238 m` so foot boxes plant near floor

Axis note (differs from `dronable_v0`): forward = **+X**; hip pitch axes L(+Y)/R(−Y); map via `to_joint()`.

### Headless renders (MUJOCO_GL=egl)

| PNG | Meaning |
|---|---|
| `mujoco/ainex_hiwonder/stand_first_frame.png` | Kinematic stand, 3/4 view, full robot lit |
| `mujoco/ainex_hiwonder/stand_side.png` | Side view (+X forward) |
| `mujoco/ainex_hiwonder/stand_held.png` | After **2.5 s PD hold under gravity, assist OFF** |
| `mujoco/ainex_hiwonder/stand_walk_progress.png` | After stand + open-loop step cue (assist ON) |

```bash
cd /workspace/dronable-proto && source .venv/bin/activate
MUJOCO_GL=egl python scripts/ainex_stand_then_step.py
# honest tip without assist on the step phase:
MUJOCO_GL=egl python scripts/ainex_stand_then_step.py --no-assist
```

### Honest status toward walk

| Checkpoint | Result |
|---|---|
| Load 25/25 meshes | **PASS** (both `ainex.xml` and `ainex_controls.xml`) |
| First standing render | **PASS** — T-pose fixed; arms down, slight crouch, L orange / R green |
| PD stand hold 2.5 s, no assist | **PASS** — `body_z≈0.235`, `up_z≈1.0`, did not tip |
| Open-loop step cue | **PARTIAL** — hips/knees articulate (`ptp≈0.5`), foot-lead flips; with soft assist stays upright and translates ~+0.25 m / 2 s (**likely includes skate**); **without assist, tips** |
| Continuous walking | **NOT YET** — ankle CoP tried; still tip ~2.3 s / skate; freeze ablation helps stand only |

**Concrete next step (DONE 2026-09-27 ~20:19 BST):** ankle CoP @ 50 Hz + stand disturb + freeze ablation — **clean walk still FAIL**; freeze recovers tip-threshold stand pulse only (see ankle CoP section).

**True blockers (sim):** none for load/stand. Walking blockers are control (open-loop + joint servos tip ~2.3 s under HX ±2.1 Nm) and contact (stance skate; boxes only; full STL collision off). Geometry is kit-matched meshes, not OEM STEP — Path A spend remains frozen.

## Walk gait attempt (WORLD_FIXED) — 2026-09-27 ~20:09 BST

| Asset | Path |
|---|---|
| Script | `/workspace/dronable-proto/scripts/walk_gait_ainex.py` |
| Best still (assist OFF tip) | `mujoco/ainex_hiwonder/walk_attempt.png` |
| Assist ref still | `mujoco/ainex_hiwonder/walk_attempt_assist_ref.png` |
| Honest note | `mujoco/ainex_hiwonder/walk_gait_note.txt` |
| Stats | `mujoco/ainex_hiwonder/walk_gait_stats.json` |
| MP4 (no assist) | `previews/ainex_walk/ainex_walk.mp4` |
| MP4 (assist compare) | `previews/ainex_walk/ainex_walk_assist.mp4` |

### Control levers added

1. **Lateral COM shift before swing** — hip_roll common-mode amp `0.20` rad with phase lead `0.18`; sign verified (lat=+1 → body closer to L foot).
2. **Stance plant damper** — oppose stance-foot horizontal `cvel` while in contact (`kd=45` N/(m/s), clip 18 N). External root wrench; disclosed; not friction-limited contact.
3. **Joint-space roll + pitch servos** from body lean (no free-joint quat lock).
4. **HX torque clips unchanged** (±2.1 Nm legs / ±0.7 arm-head).
5. Default run: **plant ON, balance assist OFF** (no soft-pass). `--assist` only for comparison.

### WORLD_FIXED acceptance gates (measurable)

| Gate | Threshold |
|---|---|
| Gait window | ≥ 4.0 s after stand+ramp |
| `upright_frac` (`up_z>0.85` & `body_z>0.18`) | ≥ 0.90 |
| Contact duty each foot | ∈ [0.20, 0.85] |
| Foot-lead cycles | ≥ 3 |
| Hip / knee ptp | ≥ 0.18 rad; hip L/R corr ≤ −0.45 |
| Forward progress `dx` | ≥ +0.05 m |
| Avg speed | ≤ 0.22 m/s |
| Stance foot |vx| | mean ≤ 0.08 m/s; p95 ≤ 0.18 m/s |
| Balance assist | **must be OFF** for clean PASS |

```bash
cd /workspace/dronable-proto && source .venv/bin/activate
MUJOCO_GL=egl python scripts/walk_gait_ainex.py --duration 7
# comparison only (NOT a clean-walk claim):
MUJOCO_GL=egl python scripts/walk_gait_ainex.py --duration 7 --assist
```

### Results (honest)

| Trial | Result |
|---|---|
| Stand hold (gait-off, assist OFF) | **PASS** — upright_frac=1.0, both feet contact, `body_z≈0.235` |
| Plant + COM + joint servos, **assist OFF** | **FAIL** — tips at **t≈2.27 s** (`z<0.08`, flipped); `dx=-0.24 m`; stance |vx| mean ≈0.24–0.27 m/s (skate); foot_lead_cycles=2 |
| Same CPG + plant, **assist ON** | **FAIL clean walk** — upright 5–7 s, articulates (hip_corr≈−1, lead_cycles≥3) but contact_duty≈0.08–0.10 (height assist floats feet), stance skate ≈0.27–0.39 m/s, `dx≤0`; assist ON auto-fails gate |

**clean_walk_claim: false.** Mesh can articulate under assist; open-loop contact does **not** produce a defensible walk.

### Next control lever (DONE 2026-09-27 ~20:19 BST) — ankle CoP

Implemented in `scripts/walk_gait_ainex.py` (default ON; `--no-ankle-cop` / `--disturb-stand` / `--stance-freeze`):

1. **Ankle CoP / balance servo @ 50 Hz** — lean-primary (`up_x`/`up_y`) + relative COM-xy vs support-foot xy after settle bias; HX clips on; assist OFF.
2. **Stand disturbance** — Fy=+3.8 N × 0.60 s at t=1.0 s (tip-threshold).
3. **Stance-foot world freeze ablation** — sticky xy spring-damper on foot bodies (external `xfrc`; disclosed).

| Trial | Result |
|---|---|
| Quiet stand + CoP, assist OFF | **PASS** — uf=1.0, `body_z≈0.235` |
| Disturb, **ankle CoP only** | **FAIL** — tips ~2.06 s (does not expand RoA enough under HX ±2.1) |
| Disturb, **stance-freeze ablation** | **PASS** — post_uf=1.0, recover_t≈0.40 s (same pulse) |
| Walk plant+CPG+CoP, assist OFF | **FAIL** — tips ~2.29 s; skate; `dx<0`; WORLD_FIXED gates fail |

**clean_walk_claim: still false.** Artifacts: `ankle_cop_*.png`, `ankle_cop_note.txt`, `ankle_cop_stats.json`.

```bash
MUJOCO_GL=egl python scripts/walk_gait_ainex.py --disturb-stand
MUJOCO_GL=egl python scripts/walk_gait_ainex.py --disturb-stand --stance-freeze --no-ankle-cop
MUJOCO_GL=egl python scripts/walk_gait_ainex.py --duration 7
```

**Next:** capture-point / ZMP with swing placement; real no-slip contact (freeze stays ablation). Path A buy still frozen.
