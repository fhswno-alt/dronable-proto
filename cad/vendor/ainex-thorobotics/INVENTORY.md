# AiNex CAD inventory — ainex-thorobotics vendor drop

_Generated: 2026-09-27 evening BST (Europe/London)._

## Requested source

| Item | Result |
|---|---|
| URL | `https://github.com/thorobotics/AiNex` |
| HTTP / API | **404 / not accessible** — repo does not exist (or is private and not granted) |
| GitHub user `Thorobotics` | Exists (FRC Team 6166). Public repos: Destination-Deep-Space, Infinite-Recharge, Infinite-Recharge-Rewrite, TestCode2021 — **Java FRC code only; no AiNex** |
| Releases / LFS on requested repo | N/A (repo missing) |

## Actual CAD source used

Nearest public official package with geometry:

| Field | Value |
|---|---|
| Repo | [`Hiwonder/ainex`](https://github.com/Hiwonder/ainex) |
| Commit | `e8fe2a816797cf83054135160df5a82ec3596a69` (2026-02-07 10:18 BST / +0800 push) |
| Path copied | `src/ainex_simulations/ainex_description/` → `ainex_description/` |
| Releases | none |
| Git LFS | not used |
| License (README) | “open-source … educational and research purposes” — **no SPDX license file**; treat as Hiwonder IP / unclear commercial reuse |

Also checked: `Hiwonder-docs/AiNex` (Sphinx docs only, no CAD).

## Best Onshape import candidate

**None — meshes only.**

- **No STEP / IGES / Parasolid / SolidWorks / Fusion / Onshape links** anywhere in `Hiwonder/ainex`.
- Available geometry is **25 binary STL link meshes** + URDF/xacro kinematics (parts / per-link meshes, **not** a single full assembly B-rep).
- Onshape can import STL as mesh parts (limited editing). Prefer official STEP from Hiwonder support (`support@hiwonder.com` + order number) when available — see `docs/FUSION_ASSEMBLY_PLAN.md`.

### Recommended interim import set (meshes-only)

Absolute paths under this vendor drop:

- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/body_link.STL` — Binary STL, 984.0 KB, ~20150 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/head_pan_link.STL` — Binary STL, 114.5 KB, ~2343 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/head_tilt_link.STL` — Binary STL, 233.6 KB, ~4782 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/l_ank_pitch_link.STL` — Binary STL, 353.5 KB, ~7238 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/l_ank_roll_link.STL` — Binary STL, 45.8 KB, ~936 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/l_el_pitch_link.STL` — Binary STL, 147.3 KB, ~3015 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/l_el_yaw_link.STL` — Binary STL, 249.0 KB, ~5097 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/l_gripper_link.STL` — Binary STL, 96.1 KB, ~1966 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/l_hip_pitch_link.STL` — Binary STL, 235.6 KB, ~4824 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/l_hip_roll_link.STL` — Binary STL, 353.4 KB, ~7236 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/l_hip_yaw_link.STL` — Binary STL, 90.0 KB, ~1842 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/l_knee_link.STL` — Binary STL, 91.6 KB, ~1874 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/l_sho_pitch_link.STL` — Binary STL, 99.3 KB, ~2031 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/l_sho_roll_link.STL` — Binary STL, 313.9 KB, ~6428 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/r_ank_pitch_link.STL` — Binary STL, 352.0 KB, ~7208 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/r_ank_roll_link.STL` — Binary STL, 45.8 KB, ~936 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/r_el_pitch_link.STL` — Binary STL, 151.0 KB, ~3091 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/r_el_yaw_link.STL` — Binary STL, 253.0 KB, ~5180 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/r_gripper_link.STL` — Binary STL, 98.4 KB, ~2013 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/r_hip_pitch_link.STL` — Binary STL, 236.0 KB, ~4831 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/r_hip_roll_link.STL` — Binary STL, 352.0 KB, ~7208 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/r_hip_yaw_link.STL` — Binary STL, 90.0 KB, ~1842 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/r_knee_link.STL` — Binary STL, 93.0 KB, ~1902 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/r_sho_pitch_link.STL` — Binary STL, 101.5 KB, ~2077 tris
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/meshes/r_sho_roll_link.STL` — Binary STL, 317.0 KB, ~6490 tris

Kinematics companion (not CAD, but needed to place joints):

- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/urdf/ainex.urdf.xacro`
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/urdf/ainex.xacro`
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/urdf/gazebo.xacro`
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/urdf/materials.xacro`
- `/workspace/dronable-proto/cad/vendor/ainex-thorobotics/ainex_description/urdf/transmissions.xacro`

## Full CAD file inventory

### Classification

| Class | Present? |
|---|---|
| Full STEP/IGES assembly | **No** |
| Native SolidWorks / Fusion / Parasolid / Onshape | **No** |
| Per-link meshes (STL) | **Yes** (25) |
| URDF/xacro | **Yes** (5) |

### Meshes (`ainex_description/meshes/`)

| File | Format | Size | Triangles (binary header) | Role |
|---|---|---:|---:|---|
| `body_link.STL` | Binary STL | 984.0 KB (1,007,584) | 20150 | torso |
| `head_pan_link.STL` | Binary STL | 114.5 KB (117,234) | 2343 | head |
| `head_tilt_link.STL` | Binary STL | 233.6 KB (239,184) | 4782 | head |
| `l_ank_pitch_link.STL` | Binary STL | 353.5 KB (361,984) | 7238 | link part |
| `l_ank_roll_link.STL` | Binary STL | 45.8 KB (46,884) | 936 | link part |
| `l_el_pitch_link.STL` | Binary STL | 147.3 KB (150,834) | 3015 | link part |
| `l_el_yaw_link.STL` | Binary STL | 249.0 KB (254,934) | 5097 | link part |
| `l_gripper_link.STL` | Binary STL | 96.1 KB (98,384) | 1966 | gripper |
| `l_hip_pitch_link.STL` | Binary STL | 235.6 KB (241,284) | 4824 | link part |
| `l_hip_roll_link.STL` | Binary STL | 353.4 KB (361,884) | 7236 | link part |
| `l_hip_yaw_link.STL` | Binary STL | 90.0 KB (92,184) | 1842 | link part |
| `l_knee_link.STL` | Binary STL | 91.6 KB (93,784) | 1874 | link part |
| `l_sho_pitch_link.STL` | Binary STL | 99.3 KB (101,634) | 2031 | link part |
| `l_sho_roll_link.STL` | Binary STL | 313.9 KB (321,484) | 6428 | link part |
| `r_ank_pitch_link.STL` | Binary STL | 352.0 KB (360,484) | 7208 | link part |
| `r_ank_roll_link.STL` | Binary STL | 45.8 KB (46,884) | 936 | link part |
| `r_el_pitch_link.STL` | Binary STL | 151.0 KB (154,634) | 3091 | link part |
| `r_el_yaw_link.STL` | Binary STL | 253.0 KB (259,084) | 5180 | link part |
| `r_gripper_link.STL` | Binary STL | 98.4 KB (100,734) | 2013 | gripper |
| `r_hip_pitch_link.STL` | Binary STL | 236.0 KB (241,634) | 4831 | link part |
| `r_hip_roll_link.STL` | Binary STL | 352.0 KB (360,484) | 7208 | link part |
| `r_hip_yaw_link.STL` | Binary STL | 90.0 KB (92,184) | 1842 | link part |
| `r_knee_link.STL` | Binary STL | 93.0 KB (95,184) | 1902 | link part |
| `r_sho_pitch_link.STL` | Binary STL | 101.5 KB (103,934) | 2077 | link part |
| `r_sho_roll_link.STL` | Binary STL | 317.0 KB (324,584) | 6490 | link part |
| **Total** | | **5.4 MB (5,629,100)** | | meshes-only |

### URDF / xacro (`ainex_description/urdf/`)

| File | Size | Notes |
|---|---:|---|
| `ainex.urdf.xacro` | 32.7 KB (33,514) | kinematics / materials / gazebo |
| `ainex.xacro` | 265 B (265) | kinematics / materials / gazebo |
| `gazebo.xacro` | 6.7 KB (6,818) | kinematics / materials / gazebo |
| `materials.xacro` | 717 B (717) | kinematics / materials / gazebo |
| `transmissions.xacro` | 1.8 KB (1,832) | kinematics / materials / gazebo |

### Other package files (not CAD)

- `ainex_description/CMakeLists.txt` — 281 B
- `ainex_description/launch/.gazebo.launch.swo` — 12.0 KB
- `ainex_description/launch/display.launch` — 529 B
- `ainex_description/launch/gazebo.launch` — 867 B
- `ainex_description/package.xml` — 625 B
- `ainex_description/rviz/urdf.rviz` — 10.0 KB

## Blockers

1. **Requested repo missing:** `thorobotics/AiNex` does not exist; cannot clone.
2. **No STEP assembly:** only STL meshes — weak Onshape import (mesh, not B-rep); no mates preserved.
3. **License ambiguity:** README says educational/research; no SPDX; commercial / redistribution risk.
4. **Official STEP still order-gated** per Hiwonder product FAQ (order number → support).
5. **Not too big / LFS OK:** meshes ~5.5 MB total, no LFS, all blobs present.

## Provenance note

Vendor folder name `ainex-thorobotics` kept per task path; content is from **Hiwonder/ainex**, not Thorobotics. See `SOURCE.txt`.
