# Ainex M2 145 Isaac asset

This directory is a text USD stage for a later Isaac Sim import. It was **not** loaded in Isaac Sim. No simulator was launched. The walk was **not** proven here.

## Source

- Body, inertials, joints, mesh files, foot boxes, toe visuals, and position actuators: `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` (read-only).
- Foot size, foot friction, and actuator kp / forcerange were cross-checked against `ainex_controls_m2_145_gate_f_optc.xml`. Shared links, inertials, joint attributes, foot geoms, and position actuators match. Optc-only pieces that were not ported: door bodies (`door_prop`, `door_panel_link`, `door_lever_link`) and gripper geoms `l_hand_contact` / `r_hand_contact`.
- `kit_cam` is **not** in the walk file. Pose, `xyaxes`, and `fovy` were copied from the optc camera, which matches the frozen spec.
- Mesh files are the MJCF paths under `mujoco/ainex_hiwonder/meshes/` (those `.STL` names are symlinks to the vendor meshes). Relative path from this USDA: `../../mujoco/ainex_hiwonder/meshes/<link>.STL`.
- MuJoCo XML was not modified.

## What was copied

- 25 links. Each is an `Xform` with `PhysicsRigidBodyAPI` and `PhysicsMassAPI` (mass, center of mass, diagonal inertia, principal axes). Masses were not invented. Total mass 2.3474984 kg.
- 25 visual meshes. Each is a USD `references` arc plus `mujoco:stl` asset path to the existing STL. Meshes have **no** collision API (`contype 0` in the MJCF).
- 24 `PhysicsRevoluteJoint` drives. The MuJoCo free joint `root` on `body_link` is **not** a revolute (1 free joint). `PhysicsArticulationRootAPI` is on `/ainex`. `mujoco:fixBase` is false. A GPU load must not weld the base.
- 2 foot colliders and 2 toe visuals.
- One `PhysicsScene`, gravity (0, 0, -9.81), matching the MJCF `option`. The infinite floor plane was **not** ported.
- Meters. Up axis Z, matching this MuJoCo model.

## Foot collision

Copied, not resized.

- MuJoCo box half-size `0.0725 0.0430 0.008` m, position `0.030 0.0 -0.018` m, on `l_ank_roll_link` and `r_ank_roll_link`. Authored as `mujoco:halfSize = [0.0725, 0.043, 0.008]` and `xformOp:translate = (0.03, 0, -0.018)`.
- Full size written on the UsdGeom cube (`size = 1`, `xformOp:scale = (0.145, 0.086, 0.016)`) on both feet. That is 145 mm by 86 mm by 16 mm (thickness is `2 * 0.008`).
- MuJoCo friction `1.6 0.1 0.01`, condim 3. Only the sliding coefficient is active. Authored `physics:staticFriction = 1.6` and `physics:dynamicFriction = 1.6`. Torsional 0.1 and rolling 0.01 are on `mujoco:friction` and are not enabled.
- The foot STL is not a collider. Toe spheres (`l_toe_viz`, `r_toe_viz`, radius 0.012 m) have no collision API.

## Joints and drives

USD revolute limits are **degrees**. ±2.09 rad is written as ±119.74817918 degrees. The radian values are also on `mujoco:rangeRad`.

Drive `type` is `force`, so max force is a torque.

| Group | Joints | MuJoCo forcerange | `drive:angular:physics:maxForce` | kp (N·m/rad) | USD stiffness (N·m/deg) | Joint damping (N·m·s/rad) |
| --- | --- | --- | --- | --- | --- | --- |
| Leg (hip yaw/roll) | 4 | ±2.1 | 2.1 | 40 | 0.69813170 | 0.08 |
| Leg (hip pitch, knee) | 4 | ±2.1 | 2.1 | 45 | 0.78539816 | 0.08 |
| Leg (ankle pitch/roll) | 4 | ±2.1 | 2.1 | 35 | 0.61086524 | 0.08 |
| Arm, including grippers | 10 | ±0.7 | 0.7 | 20 | 0.34906585 | 0.05 |
| Head pan/tilt | 2 | ±0.7 | 0.7 | 15 | 0.26179939 | 0.05 |

- All 24 revolute joints: range ±2.09 rad, armature 0.01 (`physxJoint:armature` and `mujoco:armature`).
- Arm joints, including grippers: MuJoCo `frictionloss` 0.2, written to `physxJoint:jointFriction` and `mujoco:frictionloss`. Legs and head are 0.
- Joint axis sign is preserved. A negative MuJoCo axis is a positive USD X/Y/Z axis with a 180° joint-frame rotation, so the positive drive direction matches MuJoCo.
- `r_el_yaw` and `l_el_yaw` do not set `axis` in the walk file. The MJCF hinge default `0 0 1` was used.
- Position actuators also set `dampratio=1`. That is stored as `mujoco:dampratio`. It was **not** turned into a numeric kv. MuJoCo computes `kv = dampratio * 2 * sqrt(kp * M0)` from the compiled joint inertia `M0`, which is not a number in the XML and was not evaluated. Drive damping is only the passive joint damping, converted from N·m·s/rad to N·m·s/deg (`damping * pi/180`).
- `physxJoint:jointFriction` carries the MuJoCo frictionloss torque (N·m). This file does not claim PhysX treats that attribute as a torque. The source number is also on `mujoco:frictionloss`.

## Camera

- Prim `/ainex/body_link/head_pan_link/head_tilt_link/kit_cam`.
- Local translate `0.02 0.019 0.007` m on `head_tilt_link`.
- Looks along +X (MuJoCo `xyaxes="0 -1 0 0 0 1"`, camera -Z). Up is +Z. No extra mass.
- `mujoco:fovy` = 104.82 degrees. Encoded as focalLength 10 mm, verticalAperture 25.97990799 mm. Recomputed fovy 104.820000 degrees.
- horizontalAperture 34.63987732 mm uses the MJCF visual buffer 960×720. That aspect is not a camera attribute.
- `clippingRange` (0.01, 100) is a USD stand-in. The MuJoCo camera does not set near/far.

## Parse

Opened with usd-core (`Usd.Stage.Open`). Rigid bodies 25, revolute joints 24. This was a schema/text parse, not an Isaac Sim or PhysX load.

Core USD cannot compose an STL as a layer. Each visual mesh `references` the relative STL, so opening the stage reports one "Cannot determine file format" warning per mesh. The same relative path is on `mujoco:stl`. Geometry is not inlined.

## Not claimed

- Not loaded in Isaac Sim. Not stepped. The walk was not proven.
- Principal axes were copied as MuJoCo `(w, x, y, z)` into USD `quatf` `(real, i, j, k)`. They were not conjugated. That convention was not checked in Isaac.

## Remaining step

GPU load on an RTX 4080-class machine (16 GB VRAM, 32 GB RAM): import this USDA in Isaac Sim, resolve the STL references, and leave the articulation base free (`fixBase` false). Do not treat that load as proof of the walk.
