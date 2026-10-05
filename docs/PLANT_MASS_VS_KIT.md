# Plant mass / inertia vs Hiwonder AiNex kit

**Prefer FAIL:** sim body matches kit within 15%, so hip rail is gait/Controls.

Docs only. No plant XML, `forcerange`, damping, feet, `kit_cam`, gait, or CommandBus edit. Both plant md5s below are the files as they already are.

Measured with MuJoCo **3.14.0** by `scripts/plant_mass_vs_kit.py` (JSON on stdout). The script checks the vendored URDF blob and loads the thaw tip with `git show` plus a mesh symlink. It does not write either plant.

## Plants

| | Path | md5 |
|---|---|---|
| Main (this branch's base) | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` | `71b2c86d133ebc603f58b99c53e496f3` |
| Thaw tip, PR #43, `cursor/plant-thaw-legs-foot-6f10` @ `b711472f309a9d0c3e042d60594654cbb00a984a` | same relative path | `17dc4ff37491c8e61900fd83b5d31f0c` |

The MJCF has no `<include>`. Each of the 25 robot bodies carries an explicit `<inertial>`. Mesh files are visual. A body with an explicit inertial does not pick up extra mass from the foot box. The 25 `<inertial>` elements are byte-identical on the two plants. Compiled `body_mass` differs by **0**. The thaw diff is leg `actuatorfrcrange` / `forcerange` (±2.45 N·m vs ±2.1 N·m) and the foot-box half-size (135×76 vs 145×86). The contact-box `pos` is the same, so the CoP used below does not move.

## Kit source

Pinned checkout: [`Hiwonder/ainex`](https://github.com/Hiwonder/ainex) **HEAD `e8fe2a816797cf83054135160df5a82ec3596a69`** (default branch as of 2026-10-05). The description files below were last touched in **`b30afce4076d1f2bc5acf52044888ee1c42db585`** (2026-01-24, "Initial commit") and are unchanged at that HEAD.

| What | Path in `Hiwonder/ainex` | Git blob at HEAD |
|---|---|---|
| Link mass, COM, inertia, joint dynamics | `src/ainex_simulations/ainex_description/urdf/ainex.urdf.xacro` | `e0c4b4301ac1d9d574e7ef73033cc41f5eca1863` |
| Gait presets | `src/ainex_driver/ainex_kinematics/src/ainex_kinematics/gait_manager.py` | `c07bf9722c5c43cf362572498f148616600696a6` |
| Walking defaults | `src/ainex_driver/ainex_kinematics/config/walking_param.yaml` | `106db7619f167821b32fb1c3fa490f197c6a90f2` |
| Arm/head pose used here | `src/ainex_driver/ainex_kinematics/config/init_pose.yaml` | `1f17d57b03788ef1f249dbe8139f12caf6f1a4f7` |

`git hash-object` on `cad/vendor/ainex-thorobotics/ainex_description/urdf/ainex.urdf.xacro` is the same blob `e0c4b430…`. The script reads that vendor file. `diff` against the clone of HEAD was empty.

Published total mass is **not** in that repo. The product page [hiwonder.com/products/ainex](https://www.hiwonder.com/products/ainex) (fetched 2026-10-05) says **Product weight: 2.25 kg (Starter); 2.45 kg (Standard)**. The URDF has no separate total. The sum of its 25 link masses is **2.347498 kg**.

No second per-link weighed table was found. The per-link "kit" column is the URDF. The 2.45 kg figure is the Standard product weight only.

HX-35H rotor inertia: the product page [hiwonder.com/products/hx-35h](https://www.hiwonder.com/products/hx-35h) (fetched 2026-10-05) lists 52 g, 25 kg·cm rotation torque, 35 kg·cm static torque, 0.18 s/60° at 11.1 V. It does not list a gear ratio or a rotor inertia. [servodatabase.com/servo/hiwonder/hx35h](https://servodatabase.com/servo/hiwonder/hx35h) leaves gear ratio blank. Robotis XM430-W350 (similar stall class, different servo) publishes a gear ratio of 353.5:1 and links a moment-of-inertia PDF; that download returned 404 on 2026-10-05, so no reflected inertia `J_rotor × N²` is cited here.

## Method

1. Compiled fields: `body_mass`, `body_ipos` (COM in the link frame), `body_inertia`, `body_iquat`. The body-frame inertia is `R diag(I) Rᵀ`. Principal moments are eigenvalues sorted descending. Kit principals are the eigenvalues of the URDF inertia matrix, sorted the same way.
2. A leg link is flagged when its mass is more than 15% above the URDF, or any principal moment is more than 15% above the URDF. Lighter is reported as a signed percent and is not a flag.
3. Total mass is the 25 robot bodies. The floor plane is not included. Leg mass is both legs: hip yaw, hip roll, hip pitch (thigh), knee (shank), ankle pitch, ankle roll (foot).
4. Static hip torque is single support on the **right** foot. CoP is the center of `r_foot_contact`. Ground reaction is `(0, 0, M g)` with `g = 9.81` from the plant and `M` the full robot mass. No horizontal force. Hip torque is that wrench plus the weight of the distal links, taken about the joint anchor and projected on the joint axis. Hip roll's distal set starts at `r_hip_roll_link`. Hip pitch's distal set starts at `r_hip_pitch_link` (the thigh).
5. Arms and head are the Hiwonder `init_pose.yaml` angles. Legs are a symmetric crouch that drops the body origin **0.015 m** from the straight-leg sole, feet level, foot XY held on the straight-leg foot XY. Achieved drop **0.014994 m**. Solved sagittal angles: forward **0.395 rad**, knee flex **0.825 rad** (`r_hip_pitch = +0.395`, `r_knee = −0.825`, `r_ank_pitch = −0.431`; left side mirrored). `walking_param.yaml` uses `init_z_offset: 0.025`; `0.015` is the bottom of `body_height_range` in `gait_manager.py` and the value in this hand-off.
6. Lateral sway of 2 cm or 4 cm moves the pelvis toward the stance foot. Stance hip roll and ankle roll change until the foot's body-frame Y has closed that gap, with the foot kept level. Achieved gap **0.01999 m** and **0.04001 m**. Hip-roll amplitudes **0.1172 rad** and **0.2362 rad**.
7. A 2 cm step moves the right foot **+0.02 m** in X from the crouch, sole height held, foot level. Hip pitch changes by **0.100 rad**. Knee changes by **0.0345 rad**. Pitch acceleration uses the hip-pitch change.
8. Sinusoid over the full gait period: `θ(t) = A sin(2π t / T)`, `α_peak = A (2π / T)²`, `τ_arm = 0.01 × α_peak`. `A` is the peak angle for that linear amplitude (one cycle per period). The gait parameter is taken as the peak, not peak-to-peak.
9. Left reflected inertia matches the right to about `1×10⁻⁸ kg·m²`. The torque table is the right leg. Both plants share mass, armature, damping, and frictionloss, so one torque table covers both.

`gait_manager.py` presets (`dsp_ratio`, lines 21–24): period, double-support ratio, `y_swap_amplitude`.

| Period | Double support | Lateral sway |
|---|---|---|
| 300 ms | 0.2 | 0.02 m |
| 400 ms | 0.2 | 0.02 m |
| 500 ms | 0.2 | 0.02 m |
| 600 ms | 0.1 | 0.04 m |

`x_move_amplitude` is capped at 0.02 m. Step height default in the walking yaml is 0.02 m. The grid below also shows 4 cm sway at 300–500 ms, which those presets do not use.

## Mass

Compiled total **2.347498 kg** on both plants (25 bodies). URDF sum **2.347498 kg**. Residual between the two sums is **2.1×10⁻⁸ kg** (float). Leg mass **0.982 kg**. Both legs are **41.83%** of the robot. One leg is **0.491 kg** (**20.92%**).

Against the Standard product weight 2.45 kg the URDF/plant total is **−0.103 kg (−4.18%)**. That gap is under 15% and it is not located in any link: every link matches the URDF. The product page names an 11.1 V 3500 mAh 5C LiPo and does not give that battery's mass, so this audit does not assign the 0.103 kg to a part.

Grouped, both sides, sim = kit:

| Role | Links | Mass (kg) |
|---|---|---|
| Torso | `body_link` | 0.743 |
| Hips | `*_hip_yaw_link` + `*_hip_roll_link` | 0.304 |
| Thighs | `*_hip_pitch_link` | 0.213 |
| Shanks | `*_knee_link` | 0.090 |
| Ankles | `*_ank_pitch_link` | 0.240 |
| Feet | `*_ank_roll_link` | 0.135 |

Per link. COM is the link-frame inertial origin in metres. Principals are kg·m², sorted descending. Sim and kit masses match at the URDF decimals (Δ% = 0). Largest |ΔCOM| on any component of any of the 25 links is **4.7×10⁻⁸ m**. Largest absolute body-frame tensor-element error on any link is **4.0×10⁻⁹ kg·m²** (last-digit rounding of `diaginertia`). Largest principal-moment percent difference is under **0.001%**. **No leg link is flagged.**

| Link | Role | Mass (kg) | COM x, y, z (m) | Principals I1, I2, I3 (kg·m²) | Δ mass | Δ inertia |
|---|---|---|---|---|---|---|
| `body_link` | torso | 0.743 | 0.003943, −0.000079, 0.050455 | 1.235×10⁻³, 1.067×10⁻³, 1.044×10⁻³ | 0% | <0.001% |
| `l_hip_yaw_link` | hip | 0.032 | −0.000031, −0.001257, −0.001371 | 2.278×10⁻⁵, 1.755×10⁻⁵, 8.977×10⁻⁶ | 0% | <0.001% |
| `r_hip_yaw_link` | hip | 0.032 | −0.000031, 0.001257, −0.001371 | 2.278×10⁻⁵, 1.755×10⁻⁵, 8.977×10⁻⁶ | 0% | <0.001% |
| `l_hip_roll_link` | hip | 0.120 | 0.001362, 0.000567, −0.013419 | 7.312×10⁻⁵, 6.051×10⁻⁵, 2.916×10⁻⁵ | 0% | <0.001% |
| `r_hip_roll_link` | hip | 0.120 | 0.001362, −0.000567, −0.013419 | 7.312×10⁻⁵, 6.051×10⁻⁵, 2.916×10⁻⁵ | 0% | <0.001% |
| `l_hip_pitch_link` | thigh | 0.1065 | 0.013594, −0.019381, −0.072705 | 1.129×10⁻⁴, 1.110×10⁻⁴, 4.065×10⁻⁵ | 0% | <0.001% |
| `r_hip_pitch_link` | thigh | 0.1065 | 0.013594, 0.019381, −0.072705 | 1.129×10⁻⁴, 1.110×10⁻⁴, 4.065×10⁻⁵ | 0% | <0.001% |
| `l_knee_link` | shank | 0.045 | 0.003343, −0.019647, −0.042505 | 6.750×10⁻⁵, 5.500×10⁻⁵, 2.056×10⁻⁵ | 0% | <0.001% |
| `r_knee_link` | shank | 0.045 | 0.003343, 0.019647, −0.042505 | 6.750×10⁻⁵, 5.500×10⁻⁵, 2.056×10⁻⁵ | 0% | <0.001% |
| `l_ank_pitch_link` | ankle | 0.120 | −0.017544, −0.019563, 0.013418 | 7.368×10⁻⁵, 6.108×10⁻⁵, 2.916×10⁻⁵ | 0% | <0.001% |
| `r_ank_pitch_link` | ankle | 0.120 | −0.017544, 0.019563, 0.013418 | 7.368×10⁻⁵, 6.108×10⁻⁵, 2.916×10⁻⁵ | 0% | <0.001% |
| `l_ank_roll_link` | foot | 0.0675 | 0.028508, 0.012873, −0.017671 | 1.260×10⁻⁴, 9.220×10⁻⁵, 4.209×10⁻⁵ | 0% | <0.001% |
| `r_ank_roll_link` | foot | 0.0675 | 0.028509, −0.012873, −0.017671 | 1.260×10⁻⁴, 9.220×10⁻⁵, 4.209×10⁻⁵ | 0% | <0.001% |

The other 12 bodies (head, arms, grippers) are in the same 25-link compare. They match to the same tensor tolerance. Head and arm mass is **0.622 kg** (total minus torso minus both legs).

## Static hip torque

Right stance, crouch above. N·m. The same masses make the kit-body FBD the same number. Pitch stays near **0.23 N·m** because the crouch leaves the contact center **11.6 mm** ahead of the hip-pitch anchor; sway is lateral and does not remove that offset.

| Sway | Hip roll | Hip pitch | Roll moment arm (CoP − hip, y) |
|---|---|---|---|
| 0 | 0.012 | 0.229 | 0.05 mm |
| 2 cm | 0.421 | 0.227 | 20.0 mm |
| 4 cm | 0.831 | 0.222 | 40.1 mm |

Distal mass in the roll FBD is **0.459 kg**. Distal mass in the pitch FBD is **0.339 kg**. At 4 cm of sway the static roll torque is about one third of the ±2.45 N·m running rail.

## Armature, damping, frictionloss

All 12 leg hinges, both plants, compiled `dof_armature`, `dof_damping`, `dof_frictionloss`. Left and right match. Thaw matches main.

| Joint | Armature (kg·m²) | Damping (N·m·s/rad) | Frictionloss (N·m) | URDF damping | URDF friction |
|---|---|---|---|---|---|
| hip yaw, hip roll, hip pitch, knee, ankle pitch, ankle roll | 0.01 | 0.08 | 0 | 0.02 | 0 |

URDF values are the xacro properties `damping = 0.02` and `friction = 0.0` on `<dynamics>` of each leg joint (`arm_damping` / `arm_friction` are the arm joints, not the legs). The URDF has no armature. `scripts/convert_ainex_urdf_to_mjcf.py` writes `armature="0.01"` on any hinge that lacks one. That 0.01 is this repo's default, not a Hiwonder rotor inertia. Leg `frictionloss` matches the URDF Coulomb friction of 0. Leg damping on the plant is 0.08; the URDF property is 0.02. Neither value is edited here.

Reflected link inertia is the joint-space mass-matrix diagonal minus armature (`mj_fullM` in MuJoCo 3.14). With every other acceleration at zero, that diagonal is the distal subtree about the joint axis plus armature.

Crouch pose, right leg (left agrees to ~1×10⁻⁸):

| Joint | Armature | Link inertia `M_ii − armature` | Armature / link | `M_ii` |
|---|---|---|---|---|
| Hip roll | 0.01000 | 0.006845 | 1.46 | 0.01685 |
| Hip pitch | 0.01000 | 0.007117 | 1.41 | 0.01712 |

Straight legs, arms still at `init_pose`, the link inertia is 0.00812 (roll) and 0.00820 (pitch). Armature is still larger than the link term (ratio 1.23 and 1.22). At the crouch, armature is **46%** above the hip-roll link inertia and **41%** above the hip-pitch link inertia. The extra term sits on the joint. The link tensors above still match the URDF. No cited HX-35H `J_rotor × N²` was found to replace 0.01, so this audit leaves 0.01 as it is.

Peak armature torque `0.01 × A × (2π / T)²` for the angles above. Static is the table in the previous section. "Static + armature" is those two added. "Static + full `M_ii`" also adds `I_link × α`. Pitch uses the 2 cm step at every row. N·m.

| Period | Sway | Preset? | τ static roll | τ armature roll | τ link roll | Static + armature roll | Static + full M roll | τ armature pitch |
|---|---|---|---|---|---|---|---|---|
| 300 ms | 2 cm | yes | 0.421 | 0.514 | 0.352 | 0.935 | 1.287 | 0.439 |
| 300 ms | 4 cm | no | 0.831 | 1.036 | 0.709 | 1.867 | 2.576 | 0.439 |
| 400 ms | 2 cm | yes | 0.421 | 0.289 | 0.198 | 0.710 | 0.908 | 0.247 |
| 400 ms | 4 cm | no | 0.831 | 0.583 | 0.399 | 1.414 | 1.813 | 0.247 |
| 500 ms | 2 cm | yes | 0.421 | 0.185 | 0.127 | 0.606 | 0.733 | 0.158 |
| 500 ms | 4 cm | no | 0.831 | 0.373 | 0.255 | 1.204 | 1.459 | 0.158 |
| 600 ms | 2 cm | no | 0.421 | 0.129 | 0.088 | 0.549 | 0.637 | 0.110 |
| 600 ms | 4 cm | yes | 0.831 | 0.259 | 0.177 | 1.090 | 1.267 | 0.110 |

On the four preset rows, static plus the full joint inertia stays at or below **1.29 N·m**, and the armature piece alone stays at or below **0.51 N·m**. Both are under the ±2.45 N·m running rail. The one grid cell that crosses 2.45 N·m is **300 ms with 4 cm sway** (2.58 N·m once link inertia is included). `gait_manager.py` pairs 300 ms with **2 cm** of sway, and 4 cm with **600 ms**.

Viscous torque at the same peak speed, `0.08 × A × (2π / T)`, is about **0.20 N·m** on the fastest preset corners (300 ms / 2 cm roll, and 600 ms / 4 cm roll). Frictionloss is 0, so there is no Coulomb term. Adding that viscous piece to the preset full-inertia totals still leaves them under 1.5 N·m.

## Knee and ankle pitch

PR #42 ran the ROBOTIS OP3 `WalkingModule` with `calcInverseKinematicsForLeg` and this plant's leg lengths (plant md5 `17dc4ff37491c8e61900fd83b5d31f0c`). The first rail on that run is the right knee at about 0.42 s, then hip roll, at periods of 300–600 ms. This section is the static knee-pitch and ankle-pitch torque of three poses, plus the knee armature and link inertia for a 2 cm step height. No plant value is changed. The script checks that the foot-contact `pos` and the foot box half-height match on the two plants, so this free-body diagram is the same on both. The comparison limit is the thaw joint actuator range, compiled `jnt_actfrcrange` **±2.45 N·m**. The main plant's compiled range on the same joint is ±2.1 N·m.

`M = 2.347498 kg` and `g = 9.81` give **`Mg = 23.029 N`** and **`Mg/2 = 11.514 N`**. Those are the "about 23 N" and "about 11.5 N" loads below.

Assumptions, all in `scripts/plant_mass_vs_kit.py`:

- Ground reaction is vertical, at the center of that foot's contact box. No horizontal force. Double support puts `Mg/2` on each foot. Single support puts `Mg` on one foot and 0 on the other. The split is equal; the script does not solve an indeterminate contact distribution.
- Knee distal links are the shank, the ankle-pitch link, and the foot (`0.2325 kg`). Ankle-pitch distal links are the ankle-pitch link and the foot (`0.1875 kg`). Torque is that wrench about the joint anchor, projected on the joint axis. Arms stay at `init_pose.yaml` and are outside both distal sets.
- The sinusoid is the same one as the hip section: `θ = A sin(2π t / T)`, `α = A (2π / T)²`, `A` the peak angle of one cycle per period. `τ_arm = 0.01 α`, `τ_link = I_link α`, `I_link = M_ii − 0.01` from `mj_fullM` at that pose. Viscous peak is `0.08 A (2π / T)`.
- The 2 cm is a sole-height rise with hip yaw, hip roll, and hip pitch held. Ankle pitch is solved so the contact box's world Z axis is as vertical as that joint can make it. `A` is the knee change that achieves the rise.

### OP3 initial pose, scaled to AiNex lengths

Checkout [`ROBOTIS-GIT/ROBOTIS-OP3`](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3) **`3bc2bd514e8ee6054e9726d23c8c0b13236f84f0`** (2024-12-13). `op3_walking_module/src/op3_walking_module.cpp` `WalkingModule::initialize` writes an init pose and then calls `loadWalkingParam`, which replaces it from `op3_walking_module/config/param.yaml` (blob `540afd68383f8e9bc3ec262657b3c2a1309571d4`) before `updatePoseParam` copies those fields into the IK. The yaml is the pose the module IK's:

| Field | param.yaml | Unit in the IK |
|---|---|---|
| `x_offset` | −0.020 | m |
| `y_offset` | 0.015 | m |
| `z_offset` | 0.035 | m |
| `roll_offset`, `pitch_offset`, `yaw_offset` | 0 | deg in the file, × `DEGREE2RADIAN` |
| `hip_pitch_offset` | 7 | deg in the file, × `DEGREE2RADIAN` |

`DEGREE2RADIAN` is `(M_PI / 180.0)` in [`ROBOTIS-Math`](https://github.com/ROBOTIS-GIT/ROBOTIS-Math) `robotis_math/include/robotis_math/robotis_math_base.h` at `2413107b1f3bebaae61c24d3eb3bdaaf8bf45234`. The hardcoded block that yaml replaces is `init_x_offset −0.010`, `init_y_offset 0.005`, `init_z_offset 0.020`, `hip_pitch_offset 13°`. `init_position_` for the twelve leg joints is 0, and the IK angle is added to that.

The live IK is `OP3KinematicsDynamics::calcInverseKinematicsForLeg` in `op3_kinematics_dynamics/src/op3_kinematics_dynamics.cpp`. OP3 lengths there are thigh `hypot(0.0001, 0.11015) = 0.110150 m`, calf `0.110 m`, ankle `0.0305 m` (leg `0.250650 m`). Scaling keeps the metre offsets and substitutes the plant chain, the same three reads as the PR #42 port: thigh `0.096887 m`, calf `0.089077 m`, sole drop `0.026 m` (leg `0.211964 m`). Joint signs are the plant axis sums. `computeLegAngle` then applies `hip_pitch_offset` on both hip pitches. The table is that pose with the gait sinusoids at zero. At time 0 the yaml's own z clock (`foot_height 0.06`, `swing_top_down 0.006`, period 600 ms, dsp 0.2) adds **−0.003 m** to `z_offset`; that sample's right-knee double-support torque is **0.313 N·m**, against **0.336 N·m** for the offset pose.

With the OP3 lengths and the same offsets, the right knee in this plant's sign is **−1.121 rad** (hip pitch **0.574 rad**, ankle pitch **−0.670 rad**). With the AiNex lengths it is **−1.225 rad**.

### The other two poses

The 0.015 m crouch is the geometric pose in method 5: forward **0.395 rad**, knee flex **0.825 rad**, sole level, foot XY on the straight-leg foot. Body-to-sole drop **0.2225 m** against a straight-leg drop of **0.2375 m**.

The +0.34 rad knee crouch is the stand in `d2e393b` (`scripts/steer_walk.py` `apply_frozen_forward_gait`: `HIP_BIAS_FWD = 0.06`, `KNEE_STANCE = 0.40`; `scripts/lipm_gait.py` adds `0.34` rad of knee and sets the ankle to hip+knee). `gait_targets` at that commit also sets hip roll to **−0.05 / +0.05 rad**. On this plant the body-to-sole distance goes from **0.2323 m** at flex 0.40 rad to **0.2179 m** at flex 0.74 rad. The measured drop is **0.0144 m**. Sole `n_z` is **0.999** because the hip roll is left in and the ankle roll stays 0.

PR #42's own stand is a different command. `scripts/op3_walk.py` there uses `z_offset 0.015 m`, x and y offsets 0, and `hit_pitch_offset_ = 0`. Its time-0 stand, z clock included, returns right knee **−0.988 rad** and hip pitch **0.475 rad** (`docs/LIPM_VENDOR_LOOK.md` on that branch rounds those to ±0.99 and ±0.48). An IK target with `z_offset 0.015 m` and the other offsets at zero, sinusoids off, returns right knee **−0.813 rad**.

| Pose | Right hip pitch | Right knee | Right ankle pitch | Body-to-sole |
|---|---|---|---|---|
| OP3 yaml, AiNex lengths | 0.578 rad | −1.225 rad | −0.769 rad | 0.2029 m |
| 0.015 m geometric crouch | 0.395 rad | −0.825 rad | −0.431 rad | 0.2225 m |
| +0.34 rad on the 0.40 rad stand | 0.060 rad | −0.740 rad | −0.680 rad | 0.2179 m |

The OP3 row's sole `n_z` is **0.993**. The IK levels the foot, and the 7° hip-pitch offset is applied after that. Straight-leg body-to-sole is **0.2375 m**, so this pose shortens that distance by **0.0346 m** (the yaml `z_offset` is 0.035 m). Left-leg magnitudes match the right to **0.0002 N·m**. The right-knee support torque is negative on the plant's −Y knee axis.

### Static torque

N·m. "Of 2.45" is `|τ| / 2.45`.

| Pose | Support | Knee | Ankle pitch | Knee / 2.45 | Knee moment arm, CoP − joint, x |
|---|---|---|---|---|---|
| OP3 yaml | double, 11.514 N/foot | 0.336 | 0.165 | 0.137 | −39.8 mm |
| OP3 yaml | single, 23.029 N | 0.793 | 0.315 | 0.324 | −39.8 mm |
| 0.015 m crouch | double | 0.203 | 0.141 | 0.083 | −25.3 mm |
| 0.015 m crouch | single | 0.494 | 0.267 | 0.202 | −25.3 mm |
| +0.34 rad | double | 0.382 | 0.140 | 0.156 | −44.2 mm |
| +0.34 rad | single | 0.891 | 0.267 | 0.364 | −44.2 mm |

With the foot unloaded (GRF 0) the OP3 right knee is **0.122 N·m**, the weight of the 0.2325 kg distal links. Every static cell is under ±2.45 N·m. The largest is the +0.34 rad pose in single support, **0.891 N·m**.

**The OP3 starting pose is not deep enough to put the knee near ±2.45 N·m before swing.** Double support, which is the pose with both feet down, is **0.336 N·m** at the right knee and **0.165 N·m** at the right ankle pitch.

### Knee armature at a 2 cm step height

Right knee, armature **0.01**. Reflected link inertia is about **0.002 kg·m²**. Armature over that link term is **4.98×** at the OP3 pose, **5.22×** at the 0.015 m crouch, and **5.05×** at the +0.34 rad pose. Achieved sole rise is **0.0200 m** on each pose (residual under **10⁻⁵ m**).

| Pose | Knee amplitude | `I_link` | 300 ms armature | 300 ms link | 300 ms full `M_ii` | 600 ms armature | 600 ms full `M_ii` |
|---|---|---|---|---|---|---|---|
| OP3 yaml | 0.337 rad | 0.002009 | 1.477 | 0.297 | 1.774 | 0.369 | 0.444 |
| 0.015 m crouch | 0.392 rad | 0.001915 | 1.718 | 0.329 | 2.047 | 0.430 | 0.512 |
| +0.34 rad | 0.308 rad | 0.001980 | 1.351 | 0.268 | 1.619 | 0.338 | 0.405 |

Armature alone, link inertia alone, and full `M_ii` alone stay under ±2.45 N·m at 300–600 ms. The largest lone term is the 0.015 m crouch at 300 ms, full `M_ii` **2.047 N·m**.

OP3 pose, static plus that acceleration. N·m.

| Period | Armature | Link | Full `M_ii` | Double + full | Single + full | Double + armature | Single + armature |
|---|---|---|---|---|---|---|---|
| 300 ms | 1.477 | 0.297 | 1.774 | 2.110 | **2.567** | 1.813 | 2.271 |
| 400 ms | 0.831 | 0.167 | 0.998 | 1.334 | 1.791 | 1.167 | 1.624 |
| 500 ms | 0.532 | 0.107 | 0.639 | 0.974 | 1.432 | 0.868 | 1.325 |
| 600 ms | 0.369 | 0.074 | 0.444 | 0.779 | 1.237 | 0.705 | 1.163 |

The cell past ±2.45 N·m is **single support plus full knee inertia at 300 ms (2.567 N·m)**. The same stack at 300 ms is **2.542 N·m** on the 0.015 m crouch and **2.510 N·m** on the +0.34 rad pose. Double support plus full inertia at 300 ms stays under the rail on all three (**2.110**, **2.251**, **2.001 N·m**). Single support plus armature, without the link term, stays under on the OP3 pose (**2.271 N·m** at 300 ms).

Peak viscous torque on the OP3 knee at 300 ms is **0.564 N·m**. Added to single support plus full inertia that is **3.132 N·m**. Added to double support plus full inertia it is **2.674 N·m**, which also crosses ±2.45 N·m. At 400–600 ms the OP3 single-support full-inertia total, viscous included, stays under (**2.215**, **1.771**, **1.519 N·m**).

Raising the OP3 foot-target z by 0.02 m, with x, y, and orientation held, changes the right knee by **−0.334 rad**. The hip-held lift above changes it by **−0.337 rad**. The hip pitch also moves **0.136 rad** in that IK, and the table assigns the height to the knee.

## Armature sensitivity

The knee peaks above use armature **0.01**, the value `scripts/convert_ainex_urdf_to_mjcf.py` writes onto a hinge that has none. This section repeats the same right-knee peak at **0.01** and at **0.045**, and does not write either number into a plant. Both plant md5s stay `71b2c86d133ebc603f58b99c53e496f3` and `17dc4ff37491c8e61900fd83b5d31f0c`.

`0.045` is the default joint armature in MuJoCo Menagerie `robotis_op3/op3.xml` (`<joint damping="1.084" armature="0.045" frictionloss="0.03"/>`), present since the initial commit `28ffb48b5c1d409a37fbfbb01c023c12e61cdefd` (2023-05-18) and still on `main`. Haarnoja et al., *Learning Agile Soccer Skills for a Bipedal Robot with Deep Reinforcement Learning*, Science Robotics (2024), DOI `10.1126/scirobotics.adi8022`, arXiv:2304.13653, fitted that actuator on an OP3 motor with a known load. The fitted values they report are damping **1.084 N·m/(rad/s)**, armature **0.045 kg·m²**, friction **0.03**, maximum torque **4.1 N·m**, proportional gain **21.1 N/rad**. The paper's robot is driven by 20 Dynamixel **XM430-350-R** servos. The ROBOTIS OP3 e-manual lists the **XM430-W350** gear ratio as **353.5:1** ([introduction](https://emanual.robotis.com/docs/en/platform/op3/introduction/), fetched 2026-10-05) and names the OP3 actuator **XM430-W350-R**. The fit publishes the reflected armature, not a separate rotor inertia.

That fit is not an HX-35H measurement. Hiwonder's HX-35H page does not publish a gear ratio or a rotor inertia (see the kit-source section). The Thanksbuyer HX-35H listing (fetched 2026-10-05) says the DC motor "is converted to higher torque through a 5-stage reduction ratio" and gives no numeric ratio ([thanksbuyer.com](https://www.thanksbuyer.com/products/hiwonder-hx-35h-35kg-cm-hv-bus-servo-dual-shaft-serial-bus-servo-w-feedback-for-robots-amp-robot-arms)). The Hiwonder SO-ARM101 manual gives **1:147** for the **HX-10HM** and **1:345** for the **HX-30HM** ([user manual](https://docs.hiwonder.com/projects/LeRobot/en/latest/docs/SO_ARM101_Open_Source_6_Axis_Robotic_Arm_User_Manual.html), fetched 2026-10-05). Those are sibling servos, not the HX-35H.

The peak is the same stack as the "full" columns above: `|τ_static| + (I_link + armature) A (2π / T)²`. `A` and `I_link` are the 2 cm hip-held knee lift already solved for each pose. Static torque does not change with the trial armature. Viscous `0.08 A (2π / T)` is left out, as it is in those columns. The script checks that the 0.01 rows match the earlier double-support and single-support full columns to `1×10⁻⁹ N·m`. Menagerie damping 1.084 and frictionloss 0.03 are not substituted.

N·m. A cell past ±2.45 N·m is marked.

| Pose | Armature | 300 ms double | 300 ms single | 400 ms double | 400 ms single | 500 ms double | 500 ms single | 600 ms double | 600 ms single |
|---|---|---|---|---|---|---|---|---|---|
| OP3 yaml | 0.01 | 2.110 | **2.567** | 1.334 | 1.791 | 0.974 | 1.432 | 0.779 | 1.237 |
| 0.015 m crouch | 0.01 | 2.251 | **2.542** | 1.355 | 1.646 | 0.941 | 1.231 | 0.715 | 1.006 |
| +0.34 rad | 0.01 | 2.001 | **2.510** | 1.293 | 1.802 | 0.965 | 1.474 | 0.787 | 1.296 |
| OP3 yaml | 0.045 | **7.280** | **7.738** | **4.242** | **4.700** | **2.836** | **3.293** | 2.072 | **2.530** |
| 0.015 m crouch | 0.045 | **8.265** | **8.556** | **4.738** | **5.029** | **3.106** | **3.397** | 2.219 | **2.510** |
| +0.34 rad | 0.045 | **6.730** | **7.239** | **3.953** | **4.462** | **2.668** | **3.176** | 1.969 | **2.478** |

**Armature is unknown, and it is not a fix.** No cited HX-35H `J_rotor × N²` exists to replace 0.01, and 0.045 is the XM430 fit above. At 0.01 the single-support peak clears ±2.45 N·m at 400, 500, and 600 ms on all three poses (OP3 pose: **1.791**, **1.432**, **1.237 N·m**) and misses at 300 ms (**2.567 N·m**). **The Controls step-time floor on this bound is >=400 ms.** **400 ms does not clear ±2.45 N·m at armature 0.045.** On the OP3 pose that row is **4.242 N·m** in double support and **4.700 N·m** in single support. At 0.045 the single-support peak stays past the rail through 600 ms on every pose (OP3 pose **2.530 N·m**).

## Kit-walk armature upper bound

The real-kit file is [`walking_param.yaml`](https://github.com/Hiwonder/ainex/blob/e8fe2a816797cf83054135160df5a82ec3596a69/src/ainex_driver/ainex_kinematics/config/walking_param.yaml) (blob `106db7619f167821b32fb1c3fa490f197c6a90f2`). Its `period_time` is **400 ms**. `move(1)` is a faster override of that period, **300 ms**, and it keeps this file's crouch and swing height. The bound is the same single-support knee stack as the table above: `|τ_static| + (I_link + J) A (2π / T)²`, viscous term left out. `J` is solved so that peak equals the limit. Both plant md5s stay `71b2c86d133ebc603f58b99c53e496f3` and `17dc4ff37491c8e61900fd83b5d31f0c`.

### What move(1) sets

Checkout [`Hiwonder/ainex`](https://github.com/Hiwonder/ainex) **`e8fe2a816797cf83054135160df5a82ec3596a69`**. `gait_manager.py` (blob `c07bf9722c5c43cf362572498f148616600696a6`):

- `dsp_ratio[0]` is `[300, 0.2, 0.02]` (lines 21–24): period 300 ms, double-support ratio 0.2, `y_swap_amplitude` 0.02 m.
- `move` (lines 199–208) treats speed `1` as index 0 and calls `set_step` with `get_gait_param()`. That call copies the live service. It does not write a new step height or body height.
- `get_gait_param` (lines 63 and 69) maps `body_height` from `init_z_offset` and `step_height` from `z_move_amplitude`, with no divide. `update_param` (line 139) writes `step_height` straight back into `z_move_amplitude`.
- `x_amplitude_range` is `[0.0, 0.02]` (line 30). Step length is the caller's `x_amplitude`, not a field of the preset. The commented call under `if __name__` is `move(1, 0.02, 0, 0)` (line 225). `ainex_tutorial/scripts/gait_control/simple_gait_control_demo.py` calls `move(1, 0.01, 0, 0)` (line 13). The knee lift below holds the hip, so that length does not enter `A`.

`ainex_controller.py` (blob `2e3df366f8318fd422221da21bd46776a4e6cde9`) loads `walking_param.yaml` at startup (lines 146 and 307). That file (blob `106db7619f167821b32fb1c3fa490f197c6a90f2`) is the service `move(1)` reads until something else writes it:

| Field | yaml | Line |
|---|---|---|
| `trajectory_step_s` | 0.008 s | 1 |
| `servo_control_cycle` | 0.02 s | 2 |
| `init_x_offset` | 0.0 m | 3 |
| `init_y_offset` | −0.005 m | 4 |
| `init_z_offset` | 0.025 m | 5 |
| `hip_pitch_offset` | 15 deg | 9 |
| `period_time` | 400 ms | 10 |
| `dsp_ratio` | 0.2 | 12 |
| `x_move_amplitude` | 0.00 m | 14 |
| `z_move_amplitude` | 0.02 m | 16 |
| `y_swap_amplitude` | 0.02 m | 18 |

`move(1)` writes period 300 ms, double-support ratio 0.2, and `y_swap_amplitude` 0.02 m. Against this file the ratio and the sway are already those values, so the override that changes the gait is the period, 400 ms to 300 ms. It leaves `z_move_amplitude` and `init_z_offset` alone. The docstring on `get_gait_param` (lines 54–55) says body height defaults to 0.015 m. That sentence matches `body_height_range` (line 28), not this file. The controller loads 0.025 m.

`walking_param_sim.yaml` in the same directory (blob `dad8cc46244ea1b02a07c6ff57dbda2cc3ed2500`) is the sim file. Its `period_time` is **1500** (line 10), `dsp_ratio` is 0.3 (line 12), and `z_move_amplitude` is 0.015 (line 16). The bound below does not use it.

`trajectory_step_s` and `servo_control_cycle` are the module's integration step and the servo period. The peak below is the continuous sinusoid at the gait period. Those clocks are not used as `T`.

A different 300 ms path is the app callback `speed == 4` (`ainex_controller.py` lines 423–444). It sets `z_move_amplitude` to **0.015 m** and, when x is nonzero, forces `|x|` to 0.01 m or 0.012 m. That is not `move(1)`.

### The internal half

Python never divides the step height. The trajectory lives in `walking_module.so` (same tree, BuildID `cdeca905b563adb25757b97170bd2e3330a29ebd`). Its `update_movement_param` follows the current OP3 `WalkingModule::updateMovementParam` at [`ROBOTIS-OP3` `3bc2bd51`](https://github.com/ROBOTIS-GIT/ROBOTIS-OP3/blob/3bc2bd514e8ee6054e9726d23c8c0b13236f84f0/op3_walking_module/src/op3_walking_module.cpp): the forward amplitude is halved only when the previous x command was zero, the lateral swap adds `shift * 0.04`, and the next parameter is divided by two and then halved again for the shift. Those two lines are:

```cpp
z_move_amplitude_ = walking_param_.z_move_amplitude / 2;
z_move_amplitude_shift_ = z_move_amplitude_ / 2;
```

(lines 383–384). The .so follows that order. After the term that multiplies by the constant 0.04, it true-divides the next walking-param value, stores the quotient, and multiplies by one half for the shift. In the OP3 function that pair is the z amplitude, and the 0.04 term is the line just above it. `wSin` is `mag * sin(2π t / period − phase) + shift` (OP3 lines 244–246), phase `π/2`, and `z_move_period_time_ = period * ssp_ratio / 2` (line 345). Over the swing window the sine runs from −1 to +1 and back to −1. With `mag = H/2` and `shift = H/4` the endpoints are `−H/4` and the peak is `3H/4`, so the swing foot rises by **H** above the stance foot. The script checks that: commanded 0.020 m, endpoint −0.005 m, peak 0.015 m, rise **0.020 m**. The same check at 0.015 m returns a rise of 0.015 m. The internal half is the sine coefficient. It is not a 1 cm step.

The sole rise for this yaml, at 400 ms and at the 300 ms `move(1)` override, is **2.0 cm**. The script checks both periods: commanded 0.020 m, endpoint −0.005 m, peak 0.015 m, rise 0.020 m. The 0.5 cm and 1.0 cm rows are the same equation at those rises. They are not a second preset. The 1.5 cm row is the app `speed == 4` command.

### Stance and the bound

The stand is that yaml run through the same IK as the OP3 section: AiNex lengths, sinusoids off, `hip_pitch_offset` 15° applied after the IK. Right hip pitch **0.768 rad**, right knee **−1.053 rad**, right ankle pitch **−0.547 rad**, body-to-sole **0.2037 m**. Sole `n_z` is **0.966** because the 15° offset pitches the foot after the IK has leveled it. The contact-box center is **8.8 mm** behind the knee. On this free-body diagram the right knee is **0.038 N·m** in double support and **0.140 N·m** in single support. `I_link` at that pose is **0.001943 kg·m²**.

The lift search stays on the flexed side of that knee. Extending through straight is a second root that also raises the sole; a gait step does not take it. Achieved rise matches the target within 10⁻⁴ m.

`α = A (2π / T)²` with `T` the gait period. `J_max = (τ_limit − τ_ss) / α − I_link`. Limits are the thaw running rail **2.45 N·m** and the HX-35H static maximum. The product page lists that maximum as **35 kg·cm** at 11.1 V. `1 kgf·cm = 0.0980665 N·m`, so `35 × 0.0980665 = 3.4323 N·m`. The stall columns use **3.43 N·m**. Knee amplitude does not depend on the period. The 400 ms columns are the real-kit `period_time`. The 300 ms columns are `move(1)`.

| Sole rise | What it is | Knee amplitude | 400 ms `J` at 2.45 | 400 ms `J` at 3.43 | 300 ms `J` at 2.45 | 300 ms `J` at 3.43 |
|---|---|---|---|---|---|---|
| 0.5 cm | same solve, not a preset | 0.246 rad | 0.03608 | 0.05221 | 0.01944 | 0.02852 |
| 1.0 cm | same solve, not a preset | 0.350 rad | 0.02480 | 0.03615 | 0.01310 | 0.01948 |
| 1.5 cm | app `speed == 4` | 0.441 rad | 0.01930 | 0.02832 | 0.01001 | 0.01508 |
| 2.0 cm | yaml `z_move_amplitude` | 0.523 rad | **0.01597** | **0.02357** | **0.00814** | **0.01241** |

On the real-kit preset (400 ms, 2.0 cm rise, this crouch) the single-support knee stays inside ±2.45 N·m for armature **≤ 0.01597 kg·m²**, and inside the 3.43 N·m stall figure for armature **≤ 0.02357 kg·m²**. Link inertia alone at that period peaks at 0.390 N·m. The converter value 0.01 is under both 400 ms ceilings.

`move(1)` is 300 ms. `gait_manager.py` lines 21 and 207–208 select `dsp_ratio[0]`, whose first entry is 300. On the same crouch and the same 2.0 cm rise the ceilings drop to **0.00814 kg·m²** at ±2.45 N·m and **0.01241 kg·m²** at 3.43 N·m. Link inertia alone peaks at 0.585 N·m. The converter value 0.01 is above the 300 ms running-rail ceiling and below the stall ceiling. At the 1.5 cm app command the 300 ms running-rail ceiling is 0.01001, so 0.01 sits on that rail.

The three poses already in the knee table, at their published 2 cm amplitudes and the same 300 ms single-support formula:

| Pose | Amplitude | Single-support static | `J` max at 2.45 | `J` max at 3.43 |
|---|---|---|---|---|
| OP3 yaml | 0.337 rad | 0.793 N·m | 0.00921 | 0.01584 |
| 0.015 m crouch | 0.392 rad | 0.494 N·m | 0.00947 | 0.01517 |
| +0.34 rad | 0.308 rad | 0.891 N·m | 0.00956 | 0.01681 |

## Foot box vs ankle axis

Neither plant has a keyframe. The pose is every joint at zero, pelvis (`body_link`) at the world origin in X and Y, so the coordinates below are world and pelvis frame together. +X is forward, +Y is to the robot's left, +Z is up. The box-to-ankle offset is box center minus the ankle-roll anchor: Y positive means the box center is to the left of that ankle, X positive means it is forward of the ankle. Both feet read the same signs. A stance y-offset is applied outward-positive: it is added to the left foot's Y and subtracted from the right foot's Y, with hip roll and ankle roll holding the sole level and the sagittal joints left at zero. The inner-edge gap is the left box's minimum world Y minus the right box's maximum world Y. Negative is overlap. No plant XML is changed. Both md5s stay `71b2c86d133ebc603f58b99c53e496f3` and `17dc4ff37491c8e61900fd83b5d31f0c`.

The two skeletons match at this pose. Ankle-roll anchors are at **(y, x) = (±28.95 mm, −18.89 mm)**. Hip-roll anchors are at **y = ±29.00 mm** (x −19.55 mm). The contact box is centered on the ankle axis laterally and **30.0 mm** forward of it.

| Plant | Foot | Ankle (y, x) mm | Hip roll y mm | Box centre (y, x, z) mm | Box full size (x, y, z) mm | Box − ankle (y, x) mm |
|---|---|---|---|---|---|---|
| Main 145×86 | right | −28.95, −18.89 | −29.00 | −28.95, 11.11, 38.54 | 145.0 × 86.0 × 16.0 | 0, +30.0 |
| Main 145×86 | left | +28.95, −18.89 | +29.00 | +28.95, 11.11, 38.54 | 145.0 × 86.0 × 16.0 | 0, +30.0 |
| Thaw 135×76 | right | −28.95, −18.89 | −29.00 | −28.95, 11.11, 38.54 | 135.0 × 76.0 × 16.0 | 0, +30.0 |
| Thaw 135×76 | left | +28.95, −18.89 | +29.00 | +28.95, 11.11, 38.54 | 135.0 × 76.0 × 16.0 | 0, +30.0 |

Compiled half-sizes are 72.5 × 43.0 × 8.0 mm on main and 67.5 × 38.0 × 8.0 mm on the thaw. Both boxes have `contype` 1 and `conaffinity` 1, so the pair is allowed to contact. The mesh geoms are `contype` 0.

| Offset per side | Main gap | Main feet collide | Thaw gap | Thaw feet collide |
|---|---|---|---|---|
| 0 | −28.10 mm | yes, 4 contacts | −18.10 mm | yes, 4 contacts |
| −5 mm (inward) | −38.12 mm | yes, 4 contacts | −28.12 mm | yes, 4 contacts |
| +5 mm | −18.09 mm | yes, 4 contacts | −8.09 mm | yes, 4 contacts |
| +18 mm | +7.87 mm | no, distance 7.87 mm | +17.87 mm | no, distance 17.87 mm |

Where the lateral overlap is deeper than the 16 mm box height, `mj_geomDistance` reports **−16.0 mm**. That is the vertical overlap, the smallest separating axis, not the lateral gap. At +18 mm, and at the thaw's +5 mm row, the distance matches the lateral gap.

The Hiwonder URDF collision for `r_ank_roll_link` and `l_ank_roll_link` is the link mesh with origin **0 0 0** (`ainex.urdf.xacro` lines 366–374 and 708–716). The ankle joint origin is `−0.019037 ±0.020 0` (lines 379–381 and 721–723). The STL in this repo, in that ankle frame, measures 135.1 × 76.1 mm in X and Y. Its center is **+29.6 mm** forward and **14.0 mm outward** of the ankle axis (right mesh Y center −14.0 mm, left +14.0 mm), not on the axis. The inner face of each mesh is **24.1 mm** toward the midline from the ankle axis. At the zero pose those inner faces clear by **9.8 mm**. The contact boxes are centered on the axis, so each one reaches **18.9 mm** (main) or **13.9 mm** (thaw) past that mesh inner face. The boxes overlap each other. The meshes do not.

## Verdict

**sim body matches kit within 15%, so hip rail is gait/Controls.**

No leg link is a mass or link-inertia candidate for a later plant peel. The sim torso, hips, thighs, shanks, ankles, and feet are the Hiwonder URDF inertials. The Standard product weight is 4.2% above that sum and is not a per-link error in this plant.

Armature `0.01` is an uncited add, about 1.4× the reflected link inertia at the crouched hips. At the kit gait presets the torque it adds is a fraction of ±2.45 N·m. It is not changed here. A later peel would need a cited rotor inertia; this pass did not find one for the HX-35H.

The OP3 yaml start, scaled to these leg lengths, holds the right knee at **0.336 N·m** in double support and **0.793 N·m** in single support. That is the stand, before the 2 cm lift. The 300 ms single-support stack with the full knee inertia of that lift is **2.567 N·m**, past ±2.45 N·m. The stand by itself is not. At armature 0.01 the same stack clears from 400 ms up, so the Controls step-time floor is >=400 ms. Armature 0.045, the Menagerie XM430 fit, leaves the 400 ms single-support peak at **4.700 N·m**. The HX-35H reflected inertia is still unpublished, and this pass does not change 0.01.

The real-kit `walking_param.yaml` preset is 400 ms with a 2.0 cm sole rise and `init_z_offset` 0.025 m. That single-support stack stays under ±2.45 N·m up to armature **0.01597 kg·m²**, and under the 3.43 N·m stall figure up to **0.02357 kg·m²**. `move(1)` replaces the period with 300 ms and keeps the crouch and the rise, which tightens those ceilings to **0.00814** and **0.01241 kg·m²**.
