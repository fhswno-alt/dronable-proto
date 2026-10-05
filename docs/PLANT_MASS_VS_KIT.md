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

## Verdict

**sim body matches kit within 15%, so hip rail is gait/Controls.**

No leg link is a mass or link-inertia candidate for a later plant peel. The sim torso, hips, thighs, shanks, ankles, and feet are the Hiwonder URDF inertials. The Standard product weight is 4.2% above that sum and is not a per-link error in this plant.

Armature `0.01` is an uncited add, about 1.4× the reflected link inertia at the crouched hips. At the kit gait presets the torque it adds is a fraction of ±2.45 N·m. It is not changed here. A later peel would need a cited rotor inertia; this pass did not find one for the HX-35H.
