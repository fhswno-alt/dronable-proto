# AiNex foot outsole — drawing only

Printable TPU outsole for the Hiwonder AiNex foot. Each foot has its own pocket, cut from that plate’s mesh, with a 1.2 mm perimeter wall. This is a drawing. It does not change the plant, it does not touch `kit_cam`, and it is not a quote.

The plant file stays commit `921f5941`, `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`, md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`.

Solids are **millimetres** in the **ankle-roll link frame**. The plant XML is in metres. Scale STEP/STL by 0.001 before placing them in that model. The older `cad/m2_outsole` pack (145 × 86 mm) is a different drawing and is left as it is.

## What changed for Manufacturing

- A **1.2 mm wall** runs around each pocket. The outline is that plate’s own bounding box, plus 0.3 mm clearance per side, plus the 1.2 mm wall per side.
- The wall underside is **0.50 mm** above the lug tips (z = −25.500 versus z = −26.000), with a 0.4 mm chamfer on the bottom edge.
- Lug tips exist only inside the plant **135 × 76 mm** box. The wall and the pocket shelf outside that box stay off the ground.
- The right pocket is cut from the right mesh (URDF lines 366–374). The left pocket is cut from the left mesh (lines 708–716). They are not mirrors of each other.

## Outlines and clearance

Clearance is the gap from every vertex of that foot’s mesh to the pocket wall. The pocket is the mesh AABB expanded by 0.3 mm, so each side measures 0.300 mm.

| | Left | Right |
| --- | ---: | ---: |
| Mesh AABB | 135.083 × 76.037 mm | 135.083 × 76.051 mm |
| Mesh centre | (29.673, 13.982) mm | (29.627, −13.974) mm |
| Underside z | −23.086 mm | −23.078 mm |
| Pocket | 135.683 × 76.637 mm | 135.683 × 76.651 mm |
| Outline | **138.083 × 79.037 mm** | **138.083 × 79.051 mm** |
| Heel clearance | 0.300 mm | 0.300 mm |
| Toe clearance | 0.300 mm | 0.300 mm |
| Inboard clearance | 0.300 mm | 0.300 mm |
| Outboard clearance | 0.300 mm | 0.300 mm |

“About 138.1 × 79.0 mm” is this pair. The two outlines differ by 0.014 mm in width because the two meshes do.

Mesh sources:

- Left collision mesh, `ainex.urdf.xacro` lines 708–716, filename on line 714: `l_ank_roll_link.STL`.
- Right collision mesh, lines 366–374, filename on line 372: `r_ank_roll_link.STL`.

Those blocks are mesh references only. No screw or clip hole is in the URDF or in the 936-triangle STLs, so none is cut here.

## Vertical stack

Lug tips sit on the plant ground plane, z = −26 mm (`pos` z = −0.018 and half-height 0.008 on lines 77 and 107). Pocket-floor z is that foot’s underside rounded down to 0.001 mm.

| Face | Left z (mm) | Right z (mm) |
| --- | ---: | ---: |
| Lug tips (ground) | −26.000 | −26.000 |
| Wall underside | −25.500 | −25.500 |
| Groove root | −24.286 | −24.279 |
| Pocket floor | −23.086 | −23.079 |
| Wall top | −20.586 | −20.579 |

| Thickness | Left | Right |
| --- | ---: | ---: |
| Lug | 1.714 mm | 1.721 mm |
| Pocket floor web | 1.200 mm | 1.200 mm |
| Wall above the floor | 2.500 mm | 2.500 mm |
| Wall lift off the lugs | 0.500 mm | 0.500 mm |
| Printed part, ground to wall top | 5.414 mm | 5.421 mm |
| Plant contact box | 16 mm | 16 mm |

The 16 mm box runs from z = −26 mm to z = −10 mm. These parts share that ground plane and stop at about z = −20.6 mm. The upper **10.586 mm** (left) of the box is above the outsole. It is still the collision-box height, not a rubber thickness.

## Inner foot-to-foot gap

Zero stance is every leg joint at angle 0. Every joint origin in the chain has `rpy="0 0 0"`, so the ankle-roll origin Y in the body frame is the sum of the origin Y values.

| Joint | Origin y (m) | xacro line |
| --- | ---: | ---: |
| `l_hip_yaw` | +0.029 | 437 |
| `l_hip_roll` | 0 | 494 |
| `l_hip_pitch` | +0.020 | 551 |
| `l_knee` | −0.00005 | 608 |
| `l_ank_pitch` | 0 | 665 |
| `l_ank_roll` | −0.020 | 722 |
| **Left ankle** | **+0.02895 m = +28.950 mm** | |
| `r_hip_yaw` | −0.029 | 95 |
| `r_hip_roll` | 0 | 152 |
| `r_hip_pitch` | −0.020 | 209 |
| `r_knee` | +0.00005 | 266 |
| `r_ank_pitch` | 0 | 323 |
| `r_ank_roll` | +0.020 | 380 |
| **Right ankle** | **−0.02895 m = −28.950 mm** | |

The right-hand residuals in the URDF (`l_hip_pitch` is exactly 0.02; `r_hip_pitch` and `r_ank_roll` carry 1e-13 m terms) cancel below 0.0001 mm.

The inner edge is the left outline’s minimum Y and the right outline’s maximum Y. At zero stance the gap is **6.812 mm**.

Kit stance moves each ankle **+0.005 m outward** and leaves the sole level (local Y still parallel to world Y, which is what plant line 9 calls `xmat I`). The gap grows by 10 mm, to **16.812 mm**.

A 79.0 mm outline centred on the plant centres y = ±14 would put each inner edge 25.500 mm inboard of its ankle-roll origin’s lateral offset from the body centre. With ankles at ±28.950 mm that estimate is 2 × (28.950 − 25.500) = **6.900 mm**, and 16.900 mm after the kit shift. The real plates are 79.037 mm and 79.051 mm wide and are centred on the mesh AABB (13.982 and −13.974), not on ±14.000, so the computed gaps are **6.812 mm** and **16.812 mm**.

## Plant contact box (unchanged)

| Dimension | Value | Source at `921f5941` |
| --- | --- | --- |
| Tread length | 135 mm | lines 77 and 107, `size` x = `0.0675` |
| Tread width | 76 mm | same, `size` y = `0.0380` |
| Box height | 16 mm | same, `size` z = `0.008` |
| Centre x | 30 mm | `pos` x = `0.030` |
| Left centre y | +14 mm | line 107, `pos` y = `0.014`. Line 9 states the offset. |
| Right centre y | −14 mm | line 77, `pos` y = `-0.014` |
| Ground | z = −26 mm | −18 − 8 |
| Sliding friction | 1.6 | `friction="1.6 0.1 0.01"` |

Only tread inside that 135 × 76 mm rectangle reaches z = −26 mm. Ground contact area on the left is 6280 mm², 61.2% of the box, because of the grooves.

## Material and attachment

Print in **TPU 95A**. Put the **ground face on the bed and the pocket facing up**. The wall underside is open toward the bed only by the 0.5 mm lift plus the chamfer, so it does not need support; the pocket is facing up.

Bond the pocket floor to the plate. The 1.2 mm wall locates the plate with 0.3 mm clearance per side. There are still no kit hole positions in the URDF or the mesh, so there are no screws in this drawing.

Plant friction 1.6 belongs to the sim and the high-friction demo mat. It is not a measured TPU 95A coefficient.

## Files

| File | What it is |
| --- | --- |
| `outsole.py` | CadQuery builder, gap calculation, PNG renders |
| `check_outsole.py` | Wall lift, ground patch, per-side clearance, both gaps, plant md5 |
| `outsole_left.step` / `outsole_right.step` | Solids, millimetres, each ankle-roll frame |
| `outsole_left.stl` / `outsole_right.stl` | Same solids |
| `outsole_top.png` | Left pocket and wall |
| `outsole_bottom.png` | Ground face |
| `outsole_iso.png` | Isometric |
| `outsole_overlay.png` | Left mesh, wider outline, 135 × 76 tread, 14 mm offset |
| `outsole_section.png` | Side section at x = 30 mm, with the wall lift called out |

```bash
python3 hardware/outsole/outsole.py
python3 hardware/outsole/check_outsole.py
```

## Open questions

- Dry-fit both kit plates. The meshes are 936-triangle visual STLs, not an OEM manufacturing model. The 0.3 mm clearance is a drawing allowance.
- The right mesh is not the mirror of the left. Its outline is 0.014 mm wider and its floor is 0.007 mm higher.
- Whether Controls later shortens the 16 mm box to the ~5.4 mm stack is separate from this drawing. The ground stays at z = −26 mm.
- If the physical plate has screws, measure them on the part. They are not in the URDF or the mesh.
