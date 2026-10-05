# AiNex foot outsole — drawing only

Printable TPU outsole for the Hiwonder AiNex foot, as a mirrored left/right pair. The outer planform matches the plant foot contact box on commit `921f5941` (`mujoco/ainex_hiwonder/ainex_controls_m2_145.xml`, md5 `207f3d5e9c6a72e16f7aa0c8d224f75e`).

This is a drawing. It does not change the plant, it does not touch `kit_cam`, and it is not a quote or a purchase.

The older `cad/m2_outsole` pack (145 × 86 mm) is a different drawing and is left as it is.

Solids are **millimetres** in the **ankle-roll link frame** (the frame of `l_ank_roll_link.STL` / `r_ank_roll_link.STL`). The plant XML is in metres. Scale these STEP/STL files by 0.001 before placing them in that model.

## Mismatches (kept visible)

1. **Thickness versus the 16 mm box.** The plant contact box is 16 mm tall and runs from z = −26 mm to z = −10 mm. The printed outsole is **5.414 mm** tall and runs from z = −26.000 mm (lug tips) to z = −20.586 mm (cleat tops). Both share the ground plane, so the sole height under the ankle-roll origin stays 26 mm. The upper **10.586 mm** of the box (z = −20.586 mm to z = −10 mm) is above this part and overlaps the hard foot plate. The 16 mm figure is the collision-box height. It is not a rubber thickness to print.

2. **The stock plate is not inside the 135 × 76 mm rectangle on every edge.** The left mesh AABB is 135.083 × 76.037 mm, centred at (29.673, 13.982) mm. The plant rectangle is 135 × 76 mm, centred at (30, 14) mm. Left overhang past that rectangle (positive means the mesh is outside):

   | Edge | Left mesh vs plant rectangle |
   | --- | ---: |
   | Heel (min x) | +0.369 mm outside |
   | Toe (max x) | −0.286 mm (mesh inside the rectangle) |
   | Inboard (min y) | +0.037 mm outside |
   | Outboard (max y) | 0.000 mm (under 0.001 mm) |

   A pocket grown from that outline by 0.3 mm per side therefore breaks out through all four flats of the 135 × 76 mm part. The chamfered corners still leave four cleats. The plate overhangs the finished heel by about 0.37 mm.

3. **The right mesh is not an exact mirror of the left mesh.** The outsole pair is an exact mirror. Right mesh AABB centre is (29.627, −13.974) mm, underside z = −23.078 mm. Against the mirrored pocket polygon the tightest right-hull vertex has **0.168 mm** of clearance, at a point that already sits about 0.05 mm outside the outer footprint. There is no cleat interference. The designed 0.3 mm clearance holds on the left hull and is thinner on the right mesh.

4. **Ground contact area is 61.2% of the box.** Lug tips cover 6280 mm² of the 10260 mm² planform. Grooves are recessed, so the first contact patch is smaller than the plant’s solid 135 × 76 mm rectangle.

5. **Friction 1.6 is the plant coefficient**, `friction="1.6 0.1 0.01"` on both contact geoms, used with a high-friction demo mat. It is not a measured coefficient for TPU 95A.

## Dimensions and sources

Plant file: `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` at commit `921f5941`. MuJoCo `size` is the half-extent, in metres.

| Dimension | Value | Source |
| --- | --- | --- |
| Outer length | 135 mm | line 77 and line 107, `size` x = `0.0675` → 135 mm |
| Outer width | 76 mm | same geoms, `size` y = `0.0380` → 76 mm |
| Contact-box height | 16 mm | same geoms, `size` z = `0.008` → 16 mm. Collision box only. |
| Centre x | 30 mm | line 77 and line 107, `pos` x = `0.030` |
| Left centre y | +14 mm | line 107, `pos` y = `0.014`. Line 9 states the same offset. |
| Right centre y | −14 mm | line 77, `pos` y = `-0.014` |
| Box centre z | −18 mm | line 77 and line 107, `pos` z = `-0.018`. Line 9 keeps x/z at 0.030 / −0.018. |
| Ground / lug tips | z = −26 mm | box centre z − half height = −18 − 8 |
| Box top | z = −10 mm | −18 + 8 |
| Sliding friction | 1.6 | `friction="1.6 0.1 0.01"` on line 77 and line 107 |

Line 9 (same commit):

> Foot contact: mesh AABB 135x76 mm (half-size 0.0675 0.0380 0.008 m; mesh-derived / not OEM). Centre y is the STL AABB centre, 14 mm outboard of the ankle axis: l local +0.014 (xmat I, world +Y), r local -0.014 (world -Y). x/z stay 0.030 / -0.018.

Foot mesh pointers in `cad/vendor/ainex-thorobotics/ainex_description/urdf/ainex.urdf.xacro`:

- Right collision mesh, lines 366–374. The filename is line 372: `package://ainex_description/meshes/r_ank_roll_link.STL`.
- Left collision mesh, lines 708–716. The filename is line 714: `package://ainex_description/meshes/l_ank_roll_link.STL`.

Those blocks are a mesh reference only. They contain no cylinder, hole, or fastener primitive. The STL files are watertight visual meshes of 936 triangles each. No screw-hole positions are present, so none are cut in this outsole.

Measured from those STLs (binary vertex AABB, metres × 1000):

| | Left | Right |
| --- | --- | --- |
| File | `meshes/l_ank_roll_link.STL` | `meshes/r_ank_roll_link.STL` |
| AABB size | 135.083 × 76.037 × 34.325 mm | 135.083 × 76.051 × 33.911 mm |
| AABB centre | (29.673, 13.982, −5.923) mm | (29.627, −13.974, −6.123) mm |
| Underside z | −23.086 mm | −23.078 mm |

The 34 mm height is the whole ankle-roll link (plate plus ankle housing). The pocket uses the convex hull of vertices below z = −16.5 mm, which is the chamfered sole plate, not the ankle tower.

## Vertical stack

Pocket-floor z is the left underside rounded down to 0.001 mm, so the floor stays at or below the tessellated plate.

| Face | z (mm) | How it is set |
| --- | ---: | --- |
| Lug tips (ground) | −26.000 | plant box bottom |
| Groove root | −24.286 | 1.200 mm below the pocket floor |
| Pocket floor (plate seat) | −23.086 | left mesh min z, rounded down to 0.001 mm |
| Cleat top | −20.586 | 2.500 mm above the pocket floor |

| Thickness | mm | Role |
| --- | ---: | --- |
| Lug height | 1.714 | tread below the web |
| Pocket floor web | 1.200 | solid between the seat and the groove root. Requirement is ≥ 1.0 mm. |
| Rubber under the plate | 2.914 | lug + web. Plate underside to the plant ground plane. |
| Cleat / pocket depth | 2.500 | locating stops above the floor. Design choice. |
| **Printed part** | **5.414** | lug + web + cleat |
| Plant box | 16.000 | shared ground, taller by 10.586 mm |

Left plate-to-floor gap is under 0.001 mm (mesh z = −23.0857 mm, floor z = −23.086 mm). Right gap is 0.008 mm because that mesh underside is higher and the pair shares one floor height.

## Pocket, clearance, tread

- **Clearance:** 0.3 mm outward from the left sole convex hull, in XY. This is a print assumption, not a measured process allowance.
- **What drops in:** the chamfered plate, located by four corner cleats. The long edges and the heel/toe flats are open because the hull-plus-clearance outline falls outside the locked 135 × 76 mm footprint there.
- **Left hull to pocket polygon:** 0.300 mm.
- **Right hull to the mirrored pocket polygon:** minimum 0.168 mm, still positive.
- **Tread:** block grid. Grooves are 1.6 mm wide on a 6.4 mm pitch. A 2.0 mm rail runs around the perimeter. An 8.0 mm square at each corner stays solid under the cleat. Groove depth equals the lug height, so the web under the seat stays 1.2 mm.
- **Contact fraction:** 61.2% of the planform.

## Material and how it attaches

Print in **TPU 95A** (a nearby 90A–98A flexible grade is the same intent). Put the **ground face on the bed and the pocket facing up**. The 1.6 mm grooves then sit on the first layers, and the pocket needs no support. A 0.4 mm nozzle can hold a 1.6 mm groove.

Attachment is **adhesive** across the pocket floor, with the corner cleats locating the chamfered corners of the plate. The URDF and the mesh do not show kit hole positions, so this drawing has no screw holes and does not claim a kit clip pattern. The cleats are stops beside the chamfers. They do not wrap over the top of the plate. Retention is the bond.

The plant’s friction of 1.6 is the sim value on the high-friction demo mat. Confirm the real TPU-on-mat pair before treating 1.6 as a hardware number.

## Files

| File | What it is |
| --- | --- |
| `outsole.py` | Parametric CadQuery builder, exporter, and PNG renders |
| `check_outsole.py` | Asserts mirror, 135 × 76 mm, centres at y = ±14 mm, floor ≥ 1.0 mm, and the plant md5 |
| `outsole_left.step` / `outsole_right.step` | Solids, millimetres, ankle-roll frame |
| `outsole_left.stl` / `outsole_right.stl` | Same solids |
| `outsole_top.png` | Left part, pocket and cleats |
| `outsole_bottom.png` | Left part, tread |
| `outsole_iso.png` | Left part, isometric |
| `outsole_overlay.png` | Left mesh plus outsole, with the 14 mm offset marked |
| `requirements.txt` | CadQuery, NumPy, Matplotlib (VTK comes with CadQuery) |

Rebuild and check from the repo root:

```bash
python3 hardware/outsole/outsole.py
python3 hardware/outsole/check_outsole.py
```

The right solid is the left solid mirrored through the XZ plane. Its centre is local y = −14 mm.

## Open questions

- The mesh is a 936-triangle visual STL, mesh-derived, not an OEM manufacturing model. Dry-fit a kit plate before relying on the 0.3 mm clearance.
- The right foot should be dry-fit on its own. Its mesh is close to the mirror of the left, and the tightest hull clearance on the mirrored pocket is 0.17 mm.
- 0.3 mm per side is an allowance for this drawing. A specific printer and TPU lot may want more.
- Whether the plant box should later be shortened to the 5.414 mm stack is a controls decision. This drawing keeps the ground at z = −26 mm so the sole height matches the current box, and it leaves the 10.586 mm height gap on the record.
- If the physical plate has screws or clips, measure them on the part. They are not in the URDF or the mesh, and this outsole does not invent them.
