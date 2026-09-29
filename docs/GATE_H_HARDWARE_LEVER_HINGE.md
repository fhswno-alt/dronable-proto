# Gate H Hardware — light lever hinge (companion)

**When:** Mon 28 Sep 2026 ~01:43 Europe/London (BST)  
**Owner:** Founding Hardware Engineer  
**Scope:** Gate **H02 lever rotate ≥15°** — add light hinge DOF on companion plant only. **Sim only. No spend. No M145 hop.**  
**Labels:** mesh-derived kit demo prop; not OEM door hardware; companion additive.

## Files

| Role | Path |
|------|------|
| Locked walk/score plant (**untouched**) | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` |
| **Gate F/G/H companion (edit here)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` |
| Gate H criteria | `docs/GATE_H_AI_CRITERIA.md` |
| Gate G contact honesty | `docs/GATE_G_HARDWARE_LEVER_CONTACT.md` |
| Gate F cam/lever honesty | `docs/GATE_F_HARDWARE_CAM_LEVER.md` |

Companion = locked M145 + Gate F cam/door + Gate G03 hand–lever contact + **Gate H02 hinged lever**.  
**145×86 foot boxes, HX actuator clips, kit_cam, locked M145: unchanged.**

## Hinge choice (kit demo)

Horizontal door-lever demo: bar lies in the **XY** plane at rest; shaft is **vertical (world Z)** through the **door-side near end** of the bar. Lever rotates in the horizontal plane (yaw about Z) — common door-handle kit demo (not a vertical-plane latch throw).

| Item | Value |
|------|-------|
| Parent | `door_prop` @ `(0.45, 0, 0)` |
| Moving body | `door_lever_link` @ local `(0.01, 0, 0.275)` = shaft / near end |
| Joint name | **`door_lever_hinge`** |
| Joint type | `hinge` |
| Axis (local = world) | **`(0, 0, 1)`** — vertical Z |
| Range | **`−0.5236 … +0.5236` rad** (±30°); Criteria need ≥15°; task margin ≥±20° |
| Rest / springref | **0°** (lever along −X from shaft) |
| Softness | `damping="0.02"`, `stiffness="0.05"` (light center spring), `armature="0.0001"` |
| Lever mass | `0.015` kg; COM at geom center `(-0.06, 0, 0)` relative to shaft body |

### Rest pose / grasp height

| Item | Rest (q=0) |
|------|------------|
| Shaft (door-side near end) | door_prop-local `(0.01, 0, 0.275)` → world `(0.46, 0, 0.275)` |
| Geom / site center | shaft + `(-0.06, 0, 0)` → door_prop-local `(-0.05, 0, 0.275)` → world **`(0.40, 0, 0.275)`** |
| **Grasp center Z AFF** | **0.275 m** (unchanged from Gate F/G fixed lever) |
| Lever half-extents | `0.06 × 0.01 × 0.008` m (same box as welded geom) |

At 0° the moving geom occupies the same world AABB as the previous welded `door_lever`, so G02 reach / G03 contact geometry at rest is preserved.

## Contype scheme (G03 still works)

Unchanged from Gate G:

| Bit | Value | Who | Purpose |
|-----|-------|-----|---------|
| 0 | `1` | `floor`, `l_foot_contact`, `r_foot_contact` | Walk / stance |
| 1 | `2` | **`door_lever`**, `l_hand_contact`, `r_hand_contact` | Hand–lever light contact |

- `door_lever` geom: `contype="2" conaffinity="2" condim="3"` on the **moving** child body.
- `door_panel`: visual-only `contype="0"` / fixed on `door_prop`.
- Hand ↔ lever still collides on bit 2; lever still does **not** collide with floor/feet.

## Names for Controls H00–H02

| Use | Name |
|-----|------|
| Hinge joint (angle / ≥15°) | `door_lever_hinge` |
| Lever contact geom | `door_lever` |
| Lever site (arc / distance; **on moving body**) | `door_lever_site` |
| Moving body | `door_lever_link` |
| Hand sites / contact | `l_hand_site`, `r_hand_site`, `l_hand_contact`, `r_hand_contact` |
| Load plant | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` |

H00/H01 (reach / grasp hold) can still run at rest with hinge at 0°. H02: command or contact-drive `door_lever_hinge` ≥ **15°** (or measure `door_lever_site` arc ≥15°) without tip/skate fail.

## Manufacturing / STEP note

- Sim hinge is a **demo prop** DOF for Gate H — **not** OEM door-handle kinematics / not UK handle-height product claim.
- Existing fab stub `cad/gate_f_lever/demo_prop_lever_275AFF_meshDerived_notOEM.step` (+ STL) remains a **fixed-bar** solid for Gate F QC. **STEP stub may need refresh later** if Manufacturing wants a hinged assembly; out of scope for this H02 unblock (no spend / no STEP edit this ship).
- Foot STEP / M2 145×86 sole: frozen; untouched.

## Explicit non-claims

- Locked **`ainex_controls_m2_145.xml`** not edited (md5 must match pre-edit).
- No foot contact / friction / HX / kit_cam change.
- No full door-panel swing / latch release / UK handle height.
- Light centering spring must not be treated as a lock — range and soft stiffness allow ≥15° under hand contact.
- Not Pi / Orin / NN-first; vision stays out of walk residual.

## Smoke

```text
model = mujoco.MjModel.from_xml_path("mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml")
# joint door_lever_hinge: axis Z, range ±0.5236, qpos0 → lever site Z ≈ 0.275
# door_lever contype=2 conaffinity=2; door_panel contype=0
# md5 of ainex_controls_m2_145.xml unchanged (fc94709c84f5598d4474ecfc4bb41fdc)
```
