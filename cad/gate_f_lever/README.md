# Gate F demo-prop lever — mesh-derived sim gauge / NOT OEM door hardware

**When:** Mon 28 Sep 2026 ~01:32 Europe/London (BST)  
**Status:** Demo prop / research stub for Manufacturing QC. **Sim-aligned only.**  
**Labels:** **demo prop / not OEM door hardware / mesh-derived sim gauge**

Aligns with `docs/GATE_F_HARDWARE_CAM_LEVER.md`, `docs/GATE_F_LEVER_FAB_PREP.md`, `docs/GATE_F_LEVER_STEP_STUB.md`.  
Rebuild: `scripts/build_gate_f_lever.py` (CadQuery, same pattern as `scripts/build_cad.py`).

## Locked dims (QC)

| Item | Spec |
|------|------|
| Lever grasp center / axis | **275 mm AFF** (band **250–300 mm**) |
| Lever bar L × W × T | **60 × 10 × 8 mm** (matches sim visual `door_lever` size attrs in mm) |
| Panel T × W × H | **8 × 100 × 340 mm** (thin vertical stub; bottom on Z=0 floor) |
| Coord | Z up; panel sits on Z=0; lever centerline at Z=275; bar protrudes −X (approach) |

## Files

- `demo_prop_lever_275AFF_meshDerived_notOEM.step`
- `demo_prop_lever_275AFF_meshDerived_notOEM.stl`

## Honesty

- Sim companion geom only (`ainex_controls_m2_145_gate_f.xml` `door_lever` @ world Z 0.275).
- **Not** UK full-height OEM door hardware. **Not** a fab PO.
- Foot STEP pack stays frozen at **145×86** (`cad/m2_outsole/`, `docs/M2_FAB_QC_FROZEN_145.md`) — this folder is lever prop only.
