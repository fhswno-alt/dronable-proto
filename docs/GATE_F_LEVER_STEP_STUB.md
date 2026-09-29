# Gate F — lever STEP/STL stub (Manufacturing QC pointer)

**When:** Mon 28 Sep 2026 ~01:32 Europe/London (BST)  
**Audience:** Founding Manufacturing Lead · Hardware  
**Aligns with:** `docs/GATE_F_LEVER_FAB_PREP.md` · `docs/GATE_F_HARDWARE_CAM_LEVER.md`  
**Scope:** Demo-prop lever solids for caliper / tape QC. **Sim-aligned only.** No PO language as an ask.

---

## Label

**demo prop / not OEM door hardware / mesh-derived sim gauge**

## CAD paths

| Role | Path |
|------|------|
| Directory | `cad/gate_f_lever/` |
| README | `cad/gate_f_lever/README.md` |
| STEP | `cad/gate_f_lever/demo_prop_lever_275AFF_meshDerived_notOEM.step` |
| STL | `cad/gate_f_lever/demo_prop_lever_275AFF_meshDerived_notOEM.stl` |
| Build script | `scripts/build_gate_f_lever.py` |

## Dims (panel on Z=0 floor)

| Dim | Value | Note |
|-----|-------|------|
| Lever grasp centerline AFF | **275 mm** | QC lock ±5 mm vs `GATE_F_LEVER_FAB_PREP.md` |
| Product band | **250–300 mm** AFF | Entire graspable bar surface |
| Lever bar L × W × T | **60 × 10 × 8 mm** | Sim visual `door_lever` (~60×10×8 mm box @ Z 0.275) |
| Panel T × W × H | **8 × 100 × 340 mm** | Thin vertical stub; bottom on finished floor |
| Cam optical Z (context) | **≈ 0.380 m** | Hardware `kit_cam_site`; not part of this solid |

Solids: vertical panel + horizontal lever bar (CadQuery boxes). Bar protrudes from panel −X face; centerline at Z=275 when panel rests on Z=0.

## QC (point Manufacturing here)

Use checklist in `docs/GATE_F_LEVER_FAB_PREP.md` against these files:

1. Height AFF — grasp axis **275 ±5 mm** above floor plane.
2. Band — graspable surface inside **250–300 mm** AFF.
3. Bar envelope — **60 × 10 × 8 mm** (L×W×T) vs STEP/STL.
4. Labels — tag **demo prop / not OEM door hardware**.

## Freeze boundaries

- **Foot STEP pack stays 145×86** — do not reopen `cad/m2_outsole/` or `docs/M2_FAB_QC_FROZEN_145.md`.
- Do **not** modify `ainex_controls_m2_145.xml` contact / HX.
- Companion plant for sim context only: `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml`.
