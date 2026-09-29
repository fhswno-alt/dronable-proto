# Gate F — lever prop fab prep (Manufacturing)

**When:** Mon 28 Sep 2026 ~01:33 Europe/London (BST)  
**Audience:** Founding Manufacturing Lead · Hardware  
**Aligns with:** `docs/GATE_F_HARDWARE_CAM_LEVER.md` · `docs/GATE_F_LEVER_STEP_STUB.md` · `docs/GATE_F_LEVER_FAB_QC.md` · `docs/M2_FAB_QC_FROZEN_145.md` · `docs/GATE_F_AI_CRITERIA.md`  
**Scope:** Pre-position QC for a **demo prop lever** only. Sim-only. **No PO.**

---

## Demo prop (research — NOT TO ORDER)

| Item | Spec |
|------|------|
| Lever height | **275 mm AFF** (product band **250–300 mm**; working center matches Hardware `door_lever`) |
| Role | Look / approach / reach gauge — **not** a UK full-height door handle |
| Candidate build | UK thin-fab stub: plywood or acrylic panel + cheap lever handle, **or** 3D-print stub + handle |
| Rough UK £ band | **~£30–80** (laser/print panel + handle; from historical kit brief) — **research only, NOT TO ORDER** |

Companion plant (sim): `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml`. Locked M145 foot plant (`ainex_controls_m2_145.xml` contact / STEP) **untouched**.

## Dims / clearances (match Hardware companion)

| Dim | Value | Note |
|-----|-------|------|
| Lever Z AFF | **0.275 m** | Hardware `door_lever` geom center |
| Historical envelope | **250–300 mm** AFF | Same Manufacturing band; 275 mm is the gauge center |
| Cam optical Z | **≈ 0.380 m** | `kit_cam_site` grounded; FOV bind |
| Look-down | `head_tilt` **≈ −16°** @ **0.4 m** cam→door | Optional ≈ −19° @ 0.3 m grasp row |
| Hand reach Z (sim F02) | **~0.29 m** AFF (L 0.291 in 0.25–0.30) | Kinematic hold — fab prop must sit in that band |
| Sim geom (visual) | ~60×10×8 mm box @ world Z 0.275 | Physical prop may be larger handle; **height AFF is the QC lock** |

## QC checks (STEP stub on disk)

Hardware dropped a **demo-prop** STEP/STL stub — caliper / tape vs AFF gauge.  
Pointers: **`docs/GATE_F_LEVER_STEP_STUB.md`** (spec) · **`docs/GATE_F_LEVER_FAB_QC.md`** (incoming QC — **PASS**).

| Artifact | Path |
|----------|------|
| STEP | `cad/gate_f_lever/demo_prop_lever_275AFF_meshDerived_notOEM.step` |
| STL | `cad/gate_f_lever/demo_prop_lever_275AFF_meshDerived_notOEM.stl` |
| README | `cad/gate_f_lever/README.md` |
| Incoming QC | `docs/GATE_F_LEVER_FAB_QC.md` |

Stub dims: panel **8 × 100 × 340 mm** (T×W×H, bottom on Z=0); lever bar **60 × 10 × 8 mm** (L×W×T) centerline **275 mm AFF**. Label: **demo prop / not OEM door hardware / mesh-derived sim gauge**. Solids **exist on disk** — use STEP/STL above for QC (no “awaiting lever STEP”).

1. **Height AFF** — lever grasp axis / center at **275 ±5 mm** above finished floor (or marked floor plane).
2. **Band** — entire graspable surface inside **250–300 mm** AFF.
3. **Reach vs hand Z** — standing kit reach to lever Z ≈ **0.29 m** (F02 band 0.25–0.30) without floor clash; reject if prop forces grasp below ~0.25 m or above ~0.30 m.
4. **Stand-off** — prop face usable at ~0.4 m cam→door (sim door_prop ahead +X); no overhang that blocks toes at approach.
5. **Labels** — tag **demo prop / not OEM door hardware**; mesh-derived cam Z honesty stays with Hardware doc.

## Freeze

- **No PO** until Dave asks after sim / assembly review.
- **STEP foot pack stays 145×86** (`docs/M2_FAB_QC_FROZEN_145.md`) — this note is **lever prop only**; do not reopen outsole / tread / pocket.
- Gate F lock is sim look+approach (`GATE_F_LOCKED_TRUE`); fab remains pre-positioned idle.
