# Gate I — panel hinge Manufacturing re-QC

**When:** Mon 28 Sep 2026 ~01:51 Europe/London (BST)  
**Audience:** Founding Manufacturing Lead · Hardware · Dave  
**Aligns with:** `docs/GATE_I_HARDWARE_PANEL_HINGE.md` · `docs/GATE_H_LEVER_HINGE_FAB_QC.md` · `docs/GATE_F_LEVER_FAB_QC.md`  
**Scope:** Dims freeze check vs existing fixed-bar lever STEP. **Sim panel hinge honesty. No PO. No STEP rewrite.**

---

## Verdict

| Lane | Result |
|------|--------|
| **Rest-pose lever** (AFF / bar envelope vs fixed-bar STEP) | **PASS** — unchanged from Gate H re-QC |
| **Panel STEP / hinged panel assembly** | **N/A this gate** — Hardware: panel STEP **not required** (visual-only geom); hinged STEP stays **DEFERRED** |
| **Foot STEP 145×86** | **FROZEN** — untouched |

---

## Frozen dims vs fab stub

| Item | Hardware (companion Gate I) | Fixed-bar STEP / Gate F–H QC | Result |
|------|-----------------------------|------------------------------|--------|
| Grasp center Z AFF (rest both q=0) | **0.275 m** | **275 mm** (±5) | **PASS** |
| Lever bar | **60 × 10 × 8 mm** (unchanged) | **60 × 10 × 8 mm** | **PASS** |
| Lever site rest | world `(0.40, 0, 0.275)` | bar centerline AFF 275 | **PASS** |
| Panel hinge | `door_panel_hinge` Z, ±30°; criteria ≥10° | N/A (welded solid) | Sim-only DOF |
| Panel geom (sim) | half-extents `0.01 × 0.05 × 0.17` m → **20 × 100 × 340 mm** | STEP panel **8 × 100 × 340 mm** | **Honesty only** — W×H match; thickness is sim visual proxy (like foot XML height ≠ CAD stack). **Not a fab thickness target.** |
| Coupling | lever **child of** panel | N/A to solids | Sim-only |

---

## Explicit

- **No PO / no spend.** Foot freeze holds.  
- No hinged panel or hinged lever STEP refresh asked — Hardware deferred panel STEP for Gate I.  
- Fixed-bar stub remains the caliper reference for a **static** demo prop if Dave later authorizes fab.

**Refs:** `docs/GATE_I_HARDWARE_PANEL_HINGE.md` · `docs/GATE_H_LEVER_HINGE_FAB_QC.md` · `docs/GATE_F_LEVER_FAB_QC.md`
