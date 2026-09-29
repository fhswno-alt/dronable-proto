# Gate H — lever hinge Manufacturing re-QC

**When:** Mon 28 Sep 2026 ~01:44 Europe/London (BST)  
**Audience:** Founding Manufacturing Lead · Hardware · Dave  
**Aligns with:** `docs/GATE_H_HARDWARE_LEVER_HINGE.md` · `docs/GATE_F_LEVER_FAB_QC.md` · `docs/GATE_F_LEVER_STEP_STUB.md`  
**Scope:** Re-QC frozen hinge dims vs existing fixed-bar lever STEP. **Sim hinge honesty. No PO. No STEP rewrite this turn.**

---

## Verdict

| Lane | Result |
|------|--------|
| **Rest-pose / fixed-bar STEP** (Gate F stub) | **PASS** — AFF, bar envelope, panel unchanged vs Gate F QC |
| **Hinged assembly STEP** | **DEFERRED** — not on disk; not required to unblock H02; refresh only if Dave authorizes a rotating physical prop later |

---

## Frozen hinge dims (from Hardware) vs fab stub

| Item | Hardware (companion) | Fixed-bar STEP / Gate F QC | Δ / note |
|------|----------------------|----------------------------|----------|
| Grasp center Z AFF (rest q=0) | **0.275 m** | **275 mm** (±5 lock) | **0** — **PASS** |
| Bar envelope | half-extents `0.06 × 0.01 × 0.008` m → **60 × 10 × 8 mm** | **60 × 10 × 8 mm** | **0** — **PASS** |
| Hinge axis | world **Z** through door-side near end | N/A (welded solid) | Sim-only DOF |
| Range | **±30°** (±0.5236 rad); Criteria ≥15° | N/A | Sim-only; no STEP joint |
| Panel | visual on `door_prop` | **8 × 100 × 340 mm** | Unchanged — **PASS** |
| Contact | bit-2 on moving `door_lever` | N/A to solids QC | G03 kept |

At rest the moving geom matches the prior welded AABB → Gate F fab QC of the fixed-bar stub remains the correct caliper target for a **static** demo prop.

---

## What we are **not** claiming

- Fixed-bar STEP is **not** a hinged assembly (no pin, bushing, or rotating bar solid).  
- Sim hinge ≠ OEM door-handle kinematics / UK handle-height product.  
- **No PO** / NOT TO ORDER. Foot STEP **145×86** untouched.

---

## Later (only if Dave asks for a physical rotating prop)

Minimum STEP refresh package (out of scope tonight):

1. Fixed panel + vertical hinge pin at shaft.  
2. Rotating bar solid (same 60×10×8) with ±30° stop faces or printed hard stops.  
3. QC: rest AFF still 275 ±5; free swing ≥15° without binding; labels demo-prop / not OEM.

Until then: keep `cad/gate_f_lever/demo_prop_lever_275AFF_meshDerived_notOEM.*` as the fab reference; Gate H is companion-sim only.

**Refs:** `docs/GATE_H_HARDWARE_LEVER_HINGE.md` · `docs/GATE_F_LEVER_FAB_QC.md` · `docs/MFG_FREEZE_STATUS_MONDAY.md`
