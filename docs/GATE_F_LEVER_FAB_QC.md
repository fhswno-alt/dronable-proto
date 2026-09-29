# Gate F — lever demo-prop incoming fab QC

**When:** Mon 28 Sep 2026 ~01:33 Europe/London (BST)  
**Audience:** Founding Manufacturing Lead · Hardware  
**Aligns with:** `docs/GATE_F_LEVER_STEP_STUB.md` · `docs/GATE_F_LEVER_FAB_PREP.md` · `docs/GATE_F_HARDWARE_CAM_LEVER.md` · `docs/M2_FAB_QC_FROZEN_145.md`  
**Scope:** Incoming QC of demo-prop lever STEP/STL stub vs stub dims. **Lever-only.** Sim-aligned. **No PO.**

---

## Verdict: **PASS**

All checklist items match stub nominals within measurement/parse precision (delta **0 mm** vs `GATE_F_LEVER_STEP_STUB.md`). No HOLD items.

---

## Artifacts inspected

| Role | Path | Notes |
|------|------|-------|
| STEP | `cad/gate_f_lever/demo_prop_lever_275AFF_meshDerived_notOEM.step` | 29172 B; Open CASCADE; `SI_UNIT(.MILLI.,.METRE.)` |
| STL | `cad/gate_f_lever/demo_prop_lever_275AFF_meshDerived_notOEM.stl` | 1484 B; binary; 28 tris |
| README | `cad/gate_f_lever/README.md` | Locked dims + honesty labels |
| Stub pointer | `docs/GATE_F_LEVER_STEP_STUB.md` | Spec source for this QC |
| Build script | `scripts/build_gate_f_lever.py` | CadQuery boxes; LEVER_Z=275 |

**CAD tools:** OpenCASCADE / CadQuery not installed on box for live load. QC used file presence + STEP `CARTESIAN_POINT` AABB parse + binary STL bbox + README / build-script nominals (same mm units).

---

## Measured vs stub

Composite AABB (STEP points + STL): **X [−64, 4] = 68** · **Y [−50, 50] = 100** · **Z [0, 340] = 340** mm.  
Decomposed from STEP vertices:

| Feature | Stub nominal | Measured | Δ | Result |
|---------|--------------|----------|---|--------|
| Lever grasp centerline AFF | **275 mm** (±5 QC lock) | **275.0 mm** (bar Z [271, 279], mid) | **0 mm** | **PASS** |
| Graspable bar band | **250–300 mm** AFF | **271–279 mm** AFF | inside band | **PASS** |
| Lever bar L × W × T | **60 × 10 × 8 mm** | **60 × 10 × 8 mm** (X [−64,−4], Y [−5,5], Z [271,279]) | **0 / 0 / 0 mm** | **PASS** |
| Panel T × W × H | **8 × 100 × 340 mm** | **8 × 100 × 340 mm** (X [−4,4], Y [−50,50], Z [0,340]; bottom on Z=0) | **0 / 0 / 0 mm** | **PASS** |
| Filename / product labels | mesh-derived / not OEM / demo prop | `…_meshDerived_notOEM.{step,stl}`; README: **demo prop / not OEM door hardware / mesh-derived sim gauge** | — | **PASS** |

---

## Checklist (vs `GATE_F_LEVER_FAB_PREP.md`)

1. **Height AFF** — grasp axis **275 ±5 mm** above floor plane → **PASS** (Δ 0).  
2. **Band** — graspable surface inside **250–300 mm** AFF → **PASS**.  
3. **Bar envelope** — **60 × 10 × 8 mm** vs STEP/STL → **PASS**.  
4. **Labels** — tag **demo prop / not OEM door hardware** (+ mesh-derived) → **PASS**.

Reach / stand-off (prep items 3–4) are physical-kit checks — **N/A** for this solids-only incoming QC; re-run when a physical prop is authorized.

---

## Freeze boundaries (unchanged)

- **Foot STEP pack stays 145 × 86** — `docs/M2_FAB_QC_FROZEN_145.md` / `cad/m2_outsole/` **not** reopened.  
- This note is **lever-only**; do not touch `ainex_controls_m2_145.xml` contact / HX.  
- **No PO** / NOT TO ORDER until Dave asks after sim / assembly review. Research UK £ band in prep doc remains research-only.
