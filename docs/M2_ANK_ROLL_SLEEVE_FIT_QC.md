# Manufacturing QC — ank_roll sleeve fit vs frozen M2 pocket

**Date:** 2026-09-28 ~01:43 BST (Europe/London)  
**Audience:** Founding Manufacturing Lead · Hardware · Dave  
**Aligns with:** `docs/ANK_ROLL_SLEEVE_FIT.md` · `docs/M2_FAB_QC_FROZEN_145.md`  
**Scope:** Incoming QC cross-check of Hardware sleeve-fit note. Sim/CAD dry-fit only. **No PO. No STEP refresh.**

---

## Verdict: **PASS**

| Check | Spec / claim | Measured / confirmed | Result |
|-------|--------------|----------------------|--------|
| Pocket opening | **137 × 78 mm** (frozen M2 QC) | Hardware STEP cavity probe + fab QC freeze | **Match** |
| Mesh planform L | ~135 × 76 | **135.083 × 76.037 mm** (independent STL AABB) | **Match** |
| Mesh planform R | ~135 × 76 | **135.083 × 76.051 mm** | **Match** |
| Clearance / side | ~1 mm design | L **0.958 × 0.982**; R **0.958 × 0.974** | **PASS** (both axes positive) |
| Attach path | bond + zip-tie; no OEM holes | Matches M2 fab QC § Incoming #3–4 | **OK** |
| Foot STEP | 145 × 86 frozen | Unchanged | **No refresh** |

**PASS** = Hardware’s dry-fit numbers reproduce under Manufacturing STL AABB; pocket clearance stays ~1 mm/side; Incoming QC #3 (pocket dry-fit) is satisfied on paper. Still **not** an OEM mating claim.

---

## Fab implications (when Dave authorizes)

1. Dry-fit printed/machined pocket against kit `ank_roll` (or mesh print stand-in) before bonding tread.  
2. Hold if clearance collapses under real part tolerance (target ≥0.5 mm/side after process).  
3. Zip-tie slots must remain usable — no fill from over-extrusion.  
4. Do **not** enlarge pocket or planform without a plant/STEP refresh gated by Dave.

**No PO** from this note. Gate H hinge QC is separate (waiting on Hardware dims).
