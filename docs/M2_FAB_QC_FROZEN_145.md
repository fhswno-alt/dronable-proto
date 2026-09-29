# M2 fab QC — frozen 145×86 STEP pack

**Date:** 2026-09-28 (Europe/London)  
**Audience:** Founding Manufacturing Lead · Hardware  
**Aligns with:** `docs/PLANT_TO_CAD_ASSEMBLY_HONESTY.md` · `cad/m2_outsole/README.md`  
**Scope:** Incoming QC for the frozen M2 145×86 mesh-derived STEP pack. Sim-only prep. **No PO.**

---

## Frozen geometry (do not resize)

| Item | Spec | Files |
|------|------|-------|
| Outsole plate | **145 × 86 × 3 mm** | `cad/m2_outsole/M2_outsole_145x86_meshAABB_notOEM.{step,stl}` |
| Sleeve pocket | **137 × 78 × ~2.2 mm** deep (≈0.8 mm floor) | same plate STEP |
| Tread | **145 × 86 × 1.5 mm** TPU 95A | `…/M2_tread_145x86_TPU_notOEM.{step,stl}` |
| Stack | **~4.5 mm** (3 + 1.5 plate+tread) | README |
| M1 bumper (optional) | **~50 × 28 × 8 mm** TPU | `…/M1_bumper_50x28x8_TPU_notOEM.{step,stl}` |

All exports labelled **mesh-derived / not OEM STEP**. Pocket is symmetric — one STEP serves L and R (mirror as needed).

---

## Plant ↔ CAD honesty (QC must keep this straight)

| Claim | Value | QC implication |
|-------|-------|----------------|
| Planform | **145 × 86 mm** = locked M145 plant (`ainex_controls_m2_145.xml` contact boxes) | Caliper L×W must match plate/tread planform |
| Sim contact height | **16 mm** XML box | **Sim proxy only** — do **not** fab to 16 mm |
| CAD stack | **~4.5 mm** plate+tread | Fab prints / machines the **CAD stack**, not the XML height |

Vertical mismatch is documented and A/B’d; it is not a fab thickness target.

---

## Incoming QC checks

Use calipers (±0.1 mm nominal) against STEP nominals. Reject or hold for Hardware review if outside typical fab tolerance for the process used.

1. **Planform dims** — plate and tread each **145 × 86 mm** (L×W).
2. **Plate thickness** — **3 mm** nominal; stack with tread ≈ **4.5 mm**.
3. **Pocket** — **137 × 78** opening, ~2.2 mm deep / ~0.8 mm floor; dry-fit against `l_ank_roll_link` / `r_ank_roll_link` mesh AABB (sleeve, not OEM clip holes).
4. **Zip-tie slots** — four heel/toe rim slots present, clear, usable for bond+tie attach (no OEM clip holes expected).
5. **TPU tread** — print/bond quality: no delamination, voids, or warpage that breaks 145×86 planform; bond face mates plate.
6. **L/R pair** — two feet (plate+tread each); pocket symmetry OK for shared STEP; mark L/R at assembly.
7. **Optional M1 bumpers** — **~50 × 28 × 8**; qty as kit note (2 + spares); outer-edge survival only.
8. **Labels** — parts tagged mesh-derived / not OEM; no silent OEM claim.

---

## Freeze / Gate E rule

- STEP pack is **frozen** at M145 (145×86 planform, 4.5 mm CAD stack) until a **GEO/plant wins Gate E** and **Dave gates a refresh**.
- This checklist is **pre-positioned only** for when fab is authorized.
- **No PO** from this note. Hardware stays idle on plant geometry while Controls scores locked M145.

**Gate context:** Gates D–G locked on M145 / companion as applicable; foot STEP still frozen 145×86. Sleeve dry-fit cross-check: `docs/M2_ANK_ROLL_SLEEVE_FIT_QC.md` (**PASS**) ↔ `docs/ANK_ROLL_SLEEVE_FIT.md`.
