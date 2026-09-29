# ank_roll sleeve / M2 outsole pocket — fit honesty

**Date:** 2026-09-28 (Europe/London, BST)  
**Audience:** Founding Hardware · Manufacturing QC  
**Aligns with:** `docs/PLANT_TO_CAD_ASSEMBLY_HONESTY.md` · `docs/M2_FAB_QC_FROZEN_145.md` · `cad/m2_outsole/README.md`  
**Scope:** Sim/CAD dry-fit honesty only. **Gates D–G locked; foot STEP frozen at 145×86.** No spend. No STEP refresh. No plant hop.

---

## Verdict: **PASS** (mesh-derived sleeve dry-fit)

| Check | Result |
|-------|--------|
| Mesh AABB footprint vs pocket 137×78 | **Fits** — positive clearance both axes, L and R |
| Design intent (1 mm/side on ~135×76) | **Holds** — measured ~0.96 mm/side L and W |
| OEM clip / hole claim | **None** — sleeve + zip-tie path only |
| Plant / STEP change | **None** — M145 frozen; measurements only |

**PASS** = mesh AABB planform sits inside the frozen pocket opening with documented clearance; honesty for sleeve dry-fit, **not** an OEM mating claim.

---

## Measured mesh AABB (`*_ank_roll_link`)

Source: binary STL vertex AABB, meters → mm (`mujoco/ainex_hiwonder/meshes/`).

| Link | File | AABB X×Y×Z (mm) | Planform (X×Y mm) | Height Z (mm) |
|------|------|-----------------|-------------------|---------------|
| L | `l_ank_roll_link.STL` | **135.083 × 76.037 × 34.325** | 135.083 × 76.037 | 34.325 |
| R | `r_ank_roll_link.STL` | **135.083 × 76.051 × 33.911** | 135.083 × 76.051 | 33.911 |

L bounds (mm): X [−37.869, 97.214] · Y [−24.037, 52.000] · Z [−23.086, 11.240]  
R bounds (mm): X [−37.914, 97.169] · Y [−52.000, 24.051] · Z [−23.078, 10.833]

L/R planforms match within ~0.015 mm on Y; X identical at reported precision. Onshape folder label: **AiNex Hiwonder mesh-derived (NOT OEM STEP)**.

CAD README design basis “135×76 mesh AABB” is the rounded intent; measured values above are what clearance uses.

---

## Frozen pocket (M2 outsole plate)

| Item | Spec | Source |
|------|------|--------|
| Plate outer | **145 × 86 × 3 mm** | STEP/STL AABB (CadQuery + STL) |
| Pocket opening | **137 × 78 mm** | STEP cavity probe: material wall at \|X\|≥68.5, \|Y\|≥39.0 |
| Pocket depth | **~2.2 mm** (floor ≈0.8 mm) | STEP: first solid at (0,0) from top at Z=0.800 |
| Files | `cad/m2_outsole/M2_outsole_145x86_meshAABB_notOEM.{step,stl}` | frozen pack |

Pocket is symmetric — one STEP for L and R (mirror as needed).

---

## Clearance: pocket 137×78 vs mesh footprint

| Foot | Mesh footprint (mm) | Clearance total L×W (mm) | Clearance / side (mm) |
|------|---------------------|--------------------------|------------------------|
| L | 135.083 × 76.037 | **1.917 × 1.963** | **0.958 × 0.982** |
| R | 135.083 × 76.051 | **1.917 × 1.949** | **0.958 × 0.974** |

- Both axes **positive** → mesh AABB does not foul the pocket opening.  
- ~1 mm/side design (README: “1 mm/side on 135×76”) remains honest at measured ~0.96 mm/side.  
- Mesh **height** (~34 mm) is the ank_roll visual solid above the sole — **not** a pocket-depth claim; pocket only sleeves the foot underside (~2.2 mm deep).

---

## Attach path (explicit)

- **Assumed:** bond + **zip-tie** through four heel/toe rim slots on the plate.  
- **Not claimed:** OEM AiNex clip holes, OEM sole STEP, or OEM fastener pattern.  
- All solids labelled **mesh-derived / not OEM STEP**.

---

## Method (reproduce)

1. Binary STL AABB on `{l,r}_ank_roll_link.STL` (numpy `frombuffer` facets; scale ×1000 m→mm).  
2. CadQuery `importStep` on outsole → outer AABB 145×86×3; point-in-solid cavity scan → pocket 137×78×2.2.  
3. Clearance = pocket opening − mesh planform AABB (X×Y).

No STEP rewrite, no plant XML edit, no fab PO.

---

## Pointers

- Plant ↔ CAD checklist item **10** (ank_roll fit): this note.  
- Fab QC pocket dry-fit: `docs/M2_FAB_QC_FROZEN_145.md` § Incoming QC #3.  
- CAD pack: `cad/m2_outsole/README.md`.
