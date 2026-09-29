# Foot contact variants (Hardware, 2026-09-27)

Path A frozen. Mesh-derived / not OEM STEP.  
**Sole bottom −0.026 / `SOLE_OFFSET = 0.026` on all M145-family plants below unless noted.**  
Locked score plant: `ainex_controls_m2_145.xml` until AI cospec says otherwise. See `docs/PLANT_CONTACT_HONESTY_PACK.md`.

Base: `ainex_controls.xml` — contact box full **110 × 56 × 16 mm** (half 0.055 × 0.028 × 0.008), pos +0.030 / 0 / −0.018 on `*_ank_roll_link`.

| File | GEO | Planform (full) | Vertical | Friction | Soft | Use |
|------|-----|-----------------|----------|----------|------|-----|
| `ainex_controls_meshsole.xml` | — | **135 × 76 mm** | 16 mm | 1.6 | default | Match ank_roll STL AABB |
| `ainex_controls_m2_145.xml` | GEO00 | **145 × 86 mm** | 16 mm | 1.6 | default | **Locked** manufacturing M2 short / score plant |
| `ainex_controls_m2_145_cadsole.xml` | GEO01 | **145 × 86 mm** | **4.5 mm** CAD | 1.6 | default | Vertical honesty A/B (not Gate E by itself) |
| `ainex_controls_m2_160x90.xml` | GEO02a | **160 × 90 mm** | 16 mm | 1.6 | default | Planform honesty primary (+15×+4 mm) |
| `ainex_controls_m2_155x86.xml` | GEO02b | **155 × 86 mm** | 16 mm | 1.6 | default | Planform honesty secondary (+10 mm length) |
| `ainex_controls_m2_160x90_cadsole.xml` | GEO02c | **160 × 90 mm** | **4.5 mm** | 1.6 | default | Planform + CAD vertical combo |
| `ainex_controls_m2_145_softsole.xml` | GEO03 | **145 × 86 mm** | 16 mm | 1.6 | **solref 0.05 0.8 / solimp 0.85 0.90 0.01 0.5 2** | Soft-contact / compliance honesty |
| `ainex_controls_m2_155.xml` | — | **155 × 96 mm** | 16 mm | 1.6 | default | Legacy M2 nominal (+20 mm); prefer GEO02b for +10 mm length A/B |

Same HX clips / PD / meshes as locked M145 on all GEO plants. Load via `walk_gait_ainex.py --model mujoco/ainex_hiwonder/<file>`.  
For Controls contact-honesty grid after AUTH HARD-FALSIFIED — **not** a soft-pass unlock / Gate E claim.
