# Gate Q — Hardware H3: foot mesh AABB vs contact-box reconfirm

**When:** Mon 28 Sep 2026 ~21:52 Europe/London (BST)  
**Owner:** Founding Hardware Engineer  
**Scope:** READ-ONLY dig. **No XML edit** to live freeze or H0 fork contact boxes. Soft-pass off. No spend.

Parent: `docs/GATE_Q_HARDWARE_H0_INSTRUMENT.md` · levers: `docs/GATE_Q_HARDWARE_T5D_PLANT_LEVERS.md`

---

## Numbers (reconfirmed on disk)

### Contact box (locked M145 = companion feet)

| Item | Value |
|------|-------|
| Geom | `l_foot_contact` / `r_foot_contact` |
| Half-size | `0.0725 0.0430 0.008` m |
| Full planform × height | **145 × 86 × 16 mm** |
| `pos` (ank_roll) | `0.030 0.0 -0.018` |
| Sole bottom / `SOLE_OFFSET` | **−0.026** / **0.026** |
| Friction | `1.6 0.1 0.01` |

### ank_roll STL AABB (meshes/`l|r_ank_roll_link.STL`, binary)

| Foot | AABB min (m) | AABB max (m) | Size L×W×H (mm) |
|------|--------------|--------------|-----------------|
| L | `[-0.03787, -0.02404, -0.02309]` | `[0.09721, 0.05200, 0.01124]` | **135.1 × 76.0 × 34.3** |
| R | `[-0.03791, -0.05200, -0.02308]` | `[0.09717, 0.02405, 0.01083]` | **135.1 × 76.1 × 33.9** |

### Delta (contact − mesh planform)

| Axis | Contact | Mesh AABB | Δ |
|------|---------|-----------|---|
| Length (X) | 145 mm | ~135 mm | **+~10 mm** sleeve |
| Width (Y) | 86 mm | ~76 mm | **+~10 mm** sleeve |
| Height (Z box) | 16 mm proxy | mesh Z ~34 mm (structure) / CAD sole stack **~4.5 mm** | XML height ≠ fab; sole bottom box −0.026 vs mesh min-Z ≈ −0.023 |

**Verdict:** Contact box remains the intentional M2 short manufacturing sleeve over mesh AABB. Honesty alternatives already on disk (`meshsole` 135×76; `cadsole` 4.5 mm height) — **not** installed on live freeze. H2 CAD-height A/B still needs Dave unlock. No freeze edit from H3.
