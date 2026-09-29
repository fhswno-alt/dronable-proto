# Plant pack — contact / geometry honesty (sim)

**Date:** 2026-09-27 (Europe/London)  
**Owner:** Hardware (XML) · Controls scores · AI locks cospec  
**Purpose:** Pre-stage GEO plants Controls can load via `walk_gait_ainex.py --model` once AI contact-geometry cospec lands.  
**Not a Gate E claim.** Locked M145 remains the default / score plant until AI cospec says otherwise.  
**Sim only. No Path A / spend talk.**

## Locked baseline (GEO00)

| Field | Value |
|-------|-------|
| File | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` |
| Planform (full) | **145 × 86 mm** (half `0.0725 × 0.0430`) |
| Vertical (full) | **16 mm** collision proxy (half `0.008`) |
| Contact `pos` | `0.030 0.0 -0.018` |
| Sole bottom | `pos_z − half_z = **−0.026**` |
| `SOLE_OFFSET` | **0.026** (`(-pos_z) + half_z`) |
| Friction (floor + feet) | `1.6 0.1 0.01` |
| Soft params | MuJoCo defaults (no geom `solref`/`solimp`) |

## Pack table (GEO01–GEO03)

| GEO | File | Planform (full mm) | Half-size (m) | Vertical | `pos` z | Sole bottom | Friction | Soft / notes | Δ vs locked M145 |
|-----|------|--------------------|---------------|----------|---------|-------------|----------|--------------|------------------|
| GEO00 | `ainex_controls_m2_145.xml` | 145 × 86 | `0.0725 0.0430 0.008` | 16 mm | −0.018 | −0.026 | 1.6 | default | — (locked) |
| GEO01 | `ainex_controls_m2_145_cadsole.xml` | 145 × 86 | `0.0725 0.0430 0.00225` | **4.5 mm** CAD stack | −0.02375 | −0.026 | 1.6 | default | half_z −0.00575; pos_z −0.00575 |
| GEO02a | `ainex_controls_m2_160x90.xml` | **160 × 90** | `0.080 0.045 0.008` | 16 mm | −0.018 | −0.026 | 1.6 | default | +15 mm L, +4 mm W |
| GEO02b | `ainex_controls_m2_155x86.xml` | **155 × 86** | `0.0775 0.0430 0.008` | 16 mm | −0.018 | −0.026 | 1.6 | default | **+10 mm length only** |
| GEO02c | `ainex_controls_m2_160x90_cadsole.xml` | **160 × 90** | `0.080 0.045 0.00225` | **4.5 mm** | −0.02375 | −0.026 | 1.6 | default | GEO02a planform + GEO01 vertical |
| GEO03 | `ainex_controls_m2_145_softsole.xml` | 145 × 86 | `0.0725 0.0430 0.008` | 16 mm | −0.018 | −0.026 | 1.6 | **soft** (below) | compliance only |

All rows keep **`SOLE_OFFSET = 0.026`** and sole bottom **−0.026** in the ankle-roll body frame. HX clips, PD, meshes, actuator block unchanged from locked M145.

### GEO03 soft-contact compliance

Foot contact geoms only (`l_foot_contact` / `r_foot_contact`):

| Param | Locked / default | GEO03 softsole |
|-------|------------------|----------------|
| `solref` | ~`0.02 1` (MuJoCo default) | **`0.05 0.8`** — larger timeconst → softer spring; dampratio 0.8 |
| `solimp` | ~`0.9 0.95 0.001 0.5 2` | **`0.85 0.90 0.01 0.5 2`** — lower impedance band / more penetration allowance |

Intent: tread-like soft box without changing planform or sole bottom. Floor geom unchanged. Still a box proxy (not a meshed tread STL).

## How Controls loads

```bash
MUJOCO_GL=egl .venv/bin/python scripts/walk_gait_ainex.py \
  --model mujoco/ainex_hiwonder/ainex_controls_m2_160x90.xml \
  --duration 9 --tag GEO02a_T75 ...
```

Swap the `--model` path for any row above. Geom names stay `l_foot_contact` / `r_foot_contact` (no script rename). Prefer calling `walk_gait_ainex.py` directly; `ss_step_ainex.py` may still hardcode locked M145 unless Controls adds a flag.

## Non-claims / score plant

- **Do not claim Gate E** from shipping these XMLs.
- **Locked score plant** remains `ainex_controls_m2_145.xml` (GEO00) until AI cospec locks a winning honesty plant.
- AUTH envelope is HARD-FALSIFIED; this pack is the next lever (plant/contact honesty), still sim.
- No Path A / buy / spend language. Mesh-derived / not OEM STEP.

## Related

- Cospec intent: `docs/CONTACT_GEOMETRY_HONESTY_COSPEC.md`
- Cadsole residual note: `docs/PLANT_CADSOLE_AB.md`
- Variant index: `mujoco/ainex_hiwonder/FOOT_CONTACT_VARIANTS.md`
- Hardware Gate E support note: `docs/GATE_E_HARDWARE_SUPPORT.md`
