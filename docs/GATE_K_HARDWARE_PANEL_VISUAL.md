# Gate K — panel open-angle visual honesty (Hardware)

**Status:** VISUAL_HOLD helper. Companion plant unchanged. Soft-pass forbidden. No M145 edit. No spend.

**Plant:** `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml`  
**Geom:** `door_panel` box half-extents `0.01 × 0.05 × 0.17` (20 mm thick × 100 mm wide × 340 mm tall), hinge about **Z** at the −Y edge.

## Dig result (qpos vs silhouette)

Measured with MuJoCo forward kinematics on the companion:

| Panel hinge | geom xpos (m) | Face-on Y span (m) | Free-edge X travel vs 0° |
|-------------|---------------|--------------------|---------------------------|
| 0° | (0.45, 0, 0.17) | 0.100 | — |
| +30° | (~0.425, −0.007, 0.17) | **~0.097** | **~0.05 m** |

Face-on projected width at +30° is only ~3% smaller than closed. From a robot / +X-looking / face-on review cam, a thin slab at ±30° **looks almost closed** even when `qpos` is honest +30°. That matches the open-angle watch FAIL vs HUD while §9 world feet can still PASS.

**Not a pin, not a detached panel, not a wrong joint axis.** Coupling (lever child of `door_panel_link`) and hinge limits (±30°) are intact. The failure mode is **aspect-ratio / camera framing**, not controller soft-pass.

## What Hardware will / will not do

| Do | Don't |
|----|--------|
| Keep this note as the geom dig answer | Soften springs or widen Alt A unless AI/Controls ask |
| Recommend **oblique / panel-edge** review cam (or thicker edge stripe geom) for open-angle watches | Edit locked M145 walk plant |
| Optional additive companion: high-contrast edge marker / slightly thicker visual box (visual-only) if Controls wants a clearer tell | Claim full door open / latch / walk-through |

## Recommendation

Keep Gate K **VISUAL_HOLD** until open-angle evidence uses a cam that shows the free-edge swing (or a visual tell geom). Do **not** lock on metrics + face-on dual alone. Soft-pass stays forbidden.
