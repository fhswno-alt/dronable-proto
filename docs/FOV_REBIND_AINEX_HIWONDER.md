# FOV re-bind — kit-matched AiNex MuJoCo (mesh-derived)

**When:** Sun 27 Sep 2026 ~20:06 Europe/London (BST)  
**Model:** `mujoco/ainex_hiwonder/ainex_controls.xml` (prefer over bare `ainex.xml`)  
**Source:** Hiwonder URDF + 25 STL — **NOT OEM STEP**  
**Path A:** frozen / not approved — geometry check only, **not a spend signal**

## Measured stand geometry (feet grounded)

| Item | Value |
|------|-------|
| Total mass | **2.35 kg** (kit claim 2.45 kg Standard) |
| Height span (geom) | **~388 mm** (kit claim 415 mm — mesh envelope short of brochure; label mesh-derived) |
| `head_tilt_link` origin Z | **0.365 m** |
| Head mesh geom center Z | **≈0.388 m** |
| **Working cam Z (optical estimate)** | **≈0.380 m** (between tilt origin and head geom top) |
| Lever Z | **0.275 m** (unchanged product band 0.25–0.30 m) |
| Pan-tilt joints | **`head_pan`** + **`head_tilt`** (kit names — **not** old `neck_pitch`) |
| Kit HFOV claim | **120°** (no cam site in MJCF yet — geometric only) |

## Depression to center lever on optical axis

| Cam→door dist | Depression @ cam Z 0.380 m | Prior placeholder (cam Z 0.385) |
|---------------|----------------------------|--------------------------------|
| 0.50 m | **~12°** | 12.4° |
| 0.40 m | **~15°** | 15.4° |
| 0.30 m | **~19°** | 20.1° |

## Re-bind recommendation (AI)

1. Keep default approach look-down **≈ −15° to −16°** at 0.4 m; deepen to **≈ −19°/−20°** near 0.3 m grasp.
2. Command **`head_tilt`** (Controls: confirm sign that depresses optical axis toward lever — axis is `(0,−1,0)` in this MJCF).
3. **No fab shim.** Still prefer stock 2DOF head.
4. Prior run `2fe04e48e07d` numbers stay directionally valid; deltas are **<2°** vs kit-matched mesh — do **not** treat as Path A unlock.
5. When Hardware adds `kit_cam` site / camera to `ainex_controls.xml`, re-measure optical Z and replace this estimate.

## Honest limits

- Mesh-derived, not OEM STEP — cam Z ±15 mm uncertainty remains.
- Geom height ~388 mm vs brochure 415 mm — do not RFQ fab to brochure until STEP or physical measure.
- Walk locomotion still **FAIL** per Controls (`a012325db637`) — FOV re-bind ≠ clean walk / ≠ spend green light.
