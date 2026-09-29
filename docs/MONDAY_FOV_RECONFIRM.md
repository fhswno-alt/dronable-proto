# Monday FOV reconfirm — AiNex Hiwonder (sim honesty)

**When:** Mon 28 Sep 2026 ~01:32 Europe/London (BST)  
**Owner:** Founding Hardware Engineer  
**Checklist:** `docs/MONDAY_PERCEPTION_COMPUTE_CHECKLIST.md` item #1  
**Scope:** Confirm FOV / look-down still bound to AiNex Hiwonder mesh on **locked** M145 + Gate F companion.  
**Labels:** mesh-derived kit; sim-only; **no spend**; no STEP; no contact/HX plant hop; GEO honesty not reopened.

## Plants

| Role | Path | Notes |
|------|------|-------|
| Locked score (Gate E) | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` | **Not modified** this pass (sha16 `4f30b6e53578789f`; mtime 27 Sep 22:22 BST) |
| Gate F companion (measured) | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` | Additive only: `door_prop`/`door_lever` + `kit_cam`/`kit_cam_site` on `head_tilt_link` |

Diff vs M145 = header + materials + door body + cam/site only. Foot contact boxes (145×86, half `0.0725 0.0430 0.008`), friction, actuator `forcerange`/HX clips: **identical**.

## Method

Kinematic **foot-grounded** stand on gate_f plant:

- qpos0 joint angles (no settle / no dynamics)
- Free-joint body Z lowered so min foot-contact geom bottom = 0
- Body X placed so `kit_cam_site`→`door_lever` **horizontal = 0.400 m**
- Optical axis from `kit_cam` (−Z look / +X kit forward); depression via `head_tilt` axis `(0,−1,0)` (negative depresses)

## Measured numbers

| Item | Measured | Claim (FOV_REBIND / GATE_F_HARDWARE) | Tol | Result |
|------|----------|--------------------------------------|-----|--------|
| `kit_cam_site` world Z (grounded, tilt=0) | **0.3804 m** | ≈0.380 m (HW 0.3804) | ±5 mm | **PASS** |
| Floating site Z (body Z=0.268) | 0.4110 m | ≈0.411 m | — | match |
| Lever geom Z (`door_lever`) | **0.275 m** | 0.275 m | exact | **PASS** |
| Geom depression @ 0.40 m horiz | **14.8°** | ~15° @ 0.4 m (FOV table) | ±2.5° | **PASS** |
| Centering `head_tilt` (elev≈0 @ 0.4 m) | **−14.1°** | ≈ −15° to −16° working | ±2.5° | **PASS** |
| `head_tilt = −16°` @ 0.4 m | elev **+2.0°** off axis | HW ≈1°; bind OK | ≤3° off | **PASS** |
| Cam / site parent | **`head_tilt_link`** | head_tilt_link | — | **PASS** |
| Joints | **`head_pan` + `head_tilt`** | kit names | — | **PASS** |
| `neck_pitch` present? | **No** (neither plant) | must not use | — | **PASS** |
| `kit_cam` `fovy` | **104.82°** | 104.82 | exact | **PASS** |
| HFOV @ 640×480 from fovy | **120.00°** | kit claim 120° | ±0.5° | **PASS** |
| Gate E plant edited? | **No** | must stay locked | — | **PASS** |

**Working claim hold:** optical Z ≈0.380 m; approach look-down **`head_tilt ≈ −16°` @ 0.4 m** to lever 0.275 m; HFOV ~120° / fovy 104.82; cam parent = `head_tilt_link`.

### Caveat (stated, not a fail)

- At **exact** 0.40 m horiz, pure geom depression is **14.8°** and optical centering tilt is **−14.1°** (cam drops slightly under tilt). Commanded **−16°** leaves lever ≈**2°** above the optical axis — within FOV re-bind “&lt;2° delta / still prefer −15° to −16°” band and Gate F F00 lock (−16°).
- GATE_F_HARDWARE “≈15.7° @ origin stand” used ~0.375–0.39 m cam→lever (body near door), not exact 0.40 m — consistent with atan2(0.105, 0.375)≈15.7°.
- Optical Z remains **mesh-derived** (±15 mm vs physical / OEM STEP). Not a Path A / spend signal.

## Ego previews (optional)

| Shot | Path |
|------|------|
| pitch 0 @ 0.4 m (lever low) | `previews/ego_cam/monday_reconfirm/01_pitch0_dy0p40.png` |
| head_tilt −16° @ 0.4 m (lever near center) | `previews/ego_cam/monday_reconfirm/02_pitch-16_dy0p40.png` |

## Verdict

**PASS** vs `docs/FOV_REBIND_AINEX_HIWONDER.md` + `docs/GATE_F_HARDWARE_CAM_LEVER.md` within stated tolerances (±5 mm optical Z; ±2.5° depression / ≤3° off-axis at −16°).

Gate F remains **LOCKED TRUE** on companion plant; Gate E locked M145 untouched; vision off walk; **no spend**.

## Refs

- `docs/FOV_REBIND_AINEX_HIWONDER.md`
- `docs/GATE_F_HARDWARE_CAM_LEVER.md`
- `docs/GATE_F_AI_CRITERIA.md`
- `EGO_CAM_REPORT.md` (legacy `neck_pitch` / v0 — superseded for AiNex Controls by `head_tilt` + gate_f)
- `previews/ainex_walk/iterate/GATE_F_AI_LOCK.md`
