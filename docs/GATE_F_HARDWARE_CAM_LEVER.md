# Gate F Hardware — kit_cam + door/lever prop (sim)

**When:** Mon 28 Sep 2026 ~01:05 Europe/London (BST)  
**Owner:** Founding Hardware Engineer  
**Scope:** Additive MJCF for Gate F lever approach. **No Gate E plant hop.**  
**Labels:** mesh-derived kit; sim-only; no spend.

## Files

| Role | Path |
|------|------|
| Locked score plant (unchanged) | `mujoco/ainex_hiwonder/ainex_controls_m2_145.xml` |
| **Gate F companion (load this)** | `mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml` |
| Criteria | `docs/GATE_F_AI_CRITERIA.md` |
| FOV rebind | `docs/FOV_REBIND_AINEX_HIWONDER.md` |

Companion = locked M145 + additive world `door_prop` + `kit_cam`/`kit_cam_site` on `head_tilt_link`.  
**145×86 contact boxes, solref/solimp, friction, actuator forcerange/HX clips, mesh paths: untouched** (diff vs M145 is header + materials + door body + cam/site only).

## Cam site local pose

Parent: **`head_tilt_link`** (kit look-down; joint `head_tilt`, axis `(0,−1,0)` — **negative** depresses optical axis).

| Item | Value |
|------|-------|
| `kit_cam_site` / `kit_cam` local pos | **`0.020 0.019 0.007`** m |
| Camera `xyaxes` | `0 -1 0 0 0 1` → MuJoCo −Z looks **+X** (toes / forward) |
| `fovy` | **104.82°** (kit HFOV claim 120° @ 640×480) |

### Optical Z (measured, kinematic foot-grounded)

Free-joint body lowered so foot-contact bottom = 0 (qpos0 joint angles, no settle):

| Item | Value |
|------|-------|
| **`kit_cam_site` world Z** | **≈ 0.380 m** (0.3804 m this plant) |
| FOV rebind working estimate | ≈ 0.380 m — matched |
| Lever geom center Z | **0.275 m** AFF (`door_lever`) |
| Cam→lever @ origin stand | ≈ **0.39 m**; geom depression ≈ **15.7°** |
| `head_tilt = −16°` | lever ≈ **1°** off optical axis (FOV bind OK) |

qpos0 floating (body Z=0.268, feet above floor): site Z ≈ 0.411 m — use grounded / stand scripts for Gate F rows.

**Caveat:** optical Z is mesh-derived estimate (±15 mm), not OEM STEP / not physical measure. URDF `camera_link` is fixed on `body_link` (does not tilt) — **not** used; Gate F requires cam on `head_tilt_link`.

## Lever prop

```
door_prop @ (0.45, 0, 0)   # ahead of +X-facing kit
door_lever size 0.06×0.01×0.008 @ local (−0.05, 0, 0.275) → world (0.40, 0, 0.275)
```

Gate F ship: prop geoms were visual/marker (`contype="0"`). **Update (Gate G):** `door_lever` is now contactable on the companion; `door_panel` stays visual-only.

**Gate G03:** companion now has a **contactable** `door_lever` (bit 2) + hand contact sites/geoms — see `docs/GATE_G_HARDWARE_LEVER_CONTACT.md`. Locked M145 still untouched.

## Gate E plant contact untouched

Do **not** score Gate E / walk residual on the Gate F companion unless Controls explicitly A/B’s it. Locked walk/score plant remains `ainex_controls_m2_145.xml`. GEO honesty / STEP / sole thickness: out of scope for F.

## Controls load path (F00–F02)

```text
model = mujoco/ainex_hiwonder/ainex_controls_m2_145_gate_f.xml
```

- Command **`head_tilt`** (not old `neck_pitch`) ≈ **−16°** at 0.4 m; optional ≈ **−19°/−20°** at 0.3 m grasp row.
- Render / ego: camera name **`kit_cam`**; site **`kit_cam_site`**.
- Vision stays **out** of Gate E walk residual / PPO obs.
- Prove tags F00 / F01 / F02 per `docs/GATE_F_AI_CRITERIA.md`.

## Honesty

- Sim-only prop + FOV estimate cam; no spend / cart / Path A unlock.
- Old `dronable_v0` kit_cam used `neck_pitch` and +Y forward — superseded for AiNex Controls by this companion.


## Demo-prop STEP/STL stub (Manufacturing QC)

Sim-aligned solids for fab QC — **not OEM door hardware**; mesh-derived sim gauge.

| Item | Value |
|------|-------|
| Lever AFF | **275 mm** (band 250–300) |
| Lever bar L×W×T | **60 × 10 × 8 mm** |
| Panel T×W×H | **8 × 100 × 340 mm** (bottom on Z=0) |
| STEP / STL | `cad/gate_f_lever/demo_prop_lever_275AFF_meshDerived_notOEM.{step,stl}` |
| QC pointer | `docs/GATE_F_LEVER_STEP_STUB.md` · checklist `docs/GATE_F_LEVER_FAB_PREP.md` |

Foot STEP pack remains frozen at **145×86**. No change to `ainex_controls_m2_145.xml` contact/HX.
